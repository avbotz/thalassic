import math
import os
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.task import Future
from sub_control_interfaces.msg import ControllerStatus, MotionSetpoint
from sub_pid_tuner.ros_adapter import (
    SERVICE_TIMEOUT,
    STREAM_TIMEOUT,
    RosAdapter,
    coerce_parameter,
    parameter_value,
)
from sub_pid_tuner.routines import Routine

KEEP, POSITION, VELOCITY, HOLD = (
    MotionSetpoint.KEEP,
    MotionSetpoint.POSITION,
    MotionSetpoint.VELOCITY,
    MotionSetpoint.HOLD,
)


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def adapter():
    # Away from any stack running on this machine.
    rclpy.init(domain_id=50 + os.getpid() % 50)
    node = RosAdapter(namespace=f"/dashboard_test_{os.getpid()}")
    node._publisher = Recorder()
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def controller(adapter):
    """A node where the adapter looks for sub_control."""
    node = rclpy.create_node("sub_control", namespace=adapter.get_namespace())
    deadline = time.monotonic() + 5.0
    while ("sub_control", adapter.get_namespace()) not in adapter.get_node_names_and_namespaces():
        assert time.monotonic() < deadline, "sub_control never appeared in the graph"
        time.sleep(0.01)
    yield node
    node.destroy_node()


class SilentClient:
    """A parameter client whose node died: its requests are never answered."""

    def __init__(self):
        self.requests = 0

    def services_are_ready(self):
        return True

    def list_parameters(self, prefixes, depth):
        self.requests += 1
        return Future()


def odometry(x=0.0, y=0.0, z=-1.5, yaw=0.0, body_vx=0.0, yaw_rate=0.0) -> Odometry:
    msg = Odometry()
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = x, y, z
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    msg.twist.twist.linear.x = body_vx
    msg.twist.twist.angular.z = yaw_rate
    return msg


def status(
    killed=False, target=(0.0, 0.0, -1.5, 0.0, 0.0, 0.0), reference=None
) -> ControllerStatus:
    msg = ControllerStatus()
    msg.mode = [POSITION] * 6
    msg.target = list(target)
    msg.reference_position = list(reference or target)
    msg.killed = killed
    msg.dvl_ok = msg.depth_ok = True
    msg.axis_controllable = [True] * 6
    return msg


def live(node, **kwargs):
    node._odometry_callback(odometry(**kwargs))
    node._status_callback(status())


def back_date(node, seconds):
    node._run["started"] = node.get_clock().now() - Duration(seconds=seconds)


def test_parameter_coercion():
    assert coerce_parameter("2.5", ParameterType.PARAMETER_DOUBLE) == 2.5
    assert coerce_parameter([1, "2", 3.5], ParameterType.PARAMETER_DOUBLE_ARRAY) == [1.0, 2.0, 3.5]
    assert coerce_parameter("false", ParameterType.PARAMETER_BOOL) is False
    assert coerce_parameter([3], ParameterType.PARAMETER_INTEGER_ARRAY) == [3]
    with pytest.raises(ValueError):
        coerce_parameter(float("nan"), ParameterType.PARAMETER_DOUBLE)
    with pytest.raises(ValueError):
        coerce_parameter([1.0, float("inf")], ParameterType.PARAMETER_DOUBLE_ARRAY)


def test_integers_are_not_truncated_and_bytes_round_trip():
    assert coerce_parameter(3.0, ParameterType.PARAMETER_INTEGER) == 3
    with pytest.raises(ValueError, match="not an integer"):
        coerce_parameter(2.5, ParameterType.PARAMETER_INTEGER)
    with pytest.raises(ValueError, match="not an integer"):
        coerce_parameter([1, 2.5], ParameterType.PARAMETER_INTEGER_ARRAY)
    # rclpy holds byte arrays as one-byte bytes objects; the browser gets numbers.
    data = coerce_parameter([1, 255], ParameterType.PARAMETER_BYTE_ARRAY)
    parameter = Parameter("data", Parameter.Type.BYTE_ARRAY, data)
    assert parameter_value(parameter.to_parameter_msg().value) == [1, 255]


def test_parameters_are_read_with_one_client_per_node(adapter, controller):
    executor = SingleThreadedExecutor()
    executor.add_node(adapter)
    executor.add_node(controller)

    def read():
        adapter._remote_parameters.clear()
        deadline = time.monotonic() + 5.0
        while adapter.controller_fqn not in adapter._remote_parameters:
            assert time.monotonic() < deadline, "the controller's parameters were never read"
            adapter.refresh_parameters()
            executor.spin_once(timeout_sec=0.05)

    try:
        read()
        assert adapter._remote_parameters[adapter.controller_fqn]["use_sim_time"]["value"] is False
        clients = len(list(adapter.clients))
        read()
        assert len(list(adapter.clients)) == clients
    finally:
        executor.shutdown()


def test_an_unanswered_parameter_request_is_retried(adapter, controller):
    client = adapter._parameter_clients[adapter.controller_fqn] = SilentClient()
    adapter.refresh_parameters()
    adapter.refresh_parameters()
    assert client.requests == 1  # still waiting on the first
    adapter._pending_nodes[adapter.controller_fqn] -= SERVICE_TIMEOUT
    adapter.refresh_parameters()
    assert client.requests == 2


def test_controller_is_found_in_the_namespace(adapter):
    assert adapter.controller_fqn == f"/dashboard_test_{os.getpid()}/sub_control"


def test_telemetry_is_in_the_frames_of_controller_status(adapter):
    # Facing odom +y (yaw 90 degrees, counter-clockwise), surging at 0.4 m/s.
    adapter._odometry_callback(odometry(x=1.0, z=-2.0, yaw=math.pi / 2, body_vx=0.4, yaw_rate=0.1))
    adapter._status_callback(status(reference=(1.1, 0.0, -2.0, 0.0, 0.0, -math.pi + 0.05)))
    signals = adapter.snapshot()["signals"]
    assert signals["x.velocity"] == pytest.approx(0.0, abs=1e-9)
    assert signals["y.velocity"] == pytest.approx(0.4)
    assert signals["yaw.velocity"] == pytest.approx(0.1)
    assert signals["z.measured"] == pytest.approx(-2.0)
    assert signals["yaw.measured"] == pytest.approx(math.pi / 2)
    assert signals["x.tracking_error"] == pytest.approx(0.1)
    # Reference minus measured, the short way round.
    assert signals["yaw.tracking_error"] == pytest.approx(math.pi / 2 + 0.05)


def test_a_routine_needs_live_telemetry_and_an_unkilled_vehicle(adapter):
    with pytest.raises(RuntimeError, match="No live"):
        adapter.start_routine("step", "z", 0.5, 4.0, 0.0, 1)
    adapter._odometry_callback(odometry())
    adapter._status_callback(status(killed=True))
    with pytest.raises(RuntimeError, match="Killed"):
        adapter.start_routine("step", "z", 0.5, 4.0, 0.0, 1)
    assert adapter._publisher.messages == []


def test_a_routine_refuses_to_share_the_vehicle(adapter):
    live(adapter)
    adapter._count_publishers = lambda: None
    adapter._publisher_counts = {"motion_setpoint": 2}
    with pytest.raises(RuntimeError, match="motion_setpoint"):
        adapter.start_routine("hold", "z", 0.0, 10.0, 0.0, 1)


def test_step_holds_everything_then_steps_from_where_the_controller_holds(adapter):
    live(adapter, x=0.02, z=-1.48)  # a little off the target it is holding
    started = adapter.start_routine("step", "z", 0.5, 4.0, 0.0, 1)
    assert started["active"] is True and started["duration"] == pytest.approx(16.0)

    hold, first = adapter._publisher.messages
    assert list(hold.mode) == [POSITION] * 6
    assert list(hold.position) == pytest.approx([0.0, 0.0, -1.5, 0.0, 0.0, 0.0])
    assert list(first.mode) == [KEEP, KEEP, POSITION, KEEP, KEEP, KEEP]
    assert first.position[2] == pytest.approx(-1.0)
    assert first.timeout == 0.0

    # Same segment: nothing new is sent.
    adapter._routine_tick()
    assert len(adapter._publisher.messages) == 2
    back_date(adapter, 9.0)
    adapter._routine_tick()
    assert adapter._publisher.messages[-1].position[2] == pytest.approx(-2.0)

    # The last segment already brought it back: completing sends nothing more.
    back_date(adapter, 16.5)
    adapter._routine_tick()
    assert len(adapter._publisher.messages) == 3
    assert adapter.routine_snapshot()["status"] == "complete"


def test_cruise_streams_velocity_with_a_timeout(adapter):
    live(adapter)
    adapter.start_routine("cruise", "x", 0.2, 5.0, 3.0, 1)
    adapter._routine_tick()
    streamed = adapter._publisher.messages[1:]
    assert len(streamed) == 2
    for msg in streamed:
        assert list(msg.mode[:2]) == [VELOCITY, VELOCITY]
        assert list(msg.velocity[:2]) == pytest.approx([0.2, 0.0])
        assert msg.timeout == STREAM_TIMEOUT
        assert msg.velocity_frame == MotionSetpoint.WORLD
    back_date(adapter, 6.0)
    adapter._routine_tick()
    assert list(adapter._publisher.messages[-1].mode[:2]) == [HOLD, HOLD]


def test_stop_comes_to_a_smooth_hold(adapter):
    live(adapter)
    adapter.start_routine("step", "x", 1.0, 4.0, 0.0, 1)
    adapter.stop_routine()
    stop = adapter._publisher.messages[-1]
    assert list(stop.mode) == [HOLD, HOLD, HOLD, POSITION, POSITION, HOLD]
    assert adapter.routine_snapshot()["status"] == "stopped"


def test_no_routine_target_follows_a_stop(adapter, monkeypatch):
    live(adapter)
    adapter.start_routine("step", "x", 1.0, 4.0, 0.0, 1)
    back_date(adapter, 5.0)  # the second segment is due
    segment_at = Routine.segment_at

    def stopped_meanwhile(routine, elapsed):
        # The operator's Stop lands while this tick works out its segment.
        adapter.stop_routine()
        return segment_at(routine, elapsed)

    monkeypatch.setattr(Routine, "segment_at", stopped_meanwhile)
    adapter._routine_tick()
    stop = adapter._publisher.messages[-1]
    assert list(stop.mode) == [HOLD, HOLD, HOLD, POSITION, POSITION, HOLD]


def test_kill_aborts_without_sending_anything(adapter):
    live(adapter)
    adapter.start_routine("hold", "z", 0.0, 10.0, 0.0, 1)
    sent = len(adapter._publisher.messages)
    adapter._status_callback(status(killed=True))
    assert adapter.routine_snapshot()["status"] == "aborted"
    assert len(adapter._publisher.messages) == sent


def test_stale_telemetry_aborts_and_holds(adapter):
    live(adapter)
    adapter.start_routine("hold", "z", 0.0, 10.0, 0.0, 1)
    adapter._odometry_time -= 5.0
    adapter._routine_tick()
    assert adapter.routine_snapshot()["status"] == "aborted"
    assert list(adapter._publisher.messages[-1].mode) == [
        HOLD,
        HOLD,
        HOLD,
        POSITION,
        POSITION,
        HOLD,
    ]


def test_manual_commands_are_checked(adapter):
    sent = adapter.publish_command(
        ["keep", "keep", "position", "keep", "keep", "velocity"],
        [0, 0, -1.2, 0, 0, 0.2],
        timeout=2.0,
    )
    msg = adapter._publisher.messages[-1]
    assert list(msg.mode) == [KEEP, KEEP, POSITION, KEEP, KEEP, VELOCITY]
    assert msg.position[2] == pytest.approx(-1.2) and msg.velocity[5] == pytest.approx(0.2)
    assert sent["timeout"] == 2.0 and sent["frame"] == "heading"

    with pytest.raises(ValueError, match="share a mode"):
        adapter.publish_command(
            ["velocity", "keep", "keep", "keep", "keep", "keep"], [0.1] + [0] * 5
        )
    with pytest.raises(ValueError, match="timeout"):
        adapter.publish_command(["keep"] * 5 + ["effort"], [0] * 5 + [2.0], timeout=0.0)

    # KEEP and HOLD take no value: whatever the form had there is not read.
    sent = adapter.publish_command(
        ["keep", "keep", "position", "hold", "hold", "hold"], ["", None, -1.2, "x", 0, None]
    )
    assert sent["values"] == [None, None, -1.2, None, None, None]
    assert adapter._publisher.messages[-1].position[2] == pytest.approx(-1.2)

    live(adapter)
    adapter.start_routine("hold", "z", 0.0, 10.0, 0.0, 1)
    with pytest.raises(RuntimeError, match="stop the routine"):
        adapter.publish_command(["hold"] * 6, [0] * 6)
