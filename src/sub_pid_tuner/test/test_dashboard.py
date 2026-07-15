import asyncio
from concurrent.futures import Future
import time

import pytest
import rclpy
from aiohttp import web
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import ParameterType
from sub_control_interfaces.msg import Error, Setpoint

from sub_pid_tuner.ros_adapter import RosAdapter, coerce_parameter
from sub_pid_tuner.server import BROADCAST_HZ, DashboardServer
from sub_pid_tuner.tracking_profile import TrackingProfile


class FakeRos:
    def __init__(self):
        self.values = {}

    def set_parameters(self, node, changes):
        future = Future()
        applied = []
        for change in changes:
            name = change["name"]
            value = change["value"]
            self.values[(node, name)] = value
            applied.append({"node": node, "name": name, "value": value})
        future.set_result(applied)
        return future

    def publish_setpoint(self, mode, values, altitude=False):
        return {"mode": mode, "values": values, "altitude": altitude}

    def start_tracking(
        self, experiment, mode, axis, amplitude, ramp_time, hold_time, cycles,
        max_tracking_error=None,
    ):
        self.tracking = {
            "active": True,
            "experiment": experiment,
            "mode": mode,
            "axis": axis,
            "amplitude": amplitude,
            "ramp_time": ramp_time,
            "hold_time": hold_time,
            "cycles": cycles,
            "max_tracking_error": max_tracking_error,
        }
        return self.tracking

    def stop_tracking(self):
        self.tracking = {"active": False, "status": "stopped"}
        return self.tracking

    def advance_tracking(self):
        self.tracking["waypoint_index"] = self.tracking.get("waypoint_index", 0) + 1
        return self.tracking

    def start_characterization(self, axis, amplitude, slow_slope, fast_slope, dwell):
        self.characterization = {
            "active": True,
            "axis": axis,
            "amplitude": amplitude,
            "slow_slope": slow_slope,
            "fast_slope": fast_slope,
            "dwell": dwell,
        }
        return self.characterization

    def stop_characterization(self):
        self.characterization = {"active": False, "status": "stopped"}
        return self.characterization

    def stop_all(self):
        return {
            "tracking": self.stop_tracking(),
            "characterization": self.stop_characterization(),
            "holding": "pose",
            "message": "All dashboard routines stopped; holding measured pose",
        }

    def characterization_changes(self):
        return [
            {"node": "/sub", "name": "model.linear_drag", "value": [12.0] * 6}
        ]

    def mark_characterization_applied(self):
        self.characterization = {"active": False, "status": "complete", "applied": True}

    def characterization_snapshot(self):
        return self.characterization

    def refresh_parameters(self):
        pass

    def snapshot(self, include_parameters=True):
        result = {"robot_name": "test", "signals": {}, "telemetry_age": None, "killed": None}
        if include_parameters:
            result["parameters"] = {}
        return result


def test_parameter_coercion():
    assert coerce_parameter("2.5", ParameterType.PARAMETER_DOUBLE) == 2.5
    assert coerce_parameter([1, "2", 3.5], ParameterType.PARAMETER_DOUBLE_ARRAY) == [1.0, 2.0, 3.5]
    assert coerce_parameter("false", ParameterType.PARAMETER_BOOL) is False
    with pytest.raises(ValueError):
        coerce_parameter(float("nan"), ParameterType.PARAMETER_DOUBLE)


def test_two_clients_can_write_without_a_control_lease(tmp_path):
    ros = FakeRos()
    server = DashboardServer(ros, tmp_path)

    async def run():
        first = await server.handle(
            {"type": "set_parameters", "request_id": "a", "changes": [{"node": "/sub", "name": "kp", "value": 1.0}]}
        )
        second = await server.handle(
            {"type": "set_parameters", "request_id": "b", "changes": [{"node": "/sub", "name": "kp", "value": 2.0}]}
        )
        return first, second

    first, second = asyncio.run(run())
    assert first["applied"][0]["value"] == 1.0
    assert second["applied"][0]["value"] == 2.0
    assert ros.values[("/sub", "kp")] == 2.0


def test_setpoint_request():
    server = DashboardServer(FakeRos(), None)
    result = asyncio.run(
        server.handle(
            {"type": "publish_setpoint", "mode": "position", "values": [1.0, 2.0, -1.0], "altitude": False}
        )
    )
    assert result["values"] == [1.0, 2.0, -1.0]


def test_stop_all_request_stops_both_dashboard_routines():
    ros = FakeRos()
    ros.tracking = {"active": True}
    ros.characterization = {"active": True}
    result = asyncio.run(
        DashboardServer(ros, None).handle(
            {"type": "stop_all", "request_id": "stop-now"}
        )
    )
    assert result["type"] == "stop_all_result"
    assert result["tracking"] == {"active": False, "status": "stopped"}
    assert result["characterization"] == {"active": False, "status": "stopped"}
    assert result["holding"] == "pose"


def test_minimum_jerk_profile_moves_holds_and_returns_smoothly():
    profile = TrackingProfile.from_values("minimum_jerk", "position", "x", 2.0, 2.0, 1.0, 1)
    assert profile.duration == 6.0
    assert profile.value_at(0.0) == pytest.approx(0.0)
    assert profile.value_at(1.0) == pytest.approx(1.0)
    assert profile.value_at(2.5) == pytest.approx(2.0)
    assert profile.value_at(4.0) == pytest.approx(1.0)
    assert profile.value_at(5.5) == pytest.approx(0.0)
    assert profile.value_at(6.0) == pytest.approx(0.0)


def test_step_profile_settles_at_zero_between_directions():
    profile = TrackingProfile.from_values("step", "velocity", "y", 0.5, 1.0, 1.0, 1)
    assert profile.duration == 4.0
    assert profile.value_at(0.5) == pytest.approx(0.5)
    assert profile.value_at(1.5) == pytest.approx(0.0)
    assert profile.value_at(2.5) == pytest.approx(-0.5)
    assert profile.value_at(3.5) == pytest.approx(0.0)


def test_trapezoid_profile_ramps_cruises_and_settles_before_reversing():
    profile = TrackingProfile.from_values("trapezoid", "velocity", "x", 0.4, 2.0, 1.0, 1)
    assert profile.duration == 12.0
    assert profile.value_at(0.0) == pytest.approx(0.0)
    assert profile.value_at(1.0) == pytest.approx(0.2)
    assert profile.value_at(2.5) == pytest.approx(0.4)
    assert profile.value_at(4.0) == pytest.approx(0.2)
    assert profile.value_at(5.5) == pytest.approx(0.0)
    assert profile.value_at(7.0) == pytest.approx(-0.2)
    assert profile.value_at(8.5) == pytest.approx(-0.4)
    assert profile.value_at(10.0) == pytest.approx(-0.2)
    assert profile.value_at(11.5) == pytest.approx(0.0)
    assert profile.public_state()["ramp_slope"] == pytest.approx(0.2)


def test_trapezoid_profile_is_only_available_for_inner_loops():
    with pytest.raises(ValueError, match="velocity and angular-rate"):
        TrackingProfile.from_values("trapezoid", "position", "x", 0.4, 2.0, 1.0, 1)


def test_follower_pid_profile_is_a_bounded_closed_planar_path():
    profile = TrackingProfile.from_values("follower_pid", "position", "xy", 0.5, 8.0, 2.0, 1)
    preview = profile.preview()
    assert preview[0] == pytest.approx([0.0, 0.0, 0.0])
    assert preview[-1] == pytest.approx([0.0, 0.0, 0.0])
    assert max(abs(point[0]) for point in preview) <= 0.5
    assert max(abs(point[1]) for point in preview) <= 0.65
    assert all(point[2] == pytest.approx(0.0) for point in preview)
    assert profile.vector_at(profile.ramp_time + 0.5) == pytest.approx((0.0, 0.0, 0.0))
    assert profile.public_state()["path_length"] > 2.5
    assert profile.public_state()["nominal_speed"] == pytest.approx(
        profile.path_length / profile.ramp_time
    )


def test_square_test_has_four_straight_sides_and_returns_home():
    profile = TrackingProfile.from_values("square_test", "position", "xy", 0.4, 8.0, 1.0, 1)
    assert profile.vector_at(0.0) == pytest.approx((0.0, 0.0, 0.0))
    assert profile.vector_at(1.8) == pytest.approx((0.4, 0.0, 0.0))
    assert profile.vector_at(2.0) == pytest.approx((0.4, 0.0, 0.0))
    assert profile.vector_at(4.0) == pytest.approx((0.4, 0.4, 0.0))
    assert profile.vector_at(6.0) == pytest.approx((0.0, 0.4, 0.0))
    assert profile.vector_at(8.0) == pytest.approx((0.0, 0.0, 0.0))
    assert profile.vector_at(8.5) == pytest.approx((0.0, 0.0, 0.0))


def test_spline_test_returns_home_and_respects_selected_plane():
    profile = TrackingProfile.from_values("spline_test", "position", "xz", 0.6, 8.0, 2.0, 1)
    preview = profile.preview()
    assert preview[0] == pytest.approx([0.0, 0.0, 0.0])
    assert preview[-1] == pytest.approx([0.0, 0.0, 0.0])
    assert all(point[1] == pytest.approx(0.0) for point in preview)
    assert any(abs(point[2]) > 0.05 for point in preview)


def test_path_op_modes_require_position_and_a_valid_plane():
    with pytest.raises(ValueError, match="position controller"):
        TrackingProfile.from_values("follower_pid", "velocity", "xy", 0.4, 4.0, 1.0, 1)
    with pytest.raises(ValueError, match="path plane"):
        TrackingProfile.from_values("follower_pid", "position", "xyz", 0.4, 4.0, 1.0, 1)
    with pytest.raises(ValueError, match="path plane"):
        TrackingProfile.from_values("spline_test", "position", "x", 0.4, 4.0, 1.0, 1)
    with pytest.raises(ValueError, match="between 4 and 180"):
        TrackingProfile.from_values("follower_pid", "position", "xy", 0.4, 181.0, 1.0, 1)


def test_windowed_sine_starts_and_ends_at_zero():
    profile = TrackingProfile.from_values("sine", "angular_velocity", "z", 0.5, 4.0, 1.0, 3)
    assert profile.duration == 13.0
    assert profile.value_at(0.0) == pytest.approx(0.0)
    assert abs(profile.value_at(1.0)) <= 0.5
    assert profile.value_at(12.0) == pytest.approx(0.0)
    assert profile.value_at(12.5) == pytest.approx(0.0)


def test_tracking_profile_enforces_pool_safety_bounds():
    with pytest.raises(ValueError, match="at most"):
        TrackingProfile.from_values("minimum_jerk", "attitude", "z", 1.0, 2.0, 1.0, 1)
    with pytest.raises(ValueError, match="ramp time"):
        TrackingProfile.from_values("minimum_jerk", "position", "x", 0.5, 0.1, 1.0, 1)
    with pytest.raises(ValueError, match="whole number"):
        TrackingProfile.from_values("minimum_jerk", "position", "x", 0.5, 1.0, 1.0, 1.5)
    with pytest.raises(ValueError, match="for position and attitude"):
        TrackingProfile.from_values("minimum_jerk", "velocity", "x", 0.2, 1.0, 1.0, 1)


def test_tracking_start_and_stop_requests():
    server = DashboardServer(FakeRos(), None)
    start = asyncio.run(
        server.handle(
            {
                "type": "start_tracking",
                "experiment": "step",
                "mode": "velocity",
                "axis": "x",
                "amplitude": 0.3,
                "ramp_time": 2.0,
                "hold_time": 3.0,
                "cycles": 2,
            }
        )
    )
    assert start["tracking"]["active"] is True
    assert start["tracking"]["mode"] == "velocity"
    stop = asyncio.run(server.handle({"type": "stop_tracking"}))
    assert stop["tracking"] == {"active": False, "status": "stopped"}


def test_characterization_start_stop_and_apply_requests():
    ros = FakeRos()
    server = DashboardServer(ros, None)
    started = asyncio.run(
        server.handle(
            {
                "type": "start_characterization",
                "axis": "x",
                "amplitude": 0.12,
                "slow_slope": 0.02,
                "fast_slope": 0.08,
                "dwell": 1.5,
            }
        )
    )
    assert started["characterization"]["active"] is True
    stopped = asyncio.run(server.handle({"type": "stop_characterization"}))
    assert stopped["characterization"]["status"] == "stopped"
    applied = asyncio.run(server.handle({"type": "apply_characterization"}))
    assert applied["applied"][0]["name"] == "model.linear_drag"
    assert applied["characterization"]["applied"] is True


def test_path_tracking_request_forwards_the_lag_limit():
    ros = FakeRos()
    server = DashboardServer(ros, None)
    result = asyncio.run(
        server.handle(
            {
                "type": "start_tracking",
                "experiment": "square_test",
                "mode": "position",
                "axis": "xy",
                "amplitude": 0.5,
                "ramp_time": 60.0,
                "hold_time": 5.0,
                "cycles": 1,
                "max_tracking_error": 0.15,
            }
        )
    )
    assert result["tracking"]["max_tracking_error"] == pytest.approx(0.15)


def test_advance_tracking_request_is_forwarded():
    ros = FakeRos()
    ros.tracking = {"active": True, "waypoint_index": 0}
    result = asyncio.run(
        DashboardServer(ros, None).handle({"type": "advance_tracking"})
    )
    assert result["tracking"]["waypoint_index"] == 1


def test_dashboard_assets_are_not_cached(tmp_path):
    server = DashboardServer(FakeRos(), tmp_path)

    async def handler(_request):
        return web.Response(text="ok")

    response = asyncio.run(server.security_headers(None, handler))
    assert response.headers["Cache-Control"] == "no-store"


def test_dashboard_stream_rate_is_suitable_for_live_tuning():
    assert BROADCAST_HZ >= 30
    state = DashboardServer(FakeRos(), None).state(include_parameters=False)
    assert isinstance(state["server_time"], float)


def test_broadcast_tolerates_a_client_removing_itself():
    ros = FakeRos()
    server = DashboardServer(ros, None)

    class DisconnectingSocket:
        async def send_str(self, _message):
            server.clients.discard(self)

    async def run():
        server.clients.add(DisconnectingSocket())
        task = asyncio.create_task(server.broadcast_loop())
        await asyncio.sleep(0.05)
        assert not task.done()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(run())


def test_remote_parameter_cache_does_not_replace_rclpy_parameter_store():
    rclpy.init()
    adapter = RosAdapter("test_robot", demo=True)
    try:
        assert adapter._remote_parameters["/test_robot/sub_control"]
        assert adapter._parameters is not adapter._remote_parameters
        assert all(not isinstance(value, dict) for value in adapter._parameters.values())
    finally:
        adapter.destroy_node()
        rclpy.shutdown()


def test_pool_controller_node_is_discovered_and_editable_in_demo():
    rclpy.init()
    adapter = RosAdapter("test_robot", controller_node="sub_control_mcu", demo=True)
    try:
        assert adapter.controller_fqn == "/test_robot/sub_control_mcu"
        assert adapter.controller_fqn in adapter._watched_nodes
        assert adapter.controller_fqn in adapter._remote_parameters
        assert adapter._remote_parameters[adapter.controller_fqn]["vel_pid.z"]["read_only"] is False
    finally:
        adapter.destroy_node()
        rclpy.shutdown()


def test_ros_adapter_runs_and_completes_a_tracking_profile():
    rclpy.init()
    adapter = RosAdapter("test_robot", demo=True)
    try:
        adapter._last_odom = 0.0
        with pytest.raises(RuntimeError, match="live odometry"):
            adapter.start_tracking("minimum_jerk", "position", "x", 0.1, 0.5, 0.2, 1)
        adapter._last_odom = adapter._last_error
        started = adapter.start_tracking("minimum_jerk", "position", "x", 0.1, 0.5, 0.2, 1)
        assert started["active"] is True
        adapter._tracking_run["started_at"] -= started["duration"] + 0.1
        adapter._tracking_tick()
        completed = adapter.tracking_snapshot()
        assert completed["active"] is False
        assert completed["status"] == "complete"
    finally:
        adapter.destroy_node()
        rclpy.shutdown()


def test_ros_adapter_characterization_owns_commands_and_stops_safely():
    rclpy.init()
    adapter = RosAdapter("test_robot", demo=True)
    try:
        adapter._last_allocated_wrench = time.monotonic()
        started = adapter.start_characterization("x", 0.08, 0.04, 0.1, 0.5)
        assert started["active"] is True
        assert started["axis"] == "x"
        with pytest.raises(RuntimeError, match="characterization"):
            adapter.start_tracking(
                "minimum_jerk", "position", "x", 0.1, 0.5, 0.2, 1
            )
        stopped = adapter.stop_characterization()
        assert stopped["active"] is False
        assert stopped["status"] == "stopped"
    finally:
        adapter.destroy_node()
        rclpy.shutdown()


def test_stop_all_holds_the_measured_pose_when_armed_and_live():
    rclpy.init()
    adapter = RosAdapter("test_robot", demo=True)
    published = []
    adapter.publish_setpoint = lambda mode, values, altitude=False: published.append(
        (mode, list(values), altitude)
    ) or {"mode": mode, "values": list(values), "altitude": altitude}
    try:
        with adapter._lock:
            adapter._signals.update(
                {
                    "pid.position.x.current": 1.2,
                    "pid.position.y.current": -0.4,
                    "pid.position.z.current": 0.7,
                    "pid.attitude.x.current": 0.1,
                    "pid.attitude.y.current": -0.2,
                    "pid.attitude.z.current": 0.3,
                }
            )
        result = adapter.stop_all()
        assert result["holding"] == "pose"
        assert published == [
            ("position", [1.2, -0.4, 0.7], False),
            ("attitude", [0.1, -0.2, 0.3], False),
        ]
    finally:
        adapter.destroy_node()
        rclpy.shutdown()


def test_stop_all_clears_rate_commands_when_pose_is_not_safe_to_hold():
    rclpy.init()
    adapter = RosAdapter("test_robot", demo=True)
    published = []
    adapter.publish_setpoint = lambda mode, values, altitude=False: published.append(
        (mode, list(values), altitude)
    ) or {"mode": mode, "values": list(values), "altitude": altitude}
    try:
        with adapter._lock:
            adapter._kill = True
            adapter._last_odom = 0.0
            adapter._last_error = 0.0
        result = adapter.stop_all()
        assert result["holding"] == "zero_rate"
        assert published == [
            ("velocity", [0.0, 0.0, 0.0], False),
            ("angular_velocity", [0.0, 0.0, 0.0], False),
        ]
    finally:
        adapter.destroy_node()
        rclpy.shutdown()


def test_inner_loop_target_does_not_spike_between_async_callbacks():
    rclpy.init()
    adapter = RosAdapter("test_robot", demo=True)
    try:
        reference = TwistStamped()
        reference.twist.linear.x = 0.18
        reference.twist.angular.z = -0.12
        adapter._reference_velocity_cb(reference)

        command = Setpoint()
        command.velocity = True
        command.setpoint.x = 0.25
        adapter._pos_setpoint_cb(command)
        angular_command = Setpoint()
        angular_command.velocity = True
        angular_command.setpoint.yaw = -0.2
        adapter._att_setpoint_cb(angular_command)

        with adapter._lock:
            adapter._signals["pid.velocity.x.current"] = 0.07
            adapter._signals["pid.angular_velocity.z.current"] = -0.03
        error = Error()
        error.vel_error[0] = 0.06
        error.angvel_error[2] = -0.04
        adapter._error_cb(error)

        # The raw command and current+error reconstruction are useful
        # diagnostics, but neither may overwrite the controller's published
        # acceleration-limited reference used by the graph.
        assert adapter._signals["command.velocity.x"] == pytest.approx(0.25)
        assert adapter._signals["command.angular_velocity.z"] == pytest.approx(-0.2)
        assert adapter._signals["pid.velocity.x.target"] == pytest.approx(0.18)
        assert adapter._signals["pid.angular_velocity.z.target"] == pytest.approx(-0.12)
    finally:
        adapter.destroy_node()
        rclpy.shutdown()


def test_legacy_inner_loop_target_uses_recent_command_then_error_fallback():
    rclpy.init()
    adapter = RosAdapter("test_robot", demo=True)
    try:
        command = Setpoint()
        command.velocity = True
        command.setpoint.x = 0.25
        adapter._pos_setpoint_cb(command)
        with adapter._lock:
            adapter._signals["pid.velocity.x.current"] = 0.10

        error = Error()
        error.vel_error[0] = 0.12
        adapter._error_cb(error)
        assert adapter._signals["pid.velocity.x.target"] == pytest.approx(0.25)

        adapter._last_inner_command["velocity"] -= 1.0
        adapter._error_cb(error)
        assert adapter._signals["pid.velocity.x.target"] == pytest.approx(0.22)
    finally:
        adapter.destroy_node()
        rclpy.shutdown()


def test_ros_adapter_exposes_path_preview_and_streams_full_position_reference():
    rclpy.init()
    adapter = RosAdapter("test_robot", demo=True)
    published = []
    original_publish = adapter.publish_setpoint
    adapter.publish_setpoint = lambda mode, values, altitude=False: (
        published.append((mode, list(values))), original_publish(mode, values, altitude)
    )[1]
    try:
        started = adapter.start_tracking("follower_pid", "position", "xy", 0.2, 4.0, 0.5, 1)
        assert started["active"] is True
        assert len(started["path_preview"]) > 100
        assert started["path_preview"][0] == pytest.approx([0.0, 0.0, 0.0])
        adapter._publish_tracking_reference(adapter._tracking_run, 1.0)
        assert published[-1][0] == "position"
        assert published[-1][1] != pytest.approx([0.0, 0.0, 0.0])
        assert adapter.tracking_snapshot()["current_reference"] == pytest.approx(published[-1][1])
    finally:
        adapter.stop_tracking()
        adapter.destroy_node()
        rclpy.shutdown()


def test_path_reference_pauses_for_lag_and_resumes_after_catchup():
    rclpy.init()
    adapter = RosAdapter("test_robot", demo=True)
    try:
        adapter.start_tracking(
            "follower_pid", "position", "xy", 0.4, 20.0, 1.0, 1, 0.1
        )
        run = adapter._tracking_run
        run["path_elapsed"] = 5.0
        adapter._publish_tracking_reference(run, 5.0)
        reference = list(adapter.tracking_snapshot()["current_reference"])

        run["last_tick"] = time.monotonic() - 0.1
        adapter._tracking_tick()
        paused = adapter.tracking_snapshot()
        assert run["path_elapsed"] == pytest.approx(5.0)
        assert paused["reference_rate"] == pytest.approx(0.0)
        assert paused["reference_limited"] is True
        assert "catches up" in paused["message"]

        with adapter._lock:
            for axis, value in zip("xyz", reference):
                adapter._signals[f"pid.position.{axis}.current"] = value
        run["last_tick"] = time.monotonic() - 0.1
        adapter._tracking_tick()
        resumed = adapter.tracking_snapshot()
        assert run["path_elapsed"] > 5.0
        assert resumed["reference_rate"] == pytest.approx(1.0)
        assert resumed["reference_limited"] is False
    finally:
        adapter.stop_tracking()
        adapter.destroy_node()
        rclpy.shutdown()


def test_square_test_waits_for_each_corner_and_requires_settling():
    rclpy.init()
    adapter = RosAdapter("test_robot", demo=True)
    try:
        started = adapter.start_tracking(
            "square_test", "position", "xy", 0.4, 60.0, 1.0, 1, 0.05
        )
        assert started["manual_advance"] is True
        assert started["ready_for_next"] is False
        assert started["waypoint_index"] == 0

        adapter._tracking_tick()
        assert adapter.tracking_snapshot()["ready_for_next"] is True

        first = adapter.advance_tracking()
        assert first["waypoint_index"] == 1
        assert first["ready_for_next"] is False
        assert first["current_reference"] == pytest.approx([0.4, 0.0, 0.0])
        with pytest.raises(RuntimeError, match="reach and settle"):
            adapter.advance_tracking()

        with adapter._lock:
            for axis, value in zip("xyz", first["current_reference"]):
                adapter._signals[f"pid.position.{axis}.current"] = value
            adapter._signals["pid.velocity.x.current"] = 0.08
            adapter._signals["pid.velocity.y.current"] = 0.0
        adapter._tracking_tick()
        assert adapter.tracking_snapshot()["ready_for_next"] is False

        with adapter._lock:
            adapter._signals["pid.velocity.x.current"] = 0.0
        adapter._tracking_tick()
        reached = adapter.tracking_snapshot()
        assert reached["ready_for_next"] is True
        assert "press Next corner" in reached["message"]
    finally:
        adapter.stop_tracking()
        adapter.destroy_node()
        rclpy.shutdown()


def test_square_test_completes_only_after_returning_home():
    rclpy.init()
    adapter = RosAdapter("test_robot", demo=True)
    try:
        adapter.start_tracking(
            "square_test", "position", "xy", 0.2, 60.0, 1.0, 1, 0.05
        )
        adapter._tracking_tick()
        for waypoint in range(1, 5):
            state = adapter.advance_tracking()
            with adapter._lock:
                for axis, value in zip("xyz", state["current_reference"]):
                    adapter._signals[f"pid.position.{axis}.current"] = value
                    adapter._signals[f"pid.velocity.{axis}.current"] = 0.0
            adapter._tracking_tick()
            if waypoint < 4:
                assert adapter.tracking_snapshot()["ready_for_next"] is True
        completed = adapter.tracking_snapshot()
        assert completed["active"] is False
        assert completed["status"] == "complete"
        assert completed["progress"] == pytest.approx(1.0)
    finally:
        adapter.destroy_node()
        rclpy.shutdown()


def test_path_tracking_rejects_unsafe_lag_limits():
    rclpy.init()
    adapter = RosAdapter("test_robot", demo=True)
    try:
        with pytest.raises(ValueError, match="maximum path lag"):
            adapter.start_tracking(
                "follower_pid", "position", "xy", 0.4, 20.0, 1.0, 1, 0.01
            )
    finally:
        adapter.destroy_node()
        rclpy.shutdown()


def test_tracking_rejects_duplicate_controller_or_sim_publishers():
    rclpy.init()
    adapter = RosAdapter("test_robot", demo=True)
    try:
        adapter._source_counts = {"control/error": 2}
        adapter._refresh_source_health = lambda: None
        with pytest.raises(RuntimeError, match="multiple ROS control/sim sources"):
            adapter.start_tracking("minimum_jerk", "position", "x", 0.1, 0.5, 0.2, 1)
    finally:
        adapter.destroy_node()
        rclpy.shutdown()


def test_active_tracking_aborts_if_a_duplicate_source_appears():
    rclpy.init()
    adapter = RosAdapter("test_robot", demo=True)
    try:
        adapter._refresh_source_health = lambda: None
        adapter.start_tracking("minimum_jerk", "position", "x", 0.1, 0.5, 0.2, 1)
        adapter._source_counts = {"odometry/filtered": 2}
        adapter._tracking_tick()
        assert adapter.tracking_snapshot()["status"] == "aborted"
    finally:
        adapter.destroy_node()
        rclpy.shutdown()


def test_server_shutdown_stops_an_active_tracking_profile(tmp_path):
    ros = FakeRos()
    ros.tracking = {"active": True}
    server = DashboardServer(ros, tmp_path)
    asyncio.run(server.on_shutdown(web.Application()))
    assert ros.tracking == {"active": False, "status": "stopped"}



def test_one_request_batches_changes_for_a_node(tmp_path):
    ros = FakeRos()
    server = DashboardServer(ros, tmp_path)

    result = asyncio.run(
        server.handle(
            {
                "type": "set_parameters",
                "changes": [
                    {"node": "/sub", "name": "kp", "value": 1.0},
                    {"node": "/sub", "name": "ki", "value": 0.2},
                ],
            }
        )
    )

    assert [item["name"] for item in result["applied"]] == ["kp", "ki"]
    assert ros.values == {("/sub", "kp"): 1.0, ("/sub", "ki"): 0.2}


def test_permanent_save_preflights_profile_before_runtime_write(tmp_path):
    class ChangedProfile:
        available = True
        path = tmp_path / "gains.yaml"
        controller_name = "/sub"
        values = {}

        def ensure_unchanged(self):
            raise RuntimeError("profile changed on disk")

    ros = FakeRos()
    server = DashboardServer(ros, tmp_path, ChangedProfile())
    async def save():
        with pytest.raises(RuntimeError, match="changed on disk"):
            await server.handle(
                {
                    "type": "permanent_save",
                    "changes": [{"node": "/sub", "name": "kp", "value": 3.0}],
                }
            )

    asyncio.run(save())
    assert ros.values == {}
