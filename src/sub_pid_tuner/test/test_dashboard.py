import asyncio
from concurrent.futures import Future

import pytest
import rclpy
from aiohttp import web
from rcl_interfaces.msg import ParameterType

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

    def start_tracking(self, experiment, mode, axis, amplitude, ramp_time, hold_time, cycles):
        self.tracking = {
            "active": True,
            "experiment": experiment,
            "mode": mode,
            "axis": axis,
            "amplitude": amplitude,
            "ramp_time": ramp_time,
            "hold_time": hold_time,
            "cycles": cycles,
        }
        return self.tracking

    def stop_tracking(self):
        self.tracking = {"active": False, "status": "stopped"}
        return self.tracking

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


def test_trapezoid_profile_is_only_available_for_inner_loops():
    with pytest.raises(ValueError, match="velocity and angular-rate"):
        TrackingProfile.from_values("trapezoid", "position", "x", 0.4, 2.0, 1.0, 1)


def test_follower_pid_profile_is_a_bounded_closed_3d_path():
    profile = TrackingProfile.from_values("follower_pid", "position", "xyz", 0.5, 8.0, 2.0, 1)
    preview = profile.preview()
    assert preview[0] == pytest.approx([0.0, 0.0, 0.0])
    assert preview[-1] == pytest.approx([0.0, 0.0, 0.0])
    assert max(abs(point[0]) for point in preview) <= 0.5
    assert max(abs(point[1]) for point in preview) <= 0.65
    assert any(abs(point[2]) > 0.05 for point in preview)
    assert profile.vector_at(profile.ramp_time + 0.5) == pytest.approx((0.0, 0.0, 0.0))


def test_spline_test_returns_home_and_respects_selected_plane():
    profile = TrackingProfile.from_values("spline_test", "position", "xz", 0.6, 8.0, 2.0, 1)
    preview = profile.preview()
    assert preview[0] == pytest.approx([0.0, 0.0, 0.0])
    assert preview[-1] == pytest.approx([0.0, 0.0, 0.0])
    assert all(point[1] == pytest.approx(0.0) for point in preview)
    assert any(abs(point[2]) > 0.05 for point in preview)


def test_path_op_modes_require_position_and_a_valid_plane():
    with pytest.raises(ValueError, match="position controller"):
        TrackingProfile.from_values("follower_pid", "velocity", "xyz", 0.4, 4.0, 1.0, 1)
    with pytest.raises(ValueError, match="path plane"):
        TrackingProfile.from_values("spline_test", "position", "x", 0.4, 4.0, 1.0, 1)


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


def test_ros_adapter_exposes_path_preview_and_streams_full_position_reference():
    rclpy.init()
    adapter = RosAdapter("test_robot", demo=True)
    published = []
    original_publish = adapter.publish_setpoint
    adapter.publish_setpoint = lambda mode, values, altitude=False: (
        published.append((mode, list(values))), original_publish(mode, values, altitude)
    )[1]
    try:
        started = adapter.start_tracking("follower_pid", "position", "xyz", 0.2, 4.0, 0.5, 1)
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
