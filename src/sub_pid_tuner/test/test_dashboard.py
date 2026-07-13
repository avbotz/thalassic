import asyncio
from concurrent.futures import Future

import pytest
import rclpy
from aiohttp import web
from rcl_interfaces.msg import ParameterType

from sub_pid_tuner.ros_adapter import RosAdapter, coerce_parameter
from sub_pid_tuner.server import BROADCAST_HZ, DashboardServer


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
