import asyncio
import json
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

import pytest
from aiohttp import WSServerHandshakeError, web
from aiohttp.test_utils import TestClient, TestServer
from sub_pid_tuner.profile import GainProfile
from sub_pid_tuner.server import BROADCAST_HZ, DashboardServer, encode

WEB = Path(__file__).resolve().parents[1] / "web"
CONTROLLER = "/marlin_v3/sub_control"


def done(value) -> Future:
    future = Future()
    future.set_result(value)
    return future


class FakeRos:
    """Stands in for RosAdapter: records what the server asks of it."""

    controller_fqn = CONTROLLER
    parameters_version = 0

    def __init__(self):
        self.values = {
            (CONTROLLER, "power_limit"): 0.6,
            (CONTROLLER, "gains.x"): [1.92, 0.512, 2.4],
        }
        self.routine = {"active": False, "status": "idle", "message": "Ready"}
        self.commands = []
        self.errors = []

    def get_logger(self):
        return SimpleNamespace(error=lambda message, **_: self.errors.append(message))

    def set_parameters(self, node, changes):
        for change in changes:
            self.values[(node, change["name"])] = change["value"]
        return done([{"node": node, "name": c["name"], "value": c["value"]} for c in changes])

    def read_parameters(self, node):
        return done({name: value for (owner, name), value in self.values.items() if owner == node})

    def publish_command(self, modes, values, altitude, frame, timeout):
        self.commands.append((modes, values, altitude, frame, timeout))
        return {
            "modes": modes,
            "values": values,
            "altitude": altitude,
            "frame": frame,
            "timeout": timeout,
        }

    def hold_here(self):
        self.routine = {"active": False, "status": "stopped", "message": "Stopped by operator"}
        return self.routine

    def zero_pose(self):
        return done({"success": True, "message": "zeroing x, y and yaw"})

    def start_routine(self, kind, axis, amplitude, hold, settle, cycles):
        self.routine = {"active": True, "kind": kind, "axis": axis, "preview": [[0.0, 0.0, -1.0]]}
        return self.routine

    def stop_routine(self):
        self.routine = {"active": False, "status": "stopped", "message": "Stopped by operator"}
        return self.routine

    def refresh_parameters(self):
        pass

    def request_parameters(self, node):
        pass

    def snapshot(self, include_parameters=True):
        result = {
            "namespace": "/marlin_v3",
            "controller": CONTROLLER,
            "signals": {"z.measured": -1.0},
            "routine": dict(self.routine),
            "telemetry_age": 0.01,
            "killed": False,
        }
        if include_parameters:
            result["parameters"] = {}
        return result


def run(server, message):
    return asyncio.run(server.handle(message))


def test_parameter_changes_are_applied_per_node():
    ros = FakeRos()
    server = DashboardServer(ros, WEB)
    result = run(
        server,
        {
            "type": "set_parameters",
            "changes": [
                {"node": CONTROLLER, "name": "gains.x", "value": [3.0, 1.0, 3.0]},
                {"node": "/marlin_v3/ekf_filter_node", "name": "frequency", "value": 30.0},
            ],
        },
    )
    assert [item["name"] for item in result["applied"]] == ["gains.x", "frequency"]
    assert ros.values[(CONTROLLER, "gains.x")] == [3.0, 1.0, 3.0]
    with pytest.raises(ValueError, match="list of"):
        run(server, {"type": "set_parameters", "changes": ["gains.x"]})


def test_a_node_that_never_answers_fails_the_request_and_frees_the_lock(monkeypatch):
    monkeypatch.setattr("sub_pid_tuner.server.SERVICE_TIMEOUT", 0.05)
    ros = FakeRos()
    unanswered = Future()
    ros.set_parameters = lambda node, changes: unanswered
    server = DashboardServer(ros, WEB)
    change = {"node": CONTROLLER, "name": "power_limit", "value": 0.5}

    async def apply():
        with pytest.raises(RuntimeError, match="did not answer"):
            await asyncio.wait_for(
                server.handle({"type": "set_parameters", "changes": [change]}), 1.0
            )
        assert not server.write_lock.locked()

    asyncio.run(apply())
    # Still the ROS thread's to resolve: resolving a cancelled future would raise there.
    assert not unanswered.cancelled()


def test_state_is_json_the_browser_can_parse():
    text = encode({"signals": {"z.measured": float("nan")}, "health": [float("inf"), 1.0]})
    assert json.loads(text) == {"signals": {"z.measured": None}, "health": [None, 1.0]}


def test_save_profile_applies_then_writes_the_controller_values(tmp_path):
    path = tmp_path / "gains_sim.yaml"
    path.write_text(
        "/**:\n  ros__parameters:\n    power_limit: 0.6  # cap\n    gains:\n      x: [1.92, 0.512, 2.4]\n"
    )
    server = DashboardServer(FakeRos(), WEB, GainProfile(path))
    result = run(
        server,
        {
            "type": "save_profile",
            "changes": [{"node": CONTROLLER, "name": "gains.x", "value": [3.0, 1.0, 3.0]}],
        },
    )
    assert result["written"] == {"gains.x": [3.0, 1.0, 3.0]}
    assert "      x: [3.0, 1.0, 3.0]\n" in path.read_text()
    assert "power_limit: 0.6  # cap" in path.read_text()


def test_save_profile_checks_the_file_before_changing_the_controller(tmp_path):
    path = tmp_path / "gains.yaml"
    path.write_text("/**:\n  ros__parameters:\n    power_limit: 0.6\n")
    ros = FakeRos()
    server = DashboardServer(ros, WEB, GainProfile(path))
    path.write_text("/**:\n  ros__parameters:\n    power_limit: 0.5\n")
    with pytest.raises(RuntimeError, match="changed on disk"):
        run(
            server,
            {
                "type": "save_profile",
                "changes": [{"node": CONTROLLER, "name": "power_limit", "value": 0.4}],
            },
        )
    assert ros.values[(CONTROLLER, "power_limit")] == 0.6


def test_save_profile_needs_a_profile():
    with pytest.raises(RuntimeError, match="no gains profile"):
        run(DashboardServer(FakeRos(), WEB), {"type": "save_profile", "changes": []})


def test_commands_and_routines_reach_the_adapter():
    ros = FakeRos()
    server = DashboardServer(ros, WEB)
    command = run(
        server,
        {
            "type": "command",
            "modes": ["KEEP", "KEEP", "POSITION", "KEEP", "KEEP", "KEEP"],
            "values": [0, 0, -1.2, 0, 0, 0],
            "timeout": 0,
        },
    )
    assert command["modes"][2] == "POSITION"
    assert ros.commands[0][1][2] == -1.2
    started = run(server, {"type": "start_routine", "kind": "step", "axis": "z", "amplitude": 0.5})
    assert started["routine"]["active"] is True
    assert run(server, {"type": "stop_routine"})["routine"]["status"] == "stopped"
    assert run(server, {"type": "zero_pose"})["success"] is True
    with pytest.raises(ValueError, match="unknown request"):
        run(server, {"type": "pid"})


def test_streamed_state_leaves_out_unchanged_parameters():
    server = DashboardServer(FakeRos(), WEB)
    run(server, {"type": "start_routine", "kind": "step", "axis": "z", "amplitude": 0.5})
    state = server.state(include_parameters=False)
    assert "parameters" not in state and "profile" not in state
    assert state["routine"]["preview"] == [[0.0, 0.0, -1.0]]
    assert isinstance(state["server_time"], float)
    assert BROADCAST_HZ >= 30


def test_serves_the_page_and_streams_state_over_the_websocket():
    ros = FakeRos()
    server = DashboardServer(ros, WEB)

    async def browse():
        async with TestClient(TestServer(server.application())) as client:
            page = await client.get("/")
            assert page.status == 200
            assert "AVBotz Dashboard" in await page.text()
            assert page.headers["Cache-Control"] == "no-store"
            for asset in ("app.js", "style.css"):
                assert (await client.get(f"/static/{asset}")).status == 200

            socket = await client.ws_connect("/api/ws")
            state = await socket.receive_json()
            assert state["type"] == "state"
            assert state["namespace"] == "/marlin_v3"
            assert "parameters" in state

            await socket.send_json({"type": "start_routine", "request_id": 7, "kind": "hold"})
            while (reply := await socket.receive_json())["type"] == "state":
                pass
            assert reply == {"type": "routine", "routine": ros.routine, "request_id": 7}

            await socket.send_json({"type": "nonsense", "request_id": 8})
            while (reply := await socket.receive_json())["type"] == "state":
                pass
            assert reply["type"] == "error" and reply["request_id"] == 8

            # The browser that started a routine going away stops it.
            await socket.close()
            for _ in range(50):
                if not ros.routine["active"]:
                    break
                await asyncio.sleep(0.01)
            assert ros.routine["status"] == "stopped"

    asyncio.run(browse())


def test_the_websocket_serves_only_the_dashboard_page_and_survives_bad_requests():
    async def browse():
        async with TestClient(TestServer(DashboardServer(FakeRos(), WEB).application())) as client:
            # Another site open in the operator's browser must not drive the vehicle.
            with pytest.raises(WSServerHandshakeError) as refused:
                await client.ws_connect("/api/ws", origin="http://example.com")
            assert refused.value.status == 403

            socket = await client.ws_connect(
                "/api/ws", origin=f"http://{client.host}:{client.port}"
            )
            await socket.receive_json()
            await socket.send_str("[1]")
            while (reply := await socket.receive_json())["type"] == "state":
                pass
            assert reply == {
                "type": "error",
                "message": "a request is a JSON object",
                "request_id": None,
            }
            await socket.send_json({"type": "hold", "request_id": 2})
            while (reply := await socket.receive_json())["type"] == "state":
                pass
            assert reply["type"] == "routine" and reply["request_id"] == 2

    asyncio.run(browse())


def test_a_failed_snapshot_does_not_end_the_state_stream():
    ros = FakeRos()
    snapshot = ros.snapshot
    calls = 0

    def first_broadcast_fails(include_parameters=True):
        nonlocal calls
        calls += 1
        if calls == 2:  # the first after the state a new browser gets
            raise RuntimeError("snapshot failed")
        return snapshot(include_parameters)

    ros.snapshot = first_broadcast_fails

    async def watch():
        async with TestClient(TestServer(DashboardServer(ros, WEB).application())) as client:
            socket = await client.ws_connect("/api/ws")
            await socket.receive_json()
            frame = await asyncio.wait_for(socket.receive_json(), 1.0)
            assert frame["type"] == "state"

    asyncio.run(watch())
    assert ros.errors == ["state broadcast failed: RuntimeError('snapshot failed')"]


def test_shutdown_stops_a_running_routine():
    ros = FakeRos()
    ros.routine = {"active": True}
    asyncio.run(DashboardServer(ros, WEB).on_shutdown(web.Application()))
    assert ros.routine["status"] == "stopped"


def test_parameters_and_profile_are_streamed_when_they_change(tmp_path):
    path = tmp_path / "gains.yaml"
    path.write_text("/**:\n  ros__parameters:\n    power_limit: 0.6\n")
    ros = FakeRos()
    server = DashboardServer(ros, WEB, GainProfile(path))

    async def watch():
        async with TestClient(TestServer(server.application())) as client:
            socket = await client.ws_connect("/api/ws")
            assert "profile" in await socket.receive_json()
            while "parameters" in (frame := await socket.receive_json()):
                pass
            assert "profile" not in frame
            ros.parameters_version += 1
            for _ in range(5):
                if "parameters" in (frame := await socket.receive_json()):
                    break
            assert frame["profile"]["values"] == {"power_limit": 0.6}

    asyncio.run(watch())
