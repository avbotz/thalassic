"""The dashboard's web server: the page, and one websocket per browser that
streams state at BROADCAST_HZ and carries the browser's requests.

    ros2 run sub_pid_tuner dashboard --ros-args -r __ns:=/marlin_v3 \
        -p profile:=src/sub_bringup/config/control/gains_sim.yaml -p use_sim_time:=true
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import math
import threading
import time
from concurrent.futures import Future
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

import rclpy
from aiohttp import WSCloseCode, hdrs, web
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from .profile import GainProfile
from .ros_adapter import SERVICE_TIMEOUT, RosAdapter

BROADCAST_HZ = 30
BROADCAST_TASK = web.AppKey("broadcast", asyncio.Task)


def encode(message: dict) -> str:
    """`message` as JSON for the browser.

    JSON has no NaN or infinity, so they become null; anything else it cannot
    hold (a date in the gains file) goes as its str(). One such value must not
    stop the state stream.
    """
    return json.dumps(_finite(message), allow_nan=False, default=str)


def _finite(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: _finite(item) for key, item in value.items()}
    if isinstance(value, list | tuple):
        return [_finite(item) for item in value]
    return value


async def answer(future: Future, what: str) -> Any:
    """The result of a ROS request, or an error once `what` has not answered for SERVICE_TIMEOUT."""
    try:
        # Shielded: a timeout must not cancel the future, which the ROS thread still resolves.
        return await asyncio.wait_for(asyncio.shield(asyncio.wrap_future(future)), SERVICE_TIMEOUT)
    except TimeoutError:
        raise RuntimeError(f"{what} did not answer within {SERVICE_TIMEOUT:g} s") from None


class DashboardServer:
    def __init__(self, ros: RosAdapter, web_dir: Path, profile: GainProfile | None = None):
        self.ros = ros
        self.web_dir = web_dir
        self.profile = profile
        self.clients: set[web.WebSocketResponse] = set()
        # The browser that started the running routine; the routine stops if it goes.
        self.routine_owner: web.WebSocketResponse | None = None
        self.write_lock = asyncio.Lock()
        # ros.parameters_version last broadcast; None sends the parameters again.
        self.sent_version: int | None = None

    def state(self, include_parameters: bool = True) -> dict:
        state = self.ros.snapshot(include_parameters=include_parameters)
        state["server_time"] = time.monotonic()
        if include_parameters:
            state["profile"] = (
                {"path": str(self.profile.path), "values": dict(self.profile.values)}
                if self.profile
                else None
            )
        return state

    def application(self) -> web.Application:
        app = web.Application(middlewares=[self.security_headers])
        app.router.add_get("/", self.index)
        app.router.add_get("/api/ws", self.websocket)
        app.router.add_static("/static/", self.web_dir, show_index=False, follow_symlinks=True)
        app.on_startup.append(self.on_startup)
        app.on_shutdown.append(self.on_shutdown)
        return app

    @web.middleware
    async def security_headers(self, request, handler):
        response = await handler(request)
        response.headers["Content-Security-Policy"] = (
            "default-src 'self'; connect-src 'self' ws: wss:; "
            "style-src 'self' 'unsafe-inline'; script-src 'self'"
        )
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["Cache-Control"] = "no-store"
        return response

    async def index(self, _request):
        return web.FileResponse(self.web_dir / "index.html")

    async def on_startup(self, app: web.Application) -> None:
        app[BROADCAST_TASK] = asyncio.create_task(self.broadcast_loop())

    async def on_shutdown(self, app: web.Application) -> None:
        # Never leave a routine streaming targets.
        with contextlib.suppress(Exception):
            self.ros.stop_routine()
        for socket in list(self.clients):
            await socket.close(code=WSCloseCode.GOING_AWAY, message=b"dashboard stopping")
        self.clients.clear()
        task = app.get(BROADCAST_TASK)
        if task:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

    async def broadcast_loop(self) -> None:
        loop = asyncio.get_running_loop()
        deadline = loop.time()
        while True:
            if self.clients:
                # Parameters go out when they change; new browsers get them on connecting.
                version = self.ros.parameters_version
                try:
                    state = self.state(include_parameters=version != self.sent_version)
                    message = encode({"type": "state", **state})
                except Exception as exc:
                    # Skip this one: ending the loop would freeze every browser on stale state.
                    self.ros.get_logger().error(
                        f"state broadcast failed: {exc!r}", throttle_duration_sec=5.0
                    )
                else:
                    self.sent_version = version
                    for socket in list(self.clients):
                        try:
                            await socket.send_str(message)
                        except (ConnectionError, RuntimeError):
                            self.clients.discard(socket)
            # On a fixed period, not a sleep after the work; no catching up after a stall.
            deadline = max(deadline + 1.0 / BROADCAST_HZ, loop.time())
            await asyncio.sleep(deadline - loop.time())

    async def websocket(self, request):
        # Any page the browser has open may open a websocket here, and this one
        # drives the vehicle: only the dashboard's own page gets one.
        origin = request.headers.get(hdrs.ORIGIN)
        if origin is not None and urlsplit(origin).netloc != request.host:
            raise web.HTTPForbidden(text=f"the dashboard does not take requests from {origin}")
        socket = web.WebSocketResponse(heartbeat=10.0, max_msg_size=1024 * 1024)
        await socket.prepare(request)
        self.clients.add(socket)
        try:
            await socket.send_str(encode({"type": "state", **self.state()}))
            async for message in socket:
                if message.type != web.WSMsgType.TEXT:
                    continue
                request_id = None
                try:
                    payload = json.loads(message.data)
                    if not isinstance(payload, dict):
                        raise ValueError("a request is a JSON object")
                    request_id = payload.get("request_id")
                    response = await self.handle(payload)
                    if payload.get("type") == "start_routine":
                        self.routine_owner = socket
                except Exception as exc:
                    response = {"type": "error", "message": str(exc) or type(exc).__name__}
                response["request_id"] = request_id
                await socket.send_str(encode(response))
        finally:
            self.clients.discard(socket)
            if self.routine_owner is socket:
                self.routine_owner = None
                with contextlib.suppress(Exception):
                    self.ros.stop_routine()
        return socket

    async def handle(self, message: dict) -> dict:
        kind = message.get("type")
        if kind == "refresh":
            self.ros.refresh_parameters()
            if self.profile:
                self.profile.reload()
            self.sent_version = None
            return {"type": "refreshed"}
        if kind == "load_parameters":
            self.ros.request_parameters(str(message.get("node", "")))
            return {"type": "loading"}
        if kind == "set_parameters":
            async with self.write_lock:
                applied = await self.apply_changes(message.get("changes"))
            return {"type": "applied", "applied": applied}
        if kind == "save_profile":
            if self.profile is None:
                raise RuntimeError("no gains profile was given (the profile parameter)")
            async with self.write_lock:
                # Before touching the controller, so a failed save changes nothing.
                self.profile.ensure_unchanged()
                changes = message.get("changes") or []
                applied = await self.apply_changes(changes) if changes else []
                controller = self.ros.controller_fqn
                runtime = await answer(self.ros.read_parameters(controller), controller)
                saved = self.profile.save(runtime)
            self.sent_version = None
            return {"type": "saved", "applied": applied, **saved}
        if kind == "command":
            result = self.ros.publish_command(
                list(message.get("modes", [])),
                list(message.get("values", [])),
                bool(message.get("altitude", False)),
                str(message.get("frame", "heading")),
                float(message.get("timeout", 0.0)),
            )
            return {"type": "commanded", **result}
        if kind == "hold":
            return {"type": "routine", "routine": self.ros.hold_here()}
        if kind == "zero_pose":
            reset_pose = f"{self.ros.controller_fqn}/reset_pose"
            return {"type": "zeroed", **await answer(self.ros.zero_pose(), reset_pose)}
        if kind == "start_routine":
            routine = self.ros.start_routine(
                str(message.get("kind", "")),
                str(message.get("axis", "")),
                message.get("amplitude", 0.0),
                message.get("hold", 0.0),
                message.get("settle", 0.0),
                message.get("cycles", 1),
            )
            return {"type": "routine", "routine": routine}
        if kind == "stop_routine":
            return {"type": "routine", "routine": self.ros.stop_routine()}
        raise ValueError(f"unknown request: {kind}")

    async def apply_changes(self, changes) -> list[dict]:
        """Apply [{node, name, value}], atomically per node, and return the read-back values."""
        if (
            not isinstance(changes, list)
            or not changes
            or not all(isinstance(change, dict) for change in changes)
        ):
            raise ValueError("changes must be a non-empty list of {node, name, value}")
        by_node: dict[str, list[dict]] = {}
        for change in changes:
            by_node.setdefault(str(change.get("node", "")), []).append(change)
        applied = []
        for node, node_changes in by_node.items():
            applied.extend(await answer(self.ros.set_parameters(node, node_changes), node))
        return applied


def main(args=None):
    # aiohttp handles SIGINT and SIGTERM, and stops any routine on the way out.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    ros = RosAdapter()
    profile_path = ros.get_parameter("profile").value
    profile = GainProfile(Path(profile_path)) if profile_path else None
    # One thread: the node's callbacks are mutually exclusive anyway, and more
    # threads starve the web server of the GIL.
    executor = SingleThreadedExecutor()
    executor.add_node(ros)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    try:
        host = ros.get_parameter("host").value
        port = ros.get_parameter("port").value
        server = DashboardServer(
            ros, Path(get_package_share_directory("sub_pid_tuner")) / "web", profile
        )
        ros.get_logger().info(
            f"dashboard on http://{host}:{port}"
            + (f", saving to {profile.path}" if profile else ", no profile to save to")
        )
        web.run_app(
            server.application(),
            host=host,
            port=port,
            handle_signals=True,
            shutdown_timeout=1.0,
            print=None,
        )
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=1.0)
        ros.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spinner.join(timeout=1.0)


if __name__ == "__main__":
    main()
