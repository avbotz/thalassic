from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import threading
import time
from pathlib import Path

import rclpy
from aiohttp import WSCloseCode, web
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from .profile import GainProfile
from .ros_adapter import RosAdapter

BROADCAST_HZ = 30


class DashboardServer:
    def __init__(self, ros: RosAdapter, web_dir: Path, profile: GainProfile | None = None):
        self.ros = ros
        self.web_dir = web_dir
        self.profile = profile
        self.clients: set[web.WebSocketResponse] = set()
        self.tracking_owner: web.WebSocketResponse | None = None
        self.write_lock = asyncio.Lock()

    def state(self, include_parameters: bool = True) -> dict:
        state = self.ros.snapshot(include_parameters=include_parameters)
        if not include_parameters and state.get("tracking"):
            # The sampled path is static. Send it on connect/start and with the
            # occasional full state, not in every 30 Hz telemetry frame.
            state["tracking"].pop("path_preview", None)
        state["server_time"] = time.monotonic()
        state["profile"] = {
            "available": bool(self.profile and self.profile.available),
            "path": str(self.profile.path) if self.profile and self.profile.path else None,
            "controller": self.profile.controller_name if self.profile else None,
            "values": dict(self.profile.values) if self.profile else {},
        }
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
        app["broadcast_task"] = asyncio.create_task(self.broadcast_loop())

    async def on_shutdown(self, app: web.Application) -> None:
        # A graceful dashboard shutdown must never leave a rate command active.
        with contextlib.suppress(Exception):
            self.ros.stop_tracking()
        for socket in list(self.clients):
            await socket.close(code=WSCloseCode.GOING_AWAY, message=b"dashboard stopping")
        self.clients.clear()
        task = app.get("broadcast_task")
        if task:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

    async def broadcast_loop(self) -> None:
        tick = 0
        while True:
            if self.clients:
                message = json.dumps(
                    {
                        "type": "state",
                        **self.state(include_parameters=tick % (BROADCAST_HZ * 2) == 0),
                    }
                )
                dead = []
                for socket in self.clients:
                    try:
                        await socket.send_str(message)
                    except (ConnectionError, RuntimeError):
                        dead.append(socket)
                for socket in dead:
                    self.clients.discard(socket)
            tick += 1
            await asyncio.sleep(1.0 / BROADCAST_HZ)

    async def websocket(self, request):
        socket = web.WebSocketResponse(heartbeat=10.0, max_msg_size=1024 * 1024)
        await socket.prepare(request)
        self.clients.add(socket)
        await socket.send_json({"type": "state", **self.state()})
        try:
            async for message in socket:
                if message.type != web.WSMsgType.TEXT:
                    continue
                payload = {}
                try:
                    payload = json.loads(message.data)
                    response = await self.handle(payload)
                    if payload.get("type") == "start_tracking":
                        self.tracking_owner = socket
                    elif payload.get("type") == "stop_tracking" and self.tracking_owner is socket:
                        self.tracking_owner = None
                except Exception as exc:  # noqa: BLE001
                    response = {
                        "type": "error",
                        "request_id": payload.get("request_id"),
                        "message": str(exc),
                    }
                await socket.send_json(response)
        finally:
            self.clients.discard(socket)
            if self.tracking_owner is socket:
                self.tracking_owner = None
                with contextlib.suppress(Exception):
                    self.ros.stop_tracking()
        return socket

    async def handle(self, message: dict) -> dict:
        request_id = message.get("request_id")
        kind = message.get("type")
        if kind == "refresh":
            self.ros.refresh_parameters()
            if self.profile:
                self.profile.reload()
            return {"type": "refresh_result", "request_id": request_id}
        if kind == "load_parameters":
            self.ros.request_parameters(str(message.get("node", "")))
            return {"type": "load_parameters_result", "request_id": request_id}
        if kind == "set_parameters":
            async with self.write_lock:
                applied = await self.apply_changes(message.get("changes"))
            return {
                "type": "parameters_result",
                "request_id": request_id,
                "applied": applied,
            }
        if kind == "permanent_save":
            async with self.write_lock:
                changes = message.get("changes", [])
                if not self.profile or not self.profile.available:
                    raise RuntimeError(
                        "permanent save is unavailable: no profile was configured"
                    )
                self.profile.ensure_unchanged()
                applied = await self.apply_changes(changes) if changes else []
                parameters = self.ros.snapshot()["parameters"].get(
                    self.profile.controller_name, {}
                )
                saved = self.profile.save(parameters)
            return {
                "type": "permanent_result",
                "request_id": request_id,
                "applied": applied,
                **saved,
            }
        if kind == "publish_setpoint":
            result = self.ros.publish_setpoint(
                str(message.get("mode", "")),
                list(message.get("values", [])),
                bool(message.get("altitude", False)),
            )
            return {
                "type": "setpoint_result",
                "request_id": request_id,
                **result,
            }
        if kind == "start_tracking":
            result = self.ros.start_tracking(
                str(message.get("experiment", "")),
                str(message.get("mode", "")),
                str(message.get("axis", "")),
                message.get("amplitude"),
                message.get("ramp_time"),
                message.get("hold_time"),
                message.get("cycles"),
            )
            return {
                "type": "tracking_result",
                "request_id": request_id,
                "tracking": result,
            }
        if kind == "stop_tracking":
            return {
                "type": "tracking_result",
                "request_id": request_id,
                "tracking": self.ros.stop_tracking(),
            }
        raise ValueError(f"unknown request type: {kind}")

    async def apply_changes(self, changes) -> list[dict]:
        if not isinstance(changes, list) or not changes:
            raise ValueError("changes must be a non-empty list")
        grouped: dict[str, list[dict]] = {}
        for change in changes:
            node = str(change.get("node", ""))
            grouped.setdefault(node, []).append(change)

        applied = []
        for node, node_changes in grouped.items():
            future = self.ros.set_parameters(node, node_changes)
            applied.extend(await asyncio.wrap_future(future))
        return applied


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="AVBotz live ROS debugging dashboard")
    parser.add_argument("--robot-name", default="marlin_v2")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8080, type=int)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--controller-node", default="sub_control")
    parser.add_argument("--demo", action="store_true")
    return parser.parse_known_args(argv)[0]


def main(argv=None):
    args = parse_args(argv)
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    ros = RosAdapter(args.robot_name, controller_node=args.controller_node, demo=args.demo)
    profile = GainProfile(
        args.profile,
        controller_name=ros.controller_fqn,
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(ros)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        web_dir = Path(get_package_share_directory("sub_pid_tuner")) / "web"
        server = DashboardServer(ros, web_dir, profile)
        print(f"AVBotz Dashboard: http://{args.host}:{args.port}")
        web.run_app(
            server.application(),
            host=args.host,
            port=args.port,
            handle_signals=True,
            shutdown_timeout=1.0,
            print=None,
        )
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=1.0)
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=1.0)
        ros.destroy_node()


if __name__ == "__main__":
    main()
