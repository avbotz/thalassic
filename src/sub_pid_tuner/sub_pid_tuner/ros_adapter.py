from __future__ import annotations

import math
import threading
import time
from concurrent.futures import Future
from typing import Any

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64
from sub_control_interfaces.msg import Error, Setpoint

from .tracking_profile import TrackingProfile


TYPE_NAMES = {
    ParameterType.PARAMETER_BOOL: "bool",
    ParameterType.PARAMETER_INTEGER: "integer",
    ParameterType.PARAMETER_DOUBLE: "double",
    ParameterType.PARAMETER_STRING: "string",
    ParameterType.PARAMETER_BYTE_ARRAY: "byte_array",
    ParameterType.PARAMETER_BOOL_ARRAY: "bool_array",
    ParameterType.PARAMETER_INTEGER_ARRAY: "integer_array",
    ParameterType.PARAMETER_DOUBLE_ARRAY: "double_array",
    ParameterType.PARAMETER_STRING_ARRAY: "string_array",
}


def parameter_value(value: Any) -> Any:
    fields = {
        ParameterType.PARAMETER_BOOL: "bool_value",
        ParameterType.PARAMETER_INTEGER: "integer_value",
        ParameterType.PARAMETER_DOUBLE: "double_value",
        ParameterType.PARAMETER_STRING: "string_value",
        ParameterType.PARAMETER_BYTE_ARRAY: "byte_array_value",
        ParameterType.PARAMETER_BOOL_ARRAY: "bool_array_value",
        ParameterType.PARAMETER_INTEGER_ARRAY: "integer_array_value",
        ParameterType.PARAMETER_DOUBLE_ARRAY: "double_array_value",
        ParameterType.PARAMETER_STRING_ARRAY: "string_array_value",
    }
    field = fields.get(value.type)
    if field is None:
        return None
    result = getattr(value, field)
    return list(result) if value.type >= ParameterType.PARAMETER_BYTE_ARRAY else result


def coerce_parameter(value: Any, type_id: int) -> Any:
    if type_id == ParameterType.PARAMETER_BOOL:
        if isinstance(value, bool):
            return value
        if str(value).lower() in ("true", "1"):
            return True
        if str(value).lower() in ("false", "0"):
            return False
        raise ValueError("expected true or false")
    if type_id == ParameterType.PARAMETER_INTEGER:
        return int(value)
    if type_id == ParameterType.PARAMETER_DOUBLE:
        result = float(value)
        if not math.isfinite(result):
            raise ValueError("value must be finite")
        return result
    if type_id == ParameterType.PARAMETER_STRING:
        return str(value)
    if type_id in (
        ParameterType.PARAMETER_BYTE_ARRAY,
        ParameterType.PARAMETER_BOOL_ARRAY,
        ParameterType.PARAMETER_INTEGER_ARRAY,
        ParameterType.PARAMETER_DOUBLE_ARRAY,
        ParameterType.PARAMETER_STRING_ARRAY,
    ):
        if not isinstance(value, list):
            raise ValueError("array value must be a JSON list")
        scalar_type = {
            ParameterType.PARAMETER_BYTE_ARRAY: int,
            ParameterType.PARAMETER_INTEGER_ARRAY: int,
            ParameterType.PARAMETER_DOUBLE_ARRAY: float,
            ParameterType.PARAMETER_STRING_ARRAY: str,
        }.get(type_id)
        converted = (
            [coerce_parameter(item, ParameterType.PARAMETER_BOOL) for item in value]
            if type_id == ParameterType.PARAMETER_BOOL_ARRAY
            else [scalar_type(item) for item in value]
        )
        if type_id == ParameterType.PARAMETER_BYTE_ARRAY and any(not 0 <= item <= 255 for item in converted):
            raise ValueError("byte array values must be in [0, 255]")
        if type_id == ParameterType.PARAMETER_DOUBLE_ARRAY and any(not math.isfinite(item) for item in converted):
            raise ValueError("array values must be finite")
        return converted
    raise ValueError("unsupported or unset parameter type")


class RosAdapter(Node):
    def __init__(self, robot_name: str, controller_node: str = "sub_control", demo: bool = False):
        super().__init__("dashboard", namespace=robot_name)
        self.robot_name = robot_name.strip("/")
        self.controller_node = controller_node.strip("/")
        self.controller_fqn = f"/{self.robot_name}/{self.controller_node}"
        self.demo = demo
        self._lock = threading.RLock()
        # Do not use Node._parameters: rclpy owns that attribute for this
        # node's local parameter service.
        self._remote_parameters: dict[str, dict[str, dict]] = {}
        self._parameter_clients: dict[str, AsyncParameterClient] = {}
        self._pending_nodes: set[str] = set()
        self._known_nodes: set[str] = set()
        self._watched_nodes: set[str] = {self.controller_fqn}
        self._signals: dict[str, float] = {}
        self._last_signal = 0.0
        self._last_odom = 0.0
        self._last_error = 0.0
        self._kill = None
        self._source_counts: dict[str, int] = {}
        self._tracking_run: dict | None = None
        self._tracking_state = {
            "active": False,
            "status": "idle",
            "message": "Ready for a tracking routine",
        }

        self.create_subscription(Error, "control/error", self._error_cb, 10)
        self.create_subscription(Odometry, "odometry/filtered", self._odom_cb, 10)
        self.create_subscription(Setpoint, "pos_setpoint", self._pos_setpoint_cb, 10)
        self.create_subscription(Setpoint, "att_setpoint", self._att_setpoint_cb, 10)
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_cb, 10)
        kill_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "kill_switch", self._kill_cb, kill_qos)
        self.create_subscription(Float64, "altitude", lambda msg: self._signal("altitude.current", msg.data), 10)
        for index in range(8):
            self.create_subscription(
                Float64,
                f"control/thruster_{index}",
                lambda msg, index=index: self._signal(f"thruster.{index}", msg.data),
                10,
            )
        self._pos_pub = self.create_publisher(Setpoint, "pos_setpoint", 10)
        self._att_pub = self.create_publisher(Setpoint, "att_setpoint", 10)
        self.create_timer(2.0, self.refresh_parameters)
        self.create_timer(1.0, self._refresh_source_health)
        self.create_timer(1.0 / 30.0, self._tracking_tick)

        if demo:
            self._install_demo_data()

    def _install_demo_data(self) -> None:
        gains = {}
        for bank, base in (("pos_pid", 0.8), ("vel_pid", 30.0), ("att_pid", 0.8), ("ang_pid", 3.5)):
            for axis in "xyz":
                gains[f"{bank}.{axis}"] = {
                    "type": "double_array",
                    "type_id": ParameterType.PARAMETER_DOUBLE_ARRAY,
                    "value": [base, 0.0, 0.0, 1.0],
                    "read_only": False,
                    "description": "",
                }
        gains["control_rate_hz"] = {
            "type": "double",
            "type_id": ParameterType.PARAMETER_DOUBLE,
            "value": 20.0,
            "read_only": True,
            "description": "",
        }
        self._remote_parameters[self.controller_fqn] = gains
        self._kill = False
        for group in ("position", "velocity", "attitude", "angular_velocity"):
            for axis in "xyz":
                self._signals[f"pid.{group}.{axis}.current"] = 0.0
                self._signals[f"pid.{group}.{axis}.error"] = 0.0
                self._signals[f"pid.{group}.{axis}.target"] = 0.0
        self._last_odom = time.monotonic()
        self._last_error = self._last_odom
        self.create_timer(0.05, self._demo_tick)

    def _demo_tick(self) -> None:
        with self._lock:
            for group in ("position", "velocity", "attitude", "angular_velocity"):
                for axis in "xyz":
                    target = self._signals.get(f"pid.{group}.{axis}.target", 0.0)
                    current_name = f"pid.{group}.{axis}.current"
                    current = self._signals.get(current_name, 0.0)
                    current += (target - current) * 0.08
                    self._signals[current_name] = current
                    self._signals[f"pid.{group}.{axis}.error"] = target - current
            self._signals["thruster.0"] = max(
                -0.6, min(0.6, self._signals["pid.position.z.error"] * 0.9)
            )
            self._last_odom = time.monotonic()
            self._last_error = self._last_odom
            self._last_signal = self._last_odom

    def _signal(self, name: str, value: float) -> None:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return
        if not math.isfinite(number):
            return
        with self._lock:
            self._signals[name] = number
            self._last_signal = time.monotonic()

    def _odom_cb(self, msg: Odometry) -> None:
        with self._lock:
            self._last_odom = time.monotonic()
        q = msg.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        groups = {
            "position": (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
            "velocity": (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z),
            "attitude": (roll, pitch, yaw),
            "angular_velocity": (
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ),
        }
        for group, values in groups.items():
            for axis, value in zip("xyz", values):
                self._signal(f"pid.{group}.{axis}.current", value)

    def _error_cb(self, msg: Error) -> None:
        with self._lock:
            self._last_error = time.monotonic()
        groups = {
            "position": msg.pos_error,
            "velocity": msg.vel_error,
            "attitude": msg.att_error,
            "angular_velocity": msg.angvel_error,
        }
        for group, values in groups.items():
            for axis, value in zip("xyz", values):
                self._signal(f"pid.{group}.{axis}.error", value)
                current = self._signals.get(f"pid.{group}.{axis}.current")
                if current is not None and group in ("velocity", "angular_velocity"):
                    self._signal(f"pid.{group}.{axis}.target", current + value)

    def _pos_setpoint_cb(self, msg: Setpoint) -> None:
        group = "velocity" if msg.velocity else "position"
        for axis in "xyz":
            self._signal(f"pid.{group}.{axis}.target", getattr(msg.setpoint, axis))

    def _att_setpoint_cb(self, msg: Setpoint) -> None:
        group = "angular_velocity" if msg.velocity else "attitude"
        for axis, field in zip("xyz", ("roll", "pitch", "yaw")):
            self._signal(f"pid.{group}.{axis}.target", getattr(msg.setpoint, field))

    def _cmd_vel_cb(self, msg: Twist) -> None:
        for axis in "xyz":
            self._signal(f"pid.velocity.{axis}.target", getattr(msg.linear, axis))
            self._signal(f"pid.angular_velocity.{axis}.target", getattr(msg.angular, axis))

    def _kill_cb(self, msg: Bool) -> None:
        stop_tracking = False
        with self._lock:
            was_killed = self._kill
            self._kill = bool(msg.data)
            stop_tracking = self._kill and self._tracking_run is not None
            if was_killed is True and self._kill is False:
                for name in list(self._signals):
                    if name.startswith("pid.") and name.endswith(".target"):
                        self._signals[name] = 0.0
        if stop_tracking:
            self._finish_tracking("aborted", "Kill switch engaged", publish_hold=False)

    def _refresh_source_health(self) -> None:
        counts = {
            "odometry/filtered": self.count_publishers("odometry/filtered"),
            "control/error": self.count_publishers("control/error"),
            "control/thruster_0": self.count_publishers("control/thruster_0"),
        }
        with self._lock:
            self._source_counts = counts

    def _duplicate_sources(self) -> list[str]:
        return sorted(name for name, count in self._source_counts.items() if count > 1)

    def refresh_parameters(self) -> None:
        if self.demo:
            return
        own_name = self.get_fully_qualified_name()
        active = set()
        for name, namespace in self.get_node_names_and_namespaces():
            full_name = f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
            if full_name == own_name:
                continue
            active.add(full_name)
        self._known_nodes = active
        for full_name in sorted(active & self._watched_nodes):
            if full_name in self._pending_nodes:
                continue
            client = self._parameter_clients.setdefault(full_name, AsyncParameterClient(self, full_name))
            if not client.services_are_ready():
                continue
            self._pending_nodes.add(full_name)
            future = client.list_parameters([], 10)
            future.add_done_callback(lambda done, node=full_name, client=client: self._listed(node, client, done))
        with self._lock:
            for stale in set(self._remote_parameters) - active:
                self._remote_parameters.pop(stale, None)
                self._parameter_clients.pop(stale, None)

    def request_parameters(self, node: str) -> None:
        node = str(node)
        if not node.startswith("/") or node == self.get_fully_qualified_name():
            raise ValueError(f"invalid ROS node: {node}")
        self._watched_nodes.add(node)
        self.refresh_parameters()

    def _listed(self, node: str, client: AsyncParameterClient, future) -> None:
        try:
            names = list(future.result().result.names)
            if not names:
                with self._lock:
                    self._remote_parameters[node] = {}
                return
            state = {"values": None, "descriptors": None}

            def finish() -> None:
                if state["values"] is None or state["descriptors"] is None:
                    return
                result = {}
                for name, value, descriptor in zip(names, state["values"], state["descriptors"]):
                    result[name] = {
                        "type": TYPE_NAMES.get(value.type, "unsupported"),
                        "type_id": int(value.type),
                        "value": parameter_value(value),
                        "read_only": bool(descriptor.read_only),
                        "description": descriptor.description,
                    }
                with self._lock:
                    self._remote_parameters[node] = result

            values_future = client.get_parameters(names)
            descriptions_future = client.describe_parameters(names)

            def values_done(done) -> None:
                state["values"] = list(done.result().values)
                finish()

            def descriptions_done(done) -> None:
                state["descriptors"] = list(done.result().descriptors)
                finish()

            values_future.add_done_callback(values_done)
            descriptions_future.add_done_callback(descriptions_done)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f"parameter discovery failed for {node}: {exc}")
        finally:
            self._pending_nodes.discard(node)

    def set_parameter(self, node: str, name: str, value: Any) -> Future:
        return self.set_parameters(node, [{"name": name, "value": value}])

    def set_parameters(self, node: str, changes: list[dict]) -> Future:
        result: Future = Future()
        try:
            if not changes:
                raise ValueError("no parameter changes were provided")
            prepared = []
            with self._lock:
                node_parameters = self._remote_parameters.get(node, {})
                for change in changes:
                    name = str(change.get("name", ""))
                    metadata = node_parameters.get(name)
                    if metadata is None:
                        raise ValueError(f"unknown parameter: {node} {name}")
                    if metadata["read_only"]:
                        raise ValueError(f"{name} is declared read-only by {node}")
                    converted = coerce_parameter(change.get("value"), metadata["type_id"])
                    prepared.append((name, metadata, converted))
        except Exception as exc:  # noqa: BLE001
            result.set_exception(exc)
            return result

        if self.demo:
            with self._lock:
                for _name, metadata, converted in prepared:
                    metadata["value"] = converted
            result.set_result(
                [
                    {"node": node, "name": name, "value": converted}
                    for name, _metadata, converted in prepared
                ]
            )
            return result

        client = self._parameter_clients.get(node)
        if client is None or not client.services_are_ready():
            result.set_exception(RuntimeError(f"parameter service unavailable: {node}"))
            return result

        ros_future = client.set_parameters_atomically(
            [
                Parameter(
                    name=name,
                    type_=Parameter.Type(metadata["type_id"]),
                    value=converted,
                )
                for name, metadata, converted in prepared
            ]
        )

        def applied(done) -> None:
            try:
                response = done.result().result
                if not response.successful:
                    raise RuntimeError(response.reason or "parameter update rejected")
                readback = client.get_parameters(
                    [name for name, _metadata, _converted in prepared]
                )

                def verified(read_done) -> None:
                    try:
                        values = list(read_done.result().values)
                        if len(values) != len(prepared):
                            raise RuntimeError("parameter readback was incomplete")
                        applied_values = []
                        with self._lock:
                            for (name, metadata, _converted), value in zip(prepared, values):
                                actual = parameter_value(value)
                                metadata["value"] = actual
                                applied_values.append(
                                    {"node": node, "name": name, "value": actual}
                                )
                        result.set_result(applied_values)
                    except Exception as exc:  # noqa: BLE001
                        result.set_exception(exc)

                readback.add_done_callback(verified)
            except Exception as exc:  # noqa: BLE001
                result.set_exception(exc)

        ros_future.add_done_callback(applied)
        return result

    def publish_setpoint(self, mode: str, values: list[float], altitude: bool = False) -> dict:
        if len(values) != 3 or any(not math.isfinite(float(value)) for value in values):
            raise ValueError("setpoint must contain three finite values")
        values = [float(value) for value in values]
        msg = Setpoint()
        if mode in ("position", "velocity"):
            msg.velocity = mode == "velocity"
            msg.altitude = bool(altitude and mode == "position")
            msg.setpoint.x, msg.setpoint.y, msg.setpoint.z = values
            self._pos_pub.publish(msg)
        elif mode in ("attitude", "angular_velocity"):
            msg.velocity = mode == "angular_velocity"
            msg.altitude = False
            msg.setpoint.roll, msg.setpoint.pitch, msg.setpoint.yaw = values
            self._att_pub.publish(msg)
        else:
            raise ValueError(f"unknown setpoint mode: {mode}")
        return {"mode": mode, "values": values, "altitude": msg.altitude}

    def _current_triplet(self, group: str) -> list[float] | None:
        with self._lock:
            values = [self._signals.get(f"pid.{group}.{axis}.current") for axis in "xyz"]
        return [float(value) for value in values] if all(value is not None for value in values) else None

    def start_tracking(
        self,
        experiment: str,
        mode: str,
        axis: str,
        amplitude: float,
        ramp_time: float,
        hold_time: float,
        cycles: int,
    ) -> dict:
        profile = TrackingProfile.from_values(
            experiment, mode, axis, amplitude, ramp_time, hold_time, cycles
        )
        self._refresh_source_health()
        now = time.monotonic()
        with self._lock:
            if self._tracking_run is not None:
                raise RuntimeError("a tracking routine is already running")
            duplicate_sources = self._duplicate_sources()
            if duplicate_sources:
                raise RuntimeError(
                    "multiple ROS control/sim sources detected on "
                    + ", ".join(duplicate_sources)
                    + "; stop the duplicate launch before tuning"
                )
            telemetry_age = (
                max(now - self._last_odom, now - self._last_error)
                if self._last_odom and self._last_error
                else None
            )
            if self._kill is not False:
                raise RuntimeError("release the kill switch before starting a tracking routine")
            if telemetry_age is None or telemetry_age > 1.0:
                raise RuntimeError("live odometry/control telemetry is required")
            position = [
                self._signals.get(f"pid.position.{axis_name}.current")
                for axis_name in "xyz"
            ]
            attitude = [
                self._signals.get(f"pid.attitude.{axis_name}.current")
                for axis_name in "xyz"
            ]
            if any(value is None for value in position + attitude):
                raise RuntimeError("position and attitude measurements are not ready")
            position = [float(value) for value in position]
            attitude = [float(value) for value in attitude]
            run = {
                "profile": profile,
                "started_at": now,
                "position_hold": position,
                "attitude_hold": attitude,
            }
            self._tracking_run = run
            self._tracking_state = {
                "active": True,
                "status": "running",
                "message": "Tracking profile running",
                "elapsed": 0.0,
                "progress": 0.0,
                **profile.public_state(),
            }

        # Hold the companion loop once; only the selected command needs to be
        # streamed while its reference changes.
        if mode in ("position", "velocity"):
            self.publish_setpoint("attitude", attitude)
        else:
            self.publish_setpoint("position", position)
        self._publish_tracking_reference(run, 0.0)
        return self.tracking_snapshot()

    def stop_tracking(self) -> dict:
        with self._lock:
            active = self._tracking_run is not None
        if active:
            self._finish_tracking("stopped", "Stopped by operator", publish_hold=True)
        return self.tracking_snapshot()

    def _publish_tracking_reference(self, run: dict, elapsed: float) -> None:
        profile: TrackingProfile = run["profile"]
        value = profile.value_at(elapsed)
        axis = "xyz".index(profile.axis)
        if profile.mode == "position":
            command = list(run["position_hold"])
            command[axis] += value
        elif profile.mode == "attitude":
            command = list(run["attitude_hold"])
            command[axis] += value
        else:
            command = [0.0, 0.0, 0.0]
            command[axis] = value
        self.publish_setpoint(profile.mode, command)

    def _tracking_tick(self) -> None:
        with self._lock:
            run = self._tracking_run
            killed = self._kill
            duplicate_sources = self._duplicate_sources()
            measurement_age = (
                max(
                    time.monotonic() - self._last_odom,
                    time.monotonic() - self._last_error,
                )
                if self._last_odom and self._last_error
                else None
            )
        if run is None:
            return
        if killed:
            self._finish_tracking("aborted", "Kill switch engaged", publish_hold=False)
            return
        if duplicate_sources:
            self._finish_tracking(
                "aborted",
                "Multiple ROS control/sim sources detected",
                publish_hold=True,
            )
            return
        if measurement_age is None or measurement_age > 1.0:
            self._finish_tracking("aborted", "Telemetry became stale", publish_hold=True)
            return

        elapsed = time.monotonic() - run["started_at"]
        profile: TrackingProfile = run["profile"]
        if elapsed >= profile.duration:
            self._finish_tracking("complete", "Tracking routine complete", publish_hold=True)
            return
        self._publish_tracking_reference(run, elapsed)
        with self._lock:
            if self._tracking_run is run:
                self._tracking_state["elapsed"] = elapsed
                self._tracking_state["progress"] = elapsed / profile.duration

    def _finish_tracking(self, status: str, message: str, publish_hold: bool) -> None:
        with self._lock:
            run = self._tracking_run
            if run is None:
                return
            self._tracking_run = None
            profile: TrackingProfile = run["profile"]
            elapsed = min(time.monotonic() - run["started_at"], profile.duration)
            self._tracking_state = {
                "active": False,
                "status": status,
                "message": message,
                "elapsed": elapsed,
                "progress": min(elapsed / profile.duration, 1.0),
                **profile.public_state(),
            }
        if publish_hold:
            position = self._current_triplet("position") or run["position_hold"]
            attitude = self._current_triplet("attitude") or run["attitude_hold"]
            self.publish_setpoint("position", position)
            self.publish_setpoint("attitude", attitude)

    def tracking_snapshot(self) -> dict:
        with self._lock:
            return dict(self._tracking_state)

    def snapshot(self, include_parameters: bool = True) -> dict:
        with self._lock:
            now = time.monotonic()
            odom_age = now - self._last_odom if self._last_odom else None
            error_age = now - self._last_error if self._last_error else None
            result = {
                "robot_name": self.robot_name,
                "demo": self.demo,
                "signals": dict(self._signals),
                "telemetry_age": (
                    max(odom_age, error_age)
                    if odom_age is not None and error_age is not None
                    else None
                ),
                "odometry_age": odom_age,
                "control_error_age": error_age,
                "source_counts": dict(self._source_counts),
                "duplicate_sources": self._duplicate_sources(),
                "killed": self._kill,
                "tracking": dict(self._tracking_state),
                "nodes": sorted(self._known_nodes | set(self._remote_parameters)),
            }
            if include_parameters:
                result["parameters"] = {
                    node: {name: dict(metadata) for name, metadata in params.items()}
                    for node, params in self._remote_parameters.items()
                }
            return result
