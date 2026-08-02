from __future__ import annotations

import math
import threading
import time
from concurrent.futures import Future
from typing import Any

from geometry_msgs.msg import AccelStamped, Twist, TwistStamped, WrenchStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64
from sub_control_interfaces.msg import Error, Setpoint

from .feedforward_identification import (
    AXES,
    CharacterizationConfig,
    CharacterizationSample,
    fit_feedforward,
)
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

# Keep a recently observed command/reference authoritative over the target
# reconstructed from asynchronous odometry and error messages. Both streams
# normally run at 20-30 Hz, so this leaves ample room for ordinary callback
# jitter while still supporting legacy controllers that only expose errors.
INNER_TARGET_FRESHNESS_S = 0.25


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
        self._last_reference_velocity = 0.0
        self._last_allocated_wrench = 0.0
        self._last_inner_command = {"velocity": 0.0, "angular_velocity": 0.0}
        self._kill = None
        self._source_counts: dict[str, int] = {}
        self._tracking_run: dict | None = None
        self._tracking_state = {
            "active": False,
            "status": "idle",
            "message": "Ready for a tracking routine",
        }
        self._characterization_run: dict | None = None
        self._characterization_state = {
            "active": False,
            "status": "idle",
            "message": "Ready to characterize one body axis",
        }

        self.create_subscription(Error, "control/error", self._error_cb, 10)
        self.create_subscription(Odometry, "odometry/filtered", self._odom_cb, 10)
        self.create_subscription(Setpoint, "pos_setpoint", self._pos_setpoint_cb, 10)
        self.create_subscription(Setpoint, "att_setpoint", self._att_setpoint_cb, 10)
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_cb, 10)
        for topic, prefix in (
            ("control/feedforward_wrench", "wrench.feedforward"),
            ("control/feedback_wrench", "wrench.feedback"),
            ("control/commanded_wrench", "wrench.commanded"),
            ("control/allocated_wrench", "wrench.allocated"),
            ("control/allocation_residual", "wrench.residual"),
        ):
            self.create_subscription(
                WrenchStamped,
                topic,
                lambda msg, prefix=prefix: self._wrench_cb(prefix, msg),
                10,
            )
        self.create_subscription(
            TwistStamped,
            "control/reference_velocity",
            self._reference_velocity_cb,
            10,
        )
        self.create_subscription(
            AccelStamped,
            "control/reference_acceleration",
            self._reference_acceleration_cb,
            10,
        )
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
        self.create_timer(1.0 / 30.0, self._characterization_tick)

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
        gains["power_limit"] = {
            "type": "double",
            "type_id": ParameterType.PARAMETER_DOUBLE,
            "value": 0.4,
            "read_only": False,
            "description": "",
        }
        for name, values in (
            ("model.effective_mass", [45.0, 45.0, 48.0, 28.0, 44.0, 100.0]),
            ("model.linear_drag", [100.0, 100.0, 120.0, 15.0, 15.0, 30.0]),
            ("model.quadratic_drag", [0.0] * 6),
            ("model.trim", [0.0] * 6),
            ("model.restoring_stiffness", [0.0] * 6),
        ):
            gains[name] = {
                "type": "double_array",
                "type_id": ParameterType.PARAMETER_DOUBLE_ARRAY,
                "value": values,
                "read_only": False,
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
            for axis in AXES:
                self._signals[f"wrench.allocated.{axis}"] = 0.0
                self._signals[f"wrench.residual.{axis}"] = 0.0
            self._last_odom = time.monotonic()
            self._last_error = self._last_odom
            self._last_allocated_wrench = self._last_odom
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
        now = time.monotonic()
        with self._lock:
            self._last_error = now
            reference_is_fresh = (
                self._last_reference_velocity > 0.0
                and now - self._last_reference_velocity < INNER_TARGET_FRESHNESS_S
            )
            command_is_fresh = {
                group: timestamp > 0.0 and now - timestamp < INNER_TARGET_FRESHNESS_S
                for group, timestamp in self._last_inner_command.items()
            }
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
                if (
                    current is not None
                    and group in ("velocity", "angular_velocity")
                    and not reference_is_fresh
                    and not command_is_fresh[group]
                ):
                    self._signal(f"pid.{group}.{axis}.target", current + value)

    def _pos_setpoint_cb(self, msg: Setpoint) -> None:
        group = "velocity" if msg.velocity else "position"
        now = time.monotonic()
        with self._lock:
            reference_is_fresh = (
                self._last_reference_velocity > 0.0
                and now - self._last_reference_velocity < INNER_TARGET_FRESHNESS_S
            )
            if group == "velocity":
                self._last_inner_command[group] = now
        for axis in "xyz":
            value = getattr(msg.setpoint, axis)
            if group == "velocity":
                self._signal(f"command.{group}.{axis}", value)
            if group != "velocity" or not reference_is_fresh:
                self._signal(f"pid.{group}.{axis}.target", value)

    def _att_setpoint_cb(self, msg: Setpoint) -> None:
        group = "angular_velocity" if msg.velocity else "attitude"
        now = time.monotonic()
        with self._lock:
            reference_is_fresh = (
                self._last_reference_velocity > 0.0
                and now - self._last_reference_velocity < INNER_TARGET_FRESHNESS_S
            )
            if group == "angular_velocity":
                self._last_inner_command[group] = now
        for axis, field in zip("xyz", ("roll", "pitch", "yaw")):
            value = getattr(msg.setpoint, field)
            if group == "angular_velocity":
                self._signal(f"command.{group}.{axis}", value)
            if group != "angular_velocity" or not reference_is_fresh:
                self._signal(f"pid.{group}.{axis}.target", value)

    def _cmd_vel_cb(self, msg: Twist) -> None:
        now = time.monotonic()
        with self._lock:
            self._last_inner_command["velocity"] = now
            self._last_inner_command["angular_velocity"] = now
            reference_is_fresh = (
                self._last_reference_velocity > 0.0
                and now - self._last_reference_velocity < INNER_TARGET_FRESHNESS_S
            )
        for axis in "xyz":
            linear = getattr(msg.linear, axis)
            angular = getattr(msg.angular, axis)
            self._signal(f"command.velocity.{axis}", linear)
            self._signal(f"command.angular_velocity.{axis}", angular)
            if not reference_is_fresh:
                self._signal(f"pid.velocity.{axis}.target", linear)
                self._signal(f"pid.angular_velocity.{axis}.target", angular)

    def _wrench_cb(self, prefix: str, msg: WrenchStamped) -> None:
        values = (
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        )
        for axis, value in zip(("x", "y", "z", "roll", "pitch", "yaw"), values):
            self._signal(f"{prefix}.{axis}", value)
        if prefix == "wrench.allocated":
            with self._lock:
                self._last_allocated_wrench = time.monotonic()

    def _reference_velocity_cb(self, msg: TwistStamped) -> None:
        linear = (
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
        )
        angular = (
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z,
        )
        if any(not math.isfinite(float(value)) for value in linear + angular):
            return
        with self._lock:
            self._last_reference_velocity = time.monotonic()
        for axis, value in zip("xyz", linear):
            self._signal(f"reference.velocity.{axis}", value)
            self._signal(f"pid.velocity.{axis}.target", value)
        for axis, display_axis, value in zip("xyz", ("roll", "pitch", "yaw"), angular):
            self._signal(f"reference.velocity.{display_axis}", value)
            self._signal(f"pid.angular_velocity.{axis}.target", value)

    def _reference_acceleration_cb(self, msg: AccelStamped) -> None:
        values = (
            msg.accel.linear.x,
            msg.accel.linear.y,
            msg.accel.linear.z,
            msg.accel.angular.x,
            msg.accel.angular.y,
            msg.accel.angular.z,
        )
        for axis, value in zip(("x", "y", "z", "roll", "pitch", "yaw"), values):
            self._signal(f"reference.acceleration.{axis}", value)

    def _kill_cb(self, msg: Bool) -> None:
        stop_tracking = False
        stop_characterization = False
        with self._lock:
            was_killed = self._kill
            self._kill = bool(msg.data)
            stop_tracking = self._kill and self._tracking_run is not None
            stop_characterization = self._kill and self._characterization_run is not None
            if was_killed is True and self._kill is False:
                for name in list(self._signals):
                    if name.startswith("pid.") and name.endswith(".target"):
                        self._signals[name] = 0.0
        if stop_tracking:
            self._finish_tracking("aborted", "Kill switch engaged", publish_hold=False)
        if stop_characterization:
            self._finish_characterization(
                "aborted", "Kill switch engaged", publish_hold=False
            )

    def _refresh_source_health(self) -> None:
        counts = {
            "odometry/filtered": self.count_publishers("odometry/filtered"),
            "control/error": self.count_publishers("control/error"),
            "control/thruster_0": self.count_publishers("control/thruster_0"),
            # The dashboard itself owns one publisher on each setpoint topic.
            # A second publisher means a mission or teleop process can fight a
            # tuning routine for command ownership.
            "pos_setpoint": self.count_publishers("pos_setpoint"),
            "att_setpoint": self.count_publishers("att_setpoint"),
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

    def _controller_parameter(self, name: str) -> Any:
        metadata = self._remote_parameters.get(self.controller_fqn, {}).get(name)
        return metadata.get("value") if metadata else None

    def start_characterization(
        self,
        axis: str,
        amplitude: float,
        slow_slope: float,
        fast_slope: float,
        dwell: float,
    ) -> dict:
        config = CharacterizationConfig(
            str(axis),
            float(amplitude),
            float(slow_slope),
            float(fast_slope),
            float(dwell),
        )
        self._refresh_source_health()
        now = time.monotonic()
        required_parameters = (
            "power_limit",
            "model.effective_mass",
            "model.linear_drag",
            "model.quadratic_drag",
            "model.trim",
            "model.restoring_stiffness",
        )
        with self._lock:
            if self._tracking_run is not None:
                raise RuntimeError("stop the tracking routine before characterization")
            if self._characterization_run is not None:
                raise RuntimeError("feedforward characterization is already running")
            missing = [
                name for name in required_parameters
                if self._controller_parameter(name) is None
            ]
            if missing:
                raise RuntimeError(
                    "selected controller does not expose feedforward parameters: "
                    + ", ".join(missing)
                )
            duplicate_sources = self._duplicate_sources()
            if duplicate_sources:
                raise RuntimeError(
                    "multiple ROS control/sim sources detected on "
                    + ", ".join(duplicate_sources)
                )
            telemetry_age = (
                max(now - self._last_odom, now - self._last_error)
                if self._last_odom and self._last_error
                else None
            )
            wrench_age = (
                now - self._last_allocated_wrench
                if self._last_allocated_wrench
                else None
            )
            if self._kill is not False:
                raise RuntimeError("release the kill switch before characterization")
            if telemetry_age is None or telemetry_age > 0.5:
                raise RuntimeError("fresh odometry and controller telemetry are required")
            if wrench_age is None or wrench_age > 0.5:
                raise RuntimeError("allocated-wrench telemetry is required")
            position = self._current_triplet("position")
            attitude = self._current_triplet("attitude")
            if position is None or attitude is None:
                raise RuntimeError("position and attitude measurements are not ready")
            run = {
                "config": config,
                "started_at": now,
                "samples": [],
                "position_hold": position,
                "attitude_hold": attitude,
                "saturation_started": None,
            }
            self._characterization_run = run
            self._characterization_state = {
                "active": True,
                "status": "running",
                "message": "Settling before bias measurement",
                "axis": config.axis,
                "mode": config.mode,
                "phase": "bias hold",
                "elapsed": 0.0,
                "duration": config.duration,
                "progress": 0.0,
                "samples": 0,
                "command": 0.0,
                "result": None,
                "applied": False,
            }

        if config.mode == "velocity":
            self.publish_setpoint("attitude", attitude)
        else:
            self.publish_setpoint("position", position)
        self._publish_characterization_command(config, 0.0)
        return self.characterization_snapshot()

    def _publish_characterization_command(
        self, config: CharacterizationConfig, value: float
    ) -> None:
        command = [0.0, 0.0, 0.0]
        command[AXES.index(config.axis) % 3] = value
        self.publish_setpoint(config.mode, command)

    def _characterization_tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            run = self._characterization_run
            killed = self._kill
            duplicate_sources = self._duplicate_sources()
            telemetry_age = (
                max(now - self._last_odom, now - self._last_error)
                if self._last_odom and self._last_error
                else None
            )
            wrench_age = (
                now - self._last_allocated_wrench
                if self._last_allocated_wrench
                else None
            )
        if run is None:
            return
        if killed:
            self._finish_characterization(
                "aborted", "Kill switch engaged", publish_hold=False
            )
            return
        if duplicate_sources:
            self._finish_characterization(
                "aborted", "Multiple control sources detected", publish_hold=True
            )
            return
        # Stonefish can occasionally pause its sensor publishers for a frame
        # while rendering.  The controller already drops thrust after its own
        # shorter command/odometry timeout; give the dashboard one second to
        # distinguish a real data loss from a single simulator scheduling
        # hitch, then abort and hold.
        if (
            telemetry_age is None
            or telemetry_age > 1.0
            or wrench_age is None
            or wrench_age > 1.0
        ):
            ages = []
            if telemetry_age is not None:
                ages.append(f"pose/error age {telemetry_age:.2f}s")
            if wrench_age is not None:
                ages.append(f"wrench age {wrench_age:.2f}s")
            detail = ", ".join(ages) if ages else "telemetry unavailable"
            self._finish_characterization(
                "aborted",
                f"Characterization telemetry became stale ({detail})",
                publish_hold=True,
            )
            return

        config: CharacterizationConfig = run["config"]
        elapsed = now - run["started_at"]
        if elapsed >= config.duration:
            self._finish_characterization(
                "complete", "Characterization complete", publish_hold=True
            )
            return
        command, phase = config.command_at(elapsed)
        axis_index = AXES.index(config.axis)
        signal_axis = "xyz"[axis_index % 3]
        group = "angular_velocity" if axis_index >= 3 else "velocity"
        configuration_group = "attitude" if axis_index >= 3 else "position"
        configuration_axis = "xyz"[axis_index % 3]
        with self._lock:
            velocity = self._signals.get(f"pid.{group}.{signal_axis}.current")
            wrench = self._signals.get(f"wrench.allocated.{config.axis}")
            residual = self._signals.get(f"wrench.residual.{config.axis}")
            configuration = self._signals.get(
                f"pid.{configuration_group}.{configuration_axis}.current"
            )
            peer_axes = [axis for axis in "xyz" if axis != signal_axis]
            peer_values = [
                abs(float(self._signals.get(f"pid.{group}.{axis}.current", 0.0)))
                for axis in peer_axes
            ]
            cross_velocity = max(peer_values, default=0.0)
            power_limit = float(self._controller_parameter("power_limit") or 1.0)
            thruster_fraction = max(
                (
                    abs(float(self._signals.get(f"thruster.{index}", 0.0)))
                    / max(power_limit, 1e-6)
                    for index in range(8)
                ),
                default=0.0,
            )
            values = (velocity, wrench, residual, configuration)
            if all(value is not None and math.isfinite(float(value)) for value in values):
                run["samples"].append(
                    CharacterizationSample(
                        elapsed,
                        phase,
                        float(velocity),
                        float(wrench),
                        float(configuration),
                        float(residual),
                        thruster_fraction,
                        cross_velocity,
                    )
                )
            if thruster_fraction >= 0.995:
                if run["saturation_started"] is None:
                    run["saturation_started"] = now
            else:
                run["saturation_started"] = None
            saturated_too_long = (
                run["saturation_started"] is not None
                and now - run["saturation_started"] > 0.75
            )
            excessive_tilt = False
            if config.axis in ("roll", "pitch") and configuration is not None:
                tilt_index = 0 if config.axis == "roll" else 1
                start_angle = float(run["attitude_hold"][tilt_index])
                angle_delta = math.atan2(
                    math.sin(float(configuration) - start_angle),
                    math.cos(float(configuration) - start_angle),
                )
                excessive_tilt = abs(angle_delta) > math.radians(20.0)
            sample_count = len(run["samples"])
            self._characterization_state.update(
                phase=phase,
                elapsed=elapsed,
                progress=elapsed / config.duration,
                samples=sample_count,
                command=command,
                message=f"{phase} · collecting measured wrench and motion",
            )
        if saturated_too_long:
            self._finish_characterization(
                "aborted",
                "Thrusters remained saturated; lower amplitude or slope",
                publish_hold=True,
            )
            return
        if excessive_tilt:
            self._finish_characterization(
                "aborted",
                "Roll/pitch moved more than 20 degrees; lower speed or dwell",
                publish_hold=True,
            )
            return
        self._publish_characterization_command(config, command)

    def _finish_characterization(
        self, status: str, message: str, publish_hold: bool
    ) -> None:
        with self._lock:
            run = self._characterization_run
            if run is None:
                return
            self._characterization_run = None
            config: CharacterizationConfig = run["config"]
            samples = list(run["samples"])
        result = None
        if status == "complete":
            try:
                axis_index = AXES.index(config.axis)
                baseline = {}
                for result_name, parameter_name in (
                    ("effective_mass", "model.effective_mass"),
                    ("linear_drag", "model.linear_drag"),
                    ("quadratic_drag", "model.quadratic_drag"),
                    ("trim", "model.trim"),
                    ("restoring_stiffness", "model.restoring_stiffness"),
                ):
                    values = self._controller_parameter(parameter_name)
                    if isinstance(values, list) and len(values) > axis_index:
                        baseline[result_name] = float(values[axis_index])
                result = fit_feedforward(samples, config, baseline)
                if result["passed"]:
                    message = "Fit passed quality checks; review before applying"
                else:
                    message = "Fit rejected: " + "; ".join(result["failures"])
                    status = "rejected"
            except Exception as exc:  # noqa: BLE001
                status = "rejected"
                message = f"Fit rejected: {exc}"
        with self._lock:
            self._characterization_state = {
                "active": False,
                "status": status,
                "message": message,
                "axis": config.axis,
                "mode": config.mode,
                "phase": "complete" if status in ("complete", "rejected") else status,
                "elapsed": min(time.monotonic() - run["started_at"], config.duration),
                "duration": config.duration,
                "progress": 1.0 if status in ("complete", "rejected") else 0.0,
                "samples": len(samples),
                "command": 0.0,
                "result": result,
                "applied": False,
            }
        if publish_hold:
            position = self._current_triplet("position") or run["position_hold"]
            attitude = self._current_triplet("attitude") or run["attitude_hold"]
            self.publish_setpoint("position", position)
            self.publish_setpoint("attitude", attitude)

    def stop_characterization(self) -> dict:
        with self._lock:
            active = self._characterization_run is not None
        if active:
            self._finish_characterization(
                "stopped", "Stopped by operator", publish_hold=True
            )
        return self.characterization_snapshot()

    def characterization_changes(self) -> list[dict]:
        with self._lock:
            state = dict(self._characterization_state)
            result = state.get("result")
            if state.get("active") or not result or not result.get("passed"):
                raise RuntimeError("no quality-approved characterization result is available")
            axis = state["axis"]
            coefficients = result["coefficients"]
            axis_index = AXES.index(axis)
            names = (
                "model.effective_mass",
                "model.linear_drag",
                "model.quadratic_drag",
                "model.trim",
            )
            coefficient_names = (
                "effective_mass",
                "linear_drag",
                "quadratic_drag",
                "trim",
            )
            changes = []
            for name, coefficient_name in zip(names, coefficient_names):
                values = list(self._controller_parameter(name) or [])
                if len(values) != 6:
                    raise RuntimeError(f"controller parameter {name} is unavailable")
                values[axis_index] = float(coefficients[coefficient_name])
                changes.append(
                    {"node": self.controller_fqn, "name": name, "value": values}
                )
            if axis in ("roll", "pitch"):
                name = "model.restoring_stiffness"
                values = list(self._controller_parameter(name) or [])
                if len(values) != 6:
                    raise RuntimeError(f"controller parameter {name} is unavailable")
                values[axis_index] = float(coefficients["restoring_stiffness"])
                changes.append(
                    {"node": self.controller_fqn, "name": name, "value": values}
                )
            return changes

    def mark_characterization_applied(self) -> None:
        with self._lock:
            if self._characterization_state.get("result"):
                self._characterization_state["applied"] = True
                self._characterization_state["message"] = (
                    "Estimated feedforward constants applied and verified at runtime"
                )

    def characterization_snapshot(self) -> dict:
        with self._lock:
            return dict(self._characterization_state)

    def start_tracking(
        self,
        experiment: str,
        mode: str,
        axis: str,
        amplitude: float,
        ramp_time: float,
        hold_time: float,
        cycles: int,
        max_tracking_error: float | None = None,
    ) -> dict:
        profile = TrackingProfile.from_values(
            experiment, mode, axis, amplitude, ramp_time, hold_time, cycles
        )
        path_error_limit = None
        if profile.path_experiment:
            path_error_limit = 0.12 if max_tracking_error is None else float(max_tracking_error)
            if not math.isfinite(path_error_limit) or not 0.02 <= path_error_limit <= 1.0:
                raise ValueError("maximum path lag must be between 0.02 and 1 metre")
        self._refresh_source_health()
        now = time.monotonic()
        with self._lock:
            if self._tracking_run is not None:
                raise RuntimeError("a tracking routine is already running")
            if self._characterization_run is not None:
                raise RuntimeError("feedforward characterization is already running")
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
            if profile.path_experiment:
                manual_advance = profile.experiment == "square_test"
                run.update(
                    path_elapsed=0.0,
                    last_tick=now,
                    max_tracking_error=path_error_limit,
                    manual_advance=manual_advance,
                    waypoint_index=0,
                    waypoint_count=4 if manual_advance else 0,
                    timeout_after=(
                        900.0
                        if manual_advance
                        else min(
                            max(profile.duration * 4.0, profile.duration + 120.0),
                            profile.duration + 600.0,
                        )
                    ),
                )
                run["path_preview"] = [
                    [position[index] + offset[index] for index in range(3)]
                    for offset in profile.preview()
                ]
            self._tracking_run = run
            self._tracking_state = {
                "active": True,
                "status": "running",
                "message": "Tracking profile running",
                "elapsed": 0.0,
                "progress": 0.0,
                "path_preview": run.get("path_preview", []),
                "current_reference": position if profile.path_experiment else None,
                **(
                    {
                        "max_tracking_error": path_error_limit,
                        "tracking_error": 0.0,
                        "reference_rate": 1.0,
                        "reference_limited": False,
                        "wall_elapsed": 0.0,
                        "manual_advance": manual_advance,
                        "waypoint_index": 0,
                        "waypoint_count": run["waypoint_count"],
                        "ready_for_next": False,
                        "vehicle_speed": 0.0,
                    }
                    if profile.path_experiment
                    else {}
                ),
                **profile.public_state(),
            }

        # Hold the companion loop once; only the selected command needs to be
        # streamed while its reference changes.
        if mode in ("position", "velocity"):
            self.publish_setpoint("attitude", attitude)
        else:
            self.publish_setpoint("position", position)
        self._publish_tracking_reference(run, 0.0)
        if profile.experiment == "square_test":
            with self._lock:
                if self._tracking_run is run:
                    self._tracking_state["message"] = (
                        "Settling at the measured start"
                    )
        return self.tracking_snapshot()

    def stop_tracking(self) -> dict:
        with self._lock:
            active = self._tracking_run is not None
        if active:
            self._finish_tracking("stopped", "Stopped by operator", publish_hold=True)
        return self.tracking_snapshot()

    def stop_all(self) -> dict:
        """Cancel dashboard routines and clear every dashboard motion command."""
        tracking = self.stop_tracking()
        characterization = self.stop_characterization()
        now = time.monotonic()
        with self._lock:
            telemetry_fresh = bool(
                self._last_odom
                and self._last_error
                and max(now - self._last_odom, now - self._last_error) < 1.0
            )
            armed = self._kill is False

        position = self._current_triplet("position") if telemetry_fresh else None
        attitude = self._current_triplet("attitude") if telemetry_fresh else None
        if armed and position is not None and attitude is not None:
            self.publish_setpoint("position", position)
            self.publish_setpoint("attitude", attitude)
            holding = "pose"
            message = "All dashboard routines stopped; holding measured pose"
        else:
            # Clearing both rate modes is safe even while killed or when pose
            # telemetry is stale.  It also prevents an old manual rate command
            # from resuming when telemetry or the kill switch returns.
            self.publish_setpoint("velocity", [0.0, 0.0, 0.0])
            self.publish_setpoint("angular_velocity", [0.0, 0.0, 0.0])
            holding = "zero_rate"
            message = "All dashboard routines stopped; commanded rates are zero"
        return {
            "tracking": tracking,
            "characterization": characterization,
            "holding": holding,
            "message": message,
        }

    def advance_tracking(self) -> dict:
        """Advance an operator-paced waypoint routine by one target."""
        with self._lock:
            run = self._tracking_run
            if run is None or not run.get("manual_advance"):
                raise RuntimeError("no operator-paced tracking routine is running")
            if not self._tracking_state.get("ready_for_next", False):
                raise RuntimeError("wait for the vehicle to reach and settle at this corner")
            waypoint_index = int(run["waypoint_index"])
            waypoint_count = int(run["waypoint_count"])
            if waypoint_index >= waypoint_count:
                raise RuntimeError("the final waypoint is already active")
            waypoint_index += 1
            run["waypoint_index"] = waypoint_index
            run["path_elapsed"] = (
                run["profile"].ramp_time * waypoint_index / waypoint_count
            )
            run["last_tick"] = time.monotonic()
            destination = "home" if waypoint_index == waypoint_count else f"corner {waypoint_index}"
            self._tracking_state.update(
                waypoint_index=waypoint_index,
                ready_for_next=False,
                elapsed=run["path_elapsed"],
                progress=waypoint_index / waypoint_count,
                message=f"Moving to {destination}",
            )
            elapsed = run["path_elapsed"]

        self._publish_tracking_reference(run, elapsed)
        return self.tracking_snapshot()

    def _publish_tracking_reference(self, run: dict, elapsed: float) -> None:
        profile: TrackingProfile = run["profile"]
        if profile.path_experiment:
            offset = profile.vector_at(elapsed)
            command = [run["position_hold"][index] + offset[index] for index in range(3)]
            self.publish_setpoint("position", command)
            with self._lock:
                if self._tracking_run is run:
                    self._tracking_state["current_reference"] = command
            return

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
        now = time.monotonic()
        with self._lock:
            run = self._tracking_run
            killed = self._kill
            duplicate_sources = self._duplicate_sources()
            measurement_age = (
                max(
                    now - self._last_odom,
                    now - self._last_error,
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

        profile: TrackingProfile = run["profile"]
        if profile.path_experiment:
            if run.get("manual_advance"):
                self._tracking_waypoint_tick(run, now)
                return
            self._tracking_path_tick(run, now)
            return

        elapsed = now - run["started_at"]
        if elapsed >= profile.duration:
            self._finish_tracking("complete", "Tracking routine complete", publish_hold=True)
            return
        self._publish_tracking_reference(run, elapsed)
        with self._lock:
            if self._tracking_run is run:
                self._tracking_state["elapsed"] = elapsed
                self._tracking_state["progress"] = elapsed / profile.duration

    def _tracking_waypoint_tick(self, run: dict, now: float) -> None:
        """Hold a square corner until the operator explicitly advances it."""
        profile: TrackingProfile = run["profile"]
        with self._lock:
            if self._tracking_run is not run:
                return
            current = [
                float(self._signals[f"pid.position.{axis}.current"])
                for axis in "xyz"
            ]
            velocity = [
                float(self._signals[f"pid.velocity.{axis}.current"])
                for axis in "xyz"
            ]
            reference = list(self._tracking_state["current_reference"])
            ready = bool(self._tracking_state.get("ready_for_next", False))
            waypoint_index = int(run["waypoint_index"])
            waypoint_count = int(run["waypoint_count"])

        selected = ["xyz".index(axis) for axis in profile.axis]
        tracking_error = math.sqrt(
            sum((current[index] - reference[index]) ** 2 for index in selected)
        )
        vehicle_speed = math.sqrt(sum(velocity[index] ** 2 for index in selected))
        arrived = (
            tracking_error <= float(run["max_tracking_error"])
            and vehicle_speed <= 0.05
        )
        wall_elapsed = max(0.0, now - run["started_at"])
        if wall_elapsed >= run["timeout_after"]:
            self._finish_tracking(
                "aborted", "Waypoint test timed out", publish_hold=True
            )
            return
        if waypoint_index == waypoint_count and arrived:
            self._finish_tracking(
                "complete", "Square Test complete", publish_hold=True
            )
            return

        destination = "start" if waypoint_index == 0 else (
            "home" if waypoint_index == waypoint_count else f"corner {waypoint_index}"
        )
        if arrived and not ready:
            ready = True
            message = f"{destination.capitalize()} reached — press Next corner"
        elif ready:
            message = f"Holding {destination} — press Next corner"
        else:
            message = f"Moving to {destination}"
        with self._lock:
            if self._tracking_run is run:
                self._tracking_state.update(
                    elapsed=float(run["path_elapsed"]),
                    progress=waypoint_index / waypoint_count,
                    wall_elapsed=wall_elapsed,
                    tracking_error=tracking_error,
                    vehicle_speed=vehicle_speed,
                    reference_rate=0.0,
                    reference_limited=False,
                    ready_for_next=ready,
                    message=message,
                )

    def _tracking_path_tick(self, run: dict, now: float) -> None:
        """Advance a path reference only as quickly as the AUV can follow it.

        Path validation is different from an inner-loop step response. Advancing
        a geometric path strictly from wall time makes it impossible to tell a
        bad follower from a reference that simply exceeds the vehicle's
        acceleration and drag limits. This governor leaves the requested path
        unchanged while slowing its virtual clock as planar tracking lag grows.
        """
        profile: TrackingProfile = run["profile"]
        with self._lock:
            if self._tracking_run is not run:
                return
            current = [
                float(self._signals[f"pid.position.{axis}.current"])
                for axis in "xyz"
            ]
            reference = self._tracking_state.get("current_reference")
            path_elapsed = float(run["path_elapsed"])
            last_tick = float(run["last_tick"])
            max_error = float(run["max_tracking_error"])

        if not isinstance(reference, list) or len(reference) != 3:
            offset = profile.vector_at(path_elapsed)
            reference = [
                run["position_hold"][index] + offset[index]
                for index in range(3)
            ]

        selected = ["xyz".index(axis) for axis in profile.axis]
        tracking_error = math.sqrt(
            sum((current[index] - float(reference[index])) ** 2 for index in selected)
        )
        # A circle needs a small, continuously moving lookahead. Slow it from
        # the first measurable lag so inertia cannot make the vehicle cut most
        # of the loop. Other continuous paths retain a larger lookahead.
        slowdown_error = 0.0 if profile.experiment == "follower_pid" else max_error * 0.5
        if tracking_error <= slowdown_error:
            reference_rate = 1.0
        elif tracking_error >= max_error:
            reference_rate = 0.0
        else:
            reference_rate = (max_error - tracking_error) / (
                max_error - slowdown_error
            )

        # Clamp timer jitter so a delayed callback cannot jump the target.
        delta = max(0.0, min(now - last_tick, 0.2))
        path_elapsed = min(path_elapsed + delta * reference_rate, profile.duration)
        wall_elapsed = max(0.0, now - run["started_at"])
        with self._lock:
            if self._tracking_run is not run:
                return
            run["path_elapsed"] = path_elapsed
            run["last_tick"] = now

        if wall_elapsed >= run["timeout_after"]:
            self._finish_tracking(
                "aborted",
                "Path timed out while waiting for the vehicle to catch up",
                publish_hold=True,
            )
            return
        if path_elapsed >= profile.duration:
            self._finish_tracking("complete", "Tracking routine complete", publish_hold=True)
            return

        self._publish_tracking_reference(run, path_elapsed)
        limited = reference_rate < 0.999
        if reference_rate < 0.05:
            message = "Target paused while the vehicle catches up"
        elif limited:
            message = "Target slowed for tracking lag"
        else:
            message = "Tracking profile running"
        with self._lock:
            if self._tracking_run is run:
                self._tracking_state.update(
                    elapsed=path_elapsed,
                    progress=path_elapsed / profile.duration,
                    wall_elapsed=wall_elapsed,
                    tracking_error=tracking_error,
                    reference_rate=reference_rate,
                    reference_limited=limited,
                    message=message,
                )

    def _finish_tracking(self, status: str, message: str, publish_hold: bool) -> None:
        with self._lock:
            run = self._tracking_run
            if run is None:
                return
            self._tracking_run = None
            profile: TrackingProfile = run["profile"]
            wall_elapsed = max(0.0, time.monotonic() - run["started_at"])
            elapsed = (
                profile.duration
                if status == "complete" and run.get("manual_advance")
                else min(float(run["path_elapsed"]), profile.duration)
                if profile.path_experiment
                else min(wall_elapsed, profile.duration)
            )
            self._tracking_state = {
                "active": False,
                "status": status,
                "message": message,
                "elapsed": elapsed,
                "progress": min(elapsed / profile.duration, 1.0),
                "path_preview": run.get("path_preview", []),
                "current_reference": None,
                **(
                    {
                        "max_tracking_error": run["max_tracking_error"],
                        "tracking_error": self._tracking_state.get("tracking_error", 0.0),
                        "reference_rate": 0.0,
                        "reference_limited": False,
                        "wall_elapsed": wall_elapsed,
                        "manual_advance": run.get("manual_advance", False),
                        "waypoint_index": run.get("waypoint_index", 0),
                        "waypoint_count": run.get("waypoint_count", 0),
                        "ready_for_next": False,
                        "vehicle_speed": self._tracking_state.get("vehicle_speed", 0.0),
                    }
                    if profile.path_experiment
                    else {}
                ),
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
                "characterization": dict(self._characterization_state),
                "nodes": sorted(self._known_nodes | set(self._remote_parameters)),
            }
            if include_parameters:
                result["parameters"] = {
                    node: {name: dict(metadata) for name, metadata in params.items()}
                    for node, params in self._remote_parameters.items()
                }
            return result
