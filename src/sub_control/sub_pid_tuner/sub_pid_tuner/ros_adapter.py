"""The dashboard's ROS node: parameters of any node, sub_control's telemetry,
and the commands and tuning routines it sends sub_control.

Telemetry is put in the frames of ControllerStatus so that target, reference
and measurement compare directly: translation in odom ENU, rotation as ZYX
Euler angles and their rates. The wrench, integral and disturbance stay body
FLU, as the controller reports them.
"""

from __future__ import annotations

import math
import threading
import time
from concurrent.futures import Future
from typing import Any

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_srvs.srv import Trigger
from sub_control_interfaces.msg import ControllerStatus, MotionSetpoint

from . import routines
from .routines import AXES, HOLD, KEEP, PITCH, POSITION, ROLL, VELOCITY, Segment, X, Y, Z

MODE_NAMES = ("KEEP", "POSITION", "VELOCITY", "HOLD", "EFFORT")
NUM_THRUSTERS = 8

# [s] odometry and control/status older than this stop a routine.
TELEMETRY_TIMEOUT = 1.0
# [s] a service request not answered by then is given up on: its node died.
SERVICE_TIMEOUT = 5.0
# [s] the timeout streamed VELOCITY targets carry: if the dashboard stops
# sending them, sub_control brings those axes to a HOLD this soon.
STREAM_TIMEOUT = 0.5
# [s] longest timeout a manual VELOCITY or EFFORT command may latch for.
MAX_COMMAND_TIMEOUT = 30.0
ROUTINE_RATE = 20.0  # [Hz]

# Topics whose publishers other than these would fight the dashboard for the
# vehicle (a mission, teleop, identify_model) or mean two stacks are running.
EXPECTED_PUBLISHERS = {
    "motion_setpoint": 1,  # the dashboard's own
    "cmd_vel": 0,
    "control/status": 1,
    "odometry/filtered": 1,
}

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

VALUE_FIELDS = {
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


def parameter_value(value: Any) -> Any:
    field = VALUE_FIELDS.get(value.type)
    if field is None:
        return None
    result = getattr(value, field)
    if value.type == ParameterType.PARAMETER_BYTE_ARRAY:
        return list(b"".join(result))  # one-byte bytes objects, which JSON cannot carry
    return list(result) if value.type > ParameterType.PARAMETER_BYTE_ARRAY else result


def _integer(value: Any) -> int:
    # int() alone would turn 2.5 into 2 without a word.
    if isinstance(value, float) and not value.is_integer():
        raise ValueError(f"{value} is not an integer")
    return int(value)


def coerce_parameter(value: Any, type_id: int) -> Any:
    """A value from the browser as the parameter's declared type."""
    if type_id == ParameterType.PARAMETER_BOOL:
        if isinstance(value, bool):
            return value
        if str(value).lower() in ("true", "1"):
            return True
        if str(value).lower() in ("false", "0"):
            return False
        raise ValueError("expected true or false")
    if type_id == ParameterType.PARAMETER_INTEGER:
        return _integer(value)
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
        if type_id == ParameterType.PARAMETER_BOOL_ARRAY:
            return [coerce_parameter(item, ParameterType.PARAMETER_BOOL) for item in value]
        if type_id == ParameterType.PARAMETER_STRING_ARRAY:
            return [str(item) for item in value]
        if type_id == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return [coerce_parameter(item, ParameterType.PARAMETER_DOUBLE) for item in value]
        converted = [_integer(item) for item in value]
        if type_id == ParameterType.PARAMETER_BYTE_ARRAY:
            if any(not 0 <= item <= 255 for item in converted):
                raise ValueError("byte array values must be in [0, 255]")
            return [bytes([item]) for item in converted]  # what rclpy's Parameter takes
        return converted
    raise ValueError("unsupported or unset parameter type")


def rpy_from_quaternion(q) -> tuple[float, float, float]:
    """ZYX Euler angles [roll, pitch, yaw] of a quaternion, as sub_control's rpy_from_quaternion()."""
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def rotate(q, v) -> tuple[float, float, float]:
    """Vector v rotated by quaternion q (body to odom)."""
    # v + 2 w (u x v) + 2 u x (u x v), with u the quaternion's vector part.
    tx = 2 * (q.y * v.z - q.z * v.y)
    ty = 2 * (q.z * v.x - q.x * v.z)
    tz = 2 * (q.x * v.y - q.y * v.x)
    return (
        v.x + q.w * tx + q.y * tz - q.z * ty,
        v.y + q.w * ty + q.z * tx - q.x * tz,
        v.z + q.w * tz + q.x * ty - q.y * tx,
    )


def euler_rates(roll: float, pitch: float, body) -> tuple[float, float, float]:
    """ZYX Euler angle rates from body angular velocity [p, q, r]."""
    p, q, r = body.x, body.y, body.z
    cos_pitch = math.copysign(max(abs(math.cos(pitch)), 1e-6), math.cos(pitch))
    lateral = q * math.sin(roll) + r * math.cos(roll)
    return (
        p + lateral * math.tan(pitch),
        q * math.cos(roll) - r * math.sin(roll),
        lateral / cos_pitch,
    )


def wrap_angle(angle: float) -> float:
    return math.remainder(angle, 2 * math.pi)


class RosAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__("dashboard", **kwargs)
        # The node whose parameters the gains profile holds and whose
        # reset_pose service "Zero pose" calls; relative to this namespace.
        self.declare_parameter("controller", "sub_control")
        self.declare_parameter("profile", "")  # gains file to save to; empty disables saving
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8080)

        controller = self.get_parameter("controller").value
        if not controller.startswith("/"):
            controller = f"{self.get_namespace().rstrip('/')}/{controller}"
        self.controller_fqn = controller

        self._lock = threading.RLock()
        # Not Node._parameters: rclpy keeps this node's own parameters there.
        self._remote_parameters: dict[str, dict[str, dict]] = {}
        # Counts changes to _remote_parameters, so the server can send them when they change.
        self.parameters_version = 0
        # One per node, kept while the dashboard runs: each holds six service clients.
        self._parameter_clients: dict[str, AsyncParameterClient] = {}
        # Nodes whose parameters are being read, and since when [time.monotonic()].
        self._pending_nodes: dict[str, float] = {}
        self._known_nodes: set[str] = set()
        self._watched_nodes: set[str] = {self.controller_fqn}

        self._odometry: Odometry | None = None
        self._status: ControllerStatus | None = None
        self._altitude: float | None = None
        self._odometry_time = 0.0
        self._status_time = 0.0
        self._publisher_counts: dict[str, int] = {}

        self._run: dict | None = None
        self._routine_state: dict = {"active": False, "status": "idle", "message": "Ready"}

        self.create_subscription(Odometry, "odometry/filtered", self._odometry_callback, 10)
        self.create_subscription(ControllerStatus, "control/status", self._status_callback, 10)
        self.create_subscription(
            Range, "altitude", self._altitude_callback, qos_profile_sensor_data
        )
        self._publisher = self.create_publisher(MotionSetpoint, "motion_setpoint", 10)
        self._reset_pose = self.create_client(Trigger, f"{self.controller_fqn}/reset_pose")

        self.create_timer(2.0, self.refresh_parameters)
        self.create_timer(1.0, self._count_publishers)
        # Wall time, so a routine still notices when the sim's clock stops.
        self.create_timer(
            1.0 / ROUTINE_RATE, self._routine_tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    # Telemetry

    def _odometry_callback(self, msg: Odometry) -> None:
        with self._lock:
            self._odometry = msg
            self._odometry_time = time.monotonic()

    def _status_callback(self, msg: ControllerStatus) -> None:
        stop = False
        with self._lock:
            self._status = msg
            self._status_time = time.monotonic()
            stop = msg.killed and self._run is not None
        if stop:
            self._finish("aborted", "Killed", hold=False)

    def _altitude_callback(self, msg: Range) -> None:
        if math.isfinite(msg.range) and msg.min_range <= msg.range <= msg.max_range:
            with self._lock:
                self._altitude = msg.range

    def _measured(self) -> tuple[list[float], list[float]] | None:
        """Measured pose and its rates, in the frames of ControllerStatus's reference."""
        msg = self._odometry
        if msg is None:
            return None
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll, pitch, yaw = rpy_from_quaternion(q)
        pose = [position.x, position.y, position.z, roll, pitch, yaw]
        # The EKF's twist is body FLU (REP-105).
        twist = msg.twist.twist
        velocity = [*rotate(q, twist.linear), *euler_rates(roll, pitch, twist.angular)]
        if self._status is not None and self._status.altitude and self._altitude is not None:
            pose[Z] = self._altitude  # the reference is in altitude coordinates
        return pose, velocity

    def _signals(self) -> dict[str, float]:
        signals: dict[str, float] = {}
        measured = self._measured()
        status = self._status
        if measured is not None:
            for index, axis in enumerate(AXES):
                signals[f"{axis}.measured"] = measured[0][index]
                signals[f"{axis}.velocity"] = measured[1][index]
        if status is not None:
            for index, axis in enumerate(AXES):
                signals[f"{axis}.target"] = status.target[index]
                signals[f"{axis}.reference"] = status.reference_position[index]
                signals[f"{axis}.reference_velocity"] = status.reference_velocity[index]
                signals[f"{axis}.reference_acceleration"] = status.reference_acceleration[index]
                signals[f"{axis}.wrench_command"] = status.wrench_command[index]
                signals[f"{axis}.wrench_achieved"] = status.wrench_achieved[index]
                signals[f"{axis}.integral"] = status.integral[index]
                signals[f"{axis}.disturbance"] = status.disturbance[index]
                if measured is not None:
                    error = status.reference_position[index] - measured[0][index]
                    signals[f"{axis}.tracking_error"] = wrap_angle(error) if index > Z else error
            for thruster in range(NUM_THRUSTERS):
                signals[f"thruster_{thruster}.command"] = status.thruster_command[thruster]
                signals[f"thruster_{thruster}.force"] = status.thruster_force[thruster]
            signals["state_age"] = min(status.state_age, 1e3)
        if self._altitude is not None:
            signals["altitude"] = self._altitude
        return {name: float(value) for name, value in signals.items() if math.isfinite(value)}

    def _controller_state(self) -> dict | None:
        status = self._status
        if status is None:
            return None
        return {
            "mode": [
                MODE_NAMES[mode] if mode < len(MODE_NAMES) else str(mode) for mode in status.mode
            ],
            "altitude": bool(status.altitude),
            "killed": bool(status.killed),
            "failsafe": bool(status.failsafe),
            "dvl_ok": bool(status.dvl_ok),
            "depth_ok": bool(status.depth_ok),
            "altitude_ok": bool(status.altitude_ok),
            "axis_controllable": [bool(value) for value in status.axis_controllable],
            "thruster_saturated": [bool(value) for value in status.thruster_saturated],
            "thruster_health": [float(value) for value in status.thruster_health],
        }

    def _count_publishers(self) -> None:
        counts = {topic: self.count_publishers(topic) for topic in EXPECTED_PUBLISHERS}
        with self._lock:
            self._publisher_counts = counts

    def _conflicts(self) -> list[str]:
        return sorted(
            topic
            for topic, count in self._publisher_counts.items()
            if count > EXPECTED_PUBLISHERS[topic]
        )

    def _problem(self) -> str | None:
        """Why a routine cannot run now, or None."""
        now = time.monotonic()
        conflicts = self._conflicts()
        if conflicts:
            return (
                "Other publishers on "
                + ", ".join(conflicts)
                + "; stop the mission, teleop or second stack"
            )
        if (
            now - self._odometry_time > TELEMETRY_TIMEOUT
            or now - self._status_time > TELEMETRY_TIMEOUT
        ):
            return "No live odometry/filtered and control/status"
        if self._status.killed:
            return "Killed"
        if self._status.failsafe:
            return "sub_control is in failsafe (no state estimate)"
        return None

    # Parameters

    def refresh_parameters(self) -> None:
        own_name = self.get_fully_qualified_name()
        active = set()
        for name, namespace in self.get_node_names_and_namespaces():
            full_name = f"{namespace.rstrip('/')}/{name}"
            if full_name != own_name:
                active.add(full_name)
        now = time.monotonic()
        ready = []
        with self._lock:
            self._known_nodes = active
            for stale in set(self._remote_parameters) - active:
                self._remote_parameters.pop(stale, None)
                self.parameters_version += 1
            for node in sorted(active & self._watched_nodes):
                # A request that went unanswered (its node died) does not block the node for good.
                if now - self._pending_nodes.get(node, -math.inf) < SERVICE_TIMEOUT:
                    continue
                client = self._parameter_clients.get(node)
                if client is None:
                    client = self._parameter_clients[node] = AsyncParameterClient(self, node)
                if client.services_are_ready():
                    self._pending_nodes[node] = now
                    ready.append((node, client))
        for node, client in ready:
            client.list_parameters([], 10).add_done_callback(
                lambda done, node=node, client=client: self._listed(node, client, done)
            )

    def request_parameters(self, node: str) -> None:
        """Start watching `node`'s parameters (the browser selected it)."""
        if not node.startswith("/") or node == self.get_fully_qualified_name():
            raise ValueError(f"invalid ROS node: {node}")
        with self._lock:
            self._watched_nodes.add(node)
        self.refresh_parameters()

    def _listed(self, node: str, client: AsyncParameterClient, future) -> None:
        try:
            names = list(future.result().result.names)
            values = client.get_parameters(names)
            descriptors = client.describe_parameters(names)

            def finish(_done) -> None:
                if not (values.done() and descriptors.done()):
                    return
                try:
                    result = {}
                    for name, value, descriptor in zip(
                        names,
                        values.result().values,
                        descriptors.result().descriptors,
                        strict=True,
                    ):
                        result[name] = {
                            "type": TYPE_NAMES.get(value.type, "unsupported"),
                            "type_id": int(value.type),
                            "value": parameter_value(value),
                            # A parameter that is not set has no type to set it as.
                            "read_only": bool(descriptor.read_only) or value.type not in TYPE_NAMES,
                            "description": descriptor.description,
                        }
                    with self._lock:
                        if self._remote_parameters.get(node) != result:
                            self._remote_parameters[node] = result
                            self.parameters_version += 1
                except Exception as exc:
                    self.get_logger().debug(f"parameter discovery failed for {node}: {exc}")
                finally:
                    with self._lock:
                        self._pending_nodes.pop(node, None)

            values.add_done_callback(finish)
            descriptors.add_done_callback(finish)
        except Exception as exc:
            self.get_logger().debug(f"parameter discovery failed for {node}: {exc}")
            with self._lock:
                self._pending_nodes.pop(node, None)

    def set_parameters(self, node: str, changes: list[dict]) -> Future:
        """Set `changes` ([{name, value}]) on `node` atomically and read them back."""
        result: Future = Future()
        try:
            if not changes:
                raise ValueError("no parameter changes were given")
            prepared = []
            with self._lock:
                known = self._remote_parameters.get(node, {})
                for change in changes:
                    name = str(change.get("name", ""))
                    metadata = known.get(name)
                    if metadata is None:
                        raise ValueError(f"unknown parameter: {node} {name}")
                    if metadata["read_only"]:
                        raise ValueError(f"{name} is read-only on {node}")
                    prepared.append(
                        (name, metadata, coerce_parameter(change.get("value"), metadata["type_id"]))
                    )
            client = self._parameter_clients.get(node)
            if client is None or not client.services_are_ready():
                raise RuntimeError(f"parameter services of {node} are not available")
        except Exception as exc:
            result.set_exception(exc)
            return result

        names = [name for name, _, _ in prepared]

        def verified(done) -> None:
            try:
                values = list(done.result().values)
                if len(values) != len(prepared):
                    raise RuntimeError("parameter readback was incomplete")
                applied = []
                with self._lock:
                    for (name, metadata, _), value in zip(prepared, values, strict=True):
                        metadata["value"] = parameter_value(value)
                        self.parameters_version += 1
                        applied.append({"node": node, "name": name, "value": metadata["value"]})
                result.set_result(applied)
            except Exception as exc:
                result.set_exception(exc)

        def applied(done) -> None:
            try:
                response = done.result().result
                if not response.successful:
                    raise RuntimeError(response.reason or "parameter update rejected")
                client.get_parameters(names).add_done_callback(verified)
            except Exception as exc:
                result.set_exception(exc)

        client.set_parameters_atomically(
            [
                Parameter(name, Parameter.Type(metadata["type_id"]), value)
                for name, metadata, value in prepared
            ]
        ).add_done_callback(applied)
        return result

    def read_parameters(self, node: str) -> Future:
        """Fresh values of `node`'s writable parameters, {name: value}."""
        result: Future = Future()
        with self._lock:
            names = [
                name
                for name, metadata in self._remote_parameters.get(node, {}).items()
                if not metadata["read_only"]
            ]
        client = self._parameter_clients.get(node)
        if not names or client is None or not client.services_are_ready():
            result.set_exception(RuntimeError(f"parameters of {node} are not available"))
            return result

        def read(done) -> None:
            try:
                values = [parameter_value(value) for value in done.result().values]
                result.set_result(dict(zip(names, values, strict=True)))
            except Exception as exc:
                result.set_exception(exc)

        client.get_parameters(names).add_done_callback(read)
        return result

    # Commands

    def _send(self, segment: Segment, timeout: float = 0.0) -> None:
        msg = MotionSetpoint()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = list(segment.mode)
        msg.position = list(segment.position)
        msg.velocity = list(segment.velocity)
        # Routines work in odom axes, like the telemetry they are judged by.
        msg.velocity_frame = MotionSetpoint.WORLD
        msg.timeout = timeout
        self._publisher.publish(msg)

    def _stop_segment(self, roll: float = 0.0, pitch: float = 0.0) -> Segment:
        """Come to a smooth stop and hold there, with roll and pitch at the given angles (level)."""
        position = [0.0] * 6
        position[ROLL], position[PITCH] = roll, pitch
        return Segment(
            0.0, "hold here", (HOLD, HOLD, HOLD, POSITION, POSITION, HOLD), tuple(position)
        )

    def publish_command(
        self,
        modes: list[str],
        values: list[float],
        altitude: bool = False,
        frame: str = "heading",
        timeout: float = 0.0,
    ) -> dict:
        """One MotionSetpoint from the manual command form."""
        with self._lock:
            if self._run is not None:
                raise RuntimeError("stop the routine first")
        if len(modes) != 6 or len(values) != 6:
            raise ValueError("a command has a mode and a value for each of the six axes")
        names = [str(mode).upper() for mode in modes]
        if any(name not in MODE_NAMES for name in names):
            raise ValueError("modes are KEEP, POSITION, VELOCITY, HOLD or EFFORT")
        msg = MotionSetpoint()
        msg.mode = [MODE_NAMES.index(name) for name in names]
        if msg.mode[X] != msg.mode[Y]:
            raise ValueError("x and y must share a mode")
        # Each axis's commanded value; KEEP and HOLD take none, and their field is ignored.
        commanded: list[float | None] = [None] * 6
        for axis, (mode, value) in enumerate(zip(msg.mode, values, strict=True)):
            if mode in (KEEP, HOLD):
                continue
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"{AXES[axis]} needs a finite value")
            commanded[axis] = value
            if mode == POSITION:
                msg.position[axis] = value
            elif mode == VELOCITY:
                msg.velocity[axis] = value
            else:
                msg.effort[axis] = value
        streamed = any(mode in (VELOCITY, MotionSetpoint.EFFORT) for mode in msg.mode)
        timeout = float(timeout)
        if streamed and not 0.0 < timeout <= MAX_COMMAND_TIMEOUT:
            raise ValueError(
                f"velocity and effort commands need a timeout of up to {MAX_COMMAND_TIMEOUT:g} s"
            )
        msg.timeout = timeout if streamed else 0.0
        msg.velocity_frame = MotionSetpoint.WORLD if frame == "world" else MotionSetpoint.HEADING
        msg.altitude = bool(altitude) and msg.mode[Z] == POSITION
        msg.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(msg)
        return {
            "modes": names,
            "values": commanded,
            "altitude": msg.altitude,
            "frame": "world" if msg.velocity_frame == MotionSetpoint.WORLD else "heading",
            "timeout": msg.timeout,
        }

    def hold_here(self) -> dict:
        """Stop any routine; every axis stops smoothly and holds, level."""
        with self._lock:
            active = self._run is not None
        if active:
            self._finish("stopped", "Stopped by operator", hold=True)
        else:
            self._send(self._stop_segment())
        return self.routine_snapshot()

    def zero_pose(self) -> Future:
        """Call sub_control's reset_pose: the vehicle's position and heading become the origin."""
        result: Future = Future()
        with self._lock:
            active = self._run is not None
        if active:
            result.set_exception(RuntimeError("stop the routine first"))
        elif not self._reset_pose.service_is_ready():
            result.set_exception(RuntimeError(f"{self.controller_fqn}/reset_pose is not available"))
        else:

            def done(future) -> None:
                try:
                    response = future.result()
                    result.set_result({"success": response.success, "message": response.message})
                except Exception as exc:
                    result.set_exception(exc)

            self._reset_pose.call_async(Trigger.Request()).add_done_callback(done)
        return result

    # Routines

    def _start_pose(self) -> list[float]:
        """Where sub_control is holding: each POSITION axis's target, else the measured pose."""
        measured = self._measured()
        status = self._status
        if measured is None or status is None:
            raise RuntimeError("no odometry and control/status yet")
        pose = list(measured[0])
        for axis in range(6):
            # A z target in altitude coordinates is not an odom z.
            if status.mode[axis] == POSITION and not (axis == Z and status.altitude):
                pose[axis] = status.target[axis]
        if status.altitude and self._odometry is not None:
            pose[Z] = self._odometry.pose.pose.position.z
        return pose

    def start_routine(
        self,
        kind: str,
        axis: str,
        amplitude: float,
        hold: float,
        settle: float,
        cycles: int,
    ) -> dict:
        self._count_publishers()
        with self._lock:
            if self._run is not None:
                raise RuntimeError("a routine is already running")
            problem = self._problem()
            if problem:
                raise RuntimeError(problem)
            start = self._start_pose()
        routine = routines.plan(kind, axis, amplitude, hold, settle, cycles, start)
        # Hold every axis where it is first, so the routine moves only what it tests.
        self._send(routines.hold_at(routine.start))
        with self._lock:
            self._run = {"routine": routine, "started": self.get_clock().now(), "segment": None}
            self._routine_state = {
                "active": True,
                "status": "running",
                "message": "Running",
                "elapsed": 0.0,
                "progress": 0.0,
                **routine.public_state(),
            }
        self._routine_tick()
        return self.routine_snapshot()

    def stop_routine(self) -> dict:
        with self._lock:
            active = self._run is not None
        if active:
            self._finish("stopped", "Stopped by operator", hold=True)
        return self.routine_snapshot()

    def _routine_tick(self) -> None:
        with self._lock:
            run = self._run
            if run is None:
                return
            problem = self._problem()
            killed = self._status is not None and self._status.killed
        if problem:
            self._finish("aborted", problem, hold=not killed)
            return

        routine: routines.Routine = run["routine"]
        elapsed = (self.get_clock().now() - run["started"]).nanoseconds * 1e-9
        index = routine.segment_at(elapsed)
        if index is None:
            # The last segment already holds the start (or stops a cruise).
            self._finish("complete", "Complete", hold=False)
            return
        segment = routine.segments[index]
        with self._lock:
            # Checked and sent under the lock _finish holds while it sends the
            # hold, so no routine target can follow a stop.
            if self._run is run:
                if index != run["segment"] or segment.streamed:
                    self._send(segment, STREAM_TIMEOUT if segment.streamed else 0.0)
                run["segment"] = index
                self._routine_state.update(
                    elapsed=elapsed,
                    progress=elapsed / routine.duration,
                    segment=index,
                    message=f"{index + 1}/{len(routine.segments)}: {segment.label}",
                )

    def _finish(self, status: str, message: str, hold: bool) -> None:
        with self._lock:
            run = self._run
            if run is None:
                return
            self._run = None
            routine: routines.Routine = run["routine"]
            self._routine_state = {
                **self._routine_state,
                "active": False,
                "status": status,
                "message": message,
                "progress": 1.0
                if status == "complete"
                else self._routine_state.get("progress", 0.0),
            }
            if hold:
                self._send(self._stop_segment(routine.start[ROLL], routine.start[PITCH]))

    def routine_snapshot(self) -> dict:
        with self._lock:
            return dict(self._routine_state)

    def snapshot(self, include_parameters: bool = True) -> dict:
        with self._lock:
            now = time.monotonic()
            odometry_age = now - self._odometry_time if self._odometry_time else None
            status_age = now - self._status_time if self._status_time else None
            result = {
                "namespace": self.get_namespace(),
                "controller": self.controller_fqn,
                "signals": self._signals(),
                "controller_state": self._controller_state(),
                "killed": bool(self._status.killed) if self._status is not None else None,
                "odometry_age": odometry_age,
                "status_age": status_age,
                "telemetry_age": (
                    max(odometry_age, status_age)
                    if odometry_age is not None and status_age is not None
                    else None
                ),
                "conflicts": self._conflicts(),
                "routine": dict(self._routine_state),
                "nodes": sorted(self._known_nodes | set(self._remote_parameters)),
            }
            if include_parameters:
                result["parameters"] = {
                    node: {name: dict(metadata) for name, metadata in parameters.items()}
                    for node, parameters in self._remote_parameters.items()
                }
            return result
