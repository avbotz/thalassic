"""NaviGuider IMU over USB serial, as sensor_msgs/Imu on imu/data.

Each accelerometer line is published with the latest gyro and orientation,
but only while both are fresh (`max_sample_age`): a sensor that stops
reporting makes imu/data stop, which sub_control treats as losing attitude,
rather than repeating its last value. If the serial link drops, the port is
reopened and the sensors restarted as soon as it comes back.
"""

import math
import time

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu

MAX_RX_BUFFER = 4096

# Virtual sensor IDs (see NaviGuider manual).
SENSOR_ACCELEROMETER = 1  # payload: X, Y, Z, accuracy  (m/s^2)
SENSOR_GYROSCOPE = 4  # payload: X, Y, Z, accuracy (rad/s)
SENSOR_GAME_ROTATION_VECTOR = 15  # 6-DOF accel+gyro: QX, QY, QZ, QW, accuracy


class NaviGuiderIMUDriver(LifecycleNode):
    """Streams accelerometer, gyroscope, and orientation as sensor_msgs/Imu."""

    def __init__(self, **kwargs):
        super().__init__("naviguider_imu", **kwargs)
        self.declare_parameter("device", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("frame_id", "imu_link")
        # Per-sensor output rates in Hz. 0 disables a sensor. Aggregate data
        # rate must not exceed 1200 Hz.
        self.declare_parameter("accel_rate", 50)
        self.declare_parameter("gyro_rate", 50)
        self.declare_parameter("orientation_rate", 50)
        # Oldest gyro or orientation sample [s] an accelerometer line is published with.
        self.declare_parameter("max_sample_age", 0.2)
        # How often to try the port again while it is gone [s].
        self.declare_parameter("reconnect_period", 1.0)

        self._serial: serial.Serial | None = None
        self._device = ""
        self._baud = 115200
        self._rx_buffer = bytearray()
        self._frame_id = "imu_link"
        self._is_active = False
        self._configured = False

        self._accel_rate = 0
        self._gyro_rate = 0
        self._orientation_rate = 0
        # time.monotonic() each was last heard from since activation; None if not yet.
        self._last_gyro: float | None = None
        self._last_orientation: float | None = None
        self._last_publish: float | None = None
        self._last_open_attempt: float | None = None
        self._published = 0
        self._stale_skipped = 0
        self._disconnects = 0

        self._imu_msg = Imu()
        self._imu_pub = None
        self._poll_timer = None

        self._diagnostics = Updater(self)
        self._diagnostics.setHardwareID("naviguider")
        self._diagnostics.add("imu", self._diagnose)

    def on_configure(self, state) -> TransitionCallbackReturn:
        self._device = self.get_parameter("device").get_parameter_value().string_value
        self._baud = self.get_parameter("baud").get_parameter_value().integer_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._accel_rate = self.get_parameter("accel_rate").get_parameter_value().integer_value
        self._gyro_rate = self.get_parameter("gyro_rate").get_parameter_value().integer_value
        self._orientation_rate = (
            self.get_parameter("orientation_rate").get_parameter_value().integer_value
        )
        # A device that is not there yet does not fail the transition for good:
        # the poll timer keeps trying it.
        self._open()

        self._imu_msg.orientation_covariance = [
            6.801009e-04,
            -2.957911e-04,
            -4.988638e-05,
            -2.957911e-04,
            6.518145e-04,
            1.541138e-05,
            -4.988638e-05,
            1.541138e-05,
            1.107581e-05,
        ]
        self._imu_msg.angular_velocity_covariance = [
            2.234328e-06,
            3.029158e-07,
            -2.190460e-08,
            3.029158e-07,
            1.560898e-05,
            2.350362e-06,
            -2.190460e-08,
            2.350362e-06,
            2.446427e-06,
        ]
        self._imu_msg.linear_acceleration_covariance = [
            1e-2,
            0.0,
            0.0,
            0.0,
            1e-2,
            0.0,
            0.0,
            0.0,
            1e-2,
        ]

        self._imu_pub = self.create_lifecycle_publisher(
            Imu,
            "imu/data",
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )

        self._poll_timer = self.create_timer(0.005, self._poll_timer_callback)
        self._configured = True

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self._rx_buffer.clear()
        self._start_sensors()
        self._is_active = True
        self.get_logger().info("naviguider_imu active")
        return super().on_activate(state)

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._is_active = False
        self._stop_sensors()
        self.get_logger().info("naviguider_imu deactivated")
        return super().on_deactivate(state)

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self._is_active = False
        self._stop_sensors()
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def _teardown(self) -> None:
        self._configured = False
        if self._poll_timer is not None:
            self.destroy_timer(self._poll_timer)
            self._poll_timer = None
        if self._imu_pub is not None:
            self.destroy_lifecycle_publisher(self._imu_pub)
            self._imu_pub = None
        self._close()

    def _open(self) -> bool:
        self._last_open_attempt = time.monotonic()
        try:
            # timeout=0 -> non-blocking reads, so read() returns immediately
            # with whatever bytes are currently buffered.
            self._serial = serial.Serial(port=self._device, baudrate=self._baud, timeout=0)
        except (serial.SerialException, ValueError, OSError) as exc:
            self.get_logger().error(
                f"cannot open serial port '{self._device}': {exc}; retrying",
                throttle_duration_sec=10.0,
            )
            return False
        self._rx_buffer.clear()
        self.get_logger().info(f"naviguider_imu connected to {self._device} @ {self._baud} baud")
        return True

    def _close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except (serial.SerialException, OSError):
                pass
            self._serial = None
        self._rx_buffer.clear()

    def _write(self, command: str) -> None:
        """Send an ASCII command string to the module."""
        if self._serial is None:
            return
        try:
            self._serial.write(command.encode("ascii"))
        except (serial.SerialException, OSError) as exc:
            self._lost(f"serial write failed ({exc})")

    def _start_sensors(self) -> None:
        # Nothing the gyro and the orientation said before this counts.
        self._last_gyro = None
        self._last_orientation = None
        if self._serial is None:
            return
        # Commands are case-sensitive and terminated by a carriage return (0x0D).
        self._write("M1\r")  # set mounting orientation
        self._write("V0\r")  # non-verbose: emit numeric sensor IDs in the stream
        self._write("J4\r")  # ENU orientation frame (ROS convention)

        if self._accel_rate > 0:
            self._write(f"s {SENSOR_ACCELEROMETER},{self._accel_rate}\r")
        if self._gyro_rate > 0:
            self._write(f"s {SENSOR_GYROSCOPE},{self._gyro_rate}\r")
        if self._orientation_rate > 0:
            self._write(f"s {SENSOR_GAME_ROTATION_VECTOR},{self._orientation_rate}\r")

    def _stop_sensors(self) -> None:
        if self._serial is None:
            return
        # A zero sample rate disables the virtual sensor (manual, "Sample_Rate" key).
        self._write(f"s {SENSOR_ACCELEROMETER},0\r")
        self._write(f"s {SENSOR_GYROSCOPE},0\r")
        self._write(f"s {SENSOR_GAME_ROTATION_VECTOR},0\r")

    def _lost(self, reason: str) -> None:
        if self._serial is None:
            return
        self._disconnects += 1
        self.get_logger().error(f"{reason}: IMU link lost, reconnecting")
        self._close()

    def _poll_timer_callback(self) -> None:
        if self._serial is None:
            self._reconnect()
        elif self._is_active:
            self._poll_serial()

    def _reconnect(self) -> None:
        """Tries the port again, at most once every reconnect_period."""
        if not self._configured:
            return
        period = self.get_parameter("reconnect_period").value
        last = self._last_open_attempt
        if last is not None and time.monotonic() - last < period:
            return
        if self._open() and self._is_active:
            # The module may have been power cycled: tell it again what to stream.
            self._start_sensors()

    def _poll_serial(self) -> None:
        if self._serial is None:
            return

        try:
            waiting = self._serial.in_waiting
            chunk = self._serial.read(waiting) if waiting else b""
        except (serial.SerialException, OSError, TypeError) as exc:
            self._lost(f"serial read failed ({exc})")
            return

        self._rx_buffer.extend(chunk)

        start = 0
        newline = self._rx_buffer.find(b"\n", start)
        while newline != -1:
            self._handle_line(bytes(self._rx_buffer[start:newline]))
            start = newline + 1
            newline = self._rx_buffer.find(b"\n", start)
        del self._rx_buffer[:start]

        if len(self._rx_buffer) > MAX_RX_BUFFER:
            self._rx_buffer.clear()

    def _handle_line(self, line: bytes) -> None:
        fields = [tok.strip() for tok in line.split(b",")]
        if len(fields) < 2:
            return

        # fields[0] is the timestamp, fields[1] the numeric sensor ID. Lines
        # that do not start with "<timestamp>,<id>" (e.g. residual verbose text
        # or meta events) simply fail the integer parse and are ignored.
        try:
            sensor_id = int(fields[1])
        except ValueError:
            return

        def value(idx: int):
            if idx >= len(fields):
                return None
            try:
                v = float(fields[idx])
            except ValueError:
                return None
            # float() accepts "nan" and "inf"; neither is a measurement.
            return v if math.isfinite(v) else None

        if sensor_id == SENSOR_ACCELEROMETER:
            x, y, z = value(2), value(3), value(4)
            if x is not None and y is not None and z is not None:
                self._imu_msg.linear_acceleration.x = x
                self._imu_msg.linear_acceleration.y = y
                self._imu_msg.linear_acceleration.z = z
                # Until the gyro and the orientation have reported since activation, the
                # message's rates and orientation are unmeasured defaults; once one stops,
                # they are stale. Either way, nothing goes out.
                if not self._fresh():
                    self._stale_skipped += 1
                    return

                self._imu_msg.header.stamp = self.get_clock().now().to_msg()
                self._imu_msg.header.frame_id = self._frame_id
                self._imu_pub.publish(self._imu_msg)
                self._published += 1
                self._last_publish = time.monotonic()
        elif sensor_id == SENSOR_GYROSCOPE:
            x, y, z = value(2), value(3), value(4)
            if x is not None and y is not None and z is not None:
                # The NaviGuider gyro stream is already expressed in rad/s, which
                # matches sensor_msgs/Imu, so publish the values directly.
                self._imu_msg.angular_velocity.x = x
                self._imu_msg.angular_velocity.y = y
                self._imu_msg.angular_velocity.z = z
                self._last_gyro = time.monotonic()
        elif sensor_id == SENSOR_GAME_ROTATION_VECTOR:
            qx, qy, qz, qw = value(2), value(3), value(4), value(5)
            # An all-zero quaternion is not a rotation; normalizing it downstream gives NaN.
            if None not in (qx, qy, qz, qw) and (qx, qy, qz, qw) != (0.0, 0.0, 0.0, 0.0):
                self._imu_msg.orientation.x = qx
                self._imu_msg.orientation.y = qy
                self._imu_msg.orientation.z = qz
                self._imu_msg.orientation.w = qw
                self._last_orientation = time.monotonic()

    def _fresh(self) -> bool:
        """Whether the gyro and the orientation (those enabled) are recent enough to publish."""
        limit = self.get_parameter("max_sample_age").value
        now = time.monotonic()
        for rate, last in (
            (self._gyro_rate, self._last_gyro),
            (self._orientation_rate, self._last_orientation),
        ):
            if rate > 0 and (last is None or now - last > limit):
                return False
        return True

    def _diagnose(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        now = time.monotonic()

        def since(stamp: float | None) -> str:
            return "never" if stamp is None else f"{now - stamp:.2f} s ago"

        stat.add("device", self._device)
        stat.add("connected", str(self._serial is not None))
        stat.add("disconnects", str(self._disconnects))
        stat.add("published", str(self._published))
        stat.add("held back for a stale gyro or orientation", str(self._stale_skipped))
        stat.add("last published", since(self._last_publish))
        stat.add("last gyro", since(self._last_gyro))
        stat.add("last orientation", since(self._last_orientation))
        if self._serial is None:
            stat.summary(DiagnosticStatus.ERROR, "not connected")
        elif not self._is_active:
            stat.summary(DiagnosticStatus.WARN, "inactive")
        elif self._last_publish is None or now - self._last_publish > 0.5:
            stat.summary(DiagnosticStatus.ERROR, "not publishing: a sensor is not reporting")
        else:
            stat.summary(DiagnosticStatus.OK, "publishing")
        return stat


def main(args=None):
    rclpy.init(args=args)
    node = NaviGuiderIMUDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
