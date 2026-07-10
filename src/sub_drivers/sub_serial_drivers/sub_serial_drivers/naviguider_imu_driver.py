import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu

import serial

MAX_RX_BUFFER = 4096
GYRO_SCALE_TO_RAD_PER_SEC = 1.0

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

        self._serial: serial.Serial | None = None
        self._rx_buffer = bytearray()
        self._frame_id = "imu_link"
        self._is_active = False

        self._accel_rate = 0
        self._gyro_rate = 0
        self._orientation_rate = 0

        self._imu_msg = Imu()
        self._imu_pub = None
        self._poll_timer = None

    def on_configure(self, state) -> TransitionCallbackReturn:
        device = self.get_parameter("device").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._accel_rate = self.get_parameter("accel_rate").get_parameter_value().integer_value
        self._gyro_rate = self.get_parameter("gyro_rate").get_parameter_value().integer_value
        self._orientation_rate = (
            self.get_parameter("orientation_rate").get_parameter_value().integer_value
        )

        try:
            # timeout=0 -> non-blocking reads, so read() returns immediately
            # with whatever bytes are currently buffered.
            self._serial = serial.Serial(port=device, baudrate=baud, timeout=0)
        except Exception as exc:  # noqa: BLE001 - report any open/config failure
            self.get_logger().error(f"could not open serial: {exc}")
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info(f"naviguider_imu connected to {device} @ {baud} baud")

        self._imu_msg.orientation_covariance = [
            6.801009e-04, -2.957911e-04, -4.988638e-05,
            -2.957911e-04, 6.518145e-04, 1.541138e-05,
            -4.988638e-05, 1.541138e-05, 1.107581e-05,
        ]
        self._imu_msg.angular_velocity_covariance = [
            2.234328e-06, 3.029158e-07, -2.190460e-08,
            3.029158e-07, 1.560898e-05, 2.350362e-06,
            -2.190460e-08, 2.350362e-06, 2.446427e-06,
        ]
        self._imu_msg.linear_acceleration_covariance = [
            1e-2, 0.0, 0.0,
            0.0, 1e-2, 0.0,
            0.0, 0.0, 1e-2,
        ]

        self._imu_pub = self.create_lifecycle_publisher(
            Imu, "imu/data", QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        self._poll_timer = self.create_timer(0.005, self._poll_timer_callback)

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
        if self._poll_timer is not None:
            self.destroy_timer(self._poll_timer)
            self._poll_timer = None
        if self._imu_pub is not None:
            self.destroy_publisher(self._imu_pub)
            self._imu_pub = None
        if self._serial is not None:
            self._serial.close()
            self._serial = None
        self._rx_buffer.clear()

    def _write(self, command: str) -> None:
        """Send an ASCII command string to the module."""
        if self._serial is None:
            return
        self._serial.write(command.encode("ascii"))

    def _start_sensors(self) -> None:
        if self._serial is None:
            return
        # Commands are case-sensitive and terminated by a carriage return (0x0D).
        self._write("M1\r")  # non-verbose: emit numeric sensor IDs in the stream
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

    def _poll_timer_callback(self) -> None:
        if self._is_active:
            self._poll_serial()

    def _poll_serial(self) -> None:
        if self._serial is None:
            return

        try:
            waiting = self._serial.in_waiting
            chunk = self._serial.read(waiting) if waiting else b""
        except (serial.SerialException, OSError):
            chunk = None

        if chunk is None:
            self.get_logger().error(
                "serial read error / device disconnected",
                throttle_duration_sec=1.0,
            )
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
                return float(fields[idx])
            except ValueError:
                return None

        if sensor_id == SENSOR_ACCELEROMETER:
            x, y, z = value(2), value(3), value(4)
            if x is not None and y is not None and z is not None:
                self._imu_msg.linear_acceleration.x = x
                self._imu_msg.linear_acceleration.y = y
                self._imu_msg.linear_acceleration.z = z

                self._imu_msg.header.stamp = self.get_clock().now().to_msg()
                self._imu_msg.header.frame_id = self._frame_id
                self._imu_pub.publish(self._imu_msg)
        elif sensor_id == SENSOR_GYROSCOPE:
            x, y, z = value(2), value(3), value(4)
            if x is not None and y is not None and z is not None:
                # The NaviGuider gyro stream is already expressed in rad/s, which
                # matches sensor_msgs/Imu, so publish the values directly.
                self._imu_msg.angular_velocity.x = x * GYRO_SCALE_TO_RAD_PER_SEC
                self._imu_msg.angular_velocity.y = y * GYRO_SCALE_TO_RAD_PER_SEC
                self._imu_msg.angular_velocity.z = z * GYRO_SCALE_TO_RAD_PER_SEC
        elif sensor_id == SENSOR_GAME_ROTATION_VECTOR:
            qx, qy, qz, qw = value(2), value(3), value(4), value(5)
            if None not in (qx, qy, qz, qw):
                self._imu_msg.orientation.x = qx
                self._imu_msg.orientation.y = qy
                self._imu_msg.orientation.z = qz
                self._imu_msg.orientation.w = qw


def main(args=None):
    rclpy.init(args=args)
    node = NaviGuiderIMUDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
