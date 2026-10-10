import math
import threading
import time

import rclpy
import serial
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, Float64
from sub_driver_interfaces.srv import LaunchTorpedo, SetDropper

NUM_THRUSTERS = 8
NUM_TORPEDO_THRUSTERS = 2
NUM_DROPPERS = 2

READER_JOIN_TIMEOUT = 2.0
# Longest line the board sends is a few tens of bytes; anything longer is noise.
MAX_LINE_LENGTH = 256
# Hard ceiling on |normalized thruster command|: the board sends 1500 +/- 400 * command us
# to the Basic ESCs, so this keeps them within 1260-1740 us whatever is published. Matches
# MAX_POWER_LIMIT in sub_control.
MAX_COMMAND = 0.6


class SubLow(LifecycleNode):
    def __init__(self, **kwargs):
        super().__init__("sub_low", **kwargs)

        self.declare_parameter("device", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("serial_timeout", 1.0)

        # Thrusters stop if sub_control goes quiet for this long [s] (it
        # publishes every control cycle, killed or not).
        self.declare_parameter("command_timeout", 0.5)

        self.declare_parameter("depth_frame_id", "odom")
        self.declare_parameter("depth_child_frame_id", "base_link")
        self.declare_parameter("depth_z_variance", 0.01)

        self._serial: serial.Serial | None = None
        self._write_lock = threading.Lock()

        self._reader_thread: threading.Thread | None = None
        self._reader_stop = threading.Event()

        self._is_active = False
        # time.monotonic() of the last thruster command, so that a step of the system clock
        # cannot hold off the watchdog.
        self._last_command: float | None = None
        self._stopped = True
        self._watchdog = None

        self._kill_pub = None
        self._depth_pub = None
        self._thruster_subs = []
        self._launch_torpedo_srv = None
        self._set_dropper_srv = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        if not self._open_serial():
            return TransitionCallbackReturn.FAILURE
        # Start from stopped thrusters, whatever the board was last told: a sub_low that
        # died without stopping them would otherwise leave them running.
        self._write("a 0\n")

        kill_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._kill_pub = self.create_lifecycle_publisher(Bool, "kill_switch", kill_qos)

        self._depth_frame_id = self.get_parameter("depth_frame_id").value
        self._depth_child_frame_id = self.get_parameter("depth_child_frame_id").value
        self._depth_z_variance = self.get_parameter("depth_z_variance").value
        self._depth_pub = self.create_lifecycle_publisher(
            Odometry, "odometry/depth", qos_profile_sensor_data
        )

        thruster_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._thruster_subs = [
            self.create_subscription(
                Float64,
                f"control/thruster_{i}",
                self._make_thruster_callback(i),
                thruster_qos,
            )
            for i in range(NUM_THRUSTERS)
        ]

        self._launch_torpedo_srv = self.create_service(
            LaunchTorpedo, "launch_torpedo", self._launch_torpedo_callback
        )
        self._set_dropper_srv = self.create_service(
            SetDropper, "set_dropper", self._set_dropper_callback
        )

        self._watchdog = self.create_timer(
            0.1, self._check_commands, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

        self._reader_stop.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="sub_low_serial_reader", daemon=True
        )
        self._reader_thread.start()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = False
        # Stop all thrusters before going idle.
        self._write("a 0\n")
        self._stopped = True
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._is_active = False
        self._write("a 0\n")
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def destroy_node(self) -> None:
        # rclpy runs no lifecycle transition when the process is stopped (Ctrl-C, or launch
        # shutting down), so stop the thrusters here rather than leave them at their last
        # command.
        self._is_active = False
        self._write("a 0\n")
        self._teardown()
        super().destroy_node()

    def _open_serial(self) -> bool:
        device = self.get_parameter("device").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        timeout = self.get_parameter("serial_timeout").get_parameter_value().double_value

        try:
            ser = serial.Serial()
            ser.port = device
            ser.baudrate = baud
            ser.timeout = timeout
            # Without it, a board that stops reading blocks write() - and with it the
            # executor, the command watchdog and shutdown - forever.
            ser.write_timeout = timeout
            ser.dtr = False
            ser.rts = False
            try:
                ser.exclusive = True
            except (AttributeError, ValueError):
                pass

            ser.open()
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except (serial.SerialException, ValueError) as exc:
            self.get_logger().error(f"failed to open serial port '{device}': {exc}")
            return False

        self._serial = ser
        self.get_logger().info(f"opened serial port '{device}' @ {baud} baud")
        return True

    def _write(self, command: str) -> bool:
        if self._serial is None:
            return False

        try:
            with self._write_lock:
                self._serial.write(command.encode("ascii"))
            return True
        except (serial.SerialException, UnicodeEncodeError) as exc:
            self.get_logger().warning(f"serial write failed: {exc}", throttle_duration_sec=1.0)
            return False

    def _reader_loop(self) -> None:
        ser = self._serial
        partial = b""
        while ser is not None and not self._reader_stop.is_set():
            try:
                chunk = ser.readline()
            except serial.SerialException as exc:
                self.get_logger().error(f"serial read failed: {exc}")
                break

            if not chunk.endswith(b"\n"):
                # readline() timed out, possibly mid-line. Keep what arrived for the next
                # read, and re-check the stop flag: a truncated "d 0.53" must not be
                # parsed as 0.
                partial += chunk
                if len(partial) > MAX_LINE_LENGTH:
                    partial = b""
                continue
            line, partial = partial + chunk, b""

            try:
                self._handle_line(line)
            except Exception as exc:  # never let the reader thread die silently
                self.get_logger().warning(
                    f"error while handling serial line {line!r}: {exc}",
                    throttle_duration_sec=1.0,
                )

    def _handle_line(self, line: bytes) -> None:
        fields = line.split()
        if not fields:
            return

        if fields[0] == b"x":
            try:
                value = int(fields[1])
            except (IndexError, ValueError):
                return

            if self._kill_pub is not None:
                self._kill_pub.publish(Bool(data=bool(value)))
        elif fields[0] == b"d":
            try:
                depth = float(fields[1])
            except (IndexError, ValueError):
                return
            # float() accepts "nan" and "inf", either of which would corrupt the EKF.
            if not math.isfinite(depth):
                return

            self._publish_depth(depth)

    def _publish_depth(self, depth: float) -> None:
        if self._depth_pub is None:
            return

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self._depth_frame_id
        odom.child_frame_id = self._depth_child_frame_id
        # Sensor reports depth (positive down); report ENU z (positive up).
        odom.pose.pose.position.z = -depth
        odom.pose.pose.orientation.w = 1.0
        odom.pose.covariance[14] = self._depth_z_variance

        self._depth_pub.publish(odom)

    def _teardown(self) -> None:
        self._stop_reader()
        self._close_serial()
        self._destroy_entities()

    def _stop_reader(self) -> None:
        self._reader_stop.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=READER_JOIN_TIMEOUT)
            if self._reader_thread.is_alive():
                self.get_logger().warning("serial reader thread did not stop cleanly")
            self._reader_thread = None

    def _close_serial(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except serial.SerialException as exc:
                self.get_logger().warning(f"error closing serial port: {exc}")
            self._serial = None
            self._is_active = False

    def _destroy_entities(self) -> None:
        if self._kill_pub is not None:
            self.destroy_lifecycle_publisher(self._kill_pub)
            self._kill_pub = None
        if self._depth_pub is not None:
            self.destroy_lifecycle_publisher(self._depth_pub)
            self._depth_pub = None
        if self._launch_torpedo_srv is not None:
            self.destroy_service(self._launch_torpedo_srv)
            self._launch_torpedo_srv = None
        if self._set_dropper_srv is not None:
            self.destroy_service(self._set_dropper_srv)
            self._set_dropper_srv = None
        for sub in self._thruster_subs:
            self.destroy_subscription(sub)
        self._thruster_subs = []
        if self._watchdog is not None:
            self.destroy_timer(self._watchdog)
            self._watchdog = None

    def _make_thruster_callback(self, index: int):
        def callback(msg: Float64) -> None:
            if not self._is_active:
                return
            self._last_command = time.monotonic()
            self._stopped = False
            thrust = msg.data
            if not math.isfinite(thrust):
                self.get_logger().error(
                    f"thruster {index} commanded {thrust}: stopping it",
                    throttle_duration_sec=1.0,
                )
                thrust = 0.0
            thrust = min(max(thrust, -MAX_COMMAND), MAX_COMMAND)
            self._write(f"p {index} {round(thrust, 3)}\n")

        return callback

    def _check_commands(self) -> None:
        """Stop every thruster if sub_control stops commanding them."""
        if not self._is_active or self._stopped or self._last_command is None:
            return
        timeout = self.get_parameter("command_timeout").value
        if time.monotonic() - self._last_command > timeout:
            self.get_logger().error(
                f"no thruster commands for {timeout:.1f} s: stopping all thrusters"
            )
            self._write("a 0\n")
            self._stopped = True

    def _launch_torpedo_callback(self, request, response):
        if not self._is_active or self._serial is None:
            response.success = False
            response.message = "sub_low is not active."
            return response

        if not 0 <= request.torpedo_id < NUM_TORPEDO_THRUSTERS:
            response.success = False
            response.message = f"Invalid torpedo id {request.torpedo_id}."
            return response

        if not self._write(f"t {request.torpedo_id} {int(request.open)}\n"):
            response.success = False
            response.message = "serial write failed"
            return response

        state = "opened" if request.open else "closed"
        response.success = True
        response.message = f"Torpedo thruster {request.torpedo_id} {state}."
        return response

    def _set_dropper_callback(self, request, response):
        if not self._is_active or self._serial is None:
            response.success = False
            response.message = "sub_low is not active."
            return response

        if not 0 <= request.dropper_id < NUM_DROPPERS:
            response.success = False
            response.message = f"Invalid dropper id {request.dropper_id}."
            return response

        if not self._write(f"d {request.dropper_id} {int(request.open)}\n"):
            response.success = False
            response.message = "serial write failed"
            return response

        state = "opened" if request.open else "closed"
        response.success = True
        response.message = f"Dropper {request.dropper_id} {state}."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SubLow()
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
