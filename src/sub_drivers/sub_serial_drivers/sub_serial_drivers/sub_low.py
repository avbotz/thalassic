"""Bridge to the Maritime board over USB serial.

The protocol is newline-terminated ASCII (maritime's README has the board's side):

  host -> board
    m <c0> ... <c7>   all eight thruster commands at once (protocol 2 and later)
    p <i> <c>         one thruster's command (every protocol)
    a <c>             every thruster
    t <i> <0|1>       close/open torpedo i
    d <i> <0|1>       close/open dropper i
  board -> host
    x <0|1>           kill switch, 1 = killed, 5 Hz
    d <m>             depth [m, positive down], 5 Hz
    h <protocol> <uptime_ms> <command_timeouts>
                      heartbeat, 5 Hz (protocol 2 and later); command_timeouts
                      counts the times the board's own watchdog stopped the
                      thrusters because commands stopped
    i ...             AHRS angles (not used)

Thruster commands go out as one `m` line once the board's heartbeat says it
speaks protocol 2, and as eight `p` lines until then, so a board still running
older firmware keeps working. If the serial link drops (the USB cable, the
board resetting), the port is reopened as soon as it comes back and the
thrusters are stopped first. Nothing here fakes a kill when the link is lost:
the board's own watchdog stops the thrusters, and /diagnostics reports it.
"""

import math
import threading
import time

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
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
from rclpy.time import Time
from std_msgs.msg import Bool

from sub_control_interfaces.msg import ThrusterCommand
from sub_driver_interfaces.srv import LaunchTorpedo, SetDropper

NUM_THRUSTERS = 8
NUM_TORPEDO_THRUSTERS = 2
NUM_DROPPERS = 2

READER_JOIN_TIMEOUT = 2.0
# Longest line the board sends is a few tens of bytes; anything longer is noise.
MAX_LINE_LENGTH = 256
# Hard ceiling on |normalized thruster command|: the board sends 1500 +/- 400 * command us
# to the Basic ESCs, so this keeps them within 1260-1740 us whatever is published. Matches
# MAX_POWER_LIMIT in sub_control and MAX_POWER in the firmware.
MAX_COMMAND = 0.6
# The protocol whose `m` line carries all eight thruster commands.
BATCH_PROTOCOL = 2
# [s] without a heartbeat after which the board is treated as speaking protocol 1 again.
HEARTBEAT_TIMEOUT = 1.0
# [s] without a kill switch or depth line after which /diagnostics reports them stale.
BOARD_STATE_TIMEOUT = 1.0


def thruster_lines(commands: list[float], protocol: int) -> str:
    """The serial text that sets every thruster: one `m` line, or eight `p` lines."""
    values = [f"{c:.3f}" for c in commands]
    if protocol >= BATCH_PROTOCOL:
        return "m " + " ".join(values) + "\n"
    return "".join(f"p {i} {v}\n" for i, v in enumerate(values))


def sanitize(command: float) -> float:
    """A thruster command the ESCs can take: NaN/inf stop, the rest within +/-MAX_COMMAND."""
    if not math.isfinite(command):
        return 0.0
    return min(max(command, -MAX_COMMAND), MAX_COMMAND)


def parse_line(line: bytes) -> tuple[str, tuple] | None:
    """One line from the board as (kind, values), or None if it is not one we use.

    Kinds: ("kill", (killed,)), ("depth", (metres,)),
    ("heartbeat", (protocol, uptime_ms, command_timeouts)).
    """
    fields = line.split()
    if not fields:
        return None
    try:
        if fields[0] == b"x" and len(fields) == 2:
            return ("kill", (bool(int(fields[1])),))
        if fields[0] == b"d" and len(fields) == 2:
            depth = float(fields[1])
            # float() accepts "nan" and "inf", either of which would corrupt the EKF.
            return ("depth", (depth,)) if math.isfinite(depth) else None
        if fields[0] == b"h" and len(fields) == 4:
            return ("heartbeat", tuple(int(f) for f in fields[1:]))
    except ValueError:
        return None
    return None


class SubLow(LifecycleNode):
    def __init__(self, **kwargs):
        super().__init__("sub_low", **kwargs)

        self.declare_parameter("device", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("serial_timeout", 1.0)
        # How often to try the port again while it is gone [s].
        self.declare_parameter("reconnect_period", 1.0)

        # Thrusters stop if sub_control goes quiet for this long [s] (it
        # publishes every control cycle, killed or not); a command older than
        # this when it arrives is not acted on either.
        self.declare_parameter("command_timeout", 0.5)

        self.declare_parameter("depth_frame_id", "odom")
        self.declare_parameter("depth_child_frame_id", "base_link")
        self.declare_parameter("depth_z_variance", 0.01)

        # The open port, or None while the link is down. Replaced only under _io_lock,
        # which every write holds too.
        self._serial: serial.Serial | None = None
        self._io_lock = threading.Lock()

        self._reader_thread: threading.Thread | None = None
        self._reader_stop = threading.Event()

        self._is_active = False
        # time.monotonic() of the last thruster command, so that a step of the system clock
        # cannot hold off the watchdog.
        self._last_command: float | None = None
        self._stopped = True
        self._watchdog = None

        # What the board last said, each with the time.monotonic() it said it.
        self._board_protocol = 1
        self._last_heartbeat: float | None = None
        self._board_uptime_ms: int | None = None
        self._board_timeouts = 0
        self._board_restarts = 0
        self._killed: bool | None = None
        self._last_kill: float | None = None
        self._depth: float | None = None
        self._last_depth: float | None = None
        self._disconnects = 0
        self._stale_commands = 0

        self._kill_pub = None
        self._depth_pub = None
        self._thruster_sub = None
        self._launch_torpedo_srv = None
        self._set_dropper_srv = None

        self._diagnostics = Updater(self)
        self._diagnostics.setHardwareID("maritime")
        self._diagnostics.add("serial link", self._diagnose_link)
        self._diagnostics.add("kill switch and depth", self._diagnose_board_state)
        self._diagnostics.add("thruster commands", self._diagnose_commands)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
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
        self._thruster_sub = self.create_subscription(
            ThrusterCommand, "control/thrusters", self._thrusters_callback, thruster_qos
        )

        self._launch_torpedo_srv = self.create_service(
            LaunchTorpedo, "launch_torpedo", self._launch_torpedo_callback
        )
        self._set_dropper_srv = self.create_service(
            SetDropper, "set_dropper", self._set_dropper_callback
        )

        self._watchdog = self.create_timer(
            0.1, self._check_commands, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

        # Opens the port (and reopens it whenever it drops), so a board that is
        # not plugged in yet does not fail the configure transition for good.
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

    # --- the serial link

    def _open_serial(self) -> serial.Serial | None:
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
        except (serial.SerialException, ValueError, OSError) as exc:
            self.get_logger().error(
                f"cannot open serial port '{device}': {exc}; retrying",
                throttle_duration_sec=10.0,
            )
            return None
        return ser

    def _connect(self) -> bool:
        ser = self._open_serial()
        if ser is None:
            return False
        try:
            # Start from stopped thrusters, whatever the board was last told: a sub_low that
            # died, or a link that dropped, would otherwise leave them running.
            ser.write(b"a 0\n")
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"serial write failed right after opening: {exc}")
            ser.close()
            return False
        with self._io_lock:
            self._serial = ser
            # Until its heartbeat says otherwise, the board may be running older firmware.
            self._board_protocol = 1
            self._last_heartbeat = None
        self.get_logger().info(f"opened serial port '{ser.port}' @ {ser.baudrate} baud")
        return True

    def _disconnect(self, reason: str) -> None:
        with self._io_lock:
            ser, self._serial = self._serial, None
        if ser is None:
            return
        self._disconnects += 1
        self.get_logger().error(f"{reason}: serial link lost, reconnecting")
        try:
            ser.close()
        except (serial.SerialException, OSError):
            pass

    def _write(self, command: str) -> bool:
        error: Exception
        with self._io_lock:
            ser = self._serial
            if ser is None:
                return False
            try:
                ser.write(command.encode("ascii"))
                return True
            except serial.SerialTimeoutException as exc:
                # The board stopped reading but the port is there: not a disconnect.
                self.get_logger().warning(
                    f"serial write timed out: {exc}", throttle_duration_sec=1.0
                )
                return False
            except (serial.SerialException, OSError) as exc:
                error = exc
        # Outside the lock, which _disconnect() takes.
        self._disconnect(f"serial write failed ({error})")
        return False

    def _reader_loop(self) -> None:
        partial = b""
        while not self._reader_stop.is_set():
            ser = self._serial
            if ser is None:
                partial = b""
                if not self._connect():
                    self._reader_stop.wait(self.get_parameter("reconnect_period").value)
                continue
            try:
                chunk = ser.readline()
            except (serial.SerialException, OSError, TypeError, AttributeError) as exc:
                # pyserial raises TypeError/AttributeError when the port is closed under it.
                if not self._reader_stop.is_set():
                    self._disconnect(f"serial read failed ({exc})")
                continue

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
        parsed = parse_line(line)
        if parsed is None:
            return
        kind, values = parsed
        now = time.monotonic()
        if kind == "kill":
            self._killed = values[0]
            self._last_kill = now
            if self._kill_pub is not None:
                self._kill_pub.publish(Bool(data=values[0]))
        elif kind == "depth":
            self._depth = values[0]
            self._last_depth = now
            self._publish_depth(values[0])
        elif kind == "heartbeat":
            protocol, uptime_ms, timeouts = values
            if self._board_uptime_ms is not None and uptime_ms < self._board_uptime_ms:
                self._board_restarts += 1
                self.get_logger().error(
                    f"the board restarted (uptime {uptime_ms} ms, was {self._board_uptime_ms} ms)"
                )
            if timeouts > self._board_timeouts:
                self.get_logger().error(
                    "the board stopped the thrusters itself: no thruster commands reached it"
                )
            if protocol != self._board_protocol:
                self.get_logger().info(f"board speaks protocol {protocol}")
            self._board_protocol = protocol
            self._board_uptime_ms = uptime_ms
            self._board_timeouts = timeouts
            self._last_heartbeat = now

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
        with self._io_lock:
            ser, self._serial = self._serial, None
        if ser is not None:
            try:
                ser.close()
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warning(f"error closing serial port: {exc}")
        self._is_active = False
        self._destroy_entities()

    def _stop_reader(self) -> None:
        self._reader_stop.set()
        if self._reader_thread is not None:
            # The read times out within serial_timeout, so the thread sees the flag.
            self._reader_thread.join(timeout=READER_JOIN_TIMEOUT)
            if self._reader_thread.is_alive():
                self.get_logger().warning("serial reader thread did not stop cleanly")
            self._reader_thread = None

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
        if self._thruster_sub is not None:
            self.destroy_subscription(self._thruster_sub)
            self._thruster_sub = None
        if self._watchdog is not None:
            self.destroy_timer(self._watchdog)
            self._watchdog = None

    # --- thrusters

    def _protocol(self) -> int:
        """The protocol the board speaks now: 1 unless it has said otherwise lately."""
        heartbeat = self._last_heartbeat
        if heartbeat is None or time.monotonic() - heartbeat > HEARTBEAT_TIMEOUT:
            return 1
        return self._board_protocol

    def _thrusters_callback(self, msg: ThrusterCommand) -> None:
        if not self._is_active:
            return
        timeout = self.get_parameter("command_timeout").value
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        if age > timeout:
            # Delayed in the graph: acting on it would replay the past. The watchdog
            # stops the thrusters if nothing fresher comes.
            self._stale_commands += 1
            self.get_logger().error(
                f"ignoring a thruster command {age:.2f} s old", throttle_duration_sec=1.0
            )
            return
        self._last_command = time.monotonic()
        self._stopped = False
        commands = [sanitize(c) for c in msg.command]
        if any(not math.isfinite(c) for c in msg.command):
            self.get_logger().error(
                f"thruster commands {list(msg.command)} are not all finite: stopping those",
                throttle_duration_sec=1.0,
            )
        self._write(thruster_lines(commands, self._protocol()))

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

    # --- actuators

    def _launch_torpedo_callback(self, request, response):
        if not self._is_active or self._serial is None:
            response.success = False
            response.message = "sub_low is not active or the board is not connected."
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
            response.message = "sub_low is not active or the board is not connected."
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

    # --- /diagnostics

    @staticmethod
    def _since(stamp: float | None) -> str:
        return "never" if stamp is None else f"{time.monotonic() - stamp:.2f} s ago"

    def _diagnose_link(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        connected = self._serial is not None
        protocol = self._protocol()
        stat.add("device", self.get_parameter("device").value)
        stat.add("connected", str(connected))
        stat.add("disconnects", str(self._disconnects))
        stat.add("protocol", str(protocol))
        stat.add("last heartbeat", self._since(self._last_heartbeat))
        stat.add("board uptime [ms]", str(self._board_uptime_ms))
        stat.add("board restarts seen", str(self._board_restarts))
        stat.add("board watchdog stops", str(self._board_timeouts))
        if not connected:
            stat.summary(DiagnosticStatus.ERROR, "board not connected")
        elif protocol < BATCH_PROTOCOL:
            stat.summary(
                DiagnosticStatus.WARN,
                "no heartbeat from the board: older firmware without a thruster watchdog?",
            )
        else:
            stat.summary(DiagnosticStatus.OK, "connected")
        return stat

    def _diagnose_board_state(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        now = time.monotonic()
        stat.add("killed", str(self._killed))
        stat.add("kill switch reported", self._since(self._last_kill))
        stat.add("depth [m]", str(self._depth))
        stat.add("depth reported", self._since(self._last_depth))
        stale = [
            name
            for name, stamp in (("kill switch", self._last_kill), ("depth", self._last_depth))
            if stamp is None or now - stamp > BOARD_STATE_TIMEOUT
        ]
        if stale:
            stat.summary(DiagnosticStatus.ERROR, "not reported: " + ", ".join(stale))
        else:
            stat.summary(DiagnosticStatus.OK, "killed" if self._killed else "released")
        return stat

    def _diagnose_commands(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        stat.add("active", str(self._is_active))
        stat.add("last command", self._since(self._last_command))
        stat.add("stale commands ignored", str(self._stale_commands))
        if not self._is_active:
            stat.summary(DiagnosticStatus.WARN, "inactive: thruster commands are ignored")
        elif self._stopped:
            stat.summary(DiagnosticStatus.WARN, "stopped: no thruster commands from sub_control")
        else:
            stat.summary(DiagnosticStatus.OK, "commanding")
        return stat


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
