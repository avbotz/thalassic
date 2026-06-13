"""Lifecycle driver bridging the sub_low control board over a serial port.

configure  -> open the serial port, create the kill-switch publisher
activate   -> start forwarding thruster commands and polling the board
deactivate -> stop the thrusters and stop forwarding/polling
cleanup    -> close the serial port
shutdown   -> stop the thrusters and close the serial port
"""

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, Float64
from sub_driver_interfaces.srv import LaunchTorpedo, SetDropper

from sub_serial_drivers.serial_port import SerialPort

NUM_THRUSTERS = 8
NUM_TORPEDO_THRUSTERS = 2
MAX_RX_BUFFER = 4096


class SubLow(LifecycleNode):
    """Forwards thruster / actuator commands to the board and publishes the kill switch."""

    def __init__(self, **kwargs):
        super().__init__("sub_low", **kwargs)
        self.declare_parameter("device", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)

        self._serial: SerialPort | None = None
        self._rx_buffer = bytearray()
        self._is_active = False

        self._thruster_subs = []
        self._kill_pub = None
        self._launch_torpedo_srv = None
        self._set_dropper_srv = None
        self._poll_timer = None

    # --- lifecycle transitions -------------------------------------------------

    def on_configure(self, state) -> TransitionCallbackReturn:
        device = self.get_parameter("device").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        try:
            self._serial = SerialPort(device, baud)
        except Exception as exc:  # noqa: BLE001 - report any open/config failure
            self.get_logger().error(f"could not open serial: {exc}")
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info(f"sub_low connected to {device} @ {baud} baud")

        kill_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._kill_pub = self.create_lifecycle_publisher(Bool, "kill_switch", kill_qos)

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

        self._poll_timer = self.create_timer(0.005, self._poll_timer_callback)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self._is_active = True
        self.get_logger().info("sub_low active")
        return super().on_activate(state)

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._is_active = False
        self._stop_thrusters()
        self.get_logger().info("sub_low deactivated")
        return super().on_deactivate(state)

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self._is_active = False
        self._stop_thrusters()
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def _teardown(self) -> None:
        if self._poll_timer is not None:
            self.destroy_timer(self._poll_timer)
            self._poll_timer = None
        if self._kill_pub is not None:
            self.destroy_publisher(self._kill_pub)
            self._kill_pub = None
        if self._launch_torpedo_srv is not None:
            self.destroy_service(self._launch_torpedo_srv)
            self._launch_torpedo_srv = None
        if self._set_dropper_srv is not None:
            self.destroy_service(self._set_dropper_srv)
            self._set_dropper_srv = None
        for sub in self._thruster_subs:
            self.destroy_subscription(sub)
        self._thruster_subs = []
        if self._serial is not None:
            self._serial.close()
            self._serial = None
        self._rx_buffer.clear()

    # --- command handling ------------------------------------------------------

    def _make_thruster_callback(self, index: int):
        def callback(msg: Float64) -> None:
            if self._is_active:
                self._set_thruster_power(index, msg.data)

        return callback

    def _set_thruster_power(self, index: int, normalized: float) -> None:
        if not self._serial.write(f"p {index} {normalized}\n"):
            self.get_logger().warn("serial write failed", throttle_duration_sec=1.0)

    def _launch_torpedo_callback(self, request, response):
        if not self._is_active:
            response.success = False
            response.message = "sub_low is not active."
            return response

        if request.torpedo_id >= NUM_TORPEDO_THRUSTERS:
            response.success = False
            response.message = f"Invalid torpedo id {request.torpedo_id}."
            return response

        if not self._serial.write(f"t {request.torpedo_id} {1 if request.open else 0}\n"):
            response.success = False
            response.message = "serial write failed"
            return response

        response.success = True
        state = "opened" if request.open else "closed"
        response.message = f"Torpedo thruster {request.torpedo_id} {state}."
        return response

    def _set_dropper_callback(self, request, response):
        if not self._is_active:
            response.success = False
            response.message = "sub_low is not active."
            return response

        if not self._serial.write(f"d {1 if request.open else 0}\n"):
            response.success = False
            response.message = "serial write failed"
            return response

        response.success = True
        response.message = f"Dropper {'opened' if request.open else 'closed'}."
        return response

    def _stop_thrusters(self) -> None:
        if self._serial is None:
            return
        self._serial.write("a 0\n")

    # --- serial polling --------------------------------------------------------

    def _poll_timer_callback(self) -> None:
        if self._is_active:
            self._poll_serial()

    def _poll_serial(self) -> None:
        chunk = self._serial.read_available()
        if chunk is None:
            self.get_logger().error(
                f"serial read error: {self._serial.last_error}",
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
        fields = line.split()
        if not fields:
            return

        tag = fields[0]
        if tag == b"x":
            try:
                value = int(fields[1])
            except (IndexError, ValueError):
                return
            msg = Bool()
            msg.data = bool(value)
            self._kill_pub.publish(msg)
        elif tag == b"d":
            # Dropper acknowledgement (float), currently ignored.
            return


def main(args=None):
    rclpy.init(args=args)
    node = SubLow()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
