"""
Node that subscribes to the simulated DVL topic (stonefish message)
and republishes a `marine_acoustic_msgs/msg/Dvl` message so the rest of the
stack can run unchanged in simulation.

This translator copies fields with the same name when possible and logs
any missing fields it cannot map.
"""
import rclpy
from rclpy.node import Node

try:
    from stonefish_ros2.msg import DVL as StonefishDVL
except Exception:
    StonefishDVL = None

try:
    from marine_acoustic_msgs.msg import Dvl as MarineDvl
except Exception:
    MarineDvl = None


class SimDVLBridge(Node):
    def __init__(self):
        super().__init__("sim_dvl_bridge")

        self.declare_parameter("sim_topic", "/marlin_v2/sim/dvl")
        self.declare_parameter("out_topic", "/marlin_v2/dvl")

        sim_topic = self.get_parameter("sim_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("out_topic").get_parameter_value().string_value

        if StonefishDVL is None or MarineDvl is None:
            self.get_logger().error(
                "Required message packages not available: stonefish_ros2 or marine_acoustic_msgs"
            )
            return
        self.sub = self.create_subscription(StonefishDVL, sim_topic, self.callback, 10)
        self.pub = self.create_publisher(MarineDvl, out_topic, 10)
        self.get_logger().info(f"Subscribed {sim_topic} -> Publishing {out_topic}")

    def callback(self, msg: "StonefishDVL"):
        import math
        
        # Create destination message
        out = MarineDvl()

        # Header
        out.header = msg.header

        # Velocity (geometry_msgs/Vector3) - direct copy
        out.velocity.x = msg.velocity.x
        out.velocity.y = msg.velocity.y
        out.velocity.z = msg.velocity.z

        # Velocity covariance (float64[9])
        for i in range(9):
            out.velocity_covar[i] = msg.velocity_covariance[i]

        # Altitude
        out.altitude = msg.altitude

        # Beams - marine_acoustic_msgs expects float32[4] and float64[4] arrays
        num_beams = min(len(msg.beams), 4)
        for i in range(num_beams):
            b = msg.beams[i]
            out.range[i] = float(b.range)
            out.beam_velocity[i] = float(b.velocity)
            # Approximate beam quality: 255 if valid range, 0 otherwise
            out.beam_quality[i] = 255.0 if b.range >= 0 else 0.0

        # Set beam validity flags
        out.beam_ranges_valid = num_beams > 0
        out.beam_velocities_valid = num_beams > 0
        out.num_good_beams = sum(1 for b in msg.beams[:4] if b.range >= 0)

        # Compute derived fields
        out.course_gnd = math.atan2(msg.velocity.y, msg.velocity.x)
        out.speed_gnd = math.hypot(msg.velocity.x, msg.velocity.y)

        # Set mode and type (simulation defaults)
        out.velocity_mode = MarineDvl.DVL_MODE_BOTTOM
        out.dvl_type = MarineDvl.DVL_TYPE_PISTON

        # Publish mapped message
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimDVLBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
