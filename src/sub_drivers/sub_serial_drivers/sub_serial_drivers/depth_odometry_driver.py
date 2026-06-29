import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64


class DepthOdometryDriver(Node):
    def __init__(self):
        super().__init__("depth_odometry_driver")

        self.declare_parameter("frame_id", "marlin_v2/odom")
        self.declare_parameter("child_frame_id", "marlin_v2/base_link")
        self.declare_parameter("z_variance", 0.01)

        self._frame_id = self.get_parameter("frame_id").value
        self._child_frame_id = self.get_parameter("child_frame_id").value
        self._z_variance = self.get_parameter("z_variance").value

        self._publisher = self.create_publisher(
            Odometry, "odometry/depth", qos_profile_sensor_data
        )
        self.create_subscription(
            Float64, "depth", self._depth_callback, qos_profile_sensor_data
        )

    def _depth_callback(self, msg: Float64) -> None:
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = self._child_frame_id
        odom.pose.pose.position.z = -msg.data
        odom.pose.pose.orientation.w = 1.0
        odom.pose.covariance[14] = self._z_variance

        self._publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = DepthOdometryDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
