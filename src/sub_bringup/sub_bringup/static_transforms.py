"""
Publishes every static transform of the vehicle description from one node.

Avoids issues with limited number of DDS participants and decreases clutter
in node list.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from sub_bringup.vehicle import load_vehicle


class StaticTransforms(Node):
    def __init__(self):
        super().__init__("static_transforms")
        robot_name = self.declare_parameter("robot_name", "").value
        if not robot_name:
            raise ValueError("robot_name is not set; it selects config/vehicles/<robot_name>.yaml")

        transforms = list(load_vehicle(robot_name).transforms)
        stamp = self.get_clock().now().to_msg()
        for transform in transforms:
            transform.header.stamp = stamp

        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(transforms)
        self.get_logger().info(f"Publishing {len(transforms)} static transforms of {robot_name}")


def main(args=None):
    rclpy.init(args=args)
    node = StaticTransforms()
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
