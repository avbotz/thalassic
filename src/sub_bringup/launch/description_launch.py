"""
Static transforms for the vehicle selected by the robot_name launch parameter.

Every transform in config/vehicles/<robot_name>.yaml, device mounts and
thrusters included, from one static_transforms node (sub_bringup/static_transforms.py
says why one node and not a static_transform_publisher each).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="marlin_v3"),
            Node(
                package="sub_bringup",
                executable="static_transforms",
                namespace=robot_name,
                parameters=[{"robot_name": robot_name}],
                ros_arguments=["--disable-stdout-logs"],
            ),
        ]
    )
