"""Static transforms for the vehicle named by robot_name.

The geometry itself is in sub_bringup/config/<robot_name>.yaml; this file only
turns it into static_transform_publisher nodes. Included by common_launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from sub_bringup.vehicle import load_vehicle


def _transforms(context, *_, **__):
    vehicle = load_vehicle(LaunchConfiguration("robot_name").perform(context))
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=args,
            ros_arguments=["--disable-stdout-logs"],
        )
        for args in vehicle.transform_arguments()
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="marlin_v3"),
            OpaqueFunction(function=_transforms),
        ]
    )
