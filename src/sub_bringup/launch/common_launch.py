"""Everything the simulation and the vehicle run identically.

The two stacks differ in where sensor data comes from, not in what is done
with it: both publish the same transforms, run the same EKF and controller,
execute missions the same way, and are watched over the same Foxglove bridge.
That core lives here so it cannot be half-adopted by one of them.

Included by sim_launch.py and pool_test_launch.py, and launchable on its own
against a stack whose drivers are already up:

    ros2 launch sub_bringup common_launch.py gains:=control_gains.yaml

An include shares the parent's launch configurations, so the parent's
robot_name, mission, role, gains and foxglove_port reach the nodes below; the
arguments declared here are the defaults for running this file directly.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    NotEqualsSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from sub_bringup.entities import common_launch_arguments, package_config


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    mission = LaunchConfiguration("mission")

    # The vehicle's static transforms, from config/<robot_name>.yaml.
    description = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("sub_bringup"), "launch", "description_launch.py"])
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace=robot_name,
        output="both",
        parameters=[
            package_config("sub_bringup", "ekf.yaml"),
            # The EKF runs in odometry mode, so world_frame is the odom frame.
            {
                "odom_frame": [robot_name, "/odom"],
                "base_link_frame": [robot_name, "/base_link"],
                "world_frame": [robot_name, "/odom"],
            },
        ],
    )

    sub_control_node = Node(
        package="sub_control",
        executable="sub_control",
        name="sub_control",
        namespace=robot_name,
        output="both",
        parameters=[
            package_config("sub_bringup", LaunchConfiguration("gains")),
            {"robot_name": robot_name},
        ],
    )

    # The mission executive, run only when a mission was named.
    mission_node = Node(
        package="sub_mission",
        executable="mission",
        name="sub_mission",
        namespace=robot_name,
        output="screen",
        parameters=[{"mission": mission, "role": LaunchConfiguration("role")}],
        condition=IfCondition(NotEqualsSubstitution(mission, "")),
    )

    # The bridge releases its port when the launch shuts it down. If the port
    # is busy, the holder is a stack someone forgot to stop: `pixi run
    # reset-dds`, or bring this one up with another foxglove_port:=.
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[
            {
                "port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int),
                "use_compression": True,
            }
        ],
        ros_arguments=["--disable-stdout-logs"],
    )

    return LaunchDescription(
        [
            *common_launch_arguments(),
            description,
            ekf_node,
            sub_control_node,
            mission_node,
            foxglove_bridge,
        ]
    )
