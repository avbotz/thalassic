"""Pieces the bringup launch files build in the same way.

The nodes both stacks run identically live in launch/common_launch.py, which
both include. This module holds what each top-level file needs to describe its
own half: the shared launch arguments, and the sub_vision node both run but
wire to different cameras.
"""

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def package_config(package: str, filename: SomeSubstitutionsType) -> PathJoinSubstitution:
    """A config file inside an installed package's share directory."""
    return PathJoinSubstitution([FindPackageShare(package), "config", filename])


def common_launch_arguments(gains: str = "control_gains.yaml") -> list[DeclareLaunchArgument]:
    """The arguments every bringup launch file accepts.

    Declared by the top-level launch files and by common_launch.py, so each can
    run on its own. An include shares the parent's launch configurations, and a
    default only applies when the configuration is still unset, so the parent's
    `gains` is what reaches common_launch.py.
    """
    return [
        DeclareLaunchArgument(
            "robot_name",
            default_value="marlin_v3",
            description="Vehicle to bring up; selects config/<robot_name>.yaml",
        ),
        DeclareLaunchArgument(
            "mission",
            default_value="",
            description=(
                "Mission tree to execute (resources/missions/<name>.xml or a path); "
                "empty skips sub_mission"
            ),
        ),
        DeclareLaunchArgument(
            "role",
            default_value="SURVEY",
            description="Mission vision role: SURVEY or SEARCH",
        ),
        DeclareLaunchArgument(
            "gains",
            default_value=gains,
            description="PID gains for sub_control, a file in sub_bringup/config",
        ),
        DeclareLaunchArgument(
            "foxglove_port",
            default_value="8765",
            description="Websocket port for the Foxglove bridge; change it to run two stacks",
        ),
    ]


def common_launch_include(gains: str) -> list:
    """The arguments above plus common_launch.py: transforms, EKF, controller, mission, Foxglove."""
    return [
        *common_launch_arguments(gains),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("sub_bringup"), "launch", "common_launch.py"])
        ),
    ]


def vision_node(name: str, config_file: str, camera: str, image_topic: str) -> Node:
    """One sub_vision instance reading <camera>/<image_topic>."""
    return Node(
        package="sub_vision",
        executable="sub_vision",
        name=name,
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        parameters=[
            package_config("sub_vision", config_file),
            {
                "rgb_topic": f"{camera}/{image_topic}",
                "camera_info_topic": f"{camera}/camera_info",
                "image_transport": "raw",
            },
        ],
    )
