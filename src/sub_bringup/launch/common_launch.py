"""
Common launch arguments and nodes between the simulation and physical vehicle:
transforms, EKF, sub_control, the mission, the tuning dashboard, the
diagnostics aggregator and the Foxglove bridge.

An include shares the parent's launch configurations, so the parent's
arguments (entities.common_launch_arguments) reach the nodes below.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotEqualsSubstitution,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from sub_bringup.entities import common_launch_arguments, package_config, sim_time
from sub_bringup.vehicle import load_vehicle


def _sub_control(context, *_, **__):
    """The controller, which needs the thruster layout from the vehicle description."""
    robot_name = LaunchConfiguration("robot_name")
    vehicle = load_vehicle(robot_name.perform(context))
    return [
        Node(
            package="sub_control",
            executable="sub_control",
            name="sub_control",
            namespace=robot_name,
            output="both",
            parameters=[
                package_config("sub_bringup", LaunchConfiguration("gains")),
                {"robot_name": robot_name, **vehicle.thruster_layout()},
                sim_time(),
            ],
        )
    ]


def _diagnostic_aggregator(context, *_, **__):
    """Groups every node's /diagnostics into /diagnostics_agg (config/diagnostics.yaml).

    Marks an item that stops reporting as stale. Not namespaced: the diagnostics
    topics are global. On the vehicle (not on simulation time) it groups the
    hardware drivers too; in the simulator that group would be empty, which
    reads as stale.
    """
    configs = ["diagnostics.yaml"]
    if not IfCondition(LaunchConfiguration("use_sim_time")).evaluate(context):
        configs.append("diagnostics_vehicle.yaml")
    return [
        Node(
            package="diagnostic_aggregator",
            executable="aggregator_node",
            name="diagnostic_aggregator",
            output="log",
            parameters=[*(package_config("sub_bringup", c) for c in configs), sim_time()],
        )
    ]


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    mission = LaunchConfiguration("mission")

    # The vehicle's static transforms, from config/vehicles/<robot_name>.yaml.
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
            # No absolute reference (like GPS), so keep world_frame as odom instead of map
            {
                "odom_frame": [robot_name, "/odom"],
                "base_link_frame": [robot_name, "/base_link"],
                "world_frame": [robot_name, "/odom"],
            },
            sim_time(),
        ],
        # Its INFO logs are noise: every set_pose (sub_control's pose reset on
        # kill release) dumps the whole request, covariance included.
        ros_arguments=["--log-level", "warn"],
    )

    # Run only when mission is passed. Under restart, sub_mission's supervisor
    # runs it in a child process instead (docs/mission.md#restart-supervisor).
    has_mission = NotEqualsSubstitution(mission, "")
    restart = LaunchConfiguration("restart")
    mission_parameters = [
        {
            "mission": mission,
            "role": LaunchConfiguration("role"),
            "groot_port": ParameterValue(LaunchConfiguration("groot_port"), value_type=int),
        },
        sim_time(),
    ]
    mission_node = Node(
        package="sub_mission",
        executable="mission",
        name="sub_mission",
        namespace=robot_name,
        output="screen",
        parameters=mission_parameters,
        condition=IfCondition(AndSubstitution(has_mission, NotSubstitution(restart))),
    )
    mission_restart_node = Node(
        package="sub_mission",
        executable="restart",
        name="sub_mission_restart",
        namespace=robot_name,
        output="screen",
        parameters=mission_parameters,
        condition=IfCondition(AndSubstitution(has_mission, restart)),
    )

    # Saves to the gains file sub_control was started with; installed as a
    # symlink to the source tree, so that is where the change lands. Serves on
    # every interface, like the Foxglove bridge and the Groot2 view.
    dashboard = Node(
        package="sub_pid_tuner",
        executable="dashboard",
        name="dashboard",
        namespace=robot_name,
        output="screen",
        parameters=[
            {
                "profile": package_config("sub_bringup", LaunchConfiguration("gains")),
                "host": "0.0.0.0",
                "port": ParameterValue(LaunchConfiguration("dashboard_port"), value_type=int),
            },
            sim_time(),
        ],
        condition=IfCondition(LaunchConfiguration("dashboard")),
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[
            {
                "port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int),
                "use_compression": ParameterValue(
                    LaunchConfiguration("foxglove_compression"), value_type=bool
                ),
            },
            sim_time(),
        ],
        ros_arguments=["--disable-stdout-logs"],
    )

    return LaunchDescription(
        [
            *common_launch_arguments(),
            description,
            ekf_node,
            OpaqueFunction(function=_sub_control),
            mission_node,
            mission_restart_node,
            dashboard,
            OpaqueFunction(function=_diagnostic_aggregator),
            foxglove_bridge,
        ]
    )
