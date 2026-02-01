import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
    SetLaunchConfiguration,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory

from launch.event_handlers import OnProcessExit

from sub_sim.generate_robot import render_robot_scenario
from sub_sim.randomize_locs import randomize_scenario_locations
from sub_sim.robot_scenario_to_urdf import robot_scenario_to_urdf


def _render_scn(context, *_, **__):
    lc = lambda k: LaunchConfiguration(k).perform(context)

    DX = float(lc("DX"))
    DY = float(lc("DY"))
    DZ = float(lc("DZ"))
    DYAW = float(lc("DYAW"))
    SEED = int(lc("seed")) if lc("seed") != "" and lc("seed").isdigit() else None

    sub_sim_share = get_package_share_directory("sub_sim")

    scenario_file = Path(sub_sim_share) / "scenarios" / "woollett.scn.j2"
    robot_scenario_file = (
        Path(sub_sim_share) / "data" / "robots" / "marlin_v2" / "layout.scn.j2"
    )

    robot_rendered_path = render_robot_scenario(robot_scenario_file)

    urdf_robot = robot_scenario_to_urdf(
        scenario_xml=robot_rendered_path,
        robot_name="marlin_v2",
        mesh_prefix=f"file://{Path(sub_sim_share) / "data"}/",
    )

    robot_description = urdf_robot.read_text()

    temp_path = randomize_scenario_locations(
        scenario_template_file=scenario_file,
        DX=DX,
        DY=DY,
        DZ=DZ,
        DYAW=DYAW,
        seed=SEED,
        ROBOT_SCENARIO_PATH=robot_rendered_path.as_posix(),
    )

    return [
        SetLaunchConfiguration("scenario_file", temp_path.as_posix()),
        SetLaunchConfiguration("robot_urdf_file", urdf_robot.as_posix()),
        SetLaunchConfiguration("robot_description", robot_description),
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument("seed", default_value=""),
        DeclareLaunchArgument("DX", default_value="0.25"),
        DeclareLaunchArgument("DY", default_value="0.25"),
        DeclareLaunchArgument("DZ", default_value="0.10"),
        DeclareLaunchArgument("DYAW", default_value="0.10"),
    ]

    render = OpaqueFunction(function=_render_scn)

    include_transforms = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("sub_bringup"), "launch", "marlin_v2_launch.py"]
        ),
    )

    include_stonefish = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("stonefish_ros2"),
                "launch",
                "stonefish_simulator.launch.py",
            ]
        ),
        launch_arguments={
            "simulation_data": PathJoinSubstitution(
                [FindPackageShare("sub_sim"), "data"]
            ),
            "scenario_desc": LaunchConfiguration("scenario_file"),
            "simulation_rate": "300.0",
            "window_res_x": "1900",
            "window_res_y": "1000",
            "rendering_quality": "medium",
            "use_sim_time": "false",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace="marlin_v2",
        parameters=[
            {
                "robot_description": LaunchConfiguration("robot_description"),
            }
        ],
    )

    sim_dvl_remapper = Node(
        package="sub_sim_sensors",
        executable="sim_dvl_remapper",
        name="sim_dvl_remapper",
        namespace="marlin_v2",
        parameters=[
            {
                "robot_name": "marlin_v2",
            }
        ],
    )

    dvl_odom_remapping = Node(
        package="sub_drivers_mappings",
        executable="dvl_odom_remapper",
        name="dvl_odom_remapper",
        namespace="marlin_v2",
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        namespace="marlin_v2",
        parameters=[
            os.path.join(get_package_share_directory("sub_bringup"), "config/ekf.yaml"),
        ],
    )

    clear_port = ExecuteProcess(
        cmd=['fuser', '-k', '8765/tcp'], #free port 8765 (automatic foxglove bridge)
        output='screen' 
    )

    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[{
                'port': 8765,
                'use_compression': True,
                'use_sim_time': True,
        }],
    )

    bridge_after_port_clear = RegisterEventHandler(
        event_handler= OnProcessExit(
            target_action=clear_port,
            on_exit=[foxglove_bridge_node],
        )
    )

    return LaunchDescription(
        args
        + [
            render,
            include_stonefish,
            include_transforms,
            robot_state_publisher,
            sim_dvl_remapper,
            dvl_odom_remapping,
            robot_localization_node,
            clear_port,
            bridge_after_port-clear,
        ]
    )
