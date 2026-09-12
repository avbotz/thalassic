"""Bring the stack up against the Stonefish simulator.

Renders a randomised scenario, starts Stonefish and the sensor bridges, then
the same estimation, control, vision and mission stack the vehicle runs.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from sub_bringup.entities import common_launch_include, vision_node
from sub_sim.generate_robot import render_robot_scenario
from sub_sim.randomize_locs import randomize_scenario_locations
from sub_sim.robot_scenario_to_urdf import robot_scenario_to_urdf


def _render_scn(context, *_, **__):
    DX = float(LaunchConfiguration("DX").perform(context))
    DY = float(LaunchConfiguration("DY").perform(context))
    DZ = float(LaunchConfiguration("DZ").perform(context))
    DYAW = float(LaunchConfiguration("DYAW").perform(context))

    try:
        SEED = int(LaunchConfiguration("seed").perform(context))
    except ValueError:
        SEED = None

    robot_name = LaunchConfiguration("robot_name").perform(context)

    sub_sim_share = Path(get_package_share_directory("sub_sim"))

    scenario_file = sub_sim_share / "scenarios" / "woollett.scn.j2"
    robot_scenario_file = sub_sim_share / "data" / "robots" / robot_name / "layout.scn.j2"

    rendered_robot_path = None

    def render_robot(**pose):
        nonlocal rendered_robot_path
        rendered_robot_path = render_robot_scenario(robot_scenario_file, **pose)
        return rendered_robot_path.as_posix()

    temp_path = randomize_scenario_locations(
        scenario_template_file=scenario_file,
        DX=DX,
        DY=DY,
        DZ=DZ,
        DYAW=DYAW,
        seed=SEED,
        render_robot=render_robot,
    )

    if rendered_robot_path is None:
        raise RuntimeError("No robot rendered")

    urdf_robot = robot_scenario_to_urdf(
        scenario_xml=rendered_robot_path,
        robot_name=robot_name,
        mesh_prefix=f"file://{sub_sim_share / 'data'}/",
    )

    robot_description = urdf_robot.read_text()

    return [
        SetLaunchConfiguration("scenario_file", temp_path.as_posix()),
        SetLaunchConfiguration("robot_description", robot_description),
    ]


def sim_entities() -> list:
    robot_name = LaunchConfiguration("robot_name")

    args = [
        DeclareLaunchArgument("seed", default_value=""),
        DeclareLaunchArgument("DX", default_value="0.25"),
        DeclareLaunchArgument("DY", default_value="0.25"),
        DeclareLaunchArgument("DZ", default_value="0.10"),
        DeclareLaunchArgument("DYAW", default_value="0.10"),
        DeclareLaunchArgument(
            "labeling",
            default_value="false",
            description="Write labelled training images from the segmentation camera",
        ),
    ]

    include_stonefish = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("stonefish_ros2"),
                "launch",
                "stonefish_simulator.launch.py",
            ]
        ),
        launch_arguments={
            "simulation_data": PathJoinSubstitution([FindPackageShare("sub_sim"), "data"]),
            "scenario_desc": LaunchConfiguration("scenario_file"),
            "simulation_rate": "500.0",
            "window_res_x": "1900",
            "window_res_y": "1000",
            "rendering_quality": "medium",
        }.items(),
    )

    def sim_component(executable, plugin, **kwargs):
        return ComposableNode(
            package="sub_sim_sensors",
            plugin=plugin,
            name=executable,
            namespace=robot_name,
            extra_arguments=[{"use_intra_process_comms": True}],
            **kwargs,
        )

    sim_sensors_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        name="sim_sensors_container",
        namespace=robot_name,
        output="both",
        composable_node_descriptions=[
            sim_component(
                "sim_dvl_remapper",
                "SimDVLRemapper",
                parameters=[{"dvl_link": [robot_name, "/dvl_link"]}],
            ),
            sim_component(
                "sim_imu_remapper",
                "SimIMURemapper",
                parameters=[{"imu_link": [robot_name, "/imu_link"]}],
            ),
            sim_component(
                "sim_pressure_to_depth",
                "SimPressureToDepth",
                parameters=[
                    {
                        "frame_id": [robot_name, "/odom"],
                        "child_frame_id": [robot_name, "/base_link"],
                    }
                ],
            ),
            sim_component("sim_thruster_republisher", "SimThrusterRepublisher"),
            sim_component("sim_torpedo_launcher", "SimTorpedoLauncher"),
            sim_component(
                "sim_dropper",
                "SimDropper",
                parameters=[{"joint_name": [robot_name, "/dropper_joint"]}],
            ),
            sim_component(
                "sim_kill_switch",
                "SimKillSwitch",
                parameters=[{"off_delay": 6.0}],
            ),
        ],
        ros_arguments=["--disable-stdout-logs"],
    )

    # Publishes the URDF rendered from the scenario; the vehicle has no equivalent.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        parameters=[{"robot_description": LaunchConfiguration("robot_description")}],
    )

    sim_labeling_node = Node(
        package="sim_labeling",
        executable="labeling",
        name="sim_labeling",
        namespace=robot_name,
        condition=IfCondition(LaunchConfiguration("labeling")),
        parameters=[
            {
                "scenario_file": LaunchConfiguration("scenario_file"),
                "output_dir": "train_imgs",
                # Absolute, like the node's own defaults. The segmentation
                # camera is aligned with the scenario's front camera, not the
                # OAK camera the node's front_cam_topic default still names.
                "seg_topic": ["/", robot_name, "/sim/segment/image_raw"],
                "front_cam_topic": ["/", robot_name, "/front_camera/image_color"],
            }
        ],
    )

    return [
        *args,
        OpaqueFunction(function=_render_scn),
        include_stonefish,
        sim_sensors_container,
        robot_state_publisher,
        sim_labeling_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            # Transforms, EKF, controller, mission executive, Foxglove bridge.
            *common_launch_include("control_gains_sim.yaml"),
            *sim_entities(),
            # Stonefish publishes both cameras as rgb8 on image_color; cv_bridge
            # converts to bgr8 inside the node, so no bridge is needed.
            vision_node("sub_vision", "sub_vision.yaml", "front_camera", "image_color"),
            vision_node("sub_vision_down", "sub_vision_down.yaml", "down_camera", "image_color"),
            Node(
                package="sub_vision",
                executable="annotation_visualizer",
                name="annotation_visualizer",
                namespace=LaunchConfiguration("robot_name"),
                output="screen",
            ),
        ]
    )
