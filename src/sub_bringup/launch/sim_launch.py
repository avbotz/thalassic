from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.actions.node import ExecuteProcess
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from sub_sim.generate_robot import render_robot_scenario
from sub_sim.randomize_locs import randomize_scenario_locations
from sub_sim.robot_scenario_to_urdf import robot_scenario_to_urdf

ROBOT_NAME = "marlin_v2"


def _render_scn(context, *_, **__):
    DX = float(LaunchConfiguration("DX").perform(context))
    DY = float(LaunchConfiguration("DY").perform(context))
    DZ = float(LaunchConfiguration("DZ").perform(context))
    DYAW = float(LaunchConfiguration("DYAW").perform(context))
    SEED = (
        int(LaunchConfiguration("seed").perform(context))
        if LaunchConfiguration("seed").perform(context).isdigit()
        else None
    )

    sub_sim_share = Path(get_package_share_directory("sub_sim"))

    scenario_file = sub_sim_share / "scenarios" / "woollett.scn.j2"
    robot_scenario_file = (
        sub_sim_share / "data" / "robots" / ROBOT_NAME / "layout.scn.j2"
    )

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
        robot_name=ROBOT_NAME,
        mesh_prefix=f"file://{sub_sim_share / 'data'}/",
    )

    robot_description = urdf_robot.read_text()

    return [
        SetLaunchConfiguration("scenario_file", temp_path.as_posix()),
        SetLaunchConfiguration("robot_description", robot_description),
    ]


def sim_entities() -> list:
    args = [
        DeclareLaunchArgument("seed", default_value=""),
        DeclareLaunchArgument("DX", default_value="0.25"),
        DeclareLaunchArgument("DY", default_value="0.25"),
        DeclareLaunchArgument("DZ", default_value="0.10"),
        DeclareLaunchArgument("DYAW", default_value="0.10"),
    ]

    render = OpaqueFunction(function=_render_scn)

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
            namespace=LaunchConfiguration("ns"),
            extra_arguments=[{"use_intra_process_comms": True}],
            **kwargs,
        )

    sim_sensors_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        name="sim_sensors_container",
        namespace=LaunchConfiguration("ns"),
        output="both",
        composable_node_descriptions=[
            sim_component(
                "sim_dvl_remapper",
                "SimDVLRemapper",
                parameters=[{"robot_name": ROBOT_NAME}],
            ),
            sim_component(
                "sim_imu_remapper",
                "SimIMURemapper",
                parameters=[{"robot_name": ROBOT_NAME}],
            ),
            sim_component("sim_thruster_republisher", "SimThrusterRepublisher"),
            sim_component("sim_torpedo_launcher", "SimTorpedoLauncher"),
            sim_component("sim_dropper", "SimDropper"),
            sim_component(
                "sim_kill_switch",
                "SimKillSwitch",
                parameters=[{"off_delay": 8.0}],
            ),
        ],
        ros_arguments=["--disable-stdout-logs"],
    )

    sim_labeling_node = Node(
        package="sim_labeling",
        executable="labeling",
        name="sim_labeling",
        namespace=LaunchConfiguration("ns"),
        parameters=[
            {
                "scenario_file": LaunchConfiguration("scenario_file"),
                "output_dir": "train_imgs",
            }
        ],
    )

    return [
        *args,
        render,
        include_stonefish,
        sim_sensors_container,
        # sim_labeling_node,
    ]


def control_and_state_entities() -> list[Node]:
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=LaunchConfiguration("ns"),
        parameters=[
            {
                "robot_description": LaunchConfiguration("robot_description"),
            }
        ],
    )

    # # RTAB-Map RGBD sync to handle timestamp differences between cameras
    # rgbd_sync = Node(
    #     package="rtabmap_sync",
    #     executable="rgbd_sync",
    #     name="rgbd_sync",
    #     namespace=LaunchConfiguration("ns"),
    #     output="screen",
    #     parameters=[
    #         {
    #             "approx_sync": True,
    #             "approx_sync_max_interval": 1.0,  # Very permissive for sim
    #             "qos": 1,
    #             "qos_image": 1,
    #             "qos_camera_info": 1,
    #         }
    #     ],
    #     remappings=[
    #         ("rgb/image", f"/{ROBOT_NAME}/oak/rgb/image_raw"),
    #         ("depth/image", f"/{ROBOT_NAME}/oak/stereo/image_raw"),
    #         ("rgb/camera_info", f"/{ROBOT_NAME}/oak/rgb/camera_info"),
    #         ("rgbd_image", f"/{ROBOT_NAME}/rgbd_image"),
    #     ],
    # )

    # # RTAB-Map visual odometry using synchronized RGBD
    # depth_camera_visual_odom = Node(
    #     package="rtabmap_odom",
    #     executable="rgbd_odometry",
    #     name="rgbd_odometry",
    #     namespace=LaunchConfiguration("ns"),
    #     output="screen",
    #     parameters=[
    #         {
    #             "frame_id": f"{ROBOT_NAME}/base_link",
    #             "odom_frame_id": f"{ROBOT_NAME}/front_camera",
    #             "publish_tf": False,  # Let robot_localization handle TF
    #             "subscribe_depth": False,
    #             "subscribe_rgbd": True,  # Use synchronized RGBD topic
    #             "wait_for_transform": 0.2,
    #             "qos": 1,
    #             # RTAB-Map internal parameters
    #             "Odom/Strategy": "0",  # 0=Frame-to-Map, 1=Frame-to-Frame
    #             "Odom/ResetCountdown": "1",
    #             "Vis/MaxFeatures": "500",
    #             "Vis/MinInliers": "10",
    #         }
    #     ],
    #     remappings=[
    #         ("rgbd_image", f"/{ROBOT_NAME}/rgbd_image"),
    #         ("odom", f"/{ROBOT_NAME}/odom/depth_camera"),
    #     ],
    # )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="both",
        namespace=LaunchConfiguration("ns"),
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("sub_bringup"),
                    "config/ekf.yaml",
                ]
            ),
        ],
    )

    sub_control_node = Node(
        package="sub_control",
        executable="sub_control",
        name="sub_control",
        output="both",
        namespace=LaunchConfiguration("ns"),
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("sub_bringup"),
                    "config/control_gains_sim.yaml",
                ]
            ),
            {"robot_name": ROBOT_NAME},
        ],
    )

    return [
        robot_state_publisher,
        # Depth camera not on marlin
        # rgbd_sync,
        # depth_camera_visual_odom,
        robot_localization_node,
        sub_control_node,
    ]


def foxglove_entities() -> list:
    clear_port = ExecuteProcess(
        cmd=["fuser", "-k", "8765/tcp"],  # free port 8765 (automatic foxglove bridge)
        output="screen",
    )

    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[
            {
                "port": 8765,
                "use_compression": True,
                "use_sim_time": True,
            }
        ],
        ros_arguments=["--disable-stdout-logs"],
    )

    bridge_after_port_clear = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=clear_port,
            on_exit=[foxglove_bridge_node],
        )
    )

    return [clear_port, bridge_after_port_clear]


def vision_entities() -> list[Node | IncludeLaunchDescription]:
    # Depth camera not on marlin
    # deepseecolor_node = Node(
    #     package="sub_color_correction",
    #     executable="deepseecolor_node",
    #     name="deepseecolor",
    #     namespace=LaunchConfiguration("ns"),
    #     output="screen",
    #     condition=IfCondition(LaunchConfiguration("enable_deepseecolor")),
    #     parameters=[
    #         {
    #             "rgb_topic": f"/{ROBOT_NAME}/oak/rgb/image_raw",
    #             "depth_topic": f"/{ROBOT_NAME}/oak/stereo/image_raw",
    #             "corrected_topic": f"/{ROBOT_NAME}/oak/rgb/image_color_corrected",
    #             "device": LaunchConfiguration("deepseecolor_device"),
    #             "init_iters": 10,
    #             "iters": 2,
    #             "max_inference_dimension": 640,
    #             "sync_slop": 0.15,
    #         }
    #     ],
    # )

    sub_vision_node = Node(
        package="sub_vision",
        executable="sub_vision",
        name="sub_vision",
        output="screen",
        namespace=LaunchConfiguration("ns"),
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("sub_vision"),
                    "config/sub_vision.yaml",
                ]
            ),
        ],
    )

    return [
        sub_vision_node,
        # deepseecolor_node,
    ]


def generate_launch_description():
    declare_ns = DeclareLaunchArgument("ns", default_value=ROBOT_NAME)

    include_transforms = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("sub_bringup"), "launch", f"{ROBOT_NAME}_launch.py"]
        ),
    )

    return LaunchDescription(
        [
            declare_ns,
            include_transforms,
            *sim_entities(),
            *control_and_state_entities(),
            *vision_entities(),
            *foxglove_entities(),
        ]
    )
