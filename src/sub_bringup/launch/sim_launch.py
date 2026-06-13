from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
    GroupAction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node, SetRemap
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
    SEED = int(LaunchConfiguration("seed").perform(context)) if LaunchConfiguration("seed").perform(context).isdigit() else None

    sub_sim_share = Path(get_package_share_directory("sub_sim"))

    scenario_file = sub_sim_share / "scenarios" / "woollett.scn.j2"
    robot_scenario_file = sub_sim_share / "data" / "robots" / ROBOT_NAME / "layout.scn.j2"

    rendered_robot_path = None

    def render_robot(**pose):
        nonlocal rendered_robot_path
        path = render_robot_scenario(robot_scenario_file, **pose)
        rendered_robot_path = path
        return path.as_posix()

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
        SetLaunchConfiguration("robot_urdf_file", urdf_robot.as_posix()),
        SetLaunchConfiguration("robot_description", robot_description),
    ]


def sim_entities() -> list[Node]:
    include_stonefish = GroupAction(
        [
            SetRemap(
                src="/marlin_v2/sim/front_camera/camera_info",
                dst="/marlin_v2/front_camera/camera_info",
            ),
            SetRemap(
                src="/marlin_v2/sim/front_camera/image_color",
                dst="/marlin_v2/front_camera/image_color",
            ),
            SetRemap(
                src="/marlin_v2/sim/front_camera/image_depth",
                dst="/marlin_v2/front_camera/image_depth",
            ),
            IncludeLaunchDescription(
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
        ]
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

    # All sim sensor/actuator shims share one process. The container is
    # multithreaded because sim_torpedo_launcher blocks on the glue service
    # inside its service callback.
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
                parameters=[{"off_delay": 5.0}],
            ),
            # sim_component("sim_oak_camera_remapper", "SimOakCameraRemapper"),
        ],
    )

    sim_labeling_node = Node(
        package="sim_labeling",
        executable="labeling",
        name="sim_labeling",
        namespace=ROBOT_NAME,
        parameters=[
            {
                "scenario_file": LaunchConfiguration("scenario_file"),
                "output_dir": "train_imgs",
            }
        ],
    )

    return [
        include_stonefish,
        sim_sensors_container,
        sim_labeling_node,
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

    dvl_odom_remapping = Node(
        package="sub_drivers_mappings",
        executable="dvl_odom_remapper",
        name="dvl_odom_remapper",
        namespace=LaunchConfiguration("ns"),
        parameters=[
            {
                "robot_name": LaunchConfiguration("ns"),
            }
        ],
    )

    # RTAB-Map RGBD sync to handle timestamp differences between cameras
    rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync",
        namespace=LaunchConfiguration("ns"),
        output="screen",
        parameters=[
            {
                "approx_sync": True,
                "approx_sync_max_interval": 1.0,  # Very permissive for sim
                "qos": 1,
                "qos_image": 1,
                "qos_camera_info": 1,
            }
        ],
        remappings=[
            ("rgb/image", f"/{ROBOT_NAME}/oak/rgb/image_raw"),
            ("depth/image", f"/{ROBOT_NAME}/oak/stereo/image_raw"),
            ("rgb/camera_info", f"/{ROBOT_NAME}/oak/rgb/camera_info"),
            ("rgbd_image", f"/{ROBOT_NAME}/rgbd_image"),
        ],
    )

    # RTAB-Map visual odometry using synchronized RGBD
    depth_camera_visual_odom = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        namespace=LaunchConfiguration("ns"),
        output="screen",
        parameters=[
            {
                "frame_id": f"{ROBOT_NAME}/base_link",
                "odom_frame_id": f"{ROBOT_NAME}/front_camera",
                "publish_tf": False,  # Let robot_localization handle TF
                "subscribe_depth": False,
                "subscribe_rgbd": True,  # Use synchronized RGBD topic
                "wait_for_transform": 0.2,
                "qos": 1,
                # RTAB-Map internal parameters
                "Odom/Strategy": "0",  # 0=Frame-to-Map, 1=Frame-to-Frame
                "Odom/ResetCountdown": "1",
                "Vis/MaxFeatures": "500",
                "Vis/MinInliers": "10",
            }
        ],
        remappings=[
            ("rgbd_image", f"/{ROBOT_NAME}/rgbd_image"),
            ("odom", f"/{ROBOT_NAME}/odom/depth_camera"),
        ],
    )

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
        ],
    )

    return [
        robot_state_publisher,
        dvl_odom_remapping,
        # Depth camera not on marlin
        # rgbd_sync,
        # depth_camera_visual_odom,
        robot_localization_node,
        sub_control_node,
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument("seed", default_value=""),
        DeclareLaunchArgument("DX", default_value="0.25"),
        DeclareLaunchArgument("DY", default_value="0.25"),
        DeclareLaunchArgument("DZ", default_value="0.10"),
        DeclareLaunchArgument("DYAW", default_value="0.10"),
        DeclareLaunchArgument(
            "enable_deepseecolor",
            default_value="true",
            description="Start the DeepSeeColor RGB-D color correction node.",
        ),
        DeclareLaunchArgument(
            "deepseecolor_device",
            default_value="cuda:0",
            description="Torch device for DeepSeeColor, e.g. cuda:0 or cpu.",
        ),
    ]

    declare_ns = DeclareLaunchArgument("ns", default_value=ROBOT_NAME)

    render = OpaqueFunction(function=_render_scn)

    include_transforms = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("sub_bringup"), "launch", f"{ROBOT_NAME}_launch.py"]
        ),
    )

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
    )

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

    return LaunchDescription(
        args
        + [
            declare_ns,
            render,
            include_transforms,
            # deepseecolor_node,
            foxglove_bridge_node,
            sub_vision_node,
            *sim_entities(),
            *control_and_state_entities(),
        ]
    )
