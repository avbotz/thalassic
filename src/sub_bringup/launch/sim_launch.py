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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    NotEqualsSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.actions.node import ExecuteProcess
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
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
            namespace=LaunchConfiguration("robot_name"),
            extra_arguments=[{"use_intra_process_comms": True}],
            **kwargs,
        )

    sim_sensors_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        name="sim_sensors_container",
        namespace=LaunchConfiguration("robot_name"),
        output="both",
        composable_node_descriptions=[
            sim_component(
                "sim_dvl_remapper",
                "SimDVLRemapper",
                parameters=[{"dvl_link": [LaunchConfiguration("robot_name"), "/dvl_link"]}],
            ),
            sim_component(
                "sim_imu_remapper",
                "SimIMURemapper",
                parameters=[{"imu_link": [LaunchConfiguration("robot_name"), "/imu_link"]}],
            ),
            sim_component(
                "sim_pressure_to_depth",
                "SimPressureToDepth",
                parameters=[
                    {
                        "frame_id": [LaunchConfiguration("robot_name"), "/odom"],
                        "child_frame_id": [LaunchConfiguration("robot_name"), "/base_link"],
                    }
                ],
            ),
            sim_component("sim_thruster_republisher", "SimThrusterRepublisher"),
            sim_component("sim_torpedo_launcher", "SimTorpedoLauncher"),
            sim_component(
                "sim_dropper",
                "SimDropper",
                parameters=[
                    {"joint_name": "marlin_v2/dropper_joint"}
                ],
            ),
            sim_component(
                "sim_kill_switch",
                "SimKillSwitch",
                parameters=[{"off_delay": 6.0}],
            ),
        ],
        ros_arguments=["--disable-stdout-logs"],
    )

    sim_labeling_node = Node(
        package="sim_labeling",
        executable="labeling",
        name="sim_labeling",
        namespace=LaunchConfiguration("robot_name"),
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
        namespace=LaunchConfiguration("robot_name"),
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
    #     namespace=LaunchConfiguration("robot_name"),
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
    #         ("rgb/image", ["/", LaunchConfiguration("robot_name"), "/oak/rgb/image_raw"]),
    #         ("depth/image", ["/", LaunchConfiguration("robot_name"), "/oak/stereo/image_raw"]),
    #         ("rgb/camera_info", ["/", LaunchConfiguration("robot_name"), "/oak/rgb/camera_info"]),
    #         ("rgbd_image", ["/", LaunchConfiguration("robot_name"), "/rgbd_image"]),
    #     ],
    # )

    # # RTAB-Map visual odometry using synchronized RGBD
    # depth_camera_visual_odom = Node(
    #     package="rtabmap_odom",
    #     executable="rgbd_odometry",
    #     name="rgbd_odometry",
    #     namespace=LaunchConfiguration("robot_name"),
    #     output="screen",
    #     parameters=[
    #         {
    #             "frame_id": [LaunchConfiguration("robot_name"), "/base_link"],
    #             "odom_frame_id": [LaunchConfiguration("robot_name"), "/front_camera"],
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
    #         ("rgbd_image", ["/", LaunchConfiguration("robot_name"), "/rgbd_image"]),
    #         ("odom", ["/", LaunchConfiguration("robot_name"), "/odom/depth_camera"]),
    #     ],
    # )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="both",
        namespace=LaunchConfiguration("robot_name"),
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
        namespace=LaunchConfiguration("robot_name"),
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("sub_bringup"),
                    "config/control_gains_sim.yaml",
                ]
            ),
            {"robot_name": LaunchConfiguration("robot_name")},
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
    #     namespace=LaunchConfiguration("robot_name"),
    #     output="screen",
    #     condition=IfCondition(LaunchConfiguration("enable_deepseecolor")),
    #     parameters=[
    #         {
    #             "rgb_topic": ["/", LaunchConfiguration("robot_name"), "/oak/rgb/image_raw"],
    #             "depth_topic": ["/", LaunchConfiguration("robot_name"), "/oak/stereo/image_raw"],
    #             "corrected_topic": ["/", LaunchConfiguration("robot_name"), "/oak/rgb/image_color_corrected"],
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
        namespace=LaunchConfiguration("robot_name"),
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("sub_vision"),
                    "config/sub_vision.yaml",
                ]
            ),
            # Stonefish publishes the front camera as rgb8 on image_color;
            # cv_bridge converts to bgr8 in the node, so no bridge is needed.
            {
                "rgb_topic": "front_camera/image_color",
                "camera_info_topic": "front_camera/camera_info",
                "image_transport": "raw",
            },
        ],
    )

    sub_vision_down_node = Node(
        package="sub_vision",
        executable="sub_vision",
        name="sub_vision_down",
        output="screen",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("sub_vision"),
                    "config/sub_vision_down.yaml",
                ]
            ),
            # Stonefish publishes the front camera as rgb8 on image_color;
            # cv_bridge converts to bgr8 in the node, so no bridge is needed.
            {
                "rgb_topic": "down_camera/image_color",
                "camera_info_topic": "down_camera/camera_info",
                "image_transport": "raw",
            },
        ],
    )

    sub_annotation_node = Node(
        package="sub_vision",
        executable="annotation_visualizer",
        name="annotation_visualizer",
        output="screen",
        namespace=LaunchConfiguration("robot_name"),
    )

    return [
        sub_vision_node,
        sub_vision_down_node,
        sub_annotation_node,
        # deepseecolor_node,
    ]


def generate_launch_description():
    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="marlin_v2")

    # Empty (the default) brings the stack up without a mission; pass
    # mission:=<name or path> to also run the mission executive, e.g.
    #   ros2 launch sub_bringup sim_launch.py mission:=pool_a
    declare_mission = DeclareLaunchArgument(
        "mission",
        default_value="",
        description="Mission tree to execute (resources/missions/<name>.xml or a path); empty skips sub_mission",
    )
    declare_role = DeclareLaunchArgument(
        "role",
        default_value="SURVEY",
        description="Mission vision role: SURVEY or SEARCH",
    )

    include_transforms = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("sub_bringup"),
                "launch",
                [LaunchConfiguration("robot_name"), "_launch.py"],
            ]
        ),
    )

    sub_mission_node = Node(
        package="sub_mission",
        executable="mission",
        name="sub_mission",
        output="screen",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[
            {
                "mission": LaunchConfiguration("mission"),
                "role": LaunchConfiguration("role"),
            }
        ],
        condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration("mission"), "")),
    )

    return LaunchDescription(
        [
            declare_robot_name,
            declare_mission,
            declare_role,
            include_transforms,
            sub_mission_node,
            *sim_entities(),
            *control_and_state_entities(),
            *vision_entities(),
            *foxglove_entities(),
        ]
    )
