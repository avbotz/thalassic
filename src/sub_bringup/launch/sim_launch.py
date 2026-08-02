import random
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    AndSubstitution,
    EqualsSubstitution,
    LaunchConfiguration,
    NotEqualsSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.actions.node import ExecuteProcess
from launch_ros.actions.set_remap import SetRemap
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from sub_sim.generate_robot import render_robot_scenario
from sub_sim.randomize_locs import randomize_scenario_locations
from sub_sim.robot_scenario_to_urdf import robot_scenario_to_urdf


def _stonefish_processes(proc_root=Path("/proc")) -> list[int]:
    """Return running Stonefish PIDs without depending on ROS discovery."""
    processes = []
    for entry in proc_root.glob("[0-9]*"):
        try:
            command = entry.joinpath("cmdline").read_bytes().split(b"\0", 1)[0]
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if command and Path(command.decode(errors="replace")).name == "stonefish_simulator":
            processes.append(int(entry.name))
    return processes


def _reject_second_sim(_context, *_, **__):
    processes = _stonefish_processes()
    if processes:
        pids = ", ".join(str(pid) for pid in processes)
        raise RuntimeError(
            "Stonefish is already running "
            f"(PID {pids}). Stop the existing sim before launching another one."
        )
    return []


def _render_scn(context, *_, **__):
    DX = float(LaunchConfiguration("DX").perform(context))
    DY = float(LaunchConfiguration("DY").perform(context))
    DZ = float(LaunchConfiguration("DZ").perform(context))
    DYAW = float(LaunchConfiguration("DYAW").perform(context))
    ROBOT_X = float(LaunchConfiguration("robot_x").perform(context))
    ROBOT_Y = float(LaunchConfiguration("robot_y").perform(context))
    ROBOT_Z = float(LaunchConfiguration("robot_z").perform(context))
    robot_yaw_value = LaunchConfiguration("robot_yaw").perform(context)
    ROBOT_YAW = float(robot_yaw_value) if robot_yaw_value else None

    try:
        SEED = int(LaunchConfiguration("seed").perform(context))
    except ValueError:
        SEED = None

    robot_name = LaunchConfiguration("robot_name").perform(context)
    requested_torp_board_type = LaunchConfiguration("torp_board_type").perform(context)
    if requested_torp_board_type == "random":
        torp_board_type = random.Random(SEED).choice(("v1", "v2"))
    elif requested_torp_board_type in {"v1", "v2"}:
        torp_board_type = requested_torp_board_type
    else:
        raise RuntimeError("torp_board_type must be random, v1, or v2")

    sub_sim_share = Path(get_package_share_directory("sub_sim"))
    scenario_name = LaunchConfiguration("scenario").perform(context)
    scenario_templates = {
        "woollett": "woollett.scn.j2",
        "slalom_regression": "slalom_regression.scn.j2",
    }
    if scenario_name not in scenario_templates:
        choices = ", ".join(sorted(scenario_templates))
        raise RuntimeError(f"unknown simulator scenario '{scenario_name}' (choose: {choices})")

    scenario_file = sub_sim_share / "scenarios" / scenario_templates[scenario_name]
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
        ROBOT_X=ROBOT_X,
        ROBOT_Y=ROBOT_Y,
        ROBOT_Z=ROBOT_Z,
        ROBOT_YAW=ROBOT_YAW,
        TORP_BOARD_TYPE=torp_board_type,
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
        SetLaunchConfiguration("resolved_torp_board_type", torp_board_type),
    ]


def sim_entities() -> list:
    args = [
        DeclareLaunchArgument(
            "scenario",
            default_value="woollett",
            description="Simulator scene: woollett",
        ),
        DeclareLaunchArgument("seed", default_value=""),
        DeclareLaunchArgument("DX", default_value="0.25"),
        DeclareLaunchArgument("DY", default_value="0.25"),
        DeclareLaunchArgument("DZ", default_value="0.10"),
        DeclareLaunchArgument("DYAW", default_value="0.10"),
        DeclareLaunchArgument("robot_x", default_value="17.25"),
        DeclareLaunchArgument("robot_y", default_value="-10.25"),
        DeclareLaunchArgument("robot_z", default_value="0.25"),
        DeclareLaunchArgument("robot_yaw", default_value=""),
        DeclareLaunchArgument(
            "sim_rate",
            default_value="500.0",
            description="Stonefish physics/update rate in Hz",
        ),
        DeclareLaunchArgument("sim_window_res_x", default_value="1900"),
        DeclareLaunchArgument("sim_window_res_y", default_value="1000"),
        DeclareLaunchArgument("sim_rendering_quality", default_value="medium"),
    ]

    render = OpaqueFunction(function=_render_scn)

    include_stonefish = GroupAction(
        actions=[
            SetRemap(src="/marlin_v2/front_camera/image_color", dst="/marlin_v2/front_camera/image_raw"),
            SetRemap(src="/marlin_v2/front_camera/image_color/compressed", dst="/marlin_v2/front_camera/image_raw/compressed"),
            SetRemap(src="/marlin_v2/front_camera/image_color/compressedDepth", dst="/marlin_v2/front_camera/image_raw/compressedDepth"),
            SetRemap(src="/marlin_v2/front_camera/image_color/theora", dst="/marlin_v2/front_camera/image_raw/theora"),
            SetRemap(src="/marlin_v2/front_camera/image_color/zstd", dst="/marlin_v2/front_camera/image_raw/zstd"),

            SetRemap(src="/marlin_v2/down_camera/image_color", dst="/marlin_v2/down_camera/image_raw"),
            SetRemap(src="/marlin_v2/down_camera/image_color/compressed", dst="/marlin_v2/down_camera/image_raw/compressed"),
            SetRemap(src="/marlin_v2/down_camera/image_color/compressedDepth", dst="/marlin_v2/down_camera/image_raw/compressedDepth"),
            SetRemap(src="/marlin_v2/down_camera/image_color/theora", dst="/marlin_v2/down_camera/image_raw/theora"),
            SetRemap(src="/marlin_v2/down_camera/image_color/zstd", dst="/marlin_v2/down_camera/image_raw/zstd"),

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
                    "simulation_rate": LaunchConfiguration("sim_rate"),
                    "window_res_x": LaunchConfiguration("sim_window_res_x"),
                    "window_res_y": LaunchConfiguration("sim_window_res_y"),
                    "rendering_quality": LaunchConfiguration("sim_rendering_quality"),
                }.items(),
            ),
        ]
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
            # Bring the simulated kill switch up first.  It publishes a
            # transient initial killed state, so loading it after the mission
            # has begun can abort an otherwise healthy task mid-run.
            sim_component(
                "sim_kill_switch",
                "SimKillSwitch",
                parameters=[{"off_delay": 0.0}],
            ),
            # The mission's first marker drop can occur before the remaining
            # sensor adapters finish loading, so make its service available
            # alongside the kill switch.
            sim_component(
                "sim_dropper",
                "SimDropper",
                parameters=[
                    {"joint_name": "marlin_v2/dropper_joint"}
                ],
            ),
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
                        # Stonefish publishes hydrostatic gauge pressure, not
                        # absolute pressure including one atmosphere.
                        "surface_pressure_pa": 0.0,
                    }
                ],
            ),
            sim_component("sim_thruster_republisher", "SimThrusterRepublisher"),
            sim_component("sim_torpedo_launcher", "SimTorpedoLauncher"),
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
                    "config/ekf_sim.yaml",
                ]
            ),
        ],
    )

    legacy_control_node = Node(
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
            {"robot_name": LaunchConfiguration("robot_name"), "start_un_killed": True},
        ],
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("controller"), "legacy")
        ),
    )

    feedforward_control_node = Node(
        package="sub_control",
        executable="sub_control_ff",
        name="sub_control",
        output="both",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("sub_bringup"),
                    "config/control_ff_sim.yaml",
                ]
            ),
            {"robot_name": LaunchConfiguration("robot_name")},
        ],
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("controller"), "feedforward")
        ),
    )

    return [
        robot_state_publisher,
        # Depth camera not on marlin
        # rgbd_sync,
        # depth_camera_visual_odom,
        robot_localization_node,
        legacy_control_node,
        feedforward_control_node,
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
            # Sim publishes the front camera as rgb8 on image_raw;
            # cv_bridge converts to bgr8 in the node, so no bridge is needed.
            {
                "rgb_topic": "front_camera/image_raw",
                "camera_info_topic": "front_camera/camera_info",
                "image_transport": "raw",
                "model_dir": PathJoinSubstitution(
                    [FindPackageShare("sub_vision"), "models"]
                ),
                # Depth Anything debug is simulation-only.
                "depth_enabled": True,
                "depth_model_path": PathJoinSubstitution(
                    [FindPackageShare("sub_vision"), "weights/depth_anything_v2_vits.onnx"]
                ),
                "depth_debug_enabled": True,
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
            # Sim publishes the front camera as rgb8 on image_raw;
            # cv_bridge converts to bgr8 in the node, so no bridge is needed.
            {
                "rgb_topic": "down_camera/image_raw",
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

    sub_annotation_down_node = Node(
        package="sub_vision",
        executable="annotation_visualizer",
        name="annotation_visualizer_down",
        output="screen",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[{"image_transport": "compressed"}],
        remappings=[
            ("front_camera/image_raw", "down_camera/image_raw"),
            ("front_camera/image_raw/compressed", "down_camera/image_raw/compressed"),
            ("vision/detections", "vision/detections_down"),
            ("vision/debug_image", "vision/debug_image_down"),
        ],
    )

    return [
        sub_vision_node,
        sub_vision_down_node,
        sub_annotation_node,
        sub_annotation_down_node,
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
    declare_torp_board_type = DeclareLaunchArgument(
        "torp_board_type",
        default_value="random",
        description="Torpedo-board artwork: v1, v2, or random (the selected layout is passed to the mission)",
    )
    declare_dashboard = DeclareLaunchArgument(
        "dashboard",
        default_value="false",
        description="Start the browser ROS debugging dashboard on port 8080",
    )
    declare_dashboard_host = DeclareLaunchArgument(
        "dashboard_host", default_value="127.0.0.1"
    )
    declare_dashboard_port = DeclareLaunchArgument(
        "dashboard_port", default_value="8080"
    )
    declare_controller = DeclareLaunchArgument(
        "controller",
        default_value="feedforward",
        choices=["feedforward", "legacy"],
        description="High-level controller implementation to run",
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
                "torp_board_type": LaunchConfiguration("resolved_torp_board_type"),
                "ignore_initial_kill": True,
            }
        ],
        condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration("mission"), "")),
    )

    legacy_dashboard_node = Node(
        package="sub_pid_tuner",
        executable="dashboard",
        name="dashboard_server",
        output="screen",
        arguments=[
            "--robot-name", LaunchConfiguration("robot_name"),
            "--controller-node", "sub_control",
            "--profile", PathJoinSubstitution(
                [FindPackageShare("sub_bringup"), "config", "control_gains_sim.yaml"]
            ),
            "--host", LaunchConfiguration("dashboard_host"),
            "--port", LaunchConfiguration("dashboard_port"),
        ],
        condition=IfCondition(
            AndSubstitution(
                LaunchConfiguration("dashboard"),
                EqualsSubstitution(LaunchConfiguration("controller"), "legacy"),
            )
        ),
    )

    feedforward_dashboard_node = Node(
        package="sub_pid_tuner",
        executable="dashboard",
        name="dashboard_server",
        output="screen",
        arguments=[
            "--robot-name", LaunchConfiguration("robot_name"),
            "--controller-node", "sub_control",
            "--profile", PathJoinSubstitution(
                [FindPackageShare("sub_bringup"), "config", "control_ff_sim.yaml"]
            ),
            "--host", LaunchConfiguration("dashboard_host"),
            "--port", LaunchConfiguration("dashboard_port"),
        ],
        condition=IfCondition(
            AndSubstitution(
                LaunchConfiguration("dashboard"),
                EqualsSubstitution(
                    LaunchConfiguration("controller"), "feedforward"
                ),
            )
        ),
    )

    return LaunchDescription(
        [
            declare_robot_name,
            declare_mission,
            declare_role,
            declare_torp_board_type,
            declare_dashboard,
            declare_dashboard_host,
            declare_dashboard_port,
            declare_controller,
            OpaqueFunction(function=_reject_second_sim),
            include_transforms,
            *sim_entities(),
            sub_mission_node,
            legacy_dashboard_node,
            feedforward_dashboard_node,
            *control_and_state_entities(),
            *vision_entities(),
            *foxglove_entities(),
        ]
    )
