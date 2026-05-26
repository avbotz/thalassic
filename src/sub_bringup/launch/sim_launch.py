import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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
        robot_name="marlin_v2",
        mesh_prefix=f"file://{Path(sub_sim_share) / 'data'}/",
    )

    robot_description = urdf_robot.read_text()

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
    declare_ns = DeclareLaunchArgument("ns", default_value="marlin_v2")

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
                "robot_name": "marlin_v2",
            }
        ],
    )

    # RTAB-Map RGBD sync to handle timestamp differences between cameras
    rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync",
        namespace="marlin_v2",
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
            ("rgb/image", "/marlin_v2/front_camera/image_color"),
            ("depth/image", "/marlin_v2/depth_camera/image_depth"),
            ("rgb/camera_info", "/marlin_v2/front_camera/camera_info"),
            ("rgbd_image", "/marlin_v2/rgbd_image"),
        ],
    )

    # RTAB-Map visual odometry using synchronized RGBD
    depth_camera_visual_odom = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        namespace="marlin_v2",
        output="screen",
        parameters=[
            {
                "frame_id": "marlin_v2/base_link_ned",
                "odom_frame_id": "marlin_v2/visual_odom",
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
            ("rgbd_image", "/marlin_v2/rgbd_image"),
            ("odom", "/marlin_v2/depth_camera_odom"),
        ],
    )

    deepseecolor_node = Node(
        package="sub_color_correction",
        executable="deepseecolor_node",
        name="deepseecolor",
        namespace="marlin_v2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_deepseecolor")),
        parameters=[
            {
                "rgb_topic": "/marlin_v2/front_camera/image_color",
                "depth_topic": "/marlin_v2/depth_camera/image_depth",
                "corrected_topic": "/marlin_v2/front_camera/image_color_corrected",
                "device": LaunchConfiguration("deepseecolor_device"),
                "init_iters": 10,
                "iters": 2,
                "max_inference_dimension": 640,
                "sync_slop": 0.15,
            }
        ],
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="both",
        namespace=LaunchConfiguration("ns"),
        parameters=[
            os.path.join(get_package_share_directory("sub_bringup"), "config/ekf.yaml"),
        ],
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
    )

    sub_control_node = Node(
        package="sub_control",
        executable="sub_control",
        name="sub_control",
        output="both",
        namespace=LaunchConfiguration("ns"),
        parameters=[
            os.path.join(
                get_package_share_directory("sub_bringup"),
                "config/control_gains_sim.yaml",
            ),
            {
                "world_frame": "map",
                "control_frame": "marlin_v2/base_link",
            },
        ],
    )

    sim_dvl_remapper = Node(
        package="sub_sim_sensors",
        executable="sim_dvl_remapper",
        name="sim_dvl_remapper",
        namespace=LaunchConfiguration("ns"),
        parameters=[
            {
                "robot_name": "marlin_v2",
            }
        ],
    )

    sim_imu_remapper = Node(
        package="sub_sim_sensors",
        executable="sim_imu_remapper",
        name="sim_imu_remapper",
        namespace=LaunchConfiguration("ns"),
        parameters=[
            {
                "robot_name": "marlin_v2",
            }
        ],
    )

    sim_thruster_republisher = Node(
        package="sub_sim_sensors",
        executable="sim_thruster_republisher",
        name="sim_thruster_republisher",
        namespace=LaunchConfiguration("ns"),
    )

    sim_torpedo_launcher = Node(
        package="sub_sim_sensors",
        executable="sim_torpedo_launcher",
        name="sim_torpedo_launcher",
        output="both",
        namespace=LaunchConfiguration("ns"),
    )

    sim_dropper = Node(
        package="sub_sim_sensors",
        executable="sim_dropper",
        name="sim_dropper",
        output="both",
        namespace=LaunchConfiguration("ns"),
    )

    sim_kill_switch = Node(
        package="sub_sim_sensors",
        executable="sim_kill_switch",
        name="sim_kill_switch",
        output="both",
        namespace=LaunchConfiguration("ns"),
        parameters=[
            {
                "off_delay": 5.0,
            }
        ],
    )

    return LaunchDescription(
        args
        + [
            declare_ns,
            render,
            include_stonefish,
            include_transforms,
            robot_state_publisher,
            dvl_odom_remapping,
            rgbd_sync,
            depth_camera_visual_odom,
            deepseecolor_node,
            robot_localization_node,
            foxglove_bridge_node,
            # sub_control_node,
            sim_dvl_remapper,
            sim_thruster_republisher,
            sim_imu_remapper,
            sim_torpedo_launcher,
            sim_dropper,
            sim_kill_switch,
        ]
    )
