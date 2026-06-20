from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.actions.node import ExecuteProcess
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


ROBOT_NAME = "marlin_v2"


def lifecycle_startup(node):
    """Configures and activates a lifecycle node."""
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )
    return [node, configure, activate]


def camera_entities():
    blackfly_camera_driver = Node(
        package="spinnaker_camera_driver",
        executable="camera_driver_node",
        namespace=LaunchConfiguration("ns"),
        output="both",
        name="blackfly",
        parameters=[
            {
                "buffer_queue_size": 1,
                "serial_number": "16359776",
                "gain_auto": "Continuous",
                "pixel_format": "BayerRG8",
                "exposure_auto": "Off",
                "exposure_time": 8e3,
                "frame_rate_enable": True,
                "frame_rate": 10.0,
                "trigger_mode": "Off",
                "stream_buffer_handling_mode": "NewestOnly",
                "gev_scps_packet_size": 1500,

                "frame_id": f"{ROBOT_NAME}/front_camera",
                "parameter_file": PathJoinSubstitution(
                    [
                        FindPackageShare("spinnaker_camera_driver"),
                        "config",
                        "blackfly.yaml",
                    ]
                ),
            },
        ],
    )

    # oak_driver_node = Node(
    #     package="depthai_ros_driver_v3",
    #     executable="driver_node",
    #     name="oak",
    #     namespace=LaunchConfiguration("ns"),
    #     output="screen",
    #     parameters=[
    #         PathJoinSubstitution(
    #             [
    #                 FindPackageShare("sub_bringup"),
    #                 "config/oak_d_pro.yaml",
    #             ]
    #         ),
    #     ],
    # )

    logitech_c922_driver = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        output="log",
        name="logitech_c922_driver",
        namespace=f"{ROBOT_NAME}/front_camera",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("sub_bringup"),
                    "config/params.yaml",
                ]
            ),
        ],
    )

    return [
        blackfly_camera_driver,
        # Depth camera not on marlin
        # oak_driver_node,
        logitech_c922_driver,
    ]


def vision_entities():
    sub_vision_node = Node(
        package="sub_vision",
        executable="sub_vision",
        name="sub_vision",
        namespace=LaunchConfiguration("ns"),
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("sub_vision"),
                    "config/sub_vision.yaml",
                ]
            ),
        ],
    )

    return [sub_vision_node]


def control_and_state_entities():
    waterlinked_dvl_driver_node = LifecycleNode(
        package="waterlinked_dvl_driver",
        executable="waterlinked_dvl_driver",
        name="waterlinked_dvl_driver",
        namespace=LaunchConfiguration("ns"),
        output="both",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("sub_bringup"),
                    "config/dvl.yaml",
                ]
            ),
        ],
        remappings=[
            ("~/odom", "odometry/dvl"),
        ],
    )

    sub_low_node = LifecycleNode(
        package="sub_serial_drivers",
        executable="sub_low",
        name="sub_low",
        namespace=LaunchConfiguration("ns"),
        parameters=[{"device": "/dev/arduino_mega"}],
    )

    naviguider_imu_driver_node = LifecycleNode(
        package="sub_serial_drivers",
        executable="naviguider_imu_driver",
        name="naviguider_imu_driver",
        namespace=LaunchConfiguration("ns"),
        output="both",
        parameters=[{"device": "/dev/naviguider_imu"}],
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
                    "config/control_gains.yaml",
                ]
            ),
        ],
    )

    return [
        robot_localization_node,
        sub_control_node,
        *lifecycle_startup(waterlinked_dvl_driver_node),
        *lifecycle_startup(naviguider_imu_driver_node),
        *lifecycle_startup(sub_low_node),
    ]


def foxglove_entities() -> list:
    clear_port = ExecuteProcess(
        cmd=["fuser", "-k", "8765/tcp"],  # free port 8765
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
            *control_and_state_entities(),
            *camera_entities(),
            *vision_entities(),
            *foxglove_entities(),
        ]
    )
