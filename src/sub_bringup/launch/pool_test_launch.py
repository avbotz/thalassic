import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def dvl_driver_entities():
    dvl_driver_node = LifecycleNode(
        package="waterlinked_dvl_driver",
        executable="waterlinked_dvl_driver",
        name="waterlinked_dvl_driver",
        namespace=LaunchConfiguration("ns"),
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("sub_bringup"), "config/dvl.yaml"),
        ],
    )

    dvl_driver_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(dvl_driver_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    dvl_driver_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=dvl_driver_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(dvl_driver_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return [
        dvl_driver_node,
        dvl_driver_configure_event,
        dvl_driver_activate_event,
    ]


def imu_driver_entities():
    naviguider_imu_driver_node = Node(
        package="sub_serial_drivers",
        executable="naviguider_imu_driver",
        name="naviguider_imu_driver",
        namespace=LaunchConfiguration("ns"),
        output="screen",
        parameters=[{"device": "/dev/naviguider_imu"}],
    )

    naviguider_imu_driver_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(naviguider_imu_driver_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    naviguider_imu_driver_activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(naviguider_imu_driver_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    return [
        naviguider_imu_driver_node,
        naviguider_imu_driver_configure_event,
        naviguider_imu_driver_activate_event,
    ]


def sub_low_entities():
    sub_low_node = Node(
        package="sub_serial_drivers",
        executable="sub_low",
        name="sub_low",
        namespace=LaunchConfiguration("ns"),
        parameters=[{"device": "/dev/arduino_mega"}],
    )

    sub_low_node_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(sub_low_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    sub_low_node_activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(sub_low_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    return [
        sub_low_node,
        sub_low_node_configure_event,
        sub_low_node_activate_event,
    ]


def spinnaker_camera_entities():
    # Using blackfly camera
    # Parameters from gige_node.launch.py sample file
    parameters = {
        "debug": False,
        "dump_node_map": False,
        "gain_auto": "Continuous",
        "pixel_format": "BayerRG8",
        "exposure_auto": "Continuous",
        "frame_rate_auto": "Off",
        "frame_rate": 40.0,
        "frame_rate_enable": True,
        "buffer_queue_size": 10,
        "trigger_mode": "Off",
        # 'stream_buffer_handling_mode': 'NewestFirst',
        # 'multicast_monitor_mode': False
    }
    parameter_file = PathJoinSubstitution(
        [
            FindPackageShare("spinnaker_camera_driver"),
            "config",
            "blackfly.yaml",
        ]
    )

    node = Node(
        package="spinnaker_camera_driver",
        executable="camera_driver_node",
        namespace=LaunchConfiguration("ns"),
        output="screen",
        name=["blackfly"],
        parameters=[
            parameters,
            {
                "ffmpeg_image_transport.encoding": "hevc_nvenc",
                "parameter_file": parameter_file,
                "serial_number": ["'16359776'"],
            },
        ],
        remappings=[
            ("~/control", "exposure_control/control"),
        ],
    )

    return [node]


def oak_camera_entities():
    oak_driver_node = Node(
        package="depthai_ros_driver_v3",
        executable="driver_node",
        name="oak",
        namespace=LaunchConfiguration("ns"),
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("sub_bringup"),
                    "config/oak_d_pro.yaml",
                ]
            ),
        ],
    )

    return [oak_driver_node]


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
        output="screen",
        namespace="marlin_v2",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("sub_bringup"),
                    "config/control_gains.yaml",
                ]
            ),
            {
                "world_frame": "map",
                "control_frame": "marlin_v2/base_link",
            },
        ],
    )

    return [dvl_odom_remapping, robot_localization_node, sub_control_node]


def generate_launch_description():
    declare_ns = DeclareLaunchArgument("ns", default_value="marlin_v2")

    include_transforms = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("sub_bringup"), "launch", "marlin_v2_launch.py"]
        ),
    )

    return LaunchDescription(
        [
            declare_ns,
            include_transforms,
            *dvl_driver_entities(),
            *imu_driver_entities(),
            *sub_low_entities(),
            *control_and_state_entities(),
            *oak_camera_entities(),
            *vision_entities(),
        ]
    )
