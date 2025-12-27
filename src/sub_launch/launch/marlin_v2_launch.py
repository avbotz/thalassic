import math

from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    tf_position = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "-0.0",
            "--z",
            "-0.0",
            "--roll",
            "-0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "marlin_v2/odom",
        ],
    )

    tf_base_link_to_base_link_ned = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "-0.0",
            "--z",
            "-0.0",
            "--roll",
            str(math.pi),
            "--pitch",
            "0.0",
            "--yaw",
            str(math.pi / 2),
            "--frame-id",
            "marlin_v2/base_link",
            "--child-frame-id",
            "marlin_v2/base_link_ned",
        ],
    )

    tf_front_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "-0.37506",
            "--z",
            "-0.16631",
            "--roll",
            "3.141519",
            "--pitch",
            "0.0",
            "--yaw",
            "-1.570796",
            "--frame-id",
            "marlin_v2/base_link_ned",
            "--child-frame-id",
            "marlin_v2/front_camera",
        ],
    )

    # tf_down_cam = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["0.16", "0.0725", "0.15", "1.571", "0", "1.571", "base_link", "bluerov2/camera_right"],
    # )

    tf_dvl = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.015797",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "marlin_v2/base_link_ned",
            "--child-frame-id",
            "marlin_v2/dvl_link",
        ],
    )

    tf_dvl = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.015797",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "marlin_v2/base_link_ned",
            "--child-frame-id",
            "marlin_v2/dvl_imu_link",
        ],
    )

    tf_dropper = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "-0.19074",
            "--y",
            "0.40252",
            "--z",
            "0.15565",
            "--roll",
            "-0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "marlin_v2/base_link_ned",
            "--child-frame-id",
            "marlin_v2/dropper_link",
        ],
    )

    tf_grabber_left = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.056319",
            "--y",
            "0.3506",
            "--z",
            "0.13631",
            "--roll",
            "-0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "marlin_v2/base_link_ned",
            "--child-frame-id",
            "marlin_v2/left_grabber_link",
        ],
    )

    tf_grabber_right = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.091319",
            "--y",
            "0.35087",
            "--z",
            "0.13636",
            "--roll",
            "-0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "marlin_v2/base_link_ned",
            "--child-frame-id",
            "marlin_v2/right_grabber_link",
        ],
    )

    return LaunchDescription(
        [
            tf_position,
            tf_base_link_to_base_link_ned,
            tf_front_cam,
            tf_dvl,
            tf_dropper,
            tf_grabber_left,
            tf_grabber_right,
            # tf_down_cam,
        ]
    )
