from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
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
            "--yaw",
            "-1.570796",
            "--pitch",
            "0.0",
            "--roll",
            "3.141519",
            "--frame-id",
            "marlin_v2/base_link",
            "--child-frame-id",
            "marlin_v2/front_camera",
        ],
    )

    # tf_down_cam = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["0.16", "0.0725", "0.15", "1.571", "0", "1.571", "base_link", "bluerov2/camera_right"],
    # )

    return LaunchDescription(
        [
            tf_front_cam,
            # tf_down_cam,
        ]
    )
