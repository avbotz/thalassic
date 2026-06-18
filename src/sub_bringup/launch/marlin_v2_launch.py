import math

from launch import LaunchDescription
from launch_ros.actions import Node

ROBOT = "marlin_v2"
BASE_NED = f"{ROBOT}/base_link_ned"


def static_tf(parent, child, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            str(x),
            "--y",
            str(y),
            "--z",
            str(z),
            "--roll",
            str(roll),
            "--pitch",
            str(pitch),
            "--yaw",
            str(yaw),
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ],
        ros_arguments=["--disable-stdout-logs"],
    )


def generate_launch_description():
    transforms = [
        static_tf("map", f"{ROBOT}/odom"),
        # base_link is REP-103 FLU (X-fwd, Y-left, Z-up). The Stonefish body
        # frame ("base_link_ned") is X-right, Y-back, Z-down, so the FLU->that
        # rotation is roll=pi (flip Z), yaw=-pi/2. All children below are
        # expressed in this frame, matching the sim's layout.scn.j2 exactly.
        static_tf(f"{ROBOT}/base_link", BASE_NED, roll=math.pi, yaw=-math.pi / 2),
        static_tf(
            BASE_NED, f"{ROBOT}/front_camera", y=-0.33, z=-0.16631, roll=math.pi / 2
        ),
        static_tf(BASE_NED, f"{ROBOT}/dvl_link", z=0.015797, yaw=-math.pi / 2),
        # Naviguider IMU outputs ENU
        static_tf(f"{ROBOT}/base_link", f"{ROBOT}/imu_link", y=-0.1, z=0.2),
        static_tf(BASE_NED, f"{ROBOT}/dropper_link", x=-0.19074, y=0.40252, z=0.15565),
        static_tf(
            BASE_NED, f"{ROBOT}/left_grabber_link", x=0.056319, y=0.3506, z=0.13631
        ),
        static_tf(
            BASE_NED, f"{ROBOT}/right_grabber_link", x=0.091319, y=0.35087, z=0.13636
        ),
    ]

    thrusters = [
        (-0.29, 0.34, -0.08, 0.0, 0.0, 7 * math.pi / 4),
        (-0.29, -0.34, -0.08, 0.0, 0.0, math.pi / 4),
        (0.29, 0.34, -0.08, 0.0, 0.0, 5 * math.pi / 4),
        (0.29, -0.34, -0.08, 0.0, 0.0, 3 * math.pi / 4),
        (0.23, -0.22, 0.00, 0.0, math.pi / 2, 0.0),
        (-0.23, -0.22, 0.00, 0.0, math.pi / 2, 0.0),
        (0.23, 0.22, 0.00, 0.0, math.pi / 2, 0.0),
        (-0.23, 0.22, 0.00, 0.0, math.pi / 2, 0.0),
    ]
    transforms += [
        static_tf(
            BASE_NED,
            f"{ROBOT}/thruster_{i}_link",
            x=x,
            y=y,
            z=z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )
        for i, (x, y, z, roll, pitch, yaw) in enumerate(thrusters)
    ]

    return LaunchDescription(transforms)
