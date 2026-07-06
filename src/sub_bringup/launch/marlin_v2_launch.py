import math

from launch import LaunchDescription

from sub_bringup.launch_utils import static_tf

ROBOT = "marlin_v2"
BASE_NED = f"{ROBOT}/base_link_ned"

def generate_launch_description():
    transforms = [
        static_tf("map", f"{ROBOT}/odom"),
        static_tf(f"{ROBOT}/base_link", BASE_NED, roll=math.pi, yaw=-math.pi / 2),
        static_tf(
            BASE_NED, f"{ROBOT}/front_camera", y=-0.33, z=-0.16631, roll=math.pi / 2
        ),
        static_tf(BASE_NED, f"{ROBOT}/dvl_link", z=0.015797, yaw=-math.pi / 2),
        static_tf(BASE_NED, f"{ROBOT}/pressure_link", y=0.34012, z=-0.19384),
        # Naviguider IMU outputs ENU
        static_tf(f"{ROBOT}/base_link", f"{ROBOT}/imu_link", x=-0.199, y=-0.09, z=0.185622),
        static_tf(BASE_NED, f"{ROBOT}/dropper_link", x=-0.19074, y=0.40252, z=0.15565),
        static_tf(
            BASE_NED, f"{ROBOT}/left_grabber_link", x=0.056319, y=0.3506, z=0.13631
        ),
        static_tf(
            BASE_NED, f"{ROBOT}/right_grabber_link", x=0.091319, y=0.35087, z=0.13636
        ),
    ]

    thrusters = [
        (-0.23, -0.22, 0.00, 0.0, math.pi / 2, 0.0),
        (0.23, -0.22, 0.00, 0.0, math.pi / 2, 0.0),
        (-0.23, 0.22, 0.00, 0.0, math.pi / 2, 0.0),
        (0.23, 0.22, 0.00, 0.0, math.pi / 2, 0.0),
        (-0.285, -0.315, -0.08, 0.0, 0.0, math.pi * 3 / 4),
        (0.285, -0.315, -0.08, 0.0, 0.0, math.pi / 4),
        (-0.285, 0.315, -0.08, 0.0, 0.0, math.pi / 4),
        (0.285, 0.315, -0.08, 0.0, 0.0, math.pi * 3 / 4),
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
