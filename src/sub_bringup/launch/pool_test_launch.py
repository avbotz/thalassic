"""Bring the stack up on the vehicle.

Everything specific to the hull - device names, the down camera's serial
number, mounting geometry - comes from sub_bringup/config/<robot_name>.yaml
via sub_bringup.vehicle.
"""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare
from sub_bringup.entities import common_launch_include, package_config, vision_node
from sub_bringup.launch_utils import lifecycle_startup
from sub_bringup.vehicle import Vehicle, load_vehicle


def camera_entities(vehicle: Vehicle) -> list[Node]:
    blackfly_camera_driver = Node(
        package="spinnaker_camera_driver",
        executable="camera_driver_node",
        name="blackfly",
        namespace=vehicle.name,
        output="both",
        parameters=[
            package_config("sub_bringup", "blackfly_down.yaml"),
            {
                "serial_number": vehicle.camera("down")["serial_number"],
                "frame_id": vehicle.frame("down_camera"),
                "parameter_file": PathJoinSubstitution(
                    [FindPackageShare("spinnaker_camera_driver"), "config", "blackfly.yaml"]
                ),
            },
        ],
    )

    logitech_c922_driver = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="logitech_c922_driver",
        namespace=f"{vehicle.name}/front_camera",
        output="log",
        parameters=[
            package_config("sub_bringup", "logitech_c922.yaml"),
            {
                "video_device": vehicle.camera("front")["video_device"],
                "frame_id": vehicle.frame("front_camera"),
            },
        ],
    )

    return [blackfly_camera_driver, logitech_c922_driver]


def driver_entities(vehicle: Vehicle) -> list:
    waterlinked_dvl_driver_node = LifecycleNode(
        package="waterlinked_dvl_driver",
        executable="waterlinked_dvl_driver",
        name="waterlinked_dvl_driver",
        namespace=vehicle.name,
        output="both",
        parameters=[
            package_config("sub_bringup", "dvl.yaml"),
            {
                "ip_address": vehicle.dvl_address(),
                "frame_id": vehicle.frame("dvl_link"),
            },
        ],
        remappings=[("~/odom", "odometry/dvl")],
    )

    sub_low_node = LifecycleNode(
        package="sub_serial_drivers",
        executable="sub_low",
        name="sub_low",
        namespace=vehicle.name,
        parameters=[
            {
                "device": vehicle.device("maritime_mcu"),
                "depth_frame_id": vehicle.frame("odom"),
                "depth_child_frame_id": vehicle.frame("depth_link"),
            }
        ],
    )

    naviguider_imu_driver_node = LifecycleNode(
        package="sub_serial_drivers",
        executable="naviguider_imu_driver",
        name="naviguider_imu_driver",
        namespace=vehicle.name,
        output="both",
        parameters=[
            {
                "device": vehicle.device("imu"),
                "frame_id": vehicle.frame("imu_link"),
            }
        ],
    )

    return [
        *lifecycle_startup(waterlinked_dvl_driver_node),
        *lifecycle_startup(naviguider_imu_driver_node),
        *lifecycle_startup(sub_low_node),
    ]


def _vehicle_entities(context, *_, **__):
    """Entities that need the vehicle description, so robot_name must be resolved first."""
    vehicle = load_vehicle(LaunchConfiguration("robot_name").perform(context))
    return [*driver_entities(vehicle), *camera_entities(vehicle)]


def generate_launch_description():
    return LaunchDescription(
        [
            # Transforms, EKF, controller, mission executive, Foxglove bridge.
            *common_launch_include("control_gains.yaml"),
            OpaqueFunction(function=_vehicle_entities),
            # The Logitech C922 publishes image_raw through usb_cam.
            vision_node("sub_vision", "sub_vision.yaml", "front_camera", "image_raw"),
        ]
    )
