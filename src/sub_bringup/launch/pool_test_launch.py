"""Bring the stack up on the vehicle.

Everything specific to the hull - device names, the down camera's serial
number, mounting geometry - comes from
sub_bringup/config/vehicles/<robot_name>.yaml via sub_bringup.vehicle.
"""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare

from sub_bringup.entities import (
    common_launch_include,
    jpeg_quality,
    package_config,
    recorder,
    vision_entities,
)
from sub_bringup.vehicle import Vehicle, load_vehicle


def camera_entities(vehicle: Vehicle) -> list[Node]:
    down, front = vehicle.devices["down_camera"], vehicle.devices["front_camera"]
    front_namespace = f"{vehicle.name}/front_camera"
    return [
        Node(
            package="spinnaker_camera_driver",
            executable="camera_driver_node",
            # Publishes ~/image_raw, so down_camera/image_raw like in the sim.
            name="down_camera",
            namespace=vehicle.name,
            output="both",
            parameters=[
                package_config("sub_bringup", "devices/flir_blackfly_s.yaml"),
                {
                    "serial_number": down.serial_number,
                    "frame_id": down.frame,
                    "parameter_file": PathJoinSubstitution(
                        [FindPackageShare("spinnaker_camera_driver"), "config", "blackfly.yaml"]
                    ),
                },
            ],
        ),
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="logitech_c922_driver",
            # Publishes image_raw in its namespace, so front_camera/image_raw like in the sim.
            namespace=front_namespace,
            output="log",
            parameters=[
                package_config("sub_bringup", "devices/logitech_c922.yaml"),
                {
                    "video_device": front.video_device,
                    "frame_id": front.frame,
                    # Its JPEG copy (image_raw/compressed), which the bag records.
                    **jpeg_quality(front_namespace, [f"/{front_namespace}/image_raw"]),
                },
            ],
        ),
    ]


def driver_entities(vehicle: Vehicle) -> list[LifecycleNode]:
    """The sensor and MCU drivers: lifecycle nodes, configured and activated as they start."""
    dvl, imu = vehicle.devices["dvl"], vehicle.devices["imu"]
    return [
        LifecycleNode(
            package="waterlinked_dvl_driver",
            executable="waterlinked_dvl_driver",
            name="waterlinked_dvl_driver",
            namespace=vehicle.name,
            output="both",
            autostart=True,
            parameters=[
                package_config("sub_bringup", "devices/waterlinked_dvl_a50.yaml"),
                {"ip_address": dvl.ip_address, "frame_id": dvl.frame},
            ],
            remappings=[("~/odom", "odometry/dvl"), ("~/altitude", "altitude")],
        ),
        LifecycleNode(
            package="sub_serial_drivers",
            executable="naviguider_imu_driver",
            name="naviguider_imu_driver",
            namespace=vehicle.name,
            output="both",
            autostart=True,
            parameters=[{"device": imu.serial_port, "frame_id": imu.frame}],
        ),
        LifecycleNode(
            package="sub_serial_drivers",
            executable="sub_low",
            name="sub_low",
            namespace=vehicle.name,
            autostart=True,
            parameters=[
                {
                    "device": vehicle.devices["maritime_mcu"].serial_port,
                    "depth_frame_id": vehicle.frame("odom"),
                    "depth_child_frame_id": vehicle.depth.frame,
                    "depth_z_variance": vehicle.depth.z_variance,
                }
            ],
        ),
    ]


def _vehicle_entities(context, *_, **__):
    """Entities that need the vehicle description, so robot_name must be resolved first."""
    vehicle = load_vehicle(LaunchConfiguration("robot_name").perform(context))
    return [*driver_entities(vehicle), *camera_entities(vehicle)]


def generate_launch_description():
    return LaunchDescription(
        [
            # Transforms, EKF, controller, mission executive, tuning dashboard,
            # Foxglove bridge. A mission runs under the restart supervisor, so a
            # diver can kill the vehicle, carry it back to the start and release
            # it to rerun.
            *common_launch_include(gains="control/gains.yaml", restart="true"),
            OpaqueFunction(function=_vehicle_entities),
            # The Logitech C922 publishes image_raw through usb_cam, and the
            # Blackfly through the Spinnaker driver. TensorRT is pinned, not
            # auto, so a vehicle without it or without an engine fails to load
            # models instead of running them on the CPU.
            *vision_entities("image_raw", {"device": "tensorrt"}),
            recorder(
                exclude=[
                    # Recorded as JPEG (image_raw/compressed): a tenth the size or less.
                    "front_camera/image_raw",
                    # The Blackfly's images are a Bayer mosaic, which the JPEG copy
                    # stores as a grey image. Raw, they are lossless (zstd in the
                    # bag) and Foxglove shows them in colour.
                    "down_camera/image_raw/compressed",
                    # The annotators' drawings, raw and JPEG, of images and
                    # detections that are recorded; recording them would also
                    # keep the annotators drawing.
                    "vision/debug_image.*",
                    # Depth Anything's maps if depth_debug is on, about 6 MB a frame, for viewing.
                    "sub_vision.*/depth(/color)?",
                ]
            ),
        ]
    )
