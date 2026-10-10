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
    annotation_visualizer,
    common_launch_include,
    jpeg_quality,
    lifecycle_startup,
    package_config,
    recorder,
    vision_node,
)
from sub_bringup.vehicle import Vehicle, load_vehicle


def camera_entities(vehicle: Vehicle) -> list[Node]:
    blackfly_camera_driver = Node(
        package="spinnaker_camera_driver",
        executable="camera_driver_node",
        # Publishes ~/image_raw, so down_camera/image_raw like in the sim.
        name="down_camera",
        namespace=vehicle.name,
        output="both",
        parameters=[
            package_config("sub_bringup", "devices/flir_blackfly_s.yaml"),
            {
                "serial_number": vehicle.devices["down_camera"].serial_number,
                "frame_id": vehicle.devices["down_camera"].frame,
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
            package_config("sub_bringup", "devices/logitech_c922.yaml"),
            {
                "video_device": vehicle.devices["front_camera"].video_device,
                "frame_id": vehicle.devices["front_camera"].frame,
                # Its JPEG copy (image_raw/compressed), which the bag records.
                **jpeg_quality(
                    f"{vehicle.name}/front_camera", [f"/{vehicle.name}/front_camera/image_raw"]
                ),
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
            package_config("sub_bringup", "devices/waterlinked_dvl_a50.yaml"),
            {
                "ip_address": vehicle.devices["dvl"].ip_address,
                "frame_id": vehicle.devices["dvl"].frame,
            },
        ],
        remappings=[("~/odom", "odometry/dvl"), ("~/altitude", "altitude")],
    )

    sub_low_node = LifecycleNode(
        package="sub_serial_drivers",
        executable="sub_low",
        name="sub_low",
        namespace=vehicle.name,
        parameters=[
            {
                "device": vehicle.devices["maritime_mcu"].serial_port,
                "depth_frame_id": vehicle.frame("odom"),
                "depth_child_frame_id": vehicle.depth.frame,
                "depth_z_variance": vehicle.depth.z_variance,
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
                "device": vehicle.devices["imu"].serial_port,
                "frame_id": vehicle.devices["imu"].frame,
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
            # Transforms, EKF, controller, mission executive, tuning dashboard,
            # Foxglove bridge. A mission runs under the restart supervisor, so a
            # diver can kill the vehicle, carry it back to the start and release
            # it to rerun. The dashboard serves the laptops on the vehicle network.
            *common_launch_include("control/gains.yaml", restart="true", dashboard_host="0.0.0.0"),
            OpaqueFunction(function=_vehicle_entities),
            # The Logitech C922 publishes image_raw through usb_cam, and the
            # Blackfly through the Spinnaker driver.
            vision_node("sub_vision", "front_camera", "image_raw", "vision/detections"),
            vision_node("sub_vision_down", "down_camera", "image_raw", "vision/detections_down"),
            annotation_visualizer(
                "annotation_visualizer",
                "front_camera",
                "image_raw",
                "vision/detections",
                "vision/debug_image",
            ),
            annotation_visualizer(
                "annotation_visualizer_down",
                "down_camera",
                "image_raw",
                "vision/detections_down",
                "vision/debug_image_down",
            ),
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
