"""Bring the stack up against the Stonefish simulator.

Renders a randomised scenario, starts Stonefish and the sensor bridges, then
the same estimation, control, vision and mission stack the vehicle runs.
"""

import random
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from sub_bringup.entities import (
    annotation_visualizer,
    common_launch_include,
    jpeg_quality,
    package_config,
    recorder,
    sim_time,
    vision_node,
)
from sub_bringup.vehicle import load_vehicle
from sub_sim.generate_robot import mount_poses, render_robot_scenario, thruster_specs
from sub_sim.randomize_locs import randomize_scenario_locations
from sub_sim.robot_scenario_to_urdf import robot_scenario_to_urdf


def _render_scn(context, *_, **__):
    """Render a randomised scenario and the URDF of the robot placed in it."""
    robot_name = LaunchConfiguration("robot_name").perform(context)
    offsets = {
        k: float(LaunchConfiguration(k).perform(context)) for k in ("DX", "DY", "DZ", "DYAW")
    }
    # Always seeded, so the seed recorded with the run reproduces its scenario.
    seed = LaunchConfiguration("seed").perform(context) or str(random.randrange(2**31))
    # As IfCondition reads them (true, false, 1 or 0; anything else is an error), so
    # labeling agrees with sim_labeling's condition.
    sensor_noise, labeling, depth_camera = (
        IfCondition(LaunchConfiguration(k)).evaluate(context)
        for k in ("sensor_noise", "labeling", "depth_camera")
    )
    # Given in ENU like everything else; Stonefish's world is NED.
    current = LaunchConfiguration("current").perform(context)
    try:
        east, north, up = (float(v) for v in current.split())
    except ValueError as e:
        raise ValueError(f'current must be ENU "x y z" in m/s, not {current!r}') from e
    current_ned = f"{north} {east} {-up}"

    sub_sim_share = Path(get_package_share_directory("sub_sim"))
    robot_scenario_file = sub_sim_share / "data" / "robots" / robot_name / "layout.scn.j2"
    # Thrusters and actuators go where the vehicle description says, as for the
    # allocator and the transforms.
    vehicle = load_vehicle(robot_name)
    thrusters = thruster_specs(vehicle.thrusters)
    frames = mount_poses(vehicle)

    # The scenario template calls back into this with the robot's pose.
    rendered_robot = None

    def render_robot(**pose):
        nonlocal rendered_robot
        rendered_robot = render_robot_scenario(
            robot_scenario_file,
            **pose,
            thrusters=thrusters,
            frames=frames,
            sensor_noise=sensor_noise,
            # sim_labeling labels the segmentation camera's images.
            segmentation_camera=labeling,
            depth_camera=depth_camera,
        )
        return rendered_robot.as_posix()

    scenario = randomize_scenario_locations(
        scenario_template_file=sub_sim_share / "scenarios" / "woollett.scn.j2",
        seed=int(seed),
        render_robot=render_robot,
        current=current_ned,
        course=LaunchConfiguration("course").perform(context),
        **offsets,
    )
    if rendered_robot is None:
        raise RuntimeError("The scenario template did not render a robot")

    urdf = robot_scenario_to_urdf(
        scenario_xml=rendered_robot,
        robot_name=robot_name,
        mesh_prefix=f"file://{sub_sim_share / 'data'}/",
    )
    robot_description = urdf.read_text()
    urdf.unlink()

    # The rendered files are temporary, and Stonefish reads them only as it starts.
    def remove_scenario(*_):
        for path in (scenario, rendered_robot):
            path.unlink(missing_ok=True)

    return [
        SetLaunchConfiguration("seed", seed),
        SetLaunchConfiguration("scenario_file", scenario.as_posix()),
        SetLaunchConfiguration("robot_description", robot_description),
        RegisterEventHandler(OnShutdown(on_shutdown=remove_scenario)),
    ]


def _stonefish(context, *_, **__):
    """The simulator, whose camera parameters are named after the robot's camera topics."""
    robot_name = LaunchConfiguration("robot_name").perform(context)
    return [
        Node(
            package="stonefish_ros2",
            executable="stonefish_simulator",
            name="stonefish_simulator",
            namespace="stonefish_ros2",
            output="screen",
            arguments=[
                PathJoinSubstitution([FindPackageShare("sub_sim"), "data"]),
                LaunchConfiguration("scenario_file"),
                "500.0",  # simulation rate [Hz]
                LaunchConfiguration("window_width"),
                LaunchConfiguration("window_height"),
                LaunchConfiguration("rendering_quality"),
            ],
            parameters=[
                {
                    # Everything else runs on this clock (use_sim_time).
                    "publish_clock": True,
                    "realtime_factor": 1.0,
                    # /clock and the robot's joint and thruster states, published from the
                    # physics thread at a cost per subscriber. A whole number of physics
                    # steps, into which the 50 Hz control and EKF periods divide evenly.
                    "state_publish_rate": 250.0,
                    # The cameras' JPEG copies (image_color/compressed), which the bag
                    # records and Foxglove shows.
                    **jpeg_quality(
                        "stonefish_ros2",
                        [
                            f"/{robot_name}/{camera}/image_color"
                            for camera in ("front_camera", "down_camera")
                        ],
                    ),
                }
            ],
            # A shared-memory segment large enough for the camera images, which the
            # default one drops and delays (see the profile). The profile sets up
            # localhost-only discovery itself, so rmw must not.
            additional_env={
                "FASTRTPS_DEFAULT_PROFILES_FILE": package_config(
                    "sub_bringup", "stonefish_fastdds.xml"
                ),
                "ROS_AUTOMATIC_DISCOVERY_RANGE": "SYSTEM_DEFAULT",
            },
        )
    ]


def _sim_sensors(context, *_, **__):
    """The sensor bridges, which need the vehicle description, so robot_name must be resolved first."""
    vehicle = load_vehicle(LaunchConfiguration("robot_name").perform(context))
    robot_name = LaunchConfiguration("robot_name")

    # The grabber holds things between its jaws, below their hinges.
    frames = mount_poses(vehicle)
    hinges = [frames[jaw].xyz for jaw in ("left_grabber_link", "right_grabber_link")]
    grasp_point = [(left + right) / 2 for left, right in zip(*hinges, strict=True)]
    grasp_point[2] += 0.07

    def sim_component(executable, plugin, parameters=(), **kwargs):
        return ComposableNode(
            package="sub_sim_sensors",
            plugin=plugin,
            name=executable,
            namespace=robot_name,
            parameters=[*parameters, sim_time()],
            extra_arguments=[{"use_intra_process_comms": True}],
            **kwargs,
        )

    sim_sensors_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        name="sim_sensors_container",
        namespace=robot_name,
        output="both",
        composable_node_descriptions=[
            sim_component(
                "sim_dvl_remapper",
                "SimDVLRemapper",
                parameters=[{"dvl_link": vehicle.devices["dvl"].frame}],
            ),
            sim_component(
                "sim_imu_remapper",
                "SimIMURemapper",
                parameters=[{"imu_link": vehicle.devices["imu"].frame}],
            ),
            sim_component(
                "sim_pressure_to_depth",
                "SimPressureToDepth",
                # Reports exactly like sub_low does on the vehicle.
                parameters=[
                    {
                        "frame_id": vehicle.frame("odom"),
                        "child_frame_id": vehicle.depth.frame,
                        "z_variance": vehicle.depth.z_variance,
                    }
                ],
            ),
            # Stands in for the ESCs; injects thruster faults on request.
            sim_component("sim_thrusters", "SimThrusters"),
            sim_component("sim_torpedo_launcher", "SimTorpedoLauncher"),
            sim_component(
                "sim_dropper",
                "SimDropper",
                parameters=[{"joint_name": [robot_name, "/dropper_joint"]}],
            ),
            sim_component(
                "sim_grabber",
                "SimGrabber",
                parameters=[
                    {
                        "left_joint": [robot_name, "/left_grabber_joint"],
                        "right_joint": [robot_name, "/right_grabber_joint"],
                        "grasp_point": grasp_point,
                    }
                ],
            ),
            sim_component(
                "sim_kill_switch",
                "SimKillSwitch",
                parameters=[{"off_delay": 6.0}],
            ),
        ],
        parameters=[sim_time()],
        ros_arguments=["--disable-stdout-logs"],
    )

    # Declares thrusters failed when they stop pushing what they are commanded
    # to (from the thrust sim_thrusters measures), so sub_control works around them.
    thruster_monitor = Node(
        package="sub_control",
        executable="thruster_monitor",
        name="thruster_monitor",
        namespace=robot_name,
        output="both",
        parameters=[
            {
                "allocation.thruster_reversed": vehicle.thruster_layout()[
                    "allocation.thruster_reversed"
                ]
            },
            sim_time(),
        ],
    )
    return [sim_sensors_container, thruster_monitor]


def sim_entities() -> list:
    robot_name = LaunchConfiguration("robot_name")

    args = [
        DeclareLaunchArgument(
            "seed",
            default_value="",
            description="Seed for the scenario's random placement; empty picks one at random",
        ),
        DeclareLaunchArgument("DX", default_value="0.25"),
        DeclareLaunchArgument("DY", default_value="0.25"),
        DeclareLaunchArgument("DZ", default_value="0.10"),
        DeclareLaunchArgument("DYAW", default_value="0.10"),
        DeclareLaunchArgument(
            "course",
            default_value="D",
            choices=["A", "B1", "C1", "D"],
            description="The dock the robot starts from on the RoboSub 2026 course map",
        ),
        DeclareLaunchArgument(
            "labeling",
            default_value="false",
            description="Write labelled training images from the segmentation camera",
        ),
        DeclareLaunchArgument(
            "current",
            default_value="0.02 0.03 0.0",
            description='Uniform water current, ENU "x y z" in m/s (a pool pump moves a few cm/s)',
        ),
        DeclareLaunchArgument(
            "depth_camera",
            default_value="false",
            description="Add the depth camera (sim/depth_camera), which nothing in the stack reads yet",
        ),
        # The GPU renders this window and one sensor camera per frame, so a camera's
        # rate is the window's frame rate over the number of cameras: the window's
        # size trades against every camera's rate. Quality also changes the cameras' images.
        DeclareLaunchArgument("window_width", default_value="1280"),
        DeclareLaunchArgument("window_height", default_value="720"),
        DeclareLaunchArgument(
            "rendering_quality",
            default_value="medium",
            # Stonefish renders anything else as medium.
            choices=["low", "medium", "high"],
            description="Stonefish rendering quality, for the window and the cameras",
        ),
        DeclareLaunchArgument(
            "sensor_noise",
            default_value="true",
            description="Give the simulated DVL, IMU and pressure sensor the noise of the real ones",
        ),
        DeclareLaunchArgument(
            "vision_depth",
            default_value="false",
            description=(
                "Run Depth Anything on the front camera beside the detector (torp uses it to find "
                "the board) and publish its maps; each run delays that frame's detections"
            ),
        ),
    ]

    # Publishes the URDF rendered from the scenario; the vehicle has no equivalent.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        parameters=[
            # As a string: launch_ros otherwise parses it as YAML, which folds its lines
            # and would cut it at a " #" or reject it at a ": ".
            {
                "robot_description": ParameterValue(
                    LaunchConfiguration("robot_description"), value_type=str
                )
            },
            sim_time(),
        ],
    )

    sim_labeling_node = Node(
        package="sim_labeling",
        executable="labeling",
        name="sim_labeling",
        namespace=robot_name,
        condition=IfCondition(LaunchConfiguration("labeling")),
        parameters=[sim_time()],
    )

    return [
        *args,
        OpaqueFunction(function=_render_scn),
        OpaqueFunction(function=_stonefish),
        OpaqueFunction(function=_sim_sensors),
        robot_state_publisher,
        sim_labeling_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            # OpenCV's OpenMP workers (cv_bridge colour conversion, the compressed
            # image transport) otherwise spin between frames: with several image
            # nodes that is a few cores, taken from the physics thread.
            SetEnvironmentVariable("OMP_WAIT_POLICY", "PASSIVE"),
            # Transforms, EKF, controller, mission executive, tuning dashboard,
            # Foxglove bridge.
            # Foxglove connects over localhost, where compression only adds latency.
            *common_launch_include(
                "control/gains_sim.yaml", use_sim_time="true", foxglove_compression="false"
            ),
            *sim_entities(),
            # Stonefish publishes both cameras as rgb8 on image_color; cv_bridge
            # converts to bgr8 inside the node, so no bridge is needed.
            vision_node(
                "sub_vision",
                "front_camera",
                "image_color",
                "vision/detections",
                parameters={
                    key: ParameterValue(LaunchConfiguration("vision_depth"), value_type=bool)
                    for key in ("depth_enabled", "depth_debug")
                },
            ),
            vision_node("sub_vision_down", "down_camera", "image_color", "vision/detections_down"),
            annotation_visualizer(
                "annotation_visualizer",
                "front_camera",
                "image_color",
                "vision/detections",
                "vision/debug_image",
            ),
            annotation_visualizer(
                "annotation_visualizer_down",
                "down_camera",
                "image_color",
                "vision/detections_down",
                "vision/debug_image_down",
            ),
            # After sim_entities(), which picks the seed the bag records.
            recorder(
                exclude=[
                    # Recorded as JPEG (image_color/compressed): a tenth the size or less.
                    "front_camera/image_color",
                    "down_camera/image_color",
                    # The annotators' drawings, raw and JPEG, of images and
                    # detections that are recorded; recording them would also
                    # keep the annotators drawing.
                    "vision/debug_image.*",
                    # vision_depth's maps, about 6 MB a frame, for viewing.
                    "sub_vision.*/depth(/color)?",
                    # The bag's timestamps are already sim time.
                    "clock",
                    # The depth camera's images are 32-bit float, which its JPEG copy
                    # cannot hold (subscribing makes it log an error every frame),
                    # so they are recorded raw.
                    "sim/depth_camera/image_depth/compressed",
                    # sim_labeling saves what the segmentation camera sees itself.
                    "sim/segment/.*",
                ]
            ),
        ]
    )
