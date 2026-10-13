"""Building blocks the launch files share.

common_launch.py holds the nodes both stacks run identically, and both
top-level launch files include it. This module holds what those files build
the same way but wire differently: the launch arguments, the vision nodes
(each stack's cameras publish on different topics) and the run's recorder.
"""

import os
import subprocess
from collections.abc import Sequence
from datetime import datetime
from pathlib import Path

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def package_config(package: str, filename: SomeSubstitutionsType) -> PathJoinSubstitution:
    """A config file inside an installed package's share directory."""
    return PathJoinSubstitution([FindPackageShare(package), "config", filename])


def common_launch_arguments(
    gains: str = "control/gains.yaml",
    use_sim_time: str = "false",
    foxglove_compression: str = "true",
    restart: str = "false",
) -> list[DeclareLaunchArgument]:
    """The arguments every bringup launch file accepts.

    Declared by the top-level launch files and by common_launch.py, so each can
    run on its own. An include shares the parent's launch configurations, and a
    default only applies when the configuration is still unset, so the parent's
    defaults (`gains`, `use_sim_time`, ...) are what reach common_launch.py.
    """
    return [
        DeclareLaunchArgument(
            "robot_name",
            default_value="marlin_v3",
            description="Vehicle to bring up; selects config/vehicles/<robot_name>.yaml",
        ),
        DeclareLaunchArgument(
            "mission",
            default_value="",
            description=(
                "Mission tree to execute (resources/missions/<name>.xml or a path); "
                "empty skips sub_mission"
            ),
        ),
        DeclareLaunchArgument(
            "role",
            default_value="SURVEY",
            # sub_mission refuses anything else, but only once it starts.
            choices=["SURVEY", "SEARCH"],
            description="Mission vision role",
        ),
        DeclareLaunchArgument(
            "restart",
            default_value=restart,
            description=(
                "Run the mission under sub_mission's restart supervisor, which stops it when "
                "the kill switch is pulled and starts it again from the beginning on release"
            ),
        ),
        DeclareLaunchArgument(
            "groot_port",
            default_value="5555",
            description=(
                "Port of the mission's Groot2 live view, which takes the next port too; "
                "0 turns it off"
            ),
        ),
        DeclareLaunchArgument(
            "gains",
            default_value=gains,
            description=(
                "sub_control profile (vehicle model, trajectory limits, gains), "
                "a path relative to sub_bringup/config"
            ),
        ),
        DeclareLaunchArgument(
            "foxglove_port",
            default_value="8765",
            description="Websocket port for the Foxglove bridge; change it to run two stacks",
        ),
        DeclareLaunchArgument(
            "foxglove_compression",
            default_value=foxglove_compression,
            description=(
                "Deflate the Foxglove bridge's messages: saves bandwidth to a remote client, "
                "costs CPU and latency per message (images especially)"
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=use_sim_time,
            description="Run every node on the simulator's /clock instead of the wall clock",
        ),
        DeclareLaunchArgument(
            "jpeg_quality",
            default_value="85",
            description=(
                "JPEG quality (1-100) of the cameras' compressed images, which the bag "
                "records and Foxglove shows; 85 is about half the size of image_transport's 95"
            ),
        ),
        DeclareLaunchArgument(
            "record",
            default_value="false",
            description="Record the run into an MCAP bag under bag_dir",
        ),
        DeclareLaunchArgument(
            "bag_dir",
            default_value=PathJoinSubstitution(
                [EnvironmentVariable("PIXI_PROJECT_ROOT", default_value="."), "bags"]
            ),
            description="Where each run's bag goes, in a folder named after the run",
        ),
    ]


def common_launch_include(**defaults: str) -> list:
    """The arguments above, with `defaults` as their defaults, plus common_launch.py."""
    return [
        *common_launch_arguments(**defaults),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("sub_bringup"), "launch", "common_launch.py"])
        ),
    ]


def sim_time() -> dict:
    """The use_sim_time parameter, from the launch argument of the same name."""
    return {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}


def vision_entities(
    image_topic: str, parameters: dict | None = None, front_parameters: dict | None = None
) -> list[Node]:
    """A sub_vision node and an annotation visualizer per camera, reading <camera>/<image_topic>.

    sub_mission finds each camera's instance by its name and detections topic
    (sub_mission/src/vision_client.cpp): sub_vision publishes vision/detections
    from the front camera and sub_vision_down vision/detections_down from the
    down camera. `parameters` override sub_vision.yaml for both instances,
    `front_parameters` for the front one only.

    The visualizers draw those detections over the images, on
    vision/debug_image and vision/debug_image_down, only while something
    subscribes. The node reads front_camera/image_color and vision/detections
    and writes vision/debug_image, each with its /compressed JPEG copy, so it
    is wired to a camera by remapping those names.
    """
    entities = []
    for camera, suffix, extra in (
        ("front_camera", "", front_parameters),
        ("down_camera", "_down", None),
    ):
        image = f"{camera}/{image_topic}"
        detections = f"vision/detections{suffix}"
        debug = f"vision/debug_image{suffix}"
        entities += [
            Node(
                package="sub_vision",
                executable="sub_vision",
                name=f"sub_vision{suffix}",
                namespace=LaunchConfiguration("robot_name"),
                output="screen",
                parameters=[
                    package_config("sub_vision", "sub_vision.yaml"),
                    {
                        "rgb_topic": image,
                        "camera_info_topic": f"{camera}/camera_info",
                        "detections_topic": detections,
                        **(parameters or {}),
                        **(extra or {}),
                    },
                    sim_time(),
                ],
            ),
            Node(
                package="sub_vision",
                executable="annotation_visualizer",
                name=f"annotation_visualizer{suffix}",
                namespace=LaunchConfiguration("robot_name"),
                output="screen",
                parameters=[sim_time()],
                remappings=[
                    ("front_camera/image_color", image),
                    ("front_camera/image_color/compressed", f"{image}/compressed"),
                    ("vision/detections", detections),
                    ("vision/debug_image", debug),
                    ("vision/debug_image/compressed", f"{debug}/compressed"),
                ],
            ),
        ]
    return entities


def compressed_parameter(node_namespace: str, image_topic: str, name: str, value) -> dict:
    """One parameter of the compressed image_transport publisher of a node's image topic.

    compressed_image_transport names each publisher's parameters after its image
    topic with the first len(node namespace) characters cut off, whether or not
    the topic is in that namespace, and dots for slashes: /marlin_v3/front_camera/image_color
    published from /stonefish_ros2 is t_camera.image_color.compressed.<name>.
    """
    namespace = "/" + node_namespace.strip("/")
    return {image_topic[len(namespace) :].replace("/", ".") + f".compressed.{name}": value}


def jpeg_quality(node_namespace: str, image_topics: Sequence[str]) -> dict:
    """The jpeg_quality parameters of a node's compressed image publishers, from the launch argument."""
    quality = ParameterValue(LaunchConfiguration("jpeg_quality"), value_type=int)
    parameters = {}
    for topic in image_topics:
        parameters |= compressed_parameter(node_namespace, topic, "jpeg_quality", quality)
    return parameters


# Launch arguments that say what the run was, kept in the bag's metadata. The
# sim's own (seed, course, ...) are what its scenario was rendered from.
_RUN_ARGUMENTS = (
    *("robot_name", "mission", "role", "gains"),
    *("seed", "course", "current", "sensor_noise", "DX", "DY", "DZ", "DYAW"),
)


def _git_describe() -> str | None:
    """The workspace's commit, with -dirty if it has uncommitted changes."""
    workspace = os.environ.get("PIXI_PROJECT_ROOT", ".")
    try:
        return subprocess.run(
            ["git", "-C", workspace, "describe", "--always", "--dirty"],
            capture_output=True,
            text=True,
            check=True,
            timeout=5,
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def recorder(exclude: Sequence[str]) -> OpaqueFunction:
    """Records the run into <bag_dir>/<date>_<time>_<sim or robot_name>[_<mission>].

    Every topic goes into one MCAP file, zstd-compressed in chunks, except
    image_transport's theora, zstd and compressedDepth copies of each image and
    the topics in `exclude`: regular expressions matched against the end of a
    topic's name. The top-level launch files exclude each colour camera's raw
    images, so it is recorded as its JPEG copy (<image>/compressed) instead.
    """

    def record(context, *_, **__):
        if not IfCondition(LaunchConfiguration("record")).evaluate(context):
            return []
        config = context.launch_configurations
        # 1 and True count, as for the nodes' use_sim_time, so the bag's clock is theirs.
        sim = IfCondition(LaunchConfiguration("use_sim_time")).evaluate(context)
        run = [
            datetime.now().strftime("%Y-%m-%d_%H-%M-%S"),
            "sim" if sim else config["robot_name"],
            Path(config["mission"]).stem if config.get("mission") else "",
        ]
        bag = Path(config["bag_dir"]).expanduser().absolute() / "_".join(filter(None, run))

        custom_data = [f"{key}={config[key]}" for key in _RUN_ARGUMENTS if config.get(key)]
        if commit := _git_describe():
            custom_data.append(f"commit={commit}")

        topics = "|".join(exclude)
        return [
            LogInfo(msg=f"Recording this run to {bag}"),
            ExecuteProcess(
                name="recorder",
                output="screen",
                cmd=[
                    *("ros2", "bag", "record", "--all-topics"),
                    *("--exclude-regex", f"/(compressedDepth|theora|zstd)$|/({topics})$"),
                    *("--output", str(bag), "--storage", "mcap"),
                    *("--storage-config-file", package_config("sub_bringup", "mcap_writer.yaml")),
                    # Messages from sim nodes are stamped with sim time, so the bag is too.
                    *(["--use-sim-time"] if sim else []),
                    *("--custom-data", *custom_data),
                    *("--disable-keyboard-controls", "--log-level", "warn"),
                ],
            ),
        ]

    return OpaqueFunction(function=record)
