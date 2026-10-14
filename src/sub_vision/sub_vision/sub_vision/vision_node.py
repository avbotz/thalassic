import os
import time
import traceback

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import (
    DiagnosticStatusWrapper,
    FrequencyStatusParam,
    TimeStampStatusParam,
    TopicDiagnostic,
    Updater,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    ObjectHypothesisWithPose,
)

from sub_vision import geometry, post_processors
from sub_vision.depth_backend import DepthModel
from sub_vision.model_manager import LoadResult, ModelManager
from sub_vision_interfaces.msg import Detection, DetectionArray
from sub_vision_interfaces.srv import LoadModel

# Only the newest frame is kept: with sensor-data QoS's depth of 5, a node slower than the camera
# works through a backlog and every detection comes from a frame up to 5 frames old.
FRAME_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

# Once a model is loaded, detections slower than this [Hz] are a warning in /diagnostics, and none
# at all (a camera that stopped) or one from a frame older than MAX_DETECTION_AGE [s] an error.
MIN_DETECTION_RATE = 1.0
MAX_DETECTION_AGE = 1.0


def workspace_path(path: str) -> str:
    """``path``, or if relative, resolved against the workspace root (as pixi sets it)."""
    return os.path.join(os.environ.get("PIXI_PROJECT_ROOT", "."), os.path.expanduser(path))


class VisionNode(Node):
    """Perception node wiring the camera, model manager, and post-processors."""

    def __init__(self):
        super().__init__("sub_vision")

        self.declare_parameter("model_dir", "weights")
        self.declare_parameter("default_task", "")
        self.declare_parameter("device", "auto")  # auto|cpu|cuda|tensorrt
        self.declare_parameter("input_size", 640)
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("warmup_iterations", 3)
        self.declare_parameter("rgb_topic", "front_camera/image_raw")
        self.declare_parameter("camera_info_topic", "front_camera/camera_info")
        self.declare_parameter("detections_topic", "vision/detections")
        # image_transport convention (rclpy has no image_transport bindings):
        # "raw" subscribes <rgb_topic>; "compressed" subscribes
        # <rgb_topic>/compressed — use it when frames cross the network.
        self.declare_parameter("image_transport", "raw")
        self.declare_parameter("depth_enabled", False)
        self.declare_parameter("depth_model", "depth_anything_v2_vits")
        self.declare_parameter("depth_input_size", 518)
        self.declare_parameter("depth_rate_hz", 5.0)
        self.declare_parameter("depth_debug", False)

        self._model_dir = workspace_path(self.get_parameter("model_dir").value)
        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None
        self._last_infer_ms = float("nan")
        # One instance per task, kept for the node's lifetime: torp carries state between frames.
        self._processors: dict[str, post_processors.TaskPostProcessor | None] = {}

        self._manager = ModelManager(
            model_dir=self._model_dir,
            device_pref=self.get_parameter("device").value,
            input_size=int(self.get_parameter("input_size").value),
            conf_threshold=float(self.get_parameter("conf_threshold").value),
            warmup_iterations=int(self.get_parameter("warmup_iterations").value),
            log=self.get_logger().info,
        )

        self._depth: DepthModel | None = None
        if self.get_parameter("depth_enabled").value:
            try:
                self._depth = DepthModel(
                    model_dir=self._model_dir,
                    name=self.get_parameter("depth_model").value,
                    device_pref=self.get_parameter("device").value,
                    input_size=int(self.get_parameter("depth_input_size").value),
                    rate_hz=float(self.get_parameter("depth_rate_hz").value),
                    log=self.get_logger().info,
                )
            except Exception as exc:  # detection does not need depth; run without it
                self.get_logger().error(f"depth model failed to load, running without: {exc}")

        # Register the shipped per-task post-processors (gate, ...).
        post_processors.load_builtin_post_processors()

        # --- I/O ------------------------------------------------------------- #
        self._pub = self.create_publisher(
            DetectionArray, self.get_parameter("detections_topic").value, qos_profile_sensor_data
        )
        self._depth_pub = self._depth_color_pub = None
        if self._depth is not None and self.get_parameter("depth_debug").value:
            self._depth_pub = self.create_publisher(Image, "~/depth", qos_profile_sensor_data)
            self._depth_color_pub = self.create_publisher(
                Image, "~/depth/color", qos_profile_sensor_data
            )

        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._on_camera_info,
            qos_profile_sensor_data,
        )

        rgb_topic = self.get_parameter("rgb_topic").value
        transport = self.get_parameter("image_transport").value
        if transport == "compressed":
            self.create_subscription(
                CompressedImage, f"{rgb_topic}/compressed", self._on_compressed_frame, FRAME_QOS
            )
        elif transport == "raw":
            self.create_subscription(Image, rgb_topic, self._on_frame, FRAME_QOS)
        else:
            raise ValueError(f"unsupported image_transport '{transport}' (raw|compressed)")

        self._load_srv = self.create_service(LoadModel, "~/load_model", self._on_load_model)

        # One instance per camera (sub_vision, sub_vision_down), told apart by node name.
        self._diagnostics = Updater(self, period=2.0)
        self._diagnostics.setHardwareID(rgb_topic)
        self._diagnostics.add("detector", self._diagnose)
        # The detections' rate and age, from the first model loaded: before that there are none.
        self._detections_diagnostic: TopicDiagnostic | None = None

        # Optionally pre-load a task so the node is ready without a service call.
        default_task = self.get_parameter("default_task").value
        if default_task:
            self._load(default_task)

        self.get_logger().info("sub_vision node ready")

    # ----------------------------------------------------------------------- #
    # Callbacks
    # ----------------------------------------------------------------------- #
    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_load_model(
        self, request: LoadModel.Request, response: LoadModel.Response
    ) -> LoadModel.Response:
        result = self._load(request.task)
        response.success = result.success
        response.message = result.message
        response.active_model = result.active_model
        response.load_time_s = float(result.load_time_s)
        return response

    def _load(self, task: str) -> LoadResult:
        result = self._manager.load(task)
        if not result.success:
            self.get_logger().error(result.message)
        elif self._detections_diagnostic is None:
            self._detections_diagnostic = TopicDiagnostic(
                "detections",
                self._diagnostics,
                FrequencyStatusParam({"min": MIN_DETECTION_RATE}),
                TimeStampStatusParam(max_acceptable=MAX_DETECTION_AGE),
            )
        return result

    def _on_frame(self, rgb_msg: Image) -> None:
        self._process_frame(
            rgb_msg.header, lambda: self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        )

    def _on_compressed_frame(self, rgb_msg: CompressedImage) -> None:
        self._process_frame(
            rgb_msg.header,
            lambda: self._bridge.compressed_imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8"),
        )

    def _process_frame(self, header, decode) -> None:
        if self._manager.active_task is None and self._depth is None:
            return  # no model loaded yet
        # An exception escaping a callback would end spin(), and the node with it: drop the frame.
        try:
            self._process_image(header, decode())
        except Exception:
            self.get_logger().error(
                f"dropped a frame:\n{traceback.format_exc()}", throttle_duration_sec=5.0
            )

    def _process_image(self, header, rgb: np.ndarray) -> None:
        # At depth_rate_hz; None on the frames between, so a map never meets another frame.
        depth = self._infer_depth(rgb)
        if depth is not None and self._depth_pub is not None:
            self._publish_depth(header, depth)

        if self._manager.active_task is None:
            return
        camera_info = self._camera_info
        if camera_info is None:
            self.get_logger().warn("no CameraInfo yet; skipping frame", throttle_duration_sec=5.0)
            return
        if camera_info.k[0] <= 0.0 or camera_info.k[4] <= 0.0:
            self.get_logger().warn(
                "CameraInfo has no intrinsics (uncalibrated camera): every bearing is 0",
                throttle_duration_sec=30.0,
            )

        # Wall time: the node's clock is the simulator's under use_sim_time
        t0 = time.perf_counter()
        result = self._manager.infer(rgb)
        self._last_infer_ms = (time.perf_counter() - t0) * 1000.0
        if result is None:
            return
        task, raw = result

        msg = self._build_detection_array(header, task, raw.boxes, camera_info)

        # Per-task OpenCV enrichment (pose, extra). No-op fallback keeps the 2D
        # metadata when a task has no registered processor.
        if task not in self._processors:
            self._processors[task] = post_processors.get_post_processor(task)
            if self._processors[task] is None:
                self.get_logger().info(
                    f"no post-processor for task '{task}'; publishing 2D detections only"
                )
        processor = self._processors[task]
        if processor is not None:
            msg = processor.process(msg, rgb, depth, camera_info, raw.masks)

        self._pub.publish(msg)
        # diagnostic_updater's Python TimeStampStatus ages a stamp on the wall clock, not on the
        # node's (the simulator's under use_sim_time), so it is given the frame's age on the
        # node's clock as a wall-clock stamp.
        age = (self.get_clock().now() - Time.from_msg(header.stamp)).nanoseconds * 1e-9
        self._detections_diagnostic.tick(time.time() - age)

    def _infer_depth(self, rgb: np.ndarray) -> np.ndarray | None:
        if self._depth is None:
            return None
        try:
            return self._depth.infer_if_due(rgb)
        except Exception as exc:  # detection does not need depth; run this frame without it
            self.get_logger().error(f"depth inference failed: {exc}", throttle_duration_sec=5.0)
            return None

    def _publish_depth(self, header, depth: np.ndarray) -> None:
        """The relative depth map as 32FC1, and a Turbo rendering stretched over its 1-99th
        percentiles for viewing."""
        depth_msg = self._bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_msg.header = header
        self._depth_pub.publish(depth_msg)

        low, high = np.percentile(depth, (1.0, 99.0))
        gray = np.clip((depth - low) * 255.0 / max(high - low, 1e-6), 0, 255).astype(np.uint8)
        color_msg = self._bridge.cv2_to_imgmsg(
            cv2.applyColorMap(gray, cv2.COLORMAP_TURBO), encoding="bgr8"
        )
        color_msg.header = header
        self._depth_color_pub.publish(color_msg)

    # ----------------------------------------------------------------------- #
    # Message assembly
    # ----------------------------------------------------------------------- #
    def _build_detection_array(self, header, task, raw, camera_info) -> DetectionArray:
        out = DetectionArray()
        out.header = header
        out.task = task

        for x1, y1, x2, y2, score, cls in raw:
            det = Detection()
            det.detection = self._make_detection2d(header, x1, y1, x2, y2, score, cls)
            det.bearing_horizontal, det.bearing_vertical = self._bearings(
                det.detection.bbox.center.position, camera_info
            )
            det.distance_m = float("nan")  # unknown until a post-processor ranges the object
            out.detections.append(det)
        return out

    @staticmethod
    def _bearings(center, camera_info) -> tuple[float, float]:
        """Angles from the optical axis to a pixel, with lens distortion removed.

        Positive horizontal = right of center, positive vertical = below center
        (optical-frame convention).
        """
        if camera_info.k[0] <= 0.0 or camera_info.k[4] <= 0.0:  # uncalibrated: no usable bearing
            return 0.0, 0.0
        return geometry.bearings_from_pixel(center.x, center.y, camera_info)

    @staticmethod
    def _make_detection2d(header, x1, y1, x2, y2, score, cls) -> Detection2D:
        det2d = Detection2D()
        det2d.header = header

        bbox = BoundingBox2D()
        bbox.center.position.x = float((x1 + x2) / 2.0)
        bbox.center.position.y = float((y1 + y2) / 2.0)
        bbox.size_x = float(x2 - x1)
        bbox.size_y = float(y2 - y1)
        det2d.bbox = bbox

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(int(cls))
        hyp.hypothesis.score = float(score)
        det2d.results.append(hyp)
        return det2d

    # ----------------------------------------------------------------------- #
    # Diagnostics
    # ----------------------------------------------------------------------- #
    def _diagnose(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        # Without intrinsics every bearing is 0, so alignments "succeed" at once.
        camera_info = self._camera_info
        if camera_info is None:
            stat.summary(DiagnosticStatus.ERROR, "no camera_info")
        elif camera_info.k[0] <= 0.0 or camera_info.k[4] <= 0.0:
            stat.summary(
                DiagnosticStatus.ERROR,
                "camera not calibrated (camera_info has no intrinsics): bearings are 0",
            )
        elif self._manager.active_task:
            stat.summary(DiagnosticStatus.OK, f"active: {self._manager.active_model}")
        else:
            stat.summary(DiagnosticStatus.WARN, "no model loaded")
        stat.add("active_task", str(self._manager.active_task))
        stat.add("last_infer_ms", f"{self._last_infer_ms:.1f}")
        if self._depth is not None:
            stat.add("depth_infer_ms", f"{self._depth.last_infer_ms:.1f}")
        warmup = self._manager.last_warmup
        if warmup is not None:
            stat.add("warmup_mean_ms", f"{warmup.mean_ms:.1f}")
            stat.add("warmup_last_ms", f"{warmup.last_ms:.1f}")
        return stat

    def destroy_node(self) -> None:
        self._manager.close()
        if self._depth is not None:
            self._depth.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
