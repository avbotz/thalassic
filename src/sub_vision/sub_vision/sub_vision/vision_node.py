import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    ObjectHypothesisWithPose,
)

from sub_vision import constants, pnp, post_processors
from sub_vision.depth_backend import DepthManager
from sub_vision.model_manager import ModelManager
from sub_vision_interfaces.msg import Detection, DetectionArray
from sub_vision_interfaces.srv import LoadModel


class VisionNode(Node):
    """Perception node wiring the camera, model manager, and post-processors."""

    def __init__(self):
        super().__init__("sub_vision")

        self.declare_parameter("model_dir", "weights")
        self.declare_parameter("default_task", "")
        self.declare_parameter("backend", "auto")  # auto|tensorrt|onnxruntime
        self.declare_parameter("device", "auto")   # auto|cpu|cuda (onnxruntime only)
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
        self.declare_parameter("depth_model_path", "weights/depth_anything_v2_vits.onnx")
        self.declare_parameter("depth_backend", "auto")
        self.declare_parameter("depth_device", "auto")
        self.declare_parameter("depth_input_size", 518)
        self.declare_parameter("depth_rate_hz", 5.0)
        self.declare_parameter("depth_max_age_s", 0.15)
        self.declare_parameter("depth_debug_enabled", False)
        self.declare_parameter("depth_debug_topic", "vision/depth_anything")
        self.declare_parameter("depth_debug_color_topic", "vision/depth_anything_color")

        raw_model_dir = self.get_parameter("model_dir").value
        self._model_dir = (
            raw_model_dir
            if os.path.isabs(raw_model_dir)
            else os.path.join(get_package_share_directory("sub_vision"), raw_model_dir)
        )
        self._detections_task = ""
        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None
        self._last_infer_ms = float("nan")
        self._depth_manager = None
        self._depth_debug_pub = None
        self._depth_debug_color_pub = None
        self._processors = {}

        self._manager = ModelManager(
            model_dir=self._model_dir,
            backend_pref=self.get_parameter("backend").value,
            device_pref=self.get_parameter("device").value,
            input_size=int(self.get_parameter("input_size").value),
            conf_threshold=float(self.get_parameter("conf_threshold").value),
            warmup_iterations=int(self.get_parameter("warmup_iterations").value),
            log=self.get_logger().info,
        )
        if self.get_parameter("depth_enabled").value:
            depth_device = self.get_parameter("depth_device").value
            if depth_device == "auto":
                try:
                    import onnxruntime as ort
                    depth_device = "cuda" if "CUDAExecutionProvider" in ort.get_available_providers() else "cpu"
                except ImportError:
                    depth_device = "cpu"
            try:
                self._depth_manager = DepthManager(
                    self.get_parameter("depth_model_path").value, self.get_parameter("depth_backend").value, depth_device,
                    int(self.get_parameter("depth_input_size").value),
                    float(self.get_parameter("depth_rate_hz").value),
                    float(self.get_parameter("depth_max_age_s").value), self.get_logger().info,
                )
            except Exception as e:
                self.get_logger().error(f"Failed to initialize depth manager: {e}")
                self.get_logger().warn("Continuing without depth sidecar.")

        # Register the shipped per-task post-processors (gate, ...).
        post_processors.load_builtin_post_processors()

        # --- I/O ------------------------------------------------------------- #
        self._pub = self.create_publisher(
            DetectionArray, self.get_parameter("detections_topic").value, qos_profile_sensor_data
        )
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        if self.get_parameter("depth_debug_enabled").value:
            if not self.get_parameter("depth_enabled").value:
                self.get_logger().warn("depth debug requested while depth_enabled is false")
            self._depth_debug_pub = self.create_publisher(
                Image, self.get_parameter("depth_debug_topic").value, qos_profile_sensor_data
            )
            self._depth_debug_color_pub = self.create_publisher(
                Image, self.get_parameter("depth_debug_color_topic").value, qos_profile_sensor_data
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
                CompressedImage,
                f"{rgb_topic}/compressed",
                self._on_compressed_frame,
                qos_profile_sensor_data,
            )
        elif transport == "raw":
            self.create_subscription(
                Image,
                rgb_topic,
                self._on_frame,
                qos_profile_sensor_data,
            )
        else:
            raise ValueError(f"unsupported image_transport '{transport}' (raw|compressed)")

        self._load_srv = self.create_service(LoadModel, "~/load_model", self._on_load_model)
        self.create_timer(2.0, self._publish_diagnostics)

        # Optionally pre-load a task so the node is ready without a service call.
        default_task = self.get_parameter("default_task").value
        if default_task:
            self._manager.load(default_task)

        self.get_logger().info("sub_vision node ready")

    # ----------------------------------------------------------------------- #
    # Callbacks
    # ----------------------------------------------------------------------- #
    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_load_model(self, request, response):
        result = self._manager.load(request.task)
        response.success = result.success
        response.message = result.message
        response.active_model = result.active_model
        response.load_time_s = float(result.load_time_s)
        return response

    def _on_frame(self, rgb_msg: Image) -> None:
        self._process_frame(rgb_msg.header, lambda: self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8"))

    def _on_compressed_frame(self, rgb_msg: CompressedImage) -> None:
        self._process_frame(
            rgb_msg.header, lambda: self._bridge.compressed_imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        )

    def _process_frame(self, header, decode) -> None:
        task = self._manager.active_task
        rgb = decode()
        source_stamp_ns = int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
        depth = self._depth_manager.infer_if_due(rgb, source_stamp_ns) if self._depth_manager else None
        if depth is not None and depth.produced:
            self._publish_depth_debug(header, depth.map)

        if task is None:
            return  # depth debug may run without a detector task
        if self._camera_info is None:
            self.get_logger().warn("no CameraInfo yet; skipping frame", throttle_duration_sec=5.0)
            return

        t0 = self.get_clock().now()
        raw = self._manager.infer(rgb)
        self._last_infer_ms = (self.get_clock().now() - t0).nanoseconds / 1e6
        if raw is None:
            return

        msg = self._build_detection_array(header, task, raw.boxes, self._camera_info)
        # Per-task OpenCV enrichment (pose, extra). No-op fallback keeps the 2D
        # metadata when a task has no registered processor.
        processor = self._processors.get(task)
        if processor is None:
            processor = post_processors.get_post_processor(task)
            self._processors[task] = processor
        if processor is not None:
            # A cached relative-depth result must never influence a new RGB
            # frame. The torp processor only receives an exact source-stamp
            # match; its established YOLO/OpenCV outline path remains active
            # on all other frames.
            current_depth = depth if depth is not None and depth.current else None
            msg = processor.process(msg, rgb, current_depth, self._camera_info, raw.masks)

        self._pub.publish(msg)

    def _publish_depth_debug(self, header, depth: np.ndarray) -> None:
        """Publish raw relative depth and an auto-normalized color view."""
        if self._depth_debug_pub is None or self._depth_debug_color_pub is None:
            return
        raw = np.ascontiguousarray(depth.astype(np.float32))
        raw_msg = self._bridge.cv2_to_imgmsg(raw, encoding="32FC1")
        raw_msg.header = header
        self._depth_debug_pub.publish(raw_msg)

        valid = raw[np.isfinite(raw)]
        if valid.size == 0:
            gray = np.zeros(raw.shape, dtype=np.uint8)
        else:
            low, high = np.percentile(valid, (1.0, 99.0))
            if high <= low:
                high = low + 1.0
            gray = np.clip((raw - low) * 255.0 / (high - low), 0, 255).astype(np.uint8)
        color_msg = self._bridge.cv2_to_imgmsg(cv2.applyColorMap(gray, cv2.COLORMAP_TURBO), encoding="bgr8")
        color_msg.header = header
        self._depth_debug_color_pub.publish(color_msg)

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

            # TODO: Calculate depth based on bbox size

            out.detections.append(det)
        return out

    @staticmethod
    def _bearings(center, camera_info) -> tuple[float, float]:
        """Angles from the optical axis to a pixel, correcting for lens distortion.

        Positive horizontal = right of center, positive vertical = below
        center (optical-frame convention). See ``sub_vision.pnp`` for the
        undistortion this relies on.
        """
        if camera_info.k[0] <= 0.0 or camera_info.k[4] <= 0.0:  # uncalibrated source: no usable bearing
            return 0.0, 0.0
        return pnp.bearings_from_pixel(center.x, center.y, camera_info)

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
    def _publish_diagnostics(self) -> None:
        status = DiagnosticStatus(name="sub_vision", hardware_id="oak_d_pro")
        status.level = DiagnosticStatus.OK if self._manager.active_task else DiagnosticStatus.WARN
        status.message = (
            f"active: {self._manager.active_model}"
            if self._manager.active_task
            else "no model loaded"
        )
        status.values.append(KeyValue(key="active_task", value=str(self._manager.active_task)))
        status.values.append(KeyValue(key="last_infer_ms", value=f"{self._last_infer_ms:.1f}"))
        if self._depth_manager is not None:
            status.values.append(KeyValue(key="depth_last_infer_ms", value=f"{self._depth_manager.last_infer_ms:.1f}"))
            status.values.append(KeyValue(key="depth_stale_frames", value=str(self._depth_manager.stale_frames)))
        warmup = self._manager.last_warmup
        if warmup is not None:
            status.values.append(KeyValue(key="warmup_mean_ms", value=f"{warmup.mean_ms:.1f}"))

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.status.append(status)
        self._diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._depth_manager is not None:
            node._depth_manager.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
