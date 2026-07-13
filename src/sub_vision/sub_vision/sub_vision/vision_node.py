import math
import os

import rclpy
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

from sub_vision import constants, post_processors
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

        self._model_dir = self.get_parameter("model_dir").value
        self._detections_task = ""
        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None
        self._last_infer_ms = float("nan")

        self._manager = ModelManager(
            model_dir=self._model_dir,
            backend_pref=self.get_parameter("backend").value,
            device_pref=self.get_parameter("device").value,
            input_size=int(self.get_parameter("input_size").value),
            conf_threshold=float(self.get_parameter("conf_threshold").value),
            warmup_iterations=int(self.get_parameter("warmup_iterations").value),
            log=self.get_logger().info,
        )

        # Register the shipped per-task post-processors (gate, ...).
        post_processors.load_builtin_post_processors()

        # --- I/O ------------------------------------------------------------- #
        self._pub = self.create_publisher(
            DetectionArray, self.get_parameter("detections_topic").value, qos_profile_sensor_data
        )
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

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
        if task is None:
            return  # no model loaded yet
        if self._camera_info is None:
            self.get_logger().warn("no CameraInfo yet; skipping frame", throttle_duration_sec=5.0)
            return

        rgb = decode()

        t0 = self.get_clock().now()
        raw = self._manager.infer(rgb)
        self._last_infer_ms = (self.get_clock().now() - t0).nanoseconds / 1e6
        if raw is None:
            return

        msg = self._build_detection_array(header, task, raw.boxes, self._camera_info)

        # Per-task OpenCV enrichment (pose, extra). No-op fallback keeps the 2D
        # metadata when a task has no registered processor.
        processor = post_processors.get_post_processor(task)
        if processor is not None:
            msg = processor.process(msg, rgb, None, self._camera_info, raw.masks)

        self._pub.publish(msg)

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
        """Angles from the optical axis to a pixel, via the pinhole intrinsics.

        K = [fx 0 cx; 0 fy cy; 0 0 1]. Positive horizontal = right of center,
        positive vertical = below center (optical-frame convention).
        """
        fx, cx = camera_info.k[0], camera_info.k[2]
        fy, cy = camera_info.k[4], camera_info.k[5]
        if fx <= 0.0 or fy <= 0.0:  # uncalibrated source: no usable bearing
            return 0.0, 0.0
        return (
            float(math.atan2(center.x - cx, fx)),
            float(math.atan2(center.y - cy, fy)),
        )

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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
