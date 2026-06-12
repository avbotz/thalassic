from __future__ import annotations

from time import perf_counter

import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

try:
    import torch
except ImportError:  # pragma: no cover - handled during node startup.
    torch = None

from sub_color_correction.deepseecolor_model import DeepSeeColorProcessor


class DeepSeeColorNode(Node):
    def __init__(self):
        super().__init__("deepseecolor")

        self.declare_parameter("rgb_topic", "/marlin_v2/oak/rgb/image_raw")
        self.declare_parameter("depth_topic", "/marlin_v2/oak/stereo/image_raw")
        self.declare_parameter("corrected_topic", "/marlin_v2/oak/rgb/image_color_corrected")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("init_iters", 10)
        self.declare_parameter("iters", 2)
        self.declare_parameter("learning_rate", 1e-2)
        self.declare_parameter("depth_quantile", 0.0001)
        self.declare_parameter("mask_max_depth", False)
        self.declare_parameter("max_inference_dimension", 640)
        self.declare_parameter("queue_size", 5)
        self.declare_parameter("sync_slop", 0.15)
        self.declare_parameter("publish_every_n", 1)

        if torch is None:
            raise RuntimeError("DeepSeeColor requires torch. Install PyTorch before running this node.")

        device = self.get_parameter("device").value
        if str(device).startswith("cuda") and not torch.cuda.is_available():
            self.get_logger().warn("CUDA requested but unavailable; falling back to CPU.")
            device = "cpu"

        self.bridge = CvBridge()
        self.max_inference_dimension = int(self.get_parameter("max_inference_dimension").value)
        self.publish_every_n = max(1, int(self.get_parameter("publish_every_n").value))
        self.frame_count = 0

        self.processor = DeepSeeColorProcessor(
            device=device,
            init_iters=int(self.get_parameter("init_iters").value),
            iters=int(self.get_parameter("iters").value),
            learning_rate=float(self.get_parameter("learning_rate").value),
            depth_quantile=float(self.get_parameter("depth_quantile").value),
            mask_max_depth=bool(self.get_parameter("mask_max_depth").value),
        )

        qos = QoSProfile(depth=int(self.get_parameter("queue_size").value))
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        rgb_topic = self.get_parameter("rgb_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        corrected_topic = self.get_parameter("corrected_topic").value

        self.rgb_sub = Subscriber(self, Image, rgb_topic, qos_profile=qos)
        self.depth_sub = Subscriber(self, Image, depth_topic, qos_profile=qos)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=int(self.get_parameter("queue_size").value),
            slop=float(self.get_parameter("sync_slop").value),
        )
        self.sync.registerCallback(self.image_callback)
        self.publisher = self.create_publisher(Image, corrected_topic, qos)

        self.get_logger().info(
            "DeepSeeColor node started: "
            f"rgb={rgb_topic}, depth={depth_topic}, corrected={corrected_topic}, device={device}"
        )

    def image_callback(self, rgb_msg: Image, depth_msg: Image):
        self.frame_count += 1
        if self.frame_count % self.publish_every_n != 0:
            return

        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            self.get_logger().warn(f"Failed to convert image messages: {exc}")
            return

        if depth_msg.encoding == "16UC1":
            depth = depth.astype(np.float32) / 1000.0
        else:
            depth = depth.astype(np.float32)

        if rgb.shape[:2] != depth.shape[:2]:
            self.get_logger().warn(
                f"RGB/depth shape mismatch: rgb={rgb.shape[:2]}, depth={depth.shape[:2]}",
                throttle_duration_sec=2.0,
            )
            return

        start = perf_counter()
        rgb_tensor = self._rgb_to_tensor(rgb)
        depth_tensor = self._depth_to_tensor(depth)

        try:
            corrected_tensor, stats = self.processor.correct(
                rgb_tensor,
                depth_tensor,
                max_dimension=self.max_inference_dimension,
            )
        except RuntimeError as exc:
            self.get_logger().error(f"DeepSeeColor processing failed: {exc}")
            return

        corrected = self._tensor_to_rgb(corrected_tensor)
        out_msg = self.bridge.cv2_to_imgmsg(corrected, encoding="rgb8")
        out_msg.header = rgb_msg.header
        self.publisher.publish(out_msg)

        elapsed_ms = (perf_counter() - start) * 1000.0
        self.get_logger().info(
            "Published corrected frame "
            f"{self.frame_count}: {elapsed_ms:.1f} ms, "
            f"iters={stats.trained_iterations}, "
            f"bs_loss={stats.backscatter_loss:.5f}, "
            f"da_loss={stats.deattenuation_loss:.5f}",
            throttle_duration_sec=2.0,
        )

    def _rgb_to_tensor(self, rgb):
        rgb = np.ascontiguousarray(rgb.astype(np.float32) / 255.0)
        tensor = torch.from_numpy(rgb).permute(2, 0, 1).unsqueeze(0)
        return tensor.to(self.processor.device)

    def _depth_to_tensor(self, depth):
        depth = np.ascontiguousarray(depth.astype(np.float32))
        tensor = torch.from_numpy(depth).unsqueeze(0).unsqueeze(0)
        return tensor.to(self.processor.device)

    def _tensor_to_rgb(self, tensor):
        image = tensor.squeeze(0).permute(1, 2, 0).detach().cpu().numpy()
        return np.ascontiguousarray(np.clip(image * 255.0, 0, 255).astype(np.uint8))


def main(args=None):
    rclpy.init(args=args)
    node = DeepSeeColorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
