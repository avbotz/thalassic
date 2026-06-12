from __future__ import annotations

import argparse
from dataclasses import dataclass
from collections import deque
from time import monotonic

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


@dataclass
class ReceivedImage:
    msg: Image
    array: np.ndarray


class DeepSeeColorSmokeTest(Node):
    def __init__(self, rgb_topic: str, depth_topic: str, corrected_topic: str):
        super().__init__("deepseecolor_smoke_test")
        self.bridge = CvBridge()
        self.rgb: ReceivedImage | None = None
        self.depth: ReceivedImage | None = None
        self.corrected: ReceivedImage | None = None
        self.matched_rgb: ReceivedImage | None = None
        self.rgb_history = deque(maxlen=60)

        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(Image, rgb_topic, self._rgb_callback, qos)
        self.create_subscription(Image, depth_topic, self._depth_callback, qos)
        self.create_subscription(Image, corrected_topic, self._corrected_callback, qos)

    def complete(self):
        return self.matched_rgb is not None and self.depth is not None and self.corrected is not None

    def _rgb_callback(self, msg):
        self.rgb = ReceivedImage(msg, self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"))
        self.rgb_history.append(self.rgb)

    def _depth_callback(self, msg):
        self.depth = ReceivedImage(msg, self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"))

    def _corrected_callback(self, msg):
        self.corrected = ReceivedImage(msg, self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"))
        self.matched_rgb = self._find_matching_rgb(msg)

    def _find_matching_rgb(self, corrected_msg: Image):
        corrected_stamp = _stamp_nanoseconds(corrected_msg)
        for image in reversed(self.rgb_history):
            if _stamp_nanoseconds(image.msg) == corrected_stamp:
                return image
        return None


def _stamp_seconds(msg: Image):
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def _stamp_nanoseconds(msg: Image):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def _validate(node: DeepSeeColorSmokeTest):
    assert node.matched_rgb is not None
    assert node.depth is not None
    assert node.corrected is not None

    failures = []
    warnings = []

    rgb = node.matched_rgb.array
    depth = node.depth.array
    corrected = node.corrected.array

    if node.matched_rgb.msg.encoding not in ("rgb8", "bgr8"):
        failures.append(f"raw RGB encoding is {node.matched_rgb.msg.encoding}, expected rgb8/bgr8")
    if node.depth.msg.encoding not in ("32FC1", "16UC1"):
        failures.append(f"depth encoding is {node.depth.msg.encoding}, expected 32FC1 or 16UC1")
    if node.corrected.msg.encoding != "rgb8":
        failures.append(f"corrected encoding is {node.corrected.msg.encoding}, expected rgb8")

    if rgb.shape != corrected.shape:
        failures.append(f"corrected shape {corrected.shape} does not match raw RGB shape {rgb.shape}")
    if rgb.shape[:2] != depth.shape[:2]:
        failures.append(f"depth shape {depth.shape[:2]} does not match RGB shape {rgb.shape[:2]}")

    depth_float = depth.astype(np.float32)
    if node.depth.msg.encoding == "16UC1":
        depth_float = depth_float / 1000.0

    finite_depth = depth_float[np.isfinite(depth_float)]
    valid_depth = finite_depth[finite_depth > 0.0]
    if valid_depth.size == 0:
        failures.append("depth image has no finite positive depth pixels")
    else:
        print(
            "depth: "
            f"encoding={node.depth.msg.encoding}, "
            f"valid_px={valid_depth.size}, "
            f"min={float(np.min(valid_depth)):.3f}m, "
            f"median={float(np.median(valid_depth)):.3f}m, "
            f"max={float(np.max(valid_depth)):.3f}m"
        )

    mean_abs_delta = float(np.mean(np.abs(corrected.astype(np.int16) - rgb.astype(np.int16))))
    if mean_abs_delta < 0.1:
        warnings.append(
            "corrected image is nearly identical to raw RGB; this can happen on the first frames "
            "or if the node is configured with zero training iterations"
        )

    stamp_delta = abs(_stamp_seconds(node.matched_rgb.msg) - _stamp_seconds(node.corrected.msg))
    if stamp_delta != 0.0:
        failures.append(f"corrected header stamp differs from matched RGB by {stamp_delta:.9f}s")

    print(f"rgb: shape={rgb.shape}, encoding={node.matched_rgb.msg.encoding}")
    print(f"corrected: shape={corrected.shape}, mean_abs_delta={mean_abs_delta:.3f}")

    for warning in warnings:
        print(f"WARNING: {warning}")

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}")
        return 1

    print("PASS: DeepSeeColor integration is publishing synchronized corrected RGB from RGB-D input.")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="Smoke test the DeepSeeColor ROS integration.")
    parser.add_argument("--rgb-topic", default="/marlin_v2/front_camera/image_color")
    parser.add_argument("--depth-topic", default="/marlin_v2/depth_camera/image_depth")
    parser.add_argument("--corrected-topic", default="/marlin_v2/front_camera/image_color_corrected")
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = DeepSeeColorSmokeTest(args.rgb_topic, args.depth_topic, args.corrected_topic)
    deadline = monotonic() + args.timeout

    try:
        while rclpy.ok() and monotonic() < deadline and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.1)

        if not node.complete():
            missing = []
            if node.rgb is None:
                missing.append(args.rgb_topic)
            if node.depth is None:
                missing.append(args.depth_topic)
            if node.corrected is None:
                missing.append(args.corrected_topic)
            if node.rgb is not None and node.corrected is not None and node.matched_rgb is None:
                missing.append("timestamp-matched raw RGB for corrected frame")
            print(f"FAIL: timed out waiting for: {', '.join(missing)}")
            return 1

        return _validate(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
