from __future__ import annotations

import argparse
from pathlib import Path
from time import monotonic

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class DepthSnapshot(Node):
    def __init__(self, topic: str):
        super().__init__("depth_snapshot")
        self.bridge = CvBridge()
        self.depth = None
        self.encoding = ""

        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Image, topic, self._callback, qos)

    def _callback(self, msg: Image):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth = depth.astype(np.float32)
        if msg.encoding == "16UC1":
            depth = depth / 1000.0
        self.depth = depth
        self.encoding = msg.encoding


def _normalize_depth(depth, min_depth, max_depth):
    valid = np.isfinite(depth) & (depth > 0.0)
    if not np.any(valid):
        raise RuntimeError("depth frame has no finite positive values")

    if min_depth is None:
        min_depth = float(np.percentile(depth[valid], 1.0))
    if max_depth is None:
        max_depth = float(np.percentile(depth[valid], 99.0))
    if max_depth <= min_depth:
        max_depth = min_depth + 1.0

    clipped = np.clip(depth, min_depth, max_depth)
    normalized = (clipped - min_depth) / (max_depth - min_depth)
    normalized[~valid] = 0.0
    gray = np.clip(normalized * 255.0, 0, 255).astype(np.uint8)
    color = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    color[~valid] = (0, 0, 0)
    return color, min_depth, max_depth, valid


def main(argv=None):
    parser = argparse.ArgumentParser(description="Save a normalized depth image PNG.")
    parser.add_argument("--topic", default="/marlin_v3/oak/stereo/image_raw")
    parser.add_argument("--save", default="debug_outputs/depth_snapshot.png")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--min-depth", type=float, default=None)
    parser.add_argument("--max-depth", type=float, default=None)
    args = parser.parse_args(argv)

    rclpy.init()
    node = DepthSnapshot(args.topic)
    deadline = monotonic() + args.timeout

    try:
        while rclpy.ok() and monotonic() < deadline and node.depth is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.depth is None:
            print(f"FAIL: timed out waiting for {args.topic}")
            return 1

        color, min_depth, max_depth, valid = _normalize_depth(
            node.depth,
            args.min_depth,
            args.max_depth,
        )
        output = Path(args.save)
        output.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output), color)

        valid_depth = node.depth[valid]
        print(
            f"saved {output} from {args.topic}: "
            f"encoding={node.encoding}, "
            f"valid_px={valid_depth.size}, "
            f"min={float(np.min(valid_depth)):.3f}m, "
            f"median={float(np.median(valid_depth)):.3f}m, "
            f"max={float(np.max(valid_depth)):.3f}m, "
            f"visualized_range={min_depth:.3f}-{max_depth:.3f}m"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
