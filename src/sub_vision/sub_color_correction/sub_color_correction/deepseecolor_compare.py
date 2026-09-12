from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path
from time import monotonic

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class DeepSeeColorCompare(Node):
    def __init__(
        self, rgb_topic: str, corrected_topic: str, max_width: int, crop_top: int, crop_bottom: int
    ):
        super().__init__("deepseecolor_compare")
        self.bridge = CvBridge()
        self.max_width = max_width
        self.crop_top = max(0, crop_top)
        self.crop_bottom = max(0, crop_bottom)
        self.rgb_history = deque(maxlen=60)
        self.latest_pair = None

        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(Image, rgb_topic, self._rgb_callback, qos)
        self.create_subscription(Image, corrected_topic, self._corrected_callback, qos)

    def _rgb_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        self.rgb_history.append((_stamp_nanoseconds(msg), image))

    def _corrected_callback(self, msg):
        corrected = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        stamp = _stamp_nanoseconds(msg)
        for rgb_stamp, rgb in reversed(self.rgb_history):
            if rgb_stamp == stamp:
                self.latest_pair = self._make_comparison(rgb, corrected)
                return

    def _make_comparison(self, rgb, corrected):
        rgb, corrected = self._crop_pair(rgb, corrected)
        rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        corrected_bgr = cv2.cvtColor(corrected, cv2.COLOR_RGB2BGR)
        if rgb_bgr.shape != corrected_bgr.shape:
            corrected_bgr = cv2.resize(corrected_bgr, (rgb_bgr.shape[1], rgb_bgr.shape[0]))

        comparison = np.hstack((rgb_bgr, corrected_bgr))
        label_h = max(32, comparison.shape[0] // 18)
        font_scale = max(0.6, comparison.shape[0] / 900.0)
        thickness = max(1, int(round(comparison.shape[0] / 500.0)))
        cv2.rectangle(comparison, (0, 0), (comparison.shape[1], label_h), (0, 0, 0), -1)
        cv2.putText(
            comparison,
            "raw RGB",
            (16, int(label_h * 0.7)),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (255, 255, 255),
            thickness,
        )
        cv2.putText(
            comparison,
            "DeepSeeColor",
            (rgb_bgr.shape[1] + 16, int(label_h * 0.7)),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (255, 255, 255),
            thickness,
        )

        if self.max_width > 0 and comparison.shape[1] > self.max_width:
            scale = self.max_width / float(comparison.shape[1])
            comparison = cv2.resize(
                comparison,
                (self.max_width, max(1, int(round(comparison.shape[0] * scale)))),
                interpolation=cv2.INTER_AREA,
            )
        return comparison

    def _crop_pair(self, rgb, corrected):
        height = min(rgb.shape[0], corrected.shape[0])
        top = min(self.crop_top, height - 1)
        bottom = max(top + 1, height - self.crop_bottom)
        return rgb[top:bottom, :, :], corrected[top:bottom, :, :]


def _stamp_nanoseconds(msg: Image):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="View or save raw RGB vs DeepSeeColor corrected images."
    )
    parser.add_argument("--rgb-topic", default="/marlin_v3/oak/rgb/image_raw")
    parser.add_argument("--corrected-topic", default="/marlin_v3/oak/rgb/image_color_corrected")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--max-width", type=int, default=1600)
    parser.add_argument("--crop-top", type=int, default=0)
    parser.add_argument("--crop-bottom", type=int, default=0)
    parser.add_argument("--save", type=str, default="")
    parser.add_argument("--no-window", action="store_true")
    args = parser.parse_args(argv)

    rclpy.init()
    node = DeepSeeColorCompare(
        args.rgb_topic,
        args.corrected_topic,
        args.max_width,
        args.crop_top,
        args.crop_bottom,
    )
    deadline = monotonic() + args.timeout

    try:
        while rclpy.ok() and monotonic() < deadline and node.latest_pair is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.latest_pair is None:
            print("FAIL: timed out waiting for timestamp-matched raw/corrected frames")
            return 1

        if args.save:
            output = Path(args.save)
            output.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(output), node.latest_pair)
            print(f"saved {output}")

        if not args.no_window:
            cv2.imshow("DeepSeeColor comparison", node.latest_pair)
            print("press any key in the image window to close")
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
