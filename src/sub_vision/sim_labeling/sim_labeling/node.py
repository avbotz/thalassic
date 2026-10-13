"""
ROS 2 node for automatic image labeling using Stonefish's segmentation camera.

Subscribes to:
  - Front color camera image (RGB)
  - Segmentation camera image (16UC1, one pixel value per Stonefish object)
  - Segmentation labels (vision_msgs/LabelInfo, latched): the name of the
    object drawn with each pixel value

The segmentation camera is paired with the front camera in the scenario, so it
is rendered in the same frames, from the same viewpoint, and both images of a
frame carry the same stamp. Each pair is matched exactly by stamp. Pixel-perfect
segmentation polygons are extracted for the objects whose names match a prop
class, small detections are discarded, and the color image is saved with its
YOLOv11 segmentation annotation.
"""

import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import Subscriber, TimeSynchronizer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from vision_msgs.msg import LabelInfo

from sim_labeling.classes import build_pixel_to_class_id, get_class_names
from sim_labeling.save import save_dataset_yaml, save_labeled_image


class LabelNode(Node):
    """Node that pairs segmentation frames with color frames for auto-labeling."""

    def __init__(self):
        super().__init__("label_node")

        # ── ROS parameters ──────────────────────────────────────────────
        self.declare_parameter("output_dir", "train_imgs")
        self.declare_parameter("min_bbox_area", 400)
        # Relative to the vehicle namespace. The segmentation camera is paired
        # with the scenario's front camera.
        self.declare_parameter("seg_topic", "sim/segment/image_raw")
        self.declare_parameter("labels_topic", "sim/segment/label_info")
        self.declare_parameter("front_cam_topic", "front_camera/image_color")

        self.output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self.min_bbox_area = self.get_parameter("min_bbox_area").get_parameter_value().integer_value
        seg_topic = self.get_parameter("seg_topic").get_parameter_value().string_value
        labels_topic = self.get_parameter("labels_topic").get_parameter_value().string_value
        front_cam_topic = self.get_parameter("front_cam_topic").get_parameter_value().string_value

        self.class_names: list[str] = get_class_names()
        save_dataset_yaml(self.class_names, self.output_dir)

        # ── State ───────────────────────────────────────────────────────
        self.cvb = CvBridge()
        self.pixel_to_class: dict[int, int] = {}
        # Indexed by pixel value: whether the labels name it. None until the
        # labels arrive.
        self.known_pixels: np.ndarray | None = None
        self.frame_count = 0
        self.saved_count = 0

        # ── Subscriptions ───────────────────────────────────────────────
        # Latched by the simulator, and republished when objects are added.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.labels_sub = self.create_subscription(
            LabelInfo, labels_topic, self.labels_callback, latched
        )
        # Exact: frames the paired cameras render together have equal stamps.
        self.front_sub = Subscriber(self, Image, front_cam_topic, qos_profile=10)
        self.seg_sub = Subscriber(self, Image, seg_topic, qos_profile=10)
        self.sync = TimeSynchronizer([self.front_sub, self.seg_sub], queue_size=10)
        self.sync.registerCallback(self.frames_callback)

        self.get_logger().info(
            f"Label node started. "
            f"Seg topic: {seg_topic}, "
            f"Labels topic: {labels_topic}, "
            f"Front cam topic: {front_cam_topic}, "
            f"Output dir: {self.output_dir}"
        )

    # ── Callbacks ───────────────────────────────────────────────────────

    def labels_callback(self, msg: LabelInfo):
        """Map the simulator's object names to classes."""
        object_names = {c.class_id: c.class_name for c in msg.class_map}
        self.pixel_to_class = build_pixel_to_class_id(object_names)

        known = np.zeros(np.iinfo(np.uint16).max + 1, dtype=bool)
        known[0] = True  # background
        known[list(object_names)] = True
        self.known_pixels = known

        self.get_logger().info(
            f"Loaded {len(object_names)} segmentation labels, "
            f"{len(self.pixel_to_class)} of them annotated."
        )
        for class_id, class_name in enumerate(self.class_names):
            objects = [
                object_names[px] for px, cid in self.pixel_to_class.items() if cid == class_id
            ]
            if objects:
                self.get_logger().info(f'  Class "{class_name}" ({class_id}): {objects}')

    def frames_callback(self, front_msg: Image, seg_msg: Image):
        """Handle a front camera image and the segmentation of the same frame."""
        if self.known_pixels is None:
            self.get_logger().warn(
                "No segmentation labels received yet. Skipping.",
                throttle_duration_sec=5.0,
            )
            return

        self.frame_count += 1

        # Pixel values are object IDs, so any other encoding (such as the rgb8
        # display image on image_color) cannot be read as one.
        if seg_msg.encoding != "16UC1":
            self.get_logger().error(
                f"{self.seg_sub.sub.topic_name} is {seg_msg.encoding}, not the segmentation "
                "camera's 16UC1 object IDs. Skipping.",
                throttle_duration_sec=5.0,
            )
            return
        seg_img = self.cvb.imgmsg_to_cv2(seg_msg, desired_encoding="passthrough")

        # Objects added after the labels were published would be missing from
        # the annotations, so the frame is left for the updated labels.
        if not self.known_pixels[seg_img].all():
            self.get_logger().warn(
                "Segmentation image has objects missing from the labels. Skipping.",
                throttle_duration_sec=5.0,
            )
            return

        try:
            saved = save_labeled_image(
                front_cam_img=self.cvb.imgmsg_to_cv2(front_msg, desired_encoding="rgb8"),
                seg_img=seg_img,
                pixel_to_class=self.pixel_to_class,
                output_dir=self.output_dir,
                min_contour_area_px=int(self.min_bbox_area),
            )
        except OSError as exc:
            self.get_logger().error(
                f"Could not save a labeled image: {exc}", throttle_duration_sec=5.0
            )
            return

        if saved:
            self.saved_count += 1
            if self.saved_count % 50 == 0:
                self.get_logger().info(
                    f"Saved {self.saved_count} labeled images "
                    f"({self.frame_count} frames processed)."
                )


def main(args=None):
    rclpy.init(args=args)
    node = LabelNode()
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
