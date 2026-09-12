"""
ROS 2 node for automatic image labeling using Stonefish's segmentation camera.

Subscribes to:
  - Segmentation camera image (16UC1, pixel value = objectId + 1)
  - Front color camera image (RGB)

Parses the Stonefish scenario XML to determine which segmentation IDs
correspond to which props.  Extracts pixel-perfect segmentation polygons
(merging multiple OBJ-mesh IDs that belong to the same prop), discards
small detections, and saves the color image + YOLOv11 segmentation
annotation.
"""

from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from sim_labeling.parse_ids import (
    build_pixel_to_class_id,
    get_class_names,
    parse_scenario_ids,
)
from sim_labeling.save import save_dataset_yaml, save_labeled_image


class LabelNode(Node):
    """Node that pairs segmentation frames with color frames for auto-labeling."""

    def __init__(self):
        super().__init__("label_node")

        # ── ROS parameters ──────────────────────────────────────────────
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("output_dir", "train_imgs")
        self.declare_parameter("min_bbox_area", 400)
        self.declare_parameter("seg_topic", "/marlin_v3/sim/segment/image_raw")
        self.declare_parameter("front_cam_topic", "/marlin_v3/oak/rgb/image_raw")

        scenario_file = self.get_parameter("scenario_file").get_parameter_value().string_value
        self.output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self.min_bbox_area = self.get_parameter("min_bbox_area").get_parameter_value().integer_value
        seg_topic = self.get_parameter("seg_topic").get_parameter_value().string_value
        front_cam_topic = self.get_parameter("front_cam_topic").get_parameter_value().string_value

        # save_dataset_yaml writes straight into this directory, so it has to
        # exist before the first frame arrives.
        if self.output_dir:
            Path(self.output_dir).mkdir(parents=True, exist_ok=True)

        # ── Build pixel → class_id mapping from scenario ────────────────
        self.pixel_to_class: dict[int, int] = {}
        self.class_names: list[str] = get_class_names()

        if scenario_file:
            self._load_scenario(scenario_file)
        else:
            self.get_logger().warn(
                "No scenario_file parameter set. "
                "Will attempt to find it from sub_sim share directory."
            )
            self._try_auto_detect_scenario()

        if self.pixel_to_class:
            self.get_logger().info(
                f"Loaded {len(self.pixel_to_class)} segmentation pixel "
                f"mappings across {len(self.class_names)} classes."
            )
            # Write dataset.yaml once
            save_dataset_yaml(self.class_names, self.output_dir)
        else:
            self.get_logger().error(
                "No pixel-to-class mapping could be built! Labeling will be non-functional."
            )

        # ── State ───────────────────────────────────────────────────────
        self.cvb = CvBridge()
        self.latest_front_image: np.ndarray | None = None
        self.frame_count = 0
        self.saved_count = 0

        # ── Subscriptions ───────────────────────────────────────────────
        self.seg_sub = self.create_subscription(Image, seg_topic, self.seg_callback, 10)
        self.front_sub = self.create_subscription(Image, front_cam_topic, self.front_callback, 10)

        self.get_logger().info(
            f"Label node started. "
            f"Seg topic: {seg_topic}, "
            f"Front cam topic: {front_cam_topic}, "
            f"Output dir: {self.output_dir}"
        )

    # ── Scenario loading ────────────────────────────────────────────────

    def _load_scenario(self, scenario_file: str):
        """Load scenario and build pixel→class mapping."""
        try:
            # The data_dir is the parent of "scenarios/"
            scenario_path = Path(scenario_file)

            # Try to find data_dir from the scenario path or sub_sim share
            data_dir = self._find_data_dir(scenario_path)

            if data_dir is None:
                self.get_logger().error(
                    f"Could not determine data_dir for scenario {scenario_file}"
                )
                return

            self.get_logger().info(f"Parsing scenario: {scenario_file} with data_dir: {data_dir}")

            self.pixel_to_class, self.class_names = build_pixel_to_class_id(scenario_file, data_dir)

            # Log the mapping for debugging
            _pixel_to_prop, prop_groups, _all_objs = parse_scenario_ids(scenario_file, data_dir)
            for g in prop_groups:
                self.get_logger().info(
                    f'  Prop "{g.class_name}" (class {g.class_id}): {len(g.seg_ids)} seg IDs'
                )

        except Exception as e:
            self.get_logger().error(f"Failed to parse scenario: {e}")
            import traceback

            self.get_logger().error(traceback.format_exc())

    def _find_data_dir(self, scenario_path: Path) -> Path | None:
        """Try to find the Stonefish data directory."""
        # Check if scenario is under a 'scenarios' folder
        for parent in scenario_path.parents:
            if (parent / "models").is_dir():
                return parent
            if parent.name == "data" and (parent / "models").is_dir():
                return parent

        # Fall back to sub_sim share directory
        try:
            sub_sim_share = get_package_share_directory("sub_sim")
            data_dir = Path(sub_sim_share) / "data"
            if data_dir.is_dir():
                return data_dir
        except Exception:
            pass

        return None

    def _try_auto_detect_scenario(self):
        """Try to find the scenario file automatically."""
        try:
            sub_sim_share = get_package_share_directory("sub_sim")
            data_dir = Path(sub_sim_share) / "data"
            scenario_dir = Path(sub_sim_share) / "scenarios"

            # Look for rendered .scn files in /tmp
            import glob

            scn_files = sorted(
                glob.glob("/tmp/woollett*.scn"),
                key=lambda f: Path(f).stat().st_mtime,
                reverse=True,
            )
            if scn_files:
                self.get_logger().info(f"Auto-detected scenario file: {scn_files[0]}")
                self._load_scenario_with_data_dir(scn_files[0], str(data_dir))
                return

            # Fall back to template (won't have resolved Jinja vars,
            # but structure is the same for ID assignment)
            template = scenario_dir / "woollett.scn.j2"
            if template.exists():
                self.get_logger().warn(
                    "Using scenario template (IDs may differ if randomization "
                    "changes include order, but include order is fixed)."
                )
                self._load_scenario_with_data_dir(str(template), str(data_dir))

        except Exception as e:
            self.get_logger().error(f"Auto-detect failed: {e}")

    def _load_scenario_with_data_dir(self, scenario_file: str, data_dir: str):
        """Load scenario with explicit data_dir."""
        try:
            self.pixel_to_class, self.class_names = build_pixel_to_class_id(scenario_file, data_dir)
        except Exception as e:
            self.get_logger().error(f"Failed to parse scenario: {e}")

    # ── Callbacks ───────────────────────────────────────────────────────

    def seg_callback(self, msg: Image):
        """Handle segmentation camera image."""
        if self.latest_front_image is None:
            self.get_logger().warn(
                "No front image available for segmentation.",
                throttle_duration_sec=5.0,
            )
            return

        if not self.pixel_to_class:
            self.get_logger().warn(
                "No pixel-to-class mapping loaded. Skipping.",
                throttle_duration_sec=10.0,
            )
            return

        self.frame_count += 1

        # Convert segmentation image: 16UC1 → numpy uint16
        seg_img = self.cvb.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if seg_img.dtype != np.uint16:
            seg_img = seg_img.astype(np.uint16)

        saved = save_labeled_image(
            front_cam_img=self.latest_front_image,
            seg_img=seg_img,
            pixel_to_class=self.pixel_to_class,
            output_dir=self.output_dir,
            class_names=self.class_names,
            min_contour_area_px=int(self.min_bbox_area),
        )

        if saved:
            self.saved_count += 1
            if self.saved_count % 50 == 0:
                self.get_logger().info(
                    f"Saved {self.saved_count} labeled images "
                    f"({self.frame_count} frames processed)."
                )

    def front_callback(self, msg: Image):
        """Handle front camera color image."""
        self.latest_front_image = self.cvb.imgmsg_to_cv2(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LabelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
