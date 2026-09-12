"""Gate task post-processing.

The gate model is trained as a segmentation model, but the ROS detection
message currently carries only boxes/classes. This processor therefore uses
the gate-class box as the stable runtime contract and adds gate-specific
metadata: range from known gate width, role-side aim pixels, and red/black
post layout estimated from the RGB crop.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from diagnostic_msgs.msg import KeyValue

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor

GATE_CLASS_ID = 1
GATE_WIDTH_M = 1.5
_AIM_Y_FRACTION = 0.68
_POST_STRIP_FRACTION = 0.22


@dataclass(frozen=True)
class GateColorLayout:
    left_upper: str
    left_lower: str
    right_upper: str
    right_lower: str

    @property
    def red_right_above(self) -> bool:
        return self.left_lower == "red" and self.right_upper == "red"


def estimate_gate_distance_m(bbox_width_px: float, camera_k: np.ndarray) -> float:
    """Estimate camera-to-gate range from the known physical gate width."""
    fx = float(camera_k[0, 0])
    if bbox_width_px <= 0.0 or fx <= 0.0:
        return float("nan")
    return float((GATE_WIDTH_M * fx) / bbox_width_px)


def gate_aim_pixels(
    bbox: tuple[float, float, float, float],
) -> dict[str, tuple[float, float]]:
    """Return left/center/right aim pixels inside a gate bbox.

    The side aims split the opening into quarters. The vertical aim point is
    slightly below center to bias toward the open swim-through rather than the
    top crossbar in box-only detections.
    """
    x0, y0, x1, y1 = bbox
    width = x1 - x0
    height = y1 - y0
    y = y0 + height * _AIM_Y_FRACTION
    return {
        "left": (x0 + width * 0.25, y),
        "center": (x0 + width * 0.50, y),
        "right": (x0 + width * 0.75, y),
    }


def estimate_gate_color_layout(
    image_bgr: np.ndarray, bbox: tuple[float, float, float, float]
) -> GateColorLayout:
    """Estimate the red/black pattern on the left and right gate posts."""
    image_h, image_w = image_bgr.shape[:2]
    x0, y0, x1, y1 = _clip_bbox(bbox, image_w, image_h)
    width = x1 - x0
    height = y1 - y0
    if width <= 1 or height <= 1:
        return GateColorLayout("unknown", "unknown", "unknown", "unknown")

    strip_w = max(1, int(round(width * _POST_STRIP_FRACTION)))
    mid_y = y0 + height // 2
    left_x1 = min(x1, x0 + strip_w)
    right_x0 = max(x0, x1 - strip_w)

    return GateColorLayout(
        left_upper=_classify_patch(image_bgr[y0:mid_y, x0:left_x1]),
        left_lower=_classify_patch(image_bgr[mid_y:y1, x0:left_x1]),
        right_upper=_classify_patch(image_bgr[y0:mid_y, right_x0:x1]),
        right_lower=_classify_patch(image_bgr[mid_y:y1, right_x0:x1]),
    )


def _classify_patch(patch_bgr: np.ndarray) -> str:
    if patch_bgr.size == 0:
        return "unknown"
    mean_b, mean_g, mean_r = patch_bgr.reshape(-1, 3).mean(axis=0)
    brightness = (float(mean_b) + float(mean_g) + float(mean_r)) / 3.0
    if brightness < 70.0:
        return "black"
    if mean_r > 90.0 and mean_r > mean_g * 1.35 and mean_r > mean_b * 1.35:
        return "red"
    return "unknown"


def _clip_bbox(
    bbox: tuple[float, float, float, float], image_w: int, image_h: int
) -> tuple[int, int, int, int]:
    x0, y0, x1, y1 = bbox
    return (
        max(0, min(image_w, int(round(x0)))),
        max(0, min(image_h, int(round(y0)))),
        max(0, min(image_w, int(round(x1)))),
        max(0, min(image_h, int(round(y1)))),
    )


def _class_id(det) -> int:
    if not det.detection.results:
        return -1
    try:
        return int(det.detection.results[0].hypothesis.class_id)
    except ValueError:
        return -1


def _score(det) -> float:
    if not det.detection.results:
        return 0.0
    return float(det.detection.results[0].hypothesis.score)


def _bbox_xyxy(det) -> tuple[float, float, float, float]:
    bbox = det.detection.bbox
    half_w = bbox.size_x / 2.0
    half_h = bbox.size_y / 2.0
    cx = bbox.center.position.x
    cy = bbox.center.position.y
    return (cx - half_w, cy - half_h, cx + half_w, cy + half_h)


def _deproject_pixel(u: float, v: float, depth_m: float, camera_k: np.ndarray) -> np.ndarray:
    fx, fy = camera_k[0, 0], camera_k[1, 1]
    cx, cy = camera_k[0, 2], camera_k[1, 2]
    return np.array(
        [
            (u - cx) * depth_m / fx,
            (v - cy) * depth_m / fy,
            depth_m,
        ],
        dtype=np.float64,
    )


@register_post_processor("gate")
class GatePostProcessor(TaskPostProcessor):
    """Keep gate detections and add aim/range/layout metadata."""

    def process(self, detections, rgb_image, depth_image, camera_info):
        k = np.array(camera_info.k, dtype=np.float64).reshape(3, 3)
        gates = [det for det in detections.detections if _class_id(det) == GATE_CLASS_ID]
        gates.sort(key=_score, reverse=True)
        detections.detections = gates

        for det in detections.detections:
            bbox = det.detection.bbox
            xyxy = _bbox_xyxy(det)
            distance_m = estimate_gate_distance_m(float(bbox.size_x), k)
            det.distance_m = distance_m

            aims = gate_aim_pixels(xyxy)
            layout = estimate_gate_color_layout(rgb_image, xyxy)
            selected_side = "right" if layout.red_right_above else "left"
            selected_u, selected_v = aims[selected_side]

            det.extra.append(KeyValue(key="aim_left_px", value=_format_pixel(aims["left"])))
            det.extra.append(KeyValue(key="aim_center_px", value=_format_pixel(aims["center"])))
            det.extra.append(KeyValue(key="aim_right_px", value=_format_pixel(aims["right"])))
            det.extra.append(KeyValue(key="selected_aim", value=selected_side))
            det.extra.append(
                KeyValue(key="red_right_above", value=str(layout.red_right_above).lower())
            )
            det.extra.append(
                KeyValue(
                    key="color_layout",
                    value=(
                        f"LU={layout.left_upper},LL={layout.left_lower},"
                        f"RU={layout.right_upper},RL={layout.right_lower}"
                    ),
                )
            )
            det.extra.append(KeyValue(key="pose_semantics", value="aim_point"))

            if not np.isfinite(distance_m) or k[0, 0] <= 0.0 or k[1, 1] <= 0.0:
                det.pose_valid = False
                continue

            point = _deproject_pixel(selected_u, selected_v, distance_m, k)
            det.pose.position.x = float(point[0])
            det.pose.position.y = float(point[1])
            det.pose.position.z = float(point[2])
            det.pose.orientation.w = 1.0
            det.pose_valid = True

            bearing_h = math.atan2(selected_u - k[0, 2], k[0, 0])
            bearing_v = math.atan2(selected_v - k[1, 2], k[1, 1])
            det.extra.append(KeyValue(key="aim_bearing_horizontal", value=f"{bearing_h:.6f}"))
            det.extra.append(KeyValue(key="aim_bearing_vertical", value=f"{bearing_v:.6f}"))
            det.extra.append(
                KeyValue(
                    key="aim_point_m",
                    value=f"{point[0]:.4f},{point[1]:.4f},{point[2]:.4f}",
                )
            )

        return detections


def _format_pixel(pixel: tuple[float, float]) -> str:
    return f"{pixel[0]:.1f},{pixel[1]:.1f}"
