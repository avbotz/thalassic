"""Slalom: the nearest red divider pole, for yaw-locked navigation.

``slalom_redpoles_osu`` (class 0 = red pole) occasionally boxes a white pole
as red, so a box is only kept when its crop also looks red. ``slalom``
(0 = red pole, 1 = white pole) goes through the same filter.
"""

from __future__ import annotations

import copy
import math

import numpy as np
from diagnostic_msgs.msg import KeyValue

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor

RED_POLE_CLASS_ID = 0
# Class of the published target, distinct from the model's own classes
TARGET_CLASS_ID = "2"
MIN_POLE_SCORE = 0.10
MIN_POLE_HEIGHT_PX = 8.0
POLE_NMS_IOU = 0.70
MIN_RED_PIXEL_FRACTION = 0.05
# Height of the simulator's poles
POLE_HEIGHT_M = 0.90


def _class_id(det) -> int:
    if not det.detection.results:
        return -1
    try:
        return int(det.detection.results[0].hypothesis.class_id)
    except ValueError:
        return -1


def _score(det) -> float:
    return float(det.detection.results[0].hypothesis.score) if det.detection.results else 0.0


def _bbox_xyxy(det) -> tuple[float, float, float, float]:
    box = det.detection.bbox
    return (
        box.center.position.x - box.size_x / 2.0,
        box.center.position.y - box.size_y / 2.0,
        box.center.position.x + box.size_x / 2.0,
        box.center.position.y + box.size_y / 2.0,
    )


def _has_red_appearance(det, image_bgr: np.ndarray) -> bool:
    """Whether at least MIN_RED_PIXEL_FRACTION of the box is clearly red."""
    if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] < 3:
        return False

    image_h, image_w = image_bgr.shape[:2]
    x1, y1, x2, y2 = _bbox_xyxy(det)
    x1 = max(0, min(image_w, math.floor(x1)))
    y1 = max(0, min(image_h, math.floor(y1)))
    x2 = max(0, min(image_w, math.ceil(x2)))
    y2 = max(0, min(image_h, math.ceil(y2)))
    if x2 <= x1 or y2 <= y1:
        return False

    # Inset horizontally so the water either side of a narrow pole does not dominate
    inset = int((x2 - x1) * 0.20)
    crop = image_bgr[y1:y2, x1 + inset : x2 - inset]
    if crop.size == 0:
        return False

    blue, green, red = np.moveaxis(crop[..., :3].astype(np.float32), -1, 0)
    red_pixels = (
        (red >= 25.0)
        & (red >= 1.08 * blue)
        & (red >= 1.08 * green)
        & (red - np.maximum(blue, green) >= 8.0)
    )
    return float(np.mean(red_pixels)) >= MIN_RED_PIXEL_FRACTION


def _iou(first, second) -> float:
    ax1, ay1, ax2, ay2 = _bbox_xyxy(first)
    bx1, by1, bx2, by2 = _bbox_xyxy(second)
    overlap = max(0.0, min(ax2, bx2) - max(ax1, bx1)) * max(0.0, min(ay2, by2) - max(ay1, by1))
    union = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - overlap
    return overlap / union if union > 0.0 else 0.0


def _deduplicate(poles) -> list:
    unique = []
    for pole in sorted(poles, key=_score, reverse=True):
        if not any(_iou(pole, kept) >= POLE_NMS_IOU for kept in unique):
            unique.append(pole)
    return unique


@register_post_processor("slalom_redpoles_osu")
@register_post_processor("slalom")
class SlalomPostProcessor(TaskPostProcessor):
    """Publish only the nearest (tallest) red pole, ranged from its height."""

    def process(self, detections, rgb_image, depth_image, camera_info, model_masks=None):
        poles = _deduplicate(
            [
                det
                for det in detections.detections
                if _class_id(det) == RED_POLE_CLASS_ID
                and _score(det) >= MIN_POLE_SCORE
                and _has_red_appearance(det, rgb_image)
            ]
        )
        detections.detections = []
        if not poles:
            return detections

        pole = copy.deepcopy(max(poles, key=lambda p: (p.detection.bbox.size_y, _score(p))))
        fy = float(camera_info.k[4])
        height_px = float(pole.detection.bbox.size_y)
        if fy <= 0.0 or height_px < MIN_POLE_HEIGHT_PX:
            return detections

        # A vertical pole's image height gives its range along the optical axis,
        # whether or not it is centred.
        pole_range = POLE_HEIGHT_M * fy / height_px
        source_class = _class_id(pole)
        pole.detection.results[0].hypothesis.class_id = TARGET_CLASS_ID
        pole.distance_m = pole_range
        pole.pose_valid = False
        pole.extra = [
            KeyValue(key="target", value="red_divider_center"),
            KeyValue(key="pole_range_m", value=f"{pole_range:.3f}"),
            KeyValue(key="source_class", value=str(source_class)),
            KeyValue(key="pose_semantics", value="pole_center"),
        ]
        detections.detections = [pole]
        return detections
