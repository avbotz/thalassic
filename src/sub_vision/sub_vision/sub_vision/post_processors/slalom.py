"""Single-pole slalom steering.

Each stationary scan publishes one synthetic target: the image-leftmost
detected pole, displaced a fixed safe distance to its right.  Mission control
then compares these targets over the complete spin and selects the globally
leftmost pole.  No row association or three-pole geometry is needed.
"""

from __future__ import annotations

import copy
import math

from diagnostic_msgs.msg import KeyValue
import numpy as np

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor


RAW_POLE_CLASS_IDS = {0, 1}
SLALOM_TARGET_CLASS_ID = '2'
MIN_POLE_SCORE = 0.10
MIN_POLE_HEIGHT_PX = 8.0
POLE_NMS_IOU = 0.70
# The pole is 0.9 m tall in Stonefish.  Staying 0.9 m to its right clears a
# pole and the vehicle hull without needing to identify the other poles.
POLE_HEIGHT_M = 0.90
RIGHT_CLEARANCE_M = 0.90


def _class_id(det) -> int:
    if not det.detection.results:
        return -1
    try:
        return int(det.detection.results[0].hypothesis.class_id)
    except ValueError:
        return -1


def _score(det) -> float:
    return float(det.detection.results[0].hypothesis.score) if det.detection.results else 0.0


def _center_x(det) -> float:
    return float(det.detection.bbox.center.position.x)


def _bbox_xyxy(det) -> tuple[float, float, float, float]:
    box = det.detection.bbox
    return (box.center.position.x - box.size_x / 2.0, box.center.position.y - box.size_y / 2.0,
            box.center.position.x + box.size_x / 2.0, box.center.position.y + box.size_y / 2.0)


def _iou(first, second) -> float:
    ax1, ay1, ax2, ay2 = _bbox_xyxy(first)
    bx1, by1, bx2, by2 = _bbox_xyxy(second)
    intersection = max(0.0, min(ax2, bx2) - max(ax1, bx1)) * max(0.0, min(ay2, by2) - max(ay1, by1))
    union = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1) + max(0.0, bx2 - bx1) * max(0.0, by2 - by1) - intersection
    return intersection / union if union > 0.0 else 0.0


def _deduplicate(poles) -> list:
    unique = []
    for pole in sorted(poles, key=_score, reverse=True):
        if not any(_iou(pole, kept) >= POLE_NMS_IOU for kept in unique):
            unique.append(pole)
    return unique


def _right_of_pole_target(pole, k: np.ndarray):
    """Aim at a point RIGHT_CLEARANCE_M right of one detected pole."""
    fx, fy = float(k[0, 0]), float(k[1, 1])
    height_px = float(pole.detection.bbox.size_y)
    if fx <= 0.0 or fy <= 0.0 or height_px < MIN_POLE_HEIGHT_PX:
        return None

    pole_bearing = float(pole.bearing_horizontal)
    pole_range = POLE_HEIGHT_M * fy / height_px
    forward_range = pole_range * math.cos(pole_bearing)
    if not math.isfinite(forward_range) or forward_range <= 0.0:
        return None

    # Optical x is right-positive.  The command conversion below maps that
    # rightward optical aim into the vehicle's REP-103 yaw convention.
    desired_bearing = math.atan2(
        forward_range * math.tan(pole_bearing) + RIGHT_CLEARANCE_M,
        forward_range,
    )
    path_distance = forward_range / math.cos(desired_bearing)
    if not math.isfinite(path_distance) or path_distance <= 0.0:
        return None

    target = copy.deepcopy(pole)
    hypothesis = target.detection.results[0].hypothesis
    hypothesis.class_id = SLALOM_TARGET_CLASS_ID
    target.bearing_horizontal = desired_bearing
    target.distance_m = path_distance
    target.pose_valid = False
    target.extra = [
        KeyValue(key='yaw_deg', value=f'{-math.degrees(desired_bearing):.6f}'),
        KeyValue(key='target', value='right_of_leftmost_pole'),
        KeyValue(key='pole_count', value='1'),
        KeyValue(key='pole_range_m', value=f'{pole_range:.3f}'),
        KeyValue(key='right_clearance_m', value=f'{RIGHT_CLEARANCE_M:.3f}'),
        KeyValue(key='source_class', value=str(_class_id(pole))),
        KeyValue(key='source_pole_x_px', value=f'{_center_x(pole):.1f}'),
        KeyValue(key='pose_semantics', value='orientation'),
    ]
    return target


@register_post_processor('slalom_redpoles_osu')
@register_post_processor('slalom')
class SlalomPostProcessor(TaskPostProcessor):
    """Create one right-clearance target from the image-leftmost raw pole."""

    def process(self, detections, rgb_image, depth_image, camera_info, model_masks=None):
        poles = _deduplicate([
            det for det in detections.detections
            if _class_id(det) in RAW_POLE_CLASS_IDS and _score(det) >= MIN_POLE_SCORE
        ])
        if not poles:
            detections.detections = []
            return detections

        leftmost = min(poles, key=_center_x)
        target = _right_of_pole_target(leftmost, np.array(camera_info.k, dtype=np.float64).reshape(3, 3))
        detections.detections = [leftmost, target] if target is not None else [leftmost]
        return detections
