"""Red-divider selection for the yaw-locked slalom mission.

``slalom_redpoles_osu`` may occasionally classify a white pole as red. Keep
the nearest candidate anywhere in frame only when its image crop passes the
OpenCV red-appearance test.
"""

from __future__ import annotations

import copy
import math

from diagnostic_msgs.msg import KeyValue
import numpy as np

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor


RAW_POLE_CLASS_IDS = {0}
SLALOM_RED_POLE_CLASS_ID = '2'
MIN_POLE_SCORE = 0.10
MIN_POLE_HEIGHT_PX = 8.0
POLE_NMS_IOU = 0.70
MIN_RED_PIXEL_FRACTION = 0.05
# The pole is 0.9 m tall in Stonefish.
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


def _has_red_appearance(det, image_bgr: np.ndarray) -> bool:
    """Reject white-pole boxes that the red-only model occasionally emits."""
    if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] < 3:
        return False

    image_height, image_width = image_bgr.shape[:2]
    x1, y1, x2, y2 = _bbox_xyxy(det)
    x1 = max(0, min(image_width, int(math.floor(x1))))
    y1 = max(0, min(image_height, int(math.floor(y1))))
    x2 = max(0, min(image_width, int(math.ceil(x2))))
    y2 = max(0, min(image_height, int(math.ceil(y2))))
    if x2 <= x1 or y2 <= y1:
        return False

    # Inset horizontally so the pool background and annotation edges do not
    # dominate a narrow pole crop.
    inset = int((x2 - x1) * 0.20)
    crop = image_bgr[y1:y2, x1 + inset:x2 - inset]
    if crop.size == 0:
        return False

    pixels = crop.astype(np.float32)
    blue, green, red = np.moveaxis(pixels[..., :3], -1, 0)
    strongest_other = np.maximum(blue, green)
    red_pixels = (
        (red >= 25.0)
        & (red >= 1.08 * blue)
        & (red >= 1.08 * green)
        & ((red - strongest_other) >= 8.0)
    )
    return float(np.mean(red_pixels)) >= MIN_RED_PIXEL_FRACTION


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


def _red_pole_target(pole, k: np.ndarray, output_class_id: str, target_name: str):
    """Publish the red pole's center ray and forward range."""
    _, fy = float(k[0, 0]), float(k[1, 1])
    height_px = float(pole.detection.bbox.size_y)
    if fy <= 0.0 or height_px < MIN_POLE_HEIGHT_PX:
        return None

    pole_range = POLE_HEIGHT_M * fy / height_px
    if not math.isfinite(pole_range) or pole_range <= 0.0:
        return None

    source_class = _class_id(pole)
    hypothesis = pole.detection.results[0].hypothesis
    hypothesis.class_id = output_class_id
    # A vertical pole's image height yields its optical-axis (body-forward)
    # range directly; do not shorten it when the pole starts off-center.
    pole.distance_m = pole_range
    pole.pose_valid = False
    pole.extra = [
        KeyValue(key='target', value=target_name),
        KeyValue(key='pole_range_m', value=f'{pole_range:.3f}'),
        KeyValue(key='source_class', value=str(source_class)),
        KeyValue(key='pose_semantics', value='pole_center'),
    ]
    return pole


@register_post_processor('slalom_redpoles_osu')
@register_post_processor('slalom')
class SlalomPostProcessor(TaskPostProcessor):
    """Keep the closest model-detected red divider for yaw-locked navigation."""

    def process(self, detections, rgb_image, depth_image, camera_info, model_masks=None):
        k = np.array(camera_info.k, dtype=np.float64).reshape(3, 3)
        poles = _deduplicate([
            det for det in detections.detections
            if _class_id(det) in RAW_POLE_CLASS_IDS
            and _score(det) >= MIN_POLE_SCORE
            and _has_red_appearance(det, rgb_image)
        ])
        if not poles:
            detections.detections = []
            return detections

        nearest = max(poles, key=lambda pole: (pole.detection.bbox.size_y, _score(pole)))
        target = _red_pole_target(
            copy.deepcopy(nearest),
            k,
            SLALOM_RED_POLE_CLASS_ID,
            'red_divider_center',
        )
        detections.detections = [target] if target is not None else []
        return detections
