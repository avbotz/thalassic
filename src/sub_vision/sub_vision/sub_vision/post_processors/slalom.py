"""
Slalom pole post-processing.

Selects the two leftmost poles from the three largest detections and publishes
one synthetic class-2 detection whose bearing points between the poles.
"""

from __future__ import annotations

import copy
import math

from diagnostic_msgs.msg import KeyValue
import numpy as np

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor

RAW_POLE_CLASS_IDS = {0, 1}
SLALOM_PAIR_CLASS_ID = "2"


def _class_id(det) -> int:
    if not det.detection.results:
        return -1
    try:
        return int(det.detection.results[0].hypothesis.class_id)
    except ValueError:
        return -1


def _bbox_width(det) -> float:
    return float(det.detection.bbox.size_x)


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


def _bbox_center_x(det) -> float:
    return float(det.detection.bbox.center.position.x)


def _format_pixel(pixel: tuple[float, float]) -> str:
    return f"{pixel[0]:.1f},{pixel[1]:.1f}"


def _format_bbox(box: tuple[float, float, float, float]) -> str:
    return ",".join(f"{coordinate:.1f}" for coordinate in box)


@register_post_processor("slalom")
class SlalomPostProcessor(TaskPostProcessor):
    """Replace raw pole detections with one combined pole-pair target."""

    def process(self, detections, rgb_image, depth_image, camera_info):
        k = np.array(camera_info.k, dtype=np.float64).reshape(3, 3)

        poles = [det for det in detections.detections if _class_id(det) in RAW_POLE_CLASS_IDS]
        poles.sort(key=_bbox_width, reverse=True)
        top_poles = poles[:3]

        top_poles.sort(key=_bbox_center_x)
        selected = top_poles[:2]

        if len(selected) < 2:
            detections.detections = []
            return detections

        p0 = _bbox_xyxy(selected[0])
        p1 = _bbox_xyxy(selected[1])
        mid_u = (p0[0] + p0[2] + p1[0] + p1[2]) / 4.0
        mid_v = (p0[1] + p0[3] + p1[1] + p1[3]) / 4.0

        fx, fy = k[0, 0], k[1, 1]
        bearing_h = math.atan2(mid_u - k[0, 2], fx) if fx > 0.0 else 0.0
        bearing_v = math.atan2(mid_v - k[1, 2], fy) if fy > 0.0 else 0.0

        pair = copy.deepcopy(selected[0])
        pair_box = pair.detection.bbox
        x1, y1 = min(p0[0], p1[0]), min(p0[1], p1[1])
        x2, y2 = max(p0[2], p1[2]), max(p0[3], p1[3])
        pair_box.center.position.x = (x1 + x2) / 2.0
        pair_box.center.position.y = (y1 + y2) / 2.0
        pair_box.size_x = x2 - x1
        pair_box.size_y = y2 - y1

        hypothesis = pair.detection.results[0].hypothesis
        hypothesis.class_id = SLALOM_PAIR_CLASS_ID
        hypothesis.score = min(_score(selected[0]), _score(selected[1]))
        pair.bearing_horizontal = bearing_h
        pair.bearing_vertical = bearing_v
        pair.distance_m = float("nan")
        pair.pose_valid = False
        pair.extra = [
            KeyValue(key="midpoint_px", value=_format_pixel((mid_u, mid_v))),
            KeyValue(key="source_classes", value=f"{_class_id(selected[0])},{_class_id(selected[1])}"),
            KeyValue(key="source_boxes_px", value=f"{_format_bbox(p0)};{_format_bbox(p1)}"),
            KeyValue(key="pose_semantics", value="midpoint"),
        ]

        detections.detections = [pair]

        return detections
