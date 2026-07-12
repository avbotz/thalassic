"""
Slalom pole post-processing.

Selects the two leftmost poles from the three largest detections and computes
the midpoint bearing between them (similar to gate post-processing).
"""

from __future__ import annotations

import math
from diagnostic_msgs.msg import KeyValue
import numpy as np

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor

SLALOM_CLASS_ID = 2


def _class_id(det) -> int:
    if not det.detection.results:
        return -1
    try:
        return int(det.detection.results[0].hypothesis.class_id)
    except ValueError:
        return -1


def _bbox_width(det) -> float:
    return float(det.detection.bbox.size_x)


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


@register_post_processor("slalom")
class SlalomPostProcessor(TaskPostProcessor):
    """Keep the two leftmost large poles and add midpoint bearing."""

    def process(self, detections, rgb_image, depth_image, camera_info):
        k = np.array(camera_info.k, dtype=np.float64).reshape(3, 3)

        poles = [
            det for det in detections.detections if _class_id(det) == SLALOM_CLASS_ID
        ]
        poles.sort(key=_bbox_width, reverse=True)
        top_poles = poles[:3]

        top_poles.sort(key=_bbox_center_x)
        selected = top_poles[:2]

        detections.detections = selected

        if len(selected) < 2:
            return detections

        p0 = _bbox_xyxy(selected[0])
        p1 = _bbox_xyxy(selected[1])
        mid_u = (p0[0] + p0[2] + p1[0] + p1[2]) / 4.0
        mid_v = (p0[1] + p0[3] + p1[1] + p1[3]) / 4.0

        bearing_h = math.atan2(mid_u - k[0, 2], k[0, 0])
        bearing_v = math.atan2(mid_v - k[1, 2], k[1, 1])

        for det in selected:
            det.extra.append(
                KeyValue(key="midpoint_px", value=_format_pixel((mid_u, mid_v)))
            )
            det.extra.append(
                KeyValue(key="aim_bearing_horizontal", value=f"{bearing_h:.6f}")
            )
            det.extra.append(
                KeyValue(key="aim_bearing_vertical", value=f"{bearing_v:.6f}")
            )
            det.extra.append(KeyValue(key="pose_semantics", value="midpoint"))

        return detections
