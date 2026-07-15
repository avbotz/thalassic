"""Octagon table distance estimation."""

from __future__ import annotations

import numpy as np

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor


_TABLE_WIDTH_M = 21.5 * 0.0254  # 0.5461 m


def _estimate_distance_m(bbox, camera_k) -> float:
    fx = float(camera_k[0, 0])
    if bbox.size_x <= 0.0 or fx <= 0.0:
        return float("nan")
    return float((_TABLE_WIDTH_M * fx) / bbox.size_x)


@register_post_processor("octagon_table")
class OctagonTablePostProcessor(TaskPostProcessor):
    """Add distance estimate for the octagon table."""

    def process(self, detections, rgb_image, depth_image, camera_info, model_masks=None):
        k = camera_info.k if hasattr(camera_info, 'k') else None
        if k is None:
            return detections
        k = np.array(k, dtype=np.float64).reshape(3, 3)
        for detection in detections.detections:
            detection.distance_m = _estimate_distance_m(detection.detection.bbox, k)
        return detections
