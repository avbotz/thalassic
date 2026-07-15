"""Octagon search-image class remapping and distance."""

from __future__ import annotations

import numpy as np

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor


# Model output classes:  0=SOS,  1=hammer_and_wrench,  2=compass,  3=buoy
# Remap to:              0=SOS+buoy,  1=compass+tools
_CLASS_REMAP = {'3': '0', '2': '1'}
_CLASS_WIDTH_M = {'0': 0.4, '1': 0.3}


def _class_id(detection) -> str | None:
    if not detection.detection.results:
        return None
    return detection.detection.results[0].hypothesis.class_id


def _estimate_distance_m(bbox, class_id: str, camera_k) -> float:
    width_m = _CLASS_WIDTH_M.get(class_id)
    if width_m is None:
        return float("nan")
    fx = float(camera_k[0, 0])
    if bbox.size_x <= 0.0 or fx <= 0.0:
        return float("nan")
    return float((width_m * fx) / bbox.size_x)


@register_post_processor("octagon_search_image")
class OctagonSearchImagePostProcessor(TaskPostProcessor):
    """Remap model classes: 0 ← SOS+buoy, 1 ← compass+tools, with distance."""

    def process(self, detections, rgb_image, depth_image, camera_info, model_masks=None):
        k = camera_info.k if hasattr(camera_info, 'k') else None
        if k is None:
            return detections
        k = np.array(k, dtype=np.float64).reshape(3, 3)
        for detection in detections.detections:
            cid = _class_id(detection)
            if cid is None:
                continue
            if cid in _CLASS_REMAP:
                detection.detection.results[0].hypothesis.class_id = _CLASS_REMAP[cid]
                cid = _CLASS_REMAP[cid]
            detection.distance_m = _estimate_distance_m(detection.detection.bbox, cid, k)
        return detections
