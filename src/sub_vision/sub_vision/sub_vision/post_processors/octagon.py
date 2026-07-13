"""Normalize detections from the combined octagon front-camera model."""

from __future__ import annotations

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor


TASK = 'octagon_gate_buoy_compass_sos_hammer'
IGNORED_CLASS_ID = '1'
UNIFIED_DIVERSION_CLASS_ID = '2'


def _class_id(detection) -> str | None:
    if not detection.detection.results:
        return None
    return detection.detection.results[0].hypothesis.class_id


@register_post_processor(TASK)
class OctagonPostProcessor(TaskPostProcessor):
    """Drop class 1 and expose raw classes 2/3 as one class-2 target."""

    def process(self, detections, rgb_image, depth_image, camera_info, model_masks=None):
        normalized = []
        for detection in detections.detections:
            class_id = _class_id(detection)
            if class_id == IGNORED_CLASS_ID:
                continue
            if class_id == '3':
                detection.detection.results[0].hypothesis.class_id = UNIFIED_DIVERSION_CLASS_ID
            normalized.append(detection)
        detections.detections = normalized
        return detections
