"""Normalize detections from the combined octagon front-camera model."""

from __future__ import annotations

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor


TASK = 'octagon_search_image'


# Model output classes:  0=SOS,  1=hammer_and_wrench,  2=compass,  3=buoy
# Remap to:              0=SOS+buoy,  1=compass+tools
CLASS_REMAP = {'3': '0', '2': '1'}


def _class_id(detection) -> str | None:
    if not detection.detection.results:
        return None
    return detection.detection.results[0].hypothesis.class_id


@register_post_processor(TASK)
class OctagonPostProcessor(TaskPostProcessor):
    """Remap model classes: 0 ← SOS+buoy, 1 ← compass+tools."""

    def process(self, detections, rgb_image, depth_image, camera_info, model_masks=None):
        for detection in detections.detections:
            class_id = _class_id(detection)
            if class_id in CLASS_REMAP:
                detection.detection.results[0].hypothesis.class_id = CLASS_REMAP[class_id]
        return detections
