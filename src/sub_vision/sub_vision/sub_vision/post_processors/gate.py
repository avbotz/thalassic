"""PnP post-processor for the gate task."""

from __future__ import annotations

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.object_points import (
    GATE_BBOX_OBJECT_POINTS,
    GATE_FRONT_OBJECT_POINTS,
)
from sub_vision.post_processors.planar import (
    MultiPartPlanarObjectPostProcessor,
    SinglePlanarObjectPostProcessor,
)
from sub_vision.post_processors.pose_geometry import best_detections_by_class
from sub_vision.post_processors.registry import register_post_processor


@register_post_processor("gate")
class GatePostProcessor(TaskPostProcessor):
    """Estimate gate pose from multipart gate labels or a whole-gate bbox."""

    def __init__(self):
        self._multipart = MultiPartPlanarObjectPostProcessor(
            GATE_FRONT_OBJECT_POINTS,
            min_parts=2,
            output_class_name="gate",
            max_reprojection_error=100.0,
        )
        self._whole_gate = SinglePlanarObjectPostProcessor(
            GATE_BBOX_OBJECT_POINTS,
            {"gate"},
            max_reprojection_error=20.0,
            refine=True,
        )

    def process(self, detections, rgb_image, depth_image, camera_info):
        part_detections = best_detections_by_class(detections, GATE_FRONT_OBJECT_POINTS)
        if len(part_detections) >= 2:
            return self._multipart.process(detections, rgb_image, depth_image, camera_info)
        return self._whole_gate.process(detections, rgb_image, depth_image, camera_info)
