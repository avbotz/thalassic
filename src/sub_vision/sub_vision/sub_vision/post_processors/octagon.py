"""Octagon task: range to the table, and the search images grouped by role.

Both models are box-only, so range comes from each object's known width.
"""

from __future__ import annotations

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor

TABLE_WIDTH_M = 21.5 * 0.0254  # 0.5461 m

# The search-image model's classes are 0 SOS, 1 hammer and wrench, 2 compass, 3 buoy. They are
# merged by role: 0 = SOS + buoy (Search & Rescue), 1 = compass + hammer and wrench (Survey &
# Repair).
SEARCH_IMAGE_ROLES = {"0": "0", "1": "1", "2": "1", "3": "0"}
SEARCH_IMAGE_WIDTH_M = {"0": 0.4, "1": 0.3}


def _width_distance_m(width_m: float, bbox, camera_info) -> float:
    """Range from an object's known width and its box width: ``width_m * fx / width_px``."""
    fx = float(camera_info.k[0])
    if bbox.size_x <= 0.0 or fx <= 0.0:
        return float("nan")
    return float(width_m * fx / bbox.size_x)


@register_post_processor("octagon_table")
class OctagonTablePostProcessor(TaskPostProcessor):
    """Range every table detection from the table's width."""

    def process(self, detections, rgb_image, depth_image, camera_info, model_masks=None):
        for det in detections.detections:
            det.distance_m = _width_distance_m(TABLE_WIDTH_M, det.detection.bbox, camera_info)
        return detections


@register_post_processor("octagon_search_image")
class OctagonSearchImagePostProcessor(TaskPostProcessor):
    """Relabel each search image by role and range it from its width."""

    def process(self, detections, rgb_image, depth_image, camera_info, model_masks=None):
        for det in detections.detections:
            if not det.detection.results:
                continue
            hypothesis = det.detection.results[0].hypothesis
            hypothesis.class_id = SEARCH_IMAGE_ROLES.get(hypothesis.class_id, hypothesis.class_id)
            width_m = SEARCH_IMAGE_WIDTH_M.get(hypothesis.class_id, float("nan"))
            det.distance_m = _width_distance_m(width_m, det.detection.bbox, camera_info)
        return detections
