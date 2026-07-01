"""Approximate PnP post-processor for one visible slalom layer."""

from __future__ import annotations

import math

import numpy as np
from diagnostic_msgs.msg import KeyValue

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.object_points import SLALOM_GATE_OBJECT_POINTS
from sub_vision.post_processors.pose_geometry import (
    PinholeCamera,
    bbox_corners,
    class_name,
    fill_pose,
    match_polygon_sequence,
    score,
    solve_object_pose,
    yaw_from_rvec,
)
from sub_vision.post_processors.registry import register_post_processor


@register_post_processor("slalom")
class SlalomPostProcessor(TaskPostProcessor):
    """Estimate pose for one slalom gate layer from red/white pole bboxes."""

    def process(self, detections, rgb_image, depth_image, camera_info):
        red = self._best_by_name(detections.detections, "red_pole")
        white = [det for det in detections.detections if class_name(det) == "white_pole"]
        if red is None or not white:
            return detections

        red_x = red.detection.bbox.center.position.x
        left = self._best_side(white, red_x, want_left=True)
        right = self._best_side(white, red_x, want_left=False)

        selected = [("red_pole", red)]
        if left is not None:
            selected.insert(0, ("white_pole_left", left))
        if right is not None:
            selected.append(("white_pole_right", right))

        if len(selected) < 2:
            return detections

        object_polygons = [SLALOM_GATE_OBJECT_POINTS[label] for label, _ in selected]
        image_polygons = [bbox_corners(det) for _, det in selected]
        object_points_2d, image_points = match_polygon_sequence(object_polygons, image_polygons)
        object_points = np.hstack([object_points_2d, np.zeros((len(object_points_2d), 1))])

        try:
            rvec, tvec, inliers = solve_object_pose(
                PinholeCamera.from_camera_info(camera_info),
                object_points,
                image_points,
                max_reprojection_error=75.0,
                refine=False,
            )
        except Exception as exc:  # noqa: BLE001
            for _, det in selected:
                det.pose_valid = False
                det.extra.append(KeyValue(key="pose_error", value=str(exc)))
            return detections

        for label, det in selected:
            fill_pose(det, rvec, tvec)
            det.extra.append(KeyValue(key="object_frame", value="slalom_layer"))
            det.extra.append(KeyValue(key="matched_part", value=label))
            det.extra.append(
                KeyValue(key="yaw_deg", value=f"{math.degrees(yaw_from_rvec(rvec)):.1f}")
            )
            det.extra.append(KeyValue(key="pnp_inliers", value=str(len(inliers))))

        return detections

    @staticmethod
    def _best_by_name(detections, name: str):
        matching = [det for det in detections if class_name(det) == name]
        if not matching:
            return None
        return max(matching, key=score)

    @staticmethod
    def _best_side(detections, red_x: float, want_left: bool):
        side = [
            det
            for det in detections
            if (det.detection.bbox.center.position.x < red_x) == want_left
        ]
        if not side:
            return None
        return max(side, key=score)
