"""Reusable planar-object PnP post-processors."""

from __future__ import annotations

import math

import numpy as np
from diagnostic_msgs.msg import KeyValue

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.pose_geometry import (
    PinholeCamera,
    bbox_corners,
    best_detections_by_class,
    fill_pose,
    match_polygon_sequence,
    solve_object_pose,
    yaw_from_rvec,
)


class SinglePlanarObjectPostProcessor(TaskPostProcessor):
    """Estimate one detection's pose from its bbox and known planar geometry."""

    def __init__(
        self,
        object_points: np.ndarray,
        class_names: set[str],
        max_reprojection_error: float = 8.0,
        refine: bool = True,
    ):
        self._object_points = object_points
        self._class_names = class_names
        self._max_reprojection_error = max_reprojection_error
        self._refine = refine

    def process(self, detections, rgb_image, depth_image, camera_info):
        camera = PinholeCamera.from_camera_info(camera_info)
        best = best_detections_by_class(detections, self._class_names)
        candidates = best.values() if best else detections.detections

        for det in candidates:
            try:
                object_points = np.hstack(
                    [self._object_points, np.zeros((len(self._object_points), 1))]
                )
                image_points = bbox_corners(det)
                rvec, tvec, inliers = solve_object_pose(
                    camera,
                    object_points,
                    image_points,
                    max_reprojection_error=self._max_reprojection_error,
                    refine=self._refine,
                )
            except Exception as exc:  # noqa: BLE001 - per-frame enrichment should not crash
                det.pose_valid = False
                det.extra.append(KeyValue(key="pose_error", value=str(exc)))
                continue

            fill_pose(det, rvec, tvec)
            det.extra.append(
                KeyValue(key="yaw_deg", value=f"{math.degrees(yaw_from_rvec(rvec)):.1f}")
            )
            det.extra.append(KeyValue(key="pnp_inliers", value=str(len(inliers))))

        return detections


class MultiPartPlanarObjectPostProcessor(TaskPostProcessor):
    """Estimate one shared object pose from multiple detected planar parts."""

    def __init__(
        self,
        object_points_by_class: dict[str, np.ndarray],
        min_parts: int,
        output_class_name: str,
        max_reprojection_error: float = 100.0,
    ):
        self._object_points_by_class = object_points_by_class
        self._min_parts = min_parts
        self._output_class_name = output_class_name
        self._max_reprojection_error = max_reprojection_error

    def process(self, detections, rgb_image, depth_image, camera_info):
        best = best_detections_by_class(detections, self._object_points_by_class)
        if len(best) < self._min_parts:
            return detections

        labels = list(best)
        object_polygons = [self._object_points_by_class[label] for label in labels]
        image_polygons = [bbox_corners(best[label]) for label in labels]
        object_points_2d, image_points = match_polygon_sequence(object_polygons, image_polygons)
        object_points = np.hstack([object_points_2d, np.zeros((len(object_points_2d), 1))])

        try:
            rvec, tvec, inliers = solve_object_pose(
                PinholeCamera.from_camera_info(camera_info),
                object_points,
                image_points,
                max_reprojection_error=self._max_reprojection_error,
                refine=False,
            )
        except Exception as exc:  # noqa: BLE001
            for det in best.values():
                det.pose_valid = False
                det.extra.append(KeyValue(key="pose_error", value=str(exc)))
            return detections

        for label, det in best.items():
            fill_pose(det, rvec, tvec)
            det.extra.append(KeyValue(key="object_frame", value=self._output_class_name))
            det.extra.append(KeyValue(key="matched_part", value=label))
            det.extra.append(
                KeyValue(key="yaw_deg", value=f"{math.degrees(yaw_from_rvec(rvec)):.1f}")
            )
            det.extra.append(KeyValue(key="pnp_inliers", value=str(len(inliers))))

        return detections
