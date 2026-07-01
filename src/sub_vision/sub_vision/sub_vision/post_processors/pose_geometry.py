"""Shared camera geometry and PnP helpers for vision post-processors."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence

import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo


@dataclass(frozen=True)
class PinholeCamera:
    """Minimal pinhole camera model built from ``sensor_msgs/CameraInfo``."""

    frame_id: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    d: np.ndarray

    @classmethod
    def from_camera_info(cls, camera_info: CameraInfo, rectified: bool = False):
        if rectified:
            return cls(
                camera_info.header.frame_id,
                camera_info.width,
                camera_info.height,
                camera_info.p[0],
                camera_info.p[5],
                camera_info.p[2],
                camera_info.p[6],
                np.zeros(5, dtype=np.float64),
            )
        return cls(
            camera_info.header.frame_id,
            camera_info.width,
            camera_info.height,
            camera_info.k[0],
            camera_info.k[4],
            camera_info.k[2],
            camera_info.k[5],
            np.asarray(camera_info.d, dtype=np.float64),
        )

    def camera_matrix(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    def dist_coeffs(self) -> np.ndarray:
        return self.d


def bbox_corners(det) -> np.ndarray:
    """Return bbox corners as top-left, bottom-left, bottom-right, top-right."""
    bbox = det.detection.bbox
    half_w = bbox.size_x / 2.0
    half_h = bbox.size_y / 2.0
    cx = bbox.center.position.x
    cy = bbox.center.position.y
    return np.array(
        [
            [cx - half_w, cy - half_h],
            [cx - half_w, cy + half_h],
            [cx + half_w, cy + half_h],
            [cx + half_w, cy - half_h],
        ],
        dtype=np.float64,
    )


def class_name(det) -> str:
    if not det.detection.results:
        return ""
    return det.detection.results[0].hypothesis.class_id


def score(det) -> float:
    if not det.detection.results:
        return 0.0
    return float(det.detection.results[0].hypothesis.score)


def best_detections_by_class(detections, names: Iterable[str]) -> dict[str, object]:
    best: dict[str, object] = {}
    names = set(names)
    for det in detections.detections:
        name = class_name(det)
        if name not in names:
            continue
        if name not in best or score(det) > score(best[name]):
            best[name] = det
    return best


def match_polygon_points(
    object_polygon: np.ndarray, image_polygon: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Order two equal-size polygons and choose the lowest-cost cyclic match."""
    if len(object_polygon) != len(image_polygon):
        raise ValueError("polygons must have the same number of points")

    obj = _order_clockwise(object_polygon)
    img = _order_clockwise(image_polygon)
    obj_norm = _normalize_polygon(obj)
    img_norm = _normalize_polygon(img)

    best_cost = float("inf")
    best_perm = np.arange(len(img))
    identity = np.arange(len(img))
    for shift in range(len(img)):
        perm = np.roll(identity, shift)
        cost = np.sum(np.linalg.norm(obj_norm - img_norm[perm], axis=1) ** 2)
        if cost < best_cost:
            best_cost = cost
            best_perm = perm

    return obj, img[best_perm]


def match_polygon_sequence(
    object_polygons: Sequence[np.ndarray], image_polygons: Sequence[np.ndarray]
) -> tuple[np.ndarray, np.ndarray]:
    object_points = []
    image_points = []
    for object_polygon, image_polygon in zip(object_polygons, image_polygons):
        matched_object, matched_image = match_polygon_points(object_polygon, image_polygon)
        object_points.extend(matched_object)
        image_points.extend(matched_image)
    return np.asarray(object_points, dtype=np.float64), np.asarray(image_points, dtype=np.float64)


def solve_object_pose(
    camera: PinholeCamera,
    object_points: np.ndarray,
    image_points: np.ndarray,
    max_reprojection_error: float = 8.0,
    refine: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
    """Estimate object pose using RANSAC PnP, with optional VVS refinement."""
    if len(object_points) < 4:
        raise ValueError(f"at least 4 points are required, got {len(object_points)}")
    if len(object_points) != len(image_points):
        raise ValueError("object and image point counts must match")

    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points.astype(np.float64),
        image_points.astype(np.float64),
        camera.camera_matrix(),
        camera.dist_coeffs(),
        useExtrinsicGuess=False,
        reprojectionError=max_reprojection_error,
        flags=cv2.SOLVEPNP_SQPNP,
    )
    if not ok or inliers is None:
        raise ValueError("PnP failed to find inliers")

    if refine:
        rvec, tvec = cv2.solvePnPRefineVVS(
            object_points.astype(np.float64),
            image_points.astype(np.float64),
            camera.camera_matrix(),
            camera.dist_coeffs(),
            rvec,
            tvec,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 1000, 1e-6),
        )

    return rvec, tvec, inliers


def fill_pose(det, rvec: np.ndarray, tvec: np.ndarray) -> None:
    translation = np.asarray(tvec, dtype=np.float64).reshape(3)
    det.pose.position.x = float(translation[0])
    det.pose.position.y = float(translation[1])
    det.pose.position.z = float(translation[2])
    qx, qy, qz, qw = rvec_to_quaternion(rvec)
    det.pose.orientation.x = qx
    det.pose.orientation.y = qy
    det.pose.orientation.z = qz
    det.pose.orientation.w = qw
    det.distance_m = float(np.linalg.norm(translation))
    det.pose_valid = True


def rvec_to_quaternion(rvec: np.ndarray) -> tuple[float, float, float, float]:
    rot, _ = cv2.Rodrigues(rvec)
    trace = rot[0, 0] + rot[1, 1] + rot[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rot[2, 1] - rot[1, 2]) / s
        qy = (rot[0, 2] - rot[2, 0]) / s
        qz = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        qw = (rot[2, 1] - rot[1, 2]) / s
        qx = 0.25 * s
        qy = (rot[0, 1] + rot[1, 0]) / s
        qz = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        qw = (rot[0, 2] - rot[2, 0]) / s
        qx = (rot[0, 1] + rot[1, 0]) / s
        qy = 0.25 * s
        qz = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        qw = (rot[1, 0] - rot[0, 1]) / s
        qx = (rot[0, 2] + rot[2, 0]) / s
        qy = (rot[1, 2] + rot[2, 1]) / s
        qz = 0.25 * s
    return float(qx), float(qy), float(qz), float(qw)


def yaw_from_rvec(rvec: np.ndarray) -> float:
    rot, _ = cv2.Rodrigues(rvec)
    return math.atan2(rot[1, 0], rot[0, 0])


def _order_clockwise(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64)
    centroid = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - centroid[1], pts[:, 0] - centroid[0])
    return pts[np.argsort(angles)]


def _normalize_polygon(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64).copy()
    pts -= pts.mean(axis=0)
    avg_distance = np.mean(np.linalg.norm(pts, axis=1))
    if avg_distance == 0.0:
        raise ValueError("cannot normalize a zero-area polygon")
    return pts / avg_distance
