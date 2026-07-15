"""Shared camera-geometry / PnP utilities.

Ports the distortion-aware backprojection, RANSAC PnP, and pose-covariance
techniques from BumblebeeAS/pose_estimator's ``utils/pose_estimator.py`` into
one helper module, so post-processors share a single, distortion-correct
implementation instead of re-deriving pinhole math (which silently assumes
zero lens distortion) in each task module.
"""

from __future__ import annotations

import cv2
import numpy as np


def _camera_matrix(camera_info) -> np.ndarray:
    matrix = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError("camera intrinsics are unavailable")
    return matrix


def _distortion(camera_info) -> np.ndarray:
    return np.asarray(camera_info.d, dtype=np.float64)


def undistort_normalized(u: float, v: float, camera_info) -> tuple[float, float]:
    """Undo lens distortion, returning normalized (x/z, y/z) camera-ray coordinates."""
    point = np.array([[[u, v]]], dtype=np.float64)
    xn, yn = cv2.undistortPoints(point, _camera_matrix(camera_info), _distortion(camera_info))[0][0]
    return float(xn), float(yn)


def bearings_from_pixel(u: float, v: float, camera_info) -> tuple[float, float]:
    """Angles from the optical axis to a pixel, correcting for lens distortion.

    Positive horizontal = target right of center, positive vertical = target
    below center (optical-frame convention). With zero distortion this
    matches the plain pinhole ``atan2((u - cx) / fx, 1)`` formula it replaces.
    """
    xn, yn = undistort_normalized(u, v, camera_info)
    return float(np.arctan2(xn, 1.0)), float(np.arctan2(yn, 1.0))


def deproject_pixel(u: float, v: float, depth_m: float, camera_info) -> np.ndarray:
    """Backproject a pixel to a camera-frame 3D point given known depth (Z)."""
    xn, yn = undistort_normalized(u, v, camera_info)
    return np.array([xn * depth_m, yn * depth_m, depth_m], dtype=np.float64)


def solve_pnp_ransac(
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera_info,
    max_reprojection_error: float = 8.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
    """RANSAC PnP via SQPnP; returns ``(rvec, tvec, inlier_indices_or_none)``.

    Ported from pose_estimator's ``get_object_pose``. SQPnP handles planar and
    non-planar point sets alike, unlike the coplanarity assumptions IPPE/EPnP
    make, at the cost of needing >= 4 correspondences.
    """
    if len(object_points) < 4:
        raise ValueError(f"at least 4 points needed for PnP, got {len(object_points)}")

    _, rvec, tvec, inliers = cv2.solvePnPRansac(
        np.ascontiguousarray(object_points, dtype=np.float64),
        np.ascontiguousarray(image_points, dtype=np.float64),
        _camera_matrix(camera_info),
        _distortion(camera_info),
        useExtrinsicGuess=False,
        reprojectionError=max_reprojection_error,
        flags=cv2.SOLVEPNP_SQPNP,
    )
    return rvec, tvec, inliers


def filter_points_by_homography(
    object_points_xy: np.ndarray, image_points: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Keep only point pairs consistent with a planar homography fit.

    ``object_points_xy`` is Nx2 (already flattened to the object's local
    plane). Ported from pose_estimator's ``filter_by_homography``; useful
    before :func:`solve_pnp_ransac` when correspondences come from noisy
    contour/segmentation matching rather than a small set of known-rigid
    points, where a strict RANSAC reprojection threshold alone leaves too few
    inliers.
    """
    if len(object_points_xy) < 4:
        raise ValueError(f"at least 4 points needed for homography, got {len(object_points_xy)}")
    if len(object_points_xy) != len(image_points):
        raise ValueError("object_points_xy and image_points must be the same length")

    _, mask = cv2.findHomography(
        np.ascontiguousarray(object_points_xy, dtype=np.float64),
        np.ascontiguousarray(image_points, dtype=np.float64),
        cv2.USAC_MAGSAC,
        3.5,
        maxIters=1_000,
        confidence=0.999,
    )
    keep = mask.flatten().astype(bool)
    return object_points_xy[keep], image_points[keep]


def estimate_pose_covariance(
    object_points: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, camera_info
) -> np.ndarray:
    """6x6 covariance of ``[tx, ty, tz, rx, ry, rz]`` from the PnP reprojection Jacobian.

    Ported from pose_estimator's ``estimate_covariance`` (Fisher information
    from the ``cv2.projectPoints`` Jacobian). This reflects only how well the
    point geometry constrains the solve, not full sensor-noise -- but it lets
    a downstream consumer discount a poorly-conditioned PnP fit (e.g. a
    near-degenerate or nearly-collinear point set) instead of treating every
    ``pose_valid`` detection as equally trustworthy.

    Raises:
        numpy.linalg.LinAlgError: if the Jacobian is singular (e.g. too few
            or degenerate points).
    """
    _, jacobian = cv2.projectPoints(
        np.ascontiguousarray(object_points, dtype=np.float64),
        rvec,
        tvec,
        _camera_matrix(camera_info),
        _distortion(camera_info),
    )
    jacobian = jacobian[:, :6].copy()
    # cv2 orders Jacobian columns [rvec(3), tvec(3), ...]; swap to match the
    # ROS pose-covariance convention of [position(3), orientation(3)].
    jacobian[:, :3], jacobian[:, 3:] = jacobian[:, 3:].copy(), jacobian[:, :3].copy()
    return np.linalg.inv(jacobian.T @ jacobian)
