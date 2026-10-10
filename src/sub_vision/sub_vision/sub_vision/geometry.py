"""Camera geometry shared by the node and the post-processors.

Everything here goes through the CameraInfo distortion model, so a calibrated lens gives the same
answers as the pinhole formulas it replaces would on an undistorted image. Pixels and rays are in
the camera optical frame (x right, y down, z forward).
"""

from __future__ import annotations

import cv2
import numpy as np


def camera_matrix(camera_info) -> np.ndarray:
    """K as a 3x3 array; raises ``ValueError`` for an uncalibrated source."""
    k = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
    if k[0, 0] <= 0.0 or k[1, 1] <= 0.0:
        raise ValueError("camera intrinsics are unavailable")
    return k


def distortion(camera_info) -> np.ndarray:
    return np.asarray(camera_info.d, dtype=np.float64)


def undistort_normalized(u: float, v: float, camera_info) -> tuple[float, float]:
    """The ray through a pixel, as normalized (x/z, y/z) with lens distortion removed."""
    point = np.array([[[u, v]]], dtype=np.float64)
    xn, yn = cv2.undistortPoints(point, camera_matrix(camera_info), distortion(camera_info))[0][0]
    return float(xn), float(yn)


def bearings_from_pixel(u: float, v: float, camera_info) -> tuple[float, float]:
    """Angles from the optical axis to a pixel: positive horizontal = right of center, positive
    vertical = below center."""
    xn, yn = undistort_normalized(u, v, camera_info)
    return float(np.arctan2(xn, 1.0)), float(np.arctan2(yn, 1.0))


def pose_covariance(
    object_points: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, camera_info
) -> np.ndarray:
    """6x6 covariance of ``[tx, ty, tz, rx, ry, rz]`` for a PnP solution, per pixel² of noise.

    The inverse Fisher information of the reprojection Jacobian (BumblebeeAS/pose_estimator's
    ``estimate_covariance``): it measures only how well the point geometry constrains the solve,
    which is enough to tell a near-degenerate fit from a good one.

    Raises:
        numpy.linalg.LinAlgError: if the Jacobian is singular (too few or degenerate points).
    """
    _, jacobian = cv2.projectPoints(
        np.ascontiguousarray(object_points, dtype=np.float64),
        rvec,
        tvec,
        camera_matrix(camera_info),
        distortion(camera_info),
    )
    # cv2 orders the columns [rvec, tvec, ...]; ROS pose covariances are [position, orientation]
    jacobian = np.hstack([jacobian[:, 3:6], jacobian[:, 0:3]])
    return np.linalg.inv(jacobian.T @ jacobian)
