"""Depth + intrinsics math for enriching 2D detections with 3D metadata.

This module is deliberately dependency-light (numpy only) so the geometry can
be unit tested without ROS, OpenCV, or any inference backend installed.

Conventions
-----------
* Depth images are ``16UC1`` in **millimeters**, matching the OAK-D Pro stereo
  output (and the sim bridge). A value of ``0`` means "no return" / invalid.
* The camera intrinsics ``K`` are the 3x3 matrix from ``CameraInfo.k``::

      [[fx,  0, cx],
       [ 0, fy, cy],
       [ 0,  0,  1]]

* Returned 3D points are in the camera **optical** frame: +x right, +y down,
  +z forward (the standard ROS optical convention).
"""

import numpy as np

# Depth pixels below this many millimeters are treated as invalid sensor noise.
_MIN_VALID_DEPTH_MM = 1


def deproject_pixel(u: float, v: float, depth_m: float, k: np.ndarray) -> np.ndarray:
    """Back-project a pixel + depth into a 3D point in the optical frame.

    Args:
        u, v: Pixel coordinates (column, row).
        depth_m: Depth at that pixel, in meters.
        k: 3x3 camera intrinsics matrix.

    Returns:
        ``np.array([x, y, z])`` in meters, camera optical frame.
    """
    fx, fy = k[0, 0], k[1, 1]
    cx, cy = k[0, 2], k[1, 2]
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    return np.array([x, y, depth_m], dtype=np.float64)


def sample_bbox_depth_m(
    depth_mm: np.ndarray,
    cx: float,
    cy: float,
    width: float,
    height: float,
    shrink: float = 0.5,
) -> float:
    """Robustly estimate the distance to a bbox as the median valid depth.

    A centered sub-window (scaled by ``shrink``) is sampled so that background
    pixels around the object edges contribute less. Invalid (zero) depths are
    ignored.

    Args:
        depth_mm: ``16UC1`` depth image in millimeters (H x W).
        cx, cy: Bounding-box center, in pixels.
        width, height: Bounding-box size, in pixels.
        shrink: Fraction of the bbox to sample about its center, in (0, 1].

    Returns:
        Median depth in meters, or ``float('nan')`` if no valid pixel is found.
    """
    img_h, img_w = depth_mm.shape[:2]

    half_w = max(width * shrink / 2.0, 0.5)
    half_h = max(height * shrink / 2.0, 0.5)

    x0 = int(np.clip(np.floor(cx - half_w), 0, img_w - 1))
    x1 = int(np.clip(np.ceil(cx + half_w), 0, img_w - 1))
    y0 = int(np.clip(np.floor(cy - half_h), 0, img_h - 1))
    y1 = int(np.clip(np.ceil(cy + half_h), 0, img_h - 1))

    window = depth_mm[y0 : y1 + 1, x0 : x1 + 1]
    valid = window[window >= _MIN_VALID_DEPTH_MM]
    if valid.size == 0:
        return float("nan")

    return float(np.median(valid)) / 1000.0


def estimate_distance_and_point(
    depth_mm: np.ndarray,
    bbox_cx: float,
    bbox_cy: float,
    bbox_w: float,
    bbox_h: float,
    k: np.ndarray,
) -> tuple[float, np.ndarray | None]:
    """Estimate distance to a detection and the 3D point at its bbox center.

    Returns:
        ``(distance_m, point_xyz)`` where ``distance_m`` is the median depth in
        meters (NaN if unavailable) and ``point_xyz`` is the deprojected bbox
        center (``None`` when no valid depth exists).
    """
    distance_m = sample_bbox_depth_m(depth_mm, bbox_cx, bbox_cy, bbox_w, bbox_h)
    if not np.isfinite(distance_m):
        return distance_m, None

    point = deproject_pixel(bbox_cx, bbox_cy, distance_m, k)
    return distance_m, point
