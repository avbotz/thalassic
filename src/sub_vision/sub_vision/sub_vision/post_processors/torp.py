"""Torpedo board: pose by PnP on its four corners, and the hole to aim at.

``torp`` segments the board (class 0) and its holes (class 1); ``torp_find`` only boxes the
board. The board's corners come from, in order of preference: the relative depth map (a plane
inside the box with a clear edge), the model's board mask, the board's colour inside the box, or
the box itself. ``solvePnP`` on those corners gives the board pose in the camera optical frame.
Only a whole real outline (not the box, nor one the image edge cuts off) can show which way the
board faces, so a PnP pose's ``heading_yaw_deg`` and aim point are published only for one.

When depth runs, a good PnP solve on the best board's whole outline calibrates the relative depth
map to metres on the board plane. For up to DEPTH_CALIBRATION_MAX_AGE_S after, a frame whose PnP
fails takes the board plane, and its heading, from the calibrated depth instead.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import cv2
import numpy as np
from diagnostic_msgs.msg import KeyValue

from sub_vision import geometry
from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor

TORP_CLASS_ID = 0
TORP_HOLE_CLASS_ID = 1
# Class given to the hole to fire at
TORP_AIM_HOLE_CLASS_ID = "torp_aim_hole"
TORP_BOARD_SIDE_M = 0.6096
# The hole to aim at, in board coordinates (x right, y down from the board centre): the
# simulator's board has one at the upper centre. Board coordinates keep it the same hole as the
# vehicle moves or sees the board at an angle.
AIM_HOLE_BOARD_X_M = 0.0
AIM_HOLE_BOARD_Y_M = -0.18
MIN_AIM_HOLE_SCORE = 0.50
# Corners from a box are tens of pixels off on an angled board, so only clearly wrong fits are
# rejected; the error is published.
MAX_REPROJECTION_ERROR_PX = 50.0
MIN_OPENCV_CORNER_QUALITY = 0.25
DEPTH_CALIBRATION_MAX_AGE_S = 0.5
MIN_DEPTH_CALIBRATION_PIXELS = 40

# The board's corners in its own frame (centre origin, in its z = 0 plane), in the order the
# image corners are given: top-left, bottom-left, bottom-right, top-right. solvePnP's translation
# is then the board centre.
_HALF_SIDE_M = TORP_BOARD_SIDE_M / 2.0
_OBJECT_CORNERS = np.array(
    [
        [-_HALF_SIDE_M, -_HALF_SIDE_M, 0.0],
        [-_HALF_SIDE_M, _HALF_SIDE_M, 0.0],
        [_HALF_SIDE_M, _HALF_SIDE_M, 0.0],
        [_HALF_SIDE_M, -_HALF_SIDE_M, 0.0],
    ],
    dtype=np.float64,
)


@dataclass
class _DepthCalibration:
    """Camera-frame z (m) = scale * relative depth + offset."""

    scale: float
    offset: float
    created_at: float


def _class_id(detection) -> int:
    if not detection.detection.results:
        return -1
    try:
        return int(detection.detection.results[0].hypothesis.class_id)
    except ValueError:
        return -1


def _score(detection) -> float:
    if not detection.detection.results:
        return 0.0
    return float(detection.detection.results[0].hypothesis.score)


def _box_center(detection) -> np.ndarray:
    center = detection.detection.bbox.center.position
    return np.array([center.x, center.y], dtype=np.float64)


def _padded_box(detection, width: int, height: int) -> tuple[int, int, int, int]:
    """The detection's box grown by 20% (at least 8 px) a side, clipped to the image."""
    bbox = detection.detection.bbox
    pad_x = int(max(8.0, 0.20 * bbox.size_x))
    pad_y = int(max(8.0, 0.20 * bbox.size_y))
    x0 = max(0, int(round(bbox.center.position.x - bbox.size_x / 2.0)) - pad_x)
    x1 = min(width, int(round(bbox.center.position.x + bbox.size_x / 2.0)) + pad_x)
    y0 = max(0, int(round(bbox.center.position.y - bbox.size_y / 2.0)) - pad_y)
    y1 = min(height, int(round(bbox.center.position.y + bbox.size_y / 2.0)) + pad_y)
    return x0, y0, x1, y1


def bbox_corners(detection) -> np.ndarray:
    """Return bbox corners: top-left, bottom-left, bottom-right, top-right."""
    bbox = detection.detection.bbox
    half_width = bbox.size_x / 2.0
    half_height = bbox.size_y / 2.0
    center_x, center_y = _box_center(detection)
    return np.array(
        [
            [center_x - half_width, center_y - half_height],
            [center_x - half_width, center_y + half_height],
            [center_x + half_width, center_y + half_height],
            [center_x + half_width, center_y - half_height],
        ],
        dtype=np.float64,
    )


def _cut_off(detection, image: np.ndarray | None) -> bool:
    """Whether the box reaches the image edge (within a pixel), past which the board may go on."""
    if image is None:
        return False
    height, width = image.shape[:2]
    (x0, y0), (x1, y1) = bbox_corners(detection)[[0, 2]]
    return x0 <= 1.0 or y0 <= 1.0 or x1 >= width - 1.0 or y1 >= height - 1.0


def yolo_mask_corners(mask: np.ndarray | None, detection) -> tuple[np.ndarray, float] | None:
    """The board's four corners from a board mask, and how well they fit the box (0-1).

    Takes the two leftmost and two rightmost vertices of the mask's simplified outline (the
    approach the 2025 PnP used), which tolerates the extra vertices the holes add.
    """
    if mask is None or mask.ndim != 2:
        return None
    contours, _ = cv2.findContours(
        mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 0.0:
        return None
    polygon = cv2.approxPolyDP(contour, 0.015 * perimeter, True).reshape(-1, 2).astype(np.float64)
    if len(polygon) < 4:
        return None

    by_x = polygon[np.argsort(polygon[:, 0])]
    left = by_x[:2][np.argsort(by_x[:2, 1])]
    right = by_x[-2:][np.argsort(by_x[-2:, 1])]
    quad = np.array([left[0], left[1], right[1], right[0]], dtype=np.float64)
    if not cv2.isContourConvex(quad.astype(np.float32).reshape(-1, 1, 2)):
        return None

    bbox = detection.detection.bbox
    bbox_area = float(bbox.size_x * bbox.size_y)
    if bbox_area <= 1.0:
        return None
    area_ratio = abs(float(cv2.contourArea(quad.astype(np.float32)))) / bbox_area
    center_error = float(np.linalg.norm(np.mean(quad, axis=0) - _box_center(detection)))
    diagonal = math.hypot(bbox.size_x, bbox.size_y)
    sides = np.linalg.norm(np.roll(quad, -1, axis=0) - quad, axis=1)
    if (
        area_ratio < 0.20
        or area_ratio > 1.80
        or center_error > 0.35 * diagonal
        or np.min(sides) < 8.0
        or np.max(sides) / np.min(sides) > 3.5
    ):
        return None
    quality = min(1.0, area_ratio) * max(0.0, 1.0 - center_error / (0.35 * diagonal))
    return quad, quality


def depth_board_corners(depth_map: np.ndarray | None, detection) -> tuple[np.ndarray, float] | None:
    """The board's corners from the relative depth map, as :func:`yolo_mask_corners`.

    Fits a plane to the centre of the box, grows it to the pixels on that plane, and keeps the
    component nearest the box centre. That component must not reach the edge of the padded box:
    otherwise depth found no edge to the board (a smooth background) and has no corners to give.
    """
    if depth_map is None or depth_map.ndim != 2 or not np.all(np.isfinite(depth_map)):
        return None
    bbox = detection.detection.bbox
    h, w = depth_map.shape
    x0, y0, x1, y1 = _padded_box(detection, w, h)
    if x1 - x0 < 30 or y1 - y0 < 30:
        return None

    crop = depth_map[y0:y1, x0:x1].astype(np.float64)
    yy, xx = np.indices(crop.shape)
    core = (
        (xx >= crop.shape[1] // 4)
        & (xx < 3 * crop.shape[1] // 4)
        & (yy >= crop.shape[0] // 4)
        & (yy < 3 * crop.shape[0] // 4)
    )
    design = np.column_stack((xx[core], yy[core], np.ones(np.count_nonzero(core))))
    values = crop[core]
    if len(values) < 100 or np.ptp(values) < 1e-6:
        return None
    # Refit after rejecting outliers in the core (a hole, say)
    coeffs = np.linalg.lstsq(design, values, rcond=None)[0]
    residual = np.abs(values - design @ coeffs)
    scale = 1.4826 * float(np.median(residual))
    if not math.isfinite(scale):
        return None
    keep = residual <= max(2.5 * scale, 0.01 * float(np.ptp(values)))
    if np.count_nonzero(keep) < 100:
        return None
    coeffs = np.linalg.lstsq(design[keep], values[keep], rcond=None)[0]
    plane_residual = np.abs(crop - (coeffs[0] * xx + coeffs[1] * yy + coeffs[2]))
    threshold = max(3.0 * scale, 0.015 * float(np.ptp(values)))
    candidate = (plane_residual <= threshold).astype(np.uint8)
    candidate = cv2.morphologyEx(candidate, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    candidate = cv2.morphologyEx(candidate, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

    count, labels, stats, centroids = cv2.connectedComponentsWithStats(candidate)
    center = _box_center(detection) - np.array([x0, y0])
    choices = [
        label
        for label in range(1, count)
        if stats[label, cv2.CC_STAT_AREA] >= 0.05 * bbox.size_x * bbox.size_y
    ]
    if not choices:
        return None
    label = min(choices, key=lambda i: float(np.linalg.norm(centroids[i] - center)))
    left, top, width, height, _ = stats[label]
    if (
        left <= 1
        or top <= 1
        or left + width >= crop.shape[1] - 1
        or top + height >= crop.shape[0] - 1
    ):
        return None
    mask = np.zeros_like(depth_map, dtype=np.uint8)
    mask[y0:y1, x0:x1][labels == label] = 255
    return yolo_mask_corners(mask, detection)


def opencv_board_corners(
    rgb_image: np.ndarray | None, detection
) -> tuple[np.ndarray, float] | None:
    """The board's corners from its painted face inside the box, for box-only models.

    The HSV contour approach of the team's earlier torpedo detector, kept to the box so pool
    props cannot be picked up.
    """
    if rgb_image is None or rgb_image.ndim != 3:
        return None
    bbox = detection.detection.bbox
    x0, y0, x1, y1 = _padded_box(detection, rgb_image.shape[1], rgb_image.shape[0])
    if x1 - x0 < 20 or y1 - y0 < 20:
        return None

    # A broad warm-to-green range, covering both the simulator's painted board and the old detector's
    hsv = cv2.cvtColor(rgb_image[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 30, 80], np.uint8), np.array([95, 245, 255], np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = 0.05 * bbox.size_x * bbox.size_y
    contours = [contour for contour in contours if cv2.contourArea(contour) >= min_area]
    if not contours:
        return None
    hull = cv2.convexHull(np.vstack(contours))
    # Colour reaching the edge of the padded box (water or a wall in the range, say) found no edge
    # to the board, as in depth_board_corners: the outline would be the crop's
    hull_x, hull_y, hull_w, hull_h = cv2.boundingRect(hull)
    if (
        hull_x <= 1
        or hull_y <= 1
        or hull_x + hull_w >= mask.shape[1] - 1
        or hull_y + hull_h >= mask.shape[0] - 1
    ):
        return None
    perimeter = cv2.arcLength(hull, True)
    quad = cv2.approxPolyDP(hull, 0.045 * perimeter, True).reshape(-1, 2).astype(np.float64)
    if len(quad) != 4:
        return None
    quad += (x0, y0)
    # Match _OBJECT_CORNERS: top-left, bottom-left, bottom-right, top-right
    by_y = quad[np.argsort(quad[:, 1])]
    top = by_y[:2][np.argsort(by_y[:2, 0])]
    bottom = by_y[2:][np.argsort(by_y[2:, 0])]
    ordered = np.array([top[0], bottom[0], bottom[1], top[1]], dtype=np.float64)
    area_ratio = abs(float(cv2.contourArea(ordered.astype(np.float32)))) / max(
        float(bbox.size_x * bbox.size_y), 1.0
    )
    if area_ratio < MIN_OPENCV_CORNER_QUALITY or area_ratio > 2.0:
        return None
    return ordered, min(1.0, area_ratio)


def solve_torp_pose(image_points: np.ndarray, camera_info) -> tuple[np.ndarray, np.ndarray, float]:
    """Solve board pose and return rotation, translation, and RMS pixel error."""
    k = geometry.camera_matrix(camera_info)
    d = geometry.distortion(camera_info)
    success, rvec, tvec = cv2.solvePnP(_OBJECT_CORNERS, image_points, k, d, flags=cv2.SOLVEPNP_IPPE)
    if not success or float(tvec[2, 0]) <= 0.0:
        raise ValueError("PnP did not produce a forward-facing board pose")

    projected, _ = cv2.projectPoints(_OBJECT_CORNERS, rvec, tvec, k, d)
    error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - image_points) ** 2, axis=1))))
    if not math.isfinite(error) or error > MAX_REPROJECTION_ERROR_PX:
        raise ValueError(f"PnP reprojection error {error:.1f}px exceeds limit")
    return rvec, tvec, error


def _board_plane_depths(
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_info,
    shape: tuple[int, int],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Up to 900 pixels well inside the board outline, in an image of ``shape``, and the
    camera-frame z (m) of the solved board plane at each."""
    mask = np.zeros(shape, dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.round(image_points).astype(np.int32), 255)
    mask = cv2.erode(mask, np.ones((9, 9), np.uint8))
    ys, xs = np.nonzero(mask)
    if len(xs) > 900:
        picks = np.linspace(0, len(xs) - 1, 900, dtype=int)
        xs, ys = xs[picks], ys[picks]
    k = geometry.camera_matrix(camera_info)
    normal = cv2.Rodrigues(rvec)[0][:, 2]
    # The ray through (u, v) is ((u - cx) / fx, (v - cy) / fy, 1) * z; it meets the plane
    # n . p = n . t where z = n . t / (n . ray)
    denom = normal[0] * (xs - k[0, 2]) / k[0, 0] + normal[1] * (ys - k[1, 2]) / k[1, 1] + normal[2]
    valid = np.abs(denom) > 1e-6
    return xs[valid], ys[valid], float(normal @ tvec.reshape(3)) / denom[valid]


def _fit_depth_calibration(
    depth_map, image_points, rvec, tvec, camera_info
) -> _DepthCalibration | None:
    """Fit metres to relative depth over the board, where PnP gives the true z, or ``None`` if
    the fit is poor."""
    if depth_map is None or depth_map.ndim != 2:
        return None
    xs, ys, metric_z = _board_plane_depths(image_points, rvec, tvec, camera_info, depth_map.shape)
    relative = depth_map[ys, xs].astype(np.float64)
    valid = np.isfinite(relative) & np.isfinite(metric_z)
    relative, metric_z = relative[valid], metric_z[valid]
    if len(relative) < MIN_DEPTH_CALIBRATION_PIXELS or np.ptp(relative) < 1e-5:
        return None
    # Least squares, then again without the outliers
    design = np.column_stack((relative, np.ones_like(relative)))
    scale, offset = np.linalg.lstsq(design, metric_z, rcond=None)[0]
    residual = np.abs(metric_z - (scale * relative + offset))
    keep = residual <= max(0.05, 2.5 * float(np.median(residual)))
    if np.count_nonzero(keep) < MIN_DEPTH_CALIBRATION_PIXELS:
        return None
    scale, offset = np.linalg.lstsq(design[keep], metric_z[keep], rcond=None)[0]
    rms = float(np.sqrt(np.mean((metric_z[keep] - (scale * relative[keep] + offset)) ** 2)))
    if not math.isfinite(rms) or rms > 0.20:
        return None
    return _DepthCalibration(float(scale), float(offset), time.monotonic())


def _depth_plane_pose(
    depth_map, calibration: _DepthCalibration, image_points, camera_info
) -> tuple[np.ndarray, np.ndarray, float] | None:
    """The board plane from calibrated depth inside the outline: rotation, centroid (camera
    frame, m) and median distance of the points from the plane (m)."""
    mask = np.zeros(depth_map.shape, np.uint8)
    cv2.fillConvexPoly(mask, np.round(image_points).astype(np.int32), 255)
    ys, xs = np.nonzero(mask)
    if len(xs) > 700:
        picks = np.linspace(0, len(xs) - 1, 700, dtype=int)
        xs, ys = xs[picks], ys[picks]
    depth_m = calibration.scale * depth_map[ys, xs] + calibration.offset
    valid = np.isfinite(depth_m) & (depth_m > 0.05) & (depth_m < 20.0)
    if np.count_nonzero(valid) < MIN_DEPTH_CALIBRATION_PIXELS:
        return None
    xs, ys, depth_m = xs[valid], ys[valid], depth_m[valid]
    k = geometry.camera_matrix(camera_info)
    points = np.column_stack(
        ((xs - k[0, 2]) * depth_m / k[0, 0], (ys - k[1, 2]) * depth_m / k[1, 1], depth_m)
    )
    centroid = np.median(points, axis=0)
    normal = np.linalg.svd(points - centroid)[2][-1]
    # Into the board, away from the camera, as PnP's board z (see _heading_yaw_deg)
    if normal @ centroid < 0.0:
        normal *= -1.0
    residual = float(np.median(np.abs((points - centroid) @ normal)))
    if residual > 0.08:
        return None
    # A plane has no in-plane axes; take x along the camera's x
    x_axis = np.array([1.0, 0.0, 0.0]) - normal * normal[0]
    if np.linalg.norm(x_axis) < 1e-5:
        return None
    x_axis /= np.linalg.norm(x_axis)
    rotation = np.column_stack((x_axis, np.cross(normal, x_axis), normal))
    return cv2.Rodrigues(rotation)[0], centroid, residual


def _quaternion_from_rvec(rvec: np.ndarray) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation vector."""
    r = cv2.Rodrigues(rvec)[0]
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, s / 4)
    index = int(np.argmax(np.diag(r)))
    if index == 0:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        return (s / 4, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s)
    if index == 1:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        return ((r[0, 1] + r[1, 0]) / s, s / 4, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s)
    s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
    return ((r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, s / 4, (r[1, 0] - r[0, 1]) / s)


def _image_roll_deg(rvec: np.ndarray) -> float:
    """The board's twist in the image plane, for telemetry, not steering."""
    r = cv2.Rodrigues(rvec)[0]
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def _heading_yaw_deg(rvec: np.ndarray, tvec: np.ndarray) -> float | None:
    """The vehicle yaw (degrees, counter-clockwise positive) that squares the camera to the board.

    The board's normal is its rotation's third column, in the optical frame (x right, y down,
    z forward); its x/z angle is the horizontal error. Image right is starboard, hence the sign.
    """
    normal = cv2.Rodrigues(rvec)[0][:, 2]
    # The normal points into the board, away from the camera along the ray to its centre. Its z
    # alone is no test: a board off to one side, seen obliquely, can have a negative one.
    if float(normal @ np.reshape(tvec, 3)) < 0.0:
        normal = -normal
    if math.hypot(normal[0], normal[2]) <= 1e-6:
        return None
    return -math.degrees(math.atan2(normal[0], normal[2]))


def _aim_hole_bearings(rvec: np.ndarray, tvec: np.ndarray) -> tuple[float, float] | None:
    """Bearings (optical convention, as Detection's) of the aim hole on the solved board."""
    aim = np.array([AIM_HOLE_BOARD_X_M, AIM_HOLE_BOARD_Y_M, 0.0])
    point = cv2.Rodrigues(rvec)[0] @ aim + tvec.reshape(3)
    if point[2] <= 1e-6:
        return None
    return math.atan2(point[0], point[2]), math.atan2(point[1], point[2])


def _hole_board_coordinate(
    hole, rvec: np.ndarray, tvec: np.ndarray, camera_info
) -> np.ndarray | None:
    """A hole's box centre, back-projected to the solved board plane (board x, y in m)."""
    try:
        normalized = geometry.undistort_normalized(*_box_center(hole), camera_info)
        rotation = cv2.Rodrigues(rvec)[0]
        homography = np.column_stack((rotation[:, 0], rotation[:, 1], tvec.reshape(3)))
        board = np.linalg.solve(homography, np.array([*normalized, 1.0]))
    except (ValueError, cv2.error, np.linalg.LinAlgError):
        return None
    if abs(float(board[2])) <= 1e-8:
        return None
    return board[:2] / board[2]


def _mark_aim_hole(board, candidates, pose=None, camera_info=None) -> None:
    """Relabel as TORP_AIM_HOLE_CLASS_ID the confident hole nearest the aim point: in board
    coordinates with a board pose, else in the box (its upper centre)."""
    holes = [
        candidate
        for candidate in candidates
        if _class_id(candidate) == TORP_HOLE_CLASS_ID and _score(candidate) >= MIN_AIM_HOLE_SCORE
    ]
    if not holes:
        return
    target = None
    if pose is not None:
        aim = np.array([AIM_HOLE_BOARD_X_M, AIM_HOLE_BOARD_Y_M])
        on_board = [(hole, _hole_board_coordinate(hole, *pose, camera_info)) for hole in holes]
        on_board = [(hole, point) for hole, point in on_board if point is not None]
        if on_board:
            target = min(on_board, key=lambda pair: float(np.linalg.norm(pair[1] - aim)))[0]
    if target is None:
        bbox = board.detection.bbox
        size = np.array([max(bbox.size_x, 1.0), max(bbox.size_y, 1.0)])
        target = min(
            holes,
            key=lambda hole: float(
                np.linalg.norm((_box_center(hole) - _box_center(board)) / size - (0.0, -0.30))
            ),
        )
    target.detection.results[0].hypothesis.class_id = TORP_AIM_HOLE_CLASS_ID
    target.extra.append(KeyValue(key="hole_role", value="aim_target"))


def _set_pose(board, rvec: np.ndarray, translation: np.ndarray) -> None:
    board.pose.position.x, board.pose.position.y, board.pose.position.z = map(float, translation)
    orientation = board.pose.orientation
    orientation.x, orientation.y, orientation.z, orientation.w = _quaternion_from_rvec(rvec)
    # Steering is horizontal: bear and range on the board centre in the x/z plane, so the
    # camera's height relative to the board does not read as range
    board.bearing_horizontal = math.atan2(translation[0], translation[2])
    board.distance_m = math.hypot(translation[0], translation[2])
    board.pose_valid = True


def _heading_extra(rvec: np.ndarray, tvec: np.ndarray) -> list[KeyValue]:
    heading = _heading_yaw_deg(rvec, tvec)
    if heading is None:
        return [KeyValue(key="heading_status", value="unavailable")]
    return [
        KeyValue(key="heading_yaw_deg", value=f"{heading:.6f}"),
        KeyValue(key="heading_status", value="available"),
    ]


@register_post_processor("torp")
@register_post_processor("torp_find")
class TorpPostProcessor(TaskPostProcessor):
    """Board pose, heading and aim point by PnP; marks the hole to fire at."""

    def __init__(self):
        self._depth_calibration: _DepthCalibration | None = None

    def process(self, detections, rgb_image, depth_image, camera_info, model_masks=None):
        # Holes stay in the output: the firing stage aims at a detected hole, not the board centre
        others = [det for det in detections.detections if _class_id(det) != TORP_CLASS_ID]
        masks = model_masks or ()
        boards = [
            (det, masks[index] if index < len(masks) else None)
            for index, det in enumerate(detections.detections)
            if _class_id(det) == TORP_CLASS_ID
        ]
        boards.sort(key=lambda pair: _score(pair[0]), reverse=True)
        best_pose = None

        for index, (board, mask) in enumerate(boards):
            corner_source, corners = "depth_relative", depth_board_corners(depth_image, board)
            if corners is None:
                corner_source, corners = "yolo_segment", yolo_mask_corners(mask, board)
            if corners is None:
                corner_source, corners = "opencv_contour", opencv_board_corners(rgb_image, board)
            if corners is None:
                corner_source, corners = "bbox_fallback", (bbox_corners(board), 0.0)
            image_points, corner_quality = corners
            # Only a whole outline shows which way the board faces: a box's corners are not the
            # board's, nor are those of an outline the image edge cuts off
            outline = corner_source != "bbox_fallback" and not _cut_off(board, rgb_image)

            try:
                rvec, tvec, reprojection_error = solve_torp_pose(image_points, camera_info)
            except (ValueError, cv2.error) as error:
                self._depth_plane_fallback(board, error, depth_image, image_points, camera_info)
                continue

            translation = tvec.reshape(3)
            _set_pose(board, rvec, translation)
            # How well the corners constrain the solve, so a near-degenerate fit (box corners,
            # say) can be trusted less than a good one
            try:
                covariance = geometry.pose_covariance(_OBJECT_CORNERS, rvec, tvec, camera_info)
                std_devs = ",".join(
                    f"{v:.5f}" for v in np.sqrt(np.clip(np.diag(covariance), 0, None))
                )
            except np.linalg.LinAlgError:
                std_devs = "unavailable"
            board.extra.append(KeyValue(key="pose_std_dev_txyz_rxyz", value=std_devs))
            # Only the best board's outline calibrates depth: without one the solved plane is made
            # up, and a lesser board must not replace the best one's calibration
            if index == 0 and outline:
                board.extra.extend(
                    self._calibrate_depth(depth_image, image_points, rvec, tvec, camera_info)
                )
            board.extra.extend(
                [
                    KeyValue(key="image_roll_deg", value=f"{_image_roll_deg(rvec):.6f}"),
                    KeyValue(key="pnp_reprojection_error_px", value=f"{reprojection_error:.3f}"),
                    KeyValue(
                        key="bbox_corners_px",
                        value=";".join(f"{x:.1f},{y:.1f}" for x, y in image_points),
                    ),
                    KeyValue(key="corner_source", value=corner_source),
                    KeyValue(key="corner_quality", value=f"{corner_quality:.3f}"),
                    # The pose's orientation and image roll are not a vehicle-yaw correction;
                    # heading_yaw_deg is
                    KeyValue(key="pose_semantics", value="board_normal_heading"),
                ]
            )
            if not outline:
                board.extra.append(KeyValue(key="heading_status", value="unavailable"))
                board.extra.append(KeyValue(key="aim_point_status", value="unavailable"))
            else:
                board.extra.extend(_heading_extra(rvec, tvec))
                aim = _aim_hole_bearings(rvec, tvec)
                if aim is None:
                    board.extra.append(KeyValue(key="aim_point_status", value="unavailable"))
                else:
                    board.extra.extend(
                        [
                            KeyValue(key="aim_bearing_horizontal", value=f"{aim[0]:.6f}"),
                            KeyValue(key="aim_bearing_vertical", value=f"{aim[1]:.6f}"),
                            KeyValue(key="aim_point_status", value="available"),
                        ]
                    )
            if index == 0:
                best_pose = (rvec, tvec)

        if boards:
            _mark_aim_hole(boards[0][0], others, best_pose, camera_info)
        others.sort(key=_score, reverse=True)
        detections.detections = [board for board, _ in boards] + others
        return detections

    def _calibrate_depth(self, depth_map, image_points, rvec, tvec, camera_info) -> list[KeyValue]:
        """Refit the depth calibration to this solve, and how well depth agrees with it."""
        if depth_map is None:
            return []
        calibration = _fit_depth_calibration(depth_map, image_points, rvec, tvec, camera_info)
        if calibration is None:
            return [KeyValue(key="depth_status", value="calibration_rejected")]
        self._depth_calibration = calibration
        mask = np.zeros(depth_map.shape, dtype=np.uint8)
        cv2.fillConvexPoly(mask, np.round(image_points).astype(np.int32), 255)
        board_depth = calibration.scale * depth_map[mask > 0] + calibration.offset
        board_depth = board_depth[np.isfinite(board_depth)]
        if not len(board_depth):
            return []
        depth_range = float(np.median(board_depth))
        return [
            KeyValue(key="depth_status", value="validated"),
            KeyValue(key="depth_pose_source", value="pnp_validated"),
            KeyValue(key="depth_range_m", value=f"{depth_range:.4f}"),
            KeyValue(
                key="depth_pnp_range_residual_m", value=f"{depth_range - float(tvec[2, 0]):.4f}"
            ),
        ]

    def _depth_plane_fallback(self, board, error, depth_map, image_points, camera_info) -> None:
        """When PnP fails, the board plane from a recent depth calibration, else no pose."""
        calibration = self._depth_calibration
        age = time.monotonic() - calibration.created_at if calibration else math.inf
        plane = (
            _depth_plane_pose(depth_map, calibration, image_points, camera_info)
            if depth_map is not None and age <= DEPTH_CALIBRATION_MAX_AGE_S
            else None
        )
        board.extra.append(KeyValue(key="pose_error", value=str(error)))
        if plane is None:
            board.pose_valid = False
            board.distance_m = math.nan
            board.extra.append(KeyValue(key="depth_status", value="unavailable"))
            return
        rvec, centroid, residual = plane
        _set_pose(board, rvec, centroid)
        board.extra.extend(
            [
                KeyValue(key="depth_status", value="fallback"),
                KeyValue(key="depth_pose_source", value="cached_pnp_calibrated_plane"),
                KeyValue(key="depth_calibration_age_s", value=f"{age:.3f}"),
                KeyValue(key="depth_plane_residual_m", value=f"{residual:.4f}"),
                KeyValue(key="pose_semantics", value="board_normal_heading"),
                *_heading_extra(rvec, centroid),
                # A plane has no in-plane axes, so where on it the aim hole is is unknown
                KeyValue(key="aim_point_status", value="unavailable"),
            ]
        )
