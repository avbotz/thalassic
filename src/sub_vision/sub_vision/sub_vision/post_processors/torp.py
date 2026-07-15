"""PnP enrichment for the front-facing torpedo board."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import cv2
from diagnostic_msgs.msg import KeyValue
import numpy as np

from sub_vision import pnp
from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor


TORP_CLASS_ID = 0
TORP_HOLE_CLASS_ID = 1
TORP_AIM_HOLE_CLASS_ID = "torp_aim_hole"
TORP_BOARD_SIDE_M = 0.6096
# The simulated board has a repeatable upper-centre hole. This is expressed in
# physical board coordinates, not image coordinates, so its identity is stable
# as the vehicle translates or observes the board obliquely.
_AIM_HOLE_BOARD_X_M = 0.0
_AIM_HOLE_BOARD_Y_M = -0.18
# The detector exposes an axis-aligned bounding box, not the board's four
# perspective-correct corners.  Its corners can therefore be tens of pixels
# from the true projection on an angled board.  Keep the error as telemetry,
# but only reject clearly implausible fits.
MAX_REPROJECTION_ERROR_PX = 50.0
MIN_YOLO_MASK_QUALITY = 0.45
MIN_OPENCV_CORNER_QUALITY = 0.25
DEPTH_CALIBRATION_MAX_AGE_S = 0.5
MIN_DEPTH_CALIBRATION_PIXELS = 40


@dataclass
class _DepthCalibration:
    scale: float
    offset: float
    created_at: float


def _depth_array(depth_image) -> tuple[np.ndarray | None, float | None]:
    """Accept a depth sidecar result without coupling processors to its backend."""
    if depth_image is None:
        return None, None
    if isinstance(depth_image, np.ndarray):
        return depth_image, 0.0
    if getattr(depth_image, "current", True) is False:
        return None, None
    depth_map = getattr(depth_image, "map", None)
    age_s = getattr(depth_image, "age_s", None)
    if isinstance(depth_map, np.ndarray):
        return depth_map, float(age_s) if age_s is not None else None
    return None, None


def _board_plane_depths(image_points, rvec, tvec, camera_info) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Sample the PnP board plane at safe interior pixels, in camera-Z metres."""
    mask = np.zeros((int(camera_info.height) or 480, int(camera_info.width) or 640), dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.round(image_points).astype(np.int32), 255)
    mask = cv2.erode(mask, np.ones((9, 9), np.uint8))
    ys, xs = np.where(mask > 0)
    if len(xs) > 900:
        picks = np.linspace(0, len(xs) - 1, 900, dtype=int)
        xs, ys = xs[picks], ys[picks]
    k = _camera_matrix(camera_info)
    rotation, _ = cv2.Rodrigues(rvec)
    normal = rotation[:, 2]
    denom = normal[0] * ((xs - k[0, 2]) / k[0, 0]) + normal[1] * ((ys - k[1, 2]) / k[1, 1]) + normal[2]
    numerator = float(normal @ tvec.reshape(3))
    valid = np.abs(denom) > 1e-6
    return xs[valid], ys[valid], (numerator / denom[valid]).astype(np.float64)


def _fit_depth_calibration(depth_map, image_points, rvec, tvec, camera_info) -> _DepthCalibration | None:
    if depth_map is None or depth_map.ndim != 2:
        return None
    xs, ys, metric_z = _board_plane_depths(image_points, rvec, tvec, camera_info)
    valid = (xs >= 0) & (xs < depth_map.shape[1]) & (ys >= 0) & (ys < depth_map.shape[0])
    relative = depth_map[ys[valid], xs[valid]].astype(np.float64)
    metric_z = metric_z[valid]
    valid = np.isfinite(relative) & np.isfinite(metric_z)
    relative, metric_z = relative[valid], metric_z[valid]
    if len(relative) < MIN_DEPTH_CALIBRATION_PIXELS or np.ptp(relative) < 1e-5:
        return None
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


def _depth_plane_pose(depth_map, calibration, image_points, camera_info):
    """Fit a camera-frame board plane from calibrated relative depth."""
    xs, ys = np.where(cv2.fillConvexPoly(np.zeros(depth_map.shape, np.uint8), np.round(image_points).astype(np.int32), 255) > 0)
    # np.where returns y then x; retain a bounded, deterministic sample.
    ys, xs = xs, ys
    if len(xs) > 700:
        chosen = np.linspace(0, len(xs) - 1, 700, dtype=int)
        xs, ys = xs[chosen], ys[chosen]
    depth_m = calibration.scale * depth_map[ys, xs] + calibration.offset
    valid = np.isfinite(depth_m) & (depth_m > 0.05) & (depth_m < 20.0)
    if np.count_nonzero(valid) < MIN_DEPTH_CALIBRATION_PIXELS:
        return None
    xs, ys, depth_m = xs[valid], ys[valid], depth_m[valid]
    k = _camera_matrix(camera_info)
    points = np.column_stack(((xs - k[0, 2]) * depth_m / k[0, 0], (ys - k[1, 2]) * depth_m / k[1, 1], depth_m))
    centroid = np.median(points, axis=0)
    _, _, vh = np.linalg.svd(points - centroid)
    normal = vh[-1]
    if normal[2] < 0.0:
        normal *= -1.0
    residual = np.abs((points - centroid) @ normal)
    if float(np.median(residual)) > 0.08:
        return None
    x_axis = np.array([1.0, 0.0, 0.0])
    x_axis -= normal * float(x_axis @ normal)
    if np.linalg.norm(x_axis) < 1e-5:
        return None
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(normal, x_axis)
    rotation = np.column_stack((x_axis, y_axis, normal))
    rvec, _ = cv2.Rodrigues(rotation)
    return rvec, centroid, float(np.median(residual))

# Clockwise in the same order as the image bounding-box corners: top-left,
# bottom-left, bottom-right, top-right.  The board lies in Z=0 and its origin
# is its centre, so solvePnP's translation is the board-centre bearing/range
# used by the close-alignment controller.
_HALF_BOARD_SIDE_M = TORP_BOARD_SIDE_M / 2.0
_OBJECT_CORNERS = np.array(
    [
        [-_HALF_BOARD_SIDE_M, -_HALF_BOARD_SIDE_M, 0.0],
        [-_HALF_BOARD_SIDE_M, _HALF_BOARD_SIDE_M, 0.0],
        [_HALF_BOARD_SIDE_M, _HALF_BOARD_SIDE_M, 0.0],
        [_HALF_BOARD_SIDE_M, -_HALF_BOARD_SIDE_M, 0.0],
    ],
    dtype=np.float64,
)


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


def bbox_corners(detection) -> np.ndarray:
    """Return bbox corners: top-left, bottom-left, bottom-right, top-right."""
    bbox = detection.detection.bbox
    half_width = float(bbox.size_x) / 2.0
    half_height = float(bbox.size_y) / 2.0
    center_x = float(bbox.center.position.x)
    center_y = float(bbox.center.position.y)
    return np.array(
        [
            [center_x - half_width, center_y - half_height],
            [center_x - half_width, center_y + half_height],
            [center_x + half_width, center_y + half_height],
            [center_x + half_width, center_y - half_height],
        ],
        dtype=np.float64,
    )


def _hole_board_coordinate(hole, rvec: np.ndarray, tvec: np.ndarray, camera_info) -> np.ndarray | None:
    """Back-project a YOLO hole-box centre to the solved board plane."""
    try:
        camera_matrix = _camera_matrix(camera_info)
        distortion = np.asarray(camera_info.d, dtype=np.float64)
        image_point = np.array(
            [[[hole.detection.bbox.center.position.x, hole.detection.bbox.center.position.y]]], dtype=np.float64
        )
        normalized = cv2.undistortPoints(image_point, camera_matrix, distortion).reshape(2)
        rotation, _ = cv2.Rodrigues(rvec)
        homography = np.column_stack((rotation[:, 0], rotation[:, 1], tvec.reshape(3)))
        board_homogeneous = np.linalg.solve(homography, np.array([normalized[0], normalized[1], 1.0]))
        if abs(float(board_homogeneous[2])) <= 1e-8:
            return None
        return board_homogeneous[:2] / board_homogeneous[2]
    except (ValueError, cv2.error, np.linalg.LinAlgError):
        return None


def _mark_aim_hole(board, candidates, rvec=None, tvec=None, camera_info=None) -> None:
    """Relabel one fixed board-relative YOLO hole as the aim target."""
    holes = [
        candidate
        for candidate in candidates
        if _class_id(candidate) == TORP_HOLE_CLASS_ID and _score(candidate) >= 0.50
    ]
    if not holes:
        return
    target_position = np.array([_AIM_HOLE_BOARD_X_M, _AIM_HOLE_BOARD_Y_M])
    if rvec is not None and tvec is not None and camera_info is not None:
        projected_holes = [
            (hole, _hole_board_coordinate(hole, rvec, tvec, camera_info)) for hole in holes
        ]
        projected_holes = [(hole, point) for hole, point in projected_holes if point is not None]
        if projected_holes:
            # The PnP homography comes from the YOLO board mask; only YOLO box
            # centres are used for the holes. This stays stable under changing
            # board scale and perspective, unlike an axis-aligned bbox rule.
            target = min(projected_holes, key=lambda pair: float(np.linalg.norm(pair[1] - target_position)))[0]
        else:
            target = None
    else:
        target = None
    if target is None:
        # Safe fallback for an unavailable board pose. The normal mission path
        # requires the PnP pose before this firing stage.
        board_bbox = board.detection.bbox
        board_center = np.array([board_bbox.center.position.x, board_bbox.center.position.y], dtype=np.float64)
        board_size = np.array([max(float(board_bbox.size_x), 1.0), max(float(board_bbox.size_y), 1.0)])
        target = min(
            holes,
            key=lambda hole: float(
                np.linalg.norm(
                    (np.array([hole.detection.bbox.center.position.x, hole.detection.bbox.center.position.y]) - board_center)
                    / board_size
                    - np.array([0.0, -0.30])
                )
            ),
        )
    target.detection.results[0].hypothesis.class_id = TORP_AIM_HOLE_CLASS_ID
    target.extra.append(KeyValue(key="hole_role", value="aim_target"))


def yolo_mask_corners(mask: np.ndarray | None, detection) -> tuple[np.ndarray, float] | None:
    """Return four board corners from the model's segmentation mask.

    This performs only polygon extraction on pixels classified as ``board`` by
    YOLO. It deliberately does not threshold or otherwise search the camera
    image, keeping recognition robust to pool lighting and appearance changes.
    """
    if mask is None or mask.ndim != 2:
        return None
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 0.0:
        return None
    polygon = cv2.approxPolyDP(contour, 0.015 * perimeter, True).reshape(-1, 2).astype(np.float64)
    if len(polygon) < 4:
        return None

    # The 2025 successful PnP solution selected the two horizontal extremes
    # of the board silhouette, then ordered them vertically. A segmentation
    # polygon can include a few extra vertices around circular holes, so this
    # is more stable than requiring exactly four vertices.
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
    quad_area = abs(float(cv2.contourArea(quad.astype(np.float32))))
    area_ratio = quad_area / bbox_area
    center = np.mean(quad, axis=0)
    center_error = float(np.linalg.norm(center - np.array([bbox.center.position.x, bbox.center.position.y])))
    diagonal = math.hypot(float(bbox.size_x), float(bbox.size_y))
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
    """Extract a board quadrilateral from a fresh relative-depth map.

    The detector box supplies the object hypothesis; depth only supplies a
    local planar-consistency mask. Requiring its connected component to stay
    inside a padded ROI prevents a smooth background from silently becoming a
    board outline. Callers must enforce source-image timestamp equality before
    using this helper.
    """
    if depth_map is None or depth_map.ndim != 2 or not np.all(np.isfinite(depth_map)):
        return None
    bbox = detection.detection.bbox
    h, w = depth_map.shape
    pad_x, pad_y = int(max(8.0, 0.20 * bbox.size_x)), int(max(8.0, 0.20 * bbox.size_y))
    x0 = max(0, int(round(bbox.center.position.x - bbox.size_x / 2.0)) - pad_x)
    x1 = min(w, int(round(bbox.center.position.x + bbox.size_x / 2.0)) + pad_x)
    y0 = max(0, int(round(bbox.center.position.y - bbox.size_y / 2.0)) - pad_y)
    y1 = min(h, int(round(bbox.center.position.y + bbox.size_y / 2.0)) + pad_y)
    if x1 - x0 < 30 or y1 - y0 < 30:
        return None

    crop = depth_map[y0:y1, x0:x1].astype(np.float64)
    yy, xx = np.indices(crop.shape)
    core = (
        (xx >= crop.shape[1] // 4) & (xx < 3 * crop.shape[1] // 4)
        & (yy >= crop.shape[0] // 4) & (yy < 3 * crop.shape[0] // 4)
    )
    design = np.column_stack((xx[core], yy[core], np.ones(np.count_nonzero(core))))
    values = crop[core]
    if len(values) < 100 or np.ptp(values) < 1e-6:
        return None
    # Refit after rejecting central outliers (for example, a detected hole).
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

    labels, _, stats, centroids = cv2.connectedComponentsWithStats(candidate)
    if labels <= 1:
        return None
    center = np.array([bbox.center.position.x - x0, bbox.center.position.y - y0])
    choices = [label for label in range(1, labels) if stats[label, cv2.CC_STAT_AREA] >= 0.05 * bbox.size_x * bbox.size_y]
    if not choices:
        return None
    label = min(choices, key=lambda index: float(np.linalg.norm(centroids[index] - center)))
    left, top, width, height, _ = stats[label]
    # The padded border must contain non-board pixels, otherwise depth did not
    # identify a local object boundary and is not safe for PnP corners.
    if left <= 1 or top <= 1 or left + width >= crop.shape[1] - 1 or top + height >= crop.shape[0] - 1:
        return None
    mask = np.zeros_like(depth_map, dtype=np.uint8)
    mask[y0:y1, x0:x1][candidate == label] = 255
    return yolo_mask_corners(mask, detection)


def opencv_board_corners(rgb_image: np.ndarray | None, detection) -> tuple[np.ndarray, float] | None:
    """Recover the board quadrilateral from its coloured face inside a YOLO box.

    ``torp_find`` is a box-only model, so it has no YOLO segmentation mask.
    This is the same HSV/contour approach used by the team's earlier torpedo
    detector, constrained to the detector's box to avoid selecting pool props.
    """
    if rgb_image is None or rgb_image.ndim != 3:
        return None
    bbox = detection.detection.bbox
    image_h, image_w = rgb_image.shape[:2]
    pad_x = int(max(8.0, 0.20 * float(bbox.size_x)))
    pad_y = int(max(8.0, 0.20 * float(bbox.size_y)))
    x0 = max(0, int(round(bbox.center.position.x - bbox.size_x / 2.0)) - pad_x)
    x1 = min(image_w, int(round(bbox.center.position.x + bbox.size_x / 2.0)) + pad_x)
    y0 = max(0, int(round(bbox.center.position.y - bbox.size_y / 2.0)) - pad_y)
    y1 = min(image_h, int(round(bbox.center.position.y + bbox.size_y / 2.0)) + pad_y)
    if x1 - x0 < 20 or y1 - y0 < 20:
        return None

    # ROS OpenCV images are BGR. This broad warm/green range matches the
    # painted simulator board and the previous working detector.
    crop = rgb_image[y0:y1, x0:x1]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 30, 80], dtype=np.uint8), np.array([95, 245, 255], dtype=np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [contour for contour in contours if cv2.contourArea(contour) >= 0.05 * bbox.size_x * bbox.size_y]
    if not contours:
        return None
    hull = cv2.convexHull(np.vstack(contours))
    perimeter = cv2.arcLength(hull, True)
    quad = cv2.approxPolyDP(hull, 0.045 * perimeter, True).reshape(-1, 2).astype(np.float64)
    if len(quad) != 4:
        return None
    quad[:, 0] += x0
    quad[:, 1] += y0
    # Match _OBJECT_CORNERS: top-left, bottom-left, bottom-right, top-right.
    by_y = quad[np.argsort(quad[:, 1])]
    top = by_y[:2][np.argsort(by_y[:2, 0])]
    bottom = by_y[2:][np.argsort(by_y[2:, 0])]
    ordered = np.array([top[0], bottom[0], bottom[1], top[1]], dtype=np.float64)
    area_ratio = abs(float(cv2.contourArea(ordered.astype(np.float32)))) / max(float(bbox.size_x * bbox.size_y), 1.0)
    if area_ratio < MIN_OPENCV_CORNER_QUALITY or area_ratio > 2.0:
        return None
    return ordered, min(1.0, area_ratio)


def _camera_matrix(camera_info) -> np.ndarray:
    matrix = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError("camera intrinsics are unavailable")
    return matrix


def _quaternion_from_rvec(rvec: np.ndarray) -> tuple[float, float, float, float]:
    rotation, _ = cv2.Rodrigues(rvec)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return (
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        )
    index = int(np.argmax(np.diag(rotation)))
    if index == 0:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        return (0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale, (rotation[2, 1] - rotation[1, 2]) / scale)
    if index == 1:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        return ((rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale, (rotation[0, 2] - rotation[2, 0]) / scale)
    scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
    return ((rotation[0, 2] + rotation[2, 0]) / scale, (rotation[1, 2] + rotation[2, 1]) / scale,
            0.25 * scale, (rotation[1, 0] - rotation[0, 1]) / scale)


def _image_roll_deg(rvec: np.ndarray) -> float:
    """Return image-plane board twist for telemetry, not vehicle steering."""
    rotation, _ = cv2.Rodrigues(rvec)
    return math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))


def _heading_yaw_deg(rvec: np.ndarray) -> float | None:
    """Return the ROS yaw correction that makes the camera square to the board.

    ``solvePnP`` returns the board pose in optical coordinates (x right, y
    down, z forward).  The board's local +z normal is therefore the third
    rotation column.  Its x/z projection is the horizontal board-normal
    error; image-right is starboard, so its sign is inverted for ROS/FLU yaw.
    """
    rotation, _ = cv2.Rodrigues(rvec)
    normal = rotation[:, 2].astype(np.float64)
    # IPPE's equivalent planar representation can flip the normal. Select the
    # one facing the camera optical forward direction before extracting yaw.
    if normal[2] < 0.0:
        normal *= -1.0
    horizontal_norm = math.hypot(float(normal[0]), float(normal[2]))
    if horizontal_norm <= 1e-6:
        return None
    return -math.degrees(math.atan2(float(normal[0]), float(normal[2])))


def _aim_hole_bearings(rvec: np.ndarray, tvec: np.ndarray) -> tuple[float, float] | None:
    """Return the fixed upper-centre hole ray in camera optical coordinates."""
    rotation, _ = cv2.Rodrigues(rvec)
    point_camera = rotation @ np.array([_AIM_HOLE_BOARD_X_M, _AIM_HOLE_BOARD_Y_M, 0.0]) + tvec.reshape(3)
    if float(point_camera[2]) <= 1e-6:
        return None
    return (
        float(math.atan2(float(point_camera[0]), float(point_camera[2]))),
        float(math.atan2(float(point_camera[1]), float(point_camera[2]))),
    )


def solve_torp_pose(image_points: np.ndarray, camera_info) -> tuple[np.ndarray, np.ndarray, float]:
    """Solve board pose and return rotation, translation, and RMS pixel error."""
    camera_matrix = _camera_matrix(camera_info)
    distortion = np.asarray(camera_info.d, dtype=np.float64)
    success, rvec, tvec = cv2.solvePnP(
        _OBJECT_CORNERS,
        image_points,
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_IPPE,
    )
    if not success or float(tvec.reshape(3)[2]) <= 0.0:
        raise ValueError("PnP did not produce a forward-facing board pose")

    projected, _ = cv2.projectPoints(_OBJECT_CORNERS, rvec, tvec, camera_matrix, distortion)
    error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - image_points) ** 2, axis=1))))
    if not math.isfinite(error) or error > MAX_REPROJECTION_ERROR_PX:
        raise ValueError(f"PnP reprojection error {error:.1f}px exceeds limit")
    return rvec, tvec, error


@register_post_processor("torpedo")
@register_post_processor("torp")
@register_post_processor("torp_find")
class TorpPostProcessor(TaskPostProcessor):
    """Estimate board range from boxes and board normal from YOLO segmentation."""

    def __init__(self):
        self._depth_calibration: _DepthCalibration | None = None

    def process(self, detections, rgb_image, depth_image, camera_info, model_masks=None):
        # Keep YOLO's ``hole`` instances in the outgoing array. The board pose
        # owns the perpendicular firing geometry, while a later firing stage
        # needs a model-detected hole rather than the board centre as its ray
        # target. Do not use the RGB image to rediscover either object.
        non_boards = [det for det in detections.detections if _class_id(det) != TORP_CLASS_ID]
        boards = [
            (det, model_masks[index] if model_masks is not None and index < len(model_masks) else None)
            for index, det in enumerate(detections.detections)
            if _class_id(det) == TORP_CLASS_ID
        ]
        boards.sort(key=lambda pair: _score(pair[0]), reverse=True)
        best_board_pose = None
        depth_map, depth_age_s = _depth_array(depth_image)
        if depth_map is not None and depth_age_s is not None and depth_age_s > 0.15:
            depth_map = None

        for board_index, (board, model_mask) in enumerate(boards):
            corners = depth_board_corners(depth_map, board)
            corner_source = "depth_relative"
            if corners is None:
                corners = yolo_mask_corners(model_mask, board)
                corner_source = "yolo_segment"
            if corners is None:
                corners = opencv_board_corners(rgb_image, board)
                corner_source = "opencv_contour" if corners is not None else "bbox_fallback"
            image_points = corners[0] if corners is not None else bbox_corners(board)
            corner_quality = corners[1] if corners is not None else 0.0
            try:
                rvec, tvec, reprojection_error = solve_torp_pose(image_points, camera_info)
            except (ValueError, cv2.error) as error:
                calibration = self._depth_calibration
                calibration_age = time.monotonic() - calibration.created_at if calibration else float("inf")
                fallback = (
                    _depth_plane_pose(depth_map, calibration, image_points, camera_info)
                    if depth_map is not None and calibration is not None and calibration_age <= DEPTH_CALIBRATION_MAX_AGE_S
                    else None
                )
                if fallback is None:
                    board.pose_valid = False
                    board.distance_m = float("nan")
                    board.extra.append(KeyValue(key="pose_error", value=str(error)))
                    board.extra.append(KeyValue(key="depth_status", value="unavailable"))
                    continue
                rvec, translation, plane_residual = fallback
                board.pose.position.x = float(translation[0])
                board.pose.position.y = float(translation[1])
                board.pose.position.z = float(translation[2])
                (
                    board.pose.orientation.x, board.pose.orientation.y,
                    board.pose.orientation.z, board.pose.orientation.w,
                ) = _quaternion_from_rvec(rvec)
                board.distance_m = float(math.hypot(translation[0], translation[2]))
                board.bearing_horizontal = float(math.atan2(translation[0], translation[2]))
                board.pose_valid = True
                board.extra.extend([
                    KeyValue(key="pose_error", value=str(error)),
                    KeyValue(key="depth_status", value="fallback"),
                    KeyValue(key="depth_pose_source", value="cached_pnp_calibrated_plane"),
                    KeyValue(key="depth_calibration_age_s", value=f"{calibration_age:.3f}"),
                    KeyValue(key="depth_plane_residual_m", value=f"{plane_residual:.4f}"),
                    KeyValue(key="pose_semantics", value="board_normal_heading"),
                    KeyValue(key="heading_status", value="available"),
                    KeyValue(key="heading_yaw_deg", value=f"{_heading_yaw_deg(rvec):.6f}"),
                    # A plane alone cannot recover the board's in-plane axes,
                    # so do not pretend its fixed upper-centre hole is known.
                    KeyValue(key="aim_point_status", value="unavailable"),
                ])
                continue

            translation = tvec.reshape(3)
            board.pose.position.x = float(translation[0])
            board.pose.position.y = float(translation[1])
            board.pose.position.z = float(translation[2])
            (
                board.pose.orientation.x,
                board.pose.orientation.y,
                board.pose.orientation.z,
                board.pose.orientation.w,
            ) = _quaternion_from_rvec(rvec)
            # The controller operates in the horizontal plane.  A true PnP
            # board-centre ray is more accurate than the detector-box centre,
            # and avoids treating vertical camera/board offset as forward
            # range while closing to the firing stand-off.
            board.bearing_horizontal = float(math.atan2(translation[0], translation[2]))
            board.distance_m = float(math.hypot(translation[0], translation[2]))
            board.pose_valid = True
            # Reprojection-Jacobian covariance (Fisher information) flags a
            # poorly-conditioned corner geometry -- e.g. a near-degenerate
            # bbox_fallback quad -- so a downstream fusion filter need not
            # trust every pose_valid solve equally.
            try:
                covariance = pnp.estimate_pose_covariance(_OBJECT_CORNERS, rvec, tvec, camera_info)
                std_devs = np.sqrt(np.clip(np.diag(covariance), 0.0, None))
                board.extra.append(
                    KeyValue(
                        key="pose_std_dev_txyz_rxyz",
                        value=",".join(f"{value:.5f}" for value in std_devs),
                    )
                )
            except np.linalg.LinAlgError:
                board.extra.append(KeyValue(key="pose_std_dev_txyz_rxyz", value="unavailable"))
            calibration = _fit_depth_calibration(depth_map, image_points, rvec, tvec, camera_info)
            if calibration is not None:
                self._depth_calibration = calibration
                predicted = calibration.scale * depth_map + calibration.offset
                mask = np.zeros(depth_map.shape, dtype=np.uint8)
                cv2.fillConvexPoly(mask, np.round(image_points).astype(np.int32), 255)
                board_depth = predicted[mask > 0]
                board_depth = board_depth[np.isfinite(board_depth)]
                if len(board_depth):
                    board.extra.extend([
                        KeyValue(key="depth_status", value="validated"),
                        KeyValue(key="depth_pose_source", value="pnp_validated"),
                        KeyValue(key="depth_range_m", value=f"{float(np.median(board_depth)):.4f}"),
                        KeyValue(key="depth_pnp_range_residual_m", value=f"{float(np.median(board_depth) - translation[2]):.4f}"),
                    ])
            elif depth_map is not None:
                board.extra.append(KeyValue(key="depth_status", value="calibration_rejected"))
            # A standard YOLO box cannot determine board facing. Only the
            # model-derived segment polygon is allowed to steer perpendicular.
            heading_yaw_deg = _heading_yaw_deg(rvec) if corner_source != "bbox_fallback" else None
            aim_bearings = _aim_hole_bearings(rvec, tvec) if corner_source != "bbox_fallback" else None
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
                    # Do not let generic yaw actions interpret the PnP
                    # quaternion or image twist as a vehicle-yaw correction.
                    KeyValue(key="pose_semantics", value="board_normal_heading"),
                ]
            )
            if heading_yaw_deg is None:
                board.extra.append(KeyValue(key="heading_status", value="unavailable"))
            else:
                board.extra.extend(
                    [
                        KeyValue(key="heading_yaw_deg", value=f"{heading_yaw_deg:.6f}"),
                        KeyValue(key="heading_status", value="available"),
                    ]
                )
            if aim_bearings is None:
                board.extra.append(KeyValue(key="aim_point_status", value="unavailable"))
            else:
                board.extra.extend(
                    [
                        KeyValue(key="aim_bearing_horizontal", value=f"{aim_bearings[0]:.6f}"),
                        KeyValue(key="aim_bearing_vertical", value=f"{aim_bearings[1]:.6f}"),
                        KeyValue(key="aim_point_status", value="available"),
                    ]
                )
            if board_index == 0:
                best_board_pose = (rvec, tvec)
        if boards:
            if best_board_pose is None:
                _mark_aim_hole(boards[0][0], non_boards)
            else:
                _mark_aim_hole(boards[0][0], non_boards, *best_board_pose, camera_info)
        non_boards.sort(key=_score, reverse=True)
        detections.detections = [board for board, _ in boards] + non_boards
        return detections
