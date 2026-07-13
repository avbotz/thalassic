"""PnP enrichment for the front-facing torpedo board."""

from __future__ import annotations

import math

import cv2
from diagnostic_msgs.msg import KeyValue
import numpy as np

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
class TorpPostProcessor(TaskPostProcessor):
    """Estimate board range from boxes and board normal from YOLO segmentation."""

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

        for board_index, (board, model_mask) in enumerate(boards):
            corners = yolo_mask_corners(model_mask, board)
            image_points = corners[0] if corners is not None else bbox_corners(board)
            corner_quality = corners[1] if corners is not None else 0.0
            corner_source = "yolo_segment" if corners is not None else "bbox_fallback"
            try:
                rvec, tvec, reprojection_error = solve_torp_pose(image_points, camera_info)
            except (ValueError, cv2.error) as error:
                board.pose_valid = False
                board.distance_m = float("nan")
                board.extra.append(KeyValue(key="pose_error", value=str(error)))
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
            # A standard YOLO box cannot determine board facing. Only the
            # model-derived segment polygon is allowed to steer perpendicular.
            heading_yaw_deg = _heading_yaw_deg(rvec) if corner_quality >= MIN_YOLO_MASK_QUALITY else None
            aim_bearings = _aim_hole_bearings(rvec, tvec) if corner_quality >= MIN_YOLO_MASK_QUALITY else None
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
