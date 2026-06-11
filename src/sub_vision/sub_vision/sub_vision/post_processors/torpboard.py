"""Torpboard target extraction and depth-backed aim point estimation."""

from __future__ import annotations

import math

import cv2
from diagnostic_msgs.msg import KeyValue
import numpy as np

from sub_vision import metadata
from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor


def find_red_ring_centers(image_bgr: np.ndarray) -> list[tuple[float, float, float]]:
    """Return plausible red ring centers as ``(x, y, radius)``."""
    if image_bgr.size == 0:
        return []

    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    low_red = cv2.inRange(hsv, (0, 90, 70), (12, 255, 255))
    high_red = cv2.inRange(hsv, (165, 90, 70), (179, 255, 255))
    mask = cv2.bitwise_or(low_red, high_red)
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    image_area = float(image_bgr.shape[0] * image_bgr.shape[1])
    candidates = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < max(40.0, image_area * 0.001):
            continue
        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 0.0:
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        x, y, width, height = cv2.boundingRect(contour)
        aspect = width / max(float(height), 1.0)
        if circularity < 0.45 or not 0.6 <= aspect <= 1.4:
            continue
        (cx, cy), radius = cv2.minEnclosingCircle(contour)
        candidates.append((float(cx), float(cy), float(radius)))

    return sorted(candidates, key=lambda item: (item[1], item[0]))


def select_aim_center(
    centers: list[tuple[float, float, float]],
    optical_center: tuple[float, float],
) -> tuple[float, float, float] | None:
    """Select the visible opening nearest the camera optical axis."""
    if not centers:
        return None
    ox, oy = optical_center
    return min(centers, key=lambda item: (item[0] - ox) ** 2 + (item[1] - oy) ** 2)


def find_depth_hole_centers(
    depth_mm: np.ndarray,
    bbox: tuple[int, int, int, int],
    board_distance_m: float,
) -> list[tuple[float, float, float]]:
    """Find openings as compact regions substantially behind the board plane."""
    if depth_mm.size == 0 or not np.isfinite(board_distance_m):
        return []

    x0, y0, x1, y1 = bbox
    roi = depth_mm[y0:y1, x0:x1]
    if roi.size == 0:
        return []

    threshold_mm = board_distance_m * 1000.0 + 250.0
    mask = ((roi > threshold_mm) & (roi > 0)).astype(np.uint8) * 255
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    roi_area = float(roi.shape[0] * roi.shape[1])
    centers = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if not max(30.0, roi_area * 0.002) <= area <= roi_area * 0.20:
            continue
        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 0.0:
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        _, _, width, height = cv2.boundingRect(contour)
        aspect = width / max(float(height), 1.0)
        if circularity < 0.35 or not 0.4 <= aspect <= 2.5:
            continue
        (cx, cy), radius = cv2.minEnclosingCircle(contour)
        centers.append((float(cx + x0), float(cy + y0), float(radius)))

    return sorted(centers, key=lambda item: (item[1], item[0]))


def _class_id(det) -> int:
    if not det.detection.results:
        return -1
    return int(det.detection.results[0].hypothesis.class_id)


def _score(det) -> float:
    if not det.detection.results:
        return 0.0
    return float(det.detection.results[0].hypothesis.score)


def _bbox(det, image_w: int, image_h: int) -> tuple[int, int, int, int]:
    bbox = det.detection.bbox
    return (
        max(0, int(round(bbox.center.position.x - bbox.size_x / 2.0))),
        max(0, int(round(bbox.center.position.y - bbox.size_y / 2.0))),
        min(image_w, int(round(bbox.center.position.x + bbox.size_x / 2.0))),
        min(image_h, int(round(bbox.center.position.y + bbox.size_y / 2.0))),
    )


def _model_hole_centers(detections) -> list[tuple[float, float, float]]:
    centers = []
    for det in detections:
        bbox = det.detection.bbox
        centers.append(
            (
                float(bbox.center.position.x),
                float(bbox.center.position.y),
                float(min(bbox.size_x, bbox.size_y) / 2.0),
            )
        )
    return sorted(centers, key=lambda item: (item[1], item[0]))


@register_post_processor("torpboard")
class TorpboardPostProcessor(TaskPostProcessor):
    """Use learned hole instances, with depth and RGB fallbacks."""

    def process(self, detections, rgb_image, depth_image, camera_info):
        k = np.array(camera_info.k, dtype=np.float64).reshape(3, 3)
        image_h, image_w = rgb_image.shape[:2]
        boards = [
            det for det in detections.detections if _class_id(det) == 0
        ]
        holes = [
            det for det in detections.detections if _class_id(det) == 1
        ]
        if boards:
            primary_board = max(boards, key=_score)
            x0, y0, x1, y1 = _bbox(primary_board, image_w, image_h)
            holes = [
                hole
                for hole in holes
                if x0 <= hole.detection.bbox.center.position.x <= x1
                and y0 <= hole.detection.bbox.center.position.y <= y1
            ]
            unrelated = [
                det
                for det in detections.detections
                if _class_id(det) not in (0, 1)
            ]
            detections.detections = [primary_board, *holes, *unrelated]
            boards = [primary_board]

        for det in boards:
            x0, y0, x1, y1 = _bbox(det, image_w, image_h)
            roi = rgb_image[y0:y1, x0:x1]
            board_holes = [
                hole
                for hole in holes
                if x0 <= hole.detection.bbox.center.position.x <= x1
                and y0 <= hole.detection.bbox.center.position.y <= y1
            ]
            centers = _model_hole_centers(board_holes)
            source = "model"
            if not centers:
                centers = find_depth_hole_centers(
                    depth_image, (x0, y0, x1, y1), float(det.distance_m)
                )
                source = "depth"
            if not centers:
                local_centers = find_red_ring_centers(roi)
                centers = [
                    (x + x0, y + y0, radius)
                    for x, y, radius in local_centers
                ]
                source = "rgb"

            selected = select_aim_center(centers, (k[0, 2], k[1, 2]))
            det.extra.append(KeyValue(key="target_count", value=str(len(centers))))
            det.extra.append(KeyValue(key="target_source", value=source))
            for index, (u, v, radius) in enumerate(centers):
                det.extra.append(
                    KeyValue(
                        key=f"target_{index}_pixel",
                        value=f"{u:.1f},{v:.1f},{radius:.1f}",
                    )
                )

            if selected is None:
                det.pose_valid = False
                continue

            u, v, _radius = selected
            selected_index = centers.index(selected)
            # The center of a physical opening sees the pool behind the board.
            # Use the detector's robust board-plane depth instead.
            distance_m = float(det.distance_m)
            if not np.isfinite(distance_m):
                det.pose_valid = False
                continue

            point = metadata.deproject_pixel(u, v, distance_m, k)
            det.pose.position.x = float(point[0])
            det.pose.position.y = float(point[1])
            det.pose.position.z = float(point[2])
            det.pose.orientation.w = 1.0
            det.pose_valid = True
            det.extra.append(
                KeyValue(key="selected_target", value=str(selected_index))
            )
            det.extra.append(KeyValue(key="pose_semantics", value="aim_point"))
            det.extra.append(KeyValue(key="aim_pixel", value=f"{u:.1f},{v:.1f}"))
            det.extra.append(
                KeyValue(
                    key="aim_point_m",
                    value=f"{point[0]:.4f},{point[1]:.4f},{point[2]:.4f}",
                )
            )

        # Give each learned hole a board-plane aim point as well. Its own depth
        # looks through the opening and therefore does not represent firing range.
        for hole in holes:
            u = float(hole.detection.bbox.center.position.x)
            v = float(hole.detection.bbox.center.position.y)
            containing = [
                board
                for board in boards
                if (
                    _bbox(board, image_w, image_h)[0]
                    <= u
                    <= _bbox(board, image_w, image_h)[2]
                    and _bbox(board, image_w, image_h)[1]
                    <= v
                    <= _bbox(board, image_w, image_h)[3]
                )
            ]
            if not containing:
                hole.pose_valid = False
                continue
            distance_m = float(containing[0].distance_m)
            if not np.isfinite(distance_m):
                hole.pose_valid = False
                continue
            point = metadata.deproject_pixel(u, v, distance_m, k)
            hole.distance_m = distance_m
            hole.pose.position.x = float(point[0])
            hole.pose.position.y = float(point[1])
            hole.pose.position.z = float(point[2])
            hole.pose.orientation.w = 1.0
            hole.pose_valid = True
            hole.extra.append(KeyValue(key="pose_semantics", value="aim_point"))

        return detections
