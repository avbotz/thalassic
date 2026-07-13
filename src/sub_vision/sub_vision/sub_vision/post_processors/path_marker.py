"""Path-marker centering and orientation estimation for the down camera."""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
from diagnostic_msgs.msg import KeyValue
import numpy as np
from vision_msgs.msg import ObjectHypothesisWithPose

from sub_vision_interfaces.msg import Detection
from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor


MIN_LONG_AXIS_PX = 18.0
MIN_GEOMETRIC_AXIS_RATIO = 2.4
MIN_COMPONENT_AREA_PX = 80
MAX_COMPONENT_AREA_FRACTION = 0.75
ROI_PADDING_FRACTION = 0.18
MAX_ESTIMATOR_DISAGREEMENT_DEG = 12.0
FULL_FRAME_CENTER_FRACTION = 0.78


@dataclass(frozen=True)
class _AxisEstimate:
    yaw_deg: float
    confidence: float
    source: str


def _normalize_axial_deg(angle: float) -> float:
    """Normalize an undirected line angle to [-90, 90)."""
    return (angle + 90.0) % 180.0 - 90.0


def _axial_difference_deg(first: float, second: float) -> float:
    return abs(_normalize_axial_deg(first - second))


def _vector_yaw_deg(axis_u: float, axis_v: float) -> float:
    """Convert an image line vector to relative ROS yaw."""
    if axis_v > 0.0 or (abs(axis_v) < 1e-9 and axis_u < 0.0):
        axis_u, axis_v = -axis_u, -axis_v
    return _normalize_axial_deg(math.degrees(-math.atan2(axis_u, -axis_v)))


def _bbox_bounds(detection, width: int, height: int) -> tuple[int, int, int, int] | None:
    bbox = detection.detection.bbox
    values = (
        bbox.center.position.x,
        bbox.center.position.y,
        bbox.size_x,
        bbox.size_y,
    )
    if not all(math.isfinite(value) for value in values) or bbox.size_x <= 0.0 or bbox.size_y <= 0.0:
        return None
    half_width = bbox.size_x / 2.0
    half_height = bbox.size_y / 2.0
    x0 = max(0, min(width, int(math.floor(bbox.center.position.x - half_width))))
    y0 = max(0, min(height, int(math.floor(bbox.center.position.y - half_height))))
    x1 = max(0, min(width, int(math.ceil(bbox.center.position.x + half_width))))
    y1 = max(0, min(height, int(math.ceil(bbox.center.position.y + half_height))))
    return (x0, y0, x1, y1) if x1 - x0 >= 3 and y1 - y0 >= 3 else None


def _padded_bounds(
    bounds: tuple[int, int, int, int], width: int, height: int
) -> tuple[int, int, int, int]:
    x0, y0, x1, y1 = bounds
    pad = max(4, int(round(max(x1 - x0, y1 - y0) * ROI_PADDING_FRACTION)))
    return max(0, x0 - pad), max(0, y0 - pad), min(width, x1 + pad), min(height, y1 + pad)


def _estimate_from_points(points_uv: np.ndarray, source: str) -> _AxisEstimate | None:
    if len(points_uv) < MIN_COMPONENT_AREA_PX:
        return None
    covariance = np.cov(points_uv.astype(np.float64), rowvar=False)
    if covariance.shape != (2, 2) or not np.isfinite(covariance).all():
        return None
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    major_index = int(np.argmax(eigenvalues))
    major = float(eigenvalues[major_index])
    minor = float(eigenvalues[1 - major_index])
    if major <= 0.0 or minor <= 0.0:
        return None
    long_axis = math.sqrt(12.0 * major)
    axis_ratio = math.sqrt(major / minor)
    if long_axis < MIN_LONG_AXIS_PX or axis_ratio < MIN_GEOMETRIC_AXIS_RATIO:
        return None
    axis_u, axis_v = eigenvectors[:, major_index]
    shape_confidence = min(1.0, (axis_ratio - MIN_GEOMETRIC_AXIS_RATIO) / 3.5 + 0.35)
    size_confidence = min(1.0, long_axis / 80.0)
    return _AxisEstimate(
        _vector_yaw_deg(float(axis_u), float(axis_v)),
        float(0.55 * shape_confidence + 0.45 * size_confidence),
        source,
    )


def _component_estimates(
    binary: np.ndarray,
    expected_center: tuple[float, float],
    max_area: float,
    source: str,
) -> list[_AxisEstimate]:
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)
    estimates: list[_AxisEstimate] = []
    scale = max(binary.shape)
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        if area < MIN_COMPONENT_AREA_PX or area > max_area:
            continue
        cx, cy = centroids[index]
        center_distance = math.hypot(cx - expected_center[0], cy - expected_center[1])
        if center_distance > 0.32 * scale:
            continue
        left = int(stats[index, cv2.CC_STAT_LEFT])
        top = int(stats[index, cv2.CC_STAT_TOP])
        width = int(stats[index, cv2.CC_STAT_WIDTH])
        height = int(stats[index, cv2.CC_STAT_HEIGHT])
        if left == 0 or top == 0 or left + width >= binary.shape[1] or top + height >= binary.shape[0]:
            continue
        vu = np.column_stack(np.nonzero(labels == index))
        estimate = _estimate_from_points(vu[:, ::-1], source)
        if estimate is not None:
            center_score = max(0.0, 1.0 - center_distance / (0.32 * scale))
            estimates.append(
                _AxisEstimate(estimate.yaw_deg, min(1.0, estimate.confidence * (0.45 + 0.55 * center_score)), source)
            )
    return estimates


def _threshold_estimates(
    crop_bgr: np.ndarray, expected_center: tuple[float, float], bbox_area: float
) -> list[_AxisEstimate]:
    gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4)).apply(gray)
    kernel_size = max(3, int(round(min(crop_bgr.shape[:2]) * 0.025)) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    estimates: list[_AxisEstimate] = []
    for threshold_type, name in (
        (cv2.THRESH_BINARY_INV, "dark_contour"),
        (cv2.THRESH_BINARY, "bright_contour"),
    ):
        _, binary = cv2.threshold(gray, 0, 255, threshold_type | cv2.THRESH_OTSU)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        estimates.extend(
            _component_estimates(
                binary,
                expected_center,
                max(MIN_COMPONENT_AREA_PX + 1, bbox_area * MAX_COMPONENT_AREA_FRACTION),
                name,
            )
        )
    return estimates


def _has_competing_warm_components(crop_bgr: np.ndarray) -> bool:
    """Use colour only to veto two similarly sized warm objects in one box."""
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    warm = cv2.inRange(hsv, np.array([0, 60, 25]), np.array([55, 255, 255]))
    warm = cv2.morphologyEx(warm, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
    count, _, stats, _ = cv2.connectedComponentsWithStats(warm, connectivity=8)
    areas = sorted((int(stats[index, cv2.CC_STAT_AREA]) for index in range(1, count)), reverse=True)
    return len(areas) >= 2 and areas[0] >= MIN_COMPONENT_AREA_PX and areas[1] >= 0.25 * areas[0]


def _line_estimate(
    crop_bgr: np.ndarray,
    inner_bounds: tuple[int, int, int, int],
) -> _AxisEstimate | None:
    gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    detector = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
    detected = detector.detect(gray)[0]
    if detected is None:
        return None
    ix0, iy0, ix1, iy1 = inner_bounds
    candidates: list[tuple[float, float, float]] = []
    for line in detected.reshape(-1, 4):
        u0, v0, u1, v1 = map(float, line)
        midpoint_u = (u0 + u1) / 2.0
        midpoint_v = (v0 + v1) / 2.0
        if not (ix0 <= midpoint_u <= ix1 and iy0 <= midpoint_v <= iy1):
            continue
        du, dv = u1 - u0, v1 - v0
        length = math.hypot(du, dv)
        if length < MIN_LONG_AXIS_PX:
            continue
        candidates.append((_vector_yaw_deg(du, dv), length, math.hypot(midpoint_u - (ix0 + ix1) / 2, midpoint_v - (iy0 + iy1) / 2)))
    if not candidates:
        return None
    seed = max(candidates, key=lambda item: item[1])[0]
    agreeing = [item for item in candidates if _axial_difference_deg(item[0], seed) <= MAX_ESTIMATOR_DISAGREEMENT_DEG]
    support = sum(item[1] for item in agreeing)
    if len(agreeing) < 2 and support < 0.70 * math.hypot(ix1 - ix0, iy1 - iy0):
        return None
    sin_sum = sum(length * math.sin(math.radians(2.0 * yaw)) for yaw, length, _ in agreeing)
    cos_sum = sum(length * math.cos(math.radians(2.0 * yaw)) for yaw, length, _ in agreeing)
    yaw = _normalize_axial_deg(math.degrees(0.5 * math.atan2(sin_sum, cos_sum)))
    confidence = min(1.0, 0.35 + 0.12 * len(agreeing) + support / max(1.0, 4.0 * math.hypot(ix1 - ix0, iy1 - iy0)))
    return _AxisEstimate(yaw, confidence, "line_segments")


def _mask_estimate(mask: np.ndarray | None, bounds: tuple[int, int, int, int]) -> _AxisEstimate | None:
    if mask is None or mask.ndim != 2:
        return None
    x0, y0, x1, y1 = bounds
    roi = mask[y0:y1, x0:x1]
    if roi.size == 0:
        return None
    vu = np.column_stack(np.nonzero(roi > 0))
    if len(vu) == 0:
        return None
    return _estimate_from_points(vu[:, ::-1], "model_mask")


def _best_axis_estimate(
    image_bgr: np.ndarray, detection, model_mask: np.ndarray | None = None
) -> _AxisEstimate | None:
    if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        return None
    image_height, image_width = image_bgr.shape[:2]
    bounds = _bbox_bounds(detection, image_width, image_height)
    if bounds is None:
        return None
    x0, y0, x1, y1 = bounds
    # A box clipped by the image boundary can remove an end of the marker and
    # rotate its principal axis. Fail closed and let the mission use its
    # center-only fallback instead of issuing a bad yaw command.
    if x0 == 0 or y0 == 0 or x1 == image_width or y1 == image_height:
        return None
    padded = _padded_bounds(bounds, image_width, image_height)
    px0, py0, px1, py1 = padded
    crop = image_bgr[py0:py1, px0:px1]
    inner = (x0 - px0, y0 - py0, x1 - px0, y1 - py0)
    expected_center = ((inner[0] + inner[2]) / 2.0, (inner[1] + inner[3]) / 2.0)

    estimates: list[_AxisEstimate] = []
    mask_axis = _mask_estimate(model_mask, bounds)
    if mask_axis is not None:
        estimates.append(_AxisEstimate(mask_axis.yaw_deg, min(1.0, mask_axis.confidence + 0.2), mask_axis.source))
    estimates.extend(_threshold_estimates(crop, expected_center, float((x1 - x0) * (y1 - y0))))
    if _has_competing_warm_components(crop):
        return None
    line_axis = _line_estimate(crop, inner)
    if line_axis is not None:
        estimates.append(line_axis)
    if not estimates:
        return None

    estimates.sort(key=lambda estimate: estimate.confidence, reverse=True)
    best = estimates[0]
    agreeing = [estimate for estimate in estimates if _axial_difference_deg(estimate.yaw_deg, best.yaw_deg) <= MAX_ESTIMATOR_DISAGREEMENT_DEG]
    independent_sources = len({estimate.source for estimate in agreeing})
    if independent_sources >= 2:
        weights = [max(0.05, estimate.confidence) for estimate in agreeing]
        sin_sum = sum(weight * math.sin(math.radians(2.0 * estimate.yaw_deg)) for estimate, weight in zip(agreeing, weights))
        cos_sum = sum(weight * math.cos(math.radians(2.0 * estimate.yaw_deg)) for estimate, weight in zip(agreeing, weights))
        yaw = _normalize_axial_deg(math.degrees(0.5 * math.atan2(sin_sum, cos_sum)))
        return _AxisEstimate(yaw, min(1.0, max(estimate.confidence for estimate in agreeing) + 0.15), "+".join(sorted({estimate.source for estimate in agreeing})))
    return best if best.confidence >= 0.68 else None


def _orange_axis_yaw_deg(image_bgr: np.ndarray, detection) -> float | None:
    """Compatibility wrapper retained for existing callers/tests."""
    estimate = _best_axis_estimate(image_bgr, detection)
    return estimate.yaw_deg if estimate is not None else None


def _opencv_fallback_detection(image_bgr: np.ndarray, camera_info) -> Detection | None:
    """Find one central, closed elongated object when the box model misses."""
    height, width = image_bgr.shape[:2]
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)
    _, dark = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, kernel)
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, kernel)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(dark, connectivity=8)
    candidates: list[tuple[float, int]] = []
    center_limit_x = width * FULL_FRAME_CENTER_FRACTION / 2.0
    center_limit_y = height * FULL_FRAME_CENTER_FRACTION / 2.0
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        if area < 350 or area > 0.08 * width * height:
            continue
        left, top, component_width, component_height = map(int, stats[index, :4])
        if left <= 1 or top <= 1 or left + component_width >= width - 1 or top + component_height >= height - 1:
            continue
        cx, cy = centroids[index]
        if abs(cx - width / 2.0) > center_limit_x or abs(cy - height / 2.0) > center_limit_y:
            continue
        vu = np.column_stack(np.nonzero(labels == index))
        estimate = _estimate_from_points(vu[:, ::-1], "full_frame_contour")
        if estimate is None:
            continue
        bbox_area = component_width * component_height
        fill = area / max(1, bbox_area)
        if fill < 0.20:
            continue
        center_distance = math.hypot((cx - width / 2.0) / width, (cy - height / 2.0) / height)
        score = estimate.confidence + 0.25 * fill - 0.35 * center_distance
        candidates.append((score, index))
    if not candidates:
        return None
    _, index = max(candidates)
    left, top, component_width, component_height = map(int, stats[index, :4])
    detection = Detection()
    bbox = detection.detection.bbox
    bbox.center.position.x = float(left + component_width / 2.0)
    bbox.center.position.y = float(top + component_height / 2.0)
    bbox.size_x = float(component_width)
    bbox.size_y = float(component_height)
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = "0"
    hypothesis.hypothesis.score = 0.25
    detection.detection.results.append(hypothesis)
    fx, cx = camera_info.k[0], camera_info.k[2]
    fy, cy = camera_info.k[4], camera_info.k[5]
    if fx > 0.0 and fy > 0.0:
        detection.bearing_horizontal = float(math.atan2(bbox.center.position.x - cx, fx))
        detection.bearing_vertical = float(math.atan2(bbox.center.position.y - cy, fy))
    detection.extra.append(KeyValue(key="detection_source", value="opencv_fallback"))
    return detection


@register_post_processor("path")
@register_post_processor("path_marker")
class PathMarkerPostProcessor(TaskPostProcessor):
    """Add a robust relative-yaw setpoint to path-marker detections."""

    def process(self, detections, rgb_image, depth_image, camera_info, model_masks=None):
        if not detections.detections:
            fallback = _opencv_fallback_detection(rgb_image, camera_info)
            if fallback is not None:
                fallback.detection.header = detections.header
                detections.detections.append(fallback)

        masks = model_masks or ()
        for index, detection in enumerate(detections.detections):
            preserved = [entry for entry in detection.extra if entry.key == "detection_source"]
            detection.extra = preserved + [KeyValue(key="target", value="path_marker")]
            model_mask = masks[index] if index < len(masks) else None
            estimate = _best_axis_estimate(rgb_image, detection, model_mask)
            if estimate is None:
                detection.extra.append(KeyValue(key="orientation_status", value="unavailable"))
                continue
            detection.extra.extend(
                [
                    KeyValue(key="yaw_deg", value=f"{estimate.yaw_deg:.6f}"),
                    KeyValue(key="orientation_confidence", value=f"{estimate.confidence:.3f}"),
                    KeyValue(key="orientation_source", value=estimate.source),
                    KeyValue(key="orientation_status", value="available"),
                    KeyValue(key="pose_semantics", value="orientation"),
                ]
            )
        return detections
