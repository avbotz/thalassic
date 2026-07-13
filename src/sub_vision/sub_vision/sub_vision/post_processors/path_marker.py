"""Path-marker centering and direction estimation for the down camera."""

from __future__ import annotations

import math

import cv2
from diagnostic_msgs.msg import KeyValue
import numpy as np

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor

# Stonefish shifts the marker from its texture hue (14) up to roughly 45
# underwater. Keep headroom for lighting/white-balance drift without admitting
# the blue/green pool background.
ORANGE_HUE_MAX = 55
MIN_SATURATION = 70
MIN_VALUE = 35
MIN_COMPONENT_PIXELS = 12
MIN_AXIS_STDDEV_PIXELS = 3.0
MIN_AXIS_RATIO = 2.5
MAX_SECOND_COMPONENT_RATIO = 0.25

def _bbox_bounds(detection, width: int, height: int) -> tuple[int, int, int, int]:
    bbox = detection.detection.bbox
    half_width = bbox.size_x / 2.0
    half_height = bbox.size_y / 2.0
    x0 = max(0, min(width, int(round(bbox.center.position.x - half_width))))
    y0 = max(0, min(height, int(round(bbox.center.position.y - half_height))))
    x1 = max(0, min(width, int(round(bbox.center.position.x + half_width))))
    y1 = max(0, min(height, int(round(bbox.center.position.y + half_height))))
    return x0, y0, x1, y1


def _orange_axis_yaw_deg(image_bgr: np.ndarray, detection) -> float | None:
    """Estimate a reliable marker-axis yaw, choosing the image-up end.

    The detector only localises the marker.  This routine verifies that the
    contents look like one elongated orange strip before publishing a yaw.
    It fails closed for clipped, blob-like, noisy, or colour-contaminated
    detections because a wrong 90/180-degree command is worse than no command.
    """
    image_height, image_width = image_bgr.shape[:2]
    x0, y0, x1, y1 = _bbox_bounds(detection, image_width, image_height)
    if x1 - x0 < 3 or y1 - y0 < 3 or x0 == 0 or y0 == 0 or x1 == image_width or y1 == image_height:
        return None

    crop = image_bgr[y0:y1, x0:x1]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    # The simulated marker texture is H=14 in OpenCV HSV.  Keep a generous
    # orange/yellow interval for underwater lighting while excluding blue,
    # green, and white pool clutter that otherwise rotates PCA arbitrarily.
    mask = cv2.inRange(
        hsv,
        np.array([0, MIN_SATURATION, MIN_VALUE]),
        np.array([ORANGE_HUE_MAX, 255, 255]),
    )
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    components, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if components <= 1:
        return None

    component_areas = stats[1:, cv2.CC_STAT_AREA]
    component_index = 1 + int(np.argmax(component_areas))
    component_pixels = int(stats[component_index, cv2.CC_STAT_AREA])
    if component_pixels < MIN_COMPONENT_PIXELS:
        return None
    if len(component_areas) > 1:
        second_largest = int(np.partition(component_areas, -2)[-2])
        if second_largest / component_pixels >= MAX_SECOND_COMPONENT_RATIO:
            return None
    points = np.column_stack(np.nonzero(labels == component_index))

    # np.nonzero returns (v, u). PCA's major eigenvector is the long marker
    # axis; its sign is ambiguous, so choose the image-up direction. The down
    # camera is installed with image-up aligned to vehicle-forward.
    uv = points[:, ::-1].astype(np.float64)
    covariance = np.cov(uv, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    major_index = int(np.argmax(eigenvalues))
    minor_index = 1 - major_index
    major = float(eigenvalues[major_index])
    minor = float(eigenvalues[minor_index])
    if major <= 0.0 or math.sqrt(major) < MIN_AXIS_STDDEV_PIXELS or minor <= 0.0 or major / minor < MIN_AXIS_RATIO:
        return None
    axis_u, axis_v = eigenvectors[:, major_index]
    if axis_v > 0.0:
        axis_u, axis_v = -axis_u, -axis_v

    # Positive image-right requires negative ROS/ENU yaw.
    return math.degrees(-math.atan2(axis_u, -axis_v))


@register_post_processor("path")
@register_post_processor("path_marker")
class PathMarkerPostProcessor(TaskPostProcessor):
    """Add an explicit relative-yaw setpoint to each path-marker detection."""

    def process(self, detections, rgb_image, depth_image, camera_info):
        for detection in detections.detections:
            yaw_deg = _orange_axis_yaw_deg(rgb_image, detection)
            detection.extra = [KeyValue(key="target", value="path_marker")]
            if yaw_deg is None:
                detection.extra.append(KeyValue(key="orientation_status", value="unavailable"))
                continue
            detection.extra.extend([
                KeyValue(key="yaw_deg", value=f"{yaw_deg:.6f}"),
                KeyValue(key="orientation_status", value="available"),
                KeyValue(key="pose_semantics", value="orientation"),
            ])
        return detections
