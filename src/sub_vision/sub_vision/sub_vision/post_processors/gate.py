"""Gate role-icon filtering and left-lane targeting.

The supplied front-camera model detects the two icons printed on each gate
panel, not the PVC gate itself.  This processor groups those icon detections
into the two physical panels, identifies the role shown on the left panel, and
publishes one synthetic class-8 detection at the centre of that left opening.
The mission can consequently align to one stable contract regardless of which
role the course randomizer places on the left.
"""

from __future__ import annotations

import copy
import math

from diagnostic_msgs.msg import KeyValue
import numpy as np

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor

BUOY_CLASS_ID = 1
COMPASS_CLASS_ID = 2
HAMMER_AND_WRENCH_CLASS_ID = 5
SOS_CLASS_ID = 7
LEFT_GATE_TARGET_CLASS_ID = 8
# Each artwork occupies about 49% of the 0.311 m square simulator panel.
# Using the detected icon width therefore gives a useful monocular range even
# when only one of the two role icons survives the real-to-sim domain gap.
ICON_WIDTH_M = 0.152

ROLE_CLASS_IDS = {
    "SURVEY": {COMPASS_CLASS_ID, HAMMER_AND_WRENCH_CLASS_ID},
    "SEARCH": {BUOY_CLASS_ID, SOS_CLASS_ID},
}
CLASS_NAMES = {
    BUOY_CLASS_ID: "buoy",
    COMPASS_CLASS_ID: "compass",
    HAMMER_AND_WRENCH_CLASS_ID: "hammer_and_wrench",
    SOS_CLASS_ID: "sos",
}
ROLE_LABELS = {
    "SURVEY": "Survey & Repair",
    "SEARCH": "Search & Rescue",
}


def _class_id(det) -> int:
    if not det.detection.results:
        return -1
    try:
        return int(det.detection.results[0].hypothesis.class_id)
    except ValueError:
        return -1


def _score(det) -> float:
    if not det.detection.results:
        return 0.0
    return float(det.detection.results[0].hypothesis.score)


def _center_x(det) -> float:
    return float(det.detection.bbox.center.position.x)


def _split_panels(icons) -> tuple[list, list]:
    """Split horizontally ordered icons at the largest physical-panel gap.

    The two icons printed on one square panel can be horizontally separated,
    so a gap is considered a panel boundary only when it is substantially
    wider than a typical detected icon box.
    """
    ordered = sorted(icons, key=_center_x)
    if len(ordered) < 2:
        return ordered, []

    centers = np.array([_center_x(det) for det in ordered], dtype=np.float64)
    widths = np.array(
        [max(1.0, float(det.detection.bbox.size_x)) for det in ordered],
        dtype=np.float64,
    )
    gaps = np.diff(centers)
    split_index = int(np.argmax(gaps))
    boundary_threshold = max(24.0, 1.25 * float(np.median(widths)))
    if float(gaps[split_index]) < boundary_threshold:
        return ordered, []
    return ordered[: split_index + 1], ordered[split_index + 1 :]


def _role_for_panel(panel) -> tuple[str, float] | None:
    scores = {
        role: sum(_score(det) for det in panel if _class_id(det) in class_ids)
        for role, class_ids in ROLE_CLASS_IDS.items()
    }
    role = max(scores, key=scores.get)
    if scores[role] <= 0.0:
        return None
    return role, scores[role]


def _panel_center(panel) -> tuple[float, float]:
    weights = np.array([max(_score(det), 1e-3) for det in panel], dtype=np.float64)
    x = np.array(
        [float(det.detection.bbox.center.position.x) for det in panel],
        dtype=np.float64,
    )
    y = np.array(
        [float(det.detection.bbox.center.position.y) for det in panel],
        dtype=np.float64,
    )
    return float(np.average(x, weights=weights)), float(np.average(y, weights=weights))


def _panel_bounds(panel) -> tuple[float, float, float, float]:
    x0 = min(
        det.detection.bbox.center.position.x - det.detection.bbox.size_x / 2.0
        for det in panel
    )
    y0 = min(
        det.detection.bbox.center.position.y - det.detection.bbox.size_y / 2.0
        for det in panel
    )
    x1 = max(
        det.detection.bbox.center.position.x + det.detection.bbox.size_x / 2.0
        for det in panel
    )
    y1 = max(
        det.detection.bbox.center.position.y + det.detection.bbox.size_y / 2.0
        for det in panel
    )
    return float(x0), float(y0), float(x1), float(y1)


def _extra(key: str, value: str) -> KeyValue:
    return KeyValue(key=key, value=value)


@register_post_processor("gate")
class GatePostProcessor(TaskPostProcessor):
    """Keep role icons and synthesize the always-left gate target."""

    def process(self, detections, rgb_image, depth_image, camera_info, model_masks=None):
        del depth_image, model_masks
        icons = [
            det
            for det in detections.detections
            if _class_id(det) in CLASS_NAMES
        ]
        icons.sort(key=_center_x)
        detections.detections = icons
        if not icons:
            return detections

        left_panel, right_panel = _split_panels(icons)
        if not left_panel:
            return detections

        k = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
        fx = float(k[0, 0])
        fy = float(k[1, 1])
        principal_x = float(k[0, 2])
        principal_y = float(k[1, 2])
        left_x, _ = _panel_center(left_panel)

        # With both panels visible, the left cluster is unambiguous.  If one
        # cluster remains after the vehicle starts aligning, retain it only
        # while it is on/near the left half; this prevents a lone right-side
        # icon from becoming the requested left lane at initial acquisition.
        explicit_two_panel_view = bool(right_panel)
        image_width = int(rgb_image.shape[1]) if rgb_image is not None else int(principal_x * 2.0)
        if not explicit_two_panel_view and left_x > principal_x + 0.05 * image_width:
            return detections

        role_result = _role_for_panel(left_panel)
        if role_result is None or fx <= 0.0 or fy <= 0.0:
            return detections
        role, role_score = role_result

        target = copy.deepcopy(max(left_panel, key=_score))
        target.detection.results[0].hypothesis.class_id = str(LEFT_GATE_TARGET_CLASS_ID)
        target.detection.results[0].hypothesis.score = float(
            min(1.0, role_score / max(1, len(left_panel)))
        )

        # The mean of the two icon centres is the centre of their square sign,
        # which is mounted over the corresponding swim-through lane.
        target_x, _ = _panel_center(left_panel)
        x0, y0, x1, y1 = _panel_bounds(left_panel)
        target.detection.bbox.center.position.x = target_x
        target.detection.bbox.center.position.y = principal_y
        target.detection.bbox.size_x = max(1.0, x1 - x0)
        target.detection.bbox.size_y = max(1.0, y1 - y0)
        target.bearing_horizontal = float(math.atan2(target_x - principal_x, fx))
        target.bearing_vertical = 0.0
        range_estimates = [
            ICON_WIDTH_M * fx / float(det.detection.bbox.size_x)
            for det in left_panel
            if float(det.detection.bbox.size_x) > 1.0
        ]
        target.distance_m = (
            float(np.median(range_estimates)) if range_estimates else float("nan")
        )
        target.pose_valid = False

        left_ids = [_class_id(det) for det in left_panel]
        right_ids = [_class_id(det) for det in right_panel]
        target.extra = [
            _extra("gate_target", "left_opening"),
            _extra("gate_role", role),
            _extra("gate_role_label", ROLE_LABELS[role]),
            _extra("left_icon_classes", ",".join(str(value) for value in left_ids)),
            _extra("left_icon_names", ",".join(CLASS_NAMES[value] for value in left_ids)),
            _extra("right_icon_classes", ",".join(str(value) for value in right_ids)),
            _extra("right_icon_names", ",".join(CLASS_NAMES[value] for value in right_ids)),
            _extra("two_panel_view", str(explicit_two_panel_view).lower()),
            _extra("estimated_distance_m", f"{target.distance_m:.3f}"),
            _extra("aim_bearing_horizontal", f"{target.bearing_horizontal:.6f}"),
            _extra("aim_bearing_vertical", "0.000000"),
            _extra("pose_semantics", "left_gate_lane"),
        ]
        detections.detections.insert(0, target)
        return detections
