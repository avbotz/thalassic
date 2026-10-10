"""Gate role-icon grouping and left-lane targeting.

The gate task runs the front-facing-camera model (``ffc_rs_26``, see
``model_manager.TASK_MODELS``), which detects the two role icons printed on
each gate panel, not the PVC gate itself. This processor groups the icons into
the two panels, reads the role shown on the left one, and adds one synthetic
class-8 detection at the centre of that left opening, so the mission aligns to
one target whichever role the course puts on the left.
"""

from __future__ import annotations

import copy

import numpy as np
from diagnostic_msgs.msg import KeyValue

from sub_vision import geometry
from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor

BUOY_CLASS_ID = 1
COMPASS_CLASS_ID = 2
HAMMER_AND_WRENCH_CLASS_ID = 5
SOS_CLASS_ID = 7
LEFT_GATE_TARGET_CLASS_ID = 8
# Each icon fills about 49% of the 0.311 m square simulator panel. Ranging from
# each icon's width still works when only one of the two survives detection.
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
    """Split icons, ordered left to right, at the widest gap between them.

    The two icons on one panel can themselves be apart, so a gap only counts as
    the space between panels when it is well over a typical icon's width.
    """
    ordered = sorted(icons, key=_center_x)
    if len(ordered) < 2:
        return ordered, []

    centers = np.array([_center_x(det) for det in ordered])
    widths = np.array([max(1.0, float(det.detection.bbox.size_x)) for det in ordered])
    gaps = np.diff(centers)
    split = int(np.argmax(gaps))
    if float(gaps[split]) < max(24.0, 1.25 * float(np.median(widths))):
        return ordered, []
    return ordered[: split + 1], ordered[split + 1 :]


def _role_for_panel(panel) -> tuple[str, float] | None:
    """The role whose icons on the panel score highest in total, and that total."""
    scores = {
        role: sum(_score(det) for det in panel if _class_id(det) in class_ids)
        for role, class_ids in ROLE_CLASS_IDS.items()
    }
    role = max(scores, key=scores.get)
    if scores[role] <= 0.0:
        return None
    return role, scores[role]


def _panel_center_x(panel) -> float:
    weights = [max(_score(det), 1e-3) for det in panel]
    return float(np.average([_center_x(det) for det in panel], weights=weights))


def _panel_bounds(panel) -> tuple[float, float, float, float]:
    boxes = [det.detection.bbox for det in panel]
    return (
        min(b.center.position.x - b.size_x / 2.0 for b in boxes),
        min(b.center.position.y - b.size_y / 2.0 for b in boxes),
        max(b.center.position.x + b.size_x / 2.0 for b in boxes),
        max(b.center.position.y + b.size_y / 2.0 for b in boxes),
    )


@register_post_processor("gate")
class GatePostProcessor(TaskPostProcessor):
    """Keep the role icons and add the left-lane target ahead of them."""

    def process(self, detections, rgb_image, depth_image, camera_info, model_masks=None):
        icons = sorted(
            (det for det in detections.detections if _class_id(det) in CLASS_NAMES),
            key=_center_x,
        )
        detections.detections = icons
        if not icons:
            return detections

        left_panel, right_panel = _split_panels(icons)
        k = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
        fx, cx, cy = float(k[0, 0]), float(k[0, 2]), float(k[1, 2])
        if fx <= 0.0 or k[1, 1] <= 0.0:  # uncalibrated: no bearing or range
            return detections

        # With both panels in view the left one is unambiguous. With one, keep
        # it only while it is on or near the left half, so a lone right panel
        # is never taken for the left lane when the gate first comes into view.
        two_panel_view = bool(right_panel)
        image_width = rgb_image.shape[1] if rgb_image is not None else 2.0 * cx
        target_x = _panel_center_x(left_panel)
        if not two_panel_view and target_x > cx + 0.05 * image_width:
            return detections

        role_result = _role_for_panel(left_panel)
        if role_result is None:
            return detections
        role, role_score = role_result

        # Each panel hangs over its lane: aim below its centre, level with the camera.
        target = copy.deepcopy(max(left_panel, key=_score))
        hypothesis = target.detection.results[0].hypothesis
        hypothesis.class_id = str(LEFT_GATE_TARGET_CLASS_ID)
        hypothesis.score = float(min(1.0, role_score / len(left_panel)))
        x0, y0, x1, y1 = _panel_bounds(left_panel)
        bbox = target.detection.bbox
        bbox.center.position.x = target_x
        bbox.center.position.y = cy
        bbox.size_x = max(1.0, x1 - x0)
        bbox.size_y = max(1.0, y1 - y0)
        target.bearing_horizontal = geometry.bearings_from_pixel(target_x, cy, camera_info)[0]
        target.bearing_vertical = 0.0
        ranges = [
            ICON_WIDTH_M * fx / float(det.detection.bbox.size_x)
            for det in left_panel
            if det.detection.bbox.size_x > 1.0
        ]
        target.distance_m = float(np.median(ranges)) if ranges else float("nan")
        target.pose_valid = False

        left_ids = [_class_id(det) for det in left_panel]
        right_ids = [_class_id(det) for det in right_panel]
        target.extra = [
            KeyValue(key="gate_target", value="left_opening"),
            KeyValue(key="gate_role", value=role),
            KeyValue(key="gate_role_label", value=ROLE_LABELS[role]),
            KeyValue(key="left_icon_classes", value=",".join(map(str, left_ids))),
            KeyValue(key="left_icon_names", value=",".join(CLASS_NAMES[i] for i in left_ids)),
            KeyValue(key="right_icon_classes", value=",".join(map(str, right_ids))),
            KeyValue(key="right_icon_names", value=",".join(CLASS_NAMES[i] for i in right_ids)),
            KeyValue(key="two_panel_view", value=str(two_panel_view).lower()),
            KeyValue(key="estimated_distance_m", value=f"{target.distance_m:.3f}"),
            KeyValue(key="aim_bearing_horizontal", value=f"{target.bearing_horizontal:.6f}"),
            KeyValue(key="aim_bearing_vertical", value="0.000000"),
            KeyValue(key="pose_semantics", value="left_gate_lane"),
        ]
        detections.detections.insert(0, target)
        return detections
