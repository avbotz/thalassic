"""The gate post-processor's ranges and facing, from icon boxes projected through a pinhole camera."""

import math

import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo
from sub_vision.post_processors import gate
from sub_vision_interfaces.msg import Detection, DetectionArray
from vision_msgs.msg import ObjectHypothesisWithPose

WIDTH, HEIGHT = 1920, 1080
FOCAL = (WIDTH / 2) / math.tan(math.radians(70.4) / 2)  # the simulator's front camera
PANEL_M = 0.311
# The panels' centres across the gate, as on the simulator's gate (m, right positive).
PANEL_CENTRES = {"left": -0.7175, "right": 0.8695}
# Each icon's extent on its panel, (left, right, top, bottom) as fractions of the panel's side,
# from gate.png: compass and hammer and wrench on the Survey & Repair panel, buoy and SOS on the
# Search & Rescue one.
ICONS = {
    "left": [
        (gate.COMPASS_CLASS_ID, (0.042, 0.536, 0.037, 0.531)),
        (gate.HAMMER_AND_WRENCH_CLASS_ID, (0.457, 0.951, 0.471, 0.950)),
    ],
    "right": [
        (gate.BUOY_CLASS_ID, (0.476, 0.965, 0.036, 0.562)),
        (gate.SOS_CLASS_ID, (0.036, 0.531, 0.473, 0.967)),
    ],
}


def camera_info() -> CameraInfo:
    info = CameraInfo()
    info.width, info.height = WIDTH, HEIGHT
    info.k = [FOCAL, 0.0, WIDTH / 2, 0.0, FOCAL, HEIGHT / 2, 0.0, 0.0, 1.0]
    return info


def detections(distance: float, turn_deg: float, sides=("left", "right")) -> DetectionArray:
    """The icons' boxes with the camera `distance` in front of the gate's middle, level with the
    panels, and the gate turned `turn_deg` counter-clockwise from facing the camera, seen from
    above: the turn that squares the camera to it is then `turn_deg`."""
    turn = math.radians(turn_deg)
    # Right along the gate, in (x right, z ahead): turned that way, its right end is further.
    across = np.array([math.cos(turn), math.sin(turn)])
    array = DetectionArray()
    array.task = "gate"
    for side in sides:
        for class_id, (left, right, top, bottom) in ICONS[side]:
            corners = []
            for u, v in ((left, top), (right, top), (right, bottom), (left, bottom)):
                x, z = (PANEL_CENTRES[side] + (u - 0.5) * PANEL_M) * across + [0.0, distance]
                y = (v - 0.5) * PANEL_M
                corners.append((WIDTH / 2 + FOCAL * x / z, HEIGHT / 2 + FOCAL * y / z))
            (x0, y0), (x1, y1) = np.min(corners, axis=0), np.max(corners, axis=0)
            detection = Detection()
            detection.detection.bbox.center.position.x = float(x0 + x1) / 2
            detection.detection.bbox.center.position.y = float(y0 + y1) / 2
            detection.detection.bbox.size_x = float(x1 - x0)
            detection.detection.bbox.size_y = float(y1 - y0)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(class_id)
            hypothesis.hypothesis.score = 0.9
            detection.detection.results.append(hypothesis)
            array.detections.append(detection)
    return array


def left_target(array: DetectionArray) -> tuple[Detection, dict[str, str]]:
    out = gate.GatePostProcessor().process(array, None, None, camera_info())
    target = out.detections[0]
    assert target.detection.results[0].hypothesis.class_id == str(gate.LEFT_GATE_TARGET_CLASS_ID)
    return target, {kv.key: kv.value for kv in target.extra}


@pytest.mark.parametrize("turn_deg", [-40.0, -15.0, 0.0, 15.0, 40.0])
def test_facing_is_the_turn_that_squares_the_camera(turn_deg):
    _, extra = left_target(detections(3.0, turn_deg))
    assert float(extra["heading_yaw_deg"]) == pytest.approx(turn_deg, abs=1.0)
    assert float(extra["panel_spacing_m"]) == pytest.approx(1.587, abs=0.03)


@pytest.mark.parametrize("turn_deg", [0.0, 30.0])
def test_a_turned_gate_does_not_throw_the_range(turn_deg):
    # The left panel's centre is distance - sin(turn) * its offset across the gate away.
    target, _ = left_target(detections(3.0, turn_deg))
    expected = 3.0 - math.sin(math.radians(turn_deg)) * -PANEL_CENTRES["left"]
    assert target.distance_m == pytest.approx(expected, rel=0.02)


def test_one_panel_gives_no_facing():
    _, extra = left_target(detections(3.0, 0.0, sides=("left",)))
    assert "heading_yaw_deg" not in extra
    assert extra["two_panel_view"] == "false"


def test_panels_too_close_together_give_no_facing():
    assert gate._facing_yaw_deg(-0.1, 3.0, 0.1, 3.0)[0] is None
