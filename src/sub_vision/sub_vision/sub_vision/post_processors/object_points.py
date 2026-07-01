"""Known RoboSub object geometry used by PnP post-processors.

Coordinates are meters. Planar point arrays use image-like ordering:
top-left, bottom-left, bottom-right, top-right, with x rightward and y downward.
"""

from __future__ import annotations

import numpy as np


def _points(values) -> np.ndarray:
    return np.asarray(values, dtype=np.float64)


GATE_FRONT_OBJECT_POINTS = {
    "gate_sides_left": _points(
        [(0.0, 0.1524), (0.0, 1.3716), (0.0762, 1.3716), (0.0762, 0.1524)]
    ),
    "gate_sides_right": _points(
        [(2.9718, 0.1524), (2.9718, 1.3716), (3.048, 1.3716), (3.048, 0.1524)]
    ),
    "gate_center": _points(
        [(1.4986, 0.0), (1.4986, 0.6096), (1.5494, 0.6096), (1.5494, 0.0)]
    ),
}

GATE_BACK_OBJECT_POINTS = {
    "gate_sides_left": GATE_FRONT_OBJECT_POINTS["gate_sides_right"],
    "gate_sides_right": GATE_FRONT_OBJECT_POINTS["gate_sides_left"],
    "gate_center": GATE_FRONT_OBJECT_POINTS["gate_center"],
}

GATE_BBOX_OBJECT_POINTS = _points(
    [(0.0, 0.0), (0.0, 1.524), (3.048, 1.524), (3.048, 0.0)]
)

BIN_OBJECT_POINTS = _points(
    [(0.0, 0.0), (0.0, 0.6096), (0.3048, 0.6096), (0.3048, 0.0)]
)

SLALOM_GATE_OBJECT_POINTS = {
    "white_pole_left": _points(
        [(0.0, 0.0), (0.0, 0.9144), (0.0254, 0.9144), (0.0254, 0.0)]
    ),
    "red_pole": _points(
        [(1.5494, 0.0), (1.5494, 0.9144), (1.5748, 0.9144), (1.5748, 0.0)]
    ),
    "white_pole_right": _points(
        [(3.0988, 0.0), (3.0988, 0.9144), (3.1242, 0.9144), (3.1242, 0.0)]
    ),
}

TRASH_TABLE_OBJECT_POINTS = {
    "table": _points(
        [(-0.3048, -0.3048), (-0.3048, 0.3048), (0.3048, 0.3048), (0.3048, -0.3048)]
    ),
    "yellow_bucket": _points(
        [(-0.4888, -0.135), (-0.4888, 0.135), (-0.3048, 0.135), (-0.3048, -0.135)]
    ),
    "pink_bucket": _points(
        [(0.3048, -0.135), (0.3048, 0.135), (0.4888, 0.135), (0.4888, -0.135)]
    ),
}

SYMBOL_OBJECT_POINTS = {
    "reef_shark": _points([(0.0, 0.0), (0.0, 0.305), (0.305, 0.305), (0.305, 0.0)]),
    "sawfish": _points([(0.0, 0.0), (0.0, 0.305), (0.305, 0.305), (0.305, 0.0)]),
}

TORPEDO_OBJECT_POINTS = {
    "torpedo_1": _points([(0.0, 0.0), (0.0, 0.6096), (0.6096, 0.6096), (0.6096, 0.0)]),
    "torpedo_2": _points([(0.0, 0.0), (0.0, 0.6096), (0.6096, 0.6096), (0.6096, 0.0)]),
}
