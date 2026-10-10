"""Tuning routines: timed sequences of MotionSetpoint targets for sub_control.

sub_control shapes every target into a jerk-limited trajectory within its
trajectory.* limits, so a routine only says where to go (or how fast) and for
how long; the reference, and its velocity and acceleration feedforward, are
the controller's. A routine is planned up front, relative to the pose it
starts from, as segments the dashboard sends one after another. Positions are
odom ENU (z up, so deeper is more negative) and rotations ZYX Euler angles,
as in MotionSetpoint.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from sub_control_interfaces.msg import MotionSetpoint

AXES = ("x", "y", "z", "roll", "pitch", "yaw")
X, Y, Z, ROLL, PITCH, YAW = range(6)
UNITS = ("m", "m", "m", "rad", "rad", "rad")

KEEP = MotionSetpoint.KEEP
POSITION = MotionSetpoint.POSITION
VELOCITY = MotionSetpoint.VELOCITY
HOLD = MotionSetpoint.HOLD

ROUTINES = ("step", "cruise", "hold", "square")

# Largest step [m, rad] and cruise speed [m/s, rad/s] a routine may ask for.
# Roll and pitch only step: a roll or pitch rate held for seconds would turn
# the vehicle over.
MAX_STEP = (3.0, 3.0, 1.5, 0.5, 0.5, math.pi / 2)
MAX_SPEED = (1.0, 1.0, 0.5, 0.0, 0.0, 1.0)
# [m] routines that move z keep it at least this deep.
MIN_DEPTH = 0.3


@dataclass(frozen=True)
class Segment:
    """One MotionSetpoint, held for `duration` seconds."""

    duration: float
    label: str
    mode: tuple[int, ...]
    position: tuple[float, ...] = (0.0,) * 6
    velocity: tuple[float, ...] = (0.0,) * 6

    @property
    def streamed(self) -> bool:
        """VELOCITY targets time out, so they are sent again until the segment ends."""
        return VELOCITY in self.mode


@dataclass(frozen=True)
class Routine:
    kind: str
    axis: str
    start: tuple[float, ...]  # [x, y, z, roll, pitch, yaw] it starts from
    segments: tuple[Segment, ...]
    cycles: int = 1  # the segments repeat this many times

    @property
    def duration(self) -> float:
        return sum(segment.duration for segment in self.segments)

    def segment_at(self, elapsed: float) -> int | None:
        """Index of the segment running `elapsed` seconds in, or None once it is over."""
        end = 0.0
        for index, segment in enumerate(self.segments):
            end += segment.duration
            if elapsed < end:
                return index
        return None

    def preview(self) -> list[list[float]]:
        """The [x, y, z] points one cycle moves through, for the path view; empty if it does not translate."""
        points = [list(self.start[:3])]
        for segment in self.segments[: len(self.segments) // self.cycles]:
            point = [
                segment.position[axis] if segment.mode[axis] == POSITION else points[-1][axis]
                for axis in (X, Y, Z)
            ]
            if point != points[-1]:
                points.append(point)
        return points if len(points) > 1 else []

    def public_state(self) -> dict:
        return {
            "kind": self.kind,
            "axis": self.axis,
            "start": list(self.start),
            "duration": self.duration,
            "segments": len(self.segments),
            "preview": self.preview(),
        }


def hold_at(pose: Sequence[float], duration: float = 0.0, label: str = "hold") -> Segment:
    """Every axis holds `pose`."""
    return Segment(duration, label, (POSITION,) * 6, tuple(pose))


def plan(
    kind: str,
    axis: str,
    amplitude: float,
    hold: float,
    settle: float,
    cycles: int,
    start: Sequence[float],
) -> Routine:
    """Plan a routine from `start`, the pose sub_control is holding.

    step    POSITION start + amplitude, start, start - amplitude, start on one
            axis, `hold` seconds each.
    cruise  VELOCITY +amplitude for `hold` seconds, HOLD for `settle`, then the
            same the other way, on x, y, z or yaw.
    hold    POSITION at start on every axis for `hold` seconds; push the
            vehicle and watch it come back.
    square  POSITION through the corners of a square of side `amplitude` in
            odom x/y (forward, then left), `hold` seconds at each.
    """
    if kind not in ROUTINES:
        raise ValueError(f"unknown routine: {kind}")
    start = tuple(float(value) for value in start)
    if len(start) != 6 or not all(math.isfinite(value) for value in start):
        raise ValueError("the start pose must be six finite values")
    amplitude, hold, settle = float(amplitude), float(hold), float(settle)
    if not all(math.isfinite(value) for value in (amplitude, hold, settle)):
        raise ValueError("routine values must be finite")
    cycles_value = float(cycles)
    if not math.isfinite(cycles_value) or not cycles_value.is_integer():
        raise ValueError("cycles must be a whole number")
    cycles = int(cycles_value)

    if kind == "hold":
        if not 1.0 <= hold <= 300.0:
            raise ValueError("a hold lasts 1 to 300 s")
        return Routine(kind, axis, start, (hold_at(start, hold),))

    if not 1 <= cycles <= 20:
        raise ValueError("cycles must be 1 to 20")
    if not 0.5 <= hold <= 120.0:
        raise ValueError("hold time must be 0.5 to 120 s")

    if kind == "square":
        if amplitude == 0.0 or abs(amplitude) > MAX_STEP[X]:
            raise ValueError(f"the square's side must be non-zero and at most {MAX_STEP[X]:g} m")
        corners = ((amplitude, 0.0), (amplitude, amplitude), (0.0, amplitude), (0.0, 0.0))
        segments = [
            _move(start, {X: dx, Y: dy}, hold, f"corner {corner + 1}")
            for _ in range(cycles)
            for corner, (dx, dy) in enumerate(corners)
        ]
        return Routine(kind, "xy", start, tuple(segments), cycles)

    if axis not in AXES:
        raise ValueError(f"unknown axis: {axis}")
    index = AXES.index(axis)
    unit = UNITS[index]

    if kind == "step":
        if amplitude == 0.0 or abs(amplitude) > MAX_STEP[index]:
            raise ValueError(
                f"{axis} steps must be non-zero and at most {MAX_STEP[index]:g} {unit}"
            )
        if index == Z:
            _check_depth(start[Z] + abs(amplitude))
        segments = [
            _move(start, {index: offset}, hold, f"{axis} {offset:+g} {unit}" if offset else "start")
            for _ in range(cycles)
            for offset in (amplitude, 0.0, -amplitude, 0.0)
        ]
        return Routine(kind, axis, start, tuple(segments), cycles)

    # cruise
    if MAX_SPEED[index] == 0.0:
        raise ValueError(f"{axis} does not cruise; step it instead")
    if amplitude == 0.0 or abs(amplitude) > MAX_SPEED[index]:
        raise ValueError(
            f"{axis} cruise must be non-zero and at most {MAX_SPEED[index]:g} {unit}/s"
        )
    if not 0.5 <= settle <= 60.0:
        raise ValueError("settle time must be 0.5 to 60 s")
    if index == Z:
        _check_depth(start[Z] + abs(amplitude) * hold)
    segments = []
    for _ in range(cycles):
        for speed in (amplitude, -amplitude):
            segments.append(_cruise(index, speed, hold, f"{axis} {speed:+g} {unit}/s"))
            segments.append(_stop(index, settle))
    return Routine(kind, axis, start, tuple(segments), cycles)


def _check_depth(highest_z: float) -> None:
    if highest_z > -MIN_DEPTH:
        raise ValueError(
            f"z would come within {MIN_DEPTH:g} m of the surface; start deeper or move less"
        )


def _with_xy_pair(mode: list[int]) -> tuple[int, ...]:
    """x and y always share a mode: an axis the routine does not set follows its partner.

    The partner's position stays at start and its velocity at zero.
    """
    if mode[X] == KEEP:
        mode[X] = mode[Y]
    if mode[Y] == KEEP:
        mode[Y] = mode[X]
    return tuple(mode)


def _move(
    start: tuple[float, ...], offsets: dict[int, float], duration: float, label: str
) -> Segment:
    mode = [KEEP] * 6
    position = list(start)
    for axis, offset in offsets.items():
        mode[axis] = POSITION
        position[axis] = start[axis] + offset
    return Segment(duration, label, _with_xy_pair(mode), tuple(position))


def _cruise(axis: int, speed: float, duration: float, label: str) -> Segment:
    mode = [KEEP] * 6
    velocity = [0.0] * 6
    mode[axis] = VELOCITY
    velocity[axis] = speed
    return Segment(duration, label, _with_xy_pair(mode), velocity=tuple(velocity))


def _stop(axis: int, duration: float) -> Segment:
    mode = [KEEP] * 6
    mode[axis] = HOLD
    return Segment(duration, "stop and hold", _with_xy_pair(mode))
