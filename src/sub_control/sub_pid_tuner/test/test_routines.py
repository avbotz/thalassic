import math

import pytest
from sub_pid_tuner.routines import HOLD, KEEP, POSITION, VELOCITY, X, Y, Z, plan

# Holding 1.5 m deep (odom z up), facing 90 degrees left of odom x.
START = (1.0, 2.0, -1.5, 0.0, 0.0, math.pi / 2)


def test_step_goes_out_back_the_other_way_and_back():
    routine = plan("step", "z", 0.5, 4.0, 0.0, 2, START)
    assert len(routine.segments) == 8
    # One cycle's path: start, up, start, down, start.
    assert [point[2] for point in routine.preview()] == pytest.approx(
        [-1.5, -1.0, -1.5, -2.0, -1.5]
    )
    assert routine.duration == pytest.approx(32.0)
    targets = [segment.position[Z] for segment in routine.segments[:4]]
    assert targets == pytest.approx([-1.0, -1.5, -2.0, -1.5])
    for segment in routine.segments:
        assert segment.mode == (KEEP, KEEP, POSITION, KEEP, KEEP, KEEP)
        assert not segment.streamed


def test_x_and_y_always_share_a_mode():
    step = plan("step", "x", 1.0, 4.0, 0.0, 1, START).segments[0]
    assert step.mode[X] == step.mode[Y] == POSITION
    # y holds where it started.
    assert step.position[X] == pytest.approx(2.0)
    assert step.position[Y] == pytest.approx(2.0)

    cruise = plan("cruise", "y", 0.2, 5.0, 3.0, 1, START).segments
    assert cruise[0].mode[X] == cruise[0].mode[Y] == VELOCITY
    assert cruise[0].velocity[:2] == pytest.approx((0.0, 0.2))
    assert cruise[1].mode[X] == cruise[1].mode[Y] == HOLD


def test_cruise_streams_velocity_then_holds_each_way():
    routine = plan("cruise", "yaw", 0.3, 5.0, 3.0, 1, START)
    modes = [segment.mode[5] for segment in routine.segments]
    assert modes == [VELOCITY, HOLD, VELOCITY, HOLD]
    assert [segment.velocity[5] for segment in routine.segments[::2]] == pytest.approx([0.3, -0.3])
    assert routine.segments[0].streamed and not routine.segments[1].streamed
    assert routine.duration == pytest.approx(16.0)


def test_square_visits_four_corners_counter_clockwise_and_returns():
    routine = plan("square", "", 1.0, 6.0, 0.0, 1, START)
    corners = [segment.position[:2] for segment in routine.segments]
    assert corners == [
        pytest.approx((2.0, 2.0)),
        pytest.approx((2.0, 3.0)),
        pytest.approx((1.0, 3.0)),
        pytest.approx((1.0, 2.0)),
    ]
    assert routine.preview()[0] == pytest.approx([1.0, 2.0, -1.5])
    assert routine.preview()[-1] == pytest.approx([1.0, 2.0, -1.5])
    assert len(routine.preview()) == 5


def test_hold_keeps_every_axis_at_the_start():
    routine = plan("hold", "z", 0.0, 20.0, 0.0, 1, START)
    assert len(routine.segments) == 1
    assert routine.segments[0].mode == (POSITION,) * 6
    assert routine.segments[0].position == pytest.approx(START)
    assert routine.preview() == []


def test_segment_at_walks_the_segments_and_ends():
    routine = plan("step", "x", 0.5, 2.0, 0.0, 1, START)
    assert routine.segment_at(0.0) == 0
    assert routine.segment_at(2.5) == 1
    assert routine.segment_at(7.99) == 3
    assert routine.segment_at(8.0) is None


def test_routines_keep_z_away_from_the_surface():
    shallow = (0.0, 0.0, -0.6, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="surface"):
        plan("step", "z", 0.5, 4.0, 0.0, 1, shallow)
    with pytest.raises(ValueError, match="surface"):
        plan("cruise", "z", 0.1, 5.0, 3.0, 1, shallow)
    plan("step", "z", 0.3, 4.0, 0.0, 1, shallow)


def test_routines_are_bounded():
    with pytest.raises(ValueError, match="does not cruise"):
        plan("cruise", "roll", 0.1, 5.0, 3.0, 1, START)
    with pytest.raises(ValueError, match="at most"):
        plan("step", "pitch", 0.8, 5.0, 0.0, 1, START)
    with pytest.raises(ValueError, match="non-zero"):
        plan("step", "x", 0.0, 5.0, 0.0, 1, START)
    with pytest.raises(ValueError, match="whole number"):
        plan("step", "x", 0.5, 5.0, 0.0, 1.5, START)
    with pytest.raises(ValueError, match="hold time"):
        plan("step", "x", 0.5, 0.1, 0.0, 1, START)
    with pytest.raises(ValueError, match="unknown routine"):
        plan("sine", "x", 0.5, 5.0, 0.0, 1, START)
    with pytest.raises(ValueError, match="finite"):
        plan("step", "x", float("nan"), 5.0, 0.0, 1, START)
