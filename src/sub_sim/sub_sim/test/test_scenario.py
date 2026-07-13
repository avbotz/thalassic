import math
from pathlib import Path

import pytest

from sub_sim.randomize_locs import randomize_scenario_locations


SCENARIO = Path(__file__).parents[1] / "scenarios" / "woollett.scn.j2"


def render_and_capture_yaw(tmp_path, robot_yaw=None, seed=1):
    poses = []

    def render_robot(**pose):
        poses.append(pose)
        robot = tmp_path / "robot.scn"
        robot.write_text("<scenario/>")
        return robot.as_posix()

    rendered = randomize_scenario_locations(
        SCENARIO,
        DX=0.0,
        DY=0.0,
        DZ=0.0,
        DYAW=0.0,
        ROBOT_YAW=robot_yaw,
        render_robot=render_robot,
        seed=seed,
    )
    rendered.unlink()
    assert len(poses) == 1
    return poses[0]["yaw"]


@pytest.mark.parametrize("seed", range(10))
def test_default_robot_heading_faces_the_gate(tmp_path, seed):
    assert render_and_capture_yaw(tmp_path, seed=seed) == math.pi


def test_robot_heading_override_is_preserved(tmp_path):
    assert render_and_capture_yaw(tmp_path, robot_yaw=0.75) == 0.75
