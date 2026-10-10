"""Render the pool scenario the way sim_launch.py does, without starting Stonefish."""

import math
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import jinja2
import pytest
from sub_sim.generate_robot import Pose, render_robot_scenario
from sub_sim.randomize_locs import randomize_scenario_locations
from sub_sim.robot_scenario_to_urdf import Transform, robot_scenario_to_urdf

PACKAGE = Path(__file__).parents[1]
DATA = PACKAGE / "data"
COURSES = ["A", "B1", "C1", "D"]
# The vehicle description's frames the robot template uses (sub_bringup's, so all at the
# robot's origin here).
FRAMES = [
    "dvl_link",
    "imu_link",
    "pressure_link",
    "front_camera",
    "down_camera",
    "dropper_link",
    "left_grabber_link",
    "right_grabber_link",
    "torpedo_0_link",
    "torpedo_1_link",
]


@pytest.fixture(autouse=True)
def render_into_tmp_path(tmp_path, monkeypatch):
    """The rendered scenarios go to the test's own directory, which pytest cleans up."""
    monkeypatch.setattr(tempfile, "tempdir", str(tmp_path))


def render(course="D", seed=0, **robot_options):
    """The rendered scenario, and the robot it includes rendered at each pose asked for.
    `robot_options` go to render_robot_scenario()."""
    robots = []

    def render_robot(**pose):
        robot = render_robot_scenario(
            DATA / "robots" / "marlin_v3" / "layout.scn.j2",
            **pose,
            # Both hands, so both propeller meshes are used.
            thrusters=[
                {"xyz": "0 0 0", "rpy": "0 0 0", "right": right} for right in ("true", "false") * 4
            ],
            frames={frame: Pose((0.0, 0.0, 0.0)) for frame in FRAMES},
            **robot_options,
        )
        robots.append((pose, robot))
        return robot.as_posix()

    scenario = randomize_scenario_locations(
        PACKAGE / "scenarios" / "woollett.scn.j2",
        DX=0.25,
        DY=0.25,
        DZ=0.10,
        DYAW=0.10,
        render_robot=render_robot,
        seed=seed,
        current="0.03 0.02 0.0",
        course=course,
    )
    return scenario, robots


def used_files(scn: Path, args: dict[str, str]) -> list[Path]:
    """The files a scenario uses, through its includes, with their $(arg ...) substituted.
    Stonefish looks for relative paths in the data directory."""
    text = scn.read_text()
    for name, value in args.items():
        text = text.replace(f"$(arg {name})", value)
    assert "$(arg " not in text, scn

    def resolve(path):
        return Path(path) if path.startswith("/") else DATA / path

    root = ET.fromstring(text)
    files = []
    for include in root.iter("include"):
        path = resolve(include.get("file"))
        files += [
            path,
            *used_files(path, {arg.get("name"): arg.get("value") for arg in include.iter("arg")}),
        ]
    files += [resolve(mesh.get("filename")) for mesh in root.iter("mesh")]
    files += [resolve(look.get("texture")) for look in root.iter("look") if look.get("texture")]
    return files


@pytest.mark.parametrize("course", COURSES)
def test_robot_starts_at_its_dock_facing_away_from_the_wall(course):
    _, robots = render(course)
    assert len(robots) == 1
    pose, _ = robots[0]
    assert pose["y"] == -10.25
    assert pose["yaw"] == math.pi


@pytest.mark.parametrize("course", COURSES)
@pytest.mark.parametrize("seed", range(3))
def test_every_file_the_scenario_uses_exists(course, seed):
    scenario, _ = render(course, seed)
    files = used_files(scenario, {})
    assert any("robots" in path.parts for path in files)
    assert [str(path) for path in files if not path.is_file()] == []


def test_a_seed_reproduces_its_scenario():
    def props(seed):
        # Without the robot's include, whose temporary file name differs each time.
        scenario, _ = render(seed=seed)
        return [line for line in scenario.read_text().splitlines() if "layout" not in line]

    assert props(7) == props(7)
    assert props(7) != props(8)


def test_both_droppers_have_a_ball():
    _, [(_, robot)] = render()
    glues = {glue.find("ros_service").get("topic") for glue in ET.parse(robot).iter("glue")}
    assert {f"/marlin_v3/sim/dropper_ball_{i}/glue" for i in range(2)} <= glues


@pytest.mark.parametrize("option", ["sensor_noise", "segmentation_camera", "depth_camera"])
@pytest.mark.parametrize("value", [False, True])
def test_every_robot_option_renders(option, value):
    _, [(_, robot)] = render(**{option: value})
    sensors = {sensor.get("name") for sensor in ET.parse(robot).iter("sensor")}
    assert ("seg_cam" in sensors) == (option == "segmentation_camera" and value)
    assert ("depth_camera" in sensors) == (option == "depth_camera" and value)


def test_a_variable_the_scenario_is_not_given_raises():
    """Rather than leaving the attribute it fills in empty for Stonefish to misread."""
    with pytest.raises(jinja2.UndefinedError, match="current"):
        randomize_scenario_locations(
            PACKAGE / "scenarios" / "woollett.scn.j2",
            DX=0.0,
            DY=0.0,
            DZ=0.0,
            DYAW=0.0,
            render_robot=lambda **_: "robot.scn",
            course="D",
        )


def test_the_dvl_looks_down():
    """Its beams point down base_link_ned's +z, at the pool's bottom, not up at the frame."""
    _, [(_, robot)] = render()
    [dvl] = [sensor for sensor in ET.parse(robot).iter("sensor") if sensor.get("type") == "dvl"]
    origin = dvl.find("origin")
    beams = 1.0 if dvl.find("specs").get("beam_positive_z") == "true" else -1.0
    rotation = Transform.from_xyz_rpy(
        (0.0, 0.0, 0.0), tuple(float(v) for v in origin.get("rpy").split())
    ).R
    assert beams * rotation[2][2] > 0.0


def test_the_urdf_links_every_joint_and_finds_every_mesh():
    _, [(_, robot)] = render()
    urdf = ET.parse(robot_scenario_to_urdf(robot, "marlin_v3", mesh_prefix=f"{DATA}/")).getroot()
    links = {link.get("name") for link in urdf.iter("link")}
    assert "marlin_v3/base_link_ned" in links
    for joint in urdf.iter("joint"):
        assert {joint.find(end).get("link") for end in ("parent", "child")} <= links
    meshes = [mesh.get("filename") for mesh in urdf.iter("mesh")]
    assert meshes
    assert [mesh for mesh in meshes if not Path(mesh).is_file()] == []
