import pytest
import yaml
from sub_pid_tuner.profile import GainProfile

# The layout of sub_bringup/config/control/gains*.yaml.
PROFILE = """\
/**:
  ros__parameters:
    control_rate_hz: 50.0
    power_limit: 0.6

    # Vehicle model.
    model:
      mass: [50.0, 60.0, 70.0]       # kg, rigid body plus added mass
      net_buoyancy: 0.0              # N

    # Feedback, [kp, ki, kd] per axis.
    gains:
      x: [7.14, 0.612, 10.2]
      yaw: [28.35, 0.9, 23.0]
      anti_windup: 5.0  # 1/s

    thruster_health: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
"""


@pytest.fixture
def path(tmp_path):
    path = tmp_path / "gains.yaml"
    path.write_text(PROFILE)
    path.chmod(0o644)
    return path


def runtime(**changes):
    values = {
        "power_limit": 0.6,
        "model.mass": [50.0, 60.0, 70.0],
        "model.net_buoyancy": 0.0,
        "gains.x": [7.14, 0.612, 10.2],
        "gains.yaw": [28.35, 0.9, 23.0],
        "gains.anti_windup": 5.0,
        "thruster_health": [1.0] * 8,
        "failed_thrusters": [],
    }
    values.update({name.replace("__", "."): value for name, value in changes.items()})
    return values


def test_save_rewrites_only_changed_values_and_keeps_comments(path):
    profile = GainProfile(path)
    result = profile.save(runtime(gains__x=[1.92, 0.512, 2.4], model__mass=[112.25, 60.0, 70.0]))

    assert result["written"] == {"gains.x": [1.92, 0.512, 2.4], "model.mass": [112.25, 60.0, 70.0]}
    text = path.read_text()
    expected = PROFILE.replace("x: [7.14, 0.612, 10.2]", "x: [1.92, 0.512, 2.4]").replace(
        "mass: [50.0, 60.0, 70.0]       #", "mass: [112.25, 60.0, 70.0]     #"
    )
    assert text == expected
    assert path.stat().st_mode & 0o777 == 0o644
    assert yaml.safe_load(text)["/**"]["ros__parameters"]["gains"]["x"] == [1.92, 0.512, 2.4]


def test_save_writes_doubles_that_read_back_as_doubles(path):
    GainProfile(path).save(runtime(gains__anti_windup=1e-7, power_limit=1.0))
    parameters = yaml.safe_load(path.read_text())["/**"]["ros__parameters"]
    assert parameters["gains"]["anti_windup"] == 1e-7
    assert isinstance(parameters["power_limit"], float)
    assert "anti_windup: 1.0e-07  # 1/s" in path.read_text()


def test_save_keeps_crlf_line_endings(tmp_path):
    path = tmp_path / "gains.yaml"
    path.write_bytes(PROFILE.replace("\n", "\r\n").encode())
    profile = GainProfile(path)
    profile.save(runtime(power_limit=0.5))
    profile.save(runtime(power_limit=0.4))  # what it wrote is what it expects on disk
    expected = PROFILE.replace("power_limit: 0.6", "power_limit: 0.4").replace("\n", "\r\n")
    assert path.read_bytes() == expected.encode()


def test_save_refuses_values_yaml_cannot_read_back(path):
    with pytest.raises(ValueError, match="cannot write nan"):
        GainProfile(path).save(runtime(model__net_buoyancy=float("nan")))
    assert path.read_text() == PROFILE


def test_thruster_faults_and_parameters_the_file_lacks_are_not_saved(path):
    result = GainProfile(path).save(runtime(thruster_health=[0.0] * 8, failed_thrusters=[3]))
    assert result["written"] == {}
    assert path.read_text() == PROFILE


def test_save_refuses_to_overwrite_external_edits(path):
    profile = GainProfile(path)
    path.write_text(PROFILE + "# edited by hand\n")
    with pytest.raises(RuntimeError, match="changed on disk"):
        profile.save(runtime(power_limit=0.5))
    profile.reload()
    profile.save(runtime(power_limit=0.5))
    assert "power_limit: 0.5\n" in path.read_text()


def test_save_refuses_values_it_cannot_rewrite_in_place(tmp_path):
    path = tmp_path / "gains.yaml"
    path.write_text(
        PROFILE.replace(
            "x: [7.14, 0.612, 10.2]", "x:\n        - 7.14\n        - 0.612\n        - 10.2"
        )
    )
    with pytest.raises(RuntimeError, match="one line"):
        GainProfile(path).save(runtime(gains__x=[1.0, 2.0, 3.0]))
