from pathlib import Path

import pytest
import yaml

from sub_pid_tuner.profile import GainProfile


def write_profile(path):
    path.write_text(
        yaml.safe_dump(
            {
                "/marlin_v2/sub_control": {
                    "ros__parameters": {
                        "power_limit": 0.6,
                        "pos_pid": {"x": [0.8, 0.0, 0.0, 1.0]},
                    }
                }
            },
            sort_keys=False,
        )
    )


def test_profile_save_uses_verified_runtime_values(tmp_path, monkeypatch):
    path = tmp_path / "control_gains.yaml"
    write_profile(path)
    monkeypatch.setattr(Path, "home", staticmethod(lambda: tmp_path))
    profile = GainProfile(path, "/marlin_v2/sub_control")

    result = profile.save(
        {
            "power_limit": {"value": 0.5},
            "pos_pid.x": {"value": [1.2, 0.1, 0.03, 1.0]},
        }
    )

    saved = yaml.safe_load(path.read_text())["/marlin_v2/sub_control"]["ros__parameters"]
    assert saved["power_limit"] == 0.5
    assert saved["pos_pid"]["x"] == [1.2, 0.1, 0.03, 1.0]
    assert Path(result["backup"]).exists()


def test_profile_save_rejects_external_edits(tmp_path, monkeypatch):
    path = tmp_path / "control_gains.yaml"
    write_profile(path)
    monkeypatch.setattr(Path, "home", staticmethod(lambda: tmp_path))
    profile = GainProfile(path, "/marlin_v2/sub_control")
    path.write_text(path.read_text() + "# external edit" + chr(10))

    with pytest.raises(RuntimeError, match="changed on disk"):
        profile.save({"power_limit": {"value": 0.5}})


def test_feedforward_profile_round_trips_nested_runtime_parameters(tmp_path, monkeypatch):
    path = tmp_path / "control_ff_sim.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "/**": {
                    "ros__parameters": {
                        "feedforward_enabled": True,
                        "reference": {"max_acceleration": [0.1] * 6},
                        "model": {
                            "effective_mass": [40.0] * 6,
                            "restoring_stiffness": [0.0] * 6,
                        },
                        "feedback": {"kp": [20.0] * 6},
                    }
                }
            },
            sort_keys=False,
        )
    )
    monkeypatch.setattr(Path, "home", staticmethod(lambda: tmp_path))
    profile = GainProfile(path, "/marlin_v2/sub_control")

    assert profile.values["model.effective_mass"] == [40.0] * 6
    assert profile.values["feedforward_enabled"] is True
    profile.save(
        {
            "feedforward_enabled": {"value": False},
            "reference.max_acceleration": {"value": [0.05] * 6},
            "model.effective_mass": {"value": [45.0] * 6},
            "model.restoring_stiffness": {"value": [0.0, 0.0, 0.0, 50.0, 60.0, 0.0]},
            "feedback.kp": {"value": [100.0] * 6},
        }
    )

    saved = yaml.safe_load(path.read_text())["/**"]["ros__parameters"]
    assert saved["feedforward_enabled"] is False
    assert saved["reference"]["max_acceleration"] == [0.05] * 6
    assert saved["model"]["effective_mass"] == [45.0] * 6
    assert saved["model"]["restoring_stiffness"][3:5] == [50.0, 60.0]
    assert saved["feedback"]["kp"] == [100.0] * 6
