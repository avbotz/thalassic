"""The launch files and configs parse, and the vehicle description is valid.

Nothing is launched: building each LaunchDescription catches the import,
syntax and argument mistakes that otherwise surface only when the stack starts.
"""

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchDescription
from sub_bringup.vehicle import load_vehicle

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("launch_file", sorted((PACKAGE / "launch").glob("*_launch.py")))
def test_launch_file_builds(launch_file):
    spec = importlib.util.spec_from_file_location(launch_file.stem, launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert isinstance(module.generate_launch_description(), LaunchDescription)


@pytest.mark.parametrize(
    "config", sorted(p for p in (PACKAGE / "config").rglob("*.yaml") if "vehicles" not in p.parts)
)
def test_config_is_yaml(config):
    assert yaml.safe_load(config.read_text()) is not None


def test_vehicle_description_loads():
    vehicle = load_vehicle("marlin_v3")
    layout = vehicle.thruster_layout()
    assert len(layout["allocation.thruster_reversed"]) == 8
    for role in ("down_camera", "front_camera", "dvl", "imu", "maritime_mcu"):
        assert role in vehicle.devices


def test_both_gains_profiles_set_the_same_parameters():
    def keys(document, prefix=""):
        out = set()
        for key, value in document.items():
            name = f"{prefix}{key}"
            out |= keys(value, name + ".") if isinstance(value, dict) else {name}
        return out

    profiles = [
        yaml.safe_load((PACKAGE / "config" / "control" / name).read_text())["/**"][
            "ros__parameters"
        ]
        for name in ("gains.yaml", "gains_sim.yaml")
    ]
    assert keys(profiles[0]) == keys(profiles[1])
