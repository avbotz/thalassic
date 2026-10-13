"""Checks that the launch files and configs parse, and the vehicle description is valid."""

from pathlib import Path

import pytest
import yaml
from launch import LaunchContext, LaunchDescription
from launch.launch_description_sources import (
    get_launch_description_from_python_launch_file,
    load_python_launch_file_as_module,
)
from launch_ros.actions import Node
from rclpy.parameter import parameter_dict_from_yaml_file

from sub_bringup.vehicle import load_vehicle

PACKAGE = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE / "config"
LAUNCH_DIR = PACKAGE / "launch"


def relative(path: Path) -> str:
    return str(path.relative_to(PACKAGE))


@pytest.mark.parametrize("launch_file", sorted(LAUNCH_DIR.glob("*_launch.py")), ids=relative)
def test_launch_file_builds(launch_file):
    description = get_launch_description_from_python_launch_file(launch_file)
    assert isinstance(description, LaunchDescription)


def test_hardware_entities_build():
    """The drivers and cameras of pool_test_launch.py, built only once robot_name is known."""
    launch = load_python_launch_file_as_module(str(LAUNCH_DIR / "pool_test_launch.py"))
    context = LaunchContext()
    context.launch_configurations["robot_name"] = "marlin_v3"
    entities = launch._vehicle_entities(context)
    assert entities and all(isinstance(entity, Node) for entity in entities)


@pytest.mark.parametrize("config", sorted(CONFIG_DIR.rglob("*.yaml")), ids=relative)
def test_config_is_yaml(config):
    assert yaml.safe_load(config.read_text()) is not None


def test_vehicle_description_loads():
    vehicle = load_vehicle("marlin_v3")

    assert len(vehicle.thruster_layout()["allocation.thruster_reversed"]) == 8

    required_devices = {"down_camera", "front_camera", "dvl", "imu", "maritime_mcu"}
    assert required_devices.issubset(vehicle.devices)


def test_both_gains_profiles_set_the_same_parameters():
    def names(file):
        file_path = CONFIG_DIR / "control" / file
        return set(parameter_dict_from_yaml_file(file_path, use_wildcard=True))

    assert names("gains.yaml") == names("gains_sim.yaml")
