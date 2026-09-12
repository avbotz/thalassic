"""The vehicle description: config/<robot_name>.yaml, loaded into launch files.

Hardware identity (mounting offsets, thruster geometry, device names, camera
serial numbers) lives in that YAML file, not in the launch files that use it.
"""

import math
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory


def _publisher_arguments(parent: str, child: str, pose: dict[str, Any]) -> list[str]:
    """static_transform_publisher's command line for one entry of the description.

    Translations are metres, rotations degrees; the publisher wants radians.
    """
    args = []
    for axis in ("x", "y", "z"):
        args += [f"--{axis}", str(float(pose.get(axis, 0.0)))]
    for angle in ("roll", "pitch", "yaw"):
        args += [f"--{angle}", str(math.radians(float(pose.get(angle, 0.0))))]
    return [*args, "--frame-id", parent, "--child-frame-id", child]


class Vehicle:
    """One vehicle's hardware description, keyed by frame and device name."""

    def __init__(self, name: str, description: dict[str, Any]):
        self.name = name
        self._description = description
        self._global_frames = set(description.get("global_frames", []))

    def frame(self, name: str) -> str:
        """Namespace a frame name, leaving the frames shared between vehicles alone."""
        return name if name in self._global_frames else f"{self.name}/{name}"

    def transform_arguments(self) -> list[list[str]]:
        """One static_transform_publisher argument list per transform, thrusters included."""
        entries = [(e["parent"], e["child"], e) for e in self._description.get("transforms", [])]
        entries += [
            ("base_link_ned", f"thruster_{i}_link", pose)
            for i, pose in enumerate(self._description.get("thrusters", []))
        ]
        return [
            _publisher_arguments(self.frame(parent), self.frame(child), pose)
            for parent, child, pose in entries
        ]

    def device(self, name: str) -> str:
        """The udev path of a serial device (see deploy/udev)."""
        return self._description["devices"][name]

    def camera(self, name: str) -> dict[str, Any]:
        """The identity of one camera: serial number or video device."""
        return self._description["cameras"][name]

    def dvl_address(self) -> str:
        return self._description["dvl"]["ip_address"]


def load_vehicle(robot_name: str) -> Vehicle:
    """Read config/<robot_name>.yaml from the installed sub_bringup share directory."""
    path = Path(get_package_share_directory("sub_bringup")) / "config" / f"{robot_name}.yaml"
    return Vehicle(robot_name, yaml.safe_load(path.read_text()))
