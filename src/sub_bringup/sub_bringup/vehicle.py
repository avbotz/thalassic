"""
Loads vehicle description (config/vehicles/<robot_name>.yaml).

Should contain hardware info (mounting offsets, thruster geometry, device names, camera
serial numbers).
"""

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Fixed-axis roll, pitch, yaw in radians (applied Z-Y-X), as tf2's Quaternion::setRPY."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def _transform(parent: str, child: str, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    """One transform from the description: metres, and degrees for the angles.

    Takes keyword arguments only from the file, so an unknown key raises rather than
    silently reading as zero.
    """
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation = Vector3(x=float(x), y=float(y), z=float(z))
    transform.transform.rotation = quaternion_from_euler(
        *(math.radians(float(angle)) for angle in (roll, pitch, yaw))
    )
    return transform


@dataclass(frozen=True)
class Device:
    """One device on the vehicle: how to reach this unit, and where it is mounted.

    A device is reached by exactly one of serial_number, video_device, serial_port
    or ip_address.
    """

    name: str
    mount: TransformStamped | None = None
    serial_number: str | None = None
    video_device: str | None = None
    serial_port: str | None = None
    ip_address: str | None = None

    @property
    def frame(self) -> str:
        """The frame the device reports in (its mount's child frame)."""
        if self.mount is None:
            raise KeyError(f"device {self.name!r} has no mount, so no frame")
        return self.mount.child_frame_id


@dataclass(frozen=True)
class DepthSensor:
    """The depth reading: the frame of the pressure sensor and the reading's variance [m^2]."""

    frame: str
    z_variance: float


def rotate(q: Quaternion, v: tuple[float, float, float]) -> tuple[float, float, float]:
    """Rotate vector v by unit quaternion q."""
    x, y, z = v
    tx, ty, tz = 2 * (q.y * z - q.z * y), 2 * (q.z * x - q.x * z), 2 * (q.x * y - q.y * x)
    return (
        x + q.w * tx + (q.y * tz - q.z * ty),
        y + q.w * ty + (q.z * tx - q.x * tz),
        z + q.w * tz + (q.x * ty - q.y * tx),
    )


@dataclass(frozen=True)
class Thruster:
    """One thruster: its mount, and whether it turns a left-hand (mirrored) propeller.

    The mount's +X is the way a right-hand propeller pushes on a positive command;
    a left-hand one pushes the other way.
    """

    mount: TransformStamped
    left_hand: bool
    xyz: tuple[float, float, float]
    """Position in the mount's parent frame [m], as written in the description."""
    rpy: tuple[float, float, float]
    """Orientation in the mount's parent frame [rad], applied Z-Y-X."""


@dataclass(frozen=True)
class Vehicle:
    """One vehicle's hardware description. Every frame name in it is already namespaced."""

    name: str
    global_frames: frozenset[str]
    """Frames shared between vehicles, which frame() leaves un-namespaced."""
    transforms: tuple[TransformStamped, ...]
    """Every static transform: the listed ones, device mounts and thrusters."""
    devices: dict[str, Device]
    depth: DepthSensor
    thrusters: tuple[Thruster, ...] = ()

    def frame(self, name: str) -> str:
        """Namespace a frame name, leaving the frames shared between vehicles alone."""
        return name if name in self.global_frames else f"{self.name}/{name}"

    def in_frame(
        self, transform: TransformStamped, root: str = "base_link"
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """Position of a transform's child frame and its +X axis, expressed in `root`.

        Follows the description's static transforms from the child up to `root`.
        """
        by_child = {t.child_frame_id: t for t in self.transforms}
        root = self.frame(root)
        t = transform.transform.translation
        position = (t.x, t.y, t.z)
        axis = rotate(transform.transform.rotation, (1.0, 0.0, 0.0))
        parent = transform.header.frame_id
        while parent != root:
            if parent not in by_child:
                raise ValueError(f"no transform chain from {transform.child_frame_id} up to {root}")
            up = by_child[parent]
            offset = up.transform.translation
            position = tuple(
                p + o
                for p, o in zip(
                    rotate(up.transform.rotation, position),
                    (offset.x, offset.y, offset.z),
                    strict=True,
                )
            )
            axis = rotate(up.transform.rotation, axis)
            parent = up.header.frame_id
        return position, axis

    def thruster_layout(self) -> dict[str, list]:
        """The sub_control allocation parameters: each thruster's position and +X in base_link (FLU)."""
        positions, directions = [], []
        for thruster in self.thrusters:
            position, axis = self.in_frame(thruster.mount)
            positions.extend(position)
            directions.extend(axis)
        return {
            "allocation.thruster_positions": positions,
            "allocation.thruster_directions": directions,
            "allocation.thruster_reversed": [t.left_hand for t in self.thrusters],
        }

    @classmethod
    def from_yaml(cls, name: str, description: dict[str, Any]) -> "Vehicle":
        global_frames = frozenset(description.get("global_frames", []))

        def frame(f: str) -> str:
            return f if f in global_frames else f"{name}/{f}"

        def transform(entry: dict[str, Any]) -> TransformStamped:
            entry = dict(entry)
            return _transform(frame(entry.pop("parent")), frame(entry.pop("child")), **entry)

        devices = {}
        for role, entry in description.get("devices", {}).items():
            entry = dict(entry)
            mount = transform(entry.pop("mount")) if "mount" in entry else None
            identity = {key: str(value) for key, value in entry.items()}
            if len(identity) != 1:
                raise ValueError(f"device {role!r} needs exactly one address, has {identity}")
            devices[role] = Device(role, mount, **identity)

        thrusters = []
        for i, entry in enumerate(description.get("thrusters", [])):
            pose = dict(entry)
            propeller = pose.pop("propeller", "right")
            if propeller not in ("right", "left"):
                raise ValueError(
                    f"thruster {i}: propeller must be right or left, not {propeller!r}"
                )
            # sub_sim places the thruster at xyz/rpy in base_link_ned, whatever the mount says.
            if "parent" in pose or "child" in pose:
                raise ValueError(
                    f"thruster {i}: thrusters are always base_link_ned to thruster_{i}_link, "
                    "so take no parent or child"
                )
            mount = transform({"parent": "base_link_ned", "child": f"thruster_{i}_link", **pose})
            thrusters.append(
                Thruster(
                    mount=mount,
                    left_hand=propeller == "left",
                    xyz=tuple(float(pose.get(k, 0.0)) for k in ("x", "y", "z")),
                    rpy=tuple(
                        math.radians(float(pose.get(k, 0.0))) for k in ("roll", "pitch", "yaw")
                    ),
                )
            )
        transforms = (
            *(transform(entry) for entry in description.get("transforms", [])),
            *(device.mount for device in devices.values() if device.mount is not None),
            *(thruster.mount for thruster in thrusters),
        )

        # Unknown keys raise, as for transforms.
        depth_entry = dict(description["depth"])
        depth = DepthSensor(
            frame=frame(depth_entry.pop("frame")),
            z_variance=float(depth_entry.pop("z_variance")),
            **depth_entry,
        )
        frames = {f for t in transforms for f in (t.header.frame_id, t.child_frame_id)}
        if depth.frame not in frames:
            raise ValueError(f"depth frame {depth.frame!r} is not a frame in the description")

        return cls(
            name=name,
            global_frames=global_frames,
            transforms=transforms,
            devices=devices,
            depth=depth,
            thrusters=tuple(thrusters),
        )

    @classmethod
    def from_file(cls, path: Path, name: str | None = None) -> "Vehicle":
        """Read a description file; the vehicle is named after the file unless told otherwise."""
        # Launch prints only an exception's message, so it has to say which file and key.
        try:
            return cls.from_yaml(name or path.stem, yaml.safe_load(path.read_text()) or {})
        except KeyError as e:
            raise ValueError(f"{path}: missing key {e}") from e
        except (TypeError, ValueError, yaml.YAMLError) as e:
            raise ValueError(f"{path}: {e}") from e


def load_vehicle(robot_name: str) -> Vehicle:
    """Read config/vehicles/<robot_name>.yaml from the installed sub_bringup share."""
    share = Path(get_package_share_directory("sub_bringup"))
    return Vehicle.from_file(share / "config" / "vehicles" / f"{robot_name}.yaml", robot_name)
