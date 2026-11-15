import csv
import math
import os
import random
import tempfile
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from jinja2 import Environment, FileSystemLoader, StrictUndefined

from sub_sim.robot_scenario_to_urdf import Transform


@dataclass(frozen=True)
class Pose:
    """A pose: metres, and roll, pitch, yaw in radians applied Z-Y-X (as Stonefish's rpy)."""

    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def transform(self) -> Transform:
        return Transform.from_xyz_rpy(self.xyz, self.rpy)

    @staticmethod
    def of(transform: Transform) -> "Pose":
        return Pose(tuple(transform.t), transform.rpy())


# Standard gravity and fresh water, as the scenario defines them.
GRAVITY = 9.81
WATER_DENSITY = 1000.0


def thrust_curve(path: Path | str | None = None) -> dict[str, str]:
    """The thrusters' measured thrust curves as the thruster template takes them:
    `commands`, the supply `voltages` [V] and the `forces` [N] of every voltage one
    after another, space separated. From sub_control's data/t200_thrust.csv, the
    only copy of the curve, which sub_control builds its own lookup table from."""
    if path is None:
        path = Path(get_package_share_directory("sub_control")) / "data" / "t200_thrust.csv"
    with open(path, newline="") as f:
        rows = list(csv.reader(line for line in f if not line.startswith("#")))
    header, data = rows[0], rows[1:]
    return {
        "commands": " ".join(row[0] for row in data),
        "voltages": " ".join(name.removesuffix("V") for name in header[1:]),
        "forces": "  ".join(" ".join(row[k] for row in data) for k in range(1, len(header))),
    }


@dataclass(frozen=True)
class Hydrodynamics:
    """The vehicle model of sub_control as Stonefish takes it: about the simulated
    robot's base link (base_link_ned), which shares base_link's origin."""

    mass: float  # rigid body [kg]
    center_of_gravity: tuple[float, float, float]
    inertia: tuple[float, float, float]  # with the added inertia [kg m^2]
    added_mass: tuple[float, float, float]  # [kg]
    linear_damping: tuple[float, ...]  # [N s/m] x3, [N m s/rad] x3
    quadratic_damping: tuple[float, ...]  # [N s^2/m^2] x3, [N m s^2/rad^2] x3
    displaced_volume: float  # [m^3]
    center_of_buoyancy: tuple[float, float, float]


def model_hydrodynamics(
    model: Mapping,
    base_link_ned: Transform,
    error: float = 0.0,
    rng: random.Random | None = None,
) -> Hydrodynamics:
    """sub_control's vehicle model (the `model` block of a gains file, in base_link
    FLU) in the frame and the terms the simulated robot is defined in, so that the
    simulator's vehicle is the controller's.

    `base_link_ned` is that frame's pose in base_link; its axes must lie along
    base_link's, as the model has one coefficient per axis. The rigid body's mass is
    what gives the model's weight (buoyancy less net buoyancy), and the rest of the
    model's mass is added mass.

    A real vehicle is never quite its model. With `error`, each term is scaled by a
    random factor within that fraction of 1 (drawn from `rng`), and the centres of
    gravity and buoyancy move by up to that fraction of 10 cm, so that the
    controller can be tried on a vehicle that differs from the one it expects.
    """
    R = base_link_ned.R
    # Which base_link axis each axis of base_link_ned lies along, and its sign.
    axes = []
    for column in range(3):
        along = [round(R[row][column]) for row in range(3)]
        if sorted(abs(v) for v in along) != [0, 0, 1] or any(
            abs(R[row][column] - along[row]) > 1e-6 for row in range(3)
        ):
            raise ValueError("base_link_ned's axes must lie along base_link's")
        axes.append((along.index(1) if 1 in along else along.index(-1), sum(along)))

    rng = rng or random.Random()

    def scale() -> float:
        return 1.0 + rng.uniform(-error, error) if error else 1.0

    def shift() -> float:
        return 0.1 * rng.uniform(-error, error) if error else 0.0

    def per_axis(values: Sequence[float]) -> tuple[float, ...]:
        """One value per axis: translation, then rotation if there are six."""
        return tuple(
            values[offset + axis] * scale()
            for offset in range(0, len(values), 3)
            for axis, _ in axes
        )

    def point(values: Sequence[float]) -> tuple[float, float, float]:
        return tuple(sign * values[axis] + shift() for axis, sign in axes)

    buoyancy = model["buoyancy"] * scale()
    weight = buoyancy - model["net_buoyancy"]
    if not (buoyancy > 0.0 and weight > 0.0):
        raise ValueError("The vehicle model needs a positive buoyancy and weight")
    mass = weight / GRAVITY
    added_mass = tuple(m - mass for m in per_axis(model["mass"]))
    if min(added_mass) < 0.0:
        raise ValueError("The vehicle model's mass is less than its weight gives")
    return Hydrodynamics(
        mass=mass,
        center_of_gravity=point(model["center_of_gravity"]),
        inertia=per_axis(model["inertia"]),
        added_mass=added_mass,
        linear_damping=per_axis(model["linear_drag"]),
        quadratic_damping=per_axis(model["quadratic_drag"]),
        displaced_volume=buoyancy / (WATER_DENSITY * GRAVITY),
        center_of_buoyancy=point(model["center_of_buoyancy"]),
    )


def render_robot_scenario(
    robot_template_file: Path | str,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    thrusters: list[dict[str, str]] | None = None,
    frames: dict[str, Pose] | None = None,
    sensor_noise: bool = True,
    segmentation_camera: bool = False,
    depth_camera: bool = False,
    cameras: bool = True,
    hydrodynamics: Hydrodynamics | None = None,
    thrust: dict[str, str] | None = None,
    battery_voltage: float = 16.0,
) -> Path:
    """Render the robot at a pose.

    `thrusters` places the thrusters, one dict per thruster in channel order with
    `xyz` and `rpy` (radians) strings in the robot's base link and `right`
    ("true" for a right-hand propeller); see thruster_specs(). `frames` are the
    vehicle description's frames in the robot's base link (mount_poses()).
    `sensor_noise` gives the DVL, IMU and pressure sensor the noise of the real
    ones. `segmentation_camera` adds that sensor, whose images label the front
    camera's for training data (recorded for remora), and `depth_camera` the
    depth camera, which nothing in the stack reads yet. Without `cameras` the
    robot has no camera at all (Stonefish without a GPU cannot render one), and
    neither of those either.

    `hydrodynamics` (model_hydrodynamics()) makes the robot the vehicle model that
    sub_control uses: its mass, inertia, buoyancy, added mass and damping. Without
    it Stonefish estimates them from the robot's geometry. `thrust` is the
    thrusters' thrust curve (thrust_curve(), sub_control's own by default) and
    `battery_voltage` the supply voltage they start at.

    The template's world_pose(*poses, offset, rpy) places a free body (a torpedo,
    the dropper's ball) that starts attached to the robot: it chains `poses`, given
    in the robot's base link, onto the robot's pose and then moves the result by
    `offset` and turns it by `rpy` in its own frame.
    """
    robot_template_file = Path(robot_template_file)
    robot = Pose((x, y, z), (roll, pitch, yaw)).transform()

    def world_pose(
        *poses: Pose,
        offset: Sequence[float] = (0.0, 0.0, 0.0),
        rpy: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> Pose:
        transform = robot
        for pose in (*poses, Pose(tuple(offset), tuple(rpy))):
            transform = transform @ pose.transform()
        return Pose.of(transform)

    # Let Jinja resolve `{% from "thruster.j2" import ... %}`. A variable the template uses but
    # is not given raises rather than rendering as an empty attribute.
    env = Environment(
        loader=FileSystemLoader(str(robot_template_file.parent)), undefined=StrictUndefined
    )

    template = env.get_template(robot_template_file.name)

    rendered = template.render(
        PI=math.pi,
        x=x,
        y=y,
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        thrusters=thrusters or [],
        frames=frames or {},
        world_pose=world_pose,
        sensor_noise=sensor_noise,
        segmentation_camera=segmentation_camera and cameras,
        depth_camera=depth_camera and cameras,
        cameras=cameras,
        hydrodynamics=hydrodynamics,
        thrust=thrust or thrust_curve(),
        battery_voltage=battery_voltage,
    )

    fd, temp_path = tempfile.mkstemp(
        prefix=robot_template_file.stem,
        suffix=".scn",
    )
    with os.fdopen(fd, "w") as f:
        f.write(rendered)

    return Path(temp_path)


def thruster_specs(thrusters) -> list[dict[str, str]]:
    """Template arguments for a vehicle description's thrusters (sub_bringup.vehicle.Thruster)."""
    return [
        {
            "xyz": " ".join(f"{v:.6g}" for v in thruster.xyz),
            "rpy": " ".join(f"{v:.6g}" for v in thruster.rpy),
            "right": "false" if thruster.left_hand else "true",
        }
        for thruster in thrusters
    ]


def _transform(stamped) -> Transform:
    """A geometry_msgs TransformStamped as a Transform."""
    p, q = stamped.transform.translation, stamped.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    return Transform(rotation, (p.x, p.y, p.z))


def _inverse(transform: Transform) -> Transform:
    rt = [list(row) for row in zip(*transform.R, strict=True)]
    t = transform.t
    return Transform(rt, tuple(-sum(rt[i][k] * t[k] for k in range(3)) for i in range(3)))


def base_link_ned_pose(vehicle) -> Transform:
    """The pose in base_link of base_link_ned, the simulated robot's base link."""
    by_child = {t.child_frame_id: t for t in vehicle.transforms}
    frame = vehicle.frame("base_link_ned")
    pose = Transform.identity()
    while frame != vehicle.frame("base_link"):
        if frame not in by_child:
            raise ValueError(
                f"{vehicle.name}'s description has no transform from base_link to base_link_ned"
            )
        pose = _transform(by_child[frame]) @ pose
        frame = by_child[frame].header.frame_id
    return pose


def mount_poses(vehicle) -> dict[str, Pose]:
    """Every frame of a vehicle description (sub_bringup.vehicle.Vehicle) below base_link,
    expressed in base_link_ned, the simulated robot's base link, by its name without the
    vehicle's namespace. Frames mounted on base_link (FLU), like the IMU, are included."""
    by_child = {t.child_frame_id: t for t in vehicle.transforms}
    base_link = vehicle.frame("base_link")

    def in_base_link(frame: str) -> Transform | None:
        """The frame's pose in base_link, or None if it is not below base_link."""
        pose = Transform.identity()
        while frame != base_link:
            if frame not in by_child:
                return None
            pose = _transform(by_child[frame]) @ pose
            frame = by_child[frame].header.frame_id
        return pose

    base_link_ned = in_base_link(vehicle.frame("base_link_ned"))
    if base_link_ned is None:
        raise ValueError(
            f"{vehicle.name}'s description has no transform from base_link to base_link_ned"
        )
    from_base_link = _inverse(base_link_ned)
    poses = {}
    for transform in vehicle.transforms:
        pose = in_base_link(transform.child_frame_id)
        if pose is None:
            continue
        name = transform.child_frame_id.removeprefix(f"{vehicle.name}/")
        mount = Pose.of(from_base_link @ pose)
        # Drops the rounding noise of the round trip through base_link (and -0.0).
        poses[name] = Pose(
            tuple(round(v, 9) + 0.0 for v in mount.xyz), tuple(round(v, 9) + 0.0 for v in mount.rpy)
        )
    return poses
