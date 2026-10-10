import math
import os
import tempfile
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

from jinja2 import Environment, FileSystemLoader, StrictUndefined

from sub_sim.robot_scenario_to_urdf import Transform, _mat_to_rpy


@dataclass(frozen=True)
class Pose:
    """A pose: metres, and roll, pitch, yaw in radians applied Z-Y-X (as Stonefish's rpy)."""

    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def transform(self) -> Transform:
        return Transform.from_xyz_rpy(self.xyz, self.rpy)

    @staticmethod
    def of(transform: Transform) -> "Pose":
        return Pose(tuple(transform.t), _mat_to_rpy(transform.R))


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
) -> Path:
    """Render the robot at a pose.

    `thrusters` places the thrusters, one dict per thruster in channel order with
    `xyz` and `rpy` (radians) strings in the robot's base link and `right`
    ("true" for a right-hand propeller); see thruster_specs(). `frames` are the
    vehicle description's frames in the robot's base link (mount_poses()).
    `sensor_noise` gives the DVL, IMU and pressure sensor the noise of the real
    ones. `segmentation_camera` adds that sensor, which is only used for
    sim_labeling, and `depth_camera` the depth camera, which nothing in the
    stack reads yet.

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
        segmentation_camera=segmentation_camera,
        depth_camera=depth_camera,
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
