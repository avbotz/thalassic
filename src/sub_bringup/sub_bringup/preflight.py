"""Checks a running stack before it goes in the water; exits non-zero on any failure.

    ros2 run sub_bringup preflight            # the vehicle (pixi run pool)
    ros2 run sub_bringup preflight --sim      # the simulator (pixi run sim)

(A headless simulator has no cameras, so their checks fail there.)

It listens for a few seconds and checks that every sensor and controller
topic arrives at its rate, that the kill switch is reported, that both
cameras are calibrated, that nothing in /diagnostics_agg is in error or
stale, and that every released model (and on the vehicle its TensorRT
engine) is on disk. It sends nothing and moves nothing. docs/deployment.md
has the pool-day checklist it belongs to.
"""

import argparse
import functools
import os
import sys
import time
import tomllib
from dataclasses import dataclass
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Bool
from sub_control_interfaces.msg import ThrusterCommand


@dataclass
class Rate:
    topic: str
    kind: type
    minimum: float  # [Hz]
    required: bool = True  # False: a warning only (the DVL has no lock out of the water)
    count: int = 0


CAMERAS = ("front_camera", "down_camera")

LEVELS = {
    DiagnosticStatus.OK: "OK",
    DiagnosticStatus.WARN: "WARN",
    DiagnosticStatus.ERROR: "ERROR",
    DiagnosticStatus.STALE: "STALE",
}


def rates(sim: bool) -> list[Rate]:
    # Well under each stream's rate (IMU 50 Hz, 30 in the simulator; depth 5; DVL
    # 4-15; control 50; cameras 24 and 5): these catch a dead or collapsing
    # stream, not a few hertz of jitter.
    image = "image_color" if sim else "image_raw"
    return [
        Rate("imu/data", Imu, 20.0),
        Rate("odometry/depth", Odometry, 3.0),
        Rate("odometry/dvl", Odometry, 3.0, required=False),
        # The EKF publishes as measurements arrive: about the IMU's rate.
        Rate("odometry/filtered", Odometry, 20.0),
        Rate("control/thrusters", ThrusterCommand, 30.0),
        Rate(f"front_camera/{image}", Image, 5.0),
        Rate(f"down_camera/{image}", Image, 2.0),
    ]


class Preflight(Node):
    def __init__(self, sim: bool):
        super().__init__("preflight")
        self.rates = rates(sim)
        for rate in self.rates:
            self.create_subscription(
                rate.kind, rate.topic, self._counter(rate), qos_profile_sensor_data
            )
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.killed: bool | None = None
        self.armed: bool | None = None
        self.create_subscription(Bool, "kill_switch", self._on_kill, latched)
        self.create_subscription(Bool, "control/armed", self._on_armed, latched)
        self.camera_info: dict[str, CameraInfo] = {}
        for camera in CAMERAS:
            self.create_subscription(
                CameraInfo,
                f"{camera}/camera_info",
                functools.partial(self._on_camera_info, camera),
                qos_profile_sensor_data,
            )
        self.diagnostics: dict[str, DiagnosticStatus] = {}
        self.create_subscription(DiagnosticArray, "/diagnostics_agg", self._on_diagnostics, 10)

    @staticmethod
    def _counter(rate: Rate):
        def count(_):
            rate.count += 1

        return count

    def _on_kill(self, msg: Bool) -> None:
        self.killed = msg.data

    def _on_armed(self, msg: Bool) -> None:
        self.armed = msg.data

    def _on_camera_info(self, camera: str, msg: CameraInfo) -> None:
        self.camera_info[camera] = msg

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            self.diagnostics[status.name] = status


def check_models(root: Path, engines: bool) -> list[str]:
    """Problems with weights/: released models missing, or (on the vehicle) their engines."""
    problems = []
    manifest = tomllib.loads((root / "weights" / "manifest.toml").read_text())
    for name in manifest:
        path = root / "weights" / name
        if not path.exists():
            problems.append(f"weights/{name} missing: pixi run fetch-weights")
        elif (
            engines
            and path.suffix == ".onnx"
            and not list(path.parent.glob(f"{path.stem}.*.engine"))
        ):
            problems.append(f"no TensorRT engine for weights/{name}: pixi run build-engines")
    return problems


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--sim", action="store_true", help="check the simulator's stack")
    parser.add_argument("--robot", default="marlin_v3", help="robot_name (the namespace)")
    parser.add_argument("--seconds", type=float, default=5.0, help="how long to listen")
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(os.environ.get("PIXI_PROJECT_ROOT", ".")),
        help="the workspace, where weights/ is (default: the pixi project's)",
    )
    args, ros_args = parser.parse_known_args(argv)

    # Rates are counted on the stack's clock: the simulator's can run slower than the wall's.
    clock = ["-p", "use_sim_time:=true"] if args.sim else []
    rclpy.init(args=["preflight", "--ros-args", "-r", f"__ns:=/{args.robot}", *clock, *ros_args])
    node = Preflight(args.sim)
    end = time.monotonic() + args.seconds
    start = None
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if start is None and node.get_clock().now().nanoseconds > 0:
            start = node.get_clock().now()
            for rate in node.rates:
                rate.count = 0
    elapsed = (node.get_clock().now() - start).nanoseconds * 1e-9 if start else 0.0

    failures: list[str] = []
    warnings: list[str] = []
    print(f"Listened {elapsed:.1f} s ({'simulated' if args.sim else 'wall'}) on /{args.robot}:")
    if elapsed <= 0.0:
        failures.append("no clock: is the simulator running?" if args.sim else "no time passed")
    for rate in node.rates:
        hz = rate.count / elapsed if elapsed > 0.0 else 0.0
        if hz >= rate.minimum:
            mark = "ok  "
        elif rate.required:
            mark = "FAIL"
            failures.append(f"{rate.topic} at {hz:.1f} Hz")
        else:
            mark = "warn"
            warnings.append(f"{rate.topic} at {hz:.1f} Hz")
        print(f"  {mark} {rate.topic}: {hz:.1f} Hz (>= {rate.minimum:g})")

    if node.killed is None:
        failures.append("no kill_switch state")
    else:
        print(f"  kill switch: {'killed' if node.killed else 'released'}")
    print(f"  sub_control armed: {node.armed}")

    for camera in CAMERAS:
        info = node.camera_info.get(camera)
        if info is None:
            failures.append(f"no {camera}/camera_info")
        elif info.k[0] <= 0.0 or info.k[4] <= 0.0:
            failures.append(
                f"{camera} is not calibrated (camera_info K is zero): every bearing is 0"
            )

    if not node.diagnostics:
        failures.append("nothing on /diagnostics_agg (is diagnostic_aggregator running?)")
    for name, status in sorted(node.diagnostics.items()):
        # Only the leaves: each group's status repeats the worst of its items.
        if any(other.startswith(name + "/") for other in node.diagnostics):
            continue
        level = LEVELS.get(status.level, str(status.level))
        if status.level in (DiagnosticStatus.ERROR, DiagnosticStatus.STALE):
            failures.append(f"{name}: {level} {status.message}")
        elif status.level == DiagnosticStatus.WARN:
            warnings.append(f"{name}: {status.message}")

    failures += check_models(args.root, engines=not args.sim)

    node.destroy_node()
    rclpy.shutdown()

    for warning in warnings:
        print(f"WARN  {warning}")
    for failure in failures:
        print(f"FAIL  {failure}")
    print("PREFLIGHT " + ("FAILED" if failures else "PASSED"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
