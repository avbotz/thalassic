"""Replay a recorded run's thruster commands through the simulator and compare the motion.

    ros2 run sub_bringup replay <bag> [--gains control/gains_sim.yaml] [--out replay]

How far the simulator is from the vehicle, as numbers: the bag's thruster
commands (control/thrusters) are fed to the simulated vehicle, open loop, and its
velocity is compared with the one the bag recorded (odometry/filtered). The bag
can be from the pool or from the simulator.

The run is cut into windows. Each starts a fresh simulator with the vehicle at
the depth and attitude the bag has at that moment, at rest, in open water, and
steps it in lock with the recorded commands, so the result does not depend on
the computer. Open loop the two drift apart, which is why the windows are
short. The simulated vehicle starts each one at rest, so a window ends where
the recorded one is nearly at rest too (or after twice its length), and the
first seconds of each are left out of the errors. So is the time the recorded
vehicle spends at the surface, where it is not the submerged vehicle the model
describes.

It prints the RMS velocity error per axis and writes the velocities
(<out>.csv) and a plot of them (<out>.png). The simulated vehicle is the model
block of --gains, so the errors measure that model: identify the vehicle, put
the model in the gains file, and rerun to see what is left.
"""

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
import rosbag2_py
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray
from stonefish_ros2.srv import Step

from sub_bringup.vehicle import load_vehicle
from sub_control_interfaces.msg import ThrusterCommand
from sub_sim.generate_robot import (
    base_link_ned_pose,
    model_hydrodynamics,
    mount_poses,
    render_robot_scenario,
    thruster_specs,
)

AXES = ["surge", "sway", "heave", "roll rate", "pitch rate", "yaw rate"]
SIMULATION_RATE = 500.0  # [Hz] as sim_launch.py runs it
MAX_COMMAND = 0.6  # what the ESCs' firmware and sim_thrusters clamp to

# Open water with the materials the robot's parts are made of (as the pool scenario's).
SCENARIO = """<?xml version="1.0"?>
<scenario>
  <environment>
    <ned latitude="0.0" longitude="0.0" />
    <ocean>
      <water density="1000.0" jerlov="0.20" />
      <waves height="0" />
    </ocean>
  </environment>
  <materials>
    <material name="air" density="1.2" restitution="0.0"/>
    <material name="aluminum" density="1500.0" restitution="0.7"/>
    <material name="acrylic" density="1000.0" restitution="0.5"/>
    <material name="plastic" density="800.0" restitution="0.2" />
    <material name="concrete" density="2400" restitution="0.8" />
    <material name="pvc" density="1450" restitution="0.2" />
  </materials>
  <include file="{robot}" />
</scenario>
"""


def stamp_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def read_bag(bag: Path, robot: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The bag's thruster commands [t, 8 commands] and odometry [t, x, y, z, qx, qy, qz, qw,
    u, v, w, p, q, r] (pose in odom ENU, velocity in base_link FLU), by their stamps."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=""),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    command_topic, odometry_topic = f"/{robot}/control/thrusters", f"/{robot}/odometry/filtered"
    reader.set_filter(rosbag2_py.StorageFilter(topics=[command_topic, odometry_topic]))
    commands, odometry = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == command_topic:
            msg = deserialize_message(data, ThrusterCommand)
            commands.append([stamp_seconds(msg.header.stamp), *msg.command])
        else:
            msg = deserialize_message(data, Odometry)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            v, w = msg.twist.twist.linear, msg.twist.twist.angular
            pose = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            odometry.append([stamp_seconds(msg.header.stamp), *pose, v.x, v.y, v.z, w.x, w.y, w.z])
    if not commands or not odometry:
        raise SystemExit(f"{bag} has no {command_topic} or no {odometry_topic}")
    commands, odometry = np.array(commands), np.array(odometry)
    return commands[np.argsort(commands[:, 0])], odometry[np.argsort(odometry[:, 0])]


def rotation(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rpy(R: np.ndarray) -> tuple[float, float, float]:
    """Roll, pitch and yaw of R = Rz(yaw) Ry(pitch) Rx(roll), as Stonefish's rpy."""
    return (
        math.atan2(R[2, 1], R[2, 2]),
        math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0])),
        math.atan2(R[1, 0], R[0, 0]),
    )


class Simulator:
    """A paused stonefish_simulator_nogpu with the robot alone in open water, stepped on request."""

    def __init__(self, node: Node, scenario: Path, robot: str, domain: int):
        data = Path(get_package_share_directory("sub_sim")) / "data"
        self.process = subprocess.Popen(
            [
                *("ros2", "run", "stonefish_ros2", "stonefish_simulator_nogpu"),
                *(str(data), str(scenario), str(SIMULATION_RATE)),
                *("--ros-args", "-p", "start_paused:=true", "-p", "publish_clock:=true"),
                *("-p", "seed:=0"),
            ],
            env={**os.environ, "ROS_DOMAIN_ID": str(domain)},
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        self.node = node
        self.clock = 0.0
        self.velocities: list[list[float]] = []
        self.subscriptions = [
            node.create_subscription(
                Clock,
                "/clock",
                self._on_clock,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            ),
            node.create_subscription(Odometry, f"/{robot}/sim/odometry", self._on_odometry, 100),
        ]
        self.setpoints = node.create_publisher(
            Float64MultiArray, f"/{robot}/sim/thruster_setpoints", 10
        )
        self.step_client = node.create_client(Step, "/step")
        if not self.step_client.wait_for_service(timeout_sec=30.0):
            self.close()
            raise SystemExit("The simulator did not start")
        # Until the setpoint publisher is matched, the first commands would be lost.
        deadline = time.monotonic() + 10.0
        while self.setpoints.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

    def _on_clock(self, msg: Clock):
        self.clock = stamp_seconds(msg.clock)

    def _on_odometry(self, msg: Odometry):
        # Ground truth: the velocity of base_link_ned (X right, Y back, Z down) in its
        # own axes, turned into base_link's (FLU).
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        self.velocities.append(
            [stamp_seconds(msg.header.stamp), -v.y, -v.x, -v.z, -w.y, -w.x, -w.z]
        )

    def advance(self, steps: int, command: np.ndarray):
        """Applies the command and simulates `steps` steps."""
        if steps <= 0:
            return
        clamped = np.clip(np.nan_to_num(command), -MAX_COMMAND, MAX_COMMAND)
        self.setpoints.publish(Float64MultiArray(data=[float(c) for c in clamped]))
        rclpy.spin_once(self.node, timeout_sec=0.0)
        future = self.step_client.call_async(Step.Request(steps=steps))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        if future.result() is None or not future.result().success:
            raise SystemExit("The simulator did not step")
        deadline = time.monotonic() + 30.0
        while self.clock < future.result().time - 1e-9 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def close(self):
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.node.destroy_publisher(self.setpoints)
        self.node.destroy_client(self.step_client)
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    parser.add_argument("bag", type=Path, help="the recorded run (a rosbag2 directory)")
    parser.add_argument("--robot", default="marlin_v3")
    parser.add_argument(
        "--gains",
        default="control/gains_sim.yaml",
        help="the gains file whose model block is the simulated vehicle",
    )
    parser.add_argument("--window", type=float, default=10.0, help="length of a window [s]")
    parser.add_argument(
        "--warmup", type=float, default=3.0, help="start of each window left out of the errors [s]"
    )
    parser.add_argument(
        "--min-depth",
        type=float,
        default=0.4,
        help="depth of base_link above which the vehicle is at the surface and not compared [m]",
    )
    parser.add_argument("--start", type=float, default=0.0, help="where in the bag to start [s]")
    parser.add_argument("--duration", type=float, default=math.inf, help="how much to replay [s]")
    parser.add_argument("--battery-voltage", type=float, default=16.0)
    parser.add_argument("--out", type=Path, default=Path("replay"), help="output files' prefix")
    parser.add_argument(
        "--domain", type=int, default=90 + os.getpid() % 10, help="ROS domain to run in"
    )
    args = parser.parse_args()

    commands, odometry = read_bag(args.bag, args.robot)
    # Only while the thrusters were being driven.
    driven = commands[np.any(np.abs(commands[:, 1:]) > 1e-3, axis=1)]
    if len(driven) == 0:
        raise SystemExit("The bag's thruster commands are all zero")
    begin = max(driven[0, 0], odometry[0, 0]) + args.start
    end = min(driven[-1, 0], odometry[-1, 0], begin + args.duration)
    if end - begin < args.warmup + 1.0:
        raise SystemExit("Too little of the bag has both commands and odometry")

    vehicle = load_vehicle(args.robot)
    gains = Path(get_package_share_directory("sub_bringup")) / "config" / args.gains
    model = yaml.safe_load(gains.read_text())["/**"]["ros__parameters"]["model"]
    base_link_ned = np.array(base_link_ned_pose(vehicle).R)
    robot_template = (
        Path(get_package_share_directory("sub_sim"))
        / "data"
        / "robots"
        / args.robot
        / "layout.scn.j2"
    )

    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    rclpy.init()
    node = Node("replay")
    rows = []
    try:
        # Where the recorded vehicle is nearly at rest: 3 cm/s and 0.03 rad/s.
        quiet = (np.linalg.norm(odometry[:, 8:11], axis=1) < 0.03) & (
            np.linalg.norm(odometry[:, 11:14], axis=1) < 0.03
        )
        window_start = begin
        while window_start + args.warmup + 1.0 <= end:
            # The window's length, then on to the next quiet moment.
            later = (odometry[:, 0] >= window_start + args.window) & quiet
            window_end = odometry[np.argmax(later), 0] if later.any() else end
            window_end = min(window_end, window_start + 2.0 * args.window, end)
            # The vehicle as the bag has it at the window's start: odom is ENU and
            # base_link FLU, the simulator's world NED and its base link base_link_ned.
            state = odometry[
                np.searchsorted(odometry[:, 0], window_start).clip(0, len(odometry) - 1)
            ]
            enu_to_ned = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
            roll, pitch, yaw = rpy(enu_to_ned @ rotation(state[4:8]) @ base_link_ned)
            robot = render_robot_scenario(
                robot_template,
                z=max(-state[3], 0.3),
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                thrusters=thruster_specs(vehicle.thrusters),
                frames=mount_poses(vehicle),
                sensor_noise=False,
                cameras=False,
                hydrodynamics=model_hydrodynamics(model, base_link_ned_pose(vehicle)),
                battery_voltage=args.battery_voltage,
            )
            with tempfile.NamedTemporaryFile("w", suffix=".scn", delete=False) as scenario:
                scenario.write(SCENARIO.format(robot=robot.as_posix()))
            simulator = Simulator(node, Path(scenario.name), args.robot, args.domain)
            try:
                window = commands[(commands[:, 0] >= window_start) & (commands[:, 0] < window_end)]
                simulated = 0.0
                for i, row in enumerate(window):
                    until = (window[i + 1, 0] if i + 1 < len(window) else window_end) - window_start
                    steps = round((until - simulated) * SIMULATION_RATE)
                    simulator.advance(steps, row[1:])
                    simulated += steps / SIMULATION_RATE
                for _ in range(20):
                    rclpy.spin_once(node, timeout_sec=0.01)
                sim = np.array(simulator.velocities)
            finally:
                simulator.close()
                Path(scenario.name).unlink(missing_ok=True)
                robot.unlink(missing_ok=True)
            if len(sim) > 1:
                # The simulator's time starts at zero in every window.
                t = sim[:, 0] - sim[0, 0]
                recorded = np.column_stack(
                    [
                        np.interp(window_start + t, odometry[:, 0], odometry[:, 8 + k])
                        for k in range(6)
                    ]
                )
                depth = -np.interp(window_start + t, odometry[:, 0], odometry[:, 3])
                scored = (t >= args.warmup) & (depth >= args.min_depth)
                rows += [
                    [window_start - begin + t[i], scored[i], *recorded[i], *sim[i, 1:]]
                    for i in range(len(t))
                ]
            print(f"replayed {window_end - begin:6.1f} of {end - begin:.1f} s", file=sys.stderr)
            window_start = window_end
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not rows:
        raise SystemExit("The simulator reported no motion")
    data = np.array(rows, dtype=float)
    scored = data[data[:, 1] > 0.5]
    recorded, simulated = scored[:, 2:8], scored[:, 8:14]
    print(f"\n{args.bag.name}: {len(scored) / 50.0:.0f} s compared, model of {args.gains}")
    print(f"{'axis':<11}{'RMS error':>11}{'RMS recorded':>14}{'error / recorded':>18}")
    for k, axis in enumerate(AXES):
        error = math.sqrt(float(np.mean((simulated[:, k] - recorded[:, k]) ** 2)))
        size = math.sqrt(float(np.mean(recorded[:, k] ** 2)))
        unit = "m/s" if k < 3 else "rad/s"
        ratio = f"{100.0 * error / size:.0f}%" if size > 1e-3 else "-"
        print(f"{axis:<11}{error:>7.3f} {unit:<5}{size:>8.3f} {unit:<5}{ratio:>12}")

    csv_path = args.out.with_suffix(".csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["time", "scored"] + [f"recorded {a}" for a in AXES] + [f"simulated {a}" for a in AXES]
        )
        writer.writerows(data.tolist())
    print(f"\nvelocities: {csv_path}")

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    figure, plots = plt.subplots(6, 1, figsize=(11, 13), sharex=True)
    for k, (axis, plot) in enumerate(zip(AXES, plots, strict=True)):
        plot.plot(data[:, 0], data[:, 2 + k], label="recorded", linewidth=1.0)
        plot.plot(data[:, 0], data[:, 8 + k], label="simulated", linewidth=1.0)
        plot.set_ylabel(f"{axis} [{'m/s' if k < 3 else 'rad/s'}]")
        plot.grid(True, alpha=0.3)
    plots[0].legend(loc="upper right", ncol=2)
    plots[-1].set_xlabel("time [s]")
    figure.suptitle(f"{args.bag.name}: recorded and simulated velocity (base_link)")
    figure.tight_layout()
    figure.savefig(args.out.with_suffix(".png"), dpi=110)
    print(f"plot: {args.out.with_suffix('.png')}")


if __name__ == "__main__":
    main()
