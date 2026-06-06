#!/usr/bin/env python3
"""Automatic cascade-PID tuner for sub_control (marlin_v2).

Drives the running simulator over live ROS parameters: for each PID loop it
applies a step setpoint, records the per-axis error topic the controller already
publishes, scores the response, and searches kp/ki/kd with coordinate-descent
("twiddle"). Inner loops (vel, ang) are tuned before the outer loops (pos, att)
that depend on them. The best gains are written back to control_gains.yaml so the
next launch picks them up.

Prereqs: workspace built and sourced (so `sub_control_interfaces` imports and the
`sub_control` node exposes gains.* params -- i.e. this branch's sub_control).

Usage (auto-launches sim, tunes everything, writes YAML, shuts sim down):
    python3 src/sub_bringup/scripts/tune_pid.py

Connect to an already-running `ros2 launch sub_bringup sim_launch.py`:
    python3 src/sub_bringup/scripts/tune_pid.py --no-launch

Tune a subset and iterate faster:
    python3 src/sub_bringup/scripts/tune_pid.py --no-launch \
        --loops gains.vel.z,gains.ang.z --budget 16 --window 5
"""

import argparse
import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
import yaml
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float64
from sub_control_interfaces.msg import Setpoint

# --- Loop catalogue -------------------------------------------------------
# key            : parameter prefix (gains.<loop>.<axis>)
# error_topic    : topic the controller publishes that loop's error on
# axis           : 0/1/2 index into the setpoint Vector3
# via            : how to excite it -- which setpoint topic + velocity flag
# mag            : step magnitude (m, m/s, rad, or rad/s depending on loop)
#
# Order matters: inner loops (vel/ang) first so the outer loops they feed are
# tuned against already-good inner dynamics.
Loop = lambda key, topic, axis, via, mag: dict(
    key=key, topic=topic, axis=axis, via=via, mag=mag
)
ALL_LOOPS = [
    # inner velocity (force) -- excited with a body-velocity setpoint
    Loop("gains.vel.x", "/control/vel/x", 0, "pos_vel", 0.5),
    Loop("gains.vel.y", "/control/vel/y", 1, "pos_vel", 0.5),
    Loop("gains.vel.z", "/control/vel/z", 2, "pos_vel", 0.4),
    # inner angular rate (moment) -- excited with an angular-rate setpoint
    Loop("gains.ang.x", "/control/angvel/x", 0, "att_vel", 0.3),
    Loop("gains.ang.y", "/control/angvel/y", 1, "att_vel", 0.3),
    Loop("gains.ang.z", "/control/angvel/z", 2, "att_vel", 0.4),
    # outer position -- excited with a position step (drives the vel loop)
    Loop("gains.pos.x", "/control/pos/x", 0, "pos", 1.0),
    Loop("gains.pos.y", "/control/pos/y", 1, "pos", 1.0),
    Loop("gains.pos.z", "/control/pos/z", 2, "pos", 0.5),
    # outer attitude -- excited with an attitude step (drives the ang loop)
    Loop("gains.att.x", "/control/ang/x", 0, "att", 0.3),
    Loop("gains.att.y", "/control/ang/y", 1, "att", 0.3),
    Loop("gains.att.z", "/control/ang/z", 2, "att", 0.5),
]
ALL_TOPICS = [lp["topic"] for lp in ALL_LOOPS]

UNSTABLE_COST = 1e6  # returned for diverging / NaN responses


class Tuner(Node):
    def __init__(self, node_name: str):
        super().__init__("pid_tuner")
        ns = node_name.rsplit("/", 1)[0] or ""
        self.pos_pub = self.create_publisher(Setpoint, f"{ns}/pos_setpoint", 10)
        self.att_pub = self.create_publisher(Setpoint, f"{ns}/att_setpoint", 10)
        self.set_cli = self.create_client(
            SetParameters, f"{node_name}/set_parameters"
        )

        # Drive the sim kill-switch relay (sub_sim_sensors/sim_kill_switch), which
        # forwards sim/kill_switch -> kill_switch once its startup window is over.
        # Latched, to match the relay's transient_local subscriber.
        kill_qos = QoSProfile(depth=1)
        kill_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        kill_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.kill_pub = self.create_publisher(Bool, f"{ns}/sim/kill_switch", kill_qos)

        self._lock = threading.Lock()
        self._active_topic = None
        self._recording = False
        self._samples = []
        self._seen = set()  # topics we've received at least once (readiness)
        for topic in ALL_TOPICS:
            self.create_subscription(Float64, topic, self._make_cb(topic), 10)

    def _make_cb(self, topic):
        def cb(msg):
            self._seen.add(topic)
            with self._lock:
                if self._recording and topic == self._active_topic:
                    self._samples.append((time.monotonic(), msg.data))

        return cb

    # --- setpoint helpers ---
    def _send(self, pub, vec, velocity):
        m = Setpoint()
        m.velocity = velocity
        m.use_altitude = False
        m.setpoint.x, m.setpoint.y, m.setpoint.z = float(vec[0]), float(vec[1]), float(vec[2])
        pub.publish(m)

    def set_kill(self, value):
        """Engage (True) / release (False) sub_control's kill switch."""
        self.kill_pub.publish(Bool(data=bool(value)))

    def command(self, loop, value):
        vec = [0.0, 0.0, 0.0]
        vec[loop["axis"]] = value
        if loop["via"] in ("pos_vel", "pos"):
            self._send(self.pos_pub, vec, velocity=(loop["via"] == "pos_vel"))
        else:
            self._send(self.att_pub, vec, velocity=(loop["via"] == "att_vel"))

    # --- recording ---
    def start_record(self, topic):
        with self._lock:
            self._active_topic = topic
            self._samples = []
            self._recording = True

    def stop_record(self):
        with self._lock:
            self._recording = False
            return list(self._samples)

    # --- parameters ---
    def set_gains(self, key, kp, ki, kd, timeout=5.0):
        req = SetParameters.Request()
        for name, val in ((f"{key}.kp", kp), (f"{key}.ki", ki), (f"{key}.kd", kd)):
            pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(val))
            req.parameters.append(Parameter(name=name, value=pv))
        fut = self.set_cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.01)
        return fut.done() and all(r.successful for r in fut.result().results)


def score(samples, mag, window):
    """Lower is better. Normalised IAE + steady-state error + overshoot."""
    if len(samples) < 5:
        return UNSTABLE_COST
    t0 = samples[0][0]
    ts = [s[0] - t0 for s in samples]
    es = [s[1] / mag for s in samples]  # normalise by step size
    if any(math.isnan(e) or math.isinf(e) or abs(e) > 8.0 for e in es):
        return UNSTABLE_COST

    iae = 0.0
    for i in range(1, len(es)):
        dt = ts[i] - ts[i - 1]
        iae += 0.5 * (abs(es[i]) + abs(es[i - 1])) * dt

    # error is (setpoint - meas) starting near +1 and decaying to 0; an excursion
    # below 0 is overshoot past the target.
    overshoot = max(0.0, max(-e for e in es))
    tail = es[int(0.75 * len(es)):] or es[-1:]
    sse = sum(abs(e) for e in tail) / len(tail)

    return iae + 8.0 * sse * window + 3.0 * overshoot * window


class Orchestrator:
    def __init__(self, tuner, args):
        self.t = tuner
        self.a = args
        self.sign = 1

    def settle(self, loop, seconds):
        """Hold the loop's zero setpoint so the sub stops drifting between trials."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.t.command(loop, 0.0)
            time.sleep(0.1)

    def trial(self, loop, gains):
        if not self.t.set_gains(loop["key"], *gains):
            self.t.get_logger().warn(f"set_gains failed for {loop['key']}")
            return UNSTABLE_COST
        self.settle(loop, self.a.settle)

        self.sign *= -1  # alternate direction to keep net drift bounded
        self.t.start_record(loop["topic"])
        self.t.command(loop, self.sign * loop["mag"])
        time.sleep(self.a.window)
        samples = self.t.stop_record()
        cost = score(samples, loop["mag"], self.a.window)
        return cost

    def twiddle(self, loop, p0):
        """Coordinate descent on [kp, ki, kd] with adaptive step size."""
        p = list(p0)
        kp_scale = p[0] if p[0] > 1e-9 else 1.0
        dp = [0.30 * kp_scale, 0.08 * kp_scale, 0.10 * kp_scale]
        best = self.trial(loop, p)
        evals = 1
        self.t.get_logger().info(
            f"[{loop['key']}] start {fmt(p)} cost={best:.3f}"
        )
        while evals < self.a.budget and sum(dp) > 1e-4 * kp_scale:
            for i in range(3):
                if evals >= self.a.budget:
                    break
                p[i] += dp[i]
                c = self.trial(loop, p)
                evals += 1
                if c < best:
                    best = c
                    dp[i] *= 1.1
                    continue
                p[i] = max(0.0, p[i] - 2 * dp[i])  # gains stay non-negative
                c = self.trial(loop, p)
                evals += 1
                if c < best:
                    best = c
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
            self.t.get_logger().info(
                f"[{loop['key']}] eval {evals}/{self.a.budget} "
                f"best={best:.3f} {fmt(p)}"
            )
        # lock in the winner
        self.t.set_gains(loop["key"], *p)
        return p, best


def fmt(p):
    return f"kp={p[0]:.4g} ki={p[1]:.4g} kd={p[2]:.4g}"


def load_yaml_gains(path):
    """Return {key: [kp,ki,kd]} from a control_gains.yaml, or {} if absent."""
    if not path.exists():
        return {}
    data = yaml.safe_load(path.read_text()) or {}
    root = (data.get("/**", {}) or {}).get("ros__parameters", {}).get("gains", {})
    out = {}
    for loop, axes in root.items():
        for axis, g in axes.items():
            out[f"gains.{loop}.{axis}"] = [
                float(g.get("kp", 0.0)), float(g.get("ki", 0.0)), float(g.get("kd", 0.0))
            ]
    return out


def write_yaml_gains(path, gains):
    tree = {}
    for key, (kp, ki, kd) in sorted(gains.items()):
        _, loop, axis = key.split(".")
        tree.setdefault(loop, {})[axis] = {
            "kp": round(kp, 5), "ki": round(ki, 5), "kd": round(kd, 5)
        }
    doc = {"/**": {"ros__parameters": {"gains": tree}}}
    header = (
        "# Cascade PID gains for sub_control. Written by tune_pid.py.\n"
        "# parameter name: gains.<loop>.<axis>.<k>\n"
    )
    path.write_text(header + yaml.safe_dump(doc, default_flow_style=False, sort_keys=False))


def wait_ready(tuner, timeout):
    tuner.get_logger().info("Waiting for sub_control set_parameters service...")
    if not tuner.set_cli.wait_for_service(timeout_sec=timeout):
        return False
    tuner.get_logger().info("Waiting for control loop to publish errors...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if "/control/vel/x" in tuner._seen and "/control/angvel/z" in tuner._seen:
            return True
        time.sleep(0.2)
    return False


def main():
    repo = Path(__file__).resolve().parents[3]  # .../thalassic
    default_yaml = repo / "src/sub_bringup/config/control_gains.yaml"

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--node", default="/marlin_v2/sub_control",
                    help="fully-qualified sub_control node name")
    ap.add_argument("--loops", default="all",
                    help="'all', 'inner', 'outer', or comma-separated gains.* keys")
    ap.add_argument("--budget", type=int, default=24, help="trials per loop")
    ap.add_argument("--window", type=float, default=6.0, help="step record seconds")
    ap.add_argument("--settle", type=float, default=3.0, help="settle seconds between trials")
    ap.add_argument("--yaml", type=Path, default=default_yaml, help="gains YAML to update")
    ap.add_argument("--launch", default=True, action=argparse.BooleanOptionalAction,
                    help="auto-launch sim_launch.py (use --no-launch if already running)")
    ap.add_argument("--ready-timeout", type=float, default=180.0)
    args = ap.parse_args()

    if args.loops == "all":
        loops = ALL_LOOPS
    elif args.loops == "inner":
        loops = [lp for lp in ALL_LOOPS if lp["key"].split(".")[1] in ("vel", "ang")]
    elif args.loops == "outer":
        loops = [lp for lp in ALL_LOOPS if lp["key"].split(".")[1] in ("pos", "att")]
    else:
        wanted = set(args.loops.split(","))
        loops = [lp for lp in ALL_LOOPS if lp["key"] in wanted]
    if not loops:
        ap.error("no loops selected")

    sim = None
    if args.launch:
        print(">> launching: ros2 launch sub_bringup sim_launch.py")
        sim = subprocess.Popen(
            ["ros2", "launch", "sub_bringup", "sim_launch.py"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

    rclpy.init()
    tuner = Tuner(args.node)
    executor = MultiThreadedExecutor()
    executor.add_node(tuner)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    results = load_yaml_gains(args.yaml)  # seed from existing/best-known gains
    try:
        # Release the kill switch before tuning. sub_control publishes nothing
        # (including the error topics wait_ready needs) while killed, so arm first.
        # The relay ignores this during its startup window (a fresh sim releases
        # itself after a few seconds), but on an already-running sim the ON->OFF
        # edge resets the origin to the current pose for a clean (0,0)/yaw=0 run.
        print(">> arming sub via sim/kill_switch (resets origin to current pose)")
        tuner.set_kill(True)
        time.sleep(1.0)
        tuner.set_kill(False)

        if not wait_ready(tuner, args.ready_timeout):
            print("!! sim/control not ready -- aborting")
            return 1
        print(">> system ready, starting tune\n")
        time.sleep(2.0)

        orch = Orchestrator(tuner, args)
        for lp in loops:
            seed = results.get(lp["key"])
            if seed is None:
                # fall back to whatever the node currently reports via defaults
                seed = {"gains.pos": [0.8, 0, 0], "gains.att": [1.2, 0, 0],
                        "gains.vel": [40, 8, 2], "gains.ang": [6, 0, 1]}.get(
                    ".".join(lp["key"].split(".")[:2]), [1.0, 0.0, 0.0])
            best_p, best_c = orch.twiddle(lp, seed)
            results[lp["key"]] = best_p
            print(f"== {lp['key']}: {fmt(best_p)} cost={best_c:.3f}\n")
            write_yaml_gains(args.yaml, results)  # persist after every loop

        print(">> done. Tuned gains written to", args.yaml)
        for lp in loops:
            print(f"   {lp['key']:14s} {fmt(results[lp['key']])}")
    finally:
        # Re-engage the kill switch so the sub is safed once tuning ends.
        try:
            tuner.set_kill(True)
            time.sleep(0.5)
        except Exception:
            pass
        tuner.destroy_node()
        rclpy.shutdown()
        if sim is not None:
            print(">> shutting sim down")
            os.killpg(os.getpgid(sim.pid), signal.SIGINT)
            try:
                sim.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
