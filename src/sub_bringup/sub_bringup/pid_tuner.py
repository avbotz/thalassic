#!/usr/bin/env python3
"""Automatic, resumable PID tuner for sub_control (simulator and real sub).

sub_control reads its gains once at startup (declare-only parameters), so the
tuner owns the sub_control process: for every candidate gain set it writes a
params file and (re)starts sub_control with it, then runs a step-response
trial through the normal setpoint topics and scores the tracking error from
control/error. A per-axis coordinate-descent (Twiddle) optimizer proposes the
next candidate. All progress is checkpointed to disk after every trial, so
the tuner can be stopped (Ctrl+C, battery pull, crash) and rerun to resume.

Safety: every phase of a trial — including the settle back to the home pose —
runs under a watchdog that checks the depth/attitude/speed envelope AND an
oscillation detector on the control/error channels. A candidate that starts
limit-cycling is cut off within a couple of cycles: sub_control is stopped
(thrusters zeroed), the sub coasts, and the candidate is scored as a failure.
When a candidate fails while probing a gain upward, the exploration ceiling
for that gain is pulled down below the failed value so the unstable region is
never probed again. If the *baseline* gains themselves are unstable, the
tuner shrinks them and re-baselines instead of accepting garbage.

Kill switch: the tuner listens to the latched kill_switch topic. A kill at
any point aborts the current trial without scoring it; the tuner waits for
unkill, re-homes (sub_control zeroes the EKF pose on unkill), and re-runs the
same trial. Killing the same candidate --max-kill-rejects times (default 2)
is treated as an operator veto: the candidate is scored as a failure and the
optimizer backs away from it.

Usage (stack must already be running; the tuner replaces its sub_control):

  Simulator:  ros2 launch sub_bringup sim_launch.py
              ros2 run sub_bringup pid_tuner --mode sim
  Real sub:   ros2 launch sub_bringup pool_test_launch.py
              ros2 run sub_bringup pid_tuner --mode real --home-depth -1.5

  Pick loops: ros2 run sub_bringup pid_tuner --mode sim --loops vel.z,ang.z
  Gentler:    ros2 run sub_bringup pid_tuner --mode real --step-scale 0.6
  Progress:   ros2 run sub_bringup pid_tuner --mode sim --status
  Start over: ros2 run sub_bringup pid_tuner --mode sim --fresh

Loops are named <group>.<axis> with groups vel/ang/pos/att (matching the
vel_pid/ang_pid/pos_pid/att_pid parameters); inner loops are tuned before the
outer loops that command them. Within a loop, gains are probed in the order
kp -> kd -> ki (damping before integral action). Tuned gains are continuously
written to <state-dir>/tuned_gains.yaml in the control_gains.yaml format —
copy it over src/sub_bringup/config/control_gains{_sim}.yaml when satisfied.

All frames are REP-103 FLU/ENU like the rest of the stack: z is up, so
depths (--home-depth, --z-min, --z-max) are negative underwater.
"""

import argparse
import copy
import csv
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64
from sub_control_interfaces.msg import Error, Setpoint

STATE_VERSION = 2
FAIL_COST = 1.0e6
NUM_THRUSTERS = 8
GROUP_TO_PARAM = {"pos": "pos_pid", "vel": "vel_pid", "att": "att_pid", "ang": "ang_pid"}
AXIS_INDEX = {"x": 0, "y": 1, "z": 2}
GAIN_NAMES = ("kp", "ki", "kd")
# Probe kp first, then kd (damping), then ki (integral action last): raising
# ki before the loop is damped is the classic way to create a limit cycle.
GAIN_ORDER = (0, 2, 1)
# Weight of thruster-command roughness (mean |delta u| per 20 ms cycle) in the
# trial cost: penalizes gains that track well but hammer the thrusters.
CHATTER_WEIGHT = 10.0
# Inner loops (velocity/rate) first: the outer loops command them.
DEFAULT_PLAN = ["vel.z", "vel.x", "vel.y", "ang.z", "pos.z", "pos.x", "pos.y", "att.z"]
ALL_LOOPS = [
    "vel.z", "vel.x", "vel.y", "ang.z", "ang.x", "ang.y",
    "pos.z", "pos.x", "pos.y", "att.z", "att.x", "att.y",
]


class KillAbort(Exception):
    """Kill switch went active while a trial was in progress."""


class StackDead(Exception):
    """sub_control died or control/error went stale while unkilled."""


class SettleTimeout(Exception):
    """The sub could not reach the trial start state in time."""


class WatchdogAbort(Exception):
    """Safety envelope or oscillation detector tripped."""


def norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------------------
# Twiddle (coordinate descent) as an explicit, JSON-serializable state machine
# so a trial boundary is always a valid checkpoint.
# ---------------------------------------------------------------------------

def new_twiddle(gains3: list, budget: int, trials_used: int = 0) -> dict:
    kp = gains3[0]
    if kp <= 1e-9:
        raise ValueError("cannot seed tuner from a zero kp")
    deltas, lo, hi = [], [], []
    # Exploration steps and ceilings are deliberately tight: the tuner refines
    # around a working point, it does not survey gain space. Ceilings for
    # ki/kd are additionally capped relative to kp so integral/derivative
    # action can never dwarf the proportional term.
    seeds = [0.15 * kp, 0.08 * kp, 0.05 * kp]
    caps = [3.0 * kp, 0.8 * kp, 0.8 * kp]
    rel_caps = [3.0 * kp, 1.5 * kp, 1.2 * kp]
    for i, g in enumerate(gains3):
        deltas.append(0.15 * g if g > 1e-9 else seeds[i])
        lo.append(max(0.15 * kp, 1e-3) if i == 0 else 0.0)
        hi.append(min(3.0 * g, rel_caps[i]) if g > 1e-9 else caps[i])
    return {
        "params": [float(g) for g in gains3],
        "deltas": deltas,
        "lo": lo,
        "hi": hi,
        "best_cost": None,
        "ord": 0,
        "phase": "baseline",  # baseline -> up -> down per gain
        "cand": None,
        "trials": int(trials_used),
        "budget": int(budget),
        "baseline_fails": 0,
        "done": False,
    }


def twiddle_idx(st: dict) -> int:
    """Index into params of the gain currently being probed."""
    return GAIN_ORDER[st["ord"] % 3]


def twiddle_candidate(st: dict) -> list:
    p, d = st["params"], st["deltas"]
    i = twiddle_idx(st)
    cand = list(p)
    if st["phase"] != "baseline":
        step = d[i] if st["phase"] == "up" else -d[i]
        cand[i] = min(max(p[i] + step, st["lo"][i]), st["hi"][i])
    st["cand"] = cand
    return cand


def _twiddle_advance(st: dict):
    st["phase"] = "up"
    st["ord"] = (st["ord"] + 1) % 3
    scale = sum(abs(g) for g in st["params"]) + 1e-9
    if sum(st["deltas"]) / scale < 0.03:
        st["done"] = True


def twiddle_record(st: dict, cost: float, count_trial: bool = True) -> bool:
    """Feed the cost of the last candidate back in. Returns True on improvement."""
    if count_trial:
        st["trials"] += 1
    i = twiddle_idx(st)
    improved = False
    if st["phase"] == "baseline":
        st["best_cost"] = cost
        st["phase"] = "up"
    elif cost < st["best_cost"] and cost < FAIL_COST:
        st["best_cost"] = cost
        st["params"] = list(st["cand"])
        # Grow slower than we shrink, and never let a step exceed a third of
        # the allowed range: improvements must not launch the next probe deep
        # into untested territory.
        st["deltas"][i] = min(1.1 * st["deltas"][i], 0.35 * st["hi"][i])
        improved = True
        _twiddle_advance(st)
    elif st["phase"] == "up":
        st["phase"] = "down"
    else:
        st["deltas"][i] *= 0.7
        _twiddle_advance(st)
    if st["trials"] >= st["budget"]:
        st["done"] = True
    return improved


def twiddle_carve_ceiling(st: dict, status: str):
    """After an instability-type failure while probing a gain upward, pull
    that gain's ceiling below the failed value so the unstable region is
    never probed again."""
    if st["phase"] != "up" or status not in ("watchdog", "no-settle"):
        return
    i = twiddle_idx(st)
    if st["cand"][i] > st["params"][i]:
        st["hi"][i] = max(st["params"][i], min(st["hi"][i], 0.9 * st["cand"][i]))


# ---------------------------------------------------------------------------
# Trial scoring
# ---------------------------------------------------------------------------

def score_step_response(samples: list, step: float, duration: float):
    """Cost of one step response: ITAE + settling time + overshoot + oscillation.

    samples: [(t_since_step, error)] where error should decay from ~step to 0.
    Returns None if too little data arrived to judge the trial.
    """
    if abs(step) < 1e-9 or len(samples) < max(10, int(10 * duration)):
        return None
    s = 1.0 if step > 0 else -1.0
    itae = 0.0
    prev_t = samples[0][0]
    overshoot = 0.0
    crossings = 0
    prev_sign = 0
    deadband = 0.03 * abs(step)
    settle_band = 0.10 * abs(step)
    t_settled = 0.0
    for t, e in samples:
        dt = max(t - prev_t, 0.0)
        prev_t = t
        itae += t * abs(e) * dt
        overshoot = max(overshoot, -e * s)
        if abs(e) > settle_band:
            t_settled = t
        if abs(e) > deadband:
            sign = 1 if e * s > 0 else -1
            if prev_sign != 0 and sign != prev_sign:
                crossings += 1
            prev_sign = sign
    itae_norm = itae / (abs(step) * duration * duration / 2.0)
    overshoot_frac = overshoot / abs(step)
    return (
        itae_norm
        + t_settled / duration
        + 3.0 * max(0.0, overshoot_frac - 0.05)
        + 0.5 * max(0, crossings - 1)
    )


# ---------------------------------------------------------------------------
# ROS node: latched kill switch, error/odom mirrors, setpoint publishers,
# rolling error history for the oscillation detector, thruster roughness.
# ---------------------------------------------------------------------------

ERR_CHANNELS = (
    [("pos", i, 0.12) for i in range(3)]        # m
    + [("att", i, 0.08) for i in range(3)]      # rad
    + [("vel", i, 0.12) for i in range(3)]      # m/s
    + [("angvel", i, 0.25) for i in range(3)]   # rad/s
)


class TunerNode(Node):
    def __init__(self, robot_name: str):
        super().__init__("pid_tuner", namespace=robot_name)
        self._lock = threading.Lock()
        self.killed = None  # None = no kill publisher seen yet
        self.err = None
        self.err_time = 0.0
        self.odom = None
        self.odom_time = 0.0
        self._extract = None
        self._rec_t0 = 0.0
        self._rec_buf = []
        self._err_hist = deque()  # (t, 12 error channels), ~6 s rolling
        self._u_prev = [None] * NUM_THRUSTERS
        self._u_tv = 0.0
        self._u_n = 0

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "kill_switch", self._kill_cb, latched)
        self.create_subscription(Error, "control/error", self._err_cb, 10)
        self.create_subscription(Odometry, "odometry/filtered", self._odom_cb, 10)
        for i in range(NUM_THRUSTERS):
            self.create_subscription(
                Float64, f"control/thruster_{i}",
                lambda msg, i=i: self._thruster_cb(i, msg), 10)

        self._pos_pub = self.create_publisher(Setpoint, "pos_setpoint", 10)
        self._att_pub = self.create_publisher(Setpoint, "att_setpoint", 10)
        self._thruster_pubs = [
            self.create_publisher(Float64, f"control/thruster_{i}", 10)
            for i in range(NUM_THRUSTERS)
        ]

    def _kill_cb(self, msg: Bool):
        with self._lock:
            prev = self.killed
            self.killed = bool(msg.data)
        if msg.data and prev is not True:
            self.get_logger().warn("kill switch ACTIVE — pausing tuning")
        elif not msg.data and prev is True:
            self.get_logger().info("kill switch released")

    def _err_cb(self, msg: Error):
        now = time.monotonic()
        chans = tuple(msg.pos_error) + tuple(msg.att_error) \
            + tuple(msg.vel_error) + tuple(msg.angvel_error)
        with self._lock:
            self.err = msg
            self.err_time = now
            self._err_hist.append((now, chans))
            while self._err_hist and now - self._err_hist[0][0] > 6.0:
                self._err_hist.popleft()
            if self._extract is not None:
                self._rec_buf.append((now - self._rec_t0, self._extract(msg)))

    def _thruster_cb(self, i: int, msg: Float64):
        with self._lock:
            if self._extract is not None and self._u_prev[i] is not None:
                self._u_tv += abs(msg.data - self._u_prev[i])
                self._u_n += 1
            self._u_prev[i] = msg.data

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        with self._lock:
            self.odom = {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "speed": math.sqrt(v.x**2 + v.y**2 + v.z**2),
                "ang_speed": math.sqrt(w.x**2 + w.y**2 + w.z**2),
            }
            self.odom_time = time.monotonic()

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "killed": self.killed,
                "err": self.err,
                "err_age": time.monotonic() - self.err_time if self.err else None,
                "odom": dict(self.odom) if self.odom else None,
                "odom_age": time.monotonic() - self.odom_time if self.odom else None,
            }

    def oscillation(self, window: float, min_flips: int):
        """Check every control/error channel for a sustained limit cycle:
        min_flips sign changes within the window, each with the error beyond
        that channel's deadband on both sides. Returns a description string
        of the worst offender, or None."""
        now = time.monotonic()
        with self._lock:
            hist = [s for s in self._err_hist if now - s[0] <= window]
        for ch, (fam, ax, db) in enumerate(ERR_CHANNELS):
            flips = 0
            prev_sign = 0
            for _, chans in hist:
                v = chans[ch]
                if abs(v) > db:
                    sign = 1 if v > 0 else -1
                    if prev_sign != 0 and sign != prev_sign:
                        flips += 1
                    prev_sign = sign
            if flips >= min_flips:
                return (f"{fam}_error[{'xyz'[ax]}] oscillating "
                        f"({flips} flips in {window:.0f}s)")
        return None

    def start_recording(self, extract):
        with self._lock:
            self._rec_buf = []
            self._rec_t0 = time.monotonic()
            self._u_tv = 0.0
            self._u_n = 0
            self._extract = extract

    def stop_recording(self):
        """Returns (error samples, thruster roughness = mean |delta u|)."""
        with self._lock:
            self._extract = None
            chatter = self._u_tv / max(self._u_n, 1)
            return list(self._rec_buf), chatter

    def send_pos(self, x=0.0, y=0.0, z=0.0, velocity=False):
        m = Setpoint()
        m.velocity = velocity
        m.altitude = False
        m.setpoint.x, m.setpoint.y, m.setpoint.z = float(x), float(y), float(z)
        self._pos_pub.publish(m)

    def send_att(self, roll=0.0, pitch=0.0, yaw=0.0, velocity=False):
        m = Setpoint()
        m.velocity = velocity
        m.altitude = False
        m.setpoint.roll = float(roll)
        m.setpoint.pitch = float(pitch)
        m.setpoint.yaw = float(yaw)
        self._att_pub.publish(m)

    def zero_thrusters(self):
        m = Float64()
        m.data = 0.0
        for _ in range(3):
            for pub in self._thruster_pubs:
                pub.publish(m)
            time.sleep(0.05)


# ---------------------------------------------------------------------------
# sub_control process management (gains are declare-only -> restart to apply)
# ---------------------------------------------------------------------------

class ControllerManager:
    def __init__(self, node: TunerNode, robot_name: str, state_dir: Path):
        self.node = node
        self.robot_name = robot_name
        self.state_dir = state_dir
        self.proc = None
        self.loaded_key = None
        self.started_at = 0.0
        prefix = subprocess.check_output(
            ["ros2", "pkg", "prefix", "sub_control"], text=True
        ).strip()
        self.binary = Path(prefix) / "lib" / "sub_control" / "sub_control"
        if not self.binary.exists():
            raise FileNotFoundError(f"sub_control binary not found at {self.binary}")
        self.log_file = open(state_dir / "sub_control.log", "a")

    def _foreign_pids(self):
        """PIDs whose executable is the sub_control binary (not our child)."""
        target = os.path.realpath(self.binary)
        own = self.proc.pid if self.proc is not None else -1
        pids = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit() or int(entry.name) == own:
                continue
            try:
                exe = os.readlink(entry / "exe")
            except OSError:
                continue
            if exe == target or exe.endswith("/lib/sub_control/sub_control"):
                pids.append(int(entry.name))
        return pids

    def kill_foreign(self):
        """Take over from a sub_control started by the bringup launch file."""
        pids = self._foreign_pids()
        if not pids:
            return
        self.node.get_logger().info(
            "stopping the stack's sub_control; the tuner manages its own")
        for sig in (signal.SIGINT, signal.SIGKILL):
            for pid in pids:
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
            time.sleep(1.0)
            pids = self._foreign_pids()
            if not pids:
                break
        self.node.zero_thrusters()

    def ensure(self, params: dict):
        key = json.dumps(params, sort_keys=True)
        if self.alive() and key == self.loaded_key:
            return
        self.stop()
        path = self.state_dir / "candidate.yaml"
        path.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}},
                                       default_flow_style=None, sort_keys=False))
        argv = [
            str(self.binary), "--ros-args",
            "-r", f"__ns:=/{self.robot_name}",
            "-r", "__node:=sub_control",
            "--params-file", str(path),
            "-p", f"robot_name:={self.robot_name}",
        ]
        self.log_file.write(f"\n=== {datetime.now().isoformat()} {argv}\n")
        self.log_file.flush()
        self.proc = subprocess.Popen(
            argv, stdout=self.log_file, stderr=subprocess.STDOUT,
            start_new_session=True)
        self.loaded_key = key
        self.started_at = time.monotonic()

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def stop(self):
        if self.proc is not None:
            if self.proc.poll() is None:
                self.proc.send_signal(signal.SIGINT)
                try:
                    self.proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                    self.proc.wait()
            self.proc = None
            self.loaded_key = None
            # Nothing is driving the thrusters now; make sure they are off.
            self.node.zero_thrusters()


# ---------------------------------------------------------------------------
# Persistent tuner state
# ---------------------------------------------------------------------------

class TunerState:
    def __init__(self, path: Path, data: dict):
        self.path = path
        self.data = data

    @classmethod
    def load_or_new(cls, path: Path, plan, gains, extras, mode):
        if path.exists():
            data = json.loads(path.read_text())
            if data.get("version") != STATE_VERSION:
                sys.exit(
                    f"{path} was written by an older tuner (version "
                    f"{data.get('version')}); rerun with --fresh to discard it.")
            if data.get("plan") != plan:
                sys.exit(
                    f"{path} was created for loops {data.get('plan')} but "
                    f"{plan} was requested; rerun with --fresh or a different "
                    "--state-dir to start a new session.")
            return cls(path, data), True
        data = {
            "version": STATE_VERSION,
            "mode": mode,
            "created": datetime.now().isoformat(),
            "plan": plan,
            "loop_idx": 0,
            "twiddle": None,
            "gains": gains,
            "extras": extras,
            "trial_seq": 0,
        }
        st = cls(path, data)
        st.save()
        return st, False

    def save(self):
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.data, indent=2))
        tmp.replace(self.path)


def gains_to_param_dict(gains: dict, extras: dict) -> dict:
    params = {
        "control_rate_hz": float(extras.get("control_rate_hz", 50.0)),
        "power_limit": float(extras.get("power_limit", 0.6)),
    }
    for group in ("pos_pid", "vel_pid", "att_pid", "ang_pid"):
        params[group] = {
            ax: [float(v) for v in gains[f"{group}.{ax}"]] for ax in "xyz"
        }
    return params


def write_gains_yaml(path: Path, gains: dict, extras: dict):
    doc = {"/**": {"ros__parameters": gains_to_param_dict(gains, extras)}}
    path.write_text(yaml.safe_dump(doc, default_flow_style=None, sort_keys=False))


def load_base_gains(path: Path):
    doc = yaml.safe_load(path.read_text())
    params = next(iter(doc.values()))["ros__parameters"]
    gains = {}
    for group in ("pos_pid", "vel_pid", "att_pid", "ang_pid"):
        for ax in "xyz":
            gains[f"{group}.{ax}"] = [float(v) for v in params[group][ax]]
    extras = {
        "control_rate_hz": float(params.get("control_rate_hz", 50.0)),
        "power_limit": float(params.get("power_limit", 0.6)),
    }
    return gains, extras


# ---------------------------------------------------------------------------
# The tuner itself
# ---------------------------------------------------------------------------

@dataclass
class TrialConfig:
    home_depth: float
    z_min: float
    z_max: float
    pos_step_xy: float = 1.0
    pos_step_z: float = 0.5
    pos_duration: float = 12.0
    att_step_yaw: float = 0.7
    att_step_rp: float = 0.2
    att_duration: float = 10.0
    vel_step_xy: float = 0.25
    vel_step_z: float = 0.12
    vel_duration: float = 5.0
    ang_step_yaw: float = 0.4
    ang_step_rp: float = 0.25
    ang_duration: float = 4.0
    settle_pos_tol: float = 0.15
    settle_depth_tol: float = 0.10
    settle_att_tol: float = 0.09
    settle_speed_tol: float = 0.08
    settle_hold: float = 1.5
    settle_timeout: float = 60.0
    max_roll_pitch: float = 0.7
    max_speed: float = 1.0
    max_ang_speed: float = 2.0
    osc_window: float = 4.0
    osc_min_flips: int = 6


class Tuner:
    def __init__(self, node, ctrl, state, cfg, args, csv_path):
        self.node = node
        self.ctrl = ctrl
        self.state = state
        self.cfg = cfg
        self.args = args
        self.csv_path = csv_path
        self.log = node.get_logger()
        self._last_osc_check = 0.0

    # ---- low-level waiting primitives -------------------------------------

    def _check(self, env=None):
        """Liveness checks, plus the safety envelope when env is set.

        env is None (bookkeeping only), "settle" (envelope minus the depth
        bounds — the sub may legitimately start at the surface — and with
        speed slack for the transit home) or "step" (full envelope). The
        oscillation detector runs for both settle and step: an unstable
        candidate must be cut off no matter which phase it thrashes in.
        """
        snap = self.node.snapshot()
        if snap["killed"]:
            raise KillAbort()
        if self.ctrl.proc is not None and not self.ctrl.alive():
            raise StackDead("sub_control process exited")
        started_for = time.monotonic() - self.ctrl.started_at
        if (snap["err"] is not None and snap["err_age"] > 3.0 and started_for > 10.0):
            raise StackDead("control/error went stale")
        if env is not None:
            now = time.monotonic()
            if now - self._last_osc_check > 0.2:
                self._last_osc_check = now
                why = self.node.oscillation(self.cfg.osc_window,
                                            self.cfg.osc_min_flips)
                if why is not None:
                    raise WatchdogAbort(why)
            od = snap["odom"]
            if od is not None:
                if abs(od["roll"]) > self.cfg.max_roll_pitch or \
                   abs(od["pitch"]) > self.cfg.max_roll_pitch:
                    raise WatchdogAbort("roll/pitch limit")
                if od["ang_speed"] > self.cfg.max_ang_speed:
                    raise WatchdogAbort("angular rate limit")
                speed_cap = self.cfg.max_speed * (1.25 if env == "settle" else 1.0)
                if od["speed"] > speed_cap:
                    raise WatchdogAbort("speed limit")
                if env == "step" and \
                        not (self.cfg.z_min - 0.3 <= od["z"] <= self.cfg.z_max + 0.3):
                    raise WatchdogAbort("depth bounds")
        return snap

    def _sleep(self, duration, env=None, repub=None):
        t0 = time.monotonic()
        next_repub = 0.0
        while time.monotonic() - t0 < duration:
            self._check(env)
            if repub is not None and time.monotonic() >= next_repub:
                repub()
                next_repub = time.monotonic() + 0.5
            time.sleep(0.05)

    def wait_unkilled(self):
        last_note = 0.0
        while True:
            snap = self.node.snapshot()
            if not snap["killed"]:
                return
            if time.monotonic() - last_note > 10.0:
                self.log.info("killed — waiting for the kill switch to be released")
                last_note = time.monotonic()
            time.sleep(0.1)

    def wait_stack_ready(self, timeout=25.0):
        t0 = time.monotonic()
        while True:
            snap = self._check()
            if snap["err"] is not None and snap["err_age"] < 0.5 \
                    and snap["odom"] is not None and snap["odom_age"] < 1.0:
                return
            if time.monotonic() - t0 > timeout:
                raise StackDead("control/error or odometry/filtered not flowing")
            time.sleep(0.1)

    # ---- homing ------------------------------------------------------------

    def capture_home(self) -> dict:
        od = self.node.snapshot()["odom"]
        return {"x": od["x"], "y": od["y"], "z": self.cfg.home_depth, "yaw": od["yaw"]}

    def hold_home(self, home):
        self.node.send_pos(home["x"], home["y"], home["z"])
        self.node.send_att(0.0, 0.0, home["yaw"])

    def settle(self, home):
        t0 = time.monotonic()
        stable_since = None
        while True:
            self._check(env="settle")
            self.hold_home(home)
            snap = self.node.snapshot()
            ok = False
            if snap["err"] is not None and snap["err_age"] < 0.5 and snap["odom"]:
                e = snap["err"]
                ok = (
                    abs(e.pos_error[0]) < self.cfg.settle_pos_tol
                    and abs(e.pos_error[1]) < self.cfg.settle_pos_tol
                    and abs(e.pos_error[2]) < self.cfg.settle_depth_tol
                    and all(abs(a) < self.cfg.settle_att_tol for a in e.att_error)
                    and snap["odom"]["speed"] < self.cfg.settle_speed_tol
                )
            if ok:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since > self.cfg.settle_hold:
                    return
            else:
                stable_since = None
            if time.monotonic() - t0 > self.cfg.settle_timeout:
                raise SettleTimeout()
            time.sleep(0.1)

    # ---- trials ------------------------------------------------------------

    def run_trial(self, loop_key: str):
        group, axis_name = loop_key.split(".")
        axis = AXIS_INDEX[axis_name]
        cfg = self.cfg
        home = self.capture_home()
        self.hold_home(home)
        self.settle(home)
        home = {**self.capture_home(), "z": cfg.home_depth}

        try:
            if group == "pos":
                return self._trial_pos(home, axis)
            if group == "att":
                return self._trial_att(home, axis)
            if group == "vel":
                return self._trial_vel(home, axis)
            if group == "ang":
                return self._trial_ang(home, axis)
            raise ValueError(loop_key)
        finally:
            self.node.stop_recording()

    def _score_or_fail(self, samples, step, duration):
        cost = score_step_response(samples, step, duration)
        if cost is None:
            raise StackDead("too few control/error samples during trial")
        return cost

    def _trial_pos(self, home, axis):
        cfg = self.cfg
        if axis == 2:
            target_z = min(max(home["z"] - cfg.pos_step_z, cfg.z_min + 0.2), cfg.z_max)
            step = target_z - home["z"]
            target = {**home, "z": target_z}
            self.node.start_recording(lambda m: m.pos_error[2])
        else:
            yaw = home["yaw"]
            # Step along the body axis under test: surge for x, sway for y.
            dx, dy = ((math.cos(yaw), math.sin(yaw)) if axis == 0
                      else (-math.sin(yaw), math.cos(yaw)))
            step = cfg.pos_step_xy
            target = {**home, "x": home["x"] + step * dx, "y": home["y"] + step * dy}
            self.node.start_recording(
                lambda m, dx=dx, dy=dy: m.pos_error[0] * dx + m.pos_error[1] * dy)

        self._sleep(cfg.pos_duration, env="step",
                    repub=lambda: self.hold_home(target))
        samples, chatter = self.node.stop_recording()
        self.hold_home(home)  # come back; next trial settles here anyway
        return self._score_or_fail(samples, step, cfg.pos_duration) \
            + CHATTER_WEIGHT * chatter

    def _trial_att(self, home, axis):
        cfg = self.cfg
        rpy = [0.0, 0.0, home["yaw"]]
        step = cfg.att_step_yaw if axis == 2 else cfg.att_step_rp
        rpy[axis] = norm_angle(rpy[axis] + step)
        self.node.start_recording(lambda m, axis=axis: m.att_error[axis])
        repub = lambda: (self.node.send_pos(home["x"], home["y"], home["z"]),
                         self.node.send_att(*rpy))
        self._sleep(cfg.att_duration, env="step", repub=repub)
        samples, chatter = self.node.stop_recording()
        self.hold_home(home)
        return self._score_or_fail(samples, step, cfg.att_duration) \
            + CHATTER_WEIGHT * chatter

    def _trial_vel(self, home, axis):
        cfg = self.cfg
        v = cfg.vel_step_z if axis == 2 else cfg.vel_step_xy
        # Out-and-back keeps net drift near zero; dive first on the z axis.
        signs = (-1.0, 1.0) if axis == 2 else (1.0, -1.0)
        duration = cfg.vel_duration
        self.node.start_recording(lambda m, axis=axis: m.vel_error[axis])
        try:
            for sgn in signs:
                sp = [0.0, 0.0, 0.0]
                sp[axis] = sgn * v

                def repub(sp=sp):
                    self.node.send_pos(*sp, velocity=True)
                    self.node.send_att(0.0, 0.0, home["yaw"])

                repub()
                self._sleep(duration, env="step", repub=repub)
        finally:
            samples, chatter = self.node.stop_recording()
            # Leave velocity mode: hold wherever we ended up, at home depth.
            od = self.node.snapshot()["odom"]
            if od is not None:
                self.node.send_pos(od["x"], od["y"], home["z"])
                self.node.send_att(0.0, 0.0, home["yaw"])
        seg1 = [(t, e) for t, e in samples if t < duration]
        seg2 = [(t - duration, e) for t, e in samples if t >= duration]
        c1 = self._score_or_fail(seg1, signs[0] * v, duration)
        # The second command reverses the velocity, so the error step is ~2v.
        c2 = self._score_or_fail(seg2, (signs[1] - signs[0]) * v, duration)
        return 0.5 * (c1 + c2) + CHATTER_WEIGHT * chatter

    def _trial_ang(self, home, axis):
        cfg = self.cfg
        r = cfg.ang_step_yaw if axis == 2 else cfg.ang_step_rp
        duration = cfg.ang_duration
        self.node.start_recording(lambda m, axis=axis: m.angvel_error[axis])
        try:
            for sgn in (1.0, -1.0):
                rate = [0.0, 0.0, 0.0]
                rate[axis] = sgn * r

                def repub(rate=rate):
                    self.node.send_pos(home["x"], home["y"], home["z"])
                    self.node.send_att(*rate, velocity=True)

                repub()
                self._sleep(duration, env="step", repub=repub)
        finally:
            samples, chatter = self.node.stop_recording()
            od = self.node.snapshot()["odom"]
            yaw = od["yaw"] if od is not None else home["yaw"]
            self.node.send_att(0.0, 0.0, yaw)  # back to attitude hold
            self.node.send_pos(home["x"], home["y"], home["z"])
        seg1 = [(t, e) for t, e in samples if t < duration]
        seg2 = [(t - duration, e) for t, e in samples if t >= duration]
        c1 = self._score_or_fail(seg1, r, duration)
        c2 = self._score_or_fail(seg2, -2.0 * r, duration)
        return 0.5 * (c1 + c2) + CHATTER_WEIGHT * chatter

    # ---- candidate evaluation with kill/restart recovery --------------------

    def evaluate(self, loop_key: str, cand: list):
        """Returns (cost, status). Never raises for a bad candidate — bad
        candidates get FAIL_COST so the optimizer backs away from them."""
        gains = copy.deepcopy(self.state.data["gains"])
        param_key = f"{GROUP_TO_PARAM[loop_key.split('.')[0]]}.{loop_key.split('.')[1]}"
        gains[param_key][:3] = [float(g) for g in cand]
        params = gains_to_param_dict(gains, self.state.data["extras"])

        kills = 0
        restarts = 0
        while True:
            try:
                self.wait_unkilled()
                self.ctrl.ensure(params)
                self.wait_stack_ready()
                return self.run_trial(loop_key), "ok"
            except KillAbort:
                kills += 1
                if kills >= self.args.max_kill_rejects:
                    self.log.warn(
                        f"candidate killed {kills}x — treating as operator "
                        "veto and scoring it as a failure")
                    return FAIL_COST, "kill-veto"
                self.log.info("trial aborted by kill; will re-run after unkill")
                self.wait_unkilled()
                # sub_control zeroes the EKF pose via set_pose on unkill; give
                # the filter a moment before we trust odometry again.
                time.sleep(2.5)
            except WatchdogAbort as e:
                # Do not leave an unstable candidate in charge of the
                # thrusters: stop sub_control (zeroes them) and let the sub
                # coast before the next candidate takes over.
                self.log.warn(f"safety watchdog tripped ({e}) — stopping "
                              "sub_control and scoring as failure")
                self.ctrl.stop()
                time.sleep(3.0)
                return FAIL_COST, "watchdog"
            except SettleTimeout:
                self.log.warn("could not settle at trial start — scoring as failure")
                return FAIL_COST, "no-settle"
            except StackDead as e:
                restarts += 1
                self.log.warn(f"stack problem ({e}); restarting sub_control "
                              f"(attempt {restarts})")
                self.ctrl.stop()
                if restarts >= 5:
                    raise RuntimeError(
                        "sub_control keeps dying — check "
                        f"{self.ctrl.state_dir / 'sub_control.log'}") from e

    # ---- top level -----------------------------------------------------------

    def log_trial(self, seq, loop_key, st, cand, cost, status):
        new = not self.csv_path.exists()
        with open(self.csv_path, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["seq", "time", "loop", "phase", "gain", "kp", "ki",
                            "kd", "cost", "best_cost", "status"])
            w.writerow([seq, datetime.now().isoformat(timespec="seconds"),
                        loop_key, st["phase"], GAIN_NAMES[twiddle_idx(st)],
                        f"{cand[0]:.4f}", f"{cand[1]:.4f}", f"{cand[2]:.4f}",
                        f"{cost:.4f}", f"{st['best_cost']:.4f}" if st["best_cost"]
                        is not None else "", status])

    def _rebaseline(self, st: dict, param_key: str) -> dict:
        """The baseline gains themselves failed twice: shrink toward a more
        conservative point and try again, rather than letting FAIL_COST
        become the score every garbage candidate 'improves' on."""
        old = st["params"]
        shrunk = [0.7 * old[0], 0.5 * old[1], 0.7 * old[2]]
        self.log.warn(
            f"baseline gains unstable — shrinking kp {old[0]:.3f}->{shrunk[0]:.3f} "
            f"ki {old[1]:.3f}->{shrunk[1]:.3f} kd {old[2]:.3f}->{shrunk[2]:.3f} "
            "and re-baselining")
        fresh = new_twiddle(shrunk, st["budget"], trials_used=st["trials"])
        self.state.data["gains"][param_key][:3] = shrunk
        return fresh

    def run(self):
        data = self.state.data
        self.ctrl.kill_foreign()

        # Learn the kill state before moving anything.
        t0 = time.monotonic()
        while self.node.snapshot()["killed"] is None and time.monotonic() - t0 < 10.0:
            time.sleep(0.2)
        if self.node.snapshot()["killed"] is None:
            self.log.warn("no kill_switch publisher found — assuming not killed "
                          "(fine on a bench, do NOT ignore this in the water)")

        plan = data["plan"]
        while data["loop_idx"] < len(plan):
            loop_key = plan[data["loop_idx"]]
            group, ax = loop_key.split(".")
            param_key = f"{GROUP_TO_PARAM[group]}.{ax}"
            if data["twiddle"] is None:
                data["twiddle"] = new_twiddle(data["gains"][param_key][:3],
                                              self.args.trials_per_loop)
                self.state.save()
            st = data["twiddle"]
            self.log.info(
                f"=== tuning {loop_key} ({data['loop_idx'] + 1}/{len(plan)}), "
                f"{st['trials']}/{st['budget']} trials used ===")

            while not st["done"]:
                cand = twiddle_candidate(st)
                if st["phase"] != "baseline" and all(
                        abs(c - p) < 1e-9 for c, p in zip(cand, st["params"])):
                    # Clamped into the current best: no new information, and
                    # no water time spent, so it does not consume the budget.
                    twiddle_record(st, FAIL_COST, count_trial=False)
                    self.state.save()
                    continue
                seq = data["trial_seq"] = data["trial_seq"] + 1
                self.log.info(
                    f"[{loop_key}] trial {st['trials'] + 1}/{st['budget']} "
                    f"({st['phase']} gain {GAIN_NAMES[twiddle_idx(st)]}): "
                    f"kp={cand[0]:.3f} ki={cand[1]:.3f} kd={cand[2]:.3f}")
                cost, status = self.evaluate(loop_key, cand)
                self.log_trial(seq, loop_key, st, cand, cost, status)

                if st["phase"] == "baseline" and cost >= FAIL_COST:
                    # A failed baseline must never become best_cost: retry,
                    # and after two failures shrink the gains themselves.
                    st["trials"] += 1
                    st["baseline_fails"] += 1
                    if st["trials"] >= st["budget"]:
                        self.log.error(
                            f"[{loop_key}] budget exhausted without a stable "
                            "baseline — keeping the original gains")
                        st["done"] = True
                    elif st["baseline_fails"] >= 2:
                        st = data["twiddle"] = self._rebaseline(st, param_key)
                    self.state.save()
                    continue

                twiddle_carve_ceiling(st, status)
                improved = twiddle_record(st, cost)
                data["gains"][param_key][:3] = st["params"]
                self.state.save()
                write_gains_yaml(self.ctrl.state_dir / "tuned_gains.yaml",
                                 data["gains"], data["extras"])
                marker = "IMPROVED" if improved else status
                self.log.info(
                    f"[{loop_key}] cost={cost:.4f} best={st['best_cost']:.4f} "
                    f"[{marker}]")

            self.log.info(
                f"=== {loop_key} done: kp={st['params'][0]:.3f} "
                f"ki={st['params'][1]:.3f} kd={st['params'][2]:.3f} "
                f"(cost {st['best_cost'] if st['best_cost'] is not None else 'n/a'}) ===")
            data["loop_idx"] += 1
            data["twiddle"] = None
            self.state.save()

        write_gains_yaml(self.ctrl.state_dir / "tuned_gains.yaml",
                         data["gains"], data["extras"])
        self.log.info("tuning complete")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args(argv):
    p = argparse.ArgumentParser(
        prog="pid_tuner", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mode", choices=("sim", "real"), required=True,
                   help="selects the base gains file and the state directory")
    p.add_argument("--robot-name", default="marlin_v2")
    p.add_argument("--loops", default=",".join(DEFAULT_PLAN),
                   help="comma-separated loops to tune in order, or 'all'; "
                        f"default: {','.join(DEFAULT_PLAN)}")
    p.add_argument("--base-gains", default=None,
                   help="gains yaml to start from (default: the mode's "
                        "control_gains yaml from the sub_bringup share dir)")
    p.add_argument("--state-dir", default=None,
                   help="checkpoint directory (default: ~/.thalassic_pid_tuner/<mode>)")
    p.add_argument("--trials-per-loop", type=int, default=24)
    p.add_argument("--max-kill-rejects", type=int, default=2,
                   help="kills of the same candidate before it is scored as "
                        "a failure instead of retried")
    p.add_argument("--step-scale", type=float, default=1.0,
                   help="scale factor on all trial step sizes; use <1 for "
                        "gentler trials on the real sub (e.g. 0.6)")
    p.add_argument("--home-depth", type=float, default=-1.0,
                   help="trial depth in FLU z (negative = underwater)")
    p.add_argument("--z-min", type=float, default=-3.0,
                   help="deepest allowed z during trials")
    p.add_argument("--z-max", type=float, default=-0.3,
                   help="shallowest allowed z during trials")
    p.add_argument("--settle-timeout", type=float, default=60.0)
    p.add_argument("--status", action="store_true",
                   help="print checkpoint progress and exit")
    p.add_argument("--fresh", action="store_true",
                   help="discard the checkpoint and start over")
    return p.parse_args(argv)


def resolve_plan(loops_arg: str):
    plan = ALL_LOOPS if loops_arg.strip() == "all" else [
        s.strip() for s in loops_arg.split(",") if s.strip()]
    for loop in plan:
        if loop not in ALL_LOOPS:
            sys.exit(f"unknown loop '{loop}'; valid: {', '.join(ALL_LOOPS)}")
    return plan


def print_status(state_path: Path):
    if not state_path.exists():
        print(f"no checkpoint at {state_path}")
        return
    d = json.loads(state_path.read_text())
    print(f"checkpoint: {state_path} (mode={d['mode']}, created {d['created']})")
    for i, loop in enumerate(d["plan"]):
        group, ax = loop.split(".")
        g = d["gains"][f"{GROUP_TO_PARAM[group]}.{ax}"]
        if i < d["loop_idx"]:
            mark = "done"
        elif i == d["loop_idx"]:
            tw = d.get("twiddle")
            mark = (f"in progress ({tw['trials']}/{tw['budget']} trials, "
                    f"best {tw['best_cost']})" if tw else "next")
        else:
            mark = "pending"
        print(f"  {loop:7s} kp={g[0]:8.3f} ki={g[1]:8.3f} kd={g[2]:8.3f} "
              f"lim={g[3]:6.2f}  {mark}")


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])

    state_dir = Path(args.state_dir or
                     Path.home() / ".thalassic_pid_tuner" / args.mode)
    state_dir.mkdir(parents=True, exist_ok=True)
    state_path = state_dir / "state.json"

    if args.status:
        print_status(state_path)
        return

    if args.fresh and state_path.exists():
        state_path.unlink()
        print(f"discarded checkpoint {state_path}")

    if args.base_gains:
        base_path = Path(args.base_gains)
    else:
        from ament_index_python.packages import get_package_share_directory
        name = "control_gains_sim.yaml" if args.mode == "sim" else "control_gains.yaml"
        base_path = Path(get_package_share_directory("sub_bringup")) / "config" / name

    plan = resolve_plan(args.loops)
    gains, extras = load_base_gains(base_path)
    state, resumed = TunerState.load_or_new(state_path, plan, gains, extras, args.mode)

    # Handle Ctrl+C/SIGTERM ourselves (KeyboardInterrupt in the main loop) so
    # the checkpoint is saved and sub_control is stopped cleanly. Installed
    # explicitly because a backgrounded process inherits SIGINT as ignored.
    def _sig_handler(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = TunerNode(args.robot_name)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    def _spin():
        try:
            executor.spin()
        except Exception as e:  # surfaced via error staleness watchdogs
            print(f"spin thread died: {e}", file=sys.stderr)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    log = node.get_logger()
    log.info(f"{'resuming from' if resumed else 'new session at'} {state_path}")
    log.info(f"base gains: {base_path}")
    log.info(f"plan: {' -> '.join(plan)}")

    if not (0.1 <= args.step_scale <= 1.0):
        sys.exit("--step-scale must be in [0.1, 1.0]")
    cfg = TrialConfig(home_depth=args.home_depth, z_min=args.z_min,
                      z_max=args.z_max, settle_timeout=args.settle_timeout)
    for field in ("pos_step_xy", "pos_step_z", "att_step_yaw", "att_step_rp",
                  "vel_step_xy", "vel_step_z", "ang_step_yaw", "ang_step_rp"):
        setattr(cfg, field, getattr(cfg, field) * args.step_scale)
    if not (cfg.z_min < cfg.home_depth < cfg.z_max):
        sys.exit("--home-depth must lie between --z-min and --z-max")

    ctrl = ControllerManager(node, args.robot_name, state_dir)
    tuner = Tuner(node, ctrl, state, cfg, args, state_dir / "trials.csv")

    try:
        tuner.run()
        log.info(f"tuned gains written to {state_dir / 'tuned_gains.yaml'} — "
                 "copy into src/sub_bringup/config/ and rebuild to adopt them")
    except KeyboardInterrupt:
        log.info(f"interrupted — progress saved; rerun the same command to "
                 f"resume ({state_path})")
    finally:
        # A late Ctrl+C must not interrupt the cleanup itself (it stops
        # sub_control and zeroes the thrusters).
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        state.save()
        ctrl.stop()  # also zeroes thrusters; relaunch the stack to drive again
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=3.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
