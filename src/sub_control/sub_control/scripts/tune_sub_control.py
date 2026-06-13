#!/usr/bin/env python3

"""Conservative coordinate-search tuner for the sub_control state-feedback node."""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from std_msgs.msg import Float64
from sub_control_interfaces.msg import Error, Setpoint


AXES = {"x": 0, "y": 1, "z": 2}
TUNING_PARAMETERS = {
    "position": [
        "controller.velocity_kp",
        "controller.velocity_ki",
        "controller.position_gain",
        "controller.position_integral_gain",
    ],
    "attitude": [
        "controller.angular_rate_kp",
        "controller.angular_rate_ki",
        "controller.attitude_gain",
    ],
}
INNER_PARAMETERS = {
    "controller.velocity_kp",
    "controller.velocity_ki",
    "controller.angular_rate_kp",
    "controller.angular_rate_ki",
}


@dataclass
class TrialResult:
    score: float
    mean_absolute_error: float
    final_error: float
    overshoot: float
    mean_effort: float
    sample_count: int


class ControlTuner(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        namespace = args.namespace.strip("/")
        super().__init__("sub_control_tuner", namespace=namespace)
        self.args = args
        self.axis = AXES[args.axis]
        self.error_samples: list[float] = []
        self.effort_samples: list[float] = []
        self.aborted = False
        self.direct_mode = False

        prefix = f"/{namespace}" if namespace else ""
        self.target_node = f"{prefix}/sub_control"
        self.parameter_client = AsyncParameterClient(self, self.target_node)
        self.position_publisher = self.create_publisher(Setpoint, "pos_setpoint", 10)
        self.attitude_publisher = self.create_publisher(Setpoint, "att_setpoint", 10)
        self.create_subscription(Error, "control/error", self._error_callback, 20)
        for thruster in range(8):
            self.create_subscription(
                Float64,
                f"control/thruster_{thruster}",
                self._effort_callback,
                10,
            )

    def _error_callback(self, message: Error) -> None:
        # Error fields are REP-103 body FLU, matching the cmd_* topics.
        if self.args.loop == "position":
            errors = message.vel_error if self.direct_mode else message.pos_error
        else:
            errors = message.angvel_error if self.direct_mode else message.att_error
        value = float(errors[self.axis])
        if math.isfinite(value):
            self.error_samples.append(value)
            if abs(value) > self.args.max_error:
                self.aborted = True

    def _effort_callback(self, message: Float64) -> None:
        if math.isfinite(message.data):
            self.effort_samples.append(abs(float(message.data)))

    def wait_for_controller(self) -> None:
        if not self.parameter_client.wait_for_services(timeout_sec=10.0):
            raise RuntimeError(f"parameter services unavailable for {self.target_node}")

    def get_parameter_array(self, name: str) -> list[float]:
        future = self.parameter_client.get_parameters([name])
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None or not future.result().values:
            raise RuntimeError(f"could not read {name}")
        return list(future.result().values[0].double_array_value)

    def set_parameter_array(self, name: str, values: list[float]) -> None:
        parameter = Parameter(name, Parameter.Type.DOUBLE_ARRAY, values)
        future = self.parameter_client.set_parameters([parameter])
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.results or not response.results[0].successful:
            reason = "no response" if response is None else response.results[0].reason
            raise RuntimeError(f"could not set {name}: {reason}")

    def publish_command(self, amplitude: float, direct: bool) -> None:
        # Commands follow REP-103 (body FLU, z up): a positive z amplitude
        # moves the sub up; pass a negative amplitude to step down. direct
        # selects the inner (velocity / angular-rate) loop via Setpoint.velocity.
        message = Setpoint()
        message.velocity = direct
        if self.args.loop == "position":
            setattr(message.setpoint, "xyz"[self.axis], amplitude)
            publisher = self.position_publisher
        else:
            setattr(message.setpoint, ("roll", "pitch", "yaw")[self.axis], amplitude)
            publisher = self.attitude_publisher
        publisher.publish(message)

    def spin_for(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.aborted:
                raise RuntimeError(
                    f"trial exceeded safety error limit ({self.args.max_error:g})"
                )

    def run_trial(self, parameter_name: str) -> TrialResult:
        self.error_samples.clear()
        self.effort_samples.clear()
        self.aborted = False
        self.direct_mode = parameter_name in INNER_PARAMETERS
        amplitude = self.args.inner_amplitude if self.direct_mode else self.args.amplitude

        self.publish_command(amplitude, self.direct_mode)
        self.spin_for(self.args.duration)
        if len(self.error_samples) < self.args.minimum_samples:
            raise RuntimeError(
                f"received only {len(self.error_samples)} error samples; "
                "check controller, odometry, and kill switch"
            )

        errors = self.error_samples
        tail = errors[max(0, int(len(errors) * 0.75)) :]
        mean_absolute_error = statistics.fmean(abs(value) for value in errors)
        final_error = statistics.fmean(abs(value) for value in tail)
        overshoot = max(
            (abs(value) for value in errors if value * amplitude < 0.0),
            default=0.0,
        )
        mean_effort = (
            statistics.fmean(self.effort_samples) if self.effort_samples else 0.0
        )
        score = (
            mean_absolute_error
            + 3.0 * final_error
            + 2.0 * overshoot
            + self.args.effort_weight * mean_effort
        )

        self.publish_command(0.0, self.direct_mode)
        self.spin_for(self.args.settle)
        return TrialResult(
            score=score,
            mean_absolute_error=mean_absolute_error,
            final_error=final_error,
            overshoot=overshoot,
            mean_effort=mean_effort,
            sample_count=len(errors),
        )


def parse_scales(value: str) -> list[float]:
    scales = [float(item) for item in value.split(",")]
    if not scales or any(scale <= 0.0 or not math.isfinite(scale) for scale in scales):
        raise argparse.ArgumentTypeError("scales must be positive finite numbers")
    return scales


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Tune one sub_control axis using conservative step-response search."
    )
    parser.add_argument("--namespace", default="/marlin_v2")
    parser.add_argument("--loop", choices=("position", "attitude"), default="position")
    parser.add_argument("--axis", choices=tuple(AXES), default="x")
    parser.add_argument(
        "--amplitude",
        type=float,
        default=0.5,
        help="Outer-loop step in REP-103 body FLU (z is up: use a negative "
        "z-axis amplitude to step deeper).",
    )
    parser.add_argument(
        "--inner-amplitude",
        type=float,
        default=0.2,
        help="Direct velocity/rate step used while tuning inner-loop gains.",
    )
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--settle", type=float, default=4.0)
    parser.add_argument("--scales", type=parse_scales, default=parse_scales("0.75,1.0,1.25"))
    parser.add_argument("--max-error", type=float, default=2.0)
    parser.add_argument("--minimum-samples", type=int, default=50)
    parser.add_argument("--effort-weight", type=float, default=0.05)
    parser.add_argument(
        "--parameters",
        nargs="+",
        help="Parameter arrays to tune; defaults to the selected loop's gain arrays.",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Keep the best gains. Without this flag every parameter is restored.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if (
        args.amplitude == 0.0
        or args.inner_amplitude == 0.0
        or args.duration <= 0.0
        or args.settle < 0.0
    ):
        print("amplitudes must be nonzero; duration must be positive", file=sys.stderr)
        return 2

    rclpy.init()
    tuner = ControlTuner(args)
    parameters = args.parameters or TUNING_PARAMETERS[args.loop]
    baseline: dict[str, list[float]] = {}
    selected: dict[str, list[float]] = {}

    try:
        tuner.wait_for_controller()
        for name in parameters:
            values = tuner.get_parameter_array(name)
            if len(values) != 3:
                raise RuntimeError(f"{name} is not a three-axis parameter")
            baseline[name] = values
            selected[name] = values.copy()

        print(
            f"Tuning {args.loop} {args.axis}-axis on {tuner.target_node}; "
            "ensure the test area is clear and the vehicle is armed."
        )
        tuner.publish_command(0.0, False)
        tuner.spin_for(args.settle)

        for name in parameters:
            current = selected[name]
            best_values = current.copy()
            best_result: TrialResult | None = None
            print(f"\n{name}: baseline {current[tuner.axis]:.6g}")

            for scale in args.scales:
                candidate = current.copy()
                candidate[tuner.axis] = current[tuner.axis] * scale
                tuner.set_parameter_array(name, candidate)
                result = tuner.run_trial(name)
                print(
                    f"  {'inner' if tuner.direct_mode else 'outer'} "
                    f"scale={scale:.3g} value={candidate[tuner.axis]:.6g} "
                    f"score={result.score:.5f} final={result.final_error:.5f} "
                    f"overshoot={result.overshoot:.5f} effort={result.mean_effort:.4f}"
                )
                if best_result is None or result.score < best_result.score:
                    best_result = result
                    best_values = candidate

            selected[name] = best_values
            tuner.set_parameter_array(name, best_values)
            print(f"  selected {best_values[tuner.axis]:.6g}")

        print("\nRecommended parameter arrays:")
        for name, values in selected.items():
            print(f"  {name}: {values}")

        if not args.apply:
            print("Dry run complete; restoring original parameters.")
            for name, values in baseline.items():
                tuner.set_parameter_array(name, values)
        else:
            print("Best parameters remain active. Update the YAML profile to persist them.")
        return 0
    except (KeyboardInterrupt, RuntimeError) as error:
        print(f"Tuning aborted: {error}", file=sys.stderr)
        for name, values in baseline.items():
            try:
                tuner.set_parameter_array(name, values)
            except RuntimeError:
                pass
        return 1
    finally:
        try:
            tuner.publish_command(0.0, tuner.direct_mode)
            tuner.spin_for(0.25)
        except RuntimeError:
            pass
        tuner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
