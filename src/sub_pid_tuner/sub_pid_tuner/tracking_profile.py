from __future__ import annotations

from dataclasses import asdict, dataclass
import math


MODE_LIMITS = {
    "position": {"unit": "m", "max_amplitude": 2.0},
    "velocity": {"unit": "m/s", "max_amplitude": 1.0},
    "attitude": {"unit": "rad", "max_amplitude": math.radians(30.0)},
    "angular_velocity": {"unit": "rad/s", "max_amplitude": 1.0},
}

EXPERIMENTS = {
    "minimum_jerk",
    "trapezoid",
    "step",
    "sine",
    "hold",
    "follower_pid",
    "square_test",
    "spline_test",
}
PATH_EXPERIMENTS = {"follower_pid", "square_test", "spline_test"}
PATH_AXES = {"xy", "xz", "yz"}


@dataclass(frozen=True)
class TrackingProfile:
    """A repeatable, bounded reference for manual controller tuning.

    The experiment shapes are deliberately separate. A minimum-jerk move tests
    an outer pose loop without the acceleration impulses of a linear ramp. A
    step always returns to zero and settles before testing the opposite
    direction. A windowed sine starts and ends at zero amplitude and exposes
    phase lag without abrupt reversals. Hold captures the pose for a manual
    disturbance test.
    """

    experiment: str
    mode: str
    axis: str
    amplitude: float
    ramp_time: float
    hold_time: float
    cycles: int

    @classmethod
    def from_values(
        cls,
        experiment: str,
        mode: str,
        axis: str,
        amplitude: float,
        ramp_time: float,
        hold_time: float,
        cycles: int,
    ) -> "TrackingProfile":
        cycles_value = float(cycles)
        if not math.isfinite(cycles_value) or not cycles_value.is_integer():
            raise ValueError("cycles must be a whole number")
        profile = cls(
            experiment=str(experiment),
            mode=str(mode),
            axis=str(axis),
            amplitude=float(amplitude),
            ramp_time=float(ramp_time),
            hold_time=float(hold_time),
            cycles=int(cycles_value),
        )
        profile.validate()
        return profile

    def validate(self) -> None:
        if self.experiment not in EXPERIMENTS:
            raise ValueError(f"unknown tuning experiment: {self.experiment}")
        if self.mode not in MODE_LIMITS:
            raise ValueError(f"unknown tuning mode: {self.mode}")
        if self.path_experiment:
            if self.mode != "position":
                raise ValueError("path op modes use the position controller")
            if self.axis not in PATH_AXES:
                raise ValueError("path plane must be xy, xz, or yz")
        elif self.axis not in "xyz" or len(self.axis) != 1:
            raise ValueError("axis must be x, y, or z")
        values = (self.amplitude, self.ramp_time, self.hold_time)
        if any(not math.isfinite(value) for value in values):
            raise ValueError("tracking routine values must be finite")
        limit = MODE_LIMITS[self.mode]["max_amplitude"]
        if self.experiment == "minimum_jerk" and not self.outer_loop:
            raise ValueError("minimum-jerk tracking is for position and attitude loops")
        if self.experiment == "hold" and not self.outer_loop:
            raise ValueError("disturbance hold is for position and attitude loops")
        if self.experiment == "trapezoid" and self.outer_loop:
            raise ValueError("trapezoidal cruise tuning is for velocity and angular-rate loops")
        if self.experiment != "hold" and (
            self.amplitude == 0.0 or abs(self.amplitude) > limit
        ):
            raise ValueError(
                f"{self.mode.replace('_', ' ')} amplitude must be non-zero and at most "
                f"{limit:g} {MODE_LIMITS[self.mode]['unit']}"
            )
        if not 0.5 <= self.ramp_time <= 30.0:
            raise ValueError("ramp time must be between 0.5 and 30 seconds")
        if not 0.2 <= self.hold_time <= 30.0:
            raise ValueError("hold time must be between 0.2 and 30 seconds")
        if not 1 <= self.cycles <= 20:
            raise ValueError("cycles must be between 1 and 20")

    @property
    def outer_loop(self) -> bool:
        return self.mode in ("position", "attitude")

    @property
    def path_experiment(self) -> bool:
        return self.experiment in PATH_EXPERIMENTS

    @property
    def segment_time(self) -> float:
        return self.ramp_time + self.hold_time

    @property
    def cycle_duration(self) -> float:
        if self.experiment == "minimum_jerk":
            return 2.0 * self.segment_time
        if self.experiment == "step":
            return 2.0 * self.segment_time
        if self.experiment == "trapezoid":
            # Ramp, cruise, ramp down, and settle in each direction. The
            # vehicle is never commanded directly from +A to -A.
            return 4.0 * self.segment_time
        if self.path_experiment:
            return self.segment_time
        if self.experiment == "sine":
            return self.ramp_time
        return self.hold_time

    @property
    def duration(self) -> float:
        duration = self.cycle_duration * self.cycles
        return duration + self.hold_time if self.experiment == "sine" else duration

    def value_at(self, elapsed: float) -> float:
        """Return the reference offset/rate at elapsed monotonic seconds."""
        elapsed = max(0.0, min(float(elapsed), self.duration))
        if elapsed >= self.duration:
            return 0.0
        if self.experiment == "hold":
            return 0.0
        if self.path_experiment:
            return 0.0

        if self.experiment == "sine":
            wave_duration = self.ramp_time * self.cycles
            if elapsed >= wave_duration:
                return 0.0
            fade = min(self.ramp_time * 0.5, wave_duration * 0.5)
            edge = min(elapsed, wave_duration - elapsed)
            envelope = 1.0 if edge >= fade else 0.5 - 0.5 * math.cos(math.pi * edge / fade)
            return self.amplitude * envelope * math.sin(2.0 * math.pi * elapsed / self.ramp_time)

        if self.experiment == "step":
            # +command -> zero/settle -> -command -> zero/settle. Momentum
            # from one direction is never carried directly into the other.
            local = elapsed % self.cycle_duration
            if local < self.ramp_time:
                return self.amplitude
            if local < self.segment_time:
                return 0.0
            if local < self.segment_time + self.ramp_time:
                return -self.amplitude
            return 0.0

        if self.experiment == "trapezoid":
            local = elapsed % self.cycle_duration
            segment = int(local / self.segment_time)
            within = local % self.segment_time
            direction = 1.0 if segment < 2 else -1.0
            phase = segment % 2
            if within >= self.ramp_time:
                return direction * self.amplitude if phase == 0 else 0.0
            fraction = within / self.ramp_time
            if phase == 0:
                return direction * self.amplitude * fraction
            return direction * self.amplitude * (1.0 - fraction)

        # Quintic smoothstep: position, velocity, and acceleration are all
        # continuous and zero at the endpoints.
        segment = int((elapsed % self.cycle_duration) / self.segment_time)
        local = elapsed % self.segment_time
        u = min(local / self.ramp_time, 1.0)
        smooth = u * u * u * (10.0 + u * (-15.0 + 6.0 * u))
        fraction = smooth if segment == 0 else 1.0 - smooth
        return self.amplitude * fraction

    @staticmethod
    def _smoothstep(value: float) -> float:
        value = max(0.0, min(1.0, value))
        return value * value * value * (10.0 + value * (-15.0 + 6.0 * value))

    def vector_at(self, elapsed: float) -> tuple[float, float, float]:
        """Return a bounded path offset for a multi-axis op mode."""
        if not self.path_experiment:
            axis = "xyz".index(self.axis)
            result = [0.0, 0.0, 0.0]
            result[axis] = self.value_at(elapsed)
            return tuple(result)

        elapsed = max(0.0, min(float(elapsed), self.duration))
        if elapsed >= self.duration:
            return (0.0, 0.0, 0.0)
        local = elapsed % self.cycle_duration
        if local >= self.ramp_time:
            return (0.0, 0.0, 0.0)
        phase = local / self.ramp_time

        if self.experiment == "square_test":
            # Four straight segments with a quintic time law on each edge.
            # The final 20% of each segment holds the corner so the AUV can
            # shed momentum before the next direction change.
            edge_position = phase * 4.0
            edge = min(int(edge_position), 3)
            u = self._smoothstep(min((edge_position - edge) / 0.8, 1.0))
            corners = (
                (0.0, 0.0),
                (self.amplitude, 0.0),
                (self.amplitude, self.amplitude),
                (0.0, self.amplitude),
                (0.0, 0.0),
            )
            start = corners[edge]
            end = corners[edge + 1]
            canonical = (
                start[0] + (end[0] - start[0]) * u,
                start[1] + (end[1] - start[1]) * u,
                0.0,
            )
        elif self.experiment == "follower_pid":
            theta = 2.0 * math.pi * self._smoothstep(phase)
            canonical = (
                self.amplitude * math.sin(theta),
                0.65 * self.amplitude * (1.0 - math.cos(theta)),
                0.0,
            )
        else:
            # A smooth planar cubic Bezier out and back. Quintic time scaling
            # makes velocity and acceleration zero at both endpoints.
            outward = phase <= 0.5
            u = phase * 2.0 if outward else (1.0 - phase) * 2.0
            u = self._smoothstep(u)
            p0 = (0.0, 0.0)
            p1 = (0.28, 0.65)
            p2 = (0.72, -0.55)
            p3 = (1.0, 0.0)
            inverse = 1.0 - u
            curve = tuple(
                self.amplitude
                * (
                    inverse**3 * p0[index]
                    + 3.0 * inverse**2 * u * p1[index]
                    + 3.0 * inverse * u**2 * p2[index]
                    + u**3 * p3[index]
                )
                for index in range(2)
            )
            canonical = (curve[0], curve[1], 0.0)

        x, y, z = canonical
        if self.axis == "xy":
            return (x, y, 0.0)
        if self.axis == "xz":
            return (x, 0.0, y)
        if self.axis == "yz":
            return (0.0, x, y)
        raise ValueError(f"unsupported path plane: {self.axis}")

    def preview(self, samples: int = 121) -> list[list[float]]:
        if not self.path_experiment:
            return []
        samples = max(2, min(int(samples), 301))
        return [
            list(self.vector_at(self.ramp_time * index / (samples - 1)))
            for index in range(samples)
        ]

    def public_state(self) -> dict:
        state = {**asdict(self), "duration": self.duration}
        if self.experiment == "trapezoid":
            state["ramp_slope"] = abs(self.amplitude) / self.ramp_time
        return state
