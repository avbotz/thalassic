from __future__ import annotations

from dataclasses import dataclass
import math
import statistics


AXES = ("x", "y", "z", "roll", "pitch", "yaw")
ROTATIONAL_AXES = {"roll", "pitch", "yaw"}


@dataclass(frozen=True)
class CharacterizationConfig:
    axis: str
    amplitude: float
    slow_slope: float
    fast_slope: float
    dwell: float

    def __post_init__(self) -> None:
        values = (self.amplitude, self.slow_slope, self.fast_slope, self.dwell)
        if self.axis not in AXES:
            raise ValueError("characterization axis must be x/y/z/roll/pitch/yaw")
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("characterization values must be finite and positive")
        maximum = 0.8 if self.axis in ROTATIONAL_AXES else 0.5
        minimum = 0.03 if self.axis in ROTATIONAL_AXES else 0.015
        if not minimum <= self.amplitude <= maximum:
            unit = "rad/s" if self.axis in ROTATIONAL_AXES else "m/s"
            raise ValueError(
                f"{self.axis} amplitude must be between {minimum:g} and {maximum:g} {unit}"
            )
        if self.fast_slope < self.slow_slope * 1.5:
            raise ValueError("fast slope must be at least 1.5 times the slow slope")
        if self.amplitude / self.slow_slope > 30.0:
            raise ValueError("slow ramp must reach its command within 30 seconds")
        if self.amplitude / self.fast_slope < 0.4:
            raise ValueError("fast ramp must take at least 0.4 seconds")
        if not 0.5 <= self.dwell <= 10.0:
            raise ValueError("dwell must be between 0.5 and 10 seconds")

    @property
    def mode(self) -> str:
        return "angular_velocity" if self.axis in ROTATIONAL_AXES else "velocity"

    @property
    def segments(self) -> tuple[tuple[str, float, float, float], ...]:
        # Two cruise speeds are required to distinguish linear from quadratic
        # drag.  The low-speed passes also make the test less aggressive than
        # repeatedly driving the vehicle to its maximum characterization speed.
        low_amplitude = 0.5 * self.amplitude
        slow_time = low_amplitude / self.slow_slope
        fast_time = self.amplitude / self.fast_slope
        a = self.amplitude
        low = low_amplitude
        d = self.dwell
        return (
            ("bias hold", 2.0 * d, 0.0, 0.0),
            ("low ramp +", slow_time, 0.0, low),
            ("low cruise +", 2.0 * d, low, low),
            ("low return +", slow_time, low, 0.0),
            ("zero settle", d, 0.0, 0.0),
            ("low ramp -", slow_time, 0.0, -low),
            ("low cruise -", 2.0 * d, -low, -low),
            ("low return -", slow_time, -low, 0.0),
            ("zero settle", d, 0.0, 0.0),
            ("fast ramp +", fast_time, 0.0, a),
            ("fast cruise +", d, a, a),
            ("fast return +", fast_time, a, 0.0),
            ("zero settle", d, 0.0, 0.0),
            ("fast ramp -", fast_time, 0.0, -a),
            ("fast cruise -", d, -a, -a),
            ("fast return -", fast_time, -a, 0.0),
            ("final hold", 2.0 * d, 0.0, 0.0),
        )

    @property
    def duration(self) -> float:
        return sum(segment[1] for segment in self.segments)

    def command_at(self, elapsed: float) -> tuple[float, str]:
        remaining = max(0.0, float(elapsed))
        for name, duration, start, end in self.segments:
            if remaining <= duration:
                fraction = min(max(remaining / duration, 0.0), 1.0)
                return start + (end - start) * fraction, name
            remaining -= duration
        return 0.0, "complete"


@dataclass(frozen=True)
class CharacterizationSample:
    time: float
    phase: str
    velocity: float
    wrench: float
    configuration: float
    residual: float
    thruster_fraction: float
    cross_velocity: float


def _solve(matrix: list[list[float]], vector: list[float]) -> tuple[list[float], float]:
    size = len(vector)
    augmented = [list(row) + [value] for row, value in zip(matrix, vector)]
    pivots = []
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        value = abs(augmented[pivot][column])
        if value < 1e-10:
            raise ValueError("identification data is singular; use a wider speed range")
        pivots.append(value)
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        augmented[column] = [item / divisor for item in augmented[column]]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                current - factor * source
                for current, source in zip(augmented[row], augmented[column])
            ]
    return [augmented[index][-1] for index in range(size)], max(pivots) / min(pivots)


def _weighted_fit(
    rows: list[list[float]], values: list[float], weights: list[float]
) -> tuple[list[float], float]:
    columns = len(rows[0])
    scales = []
    for column in range(columns):
        if column == 0:
            scales.append(1.0)
            continue
        rms = math.sqrt(
            sum(weight * row[column] ** 2 for row, weight in zip(rows, weights))
            / max(sum(weights), 1e-12)
        )
        scales.append(max(rms, 1e-8))
    normalized = [
        [value / scales[index] for index, value in enumerate(row)] for row in rows
    ]
    normal = [[0.0] * columns for _ in range(columns)]
    target = [0.0] * columns
    for row, value, weight in zip(normalized, values, weights):
        for left in range(columns):
            target[left] += weight * row[left] * value
            for right in range(columns):
                normal[left][right] += weight * row[left] * row[right]
    ridge = 1e-7 * max(sum(normal[index][index] for index in range(columns)), 1.0)
    for index in range(1, columns):
        normal[index][index] += ridge
    normalized_coefficients, pivot_ratio = _solve(normal, target)
    return [
        coefficient / scales[index]
        for index, coefficient in enumerate(normalized_coefficients)
    ], pivot_ratio


def _robust_fit(
    rows: list[list[float]], values: list[float]
) -> tuple[list[float], list[float], float]:
    weights = [1.0] * len(values)
    coefficients: list[float] = []
    pivot_ratio = 1.0
    for _ in range(8):
        coefficients, pivot_ratio = _weighted_fit(rows, values, weights)
        residuals = [
            value - sum(coefficient * feature for coefficient, feature in zip(coefficients, row))
            for row, value in zip(rows, values)
        ]
        center = statistics.median(residuals)
        mad = statistics.median(abs(residual - center) for residual in residuals)
        scale = max(1.4826 * mad, 1e-6)
        cutoff = 1.5 * scale
        weights = [
            1.0 if abs(residual - center) <= cutoff else cutoff / abs(residual - center)
            for residual in residuals
        ]
    return coefficients, weights, pivot_ratio


def _accelerations(samples: list[CharacterizationSample], window: float = 0.22) -> list[float]:
    result = []
    for sample in samples:
        neighborhood = [other for other in samples if abs(other.time - sample.time) <= window]
        if len(neighborhood) < 5:
            result.append(float("nan"))
            continue
        mean_time = sum(other.time for other in neighborhood) / len(neighborhood)
        mean_velocity = sum(other.velocity for other in neighborhood) / len(neighborhood)
        denominator = sum((other.time - mean_time) ** 2 for other in neighborhood)
        numerator = sum(
            (other.time - mean_time) * (other.velocity - mean_velocity)
            for other in neighborhood
        )
        result.append(numerator / denominator if denominator > 1e-9 else float("nan"))
    return result


def fit_feedforward(
    samples: list[CharacterizationSample],
    config: CharacterizationConfig,
    baseline_coefficients: dict[str, float] | None = None,
) -> dict:
    ordered = sorted(samples, key=lambda sample: sample.time)
    accelerations = _accelerations(ordered)
    finite = [
        (sample, acceleration)
        for sample, acceleration in zip(ordered, accelerations)
        if math.isfinite(acceleration)
    ]
    saturated_count = sum(sample.thruster_fraction >= 0.98 for sample, _ in finite)
    allocation_limited_count = sum(
        abs(sample.residual) > max(0.75, 0.12 * abs(sample.wrench))
        for sample, _ in finite
    )
    usable = [
        (sample, acceleration)
        for sample, acceleration in finite
        if sample.thruster_fraction < 0.98
        and abs(sample.residual) <= max(0.75, 0.12 * abs(sample.wrench))
    ]
    if len(usable) < 120:
        raise ValueError("not enough unsaturated telemetry for a trustworthy fit")

    velocities = [sample.velocity for sample, _ in usable]
    actual_accelerations = [acceleration for _, acceleration in usable]
    maximum_positive = max(velocities)
    maximum_negative = min(velocities)
    acceleration_span = max(actual_accelerations) - min(actual_accelerations)
    saturation_fraction = saturated_count / max(len(finite), 1)
    allocation_limited_fraction = allocation_limited_count / max(len(finite), 1)
    rejected_fraction = 1.0 - len(usable) / max(len(finite), 1)
    cross_rms = math.sqrt(
        sum(sample.cross_velocity**2 for sample, _ in usable) / len(usable)
    )
    failures = []
    warnings = []
    minimum_response = 0.25 * config.amplitude
    if maximum_positive < minimum_response or maximum_negative > -minimum_response:
        failures.append("the vehicle did not produce useful motion in both directions")
    if acceleration_span < 0.25 * config.fast_slope:
        failures.append("measured acceleration was too small to identify kA")
    if saturation_fraction > 0.15:
        failures.append("thrusters saturated for more than 15% of the test")
    if allocation_limited_fraction > 0.15:
        failures.append("allocation residual was excessive for more than 15% of the test")

    include_restoring = config.axis in {"roll", "pitch"}

    # Fit drag only from settled holds and cruises.  A single joint fit can
    # report an excellent R² while trading acceleration against drag because
    # both rise together on a ramp.  Separating the steady and transient data
    # makes each coefficient answer a physically distinct part of the test.
    steady_acceleration_limit = max(0.12 * config.fast_slope, 0.003)
    steady = [
        (sample, acceleration)
        for sample, acceleration in usable
        if ("cruise" in sample.phase or "hold" in sample.phase or "settle" in sample.phase)
        and abs(acceleration) <= steady_acceleration_limit
    ]
    minimum_steady_samples = 60
    if len(steady) < minimum_steady_samples:
        failures.append("the vehicle did not settle long enough to identify drag")

    drag_rows = []
    drag_values = []
    for sample, _ in steady:
        # The deliberately simple Road Runner-style calibration identifies an
        # effective kV over the tested operating range.  With only two modest
        # pool-safe speeds, kV and kQ trade strongly while predicting nearly
        # the same thrust.  Applying both gives fragile extrapolation, so the
        # dashboard zeros kQ and leaves a future wide-range test to identify it.
        row = [1.0, sample.velocity]
        if include_restoring:
            row.append(math.sin(sample.configuration))
        drag_rows.append(row)
        drag_values.append(sample.wrench)
    try:
        drag_coefficients, drag_weights, drag_condition = _robust_fit(
            drag_rows, drag_values
        )
    except (ValueError, IndexError):
        failures.append("steady data could not separate bias and velocity feedforward")
        drag_coefficients = [0.0, 0.0] + ([0.0] if include_restoring else [])
        drag_weights = [1.0] * len(drag_values)
        drag_condition = float("inf")

    bias, linear_drag = drag_coefficients[:2]
    quadratic_drag = 0.0
    restoring = drag_coefficients[2] if include_restoring else 0.0

    transient_acceleration_minimum = max(0.10 * config.fast_slope, 0.002)
    transient = [
        (sample, acceleration)
        for sample, acceleration in usable
        if ("ramp" in sample.phase or "return" in sample.phase)
        and abs(acceleration) >= transient_acceleration_minimum
    ]
    if len(transient) < 40:
        failures.append("not enough acceleration telemetry remained to identify kA")

    mass_rows = []
    mass_values = []
    for sample, acceleration in transient:
        steady_prediction = (
            bias
            + linear_drag * sample.velocity
            + quadratic_drag * abs(sample.velocity) * sample.velocity
            + restoring * math.sin(sample.configuration)
        )
        mass_rows.append([acceleration])
        mass_values.append(sample.wrench - steady_prediction)
    try:
        mass_coefficients, mass_weights, mass_condition = _robust_fit(
            mass_rows, mass_values
        )
        mass_estimate = mass_coefficients[0]
    except (ValueError, IndexError):
        failures.append("acceleration data could not identify effective mass")
        mass_estimate = 0.0
        mass_weights = [1.0] * len(mass_values)
        mass_condition = float("inf")

    effective_mass = mass_estimate
    if baseline_coefficients is not None:
        baseline_mass = float(baseline_coefficients.get("effective_mass", 0.0))
        if math.isfinite(baseline_mass) and baseline_mass > 0.0:
            effective_mass = baseline_mass
            warnings.append(
                f"kA estimate {mass_estimate:.3g} is diagnostic only; retained current {baseline_mass:.3g}"
            )

    rows = []
    values = []
    for sample, acceleration in usable:
        row = [1.0, sample.velocity, abs(sample.velocity) * sample.velocity, acceleration]
        if include_restoring:
            row.append(math.sin(sample.configuration))
        rows.append(row)
        values.append(sample.wrench)
    # Global fit quality uses the observed transient mass estimate.  The value
    # proposed for application may intentionally retain the current kA.
    coefficients = [bias, linear_drag, quadratic_drag, mass_estimate]
    if include_restoring:
        coefficients.append(restoring)
    predictions = [
        sum(coefficient * feature for coefficient, feature in zip(coefficients, row))
        for row in rows
    ]
    # Use the robust weights from the relevant stage for the combined quality
    # score, leaving ordinary samples at full weight.
    steady_weight_by_id = {id(sample): weight for (sample, _), weight in zip(steady, drag_weights)}
    transient_weight_by_id = {
        id(sample): weight for (sample, _), weight in zip(transient, mass_weights)
    }
    weights = [
        min(steady_weight_by_id.get(id(sample), 1.0), transient_weight_by_id.get(id(sample), 1.0))
        for sample, _ in usable
    ]
    weighted_total = sum(weights)
    mean = sum(weight * value for weight, value in zip(weights, values)) / weighted_total
    squared_error = sum(
        weight * (value - prediction) ** 2
        for weight, value, prediction in zip(weights, values, predictions)
    )
    squared_total = sum(weight * (value - mean) ** 2 for weight, value in zip(weights, values))
    r_squared = 1.0 - squared_error / max(squared_total, 1e-12)
    rmse = math.sqrt(squared_error / weighted_total)
    wrench_span = max(values) - min(values)
    normalized_rmse = rmse / max(wrench_span, 1e-6)
    outlier_fraction = sum(weight < 0.5 for weight in weights) / len(weights)

    drag_predictions = [
        bias
        + linear_drag * sample.velocity
        + restoring * math.sin(sample.configuration)
        for sample, _ in steady
    ]
    drag_weight_total = max(sum(drag_weights), 1e-12)
    drag_mean = (
        sum(weight * value for weight, value in zip(drag_weights, drag_values))
        / drag_weight_total
    )
    drag_squared_error = sum(
        weight * (value - prediction) ** 2
        for weight, value, prediction in zip(
            drag_weights, drag_values, drag_predictions
        )
    )
    drag_squared_total = sum(
        weight * (value - drag_mean) ** 2
        for weight, value in zip(drag_weights, drag_values)
    )
    drag_r_squared = 1.0 - drag_squared_error / max(drag_squared_total, 1e-12)

    physical = {
        "linear_drag": linear_drag,
        "quadratic_drag": quadratic_drag,
        "effective_mass": effective_mass,
        "trim": bias,
        "restoring_stiffness": restoring,
    }
    for name in ("linear_drag", "quadratic_drag", "effective_mass"):
        if physical[name] < 0.0:
            failures.append(f"fitted {name} was negative")
    if mass_estimate < 0.0:
        failures.append("fitted effective_mass was negative")
    if include_restoring and restoring < 0.0:
        failures.append("fitted restoring stiffness was negative")
    if drag_r_squared < 0.80:
        failures.append(f"steady feedforward quality was too low (R²={drag_r_squared:.2f})")
    condition = max(drag_condition, mass_condition)
    if drag_condition > 2.0e7:
        failures.append("cruise speeds did not independently excite the drag terms")
    cross_limit = max(0.04, 0.75 * config.amplitude)
    if cross_rms > cross_limit:
        failures.append("cross-axis motion was too large for a diagonal-axis fit")

    return {
        "passed": not failures,
        "failures": failures,
        "warnings": warnings,
        "coefficients": physical,
        "quality": {
            "samples": len(ordered),
            "usable_samples": len(usable),
            "r_squared": r_squared,
            "drag_r_squared": drag_r_squared,
            "rmse": rmse,
            "normalized_rmse": normalized_rmse,
            "outlier_fraction": outlier_fraction,
            "saturation_fraction": saturation_fraction,
            "allocation_limited_fraction": allocation_limited_fraction,
            "rejected_fraction": rejected_fraction,
            "condition": condition,
            "drag_condition": drag_condition,
            "mass_condition": mass_condition,
            "steady_samples": len(steady),
            "transient_samples": len(transient),
            "candidate_effective_mass": mass_estimate,
            "maximum_positive_velocity": maximum_positive,
            "maximum_negative_velocity": maximum_negative,
            "acceleration_span": acceleration_span,
            "cross_axis_rms": cross_rms,
        },
    }
