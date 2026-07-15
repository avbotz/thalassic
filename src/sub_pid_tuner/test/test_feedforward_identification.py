import math
import random

import pytest

from sub_pid_tuner.feedforward_identification import (
    CharacterizationConfig,
    CharacterizationSample,
    fit_feedforward,
)


def test_characterization_profile_has_slow_fast_and_bidirectional_segments():
    config = CharacterizationConfig("x", 0.12, 0.03, 0.12, 1.0)
    assert config.command_at(0.0) == (0.0, "bias hold")
    commands = [config.command_at(config.duration * index / 500)[0] for index in range(501)]
    assert max(commands) == pytest.approx(0.12, abs=1e-3)
    assert min(commands) == pytest.approx(-0.12, abs=1e-3)
    assert config.command_at(config.duration + 1.0) == (0.0, "complete")


def test_characterization_rejects_unsafe_or_uninformative_configuration():
    with pytest.raises(ValueError, match="1.5 times"):
        CharacterizationConfig("yaw", 0.2, 0.1, 0.12, 1.0)
    with pytest.raises(ValueError, match="within 30 seconds"):
        CharacterizationConfig("x", 0.2, 0.001, 0.1, 1.0)
    with pytest.raises(ValueError, match="between"):
        CharacterizationConfig("z", 0.8, 0.1, 0.3, 1.0)


def test_robust_fit_recovers_known_axis_dynamics_with_noise_and_outliers():
    rng = random.Random(7)
    config = CharacterizationConfig("x", 0.16, 0.04, 0.16, 0.8)
    dt = 1.0 / 30.0
    velocity = 0.0
    mass = 43.0
    linear = 115.0
    quadratic = 0.0
    bias = -1.8
    samples = []
    previous_velocity = velocity
    for index in range(int(config.duration / dt)):
        elapsed = index * dt
        target, phase = config.command_at(elapsed)
        # A bandwidth-limited closed-loop response is intentionally used here;
        # the fitter must identify the plant from measured motion, not assume
        # that target and current are equal.
        velocity += (target - velocity) * min(dt / 0.35, 1.0)
        acceleration = (velocity - previous_velocity) / dt
        previous_velocity = velocity
        wrench = (
            bias
            + linear * velocity
            + quadratic * abs(velocity) * velocity
            + mass * acceleration
            + rng.gauss(0.0, 0.18)
        )
        if index % 173 == 0:
            wrench += rng.choice((-8.0, 8.0))
        samples.append(
            CharacterizationSample(
                elapsed,
                phase,
                velocity + rng.gauss(0.0, 0.0005),
                wrench,
                0.0,
                0.01,
                0.45,
                0.005,
            )
        )

    result = fit_feedforward(samples, config)
    assert result["passed"], result["failures"]
    fitted = result["coefficients"]
    assert fitted["trim"] == pytest.approx(bias, abs=0.4)
    assert fitted["linear_drag"] == pytest.approx(linear, rel=0.18)
    assert fitted["quadratic_drag"] == 0.0
    assert fitted["effective_mass"] == pytest.approx(mass, rel=0.15)
    assert result["quality"]["r_squared"] > 0.9


def test_dashboard_fit_retains_baseline_mass_and_reports_dynamic_candidate():
    rng = random.Random(11)
    config = CharacterizationConfig("x", 0.14, 0.035, 0.14, 1.2)
    dt = 1.0 / 30.0
    velocity = 0.0
    previous_velocity = 0.0
    samples = []
    for index in range(int(config.duration / dt)):
        elapsed = index * dt
        target, phase = config.command_at(elapsed)
        velocity += (target - velocity) * min(dt / 0.3, 1.0)
        acceleration = (velocity - previous_velocity) / dt
        previous_velocity = velocity
        samples.append(
            CharacterizationSample(
                elapsed,
                phase,
                velocity,
                -0.7 + 130.0 * velocity + 72.0 * acceleration + rng.gauss(0.0, 0.1),
                0.0,
                0.01,
                0.4,
                0.002,
            )
        )
    result = fit_feedforward(samples, config, {"effective_mass": 45.0})
    assert result["passed"], result["failures"]
    assert result["coefficients"]["effective_mass"] == 45.0
    assert result["quality"]["candidate_effective_mass"] == pytest.approx(72.0, rel=0.15)
    assert result["warnings"]


def test_fit_refuses_one_sided_or_saturated_data():
    config = CharacterizationConfig("x", 0.1, 0.04, 0.12, 0.8)
    samples = [
        CharacterizationSample(
            index / 30.0,
            "bad",
            0.05 + 0.01 * math.sin(index / 10),
            5.0,
            0.0,
            2.0,
            1.0,
            0.0,
        )
        for index in range(300)
    ]
    with pytest.raises(ValueError, match="not enough unsaturated telemetry"):
        fit_feedforward(samples, config)
