#include "sub_control/allocator.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace {

// A health or power limit clamped into [0, 1]. std::clamp passes NaN through,
// which would leave a thruster with NaN bounds and force; it counts as 0 (off).
double unit_fraction(double value) { return std::isnan(value) ? 0.0 : std::clamp(value, 0.0, 1.0); }

}  // namespace

ThrusterAllocator::ThrusterAllocator(const ThrusterGeometries& thrusters, const Eigen::Vector3d& center_of_gravity) {
    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        const Eigen::Vector3d axis = thrusters[i].direction.normalized();
        const Eigen::Vector3d r = thrusters[i].position - center_of_gravity;
        b_.block<3, 1>(0, i) = axis;
        b_.block<3, 1>(3, i) = r.cross(axis);
    }
}

Vector6d ThrusterAllocator::wrench_from_forces(const ThrusterArray& forces) const {
    return b_ * Eigen::Map<const Eigen::Matrix<double, NUM_THRUSTERS, 1>>(forces.data());
}

ThrusterArray ThrusterAllocator::allocate(const Vector6d& wrench, const ThrusterArray& lower,
                                          const ThrusterArray& upper, const Vector6d& axis_weights) const {
    using Forces = Eigen::Matrix<double, NUM_THRUSTERS, 1>;
    using Hessian = Eigen::Matrix<double, NUM_THRUSTERS, NUM_THRUSTERS>;

    // Weight on ||f||^2 relative to the wrench error. Small enough that it only
    // chooses among forces that achieve the wrench (the null space of B), but
    // nonzero so the problem is strictly convex and the solution unique.
    constexpr double POWER_WEIGHT = 1e-6;

    Eigen::Matrix<double, NUM_DOF, NUM_THRUSTERS> weighted_b;
    Vector6d weighted_wrench;
    for (int d = 0; d < NUM_DOF; ++d) {
        const double w = std::max(axis_weights[d], 1e-6);
        weighted_wrench(d) = w * wrench[d];
        weighted_b.row(d) = w * b_.row(d);
    }
    // Minimize 0.5 f'Hf - g'f subject to lower <= f <= upper.
    const Hessian H = weighted_b.transpose() * weighted_b + POWER_WEIGHT * Hessian::Identity();
    const Forces g = weighted_b.transpose() * weighted_wrench;

    // Primal active-set method. Every iterate is feasible; each pass either pins
    // a thruster at the bound it ran into or frees the one whose bound costs the
    // most. With eight variables it finishes in a handful of passes.
    enum class Bound { FREE, LOWER, UPPER, FIXED };
    std::array<Bound, NUM_THRUSTERS> state{};
    Forces f;
    for (int t = 0; t < NUM_THRUSTERS; ++t) {
        f(t) = std::clamp(0.0, lower[t], upper[t]);
        state[t] = (upper[t] - lower[t] <= 1e-9) ? Bound::FIXED : Bound::FREE;
    }

    const double tolerance = 1e-9 * (1.0 + g.cwiseAbs().maxCoeff());
    for (int pass = 0; pass < 4 * NUM_THRUSTERS; ++pass) {
        // Optimum over the free thrusters with the rest held where they are:
        // pinned rows become identity rows that reproduce the current value.
        Hessian K = H;
        Forces pinned = f;
        for (int t = 0; t < NUM_THRUSTERS; ++t) {
            if (state[t] == Bound::FREE) {
                pinned(t) = 0.0;
            }
        }
        Forces rhs = g - H * pinned;
        for (int t = 0; t < NUM_THRUSTERS; ++t) {
            if (state[t] != Bound::FREE) {
                K.row(t).setZero();
                K.col(t).setZero();
                K(t, t) = 1.0;
                rhs(t) = f(t);
            }
        }
        const Forces target = K.ldlt().solve(rhs);

        // Walk toward it, stopping at the first bound in the way.
        double step = 1.0;
        int blocking = -1;
        for (int t = 0; t < NUM_THRUSTERS; ++t) {
            if (state[t] != Bound::FREE || target(t) == f(t)) {
                continue;
            }
            const double bound = target(t) > upper[t] ? upper[t] : target(t) < lower[t] ? lower[t] : target(t);
            const double fraction = (bound - f(t)) / (target(t) - f(t));
            if (bound != target(t) && fraction < step) {
                step = fraction;
                blocking = t;
            }
        }
        f += step * (target - f);
        if (blocking >= 0) {
            const bool at_upper = target(blocking) > upper[blocking];
            f(blocking) = at_upper ? upper[blocking] : lower[blocking];
            state[blocking] = at_upper ? Bound::UPPER : Bound::LOWER;
            continue;
        }

        // At the optimum for this set of pinned thrusters. Release the one whose
        // bound most prevents a better solution; if none does, this is optimal.
        const Forces gradient = H * f - g;
        int release = -1;
        double worst = tolerance;
        for (int t = 0; t < NUM_THRUSTERS; ++t) {
            const double pressure = state[t] == Bound::LOWER   ? -gradient(t)
                                    : state[t] == Bound::UPPER ? gradient(t)
                                                               : 0.0;
            if (pressure > worst) {
                worst = pressure;
                release = t;
            }
        }
        if (release < 0) {
            break;
        }
        state[release] = Bound::FREE;
    }

    ThrusterArray result{};
    for (int t = 0; t < NUM_THRUSTERS; ++t) {
        result[t] = f(t);
    }
    return result;
}

AxisFlags ThrusterAllocator::controllable_axes(const std::array<bool, NUM_THRUSTERS>& usable,
                                               const std::array<int, NUM_DOF>& priority) const {
    // Gram-Schmidt over the rows of B restricted to the usable thrusters, in
    // priority order (PX4's control allocator drops dependent axes the same way).
    constexpr double DEPENDENT = 1e-3;

    Eigen::Matrix<double, NUM_DOF, NUM_THRUSTERS> rows = b_;
    for (int t = 0; t < NUM_THRUSTERS; ++t) {
        if (!usable[t]) {
            rows.col(t).setZero();
        }
    }

    AxisFlags controllable{};
    std::array<Eigen::Matrix<double, 1, NUM_THRUSTERS>, NUM_DOF> basis;
    int kept = 0;
    for (const int axis : priority) {
        const double full = b_.row(axis).norm();
        Eigen::Matrix<double, 1, NUM_THRUSTERS> residual = rows.row(axis);
        for (int k = 0; k < kept; ++k) {
            residual -= residual.dot(basis[k]) * basis[k];
        }
        if (full > 0.0 && residual.norm() > DEPENDENT * full) {
            basis[kept++] = residual.normalized();
            controllable[axis] = true;
        }
    }
    return controllable;
}

ThrusterOutput drive_thrusters(const ThrusterAllocator& allocator, const Vector6d& wrench,
                               const ThrusterSettings& settings, const ThrusterArray& previous, double dt) {
    const double power = std::min(unit_fraction(settings.power_limit), MAX_POWER_LIMIT);
    // The same range either way round: see thruster_command().
    const double reverse_limit = norm_to_force(-power);
    const double forward_limit = norm_to_force(power);

    ThrusterArray health{};
    ThrusterArray lower{};
    ThrusterArray upper{};
    for (int t = 0; t < NUM_THRUSTERS; ++t) {
        health[t] = unit_fraction(settings.health[t]);
        lower[t] = health[t] * reverse_limit;
        upper[t] = health[t] * forward_limit;
    }
    const ThrusterArray planned = allocator.allocate(wrench, lower, upper, settings.axis_weights);

    ThrusterOutput out;
    for (int t = 0; t < NUM_THRUSTERS; ++t) {
        if (health[t] <= 0.0) {
            continue;  // failed: commanded off
        }
        // A weakened thruster needs a bigger command for the same force.
        double command = std::clamp(thruster_command(planned[t] / health[t], settings.reversed[t]), -power, power);
        bool limited = false;
        if (settings.max_command_rate > 0.0 && dt > 0.0) {
            const double step = settings.max_command_rate * dt;
            // The power limit caps the slewed command too: right after the limit
            // is lowered, `previous` can be above it (dropping to it reverses nothing).
            const double slewed =
                std::clamp(std::clamp(command, previous[t] - step, previous[t] + step), -power, power);
            limited = slewed != command;
            command = slewed;
        }
        out.command[t] = command;
        out.force[t] = health[t] * thruster_force(command, settings.reversed[t]);
        const double margin = 1e-6 * (1.0 + forward_limit);
        out.saturated[t] = limited || planned[t] >= upper[t] - margin || planned[t] <= lower[t] + margin;
    }
    out.achieved = allocator.wrench_from_forces(out.force);
    return out;
}
