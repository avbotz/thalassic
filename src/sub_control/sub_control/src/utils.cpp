#include "sub_control/utils.hpp"

#include "sub_control/thrust_table.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace {

// The thrust curve at the supply voltage in use, interpolated between the
// measured ones (generated from data/t200_thrust.csv).
using ThrustCurve = std::array<double, THRUST_COMMAND_COUNT>;

ThrustCurve curve_at(double voltage) {
    voltage = std::clamp(voltage, THRUST_VOLTAGES.front(), THRUST_VOLTAGES.back());
    size_t upper = 1;
    while (upper < THRUST_VOLTAGES.size() - 1 && THRUST_VOLTAGES[upper] < voltage) {
        ++upper;
    }
    const double fraction =
        (voltage - THRUST_VOLTAGES[upper - 1]) / (THRUST_VOLTAGES[upper] - THRUST_VOLTAGES[upper - 1]);
    ThrustCurve curve{};
    for (size_t i = 0; i < curve.size(); ++i) {
        curve[i] = THRUST_FORCES[upper - 1][i] + fraction * (THRUST_FORCES[upper][i] - THRUST_FORCES[upper - 1][i]);
    }
    return curve;
}

double active_voltage = DEFAULT_THRUSTER_VOLTAGE;
ThrustCurve active_curve = curve_at(DEFAULT_THRUSTER_VOLTAGE);

}  // namespace

void set_thruster_voltage(double voltage) {
    if (std::isnan(voltage)) {
        return;
    }
    active_voltage = std::clamp(voltage, THRUST_VOLTAGES.front(), THRUST_VOLTAGES.back());
    active_curve = curve_at(active_voltage);
}

double thruster_voltage() { return active_voltage; }

double force_to_norm(double force_n) {
    // NaN fails every comparison below and would fall through to full scale.
    if (std::isnan(force_n) || std::fabs(force_n) < 1e-6) {
        return 0.0;
    }
    if (force_n <= active_curve.front()) {
        return -1.0;
    }
    if (force_n >= active_curve.back()) {
        return 1.0;
    }

    // The first command to give the force, on the side of zero the force is on:
    // the curve is flat (zero) across its deadband, and measurement noise makes
    // a few neighbouring points not quite monotonic.
    const size_t zero = (active_curve.size() - 1) / 2;
    if (force_n > 0.0) {
        for (size_t i = zero + 1; i < active_curve.size(); ++i) {
            if (active_curve[i] >= force_n) {
                const double o0 = active_curve[i - 1];
                const double o1 = active_curve[i];
                const double t = (o1 > o0) ? (force_n - o0) / (o1 - o0) : 0.0;
                return THRUST_COMMANDS[i - 1] + t * (THRUST_COMMANDS[i] - THRUST_COMMANDS[i - 1]);
            }
        }
        return 1.0;
    }
    for (size_t i = zero; i-- > 0;) {
        if (active_curve[i] <= force_n) {
            const double o0 = active_curve[i + 1];
            const double o1 = active_curve[i];
            const double t = (o1 < o0) ? (force_n - o0) / (o1 - o0) : 0.0;
            return THRUST_COMMANDS[i + 1] + t * (THRUST_COMMANDS[i] - THRUST_COMMANDS[i + 1]);
        }
    }
    return -1.0;
}

double norm_to_force(double normalized) {
    if (std::isnan(normalized)) {
        return 0.0;
    }
    normalized = std::clamp(normalized, -1.0, 1.0);
    if (normalized <= THRUST_COMMANDS.front()) {
        return active_curve.front();
    }
    if (normalized >= THRUST_COMMANDS.back()) {
        return active_curve.back();
    }

    for (size_t i = 1; i < active_curve.size(); ++i) {
        if (THRUST_COMMANDS[i] >= normalized) {
            const double x0 = THRUST_COMMANDS[i - 1];
            const double x1 = THRUST_COMMANDS[i];
            const double fraction = (normalized - x0) / (x1 - x0);
            return active_curve[i - 1] + fraction * (active_curve[i] - active_curve[i - 1]);
        }
    }
    return active_curve.back();
}

double thruster_command(double force_n, bool reversed) {
    return reversed ? -force_to_norm(force_n) : force_to_norm(force_n);
}

double thruster_force(double command, bool reversed) {
    return reversed ? norm_to_force(-command) : norm_to_force(command);
}

double wrap_angle(double angle) { return std::remainder(angle, 2.0 * std::numbers::pi); }

Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d& rpy) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()));
}

Eigen::Vector3d rpy_from_quaternion(const Eigen::Quaterniond& q) {
    const Eigen::Quaterniond n = q.normalized();
    const double w = n.w();
    const double x = n.x();
    const double y = n.y();
    const double z = n.z();
    const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
    const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return {roll, pitch, yaw};
}

Eigen::Vector3d attitude_error(const Eigen::Quaterniond& target, const Eigen::Quaterniond& current) {
    Eigen::Quaterniond error = current.normalized().conjugate() * target.normalized();
    if (error.w() < 0.0) {
        error.coeffs() *= -1.0;  // the same rotation the short way round
    }
    const Eigen::Vector3d axis = error.vec();
    const double sine = axis.norm();
    if (sine < 1e-9) {
        return 2.0 * axis;
    }
    return (2.0 * std::atan2(sine, error.w()) / sine) * axis;
}

Eigen::Vector3d euler_rates_to_body(const Eigen::Vector3d& rpy, const Eigen::Vector3d& rates) {
    const double sr = std::sin(rpy.x());
    const double cr = std::cos(rpy.x());
    const double sp = std::sin(rpy.y());
    const double cp = std::cos(rpy.y());
    return {rates.x() - sp * rates.z(), cr * rates.y() + sr * cp * rates.z(), -sr * rates.y() + cr * cp * rates.z()};
}
