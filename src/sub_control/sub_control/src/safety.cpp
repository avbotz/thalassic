#include "sub_control/safety.hpp"

#include <format>
#include <limits>

double message_age(const std::optional<double>& stamp, double now) {
    if (!stamp) {
        return std::numeric_limits<double>::infinity();
    }
    const double age = now - *stamp;
    if (age < -FUTURE_STAMP_TOLERANCE) {
        return std::numeric_limits<double>::infinity();
    }
    return age > 0.0 ? age : 0.0;
}

std::vector<std::string> arming_blockers(const ArmingInputs& inputs, const ArmingLimits& limits) {
    std::vector<std::string> blockers;
    if (limits.require_kill_cycle && !inputs.kill_cycle_seen) {
        blockers.emplace_back(
            "the kill switch was already released when sub_control started: engage it and release it again");
    }
    const auto check = [&blockers](const char* what, double age, double timeout) {
        if (!(age <= timeout)) {
            blockers.push_back(age == std::numeric_limits<double>::infinity()
                                   ? std::format("no {}", what)
                                   : std::format("{} is {:.1f} s old (limit {:.1f} s)", what, age, timeout));
        }
    };
    check("state estimate (odometry/filtered)", inputs.state_age, limits.state_timeout);
    check("IMU (imu/data)", inputs.imu_age, limits.attitude_timeout);
    check("depth (odometry/depth)", inputs.depth_age, limits.depth_timeout);
    return blockers;
}
