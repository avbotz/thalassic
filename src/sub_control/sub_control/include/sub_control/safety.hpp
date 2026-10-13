#ifndef SUB_CONTROL_SAFETY_HPP_
#define SUB_CONTROL_SAFETY_HPP_

#include <optional>
#include <string>
#include <vector>

// A stamp further ahead of this node's clock than this is not believed: the
// clock went back (the simulator restarted), or the sender's clock is not
// synced with this one (the DVL stamps its reports with its own clock).
inline constexpr double FUTURE_STAMP_TOLERANCE = 0.25;  // [s]

// How old a message stamped `stamp` is at `now` (both clock seconds): infinite
// if there is none or it is stamped further ahead than FUTURE_STAMP_TOLERANCE,
// zero if it is stamped a little ahead.
double message_age(const std::optional<double>& stamp, double now);

// Ages [s] of what the vehicle needs to be measured to be driven at all.
struct ArmingInputs {
    bool kill_cycle_seen{false};  // the kill switch has been engaged since startup
    double state_age{0.0};        // odometry/filtered
    double imu_age{0.0};          // attitude and rates
    double depth_age{0.0};        // odometry/depth
};

struct ArmingLimits {
    bool require_kill_cycle{true};
    double state_timeout{0.5};
    double attitude_timeout{1.0};
    double depth_timeout{1.0};
};

// Why sub_control may not arm after a kill switch release (empty if it may).
// It arms only once the switch has been engaged since startup, so a stack that
// starts or restarts with the switch released does not drive off, and only
// with a fresh state estimate, attitude and depth. The DVL is not needed: x/y
// feedback stays off until it has bottom lock.
std::vector<std::string> arming_blockers(const ArmingInputs& inputs, const ArmingLimits& limits);

#endif  // SUB_CONTROL_SAFETY_HPP_
