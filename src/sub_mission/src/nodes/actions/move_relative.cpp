#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

namespace {

// MoveRelative offsets along the commanded heading (forward, left, up);
// MoveRelativePos along odom's axes, whatever the heading.
template <bool ODOM_AXES>
class MoveRelativeAction : public BT::StatefulActionNode {
   public:
    static constexpr const char *NAME = ODOM_AXES ? "MoveRelativePos" : "MoveRelative";

    MoveRelativeAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                       SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                       rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        if (ODOM_AXES) {
            return {BT::InputPort<double>("x", 0.0, "Offset along odom x in meters"),
                    BT::InputPort<double>("y", 0.0, "Offset along odom y in meters"),
                    BT::InputPort<double>("z", 0.0, "Up offset in meters"), moveTimeoutPort()};
        }
        return {BT::InputPort<double>("x", 0.0, "Forward offset in meters"),
                BT::InputPort<double>("y", 0.0, "Left offset in meters"),
                BT::InputPort<double>("z", 0.0, "Up offset in meters"), moveTimeoutPort()};
    }

    BT::NodeStatus onStart() override {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        int timeout_msec = 0;
        if (!readPort(*this, "x", x, logger_) || !readPort(*this, "y", y, logger_) ||
            !readPort(*this, "z", z, logger_) || !readPort(*this, "timeout_msec", timeout_msec, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        // NaN keeps an axis where it is, as in the trees' x="nan".
        x = std::isfinite(x) ? x : 0.0;
        y = std::isfinite(y) ? y : 0.0;
        z = std::isfinite(z) ? z : 0.0;

        const double yaw = ODOM_AXES ? 0.0 : node_.attitudeTarget(2);
        const double dx = std::cos(yaw) * x - std::sin(yaw) * y;
        const double dy = std::sin(yaw) * x + std::cos(yaw) * y;

        active_axes_ = {std::fabs(x) > 0.0 || std::fabs(y) > 0.0, std::fabs(x) > 0.0 || std::fabs(y) > 0.0,
                        std::fabs(z) > 0.0};
        // Only the axes that move: the rest, z held as an altitude included, carry on.
        std::array<double, 3> target{UNSPECIFIED_PORT, UNSPECIFIED_PORT, UNSPECIFIED_PORT};
        if (active_axes_[0]) {
            target[0] = node_.positionTarget(0) + dx;
            target[1] = node_.positionTarget(1) + dy;
        }
        // z is up whether it holds an odom z or an altitude.
        const bool altitude = node_.commanded_altitude;
        if (active_axes_[2]) {
            target[2] = node_.positionTarget(2) + z;
        }
        // An altitude below the bottom: sub_control would reject the whole
        // command, and control/error would go on describing the old target,
        // which can read as arrival.
        if (altitude && active_axes_[2] && target[2] < 0.0) {
            RCLCPP_ERROR(logger_, "%s: altitude target %.3f is below the bottom.", NAME, target[2]);
            return BT::NodeStatus::FAILURE;
        }
        node_.commandPosition(target, altitude);
        start_updates_ = node_.control_error_updates;
        deadline_ = moveDeadline(timeout_msec, std::hypot(x, y), z, 0.0);
        setpoint_publisher_->publish(positionCommand(*clock_, target, altitude));

        RCLCPP_INFO(logger_, "%s: offset=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f)", NAME, x, y, z, target[0],
                    target[1], target[2]);
        return checkErrors();
    }

    BT::NodeStatus onRunning() override { return checkErrors(); }

    void onHalted() override { RCLCPP_INFO(logger_, "%s wait halted.", NAME); }

   private:
    bool freshAndWithinTolerance(const std::size_t index) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= POSITION_TOLERANCE;
    }

    BT::NodeStatus checkErrors() const {
        if (!node_.subAlive()) {
            RCLCPP_WARN(logger_, "%s failed because kill switch is engaged.", NAME);
            return BT::NodeStatus::FAILURE;
        }
        if ((!active_axes_[0] || freshAndWithinTolerance(0)) && (!active_axes_[1] || freshAndWithinTolerance(1)) &&
            (!active_axes_[2] || freshAndWithinTolerance(2))) {
            return BT::NodeStatus::SUCCESS;
        }
        if (std::chrono::steady_clock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "%s timed out; error xyz=(%.3f, %.3f, %.3f).", NAME, node_.control_errors[0],
                        node_.control_errors[1], node_.control_errors[2]);
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<bool, 3> active_axes_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
    std::chrono::steady_clock::time_point deadline_;
};

}  // namespace

void registerMoveRelativeAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                rclcpp::Logger logger) {
    factory.registerBuilder<MoveRelativeAction<false>>(
        "MoveRelative",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<MoveRelativeAction<false>>(name, config, node, setpoint_publisher, clock, logger);
        });
    factory.registerBuilder<MoveRelativeAction<true>>(
        "MoveRelativePos",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<MoveRelativeAction<true>>(name, config, node, setpoint_publisher, clock, logger);
        });
}
