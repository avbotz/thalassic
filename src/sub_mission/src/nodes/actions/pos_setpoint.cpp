#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

namespace {

class PosSetpointAction : public BT::StatefulActionNode {
   public:
    PosSetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                      SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                      rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("x", UNSPECIFIED_PORT, "Odom x setpoint in meters (forward at the start)"),
                BT::InputPort<double>("y", UNSPECIFIED_PORT, "Odom y setpoint in meters (left at the start)"),
                BT::InputPort<double>("z", UNSPECIFIED_PORT, "Odom z setpoint in meters (up; 1 m deep is -1)"),
                moveTimeoutPort()};
    }

    BT::NodeStatus onStart() override {
        std::array<double, 3> inputs{};
        int timeout_msec = 0;
        if (!readPort(*this, "x", inputs[0], logger_) || !readPort(*this, "y", inputs[1], logger_) ||
            !readPort(*this, "z", inputs[2], logger_) || !readPort(*this, "timeout_msec", timeout_msec, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        active_axes_ = {std::isfinite(inputs[0]), std::isfinite(inputs[1]), std::isfinite(inputs[2])};
        std::array<double, 3> target = inputs;
        // x and y move together, so giving one keeps the other where it is meant to be.
        if (active_axes_[0] != active_axes_[1]) {
            const std::size_t other = active_axes_[0] ? 1 : 0;
            target[other] = node_.positionTarget(other);
        }
        node_.commandPosition(target, false);
        setpoint_publisher_->publish(positionCommand(*clock_, target, false));

        start_updates_ = node_.control_error_updates;
        const auto &pos = node_.measured_pos;
        const double horizontal =
            active_axes_[0] || active_axes_[1] ? std::hypot(target[0] - pos[0], target[1] - pos[1]) : 0.0;
        deadline_ = moveDeadline(timeout_msec, horizontal, active_axes_[2] ? target[2] - pos[2] : 0.0, 0.0);

        RCLCPP_INFO(logger_, "Published position command: xyz=(%.3f, %.3f, %.3f)", target[0], target[1], target[2]);
        return checkErrors();
    }

    BT::NodeStatus onRunning() override { return checkErrors(); }

    void onHalted() override { RCLCPP_INFO(logger_, "PosSetpoint wait halted."); }

   private:
    bool freshAndWithinTolerance(const std::size_t index) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= POSITION_TOLERANCE;
    }

    BT::NodeStatus checkErrors() const {
        if (!node_.subAlive()) {
            RCLCPP_WARN(logger_, "PosSetpoint wait failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }

        if ((!active_axes_[0] || freshAndWithinTolerance(0)) && (!active_axes_[1] || freshAndWithinTolerance(1)) &&
            (!active_axes_[2] || freshAndWithinTolerance(2))) {
            return BT::NodeStatus::SUCCESS;
        }

        if (std::chrono::steady_clock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "PosSetpoint timed out; error xyz=(%.3f, %.3f, %.3f).", node_.control_errors[0],
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

void registerPosSetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                               SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                               rclcpp::Logger logger) {
    factory.registerBuilder<PosSetpointAction>(
        "PosSetpoint",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<PosSetpointAction>(name, config, node, setpoint_publisher, clock, logger);
        });
}
