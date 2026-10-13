#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

namespace {

class AddAttSetpointAction : public BT::StatefulActionNode {
   public:
    AddAttSetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                         SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                         rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("roll", UNSPECIFIED_PORT, "Roll offset in radians"),
                BT::InputPort<double>("pitch", UNSPECIFIED_PORT, "Pitch offset in radians"),
                BT::InputPort<double>("yaw", UNSPECIFIED_PORT, "Yaw offset in radians; positive turns left"),
                moveTimeoutPort()};
    }

    BT::NodeStatus onStart() override {
        std::array<double, 3> inputs{};
        int timeout_msec = 0;
        if (!readPort(*this, "roll", inputs[0], logger_) || !readPort(*this, "pitch", inputs[1], logger_) ||
            !readPort(*this, "yaw", inputs[2], logger_) || !readPort(*this, "timeout_msec", timeout_msec, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        active_axes_ = {std::isfinite(inputs[0]), std::isfinite(inputs[1]), std::isfinite(inputs[2])};
        std::array<double, 3> target{UNSPECIFIED_PORT, UNSPECIFIED_PORT, UNSPECIFIED_PORT};
        double turn = 0.0;
        for (std::size_t axis = 0; axis < target.size(); ++axis) {
            if (active_axes_[axis]) {
                target[axis] = node_.attitudeTarget(axis) + inputs[axis];
                turn = std::max(turn, std::fabs(normalizeAngle(target[axis] - node_.measured_att[axis])));
            }
        }
        if (active_axes_[2]) {
            target[2] = normalizeAngle(target[2]);
        }

        node_.commandAttitude(target);
        start_updates_ = node_.control_error_updates;
        deadline_ = moveDeadline(timeout_msec, 0.0, 0.0, turn);
        setpoint_publisher_->publish(attitudeCommand(*clock_, target));
        RCLCPP_INFO(logger_, "Published additive attitude target rpy=(%.3f, %.3f, %.3f)", target[0], target[1],
                    target[2]);
        return checkErrors();
    }

    BT::NodeStatus onRunning() override { return checkErrors(); }

    void onHalted() override { RCLCPP_INFO(logger_, "AddAttSetpoint wait halted."); }

   private:
    bool freshAndWithinTolerance(const std::size_t index) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= ANGLE_TOLERANCE;
    }

    BT::NodeStatus checkErrors() const {
        if (!node_.subAlive()) {
            RCLCPP_WARN(logger_, "AddAttSetpoint failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        if ((!active_axes_[0] || freshAndWithinTolerance(6)) && (!active_axes_[1] || freshAndWithinTolerance(7)) &&
            (!active_axes_[2] || freshAndWithinTolerance(8))) {
            return BT::NodeStatus::SUCCESS;
        }
        if (std::chrono::steady_clock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "AddAttSetpoint timed out; error rpy=(%.3f, %.3f, %.3f).", node_.control_errors[6],
                        node_.control_errors[7], node_.control_errors[8]);
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

void registerAddAttSetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                  SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                  rclcpp::Logger logger) {
    factory.registerBuilder<AddAttSetpointAction>(
        "AddAttSetpoint",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<AddAttSetpointAction>(name, config, node, setpoint_publisher, clock, logger);
        });
}
