#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"
#include "sub_mission/utils.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

namespace {

class AttSetpointAction : public BT::StatefulActionNode {
   public:
    AttSetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                      QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
                      rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          attitude_publisher_(attitude_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("roll", UNSPECIFIED_PORT, "Roll setpoint in radians"),
                BT::InputPort<double>("pitch", UNSPECIFIED_PORT, "Pitch setpoint in radians"),
                BT::InputPort<double>("yaw", UNSPECIFIED_PORT, "Yaw setpoint in radians")};
    }

    BT::NodeStatus onStart() override {
        std::array<double, 3> inputs{};
        getInput("roll", inputs[0]);
        getInput("pitch", inputs[1]);
        getInput("yaw", inputs[2]);
        active_axes_ = {std::isfinite(inputs[0]), std::isfinite(inputs[1]), std::isfinite(inputs[2])};
        std::array<double, 3> target = node_.commanded_att;

        if (active_axes_[0]) {
            target[0] = inputs[0];
        }
        if (active_axes_[1]) {
            target[1] = inputs[1];
        }
        if (active_axes_[2]) {
            target[2] = normalizeAngle(inputs[2]);
        }
        node_.commanded_att = target;
        attitude_publisher_->publish(attitudeCommand(*clock_, target));

        start_updates_ = node_.control_error_updates;

        RCLCPP_INFO(logger_, "Published attitude command: rpy=(%.3f, %.3f, %.3f)", target[0], target[1], target[2]);
        return checkErrors();
    }

    BT::NodeStatus onRunning() override { return checkErrors(); }

    void onHalted() override { RCLCPP_INFO(logger_, "AttSetpoint wait halted."); }

   private:
    bool freshAndWithinTolerance(const std::size_t index) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= ANGLE_TOLERANCE;
    }

    BT::NodeStatus checkErrors() const {
        if (!node_.subAlive()) {
            RCLCPP_WARN(logger_, "AttSetpoint wait failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }

        if ((!active_axes_[0] || freshAndWithinTolerance(6)) && (!active_axes_[1] || freshAndWithinTolerance(7)) &&
            (!active_axes_[2] || freshAndWithinTolerance(8))) {
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<bool, 3> active_axes_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
};

}  // namespace

void registerAttSetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                               QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
                               rclcpp::Logger logger) {
    factory.registerBuilder<AttSetpointAction>(
        "AttSetpoint",
        [&node, attitude_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<AttSetpointAction>(name, config, node, attitude_publisher, clock, logger);
        });
}
