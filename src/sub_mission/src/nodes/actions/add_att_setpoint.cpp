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

class AddAttSetpointAction : public BT::StatefulActionNode {
   public:
    AddAttSetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                         QuaternionCmdPublisher::SharedPtr publisher, rclcpp::Clock::SharedPtr clock,
                         rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config), node_(node), publisher_(publisher), clock_(clock), logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("roll", UNSPECIFIED_PORT, "Roll offset in radians (REP-103 FLU)"),
                BT::InputPort<double>("pitch", UNSPECIFIED_PORT, "Pitch offset in radians (REP-103 FLU)"),
                BT::InputPort<double>("yaw", UNSPECIFIED_PORT, "Yaw offset in radians (ENU, CCW-positive)")};
    }

    BT::NodeStatus onStart() override {
        std::array<double, 3> inputs{};
        getInput("roll", inputs[0]);
        getInput("pitch", inputs[1]);
        getInput("yaw", inputs[2]);
        active_axes_ = {std::isfinite(inputs[0]), std::isfinite(inputs[1]), std::isfinite(inputs[2])};
        if (active_axes_[0]) {
            node_.commanded_att[0] += inputs[0];
        }
        if (active_axes_[1]) {
            node_.commanded_att[1] += inputs[1];
        }
        if (active_axes_[2]) {
            node_.commanded_att[2] = normalizeAngle(node_.commanded_att[2] + inputs[2]);
        }

        start_updates_ = node_.control_error_updates;
        publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
        RCLCPP_INFO(logger_, "Published additive attitude target rpy=(%.3f, %.3f, %.3f)", node_.commanded_att[0],
                    node_.commanded_att[1], node_.commanded_att[2]);
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
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    QuaternionCmdPublisher::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<bool, 3> active_axes_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
};

}  // namespace

void registerAddAttSetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                  QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
                                  rclcpp::Logger logger) {
    factory.registerBuilder<AddAttSetpointAction>(
        "AddAttSetpoint",
        [&node, attitude_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<AddAttSetpointAction>(name, config, node, attitude_publisher, clock, logger);
        });
}
