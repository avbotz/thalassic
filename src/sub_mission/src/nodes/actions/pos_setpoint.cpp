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

class PosSetpointAction : public BT::StatefulActionNode {
   public:
    PosSetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                      PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                      rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          position_publisher_(position_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("x", UNSPECIFIED_PORT, "World x position setpoint (ENU)"),
                BT::InputPort<double>("y", UNSPECIFIED_PORT, "World y position setpoint (ENU)"),
                BT::InputPort<double>("z", UNSPECIFIED_PORT, "World z position setpoint (ENU, up; underwater < 0)")};
    }

    BT::NodeStatus onStart() override {
        std::array<double, 3> inputs{};
        getInput("x", inputs[0]);
        getInput("y", inputs[1]);
        getInput("z", inputs[2]);
        active_axes_ = {std::isfinite(inputs[0]), std::isfinite(inputs[1]), std::isfinite(inputs[2])};
        std::array<double, 3> target = node_.commanded_pos;

        if (active_axes_[0]) {
            target[0] = inputs[0];
        }
        if (active_axes_[1]) {
            target[1] = inputs[1];
        }
        if (active_axes_[2]) {
            target[2] = inputs[2];
        }
        node_.commanded_pos = target;
        position_publisher_->publish(positionCommand(*clock_, target, false));

        start_updates_ = node_.control_error_updates;

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

        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<bool, 3> active_axes_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
};

}  // namespace

void registerPosSetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                               PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                               rclcpp::Logger logger) {
    factory.registerBuilder<PosSetpointAction>(
        "PosSetpoint",
        [&node, position_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<PosSetpointAction>(name, config, node, position_publisher, clock, logger);
        });
}
