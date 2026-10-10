#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"

#include <array>
#include <cmath>
#include <memory>
#include <string>

namespace {

class AngularVelocitySetpointAction : public BT::SyncActionNode {
   public:
    AngularVelocitySetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                                  SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                  rclcpp::Logger logger)
        : BT::SyncActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        // Unset axes keep doing what they were (e.g. holding level).
        return {BT::InputPort<double>("roll", UNSPECIFIED_PORT, "Roll rate in radians per second"),
                BT::InputPort<double>("pitch", UNSPECIFIED_PORT, "Pitch rate in radians per second"),
                BT::InputPort<double>("yaw", UNSPECIFIED_PORT, "Yaw rate in radians per second; positive turns left")};
    }

    BT::NodeStatus tick() override {
        std::array<double, 3> target{};
        if (!readPort(*this, "roll", target[0], logger_) || !readPort(*this, "pitch", target[1], logger_) ||
            !readPort(*this, "yaw", target[2], logger_)) {
            return BT::NodeStatus::FAILURE;
        }

        // An axis left alone keeps the rate it had.
        for (std::size_t i = 0; i < target.size(); ++i) {
            if (std::isfinite(target[i])) {
                node_.last_angvel_setpoint[i] = target[i];
            }
        }
        node_.releaseAttitude(target);
        setpoint_publisher_->publish(angularVelocityCommand(*clock_, target));
        RCLCPP_INFO(logger_, "Published angular velocity command: rpy=(%.3f, %.3f, %.3f)", target[0], target[1],
                    target[2]);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
};

}  // namespace

void registerAngularVelocitySetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                           SetpointPublisher::SharedPtr setpoint_publisher,
                                           rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger) {
    factory.registerBuilder<AngularVelocitySetpointAction>(
        "AngularVelocitySetpoint",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<AngularVelocitySetpointAction>(name, config, node, setpoint_publisher, clock,
                                                                   logger);
        });
}
