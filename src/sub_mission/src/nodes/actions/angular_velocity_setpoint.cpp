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

class AngularVelocitySetpointAction : public BT::SyncActionNode {
   public:
    AngularVelocitySetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                                  VectorCmdPublisher::SharedPtr angular_velocity_publisher,
                                  rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger)
        : BT::SyncActionNode(name, config),
          node_(node),
          angular_velocity_publisher_(angular_velocity_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("roll", 0.0, "Roll rate in radians per second (REP-103 FLU)"),
                BT::InputPort<double>("pitch", 0.0, "Pitch rate in radians per second (REP-103 FLU)"),
                BT::InputPort<double>("yaw", 0.0, "Yaw rate in radians per second (CCW-positive)")};
    }

    BT::NodeStatus tick() override {
        std::array<double, 3> target{};
        getInput("roll", target[0]);
        getInput("pitch", target[1]);
        getInput("yaw", target[2]);

        node_.last_angvel_setpoint = target;
        angular_velocity_publisher_->publish(angularVelocityCommand(*clock_, target));
        RCLCPP_INFO(logger_, "Published angular velocity command: rpy=(%.3f, %.3f, %.3f)", target[0], target[1],
                    target[2]);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    MissionNode &node_;
    VectorCmdPublisher::SharedPtr angular_velocity_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
};

}  // namespace

void registerAngularVelocitySetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                           PointCmdPublisher::SharedPtr angular_velocity_publisher,
                                           rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger) {
    factory.registerBuilder<AngularVelocitySetpointAction>(
        "AngularVelocitySetpoint",
        [&node, angular_velocity_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<AngularVelocitySetpointAction>(name, config, node, angular_velocity_publisher,
                                                                   clock, logger);
        });
}
