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

class VelocitySetpointAction : public BT::SyncActionNode {
   public:
    VelocitySetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                           VectorCmdPublisher::SharedPtr velocity_publisher, rclcpp::Clock::SharedPtr clock,
                           rclcpp::Logger logger)
        : BT::SyncActionNode(name, config),
          node_(node),
          velocity_publisher_(velocity_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("x", 0.0, "Forward velocity in meters per second"),
                BT::InputPort<double>("y", 0.0, "Right velocity in meters per second"),
                BT::InputPort<double>("z", 0.0, "Up velocity in meters per second")};
    }

    BT::NodeStatus tick() override {
        std::array<double, 3> target{};
        getInput("x", target[0]);
        getInput("y", target[1]);
        getInput("z", target[2]);

        node_.last_velocity_setpoint = target;
        velocity_publisher_->publish(linearVelocityCommand(*clock_, target));
        RCLCPP_INFO(logger_, "Published velocity command: xyz=(%.3f, %.3f, %.3f)", target[0], target[1], target[2]);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    MissionNode &node_;
    VectorCmdPublisher::SharedPtr velocity_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
};

}  // namespace

void registerVelocitySetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                    PointCmdPublisher::SharedPtr velocity_publisher, rclcpp::Clock::SharedPtr clock,
                                    rclcpp::Logger logger) {
    factory.registerBuilder<VelocitySetpointAction>(
        "VelocitySetpoint",
        [&node, velocity_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<VelocitySetpointAction>(name, config, node, velocity_publisher, clock, logger);
        });
}
