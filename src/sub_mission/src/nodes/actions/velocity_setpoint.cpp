#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"

#include <array>
#include <cmath>
#include <memory>
#include <string>

namespace {

class VelocitySetpointAction : public BT::SyncActionNode {
   public:
    VelocitySetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                           SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                           rclcpp::Logger logger)
        : BT::SyncActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        // Unset axes keep doing what they were (e.g. holding depth).
        return {BT::InputPort<double>("x", UNSPECIFIED_PORT, "Forward velocity in meters per second"),
                BT::InputPort<double>("y", UNSPECIFIED_PORT, "Left velocity in meters per second"),
                BT::InputPort<double>("z", UNSPECIFIED_PORT, "Up velocity in meters per second")};
    }

    BT::NodeStatus tick() override {
        std::array<double, 3> target{};
        if (!readPort(*this, "x", target[0], logger_) || !readPort(*this, "y", target[1], logger_) ||
            !readPort(*this, "z", target[2], logger_)) {
            return BT::NodeStatus::FAILURE;
        }

        // What WaitUntilHit measures speed against, for the axes this sets (x
        // and y go together, a NaN one at zero); an axis left alone keeps its own.
        const bool horizontal = std::isfinite(target[0]) || std::isfinite(target[1]);
        for (std::size_t i = 0; i < target.size(); ++i) {
            if (std::isfinite(target[i]) || (i < 2 && horizontal)) {
                node_.last_velocity_setpoint[i] = std::isfinite(target[i]) ? target[i] : 0.0;
            }
        }
        node_.releasePosition(target);
        setpoint_publisher_->publish(linearVelocityCommand(*clock_, target));
        RCLCPP_INFO(logger_, "Published velocity command: xyz=(%.3f, %.3f, %.3f)", target[0], target[1], target[2]);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
};

}  // namespace

void registerVelocitySetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                    SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                    rclcpp::Logger logger) {
    factory.registerBuilder<VelocitySetpointAction>(
        "VelocitySetpoint",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<VelocitySetpointAction>(name, config, node, setpoint_publisher, clock, logger);
        });
}
