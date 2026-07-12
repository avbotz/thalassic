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

class AltitudeSetpointAction : public BT::StatefulActionNode {
   public:
    AltitudeSetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                           PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                           rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          position_publisher_(position_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() { return {BT::InputPort<double>("z", "Altitude above bottom in meters")}; }

    BT::NodeStatus onStart() override {
        // Altitude is positive-up above the bottom, not an ENU world z, so it
        // must not persist into commanded_pos[2]: a later absolute position
        // command would replay it as a world z above the surface. Track the
        // measured z instead so the next non-altitude command holds depth.
        double altitude = 0.0;
        getInput("z", altitude);
        std::array<double, 3> target = node_.commanded_pos;
        target[2] = altitude;
        node_.commanded_pos[2] -= node_.control_errors[2];
        start_updates_ = node_.control_error_updates;
        position_publisher_->publish(positionCommand(*clock_, target, true));
        RCLCPP_INFO(logger_, "Published altitude command: z=%.3f", altitude);
        return checkErrors();
    }

    BT::NodeStatus onRunning() override { return checkErrors(); }

    void onHalted() override { RCLCPP_INFO(logger_, "AltitudeSetpoint wait halted."); }

   private:
    BT::NodeStatus checkErrors() const {
        if (!node_.subAlive()) {
            RCLCPP_WARN(logger_, "AltitudeSetpoint wait failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        if (node_.control_error_updates[2] > start_updates_[2] &&
            std::fabs(node_.control_errors[2]) <= POSITION_TOLERANCE) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<std::uint64_t, 12> start_updates_ = {};
};

}  // namespace

void registerAltitudeSetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                    PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                                    rclcpp::Logger logger) {
    factory.registerBuilder<AltitudeSetpointAction>(
        "AltitudeSetpoint",
        [&node, position_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<AltitudeSetpointAction>(name, config, node, position_publisher, clock, logger);
        });
}
