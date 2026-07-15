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

// Use body-frame velocity for short, deliberate offsets such as the slalom
// red-to-white gap.  Unlike a position target, this never relies on the
// simulator's world-frame convention, so it cannot turn a strafe into an
// unbounded trip toward a pool wall.
class TimedVelocityAction : public BT::StatefulActionNode {
   public:
    TimedVelocityAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                        VectorCmdPublisher::SharedPtr velocity_publisher, rclcpp::Clock::SharedPtr clock,
                        rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          velocity_publisher_(velocity_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("x", 0.0, "Body-forward velocity in meters per second"),
                BT::InputPort<double>("y", 0.0, "Body-left velocity in meters per second"),
                BT::InputPort<double>("z", 0.0, "Body-up velocity in meters per second"),
                BT::InputPort<int>("duration_msec", 1000, "Command duration in milliseconds")};
    }

    BT::NodeStatus onStart() override {
        getInput("x", target_[0]);
        getInput("y", target_[1]);
        getInput("z", target_[2]);
        getInput("duration_msec", duration_msec_);
        if (duration_msec_ <= 0 || !std::isfinite(target_[0]) || !std::isfinite(target_[1]) ||
            !std::isfinite(target_[2])) {
            RCLCPP_ERROR(logger_, "TimedVelocity needs finite velocity values and positive duration_msec.");
            return BT::NodeStatus::FAILURE;
        }
        node_.last_velocity_setpoint = target_;
        velocity_publisher_->publish(linearVelocityCommand(*clock_, target_));
        deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(duration_msec_);
        RCLCPP_INFO(logger_, "TimedVelocity: body xyz=(%.3f, %.3f, %.3f) for %dms.", target_[0], target_[1],
                    target_[2], duration_msec_);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!node_.subAlive()) {
            stop();
            return BT::NodeStatus::FAILURE;
        }
        if (std::chrono::steady_clock::now() < deadline_) {
            return BT::NodeStatus::RUNNING;
        }
        stop();
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override { stop(); }

   private:
    void stop() {
        node_.last_velocity_setpoint = {0.0, 0.0, 0.0};
        velocity_publisher_->publish(linearVelocityCommand(*clock_, node_.last_velocity_setpoint));
    }

    MissionNode &node_;
    VectorCmdPublisher::SharedPtr velocity_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<double, 3> target_{};
    std::chrono::steady_clock::time_point deadline_;
    int duration_msec_ = 1000;
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

void registerTimedVelocityAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                 PointCmdPublisher::SharedPtr velocity_publisher, rclcpp::Clock::SharedPtr clock,
                                 rclcpp::Logger logger) {
    factory.registerBuilder<TimedVelocityAction>(
        "TimedVelocity", [&node, velocity_publisher, clock, logger](const std::string &name,
                                                                       const BT::NodeConfig &config) {
            return std::make_unique<TimedVelocityAction>(name, config, node, velocity_publisher, clock, logger);
        });
}
