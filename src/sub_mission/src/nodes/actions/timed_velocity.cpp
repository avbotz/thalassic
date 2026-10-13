#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace {

// A velocity along the heading for a fixed time, then a HOLD on the axes it
// moved: a short open-loop offset such as the slalom strafe onto the opening.
class TimedVelocityAction : public BT::StatefulActionNode {
   public:
    TimedVelocityAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                        SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                        rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        // Unset axes keep doing what they were (e.g. holding depth).
        return {BT::InputPort<double>("x", UNSPECIFIED_PORT, "Forward velocity in meters per second"),
                BT::InputPort<double>("y", UNSPECIFIED_PORT, "Left velocity in meters per second"),
                BT::InputPort<double>("z", UNSPECIFIED_PORT, "Up velocity in meters per second"),
                BT::InputPort<int>("duration_msec", 1000, "How long to move before holding")};
    }

    BT::NodeStatus onStart() override {
        std::array<double, 3> target{};
        int duration_msec = 0;
        if (!readPort(*this, "x", target[0], logger_) || !readPort(*this, "y", target[1], logger_) ||
            !readPort(*this, "z", target[2], logger_) || !readPort(*this, "duration_msec", duration_msec, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        moved_ = {std::isfinite(target[0]) || std::isfinite(target[1]),
                  std::isfinite(target[0]) || std::isfinite(target[1]), std::isfinite(target[2])};
        if (duration_msec <= 0 || (!moved_[0] && !moved_[2])) {
            RCLCPP_ERROR(logger_, "TimedVelocity needs a velocity and a positive duration_msec.");
            return BT::NodeStatus::FAILURE;
        }
        if (!node_.subAlive()) {
            return BT::NodeStatus::FAILURE;
        }

        // What WaitUntilHit measures speed against, for the axes this sets (x
        // and y go together, a NaN one at zero); an axis left alone keeps its own.
        for (std::size_t i = 0; i < target.size(); ++i) {
            if (moved_[i]) {
                node_.last_velocity_setpoint[i] = std::isfinite(target[i]) ? target[i] : 0.0;
            }
        }
        node_.releasePosition(target);
        setpoint_publisher_->publish(linearVelocityCommand(*clock_, target));
        deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(duration_msec);
        RCLCPP_INFO(logger_, "TimedVelocity: xyz=(%.3f, %.3f, %.3f) for %dms.", target[0], target[1], target[2],
                    duration_msec);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!node_.subAlive()) {
            return BT::NodeStatus::FAILURE;
        }
        if (std::chrono::steady_clock::now() < deadline_) {
            return BT::NodeStatus::RUNNING;
        }
        hold();
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override { hold(); }

   private:
    void hold() { node_.hold({moved_[0], moved_[1], moved_[2], false, false, false}); }

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<bool, 3> moved_ = {};
    std::chrono::steady_clock::time_point deadline_;
};

}  // namespace

void registerTimedVelocityAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                 SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                 rclcpp::Logger logger) {
    factory.registerBuilder<TimedVelocityAction>(
        "TimedVelocity",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<TimedVelocityAction>(name, config, node, setpoint_publisher, clock, logger);
        });
}
