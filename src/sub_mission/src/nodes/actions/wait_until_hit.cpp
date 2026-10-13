#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace {

class WaitUntilHitAction : public BT::StatefulActionNode {
   public:
    WaitUntilHitAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node, rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config), node_(node), logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("axis", "x", "Velocity axis to monitor: x, y, or z"),
                BT::InputPort<double>("threshold", -1.0, "Speed ratio threshold; negative selects old defaults"),
                BT::InputPort<int>("timeout_msec", 40000, "Maximum wait time before returning FAILURE")};
    }

    BT::NodeStatus onStart() override {
        std::string axis;
        double threshold = 0.0;
        int timeout_msec = 0;
        if (!readPort(*this, "axis", axis, logger_) || !readPort(*this, "threshold", threshold, logger_) ||
            !readPort(*this, "timeout_msec", timeout_msec, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        if ((axis != "x" && axis != "y" && axis != "z") || !std::isfinite(threshold)) {
            RCLCPP_ERROR(logger_, "WaitUntilHit needs an axis of x, y or z and a finite threshold, not %s and %.3f.",
                         axis.c_str(), threshold);
            return BT::NodeStatus::FAILURE;
        }

        axis_index_ = (axis == "z") ? 2 : (axis == "y") ? 1 : 0;
        threshold_ = threshold >= 0.0 ? threshold : (axis_index_ == 2) ? 0.25 : 0.8;
        deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_msec);
        initial_speed_ = measuredSpeed();
        if (std::fabs(initial_speed_) <= 0.03) {
            RCLCPP_INFO(logger_, "WaitUntilHit initial speed is low; assuming already hit object.");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!node_.subAlive()) {
            return BT::NodeStatus::FAILURE;
        }
        const double current_speed = measuredSpeed();
        // Signed: bouncing back off what it hit counts too.
        if (current_speed / initial_speed_ < threshold_ || std::fabs(current_speed) < 0.01) {
            return BT::NodeStatus::SUCCESS;
        }
        if (std::chrono::steady_clock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "WaitUntilHit timed out. initial=%.3f current=%.3f", initial_speed_, current_speed);
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { RCLCPP_INFO(logger_, "WaitUntilHit halted."); }

   private:
    double measuredSpeed() const {
        const std::size_t error_index = 3 + axis_index_;
        return node_.last_velocity_setpoint[axis_index_] - node_.control_errors[error_index];
    }

    MissionNode &node_;
    rclcpp::Logger logger_;
    std::size_t axis_index_ = 0;
    double threshold_ = 0.8;
    double initial_speed_ = 0.0;
    std::chrono::steady_clock::time_point deadline_;
};

}  // namespace

void registerWaitUntilHitAction(BT::BehaviorTreeFactory &factory, MissionNode &node, rclcpp::Logger logger) {
    factory.registerBuilder<WaitUntilHitAction>(
        "WaitUntilHit", [&node, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<WaitUntilHitAction>(name, config, node, logger);
        });
}
