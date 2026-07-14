#include "actions.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace {

class SleepAction : public BT::StatefulActionNode {
   public:
    SleepAction(const std::string &name, const BT::NodeConfig &config, rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config), logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<int>("msec", 1000, "Duration of the sleep in milliseconds")};
    }

    BT::NodeStatus onStart() override {
        int msec = 1000;
        getInput("msec", msec);
        if (msec <= 0) {
            RCLCPP_WARN(logger_, "Sleep duration must be positive (%d msec given); returning SUCCESS.", msec);
            return BT::NodeStatus::SUCCESS;
        }
        deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(msec);
        RCLCPP_INFO(logger_, "Sleep started: %d msec.", msec);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (std::chrono::steady_clock::now() >= deadline_) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { RCLCPP_INFO(logger_, "Sleep halted."); }

   private:
    rclcpp::Logger logger_;
    std::chrono::steady_clock::time_point deadline_;
};

}  // namespace

void registerSleepAction(BT::BehaviorTreeFactory &factory, rclcpp::Logger logger) {
    factory.registerBuilder<SleepAction>(
        "Sleep", [logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<SleepAction>(name, config, logger);
        });
}
