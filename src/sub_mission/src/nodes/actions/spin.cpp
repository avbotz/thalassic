#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"
#include "sub_mission/utils.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

namespace {

// Publishes one relative-yaw Spin command and waits for sub_control to finish
// it. Control executes the spin closed-loop against the measured heading (so
// it settles without overshoot) and reports progress through the spin_active
// flag and the remaining angle in control_errors[8].
class SpinAction : public BT::StatefulActionNode {
   public:
    SpinAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
               SpinCmdPublisher::SharedPtr spin_publisher, QuaternionCmdPublisher::SharedPtr attitude_publisher,
               rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          spin_publisher_(spin_publisher),
          attitude_publisher_(attitude_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("yaw", 12.5663706, "Yaw distance to spin in radians (positive = CCW)"),
                BT::InputPort<double>("rate", 3.14159265, "Maximum yaw rate in radians per second")};
    }

    BT::NodeStatus onStart() override {
        double target = 0.0;
        double rate = 0.0;
        getInput("yaw", target);
        getInput("rate", rate);
        if (std::fabs(target) <= ANGLE_TOLERANCE) {
            return BT::NodeStatus::SUCCESS;
        }

        // Control bases its target on the commanded heading, so this stays the
        // authoritative post-spin attitude for later actions.
        node_.commanded_att[2] = normalizeAngle(node_.commanded_att[2] + target);
        start_updates_ = node_.control_error_updates[8];
        start_spin_acks_ = node_.spin_active_updates;
        saw_spin_active_ = false;
        warned_no_ack_ = false;
        start_time_ = std::chrono::steady_clock::now();

        spin_publisher_->publish(spinCommand(*clock_, target, std::fabs(rate)));
        RCLCPP_INFO(logger_, "Published spin yaw=%.3f max rate=%.3f", target, std::fabs(rate));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!node_.subAlive()) {
            RCLCPP_WARN(logger_, "Spin failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }

        // Acknowledgment latches on the count of spin_active=true messages, not
        // the sampled bool: a spin that completes within one control tick shows
        // spin_active true for a single message, which a tick-rate sample of
        // the flag can miss entirely.
        if (node_.spin_active_updates > start_spin_acks_) {
            saw_spin_active_ = true;
        }
        if (saw_spin_active_) {
            // Only trust errors from messages received after the command was
            // published; the last pre-command value (e.g. frozen across a kill,
            // when control stops publishing errors) could be stale.
            const bool fresh = node_.control_error_updates[8] > start_updates_;
            if (fresh && !node_.control_spin_active && std::fabs(node_.control_errors[8]) <= ANGLE_TOLERANCE) {
                return BT::NodeStatus::SUCCESS;
            }
        } else {
            const auto elapsed = std::chrono::steady_clock::now() - start_time_;
            if (elapsed > std::chrono::seconds(5)) {
                RCLCPP_ERROR(logger_, "Spin was never acknowledged by control; is the new sub_control running?");
                return BT::NodeStatus::FAILURE;
            }
            if (!warned_no_ack_ && elapsed > std::chrono::seconds(2)) {
                warned_no_ack_ = true;
                RCLCPP_WARN(logger_, "Spin not yet acknowledged by control.");
            }
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        // Cancels the spin in control; the sub settles at the nearest heading
        // congruent to the final target, so commanded_att stays consistent.
        attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
    }

   private:
    MissionNode &node_;
    SpinCmdPublisher::SharedPtr spin_publisher_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::uint64_t start_updates_ = 0;
    std::uint64_t start_spin_acks_ = 0;
    bool saw_spin_active_ = false;
    bool warned_no_ack_ = false;
    std::chrono::steady_clock::time_point start_time_;
};

}  // namespace

void registerSpinAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                        SpinCmdPublisher::SharedPtr spin_publisher,
                        QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
                        rclcpp::Logger logger) {
    factory.registerBuilder<SpinAction>("Spin", [&node, spin_publisher, attitude_publisher, clock, logger](
                                                    const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<SpinAction>(name, config, node, spin_publisher, attitude_publisher, clock, logger);
    });
}
