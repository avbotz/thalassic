#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

namespace {

// Turns through `yaw` at `rate`, then holds the heading it was meant to reach.
// The turn is a yaw velocity, since a yaw target goes the short way round.
// Within HANDOFF of the end it becomes a target at the final heading, which
// sub_control's trajectory brakes into; stopping the velocity at the end
// instead would overshoot by the braking distance and pull back.
class SpinAction : public BT::StatefulActionNode {
   public:
    SpinAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
               SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("yaw", -4.0 * M_PI, "Yaw distance to spin in radians; positive turns left"),
                BT::InputPort<double>("rate", M_PI, "Yaw rate in radians per second"), moveTimeoutPort()};
    }

    BT::NodeStatus onStart() override {
        int timeout_msec = 0;
        if (!readPort(*this, "yaw", target_, logger_) || !readPort(*this, "rate", rate_, logger_) ||
            !readPort(*this, "timeout_msec", timeout_msec, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        // NaN, as elsewhere, leaves yaw alone.
        if (std::isnan(target_) || std::fabs(target_) <= ANGLE_TOLERANCE) {
            return BT::NodeStatus::SUCCESS;
        }
        rate_ = std::fabs(rate_);
        if (!std::isfinite(target_) || !std::isfinite(rate_) || rate_ <= 0.0) {
            RCLCPP_ERROR(logger_, "Spin needs a finite yaw and a finite, non-zero rate, not yaw=%.3f rate=%.3f.",
                         target_, rate_);
            return BT::NodeStatus::FAILURE;
        }

        accumulated_ = 0.0;
        last_time_ = clock_->now();
        deadline_ = moveDeadline(timeout_msec, 0.0, 0.0, target_);
        // The heading the spin is counted from, before yaw is let go.
        start_yaw_ = node_.attitudeTarget(2);
        if (std::fabs(target_) <= HANDOFF) {
            settle();
            return BT::NodeStatus::RUNNING;
        }
        settling_ = false;
        start_updates_ = node_.control_error_updates;

        // Spin in yaw only; roll and pitch stay level.
        const std::array<double, 3> velocity{UNSPECIFIED_PORT, UNSPECIFIED_PORT, std::copysign(rate_, target_)};
        node_.last_angvel_setpoint[2] = velocity[2];
        node_.releaseAttitude(velocity);
        setpoint_publisher_->publish(angularVelocityCommand(*clock_, velocity));
        RCLCPP_INFO(logger_, "Published spin yaw=%.3f rate=%.3f", target_, velocity[2]);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!node_.subAlive()) {
            onHalted();
            return BT::NodeStatus::FAILURE;
        }

        // The measured rate is per second of ROS time (the simulator's clock in
        // simulation, which falls behind the wall clock when it cannot keep
        // up), so dt is too. The deadline stays on the steady clock.
        const rclcpp::Time now = clock_->now();
        const double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (!settling_) {
            if (node_.control_error_updates[11] > start_updates_[11]) {
                const double measured_rate = node_.last_angvel_setpoint[2] - node_.control_errors[11];
                accumulated_ += measured_rate * dt;
            }
            if (std::fabs(accumulated_) >= std::fabs(target_) - HANDOFF) {
                settle();
            }
        } else if (node_.control_error_updates[8] > start_updates_[8] &&
                   std::fabs(node_.control_errors[8]) <= ANGLE_TOLERANCE) {
            return BT::NodeStatus::SUCCESS;
        }

        if (std::chrono::steady_clock::now() >= deadline_) {
            // Hold the heading it got to, rather than a zero rate that lets it drift.
            RCLCPP_WARN(logger_, "Spin timed out after %.2f of %.2f rad.", accumulated_, target_);
            node_.last_angvel_setpoint[2] = 0.0;
            holdYaw(node_.measured_att[2]);
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    // Mid-turn, stop turning; once settling, keep the final heading.
    void onHalted() override {
        if (!settling_) {
            stopSpin();
        }
    }

   private:
    // Under pi, so the short way to the final heading is still the way the
    // spin turns; over the braking distance at sub_control's yaw limits
    // (0.45 rad on the vehicle, 0.72 in simulation), so it brakes, not stops.
    static constexpr double HANDOFF = M_PI_2;

    void stopSpin() {
        node_.last_angvel_setpoint[2] = 0.0;
        setpoint_publisher_->publish(angularVelocityCommand(*clock_, {UNSPECIFIED_PORT, UNSPECIFIED_PORT, 0.0}));
    }

    void settle() {
        settling_ = true;
        node_.last_angvel_setpoint[2] = 0.0;
        start_updates_ = node_.control_error_updates;
        holdYaw(start_yaw_ + target_);
    }

    void holdYaw(const double yaw) {
        const std::array<double, 3> target{UNSPECIFIED_PORT, UNSPECIFIED_PORT, normalizeAngle(yaw)};
        node_.commandAttitude(target);
        setpoint_publisher_->publish(attitudeCommand(*clock_, target));
    }

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    double target_ = 0.0;
    double rate_ = 0.0;
    double accumulated_ = 0.0;
    double start_yaw_ = 0.0;
    bool settling_ = false;
    std::array<std::uint64_t, 12> start_updates_ = {};
    rclcpp::Time last_time_;
    std::chrono::steady_clock::time_point deadline_;
};

}  // namespace

void registerSpinAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                        SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                        rclcpp::Logger logger) {
    factory.registerBuilder<SpinAction>(
        "Spin", [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<SpinAction>(name, config, node, setpoint_publisher, clock, logger);
        });
}
