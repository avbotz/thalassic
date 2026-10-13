#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"

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
                           SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                           rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("z", "Altitude above bottom in meters"), moveTimeoutPort()};
    }

    BT::NodeStatus onStart() override {
        double altitude = UNSPECIFIED_PORT;
        int timeout_msec = 0;
        if (!readPort(*this, "z", altitude, logger_) || !readPort(*this, "timeout_msec", timeout_msec, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        // A height above the bottom, so positive. sub_control rejects one below
        // the bottom (an odom z given here by mistake), and control/error would
        // go on describing the old target, which can read as arrival; zero
        // would put the vehicle on the floor.
        if (!std::isfinite(altitude) || altitude <= 0.0) {
            RCLCPP_ERROR(logger_, "AltitudeSetpoint needs a z above the bottom (positive), not %.3f.", altitude);
            return BT::NodeStatus::FAILURE;
        }
        // z only: x, y and the heading carry on.
        const std::array<double, 3> target{UNSPECIFIED_PORT, UNSPECIFIED_PORT, altitude};
        node_.commandPosition(target, true);
        start_updates_ = node_.control_error_updates;
        // The altitude now is not measured here, and sub_control rejects the
        // command outright without bottom lock, so allow for crossing the
        // whole water column of a RoboSub pool (2.1 m).
        deadline_ = moveDeadline(timeout_msec, 0.0, 2.1, 0.0);
        setpoint_publisher_->publish(positionCommand(*clock_, target, true));
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
        if (std::chrono::steady_clock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "AltitudeSetpoint timed out; z error %.3f.", node_.control_errors[2]);
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<std::uint64_t, 12> start_updates_ = {};
    std::chrono::steady_clock::time_point deadline_;
};

}  // namespace

void registerAltitudeSetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                    SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                    rclcpp::Logger logger) {
    factory.registerBuilder<AltitudeSetpointAction>(
        "AltitudeSetpoint",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<AltitudeSetpointAction>(name, config, node, setpoint_publisher, clock, logger);
        });
}
