#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "tf2/exceptions.h"

namespace {

// Moves the vehicle so that target_frame ends up where source_frame is now:
// both are mounted on the vehicle (config/vehicles/<robot_name>.yaml), so this
// is a relative move by source minus target, measured in base_link. E.g. after
// centering the down camera on a bin, down_camera -> dropper_link puts the
// dropper over it. Attitude is left alone.
class NavigateToTransformAction : public BT::StatefulActionNode {
   public:
    NavigateToTransformAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                              SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                              rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("source_frame", "Vehicle frame whose position the target frame takes"),
                BT::InputPort<std::string>("target_frame", "Vehicle frame to move there"),
                BT::InputPort<bool>("horizontal_only", false, "Move in x and y only, keeping z"), moveTimeoutPort()};
    }

    BT::NodeStatus onStart() override {
        std::string source;
        std::string target;
        bool horizontal_only = false;
        int timeout_msec = 0;
        if (!readPort(*this, "source_frame", source, logger_) || !readPort(*this, "target_frame", target, logger_) ||
            !readPort(*this, "horizontal_only", horizontal_only, logger_) ||
            !readPort(*this, "timeout_msec", timeout_msec, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        if (source.empty() || target.empty()) {
            RCLCPP_ERROR(logger_, "NavigateToTransform needs a source_frame and a target_frame.");
            return BT::NodeStatus::FAILURE;
        }

        // base_link is FLU, so the difference is already (forward, left, up).
        std::array<double, 3> offset{};
        try {
            const auto base = node_.frame("base_link");
            const auto to_source =
                node_.tfBuffer().lookupTransform(base, node_.frame(source), tf2::TimePointZero).transform.translation;
            const auto to_target =
                node_.tfBuffer().lookupTransform(base, node_.frame(target), tf2::TimePointZero).transform.translation;
            offset = {to_source.x - to_target.x, to_source.y - to_target.y, to_source.z - to_target.z};
        } catch (const tf2::TransformException &error) {
            RCLCPP_ERROR(logger_, "NavigateToTransform %s -> %s: %s", source.c_str(), target.c_str(), error.what());
            return BT::NodeStatus::FAILURE;
        }
        if (horizontal_only) {
            offset[2] = 0.0;
        }

        const double yaw = node_.attitudeTarget(2);
        active_axes_ = {true, true, !horizontal_only};
        std::array<double, 3> target_pos{
            node_.positionTarget(0) + std::cos(yaw) * offset[0] - std::sin(yaw) * offset[1],
            node_.positionTarget(1) + std::sin(yaw) * offset[0] + std::cos(yaw) * offset[1], UNSPECIFIED_PORT};
        // z is up whether it holds an odom z or an altitude.
        const bool altitude = node_.commanded_altitude;
        if (active_axes_[2]) {
            target_pos[2] = node_.positionTarget(2) + offset[2];
        }
        // An altitude below the bottom: sub_control would reject the whole
        // command, and control/error would go on describing the old target,
        // which can read as arrival.
        if (altitude && active_axes_[2] && target_pos[2] < 0.0) {
            RCLCPP_ERROR(logger_, "NavigateToTransform %s -> %s: altitude target %.3f is below the bottom.",
                         source.c_str(), target.c_str(), target_pos[2]);
            return BT::NodeStatus::FAILURE;
        }
        node_.commandPosition(target_pos, altitude);
        start_updates_ = node_.control_error_updates;
        deadline_ = moveDeadline(timeout_msec, std::hypot(offset[0], offset[1]), offset[2], 0.0);
        setpoint_publisher_->publish(positionCommand(*clock_, target_pos, altitude));
        RCLCPP_INFO(logger_, "NavigateToTransform %s -> %s: offset=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f)",
                    source.c_str(), target.c_str(), offset[0], offset[1], offset[2], target_pos[0], target_pos[1],
                    target_pos[2]);
        return checkErrors();
    }

    BT::NodeStatus onRunning() override { return checkErrors(); }

    void onHalted() override { RCLCPP_INFO(logger_, "NavigateToTransform wait halted."); }

   private:
    bool freshAndWithinTolerance(const std::size_t index) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= POSITION_TOLERANCE;
    }

    BT::NodeStatus checkErrors() const {
        if (!node_.subAlive()) {
            RCLCPP_WARN(logger_, "NavigateToTransform failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        if (freshAndWithinTolerance(0) && freshAndWithinTolerance(1) &&
            (!active_axes_[2] || freshAndWithinTolerance(2))) {
            return BT::NodeStatus::SUCCESS;
        }
        if (std::chrono::steady_clock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "NavigateToTransform timed out; error xyz=(%.3f, %.3f, %.3f).",
                        node_.control_errors[0], node_.control_errors[1], node_.control_errors[2]);
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<bool, 3> active_axes_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
    std::chrono::steady_clock::time_point deadline_;
};

}  // namespace

void registerNavigateToTransformAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                       SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                       rclcpp::Logger logger) {
    factory.registerBuilder<NavigateToTransformAction>(
        "NavigateToTransform",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<NavigateToTransformAction>(name, config, node, setpoint_publisher, clock, logger);
        });
}
