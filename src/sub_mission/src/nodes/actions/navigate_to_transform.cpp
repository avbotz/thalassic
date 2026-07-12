#include "actions.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

#include "sub_mission/nodes/mission.hpp"

namespace {

using SteadyClock = std::chrono::steady_clock;

class NavigateToTransformAction : public BT::StatefulActionNode {
   public:
    NavigateToTransformAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                              PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                              rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          position_publisher_(position_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("source_frame", "Frame whose current position is the aim point"),
                BT::InputPort<std::string>("target_frame", "Attached frame to move into the source position"),
                BT::InputPort<int>("timeout_msec", 30000, "Maximum TF lookup and movement time before FAILURE")};
    }

    BT::NodeStatus onStart() override {
        std::string source;
        std::string target;
        getInput("source_frame", source);
        getInput("target_frame", target);
        getInput("timeout_msec", timeout_msec_);
        if (source.empty() || target.empty() || timeout_msec_ <= 0) {
            RCLCPP_ERROR(logger_, "NavigateToTransform requires source_frame, target_frame, and a positive timeout.");
            return BT::NodeStatus::FAILURE;
        }

        base_frame_ = qualifyFrame("base_link");
        source_frame_ = qualifyFrame(source);
        target_frame_ = qualifyFrame(target);
        commanded_ = false;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(timeout_msec_);
        return tickImpl();
    }

    BT::NodeStatus onRunning() override { return tickImpl(); }

    void onHalted() override { RCLCPP_INFO(logger_, "NavigateToTransform halted."); }

   private:
    std::string qualifyFrame(const std::string &frame) const {
        if (frame.find('/') != std::string::npos) {
            return frame.front() == '/' ? frame.substr(1) : frame;
        }
        std::string ns = node_.get_namespace();
        if (!ns.empty() && ns.front() == '/') {
            ns.erase(0, 1);
        }
        return ns.empty() ? frame : ns + "/" + frame;
    }

    BT::NodeStatus tickImpl() {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "NavigateToTransform failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "NavigateToTransform timed out: %s -> %s.", source_frame_.c_str(),
                        target_frame_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (!commanded_ && !commandOffset()) {
            return BT::NodeStatus::RUNNING;
        }
        if ((!active_axes_[0] || freshAndWithinTolerance(0)) && (!active_axes_[1] || freshAndWithinTolerance(1)) &&
            (!active_axes_[2] || freshAndWithinTolerance(2))) {
            RCLCPP_INFO(logger_, "NavigateToTransform applied %s -> %s offset.", source_frame_.c_str(),
                        target_frame_.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    bool commandOffset() {
        try {
            const auto base_to_source =
                node_.tfBuffer().lookupTransform(base_frame_, source_frame_, tf2::TimePointZero);
            const auto base_to_target =
                node_.tfBuffer().lookupTransform(base_frame_, target_frame_, tf2::TimePointZero);

            // Translate the vehicle by source-target so the attached target
            // frame reaches the source frame's position at this instant.
            const double forward = base_to_source.transform.translation.x - base_to_target.transform.translation.x;
            const double right = -(base_to_source.transform.translation.y - base_to_target.transform.translation.y);
            const double up = base_to_source.transform.translation.z - base_to_target.transform.translation.z;
            const double yaw = node_.commanded_att[2];
            const double dx = std::cos(yaw) * forward - std::sin(yaw) * right;
            const double dy = std::sin(yaw) * forward + std::cos(yaw) * right;

            node_.commanded_pos[0] += dx;
            node_.commanded_pos[1] += dy;
            node_.commanded_pos[2] += up;
            active_axes_ = {std::fabs(forward) > 0.0 || std::fabs(right) > 0.0,
                            std::fabs(forward) > 0.0 || std::fabs(right) > 0.0, std::fabs(up) > 0.0};
            start_updates_ = node_.control_error_updates;
            position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos));
            commanded_ = true;
            RCLCPP_INFO(logger_, "NavigateToTransform offset=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f).", forward,
                        right, up, node_.commanded_pos[0], node_.commanded_pos[1], node_.commanded_pos[2]);
            return true;
        } catch (const tf2::TransformException &error) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "NavigateToTransform waiting for TF: %s", error.what());
            return false;
        }
    }

    bool freshAndWithinTolerance(const std::size_t index) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= POSITION_TOLERANCE;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::string base_frame_;
    std::string source_frame_;
    std::string target_frame_;
    std::array<bool, 3> active_axes_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
    SteadyClock::time_point deadline_;
    int timeout_msec_ = 30000;
    bool commanded_ = false;
};

}  // namespace

void registerNavigateToTransformAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                       PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                                       const rclcpp::Logger logger) {
    factory.registerBuilder<NavigateToTransformAction>(
        "NavigateToTransform",
        [&node, position_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<NavigateToTransformAction>(name, config, node, position_publisher, clock, logger);
        });
}
