#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"
#include "sub_mission/utils.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "tf2/exceptions.h"

namespace {

class MoveRelativeAction : public BT::StatefulActionNode {
   public:
    MoveRelativeAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                       PointCmdPublisher::SharedPtr publisher, rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config), node_(node), publisher_(publisher), clock_(clock), logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("x", 0.0, "Forward offset in meters"),
                BT::InputPort<double>("y", 0.0, "Right offset in meters"),
                BT::InputPort<double>("z", 0.0, "Vertical offset in meters; negative is down"),
                BT::InputPort<double>("min_travel_m", 0.0,
                                      "Require this much measured horizontal odom travel before succeeding")};
    }

    BT::NodeStatus onStart() override {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double min_travel_m = 0.0;
        getInput("x", x);
        getInput("y", y);
        getInput("z", z);
        getInput("min_travel_m", min_travel_m);
        if (std::isnan(x))
            x = 0.0;
        if (std::isnan(y))
            y = 0.0;
        if (std::isnan(z))
            z = 0.0;
        if (!std::isfinite(min_travel_m) || min_travel_m < 0.0) {
            RCLCPP_ERROR(logger_, "MoveRelative needs a non-negative min_travel_m.");
            return BT::NodeStatus::FAILURE;
        }

        initial_position_ = measuredPosition();
        min_travel_m_ = min_travel_m;

        const double yaw = node_.commanded_att[2];
        const double dx = std::cos(yaw) * x - std::sin(yaw) * y;
        const double dy = std::sin(yaw) * x + std::cos(yaw) * y;

        node_.commanded_pos[0] += dx;
        node_.commanded_pos[1] += dy;
        node_.commanded_pos[2] += z;

        active_axes_ = {std::fabs(x) > 0.0 || std::fabs(y) > 0.0, std::fabs(x) > 0.0 || std::fabs(y) > 0.0,
                        std::fabs(z) > 0.0};
        start_updates_ = node_.control_error_updates;
        publisher_->publish(positionCommand(*clock_, node_.commanded_pos));

        RCLCPP_INFO(logger_, "Published relative move: offset=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f)", x, y, z,
                    node_.commanded_pos[0], node_.commanded_pos[1], node_.commanded_pos[2]);
        return checkErrors();
    }

    BT::NodeStatus onRunning() override { return checkErrors(); }

    void onHalted() override { RCLCPP_INFO(logger_, "MoveRelative wait halted."); }

   private:
    bool freshAndWithinTolerance(const std::size_t index) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= POSITION_TOLERANCE;
    }

    std::optional<std::array<double, 3>> measuredPosition() const {
        const std::string ns = node_.get_namespace();
        const std::string prefix = ns.empty() || ns == "/" ? "" : (ns.front() == '/' ? ns.substr(1) : ns) + "/";
        try {
            const auto transform =
                node_.tfBuffer().lookupTransform(prefix + "odom", prefix + "base_link", tf2::TimePointZero);
            return std::array<double, 3>{transform.transform.translation.x, transform.transform.translation.y,
                                         transform.transform.translation.z};
        } catch (const tf2::TransformException &error) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "MoveRelative waiting for odom TF: %s", error.what());
            return std::nullopt;
        }
    }

    bool travelledEnough() const {
        if (min_travel_m_ == 0.0) {
            return true;
        }
        if (!initial_position_) {
            return false;
        }
        const auto current = measuredPosition();
        if (!current) {
            return false;
        }
        return std::hypot((*current)[0] - (*initial_position_)[0], (*current)[1] - (*initial_position_)[1]) >=
               min_travel_m_;
    }

    BT::NodeStatus checkErrors() const {
        if (!node_.subAlive()) {
            RCLCPP_WARN(logger_, "MoveRelative failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        if (travelledEnough() && (!active_axes_[0] || freshAndWithinTolerance(0)) &&
            (!active_axes_[1] || freshAndWithinTolerance(1)) &&
            (!active_axes_[2] || freshAndWithinTolerance(2))) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<bool, 3> active_axes_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
    std::optional<std::array<double, 3>> initial_position_;
    double min_travel_m_ = 0.0;
};

}  // namespace

void registerMoveRelativeAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                                rclcpp::Logger logger) {
    factory.registerBuilder<MoveRelativeAction>(
        "MoveRelative",
        [&node, position_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<MoveRelativeAction>(name, config, node, position_publisher, clock, logger);
        });
}
