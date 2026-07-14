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

class MoveRelativePosAction : public BT::StatefulActionNode {
   public:
    MoveRelativePosAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                          PointCmdPublisher::SharedPtr publisher, rclcpp::Clock::SharedPtr clock,
                          rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config), node_(node), publisher_(publisher), clock_(clock), logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("x", 0.0, "Global east offset in meters"),
                BT::InputPort<double>("y", 0.0, "Global north offset in meters"),
                BT::InputPort<double>("z", 0.0, "Global vertical offset in meters; negative is down")};
    }

    BT::NodeStatus onStart() override {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        getInput("x", x);
        getInput("y", y);
        getInput("z", z);
        if (std::isnan(x))
            x = 0.0;
        if (std::isnan(y))
            y = 0.0;
        if (std::isnan(z))
            z = 0.0;

        node_.commanded_pos[0] += x;
        node_.commanded_pos[1] += y;
        node_.commanded_pos[2] += z;

        active_axes_ = {std::fabs(x) > 0.0, std::fabs(y) > 0.0, std::fabs(z) > 0.0};
        start_updates_ = node_.control_error_updates;
        publisher_->publish(positionCommand(*clock_, node_.commanded_pos));

        RCLCPP_INFO(logger_, "Published global relative move: offset=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f)",
                    x, y, z, node_.commanded_pos[0], node_.commanded_pos[1], node_.commanded_pos[2]);
        return checkErrors();
    }

    BT::NodeStatus onRunning() override { return checkErrors(); }

    void onHalted() override { RCLCPP_INFO(logger_, "MoveRelativePos wait halted."); }

   private:
    bool freshAndWithinTolerance(const std::size_t index) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= POSITION_TOLERANCE;
    }

    BT::NodeStatus checkErrors() const {
        if (!node_.subAlive()) {
            RCLCPP_WARN(logger_, "MoveRelativePos failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        if ((!active_axes_[0] || freshAndWithinTolerance(0)) && (!active_axes_[1] || freshAndWithinTolerance(1)) &&
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
};

}  // namespace

void registerMoveRelativePosAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                   PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                                   rclcpp::Logger logger) {
    factory.registerBuilder<MoveRelativePosAction>(
        "MoveRelativePos",
        [&node, position_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<MoveRelativePosAction>(name, config, node, position_publisher, clock, logger);
        });
}
