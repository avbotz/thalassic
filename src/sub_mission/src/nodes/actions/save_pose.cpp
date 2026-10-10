#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"

#include <memory>
#include <string>

namespace {

// Writes where the vehicle is meant to be (its targets, or where it is on an
// axis that holds none) to the blackboard, e.g. the gate home for Return Home
// or a task's starting heading to recover if the task fails. Ports left
// unconnected are skipped.
class SavePoseAction : public BT::SyncActionNode {
   public:
    SavePoseAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node, rclcpp::Logger logger)
        : BT::SyncActionNode(name, config), node_(node), logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<double>("x", "Odom x in meters"), BT::OutputPort<double>("y", "Odom y in meters"),
                BT::OutputPort<double>("yaw", "Yaw in radians (odom, counter-clockwise)")};
    }

    BT::NodeStatus tick() override {
        const double x = node_.positionTarget(0);
        const double y = node_.positionTarget(1);
        const double yaw = node_.attitudeTarget(2);
        setOutput("x", x);
        setOutput("y", y);
        setOutput("yaw", yaw);
        RCLCPP_INFO(logger_, "%s: saved xy=(%.3f, %.3f) yaw=%.3f", name().c_str(), x, y, yaw);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    MissionNode &node_;
    rclcpp::Logger logger_;
};

}  // namespace

void registerSavePoseAction(BT::BehaviorTreeFactory &factory, MissionNode &node, rclcpp::Logger logger) {
    factory.registerBuilder<SavePoseAction>("SavePose",
                                            [&node, logger](const std::string &name, const BT::NodeConfig &config) {
                                                return std::make_unique<SavePoseAction>(name, config, node, logger);
                                            });
}
