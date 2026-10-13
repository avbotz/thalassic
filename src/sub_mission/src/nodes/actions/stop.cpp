#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"

#include <memory>
#include <string>

namespace {

// Brings x, y, z and yaw to rest where they are (MissionNode::stop). Ends a
// velocity or spin an earlier node left running, e.g. before a transit after
// a task that failed part way.
class StopAction : public BT::SyncActionNode {
   public:
    StopAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node)
        : BT::SyncActionNode(name, config), node_(node) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        node_.stop();
        return BT::NodeStatus::SUCCESS;
    }

   private:
    MissionNode &node_;
};

}  // namespace

void registerStopAction(BT::BehaviorTreeFactory &factory, MissionNode &node, rclcpp::Logger) {
    factory.registerBuilder<StopAction>("Stop", [&node](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<StopAction>(name, config, node);
    });
}
