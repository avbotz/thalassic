#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"

#include <memory>
#include <string>

namespace {

class SaveGateHomeAction : public BT::SyncActionNode {
   public:
    SaveGateHomeAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node, rclcpp::Logger logger)
        : BT::SyncActionNode(name, config), node_(node), logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<double>("x", "Saved gate-home x coordinate"),
                BT::OutputPort<double>("y", "Saved gate-home y coordinate")};
    }

    BT::NodeStatus tick() override {
        setOutput("x", node_.commanded_pos[0]);
        setOutput("y", node_.commanded_pos[1]);
        RCLCPP_INFO(logger_, "Saved gate-home coordinate: xy=(%.3f, %.3f)", node_.commanded_pos[0],
                    node_.commanded_pos[1]);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    MissionNode &node_;
    rclcpp::Logger logger_;
};

}  // namespace

void registerSaveGateHomeAction(BT::BehaviorTreeFactory &factory, MissionNode &node, rclcpp::Logger logger) {
    factory.registerBuilder<SaveGateHomeAction>(
        "SaveGateHome", [&node, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<SaveGateHomeAction>(name, config, node, logger);
        });
}
