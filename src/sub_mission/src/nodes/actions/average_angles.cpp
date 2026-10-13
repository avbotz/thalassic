#include "actions.hpp"

#include "sub_mission/nodes/commands.hpp"

#include <cmath>
#include <memory>
#include <string>

namespace {

class AverageAnglesAction : public BT::SyncActionNode {
   public:
    AverageAnglesAction(const std::string &name, const BT::NodeConfig &config, rclcpp::Logger logger)
        : BT::SyncActionNode(name, config), logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("a", "First angle in radians"),
                BT::InputPort<double>("b", "Second angle in radians"),
                BT::OutputPort<double>("out", "Circular mean of a and b in radians")};
    }

    BT::NodeStatus tick() override {
        double a = 0.0;
        double b = 0.0;
        if (!readPort(*this, "a", a, logger_) || !readPort(*this, "b", b, logger_)) {
            return BT::NodeStatus::FAILURE;
        }

        const double x = std::cos(a) + std::cos(b);
        const double y = std::sin(a) + std::sin(b);
        // Opposite angles (or a NaN) have no mean: atan2 would make one up.
        if (!(std::hypot(x, y) > 1e-9)) {
            RCLCPP_ERROR(logger_, "AverageAngles: a=%.3f and b=%.3f have no mean.", a, b);
            return BT::NodeStatus::FAILURE;
        }
        const double out = normalizeAngle(std::atan2(y, x));
        if (const auto written = setOutput("out", out); !written) {
            RCLCPP_ERROR(logger_, "AverageAngles: output 'out': %s", written.error().c_str());
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(logger_, "AverageAngles: a=%.3f b=%.3f out=%.3f", a, b, out);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    rclcpp::Logger logger_;
};

}  // namespace

void registerAverageAnglesAction(BT::BehaviorTreeFactory &factory, rclcpp::Logger logger) {
    factory.registerBuilder<AverageAnglesAction>("AverageAngles",
                                                 [logger](const std::string &name, const BT::NodeConfig &config) {
                                                     return std::make_unique<AverageAnglesAction>(name, config, logger);
                                                 });
}
