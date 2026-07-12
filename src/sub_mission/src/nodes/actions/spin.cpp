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

class SpinAction : public BT::StatefulActionNode {
   public:
    SpinAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
               VectorCmdPublisher::SharedPtr angular_velocity_publisher,
               QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
               rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          angular_velocity_publisher_(angular_velocity_publisher),
          attitude_publisher_(attitude_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("yaw", 12.5663706, "Yaw distance to spin in radians (positive = CCW)"),
                BT::InputPort<double>("rate", 3.14159265, "Yaw rate in radians per second")};
    }

    BT::NodeStatus onStart() override {
        getInput("yaw", target_);
        getInput("rate", rate_);
        rate_ = std::fabs(rate_);
        if (std::fabs(target_) <= ANGLE_TOLERANCE || rate_ <= 0.0) {
            return BT::NodeStatus::SUCCESS;
        }

        accumulated_ = 0.0;
        last_time_ = std::chrono::steady_clock::now();
        start_updates_ = node_.control_error_updates[11];

        node_.last_angvel_setpoint = {0.0, 0.0, std::copysign(rate_, target_)};
        angular_velocity_publisher_->publish(angularVelocityCommand(*clock_, node_.last_angvel_setpoint));
        RCLCPP_INFO(logger_, "Published spin yaw=%.3f rate=%.3f", target_, node_.last_angvel_setpoint[2]);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!node_.subAlive()) {
            stopSpin();
            return BT::NodeStatus::FAILURE;
        }

        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - last_time_).count();
        last_time_ = now;

        if (node_.control_error_updates[11] > start_updates_) {
            const double measured_rate = node_.last_angvel_setpoint[2] - node_.control_errors[11];
            accumulated_ += measured_rate * dt;
        }

        if (std::fabs(accumulated_) >= std::fabs(target_)) {
            stopSpin();
            node_.commanded_att[2] = normalizeAngle(node_.commanded_att[2] + target_);
            attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { stopSpin(); }

   private:
    void stopSpin() {
        node_.last_angvel_setpoint = {0.0, 0.0, 0.0};
        angular_velocity_publisher_->publish(angularVelocityCommand(*clock_, node_.last_angvel_setpoint));
    }

    MissionNode &node_;
    VectorCmdPublisher::SharedPtr angular_velocity_publisher_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    double target_ = 0.0;
    double rate_ = 0.0;
    double accumulated_ = 0.0;
    std::uint64_t start_updates_ = 0;
    std::chrono::steady_clock::time_point last_time_;
};

}  // namespace

void registerSpinAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                        PointCmdPublisher::SharedPtr angular_velocity_publisher,
                        QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
                        rclcpp::Logger logger) {
    factory.registerBuilder<SpinAction>("Spin", [&node, angular_velocity_publisher, attitude_publisher, clock, logger](
                                                    const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<SpinAction>(name, config, node, angular_velocity_publisher, attitude_publisher, clock,
                                            logger);
    });
}
