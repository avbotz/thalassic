#include "actions.hpp"

#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "sub_control_interfaces/action/control_setpoint.hpp"

namespace {

using ControlSetpoint = sub_control_interfaces::action::ControlSetpoint;
using GoalHandleControlSetpoint = rclcpp_action::ClientGoalHandle<ControlSetpoint>;

class ControlSetpointAction : public BT::StatefulActionNode {
   public:
    ControlSetpointAction(const std::string& name, const BT::NodeConfig& config,
                          rclcpp_action::Client<ControlSetpoint>::SharedPtr client, uint8_t command_type,
                          BT::PortsList ports, rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config), client_(std::move(client)), command_type_(command_type), ports_(std::move(ports)), logger_(logger) {}

    BT::NodeStatus onStart() override {
        if (!client_->wait_for_action_server(std::chrono::seconds(0))) {
            RCLCPP_ERROR(logger_, "ControlSetpoint action server is unavailable.");
            return BT::NodeStatus::FAILURE;
        }

        ControlSetpoint::Goal goal;
        goal.command_type = command_type_;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        goal.setpoint.setpoint.x = nan;
        goal.setpoint.setpoint.y = nan;
        goal.setpoint.setpoint.z = nan;
        goal.setpoint.setpoint.roll = nan;
        goal.setpoint.setpoint.pitch = nan;
        goal.setpoint.setpoint.yaw = nan;
        if (command_type_ == ControlSetpoint::Goal::ATTITUDE) {
            getInput("roll", goal.setpoint.setpoint.roll);
            getInput("pitch", goal.setpoint.setpoint.pitch);
            getInput("yaw", goal.setpoint.setpoint.yaw);
        } else {
            getInput("x", goal.setpoint.setpoint.x);
            getInput("y", goal.setpoint.setpoint.y);
            getInput("z", goal.setpoint.setpoint.z);
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            terminal_status_.reset();
            goal_handle_.reset();
        }
        typename rclcpp_action::Client<ControlSetpoint>::SendGoalOptions options;
        options.goal_response_callback = [this](GoalHandleControlSetpoint::SharedPtr handle) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            goal_handle_ = std::move(handle);
            if (!goal_handle_) terminal_status_ = BT::NodeStatus::FAILURE;
        };
        options.result_callback = [this](const GoalHandleControlSetpoint::WrappedResult& result) {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                terminal_status_ = result.code == rclcpp_action::ResultCode::SUCCEEDED ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            }
            if (result.result) RCLCPP_INFO(logger_, "ControlSetpoint finished: %s", result.result->message.c_str());
        };
        client_->async_send_goal(goal, options);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return terminal_status_.value_or(BT::NodeStatus::RUNNING);
    }

    void onHalted() override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (goal_handle_) client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
        terminal_status_.reset();
    }

   private:
    rclcpp_action::Client<ControlSetpoint>::SharedPtr client_;
    uint8_t command_type_;
    BT::PortsList ports_;
    rclcpp::Logger logger_;
    GoalHandleControlSetpoint::SharedPtr goal_handle_;
    std::optional<BT::NodeStatus> terminal_status_;
    std::mutex state_mutex_;
};

BT::PortsList positionPorts() {
    return {BT::InputPort<double>("x", std::numeric_limits<double>::quiet_NaN(), "North/forward position setpoint"),
            BT::InputPort<double>("y", std::numeric_limits<double>::quiet_NaN(), "East/right position setpoint"),
            BT::InputPort<double>("z", std::numeric_limits<double>::quiet_NaN(), "Vertical position setpoint; negative is down")};
}

BT::PortsList velocityPorts() {
    return {BT::InputPort<double>("x", 0.0, "Forward velocity in meters per second"),
            BT::InputPort<double>("y", 0.0, "Right velocity in meters per second"),
            BT::InputPort<double>("z", 0.0, "Up velocity in meters per second")};
}

BT::PortsList attitudePorts() {
    return {BT::InputPort<double>("roll", std::numeric_limits<double>::quiet_NaN(), "Roll setpoint in radians (REP-103 FLU)"),
            BT::InputPort<double>("pitch", std::numeric_limits<double>::quiet_NaN(), "Pitch setpoint in radians (REP-103 FLU)"),
            BT::InputPort<double>("yaw", std::numeric_limits<double>::quiet_NaN(), "Yaw setpoint in radians (ENU, CCW-positive)")};
}

}  // namespace

void registerControlSetpointActions(BT::BehaviorTreeFactory& factory,
                                    rclcpp_action::Client<ControlSetpoint>::SharedPtr client, rclcpp::Logger logger) {
    const auto register_action = [&factory, client, logger](const char* id, uint8_t type, BT::PortsList ports) {
        const BT::TreeNodeManifest manifest{BT::NodeType::ACTION, id, ports, {}};
        factory.registerBuilder(
            manifest, [client, type, ports = std::move(ports), logger](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<ControlSetpointAction>(name, config, client, type, ports, logger);
            });
    };
    register_action("PosSetpoint", ControlSetpoint::Goal::POSITION, positionPorts());
    register_action("VelocitySetpoint", ControlSetpoint::Goal::VELOCITY, velocityPorts());
    register_action("AttSetpoint", ControlSetpoint::Goal::ATTITUDE, attitudePorts());
}
