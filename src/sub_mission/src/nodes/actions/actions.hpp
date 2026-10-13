#pragma once

#include "sub_mission/nodes/commands.hpp"

#include <chrono>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

class MissionNode;

// Reads an input port. False, with the reason logged, when the port's
// blackboard entry is missing or does not parse: the caller fails rather than
// carry on with a default, so a typo cannot quietly become a move to zero.
template <typename T>
bool readPort(const BT::TreeNode &node, const char *name, T &value, const rclcpp::Logger &logger) {
    const BT::Expected<T> result = node.getInput<T>(name);
    if (!result) {
        RCLCPP_ERROR(logger, "%s: input '%s': %s", node.name().c_str(), name, result.error().c_str());
        return false;
    }
    value = result.value();
    return true;
}

// The timeout_msec port of the actions that wait for sub_control to arrive.
inline BT::PortsList::value_type moveTimeoutPort() {
    return BT::InputPort<int>("timeout_msec", 0, "Maximum wait before FAILURE; 0 scales it with the size of the move");
}

// When a move started now gives up: after timeout_msec (the port) if it is
// positive, else after moveTimeout().
inline std::chrono::steady_clock::time_point moveDeadline(const int timeout_msec, const double horizontal,
                                                          const double vertical, const double angle) {
    const auto timeout =
        timeout_msec > 0 ? std::chrono::milliseconds(timeout_msec) : moveTimeout(horizontal, vertical, angle);
    return std::chrono::steady_clock::now() + timeout;
}

void registerPosSetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                               SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                               rclcpp::Logger logger);
void registerVelocitySetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                    SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                    rclcpp::Logger logger);
void registerAltitudeSetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                    SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                    rclcpp::Logger logger);
void registerAttSetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                               SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                               rclcpp::Logger logger);
void registerAngularVelocitySetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                           SetpointPublisher::SharedPtr setpoint_publisher,
                                           rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger);
// MoveRelative and MoveRelativePos.
void registerMoveRelativeAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                rclcpp::Logger logger);
void registerNavigateToTransformAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                       SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                       rclcpp::Logger logger);
void registerTimedVelocityAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                 SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                 rclcpp::Logger logger);
void registerAddAttSetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                  SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                  rclcpp::Logger logger);
void registerSpinAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                        SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                        rclcpp::Logger logger);
void registerWaitUntilHitAction(BT::BehaviorTreeFactory &factory, MissionNode &node, rclcpp::Logger logger);
void registerSavePoseAction(BT::BehaviorTreeFactory &factory, MissionNode &node, rclcpp::Logger logger);
void registerStopAction(BT::BehaviorTreeFactory &factory, MissionNode &node, rclcpp::Logger logger);
void registerAverageAnglesAction(BT::BehaviorTreeFactory &factory, rclcpp::Logger logger);
void registerOctagonSurfaceSweepAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                       SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                       rclcpp::Logger logger);
void registerActuatorActions(BT::BehaviorTreeFactory &factory, MissionNode &node, rclcpp::Logger logger);
