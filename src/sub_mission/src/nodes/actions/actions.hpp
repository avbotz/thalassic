#pragma once

#include "sub_mission/nodes/commands.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sub_control_interfaces/action/control_setpoint.hpp"

class MissionNode;

void registerControlSetpointActions(
    BT::BehaviorTreeFactory& factory,
    rclcpp_action::Client<sub_control_interfaces::action::ControlSetpoint>::SharedPtr client, rclcpp::Logger logger);
void registerTimedVelocityAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                 PointCmdPublisher::SharedPtr velocity_publisher, rclcpp::Clock::SharedPtr clock,
                                 rclcpp::Logger logger);
void registerAngularVelocitySetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                           PointCmdPublisher::SharedPtr angular_velocity_publisher,
                                           rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger);
void registerMoveRelativeAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                                rclcpp::Logger logger);
void registerMoveRelativePosAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                   PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                                   rclcpp::Logger logger);
void registerNavigateToTransformAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                       PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                                       rclcpp::Logger logger);
void registerAddAttSetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                  QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
                                  rclcpp::Logger logger);
void registerSpinAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                        SpinCmdPublisher::SharedPtr spin_publisher,
                        QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
                        rclcpp::Logger logger);
void registerWaitUntilHitAction(BT::BehaviorTreeFactory &factory, MissionNode &node, rclcpp::Logger logger);
void registerSaveGateHomeAction(BT::BehaviorTreeFactory &factory, MissionNode &node, rclcpp::Logger logger);
void registerAverageAnglesAction(BT::BehaviorTreeFactory &factory, rclcpp::Logger logger);
void registerActuatorActions(BT::BehaviorTreeFactory &factory, MissionNode &node, rclcpp::Logger logger);
void registerSleepAction(BT::BehaviorTreeFactory &factory, rclcpp::Logger logger);
void registerOctagonSurfaceSweepAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                       PointCmdPublisher::SharedPtr position_publisher,
                                       QuaternionCmdPublisher::SharedPtr attitude_publisher,
                                       rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger);
