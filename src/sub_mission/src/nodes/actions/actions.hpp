#pragma once

#include "sub_mission/nodes/commands.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

class MissionNode;

void registerPosSetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                               PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                               rclcpp::Logger logger);
void registerVelocitySetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                    PointCmdPublisher::SharedPtr velocity_publisher, rclcpp::Clock::SharedPtr clock,
                                    rclcpp::Logger logger);
void registerAttSetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                               QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
                               rclcpp::Logger logger);
void registerAngularVelocitySetpointAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                           PointCmdPublisher::SharedPtr angular_velocity_publisher,
                                           rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger);
void registerMoveRelativeAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
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
void registerOctagonSurfaceSweepAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                       PointCmdPublisher::SharedPtr position_publisher,
                                       QuaternionCmdPublisher::SharedPtr attitude_publisher,
                                       rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger);
