#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "sub_mission/nodes/commands.hpp"

class MissionNode;

// Registers the generic perception BT leaves (src/nodes/vision.cpp):
//   LoadModel         -- swap the active detector via sub_vision's service
//   DetectionVisible  -- condition: a fresh matching detection exists
//   WaitForDetection  -- block until a matching detection arrives, or time out
//   AlignToDetection  -- closed-loop yaw/depth centering on a detection
// Task trees compose these in XML; new tasks need no C++.
void registerVisionNodes(BT::BehaviorTreeFactory &factory, MissionNode &node, rclcpp::Logger logger,
                         PointCmdPublisher::SharedPtr position_publisher,
                         PointCmdPublisher::SharedPtr linear_velocity_publisher,
                         QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock);
