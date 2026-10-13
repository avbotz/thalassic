#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "sub_mission/nodes/commands.hpp"

class MissionNode;

// Registers the perception BT leaves (src/nodes/vision.cpp), the generic ones
//   LoadModel          -- swap the active detector via sub_vision's service
//   DetectionVisible   -- condition: a fresh matching detection exists
//   WaitForDetection   -- block until a matching detection arrives, or time out
//   SearchForDetection -- move at a fixed velocity until one arrives
//   AlignToDetection   -- closed-loop yaw/depth centering on a detection
// and the sweeps, approaches and centerings built on them (docs/mission.md).
// Task trees compose these in XML.
void registerVisionNodes(BT::BehaviorTreeFactory &factory, MissionNode &node, rclcpp::Logger logger,
                         SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock);
