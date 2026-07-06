#pragma once

#include <string>
#include <vector>

#include "behaviortree_cpp/basic_types.h"

const char *statusName(BT::NodeStatus status);

// Share-directory locations of the mission trees and the task trees.
std::string missionsDirectory();
std::string treesDirectory();

// Turns a mission name ("pool_a") or an explicit file path into a file path.
std::string resolveMissionPath(const std::string &name_or_path);

// Mission names available in missionsDirectory(), sorted.
std::vector<std::string> availableMissions();

// Behavior tree files in treesDirectory(), sorted.
std::vector<std::string> treeFiles();
