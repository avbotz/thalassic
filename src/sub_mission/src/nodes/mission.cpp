/*
 * This file creates the node that
 * executes the behavior-tree mission.
 *
 * The node class that is responsible for
 * executing the tree is defined in
 * node.cpp
 */

#include "sub_mission/nodes/mission.hpp"

int main(int argc, char** argv) {
    // Init node
    RCLCPP_INFO(rclcpp::get_logger("mission"), "Init node");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // Run the mission behavior tree
    node->run();

    executor.cancel();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    return 0;
}
