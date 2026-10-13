/*
 * This file creates the node that
 * executes the behavior-tree mission.
 *
 * The node class that is responsible for
 * executing the tree is defined in
 * node.cpp
 */

#include "sub_mission/nodes/mission.hpp"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <memory>
#include <thread>

namespace {

// The node SIGINT and SIGTERM interrupt, once it exists.
std::atomic<MissionNode*> interruptible{nullptr};
static_assert(std::atomic<MissionNode*>::is_always_lock_free, "read in a signal handler");
static_assert(std::atomic<bool>::is_always_lock_free, "MissionNode::interrupt() runs in a signal handler");

void handle_signal(int) {
    if (MissionNode* node = interruptible.load()) {
        node->interrupt();
    }
}

}  // namespace

int main(int argc, char** argv) {
    // Init node. SIGINT and SIGTERM end the run through MissionNode::interrupt
    // instead of shutting rclcpp down, so that the mission can still stop the
    // vehicle: a velocity left running would carry on without it.
    RCLCPP_INFO(rclcpp::get_logger("mission"), "Init node");
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::shared_ptr<MissionNode> node;
    try {
        node = std::make_shared<MissionNode>();
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("mission"), "%s", error.what());
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
    interruptible = node.get();
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // Run the mission behavior tree
    bool succeeded = false;
    try {
        succeeded = node->run();
    } catch (const std::exception& error) {
        RCLCPP_FATAL(node->get_logger(), "%s", error.what());
    }

    // From here a signal ends the process: there is nothing left to stop.
    std::signal(SIGINT, SIG_DFL);
    std::signal(SIGTERM, SIG_DFL);
    interruptible = nullptr;

    // Shut down before joining: unlike executor.cancel(), it also ends a spin()
    // that had not started yet.
    rclcpp::shutdown();
    spin_thread.join();

    return succeeded ? EXIT_SUCCESS : EXIT_FAILURE;
}
