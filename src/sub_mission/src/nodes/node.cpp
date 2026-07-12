/** @file mission_node.cpp
 *  @brief Ros2 node class to run mission task.
 *
 * Mission, implemented as a ros2 node.
 *
 * Here, we're using it to implement the mission task,
 * where the sub completes the competition tasks in the
 * competition pool (gate, slalom, bins, torpedoes, and octagon).
 *
 * The way this is implemented is basically:
 * - The node will configure itself (init clients if needed)
 * - Wait for activation signal from master
 *   - On activate, it will start the behavior tree mission
 *   - When all tasks are finished, node will shutdown
 */

#include "actions/actions.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sub_mission/nodes/commands.hpp"
#include "sub_mission/nodes/mission.hpp"
#include "sub_mission/nodes/vision.hpp"
#include "sub_mission/utils.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

namespace {

void registerMissionNodes(BT::BehaviorTreeFactory &factory, MissionNode &node, const rclcpp::Logger logger,
                          PointCmdPublisher::SharedPtr position_publisher,
                          QuaternionCmdPublisher::SharedPtr attitude_publisher,
                          PointCmdPublisher::SharedPtr linear_velocity_publisher,
                          PointCmdPublisher::SharedPtr angular_velocity_publisher,
                          SpinCmdPublisher::SharedPtr spin_publisher, rclcpp::Clock::SharedPtr clock) {
    factory.registerSimpleCondition("NotKilled", [&node](BT::TreeNode &) {
        return node.subAlive() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });

    registerPosSetpointAction(factory, node, position_publisher, clock, logger);
    registerVelocitySetpointAction(factory, node, linear_velocity_publisher, clock, logger);
    registerAttSetpointAction(factory, node, attitude_publisher, clock, logger);
    registerAngularVelocitySetpointAction(factory, node, angular_velocity_publisher, clock, logger);
    registerMoveRelativeAction(factory, node, position_publisher, clock, logger);
    registerNavigateToTransformAction(factory, node, position_publisher, clock, logger);
    registerAddAttSetpointAction(factory, node, attitude_publisher, clock, logger);
    registerSpinAction(factory, node, spin_publisher, attitude_publisher, clock, logger);
    registerWaitUntilHitAction(factory, node, logger);
    registerSaveGateHomeAction(factory, node, logger);
    registerAverageAnglesAction(factory, logger);
    registerActuatorActions(factory, node, logger);
}

}  // namespace

MissionNode::MissionNode() : rclcpp::Node("mission") {
    rclcpp::QoS kill_qos(1);
    kill_qos.transient_local();
    this->kill_sub = this->create_subscription<std_msgs::msg::Bool>(
        "kill_switch", kill_qos, std::bind(&MissionNode::kill_callback, this, std::placeholders::_1));

    this->control_error_sub = this->create_subscription<sub_control_interfaces::msg::Error>(
        "control/error", 10, std::bind(&MissionNode::control_error_callback, this, std::placeholders::_1));

    this->vision_client = std::make_unique<VisionClient>(*this);
    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener = std::make_shared<tf2_ros::TransformListener>(
        *this->tf_buffer, this->get_node_base_interface(), this->get_node_logging_interface(),
        this->get_node_parameters_interface(), this->get_node_topics_interface(), false);

    // Mission config name (resources/missions/<name>.xml) or file path
    this->declare_parameter<std::string>("mission", "");
    this->get_parameter("mission", this->mission);

    this->declare_parameter<std::string>("role", "SURVEY");
    this->get_parameter("role", this->role);
    if (this->role != "SURVEY" && this->role != "SEARCH") {
        throw std::invalid_argument("sub_mission role must be SURVEY or SEARCH");
    }
    RCLCPP_INFO(this->get_logger(), "Mission role: %s", this->role.c_str());
}

std::string MissionNode::visionModelTask(const std::string &task) const {
    return task;
}

bool MissionNode::visionTaskMatches(const std::string &reported_task, const std::string &logical_task) const {
    return logical_task.empty() || reported_task == logical_task || reported_task == visionModelTask(logical_task);
}

void MissionNode::kill_callback(const std_msgs::msg::Bool &msg) {
    this->killed = msg.data;
    if (this->killed) {
        RCLCPP_WARN(this->get_logger(), "Kill switch engaged.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Kill switch released.");
    }
}

void MissionNode::control_error_callback(const sub_control_interfaces::msg::Error &msg) {
    const std::array<std::array<double, 3>, 4> errors = {
        std::array<double, 3>{msg.pos_error[0], -msg.pos_error[1], msg.pos_error[2]},
        std::array<double, 3>{msg.vel_error[0], -msg.vel_error[1], msg.vel_error[2]},
        std::array<double, 3>{msg.att_error[0], -msg.att_error[1], -msg.att_error[2]},
        std::array<double, 3>{msg.angvel_error[0], -msg.angvel_error[1], -msg.angvel_error[2]},
    };

    for (std::size_t group = 0; group < errors.size(); ++group) {
        for (std::size_t axis = 0; axis < errors[group].size(); ++axis) {
            const std::size_t index = group * errors[group].size() + axis;
            this->control_errors[index] = errors[group][axis];
            ++this->control_error_updates[index];
        }
    }

    this->control_spin_active = msg.spin_active;
    if (msg.spin_active) {
        ++this->spin_active_updates;
    }
}

void MissionNode::activate() {
    RCLCPP_INFO(this->get_logger(), "Activating mission_node");

    // Wait for sub to turn on
    RCLCPP_INFO(this->get_logger(), "Wait for unkill");
    while (rclcpp::ok() && !this->subAlive()) {
        RCLCPP_INFO(this->get_logger(), "Sub is killed, waiting...");
        std::this_thread::sleep_for(0.5s);
    }

    RCLCPP_INFO(this->get_logger(), "Kill switch released");
    RCLCPP_INFO(this->get_logger(), "Wait for motors to start up");
    std::this_thread::sleep_for(7s);
}

bool MissionNode::load_mission() {
    std::string available;
    for (const std::string &name : availableMissions()) {
        available += "\n  " + name;
    }

    if (this->mission.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "No mission selected. Relaunch with -p mission:=<name or path>. Available missions:%s",
                     available.c_str());
        return false;
    }

    const std::string mission_path = resolveMissionPath(this->mission);
    if (!std::filesystem::exists(mission_path)) {
        RCLCPP_ERROR(this->get_logger(), "Mission file '%s' does not exist. Available missions:%s",
                     mission_path.c_str(), available.c_str());
        return false;
    }

    try {
        BT::BehaviorTreeFactory factory;
        const auto position_publisher = this->create_publisher<PointCmdMsg>("pos_setpoint", 10);
        const auto attitude_publisher = this->create_publisher<QuaternionCmdMsg>("att_setpoint", 10);
        const auto linear_velocity_publisher = position_publisher;
        const auto angular_velocity_publisher = attitude_publisher;
        const auto spin_publisher = this->create_publisher<SpinCmdMsg>("spin_setpoint", 10);
        registerMissionNodes(factory, *this, this->get_logger(), position_publisher, attitude_publisher,
                             linear_velocity_publisher, angular_velocity_publisher, spin_publisher, this->get_clock());
        registerVisionNodes(factory, *this, this->get_logger(), position_publisher, linear_velocity_publisher,
                            attitude_publisher, this->get_clock());
        for (const std::string &tree_file : treeFiles()) {
            factory.registerBehaviorTreeFromFile(tree_file);
        }

        // The mission file's main_tree_to_execute composes the registered task trees.
        this->tree = factory.createTreeFromFile(mission_path);
        RCLCPP_INFO(this->get_logger(), "Loaded mission from %s", mission_path.c_str());
    } catch (const std::exception &error) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load mission '%s': %s", mission_path.c_str(), error.what());
        return false;
    }
    return true;
}

void MissionNode::execute() {
    if (!this->tree.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "No mission tree loaded.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Executing mission '%s'", this->mission.c_str());

    try {
        BT::Groot2Publisher groot_publish(*this->tree, 5555);
        BT::NodeStatus status = BT::NodeStatus::IDLE;
        while (rclcpp::ok()) {
            status = this->tree->tickOnce();
            if (status != BT::NodeStatus::RUNNING) {
                break;
            }
            this->tree->sleep(std::chrono::milliseconds(50));
        }

        if (status == BT::NodeStatus::RUNNING) {
            this->tree->haltTree();
            RCLCPP_INFO(this->get_logger(), "Mission '%s' halted while RUNNING.", this->mission.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Mission '%s' finished with %s.", this->mission.c_str(), statusName(status));
    } catch (const std::exception &error) {
        RCLCPP_ERROR(this->get_logger(), "Mission failed: %s", error.what());
    }
}

void MissionNode::run() {
    // Loads the mission tree, activates the node, and executes the mission.
    // Loading first surfaces XML mistakes at launch, before the sub is unkilled.
    if (!this->load_mission()) {
        return;
    }
    this->activate();
    this->execute();
}
