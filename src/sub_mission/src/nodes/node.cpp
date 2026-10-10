/** @file node.cpp
 *  @brief Ros2 node class to run mission task.
 *
 * Mission, implemented as a ros2 node.
 *
 * Here, we're using it to implement the mission task,
 * where the sub completes the competition tasks in the
 * competition pool (gate, slalom, bins, torpedoes, and octagon).
 *
 * The way this is implemented is basically:
 * - The node loads the mission tree, so XML mistakes surface at launch,
 *   and serves it to Groot2's live view
 * - It waits for the kill switch to be released, then for the motors
 *   - Then it ticks the tree until it finishes, the kill switch is
 *     engaged, or the process is interrupted
 *   - Unless killed, it then stops the vehicle, and the process exits
 *     once Groot2 has had time to show how the run ended
 */

#include "actions/actions.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sub_mission/nodes/commands.hpp"
#include "sub_mission/nodes/mission.hpp"
#include "sub_mission/nodes/vision.hpp"
#include "sub_mission/utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"

namespace {

// How long the Groot2 live view outlasts the run, for Groot2 to fetch the
// tree's final state: it polls, and the view goes when the process exits.
constexpr auto GROOT_LINGER = 2s;

void registerMissionNodes(BT::BehaviorTreeFactory &factory, MissionNode &node, const rclcpp::Logger logger,
                          SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock) {
    factory.registerSimpleCondition("NotKilled", [&node](BT::TreeNode &) {
        return node.subAlive() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
    factory.registerSimpleCondition("SurveyRole", [&node](BT::TreeNode &) {
        return node.role == "SURVEY" ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
    factory.registerSimpleCondition("SearchRole", [&node](BT::TreeNode &) {
        return node.role == "SEARCH" ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });

    registerPosSetpointAction(factory, node, setpoint_publisher, clock, logger);
    registerVelocitySetpointAction(factory, node, setpoint_publisher, clock, logger);
    registerAltitudeSetpointAction(factory, node, setpoint_publisher, clock, logger);
    registerAttSetpointAction(factory, node, setpoint_publisher, clock, logger);
    registerAngularVelocitySetpointAction(factory, node, setpoint_publisher, clock, logger);
    registerMoveRelativeAction(factory, node, setpoint_publisher, clock, logger);
    registerNavigateToTransformAction(factory, node, setpoint_publisher, clock, logger);
    registerTimedVelocityAction(factory, node, setpoint_publisher, clock, logger);
    registerAddAttSetpointAction(factory, node, setpoint_publisher, clock, logger);
    registerSpinAction(factory, node, setpoint_publisher, clock, logger);
    registerWaitUntilHitAction(factory, node, logger);
    registerSavePoseAction(factory, node, logger);
    registerStopAction(factory, node, logger);
    registerAverageAnglesAction(factory, logger);
    registerOctagonSurfaceSweepAction(factory, node, setpoint_publisher, clock, logger);
    registerActuatorActions(factory, node, logger);
}

}  // namespace

MissionNode::MissionNode() : rclcpp::Node("mission") {
    // Deeper than the one state the publisher latches, so that a kill and its
    // release both arrive even if they come while a tick holds state_mutex.
    rclcpp::QoS kill_qos(10);
    kill_qos.transient_local();
    this->kill_sub = this->create_subscription<std_msgs::msg::Bool>(
        "kill_switch", kill_qos, std::bind(&MissionNode::kill_callback, this, std::placeholders::_1));

    this->control_error_sub = this->create_subscription<sub_control_interfaces::msg::Error>(
        "control/error", 10, std::bind(&MissionNode::control_error_callback, this, std::placeholders::_1));
    this->odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10, std::bind(&MissionNode::odometry_callback, this, std::placeholders::_1));
    this->setpoint_publisher = this->create_publisher<SetpointMsg>("motion_setpoint", 10);

    this->vision_client = std::make_unique<VisionClient>(*this);
    // The vehicle's mounts, from /tf_static only: they are all NavigateToTransform
    // needs, and /tf would cost this node's executor every odometry update.
    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener = std::make_unique<tf2_ros::TransformListener>(
        *this->tf_buffer, this, false, tf2_ros::DynamicListenerQoS(), tf2_ros::StaticListenerQoS(),
        tf2_ros::detail::get_default_transform_listener_sub_options(),
        tf2_ros::detail::get_default_transform_listener_static_sub_options(), true);

    // Mission config name (resources/missions/<name>.xml) or file path
    this->declare_parameter<std::string>("mission", "");
    this->get_parameter("mission", this->mission);

    this->declare_parameter<std::string>("role", "SURVEY");
    this->get_parameter("role", this->role);
    if (this->role != "SURVEY" && this->role != "SEARCH") {
        throw std::invalid_argument("sub_mission role must be SURVEY or SEARCH");
    }
    RCLCPP_INFO(this->get_logger(), "Mission role: %s", this->role.c_str());

    // The live view's port; Groot2Publisher binds the next one too.
    const auto groot_port = this->declare_parameter<std::int64_t>("groot_port", 5555);
    if (groot_port < 0 || groot_port > 65534) {
        throw std::invalid_argument("sub_mission groot_port must be 0 (off) or a port up to 65534");
    }
    this->groot_port = static_cast<unsigned>(groot_port);
}

std::string MissionNode::frame(const std::string &name) const {
    const std::string ns = this->get_namespace();
    return ns == "/" ? name : ns.substr(1) + "/" + name;
}

std::string MissionNode::visionModelTask(const std::string &task) const { return task; }

bool MissionNode::visionTaskMatches(const std::string &reported_task, const std::string &logical_task) const {
    return logical_task.empty() || reported_task == logical_task || reported_task == visionModelTask(logical_task);
}

void MissionNode::kill_callback(const std_msgs::msg::Bool &msg) {
    const std::lock_guard<std::mutex> lock(this->state_mutex);
    if (msg.data == this->killed) {
        return;
    }
    if (msg.data) {
        ++this->kill_count;
        RCLCPP_WARN(this->get_logger(), "Kill switch engaged.");
    } else {
        // sub_control re-zeroes the pose and holds where the vehicle is, so
        // nothing the mission sent before still stands.
        this->commanded_pos = UNSET;
        this->commanded_att = UNSET;
        this->commanded_altitude = false;
        this->last_velocity_setpoint = {};
        this->last_angvel_setpoint = {};
        RCLCPP_INFO(this->get_logger(), "Kill switch released.");
    }
    this->killed = msg.data;
}

void MissionNode::odometry_callback(const nav_msgs::msg::Odometry &msg) {
    // ZYX Euler angles, as MotionSetpoint takes them.
    const auto &q = msg.pose.pose.orientation;
    const double roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    const double pitch = std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    const auto &p = msg.pose.pose.position;
    const std::lock_guard<std::mutex> lock(this->state_mutex);
    this->measured_pos = {p.x, p.y, p.z};
    this->measured_att = {roll, pitch, yaw};
}

void MissionNode::commandPosition(const std::array<double, 3> &target, const bool altitude) {
    for (std::size_t axis = 0; axis < target.size(); ++axis) {
        if (std::isfinite(target[axis])) {
            this->commanded_pos[axis] = target[axis];
        }
    }
    // A target ends the velocity the axis had (x and y share a mode).
    if (std::isfinite(target[0]) || std::isfinite(target[1])) {
        this->last_velocity_setpoint[0] = this->last_velocity_setpoint[1] = 0.0;
    }
    if (std::isfinite(target[2])) {
        this->commanded_altitude = altitude;
        this->last_velocity_setpoint[2] = 0.0;
    }
}

void MissionNode::commandAttitude(const std::array<double, 3> &target) {
    for (std::size_t axis = 0; axis < target.size(); ++axis) {
        if (std::isfinite(target[axis])) {
            this->commanded_att[axis] = target[axis];
            this->last_angvel_setpoint[axis] = 0.0;
        }
    }
}

void MissionNode::releasePosition(const std::array<double, 3> &velocity) {
    // x and y share a mode: a velocity on either moves both.
    if (std::isfinite(velocity[0]) || std::isfinite(velocity[1])) {
        this->commanded_pos[0] = this->commanded_pos[1] = UNSET[0];
    }
    if (std::isfinite(velocity[2])) {
        this->commanded_pos[2] = UNSET[2];
        this->commanded_altitude = false;
    }
}

void MissionNode::releaseAttitude(const std::array<double, 3> &velocity) {
    for (std::size_t axis = 0; axis < velocity.size(); ++axis) {
        if (std::isfinite(velocity[axis])) {
            this->commanded_att[axis] = UNSET[axis];
        }
    }
}

double MissionNode::positionTarget(const std::size_t axis) const {
    return std::isfinite(this->commanded_pos[axis]) ? this->commanded_pos[axis] : this->measured_pos[axis];
}

double MissionNode::attitudeTarget(const std::size_t axis) const {
    return std::isfinite(this->commanded_att[axis]) ? this->commanded_att[axis] : this->measured_att[axis];
}

void MissionNode::registerNodes(BT::BehaviorTreeFactory &factory) {
    registerMissionNodes(factory, *this, this->get_logger(), this->setpoint_publisher, this->get_clock());
    registerVisionNodes(factory, *this, this->get_logger(), this->setpoint_publisher, this->get_clock());
}

void MissionNode::stop() {
    // Each axis holds wherever it comes to rest, which the mission does not know.
    this->releasePosition({0.0, 0.0, 0.0});
    this->releaseAttitude({UNSPECIFIED_PORT, UNSPECIFIED_PORT, 0.0});
    this->last_velocity_setpoint = {};
    this->last_angvel_setpoint = {};
    this->setpoint_publisher->publish(holdCommand(*this->get_clock()));
    RCLCPP_INFO(this->get_logger(), "Stop: holding x, y, z and yaw where they come to rest.");
}

void MissionNode::hold(const std::array<bool, 6> &axes) {
    const auto held = [&axes](const std::size_t axis) { return axes[axis] ? 0.0 : UNSPECIFIED_PORT; };
    this->releasePosition({held(0), held(1), held(2)});
    this->releaseAttitude({held(3), held(4), held(5)});
    for (std::size_t axis = 0; axis < 3; ++axis) {
        // x and y share a mode, so holding one holds both.
        if (axes[axis] || (axis < 2 && axes[1 - axis])) {
            this->last_velocity_setpoint[axis] = 0.0;
        }
        if (axes[3 + axis]) {
            this->last_angvel_setpoint[axis] = 0.0;
        }
    }
    this->setpoint_publisher->publish(holdCommand(*this->get_clock(), axes));
}

void MissionNode::control_error_callback(const sub_control_interfaces::msg::Error &msg) {
    const std::array<std::array<double, 3>, 4> errors = {msg.pos_error, msg.vel_error, msg.att_error, msg.angvel_error};

    const std::lock_guard<std::mutex> lock(this->state_mutex);
    for (std::size_t group = 0; group < errors.size(); ++group) {
        for (std::size_t axis = 0; axis < errors[group].size(); ++axis) {
            const std::size_t index = group * errors[group].size() + axis;
            this->control_errors[index] = errors[group][axis];
            ++this->control_error_updates[index];
        }
    }
}

void MissionNode::activate() {
    RCLCPP_INFO(this->get_logger(), "Activating mission_node");

    // Wait for sub to turn on
    RCLCPP_INFO(this->get_logger(), "Wait for unkill");
    while (rclcpp::ok() && !this->interrupted && !this->subAlive()) {
        RCLCPP_INFO(this->get_logger(), "Sub is killed, waiting...");
        std::this_thread::sleep_for(0.5s);
    }
    if (this->interrupted) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Kill switch released");
    RCLCPP_INFO(this->get_logger(), "Wait for motors to start up");
    // In steps, so that an interrupt need not wait it out.
    const auto motors_ready = std::chrono::steady_clock::now() + 7s;
    while (rclcpp::ok() && !this->interrupted && std::chrono::steady_clock::now() < motors_ready) {
        std::this_thread::sleep_for(100ms);
    }
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
        this->registerNodes(factory);

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

    // Up from here, so that Groot2 can connect before the run starts. Optional:
    // a port held by another process must not cost the run.
    if (this->groot_port != 0) {
        try {
            this->groot_publisher = std::make_unique<BT::Groot2Publisher>(*this->tree, this->groot_port);
            RCLCPP_INFO(this->get_logger(), "Groot2 live view on port %u.", this->groot_port);
        } catch (const std::exception &error) {
            RCLCPP_WARN(this->get_logger(), "Groot2 live view unavailable on port %u (%s); running without it.",
                        this->groot_port, error.what());
        }
    }
    return true;
}

void MissionNode::execute() {
    if (!this->tree.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "No mission tree loaded.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Executing mission '%s'", this->mission.c_str());

    {
        // The tree reads and writes the state the callbacks update, so it
        // ticks holding state_mutex and lets go only between ticks.
        std::unique_lock<std::mutex> state(this->state_mutex);
        const std::uint64_t kills = this->kill_count;
        try {
            BT::NodeStatus status = BT::NodeStatus::IDLE;
            while (rclcpp::ok() && !this->interrupted) {
                if (this->killed || this->kill_count != kills) {
                    // sub_control drops every command while killed and re-zeroes
                    // the pose at the release (also after a kill too short to
                    // see here), so nothing the tree is doing still applies. End
                    // the run here instead of failing through every remaining
                    // task; `restart` starts it again from the top.
                    this->tree->haltTree();
                    RCLCPP_WARN(this->get_logger(), "Kill switch engaged: mission '%s' stopped.",
                                this->mission.c_str());
                    return;
                }
                status = this->tree->tickOnce();
                if (status != BT::NodeStatus::RUNNING) {
                    break;
                }
                state.unlock();
                this->tree->sleep(std::chrono::milliseconds(50));
                state.lock();
            }

            if (!rclcpp::ok()) {
                // rclcpp shut down under the tree: nothing left to publish on.
                this->tree->haltTree();
                RCLCPP_INFO(this->get_logger(), "Mission '%s' halted with %s.", this->mission.c_str(),
                            statusName(status));
                return;
            }

            if (status == BT::NodeStatus::RUNNING || status == BT::NodeStatus::IDLE) {
                // interrupt(): SIGINT or SIGTERM, e.g. `restart` stopping us.
                this->tree->haltTree();
                RCLCPP_WARN(this->get_logger(), "Mission '%s' interrupted.", this->mission.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Mission '%s' finished with %s.", this->mission.c_str(),
                            statusName(status));
            }
        } catch (const std::exception &error) {
            RCLCPP_ERROR(this->get_logger(), "Mission failed: %s", error.what());
        }

        // A velocity or spin the tree left running would carry on without it.
        // Hold still instead, submerged: a breach outside the octagon ends a run.
        this->stop();
    }
    // The process exits next, which could drop the stop before sub_control has it.
    this->setpoint_publisher->wait_for_all_acked(1s);
}

bool MissionNode::run() {
    // Loads the mission tree, activates the node, and executes the mission.
    // Loading first surfaces XML mistakes at launch, before the sub is unkilled.
    if (!this->load_mission()) {
        return false;
    }
    this->activate();
    if (this->interrupted) {
        RCLCPP_INFO(this->get_logger(), "Interrupted before mission '%s' started.", this->mission.c_str());
        return true;
    }
    this->execute();

    // The final state shows where the run ended, and with what.
    if (this->groot_publisher) {
        const auto deadline = std::chrono::steady_clock::now() + GROOT_LINGER;
        while (!this->interrupted && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(100ms);
        }
    }
    return true;
}
