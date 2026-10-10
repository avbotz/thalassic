#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sub_control_interfaces/msg/error.hpp"
#include "sub_mission/nodes/commands.hpp"
#include "sub_mission/vision_client.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class MissionNode : public rclcpp::Node {
   public:
    MissionNode();

    bool load_mission();
    void activate();
    void execute();
    // False if the mission could not be loaded.
    bool run();

    // Ends the run as soon as the tree is between ticks, stopping the vehicle
    // (SIGINT and SIGTERM, from mission.cpp). Async-signal-safe.
    void interrupt() { interrupted = true; }

    // Registers every custom BT node (movement, vision, actuators) with a factory.
    void registerNodes(BT::BehaviorTreeFactory &factory);

    // HOLD on x, y, z and yaw (commands.hpp holdCommand): the vehicle comes to
    // rest where it is, submerged, and none of those axes holds a target.
    void stop();
    // HOLD on the given axes ([x, y, z, roll, pitch, yaw]) only.
    void hold(const std::array<bool, 6> &axes);

    void kill_callback(const std_msgs::msg::Bool &msg);
    void control_error_callback(const sub_control_interfaces::msg::Error &msg);
    void odometry_callback(const nav_msgs::msg::Odometry &msg);

    // Name (resources/missions/<name>.xml) or path of the mission tree to run.
    std::string mission;
    // Port of the Groot2 live view, which takes the next one too; 0 turns it off.
    unsigned groot_port = 0;
    // SURVEY or SEARCH: the role parameter, until SelectGateRole reads the
    // role off the gate the vehicle passes through.
    std::string role;

    // Logical BT task name -> sub_vision model task name.
    std::string visionModelTask(const std::string &task) const;
    bool visionTaskMatches(const std::string &reported_task, const std::string &logical_task) const;
    bool subAlive() const { return !killed; }

    // Detections + load_model access for the vision BT nodes (src/nodes/vision.cpp).
    VisionClient &vision() { return *vision_client; }

    // The vehicle's frames (config/vehicles/<robot_name>.yaml), for NavigateToTransform.
    tf2_ros::Buffer &tfBuffer() { return *tf_buffer; }
    // `name` as sub_bringup publishes it: <namespace>/<name>, the namespace
    // being the robot name.
    std::string frame(const std::string &name) const;

    // The axes of a position (attitude) command that it sets: those given, not
    // NaN. They no longer move at a velocity.
    void commandPosition(const std::array<double, 3> &target, bool altitude);
    void commandAttitude(const std::array<double, 3> &target);
    // Axes now moving at a velocity, which hold no target from then on.
    void releasePosition(const std::array<double, 3> &velocity);
    void releaseAttitude(const std::array<double, 3> &velocity);

    // Where an axis is meant to be: its target, or where it is if it holds
    // none. z is an altitude above the bottom while commanded_altitude.
    double positionTarget(std::size_t axis) const;
    double attitudeTarget(std::size_t axis) const;

    // The callbacks run on the executor's thread and execute() ticks the tree
    // on another, so the state below is only touched under state_mutex (the
    // tree holds it for each tick). `killed` is atomic for activate(), which
    // polls it outside a tick.
    std::atomic<bool> killed{true};
    std::array<double, 12> control_errors = {};
    std::array<std::uint64_t, 12> control_error_updates = {};
    // The vehicle's pose from odometry/filtered, in odom (ENU, z up; see
    // commands.hpp): position, then roll, pitch and yaw.
    std::array<double, 3> measured_pos = {};
    std::array<double, 3> measured_att = {};
    // The position targets the mission last sent sub_control, in odom; NaN on
    // an axis that holds none (it moves at a velocity, or nothing was sent
    // since the kill switch was released).
    std::array<double, 3> commanded_pos = UNSET;
    std::array<double, 3> commanded_att = UNSET;
    bool commanded_altitude = false;  // commanded_pos[2] is an altitude
    // The linear velocity (along the heading) and rotation rates last sent;
    // zero on an axis holding a target or coming to rest. What WaitUntilHit
    // and Spin measure the vehicle's speed against.
    std::array<double, 3> last_velocity_setpoint = {};
    std::array<double, 3> last_angvel_setpoint = {};

   private:
    static constexpr std::array<double, 3> UNSET = {std::numeric_limits<double>::quiet_NaN(),
                                                    std::numeric_limits<double>::quiet_NaN(),
                                                    std::numeric_limits<double>::quiet_NaN()};

    std::mutex state_mutex;
    // Times the kill switch has gone from released to killed, so that
    // execute() sees a kill shorter than a tick. Guarded by state_mutex.
    std::uint64_t kill_count = 0;
    std::atomic<bool> interrupted{false};

    // One topic for every motion command, so they arrive in order.
    SetpointPublisher::SharedPtr setpoint_publisher;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub;
    rclcpp::Subscription<sub_control_interfaces::msg::Error>::SharedPtr control_error_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;
    std::unique_ptr<VisionClient> vision_client;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;

    // Groot2's live view of the tree, from load_mission() until the process
    // exits; null if groot_port is 0 or was taken. Before the tree, so that it
    // still sees the halt ~Tree does.
    std::unique_ptr<BT::Groot2Publisher> groot_publisher;

    // Built by load_mission() so XML mistakes surface at launch, ticked by
    // execute(). Last, so that it is destroyed first: ~Tree halts any node
    // still running, and those use the members above.
    std::optional<BT::Tree> tree;
};
