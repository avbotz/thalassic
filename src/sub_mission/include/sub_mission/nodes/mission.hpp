#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sub_control_interfaces/msg/error.hpp"
#include "sub_mission/vision_client.hpp"

using namespace std::chrono_literals;

class MissionNode : public rclcpp::Node {
   public:
    MissionNode();

    bool load_mission();
    void activate();
    void execute();
    void run();

    void kill_callback(const std_msgs::msg::Bool &msg);
    void control_error_callback(const sub_control_interfaces::msg::Error &msg);

    // Name (resources/missions/<name>.xml) or path of the mission tree to run.
    std::string mission;
    std::string role;

    // Logical BT task name -> role-specific sub_vision model task name.
    std::string visionModelTask(const std::string &task) const;
    bool visionTaskMatches(const std::string &reported_task, const std::string &logical_task) const;
    bool subAlive() const { return !killed; }

    // Detections + load_model access for the vision BT nodes (src/nodes/vision.cpp).
    VisionClient &vision() { return *vision_client; }

    bool killed = false;
    std::array<double, 12> control_errors = {};
    std::array<std::uint64_t, 12> control_error_updates = {};
    std::array<double, 3> commanded_pos = {};
    std::array<double, 3> commanded_att = {};
    std::array<double, 3> last_velocity_setpoint = {};
    std::array<double, 3> last_angvel_setpoint = {};

   private:
    // Built by load_mission() so XML mistakes surface at launch, ticked by execute().
    std::optional<BT::Tree> tree;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub;
    rclcpp::Subscription<sub_control_interfaces::msg::Error>::SharedPtr control_error_sub;
    std::unique_ptr<VisionClient> vision_client;
};
