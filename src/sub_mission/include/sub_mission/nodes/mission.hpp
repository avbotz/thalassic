#pragma once

#include <chrono>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sub_control_interfaces/msg/error.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

namespace service_client
{
void init_clients();
bool alive();
void write(const std::string &command);
}  // namespace service_client

class MissionNode : public rclcpp::Node
{
public:
    MissionNode();

    void activate();
    void execute();
    void run();

    void kill_callback(const std_msgs::msg::Bool &msg);
    void control_error_callback(const sub_control_interfaces::msg::Error &msg);

    bool POOL_A = false;
    bool POOL_B = false;
    bool POOL_C = false;
    bool POOL_D = false;
    bool HEADS = false;
    bool TAILS = false;
    bool SIM = false;
    bool COIN_FLIP = false;
    bool GATE = false;
    bool BUOY = false;
    bool BINS = false;
    bool TORP = false;
    bool OCTAGON = false;
    bool PRELIM = false;
    bool POOL_TEST = false;
    bool VISION_TEST = false;
    bool PID_TUNING_SEQUENCE = false;

    bool killed = true;
    std::array<double, 12> control_errors = {};
    std::array<std::uint64_t, 12> control_error_updates = {};
    std::array<double, 3> commanded_pos = {};
    std::array<double, 3> commanded_att = {};
    std::array<double, 3> last_velocity_setpoint = {};
    std::array<double, 3> last_angvel_setpoint = {};

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub;
    rclcpp::Subscription<sub_control_interfaces::msg::Error>::SharedPtr control_error_sub;
};
