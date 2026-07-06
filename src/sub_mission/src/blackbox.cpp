#include <string>

#include "rclcpp/rclcpp.hpp"

namespace service_client
{

void init_clients()
{
    RCLCPP_INFO(rclcpp::get_logger("mission"), "Blackbox service clients initialized.");
}

bool alive()
{
    return true;
}

void write(const std::string &command)
{
    RCLCPP_INFO(
        rclcpp::get_logger("mission"),
        "Blackbox service write: %s",
        command.c_str());
}

}  // namespace service_client
