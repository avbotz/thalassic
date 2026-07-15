#include "sub_control/sub_control_ff.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubControlFF>());
  rclcpp::shutdown();
  return 0;
}
