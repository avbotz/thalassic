#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <array>
#include <string>

class ThrusterRepublisher : public rclcpp::Node {
   public:
    ThrusterRepublisher();

   private:
    std::string robot_name_;
    std::array<double, 8> thruster_values_;
    std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, 8> subscribers_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

    double thrust_to_pwm(double thrust);
};

int main(int argc, char* argv[]);
