#include "marine_acoustic_msgs/msg/dvl.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class DVLOdomRemapper : public rclcpp::Node {
   public:
    DVLOdomRemapper();
    void dvl_callback(const marine_acoustic_msgs::msg::Dvl::SharedPtr msg);

   private:
    std::string robot_name_;
    rclcpp::Subscription<marine_acoustic_msgs::msg::Dvl>::SharedPtr subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr altitude_publisher_;

    nav_msgs::msg::Odometry odom_msg{};
    std_msgs::msg::Float64 alt_msg;
};

int main(int argc, char* argv[]);
