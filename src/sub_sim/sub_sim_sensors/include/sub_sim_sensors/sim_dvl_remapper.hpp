#include "marine_acoustic_msgs/msg/dvl.hpp"
#include "rclcpp/rclcpp.hpp"
#include "stonefish_ros2/msg/dvl.hpp"

using namespace std::chrono_literals;

class SimDVLRemapper : public rclcpp::Node {
   public:
    SimDVLRemapper();
    void dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr msg);

   private:
    std::string robot_name_;
    rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr subscriber_;
    rclcpp::Publisher<marine_acoustic_msgs::msg::Dvl>::SharedPtr vel_publisher_;
};

int main(int argc, char* argv[]);
