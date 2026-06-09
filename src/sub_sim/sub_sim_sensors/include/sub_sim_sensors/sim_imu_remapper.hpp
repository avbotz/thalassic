#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class SimIMURemapper : public rclcpp::Node {
   public:
    SimIMURemapper();
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

   private:
    std::string robot_name_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char* argv[]);
