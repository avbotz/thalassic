#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "marine_acoustic_msgs/msg/dvl.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class DVLOdomRemapper : public rclcpp::Node {
   public:
    DVLOdomRemapper();
    void dvl_callback(const marine_acoustic_msgs::msg::Dvl::SharedPtr msg);

   private:
    rclcpp::Subscription<marine_acoustic_msgs::msg::Dvl>::SharedPtr subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
};

int main(int argc, char* argv[]);
