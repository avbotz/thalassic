#ifndef SIM_TORPEDO_LAUNCHER_HPP_
#define SIM_TORPEDO_LAUNCHER_HPP_

#include <array>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sub_driver_interfaces/srv/launch_torpedo.hpp>

class SimTorpedoLauncher : public rclcpp::Node {
   public:
    SimTorpedoLauncher();

   private:
    static constexpr size_t NUM_TORPEDOES = 2;

    struct Torpedo {
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_pub;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr glue_client;
        rclcpp::TimerBase::SharedPtr burst_timer;
        bool launched = false;
    };

    void launch_callback(const std::shared_ptr<sub_driver_interfaces::srv::LaunchTorpedo::Request> request,
                         std::shared_ptr<sub_driver_interfaces::srv::LaunchTorpedo::Response> response);

    rclcpp::CallbackGroup::SharedPtr cb_group_;

    rclcpp::Service<sub_driver_interfaces::srv::LaunchTorpedo>::SharedPtr launch_srv_;

    std::array<Torpedo, NUM_TORPEDOES> torpedoes_;

    std::mutex launch_mutex_;
};

#endif  // SIM_TORPEDO_LAUNCHER_HPP_
