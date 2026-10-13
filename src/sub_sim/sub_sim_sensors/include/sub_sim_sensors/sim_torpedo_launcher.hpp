#ifndef SIM_TORPEDO_LAUNCHER_HPP_
#define SIM_TORPEDO_LAUNCHER_HPP_

#include <array>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sub_driver_interfaces/srv/launch_torpedo.hpp>

class SimTorpedoLauncher : public rclcpp::Node {
   public:
    explicit SimTorpedoLauncher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    using LaunchTorpedo = sub_driver_interfaces::srv::LaunchTorpedo;

    static constexpr size_t NUM_TORPEDOES = 2;

    struct Torpedo {
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_pub;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr glue_client;
        rclcpp::TimerBase::SharedPtr burst_timer;
        bool releasing = false;
        bool launched = false;
    };

    void launch_callback(const std::shared_ptr<rmw_request_id_t>& header,
                         const std::shared_ptr<LaunchTorpedo::Request>& request);
    // Pushes a released torpedo out of its tube.
    void push(size_t id);
    void respond(rmw_request_id_t header, bool success, const std::string& message);

    rclcpp::Service<LaunchTorpedo>::SharedPtr launch_srv_;
    std::array<Torpedo, NUM_TORPEDOES> torpedoes_;
};

#endif  // SIM_TORPEDO_LAUNCHER_HPP_
