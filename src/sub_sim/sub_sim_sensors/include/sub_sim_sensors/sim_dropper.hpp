#ifndef SIM_DROPPER_HPP_
#define SIM_DROPPER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sub_driver_interfaces/srv/set_dropper.hpp>

class SimDropper : public rclcpp::Node {
   public:
    explicit SimDropper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void set_dropper_callback(const std::shared_ptr<sub_driver_interfaces::srv::SetDropper::Request> request,
                              std::shared_ptr<sub_driver_interfaces::srv::SetDropper::Response> response);
    void joint_state_callback(const sensor_msgs::msg::JointState& msg);

    rclcpp::Service<sub_driver_interfaces::srv::SetDropper>::SharedPtr set_dropper_srv_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_setpoint_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr glue_client_;

    bool released_ = false;
    bool request_in_flight_ = false;
};

#endif  // SIM_DROPPER_HPP_
