#ifndef SIM_GRABBER_HPP_
#define SIM_GRABBER_HPP_

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sub_driver_interfaces/srv/set_grabber.hpp>

class SimGrabber : public rclcpp::Node {
   public:
    explicit SimGrabber(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    using SetGrabber = sub_driver_interfaces::srv::SetGrabber;
    using SetBool = std_srvs::srv::SetBool;

    // Something the scenario lets the grabber pick up (a glue to the robot, inactive).
    struct Graspable {
        rclcpp::Client<SetBool>::SharedPtr glue;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;
        std::optional<tf2::Vector3> position;  // world_ned
    };

    void set_grabber_callback(const std::shared_ptr<rmw_request_id_t>& header,
                              const std::shared_ptr<SetGrabber::Request>& request);
    void command_jaws(bool open);
    // Calls `name`'s glue, then answers the request with `message` if that worked.
    void set_glue(const std::string& name, bool hold, const rmw_request_id_t& header, const std::string& message);
    // The graspable closest to the grasp point and how far it is from it; no name when
    // there is nothing to grab.
    std::pair<std::string, double> closest_graspable() const;
    void discover();
    void respond(rmw_request_id_t header, bool success, const std::string& message);

    rclcpp::Service<SetGrabber>::SharedPtr set_grabber_srv_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_setpoint_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_sub_;
    rclcpp::TimerBase::SharedPtr discovery_timer_;

    std::map<std::string, Graspable> graspables_;
    std::optional<tf2::Transform> vehicle_;  // base_link_ned in world_ned
    std::string held_;
    bool busy_ = false;
};

#endif  // SIM_GRABBER_HPP_
