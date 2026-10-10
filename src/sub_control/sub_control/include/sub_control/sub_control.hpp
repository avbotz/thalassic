#ifndef SUB_CONTROL_SUB_CONTROL_HPP_
#define SUB_CONTROL_SUB_CONTROL_HPP_

#include "sub_control/allocator.hpp"
#include "sub_control/controller.hpp"
#include "sub_control/reference_generator.hpp"
#include "sub_control/utils.hpp"
#include "sub_control_interfaces/msg/controller_status.hpp"
#include "sub_control_interfaces/msg/error.hpp"
#include "sub_control_interfaces/msg/motion_setpoint.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_localization/srv/set_pose.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// The sub_control node: trajectory generator -> control law -> thrust
// allocation, plus the watchdogs that decide which axes still have feedback.
// See docs/control.md.
class SubControl : public rclcpp::Node {
   public:
    SubControl();

   private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void altitude_callback(const sensor_msgs::msg::Range::SharedPtr msg);
    void kill_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void thruster_health_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    void motion_setpoint_callback(const sub_control_interfaces::msg::MotionSetpoint::SharedPtr msg);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void run();

    void apply_command(const MotionCommand& command, const std::string& source);
    // Asks the EKF to zero x, y and yaw. The request is stamped `now`; run()
    // passes its cycle's time, so that seconds_since() never sees it in the future.
    void request_pose_reset(const rclcpp::Time& now);
    void seed();
    void reset_pose_callback(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response);
    void rebase(bool zeroed);
    void update_health();
    void publish_zero_thrusters();
    void publish_diagnostics(const rclcpp::Time& now, const ThrusterOutput& output, const Vector6d& wrench);

    // Measured pose [x, y, z, roll, pitch, yaw], z in odom (depth) coordinates
    // even while the z target is an altitude.
    Vector6d measured_pose() const;
    double seconds_since(const std::optional<rclcpp::Time>& stamp, const rclcpp::Time& now) const;

    rcl_interfaces::msg::SetParametersResult on_parameters_set(const std::vector<rclcpp::Parameter>& params);
    void declare_parameters();

    // Parameters
    double control_rate_hz_{50.0};
    std::string robot_name_{""};
    ControlGains gains_;
    VehicleModel model_;
    ReferenceLimits limits_;
    ThrusterSettings thruster_settings_;
    ThrusterGeometries thruster_geometry_{};
    ThrusterArray health_parameter_{1, 1, 1, 1, 1, 1, 1, 1};
    std::array<bool, NUM_THRUSTERS> failed_parameter_{};
    ThrusterArray health_reported_{1, 1, 1, 1, 1, 1, 1, 1};
    Vector6d axis_weights_{(Vector6d() << 1, 1, 100, 100, 100, 10).finished()};
    double state_timeout_{0.5};
    double imu_timeout_{0.2};
    double dvl_timeout_{1.5};
    double depth_timeout_{1.0};
    double altitude_timeout_{1.0};
    double cmd_vel_timeout_{0.5};

    // The pipeline
    ReferenceGenerator generator_;
    MotionController controller_;
    ThrusterAllocator allocator_;
    AxisFlags controllable_{true, true, true, true, true, true};
    ThrusterArray last_command_{};
    Vector6d last_wrench_{Vector6d::Zero()};
    ThrusterOutput last_output_;

    // Latest measurements
    VehicleState state_;
    std::optional<rclcpp::Time> odom_time_;
    std::optional<rclcpp::Time> imu_time_;
    double altitude_{NAN};
    std::optional<rclcpp::Time> altitude_time_;
    std::optional<rclcpp::Time> dvl_time_;
    std::optional<rclcpp::Time> depth_time_;

    // Supervision
    bool killed_{true};
    bool seeded_{false};
    std::optional<rclcpp::Time> reset_requested_;
    std::optional<rclcpp::Time> reset_done_;
    std::vector<MotionCommand> pending_commands_;
    // A reset_pose call is waiting for the EKF; rebase_yaw_ is the yaw it zeroes.
    bool rebasing_{false};
    double rebase_yaw_{0.0};
    bool failsafe_{false};
    bool dvl_ok_{false};
    bool depth_ok_{false};
    std::optional<rclcpp::Time> last_update_time_;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr altitude_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dvl_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr depth_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_health_sub_;
    rclcpp::Subscription<sub_control_interfaces::msg::MotionSetpoint>::SharedPtr motion_setpoint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Publishers
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, NUM_THRUSTERS> thruster_pubs_;
    rclcpp::Publisher<sub_control_interfaces::msg::Error>::SharedPtr error_pub_;
    rclcpp::Publisher<sub_control_interfaces::msg::ControllerStatus>::SharedPtr status_pub_;

    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr set_pose_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_pose_srv_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::optional<Eigen::Quaterniond> imu_to_base_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

#endif  // SUB_CONTROL_SUB_CONTROL_HPP_
