#ifndef SUB_CONTROL_SUB_CONTROL_HPP_
#define SUB_CONTROL_SUB_CONTROL_HPP_

#include "sub_control/allocator.hpp"
#include "sub_control/controller.hpp"
#include "sub_control/reference_generator.hpp"
#include "sub_control/safety.hpp"
// Generated from src/sub_control_parameters.yaml by generate_parameter_library.
#include "sub_control/sub_control_parameters.hpp"
#include "sub_control/utils.hpp"
#include "sub_control_interfaces/msg/controller_status.hpp"
#include "sub_control_interfaces/msg/error.hpp"
#include "sub_control_interfaces/msg/motion_setpoint.hpp"
#include "sub_control_interfaces/msg/thruster_command.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

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
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// The sub_control node: trajectory generator -> control law -> thrust
// allocation, plus arming and the watchdogs that decide which axes still have
// feedback and whether the thrusters may run at all. See docs/control.md.
class SubControl : public rclcpp::Node {
   public:
    SubControl();

   private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void altitude_callback(const sensor_msgs::msg::Range::SharedPtr msg);
    void kill_callback(const std_msgs::msg::Bool::SharedPtr msg);

    void motion_setpoint_callback(const sub_control_interfaces::msg::MotionSetpoint::SharedPtr msg);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void run();
    // Arms once released after a kill with nothing blocking it; returns whether armed.
    bool try_arm(const rclcpp::Time& now);
    void set_armed(bool armed);

    // Applies a command, or queues it while the pose reset is under way. A
    // MotionSetpoint's `id` is acknowledged in control/error and control/status.
    void apply_command(const MotionCommand& command, const std::string& source,
                       std::optional<std::uint64_t> id = std::nullopt);
    void acknowledge(std::optional<std::uint64_t> id, const std::string& rejected);
    // Applies the commands queued while the pose reset was under way.
    void apply_pending_commands();
    // Asks the EKF to zero x, y and yaw. The request is stamped `now`; run()
    // passes its cycle's time, so that seconds_since() never sees it in the future.
    void request_pose_reset(const rclcpp::Time& now);
    void seed();
    void reset_pose_callback(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response);
    void rebase(bool zeroed);
    void update_health();
    void publish_thrusters(const rclcpp::Time& now, const ThrusterArray& command);
    void publish_zero_thrusters(const rclcpp::Time& now);
    void publish_diagnostics(const rclcpp::Time& now, const ThrusterOutput& output, const Vector6d& wrench);
    void diagnose_arming(diagnostic_updater::DiagnosticStatusWrapper& status);
    void diagnose_sensors(diagnostic_updater::DiagnosticStatusWrapper& status);
    void diagnose_thrusters(diagnostic_updater::DiagnosticStatusWrapper& status);

    // Measured pose [x, y, z, roll, pitch, yaw], z in odom (depth) coordinates
    // even while the z target is an altitude.
    Vector6d measured_pose() const;
    // Since an event this node timed itself (infinite if none, or if the clock went back).
    double seconds_since(const std::optional<rclcpp::Time>& stamp, const rclcpp::Time& now) const;
    // Age of the latest message on a topic, by its stamp (see message_age()).
    static double age(const std::optional<rclcpp::Time>& stamp, const rclcpp::Time& now);
    // The time a message describes: its stamp, or its arrival if it has none.
    rclcpp::Time stamp_of(const std_msgs::msg::Header& header) const;
    // The DVL altitude, or NaN if it is older than safety.altitude_timeout.
    double fresh_altitude(const rclcpp::Time& now) const;
    ArmingInputs arming_inputs(const rclcpp::Time& now) const;
    ArmingLimits arming_limits() const;

    // Takes up a validated set of parameters (see sub_control_parameters.yaml).
    void apply_parameters(const sub_control::Params& params);
    // Rejects a change that would break a check across parameters, before the
    // parameter listener takes it up.
    rcl_interfaces::msg::SetParametersResult check_parameters(const std::vector<rclcpp::Parameter>& changes) const;

    // Parameters: as set, and mapped onto the pipeline's structs
    std::unique_ptr<sub_control::ParamListener> param_listener_;
    sub_control::Params params_;
    ControlGains gains_;
    VehicleModel model_;
    ReferenceLimits limits_;
    ThrusterSettings thruster_settings_;

    // The pipeline
    ReferenceGenerator generator_;
    MotionController controller_;
    ThrusterAllocator allocator_;
    AxisFlags controllable_{true, true, true, true, true, true};
    ThrusterArray last_command_{};
    Vector6d last_wrench_{Vector6d::Zero()};
    ThrusterOutput last_output_;

    // Latest measurements, each with the time it was measured (its stamp)
    VehicleState state_;
    std::optional<rclcpp::Time> odom_time_;
    std::optional<rclcpp::Time> imu_time_;       // any IMU message: attitude is measured
    std::optional<rclcpp::Time> imu_rate_time_;  // the gyro rate in state_ came from the IMU
    double altitude_{NAN};
    std::optional<rclcpp::Time> altitude_time_;
    std::optional<rclcpp::Time> dvl_time_;
    std::optional<rclcpp::Time> depth_time_;

    // Supervision
    bool killed_{true};
    bool kill_seen_{false};  // the kill switch has been engaged since startup
    bool armed_{false};
    std::vector<std::string> arming_blockers_;
    std::optional<rclcpp::Time> arming_logged_;
    bool seeded_{false};
    std::optional<rclcpp::Time> reset_requested_;
    std::optional<rclcpp::Time> reset_done_;
    std::vector<std::pair<MotionCommand, std::optional<std::uint64_t>>> pending_commands_;
    // A reset_pose call is waiting for the EKF; rebase_yaw_ is the yaw it zeroes.
    bool rebasing_{false};
    double rebase_yaw_{0.0};
    bool failsafe_{false};
    std::string failsafe_reason_;
    bool imu_ok_{false};
    bool dvl_ok_{false};
    bool depth_ok_{false};
    std::optional<rclcpp::Time> last_update_time_;
    // The last MotionSetpoint taken up, and why it was rejected ("" if applied).
    std::uint64_t command_id_{0};
    std::string command_rejected_;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr altitude_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dvl_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr depth_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub_;
    rclcpp::Subscription<sub_control_interfaces::msg::MotionSetpoint>::SharedPtr motion_setpoint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Publishers
    rclcpp::Publisher<sub_control_interfaces::msg::ThrusterCommand>::SharedPtr thruster_pub_;
    rclcpp::Publisher<sub_control_interfaces::msg::Error>::SharedPtr error_pub_;
    rclcpp::Publisher<sub_control_interfaces::msg::ControllerStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;

    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr set_pose_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_pose_srv_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::optional<Eigen::Quaterniond> imu_to_base_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    // /diagnostics, once a second. Last, so it is destroyed before what it reads.
    std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
};

#endif  // SUB_CONTROL_SUB_CONTROL_HPP_
