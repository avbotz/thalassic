#ifndef SUB_CONTROL_SUB_CONTROL_HPP_
#define SUB_CONTROL_SUB_CONTROL_HPP_

#include <array>
#include <map>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sub_control/PID.hpp"
#include "sub_control/utils.hpp"
#include "sub_control_interfaces/msg/error.hpp"
#include "sub_control_interfaces/msg/setpoint.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

class ThrusterControl : public rclcpp::Node {
   public:
    ThrusterControl();

   private:
    rclcpp::Subscription<sub_control_interfaces::msg::Setpoint>::SharedPtr pos_sub;
    rclcpp::Subscription<sub_control_interfaces::msg::Setpoint>::SharedPtr att_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr altitude_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub_;

    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, NUM_THRUSTERS> thruster_pubs_;

    rclcpp::Publisher<sub_control_interfaces::msg::Error>::SharedPtr error_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string world_frame_;
    std::string control_frame_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Time last_control_time_;
    bool have_last_control_time_{false};

    double control_rate_hz;
    double max_force_{25.0};  // per-thruster force cap [N] (T200 ~+/-40 N usable)

    // Clamps every thruster's normalized command to [-power_level_, +power_level_].
    // Limits the PWM band to 1500 +/- power_level_*400 us.
    static constexpr double MAX_POWER_LEVEL = 0.6;
    double power_level_{MAX_POWER_LEVEL};

    // Kill switch (subscribed topic). While true, zero commands are published.
    // On the true->false edge the sub adopts its current pose as the (0, 0)
    // position / yaw=0 origin (see kill_callback).
    bool killed_{true};

    std::mutex state_mutex_;
    std::array<double, 3> pos_curr_{};
    std::array<double, 3> vel_curr_{};
    std::array<double, 3> att_curr_{};
    std::array<double, 3> ang_curr_{};
    double altitude_curr_{0.0};

    std::array<double, 3> initial_pos_{};
    double initial_yaw_{0.0};
    bool state_initialized_{false};

    std::array<double, 3> pos_setpoint{};
    std::array<double, 3> vel_setpoint{};
    bool vel_control{true};
    bool use_altitude_{false};

    std::array<double, 3> att_setpoint{};
    std::array<double, 3> ang_setpoint{};
    bool ang_control{false};

    ThrusterAllocator allocator_;
    std::array<double, NUM_DOF> cap_{};  // per-axis wrench capacity at max_force_

    std::array<PID, 3> pos_pid{};
    std::array<PID, 3> vel_pid{};
    std::array<PID, 3> att_pid{};
    std::array<PID, 3> ang_pid{};

    // Live gain reconfigure: every PID is declared as parameters
    // "<key>.kp/.ki/.kd" (key e.g. "gains.vel.x"). pid_by_key_ maps the key to
    // the controller; gain_cache_ holds its current [kp,ki,kd] so the post-set
    // callback can rebuild gains from one changed parameter without re-reading.
    std::map<std::string, PID*> pid_by_key_;
    std::map<std::string, std::array<double, 3>> gain_cache_;
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_handle_;

    // Derivative low-pass time constants [s]. Outer (pos/att) loops act on the
    // smoother TF pose, inner (vel/ang) on the noisier twist estimate; without a
    // filter the derivative-on-measurement term chatters the thrusters at rest.
    double pos_tau_d_{0.10}, att_tau_d_{0.10};
    double vel_tau_d_{0.05}, ang_tau_d_{0.05};
    double max_speed_{3.0};
    double max_ang_rate_{0.6};
    double inner_limit_margin_{0.9};

    void update_pos_setpoint(const sub_control_interfaces::msg::Setpoint& setpoint);
    void update_att_setpoint(const sub_control_interfaces::msg::Setpoint& setpoint);
    void odom_callback(const nav_msgs::msg::Odometry& odom);
    void altitude_callback(const std_msgs::msg::Float64& msg);
    void kill_callback(const std_msgs::msg::Bool& msg);
    void pid_control_loop();
    bool update_pose_from_tf();
    void publish_zero_thrusters();
    void reset_controllers();

    void declare_gain_parameters();

    // Declare "<key>.kp/.ki/.kd" parameters (defaults given), build the PID into
    // `slot`, and register it for live reconfigure.
    void init_pid(const std::string& key, PID& slot, double kp, double ki, double kd, double tau_d, double out_min,
                  double out_max);
};

#endif  // SUB_CONTROL_SUB_CONTROL_HPP_
