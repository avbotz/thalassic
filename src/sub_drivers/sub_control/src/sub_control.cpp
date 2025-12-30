#include "sub_control/sub_control.hpp"
#include "sub_control/utils.hpp"

#include <array>
#include <cmath>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sub_control/PID.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// All frame conversions (ENU<->NED, angle wrapping, planar rotation) and the
// thruster allocator live in sub_control/utils so they can be unit tested.

ThrusterControl::ThrusterControl() : rclcpp::Node("thruster_control") {
    this->declare_parameter("control_frame", "base_link");
    this->declare_parameter("world_frame", "odom");
    this->declare_parameter<std::string>("odom_topic", "odometry/filtered");
    this->declare_parameter("control_rate_hz", 50.0);
    this->declare_parameter("thruster_max_force", 35.0);

    control_frame_ = this->get_parameter("control_frame").as_string();
    world_frame_ = this->get_parameter("world_frame").as_string();
    control_rate_hz = this->get_parameter("control_rate_hz").as_double();
    max_force_ = this->get_parameter("thruster_max_force").as_double();

    const std::string odom_topic = this->get_parameter("odom_topic").as_string();

    cap_ = allocator_.max_wrench(max_force_);

    pos_pid[0] = PID{0.8, 0.0, 0.0, 0.0, -max_speed_, max_speed_};
    pos_pid[1] = PID{0.8, 0.0, 0.0, 0.0, -max_speed_, max_speed_};
    pos_pid[2] = PID{0.8, 0.0, 0.0, 0.0, -max_speed_, max_speed_};

    att_pid[0] = PID{1.5, 0.0, 0.0, 0.0, -max_ang_rate_, max_ang_rate_};
    att_pid[1] = PID{1.5, 0.0, 0.0, 0.0, -max_ang_rate_, max_ang_rate_};
    att_pid[2] = PID{1.0, 0.0, 0.0, 0.0, -max_ang_rate_, max_ang_rate_};

    vel_pid[0] = PID{40.0, 8.0, 2.0, 0.0, -cap_[0], cap_[0]};
    vel_pid[1] = PID{40.0, 8.0, 2.0, 0.0, -cap_[1], cap_[1]};
    vel_pid[2] = PID{60.0, 12.0, 3.0, 0.0, -cap_[2], cap_[2]};

    ang_pid[0] = PID{6.0, 0.0, 1.0, 0.0, -cap_[3], cap_[3]};
    ang_pid[1] = PID{6.0, 0.0, 1.0, 0.0, -cap_[4], cap_[4]};
    ang_pid[2] = PID{8.0, 0.0, 1.5, 0.0, -cap_[5], cap_[5]};

    pos_sub = this->create_subscription<sub_control_interfaces::msg::Setpoint>(
        "pos_setpoint", 10,
        std::bind(&ThrusterControl::update_pos_setpoint, this, std::placeholders::_1));
    att_sub = this->create_subscription<sub_control_interfaces::msg::Setpoint>(
        "att_setpoint", 10,
        std::bind(&ThrusterControl::update_att_setpoint, this, std::placeholders::_1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(),
        std::bind(&ThrusterControl::odom_callback, this, std::placeholders::_1));
    altitude_sub = this->create_subscription<std_msgs::msg::Float64>(
        "dvl_altitude", 10,
        std::bind(&ThrusterControl::altitude_callback, this, std::placeholders::_1));

    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        thruster_pubs_[i] = this->create_publisher<std_msgs::msg::Float64>(
            "control/thruster_" + std::to_string(i), 10);
    }

    const std::array<std::string, 3> axes{"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
        pos_error_pubs_[i] = this->create_publisher<std_msgs::msg::Float64>("/control/pos/" + axes[i], 10);
        vel_error_pubs_[i] = this->create_publisher<std_msgs::msg::Float64>("/control/vel/" + axes[i], 10);
        att_error_pubs_[i] = this->create_publisher<std_msgs::msg::Float64>("/control/ang/" + axes[i], 10);
        angvel_error_pubs_[i] = this->create_publisher<std_msgs::msg::Float64>("/control/angvel/" + axes[i], 10);
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_hz)),
        std::bind(&ThrusterControl::pid_control_loop, this));

    RCLCPP_INFO(this->get_logger(),
                "Thruster Control started. rate=%.1f Hz, max_force=%.1f N. "
                "Capacity [N,N*m] surge=%.0f sway=%.0f heave=%.0f roll=%.1f pitch=%.1f yaw=%.1f",
                control_rate_hz, max_force_, cap_[0], cap_[1], cap_[2], cap_[3], cap_[4], cap_[5]);
}

void ThrusterControl::declare_gain_parameters() {
    max_speed_    = this->declare_parameter("max_speed", max_speed_);
    max_ang_rate_ = this->declare_parameter("max_ang_rate", max_ang_rate_);
    vel_tau_d_    = this->declare_parameter("vel_tau_d", vel_tau_d_);
    ang_tau_d_    = this->declare_parameter("ang_tau_d", ang_tau_d_);
}

void ThrusterControl::update_pos_setpoint(const sub_control_interfaces::msg::Setpoint& setpoint) {
    if (setpoint.velocity) {
        vel_setpoint[0] = setpoint.setpoint.x;
        vel_setpoint[1] = setpoint.setpoint.y;
        vel_setpoint[2] = setpoint.setpoint.z;
    } else {
        pos_setpoint[0] = setpoint.setpoint.x;
        pos_setpoint[1] = setpoint.setpoint.y;
        pos_setpoint[2] = setpoint.setpoint.z;
    }
    vel_control = setpoint.velocity;
    use_altitude_ = setpoint.use_altitude;
}

void ThrusterControl::update_att_setpoint(const sub_control_interfaces::msg::Setpoint& setpoint) {
    if (setpoint.velocity) {
        ang_setpoint[0] = setpoint.setpoint.x;
        ang_setpoint[1] = setpoint.setpoint.y;
        ang_setpoint[2] = setpoint.setpoint.z;
    } else {
        att_setpoint[0] = setpoint.setpoint.x;
        att_setpoint[1] = setpoint.setpoint.y;
        att_setpoint[2] = setpoint.setpoint.z;
    }
    ang_control = setpoint.velocity;
}

void ThrusterControl::odom_callback(const nav_msgs::msg::Odometry& odom) {
    std::lock_guard<std::mutex> lk(state_mutex_);

    // EKF publishes twist in base_link (FLU). Convert to FRD for the
    // NED-internal cascade: forward unchanged, y/z flipped.
    auto sanitize = [](double v) { return std::fabs(v) < 5.0 ? v : 0.0; };
    vel_curr_[0] =  sanitize(odom.twist.twist.linear.x);
    vel_curr_[1] = -sanitize(odom.twist.twist.linear.y);
    vel_curr_[2] = -sanitize(odom.twist.twist.linear.z);

    ang_curr_[0] =  odom.twist.twist.angular.x;
    ang_curr_[1] = -odom.twist.twist.angular.y;
    ang_curr_[2] = -odom.twist.twist.angular.z;
}

void ThrusterControl::altitude_callback(const std_msgs::msg::Float64& msg) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    altitude_curr_ = msg.data;
}

bool ThrusterControl::update_pose_from_tf() {
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(world_frame_, control_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Could not lookup transform %s -> %s: %s",
                             world_frame_.c_str(), control_frame_.c_str(), ex.what());
        return false;
    }

    tf2::Quaternion tf2_quat;
    tf2::fromMsg(tf.transform.rotation, tf2_quat);
    tf2_quat.normalize();

    double roll_e = 0.0, pitch_e = 0.0, yaw_e = 0.0;
    tf2::Matrix3x3(tf2_quat).getRPY(roll_e, pitch_e, yaw_e);

    double north = 0.0, east = 0.0, down = 0.0;
    enu_to_ned_position(tf.transform.translation.x,
                        tf.transform.translation.y,
                        tf.transform.translation.z,
                        north, east, down);

    double roll_n = 0.0, pitch_n = 0.0, yaw_n = 0.0;
    enu_to_ned_rpy(roll_e, pitch_e, yaw_e, roll_n, pitch_n, yaw_n);

    std::lock_guard<std::mutex> lk(state_mutex_);

    if (!state_initialized_) {
        initial_pos_[0] = north;
        initial_pos_[1] = east;
        initial_pos_[2] = down;
        initial_yaw_ = yaw_n;
        state_initialized_ = true;
    }

    pos_curr_[0] = north - initial_pos_[0];
    pos_curr_[1] = east  - initial_pos_[1];
    pos_curr_[2] = down  - initial_pos_[2];

    att_curr_[0] = normalize_angle(roll_n);
    att_curr_[1] = normalize_angle(pitch_n);
    att_curr_[2] = normalize_angle(yaw_n - initial_yaw_);

    return true;
}

void ThrusterControl::pid_control_loop() {
    if (!update_pose_from_tf()) {
        return;
    }

    std::array<double, 6> wrench{};          // [Fx, Fy, Fz, Mx, My, Mz] in N / N*m
    std::array<double, 3> pos_errors{};
    std::array<double, 3> pos_meas{};
    std::array<double, 3> vel_errors{};
    std::array<double, 3> att_errors{};
    std::array<double, 3> angvel_errors{};

    // Use actual elapsed time so the integrator and ARW step match real dt.
    const rclcpp::Time now = this->get_clock()->now();
    double dt_s = 1.0 / control_rate_hz;
    if (have_last_control_time_) {
        const double measured = (now - last_control_time_).seconds();
        if (measured > 1e-4 && measured < 1.0) {
            dt_s = measured;
        }
    }
    last_control_time_ = now;
    have_last_control_time_ = true;

    state_mutex_.lock();

    {
        const auto pos_ib = to_rotated_frame(pos_curr_[0], pos_curr_[1], initial_yaw_);
        const double err_x_ib = pos_setpoint[0] - pos_ib[0];
        const double err_y_ib = pos_setpoint[1] - pos_ib[1];

        const double rel_yaw = att_curr_[2];
        const auto err_body = to_rotated_frame(err_x_ib, err_y_ib, rel_yaw);
        pos_errors[0] = err_body[0];
        pos_errors[1] = err_body[1];

        const auto meas_body = to_rotated_frame(pos_ib[0], pos_ib[1], rel_yaw);
        pos_meas[0] = meas_body[0];
        pos_meas[1] = meas_body[1];
        pos_meas[2] = use_altitude_ ? altitude_curr_ : pos_curr_[2];

        pos_errors[2] = use_altitude_ ? (altitude_curr_ - pos_setpoint[2])
                                      : (pos_setpoint[2] - pos_curr_[2]);
    }

    att_errors[0] = angle_difference(att_setpoint[0], att_curr_[0]);
    att_errors[1] = angle_difference(att_setpoint[1], att_curr_[1]);
    att_errors[2] = angle_difference(att_setpoint[2], att_curr_[2]);

    RCLCPP_DEBUG(this->get_logger(),
                 "Pos err body: [%.2f %.2f %.2f]  Att err: [%.2f %.2f %.2f]",
                 pos_errors[0], pos_errors[1], pos_errors[2],
                 att_errors[0], att_errors[1], att_errors[2]);

    if (!vel_control) {
        for (int i = 0; i < 3; ++i)
            vel_setpoint[i] = pos_pid[i].update(pos_errors[i], pos_meas[i], dt_s);
    }
    if (!ang_control) {
        for (int i = 0; i < 3; ++i)
            ang_setpoint[i] = att_pid[i].update(att_errors[i], att_curr_[i], dt_s);
    }

    for (int dof = 0; dof < 3; ++dof) {
        vel_errors[dof] = vel_setpoint[dof] - vel_curr_[dof];
        wrench[dof] = vel_pid[dof].update(vel_errors[dof], vel_curr_[dof], dt_s);

        angvel_errors[dof] = ang_setpoint[dof] - ang_curr_[dof];
        wrench[3 + dof] = ang_pid[dof].update(angvel_errors[dof], ang_curr_[dof], dt_s);
    }

    state_mutex_.unlock();

    const std::array<double, 8> thruster_forces = allocator_.allocate(wrench, max_force_);
    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        std_msgs::msg::Float64 m;
        m.data = force_to_norm(thruster_forces[i]);
        thruster_pubs_[i]->publish(m);
    }

    for (int i = 0; i < 3; ++i) {
        std_msgs::msg::Float64 m;
        m.data = pos_errors[i];   pos_error_pubs_[i]->publish(m);
        m.data = vel_errors[i];   vel_error_pubs_[i]->publish(m);
        m.data = att_errors[i];   att_error_pubs_[i]->publish(m);
        m.data = angvel_errors[i]; angvel_error_pubs_[i]->publish(m);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterControl>());
    rclcpp::shutdown();
    return 0;
}
