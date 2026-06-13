#include "sub_control/sub_control.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <format>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace {

template <std::size_t N>
std::array<double, N> declare_array(rclcpp::Node& node, const std::string& name,
                                    const std::array<double, N>& defaults) {
    const std::vector<double> values =
        node.declare_parameter<std::vector<double>>(name, std::vector<double>(defaults.begin(), defaults.end()));
    if (values.size() != N) {
        throw std::invalid_argument(std::format("parameter '{}' must contain {} values", name, N));
    }

    std::array<double, N> result{};
    std::copy(values.begin(), values.end(), result.begin());
    return result;
}

double finite_or_zero(double value) { return std::isfinite(value) ? value : 0.0; }

}  // namespace

ThrusterControl::ThrusterControl() : rclcpp::Node("thruster_control"), controller_(controller_config_) {
    const std::string odom_topic = this->declare_parameter<std::string>("odom_topic", "odometry/filtered");
    control_rate_hz_ = std::max(this->declare_parameter("control_rate_hz", control_rate_hz_), 1.0);
    feedback_timeout_s_ = std::max(this->declare_parameter("feedback_timeout", feedback_timeout_s_), 0.05);
    max_force_ = std::max(this->declare_parameter("thruster_max_force", max_force_), 0.0);
    power_level_ =
        std::clamp(this->declare_parameter("power_level", power_level_), 0.0, MAX_POWER_LEVEL);
    const double power_limited_force =
        std::min(std::fabs(norm_to_force(-power_level_)), std::fabs(norm_to_force(power_level_)));
    max_force_ = std::min(max_force_, power_limited_force);

    load_controller_config();
    parameter_callback_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) { return update_parameters(parameters); });

    pos_setpoint_sub_ = this->create_subscription<sub_control_interfaces::msg::Setpoint>(
        "pos_setpoint", 10, std::bind(&ThrusterControl::pos_setpoint_callback, this, std::placeholders::_1));
    att_setpoint_sub_ = this->create_subscription<sub_control_interfaces::msg::Setpoint>(
        "att_setpoint", 10, std::bind(&ThrusterControl::att_setpoint_callback, this, std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&ThrusterControl::cmd_vel_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(), std::bind(&ThrusterControl::odom_callback, this, std::placeholders::_1));
    altitude_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "dvl_altitude", rclcpp::SensorDataQoS(),
        std::bind(&ThrusterControl::altitude_callback, this, std::placeholders::_1));
    kill_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "kill_switch", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) { kill_callback(*msg); });

    for (int thruster = 0; thruster < NUM_THRUSTERS; ++thruster) {
        thruster_pubs_[thruster] = this->create_publisher<std_msgs::msg::Float64>(
            std::format("control/thruster_{}", thruster), rclcpp::QoS(1));
    }
    error_pub_ = this->create_publisher<sub_control_interfaces::msg::Error>("control/error", 10);

    const auto period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / control_rate_hz_));
    control_timer_ = this->create_wall_timer(period, std::bind(&ThrusterControl::control_loop, this));

    const auto capacity = allocator_.max_wrench(max_force_);
    RCLCPP_INFO(this->get_logger(),
                "sub_control ready: %.1f Hz, feedback timeout %.2fs, max thruster %.1f N, power %.2f. "
                "Wrench capacity [%.1f %.1f %.1f %.2f %.2f %.2f]",
                control_rate_hz_, feedback_timeout_s_, max_force_, power_level_, capacity[0], capacity[1], capacity[2],
                capacity[3], capacity[4], capacity[5]);
}

void ThrusterControl::load_controller_config() {
    const auto capacity = allocator_.max_wrench(max_force_);
    controller_config_.max_force = {capacity[0] * 0.9, capacity[1] * 0.9, capacity[2] * 0.9};
    controller_config_.max_torque = {capacity[3] * 0.9, capacity[4] * 0.9, capacity[5] * 0.9};

    controller_config_.position_gain =
        declare_array(*this, "controller.position_gain", controller_config_.position_gain);
    controller_config_.position_integral_gain =
        declare_array(*this, "controller.position_integral_gain", controller_config_.position_integral_gain);
    controller_config_.attitude_gain =
        declare_array(*this, "controller.attitude_gain", controller_config_.attitude_gain);
    controller_config_.velocity_kp =
        declare_array(*this, "controller.velocity_kp", controller_config_.velocity_kp);
    controller_config_.velocity_ki =
        declare_array(*this, "controller.velocity_ki", controller_config_.velocity_ki);
    controller_config_.angular_rate_kp =
        declare_array(*this, "controller.angular_rate_kp", controller_config_.angular_rate_kp);
    controller_config_.angular_rate_ki =
        declare_array(*this, "controller.angular_rate_ki", controller_config_.angular_rate_ki);
    controller_config_.max_velocity =
        declare_array(*this, "limits.velocity", controller_config_.max_velocity);
    controller_config_.max_angular_rate =
        declare_array(*this, "limits.angular_rate", controller_config_.max_angular_rate);
    controller_config_.max_linear_acceleration =
        declare_array(*this, "limits.linear_acceleration", controller_config_.max_linear_acceleration);
    controller_config_.max_angular_acceleration =
        declare_array(*this, "limits.angular_acceleration", controller_config_.max_angular_acceleration);
    controller_config_.velocity_integral_limit =
        declare_array(*this, "limits.velocity_integral", controller_config_.velocity_integral_limit);
    controller_config_.angular_integral_limit =
        declare_array(*this, "limits.angular_integral", controller_config_.angular_integral_limit);
    controller_config_.position_integral_limit =
        declare_array(*this, "limits.position_integral", controller_config_.position_integral_limit);
    allocation_weights_ = declare_array(*this, "allocator.axis_weights", allocation_weights_);

    std::lock_guard<std::mutex> lock(controller_mutex_);
    controller_.set_config(controller_config_);
}

rcl_interfaces::msg::SetParametersResult ThrusterControl::update_parameters(
    const std::vector<rclcpp::Parameter>& parameters) {
    ControllerConfig updated_config;
    std::array<double, NUM_DOF> updated_weights{};
    double updated_power = 0.0;
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        updated_config = controller_config_;
        updated_weights = allocation_weights_;
        updated_power = power_level_;
    }

    auto copy_array = [](const rclcpp::Parameter& parameter, auto& target) {
        const auto values = parameter.as_double_array();
        if (values.size() != target.size()) {
            return false;
        }
        std::copy(values.begin(), values.end(), target.begin());
        return std::all_of(target.begin(), target.end(), [](double value) { return std::isfinite(value); });
    };

    for (const auto& parameter : parameters) {
        const std::string& name = parameter.get_name();
        bool valid = true;
        try {
            if (name == "controller.position_gain") {
                valid = copy_array(parameter, updated_config.position_gain);
            } else if (name == "controller.position_integral_gain") {
                valid = copy_array(parameter, updated_config.position_integral_gain);
            } else if (name == "controller.attitude_gain") {
                valid = copy_array(parameter, updated_config.attitude_gain);
            } else if (name == "controller.velocity_kp") {
                valid = copy_array(parameter, updated_config.velocity_kp);
            } else if (name == "controller.velocity_ki") {
                valid = copy_array(parameter, updated_config.velocity_ki);
            } else if (name == "controller.angular_rate_kp") {
                valid = copy_array(parameter, updated_config.angular_rate_kp);
            } else if (name == "controller.angular_rate_ki") {
                valid = copy_array(parameter, updated_config.angular_rate_ki);
            } else if (name == "limits.velocity") {
                valid = copy_array(parameter, updated_config.max_velocity);
            } else if (name == "limits.angular_rate") {
                valid = copy_array(parameter, updated_config.max_angular_rate);
            } else if (name == "limits.linear_acceleration") {
                valid = copy_array(parameter, updated_config.max_linear_acceleration);
            } else if (name == "limits.angular_acceleration") {
                valid = copy_array(parameter, updated_config.max_angular_acceleration);
            } else if (name == "limits.velocity_integral") {
                valid = copy_array(parameter, updated_config.velocity_integral_limit);
            } else if (name == "limits.angular_integral") {
                valid = copy_array(parameter, updated_config.angular_integral_limit);
            } else if (name == "limits.position_integral") {
                valid = copy_array(parameter, updated_config.position_integral_limit);
            } else if (name == "allocator.axis_weights") {
                valid = copy_array(parameter, updated_weights);
            } else if (name == "power_level") {
                updated_power = parameter.as_double();
                valid = std::isfinite(updated_power) && updated_power >= 0.0 && updated_power <= MAX_POWER_LEVEL;
            }
        } catch (const std::exception&) {
            valid = false;
        }

        if (!valid) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = false;
            result.reason = std::format("invalid value for parameter '{}'", name);
            return result;
        }
    }

    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        controller_config_ = updated_config;
        allocation_weights_ = updated_weights;
        power_level_ = updated_power;
    }
    {
        std::lock_guard<std::mutex> controller_lock(controller_mutex_);
        controller_.set_config(updated_config);
        have_control_time_ = false;
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void ThrusterControl::pos_setpoint_callback(const sub_control_interfaces::msg::Setpoint& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Mission FLU -> internal NED-style axes (x forward, y right, z down). The
    // same conversion applies whether the command is a position (m) or a
    // linear velocity (m/s).
    const std::array<double, CONTROL_AXES> command{
        finite_or_zero(msg.setpoint.x),
        -finite_or_zero(msg.setpoint.y),
        -finite_or_zero(msg.setpoint.z),
    };
    const bool new_position_control = !msg.velocity;
    if (msg.velocity) {
        velocity_setpoint_ = command;
    } else {
        position_setpoint_ = command;
    }
    if (position_control_ != new_position_control) {
        reset_control_state();
    }
    position_control_ = new_position_control;
    use_altitude_ = false;
}

void ThrusterControl::att_setpoint_callback(const sub_control_interfaces::msg::Setpoint& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Mission FLU (roll about x, pitch about y, yaw about z) -> internal
    // NED-relative axes: pitch and yaw flip sign.
    const double roll = finite_or_zero(msg.setpoint.roll);
    const double pitch = -finite_or_zero(msg.setpoint.pitch);
    const double yaw = -finite_or_zero(msg.setpoint.yaw);
    if (msg.velocity) {
        // Angular rates (rad/s) are not wrapped to [-pi, pi].
        angular_rate_setpoint_ = {roll, pitch, yaw};
    } else {
        attitude_setpoint_ = {normalize_angle(roll), normalize_angle(pitch), normalize_angle(yaw)};
    }
    attitude_control_ = !msg.velocity;
}

void ThrusterControl::cmd_vel_callback(const geometry_msgs::msg::Twist& msg) {
    set_linear_velocity_command(msg.linear.x, msg.linear.y, msg.linear.z);
    set_angular_velocity_command(msg.angular.x, msg.angular.y, msg.angular.z);
}

void ThrusterControl::set_linear_velocity_command(double x_flu, double y_flu, double z_flu) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Body FLU -> FRD.
    velocity_setpoint_ = {finite_or_zero(x_flu), -finite_or_zero(y_flu), -finite_or_zero(z_flu)};
    if (position_control_) {
        reset_control_state();
    }
    position_control_ = false;
    use_altitude_ = false;
}

void ThrusterControl::set_angular_velocity_command(double x_flu, double y_flu, double z_flu) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Body FLU -> FRD.
    angular_rate_setpoint_ = {finite_or_zero(x_flu), -finite_or_zero(y_flu), -finite_or_zero(z_flu)};
    attitude_control_ = false;
}

void ThrusterControl::odom_callback(const nav_msgs::msg::Odometry& odom) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    const auto& pose = odom.pose.pose;
    tf2::Quaternion quaternion;
    tf2::fromMsg(pose.orientation, quaternion);
    if (quaternion.length2() < 1e-12) {
        return;
    }
    quaternion.normalize();

    double roll_enu = 0.0;
    double pitch_enu = 0.0;
    double yaw_enu = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll_enu, pitch_enu, yaw_enu);

    std::array<double, CONTROL_AXES> position{};
    enu_to_ned_position(pose.position.x, pose.position.y, pose.position.z, position[0], position[1], position[2]);
    std::array<double, CONTROL_AXES> attitude{};
    enu_to_ned_rpy(roll_enu, pitch_enu, yaw_enu, attitude[0], attitude[1], attitude[2]);

    if (!state_initialized_) {
        initial_position_ned_ = position;
        initial_yaw_ = attitude[2];
        state_initialized_ = true;
    }
    for (int axis = 0; axis < CONTROL_AXES; ++axis) {
        position_ned_[axis] = position[axis] - initial_position_ned_[axis];
    }
    attitude_ned_[0] = normalize_angle(attitude[0]);
    attitude_ned_[1] = normalize_angle(attitude[1]);
    attitude_ned_[2] = normalize_angle(attitude[2] - initial_yaw_);

    velocity_frd_ = {
        finite_or_zero(odom.twist.twist.linear.x),
        -finite_or_zero(odom.twist.twist.linear.y),
        -finite_or_zero(odom.twist.twist.linear.z),
    };
    angular_rate_frd_ = {
        finite_or_zero(odom.twist.twist.angular.x),
        -finite_or_zero(odom.twist.twist.angular.y),
        -finite_or_zero(odom.twist.twist.angular.z),
    };
    last_odom_time_ = std::chrono::steady_clock::now();
    have_odom_ = true;
}

void ThrusterControl::altitude_callback(const std_msgs::msg::Float64& msg) {
    if (!std::isfinite(msg.data) || msg.data < 0.0) {
        return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    altitude_ = msg.data;
    last_altitude_time_ = std::chrono::steady_clock::now();
    have_altitude_ = true;
}

void ThrusterControl::kill_callback(const std_msgs::msg::Bool& msg) {
    bool stop_now = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (killed_ && !msg.data) {
            state_initialized_ = false;
            position_setpoint_.fill(0.0);
            velocity_setpoint_.fill(0.0);
            attitude_setpoint_.fill(0.0);
            angular_rate_setpoint_.fill(0.0);
            position_control_ = false;
            attitude_control_ = true;
            use_altitude_ = false;
            reset_control_state();
            RCLCPP_INFO(this->get_logger(), "Kill switch released; recapturing control origin");
        } else if (!killed_ && msg.data) {
            reset_control_state();
            stop_now = true;
            RCLCPP_WARN(this->get_logger(), "Kill switch engaged; thrusters stopped");
        }
        killed_ = msg.data;
    }
    if (stop_now) {
        publish_zero_thrusters();
    }
}

bool ThrusterControl::feedback_is_fresh(bool require_altitude) const {
    const auto now = std::chrono::steady_clock::now();
    if (!have_odom_ || std::chrono::duration<double>(now - last_odom_time_).count() > feedback_timeout_s_) {
        return false;
    }
    return !require_altitude ||
           (have_altitude_ &&
            std::chrono::duration<double>(now - last_altitude_time_).count() <= feedback_timeout_s_);
}

void ThrusterControl::control_loop() {
    bool killed = false;
    bool require_altitude = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        killed = killed_;
        require_altitude = use_altitude_;
        if (killed) {
            reset_control_state();
        }
    }
    if (killed) {
        publish_zero_thrusters();
        return;
    }
    ControlInput input;
    std::array<double, CONTROL_AXES> position_error{};
    std::array<double, CONTROL_AXES> attitude_error{};
    double power = 0.0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (killed_ || !feedback_is_fresh(require_altitude)) {
            reset_control_state();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Control feedback stale; commanding zero thrust");
            power = -1.0;
        } else {
            const auto position_initial =
                to_rotated_frame(position_ned_[0], position_ned_[1], initial_yaw_);
            const auto planar_error_initial =
                std::array<double, 2>{position_setpoint_[0] - position_initial[0],
                                      position_setpoint_[1] - position_initial[1]};
            const auto planar_error_body =
                to_rotated_frame(planar_error_initial[0], planar_error_initial[1], attitude_ned_[2]);

            position_error[0] = planar_error_body[0];
            position_error[1] = planar_error_body[1];
            position_error[2] =
                use_altitude_ ? altitude_ - position_setpoint_[2] : position_setpoint_[2] - position_ned_[2];
            attitude_error = ::attitude_error(attitude_setpoint_, attitude_ned_);

            input.position_error = position_error;
            input.attitude_error = attitude_error;
            input.velocity = velocity_frd_;
            input.angular_rate = angular_rate_frd_;
            input.velocity_command = velocity_setpoint_;
            input.angular_rate_command = angular_rate_setpoint_;
            input.position_control = position_control_;
            input.attitude_control = attitude_control_;
            power = power_level_;
        }
    }
    if (power < 0.0) {
        publish_zero_thrusters();
        return;
    }

    ControlOutput output;
    {
        std::lock_guard<std::mutex> lock(controller_mutex_);
        const auto now = std::chrono::steady_clock::now();
        double dt = 1.0 / control_rate_hz_;
        if (have_control_time_) {
            dt = std::chrono::duration<double>(now - last_control_time_).count();
        }
        last_control_time_ = now;
        have_control_time_ = true;
        output = controller_.update(input, dt);
    }
    const auto forces = allocator_.allocate(output.wrench, max_force_, allocation_weights_);
    publish_thrusters(forces, power);
    publish_errors(position_error, attitude_error, output);
}

void ThrusterControl::publish_errors(const std::array<double, CONTROL_AXES>& position_error,
                                     const std::array<double, CONTROL_AXES>& attitude_error,
                                     const ControlOutput& output) {
    // Internal FRD errors -> REP-103 FLU to match the cmd_* topics: x keeps
    // its sign, y and z flip.
    const auto to_flu = [](const std::array<double, CONTROL_AXES>& frd) {
        return std::array<double, CONTROL_AXES>{frd[0], -frd[1], -frd[2]};
    };
    sub_control_interfaces::msg::Error error;
    error.header.stamp = this->get_clock()->now();
    error.header.frame_id = "base_link";
    error.pos_error = to_flu(position_error);
    error.vel_error = to_flu(output.velocity_error);
    error.att_error = to_flu(attitude_error);
    error.angvel_error = to_flu(output.angular_rate_error);
    error_pub_->publish(error);
}

void ThrusterControl::publish_thrusters(const std::array<double, NUM_THRUSTERS>& forces, double power) {
    for (int thruster = 0; thruster < NUM_THRUSTERS; ++thruster) {
        std_msgs::msg::Float64 command;
        command.data = std::clamp(force_to_norm(forces[thruster]), -power, power);
        thruster_pubs_[thruster]->publish(command);
    }
}

void ThrusterControl::publish_zero_thrusters() {
    std_msgs::msg::Float64 command;
    command.data = 0.0;
    for (const auto& publisher : thruster_pubs_) {
        publisher->publish(command);
    }
}

void ThrusterControl::reset_control_state() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    controller_.reset();
    have_control_time_ = false;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterControl>());
    rclcpp::shutdown();
    return 0;
}
