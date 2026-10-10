#include "sub_control/sub_control.hpp"

#include <tf2/exceptions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <functional>
#include <limits>
#include <stdexcept>

using sub_control_interfaces::msg::MotionSetpoint;

namespace {

constexpr std::array<const char*, NUM_DOF> AXIS_NAMES{"x", "y", "z", "roll", "pitch", "yaw"};
// When thrusters fail, which axes keep the remaining authority: depth first,
// then heading and pitch; roll last, since the hull rights itself.
constexpr std::array<int, NUM_DOF> AXIS_PRIORITY{Z, YAW, PITCH, X, Y, ROLL};
// An axis the thrusters can no longer move gets (almost) no say in allocation.
constexpr double DROPPED_AXIS_WEIGHT = 1e-6;
// How long to wait for the EKF to confirm the pose reset at unkill.
constexpr double POSE_RESET_TIMEOUT = 1.0;

// A double parameter's value, if it is finite. NaN slips through every range
// check below (each comparison is false), so it is rejected here.
std::optional<double> finite_double(const rclcpp::Parameter& p) {
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE && std::isfinite(p.as_double())) {
        return p.as_double();
    }
    return std::nullopt;
}

bool all_of(const std::vector<double>& v, const std::function<bool(double)>& ok) { return std::ranges::all_of(v, ok); }

// A double array parameter's value, if it has n elements, all finite.
std::optional<std::vector<double>> doubles(const rclcpp::Parameter& p, size_t n) {
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY && p.as_double_array().size() == n &&
        all_of(p.as_double_array(), [](double v) { return std::isfinite(v); })) {
        return p.as_double_array();
    }
    return std::nullopt;
}

Eigen::Vector3d vector3(const std::vector<double>& v, size_t offset = 0) {
    return {v[offset], v[offset + 1], v[offset + 2]};
}

Vector6d vector6(const std::vector<double>& v) { return Eigen::Map<const Vector6d>(v.data()); }

bool finite(const nav_msgs::msg::Odometry& msg) {
    const auto& p = msg.pose.pose;
    const auto& t = msg.twist.twist;
    for (const double v :
         {p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
          t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z}) {
        if (!std::isfinite(v)) {
            return false;
        }
    }
    return true;
}

}  // namespace

SubControl::SubControl() : Node("sub_control") {
    declare_parameters();
    std::vector<rclcpp::Parameter> initial_params;
    for (const std::string& name : this->list_parameters({}, 0).names) {
        // Not get_parameter(name): it throws for a parameter declared with only
        // a type and never set, before the explanation below could be given.
        rclcpp::Parameter param;
        if (!this->get_parameter(name, param)) {
            if (name.starts_with("allocation.thruster_")) {
                throw std::invalid_argument(name +
                                            " is not set. The thruster layout comes from the vehicle "
                                            "description; launch sub_control through sub_bringup.");
            }
            continue;
        }
        initial_params.push_back(param);
    }
    const auto initial = on_parameters_set(initial_params);
    if (!initial.successful) {
        throw std::invalid_argument(initial.reason);
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(msg); });
    altitude_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "altitude", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Range::SharedPtr msg) { altitude_callback(msg); });
    // Only their arrival matters here: they say whether the EKF's x/y and z
    // are being corrected or only predicted.
    dvl_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/dvl", rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr) { dvl_time_ = this->now(); });
    depth_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/depth", rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr) { depth_time_ = this->now(); });
    kill_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "kill_switch", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) { kill_callback(msg); });
    thruster_health_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "thruster_health", 1,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { thruster_health_callback(msg); });

    motion_setpoint_sub_ = this->create_subscription<MotionSetpoint>(
        "motion_setpoint", 10, [this](const MotionSetpoint::SharedPtr msg) { motion_setpoint_callback(msg); });
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmd_vel_callback(msg); });

    for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
        thruster_pubs_[i] = this->create_publisher<std_msgs::msg::Float64>(std::format("control/thruster_{}", i), 10);
    }
    error_pub_ = this->create_publisher<sub_control_interfaces::msg::Error>("control/error", 10);
    status_pub_ = this->create_publisher<sub_control_interfaces::msg::ControllerStatus>("control/status", 10);

    set_pose_client_ = this->create_client<robot_localization::srv::SetPose>("set_pose");
    reset_pose_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/reset_pose",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) { reset_pose_callback(response); });

    control_timer_ = this->create_timer(std::chrono::duration<double>(1.0 / control_rate_hz_), [this]() { run(); });

    param_cb_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) { return on_parameters_set(params); });
}

void SubControl::declare_parameters() {
    // control_rate_hz, robot_name and the thruster layout are only used at
    // startup, so a runtime change would be silently ignored.
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;

    this->declare_parameter("control_rate_hz", control_rate_hz_, read_only);
    this->declare_parameter("robot_name", robot_name_, read_only);
    this->declare_parameter("power_limit", thruster_settings_.power_limit);

    // The thruster layout comes from config/vehicles/<robot_name>.yaml via the
    // launch file (sub_bringup/vehicle.py), base_link FLU, eight of each.
    this->declare_parameter("allocation.thruster_positions", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY, read_only);
    this->declare_parameter("allocation.thruster_directions", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY, read_only);
    this->declare_parameter("allocation.thruster_reversed", rclcpp::ParameterType::PARAMETER_BOOL_ARRAY, read_only);
    this->declare_parameter("allocation.axis_weights", std::vector<double>(axis_weights_.begin(), axis_weights_.end()));
    this->declare_parameter("allocation.max_command_rate", thruster_settings_.max_command_rate);
    this->declare_parameter("thruster_health", std::vector<double>(NUM_THRUSTERS, 1.0));
    this->declare_parameter("failed_thrusters", std::vector<int64_t>{});

    const auto v3 = [](const Eigen::Vector3d& v) { return std::vector<double>{v.x(), v.y(), v.z()}; };
    const auto v6 = [](const Vector6d& v) { return std::vector<double>(v.begin(), v.end()); };
    this->declare_parameter("model.mass", v3(model_.mass));
    this->declare_parameter("model.inertia", v3(model_.inertia));
    this->declare_parameter("model.net_buoyancy", model_.net_buoyancy);
    this->declare_parameter("model.buoyancy", model_.buoyancy);
    this->declare_parameter("model.center_of_gravity", v3(model_.center_of_gravity));
    this->declare_parameter("model.center_of_buoyancy", v3(model_.center_of_buoyancy));
    this->declare_parameter("model.linear_drag", v6(model_.linear_drag));
    this->declare_parameter("model.quadratic_drag", v6(model_.quadratic_drag));

    this->declare_parameter("feedforward.acceleration", gains_.acceleration_feedforward);
    this->declare_parameter("feedforward.drag", gains_.drag_feedforward);
    this->declare_parameter("feedforward.restoring", gains_.restoring_feedforward);

    for (int axis = 0; axis < NUM_DOF; ++axis) {
        this->declare_parameter(std::format("gains.{}", AXIS_NAMES[axis]), std::vector<double>{0.0, 0.0, 0.0});
    }
    this->declare_parameter("gains.integral_limit", v6(gains_.integral_limit));
    this->declare_parameter("gains.anti_windup", gains_.anti_windup);
    this->declare_parameter("observer.bandwidth", v6(gains_.observer_bandwidth));

    const auto limits = [](const ShaperLimits& l) {
        return std::vector<double>{l.max_velocity, l.max_acceleration, l.max_jerk};
    };
    this->declare_parameter("trajectory.horizontal", limits(limits_.horizontal));
    this->declare_parameter("trajectory.vertical", limits(limits_.vertical));
    this->declare_parameter("trajectory.roll", limits(limits_.roll));
    this->declare_parameter("trajectory.pitch", limits(limits_.pitch));
    this->declare_parameter("trajectory.yaw", limits(limits_.yaw));
    this->declare_parameter("trajectory.translation_leash", limits_.translation_leash);
    this->declare_parameter("trajectory.rotation_leash", limits_.rotation_leash);

    this->declare_parameter("safety.state_timeout", state_timeout_);
    this->declare_parameter("safety.imu_timeout", imu_timeout_);
    this->declare_parameter("safety.dvl_timeout", dvl_timeout_);
    this->declare_parameter("safety.depth_timeout", depth_timeout_);
    this->declare_parameter("safety.altitude_timeout", altitude_timeout_);
    this->declare_parameter("safety.cmd_vel_timeout", cmd_vel_timeout_);
}

rcl_interfaces::msg::SetParametersResult SubControl::on_parameters_set(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    const auto reject = [&result](const std::string& reason) {
        result.successful = false;
        result.reason = reason;
        return result;
    };

    // Validate everything into copies; commit only if all of it is good.
    ControlGains gains = gains_;
    VehicleModel model = model_;
    ReferenceLimits limits = limits_;
    ThrusterSettings settings = thruster_settings_;
    ThrusterGeometries geometry = thruster_geometry_;
    ThrusterArray health = health_parameter_;
    std::array<bool, NUM_THRUSTERS> failed = failed_parameter_;
    Vector6d weights = axis_weights_;
    std::array<double*, 6> timeouts{&state_timeout_, &imu_timeout_,      &dvl_timeout_,
                                    &depth_timeout_, &altitude_timeout_, &cmd_vel_timeout_};
    std::array<double, 6> timeout_values{};
    for (size_t i = 0; i < timeouts.size(); ++i) {
        timeout_values[i] = *timeouts[i];
    }
    bool geometry_changed = false;

    const auto positive = [](double v) { return v > 0.0; };
    const auto non_negative = [](double v) { return v >= 0.0; };

    for (const auto& param : params) {
        const std::string& name = param.get_name();
        const auto type = param.get_type();
        if (type == rclcpp::ParameterType::PARAMETER_NOT_SET) {
            continue;
        }
        const std::optional<double> number = finite_double(param);
        if (name == "control_rate_hz") {
            if (!number || *number <= 0.0) {
                return reject("control_rate_hz must be a positive double");
            }
            control_rate_hz_ = *number;
        } else if (name == "robot_name") {
            robot_name_ = param.as_string();
        } else if (name == "power_limit") {
            if (!number || *number < 0.0 || *number > MAX_POWER_LIMIT) {
                return reject(std::format("power_limit must be a double in [0, {}]", MAX_POWER_LIMIT));
            }
            settings.power_limit = *number;
        } else if (name == "allocation.thruster_positions" || name == "allocation.thruster_directions") {
            const auto v = doubles(param, 3 * NUM_THRUSTERS);
            if (!v) {
                return reject(std::format("{} must be {} finite doubles, x/y/z per thruster", name, 3 * NUM_THRUSTERS));
            }
            for (int t = 0; t < NUM_THRUSTERS; ++t) {
                const Eigen::Vector3d value = vector3(*v, 3 * t);
                if (name == "allocation.thruster_positions") {
                    geometry[t].position = value;
                } else {
                    if (value.norm() < 1e-6) {
                        return reject(std::format("thruster {} has no direction", t));
                    }
                    geometry[t].direction = value.normalized();
                }
            }
            geometry_changed = true;
        } else if (name == "allocation.thruster_reversed") {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL_ARRAY || param.as_bool_array().size() != NUM_THRUSTERS) {
                return reject(std::format("{} must be {} booleans", name, NUM_THRUSTERS));
            }
            for (int t = 0; t < NUM_THRUSTERS; ++t) {
                geometry[t].reversed = param.as_bool_array()[t];
                settings.reversed[t] = geometry[t].reversed;
            }
        } else if (name == "allocation.axis_weights") {
            const auto v = doubles(param, NUM_DOF);
            if (!v || !all_of(*v, positive)) {
                return reject("allocation.axis_weights must be six positive doubles [x, y, z, roll, pitch, yaw]");
            }
            weights = vector6(*v);
        } else if (name == "allocation.max_command_rate") {
            if (!number || *number < 0.0) {
                return reject("allocation.max_command_rate must be a non-negative double");
            }
            settings.max_command_rate = *number;
        } else if (name == "thruster_health") {
            const auto v = doubles(param, NUM_THRUSTERS);
            if (!v || !all_of(*v, [](double h) { return h >= 0.0 && h <= 1.0; })) {
                return reject(std::format("thruster_health must be {} doubles in [0, 1]", NUM_THRUSTERS));
            }
            std::ranges::copy(*v, health.begin());
        } else if (name == "failed_thrusters") {
            if (type != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
                return reject("failed_thrusters must be an integer array");
            }
            failed.fill(false);
            for (const int64_t thruster : param.as_integer_array()) {
                if (thruster < 0 || thruster >= static_cast<int64_t>(NUM_THRUSTERS)) {
                    return reject(std::format("failed_thrusters: no thruster {}", thruster));
                }
                failed[thruster] = true;
            }
        } else if (name == "model.mass" || name == "model.inertia") {
            const auto v = doubles(param, 3);
            if (!v || !all_of(*v, positive)) {
                return reject(name + " must be three positive doubles, per body axis");
            }
            (name == "model.mass" ? model.mass : model.inertia) = vector3(*v);
        } else if (name == "model.net_buoyancy" || name == "model.buoyancy") {
            if (!number) {
                return reject(name + " must be a finite double");
            }
            (name == "model.net_buoyancy" ? model.net_buoyancy : model.buoyancy) = *number;
        } else if (name == "model.center_of_gravity" || name == "model.center_of_buoyancy") {
            const auto v = doubles(param, 3);
            if (!v) {
                return reject(name + " must be three finite doubles, base_link [x, y, z]");
            }
            if (name == "model.center_of_gravity") {
                model.center_of_gravity = vector3(*v);
                geometry_changed = true;  // thruster moments are about it
            } else {
                model.center_of_buoyancy = vector3(*v);
            }
        } else if (name == "model.linear_drag" || name == "model.quadratic_drag") {
            const auto v = doubles(param, NUM_DOF);
            if (!v || !all_of(*v, non_negative)) {
                return reject(name + " must be six non-negative doubles [x, y, z, roll, pitch, yaw]");
            }
            (name == "model.linear_drag" ? model.linear_drag : model.quadratic_drag) = vector6(*v);
        } else if (name.starts_with("feedforward.")) {
            if (!number || *number < 0.0) {
                return reject(name + " must be a non-negative double (a scale, normally 0 to 1)");
            }
            const std::string term = name.substr(std::string("feedforward.").size());
            if (term == "acceleration") {
                gains.acceleration_feedforward = *number;
            } else if (term == "drag") {
                gains.drag_feedforward = *number;
            } else if (term == "restoring") {
                gains.restoring_feedforward = *number;
            }
        } else if (name == "gains.integral_limit" || name == "observer.bandwidth") {
            const auto v = doubles(param, NUM_DOF);
            if (!v || !all_of(*v, non_negative)) {
                return reject(name + " must be six non-negative doubles [x, y, z, roll, pitch, yaw]");
            }
            (name == "gains.integral_limit" ? gains.integral_limit : gains.observer_bandwidth) = vector6(*v);
        } else if (name == "gains.anti_windup") {
            if (!number || *number < 0.0) {
                return reject("gains.anti_windup must be a non-negative double");
            }
            gains.anti_windup = *number;
        } else if (name.starts_with("gains.")) {
            const auto axis = std::ranges::find(AXIS_NAMES, name.substr(std::string("gains.").size()));
            if (axis == AXIS_NAMES.end()) {
                continue;
            }
            const auto v = doubles(param, 3);
            if (!v || !all_of(*v, non_negative)) {
                return reject(name + " must be [kp, ki, kd], non-negative");
            }
            const auto i = std::distance(AXIS_NAMES.begin(), axis);
            gains.kp[i] = (*v)[0];
            gains.ki[i] = (*v)[1];
            gains.kd[i] = (*v)[2];
        } else if (name == "trajectory.translation_leash" || name == "trajectory.rotation_leash") {
            if (!number || *number <= 0.0) {
                return reject(name + " must be a positive double");
            }
            (name == "trajectory.translation_leash" ? limits.translation_leash : limits.rotation_leash) = *number;
        } else if (name.starts_with("trajectory.")) {
            const std::string group = name.substr(std::string("trajectory.").size());
            ShaperLimits* target = group == "horizontal" ? &limits.horizontal
                                   : group == "vertical" ? &limits.vertical
                                   : group == "roll"     ? &limits.roll
                                   : group == "pitch"    ? &limits.pitch
                                   : group == "yaw"      ? &limits.yaw
                                                         : nullptr;
            if (target == nullptr) {
                continue;
            }
            const auto v = doubles(param, 3);
            if (!v || !all_of(*v, positive)) {
                return reject(name + " must be [max velocity, max acceleration, max jerk], positive");
            }
            *target = {(*v)[0], (*v)[1], (*v)[2]};
        } else if (name.starts_with("safety.")) {
            static const std::array<const char*, 6> names{"safety.state_timeout",    "safety.imu_timeout",
                                                          "safety.dvl_timeout",      "safety.depth_timeout",
                                                          "safety.altitude_timeout", "safety.cmd_vel_timeout"};
            const auto it = std::ranges::find(names, name);
            if (it == names.end()) {
                continue;
            }
            if (!number || *number <= 0.0) {
                return reject(name + " must be a positive double [s]");
            }
            timeout_values[std::distance(names.begin(), it)] = *number;
        }
    }
    // The observer is integrated once per cycle, so its estimate error goes as
    // (1 - K dt) per cycle: it rings once K dt passes 1 and diverges past 2.
    // Checked here, whatever order the batch set control_rate_hz in.
    if ((gains.observer_bandwidth.array() >= control_rate_hz_).any()) {
        return reject(std::format("observer.bandwidth must stay below control_rate_hz ({} rad/s)", control_rate_hz_));
    }

    gains_ = gains;
    model_ = model;
    limits_ = limits;
    thruster_settings_ = settings;
    thruster_geometry_ = geometry;
    health_parameter_ = health;
    failed_parameter_ = failed;
    axis_weights_ = weights;
    for (size_t i = 0; i < timeouts.size(); ++i) {
        *timeouts[i] = timeout_values[i];
    }
    controller_.set_model(model_);
    controller_.set_gains(gains_);
    if (geometry_changed) {
        allocator_ = ThrusterAllocator(thruster_geometry_, model_.center_of_gravity);
    }
    update_health();
    return result;
}

void SubControl::update_health() {
    std::array<bool, NUM_THRUSTERS> usable{};
    for (int t = 0; t < NUM_THRUSTERS; ++t) {
        const double health = failed_parameter_[t] ? 0.0
                                                   : std::clamp(health_parameter_[t], 0.0, 1.0) *
                                                         std::clamp(health_reported_[t], 0.0, 1.0);
        if (health != thruster_settings_.health[t]) {
            if (health < 1.0) {
                RCLCPP_WARN(this->get_logger(), "thruster %d at %.0f%% health; allocating around it", t,
                            100.0 * health);
            } else {
                RCLCPP_INFO(this->get_logger(), "thruster %d healthy", t);
            }
        }
        thruster_settings_.health[t] = health;
        usable[t] = health > 0.0;
    }

    const AxisFlags controllable = allocator_.controllable_axes(usable, AXIS_PRIORITY);
    for (int axis = 0; axis < NUM_DOF; ++axis) {
        if (controllable[axis] != controllable_[axis]) {
            if (controllable[axis]) {
                RCLCPP_INFO(this->get_logger(), "%s is controllable again", AXIS_NAMES[axis]);
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "the working thrusters can no longer move %s independently; leaving it to the hull",
                             AXIS_NAMES[axis]);
            }
        }
        thruster_settings_.axis_weights[axis] = controllable[axis] ? axis_weights_[axis] : DROPPED_AXIS_WEIGHT;
    }
    controllable_ = controllable;
}

void SubControl::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto& p = msg->pose.pose;
    const auto& t = msg->twist.twist;
    const Eigen::Quaterniond orientation(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    // A zero quaternion carries no attitude: normalized() leaves it zero, which
    // would read as level at yaw 0.
    if (!finite(*msg) || orientation.norm() < 1e-6) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "ignoring odometry with NaN/inf or no orientation in it");
        return;
    }
    state_.position = {p.position.x, p.position.y, p.position.z};
    state_.orientation = orientation.normalized();
    state_.linear_velocity = {t.linear.x, t.linear.y, t.linear.z};
    if (!imu_time_ || seconds_since(imu_time_, this->now()) > imu_timeout_) {
        state_.angular_velocity = {t.angular.x, t.angular.y, t.angular.z};
    }
    odom_time_ = this->now();
}

void SubControl::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // The gyro is the freshest, least filtered rate on the vehicle, so the rate
    // feedback uses it directly rather than the EKF's copy of it.
    const auto& w = msg->angular_velocity;
    if (!std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z)) {
        return;
    }
    if (!imu_to_base_) {
        const std::string base = robot_name_.empty() ? "base_link" : robot_name_ + "/base_link";
        try {
            const auto tf = tf_buffer_->lookupTransform(base, msg->header.frame_id, tf2::TimePointZero);
            const auto& q = tf.transform.rotation;
            imu_to_base_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "no transform from %s to %s yet (%s); using the EKF's rates",
                                 msg->header.frame_id.c_str(), base.c_str(), e.what());
            return;
        }
    }
    state_.angular_velocity = *imu_to_base_ * Eigen::Vector3d(w.x, w.y, w.z);
    imu_time_ = this->now();
}

void SubControl::altitude_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
    if (!std::isfinite(msg->range) || msg->range < msg->min_range || msg->range > msg->max_range) {
        return;
    }
    altitude_ = msg->range;
    altitude_time_ = this->now();
}

void SubControl::thruster_health_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != NUM_THRUSTERS) {
        RCLCPP_WARN(this->get_logger(), "thruster_health needs %d values, got %zu", NUM_THRUSTERS, msg->data.size());
        return;
    }
    // NaN is not a health reading, and std::clamp in update_health() passes it through.
    if (!all_of(msg->data, [](double h) { return std::isfinite(h); })) {
        RCLCPP_WARN(this->get_logger(), "ignoring thruster_health with NaN/inf in it");
        return;
    }
    std::ranges::copy(msg->data, health_reported_.begin());
    update_health();
}

void SubControl::kill_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (killed_ && !msg->data) {
        // Released: the pose here becomes the odom origin, and the controller
        // starts holding it once the EKF has caught up. run() asks for the reset
        // once there is a fresh estimate whose depth, roll and pitch it can keep
        // (the switch can be released before the first odometry arrives).
        seeded_ = false;
        pending_commands_.clear();
        reset_requested_.reset();
        reset_done_.reset();
    }
    killed_ = msg->data;
    if (killed_) {
        rebasing_ = false;
        publish_zero_thrusters();
    }
}

void SubControl::request_pose_reset(const rclcpp::Time& now) {
    reset_requested_ = now;
    reset_done_.reset();
    if (!set_pose_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "EKF set_pose is not available; holding the pose the estimate has");
        reset_done_ = now;
        return;
    }
    // Zero x, y and yaw. Depth comes from the pressure sensor and roll/pitch
    // from gravity, so resetting those would only make the EKF jump back.
    const Eigen::Vector3d rpy = rpy_from_quaternion(state_.orientation);
    const Eigen::Quaterniond level_heading = quaternion_from_rpy({rpy.x(), rpy.y(), 0.0});
    auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
    request->pose.header.frame_id = robot_name_.empty() ? "odom" : robot_name_ + "/odom";
    request->pose.header.stamp = now;
    request->pose.pose.pose.position.z = state_.position.z();
    request->pose.pose.pose.orientation.w = level_heading.w();
    request->pose.pose.pose.orientation.x = level_heading.x();
    request->pose.pose.pose.orientation.y = level_heading.y();
    request->pose.pose.pose.orientation.z = level_heading.z();
    // A late reply to an earlier request (killed and released since) confirms
    // nothing about this one.
    set_pose_client_->async_send_request(
        request, [this, requested = now](rclcpp::Client<robot_localization::srv::SetPose>::SharedFuture) {
            if (reset_requested_ == requested) {
                reset_done_ = this->now();
            }
        });
}

void SubControl::reset_pose_callback(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    if (killed_) {
        response->message = "killed: x, y and yaw are zeroed when the kill switch is released";
    } else if (!seeded_ || rebasing_) {
        response->message = "a pose reset is already under way";
    } else if (failsafe_) {
        response->message = "no state estimate";
    } else if (!set_pose_client_->service_is_ready()) {
        response->message = "EKF set_pose is not available";
    } else {
        rebase_yaw_ = rpy_from_quaternion(state_.orientation).z();
        rebasing_ = true;
        request_pose_reset(this->now());
        response->success = true;
        response->message = "zeroing x, y and yaw; holding here once the EKF confirms";
        RCLCPP_INFO(this->get_logger(), "reset_pose: zeroing x, y and yaw");
    }
}

void SubControl::rebase(bool zeroed) {
    // x, y and yaw count from here now, so hold where the vehicle is. Depth,
    // roll and pitch kept their frame: what they were holding carries over.
    const Vector6d target = generator_.target();
    MotionCommand carry;
    carry.altitude = generator_.altitude();
    for (const int axis : {Z, ROLL, PITCH}) {
        // reset() drops the altitude and only a z POSITION brings it back, so an
        // altitude z without a target yet holds where its reference is.
        const bool position = generator_.mode(axis) == AxisMode::POSITION || (axis == Z && carry.altitude);
        carry.mode[axis] = position ? AxisMode::POSITION : AxisMode::HOLD;
        carry.position[axis] = target[axis];
    }
    generator_.reset(measured_pose());
    const double altitude = seconds_since(altitude_time_, this->now()) <= altitude_timeout_
                                ? altitude_
                                : std::numeric_limits<double>::quiet_NaN();
    const std::string rejected = generator_.apply(carry, measured_pose(), altitude);
    if (!rejected.empty()) {
        RCLCPP_WARN(this->get_logger(), "after the pose reset, holding depth and level: %s", rejected.c_str());
    }
    // Currents and buoyancy errors did not turn with the frame.
    if (zeroed) {
        controller_.rotate_world(-rebase_yaw_);
    }
    rebasing_ = false;
    RCLCPP_INFO(this->get_logger(), "pose re-zeroed: holding here");
    for (const MotionCommand& command : pending_commands_) {
        apply_command(command, "queued");
    }
    pending_commands_.clear();
}

void SubControl::seed() {
    generator_.reset(measured_pose());
    controller_.reset();
    last_command_.fill(0.0);
    seeded_ = true;
    for (const MotionCommand& command : pending_commands_) {
        apply_command(command, "queued");
    }
    pending_commands_.clear();
}

Vector6d SubControl::measured_pose() const {
    Vector6d pose;
    pose << state_.position, rpy_from_quaternion(state_.orientation);
    return pose;
}

double SubControl::seconds_since(const std::optional<rclcpp::Time>& stamp, const rclcpp::Time& now) const {
    // A stamp from the future means the clock jumped back (the simulator
    // restarted): stale, or no watchdog would fire until the clock caught up.
    return stamp && *stamp <= now ? (now - *stamp).seconds() : std::numeric_limits<double>::infinity();
}

void SubControl::apply_command(const MotionCommand& command, const std::string& source) {
    if (killed_) {
        RCLCPP_DEBUG(this->get_logger(), "ignoring %s while killed", source.c_str());
        return;
    }
    if (!seeded_ || rebasing_) {
        pending_commands_.push_back(command);
        return;
    }
    const double altitude = seconds_since(altitude_time_, this->now()) <= altitude_timeout_
                                ? altitude_
                                : std::numeric_limits<double>::quiet_NaN();
    const std::string rejected = generator_.apply(command, measured_pose(), altitude);
    if (!rejected.empty()) {
        RCLCPP_WARN(this->get_logger(), "%s rejected: %s", source.c_str(), rejected.c_str());
    }
}

void SubControl::motion_setpoint_callback(const MotionSetpoint::SharedPtr msg) {
    MotionCommand command;
    for (int axis = 0; axis < NUM_DOF; ++axis) {
        if (msg->mode[axis] > MotionSetpoint::EFFORT) {
            RCLCPP_WARN(this->get_logger(), "motion_setpoint rejected: unknown mode %d on %s", msg->mode[axis],
                        AXIS_NAMES[axis]);
            return;
        }
        command.mode[axis] = static_cast<AxisMode>(msg->mode[axis]);
        command.position[axis] = msg->position[axis];
        command.velocity[axis] = msg->velocity[axis];
        command.effort[axis] = msg->effort[axis];
    }
    if (msg->velocity_frame > MotionSetpoint::WORLD) {
        RCLCPP_WARN(this->get_logger(), "motion_setpoint rejected: unknown velocity_frame %d", msg->velocity_frame);
        return;
    }
    // NaN or inf would latch a stream that was meant to stop.
    if (!std::isfinite(msg->timeout) || msg->timeout < 0.0) {
        RCLCPP_WARN(this->get_logger(), "motion_setpoint rejected: timeout must be finite and non-negative, not %f",
                    msg->timeout);
        return;
    }
    command.velocity_frame =
        msg->velocity_frame == MotionSetpoint::WORLD ? VelocityFrame::WORLD : VelocityFrame::HEADING;
    command.altitude = msg->altitude;
    command.expires = msg->timeout > 0.0 ? this->now().seconds() + msg->timeout : 0.0;
    apply_command(command, "motion_setpoint");
}

void SubControl::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // A teleop-style stream: if it stops, the vehicle stops.
    MotionCommand command;
    command.mode.fill(AxisMode::VELOCITY);
    command.velocity << msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z;
    command.expires = this->now().seconds() + cmd_vel_timeout_;
    apply_command(command, "cmd_vel");
}

void SubControl::publish_zero_thrusters() {
    std_msgs::msg::Float64 msg;
    msg.data = 0.0;
    for (auto& pub : thruster_pubs_) {
        pub->publish(msg);
    }
    last_command_.fill(0.0);
    last_output_ = ThrusterOutput{};
    last_wrench_.setZero();
}

void SubControl::run() {
    const rclcpp::Time now = this->now();
    const double dt =
        last_update_time_ ? std::clamp((now - *last_update_time_).seconds(), 1e-4, 0.1) : 1.0 / control_rate_hz_;
    last_update_time_ = now;

    if (killed_) {
        publish_zero_thrusters();
        publish_diagnostics(now, last_output_, Vector6d::Zero());
        return;
    }

    // No state estimate: nothing sensible to do but let the positively
    // buoyant hull float up.
    if (seconds_since(odom_time_, now) > state_timeout_) {
        if (!failsafe_) {
            RCLCPP_ERROR(this->get_logger(), "no odometry for %.2f s: thrusters off", state_timeout_);
        }
        failsafe_ = true;
        publish_zero_thrusters();
        publish_diagnostics(now, last_output_, Vector6d::Zero());
        return;
    }
    if (failsafe_) {
        RCLCPP_WARN(this->get_logger(), "odometry is back; resuming");
        failsafe_ = false;
    }

    if (rebasing_) {
        const bool confirmed = reset_done_ && odom_time_ && *odom_time_ > *reset_done_;
        if (!confirmed && seconds_since(reset_requested_, now) <= POSE_RESET_TIMEOUT) {
            // The references are in the old frame until the EKF re-zeros, so keep
            // the thrust as it was rather than chase them or drop the hold.
            for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
                std_msgs::msg::Float64 msg;
                msg.data = last_command_[i];
                thruster_pubs_[i]->publish(msg);
            }
            publish_diagnostics(now, last_output_, last_wrench_);
            return;
        }
        if (!confirmed) {
            RCLCPP_WARN(this->get_logger(), "EKF did not confirm the pose reset; holding the pose it has");
        }
        rebase(confirmed);
    }

    if (!seeded_) {
        if (!reset_requested_) {
            request_pose_reset(now);
        }
        const bool confirmed = reset_done_ && odom_time_ && *odom_time_ > *reset_done_;
        const bool gave_up = seconds_since(reset_requested_, now) > POSE_RESET_TIMEOUT;
        if (!confirmed && !gave_up) {
            publish_zero_thrusters();
            return;
        }
        if (!confirmed) {
            RCLCPP_WARN(this->get_logger(), "EKF did not confirm the pose reset; holding the pose it has");
        }
        seed();
    }

    // Which axes still have feedback. The EKF keeps predicting through a
    // sensor gap, so its estimate stays fresh while drifting: without DVL
    // updates x/y velocity is only a guess, and without depth so is z.
    const bool dvl_ok = seconds_since(dvl_time_, now) <= dvl_timeout_;
    const bool depth_ok = seconds_since(depth_time_, now) <= depth_timeout_;
    if (dvl_ok != dvl_ok_) {
        if (dvl_ok) {
            RCLCPP_INFO(this->get_logger(), "DVL back: x/y feedback on");
        } else {
            RCLCPP_WARN(this->get_logger(), "no DVL for %.1f s: x/y feedback off until it returns", dvl_timeout_);
        }
    }
    if (depth_ok != depth_ok_) {
        if (depth_ok) {
            RCLCPP_INFO(this->get_logger(), "depth back: z feedback on");
        } else {
            RCLCPP_ERROR(this->get_logger(), "no depth for %.1f s: z feedback off, floating up", depth_timeout_);
        }
    }
    dvl_ok_ = dvl_ok;
    depth_ok_ = depth_ok;
    const bool altitude_ok = seconds_since(altitude_time_, now) <= altitude_timeout_;
    if (generator_.altitude() && !altitude_ok) {
        RCLCPP_WARN(this->get_logger(), "lost DVL altitude: holding depth instead");
        generator_.drop_altitude(state_.position.z());
    }

    std::array<bool, NUM_DOF> active{};
    for (int axis = 0; axis < NUM_DOF; ++axis) {
        const bool feedback = axis < 2 ? dvl_ok : axis == Z ? depth_ok : true;
        generator_.set_passive(axis, !feedback || !controllable_[axis]);
        active[axis] = feedback && controllable_[axis] && generator_.mode(axis) != AxisMode::EFFORT;
    }

    Vector6d pose = measured_pose();
    if (generator_.altitude()) {
        pose[Z] = altitude_;
    }
    generator_.update(now.seconds(), dt, pose, limits_);
    const ControlReference reference{generator_.position(), generator_.velocity(), generator_.acceleration()};

    Vector6d wrench = controller_.update(state_, reference, active, dt, altitude_, generator_.altitude());
    if (!depth_ok) {
        // Stop holding against buoyancy so the vehicle rises.
        wrench.head<3>() -=
            gains_.restoring_feedforward * model_.restoring_hold(state_.orientation, state_.orientation).head<3>();
    }
    // After the buoyancy hold comes off, so an EFFORT axis gets exactly its effort.
    for (int axis = 0; axis < NUM_DOF; ++axis) {
        if (generator_.mode(axis) == AxisMode::EFFORT) {
            wrench[axis] = generator_.effort(axis);
        }
    }
    // Nothing non-finite may reach the thrusters, or the integrators through
    // achieved(), where it would stay. Every input is checked, so getting here
    // is a bug.
    if (!wrench.allFinite()) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "non-finite wrench: thrusters off, integrators reset");
        controller_.reset();
        publish_zero_thrusters();
        publish_diagnostics(now, last_output_, Vector6d::Zero());
        return;
    }

    const ThrusterOutput output = drive_thrusters(allocator_, wrench, thruster_settings_, last_command_, dt);
    controller_.achieved(output.achieved, dt);
    for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
        std_msgs::msg::Float64 msg;
        msg.data = output.command[i];
        thruster_pubs_[i]->publish(msg);
    }
    last_command_ = output.command;
    last_output_ = output;
    last_wrench_ = wrench;
    publish_diagnostics(now, output, wrench);
}

void SubControl::publish_diagnostics(const rclcpp::Time& now, const ThrusterOutput& output, const Vector6d& wrench) {
    const Vector6d pose = [&] {
        Vector6d p = measured_pose();
        if (generator_.altitude()) {
            p[Z] = altitude_;
        }
        return p;
    }();
    const Vector6d target = generator_.target();
    const Eigen::Matrix3d R = state_.orientation.toRotationMatrix();

    // The mission's view: how far from the commanded target, per axis.
    sub_control_interfaces::msg::Error error;
    error.header.stamp = now;
    Eigen::Vector3d velocity_world = generator_.velocity().head<3>();
    Eigen::Vector3d rpy_target = pose.tail<3>();
    Eigen::Vector3d rpy_rates = generator_.velocity().tail<3>();
    if (generator_.mode(X) == AxisMode::VELOCITY) {
        const Eigen::Vector2d v{generator_.velocity_target(X), generator_.velocity_target(Y)};
        velocity_world.head<2>() = generator_.velocity_frame() == VelocityFrame::HEADING
                                       ? Eigen::Rotation2Dd(generator_.position()[YAW]) * v
                                       : v;
    }
    if (generator_.mode(Z) == AxisMode::VELOCITY) {
        velocity_world.z() = generator_.velocity_target(Z);
    }
    for (int i = 0; i < 3; ++i) {
        if (seeded_ && generator_.mode(i) == AxisMode::POSITION) {
            error.pos_error[i] = target[i] - pose[i];
        }
        if (generator_.mode(ROLL + i) == AxisMode::POSITION) {
            rpy_target[i] = target[ROLL + i];
        } else if (generator_.mode(ROLL + i) == AxisMode::VELOCITY) {
            rpy_rates[i] = generator_.velocity_target(ROLL + i);
        }
    }
    const Eigen::Vector3d att_error =
        seeded_ ? attitude_error(quaternion_from_rpy(rpy_target), state_.orientation) : Eigen::Vector3d::Zero();
    const Eigen::Vector3d vel_error = R.transpose() * velocity_world - state_.linear_velocity;
    const Eigen::Vector3d angvel_error = euler_rates_to_body(pose.tail<3>(), rpy_rates) - state_.angular_velocity;
    for (int i = 0; i < 3; ++i) {
        error.vel_error[i] = vel_error[i];
        error.att_error[i] = att_error[i];
        error.angvel_error[i] = angvel_error[i];
    }
    error_pub_->publish(error);

    sub_control_interfaces::msg::ControllerStatus status;
    status.header.stamp = now;
    const Vector6d reference_position = generator_.position();
    const Vector6d reference_velocity = generator_.velocity();
    const Vector6d reference_acceleration = generator_.acceleration();
    const Vector6d integral = controller_.integral();
    const Vector6d disturbance = controller_.disturbance();
    for (int axis = 0; axis < NUM_DOF; ++axis) {
        status.mode[axis] = static_cast<uint8_t>(generator_.mode(axis));
        const bool angle = axis >= ROLL;
        status.target[axis] = angle ? wrap_angle(target[axis]) : target[axis];
        status.reference_position[axis] = angle ? wrap_angle(reference_position[axis]) : reference_position[axis];
        status.reference_velocity[axis] = reference_velocity[axis];
        status.reference_acceleration[axis] = reference_acceleration[axis];
        status.wrench_command[axis] = wrench[axis];
        status.wrench_achieved[axis] = output.achieved[axis];
        status.integral[axis] = integral[axis];
        status.disturbance[axis] = disturbance[axis];
        status.axis_controllable[axis] = controllable_[axis];
    }
    for (int t = 0; t < NUM_THRUSTERS; ++t) {
        status.thruster_force[t] = output.force[t];
        status.thruster_command[t] = output.command[t];
        status.thruster_saturated[t] = output.saturated[t];
        status.thruster_health[t] = thruster_settings_.health[t];
    }
    status.altitude = generator_.altitude();
    status.killed = killed_;
    status.failsafe = failsafe_;
    status.dvl_ok = dvl_ok_;
    status.depth_ok = depth_ok_;
    status.altitude_ok = seconds_since(altitude_time_, now) <= altitude_timeout_;
    status.state_age = std::min(seconds_since(odom_time_, now), 1e9);
    status_pub_->publish(status);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubControl>());
    rclcpp::shutdown();
    return 0;
}
