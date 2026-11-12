#include "sub_serial_drivers/naviguider_imu.hpp"

#include <algorithm>
#include <cmath>
#include <format>

#include <rclcpp_components/register_node_macro.hpp>

namespace sub_serial_drivers {

namespace naviguider {

std::optional<Sample> parse_line(std::string_view line) {
    const std::vector<std::string_view> fields = split(line, ',');
    const auto sensor = fields.size() > 1 ? parse_number<int>(fields[1]) : std::nullopt;
    // Values after the timestamp and the sensor ID (manual, "Payload size").
    const size_t size = sensor == ACCELEROMETER || sensor == GYROSCOPE ? 4 : sensor == GAME_ROTATION_VECTOR ? 5 : 0;
    if (size == 0 || fields.size() < 2 + size) {
        return std::nullopt;
    }
    Sample sample{*sensor, {}, 0.0};
    for (size_t i = 0; i < size; ++i) {
        const auto value = parse_number<double>(fields[2 + i]);
        if (!value) {
            return std::nullopt;
        }
        sample.values[i] = *value;
    }
    sample.accuracy = sample.values[size - 1];
    return sample;
}

}  // namespace naviguider

NaviGuiderImu::NaviGuiderImu(const rclcpp::NodeOptions& options)
    : SerialNode("naviguider_imu", options),
      // Read-only, so the listener is not needed after this.
      params_(naviguider_imu::ParamListener(get_node_parameters_interface(), get_logger()).get_params()),
      sensors_{{{naviguider::ACCELEROMETER, "accelerometer", params_.accel_rate, {}, {}},
                {naviguider::GYROSCOPE, "gyro", params_.gyro_rate, {}, {}},
                {naviguider::GAME_ROTATION_VECTOR, "orientation", params_.orientation_rate, {}, {}}}} {
    std::int64_t bytes_per_second = 0;
    for (const Sensor& s : sensors_) {
        bytes_per_second += s.rate * naviguider::line_bytes(s.id);
    }
    // At 8N1 a byte is 10 bits on the wire.
    const auto baud = get_parameter("baud").as_int();
    if (bytes_per_second * 10 > naviguider::LINK_BUDGET * static_cast<double>(baud)) {
        RCLCPP_WARN(get_logger(),
                    "these rates need up to %ld bytes/s, more than %g%% of %ld baud: the module would drop samples",
                    bytes_per_second, naviguider::LINK_BUDGET * 100.0, baud);
    }

    msg_.header.frame_id = params_.frame_id;
    msg_.angular_velocity_covariance = naviguider::GYRO_COVARIANCE;
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::QoS(10).reliable());

    diagnostics_->setHardwareID("naviguider");
    diagnostics_->add("imu", this, &NaviGuiderImu::diagnose);
    // One message per gyro sample.
    imu_diagnostic_ = std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(
        "imu/data", *diagnostics_, min_rate(static_cast<double>(sensor(naviguider::GYROSCOPE).rate)));
}

NaviGuiderImu::~NaviGuiderImu() {
    // A zero sample rate disables a sensor (manual, "Sample_Rate").
    std::string commands;
    for (const Sensor& s : sensors_) {
        commands += std::format("s {},0\r", s.id);
    }
    write(commands);
}

void NaviGuiderImu::on_connect() {
    // Nothing the sensors said before this counts.
    for (Sensor& s : sensors_) {
        s.last_heard.reset();
        s.accuracy.reset();
        s.fresh = false;
    }
    // Commands are case-sensitive and end with a carriage return. Each one is sent
    // even if it is the default: the module keeps settings from an earlier session.
    std::string commands =
        "M1\r"   // standard mounting
        "V0\r"   // non-verbose: numeric sensor IDs in the stream, the shortest lines
        "J4\r"   // ENU orientation frame (ROS convention)
        "m0\r"   // no meta events (stillness, calibration changes): nothing reads them
        "D1\r";  // sensor data shown, in case a terminal session turned it off
    // Disabled ones too: one left running by an earlier session would only crowd the link.
    for (const Sensor& s : sensors_) {
        commands += std::format("s {},{}\r", s.id, s.rate);
    }
    write(commands);
}

bool NaviGuiderImu::handle_record(std::string_view line) {
    const auto sample = naviguider::parse_line(trim(line));
    if (!sample || sensor(sample->sensor).rate <= 0) {
        return false;
    }
    const auto& v = sample->values;
    if (sample->sensor == naviguider::GAME_ROTATION_VECTOR) {
        // Not a rotation: all zero (as the module sends while starting; normalizing it
        // downstream gives NaN), or digits lost on the link.
        const double norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
        if (std::abs(norm - 1.0) > naviguider::MAX_QUATERNION_NORM_ERROR) {
            return false;
        }
        msg_.orientation.x = v[0];
        msg_.orientation.y = v[1];
        msg_.orientation.z = v[2];
        msg_.orientation.w = v[3];
    } else if (sample->sensor == naviguider::ACCELEROMETER) {
        msg_.linear_acceleration.x = v[0];
        msg_.linear_acceleration.y = v[1];
        msg_.linear_acceleration.z = v[2];
    } else {
        // Already rad/s, as sensor_msgs/Imu wants.
        msg_.angular_velocity.x = v[0];
        msg_.angular_velocity.y = v[1];
        msg_.angular_velocity.z = v[2];
        msg_.angular_velocity_covariance = sample->accuracy >= naviguider::ACCURACY_MEDIUM
                                               ? naviguider::GYRO_COVARIANCE
                                               : naviguider::GYRO_COVARIANCE_UNCALIBRATED;
    }
    const SteadyTime now = std::chrono::steady_clock::now();
    Sensor& s = sensor(sample->sensor);
    s.last_heard = now;
    s.accuracy = sample->accuracy;
    s.fresh = true;
    return sample->sensor == naviguider::GYROSCOPE && publish(now);
}

bool NaviGuiderImu::publish(SteadyTime now) {
    // Until the accelerometer and the orientation have reported since the sensors
    // started, the message's acceleration and orientation are unmeasured defaults;
    // once one stops, they are stale. Either way, nothing goes out.
    const std::chrono::duration<double> max_age(params_.max_sample_age);
    for (const Sensor& s : sensors_) {
        if (s.id != naviguider::GYROSCOPE && s.rate > 0 && (!s.last_heard || now - *s.last_heard > max_age)) {
            ++stale_skipped_;
            return false;
        }
    }
    // A disabled sensor is never fresh, so its field is never provided.
    msg_.orientation_covariance =
        sensor(naviguider::GAME_ROTATION_VECTOR).fresh ? naviguider::ORIENTATION_COVARIANCE : naviguider::NOT_PROVIDED;
    msg_.linear_acceleration_covariance =
        sensor(naviguider::ACCELEROMETER).fresh ? naviguider::ACCEL_COVARIANCE : naviguider::NOT_PROVIDED;
    msg_.header.stamp = this->now();
    imu_pub_->publish(msg_);
    imu_diagnostic_->tick();
    for (Sensor& s : sensors_) {
        s.fresh = false;
    }
    return true;
}

NaviGuiderImu::Sensor& NaviGuiderImu::sensor(int id) { return *std::ranges::find(sensors_, id, &Sensor::id); }

void NaviGuiderImu::diagnose(diagnostic_updater::DiagnosticStatusWrapper& status) {
    status.add("held back for a stale accelerometer or orientation", stale_skipped_);
    for (const Sensor& s : sensors_) {
        if (s.rate > 0) {
            status.add(std::format("last {}", s.name), since(s.last_heard));
            status.add(std::format("{} accuracy", s.name), s.accuracy ? std::format("{:g}", *s.accuracy) : "none yet");
        }
    }
    // Whether imu/data keeps coming is its topic status's to report.
    const std::optional<double> gyro_accuracy = sensor(naviguider::GYROSCOPE).accuracy;
    if (!gyro_accuracy) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "no gyro samples yet");
    } else if (*gyro_accuracy < naviguider::ACCURACY_MEDIUM) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                       std::format("gyro calibration low (accuracy {:g} of 3)", *gyro_accuracy));
    } else {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                       std::format("gyro calibrated (accuracy {:g} of 3)", *gyro_accuracy));
    }
}

}  // namespace sub_serial_drivers

RCLCPP_COMPONENTS_REGISTER_NODE(sub_serial_drivers::NaviGuiderImu)
