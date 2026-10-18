#include "sub_serial_drivers/naviguider_imu.hpp"

#include <algorithm>
#include <format>
#include <stdexcept>

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

namespace {

// The longest line, a quaternion (the manual's example) [bytes]. At 8N1 a byte
// is 10 bits on the wire.
constexpr int LINE_BYTES = 70;

}  // namespace

NaviGuiderImu::NaviGuiderImu(const rclcpp::NodeOptions& options)
    : SerialNode("naviguider_imu", options, "/dev/ttyUSB0"),
      // Output rates [Hz] (0-400). Each gyro sample is one message, so the gyro
      // cannot be disabled; 0 disables the accelerometer or the orientation, whose
      // covariance is then marked not provided. At 115200 baud the three fit up to
      // about 50 Hz each, the gyro and orientation alone about 80 Hz.
      sensors_{{{naviguider::ACCELEROMETER, "accelerometer", declare_parameter("accel_rate", 50), {}, {}},
                {naviguider::GYROSCOPE, "gyro", declare_parameter("gyro_rate", 50), {}, {}},
                {naviguider::GAME_ROTATION_VECTOR, "orientation", declare_parameter("orientation_rate", 50), {}, {}}}},
      // Oldest accelerometer or orientation sample a gyro sample is published with [s].
      max_sample_age_(declare_seconds("max_sample_age", 0.2)) {
    if (sensor(naviguider::GYROSCOPE).rate <= 0) {
        throw std::invalid_argument("gyro_rate must be positive: each gyro sample is a message");
    }
    std::int64_t lines = 0;
    for (const Sensor& s : sensors_) {
        lines += std::max<std::int64_t>(s.rate, 0);
    }
    const int baud = static_cast<int>(get_parameter("baud").as_int());
    if (lines * LINE_BYTES * 10 > baud) {
        RCLCPP_WARN(get_logger(), "%ld lines/s may not fit through %d baud: the module would drop samples", lines,
                    baud);
    }

    msg_.header.frame_id = declare_parameter<std::string>("frame_id", "imu_link");
    msg_.orientation_covariance = sensor(naviguider::GAME_ROTATION_VECTOR).rate > 0 ? naviguider::ORIENTATION_COVARIANCE
                                                                                    : naviguider::NOT_PROVIDED;
    msg_.angular_velocity_covariance = naviguider::GYRO_COVARIANCE;
    msg_.linear_acceleration_covariance =
        sensor(naviguider::ACCELEROMETER).rate > 0 ? naviguider::ACCEL_COVARIANCE : naviguider::NOT_PROVIDED;
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
    }
    // Commands are case-sensitive and end with a carriage return.
    std::string commands =
        "M1\r"   // standard mounting
        "V0\r"   // non-verbose: numeric sensor IDs in the stream
        "J4\r";  // ENU orientation frame (ROS convention)
    // Disabled ones too: one left running by an earlier session would only crowd the link.
    for (const Sensor& s : sensors_) {
        commands += std::format("s {},{}\r", s.id, std::max<std::int64_t>(s.rate, 0));
    }
    write(commands);
}

bool NaviGuiderImu::handle_line(std::string_view line) {
    const auto sample = naviguider::parse_line(line);
    if (!sample || sensor(sample->sensor).rate <= 0) {
        return false;
    }
    const auto& v = sample->values;
    if (sample->sensor == naviguider::GAME_ROTATION_VECTOR) {
        // An all-zero quaternion is not a rotation; normalizing it downstream gives NaN.
        if (v[0] == 0.0 && v[1] == 0.0 && v[2] == 0.0 && v[3] == 0.0) {
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
    return sample->sensor == naviguider::GYROSCOPE && publish(now);
}

bool NaviGuiderImu::publish(SteadyTime now) {
    // Until the accelerometer and the orientation have reported since the sensors
    // started, the message's acceleration and orientation are unmeasured defaults;
    // once one stops, they are stale. Either way, nothing goes out.
    for (const Sensor& s : sensors_) {
        if (s.id != naviguider::GYROSCOPE && s.rate > 0 && (!s.last_heard || now - *s.last_heard > max_sample_age_)) {
            ++stale_skipped_;
            return false;
        }
    }
    msg_.header.stamp = this->now();
    imu_pub_->publish(msg_);
    imu_diagnostic_->tick();
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
