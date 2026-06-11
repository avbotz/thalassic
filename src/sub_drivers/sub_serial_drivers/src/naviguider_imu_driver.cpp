#include "sub_serial_drivers/naviguider_imu_driver.hpp"

#include <charconv>
#include <format>
#include <vector>

using namespace std::chrono_literals;

static constexpr size_t MAX_RX_BUFFER = 4096;

static std::vector<std::string_view> split_csv(std::string_view line) {
    std::vector<std::string_view> out;
    size_t pos = 0;
    while (pos <= line.size()) {
        size_t comma = line.find(',', pos);
        if (comma == std::string_view::npos) {
            comma = line.size();
        }
        std::string_view tok = line.substr(pos, comma - pos);
        while (!tok.empty() && (tok.front() == ' ' || tok.front() == '\t')) {
            tok.remove_prefix(1);
        }
        while (!tok.empty() && (tok.back() == ' ' || tok.back() == '\t' || tok.back() == '\r')) {
            tok.remove_suffix(1);
        }
        out.push_back(tok);
        pos = comma + 1;
    }
    return out;
}

static bool parse_int(std::string_view s, int& out) {
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
    return ec == std::errc() && ptr == s.data() + s.size();
}

static bool parse_double(std::string_view s, double& out) {
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
    return ec == std::errc() && ptr == s.data() + s.size();
}

NaviGuiderIMUDriver::NaviGuiderIMUDriver(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("naviguider_imu", options) {
    this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud", 115200);
    this->declare_parameter<std::string>("frame_id", "imu_link");
    // Per-sensor output rates in Hz. 0 disables a sensor. Aggregate data rate must not exceed 1200 Hz.
    this->declare_parameter<int>("accel_rate", 100);
    this->declare_parameter<int>("gyro_rate", 100);
    this->declare_parameter<int>("orientation_rate", 100);
}

NaviGuiderIMUDriver::~NaviGuiderIMUDriver() { stop_sensors(); }

NaviGuiderIMUDriver::CallbackReturn NaviGuiderIMUDriver::on_configure(const rclcpp_lifecycle::State&) {
    const std::string device = this->get_parameter("device").as_string();
    const int baud = this->get_parameter("baud").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    accel_rate_ = this->get_parameter("accel_rate").as_int();
    gyro_rate_ = this->get_parameter("gyro_rate").as_int();
    orientation_rate_ = this->get_parameter("orientation_rate").as_int();

    try {
        serial_ = std::make_unique<SerialPort>(device, baud);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "could not open serial: %s", e.what());
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(this->get_logger(), "naviguider_imu connected to %s @ %d baud", device.c_str(), baud);

    imu_msg_.orientation_covariance = {6.801009e-04, -2.957911e-04, -4.988638e-05, -2.957911e-04, 6.518145e-04,
                                       1.541138e-05, -4.988638e-05, 1.541138e-05,  1.107581e-05};
    imu_msg_.angular_velocity_covariance = {1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4};
    imu_msg_.linear_acceleration_covariance = {1e-2, 0.0, 0.0, 0.0, 1e-2, 0.0, 0.0, 0.0, 1e-2};

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());

    poll_timer_ = this->create_wall_timer(5ms, [this]() {
        if (is_active_) {
            poll_serial();
        }
    });

    return CallbackReturn::SUCCESS;
}

NaviGuiderIMUDriver::CallbackReturn NaviGuiderIMUDriver::on_activate(const rclcpp_lifecycle::State& state) {
    LifecycleNode::on_activate(state);

    rx_buffer_.clear();
    start_sensors();
    is_active_ = true;

    RCLCPP_INFO(this->get_logger(), "naviguider_imu active");
    return CallbackReturn::SUCCESS;
}

NaviGuiderIMUDriver::CallbackReturn NaviGuiderIMUDriver::on_deactivate(const rclcpp_lifecycle::State& state) {
    LifecycleNode::on_deactivate(state);

    is_active_ = false;
    stop_sensors();

    RCLCPP_INFO(this->get_logger(), "naviguider_imu deactivated");
    return CallbackReturn::SUCCESS;
}

NaviGuiderIMUDriver::CallbackReturn NaviGuiderIMUDriver::on_cleanup(const rclcpp_lifecycle::State&) {
    poll_timer_.reset();
    imu_pub_.reset();
    serial_.reset();
    rx_buffer_.clear();
    return CallbackReturn::SUCCESS;
}

NaviGuiderIMUDriver::CallbackReturn NaviGuiderIMUDriver::on_shutdown(const rclcpp_lifecycle::State&) {
    is_active_ = false;
    stop_sensors();

    poll_timer_.reset();
    imu_pub_.reset();
    serial_.reset();
    rx_buffer_.clear();
    return CallbackReturn::SUCCESS;
}

void NaviGuiderIMUDriver::start_sensors() {
    if (!serial_) {
        return;
    }
    // Commands are case-sensitive and terminated by a carriage return (0x0D).
    serial_->write("V0\r");  // non-verbose: emit numeric sensor IDs in the stream
    serial_->write("J4\r");  // ENU orientation frame (ROS convention)

    if (accel_rate_ > 0) {
        serial_->write(std::format("s {},{}\r", SENSOR_ACCELEROMETER, accel_rate_));
    }
    if (gyro_rate_ > 0) {
        serial_->write(std::format("s {},{}\r", SENSOR_GYROSCOPE, gyro_rate_));
    }
    if (orientation_rate_ > 0) {
        serial_->write(std::format("s {},{}\r", SENSOR_GAME_ROTATION_VECTOR, orientation_rate_));
    }
}

void NaviGuiderIMUDriver::stop_sensors() {
    if (!serial_) {
        return;
    }
    // A zero sample rate disables the virtual sensor (manual, "Sample_Rate" key).
    serial_->write(std::format("s {},0\r", SENSOR_ACCELEROMETER));
    serial_->write(std::format("s {},0\r", SENSOR_GYROSCOPE));
    serial_->write(std::format("s {},0\r", SENSOR_GAME_ROTATION_VECTOR));
}

void NaviGuiderIMUDriver::poll_serial() {
    if (!serial_->read_available(rx_buffer_)) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "serial read error / device disconnected");
        return;
    }

    size_t start_pos = 0;
    for (size_t i = 0; i < rx_buffer_.size(); ++i) {
        if (rx_buffer_[i] == '\n') {
            handle_line(std::string_view(rx_buffer_.data() + start_pos, i - start_pos));
            start_pos = i + 1;
        }
    }
    rx_buffer_.erase(0, start_pos);

    if (rx_buffer_.size() > MAX_RX_BUFFER) {
        rx_buffer_.clear();
    }
}

void NaviGuiderIMUDriver::handle_line(std::string_view line) {
    const std::vector<std::string_view> fields = split_csv(line);
    if (fields.size() < 2) {
        return;
    }

    // fields[0] is the timestamp, fields[1] the numeric sensor ID. Lines that do
    // not start with "<timestamp>,<id>" (e.g. residual verbose text or meta
    // events) simply fail the integer parse and are ignored.
    int sensor_id;
    if (!parse_int(fields[1], sensor_id)) {
        return;
    }

    auto value = [&](size_t idx, double& out) { return idx < fields.size() && parse_double(fields[idx], out); };

    switch (sensor_id) {
        case SENSOR_ACCELEROMETER: {
            double x, y, z;
            if (value(2, x) && value(3, y) && value(4, z)) {
                imu_msg_.linear_acceleration.x = x;
                imu_msg_.linear_acceleration.y = y;
                imu_msg_.linear_acceleration.z = z;

                imu_msg_.header.stamp = this->now();
                imu_msg_.header.frame_id = frame_id_;
                imu_pub_->publish(imu_msg_);
            }
            break;
        }
        case SENSOR_GYROSCOPE: {
            double x, y, z;
            if (value(2, x) && value(3, y) && value(4, z)) {
                imu_msg_.angular_velocity.x = x;
                imu_msg_.angular_velocity.y = y;
                imu_msg_.angular_velocity.z = z;
            }
            break;
        }
        case SENSOR_GAME_ROTATION_VECTOR: {
            double qx, qy, qz, qw;
            if (value(2, qx) && value(3, qy) && value(4, qz) && value(5, qw)) {
                imu_msg_.orientation.x = qx;
                imu_msg_.orientation.y = qy;
                imu_msg_.orientation.z = qz;
                imu_msg_.orientation.w = qw;
            }
            break;
        }
        default:
            break;
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto node = std::make_shared<NaviGuiderIMUDriver>();

    exec.add_node(node->get_node_base_interface());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
