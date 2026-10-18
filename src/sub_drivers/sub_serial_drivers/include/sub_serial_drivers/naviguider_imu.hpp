#pragma once

#include <array>
#include <cstdint>
#include <numbers>
#include <optional>
#include <string_view>

#include <sensor_msgs/msg/imu.hpp>

#include "sub_serial_drivers/serial_node.hpp"

namespace sub_serial_drivers {

// The NaviGuider's UART protocol (NaviGuider UART user manual V1.7). In
// non-verbose mode (V0) each line is "<timestamp>, <sensor ID>, <values>...",
// and lines end with LF CR.
namespace naviguider {

// Virtual sensor IDs (manual, Table 4-2).
constexpr int ACCELEROMETER = 1;          // X, Y, Z [m/s^2], accuracy
constexpr int GYROSCOPE = 4;              // X, Y, Z [rad/s], accuracy
constexpr int GAME_ROTATION_VECTOR = 15;  // 6-DOF accel+gyro quaternion: QX, QY, QZ, QW, accuracy

// The accelerometer's and the gyroscope's accuracy is the calibration status of
// the module's SENtral-A2 fusion hub, on Android's scale (SENSOR_STATUS_*):
// 0 unreliable, 1 low, 2 medium, 3 high. A still gyro reports 2 or 3. The manual
// does not define the game rotation vector's: it is in the diagnostics, unused.
constexpr double ACCURACY_MEDIUM = 2.0;

// Measured still on the bench (June 2026): roll, pitch, yaw [rad^2].
constexpr std::array<double, 9> ORIENTATION_COVARIANCE = {2.071706e-07,  -9.010312e-08, -1.519626e-08,  //
                                                          -9.010312e-08, 1.985541e-07,  4.694574e-09,   //
                                                          -1.519626e-08, 4.694574e-09,  3.373885e-09};
// Measured still on the bench (June 2026), mostly at accuracy 3 [(rad/s)^2].
constexpr std::array<double, 9> GYRO_COVARIANCE = {2.234328e-06,  3.029158e-07, -2.190460e-08,  //
                                                   3.029158e-07,  1.560898e-05, 2.350362e-06,   //
                                                   -2.190460e-08, 2.350362e-06, 2.446427e-06};
// An uncalibrated MEMS gyro's zero-rate offset is typically a few deg/s [rad/s].
constexpr double UNCALIBRATED_GYRO_BIAS = 3.0 * std::numbers::pi / 180.0;
// The gyro covariance below ACCURACY_MEDIUM: the offset's variance added to the diagonal.
constexpr std::array<double, 9> GYRO_COVARIANCE_UNCALIBRATED = [] {
    std::array<double, 9> covariance = GYRO_COVARIANCE;
    for (const int i : {0, 4, 8}) {
        covariance[i] += UNCALIBRATED_GYRO_BIAS * UNCALIBRATED_GYRO_BIAS;
    }
    return covariance;
}();
constexpr std::array<double, 9> ACCEL_COVARIANCE = {1e-2, 0.0, 0.0, 0.0, 1e-2, 0.0, 0.0, 0.0, 1e-2};
// The covariance of a field that is not measured (sensor_msgs/Imu).
constexpr std::array<double, 9> NOT_PROVIDED = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

struct Sample {
    int sensor;
    // As many as the sensor sends, the last of them its accuracy.
    std::array<double, 5> values;
    double accuracy;
};

// A line from one of those three sensors. Nothing for anything else: verbose
// text, a meta event, another sensor, fewer values than the sensor sends (bytes
// lost on the link), or a value that is not a finite number.
std::optional<Sample> parse_line(std::string_view line);

}  // namespace naviguider

// The NaviGuider IMU over USB serial, as sensor_msgs/Imu on imu/data (ENU).
//
// Each gyroscope sample is published with the latest orientation and
// acceleration, but only while both are fresh (`max_sample_age`): a sensor that
// stops reporting makes imu/data stop, which sub_control treats as losing
// attitude, rather than repeating its last value. Nothing published for
// `data_timeout` (the module rebooted, or missed its start commands while it was
// booting) reopens the port and restarts the sensors.
//
// Below medium accuracy the gyro's bias is not trusted: the published rate
// covariance includes a typical uncalibrated offset, and the diagnostics warn.
class NaviGuiderImu : public SerialNode {
   public:
    explicit NaviGuiderImu(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~NaviGuiderImu() override;

   private:
    struct Sensor {
        int id;
        const char* name;
        std::int64_t rate;  // [Hz]; 0 disables it
        // Since the sensors were last started.
        std::optional<SteadyTime> last_heard;
        std::optional<double> accuracy;
    };

    bool handle_line(std::string_view line) override;
    void on_connect() override;
    // Publishes the message unless the acceleration or orientation in it is stale. Whether it did.
    bool publish(SteadyTime now);
    Sensor& sensor(int id);
    void diagnose(diagnostic_updater::DiagnosticStatusWrapper& status);

    std::array<Sensor, 3> sensors_;
    std::chrono::duration<double> max_sample_age_;

    sensor_msgs::msg::Imu msg_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> imu_diagnostic_;
    int stale_skipped_{0};
};

}  // namespace sub_serial_drivers
