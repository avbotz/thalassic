#ifndef SUB_SERIAL_DRIVERS_NAVIGUIDER_IMU_HPP_
#define SUB_SERIAL_DRIVERS_NAVIGUIDER_IMU_HPP_

#include <memory>
#include <string>
#include <string_view>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "sub_serial_drivers/serial_port.hpp"

// configure  -> open the serial port, build the Imu template, create publisher
// activate   -> configure the module (non-verbose output, ENU frame), start the
//               accelerometer / gyroscope / rotation-vector virtual sensors and
//               begin polling the port
// deactivate -> stop the virtual sensors and stop polling
// cleanup    -> close the serial port
// shutdown   -> stop the sensors and close the serial port
//
// Parses Timestamp,SensorID,Value1,Value2,...
class NaviGuiderIMUDriver : public rclcpp_lifecycle::LifecycleNode {
   public:
    explicit NaviGuiderIMUDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~NaviGuiderIMUDriver();

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

   private:
    static constexpr int SENSOR_ACCELEROMETER = 1;          // payload: X, Y, Z, accuracy  (m/s^2)
    static constexpr int SENSOR_GYROSCOPE = 4;              // payload: X, Y, Z, accuracy  (rad/s)
    static constexpr int SENSOR_GAME_ROTATION_VECTOR = 15;  // 6-DOF accel+gyro: QX, QY, QZ, QW, accuracy

    void start_sensors();
    void stop_sensors();
    void poll_serial();
    void handle_line(std::string_view line);

    std::unique_ptr<SerialPort> serial_;
    std::string rx_buffer_;
    std::string frame_id_;
    bool is_active_{false};

    int accel_rate_{0};
    int gyro_rate_{0};
    int orientation_rate_{0};

    sensor_msgs::msg::Imu imu_msg_;

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr poll_timer_;
};

#endif  // SUB_SERIAL_DRIVERS_NAVIGUIDER_IMU_HPP_
