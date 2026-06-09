#ifndef SUB_SERIAL_DRIVERS_SUB_LOW_HPP_
#define SUB_SERIAL_DRIVERS_SUB_LOW_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <string_view>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include "sub_serial_drivers/serial_port.hpp"

// configure  -> open the serial port, create the kill-switch publisher
// activate   -> start forwarding thruster commands and polling the board
// deactivate -> stop the thrusters and stop forwarding/polling
// cleanup    -> close the serial port
// shutdown   -> stop the thrusters and close the serial port
class SubLow : public rclcpp_lifecycle::LifecycleNode {
   public:
    explicit SubLow(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SubLow();

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

   private:
    static constexpr int NUM_THRUSTERS = 8;

    void set_thruster_power(int index, double normalized);
    void poll_serial();
    void handle_line(const std::string_view line);
    void stop_thrusters();

    std::unique_ptr<SerialPort> serial_;
    std::string rx_buffer_;
    std::atomic<bool> is_active_;

    std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, NUM_THRUSTERS> thruster_subs_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr kill_pub_;
    rclcpp::TimerBase::SharedPtr poll_timer_;
};

#endif  // SUB_SERIAL_DRIVERS_SUB_LOW_HPP_
