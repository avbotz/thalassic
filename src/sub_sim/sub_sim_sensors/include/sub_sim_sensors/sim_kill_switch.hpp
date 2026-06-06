#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

// Sub is kept killed for the first `off_delay` seconds, then released (true -> false on /kill_switch).
// After, what is published on /sim/kill_switch is transparently forwarded to /kill_switch.
class SimKillSwitch : public rclcpp::Node {
   public:
    SimKillSwitch();

   private:
    void sim_callback(const std_msgs::msg::Bool& msg);
    void release();
    void publish(bool killed);

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sim_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr startup_timer_;

    bool startup_done_{false};
};

int main(int argc, char* argv[]);
