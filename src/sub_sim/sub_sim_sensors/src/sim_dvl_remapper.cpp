#include "sub_sim_sensors/sim_dvl_remapper.hpp"

#include <memory>

#include "marine_acoustic_msgs/msg/dvl.hpp"
#include "rclcpp/rclcpp.hpp"
#include "stonefish_ros2/msg/dvl.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

SimDVLRemapper::SimDVLRemapper() : Node("sim_dvl_remapper") {
    this->declare_parameter("robot_name", "");
    robot_name_ = this->get_parameter("robot_name").as_string();

    subscriber_ = this->create_subscription<stonefish_ros2::msg::DVL>("sim/dvl", 10, std::bind(&SimDVLRemapper::dvl_callback, this, _1));

    vel_publisher_ = this->create_publisher<marine_acoustic_msgs::msg::Dvl>("dvl", 10);
}

void SimDVLRemapper::dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr msg_stonefish) {
    marine_acoustic_msgs::msg::Dvl msg_marine{};

    msg_marine.header = msg_stonefish->header;
    msg_marine.header.frame_id = robot_name_ + "/dvl_link";
    msg_marine.velocity.x = msg_stonefish->velocity.x;
    msg_marine.velocity.y = msg_stonefish->velocity.y;
    msg_marine.velocity.z = msg_stonefish->velocity.z;
    msg_marine.altitude = msg_stonefish->altitude;

    msg_marine.velocity_covar = msg_stonefish->velocity_covariance;

    size_t num_beams{std::min(static_cast<size_t>(4), msg_stonefish->beams.size())};
    for (size_t i = 0; i < num_beams; ++i) {
        const auto& b = msg_stonefish->beams[i];
        msg_marine.range[i] = b.range;
        msg_marine.beam_velocity[i] = b.velocity;
        msg_marine.beam_quality[i] = (b.range >= 0) ? 255.0f : 0.0f;
    }

    msg_marine.beam_ranges_valid = true;
    msg_marine.beam_velocities_valid = true;
    msg_marine.num_good_beams = 4;

    msg_marine.course_gnd = std::atan2(msg_stonefish->velocity.y, msg_stonefish->velocity.x);
    msg_marine.speed_gnd = std::hypot(msg_stonefish->velocity.x, msg_stonefish->velocity.y);

    msg_marine.velocity_mode = marine_acoustic_msgs::msg::Dvl::DVL_MODE_BOTTOM;
    msg_marine.dvl_type = marine_acoustic_msgs::msg::Dvl::DVL_TYPE_PISTON;

    vel_publisher_->publish(msg_marine);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimDVLRemapper>());
    rclcpp::shutdown();
    return 0;
}
