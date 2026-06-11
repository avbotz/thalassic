#include "sub_sim_sensors/sim_imu_remapper.hpp"

#include <array>
#include <memory>

#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

namespace {

// Quaternion (x, y, z, w) Hamilton product: returns a * b.
geometry_msgs::msg::Quaternion qmul(const geometry_msgs::msg::Quaternion& a, const geometry_msgs::msg::Quaternion& b) {
    geometry_msgs::msg::Quaternion r;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    return r;
}

geometry_msgs::msg::Quaternion make_quat(double x, double y, double z, double w) {
    geometry_msgs::msg::Quaternion q;
    q.x = x;
    q.y = y;
    q.z = z;
    q.w = w;
    return q;
}

// Stonefish reports IMU orientation in NED world / FRD body. ROS (rviz,
// robot_localization) expects ENU world / FLU body. The full conversion is
//   q_enu_flu = NED_ENU * q_ned_frd * FRD_FLU
// where both static quaternions are 180-degree rotations (their own inverses):
//   NED_ENU: 180 deg about (1,1,0)/sqrt(2)  -> swaps the world reference NED<->ENU
//   FRD_FLU: 180 deg about X                -> swaps the body axes FRD<->FLU
const double kSqrtHalf = 0.7071067811865476;
const geometry_msgs::msg::Quaternion kNedEnu = make_quat(kSqrtHalf, kSqrtHalf, 0.0, 0.0);
const geometry_msgs::msg::Quaternion kFrdFlu = make_quat(1.0, 0.0, 0.0, 0.0);

}  // namespace

SimIMURemapper::SimIMURemapper() : Node("sim_imu_remapper") {
    this->declare_parameter("robot_name", "");
    robot_name_ = this->get_parameter("robot_name").as_string();

    subscriber_ =
        this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&SimIMURemapper::imu_callback, this, _1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_enu", 10);
}

void SimIMURemapper::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg_ned) {
    sensor_msgs::msg::Imu msg_enu{};

    msg_enu.header = msg_ned->header;
    // After conversion the orientation is base_link (FLU) expressed in the ENU
    // world, so stamp it as base_link. robot_localization then applies an
    // identity body transform instead of re-rotating the measurement.
    msg_enu.header.frame_id = robot_name_ + "/base_link";

    // Orientation: full NED/FRD -> ENU/FLU change of reference and body frame.
    msg_enu.orientation = qmul(kNedEnu, qmul(msg_ned->orientation, kFrdFlu));

    // Angular velocity and linear acceleration are body-frame vectors. Going
    // FRD -> FLU is a 180 deg rotation about X, i.e. negate y and z.
    msg_enu.angular_velocity.x = msg_ned->angular_velocity.x;
    msg_enu.angular_velocity.y = -msg_ned->angular_velocity.y;
    msg_enu.angular_velocity.z = -msg_ned->angular_velocity.z;

    msg_enu.linear_acceleration.x = msg_ned->linear_acceleration.x;
    msg_enu.linear_acceleration.y = -msg_ned->linear_acceleration.y;
    msg_enu.linear_acceleration.z = -msg_ned->linear_acceleration.z;

    // Covariances are diagonal (axis variances); a pure axis swap/negation
    // leaves the diagonal magnitudes unchanged, so copy them through.
    msg_enu.orientation_covariance = msg_ned->orientation_covariance;
    msg_enu.angular_velocity_covariance = msg_ned->angular_velocity_covariance;
    msg_enu.linear_acceleration_covariance = msg_ned->linear_acceleration_covariance;

    publisher_->publish(msg_enu);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimIMURemapper>());
    rclcpp::shutdown();
    return 0;
}
