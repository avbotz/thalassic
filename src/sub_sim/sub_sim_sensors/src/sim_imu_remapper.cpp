#include "sub_sim_sensors/sim_imu_remapper.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Include tf2 headers for robust quaternion and matrix math
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <array>

namespace {
// Helper function to transform 3x3 covariance matrices: C_new = T * C_old * T^T
std::array<double, 9> transformCovariance(const std::array<double, 9>& cov_in, const tf2::Matrix3x3& T) {
    // Pass through if uninitialized (-1.0 on first element)
    if (cov_in[0] == -1.0) {
        return cov_in;
    }

    // Pass through if completely empty/zero
    bool is_zero = true;
    for (double v : cov_in) {
        if (v != 0.0) {
            is_zero = false;
            break;
        }
    }
    if (is_zero) {
        return cov_in;
    }

    // Map std::array to tf2::Matrix3x3
    tf2::Matrix3x3 C_old(cov_in[0], cov_in[1], cov_in[2], cov_in[3], cov_in[4], cov_in[5], cov_in[6], cov_in[7],
                         cov_in[8]);

    // Compute transformed covariance
    tf2::Matrix3x3 C_new = T * C_old * T.transpose();

    // Return as flat std::array
    return {C_new[0][0], C_new[0][1], C_new[0][2], C_new[1][0], C_new[1][1],
            C_new[1][2], C_new[2][0], C_new[2][1], C_new[2][2]};
}
}  // anonymous namespace

SimIMURemapper::SimIMURemapper(const rclcpp::NodeOptions& options) : Node("sim_imu_remapper", options) {
    this->declare_parameter("imu_link", "imu_link");
    imu_link_ = this->get_parameter("imu_link").as_string();

    subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "sim/imu", 10, [this](sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(msg); });

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
}

void SimIMURemapper::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg_ned) {
    sensor_msgs::msg::Imu msg_enu{};

    msg_enu.header = msg_ned->header;
    msg_enu.header.frame_id = imu_link_;

    // T_world: Transforms World Frame from NED to ENU (E=N_old, N=E_old, U=-D)
    static const tf2::Matrix3x3 T_world(0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0);

    // T_body: Transforms the sim IMU body frame to FLU. The IMU is rigidly
    // attached to the Stonefish base_link_ned, whose axes are X=Right, Y=Back,
    // Z=Down (NOT standard FRD). Mapping (R,B,D) -> FLU (F,L,U) gives
    // F=-Back, L=-Right, U=-Down, i.e. (x,y,z) -> (-y,-x,-z).
    static const tf2::Matrix3x3 T_body(0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0);

    // 1. Convert Linear Acceleration and Angular Velocity (Body Frame: FRD -> FLU)
    // Note: Accel and Gyro are fixed to the body, so they only use T_body!
    tf2::Vector3 accel_frd(msg_ned->linear_acceleration.x, msg_ned->linear_acceleration.y,
                           msg_ned->linear_acceleration.z);
    tf2::Vector3 accel_flu = T_body * accel_frd;
    msg_enu.linear_acceleration.x = accel_flu.x();
    msg_enu.linear_acceleration.y = accel_flu.y();
    msg_enu.linear_acceleration.z = accel_flu.z();

    tf2::Vector3 gyro_frd(msg_ned->angular_velocity.x, msg_ned->angular_velocity.y, msg_ned->angular_velocity.z);
    tf2::Vector3 gyro_flu = T_body * gyro_frd;
    msg_enu.angular_velocity.x = gyro_flu.x();
    msg_enu.angular_velocity.y = gyro_flu.y();
    msg_enu.angular_velocity.z = gyro_flu.z();

    // 2. Convert Orientation Quaternion (World: NED -> ENU & Body: FRD -> FLU)
    tf2::Quaternion q_ned(msg_ned->orientation.x, msg_ned->orientation.y, msg_ned->orientation.z,
                          msg_ned->orientation.w);

    if (q_ned.length() > 0.0) {
        tf2::Matrix3x3 R_ned(q_ned);

        // Apply Both World and Body transformations to the orientation
        tf2::Matrix3x3 R_enu = T_world * R_ned * T_body;

        tf2::Quaternion q_enu;
        R_enu.getRotation(q_enu);
        q_enu.normalize();

        msg_enu.orientation.x = q_enu.x();
        msg_enu.orientation.y = q_enu.y();
        msg_enu.orientation.z = q_enu.z();
        msg_enu.orientation.w = q_enu.w();
    } else {
        msg_enu.orientation = msg_ned->orientation;
    }

    // 3. Transform Covariance Matrices
    msg_enu.linear_acceleration_covariance = transformCovariance(msg_ned->linear_acceleration_covariance, T_body);
    msg_enu.angular_velocity_covariance = transformCovariance(msg_ned->angular_velocity_covariance, T_body);
    msg_enu.orientation_covariance = transformCovariance(msg_ned->orientation_covariance, T_world);

    publisher_->publish(msg_enu);
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimIMURemapper)
