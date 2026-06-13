/** @file mission_node.cpp
 *  @brief Ros2 node class to run mission task.
 *
 * Mission, implemented as a ros2 node.
 *
 * Here, we're using it to implement the mission task,
 * where the sub completes the competition tasks in the
 * competition pool (gate, buoy, bins, hydrophones).
 *
 * The way this is implemented is basically:
 * - The node will configure itself (init clients if needed)
 * - Wait for activation signal from master
 *   - On activate, it will start the behavior tree mission
 *   - When all tasks are finished, node will shutdown
 */

#include "sub_mission/nodes/mission.hpp"
#include "sub_mission/utils.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

namespace {

using PointCmdMsg = geometry_msgs::msg::PointStamped;
using QuaternionCmdMsg = geometry_msgs::msg::QuaternionStamped;
using VectorCmdMsg = geometry_msgs::msg::Vector3Stamped;
using PointCmdPublisher = rclcpp::Publisher<PointCmdMsg>;
using QuaternionCmdPublisher = rclcpp::Publisher<QuaternionCmdMsg>;
using VectorCmdPublisher = rclcpp::Publisher<VectorCmdMsg>;
bool mission_killed = false;
constexpr double POSITION_TOLERANCE = 0.25;
constexpr double ANGLE_TOLERANCE = 0.0872665;
constexpr const char *ALTITUDE_FRAME = "altitude";

double normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

bool portProvided(const BT::TreeNode &node, const std::string &port) {
    return node.config().input_ports.find(port) != node.config().input_ports.end();
}

QuaternionCmdMsg quaternionCommand(const rclcpp::Clock &clock, const double roll, const double pitch,
                                   const double yaw) {
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);

    QuaternionCmdMsg msg;
    msg.header.stamp = clock.now();
    msg.header.frame_id = "base_link";
    msg.quaternion.w = cr * cp * cy + sr * sp * sy;
    msg.quaternion.x = sr * cp * cy - cr * sp * sy;
    msg.quaternion.y = cr * sp * cy + sr * cp * sy;
    msg.quaternion.z = cr * cp * sy - sr * sp * cy;
    return msg;
}

PointCmdMsg positionCommand(const rclcpp::Clock &clock, const std::array<double, 3> &target,
                            const bool use_altitude) {
    PointCmdMsg msg;
    msg.header.stamp = clock.now();
    msg.header.frame_id = use_altitude ? ALTITUDE_FRAME : "base_link";
    msg.point.x = target[0];
    msg.point.y = -target[1];
    msg.point.z = use_altitude ? target[2] : -target[2];
    return msg;
}

VectorCmdMsg vectorCommand(const rclcpp::Clock &clock, const std::array<double, 3> &target) {
    VectorCmdMsg msg;
    msg.header.stamp = clock.now();
    msg.header.frame_id = "base_link";
    msg.vector.x = target[0];
    msg.vector.y = -target[1];
    msg.vector.z = -target[2];
    return msg;
}

class PosSetpointAction : public BT::StatefulActionNode {
   public:
    PosSetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                      PointCmdPublisher::SharedPtr position_publisher,
                      VectorCmdPublisher::SharedPtr velocity_publisher, rclcpp::Clock::SharedPtr clock,
                      rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          position_publisher_(position_publisher),
          velocity_publisher_(velocity_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("x", 0.0, "North/forward position or velocity setpoint"),
                BT::InputPort<double>("y", 0.0, "East/right position or velocity setpoint"),
                BT::InputPort<double>("z", 0.0, "Down/depth position or velocity setpoint"),
                BT::InputPort<bool>("velocity", false, "True for velocity mode"),
                BT::InputPort<bool>("use_altitude", false, "True when z is an altitude setpoint")};
    }

    BT::NodeStatus onStart() override {
        bool velocity = false;
        bool use_altitude = false;
        getInput("velocity", velocity);
        getInput("use_altitude", use_altitude);
        active_axes_ = {portProvided(*this, "x"), portProvided(*this, "y"), portProvided(*this, "z")};
        std::array<double, 3> target{};

        if (velocity) {
            getInput("x", target[0]);
            getInput("y", target[1]);
            getInput("z", target[2]);
            node_.last_velocity_setpoint = target;
            velocity_publisher_->publish(vectorCommand(*clock_, target));
        } else {
            target = node_.commanded_pos;
            if (active_axes_[0]) {
                getInput("x", target[0]);
            }
            if (active_axes_[1]) {
                getInput("y", target[1]);
            }
            if (active_axes_[2]) {
                getInput("z", target[2]);
            }
            node_.commanded_pos = target;
            position_publisher_->publish(positionCommand(*clock_, target, use_altitude));
        }

        start_updates_ = node_.control_error_updates;

        RCLCPP_INFO(logger_, "Published position command: velocity=%s use_altitude=%s xyz=(%.3f, %.3f, %.3f)",
                    velocity ? "true" : "false", use_altitude ? "true" : "false", target[0], target[1], target[2]);
        if (velocity) {
            return BT::NodeStatus::SUCCESS;
        }
        return checkErrors();
    }

    BT::NodeStatus onRunning() override { return checkErrors(); }

    void onHalted() override { RCLCPP_INFO(logger_, "PosSetpoint wait halted."); }

   private:
    bool freshAndWithinTolerance(const std::size_t index) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= POSITION_TOLERANCE;
    }

    BT::NodeStatus checkErrors() const {
        if (mission_killed) {
            RCLCPP_WARN(logger_, "PosSetpoint wait failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }

        if ((!active_axes_[0] || freshAndWithinTolerance(0)) &&
            (!active_axes_[1] || freshAndWithinTolerance(1)) &&
            (!active_axes_[2] || freshAndWithinTolerance(2))) {
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    VectorCmdPublisher::SharedPtr velocity_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<bool, 3> active_axes_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
};

class AttSetpointAction : public BT::StatefulActionNode {
   public:
    AttSetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                      QuaternionCmdPublisher::SharedPtr attitude_publisher,
                      VectorCmdPublisher::SharedPtr angular_velocity_publisher, rclcpp::Clock::SharedPtr clock,
                      rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          attitude_publisher_(attitude_publisher),
          angular_velocity_publisher_(angular_velocity_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("roll", 0.0, "Roll setpoint in radians"),
                BT::InputPort<double>("pitch", 0.0, "Pitch setpoint in radians"),
                BT::InputPort<double>("yaw", 0.0, "Yaw setpoint in radians"),
                BT::InputPort<bool>("velocity", false, "True for angular velocity mode")};
    }

    BT::NodeStatus onStart() override {
        bool velocity = false;
        getInput("velocity", velocity);
        active_axes_ = {portProvided(*this, "roll"), portProvided(*this, "pitch"), portProvided(*this, "yaw")};
        std::array<double, 3> target{};

        if (velocity) {
            getInput("roll", target[0]);
            getInput("pitch", target[1]);
            getInput("yaw", target[2]);
            node_.last_angvel_setpoint = target;
            angular_velocity_publisher_->publish(vectorCommand(*clock_, target));
        } else {
            target = node_.commanded_att;
            if (active_axes_[0]) {
                getInput("roll", target[0]);
            }
            if (active_axes_[1]) {
                getInput("pitch", target[1]);
            }
            if (active_axes_[2]) {
                getInput("yaw", target[2]);
                target[2] = normalizeAngle(target[2]);
            }
            node_.commanded_att = target;
            attitude_publisher_->publish(quaternionCommand(*clock_, target[0], -target[1], -target[2]));
        }

        start_updates_ = node_.control_error_updates;

        RCLCPP_INFO(logger_, "Published attitude command: velocity=%s rpy=(%.3f, %.3f, %.3f)",
                    velocity ? "true" : "false", target[0], target[1], target[2]);
        if (velocity) {
            return BT::NodeStatus::SUCCESS;
        }
        return checkErrors();
    }

    BT::NodeStatus onRunning() override { return checkErrors(); }

    void onHalted() override { RCLCPP_INFO(logger_, "AttSetpoint wait halted."); }

   private:
    bool freshAndWithinTolerance(const std::size_t index) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= ANGLE_TOLERANCE;
    }

    BT::NodeStatus checkErrors() const {
        if (mission_killed) {
            RCLCPP_WARN(logger_, "AttSetpoint wait failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }

        if ((!active_axes_[0] || freshAndWithinTolerance(6)) &&
            (!active_axes_[1] || freshAndWithinTolerance(7)) &&
            (!active_axes_[2] || freshAndWithinTolerance(8))) {
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    VectorCmdPublisher::SharedPtr angular_velocity_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<bool, 3> active_axes_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
};

class MoveRelativeAction : public BT::StatefulActionNode {
   public:
    MoveRelativeAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                       PointCmdPublisher::SharedPtr publisher, rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config), node_(node), publisher_(publisher), clock_(clock), logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("x", 0.0, "Forward offset in meters"),
                BT::InputPort<double>("y", 0.0, "Right offset in meters"),
                BT::InputPort<double>("z", 0.0, "Down/depth offset in meters")};
    }

    BT::NodeStatus onStart() override {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        getInput("x", x);
        getInput("y", y);
        getInput("z", z);

        const double yaw = node_.commanded_att[2];
        const double dx = std::cos(yaw) * x - std::sin(yaw) * y;
        const double dy = std::sin(yaw) * x + std::cos(yaw) * y;

        node_.commanded_pos[0] += dx;
        node_.commanded_pos[1] += dy;
        node_.commanded_pos[2] += z;

        active_axes_ = {std::fabs(x) > 0.0 || std::fabs(y) > 0.0,
                        std::fabs(x) > 0.0 || std::fabs(y) > 0.0,
                        std::fabs(z) > 0.0};
        start_updates_ = node_.control_error_updates;
        publisher_->publish(positionCommand(*clock_, node_.commanded_pos, false));

        RCLCPP_INFO(logger_, "Published relative move: offset=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f)",
                    x, y, z, node_.commanded_pos[0], node_.commanded_pos[1], node_.commanded_pos[2]);
        return checkErrors();
    }

    BT::NodeStatus onRunning() override { return checkErrors(); }

    void onHalted() override { RCLCPP_INFO(logger_, "MoveRelative wait halted."); }

   private:
    bool freshAndWithinTolerance(const std::size_t index) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= POSITION_TOLERANCE;
    }

    BT::NodeStatus checkErrors() const {
        if (mission_killed) {
            RCLCPP_WARN(logger_, "MoveRelative failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        if ((!active_axes_[0] || freshAndWithinTolerance(0)) &&
            (!active_axes_[1] || freshAndWithinTolerance(1)) &&
            (!active_axes_[2] || freshAndWithinTolerance(2))) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<bool, 3> active_axes_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
};

class AddAttSetpointAction : public BT::StatefulActionNode {
   public:
    AddAttSetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                         QuaternionCmdPublisher::SharedPtr publisher, rclcpp::Clock::SharedPtr clock,
                         rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config), node_(node), publisher_(publisher), clock_(clock), logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("roll", 0.0, "Roll offset in radians"),
                BT::InputPort<double>("pitch", 0.0, "Pitch offset in radians"),
                BT::InputPort<double>("yaw", 0.0, "Yaw offset in radians")};
    }

    BT::NodeStatus onStart() override {
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        getInput("roll", roll);
        getInput("pitch", pitch);
        getInput("yaw", yaw);

        active_axes_ = {portProvided(*this, "roll"), portProvided(*this, "pitch"), portProvided(*this, "yaw")};
        node_.commanded_att[0] += roll;
        node_.commanded_att[1] += pitch;
        node_.commanded_att[2] = normalizeAngle(node_.commanded_att[2] + yaw);

        start_updates_ = node_.control_error_updates;
        publisher_->publish(quaternionCommand(*clock_, node_.commanded_att[0], -node_.commanded_att[1],
                                              -node_.commanded_att[2]));
        RCLCPP_INFO(logger_, "Published additive attitude target rpy=(%.3f, %.3f, %.3f)",
                    node_.commanded_att[0], node_.commanded_att[1], node_.commanded_att[2]);
        return checkErrors();
    }

    BT::NodeStatus onRunning() override { return checkErrors(); }

    void onHalted() override { RCLCPP_INFO(logger_, "AddAttSetpoint wait halted."); }

   private:
    bool freshAndWithinTolerance(const std::size_t index) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= ANGLE_TOLERANCE;
    }

    BT::NodeStatus checkErrors() const {
        if (mission_killed) {
            RCLCPP_WARN(logger_, "AddAttSetpoint failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        if ((!active_axes_[0] || freshAndWithinTolerance(6)) &&
            (!active_axes_[1] || freshAndWithinTolerance(7)) &&
            (!active_axes_[2] || freshAndWithinTolerance(8))) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    QuaternionCmdPublisher::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<bool, 3> active_axes_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
};

class SpinAction : public BT::StatefulActionNode {
   public:
    SpinAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
               VectorCmdPublisher::SharedPtr angular_velocity_publisher,
               QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
               rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          angular_velocity_publisher_(angular_velocity_publisher),
          attitude_publisher_(attitude_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("yaw", 12.5663706, "Yaw distance to spin in radians"),
                BT::InputPort<double>("rate", 3.14159265, "Yaw rate in radians per second")};
    }

    BT::NodeStatus onStart() override {
        getInput("yaw", target_);
        getInput("rate", rate_);
        rate_ = std::fabs(rate_);
        if (std::fabs(target_) <= ANGLE_TOLERANCE || rate_ <= 0.0) {
            return BT::NodeStatus::SUCCESS;
        }

        accumulated_ = 0.0;
        last_time_ = std::chrono::steady_clock::now();
        start_updates_ = node_.control_error_updates[11];

        node_.last_angvel_setpoint = {0.0, 0.0, std::copysign(rate_, target_)};
        angular_velocity_publisher_->publish(vectorCommand(*clock_, node_.last_angvel_setpoint));
        RCLCPP_INFO(logger_, "Published spin yaw=%.3f rate=%.3f", target_, node_.last_angvel_setpoint[2]);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (mission_killed) {
            stopSpin();
            return BT::NodeStatus::FAILURE;
        }

        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - last_time_).count();
        last_time_ = now;

        if (node_.control_error_updates[11] > start_updates_) {
            const double measured_rate = node_.last_angvel_setpoint[2] - node_.control_errors[11];
            accumulated_ += measured_rate * dt;
        }

        if (std::fabs(accumulated_) >= std::fabs(target_)) {
            stopSpin();
            node_.commanded_att[2] = normalizeAngle(node_.commanded_att[2] + target_);
            attitude_publisher_->publish(quaternionCommand(*clock_, node_.commanded_att[0], -node_.commanded_att[1],
                                                           -node_.commanded_att[2]));
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { stopSpin(); }

   private:
    void stopSpin() {
        node_.last_angvel_setpoint = {0.0, 0.0, 0.0};
        angular_velocity_publisher_->publish(vectorCommand(*clock_, node_.last_angvel_setpoint));
    }

    MissionNode &node_;
    VectorCmdPublisher::SharedPtr angular_velocity_publisher_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    double target_ = 0.0;
    double rate_ = 0.0;
    double accumulated_ = 0.0;
    std::uint64_t start_updates_ = 0;
    std::chrono::steady_clock::time_point last_time_;
};

class WaitUntilHitAction : public BT::StatefulActionNode {
   public:
    WaitUntilHitAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                       rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config), node_(node), logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("axis", "x", "Velocity axis to monitor: x, y, or z"),
                BT::InputPort<double>("threshold", -1.0, "Speed ratio threshold; negative selects old defaults"),
                BT::InputPort<int>("timeout_msec", 40000, "Maximum wait time before returning FAILURE")};
    }

    BT::NodeStatus onStart() override {
        std::string axis = "x";
        int timeout_msec = 40000;
        getInput("axis", axis);
        getInput("threshold", threshold_);
        getInput("timeout_msec", timeout_msec);

        axis_index_ = (axis == "z") ? 2 : (axis == "y") ? 1 : 0;
        if (threshold_ < 0.0) {
            threshold_ = (axis_index_ == 2) ? 0.25 : 0.8;
        }
        deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_msec);
        initial_speed_ = measuredSpeed();
        if (std::fabs(initial_speed_) <= 0.03) {
            RCLCPP_INFO(logger_, "WaitUntilHit initial speed is low; assuming already hit object.");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (mission_killed) {
            return BT::NodeStatus::FAILURE;
        }
        const double current_speed = measuredSpeed();
        if (std::fabs(current_speed / initial_speed_) < threshold_ || std::fabs(current_speed) < 0.01) {
            return BT::NodeStatus::SUCCESS;
        }
        if (std::chrono::steady_clock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "WaitUntilHit timed out. initial=%.3f current=%.3f", initial_speed_, current_speed);
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { RCLCPP_INFO(logger_, "WaitUntilHit halted."); }

   private:
    double measuredSpeed() const {
        const std::size_t error_index = 3 + axis_index_;
        return node_.last_velocity_setpoint[axis_index_] - node_.control_errors[error_index];
    }

    MissionNode &node_;
    rclcpp::Logger logger_;
    std::size_t axis_index_ = 0;
    double threshold_ = 0.8;
    double initial_speed_ = 0.0;
    std::chrono::steady_clock::time_point deadline_;
};

class SurfaceAction : public BT::StatefulActionNode {
   public:
    SurfaceAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                  PointCmdPublisher::SharedPtr position_publisher, VectorCmdPublisher::SharedPtr velocity_publisher,
                  rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          position_publisher_(position_publisher),
          velocity_publisher_(velocity_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override {
        node_.commanded_pos[2] = 0.0;
        start_updates_ = node_.control_error_updates;
        position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos, false));
        RCLCPP_INFO(logger_, "Published surface target z=0.0");
        return checkErrors();
    }

    BT::NodeStatus onRunning() override { return checkErrors(); }

    void onHalted() override { publishZeroVelocity(); }

   private:
    BT::NodeStatus checkErrors() {
        if (mission_killed) {
            publishZeroVelocity();
            return BT::NodeStatus::FAILURE;
        }
        if (node_.control_error_updates[2] > start_updates_[2] &&
            std::fabs(node_.control_errors[2]) <= POSITION_TOLERANCE) {
            publishZeroVelocity();
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void publishZeroVelocity() {
        node_.last_velocity_setpoint = {0.0, 0.0, 0.0};
        velocity_publisher_->publish(vectorCommand(*clock_, node_.last_velocity_setpoint));
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    VectorCmdPublisher::SharedPtr velocity_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<std::uint64_t, 12> start_updates_ = {};
};

BT::NodeStatus blackboxAction(const rclcpp::Logger &logger, const std::string &name) {
    RCLCPP_INFO(logger, "Blackbox BT action '%s' ticked. Replace this stub when the mission primitive is ready.",
                name.c_str());
    return BT::NodeStatus::SUCCESS;
}

void registerTaskFlagCondition(BT::BehaviorTreeFactory &factory, const std::string &condition_name,
                               const bool enabled) {
    factory.registerSimpleCondition(condition_name, [enabled](BT::TreeNode &) {
        return enabled ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
}

void registerMissionNodes(BT::BehaviorTreeFactory &factory, MissionNode &node, const rclcpp::Logger logger,
                          PointCmdPublisher::SharedPtr position_publisher,
                          QuaternionCmdPublisher::SharedPtr attitude_publisher,
                          VectorCmdPublisher::SharedPtr linear_velocity_publisher,
                          VectorCmdPublisher::SharedPtr angular_velocity_publisher, rclcpp::Clock::SharedPtr clock) {
    factory.registerSimpleCondition("PoolAOrD", [&node](BT::TreeNode &) {
        return (node.POOL_A || node.POOL_D) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });

    factory.registerSimpleCondition("NotKilled", [](BT::TreeNode &) {
        return !mission_killed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });

    registerTaskFlagCondition(factory, "RunCoinFlip", node.COIN_FLIP);
    registerTaskFlagCondition(factory, "RunGate", node.GATE);
    registerTaskFlagCondition(factory, "RunBuoy", node.BUOY);
    registerTaskFlagCondition(factory, "RunBins", node.BINS);
    registerTaskFlagCondition(factory, "RunTorp", node.TORP);
    registerTaskFlagCondition(factory, "RunOctagon", node.OCTAGON);
    registerTaskFlagCondition(factory, "RunPrelim", node.PRELIM);
    registerTaskFlagCondition(factory, "RunPoolTest", node.POOL_TEST);
    registerTaskFlagCondition(factory, "RunVisionTestFlag", node.VISION_TEST);
    registerTaskFlagCondition(factory, "RunPidTuningSequence", node.PID_TUNING_SEQUENCE);

    factory.registerBuilder<PosSetpointAction>(
        "PosSetpoint", [&node, position_publisher, linear_velocity_publisher, clock,
                        logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<PosSetpointAction>(name, config, node, position_publisher,
                                                       linear_velocity_publisher, clock, logger);
        });

    factory.registerBuilder<AttSetpointAction>(
        "AttSetpoint", [&node, attitude_publisher, angular_velocity_publisher, clock,
                        logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<AttSetpointAction>(name, config, node, attitude_publisher,
                                                       angular_velocity_publisher, clock, logger);
        });

    factory.registerBuilder<MoveRelativeAction>(
        "MoveRelative", [&node, position_publisher, clock, logger](const std::string &name,
                                                                   const BT::NodeConfig &config) {
            return std::make_unique<MoveRelativeAction>(name, config, node, position_publisher, clock, logger);
        });

    factory.registerBuilder<AddAttSetpointAction>(
        "AddAttSetpoint", [&node, attitude_publisher, clock, logger](const std::string &name,
                                                                     const BT::NodeConfig &config) {
            return std::make_unique<AddAttSetpointAction>(name, config, node, attitude_publisher, clock, logger);
        });

    factory.registerBuilder<SpinAction>(
        "Spin", [&node, angular_velocity_publisher, attitude_publisher, clock,
                 logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<SpinAction>(name, config, node, angular_velocity_publisher, attitude_publisher,
                                                clock, logger);
        });

    factory.registerBuilder<WaitUntilHitAction>(
        "WaitUntilHit", [&node, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<WaitUntilHitAction>(name, config, node, logger);
        });

    auto surface_builder = [&node, position_publisher, linear_velocity_publisher, clock,
                            logger](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<SurfaceAction>(name, config, node, position_publisher, linear_velocity_publisher, clock,
                                               logger);
    };
    factory.registerBuilder<SurfaceAction>("SurfaceAtOctagon", surface_builder);
    factory.registerBuilder<SurfaceAction>("SurfaceAndKill", surface_builder);

    const std::vector<std::string> blackbox_nodes = {"SearchGateAfterCoinFlip",
                                                     "SetYawZero",
                                                     "AbortMission",
                                                     "AlignWithGate",
                                                     "ApproachAbydosSymbol",
                                                     "FindPathMarkerForBuoy",
                                                     "SearchForBuoy",
                                                     "ApproachBuoy",
                                                     "ReturnToBuoyStart",
                                                     "RecoverFromMissingBuoy",
                                                     "FindPathMarkerForBins",
                                                     "AlignAboveBinsPathMarker",
                                                     "FindBins",
                                                     "AlignOverBins",
                                                     "AlignOverBinShark",
                                                     "DropBalls",
                                                     "SearchTorpBoard",
                                                     "RecoverTorpBoardSearch",
                                                     "VerifyTorpBoard",
                                                     "ApproachTorpBoard",
                                                     "OrientToTorpBoard",
                                                     "ShootOpenTorpHole",
                                                     "ShootClosedTorpHole",
                                                     "SearchDhdFront",
                                                     "ApproachDhdFront",
                                                     "FindDhdDown",
                                                     "AlignOverDhd",
                                                     "SurfaceForOctagonPoints",
                                                     "GrabYellowObject",
                                                     "DropYellowObject",
                                                     "GrabRedObject",
                                                     "DropRedObject",
                                                     "ReturnToDhdCenter",
                                                     "RunVisionTest"};

    for (const std::string &name : blackbox_nodes) {
        factory.registerSimpleAction(name, [logger, name](BT::TreeNode &) { return blackboxAction(logger, name); });
    }
}

}  // namespace

MissionNode::MissionNode() : rclcpp::Node("mission") {
    rclcpp::QoS kill_qos(1);
    kill_qos.transient_local();
    this->kill_sub = this->create_subscription<std_msgs::msg::Bool>(
        "kill_switch", kill_qos, std::bind(&MissionNode::kill_callback, this, std::placeholders::_1));

    this->control_error_sub = this->create_subscription<sub_control_interfaces::msg::Error>(
        "control/error", 10, std::bind(&MissionNode::control_error_callback, this, std::placeholders::_1));

    // Set up parameters that can be passed to the node via a launch file or terminal (default=false)
    this->declare_parameter<bool>("POOL_A", false);
    this->declare_parameter<bool>("POOL_B", false);
    this->declare_parameter<bool>("POOL_C", false);
    this->declare_parameter<bool>("POOL_D", false);
    this->declare_parameter<bool>("HEADS", false);
    this->declare_parameter<bool>("TAILS", false);
    this->declare_parameter<bool>("SIM", false);
    this->declare_parameter<bool>("COIN_FLIP", false);
    this->declare_parameter<bool>("GATE", false);
    this->declare_parameter<bool>("BUOY", false);
    this->declare_parameter<bool>("BINS", false);
    this->declare_parameter<bool>("TORP", false);
    this->declare_parameter<bool>("OCTAGON", false);
    this->declare_parameter<bool>("PRELIM", false);
    this->declare_parameter<bool>("POOL_TEST", false);
    this->declare_parameter<bool>("VISION_TEST", false);
    this->declare_parameter<bool>("PID_TUNING_SEQUENCE", false);

    // Store the parameters inside variables
    this->get_parameter("POOL_A", this->POOL_A);
    this->get_parameter("POOL_B", this->POOL_B);
    this->get_parameter("POOL_C", this->POOL_C);
    this->get_parameter("POOL_D", this->POOL_D);
    this->get_parameter("HEADS", this->HEADS);
    this->get_parameter("TAILS", this->TAILS);
    this->get_parameter("SIM", this->SIM);
    this->get_parameter("COIN_FLIP", this->COIN_FLIP);
    this->get_parameter("GATE", this->GATE);
    this->get_parameter("BUOY", this->BUOY);
    this->get_parameter("BINS", this->BINS);
    this->get_parameter("TORP", this->TORP);
    this->get_parameter("OCTAGON", this->OCTAGON);
    this->get_parameter("PRELIM", this->PRELIM);
    this->get_parameter("POOL_TEST", this->POOL_TEST);
    this->get_parameter("VISION_TEST", this->VISION_TEST);
    this->get_parameter("PID_TUNING_SEQUENCE", this->PID_TUNING_SEQUENCE);

}

void MissionNode::kill_callback(const std_msgs::msg::Bool &msg) {
    this->killed = msg.data;
    mission_killed = msg.data;
    if (this->killed) {
        RCLCPP_WARN(this->get_logger(), "Kill switch engaged.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Kill switch released.");
    }
}

void MissionNode::control_error_callback(const sub_control_interfaces::msg::Error &msg) {
    const std::array<std::array<double, 3>, 4> errors = {
        std::array<double, 3>{msg.pos_error[0], -msg.pos_error[1], -msg.pos_error[2]},
        std::array<double, 3>{msg.vel_error[0], -msg.vel_error[1], -msg.vel_error[2]},
        std::array<double, 3>{msg.att_error[0], -msg.att_error[1], -msg.att_error[2]},
        std::array<double, 3>{msg.angvel_error[0], -msg.angvel_error[1], -msg.angvel_error[2]},
    };

    for (std::size_t group = 0; group < errors.size(); ++group) {
        for (std::size_t axis = 0; axis < errors[group].size(); ++axis) {
            const std::size_t index = group * errors[group].size() + axis;
            this->control_errors[index] = errors[group][axis];
            ++this->control_error_updates[index];
        }
    }
}

void MissionNode::activate() {
    RCLCPP_INFO(this->get_logger(), "Activating mission_node");
    RCLCPP_INFO(this->get_logger(), "Init control clients");
    service_client::init_clients();

    // Wait for sub to turn on
    RCLCPP_INFO(this->get_logger(), "Wait for unkill");
    while (rclcpp::ok() && !service_client::alive()) {
        RCLCPP_INFO(this->get_logger(), "Sub is not alive yet, waiting...");
        std::this_thread::sleep_for(0.5s);
    }

    RCLCPP_INFO(this->get_logger(), "Received alive signal");
    RCLCPP_INFO(this->get_logger(), "Wait for motors to start up");
    std::this_thread::sleep_for(7s);
}

void MissionNode::execute() {
    RCLCPP_INFO(this->get_logger(), "Executing mission");

    try {
        BT::BehaviorTreeFactory factory;
        const auto position_publisher = this->create_publisher<PointCmdMsg>("cmd_position", 10);
        const auto attitude_publisher = this->create_publisher<QuaternionCmdMsg>("cmd_attitude", 10);
        const auto linear_velocity_publisher = this->create_publisher<VectorCmdMsg>("cmd_linear_velocity", 10);
        const auto angular_velocity_publisher = this->create_publisher<VectorCmdMsg>("cmd_angular_velocity", 10);
        registerMissionNodes(factory, *this, this->get_logger(), position_publisher, attitude_publisher,
                             linear_velocity_publisher, angular_velocity_publisher, this->get_clock());

        factory.registerBehaviorTreeFromFile(missionTreePath());
        BT::Tree tree = factory.createTree("SelectedMission");
        BT::Groot2Publisher groot_publish(tree, 5555);
        BT::NodeStatus status = BT::NodeStatus::IDLE;
        while (rclcpp::ok()) {
            status = tree.tickOnce();
            if (status != BT::NodeStatus::RUNNING) {
                break;
            }
            tree.sleep(std::chrono::milliseconds(50));
        }

        if (status == BT::NodeStatus::RUNNING) {
            tree.haltTree();
            RCLCPP_INFO(this->get_logger(), "Behavior tree 'SelectedMission' halted while RUNNING.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Behavior tree 'SelectedMission' finished with %s.", statusName(status));
    } catch (const std::exception &error) {
        RCLCPP_ERROR(this->get_logger(), "Behavior tree failed: %s", error.what());
    }
}

void MissionNode::run() {
    // Activates node, and executes the mission (does everything in one function)
    this->activate();
    this->execute();
}
