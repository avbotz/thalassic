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
#include "sub_mission/nodes/commands.hpp"
#include "sub_mission/nodes/vision.hpp"
#include "sub_mission/utils.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

namespace {

bool mission_killed = false;
class PosSetpointAction : public BT::StatefulActionNode {
   public:
    PosSetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                      PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                      rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          position_publisher_(position_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("x", UNSPECIFIED_PORT, "North/forward position setpoint"),
                BT::InputPort<double>("y", UNSPECIFIED_PORT, "East/right position setpoint"),
                BT::InputPort<double>("z", UNSPECIFIED_PORT, "Down/depth position setpoint")};
    }

    BT::NodeStatus onStart() override {
        std::array<double, 3> inputs{};
        getInput("x", inputs[0]);
        getInput("y", inputs[1]);
        getInput("z", inputs[2]);
        active_axes_ = {std::isfinite(inputs[0]), std::isfinite(inputs[1]), std::isfinite(inputs[2])};
        std::array<double, 3> target = node_.commanded_pos;

        if (active_axes_[0]) {
            target[0] = inputs[0];
        }
        if (active_axes_[1]) {
            target[1] = inputs[1];
        }
        if (active_axes_[2]) {
            target[2] = inputs[2];
        }
        node_.commanded_pos = target;
        position_publisher_->publish(positionCommand(*clock_, target, false));

        start_updates_ = node_.control_error_updates;

        RCLCPP_INFO(logger_, "Published position command: xyz=(%.3f, %.3f, %.3f)", target[0], target[1], target[2]);
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
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<bool, 3> active_axes_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
};

class VelocitySetpointAction : public BT::SyncActionNode {
   public:
    VelocitySetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                           VectorCmdPublisher::SharedPtr velocity_publisher, rclcpp::Clock::SharedPtr clock,
                           rclcpp::Logger logger)
        : BT::SyncActionNode(name, config),
          node_(node),
          velocity_publisher_(velocity_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("x", 0.0, "Forward velocity in meters per second"),
                BT::InputPort<double>("y", 0.0, "Right velocity in meters per second"),
                BT::InputPort<double>("z", 0.0, "Down velocity in meters per second")};
    }

    BT::NodeStatus tick() override {
        std::array<double, 3> target{};
        getInput("x", target[0]);
        getInput("y", target[1]);
        getInput("z", target[2]);

        node_.last_velocity_setpoint = target;
        velocity_publisher_->publish(linearVelocityCommand(*clock_, target));
        RCLCPP_INFO(logger_, "Published velocity command: xyz=(%.3f, %.3f, %.3f)", target[0], target[1], target[2]);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    MissionNode &node_;
    VectorCmdPublisher::SharedPtr velocity_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
};

class AltitudeSetpointAction : public BT::StatefulActionNode {
   public:
    AltitudeSetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                           PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                           rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          position_publisher_(position_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("z", "Altitude above bottom in meters")};
    }

    BT::NodeStatus onStart() override {
        getInput("z", node_.commanded_pos[2]);
        start_updates_ = node_.control_error_updates;
        position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos, true));
        RCLCPP_INFO(logger_, "Published altitude command: z=%.3f", node_.commanded_pos[2]);
        return checkErrors();
    }

    BT::NodeStatus onRunning() override { return checkErrors(); }

    void onHalted() override { RCLCPP_INFO(logger_, "AltitudeSetpoint wait halted."); }

   private:
    BT::NodeStatus checkErrors() const {
        if (mission_killed) {
            RCLCPP_WARN(logger_, "AltitudeSetpoint wait failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        if (node_.control_error_updates[2] > start_updates_[2] &&
            std::fabs(node_.control_errors[2]) <= POSITION_TOLERANCE) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<std::uint64_t, 12> start_updates_ = {};
};

class AttSetpointAction : public BT::StatefulActionNode {
   public:
    AttSetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                      QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
                      rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          attitude_publisher_(attitude_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("roll", UNSPECIFIED_PORT, "Roll setpoint in radians"),
                BT::InputPort<double>("pitch", UNSPECIFIED_PORT, "Pitch setpoint in radians"),
                BT::InputPort<double>("yaw", UNSPECIFIED_PORT, "Yaw setpoint in radians")};
    }

    BT::NodeStatus onStart() override {
        std::array<double, 3> inputs{};
        getInput("roll", inputs[0]);
        getInput("pitch", inputs[1]);
        getInput("yaw", inputs[2]);
        active_axes_ = {std::isfinite(inputs[0]), std::isfinite(inputs[1]), std::isfinite(inputs[2])};
        std::array<double, 3> target = node_.commanded_att;

        if (active_axes_[0]) {
            target[0] = inputs[0];
        }
        if (active_axes_[1]) {
            target[1] = inputs[1];
        }
        if (active_axes_[2]) {
            target[2] = normalizeAngle(inputs[2]);
        }
        node_.commanded_att = target;
        attitude_publisher_->publish(attitudeCommand(*clock_, target));

        start_updates_ = node_.control_error_updates;

        RCLCPP_INFO(logger_, "Published attitude command: rpy=(%.3f, %.3f, %.3f)", target[0], target[1], target[2]);
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
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<bool, 3> active_axes_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
};

class AngularVelocitySetpointAction : public BT::SyncActionNode {
   public:
    AngularVelocitySetpointAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                                  VectorCmdPublisher::SharedPtr angular_velocity_publisher,
                                  rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger)
        : BT::SyncActionNode(name, config),
          node_(node),
          angular_velocity_publisher_(angular_velocity_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("roll", 0.0, "Roll rate in radians per second"),
                BT::InputPort<double>("pitch", 0.0, "Pitch rate in radians per second"),
                BT::InputPort<double>("yaw", 0.0, "Yaw rate in radians per second")};
    }

    BT::NodeStatus tick() override {
        std::array<double, 3> target{};
        getInput("roll", target[0]);
        getInput("pitch", target[1]);
        getInput("yaw", target[2]);

        node_.last_angvel_setpoint = target;
        angular_velocity_publisher_->publish(angularVelocityCommand(*clock_, target));
        RCLCPP_INFO(logger_, "Published angular velocity command: rpy=(%.3f, %.3f, %.3f)", target[0], target[1],
                    target[2]);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    MissionNode &node_;
    VectorCmdPublisher::SharedPtr angular_velocity_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
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
        return {BT::InputPort<double>("roll", UNSPECIFIED_PORT, "Roll offset in radians"),
                BT::InputPort<double>("pitch", UNSPECIFIED_PORT, "Pitch offset in radians"),
                BT::InputPort<double>("yaw", UNSPECIFIED_PORT, "Yaw offset in radians")};
    }

    BT::NodeStatus onStart() override {
        std::array<double, 3> inputs{};
        getInput("roll", inputs[0]);
        getInput("pitch", inputs[1]);
        getInput("yaw", inputs[2]);
        active_axes_ = {std::isfinite(inputs[0]), std::isfinite(inputs[1]), std::isfinite(inputs[2])};
        if (active_axes_[0]) {
            node_.commanded_att[0] += inputs[0];
        }
        if (active_axes_[1]) {
            node_.commanded_att[1] += inputs[1];
        }
        if (active_axes_[2]) {
            node_.commanded_att[2] = normalizeAngle(node_.commanded_att[2] + inputs[2]);
        }

        start_updates_ = node_.control_error_updates;
        publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
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
        angular_velocity_publisher_->publish(angularVelocityCommand(*clock_, node_.last_angvel_setpoint));
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
            attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { stopSpin(); }

   private:
    void stopSpin() {
        node_.last_angvel_setpoint = {0.0, 0.0, 0.0};
        angular_velocity_publisher_->publish(angularVelocityCommand(*clock_, node_.last_angvel_setpoint));
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
        velocity_publisher_->publish(linearVelocityCommand(*clock_, node_.last_velocity_setpoint));
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    VectorCmdPublisher::SharedPtr velocity_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<std::uint64_t, 12> start_updates_ = {};
};

// ---------------------------------------------------------------------------
// Blackbox action framework
//
// A "blackbox" is a mission primitive whose real behavior lives in the
// perception/strategy layer and is not wired up yet (vision alignment, object
// manipulation, torpedo/dropper actuation, ...). Registering it as a stub lets
// a full mission tree load and dry-run in the sim before the primitive exists;
// each stub logs its human-readable description and returns a fixed status.
//
// Promotion path -- to give a blackbox real behavior:
//   1. Write a BT::StatefulActionNode for it (PosSetpointAction above is the
//      template). It can publish setpoints, read node_.control_errors and the
//      {@...} blackboard, and return RUNNING until the goal is reached.
//   2. registerBuilder<YourAction>("Name", ...) it in registerMissionNodes().
//   3. Delete its row from kBlackboxActions below.
// Whatever remains in kBlackboxActions is, by definition, not implemented yet.
struct BlackboxAction {
    const char *name;
    const char *description;
    BT::NodeStatus result = BT::NodeStatus::SUCCESS;
};

BT::NodeStatus tickBlackbox(const rclcpp::Logger &logger, const BlackboxAction &spec) {
    RCLCPP_INFO(logger, "[blackbox] %s -> %s  (%s)", spec.name, statusName(spec.result), spec.description);
    return spec.result;
}

void registerMissionNodes(BT::BehaviorTreeFactory &factory, MissionNode &node, const rclcpp::Logger logger,
                          PointCmdPublisher::SharedPtr position_publisher,
                          QuaternionCmdPublisher::SharedPtr attitude_publisher,
                          VectorCmdPublisher::SharedPtr linear_velocity_publisher,
                          VectorCmdPublisher::SharedPtr angular_velocity_publisher, rclcpp::Clock::SharedPtr clock) {
    factory.registerSimpleCondition("NotKilled", [](BT::TreeNode &) {
        return !mission_killed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });

    factory.registerBuilder<PosSetpointAction>(
        "PosSetpoint", [&node, position_publisher, clock,
                        logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<PosSetpointAction>(name, config, node, position_publisher, clock, logger);
        });

    factory.registerBuilder<VelocitySetpointAction>(
        "VelocitySetpoint", [&node, linear_velocity_publisher, clock,
                             logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<VelocitySetpointAction>(name, config, node, linear_velocity_publisher, clock,
                                                            logger);
        });

    factory.registerBuilder<AltitudeSetpointAction>(
        "AltitudeSetpoint", [&node, position_publisher, clock,
                             logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<AltitudeSetpointAction>(name, config, node, position_publisher, clock, logger);
        });

    factory.registerBuilder<AttSetpointAction>(
        "AttSetpoint", [&node, attitude_publisher, clock,
                        logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<AttSetpointAction>(name, config, node, attitude_publisher, clock, logger);
        });

    factory.registerBuilder<AngularVelocitySetpointAction>(
        "AngularVelocitySetpoint", [&node, angular_velocity_publisher, clock,
                                    logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<AngularVelocitySetpointAction>(name, config, node, angular_velocity_publisher,
                                                                   clock, logger);
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

    constexpr BT::NodeStatus OK = BT::NodeStatus::SUCCESS;
    constexpr BT::NodeStatus FAIL = BT::NodeStatus::FAILURE;
    const std::vector<BlackboxAction> blackbox_actions = {
        // Coin flip / orientation
        {"SearchGateAfterCoinFlip", "Rotate to reacquire the gate after the coin-flip drop", OK},
        {"SetYawZero", "Latch the current heading as the mission yaw reference", OK},
        {"AbortMission", "Give up on the current task and fail out", FAIL},
        // Gate alignment/pass-through is implemented in trees/gate.xml with
        // SweepCheck + ForwardAlign.
        // Slalom (2026: weave the three red/white pipe gates)
        {"SearchForSlalom", "Sweep to bring the slalom pipes into view", OK},
        {"RecoverSlalomSearch", "Dead-reckon forward when the slalom is not seen", OK},
        {"PassSlalomChannel", "Weave through the slalom staying within the pipe plane", OK},
        // Bins (2026: single bin, shark/sawfish halves)
        {"FindPathMarkerForBins", "Find the path marker pointing toward the bin", OK},
        {"AlignAboveBinsPathMarker", "Align heading to the bin path marker", OK},
        {"FindBins", "Bring the bin into the down-camera view", OK},
        {"AlignOverBins", "Center over the bin", OK},
        {"AlignOverBinTarget", "Center over the {@target_animal} half of the bin", OK},
        {"DropBalls", "Release both droppers", OK},
        // Torpedoes (2026: board with two openings)
        {"SearchTorpBoard", "Sweep to find the torpedo board", OK},
        {"RecoverTorpBoardSearch", "Dead-reckon along the search heading when the board is not seen", OK},
        {"VerifyTorpBoard", "Confirm the detection is the torpedo board", OK},
        {"ApproachTorpBoard", "Close range on the torpedo board", OK},
        {"OrientToTorpBoard", "Square up to the board face", OK},
        {"ShootChosenTorpHole", "Fire a torpedo through the {@target_animal} opening", OK},
        {"ShootOtherTorpHole", "Fire the second torpedo through the other opening", OK},
        // Octagon (2026: home the pinger, surface, sort trash by color)
        {"SearchDhdFront", "Search the front camera for the octagon marker", OK},
        {"ApproachDhdFront", "Approach the octagon", OK},
        {"FindDhdDown", "Bring the octagon center into the down camera", OK},
        {"AlignOverDhd", "Center over the octagon", OK},
        {"SurfaceForOctagonPoints", "Surface inside the octagon", OK},
        {"GrabYellowObject", "Pick up the yellow object", OK},
        {"DropYellowObject", "Sort the yellow object onto the table", OK},
        {"GrabRedObject", "Pick up the red object", OK},
        {"DropRedObject", "Sort the red object onto the table", OK},
        {"ReturnToDhdCenter", "Return to the octagon center", OK},
        // Buoy (legacy 2024 task; still available via trees/buoy.xml, not in the 2026 run)
        {"FindPathMarkerForBuoy", "Find the path marker pointing toward the buoy", OK},
        {"SearchForBuoy", "Sweep to find the buoy", OK},
        {"ApproachBuoy", "Close range on the buoy", OK},
        {"ReturnToBuoyStart", "Return to the buoy approach start", OK},
        {"RecoverFromMissingBuoy", "Recover when the buoy is not found", OK},
    };

    for (const BlackboxAction &spec : blackbox_actions) {
        factory.registerSimpleAction(spec.name,
                                     [logger, spec](BT::TreeNode &) { return tickBlackbox(logger, spec); });
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

    this->vision_client = std::make_unique<VisionClient>(*this);

    // Mission config name (resources/missions/<name>.xml) or file path
    this->declare_parameter<std::string>("mission", "");
    this->get_parameter("mission", this->mission);

    this->declare_parameter<std::string>("role", "SURVEY");
    this->get_parameter("role", this->role);
    if (this->role != "SURVEY" && this->role != "SEARCH") {
        throw std::invalid_argument("sub_mission role must be SURVEY or SEARCH");
    }
    RCLCPP_INFO(this->get_logger(), "Mission role: %s", this->role.c_str());
}

std::string MissionNode::visionModelTask(const std::string &task) const {
    if (task.empty()) {
        return task;
    }
    return task + (this->role == "SEARCH" ? "_search" : "_survey");
}

bool MissionNode::visionTaskMatches(const std::string &reported_task, const std::string &logical_task) const {
    return logical_task.empty() || reported_task == logical_task || reported_task == visionModelTask(logical_task);
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

bool MissionNode::load_mission() {
    std::string available;
    for (const std::string &name : availableMissions()) {
        available += "\n  " + name;
    }

    if (this->mission.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "No mission selected. Relaunch with -p mission:=<name or path>. Available missions:%s",
                     available.c_str());
        return false;
    }

    const std::string mission_path = resolveMissionPath(this->mission);
    if (!std::filesystem::exists(mission_path)) {
        RCLCPP_ERROR(this->get_logger(), "Mission file '%s' does not exist. Available missions:%s",
                     mission_path.c_str(), available.c_str());
        return false;
    }

    try {
        BT::BehaviorTreeFactory factory;
        const auto position_publisher = this->create_publisher<PointCmdMsg>("pos_setpoint", 10);
        const auto attitude_publisher = this->create_publisher<QuaternionCmdMsg>("att_setpoint", 10);
        const auto linear_velocity_publisher = position_publisher;
        const auto angular_velocity_publisher = attitude_publisher;
        registerMissionNodes(factory, *this, this->get_logger(), position_publisher, attitude_publisher,
                             linear_velocity_publisher, angular_velocity_publisher, this->get_clock());
        registerVisionNodes(factory, *this, this->get_logger(), position_publisher, attitude_publisher,
                            this->get_clock());

        for (const std::string &tree_file : treeFiles()) {
            factory.registerBehaviorTreeFromFile(tree_file);
        }

        // The mission file's main_tree_to_execute composes the registered task trees.
        this->tree = factory.createTreeFromFile(mission_path);
        RCLCPP_INFO(this->get_logger(), "Loaded mission from %s", mission_path.c_str());
    } catch (const std::exception &error) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load mission '%s': %s", mission_path.c_str(), error.what());
        return false;
    }
    return true;
}

void MissionNode::execute() {
    if (!this->tree.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "No mission tree loaded.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Executing mission '%s'", this->mission.c_str());

    try {
        BT::Groot2Publisher groot_publish(*this->tree, 5555);
        BT::NodeStatus status = BT::NodeStatus::IDLE;
        while (rclcpp::ok()) {
            status = this->tree->tickOnce();
            if (status != BT::NodeStatus::RUNNING) {
                break;
            }
            this->tree->sleep(std::chrono::milliseconds(50));
        }

        if (status == BT::NodeStatus::RUNNING) {
            this->tree->haltTree();
            RCLCPP_INFO(this->get_logger(), "Mission '%s' halted while RUNNING.", this->mission.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Mission '%s' finished with %s.", this->mission.c_str(), statusName(status));
    } catch (const std::exception &error) {
        RCLCPP_ERROR(this->get_logger(), "Mission failed: %s", error.what());
    }
}

void MissionNode::run() {
    // Loads the mission tree, activates the node, and executes the mission.
    // Loading first surfaces XML mistakes at launch, before the sub is unkilled.
    if (!this->load_mission()) {
        return;
    }
    this->activate();
    this->execute();
}
