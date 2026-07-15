#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"
#include "sub_mission/vision_client.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

namespace {

using SteadyClock = std::chrono::steady_clock;

class OctagonSurfaceSweepAction : public BT::StatefulActionNode {
   public:
    OctagonSurfaceSweepAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                              PointCmdPublisher::SharedPtr position_publisher,
                              QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
                              rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          position_publisher_(position_publisher),
          attitude_publisher_(attitude_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("task", "", "Front-camera model task to inspect"),
                BT::InputPort<std::string>("surface_class_id", "0", "Class that triggers surfacing"),
                BT::InputPort<std::string>("diversion_class_id", "1", "Class that triggers yaw diversion"),
                BT::InputPort<double>("min_score", 0.0, "Minimum detection confidence"),
                BT::InputPort<double>("yaw_step_deg", 30.0, "Yaw increment while searching for surface class"),
                BT::InputPort<double>("diversion_deg", 90.0, "One-time yaw increment after seeing diversion class"),
                BT::InputPort<double>("surface_z", -0.3, "Final ENU z setpoint after seeing surface class"),
                BT::InputPort<int>("move_timeout_msec", 20000, "Maximum wait for each yaw or depth command")};
    }

    BT::NodeStatus onStart() override {
        getInput("task", task_);
        getInput("surface_class_id", surface_class_id_);
        getInput("diversion_class_id", diversion_class_id_);
        getInput("min_score", min_score_);
        getInput("yaw_step_deg", yaw_step_deg_);
        getInput("diversion_deg", diversion_deg_);
        getInput("surface_z", surface_z_);
        getInput("move_timeout_msec", move_timeout_msec_);

        if (task_.empty() || yaw_step_deg_ == 0.0 || move_timeout_msec_ <= 0) {
            RCLCPP_ERROR(logger_, "OctagonSurfaceSweep requires a task, non-zero yaw_step_deg, and positive timeout.");
            return BT::NodeStatus::FAILURE;
        }

        last_processed_ = SteadyClock::now();
        diversion_applied_ = false;
        surfacing_ = false;
        commandYaw(yaw_step_deg_);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!node_.subAlive()) {
            RCLCPP_WARN(logger_, "OctagonSurfaceSweep failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }

        if (!surfacing_) {
            inspectLatestFrame();
        }

        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "OctagonSurfaceSweep timed out waiting for %s convergence.",
                        surfacing_ ? "surface" : "yaw");
            return BT::NodeStatus::FAILURE;
        }

        if (surfacing_) {
            if (node_.control_error_updates[2] > start_updates_ &&
                std::fabs(node_.control_errors[2]) <= POSITION_TOLERANCE) {
                RCLCPP_INFO(logger_, "OctagonSurfaceSweep: surface setpoint reached.");
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;
        }

        if (node_.control_error_updates[8] > start_updates_ && std::fabs(node_.control_errors[8]) <= ANGLE_TOLERANCE) {
            commandYaw(yaw_step_deg_);
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { RCLCPP_INFO(logger_, "OctagonSurfaceSweep halted."); }

   private:
    void inspectLatestFrame() {
        const VisionClient::Snapshot snapshot = node_.vision().latest("front");
        if (!snapshot.detections || snapshot.received_at <= last_processed_ ||
            !node_.visionTaskMatches(snapshot.detections->task, task_)) {
            return;
        }
        last_processed_ = snapshot.received_at;

        bool saw_diversion = false;
        for (const auto &detection : snapshot.detections->detections) {
            if (detection.detection.results.empty()) {
                continue;
            }
            const auto &hypothesis = detection.detection.results[0].hypothesis;
            if (hypothesis.score < min_score_) {
                continue;
            }
            if (hypothesis.class_id == surface_class_id_) {
                commandSurface();
                return;
            }
            saw_diversion = saw_diversion || hypothesis.class_id == diversion_class_id_;
        }

        if (saw_diversion && !diversion_applied_) {
            diversion_applied_ = true;
            commandYaw(diversion_deg_);
            RCLCPP_INFO(logger_, "OctagonSurfaceSweep: class %s seen; applying %.1f degree diversion.",
                        diversion_class_id_.c_str(), diversion_deg_);
        }
    }

    void commandYaw(const double degrees) {
        node_.commanded_att[2] = normalizeAngle(node_.commanded_att[2] + degrees * M_PI / 180.0);
        start_updates_ = node_.control_error_updates[8];
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
    }

    void commandSurface() {
        surfacing_ = true;
        node_.commanded_pos[2] = surface_z_;
        start_updates_ = node_.control_error_updates[2];
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos));
        RCLCPP_INFO(logger_, "OctagonSurfaceSweep: class %s seen; surfacing to z=%.2f.",
                    surface_class_id_.c_str(), surface_z_);
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point deadline_;
    std::uint64_t start_updates_ = 0;
    std::string task_;
    std::string surface_class_id_ = "0";
    std::string diversion_class_id_ = "1";
    double min_score_ = 0.0;
    double yaw_step_deg_ = 30.0;
    double diversion_deg_ = 90.0;
    double surface_z_ = -0.3;
    int move_timeout_msec_ = 20000;
    bool diversion_applied_ = false;
    bool surfacing_ = false;
};

}  // namespace

void registerOctagonSurfaceSweepAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                       PointCmdPublisher::SharedPtr position_publisher,
                                       QuaternionCmdPublisher::SharedPtr attitude_publisher,
                                       rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger) {
    factory.registerBuilder<OctagonSurfaceSweepAction>(
        "OctagonSurfaceSweep", [&node, position_publisher, attitude_publisher, clock, logger](
                                   const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<OctagonSurfaceSweepAction>(name, config, node, position_publisher,
                                                               attitude_publisher, clock, logger);
        });
}
