#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"
#include "sub_mission/vision_client.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

namespace {

using SteadyClock = std::chrono::steady_clock;

// Inside the octagon: turn in yaw_step_deg steps, looking at the front camera,
// until a surface_class_id image is in view, then rise to surface_z. The first
// diversion_class_id image turns the vehicle diversion_deg once, away from
// the images that are not the ones to surface at.
class OctagonSurfaceSweepAction : public BT::StatefulActionNode {
   public:
    OctagonSurfaceSweepAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                              SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                              rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("task", "", "Front-camera task whose detections are inspected"),
            BT::InputPort<std::string>("surface_class_id", "0", "Class that sends the vehicle up"),
            BT::InputPort<std::string>("diversion_class_id", "1", "Class that turns the vehicle diversion_deg, once"),
            BT::InputPort<double>("min_score", 0.0, "Minimum detection confidence"),
            BT::InputPort<double>("yaw_step_deg", 30.0, "Search turn per step in degrees; positive turns left"),
            BT::InputPort<double>("diversion_deg", 90.0, "Turn on seeing the diversion class; positive turns left"),
            BT::InputPort<double>("surface_z", -0.3, "z to rise to on seeing the surface class (m)"),
            BT::InputPort<int>("move_timeout_msec", 20000, "Maximum wait for each turn or the rise")};
    }

    BT::NodeStatus onStart() override {
        if (!readPort(*this, "task", task_, logger_) ||
            !readPort(*this, "surface_class_id", surface_class_id_, logger_) ||
            !readPort(*this, "diversion_class_id", diversion_class_id_, logger_) ||
            !readPort(*this, "min_score", min_score_, logger_) ||
            !readPort(*this, "yaw_step_deg", yaw_step_deg_, logger_) ||
            !readPort(*this, "diversion_deg", diversion_deg_, logger_) ||
            !readPort(*this, "surface_z", surface_z_, logger_) ||
            !readPort(*this, "move_timeout_msec", move_timeout_msec_, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        if (task_.empty() || !std::isfinite(yaw_step_deg_) || yaw_step_deg_ == 0.0 || !std::isfinite(diversion_deg_) ||
            !std::isfinite(min_score_) || !std::isfinite(surface_z_) || move_timeout_msec_ <= 0) {
            RCLCPP_ERROR(logger_,
                         "OctagonSurfaceSweep needs a task, a finite non-zero yaw_step_deg, finite diversion_deg, "
                         "min_score and surface_z, and a positive move_timeout_msec.");
            return BT::NodeStatus::FAILURE;
        }

        last_processed_ = SteadyClock::now();
        diverted_ = false;
        surfacing_ = false;
        turn(yaw_step_deg_);
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
            RCLCPP_WARN(logger_, "OctagonSurfaceSweep: %s timed out.", surfacing_ ? "rise" : "turn");
            return BT::NodeStatus::FAILURE;
        }
        if (surfacing_) {
            if (fresh(2) && std::fabs(node_.control_errors[2]) <= POSITION_TOLERANCE) {
                RCLCPP_INFO(logger_, "OctagonSurfaceSweep: at z=%.2f.", surface_z_);
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;
        }
        if (fresh(8) && std::fabs(node_.control_errors[8]) <= ANGLE_TOLERANCE) {
            turn(yaw_step_deg_);
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

        bool diversion = false;
        for (const auto &detection : snapshot.detections->detections) {
            if (detection.detection.results.empty()) {
                continue;
            }
            const auto &hypothesis = detection.detection.results[0].hypothesis;
            if (hypothesis.score < min_score_) {
                continue;
            }
            if (hypothesis.class_id == surface_class_id_) {
                rise();
                return;
            }
            diversion = diversion || hypothesis.class_id == diversion_class_id_;
        }
        if (diversion && !diverted_) {
            diverted_ = true;
            turn(diversion_deg_);
            RCLCPP_INFO(logger_, "OctagonSurfaceSweep: class %s in view; turning %.1f deg.",
                        diversion_class_id_.c_str(), diversion_deg_);
        }
    }

    // A turn from where the heading is meant to be, so the steps add up.
    void turn(const double degrees) {
        const std::array<double, 3> target{UNSPECIFIED_PORT, UNSPECIFIED_PORT,
                                           normalizeAngle(node_.attitudeTarget(2) + degrees * M_PI / 180.0)};
        node_.commandAttitude(target);
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        setpoint_publisher_->publish(attitudeCommand(*clock_, target));
    }

    // surface_z is the tree's own value, not one derived from a detection, so
    // it is not bounded like vision steering is.
    void rise() {
        surfacing_ = true;
        const std::array<double, 3> target{UNSPECIFIED_PORT, UNSPECIFIED_PORT, surface_z_};
        node_.commandPosition(target, false);
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        setpoint_publisher_->publish(positionCommand(*clock_, target, false));
        RCLCPP_INFO(logger_, "OctagonSurfaceSweep: class %s in view; rising to z=%.2f.", surface_class_id_.c_str(),
                    surface_z_);
    }

    bool fresh(const std::size_t index) const { return node_.control_error_updates[index] > start_updates_[index]; }

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point deadline_;
    std::array<std::uint64_t, 12> start_updates_ = {};
    std::string task_;
    std::string surface_class_id_ = "0";
    std::string diversion_class_id_ = "1";
    double min_score_ = 0.0;
    double yaw_step_deg_ = 30.0;
    double diversion_deg_ = 90.0;
    double surface_z_ = -0.3;
    int move_timeout_msec_ = 20000;
    bool diverted_ = false;
    bool surfacing_ = false;
};

}  // namespace

void registerOctagonSurfaceSweepAction(BT::BehaviorTreeFactory &factory, MissionNode &node,
                                       SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                       rclcpp::Logger logger) {
    factory.registerBuilder<OctagonSurfaceSweepAction>(
        "OctagonSurfaceSweep",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<OctagonSurfaceSweepAction>(name, config, node, setpoint_publisher, clock, logger);
        });
}
