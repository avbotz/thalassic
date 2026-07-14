/** @file vision.cpp
 *  @brief Generic perception BT leaves connecting sub_mission to sub_vision.
 *
 * Reusable primitives, filtered by (camera, task, class_id, min_score):
 *
 *   LoadModel         -- ask a camera's sub_vision node to make a task's
 *                        detector active (async service call, no-op when the
 *                        model is already loaded).
 *   DetectionVisible  -- condition: a fresh matching detection exists.
 *   WaitForDetection  -- wait until a matching detection arrives, or fail
 *                        after a timeout.
 *   AlignToDetection  -- closed-loop centering: steer yaw and/or depth until
 *                        the detection's bearing is within tolerance.
 *
 * ForwardSweepAlign preserves the old mission's forwardSweepCheck behavior as
 * one C++ BT node: it sweeps yaw, samples detections, moves forward between
 * sweeps, and waits for control convergence internally.
 */

#include "sub_mission/nodes/vision.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "sub_mission/nodes/mission.hpp"
#include "sub_mission/vision_client.hpp"

namespace {

using sub_vision_interfaces::msg::Detection;
using sub_vision_interfaces::srv::LoadModel;
using SteadyClock = std::chrono::steady_clock;

// The (camera, task, class_id, min_score) ports shared by every vision leaf.
struct DetectionFilter {
    std::string camera = "front";
    std::string task;
    std::string class_id;
    double min_score = 0.0;

    static BT::PortsList ports() {
        return {BT::InputPort<std::string>("camera", "front", "Detection source: front or down"),
                BT::InputPort<std::string>("task", "", "Only accept detections from this task's model"),
                BT::InputPort<std::string>("class_id", "", "Only accept this class id (empty = any)"),
                BT::InputPort<double>("min_score", 0.0, "Minimum detection confidence")};
    }

    static DetectionFilter read(const BT::TreeNode &node) {
        DetectionFilter filter;
        node.getInput("camera", filter.camera);
        node.getInput("task", filter.task);
        node.getInput("class_id", filter.class_id);
        node.getInput("min_score", filter.min_score);
        return filter;
    }

    std::string describe() const {
        return "camera=" + camera + " task=" + (task.empty() ? "*" : task) +
               " class=" + (class_id.empty() ? "*" : class_id);
    }
};

// Latest detection matching the filter, provided the snapshot was received
// after `not_before`. Returns the owning array so the Detection* stays valid.
struct Match {
    sub_vision_interfaces::msg::DetectionArray::ConstSharedPtr array;
    const Detection *detection = nullptr;
    SteadyClock::time_point received_at{};
};

Match latestMatch(MissionNode &node, const DetectionFilter &filter, const SteadyClock::time_point not_before) {
    Match match;
    const VisionClient::Snapshot snapshot = node.vision().latest(filter.camera);
    if (!snapshot.detections || snapshot.received_at < not_before) {
        return match;
    }
    if (!node.visionTaskMatches(snapshot.detections->task, filter.task)) {
        return match;
    }
    match.detection = VisionClient::bestMatch(*snapshot.detections, "", filter.class_id, filter.min_score);
    if (match.detection != nullptr) {
        match.array = snapshot.detections;
        match.received_at = snapshot.received_at;
    }
    return match;
}

bool validDistanceValue(const double distance) { return std::isfinite(distance) && distance > 0.0; }

double radians(const double degrees) { return degrees * M_PI / 180.0; }

std::optional<double> extraDouble(const Detection &detection, const std::string &key) {
    for (const auto &entry : detection.extra) {
        if (entry.key != key) {
            continue;
        }
        try {
            return std::stod(entry.value);
        } catch (const std::exception &) {
            return std::nullopt;
        }
    }
    return std::nullopt;
}

std::optional<double> orientationYaw(const Detection &detection) {
    if (const auto yaw_deg = extraDouble(detection, "yaw_deg")) {
        return radians(*yaw_deg);
    }
    for (const auto &entry : detection.extra) {
        if (entry.key == "pose_semantics" && entry.value != "orientation") {
            return std::nullopt;
        }
    }
    if (!detection.pose_valid) {
        return std::nullopt;
    }

    const auto &q = detection.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// A task may publish a calibrated vehicle-yaw correction separately from its
// pose quaternion or image-plane roll. Keep this explicit so generic actions
// never confuse a board's visual twist with a heading command.
std::optional<double> headingYaw(const Detection &detection) {
    if (const auto yaw_deg = extraDouble(detection, "heading_yaw_deg")) {
        return radians(*yaw_deg);
    }
    return std::nullopt;
}

std::array<double, 3> actualPosition(const MissionNode &node) {
    return {node.commanded_pos[0] - node.control_errors[0], node.commanded_pos[1] - node.control_errors[1],
            node.commanded_pos[2] - node.control_errors[2]};
}

double actualYaw(const MissionNode &node) { return node.commanded_att[2] + node.control_errors[8]; }

class LoadModelAction : public BT::StatefulActionNode {
   public:
    LoadModelAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node, rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config), node_(node), logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("camera", "front", "Camera whose vision node loads the model"),
                BT::InputPort<std::string>("task", "Task model to load, e.g. gate"),
                BT::InputPort<int>("timeout_msec", 20000, "Maximum wait for service + load")};
    }

    BT::NodeStatus onStart() override {
        std::string task;
        int timeout_msec = 20000;
        std::string camera = "front";
        getInput("camera", camera);
        getInput("timeout_msec", timeout_msec);
        if (!getInput("task", task) || task.empty()) {
            RCLCPP_ERROR(logger_, "LoadModel needs a task port, e.g. <LoadModel task=\"gate\"/>.");
            return BT::NodeStatus::FAILURE;
        }

        client_ = node_.vision().loadModelClient(camera);
        if (!client_) {
            RCLCPP_ERROR(logger_, "LoadModel: unknown camera '%s'.", camera.c_str());
            return BT::NodeStatus::FAILURE;
        }

        task_ = node_.visionModelTask(task);
        logical_task_ = task;
        camera_ = camera;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(timeout_msec);
        sent_ = false;
        return poll();
    }

    BT::NodeStatus onRunning() override { return poll(); }

    void onHalted() override { dropPendingRequest(); }

   private:
    // No kill-switch guard: loading a model moves nothing, and keeping it
    // legal while killed lets vision_test run on the bench.
    BT::NodeStatus poll() {
        if (!sent_) {
            if (client_->service_is_ready()) {
                auto request = std::make_shared<LoadModel::Request>();
                request->task = task_;
                auto future_and_id = client_->async_send_request(request);
                request_id_ = future_and_id.request_id;
                future_ = future_and_id.future.share();
                sent_ = true;
                RCLCPP_INFO(logger_, "LoadModel: requested '%s' for task '%s' on camera '%s'.", task_.c_str(),
                            logical_task_.c_str(), camera_.c_str());
            } else if (SteadyClock::now() >= deadline_) {
                RCLCPP_ERROR(logger_, "LoadModel '%s' for task '%s': camera '%s' vision service unavailable.",
                             task_.c_str(), logical_task_.c_str(), camera_.c_str());
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }

        if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            const auto response = future_.get();
            if (response->success) {
                RCLCPP_INFO(logger_, "LoadModel: '%s' active for task '%s' (%s, %.1fs).", task_.c_str(),
                            logical_task_.c_str(), response->active_model.c_str(), response->load_time_s);
                return BT::NodeStatus::SUCCESS;
            }
            RCLCPP_ERROR(logger_, "LoadModel '%s' for task '%s' failed: %s", task_.c_str(), logical_task_.c_str(),
                         response->message.c_str());
            return BT::NodeStatus::FAILURE;
        }

        if (SteadyClock::now() >= deadline_) {
            RCLCPP_ERROR(logger_, "LoadModel '%s' for task '%s' timed out waiting for the load to finish.",
                         task_.c_str(), logical_task_.c_str());
            dropPendingRequest();
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void dropPendingRequest() {
        if (sent_ && client_ && future_.valid() &&
            future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            client_->remove_pending_request(request_id_);
        }
        sent_ = false;
    }

    MissionNode &node_;
    rclcpp::Logger logger_;
    rclcpp::Client<LoadModel>::SharedPtr client_;
    std::shared_future<LoadModel::Response::SharedPtr> future_;
    std::int64_t request_id_ = 0;
    std::string task_;
    std::string logical_task_;
    std::string camera_;
    bool sent_ = false;
    SteadyClock::time_point deadline_;
};

class DetectionVisibleCondition : public BT::ConditionNode {
   public:
    DetectionVisibleCondition(const std::string &name, const BT::NodeConfig &config, MissionNode &node)
        : BT::ConditionNode(name, config), node_(node) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = DetectionFilter::ports();
        ports.insert(BT::InputPort<int>("max_age_msec", 1000, "How recent the detection must be"));
        return ports;
    }

    BT::NodeStatus tick() override {
        int max_age_msec = 1000;
        getInput("max_age_msec", max_age_msec);
        const DetectionFilter filter = DetectionFilter::read(*this);
        const auto not_before = SteadyClock::now() - std::chrono::milliseconds(max_age_msec);
        return latestMatch(node_, filter, not_before).detection != nullptr ? BT::NodeStatus::SUCCESS
                                                                           : BT::NodeStatus::FAILURE;
    }

   private:
    MissionNode &node_;
};

class WaitForDetectionAction : public BT::StatefulActionNode {
   public:
    WaitForDetectionAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                           rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config), node_(node), logger_(logger) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = DetectionFilter::ports();
        ports.insert(BT::InputPort<int>("timeout_msec", 15000, "Maximum wait before returning FAILURE"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        int timeout_msec = 15000;
        getInput("timeout_msec", timeout_msec);
        filter_ = DetectionFilter::read(*this);
        started_at_ = SteadyClock::now();
        deadline_ = started_at_ + std::chrono::milliseconds(timeout_msec);
        return check();
    }

    BT::NodeStatus onRunning() override { return check(); }

    void onHalted() override { RCLCPP_INFO(logger_, "WaitForDetection halted."); }

   private:
    // No kill-switch guard (passive, bounded by timeout_msec): still usable
    // for bench tests while the sub is killed.
    BT::NodeStatus check() {
        // Only frames that arrived after this node started count: a stale
        // detection from a previous task must not satisfy the wait.
        const Match match = latestMatch(node_, filter_, started_at_);
        if (match.detection != nullptr) {
            const auto &hypothesis = match.detection->detection.results[0].hypothesis;
            RCLCPP_INFO(logger_, "WaitForDetection: %s -> class=%s score=%.2f bearing=(%.3f, %.3f).",
                        filter_.describe().c_str(), hypothesis.class_id.c_str(), hypothesis.score,
                        match.detection->bearing_horizontal, match.detection->bearing_vertical);
            return BT::NodeStatus::SUCCESS;
        }

        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "WaitForDetection timed out: %s.", filter_.describe().c_str());
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    SteadyClock::time_point started_at_;
    SteadyClock::time_point deadline_;
};

class AlignToDetectionAction : public BT::StatefulActionNode {
   public:
    AlignToDetectionAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
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
        BT::PortsList ports = DetectionFilter::ports();
        ports.insert(BT::InputPort<bool>("yaw", true, "Steer yaw to zero the horizontal bearing"));
        ports.insert(BT::InputPort<bool>("depth", false, "Steer depth to zero the vertical bearing"));
        ports.insert(BT::InputPort<double>("tolerance", 0.05, "Aligned when |bearing| <= tolerance (rad)"));
        ports.insert(BT::InputPort<double>("depth_gain", 1.5, "Meters of depth change per radian of bearing"));
        ports.insert(BT::InputPort<double>("max_depth_step", 0.5, "Depth correction clamp per frame (m)"));
        ports.insert(BT::InputPort<int>("timeout_msec", 30000, "Maximum align time before FAILURE"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        int timeout_msec = 30000;
        getInput("yaw", align_yaw_);
        getInput("depth", align_depth_);
        getInput("tolerance", tolerance_);
        getInput("depth_gain", depth_gain_);
        getInput("max_depth_step", max_depth_step_);
        getInput("timeout_msec", timeout_msec);
        filter_ = DetectionFilter::read(*this);

        if (!align_yaw_ && !align_depth_) {
            RCLCPP_ERROR(logger_, "AlignToDetection has both yaw and depth disabled; nothing to do.");
            return BT::NodeStatus::FAILURE;
        }

        last_processed_ = SteadyClock::now();
        deadline_ = last_processed_ + std::chrono::milliseconds(timeout_msec);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "AlignToDetection failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "AlignToDetection timed out: %s.", filter_.describe().c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Act once per camera frame: only a snapshot newer than the last one
        // we steered on carries new information.
        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection == nullptr) {
            return BT::NodeStatus::RUNNING;
        }
        last_processed_ = match.received_at;

        const double bearing_h = match.detection->bearing_horizontal;
        const double bearing_v = match.detection->bearing_vertical;
        const bool yaw_aligned = !align_yaw_ || std::fabs(bearing_h) <= tolerance_;
        const bool depth_aligned = !align_depth_ || std::fabs(bearing_v) <= tolerance_;
        if (yaw_aligned && depth_aligned) {
            RCLCPP_INFO(logger_, "AlignToDetection: aligned, bearing=(%.3f, %.3f).", bearing_h, bearing_v);
            return BT::NodeStatus::SUCCESS;
        }

        // Absolute targets referenced to the *measured* state (commanded minus
        // tracking error), so re-issuing on every frame stays convergent
        // instead of integrating the correction.
        if (align_yaw_ && !yaw_aligned) {
            const double actual_yaw = actualYaw(node_);
            node_.commanded_att[2] = normalizeAngle(actual_yaw - bearing_h);
            attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
        }
        if (align_depth_ && !depth_aligned) {
            const double actual_z = node_.commanded_pos[2] - node_.control_errors[2];
            const double step = std::clamp(-depth_gain_ * bearing_v, -max_depth_step_, max_depth_step_);
            node_.commanded_pos[2] = actual_z + step;
            position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos));
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { RCLCPP_INFO(logger_, "AlignToDetection halted."); }

   private:
    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    bool align_yaw_ = true;
    bool align_depth_ = false;
    double tolerance_ = 0.05;
    double depth_gain_ = 1.5;
    double max_depth_step_ = 0.5;
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point deadline_;
};

// Center a front-camera ray on a detected point without changing vehicle yaw.
// It is used after a board-normal firing pose is verified: a model-detected
// torpedo hole may be laterally/depth-offset from the board centre, but the
// launcher must keep the board-normal attitude while translating onto its ray.
class LateralAlignToDetectionAction : public BT::StatefulActionNode {
   public:
    LateralAlignToDetectionAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                                  PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                                  rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          position_publisher_(position_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = DetectionFilter::ports();
        ports.insert(BT::InputPort<bool>("use_board_aim_bearing", false,
                                         "Use calibrated fixed-hole bearings published with a board detection"));
        ports.insert(BT::InputPort<bool>("lock_board_aim", false,
                                         "Lock three consistent aim rays into an odom-frame lateral/depth target"));
        ports.insert(BT::InputPort<double>("target_distance", 1.15,
                                           "Camera-to-target plane distance used to project pixel bearings"));
        ports.insert(BT::InputPort<double>("target_offset_right_m", 0.0,
                                           "Fixed target offset to camera-right from the detected item, in meters"));
        ports.insert(BT::InputPort<double>("target_offset_down_m", 0.0,
                                           "Fixed target offset downward from the detected item, in meters"));
        ports.insert(BT::InputPort<double>(
            "tolerance", 0.012, "Centered when horizontal and vertical bearing are within this radian bound"));
        ports.insert(
            BT::InputPort<double>("max_lateral_step", 0.10, "Maximum body-lateral correction per update in meters"));
        ports.insert(BT::InputPort<double>("max_depth_step", 0.08, "Maximum vertical correction per update in meters"));
        ports.insert(
            BT::InputPort<int>("settle_frames", 8, "Consecutive centered model detections required before SUCCESS"));
        ports.insert(BT::InputPort<int>("min_msec", 1500, "Minimum closed-loop alignment time"));
        ports.insert(BT::InputPort<int>("timeout_msec", 15000, "Maximum alignment time before FAILURE"));
        ports.insert(BT::InputPort<int>("update_msec", 300, "Minimum time between translation corrections"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        filter_ = DetectionFilter::read(*this);
        getInput("use_board_aim_bearing", use_board_aim_bearing_);
        getInput("lock_board_aim", lock_board_aim_);
        getInput("target_distance", target_distance_);
        getInput("target_offset_right_m", target_offset_right_m_);
        getInput("target_offset_down_m", target_offset_down_m_);
        getInput("tolerance", tolerance_);
        getInput("max_lateral_step", max_lateral_step_);
        getInput("max_depth_step", max_depth_step_);
        getInput("settle_frames", settle_frames_);
        getInput("min_msec", min_msec_);
        getInput("timeout_msec", timeout_msec_);
        getInput("update_msec", update_msec_);
        if (filter_.task.empty() || filter_.class_id.empty() || target_distance_ <= 0.0 || tolerance_ <= 0.0 ||
            max_lateral_step_ <= 0.0 || max_depth_step_ <= 0.0 || settle_frames_ <= 0 || min_msec_ < 0 ||
            timeout_msec_ <= 0 || update_msec_ <= 0 || !std::isfinite(target_offset_right_m_) ||
            !std::isfinite(target_offset_down_m_)) {
            RCLCPP_ERROR(
                logger_,
                "LateralAlignToDetection needs task, class_id, positive geometry/timing values, and settle_frames.");
            return BT::NodeStatus::FAILURE;
        }
        started_at_ = SteadyClock::now();
        deadline_ = started_at_ + std::chrono::milliseconds(timeout_msec_);
        last_processed_ = started_at_;
        last_commanded_ = started_at_ - std::chrono::milliseconds(update_msec_);
        have_filtered_bearing_ = false;
        have_locked_target_ = false;
        locked_aim_frames_ = 0;
        settled_frames_ = 0;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "LateralAlignToDetection failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "LateralAlignToDetection timed out: %s.", filter_.describe().c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (have_locked_target_) {
            return tickLockedTarget();
        }
        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection == nullptr) {
            return BT::NodeStatus::RUNNING;
        }
        last_processed_ = match.received_at;
        double horizontal = match.detection->bearing_horizontal;
        double vertical = match.detection->bearing_vertical;
        if (use_board_aim_bearing_) {
            const auto aim_horizontal = extraDouble(*match.detection, "aim_bearing_horizontal");
            const auto aim_vertical = extraDouble(*match.detection, "aim_bearing_vertical");
            if (!aim_horizontal || !aim_vertical) {
                locked_aim_frames_ = 0;
                RCLCPP_WARN_THROTTLE(
                    logger_, *clock_, 1000,
                    "LateralAlignToDetection: calibrated board aim point is unavailable; holding position.");
                return BT::NodeStatus::RUNNING;
            }
            horizontal = *aim_horizontal;
            vertical = *aim_vertical;
        }
        // The torp2 model detects the artwork, not the hole itself. At the
        // fixed full-board framing distance the target-hole displacement is a
        // fixed board-plane vector, so convert that vector into the matching
        // camera ray before filtering or commanding motion.
        horizontal += std::atan2(target_offset_right_m_, target_distance_);
        vertical += std::atan2(target_offset_down_m_, target_distance_);
        if (!have_filtered_bearing_) {
            filtered_horizontal_ = horizontal;
            filtered_vertical_ = vertical;
            have_filtered_bearing_ = true;
        } else {
            // Hole boxes are model detections, but a score-order swap or a
            // partial ring must not produce a large sideways shot correction.
            static constexpr double MAX_BEARING_JUMP_RAD = 0.15;
            if (std::fabs(horizontal - filtered_horizontal_) > MAX_BEARING_JUMP_RAD ||
                std::fabs(vertical - filtered_vertical_) > MAX_BEARING_JUMP_RAD) {
                locked_aim_frames_ = 0;
                RCLCPP_WARN_THROTTLE(
                    logger_, *clock_, 1000,
                    "LateralAlignToDetection rejected discontinuous aim bearing (%.3f, %.3f)->(%.3f, %.3f).",
                    filtered_horizontal_, filtered_vertical_, horizontal, vertical);
                return BT::NodeStatus::RUNNING;
            }
            static constexpr double FILTER_ALPHA = 0.25;
            filtered_horizontal_ += FILTER_ALPHA * (horizontal - filtered_horizontal_);
            filtered_vertical_ += FILTER_ALPHA * (vertical - filtered_vertical_);
        }

        if (lock_board_aim_) {
            if (++locked_aim_frames_ < 3) {
                return BT::NodeStatus::RUNNING;
            }
            lockTarget();
            return tickLockedTarget();
        }

        const bool old_enough = SteadyClock::now() - started_at_ >= std::chrono::milliseconds(min_msec_);
        const bool centered =
            std::fabs(filtered_horizontal_) <= tolerance_ && std::fabs(filtered_vertical_) <= tolerance_;
        if (old_enough && centered) {
            ++settled_frames_;
            if (settled_frames_ >= settle_frames_) {
                RCLCPP_INFO(logger_,
                            "LateralAlignToDetection: centered %s after %d settled frame(s), preserving yaw %.3f.",
                            filter_.describe().c_str(), settled_frames_, actualYaw(node_));
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;
        }
        settled_frames_ = 0;

        if (SteadyClock::now() - last_commanded_ < std::chrono::milliseconds(update_msec_)) {
            return BT::NodeStatus::RUNNING;
        }
        const auto measured = actualPosition(node_);
        const double yaw = actualYaw(node_);
        // Positive image-horizontal is camera-right, while mission body y is
        // left. Move the camera to the hole's projected plane position, not
        // yaw the vehicle away from the already-verified board normal.
        const double left_step =
            std::clamp(-target_distance_ * std::tan(filtered_horizontal_), -max_lateral_step_, max_lateral_step_);
        const double z_step =
            std::clamp(-target_distance_ * std::tan(filtered_vertical_), -max_depth_step_, max_depth_step_);
        node_.commanded_pos[0] = measured[0] - std::sin(yaw) * left_step;
        node_.commanded_pos[1] = measured[1] + std::cos(yaw) * left_step;
        node_.commanded_pos[2] = measured[2] + z_step;
        position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos));
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 500,
                             "LateralAlignToDetection: adjusted bearing=(%.3f, %.3f) offset=(%.3f right, %.3f down) "
                             "body_left=%.3f z=%.3f yaw=%.3f.",
                             filtered_horizontal_, filtered_vertical_, target_offset_right_m_, target_offset_down_m_,
                             left_step, z_step, yaw);
        last_commanded_ = SteadyClock::now();
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { RCLCPP_INFO(logger_, "LateralAlignToDetection halted."); }

   private:
    void lockTarget() {
        const auto measured = actualPosition(node_);
        const double yaw = actualYaw(node_);
        const double left_offset = -target_distance_ * std::tan(filtered_horizontal_);
        const double z_offset = -target_distance_ * std::tan(filtered_vertical_);
        locked_target_x_ = measured[0] - std::sin(yaw) * left_offset;
        locked_target_y_ = measured[1] + std::cos(yaw) * left_offset;
        locked_target_z_ = measured[2] + z_offset;
        have_locked_target_ = true;
        settled_frames_ = 0;
        RCLCPP_INFO(logger_,
                    "LateralAlignToDetection: locked three consistent model/PnP aim rays at lateral/depth target "
                    "(%.2f, %.2f, %.2f).",
                    locked_target_x_, locked_target_y_, locked_target_z_);
    }

    BT::NodeStatus tickLockedTarget() {
        const auto measured = actualPosition(node_);
        const double yaw = actualYaw(node_);
        const double dx = locked_target_x_ - measured[0];
        const double dy = locked_target_y_ - measured[1];
        const double left_error = -std::sin(yaw) * dx + std::cos(yaw) * dy;
        const double z_error = locked_target_z_ - measured[2];
        const double position_tolerance = target_distance_ * std::tan(tolerance_);
        const bool old_enough = SteadyClock::now() - started_at_ >= std::chrono::milliseconds(min_msec_);
        if (old_enough && std::fabs(left_error) <= position_tolerance && std::fabs(z_error) <= position_tolerance) {
            if (++settled_frames_ >= settle_frames_) {
                RCLCPP_INFO(logger_,
                            "LateralAlignToDetection: locked aim target reached after %d settled frame(s), preserving "
                            "yaw %.3f.",
                            settled_frames_, yaw);
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;
        }
        settled_frames_ = 0;
        if (SteadyClock::now() - last_commanded_ < std::chrono::milliseconds(update_msec_)) {
            return BT::NodeStatus::RUNNING;
        }
        const double left_step = std::clamp(left_error, -max_lateral_step_, max_lateral_step_);
        const double z_step = std::clamp(z_error, -max_depth_step_, max_depth_step_);
        node_.commanded_pos[0] = measured[0] - std::sin(yaw) * left_step;
        node_.commanded_pos[1] = measured[1] + std::cos(yaw) * left_step;
        node_.commanded_pos[2] = measured[2] + z_step;
        position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos));
        last_commanded_ = SteadyClock::now();
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    SteadyClock::time_point started_at_;
    SteadyClock::time_point deadline_;
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point last_commanded_;
    double target_distance_ = 1.15;
    double target_offset_right_m_ = 0.0;
    double target_offset_down_m_ = 0.0;
    double tolerance_ = 0.012;
    double max_lateral_step_ = 0.10;
    double max_depth_step_ = 0.08;
    double filtered_horizontal_ = 0.0;
    double filtered_vertical_ = 0.0;
    double locked_target_x_ = 0.0;
    double locked_target_y_ = 0.0;
    double locked_target_z_ = 0.0;
    int settle_frames_ = 8;
    int settled_frames_ = 0;
    int min_msec_ = 1500;
    int timeout_msec_ = 15000;
    int update_msec_ = 300;
    int locked_aim_frames_ = 0;
    bool use_board_aim_bearing_ = false;
    bool lock_board_aim_ = false;
    bool have_filtered_bearing_ = false;
    bool have_locked_target_ = false;
};

class ForwardSweepAlignAction : public BT::StatefulActionNode {
   public:
    ForwardSweepAlignAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
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
        BT::PortsList ports = DetectionFilter::ports();
        ports.insert(BT::InputPort<int>("attempts", 4, "Detection frames sampled at each yaw"));
        ports.insert(BT::InputPort<int>("num_sweeps", 3, "Forward sweep passes before FAILURE"));
        ports.insert(BT::InputPort<double>("forward_step", 2.0, "Forward move between sweeps in meters"));
        ports.insert(BT::InputPort<double>("sample_timeout_msec", 2500.0, "Maximum detection sample time per yaw"));
        ports.insert(BT::InputPort<int>("move_timeout_msec", 20000, "Maximum wait for each commanded movement"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        filter_ = DetectionFilter::read(*this);
        getInput("attempts", attempts_);
        getInput("num_sweeps", num_sweeps_);
        getInput("forward_step", forward_step_);
        getInput("sample_timeout_msec", sample_timeout_msec_);
        getInput("move_timeout_msec", move_timeout_msec_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_, "ForwardSweepAlign needs a task port, e.g. <ForwardSweepAlign task=\"gate\"/>.");
            return BT::NodeStatus::FAILURE;
        }
        if (attempts_ <= 0 || num_sweeps_ <= 0) {
            RCLCPP_ERROR(logger_, "ForwardSweepAlign attempts and num_sweeps must be positive.");
            return BT::NodeStatus::FAILURE;
        }

        phase_ = Phase::YAW_MOVE;
        sweep_index_ = 0;
        angle_index_ = 0;
        detection_count_ = 0;
        detection_sum_ = 0.0;
        last_processed_ = SteadyClock::now();
        beginYawMove();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override { return tickImpl(); }

    void onHalted() override { RCLCPP_INFO(logger_, "ForwardSweepAlign halted."); }

   private:
    enum class Phase {
        FORWARD_MOVE,
        YAW_MOVE,
        SAMPLE_DETECTIONS,
        FINAL_ALIGN,
    };

    static double radians(const double degrees) { return degrees * M_PI / 180.0; }

    BT::NodeStatus tickImpl() {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "ForwardSweepAlign failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }

        switch (phase_) {
            case Phase::FORWARD_MOVE:
                return tickForwardMove();
            case Phase::YAW_MOVE:
                return tickYawMove();
            case Phase::SAMPLE_DETECTIONS:
                return tickSampleDetections();
            case Phase::FINAL_ALIGN:
                return tickFinalAlign();
        }
        return BT::NodeStatus::FAILURE;
    }

    void beginForwardMove() {
        const double yaw = node_.commanded_att[2];
        const double dx = std::cos(yaw) * forward_step_;
        const double dy = std::sin(yaw) * forward_step_;
        node_.commanded_pos[0] += dx;
        node_.commanded_pos[1] += dy;
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        phase_ = Phase::FORWARD_MOVE;
        position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos));
        RCLCPP_INFO(logger_, "ForwardSweepAlign: sweep %d/%d moving forward %.2fm.", sweep_index_ + 1, num_sweeps_,
                    forward_step_);
    }

    BT::NodeStatus tickForwardMove() {
        if (motionTimedOut("forward move")) {
            return BT::NodeStatus::FAILURE;
        }
        if (freshAndWithinTolerance(0, POSITION_TOLERANCE) && freshAndWithinTolerance(1, POSITION_TOLERANCE)) {
            angle_index_ = 0;
            beginYawMove();
        }
        return BT::NodeStatus::RUNNING;
    }

    void beginYawMove() {
        static constexpr std::array<double, 6> SWEEP_DEGREES = {0.0, 40.0, 40.0, -160.0, 40.0, 40.0};
        node_.commanded_att[2] = normalizeAngle(node_.commanded_att[2] + radians(SWEEP_DEGREES[angle_index_]));
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        phase_ = Phase::YAW_MOVE;
        attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
        RCLCPP_INFO(logger_, "ForwardSweepAlign: sweep %d/%d yaw step %.1f deg.", sweep_index_ + 1, num_sweeps_,
                    SWEEP_DEGREES[angle_index_]);
    }

    BT::NodeStatus tickYawMove() {
        if (motionTimedOut("yaw move")) {
            return BT::NodeStatus::FAILURE;
        }
        if (freshAndWithinTolerance(8, ANGLE_TOLERANCE)) {
            detection_count_ = 0;
            detection_sum_ = 0.0;
            last_processed_ = SteadyClock::now();
            deadline_ = last_processed_ + std::chrono::milliseconds(static_cast<int>(sample_timeout_msec_));
            phase_ = Phase::SAMPLE_DETECTIONS;
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus tickSampleDetections() {
        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection != nullptr) {
            last_processed_ = match.received_at;
            detection_sum_ += match.detection->bearing_horizontal;
            ++detection_count_;
            if (detection_count_ >= attempts_) {
                const double average_bearing = detection_sum_ / static_cast<double>(detection_count_);
                const double actual_yaw = actualYaw(node_);
                node_.commanded_att[2] = normalizeAngle(actual_yaw - average_bearing);
                start_updates_ = node_.control_error_updates;
                deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
                phase_ = Phase::FINAL_ALIGN;
                attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
                RCLCPP_INFO(logger_, "ForwardSweepAlign: found %s, average bearing %.3f rad.",
                            filter_.describe().c_str(), average_bearing);
            }
            return BT::NodeStatus::RUNNING;
        }

        if (SteadyClock::now() < deadline_) {
            return BT::NodeStatus::RUNNING;
        }

        ++angle_index_;
        if (angle_index_ < SWEEP_COUNT) {
            beginYawMove();
            return BT::NodeStatus::RUNNING;
        }

        ++sweep_index_;
        if (sweep_index_ < num_sweeps_) {
            beginForwardMove();
            return BT::NodeStatus::RUNNING;
        }

        RCLCPP_WARN(logger_, "ForwardSweepAlign: failed to find %s after %d sweeps.", filter_.describe().c_str(),
                    num_sweeps_);
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus tickFinalAlign() {
        if (motionTimedOut("final yaw align")) {
            return BT::NodeStatus::FAILURE;
        }
        if (freshAndWithinTolerance(8, ANGLE_TOLERANCE)) {
            RCLCPP_INFO(logger_, "ForwardSweepAlign: final yaw reached.");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    bool freshAndWithinTolerance(const std::size_t index, const double tolerance) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= tolerance;
    }

    bool motionTimedOut(const char *motion) const {
        if (SteadyClock::now() < deadline_) {
            return false;
        }
        RCLCPP_WARN(logger_, "ForwardSweepAlign: %s timed out.", motion);
        return true;
    }

    static constexpr std::size_t SWEEP_COUNT = 6;

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    Phase phase_ = Phase::YAW_MOVE;
    std::array<std::uint64_t, 12> start_updates_ = {};
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point deadline_;
    int attempts_ = 4;
    int num_sweeps_ = 3;
    int sweep_index_ = 0;
    std::size_t angle_index_ = 0;
    int move_timeout_msec_ = 20000;
    double forward_step_ = 2.0;
    double sample_timeout_msec_ = 2500.0;
    int detection_count_ = 0;
    double detection_sum_ = 0.0;
};

// Rotate in place to acquire the next slalom layer.  Unlike
// ForwardSweepAlign this node never publishes a position command: it is used
// immediately after crossing a layer, while the vehicle is still as far back
// from the next layer as the front-camera FOV allows.  Every heading is
// sampled before committing: this prevents the first edge-of-FOV detection
// from making us skip a left-side pole. Only a single confirmed pole with a
// right-clearance path may command a transit.
class StationarySweepAlignAction : public BT::StatefulActionNode {
   public:
    StationarySweepAlignAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                               PointCmdPublisher::SharedPtr velocity_publisher,
                               QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
                               rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          velocity_publisher_(velocity_publisher),
          attitude_publisher_(attitude_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = DetectionFilter::ports();
        ports.insert(BT::InputPort<int>("attempts", 3, "Stable matching frames required for an acquisition"));
        ports.insert(BT::InputPort<int>("num_sweeps", 1, "Complete stationary left/right sweeps before acquisition"));
        ports.insert(BT::InputPort<int>("required_pole_count", 1, "Pole count required to confirm the selected pole"));
        ports.insert(BT::InputPort<double>("max_left_offset_deg", 45.0,
                                           "Largest safe port-side scan offset before using the least-oblique pole"));
        ports.insert(
            BT::InputPort<double>("sample_timeout_msec", 1800.0, "Maximum detection sampling time at each heading"));
        ports.insert(BT::InputPort<int>("move_timeout_msec", 20000, "Maximum wait for each in-place yaw command"));
        ports.insert(BT::OutputPort<double>("transit_distance_m",
                                            "Vision range to the right-clearance path around the selected pole"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        filter_ = DetectionFilter::read(*this);
        getInput("attempts", attempts_);
        getInput("num_sweeps", num_sweeps_);
        getInput("required_pole_count", required_pole_count_);
        getInput("max_left_offset_deg", max_left_offset_deg_);
        getInput("sample_timeout_msec", sample_timeout_msec_);
        getInput("move_timeout_msec", move_timeout_msec_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_,
                         "StationarySweepAlign needs a task port, e.g. <StationarySweepAlign task=\"slalom\"/>.");
            stopVelocity();
            return BT::NodeStatus::FAILURE;
        }
        if (attempts_ <= 0 || num_sweeps_ <= 0 || required_pole_count_ != 1 || sample_timeout_msec_ <= 0.0 ||
            move_timeout_msec_ <= 0 || max_left_offset_deg_ <= 0.0 || max_left_offset_deg_ > 180.0) {
            RCLCPP_ERROR(logger_, "StationarySweepAlign requires valid timings, one pole, and a safe left-offset limit.");
            stopVelocity();
            return BT::NodeStatus::FAILURE;
        }

        phase_ = Phase::YAW_MOVE;
        sweep_index_ = 0;
        angle_index_ = 0;
        scan_start_yaw_ = actualYaw(node_);
        candidates_.clear();
        observed_task_frames_ = 0;
        target_frames_ = 0;
        complete_target_frames_ = 0;
        last_observed_snapshot_ = SteadyClock::time_point{};
        stopVelocity();
        beginYawMove();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "StationarySweepAlign failed because kill switch is engaged.");
            stopVelocity();
            return BT::NodeStatus::FAILURE;
        }
        switch (phase_) {
            case Phase::YAW_MOVE:
                return tickYawMove();
            case Phase::SAMPLE_DETECTIONS:
                return tickSampleDetections();
            case Phase::FINAL_ALIGN:
                return tickFinalAlign();
        }
        return BT::NodeStatus::FAILURE;
    }

    void onHalted() override {
        stopVelocity();
        RCLCPP_INFO(logger_, "StationarySweepAlign halted.");
    }

   private:
    enum class Phase {
        YAW_MOVE,
        SAMPLE_DETECTIONS,
        FINAL_ALIGN,
    };

    struct Candidate {
        int samples = 0;
        int distance_samples = 0;
        double sin_sum = 0.0;
        double cos_sum = 0.0;
        double distance_sum = 0.0;

        void add(const double yaw, const double distance) {
            sin_sum += std::sin(yaw);
            cos_sum += std::cos(yaw);
            ++samples;
            if (validDistanceValue(distance)) {
                distance_sum += distance;
                ++distance_samples;
            }
        }

        double yaw() const { return std::atan2(sin_sum, cos_sum); }
        std::optional<double> distance() const {
            if (distance_samples <= 0) {
                return std::nullopt;
            }
            return distance_sum / static_cast<double>(distance_samples);
        }
    };

    static double radians(const double degrees) { return degrees * M_PI / 180.0; }

    static int poleCount(const Detection &detection) {
        if (const auto value = extraDouble(detection, "pole_count")) {
            return static_cast<int>(std::lround(*value));
        }
        return 0;
    }

    double targetYaw(const Detection &detection) const {
        const double yaw_offset = orientationYaw(detection).value_or(-detection.bearing_horizontal);
        return normalizeAngle(actualYaw(node_) + yaw_offset);
    }

    static double angularDistance(const double first, const double second) {
        return std::fabs(normalizeAngle(first - second));
    }

    void observeCompleteLayer(const Detection &detection) {
        // A fixed pass is safe only when the selected pole has a finite
        // clearance-path range. Do not create a heading-only target that
        // could later make the vehicle move an arbitrary distance.
        if (!validDistanceValue(detection.distance_m)) {
            return;
        }
        const double yaw = targetYaw(detection);
        // A physical pole remains at the same world yaw while the vehicle
        // samples neighbouring headings. Keep observations of separate gaps
        // apart so we can deliberately choose the left-most one afterwards.
        constexpr double SAME_GAP_TOLERANCE = M_PI / 12.0;  // 15 degrees
        for (Candidate &candidate : candidates_) {
            if (angularDistance(candidate.yaw(), yaw) <= SAME_GAP_TOLERANCE) {
                candidate.add(yaw, detection.distance_m);
                return;
            }
        }
        Candidate candidate;
        candidate.add(yaw, detection.distance_m);
        candidates_.push_back(candidate);
    }

    const Candidate *safestConfirmedCandidate() const {
        const Candidate *best = nullptr;
        const double safe_offset = radians(max_left_offset_deg_);
        for (const Candidate &candidate : candidates_) {
            if (candidate.samples < attempts_ || candidate.distance_samples < attempts_) {
                continue;
            }
            const double offset = normalizeAngle(candidate.yaw() - scan_start_yaw_);
            if (std::fabs(offset) > safe_offset) {
                continue;
            }
            // A staggered course can put the "left-most" visible gap behind
            // another row.  Crossing the most direct confirmed gap is the
            // primary safety criterion; port/left bias is only a tie-breaker.
            // This prevents a large sideways transit into an adjacent pole.
            if (best == nullptr) {
                best = &candidate;
                continue;
            }
            const double best_offset = normalizeAngle(best->yaw() - scan_start_yaw_);
            if (std::fabs(offset) < std::fabs(best_offset) - radians(2.0) ||
                (std::fabs(std::fabs(offset) - std::fabs(best_offset)) <= radians(2.0) && offset > best_offset)) {
                best = &candidate;
            }
        }
        return best;
    }

    void beginYawMove() {
        // Center, port, starboard. These are yaw-only commands: no
        // translation is allowed until the complete sweep identifies a pole.
        // A final return-to-centre is deliberately omitted: final alignment
        // below already turns directly to the selected safe gap, so the
        // extra movement only adds latency without adding coverage.
        static constexpr std::array<double, 3> SWEEP_DEGREES = {0.0, 60.0, -120.0};
        node_.commanded_att[2] = normalizeAngle(node_.commanded_att[2] + radians(SWEEP_DEGREES[angle_index_]));
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        phase_ = Phase::YAW_MOVE;
        attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
        RCLCPP_INFO(logger_, "StationarySweepAlign: sweep %d/%d yaw step %.1f deg (no translation).", sweep_index_ + 1,
                    num_sweeps_, SWEEP_DEGREES[angle_index_]);
    }

    BT::NodeStatus tickYawMove() {
        if (motionTimedOut("yaw move")) {
            return BT::NodeStatus::FAILURE;
        }
        if (freshAndWithinTolerance(8, ANGLE_TOLERANCE)) {
            last_processed_ = SteadyClock::now();
            deadline_ = last_processed_ + std::chrono::milliseconds(static_cast<int>(sample_timeout_msec_));
            phase_ = Phase::SAMPLE_DETECTIONS;
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus tickSampleDetections() {
        // Keep bounded telemetry for an acquisition failure.  This makes a
        // perception/transport fault distinguishable from a genuine "no
        // complete layer" result without changing any motion decision.
        const VisionClient::Snapshot snapshot = node_.vision().latest(filter_.camera);
        if (snapshot.detections && snapshot.received_at > last_observed_snapshot_) {
            last_observed_snapshot_ = snapshot.received_at;
            if (node_.visionTaskMatches(snapshot.detections->task, filter_.task)) {
                ++observed_task_frames_;
            }
        }

        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection != nullptr) {
            last_processed_ = match.received_at;
            ++target_frames_;
            const int count = poleCount(*match.detection);
            if (count >= required_pole_count_) {
                ++complete_target_frames_;
                observeCompleteLayer(*match.detection);
            }
        }

        if (SteadyClock::now() < deadline_) {
            return BT::NodeStatus::RUNNING;
        }

        ++angle_index_;
        if (angle_index_ < SWEEP_COUNT) {
            beginYawMove();
            return BT::NodeStatus::RUNNING;
        }

        ++sweep_index_;
        if (sweep_index_ < num_sweeps_) {
            angle_index_ = 0;
            beginYawMove();
            return BT::NodeStatus::RUNNING;
        }

        if (const Candidate *candidate = safestConfirmedCandidate()) {
            const auto range = candidate->distance();
            if (!range) {
                RCLCPP_WARN(logger_, "StationarySweepAlign: selected gap has no valid vision range.");
                return BT::NodeStatus::FAILURE;
            }
            const double offset = normalizeAngle(candidate->yaw() - scan_start_yaw_);
            setOutput("transit_distance_m", *range);
            RCLCPP_INFO(logger_,
                        "StationarySweepAlign: full scan confirmed %d pole candidate(s); selecting safest %.1f deg "
                        "at %.2fm.",
                        static_cast<int>(candidates_.size()), std::round(offset * 180.0 / M_PI), *range);
            return beginFinalAlign(candidate->yaw(),
                                   "direct confirmed right-clearance path after stationary scan");
        }

        RCLCPP_WARN(logger_,
                    "StationarySweepAlign: no stable selected pole for %s "
                    "(task_frames=%zu, target_frames=%zu, complete_target_frames=%zu).",
                    filter_.describe().c_str(), observed_task_frames_, target_frames_, complete_target_frames_);
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus beginFinalAlign(const double yaw, const char *const reason) {
        node_.commanded_att[2] = normalizeAngle(yaw);
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        phase_ = Phase::FINAL_ALIGN;
        attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
        RCLCPP_INFO(logger_, "StationarySweepAlign: acquired %s; aligning before transit.", reason);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus tickFinalAlign() {
        if (motionTimedOut("final yaw align")) {
            return BT::NodeStatus::FAILURE;
        }
        if (freshAndWithinTolerance(8, ANGLE_TOLERANCE)) {
            RCLCPP_INFO(logger_, "StationarySweepAlign: final yaw reached; beginning visual transit.");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    bool freshAndWithinTolerance(const std::size_t index, const double tolerance) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= tolerance;
    }

    bool motionTimedOut(const char *const motion) const {
        if (SteadyClock::now() < deadline_) {
            return false;
        }
        RCLCPP_WARN(logger_, "StationarySweepAlign: %s timed out.", motion);
        return true;
    }

    void stopVelocity() {
        if (node_.last_velocity_setpoint == std::array<double, 3>{0.0, 0.0, 0.0}) {
            return;
        }
        node_.last_velocity_setpoint = {0.0, 0.0, 0.0};
        velocity_publisher_->publish(linearVelocityCommand(*clock_, node_.last_velocity_setpoint));
    }

    static constexpr std::size_t SWEEP_COUNT = 3;

    MissionNode &node_;
    PointCmdPublisher::SharedPtr velocity_publisher_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    Phase phase_ = Phase::YAW_MOVE;
    std::array<std::uint64_t, 12> start_updates_ = {};
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point deadline_;
    std::vector<Candidate> candidates_;
    SteadyClock::time_point last_observed_snapshot_;
    int attempts_ = 3;
    int num_sweeps_ = 1;
    int required_pole_count_ = 1;
    double max_left_offset_deg_ = 45.0;
    int sweep_index_ = 0;
    std::size_t angle_index_ = 0;
    int move_timeout_msec_ = 20000;
    double sample_timeout_msec_ = 1800.0;
    double scan_start_yaw_ = 0.0;
    std::size_t observed_task_frames_ = 0;
    std::size_t target_frames_ = 0;
    std::size_t complete_target_frames_ = 0;
};

class SweepCheckAction : public BT::StatefulActionNode {
   public:
    SweepCheckAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                     QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
                     rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          attitude_publisher_(attitude_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = DetectionFilter::ports();
        ports.insert(BT::InputPort<int>("attempts", 4, "Detection frames sampled at each yaw"));
        ports.insert(BT::InputPort<double>("sample_timeout_msec", 2500.0, "Maximum detection sample time per yaw"));
        ports.insert(BT::InputPort<int>("move_timeout_msec", 20000, "Maximum wait for each yaw move"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        filter_ = DetectionFilter::read(*this);
        getInput("attempts", attempts_);
        getInput("sample_timeout_msec", sample_timeout_msec_);
        getInput("move_timeout_msec", move_timeout_msec_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_, "SweepCheck needs a task port, e.g. <SweepCheck task=\"gate\"/>.");
            return BT::NodeStatus::FAILURE;
        }
        if (attempts_ <= 0) {
            RCLCPP_ERROR(logger_, "SweepCheck attempts must be positive.");
            return BT::NodeStatus::FAILURE;
        }
        phase_ = Phase::YAW_MOVE;
        angle_index_ = 0;
        detection_count_ = 0;
        detection_sum_ = 0.0;
        last_processed_ = SteadyClock::now();
        beginYawMove();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override { return tickImpl(); }

    void onHalted() override { RCLCPP_INFO(logger_, "SweepCheck halted."); }

   private:
    enum class Phase {
        YAW_MOVE,
        SAMPLE_DETECTIONS,
        FINAL_ALIGN,
    };

    static double radians(const double degrees) { return degrees * M_PI / 180.0; }

    BT::NodeStatus tickImpl() {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "SweepCheck failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        switch (phase_) {
            case Phase::YAW_MOVE:
                return tickYawMove();
            case Phase::SAMPLE_DETECTIONS:
                return tickSampleDetections();
            case Phase::FINAL_ALIGN:
                return tickFinalAlign();
        }
        return BT::NodeStatus::FAILURE;
    }

    void beginYawMove() {
        static constexpr std::array<double, 6> SWEEP_DEGREES = {0.0, 40.0, 40.0, -160.0, 40.0, 40.0};
        node_.commanded_att[2] = normalizeAngle(node_.commanded_att[2] + radians(SWEEP_DEGREES[angle_index_]));
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        phase_ = Phase::YAW_MOVE;
        attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
        RCLCPP_INFO(logger_, "SweepCheck: yaw step %.1f deg.", SWEEP_DEGREES[angle_index_]);
    }

    BT::NodeStatus tickYawMove() {
        if (motionTimedOut("yaw move")) {
            return BT::NodeStatus::FAILURE;
        }
        if (freshAndWithinTolerance(8, ANGLE_TOLERANCE)) {
            detection_count_ = 0;
            detection_sum_ = 0.0;
            last_processed_ = SteadyClock::now();
            deadline_ = last_processed_ + std::chrono::milliseconds(static_cast<int>(sample_timeout_msec_));
            phase_ = Phase::SAMPLE_DETECTIONS;
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus tickSampleDetections() {
        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection != nullptr) {
            last_processed_ = match.received_at;
            detection_sum_ += match.detection->bearing_horizontal;
            ++detection_count_;
            if (detection_count_ >= attempts_) {
                const double average_bearing = detection_sum_ / static_cast<double>(detection_count_);
                const double actual_yaw = actualYaw(node_);
                node_.commanded_att[2] = normalizeAngle(actual_yaw - average_bearing);
                start_updates_ = node_.control_error_updates;
                deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
                phase_ = Phase::FINAL_ALIGN;
                attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
                RCLCPP_INFO(logger_, "SweepCheck: found %s, average bearing %.3f rad.", filter_.describe().c_str(),
                            average_bearing);
            }
            return BT::NodeStatus::RUNNING;
        }
        if (SteadyClock::now() < deadline_) {
            return BT::NodeStatus::RUNNING;
        }

        ++angle_index_;
        if (angle_index_ < SWEEP_COUNT) {
            beginYawMove();
            return BT::NodeStatus::RUNNING;
        }

        RCLCPP_WARN(logger_, "SweepCheck: failed to find %s.", filter_.describe().c_str());
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus tickFinalAlign() {
        if (motionTimedOut("final yaw align")) {
            return BT::NodeStatus::FAILURE;
        }
        if (freshAndWithinTolerance(8, ANGLE_TOLERANCE)) {
            RCLCPP_INFO(logger_, "SweepCheck: final yaw reached.");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    bool freshAndWithinTolerance(const std::size_t index, const double tolerance) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= tolerance;
    }

    bool motionTimedOut(const char *motion) const {
        if (SteadyClock::now() < deadline_) {
            return false;
        }
        RCLCPP_WARN(logger_, "SweepCheck: %s timed out.", motion);
        return true;
    }

    static constexpr std::size_t SWEEP_COUNT = 6;

    MissionNode &node_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    Phase phase_ = Phase::YAW_MOVE;
    std::array<std::uint64_t, 12> start_updates_ = {};
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point deadline_;
    int attempts_ = 4;
    std::size_t angle_index_ = 0;
    int move_timeout_msec_ = 20000;
    double sample_timeout_msec_ = 2500.0;
    int detection_count_ = 0;
    double detection_sum_ = 0.0;
};

class SweepAngleAction : public BT::StatefulActionNode {
   public:
    SweepAngleAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                     QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
                     rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          attitude_publisher_(attitude_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = DetectionFilter::ports();
        ports.insert(BT::InputPort<int>("attempts", 4, "Detection frames sampled at each yaw"));
        ports.insert(BT::InputPort<double>("sample_timeout_msec", 2500.0, "Maximum detection sample time per yaw"));
        ports.insert(BT::InputPort<int>("move_timeout_msec", 20000, "Maximum wait for each yaw move"));
        ports.insert(BT::OutputPort<double>("yaw", "Absolute mission yaw toward the detected object"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        filter_ = DetectionFilter::read(*this);
        getInput("attempts", attempts_);
        getInput("sample_timeout_msec", sample_timeout_msec_);
        getInput("move_timeout_msec", move_timeout_msec_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_, "SweepAngle needs a task port, e.g. <SweepAngle task=\"slalom\"/>.");
            return BT::NodeStatus::FAILURE;
        }
        if (attempts_ <= 0) {
            RCLCPP_ERROR(logger_, "SweepAngle attempts must be positive.");
            return BT::NodeStatus::FAILURE;
        }
        phase_ = Phase::YAW_MOVE;
        angle_index_ = 0;
        detection_count_ = 0;
        detection_sum_ = 0.0;
        last_processed_ = SteadyClock::now();
        beginYawMove();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override { return tickImpl(); }

    void onHalted() override { RCLCPP_INFO(logger_, "SweepAngle halted."); }

   private:
    enum class Phase {
        YAW_MOVE,
        SAMPLE_DETECTIONS,
    };

    BT::NodeStatus tickImpl() {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "SweepAngle failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        switch (phase_) {
            case Phase::YAW_MOVE:
                return tickYawMove();
            case Phase::SAMPLE_DETECTIONS:
                return tickSampleDetections();
        }
        return BT::NodeStatus::FAILURE;
    }

    void beginYawMove() {
        static constexpr std::array<double, 6> SWEEP_DEGREES = {0.0, 40.0, 40.0, -160.0, 40.0, 40.0};
        node_.commanded_att[2] = normalizeAngle(node_.commanded_att[2] + radians(SWEEP_DEGREES[angle_index_]));
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        phase_ = Phase::YAW_MOVE;
        attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
        RCLCPP_INFO(logger_, "SweepAngle: yaw step %.1f deg.", SWEEP_DEGREES[angle_index_]);
    }

    BT::NodeStatus tickYawMove() {
        if (motionTimedOut("yaw move")) {
            return BT::NodeStatus::FAILURE;
        }
        if (freshAndWithinTolerance(8, ANGLE_TOLERANCE)) {
            detection_count_ = 0;
            detection_sum_ = 0.0;
            last_processed_ = SteadyClock::now();
            deadline_ = last_processed_ + std::chrono::milliseconds(static_cast<int>(sample_timeout_msec_));
            phase_ = Phase::SAMPLE_DETECTIONS;
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus tickSampleDetections() {
        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection != nullptr) {
            last_processed_ = match.received_at;
            detection_sum_ += match.detection->bearing_horizontal;
            ++detection_count_;
            if (detection_count_ >= attempts_) {
                const double average_bearing = detection_sum_ / static_cast<double>(detection_count_);
                const double target_yaw = normalizeAngle(actualYaw(node_) - average_bearing);
                setOutput("yaw", target_yaw);
                RCLCPP_INFO(logger_, "SweepAngle: found %s, yaw %.3f rad.", filter_.describe().c_str(), target_yaw);
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;
        }
        if (SteadyClock::now() < deadline_) {
            return BT::NodeStatus::RUNNING;
        }

        ++angle_index_;
        if (angle_index_ < SWEEP_COUNT) {
            beginYawMove();
            return BT::NodeStatus::RUNNING;
        }

        RCLCPP_WARN(logger_, "SweepAngle: failed to find %s.", filter_.describe().c_str());
        return BT::NodeStatus::FAILURE;
    }

    bool freshAndWithinTolerance(const std::size_t index, const double tolerance) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= tolerance;
    }

    bool motionTimedOut(const char *motion) const {
        if (SteadyClock::now() < deadline_) {
            return false;
        }
        RCLCPP_WARN(logger_, "SweepAngle: %s timed out.", motion);
        return true;
    }

    static constexpr std::size_t SWEEP_COUNT = 6;

    MissionNode &node_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    Phase phase_ = Phase::YAW_MOVE;
    std::array<std::uint64_t, 12> start_updates_ = {};
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point deadline_;
    int attempts_ = 4;
    std::size_t angle_index_ = 0;
    int move_timeout_msec_ = 20000;
    double sample_timeout_msec_ = 2500.0;
    int detection_count_ = 0;
    double detection_sum_ = 0.0;
};

class ForwardContinuousAlignAction : public BT::StatefulActionNode {
   public:
    ForwardContinuousAlignAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                                 PointCmdPublisher::SharedPtr velocity_publisher,
                                 QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
                                 rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          velocity_publisher_(velocity_publisher),
          attitude_publisher_(attitude_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = DetectionFilter::ports();
        ports.insert(BT::InputPort<double>("max_dist", 10.0, "Maximum forward travel before SUCCESS"));
        ports.insert(
            BT::InputPort<double>("forward_velocity", 1.0, "Fixed body-forward velocity in meters per second"));
        ports.insert(BT::InputPort<double>("close_distance", 3.0, "Distance that completes the approach"));
        ports.insert(BT::InputPort<int>("detection_loss_msec", 3000, "Target-loss time that completes the approach"));
        ports.insert(BT::InputPort<int>("timeout_msec", 30000, "Maximum align time before SUCCESS"));
        ports.insert(BT::InputPort<int>("confirmation_frames", 1, "Matching frames required before propulsion"));
        ports.insert(BT::InputPort<bool>("continue_on_loss", true, "Continue on last heading after target loss"));
        ports.insert(BT::InputPort<bool>("stop_on_close", true, "Stop immediately when the target is close"));
        ports.insert(
            BT::InputPort<bool>("loss_is_success", false, "Treat post-close target loss as a completed transit"));
        ports.insert(BT::InputPort<double>("loss_min_travel", 0.0,
                                           "Minimum odom travel before post-close target loss can succeed"));
        ports.insert(BT::InputPort<bool>("timeout_is_success", true, "Treat bounded timeout as SUCCESS"));
        ports.insert(BT::InputPort<bool>("max_dist_is_success", true, "Treat maximum travel as SUCCESS"));
        ports.insert(BT::InputPort<double>("max_yaw_step_deg", 45.0, "Maximum heading correction per frame"));
        ports.insert(BT::InputPort<bool>("fixed_distance_from_detection", false,
                                         "Lock the transit distance from a confirmed vision range"));
        ports.insert(BT::InputPort<double>("pass_distance", 0.0,
                                           "Meters to continue beyond the detected gap plane in fixed-distance mode"));
        ports.insert(
            BT::InputPort<double>("min_transit_distance", 0.0, "Minimum safe transit distance in fixed-distance mode"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        filter_ = DetectionFilter::read(*this);
        getInput("max_dist", max_dist_);
        getInput("forward_velocity", forward_velocity_);
        getInput("close_distance", close_distance_);
        getInput("detection_loss_msec", detection_loss_msec_);
        getInput("timeout_msec", timeout_msec_);
        getInput("confirmation_frames", confirmation_frames_);
        getInput("continue_on_loss", continue_on_loss_);
        getInput("stop_on_close", stop_on_close_);
        getInput("loss_is_success", loss_is_success_);
        getInput("loss_min_travel", loss_min_travel_);
        getInput("timeout_is_success", timeout_is_success_);
        getInput("max_dist_is_success", max_dist_is_success_);
        getInput("max_yaw_step_deg", max_yaw_step_deg_);
        getInput("fixed_distance_from_detection", fixed_distance_from_detection_);
        getInput("pass_distance", pass_distance_);
        getInput("min_transit_distance", min_transit_distance_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_,
                         "ForwardContinuousAlign needs a task port, e.g. <ForwardContinuousAlign task=\"gate\"/>.");
            stopVelocity();
            return BT::NodeStatus::FAILURE;
        }
        if (max_dist_ <= 0.0 || forward_velocity_ <= 0.0 || detection_loss_msec_ <= 0 || timeout_msec_ <= 0 ||
            confirmation_frames_ <= 0 || max_yaw_step_deg_ <= 0.0 || loss_min_travel_ < 0.0 || pass_distance_ < 0.0 ||
            min_transit_distance_ < 0.0) {
            RCLCPP_ERROR(logger_,
                         "ForwardContinuousAlign max_dist, forward_velocity, detection_loss_msec, and timeout_msec "
                         "must be "
                         "positive.");
            stopVelocity();
            return BT::NodeStatus::FAILURE;
        }

        const auto initial_pos = horizontalPosition();
        if (!initial_pos) {
            stopVelocity();
            return BT::NodeStatus::FAILURE;
        }
        initial_pos_ = *initial_pos;
        last_processed_ = SteadyClock::now();
        seen_detection_ = false;
        seen_close_target_ = false;
        consecutive_detections_ = 0;
        planned_travel_distance_.reset();
        started_at_ = last_processed_;
        deadline_ = started_at_ + std::chrono::milliseconds(timeout_msec_);
        stopVelocity();
        RCLCPP_INFO(logger_, "ForwardContinuousAlign: waiting for %d confirmed frame(s) of %s.", confirmation_frames_,
                    filter_.describe().c_str());
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "ForwardContinuousAlign failed because kill switch is engaged.");
            stopVelocity();
            return BT::NodeStatus::FAILURE;
        }
        return tickActive();
    }

    void onHalted() override {
        stopVelocity();
        RCLCPP_INFO(logger_, "ForwardContinuousAlign halted.");
    }

   private:
    BT::NodeStatus tickActive() {
        const auto actual_pos = horizontalPosition();
        if (!actual_pos) {
            stopVelocity();
            return BT::NodeStatus::FAILURE;
        }
        const double travel = std::hypot((*actual_pos)[0] - initial_pos_[0], (*actual_pos)[1] - initial_pos_[1]);
        if (planned_travel_distance_ && travel >= *planned_travel_distance_) {
            RCLCPP_INFO(logger_, "ForwardContinuousAlign: completed fixed vision transit of %.2fm.",
                        *planned_travel_distance_);
            stopVelocity();
            return BT::NodeStatus::SUCCESS;
        }
        if (travel >= max_dist_) {
            RCLCPP_INFO(logger_, "ForwardContinuousAlign: max distance %.2fm reached.", max_dist_);
            stopVelocity();
            return max_dist_is_success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "ForwardContinuousAlign: timeout reached after %dms.", timeout_msec_);
            stopVelocity();
            return timeout_is_success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection != nullptr) {
            last_processed_ = match.received_at;
            last_detection_ = SteadyClock::now();
            seen_detection_ = true;
            ++consecutive_detections_;
            steerFromDetection(*match.detection);
            if (consecutive_detections_ >= confirmation_frames_) {
                if (fixed_distance_from_detection_ && !planned_travel_distance_) {
                    if (!validDistance(match.detection->distance_m)) {
                        RCLCPP_WARN(
                            logger_,
                            "ForwardContinuousAlign: fixed transit requires a valid vision distance; stopping.");
                        stopVelocity();
                        return BT::NodeStatus::FAILURE;
                    }
                    const double planned =
                        std::max(min_transit_distance_, match.detection->distance_m + pass_distance_);
                    if (planned > max_dist_) {
                        RCLCPP_WARN(
                            logger_,
                            "ForwardContinuousAlign: vision transit %.2fm exceeds %.2fm safety limit; stopping.",
                            planned, max_dist_);
                        stopVelocity();
                        return BT::NodeStatus::FAILURE;
                    }
                    planned_travel_distance_ = planned;
                    RCLCPP_INFO(
                        logger_,
                        "ForwardContinuousAlign: fixed transit %.2fm from vision range %.2fm + %.2fm pass margin.",
                        planned, match.detection->distance_m, pass_distance_);
                }
                if (!velocity_active_) {
                    startVelocity();
                }
            }
            if (consecutive_detections_ >= confirmation_frames_ && validDistance(match.detection->distance_m) &&
                match.detection->distance_m <= close_distance_) {
                RCLCPP_INFO(logger_, "ForwardContinuousAlign: close target detected at %.2fm.",
                            match.detection->distance_m);
                seen_close_target_ = true;
                if (stop_on_close_) {
                    stopVelocity();
                    return BT::NodeStatus::SUCCESS;
                }
            }
        } else if (seen_detection_ &&
                   SteadyClock::now() - last_detection_ >= std::chrono::milliseconds(detection_loss_msec_)) {
            // A pass may be recognized only after the target was genuinely
            // close, then leaves the front camera's field of view.  This is
            // intentionally different from accepting arbitrary target loss in
            // open water.
            if (loss_is_success_ && seen_close_target_ && travel >= loss_min_travel_) {
                RCLCPP_INFO(logger_,
                            "ForwardContinuousAlign: close target left the front FOV after %.2fm; stopping transit.",
                            travel);
                stopVelocity();
                return BT::NodeStatus::SUCCESS;
            }
            if (continue_on_loss_) {
                RCLCPP_WARN(logger_, "ForwardContinuousAlign: target lost for %dms; continuing on the last heading.",
                            detection_loss_msec_);
            } else {
                RCLCPP_WARN(logger_, "ForwardContinuousAlign: target lost for %dms; stopping for reacquisition.",
                            detection_loss_msec_);
                stopVelocity();
                consecutive_detections_ = 0;
                seen_detection_ = false;
                return BT::NodeStatus::FAILURE;
            }
            seen_detection_ = false;
        }
        return BT::NodeStatus::RUNNING;
    }

    void steerFromDetection(const Detection &detection) {
        const auto actual_yaw = measuredYaw();
        if (!actual_yaw) {
            return;
        }
        // Prefer a post-processor's explicit relative angle setpoint.  The
        // bearing fallback preserves the generic behavior for other tasks.
        const double raw_yaw_offset = orientationYaw(detection).value_or(-detection.bearing_horizontal);
        const double max_step = radians(max_yaw_step_deg_);
        const double yaw_offset = std::clamp(raw_yaw_offset, -max_step, max_step);
        node_.commanded_att[2] = normalizeAngle(*actual_yaw + yaw_offset);
        attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 500,
                             "ForwardContinuousAlign: bearing=%.3f target_yaw=%.3f distance=%.2f.",
                             detection.bearing_horizontal, node_.commanded_att[2], detection.distance_m);
    }

    std::optional<std::array<double, 2>> horizontalPosition() const {
        const std::string ns = node_.get_namespace();
        const std::string prefix = ns.empty() || ns == "/" ? "" : (ns.front() == '/' ? ns.substr(1) : ns) + "/";
        try {
            const auto transform =
                node_.tfBuffer().lookupTransform(prefix + "odom", prefix + "base_link", tf2::TimePointZero);
            return std::array<double, 2>{transform.transform.translation.x, transform.transform.translation.y};
        } catch (const tf2::TransformException &error) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "ForwardContinuousAlign waiting for odom TF: %s",
                                 error.what());
            return std::nullopt;
        }
    }

    std::optional<double> measuredYaw() const {
        const std::string ns = node_.get_namespace();
        const std::string prefix = ns.empty() || ns == "/" ? "" : (ns.front() == '/' ? ns.substr(1) : ns) + "/";
        try {
            const auto transform =
                node_.tfBuffer().lookupTransform(prefix + "odom", prefix + "base_link", tf2::TimePointZero);
            const auto &q = transform.transform.rotation;
            const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            return std::atan2(siny_cosp, cosy_cosp);
        } catch (const tf2::TransformException &error) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "ForwardContinuousAlign waiting for yaw TF: %s", error.what());
            return std::nullopt;
        }
    }

    static bool validDistance(const double distance) { return std::isfinite(distance) && distance > 0.0; }

    void startVelocity() {
        node_.last_velocity_setpoint = {forward_velocity_, 0.0, 0.0};
        velocity_publisher_->publish(linearVelocityCommand(*clock_, node_.last_velocity_setpoint));
        velocity_active_ = true;
        RCLCPP_INFO(logger_, "ForwardContinuousAlign: confirmed target; moving forward at %.2fm/s.", forward_velocity_);
    }

    void stopVelocity() {
        if (!velocity_active_ && node_.last_velocity_setpoint == std::array<double, 3>{0.0, 0.0, 0.0}) {
            return;
        }
        node_.last_velocity_setpoint = {0.0, 0.0, 0.0};
        velocity_publisher_->publish(linearVelocityCommand(*clock_, node_.last_velocity_setpoint));
        velocity_active_ = false;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr velocity_publisher_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    std::array<double, 2> initial_pos_ = {};
    SteadyClock::time_point started_at_;
    SteadyClock::time_point deadline_;
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point last_detection_;
    double max_dist_ = 10.0;
    double forward_velocity_ = 1.0;
    double close_distance_ = 3.0;
    int detection_loss_msec_ = 3000;
    int timeout_msec_ = 30000;
    int confirmation_frames_ = 1;
    int consecutive_detections_ = 0;
    double max_yaw_step_deg_ = 45.0;
    bool continue_on_loss_ = true;
    bool stop_on_close_ = true;
    bool loss_is_success_ = false;
    bool timeout_is_success_ = true;
    bool max_dist_is_success_ = true;
    bool velocity_active_ = false;
    bool seen_detection_ = false;
    bool seen_close_target_ = false;
    double loss_min_travel_ = 0.0;
    bool fixed_distance_from_detection_ = false;
    double pass_distance_ = 0.0;
    double min_transit_distance_ = 0.0;
    std::optional<double> planned_travel_distance_;
};

// A deliberately small slalom transit primitive.  The immediately preceding
// StationarySweepAlign has already verified a pole and a right-clearance path
// at a stable heading and supplied its range, so reacquiring the target while
// passing the pole is redundant and can create a false abort at a low
// camera frame rate.  This node therefore makes exactly one bounded
// body-forward pass, supervised by odometry, timeout, and the kill switch.
class ForwardFixedTransitAction : public BT::StatefulActionNode {
   public:
    ForwardFixedTransitAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                              PointCmdPublisher::SharedPtr velocity_publisher, rclcpp::Clock::SharedPtr clock,
                              rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          velocity_publisher_(velocity_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("vision_distance_m", "Range produced by StationarySweepAlign"),
                BT::InputPort<double>("pass_distance", 0.0, "Meters to travel beyond the detected gap plane"),
                BT::InputPort<double>("min_transit_distance", 0.0, "Minimum safe fixed transit distance in meters"),
                BT::InputPort<double>("completion_tolerance_m", 0.15,
                                      "Odometry shortfall accepted within the pass margin"),
                BT::InputPort<double>("max_dist", 10.0, "Hard maximum forward distance in meters"),
                BT::InputPort<double>("forward_velocity", 0.3, "Body-forward transit velocity in meters per second"),
                BT::InputPort<int>("timeout_msec", 30000, "Maximum transit time before a safe failure"),
                BT::InputPort<double>("min_progress_m", 0.10, "Minimum odometry advance that proves motion"),
                BT::InputPort<int>("progress_timeout_msec", 4000,
                                   "Maximum time without sufficient odometry progress")};
    }

    BT::NodeStatus onStart() override {
        getInput("vision_distance_m", vision_distance_m_);
        getInput("pass_distance", pass_distance_);
        getInput("min_transit_distance", min_transit_distance_);
        getInput("completion_tolerance_m", completion_tolerance_m_);
        getInput("max_dist", max_dist_);
        getInput("forward_velocity", forward_velocity_);
        getInput("timeout_msec", timeout_msec_);
        getInput("min_progress_m", min_progress_m_);
        getInput("progress_timeout_msec", progress_timeout_msec_);

        if (!validDistanceValue(vision_distance_m_) || pass_distance_ < 0.0 || min_transit_distance_ < 0.0 ||
            completion_tolerance_m_ < 0.0 || max_dist_ <= 0.0 || forward_velocity_ <= 0.0 || timeout_msec_ <= 0 ||
            min_progress_m_ <= 0.0 || progress_timeout_msec_ <= 0) {
            RCLCPP_ERROR(logger_,
                         "ForwardFixedTransit received invalid range, distance, velocity, or timeout settings.");
            stopVelocity();
            return BT::NodeStatus::FAILURE;
        }

        planned_distance_ = std::max(min_transit_distance_, vision_distance_m_ + pass_distance_);
        completion_distance_ = std::max(min_transit_distance_, planned_distance_ - completion_tolerance_m_);
        if (planned_distance_ > max_dist_) {
            RCLCPP_WARN(logger_, "ForwardFixedTransit: requested %.2fm exceeds %.2fm safety limit.", planned_distance_,
                        max_dist_);
            stopVelocity();
            return BT::NodeStatus::FAILURE;
        }

        const auto initial_position = horizontalPosition();
        if (!initial_position) {
            stopVelocity();
            return BT::NodeStatus::FAILURE;
        }
        initial_position_ = *initial_position;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(timeout_msec_);
        progress_deadline_ = SteadyClock::now() + std::chrono::milliseconds(progress_timeout_msec_);
        last_progress_travel_ = 0.0;
        node_.last_velocity_setpoint = {forward_velocity_, 0.0, 0.0};
        velocity_publisher_->publish(linearVelocityCommand(*clock_, node_.last_velocity_setpoint));
        last_velocity_publish_ = SteadyClock::now();
        velocity_active_ = true;
        RCLCPP_INFO(logger_, "ForwardFixedTransit: passing right of confirmed pole for %.2fm at %.2fm/s.",
                    planned_distance_, forward_velocity_);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "ForwardFixedTransit failed because kill switch is engaged.");
            stopVelocity();
            return BT::NodeStatus::FAILURE;
        }
        // Hold command ownership for the entire transit. Telemetry/teleop
        // bridges may publish zero velocity keepalives after this action's
        // initial command; without reassertion, one such message silently
        // stops the vehicle and the progress watchdog fires several seconds
        // later. Re-publishing is safe because setpoint updates preserve PID
        // state in sub_control.
        if (SteadyClock::now() - last_velocity_publish_ >= std::chrono::milliseconds(200)) {
            node_.last_velocity_setpoint = {forward_velocity_, 0.0, 0.0};
            velocity_publisher_->publish(linearVelocityCommand(*clock_, node_.last_velocity_setpoint));
            last_velocity_publish_ = SteadyClock::now();
        }
        const auto current_position = horizontalPosition();
        if (!current_position) {
            stopVelocity();
            return BT::NodeStatus::FAILURE;
        }
        const double travel =
            std::hypot((*current_position)[0] - initial_position_[0], (*current_position)[1] - initial_position_[1]);
        if (travel >= completion_distance_) {
            RCLCPP_INFO(logger_, "ForwardFixedTransit: completed %.2fm confirmed-gap pass (planned %.2fm).", travel,
                        planned_distance_);
            stopVelocity();
            return BT::NodeStatus::SUCCESS;
        }
        if (travel >= last_progress_travel_ + min_progress_m_) {
            last_progress_travel_ = travel;
            progress_deadline_ = SteadyClock::now() + std::chrono::milliseconds(progress_timeout_msec_);
        } else if (SteadyClock::now() >= progress_deadline_) {
            RCLCPP_WARN(logger_, "ForwardFixedTransit stopped after %.2fs without %.2fm odometry progress (travel=%.2fm).",
                        progress_timeout_msec_ / 1000.0, min_progress_m_, travel);
            stopVelocity();
            return BT::NodeStatus::FAILURE;
        }
        if (travel >= max_dist_ || SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "ForwardFixedTransit stopped before completing its confirmed-gap pass (travel=%.2fm).",
                        travel);
            stopVelocity();
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { stopVelocity(); }

   private:
    std::optional<std::array<double, 2>> horizontalPosition() const {
        const std::string ns = node_.get_namespace();
        const std::string prefix = ns.empty() || ns == "/" ? "" : (ns.front() == '/' ? ns.substr(1) : ns) + "/";
        try {
            const auto transform =
                node_.tfBuffer().lookupTransform(prefix + "odom", prefix + "base_link", tf2::TimePointZero);
            return std::array<double, 2>{transform.transform.translation.x, transform.transform.translation.y};
        } catch (const tf2::TransformException &error) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "ForwardFixedTransit waiting for odom TF: %s", error.what());
            return std::nullopt;
        }
    }

    void stopVelocity() {
        if (!velocity_active_ && node_.last_velocity_setpoint == std::array<double, 3>{0.0, 0.0, 0.0}) {
            return;
        }
        node_.last_velocity_setpoint = {0.0, 0.0, 0.0};
        velocity_publisher_->publish(linearVelocityCommand(*clock_, node_.last_velocity_setpoint));
        velocity_active_ = false;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr velocity_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<double, 2> initial_position_ = {};
    SteadyClock::time_point deadline_;
    double vision_distance_m_ = std::numeric_limits<double>::quiet_NaN();
    double pass_distance_ = 0.0;
    double min_transit_distance_ = 0.0;
    double completion_tolerance_m_ = 0.15;
    double max_dist_ = 10.0;
    double forward_velocity_ = 0.3;
    double planned_distance_ = 0.0;
    double completion_distance_ = 0.0;
    double min_progress_m_ = 0.10;
    int timeout_msec_ = 30000;
    int progress_timeout_msec_ = 4000;
    double last_progress_travel_ = 0.0;
    bool velocity_active_ = false;
    SteadyClock::time_point progress_deadline_;
    SteadyClock::time_point last_velocity_publish_;
};

class OrientToDetectionAtDistAction : public BT::StatefulActionNode {
   public:
    OrientToDetectionAtDistAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
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
        BT::PortsList ports = DetectionFilter::ports();
        ports.insert(BT::InputPort<double>("desired_distance", 2.0, "Distance to hold from the target in meters"));
        ports.insert(BT::InputPort<double>("max_detection_distance", 0.0,
                                           "Reject pose observations beyond this range; zero disables the bound"));
        ports.insert(BT::InputPort<double>("tolerance", 0.35, "Centered when |horizontal bearing| <= tolerance (rad)"));
        ports.insert(BT::InputPort<double>("distance_tolerance", 0.5, "Distance error allowed at completion (m)"));
        ports.insert(BT::InputPort<double>("orient_tolerance_deg", 5.0, "Orientation error allowed at completion"));
        ports.insert(BT::InputPort<double>("max_position_step", 0.5, "Maximum translation correction per update (m)"));
        ports.insert(BT::InputPort<bool>("use_aim_bearing", false,
                                         "Use calibrated aim_bearing_horizontal metadata when available"));
        ports.insert(
            BT::InputPort<bool>("require_heading", false,
                                "Fail instead of bearing-only fallback when a board-normal heading is unavailable"));
        ports.insert(BT::InputPort<int>("settle_frames", 1,
                                        "Consecutive in-tolerance detection frames required before SUCCESS"));
        ports.insert(BT::InputPort<int>("min_msec", 6500, "Minimum closed-loop alignment time"));
        ports.insert(BT::InputPort<int>("timeout_msec", 30000, "Maximum align time before FAILURE"));
        ports.insert(BT::InputPort<int>("update_msec", 300, "Minimum time between movement corrections"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        filter_ = DetectionFilter::read(*this);
        getInput("desired_distance", desired_distance_);
        getInput("max_detection_distance", max_detection_distance_);
        getInput("tolerance", tolerance_);
        getInput("distance_tolerance", distance_tolerance_);
        getInput("orient_tolerance_deg", orient_tolerance_deg_);
        getInput("max_position_step", max_position_step_);
        getInput("use_aim_bearing", use_aim_bearing_);
        getInput("require_heading", require_heading_);
        getInput("settle_frames", settle_frames_);
        getInput("min_msec", min_msec_);
        getInput("timeout_msec", timeout_msec_);
        getInput("update_msec", update_msec_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_,
                         "OrientToDetectionAtDist needs a task port, e.g. <OrientToDetectionAtDist task=\"torp\"/>.");
            return BT::NodeStatus::FAILURE;
        }
        if (desired_distance_ <= 0.0 || max_detection_distance_ < 0.0 || tolerance_ < 0.0 ||
            distance_tolerance_ < 0.0 || orient_tolerance_deg_ < 0.0 || max_position_step_ <= 0.0 ||
            settle_frames_ <= 0 || update_msec_ <= 0 || min_msec_ < 0 || timeout_msec_ <= 0) {
            RCLCPP_ERROR(logger_,
                         "OrientToDetectionAtDist requires positive geometry/timing values and a non-negative "
                         "max_detection_distance.");
            return BT::NodeStatus::FAILURE;
        }

        last_processed_ = SteadyClock::now();
        last_commanded_ = last_processed_ - std::chrono::milliseconds(update_msec_);
        started_at_ = last_processed_;
        deadline_ = started_at_ + std::chrono::milliseconds(timeout_msec_);
        have_filtered_detection_ = false;
        have_filtered_heading_ = false;
        have_candidate_ = false;
        candidate_frames_ = 0;
        settled_frames_ = 0;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "OrientToDetectionAtDist failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        return tickActive();
    }

    void onHalted() override { RCLCPP_INFO(logger_, "OrientToDetectionAtDist halted."); }

   private:
    struct PlanarPose {
        double x;
        double y;
        double yaw;
    };

    std::optional<PlanarPose> measuredPose() const {
        const std::string ns = node_.get_namespace();
        const std::string prefix = ns.empty() || ns == "/" ? "" : (ns.front() == '/' ? ns.substr(1) : ns) + "/";
        try {
            const auto transform =
                node_.tfBuffer().lookupTransform(prefix + "odom", prefix + "base_link", tf2::TimePointZero);
            const auto &q = transform.transform.rotation;
            const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            return PlanarPose{transform.transform.translation.x, transform.transform.translation.y,
                              std::atan2(siny_cosp, cosy_cosp)};
        } catch (const tf2::TransformException &error) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "OrientToDetectionAtDist waiting for odom TF: %s",
                                 error.what());
            return std::nullopt;
        }
    }

    void clearCandidate() {
        have_candidate_ = false;
        candidate_frames_ = 0;
    }

    bool advanceCandidate(const double bearing, const double distance, const double heading) {
        // A contour/PnP pose arrives at camera rate, but the simulator can
        // occasionally drop frames. Two mutually-consistent observations are
        // enough to reject a one-frame contour error without starving the
        // close-range alignment loop.
        static constexpr int REQUIRED_CANDIDATE_FRAMES = 2;
        static constexpr double MAX_CANDIDATE_RANGE_SPREAD_M = 0.35;
        static constexpr double MAX_CANDIDATE_BEARING_SPREAD_RAD = 0.12;
        static const double MAX_CANDIDATE_HEADING_SPREAD_RAD = radians(15.0);
        const bool consistent =
            have_candidate_ && std::fabs(distance - candidate_distance_) <= MAX_CANDIDATE_RANGE_SPREAD_M &&
            std::fabs(normalizeAngle(bearing - candidate_bearing_)) <= MAX_CANDIDATE_BEARING_SPREAD_RAD &&
            std::fabs(normalizeAngle(heading - candidate_heading_)) <= MAX_CANDIDATE_HEADING_SPREAD_RAD;
        if (!consistent) {
            candidate_bearing_ = bearing;
            candidate_distance_ = distance;
            candidate_heading_ = heading;
            have_candidate_ = true;
            candidate_frames_ = 1;
            return false;
        }
        constexpr double CANDIDATE_ALPHA = 0.5;
        candidate_bearing_ =
            normalizeAngle(candidate_bearing_ + CANDIDATE_ALPHA * normalizeAngle(bearing - candidate_bearing_));
        candidate_distance_ += CANDIDATE_ALPHA * (distance - candidate_distance_);
        candidate_heading_ =
            normalizeAngle(candidate_heading_ + CANDIDATE_ALPHA * normalizeAngle(heading - candidate_heading_));
        return ++candidate_frames_ >= REQUIRED_CANDIDATE_FRAMES;
    }

    BT::NodeStatus tickActive() {
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "OrientToDetectionAtDist timed out: %s.", filter_.describe().c_str());
            return BT::NodeStatus::FAILURE;
        }

        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection == nullptr) {
            return BT::NodeStatus::RUNNING;
        }
        last_processed_ = match.received_at;

        const Detection &detection = *match.detection;
        const auto heading = headingYaw(detection);
        if (require_heading_ && !heading) {
            clearCandidate();
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                                 "OrientToDetectionAtDist: calibrated heading is unavailable; holding position.");
            return BT::NodeStatus::RUNNING;
        }
        const bool has_distance = validDistanceValue(detection.distance_m);
        if (!has_distance && !have_filtered_detection_) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                                 "OrientToDetectionAtDist: waiting for a valid range observation.");
            return BT::NodeStatus::RUNNING;
        }
        const double distance = has_distance ? detection.distance_m : filtered_distance_;
        const double bearing = use_aim_bearing_
                                   ? extraDouble(detection, "aim_bearing_horizontal")
                                         .value_or(detection.bearing_horizontal)
                                   : detection.bearing_horizontal;
        if (max_detection_distance_ > 0.0 && distance > max_detection_distance_) {
            clearCandidate();
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                                 "OrientToDetectionAtDist: rejected %.2fm pose beyond %.2fm close-range bound.",
                                 distance, max_detection_distance_);
            return BT::NodeStatus::RUNNING;
        }
        if (require_heading_) {
            // A complete YOLO board mask normally changes smoothly between
            // consecutive camera frames. Require a compact three-frame
            // cluster before the first command, and re-acquire that way after
            // a rejection. This prevents a one-frame partial mask from
            // becoming a stale reference that rejects the real board.
            static constexpr double MAX_RANGE_JUMP_M = 0.65;
            static constexpr double MAX_BEARING_JUMP_RAD = 0.25;
            static const double MAX_HEADING_JUMP_RAD = radians(18.0);
            const bool relative_to_reference =
                have_filtered_detection_ && std::fabs(distance - filtered_distance_) <= MAX_RANGE_JUMP_M &&
                std::fabs(normalizeAngle(bearing - filtered_bearing_)) <= MAX_BEARING_JUMP_RAD &&
                (!have_filtered_heading_ ||
                 std::fabs(normalizeAngle(*heading - filtered_heading_)) <= MAX_HEADING_JUMP_RAD);
            if (!relative_to_reference) {
                if (advanceCandidate(bearing, distance, *heading)) {
                    filtered_bearing_ = candidate_bearing_;
                    filtered_distance_ = candidate_distance_;
                    filtered_heading_ = candidate_heading_;
                    have_filtered_detection_ = true;
                    have_filtered_heading_ = true;
                    clearCandidate();
                    settled_frames_ = 0;
                    RCLCPP_INFO(logger_, "OrientToDetectionAtDist: accepted a stable three-frame YOLO/PnP board pose.");
                } else {
                    if (have_filtered_detection_) {
                        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                                             "OrientToDetectionAtDist: rejected discontinuous YOLO/PnP observation; "
                                             "waiting for stable re-acquisition.");
                    }
                    return BT::NodeStatus::RUNNING;
                }
            } else {
                clearCandidate();
            }
        }
        if (!have_filtered_detection_) {
            filtered_bearing_ = bearing;
            filtered_distance_ = distance;
            have_filtered_detection_ = true;
        } else {
            // Monocular range and box centers are noisy. Smooth them before
            // turning each camera frame into a continuously moving reference.
            static constexpr double FILTER_ALPHA = 0.25;
            filtered_bearing_ = normalizeAngle(filtered_bearing_ +
                                               FILTER_ALPHA * normalizeAngle(bearing - filtered_bearing_));
            filtered_distance_ += FILTER_ALPHA * (distance - filtered_distance_);
        }
        if (heading) {
            // The pose of a model-derived mask is much more informative than a
            // box, but individual polygon edges can still move a few pixels
            // between frames. Average only the wrapped yaw error so the normal
            // never jumps through +/-pi.
            static constexpr double HEADING_FILTER_ALPHA = 0.20;
            if (!have_filtered_heading_) {
                filtered_heading_ = *heading;
                have_filtered_heading_ = true;
            } else {
                filtered_heading_ = normalizeAngle(filtered_heading_ +
                                                   HEADING_FILTER_ALPHA * normalizeAngle(*heading - filtered_heading_));
            }
        }
        const bool old_enough = SteadyClock::now() - started_at_ >= std::chrono::milliseconds(min_msec_);
        const bool centered = std::fabs(filtered_bearing_) <= tolerance_;
        const bool square = !heading || std::fabs(filtered_heading_) <= radians(orient_tolerance_deg_);
        const bool distance_ok = std::fabs(filtered_distance_ - desired_distance_) <= distance_tolerance_;
        if (old_enough && centered && square && distance_ok) {
            ++settled_frames_;
            if (settled_frames_ >= settle_frames_) {
                RCLCPP_INFO(logger_, "OrientToDetectionAtDist: aligned to %s after %d settled frame(s).",
                            filter_.describe().c_str(), settled_frames_);
                return BT::NodeStatus::SUCCESS;
            }
            // We are already in the measured firing envelope. Hold the last
            // position and attitude command while collecting independent
            // camera confirmations; continuously chasing sub-degree detector
            // noise here causes needless yaw/bearing oscillation.
            return BT::NodeStatus::RUNNING;
        } else {
            settled_frames_ = 0;
        }

        if (SteadyClock::now() - last_commanded_ < std::chrono::milliseconds(update_msec_)) {
            return BT::NodeStatus::RUNNING;
        }

        const auto pose = measuredPose();
        if (!pose) {
            return BT::NodeStatus::RUNNING;
        }

        // Compute one absolute target from one measured-pose snapshot. With a
        // calibrated board normal, solve the desired camera position directly:
        // board centre minus desired stand-off along that normal. This is the
        // lateral-plus-forward correction needed to become perpendicular; a
        // simple range step cannot converge from an off-axis starting pose.
        double forward_step = 0.0;
        double left_step = 0.0;
        const double target_bearing_yaw = normalizeAngle(pose->yaw - filtered_bearing_);
        double target_yaw = target_bearing_yaw;
        if (heading) {
            const double board_forward = filtered_distance_ * std::cos(filtered_bearing_);
            const double board_left = -filtered_distance_ * std::sin(filtered_bearing_);
            forward_step = board_forward - desired_distance_ * std::cos(filtered_heading_);
            left_step = board_left - desired_distance_ * std::sin(filtered_heading_);
            // Hold the centre ray while translating toward the board-normal
            // point. At that point this look-at yaw is exactly perpendicular
            // to the board. Turning directly to the normal while still far
            // off-axis can take the board out of the camera FOV.
        } else {
            forward_step = std::clamp(filtered_distance_ - desired_distance_, -max_position_step_, max_position_step_);
        }
        const double step_norm = std::hypot(forward_step, left_step);
        if (step_norm > max_position_step_) {
            const double scale = max_position_step_ / step_norm;
            forward_step *= scale;
            left_step *= scale;
        }
        node_.commanded_pos[0] = pose->x + std::cos(pose->yaw) * forward_step - std::sin(pose->yaw) * left_step;
        node_.commanded_pos[1] = pose->y + std::sin(pose->yaw) * forward_step + std::cos(pose->yaw) * left_step;
        node_.commanded_att[2] = target_yaw;

        position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos));
        attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 500,
                             "OrientToDetectionAtDist: bearing=%.3f distance=%.2f heading=%s%.3f translate=%s "
                             "target=(%.2f, %.2f, %.3f).",
                             filtered_bearing_, filtered_distance_, heading ? "" : "unavailable/",
                             heading ? filtered_heading_ : 0.0,
                             std::hypot(forward_step, left_step) > 1e-6 ? "yes" : "no", node_.commanded_pos[0],
                             node_.commanded_pos[1], node_.commanded_att[2]);
        last_commanded_ = SteadyClock::now();
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    SteadyClock::time_point started_at_;
    SteadyClock::time_point deadline_;
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point last_commanded_;
    double filtered_bearing_ = 0.0;
    double filtered_distance_ = 0.0;
    double filtered_heading_ = 0.0;
    double candidate_bearing_ = 0.0;
    double candidate_distance_ = 0.0;
    double candidate_heading_ = 0.0;
    double desired_distance_ = 2.0;
    double max_detection_distance_ = 0.0;
    double tolerance_ = 0.35;
    double distance_tolerance_ = 0.5;
    double orient_tolerance_deg_ = 5.0;
    double max_position_step_ = 0.5;
    int min_msec_ = 6500;
    int timeout_msec_ = 30000;
    int update_msec_ = 300;
    int settle_frames_ = 1;
    int settled_frames_ = 0;
    int candidate_frames_ = 0;
    bool have_filtered_detection_ = false;
    bool have_filtered_heading_ = false;
    bool have_candidate_ = false;
    bool require_heading_ = false;
    bool use_aim_bearing_ = false;
};

class DownForwardAlignAction : public BT::StatefulActionNode {
   public:
    DownForwardAlignAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                           PointCmdPublisher::SharedPtr position_publisher,
                           QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock,
                           rclcpp::Logger logger, const bool sweep_when_missing = false)
        : BT::StatefulActionNode(name, config),
          node_(node),
          position_publisher_(position_publisher),
          attitude_publisher_(attitude_publisher),
          clock_(clock),
          logger_(logger),
          sweep_when_missing_(sweep_when_missing) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("task", "", "Only accept detections from this task"),
                BT::InputPort<std::string>("class_id", "", "Only accept this class id (empty = any)"),
                BT::InputPort<double>("min_score", 0.0, "Minimum detection confidence"),
                BT::InputPort<double>("max_dist", 10.0, "Maximum forward travel before SUCCESS"),
                BT::InputPort<double>("forward_step", 1.0, "Forward target extension per update in meters"),
                BT::InputPort<double>("default_distance", 1.0, "Distance used when detection distance is unavailable"),
                BT::InputPort<double>("center_gain", 1.0, "Scale factor for down-camera x/y centering offsets"),
                BT::InputPort<bool>("orient", false, "Follow detection orientation metadata"),
                BT::InputPort<double>("lateral_sweep_step", 1.0,
                                      "Right/left search step when no down-camera detection is visible"),
                BT::InputPort<std::string>("stop_camera", "", "Succeed when this camera sees the stop target"),
                BT::InputPort<std::string>("stop_task", "", "Succeed when this task is detected (empty disables)"),
                BT::InputPort<std::string>("stop_class_id", "", "Only accept this stop-target class id"),
                BT::InputPort<double>("stop_min_score", 0.0, "Minimum stop-target detection confidence"),
                BT::InputPort<int>("update_msec", 300, "Minimum time between movement corrections"),
                BT::InputPort<int>("timeout_msec", 30000, "Maximum align time before SUCCESS")};
    }

    BT::NodeStatus onStart() override {
        filter_.camera = "down";
        getInput("task", filter_.task);
        getInput("class_id", filter_.class_id);
        getInput("min_score", filter_.min_score);
        getInput("max_dist", max_dist_);
        getInput("forward_step", forward_step_);
        getInput("default_distance", default_distance_);
        getInput("center_gain", center_gain_);
        getInput("orient", orient_);
        getInput("lateral_sweep_step", lateral_sweep_step_);
        getInput("stop_camera", stop_filter_.camera);
        getInput("stop_task", stop_filter_.task);
        getInput("stop_class_id", stop_filter_.class_id);
        getInput("stop_min_score", stop_filter_.min_score);
        getInput("update_msec", update_msec_);
        getInput("timeout_msec", timeout_msec_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_, "DownForwardAlign needs a task port, e.g. <DownForwardAlign task=\"bins\"/>.");
            return BT::NodeStatus::FAILURE;
        }
        if (max_dist_ <= 0.0 || forward_step_ <= 0.0 || default_distance_ <= 0.0 || lateral_sweep_step_ <= 0.0 ||
            update_msec_ <= 0) {
            RCLCPP_ERROR(logger_,
                         "DownForwardAlign max_dist, forward_step, default_distance, lateral_sweep_step, and "
                         "update_msec must be positive.");
            return BT::NodeStatus::FAILURE;
        }

        initial_pos_ = actualPosition(node_);
        if (const auto measured = measuredPosition()) {
            initial_pos_ = *measured;
        }
        forward_progress_ = 0.0;
        sweep_direction_ = 1.0;
        last_processed_ = SteadyClock::now();
        last_commanded_ = last_processed_ - std::chrono::milliseconds(update_msec_);
        started_at_ = last_processed_;
        deadline_ = started_at_ + std::chrono::milliseconds(timeout_msec_);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "DownForwardAlign failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        return tickActive();
    }

    void onHalted() override { RCLCPP_INFO(logger_, "DownForwardAlign halted."); }

   private:
    std::optional<std::array<double, 3>> measuredPosition() const {
        const std::string ns = node_.get_namespace();
        const std::string prefix = ns.empty() || ns == "/" ? "" : (ns.front() == '/' ? ns.substr(1) : ns) + "/";
        try {
            const auto transform =
                node_.tfBuffer().lookupTransform(prefix + "odom", prefix + "base_link", tf2::TimePointZero);
            return std::array<double, 3>{transform.transform.translation.x, transform.transform.translation.y,
                                         transform.transform.translation.z};
        } catch (const tf2::TransformException &error) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "DownForwardAlign waiting for odom TF: %s", error.what());
            return std::nullopt;
        }
    }

    BT::NodeStatus tickActive() {
        if (!stop_filter_.task.empty()) {
            const Match stop_match = latestMatch(node_, stop_filter_, SteadyClock::time_point{});
            if (stop_match.detection != nullptr) {
                RCLCPP_INFO(logger_, "DownForwardAlign: stop target found: %s.", stop_filter_.describe().c_str());
                return BT::NodeStatus::SUCCESS;
            }
        }

        const auto measured_pos = measuredPosition();
        if (!measured_pos) {
            return BT::NodeStatus::RUNNING;
        }
        const std::array<double, 3> &pos = *measured_pos;
        if (std::hypot(pos[0] - initial_pos_[0], pos[1] - initial_pos_[1]) >= max_dist_) {
            RCLCPP_INFO(logger_, "DownForwardAlign: max distance %.2fm reached.", max_dist_);
            return sweep_when_missing_ || !stop_filter_.task.empty() ? BT::NodeStatus::FAILURE
                                                                     : BT::NodeStatus::SUCCESS;
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "DownForwardAlign: timeout reached after %dms.", timeout_msec_);
            return sweep_when_missing_ || !stop_filter_.task.empty() ? BT::NodeStatus::FAILURE
                                                                     : BT::NodeStatus::SUCCESS;
        }

        double forward = forward_step_;
        double right = 0.0;
        std::optional<double> yaw_offset;
        const Match match = latestMatch(node_, filter_, sweep_when_missing_ ? SteadyClock::time_point{} : last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection != nullptr) {
            if (sweep_when_missing_) {
                RCLCPP_INFO(logger_, "DownForwardSweepAlign: found %s.", filter_.describe().c_str());
                if (const auto hold = measuredPosition()) {
                    node_.commanded_pos[0] = (*hold)[0];
                    node_.commanded_pos[1] = (*hold)[1];
                    position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos));
                }
                return BT::NodeStatus::SUCCESS;
            }
            last_processed_ = match.received_at;
            const double distance =
                validDistanceValue(match.detection->distance_m) ? match.detection->distance_m : default_distance_;
            // Image-up is vehicle-forward for the down camera. A target above
            // center has a negative vertical bearing, so move forward toward
            // it (the image vertical sign is opposite body-forward).
            forward -= center_gain_ * std::tan(match.detection->bearing_vertical) * distance;
            right = center_gain_ * std::tan(match.detection->bearing_horizontal) * distance;
            if (orient_) {
                yaw_offset = orientationYaw(*match.detection);
            }
        } else if (sweep_when_missing_) {
            right = sweep_direction_ * lateral_sweep_step_;
            sweep_direction_ *= -1.0;
        }

        if (SteadyClock::now() - last_commanded_ >= std::chrono::milliseconds(update_msec_)) {
            if (sweep_when_missing_) {
                // Search around a bounded, absolute track from the position at
                // node start. Building each target on the previous command can
                // run away while the controller is still tracking that command.
                forward_progress_ = std::min(forward_progress_ + forward_step_, max_dist_);
                const double yaw = node_.commanded_att[2];
                node_.commanded_pos[0] = initial_pos_[0] + std::cos(yaw) * forward_progress_ - std::sin(yaw) * right;
                node_.commanded_pos[1] = initial_pos_[1] + std::sin(yaw) * forward_progress_ + std::cos(yaw) * right;
            } else {
                const double yaw = actualYaw(node_);
                node_.commanded_pos[0] = pos[0] + std::cos(yaw) * forward - std::sin(yaw) * right;
                node_.commanded_pos[1] = pos[1] + std::sin(yaw) * forward + std::cos(yaw) * right;
            }
            position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos));
            if (yaw_offset) {
                node_.commanded_att[2] = normalizeAngle(actualYaw(node_) + *yaw_offset);
                attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
            }
            last_commanded_ = SteadyClock::now();
        }
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    DetectionFilter stop_filter_;
    std::array<double, 3> initial_pos_ = {};
    SteadyClock::time_point started_at_;
    SteadyClock::time_point deadline_;
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point last_commanded_;
    double max_dist_ = 10.0;
    double forward_step_ = 1.0;
    double default_distance_ = 1.0;
    double center_gain_ = 1.0;
    bool orient_ = false;
    double lateral_sweep_step_ = 1.0;
    double sweep_direction_ = 1.0;
    double forward_progress_ = 0.0;
    int update_msec_ = 300;
    int timeout_msec_ = 30000;
    bool sweep_when_missing_ = false;
};

class DownSweepAlignAction : public BT::StatefulActionNode {
   public:
    DownSweepAlignAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                         PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                         rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          position_publisher_(position_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("task", "", "Only accept down-camera detections from this task"),
                BT::InputPort<std::string>("class_id", "", "Only accept this class id (empty = any)"),
                BT::InputPort<double>("min_score", 0.0, "Minimum detection confidence"),
                BT::InputPort<double>("lateral_step", 1.0, "Distance from the start point to each side (m)"),
                BT::InputPort<int>("num_sweeps", 3, "Complete right-to-left passes before FAILURE"),
                BT::InputPort<int>("sample_timeout_msec", 1000, "Detection sample time at each side"),
                BT::InputPort<int>("move_timeout_msec", 20000, "Maximum wait for each lateral movement")};
    }

    BT::NodeStatus onStart() override {
        filter_.camera = "down";
        getInput("task", filter_.task);
        getInput("class_id", filter_.class_id);
        getInput("min_score", filter_.min_score);
        getInput("lateral_step", lateral_step_);
        getInput("num_sweeps", num_sweeps_);
        getInput("sample_timeout_msec", sample_timeout_msec_);
        getInput("move_timeout_msec", move_timeout_msec_);
        if (filter_.task.empty() || lateral_step_ <= 0.0 || num_sweeps_ <= 0 || sample_timeout_msec_ <= 0 ||
            move_timeout_msec_ <= 0) {
            RCLCPP_ERROR(logger_, "DownSweepAlign needs a task and positive sweep distances/timings.");
            return BT::NodeStatus::FAILURE;
        }

        initial_pos_ = actualPosition(node_);
        initial_yaw_ = actualYaw(node_);
        completed_sweeps_ = 0;
        target_right_ = true;
        phase_ = Phase::SAMPLE;
        last_processed_ = SteadyClock::now();
        deadline_ = last_processed_ + std::chrono::milliseconds(sample_timeout_msec_);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "DownSweepAlign failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        if (const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
            match.detection != nullptr) {
            RCLCPP_INFO(logger_, "DownSweepAlign: found %s.", filter_.describe().c_str());
            return BT::NodeStatus::SUCCESS;
        }

        if (phase_ == Phase::SAMPLE) {
            if (SteadyClock::now() < deadline_) {
                return BT::NodeStatus::RUNNING;
            }
            if (completed_sweeps_ >= num_sweeps_) {
                RCLCPP_WARN(logger_, "DownSweepAlign: target not found after %d sweep(s).", num_sweeps_);
                return BT::NodeStatus::FAILURE;
            }
            beginMove();
            return BT::NodeStatus::RUNNING;
        }

        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "DownSweepAlign lateral movement timed out.");
            return BT::NodeStatus::FAILURE;
        }
        if (freshAndWithinTolerance(0) && freshAndWithinTolerance(1)) {
            if (!target_right_) {
                ++completed_sweeps_;
            }
            target_right_ = !target_right_;
            phase_ = Phase::SAMPLE;
            deadline_ = SteadyClock::now() + std::chrono::milliseconds(sample_timeout_msec_);
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { RCLCPP_INFO(logger_, "DownSweepAlign halted."); }

   private:
    enum class Phase {
        SAMPLE,
        MOVE,
    };

    void beginMove() {
        const double right = target_right_ ? lateral_step_ : -lateral_step_;
        node_.commanded_pos[0] = initial_pos_[0] - std::sin(initial_yaw_) * right;
        node_.commanded_pos[1] = initial_pos_[1] + std::cos(initial_yaw_) * right;
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        phase_ = Phase::MOVE;
        position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos));
        RCLCPP_INFO(logger_, "DownSweepAlign: moving %s %.2fm.", target_right_ ? "right" : "left", right);
    }

    bool freshAndWithinTolerance(const std::size_t index) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= POSITION_TOLERANCE;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    std::array<double, 3> initial_pos_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point deadline_;
    double initial_yaw_ = 0.0;
    double lateral_step_ = 1.0;
    int num_sweeps_ = 3;
    int completed_sweeps_ = 0;
    int sample_timeout_msec_ = 1000;
    int move_timeout_msec_ = 20000;
    bool target_right_ = true;
    Phase phase_ = Phase::SAMPLE;
};

class DownAlignToDetectionAction : public BT::StatefulActionNode {
   public:
    DownAlignToDetectionAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
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
        return {BT::InputPort<std::string>("task", "", "Only accept down-camera detections from this task"),
                BT::InputPort<std::string>("class_id", "", "Only accept this class id (empty = any)"),
                BT::InputPort<double>("min_score", 0.0, "Minimum detection confidence"),
                BT::InputPort<double>("tolerance", 0.35, "Centered when both bearings are within tolerance (rad)"),
                BT::InputPort<double>("desired_distance", std::numeric_limits<double>::quiet_NaN(),
                                      "Optional camera-to-object distance to hold in meters"),
                BT::InputPort<double>("distance_tolerance", 0.5, "Distance error allowed at completion (m)"),
                BT::InputPort<double>("default_distance", 1.0, "Distance used when detection distance is unavailable"),
                BT::InputPort<double>("center_gain", 1.0, "Scale factor for down-camera x/y centering offsets"),
                BT::InputPort<bool>("orient", true, "Yaw to detection orientation metadata when present"),
                BT::InputPort<bool>(
                    "require_orientation", false,
                    "Fail on timeout instead of accepting center-only alignment when orientation is absent"),
                BT::InputPort<int>("orientation_settle_frames", 3,
                                   "Consecutive consistent orientation frames required before yawing"),
                BT::InputPort<double>("orientation_consistency_deg", 10.0,
                                      "Maximum frame-to-frame axial-angle change accepted as consistent"),
                BT::InputPort<double>("orient_tolerance_deg", 5.0, "Orientation error allowed at completion"),
                BT::InputPort<int>("min_msec", 3000, "Minimum closed-loop alignment time"),
                BT::InputPort<int>("timeout_msec", 30000, "Maximum align time before FAILURE"),
                BT::InputPort<int>("update_msec", 200, "Minimum time between movement corrections")};
    }

    BT::NodeStatus onStart() override {
        filter_.camera = "down";
        getInput("task", filter_.task);
        getInput("class_id", filter_.class_id);
        getInput("min_score", filter_.min_score);
        getInput("tolerance", tolerance_);
        getInput("desired_distance", desired_distance_);
        getInput("distance_tolerance", distance_tolerance_);
        getInput("default_distance", default_distance_);
        getInput("center_gain", center_gain_);
        getInput("orient", orient_);
        getInput("require_orientation", require_orientation_);
        getInput("orientation_settle_frames", orientation_settle_frames_);
        getInput("orientation_consistency_deg", orientation_consistency_deg_);
        getInput("orient_tolerance_deg", orient_tolerance_deg_);
        getInput("min_msec", min_msec_);
        getInput("timeout_msec", timeout_msec_);
        getInput("update_msec", update_msec_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_, "DownAlignToDetection needs a task port, e.g. <DownAlignToDetection task=\"bin\"/>.");
            return BT::NodeStatus::FAILURE;
        }
        if (tolerance_ <= 0.0 || default_distance_ <= 0.0 || update_msec_ <= 0 || orientation_settle_frames_ <= 0 ||
            orientation_consistency_deg_ <= 0.0) {
            RCLCPP_ERROR(logger_,
                         "DownAlignToDetection geometry, orientation filtering, and timing values must be positive.");
            return BT::NodeStatus::FAILURE;
        }

        last_processed_ = SteadyClock::now();
        last_commanded_ = last_processed_ - std::chrono::milliseconds(update_msec_);
        started_at_ = last_processed_;
        deadline_ = started_at_ + std::chrono::milliseconds(timeout_msec_);
        filtered_orientation_.reset();
        consistent_orientation_frames_ = 0;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "DownAlignToDetection failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        return tickActive();
    }

    void onHalted() override { RCLCPP_INFO(logger_, "DownAlignToDetection halted."); }

   private:
    struct MeasuredPose {
        double x;
        double y;
        double z;
        double yaw;
    };

    std::optional<MeasuredPose> measuredPose() const {
        const std::string ns = node_.get_namespace();
        const std::string prefix = ns.empty() || ns == "/" ? "" : (ns.front() == '/' ? ns.substr(1) : ns) + "/";
        try {
            const auto transform =
                node_.tfBuffer().lookupTransform(prefix + "odom", prefix + "base_link", tf2::TimePointZero);
            const auto &q = transform.transform.rotation;
            const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            return MeasuredPose{transform.transform.translation.x, transform.transform.translation.y,
                                transform.transform.translation.z, std::atan2(siny_cosp, cosy_cosp)};
        } catch (const tf2::TransformException &error) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "DownAlignToDetection waiting for odom TF: %s", error.what());
            return std::nullopt;
        }
    }

    std::optional<double> updateOrientation(const std::optional<double> raw_orientation) {
        if (!orient_) {
            return std::nullopt;
        }
        if (!raw_orientation || !std::isfinite(*raw_orientation)) {
            filtered_orientation_.reset();
            consistent_orientation_frames_ = 0;
            if (require_orientation_) {
                RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                                     "DownAlignToDetection: waiting for reliable orientation; continuing centering.");
            }
            return std::nullopt;
        }

        if (!filtered_orientation_) {
            filtered_orientation_ = std::remainder(*raw_orientation, M_PI);
            consistent_orientation_frames_ = 1;
        } else {
            const double delta = std::remainder(*raw_orientation - *filtered_orientation_, M_PI);
            if (std::fabs(delta) > radians(orientation_consistency_deg_)) {
                RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                                     "DownAlignToDetection: rejected orientation jump %.1f deg; rebuilding consensus.",
                                     std::fabs(delta) * 180.0 / M_PI);
                filtered_orientation_ = std::remainder(*raw_orientation, M_PI);
                consistent_orientation_frames_ = 1;
            } else {
                static constexpr double FILTER_ALPHA = 0.45;
                filtered_orientation_ = std::remainder(*filtered_orientation_ + FILTER_ALPHA * delta, M_PI);
                ++consistent_orientation_frames_;
            }
        }

        if (consistent_orientation_frames_ < orientation_settle_frames_) {
            return std::nullopt;
        }
        return filtered_orientation_;
    }

    BT::NodeStatus tickActive() {
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "DownAlignToDetection timed out: %s.", filter_.describe().c_str());
            return BT::NodeStatus::FAILURE;
        }

        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection == nullptr) {
            return BT::NodeStatus::RUNNING;
        }
        last_processed_ = match.received_at;

        const Detection &detection = *match.detection;
        const double distance = validDistanceValue(detection.distance_m) ? detection.distance_m : default_distance_;
        const auto raw_orientation = orientationYaw(detection);
        const auto orient_error = updateOrientation(raw_orientation);
        const bool hold_distance = std::isfinite(desired_distance_);
        const bool centered = std::fabs(detection.bearing_horizontal) <= tolerance_ &&
                              std::fabs(detection.bearing_vertical) <= tolerance_;
        const bool distance_ok = !hold_distance || std::fabs(distance - desired_distance_) <= distance_tolerance_;
        const bool orient_ok = !orient_ ||
                               (orient_error && std::fabs(*orient_error) <= radians(orient_tolerance_deg_)) ||
                               (!require_orientation_ && !orient_error);
        const bool old_enough = SteadyClock::now() - started_at_ >= std::chrono::milliseconds(min_msec_);
        if (old_enough && centered && distance_ok && orient_ok) {
            RCLCPP_INFO(logger_, "DownAlignToDetection: aligned to %s.", filter_.describe().c_str());
            return BT::NodeStatus::SUCCESS;
        }

        if (SteadyClock::now() - last_commanded_ < std::chrono::milliseconds(update_msec_)) {
            return BT::NodeStatus::RUNNING;
        }

        const auto pose = measuredPose();
        if (!pose) {
            return BT::NodeStatus::RUNNING;
        }
        const double yaw = pose->yaw;
        // Image-up is vehicle-forward for the down camera; see the matching
        // sign convention in DownForwardAlignAction.
        const double forward = -center_gain_ * std::tan(detection.bearing_vertical) * distance;
        const double right = center_gain_ * std::tan(detection.bearing_horizontal) * distance;
        node_.commanded_pos[0] = pose->x + std::cos(yaw) * forward - std::sin(yaw) * right;
        node_.commanded_pos[1] = pose->y + std::sin(yaw) * forward + std::cos(yaw) * right;
        if (hold_distance) {
            node_.commanded_pos[2] = pose->z - distance + desired_distance_;
        }
        position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos));

        if (orient_ && orient_error) {
            node_.commanded_att[2] = normalizeAngle(yaw + *orient_error);
            attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
        }

        last_commanded_ = SteadyClock::now();
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    SteadyClock::time_point started_at_;
    SteadyClock::time_point deadline_;
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point last_commanded_;
    double tolerance_ = 0.35;
    double desired_distance_ = std::numeric_limits<double>::quiet_NaN();
    double distance_tolerance_ = 0.5;
    double default_distance_ = 1.0;
    double center_gain_ = 1.0;
    double orient_tolerance_deg_ = 5.0;
    double orientation_consistency_deg_ = 10.0;
    int min_msec_ = 3000;
    int timeout_msec_ = 30000;
    int update_msec_ = 200;
    int orientation_settle_frames_ = 3;
    int consistent_orientation_frames_ = 0;
    bool orient_ = true;
    bool require_orientation_ = false;
    std::optional<double> filtered_orientation_;
};

class DownPatternScanAction : public BT::StatefulActionNode {
   public:
    DownPatternScanAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                          PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                          rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          position_publisher_(position_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("task", "", "Task model to search for with the down camera"),
                BT::InputPort<std::string>("class_id", "", "Only accept this class id (empty = any)"),
                BT::InputPort<double>("min_score", 0.0, "Minimum detection confidence"),
                BT::InputPort<double>("movement_dist", 1.65, "Search-pattern movement distance in meters"),
                BT::InputPort<int>("sample_timeout_msec", 2500, "Maximum detection sample time at each point"),
                BT::InputPort<int>("move_timeout_msec", 20000, "Maximum wait for each pattern move")};
    }

    BT::NodeStatus onStart() override {
        filter_.camera = "down";
        getInput("task", filter_.task);
        getInput("class_id", filter_.class_id);
        getInput("min_score", filter_.min_score);
        getInput("movement_dist", movement_dist_);
        getInput("sample_timeout_msec", sample_timeout_msec_);
        getInput("move_timeout_msec", move_timeout_msec_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_, "DownPatternScan needs a task port, e.g. <DownPatternScan task=\"bin\"/>.");
            return BT::NodeStatus::FAILURE;
        }
        if (movement_dist_ <= 0.0 || sample_timeout_msec_ <= 0 || move_timeout_msec_ <= 0) {
            RCLCPP_ERROR(logger_, "DownPatternScan movement_dist and timeouts must be positive.");
            return BT::NodeStatus::FAILURE;
        }

        initial_pos_ = actualPosition(node_);
        initial_yaw_ = actualYaw(node_);
        pattern_index_ = 0;
        returning_home_ = false;
        last_processed_ = SteadyClock::now();
        phase_ = Phase::SAMPLE;
        deadline_ = last_processed_ + std::chrono::milliseconds(sample_timeout_msec_);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            return BT::NodeStatus::FAILURE;
        }
        if (phase_ == Phase::MOVE) {
            return tickMove();
        }
        return tickSample();
    }

    void onHalted() override { RCLCPP_INFO(logger_, "DownPatternScan halted."); }

   private:
    enum class Phase {
        SAMPLE,
        MOVE,
    };

    static constexpr std::array<std::array<double, 2>, 6> PATTERN = {
        std::array<double, 2>{0.0, 0.0}, std::array<double, 2>{-1.0, 1.0}, std::array<double, 2>{0.0, -2.0},
        std::array<double, 2>{2.0, 0.0}, std::array<double, 2>{0.0, 2.0},  std::array<double, 2>{-1.0, -1.0}};

    BT::NodeStatus tickSample() {
        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection != nullptr) {
            last_processed_ = match.received_at;
            RCLCPP_INFO(logger_, "DownPatternScan: found %s.", filter_.describe().c_str());
            return BT::NodeStatus::SUCCESS;
        }
        if (SteadyClock::now() < deadline_) {
            return BT::NodeStatus::RUNNING;
        }
        ++pattern_index_;
        if (pattern_index_ >= PATTERN.size()) {
            beginMove(initial_pos_);
            returning_home_ = true;
            return BT::NodeStatus::RUNNING;
        }
        beginMove(patternTarget(pattern_index_));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus tickMove() {
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "DownPatternScan move timed out.");
            if (returning_home_) {
                return BT::NodeStatus::FAILURE;
            }
            phase_ = Phase::SAMPLE;
            last_processed_ = SteadyClock::now();
            deadline_ = last_processed_ + std::chrono::milliseconds(sample_timeout_msec_);
            return BT::NodeStatus::RUNNING;
        }
        if (freshAndWithinTolerance(0, POSITION_TOLERANCE) && freshAndWithinTolerance(1, POSITION_TOLERANCE)) {
            if (returning_home_) {
                return BT::NodeStatus::FAILURE;
            }
            phase_ = Phase::SAMPLE;
            last_processed_ = SteadyClock::now();
            deadline_ = last_processed_ + std::chrono::milliseconds(sample_timeout_msec_);
        }
        return BT::NodeStatus::RUNNING;
    }

    std::array<double, 3> patternTarget(const std::size_t index) const {
        const double forward = PATTERN[index][0] * movement_dist_;
        const double right = PATTERN[index][1] * movement_dist_;
        return {initial_pos_[0] + std::cos(initial_yaw_) * forward - std::sin(initial_yaw_) * right,
                initial_pos_[1] + std::sin(initial_yaw_) * forward + std::cos(initial_yaw_) * right,
                node_.commanded_pos[2]};
    }

    void beginMove(const std::array<double, 3> &target) {
        node_.commanded_pos[0] = target[0];
        node_.commanded_pos[1] = target[1];
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        phase_ = Phase::MOVE;
        position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos));
        RCLCPP_INFO(logger_, "DownPatternScan: moving to pattern point %zu.", pattern_index_);
    }

    bool freshAndWithinTolerance(const std::size_t index, const double tolerance) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= tolerance;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    std::array<double, 3> initial_pos_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point deadline_;
    Phase phase_ = Phase::SAMPLE;
    std::size_t pattern_index_ = 0;
    double initial_yaw_ = 0.0;
    double movement_dist_ = 1.65;
    int sample_timeout_msec_ = 2500;
    int move_timeout_msec_ = 20000;
    bool returning_home_ = false;
};

}  // namespace

void registerVisionNodes(BT::BehaviorTreeFactory &factory, MissionNode &node, const rclcpp::Logger logger,
                         PointCmdPublisher::SharedPtr position_publisher,
                         PointCmdPublisher::SharedPtr linear_velocity_publisher,
                         QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock) {
    factory.registerBuilder<LoadModelAction>("LoadModel",
                                             [&node, logger](const std::string &name, const BT::NodeConfig &config) {
                                                 return std::make_unique<LoadModelAction>(name, config, node, logger);
                                             });

    factory.registerBuilder<DetectionVisibleCondition>(
        "DetectionVisible", [&node](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<DetectionVisibleCondition>(name, config, node);
        });

    factory.registerBuilder<WaitForDetectionAction>(
        "WaitForDetection", [&node, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<WaitForDetectionAction>(name, config, node, logger);
        });

    factory.registerBuilder<AlignToDetectionAction>(
        "AlignToDetection", [&node, position_publisher, attitude_publisher, clock, logger](
                                const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<AlignToDetectionAction>(name, config, node, position_publisher, attitude_publisher,
                                                            clock, logger);
        });

    factory.registerBuilder<LateralAlignToDetectionAction>(
        "LateralAlignToDetection",
        [&node, position_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<LateralAlignToDetectionAction>(name, config, node, position_publisher, clock,
                                                                   logger);
        });

    factory.registerBuilder<ForwardSweepAlignAction>(
        "ForwardSweepAlign", [&node, position_publisher, attitude_publisher, clock, logger](
                                 const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<ForwardSweepAlignAction>(name, config, node, position_publisher, attitude_publisher,
                                                             clock, logger);
        });

    factory.registerBuilder<StationarySweepAlignAction>(
        "StationarySweepAlign", [&node, linear_velocity_publisher, attitude_publisher, clock, logger](
                                    const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<StationarySweepAlignAction>(name, config, node, linear_velocity_publisher,
                                                                attitude_publisher, clock, logger);
        });

    factory.registerBuilder<ForwardContinuousAlignAction>(
        "ForwardContinuousAlign", [&node, linear_velocity_publisher, attitude_publisher, clock, logger](
                                      const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<ForwardContinuousAlignAction>(name, config, node, linear_velocity_publisher,
                                                                  attitude_publisher, clock, logger);
        });

    factory.registerBuilder<ForwardFixedTransitAction>(
        "ForwardFixedTransit",
        [&node, linear_velocity_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<ForwardFixedTransitAction>(name, config, node, linear_velocity_publisher, clock,
                                                               logger);
        });

    factory.registerBuilder<OrientToDetectionAtDistAction>(
        "OrientToDetectionAtDist", [&node, position_publisher, attitude_publisher, clock, logger](
                                       const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<OrientToDetectionAtDistAction>(name, config, node, position_publisher,
                                                                   attitude_publisher, clock, logger);
        });

    auto down_forward_align_builder = [&node, position_publisher, attitude_publisher, clock, logger](
                                          const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<DownForwardAlignAction>(name, config, node, position_publisher, attitude_publisher,
                                                        clock, logger);
    };
    factory.registerBuilder<DownForwardAlignAction>("DownForwardAlign", down_forward_align_builder);
    factory.registerBuilder<DownForwardAlignAction>(
        "DownForwardSweepAlign", [&node, position_publisher, attitude_publisher, clock, logger](
                                     const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<DownForwardAlignAction>(name, config, node, position_publisher, attitude_publisher,
                                                            clock, logger, true);
        });

    factory.registerBuilder<DownSweepAlignAction>(
        "DownSweepAlign",
        [&node, position_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<DownSweepAlignAction>(name, config, node, position_publisher, clock, logger);
        });

    factory.registerBuilder<DownAlignToDetectionAction>(
        "DownAlignToDetection", [&node, position_publisher, attitude_publisher, clock, logger](
                                    const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<DownAlignToDetectionAction>(name, config, node, position_publisher,
                                                                attitude_publisher, clock, logger);
        });
    factory.registerBuilder<DownAlignToDetectionAction>(
        "DownContinuousAlign", [&node, position_publisher, attitude_publisher, clock, logger](
                                   const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<DownAlignToDetectionAction>(name, config, node, position_publisher,
                                                                attitude_publisher, clock, logger);
        });
    factory.registerBuilder<DownPatternScanAction>(
        "DownPatternScan",
        [&node, position_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<DownPatternScanAction>(name, config, node, position_publisher, clock, logger);
        });

    factory.registerBuilder<SweepCheckAction>("SweepCheck", [&node, attitude_publisher, clock, logger](
                                                                const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<SweepCheckAction>(name, config, node, attitude_publisher, clock, logger);
    });

    factory.registerBuilder<SweepAngleAction>("SweepAngle", [&node, attitude_publisher, clock, logger](
                                                                const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<SweepAngleAction>(name, config, node, attitude_publisher, clock, logger);
    });
}
