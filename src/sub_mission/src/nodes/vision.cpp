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

// Object yaw relative to the sub's heading, CCW-positive (REP-103) like every
// other angle in the mission.
std::optional<double> orientationYaw(const Detection &detection) {
    if (const auto yaw_deg = extraDouble(detection, "yaw_deg")) {
        return radians(*yaw_deg);
    }
    if (!detection.pose_valid) {
        return std::nullopt;
    }

    const auto &q = detection.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

std::array<double, 3> actualPosition(const MissionNode &node) {
    return {node.commanded_pos[0] - node.control_errors[0], node.commanded_pos[1] - node.control_errors[1],
            node.commanded_pos[2] - node.control_errors[2]};
}

double actualYaw(const MissionNode &node) { return node.commanded_att[2] - node.control_errors[8]; }

// Rotates a body-frame FLU offset (forward, left) by yaw into the ENU world
// frame and retargets the position command from the measured position.
void addBodyOffsetToCommand(MissionNode &node, const double forward, const double left, const double yaw) {
    const std::array<double, 3> pos = actualPosition(node);
    node.commanded_pos[0] = pos[0] + std::cos(yaw) * forward - std::sin(yaw) * left;
    node.commanded_pos[1] = pos[1] + std::sin(yaw) * forward + std::cos(yaw) * left;
}

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
        // instead of integrating the correction. Bearings are camera-optical
        // (positive = right of / below center), so ENU yaw and z move opposite
        // to the bearing sign.
        if (align_yaw_ && !yaw_aligned) {
            const double actual_yaw = node_.commanded_att[2] - node_.control_errors[8];
            node_.commanded_att[2] = normalizeAngle(actual_yaw - bearing_h);
            attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
        }
        if (align_depth_ && !depth_aligned) {
            const double actual_z = node_.commanded_pos[2] - node_.control_errors[2];
            const double step = std::clamp(depth_gain_ * bearing_v, -max_depth_step_, max_depth_step_);
            node_.commanded_pos[2] = actual_z - step;
            position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos, false));
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

    void onHalted() override {
        RCLCPP_INFO(logger_, "ForwardSweepAlign halted.");
    }

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
        position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos, false));
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
        // Starboard-first sweep; ENU yaw is CCW-positive, so turning right is negative.
        static constexpr std::array<double, 6> SWEEP_DEGREES = {0.0, -40.0, -40.0, 160.0, -40.0, -40.0};
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
                const double actual_yaw = node_.commanded_att[2] - node_.control_errors[8];
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

    void onHalted() override {
        RCLCPP_INFO(logger_, "SweepCheck halted.");
    }

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
        // Starboard-first sweep; ENU yaw is CCW-positive, so turning right is negative.
        static constexpr std::array<double, 6> SWEEP_DEGREES = {0.0, -40.0, -40.0, 160.0, -40.0, -40.0};
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
                const double actual_yaw = node_.commanded_att[2] - node_.control_errors[8];
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

    void onHalted() override {
        RCLCPP_INFO(logger_, "SweepAngle halted.");
    }

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
        // Starboard-first sweep; ENU yaw is CCW-positive, so turning right is negative.
        static constexpr std::array<double, 6> SWEEP_DEGREES = {0.0, -40.0, -40.0, 160.0, -40.0, -40.0};
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

class ForwardAlignAction : public BT::StatefulActionNode {
   public:
    ForwardAlignAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
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
        ports.insert(BT::InputPort<double>("max_dist", 10.0, "Maximum forward travel before SUCCESS"));
        ports.insert(BT::InputPort<double>("forward_step", 2.0, "Forward target extension per update in meters"));
        ports.insert(BT::InputPort<double>("close_distance", 3.0,
                                           "Distance that completes the approach or triggers the pass-through"));
        ports.insert(BT::InputPort<double>("through_distance", 0.0,
                                           "Extra travel past the detected object; 0 stops at close_distance"));
        ports.insert(BT::InputPort<double>("depth_offset", 0.25, "Extra depth added when tracking a gate-like target"));
        ports.insert(BT::InputPort<bool>("align_depth", true, "Adjust depth from vertical bearing and distance"));
        ports.insert(BT::InputPort<int>("update_msec", 300, "Minimum time between forward target updates"));
        ports.insert(BT::InputPort<int>("timeout_msec", 30000, "Maximum align time before SUCCESS"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        filter_ = DetectionFilter::read(*this);
        getInput("max_dist", max_dist_);
        getInput("forward_step", forward_step_);
        getInput("close_distance", close_distance_);
        getInput("through_distance", through_distance_);
        getInput("depth_offset", depth_offset_);
        getInput("align_depth", align_depth_);
        getInput("update_msec", update_msec_);
        getInput("timeout_msec", timeout_msec_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_, "ForwardAlign needs a task port, e.g. <ForwardAlign task=\"gate\"/>.");
            return BT::NodeStatus::FAILURE;
        }
        if (max_dist_ <= 0.0 || forward_step_ <= 0.0 || update_msec_ <= 0) {
            RCLCPP_ERROR(logger_, "ForwardAlign max_dist, forward_step, and update_msec must be positive.");
            return BT::NodeStatus::FAILURE;
        }

        final_move_ = false;
        initial_pos_ = actualPosition();
        last_processed_ = SteadyClock::now();
        last_commanded_ = last_processed_ - std::chrono::milliseconds(update_msec_);
        started_at_ = last_processed_;
        deadline_ = started_at_ + std::chrono::milliseconds(timeout_msec_);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "ForwardAlign failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        return final_move_ ? tickFinalMove() : tickActive();
    }

    void onHalted() override { RCLCPP_INFO(logger_, "ForwardAlign halted."); }

   private:
    BT::NodeStatus tickActive() {
        const std::array<double, 3> actual_pos = actualPosition();
        if (horizontalDistance(initial_pos_, actual_pos) >= max_dist_) {
            RCLCPP_INFO(logger_, "ForwardAlign: max distance %.2fm reached.", max_dist_);
            return BT::NodeStatus::SUCCESS;
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "ForwardAlign: timeout reached after %dms.", timeout_msec_);
            return BT::NodeStatus::SUCCESS;
        }

        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection != nullptr) {
            last_processed_ = match.received_at;
            steerFromDetection(*match.detection);
            if (validDistance(match.detection->distance_m) && match.detection->distance_m <= close_distance_) {
                if (through_distance_ > 0.0) {
                    beginFinalMove(match.detection->distance_m + through_distance_);
                    return BT::NodeStatus::RUNNING;
                }
                RCLCPP_INFO(logger_, "ForwardAlign: close target detected at %.2fm.", match.detection->distance_m);
                return BT::NodeStatus::SUCCESS;
            }
        }

        if (SteadyClock::now() - last_commanded_ >= std::chrono::milliseconds(update_msec_)) {
            commandForward(forward_step_);
        }
        return BT::NodeStatus::RUNNING;
    }

    // Blind pass-through: one absolute target through_distance past the
    // detected object, then wait for convergence. The camera loses the target
    // at close range, so this leg cannot keep steering from detections.
    void beginFinalMove(const double distance) {
        commandForward(distance);
        final_move_ = true;
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(timeout_msec_);
        RCLCPP_INFO(logger_, "ForwardAlign: close target detected, final pass-through %.2fm.", distance);
    }

    BT::NodeStatus tickFinalMove() {
        if (freshAndWithinTolerance(0, POSITION_TOLERANCE) && freshAndWithinTolerance(1, POSITION_TOLERANCE)) {
            RCLCPP_INFO(logger_, "ForwardAlign: final pass-through reached.");
            return BT::NodeStatus::SUCCESS;
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "ForwardAlign: final pass-through timed out.");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void steerFromDetection(const Detection &detection) {
        const double actual_yaw = node_.commanded_att[2] - node_.control_errors[8];
        node_.commanded_att[2] = normalizeAngle(actual_yaw - detection.bearing_horizontal);
        attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));

        if (align_depth_ && validDistance(detection.distance_m)) {
            const double actual_z = node_.commanded_pos[2] - node_.control_errors[2];
            // Positive vertical bearing = target below center; ENU z drops.
            const double z_offset = std::sin(detection.bearing_vertical) * detection.distance_m;
            node_.commanded_pos[2] = actual_z - z_offset - depth_offset_;
        }
    }

    void commandForward(const double distance) {
        const double yaw = node_.commanded_att[2];
        const std::array<double, 3> actual_pos = actualPosition();
        node_.commanded_pos[0] = actual_pos[0] + std::cos(yaw) * distance;
        node_.commanded_pos[1] = actual_pos[1] + std::sin(yaw) * distance;
        position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos, false));
        last_commanded_ = SteadyClock::now();
    }

    std::array<double, 3> actualPosition() const {
        return {node_.commanded_pos[0] - node_.control_errors[0], node_.commanded_pos[1] - node_.control_errors[1],
                node_.commanded_pos[2] - node_.control_errors[2]};
    }

    static bool validDistance(const double distance) { return std::isfinite(distance) && distance > 0.0; }

    static double horizontalDistance(const std::array<double, 3> &a, const std::array<double, 3> &b) {
        return std::hypot(a[0] - b[0], a[1] - b[1]);
    }

    bool freshAndWithinTolerance(const std::size_t index, const double tolerance) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= tolerance;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    std::array<double, 3> initial_pos_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
    SteadyClock::time_point started_at_;
    SteadyClock::time_point deadline_;
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point last_commanded_;
    double max_dist_ = 10.0;
    double forward_step_ = 2.0;
    double close_distance_ = 3.0;
    double through_distance_ = 0.0;
    double depth_offset_ = 0.25;
    bool align_depth_ = true;
    bool final_move_ = false;
    int update_msec_ = 300;
    int timeout_msec_ = 30000;
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
        ports.insert(BT::InputPort<double>("tolerance", 0.35, "Centered when |horizontal bearing| <= tolerance (rad)"));
        ports.insert(BT::InputPort<double>("distance_tolerance", 0.5, "Distance error allowed at completion (m)"));
        ports.insert(BT::InputPort<double>("orient_tolerance_deg", 5.0, "Orientation error allowed at completion"));
        ports.insert(BT::InputPort<int>("min_msec", 6500, "Minimum closed-loop alignment time"));
        ports.insert(BT::InputPort<int>("timeout_msec", 30000, "Maximum align time before FAILURE"));
        ports.insert(BT::InputPort<int>("update_msec", 300, "Minimum time between movement corrections"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        filter_ = DetectionFilter::read(*this);
        getInput("desired_distance", desired_distance_);
        getInput("tolerance", tolerance_);
        getInput("distance_tolerance", distance_tolerance_);
        getInput("orient_tolerance_deg", orient_tolerance_deg_);
        getInput("min_msec", min_msec_);
        getInput("timeout_msec", timeout_msec_);
        getInput("update_msec", update_msec_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_,
                         "OrientToDetectionAtDist needs a task port, e.g. <OrientToDetectionAtDist task=\"torp\"/>.");
            return BT::NodeStatus::FAILURE;
        }
        if (desired_distance_ <= 0.0 || update_msec_ <= 0 || min_msec_ < 0) {
            RCLCPP_ERROR(logger_, "OrientToDetectionAtDist desired_distance and update_msec must be positive.");
            return BT::NodeStatus::FAILURE;
        }

        last_processed_ = SteadyClock::now();
        last_commanded_ = last_processed_ - std::chrono::milliseconds(update_msec_);
        started_at_ = last_processed_;
        deadline_ = started_at_ + std::chrono::milliseconds(timeout_msec_);
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
        const auto orient = orientationYaw(detection);
        if (!orient) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "OrientToDetectionAtDist: %s has no yaw_deg or valid pose.",
                                 filter_.describe().c_str());
            return BT::NodeStatus::RUNNING;
        }
        const double distance = validDistanceValue(detection.distance_m) ? detection.distance_m : desired_distance_;
        const bool old_enough = SteadyClock::now() - started_at_ >= std::chrono::milliseconds(min_msec_);
        const bool centered = std::fabs(detection.bearing_horizontal) <= tolerance_;
        const bool square = std::fabs(*orient) <= radians(orient_tolerance_deg_);
        const bool distance_ok = std::fabs(distance - desired_distance_) <= distance_tolerance_;
        if (old_enough && centered && square && distance_ok) {
            RCLCPP_INFO(logger_, "OrientToDetectionAtDist: aligned to %s.", filter_.describe().c_str());
            return BT::NodeStatus::SUCCESS;
        }

        if (SteadyClock::now() - last_commanded_ < std::chrono::milliseconds(update_msec_)) {
            return BT::NodeStatus::RUNNING;
        }

        const double yaw = actualYaw(node_);
        // Positive horizontal bearing = target to the right = negative FLU left offset.
        const double lateral = std::sin(detection.bearing_horizontal) * std::cos(detection.bearing_vertical) * distance;
        addBodyOffsetToCommand(node_, 0.0, -lateral, yaw);

        const double theta = *orient > 0.0 ? *orient - M_PI_2 : *orient + M_PI_2;
        const double triangle_forward = std::sin(std::fabs(*orient)) * distance;
        addBodyOffsetToCommand(node_, triangle_forward, 0.0, yaw + theta);

        addBodyOffsetToCommand(node_, distance - desired_distance_, 0.0, yaw + *orient);
        node_.commanded_att[2] = normalizeAngle(yaw + *orient);

        position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos, false));
        attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
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
    double desired_distance_ = 2.0;
    double tolerance_ = 0.35;
    double distance_tolerance_ = 0.5;
    double orient_tolerance_deg_ = 5.0;
    int min_msec_ = 6500;
    int timeout_msec_ = 30000;
    int update_msec_ = 300;
};

class DownForwardAlignAction : public BT::StatefulActionNode {
   public:
    DownForwardAlignAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                           PointCmdPublisher::SharedPtr position_publisher, rclcpp::Clock::SharedPtr clock,
                           rclcpp::Logger logger, const bool sweep_when_missing = false)
        : BT::StatefulActionNode(name, config),
          node_(node),
          position_publisher_(position_publisher),
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
                BT::InputPort<double>("lateral_sweep_step", 1.0,
                                      "Right/left search step when no down-camera detection is visible"),
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
        getInput("lateral_sweep_step", lateral_sweep_step_);
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
    BT::NodeStatus tickActive() {
        const std::array<double, 3> pos = actualPosition(node_);
        if (std::hypot(pos[0] - initial_pos_[0], pos[1] - initial_pos_[1]) >= max_dist_) {
            RCLCPP_INFO(logger_, "DownForwardAlign: max distance %.2fm reached.", max_dist_);
            return sweep_when_missing_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "DownForwardAlign: timeout reached after %dms.", timeout_msec_);
            return sweep_when_missing_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        }

        double forward = forward_step_;
        double left = 0.0;
        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection != nullptr) {
            if (sweep_when_missing_) {
                RCLCPP_INFO(logger_, "DownForwardSweepAlign: found %s.", filter_.describe().c_str());
                return BT::NodeStatus::SUCCESS;
            }
            last_processed_ = match.received_at;
            const double distance =
                validDistanceValue(match.detection->distance_m) ? match.detection->distance_m : default_distance_;
            // Down-camera bearings: positive vertical = ahead of center, positive
            // horizontal = right of center = negative FLU left offset.
            forward += center_gain_ * std::tan(match.detection->bearing_vertical) * distance;
            left = -center_gain_ * std::tan(match.detection->bearing_horizontal) * distance;
        } else if (sweep_when_missing_) {
            left = sweep_direction_ * lateral_sweep_step_;
            sweep_direction_ *= -1.0;
        }

        if (SteadyClock::now() - last_commanded_ >= std::chrono::milliseconds(update_msec_)) {
            addBodyOffsetToCommand(node_, forward, left, node_.commanded_att[2]);
            position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos, false));
            last_commanded_ = SteadyClock::now();
        }
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    std::array<double, 3> initial_pos_ = {};
    SteadyClock::time_point started_at_;
    SteadyClock::time_point deadline_;
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point last_commanded_;
    double max_dist_ = 10.0;
    double forward_step_ = 1.0;
    double default_distance_ = 1.0;
    double center_gain_ = 1.0;
    double lateral_sweep_step_ = 1.0;
    double sweep_direction_ = 1.0;
    int update_msec_ = 300;
    int timeout_msec_ = 30000;
    bool sweep_when_missing_ = false;
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
        getInput("orient_tolerance_deg", orient_tolerance_deg_);
        getInput("min_msec", min_msec_);
        getInput("timeout_msec", timeout_msec_);
        getInput("update_msec", update_msec_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_, "DownAlignToDetection needs a task port, e.g. <DownAlignToDetection task=\"bin\"/>.");
            return BT::NodeStatus::FAILURE;
        }
        if (tolerance_ <= 0.0 || default_distance_ <= 0.0 || update_msec_ <= 0) {
            RCLCPP_ERROR(logger_,
                         "DownAlignToDetection tolerance, default_distance, and update_msec must be positive.");
            return BT::NodeStatus::FAILURE;
        }

        last_processed_ = SteadyClock::now();
        last_commanded_ = last_processed_ - std::chrono::milliseconds(update_msec_);
        started_at_ = last_processed_;
        deadline_ = started_at_ + std::chrono::milliseconds(timeout_msec_);
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
        const auto orient_error = orientationYaw(detection);
        const bool hold_distance = std::isfinite(desired_distance_);
        const bool centered = std::fabs(detection.bearing_horizontal) <= tolerance_ &&
                              std::fabs(detection.bearing_vertical) <= tolerance_;
        const bool distance_ok = !hold_distance || std::fabs(distance - desired_distance_) <= distance_tolerance_;
        const bool orient_ok = !orient_ || !orient_error || std::fabs(*orient_error) <= radians(orient_tolerance_deg_);
        const bool old_enough = SteadyClock::now() - started_at_ >= std::chrono::milliseconds(min_msec_);
        if (old_enough && centered && distance_ok && orient_ok) {
            RCLCPP_INFO(logger_, "DownAlignToDetection: aligned to %s.", filter_.describe().c_str());
            return BT::NodeStatus::SUCCESS;
        }

        if (SteadyClock::now() - last_commanded_ < std::chrono::milliseconds(update_msec_)) {
            return BT::NodeStatus::RUNNING;
        }

        const double yaw = actualYaw(node_);
        // Down-camera bearings: positive vertical = ahead of center, positive
        // horizontal = right of center = negative FLU left offset.
        const double forward = center_gain_ * std::tan(detection.bearing_vertical) * distance;
        const double left = -center_gain_ * std::tan(detection.bearing_horizontal) * distance;
        addBodyOffsetToCommand(node_, forward, left, yaw);
        if (hold_distance) {
            // Farther above the object than desired -> descend (ENU z down is negative).
            const double actual_z = node_.commanded_pos[2] - node_.control_errors[2];
            node_.commanded_pos[2] = actual_z - (distance - desired_distance_);
        }
        position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos, false));

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
    int min_msec_ = 3000;
    int timeout_msec_ = 30000;
    int update_msec_ = 200;
    bool orient_ = true;
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

    // (forward, left) FLU multipliers of movement_dist; same physical loop as
    // the pre-ENU (forward, right) pattern.
    static constexpr std::array<std::array<double, 2>, 6> PATTERN = {
        std::array<double, 2>{0.0, 0.0}, std::array<double, 2>{-1.0, -1.0}, std::array<double, 2>{0.0, 2.0},
        std::array<double, 2>{2.0, 0.0}, std::array<double, 2>{0.0, -2.0},  std::array<double, 2>{-1.0, 1.0}};

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
        const double left = PATTERN[index][1] * movement_dist_;
        return {initial_pos_[0] + std::cos(initial_yaw_) * forward - std::sin(initial_yaw_) * left,
                initial_pos_[1] + std::sin(initial_yaw_) * forward + std::cos(initial_yaw_) * left,
                node_.commanded_pos[2]};
    }

    void beginMove(const std::array<double, 3> &target) {
        node_.commanded_pos[0] = target[0];
        node_.commanded_pos[1] = target[1];
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        phase_ = Phase::MOVE;
        position_publisher_->publish(positionCommand(*clock_, node_.commanded_pos, false));
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

    factory.registerBuilder<ForwardSweepAlignAction>(
        "ForwardSweepAlign", [&node, position_publisher, attitude_publisher, clock, logger](
                                 const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<ForwardSweepAlignAction>(name, config, node, position_publisher, attitude_publisher,
                                                             clock, logger);
        });

    factory.registerBuilder<ForwardAlignAction>(
        "ForwardAlign", [&node, position_publisher, attitude_publisher, clock, logger](const std::string &name,
                                                                                       const BT::NodeConfig &config) {
            return std::make_unique<ForwardAlignAction>(name, config, node, position_publisher, attitude_publisher,
                                                        clock, logger);
        });

    factory.registerBuilder<OrientToDetectionAtDistAction>(
        "OrientToDetectionAtDist", [&node, position_publisher, attitude_publisher, clock, logger](
                                       const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<OrientToDetectionAtDistAction>(name, config, node, position_publisher,
                                                                   attitude_publisher, clock, logger);
        });

    auto down_forward_align_builder = [&node, position_publisher, clock, logger](const std::string &name,
                                                                                 const BT::NodeConfig &config) {
        return std::make_unique<DownForwardAlignAction>(name, config, node, position_publisher, clock, logger);
    };
    factory.registerBuilder<DownForwardAlignAction>("DownForwardAlign", down_forward_align_builder);
    factory.registerBuilder<DownForwardAlignAction>("DownForwardSweepAlign", [&node, position_publisher, clock, logger](
                                                                                 const std::string &name,
                                                                                 const BT::NodeConfig &config) {
        return std::make_unique<DownForwardAlignAction>(name, config, node, position_publisher, clock, logger, true);
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
