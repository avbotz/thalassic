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
 * one C++ BT node: it loads the model, sweeps yaw, samples detections, moves
 * forward between sweeps, and waits for control convergence internally.
 */

#include "sub_mission/nodes/vision.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <utility>
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
            const double actual_yaw = node_.commanded_att[2] - node_.control_errors[8];
            node_.commanded_att[2] = normalizeAngle(actual_yaw + bearing_h);
            attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
        }
        if (align_depth_ && !depth_aligned) {
            const double actual_z = node_.commanded_pos[2] - node_.control_errors[2];
            const double step = std::clamp(depth_gain_ * bearing_v, -max_depth_step_, max_depth_step_);
            node_.commanded_pos[2] = actual_z + step;
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
        ports.insert(BT::InputPort<int>("load_timeout_msec", 20000, "Maximum wait for load_model"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        filter_ = DetectionFilter::read(*this);
        getInput("attempts", attempts_);
        getInput("num_sweeps", num_sweeps_);
        getInput("forward_step", forward_step_);
        getInput("sample_timeout_msec", sample_timeout_msec_);
        getInput("move_timeout_msec", move_timeout_msec_);
        getInput("load_timeout_msec", load_timeout_msec_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_, "ForwardSweepAlign needs a task port, e.g. <ForwardSweepAlign task=\"gate\"/>.");
            return BT::NodeStatus::FAILURE;
        }
        if (attempts_ <= 0 || num_sweeps_ <= 0) {
            RCLCPP_ERROR(logger_, "ForwardSweepAlign attempts and num_sweeps must be positive.");
            return BT::NodeStatus::FAILURE;
        }

        client_ = node_.vision().loadModelClient(filter_.camera);
        if (!client_) {
            RCLCPP_ERROR(logger_, "ForwardSweepAlign: unknown camera '%s'.", filter_.camera.c_str());
            return BT::NodeStatus::FAILURE;
        }

        phase_ = Phase::LOAD_MODEL;
        sweep_index_ = 0;
        angle_index_ = 0;
        detection_count_ = 0;
        detection_sum_ = 0.0;
        last_processed_ = SteadyClock::now();
        deadline_ = last_processed_ + std::chrono::milliseconds(load_timeout_msec_);
        sent_load_request_ = false;
        return tickImpl();
    }

    BT::NodeStatus onRunning() override { return tickImpl(); }

    void onHalted() override {
        dropPendingRequest();
        RCLCPP_INFO(logger_, "ForwardSweepAlign halted.");
    }

   private:
    enum class Phase {
        LOAD_MODEL,
        FORWARD_MOVE,
        YAW_MOVE,
        SAMPLE_DETECTIONS,
        FINAL_ALIGN,
    };

    static double radians(const double degrees) { return degrees * M_PI / 180.0; }

    BT::NodeStatus tickImpl() {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "ForwardSweepAlign failed because kill switch is engaged.");
            dropPendingRequest();
            return BT::NodeStatus::FAILURE;
        }

        switch (phase_) {
            case Phase::LOAD_MODEL:
                return tickLoadModel();
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

    BT::NodeStatus tickLoadModel() {
        if (!sent_load_request_) {
            if (client_->service_is_ready()) {
                auto request = std::make_shared<LoadModel::Request>();
                request->task = node_.visionModelTask(filter_.task);
                auto future_and_id = client_->async_send_request(request);
                request_id_ = future_and_id.request_id;
                future_ = future_and_id.future.share();
                sent_load_request_ = true;
                RCLCPP_INFO(logger_, "ForwardSweepAlign: requested model '%s' for %s.",
                            request->task.c_str(), filter_.describe().c_str());
            } else if (SteadyClock::now() >= deadline_) {
                RCLCPP_ERROR(logger_, "ForwardSweepAlign model '%s' for %s: vision service unavailable.",
                             node_.visionModelTask(filter_.task).c_str(), filter_.describe().c_str());
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }

        if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            const auto response = future_.get();
            if (!response->success) {
                RCLCPP_ERROR(logger_, "ForwardSweepAlign load_model '%s' for %s failed: %s",
                             node_.visionModelTask(filter_.task).c_str(), filter_.describe().c_str(),
                             response->message.c_str());
                return BT::NodeStatus::FAILURE;
            }
            beginYawMove();
            return BT::NodeStatus::RUNNING;
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_ERROR(logger_, "ForwardSweepAlign model '%s' for %s timed out waiting for load_model.",
                         node_.visionModelTask(filter_.task).c_str(), filter_.describe().c_str());
            dropPendingRequest();
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
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
        RCLCPP_INFO(logger_, "ForwardSweepAlign: sweep %d/%d moving forward %.2fm.", sweep_index_ + 1,
                    num_sweeps_, forward_step_);
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
                const double actual_yaw = node_.commanded_att[2] - node_.control_errors[8];
                node_.commanded_att[2] = normalizeAngle(actual_yaw + average_bearing);
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

    void dropPendingRequest() {
        if (sent_load_request_ && client_ && future_.valid() &&
            future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            client_->remove_pending_request(request_id_);
        }
        sent_load_request_ = false;
    }

    static constexpr std::size_t SWEEP_COUNT = 6;

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    rclcpp::Client<LoadModel>::SharedPtr client_;
    std::shared_future<LoadModel::Response::SharedPtr> future_;
    std::int64_t request_id_ = 0;
    Phase phase_ = Phase::LOAD_MODEL;
    std::array<std::uint64_t, 12> start_updates_ = {};
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point deadline_;
    int attempts_ = 4;
    int num_sweeps_ = 3;
    int sweep_index_ = 0;
    std::size_t angle_index_ = 0;
    int move_timeout_msec_ = 20000;
    int load_timeout_msec_ = 20000;
    double forward_step_ = 2.0;
    double sample_timeout_msec_ = 2500.0;
    int detection_count_ = 0;
    double detection_sum_ = 0.0;
    bool sent_load_request_ = false;
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
        ports.insert(BT::InputPort<int>("load_timeout_msec", 20000, "Maximum wait for load_model"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        filter_ = DetectionFilter::read(*this);
        getInput("attempts", attempts_);
        getInput("sample_timeout_msec", sample_timeout_msec_);
        getInput("move_timeout_msec", move_timeout_msec_);
        getInput("load_timeout_msec", load_timeout_msec_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_, "SweepCheck needs a task port, e.g. <SweepCheck task=\"gate\"/>.");
            return BT::NodeStatus::FAILURE;
        }
        if (attempts_ <= 0) {
            RCLCPP_ERROR(logger_, "SweepCheck attempts must be positive.");
            return BT::NodeStatus::FAILURE;
        }
        client_ = node_.vision().loadModelClient(filter_.camera);
        if (!client_) {
            RCLCPP_ERROR(logger_, "SweepCheck: unknown camera '%s'.", filter_.camera.c_str());
            return BT::NodeStatus::FAILURE;
        }

        phase_ = Phase::LOAD_MODEL;
        angle_index_ = 0;
        detection_count_ = 0;
        detection_sum_ = 0.0;
        sent_load_request_ = false;
        last_processed_ = SteadyClock::now();
        deadline_ = last_processed_ + std::chrono::milliseconds(load_timeout_msec_);
        return tickImpl();
    }

    BT::NodeStatus onRunning() override { return tickImpl(); }

    void onHalted() override {
        dropPendingRequest();
        RCLCPP_INFO(logger_, "SweepCheck halted.");
    }

   private:
    enum class Phase {
        LOAD_MODEL,
        YAW_MOVE,
        SAMPLE_DETECTIONS,
        FINAL_ALIGN,
    };

    static double radians(const double degrees) { return degrees * M_PI / 180.0; }

    BT::NodeStatus tickImpl() {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "SweepCheck failed because kill switch is engaged.");
            dropPendingRequest();
            return BT::NodeStatus::FAILURE;
        }
        switch (phase_) {
            case Phase::LOAD_MODEL:
                return tickLoadModel();
            case Phase::YAW_MOVE:
                return tickYawMove();
            case Phase::SAMPLE_DETECTIONS:
                return tickSampleDetections();
            case Phase::FINAL_ALIGN:
                return tickFinalAlign();
        }
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus tickLoadModel() {
        if (!sent_load_request_) {
            if (client_->service_is_ready()) {
                auto request = std::make_shared<LoadModel::Request>();
                request->task = node_.visionModelTask(filter_.task);
                auto future_and_id = client_->async_send_request(request);
                request_id_ = future_and_id.request_id;
                future_ = future_and_id.future.share();
                sent_load_request_ = true;
                RCLCPP_INFO(logger_, "SweepCheck: requested model '%s' for %s.", request->task.c_str(),
                            filter_.describe().c_str());
            } else if (SteadyClock::now() >= deadline_) {
                RCLCPP_ERROR(logger_, "SweepCheck model '%s' for %s: vision service unavailable.",
                             node_.visionModelTask(filter_.task).c_str(), filter_.describe().c_str());
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }

        if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            const auto response = future_.get();
            if (!response->success) {
                RCLCPP_ERROR(logger_, "SweepCheck load_model '%s' for %s failed: %s",
                             node_.visionModelTask(filter_.task).c_str(), filter_.describe().c_str(),
                             response->message.c_str());
                return BT::NodeStatus::FAILURE;
            }
            beginYawMove();
            return BT::NodeStatus::RUNNING;
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_ERROR(logger_, "SweepCheck model '%s' for %s timed out waiting for load_model.",
                         node_.visionModelTask(filter_.task).c_str(), filter_.describe().c_str());
            dropPendingRequest();
            return BT::NodeStatus::FAILURE;
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
                node_.commanded_att[2] = normalizeAngle(actual_yaw + average_bearing);
                start_updates_ = node_.control_error_updates;
                deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
                phase_ = Phase::FINAL_ALIGN;
                attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));
                RCLCPP_INFO(logger_, "SweepCheck: found %s, average bearing %.3f rad.",
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

    void dropPendingRequest() {
        if (sent_load_request_ && client_ && future_.valid() &&
            future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            client_->remove_pending_request(request_id_);
        }
        sent_load_request_ = false;
    }

    static constexpr std::size_t SWEEP_COUNT = 6;

    MissionNode &node_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    rclcpp::Client<LoadModel>::SharedPtr client_;
    std::shared_future<LoadModel::Response::SharedPtr> future_;
    std::int64_t request_id_ = 0;
    Phase phase_ = Phase::LOAD_MODEL;
    std::array<std::uint64_t, 12> start_updates_ = {};
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point deadline_;
    int attempts_ = 4;
    std::size_t angle_index_ = 0;
    int move_timeout_msec_ = 20000;
    int load_timeout_msec_ = 20000;
    double sample_timeout_msec_ = 2500.0;
    int detection_count_ = 0;
    double detection_sum_ = 0.0;
    bool sent_load_request_ = false;
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
        ports.insert(BT::InputPort<double>("close_distance", 3.0, "Distance that triggers final pass-through target"));
        ports.insert(BT::InputPort<double>("through_distance", 1.5, "Extra travel after the detected object"));
        ports.insert(BT::InputPort<double>("depth_offset", 0.25, "Extra depth added when tracking a gate-like target"));
        ports.insert(BT::InputPort<bool>("align_depth", true, "Adjust depth from vertical bearing and distance"));
        ports.insert(BT::InputPort<int>("update_msec", 300, "Minimum time between forward target updates"));
        ports.insert(BT::InputPort<int>("timeout_msec", 30000, "Maximum align time before SUCCESS"));
        ports.insert(BT::InputPort<int>("load_timeout_msec", 20000, "Maximum wait for load_model"));
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
        getInput("load_timeout_msec", load_timeout_msec_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_, "ForwardAlign needs a task port, e.g. <ForwardAlign task=\"gate\"/>.");
            return BT::NodeStatus::FAILURE;
        }
        if (max_dist_ <= 0.0 || forward_step_ <= 0.0 || update_msec_ <= 0) {
            RCLCPP_ERROR(logger_, "ForwardAlign max_dist, forward_step, and update_msec must be positive.");
            return BT::NodeStatus::FAILURE;
        }

        client_ = node_.vision().loadModelClient(filter_.camera);
        if (!client_) {
            RCLCPP_ERROR(logger_, "ForwardAlign: unknown camera '%s'.", filter_.camera.c_str());
            return BT::NodeStatus::FAILURE;
        }

        initial_pos_ = actualPosition();
        final_move_ = false;
        final_sent_ = false;
        sent_load_request_ = false;
        last_processed_ = SteadyClock::now();
        last_commanded_ = last_processed_ - std::chrono::milliseconds(update_msec_);
        started_at_ = last_processed_;
        deadline_ = started_at_ + std::chrono::milliseconds(load_timeout_msec_);
        return tickLoadModel();
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "ForwardAlign failed because kill switch is engaged.");
            dropPendingRequest();
            return BT::NodeStatus::FAILURE;
        }
        if (!sent_load_request_ || future_.valid()) {
            return tickLoadModel();
        }
        if (final_move_) {
            return tickFinalMove();
        }
        return tickActive();
    }

    void onHalted() override {
        dropPendingRequest();
        RCLCPP_INFO(logger_, "ForwardAlign halted.");
    }

   private:
    BT::NodeStatus tickLoadModel() {
        if (!sent_load_request_) {
            if (client_->service_is_ready()) {
                auto request = std::make_shared<LoadModel::Request>();
                request->task = node_.visionModelTask(filter_.task);
                auto future_and_id = client_->async_send_request(request);
                request_id_ = future_and_id.request_id;
                future_ = future_and_id.future.share();
                sent_load_request_ = true;
                RCLCPP_INFO(logger_, "ForwardAlign: requested model '%s' for %s.", request->task.c_str(),
                            filter_.describe().c_str());
            } else if (SteadyClock::now() >= deadline_) {
                RCLCPP_ERROR(logger_, "ForwardAlign model '%s' for %s: vision service unavailable.",
                             node_.visionModelTask(filter_.task).c_str(), filter_.describe().c_str());
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }

        if (future_.valid() && future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            const auto response = future_.get();
            if (!response->success) {
                RCLCPP_ERROR(logger_, "ForwardAlign load_model '%s' for %s failed: %s",
                             node_.visionModelTask(filter_.task).c_str(), filter_.describe().c_str(),
                             response->message.c_str());
                return BT::NodeStatus::FAILURE;
            }
            started_at_ = SteadyClock::now();
            deadline_ = started_at_ + std::chrono::milliseconds(timeout_msec_);
            RCLCPP_INFO(logger_, "ForwardAlign: model ready, moving while aligned to %s.", filter_.describe().c_str());
            return BT::NodeStatus::RUNNING;
        }

        if (SteadyClock::now() >= deadline_) {
            RCLCPP_ERROR(logger_, "ForwardAlign model '%s' for %s timed out waiting for load_model.",
                         node_.visionModelTask(filter_.task).c_str(), filter_.describe().c_str());
            dropPendingRequest();
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

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
                beginFinalMove(match.detection->distance_m + through_distance_);
                return BT::NodeStatus::RUNNING;
            }
        }

        if (SteadyClock::now() - last_commanded_ >= std::chrono::milliseconds(update_msec_)) {
            commandForward(forward_step_);
        }
        return BT::NodeStatus::RUNNING;
    }

    void steerFromDetection(const Detection &detection) {
        const double actual_yaw = node_.commanded_att[2] - node_.control_errors[8];
        node_.commanded_att[2] = normalizeAngle(actual_yaw + detection.bearing_horizontal);
        attitude_publisher_->publish(attitudeCommand(*clock_, node_.commanded_att));

        if (align_depth_ && validDistance(detection.distance_m)) {
            const double actual_z = node_.commanded_pos[2] - node_.control_errors[2];
            const double z_offset = std::sin(detection.bearing_vertical) * detection.distance_m;
            node_.commanded_pos[2] = actual_z + z_offset + depth_offset_;
        }
    }

    void beginFinalMove(const double distance) {
        commandForward(distance);
        final_move_ = true;
        final_sent_ = true;
        start_updates_ = node_.control_error_updates;
        final_deadline_ = SteadyClock::now() + std::chrono::milliseconds(timeout_msec_);
        RCLCPP_INFO(logger_, "ForwardAlign: close target detected, final pass-through %.2fm.", distance);
    }

    BT::NodeStatus tickFinalMove() {
        if (!final_sent_) {
            return BT::NodeStatus::FAILURE;
        }
        if (freshAndWithinTolerance(0, POSITION_TOLERANCE) && freshAndWithinTolerance(1, POSITION_TOLERANCE)) {
            RCLCPP_INFO(logger_, "ForwardAlign: final pass-through reached.");
            return BT::NodeStatus::SUCCESS;
        }
        if (SteadyClock::now() >= final_deadline_) {
            RCLCPP_WARN(logger_, "ForwardAlign: final pass-through timed out.");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
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

    void dropPendingRequest() {
        if (sent_load_request_ && client_ && future_.valid() &&
            future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            client_->remove_pending_request(request_id_);
        }
        sent_load_request_ = false;
    }

    MissionNode &node_;
    PointCmdPublisher::SharedPtr position_publisher_;
    QuaternionCmdPublisher::SharedPtr attitude_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    rclcpp::Client<LoadModel>::SharedPtr client_;
    std::shared_future<LoadModel::Response::SharedPtr> future_;
    std::int64_t request_id_ = 0;
    std::array<double, 3> initial_pos_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
    SteadyClock::time_point started_at_;
    SteadyClock::time_point deadline_;
    SteadyClock::time_point final_deadline_;
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point last_commanded_;
    double max_dist_ = 10.0;
    double forward_step_ = 2.0;
    double close_distance_ = 3.0;
    double through_distance_ = 1.5;
    double depth_offset_ = 0.25;
    bool align_depth_ = true;
    int update_msec_ = 300;
    int timeout_msec_ = 30000;
    int load_timeout_msec_ = 20000;
    bool sent_load_request_ = false;
    bool final_move_ = false;
    bool final_sent_ = false;
};

}  // namespace

void registerVisionNodes(BT::BehaviorTreeFactory &factory, MissionNode &node, const rclcpp::Logger logger,
                         PointCmdPublisher::SharedPtr position_publisher,
                         QuaternionCmdPublisher::SharedPtr attitude_publisher, rclcpp::Clock::SharedPtr clock) {
    factory.registerBuilder<LoadModelAction>(
        "LoadModel", [&node, logger](const std::string &name, const BT::NodeConfig &config) {
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
        "AlignToDetection", [&node, position_publisher, attitude_publisher, clock,
                             logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<AlignToDetectionAction>(name, config, node, position_publisher,
                                                            attitude_publisher, clock, logger);
        });

    factory.registerBuilder<ForwardSweepAlignAction>(
        "ForwardSweepAlign", [&node, position_publisher, attitude_publisher, clock,
                              logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<ForwardSweepAlignAction>(name, config, node, position_publisher,
                                                             attitude_publisher, clock, logger);
        });

    factory.registerBuilder<ForwardAlignAction>(
        "ForwardAlign", [&node, position_publisher, attitude_publisher, clock,
                         logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<ForwardAlignAction>(name, config, node, position_publisher, attitude_publisher,
                                                        clock, logger);
        });

    factory.registerBuilder<SweepCheckAction>(
        "SweepCheck", [&node, attitude_publisher, clock,
                       logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<SweepCheckAction>(name, config, node, attitude_publisher, clock, logger);
        });
}
