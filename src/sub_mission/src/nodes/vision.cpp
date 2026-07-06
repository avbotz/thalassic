/** @file vision.cpp
 *  @brief Generic perception BT leaves connecting sub_mission to sub_vision.
 *
 * Four reusable primitives, filtered by (camera, task, class_id, min_score):
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
 * Task-specific behavior ("align with the gate") is XML composition of these
 * in resources/trees/, not new C++ -- see trees/gate.xml for the pattern.
 */

#include "sub_mission/nodes/vision.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

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
    match.detection =
        VisionClient::bestMatch(*snapshot.detections, filter.task, filter.class_id, filter.min_score);
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

        task_ = task;
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
                RCLCPP_INFO(logger_, "LoadModel: requested '%s' on camera '%s'.", task_.c_str(), camera_.c_str());
            } else if (SteadyClock::now() >= deadline_) {
                RCLCPP_ERROR(logger_, "LoadModel '%s': camera '%s' vision service unavailable.", task_.c_str(),
                             camera_.c_str());
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }

        if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            const auto response = future_.get();
            if (response->success) {
                RCLCPP_INFO(logger_, "LoadModel: '%s' active (%s, %.1fs).", task_.c_str(),
                            response->active_model.c_str(), response->load_time_s);
                return BT::NodeStatus::SUCCESS;
            }
            RCLCPP_ERROR(logger_, "LoadModel '%s' failed: %s", task_.c_str(), response->message.c_str());
            return BT::NodeStatus::FAILURE;
        }

        if (SteadyClock::now() >= deadline_) {
            RCLCPP_ERROR(logger_, "LoadModel '%s' timed out waiting for the load to finish.", task_.c_str());
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
            attitude_publisher_->publish(quaternionCommand(*clock_, node_.commanded_att[0], -node_.commanded_att[1],
                                                           -node_.commanded_att[2]));
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
}
