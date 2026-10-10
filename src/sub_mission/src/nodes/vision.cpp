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
 *   SearchForDetection -- move at a fixed velocity until a matching detection
 *                        arrives, then stop.
 *   AlignToDetection  -- closed-loop centering: steer yaw and/or depth until
 *                        the detection's bearing is within tolerance.
 *
 * The rest are task mechanics built on them: sweeps that search in yaw,
 * approaches that move while steering, lateral and down-camera centering.
 */

#include "sub_mission/nodes/vision.hpp"

#include "actions/actions.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
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

    // Reads the ports into `filter`. False, with the reason logged, when one's
    // blackboard entry is missing or does not parse: an empty task or class_id
    // would quietly accept anything.
    static bool read(const BT::TreeNode &node, DetectionFilter &filter, const rclcpp::Logger &logger) {
        filter = DetectionFilter{};
        return readPort(node, "camera", filter.camera, logger) && readPort(node, "task", filter.task, logger) &&
               readPort(node, "class_id", filter.class_id, logger) &&
               readPort(node, "min_score", filter.min_score, logger);
    }

    std::string describe() const {
        return "camera=" + camera + " task=" + (task.empty() ? "*" : task) +
               " class=" + (class_id.empty() ? "*" : class_id);
    }

    // False, with the reason logged, for a camera the node cannot use: one
    // VisionClient does not know, where nothing would ever match and a search
    // would never end; or, with `front_only`, the down camera, for the nodes
    // that read bearings as a turn or as a sideways and vertical offset. The
    // down camera's image axes are the vehicle's lateral and fore-aft ones.
    bool usableCamera(MissionNode &mission, const BT::TreeNode &node, const rclcpp::Logger &logger,
                      const bool front_only = false) const {
        if (!mission.vision().hasCamera(camera)) {
            RCLCPP_ERROR(logger, "%s: unknown camera '%s'; expected front or down.", node.name().c_str(),
                         camera.c_str());
            return false;
        }
        if (front_only && camera != "front") {
            RCLCPP_ERROR(logger, "%s steers on front-camera bearings; it cannot use camera '%s'.", node.name().c_str(),
                         camera.c_str());
            return false;
        }
        return true;
    }
};

// Latest detection matching the filter, provided the snapshot was received
// after `not_before`. Returns the owning array so the Detection* stays valid.
struct Match {
    sub_vision_interfaces::msg::DetectionArray::ConstSharedPtr array;
    const Detection *detection = nullptr;
    SteadyClock::time_point received_at{};
};

// `accept`, when given, further restricts which detections count. One without
// finite bearings never does: every leaf steers or reports on them.
Match latestMatch(MissionNode &node, const DetectionFilter &filter, const SteadyClock::time_point not_before,
                  const std::function<bool(const Detection &)> &accept = {}) {
    Match match;
    const VisionClient::Snapshot snapshot = node.vision().latest(filter.camera);
    if (!snapshot.detections || snapshot.received_at < not_before) {
        return match;
    }
    if (!node.visionTaskMatches(snapshot.detections->task, filter.task)) {
        return match;
    }
    const auto usable = [&accept](const Detection &detection) {
        return std::isfinite(detection.bearing_horizontal) && std::isfinite(detection.bearing_vertical) &&
               (!accept || accept(detection));
    };
    match.detection = VisionClient::bestMatch(*snapshot.detections, "", filter.class_id, filter.min_score, usable);
    if (match.detection != nullptr) {
        match.array = snapshot.detections;
        match.received_at = snapshot.received_at;
    }
    return match;
}

bool validDistanceValue(const double distance) { return std::isfinite(distance) && distance > 0.0; }

double radians(const double degrees) { return degrees * M_PI / 180.0; }

// Empty unless the value is all one finite number: a post-processor's "nan"
// is no measurement.
std::optional<double> extraDouble(const Detection &detection, const std::string &key) {
    for (const auto &entry : detection.extra) {
        if (entry.key != key) {
            continue;
        }
        try {
            std::size_t parsed = 0;
            const double value = std::stod(entry.value, &parsed);
            if (parsed != entry.value.size() || !std::isfinite(value)) {
                return std::nullopt;
            }
            return value;
        } catch (const std::exception &) {
            return std::nullopt;
        }
    }
    return std::nullopt;
}

std::optional<std::string> extraString(const Detection &detection, const std::string &key) {
    for (const auto &entry : detection.extra) {
        if (entry.key == key) {
            return entry.value;
        }
    }
    return std::nullopt;
}

// Turns, relative to the heading, from a post-processor's yaw metadata. Both
// are ROS/FLU, so counter-clockwise-positive like every yaw here. Detection.pose
// is not used: it is in the camera's optical frame, never a vehicle yaw.

// "yaw_deg": the turn that lines the vehicle up with the object (path markers).
std::optional<double> orientationYaw(const Detection &detection) {
    if (const auto yaw_deg = extraDouble(detection, "yaw_deg")) {
        return radians(*yaw_deg);
    }
    return std::nullopt;
}

// "heading_yaw_deg": the turn that squares the camera to the object's face
// (the torpedo board). Kept apart from orientationYaw, which steering toward
// an object reads: the board's face is not the way to it.
std::optional<double> headingYaw(const Detection &detection) {
    if (const auto yaw_deg = extraDouble(detection, "heading_yaw_deg")) {
        return radians(*yaw_deg);
    }
    return std::nullopt;
}

// The measured pose, from odometry. Targets built from it stay convergent when
// re-issued on every frame, instead of integrating the correction.
std::array<double, 3> actualPosition(const MissionNode &node) { return node.measured_pos; }

double actualYaw(const MissionNode &node) { return node.measured_att[2]; }

// sub_vision's bearings are camera-optical: positive is right of / below the
// image center. Image right is the vehicle's right on both cameras; image down
// is down on the front camera and backward on the down camera (image up is
// forward). The two helpers below turn them into the mission's ENU/FLU.

// The yaw that faces a front-camera bearing: a target on the right is a turn
// clockwise, to a smaller yaw.
double yawToward(const MissionNode &node, const double bearing_horizontal) {
    return actualYaw(node) - bearing_horizontal;
}

// The body (forward, left) offset that puts the vehicle over a down-camera
// detection `distance` below the camera.
std::array<double, 2> downOffset(const Detection &detection, const double distance) {
    return {-std::tan(detection.bearing_vertical) * distance, -std::tan(detection.bearing_horizontal) * distance};
}

// The yaw steps of a sweep, each from the one before [deg]: ahead, 40 and 80
// right, 80 and 40 left, then ahead again. Right is clockwise, so negative.
constexpr std::array<double, 6> SWEEP_DEGREES = {0.0, -40.0, -40.0, 160.0, -40.0, -40.0};

// A position target at the vehicle's horizontal position, leaving z alone.
std::array<double, 3> horizontalTarget(const MissionNode &node) {
    return {node.measured_pos[0], node.measured_pos[1], UNSPECIFIED_PORT};
}

void addBodyOffset(std::array<double, 3> &target, const double forward, const double left, const double yaw) {
    target[0] += std::cos(yaw) * forward - std::sin(yaw) * left;
    target[1] += std::sin(yaw) * forward + std::cos(yaw) * left;
}

// The highest z vision may steer to, 0.5 m deep. A breach outside the octagon
// ends a RoboSub run, so a bad range or bearing must not drive the sub up
// through the surface; surfacing is always an explicit PosSetpoint in the tree.
constexpr double MAX_VISION_Z = -0.5;

// Sends a position target, setting only the axes it gives, z no higher than
// MAX_VISION_Z.
void sendPosition(MissionNode &node, SetpointPublisher &publisher, const rclcpp::Clock &clock,
                  std::array<double, 3> target) {
    if (std::isfinite(target[2])) {
        target[2] = std::min(target[2], MAX_VISION_Z);
    }
    node.commandPosition(target, false);
    publisher.publish(positionCommand(clock, target, false));
}

void sendYaw(MissionNode &node, SetpointPublisher &publisher, const rclcpp::Clock &clock, const double yaw) {
    const std::array<double, 3> target{UNSPECIFIED_PORT, UNSPECIFIED_PORT, normalizeAngle(yaw)};
    node.commandAttitude(target);
    publisher.publish(attitudeCommand(clock, target));
}

// Moves at `velocity` along the heading (forward, left, up) until replaced;
// axes given as NaN carry on as they were, but x and y go together
// (linearVelocityCommand).
void sendVelocity(MissionNode &node, SetpointPublisher &publisher, const rclcpp::Clock &clock,
                  const std::array<double, 3> &velocity) {
    // What WaitUntilHit measures speed against, for the axes this sets (x
    // and y go together, a NaN one at zero); an axis left alone keeps its own.
    const bool horizontal = std::isfinite(velocity[0]) || std::isfinite(velocity[1]);
    for (std::size_t i = 0; i < velocity.size(); ++i) {
        if (std::isfinite(velocity[i]) || (i < 2 && horizontal)) {
            node.last_velocity_setpoint[i] = std::isfinite(velocity[i]) ? velocity[i] : 0.0;
        }
    }
    node.releasePosition(velocity);
    publisher.publish(linearVelocityCommand(clock, velocity));
}

// HOLD on x and y, ending a velocity along the heading.
void stopHorizontal(MissionNode &node) { node.hold({true, true, false, false, false, false}); }

// Of a non-empty `values`.
double median(std::vector<double> values) {
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    if (values.size() % 2 != 0) {
        return values[middle];
    }
    const double upper = values[middle];
    return 0.5 * (*std::max_element(values.begin(), values.begin() + middle) + upper);
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
        if (timeout_msec <= 0) {
            RCLCPP_ERROR(logger_, "LoadModel timeout_msec must be positive.");
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
    // No kill-switch guard: loading a model moves nothing.
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
        DetectionFilter filter;
        if (!DetectionFilter::read(*this, filter, node_.get_logger())) {
            return BT::NodeStatus::FAILURE;
        }
        const auto not_before = SteadyClock::now() - std::chrono::milliseconds(max_age_msec);
        return latestMatch(node_, filter, not_before).detection != nullptr ? BT::NodeStatus::SUCCESS
                                                                           : BT::NodeStatus::FAILURE;
    }

   private:
    MissionNode &node_;
};

// A timeout that starts now, or with the camera's first frame from now on: a
// model that has just been loaded can take seconds to produce one.
struct FrameTimeout {
    SteadyClock::time_point started_at;
    SteadyClock::time_point deadline;
    std::chrono::milliseconds timeout{0};
    bool armed = false;

    void start(const int timeout_msec, const bool after_first_frame) {
        started_at = SteadyClock::now();
        timeout = std::chrono::milliseconds(timeout_msec);
        armed = !after_first_frame;
        deadline = started_at + timeout;
    }

    bool expired(MissionNode &node, const std::string &camera) {
        if (!armed) {
            const VisionClient::Snapshot snapshot = node.vision().latest(camera);
            if (!snapshot.detections || snapshot.received_at < started_at) {
                return false;
            }
            armed = true;
            deadline = snapshot.received_at + timeout;
        }
        return SteadyClock::now() >= deadline;
    }
};

class WaitForDetectionAction : public BT::StatefulActionNode {
   public:
    WaitForDetectionAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                           rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config), node_(node), logger_(logger) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = DetectionFilter::ports();
        ports.insert(BT::InputPort<int>("timeout_msec", 15000, "Maximum wait before returning FAILURE"));
        ports.insert(BT::InputPort<bool>("start_timeout_after_first_frame", false,
                                         "Start the timeout with the camera's first frame instead of now"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        int timeout_msec = 15000;
        bool after_first_frame = false;
        getInput("timeout_msec", timeout_msec);
        getInput("start_timeout_after_first_frame", after_first_frame);
        if (!DetectionFilter::read(*this, filter_, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        if (!filter_.usableCamera(node_, *this, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        timeout_.start(timeout_msec, after_first_frame);
        return check();
    }

    BT::NodeStatus onRunning() override { return check(); }

    void onHalted() override { RCLCPP_INFO(logger_, "WaitForDetection halted."); }

   private:
    // No kill-switch guard: passive, bounded by timeout_msec.
    BT::NodeStatus check() {
        // Only frames that arrived after this node started count: a stale
        // detection from a previous task must not satisfy the wait.
        const Match match = latestMatch(node_, filter_, timeout_.started_at);
        if (match.detection != nullptr) {
            const auto &hypothesis = match.detection->detection.results[0].hypothesis;
            RCLCPP_INFO(logger_, "WaitForDetection: %s -> class=%s score=%.2f bearing=(%.3f, %.3f).",
                        filter_.describe().c_str(), hypothesis.class_id.c_str(), hypothesis.score,
                        match.detection->bearing_horizontal, match.detection->bearing_vertical);
            return BT::NodeStatus::SUCCESS;
        }

        if (timeout_.expired(node_, filter_.camera)) {
            RCLCPP_WARN(logger_, "WaitForDetection timed out: %s.", filter_.describe().c_str());
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    FrameTimeout timeout_;
};

// Reads the role off the gate: the post-processor's left-opening target
// carries the role of the images on that side (gate_role), and the vehicle
// passes through that opening. Sets MissionNode::role once enough frames in a
// row agree, for SurveyRole/SearchRole further on.
class SelectGateRoleAction : public BT::StatefulActionNode {
   public:
    SelectGateRoleAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                         rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config), node_(node), logger_(logger) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = DetectionFilter::ports();
        ports.insert(BT::InputPort<int>("confirmation_frames", 3, "Frames in a row that must show the same role"));
        ports.insert(BT::InputPort<int>("timeout_msec", 20000, "Maximum wait before FAILURE"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        int timeout_msec = 20000;
        if (!DetectionFilter::read(*this, filter_, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        getInput("confirmation_frames", confirmation_frames_);
        getInput("timeout_msec", timeout_msec);
        if (filter_.task.empty() || filter_.class_id.empty() || confirmation_frames_ <= 0 || timeout_msec <= 0) {
            RCLCPP_ERROR(logger_,
                         "SelectGateRole needs a task, a class_id and positive confirmation_frames and "
                         "timeout_msec.");
            return BT::NodeStatus::FAILURE;
        }
        if (!filter_.usableCamera(node_, *this, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        last_processed_ = SteadyClock::now();
        deadline_ = last_processed_ + std::chrono::milliseconds(timeout_msec);
        candidate_.clear();
        candidate_frames_ = 0;
        return check();
    }

    BT::NodeStatus onRunning() override { return check(); }

    void onHalted() override { RCLCPP_INFO(logger_, "SelectGateRole halted."); }

   private:
    BT::NodeStatus check() {
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "SelectGateRole timed out: %s.", filter_.describe().c_str());
            return BT::NodeStatus::FAILURE;
        }
        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection == nullptr) {
            return BT::NodeStatus::RUNNING;
        }
        last_processed_ = match.received_at;

        const auto role = extraString(*match.detection, "gate_role");
        if (!role || (*role != "SURVEY" && *role != "SEARCH")) {
            candidate_.clear();
            candidate_frames_ = 0;
            return BT::NodeStatus::RUNNING;
        }
        candidate_frames_ = *role == candidate_ ? candidate_frames_ + 1 : 1;
        candidate_ = *role;
        if (candidate_frames_ < confirmation_frames_) {
            return BT::NodeStatus::RUNNING;
        }

        node_.role = candidate_;
        RCLCPP_INFO(logger_, "SelectGateRole: %s, from the left opening's images [%s].",
                    extraString(*match.detection, "gate_role_label").value_or(candidate_).c_str(),
                    extraString(*match.detection, "left_icon_names").value_or("unknown").c_str());
        return BT::NodeStatus::SUCCESS;
    }

    MissionNode &node_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point deadline_;
    std::string candidate_;
    int confirmation_frames_ = 3;
    int candidate_frames_ = 0;
};

// Moves at a fixed velocity along the heading, holding the heading it started
// on, until a matching detection arrives; then stops. With sweep_left_m and
// sweep_right_m it is a lateral search instead: left that far, then back right
// that far from where it turned.
class SearchForDetectionAction : public BT::StatefulActionNode {
   public:
    SearchForDetectionAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                             SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                             rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = DetectionFilter::ports();
        // Unset axes keep doing what they were (e.g. holding depth).
        ports.insert(BT::InputPort<double>("x", UNSPECIFIED_PORT, "Forward velocity in meters per second"));
        ports.insert(BT::InputPort<double>("y", UNSPECIFIED_PORT, "Left velocity in meters per second"));
        ports.insert(BT::InputPort<double>("z", UNSPECIFIED_PORT, "Up velocity in meters per second"));
        ports.insert(BT::InputPort<double>("sweep_left_m", 0.0, "Lateral search: distance left before turning back"));
        ports.insert(BT::InputPort<double>("sweep_right_m", 0.0, "Lateral search: distance right after turning back"));
        ports.insert(BT::InputPort<int>("timeout_msec", 30000, "Maximum search time before FAILURE; 0 waits forever"));
        ports.insert(BT::InputPort<bool>("start_timeout_after_first_frame", false,
                                         "Start the timeout with the camera's first frame instead of now"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        std::array<double, 3> velocity{};
        int timeout_msec = 30000;
        bool after_first_frame = false;
        if (!DetectionFilter::read(*this, filter_, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        if (!readPort(*this, "x", velocity[0], logger_) || !readPort(*this, "y", velocity[1], logger_) ||
            !readPort(*this, "z", velocity[2], logger_) || !readPort(*this, "sweep_left_m", sweep_left_m_, logger_) ||
            !readPort(*this, "sweep_right_m", sweep_right_m_, logger_) ||
            !readPort(*this, "timeout_msec", timeout_msec, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        getInput("start_timeout_after_first_frame", after_first_frame);

        sweep_ = sweep_left_m_ > 0.0 || sweep_right_m_ > 0.0;
        const bool lateral_only = std::isfinite(velocity[1]) && velocity[1] != 0.0 &&
                                  !(std::isfinite(velocity[0]) && velocity[0] != 0.0) && !std::isfinite(velocity[2]);
        double speed = 0.0;
        for (const double v : velocity) {
            speed = std::isfinite(v) ? std::hypot(speed, v) : speed;
        }
        if (speed <= 0.0 || timeout_msec < 0 || sweep_left_m_ < 0.0 || sweep_right_m_ < 0.0 ||
            (sweep_ && (sweep_left_m_ <= 0.0 || sweep_right_m_ <= 0.0 || !lateral_only))) {
            RCLCPP_ERROR(logger_,
                         "SearchForDetection needs a velocity and a non-negative timeout_msec; a lateral search "
                         "needs only a y velocity and both sweep distances.");
            return BT::NodeStatus::FAILURE;
        }
        if (!filter_.usableCamera(node_, *this, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        if (!node_.subAlive()) {
            return BT::NodeStatus::FAILURE;
        }

        timeout_msec_ = timeout_msec;
        timeout_.start(timeout_msec, after_first_frame);
        // Hold the heading the search starts on, wherever the tree left it.
        entry_yaw_ = actualYaw(node_);
        sendYaw(node_, *setpoint_publisher_, *clock_, entry_yaw_);
        moving_z_ = std::isfinite(velocity[2]);
        if (sweep_) {
            turn_point_ = {node_.measured_pos[0], node_.measured_pos[1]};
            returning_ = false;
            velocity = {UNSPECIFIED_PORT, std::fabs(velocity[1]), UNSPECIFIED_PORT};
        }
        lateral_speed_ = std::fabs(velocity[1]);
        sendVelocity(node_, *setpoint_publisher_, *clock_, velocity);
        RCLCPP_INFO(logger_, "SearchForDetection: xyz=(%.3f, %.3f, %.3f) holding yaw %.3f for %s.", velocity[0],
                    velocity[1], velocity[2], entry_yaw_, filter_.describe().c_str());
        return check();
    }

    BT::NodeStatus onRunning() override { return check(); }

    void onHalted() override { stop(); }

   private:
    BT::NodeStatus check() {
        if (!node_.subAlive()) {
            RCLCPP_WARN(logger_, "SearchForDetection failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }

        const Match match = latestMatch(node_, filter_, timeout_.started_at);
        if (match.detection != nullptr) {
            RCLCPP_INFO(logger_, "SearchForDetection: found %s, bearing=(%.3f, %.3f); stopping.",
                        filter_.describe().c_str(), match.detection->bearing_horizontal,
                        match.detection->bearing_vertical);
            stop();
            return BT::NodeStatus::SUCCESS;
        }

        if (sweep_) {
            const double left = leftOf(turn_point_);
            if (!returning_ && left >= sweep_left_m_) {
                turn_point_ = {node_.measured_pos[0], node_.measured_pos[1]};
                returning_ = true;
                sendVelocity(node_, *setpoint_publisher_, *clock_,
                             {UNSPECIFIED_PORT, -lateral_speed_, UNSPECIFIED_PORT});
                RCLCPP_INFO(logger_, "SearchForDetection: %.2fm left; turning back right.", left);
            } else if (returning_ && -left >= sweep_right_m_) {
                RCLCPP_WARN(logger_, "SearchForDetection: swept %.2fm left and %.2fm right without finding %s.",
                            sweep_left_m_, -left, filter_.describe().c_str());
                stop();
                return BT::NodeStatus::FAILURE;
            }
        }

        if (timeout_msec_ > 0 && timeout_.expired(node_, filter_.camera)) {
            RCLCPP_WARN(logger_, "SearchForDetection timed out without finding %s.", filter_.describe().c_str());
            stop();
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    // How far the vehicle is left of `point`, across the heading the search holds.
    double leftOf(const std::array<double, 2> &point) const {
        return -std::sin(entry_yaw_) * (node_.measured_pos[0] - point[0]) +
               std::cos(entry_yaw_) * (node_.measured_pos[1] - point[1]);
    }

    void stop() { node_.hold({true, true, moving_z_, false, false, false}); }

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    FrameTimeout timeout_;
    std::array<double, 2> turn_point_{};
    double sweep_left_m_ = 0.0;
    double sweep_right_m_ = 0.0;
    double lateral_speed_ = 0.0;
    double entry_yaw_ = 0.0;
    int timeout_msec_ = 30000;
    bool sweep_ = false;
    bool returning_ = false;
    bool moving_z_ = false;
};

class AlignToDetectionAction : public BT::StatefulActionNode {
   public:
    AlignToDetectionAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                           SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                           rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
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
        if (!DetectionFilter::read(*this, filter_, logger_)) {
            return BT::NodeStatus::FAILURE;
        }

        if (!align_yaw_ && !align_depth_) {
            RCLCPP_ERROR(logger_, "AlignToDetection has both yaw and depth disabled; nothing to do.");
            return BT::NodeStatus::FAILURE;
        }
        // A negative gain would steer away from the target; a negative step,
        // an empty clamp.
        if (tolerance_ <= 0.0 || depth_gain_ <= 0.0 || max_depth_step_ <= 0.0 || timeout_msec <= 0) {
            RCLCPP_ERROR(logger_,
                         "AlignToDetection needs a positive tolerance, depth_gain, max_depth_step and timeout_msec.");
            return BT::NodeStatus::FAILURE;
        }
        if (!filter_.usableCamera(node_, *this, logger_, true)) {
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

        // Absolute targets referenced to the *measured* state, so re-issuing on
        // every frame stays convergent instead of integrating the correction.
        if (align_yaw_ && !yaw_aligned) {
            sendYaw(node_, *setpoint_publisher_, *clock_, yawToward(node_, bearing_h));
        }
        if (align_depth_ && !depth_aligned) {
            // Below center (positive) is deeper, a smaller z.
            const double step = std::clamp(depth_gain_ * bearing_v, -max_depth_step_, max_depth_step_);
            sendPosition(node_, *setpoint_publisher_, *clock_,
                         {UNSPECIFIED_PORT, UNSPECIFIED_PORT, actualPosition(node_)[2] - step});
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { RCLCPP_INFO(logger_, "AlignToDetection halted."); }

   private:
    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
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

// Centers a front-camera detection by moving sideways (and in z), keeping the
// heading: e.g. onto the gate's left opening, or onto a slalom pole's ray.
// Bearings become offsets at a projection distance, target_distance or the
// detection's range. One of three ways to get there:
//   lock_target                  -- lock_frames consistent rays fix one odom
//                                   target, held until the vehicle is on it;
//   use_lateral_velocity_alignment -- move sideways at a fixed speed to the
//                                   last ray's point, even once detections stop;
//   neither                      -- a bounded position step every update_msec.
class LateralAlignToDetectionAction : public BT::StatefulActionNode {
   public:
    LateralAlignToDetectionAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                                  SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                  rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = DetectionFilter::ports();
        ports.insert(BT::InputPort<double>("target_distance", 1.15, "Range that turns bearings into offsets (m)"));
        ports.insert(BT::InputPort<bool>("use_detection_distance", false,
                                         "Use each detection's range instead of target_distance"));
        ports.insert(BT::InputPort<double>("max_projection_distance", 0.0,
                                           "Cap on the range used for corrections (m); 0 for none"));
        ports.insert(BT::InputPort<double>("target_offset_left", 0.0,
                                           "Aim this far left of the detection, at target_distance (m)"));
        ports.insert(
            BT::InputPort<double>("target_offset_up", 0.0, "Aim this far above the detection, at target_distance (m)"));
        ports.insert(BT::InputPort<bool>("align_depth", true, "Also center the vertical bearing, in z"));
        ports.insert(BT::InputPort<bool>("lock_target", false, "Lock consistent rays into one odom target"));
        ports.insert(BT::InputPort<int>("lock_frames", 3, "Rays before the target is locked"));
        ports.insert(BT::InputPort<bool>("reset_timeout_on_lock", false, "Restart timeout_msec once locked"));
        ports.insert(BT::InputPort<double>("tolerance", 0.012, "Centered when the bearings are within this (rad)"));
        ports.insert(BT::InputPort<double>("max_bearing_jump_rad", 0.15,
                                           "Largest frame-to-frame bearing change taken as the same target"));
        ports.insert(BT::InputPort<double>("bearing_filter_alpha", 0.25, "Weight of each new bearing in the filter"));
        ports.insert(BT::InputPort<bool>("use_lateral_velocity_alignment", false,
                                         "Move sideways at lateral_alignment_velocity instead of in steps"));
        ports.insert(BT::InputPort<double>("lateral_alignment_velocity", 0.10, "Sideways speed (m/s)"));
        ports.insert(BT::InputPort<int>("blind_alignment_frames", 1,
                                        "Frames before a velocity alignment may finish without detections"));
        ports.insert(BT::InputPort<double>("max_lateral_step", 0.10, "Largest sideways step per update (m)"));
        ports.insert(BT::InputPort<double>("max_depth_step", 0.08, "Largest z step per update (m)"));
        ports.insert(BT::InputPort<int>("settle_frames", 8, "Centered frames in a row before SUCCESS"));
        ports.insert(BT::InputPort<int>("min_msec", 1500, "Minimum alignment time"));
        ports.insert(BT::InputPort<int>("timeout_msec", 15000, "Maximum alignment time before FAILURE"));
        ports.insert(BT::InputPort<int>("update_msec", 300, "Minimum time between corrections"));
        ports.insert(BT::InputPort<int>("initial_detection_max_age_msec", 0,
                                        "Use a frame this old at the start, e.g. the one the tree waited for"));
        ports.insert(
            BT::InputPort<int>("detection_loss_msec", 0, "FAILURE after this long without a matching frame; 0 never"));
        ports.insert(BT::OutputPort<double>("detection_distance_m", "The target's remaining range once aligned (m)"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        if (!DetectionFilter::read(*this, filter_, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        int initial_detection_max_age_msec = 0;
        getInput("target_distance", target_distance_);
        getInput("use_detection_distance", use_detection_distance_);
        getInput("max_projection_distance", max_projection_distance_);
        getInput("target_offset_left", target_offset_left_);
        getInput("target_offset_up", target_offset_up_);
        getInput("align_depth", align_depth_);
        getInput("lock_target", lock_target_);
        getInput("lock_frames", lock_frames_);
        getInput("reset_timeout_on_lock", reset_timeout_on_lock_);
        getInput("tolerance", tolerance_);
        getInput("max_bearing_jump_rad", max_bearing_jump_rad_);
        getInput("bearing_filter_alpha", bearing_filter_alpha_);
        getInput("use_lateral_velocity_alignment", velocity_alignment_);
        getInput("lateral_alignment_velocity", lateral_alignment_velocity_);
        getInput("blind_alignment_frames", blind_alignment_frames_);
        getInput("max_lateral_step", max_lateral_step_);
        getInput("max_depth_step", max_depth_step_);
        getInput("settle_frames", settle_frames_);
        getInput("min_msec", min_msec_);
        getInput("timeout_msec", timeout_msec_);
        getInput("update_msec", update_msec_);
        getInput("initial_detection_max_age_msec", initial_detection_max_age_msec);
        getInput("detection_loss_msec", detection_loss_msec_);
        if (filter_.task.empty() || filter_.class_id.empty() || target_distance_ <= 0.0 ||
            max_projection_distance_ < 0.0 || !std::isfinite(target_offset_left_) ||
            !std::isfinite(target_offset_up_) || lock_frames_ <= 0 || tolerance_ <= 0.0 ||
            max_bearing_jump_rad_ <= 0.0 || bearing_filter_alpha_ <= 0.0 || bearing_filter_alpha_ > 1.0 ||
            lateral_alignment_velocity_ <= 0.0 || blind_alignment_frames_ <= 0 || max_lateral_step_ <= 0.0 ||
            max_depth_step_ <= 0.0 || settle_frames_ <= 0 || min_msec_ < 0 || timeout_msec_ <= 0 || update_msec_ <= 0 ||
            initial_detection_max_age_msec < 0 || detection_loss_msec_ < 0) {
            RCLCPP_ERROR(logger_,
                         "LateralAlignToDetection needs a task, a class_id and positive distances, gains and "
                         "timings.");
            return BT::NodeStatus::FAILURE;
        }
        if (!filter_.usableCamera(node_, *this, logger_, true)) {
            return BT::NodeStatus::FAILURE;
        }

        started_at_ = SteadyClock::now();
        deadline_ = started_at_ + std::chrono::milliseconds(timeout_msec_);
        last_processed_ = started_at_ - std::chrono::milliseconds(initial_detection_max_age_msec);
        last_detection_at_ = started_at_;
        last_commanded_ = started_at_ - std::chrono::milliseconds(update_msec_);
        have_filtered_bearing_ = false;
        locked_ = false;
        have_velocity_target_ = false;
        accepted_frames_ = 0;
        locked_frames_ = 0;
        settled_frames_ = 0;
        aligned_distance_ = std::numeric_limits<double>::quiet_NaN();
        plane_samples_.clear();
        // The frame the tree waited for may be the newest there is: steer on it now.
        return onRunning();
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "LateralAlignToDetection failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "LateralAlignToDetection timed out: %s.", filter_.describe().c_str());
            hold();
            return BT::NodeStatus::FAILURE;
        }
        if (locked_) {
            return tickLockedTarget();
        }

        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection == nullptr) {
            // Detectors drop frames of an object that has not moved: a velocity
            // alignment finishes on the last ray once enough frames confirmed it.
            if (have_velocity_target_) {
                const BT::NodeStatus status = tickVelocityTarget();
                if (status != BT::NodeStatus::RUNNING) {
                    return status;
                }
            }
            if (detection_loss_msec_ > 0 &&
                SteadyClock::now() - last_detection_at_ >= std::chrono::milliseconds(detection_loss_msec_)) {
                RCLCPP_WARN(logger_, "LateralAlignToDetection lost %s for %dms.", filter_.describe().c_str(),
                            detection_loss_msec_);
                hold();
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }
        last_processed_ = match.received_at;
        last_detection_at_ = match.received_at;

        double distance = target_distance_;
        if (use_detection_distance_) {
            if (!validDistanceValue(match.detection->distance_m)) {
                RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "LateralAlignToDetection: waiting for a range.");
                return skipFrame();
            }
            distance = match.detection->distance_m;
        }
        // The aim point's bearings, still optical (positive right / below).
        const double horizontal =
            match.detection->bearing_horizontal + std::atan2(-target_offset_left_, target_distance_);
        const double vertical = match.detection->bearing_vertical + std::atan2(-target_offset_up_, target_distance_);
        if (!filterBearing(horizontal, vertical)) {
            return skipFrame();
        }
        ++accepted_frames_;
        // Only a frame whose bearing was accepted gives the range: a jump's is
        // another object's.
        if (use_detection_distance_) {
            distance = aligned_distance_ = lock_target_ ? recordPlaneSample(distance) : distance;
        }
        if (max_projection_distance_ > 0.0) {
            distance = std::min(distance, max_projection_distance_);
        }

        if (lock_target_) {
            if (++locked_frames_ < lock_frames_) {
                return BT::NodeStatus::RUNNING;
            }
            lockTarget(distance);
            return tickLockedTarget();
        }

        const bool old_enough = SteadyClock::now() - started_at_ >= std::chrono::milliseconds(min_msec_);
        if (old_enough && std::fabs(filtered_horizontal_) <= tolerance_ &&
            (!align_depth_ || std::fabs(filtered_vertical_) <= tolerance_)) {
            if (velocity_alignment_) {
                hold();
            }
            if (++settled_frames_ >= settle_frames_) {
                return succeed(aligned_distance_);
            }
            return BT::NodeStatus::RUNNING;
        }
        settled_frames_ = 0;

        if (velocity_alignment_) {
            if (accepted_frames_ < blind_alignment_frames_) {
                return BT::NodeStatus::RUNNING;
            }
            const auto target = rayPoint(distance);
            velocity_target_ = {target[0], target[1]};
            velocity_target_distance_ = distance;
            have_velocity_target_ = true;
            return tickVelocityTarget();
        }

        if (SteadyClock::now() - last_commanded_ < std::chrono::milliseconds(update_msec_)) {
            return BT::NodeStatus::RUNNING;
        }
        // A bounded step toward the ray, as rayPoint().
        std::array<double, 3> target = horizontalTarget(node_);
        addBodyOffset(target, 0.0,
                      std::clamp(-distance * std::tan(filtered_horizontal_), -max_lateral_step_, max_lateral_step_),
                      actualYaw(node_));
        if (align_depth_) {
            target[2] = node_.measured_pos[2] +
                        std::clamp(-distance * std::tan(filtered_vertical_), -max_depth_step_, max_depth_step_);
        }
        sendPosition(node_, *setpoint_publisher_, *clock_, target);
        last_commanded_ = SteadyClock::now();
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        hold();
        RCLCPP_INFO(logger_, "LateralAlignToDetection halted.");
    }

   private:
    // Low-passes the bearings, rejecting a jump (another object, a box that
    // swapped) rather than steering on it.
    bool filterBearing(const double horizontal, const double vertical) {
        if (!have_filtered_bearing_) {
            filtered_horizontal_ = horizontal;
            filtered_vertical_ = vertical;
            have_filtered_bearing_ = true;
            return true;
        }
        if (std::fabs(horizontal - filtered_horizontal_) > max_bearing_jump_rad_ ||
            std::fabs(vertical - filtered_vertical_) > max_bearing_jump_rad_) {
            locked_frames_ = 0;
            plane_samples_.clear();
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                                 "LateralAlignToDetection: ignored a bearing jump (%.3f, %.3f) -> (%.3f, %.3f).",
                                 filtered_horizontal_, filtered_vertical_, horizontal, vertical);
            return false;
        }
        filtered_horizontal_ += bearing_filter_alpha_ * (horizontal - filtered_horizontal_);
        filtered_vertical_ += bearing_filter_alpha_ * (vertical - filtered_vertical_);
        return true;
    }

    // Where the filtered ray points, `distance` out: sideways (right of center
    // is a negative left offset) and, when aligning depth, in z (below center
    // is a smaller z). Leaves z NaN otherwise.
    std::array<double, 3> rayPoint(const double distance) const {
        std::array<double, 3> target = horizontalTarget(node_);
        addBodyOffset(target, 0.0, -distance * std::tan(filtered_horizontal_), actualYaw(node_));
        if (align_depth_) {
            target[2] = std::min(node_.measured_pos[2] - distance * std::tan(filtered_vertical_), MAX_VISION_Z);
        }
        return target;
    }

    void lockTarget(const double distance) {
        locked_target_ = rayPoint(distance);
        locked_distance_ = distance;
        locked_ = true;
        settled_frames_ = 0;
        if (reset_timeout_on_lock_) {
            deadline_ = SteadyClock::now() + std::chrono::milliseconds(timeout_msec_);
        }
        sendPosition(node_, *setpoint_publisher_, *clock_, locked_target_);
        RCLCPP_INFO(logger_, "LateralAlignToDetection: locked target (%.2f, %.2f, %.2f).", locked_target_[0],
                    locked_target_[1], locked_target_[2]);
    }

    BT::NodeStatus tickLockedTarget() {
        const double tolerance = locked_distance_ * std::tan(tolerance_);
        const bool old_enough = SteadyClock::now() - started_at_ >= std::chrono::milliseconds(min_msec_);
        if (old_enough && std::fabs(leftOf({locked_target_[0], locked_target_[1]})) <= tolerance &&
            (!align_depth_ || std::fabs(locked_target_[2] - node_.measured_pos[2]) <= tolerance)) {
            if (++settled_frames_ < settle_frames_) {
                return BT::NodeStatus::RUNNING;
            }
            double remaining = aligned_distance_;
            if (!plane_samples_.empty()) {
                remaining = median(plane_samples_) - forwardProgress();
            }
            if (use_detection_distance_ && !validDistanceValue(remaining)) {
                RCLCPP_ERROR(logger_, "LateralAlignToDetection: no valid range left to the target.");
                return BT::NodeStatus::FAILURE;
            }
            return succeed(remaining);
        }
        settled_frames_ = 0;
        return BT::NodeStatus::RUNNING;
    }

    // A frame without a range or with a rejected bearing steers nothing. A
    // velocity alignment still stops on its last target, as when no frame
    // comes, instead of moving on past it until timeout_msec.
    BT::NodeStatus skipFrame() { return have_velocity_target_ ? tickVelocityTarget() : BT::NodeStatus::RUNNING; }

    BT::NodeStatus tickVelocityTarget() {
        const double left = -leftOf(velocity_target_);
        if (std::fabs(left) <= std::max(0.02, velocity_target_distance_ * std::tan(tolerance_))) {
            hold();
            return succeed(aligned_distance_);
        }
        if (SteadyClock::now() - last_commanded_ >= std::chrono::milliseconds(update_msec_)) {
            sendVelocity(node_, *setpoint_publisher_, *clock_,
                         {UNSPECIFIED_PORT, std::copysign(lateral_alignment_velocity_, left), UNSPECIFIED_PORT});
            last_commanded_ = SteadyClock::now();
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus succeed(const double distance) {
        if (use_detection_distance_) {
            setOutput("detection_distance_m", distance);
        }
        RCLCPP_INFO(logger_, "LateralAlignToDetection: centered on %s, range %.2fm.", filter_.describe().c_str(),
                    distance);
        return BT::NodeStatus::SUCCESS;
    }

    // How far the vehicle is left of `point`, across its heading.
    double leftOf(const std::array<double, 2> &point) const {
        const double yaw = actualYaw(node_);
        return -std::sin(yaw) * (node_.measured_pos[0] - point[0]) + std::cos(yaw) * (node_.measured_pos[1] - point[1]);
    }

    // Progress along the heading the first range was taken on.
    double forwardProgress() const {
        return std::cos(plane_yaw_) * (node_.measured_pos[0] - plane_origin_[0]) +
               std::sin(plane_yaw_) * (node_.measured_pos[1] - plane_origin_[1]);
    }

    // A range from the box size moves as the detector trims the box. Each range
    // plus the progress since the first becomes one estimate of where the
    // target's plane is along the heading; the median of the last few is the
    // remaining range used.
    double recordPlaneSample(const double distance) {
        if (plane_samples_.empty()) {
            plane_origin_ = {node_.measured_pos[0], node_.measured_pos[1]};
            plane_yaw_ = actualYaw(node_);
        }
        plane_samples_.push_back(forwardProgress() + distance);
        constexpr std::size_t MAX_SAMPLES = 15;
        if (plane_samples_.size() > MAX_SAMPLES) {
            plane_samples_.erase(plane_samples_.begin());
        }
        return median(plane_samples_) - forwardProgress();
    }

    void hold() { node_.hold({true, true, align_depth_, false, false, false}); }

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    SteadyClock::time_point started_at_;
    SteadyClock::time_point deadline_;
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point last_detection_at_;
    SteadyClock::time_point last_commanded_;
    double target_distance_ = 1.15;
    double max_projection_distance_ = 0.0;
    double target_offset_left_ = 0.0;
    double target_offset_up_ = 0.0;
    double tolerance_ = 0.012;
    double max_bearing_jump_rad_ = 0.15;
    double bearing_filter_alpha_ = 0.25;
    double lateral_alignment_velocity_ = 0.10;
    double max_lateral_step_ = 0.10;
    double max_depth_step_ = 0.08;
    double filtered_horizontal_ = 0.0;
    double filtered_vertical_ = 0.0;
    double aligned_distance_ = std::numeric_limits<double>::quiet_NaN();
    std::array<double, 3> locked_target_{};
    double locked_distance_ = 1.15;
    std::array<double, 2> velocity_target_{};
    double velocity_target_distance_ = 1.0;
    std::vector<double> plane_samples_;
    std::array<double, 2> plane_origin_{};
    double plane_yaw_ = 0.0;
    int lock_frames_ = 3;
    int locked_frames_ = 0;
    int blind_alignment_frames_ = 1;
    int accepted_frames_ = 0;
    int settle_frames_ = 8;
    int settled_frames_ = 0;
    int min_msec_ = 1500;
    int timeout_msec_ = 15000;
    int update_msec_ = 300;
    int detection_loss_msec_ = 0;
    bool use_detection_distance_ = false;
    bool align_depth_ = true;
    bool lock_target_ = false;
    bool reset_timeout_on_lock_ = false;
    bool velocity_alignment_ = false;
    bool have_filtered_bearing_ = false;
    bool locked_ = false;
    bool have_velocity_target_ = false;
};

class ForwardSweepAlignAction : public BT::StatefulActionNode {
   public:
    ForwardSweepAlignAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                            SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                            rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
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
        if (!DetectionFilter::read(*this, filter_, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        getInput("attempts", attempts_);
        getInput("num_sweeps", num_sweeps_);
        getInput("forward_step", forward_step_);
        getInput("sample_timeout_msec", sample_timeout_msec_);
        getInput("move_timeout_msec", move_timeout_msec_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_, "ForwardSweepAlign needs a task port, e.g. <ForwardSweepAlign task=\"gate\"/>.");
            return BT::NodeStatus::FAILURE;
        }
        if (attempts_ <= 0 || num_sweeps_ <= 0 || !std::isfinite(forward_step_) ||
            !std::isfinite(sample_timeout_msec_) || sample_timeout_msec_ <= 0.0 || move_timeout_msec_ <= 0) {
            RCLCPP_ERROR(logger_,
                         "ForwardSweepAlign needs positive attempts, num_sweeps, sample_timeout_msec and "
                         "move_timeout_msec, and a finite forward_step.");
            return BT::NodeStatus::FAILURE;
        }
        if (!filter_.usableCamera(node_, *this, logger_, true)) {
            return BT::NodeStatus::FAILURE;
        }

        phase_ = Phase::YAW_MOVE;
        sweep_index_ = 0;
        angle_index_ = 0;
        detection_count_ = 0;
        yaw_offset_sum_ = 0.0;
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
        std::array<double, 3> target{node_.positionTarget(0), node_.positionTarget(1), UNSPECIFIED_PORT};
        addBodyOffset(target, forward_step_, 0.0, node_.attitudeTarget(2));
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        phase_ = Phase::FORWARD_MOVE;
        sendPosition(node_, *setpoint_publisher_, *clock_, target);
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
        step_yaw_ = normalizeAngle(node_.attitudeTarget(2) + radians(SWEEP_DEGREES[angle_index_]));
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        phase_ = Phase::YAW_MOVE;
        sendYaw(node_, *setpoint_publisher_, *clock_, step_yaw_);
        RCLCPP_INFO(logger_, "ForwardSweepAlign: sweep %d/%d yaw step %.1f deg.", sweep_index_ + 1, num_sweeps_,
                    SWEEP_DEGREES[angle_index_]);
    }

    BT::NodeStatus tickYawMove() {
        if (motionTimedOut("yaw move")) {
            return BT::NodeStatus::FAILURE;
        }
        if (freshAndWithinTolerance(8, ANGLE_TOLERANCE)) {
            detection_count_ = 0;
            yaw_offset_sum_ = 0.0;
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
            // Each frame's yaw toward the object, from the heading it was seen
            // on: the vehicle is still settling into the step while it samples.
            yaw_offset_sum_ += normalizeAngle(yawToward(node_, match.detection->bearing_horizontal) - step_yaw_);
            ++detection_count_;
            if (detection_count_ >= attempts_) {
                const double target_yaw =
                    normalizeAngle(step_yaw_ + yaw_offset_sum_ / static_cast<double>(detection_count_));
                start_updates_ = node_.control_error_updates;
                deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
                phase_ = Phase::FINAL_ALIGN;
                sendYaw(node_, *setpoint_publisher_, *clock_, target_yaw);
                RCLCPP_INFO(logger_, "ForwardSweepAlign: found %s, yaw %.3f rad.", filter_.describe().c_str(),
                            target_yaw);
            }
            return BT::NodeStatus::RUNNING;
        }

        if (SteadyClock::now() < deadline_) {
            return BT::NodeStatus::RUNNING;
        }

        ++angle_index_;
        if (angle_index_ < SWEEP_DEGREES.size()) {
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

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
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
    double step_yaw_ = 0.0;        // the yaw this step of the sweep turns to
    double yaw_offset_sum_ = 0.0;  // the samples' yaws toward the object, relative to step_yaw_
};

class SweepCheckAction : public BT::StatefulActionNode {
   public:
    SweepCheckAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                     SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                     rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
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
        if (!DetectionFilter::read(*this, filter_, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        getInput("attempts", attempts_);
        getInput("sample_timeout_msec", sample_timeout_msec_);
        getInput("move_timeout_msec", move_timeout_msec_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_, "SweepCheck needs a task port, e.g. <SweepCheck task=\"gate\"/>.");
            return BT::NodeStatus::FAILURE;
        }
        if (attempts_ <= 0 || !std::isfinite(sample_timeout_msec_) || sample_timeout_msec_ <= 0.0 ||
            move_timeout_msec_ <= 0) {
            RCLCPP_ERROR(logger_, "SweepCheck needs positive attempts, sample_timeout_msec and move_timeout_msec.");
            return BT::NodeStatus::FAILURE;
        }
        if (!filter_.usableCamera(node_, *this, logger_, true)) {
            return BT::NodeStatus::FAILURE;
        }
        phase_ = Phase::YAW_MOVE;
        angle_index_ = 0;
        detection_count_ = 0;
        yaw_offset_sum_ = 0.0;
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
        step_yaw_ = normalizeAngle(node_.attitudeTarget(2) + radians(SWEEP_DEGREES[angle_index_]));
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        phase_ = Phase::YAW_MOVE;
        sendYaw(node_, *setpoint_publisher_, *clock_, step_yaw_);
        RCLCPP_INFO(logger_, "SweepCheck: yaw step %.1f deg.", SWEEP_DEGREES[angle_index_]);
    }

    BT::NodeStatus tickYawMove() {
        if (motionTimedOut("yaw move")) {
            return BT::NodeStatus::FAILURE;
        }
        if (freshAndWithinTolerance(8, ANGLE_TOLERANCE)) {
            detection_count_ = 0;
            yaw_offset_sum_ = 0.0;
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
            // Each frame's yaw toward the object, from the heading it was seen
            // on: the vehicle is still settling into the step while it samples.
            yaw_offset_sum_ += normalizeAngle(yawToward(node_, match.detection->bearing_horizontal) - step_yaw_);
            ++detection_count_;
            if (detection_count_ >= attempts_) {
                const double target_yaw =
                    normalizeAngle(step_yaw_ + yaw_offset_sum_ / static_cast<double>(detection_count_));
                start_updates_ = node_.control_error_updates;
                deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
                phase_ = Phase::FINAL_ALIGN;
                sendYaw(node_, *setpoint_publisher_, *clock_, target_yaw);
                RCLCPP_INFO(logger_, "SweepCheck: found %s, yaw %.3f rad.", filter_.describe().c_str(), target_yaw);
            }
            return BT::NodeStatus::RUNNING;
        }
        if (SteadyClock::now() < deadline_) {
            return BT::NodeStatus::RUNNING;
        }

        ++angle_index_;
        if (angle_index_ < SWEEP_DEGREES.size()) {
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

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
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
    double step_yaw_ = 0.0;        // the yaw this step of the sweep turns to
    double yaw_offset_sum_ = 0.0;  // the samples' yaws toward the object, relative to step_yaw_
};

class SweepAngleAction : public BT::StatefulActionNode {
   public:
    SweepAngleAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                     SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                     rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = DetectionFilter::ports();
        ports.insert(BT::InputPort<int>("attempts", 4, "Detection frames sampled at each yaw"));
        ports.insert(BT::InputPort<double>("sample_timeout_msec", 2500.0, "Maximum detection sample time per yaw"));
        ports.insert(BT::InputPort<int>("move_timeout_msec", 20000, "Maximum wait for each yaw move"));
        ports.insert(BT::InputPort<std::string>(
            "side", "", "Only accept detections left or right of reference_yaw (empty = any side)"));
        ports.insert(BT::InputPort<double>("reference_yaw", UNSPECIFIED_PORT, "Odom yaw `side` is measured from"));
        ports.insert(BT::OutputPort<double>("yaw", "Odom yaw toward the detected object"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        if (!DetectionFilter::read(*this, filter_, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        getInput("attempts", attempts_);
        getInput("sample_timeout_msec", sample_timeout_msec_);
        getInput("move_timeout_msec", move_timeout_msec_);
        side_.clear();
        getInput("side", side_);
        reference_yaw_ = UNSPECIFIED_PORT;
        getInput("reference_yaw", reference_yaw_);

        if (filter_.task.empty()) {
            RCLCPP_ERROR(logger_, "SweepAngle needs a task port, e.g. <SweepAngle task=\"slalom\"/>.");
            return BT::NodeStatus::FAILURE;
        }
        if (attempts_ <= 0 || !std::isfinite(sample_timeout_msec_) || sample_timeout_msec_ <= 0.0 ||
            move_timeout_msec_ <= 0) {
            RCLCPP_ERROR(logger_, "SweepAngle needs positive attempts, sample_timeout_msec and move_timeout_msec.");
            return BT::NodeStatus::FAILURE;
        }
        if (!side_.empty() && ((side_ != "left" && side_ != "right") || !std::isfinite(reference_yaw_))) {
            RCLCPP_ERROR(logger_, "SweepAngle side must be left or right, with a reference_yaw; got side='%s'.",
                         side_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (!filter_.usableCamera(node_, *this, logger_, true)) {
            return BT::NodeStatus::FAILURE;
        }
        phase_ = Phase::YAW_MOVE;
        angle_index_ = 0;
        detection_count_ = 0;
        yaw_offset_sum_ = 0.0;
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
        step_yaw_ = normalizeAngle(node_.attitudeTarget(2) + radians(SWEEP_DEGREES[angle_index_]));
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        phase_ = Phase::YAW_MOVE;
        sendYaw(node_, *setpoint_publisher_, *clock_, step_yaw_);
        RCLCPP_INFO(logger_, "SweepAngle: yaw step %.1f deg.", SWEEP_DEGREES[angle_index_]);
    }

    BT::NodeStatus tickYawMove() {
        if (motionTimedOut("yaw move")) {
            return BT::NodeStatus::FAILURE;
        }
        if (freshAndWithinTolerance(8, ANGLE_TOLERANCE)) {
            detection_count_ = 0;
            yaw_offset_sum_ = 0.0;
            last_processed_ = SteadyClock::now();
            deadline_ = last_processed_ + std::chrono::milliseconds(static_cast<int>(sample_timeout_msec_));
            phase_ = Phase::SAMPLE_DETECTIONS;
        }
        return BT::NodeStatus::RUNNING;
    }

    // Whether a detection lies on side_ of reference_yaw_ (yaw grows
    // counter-clockwise, so left is the larger angle); every detection when no
    // side is given.
    bool onSide(const Detection &detection) const {
        if (side_.empty()) {
            return true;
        }
        const double offset = normalizeAngle(yawToward(node_, detection.bearing_horizontal) - reference_yaw_);
        return side_ == "left" ? offset > 0.0 : offset < 0.0;
    }

    BT::NodeStatus tickSampleDetections() {
        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1),
                                        [this](const Detection &detection) { return onSide(detection); });
        if (match.detection != nullptr) {
            last_processed_ = match.received_at;
            // Each frame's yaw toward the object, from the heading it was seen
            // on: the vehicle is still settling into the step while it samples.
            yaw_offset_sum_ += normalizeAngle(yawToward(node_, match.detection->bearing_horizontal) - step_yaw_);
            ++detection_count_;
            if (detection_count_ >= attempts_) {
                const double target_yaw =
                    normalizeAngle(step_yaw_ + yaw_offset_sum_ / static_cast<double>(detection_count_));
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
        if (angle_index_ < SWEEP_DEGREES.size()) {
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

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
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
    double step_yaw_ = 0.0;        // the yaw this step of the sweep turns to
    double yaw_offset_sum_ = 0.0;  // the samples' yaws toward the object, relative to step_yaw_
    std::string side_;
    double reference_yaw_ = UNSPECIFIED_PORT;
};

// Moves forward at a fixed velocity, steering the heading toward a front-camera
// detection every frame, until the target is close, lost, max_dist away or
// timeout_msec on. Propulsion starts once confirmation_frames frames in a row
// have seen it.
class ForwardContinuousAlignAction : public BT::StatefulActionNode {
   public:
    ForwardContinuousAlignAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                                 SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                 rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = DetectionFilter::ports();
        ports.insert(BT::InputPort<double>("max_dist", 10.0, "Maximum forward travel before SUCCESS (m)"));
        ports.insert(BT::InputPort<double>("forward_velocity", 1.0, "Forward velocity (m/s)"));
        ports.insert(BT::InputPort<double>("close_distance", 3.0, "Range at which the target counts as close (m)"));
        ports.insert(BT::InputPort<int>("detection_loss_msec", 3000, "Time without a frame that counts as lost"));
        ports.insert(BT::InputPort<int>("timeout_msec", 30000, "Maximum time before SUCCESS"));
        ports.insert(BT::InputPort<int>("confirmation_frames", 1, "Frames in a row before moving"));
        ports.insert(BT::InputPort<bool>("continue_on_loss", true,
                                         "Carry on along the last heading when lost, instead of FAILURE"));
        ports.insert(BT::InputPort<bool>("stop_on_close", true, "SUCCESS as soon as the target is close"));
        ports.insert(BT::InputPort<bool>("loss_is_success", false, "SUCCESS when lost after being close"));
        ports.insert(BT::InputPort<double>("max_yaw_step_deg", 45.0, "Largest heading correction per frame"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        if (!DetectionFilter::read(*this, filter_, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        getInput("max_dist", max_dist_);
        getInput("forward_velocity", forward_velocity_);
        getInput("close_distance", close_distance_);
        getInput("detection_loss_msec", detection_loss_msec_);
        getInput("timeout_msec", timeout_msec_);
        getInput("confirmation_frames", confirmation_frames_);
        getInput("continue_on_loss", continue_on_loss_);
        getInput("stop_on_close", stop_on_close_);
        getInput("loss_is_success", loss_is_success_);
        getInput("max_yaw_step_deg", max_yaw_step_deg_);
        if (filter_.task.empty() || max_dist_ <= 0.0 || forward_velocity_ <= 0.0 || detection_loss_msec_ <= 0 ||
            timeout_msec_ <= 0 || confirmation_frames_ <= 0 || max_yaw_step_deg_ <= 0.0) {
            RCLCPP_ERROR(logger_,
                         "ForwardContinuousAlign needs a task and positive max_dist, forward_velocity, "
                         "detection_loss_msec, timeout_msec, confirmation_frames and max_yaw_step_deg.");
            return BT::NodeStatus::FAILURE;
        }
        if (!filter_.usableCamera(node_, *this, logger_, true)) {
            return BT::NodeStatus::FAILURE;
        }

        initial_pos_ = {node_.measured_pos[0], node_.measured_pos[1]};
        last_processed_ = SteadyClock::now();
        deadline_ = last_processed_ + std::chrono::milliseconds(timeout_msec_);
        moving_ = false;
        seen_ = false;
        seen_close_ = false;
        consecutive_frames_ = 0;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "ForwardContinuousAlign failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        const double travel =
            std::hypot(node_.measured_pos[0] - initial_pos_[0], node_.measured_pos[1] - initial_pos_[1]);
        if (travel >= max_dist_) {
            RCLCPP_INFO(logger_, "ForwardContinuousAlign: max distance %.2fm reached.", max_dist_);
            return stop(BT::NodeStatus::SUCCESS);
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "ForwardContinuousAlign: timeout reached after %dms.", timeout_msec_);
            return stop(BT::NodeStatus::SUCCESS);
        }

        // Before latestMatch, so a frame arriving between the two is never taken for a miss.
        const SteadyClock::time_point newest_frame = node_.vision().latest(filter_.camera).received_at;
        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection == nullptr && newest_frame > last_processed_) {
            // A frame that missed it: confirmation_frames counts frames in a row.
            consecutive_frames_ = 0;
        }
        if (match.detection != nullptr) {
            last_processed_ = match.received_at;
            seen_ = true;
            const bool confirmed = ++consecutive_frames_ >= confirmation_frames_;
            steer(*match.detection);
            if (confirmed && !moving_) {
                sendVelocity(node_, *setpoint_publisher_, *clock_, {forward_velocity_, 0.0, UNSPECIFIED_PORT});
                moving_ = true;
                RCLCPP_INFO(logger_, "ForwardContinuousAlign: %s confirmed; forward at %.2fm/s.",
                            filter_.describe().c_str(), forward_velocity_);
            }
            if (confirmed && validDistanceValue(match.detection->distance_m) &&
                match.detection->distance_m <= close_distance_) {
                seen_close_ = true;
                if (stop_on_close_) {
                    RCLCPP_INFO(logger_, "ForwardContinuousAlign: close, %.2fm.", match.detection->distance_m);
                    return stop(BT::NodeStatus::SUCCESS);
                }
            }
        } else if (seen_ && SteadyClock::now() - last_processed_ >= std::chrono::milliseconds(detection_loss_msec_)) {
            // A target that was close and then leaves the camera's view has
            // been passed; lost far away, it has not.
            if (loss_is_success_ && seen_close_) {
                RCLCPP_INFO(logger_, "ForwardContinuousAlign: passed the target after %.2fm.", travel);
                return stop(BT::NodeStatus::SUCCESS);
            }
            if (!continue_on_loss_) {
                RCLCPP_WARN(logger_, "ForwardContinuousAlign: lost %s.", filter_.describe().c_str());
                return stop(BT::NodeStatus::FAILURE);
            }
            RCLCPP_WARN(logger_, "ForwardContinuousAlign: lost %s; carrying on along the heading.",
                        filter_.describe().c_str());
            seen_ = false;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        stop(BT::NodeStatus::IDLE);
        RCLCPP_INFO(logger_, "ForwardContinuousAlign halted.");
    }

   private:
    // Toward the detection: its yaw_deg turn when the post-processor gives
    // one, else its bearing; a step at most max_yaw_step_deg from the heading.
    void steer(const Detection &detection) {
        const double max_step = radians(max_yaw_step_deg_);
        const double turn =
            std::clamp(orientationYaw(detection).value_or(-detection.bearing_horizontal), -max_step, max_step);
        sendYaw(node_, *setpoint_publisher_, *clock_, actualYaw(node_) + turn);
    }

    BT::NodeStatus stop(const BT::NodeStatus status) {
        if (moving_) {
            stopHorizontal(node_);
            moving_ = false;
        }
        return status;
    }

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    std::array<double, 2> initial_pos_ = {};
    SteadyClock::time_point deadline_;
    SteadyClock::time_point last_processed_;
    double max_dist_ = 10.0;
    double forward_velocity_ = 1.0;
    double close_distance_ = 3.0;
    double max_yaw_step_deg_ = 45.0;
    int detection_loss_msec_ = 3000;
    int timeout_msec_ = 30000;
    int confirmation_frames_ = 1;
    int consecutive_frames_ = 0;
    bool continue_on_loss_ = true;
    bool stop_on_close_ = true;
    bool loss_is_success_ = false;
    bool moving_ = false;
    bool seen_ = false;
    bool seen_close_ = false;
};

// One forward pass along the heading, at a fixed velocity, through something a
// previous node ranged (the gate, a slalom opening): vision_distance_m plus
// pass_distance, measured by odometry. No detections: close up, the camera
// loses what it is passing through.
class ForwardFixedTransitAction : public BT::StatefulActionNode {
   public:
    ForwardFixedTransitAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                              SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                              rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("vision_distance_m", "Range to what is being passed through (m)"),
                BT::InputPort<double>("pass_distance", 0.0, "Distance to carry on past it (m)"),
                BT::InputPort<double>("min_transit_distance", 0.0, "Shortest pass (m)"),
                BT::InputPort<double>("max_planned_distance", 0.0, "Longest pass (m); 0 for no limit"),
                BT::InputPort<double>("completion_tolerance_m", 0.15, "Shortfall accepted at the end (m)"),
                BT::InputPort<double>("max_dist", 10.0, "FAILURE if the pass would be longer (m)"),
                BT::InputPort<double>("forward_velocity", 0.3, "Forward velocity (m/s)"),
                BT::InputPort<int>("timeout_msec", 30000, "Maximum time before FAILURE"),
                BT::InputPort<double>("min_progress_m", 0.10, "Progress that shows the vehicle is moving (m)"),
                BT::InputPort<int>("progress_timeout_msec", 4000,
                                   "FAILURE after this long without min_progress_m forward")};
    }

    BT::NodeStatus onStart() override {
        double vision_distance = UNSPECIFIED_PORT;
        double pass_distance = 0.0;
        double min_transit_distance = 0.0;
        double max_planned_distance = 0.0;
        double completion_tolerance = 0.15;
        double max_dist = 10.0;
        int timeout_msec = 30000;
        // A missing blackboard entry fails the node rather than quietly taking
        // the default: max_dist, for one, bounds the pass.
        if (!readPort(*this, "vision_distance_m", vision_distance, logger_) ||
            !readPort(*this, "pass_distance", pass_distance, logger_) ||
            !readPort(*this, "min_transit_distance", min_transit_distance, logger_) ||
            !readPort(*this, "max_planned_distance", max_planned_distance, logger_) ||
            !readPort(*this, "completion_tolerance_m", completion_tolerance, logger_) ||
            !readPort(*this, "max_dist", max_dist, logger_) ||
            !readPort(*this, "forward_velocity", forward_velocity_, logger_) ||
            !readPort(*this, "timeout_msec", timeout_msec, logger_) ||
            !readPort(*this, "min_progress_m", min_progress_m_, logger_) ||
            !readPort(*this, "progress_timeout_msec", progress_timeout_msec_, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        if (!validDistanceValue(vision_distance) || pass_distance < 0.0 || min_transit_distance < 0.0 ||
            max_planned_distance < 0.0 || completion_tolerance < 0.0 || max_dist <= 0.0 || forward_velocity_ <= 0.0 ||
            timeout_msec <= 0 || min_progress_m_ <= 0.0 || progress_timeout_msec_ <= 0) {
            RCLCPP_ERROR(logger_, "ForwardFixedTransit needs a valid vision_distance_m and positive settings.");
            return BT::NodeStatus::FAILURE;
        }

        double planned = std::max(min_transit_distance, vision_distance + pass_distance);
        if (max_planned_distance > 0.0 && planned > max_planned_distance) {
            RCLCPP_WARN(logger_, "ForwardFixedTransit: shortening the %.2fm pass to %.2fm.", planned,
                        max_planned_distance);
            planned = max_planned_distance;
        }
        if (planned > max_dist) {
            RCLCPP_WARN(logger_, "ForwardFixedTransit: a %.2fm pass is over the %.2fm limit.", planned, max_dist);
            return BT::NodeStatus::FAILURE;
        }
        if (!node_.subAlive()) {
            return BT::NodeStatus::FAILURE;
        }
        completion_distance_ = std::max(min_transit_distance, planned - completion_tolerance);

        start_ = {node_.measured_pos[0], node_.measured_pos[1]};
        heading_ = actualYaw(node_);
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(timeout_msec);
        progress_deadline_ = SteadyClock::now() + std::chrono::milliseconds(progress_timeout_msec_);
        last_progress_ = 0.0;
        sendVelocity(node_, *setpoint_publisher_, *clock_, {forward_velocity_, 0.0, UNSPECIFIED_PORT});
        moving_ = true;
        RCLCPP_INFO(logger_, "ForwardFixedTransit: %.2fm forward at %.2fm/s.", planned, forward_velocity_);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "ForwardFixedTransit failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        const double dx = node_.measured_pos[0] - start_[0];
        const double dy = node_.measured_pos[1] - start_[1];
        // Along the heading it started on: a sideways or backward drift is not progress.
        const double progress = std::cos(heading_) * dx + std::sin(heading_) * dy;
        if (progress <= -REVERSE_ABORT_M) {
            RCLCPP_ERROR(logger_, "ForwardFixedTransit: moved %.2fm backward.", -progress);
            return stop(BT::NodeStatus::FAILURE);
        }
        if (progress >= completion_distance_) {
            RCLCPP_INFO(logger_, "ForwardFixedTransit: passed after %.2fm.", progress);
            return stop(BT::NodeStatus::SUCCESS);
        }
        if (progress >= last_progress_ + min_progress_m_) {
            last_progress_ = progress;
            progress_deadline_ = SteadyClock::now() + std::chrono::milliseconds(progress_timeout_msec_);
        } else if (SteadyClock::now() >= progress_deadline_) {
            RCLCPP_WARN(logger_, "ForwardFixedTransit: stuck at %.2fm.", progress);
            return stop(BT::NodeStatus::FAILURE);
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "ForwardFixedTransit: timed out at %.2fm of %.2fm.", progress, completion_distance_);
            return stop(BT::NodeStatus::FAILURE);
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { stop(BT::NodeStatus::IDLE); }

   private:
    static constexpr double REVERSE_ABORT_M = 0.10;

    BT::NodeStatus stop(const BT::NodeStatus status) {
        if (moving_) {
            stopHorizontal(node_);
            moving_ = false;
        }
        return status;
    }

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    std::array<double, 2> start_ = {};
    double heading_ = 0.0;
    double forward_velocity_ = 0.3;
    double completion_distance_ = 0.0;
    double min_progress_m_ = 0.10;
    double last_progress_ = 0.0;
    int progress_timeout_msec_ = 4000;
    bool moving_ = false;
    SteadyClock::time_point deadline_;
    SteadyClock::time_point progress_deadline_;
};

// Squares the camera to an object's face (the torpedo board) at
// desired_distance from it, looking at its center: from the face's heading
// (heading_yaw_deg), the point desired_distance out along its normal, reached
// in steps of at most max_position_step while the heading tracks the bearing.
// Without a heading it only closes to desired_distance or, with
// require_heading, only turns to look at it. A first observation, and any jump
// after, must be confirmed by a second consistent one before it is steered on.
class OrientToDetectionAtDistAction : public BT::StatefulActionNode {
   public:
    OrientToDetectionAtDistAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                                  SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                                  rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = DetectionFilter::ports();
        ports.insert(BT::InputPort<double>("desired_distance", 2.0, "Distance to hold from the target in meters"));
        ports.insert(BT::InputPort<double>("tolerance", 0.35, "Centered when |horizontal bearing| <= tolerance (rad)"));
        ports.insert(BT::InputPort<double>("distance_tolerance", 0.5, "Distance error allowed at completion (m)"));
        ports.insert(BT::InputPort<double>("orient_tolerance_deg", 5.0, "Orientation error allowed at completion"));
        ports.insert(BT::InputPort<double>("max_position_step", 0.5, "Largest translation per update (m)"));
        ports.insert(BT::InputPort<bool>("require_heading", false,
                                         "Wait for heading_yaw_deg instead of only closing the distance"));
        ports.insert(BT::InputPort<int>("settle_frames", 1, "Aligned frames in a row before SUCCESS"));
        ports.insert(BT::InputPort<int>("min_msec", 6500, "Minimum closed-loop alignment time"));
        ports.insert(BT::InputPort<int>("timeout_msec", 30000, "Maximum align time before FAILURE"));
        ports.insert(BT::InputPort<int>("update_msec", 300, "Minimum time between movement corrections"));
        return ports;
    }

    BT::NodeStatus onStart() override {
        if (!DetectionFilter::read(*this, filter_, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        getInput("desired_distance", desired_distance_);
        getInput("tolerance", tolerance_);
        getInput("distance_tolerance", distance_tolerance_);
        getInput("orient_tolerance_deg", orient_tolerance_deg_);
        getInput("max_position_step", max_position_step_);
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
        if (desired_distance_ <= 0.0 || tolerance_ <= 0.0 || distance_tolerance_ <= 0.0 ||
            orient_tolerance_deg_ <= 0.0 || max_position_step_ <= 0.0 || settle_frames_ <= 0 || update_msec_ <= 0 ||
            min_msec_ < 0 || timeout_msec_ <= 0) {
            RCLCPP_ERROR(logger_, "OrientToDetectionAtDist needs positive distances, tolerances and timings.");
            return BT::NodeStatus::FAILURE;
        }
        if (!filter_.usableCamera(node_, *this, logger_, true)) {
            return BT::NodeStatus::FAILURE;
        }

        last_processed_ = SteadyClock::now();
        last_commanded_ = last_processed_ - std::chrono::milliseconds(update_msec_);
        started_at_ = last_processed_;
        deadline_ = started_at_ + std::chrono::milliseconds(timeout_msec_);
        have_filtered_ = false;
        have_filtered_heading_ = false;
        have_candidate_ = false;
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
    // One observation: bearing (optical, positive right), range, and the turn
    // that squares the camera to the face.
    struct Observation {
        double bearing = 0.0;
        double distance = 0.0;
        double heading = 0.0;
    };

    // Whether `seen` is within the spread allowed of `reference`.
    static bool consistent(const Observation &seen, const Observation &reference, const double range_m,
                           const double bearing_rad, const double heading_rad) {
        return std::fabs(seen.distance - reference.distance) <= range_m &&
               std::fabs(normalizeAngle(seen.bearing - reference.bearing)) <= bearing_rad &&
               std::fabs(normalizeAngle(seen.heading - reference.heading)) <= heading_rad;
    }

    // A first observation, or one that jumped from the filtered one, is used
    // only once a second one agrees with it.
    bool confirmed(const Observation &seen) {
        static const double MAX_HEADING_JUMP = radians(18.0);
        if (have_filtered_ &&
            consistent(seen, filtered_, 0.65, 0.25, have_filtered_heading_ ? MAX_HEADING_JUMP : M_PI)) {
            have_candidate_ = false;
            return true;
        }
        static const double MAX_CANDIDATE_HEADING_SPREAD = radians(15.0);
        if (!have_candidate_ || !consistent(seen, candidate_, 0.35, 0.12, MAX_CANDIDATE_HEADING_SPREAD)) {
            candidate_ = seen;
            have_candidate_ = true;
            if (have_filtered_) {
                RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                                     "OrientToDetectionAtDist: waiting to confirm an observation that jumped.");
            }
            return false;
        }
        filtered_ = {normalizeAngle(candidate_.bearing + 0.5 * normalizeAngle(seen.bearing - candidate_.bearing)),
                     0.5 * (candidate_.distance + seen.distance),
                     normalizeAngle(candidate_.heading + 0.5 * normalizeAngle(seen.heading - candidate_.heading))};
        have_filtered_ = true;
        have_filtered_heading_ = true;
        have_candidate_ = false;
        settled_frames_ = 0;
        RCLCPP_INFO(logger_, "OrientToDetectionAtDist: confirmed %s at %.2fm.", filter_.describe().c_str(),
                    filtered_.distance);
        return true;
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
            have_candidate_ = false;
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "OrientToDetectionAtDist: %s has no heading_yaw_deg.",
                                 filter_.describe().c_str());
            // Still look at it: a board the image edge cuts off has no heading
            // until it is back in full view.
            if (SteadyClock::now() - last_commanded_ >= std::chrono::milliseconds(update_msec_)) {
                sendYaw(node_, *setpoint_publisher_, *clock_, yawToward(node_, detection.bearing_horizontal));
                last_commanded_ = SteadyClock::now();
            }
            return BT::NodeStatus::RUNNING;
        }
        if (!validDistanceValue(detection.distance_m) && !have_filtered_) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "OrientToDetectionAtDist: waiting for a range.");
            return BT::NodeStatus::RUNNING;
        }
        const Observation seen{detection.bearing_horizontal,
                               validDistanceValue(detection.distance_m) ? detection.distance_m : filtered_.distance,
                               heading.value_or(filtered_.heading)};
        if (require_heading_ && !confirmed(seen)) {
            return BT::NodeStatus::RUNNING;
        }
        // Monocular range and box centers are noisy; smooth them, the heading
        // more, so the commanded point does not chase noise.
        if (!have_filtered_) {
            filtered_ = seen;
            have_filtered_ = true;
        } else {
            filtered_.bearing =
                normalizeAngle(filtered_.bearing + 0.25 * normalizeAngle(seen.bearing - filtered_.bearing));
            filtered_.distance += 0.25 * (seen.distance - filtered_.distance);
        }
        if (heading) {
            filtered_.heading =
                have_filtered_heading_
                    ? normalizeAngle(filtered_.heading + 0.2 * normalizeAngle(*heading - filtered_.heading))
                    : *heading;
            have_filtered_heading_ = true;
        }

        const bool old_enough = SteadyClock::now() - started_at_ >= std::chrono::milliseconds(min_msec_);
        const bool centered = std::fabs(filtered_.bearing) <= tolerance_;
        const bool square = !heading || std::fabs(filtered_.heading) <= radians(orient_tolerance_deg_);
        const bool distance_ok = std::fabs(filtered_.distance - desired_distance_) <= distance_tolerance_;
        if (old_enough && centered && square && distance_ok) {
            if (++settled_frames_ >= settle_frames_) {
                RCLCPP_INFO(logger_, "OrientToDetectionAtDist: aligned to %s.", filter_.describe().c_str());
                return BT::NodeStatus::SUCCESS;
            }
            // Already in the envelope: hold still while frames confirm it,
            // rather than chase sub-degree noise.
            return BT::NodeStatus::RUNNING;
        }
        settled_frames_ = 0;

        if (SteadyClock::now() - last_commanded_ < std::chrono::milliseconds(update_msec_)) {
            return BT::NodeStatus::RUNNING;
        }

        // The object's center, along the heading and to the left; right of
        // center (positive) is a negative left offset.
        double forward = filtered_.distance * std::cos(filtered_.bearing);
        double left = -filtered_.distance * std::sin(filtered_.bearing);
        if (heading) {
            // From the center, back desired_distance along the face's normal,
            // which the squaring turn points along.
            forward -= desired_distance_ * std::cos(filtered_.heading);
            left -= desired_distance_ * std::sin(filtered_.heading);
        } else {
            forward = filtered_.distance - desired_distance_;
            left = 0.0;
        }
        const double step = std::hypot(forward, left);
        if (step > max_position_step_) {
            forward *= max_position_step_ / step;
            left *= max_position_step_ / step;
        }
        // Keep looking at the center while moving onto the normal: there the
        // look is square to the face, and turning square first, far off the
        // normal, could lose the object from view.
        const double yaw = actualYaw(node_);
        std::array<double, 3> target = horizontalTarget(node_);
        addBodyOffset(target, forward, left, yaw);
        sendPosition(node_, *setpoint_publisher_, *clock_, target);
        sendYaw(node_, *setpoint_publisher_, *clock_, yawToward(node_, filtered_.bearing));
        last_commanded_ = SteadyClock::now();
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    SteadyClock::time_point started_at_;
    SteadyClock::time_point deadline_;
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point last_commanded_;
    Observation filtered_;
    Observation candidate_;
    double desired_distance_ = 2.0;
    double tolerance_ = 0.35;
    double distance_tolerance_ = 0.5;
    double orient_tolerance_deg_ = 5.0;
    double max_position_step_ = 0.5;
    int settle_frames_ = 1;
    int settled_frames_ = 0;
    int min_msec_ = 6500;
    int timeout_msec_ = 30000;
    int update_msec_ = 300;
    bool require_heading_ = false;
    bool have_filtered_ = false;
    bool have_filtered_heading_ = false;
    bool have_candidate_ = false;
};

// The filter of the down-camera nodes below, which have no camera port. False,
// with the reason logged, when a port's blackboard entry is missing or does not
// parse, rather than carry on with the default: an empty class_id takes any class.
bool readDownFilter(const BT::TreeNode &node, DetectionFilter &filter, const rclcpp::Logger &logger) {
    filter = DetectionFilter{};
    filter.camera = "down";
    return readPort(node, "task", filter.task, logger) && readPort(node, "class_id", filter.class_id, logger) &&
           readPort(node, "min_score", filter.min_score, logger);
}

// Moves forward, centering a down-camera detection when there is one.
// DownForwardSweepAlign searches instead: along a track forward from where it
// starts, zigzagging across it, until a detection, then stops over it.
class DownForwardAlignAction : public BT::StatefulActionNode {
   public:
    DownForwardAlignAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                           SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                           rclcpp::Logger logger, const bool sweep_when_missing = false)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger),
          sweep_when_missing_(sweep_when_missing) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("task", "", "Only accept detections from this task"),
                BT::InputPort<std::string>("class_id", "", "Only accept this class id (empty = any)"),
                BT::InputPort<double>("min_score", 0.0, "Minimum detection confidence"),
                BT::InputPort<double>("max_dist", 10.0, "Maximum forward travel before SUCCESS (the sweep: FAILURE)"),
                BT::InputPort<double>("forward_step", 1.0, "Forward target extension per update in meters"),
                BT::InputPort<double>("default_distance", 1.0, "Distance used when detection distance is unavailable"),
                BT::InputPort<double>("center_gain", 1.0, "Scale factor for down-camera x/y centering offsets"),
                BT::InputPort<double>("lateral_sweep_step", 1.0,
                                      "Right/left search step when no down-camera detection is visible"),
                BT::InputPort<int>("update_msec", 300, "Minimum time between movement corrections"),
                BT::InputPort<int>("timeout_msec", 30000, "Maximum align time before SUCCESS (the sweep: FAILURE)")};
    }

    BT::NodeStatus onStart() override {
        if (!readDownFilter(*this, filter_, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
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
        if (max_dist_ <= 0.0 || forward_step_ <= 0.0 || default_distance_ <= 0.0 || center_gain_ < 0.0 ||
            lateral_sweep_step_ <= 0.0 || update_msec_ <= 0 || timeout_msec_ <= 0) {
            RCLCPP_ERROR(logger_,
                         "DownForwardAlign max_dist, forward_step, default_distance, lateral_sweep_step, "
                         "update_msec and timeout_msec must be positive, and center_gain not negative.");
            return BT::NodeStatus::FAILURE;
        }

        initial_pos_ = actualPosition(node_);
        track_yaw_ = node_.attitudeTarget(2);
        track_progress_ = 0.0;
        sweep_direction_ = -1.0;  // right first
        last_processed_ = SteadyClock::now();
        last_commanded_ = last_processed_ - std::chrono::milliseconds(update_msec_);
        deadline_ = last_processed_ + std::chrono::milliseconds(timeout_msec_);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "DownForwardAlign failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        return tickActive();
    }

    void onHalted() override {
        stopHorizontal(node_);
        RCLCPP_INFO(logger_, "DownForwardAlign halted.");
    }

   private:
    BT::NodeStatus tickActive() {
        const std::array<double, 3> pos = actualPosition(node_);
        if (std::hypot(pos[0] - initial_pos_[0], pos[1] - initial_pos_[1]) >= max_dist_) {
            RCLCPP_INFO(logger_, "DownForwardAlign: max distance %.2fm reached.", max_dist_);
            return finish();
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "DownForwardAlign: timeout reached after %dms.", timeout_msec_);
            return finish();
        }

        const Match match = latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1));
        if (match.detection != nullptr && sweep_when_missing_) {
            // Stop over it, for the alignment that follows.
            stopHorizontal(node_);
            RCLCPP_INFO(logger_, "DownForwardSweepAlign: found %s.", filter_.describe().c_str());
            return BT::NodeStatus::SUCCESS;
        }

        if (SteadyClock::now() - last_commanded_ < std::chrono::milliseconds(update_msec_)) {
            return BT::NodeStatus::RUNNING;
        }
        std::array<double, 3> target = horizontalTarget(node_);
        if (sweep_when_missing_) {
            // The targets lie on a track fixed at the start: they advance
            // forward_step an update, whether or not the vehicle keeps up, and
            // stop max_dist along it.
            track_progress_ = std::min(track_progress_ + forward_step_, max_dist_);
            target = {initial_pos_[0], initial_pos_[1], UNSPECIFIED_PORT};
            addBodyOffset(target, track_progress_, sweep_direction_ * lateral_sweep_step_, track_yaw_);
            sweep_direction_ = -sweep_direction_;
        } else {
            addBodyOffset(target, forward_step_, 0.0, node_.attitudeTarget(2));
            // Center on the newest frame since the last correction, consumed
            // only here so none is dropped between corrections. Its offset is
            // along the heading the camera saw it from, not the commanded one.
            if (match.detection != nullptr) {
                last_processed_ = match.received_at;
                const double distance =
                    validDistanceValue(match.detection->distance_m) ? match.detection->distance_m : default_distance_;
                const auto offset = downOffset(*match.detection, distance);
                addBodyOffset(target, center_gain_ * offset[0], center_gain_ * offset[1], actualYaw(node_));
            }
        }
        sendPosition(node_, *setpoint_publisher_, *clock_, target);
        last_commanded_ = SteadyClock::now();
        return BT::NodeStatus::RUNNING;
    }

    // Out of distance or time. The targets run ahead of the vehicle, the
    // sweep's up to max_dist from the start, so it stops where it is rather
    // than carry on to the last one.
    BT::NodeStatus finish() {
        stopHorizontal(node_);
        return sweep_when_missing_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    std::array<double, 3> initial_pos_ = {};
    SteadyClock::time_point deadline_;
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point last_commanded_;
    double max_dist_ = 10.0;
    double forward_step_ = 1.0;
    double default_distance_ = 1.0;
    double center_gain_ = 1.0;
    double lateral_sweep_step_ = 1.0;
    double sweep_direction_ = -1.0;  // the next search step: 1 left, -1 right
    double track_yaw_ = 0.0;
    double track_progress_ = 0.0;
    int update_msec_ = 300;
    int timeout_msec_ = 30000;
    bool sweep_when_missing_ = false;
};

// Searches sideways over the down camera: lateral_step right of where it
// starts, then left of it, and back, num_sweeps times, sampling at each side.
class DownSweepAlignAction : public BT::StatefulActionNode {
   public:
    DownSweepAlignAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                         SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                         rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
          clock_(clock),
          logger_(logger) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("task", "", "Only accept down-camera detections from this task"),
                BT::InputPort<std::string>("class_id", "", "Only accept this class id (empty = any)"),
                BT::InputPort<double>("min_score", 0.0, "Minimum detection confidence"),
                BT::InputPort<double>("lateral_step", 1.0, "Distance to each side of the start (m)"),
                BT::InputPort<int>("num_sweeps", 3, "Right-then-left passes before FAILURE"),
                BT::InputPort<int>("sample_timeout_msec", 1000, "Detection sample time at each point"),
                BT::InputPort<int>("move_timeout_msec", 20000, "Maximum wait for each sideways move")};
    }

    BT::NodeStatus onStart() override {
        if (!readDownFilter(*this, filter_, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
        getInput("lateral_step", lateral_step_);
        getInput("num_sweeps", num_sweeps_);
        getInput("sample_timeout_msec", sample_timeout_msec_);
        getInput("move_timeout_msec", move_timeout_msec_);
        if (filter_.task.empty() || lateral_step_ <= 0.0 || num_sweeps_ <= 0 || sample_timeout_msec_ <= 0 ||
            move_timeout_msec_ <= 0) {
            RCLCPP_ERROR(logger_, "DownSweepAlign needs a task and positive distances and timings.");
            return BT::NodeStatus::FAILURE;
        }

        start_ = horizontalTarget(node_);
        start_yaw_ = actualYaw(node_);
        completed_sweeps_ = 0;
        right_next_ = true;
        moving_ = false;
        last_processed_ = SteadyClock::now();
        deadline_ = last_processed_ + std::chrono::milliseconds(sample_timeout_msec_);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (node_.killed) {
            RCLCPP_WARN(logger_, "DownSweepAlign failed because kill switch is engaged.");
            return BT::NodeStatus::FAILURE;
        }
        if (latestMatch(node_, filter_, last_processed_ + std::chrono::nanoseconds(1)).detection != nullptr) {
            if (moving_) {
                // Stop where it came into view, not at the side it was moving to.
                stopHorizontal(node_);
            }
            RCLCPP_INFO(logger_, "DownSweepAlign: found %s.", filter_.describe().c_str());
            return BT::NodeStatus::SUCCESS;
        }

        if (!moving_) {
            if (SteadyClock::now() < deadline_) {
                return BT::NodeStatus::RUNNING;
            }
            if (completed_sweeps_ >= num_sweeps_) {
                RCLCPP_WARN(logger_, "DownSweepAlign: %s not found after %d sweeps.", filter_.describe().c_str(),
                            num_sweeps_);
                return BT::NodeStatus::FAILURE;
            }
            beginMove();
            return BT::NodeStatus::RUNNING;
        }

        if (SteadyClock::now() >= deadline_) {
            RCLCPP_WARN(logger_, "DownSweepAlign: move timed out.");
            return BT::NodeStatus::FAILURE;
        }
        if (freshAndWithinTolerance(0) && freshAndWithinTolerance(1)) {
            if (!right_next_) {
                ++completed_sweeps_;
            }
            right_next_ = !right_next_;
            moving_ = false;
            deadline_ = SteadyClock::now() + std::chrono::milliseconds(sample_timeout_msec_);
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { RCLCPP_INFO(logger_, "DownSweepAlign halted."); }

   private:
    void beginMove() {
        std::array<double, 3> target = start_;
        addBodyOffset(target, 0.0, right_next_ ? -lateral_step_ : lateral_step_, start_yaw_);
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        moving_ = true;
        sendPosition(node_, *setpoint_publisher_, *clock_, target);
        RCLCPP_INFO(logger_, "DownSweepAlign: %.2fm %s of the start.", lateral_step_, right_next_ ? "right" : "left");
    }

    bool freshAndWithinTolerance(const std::size_t index) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= POSITION_TOLERANCE;
    }

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
    DetectionFilter filter_;
    std::array<double, 3> start_ = {};
    std::array<std::uint64_t, 12> start_updates_ = {};
    SteadyClock::time_point last_processed_;
    SteadyClock::time_point deadline_;
    double start_yaw_ = 0.0;
    double lateral_step_ = 1.0;
    int num_sweeps_ = 3;
    int completed_sweeps_ = 0;
    int sample_timeout_msec_ = 1000;
    int move_timeout_msec_ = 20000;
    bool right_next_ = true;
    bool moving_ = false;
};

class DownAlignToDetectionAction : public BT::StatefulActionNode {
   public:
    DownAlignToDetectionAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                               SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                               rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
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
                BT::InputPort<bool>("require_orientation", false,
                                    "Only align with the orientation, not on the center alone"),
                BT::InputPort<int>("orientation_settle_frames", 3, "Consistent orientations in a row before yawing"),
                BT::InputPort<double>("orientation_consistency_deg", 10.0,
                                      "Largest frame-to-frame orientation change taken as consistent"),
                BT::InputPort<double>("orient_tolerance_deg", 5.0, "Orientation error allowed at completion"),
                BT::InputPort<int>("min_msec", 3000, "Minimum closed-loop alignment time"),
                BT::InputPort<int>("timeout_msec", 30000, "Maximum align time before FAILURE"),
                BT::InputPort<int>("update_msec", 200, "Minimum time between movement corrections")};
    }

    BT::NodeStatus onStart() override {
        if (!readDownFilter(*this, filter_, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
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
        // A desired_distance at or below zero would hold the vehicle descending.
        if (tolerance_ <= 0.0 || (std::isfinite(desired_distance_) && desired_distance_ <= 0.0) ||
            default_distance_ <= 0.0 || center_gain_ <= 0.0 || update_msec_ <= 0 || orientation_settle_frames_ <= 0 ||
            orientation_consistency_deg_ <= 0.0) {
            RCLCPP_ERROR(logger_,
                         "DownAlignToDetection tolerance, desired_distance (when set), default_distance, "
                         "center_gain, update_msec and the orientation filter's settings must be positive.");
            return BT::NodeStatus::FAILURE;
        }

        orientation_.reset();
        orientation_frames_ = 0;
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
    // The orientation to yaw by, once orientation_settle_frames consistent
    // ones agree. A path marker is a line, so its orientation is axial: kept
    // within (-90, 90] degrees and compared modulo 180.
    std::optional<double> filterOrientation(const std::optional<double> &orientation) {
        if (!orientation) {
            orientation_.reset();
            orientation_frames_ = 0;
            return std::nullopt;
        }
        const double jump = orientation_ ? std::remainder(*orientation - *orientation_, M_PI) : 0.0;
        if (!orientation_ || std::fabs(jump) > radians(orientation_consistency_deg_)) {
            orientation_ = std::remainder(*orientation, M_PI);
            orientation_frames_ = 1;
        } else {
            orientation_ = std::remainder(*orientation_ + 0.45 * jump, M_PI);
            ++orientation_frames_;
        }
        if (orientation_frames_ < orientation_settle_frames_) {
            return std::nullopt;
        }
        return orientation_;
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
        const auto orient_error = orient_ ? filterOrientation(orientationYaw(detection)) : std::nullopt;
        const bool hold_distance = std::isfinite(desired_distance_);
        const bool centered = std::fabs(detection.bearing_horizontal) <= tolerance_ &&
                              std::fabs(detection.bearing_vertical) <= tolerance_;
        const bool distance_ok = !hold_distance || std::fabs(distance - desired_distance_) <= distance_tolerance_;
        const bool orient_ok = !orient_ ||
                               (orient_error && std::fabs(*orient_error) <= radians(orient_tolerance_deg_)) ||
                               (!orient_error && !require_orientation_);
        const bool old_enough = SteadyClock::now() - started_at_ >= std::chrono::milliseconds(min_msec_);
        if (old_enough && centered && distance_ok && orient_ok) {
            RCLCPP_INFO(logger_, "DownAlignToDetection: aligned to %s.", filter_.describe().c_str());
            return BT::NodeStatus::SUCCESS;
        }

        if (SteadyClock::now() - last_commanded_ < std::chrono::milliseconds(update_msec_)) {
            return BT::NodeStatus::RUNNING;
        }

        const double yaw = actualYaw(node_);
        const auto offset = downOffset(detection, distance);
        std::array<double, 3> target = horizontalTarget(node_);
        addBodyOffset(target, center_gain_ * offset[0], center_gain_ * offset[1], yaw);
        if (hold_distance) {
            // Farther above it than desired: descend, to a smaller z.
            target[2] = actualPosition(node_)[2] - (distance - desired_distance_);
        }
        sendPosition(node_, *setpoint_publisher_, *clock_, target);

        if (orient_error) {
            sendYaw(node_, *setpoint_publisher_, *clock_, yaw + *orient_error);
        }

        last_commanded_ = SteadyClock::now();
        return BT::NodeStatus::RUNNING;
    }

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
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
    std::optional<double> orientation_;
    int orientation_settle_frames_ = 3;
    int orientation_frames_ = 0;
    int min_msec_ = 3000;
    int timeout_msec_ = 30000;
    int update_msec_ = 200;
    bool orient_ = true;
    bool require_orientation_ = false;
};

class DownPatternScanAction : public BT::StatefulActionNode {
   public:
    DownPatternScanAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                          SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock,
                          rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          setpoint_publisher_(setpoint_publisher),
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
        if (!readDownFilter(*this, filter_, logger_)) {
            return BT::NodeStatus::FAILURE;
        }
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
            RCLCPP_WARN(logger_, "DownPatternScan failed because kill switch is engaged.");
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

    // The points sampled, (forward, left) from where the scan started in
    // movement_dist: there, back right, left, ahead, right, back left.
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
            returning_home_ = true;
            beginMove(initial_pos_);
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
                RCLCPP_WARN(logger_, "DownPatternScan: %s not found; back at the start.", filter_.describe().c_str());
                return BT::NodeStatus::FAILURE;
            }
            phase_ = Phase::SAMPLE;
            last_processed_ = SteadyClock::now();
            deadline_ = last_processed_ + std::chrono::milliseconds(sample_timeout_msec_);
        }
        return BT::NodeStatus::RUNNING;
    }

    std::array<double, 3> patternTarget(const std::size_t index) const {
        std::array<double, 3> target{initial_pos_[0], initial_pos_[1], UNSPECIFIED_PORT};
        addBodyOffset(target, PATTERN[index][0] * movement_dist_, PATTERN[index][1] * movement_dist_, initial_yaw_);
        return target;
    }

    void beginMove(const std::array<double, 3> &target) {
        start_updates_ = node_.control_error_updates;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(move_timeout_msec_);
        phase_ = Phase::MOVE;
        // Horizontal only: z keeps whatever it holds.
        sendPosition(node_, *setpoint_publisher_, *clock_, {target[0], target[1], UNSPECIFIED_PORT});
        if (returning_home_) {
            RCLCPP_INFO(logger_, "DownPatternScan: returning to the start.");
        } else {
            RCLCPP_INFO(logger_, "DownPatternScan: moving to pattern point %zu.", pattern_index_);
        }
    }

    bool freshAndWithinTolerance(const std::size_t index, const double tolerance) const {
        return node_.control_error_updates[index] > start_updates_[index] &&
               std::fabs(node_.control_errors[index]) <= tolerance;
    }

    MissionNode &node_;
    SetpointPublisher::SharedPtr setpoint_publisher_;
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
                         SetpointPublisher::SharedPtr setpoint_publisher, rclcpp::Clock::SharedPtr clock) {
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

    factory.registerBuilder<SelectGateRoleAction>(
        "SelectGateRole", [&node, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<SelectGateRoleAction>(name, config, node, logger);
        });

    factory.registerBuilder<SearchForDetectionAction>(
        "SearchForDetection",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<SearchForDetectionAction>(name, config, node, setpoint_publisher, clock, logger);
        });

    factory.registerBuilder<AlignToDetectionAction>(
        "AlignToDetection",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<AlignToDetectionAction>(name, config, node, setpoint_publisher, clock, logger);
        });

    factory.registerBuilder<LateralAlignToDetectionAction>(
        "LateralAlignToDetection",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<LateralAlignToDetectionAction>(name, config, node, setpoint_publisher, clock,
                                                                   logger);
        });

    factory.registerBuilder<ForwardSweepAlignAction>(
        "ForwardSweepAlign",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<ForwardSweepAlignAction>(name, config, node, setpoint_publisher, clock, logger);
        });

    factory.registerBuilder<ForwardContinuousAlignAction>(
        "ForwardContinuousAlign",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<ForwardContinuousAlignAction>(name, config, node, setpoint_publisher, clock,
                                                                  logger);
        });

    factory.registerBuilder<ForwardFixedTransitAction>(
        "ForwardFixedTransit",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<ForwardFixedTransitAction>(name, config, node, setpoint_publisher, clock, logger);
        });

    factory.registerBuilder<OrientToDetectionAtDistAction>(
        "OrientToDetectionAtDist",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<OrientToDetectionAtDistAction>(name, config, node, setpoint_publisher, clock,
                                                                   logger);
        });

    auto down_forward_align_builder = [&node, setpoint_publisher, clock, logger](const std::string &name,
                                                                                 const BT::NodeConfig &config) {
        return std::make_unique<DownForwardAlignAction>(name, config, node, setpoint_publisher, clock, logger);
    };
    factory.registerBuilder<DownForwardAlignAction>("DownForwardAlign", down_forward_align_builder);
    factory.registerBuilder<DownForwardAlignAction>("DownForwardSweepAlign", [&node, setpoint_publisher, clock, logger](
                                                                                 const std::string &name,
                                                                                 const BT::NodeConfig &config) {
        return std::make_unique<DownForwardAlignAction>(name, config, node, setpoint_publisher, clock, logger, true);
    });

    factory.registerBuilder<DownSweepAlignAction>(
        "DownSweepAlign",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<DownSweepAlignAction>(name, config, node, setpoint_publisher, clock, logger);
        });

    factory.registerBuilder<DownAlignToDetectionAction>(
        "DownAlignToDetection",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<DownAlignToDetectionAction>(name, config, node, setpoint_publisher, clock, logger);
        });
    factory.registerBuilder<DownAlignToDetectionAction>(
        "DownContinuousAlign",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<DownAlignToDetectionAction>(name, config, node, setpoint_publisher, clock, logger);
        });
    factory.registerBuilder<DownPatternScanAction>(
        "DownPatternScan",
        [&node, setpoint_publisher, clock, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<DownPatternScanAction>(name, config, node, setpoint_publisher, clock, logger);
        });

    factory.registerBuilder<SweepCheckAction>("SweepCheck", [&node, setpoint_publisher, clock, logger](
                                                                const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<SweepCheckAction>(name, config, node, setpoint_publisher, clock, logger);
    });

    factory.registerBuilder<SweepAngleAction>("SweepAngle", [&node, setpoint_publisher, clock, logger](
                                                                const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<SweepAngleAction>(name, config, node, setpoint_publisher, clock, logger);
    });
}
