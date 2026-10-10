#include "sub_mission/vision_client.hpp"

#include <utility>

namespace {

// Topic/service names are relative, so they resolve inside the node's
// namespace (e.g. /marlin_v3/...) — the same convention sub_vision uses.
struct CameraSpec {
    const char *name;
    const char *detections_topic;
    const char *load_model_service;
};

constexpr CameraSpec kCameras[] = {
    {"front", "vision/detections", "sub_vision/load_model"},
    {"down", "vision/detections_down", "sub_vision_down/load_model"},
};

}  // namespace

VisionClient::VisionClient(rclcpp::Node &node) {
    for (const CameraSpec &spec : kCameras) {
        Source &source = sources_[spec.name];

        // Through a pointer, as map nodes do not move: sources_[camera] would
        // count as modifying the map, which hasCamera and loadModelClient
        // read without the lock.
        source.subscription = node.create_subscription<sub_vision_interfaces::msg::DetectionArray>(
            spec.detections_topic, rclcpp::SensorDataQoS(),
            [this, snapshot = &source.snapshot](sub_vision_interfaces::msg::DetectionArray::ConstSharedPtr msg) {
                const std::lock_guard<std::mutex> lock(mutex_);
                snapshot->detections = std::move(msg);
                snapshot->received_at = std::chrono::steady_clock::now();
            });

        source.load_model_client = node.create_client<sub_vision_interfaces::srv::LoadModel>(spec.load_model_service);
    }
}

bool VisionClient::hasCamera(const std::string &camera) const { return sources_.find(camera) != sources_.end(); }

VisionClient::Snapshot VisionClient::latest(const std::string &camera) const {
    const std::lock_guard<std::mutex> lock(mutex_);
    const auto it = sources_.find(camera);
    return it != sources_.end() ? it->second.snapshot : Snapshot{};
}

rclcpp::Client<sub_vision_interfaces::srv::LoadModel>::SharedPtr VisionClient::loadModelClient(
    const std::string &camera) const {
    const auto it = sources_.find(camera);
    return it != sources_.end() ? it->second.load_model_client : nullptr;
}

const sub_vision_interfaces::msg::Detection *VisionClient::bestMatch(
    const sub_vision_interfaces::msg::DetectionArray &array, const std::string &task, const std::string &class_id,
    const double min_score, const std::function<bool(const sub_vision_interfaces::msg::Detection &)> &accept) {
    if (!task.empty() && array.task != task) {
        return nullptr;
    }

    const sub_vision_interfaces::msg::Detection *best = nullptr;
    double best_score = -1.0;
    for (const auto &detection : array.detections) {
        if (detection.detection.results.empty()) {
            continue;
        }
        const auto &hypothesis = detection.detection.results[0].hypothesis;
        if (!class_id.empty() && hypothesis.class_id != class_id) {
            continue;
        }
        if (hypothesis.score < min_score) {
            continue;
        }
        if (accept && !accept(detection)) {
            continue;
        }
        if (hypothesis.score > best_score) {
            best_score = hypothesis.score;
            best = &detection;
        }
    }
    return best;
}
