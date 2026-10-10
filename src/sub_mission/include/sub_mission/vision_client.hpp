#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sub_vision_interfaces/msg/detection.hpp"
#include "sub_vision_interfaces/msg/detection_array.hpp"
#include "sub_vision_interfaces/srv/load_model.hpp"

// Mission-side handle to the sub_vision perception nodes.
//
// One entry per camera ("front", "down"): the latest DetectionArray snapshot
// (with its steady-clock receive time, so freshness checks work identically
// under sim and wall time) and a client for that camera's load_model service.
// Cameras whose vision node is not running are harmless: the subscription
// stays idle and load_model reports the service as unavailable.
class VisionClient {
   public:
    struct Snapshot {
        sub_vision_interfaces::msg::DetectionArray::ConstSharedPtr detections;
        std::chrono::steady_clock::time_point received_at{};
    };

    explicit VisionClient(rclcpp::Node &node);

    // True for "front" and "down".
    bool hasCamera(const std::string &camera) const;

    // Latest detections for a camera; .detections is null until the first
    // message arrives. Unknown camera returns an empty snapshot.
    Snapshot latest(const std::string &camera) const;

    // load_model service client for a camera; null for unknown cameras.
    rclcpp::Client<sub_vision_interfaces::srv::LoadModel>::SharedPtr loadModelClient(const std::string &camera) const;

    // Best detection in an array for a filter: highest score with
    // array.task == task (when task is non-empty), class_id == class_id (when
    // non-empty), score >= min_score, and accept(detection) (when given).
    // Null when nothing matches.
    static const sub_vision_interfaces::msg::Detection *bestMatch(
        const sub_vision_interfaces::msg::DetectionArray &array, const std::string &task, const std::string &class_id,
        double min_score, const std::function<bool(const sub_vision_interfaces::msg::Detection &)> &accept = {});

   private:
    struct Source {
        rclcpp::Subscription<sub_vision_interfaces::msg::DetectionArray>::SharedPtr subscription;
        rclcpp::Client<sub_vision_interfaces::srv::LoadModel>::SharedPtr load_model_client;
        Snapshot snapshot;
    };

    // Guards each Source's snapshot, which the subscription callbacks write
    // from the executor thread; the rest is fixed after construction.
    mutable std::mutex mutex_;
    std::map<std::string, Source> sources_;
};
