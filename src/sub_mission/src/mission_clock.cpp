#include "sub_mission/mission_clock.hpp"

#include <atomic>
#include <memory>
#include <utility>

namespace {

// The simulation's clock, or null on the steady clock. Read from the
// subscriptions' threads, so through an atomic pointer; `owner` keeps it alive.
std::shared_ptr<rclcpp::Clock> owner;
std::atomic<rclcpp::Clock *> simulation_clock{nullptr};

}  // namespace

void MissionClock::follow(rclcpp::Clock::SharedPtr clock, const bool sim_time) {
    simulation_clock.store(nullptr);
    owner = sim_time ? std::move(clock) : nullptr;
    simulation_clock.store(owner.get());
}

MissionClock::time_point MissionClock::now() {
    if (rclcpp::Clock *clock = simulation_clock.load()) {
        return time_point(duration(clock->now().nanoseconds()));
    }
    return time_point(std::chrono::duration_cast<duration>(std::chrono::steady_clock::now().time_since_epoch()));
}
