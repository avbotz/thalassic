#pragma once

#include <chrono>

#include "rclcpp/clock.hpp"

// The clock the mission measures its waits on: how long a move may take, how
// long a velocity is held, how old a detection is. On the vehicle that is the
// steady clock. In simulation (use_sim_time) it is the simulator's time,
// because the vehicle only moves as that time passes: a simulation that runs
// slower than real time, or is paused and stepped, then does not use up the
// mission's budgets, and a run does not depend on how fast the computer is.
//
// Waits on what is outside the simulation (a process exiting, Groot) stay on
// std::chrono::steady_clock.
class MissionClock {
   public:
    using duration = std::chrono::nanoseconds;
    using rep = duration::rep;
    using period = duration::period;
    using time_point = std::chrono::time_point<MissionClock>;
    static constexpr bool is_steady = false;

    // Follows `clock`'s time when the node runs on simulation time
    // (`sim_time`), and the steady clock otherwise. Called once, by the
    // mission node, before anything reads the time.
    static void follow(rclcpp::Clock::SharedPtr clock, bool sim_time);

    static time_point now();
};
