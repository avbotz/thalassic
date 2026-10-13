#include "sub_control/trajectory.hpp"

#include <algorithm>
#include <cmath>

double sqrt_controller(double error, double p, double accel, double dt) {
    double correction;
    if (accel <= 0.0) {
        correction = error * p;
    } else if (p <= 0.0) {
        correction = std::copysign(std::sqrt(2.0 * accel * std::fabs(error)), error);
    } else {
        const double linear_distance = accel / (p * p);
        if (std::fabs(error) > linear_distance) {
            correction = std::copysign(std::sqrt(2.0 * accel * (std::fabs(error) - linear_distance / 2.0)), error);
        } else {
            correction = error * p;
        }
    }
    if (dt > 0.0) {
        // Never more than closes the error this step, so it cannot chatter about zero.
        return std::clamp(correction, -std::fabs(error) / dt, std::fabs(error) / dt);
    }
    return correction;
}
