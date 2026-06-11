#ifndef SUB_SIM_SENSORS_DEPTH_CONVERSION_HPP
#define SUB_SIM_SENSORS_DEPTH_CONVERSION_HPP

#include <cmath>
#include <cstdint>

// Convert a single Stonefish depth sample (meters, 32FC1) into the OAK-D Pro's
// stereo-depth convention (millimeters, 16UC1). Invalid samples (NaN/inf, <= 0,
// or beyond the 16-bit range) map to 0.
inline uint16_t depth_meters_to_mm(float meters) {
    if (!std::isfinite(meters) || meters <= 0.0f) {
        return 0;
    }
    const float mm = meters * 1000.0f;
    if (mm >= 65535.0f) {
        return 0;
    }
    return static_cast<uint16_t>(mm);
}

#endif  // SUB_SIM_SENSORS_DEPTH_CONVERSION_HPP
