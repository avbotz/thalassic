#pragma once

// Validators for the *_parameters.yaml files that rsl does not have. Its gt<>
// lets inf through, and its bounds let NaN through (every comparison with it is
// false).

#include <cmath>
#include <format>
#include <string>

#include <rclcpp/parameter.hpp>
#include <tl/expected.hpp>

namespace sub_serial_drivers {

// The double parameter is finite: not NaN or inf.
inline tl::expected<void, std::string> finite(const rclcpp::Parameter& parameter) {
    if (std::isfinite(parameter.as_double())) {
        return {};
    }
    return tl::unexpected(
        std::format("Parameter '{}' must be finite, not '{}'", parameter.get_name(), parameter.as_double()));
}

}  // namespace sub_serial_drivers
