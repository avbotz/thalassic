#ifndef SUB_CONTROL_SUB_CONTROL_VALIDATORS_HPP_
#define SUB_CONTROL_SUB_CONTROL_VALIDATORS_HPP_

// Validators for sub_control_parameters.yaml that rsl does not have. Its
// bounds compare with < and >, which NaN passes (every comparison with it is
// false), and a one-sided bound lets inf through.

#include <rclcpp/parameter.hpp>
#include <tl/expected.hpp>

#include <cmath>
#include <format>
#include <string>

namespace sub_control {

// The double parameter is finite: not NaN or inf.
inline tl::expected<void, std::string> finite(const rclcpp::Parameter& parameter) {
    if (std::isfinite(parameter.as_double())) {
        return {};
    }
    return tl::unexpected(
        std::format("Parameter '{}' must be finite, not '{}'", parameter.get_name(), parameter.as_double()));
}

// Every element of the double array parameter is finite.
inline tl::expected<void, std::string> finite_elements(const rclcpp::Parameter& parameter) {
    for (const double value : parameter.as_double_array()) {
        if (!std::isfinite(value)) {
            return tl::unexpected(
                std::format("Value '{}' in parameter '{}' must be finite", value, parameter.get_name()));
        }
    }
    return {};
}

// Every element of the double array parameter is greater than `lower` (rsl's
// lower_element_bounds allows equal).
inline tl::expected<void, std::string> gt_elements(const rclcpp::Parameter& parameter, double lower) {
    for (const double value : parameter.as_double_array()) {
        if (!(value > lower)) {
            return tl::unexpected(std::format("Value '{}' in parameter '{}' must be greater than '{}'", value,
                                              parameter.get_name(), lower));
        }
    }
    return {};
}

}  // namespace sub_control

#endif  // SUB_CONTROL_SUB_CONTROL_VALIDATORS_HPP_
