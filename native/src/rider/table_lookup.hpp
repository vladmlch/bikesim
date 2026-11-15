// rider/table_lookup.hpp — shared piecewise-linear table interpolation and
// knot-table validation for launch and seated-climb effort caps.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace rider::detail {

// Piecewise-linear interpolation over strictly ascending knots, clamped at
// the endpoint values. Callers guarantee !knots.empty() and equal sizes; a
// non-positive/NaN comparison !(x > knots.front()) returns values.front()
// without invoking upper_bound on NaN.
[[nodiscard]] inline double piecewise_linear_knot(
    std::span<const double> knots,
    std::span<const double> values,
    double x) noexcept {
    if (x >= knots.back()) return values.back();
    if (!(x > knots.front())) return values.front();
    const auto upper = static_cast<std::size_t>(
        std::distance(knots.begin(), std::ranges::upper_bound(knots, x)));
    const double t = (x - knots[upper - 1]) / (knots[upper] - knots[upper - 1]);
    return values[upper - 1] + t * (values[upper] - values[upper - 1]);
}

inline void validate_strictly_ascending(
    std::span<const double> values,
    std::string_view section,
    std::string_view field) {
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (!std::isfinite(values[i])) {
            throw std::invalid_argument(
                std::string(section) + "." + std::string(field) +
                ": values must be finite");
        }
        if (i > 0 && !(values[i - 1] < values[i])) {
            throw std::invalid_argument(
                std::string(section) + "." + std::string(field) +
                ": values must ascend strictly");
        }
    }
}

inline void validate_grade_cap_table(
    std::span<const double> grades,
    std::span<const double> caps_nm,
    std::string_view section,
    std::string_view grades_field,
    std::string_view caps_field) {
    if (grades.empty()) {
        throw std::invalid_argument(
            std::string(section) + "." + std::string(grades_field) +
            ": needs at least one knot");
    }
    if (caps_nm.size() != grades.size()) {
        throw std::invalid_argument(
            std::string(section) + "." + std::string(caps_field) +
            ": needs one cap per " + std::string(grades_field) + " knot");
    }
    validate_strictly_ascending(grades, section, grades_field);
    for (const double cap : caps_nm) {
        if (!(std::isfinite(cap) && cap > 0.)) {
            throw std::invalid_argument(
                std::string(section) + "." + std::string(caps_field) +
                ": must be finite and positive");
        }
    }
}

} // namespace rider::detail
