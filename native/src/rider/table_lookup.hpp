// rider/table_lookup.hpp — shared non-allocating piecewise-linear table lookup
// for launch and seated-climb effort caps.
#pragma once

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <span>

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

} // namespace rider::detail
