#pragma once

#include <cstdint>
#include <functional>
#include <span>
#include <vector>

namespace bike::numerics {

// NOLINTNEXTLINE(performance-enum-size) enum width matches the reference ABI layout
enum class DifferenceScheme : std::uint8_t { two_point, three_point };
struct LeastSquaresOptions {
    int max_evaluations = 80;
    DifferenceScheme difference = DifferenceScheme::two_point;
    bool jacobian_scaling = true;
    // Zero selects sqrt(eps) / cbrt(eps), like approx_derivative. A
    // positive value is relative to x, with its zero-component fallback.
    double relative_difference_step = 0.;
    double ftol = 1e-8;
    double xtol = 1e-8;
    double gtol = 1e-8;
    // Called after each accepted/rejected outer iteration; true requests
    // termination. Evaluations made to obtain J are not max_nfev ticks.
    std::function<bool(std::span<const double>)> stop;
};
struct LeastSquaresResult {
    std::vector<double> x;
    double cost = 0.;
    int evaluations = 0;
    bool converged = false;
    int jacobian_evaluations = 0;
    std::int64_t residual_calls = 0;
    // scipy-compatible 0 budget, 1 gradient, 2 cost, 3 step, 4 both,
    // -2 callback. Callback stop is not an optimizer convergence claim.
    int status = 0;
    double optimality = 0.;
};
using Residual = std::function<std::vector<double>(std::span<const double>)>;
[[nodiscard]] LeastSquaresResult bounded_least_squares(
    const Residual&, std::span<const double> initial,
    std::span<const double> lower, std::span<const double> upper,
    const LeastSquaresOptions&);
[[nodiscard]] LeastSquaresResult bounded_least_squares(
    const Residual&, std::span<const double> initial,
    std::span<const double> lower, std::span<const double> upper,
    int max_evaluations);

} // namespace bike::numerics
