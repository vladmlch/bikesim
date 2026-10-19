#pragma once
#include "../rider/allocation.hpp"
#include <cstdint>
#include <string>

namespace bike::numerics {
// NOLINTNEXTLINE(performance-enum-size) enum width matches the reference ABI layout
enum class FeasibilityStatus : std::uint8_t { feasible, proven_infeasible, unknown };
struct FeasibilityResult {
    FeasibilityStatus status = FeasibilityStatus::unknown;
    std::vector<double> witness;
    std::string reason;
};
struct FeasibilityOptions {
    double inequality_slack=1e-7;
    double primal_tolerance=1e-9;
    std::size_t max_iterations=500;
};
[[nodiscard]] FeasibilityResult linear_feasibility(
    const bike::rider::Matrix& inequality, std::span<const double> bound,
    std::span<const double> lower, std::span<const double> upper,
    const bike::rider::Matrix& equality={}, std::span<const double> rhs={},
    const FeasibilityOptions& options={});
[[nodiscard]] bool linear_region_infeasible(
    const bike::rider::Matrix& inequality, std::span<const double> bound,
    std::span<const double> lower, std::span<const double> upper,
    const bike::rider::Matrix& equality={}, std::span<const double> rhs={});
} // namespace bike::numerics
