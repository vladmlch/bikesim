#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

namespace bike::io { struct Value; }

namespace bike::rider {

// migration/interface-contracts.md: owned row-major matrices, never views
// into a callback or another controller's mutable scratch.
struct Matrix {
    std::size_t rows = 0, cols = 0;
    std::vector<double> values;
};
struct ExtraConstraint {
    std::function<std::vector<double>(std::span<const double>)> evaluate;
    std::function<Matrix(std::span<const double>)> jacobian;
    std::vector<double> lower, upper;
    // Linear constraints retain their matrix for the conservative LP
    // prefilter. No callback inspection or executable fixture expression.
    std::optional<Matrix> linear;
};
struct AllocationProblem {
    std::vector<double> target;
    Matrix equality, inequality;
    std::vector<double> equality_rhs, inequality_rhs, lower, upper;
    std::vector<ExtraConstraint> extra;
    std::optional<std::vector<double>> initial;
};
struct Allocation {
    std::vector<double> solution;
    bool feasible = false;
    double violation = std::numeric_limits<double>::infinity();
};

// New dense SQP arithmetic is deliberately identified separately from
// SciPy's historical SLSQP/LAPACK identity. Captured solver fixtures must
// qualify this path before any numeric/bitwise migration row is closed.
inline constexpr std::string_view allocation_numeric_identity =
    "bike-dense-sqp-bfgs-v1;ftol=1e-12;maxiter=500;restarts=4";

void validate(const AllocationProblem&);
[[nodiscard]] double allocation_residual(const AllocationProblem&,
                                         std::span<const double> x);
[[nodiscard]] Allocation allocate(const AllocationProblem&);
[[nodiscard]] AllocationProblem allocation_problem_from_json(const bike::io::Value&);
[[nodiscard]] std::pair<Matrix, std::vector<double>> inverse_dynamics_rows(
    const Matrix& mass, const Matrix& actuation, const Matrix& support_jacobian,
    std::span<const double> bias, std::span<const double> known_qforce);
[[nodiscard]] ExtraConstraint linear_constraint(
    Matrix matrix, std::vector<double> lower, std::vector<double> upper);
[[nodiscard]] std::vector<ExtraConstraint> grip_constraints(
    const Matrix& force_map, const std::array<double, 2>& pull_direction,
    bool pulling, double limit_n = 300.);

} // namespace bike::rider
