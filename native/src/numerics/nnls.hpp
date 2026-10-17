#pragma once
#include "../rider/allocation.hpp"

namespace bike::numerics {
struct NnlsResult {
    std::vector<double> x;
    double residual_norm=0.;
    std::size_t iterations=0;
};
// Lawson-Hanson active-set NNLS with an owned Jacobi SVD subproblem.
// max_iterations=0 selects the source's 3*n iteration budget.
[[nodiscard]] NnlsResult nnls(const bike::rider::Matrix&, std::span<const double>,
                              std::size_t max_iterations=0);
} // namespace bike::numerics
