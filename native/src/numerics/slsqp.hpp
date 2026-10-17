#pragma once

#include "../rider/allocation.hpp"

namespace bike::numerics {
struct SlsqpOptions {
    double ftol = 1e-12;
    std::size_t max_iterations = 500;
    double finite_difference_step = 1.4901161193847656e-8;
};
struct SlsqpResult {
    std::vector<double> x;
    std::size_t iterations = 0;
    bool converged = false;
    // 0 optimum, 8 no accepted line-search step, 9 iteration limit,
    // 6 singular/inconsistent local QP. Feasibility is never this flag.
    int status = 9;
};
[[nodiscard]] SlsqpResult slsqp(const bike::rider::AllocationProblem&,
                              std::span<const double> initial,
                              const SlsqpOptions& = {});
} // namespace bike::numerics
