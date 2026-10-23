#pragma once

#include "meijaard_constraints.hpp"
#include "meijaard_reference.hpp"

#include <complex>
#include <span>
#include <vector>

namespace spatial_tests {

// det(lambda^2*M + lambda*v*C1 + g*K0 + v^2*K2).
std::vector<std::complex<double>> quadratic_eigenvalues(
    const WhippleMatrices&, double v, double g);

struct Kkt {
    std::vector<double> vdot, lambda;
    double relative_residual = 0.;
    int rank = 0;
};

Kkt solve_kkt(std::span<const double> M, std::span<const double> bias,
              const RollingConstraints&, int nv);

} // namespace spatial_tests
