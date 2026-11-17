// native/src/physics_core/exact_math.hpp — Host-only CPU FP64 exact math wrappers
// preserving 0-bit parity with BLAS, NumPy, CPython math.hypot, and libm pow.
#pragma once

#if defined(__METAL_VERSION__) || defined(__CUDA_ARCH__)
#error "exact_math.hpp / cpu_exact_toolkit.hpp are host-only CPU FP64 headers"
#endif

#include "common.hpp"

#include "../cblas_abi.hpp"
#include "../numeric_norm.hpp"
#include "../numeric_numpy.hpp"
#include "../numeric_sum.hpp"
#include "../writers/pyfloat.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace bike::physics_core::exact_math {

using numeric::PythonSum;
using numeric::numpy_sum;

// NOLINTBEGIN(bugprone-easily-swappable-parameters) positional wrappers mirror upstream BLAS/NumPy/CPython signatures
[[nodiscard]] inline double pow2_py(double x) {
    return pyfloat::pow(x, 2.0);
}

[[nodiscard]] inline double hypot2_py(double a, double b) noexcept {
    const std::array<double, 2> v = {a, b};
    return numeric::python_vector_norm(v);
}

[[nodiscard]] inline double interp_segment(
    double s, double x, double x0, double y0, double x1, double y1
) noexcept {
    return numeric::np_interp_segment(s, x, x0, y0, x1, y1);
}

[[nodiscard]] inline double clip_preserve_signed_zero(
    double x, double lo, double hi
) noexcept {
    return x < lo ? lo : (x > hi ? hi : x);
}
// NOLINTEND(bugprone-easily-swappable-parameters)

} // namespace bike::physics_core::exact_math
