// native/src/physics_core/cpu_exact_toolkit.hpp — Host-only CPU FP64 toolkit
// matching native/src 0-bit semantics across BLAS, NumPy, and CPython math.
#pragma once

#if defined(__METAL_VERSION__) || defined(__CUDA_ARCH__)
#error "exact_math.hpp / cpu_exact_toolkit.hpp are host-only CPU FP64 headers"
#endif

#include "common.hpp"
#include "exact_math.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <span>

namespace bike::physics_core {

// NOLINTBEGIN(bugprone-easily-swappable-parameters) positional toolkit signatures mirror GpuToolkit and BLAS/NumPy/CPython contracts
struct CpuExactToolkit {
    using Real = double;

    [[nodiscard]] static double div(double a, double b) noexcept {
        return a / b;
    }

    [[nodiscard]] static double sqrt(double x) noexcept {
        return std::sqrt(x);
    }

    [[nodiscard]] static double abs(double x) noexcept {
        return std::abs(x);
    }

    [[nodiscard]] static bool isfinite(double x) noexcept {
        return std::isfinite(x);
    }

    [[nodiscard]] static bool isnan(double x) noexcept {
        return std::isnan(x);
    }

    [[nodiscard]] static double exp(double x) noexcept {
        return std::exp(x);
    }

    [[nodiscard]] static double log(double x) noexcept {
        return std::log(x);
    }

    [[nodiscard]] static double tanh(double x) noexcept {
        return std::tanh(x);
    }

    [[nodiscard]] static double floor(double x) noexcept {
        return std::floor(x);
    }

    [[nodiscard]] static double sq(double x) noexcept {
        return x * x;
    }

    [[nodiscard]] static double pow2(double x) {
        return pyfloat::pow(x, 2.0);
    }

    [[nodiscard]] static double pow3(double x) {
        return pyfloat::pow(x, 3.0);
    }

    [[nodiscard]] static double pow_py(double a, double b) {
        return pyfloat::pow(a, b);
    }

    [[nodiscard]] static double hypot2(double a, double b) noexcept {
        return exact_math::hypot2_py(a, b);
    }

    [[nodiscard]] static double interp_segment(
        double s, double x, double x0, double y0, double x1, double y1
    ) noexcept {
        return numeric::np_interp_segment(s, x, x0, y0, x1, y1);
    }

    [[nodiscard]] static double min(double a, double b) noexcept {
        return std::min(a, b);
    }

    [[nodiscard]] static double max(double a, double b) noexcept {
        return std::max(a, b);
    }

    [[nodiscard]] static double clamp(double x, double lo, double hi) noexcept {
        return x < lo ? lo : (x > hi ? hi : x);
    }

    [[nodiscard]] static double dot2(const double *a, const double *b) noexcept {
        return blas::ddot(2, a, 1, b, 1);
    }

    [[nodiscard]] static double dot3(const double *a, const double *b) noexcept {
        return blas::ddot(3, a, 1, b, 1);
    }

    [[nodiscard]] static double dot(int n, const double *a, const double *b) noexcept {
        return blas::ddot(n, a, 1, b, 1);
    }

    [[nodiscard]] static double masked_sum(
        int n, const double *vals, const bool *mask
    ) noexcept {
        if (n <= 0 || vals == nullptr || mask == nullptr) {
            return 0.0;
        }
        const std::span<const double> v = std::views::counted(vals, n);
        const std::span<const bool> m = std::views::counted(mask, n);
        double f_result = 0.0;
        double c_sum = 0.0;
        for (std::size_t i = 0; i < v.size(); ++i) {
            if (!m[i]) {
                continue;
            }
            const double x = v[i];
            const double t = f_result + x;
            if (std::isfinite(t)) {
                if (std::abs(f_result) >= std::abs(x)) {
                    c_sum += (f_result - t) + x;
                } else {
                    c_sum += (x - t) + f_result;
                }
            }
            f_result = t;
        }
        return f_result + c_sum;
    }

    [[nodiscard]] static double norm2_ddot(const double *a) noexcept {
        return std::sqrt(blas::ddot(2, a, 1, a, 1));
    }

    [[nodiscard]] static double norm3_ddot(const double *a) noexcept {
        return std::sqrt(blas::ddot(3, a, 1, a, 1));
    }

    [[nodiscard]] static double einsum2(
        double a0, double a1, double b0, double b1
    ) noexcept {
        return 0.0 + a0 * b0 + a1 * b1;
    }

    static void gemv_n3(
        int nv, const double *A, const double *x, double *y
    ) noexcept {
        blas::dgemv(
            blas::Order::row_major, blas::Transpose::no,
            3, nv, 1.0, A, nv, x, 1, 0.0, y, 1);
    }

    static void gemv_t3(
        int nv, const double *A, const double *x, double *y
    ) noexcept {
        blas::dgemv(
            blas::Order::row_major, blas::Transpose::yes,
            3, nv, 1.0, A, nv, x, 1, 0.0, y, 1);
    }

    static void gemv_n2(
        int rows, const double *A, const double *x, double *y
    ) noexcept {
        blas::dgemv(
            blas::Order::row_major, blas::Transpose::no,
            rows, 2, 1.0, A, 2, x, 1, 0.0, y, 1);
    }

    static void ring_lift(
        int n, const double *L, const double *R, double *out
    ) noexcept {
        blas::dgemm(
            blas::Order::row_major, blas::Transpose::no, blas::Transpose::yes,
            n, 3, 3, 1.0, L, 3, R, 3, 0.0, out, 3);
    }
};
// NOLINTEND(bugprone-easily-swappable-parameters)

} // namespace bike::physics_core
