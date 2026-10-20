// rider/spindle_linalg.hpp — the small linear-algebra mirrors the
// spindle controller's ported expressions resolve to
// (rider_control.py / lean_limit.py). Every helper pins ONE numpy call
// shape on packed C-order operands:
//
//   `a @ x`   (3,3)/(2,2)        -> mat_vec / mat_vec2
//   `a.T @ x` (3,3)/(2,2)        -> mat_t_vec / mat_t_vec2, with
//                                 mat_t_vec_xz the (x,z) lane slice the
//                                 oracle actually consumes
//   `a @ b`   (3,3)              -> mat_mul
//   `a.T @ b` (3,3)              -> mat_t_mul
//   np.linalg.norm(v) 1-D        -> np_norm (sqrt(0.0 + v.dot(v)))
//
// NumPy dispatches every one of these through the provider BLAS
// (umath/matmul.c.src @TYPE@_gemv/@TYPE@_gemm for @, arraytypes.c.src
// @name@_dot for x.dot(x)) whose accumulation order is platform-private
// — the measured per-provider table lives in
// docs/agents/native-numerical-profiles.md. The implementations below
// call the same blas:: wrappers the writers use; scalar emulations are
// NOT a substitute — scipy-openblas fuses these folds where Accelerate
// does not (and vice versa for the non-transposed row matvec).
//
// binding.cpp's _debug_rider_linalg drives these exact functions, so
// the bitwise oracle tests exercise production code — never a second
// implementation.
#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <span>
#include <stdexcept>

#include "../cblas_abi.hpp"
#include "../diag.hpp"
#include "contact_math.hpp" // rider::Vec3 / Mat3

namespace rider {
    using Vec2 = std::array<double, 2>;
    using Mat2 = std::array<double, 4>; // C-order packed

    // np.linalg.norm(v) on a 1-D float64 vector: axis=None folds
    // `x.dot(x)` then sqrt — DOUBLE_dot's seeded `0. + cblas_ddot`
    // accumulation is carried inside blas::ddot (cblas_abi.hpp), so
    // an all-zeros vector yields +0.0 exactly as the wheel does.
    // Empty input is +0.0.
    [[nodiscard]] inline double np_norm(std::span<const double> v) {
        if (v.empty()) return 0.;
        if (v.size() >
            static_cast<std::size_t>(std::numeric_limits<int>::max()))
            throw std::invalid_argument(
                "np_norm vector exceeds the 32-bit call face");
        return std::sqrt(blas::ddot(static_cast<int>(v.size()), v.data(),
                                    1, v.data(), 1));
    }

    // `a @ x` on a packed C-order (n,n): NumPy's matvec ufunc issues
    // the column-dot dgemv(ColMajor, Trans) call, which is the same
    // provider kernel as the row-major NoTrans entry on both qualified
    // providers.
    [[nodiscard]] inline Vec3 mat_vec(const Mat3 &a, const Vec3 &x) {
        Vec3 out{};
        blas::dgemv(blas::Order::row_major, blas::Transpose::no,
                    3, 3, 1., a.data(), 3, x.data(), 1, 0., out.data(),
                    1);
        return out;
    }

    // `a.T @ x` on a packed C-order (3,3): the F-view operand reaches
    // dgemv(RowMajor, Trans) — a DIFFERENT provider kernel than the
    // NoTrans entry, fused on both qualified providers.
    [[nodiscard]] inline Vec3 mat_t_vec(const Mat3 &a, const Vec3 &x) {
        Vec3 out{};
        blas::dgemv(blas::Order::row_major, blas::Transpose::yes,
                    3, 3, 1., a.data(), 3, x.data(), 1, 0., out.data(),
                    1);
        return out;
    }

    // `(a.T @ x)[[0,2]]` — the xz slice the controller consumes. The
    // provider computes all three lanes anyway; per-element bits are
    // what the oracle compares.
    [[nodiscard]] inline Vec2 mat_t_vec_xz(const Mat3 &a, const Vec3 &x) {
        const Vec3 mv = mat_t_vec(a, x);
        return {mv[0], mv[2]};
    }

    // `a @ b` on packed C-order (3,3) operands — dgemm(RowMajor,
    // NoTrans, NoTrans) exactly as the matmul ufunc's special-casing
    // leaves it. Mat3-sized by-value returns are the designed interface.
    NATIVE_DIAG_PUSH
    NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
    [[nodiscard]] inline Mat3 mat_mul(const Mat3 &a, const Mat3 &b) {
        Mat3 out{};
        blas::dgemm(blas::Order::row_major, blas::Transpose::no,
                    blas::Transpose::no, 3, 3, 3, 1., a.data(), 3,
                    b.data(), 3, 0., out.data(), 3);
        return out;
    }

    // `a.T @ b` on packed C-order (3,3) operands — the F-view operand
    // reaches dgemm(RowMajor, Trans, NoTrans).
    [[nodiscard]] inline Mat3 mat_t_mul(const Mat3 &a, const Mat3 &b) {
        Mat3 out{};
        blas::dgemm(blas::Order::row_major, blas::Transpose::yes,
                    blas::Transpose::no, 3, 3, 3, 1., a.data(), 3,
                    b.data(), 3, 0., out.data(), 3);
        return out;
    }
    NATIVE_DIAG_POP

    // `a @ x` on a packed C-order (2,2) — lean_limit's `rotation @ v`.
    [[nodiscard]] inline Vec2 mat_vec2(const Mat2 &a, const Vec2 &x) {
        Vec2 out{};
        blas::dgemv(blas::Order::row_major, blas::Transpose::no,
                    2, 2, 1., a.data(), 2, x.data(), 1, 0., out.data(),
                    1);
        return out;
    }

    // `a.T @ x` on a packed C-order (2,2) — lean_limit's `rotation.T @ v`.
    [[nodiscard]] inline Vec2 mat_t_vec2(const Mat2 &a, const Vec2 &x) {
        Vec2 out{};
        blas::dgemv(blas::Order::row_major, blas::Transpose::yes,
                    2, 2, 1., a.data(), 2, x.data(), 1, 0., out.data(),
                    1);
        return out;
    }
} // namespace rider
