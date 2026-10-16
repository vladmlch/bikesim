// cblas_abi.hpp — the typed call face for the platform CBLAS backend this
// port shares with numpy.
//
// Bitwise contract: `a @ b` / `A @ x` / `np.dot` / `np.linalg.norm` in
// CPython-numpy dispatch to BLAS — the provider's ddot/dgemv/dgemm use
// vectorized multi-accumulator orders a sequential C++ loop cannot
// reproduce. The ported call sites invoke the SAME entry points through
// the typed wrappers declared below; probes verified each call
// reproduces the numpy expression bitwise (2000 trials each —
// writers/resistance.cpp keeps the mapping table):
//   a @ b            == 0. + cblas_ddot(n, a, 1, b, 1)
//   A(3,nv) @ x      == cblas_dgemv(RowMajor, NoTrans, 3, nv, 1, A, nv, ...)
//   A.T @ x (F-view) == cblas_dgemv(RowMajor, Trans,    3, nv, 1, A, nv, ...)
//
// The literal `0. +` on ddot is part of the oracle: numpy's @name@_dot
// kernel seeds `double sum = 0.` and accumulates the cblas result into
// it, so `a @ b` yields +0.0 where the bare provider fold yields -0.0
// (and quiets a signaling NaN). blas::ddot returns the seeded value —
// it mirrors numpy's dot kernel, not the raw cblas symbol; every
// production call site ports a numpy dot expression, so the seed lives
// in this one wrapper rather than at ~30 call sites.
//
// Why no foreign declarations here: the CBLAS integer width is
// platform-private. macOS links Accelerate's legacy 32-bit cblas_*;
// Linux links the NumPy wheel's scipy-openblas ILP64 exports. Each
// backend lives in exactly one translation unit
// (src/platform/blas_accelerate.cpp / blas_numpy_openblas.cpp) that
// carries the qualified foreign declarations, the compile-time signature
// pins, and these wrappers' definitions; the wrappers widen int arguments
// at that boundary and the 64-bit ABI never escapes this header.
// native/cmake/BikeNumerics.cmake compiles only the resolved backend
// into bike_native_numerics.
//
// The classic CBLAS integer constants are the stable ABI values; the
// static_asserts pin the declared ABI inside this header because no test
// case can reach a declaration-level drift.
#pragma once

#include <mujoco/mujoco.h>

#include <cstdint>
#include <type_traits>

// Every caller moves mjtNum buffers — if the engine scalar ever widened,
// these prototypes would silently disagree with the provider ABI.
static_assert(std::is_same_v<mjtNum, double>,
              "cblas_abi: the pinned CBLAS ABI requires binary64 mjtNum");
// The typed call face is 32-bit-int; backends widen to the qualified
// provider width inside their own translation units.
static_assert(sizeof(int) == 4,
              "cblas_abi: the pinned CBLAS ABI requires a 32-bit int");

namespace blas {
    // Classic CBLAS integer constants (stable since the reference CBLAS;
    // accepted unchanged by both qualified providers). enum class
    // arguments make the magic integers unrepresentable at call sites.
    // The underlying type stays narrow — the wrappers widen to the
    // provider's integer ABI at the boundary.
    enum class Order : std::uint8_t {
        row_major = 101,
        column_major = 102
    };
    enum class Transpose : std::uint8_t {
        no = 111,
        yes = 112
    };
    static_assert(static_cast<int>(Order::row_major) == 101 &&
                  static_cast<int>(Order::column_major) == 102,
                  "cblas_abi: Order must carry the CBLAS ABI values");
    static_assert(static_cast<int>(Transpose::no) == 111 &&
                  static_cast<int>(Transpose::yes) == 112,
                  "cblas_abi: Transpose must carry the CBLAS ABI values");

    // Typed call faces: Order/Transpose arrive as enum-class arguments so
    // the classic magic integers cannot reach the ABI by accident.
    // noexcept — the C entry points cannot throw; the backend C API
    // cannot either. Defined in the one selected backend TU.
    [[nodiscard]] double ddot(int n, const double *x, int incx,
                              const double *y, int incy) noexcept;

    void dgemv(Order order, Transpose trans, int m, int n, double alpha,
               const double *a, int lda, const double *x, int incx,
               double beta, double *y, int incy) noexcept;

    void dgemm(Order order, Transpose transa, Transpose transb,
               int m, int n, int k, double alpha, const double *a,
               int lda, const double *b, int ldb, double beta, double *c,
               int ldc) noexcept;
} // namespace blas
