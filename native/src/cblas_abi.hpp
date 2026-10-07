// cblas_abi.hpp — the single declaration site for the legacy CBLAS
// symbols this port shares with numpy.
//
// Bitwise contract: `a @ b` / `A @ x` / `np.dot` / `np.linalg.norm` in
// CPython-numpy dispatch to BLAS — on this platform numpy links Apple
// Accelerate, whose ddot/dgemv/dgemm use vectorized multi-accumulator
// orders a sequential C++ loop cannot reproduce. The ported call sites
// invoke the SAME entry points through the typed wrappers below; probes
// verified each call reproduces the numpy expression bitwise (2000
// trials each — writers/resistance.cpp keeps the mapping table):
//   a @ b            == cblas_ddot(n, a, 1, b, 1)
//   A(3,nv) @ x      == cblas_dgemv(RowMajor, NoTrans, 3, nv, 1, A, nv, ...)
//   A.T @ x (F-view) == cblas_dgemv(RowMajor, Trans,    3, nv, 1, A, nv, ...)
//
// Why a hand-written declaration: <Accelerate/Accelerate.h> marks these
// symbols deprecated (a hard error under -Werror) and its vecLib headers
// do not parse under gcc -fsyntax-only. The classic CBLAS integer
// constants are the stable ABI values; the static_asserts pin the
// declared ABI inside this header because no test case can reach a
// declaration-level drift.
#pragma once

#include <mujoco/mujoco.h>

#include <cstdint>
#include <type_traits>

// Every caller moves mjtNum buffers — if the engine scalar ever widened,
// these prototypes would silently disagree with the Accelerate ABI.
static_assert(std::is_same_v<mjtNum, double>,
              "cblas_abi: the pinned CBLAS ABI requires binary64 mjtNum");
// The classic CBLAS interface is 32-bit-int — the same width Accelerate
// exports on this platform.
static_assert(sizeof(int) == 4,
              "cblas_abi: the pinned CBLAS ABI requires a 32-bit int");

namespace blas {
    // Classic CBLAS integer constants (stable since the reference CBLAS;
    // the same values Accelerate's vecLib ABI accepts). enum class
    // arguments make the magic integers unrepresentable at call sites.
    // The underlying type stays narrow — the wrappers widen to the
    // 32-bit-int ABI at the boundary.
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

    // Declared inside namespace blas so nothing else in a TU sees an
    // unqualified cblas_* name; extern "C" keeps the linked symbol names
    // identical to the Accelerate exports.
    extern "C" {
        double cblas_ddot(int n, const double *x, int incx,
                          const double *y, int incy);

        void cblas_dgemv(int order, int transa, int m, int n, double alpha,
                         const double *a, int lda, const double *x, int incx,
                         double beta, double *y, int incy);

        void cblas_dgemm(int order, int transa, int transb, int m, int n,
                         int k, double alpha, const double *a, int lda,
                         const double *b, int ldb, double beta, double *c,
                         int ldc);
    }

    // Compile-time signature pins — a drifted prototype fails here before
    // a call can encode the wrong ABI.
    static_assert(
        std::is_same_v<decltype(&cblas_ddot),
                       double (*)(int, const double *, int,
                                  const double *, int)>,
        "cblas_abi: cblas_ddot prototype drifted");
    static_assert(
        std::is_same_v<decltype(&cblas_dgemv),
                       void (*)(int, int, int, int, double,
                                const double *, int, const double *, int,
                                double, double *, int)>,
        "cblas_abi: cblas_dgemv prototype drifted");
    static_assert(
        std::is_same_v<decltype(&cblas_dgemm),
                       void (*)(int, int, int, int, int, int, double,
                                const double *, int, const double *, int,
                                double, double *, int)>,
        "cblas_abi: cblas_dgemm prototype drifted");

    // Symbol-link controls: materializing each address in a constant
    // initializer gives every ODR-using TU a relocation for the symbol —
    // both the extension and the contract binary link Accelerate, so a
    // missing export surfaces at link time rather than as a surprise.
    [[maybe_unused]] inline constexpr auto cblas_link_ddot = &cblas_ddot;
    [[maybe_unused]] inline constexpr auto cblas_link_dgemv = &cblas_dgemv;
    [[maybe_unused]] inline constexpr auto cblas_link_dgemm = &cblas_dgemm;

    // Typed call faces: Order/Transpose arrive as enum-class arguments so
    // the classic magic integers cannot reach the ABI by accident.
    // noexcept — the C entry points cannot throw.
    [[nodiscard]] inline double ddot(int n, const double *x, int incx,
                                     const double *y, int incy) noexcept {
        return cblas_ddot(n, x, incx, y, incy);
    }

    inline void dgemv(Order order, Transpose trans, int m, int n,
                    double alpha, const double *a, int lda,
                    const double *x, int incx, double beta, double *y,
                    int incy) noexcept {
        cblas_dgemv(static_cast<int>(order), static_cast<int>(trans), m, n,
                    alpha, a, lda, x, incx, beta, y, incy);
    }

    inline void dgemm(Order order, Transpose transa, Transpose transb,
                    int m, int n, int k, double alpha, const double *a,
                    int lda, const double *b, int ldb, double beta,
                    double *c, int ldc) noexcept {
        cblas_dgemm(static_cast<int>(order), static_cast<int>(transa),
                    static_cast<int>(transb), m, n, k, alpha, a, lda, b,
                    ldb, beta, c, ldc);
    }
} // namespace blas
