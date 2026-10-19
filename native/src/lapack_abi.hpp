// lapack.hpp — the typed call face for the platform LAPACK backend this
// port shares with numpy.
//
// Bitwise contract: np.linalg.lstsq's gufunc (numpy/linalg/
// umath_linalg.cpp, lstsq over the 'ddd->ddid' signature) drives DGELSD
// through the provider's ILP64 entry point — dgelsd$NEWLAPACK$ILP64
// under macOS Accelerate, scipy_dgelsd_64_ inside the wheel's
// scipy-openblas on Linux. The ported solver drives the SAME exported
// routine with the same argument tuple, so the provider's own operation
// order — not a reimplemented divide-and-conquer — produces
// oracle-identical results.
//
// Why no foreign declarations here: the provider entry spellings and
// any platform detail stay backend-private; only the qualified ILP64
// (64-bit integer) ABI is expressed. Each backend lives in exactly one
// translation unit (src/platform/lapack_accelerate.cpp /
// lapack_numpy_openblas.cpp) that carries the qualified foreign
// declaration, the compile-time signature pins, and this wrapper's
// definition; native/cmake/BikeNumerics.cmake compiles only the
// resolved backend into bike_native_numerics.
//
// Fortran convention: every argument arrives by reference, including
// the read-only scalars — the const-qualified pointers describe the
// callee's access, never ownership or shared storage.
#pragma once

#include <cstdint>
#include <limits>
#include <type_traits>

namespace numeric_backend {
    // The qualified LAPACK integer width — ILP64 on both providers
    // (Accelerate's __LAPACK_int == long under ACCELERATE_LAPACK_ILP64,
    // scipy-openblas built with USE64BITINT).
    using integer = std::int64_t;

    // DGELSD's exact 14-argument Fortran signature. noexcept — a
    // Fortran routine cannot throw; the backend C API cannot either.
    // Defined in the one selected backend TU.
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) LAPACK argument order is fixed
    void dgelsd(const integer *m, const integer *n, const integer *nrhs,
                double *a, const integer *lda, double *b,
                const integer *ldb, double *s, const double *rcond,
                integer *rank, double *work, const integer *lwork,
                integer *iwork, integer *info) noexcept;
} // namespace numeric_backend

// Width and format pins: the qualified ABI is 64-bit Fortran integers
// and IEEE-754 binary64 elements — a drifted platform would silently
// encode the wrong call.
static_assert(std::is_same_v<numeric_backend::integer, std::int64_t>,
              "lapack_abi: the qualified DGELSD ABI requires int64 integers");
static_assert(sizeof(double) == 8 &&
                  std::numeric_limits<double>::is_iec559,
              "lapack_abi: the qualified DGELSD ABI requires binary64 doubles");
