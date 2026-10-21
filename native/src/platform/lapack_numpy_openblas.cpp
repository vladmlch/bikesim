// lapack_numpy_openblas.cpp — the Linux backend for lapack_abi.hpp's typed
// call face. The wheel's scipy-openblas provider is ILP64 (USE64BITINT
// in the qualified build): its Fortran entry points take 64-bit
// integers, and the export spelling scipy_dgelsd_64_ recorded in the
// dependency document's numerics record is exactly what numpy's
// _umath_linalg imports — calling it preserves the bitwise contract the
// shared header documents.
//
// The foreign name is generated into numerics_symbols.hpp by
// native/cmake/BikeNumerics.cmake (configure_file) from the schema-1
// dependency record — nothing is retyped from memory, and the record's
// checked_symbol grammar already rejected any spelling that is not a
// safe [A-Za-z_][A-Za-z0-9_$]* identifier. The name lands in an
// __asm__ label carrying the ELF convention (no leading underscore; the
// Mach-O "_" is Darwin-only and must never be reused here).
//
// Exactly one backend TU is compiled per platform — BikeNumerics.cmake
// selects this file only for the resolved NUMPY_OPENBLAS backend. The
// body is guarded positively rather than by #error: cppcheck sweeps
// every first-party TU without platform defines, so a preprocessor
// guard must leave an analyzable empty TU — a miscompile then still
// fails closed at link time with undefined numeric_backend:: wrappers.
#if defined(__linux__)

#include "../lapack_abi.hpp"

#include "numerics_symbols.hpp"

#include <type_traits>

// The generated record must describe this backend: a 64-bit LAPACK ABI
// is the whole point of this TU — compiling it against a 32-bit record
// is memory corruption, so it cannot be configured silently.
#if !defined(BIKE_NUMERICS_LAPACK_BITS) || BIKE_NUMERICS_LAPACK_BITS != 64
#error "lapack_numpy_openblas: numerics_symbols.hpp is not the ILP64 record"
#endif

namespace numeric_backend {
    // Declared inside namespace numeric_backend so nothing else in a TU
    // sees an unqualified foreign name; extern "C" plus the asm label
    // keep the linked symbol exactly the qualified provider export.
    // noexcept — the backend C API cannot throw.
    extern "C" {
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) LAPACK argument order is fixed
        void backend_dgelsd(const integer *m, const integer *n,
                            const integer *nrhs, double *a,
                            const integer *lda, double *b,
                            const integer *ldb, double *s,
                            const double *rcond, integer *rank,
                            double *work, const integer *lwork,
                            integer *iwork, integer *info) noexcept
            __asm__(BIKE_NUMERICS_SYM_DGELSD);
        void backend_dgesv(const integer *n, const integer *nrhs, double *a,
                           const integer *lda, integer *ipiv, double *b,
                           const integer *ldb, integer *info) noexcept
            __asm__(BIKE_NUMERICS_SYM_DGESV);
        void backend_dsyevd(const char *jobz, const char *uplo, const integer *n,
                            double *a, const integer *lda, double *w, double *work,
                            const integer *lwork, integer *iwork,
                            const integer *liwork, integer *info) noexcept
            __asm__(BIKE_NUMERICS_SYM_DSYEVD);
    }

    // Compile-time signature pin on the 64-bit interface — a drifted
    // prototype fails here before a call can encode the wrong ABI.
    static_assert(
        std::is_same_v<decltype(&backend_dgelsd),
                       void (*)(const integer *, const integer *,
                                const integer *, double *,
                                const integer *, double *,
                                const integer *, double *, const double *,
                                integer *, double *, const integer *,
                                integer *, integer *) noexcept>,
        "backend_dgelsd prototype drifted");
    static_assert(
        std::is_same_v<decltype(&backend_dgesv),
                       void (*)(const integer *, const integer *, double *,
                                const integer *, integer *, double *,
                                const integer *, integer *) noexcept>,
        "backend_dgesv prototype drifted");
    static_assert(
        std::is_same_v<decltype(&backend_dsyevd),
                       void (*)(const char *, const char *, const integer *,
                                double *, const integer *, double *, double *,
                                const integer *, integer *, const integer *,
                                integer *) noexcept>,
        "backend_dsyevd prototype drifted");

    // Symbol-link control: materializing the address in a constant
    // initializer gives this TU a relocation for the provider export —
    // a missing scipy_dgelsd_64_ surfaces at link time.
    [[maybe_unused]] inline constexpr auto backend_link_dgelsd =
        &backend_dgelsd;
    [[maybe_unused]] inline constexpr auto backend_link_dgesv = &backend_dgesv;
    [[maybe_unused]] inline constexpr auto backend_link_dsyevd = &backend_dsyevd;

    // The common function forwards to the one exact declaration — the
    // argument types are identical, so no LP64 pointer casts exist.
    void dgelsd(const integer *m, const integer *n, const integer *nrhs,
                double *a, const integer *lda, double *b,
                const integer *ldb, double *s, const double *rcond,
                integer *rank, double *work, const integer *lwork,
                integer *iwork, integer *info) noexcept {
        backend_dgelsd(m, n, nrhs, a, lda, b, ldb, s, rcond, rank, work,
                       lwork, iwork, info);
    }

    void dgesv(const integer *n, const integer *nrhs, double *a,
               const integer *lda, integer *ipiv, double *b,
               const integer *ldb, integer *info) noexcept {
        backend_dgesv(n, nrhs, a, lda, ipiv, b, ldb, info);
    }

    void dsyevd(const char *jobz, const char *uplo, const integer *n,
                double *a, const integer *lda, double *w, double *work,
                const integer *lwork, integer *iwork, const integer *liwork,
                integer *info) noexcept {
        backend_dsyevd(jobz, uplo, n, a, lda, w, work, lwork, iwork, liwork, info);
    }
} // namespace numeric_backend

#endif // __linux__ — the Linux backend body
