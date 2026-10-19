// lapack_accelerate.cpp — the Darwin backend for lapack_abi.hpp's typed
// call face. macOS numpy links Apple Accelerate; dgelsd$NEWLAPACK$ILP64
// is the SAME entry point the Python oracle's np.linalg.lstsq resolves
// to (nm -u shows the identical import on _umath_linalg), so invoking
// it preserves the bitwise contract the shared header documents.
//
// Why a hand-written declaration: <Accelerate/Accelerate.h> marks the
// LAPACK surface deprecated (a hard error under -Werror) and its vecLib
// headers do not parse under the gcc frontends sweep. The foreign
// signature below is a verbatim move of the declaration
// rider/least_squares.cpp carried before the backend split —
// Accelerate's ILP64 Fortran ABI, __LAPACK_int == long (signed 64-bit),
// every argument by reference — bound to the exact export the
// dependency record qualified (numerics.symbols:
// dgelsd$NEWLAPACK$ILP64, generated into numerics_symbols.hpp as
// BIKE_NUMERICS_SYM_DGELSD carrying the Mach-O "_" prefix).
//
// Exactly one backend TU is compiled per platform — BikeNumerics.cmake
// selects this file only for the resolved ACCELERATE backend. The body
// is guarded positively rather than by #error: cppcheck sweeps every
// first-party TU without platform defines, so a preprocessor guard
// must leave an analyzable empty TU — a miscompile then still fails
// closed at link time with undefined numeric_backend:: wrappers.
#if defined(__APPLE__)

#include "../lapack_abi.hpp"

#include "numerics_symbols.hpp"

#include <type_traits>

// The generated record must describe this backend: the
// NEWLAPACK$ILP64 spelling IS Accelerate's 64-bit interface — compiling
// this TU against a 32-bit LAPACK record would be memory corruption,
// so it cannot be configured silently.
#if !defined(BIKE_NUMERICS_LAPACK_BITS) || BIKE_NUMERICS_LAPACK_BITS != 64
#error "lapack_accelerate: numerics_symbols.hpp is not the ILP64 record"
#endif

namespace numeric_backend {
    // Declared inside namespace numeric_backend so nothing else in a TU
    // sees an unqualified foreign name; extern "C" plus the asm label
    // keep the linked symbol exactly the qualified provider export.
    extern "C" {
        // The ILP64 ABI numpy links: __LAPACK_int is `long` (signed
        // 64-bit) under ACCELERATE_LAPACK_ILP64, and every Fortran
        // argument arrives by reference.
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) LAPACK argument order is fixed
        void backend_dgelsd(const integer *m, const integer *n,
                            const integer *nrhs, double *a,
                            const integer *lda, double *b,
                            const integer *ldb, double *s,
                            const double *rcond, integer *rank,
                            double *work, const integer *lwork,
                            integer *iwork, integer *info) noexcept
            __asm__(BIKE_NUMERICS_SYM_DGELSD);
    }

    static_assert(
        sizeof(long) == 8,
        "dgelsd ILP64 requires a 64-bit long (__LAPACK_int)");
    // Compile-time signature pin — a drifted prototype fails here
    // before a call can encode the wrong ABI.
    static_assert(
        std::is_same_v<decltype(&backend_dgelsd),
                       void (*)(const integer *, const integer *,
                                const integer *, double *,
                                const integer *, double *,
                                const integer *, double *, const double *,
                                integer *, double *, const integer *,
                                integer *, integer *) noexcept>,
        "backend_dgelsd prototype drifted");

    // Symbol-link control: materializing the address in a constant
    // initializer gives this TU a relocation for the provider export —
    // a missing dgelsd$NEWLAPACK$ILP64 surfaces at link time.
    [[maybe_unused]] inline constexpr auto backend_link_dgelsd =
        &backend_dgelsd;

    // The common function forwards to the one exact declaration — the
    // argument types are identical, so no casts exist here at all.
    void dgelsd(const integer *m, const integer *n, const integer *nrhs,
                double *a, const integer *lda, double *b,
                const integer *ldb, double *s, const double *rcond,
                integer *rank, double *work, const integer *lwork,
                integer *iwork, integer *info) noexcept {
        backend_dgelsd(m, n, nrhs, a, lda, b, ldb, s, rcond, rank, work,
                       lwork, iwork, info);
    }
} // namespace numeric_backend

#endif // __APPLE__ — the Darwin backend body
