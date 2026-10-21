// blas_numpy_openblas.cpp — the Linux backend for cblas_abi.hpp's typed
// call face. The wheel's scipy-openblas provider is ILP64: its CBLAS
// entry points take 64-bit integers under the scipy_cblas_*64_ export
// spellings recorded in the dependency document's numerics record
// (numpy itself imports those exact symbols, so calling them preserves
// the bitwise contract the shared header documents).
//
// The foreign names are generated into numerics_symbols.hpp by
// native/cmake/BikeNumerics.cmake (configure_file) from the schema-1
// dependency record — nothing is retyped from memory, and the record's
// checked_symbol grammar already rejected any spelling that is not a
// safe [A-Za-z_][A-Za-z0-9_$]* identifier. The names land in __asm__
// labels carrying the ELF convention (no leading underscore; the Mach-O
// "_" is Darwin-only and must never be reused here).
//
// Every public int argument widens to the backend's int64_t through an
// explicit static_cast at the call — never a reinterpret_cast of integer
// storage and never a declaration shared with the 32-bit header.
//
// Exactly one backend TU is compiled per platform — BikeNumerics.cmake
// selects this file only for the resolved NUMPY_OPENBLAS backend. The
// body is guarded positively rather than by #error: cppcheck sweeps
// every first-party TU without platform defines, so a preprocessor
// guard must leave an analyzable empty TU — a miscompile then still
// fails closed at link time with undefined blas:: wrappers.
#if defined(__linux__)

#include "../cblas_abi.hpp"

#include "numerics_symbols.hpp"

#include <cstdint>
#include <type_traits>

// The generated record must describe this backend: a 64-bit BLAS ABI is
// the whole point of this TU — compiling it against a 32-bit record is
// memory corruption, so it cannot be configured silently.
#if !defined(BIKE_NUMERICS_BLAS_BITS) || BIKE_NUMERICS_BLAS_BITS != 64
#error "blas_numpy_openblas: numerics_symbols.hpp is not the ILP64 record"
#endif

namespace blas {
    // Defines the header's declaration (cblas_abi.hpp); the name matches the
    // NumPy build-dependency name the probe qualifies for this provider.
    const std::string_view provider_name = "scipy-openblas";

    // The provider's ILP64 integer type; private to this TU.
    using blas_int = std::int64_t;
    static_assert(sizeof(blas_int) * 8 == BIKE_NUMERICS_BLAS_BITS,
                  "blas_int does not match the qualified provider ABI");

    extern "C" {
        // The qualified ILP64 CBLAS signature, bound to the exact
        // provider exports by asm label. noexcept — the backend C API
        // cannot throw.
        double backend_ddot(blas_int n, const double *x, blas_int incx,
                            const double *y, blas_int incy) noexcept
            __asm__(BIKE_NUMERICS_SYM_DDOT);

        // The same Linux export has two faces: seeded ddot and the bare
        // ILP64 fold. Their distinction is arithmetic, not symbol identity.
        double backend_ddot_ilp64(blas_int n, const double *x, blas_int incx,
                                  const double *y, blas_int incy) noexcept
            __asm__(BIKE_NUMERICS_SYM_DDOT_ILP64);
        void backend_zdotu_sub(blas_int n, const void *x, blas_int incx,
                               const void *y, blas_int incy, void *out) noexcept
            __asm__(BIKE_NUMERICS_SYM_ZDOTU_SUB);

        void backend_dgemv(blas_int order, blas_int transa, blas_int m,
                           blas_int n, double alpha, const double *a,
                           blas_int lda, const double *x, blas_int incx,
                           double beta, double *y, blas_int incy) noexcept
            __asm__(BIKE_NUMERICS_SYM_DGEMV);

        void backend_dgemm(blas_int order, blas_int transa, blas_int transb,
                           blas_int m, blas_int n, blas_int k, double alpha,
                           const double *a, blas_int lda, const double *b,
                           blas_int ldb, double beta, double *c,
                           blas_int ldc) noexcept
            __asm__(BIKE_NUMERICS_SYM_DGEMM);
    }

    // Compile-time signature pins on the 64-bit interface — a drifted
    // prototype fails here before a call can encode the wrong ABI.
    static_assert(
        std::is_same_v<decltype(&backend_ddot),
                       double (*)(blas_int, const double *, blas_int,
                                  const double *, blas_int) noexcept>,
        "backend_ddot prototype drifted");
    static_assert(
        std::is_same_v<decltype(&backend_ddot_ilp64),
                       double (*)(blas_int, const double *, blas_int,
                                  const double *, blas_int) noexcept>,
        "backend_ddot_ilp64 prototype drifted");
    static_assert(
        std::is_same_v<decltype(&backend_zdotu_sub),
                       void (*)(blas_int, const void *, blas_int,
                                const void *, blas_int, void *) noexcept>,
        "backend_zdotu_sub prototype drifted");
    static_assert(
        std::is_same_v<decltype(&backend_dgemv),
                       void (*)(blas_int, blas_int, blas_int, blas_int,
                                double, const double *, blas_int,
                                const double *, blas_int, double,
                                double *, blas_int) noexcept>,
        "backend_dgemv prototype drifted");
    static_assert(
        std::is_same_v<decltype(&backend_dgemm),
                       void (*)(blas_int, blas_int, blas_int, blas_int,
                                blas_int, blas_int, double,
                                const double *, blas_int,
                                const double *, blas_int, double,
                                double *, blas_int) noexcept>,
        "backend_dgemm prototype drifted");

    // Symbol-link controls: materializing each address in a constant
    // initializer gives this TU a relocation for the provider export — a
    // missing scipy_*64_ name surfaces at link time.
    [[maybe_unused]] inline constexpr auto backend_link_ddot =
        &backend_ddot;
    [[maybe_unused]] inline constexpr auto backend_link_dgemv =
        &backend_dgemv;
    [[maybe_unused]] inline constexpr auto backend_link_dgemm =
        &backend_dgemm;
    [[maybe_unused]] inline constexpr auto backend_link_ddot_ilp64 = &backend_ddot_ilp64;
    [[maybe_unused]] inline constexpr auto backend_link_zdotu_sub = &backend_zdotu_sub;

    double ddot(int n, const double *x, int incx,
                const double *y, int incy) noexcept {
        // numpy's 1-D dot kernel (arraytypes.c.src @name@_dot) does not
        // return the raw cblas result — it seeds `double sum = 0.` and
        // accumulates `sum += cblas_ddot(chunk)`. For every n below
        // NPY_CBLAS_CHUNK that is exactly `0. + cblas_ddot(n, ...)`.
        // The literal addend is NOT cosmetic: it canonicalizes a -0.0
        // fold to +0.0 (and quiets a signaling NaN payload) the same
        // way the oracle does, so the wrapper below reproduces
        // `a @ b` bitwise where the bare provider call would not.
        return 0. + backend_ddot(static_cast<blas_int>(n), x,
                                 static_cast<blas_int>(incx), y,
                                 static_cast<blas_int>(incy));
    }

    double ddot_raw(int n, const double *x, int incx,
                    const double *y, int incy) noexcept {
        return backend_ddot_ilp64(static_cast<blas_int>(n), x,
                                  static_cast<blas_int>(incx), y,
                                  static_cast<blas_int>(incy));
    }

    void zdotu_sub(int n, const void *x, int incx, const void *y, int incy,
                   void *out) noexcept {
        backend_zdotu_sub(static_cast<blas_int>(n), x,
                          static_cast<blas_int>(incx), y,
                          static_cast<blas_int>(incy), out);
    }

    void dgemv(Order order, Transpose trans, int m, int n,
               double alpha, const double *a, int lda,
               const double *x, int incx, double beta, double *y,
               int incy) noexcept {
        backend_dgemv(static_cast<blas_int>(order),
                      static_cast<blas_int>(trans),
                      static_cast<blas_int>(m), static_cast<blas_int>(n),
                      alpha, a, static_cast<blas_int>(lda), x,
                      static_cast<blas_int>(incx), beta, y,
                      static_cast<blas_int>(incy));
    }

    void dgemm(Order order, Transpose transa, Transpose transb,
               int m, int n, int k, double alpha, const double *a,
               int lda, const double *b, int ldb, double beta,
               double *c, int ldc) noexcept {
        backend_dgemm(static_cast<blas_int>(order),
                      static_cast<blas_int>(transa),
                      static_cast<blas_int>(transb),
                      static_cast<blas_int>(m), static_cast<blas_int>(n),
                      static_cast<blas_int>(k), alpha, a,
                      static_cast<blas_int>(lda), b,
                      static_cast<blas_int>(ldb), beta, c,
                      static_cast<blas_int>(ldc));
    }
} // namespace blas

#endif // __linux__ — the Linux backend body
