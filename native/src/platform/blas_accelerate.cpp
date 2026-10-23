// blas_accelerate.cpp — the Darwin backend for cblas_abi.hpp's typed
// call face. macOS numpy links Apple Accelerate. The existing seeded dot,
// dgemv and dgemm keep their qualified legacy 32-bit exports; geometry's
// bare dot and complex convolution keep NumPy's NEWLAPACK ILP64 imports.
//
// Why hand-written declarations: <Accelerate/Accelerate.h> marks these
// symbols deprecated (a hard error under -Werror) and its vecLib headers
// do not parse under gcc -fsyntax-only. The foreign signatures below are
// verbatim copies of what cblas_abi.hpp declared before the backend
// split — the classic 32-bit CBLAS integer ABI Accelerate exports. The
// exact export spellings are also pinned by the dependency probe
// (numerics.symbols: ddot/dgemv/dgemm plus the ILP64 ddot_ilp64/zdotu_sub).
//
// Exactly one backend TU is compiled per platform — BikeNumerics.cmake
// selects this file only for the resolved ACCELERATE backend. The body is
// guarded positively rather than by #error: cppcheck sweeps every first-
// party TU without platform defines, so a preprocessor guard must leave an
// analyzable empty TU — a miscompile then still fails closed at link time
// with undefined blas:: wrappers.
#if defined(__APPLE__)

#include "../cblas_abi.hpp"

#include "numerics_symbols.hpp"

namespace blas {
    // Defines the header's declaration (cblas_abi.hpp); the name matches the
    // NumPy build-dependency name the probe qualifies for this provider.
    // NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) string_view literal init is constant-initialized; the extern decl keeps the two-provider link model
    const std::string_view provider_name = "accelerate";

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

        double backend_ddot_ilp64(std::int64_t n, const double *x,
                                  std::int64_t incx, const double *y,
                                  std::int64_t incy) noexcept
            __asm__(BIKE_NUMERICS_SYM_DDOT_ILP64);
        void backend_zdotu_sub(std::int64_t n, const void *x,
                               std::int64_t incx, const void *y,
                               std::int64_t incy, void *out) noexcept
            __asm__(BIKE_NUMERICS_SYM_ZDOTU_SUB);
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
    static_assert(
        std::is_same_v<decltype(&backend_ddot_ilp64),
                       double (*)(std::int64_t, const double *, std::int64_t,
                                  const double *, std::int64_t) noexcept>,
        "cblas_abi: ILP64 ddot prototype drifted");
    static_assert(
        std::is_same_v<decltype(&backend_zdotu_sub),
                       void (*)(std::int64_t, const void *, std::int64_t,
                                const void *, std::int64_t, void *) noexcept>,
        "cblas_abi: ILP64 zdotu_sub prototype drifted");

    // Symbol-link controls: materializing each address in a constant
    // initializer gives every ODR-using TU a relocation for the symbol —
    // the numerics backend links Accelerate, so a missing export surfaces
    // at link time rather than as a surprise.
    [[maybe_unused]] inline constexpr auto cblas_link_ddot = &cblas_ddot;
    [[maybe_unused]] inline constexpr auto cblas_link_dgemv = &cblas_dgemv;
    [[maybe_unused]] inline constexpr auto cblas_link_dgemm = &cblas_dgemm;
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
        return 0. + cblas_ddot(n, x, incx, y, incy);
    }

    double ddot_raw(int n, const double *x, int incx,
                    const double *y, int incy) noexcept {
        return backend_ddot_ilp64(static_cast<std::int64_t>(n), x,
                                  static_cast<std::int64_t>(incx), y,
                                  static_cast<std::int64_t>(incy));
    }

    void zdotu_sub(int n, const void *x, int incx, const void *y, int incy,
                   void *out) noexcept {
        backend_zdotu_sub(static_cast<std::int64_t>(n), x,
                          static_cast<std::int64_t>(incx), y,
                          static_cast<std::int64_t>(incy), out);
    }

    void dgemv(Order order, Transpose trans, int m, int n,
               double alpha, const double *a, int lda,
               const double *x, int incx, double beta, double *y,
               int incy) noexcept {
        cblas_dgemv(static_cast<int>(order), static_cast<int>(trans), m, n,
                    alpha, a, lda, x, incx, beta, y, incy);
    }

    void dgemm(Order order, Transpose transa, Transpose transb,
               int m, int n, int k, double alpha, const double *a,
               int lda, const double *b, int ldb, double beta,
               double *c, int ldc) noexcept {
        cblas_dgemm(static_cast<int>(order), static_cast<int>(transa),
                    static_cast<int>(transb), m, n, k, alpha, a, lda, b,
                    ldb, beta, c, ldc);
    }
} // namespace blas

#endif // __APPLE__ — the Darwin backend body
