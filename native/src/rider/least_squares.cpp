// least_squares.cpp — Accelerate ILP64 DGELSD driver.
//
// The entry point is declared with an asm label rather than including
// <Accelerate/Accelerate.h>: the SDK header marks the LAPACK surface
// deprecated (a hard error under -Werror) and its vecLib headers do not
// parse under the gcc frontends sweep. The pinned symbol is exactly the
// undefined import numpy's _umath_linalg carries for its lstsq gufunc —
// nm -u shows `_dgelsd$NEWLAPACK$ILP64` on both binaries.
//
// Buffer/order contract mirrored from numpy/linalg/umath_linalg.cpp
// (init_gelsd + the lstsq inner loop):
//   LDA = max(1, m), LDB = max(1, m, n); A is m*n column-major, B is
//   ldb*nrhs column-major (pad rows are pure output scratch — dgelsd
//   overwrites them — but we zero them so no stale bits reach the
//   library); one lwork = -1 workspace query, then the real call with
//   LWORK == the queried optimum — never the buffer capacity, since a
//   larger lwork lets LAPACK pick a different blocking and change
//   rounding;
//   outputs read straight from the solved buffers: x = B rows [0, n) per
//   column, s = S[0:min(m,n)], rank = RANK;
//   residuals: when m >= n and rank == n the gufunc stores abs2 over the
//   solved tail rows [n, m) of each B column, accumulated ascending;
//   np.linalg.lstsq's wrapper then keeps them iff rank == n and m > n —
//   folded into the returned residuals vector;
//   m == 0: x is forced to zero afterwards (numpy's `x[...] = 0`);
//   failures: an FE_INVALID flag already set on entry, or a nonzero info
//   from either DGELSD call, raises the flag and throws — the gufunc's
//   get_fp_invalid_and_clear / set_fp_invalid_or_clear discipline.
#include "least_squares.hpp"

#include "../model_access.hpp"

#include <algorithm>
#include <cfenv>
#include <cmath>
#include <cstddef>
#include <limits>
#include <ranges>
#include <string>
#include <string_view>
#include <type_traits>

namespace {
    // The ILP64 ABI numpy links: __LAPACK_int is `long` (signed 64-bit)
    // under ACCELERATE_LAPACK_ILP64, and every Fortran argument arrives
    // by reference. rider::lapack_int spells that width; the
    // static_assert pins the declaration against the exported symbol's
    // signature so a drifted prototype fails at compile time.
    extern "C" void dgelsd_ilp64(const rider::lapack_int *m,
                                 const rider::lapack_int *n,
                                 const rider::lapack_int *nrhs, double *a,
                                 const rider::lapack_int *lda, double *b,
                                 const rider::lapack_int *ldb, double *s,
                                 const double *rcond, rider::lapack_int *rank,
                                 double *work, const rider::lapack_int *lwork,
                                 rider::lapack_int *iwork,
                                 rider::lapack_int *info)
        __asm__("_dgelsd$NEWLAPACK$ILP64");

    static_assert(sizeof(long) == 8,
                  "dgelsd ILP64 requires a 64-bit long (__LAPACK_int)");
    static_assert(
        std::is_same_v<
            decltype(&dgelsd_ilp64),
            void (*)(const rider::lapack_int *, const rider::lapack_int *,
                     const rider::lapack_int *, double *,
                     const rider::lapack_int *, double *,
                     const rider::lapack_int *, double *, const double *,
                     rider::lapack_int *, double *,
                     const rider::lapack_int *, rider::lapack_int *,
                     rider::lapack_int *)>,
        "dgelsd ILP64 prototype drifted");

    // numpy's message verbatim — the oracle raises
    // LinAlgError("SVD did not converge in Linear Least Squares") for any
    // nonzero info and for the pre-existing-invalid-flag path alike.
    constexpr std::string_view kConvergeMessage =
        "SVD did not converge in Linear Least Squares";

    [[noreturn]] void raise_lapack_failure() {
        // set_fp_invalid_or_clear(error_occurred): raise the flag the
        // way the gufunc does before numpy's errstate callback fires —
        // the flag stays set so the surrounding numpy state sees the
        // same floating-point aftermath as a failed lstsq.
        std::feraiseexcept(FE_INVALID);
        throw rider::LeastSquaresError(std::string(kConvergeMessage));
    }

    void require_dim(rider::lapack_int value, rider::lapack_int maximum,
                     const char *name) {
        if (value < 0 || value > maximum)
            throw std::invalid_argument(
                std::string(name) +
                ": least-squares dimension out of range");
    }
} // namespace

namespace rider {

    LeastSquaresError::~LeastSquaresError() = default;

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) ordered (rows, columns, rhs) capacity triple
    LeastSquaresWorkspace::LeastSquaresWorkspace(lapack_int max_rows,
                                                 lapack_int max_columns,
                                                 lapack_int max_rhs)
        : max_rows_(max_rows), max_columns_(max_columns),
          max_rhs_(max_rhs) {
        if (max_rows_ < 0 || max_columns_ < 0 || max_rhs_ < 1)
            throw std::invalid_argument(
                "least-squares workspace needs nonnegative row/column "
                "bounds and at least one right-hand side");
        const std::size_t unlimited =
            std::numeric_limits<std::size_t>::max();
        const std::size_t rows = static_cast<std::size_t>(max_rows_);
        const std::size_t columns = static_cast<std::size_t>(max_columns_);
        const std::size_t rhs = static_cast<std::size_t>(max_rhs_);
        a_.assign(std::max<std::size_t>(
                      1, model_access::checked_product(rows, columns,
                                                       unlimited)),
                  0.);
        // B's column stride is max(1, m, n) — bound by the declared
        // maxima (the +1 keeps a degenerate (0, 0) bound usable: the
        // buffer stays a valid one-element scratch).
        const std::size_t ldb_bound =
            std::max({std::size_t{1}, rows, columns});
        b_.assign(std::max<std::size_t>(
                      1, model_access::checked_product(ldb_bound, rhs,
                                                       unlimited)),
                  0.);
        s_.assign(
            std::max<std::size_t>(1, std::min(rows, columns)), 0.);
    }

    LeastSquaresResult
    LeastSquaresWorkspace::solve(std::span<const double> a, lapack_int m,
                                 lapack_int n, std::span<const double> b,
                                 lapack_int nrhs, double rcond) {
        require_dim(m, max_rows_, "rows");
        require_dim(n, max_columns_, "columns");
        if (nrhs < 1 || nrhs > max_rhs_)
            throw std::invalid_argument(
                "nrhs: least-squares dimension out of range");
        const std::size_t em = static_cast<std::size_t>(m);
        const std::size_t en = static_cast<std::size_t>(n);
        const std::size_t ek = static_cast<std::size_t>(nrhs);
        const std::size_t unlimited =
            std::numeric_limits<std::size_t>::max();
        if (a.size() != model_access::checked_product(em, en, unlimited) ||
            b.size() != model_access::checked_product(em, ek, unlimited))
            throw std::invalid_argument("Incompatible dimensions");

        const std::size_t lda = std::max<std::size_t>(1, em);
        const std::size_t ldb = std::max({std::size_t{1}, em, en});
        // Row-major input -> column-major staging — pure permutation, so
        // loop order is arithmetically irrelevant.
        for (std::size_t j = 0; j < en; ++j)
            for (std::size_t i = 0; i < em; ++i)
                a_[i + j * lda] = a[i * en + j];
        for (std::size_t j = 0; j < ek; ++j) {
            for (std::size_t i = 0; i < em; ++i)
                b_[i + j * ldb] = b[i * ek + j];
            for (std::size_t i = em; i < ldb; ++i) b_[i + j * ldb] = 0.;
        }

        // The gufunc captures+clears FP flags BEFORE init (the query
        // call included) and either re-raises invalid on failure or
        // clears everything on success.
        const bool prior_invalid = std::fetestexcept(FE_INVALID) != 0;
        std::feclearexcept(FE_ALL_EXCEPT);

        const lapack_int lda_l = static_cast<lapack_int>(lda);
        const lapack_int ldb_l = static_cast<lapack_int>(ldb);
        lapack_int info = 0;
        lapack_int rank = 0;

        // init_gelsd's query: LWORK == -1 writes the optimal work and
        // iwork sizes into the first elements.
        const lapack_int query_lwork = -1;
        double work_query = 0.;
        lapack_int iwork_query = 0;
        dgelsd_ilp64(&m, &n, &nrhs, a_.data(), &lda_l, b_.data(), &ldb_l,
                     s_.data(), &rcond, &rank, &work_query, &query_lwork,
                     &iwork_query, &info);
        if (info != 0) raise_lapack_failure();
        // numpy truncates the queried sizes verbatim (fortran_int
        // work_count = (fortran_int)work_size_query) and hands them back
        // — a degenerate m/n == 0 query legitimately answers a smaller
        // lwork/iwork. Only an unrepresentable cast is rejected here
        // (the gufunc's (fortran_int) cast is undefined past int64);
        // a representable-but-broken answer still reaches DGELSD below
        // and reports through info, matching the oracle's failure
        // channel instead of inventing a second error kind.
        if (!std::isfinite(work_query) ||
            work_query >= 4.611686018427388e18 || // 2^62, far below int64 UB
            work_query <= -4.611686018427388e18)
            throw std::runtime_error(
                "dgelsd workspace query returned invalid sizes");
        const lapack_int needed_work =
            static_cast<lapack_int>(work_query);
        if (needed_work > 0 &&
            work_.size() < static_cast<std::size_t>(needed_work))
            work_.resize(static_cast<std::size_t>(needed_work));
        if (iwork_query > 0 &&
            iwork_.size() < static_cast<std::size_t>(iwork_query))
            iwork_.resize(static_cast<std::size_t>(iwork_query));
        // Keep at least one element so .data() is a valid Fortran
        // argument even when the queried sizes are degenerate.
        if (work_.empty()) work_.resize(1);
        if (iwork_.empty()) iwork_.resize(1);

        // LWORK is the queried optimum — not work_.size() — so LAPACK
        // sees numpy's exact argument tuple. A query answer below the
        // real minimum lands in INFO = -12 through DGELSD's own
        // argument check, like the oracle's.
        const lapack_int lwork = needed_work;
        dgelsd_ilp64(&m, &n, &nrhs, a_.data(), &lda_l, b_.data(), &ldb_l,
                     s_.data(), &rcond, &rank, work_.data(), &lwork,
                     iwork_.data(), &info);
        if (info != 0 || prior_invalid) raise_lapack_failure();

        LeastSquaresResult result;
        result.rank = rank;
        result.singular_values.assign(
            s_.begin(), s_.begin() + static_cast<std::ptrdiff_t>(
                                         std::min(em, en)));

        // x = first n rows of each solved B column, emitted row-major
        // (n, nrhs). numpy forces x to zero when m == 0; our B staging
        // already writes zeros into every pad row, so the copy below
        // produces the same zeroed solution outright.
        result.solution.assign(
            model_access::checked_product(en, ek, unlimited), 0.);
        for (std::size_t j = 0; j < ek; ++j)
            for (std::size_t i = 0; i < en; ++i)
                result.solution[i * ek + j] = b_[i + j * ldb];

        // gufunc residual: abs2 over the solved tail rows [n, m) per
        // column, ascending — then the wrapper keeps it only for an
        // overdetermined full-rank solve. numpy's binary was compiled
        // with FP contraction enabled, so its `res += el*el` lowers to
        // fmadd — a single rounding. std::fma pins that same semantics
        // even under this port's -ffp-contract=off.
        if (rank == n && m > n) {
            result.residuals.assign(ek, 0.);
            for (std::size_t j = 0; j < ek; ++j) {
                double sum = 0.;
                for (std::size_t i = en; i < em; ++i)
                    sum = std::fma(b_[i + j * ldb], b_[i + j * ldb], sum);
                result.residuals[j] = sum;
            }
        }

        std::feclearexcept(FE_ALL_EXCEPT);
        return result;
    }
} // namespace rider
