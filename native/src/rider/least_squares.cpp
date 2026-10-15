// least_squares.cpp — platform ILP64 DGELSD driver.
//
// The foreign entry point lives behind numeric_backend::dgelsd
// (src/platform/lapack.hpp) — Accelerate's dgelsd$NEWLAPACK$ILP64 on
// Darwin, the wheel's scipy_dgelsd_64_ on Linux — declared in exactly
// one selected backend TU per platform. That symbol is exactly the
// undefined import numpy's _umath_linalg carries for its lstsq gufunc
// on each platform.
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
//   failures: a nonzero info from either DGELSD call raises the flag and
//   throws — the gufunc's get_fp_invalid_and_clear /
//   set_fp_invalid_or_clear discipline. An FE_INVALID flag set on entry
//   is error_occurred's seed: it is re-raised at the end like the gufunc
//   preserves it, but never fails the call — ambient flags (Accelerate
//   BLAS leaves them raised) are not solver failures.
#include "least_squares.hpp"

#include "../model_access.hpp"
#include "../platform/lapack.hpp"

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
    // numeric_backend::integer is the qualified ILP64 Fortran integer;
    // rider::lapack_int spells the same width, so every call below is a
    // verbatim forward — no casts of any kind reach the ABI.
    static_assert(
        std::is_same_v<rider::lapack_int, numeric_backend::integer>,
        "lapack_int must match the qualified numeric_backend integer width");

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
        return solve_impl(a, m, n, b, nrhs, rcond,
                          MatrixLayout::row_major);
    }

    LeastSquaresResult
    LeastSquaresWorkspace::solve_column_major(
        std::span<const double> a, lapack_int m, lapack_int n,
        std::span<const double> b, lapack_int nrhs, double rcond) {
        return solve_impl(a, m, n, b, nrhs, rcond,
                          MatrixLayout::column_major);
    }

    LeastSquaresResult
    LeastSquaresWorkspace::solve_impl(std::span<const double> a,
                                      lapack_int m, lapack_int n,
                                      std::span<const double> b,
                                      lapack_int nrhs, double rcond,
                                      MatrixLayout layout) {
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
        if (layout == MatrixLayout::column_major) {
            // a already carries lda-strided Fortran order — staging is
            // a plain copy of the same m*n bytes the permutation below
            // would produce.
            std::ranges::copy(a, a_.begin());
        } else {
            // Row-major input -> column-major staging — pure
            // permutation, so loop order is arithmetically irrelevant.
            for (std::size_t j = 0; j < en; ++j)
                for (std::size_t i = 0; i < em; ++i)
                    a_[i + j * lda] = a[i * en + j];
        }
        for (std::size_t j = 0; j < ek; ++j) {
            for (std::size_t i = 0; i < em; ++i)
                b_[i + j * ldb] = b[i * ek + j];
            for (std::size_t i = em; i < ldb; ++i) b_[i + j * ldb] = 0.;
        }

        // The gufunc captures+clears FP flags BEFORE init (the query
        // call included); the entry flag is error_occurred's initial
        // value in umath_linalg.cpp — it only decides whether FE_INVALID
        // is re-raised at the end, never whether the call fails. Ambient
        // flags (e.g. left by Accelerate BLAS internals during model
        // setup) must not turn a well-posed solve into a failure.
        const bool prior_invalid = std::fetestexcept(FE_INVALID) != 0;
        std::feclearexcept(FE_ALL_EXCEPT);

        const lapack_int lda_l = static_cast<lapack_int>(lda);
        const lapack_int ldb_l = static_cast<lapack_int>(ldb);
        lapack_int info = 0;
        lapack_int rank = 0;

        // init_gelsd's query: LWORK == -1 writes the optimal work and
        // iwork sizes into the first elements. The answer depends only
        // on (m, n, nrhs) — lda/ldb are derived from them and matrix
        // values never enter it — so a remembered answer for the shape
        // replays the identical probe result; only unseen shapes pay
        // the extra call.
        lapack_int needed_work = 0;
        lapack_int needed_iwork = 0;
        const auto remembered = std::ranges::find_if(
            shape_work_, [&](const ShapeWork &entry) {
                return entry.m == m && entry.n == n && entry.nrhs == nrhs;
            });
        if (remembered != shape_work_.end()) {
            needed_work = remembered->lwork;
            needed_iwork = remembered->iwork;
        } else {
            const lapack_int query_lwork = -1;
            double work_query = 0.;
            lapack_int iwork_query = 0;
            numeric_backend::dgelsd(&m, &n, &nrhs, a_.data(), &lda_l,
                                    b_.data(), &ldb_l, s_.data(), &rcond,
                                    &rank, &work_query, &query_lwork,
                                    &iwork_query, &info);
            if (info != 0) raise_lapack_failure();
            // numpy truncates the queried sizes verbatim (fortran_int
            // work_count = (fortran_int)work_size_query) and hands them
            // back — a degenerate m/n == 0 query legitimately answers a
            // smaller lwork/iwork. Only an unrepresentable cast is
            // rejected here (the gufunc's (fortran_int) cast is
            // undefined past int64); a representable-but-broken answer
            // still reaches DGELSD below and reports through info,
            // matching the oracle's failure channel instead of
            // inventing a second error kind.
            if (!std::isfinite(work_query) ||
                work_query >= 4.611686018427388e18 || // 2^62, far below int64 UB
                work_query <= -4.611686018427388e18)
                throw std::runtime_error(
                    "dgelsd workspace query returned invalid sizes");
            needed_work = static_cast<lapack_int>(work_query);
            needed_iwork = iwork_query;
            shape_work_.push_back(
                {.m = m, .n = n, .nrhs = nrhs, .lwork = needed_work,
                 .iwork = needed_iwork});
        }
        if (needed_work > 0 &&
            work_.size() < static_cast<std::size_t>(needed_work))
            work_.resize(static_cast<std::size_t>(needed_work));
        if (needed_iwork > 0 &&
            iwork_.size() < static_cast<std::size_t>(needed_iwork))
            iwork_.resize(static_cast<std::size_t>(needed_iwork));
        // Keep at least one element so .data() is a valid Fortran
        // argument even when the queried sizes are degenerate.
        if (work_.empty()) work_.resize(1);
        if (iwork_.empty()) iwork_.resize(1);

        // LWORK is the queried optimum — not work_.size() — so LAPACK
        // sees numpy's exact argument tuple. A query answer below the
        // real minimum lands in INFO = -12 through DGELSD's own
        // argument check, like the oracle's.
        const lapack_int lwork = needed_work;
        numeric_backend::dgelsd(&m, &n, &nrhs, a_.data(), &lda_l,
                                b_.data(), &ldb_l, s_.data(), &rcond,
                                &rank, work_.data(), &lwork,
                                iwork_.data(), &info);
        // Only THIS call's info decides failure — like the oracle's
        // not_ok from call_gelsd; the entry flag is never an error.
        if (info != 0) raise_lapack_failure();

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

        // set_fp_invalid_or_clear(error_occurred): success preserves an
        // entry-invalid flag (re-raised like numpy's flag discipline);
        // a clean entry clears everything raised by the LAPACK calls.
        if (prior_invalid) std::feraiseexcept(FE_INVALID);
        else std::feclearexcept(FE_ALL_EXCEPT);
        return result;
    }
} // namespace rider
