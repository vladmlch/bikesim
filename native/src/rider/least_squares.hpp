// least_squares.hpp — reusable Accelerate ILP64 DGELSD workspace.
//
// numpy.linalg.lstsq's gufunc (numpy/linalg/umath_linalg.cpp, lstsq_m /
// lstsq_n over the 'ddd->ddid' signature) drives DGELSD through the
// Accelerate $NEWLAPACK$ILP64 ABI on this platform. This workspace
// reproduces that contract bitwise: the same exported entry point, the
// same column-major buffer layouts (LDA = max(1, m), LDB = max(1, m, n)),
// the same lwork/iwork workspace query, and the same outputs —
// solution, effective rank, singular values, residual sums.
//
// Error contract, mirroring the oracle exactly:
//   * dimension/shape violations -> std::invalid_argument before any
//     copy ("Incompatible dimensions" / "N-dimensional array given.
//     Array must be two-dimensional" — numpy's own messages);
//   * DGELSD info != 0 -> LeastSquaresError, registered under ValueError
//     with numpy's "SVD did not converge in Linear Least Squares"
//     message. An FE_INVALID flag already set on entry is captured and
//     re-raised at the end like the gufunc's
//     get_fp_invalid_and_clear/set_fp_invalid_or_clear discipline — it
//     never fails the call: in the oracle it is error_occurred's seed,
//     which only controls flag re-raising, not the result;
//   * nonfinite inputs are NOT pre-screened — they flow into DGELSD
//     exactly as numpy's gufunc does, so inf-in-b solves that "succeed"
//     return the identical nonfinite result and non-convergent inputs
//     raise the same error. recover_wrench-style finiteness gates belong
//     to the caller, as in attachment_wrench.py.
//
// Rank is reported, never hidden: T2b's wrench recovery requires full
// column rank, while a body/spatial-wrench reconstruction may accept a
// minimum-norm underdetermined solve after its own residual checks —
// the workspace leaves that policy to the caller, like np.linalg.lstsq.
#pragma once

#include <cstdint>
#include <span>
#include <stdexcept>
#include <vector>

namespace rider {

    // Accelerate's ILP64 Fortran integer (__LAPACK_int == long under
    // ACCELERATE_LAPACK_ILP64 — signed 64-bit on this platform, pinned at
    // the call-site declaration in least_squares.cpp).
    using lapack_int = std::int64_t;

    // One DGELSD solve's numpy.linalg.lstsq output tuple.
    struct LeastSquaresResult {
        // x — row-major (n, nrhs): solution[i * nrhs + j] is component i
        // of the answer for right-hand side j (numpy returns the n-vector
        // for a vector b and the (n, k) matrix for a 2-D b).
        std::vector<double> solution;
        // lstsq's `resids` contract: empty unless the problem is
        // overdetermined at full column rank (rank == n and m > n). When
        // nonempty, residuals[j] is the squared two-norm sum over the
        // solved right-hand-side tail rows n..m-1 — accumulated in the
        // gufunc's ascending abs2 order.
        std::vector<double> residuals;
        // Effective rank at the caller's rcond cutoff.
        lapack_int rank = 0;
        // min(m, n) singular values.
        std::vector<double> singular_values;
    };

    // numpy.linalg.LinAlgError's counterpart — a ValueError on the Python
    // side. DGELSD reporting nonzero info lands here.
    class LeastSquaresError final : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;

        ~LeastSquaresError() override; // key function: anchors the vtable
        LeastSquaresError(const LeastSquaresError &) = default;
        LeastSquaresError &operator=(const LeastSquaresError &) = default;
        LeastSquaresError(LeastSquaresError &&) = default;
        LeastSquaresError &operator=(LeastSquaresError &&) = default;
    };

    // Owning driver with reusable buffers: construct once per problem
    // bound (e.g. (nv, nv, 1) for attachment wrench recovery), then solve
    // repeatedly — dgelsd's column-major staging areas are sized at
    // construction and the LAPACK work/iwork arenas grow only as far as
    // the queried optima demand. Solve dims beyond the maxima are a
    // caller error, never a silent reallocation.
    class LeastSquaresWorkspace {
    public:
        // np.linalg.lstsq's pinned cutoff for this port
        // (attachment_wrench.py's `rcond=1e-12`).
        static constexpr double kDefaultRcond = 1e-12;

        LeastSquaresWorkspace(lapack_int max_rows, lapack_int max_columns,
                              lapack_int max_rhs);

        // Solve min_x ||Ax - b||_2 for `nrhs` right-hand sides.
        //   a: row-major (m, n) matrix — a.size() == m*n
        //   b: row-major (m, nrhs) — b.size() == m*nrhs (a vector target
        //      is nrhs == 1; numpy's n_rhs == 0 padding quirk is handled
        //      by the binding, not here: nrhs >= 1 is required).
        // Every argument is validated before a byte is staged. `rcond`
        // is passed through to DGELSD untouched — like numpy — so the
        // same cutoffs (including -1 / 0 / degenerate values) reproduce.
        [[nodiscard]] LeastSquaresResult
        solve(std::span<const double> a, lapack_int m, lapack_int n,
              std::span<const double> b, lapack_int nrhs,
              double rcond = kDefaultRcond);

    private:
        lapack_int max_rows_;
        lapack_int max_columns_;
        lapack_int max_rhs_;
        // Column-major staging: a_ is lda*max_columns with lda =
        // max(1, m) per solve; b_ is ldb*max_rhs with ldb =
        // max(1, m, n); s_ holds min(m, n) singular values.
        std::vector<double> a_;
        std::vector<double> b_;
        std::vector<double> s_;
        std::vector<double> work_;
        std::vector<lapack_int> iwork_;
    };
} // namespace rider
