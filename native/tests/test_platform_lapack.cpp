// test_platform_lapack.cpp — direct C++ contracts for the typed platform
// LAPACK adapter (src/platform/lapack.hpp). Exactly one backend TU
// implements numeric_backend::dgelsd: lapack_accelerate.cpp on Darwin
// (Accelerate's dgelsd$NEWLAPACK$ILP64) or lapack_numpy_openblas.cpp on
// Linux (the wheel's scipy_dgelsd_64_ export through the generated asm
// label). Every integer here is the shared 64-bit numeric_backend::
// integer ABI — the bitwise np.linalg.lstsq oracle lives in
// tests/reference/test_native_attachment_wrench.py via
// _rider_least_squares; this file pins the adapter call contract
// standalone, like test_platform_blas.cpp does for CBLAS.
//
// The caller rule exercised is numpy's init_gelsd/call_gelsd pair
// (numpy/linalg/umath_linalg.cpp): one LWORK = -1 workspace query writes
// the optimal sizes into WORK[0] and IWORK[0]; the caller truncates
// WORK[0] to an integer verbatim, allocates exactly that many elements
// (no rounding up — a different lwork lets LAPACK pick a different
// blocking and change rounding), then issues the real call with
// LWORK == the queried optimum.
#include "../src/platform/lapack.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
    using numeric_backend::integer;
    using TestFunction = void (*)();

    struct TestCase {
        std::string_view name;
        TestFunction run;
    };

    void require(bool condition, std::string_view message) {
        if (!condition)
            throw std::runtime_error(std::string(message));
    }

    // One dgelsd problem in its Fortran argument staging: column-major a
    // (lda * n elements) and b (ldb * nrhs elements), s at least
    // min(m, n). Members default to numpy's maxima conventions.
    struct Problem {
        integer m = 0;
        integer n = 0;
        integer nrhs = 0;
        integer lda = 1;
        integer ldb = 1;
        double rcond = -1.;
        std::vector<double> a;
        std::vector<double> b;
        std::vector<double> s;
    };

    // The LWORK = -1 workspace query with numpy's scalar slots: WORK[0]
    // receives the optimal lwork as a whole-valued double, IWORK[0] the
    // iwork element count. Returns {lwork, iwork}; nonzero info throws —
    // the query call has its own info channel, exactly like the oracle.
    std::pair<integer, integer> query_workspace(Problem &p) {
        integer lwork = -1;
        integer iwork = 0;
        integer info = 0;
        integer rank = 0;
        double work_size = 0.;
        numeric_backend::dgelsd(&p.m, &p.n, &p.nrhs, p.a.data(), &p.lda,
                                p.b.data(), &p.ldb, p.s.data(), &p.rcond,
                                &rank, &work_size, &lwork, &iwork, &info);
        if (info != 0)
            throw std::runtime_error("dgelsd workspace query reported "
                                     "info != 0");
        require(std::isfinite(work_size) &&
                    work_size == std::trunc(work_size),
                "dgelsd query must return a finite whole-valued lwork");
        return {static_cast<integer>(work_size), iwork};
    }

    // A complete numpy-caller-rule solve: query, allocate exactly the
    // returned element counts, then the real call with LWORK == the
    // queried optimum. The buffers hold enough b storage for
    // ldb * nrhs. `shrink` subtracts from the queried lwork to drive the
    // argument-error path without inventing sizes.
    struct SolveOutcome {
        integer info;
        integer rank;
    };

    SolveOutcome solve(Problem &p, integer shrink = 0) {
        const auto [want_work, want_iwork] = query_workspace(p);
        require(want_work > 0 && want_iwork > 0,
                "dgelsd query must return positive work sizes");
        integer lwork = want_work - shrink;
        require(lwork >= 1, "test driver shrunk lwork below 1");
        std::vector<double> work(static_cast<std::size_t>(want_work));
        std::vector<integer> iwork(static_cast<std::size_t>(want_iwork));
        SolveOutcome outcome{.info = 0, .rank = 0};
        numeric_backend::dgelsd(&p.m, &p.n, &p.nrhs, p.a.data(), &p.lda,
                                p.b.data(), &p.ldb, p.s.data(), &p.rcond,
                                &outcome.rank, work.data(), &lwork,
                                iwork.data(), &outcome.info);
        return outcome;
    }

    Problem make_3x2_rank_one() {
        // The T06 ABI-probe case: A columns (1,2,3) and (2,4,6) — column
        // 2 is exactly 2 * column 1, so the matrix is rank 1; b = 3 *
        // column 1 gives the minimum-norm solution x = (0.6, 1.2).
        Problem p;
        p.m = 3;
        p.n = 2;
        p.nrhs = 1;
        p.lda = 3;
        p.ldb = 3;
        p.a.assign({1., 2., 3., 2., 4., 6.});
        p.b.assign({3., 6., 9.});
        p.s.assign(2, 0.);
        return p;
    }

    // Workspace query: the provider must answer the LWORK = -1 probe
    // with positive, whole-valued sizes through WORK[0]/IWORK[0], and a
    // repeated same-shape query must return the identical answer — the
    // premise of the (m, n, nrhs) query cache in
    // LeastSquaresWorkspace.
    void test_dgelsd_workspace_query() {
        Problem p = make_3x2_rank_one();
        const auto [work1, iwork1] = query_workspace(p);
        const auto [work2, iwork2] = query_workspace(p);
        require(work1 == work2 && iwork1 == iwork2,
                "a same-shape dgelsd query must return identical sizes");
        // Both qualified providers answer this shape with lwork 802 /
        // iwork 22 today; the contract is only positivity + integrality
        // + repeatability (sizes are a provider version detail; bitwise
        // result parity is the oracle tests' job).
        require(work1 >= 1 && iwork1 >= 1,
                "dgelsd query sizes must be positive");
    }

    // Degenerate shapes still get positive, usable answers — the
    // workspace's empty-safety relies on a real queried size, never a
    // synthesized one.
    void test_dgelsd_degenerate_query() {
        Problem p;
        p.nrhs = 1;
        p.a.assign(1, 0.);
        p.b.assign(1, 0.);
        p.s.assign(1, 0.);
        const auto [work, iwork] = query_workspace(p);
        require(work >= 1 && iwork >= 1,
                "a (0, 0) dgelsd query must still return usable sizes");
        const SolveOutcome outcome = solve(p);
        require(outcome.info == 0,
                "a (0, 0) dgelsd solve must report info == 0");
        require(outcome.rank == 0,
                "a (0, 0) dgelsd solve must report rank 0");
    }

    // The query result drives a real solve end-to-end: the rank-1 3x2
    // problem reports rank 1 and deposits x = (0.6, 1.2) into the first
    // n rows of B. Solution digits are a provider numerics detail — the
    // tolerance is the ABI probe's, not a claim of cross-provider
    // bitwise parity (each platform is bitwise-matched to its own
    // numpy oracle by the reference tests).
    void test_dgelsd_solve() {
        Problem p = make_3x2_rank_one();
        const SolveOutcome outcome = solve(p);
        require(outcome.info == 0,
                "dgelsd solve must report info == 0");
        require(outcome.rank == 1,
                "dgelsd must report the exact rank-1 defect");
        require(std::fabs(p.b[0] - 0.6) < 1e-9 &&
                    std::fabs(p.b[1] - 1.2) < 1e-9,
                "dgelsd solution rows must hold x = (0.6, 1.2)");
        require(p.s[0] > 8. && p.s[0] < 9. && p.s[1] < 1e-12,
                "singular values must expose the rank-1 spectrum");
    }

    // Underdetermined multi-RHS: LDB = max(1, m, n) = n means B columns
    // carry pad rows. A = [[1,0,0],[0,1,0]] against columns (1,2) and
    // (3,4) has the exact minimum-norm answers [1,2,0] and [3,4,0] on
    // both qualified providers (verified bit-exact — integer data keeps
    // every intermediate exactly representable).
    void test_dgelsd_underdetermined_multi_rhs() {
        Problem p;
        p.m = 2;
        p.n = 3;
        p.nrhs = 2;
        p.lda = 2;
        p.ldb = 3;
        p.a.assign({1., 0., 0., 1., 0., 0.});
        p.b.assign({1., 2., 0., 3., 4., 0.});
        p.s.assign(2, 0.);
        const SolveOutcome outcome = solve(p);
        require(outcome.info == 0, "underdetermined dgelsd must succeed");
        require(outcome.rank == 2,
                "a full-row-rank wide A must report rank 2");
        require(p.b[0] == 1. && p.b[1] == 2. && p.b[2] == 0. &&
                    p.b[3] == 3. && p.b[4] == 4. && p.b[5] == 0.,
                "min-norm columns must be exact [1,2,0] and [3,4,0]");
        require(p.s[0] == 1. && p.s[1] == 1.,
                "the spectrum of [[1,0,0],[0,1,0]] is [1, 1]");
    }

    // Valid-retry: handing LWORK < the queried optimum lands in the
    // provider's own argument check (INFO = -12) — the same channel a
    // too-small buffer reports through in the oracle. The failure must
    // not poison the provider: a correctly provisioned retry on the
    // same problem still converges with the right answer.
    void test_dgelsd_valid_retry() {
        Problem p = make_3x2_rank_one();
        const SolveOutcome failed = solve(p, /*shrink=*/1);
        require(failed.info == -12,
                "a short lwork must report INFO = -12 (argument check)");
        Problem retry = make_3x2_rank_one();
        const SolveOutcome retried = solve(retry);
        require(retried.info == 0 && retried.rank == 1,
                "a valid solve must still succeed after the info error");
        require(std::fabs(retry.b[0] - 0.6) < 1e-9 &&
                    std::fabs(retry.b[1] - 1.2) < 1e-9,
                "the retry must produce the identical solution");
    }

    // Argument errors report through info — negative dims land in
    // INFO = -1 without a crash, abort, or nonlocal exit.
    void test_dgelsd_bad_arg_reports_info() {
        Problem p;
        p.m = -1;
        p.n = 2;
        p.nrhs = 1;
        p.lda = 2;
        p.ldb = 2;
        p.a.assign(4, 1.);
        p.b.assign(2, 1.);
        p.s.assign(2, 0.);
        integer info = 0, rank = 0;
        integer lwork = 64;
        std::array<double, 64> work{};
        std::array<integer, 32> iwork{};
        numeric_backend::dgelsd(&p.m, &p.n, &p.nrhs, p.a.data(), &p.lda,
                                p.b.data(), &p.ldb, p.s.data(), &p.rcond,
                                &rank, work.data(), &lwork,
                                iwork.data(), &info);
        require(info == -1,
                "a negative m must report INFO = -1, not terminate");
    }

    constexpr std::array<TestCase, 6> cases{{
        {.name = "dgelsd_workspace_query", .run = test_dgelsd_workspace_query},
        {.name = "dgelsd_degenerate_query", .run = test_dgelsd_degenerate_query},
        {.name = "dgelsd_solve", .run = test_dgelsd_solve},
        {.name = "dgelsd_underdetermined_multi_rhs",
         .run = test_dgelsd_underdetermined_multi_rhs},
        {.name = "dgelsd_valid_retry", .run = test_dgelsd_valid_retry},
        {.name = "dgelsd_bad_arg_reports_info",
         .run = test_dgelsd_bad_arg_reports_info},
    }};

    int run_case(const TestCase &test_case) {
        try {
            test_case.run();
            std::cout << "PASS " << test_case.name << '\n';
            return 0;
        } catch (const std::exception &error) {
            std::cerr << "FAIL " << test_case.name << ": " << error.what()
                      << '\n';
            return 1;
        }
    }
} // namespace

int main(int argc, char **argv) {
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    // The C process-entry contract provides argc pointers before the
    // sentinel.
    const std::span<char *> arguments(argv, static_cast<std::size_t>(argc));
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
    if (arguments.size() == 2 &&
        std::string_view(arguments[1]) == "--list") {
        for (const TestCase &test_case: cases)
            std::cout << test_case.name << '\n';
        return 0;
    }
    if (arguments.size() > 2) {
        std::cerr << "usage: native_platform_lapack_tests [case-name|--list]\n";
        return 2;
    }

    const std::string_view requested =
        arguments.size() == 1 ? "all" : std::string_view(arguments[1]);
    bool found = requested == "all";
    int status = 0;
    for (const TestCase &test_case: cases) {
        if (requested == "all" || requested == test_case.name) {
            found = true;
            if (run_case(test_case) != 0)
                status = 1;
        }
    }
    if (!found) {
        std::cerr << "unknown native platform-lapack case: " << requested
                  << '\n';
        return 2;
    }
    return status;
}
