// test_platform_blas.cpp — direct C++ contracts for the typed platform
// CBLAS adapters (src/cblas_abi.hpp). Exactly one backend TU implements
// the wrappers: src/platform/blas_accelerate.cpp on Darwin (verbatim
// legacy 32-bit cblas_* entry points) or src/platform/blas_numpy_openblas.cpp
// on Linux (scipy-openblas ILP64 exports through generated asm labels).
// The backend integer width never escapes blas:: — every argument here is
// a 32-bit int by the wrapper contract. The bitwise gate against the
// numpy oracle lives in tests/reference/test_native_platform_blas.py via
// the _debug_platform_blas binding; this file pins the adapter call
// contract standalone, like test_station.cpp does for the station law.
#include "../src/cblas_abi.hpp"

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
    using TestFunction = void (*)();

    struct TestCase {
        std::string_view name;
        TestFunction run;
    };

    void require(bool condition, std::string_view message) {
        if (!condition)
            throw std::runtime_error(std::string(message));
    }

    // dot = 11 for the brief's [1,2]·[3,4] case plus the probe's 32 for
    // [1,2,3]·[4,5,6]. Small integers keep the expectations exact in
    // binary64 on every backend.
    void test_ddot_values() {
        const std::array<double, 2> a2{1., 2.};
        const std::array<double, 2> b2{3., 4.};
        require(blas::ddot(2, a2.data(), 1, b2.data(), 1) == 11.,
                "ddot([1,2],[3,4]) must be 11");
        const std::array<double, 3> a3{1., 2., 3.};
        const std::array<double, 3> b3{4., 5., 6.};
        require(blas::ddot(3, a3.data(), 1, b3.data(), 1) == 32.,
                "ddot([1,2,3],[4,5,6]) must be 32");
    }

    // incx/incy reach the backend unchanged: element pairs at stride 2
    // and mismatched strides both compute the dense dot of the picked
    // elements.
    void test_ddot_strides() {
        const std::array<double, 4> x{1., 99., 2., 99.};
        const std::array<double, 4> y{3., 88., 4., 88.};
        require(blas::ddot(2, x.data(), 2, y.data(), 2) == 11.,
                "ddot with incx=incy=2 picks [1,2]·[3,4]");
        const std::array<double, 6> z{3., 77., 77., 5., 77., 77.};
        require(blas::ddot(2, x.data(), 2, z.data(), 3) == 13.,
                "ddot with incx=2, incy=3 picks [1,2]·[3,5] == 1*3+2*5");
    }

    // n==0 returns +0.0 on both qualified providers — the same contract
    // test_contracts.cpp pins for the existing callers.
    void test_ddot_zero_size() {
        const std::array<double, 1> a{5.};
        const double result = blas::ddot(0, a.data(), 1, a.data(), 1);
        require(result == 0. && !std::signbit(result),
                "ddot with n=0 must return +0.0");
    }

    // Row-major NoTrans: the T06 probe's 3x4 case — rows sum to
    // {10, 26, 42}; Trans computes A^T @ x over the same storage.
    void test_dgemv_row_major() {
        const std::array<double, 12> a{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
        const std::array<double, 4> ones{1., 1., 1., 1.};
        std::array<double, 3> y{};
        blas::dgemv(blas::Order::row_major, blas::Transpose::no, 3, 4, 1.,
                    a.data(), 4, ones.data(), 1, 0., y.data(), 1);
        require(y[0] == 10. && y[1] == 26. && y[2] == 42.,
                "row-major dgemv NoTrans 3x4");
        const std::array<double, 3> x{1., 1., 1.};
        std::array<double, 4> yt{};
        blas::dgemv(blas::Order::row_major, blas::Transpose::yes, 3, 4, 1.,
                    a.data(), 4, x.data(), 1, 0., yt.data(), 1);
        require(yt[0] == 15. && yt[1] == 18. && yt[2] == 21. && yt[3] == 24.,
                "row-major dgemv Trans 3x4 column sums");
    }

    // Column-major NoTrans: the same logical 3x4 matrix stored column by
    // column — the 102 enum value reaches the provider unmapped.
    void test_dgemv_column_major() {
        const std::array<double, 12> a{1, 5, 9, 2, 6, 10, 3, 7, 11, 4, 8, 12};
        const std::array<double, 4> ones{1., 1., 1., 1.};
        std::array<double, 3> y{};
        blas::dgemv(blas::Order::column_major, blas::Transpose::no, 3, 4, 1.,
                    a.data(), 3, ones.data(), 1, 0., y.data(), 1);
        require(y[0] == 10. && y[1] == 26. && y[2] == 42.,
                "column-major dgemv NoTrans 3x4");
        const std::array<double, 3> x{1., 1., 1.};
        std::array<double, 4> yt{};
        blas::dgemv(blas::Order::column_major, blas::Transpose::yes, 3, 4, 1.,
                    a.data(), 3, x.data(), 1, 0., yt.data(), 1);
        require(yt[0] == 15. && yt[1] == 18. && yt[2] == 21. && yt[3] == 24.,
                "column-major dgemv Trans 3x4");
    }

    // incx/incy on dgemv: y elements land at stride 2 inside a wider
    // buffer and x elements are picked at stride 2.
    void test_dgemv_strides() {
        const std::array<double, 4> a{1., 2., 3., 4.};
        const std::array<double, 4> x{1., 99., 1., 99.};
        std::array<double, 4> y{7., 7., 7., 7.};
        blas::dgemv(blas::Order::row_major, blas::Transpose::no, 2, 2, 1.,
                    a.data(), 2, x.data(), 2, 0., y.data(), 2);
        require(y[0] == 3. && y[1] == 7. && y[2] == 7. && y[3] == 7.,
                "dgemv incx=2 incy=2 scatter/gather");
    }

    // Zero-dimension contract: m==0 has no y elements to write — the
    // provider must leave the buffer alone. n==0 is provider-defined:
    // the CBLAS reference quick-returns while Accelerate still applies
    // the beta scaling (verified: it writes 0*y). Both outcomes are
    // legal — the binding fills numpy's documented zeros itself — so the
    // assertion only forbids out-of-contract garbage.
    void test_dgemv_zero_size() {
        const std::array<double, 4> a{1., 2., 3., 4.};
        std::array<double, 2> y{77., 88.};
        blas::dgemv(blas::Order::row_major, blas::Transpose::no, 0, 2, 1.,
                    a.data(), 2, a.data(), 1, 0., y.data(), 1);
        require(y[0] == 77. && y[1] == 88.,
                "dgemv with m=0 must not write y");
        blas::dgemv(blas::Order::row_major, blas::Transpose::no, 2, 0, 1.,
                    a.data(), 2, a.data(), 1, 0., y.data(), 1);
        require((y[0] == 77. || y[0] == 0.) &&
                    (y[1] == 88. || y[1] == 0.),
                "dgemv with n=0 may quick-return or scale by beta");
    }

    // Row-major NoTrans·NoTrans: the T06 probe's (2x3)·(3x2) case.
    void test_dgemm_row_major_nn() {
        const std::array<double, 6> a{1., 2., 3., 4., 5., 6.};
        const std::array<double, 6> b{1., 0., 0., 1., 1., 1.};
        std::array<double, 4> c{};
        blas::dgemm(blas::Order::row_major, blas::Transpose::no,
                    blas::Transpose::no, 2, 2, 3, 1., a.data(), 3,
                    b.data(), 2, 0., c.data(), 2);
        require(c[0] == 4. && c[1] == 5. && c[2] == 10. && c[3] == 11.,
                "row-major dgemm NN (2x3)(3x2)");
    }

    // Row-major NoTrans·Trans — the `local @ R.T` call shape the writers
    // actually emit (distributed_tire.cpp, tyre/profile.cpp): B is stored
    // as the untransposed (n,k) matrix and op(B)=B^T multiplies.
    void test_dgemm_row_major_nt() {
        const std::array<double, 6> a{1., 2., 3., 4., 5., 6.};
        const std::array<double, 6> b{1., 0., 1., 0., 1., 1.};
        std::array<double, 4> c{};
        blas::dgemm(blas::Order::row_major, blas::Transpose::no,
                    blas::Transpose::yes, 2, 2, 3, 1., a.data(), 3,
                    b.data(), 3, 0., c.data(), 2);
        // op(B) = [[1,0],[0,1],[1,1]] (3x2), so A@op(B) = [[4,5],[10,11]]
        require(c[0] == 4. && c[1] == 5. && c[2] == 10. && c[3] == 11.,
                "row-major dgemm NT (2x3)(2x3)^T");
    }

    // Row-major Trans·Trans: A stored (k,m), B stored (n,k) — the layout
    // an F-contiguous numpy operand maps to.
    void test_dgemm_row_major_tt() {
        const std::array<double, 6> a{1., 4., 2., 5., 3., 6.}; // A^T: 3x2
        const std::array<double, 6> b{1., 0., 1., 0., 1., 1.}; // B^T: 2x3
        std::array<double, 4> c{};
        blas::dgemm(blas::Order::row_major, blas::Transpose::yes,
                    blas::Transpose::yes, 2, 2, 3, 1., a.data(), 2,
                    b.data(), 3, 0., c.data(), 2);
        // op(A)=[[1,2,3],[4,5,6]], op(B)=[[1,0],[0,1],[1,1]] -> [[4,5],[10,11]]
        require(c[0] == 4. && c[1] == 5. && c[2] == 10. && c[3] == 11.,
                "row-major dgemm TT");
    }

    // Column-major NoTrans·NoTrans over the same logical operands.
    void test_dgemm_column_major() {
        const std::array<double, 6> a{1., 4., 2., 5., 3., 6.}; // 2x3
        const std::array<double, 6> b{1., 0., 1., 0., 1., 1.}; // 3x2
        std::array<double, 4> c{};
        blas::dgemm(blas::Order::column_major, blas::Transpose::no,
                    blas::Transpose::no, 2, 2, 3, 1., a.data(), 2,
                    b.data(), 3, 0., c.data(), 2);
        // column-major C = [[4,10],[5,11]] stored columnwise {4,10,5,11}
        require(c[0] == 4. && c[1] == 10. && c[2] == 5. && c[3] == 11.,
                "column-major dgemm NN (2x3)(3x2)");
    }

    // alpha/beta both reach the provider: C := 2·A@B + 3·C.
    void test_dgemm_alpha_beta() {
        const std::array<double, 4> a{1., 2., 3., 4.};
        const std::array<double, 4> b{5., 6., 7., 8.};
        std::array<double, 4> c{1., 1., 1., 1.};
        blas::dgemm(blas::Order::row_major, blas::Transpose::no,
                    blas::Transpose::no, 2, 2, 2, 2., a.data(), 2,
                    b.data(), 2, 3., c.data(), 2);
        // 2·[[19,22],[43,50]] + 3·ones = [[41,47],[89,103]]
        require(c[0] == 41. && c[1] == 47. && c[2] == 89. && c[3] == 103.,
                "dgemm alpha=2 beta=3");
    }

    // k==0 is numpy's (m,0)@(0,n) case: with beta==0 the provider must
    // zero C (BLAS treats an unset C as 0), and m==0 must not write.
    // Valid (unread) operand pointers plus VALID lda/ldb/ldc keep this
    // inside the strictest provider precondition — Accelerate validates
    // every parameter (fatally) before it ever looks at dimensions.
    void test_dgemm_zero_size() {
        const std::array<double, 1> stub{0.};
        std::array<double, 4> c{9., 9., 9., 9.};
        blas::dgemm(blas::Order::row_major, blas::Transpose::no,
                    blas::Transpose::no, 2, 2, 0, 1., stub.data(), 1,
                    stub.data(), 2, 0., c.data(), 2);
        require(c[0] == 0. && c[1] == 0. && c[2] == 0. && c[3] == 0.,
                "dgemm k=0 with beta=0 must zero C");
        std::array<double, 4> sentinel{5., 5., 5., 5.};
        blas::dgemm(blas::Order::row_major, blas::Transpose::no,
                    blas::Transpose::no, 0, 2, 2, 1., stub.data(), 2,
                    stub.data(), 2, 0., sentinel.data(), 2);
        require(sentinel[0] == 5. && sentinel[3] == 5.,
                "dgemm m=0 must not write C");
    }

    void test_ddot_raw_values_and_seed() {
        const std::array<double, 4> x{1., 99., 2., 99.};
        const std::array<double, 4> y{3., 88., 4., 88.};
        require(blas::ddot_raw(2, x.data(), 2, y.data(), 2) == 11.,
                "raw ILP64 dot forwards dimensions and strides");
        const std::array<double, 1> zero{-0.}, one{1.};
        const double seeded = blas::ddot(1, zero.data(), 1, one.data(), 1);
        require(seeded == 0. && !std::signbit(seeded), "ddot retains its +0 seed");
        // The bare fold's zero sign is provider-specific. Its measured
        // per-platform assertion remains pending completion-plan 1.11;
        // the no-execution implementation must not invent that evidence.
    }

    void test_zdotu_sub_values_and_strides() {
        // (1+2i)(3+4i) + (5+6i)(7+8i) = -18 + 92i; no conjugation.
        const std::array<double, 8> x{1., 2., 99., 99., 5., 6., 99., 99.};
        const std::array<double, 4> y{3., 4., 7., 8.};
        std::array<double, 2> out{};
        blas::zdotu_sub(2, x.data(), 2, y.data(), 1, out.data());
        require(out[0] == -18. && out[1] == 92.,
                "ILP64 unconjugated complex dot respects complex-element stride");
    }

    constexpr std::array<TestCase, 15> cases{{
        {.name = "ddot_values", .run = test_ddot_values},
        {.name = "ddot_strides", .run = test_ddot_strides},
        {.name = "ddot_zero_size", .run = test_ddot_zero_size},
        {.name = "ddot_raw_values_and_seed", .run = test_ddot_raw_values_and_seed},
        {.name = "zdotu_sub_values_and_strides", .run = test_zdotu_sub_values_and_strides},
        {.name = "dgemv_row_major", .run = test_dgemv_row_major},
        {.name = "dgemv_column_major", .run = test_dgemv_column_major},
        {.name = "dgemv_strides", .run = test_dgemv_strides},
        {.name = "dgemv_zero_size", .run = test_dgemv_zero_size},
        {.name = "dgemm_row_major_nn", .run = test_dgemm_row_major_nn},
        {.name = "dgemm_row_major_nt", .run = test_dgemm_row_major_nt},
        {.name = "dgemm_row_major_tt", .run = test_dgemm_row_major_tt},
        {.name = "dgemm_column_major", .run = test_dgemm_column_major},
        {.name = "dgemm_alpha_beta", .run = test_dgemm_alpha_beta},
        {.name = "dgemm_zero_size", .run = test_dgemm_zero_size},
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
        std::cerr << "usage: native_platform_blas_tests [case-name|--list]\n";
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
        std::cerr << "unknown native platform-blas case: " << requested << '\n';
        return 2;
    }
    return status;
}
