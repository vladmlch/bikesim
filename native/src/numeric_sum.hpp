// numeric_sum.hpp — the two reduction orders the ported oracles use.
//
// CPython 3.13 builtin sum() on float items (Python/bltinmodule.c
// builtin_sum_impl's PyFloat_CheckExact fast path) is a Neumaier
// compensated loop; the running total itself always takes the naive
// step. Items that are not exact floats take different 3.13 paths:
// PyLong items that fit a long get a naive `f_result += (double)value`
// (3.14 compensates them and adds a complex fast path — a real
// divergence only reachable from generators that can yield int or
// complex items), and float subclasses such as np.float64 fall back to
// the generic PyNumber_Add loop (plain sequential adds — see
// legacy/telemetry.cpp's body_mass site, which must NOT use PythonSum).
// Every ported call site below feeds Python floats only.
//
// PythonSum::add updates the compensation term only while the running
// total stays finite — a local guard with no upstream counterpart (both
// 3.13 and 3.14 update unconditionally and gate `c` only at fold-out).
// The guard is observationally equivalent: once t is non-finite the
// running sum never returns to finite, so a poisoned `c` is unreachable
// at the `if (c && Py_IS_FINITE(c))` fold-out in either version. The
// float-item path is line-equivalent between 3.13 and 3.14; verified
// bitwise against CPython 3.14.3 on 200000 adversarial vectors (plus
// inf/nan/signed-zero edges) — equally valid for 3.13's identical
// float branch.
//
// np.sum / ndarray.sum on contiguous float64 is numpy's pairwise_sum
// (numpy/_core/src/umath/loops.c.src): a sequential fold for n < 8, an
// eight-way block accumulator whose leftover tail folds into the scalar
// result for n <= 128, and a halving recursion split on a multiple of 8
// beyond that. Verified bitwise against numpy 2.5.2 for n = 0..3000.
//
// `a @ b`/`np.dot` are NOT here on purpose: they dispatch to Accelerate
// BLAS and keep their cblas_abi.hpp call sites.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>

namespace numeric {

// Accumulator form of builtin sum(): feed terms in the oracle's
// iteration order; total() applies the final hi + lo fold. An empty
// sequence totals 0.0 — equal to CPython's int-0 result under == and
// isclose (the callers' comparison faces).
class PythonSum {
public:
    void add(double x) noexcept {
        const double t = sum_ + x;
        if (std::isfinite(t)) {
            c_ += std::abs(sum_) >= std::abs(x) ? (sum_ - t) + x
                                                : (x - t) + sum_;
        }
        sum_ = t;
    }
    [[nodiscard]] double total() const noexcept { return sum_ + c_; }

private:
    double sum_ = 0., c_ = 0.;
};

// numpy pairwise_sum — contiguous float64: sequential for n < 8, the
// n <= 128 eight-way block accumulator with the leftover tail folded
// into the scalar result, and the multiple-of-8 halving recursion for
// longer inputs. This is the raw ufunc inner loop only — np.sum itself
// additionally seeds the reduce accumulator with the 0.0 identity, so
// call sites porting `np.sum(a)` use `numpy_sum` below.
// NOLINTNEXTLINE(misc-no-recursion) recursion depth mirrors numpy's pairwise halving
[[nodiscard]] inline double numpy_pairwise_sum(
        std::span<const double> a) noexcept {
    const std::size_t n = a.size();
    if (n < 8) {
        // Upstream pairwise_sum(0) returns the -0.0 seed (loops_utils);
        // this shortcut returns +0.0 — np.sum's empty-reduce identity.
        // Intentional: numpy_sum's `0.0 + pairwise_sum(...)` fold yields
        // +0.0 either way, and this helper is only reached through it.
        if (n == 0) return 0.;
        double result = -0.;
        for (const double value : a) result += value;
        return result;
    }
    if (n <= 128) {
        std::array<double, 8> r{};
        std::ranges::copy(a.first(8), r.begin());
        std::size_t i = 8;
        for (; i + 8 <= n; i += 8)
            for (std::size_t j = 0; j < 8; ++j)
                r[j] += a[i + j];
        double result = ((r[0] + r[1]) + (r[2] + r[3])) +
                        ((r[4] + r[5]) + (r[6] + r[7]));
        for (; i < n; ++i) result += a[i];
        return result;
    }
    std::size_t n2 = n / 2;
    n2 -= n2 % 8;
    return numpy_pairwise_sum(a.first(n2)) +
           numpy_pairwise_sum(a.subspan(n2));
}

// np.sum(a) — ufunc add.reduce: the accumulator is seeded with the
// reduction identity 0.0 and the inner loop folds pairwise_sum into it
// (loops_arithm_fp.dispatch.c.src: `*src0 += DOUBLE_pairwise_sum(...)`).
// `0.0 + r` is a no-op for every r except -0.0, where it yields +0.0 —
// np.sum([-0.0]) is +0.0 while pairwise_sum([-0.0]) is -0.0, a one-bit
// difference the oracle contract must reproduce.
[[nodiscard]] inline double numpy_sum(
        std::span<const double> a) noexcept {
    return 0.0 + numpy_pairwise_sum(a);
}

} // namespace numeric
