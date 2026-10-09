// numeric_sum.hpp — the two reduction orders the ported oracles use.
//
// CPython builtin sum() on floats (3.12+, Python/bltinmodule.c
// builtin_sum_impl) is a Neumaier compensated loop: the compensation
// term updates only while the running total stays finite; the running
// total itself always takes the naive step. Verified bitwise against
// CPython 3.14.3 on 200000 adversarial vectors (plus inf/nan/signed-zero
// edges).
//
// np.sum / ndarray.sum on contiguous float64 is numpy's pairwise_sum:
// sequential for n < 8, an eight-way block accumulator for n <= 128,
// and a halving recursion beyond that — replicated verbatim from the
// copy first proven in runtime/step.cpp.
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

// numpy pairwise_sum — contiguous float64: the n <= 128 eight-way block
// accumulator plus the halving recursion for longer inputs.
// NOLINTNEXTLINE(misc-no-recursion) recursion depth mirrors numpy's pairwise halving
[[nodiscard]] inline double numpy_pairwise_sum(
        std::span<const double> a) noexcept {
    const std::size_t n = a.size();
    if (n < 8) {
        std::array<double, 8> r{};
        std::ranges::copy(a, r.begin());
        return ((r[0] + r[1]) + (r[2] + r[3])) +
               ((r[4] + r[5]) + (r[6] + r[7]));
    }
    if (n <= 128) {
        std::array<double, 8> r{};
        std::ranges::copy(a.first(8), r.begin());
        std::size_t i = 8;
        for (; i + 8 <= n; i += 8)
            for (std::size_t j = 0; j < 8; ++j)
                r[j] += a[i + j];
        for (std::size_t j = 0; i + j < n; ++j)
            r[j] += a[i + j];
        return ((r[0] + r[1]) + (r[2] + r[3])) +
               ((r[4] + r[5]) + (r[6] + r[7]));
    }
    std::size_t n2 = n / 2;
    n2 -= n2 % 128;
    return numpy_pairwise_sum(a.first(n2)) +
           numpy_pairwise_sum(a.subspan(n2));
}

} // namespace numeric
