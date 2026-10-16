// random/numpy_compat.cpp — the PCG64 core and the NumpyRng object layer.
//
// The generator core is a statement-for-statement port of NumPy 2.5.2's
// numpy/random/src/pcg64/pcg64.h (XSL-RR 128-bit setseq, emulated 128-bit
// arithmetic so no compiler extension enters the build):
//
//   PCG64 Random Number Generation for C.
//   Copyright 2014 Melissa O'Neill <oneill@pcg-random.org>
//   Copyright 2015 Robert Kern <robert.kern@gmail.com>
//   Licensed under the Apache License, Version 2.0; relicensed MIT
//   May 2019 (see the upstream header for the complete text).
//
// Method semantics mirror _generator.pyx: uniform's range checks, normal's
// 'scale < 0' check, integers' closed-interval bounds and format_bounds_
// error messages, and choice()'s positive-population/probability domains.
// Draw transforms themselves live in distributions.cpp behind detail::.

#include "random/numpy_compat.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace bike::random {
namespace {

// pcg128_t — the {high, low} pair form used by PCG_EMULATED_128BIT_MATH.
struct Uint128 {
    std::uint64_t high = 0;
    std::uint64_t low = 0;
};

// PCG_DEFAULT_MULTIPLIER_{HIGH,LOW} — the LCG multiplier is fixed.
constexpr std::uint64_t kMultHigh = 2549297995355413924ULL;
constexpr std::uint64_t kMultLow = 4865540595714422341ULL;

// pcg128_add(a, b).
Uint128 add128(Uint128 a, Uint128 b) {
    Uint128 result;
    result.low = a.low + b.low;
    result.high = a.high + b.high + (result.low < b.low ? 1U : 0U);
    return result;
}

// _pcg_mult64(x, y, z1, z0) — the generic emulated 64x64 -> 128 multiply.
void mult64(std::uint64_t x, std::uint64_t y, std::uint64_t &z1,
            std::uint64_t &z0) {
    z0 = x * y;
    const std::uint64_t x0 = x & 0xFFFFFFFFULL;
    const std::uint64_t x1 = x >> 32U;
    const std::uint64_t y0 = y & 0xFFFFFFFFULL;
    const std::uint64_t y1 = y >> 32U;
    const std::uint64_t w0 = x0 * y0;
    const std::uint64_t t = x1 * y0 + (w0 >> 32U);
    std::uint64_t w1 = t & 0xFFFFFFFFULL;
    const std::uint64_t w2 = t >> 32U;
    w1 += x0 * y1;
    z1 = x1 * y1 + w2 + (w1 >> 32U);
}

// pcg128_mult(a, b).
Uint128 mult128(Uint128 a, Uint128 b) {
    const std::uint64_t h1 = a.high * b.low + a.low * b.high;
    Uint128 result;
    mult64(a.low, b.low, result.high, result.low);
    result.high += h1;
    return result;
}

// pcg_rotr_64 — rot stays in [0, 63] so (64 - rot) & 63 is (-rot) & 63.
std::uint64_t rotr64(std::uint64_t value, std::uint32_t rot) {
    return (value >> rot) | (value << ((64U - rot) & 63U));
}

// pcg_setseq_128_step_r — state = state * MULTIPLIER + inc (mod 2^128).
void step(detail::Bitgen &bitgen) {
    const Uint128 next = add128(
        mult128(Uint128{.high = bitgen.state_high, .low = bitgen.state_low},
                Uint128{.high = kMultHigh, .low = kMultLow}),
        Uint128{.high = bitgen.inc_high, .low = bitgen.inc_low});
    bitgen.state_high = next.high;
    bitgen.state_low = next.low;
}

} // namespace

// pcg64_set_seed + pcg_setseq_128_srandom_r: state=0, inc=(initseq<<1)|1,
// step, state += initstate, step. The four SeedSequence u64 words arrive as
// {initstate.high, initstate.low, initseq.high, initseq.low}.
// NOLINTBEGIN(bugprone-easily-swappable-parameters) (initstate, initseq) high/low pairs mirror pcg64_set_seed's positional order
void detail::seed_bitgen(Bitgen &bitgen, std::uint64_t seed_high,
                         std::uint64_t seed_low, std::uint64_t inc_high,
                         std::uint64_t inc_low) {
// NOLINTEND(bugprone-easily-swappable-parameters)
    bitgen.has_uint32 = false;
    bitgen.uinteger = 0;
    bitgen.state_high = 0;
    bitgen.state_low = 0;
    bitgen.inc_high = (inc_high << 1U) | (inc_low >> 63U);
    bitgen.inc_low = (inc_low << 1U) | 1U;
    step(bitgen);
    const Uint128 state =
        add128(Uint128{.high = bitgen.state_high, .low = bitgen.state_low},
               Uint128{.high = seed_high, .low = seed_low});
    bitgen.state_high = state.high;
    bitgen.state_low = state.low;
    step(bitgen);
}

// pcg_setseq_128_xsl_rr_64_random_r — the C step is taken first and the
// post-step state feeds the XSL-RR output permutation.
std::uint64_t detail::next64(Bitgen &bitgen) {
    step(bitgen);
    return rotr64(bitgen.state_high ^ bitgen.state_low,
                  static_cast<std::uint32_t>(bitgen.state_high >> 58U));
}

// pcg64_next32 — the buffered half-word lane: a fresh u64 supplies the low
// 32 bits first and stashes the high half for the next call.
std::uint32_t detail::next32(Bitgen &bitgen) {
    if (bitgen.has_uint32) {
        bitgen.has_uint32 = false;
        return bitgen.uinteger;
    }
    const std::uint64_t next = next64(bitgen);
    bitgen.has_uint32 = true;
    bitgen.uinteger = static_cast<std::uint32_t>(next >> 32U);
    return static_cast<std::uint32_t>(next & 0xffffffffULL);
}

// _common.pxd uint64_to_double.
double detail::next_double(Bitgen &bitgen) {
    return static_cast<double>(next64(bitgen) >> 11U) *
           (1.0 / 9007199254740992.0);
}

struct NumpyRng::Impl {
    detail::Bitgen bitgen;
};

NumpyRng::NumpyRng(std::uint64_t seed) : NumpyRng(SeedSequence(seed)) {}

NumpyRng::NumpyRng(std::string_view nonnegative_decimal_entropy,
                   std::span<const std::uint32_t> spawn_key)
    : NumpyRng(SeedSequence(nonnegative_decimal_entropy, spawn_key)) {}

NumpyRng::NumpyRng(const SeedSequence &sequence)
    : impl_(std::make_unique<Impl>()) {
    // BitGenerator.__cinit__: generate_state(4, np.uint64) then
    // pcg64_set_seed(seed=words[0..1], inc=words[2..3]).
    const std::vector<std::uint64_t> words = sequence.generate_state_u64(4);
    detail::seed_bitgen(impl_->bitgen, words.at(0), words.at(1),
                        words.at(2), words.at(3));
}

NumpyRng::~NumpyRng() = default;
NumpyRng::NumpyRng(NumpyRng &&) noexcept = default;
NumpyRng &NumpyRng::operator=(NumpyRng &&) noexcept = default;

std::uint64_t NumpyRng::raw64() { return detail::next64(impl_->bitgen); }

std::uint32_t NumpyRng::raw32() { return detail::next32(impl_->bitgen); }

double NumpyRng::uniform01() { return detail::next_double(impl_->bitgen); }

double NumpyRng::normal01() {
    return detail::standard_normal(impl_->bitgen);
}

void NumpyRng::uniform01(std::span<double> out) {
    // double_fill(random_standard_uniform): one next_double per element.
    for (double &value : out)
        value = detail::next_double(impl_->bitgen);
}

void NumpyRng::normal01(std::span<double> out) {
    // random_standard_normal_fill: one Ziggurat draw per element.
    for (double &value : out)
        value = detail::standard_normal(impl_->bitgen);
}

double NumpyRng::uniform(double low, double high) {
    // _generator.pyx uniform(): rng = high - low must be finite
    // (OverflowError 'high - low range exceeds valid bounds') and the
    // CONS_NON_NEGATIVE check rejects a negative signbit ('high - low < 0').
    const double range = high - low;
    if (!std::isfinite(range))
        throw std::overflow_error("high - low range exceeds valid bounds");
    if (!std::isnan(range) && std::signbit(range))
        throw std::invalid_argument("high - low < 0");
    return detail::uniform(impl_->bitgen, low, range);
}

void NumpyRng::uniform(double low, double high, std::span<double> out) {
    const double range = high - low;
    if (!std::isfinite(range))
        throw std::overflow_error("high - low range exceeds valid bounds");
    if (!std::isnan(range) && std::signbit(range))
        throw std::invalid_argument("high - low < 0");
    for (double &value : out)
        value = detail::uniform(impl_->bitgen, low, range);
}

double NumpyRng::normal(double loc, double scale) {
    // CONS_NON_NEGATIVE on 'scale'; a NaN scale passes the check exactly
    // like the oracle and produces NaN draws.
    if (!std::isnan(scale) && std::signbit(scale))
        throw std::invalid_argument("scale < 0");
    return detail::normal(impl_->bitgen, loc, scale);
}

void NumpyRng::normal(double loc, double scale, std::span<double> out) {
    if (!std::isnan(scale) && std::signbit(scale))
        throw std::invalid_argument("scale < 0");
    for (double &value : out)
        value = detail::normal(impl_->bitgen, loc, scale);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (low, high) mirrors Generator.integers' positional order
std::int64_t NumpyRng::integers(std::int64_t low, std::int64_t high,
                                bool endpoint) {
    // _rand_int64's scalar path: the internal generator produces the closed
    // interval [low, high - (not endpoint)]; bounds beyond int64 are
    // unrepresentable by the argument types so only format_bounds_error's
    // ordering leg is reachable (low > closed_high).
    if (!endpoint && high == std::numeric_limits<std::int64_t>::min()) {
        // Python computes high-1 in arbitrary precision: it lands below
        // int64, so every representable low satisfies low > closed_high
        // and format_bounds_error(closed, low) fires unconditionally.
        if (low == 0)
            throw std::invalid_argument("high <= 0");
        throw std::invalid_argument("low >= high");
    }
    const std::int64_t closed_high = endpoint ? high : high - 1;
    if (low > closed_high) {
        // format_bounds_error(closed, low).
        if (low == 0)
            throw std::invalid_argument(endpoint ? "high < 0" : "high <= 0");
        throw std::invalid_argument(endpoint ? "low > high" : "low >= high");
    }
    const std::uint64_t rng = static_cast<std::uint64_t>(closed_high) -
                              static_cast<std::uint64_t>(low);
    const std::uint64_t off = static_cast<std::uint64_t>(low);
    return std::bit_cast<std::int64_t>(
        detail::bounded_uint64(impl_->bitgen, off, rng));
}

std::int64_t NumpyRng::integers(std::int64_t high) {
    // integers(low) with high=None draws [0, low).
    return integers(0, high);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (low, high) mirrors Generator.integers' positional order
void NumpyRng::integers(std::int64_t low, std::int64_t high,
                        std::span<std::int64_t> out, bool endpoint) {
    // The size==0 path (np.prod(size) == 0) returns before any bounds work
    // in the source — no checks, no stream consumption.
    if (out.empty())
        return;
    if (!endpoint && high == std::numeric_limits<std::int64_t>::min()) {
        if (low == 0)
            throw std::invalid_argument("high <= 0");
        throw std::invalid_argument("low >= high");
    }
    const std::int64_t closed_high = endpoint ? high : high - 1;
    if (low > closed_high) {
        if (low == 0)
            throw std::invalid_argument(endpoint ? "high < 0" : "high <= 0");
        throw std::invalid_argument(endpoint ? "low > high" : "low >= high");
    }
    const std::uint64_t rng = static_cast<std::uint64_t>(closed_high) -
                              static_cast<std::uint64_t>(low);
    const std::uint64_t off = static_cast<std::uint64_t>(low);
    for (std::int64_t &value : out)
        value = std::bit_cast<std::int64_t>(
            detail::bounded_uint64(impl_->bitgen, off, rng));
}

std::int64_t NumpyRng::choice(std::int64_t population) {
    // Generator.choice scalar `a`: pop_size <= 0 fails before any draw; the
    // uniform path is exactly integers(0, pop_size).
    if (population <= 0)
        throw std::invalid_argument(
            "a must be a positive integer unless no samples are taken");
    return integers(0, population);
}

std::int64_t NumpyRng::choice(std::span<const double> probabilities) {
    std::array<std::int64_t, 1> out{0};
    choice(probabilities, out);
    return out.at(0);
}

void NumpyRng::choice(std::span<const double> probabilities,
                      std::span<std::int64_t> out) {
    // _generator.pyx choice(p=...) for a = arange(len(p)): the population
    // size is the probability count, so an empty span models the empty-a
    // rejection — which only fires when samples are actually requested
    // (np.prod(size) != 0); size=0 falls through to the probability checks
    // and kahan_sum of an empty p trips 'Probabilities do not sum to 1.'.
    if (probabilities.empty() && !out.empty())
        throw std::invalid_argument(
            "a cannot be empty unless no samples are taken");
    const double p_sum = detail::kahan_sum(probabilities);
    if (std::isnan(p_sum))
        throw std::invalid_argument("Probabilities contain NaN");
    for (const double p : probabilities)
        if (p < 0.)
            throw std::invalid_argument("Probabilities are not non-negative");
    // atol = sqrt(finfo(float64).eps) — the single-probability-dtype bound.
    if (std::abs(p_sum - 1.) >
        std::sqrt(std::numeric_limits<double>::epsilon()))
        throw std::invalid_argument(
            "Probabilities do not sum to 1. See Notes section of docstring "
            "for more information.");
    // cdf = p.cumsum() (sequential accumulate); cdf /= cdf[-1] elementwise.
    std::vector<double> cdf(probabilities.size());
    double cumulative = 0.;
    for (std::size_t i = 0; i < probabilities.size(); ++i) {
        cumulative += probabilities[i];
        cdf.at(i) = cumulative;
    }
    const double last = cdf.back();
    for (double &value : cdf)
        value /= last;
    // idx = cdf.searchsorted(uniform_samples, side='right'): one random()
    // per sample, upper-bound insertion over the running sum.
    for (std::int64_t &index : out) {
        const double sample = detail::next_double(impl_->bitgen);
        index = static_cast<std::int64_t>(std::ranges::upper_bound(
                                              cdf.begin(), cdf.end(),
                                              sample) -
                                          cdf.begin());
    }
}

} // namespace bike::random
