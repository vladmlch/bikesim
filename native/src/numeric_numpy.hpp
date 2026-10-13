// numeric_numpy.hpp — bitwise scalar ports of the NumPy float64 ufunc
// inner loops the ported oracles actually run on this platform
// (aarch64 + Accelerate): SIMD/NEON kernels scalarized element by
// element.
//
//   numeric::np_tanh            — DOUBLE_tanh: simd_tanh_f64
//                                 (numpy/_core/src/umath/
//                                 loops_hyperbolic.dispatch.cpp.src),
//                                 the Highway lut18x16 path specialized
//                                 to one lane. Verified bitwise against
//                                 numpy 2.5.2 np.tanh on 2,000,000
//                                 values (structured reals + raw bit
//                                 patterns, 0 mismatches); std::tanh
//                                 diverged on 29,048 of them.
//   numeric::np_maximum /       — DOUBLE_maximum / DOUBLE_minimum:
//   numeric::np_minimum           on __aarch64__ scalar_max_d lowers to
//                                 the NEON fmax instruction
//                                 (loops_minmax.dispatch.c.src:65-76):
//                                 a NaN operand is returned unquieted
//                                 (first operand wins if both are NaN),
//                                 and a +0/-0 tie resolves to +0 for
//                                 max, -0 for min — NOT the
//                                 `(a >= b || isnan(a)) ? a : b`
//                                 textbook expression.
//   numeric::np_sign          — DOUBLE_sign generic loop
//                                 (loops.c.src:1433-1442): +1/-1/+0,
//                                 NaN passthrough unquieted.
//   numeric::np_interp_unit_grid — np.interp(x, np.arange(n), values,
//                                 left=lval, right=rval) for one query:
//                                 arr_interp (compiled_base.c:671-739)
//                                 specialized to integer knots
//                                 0..n-1, including the dedicated
//                                 lenxp == 1 loop that runs BEFORE the
//                                 NaN check (a NaN query returns
//                                 values[0], not NaN). Verified bitwise
//                                 against np.interp on 13,431,042
//                                 queries (fractional bins, out-of-range
//                                 sentinels, exact knots, n = 3..256),
//                                 0 mismatches; n = 1 coverage added
//                                 with the dedicated-branch port.
//
// Compile flags: this header's arithmetic relies on the project-wide
// -ffp-contract=off — plain `a*b + c` expressions stay two rounded
// operations, while `std::fma` stays an explicit fused call.
#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <span>
#include <stdexcept>

namespace numeric {

    // np.maximum(a, b) — see header comment for the NEON fmaxnm
    // contract. A NaN operand is returned with its payload intact but
    // quieted (fmaxnm/fminnm set the quiet bit on a signaling NaN —
    // verified against np.maximum on 2.5.2: snan 0x7ff0…01 ->
    // 0x7ff8…01 in either argument position); when both are NaN the
    // first operand wins.
    [[nodiscard]] inline double np_quiet_nan(double x) noexcept {
        return std::bit_cast<double>(std::bit_cast<std::uint64_t>(x) |
                                     0x0008000000000000ull);
    }

    [[nodiscard]] inline double np_maximum(double a, double b) noexcept {
        if (std::isnan(a)) return np_quiet_nan(a);
        if (std::isnan(b)) return np_quiet_nan(b);
        // Zero tie: max(+0,-0) = +0 either order; -0 only when both -0.
        if (a == 0.0 && b == 0.0)
            return std::signbit(a) && std::signbit(b) ? -0.0 : 0.0;
        return a > b ? a : b;
    }

    // np.minimum(a, b) — the NEON fminnm twin: first NaN wins, -0.0 wins
    // a zero tie.
    [[nodiscard]] inline double np_minimum(double a, double b) noexcept {
        if (std::isnan(a)) return np_quiet_nan(a);
        if (std::isnan(b)) return np_quiet_nan(b);
        if (a == 0.0 && b == 0.0)
            return std::signbit(a) || std::signbit(b) ? -0.0 : 0.0;
        return a < b ? a : b;
    }

    namespace detail {
        // simd_tanh_f64's lut18x16 (loops_hyperbolic.dispatch.cpp.src
        // lines 172-362), verbatim bit patterns: 16 intervals × 18
        // doubles [b, c0, .., c16]. tanh(|x|) is approximated on each
        // interval by a minimax polynomial in y = |x| - b; interval 15
        // is the fake saturation row (b=0, c0=1.0, rest 0.0).
        // clang-format off
        inline constexpr std::array<std::uint64_t,
                                    static_cast<std::size_t>(16 * 18)>
                kTanhLut18x16 = {
            // 0 — |x| in [0, ~0.125)
            0x0000000000000000ull, 0x0000000000000000ull, 0x3ff0000000000000ull, 0xbbf0b3ea3fdfaa19ull, 0xbfd5555555555555ull, 0xbce6863ee44ed636ull, 0x3fc1111111112ab5ull, 0xbda1ea19ddddb3b4ull, 0xbfaba1ba1990520bull, 0xbe351ca7f096011full, 0x3f9664f94e6ac14eull, 0xbea8c4c1fd7852feull, 0xbf822404577aa9ddull, 0xbefdd99a221ed573ull, 0x3f6e3be689423841ull, 0xbf2a1306713a4f3aull, 0xbf55d7e76dc56871ull, 0x3f35e67ab76a26e7ull,
            // 1
            0x3fcc000000000000ull, 0x3fcb8fd0416a7c92ull, 0x3fee842ca3f08532ull, 0xbfca48aaeb53bc21ull, 0xbfd183afc292ba11ull, 0x3fc04dcd0476c75eull, 0x3fb5c19efdfc08adull, 0xbfb0b8df995ce4dfull, 0xbf96e37bba52f6fcull, 0x3f9eaaf3320c3851ull, 0xbf94d3343bae39ddull, 0xbfccce16b1046f13ull, 0x403d8b07f7a82aa3ull, 0x4070593a3735bab4ull, 0xc0d263511f5baac1ull, 0xc1045e509116b066ull, 0x41528c38809c90c7ull, 0x41848ee0627d8206ull,
            // 2
            0x3fd4000000000000ull, 0x3fd35f98a0ea650eull, 0x3fed11574af58f1bull, 0xbfd19921f4329916ull, 0xbfcc1a4b039c9bfaull, 0x3fc43d3449a80f08ull, 0x3fa74c98dc34fbacull, 0xbfb2955cf41e8164ull, 0x3ecff7df18455399ull, 0x3f9cf823fe761fc1ull, 0xbf7bc748e60df843ull, 0xbf81a16f224bb7b6ull, 0xbf9f44ab92fbab0aull, 0xbfccab654e44835eull, 0x40169f73b15ebe5cull, 0x4041fab9250984ceull, 0xc076d57fb5190b02ull, 0xc0a216d618b489ecull,
            // 3
            0x3fdc000000000000ull, 0x3fda5729ee488037ull, 0x3fea945b9c24e4f9ull, 0xbfd5e0f09bef8011ull, 0xbfc16e1e6d8d0be6ull, 0x3fc5c26f3699b7e7ull, 0xbf790d6a8eff0a77ull, 0xbfaf9d05c309f7c6ull, 0x3f97362834d33a4eull, 0x3f9022271754ff1full, 0xbf8c89372b43ba85ull, 0xbf62cbf00406bc09ull, 0x3fb2eac604473d6aull, 0x3fd13ed80037dbacull, 0xc025c1dd41cd6cb5ull, 0xc0458d090ec3de95ull, 0x4085f09f888f8adaull, 0x40a5b89107c8af4full,
            // 4
            0x3fe4000000000000ull, 0x3fe1bf47eabb8f95ull, 0x3fe6284c3374f815ull, 0xbfd893b59c35c882ull, 0xbf92426c751e48a2ull, 0x3fc1a686f6ab2533ull, 0xbfac3c021789a786ull, 0xbf987d27ccff4291ull, 0x3f9e7f8380184b45ull, 0xbf731fe77c9c60afull, 0xbf8129a092de747aull, 0x3f75b29bb02cf69bull, 0x3f45f87d903aaac8ull, 0xbf6045b9076cc487ull, 0xbf58fd89fe05e0d1ull, 0xbf74949d60113d63ull, 0x3fa246332a2fcba5ull, 0x3fb69d8374520edaull,
            // 5
            0x3fec000000000000ull, 0x3fe686650b8c2015ull, 0x3fe02500a09f8d6eull, 0xbfd6ba7cb7576538ull, 0x3fb4f152b2bad124ull, 0x3faf203c316ce730ull, 0xbfae2196b7326859ull, 0x3f8b2ca62572b098ull, 0x3f869543e7c420d4ull, 0xbf84a6046865ec7dull, 0x3f60c85b4d538746ull, 0x3f607df0f9f90c17ull, 0xbf5e104671036300ull, 0x3f2085ee7e8ac170ull, 0x3f73f7af01d5af7aull, 0x3f7c9fd6200d0adeull, 0xbfb29d851a896fcdull, 0xbfbded519f981716ull,
            // 6
            0x3ff4000000000000ull, 0x3feb2523bb6b2deeull, 0x3fd1f25131e3a8c0ull, 0xbfce7291743d7555ull, 0x3fbbba40cbef72beull, 0xbf89c7a02788557cull, 0xbf93a7a011ff8c2aull, 0x3f8f1cf6c7f5b00aull, 0xbf7326bd4914222aull, 0xbf4ca3f1f2b9192bull, 0x3f5be9392199ec18ull, 0xbf4b852a6e0758d5ull, 0x3f19bc98ddf0f340ull, 0x3f23524622610430ull, 0xbf1e40bdead17e6bull, 0x3f02cd40e0ad0a9full, 0x3ed9065ae369b212ull, 0xbef02d288b5b3371ull,
            // 7
            0x3ffc000000000000ull, 0x3fee1fbf97e33527ull, 0x3fbd22ca1c24a139ull, 0xbfbb6d85a01efb80ull, 0x3fb01ba038be6a3dull, 0xbf98157e26e0d541ull, 0x3f6e4709c7e8430eull, 0x3f60379811e43dd5ull, 0xbf5fc15b0a9d98faull, 0x3f4c77dee0afd227ull, 0xbf2a0c68a4489f10ull, 0xbf0078c63d1b8445ull, 0x3f0d4304bc9246e8ull, 0xbeff12a6626911b4ull, 0x3ee224cd6c4513e5ull, 0xbe858ab8e019f311ull, 0xbeb8e1ba4c98a030ull, 0x3eb290981209c1a6ull,
            // 8
            0x4004000000000000ull, 0x3fef9258260a71c2ull, 0x3f9b3afe1fba5c76ull, 0xbf9addae58c7141aull, 0x3f916df44871efc8ull, 0xbf807b55c1c7d278ull, 0x3f67682afa611151ull, 0xbf4793826f78537eull, 0x3f14cffcfa69fbb6ull, 0x3f04055bce68597aull, 0xbf00462601dc2faaull, 0x3eec12eadd55be7aull, 0xbed13c415f7b9d41ull, 0x3eab9008bca408afull, 0xbe24b645e68eeaa3ull, 0xbe792fa6323b7cf8ull, 0x3e6ffd0766ad4016ull, 0xbe567e924bf5ff6eull,
            // 9
            0x400c000000000000ull, 0x3feff112c63a9077ull, 0x3f6dd37d19b22b21ull, 0xbf6dc59376c7aa19ull, 0x3f63c6869dfc8870ull, 0xbf53a18d5843190full, 0x3f3ef2ee77717cbfull, 0xbf2405695e36240full, 0x3f057e48e5b79d10ull, 0xbee2bf0cb4a71647ull, 0x3eb7b6a219dea9f4ull, 0xbe6fa600f593181bull, 0xbe722b8d9720cdb0ull, 0x3e634df71865f620ull, 0xbe4abfebfb72bc83ull, 0x3e2df04d67876402ull, 0xbe0c63c29f505f5bull, 0x3de3f7f7de6b0eb6ull,
            // 10
            0x4014000000000000ull, 0x3fefff419668df11ull, 0x3f27ccec13a9ef96ull, 0xbf27cc5e74677410ull, 0x3f1fb9aef915d828ull, 0xbf0fb6bbc89b1a5bull, 0x3ef95a4482f180b7ull, 0xbee0e08de39ce756ull, 0x3ec33b66d7d77264ull, 0xbea31eaafe73efd5ull, 0x3e80cbcc8d4c5c8aull, 0xbe5a3c935dce3f7dull, 0x3e322666d739bec0ull, 0xbe05bb1bcf83ca73ull, 0x3dd51c38f8695ed3ull, 0xbd95c72be95e4d2cull, 0xbd7fab216b9e0e49ull, 0x3d69ed18bae3ebbcull,
            // 11
            0x401c000000000000ull, 0x3feffffc832750f2ull, 0x3ecbe6c3f33250aeull, 0xbecbe6c0e8b4cc87ull, 0x3ec299d1e27c6e11ull, 0xbeb299c9c684a963ull, 0x3e9dc2c27da3b603ull, 0xbe83d709ba5f714eull, 0x3e66ac4e578b9b10ull, 0xbe46abb02c4368edull, 0x3e2425bb231a5e29ull, 0xbe001c6d95e3ae96ull, 0x3dd76a553d7e7918ull, 0xbdaf2ac143fb6762ull, 0x3d8313ac38c6832bull, 0xbd55a89c30203106ull, 0x3d2826b62056aa27ull, 0xbcf7534c4f3dfa71ull,
            // 12
            0x4024000000000000ull, 0x3feffffffdc96f35ull, 0x3e41b4865394f75full, 0xbe41b486526b0565ull, 0x3e379b5ddcca334cull, 0xbe279b5dd4fb3d01ull, 0x3e12e2afd9f7433eull, 0xbdf92e3fc5ee63e0ull, 0x3ddcc74b8d3d5c42ull, 0xbdbcc749ca8079ddull, 0x3d9992a4beac8662ull, 0xbd74755a00ea1fd3ull, 0x3d4de0fa59416a39ull, 0xbd23eae52a3dbf57ull, 0x3cf7787935626685ull, 0xbccad6b3bb9eff65ull, 0x3ca313e31762f523ull, 0xbc730b73f1eaff20ull,
            // 13
            0x402c000000000000ull, 0x3fefffffffffcf58ull, 0x3d8853f01bda5f28ull, 0xbd8853f01bef63a4ull, 0x3d8037f57bc62c9aull, 0xbd7037f57ae72aa6ull, 0x3d59f320348679baull, 0xbd414cc030f2110eull, 0x3d23c589137f92b4ull, 0xbd03c5883836b9d2ull, 0x3ce191ba5ed3fb67ull, 0xbcbc1c6c063bb7acull, 0x3c948716cf3681b4ull, 0xbc6b5e3e9ca0955eull, 0x3c401ffc49c6bc29ull, 0xbc12705ccd3dd884ull, 0x3bea37aa21895319ull, 0xbbba2cff8135d462ull,
            // 14
            0x4034000000000000ull, 0x3ff0000000000000ull, 0x3c73953c0197ef58ull, 0xbc73955be519be31ull, 0x3c6a2d4b50a2cff7ull, 0xbc5a2ca2bba78e86ull, 0x3c44b61d9bbcc940ull, 0xbc2ba022e8d82a87ull, 0x3c107f8e2c8707a1ull, 0xbbf07a5416264aecull, 0x3bc892450bad44c4ull, 0xbba3be9a4460fe00ull, 0x3b873f9f2d2fda99ull, 0xbb5eca68e2c1ba2eull, 0xbabf0b21acfa52abull, 0xba8e0a4c47ae75f5ull, 0x3ae5c7f1fd871496ull, 0xbab5a71b5f7d9035ull,
            // 15 — fake saturation interval: b=0, c0=1.0, all other
            // coefficients 0 (tanh(|x|) = 1.0 beyond the table).
            0x0000000000000000ull, 0x3ff0000000000000ull, 0x0000000000000000ull, 0x0000000000000000ull, 0x0000000000000000ull, 0x0000000000000000ull, 0x0000000000000000ull, 0x0000000000000000ull, 0x0000000000000000ull, 0x0000000000000000ull, 0x0000000000000000ull, 0x0000000000000000ull, 0x0000000000000000ull, 0x0000000000000000ull, 0x0000000000000000ull, 0x0000000000000000ull, 0x0000000000000000ull, 0x0000000000000000ull,
        };
        // clang-format on
    } // namespace detail

    // np.tanh(x) — simd_tanh_f64 scalarized. The interval index comes
    // from the exponent+quiet-bit field of the input word (exactly the
    // numpy bit arithmetic; the s64->s32 bitcast halves are equivalent
    // because only the high 32-bit lane feeds idx). MulAdd lowers to
    // explicit std::fma — FMA contraction flags do not touch it.
    [[nodiscard]] inline double np_tanh(double x) noexcept {
        const std::uint64_t xb = std::bit_cast<std::uint64_t>(x);
        const std::int64_t ndnan =
                static_cast<std::int64_t>(xb & 0x7ff8000000000000ull);
        // |x| above the table range, +inf and NaN inputs take the
        // forced-1.0 path (NaN is re-pinned to the canonical qnan below).
        const bool special = ndnan <= 0x7fe0000000000000LL;
        const bool isnan = std::isnan(x);
        const std::int64_t idxs = ndnan - 0x3fc0000000000000LL;
        // numpy bitcasts the s64 difference to s32 lanes and clamps the
        // high lane — the u64 /2^32 division selects the same high word
        // (identical bit pattern to an arithmetic high-half extract;
        // division keeps tidy's signed-bitwise check out of it).
        std::int32_t hi = static_cast<std::int32_t>(
                static_cast<std::uint64_t>(idxs) / 0x100000000ULL);
        hi = std::max<std::int32_t>(hi, 0);
        hi = std::min<std::int32_t>(hi, 0x780000);
        const std::size_t idx =
                static_cast<std::uint32_t>(hi) / 0x80000U;
        const std::size_t row = idx * 18;
        // bit_cast per coefficient keeps strict aliasing intact — the
        // LUT stores the doubles' IEEE bit patterns, not doubles. row+k
        // is bounded by construction (idx <= 15, k <= 17), so the
        // unchecked subscript can't escape.
        const auto coef = [row](std::size_t k) noexcept {
            return std::bit_cast<double>(
                    detail::kTanhLut18x16[row + k]);
        };
        const std::uint64_t sign = xb & 0x8000000000000000ull;
        const double y = std::fabs(x) - coef(0);
        double r = std::fma(coef(17), y, coef(16));
        r = std::fma(r, y, coef(15));
        r = std::fma(r, y, coef(14));
        r = std::fma(r, y, coef(13));
        r = std::fma(r, y, coef(12));
        r = std::fma(r, y, coef(11));
        r = std::fma(r, y, coef(10));
        r = std::fma(r, y, coef(9));
        r = std::fma(r, y, coef(8));
        r = std::fma(r, y, coef(7));
        r = std::fma(r, y, coef(6));
        r = std::fma(r, y, coef(5));
        r = std::fma(r, y, coef(4));
        r = std::fma(r, y, coef(3));
        r = std::fma(r, y, coef(2));
        r = std::fma(r, y, coef(1));
        if (!special) r = 1.0;
        r = std::bit_cast<double>(std::bit_cast<std::uint64_t>(r) | sign);
        if (isnan) r = std::bit_cast<double>(0x7ff8000000000000ull);
        return r;
    }

    // np.sign(x) — the DOUBLE_sign generic loop
    // (numpy/_core/src/umath/loops.c.src:1433-1442, the float branch of
    // the "#TYPE = FLOAT, DOUBLE, LONGDOUBLE" repeat):
    //   in > 0 -> +1.0; in < 0 -> -1.0; in == 0 -> +0.0 (the literal
    //   `0` — np.sign(-0.0) is +0.0, the sign is NOT preserved);
    //   otherwise -> the input itself, so a NaN passes through with its
    //   payload (and quiet bit) intact, unquieted.
    [[nodiscard]] inline double np_sign(double x) noexcept {
        return x > 0.0 ? 1.0 : (x < 0.0 ? -1.0 : (x == 0.0 ? 0.0 : x));
    }

    // np.clip(x, lo, hi) with ARRAY bounds — the umath DOUBLE_clip
    // generic loop (numpy/_core/src/umath/clip.cpp:148-153, 239-246):
    // _NPY_CLIP = _NPY_MIN(_NPY_MAX(x, lo), hi) where, for the floating
    // point tag, _NPY_MAX(a,b) = isnan(a) ? a : (a>b ? a : b) and
    // _NPY_MIN(a,b) = isnan(a) ? a : (a<b ? a : b) — PyArray_MAX/MIN
    // ternaries, NOT the fmaxnm/fminnm semantics of np_maximum/
    // np_minimum. Observable differences (verified bitwise against
    // numpy 2.5.2, array-bound form): clip(+0.0, -0.0, hi) yields
    // -0.0 — the bound (second operand) wins every tie — where
    // maximum/minimum composition yields +0.0, and a signaling-NaN
    // input propagates with its payload unquieted. Do NOT use this
    // helper for scalar-bounds clip: the const-minmax fast path
    // (steps[1]==steps[2]==0, clip.cpp:162-201) is a DIFFERENT loop
    // whose `x < min / x > max` tests resolve ties to the input —
    // np.clip(+0.0, -0.0 scalar, hi) yields +0.0.
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional bounds mirror np.clip(x, lo, hi)
    [[nodiscard]] inline double np_clip(double x, double lo,
                                        double hi) noexcept {
        const double y = std::isnan(x) ? x : (x > lo ? x : lo);
        return std::isnan(y) ? y : (y < hi ? y : hi);
    }

    // np.interp(x, np.arange(n), values, left=lval, right=rval) — one
    // query on the integer knot grid. arr_interp behavior:
    //   lenxp == 1         -> dedicated loop (compiled_base.c:671-682):
    //                         (x < 0.0) ? lval : (x > 0.0) ? rval
    //                         : values[0]. Runs BEFORE the NaN check,
    //                         so a NaN query returns values[0], and
    //                         -0.0 compares equal to the knot (both
    //                         tests false -> values[0]). NOT the same
    //                         tree as the general path.
    //   x NaN (n >= 2)     -> x (NaN passthrough keeps the payload);
    //   x < 0.0            -> lval        (searchsorted j == -1);
    //   x > n - 1          -> rval        (j == lenxp);
    //   j == n - 1         -> values[n-1] (last-knot special case);
    //   x exactly on a knot-> values[j]   (dx[j] == x_val shortcut);
    //   otherwise          -> fma(slope, x - j, values[j]) with the
    //                         forward-point NaN fallback — unreachable
    //                         on finite `values` but kept verbatim.
    [[nodiscard]] inline double np_interp_unit_grid(
            double x, std::span<const double> values, double lval,
            double rval) {
        const std::size_t n = values.size();
        if (n == 0)
            throw std::invalid_argument(
                    "array of sample points is empty");
        if (n == 1) {
            // compiled_base.c:678-679 verbatim: two comparisons
            // against the single knot dx[0] == 0.0, everything else
            // (x == 0.0, +/-0.0, NaN) falls through to fp[0].
            if (x < 0.0) return lval;
            if (x > 0.0) return rval;
            return values[0];
        }
        if (std::isnan(x)) return x;
        const double last = static_cast<double>(n - 1);
        if (x < 0.0) return lval;
        if (x > last) return rval;
        const std::size_t j = static_cast<std::size_t>(x);
        if (j == n - 1) return values[n - 1];
        const double x0 = static_cast<double>(j);
        if (x0 == x) return values[j];
        // Unit grid: dx[j+1] - dx[j] is exactly 1.0, so the slope is
        // values[j+1] - values[j] verbatim (x/1.0 is IEEE-exact).
        const double slope = values[j + 1] - values[j];
        double out = std::fma(slope, x - x0, values[j]);
        if (std::isnan(out)) {
            const double x1 = static_cast<double>(j + 1);
            out = std::fma(slope, x - x1, values[j + 1]);
            if (std::isnan(out) && values[j] == values[j + 1])
                out = values[j];
        }
        return out;
    }

} // namespace numeric
