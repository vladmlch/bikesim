// numeric_sincos.hpp — sin/cos wrappers that keep ported transcendental
// pairs as two separate libm calls. The Python oracles evaluate
// np.sin / np.cos (or math.sin / math.cos) of one angle as independent
// calls, and the bitwise-parity contract requires the same lowering
// here.
//
// Optimizing codegen on this toolchain fuses a same-operand
// std::sin + std::cos pair into a single __sincos_stret libcall, whose
// returned components can differ from the standalone libcalls by one
// ulp — observed: sin(-0.57532456093834838) lowers to
// 0xbfe169535bafc62e through __sincos_stret vs 0xbfe169535bafc62d
// through the standalone call (which CPython and NumPy match). The
// finite-difference Jacobians then amplify one ulp by 1/(2*delta_rad).
//
// Reading the argument through a volatile lvalue hands each call an
// unmergeable operand: two volatile loads are distinct values to the
// optimizer, so the sin+cos same-operand fusion cannot fire. The calls
// remain the platform libm functions — bitwise-identical to the
// separate-call oracle path — at every optimization level.
//
// Oracle split — the two families are NOT interchangeable on Apple
// arm64 for non-finite inputs:
//
//   numeric::np_sin / np_cos   — the np.sin / np.cos ufunc contract.
//     npy_sin/npy_cos additionally return `x - x` for non-finite
//     inputs on __APPLE__ && __arm64__ (WORKAROUND_APPLE_TRIG_BUG,
//     npy_math_internal.h.src:296-313): the AArch64 FSUB keeps the
//     operand's NaN sign/payload (quieting a sNaN) where Apple libm
//     canonicalizes to +qNaN — np.cos(-nan 0xfff8…0000) is
//     0xfff8…0000 while libm cos is 0x7ff8…0000. The identical
//     preprocessor guard is mirrored below; on other targets npy_sin
//     is a plain libm call, which is exactly what the unconditional
//     return already is.
//
//   numeric::math_sin / math_cos — the math.sin / math.cos contract.
//     CPython calls libm directly; there is no `x - x` path, so a
//     negative-NaN angle returns libm's +qNaN, matching CPython.
//     Error-text note (W1.2x): CPython 3.14 raises
//     "expected a finite input, got %s" for nonfinite sin/cos/tan and
//     "expected a nonnegative input, got %s" for negative sqrt, where
//     3.13 raises the bare "math domain error". No error text is
//     mirrored here — these wrappers are noexcept libm calls and every
//     ported call site validates its angle upstream, so the divergence
//     is unreachable in the ported domain and no change is needed.
//
// Uniform rule: every sin/cos call mirrored from Python uses the explicit
// family of its mirror, whether the call is paired or lone. np_sin/np_cos
// where Python calls np.sin/np.cos (the ufunc contract is the npy_* function);
// math_sin/math_cos where Python calls math.sin/math.cos (CPython calls libm
// directly). A lone call gets the same family as a paired one, so each site
// is audited against its oracle without a same-operand analysis. The
// volatile read is harmless on a lone call: it only blocks the
// __sincos_stret fusion, which a lone call cannot trigger.
//
// T04 provider identity: the calls resolve to the platform libm (libSystem
// on macOS, glibc libm on Linux). The numeric provider that
// engine::Identity reports is per platform, not one fixed name, so the
// anti-fusion idiom is part of the same-provider bitwise contract on every
// platform. It is not a workaround to remove.
#pragma once

#include <cmath>

namespace numeric {
    [[nodiscard]] inline double np_sin(double angle_rad) noexcept {
        const volatile double angle = angle_rad;
#if defined(__APPLE__) && defined(__arm64__)
        if (!std::isfinite(angle)) {
            // Deliberate x - x (npy_sin): FSUB keeps the NaN sign/payload.
            return angle - angle;  // NOLINT(misc-redundant-expression)
        }
#endif
        return std::sin(angle);
    }

    [[nodiscard]] inline double np_cos(double angle_rad) noexcept {
        const volatile double angle = angle_rad;
#if defined(__APPLE__) && defined(__arm64__)
        if (!std::isfinite(angle)) {
            // Deliberate x - x (npy_cos): FSUB keeps the NaN sign/payload.
            return angle - angle;  // NOLINT(misc-redundant-expression)
        }
#endif
        return std::cos(angle);
    }

    [[nodiscard]] inline double math_sin(double angle_rad) noexcept {
        const volatile double angle = angle_rad;
        return std::sin(angle);
    }

    [[nodiscard]] inline double math_cos(double angle_rad) noexcept {
        const volatile double angle = angle_rad;
        return std::cos(angle);
    }
} // namespace numeric
