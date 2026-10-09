// numeric_sincos.hpp — sin/cos wrappers that keep ported transcendental
// pairs as two separate libm calls. The Python oracles evaluate np.sin /
// np.cos (or math.sin / math.cos) of one angle as independent calls, and
// the bitwise-parity contract requires the same lowering here.
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
// Use these wrappers only where the ported expression shares one angle
// between sin and cos. Lone calls (no matching partner) keep plain
// std::sin/std::cos — there is nothing to fuse.
#pragma once

#include <cmath>

namespace numeric {
    [[nodiscard]] inline double sin(double angle_rad) noexcept {
        const volatile double angle = angle_rad;
        return std::sin(angle);
    }

    [[nodiscard]] inline double cos(double angle_rad) noexcept {
        const volatile double angle = angle_rad;
        return std::cos(angle);
    }
} // namespace numeric
