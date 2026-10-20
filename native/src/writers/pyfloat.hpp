// pyfloat.hpp — floating-point helpers for Python-faithful ports.
//
// CPython's `float.__pow__` lowers to the platform libm `pow()` for finite
// positive bases (Objects/floatobject.c float_pow). Ports must call the SAME
// libm symbol — but under optimization clang/GCC fold pow(x, 2.0) into x*x,
// which is not guaranteed bitwise-equal to libm pow(x, 2.0). Routing the
// call through a volatile function pointer keeps the call site opaque, so
// every `a ** b` in the Python source lands on libm pow(a, b) exactly.
#pragma once

#include <cerrno>
#include <cmath>
#include <stdexcept>

namespace pyfloat {
    inline double pow(double base, double exp) {
        static double (*const volatile fn)(double, double) = &std::pow;
        return fn(base, exp);
    }

    // CPython `float ** float` — the complete float_pow special-case
    // ladder (Objects/floatobject.c:719-836, v3.13.16) wrapped around the
    // opaque libm call above, so a port site behaves like Python's `**`
    // for every input, not just the finite-positive-base fast path:
    //
    //   iw == 0            -> 1.0 (even 0**0, inf**0, nan**0)
    //   isnan(iv)          -> iv
    //   isnan(iw)          -> iv == 1.0 ? 1.0 : iw
    //   isinf(iw)          -> |iv|==1 -> 1.0; (iw>0)==(|iv|>1) -> inf;
    //                         else 0.0
    //   isinf(iv)          -> iw>0 -> (iw odd-int ? iv : |iv|);
    //                         iw<0 -> (iw odd-int ? copysign(0,iv) : 0)
    //   iv == 0.0          -> iw<0 raises; iw odd-int -> iv (keeps -0.0);
    //                         else 0.0
    //   iv < 0.0           -> iw non-integer: Python delegates to
    //                         complex_pow (a complex result, not an
    //                         exception). Unreachable under a validated
    //                         tyre spec (pressure_bar > 0); the port
    //                         throws std::domain_error instead.
    //                         integer iw: iv = -iv, negate_result = odd.
    //   iv == 1.0          -> negate_result ? -1.0 : 1.0
    //   else               -> libm pow, then _Py_ADJUST_ERANGE1: a
    //                         +-inf result with errno 0 forces ERANGE
    //                         (OverflowError), an ERANGE underflow to 0
    //                         clears it; any other errno raises.
    //
    // DOUBLE_IS_ODD_INTEGER(x) is fmod(fabs(x), 2.0) == 1.0 — correct
    // for huge integer exponents where a cast would overflow.
    [[nodiscard]] inline double float_pow(double iv, double iw) {
        if (iw == 0.0) return 1.0;
        if (std::isnan(iv)) return iv;
        if (std::isnan(iw)) return iv == 1.0 ? 1.0 : iw;
        if (std::isinf(iw)) {
            const double aiv = std::fabs(iv);
            if (aiv == 1.0) return 1.0;
            if ((iw > 0.0) == (aiv > 1.0)) return std::fabs(iw);
            return 0.0;
        }
        if (std::isinf(iv)) {
            const bool odd = std::fmod(std::fabs(iw), 2.0) == 1.0;
            if (iw > 0.0) return odd ? iv : std::fabs(iv);
            return odd ? std::copysign(0.0, iv) : 0.0;
        }
        bool negate_result = false;
        if (iv == 0.0) {
            if (iw < 0.0)
                // CPython 3.13 float_pow raises ZeroDivisionError with
                // this spelling; 3.14 shortened it to
                // "zero to a negative power".
                throw std::domain_error(
                        "0.0 cannot be raised to a negative power");
            return std::fmod(std::fabs(iw), 2.0) == 1.0 ? iv : 0.0;
        }
        if (iv < 0.0) {
            if (iw != std::floor(iw))
                // Python returns a complex here; no double can carry it.
                throw std::domain_error(
                        "negative base to a fractional power");
            iv = -iv;
            negate_result = std::fmod(std::fabs(iw), 2.0) == 1.0;
        }
        if (iv == 1.0) return negate_result ? -1.0 : 1.0;
        errno = 0;
        double ix = pow(iv, iw);
        // _Py_ADJUST_ERANGE1(ix) (pycore_pymath.h:33-41).
        if (errno == 0) {
            if (std::isinf(ix)) errno = ERANGE; // x == +-Py_INFINITY
        } else if (errno == ERANGE && ix == 0.0) {
            errno = 0;
        }
        if (negate_result) ix = -ix;
        if (errno != 0) {
            if (errno == ERANGE)
                throw std::overflow_error("math range error");
            throw std::domain_error("math domain error");
        }
        return ix;
    }
} // namespace pyfloat
