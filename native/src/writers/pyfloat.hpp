// pyfloat.hpp — floating-point helpers for Python-faithful ports.
//
// CPython's `float.__pow__` lowers to the platform libm `pow()` for finite
// positive bases (Objects/floatobject.c float_pow). Ports must call the SAME
// libm symbol — but under optimization clang/GCC fold pow(x, 2.0) into x*x,
// which is not guaranteed bitwise-equal to libm pow(x, 2.0). Routing the
// call through a volatile function pointer keeps the call site opaque, so
// every `a ** b` in the Python source lands on libm pow(a, b) exactly.
#pragma once

#include <cmath>

namespace pyfloat {

inline double pow(double base, double exp) {
    static double (*const volatile fn)(double, double) = &std::pow;
    return fn(base, exp);
}

}  // namespace pyfloat
