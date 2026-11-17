// native/src/physics_core/gpu_math.hpp — Tri-target portable FP32 math primitives
// over strict IEEE + - * / floor ldexp (no FMA, no platform expf/tanhf/hypot divergence).
#pragma once

#include "common.hpp"

#if !defined(__METAL_VERSION__)
#include <cmath>
#endif

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel math: positional float arguments match the CUDA/MSL ABI. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

BIKE_HD float pf_abs(float x) noexcept {
#if defined(__METAL_VERSION__)
    return metal::fabs(x);
#elif defined(__CUDACC__)
    return ::fabsf(x);
#else
    return std::fabs(x);
#endif
}

BIKE_HD float pf_sqrt(float x) noexcept {
#if defined(__METAL_VERSION__)
    return metal::sqrt(x);
#elif defined(__CUDACC__)
    return ::sqrtf(x);
#else
    return std::sqrt(x);
#endif
}

BIKE_HD float pf_floor(float x) noexcept {
#if defined(__METAL_VERSION__)
    return metal::floor(x);
#elif defined(__CUDACC__)
    return ::floorf(x);
#else
    return std::floor(x);
#endif
}

BIKE_HD float pf_ldexp(float x, int n) noexcept {
#if defined(__METAL_VERSION__)
    return metal::ldexp(x, n);
#elif defined(__CUDACC__)
    return ::ldexpf(x, n);
#else
    return std::ldexp(x, n);
#endif
}

BIKE_HD bool pf_isfinite(float x) noexcept {
#if defined(__METAL_VERSION__)
    return metal::isfinite(x);
#elif defined(__CUDACC__)
    return ::isfinite(x) != 0;
#else
    return std::isfinite(x);
#endif
}

BIKE_HD bool pf_isnan(float x) noexcept {
#if defined(__METAL_VERSION__)
    return metal::isnan(x);
#elif defined(__CUDACC__)
    return ::isnan(x) != 0;
#else
    return std::isnan(x);
#endif
}

BIKE_HD bool pf_isinf(float x) noexcept {
    return !pf_isfinite(x) && !pf_isnan(x);
}

BIKE_HD float pf_min(float a, float b) noexcept {
#if defined(__METAL_VERSION__)
    return metal::min(a, b);
#elif defined(__CUDACC__)
    return ::fminf(a, b);
#else
    return std::fminf(a, b);
#endif
}

BIKE_HD float pf_max(float a, float b) noexcept {
#if defined(__METAL_VERSION__)
    return metal::max(a, b);
#elif defined(__CUDACC__)
    return ::fmaxf(a, b);
#else
    return std::fmaxf(a, b);
#endif
}

BIKE_HD float pf_clamp(float x, float lo, float hi) noexcept {
    // Preserves -0.0f when x == -0.0f and lo == 0.0f (matching NumPy clip01).
    return x < lo ? lo : (x > hi ? hi : x);
}

// Cephes-style Cody-Waite range reduction over strict + - * floor ldexp (no FMA).
BIKE_HD float expf_portable(float x) noexcept {
    if (pf_isnan(x)) {
        return x;
    }
    if (x > 88.3762626647949f) {
        return 3.402823466e+38f * 2.0f;
    }
    if (x < -103.97207708399179f) {
        return 0.0f;
    }
    const float z = pf_floor(1.44269504088896341f * x + 0.5f);
    const int n = static_cast<int>(z);
    x -= z * 0.693359375f;
    x -= z * (-2.12194440e-4f);
    const float xx = x * x;
    float px = 1.9875691500e-4f;
    px = px * x + 1.3981999507e-3f;
    px = px * x + 8.3334519073e-3f;
    px = px * x + 4.1665795894e-2f;
    px = px * x + 1.6666665459e-1f;
    px = px * x + 5.0000001201e-1f;
    const float ex = 1.0f + x + px * xx;
    return pf_ldexp(ex, n);
}

// Portable exp(x) - 1 for small non-negative x in [0, ln(2)/2) without 1.0f cancellation.
BIKE_HD float expm1f_small_portable(float x) noexcept {
    const float xx = x * x;
    float px = 1.9875691500e-4f;
    px = px * x + 1.3981999507e-3f;
    px = px * x + 8.3334519073e-3f;
    px = px * x + 4.1665795894e-2f;
    px = px * x + 1.6666665459e-1f;
    px = px * x + 5.0000001201e-1f;
    return x + px * xx;
}

BIKE_HD float tanhf_portable(float x) noexcept {
    if (pf_isnan(x)) {
        return x;
    }
    if (x == 0.0f) {
        return x;
    }
    const float ax = pf_abs(x);
    const float sign = (x < 0.0f) ? -1.0f : 1.0f;
    if (ax >= 10.0f) {
        return sign;
    }
    if (ax < 1.0e-4f) {
        return x * (1.0f - x * x * (1.0f / 3.0f));
    }
    const float u = 2.0f * ax;
    if (u < 0.34657359f) {
        // Avoids (1 + u + ...) - 1 cancellation when 2*ax < ln(2)/2.
        const float d = expm1f_small_portable(u);
        return sign * (d / (2.0f + d));
    }
    const float e2x = expf_portable(u);
    return sign * ((e2x - 1.0f) / (e2x + 1.0f));
}

BIKE_HD float hypot2f_portable(float a, float b) noexcept {
    const float aa = pf_abs(a);
    const float ab = pf_abs(b);
    if (pf_isinf(aa)) {
        return aa;
    }
    if (pf_isinf(ab)) {
        return ab;
    }
    if (pf_isnan(aa)) {
        return aa;
    }
    if (pf_isnan(ab)) {
        return ab;
    }
    const float m = aa > ab ? aa : ab;
    const float n = aa > ab ? ab : aa;
    if (m == 0.0f) {
        return 0.0f;
    }
    const float r = n / m;
    return m * pf_sqrt(1.0f + r * r);
}

BIKE_HD float interp_segment_unfused(
    float s, float x, float x0, float y0, float x1, float y1
) noexcept {
    if (y0 == y1) {
        return y0;
    }
    float res = y0 + s * (x - x0);
    if (pf_isnan(res)) {
        res = y1 + s * (x - x1);
    }
    return res;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
