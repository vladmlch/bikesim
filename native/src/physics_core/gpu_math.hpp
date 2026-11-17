// native/src/physics_core/gpu_math.hpp — Tri-target portable FP32 math primitives
// over strict IEEE + - * / floor ldexp (no FMA, no platform expf/tanhf/hypot divergence).
#pragma once

#include "common.hpp"

#if !defined(__METAL_VERSION__) && !defined(__CUDACC__)
#include <bit>
#include <cmath>
#elif defined(__CUDACC__)
#include <cmath>
#include <cstring>
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

BIKE_HD u32 pf_f32_to_u32(float x) noexcept {
#if defined(__METAL_VERSION__)
    return as_type<u32>(x);
#elif defined(__CUDA_ARCH__)
    return static_cast<u32>(__float_as_uint(x));
#elif defined(__CUDACC__)
    u32 u = 0;
    std::memcpy(&u, &x, sizeof(u));
    return u;
#else
    return std::bit_cast<u32>(x);
#endif
}

BIKE_HD float pf_u32_to_f32(u32 u) noexcept {
#if defined(__METAL_VERSION__)
    return as_type<float>(u);
#elif defined(__CUDA_ARCH__)
    return __uint_as_float(u);
#elif defined(__CUDACC__)
    float x = 0.0f;
    std::memcpy(&x, &u, sizeof(x));
    return x;
#else
    return std::bit_cast<float>(u);
#endif
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
    // NOLINTNEXTLINE(modernize-use-std-numbers) bitwise-locked Cody-Waite split; MSL has no <numbers>
    const float z = pf_floor(1.44269504088896341f * x + 0.5f);
    const int n = static_cast<int>(z);
    // NOLINTNEXTLINE(modernize-use-std-numbers) bitwise-locked Cody-Waite split; MSL has no <numbers>
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

// Cephes-style natural log over IEEE bit-extraction + strict + - * (no FMA, no platform logf).
BIKE_HD float logf_portable(float x) noexcept {
    if (pf_isnan(x)) {
        return x;
    }
    if (x == 0.0f) {
        return -(3.402823466e+38f * 2.0f);
    }
    if (x < 0.0f) {
        return pf_u32_to_f32(0x7fc00000u);
    }
    if (pf_isinf(x)) {
        return x;
    }
    u32 bits = pf_f32_to_u32(x);
    int e = 0;
    if ((bits & 0x7f800000u) == 0u) {
        x *= 8388608.0f;
        bits = pf_f32_to_u32(x);
        e = -23;
    }
    e += static_cast<int>(bits >> 23u) - 126;
    float m = pf_u32_to_f32((bits & 0x007fffffu) | 0x3f000000u);
    // NOLINTNEXTLINE(modernize-use-std-numbers) bitwise-locked Cephes threshold sqrt(0.5); MSL has no <numbers>
    const float kSqrtHf = 0.707106781186547524f;
    if (m < kSqrtHf) {
        m *= 2.0f;
        e -= 1;
    }
    const float z = m - 1.0f;
    const float z2 = z * z;
    float y = 7.0376836292e-2f;
    y = y * z - 1.1514610310e-1f;
    y = y * z + 1.1676998740e-1f;
    y = y * z - 1.2420140846e-1f;
    y = y * z + 1.4249322787e-1f;
    y = y * z - 1.6668057665e-1f;
    y = y * z + 2.0000714765e-1f;
    y = y * z - 2.4999993993e-1f;
    y = y * z + 3.3333331174e-1f;
    y = y * z * z2;
    // NOLINTNEXTLINE(modernize-use-std-numbers) bitwise-locked Cody-Waite split; MSL has no <numbers>
    const float kLn2Hi = 0.693359375f;
    const float kLn2Lo = -2.12194440e-4f;
    const float ef = static_cast<float>(e);
    y += ef * kLn2Lo;
    y -= 0.5f * z2;
    return z + y + ef * kLn2Hi;
}

BIKE_HD float powf_portable(float base, float exp) noexcept {
    if (exp == 0.0f || base == 1.0f) {
        return 1.0f;
    }
    if (pf_isnan(base) || pf_isnan(exp)) {
        return pf_u32_to_f32(0x7fc00000u);
    }
    if (base == 0.0f) {
        return exp > 0.0f ? 0.0f : (3.402823466e+38f * 2.0f);
    }
    return expf_portable(exp * logf_portable(base));
}

// Portable exp(x) - 1 for small x with |x| <= ln(2)/2 without 1.0f cancellation.
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
        // Avoids (1 + u + ...) - 1 cancellation when 2*|x| < ln(2)/2.
        const float d = expm1f_small_portable(u);
        return sign * (d / (2.0f + d));
    }
    if (u < 1.03972077f) {
        // z = 1 Cody-Waite reduction (2*|x| < 1.5*ln(2)): avoids 1-bit cancellation in e^u - 1.
        // NOLINTNEXTLINE(modernize-use-std-numbers) bitwise-locked Cody-Waite split; MSL has no <numbers>
        float r = u - 0.693359375f;
        r -= -2.12194440e-4f;
        const float d = 1.0f + 2.0f * expm1f_small_portable(r);
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
