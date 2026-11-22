// native/src/physics_core/tyre_pneumatic_geometry.hpp — Tri-target (Host C++20/23,
// CUDA, Apple Metal MSL) pneumatic tyre ray-ring generation, ray/road polyline
// intersection, contiguous patch extraction, and portable transcendental helpers.
#pragma once

#include "common.hpp"
#include "gpu_math.hpp"
#include "tyre_types.hpp"

#if !defined(__METAL_VERSION__) && !defined(__CUDACC__)
#include <cmath>
#include <limits>
#endif

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel header: MSL has no std::span/std::array and device buffers are address-space-qualified raw pointers. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

BIKE_CONSTEXPR i32 kMaxPneumaticRays = 256;
BIKE_CONSTEXPR i32 kMaxPneumaticPatches = 16;

namespace pneumatic_detail {

BIKE_HD i32 max_i32(i32 a, i32 b) noexcept {
    return (a < b) ? b : a;
}

BIKE_HD i32 min_i32(i32 a, i32 b) noexcept {
    return (b < a) ? b : a;
}

BIKE_HD i32 clamp_i32(i32 v, i32 lo, i32 hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

BIKE_HD float pos_inf_of(float) noexcept {
    return pf_u32_to_f32(0x7f800000u);
}

#if !defined(__METAL_VERSION__) && !defined(__CUDACC__)
inline double pos_inf_of(double) noexcept {
    return std::numeric_limits<double>::infinity();
}
#endif

template <class Real>
BIKE_HD Real pos_inf() noexcept {
    return pos_inf_of(Real{});
}

template <class Real>
BIKE_HD Real qnan_val() noexcept {
    return static_cast<Real>(pf_u32_to_f32(0x7fc00000u));
}

BIKE_HD float flush_subnorm_f32(float x) noexcept {
    const u32 u = pf_f32_to_u32(x);
    return ((u & 0x7f800000u) == 0u) ? pf_u32_to_f32(u & 0x80000000u) : x;
}

BIKE_HD float flush_subnorm(float x) noexcept {
    return flush_subnorm_f32(x);
}

#if !defined(__METAL_VERSION__) && !defined(__CUDACC__)
inline double flush_subnorm(double x) noexcept {
    return x;
}
#endif

BIKE_HD bool isinf_val(float x) noexcept {
    return pf_isinf(x);
}

#if !defined(__METAL_VERSION__) && !defined(__CUDACC__)
inline bool isinf_val(double x) noexcept {
    return std::isinf(x);
}
#endif

BIKE_HD float copysign_val(float mag, float sign_src) noexcept {
    const u32 m_bits = pf_f32_to_u32(mag) & 0x7fffffffu;
    const u32 s_bits = pf_f32_to_u32(sign_src) & 0x80000000u;
    return pf_u32_to_f32(m_bits | s_bits);
}

#if !defined(__METAL_VERSION__) && !defined(__CUDACC__)
inline double copysign_val(double mag, double sign_src) noexcept {
    return std::copysign(mag, sign_src);
}
#endif

BIKE_HD float ceil_val_of(float x) noexcept {
#if defined(__METAL_VERSION__)
    return metal::ceil(x);
#elif defined(__CUDACC__)
    return ::ceilf(x);
#else
    return std::ceilf(x);
#endif
}

#if !defined(__METAL_VERSION__) && !defined(__CUDACC__)
inline double ceil_val_of(double x) noexcept {
    return std::ceil(x);
}
#endif

template <class Real, class TK>
BIKE_HD Real ceil_val(Real x) noexcept {
    return ceil_val_of(x);
}

BIKE_HD float hypot_libm_of(float a, float b) noexcept {
    return hypot2f_portable(a, b);
}

#if !defined(__METAL_VERSION__) && !defined(__CUDACC__)
inline double hypot_libm_of(double a, double b) noexcept {
    return std::hypot(a, b);
}
#endif

template <class Real, class TK>
BIKE_HD Real hypot_libm(Real a, Real b) noexcept {
    return hypot_libm_of(a, b);
}

BIKE_HD float idx_bound_for(float) noexcept {
    return 1.0e9f;
}

#if !defined(__METAL_VERSION__) && !defined(__CUDACC__)
inline double idx_bound_for(double) noexcept {
    return 2.0e9;
}
#endif

template <class Real, class TK>
BIKE_HD Real portable_atan(Real x_in) noexcept {
    const Real x = flush_subnorm(x_in);
    if (TK::isnan(x)) {
        return x;
    }
    if (x == static_cast<Real>(0)) {
        return x;
    }
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kPi2 = BIKE_REAL(Real, 1.57079632679489661923);
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kPi4 = BIKE_REAL(Real, 0.78539816339744830962);
    const Real ax = TK::abs(x);
    const Real sign = (x < static_cast<Real>(0))
        ? static_cast<Real>(-1)
        : static_cast<Real>(1);
    if (!TK::isfinite(ax)) {
        return sign * kPi2;
    }
    Real y = static_cast<Real>(0);
    Real z = ax;
    if (ax > BIKE_REAL(Real, 2.41421356237309504880)) {
        y = kPi2;
        z = static_cast<Real>(-1) / ax;
    } else if (ax > BIKE_REAL(Real, 0.41421356237309504880)) {
        y = kPi4;
        z = (ax - static_cast<Real>(1)) / (ax + static_cast<Real>(1));
    }
    const Real z2 = z * z;
    Real p = BIKE_REAL(Real, 8.05374449538e-2);
    p = p * z2 - BIKE_REAL(Real, 1.38776856032e-1);
    p = p * z2 + BIKE_REAL(Real, 1.99777106478e-1);
    p = p * z2 - BIKE_REAL(Real, 3.33329491539e-1);
    p = p * z2 * z + z;
    return sign * (y + p);
}

template <class Real, class TK>
BIKE_HD Real portable_atan2(Real y_in, Real x_in) noexcept {
    const Real y = flush_subnorm(y_in);
    const Real x = flush_subnorm(x_in);
    if (TK::isnan(y) || TK::isnan(x)) {
        return qnan_val<Real>();
    }
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kPi = BIKE_REAL(Real, 3.14159265358979323846);
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kPi2 = BIKE_REAL(Real, 1.57079632679489661923);
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kPi4 = BIKE_REAL(Real, 0.78539816339744830962);
    if (x == static_cast<Real>(0)) {
        if (y > static_cast<Real>(0)) {
            return kPi2;
        }
        if (y < static_cast<Real>(0)) {
            return -kPi2;
        }
        return static_cast<Real>(0);
    }
    if (y == static_cast<Real>(0)) {
        return (x < static_cast<Real>(0)) ? kPi : y;
    }
    if (!TK::isfinite(x) && !TK::isfinite(y)) {
        const Real base = (x < static_cast<Real>(0))
            ? (BIKE_REAL(Real, 3.0) * kPi4)
            : kPi4;
        return (y < static_cast<Real>(0)) ? -base : base;
    }
    if (!TK::isfinite(x)) {
        return (x > static_cast<Real>(0))
            ? static_cast<Real>(0)
            : (y < static_cast<Real>(0) ? -kPi : kPi);
    }
    if (!TK::isfinite(y)) {
        return (y > static_cast<Real>(0)) ? kPi2 : -kPi2;
    }
    const Real r = portable_atan<Real, TK>(y / x);
    if (x < static_cast<Real>(0)) {
        return (y >= static_cast<Real>(0)) ? (r + kPi) : (r - kPi);
    }
    return r;
}

template <class Real, class TK>
BIKE_HD Real portable_sin(Real x_in) noexcept {
    const Real x = flush_subnorm(x_in);
    if (!TK::isfinite(x)) {
        return qnan_val<Real>();
    }
    if (x == static_cast<Real>(0)) {
        return x;
    }
    const Real sign = (x < static_cast<Real>(0))
        ? static_cast<Real>(-1)
        : static_cast<Real>(1);
    const Real ax = TK::abs(x);
    if (ax > BIKE_REAL(Real, 1e6)) {
        return qnan_val<Real>();
    }
    const Real k = TK::floor(
        ax * BIKE_REAL(Real, 0.63661977236758134308) + BIKE_REAL(Real, 0.5));
    const u32 j = static_cast<u32>(static_cast<i32>(k));
    const Real dp1 = BIKE_REAL(Real, 1.5703125);
    const Real dp2 = BIKE_REAL(Real, 4.837512969970703125e-4);
    const Real dp3 = BIKE_REAL(Real, 7.549789948768648e-8);
    const Real r = ((ax - k * dp1) - k * dp2) - k * dp3;
    const Real r2 = r * r;
    const u32 q = j & 3u;
    if ((q & 1u) == 0u) {
        const Real s = r + r * r2 *
            (BIKE_REAL(Real, -1.6666654611e-1) +
             r2 * (BIKE_REAL(Real, 8.3321608736e-3) -
                   r2 * BIKE_REAL(Real, 1.9515295891e-4)));
        return (q == 2u) ? (-sign * s) : (sign * s);
    }
    const Real c = static_cast<Real>(1) - BIKE_REAL(Real, 0.5) * r2 +
        r2 * r2 *
            (BIKE_REAL(Real, 4.166664568298827e-2) +
             r2 * (BIKE_REAL(Real, -1.388731625493765e-3) +
                   r2 * BIKE_REAL(Real, 2.443315711809948e-5)));
    return (q == 3u) ? (-sign * c) : (sign * c);
}

template <class Real, class TK>
BIKE_HD Real portable_cos(Real x_in) noexcept {
    const Real x = flush_subnorm(x_in);
    if (!TK::isfinite(x)) {
        return qnan_val<Real>();
    }
    const Real ax = TK::abs(x);
    if (ax > BIKE_REAL(Real, 1e6)) {
        return qnan_val<Real>();
    }
    const Real k = TK::floor(
        ax * BIKE_REAL(Real, 0.63661977236758134308) + BIKE_REAL(Real, 0.5));
    const u32 j = static_cast<u32>(static_cast<i32>(k));
    const Real dp1 = BIKE_REAL(Real, 1.5703125);
    const Real dp2 = BIKE_REAL(Real, 4.837512969970703125e-4);
    const Real dp3 = BIKE_REAL(Real, 7.549789948768648e-8);
    const Real r = ((ax - k * dp1) - k * dp2) - k * dp3;
    const Real r2 = r * r;
    const u32 q = j & 3u;
    if ((q & 1u) == 0u) {
        const Real c = static_cast<Real>(1) - BIKE_REAL(Real, 0.5) * r2 +
            r2 * r2 *
                (BIKE_REAL(Real, 4.166664568298827e-2) +
                 r2 * (BIKE_REAL(Real, -1.388731625493765e-3) +
                       r2 * BIKE_REAL(Real, 2.443315711809948e-5)));
        return (q == 2u) ? -c : c;
    }
    const Real s = r + r * r2 *
        (BIKE_REAL(Real, -1.6666654611e-1) +
         r2 * (BIKE_REAL(Real, 8.3321608736e-3) -
               r2 * BIKE_REAL(Real, 1.9515295891e-4)));
    return (q == 1u) ? -s : s;
}

template <class TK>
BIKE_HD float sin_val_of(float x) noexcept {
    return portable_sin<float, TK>(x);
}

#if !defined(__METAL_VERSION__) && !defined(__CUDACC__)
template <class TK>
inline double sin_val_of(double x) noexcept {
    return std::sin(x);
}
#endif

template <class Real, class TK>
BIKE_HD Real sin_val(Real x) noexcept {
    return sin_val_of<TK>(x);
}

template <class TK>
BIKE_HD float cos_val_of(float x) noexcept {
    return portable_cos<float, TK>(x);
}

#if !defined(__METAL_VERSION__) && !defined(__CUDACC__)
template <class TK>
inline double cos_val_of(double x) noexcept {
    return std::cos(x);
}
#endif

template <class Real, class TK>
BIKE_HD Real cos_val(Real x) noexcept {
    return cos_val_of<TK>(x);
}

template <class TK>
BIKE_HD float ring_sin_val_of(float x) noexcept {
    return portable_sin<float, TK>(x);
}

template <class TK>
BIKE_HD float ring_cos_val_of(float x) noexcept {
    return portable_cos<float, TK>(x);
}

#if !defined(__METAL_VERSION__) && !defined(__CUDACC__)
template <class TK>
inline double ring_sin_val_of(double x) noexcept {
    const volatile double angle = x;
    return std::sin(angle);
}

template <class TK>
inline double ring_cos_val_of(double x) noexcept {
    const volatile double angle = x;
    return std::cos(angle);
}
#endif

template <class TK>
BIKE_HD float atan2_val_of(float y, float x) noexcept {
    return portable_atan2<float, TK>(y, x);
}

#if !defined(__METAL_VERSION__) && !defined(__CUDACC__)
template <class TK>
inline double atan2_val_of(double y, double x) noexcept {
    return std::atan2(y, x);
}
#endif

template <class Real, class TK>
BIKE_HD Real atan2_val(Real y, Real x) noexcept {
    return atan2_val_of<TK>(y, x);
}

template <class TK>
BIKE_HD float asin_val_of(float x) noexcept {
    if (TK::isnan(x) || x > 1.0f || x < -1.0f) {
        return qnan_val<float>();
    }
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const float kPi2 = 1.57079632679489661923f;
    if (x == 1.0f) {
        return kPi2;
    }
    if (x == -1.0f) {
        return -kPi2;
    }
    const float c_sq = TK::max(0.0f, (1.0f - x) * (1.0f + x));
    return portable_atan2<float, TK>(x, TK::sqrt(c_sq));
}

#if !defined(__METAL_VERSION__) && !defined(__CUDACC__)
template <class TK>
inline double asin_val_of(double x) noexcept {
    return std::asin(x);
}
#endif

template <class Real, class TK>
BIKE_HD Real asin_val(Real x) noexcept {
    return asin_val_of<TK>(x);
}

template <class Real, class TK, class PtrVals>
BIKE_HD Real np_interp_unit_grid(
    Real x,
    PtrVals values,
    i32 n,
    Real lval,
    Real rval
) noexcept {
    if (n <= 0) {
        return static_cast<Real>(0);
    }
    if (n == 1) {
        if (x < static_cast<Real>(0)) {
            return lval;
        }
        if (x > static_cast<Real>(0)) {
            return rval;
        }
        return values[0];
    }
    if (TK::isnan(x)) {
        return x;
    }
    const Real last = static_cast<Real>(n - 1);
    if (x < static_cast<Real>(0)) {
        return lval;
    }
    if (x > last) {
        return rval;
    }
    const i32 j = static_cast<i32>(x);
    if (j == n - 1) {
        return values[n - 1];
    }
    const Real x0 = static_cast<Real>(j);
    if (x0 == x) {
        return values[j];
    }
    const Real slope = values[j + 1] - values[j];
    return TK::interp_segment(
        slope, x, x0, values[j], static_cast<Real>(j + 1), values[j + 1]);
}

template <class Real, class TK, class PtrVals>
BIKE_HD Real array_sum(i32 n, PtrVals vals) noexcept {
    Real acc = static_cast<Real>(0);
    for (i32 i = 0; i < n; ++i) {
        acc += vals[i];
    }
    return acc;
}

template <class Real, class TK, class PtrPx, class PtrPz>
BIKE_HD i32 phi_upper_bound(
    PtrPx xs,
    PtrPz zs,
    i32 base,
    i32 count,
    Real cx,
    Real cz,
    Real v
) noexcept {
    i32 first = 0;
    i32 len = count;
    while (len > 0) {
        const i32 step = len / 2;
        const i32 mid = first + step;
        const i32 g = base + mid;
        const Real phi_mid = atan2_val<Real, TK>(xs[g] - cx, cz - zs[g]);
        if (v < phi_mid) {
            len = step;
        } else {
            first = mid + 1;
            len -= step + 1;
        }
    }
    return first;
}

template <class Real, class PtrX>
BIKE_HD int search_right(PtrX x, i32 n, Real v) noexcept {
    i32 first = 0;
    i32 count = n;
    while (count > 0) {
        const i32 step = count / 2;
        const i32 mid = first + step;
        if (!(v < x[mid])) {
            first = mid + 1;
            count -= step + 1;
        } else {
            count = step;
        }
    }
    return first;
}

template <class Real, class PtrX>
BIKE_HD int search_left(PtrX x, i32 n, Real v) noexcept {
    i32 first = 0;
    i32 count = n;
    while (count > 0) {
        const i32 step = count / 2;
        const i32 mid = first + step;
        if (x[mid] < v) {
            first = mid + 1;
            count -= step + 1;
        } else {
            count = step;
        }
    }
    return first;
}

} // namespace pneumatic_detail

template <class Real>
struct PneumaticRayRing {
    i32 n_rays = 0;
    Real half_angle_rad = static_cast<Real>(0);
    Real dtheta_rad = static_cast<Real>(0);
    Real theta_rad[kMaxPneumaticRays]{};
    Real ux[kMaxPneumaticRays]{};
    Real uz[kMaxPneumaticRays]{};
};

struct PneumaticRayPatch {
    i32 start = 0;
    i32 stop = 0;
};

template <class Real, class TK>
BIKE_HD Status init_pneumatic_ray_ring(
    i32 n_rays,
    Real half_angle_rad,
    BIKE_REF(PneumaticRayRing<Real>) out,
    BIKE_REF(ErrCode) err
) noexcept {
    out = PneumaticRayRing<Real>{};
    if (n_rays < 3 || n_rays > kMaxPneumaticRays) {
        err = ErrCode::out_of_bounds_station;
        return Status::invalid_argument;
    }
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real half_pi = BIKE_REAL(Real, 0.5) * BIKE_REAL(Real, 3.14159265358979323846);
    if (!TK::isfinite(half_angle_rad) ||
        !(static_cast<Real>(0) < half_angle_rad && half_angle_rad < half_pi)) {
        err = ErrCode::invalid_radius;
        return Status::invalid_argument;
    }
    out.n_rays = n_rays;
    out.half_angle_rad = half_angle_rad;
    const Real delta = half_angle_rad - (-half_angle_rad);
    const Real step = TK::div(delta, static_cast<Real>(n_rays - 1));
    for (i32 i = 0; i < n_rays; ++i) {
        out.theta_rad[i] = static_cast<Real>(i) * step + (-half_angle_rad);
    }
    out.theta_rad[n_rays - 1] = half_angle_rad;
    out.dtheta_rad = out.theta_rad[1] - out.theta_rad[0];
    for (i32 i = 0; i < n_rays; ++i) {
        const Real angle = out.theta_rad[i];
        out.ux[i] = pneumatic_detail::ring_sin_val_of<TK>(angle);
        out.uz[i] = -pneumatic_detail::ring_cos_val_of<TK>(angle);
    }
    err = ErrCode::none;
    return Status::ok;
}

template <
    class Real,
    class TK,
    class PtrTheta,
    class PtrUx,
    class PtrUz,
    class PtrPx,
    class PtrPz,
    class PtrR,
    class PtrDelta,
    class PtrRx,
    class PtrRz>
BIKE_HD Status pneumatic_intersect_window_arrays(
    i32 n_rays,
    Real half_angle_rad,
    PtrTheta theta_rad,
    PtrUx ux,
    PtrUz uz,
    Real centre_x_m,
    Real centre_z_m,
    Real radius_m,
    PtrPx xs,
    PtrPz zs,
    i32 lo,
    i32 hi,
    PtrR out_r_m,
    PtrDelta out_delta_m,
    PtrRx out_road_x_m,
    PtrRz out_road_z_m,
    BIKE_REF(u8) out_coverage_event,
    BIKE_REF(u8) out_airborne,
    BIKE_REF(ErrCode) err
) noexcept {
    out_coverage_event = 0;
    out_airborne = 0;
    if (n_rays < 3 || theta_rad == nullptr || ux == nullptr || uz == nullptr ||
        xs == nullptr || zs == nullptr || out_r_m == nullptr ||
        out_delta_m == nullptr || out_road_x_m == nullptr ||
        out_road_z_m == nullptr) {
        err = ErrCode::out_of_bounds_station;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(centre_x_m) || !TK::isfinite(centre_z_m)) {
        err = ErrCode::nonfinite_center;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(radius_m) || radius_m < static_cast<Real>(0)) {
        err = ErrCode::invalid_radius;
        return Status::invalid_argument;
    }

    const i32 n = n_rays;
    const Real inf_val = pneumatic_detail::pos_inf<Real>();
    bool coverage = false;
    bool airborne = false;

    if (hi - lo < 2) {
        for (i32 i = 0; i < n; ++i) {
            out_r_m[i] = inf_val;
        }
        airborne = true;
    } else {
        const i32 glo = lo;
        const i32 w = hi - lo;
        const Real r2 = radius_m * radius_m;
        i32 first = w;
        i32 last = 0;
        for (i32 k = 0; k < w; ++k) {
            const i32 g = glo + k;
            const Real dxs = xs[g] - centre_x_m;
            const Real dzs = centre_z_m - zs[g];
            if (dxs * dxs + dzs * dzs < r2) {
                if (first == w) {
                    first = k;
                }
                last = k;
            }
        }
        if (first == w) {
            for (i32 i = 0; i < n; ++i) {
                out_r_m[i] = inf_val;
            }
            airborne = true;
        } else {
            const i32 a = first > 0 ? first - 1 : 0;
            const i32 b = pneumatic_detail::min_i32(last + 2, w);
            const i32 base = glo + a;
            const i32 win = b - a;
            const Real cov_limit =
                half_angle_rad + BIKE_REAL(Real, 1e-9);
            bool monotone = true;
            Real prev_phi = static_cast<Real>(0);
            for (i32 j = 0; j < win; ++j) {
                const i32 g = base + j;
                const Real px = xs[g];
                const Real pz = zs[g];
                const Real phi = pneumatic_detail::atan2_val<Real, TK>(
                    px - centre_x_m, centre_z_m - pz);
                const Real dxs = px - centre_x_m;
                const Real dzs = centre_z_m - pz;
                if (dxs * dxs + dzs * dzs < r2) {
                    if (TK::abs(phi) > cov_limit || pz >= centre_z_m) {
                        coverage = true;
                    }
                }
                if (j > 0 && !(phi - prev_phi > static_cast<Real>(0))) {
                    monotone = false;
                }
                prev_phi = phi;
            }

            if (win >= 2 && monotone) {
                for (i32 i = 0; i < n; ++i) {
                    const i32 ub = pneumatic_detail::phi_upper_bound<Real, TK>(
                        xs, zs, base, win, centre_x_m, centre_z_m,
                        theta_rad[i]);
                    const i32 j = ub - 1;
                    const bool valid = j >= 0 && j < win - 1;
                    const i32 jj = pneumatic_detail::clamp_i32(j, 0, win - 2);
                    const i32 g = base + jj;
                    const Real x1 = xs[g];
                    const Real z1 = zs[g];
                    const Real dx = xs[g + 1] - x1;
                    const Real dz = zs[g + 1] - z1;
                    const Real ux_i = ux[i];
                    const Real uz_i = uz[i];
                    const Real denom = ux_i * dz - uz_i * dx;
                    const Real s = TK::div(
                        (x1 - centre_x_m) * dz - (z1 - centre_z_m) * dx,
                        denom);
                    out_r_m[i] =
                        (valid && TK::abs(denom) > BIKE_REAL(Real, 1e-15) &&
                         s > static_cast<Real>(0))
                            ? s
                            : inf_val;
                }
            } else {
                const i32 segs = win - 1;
                for (i32 i = 0; i < n; ++i) {
                    Real best = inf_val;
                    const Real ux_i = ux[i];
                    const Real uz_i = uz[i];
                    for (i32 k = 0; k < segs; ++k) {
                        const i32 g = base + k;
                        const Real x1 = xs[g];
                        const Real z1 = zs[g];
                        const Real dx = xs[g + 1] - x1;
                        const Real dz = zs[g + 1] - z1;
                        const Real denom = ux_i * dz - uz_i * dx;
                        const Real wx = x1 - centre_x_m;
                        const Real wz = z1 - centre_z_m;
                        const Real s = TK::div(wx * dz - wz * dx, denom);
                        const Real t = TK::div(wx * uz_i - wz * ux_i, denom);
                        if (TK::abs(denom) > BIKE_REAL(Real, 1e-15) &&
                            s > static_cast<Real>(0) &&
                            t >= static_cast<Real>(0) &&
                            t <= static_cast<Real>(1) &&
                            s < best) {
                            best = s;
                        }
                    }
                    out_r_m[i] = best;
                }
            }
        }
    }

    for (i32 i = 0; i < n; ++i) {
        const Real r = out_r_m[i];
        const Real reach = TK::min(r, radius_m);
        out_delta_m[i] = TK::max(radius_m - r, static_cast<Real>(0));
        out_road_x_m[i] = centre_x_m + reach * ux[i];
        out_road_z_m[i] = centre_z_m + reach * uz[i];
    }
    out_coverage_event = coverage ? 1 : 0;
    out_airborne = airborne ? 1 : 0;
    err = ErrCode::none;
    return Status::ok;
}

template <
    class Real,
    class TK,
    class RingPtr,
    class PtrPx,
    class PtrPz,
    class PtrR,
    class PtrDelta,
    class PtrRx,
    class PtrRz>
BIKE_HD Status pneumatic_intersect_window(
    RingPtr ring,
    Real centre_x_m,
    Real centre_z_m,
    Real radius_m,
    PtrPx xs,
    PtrPz zs,
    i32 lo,
    i32 hi,
    PtrR out_r_m,
    PtrDelta out_delta_m,
    PtrRx out_road_x_m,
    PtrRz out_road_z_m,
    BIKE_REF(u8) out_coverage_event,
    BIKE_REF(u8) out_airborne,
    BIKE_REF(ErrCode) err
) noexcept {
    if (ring == nullptr || ring->n_rays < 3 || ring->n_rays > kMaxPneumaticRays) {
        out_coverage_event = 0;
        out_airborne = 0;
        err = ErrCode::out_of_bounds_station;
        return Status::invalid_argument;
    }
    return pneumatic_intersect_window_arrays<Real, TK>(
        ring->n_rays, ring->half_angle_rad,
        ring->theta_rad, ring->ux, ring->uz,
        centre_x_m, centre_z_m, radius_m,
        xs, zs, lo, hi,
        out_r_m, out_delta_m, out_road_x_m, out_road_z_m,
        out_coverage_event, out_airborne, err);
}

template <
    class Real,
    class TK,
    class RingPtr,
    class PtrPx,
    class PtrPz,
    class PtrR,
    class PtrDelta,
    class PtrRx,
    class PtrRz>
BIKE_HD Status pneumatic_intersect_uniform_grid(
    RingPtr ring,
    Real centre_x_m,
    Real centre_z_m,
    Real radius_m,
    Real x0_m,
    Real dx_m,
    i32 num_vertices,
    PtrPx xs,
    PtrPz zs,
    PtrR out_r_m,
    PtrDelta out_delta_m,
    PtrRx out_road_x_m,
    PtrRz out_road_z_m,
    BIKE_REF(u8) out_coverage_event,
    BIKE_REF(u8) out_airborne,
    BIKE_REF(ErrCode) err
) noexcept {
    if (num_vertices < 2 || !TK::isfinite(x0_m) || !TK::isfinite(dx_m) ||
        !(dx_m > static_cast<Real>(0))) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(centre_x_m) || !TK::isfinite(centre_z_m)) {
        err = ErrCode::nonfinite_center;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(radius_m) || radius_m < static_cast<Real>(0)) {
        err = ErrCode::invalid_radius;
        return Status::invalid_argument;
    }
    const Real lo_f = TK::floor(TK::div((centre_x_m - radius_m) - x0_m, dx_m));
    const Real hi_f =
        pneumatic_detail::ceil_val<Real, TK>(
            TK::div((centre_x_m + radius_m) - x0_m, dx_m));
    if (!TK::isfinite(lo_f) || !TK::isfinite(hi_f)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Real idx_bound = pneumatic_detail::idx_bound_for(Real{});
    const i32 lo_raw = static_cast<i32>(TK::clamp(lo_f, -idx_bound, idx_bound));
    const i32 hi_raw = static_cast<i32>(TK::clamp(hi_f, -idx_bound, idx_bound)) + 1;
    const i32 lo = pneumatic_detail::max_i32(0, lo_raw);
    const i32 hi = pneumatic_detail::min_i32(num_vertices, hi_raw);
    return pneumatic_intersect_window<Real, TK>(
        ring, centre_x_m, centre_z_m, radius_m, xs, zs, lo, hi,
        out_r_m, out_delta_m, out_road_x_m, out_road_z_m,
        out_coverage_event, out_airborne, err);
}

template <
    class Real,
    class TK,
    class RingPtr,
    class PtrPx,
    class PtrPz,
    class PtrR,
    class PtrDelta,
    class PtrRx,
    class PtrRz>
BIKE_HD Status pneumatic_intersect_polyline_arrays(
    RingPtr ring,
    Real centre_x_m,
    Real centre_z_m,
    Real radius_m,
    PtrPx px,
    PtrPz pz,
    i32 num_vertices,
    PtrR out_r_m,
    PtrDelta out_delta_m,
    PtrRx out_road_x_m,
    PtrRz out_road_z_m,
    BIKE_REF(u8) out_coverage_event,
    BIKE_REF(u8) out_airborne,
    BIKE_REF(ErrCode) err
) noexcept {
    if (num_vertices < 2 || px == nullptr || pz == nullptr) {
        err = ErrCode::degenerate_contact;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(centre_x_m) || !TK::isfinite(centre_z_m)) {
        err = ErrCode::nonfinite_center;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(radius_m) || radius_m < static_cast<Real>(0)) {
        err = ErrCode::invalid_radius;
        return Status::invalid_argument;
    }
    const i32 lo = pneumatic_detail::max_i32(
        0,
        pneumatic_detail::search_right<Real>(
            px, num_vertices, centre_x_m - radius_m) - 1);
    const i32 hi = pneumatic_detail::min_i32(
        num_vertices,
        pneumatic_detail::search_left<Real>(
            px, num_vertices, centre_x_m + radius_m) + 1);
    return pneumatic_intersect_window<Real, TK>(
        ring, centre_x_m, centre_z_m, radius_m, px, pz, lo, hi,
        out_r_m, out_delta_m, out_road_x_m, out_road_z_m,
        out_coverage_event, out_airborne, err);
}

template <
    class Real,
    class TK,
    class RingPtr,
    class PtrR,
    class PtrDelta,
    class PtrRx,
    class PtrRz>
BIKE_HD Status pneumatic_intersect_polyline(
    RingPtr ring,
    Real centre_x_m,
    Real centre_z_m,
    Real radius_m,
    PolylineView<Real> pl,
    PtrR out_r_m,
    PtrDelta out_delta_m,
    PtrRx out_road_x_m,
    PtrRz out_road_z_m,
    BIKE_REF(u8) out_coverage_event,
    BIKE_REF(u8) out_airborne,
    BIKE_REF(ErrCode) err
) noexcept {
    return pneumatic_intersect_polyline_arrays<Real, TK>(
        ring, centre_x_m, centre_z_m, radius_m, pl.px, pl.pz, pl.num_vertices,
        out_r_m, out_delta_m, out_road_x_m, out_road_z_m,
        out_coverage_event, out_airborne, err);
}

template <class Real, class PtrDelta, class PtrPatch>
BIKE_HD Status pneumatic_patches(
    i32 n_rays,
    PtrDelta delta_m,
    i32 max_patches,
    PtrPatch out_patches,
    BIKE_REF(i32) out_patch_count,
    BIKE_REF(ErrCode) err
) noexcept {
    out_patch_count = 0;
    if (n_rays < 0 || max_patches < 0 ||
        (n_rays > 0 && delta_m == nullptr) ||
        (max_patches > 0 && out_patches == nullptr)) {
        err = ErrCode::out_of_bounds_station;
        return Status::invalid_argument;
    }
    i32 count = 0;
    bool open = false;
    i32 start = 0;
    i32 prev = 0;
    for (i32 k = 0; k < n_rays; ++k) {
        if (!(delta_m[k] > static_cast<Real>(0))) {
            continue;
        }
        if (!open) {
            start = k;
            open = true;
        } else if (k - prev > 1) {
            if (count >= max_patches) {
                err = ErrCode::out_of_bounds_station;
                return Status::out_of_bounds;
            }
            out_patches[count].start = start;
            out_patches[count].stop = prev + 1;
            ++count;
            start = k;
        }
        prev = k;
    }
    if (open) {
        if (count >= max_patches) {
            err = ErrCode::out_of_bounds_station;
            return Status::out_of_bounds;
        }
        out_patches[count].start = start;
        out_patches[count].stop = prev + 1;
        ++count;
    }
    out_patch_count = count;
    err = ErrCode::none;
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
