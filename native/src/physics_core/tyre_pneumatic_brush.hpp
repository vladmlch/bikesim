// native/src/physics_core/tyre_pneumatic_brush.hpp — Tri-target (Host C++20/23,
// CUDA, Apple Metal MSL) pneumatic longitudinal brush kernels: Stage A C1 surface
// blending, load sensitivity, lumped parabolic brush, and discretised bristle brush
// (both homogeneous fast-path and ray-wise heterogeneous).
#pragma once

#include "common.hpp"
#include "tyre_contact.hpp"
#include "tyre_pneumatic_geometry.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel header: MSL has no std::span/std::array and device buffers are address-space-qualified raw pointers. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

template <class Real>
struct PneumaticSurfaceSample {
    Real mu_peak = static_cast<Real>(0);
    Real mu_slide = static_cast<Real>(0);
    Real slip_stiffness_per_load = static_cast<Real>(0);
    Real stribeck_speed_mps = BIKE_REAL(Real, 4.5);
    Real crr_delta = static_cast<Real>(0);
    Real load_sensitivity = static_cast<Real>(0);
};

template <class Real>
struct PneumaticSurfaceSection {
    Real start_m = static_cast<Real>(0);
    Real end_m = static_cast<Real>(0);
    PneumaticSurfaceSample<Real> surface{};
    Real blend_m = static_cast<Real>(0);
    i32 surface_id = 0;
};

template <class Real>
struct PneumaticSurfaceMap {
    PneumaticSurfaceSample<Real> default_surface{};
    PneumaticSurfaceSection<Real> sections[kMaxSurfaceSections]{};
    i32 section_count = 0;
    u8 has_surface_map = 0;
    u8 pad[3]{};
    i32 default_surface_id = 0;
};

template <class Real>
struct PneumaticBrushResult {
    Real force_n = static_cast<Real>(0);
    Real friction_coefficient = static_cast<Real>(0);
    u8 fully_sliding = 0;
    u8 pad[3]{};
};

namespace pneumatic_brush_detail {

template <class Real>
BIKE_HD bool same_surface_sample(
    PneumaticSurfaceSample<Real> a,
    i32 a_id,
    PneumaticSurfaceSample<Real> b,
    i32 b_id
) noexcept {
    return a_id == b_id &&
           a.mu_peak == b.mu_peak &&
           a.mu_slide == b.mu_slide &&
           a.slip_stiffness_per_load == b.slip_stiffness_per_load &&
           a.stribeck_speed_mps == b.stribeck_speed_mps &&
           a.crr_delta == b.crr_delta &&
           a.load_sensitivity == b.load_sensitivity;
}

template <class Real, class TK>
BIKE_HD PneumaticSurfaceSample<Real> blend_surface_samples(
    PneumaticSurfaceSample<Real> left,
    i32 left_id,
    PneumaticSurfaceSample<Real> right,
    i32 right_id,
    Real x_b,
    Real width_m,
    Real x_m,
    BIKE_REF(i32) out_surface_id
) noexcept {
    out_surface_id = x_m < x_b ? left_id : right_id;
    if (same_surface_sample(left, left_id, right, right_id)) {
        return x_m < x_b ? left : right;
    }
    const Real t = TK::clamp(
        BIKE_REAL(Real, 0.5) + TK::div(x_m - x_b, width_m),
        static_cast<Real>(0),
        static_cast<Real>(1));
    const Real s = t * t * (BIKE_REAL(Real, 3.0) - BIKE_REAL(Real, 2.0) * t);
    const Real w_l = static_cast<Real>(1) - s;
    PneumaticSurfaceSample<Real> out{};
    out.mu_peak = w_l * left.mu_peak + s * right.mu_peak;
    out.mu_slide = w_l * left.mu_slide + s * right.mu_slide;
    out.slip_stiffness_per_load =
        w_l * left.slip_stiffness_per_load + s * right.slip_stiffness_per_load;
    out.stribeck_speed_mps =
        w_l * left.stribeck_speed_mps + s * right.stribeck_speed_mps;
    out.crr_delta = w_l * left.crr_delta + s * right.crr_delta;
    out.load_sensitivity =
        w_l * left.load_sensitivity + s * right.load_sensitivity;
    return out;
}

} // namespace pneumatic_brush_detail

template <class Real, class TK, class MapPtr>
BIKE_HD Status eval_surface_sample(
    MapPtr map,
    Real x_m,
    BIKE_REF(PneumaticSurfaceSample<Real>) out_sample,
    BIKE_REF(i32) out_surface_id,
    BIKE_REF(ErrCode) err
) noexcept {
    if (!TK::isfinite(x_m)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (map == nullptr) {
        out_sample = PneumaticSurfaceSample<Real>{};
        out_surface_id = 0;
        err = ErrCode::out_of_bounds_station;
        return Status::invalid_argument;
    }
    if (map->has_surface_map == 0 || map->section_count <= 0) {
        out_sample = map->default_surface;
        out_surface_id = map->default_surface_id;
        err = ErrCode::none;
        return Status::ok;
    }
    const i32 n = map->section_count < kMaxSurfaceSections
        ? map->section_count
        : kMaxSurfaceSections;
    if (n > 0) {
        if (map->sections[0].start_m > static_cast<Real>(0) &&
            map->sections[0].blend_m > static_cast<Real>(0)) {
            const Real x_b = map->sections[0].start_m;
            const Real w = map->sections[0].blend_m;
            const Real half_w = BIKE_REAL(Real, 0.5) * w;
            if (x_b - half_w < x_m && x_m < x_b + half_w) {
                out_sample = pneumatic_brush_detail::blend_surface_samples<Real, TK>(
                    map->default_surface, map->default_surface_id,
                    map->sections[0].surface, map->sections[0].surface_id,
                    x_b, w, x_m, out_surface_id);
                err = ErrCode::none;
                return Status::ok;
            }
        }
        for (i32 i = 0; i < n; ++i) {
            if (i + 1 < n && map->sections[i].end_m == map->sections[i + 1].start_m) {
                if (map->sections[i].blend_m > static_cast<Real>(0)) {
                    const Real x_b = map->sections[i].end_m;
                    const Real w = map->sections[i].blend_m;
                    const Real half_w = BIKE_REAL(Real, 0.5) * w;
                    if (x_b - half_w < x_m && x_m < x_b + half_w) {
                        out_sample = pneumatic_brush_detail::blend_surface_samples<Real, TK>(
                            map->sections[i].surface, map->sections[i].surface_id,
                            map->sections[i + 1].surface, map->sections[i + 1].surface_id,
                            x_b, w, x_m, out_surface_id);
                        err = ErrCode::none;
                        return Status::ok;
                    }
                }
            } else {
                if (map->sections[i].blend_m > static_cast<Real>(0)) {
                    const Real x_b = map->sections[i].end_m;
                    const Real w = map->sections[i].blend_m;
                    const Real half_w = BIKE_REAL(Real, 0.5) * w;
                    if (x_b - half_w < x_m && x_m < x_b + half_w) {
                        out_sample = pneumatic_brush_detail::blend_surface_samples<Real, TK>(
                            map->sections[i].surface, map->sections[i].surface_id,
                            map->default_surface, map->default_surface_id,
                            x_b, w, x_m, out_surface_id);
                        err = ErrCode::none;
                        return Status::ok;
                    }
                }
                if (i + 1 < n && map->sections[i + 1].blend_m > static_cast<Real>(0)) {
                    const Real x_b = map->sections[i + 1].start_m;
                    const Real w = map->sections[i + 1].blend_m;
                    const Real half_w = BIKE_REAL(Real, 0.5) * w;
                    if (x_b - half_w < x_m && x_m < x_b + half_w) {
                        out_sample = pneumatic_brush_detail::blend_surface_samples<Real, TK>(
                            map->default_surface, map->default_surface_id,
                            map->sections[i + 1].surface, map->sections[i + 1].surface_id,
                            x_b, w, x_m, out_surface_id);
                        err = ErrCode::none;
                        return Status::ok;
                    }
                }
            }
        }
    }

    if (n > 0 && map->sections[0].start_m == static_cast<Real>(0) &&
        x_m < static_cast<Real>(0)) {
        out_sample = map->sections[0].surface;
        out_surface_id = map->sections[0].surface_id;
        err = ErrCode::none;
        return Status::ok;
    }
    for (i32 i = 0; i < n; ++i) {
        if (map->sections[i].start_m <= x_m && x_m < map->sections[i].end_m) {
            out_sample = map->sections[i].surface;
            out_surface_id = map->sections[i].surface_id;
            err = ErrCode::none;
            return Status::ok;
        }
    }
    out_sample = map->default_surface;
    out_surface_id = map->default_surface_id;
    err = ErrCode::none;
    return Status::ok;
}

template <class Real, class TK, class MapPtr>
BIKE_HD Status pneumatic_uniform_surface_for_span(
    MapPtr map,
    Real x_min_m,
    Real x_max_m,
    BIKE_REF(i32) out_section_idx,
    BIKE_REF(bool) out_is_uniform,
    BIKE_REF(ErrCode) err
) noexcept {
    out_section_idx = -1;
    out_is_uniform = false;
    if (!TK::isfinite(x_min_m) || !TK::isfinite(x_max_m) || x_min_m > x_max_m) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (map == nullptr) {
        err = ErrCode::out_of_bounds_station;
        return Status::invalid_argument;
    }
    if (map->has_surface_map == 0 || map->section_count <= 0) {
        out_section_idx = -1;
        out_is_uniform = true;
        err = ErrCode::none;
        return Status::ok;
    }
    const i32 n = map->section_count < kMaxSurfaceSections
        ? map->section_count
        : kMaxSurfaceSections;

    if (map->sections[0].start_m > static_cast<Real>(0) &&
        map->sections[0].blend_m > static_cast<Real>(0)) {
        const Real x_b = map->sections[0].start_m;
        const Real half_w = BIKE_REAL(Real, 0.5) * map->sections[0].blend_m;
        if (x_min_m <= x_b + half_w && x_max_m >= x_b - half_w) {
            err = ErrCode::none;
            return Status::ok;
        }
    }
    for (i32 i = 0; i < n; ++i) {
        if (i + 1 < n && map->sections[i].end_m == map->sections[i + 1].start_m) {
            if (map->sections[i].blend_m > static_cast<Real>(0)) {
                const Real x_b = map->sections[i].end_m;
                const Real half_w = BIKE_REAL(Real, 0.5) * map->sections[i].blend_m;
                if (x_min_m <= x_b + half_w && x_max_m >= x_b - half_w) {
                    err = ErrCode::none;
                    return Status::ok;
                }
            }
        } else {
            if (map->sections[i].blend_m > static_cast<Real>(0)) {
                const Real x_b = map->sections[i].end_m;
                const Real half_w = BIKE_REAL(Real, 0.5) * map->sections[i].blend_m;
                if (x_min_m <= x_b + half_w && x_max_m >= x_b - half_w) {
                    err = ErrCode::none;
                    return Status::ok;
                }
            }
            if (i + 1 < n && map->sections[i + 1].blend_m > static_cast<Real>(0)) {
                const Real x_b = map->sections[i + 1].start_m;
                const Real half_w =
                    BIKE_REAL(Real, 0.5) * map->sections[i + 1].blend_m;
                if (x_min_m <= x_b + half_w && x_max_m >= x_b - half_w) {
                    err = ErrCode::none;
                    return Status::ok;
                }
            }
        }
    }

    if (map->sections[0].start_m == static_cast<Real>(0)) {
        if (x_max_m < map->sections[0].end_m) {
            out_section_idx = 0;
            out_is_uniform = true;
            err = ErrCode::none;
            return Status::ok;
        }
    } else {
        if (x_max_m < map->sections[0].start_m) {
            out_section_idx = -1;
            out_is_uniform = true;
            err = ErrCode::none;
            return Status::ok;
        }
        if (map->sections[0].start_m <= x_min_m &&
            x_max_m < map->sections[0].end_m) {
            out_section_idx = 0;
            out_is_uniform = true;
            err = ErrCode::none;
            return Status::ok;
        }
    }
    for (i32 i = 0; i < n; ++i) {
        if (i >= 1 && map->sections[i].start_m <= x_min_m &&
            x_max_m < map->sections[i].end_m) {
            out_section_idx = i;
            out_is_uniform = true;
            err = ErrCode::none;
            return Status::ok;
        }
        if (i + 1 < n && map->sections[i].end_m < map->sections[i + 1].start_m &&
            map->sections[i].end_m <= x_min_m &&
            x_max_m < map->sections[i + 1].start_m) {
            out_section_idx = -1;
            out_is_uniform = true;
            err = ErrCode::none;
            return Status::ok;
        }
    }
    if (map->sections[n - 1].end_m <= x_min_m) {
        out_section_idx = -1;
        out_is_uniform = true;
        err = ErrCode::none;
        return Status::ok;
    }
    err = ErrCode::none;
    return Status::ok;
}

template <class Real, class TK>
BIKE_HD Status pneumatic_surface_mu(
    PneumaticSurfaceSample<Real> surface,
    Real sliding_speed_mps,
    BIKE_REF(Real) out_mu,
    BIKE_REF(ErrCode) err
) noexcept {
    if (!TK::isfinite(sliding_speed_mps)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Real decay = TK::exp(
        -TK::div(TK::abs(sliding_speed_mps), surface.stribeck_speed_mps));
    const Real value =
        surface.mu_slide + (surface.mu_peak - surface.mu_slide) * decay;
    if (!TK::isfinite(value)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    out_mu = value;
    err = ErrCode::none;
    return Status::ok;
}

template <class Real, class TK>
BIKE_HD Status pneumatic_sample_mu(
    PneumaticSurfaceSample<Real> sample,
    Real sliding_speed_mps,
    Real wheel_normal_load_n,
    BIKE_REF(Real) out_mu,
    BIKE_REF(ErrCode) err
) noexcept {
    if (!TK::isfinite(sliding_speed_mps)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(wheel_normal_load_n) ||
        wheel_normal_load_n < static_cast<Real>(0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Real decay = TK::exp(
        -TK::div(TK::abs(sliding_speed_mps), sample.stribeck_speed_mps));
    const Real mu_str =
        sample.mu_slide + (sample.mu_peak - sample.mu_slide) * decay;
    const Real value =
        (wheel_normal_load_n <= BIKE_REAL(Real, 500.0) ||
         sample.load_sensitivity <= static_cast<Real>(0))
            ? mu_str
            : (load_sensitivity_factor<Real, TK>(
                   wheel_normal_load_n, sample.load_sensitivity) *
               mu_str);
    if (!TK::isfinite(value)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    out_mu = value;
    err = ErrCode::none;
    return Status::ok;
}

template <class Real, class TK>
BIKE_HD Status pneumatic_relax(
    Real kappa_prime,
    Real v_x_mps,
    Real v_s_mps,
    Real sigma_m,
    Real dt_s,
    BIKE_REF(Real) out_kappa,
    BIKE_REF(ErrCode) err
) noexcept {
    if (!TK::isfinite(kappa_prime) || !TK::isfinite(v_x_mps) ||
        !TK::isfinite(v_s_mps) || !TK::isfinite(sigma_m) ||
        !TK::isfinite(dt_s)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (sigma_m < static_cast<Real>(0) || dt_s < static_cast<Real>(0)) {
        err = ErrCode::bad_dt;
        return Status::invalid_argument;
    }
    const Real speed_mps = TK::abs(v_x_mps);
    if (sigma_m == static_cast<Real>(0)) {
        out_kappa = speed_mps > static_cast<Real>(0)
            ? TK::div(-v_s_mps, speed_mps)
            : kappa_prime;
        err = ErrCode::none;
        return Status::ok;
    }
    if (speed_mps == static_cast<Real>(0)) {
        out_kappa = kappa_prime - TK::div(v_s_mps * dt_s, sigma_m);
        err = ErrCode::none;
        return Status::ok;
    }
    const Real steady_kappa = TK::div(-v_s_mps, speed_mps);
    const Real exponent = TK::div(-speed_mps * dt_s, sigma_m);
    out_kappa =
        steady_kappa + (kappa_prime - steady_kappa) * TK::exp(exponent);
    err = ErrCode::none;
    return Status::ok;
}

template <class Real, class TK>
BIKE_HD Real pneumatic_sigma_x(Real kappa_prime) noexcept {
    if (kappa_prime <= static_cast<Real>(-1)) {
        return -pneumatic_detail::pos_inf<Real>();
    }
    if (pneumatic_detail::isinf_val(kappa_prime)) {
        return pneumatic_detail::copysign_val(
            static_cast<Real>(1), kappa_prime);
    }
    return TK::div(kappa_prime, static_cast<Real>(1) + kappa_prime);
}

template <class Real, class TK>
BIKE_HD Status pneumatic_tread_stiffness_n_m2(
    Real normal_load_n,
    Real half_length_m,
    PneumaticSurfaceSample<Real> surface,
    bool has_explicit,
    Real explicit_stiffness_n_m2,
    BIKE_REF(Real) out_c_px,
    BIKE_REF(ErrCode) err
) noexcept {
    if (has_explicit) {
        if (!TK::isfinite(explicit_stiffness_n_m2) ||
            explicit_stiffness_n_m2 < static_cast<Real>(0)) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        out_c_px = explicit_stiffness_n_m2;
        err = ErrCode::none;
        return Status::ok;
    }
    const Real a2 = TK::pow2(half_length_m);
    if (pneumatic_detail::isinf_val(a2)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    out_c_px = TK::div(
        surface.slip_stiffness_per_load * normal_load_n,
        BIKE_REAL(Real, 2.0) * a2);
    err = ErrCode::none;
    return Status::ok;
}

template <class Real, class TK>
BIKE_HD Status pneumatic_lumped_brush(
    Real kappa_prime,
    Real normal_load_n,
    Real half_length_m,
    PneumaticSurfaceSample<Real> surface,
    bool has_explicit,
    Real explicit_stiffness_n_m2,
    Real sliding_speed_mps,
    BIKE_REF(PneumaticBrushResult<Real>) out,
    BIKE_REF(ErrCode) err
) noexcept {
    out = PneumaticBrushResult<Real>{};
    if (!TK::isfinite(kappa_prime) || !TK::isfinite(normal_load_n) ||
        !TK::isfinite(half_length_m) || !TK::isfinite(sliding_speed_mps)) {
        if (!pneumatic_detail::isinf_val(kappa_prime)) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        if (!TK::isfinite(normal_load_n) || !TK::isfinite(half_length_m) ||
            !TK::isfinite(sliding_speed_mps)) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
    }
    if (normal_load_n < static_cast<Real>(0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (normal_load_n == static_cast<Real>(0)) {
        Real mu = static_cast<Real>(0);
        const Status st = pneumatic_surface_mu<Real, TK>(
            surface, sliding_speed_mps, mu, err);
        if (st != Status::ok) {
            return st;
        }
        out.force_n = static_cast<Real>(0);
        out.fully_sliding = 0;
        out.friction_coefficient = mu;
        return Status::ok;
    }
    if (half_length_m <= static_cast<Real>(0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }

    Real mu = static_cast<Real>(0);
    Status st = pneumatic_surface_mu<Real, TK>(
        surface, sliding_speed_mps, mu, err);
    if (st != Status::ok) {
        return st;
    }
    Real c_px = static_cast<Real>(0);
    st = pneumatic_tread_stiffness_n_m2<Real, TK>(
        normal_load_n, half_length_m, surface, has_explicit,
        explicit_stiffness_n_m2, c_px, err);
    if (st != Status::ok) {
        return st;
    }
    if (c_px == static_cast<Real>(0)) {
        out.force_n = static_cast<Real>(0);
        out.fully_sliding = 0;
        out.friction_coefficient = mu;
        err = ErrCode::none;
        return Status::ok;
    }

    const Real a2 = TK::pow2(half_length_m);
    if (pneumatic_detail::isinf_val(a2)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    const Real theta = TK::div(
        BIKE_REAL(Real, 2.0) * c_px * a2,
        BIKE_REAL(Real, 3.0) * mu * normal_load_n);
    const Real z = theta * pneumatic_sigma_x<Real, TK>(kappa_prime);
    if (TK::abs(z) >= static_cast<Real>(1)) {
        out.force_n =
            mu * normal_load_n *
            pneumatic_detail::copysign_val(static_cast<Real>(1), z);
        out.fully_sliding = 1;
        out.friction_coefficient = mu;
        err = ErrCode::none;
        return Status::ok;
    }
    out.force_n =
        BIKE_REAL(Real, 3.0) * mu * normal_load_n * z *
        (static_cast<Real>(1) - TK::abs(z) +
         TK::div(z * z, BIKE_REAL(Real, 3.0)));
    out.fully_sliding = 0;
    out.friction_coefficient = mu;
    err = ErrCode::none;
    return Status::ok;
}

template <
    class Real,
    class TK,
    class PtrLoads,
    class PtrBristle,
    class PtrScratch>
BIKE_HD Status discretised_brush_step(
    i32 n,
    PtrLoads normal_loads_n,
    Real half_length_m,
    Real v_x_mps,
    Real v_s_mps,
    Real sigma_m,
    PneumaticSurfaceSample<Real> surface,
    Real dt_s,
    BIKE_REF(Real) kappa_prime,
    PtrBristle bristle_deflection_m,
    PtrScratch updated_buf,
    BIKE_REF(PneumaticBrushResult<Real>) out,
    BIKE_REF(ErrCode) err
) noexcept {
    out = PneumaticBrushResult<Real>{};
    if (n < 1 || n > kMaxPneumaticRays || normal_loads_n == nullptr ||
        bristle_deflection_m == nullptr || updated_buf == nullptr) {
        err = ErrCode::out_of_bounds_station;
        return Status::invalid_argument;
    }
    for (i32 i = 0; i < n; ++i) {
        const Real load = normal_loads_n[i];
        if (!TK::isfinite(load) || load < static_cast<Real>(0)) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
    }
    if (!TK::isfinite(half_length_m) || !TK::isfinite(v_x_mps) ||
        !TK::isfinite(v_s_mps) || !TK::isfinite(sigma_m) ||
        !TK::isfinite(dt_s)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (half_length_m < static_cast<Real>(0) ||
        sigma_m < static_cast<Real>(0) || dt_s <= static_cast<Real>(0)) {
        err = ErrCode::bad_dt;
        return Status::invalid_argument;
    }

    const Real total_load_n =
        pneumatic_detail::array_sum<Real, TK>(n, normal_loads_n);
    Real mu = static_cast<Real>(0);
    Status st = pneumatic_surface_mu<Real, TK>(
        surface, TK::abs(v_s_mps), mu, err);
    if (st != Status::ok) {
        return st;
    }
    if (total_load_n <= static_cast<Real>(0) ||
        half_length_m <= static_cast<Real>(0)) {
        for (i32 i = 0; i < n; ++i) {
            bristle_deflection_m[i] = static_cast<Real>(0);
        }
        kappa_prime = static_cast<Real>(0);
        out.force_n = static_cast<Real>(0);
        out.fully_sliding = 0;
        out.friction_coefficient = mu;
        err = ErrCode::none;
        return Status::ok;
    }

    const Real input_sigma_m =
        TK::max(static_cast<Real>(0), sigma_m - half_length_m);
    Real next_kappa = kappa_prime;
    st = pneumatic_relax<Real, TK>(
        kappa_prime, v_x_mps, v_s_mps, input_sigma_m, dt_s, next_kappa, err);
    if (st != Status::ok) {
        return st;
    }
    kappa_prime = next_kappa;

    const Real dx_m = TK::div(
        BIKE_REAL(Real, 2.0) * half_length_m, static_cast<Real>(n));
    const Real filtered_v_s_mps = v_x_mps != static_cast<Real>(0)
        ? -kappa_prime * TK::abs(v_x_mps)
        : v_s_mps;
    const Real wheel_surface_speed_mps = v_x_mps - v_s_mps;

    if (wheel_surface_speed_mps != static_cast<Real>(0)) {
        const Real shift = TK::div(wheel_surface_speed_mps * dt_s, dx_m);
        for (i32 i = 0; i < n; ++i) {
            updated_buf[i] = pneumatic_detail::np_interp_unit_grid<Real, TK>(
                static_cast<Real>(i) + shift,
                bristle_deflection_m,
                n,
                static_cast<Real>(0),
                static_cast<Real>(0));
        }
    } else {
        for (i32 i = 0; i < n; ++i) {
            updated_buf[i] = bristle_deflection_m[i];
        }
    }

    const Real growth_m = filtered_v_s_mps * dt_s;
    for (i32 i = 0; i < n; ++i) {
        const Real upd = updated_buf[i] - growth_m;
        updated_buf[i] =
            normal_loads_n[i] > static_cast<Real>(0) ? upd : static_cast<Real>(0);
    }

    const Real a2 = TK::pow2(half_length_m);
    if (pneumatic_detail::isinf_val(a2)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    const Real c_px = TK::div(
        surface.slip_stiffness_per_load * total_load_n,
        BIKE_REAL(Real, 2.0) * a2);
    const Real element_stiffness_n_m = c_px * dx_m;
    const Real damping_active_n =
        element_stiffness_n_m * BIKE_REAL(Real, 0.0005) * (-filtered_v_s_mps);

    bool any_active = false;
    bool all_active_sliding = true;
    Real force_sum = static_cast<Real>(0);
    for (i32 i = 0; i < n; ++i) {
        const bool active = normal_loads_n[i] > static_cast<Real>(0);
        const Real damping = active ? damping_active_n : static_cast<Real>(0);
        const Real trial = element_stiffness_n_m * updated_buf[i] + damping;
        const Real limit = mu * normal_loads_n[i];
        const Real force = TK::clamp(trial, -limit, limit);
        force_sum += force;
        const bool sliding = active && TK::abs(trial) >= limit;
        any_active = any_active || active;
        if (active) {
            all_active_sliding = all_active_sliding && sliding;
        }
        if (element_stiffness_n_m > static_cast<Real>(0) && sliding) {
            updated_buf[i] =
                TK::div(force - damping, element_stiffness_n_m);
        }
    }

    for (i32 i = 0; i < n; ++i) {
        bristle_deflection_m[i] = updated_buf[i];
    }
    out.force_n = force_sum;
    out.fully_sliding = (any_active && all_active_sliding) ? 1 : 0;
    out.friction_coefficient = mu;
    err = ErrCode::none;
    return Status::ok;
}

template <
    class Real,
    class TK,
    class PtrLoads,
    class PtrSamples,
    class PtrBristle,
    class PtrScratch>
BIKE_HD Status discretised_brush_step_heterogeneous(
    i32 n,
    PtrLoads loads,
    Real half_length_m,
    Real v_x_mps,
    Real v_s_mps,
    Real sigma_m,
    PtrSamples samples,
    Real patch_normal_load_n,
    Real wheel_normal_load_n,
    Real dt_s,
    BIKE_REF(Real) kappa_prime,
    PtrBristle bristle_deflection_m,
    PtrScratch updated_buf,
    PtrScratch stiffness_buf,
    PtrScratch mu_buf,
    BIKE_REF(PneumaticBrushResult<Real>) out,
    BIKE_REF(ErrCode) err
) noexcept {
    out = PneumaticBrushResult<Real>{};
    if (n < 1 || n > kMaxPneumaticRays || loads == nullptr ||
        samples == nullptr || bristle_deflection_m == nullptr ||
        updated_buf == nullptr || stiffness_buf == nullptr ||
        mu_buf == nullptr) {
        err = ErrCode::out_of_bounds_station;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(half_length_m) || !TK::isfinite(v_x_mps) ||
        !TK::isfinite(v_s_mps) || !TK::isfinite(sigma_m) ||
        !TK::isfinite(patch_normal_load_n) ||
        !TK::isfinite(wheel_normal_load_n) || !TK::isfinite(dt_s) ||
        half_length_m < static_cast<Real>(0) ||
        sigma_m < static_cast<Real>(0) ||
        patch_normal_load_n < static_cast<Real>(0) ||
        wheel_normal_load_n < static_cast<Real>(0) ||
        dt_s <= static_cast<Real>(0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    for (i32 i = 0; i < n; ++i) {
        const Real load = loads[i];
        if (!TK::isfinite(load) || load < static_cast<Real>(0)) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
    }

    const Real total = pneumatic_detail::array_sum<Real, TK>(n, loads);
    if (!TK::isfinite(total)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }

    const Real ref_load = BIKE_REAL(Real, 500.0);
    const Real abs_vs = TK::abs(v_s_mps);
    if (total <= static_cast<Real>(0) || half_length_m <= static_cast<Real>(0)) {
        PneumaticSurfaceSample<Real> s0 = samples[0];
        if (wheel_normal_load_n > ref_load &&
            s0.load_sensitivity > static_cast<Real>(0)) {
            const Real factor = load_sensitivity_factor<Real, TK>(
                wheel_normal_load_n, s0.load_sensitivity);
            s0.mu_peak *= factor;
            s0.mu_slide *= factor;
            s0.load_sensitivity = static_cast<Real>(0);
        }
        Real mu0 = static_cast<Real>(0);
        const Status st = pneumatic_sample_mu<Real, TK>(
            s0, abs_vs, wheel_normal_load_n, mu0, err);
        if (st != Status::ok) {
            return st;
        }
        for (i32 i = 0; i < n; ++i) {
            bristle_deflection_m[i] = static_cast<Real>(0);
        }
        kappa_prime = static_cast<Real>(0);
        out.force_n = static_cast<Real>(0);
        out.fully_sliding = 0;
        out.friction_coefficient = mu0;
        err = ErrCode::none;
        return Status::ok;
    }

    const Real a2 = TK::pow2(half_length_m);
    if (pneumatic_detail::isinf_val(a2)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    const Real dx = TK::div(
        BIKE_REAL(Real, 2.0) * half_length_m, static_cast<Real>(n));
    Real weighted_mu = static_cast<Real>(0);
    bool identical_mu = true;
    for (i32 i = 0; i < n; ++i) {
        PneumaticSurfaceSample<Real> s = samples[i];
        if (wheel_normal_load_n > ref_load &&
            s.load_sensitivity > static_cast<Real>(0)) {
            const Real factor = load_sensitivity_factor<Real, TK>(
                wheel_normal_load_n, s.load_sensitivity);
            s.mu_peak *= factor;
            s.mu_slide *= factor;
            s.load_sensitivity = static_cast<Real>(0);
        }
        Real mu_i = static_cast<Real>(0);
        const Status mu_st = pneumatic_sample_mu<Real, TK>(
            s, abs_vs, wheel_normal_load_n, mu_i, err);
        if (mu_st != Status::ok) {
            return mu_st;
        }
        mu_buf[i] = mu_i;
        stiffness_buf[i] =
            TK::div(s.slip_stiffness_per_load * total,
                    BIKE_REAL(Real, 2.0) * a2) *
            dx;
        weighted_mu += mu_buf[i] * loads[i];
        if (i > 0 && mu_buf[i] != mu_buf[0]) {
            identical_mu = false;
        }
        if (!TK::isfinite(mu_buf[i]) || !TK::isfinite(stiffness_buf[i]) ||
            !TK::isfinite(weighted_mu)) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }
    }

    Real next_kappa = kappa_prime;
    const Status r_st = pneumatic_relax<Real, TK>(
        kappa_prime, v_x_mps, v_s_mps,
        TK::max(static_cast<Real>(0), sigma_m - half_length_m),
        dt_s, next_kappa, err);
    if (r_st != Status::ok) {
        return r_st;
    }
    const Real filtered = v_x_mps != static_cast<Real>(0)
        ? -next_kappa * TK::abs(v_x_mps)
        : v_s_mps;
    const Real speed = v_x_mps - v_s_mps;
    const Real shift = TK::div(speed * dt_s, dx);
    if (!TK::isfinite(dx) || !TK::isfinite(next_kappa) ||
        !TK::isfinite(filtered) || !TK::isfinite(speed) ||
        !TK::isfinite(shift)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }

    bool any_active = false;
    bool all_sliding = true;
    Real force = static_cast<Real>(0);
    for (i32 i = 0; i < n; ++i) {
        const bool active = loads[i] > static_cast<Real>(0);
        const Real advected = speed != static_cast<Real>(0)
            ? pneumatic_detail::np_interp_unit_grid<Real, TK>(
                  static_cast<Real>(i) + shift,
                  bristle_deflection_m,
                  n,
                  static_cast<Real>(0),
                  static_cast<Real>(0))
            : bristle_deflection_m[i];
        const Real updated = active ? (advected - filtered * dt_s) : static_cast<Real>(0);
        const Real damping = active
            ? stiffness_buf[i] * BIKE_REAL(Real, 0.0005) * (-filtered)
            : static_cast<Real>(0);
        const Real trial = stiffness_buf[i] * updated + damping;
        const Real limit = mu_buf[i] * loads[i];
        if (!TK::isfinite(advected) || !TK::isfinite(updated) ||
            !TK::isfinite(damping) || !TK::isfinite(trial) ||
            !TK::isfinite(limit)) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }
        const Real clamped_force = TK::clamp(trial, -limit, limit);
        const bool sliding = active && TK::abs(trial) >= limit;
        updated_buf[i] =
            (stiffness_buf[i] > static_cast<Real>(0) && sliding)
                ? TK::div(clamped_force - damping, stiffness_buf[i])
                : updated;
        if (!TK::isfinite(clamped_force) || !TK::isfinite(updated_buf[i])) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }
        force += clamped_force;
        any_active = any_active || active;
        if (active) {
            all_sliding = all_sliding && sliding;
        }
    }

    const Real mean_mu = identical_mu ? mu_buf[0] : TK::div(weighted_mu, total);
    if (!TK::isfinite(force) || !TK::isfinite(mean_mu)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    kappa_prime = next_kappa;
    for (i32 i = 0; i < n; ++i) {
        bristle_deflection_m[i] = updated_buf[i];
    }
    out.force_n = force;
    out.fully_sliding = (any_active && all_sliding) ? 1 : 0;
    out.friction_coefficient = mean_mu;
    err = ErrCode::none;
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
