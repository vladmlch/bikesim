// native/src/physics_core/tyre_contact.hpp — Tri-target shared tyre contact
// primitives (Stribeck surface friction, rate-independent radial hysteresis,
// bristle transport, and brush shear step) preserving 0-bit CPU FP64 parity.
#pragma once

#include "common.hpp"
#include "tyre_types.hpp"

#if !defined(__METAL_VERSION__)
#include <cmath>
#endif

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel primitives: positional parameters and templated pointer arguments match the CUDA/MSL ABI without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

template <class Real>
struct TyreVec3 {
    Real x = BIKE_REAL(Real, 0.0);
    Real y = BIKE_REAL(Real, 0.0);
    Real z = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct TyreXZ {
    Real x = BIKE_REAL(Real, 0.0);
    Real z = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct FrictionQuery {
    Real tire_mu = BIKE_REAL(Real, 0.0);
    Real contact_x = BIKE_REAL(Real, 0.0);
    Real slip = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct FrictionResult {
    Real mu = BIKE_REAL(Real, 0.0);
    i32 section_idx = -1;
};

template <class Real>
struct SurfaceSamplePod {
    Real mu_peak = BIKE_REAL(Real, 0.0);
    Real mu_slide = BIKE_REAL(Real, 0.0);
    Real slip_stiffness_per_load = BIKE_REAL(Real, 0.0);
    Real stribeck_speed_mps = BIKE_REAL(Real, 4.5);
    Real crr_delta = BIKE_REAL(Real, 0.0);
    Real load_sensitivity = BIKE_REAL(Real, 0.0);
    i32 surface_id = 0;
    i32 section_idx = -1;
};

template <class Real>
BIKE_HD SurfaceSamplePod<Real> sample_from_spec_pod(
    SurfaceSpecPod<Real> s,
    i32 section_idx
) noexcept {
    SurfaceSamplePod<Real> out{};
    out.mu_peak = s.mu_peak;
    out.mu_slide = s.mu_slide;
    out.slip_stiffness_per_load = s.slip_stiffness_per_load;
    out.stribeck_speed_mps = s.stribeck_speed_mps;
    out.crr_delta = s.crr_delta;
    out.load_sensitivity = s.load_sensitivity;
    out.surface_id = s.surface_id;
    out.section_idx = section_idx;
    return out;
}

template <class Real>
BIKE_HD bool same_surface_spec_pod(
    SurfaceSpecPod<Real> a,
    SurfaceSpecPod<Real> b
) noexcept {
    return a.surface_id == b.surface_id &&
           a.mu_peak == b.mu_peak &&
           a.mu_slide == b.mu_slide &&
           a.slip_stiffness_per_load == b.slip_stiffness_per_load &&
           a.stribeck_speed_mps == b.stribeck_speed_mps &&
           a.crr_delta == b.crr_delta &&
           a.load_sensitivity == b.load_sensitivity;
}

template <class Real, class TK>
BIKE_HD SurfaceSamplePod<Real> blend_surface_specs(
    SurfaceSpecPod<Real> left,
    i32 left_section_idx,
    SurfaceSpecPod<Real> right,
    i32 right_section_idx,
    Real x_b,
    Real width_m,
    Real x_m
) noexcept {
    const bool use_left = (x_m < x_b);
    const SurfaceSpecPod<Real> canonical = use_left ? left : right;
    const i32 canonical_idx = use_left ? left_section_idx : right_section_idx;
    if (same_surface_spec_pod(left, right)) {
        return sample_from_spec_pod(canonical, canonical_idx);
    }
    const Real t = TK::clamp(
        BIKE_REAL(Real, 0.5) + TK::div(x_m - x_b, width_m),
        BIKE_REAL(Real, 0.0),
        BIKE_REAL(Real, 1.0));
    const Real s =
        t * t * (BIKE_REAL(Real, 3.0) - BIKE_REAL(Real, 2.0) * t);
    const Real w_l = BIKE_REAL(Real, 1.0) - s;
    SurfaceSamplePod<Real> out{};
    out.mu_peak = w_l * left.mu_peak + s * right.mu_peak;
    out.mu_slide = w_l * left.mu_slide + s * right.mu_slide;
    out.slip_stiffness_per_load =
        w_l * left.slip_stiffness_per_load + s * right.slip_stiffness_per_load;
    out.stribeck_speed_mps =
        w_l * left.stribeck_speed_mps + s * right.stribeck_speed_mps;
    out.crr_delta = w_l * left.crr_delta + s * right.crr_delta;
    out.load_sensitivity =
        w_l * left.load_sensitivity + s * right.load_sensitivity;
    out.surface_id = canonical.surface_id;
    out.section_idx = canonical_idx;
    return out;
}

template <class Real, class TK, class MapPtr>
BIKE_HD Status eval_surface(
    MapPtr map,
    Real x_m,
    BIKE_REF(SurfaceSamplePod<Real>) out_sample,
    BIKE_REF(ErrCode) err
) noexcept {
    err = ErrCode::none;
    if (map == nullptr) {
        out_sample = SurfaceSamplePod<Real>{};
        return Status::ok;
    }
    if (!TK::isfinite(x_m)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const i32 count = map->section_count < kMaxSurfaceSections
        ? map->section_count
        : kMaxSurfaceSections;
    if (map->has_surface_map == 0 || count <= 0) {
        out_sample = sample_from_spec_pod(map->default_surface, -1);
        return Status::ok;
    }
    const Real half_factor = BIKE_REAL(Real, 0.5);
    if (map->sections[0].start_m > BIKE_REAL(Real, 0.0) &&
        map->sections[0].blend_m > BIKE_REAL(Real, 0.0)) {
        const Real x_b = map->sections[0].start_m;
        const Real w = map->sections[0].blend_m;
        const Real half_w = half_factor * w;
        if (x_b - half_w < x_m && x_m < x_b + half_w) {
            out_sample = blend_surface_specs<Real, TK>(
                map->default_surface, -1, map->sections[0].surface, 0, x_b, w, x_m);
            return Status::ok;
        }
    }
    for (i32 i = 0; i < count; ++i) {
        const Real end_i = map->sections[i].end_m;
        const Real blend_i = map->sections[i].blend_m;
        if (i + 1 < count && end_i == map->sections[i + 1].start_m) {
            if (blend_i > BIKE_REAL(Real, 0.0)) {
                const Real half_w = half_factor * blend_i;
                if (end_i - half_w < x_m && x_m < end_i + half_w) {
                    out_sample = blend_surface_specs<Real, TK>(
                        map->sections[i].surface, i,
                        map->sections[i + 1].surface, i + 1,
                        end_i, blend_i, x_m);
                    return Status::ok;
                }
            }
        } else {
            if (blend_i > BIKE_REAL(Real, 0.0)) {
                const Real half_w = half_factor * blend_i;
                if (end_i - half_w < x_m && x_m < end_i + half_w) {
                    out_sample = blend_surface_specs<Real, TK>(
                        map->sections[i].surface, i,
                        map->default_surface, -1,
                        end_i, blend_i, x_m);
                    return Status::ok;
                }
            }
            if (i + 1 < count && map->sections[i + 1].blend_m > BIKE_REAL(Real, 0.0)) {
                const Real next_start = map->sections[i + 1].start_m;
                const Real next_blend = map->sections[i + 1].blend_m;
                const Real half_w = half_factor * next_blend;
                if (next_start - half_w < x_m && x_m < next_start + half_w) {
                    out_sample = blend_surface_specs<Real, TK>(
                        map->default_surface, -1,
                        map->sections[i + 1].surface, i + 1,
                        next_start, next_blend, x_m);
                    return Status::ok;
                }
            }
        }
    }

    if (map->sections[0].start_m == BIKE_REAL(Real, 0.0) &&
        x_m < BIKE_REAL(Real, 0.0)) {
        out_sample = sample_from_spec_pod(map->sections[0].surface, 0);
        return Status::ok;
    }
    for (i32 i = 0; i < count; ++i) {
        if (map->sections[i].start_m <= x_m && x_m < map->sections[i].end_m) {
            out_sample = sample_from_spec_pod(map->sections[i].surface, i);
            return Status::ok;
        }
    }
    out_sample = sample_from_spec_pod(map->default_surface, -1);
    return Status::ok;
}

template <class Real, class TK, class MapPtr>
BIKE_HD Status uniform_surface_for_span(
    MapPtr map,
    Real x_min_m,
    Real x_max_m,
    BIKE_REF(bool) out_is_uniform,
    BIKE_REF(i32) out_section_idx,
    BIKE_REF(ErrCode) err
) noexcept {
    err = ErrCode::none;
    out_is_uniform = false;
    out_section_idx = -1;
    if (!TK::isfinite(x_min_m) || !TK::isfinite(x_max_m) || x_min_m > x_max_m) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (map == nullptr || map->has_surface_map == 0) {
        out_is_uniform = true;
        out_section_idx = -1;
        return Status::ok;
    }
    const i32 count = map->section_count < kMaxSurfaceSections
        ? map->section_count
        : kMaxSurfaceSections;
    if (count <= 0) {
        out_is_uniform = true;
        out_section_idx = -1;
        return Status::ok;
    }

    const Real half_factor = BIKE_REAL(Real, 0.5);
    if (map->sections[0].start_m > BIKE_REAL(Real, 0.0) &&
        map->sections[0].blend_m > BIKE_REAL(Real, 0.0)) {
        const Real x_b = map->sections[0].start_m;
        const Real half_w = half_factor * map->sections[0].blend_m;
        if (x_min_m <= x_b + half_w && x_max_m >= x_b - half_w) {
            return Status::ok;
        }
    }
    for (i32 i = 0; i < count; ++i) {
        const Real end_i = map->sections[i].end_m;
        const Real blend_i = map->sections[i].blend_m;
        if (i + 1 < count && end_i == map->sections[i + 1].start_m) {
            if (blend_i > BIKE_REAL(Real, 0.0)) {
                const Real half_w = half_factor * blend_i;
                if (x_min_m <= end_i + half_w && x_max_m >= end_i - half_w) {
                    return Status::ok;
                }
            }
        } else {
            if (blend_i > BIKE_REAL(Real, 0.0)) {
                const Real half_w = half_factor * blend_i;
                if (x_min_m <= end_i + half_w && x_max_m >= end_i - half_w) {
                    return Status::ok;
                }
            }
            if (i + 1 < count && map->sections[i + 1].blend_m > BIKE_REAL(Real, 0.0)) {
                const Real next_start = map->sections[i + 1].start_m;
                const Real half_w = half_factor * map->sections[i + 1].blend_m;
                if (x_min_m <= next_start + half_w && x_max_m >= next_start - half_w) {
                    return Status::ok;
                }
            }
        }
    }

    if (map->sections[0].start_m == BIKE_REAL(Real, 0.0)) {
        if (x_max_m < map->sections[0].end_m) {
            out_is_uniform = true;
            out_section_idx = 0;
            return Status::ok;
        }
    } else {
        if (x_max_m < map->sections[0].start_m) {
            out_is_uniform = true;
            out_section_idx = -1;
            return Status::ok;
        }
        if (map->sections[0].start_m <= x_min_m &&
            x_max_m < map->sections[0].end_m) {
            out_is_uniform = true;
            out_section_idx = 0;
            return Status::ok;
        }
    }
    for (i32 i = 0; i < count; ++i) {
        if (i >= 1 && map->sections[i].start_m <= x_min_m &&
            x_max_m < map->sections[i].end_m) {
            out_is_uniform = true;
            out_section_idx = i;
            return Status::ok;
        }
        if (i + 1 < count &&
            map->sections[i].end_m < map->sections[i + 1].start_m &&
            map->sections[i].end_m <= x_min_m &&
            x_max_m < map->sections[i + 1].start_m) {
            out_is_uniform = true;
            out_section_idx = -1;
            return Status::ok;
        }
    }
    if (map->sections[count - 1].end_m <= x_min_m) {
        out_is_uniform = true;
        out_section_idx = -1;
        return Status::ok;
    }
    return Status::ok;
}

template <class Real, class MapPtr>
BIKE_HD Status uniform_surface_for_span(
    MapPtr map,
    Real x_min_m,
    Real x_max_m,
    BIKE_REF(bool) out_is_uniform,
    BIKE_REF(i32) out_section_idx,
    BIKE_REF(ErrCode) err
) noexcept {
    struct SpanFiniteToolkit {
        BIKE_HD static bool isfinite(Real x) noexcept {
#if defined(__METAL_VERSION__)
            return metal::isfinite(x);
#elif defined(__CUDACC__)
            return ::isfinite(x) != 0;
#else
            return std::isfinite(x);
#endif
        }
    };
    return uniform_surface_for_span<Real, SpanFiniteToolkit>(
        map, x_min_m, x_max_m, out_is_uniform, out_section_idx, err);
}

template <class Real, class TK>
BIKE_HD Real load_sensitivity_factor(
    Real wheel_normal_load_n,
    Real k_z
) noexcept {
    const Real kRefLoad = BIKE_REAL(Real, 500.0);
    const Real kSmoothDelta = BIKE_REAL(Real, 0.1);
    if (wheel_normal_load_n <= kRefLoad || k_z <= BIKE_REAL(Real, 0.0)) {
        return BIKE_REAL(Real, 1.0);
    }
    const Real u = TK::div(wheel_normal_load_n, kRefLoad) - BIKE_REAL(Real, 1.0);
    if (u <= BIKE_REAL(Real, 0.0)) {
        return BIKE_REAL(Real, 1.0);
    }
#if defined(__METAL_VERSION__)
    const Real kAsymptoteU = static_cast<Real>(1e15f);
#else
    constexpr Real kAsymptoteU =
        (sizeof(Real) <= 4) ? static_cast<Real>(1e15f) : static_cast<Real>(1e150);
#endif
    const Real g = (u > kAsymptoteU) ? u : TK::div(u * u, u + kSmoothDelta);
    return TK::div(BIKE_REAL(Real, 1.0), BIKE_REAL(Real, 1.0) + k_z * g);
}

template <class Real, class TK>
BIKE_HD Status sample_mu(
    SurfaceSamplePod<Real> s,
    Real sliding_speed_mps,
    Real wheel_normal_load_n,
    BIKE_REF(Real) out_mu,
    BIKE_REF(ErrCode) err
) noexcept {
    err = ErrCode::none;
    out_mu = BIKE_REAL(Real, 0.0);
    if (!TK::isfinite(sliding_speed_mps) || !TK::isfinite(wheel_normal_load_n) ||
        wheel_normal_load_n < BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Real decay =
        TK::exp(TK::div(-TK::abs(sliding_speed_mps), s.stribeck_speed_mps));
    const Real mu_str = s.mu_slide + (s.mu_peak - s.mu_slide) * decay;
    const Real kRefLoad = BIKE_REAL(Real, 500.0);
    const Real value =
        (wheel_normal_load_n <= kRefLoad ||
         s.load_sensitivity <= BIKE_REAL(Real, 0.0))
            ? mu_str
            : load_sensitivity_factor<Real, TK>(
                  wheel_normal_load_n, s.load_sensitivity) *
                  mu_str;
    if (!TK::isfinite(value)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    out_mu = value;
    return Status::ok;
}

template <class Real, class TK>
BIKE_HD Real surface_mu(SurfaceSpecPod<Real> s, Real slip) noexcept {
    const Real decay = TK::exp(TK::div(-TK::abs(slip), s.stribeck_speed_mps));
    return s.mu_slide + (s.mu_peak - s.mu_slide) * decay;
}

template <class Real, class TK, class MapPtr>
BIKE_HD Status effective_friction(
    Real tire_mu,
    MapPtr map,
    Real contact_x,
    Real slip,
    BIKE_REF(Real) out_mu,
    BIKE_REF(i32) out_section_idx,
    BIKE_REF(ErrCode) err
) noexcept {
    if (map == nullptr || map->has_surface_map == 0) {
        out_mu = tire_mu;
        out_section_idx = -1;
        err = ErrCode::none;
        return Status::ok;
    }
    if (!TK::isfinite(contact_x)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const i32 count = map->section_count < kMaxSurfaceSections
        ? map->section_count
        : kMaxSurfaceSections;
    for (i32 i = 0; i < count; ++i) {
        if (map->sections[i].start_m <= contact_x &&
            contact_x < map->sections[i].end_m) {
            out_section_idx = i;
            out_mu = TK::min(
                tire_mu, surface_mu<Real, TK>(map->sections[i].surface, slip));
            err = ErrCode::none;
            return Status::ok;
        }
    }
    out_section_idx = -1;
    out_mu = TK::min(tire_mu, surface_mu<Real, TK>(map->default_surface, slip));
    err = ErrCode::none;
    return Status::ok;
}

template <class Real, class TK>
BIKE_HD Status normal_contact_hysteresis(
    Real delta,
    Real delta_dot,
    Real k,
    Real eta,
    Real eps,
    BIKE_REF(Real) out_force,
    BIKE_REF(Real) out_energy,
    BIKE_REF(ErrCode) err
) noexcept {
    if (delta <= BIKE_REAL(Real, 0.0)) {
        out_force = BIKE_REAL(Real, 0.0);
        out_energy = BIKE_REAL(Real, 0.0);
        err = ErrCode::none;
        return Status::ok;
    }
    const Real spring_force = k * delta;
    const Real hysteresis_force =
        eta * spring_force * TK::tanh(TK::div(delta_dot, eps));
    const Real energy = BIKE_REAL(Real, 0.5) * spring_force * delta;
    const Real total_force = spring_force + hysteresis_force;
    if (!TK::isfinite(spring_force) || !TK::isfinite(hysteresis_force) ||
        !TK::isfinite(total_force) || !TK::isfinite(energy)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    out_force = TK::max(BIKE_REAL(Real, 0.0), total_force);
    out_energy = energy;
    err = ErrCode::none;
    return Status::ok;
}

template <class Real, class TK, class PtrOld, class PtrNew>
BIKE_HD void transport_bristle(
    PtrOld old_tangent,
    PtrNew new_tangent,
    Real xi,
    BIKE_REF(Real) out_xi_transp
) noexcept {
    out_xi_transp = xi * TK::dot3(old_tangent, new_tangent);
}

template <class Real, class TK>
BIKE_HD Status brush_step(
    Real xi,
    Real u,
    Real v_roll,
    Real fn,
    Real k,
    Real mu,
    Real length,
    Real dt,
    BIKE_REF(Real) out_xi,
    BIKE_REF(Real) out_force,
    BIKE_REF(Real) out_loss,
    BIKE_REF(ErrCode) err
) noexcept {
    const Real old_energy = BIKE_REAL(Real, 0.5) * k * xi * xi;
    if (!TK::isfinite(old_energy)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    if (fn == BIKE_REAL(Real, 0.0) || mu == BIKE_REAL(Real, 0.0)) {
        out_xi = BIKE_REAL(Real, 0.0);
        out_force = BIKE_REAL(Real, 0.0);
        out_loss = old_energy;
        err = ErrCode::none;
        return Status::ok;
    }
    const Real relaxation = dt * TK::div(TK::abs(v_roll), length);
    const Real denominator = BIKE_REAL(Real, 1.0) + relaxation;
    const Real trial_numerator = xi + dt * u;
    const Real friction_limit = mu * fn;
    if (!TK::isfinite(relaxation) || !TK::isfinite(denominator) ||
        !TK::isfinite(trial_numerator) || !TK::isfinite(friction_limit)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    const Real trial = TK::div(trial_numerator, denominator);
    const Real limit = TK::div(friction_limit, k);
    if (!TK::isfinite(trial) || !TK::isfinite(limit)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    const Real new_xi = TK::max(-limit, TK::min(limit, trial));
    const Real force = -k * new_xi;
    const Real new_energy = BIKE_REAL(Real, 0.5) * k * new_xi * new_xi;
    const Real change = new_xi - xi;
    const Real implicit_loss = BIKE_REAL(Real, 0.5) * k * change * change;
    const Real relaxation_loss = BIKE_REAL(Real, 2.0) * relaxation * new_energy;
    const Real projection_loss =
        (k * new_xi) * (denominator * (trial - new_xi));
    const Real loss = implicit_loss + relaxation_loss + projection_loss;
    if (!TK::isfinite(force) || !TK::isfinite(new_energy) || !TK::isfinite(loss)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    if (loss < BIKE_REAL(Real, 0.0)) {
        err = ErrCode::overflow_passivity;
        return Status::overflow;
    }
    out_xi = new_xi;
    out_force = force;
    out_loss = loss;
    err = ErrCode::none;
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
