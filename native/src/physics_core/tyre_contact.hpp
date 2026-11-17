// native/src/physics_core/tyre_contact.hpp — Tri-target shared tyre contact
// primitives (Stribeck surface friction, rate-independent radial hysteresis,
// bristle transport, and brush shear step) preserving 0-bit CPU FP64 parity.
#pragma once

#include "common.hpp"
#include "tyre_types.hpp"

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
    Real &out_mu,
    i32 &out_section_idx,
    ErrCode &err
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
    Real &out_force,
    Real &out_energy,
    ErrCode &err
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
    Real &out_xi_transp
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
    Real &out_xi,
    Real &out_force,
    Real &out_loss,
    ErrCode &err
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
