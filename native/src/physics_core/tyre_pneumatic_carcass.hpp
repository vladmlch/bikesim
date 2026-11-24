// native/src/physics_core/tyre_pneumatic_carcass.hpp — Tri-target (Host C++20/23,
// CUDA, Apple Metal MSL) pneumatic carcass kernel: pressurized chord force,
// linear carcass stiffness, semi-Lagrangian Maxwell viscoelasticity, single-channel
// rim strike + hysteresis, rim FSM, and Stage A Bregman-Legendre potentials.
#pragma once

#include "common.hpp"
#include "tyre_pneumatic_geometry.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel header: MSL has no std::span/std::array and device buffers are address-space-qualified raw pointers. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

template <class Real>
struct PneumaticTyreSpecPod {
    Real outer_radius_mm = static_cast<Real>(0);
    Real rim_radius_mm = static_cast<Real>(0);
    Real section_height_mm = static_cast<Real>(0);
    Real pressure_bar = static_cast<Real>(0);
    Real crr_reference = static_cast<Real>(0);
    Real casing_width_mm = static_cast<Real>(0);
    Real compressed_casing_mm = static_cast<Real>(0);
    Real area_factor = static_cast<Real>(0);
    Real carcass_stiffness_n_mm2 = static_cast<Real>(0);
    Real contact_length_factor = static_cast<Real>(0);
    Real rate_stiffening = static_cast<Real>(0);
    Real rate_relaxation_s = static_cast<Real>(0);
    Real loss_factor = static_cast<Real>(0);
    Real hysteresis_rate_eps_mps = static_cast<Real>(0);
    Real relaxation_length_mm = static_cast<Real>(0);
    Real rim_stiffness_n_mm2 = static_cast<Real>(0);
    Real tread_loss_crr = static_cast<Real>(0);

    BIKE_HD Real tyre_height_mm() const noexcept {
        return outer_radius_mm - rim_radius_mm;
    }

    BIKE_HD Real rim_strike_deflection_mm() const noexcept {
        return tyre_height_mm() - compressed_casing_mm;
    }

    BIKE_HD Real pressure_pa() const noexcept {
        return pressure_bar * BIKE_REAL(Real, 100000.0);
    }
};

template <class Real>
struct PneumaticRimStrikeEvent {
    Real x_m = static_cast<Real>(0);
    Real speed_mps = static_cast<Real>(0);
    Real peak_load_n = static_cast<Real>(0);
    Real peak_rim_force_n = static_cast<Real>(0);
    Real absorbed_energy_j = static_cast<Real>(0);
    u8 valid = 0;
    u8 pad[3]{};
};

template <class Real>
struct PneumaticCarcassStatePod {
    Real delta_prev_m[kMaxPneumaticRays]{};
    Real elastic_prev_n[kMaxPneumaticRays]{};
    Real maxwell_force_n[kMaxPneumaticRays]{};
    Real rim_event_x_m = static_cast<Real>(0);
    Real rim_event_speed_mps = static_cast<Real>(0);
    Real rim_event_peak_load_n = static_cast<Real>(0);
    Real rim_event_peak_force_n = static_cast<Real>(0);
    Real rim_event_energy_j = static_cast<Real>(0);
    i32 n_rays = 0;
    u8 initialized = 0;
    u8 rim_strike_active = 0;
    u8 pad[2]{};
};

template <class Real>
struct PneumaticCarcassScratch {
    Real elastic_force_n[kMaxPneumaticRays]{};
    Real maxwell_force_n[kMaxPneumaticRays]{};
    Real rim_force_n[kMaxPneumaticRays]{};
    Real total_element_force_n[kMaxPneumaticRays]{};
    Real material_rate_mps[kMaxPneumaticRays]{};
    Real advected_a[kMaxPneumaticRays]{};
    Real advected_b[kMaxPneumaticRays]{};
    PneumaticRayPatch contact_patches[kMaxPneumaticPatches]{};
    Real contact_lengths_m[kMaxPneumaticPatches]{};
    i32 patch_count = 0;
    Real force_x_n = static_cast<Real>(0);
    Real force_z_n = static_cast<Real>(0);
    Real normal_load_n = static_cast<Real>(0);
    Real rim_energy_j = static_cast<Real>(0);
    Real dissipated_power_w = static_cast<Real>(0);
    PneumaticRimStrikeEvent<Real> rim_event{};
    u8 rim_strike_active = 0;
    u8 pad[3]{};
};

template <class Real, class TK>
BIKE_HD Real casing_chord_integral_m2(
    Real deflection_m,
    Real casing_radius_m
) noexcept {
    const Real d = TK::max(deflection_m, static_cast<Real>(0));
    const Real rho = casing_radius_m;
    if (d == static_cast<Real>(0) || rho <= static_cast<Real>(0)) {
        return static_cast<Real>(0);
    }
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kPi = BIKE_REAL(Real, 3.14159265358979323846);
    if (d >= rho) {
        return BIKE_REAL(Real, 0.5) * kPi * rho * rho +
               BIKE_REAL(Real, 2.0) * rho * (d - rho);
    }
    const Real psi = BIKE_REAL(Real, 4.0) *
        pneumatic_detail::asin_val<Real, TK>(
            TK::sqrt(TK::div(d, BIKE_REAL(Real, 2.0) * rho)));
    const Real psi2 = psi * psi;
    const Real integral = psi < BIKE_REAL(Real, 0.02)
        ? TK::div(psi * psi2, BIKE_REAL(Real, 6.0)) *
              (static_cast<Real>(1) -
               TK::div(psi2, BIKE_REAL(Real, 20.0)) +
               TK::div(psi2 * psi2, BIKE_REAL(Real, 840.0)))
        : (psi - pneumatic_detail::sin_val<Real, TK>(psi));
    return BIKE_REAL(Real, 0.5) * rho * rho * integral;
}

template <class Real, class TK>
BIKE_HD Real ray_elastic_force_n(
    Real deflection_m,
    PneumaticTyreSpecPod<Real> tyre,
    Real ds_m
) noexcept {
    const Real d = TK::max(deflection_m, static_cast<Real>(0));
    const Real rho = BIKE_REAL(Real, 0.5) * tyre.casing_width_mm *
                     BIKE_REAL(Real, 0.001);
    const Real chord = d >= rho
        ? BIKE_REAL(Real, 2.0) * rho
        : BIKE_REAL(Real, 2.0) *
              TK::sqrt(d * (BIKE_REAL(Real, 2.0) * rho - d));
    return (tyre.area_factor * tyre.pressure_pa() * chord +
            tyre.carcass_stiffness_n_mm2 * BIKE_REAL(Real, 1e6) * d) *
           ds_m;
}

template <class Real, class TK>
BIKE_HD Real ray_elastic_energy_j(
    Real deflection_m,
    PneumaticTyreSpecPod<Real> tyre,
    Real ds_m
) noexcept {
    const Real d = TK::max(deflection_m, static_cast<Real>(0));
    const Real rho = BIKE_REAL(Real, 0.5) * tyre.casing_width_mm *
                     BIKE_REAL(Real, 0.001);
    return (tyre.area_factor * tyre.pressure_pa() *
                casing_chord_integral_m2<Real, TK>(d, rho) +
            BIKE_REAL(Real, 0.5) * tyre.carcass_stiffness_n_mm2 *
                BIKE_REAL(Real, 1e6) * d * d) *
           ds_m;
}

template <class Real, class TK>
BIKE_HD Real ray_maxwell_energy_j(
    Real deflection_m,
    Real elastic_prev_n,
    Real maxwell_force_n,
    PneumaticTyreSpecPod<Real> tyre,
    Real ds_m
) noexcept {
    const Real gamma = tyre.rate_stiffening;
    if (gamma <= static_cast<Real>(0)) {
        return static_cast<Real>(0);
    }
    const Real d = TK::max(deflection_m, static_cast<Real>(0));
    const Real xi = elastic_prev_n - TK::div(maxwell_force_n, gamma);
    Real equilibrium = static_cast<Real>(0);
    if (xi > static_cast<Real>(0)) {
        const Real rho = BIKE_REAL(Real, 0.5) * tyre.casing_width_mm *
                         BIKE_REAL(Real, 0.001);
        const Real cap = ray_elastic_force_n<Real, TK>(rho, tyre, ds_m);
        const Real stiffness =
            tyre.carcass_stiffness_n_mm2 * BIKE_REAL(Real, 1e6) * ds_m;
        if (xi >= cap) {
            equilibrium = stiffness == static_cast<Real>(0)
                ? rho
                : (rho + TK::div(xi - cap, stiffness));
        } else {
            const Real pressure = tyre.area_factor * tyre.pressure_pa();
            // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
            const Real half_pi =
                BIKE_REAL(Real, 0.5) * BIKE_REAL(Real, 3.14159265358979323846);
            Real phi = pneumatic_detail::asin_val<Real, TK>(TK::min(
                static_cast<Real>(1),
                TK::div(xi, BIKE_REAL(Real, 2.0) * pressure * rho * ds_m)));
            for (i32 i = 0; i < 4; ++i) {
                const Real sine = pneumatic_detail::sin_val<Real, TK>(phi);
                const Real cosine = pneumatic_detail::cos_val<Real, TK>(phi);
                const Real residual =
                    (BIKE_REAL(Real, 2.0) * pressure * rho * sine +
                     tyre.carcass_stiffness_n_mm2 * BIKE_REAL(Real, 1e6) *
                         rho * (static_cast<Real>(1) - cosine)) *
                        ds_m -
                    xi;
                const Real derivative =
                    (BIKE_REAL(Real, 2.0) * pressure * cosine +
                     tyre.carcass_stiffness_n_mm2 * BIKE_REAL(Real, 1e6) *
                         sine) *
                    rho * ds_m;
                if (derivative > static_cast<Real>(0)) {
                    phi = TK::clamp(
                        phi - TK::div(residual, derivative),
                        static_cast<Real>(0),
                        half_pi);
                }
            }
            const Real half_sin =
                pneumatic_detail::sin_val<Real, TK>(BIKE_REAL(Real, 0.5) * phi);
            equilibrium = BIKE_REAL(Real, 2.0) * rho * half_sin * half_sin;
        }
    }
    const Real term =
        ray_elastic_energy_j<Real, TK>(d, tyre, ds_m) +
        (xi > static_cast<Real>(0)
             ? (xi * equilibrium -
                ray_elastic_energy_j<Real, TK>(equilibrium, tyre, ds_m))
             : static_cast<Real>(0)) -
        xi * d;
    return gamma * TK::max(term, static_cast<Real>(0));
}

template <class Real, class TK, class PtrDelta, class PtrPrev, class PtrOut>
BIKE_HD void pneumatic_material_rate_loop(
    i32 n,
    PtrDelta delta,
    PtrPrev previous,
    Real omega_radps,
    Real dt_s,
    Real dtheta_rad,
    PtrOut out
) noexcept {
    if (n <= 0) {
        return;
    }
    if (omega_radps > static_cast<Real>(0)) {
        for (i32 i = 0; i + 1 < n; ++i) {
            const Real gradient = TK::div(delta[i + 1] - delta[i], dtheta_rad);
            out[i] = TK::div(delta[i] - previous[i], dt_s) -
                     omega_radps * gradient;
        }
        const Real gradient =
            TK::div(static_cast<Real>(0) - delta[n - 1], dtheta_rad);
        out[n - 1] = TK::div(delta[n - 1] - previous[n - 1], dt_s) -
                     omega_radps * gradient;
    } else if (omega_radps < static_cast<Real>(0)) {
        {
            const Real gradient =
                TK::div(delta[0] - static_cast<Real>(0), dtheta_rad);
            out[0] = TK::div(delta[0] - previous[0], dt_s) -
                     omega_radps * gradient;
        }
        for (i32 i = 1; i < n; ++i) {
            const Real gradient = TK::div(delta[i] - delta[i - 1], dtheta_rad);
            out[i] = TK::div(delta[i] - previous[i], dt_s) -
                     omega_radps * gradient;
        }
    } else {
        for (i32 i = 0; i < n; ++i) {
            out[i] = TK::div(delta[i] - previous[i], dt_s) -
                     omega_radps * static_cast<Real>(0);
        }
    }
}

template <class Real, class TK, class PtrIn, class PtrOut>
BIKE_HD void pneumatic_advect_material(
    i32 n,
    PtrIn values,
    Real omega_radps,
    Real dt_s,
    Real dtheta_rad,
    PtrOut out
) noexcept {
    if (n <= 0) {
        return;
    }
    if (omega_radps == static_cast<Real>(0)) {
        for (i32 i = 0; i < n; ++i) {
            out[i] = values[i];
        }
        return;
    }
    const Real shift = TK::div(omega_radps * dt_s, dtheta_rad);
    for (i32 i = 0; i < n; ++i) {
        out[i] = pneumatic_detail::np_interp_unit_grid<Real, TK>(
            static_cast<Real>(i) + shift,
            values,
            n,
            static_cast<Real>(0),
            static_cast<Real>(0));
    }
}

template <class Real, class TK, class StatePtr, class ScratchPtr>
BIKE_HD Status pneumatic_advance_unloaded(
    StatePtr state,
    Real omega_radps,
    Real dt_s,
    Real dtheta_rad,
    Real relaxation_s,
    ScratchPtr scratch_row,
    BIKE_REF(ErrCode) err
) noexcept {
    if (state == nullptr || scratch_row == nullptr) {
        err = ErrCode::out_of_bounds_station;
        return Status::invalid_argument;
    }
    if (state->initialized == 0) {
        err = ErrCode::none;
        return Status::ok;
    }
    if (dt_s <= static_cast<Real>(0) ||
        dtheta_rad <= static_cast<Real>(0) ||
        relaxation_s <= static_cast<Real>(0)) {
        err = ErrCode::bad_dt;
        return Status::invalid_argument;
    }
    const i32 n = state->n_rays;
    if (n < 3 || n > kMaxPneumaticRays) {
        err = ErrCode::out_of_bounds_station;
        return Status::invalid_argument;
    }
    bool any = false;
    for (i32 i = 0; i < n; ++i) {
        if (state->maxwell_force_n[i] != static_cast<Real>(0)) {
            any = true;
            break;
        }
    }
    if (!any) {
        err = ErrCode::none;
        return Status::ok;
    }
    pneumatic_advect_material<Real, TK>(
        n, state->maxwell_force_n, omega_radps, dt_s, dtheta_rad, scratch_row);
    const Real decay = TK::exp(-TK::div(dt_s, relaxation_s));
    for (i32 i = 0; i < n; ++i) {
        state->maxwell_force_n[i] = decay * scratch_row[i];
    }
    err = ErrCode::none;
    return Status::ok;
}

template <
    class Real,
    class TK,
    class StatePtr,
    class PtrRx,
    class PtrOv,
    class PtrRim>
BIKE_HD PneumaticRimStrikeEvent<Real> pneumatic_update_rim_event(
    StatePtr state,
    i32 n,
    bool active,
    PtrRx road_x_m,
    PtrOv overlap_m,
    PtrRim rim_force_n,
    Real vertical_load_n,
    Real energy_j,
    Real speed_mps
) noexcept {
    PneumaticRimStrikeEvent<Real> completed{};
    if (active) {
        if (state->rim_strike_active == 0) {
            i32 ray = 0;
            Real best = overlap_m[0];
            for (i32 i = 1; i < n; ++i) {
                if (overlap_m[i] > best) {
                    best = overlap_m[i];
                    ray = i;
                }
            }
            state->rim_event_x_m = road_x_m[ray];
            state->rim_event_speed_mps = TK::abs(speed_mps);
            state->rim_event_peak_load_n = static_cast<Real>(0);
            state->rim_event_peak_force_n = static_cast<Real>(0);
            state->rim_event_energy_j = static_cast<Real>(0);
        }
        state->rim_strike_active = 1;
        state->rim_event_peak_load_n =
            TK::max(state->rim_event_peak_load_n, vertical_load_n);
        const Real rim_sum =
            pneumatic_detail::array_sum<Real, TK>(n, rim_force_n);
        state->rim_event_peak_force_n =
            TK::max(state->rim_event_peak_force_n, rim_sum);
        state->rim_event_energy_j =
            TK::max(state->rim_event_energy_j, energy_j);
        return completed;
    }
    if (state->rim_strike_active == 0) {
        return completed;
    }
    completed.x_m = state->rim_event_x_m;
    completed.speed_mps = state->rim_event_speed_mps;
    completed.peak_load_n = state->rim_event_peak_load_n;
    completed.peak_rim_force_n = state->rim_event_peak_force_n;
    completed.absorbed_energy_j = state->rim_event_energy_j;
    completed.valid = 1;
    state->rim_strike_active = 0;
    state->rim_event_x_m = static_cast<Real>(0);
    state->rim_event_speed_mps = static_cast<Real>(0);
    state->rim_event_peak_load_n = static_cast<Real>(0);
    state->rim_event_peak_force_n = static_cast<Real>(0);
    state->rim_event_energy_j = static_cast<Real>(0);
    return completed;
}

template <
    class Real,
    class TK,
    class RingPtr,
    class PtrDelta,
    class PtrRx,
    class PtrRz,
    class StateInPtr,
    class StateOutPtr,
    class ScratchPtr>
BIKE_HD Status pneumatic_evaluate_carcass(
    RingPtr ring,
    PtrDelta delta_m,
    PtrRx road_x_m,
    PtrRz road_z_m,
    PneumaticTyreSpecPod<Real> tyre,
    StateInPtr state_in,
    StateOutPtr state_out,
    Real dt_s,
    Real omega_radps,
    Real speed_mps,
    ScratchPtr scratch,
    BIKE_REF(ErrCode) err
) noexcept {
    if (ring == nullptr || delta_m == nullptr || road_x_m == nullptr ||
        road_z_m == nullptr || state_in == nullptr || state_out == nullptr ||
        scratch == nullptr) {
        err = ErrCode::out_of_bounds_station;
        return Status::invalid_argument;
    }
    if (dt_s <= static_cast<Real>(0) || !TK::isfinite(dt_s)) {
        err = ErrCode::bad_dt;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(omega_radps) || !TK::isfinite(speed_mps)) {
        err = ErrCode::nonfinite_center;
        return Status::invalid_argument;
    }
    const i32 n = ring->n_rays;
    if (n < 3 || n > kMaxPneumaticRays || state_in->n_rays != n) {
        err = ErrCode::out_of_bounds_station;
        return Status::invalid_argument;
    }
    for (i32 i = 0; i < n; ++i) {
        const Real d = delta_m[i];
        if (!TK::isfinite(d) || d < static_cast<Real>(0)) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
    }

    const Real radius_m = tyre.outer_radius_mm * BIKE_REAL(Real, 0.001);
    const Real casing_width_m = tyre.casing_width_mm * BIKE_REAL(Real, 0.001);
    const Real casing_radius_m = BIKE_REAL(Real, 0.5) * casing_width_m;
    const Real ds_m = radius_m * ring->dtheta_rad;
    const Real pressure_pa = tyre.pressure_bar * BIKE_REAL(Real, 100000.0);
    const Real area_pressure = tyre.area_factor * pressure_pa;
    const Real carcass_si =
        tyre.carcass_stiffness_n_mm2 * BIKE_REAL(Real, 1000000.0);
    const Real two_casing_radius = BIKE_REAL(Real, 2.0) * casing_radius_m;

    for (i32 i = 0; i < n; ++i) {
        const Real d = delta_m[i];
        const Real inner =
            TK::max(d * (two_casing_radius - d), static_cast<Real>(0));
        const Real chord = d >= casing_radius_m
            ? casing_width_m
            : (BIKE_REAL(Real, 2.0) * TK::sqrt(inner));
        const Real pressure_line = area_pressure * chord;
        const Real carcass_line = carcass_si * d;
        scratch->elastic_force_n[i] = (pressure_line + carcass_line) * ds_m;
    }

    Real maxwell_pre_j = static_cast<Real>(0);
    Real maxwell_post_j = static_cast<Real>(0);
    if (state_in->initialized != 0) {
        pneumatic_material_rate_loop<Real, TK>(
            n, delta_m, state_in->delta_prev_m, omega_radps, dt_s,
            ring->dtheta_rad, scratch->material_rate_mps);
        pneumatic_advect_material<Real, TK>(
            n, state_in->elastic_prev_n, omega_radps, dt_s,
            ring->dtheta_rad, scratch->advected_a);
        pneumatic_advect_material<Real, TK>(
            n, state_in->maxwell_force_n, omega_radps, dt_s,
            ring->dtheta_rad, scratch->advected_b);
        const Real decay = TK::exp(-TK::div(dt_s, tyre.rate_relaxation_s));
        for (i32 i = 0; i < n; ++i) {
            scratch->maxwell_force_n[i] =
                decay * scratch->advected_b[i] +
                tyre.rate_stiffening *
                    (scratch->elastic_force_n[i] - scratch->advected_a[i]);
        }
        for (i32 i = 0; i < n; ++i) {
            maxwell_pre_j += ray_maxwell_energy_j<Real, TK>(
                state_in->delta_prev_m[i],
                state_in->elastic_prev_n[i],
                state_in->maxwell_force_n[i],
                tyre, ds_m);
            maxwell_post_j += ray_maxwell_energy_j<Real, TK>(
                state_in->delta_prev_m[i],
                scratch->elastic_force_n[i],
                scratch->maxwell_force_n[i],
                tyre, ds_m);
        }
    } else {
        for (i32 i = 0; i < n; ++i) {
            scratch->material_rate_mps[i] = static_cast<Real>(0);
            scratch->maxwell_force_n[i] = static_cast<Real>(0);
        }
    }

    const Real rim_deflection_m =
        tyre.rim_strike_deflection_mm() * BIKE_REAL(Real, 0.001);
    const Real rim_si = tyre.rim_stiffness_n_mm2 * BIKE_REAL(Real, 1000000.0);
    Real hysteresis_loss_w = static_cast<Real>(0);
    bool rim_active = false;
    for (i32 i = 0; i < n; ++i) {
        const Real d = delta_m[i];
        const Real overlap =
            TK::max(d - rim_deflection_m, static_cast<Real>(0));
        scratch->advected_a[i] = overlap;
        if (overlap > static_cast<Real>(0)) {
            rim_active = true;
        }
        const Real rim_elastic = rim_si * overlap * ds_m;
        const Real sign = TK::tanh(
            TK::div(scratch->material_rate_mps[i], tyre.hysteresis_rate_eps_mps));
        const Real carcass_hysteresis =
            tyre.loss_factor * scratch->elastic_force_n[i] * sign;
        const Real rim_hysteresis = tyre.loss_factor * rim_elastic * sign;
        hysteresis_loss_w += TK::max(
            (carcass_hysteresis + rim_hysteresis) *
                scratch->material_rate_mps[i],
            static_cast<Real>(0));
        scratch->rim_force_n[i] = rim_elastic + rim_hysteresis;
        const Real total = TK::max(
            scratch->elastic_force_n[i] + scratch->maxwell_force_n[i] +
                carcass_hysteresis + scratch->rim_force_n[i],
            static_cast<Real>(0));
        scratch->total_element_force_n[i] =
            d > static_cast<Real>(0) ? total : static_cast<Real>(0);
        scratch->advected_b[i] = rim_si * (overlap * overlap) * ds_m;
    }

    scratch->force_x_n =
        -TK::dot(n, scratch->total_element_force_n, ring->ux);
    scratch->force_z_n =
        -TK::dot(n, scratch->total_element_force_n, ring->uz);
    scratch->normal_load_n =
        pneumatic_detail::array_sum<Real, TK>(n, scratch->total_element_force_n);
    scratch->rim_energy_j =
        BIKE_REAL(Real, 0.5) *
        pneumatic_detail::array_sum<Real, TK>(n, scratch->advected_b);
    const Real maxwell_delta = maxwell_pre_j - maxwell_post_j;
    if (!TK::isfinite(hysteresis_loss_w) || !TK::isfinite(maxwell_pre_j) ||
        !TK::isfinite(maxwell_post_j) || !TK::isfinite(maxwell_delta)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    scratch->dissipated_power_w =
        hysteresis_loss_w +
        TK::div(TK::max(static_cast<Real>(0), maxwell_delta), dt_s);
    if (!TK::isfinite(scratch->dissipated_power_w)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }

    i32 patch_count_local = 0;
    const Status patch_st = pneumatic_patches<Real>(
        n, delta_m, kMaxPneumaticPatches,
        scratch->contact_patches, patch_count_local, err);
    if (patch_st != Status::ok) {
        return patch_st;
    }
    scratch->patch_count = patch_count_local;
    for (i32 k = 0; k < scratch->patch_count; ++k) {
        const i32 start = scratch->contact_patches[k].start;
        const i32 stop = scratch->contact_patches[k].stop;
        const Real length_m = (stop - start == 1)
            ? ds_m
            : pneumatic_detail::hypot_libm<Real, TK>(
                  road_x_m[stop - 1] - road_x_m[start],
                  road_z_m[stop - 1] - road_z_m[start]);
        scratch->contact_lengths_m[k] = tyre.contact_length_factor * length_m;
    }

    // Stage pre-step rim FSM fields onto state_out before running FSM.
    state_out->n_rays = n;
    state_out->rim_strike_active = state_in->rim_strike_active;
    state_out->rim_event_x_m = state_in->rim_event_x_m;
    state_out->rim_event_speed_mps = state_in->rim_event_speed_mps;
    state_out->rim_event_peak_load_n = state_in->rim_event_peak_load_n;
    state_out->rim_event_peak_force_n = state_in->rim_event_peak_force_n;
    state_out->rim_event_energy_j = state_in->rim_event_energy_j;
    scratch->rim_strike_active = rim_active ? 1 : 0;
    scratch->rim_event = pneumatic_update_rim_event<Real, TK>(
        state_out, n, rim_active, road_x_m, scratch->advected_a,
        scratch->rim_force_n, scratch->force_z_n, scratch->rim_energy_j,
        speed_mps);

    for (i32 i = 0; i < n; ++i) {
        state_out->delta_prev_m[i] = delta_m[i];
        state_out->elastic_prev_n[i] = scratch->elastic_force_n[i];
        state_out->maxwell_force_n[i] = scratch->maxwell_force_n[i];
    }
    state_out->n_rays = n;
    state_out->initialized = 1;
    err = ErrCode::none;
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
