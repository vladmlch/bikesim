// native/src/physics_core/tyre_pneumatic_model.hpp — Tri-target (Host C++20/23,
// CUDA, Apple Metal MSL) whole-wheel pneumatic tyre kernel: patch overlap migration,
// Stage A surface dispatch, lumped/discretised brush integration, world centroid +
// rolling-moment generalized force projection, and full energy/dissipation accounting.
#pragma once

#include "common.hpp"
#include "kinematics.hpp"
#include "tyre_pneumatic_brush.hpp"
#include "tyre_pneumatic_carcass.hpp"
#include "tyre_pneumatic_geometry.hpp"
#include "tyre_types.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel header: MSL has no std::span/std::array and device buffers are address-space-qualified raw pointers. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

template <class Real>
struct GpuPneumaticPatchMeta {
    i32 start = 0;
    i32 stop = 0;
    Real kappa_prime = static_cast<Real>(0);
    Real detailed_kappa_prime = static_cast<Real>(0);
    u8 has_detailed = 0;
    u8 active = 0;
    u8 pad[2]{};
};

template <class Real>
struct GpuPneumaticWheelState {
    PneumaticCarcassStatePod<Real> carcass{};
    Real bristle_deflection_m[kMaxPneumaticRays]{};
    GpuPneumaticPatchMeta<Real> patches[kMaxPneumaticPatches]{};
    i32 patch_count = 0;
};

template <class Real>
struct PneumaticWheelSpec {
    PneumaticTyreSpecPod<Real> tyre{};
    PneumaticRayRing<Real> ring{};
    GpuWheelChain chain{};
    i32 wheel_body_id = -1;
    i32 nv = 0;
    i32 njnt = 0;
    u8 discretised_brush = 0;
    u8 pad[3]{};
};

template <class Real>
struct GpuPneumaticWheelOutput {
    Real force_world_n[3]{};
    Real normal_load_n = static_cast<Real>(0);
    Real support_n = static_cast<Real>(0);
    Real tangent_force_n = static_cast<Real>(0);
    Real mean_deflection_m = static_cast<Real>(0);
    Real contact_length_m = static_cast<Real>(0);
    Real effective_radius_m = static_cast<Real>(0);
    Real slip_ratio = static_cast<Real>(0);
    Real transient_slip_ratio = static_cast<Real>(0);
    Real slip_mps = static_cast<Real>(0);
    Real friction_coefficient = static_cast<Real>(0);
    Real rolling_crr = static_cast<Real>(0);
    Real rolling_moment_nm = static_cast<Real>(0);
    Real radial_energy_j = static_cast<Real>(0);
    Real maxwell_energy_j = static_cast<Real>(0);
    Real rim_energy_j = static_cast<Real>(0);
    Real shear_energy_j = static_cast<Real>(0);
    Real carcass_loss_j = static_cast<Real>(0);
    Real brush_and_rolling_loss_j = static_cast<Real>(0);
    Real carcass_dissipated_power_w = static_cast<Real>(0);
    Real dissipated_power_w = static_cast<Real>(0);
    PneumaticRimStrikeEvent<Real> rim_event{};
    i32 patch_count = 0;
    i32 dominant_surface_id = 0;
    u8 fully_sliding = 0;
    u8 airborne = 1;
    u8 coverage_event = 0;
    u8 rim_strike_active = 0;
};

template <class Real>
struct GpuPneumaticStepScratch {
    Real r_m[kMaxPneumaticRays]{};
    Real delta_m[kMaxPneumaticRays]{};
    Real road_x_m[kMaxPneumaticRays]{};
    Real road_z_m[kMaxPneumaticRays]{};
    PneumaticCarcassScratch<Real> carcass{};
    PneumaticSurfaceSample<Real> ray_samples[kMaxPneumaticRays]{};
    Real brush_updated[kMaxPneumaticRays]{};
    Real brush_force_n[kMaxPneumaticRays]{};
    Real brush_stiffness[kMaxPneumaticRays]{};
    Real brush_mu[kMaxPneumaticRays]{};
    Real scratch_bristles[kMaxPneumaticRays]{};
    Real loss_scratch[kMaxPneumaticRays]{};
    Real energy_scratch[kMaxPneumaticRays]{};
    Real jp[3 * kMaxNv]{};
    Real jr[3 * kMaxNv]{};
    Real tmp_nv[kMaxNv]{};
    Real patch_centroids[3 * kMaxPneumaticPatches]{};
    Real patch_normals[3 * kMaxPneumaticPatches]{};
    Real patch_forces[3 * kMaxPneumaticPatches]{};
    Real patch_loads[kMaxPneumaticPatches]{};
    GpuPneumaticPatchMeta<Real> next_patches[kMaxPneumaticPatches]{};
    u8 used_patches[kMaxPneumaticPatches]{};
};

template <class Real, class TK>
BIKE_HD Real pneumatic_clamp_pressure_bar(Real pressure_bar) noexcept {
    return TK::min(
        BIKE_REAL(Real, 3.0),
        TK::max(BIKE_REAL(Real, 0.8), pressure_bar));
}

template <class Real, class TK>
BIKE_HD Real pneumatic_tread_loss_crr_at_pressure(
    PneumaticTyreSpecPod<Real> tyre
) noexcept {
    return tyre.tread_loss_crr *
           TK::pow_py(
               TK::div(tyre.pressure_bar, BIKE_REAL(Real, 1.5)),
               BIKE_REAL(Real, -0.3));
}

template <class Real, class StatePtr>
BIKE_HD void init_pneumatic_wheel_state(
    i32 n_rays,
    StatePtr state
) noexcept {
    if (state == nullptr) {
        return;
    }
    state->carcass.n_rays = n_rays;
    state->carcass.initialized = 0;
    state->carcass.rim_strike_active = 0;
    state->carcass.rim_event_x_m = static_cast<Real>(0);
    state->carcass.rim_event_speed_mps = static_cast<Real>(0);
    state->carcass.rim_event_peak_load_n = static_cast<Real>(0);
    state->carcass.rim_event_peak_force_n = static_cast<Real>(0);
    state->carcass.rim_event_energy_j = static_cast<Real>(0);
    for (i32 i = 0; i < kMaxPneumaticRays; ++i) {
        state->carcass.delta_prev_m[i] = static_cast<Real>(0);
        state->carcass.elastic_prev_n[i] = static_cast<Real>(0);
        state->carcass.maxwell_force_n[i] = static_cast<Real>(0);
        state->bristle_deflection_m[i] = static_cast<Real>(0);
    }
    for (i32 k = 0; k < kMaxPneumaticPatches; ++k) {
        state->patches[k] = GpuPneumaticPatchMeta<Real>{};
    }
    state->patch_count = 0;
}

template <
    class Real,
    class PtrNewPatches,
    class StateInPtr,
    class StateOutPtr,
    class ScratchPtr>
BIKE_HD void pneumatic_migrate_patches(
    i32 n_rays,
    bool discretised,
    i32 new_count,
    PtrNewPatches new_patches,
    StateInPtr prev_state,
    StateOutPtr next_state,
    ScratchPtr scratch
) noexcept {
    for (i32 i = 0; i < n_rays; ++i) {
        scratch->scratch_bristles[i] = static_cast<Real>(0);
    }
    for (i32 i = 0; i < kMaxPneumaticPatches; ++i) {
        scratch->used_patches[i] = 0;
    }

    const i32 prev_count = prev_state->patch_count < kMaxPneumaticPatches
        ? prev_state->patch_count
        : kMaxPneumaticPatches;

    for (i32 p = 0; p < new_count; ++p) {
        const i32 start_i = new_patches[p].start;
        const i32 stop_i = new_patches[p].stop;
        i32 old_slot = -1;
        for (i32 i = 0; i < prev_count; ++i) {
            if (scratch->used_patches[i] == 0 &&
                prev_state->patches[i].start == start_i &&
                prev_state->patches[i].stop == stop_i) {
                old_slot = i;
                break;
            }
        }
        if (old_slot < 0) {
            i32 best_overlap = 0;
            for (i32 i = 0; i < prev_count; ++i) {
                if (scratch->used_patches[i] != 0) {
                    continue;
                }
                const i32 os = pneumatic_detail::max_i32(
                    start_i, prev_state->patches[i].start);
                const i32 oe = pneumatic_detail::min_i32(
                    stop_i, prev_state->patches[i].stop);
                const i32 overlap = oe - os;
                if (overlap <= 0) {
                    continue;
                }
                if (old_slot < 0 || overlap > best_overlap ||
                    (overlap == best_overlap &&
                     (prev_state->patches[i].start >
                          prev_state->patches[old_slot].start ||
                      (prev_state->patches[i].start ==
                           prev_state->patches[old_slot].start &&
                       prev_state->patches[i].stop >
                           prev_state->patches[old_slot].stop)))) {
                    old_slot = i;
                    best_overlap = overlap;
                }
            }
        }

        GpuPneumaticPatchMeta<Real> ns{};
        ns.start = start_i;
        ns.stop = stop_i;
        ns.has_detailed = discretised ? 1 : 0;
        ns.active = 1;
        ns.detailed_kappa_prime = static_cast<Real>(0);
        if (old_slot < 0) {
            ns.kappa_prime = static_cast<Real>(0);
        } else {
            scratch->used_patches[old_slot] = 1;
            const GpuPneumaticPatchMeta<Real> prev_meta =
                prev_state->patches[old_slot];
            ns.kappa_prime = prev_meta.kappa_prime;
            if (discretised && prev_meta.has_detailed != 0) {
                ns.detailed_kappa_prime = prev_meta.detailed_kappa_prime;
                const i32 os = pneumatic_detail::max_i32(
                    start_i, prev_meta.start);
                const i32 oe = pneumatic_detail::min_i32(
                    stop_i, prev_meta.stop);
                const i32 prev_len = prev_meta.stop - prev_meta.start;
                if (prev_len == 1 && (oe - os) >= 1) {
                    const Real val =
                        prev_state->bristle_deflection_m[prev_meta.start];
                    for (i32 r = os; r < oe; ++r) {
                        scratch->scratch_bristles[r] = val;
                    }
                } else {
                    for (i32 r = os; r < oe; ++r) {
                        scratch->scratch_bristles[r] =
                            prev_state->bristle_deflection_m[r];
                    }
                }
            }
        }
        scratch->next_patches[p] = ns;
    }

    for (i32 p = 0; p < new_count; ++p) {
        next_state->patches[p] = scratch->next_patches[p];
    }
    for (i32 p = new_count; p < kMaxPneumaticPatches; ++p) {
        next_state->patches[p] = GpuPneumaticPatchMeta<Real>{};
    }
    next_state->patch_count = new_count;
    for (i32 i = 0; i < n_rays; ++i) {
        next_state->bristle_deflection_m[i] = scratch->scratch_bristles[i];
    }
}

template <
    class Real,
    class TK,
    class RingPtr,
    class StatePtr,
    class PtrDelta,
    class PtrScratch>
BIKE_HD void pneumatic_compute_radial_energies(
    PneumaticTyreSpecPod<Real> tyre,
    RingPtr ring,
    StatePtr state,
    PtrDelta delta_m,
    PtrScratch energy_scratch,
    BIKE_REF(Real) out_radial_j,
    BIKE_REF(Real) out_maxwell_j,
    BIKE_REF(Real) out_rim_j
) noexcept {
    const i32 n = ring->n_rays;
    const Real ds_m =
        tyre.outer_radius_mm * BIKE_REAL(Real, 0.001) * ring->dtheta_rad;
    const Real k_rim =
        tyre.rim_stiffness_n_mm2 * BIKE_REAL(Real, 1000000.0);
    const Real rim_m =
        tyre.rim_strike_deflection_mm() * BIKE_REAL(Real, 0.001);

    for (i32 i = 0; i < n; ++i) {
        energy_scratch[i] = ray_elastic_energy_j<Real, TK>(delta_m[i], tyre, ds_m);
    }
    out_radial_j = pneumatic_detail::array_sum<Real, TK>(n, energy_scratch);

    if (state->carcass.initialized != 0 && tyre.rate_stiffening > static_cast<Real>(0)) {
        for (i32 i = 0; i < n; ++i) {
            energy_scratch[i] = ray_maxwell_energy_j<Real, TK>(
                delta_m[i],
                state->carcass.elastic_prev_n[i],
                state->carcass.maxwell_force_n[i],
                tyre,
                ds_m);
        }
        out_maxwell_j = pneumatic_detail::array_sum<Real, TK>(n, energy_scratch);
    } else {
        out_maxwell_j = static_cast<Real>(0);
    }

    for (i32 i = 0; i < n; ++i) {
        const Real overlap = TK::max(delta_m[i] - rim_m, static_cast<Real>(0));
        energy_scratch[i] = BIKE_REAL(Real, 0.5) * k_rim * overlap * overlap * ds_m;
    }
    out_rim_j = pneumatic_detail::array_sum<Real, TK>(n, energy_scratch);
}

template <
    class Real,
    class TK,
    class SpecPtr,
    class MapPtr,
    class PtrPx,
    class PtrPz,
    class PtrHub,
    class PtrVel,
    class StateInPtr,
    class StateOutPtr,
    class ScratchPtr>
BIKE_HD Status evaluate_pneumatic_wheel(
    SpecPtr spec,
    MapPtr surface_map,
    PtrHub hub_pos,
    PtrVel hub_vel,
    Real omega_forward_radps,
    PtrPx road_px,
    PtrPz road_pz,
    i32 road_num_vertices,
    Real road_x0_m,
    Real road_dx_m,
    bool road_is_uniform_grid,
    Real dt_s,
    StateInPtr prev_state,
    StateOutPtr next_state,
    BIKE_REF(GpuPneumaticWheelOutput<Real>) out,
    ScratchPtr scratch,
    BIKE_REF(ErrCode) err
) noexcept {
    out = GpuPneumaticWheelOutput<Real>{};
    if (spec == nullptr || hub_pos == nullptr || hub_vel == nullptr ||
        prev_state == nullptr || next_state == nullptr || scratch == nullptr) {
        err = ErrCode::out_of_bounds_station;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(hub_pos[0]) || !TK::isfinite(hub_pos[1]) ||
        !TK::isfinite(hub_pos[2]) || !TK::isfinite(hub_vel[0]) ||
        !TK::isfinite(hub_vel[1]) || !TK::isfinite(hub_vel[2]) ||
        !TK::isfinite(omega_forward_radps)) {
        err = ErrCode::nonfinite_center;
        return Status::invalid_argument;
    }

    const i32 n = spec->ring.n_rays;
    const Real outer_radius_m =
        spec->tyre.outer_radius_mm * BIKE_REAL(Real, 0.001);

    u8 coverage_event = 0;
    u8 airborne = 0;
    if (road_is_uniform_grid) {
        const Status ist = pneumatic_intersect_uniform_grid<Real, TK>(
            &spec->ring,
            hub_pos[0], hub_pos[2], outer_radius_m,
            road_x0_m, road_dx_m, road_num_vertices, road_px, road_pz,
            scratch->r_m, scratch->delta_m, scratch->road_x_m, scratch->road_z_m,
            coverage_event, airborne, err);
        if (ist != Status::ok) {
            return ist;
        }
    } else {
        PolylineView<Real> pl{};
        pl.px = road_px;
        pl.pz = road_pz;
        pl.num_vertices = road_num_vertices;
        const Status ist = pneumatic_intersect_polyline<Real, TK>(
            &spec->ring,
            hub_pos[0], hub_pos[2], outer_radius_m, pl,
            scratch->r_m, scratch->delta_m, scratch->road_x_m, scratch->road_z_m,
            coverage_event, airborne, err);
        if (ist != Status::ok) {
            return ist;
        }
    }

    bool any_delta_prev = false;
    bool any_elastic_prev = false;
    for (i32 i = 0; i < n; ++i) {
        if (prev_state->carcass.delta_prev_m[i] != static_cast<Real>(0)) {
            any_delta_prev = true;
        }
        if (prev_state->carcass.elastic_prev_n[i] != static_cast<Real>(0)) {
            any_elastic_prev = true;
        }
    }

    if (airborne != 0 && prev_state->carcass.rim_strike_active == 0 &&
        !any_delta_prev && !any_elastic_prev) {
        Real power = static_cast<Real>(0);
        Real maxwell_after = static_cast<Real>(0);
        // Stage carcass copy onto next_state only after validation passes.
        if (prev_state->carcass.initialized != 0) {
            if (!TK::isfinite(dt_s) || dt_s <= static_cast<Real>(0)) {
                err = ErrCode::bad_dt;
                return Status::invalid_argument;
            }
            const Real ds_m =
                outer_radius_m * spec->ring.dtheta_rad;
            Real before = static_cast<Real>(0);
            for (i32 i = 0; i < n; ++i) {
                before += ray_maxwell_energy_j<Real, TK>(
                    static_cast<Real>(0),
                    prev_state->carcass.elastic_prev_n[i],
                    prev_state->carcass.maxwell_force_n[i],
                    spec->tyre, ds_m);
            }
            // Copy carcass into scratch_bristles / carcass scratch first, or stage onto next_state.
            next_state->carcass = prev_state->carcass;
            const Status adv_st = pneumatic_advance_unloaded<Real, TK>(
                &next_state->carcass,
                omega_forward_radps, dt_s, spec->ring.dtheta_rad,
                spec->tyre.rate_relaxation_s,
                scratch->scratch_bristles, err);
            if (adv_st != Status::ok) {
                return adv_st;
            }
            for (i32 i = 0; i < n; ++i) {
                const Real force = next_state->carcass.maxwell_force_n[i];
                if (!TK::isfinite(force)) {
                    err = ErrCode::overflow_nonfinite;
                    return Status::overflow;
                }
                maxwell_after += ray_maxwell_energy_j<Real, TK>(
                    static_cast<Real>(0),
                    next_state->carcass.elastic_prev_n[i],
                    force, spec->tyre, ds_m);
            }
            const Real delta = before - maxwell_after;
            if (!TK::isfinite(before) || !TK::isfinite(maxwell_after) ||
                !TK::isfinite(delta)) {
                err = ErrCode::overflow_nonfinite;
                return Status::overflow;
            }
            power = TK::div(TK::max(static_cast<Real>(0), delta), dt_s);
            if (!TK::isfinite(power)) {
                err = ErrCode::overflow_nonfinite;
                return Status::overflow;
            }
        } else {
            next_state->carcass = prev_state->carcass;
        }

        for (i32 p = 0; p < kMaxPneumaticPatches; ++p) {
            next_state->patches[p] = GpuPneumaticPatchMeta<Real>{};
        }
        next_state->patch_count = 0;
        for (i32 i = 0; i < n; ++i) {
            next_state->bristle_deflection_m[i] = static_cast<Real>(0);
        }

        PneumaticSurfaceSample<Real> hub_sample{};
        i32 hub_surf_id = 0;
        const Status surf_st = eval_surface_sample<Real, TK>(
            surface_map, hub_pos[0], hub_sample, hub_surf_id, err);
        if (surf_st != Status::ok) {
            return surf_st;
        }
        Real hub_mu = static_cast<Real>(0);
        const Status mu_st = pneumatic_surface_mu<Real, TK>(
            hub_sample, static_cast<Real>(0), hub_mu, err);
        if (mu_st != Status::ok) {
            return mu_st;
        }

        out.airborne = 1;
        out.coverage_event = coverage_event;
        out.carcass_dissipated_power_w = power;
        out.dissipated_power_w = power;
        out.carcass_loss_j = power * dt_s;
        out.maxwell_energy_j = maxwell_after;
        out.friction_coefficient = hub_mu;
        out.dominant_surface_id = hub_surf_id;
        err = ErrCode::none;
        return Status::ok;
    }

    const Real speed_mps =
        pneumatic_detail::hypot_libm<Real, TK>(hub_vel[0], hub_vel[2]);
    const Status cst = pneumatic_evaluate_carcass<Real, TK>(
        &spec->ring,
        scratch->delta_m,
        scratch->road_x_m,
        scratch->road_z_m,
        spec->tyre,
        &prev_state->carcass,
        &next_state->carcass,
        dt_s,
        omega_forward_radps,
        speed_mps,
        &scratch->carcass,
        err);
    if (cst != Status::ok) {
        return cst;
    }

    if (scratch->carcass.patch_count == 0) {
        for (i32 p = 0; p < kMaxPneumaticPatches; ++p) {
            next_state->patches[p] = GpuPneumaticPatchMeta<Real>{};
        }
        next_state->patch_count = 0;
        for (i32 i = 0; i < n; ++i) {
            next_state->bristle_deflection_m[i] = static_cast<Real>(0);
        }

        PneumaticSurfaceSample<Real> hub_sample{};
        i32 hub_surf_id = 0;
        const Status surf_st = eval_surface_sample<Real, TK>(
            surface_map, hub_pos[0], hub_sample, hub_surf_id, err);
        if (surf_st != Status::ok) {
            return surf_st;
        }
        Real hub_mu = static_cast<Real>(0);
        const Status mu_st = pneumatic_surface_mu<Real, TK>(
            hub_sample, static_cast<Real>(0), hub_mu, err);
        if (mu_st != Status::ok) {
            return mu_st;
        }

        Real rad_j = static_cast<Real>(0);
        Real max_j = static_cast<Real>(0);
        Real rim_j = static_cast<Real>(0);
        pneumatic_compute_radial_energies<Real, TK>(
            spec->tyre, &spec->ring, next_state, scratch->delta_m,
            scratch->energy_scratch, rad_j, max_j, rim_j);

        out.airborne = airborne;
        out.coverage_event = coverage_event;
        out.rim_strike_active = scratch->carcass.rim_strike_active;
        out.rim_event = scratch->carcass.rim_event;
        out.carcass_dissipated_power_w = scratch->carcass.dissipated_power_w;
        out.dissipated_power_w = scratch->carcass.dissipated_power_w;
        out.carcass_loss_j = scratch->carcass.dissipated_power_w * dt_s;
        out.radial_energy_j = rad_j;
        out.maxwell_energy_j = max_j;
        out.rim_energy_j = rim_j;
        out.friction_coefficient = hub_mu;
        out.dominant_surface_id = hub_surf_id;
        err = ErrCode::none;
        return Status::ok;
    }

    const bool discretised = spec->discretised_brush != 0;
    pneumatic_migrate_patches<Real>(
        n, discretised, scratch->carcass.patch_count,
        scratch->carcass.contact_patches, prev_state, next_state, scratch);

    const Real carcass_loss_w = scratch->carcass.dissipated_power_w;
    if (!TK::isfinite(carcass_loss_w)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }

    Real wheel_normal_load_n = static_cast<Real>(0);
    for (i32 k = 0; k < scratch->carcass.patch_count; ++k) {
        const i32 start_i = scratch->carcass.contact_patches[k].start;
        const i32 stop_i = scratch->carcass.contact_patches[k].stop;
        const i32 w = stop_i - start_i;
        const Real nx = -TK::dot(
            w, scratch->carcass.total_element_force_n + start_i,
            spec->ring.ux + start_i);
        const Real nz = -TK::dot(
            w, scratch->carcass.total_element_force_n + start_i,
            spec->ring.uz + start_i);
        const Real nvec[3] = {nx, static_cast<Real>(0), nz};
        wheel_normal_load_n += TK::norm3_ddot(nvec);
    }
    if (!TK::isfinite(wheel_normal_load_n)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }

    Real total_force_world_n[3]{};
    Real total_load_n = static_cast<Real>(0);
    Real total_support_n = static_cast<Real>(0);
    Real total_tangent_n = static_cast<Real>(0);
    Real weighted_deflection_m = static_cast<Real>(0);
    Real weighted_kappa = static_cast<Real>(0);
    Real weighted_kappa_prime = static_cast<Real>(0);
    Real weighted_slip_mps = static_cast<Real>(0);
    Real weighted_mu = static_cast<Real>(0);
    Real total_contact_length_m = static_cast<Real>(0);
    Real brush_loss_w = static_cast<Real>(0);
    Real total_shear_energy_j = static_cast<Real>(0);
    Real rolling_crr = static_cast<Real>(0);
    u8 any_fully_sliding = 0;
    i32 emitted_patches = 0;
    i32 first_surface_id = 0;

    const Real relax_m =
        spec->tyre.relaxation_length_mm * BIKE_REAL(Real, 0.001);
    const Real tread_crr_p =
        pneumatic_tread_loss_crr_at_pressure<Real, TK>(spec->tyre);

    for (i32 k = 0; k < scratch->carcass.patch_count; ++k) {
        const i32 start_i = scratch->carcass.contact_patches[k].start;
        const i32 stop_i = scratch->carcass.contact_patches[k].stop;
        const i32 w = stop_i - start_i;
        const Real contact_length_m = scratch->carcass.contact_lengths_m[k];

        bool any_pos = false;
        for (i32 i = 0; i < w; ++i) {
            if (scratch->carcass.total_element_force_n[start_i + i] >
                static_cast<Real>(0)) {
                any_pos = true;
                break;
            }
        }
        if (!any_pos) {
            continue;
        }

        const Real normal_force_world_n[3] = {
            -TK::dot(w, scratch->carcass.total_element_force_n + start_i,
                     spec->ring.ux + start_i),
            static_cast<Real>(0),
            -TK::dot(w, scratch->carcass.total_element_force_n + start_i,
                     spec->ring.uz + start_i)
        };
        const Real normal_load_n = TK::norm3_ddot(normal_force_world_n);
        if (normal_load_n <= static_cast<Real>(0)) {
            continue;
        }
        const Real normal_world[3] = {
            TK::div(normal_force_world_n[0], normal_load_n),
            TK::div(static_cast<Real>(0), normal_load_n),
            TK::div(normal_force_world_n[2], normal_load_n)
        };
        const Real tangent_world[3] = {
            normal_world[2],
            static_cast<Real>(0),
            -normal_world[0]
        };

        const Real weight_sum = pneumatic_detail::array_sum<Real, TK>(
            w, scratch->carcass.total_element_force_n + start_i);
        const Real centroid_world_m[3] = {
            TK::div(TK::dot(w, scratch->carcass.total_element_force_n + start_i,
                            scratch->road_x_m + start_i),
                    weight_sum),
            hub_pos[1],
            TK::div(TK::dot(w, scratch->carcass.total_element_force_n + start_i,
                            scratch->road_z_m + start_i),
                    weight_sum)
        };
        const Real mean_deflection_m = TK::div(
            TK::dot(w, scratch->carcass.total_element_force_n + start_i,
                    scratch->delta_m + start_i),
            weight_sum);
        const Real half_length_m = BIKE_REAL(Real, 0.5) * contact_length_m;

        const Real v_x_mps = TK::dot3(hub_vel, tangent_world);
        const Real effective_radius_m = TK::max(
            outer_radius_m - TK::div(mean_deflection_m, BIKE_REAL(Real, 3.0)),
            BIKE_REAL(Real, 1e-6));
        const Real v_s_mps = v_x_mps - omega_forward_radps * effective_radius_m;
        const Real slip_ratio =
            TK::abs(v_x_mps) > BIKE_REAL(Real, 1e-9)
                ? TK::div(-v_s_mps, TK::abs(v_x_mps))
                : static_cast<Real>(0);

        Real min_x = scratch->road_x_m[start_i];
        Real max_x = min_x;
        for (i32 i = 1; i < w; ++i) {
            const Real xv = scratch->road_x_m[start_i + i];
            if (xv < min_x) {
                min_x = xv;
            }
            if (xv > max_x) {
                max_x = xv;
            }
        }
        i32 uniform_sec_idx = -1;
        bool is_uniform = false;
        const Status u_st = uniform_surface_for_span<Real, TK>(
            surface_map, min_x, max_x, uniform_sec_idx, is_uniform, err);
        if (u_st != Status::ok) {
            return u_st;
        }

        PneumaticSurfaceSample<Real> patch_surface{};
        i32 patch_surface_id = 0;
        Real patch_crr_delta = static_cast<Real>(0);
        if (is_uniform) {
            if (uniform_sec_idx >= 0) {
                patch_surface = surface_map->sections[uniform_sec_idx].surface;
                patch_surface_id =
                    surface_map->sections[uniform_sec_idx].surface_id;
            } else {
                patch_surface = surface_map->default_surface;
                patch_surface_id = surface_map->default_surface_id;
            }
            patch_crr_delta = patch_surface.crr_delta;
            if (wheel_normal_load_n > BIKE_REAL(Real, 500.0) &&
                patch_surface.load_sensitivity > static_cast<Real>(0)) {
                const Real factor = load_sensitivity_factor<Real, TK>(
                    wheel_normal_load_n, patch_surface.load_sensitivity);
                patch_surface.mu_peak *= factor;
                patch_surface.mu_slide *= factor;
            }
        } else {
            PneumaticSurfaceSample<Real> c_sample{};
            const Status cs_st = eval_surface_sample<Real, TK>(
                surface_map, centroid_world_m[0], c_sample, patch_surface_id, err);
            if (cs_st != Status::ok) {
                return cs_st;
            }
            patch_surface = PneumaticSurfaceSample<Real>{};
            patch_surface.stribeck_speed_mps = static_cast<Real>(0);
            for (i32 i = 0; i < w; ++i) {
                i32 ray_sid = 0;
                PneumaticSurfaceSample<Real> ray_sample{};
                const Status es_st = eval_surface_sample<Real, TK>(
                    surface_map, scratch->road_x_m[start_i + i],
                    ray_sample, ray_sid, err);
                if (es_st != Status::ok) {
                    return es_st;
                }
                scratch->ray_samples[i] = ray_sample;
                const Real weight = TK::div(
                    scratch->carcass.total_element_force_n[start_i + i],
                    weight_sum);
                patch_surface.mu_peak +=
                    weight * ray_sample.mu_peak;
                patch_surface.mu_slide +=
                    weight * ray_sample.mu_slide;
                patch_surface.slip_stiffness_per_load +=
                    weight * ray_sample.slip_stiffness_per_load;
                patch_surface.stribeck_speed_mps +=
                    weight * ray_sample.stribeck_speed_mps;
                patch_surface.crr_delta +=
                    weight * ray_sample.crr_delta;
                patch_surface.load_sensitivity +=
                    weight * ray_sample.load_sensitivity;
            }
            patch_crr_delta = patch_surface.crr_delta;
            const Real factor = load_sensitivity_factor<Real, TK>(
                wheel_normal_load_n, patch_surface.load_sensitivity);
            patch_surface.mu_peak *= factor;
            patch_surface.mu_slide *= factor;
        }

        const Real patch_crr = tread_crr_p + patch_crr_delta;
        if (!TK::isfinite(patch_crr) || !TK::isfinite(wheel_normal_load_n)) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }
        rolling_crr +=
            TK::div(normal_load_n, wheel_normal_load_n) * patch_crr;
        if (!TK::isfinite(rolling_crr)) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }

        PneumaticBrushResult<Real> brush_result{};
        if (discretised) {
            if (next_state->patches[k].has_detailed == 0) {
                next_state->patches[k].has_detailed = 1;
                for (i32 i = 0; i < w; ++i) {
                    next_state->bristle_deflection_m[start_i + i] =
                        static_cast<Real>(0);
                }
                next_state->patches[k].detailed_kappa_prime =
                    next_state->patches[k].kappa_prime;
            }
            Real detailed_kp = next_state->patches[k].detailed_kappa_prime;
            if (is_uniform) {
                const Status bst = discretised_brush_step<Real, TK>(
                    w,
                    scratch->carcass.total_element_force_n + start_i,
                    half_length_m, v_x_mps, v_s_mps, relax_m,
                    patch_surface, dt_s,
                    detailed_kp,
                    next_state->bristle_deflection_m + start_i,
                    scratch->brush_updated,
                    scratch->brush_force_n,
                    brush_result, err);
                if (bst != Status::ok) {
                    return bst;
                }
            } else {
                const Status bst = discretised_brush_step_heterogeneous<Real, TK>(
                    w,
                    scratch->carcass.total_element_force_n + start_i,
                    half_length_m, v_x_mps, v_s_mps, relax_m,
                    scratch->ray_samples, normal_load_n, wheel_normal_load_n,
                    dt_s,
                    detailed_kp,
                    next_state->bristle_deflection_m + start_i,
                    scratch->brush_updated,
                    scratch->brush_force_n,
                    scratch->brush_stiffness,
                    scratch->brush_mu,
                    brush_result, err);
                if (bst != Status::ok) {
                    return bst;
                }
            }
            next_state->patches[k].detailed_kappa_prime = detailed_kp;
            next_state->patches[k].kappa_prime = detailed_kp;
        } else {
            Real kp_local = next_state->patches[k].kappa_prime;
            const Status rst = pneumatic_relax<Real, TK>(
                kp_local,
                v_x_mps, v_s_mps, relax_m, dt_s,
                kp_local, err);
            if (rst != Status::ok) {
                return rst;
            }
            next_state->patches[k].kappa_prime = kp_local;
            const Status lst = pneumatic_lumped_brush<Real, TK>(
                kp_local,
                normal_load_n, half_length_m, patch_surface,
                false, static_cast<Real>(0), TK::abs(v_s_mps),
                brush_result, err);
            if (lst != Status::ok) {
                return lst;
            }
        }

        const Real tangential_force_n = brush_result.force_n;
        const Real patch_force_world_n[3] = {
            normal_force_world_n[0] + tangential_force_n * tangent_world[0],
            normal_force_world_n[1] + tangential_force_n * tangent_world[1],
            normal_force_world_n[2] + tangential_force_n * tangent_world[2]
        };
        const Real support_n = normal_force_world_n[2];
        const Real raw_slip_loss_w = -brush_result.force_n * v_s_mps;
        if (!TK::isfinite(raw_slip_loss_w)) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }
        const Real patch_loss_w =
            TK::max(static_cast<Real>(0), raw_slip_loss_w);
        brush_loss_w += patch_loss_w;

        Real shear_energy = static_cast<Real>(0);
        if (discretised && half_length_m > static_cast<Real>(0)) {
            const Real dx = TK::div(
                BIKE_REAL(Real, 2.0) * half_length_m, static_cast<Real>(w));
            const Real common = TK::div(
                weight_sum, BIKE_REAL(Real, 2.0) * TK::pow2(half_length_m));
            for (i32 i = 0; i < w; ++i) {
                const Real u = next_state->bristle_deflection_m[start_i + i];
                scratch->loss_scratch[i] = is_uniform
                    ? (u * u)
                    : (scratch->ray_samples[i].slip_stiffness_per_load *
                       common * dx * u * u);
            }
            const Real sum = pneumatic_detail::array_sum<Real, TK>(
                w, scratch->loss_scratch);
            shear_energy = is_uniform
                ? (BIKE_REAL(Real, 0.5) *
                   patch_surface.slip_stiffness_per_load *
                   common * dx * sum)
                : (BIKE_REAL(Real, 0.5) * sum);
        }
        if (!TK::isfinite(brush_loss_w) || !TK::isfinite(shear_energy)) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }
        total_shear_energy_j += shear_energy;

        if (emitted_patches == 0) {
            first_surface_id = patch_surface_id;
        }
        const i32 p3 = 3 * emitted_patches;
        scratch->patch_centroids[p3 + 0] = centroid_world_m[0];
        scratch->patch_centroids[p3 + 1] = centroid_world_m[1];
        scratch->patch_centroids[p3 + 2] = centroid_world_m[2];
        scratch->patch_normals[p3 + 0] =
            TK::div(normal_force_world_n[0], normal_load_n);
        scratch->patch_normals[p3 + 1] =
            TK::div(normal_force_world_n[1], normal_load_n);
        scratch->patch_normals[p3 + 2] =
            TK::div(normal_force_world_n[2], normal_load_n);
        scratch->patch_forces[p3 + 0] = patch_force_world_n[0];
        scratch->patch_forces[p3 + 1] = patch_force_world_n[1];
        scratch->patch_forces[p3 + 2] = patch_force_world_n[2];
        scratch->patch_loads[emitted_patches] = normal_load_n;
        ++emitted_patches;

        total_force_world_n[0] += patch_force_world_n[0];
        total_force_world_n[1] += patch_force_world_n[1];
        total_force_world_n[2] += patch_force_world_n[2];
        total_load_n += normal_load_n;
        total_support_n += support_n;
        total_tangent_n += tangential_force_n;
        weighted_deflection_m += mean_deflection_m * normal_load_n;
        weighted_kappa += slip_ratio * normal_load_n;
        weighted_kappa_prime +=
            next_state->patches[k].kappa_prime * normal_load_n;
        weighted_slip_mps += v_s_mps * normal_load_n;
        weighted_mu += brush_result.friction_coefficient * normal_load_n;
        total_contact_length_m += contact_length_m;
        if (brush_result.fully_sliding != 0) {
            any_fully_sliding = 1;
        }
    }

    Real mean_deflection_out = static_cast<Real>(0);
    Real slip_ratio_out = static_cast<Real>(0);
    Real transient_slip_out = static_cast<Real>(0);
    Real slip_mps_out = static_cast<Real>(0);
    Real mu_out = static_cast<Real>(0);
    if (total_load_n > static_cast<Real>(0)) {
        mean_deflection_out = TK::div(weighted_deflection_m, total_load_n);
        slip_ratio_out = TK::div(weighted_kappa, total_load_n);
        transient_slip_out = TK::div(weighted_kappa_prime, total_load_n);
        slip_mps_out = TK::div(weighted_slip_mps, total_load_n);
        mu_out = TK::div(weighted_mu, total_load_n);
    } else {
        PneumaticSurfaceSample<Real> hub_sample{};
        i32 hub_surf_id = 0;
        const Status surf_st = eval_surface_sample<Real, TK>(
            surface_map, hub_pos[0], hub_sample, hub_surf_id, err);
        if (surf_st != Status::ok) {
            return surf_st;
        }
        const Status mu_st = pneumatic_surface_mu<Real, TK>(
            hub_sample, static_cast<Real>(0), mu_out, err);
        if (mu_st != Status::ok) {
            return mu_st;
        }
        first_surface_id = hub_surf_id;
    }

    Real rad_j = static_cast<Real>(0);
    Real max_j = static_cast<Real>(0);
    Real rim_j = static_cast<Real>(0);
    pneumatic_compute_radial_energies<Real, TK>(
        spec->tyre, &spec->ring, next_state, scratch->delta_m,
        scratch->energy_scratch, rad_j, max_j, rim_j);

    Real effective_radius_out = static_cast<Real>(0);
    if (emitted_patches > 0) {
        const bool any_loaded = total_load_n > static_cast<Real>(0);
        Real terms = static_cast<Real>(0);
        Real weights = static_cast<Real>(0);
        for (i32 p = 0; p < emitted_patches; ++p) {
            const Real w_p =
                any_loaded ? scratch->patch_loads[p] : static_cast<Real>(1);
            const i32 p3 = 3 * p;
            const Real offset[3] = {
                scratch->patch_centroids[p3 + 0] - hub_pos[0],
                scratch->patch_centroids[p3 + 1] - hub_pos[1],
                scratch->patch_centroids[p3 + 2] - hub_pos[2]
            };
            const Real drop = -TK::dot3(offset, scratch->patch_normals + p3);
            terms += w_p * drop;
            weights += w_p;
        }
        effective_radius_out =
            TK::max(static_cast<Real>(0), TK::div(terms, weights));
    }

    out.force_world_n[0] = total_force_world_n[0];
    out.force_world_n[1] = total_force_world_n[1];
    out.force_world_n[2] = total_force_world_n[2];
    out.normal_load_n = total_load_n;
    out.support_n = total_support_n;
    out.tangent_force_n = total_tangent_n;
    out.mean_deflection_m = mean_deflection_out;
    out.contact_length_m = total_contact_length_m;
    out.effective_radius_m = effective_radius_out;
    out.slip_ratio = slip_ratio_out;
    out.transient_slip_ratio = transient_slip_out;
    out.slip_mps = slip_mps_out;
    out.friction_coefficient = mu_out;
    out.rolling_crr = rolling_crr;
    out.radial_energy_j = rad_j;
    out.maxwell_energy_j = max_j;
    out.rim_energy_j = rim_j;
    out.shear_energy_j = total_shear_energy_j;
    out.carcass_dissipated_power_w = carcass_loss_w;
    out.dissipated_power_w = carcass_loss_w + brush_loss_w;
    out.carcass_loss_j = carcass_loss_w * dt_s;
    out.brush_and_rolling_loss_j = brush_loss_w * dt_s;
    out.rim_event = scratch->carcass.rim_event;
    out.patch_count = emitted_patches;
    out.dominant_surface_id = first_surface_id;
    out.fully_sliding = any_fully_sliding;
    out.airborne = 0;
    out.coverage_event = coverage_event;
    out.rim_strike_active = scratch->carcass.rim_strike_active;
    err = ErrCode::none;
    return Status::ok;
}

template <
    class Real,
    class TK,
    class SpecPtr,
    class MapPtr,
    class PtrPx,
    class PtrPz,
    class XposPtr,
    class XmatPtr,
    class XanchorPtr,
    class XaxisPtr,
    class QvelPtr,
    class StateInPtr,
    class StateOutPtr,
    class QfrcPtr,
    class ScratchPtr>
BIKE_HD Status step_pneumatic_wheel(
    SpecPtr spec,
    MapPtr surface_map,
    Real dt_s,
    PtrPx road_px,
    PtrPz road_pz,
    i32 road_num_vertices,
    Real road_x0_m,
    Real road_dx_m,
    bool road_is_uniform_grid,
    XposPtr body_xpos,
    XmatPtr body_xmat,
    XanchorPtr jnt_xanchor,
    XaxisPtr jnt_xaxis,
    QvelPtr qvel,
    StateInPtr prev_state,
    StateOutPtr next_state,
    QfrcPtr out_qfrc,
    BIKE_REF(GpuPneumaticWheelOutput<Real>) out_summary,
    ScratchPtr scratch,
    BIKE_REF(ErrCode) err
) noexcept {
    out_summary = GpuPneumaticWheelOutput<Real>{};
    if (spec == nullptr || body_xpos == nullptr || body_xmat == nullptr ||
        jnt_xanchor == nullptr || jnt_xaxis == nullptr || qvel == nullptr ||
        prev_state == nullptr || next_state == nullptr ||
        out_qfrc == nullptr || scratch == nullptr) {
        err = ErrCode::out_of_bounds_chain;
        return Status::invalid_argument;
    }
    const i32 nv = spec->nv;
    const i32 njnt = spec->njnt;
    const i32 body = spec->wheel_body_id;
    if (nv <= 0 || nv > kMaxNv || njnt <= 0 || njnt > kMaxNjnt || body <= 0) {
        err = ErrCode::out_of_bounds_chain;
        return Status::out_of_bounds;
    }
    if (!TK::isfinite(dt_s) || dt_s <= static_cast<Real>(0)) {
        err = ErrCode::bad_dt;
        return Status::invalid_argument;
    }

    const i32 b3 = 3 * body;
    const Real hub_pos[3] = {
        body_xpos[b3 + 0],
        body_xpos[b3 + 1],
        body_xpos[b3 + 2]
    };
    const GpuWheelChain local_chain = spec->chain;
    Status st = chain_point_jacobian<Real, TK, true>(
        local_chain, body_xpos, jnt_xanchor, jnt_xaxis,
        body, nv, njnt, hub_pos, scratch->jp, scratch->jr, err);
    if (st != Status::ok) {
        return st;
    }
    Real hub_vel[3]{};
    Real omega_vec[3]{};
    TK::gemv_n3(nv, scratch->jp, qvel, hub_vel);
    TK::gemv_n3(nv, scratch->jr, qvel, omega_vec);
    const Real omega_forward = omega_vec[1];

    st = evaluate_pneumatic_wheel<Real, TK>(
        spec, surface_map, hub_pos, hub_vel, omega_forward,
        road_px, road_pz, road_num_vertices, road_x0_m, road_dx_m,
        road_is_uniform_grid, dt_s, prev_state, next_state,
        out_summary, scratch, err);
    if (st != Status::ok) {
        return st;
    }

    BIKE_THREAD Real *null_jr = nullptr;
    for (i32 p = 0; p < out_summary.patch_count; ++p) {
        const i32 p3 = 3 * p;
        const Real centroid[3] = {
            scratch->patch_centroids[p3 + 0],
            scratch->patch_centroids[p3 + 1],
            scratch->patch_centroids[p3 + 2]
        };
        st = chain_point_jacobian<Real, TK, false>(
            local_chain, body_xpos, jnt_xanchor, jnt_xaxis,
            body, nv, njnt, centroid, scratch->jp, null_jr, err);
        if (st != Status::ok) {
            return st;
        }
        const Real f_world[3] = {
            scratch->patch_forces[p3 + 0],
            scratch->patch_forces[p3 + 1],
            scratch->patch_forces[p3 + 2]
        };
        TK::gemv_t3(nv, scratch->jp, f_world, scratch->tmp_nv);
        for (i32 i = 0; i < nv; ++i) {
            out_qfrc[i] += scratch->tmp_nv[i];
        }
    }

    if (out_summary.normal_load_n > static_cast<Real>(0) &&
        out_summary.effective_radius_m > static_cast<Real>(0) &&
        out_summary.rolling_crr > static_cast<Real>(0)) {
        const i32 b9 = 9 * body;
        const Real axis[3] = {
            body_xmat[b9 + 1],
            body_xmat[b9 + 4],
            body_xmat[b9 + 7]
        };
        // scratch->jr still holds the wheel's rotational Jacobian from hub_pos above.
        TK::gemv_t3(nv, scratch->jr, axis, scratch->tmp_nv);
        const Real omega_world = TK::dot(nv, scratch->tmp_nv, qvel);
        const Real moment =
            -out_summary.rolling_crr * out_summary.normal_load_n *
            out_summary.effective_radius_m *
            TK::tanh(TK::div(omega_world, BIKE_REAL(Real, 0.2)));
        const Real raw_power = -moment * omega_world;
        if (!TK::isfinite(axis[0]) || !TK::isfinite(axis[1]) ||
            !TK::isfinite(axis[2]) || !TK::isfinite(omega_world) ||
            !TK::isfinite(moment) || !TK::isfinite(raw_power)) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }
        const Real rolling_power_w = TK::max(static_cast<Real>(0), raw_power);
        for (i32 i = 0; i < nv; ++i) {
            if (!TK::isfinite(scratch->tmp_nv[i])) {
                err = ErrCode::overflow_nonfinite;
                return Status::overflow;
            }
            out_qfrc[i] += moment * scratch->tmp_nv[i];
        }
        out_summary.rolling_moment_nm = moment;
        out_summary.dissipated_power_w += rolling_power_w;
        out_summary.brush_and_rolling_loss_j += rolling_power_w * dt_s;
    }

    for (i32 i = 0; i < nv; ++i) {
        if (!TK::isfinite(out_qfrc[i])) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }
    }
    err = ErrCode::none;
    return Status::ok;
}

template <
    class Real,
    class TK,
    class SpecPtr,
    class PtrPx,
    class PtrPz,
    class StatePtr,
    class ScratchPtr>
BIKE_HD Status pneumatic_stored_energy(
    SpecPtr spec,
    PtrPx road_px,
    PtrPz road_pz,
    i32 road_num_vertices,
    Real road_x0_m,
    Real road_dx_m,
    bool road_is_uniform_grid,
    Real hub_x_m,
    Real hub_z_m,
    StatePtr state,
    Real committed_shear_energy_j,
    BIKE_REF(Real) out_total_energy_j,
    ScratchPtr scratch,
    BIKE_REF(ErrCode) err
) noexcept {
    out_total_energy_j = static_cast<Real>(0);
    if (spec == nullptr || state == nullptr || scratch == nullptr) {
        err = ErrCode::out_of_bounds_station;
        return Status::invalid_argument;
    }
    const Real outer_radius_m =
        spec->tyre.outer_radius_mm * BIKE_REAL(Real, 0.001);
    u8 cov = 0;
    u8 air = 0;
    if (road_is_uniform_grid) {
        const Status st = pneumatic_intersect_uniform_grid<Real, TK>(
            &spec->ring, hub_x_m, hub_z_m, outer_radius_m,
            road_x0_m, road_dx_m, road_num_vertices, road_px, road_pz,
            scratch->r_m, scratch->delta_m, scratch->road_x_m, scratch->road_z_m,
            cov, air, err);
        if (st != Status::ok) {
            return st;
        }
    } else {
        PolylineView<Real> pl{};
        pl.px = road_px;
        pl.pz = road_pz;
        pl.num_vertices = road_num_vertices;
        const Status st = pneumatic_intersect_polyline<Real, TK>(
            &spec->ring, hub_x_m, hub_z_m, outer_radius_m, pl,
            scratch->r_m, scratch->delta_m, scratch->road_x_m, scratch->road_z_m,
            cov, air, err);
        if (st != Status::ok) {
            return st;
        }
    }
    Real rad_j = static_cast<Real>(0);
    Real max_j = static_cast<Real>(0);
    Real rim_j = static_cast<Real>(0);
    pneumatic_compute_radial_energies<Real, TK>(
        spec->tyre, &spec->ring, state, scratch->delta_m,
        scratch->energy_scratch, rad_j, max_j, rim_j);
    out_total_energy_j = rad_j + max_j + rim_j + committed_shear_energy_j;
    err = ErrCode::none;
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
