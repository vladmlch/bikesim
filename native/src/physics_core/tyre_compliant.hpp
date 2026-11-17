// native/src/physics_core/tyre_compliant.hpp — Tri-target Slice 1 (compliant_2d)
// wheel step kernel parameterized over TerrainFn, JacFn, QfrcAccumFn, and Io
// so CPU FP64 preserves 0-bit parity while GPU FP32 compiles allocation-free.
#pragma once

#include "common.hpp"
#include "tyre_contact.hpp"
#include "tyre_types.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

template <class Real, class Io>
struct CompliantIoTraits {
    BIKE_HD static bool eval_radial(
        BIKE_THREAD Io &io,
        i32 side,
        Real delta,
        Real delta_dot,
        Real eta,
        Real eps,
        BIKE_THREAD Real &out_normal,
        BIKE_THREAD Real &out_energy,
        BIKE_THREAD Real &out_elastic,
        BIKE_THREAD Status &out_status,
        BIKE_THREAD ErrCode &err
    ) noexcept {
        return io.eval_radial(
            side, delta, delta_dot, eta, eps,
            out_normal, out_energy, out_elastic, out_status, err);
    }

    BIKE_HD static bool custom_friction(
        BIKE_THREAD Io &io,
        Real tire_mu,
        Real contact_x,
        Real slip,
        BIKE_THREAD Real &out_mu,
        BIKE_THREAD i32 &out_section_idx,
        BIKE_THREAD Status &out_status,
        BIKE_THREAD ErrCode &err
    ) noexcept {
        return io.custom_friction(
            tire_mu, contact_x, slip, out_mu, out_section_idx,
            out_status, err);
    }

    BIKE_HD static Status record(
        BIKE_THREAD Io &io,
        i32 side,
        BIKE_THREAD const Real (&center)[3],
        BIKE_THREAD const Real (&p)[3],
        BIKE_THREAD const Real (&n)[3],
        BIKE_THREAD const PolylineContactResult<Real> &contact,
        Real delta_dot,
        Real slip,
        Real normal,
        Real radial_energy,
        Real elastic_force,
        Real force,
        Real xi_new,
        Real mu,
        i32 section_idx,
        Real release_loss,
        Real brush_loss,
        BIKE_THREAD ErrCode &err
    ) noexcept {
        return io.record(
            side, center, p, n, contact, delta_dot, slip, normal,
            radial_energy, elastic_force, force, xi_new, mu,
            section_idx, release_loss, brush_loss, err);
    }
};

template <class Real>
struct CompliantIoTraits<Real, NoOpIo> {
    BIKE_HD static bool eval_radial(
        BIKE_THREAD NoOpIo &, i32, Real, Real, Real, Real,
        BIKE_THREAD Real &, BIKE_THREAD Real &, BIKE_THREAD Real &,
        BIKE_THREAD Status &, BIKE_THREAD ErrCode &
    ) noexcept {
        return false;
    }

    BIKE_HD static bool custom_friction(
        BIKE_THREAD NoOpIo &, Real, Real, Real,
        BIKE_THREAD Real &, BIKE_THREAD i32 &,
        BIKE_THREAD Status &, BIKE_THREAD ErrCode &
    ) noexcept {
        return false;
    }

    BIKE_HD static Status record(
        BIKE_THREAD NoOpIo &, i32,
        BIKE_THREAD const Real (&)[3],
        BIKE_THREAD const Real (&)[3],
        BIKE_THREAD const Real (&)[3],
        BIKE_THREAD const PolylineContactResult<Real> &,
        Real, Real, Real, Real, Real, Real, Real, Real,
        i32, Real, Real, BIKE_THREAD ErrCode &
    ) noexcept {
        return Status::ok;
    }
};

template <
    class Real,
    class TK,
    class ModelPtr,
    class CenterPtr,
    class QvelPtr,
    class StateInPtr,
    class StateOutPtr,
    class QfrcPtr,
    class TerrainFn,
    class JacFn,
    class QfrcAccumFn,
    class Io = NoOpIo>
BIKE_HD Status step_compliant_wheel(
    ModelPtr model,
    i32 side,
    Real dt,
    CenterPtr center_in,
    QvelPtr qvel,
    StateInPtr prev_state,
    StateOutPtr next_state,
    QfrcPtr out_qfrc,
    TerrainFn &terrain_fn,
    JacFn &jac_fn,
    QfrcAccumFn &qfrc_accum_fn,
    BIKE_THREAD Io &io,
    ErrCode &err
) noexcept {
    const Real radius = model->wheels[side].radius_m;
    const Real tk = model->wheels[side].tangent_k_n_m;
    const Real tire_mu = model->wheels[side].tire_mu;
    const Real relax_len = model->wheels[side].relaxation_length_m;
    const Real rk = model->wheels[side].compliant.radial_k_n_m;
    const Real eta = model->wheels[side].compliant.loss_factor;
    const Real eps = model->wheels[side].compliant.hysteresis_rate_eps_mps;
    const u8 has_tabulated_radial =
        model->wheels[side].compliant.has_tabulated_radial;
    const Real normal_cosine = model->distinct_normal_cosine;

    const Real center[3] = {center_in[0], center_in[1], center_in[2]};
    ErrCode local_err = ErrCode::none;
    PolylineContactResult<Real> contact{};
    Status st = terrain_fn(
        center[0], center[2], radius,
        prev_state->has_segment != 0, prev_state->segment,
        contact, local_err);
    if (st != Status::ok) {
        err = local_err;
        return st;
    }

    const Real p[3] = {contact.point[0], BIKE_REAL(Real, 0.0), contact.point[1]};
    const Real n[3] = {contact.normal[0], BIKE_REAL(Real, 0.0), contact.normal[1]};
    const Real tangent[3] = {n[2], BIKE_REAL(Real, 0.0), -n[0]};

    Real velocity[3]{};
    Real center_velocity[3]{};
    st = jac_fn(side, p, center, qvel, velocity, center_velocity, local_err);
    if (st != Status::ok) {
        err = local_err;
        return st;
    }

    const Real delta_dot = -TK::dot3(velocity, n);
    const Real slip = TK::dot3(velocity, tangent);

    Real normal = BIKE_REAL(Real, 0.0);
    Real radial_energy = BIKE_REAL(Real, 0.0);
    Real elastic_force = BIKE_REAL(Real, 0.0);
    Status io_status = Status::ok;
    if (has_tabulated_radial != 0) {
        if (!CompliantIoTraits<Real, Io>::eval_radial(
                io, side, contact.delta, delta_dot, eta, eps,
                normal, radial_energy, elastic_force, io_status, local_err)) {
            err = ErrCode::missing_material;
            return Status::invalid_argument;
        }
        if (io_status != Status::ok) {
            err = local_err;
            return io_status;
        }
    } else {
        st = normal_contact_hysteresis<Real, TK>(
            contact.delta, delta_dot, rk, eta, eps,
            normal, radial_energy, local_err);
        if (st != Status::ok) {
            err = local_err;
            return st;
        }
        elastic_force = rk * TK::max(contact.delta, BIKE_REAL(Real, 0.0));
    }

    Real xi = prev_state->xi;
    Real release_loss = BIKE_REAL(Real, 0.0);
    if (prev_state->has_tangent != 0) {
        if (prev_state->has_segment == 0 || prev_state->has_point == 0) {
            err = ErrCode::invalid_prev_seg;
            return Status::invalid_argument;
        }
        const i32 seg_diff = contact.segment_id - prev_state->segment;
        const bool adjacent = (seg_diff < 0 ? -seg_diff : seg_diff) <= 1;
        const Real dp[3] = {
            p[0] - prev_state->point[0],
            p[1] - prev_state->point[1],
            p[2] - prev_state->point[2]
        };
        const bool coincident = TK::norm3_ddot(dp) < BIKE_REAL(Real, 1e-8);
        Real moved = BIKE_REAL(Real, 0.0);
        if (prev_state->has_center != 0) {
            const Real dc[3] = {
                center[0] - prev_state->center[0],
                center[1] - prev_state->center[1],
                center[2] - prev_state->center[2]
            };
            moved = TK::norm3_ddot(dc);
        }
        const Real old_tangent[3] = {
            prev_state->tangent[0],
            prev_state->tangent[1],
            prev_state->tangent[2]
        };
        const bool similar = TK::dot3(old_tangent, tangent) >= normal_cosine;
        const Real dtang[3] = {
            tangent[0] - old_tangent[0],
            tangent[1] - old_tangent[1],
            tangent[2] - old_tangent[2]
        };
        const bool continuous =
            similar &&
            TK::norm3_ddot(dp) <=
                BIKE_REAL(Real, 2.0) * moved + radius * TK::norm3_ddot(dtang) +
                    BIKE_REAL(Real, 1e-8);
        if (!adjacent && !coincident && !continuous) {
            release_loss = BIKE_REAL(Real, 0.5) * tk * xi * xi;
            xi = BIKE_REAL(Real, 0.0);
        } else {
            Real transported = BIKE_REAL(Real, 0.0);
            transport_bristle<Real, TK>(old_tangent, tangent, xi, transported);
            release_loss =
                BIKE_REAL(Real, 0.5) * tk * (xi * xi - transported * transported);
            xi = transported;
        }
    }

    Real mu = BIKE_REAL(Real, 0.0);
    i32 section_idx = -1;
    Status fric_status = Status::ok;
    if (!CompliantIoTraits<Real, Io>::custom_friction(
            io, tire_mu, p[0], slip, mu, section_idx, fric_status, local_err)) {
        st = effective_friction<Real, TK>(
            tire_mu, &model->surface_map, p[0], slip, mu, section_idx, local_err);
        if (st != Status::ok) {
            err = local_err;
            return st;
        }
    } else if (fric_status != Status::ok) {
        err = local_err;
        return fric_status;
    }

    Real xi_new = BIKE_REAL(Real, 0.0);
    Real force = BIKE_REAL(Real, 0.0);
    Real brush_loss = BIKE_REAL(Real, 0.0);
    const Real v_roll = TK::dot3(center_velocity, tangent);
    st = brush_step<Real, TK>(
        xi, slip, v_roll, normal, tk, mu, relax_len, dt,
        xi_new, force, brush_loss, local_err);
    if (st != Status::ok) {
        err = local_err;
        return st;
    }

    const Real force_world[3] = {
        normal * n[0] + force * tangent[0],
        normal * n[1] + force * tangent[1],
        normal * n[2] + force * tangent[2]
    };
    st = qfrc_accum_fn(side, p, force_world, out_qfrc, local_err);
    if (st != Status::ok) {
        err = local_err;
        return st;
    }

    st = CompliantIoTraits<Real, Io>::record(
        io, side, center, p, n, contact, delta_dot, slip,
        normal, radial_energy, elastic_force, force, xi_new,
        mu, section_idx, release_loss, brush_loss, local_err);
    if (st != Status::ok) {
        err = local_err;
        return st;
    }

    next_state->xi = xi_new;
    for (i32 k = 0; k < 3; ++k) {
        next_state->tangent[k] = tangent[k];
        next_state->point[k] = p[k];
        next_state->center[k] = center[k];
    }
    next_state->segment = contact.segment_id;
    next_state->has_tangent = 1;
    next_state->has_point = 1;
    next_state->has_center = 1;
    next_state->has_segment = 1;
    err = ErrCode::none;
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
