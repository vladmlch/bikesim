// native/src/physics_core/tyre_compliant.hpp — Tri-target Slice 1 (compliant_2d)
// wheel step kernel parameterized over TerrainFn, JacFn, QfrcAccumFn, and Io
// so CPU FP64 preserves 0-bit parity while GPU FP32 compiles allocation-free.
#pragma once

#include "common.hpp"
#include "tyre_contact.hpp"
#include "tyre_tabulated.hpp"
#include "tyre_types.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

template <class Real>
struct RadialEvalInput {
    Real delta = BIKE_REAL(Real, 0.0);
    Real delta_dot = BIKE_REAL(Real, 0.0);
    Real eta = BIKE_REAL(Real, 0.0);
    Real eps = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct RadialEvalOutput {
    Real normal = BIKE_REAL(Real, 0.0);
    Real energy = BIKE_REAL(Real, 0.0);
    Real elastic = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct CompliantJacInput {
    TyreVec3<Real> point{};
    TyreVec3<Real> center{};
};

template <class Real>
struct CompliantJacOutput {
    TyreVec3<Real> velocity{};
    TyreVec3<Real> center_velocity{};
};

template <class Real>
struct CompliantRecord {
    TyreVec3<Real> center{};
    TyreVec3<Real> point{};
    TyreVec3<Real> normal{};
    PolylineContactResult<Real> contact{};
    Real delta_dot = BIKE_REAL(Real, 0.0);
    Real slip = BIKE_REAL(Real, 0.0);
    Real normal_load = BIKE_REAL(Real, 0.0);
    Real radial_energy = BIKE_REAL(Real, 0.0);
    Real elastic_force = BIKE_REAL(Real, 0.0);
    Real tangent_force = BIKE_REAL(Real, 0.0);
    Real xi_new = BIKE_REAL(Real, 0.0);
    Real mu = BIKE_REAL(Real, 0.0);
    i32 section_idx = -1;
    Real release_loss = BIKE_REAL(Real, 0.0);
    Real brush_loss = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct CompliantWheelParams {
    Real radial_k_n_m = BIKE_REAL(Real, 0.0);
    Real loss_factor = BIKE_REAL(Real, 0.0);
    Real hysteresis_rate_eps_mps = BIKE_REAL(Real, 0.0);
    TabulatedRadialParams<Real> radial_table{};
    u8 use_tabulated = 0;
};

template <class Real, class TK, class ParamsPtr>
BIKE_HD Status eval_compliant_radial(
    ParamsPtr params,
    Real delta,
    Real delta_dot,
    BIKE_REF(RadialEvalOutput<Real>) out,
    BIKE_REF(ErrCode) err
) noexcept {
    out = RadialEvalOutput<Real>{};
    if (params == nullptr) {
        err = ErrCode::missing_material;
        return Status::invalid_argument;
    }
    if (params->use_tabulated != 0) {
        Real force = BIKE_REAL(Real, 0.0);
        Real energy = BIKE_REAL(Real, 0.0);
        Status st = tabulated_radial_contact<Real, TK>(
            &params->radial_table, delta, delta_dot,
            params->loss_factor, params->hysteresis_rate_eps_mps,
            force, energy, err);
        if (st != Status::ok) {
            return st;
        }
        Real elastic = BIKE_REAL(Real, 0.0);
        Real elastic_energy = BIKE_REAL(Real, 0.0);
        st = tabulated_radial_elastic<Real, TK>(
            &params->radial_table, delta, elastic, elastic_energy, err);
        if (st != Status::ok) {
            return st;
        }
        out.normal = force;
        out.energy = energy;
        out.elastic = elastic;
        err = ErrCode::none;
        return Status::ok;
    }
    Real force = BIKE_REAL(Real, 0.0);
    Real energy = BIKE_REAL(Real, 0.0);
    const Status st = normal_contact_hysteresis<Real, TK>(
        delta, delta_dot, params->radial_k_n_m, params->loss_factor,
        params->hysteresis_rate_eps_mps, force, energy, err);
    if (st != Status::ok) {
        return st;
    }
    out.normal = force;
    out.energy = energy;
    out.elastic = params->radial_k_n_m * TK::max(delta, BIKE_REAL(Real, 0.0));
    err = ErrCode::none;
    return Status::ok;
}

template <class Real, class TK, class TablePtr>
struct TabulatedRadialIo {
    TablePtr tables = nullptr;
    u8 enabled[2]{1, 1};

    BIKE_HD bool eval_radial(
        i32 side,
        BIKE_CREF(RadialEvalInput<Real>) in,
        BIKE_REF(RadialEvalOutput<Real>) out,
        BIKE_REF(Status) out_status,
        BIKE_REF(ErrCode) err
    ) const noexcept {
        if (tables == nullptr || side < 0 || side > 1 || enabled[side] == 0) {
            return false;
        }
        TablePtr tbl = tables + side;
        Real force = BIKE_REAL(Real, 0.0);
        Real energy = BIKE_REAL(Real, 0.0);
        Status st = tabulated_radial_contact<Real, TK>(
            tbl, in.delta, in.delta_dot, in.eta, in.eps, force, energy, err);
        if (st != Status::ok) {
            out_status = st;
            return true;
        }
        Real elastic = BIKE_REAL(Real, 0.0);
        Real elastic_energy = BIKE_REAL(Real, 0.0);
        st = tabulated_radial_elastic<Real, TK>(
            tbl, in.delta, elastic, elastic_energy, err);
        if (st != Status::ok) {
            out_status = st;
            return true;
        }
        out.normal = force;
        out.energy = energy;
        out.elastic = elastic;
        out_status = Status::ok;
        err = ErrCode::none;
        return true;
    }

    BIKE_HD bool custom_friction(
        BIKE_CREF(FrictionQuery<Real>),
        BIKE_REF(FrictionResult<Real>),
        BIKE_REF(Status),
        BIKE_REF(ErrCode)
    ) const noexcept {
        return false;
    }

    BIKE_HD Status record(
        i32,
        BIKE_CREF(CompliantRecord<Real>),
        BIKE_REF(ErrCode) err
    ) const noexcept {
        err = ErrCode::none;
        return Status::ok;
    }
};

template <class Real, class Io>
struct CompliantIoTraits {
    BIKE_HD static bool eval_radial(
        BIKE_REF(Io) io,
        i32 side,
        BIKE_CREF(RadialEvalInput<Real>) in,
        BIKE_REF(RadialEvalOutput<Real>) out,
        BIKE_REF(Status) out_status,
        BIKE_REF(ErrCode) err
    ) noexcept {
        return io.eval_radial(side, in, out, out_status, err);
    }

    BIKE_HD static bool custom_friction(
        BIKE_REF(Io) io,
        BIKE_CREF(FrictionQuery<Real>) query,
        BIKE_REF(FrictionResult<Real>) out,
        BIKE_REF(Status) out_status,
        BIKE_REF(ErrCode) err
    ) noexcept {
        return io.custom_friction(query, out, out_status, err);
    }

    BIKE_HD static Status record(
        BIKE_REF(Io) io,
        i32 side,
        BIKE_CREF(CompliantRecord<Real>) rec,
        BIKE_REF(ErrCode) err
    ) noexcept {
        return io.record(side, rec, err);
    }
};

template <class Real>
struct CompliantIoTraits<Real, NoOpIo> {
    BIKE_HD static bool eval_radial(
        BIKE_REF(NoOpIo), i32,
        BIKE_CREF(RadialEvalInput<Real>),
        BIKE_REF(RadialEvalOutput<Real>),
        BIKE_REF(Status), BIKE_REF(ErrCode)
    ) noexcept {
        return false;
    }

    BIKE_HD static bool custom_friction(
        BIKE_REF(NoOpIo),
        BIKE_CREF(FrictionQuery<Real>),
        BIKE_REF(FrictionResult<Real>),
        BIKE_REF(Status), BIKE_REF(ErrCode)
    ) noexcept {
        return false;
    }

    BIKE_HD static Status record(
        BIKE_REF(NoOpIo), i32,
        BIKE_CREF(CompliantRecord<Real>),
        BIKE_REF(ErrCode)
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
    BIKE_REF(TerrainFn) terrain_fn,
    BIKE_REF(JacFn) jac_fn,
    BIKE_REF(QfrcAccumFn) qfrc_accum_fn,
    BIKE_REF(Io) io,
    BIKE_REF(ErrCode) err
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

    const TyreVec3<Real> p_vec{p[0], p[1], p[2]};
    const TyreVec3<Real> center_vec{center[0], center[1], center[2]};
    const CompliantJacInput<Real> jac_in{p_vec, center_vec};
    CompliantJacOutput<Real> jac_out{};
    st = jac_fn(side, jac_in, qvel, jac_out, local_err);
    if (st != Status::ok) {
        err = local_err;
        return st;
    }
    const Real velocity[3] = {
        jac_out.velocity.x, jac_out.velocity.y, jac_out.velocity.z};
    const Real center_velocity[3] = {
        jac_out.center_velocity.x,
        jac_out.center_velocity.y,
        jac_out.center_velocity.z};

    const Real delta_dot = -TK::dot3(velocity, n);
    const Real slip = TK::dot3(velocity, tangent);

    Real normal = BIKE_REAL(Real, 0.0);
    Real radial_energy = BIKE_REAL(Real, 0.0);
    Real elastic_force = BIKE_REAL(Real, 0.0);
    Status io_status = Status::ok;
    if (has_tabulated_radial != 0) {
        const RadialEvalInput<Real> rad_in{contact.delta, delta_dot, eta, eps};
        RadialEvalOutput<Real> rad_out{};
        if (!CompliantIoTraits<Real, Io>::eval_radial(
                io, side, rad_in, rad_out, io_status, local_err)) {
            err = ErrCode::missing_material;
            return Status::invalid_argument;
        }
        if (io_status != Status::ok) {
            err = local_err;
            return io_status;
        }
        normal = rad_out.normal;
        radial_energy = rad_out.energy;
        elastic_force = rad_out.elastic;
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
    const FrictionQuery<Real> fquery{tire_mu, p[0], slip};
    FrictionResult<Real> fresult{};
    if (!CompliantIoTraits<Real, Io>::custom_friction(
            io, fquery, fresult, fric_status, local_err)) {
        st = effective_friction<Real, TK>(
            tire_mu, &model->surface_map, p[0], slip, mu, section_idx, local_err);
        if (st != Status::ok) {
            err = local_err;
            return st;
        }
    } else if (fric_status != Status::ok) {
        err = local_err;
        return fric_status;
    } else {
        mu = fresult.mu;
        section_idx = fresult.section_idx;
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

    const TyreVec3<Real> force_world_vec{
        normal * n[0] + force * tangent[0],
        normal * n[1] + force * tangent[1],
        normal * n[2] + force * tangent[2]
    };
    st = qfrc_accum_fn(side, p_vec, force_world_vec, out_qfrc, local_err);
    if (st != Status::ok) {
        err = local_err;
        return st;
    }

    const CompliantRecord<Real> rec{
        center_vec,
        p_vec,
        TyreVec3<Real>{n[0], n[1], n[2]},
        contact,
        delta_dot,
        slip,
        normal,
        radial_energy,
        elastic_force,
        force,
        xi_new,
        mu,
        section_idx,
        release_loss,
        brush_loss
    };
    st = CompliantIoTraits<Real, Io>::record(io, side, rec, local_err);
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
