// native/src/physics_core/energy_ledger.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// strict GPU energy balance ledger matching CPU runtime/accounting.cpp and runtime/step.cpp semantics.
#pragma once

#include "common.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

template <class Real>
struct ElasticEnergyBreakdown {
    Real u_susp_fork = BIKE_REAL(Real, 0.0);
    Real u_susp_shock = BIKE_REAL(Real, 0.0);
    Real u_tyre_radial = BIKE_REAL(Real, 0.0);
    Real u_tyre_shear = BIKE_REAL(Real, 0.0);
    Real u_tyre_maxwell = BIKE_REAL(Real, 0.0);
    Real u_tyre_rim = BIKE_REAL(Real, 0.0);
    Real u_extra = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct MechanicalEnergyBreakdown {
    Real kinetic_j = BIKE_REAL(Real, 0.0);
    Real gravitational_j = BIKE_REAL(Real, 0.0);
    Real elastic_j = BIKE_REAL(Real, 0.0);
    Real total_j = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct GpuStepEnergyInputs {
    Real E_mech = BIKE_REAL(Real, 0.0);
    Real w_muscle_signed_j = BIKE_REAL(Real, 0.0);
    Real w_muscle_pos_j = BIKE_REAL(Real, 0.0);
    Real w_motor_signed_j = BIKE_REAL(Real, 0.0);
    Real w_motor_pos_j = BIKE_REAL(Real, 0.0);
    Real d_tyre_radial_j = BIKE_REAL(Real, 0.0);
    Real d_tyre_brush_roll_j = BIKE_REAL(Real, 0.0);
    Real d_suspension_j = BIKE_REAL(Real, 0.0);
    Real d_resistance_j = BIKE_REAL(Real, 0.0);
    Real d_brake_hold_j = BIKE_REAL(Real, 0.0);
    Real d_brake_friction_j = BIKE_REAL(Real, 0.0);
    Real w_closure_signed_j = BIKE_REAL(Real, 0.0);
    Real w_closure_abs_j = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct GpuEnergyLedger {
    Real E_mech_initial = BIKE_REAL(Real, 0.0);
    Real E_mech = BIKE_REAL(Real, 0.0);
    Real E_mech_peak = BIKE_REAL(Real, 0.0);
    Real E_scale_min = BIKE_REAL(Real, 1.0);

    Real W_muscle_signed = BIKE_REAL(Real, 0.0);
    Real W_muscle_pos = BIKE_REAL(Real, 0.0);
    Real W_motor_signed = BIKE_REAL(Real, 0.0);
    Real W_motor_pos = BIKE_REAL(Real, 0.0);

    Real D_tyre_radial = BIKE_REAL(Real, 0.0);
    Real D_tyre_brush_roll = BIKE_REAL(Real, 0.0);
    Real D_suspension = BIKE_REAL(Real, 0.0);
    Real D_resistance = BIKE_REAL(Real, 0.0);
    Real D_brake = BIKE_REAL(Real, 0.0);

    Real W_closure_signed = BIKE_REAL(Real, 0.0);
    Real W_closure_abs = BIKE_REAL(Real, 0.0);

    Real delta_E_mech = BIKE_REAL(Real, 0.0);
    Real net_supply = BIKE_REAL(Real, 0.0);
    Real residual_j = BIKE_REAL(Real, 0.0);
    Real energy_scale_j = BIKE_REAL(Real, 1.0);
    Real rel_residual = BIKE_REAL(Real, 0.0);
    Real max_rel_residual = BIKE_REAL(Real, 0.0);

    i32 step_count = 0;
    u8 initialized = 0;
    u8 pad[3]{};
};

template <class Real, class TK, class PtrRowAdr, class PtrRowNnz,
          class PtrColInd, class PtrM, class PtrQvel>
BIKE_HD Status compute_kinetic_energy_csr(
    i32 nv,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM M,
    PtrQvel qvel,
    BIKE_THREAD Real &out_kinetic_j,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_kinetic_j = BIKE_REAL(Real, 0.0);
    if (nv < 0) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (nv == 0) {
        return Status::ok;
    }
    if (M_rowadr == nullptr || M_rownnz == nullptr || M_colind == nullptr ||
        M == nullptr || qvel == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }

    // MuJoCo / mujoco_warp store the lower-triangular part (j <= i) of the
    // symmetric mass matrix M in CSR form. Computing 0.5 * q^T M q directly
    // needs zero stack scratch and supports arbitrary nv.
    Real qMq = BIKE_REAL(Real, 0.0);
    for (i32 i = 0; i < nv; ++i) {
        const i32 adr = M_rowadr[i];
        const i32 nnz = M_rownnz[i];
        if (adr < 0 || nnz < 0) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        const Real vi = qvel[i];
        if (!TK::isfinite(vi)) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }
        Real row_diag = BIKE_REAL(Real, 0.0);
        Real row_off = BIKE_REAL(Real, 0.0);
        for (i32 k = 0; k < nnz; ++k) {
            const i32 j = M_colind[adr + k];
            if (j < 0 || j >= nv) {
                err = ErrCode::out_of_bounds_chain;
                return Status::out_of_bounds;
            }
            if (j > i) {
                continue;
            }
            const Real val = M[adr + k];
            if (!TK::isfinite(val)) {
                err = ErrCode::overflow_nonfinite;
                return Status::overflow;
            }
            if (j == i) {
                row_diag += val * vi * vi;
            } else {
                row_off += val * qvel[j];
            }
        }
        qMq += row_diag + BIKE_REAL(Real, 2.0) * vi * row_off;
    }
    const Real kinetic = BIKE_REAL(Real, 0.5) * qMq;
    if (!TK::isfinite(kinetic)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    out_kinetic_j = kinetic;
    return Status::ok;
}

template <class Real, class TK, class PtrMass, class PtrXipos, class PtrGrav>
BIKE_HD Status compute_gravitational_energy(
    i32 nbody,
    PtrMass body_mass,
    PtrXipos xipos_or_com,
    PtrGrav gravity,
    BIKE_THREAD Real &out_gravitational_j,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_gravitational_j = BIKE_REAL(Real, 0.0);
    if (nbody < 0 || gravity == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Real gx = gravity[0];
    const Real gy = gravity[1];
    const Real gz = gravity[2];
    if (!TK::isfinite(gx) || !TK::isfinite(gy) || !TK::isfinite(gz)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (nbody == 0) {
        return Status::ok;
    }
    if (body_mass == nullptr || xipos_or_com == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    Real grav_dot_sum = BIKE_REAL(Real, 0.0);
    for (i32 b = 0; b < nbody; ++b) {
        const Real m_b = body_mass[b];
        const Real px = xipos_or_com[3 * b + 0];
        const Real py = xipos_or_com[3 * b + 1];
        const Real pz = xipos_or_com[3 * b + 2];
        if (!TK::isfinite(m_b) || m_b < BIKE_REAL(Real, 0.0) ||
            !TK::isfinite(px) || !TK::isfinite(py) || !TK::isfinite(pz)) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        grav_dot_sum += m_b * (gx * px + gy * py + gz * pz);
    }
    const Real gravitational = -grav_dot_sum;
    if (!TK::isfinite(gravitational)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    out_gravitational_j = gravitational;
    return Status::ok;
}

template <class Real, class TK, class PtrRowAdr, class PtrRowNnz,
          class PtrColInd, class PtrM, class PtrQvel,
          class PtrMass, class PtrXipos, class PtrGrav>
BIKE_HD Status compute_mechanical_energy(
    i32 nv,
    i32 nbody,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM M,
    PtrQvel qvel,
    PtrMass body_mass,
    PtrXipos xipos_or_com,
    PtrGrav gravity,
    BIKE_CREF(ElasticEnergyBreakdown<Real>) elastic,
    BIKE_THREAD MechanicalEnergyBreakdown<Real> &out,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out = MechanicalEnergyBreakdown<Real>{};
    if (!TK::isfinite(elastic.u_susp_fork) ||
        !TK::isfinite(elastic.u_susp_shock) ||
        !TK::isfinite(elastic.u_tyre_radial) ||
        !TK::isfinite(elastic.u_tyre_shear) ||
        !TK::isfinite(elastic.u_tyre_maxwell) ||
        !TK::isfinite(elastic.u_tyre_rim) ||
        !TK::isfinite(elastic.u_extra)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    Real kinetic = BIKE_REAL(Real, 0.0);
    const Status ke_st = compute_kinetic_energy_csr<Real, TK>(
        nv, M_rowadr, M_rownnz, M_colind, M, qvel, kinetic, err);
    if (ke_st != Status::ok) {
        return ke_st;
    }
    Real gravitational = BIKE_REAL(Real, 0.0);
    const Status pe_st = compute_gravitational_energy<Real, TK>(
        nbody, body_mass, xipos_or_com, gravity, gravitational, err);
    if (pe_st != Status::ok) {
        return pe_st;
    }
    const Real elastic_sum =
        elastic.u_susp_fork + elastic.u_susp_shock +
        elastic.u_tyre_radial + elastic.u_tyre_shear +
        elastic.u_tyre_maxwell + elastic.u_tyre_rim +
        elastic.u_extra;
    const Real total = kinetic + gravitational + elastic_sum;
    if (!TK::isfinite(elastic_sum) || !TK::isfinite(total)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    out.kinetic_j = kinetic;
    out.gravitational_j = gravitational;
    out.elastic_j = elastic_sum;
    out.total_j = total;
    return Status::ok;
}

template <class Real, class TK, class PtrRowAdr, class PtrRowNnz,
          class PtrColInd, class PtrM, class PtrQvel,
          class PtrMass, class PtrXipos, class PtrGrav>
BIKE_HD Status compute_mechanical_energy(
    i32 nv,
    i32 nbody,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM M,
    PtrQvel qvel,
    PtrMass body_mass,
    PtrXipos xipos_or_com,
    PtrGrav gravity,
    Real u_susp_fork,
    Real u_susp_shock,
    Real u_tyre_radial,
    Real u_tyre_shear,
    Real u_tyre_maxwell,
    Real u_tyre_rim,
    BIKE_THREAD Real &out_total_j,
    BIKE_THREAD ErrCode &err
) noexcept {
    out_total_j = BIKE_REAL(Real, 0.0);
    ElasticEnergyBreakdown<Real> elastic{};
    elastic.u_susp_fork = u_susp_fork;
    elastic.u_susp_shock = u_susp_shock;
    elastic.u_tyre_radial = u_tyre_radial;
    elastic.u_tyre_shear = u_tyre_shear;
    elastic.u_tyre_maxwell = u_tyre_maxwell;
    elastic.u_tyre_rim = u_tyre_rim;
    MechanicalEnergyBreakdown<Real> breakdown{};
    const Status st = compute_mechanical_energy<Real, TK>(
        nv, nbody, M_rowadr, M_rownnz, M_colind, M, qvel,
        body_mass, xipos_or_com, gravity, elastic, breakdown, err);
    if (st != Status::ok) {
        return st;
    }
    out_total_j = breakdown.total_j;
    return Status::ok;
}

template <class Real, class TK>
BIKE_HD Status energy_brake_dissipation_step(
    Real tau_hold_front,
    Real tau_hold_rear,
    Real tau_fric_front,
    Real tau_fric_rear,
    Real omega_front,
    Real omega_rear,
    Real dt_s,
    BIKE_THREAD Real &out_hold_j,
    BIKE_THREAD Real &out_fric_j,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_hold_j = BIKE_REAL(Real, 0.0);
    out_fric_j = BIKE_REAL(Real, 0.0);
    if (!TK::isfinite(tau_hold_front) || !TK::isfinite(tau_hold_rear) ||
        !TK::isfinite(tau_fric_front) || !TK::isfinite(tau_fric_rear) ||
        !TK::isfinite(omega_front) || !TK::isfinite(omega_rear)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(dt_s) || dt_s <= BIKE_REAL(Real, 0.0)) {
        err = ErrCode::bad_dt;
        return Status::invalid_argument;
    }
    const Real hold_f =
        TK::max(BIKE_REAL(Real, 0.0), -(tau_hold_front * omega_front)) * dt_s;
    const Real hold_r =
        TK::max(BIKE_REAL(Real, 0.0), -(tau_hold_rear * omega_rear)) * dt_s;
    const Real fric_f =
        TK::max(BIKE_REAL(Real, 0.0), -(tau_fric_front * omega_front)) * dt_s;
    const Real fric_r =
        TK::max(BIKE_REAL(Real, 0.0), -(tau_fric_rear * omega_rear)) * dt_s;
    const Real hold_sum = hold_f + hold_r;
    const Real fric_sum = fric_f + fric_r;
    if (!TK::isfinite(hold_sum) || !TK::isfinite(fric_sum)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    out_hold_j = hold_sum;
    out_fric_j = fric_sum;
    return Status::ok;
}

template <class Real, class TK, class LedgerPtr>
BIKE_HD Status energy_ledger_init(
    LedgerPtr ledger,
    Real initial_E_mech,
    Real e_scale_min,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (ledger == nullptr || !TK::isfinite(initial_E_mech) ||
        !TK::isfinite(e_scale_min) || e_scale_min <= BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    GpuEnergyLedger<Real> fresh{};
    fresh.E_mech_initial = initial_E_mech;
    fresh.E_mech = initial_E_mech;
    fresh.E_mech_peak = TK::abs(initial_E_mech);
    fresh.E_scale_min = e_scale_min;
    fresh.energy_scale_j = TK::max(e_scale_min, TK::abs(initial_E_mech));
    fresh.initialized = 1;
    *ledger = fresh;
    return Status::ok;
}

template <class Real, class TK, class LedgerPtr>
BIKE_HD Status energy_ledger_init(
    LedgerPtr ledger,
    Real initial_E_mech,
    BIKE_THREAD ErrCode &err
) noexcept {
    return energy_ledger_init<Real, TK>(
        ledger, initial_E_mech, BIKE_REAL(Real, 1.0), err);
}

template <class Real, class TK, class LedgerPtr>
BIKE_HD Status energy_ledger_step(
    LedgerPtr ledger,
    BIKE_CREF(GpuStepEnergyInputs<Real>) in,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (ledger == nullptr || ledger->initialized == 0) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(in.E_mech) ||
        !TK::isfinite(in.w_muscle_signed_j) ||
        !TK::isfinite(in.w_muscle_pos_j) ||
        !TK::isfinite(in.w_motor_signed_j) ||
        !TK::isfinite(in.w_motor_pos_j) ||
        !TK::isfinite(in.d_tyre_radial_j) ||
        !TK::isfinite(in.d_tyre_brush_roll_j) ||
        !TK::isfinite(in.d_suspension_j) ||
        !TK::isfinite(in.d_resistance_j) ||
        !TK::isfinite(in.d_brake_hold_j) ||
        !TK::isfinite(in.d_brake_friction_j) ||
        !TK::isfinite(in.w_closure_signed_j) ||
        !TK::isfinite(in.w_closure_abs_j)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (in.w_muscle_pos_j < BIKE_REAL(Real, 0.0) ||
        in.w_motor_pos_j < BIKE_REAL(Real, 0.0) ||
        in.d_tyre_radial_j < BIKE_REAL(Real, 0.0) ||
        in.d_tyre_brush_roll_j < BIKE_REAL(Real, 0.0) ||
        in.d_suspension_j < BIKE_REAL(Real, 0.0) ||
        in.d_resistance_j < BIKE_REAL(Real, 0.0) ||
        in.d_brake_hold_j < BIKE_REAL(Real, 0.0) ||
        in.d_brake_friction_j < BIKE_REAL(Real, 0.0) ||
        in.w_closure_abs_j < BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }

    GpuEnergyLedger<Real> next = *ledger;
    next.W_muscle_signed += in.w_muscle_signed_j;
    next.W_muscle_pos += in.w_muscle_pos_j;
    next.W_motor_signed += in.w_motor_signed_j;
    next.W_motor_pos += in.w_motor_pos_j;

    next.D_tyre_radial += in.d_tyre_radial_j;
    next.D_tyre_brush_roll += in.d_tyre_brush_roll_j;
    next.D_suspension += in.d_suspension_j;
    next.D_resistance += in.d_resistance_j;
    next.D_brake += in.d_brake_hold_j + in.d_brake_friction_j;

    next.W_closure_signed += in.w_closure_signed_j;
    next.W_closure_abs += in.w_closure_abs_j;

    next.E_mech = in.E_mech;
    next.E_mech_peak = TK::max(next.E_mech_peak, TK::abs(in.E_mech));
    next.delta_E_mech = next.E_mech - next.E_mech_initial;

    const Real total_dissipation =
        next.D_tyre_radial + next.D_tyre_brush_roll + next.D_suspension +
        next.D_resistance + next.D_brake;
    next.net_supply =
        (next.W_muscle_signed + next.W_motor_signed) - total_dissipation;
    next.residual_j = next.delta_E_mech - next.net_supply;

    const Real total_source_pos = next.W_muscle_pos + next.W_motor_pos;
    Real scale = TK::max(next.E_scale_min, TK::abs(next.E_mech_initial));
    scale = TK::max(scale, next.E_mech_peak);
    scale = TK::max(scale, total_source_pos);
    scale = TK::max(scale, total_dissipation);
    if (!TK::isfinite(scale) || scale <= BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(next.residual_j)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    next.energy_scale_j = scale;
    next.rel_residual = TK::abs(next.residual_j) / scale;
    next.max_rel_residual = TK::max(next.max_rel_residual, next.rel_residual);
    if (!TK::isfinite(next.rel_residual) || !TK::isfinite(next.max_rel_residual)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    ++next.step_count;
    *ledger = next;
    return Status::ok;
}

template <class Real, class TK>
BIKE_HD bool energy_closure_work_ok(
    Real w_closure_abs,
    Real source_positive_j,
    Real roundoff_j
) noexcept {
    if (!TK::isfinite(w_closure_abs) || !TK::isfinite(source_positive_j) ||
        !TK::isfinite(roundoff_j) ||
        w_closure_abs < BIKE_REAL(Real, 0.0) ||
        source_positive_j < BIKE_REAL(Real, 0.0) ||
        roundoff_j < BIKE_REAL(Real, 0.0)) {
        return false;
    }
    return w_closure_abs <=
           TK::max(roundoff_j, BIKE_REAL(Real, 0.01) * source_positive_j);
}

template <class Real, class TK, class LedgerPtr>
BIKE_HD bool energy_ledger_contract_ok(
    LedgerPtr ledger,
    Real max_rel_residual_tol,
    Real closure_roundoff_j
) noexcept {
    if (ledger == nullptr || ledger->initialized == 0 ||
        !TK::isfinite(max_rel_residual_tol) ||
        max_rel_residual_tol < BIKE_REAL(Real, 0.0)) {
        return false;
    }
    if (!TK::isfinite(ledger->max_rel_residual) ||
        ledger->max_rel_residual > max_rel_residual_tol) {
        return false;
    }
    return energy_closure_work_ok<Real, TK>(
        ledger->W_closure_abs,
        ledger->W_muscle_pos + ledger->W_motor_pos,
        closure_roundoff_j);
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
