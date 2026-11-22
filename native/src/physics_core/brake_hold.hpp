// native/src/physics_core/brake_hold.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// static brake friction-loss bounds, latched standstill bite, and 2*pi angle re-basing.
#pragma once

#include "common.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

template <class Real>
struct BrakeHoldParams {
    Real torque_nm = BIKE_REAL(Real, 0.0);
    Real stiffness_nm_rad = BIKE_REAL(Real, 0.0);
    Real damping_nms_rad = BIKE_REAL(Real, 0.0);
    Real engage_radps = BIKE_REAL(Real, 0.5);
    Real release_radps = BIKE_REAL(Real, 1.0);
};

template <class Real, class TK>
BIKE_HD Real rebase_2pi_angle(
    BIKE_THREAD Real &theta,
    bool has_latch,
    BIKE_THREAD Real &latch_theta
) noexcept {
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>; exact FP64/FP32 pi literal
    const Real kPi = BIKE_REAL(Real, 3.14159265358979323846);
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>; exact FP64/FP32 2*pi literal
    const Real kTwoPi = BIKE_REAL(Real, 6.28318530717958647692);
    const Real turns = TK::floor((theta + kPi) / kTwoPi);
    if (turns == BIKE_REAL(Real, 0.0)) {
        return BIKE_REAL(Real, 0.0);
    }
    const Real shift = turns * kTwoPi;
    theta -= shift;
    if (has_latch) {
        latch_theta -= shift;
    }
    return shift;
}

template <class Real, class TK, class PtrFrictionLoss>
BIKE_HD Status apply_static_brake(
    Real ceiling_nm,
    Real front_demand,
    Real rear_demand,
    Real front_hold_nm,
    Real rear_hold_nm,
    i32 front_dof,
    i32 rear_dof,
    i32 nv,
    PtrFrictionLoss dof_frictionloss,
    BIKE_THREAD bool &out_changed,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_changed = false;
    if (!TK::isfinite(ceiling_nm) || ceiling_nm < BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(front_demand) || !TK::isfinite(rear_demand) ||
        !TK::isfinite(front_hold_nm) || !TK::isfinite(rear_hold_nm)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (front_dof < 0 || rear_dof < 0 || front_dof == rear_dof ||
        front_dof >= nv || rear_dof >= nv || dof_frictionloss == nullptr) {
        err = ErrCode::out_of_bounds_chain;
        return Status::out_of_bounds;
    }

    const Real front_loss = TK::max(
        BIKE_REAL(Real, 0.0),
        ceiling_nm * TK::clamp(front_demand, BIKE_REAL(Real, 0.0), BIKE_REAL(Real, 1.0)) -
            TK::abs(front_hold_nm));
    const Real rear_loss = TK::max(
        BIKE_REAL(Real, 0.0),
        ceiling_nm * TK::clamp(rear_demand, BIKE_REAL(Real, 0.0), BIKE_REAL(Real, 1.0)) -
            TK::abs(rear_hold_nm));
    out_changed = (dof_frictionloss[front_dof] != front_loss) ||
                  (dof_frictionloss[rear_dof] != rear_loss);
    dof_frictionloss[front_dof] = front_loss;
    dof_frictionloss[rear_dof] = rear_loss;
    return Status::ok;
}

template <class Real, class TK, class PtrFrictionLoss>
BIKE_HD Status apply_static_brake(
    Real ceiling_nm,
    Real front_demand,
    Real rear_demand,
    i32 front_dof,
    i32 rear_dof,
    i32 nv,
    PtrFrictionLoss dof_frictionloss,
    BIKE_THREAD bool &out_changed,
    BIKE_THREAD ErrCode &err
) noexcept {
    return apply_static_brake<Real, TK>(
        ceiling_nm,
        front_demand,
        rear_demand,
        BIKE_REAL(Real, 0.0),
        BIKE_REAL(Real, 0.0),
        front_dof,
        rear_dof,
        nv,
        dof_frictionloss,
        out_changed,
        err);
}

template <class Real, class TK, class SpecPtr>
BIKE_HD Status eval_brake_hold_wheel(
    SpecPtr spec,
    Real ceiling_nm,
    Real demand,
    Real theta,
    Real omega,
    bool advance,
    bool reseat,
    BIKE_THREAD bool &has_latch,
    BIKE_THREAD Real &latch_theta,
    BIKE_THREAD Real &out_torque,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_torque = BIKE_REAL(Real, 0.0);
    if (!TK::isfinite(ceiling_nm) || ceiling_nm < BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(demand) || !TK::isfinite(theta) || !TK::isfinite(omega) ||
        (has_latch && !TK::isfinite(latch_theta))) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }

    const Real u = TK::clamp(demand, BIKE_REAL(Real, 0.0), BIKE_REAL(Real, 1.0));
    const Real cap = TK::min(spec->torque_nm, ceiling_nm) * u;
    bool latched = has_latch;
    Real anchor = latch_theta;
    if (advance) {
        if (!latched) {
            if (u > BIKE_REAL(Real, 0.0) &&
                spec->torque_nm > BIKE_REAL(Real, 0.0) &&
                spec->stiffness_nm_rad > BIKE_REAL(Real, 0.0) &&
                TK::abs(omega) <= spec->engage_radps) {
                latched = true;
                anchor = theta;
                has_latch = true;
                latch_theta = anchor;
            }
        } else if (u <= BIKE_REAL(Real, 0.0) || TK::abs(omega) > spec->release_radps) {
            latched = false;
            has_latch = false;
        } else if (reseat && TK::abs(omega) <= spec->engage_radps &&
                   TK::abs(-spec->stiffness_nm_rad * (theta - anchor) -
                           spec->damping_nms_rad * omega) > cap) {
            anchor = theta;
            latch_theta = anchor;
        }
    }
    if (!latched) {
        return Status::ok;
    }
    out_torque = TK::clamp(
        -spec->stiffness_nm_rad * (theta - anchor) - spec->damping_nms_rad * omega,
        -cap, cap);
    return Status::ok;
}

template <class Real, class TK, class SpecPtr>
BIKE_HD Status eval_brake_hold_wheel(
    SpecPtr spec,
    Real demand,
    Real theta,
    Real omega,
    bool advance,
    bool reseat,
    BIKE_THREAD bool &has_latch,
    BIKE_THREAD Real &latch_theta,
    BIKE_THREAD Real &out_torque,
    BIKE_THREAD ErrCode &err
) noexcept {
    return eval_brake_hold_wheel<Real, TK>(
        spec,
        spec->torque_nm,
        demand,
        theta,
        omega,
        advance,
        reseat,
        has_latch,
        latch_theta,
        out_torque,
        err);
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
