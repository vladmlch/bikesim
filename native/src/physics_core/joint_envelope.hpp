// native/src/physics_core/joint_envelope.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// rider anatomical soft-edge joint envelope law (O(1) registers, zero heap allocation).
#pragma once

#include "common.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

BIKE_CONSTEXPR i32 kMaxEnvelopeJoints = 16;

template <class Real>
struct SoftEdgeResult {
    Real torque = BIKE_REAL(Real, 0.0);
    Real stored_j = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct RiderJointEnvelopeSpec {
    i32 count = 0;
    i32 qpos_adrs[kMaxEnvelopeJoints]{};
    i32 dof_adrs[kMaxEnvelopeJoints]{};
    Real lower[kMaxEnvelopeJoints]{};
    Real upper[kMaxEnvelopeJoints]{};
    Real stiffness_nm_rad = BIKE_REAL(Real, 100.0);
    Real margin_rad = BIKE_REAL(Real, 0.1);
};

template <class Real, class TK>
BIKE_HD Status soft_edge_scalar(
    Real q,
    Real lower,
    Real upper,
    Real stiffness_nm_rad,
    Real margin_rad,
    BIKE_THREAD SoftEdgeResult<Real> &out,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out = SoftEdgeResult<Real>{};
    if (!TK::isfinite(q) || !TK::isfinite(lower) || !TK::isfinite(upper) ||
        !TK::isfinite(stiffness_nm_rad) || !TK::isfinite(margin_rad) ||
        upper <= lower || stiffness_nm_rad < BIKE_REAL(Real, 0.0) ||
        margin_rad < BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }

    const Real margin =
        TK::min(margin_rad, (upper - lower) / BIKE_REAL(Real, 2.0));
    const Real left = TK::max(lower + margin - q, BIKE_REAL(Real, 0.0));
    const Real right = TK::max(q - upper + margin, BIKE_REAL(Real, 0.0));
    out.torque = stiffness_nm_rad * (left - right);
    out.stored_j =
        BIKE_REAL(Real, 0.5) * stiffness_nm_rad * (left * left + right * right);
    return Status::ok;
}

template <class Real, class TK, class SpecPtr, class PtrQpos, class PtrQfrc>
BIKE_HD Status eval_rider_joint_envelope(
    SpecPtr spec,
    i32 nq,
    i32 nv,
    PtrQpos qpos,
    PtrQfrc out_qfrc,
    BIKE_THREAD Real &out_stored_j,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_stored_j = BIKE_REAL(Real, 0.0);
    if (spec->count < 0 || spec->count > kMaxEnvelopeJoints ||
        nq <= 0 || nv <= 0 || qpos == nullptr || out_qfrc == nullptr) {
        err = ErrCode::out_of_bounds_chain;
        return Status::out_of_bounds;
    }
    if (!TK::isfinite(spec->stiffness_nm_rad) || !TK::isfinite(spec->margin_rad) ||
        spec->stiffness_nm_rad < BIKE_REAL(Real, 0.0) ||
        spec->margin_rad < BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    for (i32 i = 0; i < spec->count; ++i) {
        if (spec->qpos_adrs[i] < 0 || spec->qpos_adrs[i] >= nq ||
            spec->dof_adrs[i] < 0 || spec->dof_adrs[i] >= nv) {
            err = ErrCode::out_of_bounds_chain;
            return Status::out_of_bounds;
        }
        const Real q = qpos[spec->qpos_adrs[i]];
        const Real lo = spec->lower[i];
        const Real hi = spec->upper[i];
        if (!TK::isfinite(q) || !TK::isfinite(lo) || !TK::isfinite(hi) || hi <= lo) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
    }

    Real stored = BIKE_REAL(Real, 0.0);
    for (i32 i = 0; i < spec->count; ++i) {
        SoftEdgeResult<Real> row{};
        const Status st = soft_edge_scalar<Real, TK>(
            qpos[spec->qpos_adrs[i]], spec->lower[i], spec->upper[i],
            spec->stiffness_nm_rad, spec->margin_rad, row, err);
        if (st != Status::ok) {
            return st;
        }
        out_qfrc[spec->dof_adrs[i]] += row.torque;
        stored += row.stored_j;
    }
    out_stored_j = stored;
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
