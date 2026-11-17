// native/src/physics_core/kinematics.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// analytical point Jacobian along a wheel body's 5/6-DOF ancestor joint chain.
#pragma once

#include "common.hpp"
#include "tyre_types.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

template <
    class Real,
    class TK,
    bool ComputeJr,
    class PtrPos,
    class PtrAnchor,
    class PtrAxis,
    class PtrPt,
    class PtrJp,
    class PtrJr>
BIKE_HD Status chain_point_jacobian(
    const BIKE_THREAD GpuWheelChain &chain,
    PtrPos body_xpos,
    PtrAnchor jnt_xanchor,
    PtrAxis jnt_xaxis,
    int wheel_body_id,
    int nv,
    int njnt,
    PtrPt pt,
    PtrJp out_jp,
    PtrJr out_jr,
    BIKE_THREAD ErrCode &err
) noexcept {
    (void)body_xpos;
    err = ErrCode::none;

    if (chain.joint_count < 0 || chain.joint_count > kMaxChainJoints ||
        nv <= 0 || nv > kMaxNv || njnt <= 0 || njnt > kMaxNjnt ||
        wheel_body_id <= 0) {
        err = ErrCode::out_of_bounds_chain;
        return Status::out_of_bounds;
    }
    if (chain.wheel_body_id >= 0 && chain.wheel_body_id != wheel_body_id) {
        err = ErrCode::out_of_bounds_chain;
        return Status::invalid_argument;
    }
    if (jnt_xanchor == nullptr || jnt_xaxis == nullptr ||
        pt == nullptr || out_jp == nullptr) {
        err = ErrCode::out_of_bounds_chain;
        return Status::invalid_argument;
    }
    if constexpr (ComputeJr) {
        if (out_jr == nullptr) {
            err = ErrCode::out_of_bounds_chain;
            return Status::invalid_argument;
        }
    }

    for (int k = 0; k < chain.joint_count; ++k) {
        const GpuChainJoint j = chain.joints[k];
        if (j.joint_id < 0 || j.joint_id >= njnt ||
            j.dof_adr < 0 || j.dof_adr >= nv) {
            err = ErrCode::out_of_bounds_chain;
            return Status::out_of_bounds;
        }
        if (j.joint_type != kJntHinge && j.joint_type != kJntSlide) {
            err = ErrCode::out_of_bounds_chain;
            return Status::invalid_argument;
        }
    }

    for (int r = 0; r < 3; ++r) {
        const int row_base = r * nv;
        for (int c = 0; c < nv; ++c) {
            out_jp[row_base + c] = static_cast<Real>(0);
            if constexpr (ComputeJr) {
                out_jr[row_base + c] = static_cast<Real>(0);
            }
        }
    }

    const Real px = pt[0];
    const Real py = pt[1];
    const Real pz = pt[2];

    for (int k = 0; k < chain.joint_count; ++k) {
        const GpuChainJoint j = chain.joints[k];
        const int a3 = 3 * j.joint_id;
        const int d = j.dof_adr;
        const Real ax0 = jnt_xaxis[a3 + 0];
        const Real ax1 = jnt_xaxis[a3 + 1];
        const Real ax2 = jnt_xaxis[a3 + 2];

        if (j.joint_type == kJntHinge) {
            const Real arm0 = px - jnt_xanchor[a3 + 0];
            const Real arm1 = py - jnt_xanchor[a3 + 1];
            const Real arm2 = pz - jnt_xanchor[a3 + 2];
            out_jp[0 * nv + d] = ax1 * arm2 - ax2 * arm1;
            out_jp[1 * nv + d] = ax2 * arm0 - ax0 * arm2;
            out_jp[2 * nv + d] = ax0 * arm1 - ax1 * arm0;
            if constexpr (ComputeJr) {
                out_jr[0 * nv + d] = ax0;
                out_jr[1 * nv + d] = ax1;
                out_jr[2 * nv + d] = ax2;
            }
        } else {
            out_jp[0 * nv + d] = ax0;
            out_jp[1 * nv + d] = ax1;
            out_jp[2 * nv + d] = ax2;
        }
    }

    return Status::ok;
}

template <
    class Real,
    class TK,
    bool ComputeJr,
    class PtrPos,
    class PtrAnchor,
    class PtrAxis,
    class PtrPt,
    class PtrJp,
    class PtrJr>
BIKE_HD Status chain_point_jacobian(
    const BIKE_THREAD GpuWheelChain &chain,
    PtrPos body_xpos,
    PtrAnchor jnt_xanchor,
    PtrAxis jnt_xaxis,
    int wheel_body_id,
    int nv,
    PtrPt pt,
    PtrJp out_jp,
    PtrJr out_jr,
    BIKE_THREAD ErrCode &err
) noexcept {
    const int njnt = (chain.njnt > 0) ? chain.njnt : kMaxNjnt;
    return chain_point_jacobian<Real, TK, ComputeJr>(
        chain,
        body_xpos,
        jnt_xanchor,
        jnt_xaxis,
        wheel_body_id,
        nv,
        njnt,
        pt,
        out_jp,
        out_jr,
        err
    );
}

template <
    class Real,
    class TK,
    class PtrAxis,
    class PtrJr>
BIKE_HD Status chain_angular_jacobian(
    const BIKE_THREAD GpuWheelChain &chain,
    PtrAxis jnt_xaxis,
    int body_id,
    int nv,
    int njnt,
    PtrJr out_jr,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;

    if (chain.joint_count < 0 || chain.joint_count > kMaxChainJoints ||
        nv <= 0 || nv > kMaxNv || njnt <= 0 || njnt > kMaxNjnt ||
        body_id <= 0) {
        err = ErrCode::out_of_bounds_chain;
        return Status::out_of_bounds;
    }
    if (chain.wheel_body_id >= 0 && chain.wheel_body_id != body_id) {
        err = ErrCode::out_of_bounds_chain;
        return Status::invalid_argument;
    }
    if (jnt_xaxis == nullptr || out_jr == nullptr) {
        err = ErrCode::out_of_bounds_chain;
        return Status::invalid_argument;
    }

    for (int k = 0; k < chain.joint_count; ++k) {
        const GpuChainJoint j = chain.joints[k];
        if (j.joint_id < 0 || j.joint_id >= njnt ||
            j.dof_adr < 0 || j.dof_adr >= nv) {
            err = ErrCode::out_of_bounds_chain;
            return Status::out_of_bounds;
        }
        if (j.joint_type != kJntHinge && j.joint_type != kJntSlide) {
            err = ErrCode::out_of_bounds_chain;
            return Status::invalid_argument;
        }
    }

    for (int r = 0; r < 3; ++r) {
        const int row_base = r * nv;
        for (int c = 0; c < nv; ++c) {
            out_jr[row_base + c] = static_cast<Real>(0);
        }
    }

    for (int k = 0; k < chain.joint_count; ++k) {
        const GpuChainJoint j = chain.joints[k];
        if (j.joint_type == kJntHinge) {
            const int a3 = 3 * j.joint_id;
            const int d = j.dof_adr;
            out_jr[0 * nv + d] = jnt_xaxis[a3 + 0];
            out_jr[1 * nv + d] = jnt_xaxis[a3 + 1];
            out_jr[2 * nv + d] = jnt_xaxis[a3 + 2];
        }
    }

    return Status::ok;
}

template <
    class Real,
    class TK,
    class PtrAxis,
    class PtrJr>
BIKE_HD Status chain_angular_jacobian(
    const BIKE_THREAD GpuWheelChain &chain,
    PtrAxis jnt_xaxis,
    int body_id,
    int nv,
    PtrJr out_jr,
    BIKE_THREAD ErrCode &err
) noexcept {
    const int njnt = (chain.njnt > 0) ? chain.njnt : kMaxNjnt;
    return chain_angular_jacobian<Real, TK>(
        chain,
        jnt_xaxis,
        body_id,
        nv,
        njnt,
        out_jr,
        err
    );
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
