// native/src/physics_core/resistance.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// rolling resistance (with full rotational Jacobian coupling) and planar X-Z aerodynamic drag.
#pragma once

#include "common.hpp"
#include "kinematics.hpp"
#include "tyre_types.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

template <class Real>
struct ResistanceParams {
    Real crr = BIKE_REAL(Real, 0.0);
    Real rolling_taper_rad_s = BIKE_REAL(Real, 0.2);
    Real rho_kg_m3 = BIKE_REAL(Real, 1.2);
    Real cda_m2 = BIKE_REAL(Real, 0.5);
    Real wind_world_mps[3]{};
    Real point_body_m[3]{};
};

enum class ResDiagField : u8 {
    none = 0,
    invalid_nv,
    normal_load,
    rolling_moment,
    point,
    nonplanar_velocity,
    velocity_norm,
    drag_force,
    rolling,
    aerodynamic
};

template <class Real, class TK, class PtrAxis, class PtrChain>
struct ChainWheelJrFn {
    PtrChain chains;
    PtrAxis jnt_xaxis;
    i32 nv = 0;
    i32 njnt = 0;

    template <class PtrPos, class PtrJr>
    BIKE_HD Status operator()(
        i32 side,
        PtrPos,
        PtrJr out_jr,
        BIKE_THREAD ErrCode &err
    ) const noexcept {
        const GpuWheelChain chain = chains[side];
        return chain_angular_jacobian<Real, TK>(
            chain, jnt_xaxis, chain.wheel_body_id, nv, njnt, out_jr, err);
    }
};

template <class Real, class TK, class PtrPos, class PtrAnchor, class PtrAxis>
struct ChainFrameJpFn {
    GpuWheelChain chain{};
    PtrPos body_xpos;
    PtrAnchor jnt_xanchor;
    PtrAxis jnt_xaxis;
    i32 nv = 0;
    i32 njnt = 0;

    template <class PtrPt, class PtrJp>
    BIKE_HD Status operator()(
        PtrPt pt,
        PtrJp out_jp,
        BIKE_THREAD ErrCode &err
    ) const noexcept {
        const GpuWheelChain local_chain = chain;
        BIKE_THREAD Real *dummy_jr = nullptr;
        return chain_point_jacobian<Real, TK, false>(
            local_chain, body_xpos, jnt_xanchor, jnt_xaxis,
            local_chain.wheel_body_id, nv, njnt, pt, out_jp, dummy_jr, err);
    }
};

template <
    class Real,
    class TK,
    class PtrParams,
    class PtrQvel,
    class PtrFrontLoads,
    class PtrFrontWork,
    class PtrFrontXpos,
    class PtrFrontXmat,
    class PtrRearLoads,
    class PtrRearWork,
    class PtrRearXpos,
    class PtrRearXmat,
    class PtrFrameXpos,
    class PtrFrameXmat,
    class PtrJr,
    class PtrJp,
    class PtrRow,
    class PtrRollOut,
    class PtrAeroOut,
    class WheelJrFn,
    class FrameJpFn>
BIKE_HD Status eval_resistance(
    i32 nv,
    PtrParams cfg,
    PtrQvel qvel,
    PtrFrontLoads front_loads,
    PtrFrontWork front_working,
    i32 front_n,
    Real front_eff_r,
    PtrFrontXpos front_xpos,
    PtrFrontXmat front_xmat,
    PtrRearLoads rear_loads,
    PtrRearWork rear_working,
    i32 rear_n,
    Real rear_eff_r,
    PtrRearXpos rear_xpos,
    PtrRearXmat rear_xmat,
    PtrFrameXpos frame_xpos,
    PtrFrameXmat frame_xmat,
    PtrJr jr_scratch,
    PtrJp jp_scratch,
    PtrRow row_scratch,
    PtrRollOut rolling_out,
    PtrAeroOut aero_out,
    BIKE_THREAD WheelJrFn &wheel_jr_fn,
    BIKE_THREAD FrameJpFn &frame_jp_fn,
    BIKE_THREAD ErrCode &err,
    BIKE_THREAD ResDiagField &diag
) noexcept {
    err = ErrCode::none;
    diag = ResDiagField::none;
    if (nv < 0) {
        err = ErrCode::out_of_bounds_chain;
        diag = ResDiagField::invalid_nv;
        return Status::invalid_argument;
    }

    for (i32 i = 0; i < nv; ++i) {
        rolling_out[i] = BIKE_REAL(Real, 0.0);
    }

    for (i32 side = 0; side < 2; ++side) {
        const Real Fn = (side == 0)
            ? TK::masked_sum(front_n, front_loads, front_working)
            : TK::masked_sum(rear_n, rear_loads, rear_working);
        if (!TK::isfinite(Fn)) {
            err = ErrCode::overflow_nonfinite;
            diag = ResDiagField::normal_load;
            return Status::overflow;
        }
        Real axis[3] = {
            (side == 0) ? front_xmat[1] : rear_xmat[1],
            (side == 0) ? front_xmat[4] : rear_xmat[4],
            (side == 0) ? front_xmat[7] : rear_xmat[7]
        };
        const Status jst = (side == 0)
            ? wheel_jr_fn(0, front_xpos, jr_scratch, err)
            : wheel_jr_fn(1, rear_xpos, jr_scratch, err);
        if (jst != Status::ok) {
            return jst;
        }
        TK::gemv_t3(nv, jr_scratch, axis, row_scratch);
        const Real omega = TK::dot(nv, row_scratch, qvel);
        const Real radius = (side == 0) ? front_eff_r : rear_eff_r;
        if (Fn > BIKE_REAL(Real, 0.0) && radius > BIKE_REAL(Real, 0.0)) {
            const Real moment =
                -cfg->crr * Fn * radius * TK::tanh(omega / cfg->rolling_taper_rad_s);
            if (!TK::isfinite(moment)) {
                err = ErrCode::overflow_nonfinite;
                diag = ResDiagField::rolling_moment;
                return Status::overflow;
            }
            for (i32 i = 0; i < nv; ++i) {
                rolling_out[i] += moment * row_scratch[i];
            }
        }
    }

    Real rotp[3]{};
    TK::gemv_n3(3, frame_xmat, cfg->point_body_m, rotp);
    Real pt[3]{};
    for (i32 i = 0; i < 3; ++i) {
        pt[i] = frame_xpos[i] + rotp[i];
        if (!TK::isfinite(pt[i])) {
            err = ErrCode::overflow_nonfinite;
            diag = ResDiagField::point;
            return Status::overflow;
        }
    }

    const Status jpst = frame_jp_fn(pt, jp_scratch, err);
    if (jpst != Status::ok) {
        return jpst;
    }

    Real velocity[3]{};
    TK::gemv_n3(nv, jp_scratch, qvel, velocity);
    Real relative[3]{};
    for (i32 i = 0; i < 3; ++i) {
        relative[i] = velocity[i] - cfg->wind_world_mps[i];
    }
    if (relative[1] != BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        diag = ResDiagField::nonplanar_velocity;
        return Status::invalid_argument;
    }

    const Real norm = TK::sqrt(TK::dot3(relative, relative));
    if (!TK::isfinite(norm)) {
        err = ErrCode::overflow_nonfinite;
        diag = ResDiagField::velocity_norm;
        return Status::overflow;
    }

    const Real t = ((BIKE_REAL(Real, -0.5) * cfg->rho_kg_m3) * cfg->cda_m2) * norm;
    Real force[3]{};
    for (i32 i = 0; i < 3; ++i) {
        force[i] = t * relative[i];
        if (!TK::isfinite(force[i])) {
            err = ErrCode::overflow_nonfinite;
            diag = ResDiagField::drag_force;
            return Status::overflow;
        }
    }

    TK::gemv_t3(nv, jp_scratch, force, aero_out);
    for (i32 i = 0; i < nv; ++i) {
        if (!TK::isfinite(rolling_out[i])) {
            err = ErrCode::overflow_nonfinite;
            diag = ResDiagField::rolling;
            return Status::overflow;
        }
    }
    for (i32 i = 0; i < nv; ++i) {
        if (!TK::isfinite(aero_out[i])) {
            err = ErrCode::overflow_nonfinite;
            diag = ResDiagField::aerodynamic;
            return Status::overflow;
        }
    }
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
