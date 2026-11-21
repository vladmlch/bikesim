// native/src/physics_core/efc_monitors.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// post-solve d.efc single-pass weld/tendon/brake/contact scans, int32-tick runtime monitors
// (GroundedFilter, BridgedLoad, BalanceMonitor, CrashDetector), StaticBrake rollback latch,
// and combined SP3 per-world state footprint contract (<= 16 KiB/world).
#pragma once

#include "common.hpp"
#include "launch_plan.hpp"
#include "rider_intent.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: POD structs and templated pointers match the CUDA/MSL ABI without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

BIKE_CONSTEXPR i32 kMaxRiderCrashGeoms = 16;
BIKE_CONSTEXPR i32 kMaxTerrainGeoms = 4;
BIKE_CONSTEXPR i32 kMaxSp3StateBytesPerWorld = 16384;

// MuJoCo mjtConstraint constants (mjtype.h)
BIKE_CONSTEXPR i32 kCnstrEquality = 0;
BIKE_CONSTEXPR i32 kCnstrFrictionDof = 1;
BIKE_CONSTEXPR i32 kCnstrLimitTendon = 4;
BIKE_CONSTEXPR i32 kCnstrContactFrictionless = 5;
BIKE_CONSTEXPR i32 kCnstrContactPyramidal = 6;
BIKE_CONSTEXPR i32 kCnstrContactElliptic = 7;

enum class CrashCause : u8 {
    none = 0,
    catch_plane_contact = 1,
    rider_ground_contact = 2,
    pitch_over = 3,
    handlebar_contact = 4
};

enum class BalanceStopCore : u8 {
    none = 0,
    braked = 1
};

template <class Real>
struct EfcScanConfig {
    i32 grip_left_eq_id = -1;
    i32 grip_right_eq_id = -1;
    i32 saddle_eq_id = -1;
    i32 foot_front_eq_id = -1;
    i32 foot_rear_eq_id = -1;
    i32 frame_body_id = -1;
    i32 crank_dof = -1;
    i32 front_wheel_dof = -1;
    i32 rear_wheel_dof = -1;
    i32 freehub_tendon_id = -1;
    i32 crank_clutch_tendon_id = -1;
    i32 handlebar_geom_id = -1;
    i32 catch_geom_id = -1;
    i32 terrain_geom_count = 0;
    i32 terrain_geom_ids[kMaxTerrainGeoms]{};
    i32 rider_geom_count = 0;
    i32 rider_geom_ids[kMaxRiderCrashGeoms]{};
    Real load_threshold_n = BIKE_REAL(Real, 1.0);
    Real pitch_limit_rad = BIKE_REAL(Real, 1.0471975511965976);
};

template <class Real>
struct EfcScanResult {
    Real hand_press_n = BIKE_REAL(Real, 0.0);
    Real left_hand_press_n = BIKE_REAL(Real, 0.0);
    Real right_hand_press_n = BIKE_REAL(Real, 0.0);
    Real pelvis_saddle_force_n = BIKE_REAL(Real, 0.0);
    Real foot_front_press_n = BIKE_REAL(Real, 0.0);
    Real foot_rear_press_n = BIKE_REAL(Real, 0.0);
    Real delivered_crank_torque_nm = BIKE_REAL(Real, 0.0);
    Real front_brake_friction_torque_nm = BIKE_REAL(Real, 0.0);
    Real rear_brake_friction_torque_nm = BIKE_REAL(Real, 0.0);
    Real freehub_tendon_force_n = BIKE_REAL(Real, 0.0);
    Real crank_clutch_tendon_force_n = BIKE_REAL(Real, 0.0);
    Real handlebar_load_n = BIKE_REAL(Real, 0.0);
    u8 saddle_separating = 0;
    u8 handlebar_in_contact = 0;
    CrashCause contact_crash_cause = CrashCause::none;
    u8 has_grip_left = 0;
    u8 has_grip_right = 0;
    u8 has_saddle = 0;
    u8 has_foot_front = 0;
    u8 has_foot_rear = 0;
};

namespace detail {

template <class Real>
BIKE_HD Real dot_up3(const BIKE_THREAD Real v[3], Real up0, Real up1, Real up2) noexcept {
    return v[0] * up0 + v[1] * up1 + v[2] * up2;
}

template <class PtrArr>
BIKE_HD bool geom_in_list(i32 g, PtrArr arr, i32 count) noexcept {
    for (i32 k = 0; k < count; ++k) {
        if (arr[k] == g) {
            return true;
        }
    }
    return false;
}

} // namespace detail

template <
    class Real,
    class TK,
    class CfgPtr,
    class PtrType,
    class PtrId,
    class PtrJ,
    class PtrForce,
    class PtrXmat>
BIKE_HD Status scan_efc_reactions(
    CfgPtr cfg,
    i32 nefc,
    i32 efc_J_stride,
    i32 nbody,
    PtrType efc_type,
    PtrId efc_id,
    PtrJ efc_J,
    PtrForce efc_force,
    PtrXmat body_xmat,
    BIKE_THREAD EfcScanResult<Real> &out,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out = EfcScanResult<Real>{};
    if (nefc < 0 || efc_J_stride < 0 || nbody <= 0 ||
        cfg->frame_body_id < 0 || cfg->frame_body_id >= nbody ||
        body_xmat == nullptr) {
        err = ErrCode::out_of_bounds_chain;
        return Status::out_of_bounds;
    }
    if (nefc > 0 && (efc_type == nullptr || efc_id == nullptr || efc_force == nullptr)) {
        err = ErrCode::out_of_bounds_chain;
        return Status::invalid_argument;
    }
    if (nefc > 0 && cfg->crank_dof >= 0 &&
        (efc_J == nullptr || cfg->crank_dof >= efc_J_stride)) {
        err = ErrCode::out_of_bounds_chain;
        return Status::out_of_bounds;
    }

    const i32 fb = 9 * cfg->frame_body_id;
    const Real up0 = body_xmat[fb + 2];
    const Real up1 = body_xmat[fb + 5];
    const Real up2 = body_xmat[fb + 8];
    if (!TK::isfinite(up0) || !TK::isfinite(up1) || !TK::isfinite(up2)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }

    Real grip_left_f[3]{};
    Real grip_right_f[3]{};
    Real saddle_f[3]{};
    Real foot_front_f[3]{};
    Real foot_rear_f[3]{};
    i32 gl_cnt = 0;
    i32 gr_cnt = 0;
    i32 sd_cnt = 0;
    i32 ff_cnt = 0;
    i32 fr_cnt = 0;

    for (i32 row = 0; row < nefc; ++row) {
        const i32 ctype = efc_type[row];
        const i32 cid = efc_id[row];
        const Real f = efc_force[row];

        if (ctype == kCnstrEquality) {
            if (cid == cfg->foot_front_eq_id) {
                if (ff_cnt < 3) {
                    foot_front_f[ff_cnt] = f;
                    ++ff_cnt;
                }
                if (cfg->crank_dof >= 0 && efc_J != nullptr) {
                    const long row_off = static_cast<long>(row) * static_cast<long>(efc_J_stride);
                    out.delivered_crank_torque_nm +=
                        efc_J[row_off + static_cast<long>(cfg->crank_dof)] * f;
                }
            } else if (cid == cfg->foot_rear_eq_id) {
                if (fr_cnt < 3) {
                    foot_rear_f[fr_cnt] = f;
                    ++fr_cnt;
                }
                if (cfg->crank_dof >= 0 && efc_J != nullptr) {
                    const long row_off = static_cast<long>(row) * static_cast<long>(efc_J_stride);
                    out.delivered_crank_torque_nm +=
                        efc_J[row_off + static_cast<long>(cfg->crank_dof)] * f;
                }
            } else if (cid == cfg->grip_left_eq_id) {
                if (gl_cnt < 3) {
                    grip_left_f[gl_cnt] = f;
                    ++gl_cnt;
                }
            } else if (cid == cfg->grip_right_eq_id) {
                if (gr_cnt < 3) {
                    grip_right_f[gr_cnt] = f;
                    ++gr_cnt;
                }
            } else if (cid == cfg->saddle_eq_id) {
                if (sd_cnt < 3) {
                    saddle_f[sd_cnt] = f;
                    ++sd_cnt;
                }
            }
        } else if (ctype == kCnstrFrictionDof) {
            if (cid == cfg->front_wheel_dof && cfg->front_wheel_dof >= 0) {
                const long row_off = static_cast<long>(row) * static_cast<long>(efc_J_stride);
                const Real jac =
                    (efc_J != nullptr && cfg->front_wheel_dof < efc_J_stride)
                        ? efc_J[row_off + static_cast<long>(cfg->front_wheel_dof)]
                        : BIKE_REAL(Real, 1.0);
                out.front_brake_friction_torque_nm += jac * f;
            } else if (cid == cfg->rear_wheel_dof && cfg->rear_wheel_dof >= 0) {
                const long row_off = static_cast<long>(row) * static_cast<long>(efc_J_stride);
                const Real jac =
                    (efc_J != nullptr && cfg->rear_wheel_dof < efc_J_stride)
                        ? efc_J[row_off + static_cast<long>(cfg->rear_wheel_dof)]
                        : BIKE_REAL(Real, 1.0);
                out.rear_brake_friction_torque_nm += jac * f;
            }
        } else if (ctype == kCnstrLimitTendon) {
            if (cid == cfg->freehub_tendon_id && cfg->freehub_tendon_id >= 0) {
                out.freehub_tendon_force_n += f;
            } else if (cid == cfg->crank_clutch_tendon_id && cfg->crank_clutch_tendon_id >= 0) {
                out.crank_clutch_tendon_force_n += f;
            }
        }
    }

    if (gl_cnt == 3) {
        out.has_grip_left = 1;
        out.left_hand_press_n = detail::dot_up3(grip_left_f, up0, up1, up2);
    }
    if (gr_cnt == 3) {
        out.has_grip_right = 1;
        out.right_hand_press_n = detail::dot_up3(grip_right_f, up0, up1, up2);
    }
    out.hand_press_n = out.left_hand_press_n + out.right_hand_press_n;

    if (sd_cnt == 3) {
        out.has_saddle = 1;
        out.pelvis_saddle_force_n = detail::dot_up3(saddle_f, up0, up1, up2);
        out.saddle_separating =
            out.pelvis_saddle_force_n < BIKE_REAL(Real, 0.0) ? 1 : 0;
    }
    if (ff_cnt == 3) {
        out.has_foot_front = 1;
        out.foot_front_press_n = detail::dot_up3(foot_front_f, up0, up1, up2);
    }
    if (fr_cnt == 3) {
        out.has_foot_rear = 1;
        out.foot_rear_press_n = detail::dot_up3(foot_rear_f, up0, up1, up2);
    }

    if (!TK::isfinite(out.hand_press_n) ||
        !TK::isfinite(out.pelvis_saddle_force_n) ||
        !TK::isfinite(out.delivered_crank_torque_nm)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    return Status::ok;
}

template <
    class Real,
    class TK,
    class CfgPtr,
    class PtrGeom,
    class PtrEfcAdr,
    class PtrDim,
    class PtrType,
    class PtrId,
    class PtrForce>
BIKE_HD Status scan_contact_monitors(
    CfgPtr cfg,
    i32 ncon,
    i32 nefc,
    PtrGeom contact_geom1,
    PtrGeom contact_geom2,
    PtrEfcAdr contact_efc_address,
    PtrDim contact_dim,
    PtrType efc_type,
    PtrId efc_id,
    PtrForce efc_force,
    BIKE_THREAD EfcScanResult<Real> &in_out,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    in_out.handlebar_load_n = BIKE_REAL(Real, 0.0);
    in_out.handlebar_in_contact = 0;
    in_out.contact_crash_cause = CrashCause::none;
    if (ncon < 0 || nefc < 0 ||
        cfg->terrain_geom_count < 0 || cfg->terrain_geom_count > kMaxTerrainGeoms ||
        cfg->rider_geom_count < 0 || cfg->rider_geom_count > kMaxRiderCrashGeoms) {
        err = ErrCode::out_of_bounds_chain;
        return Status::out_of_bounds;
    }
    if (!TK::isfinite(cfg->load_threshold_n) || cfg->load_threshold_n < BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (ncon == 0) {
        return Status::ok;
    }
    if (contact_geom1 == nullptr || contact_geom2 == nullptr ||
        contact_efc_address == nullptr) {
        err = ErrCode::out_of_bounds_chain;
        return Status::invalid_argument;
    }
    if (nefc > 0 && (efc_type == nullptr || efc_force == nullptr)) {
        err = ErrCode::out_of_bounds_chain;
        return Status::invalid_argument;
    }

    for (i32 i = 0; i < ncon; ++i) {
        const i32 g1 = contact_geom1[i];
        const i32 g2 = contact_geom2[i];

        const bool g1_terrain =
            detail::geom_in_list(g1, cfg->terrain_geom_ids, cfg->terrain_geom_count);
        const bool g2_terrain =
            detail::geom_in_list(g2, cfg->terrain_geom_ids, cfg->terrain_geom_count);
        const bool bar_hit =
            cfg->handlebar_geom_id >= 0 &&
            ((g1 == cfg->handlebar_geom_id && g2_terrain) ||
             (g2 == cfg->handlebar_geom_id && g1_terrain));
        const bool catch_hit =
            cfg->catch_geom_id >= 0 &&
            (g1 == cfg->catch_geom_id || g2 == cfg->catch_geom_id);
        const bool rider_ground =
            !catch_hit && (g1_terrain || g2_terrain) &&
            (detail::geom_in_list(g1, cfg->rider_geom_ids, cfg->rider_geom_count) ||
             detail::geom_in_list(g2, cfg->rider_geom_ids, cfg->rider_geom_count));

        if (!bar_hit && !catch_hit && !rider_ground) {
            continue;
        }
        const i32 adr = contact_efc_address[i];
        if (adr < 0 || adr >= nefc) {
            continue;
        }

        Real fn = BIKE_REAL(Real, 0.0);
        const i32 ctype = efc_type[adr];
        if (ctype == kCnstrContactPyramidal) {
            if (contact_dim != nullptr) {
                const i32 dim = contact_dim[i];
                const i32 nrows = dim > 1 ? 2 * (dim - 1) : 1;
                const i32 end_row = (adr + nrows) < nefc ? (adr + nrows) : nefc;
                for (i32 r = adr; r < end_row; ++r) {
                    fn += efc_force[r];
                }
            } else if (efc_id != nullptr) {
                for (i32 r = adr;
                     r < nefc && efc_type[r] == kCnstrContactPyramidal && efc_id[r] == i;
                     ++r) {
                    fn += efc_force[r];
                }
            } else {
                fn = efc_force[adr];
            }
        } else {
            fn = efc_force[adr];
        }

        if (!TK::isfinite(fn)) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }
        if (bar_hit) {
            in_out.handlebar_load_n += TK::abs(fn);
        }
        if (in_out.contact_crash_cause == CrashCause::none &&
            fn > cfg->load_threshold_n) {
            if (catch_hit) {
                in_out.contact_crash_cause = CrashCause::catch_plane_contact;
            } else if (rider_ground) {
                in_out.contact_crash_cause = CrashCause::rider_ground_contact;
            }
        }
    }

    in_out.handlebar_in_contact =
        in_out.handlebar_load_n > cfg->load_threshold_n ? 1 : 0;
    return Status::ok;
}

// ---- GroundedFilter (int32 tick counter; 0 false +1 tick drift over 3,600 s) ----
struct GroundedFilterConfig {
    i32 hold_ticks = 4;
};

struct GroundedFilterState {
    i32 last_step = -1;
    i32 unloaded_ticks = 0;
    u8 has_loaded = 0;
    u8 value = 0;
    u8 pad[2]{};
};

template <class StatePtr>
BIKE_HD void grounded_filter_reset(StatePtr state) noexcept {
    *state = GroundedFilterState{};
}

template <class CfgPtr, class StatePtr>
BIKE_HD Status grounded_filter_update(
    CfgPtr cfg,
    bool raw_grounded,
    i32 step,
    StatePtr state,
    BIKE_THREAD bool &out_value,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_value = false;
    if (cfg->hold_ticks < 0 || step < 0) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (state->last_step >= 0) {
        if (step < state->last_step) {
            err = ErrCode::bad_time;
            return Status::invalid_argument;
        }
        if (step == state->last_step) {
            out_value = (state->value != 0);
            return Status::ok;
        }
    }
    state->last_step = step;
    if (raw_grounded) {
        state->has_loaded = 1;
        state->unloaded_ticks = 0;
        state->value = 1;
    } else {
        if (state->has_loaded != 0 && state->unloaded_ticks < cfg->hold_ticks) {
            ++state->unloaded_ticks;
        }
        state->value =
            (state->has_loaded != 0 && state->unloaded_ticks < cfg->hold_ticks) ? 1 : 0;
    }
    out_value = (state->value != 0);
    return Status::ok;
}

// ---- BridgedLoad ----
template <class Real>
struct BridgedLoadConfig {
    i32 dropout_steps = 0;
    Real threshold_n = BIKE_REAL(Real, 1.0);
};

template <class Real>
struct BridgedLoadState {
    Real held_load_n = BIKE_REAL(Real, 0.0);
    i32 unloaded_steps = 0;
};

template <class Real, class StatePtr>
BIKE_HD void bridged_load_reset(StatePtr state) noexcept {
    *state = BridgedLoadState<Real>{};
}

template <class Real, class CfgPtr, class StatePtr>
BIKE_HD Real bridged_load_update(
    CfgPtr cfg,
    Real raw_load_n,
    StatePtr state
) noexcept {
    if (raw_load_n > cfg->threshold_n) {
        state->held_load_n = raw_load_n;
        state->unloaded_steps = 0;
    } else {
        if (state->unloaded_steps <= cfg->dropout_steps) {
            ++state->unloaded_steps;
        }
        if (state->unloaded_steps > cfg->dropout_steps) {
            state->held_load_n = BIKE_REAL(Real, 0.0);
        }
    }
    return state->held_load_n;
}

// ---- BalanceMonitor (int32 grace_ticks_left & low_speed_ticks) ----
template <class Real>
struct BalanceMonitorConfig {
    Real floor_mps = BIKE_REAL(Real, 1.1111111111111112);
    i32 dwell_ticks = 400;
    i32 grace_ticks = 2400;
    Real period_s = BIKE_REAL(Real, 0.00125);
};

template <class Real>
struct BalanceMonitorState {
    i32 last_step = -1;
    i32 grace_ticks_left = 2400;
    i32 low_speed_ticks = 0;
    Real event_time_s = BIKE_REAL(Real, 0.0);
    Real event_position_m = BIKE_REAL(Real, 0.0);
    Real event_speed_mps = BIKE_REAL(Real, 0.0);
    Real cur_time_s = BIKE_REAL(Real, 0.0);
    Real cur_position_m = BIKE_REAL(Real, 0.0);
    Real cur_speed_mps = BIKE_REAL(Real, 0.0);
    u8 triggered = 0;
    u8 active = 0;
    u8 started = 0;
    u8 low_started = 0;
};

template <class Real, class CfgPtr, class StatePtr>
BIKE_HD void balance_monitor_reset(CfgPtr cfg, StatePtr state) noexcept {
    *state = BalanceMonitorState<Real>{};
    state->grace_ticks_left = cfg->grace_ticks;
}

template <class Real, class TK, class StatePtr>
BIKE_HD BalanceStopCore balance_compute_stop(
    bool motor_braking,
    Real speed_mps,
    Real standstill_speed_mps,
    LaunchPhase launch_phase,
    StatePtr bal
) noexcept {
    const bool grace_spent = (bal->started != 0) && (bal->grace_ticks_left == 0);
    if (motor_braking &&
        TK::abs(speed_mps) <= standstill_speed_mps &&
        (launch_phase == LaunchPhase::hold || grace_spent)) {
        return BalanceStopCore::braked;
    }
    return BalanceStopCore::none;
}

template <class Real, class TK, class CfgPtr, class StatePtr>
BIKE_HD Status balance_monitor_update(
    CfgPtr cfg,
    i32 step,
    Real position_m,
    Real speed_mps,
    bool riding,
    BalanceStopCore stop,
    StatePtr state,
    BIKE_THREAD bool &out_balance_lost,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_balance_lost = false;
    if (!TK::isfinite(cfg->floor_mps) || cfg->floor_mps <= BIKE_REAL(Real, 0.0) ||
        cfg->dwell_ticks <= 0 || cfg->grace_ticks < 0 ||
        !TK::isfinite(cfg->period_s) || cfg->period_s <= BIKE_REAL(Real, 0.0) ||
        step < 0 || !TK::isfinite(position_m) || !TK::isfinite(speed_mps)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (state->last_step >= 0 && step < state->last_step) {
        err = ErrCode::bad_time;
        return Status::invalid_argument;
    }
    const i32 delta_steps = (state->last_step >= 0) ? (step - state->last_step) : 1;
    state->last_step = step;

    if (!riding && stop == BalanceStopCore::braked) {
        state->started = 0;
        state->grace_ticks_left = cfg->grace_ticks;
    } else if (riding && state->started == 0) {
        state->started = 1;
        state->grace_ticks_left = cfg->grace_ticks;
    } else if (state->started != 0 && delta_steps > 0) {
        const i32 rem = state->grace_ticks_left - delta_steps;
        state->grace_ticks_left = rem > 0 ? rem : 0;
    }

    if (!riding || state->grace_ticks_left > 0 || speed_mps >= cfg->floor_mps) {
        state->low_started = 0;
        state->low_speed_ticks = 0;
        state->active = 0;
        return Status::ok;
    }

    if (state->low_started == 0) {
        state->low_started = 1;
        state->low_speed_ticks = 0;
    } else if (delta_steps > 0) {
        if (state->low_speed_ticks <= kMaxI32Ticks - delta_steps) {
            state->low_speed_ticks += delta_steps;
        } else {
            state->low_speed_ticks = kMaxI32Ticks;
        }
    }

    if (state->low_speed_ticks >= cfg->dwell_ticks && state->active == 0) {
        const Real time_s = static_cast<Real>(step) * cfg->period_s;
        state->active = 1;
        state->cur_time_s = time_s;
        state->cur_position_m = position_m;
        state->cur_speed_mps = speed_mps;
        if (state->triggered == 0) {
            state->triggered = 1;
            state->event_time_s = time_s;
            state->event_position_m = position_m;
            state->event_speed_mps = speed_mps;
        }
    }
    out_balance_lost = (state->active != 0);
    return Status::ok;
}

// ---- CrashDetector ----
template <class Real>
struct CrashState {
    Real time_s = BIKE_REAL(Real, 0.0);
    Real position_m = BIKE_REAL(Real, 0.0);
    Real pitch_rad = BIKE_REAL(Real, 0.0);
    CrashCause cause = CrashCause::none;
    u8 pad[3]{};
};

template <class Real, class TK, class StatePtr>
BIKE_HD void crash_detector_update(
    CrashCause contact_crash,
    bool handlebar_in_contact,
    Real pitch_limit_rad,
    Real time_s,
    Real position_m,
    Real pitch_rad,
    StatePtr state
) noexcept {
    if (state->cause != CrashCause::none) {
        return;
    }
    if (contact_crash != CrashCause::none) {
        state->cause = contact_crash;
        state->time_s = time_s;
        state->position_m = position_m;
        state->pitch_rad = pitch_rad;
        return;
    }
    if (TK::abs(pitch_rad) > pitch_limit_rad) {
        state->cause = CrashCause::pitch_over;
        state->time_s = time_s;
        state->position_m = position_m;
        state->pitch_rad = pitch_rad;
        return;
    }
    if (handlebar_in_contact) {
        state->cause = CrashCause::handlebar_contact;
        state->time_s = time_s;
        state->position_m = position_m;
        state->pitch_rad = pitch_rad;
    }
}

// ---- StaticBrake Rollback Latch & BrakeResolution ----
template <class Real>
struct RollbackBrakeConfig {
    Real rollback_demand = BIKE_REAL(Real, 0.0);
    Real rollback_engage_mps = BIKE_REAL(Real, 0.25);
    Real rollback_release_mps = BIKE_REAL(Real, 0.10);
    u8 rollback_brake_enabled = 0;
    u8 effort_drive_mode = 1;
    u8 pad[2]{};
};

template <class Real>
struct BrakeRequestCore {
    Real caller_front = BIKE_REAL(Real, 0.0);
    Real caller_rear = BIKE_REAL(Real, 0.0);
    Real rollback_demand = BIKE_REAL(Real, 0.0);
    Real rider_front = BIKE_REAL(Real, 0.0);
    Real rider_rear = BIKE_REAL(Real, 0.0);
    u8 caller_braking = 0;
    u8 rider_present = 1;
    u8 rider_enabled = 1;
    u8 pad = 0;
};

template <class Real>
struct BrakeResolutionCore {
    Real final_front = BIKE_REAL(Real, 0.0);
    Real final_rear = BIKE_REAL(Real, 0.0);
    u8 legs_coast = 0;
    u8 motor_braking = 0;
    u8 shift_blocked = 0;
    u8 riding = 0;
};

template <class Real>
struct StaticBrakeMonitorState {
    u8 rollback_hold = 0;
    u8 latch_front = 0;
    u8 latch_rear = 0;
    u8 pad = 0;
    Real latch_front_rad = BIKE_REAL(Real, 0.0);
    Real latch_rear_rad = BIKE_REAL(Real, 0.0);
};

template <class Real, class TK, class CfgPtr, class StatePtr>
BIKE_HD Status rollback_brake_demand(
    CfgPtr cfg,
    Real speed_mps,
    bool rider_enabled,
    StatePtr state,
    BIKE_THREAD Real &out_demand,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_demand = BIKE_REAL(Real, 0.0);
    if (!TK::isfinite(speed_mps)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (cfg->rollback_brake_enabled == 0 ||
        cfg->rollback_demand <= BIKE_REAL(Real, 0.0) ||
        cfg->effort_drive_mode == 0 || !rider_enabled) {
        state->rollback_hold = 0;
        return Status::ok;
    }
    if (state->rollback_hold != 0) {
        state->rollback_hold =
            (speed_mps <= -cfg->rollback_release_mps) ? 1 : 0;
    } else {
        state->rollback_hold =
            (speed_mps < -cfg->rollback_engage_mps) ? 1 : 0;
    }
    out_demand = (state->rollback_hold != 0)
                     ? cfg->rollback_demand
                     : BIKE_REAL(Real, 0.0);
    return Status::ok;
}

template <class Real, class TK>
BIKE_HD BrakeResolutionCore<Real> resolve_brakes_core(
    BrakeRequestCore<Real> req
) noexcept {
    BrakeResolutionCore<Real> out{};
    out.final_front =
        TK::max(req.caller_front, TK::max(req.rollback_demand, req.rider_front));
    out.final_rear =
        TK::max(req.caller_rear, TK::max(req.rollback_demand, req.rider_rear));
    const bool rider_braking =
        (req.rider_front > BIKE_REAL(Real, 0.0)) ||
        (req.rider_rear > BIKE_REAL(Real, 0.0));
    out.legs_coast = req.caller_braking;
    out.motor_braking = (req.caller_braking != 0 || rider_braking) ? 1 : 0;
    out.shift_blocked = out.motor_braking;
    out.riding =
        (req.rider_present != 0 && req.rider_enabled != 0 &&
         out.final_front < BIKE_REAL(Real, 0.5) &&
         out.final_rear < BIKE_REAL(Real, 0.5) &&
         out.motor_braking == 0)
            ? 1
            : 0;
    return out;
}

template <class Real>
struct CombinedSp3MonitorsState {
    GroundedFilterState grounded[2]{};
    BridgedLoadState<Real> bridged[2]{};
    BalanceMonitorState<Real> balance{};
    CrashState<Real> crash{};
    StaticBrakeMonitorState<Real> brake{};
};

template <class Real>
struct Sp3PerWorldState {
    RiderIntentState<Real> intent{};
    CombinedSp3MonitorsState<Real> monitors{};
};

static_assert(
    sizeof(Sp3PerWorldState<float>) <= kMaxSp3StateBytesPerWorld,
    "SP3 combined per-world state (controllers + 401/120/160 rings + monitors) must stay <= 16 KiB/world"
);

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
