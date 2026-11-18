// native/src/physics_core/launch_plan.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// autonomous standstill launch plan state machine, piecewise-linear launch config tables,
// and shared rider senses / percepts POD structures (zero double, zero std::, zero heap allocs).
#pragma once

#include "common.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: POD structs and templated pointers match the CUDA/MSL ABI without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

BIKE_CONSTEXPR i32 kMaxLaunchKnots = 16;
BIKE_CONSTEXPR i32 kMaxI32Ticks = 0x7fffffff;

enum class SurfaceLabel : u8 {
    asphalt = 0,
    hardpack = 1,
    loose = 2,
    wet = 3,
    scree = 4
};

enum class LaunchPhase : u8 {
    idle = 0,
    hold = 1,
    release = 2,
    accelerate = 3,
    handover = 4,
    blend = 5,
    done = 6
};

enum class ShiftResponse : u8 {
    none = 0,
    accepted = 1,
    deferred = 2,
    rejected = 3
};

enum class ShiftRequest : u8 {
    none = 0,
    up = 1,
    down = 2
};

template <class Real>
struct IntentSignalsCore {
    Real pitch_rate_up_rad_s = BIKE_REAL(Real, 0.0);
    Real specific_force_body_mps2[3]{};
    Real crank_rate_rad_s = BIKE_REAL(Real, 0.0);
    Real human_crank_torque_nm = BIKE_REAL(Real, 0.0);
    Real front_load_share = BIKE_REAL(Real, 0.0);
    u8 has_front_load_share = 0;
    u8 pad[3]{};
};

template <class Real>
struct RiderSensesCore {
    Real hand_press_n = BIKE_REAL(Real, 0.0);
    Real speed_mps = BIKE_REAL(Real, 0.0);
    Real grade_ahead = BIKE_REAL(Real, 0.0);
    i32 rear_teeth = 3;
    SurfaceLabel surface_front = SurfaceLabel::hardpack;
    SurfaceLabel surface_ahead = SurfaceLabel::hardpack;
    u8 pad[2]{};
};

template <class Real>
struct SenseSampleCore {
    IntentSignalsCore<Real> signals{};
    RiderSensesCore<Real> senses{};
};

template <class Real>
struct PerceptsCore {
    RiderSensesCore<Real> senses{};
    Real crank_rpm = BIKE_REAL(Real, 0.0);
    Real human_crank_torque_nm = BIKE_REAL(Real, 0.0);
    Real pitch_rate_up_rad_s = BIKE_REAL(Real, 0.0);
    Real pitch_rad = BIKE_REAL(Real, 0.0);
};

struct ShiftFeedbackCore {
    i32 shift_count = 0;
    ShiftResponse response = ShiftResponse::none;
    u8 cut_active = 0;
    u8 pad[2]{};
};

template <class Real>
struct LaunchParams {
    i32 start_cog_count = 0;
    Real start_cog_grade_breaks[kMaxLaunchKnots]{};
    i32 start_cog_teeth[kMaxLaunchKnots]{};

    Real start_pedal_angle_rad = BIKE_REAL(Real, 0.0);

    i32 start_lean_count = 0;
    Real start_lean_grades[kMaxLaunchKnots]{};
    Real start_lean_rad[kMaxLaunchKnots]{};

    u8 plan_enabled = 0;
    u8 pad[3]{};

    Real standstill_speed_mps = BIKE_REAL(Real, 0.0);
    Real standstill_crank_rpm = BIKE_REAL(Real, 0.0);
    Real effort_rate_nm_s = BIKE_REAL(Real, 0.0);

    Real effort_cap_asphalt_nm = BIKE_REAL(Real, 0.0);
    Real effort_cap_hardpack_nm = BIKE_REAL(Real, 0.0);
    Real effort_cap_loose_nm = BIKE_REAL(Real, 0.0);
    Real effort_cap_wet_nm = BIKE_REAL(Real, 0.0);
    Real effort_cap_scree_nm = BIKE_REAL(Real, 0.0);

    i32 effort_cap_count = 0;
    Real effort_cap_grades[kMaxLaunchKnots]{};
    Real effort_cap_grade_nm[kMaxLaunchKnots]{};
    Real effort_rise_speed_mps = BIKE_REAL(Real, 0.0);

    i32 release_torque_count = 0;
    Real release_torque_grades[kMaxLaunchKnots]{};
    Real release_torque_nm[kMaxLaunchKnots]{};

    Real upshift_crank_rpm = BIKE_REAL(Real, 0.0);
    Real end_speed_mps = BIKE_REAL(Real, 0.0);
    Real end_crank_rpm = BIKE_REAL(Real, 0.0);

    Real period_s = BIKE_REAL(Real, 0.0);
    i32 release_ticks = 0;
    i32 max_hold_ticks = 0;
    i32 end_hold_ticks = 0;
    i32 max_launch_ticks = 0;
    i32 handover_ticks = 0;
    i32 retry_ticks = 0;
};

template <class Real>
struct LaunchState {
    i32 phase_ticks = 0;
    i32 launch_ticks = 0;
    i32 end_ok_ticks = 0;
    i32 blend_ticks = 0;
    i32 request_count = 0;
    i32 request_teeth = 0;
    i32 blocked_teeth = 0;
    i32 rise_ticks = -1;
    i32 retry_tick = -1;
    Real effort_nm = BIKE_REAL(Real, 0.0);
    Real brake = BIKE_REAL(Real, 0.0);
    Real lean_target_rad = BIKE_REAL(Real, 0.0);
    LaunchPhase phase = LaunchPhase::idle;
    u8 shift_up = 0;
    u8 preload_missed = 0;
    u8 pad = 0;
};

template <class Real>
struct LaunchCommandCore {
    Real brake = BIKE_REAL(Real, 0.0);
    Real effort_nm = BIKE_REAL(Real, 0.0);
    Real weight = BIKE_REAL(Real, 0.0);
    Real lean_target_rad = BIKE_REAL(Real, 0.0);
    LaunchPhase phase = LaunchPhase::idle;
    u8 shift_up = 0;
    u8 owns_shifting = 0;
    u8 preload_missed = 0;
};

template <class Real, class TK, class PtrKnots, class PtrValues>
BIKE_HD Real piecewise_linear_knot(
    i32 count,
    PtrKnots knots,
    PtrValues values,
    Real x
) noexcept {
    if (count <= 0) {
        return BIKE_REAL(Real, 0.0);
    }
    if (count == 1 || x >= knots[count - 1]) {
        return values[count - 1];
    }
    if (!(x > knots[0])) {
        return values[0];
    }
    i32 lo = 0;
    i32 hi = count - 1;
    while (lo + 1 < hi) {
        const i32 mid = lo + (hi - lo) / 2;
        if (knots[mid] <= x) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    const Real denom = knots[hi] - knots[lo];
    if (denom <= BIKE_REAL(Real, 0.0)) {
        return values[lo];
    }
    const Real t = (x - knots[lo]) / denom;
    return values[lo] + t * (values[hi] - values[lo]);
}

template <class Real, class CfgPtr>
BIKE_HD i32 launch_teeth_unchecked(
    CfgPtr cfg,
    Real grade
) noexcept {
    const i32 count = cfg->start_cog_count;
    if (count <= 0) {
        return 0;
    }
    i32 idx = 0;
    while (idx + 1 < count && cfg->start_cog_grade_breaks[idx] <= grade) {
        ++idx;
    }
    return cfg->start_cog_teeth[idx];
}

template <class Real, class TK, class CfgPtr>
BIKE_HD Real launch_start_lean_unchecked(
    CfgPtr cfg,
    Real grade
) noexcept {
    return piecewise_linear_knot<Real, TK>(
        cfg->start_lean_count,
        cfg->start_lean_grades,
        cfg->start_lean_rad,
        grade
    );
}

template <class Real, class TK, class CfgPtr>
BIKE_HD Real launch_release_torque_unchecked(
    CfgPtr cfg,
    Real grade
) noexcept {
    return piecewise_linear_knot<Real, TK>(
        cfg->release_torque_count,
        cfg->release_torque_grades,
        cfg->release_torque_nm,
        grade
    );
}

template <class Real, class TK, class CfgPtr>
BIKE_HD Real launch_grade_cap_unchecked(
    CfgPtr cfg,
    Real grade
) noexcept {
    return piecewise_linear_knot<Real, TK>(
        cfg->effort_cap_count,
        cfg->effort_cap_grades,
        cfg->effort_cap_grade_nm,
        grade
    );
}

template <class Real, class CfgPtr>
BIKE_HD Real launch_surface_cap(
    CfgPtr cfg,
    SurfaceLabel surface
) noexcept {
    switch (surface) {
    case SurfaceLabel::asphalt:
        return cfg->effort_cap_asphalt_nm;
    case SurfaceLabel::hardpack:
        return cfg->effort_cap_hardpack_nm;
    case SurfaceLabel::loose:
        return cfg->effort_cap_loose_nm;
    case SurfaceLabel::wet:
        return cfg->effort_cap_wet_nm;
    case SurfaceLabel::scree:
        return cfg->effort_cap_scree_nm;
    }
    return cfg->effort_cap_hardpack_nm;
}

template <class Real, class StatePtr>
BIKE_HD void launch_reset(StatePtr state) noexcept {
    *state = LaunchState<Real>{};
}

template <class Real, class CfgPtr, class StatePtr>
BIKE_HD LaunchCommandCore<Real> launch_command(
    CfgPtr cfg,
    StatePtr state
) noexcept {
    LaunchCommandCore<Real> cmd{};
    cmd.phase = state->phase;
    cmd.brake = state->brake;
    cmd.effort_nm = state->effort_nm;
    cmd.lean_target_rad = state->lean_target_rad;
    cmd.shift_up = state->shift_up;
    cmd.preload_missed = state->preload_missed;
    switch (state->phase) {
    case LaunchPhase::idle:
    case LaunchPhase::done:
        cmd.weight = BIKE_REAL(Real, 0.0);
        break;
    case LaunchPhase::blend:
        if (cfg->handover_ticks > 0) {
            cmd.weight = BIKE_REAL(Real, 1.0) -
                         static_cast<Real>(state->blend_ticks) /
                             static_cast<Real>(cfg->handover_ticks);
        } else {
            cmd.weight = BIKE_REAL(Real, 0.0);
        }
        break;
    case LaunchPhase::hold:
    case LaunchPhase::release:
    case LaunchPhase::accelerate:
    case LaunchPhase::handover:
        cmd.weight = BIKE_REAL(Real, 1.0);
        cmd.owns_shifting = 1;
        break;
    }
    return cmd;
}

template <class Real, class TK, class CfgPtr, class StatePtr>
BIKE_HD Status launch_arm_start(
    CfgPtr cfg,
    Real lean_rad,
    StatePtr state,
    BIKE_THREAD bool &out_armed,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_armed = false;
    if (cfg->plan_enabled == 0) {
        return Status::ok;
    }
    if (!TK::isfinite(lean_rad)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    LaunchState<Real> n{};
    n.phase = LaunchPhase::hold;
    n.brake = BIKE_REAL(Real, 1.0);
    n.lean_target_rad = lean_rad;
    *state = n;
    out_armed = true;
    return Status::ok;
}

template <class Real, class TK, class CfgPtr, class PerceptsPtr>
BIKE_HD Status launch_validate_inputs(
    CfgPtr cfg,
    bool has_percepts,
    PerceptsPtr percepts,
    i32 shift_count,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (shift_count < 0) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (has_percepts) {
        if (!TK::isfinite(percepts->senses.speed_mps) ||
            !TK::isfinite(percepts->senses.grade_ahead) ||
            !TK::isfinite(percepts->crank_rpm) ||
            !TK::isfinite(percepts->human_crank_torque_nm)) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
    }
    if (cfg->plan_enabled != 0) {
        if (!TK::isfinite(cfg->period_s) || cfg->period_s <= BIKE_REAL(Real, 0.0) ||
            cfg->max_hold_ticks <= 0 || cfg->max_launch_ticks <= 0 ||
            cfg->release_ticks < 0 || cfg->end_hold_ticks < 0 ||
            cfg->handover_ticks < 0 || cfg->retry_ticks <= 0 ||
            cfg->start_cog_count <= 0 || cfg->start_cog_count > kMaxLaunchKnots ||
            cfg->start_lean_count <= 0 || cfg->start_lean_count > kMaxLaunchKnots ||
            cfg->effort_cap_count <= 0 || cfg->effort_cap_count > kMaxLaunchKnots ||
            cfg->release_torque_count <= 0 || cfg->release_torque_count > kMaxLaunchKnots) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
    }
    return Status::ok;
}

namespace detail {

template <class Real, class TK, class CfgPtr, class PerceptsPtr>
BIKE_HD Real launch_ramp(
    CfgPtr cfg,
    Real effort,
    PerceptsPtr p,
    BIKE_THREAD LaunchState<Real> &n
) noexcept {
    const Real surface = launch_surface_cap<Real>(cfg, p->senses.surface_front);
    const Real grade = launch_grade_cap_unchecked<Real, TK>(cfg, p->senses.grade_ahead);
    Real ceiling = TK::min(surface, grade);
    if (p->senses.speed_mps >= cfg->effort_rise_speed_mps) {
        if (n.rise_ticks < 0) {
            n.rise_ticks = 0;
        }
        ceiling = TK::min(
            surface,
            grade + cfg->effort_rate_nm_s * static_cast<Real>(n.rise_ticks) * cfg->period_s
        );
    }
    return TK::min(effort + cfg->effort_rate_nm_s * cfg->period_s, ceiling);
}

template <class Real, class TK, class CfgPtr, class PerceptsPtr>
BIKE_HD bool launch_standstill(
    CfgPtr cfg,
    PerceptsPtr p
) noexcept {
    return TK::abs(p->senses.speed_mps) <= cfg->standstill_speed_mps &&
           p->crank_rpm <= cfg->standstill_crank_rpm;
}

template <class Real, class CfgPtr>
BIKE_HD void launch_to_release(
    CfgPtr cfg,
    BIKE_THREAD LaunchState<Real> &n
) noexcept {
    n.phase = LaunchPhase::release;
    n.phase_ticks = 0;
    if (cfg->release_ticks == 0) {
        n.brake = BIKE_REAL(Real, 0.0);
        n.phase = LaunchPhase::accelerate;
    }
}

template <class Real, class CfgPtr, class PerceptsPtr>
BIKE_HD void launch_shifting(
    CfgPtr cfg,
    BIKE_THREAD LaunchState<Real> &n,
    PerceptsPtr p,
    ShiftFeedbackCore fb
) noexcept {
    if (n.shift_up != 0 &&
        (fb.shift_count > n.request_count || fb.response == ShiftResponse::rejected)) {
        n.shift_up = 0;
        n.blocked_teeth = n.request_teeth;
        n.retry_tick = (fb.response == ShiftResponse::rejected)
                           ? (n.launch_ticks + cfg->retry_ticks)
                           : -1;
    }
    if (n.shift_up == 0 && fb.cut_active == 0 &&
        p->crank_rpm >= cfg->upshift_crank_rpm &&
        (p->senses.rear_teeth != n.blocked_teeth ||
         (n.retry_tick >= 0 && n.launch_ticks >= n.retry_tick))) {
        n.shift_up = 1;
        n.request_count = fb.shift_count;
        n.request_teeth = p->senses.rear_teeth;
        n.retry_tick = -1;
    }
}

} // namespace detail

template <class Real, class TK, class CfgPtr, class StatePtr, class PerceptsPtr>
BIKE_HD Status launch_update(
    CfgPtr cfg,
    bool has_percepts,
    PerceptsPtr percepts,
    ShiftFeedbackCore shift,
    bool eligible,
    bool external_brake,
    StatePtr state,
    BIKE_THREAD LaunchCommandCore<Real> &out_cmd,
    BIKE_THREAD ErrCode &err
) noexcept {
    const Status vst = launch_validate_inputs<Real, TK>(
        cfg, has_percepts, percepts, shift.shift_count, err);
    if (vst != Status::ok) {
        out_cmd = LaunchCommandCore<Real>{};
        return vst;
    }
    if (cfg->plan_enabled == 0) {
        out_cmd = launch_command<Real>(cfg, state);
        return Status::ok;
    }
    if (!eligible) {
        *state = LaunchState<Real>{};
        out_cmd = launch_command<Real>(cfg, state);
        return Status::ok;
    }

    LaunchState<Real> n = *state;
    if (n.rise_ticks >= 0) {
        if (n.rise_ticks >= kMaxI32Ticks) {
            err = ErrCode::overflow_nonfinite;
            out_cmd = LaunchCommandCore<Real>{};
            return Status::overflow;
        }
        ++n.rise_ticks;
    }

    switch (n.phase) {
    case LaunchPhase::idle:
    case LaunchPhase::done:
        if (has_percepts && detail::launch_standstill<Real, TK>(cfg, percepts)) {
            const Real lean_rad =
                launch_start_lean_unchecked<Real, TK>(cfg, percepts->senses.grade_ahead);
            n = LaunchState<Real>{};
            n.phase = LaunchPhase::hold;
            n.brake = BIKE_REAL(Real, 1.0);
            n.lean_target_rad = lean_rad;
        }
        break;
    case LaunchPhase::hold:
        if (!has_percepts || external_brake) {
            break;
        }
        ++n.phase_ticks;
        n.effort_nm = detail::launch_ramp<Real, TK>(cfg, n.effort_nm, percepts, n);
        if (percepts->human_crank_torque_nm >=
            launch_release_torque_unchecked<Real, TK>(cfg, percepts->senses.grade_ahead)) {
            detail::launch_to_release<Real>(cfg, n);
        } else if (n.phase_ticks >= cfg->max_hold_ticks) {
            n.preload_missed = 1;
            detail::launch_to_release<Real>(cfg, n);
        }
        break;
    case LaunchPhase::release:
        ++n.phase_ticks;
        ++n.launch_ticks;
        if (has_percepts) {
            n.effort_nm = detail::launch_ramp<Real, TK>(cfg, n.effort_nm, percepts, n);
        }
        if (n.phase_ticks >= cfg->release_ticks) {
            n.brake = BIKE_REAL(Real, 0.0);
        } else {
            n.brake = TK::max(
                BIKE_REAL(Real, 0.0),
                BIKE_REAL(Real, 1.0) -
                    static_cast<Real>(n.phase_ticks) / static_cast<Real>(cfg->release_ticks)
            );
        }
        if (n.brake == BIKE_REAL(Real, 0.0)) {
            n.phase = LaunchPhase::accelerate;
            n.phase_ticks = 0;
        }
        break;
    case LaunchPhase::accelerate:
        ++n.launch_ticks;
        if (has_percepts) {
            n.effort_nm = detail::launch_ramp<Real, TK>(cfg, n.effort_nm, percepts, n);
            detail::launch_shifting<Real>(cfg, n, percepts, shift);
            const bool end_ok =
                percepts->senses.speed_mps >= cfg->end_speed_mps &&
                percepts->crank_rpm >= cfg->end_crank_rpm;
            if (end_ok) {
                const i32 next_ok = n.end_ok_ticks + 1;
                const i32 cap_ok = cfg->end_hold_ticks + 1;
                n.end_ok_ticks = next_ok < cap_ok ? next_ok : cap_ok;
            } else {
                n.end_ok_ticks = 0;
            }
        }
        if (n.end_ok_ticks > cfg->end_hold_ticks || n.launch_ticks >= cfg->max_launch_ticks) {
            n.phase = LaunchPhase::handover;
            n.shift_up = 0;
        }
        break;
    case LaunchPhase::handover:
        if (has_percepts) {
            n.effort_nm = detail::launch_ramp<Real, TK>(cfg, n.effort_nm, percepts, n);
        }
        if (shift.cut_active == 0) {
            n.phase = LaunchPhase::blend;
            n.blend_ticks = 0;
            if (cfg->handover_ticks == 0) {
                n = LaunchState<Real>{};
                n.phase = LaunchPhase::done;
            }
        }
        break;
    case LaunchPhase::blend:
        if (has_percepts) {
            n.effort_nm = detail::launch_ramp<Real, TK>(cfg, n.effort_nm, percepts, n);
        }
        ++n.blend_ticks;
        if (n.blend_ticks >= cfg->handover_ticks) {
            n = LaunchState<Real>{};
            n.phase = LaunchPhase::done;
        }
        break;
    }

    *state = n;
    out_cmd = launch_command<Real>(cfg, state);
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
