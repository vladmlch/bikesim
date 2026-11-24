// native/src/physics_core/pedaling.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// pedaling policy, stall detector with int32 tick counters, and 2*pi epoch re-basing.
#pragma once

#include "common.hpp"
#include "gpu_math.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

enum class PedalMode : u8 {
    disabled = 0,
    pedaling = 1,
    coasting = 2
};

enum class CoastReason : u8 {
    none = 0,
    braking = 1,
    no_effort = 2,
    cadence = 3,
    disabled = 4,
    reposition_requested = 5,
    reposition_stall = 6
};

template <class Real>
struct PedalingParams {
    Real coast_above_rpm = BIKE_REAL(Real, 120.0);
    Real resume_below_rpm = BIKE_REAL(Real, 105.0);
    Real stop_time_s = BIKE_REAL(Real, 0.35);
    Real coast_cadence_tau_s = BIKE_REAL(Real, 0.35);
    Real mash_cadence_rpm = BIKE_REAL(Real, 45.0);
    Real mash_torque_nm = BIKE_REAL(Real, 60.0);
    Real effort_slew_nm_s = BIKE_REAL(Real, 300.0);
    Real reposition_min_effort_nm = BIKE_REAL(Real, 20.0);
    i32 reposition_stall_window_ticks = 800;
    Real reposition_stall_progress_rad = BIKE_REAL(Real, 0.5);
    i32 reposition_cooldown_ticks = 800;
    Real reposition_back_rate_rad_s = BIKE_REAL(Real, 3.0);
    i32 reposition_timeout_ticks = 2400;
    Real reposition_phase_tolerance_rad = BIKE_REAL(Real, 0.05);
    Real reposition_noop_rad = BIKE_REAL(Real, 0.12);
    u8 enabled = 1;
    u8 reposition_on_stall = 0;
    u8 pad[2]{};
};

template <class Real>
struct RepositionState {
    Real crank_rad = BIKE_REAL(Real, 0.0);
    Real wheel_rad = BIKE_REAL(Real, 0.0);
    Real goal_rad = BIKE_REAL(Real, 0.0);
    i32 stall_elapsed_ticks = -1;
    i32 elapsed_ticks = 0;
    i32 cooldown_ticks = 0;
    u8 active = 0;
    u8 armed = 1;
    u8 reason = 0;
    u8 has_goal = 0;
};

template <class Real>
struct StallRepositionConfigPod {
    Real min_effort_nm = BIKE_REAL(Real, 20.0);
    Real stall_window_s = BIKE_REAL(Real, 1.0);
    Real stall_progress_rad = BIKE_REAL(Real, 0.5);
    Real cooldown_s = BIKE_REAL(Real, 1.0);
    Real back_rate_rad_s = BIKE_REAL(Real, 3.0);
    Real timeout_s = BIKE_REAL(Real, 3.0);
    Real phase_tolerance_rad = BIKE_REAL(Real, 0.05);
    Real noop_rad = BIKE_REAL(Real, 0.12);
    u8 reposition_on_stall = 0;
    u8 motor_clutch_present = 1;
    u8 spindle_pedal_attachment = 1;
    u8 pad = 0;
};

template <class Real>
struct StallRepositionStatePod {
    Real crank_rad = BIKE_REAL(Real, 0.0);
    Real wheel_rad = BIKE_REAL(Real, 0.0);
    Real stall_elapsed_s = BIKE_REAL(Real, 0.0);
    Real goal_rad = BIKE_REAL(Real, 0.0);
    Real elapsed_s = BIKE_REAL(Real, 0.0);
    Real cooldown_s = BIKE_REAL(Real, 0.0);
    u8 active = 0;
    u8 armed = 1;
    u8 reason = 0;
    u8 has_stall_elapsed = 0;
    u8 has_goal = 0;
    u8 pad[3]{};
};

template <class Real>
struct PedalingState {
    Real target_phase_rad = BIKE_REAL(Real, 0.0);
    Real target_rate_rad_s = BIKE_REAL(Real, 0.0);
    Real deceleration_rad_s2 = BIKE_REAL(Real, 0.0);
    Real effort = BIKE_REAL(Real, 0.0);
    Real cadence_ema = BIKE_REAL(Real, 0.0);
    RepositionState<Real> reposition{};
    u8 coasting = 0;
    u8 has_target_phase = 0;
    u8 has_cadence_ema = 0;
    u8 pad = 0;
};

template <class Real>
struct PedalingOutput {
    Real effort_nm = BIKE_REAL(Real, 0.0);
    Real required_cadence_rpm = BIKE_REAL(Real, 0.0);
    Real target_phase_rad = BIKE_REAL(Real, 0.0);
    Real target_rate_rad_s = BIKE_REAL(Real, 0.0);
    PedalMode mode = PedalMode::disabled;
    CoastReason reason = CoastReason::disabled;
    u8 has_target_phase = 0;
    u8 pad = 0;
};

namespace detail {

BIKE_HD float pedaling_flush_subnormal_impl(float x) noexcept {
    const u32 bits = pf_f32_to_u32(x);
    if ((bits & 0x7f800000u) == 0u) {
        return pf_u32_to_f32(bits & 0x80000000u);
    }
    return x;
}

#if !defined(__METAL_VERSION__)
BIKE_HD double pedaling_flush_subnormal_impl(double x) noexcept {
    return x;
}
#endif

BIKE_HD float pedaling_copysign_impl(float mag, float sign_src) noexcept {
    const u32 m_bits = pf_f32_to_u32(mag) & 0x7fffffffu;
    const u32 s_bits = pf_f32_to_u32(sign_src) & 0x80000000u;
    return pf_u32_to_f32(m_bits | s_bits);
}

#if !defined(__METAL_VERSION__)
BIKE_HD double pedaling_copysign_impl(double mag, double sign_src) noexcept {
    return sign_src < 0.0 ? -mag : mag;
}
#endif

BIKE_HD float pedaling_cody_waite_r(float ax, float q_f) noexcept {
    // bitwise-locked FP32 Cody-Waite pi/2 split
    const float kHalfPiHi = 1.57080078125f;
    const float kHalfPiMid = -4.410743713378906e-06f;
    const float kHalfPiLo = -4.371138828673793e-08f;
    return ((ax - q_f * kHalfPiHi) - q_f * kHalfPiMid) - q_f * kHalfPiLo;
}

#if !defined(__METAL_VERSION__)
BIKE_HD double pedaling_cody_waite_r(double ax, double q_f) noexcept {
    // bitwise-locked FP64 Cody-Waite pi/2 split
    const double kHalfPiHi = 1.5707963109016418;
    const double kHalfPiMid = 1.5893256488652696e-08;
    const double kHalfPiLo = -1.7763568394002505e-15;
    return ((ax - q_f * kHalfPiHi) - q_f * kHalfPiMid) - q_f * kHalfPiLo;
}
#endif

} // namespace detail

template <class Real>
BIKE_HD Real pedaling_flush_subnormal(Real x) noexcept {
    return detail::pedaling_flush_subnormal_impl(x);
}

template <class Real>
BIKE_HD Real pedaling_copysign(Real mag, Real sign_src) noexcept {
    return detail::pedaling_copysign_impl(mag, sign_src);
}

template <class Real, class TK>
BIKE_HD Real pedaling_cos_portable(Real x) noexcept {
    // Cody-Waite range reduction by pi/2 over strict + - * floor for 0-bit Metal/Host parity.
    const Real ax = TK::abs(x);
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>; bitwise-locked 2/pi
    const Real kTwoOverPi = BIKE_REAL(Real, 0.63661977236758134308);
    const Real q_f = TK::floor(ax * kTwoOverPi + BIKE_REAL(Real, 0.5));
    const Real q4 = q_f - BIKE_REAL(Real, 4.0) * TK::floor(q_f * BIKE_REAL(Real, 0.25));
    const u32 quadrant = static_cast<u32>(static_cast<i32>(q4)) & 3u;

    const Real r = detail::pedaling_cody_waite_r(ax, q_f);

    const Real r2 = r * r;
    Real mag = BIKE_REAL(Real, 0.0);
    if ((quadrant & 1u) == 0u) {
        Real p = BIKE_REAL(Real, -1.1470745597729725e-11);
        p = p * r2 + BIKE_REAL(Real, 2.0876756987868100e-09);
        p = p * r2 - BIKE_REAL(Real, 2.7557319223985891e-07);
        p = p * r2 + BIKE_REAL(Real, 2.4801587301587302e-05);
        p = p * r2 - BIKE_REAL(Real, 1.3888888888888889e-03);
        p = p * r2 + BIKE_REAL(Real, 4.1666666666666667e-02);
        p = p * r2 - BIKE_REAL(Real, 0.5);
        mag = BIKE_REAL(Real, 1.0) + p * r2;
    } else {
        Real p = BIKE_REAL(Real, -7.6471637318198165e-13);
        p = p * r2 + BIKE_REAL(Real, 1.6059043836821615e-10);
        p = p * r2 - BIKE_REAL(Real, 2.5052108385441719e-08);
        p = p * r2 + BIKE_REAL(Real, 2.7557319223985891e-06);
        p = p * r2 - BIKE_REAL(Real, 1.9841269841269841e-04);
        p = p * r2 + BIKE_REAL(Real, 8.3333333333333333e-03);
        p = p * r2 - BIKE_REAL(Real, 1.6666666666666667e-01);
        mag = r + r * (p * r2);
    }
    return (quadrant == 1u || quadrant == 2u) ? -mag : mag;
}

template <class Real, class TK>
BIKE_HD Status human_crank_torque(
    Real mean_nm,
    Real phase_rad,
    Real ripple,
    BIKE_THREAD Real &out_torque_nm,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_torque_nm = BIKE_REAL(Real, 0.0);
    if (!TK::isfinite(mean_nm) || !TK::isfinite(phase_rad) ||
        !TK::isfinite(ripple) || ripple < BIKE_REAL(Real, 0.0) ||
        ripple >= BIKE_REAL(Real, 1.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Real c = pedaling_cos_portable<Real, TK>(
        BIKE_REAL(Real, 2.0) * phase_rad);
    out_torque_nm = mean_nm * (BIKE_REAL(Real, 1.0) - ripple * c);
    return Status::ok;
}

template <class Real, class TK, class ParamsPtr>
BIKE_HD Status pedaling_validate_params(
    ParamsPtr params,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (params == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kPi = BIKE_REAL(Real, 3.14159265358979323846);
    if (!TK::isfinite(params->coast_above_rpm) ||
        !TK::isfinite(params->resume_below_rpm) ||
        !TK::isfinite(params->stop_time_s) ||
        !TK::isfinite(params->coast_cadence_tau_s) ||
        !TK::isfinite(params->mash_cadence_rpm) ||
        !TK::isfinite(params->mash_torque_nm) ||
        !TK::isfinite(params->effort_slew_nm_s) ||
        params->coast_above_rpm <= BIKE_REAL(Real, 0.0) ||
        params->resume_below_rpm < BIKE_REAL(Real, 0.0) ||
        params->resume_below_rpm >= params->coast_above_rpm ||
        params->stop_time_s <= BIKE_REAL(Real, 0.0) ||
        params->coast_cadence_tau_s < BIKE_REAL(Real, 0.0) ||
        params->mash_cadence_rpm < BIKE_REAL(Real, 0.0) ||
        params->mash_torque_nm < BIKE_REAL(Real, 0.0) ||
        (params->mash_torque_nm > BIKE_REAL(Real, 0.0) &&
         params->mash_cadence_rpm <= BIKE_REAL(Real, 0.0)) ||
        params->effort_slew_nm_s < BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(params->reposition_min_effort_nm) ||
        !TK::isfinite(params->reposition_stall_progress_rad) ||
        !TK::isfinite(params->reposition_back_rate_rad_s) ||
        !TK::isfinite(params->reposition_phase_tolerance_rad) ||
        !TK::isfinite(params->reposition_noop_rad) ||
        params->reposition_min_effort_nm < BIKE_REAL(Real, 0.0) ||
        params->reposition_stall_window_ticks <= 0 ||
        params->reposition_stall_progress_rad <= BIKE_REAL(Real, 0.0) ||
        params->reposition_stall_progress_rad >= kPi ||
        params->reposition_cooldown_ticks < 0 ||
        params->reposition_back_rate_rad_s <= BIKE_REAL(Real, 0.0) ||
        params->reposition_timeout_ticks <= 0 ||
        params->reposition_phase_tolerance_rad <= BIKE_REAL(Real, 0.0) ||
        params->reposition_noop_rad < BIKE_REAL(Real, 0.0) ||
        params->reposition_noop_rad >= BIKE_REAL(Real, 0.5) * kPi) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    return Status::ok;
}

template <class Real, class TK, class ReposCfgPtr>
BIKE_HD Status pedaling_validate_reposition_config(
    ReposCfgPtr cfg,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (cfg == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    // P01b: reposition_on_stall requires motor freewheel clutch (and spindle pedal attachment)
    // so automatic crank backpedaling cannot drive a rigidly coupled motor rotor backward.
    if (cfg->reposition_on_stall != 0 &&
        (cfg->motor_clutch_present == 0 || cfg->spindle_pedal_attachment == 0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kPi = BIKE_REAL(Real, 3.14159265358979323846);
    if (!TK::isfinite(cfg->min_effort_nm) ||
        !TK::isfinite(cfg->stall_window_s) ||
        !TK::isfinite(cfg->stall_progress_rad) ||
        !TK::isfinite(cfg->cooldown_s) ||
        !TK::isfinite(cfg->back_rate_rad_s) ||
        !TK::isfinite(cfg->timeout_s) ||
        !TK::isfinite(cfg->phase_tolerance_rad) ||
        !TK::isfinite(cfg->noop_rad) ||
        cfg->min_effort_nm < BIKE_REAL(Real, 0.0) ||
        cfg->stall_window_s <= BIKE_REAL(Real, 0.0) ||
        cfg->stall_progress_rad <= BIKE_REAL(Real, 0.0) ||
        cfg->stall_progress_rad >= kPi ||
        cfg->cooldown_s < BIKE_REAL(Real, 0.0) ||
        cfg->back_rate_rad_s <= BIKE_REAL(Real, 0.0) ||
        cfg->timeout_s <= BIKE_REAL(Real, 0.0) ||
        cfg->phase_tolerance_rad <= BIKE_REAL(Real, 0.0) ||
        cfg->noop_rad < BIKE_REAL(Real, 0.0) ||
        cfg->noop_rad >= BIKE_REAL(Real, 0.5) * kPi) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    return Status::ok;
}

template <class Real, class TK, class StatePtr>
BIKE_HD Status pedaling_validate_state_invariants(
    StatePtr s,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (s == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if ((s->has_target_phase != 0 && !TK::isfinite(s->target_phase_rad)) ||
        (s->has_cadence_ema != 0 && !TK::isfinite(s->cadence_ema)) ||
        !TK::isfinite(s->target_rate_rad_s) ||
        !TK::isfinite(s->deceleration_rad_s2) ||
        s->deceleration_rad_s2 < BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(s->effort) ||
        s->effort < BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (s->coasting != 0 &&
        (s->has_target_phase == 0 || s->effort != BIKE_REAL(Real, 0.0))) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (s->reposition.active != 0) {
        const RepositionState<Real> r = s->reposition;
        if (!TK::isfinite(r.crank_rad) || !TK::isfinite(r.wheel_rad) ||
            (r.has_goal != 0 && !TK::isfinite(r.goal_rad)) ||
            r.elapsed_ticks < 0 || r.cooldown_ticks < 0 || r.reason > u8{2}) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        // C2 relational invariants
        if (r.stall_elapsed_ticks < 0 &&
            (r.crank_rad != BIKE_REAL(Real, 0.0) ||
             r.wheel_rad != BIKE_REAL(Real, 0.0))) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        const bool reason_active = (r.reason == u8{1} || r.reason == u8{2});
        if ((r.has_goal != 0) != reason_active) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        if (r.has_goal != 0 &&
            (r.cooldown_ticks != 0 || s->coasting != 0 ||
             s->effort != BIKE_REAL(Real, 0.0))) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
    }
    return Status::ok;
}

template <class Real, class TK, class StatePtr, class ReposStatePtr>
BIKE_HD Status pedaling_validate_state_invariants(
    StatePtr s,
    ReposStatePtr r,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (s == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if ((s->has_target_phase != 0 && !TK::isfinite(s->target_phase_rad)) ||
        (s->has_cadence_ema != 0 && !TK::isfinite(s->cadence_ema)) ||
        !TK::isfinite(s->target_rate_rad_s) ||
        !TK::isfinite(s->deceleration_rad_s2) ||
        s->deceleration_rad_s2 < BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(s->effort) ||
        s->effort < BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (s->coasting != 0 &&
        (s->has_target_phase == 0 || s->effort != BIKE_REAL(Real, 0.0))) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (r != nullptr && r->active != 0) {
        if (!TK::isfinite(r->crank_rad) || !TK::isfinite(r->wheel_rad) ||
            (r->has_stall_elapsed != 0 &&
             (!TK::isfinite(r->stall_elapsed_s) ||
              r->stall_elapsed_s < BIKE_REAL(Real, 0.0))) ||
            (r->has_goal != 0 && !TK::isfinite(r->goal_rad)) ||
            !TK::isfinite(r->elapsed_s) || r->elapsed_s < BIKE_REAL(Real, 0.0) ||
            !TK::isfinite(r->cooldown_s) || r->cooldown_s < BIKE_REAL(Real, 0.0) ||
            r->reason > u8{2}) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        // C2 relational invariants (matching drivetrain/pedaling.cpp)
        if (r->has_stall_elapsed == 0 &&
            (r->crank_rad != BIKE_REAL(Real, 0.0) ||
             r->wheel_rad != BIKE_REAL(Real, 0.0))) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        const bool reason_active = (r->reason == u8{1} || r->reason == u8{2});
        if ((r->has_goal != 0) != reason_active) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        if (r->has_goal != 0 &&
            (r->cooldown_s != BIKE_REAL(Real, 0.0) || s->coasting != 0 ||
             s->effort != BIKE_REAL(Real, 0.0))) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
    }
    return Status::ok;
}

template <class Real, class StatePtr>
BIKE_HD void pedaling_reset(StatePtr state) noexcept {
    *state = PedalingState<Real>{};
}

// Re-base absolute crank-angle targets alongside rebase_world_rotations_2pi.
// Note: reposition.crank_rad and reposition.wheel_rad are integrated net progress
// deltas (rate * dt, reset to 0 on window start) rather than absolute qpos, so
// they remain invariant across 2*pi qpos re-basing; only absolute crank angles
// (target_phase_rad and reposition.goal_rad) shift by -dq_crank_rad.
template <class Real>
BIKE_HD void rebase_pedaling_2pi(
    BIKE_REF(PedalingState<Real>) state,
    Real dq_crank_rad,
    Real dq_wheel_rad
) noexcept {
    (void)dq_wheel_rad;
    if (state.has_target_phase != 0) {
        state.target_phase_rad -= dq_crank_rad;
    }
    if (state.reposition.active != 0 && state.reposition.has_goal != 0) {
        state.reposition.goal_rad -= dq_crank_rad;
    }
}

template <class Real>
BIKE_HD void rebase_pedaling_2pi(
    BIKE_REF(PedalingState<Real>) state,
    BIKE_REF(StallRepositionStatePod<Real>) repos,
    Real dq_crank_rad,
    Real dq_wheel_rad
) noexcept {
    rebase_pedaling_2pi<Real>(state, dq_crank_rad, dq_wheel_rad);
    if (repos.active != 0 && repos.has_goal != 0) {
        repos.goal_rad -= dq_crank_rad;
    }
}

template <class Real>
BIKE_HD bool pedaling_stall_update_seconds(
    BIKE_THREAD StallRepositionStatePod<Real> &r,
    Real window_s,
    Real progress_rad,
    Real crank_rad_s,
    Real wheel_rad_s,
    Real dt,
    bool loaded
) noexcept {
    if (!loaded) {
        r.crank_rad = BIKE_REAL(Real, 0.0);
        r.wheel_rad = BIKE_REAL(Real, 0.0);
        r.stall_elapsed_s = BIKE_REAL(Real, 0.0);
        r.has_stall_elapsed = 0;
        return false;
    }
    if (r.has_stall_elapsed == 0) {
        r.crank_rad = BIKE_REAL(Real, 0.0);
        r.wheel_rad = BIKE_REAL(Real, 0.0);
        r.stall_elapsed_s = BIKE_REAL(Real, 0.0);
        r.has_stall_elapsed = 1;
        return false;
    }
    r.crank_rad += crank_rad_s * dt;
    r.wheel_rad += wheel_rad_s * dt;
    if (r.crank_rad > progress_rad || r.wheel_rad > progress_rad) {
        r.crank_rad = BIKE_REAL(Real, 0.0);
        r.wheel_rad = BIKE_REAL(Real, 0.0);
        r.stall_elapsed_s = BIKE_REAL(Real, 0.0);
        r.has_stall_elapsed = 1;
        return false;
    }
    r.stall_elapsed_s += dt;
    if (r.stall_elapsed_s >= window_s) {
        r.crank_rad = BIKE_REAL(Real, 0.0);
        r.wheel_rad = BIKE_REAL(Real, 0.0);
        r.stall_elapsed_s = BIKE_REAL(Real, 0.0);
        r.has_stall_elapsed = 0;
        return true;
    }
    return false;
}

template <class Real>
BIKE_HD bool pedaling_stall_update(
    BIKE_THREAD RepositionState<Real> &r,
    i32 window_ticks,
    Real progress_rad,
    Real crank_rad_s,
    Real wheel_rad_s,
    Real dt,
    bool loaded
) noexcept {
    if (!loaded) {
        r.crank_rad = BIKE_REAL(Real, 0.0);
        r.wheel_rad = BIKE_REAL(Real, 0.0);
        r.stall_elapsed_ticks = -1;
        return false;
    }
    if (r.stall_elapsed_ticks < 0) {
        r.crank_rad = BIKE_REAL(Real, 0.0);
        r.wheel_rad = BIKE_REAL(Real, 0.0);
        r.stall_elapsed_ticks = 0;
        return false;
    }
    r.crank_rad += crank_rad_s * dt;
    r.wheel_rad += wheel_rad_s * dt;
    if (r.crank_rad > progress_rad || r.wheel_rad > progress_rad) {
        r.crank_rad = BIKE_REAL(Real, 0.0);
        r.wheel_rad = BIKE_REAL(Real, 0.0);
        r.stall_elapsed_ticks = 0;
        return false;
    }
    ++r.stall_elapsed_ticks;
    if (r.stall_elapsed_ticks >= window_ticks) {
        r.crank_rad = BIKE_REAL(Real, 0.0);
        r.wheel_rad = BIKE_REAL(Real, 0.0);
        r.stall_elapsed_ticks = -1;
        return true;
    }
    return false;
}

namespace detail {

template <class Real, class TK, class ParamsPtr, class StatePtr>
BIKE_HD Status pedaling_finish_step(
    ParamsPtr params,
    StatePtr state,
    BIKE_THREAD PedalingState<Real> &next,
    Real phase_rad,
    Real rate_rad_s,
    Real required_cadence_rpm,
    Real effort_nm,
    Real dt,
    bool braking,
    bool has_shaft_rad_s,
    Real shaft_rad_s,
    bool has_ceiling_nm,
    Real ceiling_nm,
    Real torque_factor,
    bool cut_after_ceiling,
    BIKE_THREAD PedalingOutput<Real> &out
) noexcept {
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kTwoPi = BIKE_REAL(Real, 6.28318530717958647692);
    const Real clamped_required =
        TK::max(required_cadence_rpm, BIKE_REAL(Real, 0.0));
    const Real actual =
        TK::abs(rate_rad_s) * BIKE_REAL(Real, 60.0) / kTwoPi;
    const Real cadence = TK::max(actual, clamped_required);
    if (params->coast_cadence_tau_s > BIKE_REAL(Real, 0.0) &&
        next.has_cadence_ema != 0) {
        const Real alpha =
            TK::min(BIKE_REAL(Real, 1.0), dt / params->coast_cadence_tau_s);
        next.cadence_ema = pedaling_flush_subnormal(
            next.cadence_ema + alpha * (actual - next.cadence_ema));
    } else {
        next.cadence_ema = actual;
        next.has_cadence_ema = 1;
    }

    const Real threshold = (next.coasting != 0)
        ? params->resume_below_rpm
        : params->coast_above_rpm;
    const bool excessive =
        (params->enabled != 0) && (next.cadence_ema >= threshold);
    const CoastReason reason =
        braking ? CoastReason::braking :
        (effort_nm == BIKE_REAL(Real, 0.0) ? CoastReason::no_effort :
        (excessive ? CoastReason::cadence : CoastReason::none));

    if (reason == CoastReason::none) {
        const Real previous_effort = next.effort;
        const Real ema = next.cadence_ema;
        const u8 has_ema = next.has_cadence_ema;
        const RepositionState<Real> repos = next.reposition;

        next = PedalingState<Real>{};
        next.cadence_ema = ema;
        next.has_cadence_ema = has_ema;
        next.reposition = repos;

        Real target = effort_nm;
        if (params->mash_torque_nm > target &&
            cadence < params->mash_cadence_rpm) {
            target = effort_nm +
                     (params->mash_torque_nm - effort_nm) *
                         (BIKE_REAL(Real, 1.0) -
                          cadence / params->mash_cadence_rpm);
        }
        if (cut_after_ceiling) {
            if (has_ceiling_nm) {
                target = TK::min(target, ceiling_nm);
            }
            target = target * torque_factor;
        } else {
            target = target * torque_factor;
            if (has_ceiling_nm) {
                target = TK::min(target, ceiling_nm);
            }
        }
        const Real slew = params->effort_slew_nm_s;
        Real final_effort = target;
        if (slew > BIKE_REAL(Real, 0.0)) {
            const Real delta = slew * dt;
            final_effort = TK::max(
                previous_effort - delta,
                TK::min(previous_effort + delta, target));
        }
        next.effort = pedaling_flush_subnormal(final_effort);
        Real wish = TK::max(
            TK::min(clamped_required, next.cadence_ema) * kTwoPi /
                BIKE_REAL(Real, 60.0),
            BIKE_REAL(Real, 0.0));
        const Real shaft = has_shaft_rad_s ? shaft_rad_s : rate_rad_s;
        if (shaft > rate_rad_s) {
            wish = TK::max(wish, shaft);
        }
        *state = next;
        out.mode = PedalMode::pedaling;
        out.reason = CoastReason::none;
        out.effort_nm = next.effort;
        out.required_cadence_rpm = clamped_required;
        out.target_phase_rad = BIKE_REAL(Real, 0.0);
        out.has_target_phase = 0;
        out.target_rate_rad_s = wish;
        return Status::ok;
    }

    if (next.coasting == 0) {
        next.deceleration_rad_s2 = TK::abs(rate_rad_s) / params->stop_time_s;
        next.target_phase_rad = phase_rad;
        next.has_target_phase = 1;
        next.target_rate_rad_s = rate_rad_s;
    }
    next.coasting = 1;
    const Real prev_rate = next.target_rate_rad_s;
    const Real next_mag = TK::max(
        BIKE_REAL(Real, 0.0),
        TK::abs(prev_rate) - next.deceleration_rad_s2 * dt);
    next.target_rate_rad_s = pedaling_copysign(next_mag, prev_rate);
    next.target_phase_rad =
        next.target_phase_rad +
        BIKE_REAL(Real, 0.5) * (prev_rate + next.target_rate_rad_s) * dt;
    next.effort = BIKE_REAL(Real, 0.0);
    *state = next;

    out.mode = PedalMode::coasting;
    out.reason = reason;
    out.effort_nm = BIKE_REAL(Real, 0.0);
    out.required_cadence_rpm = clamped_required;
    out.target_phase_rad = next.target_phase_rad;
    out.has_target_phase = 1;
    out.target_rate_rad_s = next.target_rate_rad_s;
    return Status::ok;
}

} // namespace detail

template <class Real, class TK, class ParamsPtr, class StatePtr>
BIKE_HD Status pedaling_update(
    ParamsPtr params,
    StatePtr state,
    Real phase_rad,
    Real rate_rad_s,
    Real required_cadence_rpm,
    Real effort_nm,
    Real dt,
    bool braking,
    bool reposition_request,
    bool has_shaft_rad_s,
    Real shaft_rad_s,
    bool has_ceiling_nm,
    Real ceiling_nm,
    Real torque_factor,
    bool cut_after_ceiling,
    BIKE_THREAD PedalingOutput<Real> &out,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out = PedalingOutput<Real>{};
    if (state == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Status p_st = pedaling_validate_params<Real, TK>(params, err);
    if (p_st != Status::ok) {
        return p_st;
    }
    if (!TK::isfinite(phase_rad) || !TK::isfinite(rate_rad_s) ||
        !TK::isfinite(required_cadence_rpm) ||
        (has_shaft_rad_s && !TK::isfinite(shaft_rad_s)) ||
        (has_ceiling_nm && (!TK::isfinite(ceiling_nm) || ceiling_nm < BIKE_REAL(Real, 0.0))) ||
        !TK::isfinite(torque_factor) || torque_factor < BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(effort_nm) || effort_nm < BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(dt) || dt <= BIKE_REAL(Real, 0.0)) {
        err = (!TK::isfinite(dt) || dt <= BIKE_REAL(Real, 0.0))
            ? ErrCode::bad_dt
            : ErrCode::outside_domain;
        return Status::invalid_argument;
    }

    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kPi = BIKE_REAL(Real, 3.14159265358979323846);
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kTwoPi = BIKE_REAL(Real, 6.28318530717958647692);
    const Real wheel_rad_s = required_cadence_rpm * kTwoPi / BIKE_REAL(Real, 60.0);

    if (params->enabled == 0) {
        pedaling_reset<Real>(state);
        if (reposition_request) {
            state->reposition.active = 1;
            state->reposition.armed = 0;
        }
        out.mode = PedalMode::disabled;
        out.reason = CoastReason::disabled;
        out.effort_nm = BIKE_REAL(Real, 0.0);
        out.required_cadence_rpm = TK::max(required_cadence_rpm, BIKE_REAL(Real, 0.0));
        out.has_target_phase = 0;
        out.target_phase_rad = BIKE_REAL(Real, 0.0);
        out.target_rate_rad_s = BIKE_REAL(Real, 0.0);
        return Status::ok;
    }

    PedalingState<Real> next = *state;
    const bool tracked =
        (params->reposition_on_stall != 0) || reposition_request ||
        (next.reposition.active != 0 &&
         (next.reposition.has_goal != 0 || next.reposition.cooldown_ticks > 0));
    if (tracked && next.reposition.active == 0) {
        next.reposition = RepositionState<Real>{};
        next.reposition.active = 1;
        next.reposition.armed = 1;
    }
    if (next.reposition.active != 0) {
        BIKE_THREAD RepositionState<Real> &r = next.reposition;
        if (r.cooldown_ticks > 0) {
            --r.cooldown_ticks;
        }
        if (braking) {
            r.has_goal = 0;
            r.goal_rad = BIKE_REAL(Real, 0.0);
            r.reason = 0;
            r.cooldown_ticks = params->reposition_cooldown_ticks;
        }
        bool request_edge = false;
        if (reposition_request) {
            request_edge = (r.armed != 0);
            r.armed = 0;
        } else {
            r.armed = 1;
        }
        const bool loaded = (params->reposition_on_stall != 0) && !braking &&
                            (r.has_goal == 0) && (r.cooldown_ticks <= 0) &&
                            (effort_nm >= params->reposition_min_effort_nm);
        const bool reflex_edge = pedaling_stall_update<Real>(
            r, params->reposition_stall_window_ticks,
            params->reposition_stall_progress_rad, rate_rad_s, wheel_rad_s, dt,
            loaded);
        for (i32 pass = 0; pass < 2; ++pass) {
            const bool edge = (pass == 0) ? request_edge : reflex_edge;
            const u8 why = (pass == 0) ? u8{1} : u8{2};
            if (braking || !edge || r.has_goal != 0 || r.cooldown_ticks > 0) {
                continue;
            }
            Real turn = phase_rad - TK::floor(phase_rad / kPi) * kPi;
            if (turn < BIKE_REAL(Real, 0.0)) {
                turn += kPi;
            }
            if (turn >= kPi) {
                turn -= kPi;
            }
            if (TK::min(turn, kPi - turn) <= params->reposition_noop_rad) {
                continue;
            }
            r.goal_rad = phase_rad - turn;
            r.has_goal = 1;
            r.reason = why;
            r.elapsed_ticks = 0;
            next.effort = BIKE_REAL(Real, 0.0);
            next.coasting = 0;
            break;
        }
        if (r.has_goal != 0) {
            const Real remaining = phase_rad - r.goal_rad;
            ++r.elapsed_ticks;
            if (remaining <= params->reposition_phase_tolerance_rad ||
                r.elapsed_ticks > params->reposition_timeout_ticks) {
                r.has_goal = 0;
                r.goal_rad = BIKE_REAL(Real, 0.0);
                r.reason = 0;
                r.cooldown_ticks = params->reposition_cooldown_ticks;
            } else {
                const Real kApproachTauS = BIKE_REAL(Real, 0.05);
                const Real target_rate = -TK::min(
                    params->reposition_back_rate_rad_s,
                    remaining / kApproachTauS);
                *state = next;
                out.mode = PedalMode::coasting;
                out.reason = (r.reason == 1)
                    ? CoastReason::reposition_requested
                    : CoastReason::reposition_stall;
                out.effort_nm = BIKE_REAL(Real, 0.0);
                out.required_cadence_rpm =
                    TK::max(required_cadence_rpm, BIKE_REAL(Real, 0.0));
                out.target_phase_rad = phase_rad + target_rate * dt;
                out.has_target_phase = 1;
                out.target_rate_rad_s = target_rate;
                return Status::ok;
            }
        }
        if (params->reposition_on_stall == 0 && r.has_goal == 0 &&
            r.cooldown_ticks <= 0 && r.armed != 0) {
            r = RepositionState<Real>{};
            r.active = 0;
            r.armed = 1;
        }
    }

    return detail::pedaling_finish_step<Real, TK>(
        params, state, next, phase_rad, rate_rad_s, required_cadence_rpm,
        effort_nm, dt, braking, has_shaft_rad_s, shaft_rad_s, has_ceiling_nm,
        ceiling_nm, torque_factor, cut_after_ceiling, out);
}

template <class Real, class TK, class ParamsPtr, class StatePtr,
          class ReposCfgPtr, class ReposStatePtr>
BIKE_HD Status pedaling_step_with_reposition(
    ParamsPtr params,
    StatePtr state,
    ReposCfgPtr repos_cfg,
    ReposStatePtr repos_state,
    Real phase_rad,
    Real rate_rad_s,
    Real required_cadence_rpm,
    Real effort_nm,
    Real dt,
    bool braking,
    bool reposition_request,
    bool has_shaft_rad_s,
    Real shaft_rad_s,
    bool has_ceiling_nm,
    Real ceiling_nm,
    Real torque_factor,
    bool cut_after_ceiling,
    BIKE_THREAD PedalingOutput<Real> &out,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out = PedalingOutput<Real>{};
    if (state == nullptr || repos_state == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Status p_st = pedaling_validate_params<Real, TK>(params, err);
    if (p_st != Status::ok) {
        return p_st;
    }
    const Status rc_st =
        pedaling_validate_reposition_config<Real, TK>(repos_cfg, err);
    if (rc_st != Status::ok) {
        return rc_st;
    }
    const Status inv_st =
        pedaling_validate_state_invariants<Real, TK>(state, repos_state, err);
    if (inv_st != Status::ok) {
        return inv_st;
    }
    if (!TK::isfinite(phase_rad) || !TK::isfinite(rate_rad_s) ||
        !TK::isfinite(required_cadence_rpm) ||
        (has_shaft_rad_s && !TK::isfinite(shaft_rad_s)) ||
        (has_ceiling_nm &&
         (!TK::isfinite(ceiling_nm) || ceiling_nm < BIKE_REAL(Real, 0.0))) ||
        !TK::isfinite(torque_factor) || torque_factor < BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(effort_nm) || effort_nm < BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(dt) || dt <= BIKE_REAL(Real, 0.0)) {
        err = (!TK::isfinite(dt) || dt <= BIKE_REAL(Real, 0.0))
            ? ErrCode::bad_dt
            : ErrCode::outside_domain;
        return Status::invalid_argument;
    }

    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kPi = BIKE_REAL(Real, 3.14159265358979323846);
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kTwoPi = BIKE_REAL(Real, 6.28318530717958647692);
    const Real wheel_rad_s = required_cadence_rpm * kTwoPi / BIKE_REAL(Real, 60.0);

    if (params->enabled == 0) {
        pedaling_reset<Real>(state);
        *repos_state = StallRepositionStatePod<Real>{};
        if (reposition_request) {
            repos_state->active = 1;
            repos_state->armed = 0;
        }
        out.mode = PedalMode::disabled;
        out.reason = CoastReason::disabled;
        out.effort_nm = BIKE_REAL(Real, 0.0);
        out.required_cadence_rpm = TK::max(required_cadence_rpm, BIKE_REAL(Real, 0.0));
        out.has_target_phase = 0;
        out.target_phase_rad = BIKE_REAL(Real, 0.0);
        out.target_rate_rad_s = BIKE_REAL(Real, 0.0);
        return Status::ok;
    }

    PedalingState<Real> next = *state;
    StallRepositionStatePod<Real> r = *repos_state;
    const bool tracked =
        (repos_cfg->reposition_on_stall != 0) || reposition_request ||
        (r.active != 0 &&
         (r.has_goal != 0 || r.cooldown_s > BIKE_REAL(Real, 0.0)));
    if (tracked && r.active == 0) {
        r = StallRepositionStatePod<Real>{};
        r.active = 1;
        r.armed = 1;
    }
    if (r.active != 0) {
        r.cooldown_s = TK::max(BIKE_REAL(Real, 0.0), r.cooldown_s - dt);
        if (braking) {
            r.has_goal = 0;
            r.goal_rad = BIKE_REAL(Real, 0.0);
            r.reason = 0;
            r.cooldown_s = repos_cfg->cooldown_s;
        }
        bool request_edge = false;
        if (reposition_request) {
            request_edge = (r.armed != 0);
            r.armed = 0;
        } else {
            r.armed = 1;
        }
        const bool loaded =
            (repos_cfg->reposition_on_stall != 0) && !braking &&
            (r.has_goal == 0) && (r.cooldown_s <= BIKE_REAL(Real, 0.0)) &&
            (effort_nm >= repos_cfg->min_effort_nm);
        const bool reflex_edge = pedaling_stall_update_seconds<Real>(
            r, repos_cfg->stall_window_s, repos_cfg->stall_progress_rad,
            rate_rad_s, wheel_rad_s, dt, loaded);
        for (i32 pass = 0; pass < 2; ++pass) {
            const bool edge = (pass == 0) ? request_edge : reflex_edge;
            const u8 why = (pass == 0) ? u8{1} : u8{2};
            if (braking || !edge || r.has_goal != 0 ||
                r.cooldown_s > BIKE_REAL(Real, 0.0)) {
                continue;
            }
            Real turn = phase_rad - TK::floor(phase_rad / kPi) * kPi;
            if (turn < BIKE_REAL(Real, 0.0)) {
                turn += kPi;
            }
            if (turn >= kPi) {
                turn -= kPi;
            }
            if (TK::min(turn, kPi - turn) <= repos_cfg->noop_rad) {
                continue;
            }
            r.goal_rad = phase_rad - turn;
            r.has_goal = 1;
            r.reason = why;
            r.elapsed_s = BIKE_REAL(Real, 0.0);
            next.effort = BIKE_REAL(Real, 0.0);
            next.coasting = 0;
            break;
        }
        if (r.has_goal != 0) {
            const Real remaining = phase_rad - r.goal_rad;
            r.elapsed_s += dt;
            if (remaining <= repos_cfg->phase_tolerance_rad ||
                r.elapsed_s > repos_cfg->timeout_s) {
                r.has_goal = 0;
                r.goal_rad = BIKE_REAL(Real, 0.0);
                r.reason = 0;
                r.cooldown_s = repos_cfg->cooldown_s;
            } else {
                const Real kApproachTauS = BIKE_REAL(Real, 0.05);
                const Real target_rate = -TK::min(
                    repos_cfg->back_rate_rad_s, remaining / kApproachTauS);
                *state = next;
                *repos_state = r;
                out.mode = PedalMode::coasting;
                out.reason = (r.reason == 1)
                    ? CoastReason::reposition_requested
                    : CoastReason::reposition_stall;
                out.effort_nm = BIKE_REAL(Real, 0.0);
                out.required_cadence_rpm =
                    TK::max(required_cadence_rpm, BIKE_REAL(Real, 0.0));
                out.target_phase_rad = phase_rad + target_rate * dt;
                out.has_target_phase = 1;
                out.target_rate_rad_s = target_rate;
                return Status::ok;
            }
        }
        if (repos_cfg->reposition_on_stall == 0 && r.has_goal == 0 &&
            r.cooldown_s <= BIKE_REAL(Real, 0.0) && r.armed != 0) {
            r = StallRepositionStatePod<Real>{};
            r.active = 0;
            r.armed = 1;
        }
    }

    *repos_state = r;
    return detail::pedaling_finish_step<Real, TK>(
        params, state, next, phase_rad, rate_rad_s, required_cadence_rpm,
        effort_nm, dt, braking, has_shaft_rad_s, shaft_rad_s, has_ceiling_nm,
        ceiling_nm, torque_factor, cut_after_ceiling, out);
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
