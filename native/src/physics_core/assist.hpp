// native/src/physics_core/assist.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// e-bike motor assist controller with int32 release-hold tick counter and 100% mechanical efficiency.
#pragma once

#include "common.hpp"
#include "gpu_math.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

BIKE_CONSTEXPR i32 kMaxAssistTorqueCurvePoints = 8;

enum class ProfileMode : u8 {
    eco = 0,
    tour = 1,
    emtb = 2,
    turbo = 3
};

template <class Real>
struct MotorProfileParams {
    Real eco = BIKE_REAL(Real, 0.6);
    Real tour = BIKE_REAL(Real, 1.4);
    Real emtb_low = BIKE_REAL(Real, 1.4);
    Real emtb_high = BIKE_REAL(Real, 3.4);
    Real turbo = BIKE_REAL(Real, 3.4);
    Real emtb_full_gain_at_nm = BIKE_REAL(Real, 40.0);
};

template <class Real>
struct AssistParams {
    Real gain = BIKE_REAL(Real, 2.0);
    Real max_torque = BIKE_REAL(Real, 85.0);
    Real max_power = BIKE_REAL(Real, 600.0);
    Real tau = BIKE_REAL(Real, 0.04);
    Real slew = BIKE_REAL(Real, 400.0);
    Real engage_torque_nm = BIKE_REAL(Real, 4.0);
    i32 release_hold_ticks = 240;
    Real cutoff_mps = BIKE_REAL(Real, 25.0 / 3.6);
    Real taper_width_mps = BIKE_REAL(Real, 2.0 / 3.6);
    MotorProfileParams<Real> profile{};
    Real curve_rpm[kMaxAssistTorqueCurvePoints]{};
    Real curve_nm[kMaxAssistTorqueCurvePoints]{};
    i32 curve_len = 0;
    ProfileMode profile_mode = ProfileMode::turbo;
    u8 has_profile = 0;
    u8 pad[2]{};
};

template <class Real>
struct AssistState {
    Real torque = BIKE_REAL(Real, 0.0);
    Real last_gain = BIKE_REAL(Real, 0.0);
    Real hold_power_w = BIKE_REAL(Real, 0.0);
    Real hold_torque_nm = BIKE_REAL(Real, 0.0);
    i32 release_hold_ticks = 0;
    u8 pedaling = 0;
    u8 pad[3]{};
};

template <class Real>
struct AssistTickOutput {
    Real request_nm = BIKE_REAL(Real, 0.0);
    Real safety_nm = BIKE_REAL(Real, 0.0);
    Real delivered_nm = BIKE_REAL(Real, 0.0);
    u8 enabled = 0;
    u8 safety_limited = 0;
    u8 pad[2]{};
};

template <class Real>
BIKE_HD Real assist_flush_subnormal(Real x) noexcept {
    if constexpr (sizeof(Real) == 4) {
        const u32 bits = pf_f32_to_u32(static_cast<float>(x));
        if ((bits & 0x7f800000u) == 0u) {
            return static_cast<Real>(pf_u32_to_f32(bits & 0x80000000u));
        }
    }
    return x;
}

template <class Real, class TK>
BIKE_HD Real assist_expm1(Real x) noexcept {
    if constexpr (sizeof(Real) == 4) {
        const float xf = static_cast<float>(x);
        if (xf < -103.97207708399179f) {
            return static_cast<Real>(-1.0f);
        }
        if (pf_abs(xf) <= 0.34657359f) {
            return static_cast<Real>(expm1f_small_portable(xf));
        }
        return static_cast<Real>(expf_portable(xf) - 1.0f);
    } else {
        if (TK::abs(x) <= BIKE_REAL(Real, 1e-4)) {
            const Real c4 = BIKE_REAL(Real, 1.0 / 24.0);
            const Real c3 = BIKE_REAL(Real, 1.0 / 6.0);
            const Real c2 = BIKE_REAL(Real, 0.5);
            return x * (BIKE_REAL(Real, 1.0) +
                        x * (c2 + x * (c3 + x * c4)));
        }
        const Real u = TK::exp(x);
        if (u == BIKE_REAL(Real, 1.0)) {
            return x;
        }
        if (u == BIKE_REAL(Real, 0.0)) {
            return BIKE_REAL(Real, -1.0);
        }
        return (u - BIKE_REAL(Real, 1.0)) * x / TK::log(u);
    }
}

template <class Real, class TK, class ParamsPtr>
BIKE_HD Status assist_validate_params(
    ParamsPtr params,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (params == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(params->gain) || !TK::isfinite(params->max_torque) ||
        !TK::isfinite(params->max_power) || !TK::isfinite(params->tau) ||
        !TK::isfinite(params->slew) || !TK::isfinite(params->engage_torque_nm) ||
        !TK::isfinite(params->cutoff_mps) || !TK::isfinite(params->taper_width_mps) ||
        params->gain < BIKE_REAL(Real, 0.0) ||
        params->max_torque < BIKE_REAL(Real, 0.0) ||
        params->max_power < BIKE_REAL(Real, 0.0) ||
        params->tau <= BIKE_REAL(Real, 0.0) ||
        params->slew <= BIKE_REAL(Real, 0.0) ||
        params->engage_torque_nm < BIKE_REAL(Real, 0.0) ||
        params->release_hold_ticks < 0 ||
        params->cutoff_mps < BIKE_REAL(Real, 0.0) ||
        params->taper_width_mps <= BIKE_REAL(Real, 0.0) ||
        params->taper_width_mps > params->cutoff_mps) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (params->has_profile != 0) {
        if (!TK::isfinite(params->profile.eco) ||
            !TK::isfinite(params->profile.tour) ||
            !TK::isfinite(params->profile.emtb_low) ||
            !TK::isfinite(params->profile.emtb_high) ||
            !TK::isfinite(params->profile.turbo) ||
            !TK::isfinite(params->profile.emtb_full_gain_at_nm) ||
            params->profile.eco < BIKE_REAL(Real, 0.0) ||
            params->profile.tour < BIKE_REAL(Real, 0.0) ||
            params->profile.emtb_low < BIKE_REAL(Real, 0.0) ||
            params->profile.emtb_high < BIKE_REAL(Real, 0.0) ||
            params->profile.turbo < BIKE_REAL(Real, 0.0) ||
            params->profile.emtb_full_gain_at_nm <= BIKE_REAL(Real, 0.0) ||
            params->profile.emtb_low > params->profile.emtb_high) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        if (params->profile_mode != ProfileMode::eco &&
            params->profile_mode != ProfileMode::tour &&
            params->profile_mode != ProfileMode::emtb &&
            params->profile_mode != ProfileMode::turbo) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
    }
    if (params->curve_len != 0) {
        if (params->curve_len < 2 ||
            params->curve_len > kMaxAssistTorqueCurvePoints) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        for (i32 i = 0; i < params->curve_len; ++i) {
            if (!TK::isfinite(params->curve_rpm[i]) ||
                !TK::isfinite(params->curve_nm[i]) ||
                params->curve_rpm[i] < BIKE_REAL(Real, 0.0) ||
                params->curve_nm[i] < BIKE_REAL(Real, 0.0)) {
                err = ErrCode::outside_domain;
                return Status::invalid_argument;
            }
            if (i > 0 && params->curve_rpm[i] <= params->curve_rpm[i - 1]) {
                err = ErrCode::outside_domain;
                return Status::invalid_argument;
            }
        }
    }
    return Status::ok;
}

template <class Real, class StatePtr>
BIKE_HD void assist_reset(StatePtr state) noexcept {
    *state = AssistState<Real>{};
}

template <class Real, class TK, class StatePtr>
BIKE_HD Status assist_validate_state_invariants(
    StatePtr state,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (state == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(state->torque) || !TK::isfinite(state->last_gain) ||
        !TK::isfinite(state->hold_power_w) || !TK::isfinite(state->hold_torque_nm) ||
        state->torque < BIKE_REAL(Real, 0.0) ||
        state->last_gain < BIKE_REAL(Real, 0.0) ||
        state->hold_power_w < BIKE_REAL(Real, 0.0) ||
        state->hold_torque_nm < BIKE_REAL(Real, 0.0) ||
        state->release_hold_ticks < 0) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    // C3 relational invariants:
    //   release_hold_left > 0 => hold_power > 0 && hold_torque > 0
    //   hold_power > 0 <=> hold_torque > 0
    const bool has_hold_power = state->hold_power_w > BIKE_REAL(Real, 0.0);
    const bool has_hold_torque = state->hold_torque_nm > BIKE_REAL(Real, 0.0);
    if (has_hold_power != has_hold_torque) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (state->release_hold_ticks > 0 && !has_hold_power) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    return Status::ok;
}

template <class Real, class TK>
BIKE_HD Real assist_profile_gain(
    MotorProfileParams<Real> p,
    ProfileMode mode,
    Real sensed
) noexcept {
    switch (mode) {
    case ProfileMode::eco:
        return p.eco;
    case ProfileMode::tour:
        return p.tour;
    case ProfileMode::emtb: {
        const Real fraction = TK::min(
            BIKE_REAL(Real, 1.0),
            TK::max(BIKE_REAL(Real, 0.0), sensed) / p.emtb_full_gain_at_nm);
        return p.emtb_low + (p.emtb_high - p.emtb_low) * fraction;
    }
    case ProfileMode::turbo:
        return p.turbo;
    }
    return p.turbo;
}

template <class Real, class TK, class PtrRpm, class PtrNm>
BIKE_HD Real assist_interp_curve(
    PtrRpm curve_rpm,
    PtrNm curve_nm,
    i32 curve_len,
    Real x
) noexcept {
    if (x <= curve_rpm[0]) {
        return curve_nm[0];
    }
    if (x >= curve_rpm[curve_len - 1]) {
        return curve_nm[curve_len - 1];
    }
    for (i32 i = 1; i < curve_len; ++i) {
        const Real x1 = curve_rpm[i];
        if (x1 == x) {
            return curve_nm[i];
        }
        if (x1 > x) {
            const Real x0 = curve_rpm[i - 1];
            const Real y0 = curve_nm[i - 1];
            const Real y1 = curve_nm[i];
            const Real slope = (y1 - y0) / (x1 - x0);
            return TK::interp_segment(slope, x, x0, y0, x1, y1);
        }
    }
    return curve_nm[curve_len - 1];
}

template <class Real, class TK, class ParamsPtr>
BIKE_HD Status assist_ceiling(
    ParamsPtr params,
    Real shaft_rpm,
    Real speed_mps,
    BIKE_THREAD Real &out_limit,
    BIKE_THREAD Real &out_taper,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_limit = BIKE_REAL(Real, 0.0);
    out_taper = BIKE_REAL(Real, 0.0);
    const Status p_st = assist_validate_params<Real, TK>(params, err);
    if (p_st != Status::ok) {
        return p_st;
    }
    if (!TK::isfinite(shaft_rpm) || !TK::isfinite(speed_mps)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kTwoPi = BIKE_REAL(Real, 6.28318530717958647692);
    const Real omega = shaft_rpm * kTwoPi / BIKE_REAL(Real, 60.0);
    Real limit = params->max_torque;
    if (params->curve_len >= 2) {
        limit = TK::min(
            limit,
            assist_interp_curve<Real, TK>(
                params->curve_rpm, params->curve_nm, params->curve_len,
                shaft_rpm));
    }
    if (omega > BIKE_REAL(Real, 0.0)) {
        limit = TK::min(limit, params->max_power / omega);
    }
    const Real taper = TK::max(
        BIKE_REAL(Real, 0.0),
        TK::min(
            BIKE_REAL(Real, 1.0),
            (params->cutoff_mps - TK::abs(speed_mps)) /
                params->taper_width_mps));
    out_limit = limit * taper;
    out_taper = taper;
    return Status::ok;
}

template <class Real, class TK, class ParamsPtr, class StatePtr>
BIKE_HD Status assist_step(
    ParamsPtr params,
    StatePtr state,
    Real human_nm,
    Real cadence_rpm,
    Real speed_mps,
    bool braking,
    Real dt,
    bool has_torque_request,
    Real torque_request_nm,
    bool has_shaft_rpm,
    Real shaft_rpm,
    BIKE_THREAD Real &out_torque,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_torque = BIKE_REAL(Real, 0.0);
    if (state == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Status p_st = assist_validate_params<Real, TK>(params, err);
    if (p_st != Status::ok) {
        return p_st;
    }
    if (!TK::isfinite(human_nm) || !TK::isfinite(cadence_rpm) ||
        !TK::isfinite(speed_mps) || !TK::isfinite(dt) ||
        dt <= BIKE_REAL(Real, 0.0) ||
        (has_torque_request &&
         (!TK::isfinite(torque_request_nm) ||
          torque_request_nm < BIKE_REAL(Real, 0.0))) ||
        (has_shaft_rpm && !TK::isfinite(shaft_rpm))) {
        err = (!TK::isfinite(dt) || dt <= BIKE_REAL(Real, 0.0))
            ? ErrCode::bad_dt
            : ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Status s_st = assist_validate_state_invariants<Real, TK>(state, err);
    if (s_st != Status::ok) {
        return s_st;
    }

    if (braking) {
        assist_reset<Real>(state);
        out_torque = BIKE_REAL(Real, 0.0);
        return Status::ok;
    }

    const Real sensed =
        (human_nm > params->engage_torque_nm) ? human_nm : BIKE_REAL(Real, 0.0);
    Real gain = params->gain;
    if (params->has_profile != 0) {
        gain = assist_profile_gain<Real, TK>(
            params->profile, params->profile_mode, sensed);
    }

    const Real eff_shaft_rpm = has_shaft_rpm ? shaft_rpm : cadence_rpm;
    Real limit = BIKE_REAL(Real, 0.0);
    Real taper = BIKE_REAL(Real, 0.0);
    const Status c_st = assist_ceiling<Real, TK>(
        params, eff_shaft_rpm, speed_mps, limit, taper, err);
    if (c_st != Status::ok) {
        return c_st;
    }

    const Real target = TK::min(gain * sensed * taper, limit);
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kTwoPi = BIKE_REAL(Real, 6.28318530717958647692);
    const Real omega = eff_shaft_rpm * kTwoPi / BIKE_REAL(Real, 60.0);

    AssistState<Real> next = *state;
    if (target < next.torque) {
        if (next.hold_power_w <= BIKE_REAL(Real, 0.0) &&
            omega > BIKE_REAL(Real, 0.0)) {
            const Real hold_power = next.torque * omega;
            if (!TK::isfinite(hold_power)) {
                err = ErrCode::overflow_nonfinite;
                return Status::overflow;
            }
            if (hold_power > BIKE_REAL(Real, 0.0)) {
                next.hold_power_w = hold_power;
                next.hold_torque_nm = next.torque;
                next.release_hold_ticks = params->release_hold_ticks;
            }
        }
    } else {
        next.hold_power_w = BIKE_REAL(Real, 0.0);
        next.hold_torque_nm = BIKE_REAL(Real, 0.0);
        next.release_hold_ticks = 0;
    }

    const Real expm1_val = assist_expm1<Real, TK>(-dt / params->tau);
    Real candidate = next.torque - expm1_val * (target - next.torque);
    const Real delta = params->slew * dt;
    const Real lower = next.torque - delta;
    const Real upper = next.torque + delta;
    candidate = TK::max(lower, TK::min(upper, candidate));

    Real delivery = candidate;
    if (next.release_hold_ticks > 0) {
        --next.release_hold_ticks;
        if (omega > BIKE_REAL(Real, 0.0)) {
            delivery = TK::min(
                next.hold_power_w / omega,
                TK::min(next.hold_torque_nm, limit));
        } else {
            delivery = BIKE_REAL(Real, 0.0);
        }
    }

    Real torque = TK::min(delivery, limit);
    if (has_torque_request) {
        torque = TK::min(torque, torque_request_nm);
    }
    torque = assist_flush_subnormal(TK::max(BIKE_REAL(Real, 0.0), torque));
    if (!TK::isfinite(torque)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }

    next.last_gain = gain;
    next.torque = torque;
    next.pedaling = (torque > BIKE_REAL(Real, 0.0)) ? u8{1} : u8{0};
    *state = next;
    out_torque = torque;
    return Status::ok;
}

template <class Real, class TK, class ParamsPtr, class StatePtr>
BIKE_HD Status assist_step_with_limit(
    ParamsPtr params,
    StatePtr state,
    Real human_nm,
    Real cadence_rpm,
    Real speed_mps,
    bool braking,
    Real dt,
    bool has_torque_request,
    Real torque_request_nm,
    bool has_motor_limit,
    Real motor_limit_nm,
    bool has_shaft_rpm,
    Real shaft_rpm,
    bool active,
    BIKE_THREAD AssistTickOutput<Real> &out,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out = AssistTickOutput<Real>{};
    if (has_motor_limit &&
        (!TK::isfinite(motor_limit_nm) ||
         motor_limit_nm < BIKE_REAL(Real, 0.0))) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }

    // Exact drivetrain.cpp:862-895 ordering:
    // 1) assist_step runs WITHOUT motor_limit_nm (only external motor_torque_nm).
    // 2) safety = has_motor_limit ? min(request, motor_limit_nm) : request.
    // 3) enabled = active && safety > 0 && !braking; delivered = enabled ? safety : 0.
    // 4) state->torque = delivered (only when active, matching `if (effort_) bank.assist.torque = m.delivered`).
    Real request = BIKE_REAL(Real, 0.0);
    if (active) {
        const Status st = assist_step<Real, TK>(
            params, state, human_nm, cadence_rpm, speed_mps, braking, dt,
            has_torque_request, torque_request_nm, has_shaft_rpm, shaft_rpm,
            request, err);
        if (st != Status::ok) {
            return st;
        }
    }
    const Real safety =
        has_motor_limit ? TK::min(request, motor_limit_nm) : request;
    const bool enabled = active && (safety > BIKE_REAL(Real, 0.0)) && !braking;
    const Real delivered = enabled ? safety : BIKE_REAL(Real, 0.0);
    if (active && state != nullptr) {
        state->torque = assist_flush_subnormal(
            TK::max(BIKE_REAL(Real, 0.0), delivered));
    }
    out.request_nm = request;
    out.safety_nm = safety;
    out.delivered_nm = delivered;
    out.enabled = enabled ? u8{1} : u8{0};
    out.safety_limited = (safety < request) ? u8{1} : u8{0};
    return Status::ok;
}

template <class Real, class TK, class StatePtr>
BIKE_HD Status assist_post_solve_settle(
    StatePtr state,
    Real requested_torque_nm,
    Real solved_actuator_torque_nm,
    Real neg_tol,
    Real pos_tol,
    BIKE_THREAD Real &out_settled_torque_nm,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_settled_torque_nm = BIKE_REAL(Real, 0.0);
    const Status s_st = assist_validate_state_invariants<Real, TK>(state, err);
    if (s_st != Status::ok) {
        return s_st;
    }
    if (!TK::isfinite(requested_torque_nm) ||
        !TK::isfinite(solved_actuator_torque_nm) ||
        !TK::isfinite(neg_tol) || !TK::isfinite(pos_tol) ||
        requested_torque_nm < BIKE_REAL(Real, 0.0) ||
        neg_tol < BIKE_REAL(Real, 0.0) || pos_tol < BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (solved_actuator_torque_nm < -neg_tol ||
        solved_actuator_torque_nm > requested_torque_nm + pos_tol) {
        err = ErrCode::outside_domain;
        return Status::logic_error;
    }
    const Real settled = assist_flush_subnormal(
        TK::max(BIKE_REAL(Real, 0.0), solved_actuator_torque_nm));
    state->torque = settled;
    out_settled_torque_nm = settled;
    return Status::ok;
}

template <class Real, class TK, class StatePtr>
BIKE_HD Status assist_post_solve_settle(
    StatePtr state,
    Real requested_torque_nm,
    Real solved_actuator_torque_nm,
    BIKE_THREAD Real &out_settled_torque_nm,
    BIKE_THREAD ErrCode &err
) noexcept {
    const Real neg_tol = (sizeof(Real) == 4)
        ? BIKE_REAL(Real, 1e-6)
        : BIKE_REAL(Real, 1e-10);
    const Real pos_tol = (sizeof(Real) == 4)
        ? BIKE_REAL(Real, 1e-5)
        : BIKE_REAL(Real, 1e-8);
    return assist_post_solve_settle<Real, TK>(
        state, requested_torque_nm, solved_actuator_torque_nm,
        neg_tol, pos_tol, out_settled_torque_nm, err);
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
