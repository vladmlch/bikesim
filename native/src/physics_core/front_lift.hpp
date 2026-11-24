// native/src/physics_core/front_lift.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// front-wheel-lift reaction controller with fixed circular ring buffers (3 x 401),
// incremental dtheta tracking (zero long-ride FP32 precision loss), and int32 tick counters.
#pragma once

#include "common.hpp"
#include "launch_plan.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: POD structs and templated pointers match the CUDA/MSL ABI without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

BIKE_CONSTEXPR i32 kFrontLiftRingCap = 401;

enum class ReactionPhase : u8 {
    idle = 0,
    deciding = 1,
    acting = 2,
    releasing = 3
};

template <class Real>
struct FrontLiftParams {
    Real hand_drop_ratio = BIKE_REAL(Real, 0.5);
    Real pitch_rate_up_rad_s = BIKE_REAL(Real, 0.5);
    Real pitch_rad = BIKE_REAL(Real, 0.1);
    Real ease_fraction = BIKE_REAL(Real, 0.0);
    Real ease_rate_per_s = BIKE_REAL(Real, 1.0);
    Real lean_add_rad = BIKE_REAL(Real, 0.0);
    Real lean_rate_rad_s = BIKE_REAL(Real, 1.0);
    Real period_s = BIKE_REAL(Real, 0.00125);
    i32 window_ticks = 400;
    i32 lag_ticks = 400;
    i32 decision_ticks = 0;
    i32 clear_ticks = 0;
    u8 ease_enabled = 0;
    u8 lean_enabled = 0;
    u8 pad[2]{};
};

template <class Real>
struct FrontLiftState {
    i32 decision_ticks = 0;
    i32 clear_ticks = 0;
    i32 percepts_seen = 0;
    i32 ring_head = 0;
    i32 ring_count = 0;
    i32 avg_head = 0;
    i32 avg_count = 0;
    Real effort_factor = BIKE_REAL(Real, 1.0);
    Real lean_add_rad = BIKE_REAL(Real, 0.0);
    Real crank_angle_mod_2pi_rad = BIKE_REAL(Real, 0.0);
    ReactionPhase phase = ReactionPhase::idle;
    u8 predicate = 0;
    u8 pad[2]{};

    Real hand_n[kFrontLiftRingCap]{};
    Real crank_dtheta[kFrontLiftRingCap]{};
    Real hand_average_n[kFrontLiftRingCap]{};
};

template <class Real>
struct ReactionCommandCore {
    Real effort_factor = BIKE_REAL(Real, 1.0);
    Real lean_add_rad = BIKE_REAL(Real, 0.0);
    ReactionPhase phase = ReactionPhase::idle;
    u8 predicate = 0;
    u8 pad[2]{};
};

template <class CfgPtr>
BIKE_HD i32 front_lift_percepts_cap(CfgPtr cfg) noexcept {
    const i32 sum = cfg->lag_ticks + 1 + cfg->decision_ticks + cfg->clear_ticks + 1;
    return sum > 0 ? sum : 0;
}

template <class Real, class StatePtr>
BIKE_HD void front_lift_reset(StatePtr state) noexcept {
    state->decision_ticks = 0;
    state->clear_ticks = 0;
    state->percepts_seen = 0;
    state->ring_head = 0;
    state->ring_count = 0;
    state->avg_head = 0;
    state->avg_count = 0;
    state->effort_factor = BIKE_REAL(Real, 1.0);
    state->lean_add_rad = BIKE_REAL(Real, 0.0);
    state->crank_angle_mod_2pi_rad = BIKE_REAL(Real, 0.0);
    state->phase = ReactionPhase::idle;
    state->predicate = 0;
    state->pad[0] = 0;
    state->pad[1] = 0;
}

template <class Real, class StatePtr>
BIKE_HD ReactionCommandCore<Real> front_lift_command(StatePtr state) noexcept {
    ReactionCommandCore<Real> cmd{};
    cmd.phase = state->phase;
    cmd.predicate = state->predicate;
    cmd.effort_factor = state->effort_factor;
    cmd.lean_add_rad = state->lean_add_rad;
    return cmd;
}

template <class Real, class TK, class CfgPtr, class PerceptsPtr>
BIKE_HD Status front_lift_validate_inputs(
    CfgPtr cfg,
    bool has_percepts,
    PerceptsPtr percepts,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (!TK::isfinite(cfg->period_s) || cfg->period_s <= BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(cfg->hand_drop_ratio) ||
        cfg->hand_drop_ratio <= BIKE_REAL(Real, 0.0) ||
        cfg->hand_drop_ratio > BIKE_REAL(Real, 1.0) ||
        !TK::isfinite(cfg->pitch_rate_up_rad_s) ||
        cfg->pitch_rate_up_rad_s <= BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(cfg->pitch_rad) || cfg->pitch_rad <= BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(cfg->ease_fraction) ||
        cfg->ease_fraction < BIKE_REAL(Real, 0.0) ||
        cfg->ease_fraction > BIKE_REAL(Real, 1.0) ||
        !TK::isfinite(cfg->ease_rate_per_s) ||
        cfg->ease_rate_per_s <= BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(cfg->lean_add_rad) ||
        cfg->lean_add_rad < BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(cfg->lean_rate_rad_s) ||
        cfg->lean_rate_rad_s <= BIKE_REAL(Real, 0.0) ||
        cfg->decision_ticks < 0 || cfg->clear_ticks < 0) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (cfg->window_ticks <= 0 || cfg->window_ticks >= kFrontLiftRingCap ||
        cfg->lag_ticks <= 0 || cfg->lag_ticks >= kFrontLiftRingCap) {
        err = ErrCode::out_of_bounds_station;
        return Status::out_of_bounds;
    }
    if (has_percepts) {
        if (!TK::isfinite(percepts->senses.hand_press_n) ||
            !TK::isfinite(percepts->crank_rpm) ||
            !TK::isfinite(percepts->pitch_rate_up_rad_s) ||
            !TK::isfinite(percepts->pitch_rad)) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
    }
    return Status::ok;
}

namespace detail {

template <class Real, class TK>
BIKE_HD Real front_lift_step_toward(
    Real value,
    Real target,
    Real step
) noexcept {
    if (TK::abs(target - value) <= step) {
        return target;
    }
    return value + (target > value ? step : -step);
}

template <class Real, class TK, class CfgPtr, class StatePtr, class PerceptsPtr>
BIKE_HD bool front_lift_hand_term(
    CfgPtr cfg,
    StatePtr state,
    PerceptsPtr p
) noexcept {
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>; exact FP64/FP32 pi literal
    const Real kPi = BIKE_REAL(Real, 3.14159265358979323846);
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>; exact FP64/FP32 2*pi literal
    const Real kTwoPi = BIKE_REAL(Real, 6.28318530717958647692);
    const Real dtheta =
        p->crank_rpm * (kTwoPi / BIKE_REAL(Real, 60.0)) * cfg->period_s;

    const Real next_mod = state->crank_angle_mod_2pi_rad + dtheta;
    const Real turns = TK::floor(next_mod / kTwoPi);
    state->crank_angle_mod_2pi_rad = next_mod - turns * kTwoPi;

    const i32 ring_cap = cfg->window_ticks + 1;
    if (state->ring_count < ring_cap) {
        const i32 write_idx = (state->ring_head + state->ring_count) % ring_cap;
        state->hand_n[write_idx] = p->senses.hand_press_n;
        state->crank_dtheta[write_idx] = dtheta;
        ++state->ring_count;
    } else {
        const i32 write_idx = state->ring_head;
        state->hand_n[write_idx] = p->senses.hand_press_n;
        state->crank_dtheta[write_idx] = dtheta;
        state->ring_head = (state->ring_head + 1) % ring_cap;
    }

    // Incremental backwards angle sum avoids subtracting large monotonic totals in FP32.
    i32 span = -1;
    Real accum_angle = BIKE_REAL(Real, 0.0);
    const i32 count = state->ring_count;
    for (i32 step = 1; step < count; ++step) {
        const i32 phys_idx = (state->ring_head + (count - step)) % ring_cap;
        accum_angle += state->crank_dtheta[phys_idx];
        if (accum_angle >= kPi) {
            span = step;
            break;
        }
    }
    if (span < 0 || span > cfg->window_ticks) {
        span = count < cfg->window_ticks ? count : cfg->window_ticks;
    }

    Real sum = BIKE_REAL(Real, 0.0);
    for (i32 j = count - span; j < count; ++j) {
        const i32 phys_idx = (state->ring_head + j) % ring_cap;
        sum += state->hand_n[phys_idx];
    }
    const Real average = sum / static_cast<Real>(span);

    const i32 avg_cap = cfg->lag_ticks + 1;
    if (state->avg_count < avg_cap) {
        const i32 write_idx = (state->avg_head + state->avg_count) % avg_cap;
        state->hand_average_n[write_idx] = average;
        ++state->avg_count;
    } else {
        const i32 write_idx = state->avg_head;
        state->hand_average_n[write_idx] = average;
        state->avg_head = (state->avg_head + 1) % avg_cap;
    }

    if (state->avg_count < avg_cap) {
        return false;
    }
    const Real reference = state->hand_average_n[state->avg_head];
    return reference > BIKE_REAL(Real, 0.0) &&
           average < cfg->hand_drop_ratio * reference;
}

} // namespace detail

template <class Real, class TK, class CfgPtr, class StatePtr, class PerceptsPtr>
BIKE_HD Status front_lift_update(
    CfgPtr cfg,
    bool has_percepts,
    PerceptsPtr percepts,
    StatePtr state,
    BIKE_THREAD ReactionCommandCore<Real> &out_cmd,
    BIKE_THREAD ErrCode &err
) noexcept {
    const Status vst =
        front_lift_validate_inputs<Real, TK>(cfg, has_percepts, percepts, err);
    if (vst != Status::ok) {
        out_cmd = ReactionCommandCore<Real>{};
        return vst;
    }
    if (state->ring_head < 0 || state->ring_count < 0 ||
        state->ring_count > cfg->window_ticks + 1 ||
        state->avg_head < 0 || state->avg_count < 0 ||
        state->avg_count > cfg->lag_ticks + 1) {
        err = ErrCode::out_of_bounds_station;
        out_cmd = ReactionCommandCore<Real>{};
        return Status::out_of_bounds;
    }

    if (!has_percepts) {
        state->predicate = 0;
    } else {
        const i32 cap = front_lift_percepts_cap(cfg);
        const i32 next_seen = state->percepts_seen + 1;
        state->percepts_seen = next_seen < cap ? next_seen : cap;
        const bool hands = detail::front_lift_hand_term<Real, TK>(cfg, state, percepts);
        const bool pitch_hit =
            (percepts->pitch_rate_up_rad_s > cfg->pitch_rate_up_rad_s) ||
            (percepts->pitch_rad > cfg->pitch_rad);
        state->predicate = (hands && pitch_hit) ? 1 : 0;
    }

    switch (state->phase) {
    case ReactionPhase::idle:
        if (state->predicate != 0) {
            state->phase = ReactionPhase::deciding;
            state->decision_ticks = 0;
        }
        break;
    case ReactionPhase::deciding:
        if (state->predicate == 0) {
            state->phase = ReactionPhase::idle;
            state->decision_ticks = 0;
        } else {
            ++state->decision_ticks;
        }
        break;
    case ReactionPhase::acting:
        state->clear_ticks = (state->predicate != 0) ? 0 : (state->clear_ticks + 1);
        if (state->clear_ticks > cfg->clear_ticks) {
            state->phase = ReactionPhase::releasing;
            state->clear_ticks = 0;
        }
        break;
    case ReactionPhase::releasing:
        if (state->predicate != 0) {
            state->phase = ReactionPhase::acting;
        }
        break;
    }

    if (state->phase == ReactionPhase::deciding &&
        state->decision_ticks >= cfg->decision_ticks) {
        state->phase = ReactionPhase::acting;
        state->decision_ticks = 0;
        state->clear_ticks = 0;
    }

    const bool acting = (state->phase == ReactionPhase::acting);
    const Real ease_target =
        (acting && cfg->ease_enabled != 0)
            ? (BIKE_REAL(Real, 1.0) - cfg->ease_fraction)
            : BIKE_REAL(Real, 1.0);
    const Real lean_target =
        (acting && cfg->lean_enabled != 0)
            ? cfg->lean_add_rad
            : BIKE_REAL(Real, 0.0);
    state->effort_factor = detail::front_lift_step_toward<Real, TK>(
        state->effort_factor, ease_target, cfg->ease_rate_per_s * cfg->period_s);
    state->lean_add_rad = detail::front_lift_step_toward<Real, TK>(
        state->lean_add_rad, lean_target, cfg->lean_rate_rad_s * cfg->period_s);

    if (state->phase == ReactionPhase::releasing &&
        state->effort_factor == BIKE_REAL(Real, 1.0) &&
        state->lean_add_rad == BIKE_REAL(Real, 0.0)) {
        state->phase = ReactionPhase::idle;
    }

    out_cmd = front_lift_command<Real>(state);
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
