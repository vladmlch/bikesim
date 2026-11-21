// native/src/physics_core/shifting.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// autonomous cadence shifter with int32 tick counters and 12-tendon static cassette integration.
#pragma once

#include "common.hpp"
#include "ratchet.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

BIKE_CONSTEXPR i32 kMaxCassetteCogs = kCassetteCogCount;
BIKE_CONSTEXPR i32 kShiftingMaxI32 = 2147483647;

enum class ShiftDirection : u8 {
    none = 0,
    up = 1,
    down = 2
};

enum class SlipMode : u8 {
    legacy_signed = 0,
    magnitude = 1
};

struct RiderShiftInput {
    i32 stationary_target_teeth = 0;
    ShiftDirection request = ShiftDirection::none;
    u8 owns = 0;
    u8 has_request = 0;
    u8 has_stationary_target = 0;
};

template <class Real>
struct ShiftingParams {
    i32 front_teeth = 22;
    i32 initial_rear_teeth = 24;
    i32 cog_count = kCassetteCogCount;
    i32 cassette[kCassetteCogCount]{10, 12, 14, 16, 18, 21, 24, 28, 33, 39, 45, 51};
    i32 cassette_tendon_ids[kCassetteCogCount]{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    Real target_cadence_min_rpm = BIKE_REAL(Real, 75.0);
    Real target_cadence_max_rpm = BIKE_REAL(Real, 110.0);
    i32 shift_cooldown_ticks = 200;
    i32 shift_cut_duration_ticks = 160;
    Real torque_factor_larger_cog = BIKE_REAL(Real, 0.8);
    Real torque_factor_smaller_cog = BIKE_REAL(Real, 0.3);
    Real cadence_smoothing_tau_s = BIKE_REAL(Real, 0.05);
    Real upshift_slip_limit_mps = BIKE_REAL(Real, 0.5);
    SlipMode upshift_slip_mode = SlipMode::legacy_signed;
    u8 enabled = 1;
    u8 pad[2]{};
};

template <class Real>
struct ShiftingState {
    i32 rear_teeth = 24;
    i32 from_teeth = 24;
    i32 active_cog_idx = 6;
    i32 shift_count = 0;
    i32 cooldown_ticks = 0;
    i32 cut_remaining_ticks = 0;
    Real cadence_ema = BIKE_REAL(Real, 0.0);
    Real required_ema = BIKE_REAL(Real, 0.0);
    ShiftDirection direction = ShiftDirection::none;
    ShiftResponse rider_response = ShiftResponse::none;
    u8 has_ema = 0;
    u8 pad = 0;
};

template <class Real, class TK>
BIKE_HD i32 shifting_seconds_to_ticks(Real seconds, Real dt) noexcept {
    if (!TK::isfinite(seconds) || !TK::isfinite(dt) ||
        dt <= BIKE_REAL(Real, 0.0) || seconds <= BIKE_REAL(Real, 0.0)) {
        return 0;
    }
    return static_cast<i32>(TK::floor(seconds / dt + BIKE_REAL(Real, 0.5)));
}

template <class ParamsPtr>
BIKE_HD i32 shifting_find_cog_idx(ParamsPtr params, i32 teeth) noexcept {
    for (i32 i = 0; i < params->cog_count; ++i) {
        if (params->cassette[i] == teeth) {
            return i;
        }
    }
    return -1;
}

template <class Real, class TK, class ParamsPtr>
BIKE_HD Status shifting_validate_params(
    ParamsPtr params,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (params == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (params->front_teeth < 3 || params->initial_rear_teeth < 3 ||
        params->cog_count < 1 || params->cog_count > kMaxCassetteCogs) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    for (i32 i = 0; i < params->cog_count; ++i) {
        if (params->cassette[i] < 3) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        if (i > 0 && params->cassette[i] <= params->cassette[i - 1]) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
    }
    if (params->enabled != 0 &&
        shifting_find_cog_idx(params, params->initial_rear_teeth) < 0) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(params->target_cadence_min_rpm) ||
        !TK::isfinite(params->target_cadence_max_rpm) ||
        params->target_cadence_min_rpm <= BIKE_REAL(Real, 0.0) ||
        params->target_cadence_max_rpm <= params->target_cadence_min_rpm ||
        params->shift_cooldown_ticks < 0 ||
        params->shift_cut_duration_ticks < 0 ||
        params->shift_cut_duration_ticks > params->shift_cooldown_ticks) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(params->torque_factor_larger_cog) ||
        !TK::isfinite(params->torque_factor_smaller_cog) ||
        !TK::isfinite(params->cadence_smoothing_tau_s) ||
        !TK::isfinite(params->upshift_slip_limit_mps) ||
        params->torque_factor_larger_cog < BIKE_REAL(Real, 0.0) ||
        params->torque_factor_larger_cog > BIKE_REAL(Real, 1.0) ||
        params->torque_factor_smaller_cog < BIKE_REAL(Real, 0.0) ||
        params->torque_factor_smaller_cog > BIKE_REAL(Real, 1.0) ||
        params->cadence_smoothing_tau_s < BIKE_REAL(Real, 0.0) ||
        params->upshift_slip_limit_mps < BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (params->upshift_slip_mode != SlipMode::legacy_signed &&
        params->upshift_slip_mode != SlipMode::magnitude) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    return Status::ok;
}

template <class Real, class ParamsPtr, class StatePtr>
BIKE_HD void shifting_reset(ParamsPtr params, StatePtr state) noexcept {
    ShiftingState<Real> fresh{};
    fresh.rear_teeth = params->initial_rear_teeth;
    fresh.from_teeth = params->initial_rear_teeth;
    fresh.active_cog_idx = shifting_find_cog_idx(params, params->initial_rear_teeth);
    *state = fresh;
}

template <class Real, class ParamsPtr, class StatePtr>
BIKE_HD Real shift_torque_factor(ParamsPtr params, StatePtr state) noexcept {
    if (state->cut_remaining_ticks <= 0) {
        return BIKE_REAL(Real, 1.0);
    }
    if (state->direction == ShiftDirection::down) {
        return params->torque_factor_larger_cog;
    }
    if (state->direction == ShiftDirection::up) {
        return params->torque_factor_smaller_cog;
    }
    return BIKE_REAL(Real, 1.0);
}

template <class Real, class ParamsPtr, class StatePtr>
BIKE_HD Real shifting_gear_ratio(ParamsPtr params, StatePtr state) noexcept {
    return static_cast<Real>(params->front_teeth) /
           static_cast<Real>(state->rear_teeth);
}

template <class Real, class TK, class ParamsPtr, class StatePtr>
BIKE_HD Status shifting_update(
    ParamsPtr params,
    StatePtr state,
    Real cadence_rpm,
    Real required_cadence_rpm,
    Real dt,
    bool pedaling,
    bool braking,
    bool rear_in_contact,
    bool has_rear_slip,
    Real rear_slip_mps,
    RiderShiftInput rider,
    BIKE_THREAD bool &out_shifted,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_shifted = false;
    if (state == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Status p_st = shifting_validate_params<Real, TK>(params, err);
    if (p_st != Status::ok) {
        return p_st;
    }
    if (!TK::isfinite(cadence_rpm) || !TK::isfinite(required_cadence_rpm) ||
        !TK::isfinite(dt) || dt <= BIKE_REAL(Real, 0.0) ||
        (has_rear_slip && !TK::isfinite(rear_slip_mps))) {
        err = (dt <= BIKE_REAL(Real, 0.0) || !TK::isfinite(dt))
            ? ErrCode::bad_dt
            : ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (rider.owns == 0 && (rider.has_request != 0 || rider.has_stationary_target != 0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (rider.has_request != 0 && rider.request == ShiftDirection::none) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (rider.has_request != 0 && rider.has_stationary_target != 0) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (rider.has_stationary_target != 0 && rider.stationary_target_teeth < 3) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (state->rear_teeth < 3 || state->from_teeth < 3 || state->shift_count < 0 ||
        state->cooldown_ticks < 0 || state->cut_remaining_ticks < 0) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }

    ShiftingState<Real> next = *state;
    next.rider_response = ShiftResponse::none;

    const Real tau = params->cadence_smoothing_tau_s;
    const Real alpha = (tau <= BIKE_REAL(Real, 0.0))
        ? BIKE_REAL(Real, 1.0)
        : TK::min(BIKE_REAL(Real, 1.0), dt / tau);
    if (next.has_ema == 0) {
        next.cadence_ema = cadence_rpm;
        next.required_ema = required_cadence_rpm;
        next.has_ema = 1;
    } else {
        next.cadence_ema =
            next.cadence_ema + alpha * (cadence_rpm - next.cadence_ema);
        next.required_ema =
            next.required_ema + alpha * (required_cadence_rpm - next.required_ema);
    }
    if (!TK::isfinite(next.cadence_ema) || !TK::isfinite(next.required_ema)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }

    if (next.cooldown_ticks > 0) {
        --next.cooldown_ticks;
    }
    if (next.cut_remaining_ticks > 0) {
        --next.cut_remaining_ticks;
    }

    const bool admitted = (params->enabled != 0) && pedaling && !braking &&
                          rear_in_contact && (next.cooldown_ticks <= 0) &&
                          (TK::min(cadence_rpm, required_cadence_rpm) >=
                           BIKE_REAL(Real, 0.0));

    i32 selected = next.rear_teeth;
    i32 selected_idx = shifting_find_cog_idx(params, next.rear_teeth);
    ShiftDirection direction = ShiftDirection::none;

    if (rider.owns != 0) {
        if (rider.has_stationary_target != 0) {
            if (params->enabled == 0) {
                next.rider_response = ShiftResponse::deferred;
                *state = next;
                return Status::ok;
            }
            if (selected_idx < 0) {
                err = ErrCode::logic_no_closest;
                return Status::logic_error;
            }
            const i32 target_idx =
                shifting_find_cog_idx(params, rider.stationary_target_teeth);
            if (target_idx < 0) {
                next.rider_response = ShiftResponse::rejected;
                *state = next;
                return Status::ok;
            }
            if (next.rear_teeth >= rider.stationary_target_teeth) {
                *state = next;
                return Status::ok;
            }
            if (next.cut_remaining_ticks > 0) {
                next.rider_response = ShiftResponse::deferred;
                *state = next;
                return Status::ok;
            }
            if (next.shift_count == kShiftingMaxI32) {
                err = ErrCode::overflow_nonfinite;
                return Status::overflow;
            }
            const Real stationary_landing =
                next.required_ema *
                static_cast<Real>(rider.stationary_target_teeth) /
                static_cast<Real>(next.rear_teeth);
            if (!TK::isfinite(stationary_landing)) {
                err = ErrCode::overflow_nonfinite;
                return Status::overflow;
            }
            next.from_teeth = next.rear_teeth;
            next.rear_teeth = rider.stationary_target_teeth;
            next.active_cog_idx = target_idx;
            next.required_ema = stationary_landing;
            next.direction = ShiftDirection::down;
            ++next.shift_count;
            next.cooldown_ticks = params->shift_cooldown_ticks;
            next.cut_remaining_ticks = 0;
            next.rider_response = ShiftResponse::accepted;
            *state = next;
            out_shifted = true;
            return Status::ok;
        }
        if (rider.has_request == 0) {
            *state = next;
            return Status::ok;
        }
        if (!admitted || next.cut_remaining_ticks > 0) {
            next.rider_response = ShiftResponse::deferred;
            *state = next;
            return Status::ok;
        }
        if (selected_idx < 0) {
            err = ErrCode::logic_no_closest;
            return Status::logic_error;
        }
        direction = rider.request;
        if (direction == ShiftDirection::up) {
            if (selected_idx == 0) {
                next.rider_response = ShiftResponse::rejected;
                *state = next;
                return Status::ok;
            }
            selected_idx = selected_idx - 1;
            selected = params->cassette[selected_idx];
        } else {
            if (selected_idx + 1 >= params->cog_count) {
                next.rider_response = ShiftResponse::rejected;
                *state = next;
                return Status::ok;
            }
            selected_idx = selected_idx + 1;
            selected = params->cassette[selected_idx];
        }
    } else {
        if (!admitted) {
            *state = next;
            return Status::ok;
        }
        if (next.cadence_ema > params->target_cadence_max_rpm) {
            i32 fallback = next.rear_teeth;
            i32 fallback_idx = selected_idx;
            for (i32 i = params->cog_count - 1; i >= 0; --i) {
                const i32 teeth = params->cassette[i];
                if (teeth >= next.rear_teeth) {
                    continue;
                }
                fallback = teeth;
                fallback_idx = i;
                const Real cand_landing =
                    next.required_ema * static_cast<Real>(teeth) /
                    static_cast<Real>(next.rear_teeth);
                if (cand_landing >= params->target_cadence_min_rpm &&
                    cand_landing <= params->target_cadence_max_rpm) {
                    selected = teeth;
                    selected_idx = i;
                    break;
                }
            }
            if (selected == next.rear_teeth) {
                selected = fallback;
                selected_idx = fallback_idx;
            }
            direction = ShiftDirection::up;
        } else if (next.cadence_ema < params->target_cadence_min_rpm) {
            i32 fallback = next.rear_teeth;
            i32 fallback_idx = selected_idx;
            for (i32 i = 0; i < params->cog_count; ++i) {
                const i32 teeth = params->cassette[i];
                if (teeth <= next.rear_teeth) {
                    continue;
                }
                fallback = teeth;
                fallback_idx = i;
                const Real cand_landing =
                    next.required_ema * static_cast<Real>(teeth) /
                    static_cast<Real>(next.rear_teeth);
                if (cand_landing >= params->target_cadence_min_rpm &&
                    cand_landing <= params->target_cadence_max_rpm) {
                    selected = teeth;
                    selected_idx = i;
                    break;
                }
            }
            if (selected == next.rear_teeth) {
                selected = fallback;
                selected_idx = fallback_idx;
            }
            direction = ShiftDirection::down;
        } else {
            *state = next;
            return Status::ok;
        }
        if (selected == next.rear_teeth) {
            *state = next;
            return Status::ok;
        }
    }

    Real effective_slip = rear_slip_mps;
    if (has_rear_slip && params->upshift_slip_mode == SlipMode::magnitude) {
        effective_slip = TK::abs(effective_slip);
    }
    if (direction == ShiftDirection::up && has_rear_slip &&
        effective_slip > params->upshift_slip_limit_mps) {
        next.rider_response =
            (rider.owns != 0) ? ShiftResponse::rejected : ShiftResponse::none;
        *state = next;
        return Status::ok;
    }

    const Real landing = next.required_ema * static_cast<Real>(selected) /
                         static_cast<Real>(next.rear_teeth);
    if (!TK::isfinite(landing)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    if ((direction == ShiftDirection::up &&
         landing < params->target_cadence_min_rpm) ||
        (direction == ShiftDirection::down &&
         landing > params->target_cadence_max_rpm)) {
        next.rider_response =
            (rider.owns != 0) ? ShiftResponse::rejected : ShiftResponse::none;
        *state = next;
        return Status::ok;
    }
    if (next.shift_count == kShiftingMaxI32) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }

    next.from_teeth = next.rear_teeth;
    next.rear_teeth = selected;
    next.active_cog_idx = selected_idx;
    next.required_ema = landing;
    next.direction = direction;
    ++next.shift_count;
    next.cooldown_ticks = params->shift_cooldown_ticks;
    next.cut_remaining_ticks = params->shift_cut_duration_ticks;
    next.rider_response =
        (rider.owns != 0) ? ShiftResponse::accepted : ShiftResponse::none;
    *state = next;
    out_shifted = true;
    return Status::ok;
}

template <
    class Real,
    class TK,
    class ParamsPtr,
    class ShiftStatePtr,
    class RatchetStatePtr,
    class PtrRange>
BIKE_HD Status shifting_step_with_cassette(
    ParamsPtr params,
    ShiftStatePtr shift_state,
    RatchetStatePtr ratchet_state,
    Real cadence_rpm,
    Real required_cadence_rpm,
    Real dt,
    bool pedaling,
    bool braking,
    bool rear_in_contact,
    bool has_rear_slip,
    Real rear_slip_mps,
    RiderShiftInput rider,
    Real q_driver,
    Real q_driven,
    i32 ntendon,
    PtrRange tendon_range,
    BIKE_THREAD bool &out_shifted,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_shifted = false;
    if (ratchet_state == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Status sh_st = shifting_update<Real, TK>(
        params, shift_state, cadence_rpm, required_cadence_rpm, dt, pedaling,
        braking, rear_in_contact, has_rear_slip, rear_slip_mps, rider,
        out_shifted, err);
    if (sh_st != Status::ok) {
        return sh_st;
    }
    if (!out_shifted) {
        return Status::ok;
    }
    const Real old_ratio = static_cast<Real>(params->front_teeth) /
                           static_cast<Real>(shift_state->from_teeth);
    const Real new_ratio = static_cast<Real>(params->front_teeth) /
                           static_cast<Real>(shift_state->rear_teeth);
    bool has_boundary = (ratchet_state->has_boundary != 0);
    Real boundary = ratchet_state->boundary;
    const Status rb_st = ratchet_shift_ratio_bound<Real, TK>(
        old_ratio, new_ratio, q_driver, q_driven, has_boundary, boundary, err);
    if (rb_st != Status::ok) {
        return rb_st;
    }
    ratchet_state->has_boundary = has_boundary ? u8{1} : u8{0};
    ratchet_state->boundary = boundary;
    ratchet_state->ratio = new_ratio;

    return fill_cassette_tendon_ranges<Real, TK>(
        shift_state->active_cog_idx, params->cog_count, ntendon, boundary,
        params->cassette_tendon_ids, tendon_range, err);
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
