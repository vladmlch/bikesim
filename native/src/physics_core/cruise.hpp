// native/src/physics_core/cruise.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// stateful PI cruise controller matching native/src/writers/cruise.cpp with 0-bit CPU parity.
#pragma once

#include "common.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

template <class Real>
struct CruiseConfigPod {
    Real target_speed_kmh = BIKE_REAL(Real, 25.0);
    Real kp_nm_per_mps = BIKE_REAL(Real, 0.0);
    Real ki_nm_per_mps_s = BIKE_REAL(Real, 0.0);
    Real torque_ceiling_nm = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct CruiseStatePod {
    Real target_speed_mps = BIKE_REAL(Real, 0.0);
    Real integral_mps_s = BIKE_REAL(Real, 0.0);
    Real torque_nm = BIKE_REAL(Real, 0.0);
    Real gain_scale = BIKE_REAL(Real, 1.0);
    u8 engaged = 0;
    u8 pad[3]{};
};

template <class Real, class TK>
BIKE_HD Real cruise_clamp_sym(Real value, Real limit) noexcept {
    // Match CruiseWriter::clamp: max(-limit, min(limit, value)), preserving ties
    // (including signed zero on limit == 0).
    return TK::max(-limit, TK::min(limit, value));
}

template <class Real, class TK, class ConfigPtr>
BIKE_HD Status cruise_validate_config(
    ConfigPtr cfg,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (cfg == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(cfg->kp_nm_per_mps) || cfg->kp_nm_per_mps <= BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(cfg->ki_nm_per_mps_s) || cfg->ki_nm_per_mps_s <= BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(cfg->torque_ceiling_nm) || cfg->torque_ceiling_nm <= BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(cfg->target_speed_kmh) ||
        cfg->target_speed_kmh < BIKE_REAL(Real, 15.0) ||
        cfg->target_speed_kmh > BIKE_REAL(Real, 45.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    return Status::ok;
}

template <class Real, class TK, class ConfigPtr>
BIKE_HD Status cruise_validate_scale(
    ConfigPtr cfg,
    Real scale,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (cfg == nullptr || !TK::isfinite(scale) || scale <= BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Real ki = cfg->ki_nm_per_mps_s * scale;
    if (!TK::isfinite(ki) || ki == BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    return Status::ok;
}

template <class Real, class StatePtr>
BIKE_HD void cruise_reset(StatePtr state) noexcept {
    if (state == nullptr) {
        return;
    }
    state->integral_mps_s = BIKE_REAL(Real, 0.0);
    state->torque_nm = BIKE_REAL(Real, 0.0);
    state->engaged = 0;
}

template <class Real, class TK, class StatePtr>
BIKE_HD Status cruise_set_target_speed_kmh(
    StatePtr state,
    Real value_kmh,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (state == nullptr || !TK::isfinite(value_kmh) ||
        value_kmh < BIKE_REAL(Real, 15.0) || value_kmh > BIKE_REAL(Real, 45.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    state->target_speed_mps = value_kmh / BIKE_REAL(Real, 3.6);
    return Status::ok;
}

template <class Real, class TK, class ConfigPtr, class StatePtr>
BIKE_HD Status cruise_set_assist_compensation(
    ConfigPtr cfg,
    StatePtr state,
    Real support_factor,
    BIKE_THREAD Real &out_scale,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_scale = BIKE_REAL(Real, 1.0);
    if (cfg == nullptr || state == nullptr || !TK::isfinite(support_factor)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Real scale =
        BIKE_REAL(Real, 1.0) /
        (BIKE_REAL(Real, 1.0) + TK::max(BIKE_REAL(Real, 0.0), support_factor));
    const Status sc_st = cruise_validate_scale<Real, TK>(cfg, scale, err);
    if (sc_st != Status::ok) {
        return sc_st;
    }
    state->gain_scale = scale;
    out_scale = scale;
    return Status::ok;
}

template <class Real, class TK, class ConfigPtr, class StatePtr, class SrcStatePtr>
BIKE_HD Status cruise_restore_state(
    ConfigPtr cfg,
    StatePtr dst,
    SrcStatePtr src,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (dst == nullptr || src == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Status cfg_st = cruise_validate_config<Real, TK>(cfg, err);
    if (cfg_st != Status::ok) {
        return cfg_st;
    }
    if (!TK::isfinite(src->target_speed_mps) ||
        !TK::isfinite(src->integral_mps_s) ||
        !TK::isfinite(src->torque_nm)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Real min_mps = BIKE_REAL(Real, 15.0) / BIKE_REAL(Real, 3.6);
    const Real max_mps = BIKE_REAL(Real, 45.0) / BIKE_REAL(Real, 3.6);
    if (src->target_speed_mps < min_mps || src->target_speed_mps > max_mps) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Status sc_st = cruise_validate_scale<Real, TK>(cfg, src->gain_scale, err);
    if (sc_st != Status::ok) {
        return sc_st;
    }
    CruiseStatePod<Real> staged{};
    staged.target_speed_mps = src->target_speed_mps;
    staged.integral_mps_s = src->integral_mps_s;
    staged.torque_nm = src->torque_nm;
    staged.gain_scale = src->gain_scale;
    staged.engaged = (src->engaged != 0) ? u8{1} : u8{0};
    *dst = staged;
    return Status::ok;
}

template <class Real, class TK, class ConfigPtr, class StatePtr>
BIKE_HD Status cruise_init(
    ConfigPtr cfg,
    StatePtr state,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (state == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Status cfg_st = cruise_validate_config<Real, TK>(cfg, err);
    if (cfg_st != Status::ok) {
        return cfg_st;
    }
    CruiseStatePod<Real> fresh{};
    const Status sp_st = cruise_set_target_speed_kmh<Real, TK>(
        &fresh, cfg->target_speed_kmh, err);
    if (sp_st != Status::ok) {
        return sp_st;
    }
    *state = fresh;
    return Status::ok;
}

template <class Real, class TK, class ConfigPtr, class StatePtr>
BIKE_HD Status cruise_step(
    ConfigPtr cfg,
    StatePtr state,
    Real speed_mps,
    Real dt_s,
    bool rear_in_contact,
    bool traction_limited,
    bool has_controller_grounded,
    bool controller_grounded,
    BIKE_THREAD Real &out_torque_nm,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_torque_nm = BIKE_REAL(Real, 0.0);
    if (state == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Status cfg_st = cruise_validate_config<Real, TK>(cfg, err);
    if (cfg_st != Status::ok) {
        return cfg_st;
    }

    CruiseStatePod<Real> next = *state;
    next.engaged =
        (has_controller_grounded ? controller_grounded : rear_in_contact)
            ? u8{1}
            : u8{0};
    const Real error = next.target_speed_mps - speed_mps;
    if (next.engaged == 0) {
        next.torque_nm = BIKE_REAL(Real, 0.0);
        *state = next;
        out_torque_nm = next.torque_nm;
        return Status::ok;
    }

    const Real kp = cfg->kp_nm_per_mps * next.gain_scale;
    if (!TK::isfinite(kp)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    const Real ki = cfg->ki_nm_per_mps_s * next.gain_scale;
    if (!TK::isfinite(ki)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    if (ki <= BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Real proportional = kp * error;
    if (!TK::isfinite(proportional)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    Real demand = proportional + ki * next.integral_mps_s;
    if (!TK::isfinite(demand)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    const bool pushing_further =
        (TK::abs(demand) >= cfg->torque_ceiling_nm) &&
        ((demand > BIKE_REAL(Real, 0.0)) == (error > BIKE_REAL(Real, 0.0)));
    if (!pushing_further && !traction_limited) {
        if (!TK::isfinite(dt_s) || dt_s <= BIKE_REAL(Real, 0.0)) {
            err = ErrCode::bad_dt;
            return Status::invalid_argument;
        }
        const Real integral_delta = error * dt_s;
        if (!TK::isfinite(integral_delta)) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }
        const Real integral_sum = next.integral_mps_s + integral_delta;
        if (!TK::isfinite(integral_sum)) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }
        next.integral_mps_s = cruise_clamp_sym<Real, TK>(
            integral_sum, cfg->torque_ceiling_nm / ki);
        demand = proportional + ki * next.integral_mps_s;
        if (!TK::isfinite(demand)) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }
    }
    next.torque_nm = cruise_clamp_sym<Real, TK>(demand, cfg->torque_ceiling_nm);
    *state = next;
    out_torque_nm = next.torque_nm;
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
