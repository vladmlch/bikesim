// native/src/physics_core/ride_program.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// GPU scripted ride programs supporting piecewise-constant step schedules and piecewise-linear ramps.
#pragma once

#include "common.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

BIKE_CONSTEXPR i32 kMaxProgramSegments = 32;

// NOLINTNEXTLINE(performance-enum-size) u32 bitflags match GPU ABI layout
enum class GpuProgramChannelMask : u32 {
    kProgramChannelNone = 0u,
    has_human_torque = 1u << 0u,
    has_motor_torque = 1u << 1u,
    has_motor_limit = 1u << 2u,
    has_crank_rate = 1u << 3u,
    has_front_brake = 1u << 4u,
    has_rear_brake = 1u << 5u,
    has_cruise_speed_kmh = 1u << 6u,
    has_posture_lean = 1u << 7u,
    linear_ramp = 1u << 8u
};

BIKE_CONSTEXPR u32 kProgramValueChannelsMask =
    static_cast<u32>(GpuProgramChannelMask::has_human_torque) |
    static_cast<u32>(GpuProgramChannelMask::has_motor_torque) |
    static_cast<u32>(GpuProgramChannelMask::has_motor_limit) |
    static_cast<u32>(GpuProgramChannelMask::has_crank_rate) |
    static_cast<u32>(GpuProgramChannelMask::has_front_brake) |
    static_cast<u32>(GpuProgramChannelMask::has_rear_brake) |
    static_cast<u32>(GpuProgramChannelMask::has_cruise_speed_kmh) |
    static_cast<u32>(GpuProgramChannelMask::has_posture_lean);

template <class Real>
struct GpuProgramSegment {
    Real t_end_s = BIKE_REAL(Real, 0.0);
    u32 channel_mask = 0u;
    Real human_torque_nm = BIKE_REAL(Real, 0.0);
    Real motor_torque_nm = BIKE_REAL(Real, 0.0);
    Real motor_limit_nm = BIKE_REAL(Real, 0.0);
    Real crank_rate_rad_s = BIKE_REAL(Real, 0.0);
    Real front_brake = BIKE_REAL(Real, 0.0);
    Real rear_brake = BIKE_REAL(Real, 0.0);
    Real cruise_speed_kmh = BIKE_REAL(Real, 0.0);
    Real posture_lean_rad = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct GpuProgramOverride {
    u32 channel_mask = 0u;
    Real human_torque_nm = BIKE_REAL(Real, 0.0);
    Real motor_torque_nm = BIKE_REAL(Real, 0.0);
    Real motor_limit_nm = BIKE_REAL(Real, 0.0);
    Real crank_rate_rad_s = BIKE_REAL(Real, 0.0);
    Real front_brake = BIKE_REAL(Real, 0.0);
    Real rear_brake = BIKE_REAL(Real, 0.0);
    Real cruise_speed_kmh = BIKE_REAL(Real, 0.0);
    Real posture_lean_rad = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct GpuRideProgram {
    GpuProgramSegment<Real> segments[kMaxProgramSegments]{};
    i32 segment_count = 0;
    Real reaction_delay_s = BIKE_REAL(Real, 0.0);
};

namespace detail {

template <class Real, class TK, class SegPtr>
BIKE_HD bool segment_channels_finite(SegPtr seg) noexcept {
    const u32 m = seg->channel_mask;
    if ((m & static_cast<u32>(GpuProgramChannelMask::has_human_torque)) != 0u &&
        !TK::isfinite(seg->human_torque_nm)) {
        return false;
    }
    if ((m & static_cast<u32>(GpuProgramChannelMask::has_motor_torque)) != 0u &&
        !TK::isfinite(seg->motor_torque_nm)) {
        return false;
    }
    if ((m & static_cast<u32>(GpuProgramChannelMask::has_motor_limit)) != 0u &&
        !TK::isfinite(seg->motor_limit_nm)) {
        return false;
    }
    if ((m & static_cast<u32>(GpuProgramChannelMask::has_crank_rate)) != 0u &&
        !TK::isfinite(seg->crank_rate_rad_s)) {
        return false;
    }
    if ((m & static_cast<u32>(GpuProgramChannelMask::has_front_brake)) != 0u &&
        !TK::isfinite(seg->front_brake)) {
        return false;
    }
    if ((m & static_cast<u32>(GpuProgramChannelMask::has_rear_brake)) != 0u &&
        !TK::isfinite(seg->rear_brake)) {
        return false;
    }
    if ((m & static_cast<u32>(GpuProgramChannelMask::has_cruise_speed_kmh)) != 0u &&
        !TK::isfinite(seg->cruise_speed_kmh)) {
        return false;
    }
    if ((m & static_cast<u32>(GpuProgramChannelMask::has_posture_lean)) != 0u &&
        !TK::isfinite(seg->posture_lean_rad)) {
        return false;
    }
    return true;
}

template <class Real>
BIKE_HD Real mix_channel(
    u32 bit,
    u32 curr_mask,
    Real curr_val,
    bool has_prev,
    u32 prev_mask,
    Real prev_val,
    bool do_ramp,
    Real u
) noexcept {
    if ((curr_mask & bit) == 0u) {
        return BIKE_REAL(Real, 0.0);
    }
    if (!do_ramp) {
        return curr_val;
    }
    const Real start_val =
        (has_prev && ((prev_mask & bit) != 0u))
            ? prev_val
            : BIKE_REAL(Real, 0.0);
    return start_val + (curr_val - start_val) * u;
}

} // namespace detail

template <class Real, class TK, class ProgPtr>
BIKE_HD Status validate_ride_program(
    ProgPtr prog,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (prog == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (prog->segment_count < 0 || prog->segment_count > kMaxProgramSegments ||
        !TK::isfinite(prog->reaction_delay_s) ||
        prog->reaction_delay_s < BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    for (i32 i = 0; i < prog->segment_count; ++i) {
        const Real t_end = prog->segments[i].t_end_s;
        if (!TK::isfinite(t_end) || t_end < BIKE_REAL(Real, 0.0)) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        if (i > 0 && t_end <= prog->segments[i - 1].t_end_s) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        if (!detail::segment_channels_finite<Real, TK>(&prog->segments[i])) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
    }
    return Status::ok;
}

template <class Real, class TK, class ProgPtr, class OverridePtr>
BIKE_HD Status eval_ride_program(
    ProgPtr prog,
    Real time_s,
    OverridePtr out_override,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (out_override == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    *out_override = GpuProgramOverride<Real>{};
    if (!TK::isfinite(time_s)) {
        err = ErrCode::bad_time;
        return Status::invalid_argument;
    }
    const Status v_st = validate_ride_program<Real, TK>(prog, err);
    if (v_st != Status::ok) {
        return v_st;
    }
    if (prog->segment_count == 0) {
        return Status::ok;
    }

    const Real t =
        TK::max(BIKE_REAL(Real, 0.0), time_s - prog->reaction_delay_s);

    i32 idx = prog->segment_count - 1;
    bool past_end = true;
    for (i32 i = 0; i < prog->segment_count; ++i) {
        if (t < prog->segments[i].t_end_s) {
            idx = i;
            past_end = false;
            break;
        }
    }

    const u32 curr_mask = prog->segments[idx].channel_mask;
    const bool do_ramp =
        !past_end &&
        ((curr_mask & static_cast<u32>(GpuProgramChannelMask::linear_ramp)) != 0u);
    Real u = BIKE_REAL(Real, 1.0);
    if (do_ramp) {
        const Real t_start =
            (idx == 0) ? BIKE_REAL(Real, 0.0) : prog->segments[idx - 1].t_end_s;
        const Real t_end = prog->segments[idx].t_end_s;
        const Real denom = t_end - t_start;
        if (denom > BIKE_REAL(Real, 0.0)) {
            u = TK::clamp(
                (t - t_start) / denom,
                BIKE_REAL(Real, 0.0),
                BIKE_REAL(Real, 1.0));
        }
    }

    const bool has_prev = (idx > 0);
    const u32 prev_mask =
        has_prev ? prog->segments[idx - 1].channel_mask : 0u;
    const GpuProgramSegment<Real> curr = prog->segments[idx];
    const GpuProgramSegment<Real> prev =
        has_prev ? prog->segments[idx - 1] : GpuProgramSegment<Real>{};

    GpuProgramOverride<Real> result{};
    result.channel_mask = curr_mask & kProgramValueChannelsMask;
    result.human_torque_nm = detail::mix_channel<Real>(
        static_cast<u32>(GpuProgramChannelMask::has_human_torque),
        curr_mask, curr.human_torque_nm,
        has_prev, prev_mask, prev.human_torque_nm, do_ramp, u);
    result.motor_torque_nm = detail::mix_channel<Real>(
        static_cast<u32>(GpuProgramChannelMask::has_motor_torque),
        curr_mask, curr.motor_torque_nm,
        has_prev, prev_mask, prev.motor_torque_nm, do_ramp, u);
    result.motor_limit_nm = detail::mix_channel<Real>(
        static_cast<u32>(GpuProgramChannelMask::has_motor_limit),
        curr_mask, curr.motor_limit_nm,
        has_prev, prev_mask, prev.motor_limit_nm, do_ramp, u);
    result.crank_rate_rad_s = detail::mix_channel<Real>(
        static_cast<u32>(GpuProgramChannelMask::has_crank_rate),
        curr_mask, curr.crank_rate_rad_s,
        has_prev, prev_mask, prev.crank_rate_rad_s, do_ramp, u);
    result.front_brake = detail::mix_channel<Real>(
        static_cast<u32>(GpuProgramChannelMask::has_front_brake),
        curr_mask, curr.front_brake,
        has_prev, prev_mask, prev.front_brake, do_ramp, u);
    result.rear_brake = detail::mix_channel<Real>(
        static_cast<u32>(GpuProgramChannelMask::has_rear_brake),
        curr_mask, curr.rear_brake,
        has_prev, prev_mask, prev.rear_brake, do_ramp, u);
    result.cruise_speed_kmh = detail::mix_channel<Real>(
        static_cast<u32>(GpuProgramChannelMask::has_cruise_speed_kmh),
        curr_mask, curr.cruise_speed_kmh,
        has_prev, prev_mask, prev.cruise_speed_kmh, do_ramp, u);
    result.posture_lean_rad = detail::mix_channel<Real>(
        static_cast<u32>(GpuProgramChannelMask::has_posture_lean),
        curr_mask, curr.posture_lean_rad,
        has_prev, prev_mask, prev.posture_lean_rad, do_ramp, u);

    *out_override = result;
    return Status::ok;
}

template <class Real, class TK, class ProgPtr>
BIKE_HD Status eval_ride_program(
    ProgPtr prog,
    Real time_s,
    BIKE_THREAD GpuProgramOverride<Real> &out_override,
    BIKE_THREAD ErrCode &err
) noexcept {
    return eval_ride_program<Real, TK>(prog, time_s, &out_override, err);
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
