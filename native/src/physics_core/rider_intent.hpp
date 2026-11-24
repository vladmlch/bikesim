// native/src/physics_core/rider_intent.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// Perception (120-tick ring), SeatedClimbPolicy (int32 sub-tick surge budget),
// SeatedPostureProgram (160-tick ring + 4 pulses), shared RoadProfile / SurfaceLabels lookup,
// and unified RiderIntent orchestrator (<= 16 KiB/world combined SP3 state).
#pragma once

#include "common.hpp"
#include "front_lift.hpp"
#include "gpu_math.hpp"
#include "launch_plan.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: POD structs and templated pointers match the CUDA/MSL ABI without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

BIKE_CONSTEXPR i32 kPerceptionRingCap = 120;
BIKE_CONSTEXPR i32 kPostureRingCap = 160;
BIKE_CONSTEXPR i32 kMaxPosturePulses = 4;
// 600 sub-ticks per tick makes 1/3, 1/2, 1/4, 1/5, 1/6, 1/10, 1/12, 1/100 exact in int32.
BIKE_CONSTEXPR i32 kSurgeSubTicks = 600;

struct PortableTrig {
    BIKE_HD static float flush_subnorm(float x) noexcept {
        const u32 u = pf_f32_to_u32(x);
        return ((u & 0x7f800000u) == 0u) ? pf_u32_to_f32(u & 0x80000000u) : x;
    }
#if !defined(__METAL_VERSION__)
    BIKE_HD static double flush_subnorm(double x) noexcept {
        return x;
    }
#endif

    template <class Real, class TK>
    BIKE_HD static Real atan_val(Real x_in) noexcept {
        const Real x = flush_subnorm(x_in);
        if (TK::isnan(x)) {
            return x;
        }
        if (x == BIKE_REAL(Real, 0.0)) {
            return x;
        }
        // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>; exact FP64/FP32 pi literals
        const Real kPi2 = BIKE_REAL(Real, 1.57079632679489661923);
        // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>; exact FP64/FP32 pi/4 literal
        const Real kPi4 = BIKE_REAL(Real, 0.78539816339744830962);
        const Real ax = TK::abs(x);
        const Real sign = (x < BIKE_REAL(Real, 0.0)) ? BIKE_REAL(Real, -1.0) : BIKE_REAL(Real, 1.0);
        if (!TK::isfinite(ax)) {
            return sign * kPi2;
        }
        Real y = BIKE_REAL(Real, 0.0);
        Real z = ax;
        if (ax > BIKE_REAL(Real, 2.41421356237309504880)) {
            y = kPi2;
            z = BIKE_REAL(Real, -1.0) / ax;
        } else if (ax > BIKE_REAL(Real, 0.41421356237309504880)) {
            y = kPi4;
            z = (ax - BIKE_REAL(Real, 1.0)) / (ax + BIKE_REAL(Real, 1.0));
        }
        const Real z2 = z * z;
        Real p = BIKE_REAL(Real, 8.05374449538e-2);
        p = p * z2 - BIKE_REAL(Real, 1.38776856032e-1);
        p = p * z2 + BIKE_REAL(Real, 1.99777106478e-1);
        p = p * z2 - BIKE_REAL(Real, 3.33329491539e-1);
        p = p * z2 * z + z;
        return sign * (y + p);
    }

    template <class Real, class TK>
    BIKE_HD static Real atan2_val(Real y_in, Real x_in) noexcept {
        const Real y = flush_subnorm(y_in);
        const Real x = flush_subnorm(x_in);
        if (TK::isnan(y) || TK::isnan(x)) {
            return static_cast<Real>(pf_u32_to_f32(0x7fc00000u));
        }
        // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>; exact FP64/FP32 pi literals
        const Real kPi = BIKE_REAL(Real, 3.14159265358979323846);
        // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>; exact FP64/FP32 pi/2 literal
        const Real kPi2 = BIKE_REAL(Real, 1.57079632679489661923);
        // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>; exact FP64/FP32 pi/4 literal
        const Real kPi4 = BIKE_REAL(Real, 0.78539816339744830962);
        if (x == BIKE_REAL(Real, 0.0)) {
            if (y > BIKE_REAL(Real, 0.0)) {
                return kPi2;
            }
            if (y < BIKE_REAL(Real, 0.0)) {
                return -kPi2;
            }
            return BIKE_REAL(Real, 0.0);
        }
        if (y == BIKE_REAL(Real, 0.0)) {
            return (x < BIKE_REAL(Real, 0.0)) ? kPi : y;
        }
        if (!TK::isfinite(x) && !TK::isfinite(y)) {
            const Real base = (x < BIKE_REAL(Real, 0.0)) ? (BIKE_REAL(Real, 3.0) * kPi4) : kPi4;
            return (y < BIKE_REAL(Real, 0.0)) ? -base : base;
        }
        if (!TK::isfinite(x)) {
            return (x > BIKE_REAL(Real, 0.0))
                       ? BIKE_REAL(Real, 0.0)
                       : (y < BIKE_REAL(Real, 0.0) ? -kPi : kPi);
        }
        if (!TK::isfinite(y)) {
            return (y > BIKE_REAL(Real, 0.0)) ? kPi2 : -kPi2;
        }
        const Real r = atan_val<Real, TK>(y / x);
        if (x < BIKE_REAL(Real, 0.0)) {
            return (y >= BIKE_REAL(Real, 0.0)) ? (r + kPi) : (r - kPi);
        }
        return r;
    }

    template <class Real, class TK>
    BIKE_HD static Real sin_val(Real x_in) noexcept {
        const Real x = flush_subnorm(x_in);
        if (!TK::isfinite(x)) {
            return static_cast<Real>(pf_u32_to_f32(0x7fc00000u));
        }
        if (x == BIKE_REAL(Real, 0.0)) {
            return x;
        }
        const Real sign = (x < BIKE_REAL(Real, 0.0)) ? BIKE_REAL(Real, -1.0) : BIKE_REAL(Real, 1.0);
        const Real ax = TK::abs(x);
        const Real k = TK::floor(ax * BIKE_REAL(Real, 0.63661977236758134308) + BIKE_REAL(Real, 0.5));
        const u32 j = static_cast<u32>(static_cast<i32>(k));
        const Real dp1 = BIKE_REAL(Real, 1.5703125);
        const Real dp2 = BIKE_REAL(Real, 4.837512969970703125e-4);
        const Real dp3 = BIKE_REAL(Real, 7.549789948768648e-8);
        const Real r = ((ax - k * dp1) - k * dp2) - k * dp3;
        const Real r2 = r * r;
        const u32 q = j & 3u;
        if ((q & 1u) == 0u) {
            const Real s = r + r * r2 *
                (BIKE_REAL(Real, -1.6666654611e-1) +
                 r2 * (BIKE_REAL(Real, 8.3321608736e-3) -
                       r2 * BIKE_REAL(Real, 1.9515295891e-4)));
            return (q == 2u) ? (-sign * s) : (sign * s);
        }
        const Real c = BIKE_REAL(Real, 1.0) - BIKE_REAL(Real, 0.5) * r2 +
            r2 * r2 *
                (BIKE_REAL(Real, 4.166664568298827e-2) +
                 r2 * (BIKE_REAL(Real, -1.388731625493765e-3) +
                       r2 * BIKE_REAL(Real, 2.443315711809948e-5)));
        return (q == 3u) ? (-sign * c) : (sign * c);
    }

    template <class Real, class TK>
    BIKE_HD static Real cos_val(Real x_in) noexcept {
        const Real x = flush_subnorm(x_in);
        if (!TK::isfinite(x)) {
            return static_cast<Real>(pf_u32_to_f32(0x7fc00000u));
        }
        const Real ax = TK::abs(x);
        const Real k = TK::floor(ax * BIKE_REAL(Real, 0.63661977236758134308) + BIKE_REAL(Real, 0.5));
        const u32 j = static_cast<u32>(static_cast<i32>(k));
        const Real dp1 = BIKE_REAL(Real, 1.5703125);
        const Real dp2 = BIKE_REAL(Real, 4.837512969970703125e-4);
        const Real dp3 = BIKE_REAL(Real, 7.549789948768648e-8);
        const Real r = ((ax - k * dp1) - k * dp2) - k * dp3;
        const Real r2 = r * r;
        const u32 q = j & 3u;
        if ((q & 1u) == 0u) {
            const Real c = BIKE_REAL(Real, 1.0) - BIKE_REAL(Real, 0.5) * r2 +
                r2 * r2 *
                    (BIKE_REAL(Real, 4.166664568298827e-2) +
                     r2 * (BIKE_REAL(Real, -1.388731625493765e-3) +
                           r2 * BIKE_REAL(Real, 2.443315711809948e-5)));
            return (q == 2u) ? -c : c;
        }
        const Real s = r + r * r2 *
            (BIKE_REAL(Real, -1.6666654611e-1) +
             r2 * (BIKE_REAL(Real, 8.3321608736e-3) -
                   r2 * BIKE_REAL(Real, 1.9515295891e-4)));
        return (q == 1u) ? -s : s;
    }
};

template <class Real>
struct SurfaceSectionCore {
    Real start_m = BIKE_REAL(Real, 0.0);
    Real end_m = BIKE_REAL(Real, 0.0);
    SurfaceLabel label = SurfaceLabel::hardpack;
    u8 pad[3]{};
};

template <class Real, class PtrSections>
BIKE_HD Status surface_labels_at(
    SurfaceLabel default_label,
    i32 section_count,
    PtrSections sections,
    Real x_m,
    BIKE_THREAD SurfaceLabel &out_label,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_label = default_label;
    if (section_count < 0) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (section_count > 0 && sections == nullptr) {
        err = ErrCode::out_of_bounds_chain;
        return Status::invalid_argument;
    }
    for (i32 i = 0; i < section_count; ++i) {
        if (sections[i].start_m <= x_m && x_m < sections[i].end_m) {
            out_label = sections[i].label;
            return Status::ok;
        }
    }
    return Status::ok;
}

namespace detail {

template <class Real, class PtrX>
BIKE_HD i32 find_road_segment(i32 vertex_count, PtrX road_x, Real x) noexcept {
    i32 lo = 0;
    i32 hi = vertex_count;
    while (lo < hi) {
        const i32 mid = lo + (hi - lo) / 2;
        if (road_x[mid] <= x) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    i32 seg = lo - 1;
    if (seg < 0) {
        seg = 0;
    }
    if (seg > vertex_count - 2) {
        seg = vertex_count - 2;
    }
    return seg;
}

template <class Real, class PtrX, class PtrZ>
BIKE_HD Real road_segment_grade(PtrX road_x, PtrZ road_z, i32 seg) noexcept {
    const Real dx = road_x[seg + 1] - road_x[seg];
    if (dx <= BIKE_REAL(Real, 0.0)) {
        return BIKE_REAL(Real, 0.0);
    }
    return (road_z[seg + 1] - road_z[seg]) / dx;
}

template <class Real, class PtrX, class PtrZ>
BIKE_HD Real road_height_at(i32 vertex_count, PtrX road_x, PtrZ road_z, Real x) noexcept {
    if (x <= road_x[0]) {
        return road_z[0];
    }
    if (x >= road_x[vertex_count - 1]) {
        return road_z[vertex_count - 1];
    }
    const i32 seg = find_road_segment<Real>(vertex_count, road_x, x);
    if (seg >= vertex_count - 1 || x == road_x[seg]) {
        return road_z[seg];
    }
    const Real dx = road_x[seg + 1] - road_x[seg];
    if (dx <= BIKE_REAL(Real, 0.0)) {
        return road_z[seg];
    }
    const Real slope = (road_z[seg + 1] - road_z[seg]) / dx;
    return slope * (x - road_x[seg]) + road_z[seg];
}

} // namespace detail

template <class Real, class TK, class PtrX, class PtrZ>
BIKE_HD Status eval_road_grades(
    i32 vertex_count,
    PtrX road_x,
    PtrZ road_z,
    Real front_wheel_x,
    Real rear_wheel_x,
    Real lookahead_m,
    Real spacing_m,
    BIKE_THREAD Real &out_posture_grade,
    BIKE_THREAD Real &out_preview_grade,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_posture_grade = BIKE_REAL(Real, 0.0);
    out_preview_grade = BIKE_REAL(Real, 0.0);
    if (vertex_count < 2 || road_x == nullptr || road_z == nullptr) {
        err = ErrCode::out_of_bounds_chain;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(front_wheel_x) || !TK::isfinite(rear_wheel_x) ||
        !TK::isfinite(lookahead_m) || !TK::isfinite(spacing_m) ||
        lookahead_m < BIKE_REAL(Real, 0.0) || spacing_m <= BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }

    const Real w_lo = TK::min(front_wheel_x, rear_wheel_x);
    const Real w_hi = TK::max(front_wheel_x, rear_wheel_x);
    const i32 count = static_cast<i32>(
        TK::floor(lookahead_m / spacing_m + BIKE_REAL(Real, 1e-6)));

    i32 seg = detail::find_road_segment<Real>(vertex_count, road_x, w_lo);
    Real sum_grade = detail::road_segment_grade<Real>(road_x, road_z, seg);

    while (seg + 1 < vertex_count - 1 && road_x[seg + 1] <= w_hi) {
        ++seg;
    }
    sum_grade += detail::road_segment_grade<Real>(road_x, road_z, seg);

    for (i32 k = 0; k < count; ++k) {
        const Real x = w_hi + static_cast<Real>(k + 1) * spacing_m;
        while (seg + 1 < vertex_count - 1 && road_x[seg + 1] <= x) {
            ++seg;
        }
        sum_grade += detail::road_segment_grade<Real>(road_x, road_z, seg);
    }

    const Real x_end = w_hi + lookahead_m;
    const i32 end_seg = detail::find_road_segment<Real>(vertex_count, road_x, x_end);
    const Real end_grade = detail::road_segment_grade<Real>(road_x, road_z, end_seg);
    sum_grade += end_grade;

    out_posture_grade = sum_grade / static_cast<Real>(count + 3);
    const Real dx_chord = x_end - front_wheel_x;
    if (dx_chord > BIKE_REAL(Real, 1e-9)) {
        const Real h_front = detail::road_height_at<Real>(vertex_count, road_x, road_z, front_wheel_x);
        const Real h_end = detail::road_height_at<Real>(vertex_count, road_x, road_z, x_end);
        out_preview_grade = (h_end - h_front) / dx_chord;
    } else {
        out_preview_grade = end_grade;
    }
    return Status::ok;
}

template <class Real>
struct PerceptionState {
    SenseSampleCore<Real> samples[kPerceptionRingCap]{};
    SenseSampleCore<Real> delayed{};
    i32 tick = 0;
    i32 ring_head = 0;
    i32 ring_count = 0;
    Real pitch_rad = BIKE_REAL(Real, 0.0);
    Real initial_grade_rad = BIKE_REAL(Real, 0.0);
    u8 has_delayed = 0;
    u8 has_initial_grade = 0;
    u8 pad[2]{};
};

template <class Real>
struct BodyPulseCore {
    Real start_s = BIKE_REAL(Real, 0.0);
    Real duration_s = BIKE_REAL(Real, 0.0);
    Real amplitude_rad = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct SeatedPostureState {
    Real load_samples[kPostureRingCap]{};
    BodyPulseCore<Real> pulses[kMaxPosturePulses]{};
    i32 tick = 0;
    i32 ring_head = 0;
    i32 ring_count = 0;
    i32 pulse_count = 0;
    Real lean_rad = BIKE_REAL(Real, 0.0);
    Real trim_rad = BIKE_REAL(Real, 0.0);
    Real delayed_load_share = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct SeatedClimbState {
    i32 surge_subticks = 0;
    Real inclination_rad = BIKE_REAL(Real, 0.0);
    Real lean_rad = BIKE_REAL(Real, 0.0);
    Real effort_nm = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct RiderPostureCore {
    Real torso_lean_rad = BIKE_REAL(Real, 0.0);
    Real pelvis_pitch_rad = BIKE_REAL(Real, 0.0);
    Real pelvis_offset_x_m = BIKE_REAL(Real, 0.0);
    Real pelvis_offset_z_m = BIKE_REAL(Real, 0.0);
    u8 has_pelvis_offset = 0;
    u8 use_saddle = 1;
    u8 pad[2]{};
};

template <class Real>
struct IntentControlCore {
    Real motor_torque_nm = BIKE_REAL(Real, 0.0);
    Real motor_limit_nm = BIKE_REAL(Real, 0.0);
    Real human_torque_nm = BIKE_REAL(Real, 0.0);
    Real crank_target_rate_rad_s = BIKE_REAL(Real, 0.0);
    RiderPostureCore<Real> posture{};
    u8 has_motor_torque_nm = 0;
    u8 has_motor_limit_nm = 0;
    u8 has_human_torque_nm = 0;
    u8 has_crank_target_rate_rad_s = 0;
    u8 has_posture = 0;
    u8 rider_enabled = 1;
    u8 pad[2]{};
};

template <class Real>
struct RiderCommandsCore {
    Real front_brake = BIKE_REAL(Real, 0.0);
    Real rear_brake = BIKE_REAL(Real, 0.0);
    i32 stationary_rear_teeth = -1;
    ShiftRequest shift_request = ShiftRequest::none;
    u8 owns_shifting = 0;
    u8 pad[2]{};
};

struct LaunchContextCore {
    ShiftFeedbackCore shift{};
    u8 effort_mode = 0;
    u8 external_brake = 0;
    u8 pad[2]{};
};

template <class Real>
struct IntentResolutionCore {
    IntentControlCore<Real> control{};
    u8 effort_limited = 0;
    u8 pad[3]{};
};

template <class Real>
struct IntentParams {
    u8 enabled = 0;
    u8 pad[3]{};
    Real period_s = BIKE_REAL(Real, 0.00125);
    i32 period_steps = 1;
    i32 delay_ticks = 120;
    i32 dead_time_ticks = 160;
    Real target_crank_power_w = BIKE_REAL(Real, 225.0);
    Real max_crank_torque_nm = BIKE_REAL(Real, 60.0);
    Real lean_gain = BIKE_REAL(Real, 1.0);
    Real max_forward_lean_rad = BIKE_REAL(Real, 0.5);
    Real max_backward_lean_rad = BIKE_REAL(Real, 0.10);
    Real lean_rate_rad_s = BIKE_REAL(Real, 1.0);
    Real orientation_tau_s = BIKE_REAL(Real, 0.5);
    Real surge_power_w = BIKE_REAL(Real, 400.0);
    Real surge_grade = BIKE_REAL(Real, 0.20);
    i32 surge_max_subticks = 12000 * kSurgeSubTicks;
    i32 surge_recover_subticks = 200;
    Real front_load_share_target = BIKE_REAL(Real, 0.30);
    Real lean_trim_gain_rad_s = BIKE_REAL(Real, 0.3);
    Real lean_trim_limit_rad = BIKE_REAL(Real, 0.15);
    Real min_elbow_flexion_deg = BIKE_REAL(Real, 15.0);

    i32 climb_effort_cap_count = 0;
    Real climb_effort_cap_grades[kMaxLaunchKnots]{};
    Real climb_effort_cap_grade_nm[kMaxLaunchKnots]{};
    Real climb_effort_rise_speed_mps = BIKE_REAL(Real, 0.0);
    Real climb_effort_full_speed_mps = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct RiderIntentState {
    PerceptionState<Real> perception{};
    SeatedPostureState<Real> posture{};
    SeatedClimbState<Real> climb{};
    FrontLiftState<Real> reaction{};
    LaunchState<Real> launch{};
    i32 last_tick_step = -1;
    RiderPostureCore<Real> intent_posture{};
    Real intent_effort_ceiling_nm = BIKE_REAL(Real, 0.0);
    RiderCommandsCore<Real> commands{};
};

template <class Real>
BIKE_HD Real core_quiet_nan() noexcept {
    return static_cast<Real>(pf_u32_to_f32(0x7fc00000u));
}

template <class Real, class TK, class SamplePtr>
BIKE_HD Status validate_sense_sample_core(
    SamplePtr s,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (!TK::isfinite(s->signals.pitch_rate_up_rad_s) ||
        !TK::isfinite(s->signals.crank_rate_rad_s) ||
        !TK::isfinite(s->signals.human_crank_torque_nm) ||
        !TK::isfinite(s->signals.specific_force_body_mps2[0]) ||
        !TK::isfinite(s->signals.specific_force_body_mps2[1]) ||
        !TK::isfinite(s->signals.specific_force_body_mps2[2])) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (s->signals.has_front_load_share != 0) {
        if (!TK::isfinite(s->signals.front_load_share) ||
            s->signals.front_load_share < BIKE_REAL(Real, 0.0) ||
            s->signals.front_load_share > BIKE_REAL(Real, 1.0)) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
    }
    if (!TK::isfinite(s->senses.hand_press_n) ||
        !TK::isfinite(s->senses.speed_mps) ||
        !TK::isfinite(s->senses.grade_ahead) ||
        s->senses.rear_teeth < 3 ||
        static_cast<u8>(s->senses.surface_rear) > static_cast<u8>(SurfaceLabel::scree) ||
        static_cast<u8>(s->senses.surface_front) > static_cast<u8>(SurfaceLabel::scree) ||
        static_cast<u8>(s->senses.surface_ahead) > static_cast<u8>(SurfaceLabel::scree)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    return Status::ok;
}

template <class Real, class StatePtr>
BIKE_HD void perception_reset(StatePtr state) noexcept {
    state->tick = 0;
    state->ring_head = 0;
    state->ring_count = 0;
    state->pitch_rad = BIKE_REAL(Real, 0.0);
    state->initial_grade_rad = BIKE_REAL(Real, 0.0);
    state->has_delayed = 0;
    state->has_initial_grade = 0;
    state->delayed = SenseSampleCore<Real>{};
}

template <class Real, class TK, class Trig, class StatePtr, class SamplePtr>
BIKE_HD Status perception_update_with_trig(
    i32 delay_ticks,
    Real period_s,
    SamplePtr sample,
    StatePtr state,
    BIKE_THREAD bool &out_has_percepts,
    BIKE_THREAD PerceptsCore<Real> &out_percepts,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_has_percepts = false;
    out_percepts = PerceptsCore<Real>{};
    if (!TK::isfinite(period_s) || period_s <= BIKE_REAL(Real, 0.0) ||
        delay_ticks < 0 || delay_ticks > kPerceptionRingCap) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (state->tick >= kMaxI32Ticks) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    if (state->ring_head < 0 || state->ring_count < 0 || state->ring_count > delay_ticks) {
        err = ErrCode::out_of_bounds_station;
        return Status::out_of_bounds;
    }
    const Status sv = validate_sense_sample_core<Real, TK>(sample, err);
    if (sv != Status::ok) {
        return sv;
    }

    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>; exact FP64/FP32 2*pi literal
    const Real kTwoPi = BIKE_REAL(Real, 6.28318530717958647692);
    SenseSampleCore<Real> delivered_sample{};
    bool has_delivery = false;
    if (delay_ticks == 0) {
        delivered_sample = *sample;
        has_delivery = true;
    } else if (state->ring_count == delay_ticks) {
        delivered_sample = state->samples[state->ring_head];
        has_delivery = true;
    }

    Real next_pitch = state->pitch_rad;
    Real next_initial_grade = state->initial_grade_rad;
    u8 next_has_initial_grade = state->has_initial_grade;
    if (has_delivery) {
        next_pitch = state->pitch_rad +
                     delivered_sample.signals.pitch_rate_up_rad_s * period_s;
        if (!TK::isfinite(next_pitch)) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }
        const Real trail_rad = Trig::template atan_val<Real, TK>(
            delivered_sample.senses.grade_ahead);
        const Real init_rad =
            (state->has_initial_grade != 0) ? state->initial_grade_rad : trail_rad;
        const Real pitch_rel = next_pitch - (trail_rad - init_rad);
        const Real crank_rpm =
            TK::abs(delivered_sample.signals.crank_rate_rad_s) *
            BIKE_REAL(Real, 60.0) / kTwoPi;
        if (!TK::isfinite(pitch_rel) || !TK::isfinite(crank_rpm)) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }
        next_initial_grade = init_rad;
        next_has_initial_grade = 1;
        out_has_percepts = true;
        out_percepts.senses = delivered_sample.senses;
        out_percepts.crank_rpm = crank_rpm;
        out_percepts.human_crank_torque_nm =
            delivered_sample.signals.human_crank_torque_nm;
        out_percepts.pitch_rate_up_rad_s =
            delivered_sample.signals.pitch_rate_up_rad_s;
        out_percepts.pitch_rad = pitch_rel;
    }

    if (delay_ticks == 0) {
        state->delayed = *sample;
        state->has_delayed = 1;
    } else if (state->ring_count < delay_ticks) {
        const i32 write_idx = (state->ring_head + state->ring_count) % delay_ticks;
        state->samples[write_idx] = *sample;
        ++state->ring_count;
    } else {
        state->delayed = delivered_sample;
        state->has_delayed = 1;
        state->samples[state->ring_head] = *sample;
        state->ring_head = (state->ring_head + 1) % delay_ticks;
    }
    ++state->tick;
    state->pitch_rad = next_pitch;
    state->initial_grade_rad = next_initial_grade;
    state->has_initial_grade = next_has_initial_grade;
    return Status::ok;
}

template <class Real, class TK, class StatePtr, class SamplePtr>
BIKE_HD Status perception_update(
    i32 delay_ticks,
    Real period_s,
    SamplePtr sample,
    StatePtr state,
    BIKE_THREAD bool &out_has_percepts,
    BIKE_THREAD PerceptsCore<Real> &out_percepts,
    BIKE_THREAD ErrCode &err
) noexcept {
    return perception_update_with_trig<Real, TK, PortableTrig>(
        delay_ticks, period_s, sample, state, out_has_percepts, out_percepts, err);
}

template <class Real, class StatePtr>
BIKE_HD void posture_program_reset(StatePtr state) noexcept {
    state->tick = 0;
    state->ring_head = 0;
    state->ring_count = 0;
    state->pulse_count = 0;
    state->lean_rad = BIKE_REAL(Real, 0.0);
    state->trim_rad = BIKE_REAL(Real, 0.0);
    state->delayed_load_share = core_quiet_nan<Real>();
}

template <class Real, class TK>
BIKE_HD Status body_pulse_eval(
    Real time_s,
    Real start_s,
    Real duration_s,
    Real amplitude_rad,
    BIKE_THREAD Real &out_val,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_val = BIKE_REAL(Real, 0.0);
    if (!TK::isfinite(time_s) || !TK::isfinite(start_s) ||
        !TK::isfinite(duration_s) || !TK::isfinite(amplitude_rad) ||
        duration_s <= BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Real phase = (time_s - start_s) / duration_s;
    if (!(BIKE_REAL(Real, 0.0) < phase && phase < BIKE_REAL(Real, 1.0))) {
        return Status::ok;
    }
    out_val = amplitude_rad * BIKE_REAL(Real, 64.0) *
              TK::pow3(phase) * TK::pow3(BIKE_REAL(Real, 1.0) - phase);
    return Status::ok;
}

template <class Real, class TK, class StatePtr>
BIKE_HD Status posture_program_schedule_pulse(
    StatePtr state,
    Real start_s,
    Real duration_s,
    Real amplitude_rad,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (!TK::isfinite(start_s) || !TK::isfinite(duration_s) ||
        !TK::isfinite(amplitude_rad) || duration_s <= BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (state->pulse_count >= kMaxPosturePulses) {
        err = ErrCode::out_of_bounds_station;
        return Status::overflow;
    }
    const i32 idx = state->pulse_count;
    state->pulses[idx].start_s = start_s;
    state->pulses[idx].duration_s = duration_s;
    state->pulses[idx].amplitude_rad = amplitude_rad;
    ++state->pulse_count;
    return Status::ok;
}

template <class Real, class TK, class Trig, class CfgPtr, class StatePtr>
BIKE_HD Status posture_program_update_with_trig(
    CfgPtr cfg,
    Real road_grade,
    Real dt_s,
    bool has_front_load_share,
    Real front_load_share,
    bool has_lean_limit,
    Real lean_limit_rad,
    i32 dead_time_ticks,
    bool has_min_lean,
    Real min_lean_rad,
    Real extra_target_rad,
    bool has_launch,
    Real launch_target_rad,
    Real launch_weight,
    StatePtr state,
    BIKE_THREAD Real &out_lean_rad,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_lean_rad = state->lean_rad;
    if (!TK::isfinite(road_grade) || !TK::isfinite(dt_s) || dt_s <= BIKE_REAL(Real, 0.0) ||
        dead_time_ticks < 0 || dead_time_ticks > kPostureRingCap ||
        !TK::isfinite(extra_target_rad) || extra_target_rad < BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (has_front_load_share &&
        (!TK::isfinite(front_load_share) ||
         front_load_share < BIKE_REAL(Real, 0.0) ||
         front_load_share > BIKE_REAL(Real, 1.0))) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (has_lean_limit &&
        (!TK::isfinite(lean_limit_rad) || lean_limit_rad < BIKE_REAL(Real, 0.0))) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (has_min_lean && !TK::isfinite(min_lean_rad)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (has_launch &&
        (!TK::isfinite(launch_target_rad) ||
         launch_weight <= BIKE_REAL(Real, 0.0) ||
         launch_weight > BIKE_REAL(Real, 1.0))) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (state->ring_head < 0 || state->ring_count < 0 || state->ring_count > dead_time_ticks ||
        state->pulse_count < 0 || state->pulse_count > kMaxPosturePulses) {
        err = ErrCode::out_of_bounds_station;
        return Status::out_of_bounds;
    }

    const Real time_s = static_cast<Real>(state->tick) * dt_s;
    Real extra_pulses = BIKE_REAL(Real, 0.0);
    for (i32 i = 0; i < state->pulse_count; ++i) {
        Real pval = BIKE_REAL(Real, 0.0);
        const Status pst = body_pulse_eval<Real, TK>(
            time_s, state->pulses[i].start_s, state->pulses[i].duration_s,
            state->pulses[i].amplitude_rad, pval, err);
        if (pst != Status::ok) {
            return pst;
        }
        extra_pulses += pval;
    }

    const Real incoming = has_front_load_share ? front_load_share : core_quiet_nan<Real>();
    if (dead_time_ticks == 0) {
        state->delayed_load_share = incoming;
    } else if (state->ring_count < dead_time_ticks) {
        const i32 write_idx = (state->ring_head + state->ring_count) % dead_time_ticks;
        state->load_samples[write_idx] = incoming;
        ++state->ring_count;
    } else {
        state->delayed_load_share = state->load_samples[state->ring_head];
        state->load_samples[state->ring_head] = incoming;
        state->ring_head = (state->ring_head + 1) % dead_time_ticks;
    }
    ++state->tick;

    const Real forward_limit = has_lean_limit ? lean_limit_rad : cfg->max_forward_lean_rad;
    const Real backward_limit =
        has_min_lean
            ? TK::min(forward_limit, TK::max(-cfg->max_backward_lean_rad, min_lean_rad))
            : -cfg->max_backward_lean_rad;

    Real extra = extra_pulses;
    if (extra_target_rad != BIKE_REAL(Real, 0.0)) {
        extra += extra_target_rad;
    }
    const bool plan_owns = has_launch && (launch_weight == BIKE_REAL(Real, 1.0));
    Real base = cfg->lean_gain * Trig::template atan_val<Real, TK>(road_grade);
    if (has_launch) {
        base = plan_owns
                   ? launch_target_rad
                   : (launch_weight * launch_target_rad +
                      (BIKE_REAL(Real, 1.0) - launch_weight) * base);
    }
    const Real grade_and_pulse = base + extra;
    if (!TK::isnan(state->delayed_load_share) && !plan_owns) {
        state->trim_rad +=
            cfg->lean_trim_gain_rad_s *
            (cfg->front_load_share_target - state->delayed_load_share) * dt_s;
        const Real min_trim =
            has_min_lean
                ? TK::max(-cfg->lean_trim_limit_rad, backward_limit - grade_and_pulse)
                : -cfg->lean_trim_limit_rad;
        state->trim_rad = TK::max(
            min_trim, TK::min(cfg->lean_trim_limit_rad, state->trim_rad));
    }
    const Real trim =
        plan_owns
            ? BIKE_REAL(Real, 0.0)
            : (has_launch ? ((BIKE_REAL(Real, 1.0) - launch_weight) * state->trim_rad)
                          : state->trim_rad);
    const Real target = TK::max(
        backward_limit, TK::min(forward_limit, grade_and_pulse + trim));
    const Real max_step = cfg->lean_rate_rad_s * dt_s;
    state->lean_rad += TK::max(-max_step, TK::min(max_step, target - state->lean_rad));
    if (has_min_lean && state->lean_rad < backward_limit) {
        state->lean_rad = backward_limit;
    }
    out_lean_rad = state->lean_rad;
    return Status::ok;
}

template <class Real, class TK, class CfgPtr, class StatePtr>
BIKE_HD Status posture_program_update(
    CfgPtr cfg,
    Real road_grade,
    Real dt_s,
    bool has_front_load_share,
    Real front_load_share,
    bool has_lean_limit,
    Real lean_limit_rad,
    i32 dead_time_ticks,
    bool has_min_lean,
    Real min_lean_rad,
    Real extra_target_rad,
    bool has_launch,
    Real launch_target_rad,
    Real launch_weight,
    StatePtr state,
    BIKE_THREAD Real &out_lean_rad,
    BIKE_THREAD ErrCode &err
) noexcept {
    return posture_program_update_with_trig<Real, TK, PortableTrig>(
        cfg, road_grade, dt_s, has_front_load_share, front_load_share,
        has_lean_limit, lean_limit_rad, dead_time_ticks, has_min_lean,
        min_lean_rad, extra_target_rad, has_launch, launch_target_rad,
        launch_weight, state, out_lean_rad, err);
}

namespace detail {

template <class Real, class TK, class Trig>
BIKE_HD Real wrap_angle_with_trig(Real angle_rad) noexcept {
    return Trig::template atan2_val<Real, TK>(
        Trig::template sin_val<Real, TK>(angle_rad),
        Trig::template cos_val<Real, TK>(angle_rad)
    );
}

template <class Real, class TK>
BIKE_HD Real crank_effort_ceiling_core(
    Real power_w,
    Real torque_limit_nm,
    Real crank_rate_rad_s
) noexcept {
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>; exact FP64/FP32 2*pi literal
    const Real kTwoPi = BIKE_REAL(Real, 6.28318530717958647692);
    const Real speed_floor = BIKE_REAL(Real, 20.0) * kTwoPi / BIKE_REAL(Real, 60.0);
    const Real denom = TK::max(BIKE_REAL(Real, 0.0), TK::max(crank_rate_rad_s, speed_floor));
    return TK::min(torque_limit_nm, power_w / denom);
}

template <class Real, class TK>
BIKE_HD Real vector_norm3_py(Real a, Real b, Real c) noexcept {
    const Real aa = TK::abs(a);
    const Real ab = TK::abs(b);
    const Real ac = TK::abs(c);
    const Real mx = TK::max(aa, TK::max(ab, ac));
    if (mx == BIKE_REAL(Real, 0.0)) {
        return BIKE_REAL(Real, 0.0);
    }
    const Real ra = aa / mx;
    const Real rb = ab / mx;
    const Real rc = ac / mx;
    return mx * TK::sqrt(ra * ra + rb * rb + rc * rc);
}

template <class Real, class TK, class CfgPtr, class PerceptsPtr>
BIKE_HD Real compute_climb_cap_nm(
    CfgPtr cfg,
    PerceptsPtr percepts
) noexcept {
    const Real base_cap = piecewise_linear_knot<Real, TK>(
        cfg->climb_effort_cap_count,
        cfg->climb_effort_cap_grades,
        cfg->climb_effort_cap_grade_nm,
        percepts->senses.grade_ahead
    );
    const Real speed = percepts->senses.speed_mps;
    if (!(speed > cfg->climb_effort_rise_speed_mps)) {
        return base_cap;
    }
    if (speed >= cfg->climb_effort_full_speed_mps ||
        cfg->climb_effort_full_speed_mps <= cfg->climb_effort_rise_speed_mps) {
        return cfg->max_crank_torque_nm;
    }
    const Real alpha =
        (speed - cfg->climb_effort_rise_speed_mps) /
        (cfg->climb_effort_full_speed_mps - cfg->climb_effort_rise_speed_mps);
    return base_cap + alpha * (cfg->max_crank_torque_nm - base_cap);
}

template <class Real, class CfgPtr, class StatePtr>
BIKE_HD Real climb_power_target_w(
    CfgPtr cfg,
    Real preview_grade,
    bool spend_surge,
    StatePtr climb
) noexcept {
    if (!spend_surge) {
        return cfg->target_crank_power_w;
    }
    if (preview_grade >= cfg->surge_grade) {
        if (climb->surge_subticks >= kSurgeSubTicks) {
            climb->surge_subticks -= kSurgeSubTicks;
            return cfg->surge_power_w;
        }
    } else {
        const i32 recovered = climb->surge_subticks + cfg->surge_recover_subticks;
        climb->surge_subticks =
            recovered < cfg->surge_max_subticks ? recovered : cfg->surge_max_subticks;
    }
    return cfg->target_crank_power_w;
}

template <class Real, class LaunchCfgPtr, class SensesPtr>
BIKE_HD RiderCommandsCore<Real> commands_of_core(
    LaunchCommandCore<Real> c,
    bool has_launch,
    LaunchCfgPtr launch_cfg,
    bool has_delayed_senses,
    SensesPtr delayed_senses
) noexcept {
    RiderCommandsCore<Real> out{};
    out.front_brake = c.brake;
    out.rear_brake = c.brake;
    out.shift_request = (c.shift_up != 0) ? ShiftRequest::up : ShiftRequest::none;
    out.owns_shifting = c.owns_shifting;
    out.stationary_rear_teeth = -1;
    if (c.phase == LaunchPhase::hold && has_launch && has_delayed_senses &&
        delayed_senses->grade_ahead > BIKE_REAL(Real, 0.0)) {
        const i32 target = launch_teeth_unchecked<Real>(launch_cfg, delayed_senses->grade_ahead);
        if (delayed_senses->rear_teeth < target) {
            out.stationary_rear_teeth = target;
        }
    }
    return out;
}

} // namespace detail

template <class Real, class CfgPtr, class StatePtr>
BIKE_HD void climb_policy_reset(
    CfgPtr cfg,
    StatePtr climb
) noexcept {
    climb->surge_subticks = cfg->surge_max_subticks;
    climb->inclination_rad = BIKE_REAL(Real, 0.0);
    climb->lean_rad = BIKE_REAL(Real, 0.0);
    climb->effort_nm = BIKE_REAL(Real, 0.0);
}

template <class Real, class CfgPtr, class StatePtr>
BIKE_HD void rider_intent_reset(
    CfgPtr intent_cfg,
    StatePtr state
) noexcept {
    perception_reset<Real>(&state->perception);
    posture_program_reset<Real>(&state->posture);
    climb_policy_reset<Real>(intent_cfg, &state->climb);
    front_lift_reset<Real>(&state->reaction);
    launch_reset<Real>(&state->launch);
    state->last_tick_step = -1;
    state->intent_posture = RiderPostureCore<Real>{};
    state->intent_effort_ceiling_nm = BIKE_REAL(Real, 0.0);
    state->commands = RiderCommandsCore<Real>{};
}

template <class Real, class TK, class CfgPtr, class StatePtr>
BIKE_HD Status rider_intent_seed_lean(
    CfgPtr intent_cfg,
    Real lean_rad,
    StatePtr state,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (!TK::isfinite(lean_rad) ||
        lean_rad > intent_cfg->max_forward_lean_rad ||
        lean_rad < -intent_cfg->max_backward_lean_rad) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    state->posture.lean_rad = lean_rad;
    state->climb.lean_rad = lean_rad;
    state->intent_posture.torso_lean_rad = lean_rad;
    return Status::ok;
}

template <class Real, class TK, class IntentCfgPtr, class LaunchCfgPtr, class StatePtr>
BIKE_HD Status rider_intent_arm_standstill_start(
    IntentCfgPtr intent_cfg,
    LaunchCfgPtr launch_cfg,
    Real lean_rad,
    StatePtr state,
    BIKE_THREAD bool &out_armed,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_armed = false;
    if (intent_cfg->enabled == 0) {
        return Status::ok;
    }
    const Status st = launch_arm_start<Real, TK>(
        launch_cfg, lean_rad, &state->launch, out_armed, err);
    if (st != Status::ok || !out_armed) {
        return st;
    }
    const LaunchCommandCore<Real> cmd = launch_command<Real>(launch_cfg, &state->launch);
    const BIKE_THREAD RiderSensesCore<Real> *no_senses = nullptr;
    state->commands = detail::commands_of_core<Real>(
        cmd, launch_cfg->plan_enabled != 0, launch_cfg, false, no_senses);
    return Status::ok;
}

template <class Real, class TK, class Trig, class IntentCfgPtr, class FrontLiftCfgPtr, class LaunchCfgPtr, class StatePtr>
BIKE_HD Status rider_intent_resolve_with_trig(
    IntentCfgPtr intent_cfg,
    FrontLiftCfgPtr reaction_cfg,
    LaunchCfgPtr launch_cfg,
    IntentControlCore<Real> in_control,
    IntentSignalsCore<Real> signals,
    RiderSensesCore<Real> senses,
    i32 step,
    Real road_grade,
    bool has_preview_grade,
    Real preview_grade,
    bool has_lean_limit,
    Real lean_limit_rad,
    bool active,
    bool has_min_lean,
    Real min_lean_rad,
    LaunchContextCore launch_ctx,
    StatePtr state,
    BIKE_THREAD IntentResolutionCore<Real> &out_res,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    out_res = IntentResolutionCore<Real>{};
    out_res.control = in_control;
    if (intent_cfg->enabled == 0 || !active) {
        return Status::ok;
    }
    if (step < 0 || step < state->last_tick_step) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (intent_cfg->period_steps <= 0) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }

    const bool due =
        (step % intent_cfg->period_steps == 0) && (step != state->last_tick_step);
    if (due) {
        const bool on_schedule =
            (state->last_tick_step < 0)
                ? (step == 0)
                : ((step - state->last_tick_step) == intent_cfg->period_steps);
        if (!on_schedule) {
            err = ErrCode::bad_time;
            return Status::invalid_argument;
        }
        if (!TK::isfinite(road_grade) ||
            (has_preview_grade && !TK::isfinite(preview_grade)) ||
            (has_lean_limit &&
             (!TK::isfinite(lean_limit_rad) || lean_limit_rad < BIKE_REAL(Real, 0.0))) ||
            (has_min_lean && !TK::isfinite(min_lean_rad))) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        const SenseSampleCore<Real> sample{signals, senses};
        if (validate_sense_sample_core<Real, TK>(&sample, err) != Status::ok) {
            return Status::invalid_argument;
        }
        if (launch_validate_inputs<Real, TK>(
                launch_cfg, false, static_cast<const BIKE_THREAD PerceptsCore<Real> *>(nullptr),
                launch_ctx.shift.shift_count, err) != Status::ok) {
            return Status::invalid_argument;
        }
        if (front_lift_validate_inputs<Real, TK>(
                reaction_cfg, false, static_cast<const BIKE_THREAD PerceptsCore<Real> *>(nullptr),
                err) != Status::ok) {
            return Status::invalid_argument;
        }
        if (intent_cfg->delay_ticks < 0 || intent_cfg->delay_ticks > kPerceptionRingCap ||
            intent_cfg->dead_time_ticks < 0 || intent_cfg->dead_time_ticks > kPostureRingCap ||
            state->perception.tick >= kMaxI32Ticks) {
            err = ErrCode::out_of_bounds_station;
            return Status::out_of_bounds;
        }

        // 1. Perception update (delivers delayed SenseSampleCore -> PerceptsCore)
        bool has_percepts = false;
        PerceptsCore<Real> percepts{};
        const Status pst = perception_update_with_trig<Real, TK, Trig>(
            intent_cfg->delay_ticks, intent_cfg->period_s, &sample,
            &state->perception, has_percepts, percepts, err);
        if (pst != Status::ok) {
            return pst;
        }

        // 2. FrontLiftReaction update
        ReactionCommandCore<Real> react_cmd{};
        const Status rst = front_lift_update<Real, TK>(
            reaction_cfg, has_percepts, &percepts, &state->reaction, react_cmd, err);
        if (rst != Status::ok) {
            return rst;
        }

        // 3. LaunchPlan update
        const bool eligible =
            (launch_ctx.effort_mode != 0) &&
            (in_control.rider_enabled != 0) &&
            (in_control.has_human_torque_nm == 0);
        LaunchCommandCore<Real> plan_cmd{};
        const Status lst = launch_update<Real, TK>(
            launch_cfg, has_percepts, &percepts, launch_ctx.shift, eligible,
            launch_ctx.external_brake != 0, &state->launch, plan_cmd, err);
        if (lst != Status::ok) {
            return lst;
        }

        // 4. SeatedClimbPolicy + SeatedPostureProgram update (sharing Perception's delayed signals)
        const bool owns = plan_cmd.weight > BIKE_REAL(Real, 0.0);
        const Real eff_preview = has_preview_grade ? preview_grade : road_grade;
        const Real power_target = detail::climb_power_target_w<Real>(
            intent_cfg, eff_preview, !owns, &state->climb);

        if (state->perception.has_delayed != 0) {
            const IntentSignalsCore<Real> delayed = state->perception.delayed.signals;
            state->climb.inclination_rad +=
                delayed.pitch_rate_up_rad_s * intent_cfg->period_s;
            const Real fwd = delayed.specific_force_body_mps2[0];
            const Real lat = delayed.specific_force_body_mps2[1];
            const Real up = delayed.specific_force_body_mps2[2];
            const Real norm = detail::vector_norm3_py<Real, TK>(fwd, lat, up);
            if (BIKE_REAL(Real, 0.8) * BIKE_REAL(Real, 9.81) <= norm &&
                norm <= BIKE_REAL(Real, 1.2) * BIKE_REAL(Real, 9.81)) {
                const Real estimate = Trig::template atan2_val<Real, TK>(fwd, up);
                const Real diff = detail::wrap_angle_with_trig<Real, TK, Trig>(
                    estimate - state->climb.inclination_rad);
                const Real alpha =
                    BIKE_REAL(Real, 1.0) -
                    TK::exp(-intent_cfg->period_s / intent_cfg->orientation_tau_s);
                state->climb.inclination_rad += alpha * diff;
            }
            state->climb.inclination_rad =
                detail::wrap_angle_with_trig<Real, TK, Trig>(state->climb.inclination_rad);
            state->climb.effort_nm = detail::crank_effort_ceiling_core<Real, TK>(
                power_target, intent_cfg->max_crank_torque_nm, delayed.crank_rate_rad_s);
        }

        Real new_lean = BIKE_REAL(Real, 0.0);
        const Status post_st = posture_program_update_with_trig<Real, TK, Trig>(
            intent_cfg, eff_preview, intent_cfg->period_s,
            signals.has_front_load_share != 0, signals.front_load_share,
            has_lean_limit, lean_limit_rad, intent_cfg->dead_time_ticks,
            has_min_lean, min_lean_rad, react_cmd.lean_add_rad,
            owns, plan_cmd.lean_target_rad, plan_cmd.weight,
            &state->posture, new_lean, err);
        if (post_st != Status::ok) {
            return post_st;
        }
        state->climb.lean_rad = new_lean;

        Real ceiling_nm = state->climb.effort_nm;
        if (has_percepts && intent_cfg->climb_effort_cap_count > 0) {
            const Real ccap = detail::compute_climb_cap_nm<Real, TK>(intent_cfg, &percepts);
            if (ccap < ceiling_nm) {
                ceiling_nm = ccap;
            }
        }
        if (owns) {
            ceiling_nm =
                (plan_cmd.weight == BIKE_REAL(Real, 1.0))
                    ? plan_cmd.effort_nm
                    : (plan_cmd.weight * plan_cmd.effort_nm +
                       (BIKE_REAL(Real, 1.0) - plan_cmd.weight) * ceiling_nm);
        }

        state->intent_posture = RiderPostureCore<Real>{};
        state->intent_posture.torso_lean_rad = new_lean;
        state->intent_posture.use_saddle = 1;
        state->intent_effort_ceiling_nm = ceiling_nm;

        state->commands = detail::commands_of_core<Real>(
            plan_cmd, launch_cfg->plan_enabled != 0, launch_cfg,
            has_percepts, &percepts.senses);
        state->last_tick_step = step;
    }

    if (in_control.rider_enabled == 0) {
        return Status::ok;
    }
    if (out_res.control.has_posture == 0) {
        out_res.control.has_posture = 1;
        out_res.control.posture = state->intent_posture;
    }
    if (out_res.control.has_human_torque_nm == 0) {
        out_res.control.has_human_torque_nm = 1;
        out_res.control.human_torque_nm = state->intent_effort_ceiling_nm;
        const Real factor = state->reaction.effort_factor;
        if (factor != BIKE_REAL(Real, 1.0)) {
            out_res.control.human_torque_nm =
                state->intent_effort_ceiling_nm * factor;
            out_res.effort_limited = 1;
        }
        const LaunchCommandCore<Real> cur_plan = launch_command<Real>(launch_cfg, &state->launch);
        if (cur_plan.weight > BIKE_REAL(Real, 0.0)) {
            out_res.effort_limited = 1;
        }
    }
    return Status::ok;
}

template <class Real, class TK, class IntentCfgPtr, class FrontLiftCfgPtr, class LaunchCfgPtr, class StatePtr>
BIKE_HD Status rider_intent_resolve(
    IntentCfgPtr intent_cfg,
    FrontLiftCfgPtr reaction_cfg,
    LaunchCfgPtr launch_cfg,
    IntentControlCore<Real> in_control,
    IntentSignalsCore<Real> signals,
    RiderSensesCore<Real> senses,
    i32 step,
    Real road_grade,
    bool has_preview_grade,
    Real preview_grade,
    bool has_lean_limit,
    Real lean_limit_rad,
    bool active,
    bool has_min_lean,
    Real min_lean_rad,
    LaunchContextCore launch_ctx,
    StatePtr state,
    BIKE_THREAD IntentResolutionCore<Real> &out_res,
    BIKE_THREAD ErrCode &err
) noexcept {
    return rider_intent_resolve_with_trig<Real, TK, PortableTrig>(
        intent_cfg, reaction_cfg, launch_cfg, in_control, signals, senses,
        step, road_grade, has_preview_grade, preview_grade, has_lean_limit,
        lean_limit_rad, active, has_min_lean, min_lean_rad, launch_ctx,
        state, out_res, err);
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
