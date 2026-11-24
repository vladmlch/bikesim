#include "physics_core/assist.hpp"
#include "physics_core/common.hpp"
#include "physics_core/cpu_exact_toolkit.hpp"
#include "physics_core/gpu_math.hpp"
#include "physics_core/gpu_toolkit.hpp"
#include "physics_core/pedaling.hpp"
#include "physics_core/ratchet.hpp"
#include "physics_core/shifting.hpp"

#include "config_types.hpp"
#include "drivetrain/motor.hpp"
#include "drivetrain/pedaling.hpp"
#include "drivetrain/shifting.hpp"
#include "support/fixture_test.hpp"

#if defined(__APPLE__)
#include "test_sp3_drivetrain_metal_helper.hpp"
#endif

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <numbers>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

using bike::physics_core::AssistParams;
using bike::physics_core::AssistState;
using bike::physics_core::CoastReason;
using bike::physics_core::CpuExactToolkit;
using bike::physics_core::ErrCode;
using bike::physics_core::GpuToolkit;
using bike::physics_core::PedalingOutput;
using bike::physics_core::PedalingParams;
using bike::physics_core::PedalingState;
using bike::physics_core::PedalMode;
using bike::physics_core::ProfileMode;
using bike::physics_core::RatchetBoundState;
using bike::physics_core::RepositionState;
using bike::physics_core::RiderShiftInput;
using bike::physics_core::ShiftDirection;
using bike::physics_core::ShiftingParams;
using bike::physics_core::ShiftingState;
using bike::physics_core::ShiftResponse;
using bike::physics_core::SlipMode;
using bike::physics_core::Status;
using bike::physics_core::WorldRotationsRebaseState;
using bike::physics_core::assist_reset;
using bike::physics_core::assist_step;
using bike::physics_core::human_crank_torque;
using bike::physics_core::i32;
using bike::physics_core::kCassetteCogCount;
using bike::physics_core::pedaling_reset;
using bike::physics_core::pedaling_update;
using bike::physics_core::ratchet_phi;
using bike::physics_core::rebase_pedaling_2pi;
using bike::physics_core::rebase_world_rotations_2pi;
using bike::physics_core::shift_torque_factor;
using bike::physics_core::shifting_gear_ratio;
using bike::physics_core::shifting_reset;
using bike::physics_core::shifting_seconds_to_ticks;
using bike::physics_core::shifting_step_with_cassette;
using bike::physics_core::shifting_update;
using bike::physics_core::u8;

void require(bool cond, std::string_view msg) {
    bt::require(cond, msg);
}

void require_near(double actual, double expected, double tol, std::string_view what) {
    if (!std::isfinite(actual) || !std::isfinite(expected) ||
        std::abs(actual - expected) > tol) {
        std::ostringstream oss;
        oss << what << ": actual=" << actual << ", expected=" << expected
            << ", tol=" << tol;
        throw std::runtime_error(oss.str());
    }
}

void require_bits_f32(float actual, float expected, std::string_view what) {
    const auto ua = std::bit_cast<std::uint32_t>(actual);
    const auto ue = std::bit_cast<std::uint32_t>(expected);
    if (ua != ue) {
        std::ostringstream oss;
        oss << what << ": 32-bit mismatch (actual=0x" << std::hex << ua
            << ", expected=0x" << ue << ")";
        throw std::runtime_error(oss.str());
    }
}

ShiftDirection cpu_shift_dir(std::string_view s) {
    if (s == "up") {
        return ShiftDirection::up;
    }
    if (s == "down") {
        return ShiftDirection::down;
    }
    return ShiftDirection::none;
}

ShiftResponse cpu_shift_resp(std::string_view s) {
    if (s == "accepted") {
        return ShiftResponse::accepted;
    }
    if (s == "deferred") {
        return ShiftResponse::deferred;
    }
    if (s == "rejected") {
        return ShiftResponse::rejected;
    }
    return ShiftResponse::none;
}

PedalMode cpu_pedal_mode(drivetrain::PedalMode m) {
    switch (m) {
    case drivetrain::PedalMode::disabled:
        return PedalMode::disabled;
    case drivetrain::PedalMode::pedaling:
        return PedalMode::pedaling;
    case drivetrain::PedalMode::coasting:
        return PedalMode::coasting;
    }
    return PedalMode::disabled;
}

CoastReason cpu_coast_reason(drivetrain::CoastReason r) {
    switch (r) {
    case drivetrain::CoastReason::none:
        return CoastReason::none;
    case drivetrain::CoastReason::braking:
        return CoastReason::braking;
    case drivetrain::CoastReason::no_effort:
        return CoastReason::no_effort;
    case drivetrain::CoastReason::cadence:
        return CoastReason::cadence;
    case drivetrain::CoastReason::disabled:
        return CoastReason::disabled;
    case drivetrain::CoastReason::reposition_requested:
        return CoastReason::reposition_requested;
    case drivetrain::CoastReason::reposition_stall:
        return CoastReason::reposition_stall;
    }
    return CoastReason::none;
}

template <class Real>
ShiftingParams<Real> make_shifting_params(double dt, SlipMode slip_mode) {
    ShiftingParams<Real> p{};
    p.front_teeth = 22;
    p.initial_rear_teeth = 24;
    p.cog_count = kCassetteCogCount;
    p.target_cadence_min_rpm = static_cast<Real>(75.0);
    p.target_cadence_max_rpm = static_cast<Real>(110.0);
    p.shift_cooldown_ticks = shifting_seconds_to_ticks<double, CpuExactToolkit>(
        0.25, dt);
    p.shift_cut_duration_ticks = shifting_seconds_to_ticks<double, CpuExactToolkit>(
        0.20, dt);
    p.torque_factor_larger_cog = static_cast<Real>(0.8);
    p.torque_factor_smaller_cog = static_cast<Real>(0.3);
    p.cadence_smoothing_tau_s = static_cast<Real>(0.05);
    p.upshift_slip_limit_mps = static_cast<Real>(0.5);
    p.upshift_slip_mode = slip_mode;
    p.enabled = 1;
    return p;
}

drivetrain::GearingConfig make_cpu_gearing() {
    return {.front_teeth = 22, .rear_teeth = 24, .chain_pitch_m = 0.0127};
}

drivetrain::ShiftingConfig make_cpu_shifting_cfg(std::string slip_mode_str) {
    return {
        .enabled = true,
        .cassette = {10, 12, 14, 16, 18, 21, 24, 28, 33, 39, 45, 51},
        .target_cadence_min_rpm = 75.0,
        .target_cadence_max_rpm = 110.0,
        .shift_cooldown_s = 0.25,
        .shift_cut_duration_s = 0.20,
        .torque_factor_larger_cog = 0.8,
        .torque_factor_smaller_cog = 0.3,
        .cadence_smoothing_tau_s = 0.05,
        .upshift_slip_limit_mps = 0.5,
        .upshift_slip_mode = std::move(slip_mode_str),
    };
}

template <class Real>
PedalingParams<Real> make_pedaling_params(double dt, bool reposition_on_stall,
                                          double stall_window_s = 1.0) {
    PedalingParams<Real> p{};
    p.coast_above_rpm = static_cast<Real>(120.0);
    p.resume_below_rpm = static_cast<Real>(105.0);
    p.stop_time_s = static_cast<Real>(0.35);
    p.coast_cadence_tau_s = static_cast<Real>(0.35);
    p.mash_cadence_rpm = static_cast<Real>(45.0);
    p.mash_torque_nm = static_cast<Real>(60.0);
    p.effort_slew_nm_s = static_cast<Real>(300.0);
    p.reposition_min_effort_nm = static_cast<Real>(20.0);
    p.reposition_stall_window_ticks = static_cast<i32>(std::llround(stall_window_s / dt));
    p.reposition_stall_progress_rad = static_cast<Real>(0.5);
    p.reposition_cooldown_ticks = static_cast<i32>(std::llround(1.0 / dt));
    p.reposition_back_rate_rad_s = static_cast<Real>(3.0);
    p.reposition_timeout_ticks = static_cast<i32>(std::llround(3.0 / dt));
    p.reposition_phase_tolerance_rad = static_cast<Real>(0.05);
    p.reposition_noop_rad = static_cast<Real>(0.12);
    p.enabled = 1;
    p.reposition_on_stall = reposition_on_stall ? 1 : 0;
    return p;
}

drivetrain::PedalingConfig make_cpu_pedaling_cfg(bool reposition_on_stall,
                                                 double stall_window_s = 1.0) {
    return {
        .enabled = true,
        .coast_above_rpm = 120.0,
        .resume_below_rpm = 105.0,
        .stop_time_s = 0.35,
        .coast_cadence_tau_s = 0.35,
        .mash_cadence_rpm = 45.0,
        .mash_torque_nm = 60.0,
        .effort_slew_nm_s = 300.0,
        .rollback_brake = true,
        .rollback_engage_mps = 0.25,
        .rollback_release_mps = 0.0,
        .rollback_demand = 1.0,
        .reposition_on_stall = reposition_on_stall,
        .reposition_min_effort_nm = 20.0,
        .reposition_stall_window_s = stall_window_s,
        .reposition_stall_progress_rad = 0.5,
        .reposition_cooldown_s = 1.0,
        .reposition_back_rate_rad_s = 3.0,
        .reposition_damping_nms_rad = 12.0,
        .reposition_timeout_s = 3.0,
        .reposition_phase_tolerance_rad = 0.05,
        .reposition_noop_rad = 0.12,
    };
}

template <class Real>
AssistParams<Real> make_assist_params(double dt, ProfileMode mode, bool use_profile,
                                      double release_hold_s = 0.3) {
    AssistParams<Real> p{};
    p.gain = static_cast<Real>(2.0);
    p.max_torque = static_cast<Real>(85.0);
    p.max_power = static_cast<Real>(600.0);
    p.tau = static_cast<Real>(0.04);
    p.slew = static_cast<Real>(400.0);
    p.engage_torque_nm = static_cast<Real>(4.0);
    p.release_hold_ticks = static_cast<i32>(std::llround(release_hold_s / dt));
    p.cutoff_mps = static_cast<Real>(25.0 / 3.6);
    p.taper_width_mps = static_cast<Real>(2.0 / 3.6);
    p.profile = {
        .eco = static_cast<Real>(0.6),
        .tour = static_cast<Real>(1.4),
        .emtb_low = static_cast<Real>(1.4),
        .emtb_high = static_cast<Real>(3.4),
        .turbo = static_cast<Real>(3.4),
        .emtb_full_gain_at_nm = static_cast<Real>(40.0),
    };
    p.curve_rpm[0] = static_cast<Real>(0.0);
    p.curve_nm[0] = static_cast<Real>(85.0);
    p.curve_rpm[1] = static_cast<Real>(120.0);
    p.curve_nm[1] = static_cast<Real>(85.0);
    p.curve_rpm[2] = static_cast<Real>(120.1);
    p.curve_nm[2] = static_cast<Real>(47.7);
    p.curve_rpm[3] = static_cast<Real>(180.0);
    p.curve_nm[3] = static_cast<Real>(0.0);
    p.curve_len = 4;
    p.profile_mode = mode;
    p.has_profile = use_profile ? 1 : 0;
    return p;
}

drivetrain::AssistConfig make_cpu_assist_cfg(std::string mode, bool use_profile,
                                          double release_hold_s = 0.3) {
    drivetrain::AssistConfig cfg{};
    cfg.gain = 2.0;
    cfg.max_torque = 85.0;
    cfg.max_power = 600.0;
    cfg.tau = 0.04;
    cfg.slew = 400.0;
    cfg.engage_torque_nm = 4.0;
    cfg.release_hold_s = release_hold_s;
    cfg.cutoff_mps = 25.0 / 3.6;
    cfg.taper_width_mps = 2.0 / 3.6;
    cfg.torque_curve = std::vector<std::array<double, 2>>{
        {0.0, 85.0}, {120.0, 85.0}, {120.1, 47.7}, {180.0, 0.0}};
    if (use_profile) {
        cfg.profile = drivetrain::MotorProfile{
            .eco = 0.6,
            .tour = 1.4,
            .emtb_low = 1.4,
            .emtb_high = 3.4,
            .turbo = 3.4,
            .emtb_full_gain_at_nm = 40.0,
        };
        cfg.mode = std::move(mode);
    } else {
        cfg.mode = "custom";
    }
    return cfg;
}

void test_pod_layout_and_footprint() {
    static_assert(std::is_trivially_copyable_v<ShiftingParams<float>>);
    static_assert(std::is_trivially_copyable_v<ShiftingState<float>>);
    static_assert(std::is_trivially_copyable_v<RiderShiftInput>);
    static_assert(std::is_trivially_copyable_v<PedalingParams<float>>);
    static_assert(std::is_trivially_copyable_v<RepositionState<float>>);
    static_assert(std::is_trivially_copyable_v<PedalingState<float>>);
    static_assert(std::is_trivially_copyable_v<PedalingOutput<float>>);
    static_assert(std::is_trivially_copyable_v<AssistParams<float>>);
    static_assert(std::is_trivially_copyable_v<AssistState<float>>);

    const std::size_t total_state_bytes =
        sizeof(ShiftingState<float>) + sizeof(PedalingState<float>) +
        sizeof(AssistState<float>);
    require(total_state_bytes <= 16384u,
            "combined SP3-A per-world state must stay well below 16 KiB");
    require(total_state_bytes <= 256u,
            "combined SP3-A per-world state footprint budget <= 256 B");
}

void test_shifting_cpu_parity_and_cassette() {
    constexpr double kDt = 0.00125;
    for (const bool mag_mode : {false, true}) {
        const SlipMode slip_enum =
            mag_mode ? SlipMode::magnitude : SlipMode::legacy_signed;
        const std::string slip_str = mag_mode ? "magnitude" : "legacy_signed";

        drivetrain::CadenceShifter cpu(
            make_cpu_gearing(), make_cpu_shifting_cfg(slip_str));

        const auto p64 = make_shifting_params<double>(kDt, slip_enum);
        ShiftingState<double> s64{};
        shifting_reset<double>(&p64, &s64);

        const auto p32 = make_shifting_params<float>(kDt, slip_enum);
        ShiftingState<float> s32{};
        shifting_reset<float>(&p32, &s32);

        RatchetBoundState<float> rat32{};
        rat32.has_boundary = 1;
        rat32.ratio = shifting_gear_ratio<float>(&p32, &s32);
        rat32.boundary = ratchet_phi<float, GpuToolkit>(
            rat32.ratio, 1.5f, 1.2f);
        std::array<float, 26> tendon_range{};
        tendon_range.fill(1e12f);
        for (std::size_t k = 0; k < 12; ++k) {
            tendon_range[2 * k + 0] = -1e12f;
            tendon_range[2 * k + 1] =
                std::cmp_equal(k, s32.active_cog_idx)
                    ? rat32.boundary
                    : 1e12f;
        }

        double q_shaft = 1.5;
        double q_wheel = 1.2;

        for (int step = 0; step < 2600; ++step) {
            double cadence = 92.0;
            double required = 92.0;
            bool pedaling = true;
            bool braking = false;
            bool contact = true;
            std::optional<double> slip = std::nullopt;
            drivetrain::RiderShift cpu_rider{};
            RiderShiftInput gpu_rider{};

            if (step < 500) {
                // High cadence triggers upshifts (24 -> 21 -> 18 -> 16 -> 14 -> 12 -> 10)
                cadence = 128.0;
                required = 126.0;
            } else if (step >= 500 && step < 700) {
                // Negative slip vs positive slip blocking upshifts
                cadence = 125.0;
                required = 125.0;
                slip = (step < 600) ? std::optional<double>(-0.8)
                                    : std::optional<double>(0.8);
            } else if (step >= 700 && step < 1200) {
                // Low cadence triggers downshifts
                cadence = 52.0;
                required = 52.0;
            } else if (step >= 1200 && step < 1500) {
                // Braking blocks shifts; rider-owned requests defer then accept
                cadence = 85.0;
                required = 85.0;
                if (step == 1210) {
                    braking = true;
                    cpu_rider = {.owns = true, .request = drivetrain::ShiftDirection::up};
                    gpu_rider = {.request = ShiftDirection::up, .owns = 1, .has_request = 1};
                } else if (step == 1220 || step == 1230) {
                    // Accepted at 1220, deferred during cooldown at 1230
                    cpu_rider = {.owns = true, .request = drivetrain::ShiftDirection::up};
                    gpu_rider = {.request = ShiftDirection::up, .owns = 1, .has_request = 1};
                } else if (step == 1450) {
                    // Stationary target
                    cpu_rider = {.owns = true, .stationary_target_teeth = 39};
                    gpu_rider = {.stationary_target_teeth = 39, .owns = 1, .has_stationary_target = 1};
                } else if (step >= 1460 && step < 1470) {
                    // Owns with no request suppresses band
                    cadence = 135.0;
                    required = 135.0;
                    cpu_rider = {.owns = true};
                    gpu_rider = {.owns = 1};
                }
            } else if (step >= 1500 && step < 1800) {
                // No contact / not pedaling
                cadence = 130.0;
                required = 130.0;
                pedaling = (step % 2 == 0);
                contact = (step % 3 != 0);
            } else {
                // Oscillating cadence
                cadence = 90.0 + 40.0 * std::sin(0.01 * step);
                required = 90.0 + 38.0 * std::sin(0.01 * step);
            }

            q_shaft += (cadence * 2.0 * std::numbers::pi / 60.0) * kDt;
            q_wheel += (required * cpu.gear_ratio() * 2.0 *
                        std::numbers::pi / 60.0) * kDt;

            const bool cpu_shifted = cpu.update(
                cadence, required, kDt, pedaling, braking, contact, slip, cpu_rider);

            bool shifted64 = false;
            ErrCode err64 = ErrCode::none;
            const Status st64 = shifting_update<double, CpuExactToolkit>(
                &p64, &s64, cadence, required, kDt, pedaling, braking, contact,
                slip.has_value(), slip.value_or(0.0), gpu_rider, shifted64, err64);
            require(st64 == Status::ok && err64 == ErrCode::none,
                    "shifting_update<double> ok");
            require(shifted64 == cpu_shifted, "shifting shifted64 parity");
            require(s64.rear_teeth == cpu.state().rear_teeth, "rear_teeth64 parity");
            require(s64.from_teeth == cpu.state().from_teeth, "from_teeth64 parity");
            require(s64.shift_count == cpu.state().shift_count, "shift_count64 parity");
            require(s64.direction == cpu_shift_dir(cpu.state().direction),
                    "direction64 parity");
            require(s64.rider_response == cpu_shift_resp(cpu.state().rider_response),
                    "rider_response64 parity");
            require_near(shift_torque_factor<double>(&p64, &s64),
                         cpu.torque_factor(), 1e-12, "torque_factor64 parity");
            require_near(shifting_gear_ratio<double>(&p64, &s64),
                         cpu.gear_ratio(), 1e-15, "gear_ratio64 parity");

            const i32 prev_teeth = s32.rear_teeth;
            const float prev_bnd = rat32.boundary;
            bool shifted32 = false;
            ErrCode err32 = ErrCode::none;
            const Status st32 = shifting_step_with_cassette<float, GpuToolkit>(
                &p32, &s32, &rat32,
                static_cast<float>(cadence), static_cast<float>(required),
                static_cast<float>(kDt), pedaling, braking, contact,
                slip.has_value(), static_cast<float>(slip.value_or(0.0)),
                gpu_rider, static_cast<float>(q_shaft), static_cast<float>(q_wheel),
                13, tendon_range.data(), shifted32, err32);
            require(st32 == Status::ok && err32 == ErrCode::none,
                    "shifting_step_with_cassette<float> ok");
            require(shifted32 == cpu_shifted, "shifting shifted32 parity");
            require(s32.rear_teeth == cpu.state().rear_teeth, "rear_teeth32 parity");
            require(s32.shift_count == cpu.state().shift_count, "shift_count32 parity");
            require(s32.rider_response == cpu_shift_resp(cpu.state().rider_response),
                    "rider_response32 parity");

            if (shifted32) {
                const float old_r = 22.0f / static_cast<float>(prev_teeth);
                const float new_r = 22.0f / static_cast<float>(s32.rear_teeth);
                const float q_shaft_f = static_cast<float>(q_shaft);
                const float q_wheel_f = static_cast<float>(q_wheel);
                const double exact_bnd = static_cast<double>(prev_bnd) +
                    (static_cast<double>(new_r) * static_cast<double>(q_shaft_f) -
                     static_cast<double>(q_wheel_f)) -
                    (static_cast<double>(old_r) * static_cast<double>(q_shaft_f) -
                     static_cast<double>(q_wheel_f));
                const float expected_bnd = static_cast<float>(exact_bnd);
                const auto bnd_ulps = std::abs(
                    static_cast<std::int64_t>(std::bit_cast<std::uint32_t>(rat32.boundary)) -
                    static_cast<std::int64_t>(std::bit_cast<std::uint32_t>(expected_bnd)));
                require(bnd_ulps <= 1, "cassette shift ratchet boundary <= 1 ULP vs FP64");
                for (i32 k = 0; k < 12; ++k) {
                    const std::size_t idx = static_cast<std::size_t>(k);
                    require_bits_f32(tendon_range[2u * idx + 0u], -1e12f,
                                     "cassette lower bound");
                    if (k == s32.active_cog_idx) {
                        require_bits_f32(tendon_range[2u * idx + 1u], rat32.boundary,
                                         "active cog upper bound");
                    } else {
                        require_bits_f32(tendon_range[2u * idx + 1u], 1e12f,
                                         "inactive cog upper bound");
                    }
                }
            }
        }
    }
}

void test_pedaling_and_stall_cpu_parity() {
    constexpr double kDt = 0.00125;
    // Use 0.9999 s (= 800 ticks at 0.00125 s) for step-by-step CPU parity so CPU's
    // FP64 += 0.00125 sum (0.9999999999999842 < 1.0) does not drift +1 tick relative
    // to exact 800-tick schedule; test_3600s_int32_tick_drift tests 1.0 s = 800 ticks.
    constexpr double kStallWindowS = 0.9999;
    for (const bool repos_stall : {false, true}) {
        drivetrain::PedalingPolicy cpu(make_cpu_pedaling_cfg(repos_stall, kStallWindowS));

        auto p64 = make_pedaling_params<double>(kDt, repos_stall, kStallWindowS);
        PedalingState<double> s64{};
        pedaling_reset<double>(&s64);

        auto p32 = make_pedaling_params<float>(kDt, repos_stall, kStallWindowS);
        PedalingState<float> s32{};
        pedaling_reset<float>(&s32);

        double phase = -0.8;
        for (int step = 0; step < 2600; ++step) {
            bool enabled = true;
            bool braking = false;
            bool repos_req = false;
            double rate = 8.5;
            double required = 80.0;
            double effort = 35.0;
            std::optional<double> shaft = std::nullopt;
            const std::optional<double> ceiling = 50.0;
            double tf = 1.0;
            const bool cut_after = (step % 2 == 0);

            if (step < 10) {
                enabled = false;
                repos_req = (step >= 5); // held across enable edge
            } else if (step < 40) {
                repos_req = (step < 15); // should remain disarmed until falling+rising edge
            } else if (step >= 100 && step < 260) {
                // Trigger explicit reposition
                repos_req = (step == 100);
                rate = -2.0;
            } else if (step >= 300 && step < 1200) {
                // Stall rock about dead center
                rate = 0.2 * std::sin(0.25 * step);
                required = 0.1;
                effort = 30.0;
            } else if (step >= 1200 && step < 1700) {
                // High cadence coasting hysteresis + decel ramp
                rate = 14.0;
                required = 132.0;
                shaft = 14.5;
            } else if (step >= 1700 && step < 1850) {
                // Braking coast
                braking = true;
                rate = 5.0;
            } else if (step >= 1850 && step < 2000) {
                // Zero effort coast
                effort = 0.0;
                rate = 4.0;
            } else {
                // Normal mash + shift cut
                rate = 3.5;
                required = 32.0;
                effort = 25.0;
                tf = (step % 100 < 30) ? 0.3 : 1.0;
            }

            phase += rate * kDt;

            p64.enabled = enabled ? 1 : 0;
            p32.enabled = enabled ? 1 : 0;

            const auto cpu_out = cpu.update(
                phase, rate, required, effort, kDt, enabled, braking, repos_req,
                shaft, ceiling, tf, cut_after);

            PedalingOutput<double> o64{};
            ErrCode err64 = ErrCode::none;
            const Status st64 = pedaling_update<double, CpuExactToolkit>(
                &p64, &s64, phase, rate, required, effort, kDt, braking, repos_req,
                shaft.has_value(), shaft.value_or(0.0), ceiling.has_value(),
                ceiling.value_or(0.0), tf, cut_after, o64, err64);
            require(st64 == Status::ok && err64 == ErrCode::none,
                    "pedaling_update<double> ok");
            require(o64.mode == cpu_pedal_mode(cpu_out.mode),
                    "pedal mode64 parity at step=" + std::to_string(step) +
                    " repos_stall=" + std::to_string(repos_stall ? 1 : 0) +
                    " o64.mode=" + std::to_string(static_cast<int>(o64.mode)) +
                    " cpu.mode=" + std::to_string(static_cast<int>(cpu_out.mode)) +
                    " o64.reason=" + std::to_string(static_cast<int>(o64.reason)) +
                    " cpu.reason=" + std::to_string(static_cast<int>(cpu_out.reason)) +
                    " phase=" + std::to_string(phase) +
                    " s64.stall=" + std::to_string(s64.reposition.stall_elapsed_ticks) +
                    " s64.cd=" + std::to_string(s64.reposition.cooldown_ticks) +
                    " cpu.stall=" + std::to_string(cpu.state().reposition && cpu.state().reposition->stall_elapsed_s ? *cpu.state().reposition->stall_elapsed_s : -1.0) +
                    " cpu.cd=" + std::to_string(cpu.state().reposition ? cpu.state().reposition->cooldown_s : -1.0));
            require(o64.reason == cpu_coast_reason(cpu_out.reason),
                    "pedal reason64 parity");
            require_near(o64.effort_nm, cpu_out.effort_nm, 1e-12,
                         "pedal effort64 parity");
            require_near(o64.target_rate_rad_s, cpu_out.target_rate_rad_s, 1e-12,
                         "pedal target_rate64 parity");
            require((o64.has_target_phase != 0) == cpu_out.target_phase_rad.has_value(),
                    "pedal has_target_phase64 parity");
            if (o64.has_target_phase != 0) {
                require_near(o64.target_phase_rad, *cpu_out.target_phase_rad, 1e-12,
                             "pedal target_phase64 parity");
            }

            const double cpu_tq = drivetrain::human_crank_torque(
                cpu_out.effort_nm, phase, 0.35);
            double tq64 = 0.0;
            require(human_crank_torque<double, CpuExactToolkit>(
                        o64.effort_nm, phase, 0.35, tq64, err64) == Status::ok,
                    "human_crank_torque<double> ok");
            require_near(tq64, cpu_tq, 1e-12, "human_crank_torque64 parity");

            PedalingOutput<float> o32{};
            ErrCode err32 = ErrCode::none;
            const Status st32 = pedaling_update<float, GpuToolkit>(
                &p32, &s32, static_cast<float>(phase), static_cast<float>(rate),
                static_cast<float>(required), static_cast<float>(effort),
                static_cast<float>(kDt), braking, repos_req,
                shaft.has_value(), static_cast<float>(shaft.value_or(0.0)),
                ceiling.has_value(), static_cast<float>(ceiling.value_or(0.0)),
                static_cast<float>(tf), cut_after, o32, err32);
            require(st32 == Status::ok && err32 == ErrCode::none,
                    "pedaling_update<float> ok");
            require(o32.mode == cpu_pedal_mode(cpu_out.mode), "pedal mode32 parity");
            require(o32.reason == cpu_coast_reason(cpu_out.reason),
                    "pedal reason32 parity");
            require_near(static_cast<double>(o32.effort_nm),
                         cpu_out.effort_nm, 5e-4,
                         "pedal effort32 parity");
            require_near(static_cast<double>(o32.target_rate_rad_s),
                         cpu_out.target_rate_rad_s, 1e-3,
                         "pedal target_rate32 parity");
        }
    }
}

void test_assist_cpu_parity() {
    constexpr double kDt = 0.00125;
    // Use 0.2999 s (= 240 ticks at 0.00125 s) for step-by-step CPU parity so CPU's
    // FP64 subtraction (0.3 - 240*0.00125 = 8.496e-16 > 0) does not drift +1 tick
    // relative to exact 240-tick schedule; test_3600s_int32_tick_drift tests 0.3 s = 240 ticks.
    constexpr double kReleaseHoldS = 0.2999;
    const std::array<std::pair<ProfileMode, std::string>, 4> modes{{
        {ProfileMode::eco, "eco"},
        {ProfileMode::tour, "tour"},
        {ProfileMode::emtb, "emtb"},
        {ProfileMode::turbo, "turbo"},
    }};

    for (const auto &[mode_enum, mode_str] : modes) {
        for (const bool use_profile : {true, false}) {
            if (!use_profile && mode_enum != ProfileMode::eco) {
                continue;
            }
            drivetrain::AssistController cpu(
                make_cpu_assist_cfg(mode_str, use_profile, kReleaseHoldS));

            const auto p64 = make_assist_params<double>(
                kDt, mode_enum, use_profile, kReleaseHoldS);
            AssistState<double> s64{};
            assist_reset<double>(&s64);

            const auto p32 = make_assist_params<float>(
                kDt, mode_enum, use_profile, kReleaseHoldS);
            AssistState<float> s32{};
            assist_reset<float>(&s32);

            for (int step = 0; step < 2500; ++step) {
                double human = 25.0;
                double cadence = 85.0;
                double speed = 5.0;
                bool braking = false;
                std::optional<double> req = std::nullopt;
                std::optional<double> shaft = 90.0;

                if (step >= 300 && step < 700) {
                    // Drop effort to trigger P1 release hold (240 ticks)
                    human = 2.0;
                } else if (step >= 700 && step < 900) {
                    // High rpm + speed taper
                    human = 45.0;
                    cadence = 135.0;
                    shaft = 135.0;
                    speed = 6.7;
                } else if (step >= 900 && step < 950) {
                    braking = true;
                } else if (step >= 950 && step < 1200) {
                    human = 30.0;
                    req = 40.0;
                    speed = 4.0;
                } else if (step >= 1200) {
                    // Zero-input long decay to exercise subnormal handling
                    human = 0.0;
                }

                const double cpu_tq = cpu.step(
                    human, cadence, speed, braking, kDt, req, shaft);

                double tq64 = 0.0;
                ErrCode err64 = ErrCode::none;
                const Status st64 = assist_step<double, CpuExactToolkit>(
                    &p64, &s64, human, cadence, speed, braking, kDt,
                    req.has_value(), req.value_or(0.0), shaft.has_value(),
                    shaft.value_or(0.0), tq64, err64);
                require(st64 == Status::ok && err64 == ErrCode::none,
                        "assist_step<double> ok");
                require_near(tq64, cpu_tq, 1e-10, "assist torque64 parity");
                require(s64.pedaling == (cpu.state().pedaling ? 1 : 0),
                        "assist pedaling64 parity");

                float tq32 = 0.0f;
                ErrCode err32 = ErrCode::none;
                const Status st32 = assist_step<float, GpuToolkit>(
                    &p32, &s32, static_cast<float>(human),
                    static_cast<float>(cadence), static_cast<float>(speed),
                    braking, static_cast<float>(kDt), req.has_value(),
                    static_cast<float>(req.value_or(0.0)), shaft.has_value(),
                    static_cast<float>(shaft.value_or(0.0)), tq32, err32);
                require(st32 == Status::ok && err32 == ErrCode::none,
                        "assist_step<float> ok");
                require_near(static_cast<double>(tq32), cpu_tq, 5e-3,
                             "assist torque32 parity");
                require(s32.pedaling == (cpu.state().pedaling ? 1 : 0),
                        "assist pedaling32 parity");
            }
        }
    }
}

void test_3600s_int32_tick_drift() {
    // 3,600 s at dt = 0.00125 s = 2,880,000 steps: verify 0 false +1 tick drifts.
    constexpr i32 kTotalSteps = 2880000;
    constexpr double kDtD = 0.00125;
    constexpr float kDt = 0.00125f;

    // 1. CadenceShifter cooldown (200 ticks = 0.25 s) & cut (160 ticks = 0.20 s)
    const auto sp = make_shifting_params<float>(kDtD, SlipMode::legacy_signed);
    ShiftingState<float> ss{};
    shifting_reset<float>(&sp, &ss);
    i32 shift_cooldown_countdown = 0;
    i32 shift_cut_countdown = 0;

    // 2. AssistController release_hold (240 ticks = 0.30 s)
    const auto ap = make_assist_params<float>(kDtD, ProfileMode::turbo, true);
    AssistState<float> as{};
    assist_reset<float>(&as);
    i32 expected_hold_left = 0;
    bool hold_spent = false;

    // 3. PedalingPolicy StallDetector window (800 ticks = 1.0 s) & timeout (2400 ticks = 3.0 s)
    const auto pp = make_pedaling_params<float>(kDtD, true);
    PedalingState<float> ps{};
    pedaling_reset<float>(&ps);
    i32 expected_stall_ticks = -1;

    ErrCode err = ErrCode::none;
    for (i32 step = 0; step < kTotalSteps; ++step) {
        // Trigger shifts every 500 ticks cleanly across the whole hour
        const i32 mod_shift = step % 500;
        const float cad = (mod_shift < 250) ? 130.0f : 55.0f;
        const float req_cad = cad;
        bool shifted = false;
        require(shifting_update<float, GpuToolkit>(
                    &sp, &ss, cad, req_cad, kDt, true, false, true, false, 0.0f,
                    RiderShiftInput{}, shifted, err) == Status::ok,
                "3600s shifting step");
        if (shift_cooldown_countdown > 0) {
            --shift_cooldown_countdown;
        }
        if (shift_cut_countdown > 0) {
            --shift_cut_countdown;
        }
        if (shifted) {
            require(shift_cooldown_countdown == 0,
                    "0 false +1 tick drift on shifter cooldown");
            shift_cooldown_countdown = 200;
            shift_cut_countdown = 160;
        }
        require(ss.cooldown_ticks == shift_cooldown_countdown,
                "exact int32 shifter cooldown_ticks");
        require(ss.cut_remaining_ticks == shift_cut_countdown,
                "exact int32 shifter cut_remaining_ticks");

        // Trigger assist release hold every 1000 ticks
        const i32 mod_assist = step % 1000;
        const float human = (mod_assist < 400) ? 30.0f : 0.0f;
        float tq = 0.0f;
        require(assist_step<float, GpuToolkit>(
                    &ap, &as, human, 85.0f, 4.0f, false, kDt, false, 0.0f,
                    true, 85.0f, tq, err) == Status::ok,
                "3600s assist step");
        if (mod_assist < 400) {
            expected_hold_left = 0;
            hold_spent = false;
        } else if (mod_assist == 400 && !hold_spent) {
            expected_hold_left = 240 - 1; // armed to 240 and consumed 1 tick immediately
            hold_spent = true;
        } else if (expected_hold_left > 0) {
            --expected_hold_left;
        }
        require(as.release_hold_ticks == expected_hold_left,
                "0 false +1 tick drift on assist release_hold_ticks");

        // Trigger StallDetector every 4000 ticks (loaded for 805 ticks, then cleared)
        const i32 mod_stall = step % 4000;
        const bool stall_loaded = (mod_stall >= 100 && mod_stall < 910);
        const float eff_p = stall_loaded ? 30.0f : 10.0f;
        const float rate_p = stall_loaded ? 0.0f : 8.5f;
        PedalingOutput<float> po{};
        require(pedaling_update<float, GpuToolkit>(
                    &pp, &ps, 0.8f, rate_p, 0.0f, eff_p, kDt, false, false,
                    false, 0.0f, false, 0.0f, 1.0f, false, po, err) == Status::ok,
                "3600s pedaling step");
        if (mod_stall == 100) {
            expected_stall_ticks = 0;
        } else if (stall_loaded && ps.reposition.has_goal == 0 &&
                   ps.reposition.cooldown_ticks <= 0 && expected_stall_ticks >= 0) {
            ++expected_stall_ticks;
            if (expected_stall_ticks == 800) {
                require(po.reason == CoastReason::reposition_stall,
                        "0 false +1 tick drift on stall detector trigger");
                expected_stall_ticks = -1;
            }
        } else {
            expected_stall_ticks = -1;
        }
        // Also verify reposition timeout counter on one cycle per 40000 steps
        if (mod_stall == 910) {
            require(ps.reposition.has_goal == 1 ||
                        ps.reposition.cooldown_ticks >= 0,
                    "reposition active after stall");
        }
    }
}

void test_2pi_epoch_rebase_invariance() {
    constexpr double kDt = 0.00125;
    constexpr double kTwoPi = 2.0 * std::numbers::pi;
    constexpr double kThreshold = 256.0;

    // Reference trajectory (FP64, never rebased)
    const auto pp64 = make_pedaling_params<double>(kDt, true);
    PedalingState<double> ps_ref{};
    pedaling_reset<double>(&ps_ref);
    WorldRotationsRebaseState<double> w_ref{};
    w_ref.freehub_ratio = 22.0 / 24.0;
    w_ref.has_freehub_boundary = 1;
    w_ref.has_clutch_boundary = 1;
    w_ref.has_brake_latch[0] = 1;
    w_ref.has_brake_latch[1] = 1;

    // Rebased trajectories (FP64 and FP32, rebased whenever |q| > 256 rad)
    PedalingState<double> ps_reb64{};
    pedaling_reset<double>(&ps_reb64);
    WorldRotationsRebaseState<double> w_reb64 = w_ref;

    const auto pp32 = make_pedaling_params<float>(kDt, true);
    PedalingState<float> ps_reb32{};
    pedaling_reset<float>(&ps_reb32);
    WorldRotationsRebaseState<float> w_reb32{};
    w_reb32.freehub_ratio = static_cast<float>(w_ref.freehub_ratio);
    w_reb32.has_freehub_boundary = 1;
    w_reb32.has_clutch_boundary = 1;
    w_reb32.has_brake_latch[0] = 1;
    w_reb32.has_brake_latch[1] = 1;

    ErrCode err = ErrCode::none;
    int rebase_count = 0;

    // 6,000 steps (~7.5 s, crank reaches > 600 rad, multiple 2*pi rebases including
    // during coasting, active reposition, and active StallDetector window)
    for (int step = 0; step < 6000; ++step) {
        const double rate = (step >= 3500 && step < 4400) ? 0.05 : 140.0;
        const double effort = (step >= 3000 && step < 3500) ? 0.0 : 30.0;
        const bool repos_req = (step == 3300);

        const double dq = rate * kDt;
        w_ref.crank_qpos += dq;
        w_ref.drive_shaft_qpos += dq;
        w_ref.rear_wheel_qpos += dq * w_ref.freehub_ratio;
        w_ref.front_wheel_qpos += dq * w_ref.freehub_ratio;

        w_reb64.crank_qpos += dq;
        w_reb64.drive_shaft_qpos += dq;
        w_reb64.rear_wheel_qpos += dq * w_reb64.freehub_ratio;
        w_reb64.front_wheel_qpos += dq * w_reb64.freehub_ratio;

        if (std::abs(w_reb64.crank_qpos) > kThreshold ||
            std::abs(w_reb64.rear_wheel_qpos) > kThreshold) {
            const double pre_crank64 = w_reb64.crank_qpos;
            const double pre_wheel64 = w_reb64.rear_wheel_qpos;
            require(rebase_world_rotations_2pi<double, CpuExactToolkit>(
                        &w_reb64, err) == Status::ok,
                    "rebase_world_rotations_2pi<double>");
            const double dq_c64 = pre_crank64 - w_reb64.crank_qpos;
            const double dq_w64 = pre_wheel64 - w_reb64.rear_wheel_qpos;
            rebase_pedaling_2pi<double>(ps_reb64, dq_c64, dq_w64);

            w_reb32.crank_qpos = static_cast<float>(pre_crank64);
            w_reb32.drive_shaft_qpos = static_cast<float>(pre_crank64);
            w_reb32.rear_wheel_qpos = static_cast<float>(pre_wheel64);
            w_reb32.front_wheel_qpos = static_cast<float>(pre_wheel64);
            const float pre_crank32 = w_reb32.crank_qpos;
            const float pre_wheel32 = w_reb32.rear_wheel_qpos;
            require(rebase_world_rotations_2pi<float, GpuToolkit>(
                        &w_reb32, err) == Status::ok,
                    "rebase_world_rotations_2pi<float>");
            const float dq_c32 = pre_crank32 - w_reb32.crank_qpos;
            const float dq_w32 = pre_wheel32 - w_reb32.rear_wheel_qpos;
            rebase_pedaling_2pi<float>(ps_reb32, dq_c32, dq_w32);
            ++rebase_count;

            require(std::abs(w_reb32.crank_qpos) <= static_cast<float>(kTwoPi),
                    "rebased crank_qpos bounded");
            require(std::abs(w_reb32.rear_wheel_qpos) <= static_cast<float>(kTwoPi),
                    "rebased rear_wheel_qpos bounded");
        }

        const double req_rpm = (rate * w_ref.freehub_ratio) * 60.0 / kTwoPi;
        PedalingOutput<double> o_ref{};
        PedalingOutput<double> o_reb64{};
        PedalingOutput<float> o_reb32{};
        require(pedaling_update<double, CpuExactToolkit>(
                    &pp64, &ps_ref, w_ref.crank_qpos, rate, req_rpm, effort, kDt,
                    false, repos_req, false, 0.0, false, 0.0, 1.0, false, o_ref,
                    err) == Status::ok,
                "ref pedaling_update");
        require(pedaling_update<double, CpuExactToolkit>(
                    &pp64, &ps_reb64, w_reb64.crank_qpos, rate, req_rpm, effort,
                    kDt, false, repos_req, false, 0.0, false, 0.0, 1.0, false,
                    o_reb64, err) == Status::ok,
                "reb64 pedaling_update");
        const float crank32_in = static_cast<float>(w_reb64.crank_qpos);
        require(pedaling_update<float, GpuToolkit>(
                    &pp32, &ps_reb32, crank32_in,
                    static_cast<float>(rate), static_cast<float>(req_rpm),
                    static_cast<float>(effort), static_cast<float>(kDt), false,
                    repos_req, false, 0.0f, false, 0.0f, 1.0f, false, o_reb32,
                    err) == Status::ok,
                "reb32 pedaling_update");

        require(o_reb64.mode == o_ref.mode && o_reb64.reason == o_ref.reason,
                "rebase64 mode/reason invariance");
        require(o_reb32.mode == o_ref.mode && o_reb32.reason == o_ref.reason,
                "rebase32 mode/reason invariance");
        require_near(o_reb64.effort_nm, o_ref.effort_nm, 1e-12,
                     "rebase64 effort invariance");
        require_near(o_reb64.target_rate_rad_s, o_ref.target_rate_rad_s, 1e-12,
                     "rebase64 rate invariance");
        if (o_ref.has_target_phase != 0) {
            require(o_reb64.has_target_phase != 0 && o_reb32.has_target_phase != 0,
                    "rebase has_target_phase invariance");
            require_near(o_reb64.target_phase_rad - w_reb64.crank_qpos,
                         o_ref.target_phase_rad - w_ref.crank_qpos, 1e-9,
                         "rebase64 relative target_phase invariance");
            require_near(static_cast<double>(o_reb32.target_phase_rad - crank32_in),
                         o_ref.target_phase_rad - w_ref.crank_qpos, 5e-4,
                         "rebase32 relative target_phase invariance");
        }
        const double phi_err_64 =
            (w_reb64.freehub_boundary -
             (w_reb64.freehub_ratio * w_reb64.drive_shaft_qpos -
              w_reb64.rear_wheel_qpos)) -
            (w_ref.freehub_boundary -
             (w_ref.freehub_ratio * w_ref.drive_shaft_qpos -
              w_ref.rear_wheel_qpos));
        require_near(phi_err_64, 0.0, 1e-9, "rebase64 freehub phi invariance");
    }
    require(rebase_count >= 2, "2pi rebase actually fired multiple times");
}

#if defined(__APPLE__)
void test_apple_metal_sp3_parity() {
    using namespace bike::physics_core::test_sp3_metal;

    constexpr double kDtD = 0.00125;
    constexpr float kDt = 0.00125f;
    MetalSp3Request req{};
    req.physics_core_dir = BIKE_PHYSICS_CORE_SRC_DIR;
    req.shift_params = make_shifting_params<float>(kDtD, SlipMode::magnitude);
    req.shift_init_ratchet.has_boundary = 1;
    req.shift_init_ratchet.ratio = 22.0f / 24.0f;
    req.shift_init_ratchet.boundary = ratchet_phi<float, GpuToolkit>(
        req.shift_init_ratchet.ratio, 1.5f, 1.2f);

    float q_shaft = 1.5f;
    float q_wheel = 1.2f;
    for (int step = 0; step < 800; ++step) {
        ShiftingStepInput in{};
        in.cadence_rpm = (step < 300) ? 128.0f : (step < 600 ? 55.0f : 92.0f);
        in.required_cadence_rpm = in.cadence_rpm;
        in.dt = kDt;
        in.pedaling = 1;
        in.braking = (step >= 650 && step < 670) ? 1 : 0;
        in.rear_in_contact = 1;
        if (step >= 100 && step < 120) {
            in.has_rear_slip = 1;
            in.rear_slip_mps = -0.8f;
        }
        if (step == 660) {
            in.rider = {.request = ShiftDirection::up, .owns = 1, .has_request = 1};
        } else if (step == 700) {
            in.rider = {.stationary_target_teeth = 39, .owns = 1, .has_stationary_target = 1};
        }
        q_shaft += (in.cadence_rpm * 6.2831853f / 60.0f) * kDt;
        q_wheel += (in.required_cadence_rpm * (22.0f / 24.0f) * 6.2831853f / 60.0f) * kDt;
        in.q_driver = q_shaft;
        in.q_driven = q_wheel;
        req.shift_steps.push_back(in);
    }

    req.pedal_params = make_pedaling_params<float>(kDtD, true);
    float phase = -0.8f;
    for (int step = 0; step < 1200; ++step) {
        PedalingStepInput in{};
        in.dt = kDt;
        in.rate_rad_s = (step < 300) ? 8.5f : (step < 950 ? 0.02f : 14.0f);
        in.required_cadence_rpm = (step < 950) ? 0.0f : 132.0f;
        in.effort_nm = (step >= 1100) ? 0.0f : 35.0f;
        in.braking = (step >= 1050 && step < 1070) ? 1 : 0;
        in.reposition_request = (step == 100) ? 1 : 0;
        in.has_ceiling_nm = 1;
        in.ceiling_nm = 50.0f;
        in.torque_factor = (step % 50 < 15) ? 0.3f : 1.0f;
        in.cut_after_ceiling = (step % 2 == 0) ? 1 : 0;
        in.ripple = 0.35f;
        phase += in.rate_rad_s * kDt;
        if (step == 500 || step == 1000) {
            in.do_rebase = 1;
            in.dq_crank_rebase = 6.2831853f;
            in.dq_wheel_rebase = 6.2831853f;
            phase -= in.dq_crank_rebase;
        }
        in.phase_rad = phase;
        req.pedal_steps.push_back(in);
    }

    req.assist_params = make_assist_params<float>(kDtD, ProfileMode::emtb, true);
    for (int step = 0; step < 800; ++step) {
        AssistStepInput in{};
        in.dt = kDt;
        in.human_nm = (step < 150) ? 28.0f : (step < 450 ? 2.0f : 0.0f);
        in.cadence_rpm = 88.0f;
        in.speed_mps = (step < 250) ? 4.5f : 6.6f;
        in.braking = (step == 780) ? 1 : 0;
        in.has_shaft_rpm = 1;
        in.shaft_rpm = 92.0f;
        req.assist_steps.push_back(in);
    }

    const MetalSp3Output metal = run_metal_sp3_parity(req);
    require(metal.device_available, "Metal device must be available on Apple Silicon");
    require(metal.error_message.empty(),
            "Metal SP3 parity failed: " + metal.error_message);
    require(metal.torture_3_0_passed && metal.torture_3_1_passed,
            "MSL 3.0 & 3.1 address-space torture checks must pass");

    // Host FP32 reference vs Metal 0-bit parity
    ShiftingState<float> host_shift{};
    shifting_reset<float>(&req.shift_params, &host_shift);
    RatchetBoundState<float> host_rat = req.shift_init_ratchet;
    std::array<float, 26> host_tendon{};
    for (std::size_t i = 0; i < 12; ++i) {
        host_tendon[2u * i + 0u] = -1e12f;
        host_tendon[2u * i + 1u] =
            (std::cmp_equal(i, host_shift.active_cog_idx) &&
             host_rat.has_boundary != 0)
                ? host_rat.boundary
                : 1e12f;
    }
    host_tendon[24] = -1e12f;
    host_tendon[25] = 0.0f;

    require(metal.shift_records.size() == req.shift_steps.size(),
            "metal shift record count");
    for (std::size_t i = 0; i < req.shift_steps.size(); ++i) {
        const auto &in = req.shift_steps[i];
        bool shifted = false;
        ErrCode err = ErrCode::none;
        const Status st = shifting_step_with_cassette<float, GpuToolkit>(
            &req.shift_params, &host_shift, &host_rat,
            in.cadence_rpm, in.required_cadence_rpm, in.dt,
            in.pedaling != 0, in.braking != 0, in.rear_in_contact != 0,
            in.has_rear_slip != 0, in.rear_slip_mps, in.rider,
            in.q_driver, in.q_driven, 13, host_tendon.data(), shifted, err);
        const auto &m = metal.shift_records[i];
        require(m.status == static_cast<u8>(st), "metal shift status");
        require(m.shifted == (shifted ? 1 : 0), "metal shift shifted");
        require(m.state.rear_teeth == host_shift.rear_teeth, "metal rear_teeth");
        require(m.state.cooldown_ticks == host_shift.cooldown_ticks,
                "metal cooldown_ticks");
        require(m.state.cut_remaining_ticks == host_shift.cut_remaining_ticks,
                "metal cut_remaining_ticks");
        require_bits_f32(m.state.cadence_ema, host_shift.cadence_ema,
                         "metal cadence_ema");
        require_bits_f32(m.state.required_ema, host_shift.required_ema,
                         "metal required_ema");
        require_bits_f32(m.ratchet.boundary, host_rat.boundary,
                         "metal ratchet boundary");
        require_bits_f32(m.torque_factor,
                         shift_torque_factor<float>(&req.shift_params, &host_shift),
                         "metal shift_torque_factor");
        for (std::size_t k = 0; k < 26; ++k) {
            require_bits_f32(m.tendon_ranges[k], host_tendon[k],
                             "metal cassette tendon range");
        }
    }

    PedalingState<float> host_pedal{};
    pedaling_reset<float>(&host_pedal);
    require(metal.pedal_records.size() == req.pedal_steps.size(),
            "metal pedal record count");
    for (std::size_t i = 0; i < req.pedal_steps.size(); ++i) {
        const auto &in = req.pedal_steps[i];
        if (in.do_rebase != 0) {
            rebase_pedaling_2pi<float>(host_pedal, in.dq_crank_rebase,
                                       in.dq_wheel_rebase);
        }
        PedalingOutput<float> po{};
        ErrCode err = ErrCode::none;
        const Status st = pedaling_update<float, GpuToolkit>(
            &req.pedal_params, &host_pedal,
            in.phase_rad, in.rate_rad_s, in.required_cadence_rpm,
            in.effort_nm, in.dt, in.braking != 0, in.reposition_request != 0,
            in.has_shaft_rad_s != 0, in.shaft_rad_s,
            in.has_ceiling_nm != 0, in.ceiling_nm,
            in.torque_factor, in.cut_after_ceiling != 0, po, err);
        float h_tq = 0.0f;
        ErrCode t_err = ErrCode::none;
        const Status t_st = human_crank_torque<float, GpuToolkit>(
            po.effort_nm, in.phase_rad, in.ripple, h_tq, t_err);

        const auto &m = metal.pedal_records[i];
        require(m.status == static_cast<u8>(st), "metal pedal status");
        require(m.torque_status == static_cast<u8>(t_st),
                "metal human_crank_torque status");
        require(m.output.mode == po.mode && m.output.reason == po.reason,
                "metal pedal mode/reason");
        require_bits_f32(m.output.effort_nm, po.effort_nm, "metal pedal effort_nm");
        require_bits_f32(m.output.target_phase_rad, po.target_phase_rad,
                         "metal pedal target_phase_rad");
        require_bits_f32(m.output.target_rate_rad_s, po.target_rate_rad_s,
                         "metal pedal target_rate_rad_s");
        require_bits_f32(m.human_crank_torque_nm, h_tq,
                         "metal human_crank_torque_nm");
        require(m.state.reposition.stall_elapsed_ticks ==
                    host_pedal.reposition.stall_elapsed_ticks,
                "metal stall_elapsed_ticks");
        require(m.state.reposition.elapsed_ticks ==
                    host_pedal.reposition.elapsed_ticks,
                "metal elapsed_ticks");
    }

    AssistState<float> host_assist{};
    assist_reset<float>(&host_assist);
    require(metal.assist_records.size() == req.assist_steps.size(),
            "metal assist record count");
    for (std::size_t i = 0; i < req.assist_steps.size(); ++i) {
        const auto &in = req.assist_steps[i];
        float tq = 0.0f;
        ErrCode err = ErrCode::none;
        const Status st = assist_step<float, GpuToolkit>(
            &req.assist_params, &host_assist,
            in.human_nm, in.cadence_rpm, in.speed_mps, in.braking != 0, in.dt,
            in.has_torque_request != 0, in.torque_request_nm,
            in.has_shaft_rpm != 0, in.shaft_rpm, tq, err);
        const auto &m = metal.assist_records[i];
        require(m.status == static_cast<u8>(st), "metal assist status");
        require_bits_f32(m.torque, tq, "metal assist torque");
        require_bits_f32(m.state.hold_power_w, host_assist.hold_power_w,
                         "metal assist hold_power_w");
        require(m.state.release_hold_ticks == host_assist.release_hold_ticks,
                "metal assist release_hold_ticks");
        require(m.state.pedaling == host_assist.pedaling,
                "metal assist pedaling flag");
    }

    std::cout << "[PASS] Apple Metal SP3 parity (" << metal.device_name
              << "): MSL 3.0 & 3.1 torture + 0-bit parity on "
              << req.shift_steps.size() << " shift, "
              << req.pedal_steps.size() << " pedal, "
              << req.assist_steps.size() << " assist steps\n";
}
#endif

void test_ratchet_phi_large_angle_precision() {
    const float q_ds = 200.0f;
    const float q_rw = 271.4f;
    const float ratio = 1.3571428f;
    const float got = ratchet_phi<float, GpuToolkit>(ratio, q_ds, q_rw);
    const double exact =
        static_cast<double>(ratio) * static_cast<double>(q_ds) - static_cast<double>(q_rw);
    const float ref = static_cast<float>(exact);
    const auto ulp_diff = std::abs(
        static_cast<std::int64_t>(std::bit_cast<std::uint32_t>(got)) -
        static_cast<std::int64_t>(std::bit_cast<std::uint32_t>(ref)));
    require(ulp_diff <= 1, "ratchet_phi<float> at large angle exceeded 1 ULP vs FP64");

    // Also verify ratchet_shift_ratio_bound<float> preserves gap to <= 2 ULP at large angles.
    const float new_ratio = 1.4666667f;
    bool has_b32 = true;
    float b32 = ref + 0.00125f;
    ErrCode err32 = ErrCode::none;
    require(
        ratchet_shift_ratio_bound<float, GpuToolkit>(
            ratio, new_ratio, q_ds, q_rw, has_b32, b32, err32) == Status::ok,
        "ratchet_shift_ratio_bound<float> failed");
    bool has_b64 = true;
    double b64 = exact + 0.00125;
    ErrCode err64 = ErrCode::none;
    require(
        ratchet_shift_ratio_bound<double, CpuExactToolkit>(
            static_cast<double>(ratio), static_cast<double>(new_ratio),
            static_cast<double>(q_ds), static_cast<double>(q_rw),
            has_b64, b64, err64) == Status::ok,
        "ratchet_shift_ratio_bound<double> failed");
    const float b_ref = static_cast<float>(b64);
    const float ulp_b = std::abs(std::nextafter(b_ref, std::copysign(1.0e9f, b_ref)) - b_ref);
    require(std::abs(b32 - b_ref) <= 2.0f * ulp_b,
            "ratchet_shift_ratio_bound<float> at large angle drifted > 2 ULP vs FP64");
}

} // namespace

int main() {
    try {
        test_pod_layout_and_footprint();
        test_shifting_cpu_parity_and_cassette();
        test_pedaling_and_stall_cpu_parity();
        test_assist_cpu_parity();
        test_3600s_int32_tick_drift();
        test_2pi_epoch_rebase_invariance();
        test_ratchet_phi_large_angle_precision();
#if defined(__APPLE__)
        test_apple_metal_sp3_parity();
#endif
        std::cout << "[PASS] test_sp3_drivetrain_controllers\n";
        return 0;
    } catch (const std::exception &err) {
        std::cerr << "[FAIL] test_sp3_drivetrain_controllers: " << err.what() << '\n';
        return 1;
    }
}
