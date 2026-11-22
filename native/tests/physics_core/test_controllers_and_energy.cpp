#include "physics_core/assist.hpp"
#include "physics_core/common.hpp"
#include "physics_core/cpu_exact_toolkit.hpp"
#include "physics_core/cruise.hpp"
#include "physics_core/energy_ledger.hpp"
#include "physics_core/gpu_toolkit.hpp"
#include "physics_core/pedaling.hpp"
#include "physics_core/ride_program.hpp"
#include "physics_core/shifting.hpp"

#include "drivetrain/motor.hpp"
#include "drivetrain/pedaling.hpp"
#include "support/model_fixture.hpp"
#include "writers/cruise.hpp"

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <numbers>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace {

using bike::physics_core::AssistParams;
using bike::physics_core::AssistState;
using bike::physics_core::AssistTickOutput;
using bike::physics_core::CoastReason;
using bike::physics_core::CpuExactToolkit;
using bike::physics_core::CruiseConfigPod;
using bike::physics_core::CruiseStatePod;
using bike::physics_core::ElasticEnergyBreakdown;
using bike::physics_core::ErrCode;
using bike::physics_core::GpuEnergyLedger;
using bike::physics_core::GpuProgramChannelMask;
using bike::physics_core::GpuProgramOverride;
using bike::physics_core::GpuProgramSegment;
using bike::physics_core::GpuRideProgram;
using bike::physics_core::GpuStepEnergyInputs;
using bike::physics_core::GpuToolkit;
using bike::physics_core::MechanicalEnergyBreakdown;
using bike::physics_core::PedalingOutput;
using bike::physics_core::PedalingParams;
using bike::physics_core::PedalingState;
using bike::physics_core::PedalMode;
using bike::physics_core::StallRepositionConfigPod;
using bike::physics_core::StallRepositionStatePod;
using bike::physics_core::Status;
using bike::physics_core::assist_post_solve_settle;
using bike::physics_core::assist_reset;
using bike::physics_core::assist_step;
using bike::physics_core::assist_step_with_limit;
using bike::physics_core::assist_validate_state_invariants;
using bike::physics_core::compute_mechanical_energy;
using bike::physics_core::cruise_init;
using bike::physics_core::cruise_reset;
using bike::physics_core::cruise_restore_state;
using bike::physics_core::cruise_set_assist_compensation;
using bike::physics_core::cruise_set_target_speed_kmh;
using bike::physics_core::cruise_step;
using bike::physics_core::cruise_validate_config;
using bike::physics_core::energy_brake_dissipation_step;
using bike::physics_core::energy_closure_work_ok;
using bike::physics_core::energy_ledger_contract_ok;
using bike::physics_core::energy_ledger_init;
using bike::physics_core::energy_ledger_step;
using bike::physics_core::eval_ride_program;
using bike::physics_core::i32;
using bike::physics_core::kMaxProgramSegments;
using bike::physics_core::pedaling_reset;
using bike::physics_core::pedaling_step_with_reposition;
using bike::physics_core::pedaling_validate_reposition_config;
using bike::physics_core::pedaling_validate_state_invariants;
using bike::physics_core::rebase_pedaling_2pi;
using bike::physics_core::shift_cut_active;
using bike::physics_core::shift_cut_active_ticks;
using bike::physics_core::shifting_gear_ratio;
using bike::physics_core::shifting_interpolated_gear_ratio;
using bike::physics_core::shifting_reset;
using bike::physics_core::u32;
using bike::physics_core::u8;

void require(bool cond, std::string_view msg) {
    bt::require(cond, msg);
}

void require_bits_f64(double actual, double expected, std::string_view what) {
    const auto ua = std::bit_cast<std::uint64_t>(actual);
    const auto ue = std::bit_cast<std::uint64_t>(expected);
    if (ua != ue) {
        std::ostringstream oss;
        oss << what << ": 64-bit mismatch (actual=" << actual
            << " [0x" << std::hex << ua << "], expected=" << expected
            << " [0x" << ue << "])";
        throw std::runtime_error(oss.str());
    }
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

void test_cruise_0bit_parity_and_validation() {
    constexpr std::string_view kCruiseXml = R"XML(
<mujoco><option timestep="0.00125"/>
<default><geom type="sphere" size=".05" mass="1" contype="0" conaffinity="0"/></default>
<worldbody><body name="frame"><joint name="root_x" type="slide" axis="1 0 0"/><geom/></body></worldbody>
</mujoco>
)XML";
    bt::ModelFixture fixture(kCruiseXml);
    const int root_dof = fixture.dof("root_x");

    const nativecfg::CruiseConfig cpu_cfg{
        .target_speed_kmh = 28.0,
        .kp_nm_per_mps = 120.0,
        .ki_nm_per_mps_s = 40.0,
        .torque_ceiling_nm = 90.0,
    };
    const CruiseConfigPod<double> pod_cfg{
        .target_speed_kmh = 28.0,
        .kp_nm_per_mps = 120.0,
        .ki_nm_per_mps_s = 40.0,
        .torque_ceiling_nm = 90.0,
    };

    ErrCode err = ErrCode::none;
    require(cruise_validate_config<double, CpuExactToolkit>(&pod_cfg, err) == Status::ok,
            "valid cruise config accepted");

    {
        CruiseConfigPod<double> bad = pod_cfg;
        bad.kp_nm_per_mps = 0.0;
        require(cruise_validate_config<double, CpuExactToolkit>(&bad, err) ==
                    Status::invalid_argument,
                "kp <= 0 rejected");
        bad = pod_cfg;
        bad.target_speed_kmh = 14.99;
        require(cruise_validate_config<double, CpuExactToolkit>(&bad, err) ==
                    Status::invalid_argument,
                "target_speed_kmh < 15 rejected");
        bad = pod_cfg;
        bad.target_speed_kmh = 45.01;
        require(cruise_validate_config<double, CpuExactToolkit>(&bad, err) ==
                    Status::invalid_argument,
                "target_speed_kmh > 45 rejected");
        bad = pod_cfg;
        bad.ki_nm_per_mps_s = std::numeric_limits<double>::quiet_NaN();
        require(cruise_validate_config<double, CpuExactToolkit>(&bad, err) ==
                    Status::invalid_argument,
                "NaN ki rejected");
    }

    CruiseWriter writer(fixture.model.get(), cpu_cfg);
    CruiseStatePod<double> pod_state{};
    require(cruise_init<double, CpuExactToolkit>(&pod_cfg, &pod_state, err) ==
                Status::ok,
            "cruise_init succeeds");
    require_bits_f64(pod_state.target_speed_mps,
                     writer.state().target_speed_mps,
                     "init target_speed_mps");

    struct StepCase {
        double speed_mps = 0.0;
        bool rear_in_contact = false;
        bool traction_limited = false;
        std::optional<bool> controller_grounded{};
        double dt_s = 0.00125;
        std::optional<double> set_kmh{};
        std::optional<double> support_factor{};
        bool do_reset = false;
    };

    const StepCase cases[] = {
        {0.0, false, false, std::nullopt, 0.00125, std::nullopt, std::nullopt, false},
        {3.0, false, false, true, 0.00125, std::nullopt, std::nullopt, false},
        {5.0, true, false, false, 0.00125, std::nullopt, std::nullopt, false},
        {6.0, true, false, std::nullopt, 0.00125, std::nullopt, std::nullopt, false},
        {7.5, true, true, std::nullopt, 0.00125, std::nullopt, 1.4, false},
        {7.8, true, false, std::nullopt, 0.0025, 32.0, -0.5, false},
        {2.0, true, false, std::nullopt, 0.005, std::nullopt, std::nullopt, false},
        {12.0, true, false, std::nullopt, 0.005, std::nullopt, std::nullopt, false},
        {8.0, true, false, std::nullopt, 0.00125, std::nullopt, std::nullopt, true},
        {8.5, true, false, true, 0.00125, std::nullopt, std::nullopt, false},
    };

    for (std::size_t i = 0; i < std::size(cases); ++i) {
        const auto &tc = cases[i];
        if (tc.do_reset) {
            writer.reset();
            cruise_reset<double>(&pod_state);
        }
        if (tc.set_kmh) {
            writer.set_target_speed(*tc.set_kmh);
            require(cruise_set_target_speed_kmh<double, CpuExactToolkit>(
                        &pod_state, *tc.set_kmh, err) == Status::ok,
                    "cruise_set_target_speed_kmh ok");
        }
        if (tc.support_factor) {
            const double cpu_scale =
                writer.set_assist_compensation(*tc.support_factor);
            double pod_scale = 0.0;
            require(cruise_set_assist_compensation<double, CpuExactToolkit>(
                        &pod_cfg, &pod_state, *tc.support_factor, pod_scale, err) ==
                        Status::ok,
                    "cruise_set_assist_compensation ok");
            require_bits_f64(pod_scale, cpu_scale, "assist compensation scale");
        }
        fixture.model->opt.timestep = static_cast<mjtNum>(tc.dt_s);
        fixture.data->qvel[static_cast<std::size_t>(root_dof)] =
            static_cast<mjtNum>(tc.speed_mps);
        const double cpu_tq = writer.compute(
            fixture.data.get(), tc.rear_in_contact, tc.traction_limited,
            tc.controller_grounded);
        double pod_tq = 0.0;
        const Status pod_st = cruise_step<double, CpuExactToolkit>(
            &pod_cfg,
            &pod_state,
            tc.speed_mps,
            tc.dt_s,
            tc.rear_in_contact,
            tc.traction_limited,
            tc.controller_grounded.has_value(),
            tc.controller_grounded.value_or(false),
            pod_tq,
            err);
        require(pod_st == Status::ok, "cruise_step ok");
        require_bits_f64(pod_tq, cpu_tq, "cruise torque 0-bit parity");
        require_bits_f64(pod_state.target_speed_mps,
                         writer.state().target_speed_mps,
                         "cruise target_speed_mps 0-bit parity");
        require_bits_f64(pod_state.integral_mps_s,
                         writer.state().integral_mps_s,
                         "cruise integral_mps_s 0-bit parity");
        require_bits_f64(pod_state.torque_nm,
                         writer.state().torque_nm,
                         "cruise torque_nm 0-bit parity");
        require_bits_f64(pod_state.gain_scale,
                         writer.state().gain_scale,
                         "cruise gain_scale 0-bit parity");
        require((pod_state.engaged != 0) == writer.state().engaged,
                "cruise engaged parity");
    }

    // Restore validation + atomicity
    const CruiseStatePod<double> saved = pod_state;
    CruiseStatePod<double> bad_restore = saved;
    bad_restore.target_speed_mps = 14.0 / 3.6;
    require(cruise_restore_state<double, CpuExactToolkit>(
                &pod_cfg, &pod_state, &bad_restore, err) ==
                Status::invalid_argument,
            "restore rejects out-of-band target_speed_mps");
    require_bits_f64(pod_state.target_speed_mps, saved.target_speed_mps,
                     "failed restore preserves state");

    // FP32 GpuToolkit smoke check
    const CruiseConfigPod<float> cfg32{
        .target_speed_kmh = 28.0f,
        .kp_nm_per_mps = 120.0f,
        .ki_nm_per_mps_s = 40.0f,
        .torque_ceiling_nm = 90.0f,
    };
    CruiseStatePod<float> st32{};
    require(cruise_init<float, GpuToolkit>(&cfg32, &st32, err) == Status::ok,
            "FP32 cruise_init ok");
    float tq32 = 0.0f;
    require(cruise_step<float, GpuToolkit>(
                &cfg32, &st32, 6.0f, 0.00125f, true, false, false, false, tq32, err) ==
                Status::ok,
            "FP32 cruise_step ok");
    require(tq32 > 0.0f && tq32 <= 90.0f, "FP32 cruise output in range");
}

void test_ride_program_steps_and_ramps() {
    GpuRideProgram<double> prog{};
    prog.reaction_delay_s = 0.2;
    prog.segment_count = 4;

    // Segment 0: initial knot at t=0.0 (anchor for ramp in segment 1)
    prog.segments[0].t_end_s = 0.0;
    prog.segments[0].channel_mask =
        static_cast<u32>(GpuProgramChannelMask::has_human_torque) |
        static_cast<u32>(GpuProgramChannelMask::has_front_brake);
    prog.segments[0].human_torque_nm = 10.0;
    prog.segments[0].front_brake = 0.0;

    // Segment 1: linear ramp over [0.0, 2.0)
    prog.segments[1].t_end_s = 2.0;
    prog.segments[1].channel_mask =
        static_cast<u32>(GpuProgramChannelMask::has_human_torque) |
        static_cast<u32>(GpuProgramChannelMask::has_front_brake) |
        static_cast<u32>(GpuProgramChannelMask::has_cruise_speed_kmh) |
        static_cast<u32>(GpuProgramChannelMask::linear_ramp);
    prog.segments[1].human_torque_nm = 50.0;
    prog.segments[1].front_brake = 0.8;
    prog.segments[1].cruise_speed_kmh = 30.0;

    // Segment 2: piecewise-constant step over [2.0, 4.0)
    prog.segments[2].t_end_s = 4.0;
    prog.segments[2].channel_mask =
        static_cast<u32>(GpuProgramChannelMask::has_motor_torque) |
        static_cast<u32>(GpuProgramChannelMask::has_motor_limit) |
        static_cast<u32>(GpuProgramChannelMask::has_posture_lean);
    prog.segments[2].motor_torque_nm = 42.0;
    prog.segments[2].motor_limit_nm = 60.0;
    prog.segments[2].posture_lean_rad = 0.25;

    // Segment 3: linear ramp over [4.0, 5.0)
    prog.segments[3].t_end_s = 5.0;
    prog.segments[3].channel_mask =
        static_cast<u32>(GpuProgramChannelMask::has_motor_torque) |
        static_cast<u32>(GpuProgramChannelMask::has_rear_brake) |
        static_cast<u32>(GpuProgramChannelMask::linear_ramp);
    prog.segments[3].motor_torque_nm = 0.0;
    prog.segments[3].rear_brake = 1.0;

    ErrCode err = ErrCode::none;
    GpuProgramOverride<double> ov{};

    // During reaction delay (time_s = 0.1 -> t = 0.0): start of segment 1 ramp
    require(eval_ride_program<double, CpuExactToolkit>(&prog, 0.1, ov, err) ==
                Status::ok,
            "eval_ride_program at t=0.1 ok");
    require((ov.channel_mask &
             static_cast<u32>(GpuProgramChannelMask::linear_ramp)) == 0u,
            "linear_ramp bit stripped from out_override.channel_mask");
    require_near(ov.human_torque_nm, 10.0, 1e-12, "ramp start human_torque_nm");
    require_near(ov.front_brake, 0.0, 1e-12, "ramp start front_brake");
    require_near(ov.cruise_speed_kmh, 0.0, 1e-12,
                 "channel newly added in ramp starts from 0");

    // Mid-ramp (time_s = 1.2 -> t = 1.0, u = 0.5)
    require(eval_ride_program<double, CpuExactToolkit>(&prog, 1.2, ov, err) ==
                Status::ok,
            "eval_ride_program mid-ramp ok");
    require_near(ov.human_torque_nm, 30.0, 1e-12, "mid-ramp human_torque_nm");
    require_near(ov.front_brake, 0.4, 1e-12, "mid-ramp front_brake");
    require_near(ov.cruise_speed_kmh, 15.0, 1e-12, "mid-ramp cruise_speed_kmh");

    // Piecewise-constant segment 2 (time_s = 2.5 -> t = 2.3)
    require(eval_ride_program<double, CpuExactToolkit>(&prog, 2.5, ov, err) ==
                Status::ok,
            "eval_ride_program step segment ok");
    require(ov.channel_mask ==
                (static_cast<u32>(GpuProgramChannelMask::has_motor_torque) |
                 static_cast<u32>(GpuProgramChannelMask::has_motor_limit) |
                 static_cast<u32>(GpuProgramChannelMask::has_posture_lean)),
            "step segment mask exact");
    require_near(ov.motor_torque_nm, 42.0, 1e-12, "step motor_torque_nm");
    require_near(ov.motor_limit_nm, 60.0, 1e-12, "step motor_limit_nm");
    require_near(ov.posture_lean_rad, 0.25, 1e-12, "step posture_lean_rad");

    // Segment 3 ramp at t = 4.25 (time_s = 4.45, u = 0.25)
    require(eval_ride_program<double, CpuExactToolkit>(&prog, 4.45, ov, err) ==
                Status::ok,
            "eval_ride_program seg 3 ramp ok");
    require_near(ov.motor_torque_nm, 31.5, 1e-12, "seg 3 ramp motor_torque_nm");
    require_near(ov.rear_brake, 0.25, 1e-12, "seg 3 ramp rear_brake");

    // Past end (time_s = 10.0 -> holds final segment values)
    require(eval_ride_program<double, CpuExactToolkit>(&prog, 10.0, ov, err) ==
                Status::ok,
            "eval_ride_program past end holds final segment");
    require_near(ov.motor_torque_nm, 0.0, 1e-12, "past-end motor_torque_nm");
    require_near(ov.rear_brake, 1.0, 1e-12, "past-end rear_brake");

    // Validation rejections
    {
        GpuRideProgram<double> bad = prog;
        bad.segments[2].t_end_s = 2.0; // equal to prev -> non-monotonic
        require(eval_ride_program<double, CpuExactToolkit>(&bad, 1.0, ov, err) ==
                    Status::invalid_argument,
                "non-monotonic t_end_s rejected");

        bad = prog;
        bad.segments[2].motor_torque_nm = std::numeric_limits<double>::infinity();
        require(eval_ride_program<double, CpuExactToolkit>(&bad, 1.0, ov, err) ==
                    Status::invalid_argument,
                "non-finite channel value rejected");

        bad = prog;
        bad.segment_count = kMaxProgramSegments + 1;
        require(eval_ride_program<double, CpuExactToolkit>(&bad, 1.0, ov, err) ==
                    Status::invalid_argument,
                "segment_count > kMaxProgramSegments rejected");

        require(eval_ride_program<double, CpuExactToolkit>(
                    &prog, std::numeric_limits<double>::quiet_NaN(), ov, err) ==
                    Status::invalid_argument,
                "non-finite time_s rejected");
    }

    // Empty program returns ok with zero mask
    GpuRideProgram<float> empty{};
    GpuProgramOverride<float> ov32{};
    require(eval_ride_program<float, GpuToolkit>(&empty, 0.5f, ov32, err) ==
                Status::ok &&
                ov32.channel_mask == 0u,
            "empty program yields zero mask");
}

void test_shifting_p04b_and_ratio_sweep() {
    // P04b: no (0, 1e-12] split
    require(!shift_cut_active<double>(0.0), "0.0 not active");
    require(!shift_cut_active<double>(1e-12), "1e-12 boundary not active");
    require(!shift_cut_active<double>(-1e-10), "negative not active");
    require(shift_cut_active<double>(1.0001e-12), "> 1e-12 active");
    require(shift_cut_active<double>(0.2), "0.2 active");

    require(!shift_cut_active<float>(0.0f), "0.0f not active");
    require(!shift_cut_active<float>(1e-12f), "1e-12f not active");
    require(shift_cut_active<float>(1e-6f), "1e-6f active");

    require(!shift_cut_active_ticks(0), "0 ticks not active");
    require(!shift_cut_active_ticks(-1), "-1 ticks not active");
    require(shift_cut_active_ticks(1), "1 tick active");

    // Continuous linear ratio sweep matching DrivetrainWriter::stage_pedal_advance
    const double dt = 0.00125;
    const double applied0 = 22.0 / 24.0;
    const double target = 22.0 / 28.0;
    double cut_s = 0.2;
    double applied = applied0;
    for (int step = 0; step < 160; ++step) {
        cut_s = std::max(0.0, cut_s - dt);
        applied = shifting_interpolated_gear_ratio<double, CpuExactToolkit>(
            applied, target, cut_s, dt);
    }
    require_near(applied, target, 1e-14,
                 "continuous ratio sweep converges cleanly to target");

    // Tick-based linear sweep
    bike::physics_core::ShiftingParams<double> sp{};
    sp.front_teeth = 22;
    sp.initial_rear_teeth = 24;
    bike::physics_core::ShiftingState<double> ss{};
    shifting_reset<double>(&sp, &ss);
    ss.from_teeth = 24;
    ss.rear_teeth = 28;
    ss.cut_remaining_ticks = 160;
    double tick_applied = applied0;
    for (int step = 0; step < 160; ++step) {
        --ss.cut_remaining_ticks;
        tick_applied = shifting_interpolated_gear_ratio<double, CpuExactToolkit>(
            &sp, &ss, tick_applied);
    }
    require_near(tick_applied, target, 1e-14,
                 "tick-based ratio sweep converges cleanly to target");
}

void test_pedaling_p01b_c2_and_stall_reposition() {
    ErrCode err = ErrCode::none;

    // P01b: reposition_on_stall requires motor_clutch_present and spindle attachment
    {
        StallRepositionConfigPod<double> bad_cfg{};
        bad_cfg.reposition_on_stall = 1;
        bad_cfg.motor_clutch_present = 0;
        require(pedaling_validate_reposition_config<double, CpuExactToolkit>(
                    &bad_cfg, err) == Status::invalid_argument,
                "P01b rejects reposition_on_stall without motor clutch");

        bad_cfg.motor_clutch_present = 1;
        bad_cfg.spindle_pedal_attachment = 0;
        require(pedaling_validate_reposition_config<double, CpuExactToolkit>(
                    &bad_cfg, err) == Status::invalid_argument,
                "P01b rejects reposition_on_stall without spindle attachment");

        bad_cfg.reposition_on_stall = 0;
        bad_cfg.motor_clutch_present = 0;
        require(pedaling_validate_reposition_config<double, CpuExactToolkit>(
                    &bad_cfg, err) == Status::ok,
                "P01b allows disabled reposition without motor clutch");
    }

    // C2 relational invariants
    {
        PedalingState<double> s{};
        StallRepositionStatePod<double> r{};
        r.active = 1;
        require(pedaling_validate_state_invariants<double, CpuExactToolkit>(
                    &s, &r, err) == Status::ok,
                "fresh C2 state valid");

        // !stall_elapsed => crank_rad == 0 && wheel_rad == 0
        r.has_stall_elapsed = 0;
        r.crank_rad = 0.1;
        require(pedaling_validate_state_invariants<double, CpuExactToolkit>(
                    &s, &r, err) == Status::invalid_argument,
                "C2 rejects nonzero crank_rad without stall_elapsed");

        // goal_rad <=> (reason == 1 || reason == 2)
        r = StallRepositionStatePod<double>{};
        r.active = 1;
        r.has_goal = 1;
        r.reason = 0;
        require(pedaling_validate_state_invariants<double, CpuExactToolkit>(
                    &s, &r, err) == Status::invalid_argument,
                "C2 rejects goal without reason 1/2");

        r.has_goal = 0;
        r.reason = 1;
        require(pedaling_validate_state_invariants<double, CpuExactToolkit>(
                    &s, &r, err) == Status::invalid_argument,
                "C2 rejects reason 1 without goal");

        // goal_rad => cooldown_s == 0 && !coasting && effort == 0
        r.has_goal = 1;
        r.reason = 2;
        r.cooldown_s = 0.1;
        require(pedaling_validate_state_invariants<double, CpuExactToolkit>(
                    &s, &r, err) == Status::invalid_argument,
                "C2 rejects goal with nonzero cooldown");
        r.cooldown_s = 0.0;
        s.effort = 5.0;
        require(pedaling_validate_state_invariants<double, CpuExactToolkit>(
                    &s, &r, err) == Status::invalid_argument,
                "C2 rejects goal with nonzero effort");
    }

    // Parity against drivetrain::PedalingPolicy across stall + reposition
    drivetrain::PedalingConfig cpu_cfg{
        .coast_above_rpm = 120.0,
        .resume_below_rpm = 105.0,
        .stop_time_s = 0.35,
        .coast_cadence_tau_s = 0.35,
        .mash_cadence_rpm = 45.0,
        .mash_torque_nm = 60.0,
        .effort_slew_nm_s = 300.0,
        .reposition_on_stall = true,
        .reposition_min_effort_nm = 20.0,
        .reposition_stall_window_s = 0.25,
        .reposition_stall_progress_rad = 0.5,
        .reposition_cooldown_s = 0.30,
        .reposition_back_rate_rad_s = 3.0,
        .reposition_damping_nms_rad = 12.0,
        .reposition_timeout_s = 1.0,
        .reposition_phase_tolerance_rad = 0.05,
        .reposition_noop_rad = 0.12,
    };
    drivetrain::PedalingPolicy cpu_policy(cpu_cfg);

    PedalingParams<double> pp{};
    pp.coast_above_rpm = 120.0;
    pp.resume_below_rpm = 105.0;
    pp.stop_time_s = 0.35;
    pp.coast_cadence_tau_s = 0.35;
    pp.mash_cadence_rpm = 45.0;
    pp.mash_torque_nm = 60.0;
    pp.effort_slew_nm_s = 300.0;
    pp.enabled = 1;

    StallRepositionConfigPod<double> rc{};
    rc.reposition_on_stall = 1;
    rc.motor_clutch_present = 1;
    rc.spindle_pedal_attachment = 1;
    rc.min_effort_nm = 20.0;
    rc.stall_window_s = 0.25;
    rc.stall_progress_rad = 0.5;
    rc.cooldown_s = 0.30;
    rc.back_rate_rad_s = 3.0;
    rc.timeout_s = 1.0;
    rc.phase_tolerance_rad = 0.05;
    rc.noop_rad = 0.12;

    PedalingState<double> ps{};
    pedaling_reset<double>(&ps);
    StallRepositionStatePod<double> rs{};

    const double dt = 0.01;
    double phase = 1.4;
    bool saw_stall_reposition = false;
    bool saw_request_reposition = false;

    bool sent_request = false;
    for (int step = 0; step < 150; ++step) {
        const bool braking = (step >= 70 && step < 75);
        double turn = std::fmod(phase, std::numbers::pi);
        if (turn < 0.0) {
            turn += std::numbers::pi;
        }
        const bool away_from_dead_spot =
            std::min(turn, std::numbers::pi - turn) > 0.3;
        const bool req_edge =
            !sent_request && step >= 110 && away_from_dead_spot;
        if (req_edge) {
            sent_request = true;
        }
        const double rate = (step < 30) ? 0.1 : (rs.has_goal ? rs.goal_rad - phase : 6.0);
        const double req_rpm = (step < 30) ? 0.5 : 60.0;
        const double effort = 35.0;

        const auto cpu_out = cpu_policy.update(
            phase, rate, req_rpm, effort, dt, true, braking, req_edge,
            std::nullopt, std::nullopt, 1.0, false);

        PedalingOutput<double> gpu_out{};
        const Status st = pedaling_step_with_reposition<double, CpuExactToolkit>(
            &pp, &ps, &rc, &rs, phase, rate, req_rpm, effort, dt, braking,
            req_edge, false, 0.0, false, 0.0, 1.0, false, gpu_out, err);
        require(st == Status::ok, "pedaling_step_with_reposition ok");
        require(pedaling_validate_state_invariants<double, CpuExactToolkit>(
                    &ps, &rs, err) == Status::ok,
                "C2 invariant holds after every step");

        require(static_cast<int>(gpu_out.mode) == static_cast<int>(cpu_out.mode),
                "pedaling mode matches CPU");
        require(static_cast<int>(gpu_out.reason) ==
                    static_cast<int>(cpu_out.reason),
                "pedaling coast reason matches CPU");
        require_near(gpu_out.effort_nm, cpu_out.effort_nm, 1e-12,
                     "pedaling effort_nm parity");
        require_near(gpu_out.target_rate_rad_s, cpu_out.target_rate_rad_s,
                     1e-12, "pedaling target_rate_rad_s parity");
        if (cpu_out.target_phase_rad.has_value()) {
            require(gpu_out.has_target_phase != 0, "has_target_phase matches");
            require_near(gpu_out.target_phase_rad, *cpu_out.target_phase_rad,
                         1e-12, "target_phase_rad parity");
        }
        if (gpu_out.reason == CoastReason::reposition_stall) {
            saw_stall_reposition = true;
        }
        if (gpu_out.reason == CoastReason::reposition_requested) {
            saw_request_reposition = true;
        }
        if (gpu_out.has_target_phase != 0) {
            phase = gpu_out.target_phase_rad;
        } else {
            phase += rate * dt;
        }
    }
    require(saw_stall_reposition, "stall reposition triggered");
    require(saw_request_reposition, "request reposition triggered");

    // 2*pi rebase preserves progress deltas and shifts goal_rad
    rs.active = 1;
    rs.has_goal = 1;
    rs.reason = 1;
    rs.cooldown_s = 0.0;
    rs.goal_rad = 9.0;
    ps.has_target_phase = 1;
    ps.target_phase_rad = 10.0;
    const double two_pi = 2.0 * std::numbers::pi;
    rebase_pedaling_2pi<double>(ps, rs, two_pi, two_pi);
    require_near(rs.goal_rad, 9.0 - two_pi, 1e-12, "rebase shifts repos goal_rad");
    require_near(ps.target_phase_rad, 10.0 - two_pi, 1e-12,
                 "rebase shifts target_phase_rad");
}

void test_assist_c3_limit_ordering_and_settle() {
    ErrCode err = ErrCode::none;

    // C3 relational invariants
    {
        AssistState<double> s{};
        require(assist_validate_state_invariants<double, CpuExactToolkit>(
                    &s, err) == Status::ok,
                "fresh AssistState valid");

        s.release_hold_ticks = 10;
        s.hold_power_w = 0.0;
        s.hold_torque_nm = 20.0;
        require(assist_validate_state_invariants<double, CpuExactToolkit>(
                    &s, err) == Status::invalid_argument,
                "C3 rejects release_hold_ticks > 0 with hold_power_w == 0");

        s.release_hold_ticks = 0;
        s.hold_power_w = 100.0;
        s.hold_torque_nm = 0.0;
        require(assist_validate_state_invariants<double, CpuExactToolkit>(
                    &s, err) == Status::invalid_argument,
                "C3 rejects hold_power_w > 0 with hold_torque_nm == 0");

        s.hold_torque_nm = 15.0;
        require(assist_validate_state_invariants<double, CpuExactToolkit>(
                    &s, err) == Status::ok,
                "C3 accepts spent hold marker (release_hold_ticks == 0)");
    }

    AssistParams<double> ap{};
    ap.gain = 2.0;
    ap.max_torque = 85.0;
    ap.max_power = 600.0;
    ap.tau = 0.04;
    ap.slew = 2000.0;
    ap.engage_torque_nm = 4.0;
    ap.release_hold_ticks = 240;
    ap.cutoff_mps = 25.0 / 3.6;
    ap.taper_width_mps = 2.0 / 3.6;

    AssistState<double> as{};
    assist_reset<double>(&as);

    // Step with motor_limit_nm < request_nm: verify request_nm is un-limited,
    // safety_nm / delivered_nm / state.torque take the limit.
    AssistTickOutput<double> out{};
    require(assist_step_with_limit<double, CpuExactToolkit>(
                &ap, &as, 30.0, 75.0, 4.0, false, 0.01,
                false, 0.0, true, 5.0, false, 0.0, true, out, err) == Status::ok,
            "assist_step_with_limit ok");
    require(out.request_nm > 5.0, "un-limited assist request exceeds limit");
    require_bits_f64(out.safety_nm, 5.0, "safety_nm matches limit");
    require_bits_f64(out.delivered_nm, 5.0, "delivered_nm matches limit");
    require_bits_f64(as.torque, 5.0, "state.torque reconciled to delivered");
    require(out.enabled == 1 && out.safety_limited == 1,
            "enabled and safety_limited flags set");

    // Post-solve settle
    double settled = 0.0;
    require(assist_post_solve_settle<double, CpuExactToolkit>(
                &as, out.delivered_nm, 4.75, settled, err) == Status::ok,
            "assist_post_solve_settle accepts valid actuator force");
    require_bits_f64(settled, 4.75, "settled torque matches actuator force");
    require_bits_f64(as.torque, 4.75, "state.torque updated by settle");

    // Tiny negative numerical noise within -1e-10 clamps to 0
    require(assist_post_solve_settle<double, CpuExactToolkit>(
                &as, 4.75, -5e-11, settled, err) == Status::ok,
            "assist_post_solve_settle clamps sub-tol negative noise");
    require_bits_f64(settled, 0.0, "clamped settled torque is 0.0");

    // Violating reserved effort ceiling returns Status::logic_error
    require(assist_post_solve_settle<double, CpuExactToolkit>(
                &as, 4.75, 4.76, settled, err) == Status::logic_error,
            "settle rejects solved force exceeding reserved ceiling");
    require(assist_post_solve_settle<double, CpuExactToolkit>(
                &as, 4.75, -1e-5, settled, err) == Status::logic_error,
            "settle rejects solved force below negative tolerance");
}

void test_energy_ledger_csr_and_conservation() {
    ErrCode err = ErrCode::none;

    // 1. Verify compute_mechanical_energy with a 3-DOF lower-triangular CSR mass matrix
    // M = [[4, 1, 0],
    //      [1, 3, 0.5],
    //      [0, 0.5, 2]]
    const i32 M_rowadr[3] = {0, 1, 3};
    const i32 M_rownnz[3] = {1, 2, 2};
    const i32 M_colind[5] = {0, 0, 1, 1, 2};
    const double M_vals[5] = {4.0, 1.0, 3.0, 0.5, 2.0};
    const double qvel[3] = {2.0, -1.0, 3.0};
    // q^T M q = 4*4 + 3*1 + 2*9 + 2*(1*2*(-1) + 0.5*(-1)*3) = 37 + 2*(-3.5) = 30 -> K = 15.0

    const double body_mass[2] = {10.0, 70.0};
    const double xipos[6] = {1.0, 0.0, 0.5, 1.2, 0.0, 1.2};
    const double gravity[3] = {0.0, 0.0, -9.81};
    // U_g = -(10*(-9.81)*0.5 + 70*(-9.81)*1.2) = 9.81 * (5 + 84) = 873.09

    ElasticEnergyBreakdown<double> elastic{};
    elastic.u_susp_fork = 12.0;
    elastic.u_susp_shock = 18.0;
    elastic.u_tyre_radial = 5.0;
    elastic.u_tyre_shear = 1.5;
    elastic.u_tyre_maxwell = 0.75;
    elastic.u_tyre_rim = 0.25;

    MechanicalEnergyBreakdown<double> mech{};
    require(compute_mechanical_energy<double, CpuExactToolkit>(
                3, 2, M_rowadr, M_rownnz, M_colind, M_vals, qvel,
                body_mass, xipos, gravity, elastic, mech, err) == Status::ok,
            "compute_mechanical_energy ok");
    require_near(mech.kinetic_j, 15.0, 1e-13, "CSR kinetic energy exact");
    require_near(mech.gravitational_j, 873.09, 1e-12, "gravitational PE exact");
    require_near(mech.elastic_j, 37.5, 1e-13, "elastic energy sum exact");
    require_near(mech.total_j, 15.0 + 873.09 + 37.5, 1e-12, "total E_mech exact");

    // 2. Exact energy conservation on a synthetic damped & forced harmonic oscillator
    // integrated with implicit midpoint (discrete work-energy identity holds to < 1e-13):
    //   m * (v1 - v0)/dt = f_ext - k * x_mid - c * v_mid
    //   (x1 - x0)/dt = v_mid
    const double m_val = 2.5;
    const double k_val = 180.0;
    const double c_val = 1.8;
    const double dt = 0.00125;
    const i32 m1_ra[1] = {0};
    const i32 m1_nnz[1] = {1};
    const i32 m1_ci[1] = {0};
    const double m1_d[1] = {m_val};
    const double g_zero[3] = {0.0, 0.0, 0.0};
    const double bm1[1] = {m_val};
    double xi1[3] = {0.0, 0.0, 0.0};

    double x = 0.15;
    double v = 0.8;

    auto eval_E = [&](double pos, double vel) {
        double E = 0.0;
        const double u_fork = 0.5 * k_val * pos * pos;
        const double qv[1] = {vel};
        ErrCode e = ErrCode::none;
        require(compute_mechanical_energy<double, CpuExactToolkit>(
                    1, 1, m1_ra, m1_nnz, m1_ci, m1_d, qv, bm1, xi1, g_zero,
                    u_fork, 0.0, 0.0, 0.0, 0.0, 0.0, E, e) == Status::ok,
                "eval_E ok");
        return E;
    };

    GpuEnergyLedger<double> ledger{};
    require(energy_ledger_init<double, CpuExactToolkit>(
                &ledger, eval_E(x, v), 1.0, err) == Status::ok,
            "energy_ledger_init ok");

    // Implicit midpoint linear solve coefficients:
    //   v1 = ( (m - 0.5*dt*c - 0.25*dt^2*k)*v0 - dt*k*x0 + dt*f_ext ) /
    //        ( m + 0.5*dt*c + 0.25*dt^2*k )
    const double denom = m_val + 0.5 * dt * c_val + 0.25 * dt * dt * k_val;
    const double num_v = m_val - 0.5 * dt * c_val - 0.25 * dt * dt * k_val;

    for (int step = 0; step < 800; ++step) {
        const double f_muscle = (step < 300) ? 12.0 : 0.0;
        const double f_motor = (step >= 100 && step < 450) ? 8.0 : 0.0;
        const double f_ext = f_muscle + f_motor;

        const double v_next = (num_v * v - dt * k_val * x + dt * f_ext) / denom;
        const double v_mid = 0.5 * (v + v_next);
        const double x_next = x + dt * v_mid;

        const double w_muscle = f_muscle * v_mid * dt;
        const double w_motor = f_motor * v_mid * dt;
        const double d_susp = c_val * v_mid * v_mid * dt;

        GpuStepEnergyInputs<double> in{};
        in.E_mech = eval_E(x_next, v_next);
        in.w_muscle_signed_j = w_muscle;
        in.w_muscle_pos_j = std::max(0.0, w_muscle);
        in.w_motor_signed_j = w_motor;
        in.w_motor_pos_j = std::max(0.0, w_motor);
        in.d_suspension_j = d_susp;

        require(energy_ledger_step<double, CpuExactToolkit>(&ledger, in, err) ==
                    Status::ok,
                "energy_ledger_step ok");
        x = x_next;
        v = v_next;
    }

    require(ledger.max_rel_residual < 1e-12,
            "harmonic damped/forced midpoint ledger |rel_residual| < 1e-12");
    require(energy_ledger_contract_ok<double, CpuExactToolkit>(
                &ledger, 0.05, 1e-8),
            "ledger passes < 0.05 strict contract");

    // 3. Brake dissipation helper & closure work check
    double hold_j = 0.0;
    double fric_j = 0.0;
    require(energy_brake_dissipation_step<double, CpuExactToolkit>(
                -50.0, -40.0, -100.0, -80.0, 5.0, 5.0, dt, hold_j, fric_j, err) ==
                Status::ok,
            "energy_brake_dissipation_step ok");
    require_near(hold_j, (50.0 + 40.0) * 5.0 * dt, 1e-13, "hold brake dissipation");
    require_near(fric_j, (100.0 + 80.0) * 5.0 * dt, 1e-13,
                 "friction dof brake dissipation");
    require(energy_closure_work_ok<double, CpuExactToolkit>(0.5, 100.0, 1e-8),
            "closure work <= 1% source positive ok");
    require(!energy_closure_work_ok<double, CpuExactToolkit>(1.5, 100.0, 1e-8),
            "closure work > 1% source positive rejected");

    // 4. FP32 GpuToolkit ledger check
    GpuEnergyLedger<float> ledger32{};
    require(energy_ledger_init<float, GpuToolkit>(&ledger32, 10.0f, 1.0f, err) ==
                Status::ok,
            "FP32 ledger init ok");
    GpuStepEnergyInputs<float> in32{};
    in32.E_mech = 12.5f;
    in32.w_muscle_signed_j = 4.0f;
    in32.w_muscle_pos_j = 4.0f;
    in32.d_tyre_radial_j = 0.5f;
    in32.d_tyre_brush_roll_j = 0.5f;
    in32.d_resistance_j = 0.25f;
    in32.d_brake_hold_j = 0.15f;
    in32.d_brake_friction_j = 0.10f;
    require(energy_ledger_step<float, GpuToolkit>(&ledger32, in32, err) ==
                Status::ok,
            "FP32 ledger step ok");
    require(ledger32.max_rel_residual < 1e-6f, "FP32 ledger exact on step");
    require(energy_ledger_contract_ok<float, GpuToolkit>(&ledger32, 0.05f, 1e-6f),
            "FP32 ledger contract ok");
}

} // namespace

int main() {
    try {
        test_cruise_0bit_parity_and_validation();
        test_ride_program_steps_and_ramps();
        test_shifting_p04b_and_ratio_sweep();
        test_pedaling_p01b_c2_and_stall_reposition();
        test_assist_c3_limit_ordering_and_settle();
        test_energy_ledger_csr_and_conservation();
        std::cout << "[PASS] test_controllers_and_energy\n";
        return 0;
    } catch (const std::exception &err) {
        std::cerr << "[FAIL] test_controllers_and_energy: " << err.what()
                  << '\n';
        return 1;
    }
}
