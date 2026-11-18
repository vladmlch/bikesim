#include "physics_core/common.hpp"
#include "physics_core/cpu_exact_toolkit.hpp"
#include "physics_core/efc_monitors.hpp"
#include "physics_core/front_lift.hpp"
#include "physics_core/gpu_math.hpp"
#include "physics_core/gpu_toolkit.hpp"
#include "physics_core/launch_plan.hpp"
#include "physics_core/rider_intent.hpp"

#include "engine_call.hpp"
#include "model_access.hpp"
#include "rider/equality_reactions.hpp"
#include "rider/front_lift_reaction.hpp"
#include "rider/intent.hpp"
#include "rider/launch_config.hpp"
#include "rider/launch_plan.hpp"
#include "rider/perception.hpp"
#include "runtime/control.hpp"
#include "runtime/static_brake.hpp"
#include "support/fixture_test.hpp"

#if defined(__APPLE__)
#include "test_sp3_rider_monitors_metal_helper.hpp"
#endif

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <ranges>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

using bike::physics_core::BalanceMonitorConfig;
using bike::physics_core::BalanceMonitorState;
using bike::physics_core::BalanceStopCore;
using bike::physics_core::BrakeRequestCore;
using bike::physics_core::BrakeResolutionCore;
using bike::physics_core::CombinedSp3MonitorsState;
using bike::physics_core::CpuExactToolkit;
using bike::physics_core::CrashCause;
using bike::physics_core::EfcScanConfig;
using bike::physics_core::EfcScanResult;
using bike::physics_core::ErrCode;
using bike::physics_core::FrontLiftParams;
using bike::physics_core::FrontLiftState;
using bike::physics_core::GpuToolkit;
using bike::physics_core::GroundedFilterConfig;
using bike::physics_core::GroundedFilterState;
using bike::physics_core::IntentControlCore;
using bike::physics_core::IntentParams;
using bike::physics_core::IntentResolutionCore;
using bike::physics_core::IntentSignalsCore;
using bike::physics_core::LaunchCommandCore;
using bike::physics_core::LaunchContextCore;
using bike::physics_core::LaunchParams;
using bike::physics_core::LaunchPhase;
using bike::physics_core::LaunchState;
using bike::physics_core::PerceptionState;
using bike::physics_core::PerceptsCore;
using bike::physics_core::PortableTrig;
using bike::physics_core::ReactionCommandCore;
using bike::physics_core::ReactionPhase;
using bike::physics_core::RiderIntentState;
using bike::physics_core::RiderSensesCore;
using bike::physics_core::RollbackBrakeConfig;
using bike::physics_core::SeatedPostureState;
using bike::physics_core::ShiftFeedbackCore;
using bike::physics_core::ShiftRequest;
using bike::physics_core::ShiftResponse;
using bike::physics_core::Sp3PerWorldState;
using bike::physics_core::StaticBrakeMonitorState;
using bike::physics_core::Status;
using bike::physics_core::SurfaceLabel;
using bike::physics_core::SurfaceSectionCore;
using bike::physics_core::i32;
using bike::physics_core::u8;

using ModelPtr = std::unique_ptr<mjModel, void (*)(mjModel *)>;
using DataPtr = std::unique_ptr<mjData, void (*)(mjData *)>;

void delete_model(mjModel *m) {
    if (m != nullptr) {
        mj_deleteModel(m);
    }
}

void delete_data(mjData *d) {
    if (d != nullptr) {
        mj_deleteData(d);
    }
}

void require(bool cond, std::string_view msg) {
    bt::require(cond, msg);
}

void require_near(double actual, double expected, double tol, std::string_view what) {
    const double diff = std::abs(actual - expected);
    if (!(diff <= tol)) {
        std::ostringstream oss;
        oss << what << ": actual=" << actual << " expected=" << expected
            << " diff=" << diff << " tol=" << tol;
        throw std::runtime_error(oss.str());
    }
}

void require_bits_f64(double actual, double expected, std::string_view what) {
    if (std::bit_cast<std::uint64_t>(actual) != std::bit_cast<std::uint64_t>(expected)) {
        std::ostringstream oss;
        oss << what << ": 64-bit mismatch (actual=" << actual << ", expected=" << expected << ")";
        throw std::runtime_error(oss.str());
    }
}

void require_bits_f32(float actual, float expected, std::string_view what) {
    if (std::isnan(actual) && std::isnan(expected)) {
        return;
    }
    const auto ua = std::bit_cast<std::uint32_t>(actual);
    const auto ue = std::bit_cast<std::uint32_t>(expected);
    if (ua != ue) {
        std::ostringstream oss;
        oss << what << ": 32-bit mismatch (actual=0x" << std::hex << ua
            << ", expected=0x" << ue << ")";
        throw std::runtime_error(oss.str());
    }
}

struct CpuExactTrig {
    template <class Real, class TK>
    static Real atan_val(Real x) noexcept {
        return std::atan(x);
    }
    template <class Real, class TK>
    static Real atan2_val(Real y, Real x) noexcept {
        return std::atan2(y, x);
    }
    template <class Real, class TK>
    static Real sin_val(Real x) noexcept {
        return std::sin(x);
    }
    template <class Real, class TK>
    static Real cos_val(Real x) noexcept {
        return std::cos(x);
    }
};

ModelPtr load_xml_model(const std::filesystem::path &xml_path) {
    std::array<char, 1024> err{};
    mjModel *raw = mj_loadXML(xml_path.c_str(), nullptr, err.data(),
                              static_cast<int>(err.size()));
    require(raw != nullptr, "mj_loadXML(" + xml_path.string() + "): " + err.data());
    return {raw, delete_model};
}

rider::LaunchConfig make_cpu_launch_config() {
    rider::LaunchConfig cfg;
    cfg.start_cog_grade_breaks = {0.08, 0.16};
    cfg.start_cog_teeth = {33, 45, 51};
    cfg.start_pedal_angle_rad = -0.8;
    cfg.start_lean_grades = {0.0, 0.15, 0.25};
    cfg.start_lean_rad = {0.05, 0.18, 0.30};
    cfg.plan_enabled = true;
    cfg.standstill_speed_mps = 0.25;
    cfg.standstill_crank_rpm = 10.0;
    cfg.effort_rate_nm_s = 120.0;
    cfg.effort_cap_asphalt_nm = 55.0;
    cfg.effort_cap_hardpack_nm = 48.0;
    cfg.effort_cap_loose_nm = 38.0;
    cfg.effort_cap_wet_nm = 30.0;
    cfg.effort_cap_scree_nm = 26.0;
    cfg.effort_cap_grades = {0.0, 0.15, 0.25};
    cfg.effort_cap_grade_nm = {25.0, 35.0, 45.0};
    cfg.effort_rise_speed_mps = 1.5;
    cfg.release_torque_grades = {0.0, 0.20};
    cfg.release_torque_nm = {12.0, 22.0};
    cfg.brake_release_s = 0.05;
    cfg.max_hold_s = 0.20;
    cfg.upshift_crank_rpm = 85.0;
    cfg.shift_retry_s = 0.05;
    cfg.end_speed_mps = 3.2;
    cfg.end_crank_rpm = 75.0;
    cfg.end_hold_s = 0.05;
    cfg.max_launch_s = 2.0;
    cfg.handover_s = 0.05;
    return cfg;
}

template <class Real>
LaunchParams<Real> project_launch_params(const rider::LaunchConfig &c, double period_s) {
    LaunchParams<Real> p{};
    p.start_cog_count = static_cast<i32>(c.start_cog_teeth.size());
    for (std::size_t i = 0; i < c.start_cog_grade_breaks.size(); ++i) {
        std::span{p.start_cog_grade_breaks}[i] = static_cast<Real>(c.start_cog_grade_breaks[i]);
    }
    for (std::size_t i = 0; i < c.start_cog_teeth.size(); ++i) {
        std::span{p.start_cog_teeth}[i] = c.start_cog_teeth[i];
    }
    p.start_pedal_angle_rad = static_cast<Real>(c.start_pedal_angle_rad);
    p.start_lean_count = static_cast<i32>(c.start_lean_grades.size());
    for (std::size_t i = 0; i < c.start_lean_grades.size(); ++i) {
        std::span{p.start_lean_grades}[i] = static_cast<Real>(c.start_lean_grades[i]);
        std::span{p.start_lean_rad}[i] = static_cast<Real>(c.start_lean_rad[i]);
    }
    p.plan_enabled = c.plan_enabled ? 1 : 0;
    p.standstill_speed_mps = static_cast<Real>(c.standstill_speed_mps);
    p.standstill_crank_rpm = static_cast<Real>(c.standstill_crank_rpm);
    p.effort_rate_nm_s = static_cast<Real>(c.effort_rate_nm_s);
    p.effort_cap_asphalt_nm = static_cast<Real>(c.effort_cap_asphalt_nm);
    p.effort_cap_hardpack_nm = static_cast<Real>(c.effort_cap_hardpack_nm);
    p.effort_cap_loose_nm = static_cast<Real>(c.effort_cap_loose_nm);
    p.effort_cap_wet_nm = static_cast<Real>(c.effort_cap_wet_nm);
    p.effort_cap_scree_nm = static_cast<Real>(c.effort_cap_scree_nm);
    p.effort_cap_count = static_cast<i32>(c.effort_cap_grades.size());
    for (std::size_t i = 0; i < c.effort_cap_grades.size(); ++i) {
        std::span{p.effort_cap_grades}[i] = static_cast<Real>(c.effort_cap_grades[i]);
        std::span{p.effort_cap_grade_nm}[i] = static_cast<Real>(c.effort_cap_grade_nm[i]);
    }
    p.effort_rise_speed_mps = static_cast<Real>(c.effort_rise_speed_mps);
    p.release_torque_count = static_cast<i32>(c.release_torque_grades.size());
    for (std::size_t i = 0; i < c.release_torque_grades.size(); ++i) {
        std::span{p.release_torque_grades}[i] = static_cast<Real>(c.release_torque_grades[i]);
        std::span{p.release_torque_nm}[i] = static_cast<Real>(c.release_torque_nm[i]);
    }
    p.upshift_crank_rpm = static_cast<Real>(c.upshift_crank_rpm);
    p.end_speed_mps = static_cast<Real>(c.end_speed_mps);
    p.end_crank_rpm = static_cast<Real>(c.end_crank_rpm);
    p.period_s = static_cast<Real>(period_s);
    p.release_ticks = static_cast<i32>(std::llround(c.brake_release_s / period_s));
    p.max_hold_ticks = static_cast<i32>(std::llround(c.max_hold_s / period_s));
    p.end_hold_ticks = static_cast<i32>(std::llround(c.end_hold_s / period_s));
    p.max_launch_ticks = static_cast<i32>(std::llround(c.max_launch_s / period_s));
    p.handover_ticks = static_cast<i32>(std::llround(c.handover_s / period_s));
    p.retry_ticks = static_cast<i32>(std::llround(c.shift_retry_s / period_s));
    return p;
}

rider::FrontLiftReactionConfig make_cpu_lift_config() {
    rider::FrontLiftReactionConfig cfg;
    cfg.hand_window_s = 0.5;
    cfg.hand_lag_s = 0.5;
    cfg.hand_drop_ratio = 0.6;
    cfg.pitch_rate_up_rad_s = 0.35;
    cfg.pitch_rad = 0.12;
    cfg.decision_s = 0.025;
    cfg.ease_enabled = true;
    cfg.lean_enabled = true;
    cfg.rear_brake_tap_enabled = false;
    cfg.ease_fraction = 0.35;
    cfg.ease_rate_per_s = 2.0;
    cfg.lean_add_rad = 0.15;
    cfg.lean_rate_rad_s = 1.2;
    cfg.clear_s = 0.05;
    return cfg;
}

template <class Real>
FrontLiftParams<Real> project_lift_params(const rider::FrontLiftReactionConfig &c, double period_s) {
    FrontLiftParams<Real> p{};
    p.hand_drop_ratio = static_cast<Real>(c.hand_drop_ratio);
    p.pitch_rate_up_rad_s = static_cast<Real>(c.pitch_rate_up_rad_s);
    p.pitch_rad = static_cast<Real>(c.pitch_rad);
    p.ease_fraction = static_cast<Real>(c.ease_fraction);
    p.ease_rate_per_s = static_cast<Real>(c.ease_rate_per_s);
    p.lean_add_rad = static_cast<Real>(c.lean_add_rad);
    p.lean_rate_rad_s = static_cast<Real>(c.lean_rate_rad_s);
    p.period_s = static_cast<Real>(period_s);
    p.window_ticks = static_cast<i32>(std::llround(c.hand_window_s / period_s));
    p.lag_ticks = static_cast<i32>(std::llround(c.hand_lag_s / period_s));
    p.decision_ticks = static_cast<i32>(std::llround(c.decision_s / period_s));
    p.clear_ticks = static_cast<i32>(std::llround(c.clear_s / period_s));
    p.ease_enabled = c.ease_enabled ? 1 : 0;
    p.lean_enabled = c.lean_enabled ? 1 : 0;
    return p;
}

rider::IntentConfig make_cpu_intent_config(double period_s = 0.00125) {
    rider::IntentConfig cfg;
    cfg.enabled = true;
    cfg.period_s = period_s;
    cfg.reaction_delay_s = 0.15;
    cfg.target_crank_power_w = 250.0;
    cfg.max_crank_torque_nm = 60.0;
    cfg.lean_gain = 1.0;
    cfg.max_forward_lean_rad = 0.55;
    cfg.max_backward_lean_rad = 0.10;
    cfg.lean_rate_rad_s = 1.5;
    cfg.orientation_tau_s = 0.5;
    cfg.surge_power_w = 400.0;
    cfg.surge_grade = 0.20;
    cfg.surge_budget_s = 15.0;
    cfg.surge_recovery_rate = 1.0 / 3.0;
    cfg.front_load_share_target = 0.30;
    cfg.lean_trim_gain_rad_s = 0.30;
    cfg.lean_trim_limit_rad = 0.15;
    cfg.trim_dead_time_s = 0.20;
    cfg.min_elbow_flexion_deg = 15.0;
    cfg.climb_effort_cap_grades = {0.0, 0.15, 0.25};
    cfg.climb_effort_cap_grade_nm = {30.0, 42.0, 55.0};
    cfg.climb_effort_rise_speed_mps = 1.0;
    cfg.climb_effort_full_speed_mps = 3.0;
    return cfg;
}

template <class Real>
IntentParams<Real> project_intent_params(const rider::IntentConfig &c, double dt_s) {
    IntentParams<Real> p{};
    p.enabled = c.enabled ? 1 : 0;
    p.period_s = static_cast<Real>(c.period_s);
    p.period_steps = static_cast<i32>(std::llround(c.period_s / dt_s));
    p.delay_ticks = static_cast<i32>(std::llround(c.reaction_delay_s / c.period_s));
    p.dead_time_ticks = static_cast<i32>(std::llround(c.trim_dead_time_s / c.period_s));
    p.target_crank_power_w = static_cast<Real>(c.target_crank_power_w);
    p.max_crank_torque_nm = static_cast<Real>(c.max_crank_torque_nm);
    p.lean_gain = static_cast<Real>(c.lean_gain);
    p.max_forward_lean_rad = static_cast<Real>(c.max_forward_lean_rad);
    p.max_backward_lean_rad = static_cast<Real>(c.max_backward_lean_rad);
    p.lean_rate_rad_s = static_cast<Real>(c.lean_rate_rad_s);
    p.orientation_tau_s = static_cast<Real>(c.orientation_tau_s);
    p.surge_power_w = static_cast<Real>(c.surge_power_w);
    p.surge_grade = static_cast<Real>(c.surge_grade);
    p.surge_max_subticks = static_cast<i32>(
        std::llround((c.surge_budget_s / c.period_s) *
                     static_cast<double>(bike::physics_core::kSurgeSubTicks)));
    p.surge_recover_subticks = static_cast<i32>(
        std::llround(c.surge_recovery_rate *
                     static_cast<double>(bike::physics_core::kSurgeSubTicks)));
    p.front_load_share_target = static_cast<Real>(c.front_load_share_target);
    p.lean_trim_gain_rad_s = static_cast<Real>(c.lean_trim_gain_rad_s);
    p.lean_trim_limit_rad = static_cast<Real>(c.lean_trim_limit_rad);
    p.min_elbow_flexion_deg = static_cast<Real>(c.min_elbow_flexion_deg);
    p.climb_effort_cap_count = static_cast<i32>(c.climb_effort_cap_grades.size());
    for (std::size_t i = 0; i < c.climb_effort_cap_grades.size(); ++i) {
        std::span{p.climb_effort_cap_grades}[i] = static_cast<Real>(c.climb_effort_cap_grades[i]);
        std::span{p.climb_effort_cap_grade_nm}[i] = static_cast<Real>(c.climb_effort_cap_grade_nm[i]);
    }
    p.climb_effort_rise_speed_mps = static_cast<Real>(c.climb_effort_rise_speed_mps);
    p.climb_effort_full_speed_mps = static_cast<Real>(c.climb_effort_full_speed_mps);
    return p;
}

rider::SurfaceLabel to_cpu_surface(SurfaceLabel s) {
    return static_cast<rider::SurfaceLabel>(static_cast<u8>(s));
}

rider::Percepts to_cpu_percepts(const PerceptsCore<double> &p) {
    rider::Percepts out;
    out.senses.hand_press_n = p.senses.hand_press_n;
    out.senses.speed_mps = p.senses.speed_mps;
    out.senses.rear_teeth = p.senses.rear_teeth;
    out.senses.surface_front = to_cpu_surface(p.senses.surface_front);
    out.senses.surface_ahead = to_cpu_surface(p.senses.surface_ahead);
    out.senses.grade_ahead = p.senses.grade_ahead;
    out.crank_rpm = p.crank_rpm;
    out.human_crank_torque_nm = p.human_crank_torque_nm;
    out.pitch_rate_up_rad_s = p.pitch_rate_up_rad_s;
    out.pitch_rad = p.pitch_rad;
    return out;
}

void test_sizeof_and_pod_contracts() {
    std::cout << "[SP3-B] sizeof(LaunchState<float>)=" << sizeof(LaunchState<float>)
              << " sizeof(FrontLiftState<float>)=" << sizeof(FrontLiftState<float>)
              << " sizeof(PerceptionState<float>)=" << sizeof(PerceptionState<float>)
              << " sizeof(SeatedPostureState<float>)=" << sizeof(SeatedPostureState<float>)
              << " sizeof(RiderIntentState<float>)=" << sizeof(RiderIntentState<float>)
              << " sizeof(CombinedSp3MonitorsState<float>)=" << sizeof(CombinedSp3MonitorsState<float>)
              << " sizeof(Sp3PerWorldState<float>)=" << sizeof(Sp3PerWorldState<float>)
              << "\n";
    require(sizeof(Sp3PerWorldState<float>) <= 16384u,
            "SP3 combined per-world state footprint must stay <= 16 KiB/world");
}

void test_launch_plan_parity() {
    const double period_s = 0.00125;
    const rider::LaunchConfig cpu_cfg = make_cpu_launch_config();
    const LaunchParams<double> core_cfg64 = project_launch_params<double>(cpu_cfg, period_s);
    const LaunchParams<float> core_cfg32 = project_launch_params<float>(cpu_cfg, period_s);

    rider::LaunchPlan cpu(cpu_cfg, period_s);
    LaunchState<double> st64{};
    LaunchState<float> st32{};
    launch_reset<double>(&st64);
    launch_reset<float>(&st32);

    const double start_lean = rider::prepare_launch(cpu_cfg, 0.10).lean_rad;
    require(cpu.arm_start(start_lean), "cpu arm_start");
    bool armed64 = false;
    bool armed32 = false;
    ErrCode err = ErrCode::none;
    require(launch_arm_start<double, CpuExactToolkit>(&core_cfg64, start_lean, &st64, armed64, err) == Status::ok && armed64,
            "core64 arm_start");
    require(launch_arm_start<float, GpuToolkit>(&core_cfg32, static_cast<float>(start_lean), &st32, armed32, err) == Status::ok && armed32,
            "core32 arm_start");

    for (int step = 0; step < 300; ++step) {
        PerceptsCore<double> p64{};
        p64.senses.surface_front = SurfaceLabel::asphalt;
        p64.senses.surface_ahead = SurfaceLabel::asphalt;
        p64.senses.grade_ahead = 0.12;
        p64.senses.rear_teeth = (step < 140) ? 45 : 39;
        p64.senses.speed_mps = (step < 10) ? 0.0 : 0.03 * static_cast<double>(step - 10);
        p64.crank_rpm = (step < 80) ? 20.0 : 90.0;
        p64.human_crank_torque_nm = (step < 15) ? 5.0 : 25.0;

        PerceptsCore<float> p32{};
        p32.senses.surface_front = p64.senses.surface_front;
        p32.senses.surface_ahead = p64.senses.surface_ahead;
        p32.senses.rear_teeth = p64.senses.rear_teeth;
        p32.senses.grade_ahead = static_cast<float>(p64.senses.grade_ahead);
        p32.senses.speed_mps = static_cast<float>(p64.senses.speed_mps);
        p32.crank_rpm = static_cast<float>(p64.crank_rpm);
        p32.human_crank_torque_nm = static_cast<float>(p64.human_crank_torque_nm);

        const bool ext_brake = (step < 3);
        ShiftFeedbackCore fb_core{};
        rider::ShiftFeedback fb_cpu{};
        if (step == 100) {
            fb_core.response = ShiftResponse::rejected;
            fb_cpu.response = rider::ShiftResponse::rejected;
        } else if (step >= 140) {
            fb_core.response = ShiftResponse::accepted;
            fb_core.shift_count = 1;
            fb_cpu.response = rider::ShiftResponse::accepted;
            fb_cpu.shift_count = 1;
        }
        if (step >= 180 && step < 195) {
            fb_core.cut_active = 1;
            fb_cpu.cut_active = true;
        }

        const rider::LaunchCommand c_cpu = cpu.update(rider::LaunchInputs{
            .percepts = to_cpu_percepts(p64),
            .shift = fb_cpu,
            .eligible = true,
            .external_brake = ext_brake,
        });
        LaunchCommandCore<double> c64{};
        LaunchCommandCore<float> c32{};
        require(launch_update<double, CpuExactToolkit>(
                    &core_cfg64, true, &p64, fb_core, true, ext_brake, &st64, c64, err) == Status::ok,
                "launch_update 64");
        require(launch_update<float, GpuToolkit>(
                    &core_cfg32, true, &p32, fb_core, true, ext_brake, &st32, c32, err) == Status::ok,
                "launch_update 32");

        require(static_cast<u8>(c64.phase) == static_cast<u8>(c_cpu.phase), "launch64 phase");
        require(static_cast<u8>(c32.phase) == static_cast<u8>(c_cpu.phase), "launch32 phase");
        require((c64.shift_up != 0) == c_cpu.shift_up, "launch64 shift_up");
        require((c64.owns_shifting != 0) == c_cpu.owns_shifting, "launch64 owns_shifting");
        require((c64.preload_missed != 0) == c_cpu.preload_missed, "launch64 preload_missed");
        require_bits_f64(c64.brake, c_cpu.brake, "launch64 brake");
        require_bits_f64(c64.effort_nm, c_cpu.effort_nm, "launch64 effort_nm");
        require_bits_f64(c64.weight, c_cpu.weight, "launch64 weight");
        require_bits_f64(c64.lean_target_rad, c_cpu.lean_target_rad, "launch64 lean_target_rad");
        require_near(static_cast<double>(c32.effort_nm), c_cpu.effort_nm, 1e-4, "launch32 effort_nm");
        require_near(static_cast<double>(c32.brake), c_cpu.brake, 1e-6, "launch32 brake");
        require_near(static_cast<double>(c32.weight), c_cpu.weight, 1e-6, "launch32 weight");
    }

    // Invalid input transactionality check
    const LaunchState<float> snap32 = st32;
    PerceptsCore<float> bad_p{};
    bad_p.crank_rpm = std::numeric_limits<float>::quiet_NaN();
    LaunchCommandCore<float> bad_cmd{};
    require(launch_update<float, GpuToolkit>(
                &core_cfg32, true, &bad_p, ShiftFeedbackCore{}, true, false, &st32, bad_cmd, err) ==
                Status::invalid_argument,
            "launch rejects NaN percept");
    require(st32.phase_ticks == snap32.phase_ticks && st32.launch_ticks == snap32.launch_ticks,
            "launch invalid input leaves state unchanged");
}

void test_front_lift_parity_and_long_ride_2pi() {
    const double period_s = 0.00125;
    const rider::FrontLiftReactionConfig cpu_cfg = make_cpu_lift_config();
    const FrontLiftParams<double> core_cfg64 = project_lift_params<double>(cpu_cfg, period_s);
    const FrontLiftParams<float> core_cfg32 = project_lift_params<float>(cpu_cfg, period_s);

    rider::FrontLiftReaction cpu(cpu_cfg, period_s);
    const auto st64 = std::make_unique<FrontLiftState<double>>();
    const auto st32 = std::make_unique<FrontLiftState<float>>();
    front_lift_reset<double>(st64.get());
    front_lift_reset<float>(st32.get());

    ErrCode err = ErrCode::none;
    for (int step = 0; step < 1000; ++step) {
        const bool has_p = (step >= 120);
        PerceptsCore<double> p64{};
        p64.crank_rpm = 90.0;
        p64.senses.hand_press_n = (step >= 550 && step < 720) ? 10.0 : 80.0;
        p64.pitch_rate_up_rad_s = (step >= 550 && step < 720) ? 0.55 : 0.0;
        p64.pitch_rad = (step >= 560 && step < 720) ? 0.16 : 0.02;

        PerceptsCore<float> p32{};
        p32.crank_rpm = static_cast<float>(p64.crank_rpm);
        p32.senses.hand_press_n = static_cast<float>(p64.senses.hand_press_n);
        p32.pitch_rate_up_rad_s = static_cast<float>(p64.pitch_rate_up_rad_s);
        p32.pitch_rad = static_cast<float>(p64.pitch_rad);

        const rider::ReactionCommand c_cpu = cpu.update(
            has_p ? std::optional<rider::Percepts>{to_cpu_percepts(p64)} : std::nullopt);
        ReactionCommandCore<double> c64{};
        ReactionCommandCore<float> c32{};
        require(front_lift_update<double, CpuExactToolkit>(
                    &core_cfg64, has_p, &p64, st64.get(), c64, err) == Status::ok,
                "front_lift 64");
        require(front_lift_update<float, GpuToolkit>(
                    &core_cfg32, has_p, &p32, st32.get(), c32, err) == Status::ok,
                "front_lift 32");

        require(static_cast<u8>(c64.phase) == static_cast<u8>(c_cpu.phase), "lift64 phase");
        if (c_cpu.phase != rider::ReactionPhase::idle || c32.phase != ReactionPhase::releasing ||
            c32.lean_add_rad > 1e-5f || c32.effort_factor < 1.0f - 1e-5f) {
            require(static_cast<u8>(c32.phase) == static_cast<u8>(c_cpu.phase), "lift32 phase");
        }
        require((c64.predicate != 0) == c_cpu.predicate, "lift64 predicate");
        require((c32.predicate != 0) == c_cpu.predicate, "lift32 predicate");
        require_near(c64.effort_factor, c_cpu.effort_factor, 1e-14, "lift64 effort_factor");
        require_near(c64.lean_add_rad, c_cpu.lean_add_rad, 1e-14, "lift64 lean_add_rad");
        require_near(static_cast<double>(c32.effort_factor), c_cpu.effort_factor, 1e-5, "lift32 effort_factor");
        require_near(static_cast<double>(c32.lean_add_rad), c_cpu.lean_add_rad, 1e-5, "lift32 lean_add_rad");
    }

    // Long-ride 3,600 s (2,880,000 ticks) FP32 precision test: incremental dtheta keeps full accuracy.
    rider::FrontLiftReaction cpu_fresh(cpu_cfg, period_s);
    auto st_long = std::make_unique<FrontLiftState<float>>();
    front_lift_reset<float>(st_long.get());

    PerceptsCore<float> steady_p32{};
    steady_p32.crank_rpm = 90.0f;
    steady_p32.senses.hand_press_n = 80.0f;
    steady_p32.pitch_rate_up_rad_s = 0.0f;
    steady_p32.pitch_rad = 0.02f;

    PerceptsCore<double> steady_p64{};
    steady_p64.crank_rpm = 90.0;
    steady_p64.senses.hand_press_n = 80.0;
    steady_p64.pitch_rate_up_rad_s = 0.0;
    steady_p64.pitch_rad = 0.02;

    constexpr int kOneHourTicks = 2880000;
    ReactionCommandCore<float> cmd_long{};
    for (int t = 0; t < kOneHourTicks; ++t) {
        require(front_lift_update<float, GpuToolkit>(
                    &core_cfg32, true, &steady_p32, st_long.get(), cmd_long, err) == Status::ok,
                "3600s front_lift steady");
    }
    for (int t = 0; t < 1000; ++t) {
        (void)cpu_fresh.update(to_cpu_percepts(steady_p64));
    }

    const float kTwoPi32 = 6.283185307179586f;
    require(st_long->crank_angle_mod_2pi_rad >= 0.0f &&
                st_long->crank_angle_mod_2pi_rad < kTwoPi32,
            "3600s crank_angle_mod_2pi_rad stays in [0, 2pi)");

    int long_trigger_tick = -1;
    int cpu_trigger_tick = -1;
    for (int t = 0; t < 150; ++t) {
        PerceptsCore<float> drop_p32 = steady_p32;
        PerceptsCore<double> drop_p64 = steady_p64;
        drop_p32.senses.hand_press_n = 12.0f;
        drop_p32.pitch_rate_up_rad_s = 0.6f;
        drop_p64.senses.hand_press_n = 12.0;
        drop_p64.pitch_rate_up_rad_s = 0.6;

        require(front_lift_update<float, GpuToolkit>(
                    &core_cfg32, true, &drop_p32, st_long.get(), cmd_long, err) == Status::ok,
                "3600s front_lift drop");
        const rider::ReactionCommand c_cpu = cpu_fresh.update(to_cpu_percepts(drop_p64));
        if (long_trigger_tick < 0 && cmd_long.phase == ReactionPhase::acting) {
            long_trigger_tick = t;
        }
        if (cpu_trigger_tick < 0 && c_cpu.phase == rider::ReactionPhase::acting) {
            cpu_trigger_tick = t;
        }
    }
    require(long_trigger_tick >= 0 && long_trigger_tick == cpu_trigger_tick,
            "3600s FrontLiftReaction acting trigger tick matches fresh CPU exactly");
}

void test_rider_intent_road_and_3600s_ticks() {
    // 1. RoadProfile & SurfaceLabels parity
    const std::vector<std::array<double, 2>> verts = {
        {0.0, 0.0}, {10.0, 0.5}, {25.0, 2.75}, {45.0, 7.5}, {80.0, 11.0}
    };
    const runtime::RoadProfile cpu_road(verts);
    std::vector<double> rx64, rz64;
    std::vector<float> rx32, rz32;
    for (const auto &v : verts) {
        rx64.push_back(v[0]);
        rz64.push_back(v[1]);
        rx32.push_back(static_cast<float>(v[0]));
        rz32.push_back(static_cast<float>(v[1]));
    }

    const rider::SurfaceLabels cpu_surf(
        rider::SurfaceLabel::hardpack,
        {{.start_m = 5.0, .end_m = 18.0, .label = rider::SurfaceLabel::asphalt},
         {.start_m = 22.0, .end_m = 35.0, .label = rider::SurfaceLabel::loose}});
    const std::array<SurfaceSectionCore<double>, 2> secs64 = {{
        {.start_m = 5.0, .end_m = 18.0, .label = SurfaceLabel::asphalt},
        {.start_m = 22.0, .end_m = 35.0, .label = SurfaceLabel::loose},
    }};
    ErrCode err = ErrCode::none;
    for (const double x : {1.0, 5.0, 17.99, 18.0, 24.5, 40.0}) {
        const auto cpu_samples = cpu_road.samples(std::array<double, 2>{x, x - 1.1}, 2.0, 0.05);
        double pgrade64 = 0.0;
        double pvgrade64 = 0.0;
        require(bike::physics_core::eval_road_grades<double, CpuExactToolkit>(
                    static_cast<i32>(rx64.size()), rx64.data(), rz64.data(),
                    x, x - 1.1, 2.0, 0.05, pgrade64, pvgrade64, err) == Status::ok,
                "eval_road_grades 64");
        require_near(pgrade64, runtime::road_grade_for_posture(cpu_samples), 1e-14,
                     "road_grade_for_posture");
        require_near(pvgrade64, runtime::road_grade_preview(cpu_samples), 1e-14,
                     "road_grade_preview");

        SurfaceLabel sl = SurfaceLabel::hardpack;
        require(bike::physics_core::surface_labels_at<double>(
                    SurfaceLabel::hardpack, 2, secs64.data(), x, sl, err) == Status::ok,
                "surface_labels_at");
        require(static_cast<u8>(sl) == static_cast<u8>(cpu_surf.at(x)), "surface label parity");
    }

    // 2. RiderIntent 500-tick parity (<double, CpuExactToolkit, CpuExactTrig> vs rider::RiderIntent)
    const double dt_s = 0.00125;
    const rider::IntentConfig cpu_icfg = make_cpu_intent_config(dt_s);
    const rider::FrontLiftReactionConfig cpu_fcfg = make_cpu_lift_config();
    const rider::LaunchConfig cpu_lcfg = make_cpu_launch_config();

    const IntentParams<double> icfg64 = project_intent_params<double>(cpu_icfg, dt_s);
    const FrontLiftParams<double> fcfg64 = project_lift_params<double>(cpu_fcfg, dt_s);
    const LaunchParams<double> lcfg64 = project_launch_params<double>(cpu_lcfg, dt_s);

    rider::RiderIntent cpu_intent(cpu_icfg, cpu_fcfg, cpu_lcfg, dt_s);
    auto st64 = std::make_unique<RiderIntentState<double>>();
    rider_intent_reset<double>(&icfg64, st64.get());

    const double seed = 0.12;
    cpu_intent.seed_lean(seed);
    require(bike::physics_core::rider_intent_seed_lean<double, CpuExactToolkit>(
                &icfg64, seed, st64.get(), err) == Status::ok,
            "seed_lean");
    require(cpu_intent.arm_standstill_start(seed), "cpu arm_standstill_start");
    bool armed = false;
    require(bike::physics_core::rider_intent_arm_standstill_start<double, CpuExactToolkit>(
                &icfg64, &lcfg64, seed, st64.get(), armed, err) == Status::ok && armed,
            "core arm_standstill_start");

    cpu_intent.schedule_pulse(0.10, 0.20, 0.04);
    require(bike::physics_core::posture_program_schedule_pulse<double, CpuExactToolkit>(
                &st64->posture, 0.10, 0.20, 0.04, err) == Status::ok,
            "schedule_pulse");

    for (int step = 0; step < 500; ++step) {
        const double grade = (step > 250) ? 0.22 : 0.10;
        rider::IntentSignals cpu_sig;
        cpu_sig.pitch_rate_up_rad_s = (step > 300 && step < 360) ? 0.50 : 0.01;
        cpu_sig.specific_force_body_mps2 = {0.5, 0.0, 9.75};
        cpu_sig.crank_rate_rad_s = 7.5;
        cpu_sig.human_crank_torque_nm = (step < 130) ? 25.0 : 18.0;
        cpu_sig.front_load_share = (step % 7 == 0) ? std::nullopt : std::optional<double>{0.24};

        rider::RiderSenses cpu_sns;
        cpu_sns.hand_press_n = (step >= 300 && step < 360) ? 10.0 : 85.0;
        cpu_sns.speed_mps = 0.02 * static_cast<double>(step);
        cpu_sns.rear_teeth = 33;
        cpu_sns.surface_front = rider::SurfaceLabel::asphalt;
        cpu_sns.surface_ahead = rider::SurfaceLabel::asphalt;
        cpu_sns.grade_ahead = grade;

        IntentSignalsCore<double> sig64{};
        sig64.pitch_rate_up_rad_s = cpu_sig.pitch_rate_up_rad_s;
        std::span{sig64.specific_force_body_mps2}[0] = cpu_sig.specific_force_body_mps2[0];
        std::span{sig64.specific_force_body_mps2}[1] = cpu_sig.specific_force_body_mps2[1];
        std::span{sig64.specific_force_body_mps2}[2] = cpu_sig.specific_force_body_mps2[2];
        sig64.crank_rate_rad_s = cpu_sig.crank_rate_rad_s;
        sig64.human_crank_torque_nm = cpu_sig.human_crank_torque_nm;
        sig64.has_front_load_share = cpu_sig.front_load_share.has_value() ? 1 : 0;
        sig64.front_load_share = cpu_sig.front_load_share.value_or(0.0);

        RiderSensesCore<double> sns64{};
        sns64.hand_press_n = cpu_sns.hand_press_n;
        sns64.speed_mps = cpu_sns.speed_mps;
        sns64.rear_teeth = cpu_sns.rear_teeth;
        sns64.surface_front = SurfaceLabel::asphalt;
        sns64.surface_ahead = SurfaceLabel::asphalt;
        sns64.grade_ahead = grade;

        const rider::LaunchContext cpu_ctx{
            .shift = {.response = rider::ShiftResponse::none, .shift_count = 0, .cut_active = false},
            .effort_mode = true,
            .external_brake = false,
        };
        const LaunchContextCore ctx64{
            .shift = {.shift_count = 0, .response = ShiftResponse::none, .cut_active = 0},
            .effort_mode = 1,
            .external_brake = 0,
        };

        const auto cpu_res = cpu_intent.resolve_full(
            rider::IntentControl{}, cpu_sig, cpu_sns, step, grade, grade,
            std::nullopt, true, true, std::nullopt, cpu_ctx);

        IntentControlCore<double> ctrl64{};
        ctrl64.rider_enabled = 1;
        IntentResolutionCore<double> res64{};
        require(bike::physics_core::rider_intent_resolve_with_trig<double, CpuExactToolkit, CpuExactTrig>(
                    &icfg64, &fcfg64, &lcfg64, ctrl64, sig64, sns64, step,
                    grade, true, grade, false, 0.0, true, false, 0.0,
                    ctx64, st64.get(), res64, err) == Status::ok,
                "rider_intent_resolve 64");

        require(res64.control.has_posture != 0 && cpu_res.control.posture.has_value(), "posture present");
        require(res64.control.has_human_torque_nm != 0 && cpu_res.control.human_torque_nm.has_value(),
                "human_torque present");
        if (!cpu_res.control.posture || !cpu_res.control.human_torque_nm) {
            throw std::runtime_error("missing posture or human_torque");
        }
        require((res64.effort_limited != 0) == cpu_res.effort_limited, "effort_limited parity");
        require_near(res64.control.posture.torso_lean_rad,
                     cpu_res.control.posture->torso_lean_rad, 1e-12, "torso_lean_rad parity");
        require_near(res64.control.human_torque_nm,
                     *cpu_res.control.human_torque_nm, 1e-11, "human_torque_nm parity");
        require_near(st64->climb.inclination_rad,
                     cpu_intent.policy().inclination_rad(), 1e-12, "inclination_rad parity");
        require_bits_f64(st64->commands.front_brake, cpu_intent.commands().front_brake, "front_brake parity");
        require((st64->commands.shift_request == ShiftRequest::up) ==
                    (cpu_intent.commands().shift_request == rider::ShiftRequest::up),
                "shift_request parity");
    }

    // 3. 3,600 s (2,880,000 ticks) int32 tick drift test in FP32
    const IntentParams<float> icfg32 = project_intent_params<float>(cpu_icfg, dt_s);
    const FrontLiftParams<float> fcfg32 = project_lift_params<float>(cpu_fcfg, dt_s);
    const LaunchParams<float> lcfg32 = project_launch_params<float>(cpu_lcfg, dt_s);
    auto st32 = std::make_unique<RiderIntentState<float>>();
    rider_intent_reset<float>(&icfg32, st32.get());

    IntentControlCore<float> ctrl32{};
    ctrl32.rider_enabled = 1;
    IntentSignalsCore<float> sig32{};
    std::span{sig32.specific_force_body_mps2}[2] = 9.81f;
    sig32.crank_rate_rad_s = 7.5f;
    sig32.human_crank_torque_nm = 20.0f;
    sig32.has_front_load_share = 1;
    sig32.front_load_share = 0.30f;

    RiderSensesCore<float> sns32{};
    sns32.hand_press_n = 80.0f;
    sns32.speed_mps = 4.0f;
    sns32.rear_teeth = 33;
    sns32.grade_ahead = 0.05f;

    const LaunchContextCore ctx32{};
    constexpr i32 kThreeSixHundredSecTicks = 2880000;
    for (i32 s = 0; s < kThreeSixHundredSecTicks; ++s) {
        // Test surge spend/recover exactness on periodic 60s windows
        const bool spend_window = (s >= 2820000 && s < 2832000); // exact 15s = 12,000 ticks
        const float grade = spend_window ? 0.22f : 0.05f;
        sns32.grade_ahead = grade;
        IntentResolutionCore<float> res32{};
        require(rider_intent_resolve<float, GpuToolkit>(
                    &icfg32, &fcfg32, &lcfg32, ctrl32, sig32, sns32, s,
                    grade, true, grade, false, 0.0f, true, false, 0.0f,
                    ctx32, st32.get(), res32, err) == Status::ok,
                "3600s rider_intent_resolve");
        if (s == 2831999) {
            require(st32->climb.surge_subticks == 0,
                    "3600s surge budget hits exact 0 after 12,000 spend ticks (0 false +1 drift)");
        }
    }
    require(st32->perception.tick == kThreeSixHundredSecTicks, "3600s perception int32 tick exact");
    require(st32->posture.tick == kThreeSixHundredSecTicks, "3600s posture int32 tick exact");
    // After 48,000 recovery ticks (2832000..2879999) at 1/3 rate, recovered 16,000 ticks -> clamped to 12,000 max
    require(st32->climb.surge_subticks == icfg32.surge_max_subticks,
            "3600s surge recovery reaches exact cap with 0 drift");
}

void test_monitors_and_rollback_latch_parity() {
    const double dt_s = 0.00125;
    ErrCode err = ErrCode::none;

    // 1. GroundedFilter parity + 3,600 s int32 test
    runtime::GroundedFilter cpu_gf(0.005);
    const GroundedFilterConfig gf_cfg{.hold_ticks = 4};
    GroundedFilterState gf_st{};
    grounded_filter_reset(&gf_st);

    for (i32 s = 0; s < 20; ++s) {
        const bool raw = (s == 10);
        const bool c = cpu_gf.update(raw, static_cast<double>(s) * dt_s);
        bool g = false;
        require(grounded_filter_update(&gf_cfg, raw, s, &gf_st, g, err) == Status::ok,
                "grounded_filter_update");
        require(g == c, "GroundedFilter parity");
    }
    // Note: at s=2 -> s=6, FP64 (6*0.00125 - 2*0.00125 = 0.00499999999999999965 < 0.005)
    // exhibits the exact +1 tick float-subtraction drift that int32 unloaded_ticks eliminates.
    // At t = 3,600 s, drop raw_grounded=false after 1 grounded tick and verify exact 3 true hold ticks then false
    grounded_filter_reset(&gf_st);
    bool gval = false;
    require(grounded_filter_update(&gf_cfg, true, 2880000, &gf_st, gval, err) == Status::ok && gval,
            "3600s grounded true");
    for (i32 k = 1; k <= 3; ++k) {
        require(grounded_filter_update(&gf_cfg, false, 2880000 + k, &gf_st, gval, err) == Status::ok && gval,
                "3600s grounded hold tick");
    }
    require(grounded_filter_update(&gf_cfg, false, 2880004, &gf_st, gval, err) == Status::ok && !gval,
            "3600s grounded releases at exact 4th tick (0 drift)");

    // 2. BalanceMonitor parity (including braked full-stop grace re-arm vs rolling pause)
    runtime::BalanceMonitor cpu_bal(4.0 / 3.6, 0.5, 3.0);
    const BalanceMonitorConfig<double> bcfg64{
        .floor_mps = 4.0 / 3.6,
        .dwell_ticks = 400,
        .grace_ticks = 2400,
        .period_s = dt_s,
    };
    BalanceMonitorState<double> bst64{};
    balance_monitor_reset<double>(&bcfg64, &bst64);

    for (i32 s = 0; s < 3500; ++s) {
        const bool riding = !(s >= 100 && s < 150) && !(s >= 2900 && s < 2920);
        const auto cpu_stop = (s >= 2900 && s < 2920)
                                  ? runtime::BalanceStop::braked
                                  : runtime::BalanceStop::none;
        const auto core_stop = (s >= 2900 && s < 2920)
                                   ? BalanceStopCore::braked
                                   : BalanceStopCore::none;
        const double speed = 0.5;
        const double pos = 0.001 * static_cast<double>(s);
        const auto c_ev = cpu_bal.update(static_cast<double>(s) * dt_s, pos, speed, riding, cpu_stop);
        bool lost = false;
        require(balance_monitor_update<double, CpuExactToolkit>(
                    &bcfg64, s, pos, speed, riding, core_stop, &bst64, lost, err) == Status::ok,
                "balance_monitor_update");
        require(lost == c_ev.has_value(), "BalanceMonitor active parity");
        require(bst64.triggered == (cpu_bal.event.has_value() ? 1 : 0), "BalanceMonitor triggered parity");
    }

    // 3. StaticBrake rollback latch hysteresis & resolve_brakes_core
    const RollbackBrakeConfig<float> rcfg{
        .rollback_demand = 0.6f,
        .rollback_engage_mps = 0.25f,
        .rollback_release_mps = 0.10f,
        .rollback_brake_enabled = 1,
        .effort_drive_mode = 1,
    };
    StaticBrakeMonitorState<float> sbst{};
    float dem = 0.0f;
    require(rollback_brake_demand<float, GpuToolkit>(&rcfg, -0.20f, true, &sbst, dem, err) == Status::ok && dem == 0.0f,
            "rollback below engage");
    require(rollback_brake_demand<float, GpuToolkit>(&rcfg, -0.30f, true, &sbst, dem, err) == Status::ok && dem == 0.6f,
            "rollback engages");
    require(rollback_brake_demand<float, GpuToolkit>(&rcfg, -0.15f, true, &sbst, dem, err) == Status::ok && dem == 0.6f,
            "rollback stays latched in hysteresis band");
    require(rollback_brake_demand<float, GpuToolkit>(&rcfg, -0.05f, true, &sbst, dem, err) == Status::ok && dem == 0.0f,
            "rollback releases above -release_mps");
}

EfcScanConfig<double> build_welded_efc_config(const mjModel *m, double pitch_limit_deg = 60.0) {
    EfcScanConfig<double> cfg{};
    cfg.grip_left_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_grip_left");
    cfg.grip_right_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_grip_right");
    cfg.saddle_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_saddle");
    cfg.foot_front_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_foot_front");
    cfg.foot_rear_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_foot_rear");
    cfg.frame_body_id = mj_name2id(m, mjOBJ_BODY, "frame");
    const int crank_jnt = mj_name2id(m, mjOBJ_JOINT, "crank_spin");
    const int front_jnt = mj_name2id(m, mjOBJ_JOINT, "front_wheel_spin");
    const int rear_jnt = mj_name2id(m, mjOBJ_JOINT, "rear_wheel_spin");
    const auto dofadr = model_access::readonly_buffer(m->jnt_dofadr, m->njnt, "jnt_dofadr");
    cfg.crank_dof = dofadr[static_cast<std::size_t>(crank_jnt)];
    cfg.front_wheel_dof = dofadr[static_cast<std::size_t>(front_jnt)];
    cfg.rear_wheel_dof = dofadr[static_cast<std::size_t>(rear_jnt)];
    cfg.freehub_tendon_id = mj_name2id(m, mjOBJ_TENDON, "ideal_mid_drive_freehub");
    cfg.crank_clutch_tendon_id = mj_name2id(m, mjOBJ_TENDON, "crank_clutch");
    cfg.handlebar_geom_id = mj_name2id(m, mjOBJ_GEOM, "geom_handlebar");
    cfg.catch_geom_id = mj_name2id(m, mjOBJ_GEOM, "catch_plane");
    const int terrain_id = mj_name2id(m, mjOBJ_GEOM, "terrain");
    cfg.terrain_geom_count = 0;
    if (terrain_id >= 0) {
        std::span{cfg.terrain_geom_ids}[static_cast<std::size_t>(cfg.terrain_geom_count++)] = terrain_id;
    }
    if (cfg.catch_geom_id >= 0) {
        std::span{cfg.terrain_geom_ids}[static_cast<std::size_t>(cfg.terrain_geom_count++)] = cfg.catch_geom_id;
    }
    cfg.rider_geom_count = 0;
    for (int g = 0; g < m->ngeom; ++g) {
        const char *name = mj_id2name(m, mjOBJ_GEOM, g);
        if (name != nullptr && std::string_view(name).starts_with("geom_rider_")) {
            require(cfg.rider_geom_count < bike::physics_core::kMaxRiderCrashGeoms,
                    "rider crash geom capacity");
            std::span{cfg.rider_geom_ids}[static_cast<std::size_t>(cfg.rider_geom_count++)] = g;
        }
    }
    cfg.load_threshold_n = 1.0;
    cfg.pitch_limit_rad = pitch_limit_deg * (std::numbers::pi / 180.0);
    return cfg;
}

template <class Real>
EfcScanConfig<Real> cast_efc_config(const EfcScanConfig<double> &c) {
    EfcScanConfig<Real> out{};
    out.grip_left_eq_id = c.grip_left_eq_id;
    out.grip_right_eq_id = c.grip_right_eq_id;
    out.saddle_eq_id = c.saddle_eq_id;
    out.foot_front_eq_id = c.foot_front_eq_id;
    out.foot_rear_eq_id = c.foot_rear_eq_id;
    out.frame_body_id = c.frame_body_id;
    out.crank_dof = c.crank_dof;
    out.front_wheel_dof = c.front_wheel_dof;
    out.rear_wheel_dof = c.rear_wheel_dof;
    out.freehub_tendon_id = c.freehub_tendon_id;
    out.crank_clutch_tendon_id = c.crank_clutch_tendon_id;
    out.handlebar_geom_id = c.handlebar_geom_id;
    out.catch_geom_id = c.catch_geom_id;
    out.terrain_geom_count = c.terrain_geom_count;
    for (int i = 0; i < c.terrain_geom_count; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        std::span{out.terrain_geom_ids}[idx] = std::span{c.terrain_geom_ids}[idx];
    }
    out.rider_geom_count = c.rider_geom_count;
    for (int i = 0; i < c.rider_geom_count; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        std::span{out.rider_geom_ids}[idx] = std::span{c.rider_geom_ids}[idx];
    }
    out.load_threshold_n = static_cast<Real>(c.load_threshold_n);
    out.pitch_limit_rad = static_cast<Real>(c.pitch_limit_rad);
    return out;
}

struct WeldedEfcSnapshot {
    EfcScanConfig<float> cfg32{};
    i32 nefc = 0;
    i32 stride = 0;
    i32 nbody = 0;
    i32 ncon = 0;
    std::vector<i32> etype;
    std::vector<i32> eid;
    std::vector<float> ej;
    std::vector<float> ef;
    std::vector<float> xm;
    std::vector<i32> cg1;
    std::vector<i32> cg2;
    std::vector<i32> cadr;
    std::vector<i32> cdim;
    EfcScanResult<float> host_scan{};
};

void prepare_welded_loaded_state(const mjModel *m, mjData *d) {
    const int hf_id = mj_name2id(m, mjOBJ_HFIELD, "road");
    const int t_geom = mj_name2id(m, mjOBJ_GEOM, "terrain");
    require(hf_id >= 0 && t_geom >= 0, "welded.xml road hfield & terrain geom");
    const auto hf_idx = static_cast<std::size_t>(hf_id);
    const auto hf_nrow = model_access::readonly_buffer(m->hfield_nrow, m->nhfield, "hfield_nrow")[hf_idx];
    const auto hf_ncol = model_access::readonly_buffer(m->hfield_ncol, m->nhfield, "hfield_ncol")[hf_idx];
    const auto hf_adr = model_access::readonly_buffer(m->hfield_adr, m->nhfield, "hfield_adr")[hf_idx];
    const auto hf_size = model_access::readonly_buffer(m->hfield_size, 4 * m->nhfield, "hfield_size");
    const double t_z = model_access::readonly_buffer(
        m->geom_pos, 3 * m->ngeom, "geom_pos")[static_cast<std::size_t>(3) * static_cast<std::size_t>(t_geom) + 2u];
    const double z_norm = std::clamp(
        -t_z / hf_size[static_cast<std::size_t>(4) * hf_idx + 2u], 0.0, 1.0);
    const std::span<float> hf_data = model_access::mutable_buffer(
        m->hfield_data, hf_adr + hf_nrow * hf_ncol, "hfield_data");
    for (int i = 0; i < hf_nrow * hf_ncol; ++i) {
        hf_data[static_cast<std::size_t>(hf_adr) + static_cast<std::size_t>(i)] = static_cast<float>(z_norm);
    }

    mj_resetData(m, d);
    const auto rx = static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "root_x"));
    const auto rz = static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "root_z"));
    const auto rrx = static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "rider_root_x"));
    const auto rrz = static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "rider_root_z"));
    const auto c_jnt = static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "crank_spin"));
    const auto qadr = model_access::readonly_buffer(m->jnt_qposadr, m->njnt, "jnt_qposadr");
    const auto dadr = model_access::readonly_buffer(m->jnt_dofadr, m->njnt, "jnt_dofadr");

    const std::span<double> qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
    const std::span<double> qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
    qpos[static_cast<std::size_t>(qadr[rx])] = 2.0;
    qpos[static_cast<std::size_t>(qadr[rrx])] = 2.0;
    qpos[static_cast<std::size_t>(qadr[rz])] = -0.02;
    qpos[static_cast<std::size_t>(qadr[rrz])] = -0.02;
    qvel[static_cast<std::size_t>(dadr[rx])] = 3.0;
    qvel[static_cast<std::size_t>(dadr[rrx])] = 3.0;
    qvel[static_cast<std::size_t>(dadr[c_jnt])] = 6.0;

    const auto fw_jnt = static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "front_wheel_spin"));
    const auto rw_jnt = static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "rear_wheel_spin"));
    const int fw_dof = dadr[fw_jnt];
    const int rw_dof = dadr[rw_jnt];
    const int c_dof = dadr[c_jnt];
    const std::span<double> fl = model_access::mutable_buffer(m->dof_frictionloss, m->nv, "dof_frictionloss");
    fl[static_cast<std::size_t>(fw_dof)] = 25.0;
    fl[static_cast<std::size_t>(rw_dof)] = 25.0;

    const std::span<double> qfrc_app = model_access::mutable_buffer(d->qfrc_applied, m->nv, "qfrc_applied");
    qfrc_app[static_cast<std::size_t>(c_dof)] = 15.0;
    const std::span<double> ctrl = model_access::mutable_buffer(d->ctrl, m->nu, "ctrl");
    if (m->nu >= 12) {
        ctrl[3] = 20.0;
        ctrl[8] = 35.0;
        ctrl[9] = -15.0;
        ctrl[10] = 25.0;
        ctrl[11] = -10.0;
    }

    for (int s = 0; s < 5; ++s) {
        mj_step(m, d);
    }
    mj_forward(m, d);
}

WeldedEfcSnapshot test_welded_xml_post_solve_efc_parity() {
    WeldedEfcSnapshot snap{};
    const std::filesystem::path xml_path = bt::fixture("model/welded.xml");
    const ModelPtr m = load_xml_model(xml_path);
    const DataPtr d(mj_makeData(m.get()), delete_data);
    require(d != nullptr, "mj_makeData");
    require(!mj_isSparse(m.get()), "welded.xml nv=27 uses dense efc_J");

    prepare_welded_loaded_state(m.get(), d.get());
    require(d->nefc > 0, "welded.xml active constraints after step");

    const EfcScanConfig<double> cfg64 = build_welded_efc_config(m.get());
    const EfcScanConfig<float> cfg32 = cast_efc_config<float>(cfg64);
    snap.cfg32 = cfg32;

    // CPU reference checks via EqualityReactions, StaticBrake, RuntimeContactQuery
    const rider::EqualityReactions cpu_eq(m.get());
    const std::array<int, 2> pedal_eqs{cfg64.foot_front_eq_id, cfg64.foot_rear_eq_id};
    const double cpu_crank_tq = cpu_eq.equalities_qfrc_at(d.get(), pedal_eqs, cfg64.crank_dof);
    const std::vector<double> cpu_gl = cpu_eq.force_on_rider(d.get(), cfg64.grip_left_eq_id);
    const std::vector<double> cpu_gr = cpu_eq.force_on_rider(d.get(), cfg64.grip_right_eq_id);
    const std::vector<double> cpu_sd = cpu_eq.force_on_rider(d.get(), cfg64.saddle_eq_id);

    const auto xmat_span = model_access::readonly_buffer(d->xmat, 9 * m->nbody, "xmat");
    const std::size_t fb = 9u * static_cast<std::size_t>(cfg64.frame_body_id);
    const std::array<double, 3> frame_up{xmat_span[fb + 2], xmat_span[fb + 5], xmat_span[fb + 8]};
    const auto dot_up = [&](const std::vector<double> &v) {
        return v[0] * frame_up[0] + v[1] * frame_up[1] + v[2] * frame_up[2];
    };
    const double cpu_hand_press = dot_up(cpu_gl) + dot_up(cpu_gr);
    const double cpu_saddle_force = dot_up(cpu_sd);

    const runtime::StaticBrake cpu_brake(cfg64.front_wheel_dof, cfg64.rear_wheel_dof, 100.0);
    const auto [cpu_fb_vec, cpu_rb_vec] = cpu_brake.solved_components(m.get(), d.get());
    const double cpu_fb_tq = cpu_fb_vec[static_cast<std::size_t>(cfg64.front_wheel_dof)];
    const double cpu_rb_tq = cpu_rb_vec[static_cast<std::size_t>(cfg64.rear_wheel_dof)];

    const runtime::RuntimeContactQuery cpu_cq(m.get(), 0.005, 0.0, m->opt.timestep);
    const double cpu_bar_load = cpu_cq.handlebar_load(m.get(), d.get());
    const auto cpu_crash_geoms = runtime::crash_geom_ids(m.get());
    const auto cpu_crash = runtime::physical_contact_crash(
        m.get(), d.get(), cpu_crash_geoms.catch_ids,
        cpu_crash_geoms.terrain_ids, cpu_crash_geoms.rider_ids, 1.0);

    const auto nefc = static_cast<i32>(d->nefc);
    const auto nv = static_cast<i32>(m->nv);
    const auto nbody = static_cast<i32>(m->nbody);
    const auto ncon = static_cast<i32>(d->ncon);

    const auto et_span = model_access::readonly_buffer(d->efc_type, nefc, "efc_type");
    const auto eid_span = model_access::readonly_buffer(d->efc_id, nefc, "efc_id");
    const auto ej_span = model_access::readonly_buffer(
        d->efc_J, static_cast<mjtSize>(nefc) * static_cast<mjtSize>(nv), "efc_J");
    const auto ef_span = model_access::readonly_buffer(d->efc_force, nefc, "efc_force");
    const auto con_span = model_access::readonly_buffer(d->contact, ncon, "contact");

    std::vector<i32> etype(et_span.begin(), et_span.end());
    std::vector<i32> eid(eid_span.begin(), eid_span.end());
    std::vector<float> ej32(ej_span.size());
    std::vector<float> ef32(ef_span.size());
    std::vector<float> xm32(xmat_span.size());
    for (std::size_t i = 0; i < ej_span.size(); ++i) {
        ej32[i] = static_cast<float>(ej_span[i]);
    }
    for (std::size_t i = 0; i < ef_span.size(); ++i) {
        ef32[i] = static_cast<float>(ef_span[i]);
    }
    for (std::size_t i = 0; i < xmat_span.size(); ++i) {
        xm32[i] = static_cast<float>(xmat_span[i]);
    }

    std::vector<i32> cg1(static_cast<std::size_t>(ncon));
    std::vector<i32> cg2(static_cast<std::size_t>(ncon));
    std::vector<i32> cadr(static_cast<std::size_t>(ncon));
    std::vector<i32> cdim(static_cast<std::size_t>(ncon));
    for (int i = 0; i < ncon; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        cg1[idx] = con_span[idx].geom1;
        cg2[idx] = con_span[idx].geom2;
        cadr[idx] = con_span[idx].efc_address;
        cdim[idx] = con_span[idx].dim;
    }

    ErrCode err = ErrCode::none;
    EfcScanResult<double> scan64{};
    require(bike::physics_core::scan_efc_reactions<double, CpuExactToolkit>(
                &cfg64, nefc, nv, nbody, etype.data(), eid.data(),
                ej_span.data(), ef_span.data(), xmat_span.data(), scan64, err) == Status::ok,
            "scan_efc_reactions 64");
    require(bike::physics_core::scan_contact_monitors<double, CpuExactToolkit>(
                &cfg64, ncon, nefc, cg1.data(), cg2.data(), cadr.data(), cdim.data(),
                etype.data(), eid.data(), ef_span.data(), scan64, err) == Status::ok,
            "scan_contact_monitors 64");

    EfcScanResult<float> scan32{};
    require(bike::physics_core::scan_efc_reactions<float, GpuToolkit>(
                &cfg32, nefc, nv, nbody, etype.data(), eid.data(),
                ej32.data(), ef32.data(), xm32.data(), scan32, err) == Status::ok,
            "scan_efc_reactions 32");
    require(bike::physics_core::scan_contact_monitors<float, GpuToolkit>(
                &cfg32, ncon, nefc, cg1.data(), cg2.data(), cadr.data(), cdim.data(),
                etype.data(), eid.data(), ef32.data(), scan32, err) == Status::ok,
            "scan_contact_monitors 32");

    std::cout << "[SP3-B] welded.xml post-solve parity: nefc=" << nefc << " ncon=" << ncon
              << " crank_tq(cpu=" << cpu_crank_tq << ", f64=" << scan64.delivered_crank_torque_nm
              << ", f32=" << scan32.delivered_crank_torque_nm << ")"
              << " hand_press=" << scan64.hand_press_n
              << " saddle=" << scan64.pelvis_saddle_force_n << "\n";

    require_near(scan64.delivered_crank_torque_nm, cpu_crank_tq, 1e-12,
                 "delivered_crank_torque_nm f64 vs CPU");
    require_near(static_cast<double>(scan32.delivered_crank_torque_nm), cpu_crank_tq, 5.0e-4,
                 "delivered_crank_torque_nm f32 <= 5.0e-4 Nm vs CPU");
    require_near(scan64.hand_press_n, cpu_hand_press, 1e-12, "hand_press_n f64 vs CPU");
    require_near(static_cast<double>(scan32.hand_press_n), cpu_hand_press, 5.0e-4, "hand_press_n f32 vs CPU");
    require_near(scan64.pelvis_saddle_force_n, cpu_saddle_force, 1e-12, "pelvis_saddle_force_n f64 vs CPU");
    require_near(static_cast<double>(scan32.pelvis_saddle_force_n), cpu_saddle_force, 5.0e-4,
                 "pelvis_saddle_force_n f32 vs CPU");
    require_near(scan64.front_brake_friction_torque_nm, cpu_fb_tq, 1e-12,
                 "front_brake_friction_torque_nm f64 vs CPU");
    require_near(scan64.rear_brake_friction_torque_nm, cpu_rb_tq, 1e-12,
                 "rear_brake_friction_torque_nm f64 vs CPU");
    require_near(scan64.handlebar_load_n, cpu_bar_load, 1e-12, "handlebar_load_n f64 vs CPU");
    require((scan64.contact_crash_cause == CrashCause::none) == !cpu_crash.has_value(),
            "loaded ride has no contact crash");

    // Also verify a handlebar/rider contact crash state on welded.xml!
    const auto rz_jnt = static_cast<std::size_t>(mj_name2id(m.get(), mjOBJ_JOINT, "root_z"));
    const auto rp_jnt = static_cast<std::size_t>(mj_name2id(m.get(), mjOBJ_JOINT, "root_pitch"));
    const auto rrz_jnt = static_cast<std::size_t>(mj_name2id(m.get(), mjOBJ_JOINT, "rider_root_z"));
    const auto qadr = model_access::readonly_buffer(m->jnt_qposadr, m->njnt, "jnt_qposadr");
    const std::span<double> qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
    qpos[static_cast<std::size_t>(qadr[rz_jnt])] = -0.85;
    qpos[static_cast<std::size_t>(qadr[rrz_jnt])] = -0.85;
    qpos[static_cast<std::size_t>(qadr[rp_jnt])] = 1.4; // pitched over nose-down
    mj_step(m.get(), d.get());
    mj_forward(m.get(), d.get());
    require(d->ncon > 0 && d->nefc > 0, "crash pose must produce active contacts");
    {
        const auto c_ncon = static_cast<i32>(d->ncon);
        const auto c_nefc = static_cast<i32>(d->nefc);
        const auto c_et = model_access::readonly_buffer(d->efc_type, d->nefc, "efc_type");
        const auto c_eid = model_access::readonly_buffer(d->efc_id, d->nefc, "efc_id");
        const auto c_ef = model_access::readonly_buffer(d->efc_force, d->nefc, "efc_force");
        const auto c_con = model_access::readonly_buffer(d->contact, d->ncon, "contact");
        std::vector<i32> cg1_c(static_cast<std::size_t>(c_ncon));
        std::vector<i32> cg2_c(static_cast<std::size_t>(c_ncon));
        std::vector<i32> cadr_c(static_cast<std::size_t>(c_ncon));
        std::vector<i32> cdim_c(static_cast<std::size_t>(c_ncon));
        for (i32 i = 0; i < c_ncon; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            cg1_c[idx] = c_con[idx].geom1;
            cg2_c[idx] = c_con[idx].geom2;
            cadr_c[idx] = c_con[idx].efc_address;
            cdim_c[idx] = c_con[idx].dim;
        }
        const double bar_cpu_c = cpu_cq.handlebar_load(m.get(), d.get());
        const auto crash_cpu_c = runtime::physical_contact_crash(
            m.get(), d.get(), cpu_crash_geoms.catch_ids,
            cpu_crash_geoms.terrain_ids, cpu_crash_geoms.rider_ids, 1.0);
        EfcScanResult<double> scan_c{};
        require(bike::physics_core::scan_contact_monitors<double, CpuExactToolkit>(
                    &cfg64, c_ncon, c_nefc, cg1_c.data(), cg2_c.data(),
                    cadr_c.data(), cdim_c.data(), c_et.data(), c_eid.data(),
                    c_ef.data(), scan_c, err) == Status::ok,
                "scan_contact_monitors crash state");
        std::cout << "[SP3-B] welded.xml crash contact parity: ncon=" << c_ncon
                  << " bar_cpu=" << bar_cpu_c << " bar_scan=" << scan_c.handlebar_load_n
                  << " cause=" << static_cast<int>(scan_c.contact_crash_cause) << "\n";
        require_near(scan_c.handlebar_load_n, bar_cpu_c, 1e-10,
                     "handlebar_load_n pyramidal cone parity");
        require(crash_cpu_c.has_value(), "crash pose triggers physical_contact_crash");
        if (!crash_cpu_c) {
            throw std::runtime_error("missing physical_contact_crash");
        }
        const CrashCause expected_cause =
            (*crash_cpu_c == "catch_plane_contact")
                ? CrashCause::catch_plane_contact
                : CrashCause::rider_ground_contact;
        require(scan_c.contact_crash_cause == expected_cause,
                "contact_crash_cause parity");
    }

    snap.etype = std::move(etype);
    snap.eid = std::move(eid);
    snap.ej = std::move(ej32);
    snap.ef = std::move(ef32);
    snap.xm = std::move(xm32);
    snap.cg1 = std::move(cg1);
    snap.cg2 = std::move(cg2);
    snap.cadr = std::move(cadr);
    snap.cdim = std::move(cdim);
    snap.nefc = nefc;
    snap.stride = nv;
    snap.nbody = nbody;
    snap.ncon = ncon;
    snap.host_scan = scan32;
    return snap;
}

void test_metal_parity(const WeldedEfcSnapshot &snap) {
#if defined(__APPLE__)
    bike::physics_core::test_sp3_metal::MetalSp3Request req;
    req.physics_core_dir = BIKE_PHYSICS_CORE_SRC_DIR;

    // 1. Trig inputs
    req.trig_x = {
        0.0f, -0.0f, 1.0e-7f, -1.0e-7f, 0.25f, -0.25f, 0.4142f, 1.0f, -1.0f,
        2.5f, -2.5f, std::numbers::pi_v<float>, -std::numbers::pi_v<float>,
        2.0f * std::numbers::pi_v<float>,
        std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::quiet_NaN()
    };
    req.trig_y = {
        0.0f, 1.0f, -1.0f, 0.5f, -0.5f, 0.0f, 1.0f, 1.0f, -1.0f,
        2.0f, -2.0f, 0.0f, -0.0f, 3.0f,
        std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::quiet_NaN()
    };

    // 2. LaunchPlan sequence
    const double period_s = 0.00125;
    req.launch_cfg = project_launch_params<float>(make_cpu_launch_config(), period_s);
    for (int i = 0; i < 200; ++i) {
        req.launch_has_percepts.push_back(1);
        PerceptsCore<float> p{};
        p.senses.surface_front = SurfaceLabel::asphalt;
        p.senses.surface_ahead = SurfaceLabel::asphalt;
        p.senses.grade_ahead = 0.12f;
        p.senses.rear_teeth = (i < 110) ? 45 : 39;
        p.senses.speed_mps = (i < 10) ? 0.0f : 0.04f * static_cast<float>(i - 10);
        p.crank_rpm = (i < 70) ? 20.0f : 90.0f;
        p.human_crank_torque_nm = (i < 12) ? 5.0f : 25.0f;
        req.launch_percepts.push_back(p);
        ShiftFeedbackCore fb{};
        if (i == 90) {
            fb.response = ShiftResponse::rejected;
        } else if (i >= 110) {
            fb.response = ShiftResponse::accepted;
            fb.shift_count = 1;
        }
        req.launch_shift.push_back(fb);
        req.launch_eligible.push_back(1);
        req.launch_ext_brake.push_back(i < 2 ? 1 : 0);
    }

    // 3. FrontLift sequence (500 ticks)
    req.lift_cfg = project_lift_params<float>(make_cpu_lift_config(), period_s);
    for (int i = 0; i < 500; ++i) {
        req.lift_has_percepts.push_back(i >= 10 ? 1 : 0);
        PerceptsCore<float> p{};
        p.crank_rpm = 90.0f;
        p.senses.hand_press_n = (i >= 430 && i < 470) ? 10.0f : 80.0f;
        p.pitch_rate_up_rad_s = (i >= 430 && i < 470) ? 0.55f : 0.0f;
        p.pitch_rad = (i >= 435 && i < 470) ? 0.15f : 0.02f;
        req.lift_percepts.push_back(p);
    }

    // 4. RiderIntent sequence (300 ticks)
    req.intent_cfg = project_intent_params<float>(make_cpu_intent_config(period_s), period_s);
    req.intent_lift_cfg = req.lift_cfg;
    req.intent_launch_cfg = req.launch_cfg;
    req.road_x = {0.0f, 10.0f, 25.0f, 50.0f};
    req.road_z = {0.0f, 1.0f, 4.3f, 9.8f};
    req.surface_sections = {{.start_m = 5.0f, .end_m = 20.0f, .label = SurfaceLabel::asphalt, .pad = {0, 0, 0}}};
    for (int i = 0; i < 300; ++i) {
        IntentSignalsCore<float> s{};
        s.pitch_rate_up_rad_s = (i > 180 && i < 230) ? 0.45f : 0.01f;
        std::span{s.specific_force_body_mps2}[0] = 0.4f;
        std::span{s.specific_force_body_mps2}[2] = 9.75f;
        s.crank_rate_rad_s = 7.5f;
        s.human_crank_torque_nm = 22.0f;
        s.has_front_load_share = (i % 5 == 0) ? 0 : 1;
        s.front_load_share = 0.25f;
        req.intent_signals.push_back(s);
        req.wheel_front_x.push_back(2.0f + 0.05f * static_cast<float>(i));
        req.wheel_rear_x.push_back(0.9f + 0.05f * static_cast<float>(i));
        req.speed_mps.push_back(0.02f * static_cast<float>(i));
        req.rear_teeth.push_back(33);
        req.hand_press_n.push_back((i >= 200 && i < 250) ? 10.0f : 85.0f);
        LaunchContextCore ctx{};
        ctx.effort_mode = 1;
        req.intent_launch_ctx.push_back(ctx);
    }

    // 5. EFC + monitors
    req.efc_cfg = snap.cfg32;
    req.nefc = snap.nefc;
    req.efc_J_stride = snap.stride;
    req.nbody = snap.nbody;
    req.ncon = snap.ncon;
    req.efc_type = snap.etype;
    req.efc_id = snap.eid;
    req.efc_J = snap.ej;
    req.efc_force = snap.ef;
    req.body_xmat = snap.xm;
    req.contact_geom1 = snap.cg1;
    req.contact_geom2 = snap.cg2;
    req.contact_efc_address = snap.cadr;
    req.contact_dim = snap.cdim;
    req.grounded_cfg = GroundedFilterConfig{.hold_ticks = 4};
    req.balance_cfg = BalanceMonitorConfig<float>{
        .floor_mps = 1.1111112f,
        .dwell_ticks = 10,
        .grace_ticks = 20,
        .period_s = 0.00125f,
    };
    req.rollback_cfg = RollbackBrakeConfig<float>{
        .rollback_demand = 0.6f,
        .rollback_engage_mps = 0.25f,
        .rollback_release_mps = 0.10f,
        .rollback_brake_enabled = 1,
        .effort_drive_mode = 1,
    };
    for (int i = 0; i < 60; ++i) {
        req.mon_raw_grounded.push_back((i < 5 || (i >= 15 && i < 20)) ? 1 : 0);
        req.mon_speed_mps.push_back(i < 10 ? -0.30f : (i < 20 ? -0.15f : 0.40f));
        req.mon_pos_m.push_back(0.01f * static_cast<float>(i));
        req.mon_caller_braking.push_back(i < 3 ? 1 : 0);
        req.mon_rider_enabled.push_back(1);
    }

    const auto metal_ptr = bike::physics_core::test_sp3_metal::run_metal_sp3_parity(req);
    const auto &metal = *metal_ptr;
    require(metal.device_available, "Metal device must be available on macOS");
    require(metal.error_message.empty(), "Metal SP3 error: " + metal.error_message);
    std::cout << "[SP3-B] Metal device: " << metal.device_name
              << " (MSL 3.0 & 3.1 torture + MTLMathModeSafe OK)\n";

    // Verify 0-bit parity on Trig
    for (std::size_t i = 0; i < req.trig_x.size(); ++i) {
        const float h_atan = PortableTrig::atan_val<float, GpuToolkit>(req.trig_x[i]);
        const float h_atan2 = PortableTrig::atan2_val<float, GpuToolkit>(req.trig_y[i], req.trig_x[i]);
        const float h_sin = PortableTrig::sin_val<float, GpuToolkit>(req.trig_x[i]);
        const float h_cos = PortableTrig::cos_val<float, GpuToolkit>(req.trig_x[i]);
        require_bits_f32(metal.atan_out[i], h_atan, "Metal atan 0-bit");
        require_bits_f32(metal.atan2_out[i], h_atan2, "Metal atan2 0-bit");
        require_bits_f32(metal.sin_out[i], h_sin, "Metal sin 0-bit");
        require_bits_f32(metal.cos_out[i], h_cos, "Metal cos 0-bit");
    }

    // Verify 0-bit parity on LaunchPlan
    {
        LaunchState<float> host_ls{};
        launch_reset<float>(&host_ls);
        bool armed = false;
        ErrCode err = ErrCode::none;
        (void)launch_arm_start<float, GpuToolkit>(
            &req.launch_cfg,
            bike::physics_core::launch_start_lean_unchecked<float, GpuToolkit>(&req.launch_cfg, 0.0f),
            &host_ls, armed, err);
        for (std::size_t i = 0; i < req.launch_has_percepts.size(); ++i) {
            LaunchCommandCore<float> hcmd{};
            (void)launch_update<float, GpuToolkit>(
                &req.launch_cfg, req.launch_has_percepts[i] != 0u, &req.launch_percepts[i],
                req.launch_shift[i], req.launch_eligible[i] != 0u,
                req.launch_ext_brake[i] != 0u, &host_ls, hcmd, err);
            require(static_cast<u8>(metal.launch_out[i].phase) == static_cast<u8>(hcmd.phase),
                    "Metal launch phase");
            require_bits_f32(metal.launch_out[i].effort_nm, hcmd.effort_nm, "Metal launch effort_nm 0-bit");
            require_bits_f32(metal.launch_out[i].brake, hcmd.brake, "Metal launch brake 0-bit");
            require_bits_f32(metal.launch_out[i].weight, hcmd.weight, "Metal launch weight 0-bit");
            require_bits_f32(metal.launch_out[i].lean_target_rad, hcmd.lean_target_rad, "Metal launch lean 0-bit");
        }
    }

    // Verify 0-bit parity on FrontLiftReaction
    {
        const auto host_fs = std::make_unique<FrontLiftState<float>>();
        front_lift_reset<float>(host_fs.get());
        ErrCode err = ErrCode::none;
        for (std::size_t i = 0; i < req.lift_has_percepts.size(); ++i) {
            ReactionCommandCore<float> hcmd{};
            (void)front_lift_update<float, GpuToolkit>(
                &req.lift_cfg, req.lift_has_percepts[i] != 0u,
                &req.lift_percepts[i], host_fs.get(), hcmd, err);
            require(static_cast<u8>(metal.lift_out[i].phase) == static_cast<u8>(hcmd.phase),
                    "Metal lift phase");
            require_bits_f32(metal.lift_out[i].effort_factor, hcmd.effort_factor,
                             "Metal lift effort_factor 0-bit");
            require_bits_f32(metal.lift_out[i].lean_add_rad, hcmd.lean_add_rad,
                             "Metal lift lean_add_rad 0-bit");
        }
    }

    // Verify 0-bit parity on RiderIntent + RoadProfile + SurfaceLabels
    {
        auto host_is = std::make_unique<RiderIntentState<float>>();
        rider_intent_reset<float>(&req.intent_cfg, host_is.get());
        ErrCode err = ErrCode::none;
        bool armed = false;
        (void)rider_intent_arm_standstill_start<float, GpuToolkit>(
            &req.intent_cfg, &req.intent_launch_cfg,
            bike::physics_core::launch_start_lean_unchecked<float, GpuToolkit>(
                &req.intent_launch_cfg, 0.0f),
            host_is.get(), armed, err);
        for (std::size_t i = 0; i < req.intent_signals.size(); ++i) {
            float p_grade = 0.0f;
            float prev_grade = 0.0f;
            (void)bike::physics_core::eval_road_grades<float, GpuToolkit>(
                static_cast<i32>(req.road_x.size()), req.road_x.data(), req.road_z.data(),
                req.wheel_front_x[i], req.wheel_rear_x[i],
                req.road_lookahead_m, req.road_spacing_m, p_grade, prev_grade, err);
            require_bits_f32(metal.road_posture_grade[i], p_grade, "Metal road_posture_grade 0-bit");
            require_bits_f32(metal.road_preview_grade[i], prev_grade, "Metal road_preview_grade 0-bit");

            SurfaceLabel s_front = req.default_surface;
            SurfaceLabel s_ahead = req.default_surface;
            (void)bike::physics_core::surface_labels_at<float>(
                req.default_surface, static_cast<i32>(req.surface_sections.size()),
                req.surface_sections.data(), req.wheel_front_x[i], s_front, err);
            (void)bike::physics_core::surface_labels_at<float>(
                req.default_surface, static_cast<i32>(req.surface_sections.size()),
                req.surface_sections.data(), req.wheel_front_x[i] + req.road_lookahead_m,
                s_ahead, err);

            RiderSensesCore<float> sns{};
            sns.hand_press_n = req.hand_press_n[i];
            sns.speed_mps = req.speed_mps[i];
            sns.rear_teeth = req.rear_teeth[i];
            sns.surface_front = s_front;
            sns.surface_ahead = s_ahead;
            sns.grade_ahead = prev_grade;

            IntentControlCore<float> ctrl{};
            ctrl.rider_enabled = 1;
            IntentResolutionCore<float> hres{};
            (void)rider_intent_resolve<float, GpuToolkit>(
                &req.intent_cfg, &req.intent_lift_cfg, &req.intent_launch_cfg,
                ctrl, req.intent_signals[i], sns, static_cast<i32>(i),
                p_grade, true, prev_grade, false, 0.0f, true, false, 0.0f,
                req.intent_launch_ctx[i], host_is.get(), hres, err);

            require_bits_f32(metal.intent_out[i].control.posture.torso_lean_rad,
                             hres.control.posture.torso_lean_rad, "Metal torso_lean_rad 0-bit");
            require_bits_f32(metal.intent_out[i].control.human_torque_nm,
                             hres.control.human_torque_nm, "Metal human_torque_nm 0-bit");
            require(metal.intent_out[i].effort_limited == hres.effort_limited,
                    "Metal effort_limited");
        }
        require_bits_f32(metal.intent_final_state.climb.inclination_rad,
                         host_is->climb.inclination_rad, "Metal final inclination_rad 0-bit");
    }

    // Verify 0-bit parity on EFC scan & monitors
    {
        require_bits_f32(metal.efc_out.hand_press_n, snap.host_scan.hand_press_n,
                         "Metal hand_press_n 0-bit");
        require_bits_f32(metal.efc_out.pelvis_saddle_force_n, snap.host_scan.pelvis_saddle_force_n,
                         "Metal pelvis_saddle_force_n 0-bit");
        require_bits_f32(metal.efc_out.delivered_crank_torque_nm,
                         snap.host_scan.delivered_crank_torque_nm,
                         "Metal delivered_crank_torque_nm 0-bit");
        require_bits_f32(metal.efc_out.front_brake_friction_torque_nm,
                         snap.host_scan.front_brake_friction_torque_nm,
                         "Metal front_brake_friction_torque_nm 0-bit");
        require_bits_f32(metal.efc_out.rear_brake_friction_torque_nm,
                         snap.host_scan.rear_brake_friction_torque_nm,
                         "Metal rear_brake_friction_torque_nm 0-bit");
        require_bits_f32(metal.efc_out.handlebar_load_n, snap.host_scan.handlebar_load_n,
                         "Metal handlebar_load_n 0-bit");

        GroundedFilterState host_gf{};
        grounded_filter_reset(&host_gf);
        BalanceMonitorState<float> host_bm{};
        balance_monitor_reset<float>(&req.balance_cfg, &host_bm);
        StaticBrakeMonitorState<float> host_sb{};
        ErrCode err = ErrCode::none;
        for (std::size_t i = 0; i < req.mon_raw_grounded.size(); ++i) {
            bool gval = false;
            (void)grounded_filter_update(
                &req.grounded_cfg, req.mon_raw_grounded[i] != 0u,
                static_cast<i32>(i), &host_gf, gval, err);
            require(metal.mon_grounded_out[i] == (gval ? 1u : 0u), "Metal grounded parity");

            float rb = 0.0f;
            (void)rollback_brake_demand<float, GpuToolkit>(
                &req.rollback_cfg, req.mon_speed_mps[i],
                req.mon_rider_enabled[i] != 0u, &host_sb, rb, err);
            require_bits_f32(metal.mon_rollback_demand_out[i], rb, "Metal rollback demand 0-bit");

            BrakeRequestCore<float> breq{};
            breq.caller_front = req.mon_caller_braking[i] != 0u ? 0.5f : 0.0f;
            breq.rollback_demand = rb;
            breq.caller_braking = req.mon_caller_braking[i];
            breq.rider_present = 1;
            breq.rider_enabled = req.mon_rider_enabled[i];
            const BrakeResolutionCore<float> bres =
                resolve_brakes_core<float, GpuToolkit>(breq);

            const BalanceStopCore stop = balance_compute_stop<float, GpuToolkit>(
                bres.motor_braking != 0u, req.mon_speed_mps[i], 0.25f,
                LaunchPhase::done, &host_bm);
            bool blost = false;
            (void)balance_monitor_update<float, GpuToolkit>(
                &req.balance_cfg, static_cast<i32>(i), req.mon_pos_m[i],
                req.mon_speed_mps[i], bres.riding != 0u, stop, &host_bm, blost, err);
            require(metal.mon_balance_lost_out[i] == (blost ? 1u : 0u), "Metal balance parity");
        }
        require(metal.mon_balance_final.grace_ticks_left == host_bm.grace_ticks_left &&
                    metal.mon_balance_final.low_speed_ticks == host_bm.low_speed_ticks,
                "Metal balance final ticks 0-diff");
    }
#else
    (void)snap;
#endif
}

} // namespace

int main() {
    try {
        test_sizeof_and_pod_contracts();
        test_launch_plan_parity();
        test_front_lift_parity_and_long_ride_2pi();
        test_rider_intent_road_and_3600s_ticks();
        test_monitors_and_rollback_latch_parity();

        const WeldedEfcSnapshot snap = test_welded_xml_post_solve_efc_parity();
        test_metal_parity(snap);
        std::cout << "[SP3-B] ALL TESTS PASSED\n";
        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "[SP3-B] FAILED: " << ex.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "[SP3-B] FAILED: unknown exception\n";
        return 1;
    }
}
