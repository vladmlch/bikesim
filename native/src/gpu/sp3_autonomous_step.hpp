// native/src/gpu/sp3_autonomous_step.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// autonomous SP3 closed-loop step orchestrator: init, pre_solve (road/intent/brakes/
// shifting+12-tendon cassette+dual-ratchet/pedaling/assist/suspension/tyres/resistance/rider),
// and post_solve (efc+contact scans/monitors/IMU/2*pi epoch re-basing).
//
// CRITICAL MSL < 3.2 & stack rules:
// - Never copy GpuSp3WorldState (<= 16 KiB) onto the GPU `thread` stack; mutate in-place via StatePtr.
// - Zero `double` literals (use `f` / `BIKE_REAL`), zero `std::` inside BIKE_GPU_SHARED_CODE_BEGIN..END,
//   zero `if constexpr`, zero member methods on POD structs passed through device/constant pointers.
#pragma once

#include "../physics_core/assist.hpp"
#include "../physics_core/cruise.hpp"
#include "../physics_core/efc_monitors.hpp"
#include "../physics_core/energy_ledger.hpp"
#include "../physics_core/front_lift.hpp"
#include "../physics_core/launch_plan.hpp"
#include "../physics_core/pedaling.hpp"
#include "../physics_core/ride_program.hpp"
#include "../physics_core/rider_intent.hpp"
#include "../physics_core/shifting.hpp"
#include "batch_step.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: MSL has no std::span/std::array and address-space-qualified raw pointers are the device ABI. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::gpu {

struct GpuSp3Config {
    GpuSuspensionParams suspension{};
    float tire_stiff_scale = 1.0f;
    bike::physics_core::ResistanceParams<float> resistance{};
    GpuRiderParams rider{};
    float static_brake_ceiling_nm = 200.0f;
    bike::physics_core::BrakeHoldParams<float> brake_hold{
        .torque_nm = 200.0f,
        .stiffness_nm_rad = 20000.0f,
        .damping_nms_rad = 10.0f,
        .engage_radps = 0.5f,
        .release_radps = 1.0f,
    };
    float front_max_brake_torque_nm = 420.0f;
    float rear_max_brake_torque_nm = 380.0f;
    float brake_taper_rad_s = 0.5f;

    bike::physics_core::ShiftingParams<float> shifting{};
    bike::physics_core::PedalingParams<float> pedaling{};
    bike::physics_core::AssistParams<float> assist{};
    bike::physics_core::IntentParams<float> intent{};
    bike::physics_core::FrontLiftParams<float> front_lift{};
    bike::physics_core::LaunchParams<float> launch{};
    bike::physics_core::RollbackBrakeConfig<float> rollback{};
    bike::physics_core::GroundedFilterConfig grounded_filter{};
    bike::physics_core::BridgedLoadConfig<float> bridged_load{};
    bike::physics_core::BalanceMonitorConfig<float> balance{};
    bike::physics_core::EfcScanConfig<float> efc_scan{};

    float road_lookahead_m = 2.0f;
    float road_sample_spacing_m = 0.05f;
    bike::physics_core::SurfaceSectionCore<float> surface_sections[bike::physics_core::kMaxSurfaceSections]{};
    bike::physics_core::i32 surface_section_count = 0;
    bike::physics_core::SurfaceLabel default_surface_label = bike::physics_core::SurfaceLabel::hardpack;
    bike::physics_core::u8 pad0[3]{};

    bike::physics_core::i32 motor_actuator_idx = 2;
    float motor_ctrl_min_nm = 0.0f;
    float motor_ctrl_max_nm = 85.0f;
    bike::physics_core::i32 accel_sensor_adr = -1;
    bike::physics_core::i32 gyro_sensor_adr = -1;
    float crank_torque_ripple = 0.5f;
    float caller_front_brake = 0.0f;
    float caller_rear_brake = 0.0f;
    float caller_motor_torque_nm = -1.0f;
    float caller_motor_limit_nm = -1.0f;
    float caller_human_torque_nm = -1.0f;
    float lean_limit_rad = 0.55f;
    float min_lean_rad = 0.0f;
    bike::physics_core::u8 has_lean_limit = 0;
    bike::physics_core::u8 has_min_lean = 0;
    bike::physics_core::u8 effort_mode = 1;
    bike::physics_core::u8 rider_present = 1;
    bike::physics_core::u8 caller_braking = 0;
    bike::physics_core::u8 apply_direct_crank_torque = 1;
    bike::physics_core::u8 cut_after_ceiling = 1;
    bike::physics_core::u8 seed_lean_from_launch = 1;
    bike::physics_core::u8 arm_standstill_start = 0;
    bike::physics_core::u8 use_bridged_load = 0;
    bike::physics_core::u8 reposition_request = 0;
    bike::physics_core::u8 pad1 = 0;

    bike::physics_core::CruiseConfigPod<float> cruise{};
    bike::physics_core::GpuRideProgram<float> ride_program{};
    bike::physics_core::i32 control_period_steps = 1;
    bike::physics_core::u8 cruise_enabled = 0;
    bike::physics_core::u8 program_enabled = 0;
    bike::physics_core::u8 freeze_on_crash = 0;
    bike::physics_core::u8 track_energy_ledger = 1;
};

struct GpuSp3WorldState {
    bike::physics_core::Sp3PerWorldState<float> sp3{};
    bike::physics_core::ShiftingState<float> shifting{};
    bike::physics_core::PedalingState<float> pedaling{};
    bike::physics_core::AssistState<float> assist{};
    bike::physics_core::RatchetBoundState<float> freehub{};
    bike::physics_core::RatchetBoundState<float> crank_clutch{};
    bike::physics_core::EfcScanResult<float> last_efc{};
    bike::physics_core::BrakeResolutionCore<float> last_brakes{};
    bike::physics_core::CruiseStatePod<float> cruise{};
    bike::physics_core::StallRepositionStatePod<float> stall_reposition{};
    bike::physics_core::GpuEnergyLedger<float> energy_ledger{};
    bike::physics_core::i32 step_index = 0;
    float last_wheel_load_n[2]{};
    float last_wheel_slip_mps[2]{};
    float last_wheel_eff_radius_m[2]{};
    float last_front_load_share = 0.0f;
    float last_commanded_crank_torque_nm = 0.0f;
    float last_motor_torque_nm = 0.0f;
    float last_cruise_torque_nm = 0.0f;
    float last_front_hold_nm = 0.0f;
    float last_rear_hold_nm = 0.0f;
    float last_front_frictionloss_nm = 0.0f;
    float last_rear_frictionloss_nm = 0.0f;
    float last_step_u_elastic_pre_j = 0.0f;
    float last_step_d_tyre_radial_j = 0.0f;
    float last_step_d_tyre_brush_roll_j = 0.0f;
    float last_step_d_susp_j = 0.0f;
    float last_step_d_res_j = 0.0f;
    float last_pitch_rate_up_rad_s = 0.0f;
    float last_specific_force_body_mps2[3]{0.0f, 0.0f, 9.81f};
    float cached_posture_grade = 0.0f;
    float cached_preview_grade = 0.0f;
    bike::physics_core::SurfaceLabel cached_surface_front = bike::physics_core::SurfaceLabel::hardpack;
    bike::physics_core::SurfaceLabel cached_surface_ahead = bike::physics_core::SurfaceLabel::hardpack;
    bike::physics_core::u8 has_cached_road_state = 0;
    bike::physics_core::u8 reposition_consumed = 0;
    bike::physics_core::u8 last_wheel_grounded[2]{};
    bike::physics_core::u8 last_has_front_load_share = 0;
    bike::physics_core::u8 using_cassette_tendons = 0;
    bike::physics_core::u8 frozen = 0;
    bike::physics_core::u8 pad_sp3 = 0;
};

static_assert(
    sizeof(GpuSp3WorldState) <= bike::physics_core::kMaxSp3StateBytesPerWorld,
    "SP3 per-world state must stay <= 16 KiB/world"
);

BIKE_HD void gpu_init_default_sp3_config(BIKE_THREAD GpuSp3Config &cfg) noexcept {
    cfg = GpuSp3Config{};
    cfg.suspension = gpu_make_default_suspension_params();
    cfg.resistance = gpu_make_default_resistance_params();
    cfg.rider = gpu_make_default_rider_params();
    cfg.rider.kp_posture[4] = 600.0f;
    cfg.rider.kd_posture[4] = 15.0f;
    cfg.rider.kp_posture[5] = 600.0f;
    cfg.rider.kd_posture[5] = 15.0f;
    cfg.rider.kp_posture[6] = 600.0f;
    cfg.rider.kd_posture[6] = 15.0f;
    cfg.rider.kp_posture[7] = 600.0f;
    cfg.rider.kd_posture[7] = 15.0f;
    cfg.rider.kp_posture[8] = 600.0f;
    cfg.rider.kd_posture[8] = 15.0f;

    // Shifting defaults matching viewer_physics_welded.toml
    cfg.shifting.front_teeth = 22;
    cfg.shifting.initial_rear_teeth = 24;
    cfg.shifting.cog_count = bike::physics_core::kCassetteCogCount;
    cfg.shifting.target_cadence_min_rpm = 75.0f;
    cfg.shifting.target_cadence_max_rpm = 110.0f;
    cfg.shifting.shift_cooldown_ticks = 200;
    cfg.shifting.shift_cut_duration_ticks = 160;
    cfg.shifting.torque_factor_larger_cog = 0.8f;
    cfg.shifting.torque_factor_smaller_cog = 0.3f;
    cfg.shifting.cadence_smoothing_tau_s = 0.05f;
    cfg.shifting.upshift_slip_limit_mps = 0.5f;
    cfg.shifting.upshift_slip_mode = bike::physics_core::SlipMode::legacy_signed;
    cfg.shifting.enabled = 1;

    // Pedaling defaults matching viewer_physics_welded.toml
    cfg.pedaling.coast_above_rpm = 120.0f;
    cfg.pedaling.resume_below_rpm = 105.0f;
    cfg.pedaling.stop_time_s = 0.35f;
    cfg.pedaling.coast_cadence_tau_s = 0.35f;
    cfg.pedaling.mash_cadence_rpm = 45.0f;
    cfg.pedaling.mash_torque_nm = 60.0f;
    cfg.pedaling.effort_slew_nm_s = 300.0f;
    cfg.pedaling.enabled = 1;
    cfg.pedaling.reposition_on_stall = 0;

    // Assist defaults matching Bosch CX Gen 4 in viewer_physics_welded.toml
    cfg.assist.gain = 3.4f;
    cfg.assist.max_torque = 85.0f;
    cfg.assist.max_power = 600.0f;
    cfg.assist.tau = 0.04f;
    cfg.assist.slew = 2125.0f;
    cfg.assist.engage_torque_nm = 4.0f;
    cfg.assist.release_hold_ticks = 240;
    cfg.assist.cutoff_mps = 25.0f / 3.6f;
    cfg.assist.taper_width_mps = 2.0f / 3.6f;
    cfg.assist.profile = {
        .eco = 0.6f,
        .tour = 1.4f,
        .emtb_low = 1.4f,
        .emtb_high = 3.4f,
        .turbo = 3.4f,
        .emtb_full_gain_at_nm = 40.0f,
    };
    cfg.assist.curve_rpm[0] = 0.0f;
    cfg.assist.curve_rpm[1] = 120.0f;
    cfg.assist.curve_rpm[2] = 120.1f;
    cfg.assist.curve_rpm[3] = 180.0f;
    cfg.assist.curve_nm[0] = 85.0f;
    cfg.assist.curve_nm[1] = 85.0f;
    cfg.assist.curve_nm[2] = 47.7f;
    cfg.assist.curve_nm[3] = 0.0f;
    cfg.assist.curve_len = 4;
    cfg.assist.profile_mode = bike::physics_core::ProfileMode::turbo;
    cfg.assist.has_profile = 1;

    // Seated climb / RiderIntent defaults matching viewer_physics_welded.toml
    cfg.intent.enabled = 1;
    cfg.intent.period_s = 0.00125f;
    cfg.intent.period_steps = 1;
    cfg.intent.delay_ticks = 120;
    cfg.intent.dead_time_ticks = 160;
    cfg.intent.target_crank_power_w = 250.0f;
    cfg.intent.max_crank_torque_nm = 60.0f;
    cfg.intent.lean_gain = 1.0f;
    cfg.intent.max_forward_lean_rad = 0.55f;
    cfg.intent.max_backward_lean_rad = 0.10f;
    cfg.intent.lean_rate_rad_s = 1.0f;
    cfg.intent.orientation_tau_s = 0.5f;
    cfg.intent.surge_power_w = 400.0f;
    cfg.intent.surge_grade = 0.20f;
    cfg.intent.surge_max_subticks = 12000 * bike::physics_core::kSurgeSubTicks;
    cfg.intent.surge_recover_subticks = 200;
    cfg.intent.front_load_share_target = 0.30f;
    cfg.intent.lean_trim_gain_rad_s = 0.3f;
    cfg.intent.lean_trim_limit_rad = 0.25f;
    cfg.intent.min_elbow_flexion_deg = 15.0f;
    cfg.intent.climb_effort_cap_count = 7;
    const float climb_grades[7] = {0.0f, 0.05f, 0.10f, 0.15f, 0.20f, 0.22f, 0.25f};
    const float climb_caps[7] = {45.0f, 35.0f, 30.0f, 30.0f, 34.0f, 34.0f, 35.0f};
    for (int i = 0; i < 7; ++i) {
        cfg.intent.climb_effort_cap_grades[i] = climb_grades[i];
        cfg.intent.climb_effort_cap_grade_nm[i] = climb_caps[i];
    }
    cfg.intent.climb_effort_rise_speed_mps = 2.5f;
    cfg.intent.climb_effort_full_speed_mps = 4.0f;

    // FrontLiftReaction defaults matching viewer_physics_welded.toml
    cfg.front_lift.hand_drop_ratio = 0.7f;
    cfg.front_lift.pitch_rate_up_rad_s = 0.15f;
    cfg.front_lift.pitch_rad = 0.09f;
    cfg.front_lift.ease_fraction = 0.7f;
    cfg.front_lift.ease_rate_per_s = 5.0f;
    cfg.front_lift.lean_add_rad = 0.15f;
    cfg.front_lift.lean_rate_rad_s = 1.0f;
    cfg.front_lift.period_s = 0.00125f;
    cfg.front_lift.window_ticks = 400;
    cfg.front_lift.lag_ticks = 400;
    cfg.front_lift.decision_ticks = 160;
    cfg.front_lift.clear_ticks = 320;
    cfg.front_lift.ease_enabled = 1;
    cfg.front_lift.lean_enabled = 1;

    // LaunchPlan defaults matching viewer_physics_welded.toml
    cfg.launch.start_cog_count = 5;
    const float cog_breaks[4] = {0.03f, 0.08f, 0.13f, 0.18f};
    const bike::physics_core::i32 cog_teeth[5] = {24, 33, 39, 33, 39};
    for (int i = 0; i < 4; ++i) {
        cfg.launch.start_cog_grade_breaks[i] = cog_breaks[i];
    }
    for (int i = 0; i < 5; ++i) {
        cfg.launch.start_cog_teeth[i] = cog_teeth[i];
    }
    cfg.launch.start_pedal_angle_rad = -0.8f;
    cfg.launch.start_lean_count = 2;
    cfg.launch.start_lean_grades[0] = 0.0f;
    cfg.launch.start_lean_grades[1] = 0.15f;
    cfg.launch.start_lean_rad[0] = 0.0f;
    cfg.launch.start_lean_rad[1] = 0.35f;
    cfg.launch.plan_enabled = 1;
    cfg.launch.standstill_speed_mps = 0.1f;
    cfg.launch.standstill_crank_rpm = 5.0f;
    cfg.launch.effort_rate_nm_s = 80.0f;
    cfg.launch.effort_cap_asphalt_nm = 45.0f;
    cfg.launch.effort_cap_hardpack_nm = 60.0f;
    cfg.launch.effort_cap_loose_nm = 45.0f;
    cfg.launch.effort_cap_wet_nm = 13.0f;
    cfg.launch.effort_cap_scree_nm = 40.0f;
    cfg.launch.effort_cap_count = 10;
    const float cap_grades[10] = {0.0f, 0.05f, 0.08f, 0.10f, 0.12f, 0.13f, 0.15f, 0.20f, 0.22f, 0.25f};
    const float cap_nm[10] = {36.0f, 15.0f, 14.5f, 13.2f, 14.0f, 15.5f, 19.5f, 18.5f, 21.0f, 20.0f};
    for (int i = 0; i < 10; ++i) {
        cfg.launch.effort_cap_grades[i] = cap_grades[i];
        cfg.launch.effort_cap_grade_nm[i] = cap_nm[i];
    }
    cfg.launch.effort_rise_speed_mps = 2.5f;
    cfg.launch.release_torque_count = 5;
    const float rel_grades[5] = {0.0f, 0.05f, 0.10f, 0.15f, 0.20f};
    const float rel_nm[5] = {0.0f, 9.0f, 9.0f, 9.0f, 8.0f};
    for (int i = 0; i < 5; ++i) {
        cfg.launch.release_torque_grades[i] = rel_grades[i];
        cfg.launch.release_torque_nm[i] = rel_nm[i];
    }
    cfg.launch.upshift_crank_rpm = 82.0f;
    cfg.launch.end_speed_mps = 3.0f;
    cfg.launch.end_crank_rpm = 60.0f;
    cfg.launch.period_s = 0.00125f;
    cfg.launch.release_ticks = 120;
    cfg.launch.max_hold_ticks = 1200;
    cfg.launch.end_hold_ticks = 240;
    cfg.launch.max_launch_ticks = 4800;
    cfg.launch.handover_ticks = 720;
    cfg.launch.retry_ticks = 200;

    // Rollback brake & monitors
    cfg.rollback.rollback_demand = 1.0f;
    cfg.rollback.rollback_engage_mps = 0.25f;
    cfg.rollback.rollback_release_mps = 0.0f;
    cfg.rollback.rollback_brake_enabled = 1;
    cfg.rollback.effort_drive_mode = 1;

    cfg.grounded_filter.hold_ticks = 4;
    cfg.bridged_load.dropout_steps = 10;
    cfg.bridged_load.threshold_n = 1.0f;

    cfg.balance.floor_mps = 4.0f / 3.6f;
    cfg.balance.dwell_ticks = 400;
    cfg.balance.grace_ticks = 2400;
    cfg.balance.period_s = 0.00125f;

    // Default EfcScanConfig matching tests/fixtures/model/welded.xml
    cfg.efc_scan.grip_left_eq_id = 5;
    cfg.efc_scan.grip_right_eq_id = 6;
    cfg.efc_scan.saddle_eq_id = 4;
    cfg.efc_scan.foot_front_eq_id = 2;
    cfg.efc_scan.foot_rear_eq_id = 3;
    cfg.efc_scan.frame_body_id = 1;
    cfg.efc_scan.crank_dof = 3;
    cfg.efc_scan.front_wheel_dof = 7;
    cfg.efc_scan.rear_wheel_dof = 10;
    cfg.efc_scan.freehub_tendon_id = 0;
    cfg.efc_scan.crank_clutch_tendon_id = 1;
    cfg.efc_scan.handlebar_geom_id = -1;
    cfg.efc_scan.catch_geom_id = -1;
    cfg.efc_scan.terrain_geom_count = 0;
    cfg.efc_scan.rider_geom_count = 0;
    cfg.efc_scan.load_threshold_n = 1.0f;
    cfg.efc_scan.pitch_limit_rad = 1.04719755f;
}

BIKE_HD int gpu_build_sp3_sweep_configs(
    BIKE_THREAD GpuSp3Config *configs,
    int nworld,
    const BIKE_THREAD float *fork_psi_scale,
    const BIKE_THREAD float *rebound_scale,
    const BIKE_THREAD float *shock_rate_scale,
    const BIKE_THREAD float *tire_stiff_scale
) noexcept {
    if (configs == nullptr || nworld <= 0) {
        return -1;
    }
    for (int w = 0; w < nworld; ++w) {
        BIKE_THREAD GpuSp3Config &cfg = configs[w];
        gpu_init_default_sp3_config(cfg);
        if (fork_psi_scale != nullptr) {
            cfg.suspension.params.air_spring.gauge_pressure_psi *= fork_psi_scale[w];
        }
        if (rebound_scale != nullptr) {
            const float r_scale = gpu_max(0.0f, rebound_scale[w]);
            cfg.suspension.params.fork_damper.core.c_reb_min *= r_scale;
            cfg.suspension.params.fork_damper.core.c_reb_max *= r_scale;
            cfg.suspension.params.shock_damper.core.c_reb_min *= r_scale;
            cfg.suspension.params.shock_damper.core.c_reb_max *= r_scale;

            const int fork_max_reb = cfg.suspension.params.fork_damper.core.max_reb;
            const int fork_reb_scaled = static_cast<int>(
                gpu_floor(static_cast<float>(cfg.suspension.params.fork_damper.core.rebound_clicks) * r_scale + 0.5f)
            );
            cfg.suspension.params.fork_damper.core.rebound_clicks =
                fork_reb_scaled < 0 ? 0 : (fork_reb_scaled > fork_max_reb ? fork_max_reb : fork_reb_scaled);

            const int shock_max_reb = cfg.suspension.params.shock_damper.core.max_reb;
            const int shock_reb_scaled = static_cast<int>(
                gpu_floor(static_cast<float>(cfg.suspension.params.shock_damper.core.rebound_clicks) * r_scale + 0.5f)
            );
            cfg.suspension.params.shock_damper.core.rebound_clicks =
                shock_reb_scaled < 0 ? 0 : (shock_reb_scaled > shock_max_reb ? shock_max_reb : shock_reb_scaled);
        }
        if (shock_rate_scale != nullptr) {
            cfg.suspension.params.coil.rate_n_m *= shock_rate_scale[w];
        }
        if (tire_stiff_scale != nullptr) {
            cfg.tire_stiff_scale = gpu_max(0.0f, tire_stiff_scale[w]);
        }
        gpu_init_suspension_preloads(cfg.suspension);
    }
    return 0;
}

template <class CfgPtr, class MonitorPtr>
BIKE_HD void gpu_sp3_reset_monitors(CfgPtr cfg, MonitorPtr mon) noexcept {
    bike::physics_core::grounded_filter_reset(&mon->grounded[0]);
    bike::physics_core::grounded_filter_reset(&mon->grounded[1]);
    bike::physics_core::bridged_load_reset<float>(&mon->bridged[0]);
    bike::physics_core::bridged_load_reset<float>(&mon->bridged[1]);
    bike::physics_core::balance_monitor_reset<float>(&cfg->balance, &mon->balance);
    mon->crash = bike::physics_core::CrashState<float>{};
    mon->brake = bike::physics_core::StaticBrakeMonitorState<float>{};
}

template <class CfgPtr, class ShiftStatePtr>
BIKE_HD int gpu_sp3_physical_cog_idx(CfgPtr cfg, ShiftStatePtr shifting) noexcept {
    if (cfg->shifting.shift_cut_duration_ticks > 1 &&
        shifting->from_teeth >= 3) {
        const bool off_band_downshift =
            (shifting->required_ema < 0.5f * cfg->shifting.target_cadence_min_rpm) ||
            (shifting->required_ema > 0.85f * cfg->shifting.target_cadence_max_rpm);
        const long cut_ticks = static_cast<long>(cfg->shifting.shift_cut_duration_ticks);
        const int rem_threshold = (shifting->from_teeth > shifting->rear_teeth)
            ? static_cast<int>((cut_ticks * 3L) / 8L)
            : (off_band_downshift
                ? static_cast<int>(cut_ticks / 3L)
                : static_cast<int>(cut_ticks / 6L));
        if (shifting->cut_remaining_ticks > rem_threshold) {
            const int from_cog =
                bike::physics_core::shifting_find_cog_idx(&cfg->shifting, shifting->from_teeth);
            if (from_cog >= 0 && from_cog < cfg->shifting.cog_count) {
                return from_cog;
            }
        }
    }
    return shifting->active_cog_idx;
}

template <class TendonRangePtr, class CfgPtr, class ShiftStatePtr>
BIKE_HD bike::physics_core::Status gpu_sp3_write_tendon_ranges(
    CfgPtr cfg,
    ShiftStatePtr shifting,
    const BIKE_THREAD GpuModelIndices &idx,
    int ntendon,
    float fh_bound,
    bool has_drive_shaft,
    float cc_bound,
    TendonRangePtr tendon_range_w,
    BIKE_THREAD bike::physics_core::ErrCode &err
) noexcept {
    err = bike::physics_core::ErrCode::none;
    if (tendon_range_w == nullptr || ntendon <= 0) {
        return bike::physics_core::Status::ok;
    }
    const int phys_cog = gpu_sp3_physical_cog_idx(cfg, shifting);
    if (ntendon >= cfg->shifting.cog_count && phys_cog >= 0 &&
        phys_cog < cfg->shifting.cog_count) {
        const bike::physics_core::Status fc_st =
            bike::physics_core::fill_cassette_tendon_ranges<float, bike::physics_core::GpuToolkit>(
                phys_cog,
                cfg->shifting.cog_count,
                ntendon,
                fh_bound,
                cfg->shifting.cassette_tendon_ids,
                tendon_range_w,
                err);
        if (fc_st != bike::physics_core::Status::ok) {
            return fc_st;
        }
    } else if (idx.freehub_tendon >= 0 && idx.freehub_tendon < ntendon) {
        tendon_range_w[2 * idx.freehub_tendon + 0] = -1e12f;
        tendon_range_w[2 * idx.freehub_tendon + 1] = fh_bound;
    }
    if (has_drive_shaft && idx.crank_clutch_tendon >= 0 && idx.crank_clutch_tendon < ntendon) {
        tendon_range_w[2 * idx.crank_clutch_tendon + 0] = -1e12f;
        tendon_range_w[2 * idx.crank_clutch_tendon + 1] = cc_bound;
    }
    return bike::physics_core::Status::ok;
}

template <
    class CfgPtr,
    class Sp3StatePtr,
    class TyreWorldStatePtr,
    class QposPtr,
    class TendonRangePtr>
BIKE_HD bike::physics_core::Status gpu_sp3_init_world(
    const BIKE_THREAD GpuModelIndices &idx,
    CfgPtr cfg,
    Sp3StatePtr sp3_st,
    TyreWorldStatePtr tyre_world_st,
    QposPtr qpos_w,
    float init_grade,
    int ntendon,
    TendonRangePtr tendon_range_w
) noexcept {
    if (cfg == nullptr || sp3_st == nullptr || !gpu_validate_model_indices(idx)) {
        return bike::physics_core::Status::invalid_argument;
    }
    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;

    // 1. Reset RiderIntent, monitors, shifting, pedaling, assist, cruise, energy in-place (zero large stack allocs)
    bike::physics_core::rider_intent_reset<float>(&cfg->intent, &sp3_st->sp3.intent);
    gpu_sp3_reset_monitors(cfg, &sp3_st->sp3.monitors);
    bike::physics_core::shifting_reset<float>(&cfg->shifting, &sp3_st->shifting);
    bike::physics_core::pedaling_reset<float>(&sp3_st->pedaling);
    sp3_st->stall_reposition = bike::physics_core::StallRepositionStatePod<float>{};
    bike::physics_core::assist_reset<float>(&sp3_st->assist);
    sp3_st->cruise = bike::physics_core::CruiseStatePod<float>{};
    if (cfg->cruise_enabled != 0 || cfg->program_enabled != 0) {
        const bike::physics_core::Status cr_st =
            bike::physics_core::cruise_init<float, bike::physics_core::GpuToolkit>(
                &cfg->cruise, &sp3_st->cruise, err);
        if (cr_st != bike::physics_core::Status::ok) {
            return cr_st;
        }
    }
    sp3_st->energy_ledger = bike::physics_core::GpuEnergyLedger<float>{};

    if (tyre_world_st != nullptr) {
        tyre_world_st->compliant_wheels[0] = bike::physics_core::GpuCompliantWheelState{};
        tyre_world_st->compliant_wheels[1] = bike::physics_core::GpuCompliantWheelState{};
        for (int s = 0; s < 2; ++s) {
            for (int k = 0; k < bike::physics_core::kMaxGpuTyreStations; ++k) {
                tyre_world_st->distributed_wheels[s][k] = bike::physics_core::GpuStationState{};
            }
        }
    }

    const bool has_ds =
        (idx.drive_shaft_spin_qpos >= 0 && idx.drive_shaft_spin_qpos < idx.nq);
    const float crank_q =
        (qpos_w != nullptr && idx.crank_spin_qpos >= 0 && idx.crank_spin_qpos < idx.nq)
            ? qpos_w[idx.crank_spin_qpos]
            : 0.0f;
    const float rw_q =
        (qpos_w != nullptr && idx.rear_wheel_spin_qpos >= 0 && idx.rear_wheel_spin_qpos < idx.nq)
            ? qpos_w[idx.rear_wheel_spin_qpos]
            : 0.0f;
    const float ds_q =
        (qpos_w != nullptr && has_ds) ? qpos_w[idx.drive_shaft_spin_qpos] : 0.0f;
    const float fh_driver_q = has_ds ? ds_q : crank_q;

    // Seed initial crank phase on PedalingState
    sp3_st->pedaling.target_phase_rad = crank_q;

    // Optional seed lean & arm standstill launch start
    if (cfg->intent.enabled != 0) {
        float lean0 = 0.0f;
        if (cfg->seed_lean_from_launch != 0 && cfg->launch.start_lean_count > 0) {
            lean0 = bike::physics_core::launch_start_lean_unchecked<float, bike::physics_core::GpuToolkit>(
                &cfg->launch, init_grade);
        }
        lean0 = gpu_clamp(
            lean0, -cfg->intent.max_backward_lean_rad, cfg->intent.max_forward_lean_rad);
        if (cfg->seed_lean_from_launch != 0) {
            const bike::physics_core::Status sl_st =
                bike::physics_core::rider_intent_seed_lean<float, bike::physics_core::GpuToolkit>(
                    &cfg->intent, lean0, &sp3_st->sp3.intent, err);
            if (sl_st != bike::physics_core::Status::ok) {
                return sl_st;
            }
        }
        if (cfg->arm_standstill_start != 0 && cfg->launch.plan_enabled != 0) {
            bool armed = false;
            const bike::physics_core::Status arm_st =
                bike::physics_core::rider_intent_arm_standstill_start<float, bike::physics_core::GpuToolkit>(
                    &cfg->intent, &cfg->launch, lean0, &sp3_st->sp3.intent, armed, err);
            if (arm_st != bike::physics_core::Status::ok) {
                return arm_st;
            }
        }
    }

    // 2. Dual ratchets: freehub at initial cog ratio, crank_clutch at 1.0f
    const float fh_ratio =
        bike::physics_core::shifting_gear_ratio<float>(&cfg->shifting, &sp3_st->shifting);
    bool fh_has = false;
    float fh_bound = 0.0f;
    const bike::physics_core::Status fh_st =
        bike::physics_core::ratchet_reset_bound<float, bike::physics_core::GpuToolkit>(
            fh_ratio, fh_driver_q, rw_q, fh_has, fh_bound, err);
    if (fh_st != bike::physics_core::Status::ok) {
        return fh_st;
    }
    sp3_st->freehub.ratio = fh_ratio;
    sp3_st->freehub.has_boundary = fh_has ? 1 : 0;
    sp3_st->freehub.boundary = fh_bound;

    bool cc_has = false;
    float cc_bound = 0.0f;
    if (has_ds) {
        const bike::physics_core::Status cc_st =
            bike::physics_core::ratchet_reset_bound<float, bike::physics_core::GpuToolkit>(
                1.0f, crank_q, ds_q, cc_has, cc_bound, err);
        if (cc_st != bike::physics_core::Status::ok) {
            return cc_st;
        }
        sp3_st->crank_clutch.ratio = 1.0f;
        sp3_st->crank_clutch.has_boundary = cc_has ? 1 : 0;
        sp3_st->crank_clutch.boundary = cc_bound;
    } else {
        sp3_st->crank_clutch = bike::physics_core::RatchetBoundState<float>{};
    }

    // 3. Reset scalar caches & step counter
    sp3_st->last_efc = bike::physics_core::EfcScanResult<float>{};
    sp3_st->last_brakes = bike::physics_core::BrakeResolutionCore<float>{};
    sp3_st->step_index = 0;
    sp3_st->last_wheel_load_n[0] = 0.0f;
    sp3_st->last_wheel_load_n[1] = 0.0f;
    sp3_st->last_wheel_slip_mps[0] = 0.0f;
    sp3_st->last_wheel_slip_mps[1] = 0.0f;
    sp3_st->last_wheel_eff_radius_m[0] = 0.0f;
    sp3_st->last_wheel_eff_radius_m[1] = 0.0f;
    sp3_st->last_wheel_grounded[0] = 0;
    sp3_st->last_wheel_grounded[1] = 0;
    sp3_st->last_front_load_share = 0.0f;
    sp3_st->last_has_front_load_share = 0;
    sp3_st->last_commanded_crank_torque_nm = 0.0f;
    sp3_st->last_motor_torque_nm = 0.0f;
    sp3_st->last_cruise_torque_nm = 0.0f;
    sp3_st->last_front_hold_nm = 0.0f;
    sp3_st->last_rear_hold_nm = 0.0f;
    sp3_st->last_front_frictionloss_nm = 0.0f;
    sp3_st->last_rear_frictionloss_nm = 0.0f;
    sp3_st->last_step_u_elastic_pre_j = 0.0f;
    sp3_st->last_step_d_tyre_radial_j = 0.0f;
    sp3_st->last_step_d_tyre_brush_roll_j = 0.0f;
    sp3_st->last_step_d_susp_j = 0.0f;
    sp3_st->last_step_d_res_j = 0.0f;
    sp3_st->last_pitch_rate_up_rad_s = 0.0f;
    sp3_st->last_specific_force_body_mps2[0] = 0.0f;
    sp3_st->last_specific_force_body_mps2[1] = 0.0f;
    sp3_st->last_specific_force_body_mps2[2] = 9.81f;
    sp3_st->cached_posture_grade = init_grade;
    sp3_st->cached_preview_grade = init_grade;
    sp3_st->cached_surface_front = cfg->default_surface_label;
    sp3_st->cached_surface_ahead = cfg->default_surface_label;
    sp3_st->has_cached_road_state = 0;
    sp3_st->reposition_consumed = 0;
    sp3_st->using_cassette_tendons = (ntendon >= cfg->shifting.cog_count) ? 1 : 0;
    sp3_st->frozen = 0;

    return gpu_sp3_write_tendon_ranges(
        cfg, &sp3_st->shifting, idx, ntendon, fh_bound, has_ds, cc_bound, tendon_range_w, err);
}

namespace detail {

template <class AirPtr>
BIKE_HD bike::physics_core::Status gpu_sp3_eval_fork_air_energy_j(
    AirPtr air,
    float fork_qpos_m,
    BIKE_THREAD float &out_energy_j,
    BIKE_THREAD bike::physics_core::ErrCode &err
) noexcept {
    err = bike::physics_core::ErrCode::none;
    out_energy_j = 0.0f;
    if (air == nullptr || !bike::physics_core::GpuToolkit::isfinite(fork_qpos_m)) {
        err = bike::physics_core::ErrCode::outside_domain;
        return bike::physics_core::Status::invalid_argument;
    }
    const float kPsiToPa = 6894.757293168f;
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const float kPi = 3.14159265358979323846f;
    const float limit = air->total_travel_mm / 1000.0f;
    const float inside = gpu_min(gpu_max(fork_qpos_m, 0.0f), limit);
    const float r_m = (air->stanchion_inner_diam_mm / 2.0f) / 1000.0f;
    const float area = kPi * r_m * r_m;
    const bike::physics_core::i32 tokens =
        (air->num_tokens < 0)
            ? 0
            : ((air->num_tokens > air->max_tokens) ? air->max_tokens : air->num_tokens);
    const float vp0 = area * (air->pos_chamber_length_mm / 1000.0f) -
                      static_cast<float>(tokens) * (air->token_volume_cm3 * 1e-6f);
    const float vn0 = area * (air->neg_chamber_length_mm / 1000.0f);
    const float disp = area * inside;
    const float p0 = air->gauge_pressure_psi * kPsiToPa + air->atm_pressure_pa;
    const float v_pos = vp0 - disp;
    const float v_neg = vn0 + disp;
    if (vp0 <= 0.0f || vn0 <= 0.0f || v_pos <= 0.0f || v_neg <= 0.0f ||
        p0 <= 0.0f || air->gamma <= 0.0f) {
        err = bike::physics_core::ErrCode::outside_domain;
        return bike::physics_core::Status::invalid_argument;
    }
    const float log_pos = bike::physics_core::GpuToolkit::log(v_pos / vp0);
    const float log_neg = bike::physics_core::GpuToolkit::log(v_neg / vn0);
    float energy = 0.0f;
    if (gpu_abs(air->gamma - 1.0f) < 1e-6f) {
        energy = -p0 * vp0 * log_pos - p0 * vn0 * log_neg;
    } else {
        const float one_minus_g = 1.0f - air->gamma;
        const float denom = air->gamma - 1.0f;
        energy = p0 * vp0 *
                     bike::physics_core::assist_expm1<float, bike::physics_core::GpuToolkit>(
                         one_minus_g * log_pos) /
                     denom +
                 p0 * vn0 *
                     bike::physics_core::assist_expm1<float, bike::physics_core::GpuToolkit>(
                         one_minus_g * log_neg) /
                     denom;
    }
    if (fork_qpos_m > limit) {
        float f_max = 0.0f;
        float v_disp = 0.0f;
        float v_pos0 = 0.0f;
        bike::physics_core::SuspDiagField diag = bike::physics_core::SuspDiagField::none;
        const bike::physics_core::Status af_st =
            bike::physics_core::eval_air_axial_force<float, bike::physics_core::GpuToolkit>(
                air, air->total_travel_mm, f_max, v_disp, v_pos0, err, diag);
        if (af_st != bike::physics_core::Status::ok) {
            return af_st;
        }
        energy += f_max * (fork_qpos_m - limit);
    }
    if (!bike::physics_core::GpuToolkit::isfinite(energy) || energy < -1e-4f) {
        err = bike::physics_core::ErrCode::overflow_nonfinite;
        return bike::physics_core::Status::overflow;
    }
    out_energy_j = gpu_max(0.0f, energy);
    return bike::physics_core::Status::ok;
}

template <class SuspCfgPtr>
BIKE_HD float gpu_sp3_eval_suspension_loss_w(
    SuspCfgPtr susp_cfg,
    const BIKE_THREAD bike::physics_core::SuspensionResult<float> &res,
    float fork_qvel_mps,
    float shock_qpos_m,
    float shock_qvel_mps
) noexcept {
    float loss_w = gpu_max(0.0f, res.fork_damper * fork_qvel_mps) +
                   gpu_max(0.0f, res.shock_damper * shock_qvel_mps);
    const float limit_m = susp_cfg->coil.stroke_mm / 1000.0f;
    if (shock_qpos_m < 0.0f) {
        const float low_depth = -shock_qpos_m;
        const float top_out_mag = -res.shock_top_out;
        const float elastic_f = susp_cfg->end_stops.stiffness_n_m * low_depth;
        loss_w += gpu_max(0.0f, -(top_out_mag - elastic_f) * shock_qvel_mps);
    } else if (shock_qpos_m > limit_m) {
        const float high_depth = shock_qpos_m - limit_m;
        const float upper_mag = res.shock_upper_stop;
        const float elastic_f =
            susp_cfg->coil.bumper_peak_n + susp_cfg->end_stops.stiffness_n_m * high_depth;
        loss_w += gpu_max(0.0f, (upper_mag - elastic_f) * shock_qvel_mps);
    }
    return gpu_max(0.0f, loss_w);
}

template <
    class CfgPtr,
    class Sp3StatePtr,
    class CompliantStatePtr,
    class DistStatePtr,
    class XposPtr,
    class XmatPtr,
    class XanchorPtr,
    class XaxisPtr,
    class TendonRangePtr,
    class FrictionLossPtr>
BIKE_HD bike::physics_core::Status gpu_sp3_pre_solve_core(
    int w,
    float dt_s,
    const BIKE_THREAD GpuModelIndices &idx,
    BIKE_DEVICE const bike::physics_core::GpuTyreModel *tyre_model,
    const BIKE_THREAD bike::physics_core::GpuTerrainTableDesc &terrain_desc,
    BIKE_DEVICE const float *terrain_f32,
    BIKE_DEVICE const bike::physics_core::i32 *terrain_i32,
    CfgPtr cfg,
    Sp3StatePtr sp3_st,
    CompliantStatePtr compliant_wheels_w,
    DistStatePtr dist_wheels_w,
    const BIKE_THREAD float *qpos_w,
    const BIKE_THREAD float *qvel_w,
    XposPtr xpos_w,
    XmatPtr xmat_w,
    XanchorPtr xanchor_w,
    XaxisPtr xaxis_w,
    BIKE_THREAD float *qfrc_w,
    BIKE_THREAD float *ctrl_w,
    int ntendon,
    TendonRangePtr tendon_range_w,
    FrictionLossPtr dof_frictionloss_w,
    bool has_lean_override,
    bool ov_has_lean_limit,
    float ov_lean_limit_rad,
    bool ov_has_min_lean,
    float ov_min_lean_rad,
    bool write_sp3_pd_ctrl,
    bool control_tick,
    BIKE_THREAD bike::physics_core::IntentResolutionCore<float> *out_intent_res,
    BIKE_THREAD bike::physics_core::PedalingOutput<float> *out_pedal_out
) noexcept {
    if (w < 0 || cfg == nullptr || sp3_st == nullptr || tyre_model == nullptr ||
        !gpu_validate_model_indices(idx) ||
        qpos_w == nullptr || qvel_w == nullptr ||
        qfrc_w == nullptr || ctrl_w == nullptr ||
        xpos_w == nullptr || xmat_w == nullptr ||
        xanchor_w == nullptr || xaxis_w == nullptr) {
        return bike::physics_core::Status::invalid_argument;
    }

    for (int i = 0; i < idx.nv; ++i) {
        qfrc_w[i] = 0.0f;
    }
    for (int i = 0; i < idx.nu; ++i) {
        ctrl_w[i] = 0.0f;
    }

    if (sp3_st->frozen != 0 && cfg->freeze_on_crash != 0) {
        if (out_intent_res != nullptr) {
            *out_intent_res = bike::physics_core::IntentResolutionCore<float>{};
        }
        if (out_pedal_out != nullptr) {
            *out_pedal_out = bike::physics_core::PedalingOutput<float>{};
        }
        return bike::physics_core::Status::ok;
    }

    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    const float kTwoPi = 6.28318530717958647692f;

    // 0. Optional scripted ride program evaluation
    bike::physics_core::GpuProgramOverride<float> prog_ov{};
    if (cfg->program_enabled != 0) {
        const float prog_time_s = static_cast<float>(sp3_st->step_index) * dt_s;
        const bike::physics_core::Status pr_st =
            bike::physics_core::eval_ride_program<float, bike::physics_core::GpuToolkit>(
                &cfg->ride_program, prog_time_s, prog_ov, err);
        if (pr_st != bike::physics_core::Status::ok) {
            return pr_st;
        }
    }
    const bike::physics_core::u32 ov_mask = prog_ov.channel_mask;
    const bool ov_human_tq =
        (ov_mask & static_cast<bike::physics_core::u32>(
                       bike::physics_core::GpuProgramChannelMask::has_human_torque)) != 0u;
    const bool ov_motor_tq =
        (ov_mask & static_cast<bike::physics_core::u32>(
                       bike::physics_core::GpuProgramChannelMask::has_motor_torque)) != 0u;
    const bool ov_motor_lim =
        (ov_mask & static_cast<bike::physics_core::u32>(
                       bike::physics_core::GpuProgramChannelMask::has_motor_limit)) != 0u;
    const bool ov_crank_rate =
        (ov_mask & static_cast<bike::physics_core::u32>(
                       bike::physics_core::GpuProgramChannelMask::has_crank_rate)) != 0u;
    const bool ov_front_brake =
        (ov_mask & static_cast<bike::physics_core::u32>(
                       bike::physics_core::GpuProgramChannelMask::has_front_brake)) != 0u;
    const bool ov_rear_brake =
        (ov_mask & static_cast<bike::physics_core::u32>(
                       bike::physics_core::GpuProgramChannelMask::has_rear_brake)) != 0u;
    const bool ov_cruise_kmh =
        (ov_mask & static_cast<bike::physics_core::u32>(
                       bike::physics_core::GpuProgramChannelMask::has_cruise_speed_kmh)) != 0u;
    const bool ov_posture_lean =
        (ov_mask & static_cast<bike::physics_core::u32>(
                       bike::physics_core::GpuProgramChannelMask::has_posture_lean)) != 0u;

    if (ov_cruise_kmh) {
        const bike::physics_core::Status cs_st =
            bike::physics_core::cruise_set_target_speed_kmh<float, bike::physics_core::GpuToolkit>(
                &sp3_st->cruise, prog_ov.cruise_speed_kmh, err);
        if (cs_st != bike::physics_core::Status::ok) {
            return cs_st;
        }
    }

    const float eff_caller_front_brake = ov_front_brake ? prog_ov.front_brake : cfg->caller_front_brake;
    const float eff_caller_rear_brake = ov_rear_brake ? prog_ov.rear_brake : cfg->caller_rear_brake;

    // 1. Road grades & surface labels from shared polyline terrain (updated on control_tick,
    // matching runtime/step.cpp:1248-1258).
    const float front_wheel_x =
        (idx.front_wheel_body >= 0) ? xpos_w[3 * idx.front_wheel_body + 0] : 0.0f;
    const float rear_wheel_x =
        (idx.rear_wheel_body >= 0) ? xpos_w[3 * idx.rear_wheel_body + 0] : 0.0f;

    if (control_tick || sp3_st->has_cached_road_state == 0) {
        float posture_grade = 0.0f;
        float preview_grade = 0.0f;
        if (terrain_desc.polyline_vertex_count >= 2 && terrain_f32 != nullptr &&
            terrain_desc.px_offset >= 0 && terrain_desc.pz_offset >= 0) {
            const bike::physics_core::Status rg_st =
                bike::physics_core::eval_road_grades<float, bike::physics_core::GpuToolkit>(
                    terrain_desc.polyline_vertex_count,
                    terrain_f32 + terrain_desc.px_offset,
                    terrain_f32 + terrain_desc.pz_offset,
                    front_wheel_x,
                    rear_wheel_x,
                    cfg->road_lookahead_m,
                    cfg->road_sample_spacing_m,
                    posture_grade,
                    preview_grade,
                    err);
            if (rg_st != bike::physics_core::Status::ok) {
                return rg_st;
            }
        }

        bike::physics_core::SurfaceLabel surface_front = cfg->default_surface_label;
        bike::physics_core::SurfaceLabel surface_ahead = cfg->default_surface_label;
        if (cfg->surface_section_count > 0) {
            const bike::physics_core::Status sf_st =
                bike::physics_core::surface_labels_at<float>(
                    cfg->default_surface_label,
                    cfg->surface_section_count,
                    cfg->surface_sections,
                    front_wheel_x,
                    surface_front,
                    err);
            if (sf_st != bike::physics_core::Status::ok) {
                return sf_st;
            }
            const bike::physics_core::Status sa_st =
                bike::physics_core::surface_labels_at<float>(
                    cfg->default_surface_label,
                    cfg->surface_section_count,
                    cfg->surface_sections,
                    front_wheel_x + cfg->road_lookahead_m,
                    surface_ahead,
                    err);
            if (sa_st != bike::physics_core::Status::ok) {
                return sa_st;
            }
        }
        sp3_st->cached_posture_grade = posture_grade;
        sp3_st->cached_preview_grade = preview_grade;
        sp3_st->cached_surface_front = surface_front;
        sp3_st->cached_surface_ahead = surface_ahead;
        sp3_st->has_cached_road_state = 1;
    }
    const float posture_grade = sp3_st->cached_posture_grade;
    const float preview_grade = sp3_st->cached_preview_grade;
    const bike::physics_core::SurfaceLabel surface_front = sp3_st->cached_surface_front;
    const bike::physics_core::SurfaceLabel surface_ahead = sp3_st->cached_surface_ahead;

    // 2. RiderIntent resolution using previous-step solved reactions & IMU signals
    const float speed_mps = (idx.root_x_dof >= 0) ? qvel_w[idx.root_x_dof] : 0.0f;
    const float crank_qpos = (idx.crank_spin_qpos >= 0) ? qpos_w[idx.crank_spin_qpos] : 0.0f;
    const float crank_omega = (idx.crank_spin_dof >= 0) ? qvel_w[idx.crank_spin_dof] : 0.0f;

    bike::physics_core::IntentControlCore<float> in_control{};
    in_control.rider_enabled = (cfg->rider.rider_enabled != 0) ? 1 : 0;
    if (ov_motor_tq) {
        in_control.has_motor_torque_nm = 1;
        in_control.motor_torque_nm = prog_ov.motor_torque_nm;
    } else if (cfg->caller_motor_torque_nm >= 0.0f) {
        in_control.has_motor_torque_nm = 1;
        in_control.motor_torque_nm = cfg->caller_motor_torque_nm;
    }
    if (ov_motor_lim) {
        in_control.has_motor_limit_nm = 1;
        in_control.motor_limit_nm = prog_ov.motor_limit_nm;
    } else if (cfg->caller_motor_limit_nm >= 0.0f) {
        in_control.has_motor_limit_nm = 1;
        in_control.motor_limit_nm = cfg->caller_motor_limit_nm;
    }
    if (ov_human_tq) {
        in_control.has_human_torque_nm = 1;
        in_control.human_torque_nm = prog_ov.human_torque_nm;
    } else if (cfg->caller_human_torque_nm >= 0.0f) {
        in_control.has_human_torque_nm = 1;
        in_control.human_torque_nm = cfg->caller_human_torque_nm;
    }
    if (ov_crank_rate) {
        in_control.has_crank_target_rate_rad_s = 1;
        in_control.crank_target_rate_rad_s = prog_ov.crank_rate_rad_s;
    }
    if (ov_posture_lean) {
        in_control.has_posture = 1;
        in_control.posture.torso_lean_rad = prog_ov.posture_lean_rad;
        in_control.posture.use_saddle = 1;
    }

    bike::physics_core::IntentSignalsCore<float> signals{};
    signals.pitch_rate_up_rad_s = sp3_st->last_pitch_rate_up_rad_s;
    signals.specific_force_body_mps2[0] = sp3_st->last_specific_force_body_mps2[0];
    signals.specific_force_body_mps2[1] = sp3_st->last_specific_force_body_mps2[1];
    signals.specific_force_body_mps2[2] = sp3_st->last_specific_force_body_mps2[2];
    signals.crank_rate_rad_s = crank_omega;
    signals.human_crank_torque_nm =
        (cfg->apply_direct_crank_torque != 0)
            ? sp3_st->last_commanded_crank_torque_nm
            : sp3_st->last_efc.delivered_crank_torque_nm;
    signals.has_front_load_share = sp3_st->last_has_front_load_share;
    signals.front_load_share = sp3_st->last_front_load_share;

    bike::physics_core::RiderSensesCore<float> senses{};
    senses.hand_press_n = sp3_st->last_efc.hand_press_n;
    senses.speed_mps = speed_mps;
    senses.grade_ahead = preview_grade;
    senses.rear_teeth = sp3_st->shifting.rear_teeth;
    senses.surface_front = surface_front;
    senses.surface_ahead = surface_ahead;

    bike::physics_core::LaunchContextCore launch_ctx{};
    launch_ctx.shift.shift_count = sp3_st->shifting.shift_count;
    launch_ctx.shift.response = sp3_st->shifting.rider_response;
    launch_ctx.shift.cut_active = (sp3_st->shifting.cut_remaining_ticks > 0) ? 1 : 0;
    launch_ctx.effort_mode = cfg->effort_mode;
    launch_ctx.external_brake =
        (eff_caller_front_brake > 0.0f || eff_caller_rear_brake > 0.0f || cfg->caller_braking != 0)
            ? 1
            : 0;

    const bool eff_has_lean_limit =
        has_lean_override ? ov_has_lean_limit : (cfg->has_lean_limit != 0);
    const float eff_lean_limit_rad =
        has_lean_override ? ov_lean_limit_rad : cfg->lean_limit_rad;
    const bool eff_has_min_lean =
        has_lean_override ? ov_has_min_lean : (cfg->has_min_lean != 0);
    const float eff_min_lean_rad =
        has_lean_override ? ov_min_lean_rad : cfg->min_lean_rad;

    bike::physics_core::IntentResolutionCore<float> intent_res{};
    const bike::physics_core::Status ri_st =
        bike::physics_core::rider_intent_resolve<float, bike::physics_core::GpuToolkit>(
            &cfg->intent,
            &cfg->front_lift,
            &cfg->launch,
            in_control,
            signals,
            senses,
            sp3_st->step_index,
            posture_grade,
            true,
            preview_grade,
            eff_has_lean_limit,
            eff_lean_limit_rad,
            true,
            eff_has_min_lean,
            eff_min_lean_rad,
            launch_ctx,
            &sp3_st->sp3.intent,
            intent_res,
            err);
    if (ri_st != bike::physics_core::Status::ok) {
        return ri_st;
    }

    // 3. Brakes: rollback_brake_demand -> resolve_brakes_core -> eval_brake_hold_wheel(ceiling) -> apply_static_brake(11-arg)
    float rollback_dem = 0.0f;
    const bike::physics_core::Status rb_st =
        bike::physics_core::rollback_brake_demand<float, bike::physics_core::GpuToolkit>(
            &cfg->rollback,
            speed_mps,
            intent_res.control.rider_enabled != 0,
            &sp3_st->sp3.monitors.brake,
            rollback_dem,
            err);
    if (rb_st != bike::physics_core::Status::ok) {
        return rb_st;
    }

    bike::physics_core::BrakeRequestCore<float> breq{};
    breq.caller_front = eff_caller_front_brake;
    breq.caller_rear = eff_caller_rear_brake;
    breq.rollback_demand = rollback_dem;
    breq.rider_front = (cfg->intent.enabled != 0) ? sp3_st->sp3.intent.commands.front_brake : 0.0f;
    breq.rider_rear = (cfg->intent.enabled != 0) ? sp3_st->sp3.intent.commands.rear_brake : 0.0f;
    breq.caller_braking =
        (cfg->caller_braking != 0 || eff_caller_front_brake > 0.0f || eff_caller_rear_brake > 0.0f)
            ? 1
            : 0;
    breq.rider_present = cfg->rider_present;
    breq.rider_enabled = intent_res.control.rider_enabled;

    const bike::physics_core::BrakeResolutionCore<float> brakes =
        bike::physics_core::resolve_brakes_core<float, bike::physics_core::GpuToolkit>(breq);
    sp3_st->last_brakes = brakes;

    const bool has_static_fl = (dof_frictionloss_w != nullptr);
    const float fw_qpos = (idx.front_wheel_spin_qpos >= 0) ? qpos_w[idx.front_wheel_spin_qpos] : 0.0f;
    const float fw_omega = (idx.front_wheel_spin_dof >= 0) ? qvel_w[idx.front_wheel_spin_dof] : 0.0f;
    const float rw_qpos = (idx.rear_wheel_spin_qpos >= 0) ? qpos_w[idx.rear_wheel_spin_qpos] : 0.0f;
    const float rw_omega = (idx.rear_wheel_spin_dof >= 0) ? qvel_w[idx.rear_wheel_spin_dof] : 0.0f;

    float front_hold_nm = 0.0f;
    float rear_hold_nm = 0.0f;
    float front_brake_tq = 0.0f;
    float rear_brake_tq = 0.0f;
    {
        bool latch_f = (sp3_st->sp3.monitors.brake.latch_front != 0);
        float latch_f_rad = sp3_st->sp3.monitors.brake.latch_front_rad;
        const bike::physics_core::Status fb_st =
            bike::physics_core::eval_brake_hold_wheel<float, bike::physics_core::GpuToolkit>(
                &cfg->brake_hold,
                cfg->static_brake_ceiling_nm,
                brakes.final_front,
                fw_qpos,
                fw_omega,
                true,
                false,
                latch_f,
                latch_f_rad,
                front_hold_nm,
                err);
        if (fb_st != bike::physics_core::Status::ok) {
            return fb_st;
        }
        sp3_st->sp3.monitors.brake.latch_front = latch_f ? 1 : 0;
        sp3_st->sp3.monitors.brake.latch_front_rad = latch_f_rad;
        front_brake_tq = front_hold_nm;
        if (!latch_f && !has_static_fl && brakes.final_front > 0.0f && fw_omega != 0.0f &&
            cfg->brake_taper_rad_s > 0.0f && cfg->front_max_brake_torque_nm > 0.0f) {
            const float u = gpu_clamp(brakes.final_front, 0.0f, 1.0f);
            front_brake_tq = -(u * cfg->front_max_brake_torque_nm) *
                             bike::physics_core::tanhf_portable(fw_omega / cfg->brake_taper_rad_s);
        }
        if (idx.front_wheel_spin_dof >= 0) {
            qfrc_w[idx.front_wheel_spin_dof] += front_brake_tq;
        }
    }
    {
        bool latch_r = (sp3_st->sp3.monitors.brake.latch_rear != 0);
        float latch_r_rad = sp3_st->sp3.monitors.brake.latch_rear_rad;
        const bike::physics_core::Status rb_st_w =
            bike::physics_core::eval_brake_hold_wheel<float, bike::physics_core::GpuToolkit>(
                &cfg->brake_hold,
                cfg->static_brake_ceiling_nm,
                brakes.final_rear,
                rw_qpos,
                rw_omega,
                true,
                false,
                latch_r,
                latch_r_rad,
                rear_hold_nm,
                err);
        if (rb_st_w != bike::physics_core::Status::ok) {
            return rb_st_w;
        }
        sp3_st->sp3.monitors.brake.latch_rear = latch_r ? 1 : 0;
        sp3_st->sp3.monitors.brake.latch_rear_rad = latch_r_rad;
        rear_brake_tq = rear_hold_nm;
        if (!latch_r && !has_static_fl && brakes.final_rear > 0.0f && rw_omega != 0.0f &&
            cfg->brake_taper_rad_s > 0.0f && cfg->rear_max_brake_torque_nm > 0.0f) {
            const float u = gpu_clamp(brakes.final_rear, 0.0f, 1.0f);
            rear_brake_tq = -(u * cfg->rear_max_brake_torque_nm) *
                            bike::physics_core::tanhf_portable(rw_omega / cfg->brake_taper_rad_s);
        }
        if (idx.rear_wheel_spin_dof >= 0) {
            qfrc_w[idx.rear_wheel_spin_dof] += rear_brake_tq;
        }
    }

    sp3_st->last_front_hold_nm = front_brake_tq;
    sp3_st->last_rear_hold_nm = rear_brake_tq;
    sp3_st->last_front_frictionloss_nm = 0.0f;
    sp3_st->last_rear_frictionloss_nm = 0.0f;

    if (has_static_fl) {
        bool fl_changed = false;
        const bike::physics_core::Status fl_st =
            bike::physics_core::apply_static_brake<float, bike::physics_core::GpuToolkit>(
                cfg->static_brake_ceiling_nm,
                brakes.final_front,
                brakes.final_rear,
                front_hold_nm,
                rear_hold_nm,
                idx.front_wheel_spin_dof,
                idx.rear_wheel_spin_dof,
                idx.nv,
                dof_frictionloss_w,
                fl_changed,
                err);
        if (fl_st != bike::physics_core::Status::ok) {
            return fl_st;
        }
        if (idx.front_wheel_spin_dof >= 0 && idx.front_wheel_spin_dof < idx.nv) {
            sp3_st->last_front_frictionloss_nm = dof_frictionloss_w[idx.front_wheel_spin_dof];
        }
        if (idx.rear_wheel_spin_dof >= 0 && idx.rear_wheel_spin_dof < idx.nv) {
            sp3_st->last_rear_frictionloss_nm = dof_frictionloss_w[idx.rear_wheel_spin_dof];
        }
    }

    // 4. Autonomous shifting_update -> ratchet_shift_ratio_bound -> unconditional ratchet_prepare_bound -> tendon ranges
    const bool has_drive_shaft =
        (idx.drive_shaft_spin_qpos >= 0 && idx.drive_shaft_spin_dof >= 0);
    const float ds_qpos = has_drive_shaft ? qpos_w[idx.drive_shaft_spin_qpos] : crank_qpos;
    const float fh_driver_qpos = has_drive_shaft ? ds_qpos : crank_qpos;
    const float fh_driver_omega =
        has_drive_shaft ? qvel_w[idx.drive_shaft_spin_dof] : crank_omega;

    const float prev_cog_ratio =
        bike::physics_core::shifting_gear_ratio<float>(&cfg->shifting, &sp3_st->shifting);
    const float prev_applied_ratio =
        (sp3_st->freehub.ratio > 0.0f) ? sp3_st->freehub.ratio : prev_cog_ratio;
    const float cadence_rpm = fh_driver_omega * (60.0f / kTwoPi);
    const float req_cadence_before = (rw_omega / prev_applied_ratio) * (60.0f / kTwoPi);
    const bool rider_enabled = (intent_res.control.rider_enabled != 0);
    const float effort_cmd =
        (intent_res.control.has_human_torque_nm != 0) ? intent_res.control.human_torque_nm : 0.0f;

    bike::physics_core::RiderShiftInput rider_shift{};
    if (cfg->intent.enabled != 0) {
        rider_shift.owns = sp3_st->sp3.intent.commands.owns_shifting;
        if (sp3_st->sp3.intent.commands.stationary_rear_teeth >= 3) {
            rider_shift.has_stationary_target = 1;
            rider_shift.stationary_target_teeth = sp3_st->sp3.intent.commands.stationary_rear_teeth;
        } else if (sp3_st->sp3.intent.commands.shift_request != bike::physics_core::ShiftRequest::none) {
            rider_shift.has_request = 1;
            rider_shift.request =
                (sp3_st->sp3.intent.commands.shift_request == bike::physics_core::ShiftRequest::up)
                    ? bike::physics_core::ShiftDirection::up
                    : bike::physics_core::ShiftDirection::down;
        }
    }

    const bool rear_in_contact = (sp3_st->sp3.monitors.grounded[1].value != 0);
    const bool has_rear_slip = (sp3_st->last_wheel_grounded[1] != 0);
    const float rear_slip_mps = sp3_st->last_wheel_slip_mps[1];

    bool out_shifted = false;
    if (cfg->effort_mode != 0) {
        const bike::physics_core::Status sh_st =
            bike::physics_core::shifting_update<float, bike::physics_core::GpuToolkit>(
                &cfg->shifting,
                &sp3_st->shifting,
                cadence_rpm,
                req_cadence_before,
                dt_s,
                rider_enabled && (effort_cmd > 0.0f),
                brakes.shift_blocked != 0,
                rear_in_contact,
                has_rear_slip,
                rear_slip_mps,
                rider_shift,
                out_shifted,
                err);
        if (sh_st != bike::physics_core::Status::ok) {
            return sh_st;
        }
    }

    const int phys_cog = gpu_sp3_physical_cog_idx(cfg, &sp3_st->shifting);
    float target_ratio =
        bike::physics_core::shifting_gear_ratio<float>(&cfg->shifting, &sp3_st->shifting);
    if (phys_cog != sp3_st->shifting.active_cog_idx && sp3_st->shifting.from_teeth >= 3) {
        target_ratio = static_cast<float>(cfg->shifting.front_teeth) /
                       static_cast<float>(sp3_st->shifting.from_teeth);
    }
    bool fh_has = (sp3_st->freehub.has_boundary != 0);
    float fh_bound = sp3_st->freehub.boundary;
    if (target_ratio != prev_applied_ratio) {
        const bike::physics_core::Status srb_st =
            bike::physics_core::ratchet_shift_ratio_bound<float, bike::physics_core::GpuToolkit>(
                prev_applied_ratio, target_ratio, fh_driver_qpos, rw_qpos, fh_has, fh_bound, err);
        if (srb_st != bike::physics_core::Status::ok) {
            return srb_st;
        }
    }
    // Unconditionally call ratchet_prepare_bound on EVERY step
    const bike::physics_core::Status fhp_st =
        bike::physics_core::ratchet_prepare_bound<float, bike::physics_core::GpuToolkit>(
            target_ratio, fh_driver_qpos, rw_qpos, fh_has, fh_bound, err);
    if (fhp_st != bike::physics_core::Status::ok) {
        return fhp_st;
    }
    sp3_st->freehub.ratio = target_ratio;
    sp3_st->freehub.has_boundary = fh_has ? 1 : 0;
    sp3_st->freehub.boundary = fh_bound;
    const float active_cog_ub = fh_bound;
    const float curr_applied_ratio = target_ratio;

    float cc_bound = sp3_st->crank_clutch.boundary;
    if (has_drive_shaft) {
        bool cc_has = (sp3_st->crank_clutch.has_boundary != 0);
        const bike::physics_core::Status ccp_st =
            bike::physics_core::ratchet_prepare_bound<float, bike::physics_core::GpuToolkit>(
                1.0f, crank_qpos, ds_qpos, cc_has, cc_bound, err);
        if (ccp_st != bike::physics_core::Status::ok) {
            return ccp_st;
        }
        sp3_st->crank_clutch.ratio = 1.0f;
        sp3_st->crank_clutch.has_boundary = cc_has ? 1 : 0;
        sp3_st->crank_clutch.boundary = cc_bound;
    }

    const float tendon_ub =
        (ntendon >= cfg->shifting.cog_count) ? active_cog_ub : fh_bound;
    const bike::physics_core::Status tr_st = gpu_sp3_write_tendon_ranges(
        cfg, &sp3_st->shifting, idx, ntendon, tendon_ub, has_drive_shaft, cc_bound, tendon_range_w, err);
    if (tr_st != bike::physics_core::Status::ok) {
        return tr_st;
    }
    sp3_st->using_cassette_tendons = (ntendon >= cfg->shifting.cog_count) ? 1 : 0;

    // 5. Pedaling policy & crank torque
    // Matches runtime/step.cpp:1259-1262,1367-1371:
    // - ceiling is passed whenever automatic_effort is active (disabling the legacy low-cadence
    //   mash_torque_nm ramp while SeatedClimb supplies the effort);
    // - cut_after_ceiling is the resolver's effort_limited flag (LaunchPlan / FrontLiftReaction).
    const float req_cadence_after = (rw_omega / curr_applied_ratio) * (60.0f / kTwoPi);
    const float torque_factor =
        bike::physics_core::shift_torque_factor<float>(&cfg->shifting, &sp3_st->shifting);
    const bool automatic_effort =
        (cfg->intent.enabled != 0) &&
        (cfg->caller_human_torque_nm < 0.0f) &&
        !ov_human_tq &&
        (cfg->rider.rider_enabled != 0);
    const bool has_ceiling =
        automatic_effort && (intent_res.control.has_human_torque_nm != 0);
    const float ceiling_nm = has_ceiling ? intent_res.control.human_torque_nm : 0.0f;
    const bool eff_cut_after_ceiling =
        (cfg->cut_after_ceiling != 0) && (intent_res.effort_limited != 0);

    // One-shot reposition edge consumed on the first control_tick (step.cpp:1347-1349).
    if (cfg->reposition_request == 0) {
        sp3_st->reposition_consumed = 0;
    }
    const bool fire_reposition =
        (cfg->reposition_request != 0) && control_tick && (sp3_st->reposition_consumed == 0);
    if (fire_reposition) {
        sp3_st->reposition_consumed = 1;
    }

    bike::physics_core::PedalingOutput<float> pedal_out{};
    if (!rider_enabled || cfg->effort_mode == 0) {
        bike::physics_core::pedaling_reset<float>(&sp3_st->pedaling);
        sp3_st->stall_reposition = bike::physics_core::StallRepositionStatePod<float>{};
        if (fire_reposition) {
            sp3_st->pedaling.reposition.active = 1;
            sp3_st->pedaling.reposition.armed = 0;
            sp3_st->stall_reposition.active = 1;
            sp3_st->stall_reposition.armed = 0;
        }
        pedal_out.mode = bike::physics_core::PedalMode::disabled;
        pedal_out.reason = bike::physics_core::CoastReason::disabled;
        pedal_out.effort_nm = 0.0f;
        pedal_out.required_cadence_rpm = gpu_max(req_cadence_after, 0.0f);
        pedal_out.has_target_phase = 0;
        pedal_out.target_phase_rad = 0.0f;
        pedal_out.target_rate_rad_s = 0.0f;
    } else {
        bike::physics_core::StallRepositionConfigPod<float> repos_cfg{};
        repos_cfg.min_effort_nm = cfg->pedaling.reposition_min_effort_nm;
        repos_cfg.stall_window_s = gpu_max(
            dt_s,
            static_cast<float>(cfg->pedaling.reposition_stall_window_ticks) * dt_s);
        repos_cfg.stall_progress_rad = cfg->pedaling.reposition_stall_progress_rad;
        repos_cfg.cooldown_s = gpu_max(
            0.0f,
            static_cast<float>(cfg->pedaling.reposition_cooldown_ticks) * dt_s);
        repos_cfg.back_rate_rad_s = cfg->pedaling.reposition_back_rate_rad_s;
        repos_cfg.timeout_s = gpu_max(
            dt_s,
            static_cast<float>(cfg->pedaling.reposition_timeout_ticks) * dt_s);
        repos_cfg.phase_tolerance_rad = cfg->pedaling.reposition_phase_tolerance_rad;
        repos_cfg.noop_rad = cfg->pedaling.reposition_noop_rad;
        repos_cfg.reposition_on_stall = cfg->pedaling.reposition_on_stall;
        repos_cfg.motor_clutch_present = has_drive_shaft ? 1 : 0;
        repos_cfg.spindle_pedal_attachment =
            (!write_sp3_pd_ctrl || cfg->apply_direct_crank_torque == 0) ? 1 : 0;

        const bike::physics_core::Status ped_st =
            bike::physics_core::pedaling_step_with_reposition<float, bike::physics_core::GpuToolkit>(
                &cfg->pedaling,
                &sp3_st->pedaling,
                &repos_cfg,
                &sp3_st->stall_reposition,
                crank_qpos,
                crank_omega,
                req_cadence_after,
                effort_cmd,
                dt_s,
                brakes.legs_coast != 0,
                fire_reposition,
                has_drive_shaft,
                fh_driver_omega,
                has_ceiling,
                ceiling_nm,
                torque_factor,
                eff_cut_after_ceiling,
                pedal_out,
                err);
        if (ped_st != bike::physics_core::Status::ok) {
            return ped_st;
        }
    }

    float human_crank_tq = 0.0f;
    const bike::physics_core::Status hct_st =
        bike::physics_core::human_crank_torque<float, bike::physics_core::GpuToolkit>(
            pedal_out.effort_nm, crank_qpos, cfg->crank_torque_ripple, human_crank_tq, err);
    if (hct_st != bike::physics_core::Status::ok) {
        return hct_st;
    }
    sp3_st->last_commanded_crank_torque_nm = human_crank_tq;
    if (cfg->apply_direct_crank_torque != 0 && idx.crank_spin_dof >= 0) {
        qfrc_w[idx.crank_spin_dof] += human_crank_tq;
    }

    // 6. Motor assist step -> actuator ctrl or drive_shaft_spin qfrc
    // Matches DrivetrainWriter::stage_motor_assist + stage_settle (drivetrain.cpp:862-895,1054-1065):
    // assist_step_with_limit receives only explicit motor_torque_nm and motor_limit_nm,
    // clamps against actuator range, and reconciles assist.torque in post_solve.
    const float crank_rpm = crank_omega * (60.0f / kTwoPi);
    const bool has_tq_req = (intent_res.control.has_motor_torque_nm != 0);
    const float tq_req = has_tq_req ? intent_res.control.motor_torque_nm : 0.0f;
    const bool has_motor_lim = (intent_res.control.has_motor_limit_nm != 0);
    const float motor_lim = has_motor_lim ? intent_res.control.motor_limit_nm : 0.0f;

    float motor_tq = 0.0f;
    if (cfg->effort_mode != 0) {
        const float sensed_human_tq =
            (cfg->apply_direct_crank_torque != 0)
                ? human_crank_tq
                : sp3_st->last_efc.delivered_crank_torque_nm;
        const float assist_sensor_tq =
            (pedal_out.mode == bike::physics_core::PedalMode::pedaling)
                ? sensed_human_tq
                : 0.0f;
        bike::physics_core::AssistTickOutput<float> ast_out{};
        const bike::physics_core::Status ast_st =
            bike::physics_core::assist_step_with_limit<float, bike::physics_core::GpuToolkit>(
                &cfg->assist,
                &sp3_st->assist,
                assist_sensor_tq,
                crank_rpm,
                speed_mps,
                brakes.motor_braking != 0,
                dt_s,
                has_tq_req,
                tq_req,
                has_motor_lim,
                motor_lim,
                has_drive_shaft,
                cadence_rpm,
                true,
                ast_out,
                err);
        if (ast_st != bike::physics_core::Status::ok) {
            return ast_st;
        }
        motor_tq = ast_out.delivered_nm;
    }
    if (cfg->motor_actuator_idx >= 0 && cfg->motor_actuator_idx < idx.nu) {
        motor_tq = gpu_clamp(motor_tq, cfg->motor_ctrl_min_nm, cfg->motor_ctrl_max_nm);
        ctrl_w[cfg->motor_actuator_idx] = motor_tq;
    } else if (idx.drive_shaft_spin_dof >= 0) {
        qfrc_w[idx.drive_shaft_spin_dof] += motor_tq;
    }
    if (cfg->effort_mode != 0) {
        sp3_st->assist.torque = gpu_max(motor_tq, 0.0f);
    }
    sp3_st->last_motor_torque_nm = motor_tq;

    // 7. Suspension, tyres, resistance, cruise, and rider posture/envelope
    const GpuSuspensionParams susp_p = cfg->suspension;
    bike::physics_core::SuspensionResult<float> susp_res{};
    const bike::physics_core::Status susp_st =
        gpu_write_suspension_qfrc(susp_p, idx, qpos_w, qvel_w, qfrc_w, &susp_res);
    if (susp_st != bike::physics_core::Status::ok) {
        return susp_st;
    }

    const float fork_qpos_m = (idx.fork_travel_qpos >= 0) ? qpos_w[idx.fork_travel_qpos] : 0.0f;
    const float fork_qvel_mps = (idx.fork_travel_dof >= 0) ? qvel_w[idx.fork_travel_dof] : 0.0f;
    const float shock_qpos_m = (idx.shock_stroke_qpos >= 0) ? qpos_w[idx.shock_stroke_qpos] : 0.0f;
    const float shock_qvel_mps = (idx.shock_stroke_dof >= 0) ? qvel_w[idx.shock_stroke_dof] : 0.0f;
    sp3_st->last_step_d_susp_j =
        dt_s *
        gpu_sp3_eval_suspension_loss_w(
            &susp_p.params, susp_res, fork_qvel_mps, shock_qpos_m, shock_qvel_mps);

    bike::physics_core::GpuTyreStepOutput tyre_out{};
    bike::physics_core::ErrCode tyre_err = bike::physics_core::ErrCode::none;
    const bike::physics_core::Status tyre_st = bike::physics_core::gpu_step_both_wheels_tyre(
        tyre_model,
        terrain_desc,
        terrain_f32,
        terrain_i32,
        cfg->tire_stiff_scale,
        dt_s,
        xpos_w,
        xmat_w,
        xanchor_w,
        xaxis_w,
        qvel_w,
        compliant_wheels_w,
        dist_wheels_w,
        qfrc_w,
        tyre_out,
        tyre_err,
        false);
    if (tyre_st != bike::physics_core::Status::ok) {
        return tyre_st;
    }

    sp3_st->last_step_d_tyre_radial_j = 0.0f;
    sp3_st->last_step_d_tyre_brush_roll_j =
        gpu_max(0.0f, tyre_out.wheels[0].rolling_dissipation_j) +
        gpu_max(0.0f, tyre_out.wheels[1].rolling_dissipation_j);

    for (int s = 0; s < 2; ++s) {
        sp3_st->last_wheel_load_n[s] = tyre_out.wheels[s].normal_load_n;
        sp3_st->last_wheel_slip_mps[s] = tyre_out.wheels[s].slip_mps;
        sp3_st->last_wheel_eff_radius_m[s] = tyre_out.wheels[s].effective_radius_m;
        sp3_st->last_wheel_grounded[s] =
            (tyre_out.wheels[s].normal_load_n > cfg->efc_scan.load_threshold_n) ? 1 : 0;
    }
    const float total_wheel_load =
        tyre_out.wheels[0].normal_load_n + tyre_out.wheels[1].normal_load_n;
    if (total_wheel_load > 1.0f &&
        (sp3_st->last_wheel_grounded[0] != 0 || sp3_st->last_wheel_grounded[1] != 0)) {
        sp3_st->last_has_front_load_share = 1;
        sp3_st->last_front_load_share =
            gpu_clamp(tyre_out.wheels[0].normal_load_n / total_wheel_load, 0.0f, 1.0f);
    } else {
        sp3_st->last_has_front_load_share = 0;
        sp3_st->last_front_load_share = 0.0f;
    }

    bike::physics_core::ResistanceParams<float> res_p = cfg->resistance;
    if (tyre_model->backend == bike::physics_core::TyreBackendKind::distributed_2d_reference) {
        res_p.crr = 0.0f;
    }
    float qfrc_res_before[bike::physics_core::kMaxNv]{};
    if (cfg->track_energy_ledger != 0) {
        for (int i = 0; i < idx.nv; ++i) {
            qfrc_res_before[i] = qfrc_w[i];
        }
    }
    const bike::physics_core::Status res_st = gpu_write_resistance_qfrc(
        res_p,
        tyre_model,
        idx,
        qvel_w,
        xpos_w,
        xmat_w,
        xanchor_w,
        xaxis_w,
        tyre_out.wheels[0].normal_load_n,
        tyre_out.wheels[1].normal_load_n,
        tyre_out.wheels[0].effective_radius_m,
        tyre_out.wheels[1].effective_radius_m,
        qfrc_w);
    if (res_st != bike::physics_core::Status::ok) {
        return res_st;
    }
    if (cfg->track_energy_ledger != 0) {
        float res_power_w = 0.0f;
        for (int i = 0; i < idx.nv; ++i) {
            res_power_w += -(qfrc_w[i] - qfrc_res_before[i]) * qvel_w[i];
        }
        sp3_st->last_step_d_res_j = dt_s * gpu_max(0.0f, res_power_w);
    } else {
        sp3_st->last_step_d_res_j = 0.0f;
    }

    // Ideal-speed cruise controller (when cruise_enabled or program overrides cruise speed).
    float cruise_tq = 0.0f;
    if (cfg->cruise_enabled != 0 || ov_cruise_kmh) {
        const bool braking_cruise =
            (brakes.final_front > 0.0f) || (brakes.final_rear > 0.0f) ||
            (cfg->caller_braking != 0) || (brakes.motor_braking != 0);
        if (control_tick) {
            const bike::physics_core::i32 period =
                (cfg->control_period_steps > 1) ? cfg->control_period_steps : 1;
            const float cruise_dt_s = dt_s * static_cast<float>(period);
            const bike::physics_core::Status cr_st =
                bike::physics_core::cruise_step<float, bike::physics_core::GpuToolkit>(
                    &cfg->cruise,
                    &sp3_st->cruise,
                    speed_mps,
                    cruise_dt_s,
                    sp3_st->last_wheel_grounded[1] != 0,
                    false,
                    true,
                    sp3_st->sp3.monitors.grounded[1].value != 0,
                    cruise_tq,
                    err);
            if (cr_st != bike::physics_core::Status::ok) {
                return cr_st;
            }
        } else {
            cruise_tq = sp3_st->last_cruise_torque_nm;
        }
        if (braking_cruise && cruise_tq > 0.0f) {
            cruise_tq = 0.0f;
            sp3_st->cruise.torque_nm = 0.0f;
        }
        if (idx.rear_wheel_spin_dof >= 0 && idx.rear_wheel_spin_dof < idx.nv) {
            qfrc_w[idx.rear_wheel_spin_dof] += cruise_tq;
        }
    }
    sp3_st->last_cruise_torque_nm = cruise_tq;

    GpuRiderParams rider_p = cfg->rider;
    rider_p.rider_enabled = intent_res.control.rider_enabled;
    if (intent_res.control.has_posture != 0) {
        rider_p.target_qpos_rad[4] =
            cfg->rider.target_qpos_rad[4] + intent_res.control.posture.torso_lean_rad;
    }
    if (out_intent_res != nullptr) {
        *out_intent_res = intent_res;
    }
    if (out_pedal_out != nullptr) {
        *out_pedal_out = pedal_out;
    }
    float env_energy_j = 0.0f;
    bike::physics_core::Status rider_st = bike::physics_core::Status::ok;
    if (write_sp3_pd_ctrl) {
        rider_st = gpu_write_rider_qfrc_and_ctrl(
            rider_p, idx, qpos_w, qvel_w, qfrc_w, ctrl_w, &env_energy_j);
    } else {
        rider_st = gpu_write_rider_envelope_qfrc(
            rider_p, idx, qpos_w, qfrc_w, &env_energy_j);
    }
    if (rider_st != bike::physics_core::Status::ok) {
        return rider_st;
    }

    if (cfg->track_energy_ledger != 0) {
        float fork_energy_j = 0.0f;
        const bike::physics_core::Status fa_st = gpu_sp3_eval_fork_air_energy_j(
            &susp_p.params.air_spring, fork_qpos_m, fork_energy_j, err);
        if (fa_st != bike::physics_core::Status::ok) {
            return fa_st;
        }
        const float shock_energy_j =
            susp_res.coil_energy + susp_res.bumper_energy +
            susp_res.top_out_energy + susp_res.upper_energy;
        sp3_st->last_step_u_elastic_pre_j = fork_energy_j + shock_energy_j + env_energy_j;
    } else {
        sp3_st->last_step_u_elastic_pre_j = 0.0f;
    }
    return bike::physics_core::Status::ok;
}

} // namespace detail

template <
    class CfgPtr,
    class Sp3StatePtr,
    class CompliantStatePtr,
    class DistStatePtr,
    class XposPtr,
    class XmatPtr,
    class XanchorPtr,
    class XaxisPtr,
    class TendonRangePtr,
    class FrictionLossPtr>
BIKE_HD bike::physics_core::Status gpu_sp3_pre_solve_world(
    int w,
    float dt_s,
    const BIKE_THREAD GpuModelIndices &idx,
    BIKE_DEVICE const bike::physics_core::GpuTyreModel *tyre_model,
    const BIKE_THREAD bike::physics_core::GpuTerrainTableDesc &terrain_desc,
    BIKE_DEVICE const float *terrain_f32,
    BIKE_DEVICE const bike::physics_core::i32 *terrain_i32,
    CfgPtr cfg,
    Sp3StatePtr sp3_st,
    CompliantStatePtr compliant_wheels_w,
    DistStatePtr dist_wheels_w,
    const BIKE_THREAD float *qpos_w,
    const BIKE_THREAD float *qvel_w,
    XposPtr xpos_w,
    XmatPtr xmat_w,
    XanchorPtr xanchor_w,
    XaxisPtr xaxis_w,
    BIKE_THREAD float *qfrc_w,
    BIKE_THREAD float *ctrl_w,
    int ntendon,
    TendonRangePtr tendon_range_w,
    FrictionLossPtr dof_frictionloss_w
) noexcept {
    const bike::physics_core::i32 period =
        (cfg != nullptr && cfg->control_period_steps > 1) ? cfg->control_period_steps : 1;
    const bool control_tick =
        (sp3_st != nullptr) ? ((sp3_st->step_index % period) == 0) : true;
    return detail::gpu_sp3_pre_solve_core(
        w, dt_s, idx, tyre_model, terrain_desc, terrain_f32, terrain_i32,
        cfg, sp3_st, compliant_wheels_w, dist_wheels_w,
        qpos_w, qvel_w, xpos_w, xmat_w, xanchor_w, xaxis_w,
        qfrc_w, ctrl_w, ntendon, tendon_range_w, dof_frictionloss_w,
        false, false, 0.0f, false, 0.0f,
        true, control_tick, nullptr, nullptr);
}

namespace detail {

template <class PtrArr>
BIKE_HD bool sp3_geom_in_list(int g, PtrArr arr, int count) noexcept {
    for (int k = 0; k < count; ++k) {
        if (arr[k] == g) {
            return true;
        }
    }
    return false;
}

template <
    class CfgPtr,
    class PtrWorldId,
    class PtrGeom,
    class PtrEfcAdr,
    class PtrDim,
    class PtrType,
    class PtrId,
    class PtrForce>
BIKE_HD bike::physics_core::Status sp3_scan_contact_monitors_aos(
    CfgPtr cfg,
    int world_id,
    int ncon_count,
    int nefc_w,
    int contact_efc_stride,
    PtrWorldId contact_worldid,
    PtrGeom contact_geom,
    PtrEfcAdr contact_efc_address,
    PtrDim contact_dim,
    PtrType efc_type_w,
    PtrId efc_id_w,
    PtrForce efc_force_w,
    BIKE_THREAD bike::physics_core::EfcScanResult<float> &in_out,
    BIKE_THREAD bike::physics_core::ErrCode &err
) noexcept {
    err = bike::physics_core::ErrCode::none;
    in_out.handlebar_load_n = 0.0f;
    in_out.handlebar_in_contact = 0;
    in_out.contact_crash_cause = bike::physics_core::CrashCause::none;
    if (ncon_count < 0 || nefc_w < 0 || contact_efc_stride <= 0 ||
        cfg->terrain_geom_count < 0 || cfg->terrain_geom_count > bike::physics_core::kMaxTerrainGeoms ||
        cfg->rider_geom_count < 0 || cfg->rider_geom_count > bike::physics_core::kMaxRiderCrashGeoms) {
        err = bike::physics_core::ErrCode::out_of_bounds_chain;
        return bike::physics_core::Status::out_of_bounds;
    }
    if (ncon_count == 0) {
        return bike::physics_core::Status::ok;
    }
    if (contact_geom == nullptr || contact_efc_address == nullptr) {
        err = bike::physics_core::ErrCode::out_of_bounds_chain;
        return bike::physics_core::Status::invalid_argument;
    }
    if (nefc_w > 0 && (efc_type_w == nullptr || efc_force_w == nullptr)) {
        err = bike::physics_core::ErrCode::out_of_bounds_chain;
        return bike::physics_core::Status::invalid_argument;
    }

    for (int i = 0; i < ncon_count; ++i) {
        if (contact_worldid != nullptr && contact_worldid[i] != world_id) {
            continue;
        }
        const int g1 = contact_geom[2 * i + 0];
        const int g2 = contact_geom[2 * i + 1];

        const bool g1_terrain =
            sp3_geom_in_list(g1, cfg->terrain_geom_ids, cfg->terrain_geom_count);
        const bool g2_terrain =
            sp3_geom_in_list(g2, cfg->terrain_geom_ids, cfg->terrain_geom_count);
        const bool bar_hit =
            cfg->handlebar_geom_id >= 0 &&
            ((g1 == cfg->handlebar_geom_id && g2_terrain) ||
             (g2 == cfg->handlebar_geom_id && g1_terrain));
        const bool catch_hit =
            cfg->catch_geom_id >= 0 &&
            (g1 == cfg->catch_geom_id || g2 == cfg->catch_geom_id);
        const bool rider_ground =
            !catch_hit && (g1_terrain || g2_terrain) &&
            (sp3_geom_in_list(g1, cfg->rider_geom_ids, cfg->rider_geom_count) ||
             sp3_geom_in_list(g2, cfg->rider_geom_ids, cfg->rider_geom_count));

        if (!bar_hit && !catch_hit && !rider_ground) {
            continue;
        }
        const long adr_base = static_cast<long>(i) * static_cast<long>(contact_efc_stride);
        const int adr = contact_efc_address[adr_base + 0];
        if (adr < 0 || adr >= nefc_w) {
            continue;
        }

        float fn = 0.0f;
        const int ctype = efc_type_w[adr];
        if (ctype == bike::physics_core::kCnstrContactPyramidal) {
            if (contact_dim != nullptr) {
                const int dim = contact_dim[i];
                const int nrows = dim > 1 ? 2 * (dim - 1) : 1;
                if (nrows > 1 && contact_efc_stride >= nrows &&
                    contact_efc_address[adr_base + 1] >= 0) {
                    for (int k = 0; k < nrows; ++k) {
                        const int rk = contact_efc_address[adr_base + k];
                        if (rk >= 0 && rk < nefc_w) {
                            fn += efc_force_w[rk];
                        }
                    }
                } else {
                    const int end_row = (adr + nrows) < nefc_w ? (adr + nrows) : nefc_w;
                    for (int r = adr; r < end_row; ++r) {
                        fn += efc_force_w[r];
                    }
                }
            } else if (efc_id_w != nullptr) {
                for (int r = adr;
                     r < nefc_w && efc_type_w[r] == bike::physics_core::kCnstrContactPyramidal &&
                     efc_id_w[r] == i;
                     ++r) {
                    fn += efc_force_w[r];
                }
            } else {
                fn = efc_force_w[adr];
            }
        } else {
            fn = efc_force_w[adr];
        }

        if (!bike::physics_core::GpuToolkit::isfinite(fn)) {
            err = bike::physics_core::ErrCode::overflow_nonfinite;
            return bike::physics_core::Status::overflow;
        }
        if (bar_hit) {
            in_out.handlebar_load_n += bike::physics_core::GpuToolkit::abs(fn);
        }
        if (in_out.contact_crash_cause == bike::physics_core::CrashCause::none &&
            fn > cfg->load_threshold_n) {
            if (catch_hit) {
                in_out.contact_crash_cause = bike::physics_core::CrashCause::catch_plane_contact;
            } else if (rider_ground) {
                in_out.contact_crash_cause = bike::physics_core::CrashCause::rider_ground_contact;
            }
        }
    }

    in_out.handlebar_in_contact =
        (in_out.handlebar_load_n > cfg->load_threshold_n) ? 1 : 0;
    return bike::physics_core::Status::ok;
}

} // namespace detail

template <
    class CfgPtr,
    class Sp3StatePtr,
    class QposPtr,
    class QvelPtr,
    class XmatPtr,
    class SensorPtr,
    class PtrType,
    class PtrId,
    class PtrJ,
    class PtrForce,
    class PtrWorldId,
    class PtrGeom,
    class PtrEfcAdr,
    class PtrDim,
    class ActForcePtr = const float *>
BIKE_HD bike::physics_core::Status gpu_sp3_post_solve_world(
    int w,
    float dt_s,
    const BIKE_THREAD GpuModelIndices &idx,
    CfgPtr cfg,
    Sp3StatePtr sp3_st,
    QposPtr qpos_w,
    QvelPtr qvel_w,
    XmatPtr xmat_w,
    int nsensordata,
    SensorPtr sensordata_w,
    int nefc_w,
    int efc_J_row_stride,
    PtrType efc_type_w,
    PtrId efc_id_w,
    PtrJ efc_J_w,
    PtrForce efc_force_w,
    int ncon_or_total,
    int contact_efc_stride,
    PtrWorldId contact_worldid,
    PtrGeom contact_geom,
    PtrEfcAdr contact_efc_address,
    PtrDim contact_dim,
    ActForcePtr actuator_force_w = static_cast<const float *>(nullptr)
) noexcept {
    if (w < 0 || cfg == nullptr || sp3_st == nullptr || !gpu_validate_model_indices(idx) ||
        qpos_w == nullptr || qvel_w == nullptr || xmat_w == nullptr) {
        return bike::physics_core::Status::invalid_argument;
    }

    if (sp3_st->frozen != 0 && cfg->freeze_on_crash != 0) {
        return bike::physics_core::Status::ok;
    }

    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;

    // 1. Scan per-world efc rows (resolve active cassette tendon id dynamically when ntendon >= cog_count)
    bike::physics_core::EfcScanConfig<float> efc_cfg = cfg->efc_scan;
    const int phys_cog = gpu_sp3_physical_cog_idx(cfg, &sp3_st->shifting);
    if (sp3_st->using_cassette_tendons != 0 &&
        phys_cog >= 0 &&
        phys_cog < cfg->shifting.cog_count &&
        efc_cfg.freehub_tendon_id >= 0) {
        const int cog_tid = cfg->shifting.cassette_tendon_ids[phys_cog];
        if (cog_tid >= 0) {
            efc_cfg.freehub_tendon_id = cog_tid;
        }
    }

    bike::physics_core::EfcScanResult<float> scan_res{};
    const bike::physics_core::Status efc_st =
        bike::physics_core::scan_efc_reactions<float, bike::physics_core::GpuToolkit>(
            &efc_cfg,
            nefc_w,
            efc_J_row_stride,
            idx.nbody,
            efc_type_w,
            efc_id_w,
            efc_J_w,
            efc_force_w,
            xmat_w,
            scan_res,
            err);
    if (efc_st != bike::physics_core::Status::ok) {
        return efc_st;
    }

    // 2. Scan contacts (supports both mjwarp global nacon+worldid and per-world strided AoS layouts)
    if (ncon_or_total > 0) {
        const bike::physics_core::Status cm_st = detail::sp3_scan_contact_monitors_aos(
            &efc_cfg,
            w,
            ncon_or_total,
            nefc_w,
            contact_efc_stride,
            contact_worldid,
            contact_geom,
            contact_efc_address,
            contact_dim,
            efc_type_w,
            efc_id_w,
            efc_force_w,
            scan_res,
            err);
        if (cm_st != bike::physics_core::Status::ok) {
            return cm_st;
        }
    }
    sp3_st->last_efc = scan_res;

    // 3. Update GroundedFilter, BridgedLoad, BalanceMonitor, CrashDetector
    const bike::physics_core::i32 cur_step = sp3_st->step_index;
    for (int side = 0; side < 2; ++side) {
        bool gf_val = false;
        const bike::physics_core::Status gf_st =
            bike::physics_core::grounded_filter_update(
                &cfg->grounded_filter,
                sp3_st->last_wheel_grounded[side] != 0,
                cur_step,
                &sp3_st->sp3.monitors.grounded[side],
                gf_val,
                err);
        if (gf_st != bike::physics_core::Status::ok) {
            return gf_st;
        }
        if (cfg->use_bridged_load != 0) {
            (void)bike::physics_core::bridged_load_update(
                &cfg->bridged_load,
                sp3_st->last_wheel_load_n[side],
                &sp3_st->sp3.monitors.bridged[side]);
        }
    }

    const float speed_mps = (idx.root_x_dof >= 0) ? qvel_w[idx.root_x_dof] : 0.0f;
    const float pos_x_m = (idx.root_x_qpos >= 0) ? qpos_w[idx.root_x_qpos] : 0.0f;
    const float root_pitch_rad = (idx.root_pitch_qpos >= 0) ? qpos_w[idx.root_pitch_qpos] : 0.0f;

    const bike::physics_core::BalanceStopCore stop =
        bike::physics_core::balance_compute_stop<float, bike::physics_core::GpuToolkit>(
            sp3_st->last_brakes.motor_braking != 0,
            speed_mps,
            cfg->launch.standstill_speed_mps,
            sp3_st->sp3.intent.launch.phase,
            &sp3_st->sp3.monitors.balance);
    bool bal_lost = false;
    const bike::physics_core::Status bal_st =
        bike::physics_core::balance_monitor_update<float, bike::physics_core::GpuToolkit>(
            &cfg->balance,
            cur_step,
            pos_x_m,
            speed_mps,
            sp3_st->last_brakes.riding != 0,
            stop,
            &sp3_st->sp3.monitors.balance,
            bal_lost,
            err);
    if (bal_st != bike::physics_core::Status::ok) {
        return bal_st;
    }

    const float time_s = static_cast<float>(cur_step + 1) * dt_s;
    bike::physics_core::crash_detector_update<float, bike::physics_core::GpuToolkit>(
        scan_res.contact_crash_cause,
        scan_res.handlebar_in_contact != 0,
        cfg->efc_scan.pitch_limit_rad,
        time_s,
        pos_x_m,
        root_pitch_rad,
        &sp3_st->sp3.monitors.crash);
    if (cfg->freeze_on_crash != 0 &&
        sp3_st->sp3.monitors.crash.cause != bike::physics_core::CrashCause::none) {
        sp3_st->frozen = 1;
    }

    // Reconcile motor assist state against solved actuator torque when effort_mode != 0.
    if (cfg->effort_mode != 0) {
        const float req_motor = gpu_max(0.0f, sp3_st->last_motor_torque_nm);
        float solved_motor = req_motor;
        if (actuator_force_w != nullptr &&
            cfg->motor_actuator_idx >= 0 &&
            cfg->motor_actuator_idx < idx.nu) {
            solved_motor = actuator_force_w[cfg->motor_actuator_idx];
        }
        float settled_motor = 0.0f;
        const bike::physics_core::Status stl_st =
            bike::physics_core::assist_post_solve_settle<float, bike::physics_core::GpuToolkit>(
                &sp3_st->assist, req_motor, solved_motor, settled_motor, err);
        if (stl_st != bike::physics_core::Status::ok) {
            return stl_st;
        }
    }

    // 4. Capture IMU specific_force_body_mps2 & pitch_rate_up_rad_s
    if (sensordata_w != nullptr && cfg->accel_sensor_adr >= 0 &&
        cfg->accel_sensor_adr + 2 < nsensordata) {
        sp3_st->last_specific_force_body_mps2[0] = sensordata_w[cfg->accel_sensor_adr + 0];
        sp3_st->last_specific_force_body_mps2[1] = sensordata_w[cfg->accel_sensor_adr + 1];
        sp3_st->last_specific_force_body_mps2[2] = sensordata_w[cfg->accel_sensor_adr + 2];
    } else if (idx.frame_body >= 0 && idx.frame_body < idx.nbody) {
        const int fb = 9 * idx.frame_body;
        sp3_st->last_specific_force_body_mps2[0] = 9.81f * xmat_w[fb + 6];
        sp3_st->last_specific_force_body_mps2[1] = 9.81f * xmat_w[fb + 7];
        sp3_st->last_specific_force_body_mps2[2] = 9.81f * xmat_w[fb + 8];
    }
    if (sensordata_w != nullptr && cfg->gyro_sensor_adr >= 0 &&
        cfg->gyro_sensor_adr + 2 < nsensordata) {
        sp3_st->last_pitch_rate_up_rad_s = -sensordata_w[cfg->gyro_sensor_adr + 1];
    } else {
        sp3_st->last_pitch_rate_up_rad_s =
            (idx.root_pitch_dof >= 0) ? -qvel_w[idx.root_pitch_dof] : 0.0f;
    }

    // 5. Increment step_index
    sp3_st->step_index = cur_step + 1;

    // 6. Synchronized 2*pi epoch re-basing when any drivetrain/wheel angle exceeds 256 rad.
    const float fw_q = (idx.front_wheel_spin_qpos >= 0) ? qpos_w[idx.front_wheel_spin_qpos] : 0.0f;
    const float rw_q = (idx.rear_wheel_spin_qpos >= 0) ? qpos_w[idx.rear_wheel_spin_qpos] : 0.0f;
    const float cr_q = (idx.crank_spin_qpos >= 0) ? qpos_w[idx.crank_spin_qpos] : 0.0f;
    const bool has_ds = (idx.drive_shaft_spin_qpos >= 0 && idx.drive_shaft_spin_qpos < idx.nq);
    const float ds_q = has_ds ? qpos_w[idx.drive_shaft_spin_qpos] : cr_q;

    if (gpu_abs(fw_q) > 256.0f || gpu_abs(rw_q) > 256.0f ||
        gpu_abs(cr_q) > 256.0f || gpu_abs(ds_q) > 256.0f) {
        bike::physics_core::WorldRotationsRebaseState<float> rb{};
        rb.front_wheel_qpos = fw_q;
        rb.rear_wheel_qpos = rw_q;
        rb.crank_qpos = cr_q;
        rb.drive_shaft_qpos = ds_q;
        rb.freehub_ratio = sp3_st->freehub.ratio;
        rb.freehub_boundary = sp3_st->freehub.boundary;
        rb.has_freehub_boundary = sp3_st->freehub.has_boundary;
        rb.clutch_boundary = sp3_st->crank_clutch.boundary;
        rb.has_clutch_boundary = has_ds ? sp3_st->crank_clutch.has_boundary : 0;
        rb.brake_latch[0] = sp3_st->sp3.monitors.brake.latch_front_rad;
        rb.brake_latch[1] = sp3_st->sp3.monitors.brake.latch_rear_rad;
        rb.has_brake_latch[0] = sp3_st->sp3.monitors.brake.latch_front;
        rb.has_brake_latch[1] = sp3_st->sp3.monitors.brake.latch_rear;

        const bike::physics_core::Status rb_st =
            bike::physics_core::rebase_world_rotations_2pi<float, bike::physics_core::GpuToolkit>(
                &rb, err);
        if (rb_st != bike::physics_core::Status::ok) {
            return rb_st;
        }

        const float dq_crank = cr_q - rb.crank_qpos;
        const float dq_rw = rw_q - rb.rear_wheel_qpos;

        if (idx.front_wheel_spin_qpos >= 0) {
            qpos_w[idx.front_wheel_spin_qpos] = rb.front_wheel_qpos;
        }
        if (idx.rear_wheel_spin_qpos >= 0) {
            qpos_w[idx.rear_wheel_spin_qpos] = rb.rear_wheel_qpos;
        }
        if (idx.crank_spin_qpos >= 0) {
            qpos_w[idx.crank_spin_qpos] = rb.crank_qpos;
        }
        if (has_ds) {
            qpos_w[idx.drive_shaft_spin_qpos] = rb.drive_shaft_qpos;
        }
        sp3_st->freehub.boundary = rb.freehub_boundary;
        if (has_ds) {
            sp3_st->crank_clutch.boundary = rb.clutch_boundary;
        }
        sp3_st->sp3.monitors.brake.latch_front_rad = rb.brake_latch[0];
        sp3_st->sp3.monitors.brake.latch_rear_rad = rb.brake_latch[1];

        bike::physics_core::PedalingState<float> ped_tmp = sp3_st->pedaling;
        bike::physics_core::StallRepositionStatePod<float> repos_tmp = sp3_st->stall_reposition;
        bike::physics_core::rebase_pedaling_2pi<float>(ped_tmp, repos_tmp, dq_crank, dq_rw);
        sp3_st->pedaling = ped_tmp;
        sp3_st->stall_reposition = repos_tmp;
    }

    return bike::physics_core::Status::ok;
}

template <class CfgPtr, class QposPtr>
BIKE_HD bike::physics_core::Status gpu_sp3_eval_suspension_and_envelope_energy(
    CfgPtr cfg,
    const BIKE_THREAD GpuModelIndices &idx,
    QposPtr qpos_w,
    BIKE_THREAD float &out_u_susp_fork_j,
    BIKE_THREAD float &out_u_susp_shock_j,
    BIKE_THREAD float &out_u_rider_envelope_j,
    BIKE_THREAD bike::physics_core::ErrCode &err
) noexcept {
    err = bike::physics_core::ErrCode::none;
    out_u_susp_fork_j = 0.0f;
    out_u_susp_shock_j = 0.0f;
    out_u_rider_envelope_j = 0.0f;
    if (cfg == nullptr || qpos_w == nullptr) {
        err = bike::physics_core::ErrCode::outside_domain;
        return bike::physics_core::Status::invalid_argument;
    }

    const float fork_qpos_m = (idx.fork_travel_qpos >= 0) ? qpos_w[idx.fork_travel_qpos] : 0.0f;
    const float shock_qpos_m = (idx.shock_stroke_qpos >= 0) ? qpos_w[idx.shock_stroke_qpos] : 0.0f;

    const GpuSuspensionParams susp_p = cfg->suspension;
    const bike::physics_core::Status fa_st = detail::gpu_sp3_eval_fork_air_energy_j(
        &susp_p.params.air_spring, fork_qpos_m, out_u_susp_fork_j, err);
    if (fa_st != bike::physics_core::Status::ok) {
        return fa_st;
    }

    bike::physics_core::SuspensionResult<float> susp_res{};
    const bike::physics_core::Status su_st =
        bike::physics_core::eval_suspension<float, bike::physics_core::GpuToolkit>(
            &susp_p.params, fork_qpos_m, 0.0f, shock_qpos_m, 0.0f, susp_res, err);
    if (su_st != bike::physics_core::Status::ok) {
        return su_st;
    }
    out_u_susp_shock_j =
        susp_res.coil_energy + susp_res.bumper_energy +
        susp_res.top_out_energy + susp_res.upper_energy;

    if (cfg->rider.rider_enabled != 0) {
        float env_sum = 0.0f;
        const int count = (cfg->rider.envelope.count > bike::physics_core::kMaxEnvelopeJoints)
            ? bike::physics_core::kMaxEnvelopeJoints
            : cfg->rider.envelope.count;
        if (cfg->rider.envelope.count == kNumRiderJoints) {
            for (int j = 0; j < kNumRiderJoints; ++j) {
                const int qp = idx.rider_qpos[j];
                const int dof = idx.rider_dof[j];
                if (qp < 0 || qp >= idx.nq || dof < 0 || dof >= idx.nv) {
                    continue;
                }
                bike::physics_core::SoftEdgeResult<float> row{};
                const bike::physics_core::Status se_st =
                    bike::physics_core::soft_edge_scalar<float, bike::physics_core::GpuToolkit>(
                        qpos_w[qp],
                        cfg->rider.envelope.lower[j],
                        cfg->rider.envelope.upper[j],
                        cfg->rider.envelope.stiffness_nm_rad,
                        cfg->rider.envelope.margin_rad,
                        row,
                        err);
                if (se_st != bike::physics_core::Status::ok) {
                    return se_st;
                }
                env_sum += row.stored_j;
            }
        } else {
            for (int j = 0; j < count; ++j) {
                const int qp = cfg->rider.envelope.qpos_adrs[j];
                if (qp < 0 || qp >= idx.nq) {
                    continue;
                }
                bike::physics_core::SoftEdgeResult<float> row{};
                const bike::physics_core::Status se_st =
                    bike::physics_core::soft_edge_scalar<float, bike::physics_core::GpuToolkit>(
                        qpos_w[qp],
                        cfg->rider.envelope.lower[j],
                        cfg->rider.envelope.upper[j],
                        cfg->rider.envelope.stiffness_nm_rad,
                        cfg->rider.envelope.margin_rad,
                        row,
                        err);
                if (se_st != bike::physics_core::Status::ok) {
                    return se_st;
                }
                env_sum += row.stored_j;
            }
        }
        out_u_rider_envelope_j = env_sum;
    }
    return bike::physics_core::Status::ok;
}

template <class QvelPtr, class PtrType, class PtrJ, class PtrForce>
BIKE_HD bike::physics_core::Status gpu_sp3_scan_efc_constraint_work(
    float dt_s,
    const BIKE_THREAD GpuModelIndices &idx,
    QvelPtr qvel_w,
    int nefc_w,
    int efc_J_row_stride,
    PtrType efc_type_w,
    PtrJ efc_J_w,
    PtrForce efc_force_w,
    BIKE_THREAD float &out_w_constraint_signed_j,
    BIKE_THREAD float &out_w_constraint_abs_j,
    BIKE_THREAD float &out_w_closure_signed_j,
    BIKE_THREAD float &out_w_closure_abs_j,
    BIKE_THREAD float &out_d_rider_end_stop_j,
    BIKE_THREAD bike::physics_core::ErrCode &err
) noexcept {
    err = bike::physics_core::ErrCode::none;
    out_w_constraint_signed_j = 0.0f;
    out_w_constraint_abs_j = 0.0f;
    out_w_closure_signed_j = 0.0f;
    out_w_closure_abs_j = 0.0f;
    out_d_rider_end_stop_j = 0.0f;
    if (nefc_w <= 0 || qvel_w == nullptr || efc_type_w == nullptr ||
        efc_J_w == nullptr || efc_force_w == nullptr || efc_J_row_stride <= 0) {
        return bike::physics_core::Status::ok;
    }
    const int kCnstrLimitJoint = 3;
    const int cols = (idx.nv < efc_J_row_stride) ? idx.nv : efc_J_row_stride;
    for (int r = 0; r < nefc_w; ++r) {
        const bike::physics_core::i32 t = efc_type_w[r];
        const bool is_closure =
            (t == bike::physics_core::kCnstrEquality) ||
            (t == bike::physics_core::kCnstrLimitTendon);
        const bool is_joint_limit = (t == kCnstrLimitJoint);
        if (!is_closure && !is_joint_limit) {
            continue;
        }
        float j_dot_v = 0.0f;
        const int base = r * efc_J_row_stride;
        for (int c = 0; c < cols; ++c) {
            j_dot_v += efc_J_w[base + c] * qvel_w[c];
        }
        const float work_j = efc_force_w[r] * j_dot_v * dt_s;
        if (is_closure) {
            out_w_closure_signed_j += work_j;
            out_w_closure_abs_j += gpu_abs(work_j);
        } else {
            bool is_rider_limit = false;
            for (int j = 0; j < kNumRiderJoints; ++j) {
                const int dof = idx.rider_dof[j];
                if (dof >= 0 && dof < cols && efc_J_w[base + dof] != 0.0f) {
                    is_rider_limit = true;
                    break;
                }
            }
            if (is_rider_limit) {
                out_d_rider_end_stop_j += gpu_max(0.0f, -work_j);
            } else {
                out_w_constraint_signed_j += work_j;
                out_w_constraint_abs_j += gpu_abs(work_j);
            }
        }
    }
    return bike::physics_core::Status::ok;
}

template <
    class CfgPtr,
    class Sp3StatePtr,
    class PtrRowAdr,
    class PtrRowNnz,
    class PtrColInd,
    class PtrM,
    class QposPtr,
    class QvelPtr,
    class PtrMass,
    class PtrXipos,
    class PtrGrav>
BIKE_HD bike::physics_core::Status gpu_sp3_init_energy_ledger_world(
    const BIKE_THREAD GpuModelIndices &idx,
    CfgPtr cfg,
    Sp3StatePtr sp3_st,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM M,
    QposPtr qpos_w,
    QvelPtr qvel_w,
    PtrMass body_mass,
    PtrXipos xipos_or_com_w,
    PtrGrav gravity,
    bike::physics_core::ElasticEnergyBreakdown<float> extra_elastic =
        bike::physics_core::ElasticEnergyBreakdown<float>{},
    float e_scale_min_j = 1.0f
) noexcept {
    if (cfg == nullptr || sp3_st == nullptr || !gpu_validate_model_indices(idx)) {
        return bike::physics_core::Status::invalid_argument;
    }
    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    bike::physics_core::ElasticEnergyBreakdown<float> elastic = extra_elastic;
    const bike::physics_core::Status el_st = gpu_sp3_eval_suspension_and_envelope_energy(
        cfg, idx, qpos_w,
        elastic.u_susp_fork, elastic.u_susp_shock, elastic.u_rider_envelope, err);
    if (el_st != bike::physics_core::Status::ok) {
        return el_st;
    }
    bike::physics_core::MechanicalEnergyBreakdown<float> mech{};
    const bike::physics_core::Status me_st =
        bike::physics_core::compute_mechanical_energy<float, bike::physics_core::GpuToolkit>(
            idx.nv, idx.nbody, M_rowadr, M_rownnz, M_colind, M,
            qvel_w, body_mass, xipos_or_com_w, gravity, elastic, mech, err);
    if (me_st != bike::physics_core::Status::ok) {
        return me_st;
    }
    return bike::physics_core::energy_ledger_init<float, bike::physics_core::GpuToolkit>(
        &sp3_st->energy_ledger, mech, e_scale_min_j, err);
}

template <
    class CfgPtr,
    class Sp3StatePtr,
    class PtrRowAdr,
    class PtrRowNnz,
    class PtrColInd,
    class PtrM,
    class QposPtr,
    class QvelPtr,
    class PtrMass,
    class PtrXipos,
    class PtrGrav,
    class PtrType = const bike::physics_core::i32 *,
    class PtrJ = const float *,
    class PtrForce = const float *>
BIKE_HD bike::physics_core::Status gpu_sp3_step_energy_ledger_world(
    float dt_s,
    const BIKE_THREAD GpuModelIndices &idx,
    CfgPtr cfg,
    Sp3StatePtr sp3_st,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM M,
    QposPtr qpos_w,
    QvelPtr qvel_w,
    PtrMass body_mass,
    PtrXipos xipos_or_com_w,
    PtrGrav gravity,
    float w_muscle_signed_j,
    float w_muscle_pos_j,
    float w_external_signed_j = 0.0f,
    bike::physics_core::ElasticEnergyBreakdown<float> extra_elastic =
        bike::physics_core::ElasticEnergyBreakdown<float>{},
    int nefc_w = 0,
    int efc_J_row_stride = 0,
    PtrType efc_type_w = static_cast<const bike::physics_core::i32 *>(nullptr),
    PtrJ efc_J_w = static_cast<const float *>(nullptr),
    PtrForce efc_force_w = static_cast<const float *>(nullptr),
    float extra_d_tyre_radial_j = 0.0f
) noexcept {
    if (cfg == nullptr || sp3_st == nullptr || !gpu_validate_model_indices(idx)) {
        return bike::physics_core::Status::invalid_argument;
    }
    if (cfg->track_energy_ledger == 0) {
        return bike::physics_core::Status::ok;
    }
    if (sp3_st->frozen != 0 && cfg->freeze_on_crash != 0) {
        return bike::physics_core::Status::ok;
    }

    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    bike::physics_core::ElasticEnergyBreakdown<float> elastic = extra_elastic;
    const bike::physics_core::Status el_st = gpu_sp3_eval_suspension_and_envelope_energy(
        cfg, idx, qpos_w,
        elastic.u_susp_fork, elastic.u_susp_shock, elastic.u_rider_envelope, err);
    if (el_st != bike::physics_core::Status::ok) {
        return el_st;
    }

    bike::physics_core::MechanicalEnergyBreakdown<float> mech{};
    const bike::physics_core::Status me_st =
        bike::physics_core::compute_mechanical_energy<float, bike::physics_core::GpuToolkit>(
            idx.nv, idx.nbody, M_rowadr, M_rownnz, M_colind, M,
            qvel_w, body_mass, xipos_or_com_w, gravity, elastic, mech, err);
    if (me_st != bike::physics_core::Status::ok) {
        return me_st;
    }

    const float fw_omega =
        (idx.front_wheel_spin_dof >= 0) ? qvel_w[idx.front_wheel_spin_dof] : 0.0f;
    const float rw_omega =
        (idx.rear_wheel_spin_dof >= 0) ? qvel_w[idx.rear_wheel_spin_dof] : 0.0f;
    const float ds_omega =
        (idx.drive_shaft_spin_dof >= 0) ? qvel_w[idx.drive_shaft_spin_dof] : 0.0f;
    const float crank_omega =
        (idx.crank_spin_dof >= 0) ? qvel_w[idx.crank_spin_dof] : 0.0f;

    float d_brake_hold_j = 0.0f;
    float d_brake_friction_j = 0.0f;
    const bike::physics_core::Status brk_st =
        bike::physics_core::energy_brake_dissipation_step<float, bike::physics_core::GpuToolkit>(
            sp3_st->last_front_hold_nm,
            sp3_st->last_rear_hold_nm,
            sp3_st->last_front_frictionloss_nm,
            sp3_st->last_rear_frictionloss_nm,
            fw_omega,
            rw_omega,
            dt_s,
            d_brake_hold_j,
            d_brake_friction_j,
            err);
    if (brk_st != bike::physics_core::Status::ok) {
        return brk_st;
    }

    float w_cnstr_s = 0.0f;
    float w_cnstr_a = 0.0f;
    float w_clos_s = 0.0f;
    float w_clos_a = 0.0f;
    float d_end_stop = 0.0f;
    const bike::physics_core::Status efc_st = gpu_sp3_scan_efc_constraint_work(
        dt_s, idx, qvel_w, nefc_w, efc_J_row_stride,
        efc_type_w, efc_J_w, efc_force_w,
        w_cnstr_s, w_cnstr_a, w_clos_s, w_clos_a, d_end_stop, err);
    if (efc_st != bike::physics_core::Status::ok) {
        return efc_st;
    }

    const float motor_omega =
        (idx.drive_shaft_spin_dof >= 0) ? ds_omega : crank_omega;
    const float w_motor_signed_j = sp3_st->last_motor_torque_nm * motor_omega * dt_s;
    const float w_cruise_signed_j = sp3_st->last_cruise_torque_nm * rw_omega * dt_s;

    bike::physics_core::GpuStepEnergyInputs<float> in{};
    in.E_mech = mech.total_j;
    in.w_muscle_signed_j = w_muscle_signed_j;
    in.w_muscle_pos_j = gpu_max(0.0f, w_muscle_pos_j);
    in.w_motor_signed_j = w_motor_signed_j;
    in.w_motor_pos_j = gpu_max(0.0f, w_motor_signed_j);
    in.w_cruise_signed_j = w_cruise_signed_j;
    in.w_external_signed_j = w_external_signed_j;
    in.d_tyre_radial_j =
        gpu_max(0.0f, sp3_st->last_step_d_tyre_radial_j) + gpu_max(0.0f, extra_d_tyre_radial_j);
    in.d_tyre_brush_roll_j = gpu_max(0.0f, sp3_st->last_step_d_tyre_brush_roll_j);
    in.d_suspension_j = gpu_max(0.0f, sp3_st->last_step_d_susp_j);
    in.d_resistance_j = gpu_max(0.0f, sp3_st->last_step_d_res_j);
    in.d_brake_hold_j = d_brake_hold_j;
    in.d_brake_friction_j = d_brake_friction_j;
    in.d_rider_end_stop_j = d_end_stop;
    in.w_constraint_signed_j = w_cnstr_s;
    in.w_constraint_abs_j = w_cnstr_a;
    in.w_closure_signed_j = w_clos_s;
    in.w_closure_abs_j = w_clos_a;

    if (sp3_st->energy_ledger.initialized == 0) {
        return bike::physics_core::energy_ledger_init<float, bike::physics_core::GpuToolkit>(
            &sp3_st->energy_ledger, mech, 1.0f, err);
    }
    return bike::physics_core::energy_ledger_step<float, bike::physics_core::GpuToolkit>(
        &sp3_st->energy_ledger, in, err);
}

} // namespace bike::gpu

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
