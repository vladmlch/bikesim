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
#include "../physics_core/efc_monitors.hpp"
#include "../physics_core/front_lift.hpp"
#include "../physics_core/launch_plan.hpp"
#include "../physics_core/pedaling.hpp"
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
    bike::physics_core::i32 step_index = 0;
    float last_wheel_load_n[2]{};
    float last_wheel_slip_mps[2]{};
    float last_wheel_eff_radius_m[2]{};
    float last_front_load_share = 0.0f;
    float last_commanded_crank_torque_nm = 0.0f;
    float last_motor_torque_nm = 0.0f;
    float last_pitch_rate_up_rad_s = 0.0f;
    float last_specific_force_body_mps2[3]{0.0f, 0.0f, 9.81f};
    bike::physics_core::u8 last_wheel_grounded[2]{};
    bike::physics_core::u8 last_has_front_load_share = 0;
    bike::physics_core::u8 using_cassette_tendons = 0;
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
    cfg.assist.slew = 400.0f;
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
    const float cap_nm[10] = {38.5f, 15.0f, 14.5f, 13.2f, 14.0f, 15.5f, 19.5f, 18.5f, 21.0f, 22.5f};
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
    GpuSp3Config base{};
    gpu_init_default_sp3_config(base);
    for (int w = 0; w < nworld; ++w) {
        GpuSp3Config cfg = base;
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
        configs[w] = cfg;
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
    if (ntendon >= cfg->shifting.cog_count && shifting->active_cog_idx >= 0 &&
        shifting->active_cog_idx < cfg->shifting.cog_count) {
        const bike::physics_core::Status fc_st =
            bike::physics_core::fill_cassette_tendon_ranges<float, bike::physics_core::GpuToolkit>(
                shifting->active_cog_idx,
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

    // 1. Reset RiderIntent, monitors, shifting, pedaling, assist in-place (zero large stack allocs)
    bike::physics_core::rider_intent_reset<float>(&cfg->intent, &sp3_st->sp3.intent);
    gpu_sp3_reset_monitors(cfg, &sp3_st->sp3.monitors);
    bike::physics_core::shifting_reset<float>(&cfg->shifting, &sp3_st->shifting);
    bike::physics_core::pedaling_reset<float>(&sp3_st->pedaling);
    bike::physics_core::assist_reset<float>(&sp3_st->assist);

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
    sp3_st->last_pitch_rate_up_rad_s = 0.0f;
    sp3_st->last_specific_force_body_mps2[0] = 0.0f;
    sp3_st->last_specific_force_body_mps2[1] = 0.0f;
    sp3_st->last_specific_force_body_mps2[2] = 9.81f;
    sp3_st->using_cassette_tendons = (ntendon >= cfg->shifting.cog_count) ? 1 : 0;

    return gpu_sp3_write_tendon_ranges(
        cfg, &sp3_st->shifting, idx, ntendon, fh_bound, has_ds, cc_bound, tendon_range_w, err);
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

    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    const float kTwoPi = 6.28318530717958647692f;

    // 1. Road grades & surface labels from shared polyline terrain
    const float front_wheel_x =
        (idx.front_wheel_body >= 0) ? xpos_w[3 * idx.front_wheel_body + 0] : 0.0f;
    const float rear_wheel_x =
        (idx.rear_wheel_body >= 0) ? xpos_w[3 * idx.rear_wheel_body + 0] : 0.0f;

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

    // 2. RiderIntent resolution using previous-step solved reactions & IMU signals
    const float speed_mps = (idx.root_x_dof >= 0) ? qvel_w[idx.root_x_dof] : 0.0f;
    const float crank_qpos = (idx.crank_spin_qpos >= 0) ? qpos_w[idx.crank_spin_qpos] : 0.0f;
    const float crank_omega = (idx.crank_spin_dof >= 0) ? qvel_w[idx.crank_spin_dof] : 0.0f;

    bike::physics_core::IntentControlCore<float> in_control{};
    in_control.rider_enabled = (cfg->rider.rider_enabled != 0) ? 1 : 0;
    if (cfg->caller_motor_torque_nm >= 0.0f) {
        in_control.has_motor_torque_nm = 1;
        in_control.motor_torque_nm = cfg->caller_motor_torque_nm;
    }
    if (cfg->caller_motor_limit_nm >= 0.0f) {
        in_control.has_motor_limit_nm = 1;
        in_control.motor_limit_nm = cfg->caller_motor_limit_nm;
    }
    if (cfg->caller_human_torque_nm >= 0.0f) {
        in_control.has_human_torque_nm = 1;
        in_control.human_torque_nm = cfg->caller_human_torque_nm;
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
        (cfg->caller_front_brake > 0.0f || cfg->caller_rear_brake > 0.0f || cfg->caller_braking != 0)
            ? 1
            : 0;

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
            cfg->has_lean_limit != 0,
            cfg->lean_limit_rad,
            true,
            cfg->has_min_lean != 0,
            cfg->min_lean_rad,
            launch_ctx,
            &sp3_st->sp3.intent,
            intent_res,
            err);
    if (ri_st != bike::physics_core::Status::ok) {
        return ri_st;
    }

    // 3. Brakes: rollback_brake_demand -> resolve_brakes_core -> dof_frictionloss -> eval_brake_hold_wheel
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
    breq.caller_front = cfg->caller_front_brake;
    breq.caller_rear = cfg->caller_rear_brake;
    breq.rollback_demand = rollback_dem;
    breq.rider_front = (cfg->intent.enabled != 0) ? sp3_st->sp3.intent.commands.front_brake : 0.0f;
    breq.rider_rear = (cfg->intent.enabled != 0) ? sp3_st->sp3.intent.commands.rear_brake : 0.0f;
    breq.caller_braking =
        (cfg->caller_braking != 0 || cfg->caller_front_brake > 0.0f || cfg->caller_rear_brake > 0.0f)
            ? 1
            : 0;
    breq.rider_present = cfg->rider_present;
    breq.rider_enabled = intent_res.control.rider_enabled;

    const bike::physics_core::BrakeResolutionCore<float> brakes =
        bike::physics_core::resolve_brakes_core<float, bike::physics_core::GpuToolkit>(breq);
    sp3_st->last_brakes = brakes;

    const bool has_static_fl = (dof_frictionloss_w != nullptr);
    if (has_static_fl) {
        bool fl_changed = false;
        const bike::physics_core::Status fl_st =
            bike::physics_core::apply_static_brake<float, bike::physics_core::GpuToolkit>(
                cfg->static_brake_ceiling_nm,
                brakes.final_front,
                brakes.final_rear,
                idx.front_wheel_spin_dof,
                idx.rear_wheel_spin_dof,
                idx.nv,
                dof_frictionloss_w,
                fl_changed,
                err);
        if (fl_st != bike::physics_core::Status::ok) {
            return fl_st;
        }
    }

    const float fw_qpos = (idx.front_wheel_spin_qpos >= 0) ? qpos_w[idx.front_wheel_spin_qpos] : 0.0f;
    const float fw_omega = (idx.front_wheel_spin_dof >= 0) ? qvel_w[idx.front_wheel_spin_dof] : 0.0f;
    const float rw_qpos = (idx.rear_wheel_spin_qpos >= 0) ? qpos_w[idx.rear_wheel_spin_qpos] : 0.0f;
    const float rw_omega = (idx.rear_wheel_spin_dof >= 0) ? qvel_w[idx.rear_wheel_spin_dof] : 0.0f;

    {
        bool latch_f = (sp3_st->sp3.monitors.brake.latch_front != 0);
        float latch_f_rad = sp3_st->sp3.monitors.brake.latch_front_rad;
        float fb_hold_tq = 0.0f;
        const bike::physics_core::Status fb_st =
            bike::physics_core::eval_brake_hold_wheel<float, bike::physics_core::GpuToolkit>(
                &cfg->brake_hold,
                brakes.final_front,
                fw_qpos,
                fw_omega,
                true,
                false,
                latch_f,
                latch_f_rad,
                fb_hold_tq,
                err);
        if (fb_st != bike::physics_core::Status::ok) {
            return fb_st;
        }
        sp3_st->sp3.monitors.brake.latch_front = latch_f ? 1 : 0;
        sp3_st->sp3.monitors.brake.latch_front_rad = latch_f_rad;
        float fb_tq = fb_hold_tq;
        if (!latch_f && !has_static_fl && brakes.final_front > 0.0f && fw_omega != 0.0f &&
            cfg->brake_taper_rad_s > 0.0f && cfg->front_max_brake_torque_nm > 0.0f) {
            const float u = gpu_clamp(brakes.final_front, 0.0f, 1.0f);
            fb_tq = -(u * cfg->front_max_brake_torque_nm) *
                    bike::physics_core::tanhf_portable(fw_omega / cfg->brake_taper_rad_s);
        }
        if (idx.front_wheel_spin_dof >= 0) {
            qfrc_w[idx.front_wheel_spin_dof] += fb_tq;
        }
    }
    {
        bool latch_r = (sp3_st->sp3.monitors.brake.latch_rear != 0);
        float latch_r_rad = sp3_st->sp3.monitors.brake.latch_rear_rad;
        float rb_hold_tq = 0.0f;
        const bike::physics_core::Status rb_st_w =
            bike::physics_core::eval_brake_hold_wheel<float, bike::physics_core::GpuToolkit>(
                &cfg->brake_hold,
                brakes.final_rear,
                rw_qpos,
                rw_omega,
                true,
                false,
                latch_r,
                latch_r_rad,
                rb_hold_tq,
                err);
        if (rb_st_w != bike::physics_core::Status::ok) {
            return rb_st_w;
        }
        sp3_st->sp3.monitors.brake.latch_rear = latch_r ? 1 : 0;
        sp3_st->sp3.monitors.brake.latch_rear_rad = latch_r_rad;
        float rb_tq = rb_hold_tq;
        if (!latch_r && !has_static_fl && brakes.final_rear > 0.0f && rw_omega != 0.0f &&
            cfg->brake_taper_rad_s > 0.0f && cfg->rear_max_brake_torque_nm > 0.0f) {
            const float u = gpu_clamp(brakes.final_rear, 0.0f, 1.0f);
            rb_tq = -(u * cfg->rear_max_brake_torque_nm) *
                    bike::physics_core::tanhf_portable(rw_omega / cfg->brake_taper_rad_s);
        }
        if (idx.rear_wheel_spin_dof >= 0) {
            qfrc_w[idx.rear_wheel_spin_dof] += rb_tq;
        }
    }

    // 4. Autonomous shifting_update -> ratchet_shift_ratio_bound -> unconditional ratchet_prepare_bound -> tendon ranges
    const bool has_drive_shaft =
        (idx.drive_shaft_spin_qpos >= 0 && idx.drive_shaft_spin_dof >= 0);
    const float ds_qpos = has_drive_shaft ? qpos_w[idx.drive_shaft_spin_qpos] : crank_qpos;
    const float fh_driver_qpos = has_drive_shaft ? ds_qpos : crank_qpos;
    const float fh_driver_omega =
        has_drive_shaft ? qvel_w[idx.drive_shaft_spin_dof] : crank_omega;

    const float prev_ratio =
        bike::physics_core::shifting_gear_ratio<float>(&cfg->shifting, &sp3_st->shifting);
    const float cadence_rpm = fh_driver_omega * (60.0f / kTwoPi);
    const float req_cadence_before = (rw_omega / prev_ratio) * (60.0f / kTwoPi);
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

    const float new_ratio =
        bike::physics_core::shifting_gear_ratio<float>(&cfg->shifting, &sp3_st->shifting);
    bool fh_has = (sp3_st->freehub.has_boundary != 0);
    float fh_bound = sp3_st->freehub.boundary;
    if (out_shifted) {
        const bike::physics_core::Status srb_st =
            bike::physics_core::ratchet_shift_ratio_bound<float, bike::physics_core::GpuToolkit>(
                prev_ratio, new_ratio, fh_driver_qpos, rw_qpos, fh_has, fh_bound, err);
        if (srb_st != bike::physics_core::Status::ok) {
            return srb_st;
        }
    }
    // Unconditionally call ratchet_prepare_bound on EVERY step
    const bike::physics_core::Status fhp_st =
        bike::physics_core::ratchet_prepare_bound<float, bike::physics_core::GpuToolkit>(
            new_ratio, fh_driver_qpos, rw_qpos, fh_has, fh_bound, err);
    if (fhp_st != bike::physics_core::Status::ok) {
        return fhp_st;
    }
    sp3_st->freehub.ratio = new_ratio;
    sp3_st->freehub.has_boundary = fh_has ? 1 : 0;
    sp3_st->freehub.boundary = fh_bound;

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

    const bike::physics_core::Status tr_st = gpu_sp3_write_tendon_ranges(
        cfg, &sp3_st->shifting, idx, ntendon, fh_bound, has_drive_shaft, cc_bound, tendon_range_w, err);
    if (tr_st != bike::physics_core::Status::ok) {
        return tr_st;
    }
    sp3_st->using_cassette_tendons = (ntendon >= cfg->shifting.cog_count) ? 1 : 0;

    // 5. Pedaling policy & crank torque
    const float req_cadence_after = (rw_omega / new_ratio) * (60.0f / kTwoPi);
    const float torque_factor =
        bike::physics_core::shift_torque_factor<float>(&cfg->shifting, &sp3_st->shifting);
    const bool has_ceiling =
        (intent_res.effort_limited != 0) && (intent_res.control.has_human_torque_nm != 0);
    const float ceiling_nm = has_ceiling ? intent_res.control.human_torque_nm : 0.0f;

    bike::physics_core::PedalingOutput<float> pedal_out{};
    if (!rider_enabled || cfg->effort_mode == 0) {
        bike::physics_core::pedaling_reset<float>(&sp3_st->pedaling);
        if (cfg->reposition_request != 0) {
            sp3_st->pedaling.reposition.active = 1;
            sp3_st->pedaling.reposition.armed = 0;
        }
        pedal_out.mode = bike::physics_core::PedalMode::disabled;
        pedal_out.reason = bike::physics_core::CoastReason::disabled;
        pedal_out.effort_nm = 0.0f;
        pedal_out.required_cadence_rpm = gpu_max(req_cadence_after, 0.0f);
        pedal_out.has_target_phase = 0;
        pedal_out.target_phase_rad = 0.0f;
        pedal_out.target_rate_rad_s = 0.0f;
    } else {
        const bike::physics_core::Status ped_st =
            bike::physics_core::pedaling_update<float, bike::physics_core::GpuToolkit>(
                &cfg->pedaling,
                &sp3_st->pedaling,
                crank_qpos,
                crank_omega,
                req_cadence_after,
                effort_cmd,
                dt_s,
                brakes.legs_coast != 0,
                cfg->reposition_request != 0,
                has_drive_shaft,
                fh_driver_omega,
                has_ceiling,
                ceiling_nm,
                torque_factor,
                cfg->cut_after_ceiling != 0,
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
    const float crank_rpm = crank_omega * (60.0f / kTwoPi);
    const bool has_tq_req =
        (intent_res.control.has_motor_torque_nm != 0) || (intent_res.control.has_motor_limit_nm != 0);
    float tq_req = 0.0f;
    if (intent_res.control.has_motor_torque_nm != 0 && intent_res.control.has_motor_limit_nm != 0) {
        tq_req = gpu_min(intent_res.control.motor_torque_nm, intent_res.control.motor_limit_nm);
    } else if (intent_res.control.has_motor_torque_nm != 0) {
        tq_req = intent_res.control.motor_torque_nm;
    } else if (intent_res.control.has_motor_limit_nm != 0) {
        tq_req = intent_res.control.motor_limit_nm;
    }

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
        const bike::physics_core::Status ast_st =
            bike::physics_core::assist_step<float, bike::physics_core::GpuToolkit>(
                &cfg->assist,
                &sp3_st->assist,
                assist_sensor_tq,
                crank_rpm,
                speed_mps,
                brakes.motor_braking != 0,
                dt_s,
                has_tq_req,
                tq_req,
                has_drive_shaft,
                cadence_rpm,
                motor_tq,
                err);
        if (ast_st != bike::physics_core::Status::ok) {
            return ast_st;
        }
    }
    sp3_st->last_motor_torque_nm = motor_tq;
    if (cfg->motor_actuator_idx >= 0 && cfg->motor_actuator_idx < idx.nu) {
        ctrl_w[cfg->motor_actuator_idx] =
            gpu_clamp(motor_tq, cfg->motor_ctrl_min_nm, cfg->motor_ctrl_max_nm);
    } else if (idx.drive_shaft_spin_dof >= 0) {
        qfrc_w[idx.drive_shaft_spin_dof] += motor_tq;
    }

    // 7. Suspension, tyres, resistance, and rider posture/envelope
    const GpuSuspensionParams susp_p = cfg->suspension;
    const bike::physics_core::Status susp_st =
        gpu_write_suspension_qfrc(susp_p, idx, qpos_w, qvel_w, qfrc_w);
    if (susp_st != bike::physics_core::Status::ok) {
        return susp_st;
    }

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

    const bike::physics_core::ResistanceParams<float> res_p = cfg->resistance;
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

    GpuRiderParams rider_p = cfg->rider;
    rider_p.rider_enabled = intent_res.control.rider_enabled;
    if (intent_res.control.has_posture != 0) {
        rider_p.target_qpos_rad[4] =
            cfg->rider.target_qpos_rad[4] + intent_res.control.posture.torso_lean_rad;
    }
    return gpu_write_rider_qfrc_and_ctrl(rider_p, idx, qpos_w, qvel_w, qfrc_w, ctrl_w);
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
    class PtrDim>
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
    PtrDim contact_dim
) noexcept {
    if (w < 0 || cfg == nullptr || sp3_st == nullptr || !gpu_validate_model_indices(idx) ||
        qpos_w == nullptr || qvel_w == nullptr || xmat_w == nullptr) {
        return bike::physics_core::Status::invalid_argument;
    }

    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;

    // 1. Scan per-world efc rows (resolve active cassette tendon id dynamically when ntendon >= cog_count)
    bike::physics_core::EfcScanConfig<float> efc_cfg = cfg->efc_scan;
    if (sp3_st->using_cassette_tendons != 0 &&
        sp3_st->shifting.active_cog_idx >= 0 &&
        sp3_st->shifting.active_cog_idx < cfg->shifting.cog_count &&
        efc_cfg.freehub_tendon_id >= 0) {
        const int cog_tid = cfg->shifting.cassette_tendon_ids[sp3_st->shifting.active_cog_idx];
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

    // 6. Synchronized 2*pi epoch re-basing whenever |qpos[wheel/shaft/crank]| > 256.0f
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
        bike::physics_core::rebase_pedaling_2pi<float>(ped_tmp, dq_crank, dq_rw);
        sp3_st->pedaling = ped_tmp;
    }

    return bike::physics_core::Status::ok;
}

} // namespace bike::gpu

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
