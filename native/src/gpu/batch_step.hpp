// native/src/gpu/batch_step.hpp — Unified batched C++/CUDA/MSL per-world physics
// force writer step wiring all 5 single-source bike::physics_core force laws
// (suspension, resistance, brake_hold, joint_envelope, ratchet) alongside
// bike::physics_core single-source tyre contact (compliant_2d + distributed_2d_reference)
// and 9-DOF upper-body posture PD control.
#pragma once

#include "../physics_core/brake_hold.hpp"
#include "../physics_core/gpu_math.hpp"
#include "../physics_core/joint_envelope.hpp"
#include "../physics_core/ratchet.hpp"
#include "../physics_core/resistance.hpp"
#include "../physics_core/suspension.hpp"
#include "../physics_core/tyre_gpu_step.hpp"
#include "common.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: MSL has no std::span/std::array and address-space-qualified raw pointers are the device ABI; positional float signatures mirror the CUDA/MSL kernels. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::gpu {

struct GpuSuspensionParams {
    bike::physics_core::SuspensionParams<float> params{};
    float fork_sag_fraction = 0.20f;
    float shock_sag_fraction = 0.30f;
    float fork_preload_n = 0.0f;
    float shock_preload_n = 0.0f;
};

struct GpuDrivetrainParams {
    float chainring_teeth = 22.0f;
    float cog_teeth = 51.0f;
    float freehub_stiffness_nm_rad = 850.0f;
    float pawl_engage_rad = 0.087266f;
    // Rider cadence taper aligned with pedaling resume_below_rpm = 105, coast_above_rpm = 120.
    float max_rider_cadence_rad_s = 12.566371f;
    float target_cadence_rad_s = 10.995574f;
    float nominal_crank_torque_nm = 45.0f;
    float front_max_brake_torque_nm = 420.0f;
    float rear_max_brake_torque_nm = 380.0f;
    float brake_taper_rad_s = 0.5f;
    float static_brake_ceiling_nm = 200.0f;
    bike::physics_core::BrakeHoldParams<float> brake_hold{
        .torque_nm = 200.0f,
        .stiffness_nm_rad = 20000.0f,
        .damping_nms_rad = 10.0f,
        .engage_radps = 0.5f,
        .release_radps = 1.0f,
    };
};

struct GpuDrivetrainState {
    bike::physics_core::RatchetBoundState<float> freehub{};
    bike::physics_core::RatchetBoundState<float> crank_clutch{};
    bike::physics_core::u8 front_brake_latched = 0;
    bike::physics_core::u8 rear_brake_latched = 0;
    bike::physics_core::u8 pad[2]{};
    float front_brake_latch_theta = 0.0f;
    float rear_brake_latch_theta = 0.0f;
    float crank_target_phase_rad = 0.0f;
};

struct GpuDrivetrainOutput {
    float crank_torque_nm = 0.0f;
    float rear_drive_torque_nm = 0.0f;
    float front_brake_torque_nm = 0.0f;
    float rear_brake_torque_nm = 0.0f;
    float tendon_upper_bound_rad = 0.0f;
    float crank_clutch_upper_bound_rad = 0.0f;
    bike::physics_core::u8 freehub_engaged = 0;
    bike::physics_core::u8 crank_clutch_engaged = 0;
    bike::physics_core::u8 front_brake_latched = 0;
    bike::physics_core::u8 rear_brake_latched = 0;
};

struct GpuRiderParams {
    bike::physics_core::RiderJointEnvelopeSpec<float> envelope{};
    float kp_posture[kNumRiderJoints] = {
        0.0f, 0.0f, 0.0f, 0.0f, 180.0f, 95.0f, 75.0f, 95.0f, 75.0f
    };
    float kd_posture[kNumRiderJoints] = {
        0.0f, 0.0f, 0.0f, 0.0f, 18.0f, 9.5f, 7.5f, 9.5f, 7.5f
    };
    float target_qpos_rad[kNumRiderJoints] = {
        -0.02f, 0.05f, -0.23f, -0.25f, -0.14185f, 0.08543f, -0.14391f, 0.08543f, -0.14391f
    };
    int rider_enabled = 1;
};

struct GpuWorldConfig {
    GpuSuspensionParams suspension{};
    float tire_stiff_scale = 1.0f;
    bike::physics_core::ResistanceParams<float> resistance{};
    GpuDrivetrainParams drivetrain{};
    GpuRiderParams rider{};
    float pedal_demand = 0.65f;
    float front_brake_demand = 0.0f;
    float rear_brake_demand = 0.0f;
};

struct GpuWorldState {
    bike::physics_core::GpuCompliantWheelState compliant_wheels[2]{};
    bike::physics_core::GpuStationState distributed_wheels[2][bike::physics_core::kMaxGpuTyreStations]{};
    GpuDrivetrainState drivetrain{};
};

// Recomputes static sag preloads using single-source physics_core::eval_suspension.
BIKE_HD void gpu_init_suspension_preloads(BIKE_THREAD GpuSuspensionParams &p) noexcept {
    const float fork_sag_m =
        (p.params.air_spring.total_travel_mm * 1e-3f) * p.fork_sag_fraction;
    const float shock_sag_m =
        (p.params.coil.stroke_mm * 1e-3f) * p.shock_sag_fraction;
    bike::physics_core::SuspensionResult<float> res{};
    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    if (bike::physics_core::eval_suspension<float, bike::physics_core::GpuToolkit>(
            &p.params, fork_sag_m, 0.0f, shock_sag_m, 0.0f, res, err) ==
        bike::physics_core::Status::ok) {
        p.fork_preload_n = res.fork_spring;
        p.shock_preload_n = res.shock_spring;
    } else {
        p.fork_preload_n = 0.0f;
        p.shock_preload_n = 0.0f;
    }
}

BIKE_HD GpuSuspensionParams gpu_make_default_suspension_params() noexcept {
    GpuSuspensionParams out{};
    out.params.air_spring = {
        .stanchion_inner_diam_mm = 24.0f,
        .total_travel_mm = 180.0f,
        .pos_chamber_length_mm = 280.0f,
        .neg_chamber_length_mm = 55.0f,
        .token_volume_cm3 = 8.0f,
        .max_tokens = 4,
        .gamma = 1.40f,
        .atm_pressure_pa = 101325.0f,
        .num_tokens = 2,
        .gauge_pressure_psi = 85.2f,
    };
    out.params.fork_damper = {
        .core = {
            .max_hsc = 4,
            .max_lsc = 14,
            .max_reb = 17,
            .hsc_clicks = 2,
            .lsc_clicks = 7,
            .rebound_clicks = 9,
            .c_lsc_min = 260.0f,
            .c_lsc_max = 1150.0f,
            .c_hsc_min = 180.0f,
            .c_hsc_max = 520.0f,
            .c_reb_min = 350.0f,
            .c_reb_max = 1550.0f,
            .v_knee_comp = 0.18f,
            .v_knee_reb = 0.22f,
        },
        .total_travel_mm = 180.0f,
        .hbo_start_mm = 160.0f,
        .c_hbo_base = 3500.0f,
    };
    out.params.shock_damper = {
        .core = {
            .max_hsc = 4,
            .max_lsc = 14,
            .max_reb = 14,
            .hsc_clicks = 2,
            .lsc_clicks = 7,
            .rebound_clicks = 7,
            .c_lsc_min = 650.0f,
            .c_lsc_max = 2900.0f,
            .c_hsc_min = 450.0f,
            .c_hsc_max = 1450.0f,
            .c_reb_min = 900.0f,
            .c_reb_max = 8000.0f,
            .v_knee_comp = 0.16f,
            .v_knee_reb = 0.20f,
        },
        .total_stroke_mm = 65.0f,
        .max_hbo = 4,
        .hbo_clicks = 2,
        .lockout_firm = 0,
        .hbo_start_mm = 52.0f,
        .c_hbo_min = 4000.0f,
        .c_hbo_max = 18000.0f,
        .lockout_preload_n = 480.0f,
        .lockout_stiffness = 15000.0f,
    };
    out.params.coil = {
        .rate_n_m = 114600.0f,
        .preload_mm = 0.0f,
        .stroke_mm = 65.0f,
        .bumper_length_mm = 10.0f,
        .bumper_peak_n = 7000.0f,
    };
    out.params.end_stops = {
        .stiffness_n_m = 250000.0f,
        .damping_n_s_m = 2000.0f,
    };
    out.fork_sag_fraction = 0.20f;
    out.shock_sag_fraction = 0.30f;
    gpu_init_suspension_preloads(out);
    return out;
}

BIKE_HD bike::physics_core::ResistanceParams<float> gpu_make_default_resistance_params() noexcept {
    bike::physics_core::ResistanceParams<float> rp{};
    rp.crr = 0.008f;
    rp.rolling_taper_rad_s = 0.25f;
    rp.rho_kg_m3 = 1.225f;
    rp.cda_m2 = 0.38f;
    return rp;
}

BIKE_HD GpuRiderParams gpu_make_default_rider_params() noexcept {
    GpuRiderParams rp{};
    const GpuModelIndices default_idx{};
    const float lower_rad[kNumRiderJoints] = {
        -1.326989f, -1.108838f, -1.621450f, -1.420899f,
        -1.232562f, -1.773614f, -2.379773f, -1.773614f, -2.379773f
    };
    const float upper_rad[kNumRiderJoints] = {
        1.383011f, 1.331162f, 1.088550f, 1.019101f,
        0.567438f, 2.056386f, 0.240227f, 2.056386f, 0.240227f
    };
    rp.envelope.count = kNumRiderJoints;
    rp.envelope.stiffness_nm_rad = 100.0f;
    rp.envelope.margin_rad = 0.1f;
    for (int j = 0; j < kNumRiderJoints; ++j) {
        rp.envelope.qpos_adrs[j] = default_idx.rider_qpos[j];
        rp.envelope.dof_adrs[j] = default_idx.rider_dof[j];
        rp.envelope.lower[j] = lower_rad[j];
        rp.envelope.upper[j] = upper_rad[j];
    }
    return rp;
}

BIKE_HD float gpu_drivetrain_ratio(const BIKE_THREAD GpuDrivetrainParams &dp) noexcept {
    return dp.cog_teeth > 0.0f ? (dp.chainring_teeth / dp.cog_teeth) : 1.0f;
}

// Initializes default per-world configuration and precomputes static sag preloads.
BIKE_HD void gpu_init_default_world_config(BIKE_THREAD GpuWorldConfig &cfg) noexcept {
    cfg = GpuWorldConfig{};
    cfg.suspension = gpu_make_default_suspension_params();
    cfg.resistance = gpu_make_default_resistance_params();
    cfg.drivetrain = GpuDrivetrainParams{};
    cfg.rider = gpu_make_default_rider_params();
}

// Populates an array of `nworld` heterogeneous world configurations by scaling
// fork air pressure, damper rebound, rear shock spring rate, and tire carcass
// stiffness. Returns 0 on success or -1 on invalid arguments.
BIKE_HD int gpu_build_sweep_configs(
    BIKE_THREAD GpuWorldConfig *configs,
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
        GpuWorldConfig cfg{};
        gpu_init_default_world_config(cfg);

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

// Resets drivetrain ratchet boundaries and brake latches from initial qpos.
BIKE_HD bike::physics_core::Status gpu_reset_drivetrain_state(
    const BIKE_THREAD GpuDrivetrainParams &dp,
    BIKE_THREAD GpuDrivetrainState &st,
    float crank_qpos_rad,
    float rear_wheel_qpos_rad,
    float drive_shaft_qpos_rad = 0.0f,
    bool has_drive_shaft = false
) noexcept {
    st = GpuDrivetrainState{};
    const float ratio = gpu_drivetrain_ratio(dp);
    const float fh_driver_qpos = has_drive_shaft ? drive_shaft_qpos_rad : crank_qpos_rad;

    bool fh_has = false;
    float fh_bound = 0.0f;
    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    const bike::physics_core::Status fh_st =
        bike::physics_core::ratchet_reset_bound<float, bike::physics_core::GpuToolkit>(
            ratio, fh_driver_qpos, rear_wheel_qpos_rad, fh_has, fh_bound, err);
    if (fh_st != bike::physics_core::Status::ok) {
        return fh_st;
    }
    st.freehub.ratio = ratio;
    st.freehub.has_boundary = fh_has ? 1 : 0;
    st.freehub.boundary = fh_bound;

    if (has_drive_shaft) {
        bool cc_has = false;
        float cc_bound = 0.0f;
        const bike::physics_core::Status cc_st =
            bike::physics_core::ratchet_reset_bound<float, bike::physics_core::GpuToolkit>(
                1.0f, crank_qpos_rad, drive_shaft_qpos_rad, cc_has, cc_bound, err);
        if (cc_st != bike::physics_core::Status::ok) {
            return cc_st;
        }
        st.crank_clutch.ratio = 1.0f;
        st.crank_clutch.has_boundary = cc_has ? 1 : 0;
        st.crank_clutch.boundary = cc_bound;
    }

    st.crank_target_phase_rad = crank_qpos_rad;
    return bike::physics_core::Status::ok;
}

// Zeroes compliant + distributed tyre states and initializes the drivetrain ratchet boundaries.
BIKE_HD bike::physics_core::Status gpu_init_world_state(
    BIKE_REF(GpuWorldState) st,
    const BIKE_THREAD GpuDrivetrainParams &dp,
    float crank_qpos_rad,
    float rear_wheel_qpos_rad,
    float drive_shaft_qpos_rad = 0.0f,
    bool has_drive_shaft = false
) noexcept {
    st.compliant_wheels[0] = bike::physics_core::GpuCompliantWheelState{};
    st.compliant_wheels[1] = bike::physics_core::GpuCompliantWheelState{};
    for (BIKE_THREAD auto &side_states : st.distributed_wheels) {
        for (BIKE_THREAD auto &slot : side_states) {
            slot = bike::physics_core::GpuStationState{};
        }
    }
    GpuDrivetrainState local_dt{};
    const bike::physics_core::Status dt_st = gpu_reset_drivetrain_state(
        dp, local_dt, crank_qpos_rad, rear_wheel_qpos_rad,
        drive_shaft_qpos_rad, has_drive_shaft);
    st.drivetrain = local_dt;
    return dt_st;
}

// Evaluates suspension force laws via single-source physics_core::eval_suspension
// and accumulates generalized forces into qfrc.
BIKE_HD bike::physics_core::Status gpu_write_suspension_qfrc(
    const BIKE_THREAD GpuSuspensionParams &p,
    const BIKE_THREAD GpuModelIndices &idx,
    const BIKE_THREAD float *qpos,
    const BIKE_THREAD float *qvel,
    BIKE_THREAD float *qfrc,
    BIKE_THREAD bike::physics_core::SuspensionResult<float> *out_res = nullptr
) noexcept {
    const float fork_qpos_m = (idx.fork_travel_qpos >= 0) ? qpos[idx.fork_travel_qpos] : 0.0f;
    const float fork_qvel_mps = (idx.fork_travel_dof >= 0) ? qvel[idx.fork_travel_dof] : 0.0f;
    const float shock_qpos_m = (idx.shock_stroke_qpos >= 0) ? qpos[idx.shock_stroke_qpos] : 0.0f;
    const float shock_qvel_mps = (idx.shock_stroke_dof >= 0) ? qvel[idx.shock_stroke_dof] : 0.0f;

    bike::physics_core::SuspensionResult<float> res{};
    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    const bike::physics_core::Status st =
        bike::physics_core::eval_suspension<float, bike::physics_core::GpuToolkit>(
            &p.params, fork_qpos_m, fork_qvel_mps, shock_qpos_m, shock_qvel_mps, res, err);
    if (st != bike::physics_core::Status::ok) {
        return st;
    }

    const float fork_total_n = res.fork_spring + res.fork_damper;
    if (idx.fork_travel_dof >= 0) {
        qfrc[idx.fork_travel_dof] += -fork_total_n;
    }
    if (idx.shock_stroke_dof >= 0) {
        qfrc[idx.shock_stroke_dof] += -res.shock_total;
    }
    if (out_res != nullptr) {
        *out_res = res;
    }
    return bike::physics_core::Status::ok;
}

// Evaluates rolling resistance and planar X-Z aerodynamic drag via single-source
// physics_core::eval_resistance and accumulates generalized forces into qfrc.
template <class XposPtr, class XmatPtr, class XanchorPtr, class XaxisPtr>
BIKE_HD bike::physics_core::Status gpu_write_resistance_qfrc(
    const BIKE_THREAD bike::physics_core::ResistanceParams<float> &rp,
    BIKE_DEVICE const bike::physics_core::GpuTyreModel *tyre_model,
    const BIKE_THREAD GpuModelIndices &idx,
    const BIKE_THREAD float *qvel,
    XposPtr xpos_w,
    XmatPtr xmat_w,
    XanchorPtr xanchor_w,
    XaxisPtr xaxis_w,
    float front_normal_load_n,
    float rear_normal_load_n,
    float front_eff_radius_m,
    float rear_eff_radius_m,
    BIKE_THREAD float *qfrc
) noexcept {
    if (tyre_model == nullptr || qvel == nullptr || qfrc == nullptr ||
        idx.nv <= 0 || idx.nv > bike::physics_core::kMaxNv) {
        return bike::physics_core::Status::invalid_argument;
    }

    const float front_loads[1] = {front_normal_load_n};
    const bike::physics_core::u8 front_work[1] = {1u};
    const float rear_loads[1] = {rear_normal_load_n};
    const bike::physics_core::u8 rear_work[1] = {1u};

    const bike::physics_core::GpuWheelChain wheel_chains[2] = {
        tyre_model->wheels[0].chain,
        tyre_model->wheels[1].chain
    };
    const bike::physics_core::GpuWheelChain frame_chain = tyre_model->frame_chain;

    const bike::physics_core::ChainWheelJrFn<
        float,
        bike::physics_core::GpuToolkit,
        XaxisPtr,
        const BIKE_THREAD bike::physics_core::GpuWheelChain *>
        wheel_jr_fn{wheel_chains, xaxis_w, idx.nv, tyre_model->njnt};
    const bike::physics_core::ChainFrameJpFn<
        float,
        bike::physics_core::GpuToolkit,
        XposPtr,
        XanchorPtr,
        XaxisPtr>
        frame_jp_fn{frame_chain, xpos_w, xanchor_w, xaxis_w, idx.nv, tyre_model->njnt};

    float jr_scratch[3 * bike::physics_core::kMaxNv]{};
    float jp_scratch[3 * bike::physics_core::kMaxNv]{};
    float row_scratch[bike::physics_core::kMaxNv]{};
    float rolling_out[bike::physics_core::kMaxNv]{};
    float aero_out[bike::physics_core::kMaxNv]{};

    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    bike::physics_core::ResDiagField diag = bike::physics_core::ResDiagField::none;
    const bike::physics_core::Status st =
        bike::physics_core::eval_resistance<float, bike::physics_core::GpuToolkit>(
            idx.nv,
            &rp,
            qvel,
            front_loads,
            front_work,
            1,
            front_eff_radius_m,
            xpos_w + idx.front_wheel_body * 3,
            xmat_w + idx.front_wheel_body * 9,
            rear_loads,
            rear_work,
            1,
            rear_eff_radius_m,
            xpos_w + idx.rear_wheel_body * 3,
            xmat_w + idx.rear_wheel_body * 9,
            xpos_w + idx.frame_body * 3,
            xmat_w + idx.frame_body * 9,
            jr_scratch,
            jp_scratch,
            row_scratch,
            rolling_out,
            aero_out,
            wheel_jr_fn,
            frame_jp_fn,
            err,
            diag);
    if (st != bike::physics_core::Status::ok) {
        return st;
    }

    for (int i = 0; i < idx.nv; ++i) {
        qfrc[i] += rolling_out[i] + aero_out[i];
    }
    return bike::physics_core::Status::ok;
}

// Evaluates brake hold/dynamic torque on one wheel via single-source physics_core::eval_brake_hold_wheel.
BIKE_HD bike::physics_core::Status gpu_eval_wheel_brake_torque(
    const BIKE_THREAD GpuDrivetrainParams &dp,
    float demand,
    float max_brake_torque_nm,
    float theta_rad,
    float omega_rad_s,
    bool has_static_frictionloss,
    BIKE_THREAD bike::physics_core::u8 &inout_latched,
    BIKE_THREAD float &inout_latch_theta,
    BIKE_THREAD float &out_torque_nm
) noexcept {
    out_torque_nm = 0.0f;
    const float u = gpu_clamp(demand, 0.0f, 1.0f);
    bool latched = (inout_latched != 0);
    float latch_theta = inout_latch_theta;
    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    float hold_torque = 0.0f;

    const bike::physics_core::Status st =
        bike::physics_core::eval_brake_hold_wheel<float, bike::physics_core::GpuToolkit>(
            &dp.brake_hold,
            u,
            theta_rad,
            omega_rad_s,
            true,
            false,
            latched,
            latch_theta,
            hold_torque,
            err);
    if (st != bike::physics_core::Status::ok) {
        return st;
    }

    inout_latched = latched ? 1 : 0;
    inout_latch_theta = latch_theta;

    if (latched) {
        out_torque_nm = hold_torque;
    } else if (!has_static_frictionloss &&
               u > 0.0f &&
               omega_rad_s != 0.0f &&
               dp.brake_taper_rad_s > 0.0f &&
               max_brake_torque_nm > 0.0f) {
        out_torque_nm =
            -(u * max_brake_torque_nm) *
            bike::physics_core::tanhf_portable(omega_rad_s / dp.brake_taper_rad_s);
    }
    return bike::physics_core::Status::ok;
}

// Evaluates rider pedaling torque, dual ratchet bounds (freehub + crank_clutch),
// and latched + dynamic braking via single-source physics_core::ratchet + brake_hold.
BIKE_HD bike::physics_core::Status gpu_write_drivetrain_qfrc(
    const BIKE_THREAD GpuDrivetrainParams &dp,
    const BIKE_THREAD GpuModelIndices &idx,
    BIKE_THREAD GpuDrivetrainState &dt_st,
    const BIKE_THREAD float *qpos,
    const BIKE_THREAD float *qvel,
    float pedal_demand,
    float front_brake_demand,
    float rear_brake_demand,
    BIKE_THREAD float *qfrc,
    BIKE_THREAD GpuDrivetrainOutput &out,
    bool include_rear_drive_torque,
    BIKE_THREAD float *dof_frictionloss = nullptr
) noexcept {
    out = GpuDrivetrainOutput{};
    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;

    // 1. Crank pedaling torque with braking cutout gate (simulation-decisions.md §16)
    //    and cadence linear taper above target_cadence_rad_s.
    const bool braking =
        (front_brake_demand > 1e-6f) || (rear_brake_demand > 1e-6f);
    const float effective_pedal_demand = braking ? 0.0f : gpu_clamp(pedal_demand, 0.0f, 1.0f);
    const float cadence_omega = (idx.crank_spin_dof >= 0) ? qvel[idx.crank_spin_dof] : 0.0f;
    float taper = 1.0f;
    if (dp.max_rider_cadence_rad_s > dp.target_cadence_rad_s &&
        cadence_omega > dp.target_cadence_rad_s) {
        const float frac =
            (cadence_omega - dp.target_cadence_rad_s) /
            (dp.max_rider_cadence_rad_s - dp.target_cadence_rad_s);
        taper = gpu_clamp(1.0f - frac, 0.0f, 1.0f);
    }
    out.crank_torque_nm = effective_pedal_demand * dp.nominal_crank_torque_nm * taper;
    if (idx.crank_spin_dof >= 0) {
        qfrc[idx.crank_spin_dof] += out.crank_torque_nm;
    }

    // 2. Dual ratchet bounds via single-source physics_core::ratchet_prepare_bound.
    const float ratio = gpu_drivetrain_ratio(dp);
    const bool has_drive_shaft = (idx.drive_shaft_spin_qpos >= 0);
    const float crank_qpos = (idx.crank_spin_qpos >= 0) ? qpos[idx.crank_spin_qpos] : 0.0f;
    const float rw_qpos = (idx.rear_wheel_spin_qpos >= 0) ? qpos[idx.rear_wheel_spin_qpos] : 0.0f;
    const float ds_qpos = has_drive_shaft ? qpos[idx.drive_shaft_spin_qpos] : crank_qpos;

    bool fh_has = (dt_st.freehub.has_boundary != 0);
    float fh_bound = dt_st.freehub.boundary;
    const float prev_fh_bound = fh_has ? fh_bound : (ratio * ds_qpos - rw_qpos);
    const float fh_phi = bike::physics_core::ratchet_phi<float, bike::physics_core::GpuToolkit>(
        ratio, ds_qpos, rw_qpos);
    const bike::physics_core::Status fh_st =
        bike::physics_core::ratchet_prepare_bound<float, bike::physics_core::GpuToolkit>(
            ratio, ds_qpos, rw_qpos, fh_has, fh_bound, err);
    if (fh_st != bike::physics_core::Status::ok) {
        return fh_st;
    }
    out.freehub_engaged = (dt_st.freehub.has_boundary != 0 && fh_phi > prev_fh_bound) ? 1 : 0;
    dt_st.freehub.ratio = ratio;
    dt_st.freehub.has_boundary = fh_has ? 1 : 0;
    dt_st.freehub.boundary = fh_bound;
    out.tendon_upper_bound_rad = fh_bound;

    if (has_drive_shaft) {
        bool cc_has = (dt_st.crank_clutch.has_boundary != 0);
        float cc_bound = dt_st.crank_clutch.boundary;
        const float prev_cc_bound = cc_has ? cc_bound : (crank_qpos - ds_qpos);
        const float cc_phi = bike::physics_core::ratchet_phi<float, bike::physics_core::GpuToolkit>(
            1.0f, crank_qpos, ds_qpos);
        const bike::physics_core::Status cc_st =
            bike::physics_core::ratchet_prepare_bound<float, bike::physics_core::GpuToolkit>(
                1.0f, crank_qpos, ds_qpos, cc_has, cc_bound, err);
        if (cc_st != bike::physics_core::Status::ok) {
            return cc_st;
        }
        out.crank_clutch_engaged =
            (dt_st.crank_clutch.has_boundary != 0 && cc_phi > prev_cc_bound) ? 1 : 0;
        dt_st.crank_clutch.ratio = 1.0f;
        dt_st.crank_clutch.has_boundary = cc_has ? 1 : 0;
        dt_st.crank_clutch.boundary = cc_bound;
        out.crank_clutch_upper_bound_rad = cc_bound;
    }

    if (include_rear_drive_torque && out.freehub_engaged != 0 && ratio > 0.0f) {
        const float engage_rad = fh_phi - prev_fh_bound;
        const float pawl_reaction_nm = dp.freehub_stiffness_nm_rad * engage_rad;
        out.rear_drive_torque_nm = pawl_reaction_nm + out.crank_torque_nm / ratio;
        if (idx.rear_wheel_spin_dof >= 0) {
            qfrc[idx.rear_wheel_spin_dof] += out.rear_drive_torque_nm;
        }
        const int fh_driver_dof = has_drive_shaft ? idx.drive_shaft_spin_dof : idx.crank_spin_dof;
        if (fh_driver_dof >= 0) {
            qfrc[fh_driver_dof] -= ratio * pawl_reaction_nm;
        }
    }

    // 3. Brake hold (latched standstill bite) + optional static frictionloss + dynamic opposing torque.
    const bool has_static_fl = (dof_frictionloss != nullptr);
    if (has_static_fl) {
        bool fl_changed = false;
        const bike::physics_core::Status fl_st =
            bike::physics_core::apply_static_brake<float, bike::physics_core::GpuToolkit>(
                dp.static_brake_ceiling_nm,
                front_brake_demand,
                rear_brake_demand,
                idx.front_wheel_spin_dof,
                idx.rear_wheel_spin_dof,
                idx.nv,
                dof_frictionloss,
                fl_changed,
                err);
        if (fl_st != bike::physics_core::Status::ok) {
            return fl_st;
        }
    }

    const float fw_qpos = (idx.front_wheel_spin_qpos >= 0) ? qpos[idx.front_wheel_spin_qpos] : 0.0f;
    const float fw_omega = (idx.front_wheel_spin_dof >= 0) ? qvel[idx.front_wheel_spin_dof] : 0.0f;
    const float rw_omega = (idx.rear_wheel_spin_dof >= 0) ? qvel[idx.rear_wheel_spin_dof] : 0.0f;

    const bike::physics_core::Status fb_st = gpu_eval_wheel_brake_torque(
        dp,
        front_brake_demand,
        dp.front_max_brake_torque_nm,
        fw_qpos,
        fw_omega,
        has_static_fl,
        dt_st.front_brake_latched,
        dt_st.front_brake_latch_theta,
        out.front_brake_torque_nm);
    if (fb_st != bike::physics_core::Status::ok) {
        return fb_st;
    }
    const bike::physics_core::Status rb_st = gpu_eval_wheel_brake_torque(
        dp,
        rear_brake_demand,
        dp.rear_max_brake_torque_nm,
        rw_qpos,
        rw_omega,
        has_static_fl,
        dt_st.rear_brake_latched,
        dt_st.rear_brake_latch_theta,
        out.rear_brake_torque_nm);
    if (rb_st != bike::physics_core::Status::ok) {
        return rb_st;
    }

    out.front_brake_latched = dt_st.front_brake_latched;
    out.rear_brake_latched = dt_st.rear_brake_latched;

    if (idx.front_wheel_spin_dof >= 0) {
        qfrc[idx.front_wheel_spin_dof] += out.front_brake_torque_nm;
    }
    if (idx.rear_wheel_spin_dof >= 0) {
        qfrc[idx.rear_wheel_spin_dof] += out.rear_brake_torque_nm;
    }

    return bike::physics_core::Status::ok;
}

// Evaluates 9-DOF rider ROM soft-edge envelope torques via single-source
// physics_core::eval_rider_joint_envelope and upper-body posture PD ctrl.
BIKE_HD bike::physics_core::Status gpu_write_rider_qfrc_and_ctrl(
    const BIKE_THREAD GpuRiderParams &rp,
    const BIKE_THREAD GpuModelIndices &idx,
    const BIKE_THREAD float *qpos,
    const BIKE_THREAD float *qvel,
    BIKE_THREAD float *qfrc,
    BIKE_THREAD float *ctrl,
    BIKE_THREAD float *out_envelope_energy_j = nullptr
) noexcept {
    if (rp.rider_enabled == 0) {
        if (out_envelope_energy_j != nullptr) {
            *out_envelope_energy_j = 0.0f;
        }
        return bike::physics_core::Status::ok;
    }

    bike::physics_core::RiderJointEnvelopeSpec<float> active_env = rp.envelope;
    if (rp.envelope.count == kNumRiderJoints) {
        int env_out = 0;
        for (int j = 0; j < kNumRiderJoints; ++j) {
            const int qp = idx.rider_qpos[j];
            const int dof = idx.rider_dof[j];
            if (qp < 0 || dof < 0) {
                continue;
            }
            active_env.qpos_adrs[env_out] = qp;
            active_env.dof_adrs[env_out] = dof;
            active_env.lower[env_out] = rp.envelope.lower[j];
            active_env.upper[env_out] = rp.envelope.upper[j];
            ++env_out;
        }
        active_env.count = env_out;
    }

    float env_energy = 0.0f;
    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    const bike::physics_core::Status st =
        bike::physics_core::eval_rider_joint_envelope<float, bike::physics_core::GpuToolkit>(
            &active_env, idx.nq, idx.nv, qpos, qfrc, env_energy, err);
    if (st != bike::physics_core::Status::ok) {
        return st;
    }

    for (int j = 0; j < kNumRiderJoints; ++j) {
        const int qp = idx.rider_qpos[j];
        const int dof = idx.rider_dof[j];
        const int ci = idx.rider_ctrl[j];
        if (qp < 0 || dof < 0 || ci < 0) {
            continue;
        }
        // Legs (j = 0..3) are passive followers (kp = kd = 0 -> ctrl = 0).
        // Upper body (j = 4..8) receives posture PD torque in ctrl[].
        const float kp = rp.kp_posture[j];
        const float kd = rp.kd_posture[j];
        const float pd_ctrl = (kp == 0.0f && kd == 0.0f)
            ? 0.0f
            : (kp * (rp.target_qpos_rad[j] - qpos[qp]) - kd * qvel[dof]);
        ctrl[ci] = pd_ctrl;
    }

    if (out_envelope_energy_j != nullptr) {
        *out_envelope_energy_j = env_energy;
    }
    return bike::physics_core::Status::ok;
}

template <class TendonRangePtr>
BIKE_HD void gpu_write_tendon_upper_bound(
    int w,
    int tendon_idx,
    float upper_bound_rad,
    TendonRangePtr tendon_range_batch,
    int tendon_range_world_stride
) noexcept {
    if (tendon_range_batch == nullptr || tendon_idx < 0) {
        return;
    }
    const long w64 = static_cast<long>(w);
    const long slot =
        static_cast<long>(2) * static_cast<long>(tendon_idx) + static_cast<long>(1);
    if (tendon_range_world_stride > 0) {
        if (slot < static_cast<long>(tendon_range_world_stride)) {
            tendon_range_batch[w64 * static_cast<long>(tendon_range_world_stride) + slot] =
                upper_bound_rad;
        }
    } else if (w == 0) {
        tendon_range_batch[slot] = upper_bound_rad;
    }
}

// Single-world writer execution core templated over device-resident kinematics/state pointers
// so MSL can pass `device` buffers directly while keeping qpos/qvel/qfrc/ctrl in `thread` stack.
template <
    class XposPtr,
    class XmatPtr,
    class XanchorPtr,
    class XaxisPtr,
    class CompliantStatePtr,
    class DistStatePtr,
    class TendonRangePtr>
BIKE_HD bike::physics_core::Status gpu_step_world_writers_single(
    int w,
    float dt_s,
    const BIKE_THREAD GpuModelIndices &idx,
    BIKE_DEVICE const bike::physics_core::GpuTyreModel *tyre_model,
    const BIKE_THREAD bike::physics_core::GpuTerrainTableDesc &terrain_desc,
    BIKE_DEVICE const float *terrain_f32,
    BIKE_DEVICE const bike::physics_core::i32 *terrain_i32,
    const BIKE_THREAD GpuWorldConfig &cfg,
    BIKE_THREAD GpuDrivetrainState &drivetrain_st,
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
    TendonRangePtr tendon_range_batch,
    int tendon_range_world_stride = 2,
    bool check_monotonic_terrain = false
) noexcept {
    if (w < 0 || !gpu_validate_model_indices(idx) ||
        qpos_w == nullptr || qvel_w == nullptr ||
        qfrc_w == nullptr || ctrl_w == nullptr) {
        return bike::physics_core::Status::invalid_argument;
    }

    for (int i = 0; i < idx.nv; ++i) {
        qfrc_w[i] = 0.0f;
    }
    for (int i = 0; i < idx.nu; ++i) {
        ctrl_w[i] = 0.0f;
    }

    // 1. Suspension: air spring + Charger3 HBO damper + coil shock + bumper + SuperDeluxe + end stops
    const bike::physics_core::Status susp_st =
        gpu_write_suspension_qfrc(cfg.suspension, idx, qpos_w, qvel_w, qfrc_w);
    if (susp_st != bike::physics_core::Status::ok) {
        return susp_st;
    }

    // 2. Front and rear single-source physics_core tyre contact + bristle friction
    bike::physics_core::GpuTyreStepOutput tyre_out{};
    bike::physics_core::ErrCode tyre_err = bike::physics_core::ErrCode::none;
    const bike::physics_core::Status tyre_st = bike::physics_core::gpu_step_both_wheels_tyre(
        tyre_model,
        terrain_desc,
        terrain_f32,
        terrain_i32,
        cfg.tire_stiff_scale,
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
        check_monotonic_terrain);
    if (tyre_st != bike::physics_core::Status::ok) {
        return tyre_st;
    }

    // 3. Rolling resistance + planar X-Z aerodynamic drag via full body Jacobians
    const bike::physics_core::Status res_st = gpu_write_resistance_qfrc(
        cfg.resistance,
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

    // 4. Drivetrain pedaling, dual ratchets (freehub + crank_clutch), and latched/dynamic brakes
    const bool include_rear_drive_torque =
        (tendon_range_batch == nullptr) || (idx.freehub_tendon < 0);
    GpuDrivetrainOutput dt_out{};
    const bike::physics_core::Status dt_st = gpu_write_drivetrain_qfrc(
        cfg.drivetrain,
        idx,
        drivetrain_st,
        qpos_w,
        qvel_w,
        cfg.pedal_demand,
        cfg.front_brake_demand,
        cfg.rear_brake_demand,
        qfrc_w,
        dt_out,
        include_rear_drive_torque);
    if (dt_st != bike::physics_core::Status::ok) {
        return dt_st;
    }

    // 5. 9-DOF rider ROM soft-edge envelope torques + posture PD ctrl for upper body only
    const bike::physics_core::Status rider_st =
        gpu_write_rider_qfrc_and_ctrl(cfg.rider, idx, qpos_w, qvel_w, qfrc_w, ctrl_w);
    if (rider_st != bike::physics_core::Status::ok) {
        return rider_st;
    }

    // 6. Update freehub and crank_clutch tendon upper bounds for world w
    gpu_write_tendon_upper_bound(
        w, idx.freehub_tendon, dt_out.tendon_upper_bound_rad,
        tendon_range_batch, tendon_range_world_stride);
    if (idx.drive_shaft_spin_qpos >= 0 && idx.crank_clutch_tendon >= 0 &&
        idx.crank_clutch_tendon != idx.freehub_tendon) {
        gpu_write_tendon_upper_bound(
            w, idx.crank_clutch_tendon, dt_out.crank_clutch_upper_bound_rad,
            tendon_range_batch, tendon_range_world_stride);
    }

    return bike::physics_core::Status::ok;
}

// Executes all custom physics force writers for batched world `w` (Host C++ & CUDA).
BIKE_HD bike::physics_core::Status gpu_step_world_writers(
    int w,
    float dt_s,
    const BIKE_THREAD GpuModelIndices &idx,
    const BIKE_DEVICE bike::physics_core::GpuTyreModel *tyre_model,
    const BIKE_THREAD bike::physics_core::GpuTerrainTableDesc &terrain_desc,
    const BIKE_DEVICE float *terrain_f32,
    const BIKE_DEVICE bike::physics_core::i32 *terrain_i32,
    const BIKE_THREAD GpuWorldConfig *configs,
    int per_world_config,
    BIKE_THREAD GpuWorldState *states,
    const BIKE_THREAD float *qpos_batch,
    const BIKE_THREAD float *qvel_batch,
    const BIKE_THREAD float *xpos_batch,
    const BIKE_THREAD float *xmat_batch,
    const BIKE_THREAD float *xanchor_batch,
    const BIKE_THREAD float *xaxis_batch,
    BIKE_THREAD float *qfrc_batch,
    BIKE_THREAD float *ctrl_batch,
    BIKE_THREAD float *tendon_range_batch,
    int tendon_range_world_stride = 2,
    bool check_monotonic_terrain = false
) noexcept {
    if (w < 0 || !gpu_validate_model_indices(idx) ||
        tyre_model == nullptr || configs == nullptr || states == nullptr ||
        qpos_batch == nullptr || qvel_batch == nullptr ||
        xpos_batch == nullptr || xmat_batch == nullptr ||
        xanchor_batch == nullptr || xaxis_batch == nullptr ||
        qfrc_batch == nullptr || ctrl_batch == nullptr) {
        return bike::physics_core::Status::invalid_argument;
    }
    if (tyre_model->nq != idx.nq || tyre_model->nv != idx.nv || tyre_model->nbody != idx.nbody) {
        return bike::physics_core::Status::invalid_argument;
    }

    const BIKE_THREAD GpuWorldConfig &cfg = configs[per_world_config ? w : 0];
    BIKE_THREAD GpuWorldState &st = states[w];

    const long w64 = static_cast<long>(w);
    const BIKE_THREAD float *qpos_w = qpos_batch + w64 * idx.nq;
    const BIKE_THREAD float *qvel_w = qvel_batch + w64 * idx.nv;
    const BIKE_THREAD float *xpos_w = xpos_batch + w64 * idx.nbody * 3;
    const BIKE_THREAD float *xmat_w = xmat_batch + w64 * idx.nbody * 9;
    const BIKE_THREAD float *xanchor_w = xanchor_batch + w64 * tyre_model->njnt * 3;
    const BIKE_THREAD float *xaxis_w = xaxis_batch + w64 * tyre_model->njnt * 3;
    BIKE_THREAD float *qfrc_w = qfrc_batch + w64 * idx.nv;
    BIKE_THREAD float *ctrl_w = ctrl_batch + w64 * idx.nu;

    return gpu_step_world_writers_single(
        w,
        dt_s,
        idx,
        tyre_model,
        terrain_desc,
        terrain_f32,
        terrain_i32,
        cfg,
        st.drivetrain,
        st.compliant_wheels,
        &st.distributed_wheels[0][0],
        qpos_w,
        qvel_w,
        xpos_w,
        xmat_w,
        xanchor_w,
        xaxis_w,
        qfrc_w,
        ctrl_w,
        tendon_range_batch,
        tendon_range_world_stride,
        check_monotonic_terrain);
}

} // namespace bike::gpu

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
