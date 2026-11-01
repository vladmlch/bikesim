// native/src/gpu/batch_step.hpp — Unified batched C++/CUDA per-world physics
// force writer step combining suspension, distributed multi-rib tire contact,
// drivetrain/freehub/resistance/braking, and 6-DOF rider posture/pedaling.
#pragma once

#include "common.hpp"
#include "drivetrain_resistance_kernel.hpp"
#include "rider_kernel.hpp"
#include "suspension_kernel.hpp"
#include "tire_kernel.hpp"

namespace bike::gpu {

struct GpuWorldConfig {
    GpuSuspensionParams suspension{};
    GpuTireParams front_tire{};
    GpuTireParams rear_tire{};
    GpuResistanceParams resistance{};
    GpuDrivetrainParams drivetrain{};
    GpuRiderParams rider{};
    float pedal_demand = 0.65f;
    float front_brake_demand = 0.0f;
    float rear_brake_demand = 0.0f;
};

struct GpuWorldState {
    GpuWheelStationState front_wheel{};
    GpuWheelStationState rear_wheel{};
    GpuDrivetrainState drivetrain{};
};

// Initializes default per-world configuration and precomputes static sag preloads.
BIKE_HD void gpu_init_default_world_config(GpuWorldConfig &cfg) noexcept {
    cfg = GpuWorldConfig{};
    gpu_init_suspension_preloads(cfg.suspension);
}

// Populates an array of `nworld` heterogeneous world configurations by scaling
// fork air pressure, damper rebound, rear shock spring rate, and tire carcass
// stiffness, as well as configuring tire ray grid resolution (stations x lateral ribs).
BIKE_HD void gpu_build_sweep_configs(
    GpuWorldConfig *configs,
    int nworld,
    const float *fork_psi_scale,
    const float *rebound_scale,
    const float *shock_rate_scale,
    const float *tire_stiff_scale,
    int num_stations,
    int num_lateral_ribs
) noexcept {
    if (configs == nullptr || nworld <= 0) {
        return;
    }
    for (int w = 0; w < nworld; ++w) {
        GpuWorldConfig cfg{};
        gpu_init_default_world_config(cfg);

        if (fork_psi_scale != nullptr) {
            cfg.suspension.fork_air.gauge_pressure_psi *= fork_psi_scale[w];
        }
        if (rebound_scale != nullptr) {
            const float r_scale = gpu_max(0.0f, rebound_scale[w]);
            cfg.suspension.fork_damper.core.c_reb_min *= r_scale;
            cfg.suspension.fork_damper.core.c_reb_max *= r_scale;
            cfg.suspension.shock_damper.c_reb_min *= r_scale;
            cfg.suspension.shock_damper.c_reb_max *= r_scale;

            const int fork_max_reb = cfg.suspension.fork_damper.core.max_reb;
            const int fork_reb_scaled = static_cast<int>(
                static_cast<float>(cfg.suspension.fork_damper.core.rebound_clicks) * r_scale + 0.5f
            );
            cfg.suspension.fork_damper.core.rebound_clicks =
                fork_reb_scaled < 0 ? 0 : (fork_reb_scaled > fork_max_reb ? fork_max_reb : fork_reb_scaled);

            const int shock_max_reb = cfg.suspension.shock_damper.max_reb;
            const int shock_reb_scaled = static_cast<int>(
                static_cast<float>(cfg.suspension.shock_damper.rebound_clicks) * r_scale + 0.5f
            );
            cfg.suspension.shock_damper.rebound_clicks =
                shock_reb_scaled < 0 ? 0 : (shock_reb_scaled > shock_max_reb ? shock_max_reb : shock_reb_scaled);
        }
        if (shock_rate_scale != nullptr) {
            cfg.suspension.shock_coil.spring_rate_n_mm *= shock_rate_scale[w];
        }
        if (tire_stiff_scale != nullptr) {
            const float t_scale = gpu_max(0.0f, tire_stiff_scale[w]);
            for (int k = 0; k < kMaxHingeKnots; ++k) {
                cfg.front_tire.density.stiffness_n_m[k] *= t_scale;
                cfg.rear_tire.density.stiffness_n_m[k] *= t_scale;
            }
        }
        if (num_stations > 0) {
            cfg.front_tire.num_stations = num_stations;
            cfg.rear_tire.num_stations = num_stations;
        }
        if (num_lateral_ribs > 0) {
            cfg.front_tire.num_lateral_ribs = num_lateral_ribs;
            cfg.rear_tire.num_lateral_ribs = num_lateral_ribs;
        }

        gpu_init_suspension_preloads(cfg.suspension);
        configs[w] = cfg;
    }
}

// Zeroes tire bristle states and initializes the drivetrain freehub boundary from initial qpos.
BIKE_HD void gpu_init_world_state(
    GpuWorldState &st,
    const GpuDrivetrainParams &dp,
    float crank_qpos_rad,
    float rear_wheel_qpos_rad
) noexcept {
    for (int s = 0; s < kMaxTreadStations; ++s) {
        st.front_wheel.bristle_x[s] = 0.0f;
        st.rear_wheel.bristle_x[s] = 0.0f;
    }
    gpu_reset_drivetrain_state(dp, st.drivetrain, crank_qpos_rad, rear_wheel_qpos_rad);
}

// Executes all custom physics force writers for world `w` in a single fused pass.
BIKE_HD void gpu_step_world_writers(
    int w,
    float dt_s,
    const GpuModelIndices &idx,
    const GpuHeightfield1D &hf,
    const GpuWorldConfig *configs,
    int per_world_config,
    GpuWorldState *states,
    const float *qpos_batch,
    const float *qvel_batch,
    const float *xpos_batch,
    float *qfrc_batch,
    float *ctrl_batch,
    float *tendon_range_batch,
    int tendon_range_world_stride = 2
) noexcept {
    const GpuWorldConfig &cfg = configs[per_world_config ? w : 0];
    GpuWorldState &st = states[w];

    const float *qpos_w = qpos_batch + w * idx.nq;
    const float *qvel_w = qvel_batch + w * idx.nv;
    const float *xpos_w = xpos_batch + w * (idx.nbody * 3);
    float *qfrc_w = qfrc_batch + w * idx.nv;
    float *ctrl_w = ctrl_batch + w * idx.nu;

    for (int i = 0; i < idx.nv; ++i) {
        qfrc_w[i] = 0.0f;
    }
    for (int i = 0; i < idx.nu; ++i) {
        ctrl_w[i] = 0.0f;
    }

    // 1. Fork air spring + Charger3 HBO damper + rear coil shock + bumper + top-out stops
    gpu_write_suspension_qfrc(cfg.suspension, idx, qpos_w, qvel_w, qfrc_w);

    // 2. Front and rear distributed tire / multi-rib ray-grid contact + tangential bristle friction
    GpuWheelContactOutput front_out{};
    GpuWheelContactOutput rear_out{};
    gpu_write_both_wheels_qfrc(
        xpos_w,
        qvel_w,
        dt_s,
        cfg.front_tire,
        cfg.rear_tire,
        hf,
        idx,
        &st.front_wheel,
        &st.rear_wheel,
        qfrc_w,
        &front_out,
        &rear_out
    );

    // 3. Drivetrain freehub ratcheting, pedaling torque, dynamic brakes, rolling & aero drag
    const GpuDrivetrainOutput dt_out = gpu_write_drivetrain_and_resistance_qfrc(
        cfg.resistance,
        cfg.drivetrain,
        idx,
        st.drivetrain,
        qpos_w,
        qvel_w,
        front_out.normal_load_n,
        rear_out.normal_load_n,
        front_out.effective_radius_m,
        rear_out.effective_radius_m,
        cfg.pedal_demand,
        cfg.front_brake_demand,
        cfg.rear_brake_demand,
        qfrc_w
    );

    // 4. 6-DOF rider ROM soft-edge envelope torques + posture PD & phase-locked pedaling actuators
    gpu_write_rider_qfrc_and_ctrl(cfg.rider, idx, qpos_w, qvel_w, qfrc_w, ctrl_w);

    // 5. Update freehub tendon upper bound for world w (or world 0 if tendon_range is shared [1, 1, 2])
    if (tendon_range_batch != nullptr) {
        if (tendon_range_world_stride > 0) {
            tendon_range_batch[w * tendon_range_world_stride + 1] = dt_out.tendon_upper_bound_rad;
        } else if (w == 0) {
            tendon_range_batch[1] = dt_out.tendon_upper_bound_rad;
        }
    }
}

} // namespace bike::gpu
