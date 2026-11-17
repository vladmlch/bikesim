// native/src/gpu/batch_step.hpp — Unified batched C++/CUDA/MSL per-world physics
// force writer step combining suspension, single-source physics_core tyre contact
// (compliant_2d + distributed_2d_reference), drivetrain/freehub/resistance/braking,
// and 9-DOF rider envelope + upper-body posture control.
#pragma once

#include "../physics_core/tyre_gpu_step.hpp"
#include "common.hpp"
#include "drivetrain_resistance_kernel.hpp"
#include "rider_kernel.hpp"
#include "suspension_kernel.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: MSL has no std::span/std::array and address-space-qualified raw pointers are the device ABI; positional float signatures mirror the CUDA/MSL kernels. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::gpu {

struct GpuWorldConfig {
    GpuSuspensionParams suspension{};
    float tire_stiff_scale = 1.0f;
    GpuResistanceParams resistance{};
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

// Initializes default per-world configuration and precomputes static sag preloads.
BIKE_HD void gpu_init_default_world_config(BIKE_THREAD GpuWorldConfig &cfg) noexcept {
    cfg = GpuWorldConfig{};
    gpu_init_suspension_preloads(cfg.suspension);
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
                gpu_floor(static_cast<float>(cfg.suspension.fork_damper.core.rebound_clicks) * r_scale + 0.5f)
            );
            cfg.suspension.fork_damper.core.rebound_clicks =
                fork_reb_scaled < 0 ? 0 : (fork_reb_scaled > fork_max_reb ? fork_max_reb : fork_reb_scaled);

            const int shock_max_reb = cfg.suspension.shock_damper.max_reb;
            const int shock_reb_scaled = static_cast<int>(
                gpu_floor(static_cast<float>(cfg.suspension.shock_damper.rebound_clicks) * r_scale + 0.5f)
            );
            cfg.suspension.shock_damper.rebound_clicks =
                shock_reb_scaled < 0 ? 0 : (shock_reb_scaled > shock_max_reb ? shock_max_reb : shock_reb_scaled);
        }
        if (shock_rate_scale != nullptr) {
            cfg.suspension.shock_coil.spring_rate_n_mm *= shock_rate_scale[w];
        }
        if (tire_stiff_scale != nullptr) {
            cfg.tire_stiff_scale = gpu_max(0.0f, tire_stiff_scale[w]);
        }

        gpu_init_suspension_preloads(cfg.suspension);
        configs[w] = cfg;
    }
    return 0;
}

// Zeroes compliant + distributed tyre states and initializes the drivetrain freehub boundary from initial qpos.
template <class StateRef>
BIKE_HD void gpu_init_world_state(
    StateRef &st,
    const BIKE_THREAD GpuDrivetrainParams &dp,
    float crank_qpos_rad,
    float rear_wheel_qpos_rad
) noexcept {
    st.compliant_wheels[0] = bike::physics_core::GpuCompliantWheelState{};
    st.compliant_wheels[1] = bike::physics_core::GpuCompliantWheelState{};
    for (int side = 0; side < 2; ++side) {
        for (int s = 0; s < static_cast<int>(bike::physics_core::kMaxGpuTyreStations); ++s) {
            st.distributed_wheels[side][s] = bike::physics_core::GpuStationState{};
        }
    }
    GpuDrivetrainState local_dt{};
    gpu_reset_drivetrain_state(dp, local_dt, crank_qpos_rad, rear_wheel_qpos_rad);
    st.drivetrain = local_dt;
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

    // 1. Fork air spring + Charger3 HBO damper + rear coil shock + bumper + top-out stops
    gpu_write_suspension_qfrc(cfg.suspension, idx, qpos_w, qvel_w, qfrc_w);

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

    // 3. Drivetrain freehub ratcheting, pedaling torque, dynamic brakes, rolling & aero drag
    const bool include_rear_drive_torque =
        (tendon_range_batch == nullptr) || (idx.freehub_tendon < 0);
    const GpuDrivetrainOutput dt_out = gpu_write_drivetrain_and_resistance_qfrc(
        cfg.resistance,
        cfg.drivetrain,
        idx,
        drivetrain_st,
        qpos_w,
        qvel_w,
        tyre_out.wheels[0].normal_load_n,
        tyre_out.wheels[1].normal_load_n,
        tyre_out.wheels[0].effective_radius_m,
        tyre_out.wheels[1].effective_radius_m,
        cfg.pedal_demand,
        cfg.front_brake_demand,
        cfg.rear_brake_demand,
        qfrc_w,
        include_rear_drive_torque
    );

    // 4. 9-DOF rider ROM soft-edge envelope torques + posture PD ctrl for upper body only
    gpu_write_rider_qfrc_and_ctrl(cfg.rider, idx, qpos_w, qvel_w, qfrc_w, ctrl_w);

    // 5. Update freehub tendon upper bound for world w (or world 0 if tendon_range is shared [1, ntendon, 2])
    if (tendon_range_batch != nullptr && idx.freehub_tendon >= 0) {
        const long w64 = static_cast<long>(w);
        const long slot =
            static_cast<long>(2) * static_cast<long>(idx.freehub_tendon) + static_cast<long>(1);
        if (tendon_range_world_stride > 0) {
            if (slot < static_cast<long>(tendon_range_world_stride)) {
                tendon_range_batch[w64 * static_cast<long>(tendon_range_world_stride) + slot] =
                    dt_out.tendon_upper_bound_rad;
            }
        } else if (w == 0) {
            tendon_range_batch[slot] = dt_out.tendon_upper_bound_rad;
        }
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
