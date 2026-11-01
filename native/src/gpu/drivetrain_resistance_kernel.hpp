// native/src/gpu/drivetrain_resistance_kernel.hpp — Allocation-free __host__ __device__
// drivetrain, braking, freehub ratcheting, rolling resistance, and aerodynamic drag
// kernels for batched GPU physics rollouts.
#pragma once

#include "common.hpp"

namespace bike::gpu {

struct GpuResistanceParams {
    float crr = 0.008f;
    float rho_kg_m3 = 1.225f;
    float cda_m2 = 0.38f;
    float rolling_taper_rad_s = 0.25f;
    float wind_vx_mps = 0.0f;
    float wind_vz_mps = 0.0f;
    float wheel_bearing_viscous_nm_s_rad = 0.004f;
    float wheel_bearing_coulomb_nm = 0.02f;
    float drivetrain_viscous_nm_s_rad = 0.015f;
};

struct GpuDrivetrainParams {
    float chainring_teeth = 32.0f;
    float cog_teeth = 18.0f;
    float efficiency = 0.96f;
    float freehub_stiffness_nm_rad = 850.0f;
    float freehub_damping_nm_s_rad = 4.5f;
    float pawl_engage_rad = 0.087266f;
    float max_rider_cadence_rad_s = 12.5f;
    float target_cadence_rad_s = 8.5f;
    float nominal_crank_torque_nm = 45.0f;
    float front_max_brake_torque_nm = 420.0f;
    float rear_max_brake_torque_nm = 380.0f;
    float brake_taper_rad_s = 0.5f;
};

struct GpuDrivetrainState {
    float freehub_boundary_rad = 0.0f;
    int engaged = 0;
    float crank_target_phase_rad = 0.0f;
};

struct GpuDrivetrainOutput {
    float aero_fx_n = 0.0f;
    float aero_fz_n = 0.0f;
    float front_rolling_torque_nm = 0.0f;
    float rear_rolling_torque_nm = 0.0f;
    float front_bearing_torque_nm = 0.0f;
    float rear_bearing_torque_nm = 0.0f;
    float crank_torque_nm = 0.0f;
    float rear_drive_torque_nm = 0.0f;
    float front_brake_torque_nm = 0.0f;
    float rear_brake_torque_nm = 0.0f;
    float tendon_upper_bound_rad = 0.0f;
};

// Matches resistance.cpp:80 — rolling resistance moment opposing wheel spin.
BIKE_HD float gpu_rolling_moment(
    float crr,
    float normal_load_n,
    float eff_radius_m,
    float wheel_omega_rad_s,
    float taper_rad_s
) noexcept {
    if (normal_load_n <= 0.0f || eff_radius_m <= 0.0f || taper_rad_s <= 0.0f) {
        return 0.0f;
    }
    return -crr * normal_load_n * eff_radius_m * gpu_tanh(wheel_omega_rad_s / taper_rad_s);
}

// Matches resistance.cpp:307-331 — planar X-Z aerodynamic drag force:
// v_rel = v_bike - v_wind, F = -0.5 * rho * CdA * |v_rel| * v_rel.
// Returns GpuVec2{fx, fz}.
BIKE_HD GpuVec2 gpu_aerodynamic_force(
    const GpuResistanceParams &rp,
    float bike_vx_mps,
    float bike_vz_mps
) noexcept {
    const float v_rel_x = bike_vx_mps - rp.wind_vx_mps;
    const float v_rel_z = bike_vz_mps - rp.wind_vz_mps;
    const float v_rel_norm = gpu_sqrt(v_rel_x * v_rel_x + v_rel_z * v_rel_z);
    const float scale = -0.5f * rp.rho_kg_m3 * rp.cda_m2 * v_rel_norm;
    return GpuVec2{scale * v_rel_x, scale * v_rel_z};
}

// Wheel hub bearing drag torque (viscous + smooth Coulomb friction).
BIKE_HD float gpu_wheel_bearing_torque(
    const GpuResistanceParams &rp,
    float wheel_omega_rad_s
) noexcept {
    const float taper = rp.rolling_taper_rad_s > 0.0f ? rp.rolling_taper_rad_s : 0.25f;
    return -rp.wheel_bearing_viscous_nm_s_rad * wheel_omega_rad_s
           - rp.wheel_bearing_coulomb_nm * gpu_tanh(wheel_omega_rad_s / taper);
}

// Matches brake.cpp:82-92 & 128-138 — dynamic opposing brake torque with low-speed linear taper.
BIKE_HD float gpu_brake_torque(
    float demand,
    float max_brake_torque_nm,
    float wheel_omega_rad_s,
    float brake_taper_rad_s
) noexcept {
    if (wheel_omega_rad_s == 0.0f || brake_taper_rad_s <= 0.0f || max_brake_torque_nm <= 0.0f) {
        return 0.0f;
    }
    const float clamped = gpu_clamp(demand, 0.0f, 1.0f);
    const float magnitude = clamped * max_brake_torque_nm;
    const float scale = gpu_min(gpu_abs(wheel_omega_rad_s) / brake_taper_rad_s, 1.0f);
    const float sign = wheel_omega_rad_s > 0.0f ? 1.0f : -1.0f;
    return -magnitude * scale * sign;
}

// Resets freehub boundary to current relative transmission coordinate.
BIKE_HD void gpu_reset_drivetrain_state(
    const GpuDrivetrainParams &dp,
    GpuDrivetrainState &st,
    float crank_qpos_rad,
    float rear_wheel_qpos_rad
) noexcept {
    const float ratio = dp.cog_teeth > 0.0f ? (dp.chainring_teeth / dp.cog_teeth) : 1.0f;
    st.freehub_boundary_rad = ratio * crank_qpos_rad - rear_wheel_qpos_rad;
    st.engaged = 0;
    st.crank_target_phase_rad = crank_qpos_rad;
}

// Matches transmission.cpp:298-325 & 334-350 — ratchets freehub upper bound downward
// when coasting (rear wheel outrunning crank) and holds bound when engaged.
BIKE_HD float gpu_update_freehub_bound(
    const GpuDrivetrainParams &dp,
    GpuDrivetrainState &st,
    float crank_qpos_rad,
    float rear_wheel_qpos_rad
) noexcept {
    const float ratio = dp.cog_teeth > 0.0f ? (dp.chainring_teeth / dp.cog_teeth) : 1.0f;
    const float phi = ratio * crank_qpos_rad - rear_wheel_qpos_rad;
    const float prev_bound = st.freehub_boundary_rad;
    const float next_bound = gpu_min(prev_bound, phi);
    st.freehub_boundary_rad = next_bound;
    st.engaged = (phi > prev_bound) ? 1 : 0;
    return next_bound;
}

// Computes aerodynamic drag, rolling resistance, bearing drag, rider crank torque,
// freehub engagement / rear drive torque, and dynamic braking torques.
BIKE_HD GpuDrivetrainOutput gpu_compute_drivetrain_and_resistance(
    const GpuResistanceParams &rp,
    const GpuDrivetrainParams &dp,
    GpuDrivetrainState &st,
    float bike_vx_mps,
    float bike_vz_mps,
    float crank_qpos_rad,
    float crank_omega_rad_s,
    float front_wheel_omega_rad_s,
    float rear_wheel_qpos_rad,
    float rear_wheel_omega_rad_s,
    float front_normal_load_n,
    float rear_normal_load_n,
    float front_eff_radius_m,
    float rear_eff_radius_m,
    float pedal_demand,
    float front_brake_demand,
    float rear_brake_demand
) noexcept {
    GpuDrivetrainOutput out{};

    // 1. Aerodynamic drag force in planar world X-Z
    const GpuVec2 aero = gpu_aerodynamic_force(rp, bike_vx_mps, bike_vz_mps);
    out.aero_fx_n = aero.x;
    out.aero_fz_n = aero.y;

    // 2. Front and rear rolling resistance moments
    out.front_rolling_torque_nm = gpu_rolling_moment(
        rp.crr, front_normal_load_n, front_eff_radius_m,
        front_wheel_omega_rad_s, rp.rolling_taper_rad_s
    );
    out.rear_rolling_torque_nm = gpu_rolling_moment(
        rp.crr, rear_normal_load_n, rear_eff_radius_m,
        rear_wheel_omega_rad_s, rp.rolling_taper_rad_s
    );

    // 3. Wheel bearing drag moments
    out.front_bearing_torque_nm = gpu_wheel_bearing_torque(rp, front_wheel_omega_rad_s);
    out.rear_bearing_torque_nm = gpu_wheel_bearing_torque(rp, rear_wheel_omega_rad_s);

    // 4. Dynamic braking moments
    out.front_brake_torque_nm = gpu_brake_torque(
        front_brake_demand, dp.front_max_brake_torque_nm,
        front_wheel_omega_rad_s, dp.brake_taper_rad_s
    );
    out.rear_brake_torque_nm = gpu_brake_torque(
        rear_brake_demand, dp.rear_max_brake_torque_nm,
        rear_wheel_omega_rad_s, dp.brake_taper_rad_s
    );

    // 5. Freehub ratcheting bound and relative kinematics
    const float ratio = dp.cog_teeth > 0.0f ? (dp.chainring_teeth / dp.cog_teeth) : 1.0f;
    const float phi = ratio * crank_qpos_rad - rear_wheel_qpos_rad;
    const float prev_bound = st.freehub_boundary_rad;
    out.tendon_upper_bound_rad = gpu_update_freehub_bound(
        dp, st, crank_qpos_rad, rear_wheel_qpos_rad
    );
    const float deflection = gpu_max(0.0f, phi - prev_bound);
    const float rel_omega = ratio * crank_omega_rad_s - rear_wheel_omega_rad_s;

    // 6. Rider pedaling torque with braking gate and cadence taper
    const bool braking = (front_brake_demand > 1e-6f) || (rear_brake_demand > 1e-6f);
    const float clamped_pedal = braking ? 0.0f : gpu_clamp(pedal_demand, 0.0f, 1.0f);
    float cadence_factor = 1.0f;
    if (crank_omega_rad_s >= dp.max_rider_cadence_rad_s) {
        cadence_factor = 0.0f;
    } else if (crank_omega_rad_s > dp.target_cadence_rad_s) {
        const float span = gpu_max(1e-6f, dp.max_rider_cadence_rad_s - dp.target_cadence_rad_s);
        cadence_factor = gpu_clamp(
            (dp.max_rider_cadence_rad_s - crank_omega_rad_s) / span, 0.0f, 1.0f
        );
    }
    const float rider_crank_torque = clamped_pedal * dp.nominal_crank_torque_nm * cadence_factor;
    const float crank_viscous_torque = -rp.drivetrain_viscous_nm_s_rad * crank_omega_rad_s;
    out.crank_torque_nm = rider_crank_torque + crank_viscous_torque;

    if (clamped_pedal <= 0.0f) {
        st.crank_target_phase_rad = crank_qpos_rad;
    }

    // 7. Freehub engagement & rear wheel drive torque
    float rear_drive = 0.0f;
    if (deflection > 0.0f) {
        const float pawl_scale = dp.pawl_engage_rad > 0.0f
            ? gpu_clamp(deflection / dp.pawl_engage_rad, 0.0f, 1.0f)
            : 1.0f;
        const float pawl_torque = gpu_max(
            0.0f,
            dp.freehub_stiffness_nm_rad * deflection + dp.freehub_damping_nm_s_rad * rel_omega
        );
        rear_drive = dp.efficiency * pawl_scale * pawl_torque;
        if (rider_crank_torque > 0.0f && rel_omega >= 0.0f) {
            rear_drive = gpu_max(rear_drive, rider_crank_torque * ratio * dp.efficiency);
        }
        st.engaged = rear_drive > 0.0f ? 1 : 0;
    } else if (phi >= prev_bound && rel_omega >= 0.0f && rider_crank_torque > 0.0f) {
        rear_drive = rider_crank_torque * ratio * dp.efficiency;
        st.engaged = 1;
    } else {
        st.engaged = 0;
    }
    out.rear_drive_torque_nm = rear_drive;

    return out;
}

// Overload extracting generalized coordinates and velocities from qpos / qvel via GpuModelIndices.
BIKE_HD GpuDrivetrainOutput gpu_compute_drivetrain_and_resistance(
    const GpuResistanceParams &rp,
    const GpuDrivetrainParams &dp,
    const GpuModelIndices &idx,
    GpuDrivetrainState &st,
    const float *qpos,
    const float *qvel,
    float front_normal_load_n,
    float rear_normal_load_n,
    float front_eff_radius_m,
    float rear_eff_radius_m,
    float pedal_demand,
    float front_brake_demand,
    float rear_brake_demand
) noexcept {
    return gpu_compute_drivetrain_and_resistance(
        rp,
        dp,
        st,
        qvel[idx.root_x_dof],
        qvel[idx.root_z_dof],
        qpos[idx.crank_spin_qpos],
        qvel[idx.crank_spin_dof],
        qvel[idx.front_wheel_spin_dof],
        qpos[idx.rear_wheel_spin_qpos],
        qvel[idx.rear_wheel_spin_dof],
        front_normal_load_n,
        rear_normal_load_n,
        front_eff_radius_m,
        rear_eff_radius_m,
        pedal_demand,
        front_brake_demand,
        rear_brake_demand
    );
}

// Accumulates computed drivetrain and resistance forces into qfrc_world.
BIKE_HD void gpu_write_drivetrain_and_resistance_qfrc(
    const GpuDrivetrainOutput &out,
    const GpuModelIndices &idx,
    float *qfrc_world
) noexcept {
    qfrc_world[idx.root_x_dof] += out.aero_fx_n;
    qfrc_world[idx.root_z_dof] += out.aero_fz_n;
    qfrc_world[idx.front_wheel_spin_dof] +=
        out.front_rolling_torque_nm + out.front_bearing_torque_nm + out.front_brake_torque_nm;
    qfrc_world[idx.rear_wheel_spin_dof] +=
        out.rear_rolling_torque_nm + out.rear_bearing_torque_nm
        + out.rear_drive_torque_nm + out.rear_brake_torque_nm;
    qfrc_world[idx.crank_spin_dof] += out.crank_torque_nm;
}

// Computes drivetrain and resistance forces from scalar kinematics and writes into qfrc_world.
BIKE_HD GpuDrivetrainOutput gpu_write_drivetrain_and_resistance_qfrc(
    const GpuResistanceParams &rp,
    const GpuDrivetrainParams &dp,
    const GpuModelIndices &idx,
    GpuDrivetrainState &st,
    float bike_vx_mps,
    float bike_vz_mps,
    float crank_qpos_rad,
    float crank_omega_rad_s,
    float front_wheel_omega_rad_s,
    float rear_wheel_qpos_rad,
    float rear_wheel_omega_rad_s,
    float front_normal_load_n,
    float rear_normal_load_n,
    float front_eff_radius_m,
    float rear_eff_radius_m,
    float pedal_demand,
    float front_brake_demand,
    float rear_brake_demand,
    float *qfrc_world
) noexcept {
    const GpuDrivetrainOutput out = gpu_compute_drivetrain_and_resistance(
        rp,
        dp,
        st,
        bike_vx_mps,
        bike_vz_mps,
        crank_qpos_rad,
        crank_omega_rad_s,
        front_wheel_omega_rad_s,
        rear_wheel_qpos_rad,
        rear_wheel_omega_rad_s,
        front_normal_load_n,
        rear_normal_load_n,
        front_eff_radius_m,
        rear_eff_radius_m,
        pedal_demand,
        front_brake_demand,
        rear_brake_demand
    );
    gpu_write_drivetrain_and_resistance_qfrc(out, idx, qfrc_world);
    return out;
}

// Computes drivetrain and resistance forces from qpos/qvel arrays and writes into qfrc_world.
BIKE_HD GpuDrivetrainOutput gpu_write_drivetrain_and_resistance_qfrc(
    const GpuResistanceParams &rp,
    const GpuDrivetrainParams &dp,
    const GpuModelIndices &idx,
    GpuDrivetrainState &st,
    const float *qpos,
    const float *qvel,
    float front_normal_load_n,
    float rear_normal_load_n,
    float front_eff_radius_m,
    float rear_eff_radius_m,
    float pedal_demand,
    float front_brake_demand,
    float rear_brake_demand,
    float *qfrc_world
) noexcept {
    const GpuDrivetrainOutput out = gpu_compute_drivetrain_and_resistance(
        rp,
        dp,
        idx,
        st,
        qpos,
        qvel,
        front_normal_load_n,
        rear_normal_load_n,
        front_eff_radius_m,
        rear_eff_radius_m,
        pedal_demand,
        front_brake_demand,
        rear_brake_demand
    );
    gpu_write_drivetrain_and_resistance_qfrc(out, idx, qfrc_world);
    return out;
}

} // namespace bike::gpu
