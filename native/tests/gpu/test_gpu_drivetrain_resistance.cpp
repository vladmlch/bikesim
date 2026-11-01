// native/tests/gpu/test_gpu_drivetrain_resistance.cpp — Unit tests for the
// allocation-free GPU drivetrain, freehub ratcheting, braking, rolling resistance,
// and aerodynamic drag kernels.
#include "../../src/gpu/drivetrain_resistance_kernel.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

void check_near(float actual, float expected, float tol, const char *label) {
    const float diff = std::fabs(actual - expected);
    if (!(diff <= tol)) {
        std::fprintf(
            stderr,
            "FAIL [%s]: actual=%.8f expected=%.8f diff=%.8e tol=%.8e\n",
            label,
            static_cast<double>(actual),
            static_cast<double>(expected),
            static_cast<double>(diff),
            static_cast<double>(tol)
        );
        std::exit(EXIT_FAILURE);
    }
}

void check_true(bool cond, const char *label) {
    if (!cond) {
        std::fprintf(stderr, "FAIL [%s]: condition was false\n", label);
        std::exit(EXIT_FAILURE);
    }
}

void test_rolling_moment() {
    using namespace bike::gpu;

    const float crr = 0.008f;
    const float radius = 0.34f;
    const float taper = 0.25f;
    const std::array<float, 4> loads = {120.0f, 350.0f, 520.0f, 800.0f};
    const std::array<float, 7> speeds = {-15.0f, -0.4f, -0.05f, 0.0f, 0.05f, 0.4f, 18.0f};

    for (float load : loads) {
        for (float speed : speeds) {
            const float actual = gpu_rolling_moment(crr, load, radius, speed, taper);
            const double expected_d =
                -static_cast<double>(crr) * static_cast<double>(load)
                * static_cast<double>(radius)
                * std::tanh(static_cast<double>(speed) / static_cast<double>(taper));
            check_near(actual, static_cast<float>(expected_d), 1e-6f, "gpu_rolling_moment_grid");
        }
    }

    // Zero or negative normal load / radius produces zero rolling moment.
    check_near(gpu_rolling_moment(crr, 0.0f, radius, 10.0f, taper), 0.0f, 1e-7f, "rolling_zero_load");
    check_near(gpu_rolling_moment(crr, -50.0f, radius, 10.0f, taper), 0.0f, 1e-7f, "rolling_neg_load");
    check_near(gpu_rolling_moment(crr, 400.0f, 0.0f, 10.0f, taper), 0.0f, 1e-7f, "rolling_zero_radius");
}

void test_aerodynamic_force() {
    using namespace bike::gpu;

    GpuResistanceParams rp{};
    rp.rho_kg_m3 = 1.225f;
    rp.cda_m2 = 0.38f;

    // 1. Calm air, pure horizontal motion
    {
        rp.wind_vx_mps = 0.0f;
        rp.wind_vz_mps = 0.0f;
        const float vx = 10.0f;
        const float vz = 0.0f;
        const GpuVec2 f = gpu_aerodynamic_force(rp, vx, vz);
        const float expected_fx = -0.5f * rp.rho_kg_m3 * rp.cda_m2 * vx * vx;
        check_near(f.x, expected_fx, 1e-6f, "aero_calm_fx");
        check_near(f.y, 0.0f, 1e-6f, "aero_calm_fz");
    }

    // 2. Headwind (wind_vx_mps < 0 increases opposing drag magnitude)
    {
        rp.wind_vx_mps = -4.0f;
        rp.wind_vz_mps = 0.0f;
        const float vx = 10.0f;
        const float v_rel_x = vx - rp.wind_vx_mps; // 14.0 m/s
        const GpuVec2 f = gpu_aerodynamic_force(rp, vx, 0.0f);
        const float expected_fx = -0.5f * rp.rho_kg_m3 * rp.cda_m2 * std::fabs(v_rel_x) * v_rel_x;
        check_near(f.x, expected_fx, 1e-5f, "aero_headwind_fx");
        check_near(f.y, 0.0f, 1e-6f, "aero_headwind_fz");
    }

    // 3. Strong tailwind exceeding bike speed (pushes forward, fx > 0)
    {
        rp.wind_vx_mps = 12.0f;
        rp.wind_vz_mps = 0.0f;
        const float vx = 8.0f;
        const float v_rel_x = vx - rp.wind_vx_mps; // -4.0 m/s
        const GpuVec2 f = gpu_aerodynamic_force(rp, vx, 0.0f);
        const float expected_fx = -0.5f * rp.rho_kg_m3 * rp.cda_m2 * std::fabs(v_rel_x) * v_rel_x;
        check_true(f.x > 0.0f, "aero_tailwind_pushes_forward");
        check_near(f.x, expected_fx, 1e-6f, "aero_tailwind_fx");
    }

    // 4. Combined horizontal and vertical velocity components with wind
    {
        rp.wind_vx_mps = -2.0f;
        rp.wind_vz_mps = 1.0f;
        const float vx = 6.0f;
        const float vz = -2.0f;
        const float v_rel_x = vx - rp.wind_vx_mps; // 8.0
        const float v_rel_z = vz - rp.wind_vz_mps; // -3.0
        const float v_norm = std::sqrt(v_rel_x * v_rel_x + v_rel_z * v_rel_z);
        const float scale = -0.5f * rp.rho_kg_m3 * rp.cda_m2 * v_norm;
        const GpuVec2 f = gpu_aerodynamic_force(rp, vx, vz);
        check_near(f.x, scale * v_rel_x, 1e-5f, "aero_2d_fx");
        check_near(f.y, scale * v_rel_z, 1e-5f, "aero_2d_fz");
    }
}

void test_freehub_ratcheting() {
    using namespace bike::gpu;

    GpuDrivetrainParams dp{};
    dp.chainring_teeth = 32.0f;
    dp.cog_teeth = 16.0f; // exact 2.0 ratio
    const float ratio = dp.chainring_teeth / dp.cog_teeth;

    GpuDrivetrainState st{};
    gpu_reset_drivetrain_state(dp, st, 0.0f, 0.0f);
    check_near(st.freehub_boundary_rad, 0.0f, 1e-7f, "freehub_init_bound");
    check_true(st.engaged == 0, "freehub_init_disengaged");

    // Step 1: Coasting — rear wheel spins forward to 3.0 rad while crank stays at 0.5 rad
    // phi = 2.0 * 0.5 - 3.0 = -2.0 < 0.0 -> boundary ratchets down to -2.0
    {
        const float bound = gpu_update_freehub_bound(dp, st, 0.5f, 3.0f);
        check_near(bound, -2.0f, 1e-6f, "freehub_coast_step1_bound");
        check_near(st.freehub_boundary_rad, -2.0f, 1e-6f, "freehub_coast_step1_state");
        check_true(st.engaged == 0, "freehub_coast_step1_disengaged");
    }

    // Step 2: Engaged pedaling — crank advances to 0.6 rad while rear wheel is at 3.0 rad
    // phi = 2.0 * 0.6 - 3.0 = -1.8 > -2.0 -> boundary holds at -2.0 (min(-2.0, -1.8))
    {
        const float bound = gpu_update_freehub_bound(dp, st, 0.6f, 3.0f);
        check_near(bound, -2.0f, 1e-6f, "freehub_engaged_bound_holds");
        check_near(st.freehub_boundary_rad, -2.0f, 1e-6f, "freehub_engaged_state_holds");
        check_true(st.engaged == 1, "freehub_engaged_flag");
    }

    // Step 3: Further coasting — rear wheel spins ahead to 5.0 rad while crank is at 0.6 rad
    // phi = 2.0 * 0.6 - 5.0 = -3.8 < -2.0 -> boundary ratchets down to -3.8
    {
        const float expected_phi = ratio * 0.6f - 5.0f;
        const float bound = gpu_update_freehub_bound(dp, st, 0.6f, 5.0f);
        check_near(bound, expected_phi, 1e-6f, "freehub_coast_step3_bound");
        check_true(st.engaged == 0, "freehub_coast_step3_disengaged");
    }
}

void test_braking_and_qfrc_write() {
    using namespace bike::gpu;

    GpuResistanceParams rp{};
    GpuDrivetrainParams dp{};
    GpuModelIndices idx{};
    GpuDrivetrainState st{};

    // Verify dynamic braking taper at low vs high angular velocity
    {
        // Above taper (omega = 2.0 rad/s > 0.5 rad/s): full clamped demand * max torque opposing spin
        const float t_high = gpu_brake_torque(0.5f, dp.front_max_brake_torque_nm, 2.0f, dp.brake_taper_rad_s);
        check_near(t_high, -0.5f * dp.front_max_brake_torque_nm, 1e-5f, "brake_high_speed");

        // Half taper (omega = 0.25 rad/s == 0.5 * 0.5 rad/s): 50% taper scale
        const float t_low = gpu_brake_torque(1.0f, dp.rear_max_brake_torque_nm, 0.25f, dp.brake_taper_rad_s);
        check_near(t_low, -0.5f * dp.rear_max_brake_torque_nm, 1e-5f, "brake_low_speed_taper");

        // Zero angular speed: zero brake torque
        const float t_zero = gpu_brake_torque(1.0f, dp.front_max_brake_torque_nm, 0.0f, dp.brake_taper_rad_s);
        check_near(t_zero, 0.0f, 1e-7f, "brake_zero_speed");
    }

    // Verify full compute + qfrc_world write during engaged pedaling + braking
    {
        std::array<float, kMaxNq> qpos{};
        std::array<float, kMaxNv> qvel{};
        std::array<float, kMaxNv> qfrc_world{};

        // Seed initial state at boundary = 0
        gpu_reset_drivetrain_state(dp, st, 0.0f, 0.0f);

        // Set planar velocities and joint states for engaged pedaling (no braking)
        qvel[idx.root_x_dof] = 8.0f;
        qvel[idx.root_z_dof] = -0.5f;
        qpos[idx.crank_spin_qpos] = 0.2f;
        qvel[idx.crank_spin_dof] = 6.0f;
        qvel[idx.front_wheel_spin_dof] = 24.0f;
        qpos[idx.rear_wheel_spin_qpos] = 0.25f; // phi = (32/18)*0.2 - 0.25 = 0.105556 > 0
        qvel[idx.rear_wheel_spin_dof] = 10.0f;

        const float front_load = 320.0f;
        const float rear_load = 480.0f;
        const float front_rad = 0.34f;
        const float rear_rad = 0.34f;
        const float pedal_demand = 0.8f;

        const GpuDrivetrainOutput out = gpu_write_drivetrain_and_resistance_qfrc(
            rp,
            dp,
            idx,
            st,
            qpos.data(),
            qvel.data(),
            front_load,
            rear_load,
            front_rad,
            rear_rad,
            pedal_demand,
            0.0f,
            0.0f,
            qfrc_world.data()
        );

        check_true(st.engaged == 1, "write_qfrc_engaged");
        check_near(out.tendon_upper_bound_rad, 0.0f, 1e-6f, "write_qfrc_tendon_bound");
        check_true(out.aero_fx_n < 0.0f, "write_qfrc_aero_fx_negative");
        check_true(out.aero_fz_n > 0.0f, "write_qfrc_aero_fz_positive");
        check_true(out.front_rolling_torque_nm < 0.0f, "write_qfrc_front_rolling_negative");
        check_true(out.rear_rolling_torque_nm < 0.0f, "write_qfrc_rear_rolling_negative");
        check_true(out.crank_torque_nm > 0.0f, "write_qfrc_crank_torque_positive");
        check_true(out.rear_drive_torque_nm > 0.0f, "write_qfrc_rear_drive_positive");

        // Verify qfrc_world entries match output components exactly
        check_near(qfrc_world[idx.root_x_dof], out.aero_fx_n, 1e-6f, "qfrc_root_x");
        check_near(qfrc_world[idx.root_z_dof], out.aero_fz_n, 1e-6f, "qfrc_root_z");
        check_near(
            qfrc_world[idx.front_wheel_spin_dof],
            out.front_rolling_torque_nm + out.front_bearing_torque_nm + out.front_brake_torque_nm,
            1e-6f,
            "qfrc_front_wheel_spin"
        );
        check_near(
            qfrc_world[idx.rear_wheel_spin_dof],
            out.rear_rolling_torque_nm + out.rear_bearing_torque_nm
                + out.rear_drive_torque_nm + out.rear_brake_torque_nm,
            1e-6f,
            "qfrc_rear_wheel_spin"
        );
        check_near(qfrc_world[idx.crank_spin_dof], out.crank_torque_nm, 1e-6f, "qfrc_crank_spin");
    }

    // Verify braking cuts rider crank effort and writes negative brake torques into wheel DOFs
    {
        std::array<float, kMaxNq> qpos{};
        std::array<float, kMaxNv> qvel{};
        std::array<float, kMaxNv> qfrc_world{};

        gpu_reset_drivetrain_state(dp, st, 0.0f, 0.0f);
        qvel[idx.root_x_dof] = 10.0f;
        qvel[idx.crank_spin_dof] = 5.0f;
        qvel[idx.front_wheel_spin_dof] = 25.0f;
        qpos[idx.rear_wheel_spin_qpos] = 2.0f; // coasting (wheel ahead of crank)
        qvel[idx.rear_wheel_spin_dof] = 25.0f;

        const GpuDrivetrainOutput out = gpu_write_drivetrain_and_resistance_qfrc(
            rp,
            dp,
            idx,
            st,
            qpos.data(),
            qvel.data(),
            350.0f,
            450.0f,
            0.34f,
            0.34f,
            1.0f, // pedal demand high, but gated off by braking
            0.5f, // 50% front brake
            0.25f, // 25% rear brake
            qfrc_world.data()
        );

        check_true(st.engaged == 0, "braking_coasting_disengaged");
        check_near(out.tendon_upper_bound_rad, -2.0f, 1e-6f, "braking_coasting_bound");
        check_near(out.rear_drive_torque_nm, 0.0f, 1e-6f, "braking_zero_rear_drive");
        check_near(
            out.crank_torque_nm,
            -rp.drivetrain_viscous_nm_s_rad * qvel[idx.crank_spin_dof],
            1e-6f,
            "braking_cuts_rider_torque"
        );
        check_near(out.front_brake_torque_nm, -0.5f * dp.front_max_brake_torque_nm, 1e-5f, "braking_front_torque");
        check_near(out.rear_brake_torque_nm, -0.25f * dp.rear_max_brake_torque_nm, 1e-5f, "braking_rear_torque");
        check_near(
            qfrc_world[idx.front_wheel_spin_dof],
            out.front_rolling_torque_nm + out.front_bearing_torque_nm + out.front_brake_torque_nm,
            1e-5f,
            "braking_qfrc_front"
        );
        check_near(
            qfrc_world[idx.rear_wheel_spin_dof],
            out.rear_rolling_torque_nm + out.rear_bearing_torque_nm + out.rear_brake_torque_nm,
            1e-5f,
            "braking_qfrc_rear"
        );
    }
}

} // namespace

int main() {
    test_rolling_moment();
    test_aerodynamic_force();
    test_freehub_ratcheting();
    test_braking_and_qfrc_write();
    std::printf("PASS: test_gpu_drivetrain_resistance\n");
    return EXIT_SUCCESS;
}
