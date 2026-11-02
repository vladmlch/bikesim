// native/tests/gpu/test_gpu_drivetrain_resistance.cpp — Unit tests for the
// allocation-free GPU drivetrain, freehub ratcheting, braking, rolling resistance,
// and aerodynamic drag kernels, verifying default gear ratio (22/51), parity with
// authoritative native drivetrain::Freehub, and single torque path modes.
#include "../../src/drivetrain/freehub.hpp"
// freehub.cpp is included textually on purpose: this test executable links
// standalone (no bike_native_contract_core), the same arrangement as
// test_gpu_tire.cpp/station.cpp and test_gpu_rider.cpp/spindle_math.cpp —
// see the note above bike_add_gpu_test in native/CMakeLists.txt.
// NOLINTNEXTLINE(bugprone-suspicious-include) deliberate standalone-link mechanism: freehub.cpp is compiled INTO this TU so the test links without bike_native_contract_core and avoids duplicate symbol definitions (native/CMakeLists.txt:543-547). Register: docs/TESTING.md GPU test cpp-include row.
#include "../../src/drivetrain/freehub.cpp"
#include "../../src/gpu/drivetrain_resistance_kernel.hpp"
#include "../../src/platform/argv.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

using namespace bike::gpu;

using TestFunction = void (*)();

struct TestCase {
    std::string_view name;
    TestFunction run;
};

void require(bool condition, std::string_view message) {
    if (!condition)
        throw std::runtime_error(std::string(message));
}

// Numeric helper used instead of printf-style messages. Templated on the
// arithmetic type so float comparisons keep their original float semantics
// instead of silently promoting to double.
template <class T>
void require_near(T actual, T expected, T tol, std::string_view what) {
    if (!(std::abs(actual - expected) <= tol))
        throw std::runtime_error(std::string(what) +
                                 ": actual=" + std::to_string(actual) +
                                 " expected=" + std::to_string(expected) +
                                 " tol=" + std::to_string(tol));
}

// Relative-tolerance variant of require_near: the tolerance scales with the
// expected magnitude (same scale rule as the old check_rel).
void require_rel(double actual, double expected, double rel_tol,
                 std::string_view what) {
    const double scale = std::fabs(expected) > 0.0 ? std::fabs(expected) : 1.0;
    const double rel = std::fabs(actual - expected) / scale;
    if (!(rel <= rel_tol))
        throw std::runtime_error(std::string(what) +
                                 ": actual=" + std::to_string(actual) +
                                 " expected=" + std::to_string(expected) +
                                 " rel=" + std::to_string(rel) +
                                 " tol=" + std::to_string(rel_tol));
}

// GpuModelIndices addresses are int for the kernel ABI (a disabled slot is
// -1); container subscripts take size_t, so the explicit conversion lives in
// one place.
constexpr std::size_t to_index(int field) {
    return static_cast<std::size_t>(field);
}

// The authoritative Freehub reports its boundary as an optional; the parity
// assertions need the engaged value, so a missing value is a hard failure
// rather than UB. The guarded dereference keeps the access provably checked.
double engaged_value(const std::optional<double> &value, std::string_view what) {
    if (!value.has_value())
        throw std::runtime_error(std::string(what) +
                                 ": expected engaged freehub boundary");
    return *value;
}

// Requirement 1 & 2: Default GpuDrivetrainParams teeth counts and ratio matching model.xml (22 / 51).
void test_default_gear_ratio() {
    const GpuDrivetrainParams dp{};
    require_near(dp.chainring_teeth, 22.0f, 1e-7f, "default_chainring_teeth_22");
    require_near(dp.cog_teeth, 51.0f, 1e-7f, "default_cog_teeth_51");

    const float expected_ratio = 22.0f / 51.0f;
    const float actual_ratio = gpu_drivetrain_ratio(dp);
    require_near(actual_ratio, expected_ratio, 1e-7f, "default_drivetrain_ratio_22_51");
}

void test_rolling_moment() {
    const float crr = 0.008f;
    const float radius = 0.34f;
    const float taper = 0.25f;
    const std::array<float, 4> loads = {120.0f, 350.0f, 520.0f, 800.0f};
    const std::array<float, 7> speeds = {-15.0f, -0.4f, -0.05f, 0.0f, 0.05f, 0.4f, 18.0f};

    for (const float load : loads) {
        for (const float speed : speeds) {
            const float actual = gpu_rolling_moment(crr, load, radius, speed, taper);
            const double expected_d =
                -static_cast<double>(crr) * static_cast<double>(load)
                * static_cast<double>(radius)
                * std::tanh(static_cast<double>(speed) / static_cast<double>(taper));
            require_near(actual, static_cast<float>(expected_d), 1e-6f, "gpu_rolling_moment_grid");
        }
    }

    // Zero or negative normal load / radius produces zero rolling moment.
    require_near(gpu_rolling_moment(crr, 0.0f, radius, 10.0f, taper), 0.0f, 1e-7f, "rolling_zero_load");
    require_near(gpu_rolling_moment(crr, -50.0f, radius, 10.0f, taper), 0.0f, 1e-7f, "rolling_neg_load");
    require_near(gpu_rolling_moment(crr, 400.0f, 0.0f, 10.0f, taper), 0.0f, 1e-7f, "rolling_zero_radius");
}

void test_aerodynamic_force() {
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
        require_near(f.x, expected_fx, 1e-6f, "aero_calm_fx");
        require_near(f.y, 0.0f, 1e-6f, "aero_calm_fz");
    }

    // 2. Headwind (wind_vx_mps < 0 increases opposing drag magnitude)
    {
        rp.wind_vx_mps = -4.0f;
        rp.wind_vz_mps = 0.0f;
        const float vx = 10.0f;
        const float v_rel_x = vx - rp.wind_vx_mps; // 14.0 m/s
        const GpuVec2 f = gpu_aerodynamic_force(rp, vx, 0.0f);
        const float expected_fx = -0.5f * rp.rho_kg_m3 * rp.cda_m2 * std::fabs(v_rel_x) * v_rel_x;
        require_near(f.x, expected_fx, 1e-5f, "aero_headwind_fx");
        require_near(f.y, 0.0f, 1e-6f, "aero_headwind_fz");
    }

    // 3. Strong tailwind exceeding bike speed (pushes forward, fx > 0)
    {
        rp.wind_vx_mps = 12.0f;
        rp.wind_vz_mps = 0.0f;
        const float vx = 8.0f;
        const float v_rel_x = vx - rp.wind_vx_mps; // -4.0 m/s
        const GpuVec2 f = gpu_aerodynamic_force(rp, vx, 0.0f);
        const float expected_fx = -0.5f * rp.rho_kg_m3 * rp.cda_m2 * std::fabs(v_rel_x) * v_rel_x;
        require(f.x > 0.0f, "aero_tailwind_pushes_forward");
        require_near(f.x, expected_fx, 1e-6f, "aero_tailwind_fx");
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
        require_near(f.x, scale * v_rel_x, 1e-5f, "aero_2d_fx");
        require_near(f.y, scale * v_rel_z, 1e-5f, "aero_2d_fz");
    }
}

void test_freehub_ratcheting() {
    // Test A: 2.0 ratio (32 / 16)
    {
        GpuDrivetrainParams dp{};
        dp.chainring_teeth = 32.0f;
        dp.cog_teeth = 16.0f;
        const float ratio = gpu_drivetrain_ratio(dp);
        require_near(ratio, 2.0f, 1e-7f, "custom_ratio_2_0");

        GpuDrivetrainState st{};
        gpu_reset_drivetrain_state(dp, st, 0.0f, 0.0f);
        require_near(st.freehub_boundary_rad, 0.0f, 1e-7f, "freehub_init_bound");
        require(st.engaged == 0, "freehub_init_disengaged");

        // Step 1: Coasting — rear wheel spins forward to 3.0 rad while crank stays at 0.5 rad
        // phi = 2.0 * 0.5 - 3.0 = -2.0 < 0.0 -> boundary ratchets down to -2.0
        {
            const float bound = gpu_update_freehub_bound(dp, st, 0.5f, 3.0f);
            require_near(bound, -2.0f, 1e-6f, "freehub_coast_step1_bound");
            require_near(st.freehub_boundary_rad, -2.0f, 1e-6f, "freehub_coast_step1_state");
            require(st.engaged == 0, "freehub_coast_step1_disengaged");
        }

        // Step 2: Engaged pedaling — crank advances to 0.6 rad while rear wheel is at 3.0 rad
        // phi = 2.0 * 0.6 - 3.0 = -1.8 > -2.0 -> boundary holds at -2.0 (min(-2.0, -1.8))
        {
            const float bound = gpu_update_freehub_bound(dp, st, 0.6f, 3.0f);
            require_near(bound, -2.0f, 1e-6f, "freehub_engaged_bound_holds");
            require_near(st.freehub_boundary_rad, -2.0f, 1e-6f, "freehub_engaged_state_holds");
            require(st.engaged == 1, "freehub_engaged_flag");
        }

        // Step 3: Further coasting — rear wheel spins ahead to 5.0 rad while crank is at 0.6 rad
        // phi = 2.0 * 0.6 - 5.0 = -3.8 < -2.0 -> boundary ratchets down to -3.8
        {
            const float expected_phi = ratio * 0.6f - 5.0f;
            const float bound = gpu_update_freehub_bound(dp, st, 0.6f, 5.0f);
            require_near(bound, expected_phi, 1e-6f, "freehub_coast_step3_bound");
            require(st.engaged == 0, "freehub_coast_step3_disengaged");
        }
    }

    // Test B: Default ratio (22 / 51 = 0.43137255)
    {
        const GpuDrivetrainParams dp{};
        const float ratio = gpu_drivetrain_ratio(dp);
        GpuDrivetrainState st{};
        gpu_reset_drivetrain_state(dp, st, 0.0f, 0.0f);

        // Coasting: crank = 1.0 rad, rear = 1.0 rad -> phi = 0.43137255 - 1.0 = -0.56862745
        const float bound1 = gpu_update_freehub_bound(dp, st, 1.0f, 1.0f);
        require_near(bound1, ratio * 1.0f - 1.0f, 1e-6f, "freehub_default_ratio_coast_bound");
        require(st.engaged == 0, "freehub_default_ratio_coast_disengaged");

        // Engaged: crank advances to 3.0 rad -> phi = 3 * ratio - 1.0 = 1.2941 - 1.0 = 0.2941 > bound1
        const float bound2 = gpu_update_freehub_bound(dp, st, 3.0f, 1.0f);
        require_near(bound2, bound1, 1e-6f, "freehub_default_ratio_engaged_bound_holds");
        require(st.engaged == 1, "freehub_default_ratio_engaged_flag");
    }
}

// Requirement 4.2: Direct bitwise/numerical parity with authoritative native drivetrain::Freehub
void test_authoritative_freehub_parity() {
    const GpuDrivetrainParams dp{};
    const float ratio = gpu_drivetrain_ratio(dp);

    // Construct authoritative native Freehub with identical spring stiffness and damping
    drivetrain::Freehub native_hub(static_cast<double>(dp.freehub_stiffness_nm_rad),
                                   static_cast<double>(dp.freehub_damping_nm_s_rad));

    GpuDrivetrainState gpu_st{};
    gpu_reset_drivetrain_state(dp, gpu_st, 0.0f, 0.0f);
    native_hub.set_state(drivetrain::FreehubSnapshot{.boundary = 0.0, .energy_j = 0.0, .torque_nm = 0.0});

    // 1. Coasting phase: cassette outrun by rear wheel
    {
        const float crank_qpos = 1.0f;
        const float rear_qpos = 2.0f;
        const float crank_omega = 5.0f;
        const float rear_omega = 15.0f;

        const double pc = static_cast<double>(ratio * crank_qpos);
        const double pw = static_cast<double>(rear_qpos);
        const double wc = static_cast<double>(ratio * crank_omega);
        const double ww = static_cast<double>(rear_omega);

        const double native_torque = native_hub.update(pc, pw, wc, ww);
        const float gpu_bound = gpu_update_freehub_bound(dp, gpu_st, crank_qpos, rear_qpos);

        require_near(static_cast<float>(native_torque), 0.0f, 1e-7f, "native_coast_zero_torque");
        require(gpu_st.engaged == 0, "gpu_coast_disengaged");
        require_near(gpu_bound, static_cast<float>(engaged_value(native_hub.state().boundary, "coast_bound_parity")), 1e-6f, "coast_bound_parity");
    }

    // 2. Engagement phase: cassette catches up and engages freehub
    {
        const float crank_qpos = 6.0f; // ratio * 6 = 2.588 > 2.0
        const float rear_qpos = 2.0f;
        const float crank_omega = 35.0f; // ratio * 35 = 15.098 > 10.0
        const float rear_omega = 10.0f;

        const double pc = static_cast<double>(ratio * crank_qpos);
        const double pw = static_cast<double>(rear_qpos);
        const double wc = static_cast<double>(ratio * crank_omega);
        const double ww = static_cast<double>(rear_omega);

        const double native_torque = native_hub.update(pc, pw, wc, ww);

        // Run full gpu_compute_drivetrain_and_resistance with pedal_demand = 0 to isolate pawl torque
        const GpuResistanceParams rp{};
        const GpuDrivetrainOutput out = gpu_compute_drivetrain_and_resistance(
            rp, dp, gpu_st,
            0.0f, 0.0f,
            crank_qpos, crank_omega, 0.0f,
            rear_qpos, rear_omega,
            0.0f, 0.0f, 0.34f, 0.34f,
            0.0f, // pedal_demand = 0
            0.0f, 0.0f
        );

        require(native_torque > 0.0, "native_engaged_positive_torque");
        require(gpu_st.engaged == 1, "gpu_engaged_flag");
        require_near(gpu_st.freehub_boundary_rad, static_cast<float>(engaged_value(native_hub.state().boundary, "engaged_bound_parity")), 1e-6f, "engaged_bound_parity");

        // Deflection is large (> pawl_engage_rad = 0.087266 rad), so pawl_scale == 1.0f.
        // out.rear_drive_torque_nm = efficiency * pawl_torque.
        const float expected_gpu_drive = dp.efficiency * static_cast<float>(native_torque);
        require_near(out.rear_drive_torque_nm, expected_gpu_drive, 1e-4f, "engaged_pawl_torque_parity");
    }

    // 3. Multi-point systematic sweep across relative angles and velocities
    {
        const std::array<float, 5> crank_angles = {0.0f, 2.0f, 5.0f, 10.0f, 15.0f};
        const std::array<float, 5> wheel_angles = {0.0f, 1.0f, 3.0f, 4.0f, 7.0f};
        const std::array<float, 3> crank_speeds = {0.0f, 15.0f, 30.0f};
        const std::array<float, 3> wheel_speeds = {0.0f, 8.0f, 20.0f};

        const GpuResistanceParams rp_sweep{};
        GpuDrivetrainParams dp_sweep = dp;
        dp_sweep.pawl_engage_rad = 0.0f; // Match native Freehub step engagement
        dp_sweep.efficiency = 1.0f;

        gpu_reset_drivetrain_state(dp_sweep, gpu_st, 0.0f, 0.0f);
        native_hub.set_state(drivetrain::FreehubSnapshot{.boundary = 0.0, .energy_j = 0.0, .torque_nm = 0.0});

        for (const float ca : crank_angles) {
            for (const float wa : wheel_angles) {
                for (const float cs : crank_speeds) {
                    for (const float ws : wheel_speeds) {
                        const double pc = static_cast<double>(ratio * ca);
                        const double pw = static_cast<double>(wa);
                        const double wc = static_cast<double>(ratio * cs);
                        const double ww = static_cast<double>(ws);

                        const float prev_bound = gpu_st.freehub_boundary_rad;
                        const double native_torque = native_hub.update(pc, pw, wc, ww);
                        const GpuDrivetrainOutput sweep_out = gpu_compute_drivetrain_and_resistance(
                            rp_sweep, dp_sweep, gpu_st,
                            /*bike_vx_mps=*/0.0f, /*bike_vz_mps=*/0.0f,
                            ca, cs, /*front_wheel_omega_rad_s=*/0.0f,
                            wa, ws,
                            /*front_normal_load_n=*/0.0f, /*rear_normal_load_n=*/0.0f,
                            0.34f, 0.34f,
                            /*pedal_demand=*/0.0f, 0.0f, 0.0f
                        );

                        require_near(
                            sweep_out.tendon_upper_bound_rad,
                            static_cast<float>(engaged_value(native_hub.state().boundary, "sweep_bound_parity")),
                            1e-6f,
                            "sweep_bound_parity"
                        );

                        if (ratio * ca - wa > prev_bound) {
                            const float tol = std::max(1e-4f, 1e-6f * std::fabs(static_cast<float>(native_torque)));
                            require_near(
                                sweep_out.rear_drive_torque_nm,
                                static_cast<float>(native_torque),
                                tol,
                                "sweep_pawl_torque_formula_parity"
                            );
                        }
                    }
                }
            }
        }
    }
}

// Requirement 3 & 4.3: Verify both include_rear_drive_torque == true and include_rear_drive_torque == false
void test_drivetrain_torque_paths_and_modes() {
    const GpuResistanceParams rp{};
    const GpuDrivetrainParams dp{};
    const GpuModelIndices idx{};

    // Setup kinematic states for engaged forward pedaling
    std::array<float, kMaxNq> qpos{};
    std::array<float, kMaxNv> qvel{};
    qvel[to_index(idx.root_x_dof)] = 8.0f;
    qvel[to_index(idx.root_z_dof)] = -0.2f;
    qpos[to_index(idx.crank_spin_qpos)] = 2.0f; // ratio * 2.0 = 0.8627 > 0.2
    qvel[to_index(idx.crank_spin_dof)] = 8.0f; // ratio * 8.0 = 3.451 > 3.0, and < target_cadence_rad_s (10.995574, 105 rpm)
    qvel[to_index(idx.front_wheel_spin_dof)] = 22.0f;
    qpos[to_index(idx.rear_wheel_spin_qpos)] = 0.2f;
    qvel[to_index(idx.rear_wheel_spin_dof)] = 3.0f;

    const float front_load = 320.0f;
    const float rear_load = 480.0f;
    const float front_rad = 0.34f;
    const float rear_rad = 0.34f;
    const float pedal_demand = 0.8f;

    // --- Mode 1: include_rear_drive_torque == true (standalone pawl mode) ---
    std::array<float, kMaxNv> qfrc_standalone{};
    GpuDrivetrainState st_standalone{};
    gpu_reset_drivetrain_state(dp, st_standalone, 0.0f, 0.0f);

    const GpuDrivetrainOutput out_standalone = gpu_write_drivetrain_and_resistance_qfrc(
        rp, dp, idx, st_standalone,
        qpos.data(), qvel.data(),
        front_load, rear_load, front_rad, rear_rad,
        pedal_demand, 0.0f, 0.0f,
        qfrc_standalone.data(),
        /*include_rear_drive_torque=*/true
    );

    require(st_standalone.engaged == 1, "standalone_engaged");
    require(out_standalone.rear_drive_torque_nm > 0.0f, "standalone_rear_drive_positive");
    require(out_standalone.crank_torque_nm > 0.0f, "standalone_crank_torque_positive");

    // In standalone mode, rear_drive_torque_nm MUST be included in qfrc_world[idx.rear_wheel_spin_dof]
    const float expected_rear_qfrc_standalone =
        out_standalone.rear_rolling_torque_nm + out_standalone.rear_bearing_torque_nm
        + out_standalone.rear_drive_torque_nm + out_standalone.rear_brake_torque_nm;
    require_near(
        qfrc_standalone[to_index(idx.rear_wheel_spin_dof)],
        expected_rear_qfrc_standalone,
        1e-6f,
        "standalone_qfrc_rear_wheel_includes_drive"
    );
    // In standalone mode, crank_spin_dof receives crank_torque_nm minus the equal-and-opposite
    // pawl reaction torque rear_drive_torque_nm * (ratio / efficiency).
    const float ratio = gpu_drivetrain_ratio(dp);
    const float eff = std::max(dp.efficiency, 1e-6f);
    const float expected_crank_pawl_reaction = out_standalone.rear_drive_torque_nm * (ratio / eff);
    require_near(
        qfrc_standalone[to_index(idx.crank_spin_dof)],
        out_standalone.crank_torque_nm - expected_crank_pawl_reaction,
        1e-5f,
        "standalone_qfrc_crank_spin_includes_pawl_reaction"
    );

    // --- Mode 2: include_rear_drive_torque == false (tendon-backed mode) ---
    std::array<float, kMaxNv> qfrc_tendon{};
    GpuDrivetrainState st_tendon{};
    gpu_reset_drivetrain_state(dp, st_tendon, 0.0f, 0.0f);

    const GpuDrivetrainOutput out_tendon = gpu_write_drivetrain_and_resistance_qfrc(
        rp, dp, idx, st_tendon,
        qpos.data(), qvel.data(),
        front_load, rear_load, front_rad, rear_rad,
        pedal_demand, 0.0f, 0.0f,
        qfrc_tendon.data(),
        /*include_rear_drive_torque=*/false
    );

    require(st_tendon.engaged == 1, "tendon_engaged");
    require(out_tendon.rear_drive_torque_nm > 0.0f, "tendon_output_rear_drive_still_computed");
    require_near(
        out_tendon.rear_drive_torque_nm,
        out_standalone.rear_drive_torque_nm,
        1e-6f,
        "output_rear_drive_identical_between_modes"
    );
    require_near(
        out_tendon.tendon_upper_bound_rad,
        st_tendon.freehub_boundary_rad,
        1e-6f,
        "tendon_upper_bound_matches_boundary"
    );

    // In tendon-backed mode, rear_drive_torque_nm MUST be OMITTED from qfrc_world[idx.rear_wheel_spin_dof]
    const float expected_rear_qfrc_tendon =
        out_tendon.rear_rolling_torque_nm + out_tendon.rear_bearing_torque_nm + out_tendon.rear_brake_torque_nm;
    require_near(
        qfrc_tendon[to_index(idx.rear_wheel_spin_dof)],
        expected_rear_qfrc_tendon,
        1e-6f,
        "tendon_qfrc_rear_wheel_omits_drive"
    );
    // In tendon-backed mode, crank torque is written without standalone pawl reaction (the MuJoCo tendon supplies it)
    require_near(
        qfrc_tendon[to_index(idx.crank_spin_dof)],
        out_tendon.crank_torque_nm,
        1e-6f,
        "tendon_qfrc_crank_spin"
    );

    // Exact single torque path invariant:
    // Difference between standalone and tendon rear qfrc is rear_drive_torque_nm,
    // and difference between tendon and standalone crank qfrc is the pawl reaction torque.
    const float qfrc_diff = qfrc_standalone[to_index(idx.rear_wheel_spin_dof)] - qfrc_tendon[to_index(idx.rear_wheel_spin_dof)];
    require_near(qfrc_diff, out_standalone.rear_drive_torque_nm, 1e-6f, "qfrc_diff_exact_single_torque");
    const float crank_diff = qfrc_tendon[to_index(idx.crank_spin_dof)] - qfrc_standalone[to_index(idx.crank_spin_dof)];
    require_near(crank_diff, expected_crank_pawl_reaction, 1e-5f, "crank_diff_exact_pawl_reaction");
    require_near(
        qfrc_standalone[to_index(idx.front_wheel_spin_dof)],
        qfrc_tendon[to_index(idx.front_wheel_spin_dof)],
        1e-6f,
        "front_wheel_torque_invariant_across_modes"
    );
    require_near(
        qfrc_standalone[to_index(idx.root_x_dof)],
        qfrc_tendon[to_index(idx.root_x_dof)],
        1e-6f,
        "root_x_aero_invariant_across_modes"
    );

    // --- Mode 3: Verify default argument include_rear_drive_torque = true ---
    {
        std::array<float, kMaxNv> qfrc_default{};
        GpuDrivetrainState st_def{};
        gpu_reset_drivetrain_state(dp, st_def, 0.0f, 0.0f);
        gpu_write_drivetrain_and_resistance_qfrc(
            rp, dp, idx, st_def,
            qpos.data(), qvel.data(),
            front_load, rear_load, front_rad, rear_rad,
            pedal_demand, 0.0f, 0.0f,
            qfrc_default.data()
            // default argument
        );
        require_near(
            qfrc_default[to_index(idx.rear_wheel_spin_dof)],
            qfrc_standalone[to_index(idx.rear_wheel_spin_dof)],
            1e-6f,
            "default_argument_matches_standalone"
        );
    }

    // --- Mode 4: Verify scalar overload with include_rear_drive_torque = false ---
    {
        std::array<float, kMaxNv> qfrc_scalar{};
        GpuDrivetrainState st_scalar{};
        gpu_reset_drivetrain_state(dp, st_scalar, 0.0f, 0.0f);
        gpu_write_drivetrain_and_resistance_qfrc(
            rp, dp, idx, st_scalar,
            qvel[to_index(idx.root_x_dof)], qvel[to_index(idx.root_z_dof)],
            qpos[to_index(idx.crank_spin_qpos)], qvel[to_index(idx.crank_spin_dof)],
            qvel[to_index(idx.front_wheel_spin_dof)],
            qpos[to_index(idx.rear_wheel_spin_qpos)], qvel[to_index(idx.rear_wheel_spin_dof)],
            front_load, rear_load, front_rad, rear_rad,
            pedal_demand, 0.0f, 0.0f,
            qfrc_scalar.data(),
            /*include_rear_drive_torque=*/false
        );
        require_near(
            qfrc_scalar[to_index(idx.rear_wheel_spin_dof)],
            expected_rear_qfrc_tendon,
            1e-6f,
            "scalar_overload_tendon_mode"
        );
    }
}

void test_braking_and_qfrc_write() {
    const GpuResistanceParams rp{};
    const GpuDrivetrainParams dp{};
    const GpuModelIndices idx{};
    GpuDrivetrainState st{};

    // Verify dynamic braking taper at low vs high angular velocity
    {
        // Above taper (omega = 2.0 rad/s > 0.5 rad/s): full clamped demand * max torque opposing spin
        const float t_high = gpu_brake_torque(0.5f, dp.front_max_brake_torque_nm, 2.0f, dp.brake_taper_rad_s);
        require_near(t_high, -0.5f * dp.front_max_brake_torque_nm, 1e-5f, "brake_high_speed");

        // Half taper (omega = 0.25 rad/s == 0.5 * 0.5 rad/s): 50% taper scale
        const float t_low = gpu_brake_torque(1.0f, dp.rear_max_brake_torque_nm, 0.25f, dp.brake_taper_rad_s);
        require_near(t_low, -0.5f * dp.rear_max_brake_torque_nm, 1e-5f, "brake_low_speed_taper");

        // Zero angular speed: zero brake torque
        const float t_zero = gpu_brake_torque(1.0f, dp.front_max_brake_torque_nm, 0.0f, dp.brake_taper_rad_s);
        require_near(t_zero, 0.0f, 1e-7f, "brake_zero_speed");
    }

    // Verify braking cuts rider crank effort and writes negative brake torques into wheel DOFs
    {
        std::array<float, kMaxNq> qpos{};
        std::array<float, kMaxNv> qvel{};
        std::array<float, kMaxNv> qfrc_world{};

        gpu_reset_drivetrain_state(dp, st, 0.0f, 0.0f);
        qvel[to_index(idx.root_x_dof)] = 10.0f;
        qvel[to_index(idx.crank_spin_dof)] = 5.0f;
        qvel[to_index(idx.front_wheel_spin_dof)] = 25.0f;
        qpos[to_index(idx.rear_wheel_spin_qpos)] = 2.0f; // coasting (wheel ahead of crank)
        qvel[to_index(idx.rear_wheel_spin_dof)] = 25.0f;

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

        require(st.engaged == 0, "braking_coasting_disengaged");
        require_near(out.tendon_upper_bound_rad, -2.0f, 1e-6f, "braking_coasting_bound");
        require_near(out.rear_drive_torque_nm, 0.0f, 1e-6f, "braking_zero_rear_drive");
        require_near(
            out.crank_torque_nm,
            -rp.drivetrain_viscous_nm_s_rad * qvel[to_index(idx.crank_spin_dof)],
            1e-6f,
            "braking_cuts_rider_torque"
        );
        require_near(out.front_brake_torque_nm, -0.5f * dp.front_max_brake_torque_nm, 1e-5f, "braking_front_torque");
        require_near(out.rear_brake_torque_nm, -0.25f * dp.rear_max_brake_torque_nm, 1e-5f, "braking_rear_torque");
        require_near(
            qfrc_world[to_index(idx.front_wheel_spin_dof)],
            out.front_rolling_torque_nm + out.front_bearing_torque_nm + out.front_brake_torque_nm,
            1e-5f,
            "braking_qfrc_front"
        );
        require_near(
            qfrc_world[to_index(idx.rear_wheel_spin_dof)],
            out.rear_rolling_torque_nm + out.rear_bearing_torque_nm + out.rear_brake_torque_nm,
            1e-5f,
            "braking_qfrc_rear"
        );
    }

    // Verify += accumulation on non-zero qfrc_world and disabled-DOF (-1 / out-of-bounds) safety
    {
        std::array<float, kMaxNq> qpos{};
        std::array<float, kMaxNv> qvel{};
        qvel[to_index(idx.root_x_dof)] = 8.0f;
        qvel[to_index(idx.crank_spin_dof)] = 6.0f;
        qvel[to_index(idx.front_wheel_spin_dof)] = 20.0f;
        qvel[to_index(idx.rear_wheel_spin_dof)] = 20.0f;

        std::array<float, kMaxNv> qfrc_accum{};
        qfrc_accum.fill(10.0f);
        gpu_reset_drivetrain_state(dp, st, 0.0f, 0.0f);
        const GpuDrivetrainOutput out_accum = gpu_write_drivetrain_and_resistance_qfrc(
            rp, dp, idx, st, qpos.data(), qvel.data(),
            300.0f, 400.0f, 0.34f, 0.34f, 0.5f, 0.0f, 0.0f, qfrc_accum.data(), true
        );
        require_near(qfrc_accum[to_index(idx.root_x_dof)], 10.0f + out_accum.aero_fx_n, 1e-5f, "accum_root_x");
        require_near(qfrc_accum[to_index(idx.crank_spin_dof)], 10.0f + out_accum.crank_torque_nm, 1e-5f, "accum_crank");

        // Disabled DOFs (-1 and >= kMaxNv) must not write out of bounds or modify qfrc
        GpuModelIndices disabled_idx = idx;
        disabled_idx.root_x_dof = -1;
        disabled_idx.root_z_dof = -1;
        disabled_idx.front_wheel_spin_dof = -1;
        disabled_idx.rear_wheel_spin_dof = kMaxNv;
        disabled_idx.crank_spin_dof = -1;
        disabled_idx.crank_spin_qpos = -1;
        disabled_idx.rear_wheel_spin_qpos = kMaxNq;

        std::array<float, kMaxNv> qfrc_disabled{};
        qfrc_disabled.fill(7.5f);
        gpu_write_drivetrain_and_resistance_qfrc(
            rp, dp, disabled_idx, st, qpos.data(), qvel.data(),
            300.0f, 400.0f, 0.34f, 0.34f, 0.5f, 0.0f, 0.0f, qfrc_disabled.data(), true
        );
        for (const float value : qfrc_disabled) {
            require_near(value, 7.5f, 1e-6f, "disabled_dof_unchanged");
        }
    }
}

// Rider crank torque transferred to the rear wheel by power balance through the tendon constraint
// wheel_omega = ratio * crank_omega: wheel torque = crank torque / ratio * efficiency
// = crank torque * (cog / chainring) * efficiency.
double expected_rider_wheel_torque(const GpuDrivetrainParams &dp, double crank_torque_nm) {
    return crank_torque_nm * static_cast<double>(dp.cog_teeth) / static_cast<double>(dp.chainring_teeth)
        * static_cast<double>(dp.efficiency);
}

void require_output_finite(const GpuDrivetrainOutput &out, std::string_view what) {
    require(std::isfinite(out.aero_fx_n), what);
    require(std::isfinite(out.aero_fz_n), what);
    require(std::isfinite(out.front_rolling_torque_nm), what);
    require(std::isfinite(out.rear_rolling_torque_nm), what);
    require(std::isfinite(out.front_bearing_torque_nm), what);
    require(std::isfinite(out.rear_bearing_torque_nm), what);
    require(std::isfinite(out.crank_torque_nm), what);
    require(std::isfinite(out.rear_drive_torque_nm), what);
    require(std::isfinite(out.front_brake_torque_nm), what);
    require(std::isfinite(out.rear_brake_torque_nm), what);
    require(std::isfinite(out.tendon_upper_bound_rad), what);
}

// Regression: zero freehub deflection, engaged branch (phi == prev_bound, rel_omega >= 0, rider torque > 0).
// Rear drive must equal the rider torque transferred by power balance, not rider * ratio * efficiency.
void test_rider_drive_zero_deflection_engaged() {
    const GpuResistanceParams rp{};
    const GpuDrivetrainParams dp{};
    GpuDrivetrainState st{};
    gpu_reset_drivetrain_state(dp, st, 0.0f, 0.0f); // boundary = 0; phi = 0 -> zero deflection

    const GpuDrivetrainOutput out = gpu_compute_drivetrain_and_resistance(
        rp, dp, st,
        /*bike_vx_mps=*/0.0f, /*bike_vz_mps=*/0.0f,
        /*crank_qpos_rad=*/0.0f, /*crank_omega_rad_s=*/5.0f, /*front_wheel_omega_rad_s=*/0.0f,
        /*rear_wheel_qpos_rad=*/0.0f, /*rear_wheel_omega_rad_s=*/0.0f,
        0.0f, 0.0f, 0.34f, 0.34f,
        /*pedal_demand=*/1.0f, 0.0f, 0.0f
    );

    require(st.engaged == 1, "rider_drive_zero_defl_engaged");
    require_rel(
        static_cast<double>(out.rear_drive_torque_nm),
        expected_rider_wheel_torque(dp, static_cast<double>(dp.nominal_crank_torque_nm)),
        1e-5,
        "rider_drive_zero_deflection_engaged"
    );
}

// Regression: deflection > 0 branch where the floor max(pawl_term, rider_term) dominates.
// Deflection 1e-4 rad gives a pawl term of ~0.01 Nm, far below the rider term (~100 Nm).
void test_rider_drive_small_deflection_floor() {
    const GpuResistanceParams rp{};
    const GpuDrivetrainParams dp{};
    GpuDrivetrainState st{};
    gpu_reset_drivetrain_state(dp, st, 0.0f, 0.0f); // boundary = 0

    // phi = ratio * 0 - (-1e-4) = 1e-4 > prev_bound (0) -> deflection = 1e-4 rad.
    const double ratio = static_cast<double>(dp.chainring_teeth) / static_cast<double>(dp.cog_teeth);
    const double rel_omega = ratio * 5.0 - 0.0;
    const double pawl_term = static_cast<double>(dp.efficiency)
        * (1e-4 / static_cast<double>(dp.pawl_engage_rad))
        * (static_cast<double>(dp.freehub_stiffness_nm_rad) * 1e-4
           + static_cast<double>(dp.freehub_damping_nm_s_rad) * rel_omega);
    const double expected = expected_rider_wheel_torque(dp, static_cast<double>(dp.nominal_crank_torque_nm));
    require(pawl_term < 0.01 * expected, "small_deflection_pawl_term_is_minor");

    const GpuDrivetrainOutput out = gpu_compute_drivetrain_and_resistance(
        rp, dp, st,
        /*bike_vx_mps=*/0.0f, /*bike_vz_mps=*/0.0f,
        /*crank_qpos_rad=*/0.0f, /*crank_omega_rad_s=*/5.0f, /*front_wheel_omega_rad_s=*/0.0f,
        /*rear_wheel_qpos_rad=*/-1e-4f, /*rear_wheel_omega_rad_s=*/0.0f,
        0.0f, 0.0f, 0.34f, 0.34f,
        /*pedal_demand=*/1.0f, 0.0f, 0.0f
    );

    require(st.engaged == 1, "rider_drive_small_defl_engaged");
    require_rel(static_cast<double>(out.rear_drive_torque_nm), expected, 1e-5, "rider_drive_small_deflection_floor");
}

// Regression: the cadence taper defaults must be the reference coast/resume thresholds
// (captured_bootstrap.json config.writer_config.drive.pedaling: resume_below_rpm = 105, coast_above_rpm = 120).
void test_cadence_defaults_105_120_rpm() {
    const GpuDrivetrainParams dp{};
    require_near(dp.target_cadence_rad_s, 10.995574f, 1e-6f, "cadence_target_is_105_rpm");
    require_near(dp.max_rider_cadence_rad_s, 12.566371f, 1e-6f, "cadence_max_is_120_rpm");
}

// Regression: rider torque taper. Full torque at 10.0 rad/s, zero at 120 rpm and above,
// linear in between (half torque at the midpoint of the taper band).
void test_cadence_taper_profile() {
    const GpuDrivetrainParams dp{};
    GpuResistanceParams rp{};
    rp.drivetrain_viscous_nm_s_rad = 0.0f; // crank_torque_nm == rider torque

    const auto crank_torque_at = [&](float crank_omega) -> float {
        GpuDrivetrainState st{};
        gpu_reset_drivetrain_state(dp, st, 0.0f, 0.0f);
        const GpuDrivetrainOutput out = gpu_compute_drivetrain_and_resistance(
            rp, dp, st,
            0.0f, 0.0f,
            0.0f, crank_omega, 0.0f,
            0.0f, 0.0f,
            0.0f, 0.0f, 0.34f, 0.34f,
            /*pedal_demand=*/1.0f, 0.0f, 0.0f
        );
        return out.crank_torque_nm;
    };

    require_near(crank_torque_at(10.0f), dp.nominal_crank_torque_nm, 1e-5f, "cadence_full_torque_at_10_rad_s");
    require_near(crank_torque_at(12.566371f), 0.0f, 1e-6f, "cadence_zero_at_120_rpm");
    require_near(crank_torque_at(13.0f), 0.0f, 1e-6f, "cadence_zero_above_120_rpm");
    const float mid = 0.5f * (dp.target_cadence_rad_s + dp.max_rider_cadence_rad_s);
    require_near(crank_torque_at(mid), 0.5f * dp.nominal_crank_torque_nm, 1e-4f, "cadence_linear_midpoint");
}

// Guard: zero chainring teeth gives ratio == 0; the rider-drive division must be guarded so that
// no inf/NaN is produced on any branch (the zero-deflection engaged branch must yield 0).
void test_zero_chainring_outputs_finite() {
    const GpuResistanceParams rp{};
    GpuDrivetrainParams dp{};
    dp.chainring_teeth = 0.0f;
    require_near(gpu_drivetrain_ratio(dp), 0.0f, 0.0f, "zero_chainring_ratio_is_zero");

    // Zero-deflection engaged branch with rider torque > 0.
    {
        GpuDrivetrainState st{};
        gpu_reset_drivetrain_state(dp, st, 0.0f, 0.0f);
        const GpuDrivetrainOutput out = gpu_compute_drivetrain_and_resistance(
            rp, dp, st,
            0.0f, 0.0f,
            0.0f, 5.0f, 0.0f,
            0.0f, 0.0f,
            0.0f, 0.0f, 0.34f, 0.34f,
            1.0f, 0.0f, 0.0f
        );
        require_output_finite(out, "zero_chainring_zero_defl_finite");
        require_near(out.rear_drive_torque_nm, 0.0f, 1e-7f, "zero_chainring_zero_defl_drive");
    }

    // Deflection > 0 branch with rider torque > 0.
    {
        GpuDrivetrainState st{};
        gpu_reset_drivetrain_state(dp, st, 0.0f, 0.0f);
        const GpuDrivetrainOutput out = gpu_compute_drivetrain_and_resistance(
            rp, dp, st,
            0.0f, 0.0f,
            0.0f, 5.0f, 0.0f,
            -1e-4f, 0.0f,
            0.0f, 0.0f, 0.34f, 0.34f,
            1.0f, 0.0f, 0.0f
        );
        require_output_finite(out, "zero_chainring_defl_finite");
    }

    // Qfrc write path (standalone pawl mode) stays finite.
    {
        const GpuModelIndices idx{};
        std::array<float, kMaxNq> qpos{};
        std::array<float, kMaxNv> qvel{};
        std::array<float, kMaxNv> qfrc{};
        qvel[to_index(idx.crank_spin_dof)] = 5.0f;
        GpuDrivetrainState st{};
        gpu_reset_drivetrain_state(dp, st, 0.0f, 0.0f);
        gpu_write_drivetrain_and_resistance_qfrc(
            rp, dp, idx, st, qpos.data(), qvel.data(),
            0.0f, 0.0f, 0.34f, 0.34f, 1.0f, 0.0f, 0.0f, qfrc.data(), true
        );
        for (const float value : qfrc) {
            require(std::isfinite(value), "zero_chainring_qfrc_finite");
        }
    }
}

constexpr std::array<TestCase, 12> cases{{
    {.name = "default_gear_ratio", .run = test_default_gear_ratio},
    {.name = "rolling_moment", .run = test_rolling_moment},
    {.name = "aerodynamic_force", .run = test_aerodynamic_force},
    {.name = "freehub_ratcheting", .run = test_freehub_ratcheting},
    {.name = "authoritative_freehub_parity",
     .run = test_authoritative_freehub_parity},
    {.name = "drivetrain_torque_paths_and_modes",
     .run = test_drivetrain_torque_paths_and_modes},
    {.name = "braking_and_qfrc_write", .run = test_braking_and_qfrc_write},
    {.name = "rider_drive_zero_deflection_engaged",
     .run = test_rider_drive_zero_deflection_engaged},
    {.name = "rider_drive_small_deflection_floor",
     .run = test_rider_drive_small_deflection_floor},
    {.name = "cadence_defaults_105_120_rpm",
     .run = test_cadence_defaults_105_120_rpm},
    {.name = "cadence_taper_profile", .run = test_cadence_taper_profile},
    {.name = "zero_chainring_outputs_finite",
     .run = test_zero_chainring_outputs_finite},
}};

int run_case(const TestCase &test_case) {
    try {
        test_case.run();
        std::cout << "PASS " << test_case.name << '\n';
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << test_case.name << ": " << error.what() << '\n';
        return 1;
    }
}
} // namespace

int main(int argc, char **argv) {
    try {
        const std::span<char *> arguments =
            bike::platform::main_arguments(argc, argv);
        if (arguments.size() == 2 &&
            std::string_view(arguments[1]) == "--list") {
            for (const TestCase &test_case: cases)
                std::cout << test_case.name << '\n';
            return 0;
        }
        if (arguments.size() > 2) {
            std::cerr << "usage: bike_test_gpu_drivetrain_resistance [case-name|--list]\n";
            return 2;
        }

        const std::string_view requested =
            arguments.size() == 1 ? "all" : std::string_view(arguments[1]);
        bool found = requested == "all";
        int status = 0;
        for (const TestCase &test_case: cases) {
            if (requested == "all" || requested == test_case.name) {
                found = true;
                if (run_case(test_case) != 0)
                    status = 1;
            }
        }
        if (!found) {
            std::cerr << "unknown gpu drivetrain resistance case: " << requested << '\n';
            return 2;
        }
        return status;
    } catch (const std::exception &error) {
        // A throwing argument envelope is a harness failure, not a usage error.
        std::cerr << "FAIL argument handling: " << error.what() << '\n';
        return 1;
    }
}
