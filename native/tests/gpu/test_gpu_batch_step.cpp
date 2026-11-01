// native/tests/gpu/test_gpu_batch_step.cpp — Standalone C++23 host verification
// for the unified batched GPU physics step (`batch_step.hpp` and `cuda_kernels.cu`).

#include "../../src/gpu/batch_step.hpp"
#include "../../src/gpu/cuda_kernels.cu"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {

void require_true(bool cond, const char *msg, int line) {
    if (!cond) {
        std::fprintf(stderr, "FAIL [line %d]: %s\n", line, msg);
        std::exit(1);
    }
}

#define REQUIRE(cond) require_true((cond), #cond, __LINE__)

} // namespace

int main() {
    using namespace bike::gpu;

    constexpr int kNWorld = 64;
    constexpr int kNumSteps = 10;
    constexpr int kHfSamples = 4000;
    constexpr float kDt = 0.001f;

    // 1. Verify C API struct size queries and default config initialization
    REQUIRE(bike_gpu_sizeof_world_config() == static_cast<int>(sizeof(GpuWorldConfig)));
    REQUIRE(bike_gpu_sizeof_world_state() == static_cast<int>(sizeof(GpuWorldState)));
    REQUIRE(bike_gpu_sizeof_model_indices() == static_cast<int>(sizeof(GpuModelIndices)));

    GpuModelIndices idx{};

    // 2. Build a 4000-sample synthetic rough heightfield (x in [-5.0, 195.0], dx = 0.05 m)
    std::vector<float> hf_heights(kHfSamples, 0.0f);
    constexpr float kHfXMin = -5.0f;
    constexpr float kHfDx = 0.05f;
    for (int i = 0; i < kHfSamples; ++i) {
        const float x = kHfXMin + static_cast<float>(i) * kHfDx;
        hf_heights[static_cast<std::size_t>(i)] =
            0.015f * std::sin(1.7f * x) + 0.008f * std::cos(5.3f * x);
    }
    const GpuHeightfield1D hf{
        hf_heights.data(),
        kHfSamples,
        kHfXMin,
        kHfDx,
        1.0f / kHfDx
    };

    // 3. Initialize 64 heterogeneous GpuWorldConfig entries:
    //    - First 32 worlds: 32x1 tire rays (32 rays/wheel)
    //    - Next 32 worlds:  45x10 tire rays (450 rays/wheel)
    //    - Varying fork air pressures, rebound scales, rear shock spring rates, and tire stiffness
    std::vector<float> fork_psi_scale(kNWorld, 1.0f);
    std::vector<float> rebound_scale(kNWorld, 1.0f);
    std::vector<float> shock_rate_scale(kNWorld, 1.0f);
    std::vector<float> tire_stiff_scale(kNWorld, 1.0f);

    for (int w = 0; w < kNWorld; ++w) {
        const float u = static_cast<float>(w) / static_cast<float>(kNWorld - 1);
        fork_psi_scale[static_cast<std::size_t>(w)] = 0.75f + 0.50f * u;    // [0.75, 1.25]
        rebound_scale[static_cast<std::size_t>(w)] = 0.70f + 0.60f * u;     // [0.70, 1.30]
        shock_rate_scale[static_cast<std::size_t>(w)] = 0.80f + 0.45f * u;  // [0.80, 1.25]
        tire_stiff_scale[static_cast<std::size_t>(w)] = 0.85f + 0.35f * u;  // [0.85, 1.20]
    }

    std::vector<GpuWorldConfig> configs(kNWorld);
    bike_gpu_build_sweep_configs(
        configs.data(),
        kNWorld,
        fork_psi_scale.data(),
        rebound_scale.data(),
        shock_rate_scale.data(),
        tire_stiff_scale.data(),
        32,
        1
    );

    // Configure worlds [32..63] to use 45x10 = 450 rays per wheel
    for (int w = 32; w < kNWorld; ++w) {
        configs[static_cast<std::size_t>(w)].front_tire.num_stations = 45;
        configs[static_cast<std::size_t>(w)].front_tire.num_lateral_ribs = 10;
        configs[static_cast<std::size_t>(w)].rear_tire.num_stations = 45;
        configs[static_cast<std::size_t>(w)].rear_tire.num_lateral_ribs = 10;
    }

    // Verify preload initialization and parameter scaling
    REQUIRE(configs[0].suspension.fork_preload_n > 0.0f);
    REQUIRE(configs[0].suspension.shock_preload_n > 0.0f);
    REQUIRE(configs[63].suspension.fork_preload_n > configs[0].suspension.fork_preload_n);
    REQUIRE(configs[63].suspension.shock_preload_n > configs[0].suspension.shock_preload_n);

    // 4. Allocate batched state and MuJoCo-like kinematics buffers for 64 worlds
    std::vector<GpuWorldState> states(kNWorld);
    std::vector<float> qpos_batch(static_cast<std::size_t>(kNWorld * idx.nq), 0.0f);
    std::vector<float> qvel_batch(static_cast<std::size_t>(kNWorld * idx.nv), 0.0f);
    std::vector<float> xpos_batch(static_cast<std::size_t>(kNWorld * idx.nbody * 3), 0.0f);
    std::vector<float> qfrc_batch(static_cast<std::size_t>(kNWorld * idx.nv), 0.0f);
    std::vector<float> ctrl_batch(static_cast<std::size_t>(kNWorld * idx.nu), 0.0f);
    std::vector<float> tendon_range_batch(static_cast<std::size_t>(kNWorld * 2), 0.0f);

    // Populate realistic initial bike kinematics across all 64 worlds
    for (int w = 0; w < kNWorld; ++w) {
        float *qpos_w = qpos_batch.data() + w * idx.nq;
        float *qvel_w = qvel_batch.data() + w * idx.nv;
        float *xpos_w = xpos_batch.data() + w * (idx.nbody * 3);

        qpos_w[idx.root_x_qpos] = 2.0f;
        qpos_w[idx.root_z_qpos] = 0.55f;
        qpos_w[idx.root_pitch_qpos] = 0.02f;
        qpos_w[idx.crank_spin_qpos] = 0.40f;
        qpos_w[idx.fork_travel_qpos] = 0.045f;   // 45 mm fork compression
        qpos_w[idx.front_wheel_spin_qpos] = 1.10f;
        qpos_w[idx.rear_wheel_spin_qpos] = 0.70f;
        qpos_w[idx.shock_stroke_qpos] = 0.022f;  // 22 mm shock stroke

        // Place rider joints near posture targets, with hip slightly into soft ROM margin
        qpos_w[idx.rider_qpos[0]] = -2.02f; // near -120 deg hip lower ROM margin
        qpos_w[idx.rider_qpos[1]] = -0.95f;
        qpos_w[idx.rider_qpos[2]] = 0.12f;
        qpos_w[idx.rider_qpos[3]] = 0.32f;
        qpos_w[idx.rider_qpos[4]] = 0.80f;
        qpos_w[idx.rider_qpos[5]] = 0.50f;

        qvel_w[idx.root_x_dof] = 6.5f;
        qvel_w[idx.root_z_dof] = -0.15f;
        qvel_w[idx.root_pitch_dof] = 0.05f;
        qvel_w[idx.crank_spin_dof] = 7.5f;
        qvel_w[idx.fork_travel_dof] = 0.35f;
        qvel_w[idx.front_wheel_spin_dof] = 18.5f;
        qvel_w[idx.rear_wheel_spin_dof] = 18.8f;
        qvel_w[idx.shock_stroke_dof] = 0.20f;

        const float front_x = 2.60f;
        const float rear_x = 1.40f;
        const float front_ground_z = gpu_sample_heightfield_1d(hf, front_x).x;
        const float rear_ground_z = gpu_sample_heightfield_1d(hf, rear_x).x;

        // Frame CoM
        xpos_w[3 * idx.frame_bid + 0] = 2.00f;
        xpos_w[3 * idx.frame_bid + 2] = 0.62f;

        // Front wheel center embedded ~14 mm into terrain
        xpos_w[3 * idx.front_wheel_bid + 0] = front_x;
        xpos_w[3 * idx.front_wheel_bid + 2] =
            front_ground_z + configs[static_cast<std::size_t>(w)].front_tire.nominal_radius_m - 0.014f;

        // Rear wheel center embedded ~16 mm into terrain
        xpos_w[3 * idx.rear_wheel_bid + 0] = rear_x;
        xpos_w[3 * idx.rear_wheel_bid + 2] =
            rear_ground_z + configs[static_cast<std::size_t>(w)].rear_tire.nominal_radius_m - 0.016f;

        tendon_range_batch[static_cast<std::size_t>(w * 2 + 0)] = -1.0e6f;
        tendon_range_batch[static_cast<std::size_t>(w * 2 + 1)] = 0.0f;
    }

    // Initialize world states using gpu_init_world_state
    for (int w = 0; w < kNWorld; ++w) {
        const float *qpos_w = qpos_batch.data() + w * idx.nq;
        gpu_init_world_state(
            states[static_cast<std::size_t>(w)],
            configs[static_cast<std::size_t>(w)].drivetrain,
            qpos_w[idx.crank_spin_qpos],
            qpos_w[idx.rear_wheel_spin_qpos]
        );
    }

    const float initial_bound_w0 = states[0].drivetrain.freehub_boundary_rad;

    // 5. Step gpu_step_world_writers across all 64 worlds for 10 steps
    for (int step = 0; step < kNumSteps; ++step) {
        for (int w = 0; w < kNWorld; ++w) {
            gpu_step_world_writers(
                w,
                kDt,
                idx,
                hf,
                configs.data(),
                /*per_world_config=*/1,
                states.data(),
                qpos_batch.data(),
                qvel_batch.data(),
                xpos_batch.data(),
                qfrc_batch.data(),
                ctrl_batch.data(),
                tendon_range_batch.data()
            );
        }

        // Advance kinematics slightly each step: first 5 steps coasting (rear wheel faster than crank
        // so freehub ratchets downward), next 5 steps pedaling hard (crank catching up to bound).
        for (int w = 0; w < kNWorld; ++w) {
            float *qpos_w = qpos_batch.data() + w * idx.nq;
            float *qvel_w = qvel_batch.data() + w * idx.nv;
            float *xpos_w = xpos_batch.data() + w * (idx.nbody * 3);

            const float dx = qvel_w[idx.root_x_dof] * kDt;
            qpos_w[idx.root_x_qpos] += dx;
            xpos_w[3 * idx.frame_bid + 0] += dx;
            xpos_w[3 * idx.front_wheel_bid + 0] += dx;
            xpos_w[3 * idx.rear_wheel_bid + 0] += dx;

            const float f_ground = gpu_sample_heightfield_1d(hf, xpos_w[3 * idx.front_wheel_bid + 0]).x;
            const float r_ground = gpu_sample_heightfield_1d(hf, xpos_w[3 * idx.rear_wheel_bid + 0]).x;
            xpos_w[3 * idx.front_wheel_bid + 2] =
                f_ground + configs[static_cast<std::size_t>(w)].front_tire.nominal_radius_m - 0.014f;
            xpos_w[3 * idx.rear_wheel_bid + 2] =
                r_ground + configs[static_cast<std::size_t>(w)].rear_tire.nominal_radius_m - 0.016f;

            if (step < 5) {
                // Coasting: rear wheel spins faster than ratio * crank
                qpos_w[idx.crank_spin_qpos] += 2.0f * kDt;
                qpos_w[idx.rear_wheel_spin_qpos] += 20.0f * kDt;
            } else {
                // Pedaling: crank advances faster to engage pawls against ratcheted bound
                qpos_w[idx.crank_spin_qpos] += 14.0f * kDt;
                qpos_w[idx.rear_wheel_spin_qpos] += 10.0f * kDt;
            }
        }
    }

    // 6. Verify all qfrc_applied, ctrl, and tendon_range outputs are finite and non-trivial
    for (int w = 0; w < kNWorld; ++w) {
        const float *qfrc_w = qfrc_batch.data() + w * idx.nv;
        const float *ctrl_w = ctrl_batch.data() + w * idx.nu;

        for (int i = 0; i < idx.nv; ++i) {
            REQUIRE(std::isfinite(qfrc_w[i]));
        }
        for (int i = 0; i < idx.nu; ++i) {
            REQUIRE(std::isfinite(ctrl_w[i]));
        }
        const float upper_bound = tendon_range_batch[static_cast<std::size_t>(w * 2 + 1)];
        REQUIRE(std::isfinite(upper_bound));

        // Suspension forces must oppose positive compression
        REQUIRE(qfrc_w[idx.fork_travel_dof] < -100.0f);
        REQUIRE(qfrc_w[idx.shock_stroke_dof] < -100.0f);

        // Tire vertical ground reaction force on root_z must be strongly positive
        REQUIRE(qfrc_w[idx.root_z_dof] > 200.0f);

        // Crank torque must be positive under pedaling demand
        REQUIRE(qfrc_w[idx.crank_spin_dof] > 1.0f);

        // Rider posture/pedaling actuators must be active on [6..11]
        for (int j = 0; j < 6; ++j) {
            REQUIRE(std::fabs(ctrl_w[idx.rider_ctrl[j]]) > 1e-3f);
        }

        // Hip soft-edge torque must be non-zero since hip is placed inside lower ROM margin
        REQUIRE(qfrc_w[idx.rider_dof[0]] > 0.0f);

        // Freehub upper bound must have ratcheted downward during coasting steps
        REQUIRE(upper_bound < initial_bound_w0 - 1e-3f);
    }

    // 7. Verify distinct worlds reflect their parameter sweep differences:
    //    - World 63 has higher fork pressure & shock spring rate than World 0
    //    - World 31 (32x1 rays) vs World 32 (45x10 = 450 rays) produce distinct tire forces
    const float *qfrc_w0 = qfrc_batch.data() + 0 * idx.nv;
    const float *qfrc_w31 = qfrc_batch.data() + 31 * idx.nv;
    const float *qfrc_w32 = qfrc_batch.data() + 32 * idx.nv;
    const float *qfrc_w63 = qfrc_batch.data() + 63 * idx.nv;

    REQUIRE(std::fabs(qfrc_w63[idx.fork_travel_dof]) > std::fabs(qfrc_w0[idx.fork_travel_dof]) + 50.0f);
    REQUIRE(std::fabs(qfrc_w63[idx.shock_stroke_dof]) > std::fabs(qfrc_w0[idx.shock_stroke_dof]) + 50.0f);
    REQUIRE(std::fabs(qfrc_w63[idx.root_z_dof] - qfrc_w0[idx.root_z_dof]) > 10.0f);
    REQUIRE(std::fabs(qfrc_w32[idx.root_z_dof] - qfrc_w31[idx.root_z_dof]) > 1.0f);

    // 8. Also verify the extern "C" host launch wrappers on CPU fallback
    bike_gpu_launch_init_states(
        nullptr,
        kNWorld,
        &idx,
        configs.data(),
        1,
        states.data(),
        qpos_batch.data()
    );
    bike_gpu_launch_writers(
        nullptr,
        kNWorld,
        kDt,
        &idx,
        hf_heights.data(),
        kHfSamples,
        kHfXMin,
        kHfDx,
        configs.data(),
        1,
        states.data(),
        qpos_batch.data(),
        qvel_batch.data(),
        xpos_batch.data(),
        qfrc_batch.data(),
        ctrl_batch.data(),
        tendon_range_batch.data(),
        2
    );
    REQUIRE(std::isfinite(qfrc_batch[idx.root_z_dof]));

    std::printf(
        "PASS: test_gpu_batch_step verified %d heterogeneous worlds x %d steps "
        "(fork_w0=%.1f N, fork_w63=%.1f N, shock_w0=%.1f N, shock_w63=%.1f N, "
        "fz_w0=%.1f N, fz_w32_450rays=%.1f N, tendon_ub=%.4f rad)\n",
        kNWorld,
        kNumSteps,
        qfrc_w0[idx.fork_travel_dof],
        qfrc_w63[idx.fork_travel_dof],
        qfrc_w0[idx.shock_stroke_dof],
        qfrc_w63[idx.shock_stroke_dof],
        qfrc_w0[idx.root_z_dof],
        qfrc_w32[idx.root_z_dof],
        tendon_range_batch[1]
    );
    return 0;
}
