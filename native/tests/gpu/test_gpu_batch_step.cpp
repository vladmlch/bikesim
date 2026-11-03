// native/tests/gpu/test_gpu_batch_step.cpp — Standalone C++23 host verification
// for the unified batched GPU physics step (`batch_step.hpp` and `cuda_kernels.cu`):
// C ABI size queries, GpuModelIndices defaults, a 64-world heterogeneous
// parameter sweep stepped on the host kernel path, and the extern "C" launch
// wrappers' bounds validation.
#include "../../src/gpu/batch_step.hpp"
// The extern "C" host entry points come from bike_gpu_kernels, the one
// host compile of cuda_kernels.cu: the ODR sweep links every selected TU into
// one image, so a textual #include of the .cu here would define them twice.
#include "../../src/gpu/cuda_kernels_abi.hpp"
#include "../../src/platform/argv.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

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

// GpuModelIndices addresses are int for the kernel ABI (a disabled slot is
// -1); container subscripts take size_t, so the explicit conversion lives in
// one place.
constexpr std::size_t to_index(int field) {
    return static_cast<std::size_t>(field);
}

constexpr int kNWorld = 64;
constexpr int kNumSteps = 10;
constexpr int kHfSamples = 4000;
constexpr float kDt = 0.001f;
constexpr float kHfXMin = -5.0f;
constexpr float kHfDx = 0.05f;

// Shared scenario buffers for the batch-step cases: the heterogeneous sweep
// configs, world states, flat batch kinematics/force arrays and the tendon
// range block, all sized by the default model index counts.
struct BatchScenario {
    GpuModelIndices idx{};
    std::vector<float> hf_heights;
    std::vector<GpuWorldConfig> configs;
    std::vector<GpuWorldState> states;
    std::vector<float> qpos_batch;
    std::vector<float> qvel_batch;
    std::vector<float> xpos_batch;
    std::vector<float> qfrc_batch;
    std::vector<float> ctrl_batch;
    std::vector<float> tendon_range_batch;
    float initial_bound_w0 = 0.0f;

    // The heightfield view is rebuilt on each call so it always points at
    // this scenario's own buffer, even if the struct was moved or copied.
    GpuHeightfield1D heightfield() const {
        return GpuHeightfield1D{
            .heights_m = hf_heights.data(),
            .num_samples = kHfSamples,
            .x_min_m = kHfXMin,
            .dx_m = kHfDx,
            .inv_dx_m = 1.0f / kHfDx
        };
    }
};

// 64 heterogeneous GpuWorldConfig entries:
//   - First 32 worlds: 32x1 tire rays (32 rays/wheel)
//   - Next 32 worlds:  45x10 tire rays (450 rays/wheel)
//   - Varying fork air pressures, rebound scales, rear shock spring rates,
//     and tire stiffness
std::vector<GpuWorldConfig> make_sweep_configs() {
    std::vector<float> fork_psi_scale(to_index(kNWorld), 1.0f);
    std::vector<float> rebound_scale(to_index(kNWorld), 1.0f);
    std::vector<float> shock_rate_scale(to_index(kNWorld), 1.0f);
    std::vector<float> tire_stiff_scale(to_index(kNWorld), 1.0f);

    for (int w = 0; w < kNWorld; ++w) {
        const float u = static_cast<float>(w) / static_cast<float>(kNWorld - 1);
        fork_psi_scale[to_index(w)] = 0.75f + 0.50f * u;   // [0.75, 1.25]
        rebound_scale[to_index(w)] = 0.70f + 0.60f * u;    // [0.70, 1.30]
        shock_rate_scale[to_index(w)] = 0.80f + 0.45f * u; // [0.80, 1.25]
        tire_stiff_scale[to_index(w)] = 0.85f + 0.35f * u; // [0.85, 1.20]
    }

    std::vector<GpuWorldConfig> configs(static_cast<std::size_t>(kNWorld));
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
        configs[to_index(w)].front_tire.num_stations = 45;
        configs[to_index(w)].front_tire.num_lateral_ribs = 10;
        configs[to_index(w)].rear_tire.num_stations = 45;
        configs[to_index(w)].rear_tire.num_lateral_ribs = 10;
    }
    return configs;
}

// Sections 2–5 of the original single-flow test: builds the heightfield and
// sweep configs, populates realistic initial bike kinematics across all 64
// worlds, initializes world states, then runs gpu_step_world_writers for 10
// steps — the first 5 coasting (rear wheel faster than crank, so the freehub
// ratchets downward), the next 5 pedaling hard (crank catching up to bound).
BatchScenario make_batch_scenario() {
    BatchScenario sc;
    const GpuModelIndices &idx = sc.idx;
    const std::size_t nq = to_index(idx.nq);
    const std::size_t nv = to_index(idx.nv);
    const std::size_t nbody3 = 3 * to_index(idx.nbody);

    // 2. Build a 4000-sample synthetic rough heightfield (x in [-5.0, 195.0],
    //    dx = 0.05 m)
    sc.hf_heights.assign(to_index(kHfSamples), 0.0f);
    for (int i = 0; i < kHfSamples; ++i) {
        const float x = kHfXMin + static_cast<float>(i) * kHfDx;
        sc.hf_heights[to_index(i)] =
            0.015f * std::sin(1.7f * x) + 0.008f * std::cos(5.3f * x);
    }
    const GpuHeightfield1D hf = sc.heightfield();

    // 3. Heterogeneous per-world configs (see make_sweep_configs).
    sc.configs = make_sweep_configs();

    // 4. Allocate batched state and MuJoCo-like kinematics buffers for
    //    64 worlds
    sc.states.resize(static_cast<std::size_t>(kNWorld));
    sc.qpos_batch.assign(static_cast<std::size_t>(kNWorld) * nq, 0.0f);
    sc.qvel_batch.assign(static_cast<std::size_t>(kNWorld) * nv, 0.0f);
    sc.xpos_batch.assign(static_cast<std::size_t>(kNWorld) * nbody3, 0.0f);
    sc.qfrc_batch.assign(static_cast<std::size_t>(kNWorld) * nv, 0.0f);
    sc.ctrl_batch.assign(
        static_cast<std::size_t>(kNWorld) * to_index(idx.nu), 0.0f);
    sc.tendon_range_batch.assign(static_cast<std::size_t>(kNWorld) * 2, 0.0f);

    const std::span<float> qpos_rows{sc.qpos_batch};
    const std::span<float> qvel_rows{sc.qvel_batch};
    const std::span<float> xpos_rows{sc.xpos_batch};
    const auto rider_qpos = std::span{idx.rider_qpos};

    // Populate realistic initial bike kinematics across all 64 worlds
    for (int w = 0; w < kNWorld; ++w) {
        const std::size_t wi = to_index(w);
        const std::span<float> qpos_w = qpos_rows.subspan(wi * nq, nq);
        const std::span<float> qvel_w = qvel_rows.subspan(wi * nv, nv);
        const std::span<float> xpos_w = xpos_rows.subspan(wi * nbody3, nbody3);

        qpos_w[to_index(idx.root_x_qpos)] = 2.0f;
        qpos_w[to_index(idx.root_z_qpos)] = 0.55f;
        qpos_w[to_index(idx.root_pitch_qpos)] = 0.02f;
        qpos_w[to_index(idx.crank_spin_qpos)] = 0.40f;
        qpos_w[to_index(idx.fork_travel_qpos)] = 0.045f;   // 45 mm fork compression
        qpos_w[to_index(idx.front_wheel_spin_qpos)] = 1.10f;
        qpos_w[to_index(idx.rear_wheel_spin_qpos)] = 0.70f;
        qpos_w[to_index(idx.shock_stroke_qpos)] = 0.022f;  // 22 mm shock stroke

        // Place rider joints away from posture targets, with hip_front into soft ROM margin
        qpos_w[to_index(rider_qpos[0])] = -2.02f; // below -25 deg hip_front lower ROM margin
        qpos_w[to_index(rider_qpos[1])] = 0.70f;
        qpos_w[to_index(rider_qpos[2])] = -0.35f;
        qpos_w[to_index(rider_qpos[3])] = 0.70f;
        qpos_w[to_index(rider_qpos[4])] = 0.45f;
        qpos_w[to_index(rider_qpos[5])] = 0.35f;
        qpos_w[to_index(rider_qpos[6])] = 0.55f;
        qpos_w[to_index(rider_qpos[7])] = 0.35f;
        qpos_w[to_index(rider_qpos[8])] = 0.55f;

        qvel_w[to_index(idx.root_x_dof)] = 6.5f;
        qvel_w[to_index(idx.root_z_dof)] = -0.15f;
        qvel_w[to_index(idx.root_pitch_dof)] = 0.05f;
        qvel_w[to_index(idx.crank_spin_dof)] = 7.5f;
        qvel_w[to_index(idx.fork_travel_dof)] = 0.35f;
        qvel_w[to_index(idx.front_wheel_spin_dof)] = 18.5f;
        qvel_w[to_index(idx.rear_wheel_spin_dof)] = 18.8f;
        qvel_w[to_index(idx.shock_stroke_dof)] = 0.20f;

        const float front_x = 2.60f;
        const float rear_x = 1.40f;
        const float front_ground_z = gpu_sample_heightfield_1d(hf, front_x).x;
        const float rear_ground_z = gpu_sample_heightfield_1d(hf, rear_x).x;

        // Frame CoM
        xpos_w[3 * to_index(idx.frame_body) + 0] = 2.00f;
        xpos_w[3 * to_index(idx.frame_body) + 2] = 0.62f;

        // Front wheel center embedded ~14 mm into terrain
        xpos_w[3 * to_index(idx.front_wheel_body) + 0] = front_x;
        xpos_w[3 * to_index(idx.front_wheel_body) + 2] =
            front_ground_z + sc.configs[wi].front_tire.nominal_radius_m - 0.014f;

        // Rear wheel center embedded ~16 mm into terrain
        xpos_w[3 * to_index(idx.rear_wheel_body) + 0] = rear_x;
        xpos_w[3 * to_index(idx.rear_wheel_body) + 2] =
            rear_ground_z + sc.configs[wi].rear_tire.nominal_radius_m - 0.016f;

        sc.tendon_range_batch[wi * 2 + 0] = -1.0e6f;
        sc.tendon_range_batch[wi * 2 + 1] = 0.0f;
    }

    // Initialize world states using gpu_init_world_state
    for (int w = 0; w < kNWorld; ++w) {
        const std::size_t wi = to_index(w);
        const std::span<const float> qpos_w = qpos_rows.subspan(wi * nq, nq);
        gpu_init_world_state(
            sc.states[wi],
            sc.configs[wi].drivetrain,
            qpos_w[to_index(idx.crank_spin_qpos)],
            qpos_w[to_index(idx.rear_wheel_spin_qpos)]
        );
    }

    sc.initial_bound_w0 = sc.states[0].drivetrain.freehub_boundary_rad;

    // 5. Step gpu_step_world_writers across all 64 worlds for 10 steps
    for (int step = 0; step < kNumSteps; ++step) {
        for (int w = 0; w < kNWorld; ++w) {
            gpu_step_world_writers(
                w,
                kDt,
                idx,
                hf,
                sc.configs.data(),
                /*per_world_config=*/1,
                sc.states.data(),
                sc.qpos_batch.data(),
                sc.qvel_batch.data(),
                sc.xpos_batch.data(),
                sc.qfrc_batch.data(),
                sc.ctrl_batch.data(),
                sc.tendon_range_batch.data()
            );
        }

        // Advance kinematics slightly each step: first 5 steps coasting (rear
        // wheel faster than crank so freehub ratchets downward), next 5 steps
        // pedaling hard (crank catching up to bound).
        for (int w = 0; w < kNWorld; ++w) {
            const std::size_t wi = to_index(w);
            const std::span<float> qpos_w = qpos_rows.subspan(wi * nq, nq);
            const std::span<float> qvel_w = qvel_rows.subspan(wi * nv, nv);
            const std::span<float> xpos_w =
                xpos_rows.subspan(wi * nbody3, nbody3);

            const float dx = qvel_w[to_index(idx.root_x_dof)] * kDt;
            qpos_w[to_index(idx.root_x_qpos)] += dx;
            xpos_w[3 * to_index(idx.frame_body) + 0] += dx;
            xpos_w[3 * to_index(idx.front_wheel_body) + 0] += dx;
            xpos_w[3 * to_index(idx.rear_wheel_body) + 0] += dx;

            const float f_ground = gpu_sample_heightfield_1d(
                hf, xpos_w[3 * to_index(idx.front_wheel_body) + 0]).x;
            const float r_ground = gpu_sample_heightfield_1d(
                hf, xpos_w[3 * to_index(idx.rear_wheel_body) + 0]).x;
            xpos_w[3 * to_index(idx.front_wheel_body) + 2] =
                f_ground + sc.configs[wi].front_tire.nominal_radius_m - 0.014f;
            xpos_w[3 * to_index(idx.rear_wheel_body) + 2] =
                r_ground + sc.configs[wi].rear_tire.nominal_radius_m - 0.016f;

            if (step < 5) {
                // Coasting: rear wheel spins faster than ratio * crank
                qpos_w[to_index(idx.crank_spin_qpos)] += 2.0f * kDt;
                qpos_w[to_index(idx.rear_wheel_spin_qpos)] += 20.0f * kDt;
            } else {
                // Pedaling: crank advances faster than rear_wheel / ratio
                // (ratio = 22/51) to engage pawls
                qpos_w[to_index(idx.crank_spin_qpos)] += 28.0f * kDt;
                qpos_w[to_index(idx.rear_wheel_spin_qpos)] += 8.0f * kDt;
            }
        }
    }
    return sc;
}

// Section 1: C API struct size queries, GpuModelIndices defaults, gpu_tanh
// clamping, and the default drivetrain gear ratio.
void test_api_sizes_and_model_defaults() {
    require(bike_gpu_sizeof_world_config() ==
                static_cast<int>(sizeof(GpuWorldConfig)),
            "C API world config size matches sizeof(GpuWorldConfig)");
    require(bike_gpu_sizeof_world_state() ==
                static_cast<int>(sizeof(GpuWorldState)),
            "C API world state size matches sizeof(GpuWorldState)");
    require(bike_gpu_sizeof_model_indices() ==
                static_cast<int>(sizeof(GpuModelIndices)),
            "C API model indices size matches sizeof(GpuModelIndices)");

    const GpuModelIndices idx{};
    require(idx.nq == 26, "model has 26 qpos slots");
    require(idx.nv == 26, "model has 26 qvel DOFs");
    require(idx.nu == 12, "model has 12 actuators");
    require(idx.nbody == 28, "model has 28 bodies");
    require(idx.main_pivot_dof == 8, "main pivot DOF address is 8");
    require(idx.horst_pivot_dof == 9, "horst pivot DOF address is 9");
    require(idx.freehub_tendon == 0, "freehub tendon index is 0");

    constexpr std::array<int, kNumRiderJoints> kExpectedRiderQpos{
        {22, 23, 24, 25, 17, 18, 19, 20, 21}};
    constexpr std::array<int, kNumRiderJoints> kExpectedRiderDof{
        {22, 23, 24, 25, 17, 18, 19, 20, 21}};
    constexpr std::array<int, kNumRiderJoints> kExpectedRiderCtrl{
        {8, 9, 10, 11, 3, 4, 5, 6, 7}};
    const auto rider_qpos = std::span{idx.rider_qpos};
    const auto rider_dof = std::span{idx.rider_dof};
    const auto rider_ctrl = std::span{idx.rider_ctrl};
    for (int j = 0; j < kNumRiderJoints; ++j) {
        require(rider_qpos[to_index(j)] == kExpectedRiderQpos[to_index(j)],
                "rider_qpos matches the model joint order");
        require(rider_dof[to_index(j)] == kExpectedRiderDof[to_index(j)],
                "rider_dof matches the model joint order");
        require(rider_ctrl[to_index(j)] == kExpectedRiderCtrl[to_index(j)],
                "rider_ctrl matches the model actuator order");
    }

    // Verify gpu_tanh clamps extreme inputs to [-15, 15] on host/CUDA/Metal
    // uniformly
    require(gpu_tanh(100.0f) == gpu_tanh(15.0f),
            "gpu_tanh clamps +100 to the +15 clamp bound");
    require(gpu_tanh(-100.0f) == gpu_tanh(-15.0f),
            "gpu_tanh clamps -100 to the -15 clamp bound");
    require(std::isfinite(gpu_tanh(1.0e20f)),
            "gpu_tanh of +1e20 stays finite");
    require(std::isfinite(gpu_tanh(-1.0e20f)),
            "gpu_tanh of -1e20 stays finite");

    GpuWorldConfig default_cfg{};
    bike_gpu_init_default_config(&default_cfg);
    require_near(default_cfg.drivetrain.chainring_teeth, 22.0f, 1e-6f,
                 "default chainring is 22 teeth");
    require_near(default_cfg.drivetrain.cog_teeth, 51.0f, 1e-6f,
                 "default cog is 51 teeth");
    require_near(gpu_drivetrain_ratio(default_cfg.drivetrain), 22.0f / 51.0f,
                 1e-6f, "default drivetrain ratio is 22/51");
}

// Section 3 assertions: preload initialization and parameter scaling across
// the heterogeneous sweep configs.
void test_sweep_config_preloads() {
    const std::vector<GpuWorldConfig> configs = make_sweep_configs();

    require(configs[0].suspension.fork_preload_n > 0.0f,
            "world 0 fork preload is initialized positive");
    require(configs[0].suspension.shock_preload_n > 0.0f,
            "world 0 shock preload is initialized positive");
    require(configs[63].suspension.fork_preload_n >
                configs[0].suspension.fork_preload_n,
            "world 63 fork preload exceeds world 0 (pressure sweep)");
    require(configs[63].suspension.shock_preload_n >
                configs[0].suspension.shock_preload_n,
            "world 63 shock preload exceeds world 0 (rate sweep)");
}

// Section 6: after the 10-step run all qfrc_applied, ctrl, and tendon_range
// outputs are finite and non-trivial in every world.
void test_stepped_world_outputs() {
    const BatchScenario sc = make_batch_scenario();
    const GpuModelIndices &idx = sc.idx;
    const std::size_t nq = to_index(idx.nq);
    const std::size_t nv = to_index(idx.nv);
    const std::size_t nu = to_index(idx.nu);
    const std::span<const float> qpos_rows{sc.qpos_batch};
    const std::span<const float> qvel_rows{sc.qvel_batch};
    const std::span<const float> qfrc_rows{sc.qfrc_batch};
    const std::span<const float> ctrl_rows{sc.ctrl_batch};
    const auto rider_dof = std::span{idx.rider_dof};
    const auto rider_ctrl = std::span{idx.rider_ctrl};

    for (int w = 0; w < kNWorld; ++w) {
        const std::size_t wi = to_index(w);
        const std::span<const float> qpos_w =
            qpos_rows.subspan(wi * nq, nq);
        const std::span<const float> qvel_w =
            qvel_rows.subspan(wi * nv, nv);
        const std::span<const float> qfrc_w =
            qfrc_rows.subspan(wi * nv, nv);
        const std::span<const float> ctrl_w =
            ctrl_rows.subspan(wi * nu, nu);

        for (int i = 0; i < idx.nv; ++i) {
            require(std::isfinite(qfrc_w[to_index(i)]),
                    "qfrc_applied is finite for every DOF");
        }
        for (int i = 0; i < idx.nu; ++i) {
            require(std::isfinite(ctrl_w[to_index(i)]),
                    "ctrl is finite for every actuator");
        }
        const float upper_bound = sc.tendon_range_batch[wi * 2 + 1];
        require(std::isfinite(upper_bound),
                "tendon upper bound is finite");

        // 6a. qfrc[fork_travel_dof] receives BOTH suspension force AND
        //     J^T-projected front tire ground force
        std::array<float, kMaxNv> qfrc_susp_only{};
        gpu_write_suspension_qfrc(
            sc.configs[wi].suspension, idx,
            qpos_w.data(), qvel_w.data(), qfrc_susp_only.data());
        require(qfrc_susp_only[to_index(idx.fork_travel_dof)] < -100.0f,
                "suspension-only fork DOF force is strongly negative");
        require(std::fabs(qfrc_w[to_index(idx.fork_travel_dof)] -
                          qfrc_susp_only[to_index(idx.fork_travel_dof)]) >
                    10.0f,
                "tire J^T contribution moves the fork DOF off suspension-only");
        require(qfrc_w[to_index(idx.shock_stroke_dof)] < -100.0f,
                "shock stroke DOF force is strongly negative");

        // 6b. Rear suspension linkage pivots (main_pivot_dof=8,
        //     horst_pivot_dof=9) receive J^T-projected rear tire forces
        require(std::fabs(qfrc_w[to_index(idx.main_pivot_dof)]) > 1.0f,
                "main pivot DOF receives rear tire torque");
        require(std::fabs(qfrc_w[to_index(idx.horst_pivot_dof)]) > 1.0f,
                "horst pivot DOF receives rear tire torque");

        // Tire vertical ground reaction force on root_z must be strongly
        // positive
        require(qfrc_w[to_index(idx.root_z_dof)] > 200.0f,
                "root_z tire ground reaction is strongly positive");

        // Crank torque must be positive under pedaling demand
        require(qfrc_w[to_index(idx.crank_spin_dof)] > 1.0f,
                "crank torque is positive under pedaling demand");

        // 6c. Rider actuator contract: legs (rider slots 0..3, ctrl
        //     {8, 9, 10, 11}) are passive crank followers with kp = kd = 0,
        //     so their ctrl must be exactly 0. Upper-body posture PD
        //     (slots 4..8, ctrl {3, 4, 5, 6, 7}) must be nonzero. Non-rider
        //     ctrls {0, 1, 2} stay at 0.
        for (int j = 0; j < 4; ++j) {
            require(ctrl_w[to_index(rider_ctrl[to_index(j)])] == 0.0f,
                    "leg rider ctrls stay exactly zero");
        }
        for (int j = 4; j < kNumRiderJoints; ++j) {
            require(std::fabs(ctrl_w[to_index(rider_ctrl[to_index(j)])]) >
                        1e-3f,
                    "upper-body posture ctrls are nonzero");
        }
        constexpr std::array<int, 3> kNonRiderCtrl{{0, 1, 2}};
        for (const int ctrl_slot : kNonRiderCtrl) {
            require(ctrl_w[to_index(ctrl_slot)] == 0.0f,
                    "non-rider ctrls stay at zero");
        }

        // Hip soft-edge torque must be non-zero since hip is placed inside
        // lower ROM margin (rider_dof[0] = 22)
        require(qfrc_w[to_index(rider_dof[0])] > 0.0f,
                "hip soft-edge torque pushes back inside the ROM margin");

        // Freehub upper bound must have ratcheted downward during coasting
        // steps
        require(upper_bound < sc.initial_bound_w0 - 1e-3f,
                "freehub upper bound ratcheted downward while coasting");
        require_near(
            upper_bound,
            sc.states[wi].drivetrain.freehub_boundary_rad,
            1e-6f,
            "tendon upper bound equals the world freehub boundary");
    }
}

// Section 7: single drivetrain torque path. When tendon_range_batch is
// non-null and idx.freehub_tendon >= 0 (unilateral tendon active),
// qfrc[rear_wheel_spin_dof] does NOT double-count rear_drive_torque_nm.
// When tendon_range_batch is null (or freehub_tendon < 0),
// rear_drive_torque_nm IS included and its pawl reaction torque is
// subtracted from qfrc[crank_spin_dof].
void test_drivetrain_torque_path() {
    const BatchScenario sc = make_batch_scenario();
    const GpuModelIndices &idx = sc.idx;

    std::vector<GpuWorldState> st_tendon = sc.states;
    std::vector<GpuWorldState> st_explicit = sc.states;
    std::vector<float> qvel_pedaling = sc.qvel_batch;
    // Set crank fast enough relative to rear wheel so freehub pawls are
    // engaged with positive drive torque
    qvel_pedaling[to_index(idx.crank_spin_dof)] = 8.0f;
    qvel_pedaling[to_index(idx.rear_wheel_spin_dof)] = 2.5f;

    std::vector<float> qfrc_tendon(
        static_cast<std::size_t>(kNWorld) * to_index(idx.nv), 0.0f);
    std::vector<float> qfrc_explicit(
        static_cast<std::size_t>(kNWorld) * to_index(idx.nv), 0.0f);
    std::vector<float> ctrl_tmp(
        static_cast<std::size_t>(kNWorld) * to_index(idx.nu), 0.0f);
    std::vector<float> tendon_tmp(static_cast<std::size_t>(kNWorld) * 2, 0.0f);

    const GpuHeightfield1D hf = sc.heightfield();
    gpu_step_world_writers(
        0, kDt, idx, hf, sc.configs.data(), 1, st_tendon.data(),
        sc.qpos_batch.data(), qvel_pedaling.data(), sc.xpos_batch.data(),
        qfrc_tendon.data(), ctrl_tmp.data(), tendon_tmp.data()
    );
    gpu_step_world_writers(
        0, kDt, idx, hf, sc.configs.data(), 1, st_explicit.data(),
        sc.qpos_batch.data(), qvel_pedaling.data(), sc.xpos_batch.data(),
        qfrc_explicit.data(), ctrl_tmp.data(),
        /*tendon_range_batch=*/nullptr
    );

    const float rear_torque_diff =
        qfrc_explicit[to_index(idx.rear_wheel_spin_dof)] -
        qfrc_tendon[to_index(idx.rear_wheel_spin_dof)];
    const float crank_torque_diff =
        qfrc_tendon[to_index(idx.crank_spin_dof)] -
        qfrc_explicit[to_index(idx.crank_spin_dof)];
    require(rear_torque_diff > 1.0f,
            "explicit mode adds rear drive torque the tendon mode omits");
    require(crank_torque_diff > 0.4f,
            "explicit mode subtracts the pawl reaction from crank torque");
    require_near(tendon_tmp[1], st_tendon[0].drivetrain.freehub_boundary_rad,
                 1e-6f,
                 "tendon upper bound equals the stepped freehub boundary");
}

// Section 8: distinct worlds reflect their parameter sweep differences —
// world 63 has higher fork pressure & shock spring rate than world 0, and
// world 31 (32x1 rays) vs world 32 (45x10 = 450 rays) produce distinct tire
// forces.
void test_sweep_world_differences() {
    const BatchScenario sc = make_batch_scenario();
    const GpuModelIndices &idx = sc.idx;
    const std::size_t nv = to_index(idx.nv);
    const std::span<const float> qfrc_rows{sc.qfrc_batch};
    const std::span<const float> qfrc_w0 = qfrc_rows.first(nv);
    const std::span<const float> qfrc_w31 = qfrc_rows.subspan(31 * nv, nv);
    const std::span<const float> qfrc_w32 = qfrc_rows.subspan(32 * nv, nv);
    const std::span<const float> qfrc_w63 = qfrc_rows.subspan(63 * nv, nv);

    require(std::fabs(qfrc_w63[to_index(idx.fork_travel_dof)] -
                      qfrc_w0[to_index(idx.fork_travel_dof)]) > 50.0f,
            "fork DOF force differs across the pressure sweep");
    require(std::fabs(qfrc_w63[to_index(idx.shock_stroke_dof)]) >
                std::fabs(qfrc_w0[to_index(idx.shock_stroke_dof)]) + 50.0f,
            "shock DOF force differs across the rate sweep");
    require(std::fabs(qfrc_w63[to_index(idx.root_z_dof)] -
                      qfrc_w0[to_index(idx.root_z_dof)]) > 10.0f,
            "root_z force differs across the stiffness sweep");
    require(std::fabs(qfrc_w32[to_index(idx.root_z_dof)] -
                      qfrc_w31[to_index(idx.root_z_dof)]) > 1.0f,
            "450-ray tire forces differ from 32-ray tire forces");
}

// Section 9: the extern "C" host launch wrappers run on the CPU fallback and
// reject oversized model dimensions without touching the output batch.
void test_launch_wrappers_and_bounds() {
    BatchScenario sc = make_batch_scenario();
    const GpuModelIndices &idx = sc.idx;

    bike_gpu_launch_init_states(
        nullptr,
        kNWorld,
        &idx,
        sc.configs.data(),
        1,
        sc.states.data(),
        sc.qpos_batch.data()
    );
    bike_gpu_launch_writers(
        nullptr,
        kNWorld,
        kDt,
        &idx,
        sc.hf_heights.data(),
        kHfSamples,
        kHfXMin,
        kHfDx,
        sc.configs.data(),
        1,
        sc.states.data(),
        sc.qpos_batch.data(),
        sc.qvel_batch.data(),
        sc.xpos_batch.data(),
        sc.qfrc_batch.data(),
        sc.ctrl_batch.data(),
        sc.tendon_range_batch.data(),
        2
    );
    require(std::isfinite(sc.qfrc_batch[to_index(idx.root_z_dof)]),
            "launch_writers leaves finite qfrc on the CPU fallback");

    // Verify oversized model dimensions are rejected cleanly without
    // modifying qfrc_batch
    GpuModelIndices bad_idx = idx;
    bad_idx.nq = kMaxNq + 1;
    const float saved_root_z = sc.qfrc_batch[to_index(idx.root_z_dof)];
    sc.qfrc_batch[to_index(idx.root_z_dof)] = -12345.0f;
    bike_gpu_launch_step_writers(
        nullptr,
        kNWorld,
        kDt,
        &bad_idx,
        sc.hf_heights.data(),
        kHfSamples,
        kHfXMin,
        kHfDx,
        sc.configs.data(),
        1,
        sc.states.data(),
        sc.qpos_batch.data(),
        sc.qvel_batch.data(),
        sc.xpos_batch.data(),
        sc.qfrc_batch.data(),
        sc.ctrl_batch.data(),
        sc.tendon_range_batch.data(),
        2
    );
    require(sc.qfrc_batch[to_index(idx.root_z_dof)] == -12345.0f,
            "rejected launch leaves qfrc_batch unmodified");
    sc.qfrc_batch[to_index(idx.root_z_dof)] = saved_root_z;
}

constexpr std::array<TestCase, 6> cases{{
    {.name = "api_sizes_and_model_defaults",
     .run = test_api_sizes_and_model_defaults},
    {.name = "sweep_config_preloads", .run = test_sweep_config_preloads},
    {.name = "stepped_world_outputs", .run = test_stepped_world_outputs},
    {.name = "drivetrain_torque_path", .run = test_drivetrain_torque_path},
    {.name = "sweep_world_differences", .run = test_sweep_world_differences},
    {.name = "launch_wrappers_and_bounds",
     .run = test_launch_wrappers_and_bounds},
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
            std::cerr << "usage: bike_test_gpu_batch_step [case-name|--list]\n";
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
            std::cerr << "unknown gpu batch step case: " << requested << '\n';
            return 2;
        }
        return status;
    } catch (const std::exception &error) {
        // A throwing argument envelope is a harness failure, not a usage error.
        std::cerr << "FAIL argument handling: " << error.what() << '\n';
        return 1;
    }
}
