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
#include "../../src/gpu/terrain_tables.hpp"
#include "../../src/platform/argv.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iostream>
#include <memory>
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
constexpr int kNjnt = 27;
constexpr float kDt = 0.001f;
constexpr float kHfXMin = -5.0f;
constexpr float kHfDx = 0.05f;
constexpr float kFrontRadiusM = 0.372f;
constexpr float kRearRadiusM = 0.352f;

bike::physics_core::GpuTyreModel make_synthetic_tyre_model(
    const GpuModelIndices &idx,
    bike::physics_core::TyreBackendKind backend
) {
    using namespace bike::physics_core;
    GpuTyreModel tm{};
    tm.backend = backend;
    tm.nq = idx.nq;
    tm.nv = idx.nv;
    tm.nbody = idx.nbody;
    tm.njnt = kNjnt;
    tm.station_count = (backend == TyreBackendKind::distributed_2d_reference) ? 64 : 0;
    tm.distinct_normal_cosine = 0.9781476f;

    struct WheelSpecArgs {
        int side;
        int body_id;
        float radius;
        int extra_j1;
        int extra_j2;
    };
    const std::span<TyreWheelSpec<float>, 2> tm_wheels(tm.wheels);
    const auto init_wheel = [&](const WheelSpecArgs &ws) {
        TyreWheelSpec<float> &w = tm_wheels[to_index(ws.side)];
        w.geom_id = ws.side;
        w.radius_m = ws.radius;
        w.tangent_k_n_m = 60000.0f;
        w.tire_mu = 1.1f;
        w.relaxation_length_m = 0.05f;
        w.compliant = CompliantMaterialView<float>{
            .radial_k_n_m = 110000.0f,
            .loss_factor = 0.05f,
            .hysteresis_rate_eps_mps = 0.01f,
            .valid_load_min_n = 0.0f,
            .valid_load_max_n = 4000.0f,
            .has_tabulated_radial = 0,
        };
        w.distributed_density = HingeDensityView<float>{
            .knots_m = {0.0f, 0.01f, 0.03f, 0.06f},
            .stiffness_n_m = {180000.0f, 220000.0f, 280000.0f, 360000.0f},
            .knot_count = 4,
            .damping_ns_m = 0.0f,
            .loss_factor = 0.05f,
            .hysteresis_rate_eps_mps = 0.01f,
        };
        w.valid_deflection_min_m = 0.0f;
        w.valid_deflection_max_m = 0.08f;
        w.station_weight = 6.283185307f / 64.0f;
        const std::span local_xyz(w.local_xyz);
        for (int s = 0; s < 64; ++s) {
            const float ang = 6.283185307f * static_cast<float>(s) / 64.0f;
            local_xyz[to_index(3 * s + 0)] = ws.radius * std::cos(ang);
            local_xyz[to_index(3 * s + 1)] = 0.0f;
            local_xyz[to_index(3 * s + 2)] = ws.radius * std::sin(ang);
        }

        w.chain.wheel_body_id = ws.body_id;
        w.chain.njnt = kNjnt;
        const std::span chain_joints(w.chain.joints);
        chain_joints[0] = GpuChainJoint{.joint_id = idx.root_x_dof, .joint_type = kJntSlide, .dof_adr = idx.root_x_dof};
        chain_joints[1] = GpuChainJoint{.joint_id = idx.root_z_dof, .joint_type = kJntSlide, .dof_adr = idx.root_z_dof};
        chain_joints[2] = GpuChainJoint{.joint_id = idx.root_pitch_dof, .joint_type = kJntHinge, .dof_adr = idx.root_pitch_dof};
        chain_joints[3] = GpuChainJoint{
            .joint_id = ws.extra_j1,
            .joint_type = (ws.side == 0) ? kJntSlide : kJntHinge,
            .dof_adr = ws.extra_j1,
        };
        std::size_t cnt = 4;
        if (ws.extra_j2 >= 0) {
            chain_joints[cnt++] = GpuChainJoint{.joint_id = ws.extra_j2, .joint_type = kJntHinge, .dof_adr = ws.extra_j2};
        }
        const int spin_dof = (ws.side == 0) ? idx.front_wheel_spin_dof : idx.rear_wheel_spin_dof;
        chain_joints[cnt++] = GpuChainJoint{.joint_id = spin_dof, .joint_type = kJntHinge, .dof_adr = spin_dof};
        w.chain.joint_count = static_cast<int>(cnt);
    };

    init_wheel(WheelSpecArgs{.side = 0, .body_id = idx.front_wheel_body, .radius = kFrontRadiusM, .extra_j1 = idx.fork_travel_dof, .extra_j2 = -1});
    init_wheel(WheelSpecArgs{.side = 1, .body_id = idx.rear_wheel_body, .radius = kRearRadiusM, .extra_j1 = idx.main_pivot_dof, .extra_j2 = idx.horst_pivot_dof});
    tm.frame_chain.wheel_body_id = idx.frame_body;
    tm.frame_chain.njnt = kNjnt;
    const std::span frame_joints(tm.frame_chain.joints);
    frame_joints[0] = GpuChainJoint{.joint_id = idx.root_x_dof, .joint_type = kJntSlide, .dof_adr = idx.root_x_dof};
    frame_joints[1] = GpuChainJoint{.joint_id = idx.root_z_dof, .joint_type = kJntSlide, .dof_adr = idx.root_z_dof};
    frame_joints[2] = GpuChainJoint{.joint_id = idx.root_pitch_dof, .joint_type = kJntHinge, .dof_adr = idx.root_pitch_dof};
    tm.frame_chain.joint_count = 3;
    return tm;
}

float eval_synthetic_ground_z(float x) {
    return 0.015f * std::sin(1.7f * x) + 0.008f * std::cos(5.3f * x);
}

// Shared scenario buffers for the batch-step cases.
struct BatchScenario {
    GpuModelIndices idx{};
    bike::physics_core::GpuTyreModel tyre_model{};
    HostPackedTerrain terrain{};
    std::vector<float> hf_heights;
    std::vector<GpuWorldConfig> configs;
    std::vector<GpuWorldState> states;
    std::vector<float> qpos_batch;
    std::vector<float> qvel_batch;
    std::vector<float> xpos_batch;
    std::vector<float> xmat_batch;
    std::vector<float> xanchor_batch;
    std::vector<float> xaxis_batch;
    std::vector<float> qfrc_batch;
    std::vector<float> ctrl_batch;
    std::vector<float> tendon_range_batch;
    float initial_bound_w0 = 0.0f;
};

// 64 heterogeneous GpuWorldConfig entries.
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
    require(bike_gpu_build_sweep_configs(
                configs.data(),
                kNWorld,
                fork_psi_scale.data(),
                rebound_scale.data(),
                shock_rate_scale.data(),
                tire_stiff_scale.data()) == 0,
            "bike_gpu_build_sweep_configs succeeds");
    return configs;
}

BatchScenario make_batch_scenario(
    bike::physics_core::TyreBackendKind backend = bike::physics_core::TyreBackendKind::compliant_2d
) {
    BatchScenario sc;
    const GpuModelIndices &idx = sc.idx;
    sc.tyre_model = make_synthetic_tyre_model(idx, backend);
    const std::size_t nq = to_index(idx.nq);
    const std::size_t nv = to_index(idx.nv);
    const std::size_t nbody3 = 3 * to_index(idx.nbody);
    const std::size_t nbody9 = 9 * to_index(idx.nbody);
    const std::size_t njnt3 = 3 * to_index(kNjnt);

    // 2. Build a 4000-sample synthetic rough terrain (x in [-5.0, 195.0], dx = 0.05 m)
    sc.hf_heights.assign(to_index(kHfSamples), 0.0f);
    for (int i = 0; i < kHfSamples; ++i) {
        const float x = kHfXMin + static_cast<float>(i) * kHfDx;
        sc.hf_heights[to_index(i)] = eval_synthetic_ground_z(x);
    }
    const float x_max = kHfXMin + static_cast<float>(kHfSamples - 1) * kHfDx;
    sc.terrain = pack_uniform_heightfield_to_terrain_tables(
        sc.hf_heights, {.x_min = kHfXMin, .x_max = x_max});

    // 3. Heterogeneous per-world configs
    sc.configs = make_sweep_configs();

    // 4. Allocate batched state and MuJoCo-like kinematics buffers
    sc.states.resize(static_cast<std::size_t>(kNWorld));
    sc.qpos_batch.assign(static_cast<std::size_t>(kNWorld) * nq, 0.0f);
    sc.qvel_batch.assign(static_cast<std::size_t>(kNWorld) * nv, 0.0f);
    sc.xpos_batch.assign(static_cast<std::size_t>(kNWorld) * nbody3, 0.0f);
    sc.xmat_batch.assign(static_cast<std::size_t>(kNWorld) * nbody9, 0.0f);
    sc.xanchor_batch.assign(static_cast<std::size_t>(kNWorld) * njnt3, 0.0f);
    sc.xaxis_batch.assign(static_cast<std::size_t>(kNWorld) * njnt3, 0.0f);
    sc.qfrc_batch.assign(static_cast<std::size_t>(kNWorld) * nv, 0.0f);
    sc.ctrl_batch.assign(static_cast<std::size_t>(kNWorld) * to_index(idx.nu), 0.0f);
    sc.tendon_range_batch.assign(static_cast<std::size_t>(kNWorld) * 2, 0.0f);

    const std::span<float> qpos_rows{sc.qpos_batch};
    const std::span<float> qvel_rows{sc.qvel_batch};
    const std::span<float> xpos_rows{sc.xpos_batch};
    const std::span<float> xmat_rows{sc.xmat_batch};
    const std::span<float> xanchor_rows{sc.xanchor_batch};
    const std::span<float> xaxis_rows{sc.xaxis_batch};
    const auto rider_qpos = std::span{idx.rider_qpos};

    for (int w = 0; w < kNWorld; ++w) {
        const std::size_t wi = to_index(w);
        const std::span<float> qpos_w = qpos_rows.subspan(wi * nq, nq);
        const std::span<float> qvel_w = qvel_rows.subspan(wi * nv, nv);
        const std::span<float> xpos_w = xpos_rows.subspan(wi * nbody3, nbody3);
        const std::span<float> xmat_w = xmat_rows.subspan(wi * nbody9, nbody9);
        const std::span<float> xanc_w = xanchor_rows.subspan(wi * njnt3, njnt3);
        const std::span<float> xax_w = xaxis_rows.subspan(wi * njnt3, njnt3);

        qpos_w[to_index(idx.root_x_qpos)] = 2.0f;
        qpos_w[to_index(idx.root_z_qpos)] = 0.55f;
        qpos_w[to_index(idx.root_pitch_qpos)] = 0.02f;
        qpos_w[to_index(idx.crank_spin_qpos)] = 0.40f;
        qpos_w[to_index(idx.drive_shaft_spin_qpos)] = 0.40f;
        qpos_w[to_index(idx.fork_travel_qpos)] = 0.045f;
        qpos_w[to_index(idx.front_wheel_spin_qpos)] = 1.10f;
        qpos_w[to_index(idx.rear_wheel_spin_qpos)] = 0.70f;
        qpos_w[to_index(idx.shock_stroke_qpos)] = 0.022f;

        qpos_w[to_index(rider_qpos[0])] = -2.02f;
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
        const float front_ground_z = eval_synthetic_ground_z(front_x);
        const float rear_ground_z = eval_synthetic_ground_z(rear_x);

        xpos_w[3 * to_index(idx.frame_body) + 0] = 2.00f;
        xpos_w[3 * to_index(idx.frame_body) + 2] = 0.62f;

        xpos_w[3 * to_index(idx.front_wheel_body) + 0] = front_x;
        xpos_w[3 * to_index(idx.front_wheel_body) + 2] = front_ground_z + kFrontRadiusM - 0.014f;

        xpos_w[3 * to_index(idx.rear_wheel_body) + 0] = rear_x;
        xpos_w[3 * to_index(idx.rear_wheel_body) + 2] = rear_ground_z + kRearRadiusM - 0.016f;

        for (int b = 0; b < idx.nbody; ++b) {
            xmat_w[9 * to_index(b) + 0] = 1.0f;
            xmat_w[9 * to_index(b) + 4] = 1.0f;
            xmat_w[9 * to_index(b) + 8] = 1.0f;
        }
        xax_w[3 * to_index(idx.root_x_dof) + 0] = 1.0f;
        xax_w[3 * to_index(idx.root_z_dof) + 2] = 1.0f;
        xax_w[3 * to_index(idx.root_pitch_dof) + 1] = 1.0f;
        xax_w[3 * to_index(idx.fork_travel_dof) + 0] = 0.34f;
        xax_w[3 * to_index(idx.fork_travel_dof) + 2] = -0.94f;
        xax_w[3 * to_index(idx.front_wheel_spin_dof) + 1] = 1.0f;
        xax_w[3 * to_index(idx.rear_wheel_spin_dof) + 1] = 1.0f;
        xax_w[3 * to_index(idx.main_pivot_dof) + 1] = 1.0f;
        xax_w[3 * to_index(idx.horst_pivot_dof) + 1] = 1.0f;

        xanc_w[3 * to_index(idx.root_pitch_dof) + 0] = 2.00f;
        xanc_w[3 * to_index(idx.root_pitch_dof) + 2] = 0.62f;
        xanc_w[3 * to_index(idx.main_pivot_dof) + 0] = rear_x + 0.42f;
        xanc_w[3 * to_index(idx.main_pivot_dof) + 2] = rear_ground_z + kRearRadiusM + 0.08f;
        xanc_w[3 * to_index(idx.horst_pivot_dof) + 0] = rear_x + 0.08f;
        xanc_w[3 * to_index(idx.horst_pivot_dof) + 2] = rear_ground_z + kRearRadiusM - 0.04f;
        xanc_w[3 * to_index(idx.front_wheel_spin_dof) + 0] = front_x;
        xanc_w[3 * to_index(idx.front_wheel_spin_dof) + 2] = front_ground_z + kFrontRadiusM - 0.014f;
        xanc_w[3 * to_index(idx.rear_wheel_spin_dof) + 0] = rear_x;
        xanc_w[3 * to_index(idx.rear_wheel_spin_dof) + 2] = rear_ground_z + kRearRadiusM - 0.016f;

        sc.tendon_range_batch[wi * 2 + 0] = -1.0e6f;
        sc.tendon_range_batch[wi * 2 + 1] = 0.0f;
    }

    for (int w = 0; w < kNWorld; ++w) {
        const std::size_t wi = to_index(w);
        const std::span<const float> qpos_w = qpos_rows.subspan(wi * nq, nq);
        gpu_init_world_state(
            sc.states[wi],
            sc.configs[wi].drivetrain,
            qpos_w[to_index(idx.drive_shaft_spin_qpos)],
            qpos_w[to_index(idx.rear_wheel_spin_qpos)]
        );
    }

    sc.initial_bound_w0 = sc.states[0].drivetrain.freehub.boundary;

    // 5. Step gpu_step_world_writers across all 64 worlds for 10 steps
    for (int step = 0; step < kNumSteps; ++step) {
        for (int w = 0; w < kNWorld; ++w) {
            const bike::physics_core::Status st = gpu_step_world_writers(
                w,
                kDt,
                idx,
                &sc.tyre_model,
                sc.terrain.desc,
                sc.terrain.f32.data(),
                sc.terrain.i32.data(),
                sc.configs.data(),
                /*per_world_config=*/1,
                sc.states.data(),
                sc.qpos_batch.data(),
                sc.qvel_batch.data(),
                sc.xpos_batch.data(),
                sc.xmat_batch.data(),
                sc.xanchor_batch.data(),
                sc.xaxis_batch.data(),
                sc.qfrc_batch.data(),
                sc.ctrl_batch.data(),
                sc.tendon_range_batch.data()
            );
            require(st == bike::physics_core::Status::ok,
                    "gpu_step_world_writers succeeds in scenario loop");
        }

        for (int w = 0; w < kNWorld; ++w) {
            const std::size_t wi = to_index(w);
            const std::span<float> qpos_w = qpos_rows.subspan(wi * nq, nq);
            const std::span<float> qvel_w = qvel_rows.subspan(wi * nv, nv);
            const std::span<float> xpos_w = xpos_rows.subspan(wi * nbody3, nbody3);
            const std::span<float> xanc_w = xanchor_rows.subspan(wi * njnt3, njnt3);

            const float dx = qvel_w[to_index(idx.root_x_dof)] * kDt;
            qpos_w[to_index(idx.root_x_qpos)] += dx;
            xpos_w[3 * to_index(idx.frame_body) + 0] += dx;
            xpos_w[3 * to_index(idx.front_wheel_body) + 0] += dx;
            xpos_w[3 * to_index(idx.rear_wheel_body) + 0] += dx;

            const float f_ground = eval_synthetic_ground_z(xpos_w[3 * to_index(idx.front_wheel_body) + 0]);
            const float r_ground = eval_synthetic_ground_z(xpos_w[3 * to_index(idx.rear_wheel_body) + 0]);
            xpos_w[3 * to_index(idx.front_wheel_body) + 2] = f_ground + kFrontRadiusM - 0.014f;
            xpos_w[3 * to_index(idx.rear_wheel_body) + 2] = r_ground + kRearRadiusM - 0.016f;
            xanc_w[3 * to_index(idx.root_pitch_dof) + 0] += dx;
            xanc_w[3 * to_index(idx.main_pivot_dof) + 0] += dx;
            xanc_w[3 * to_index(idx.horst_pivot_dof) + 0] += dx;
            xanc_w[3 * to_index(idx.front_wheel_spin_dof) + 0] = xpos_w[3 * to_index(idx.front_wheel_body) + 0];
            xanc_w[3 * to_index(idx.front_wheel_spin_dof) + 2] = xpos_w[3 * to_index(idx.front_wheel_body) + 2];
            xanc_w[3 * to_index(idx.rear_wheel_spin_dof) + 0] = xpos_w[3 * to_index(idx.rear_wheel_body) + 0];
            xanc_w[3 * to_index(idx.rear_wheel_spin_dof) + 2] = xpos_w[3 * to_index(idx.rear_wheel_body) + 2];

            if (step < 5) {
                qpos_w[to_index(idx.crank_spin_qpos)] += 2.0f * kDt;
                qpos_w[to_index(idx.drive_shaft_spin_qpos)] += 2.0f * kDt;
                qpos_w[to_index(idx.rear_wheel_spin_qpos)] += 20.0f * kDt;
            } else {
                qpos_w[to_index(idx.crank_spin_qpos)] += 28.0f * kDt;
                qpos_w[to_index(idx.drive_shaft_spin_qpos)] += 28.0f * kDt;
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
    static_assert(sizeof(GpuModelIndices) == 240, "GpuModelIndices must be 240 bytes");
    require(bike_gpu_sizeof_tyre_model() ==
                static_cast<int>(sizeof(bike::physics_core::GpuTyreModel)),
            "C API tyre model size matches sizeof(GpuTyreModel)");
    require(bike_gpu_sizeof_terrain_desc() ==
                static_cast<int>(sizeof(bike::physics_core::GpuTerrainTableDesc)),
            "C API terrain desc size matches sizeof(GpuTerrainTableDesc)");
    require(bike_gpu_sizeof_model_indices() == 240,
            "C API model indices size is 240 bytes");
    require(bike_gpu_sizeof_model_indices() ==
                static_cast<int>(sizeof(GpuModelIndices)),
            "C API model indices size matches sizeof(GpuModelIndices)");

    const GpuModelIndices idx{};
    require(idx.nq == 27, "model has 27 qpos slots");
    require(idx.nv == 27, "model has 27 qvel DOFs");
    require(idx.nu == 12, "model has 12 actuators");
    require(idx.nbody == 29, "model has 29 bodies");
    require(idx.drive_shaft_spin_qpos == 14, "drive_shaft_spin qpos is 14");
    require(idx.drive_shaft_spin_dof == 14, "drive_shaft_spin dof is 14");
    require(idx.main_pivot_dof == 8, "main pivot DOF address is 8");
    require(idx.horst_pivot_dof == 9, "horst pivot DOF address is 9");
    require(idx.freehub_tendon == 0, "freehub tendon index is 0");
    require(idx.crank_clutch_tendon == 1, "crank clutch tendon index is 1");

    constexpr std::array<int, kNumRiderJoints> kExpectedRiderQpos{
        {23, 24, 25, 26, 18, 19, 20, 21, 22}};
    constexpr std::array<int, kNumRiderJoints> kExpectedRiderDof{
        {23, 24, 25, 26, 18, 19, 20, 21, 22}};
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
            sc.states[wi].drivetrain.freehub.boundary,
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

    require(gpu_step_world_writers(
                0, kDt, idx, &sc.tyre_model, sc.terrain.desc,
                sc.terrain.f32.data(), sc.terrain.i32.data(),
                sc.configs.data(), 1, st_tendon.data(),
                sc.qpos_batch.data(), qvel_pedaling.data(), sc.xpos_batch.data(),
                sc.xmat_batch.data(), sc.xanchor_batch.data(), sc.xaxis_batch.data(),
                qfrc_tendon.data(), ctrl_tmp.data(), tendon_tmp.data()) ==
                bike::physics_core::Status::ok,
            "tendon mode step succeeds");
    require(gpu_step_world_writers(
                0, kDt, idx, &sc.tyre_model, sc.terrain.desc,
                sc.terrain.f32.data(), sc.terrain.i32.data(),
                sc.configs.data(), 1, st_explicit.data(),
                sc.qpos_batch.data(), qvel_pedaling.data(), sc.xpos_batch.data(),
                sc.xmat_batch.data(), sc.xanchor_batch.data(), sc.xaxis_batch.data(),
                qfrc_explicit.data(), ctrl_tmp.data(),
                /*tendon_range_batch=*/nullptr) ==
                bike::physics_core::Status::ok,
            "explicit mode step succeeds");

    const float rear_torque_diff =
        qfrc_explicit[to_index(idx.rear_wheel_spin_dof)] -
        qfrc_tendon[to_index(idx.rear_wheel_spin_dof)];
    const int fh_driver_dof =
        (idx.drive_shaft_spin_dof >= 0) ? idx.drive_shaft_spin_dof : idx.crank_spin_dof;
    const float driver_torque_diff =
        qfrc_tendon[to_index(fh_driver_dof)] -
        qfrc_explicit[to_index(fh_driver_dof)];
    require(rear_torque_diff > 1.0f,
            "explicit mode adds rear drive torque the tendon mode omits");
    require(driver_torque_diff > 0.4f,
            "explicit mode subtracts the pawl reaction from freehub driver torque");
    require_near(tendon_tmp[1], st_tendon[0].drivetrain.freehub.boundary,
                 1e-6f,
                 "tendon upper bound equals the stepped freehub boundary");
}

// Section 8: distinct worlds reflect their parameter sweep differences —
// world 63 has higher fork pressure & shock spring rate than world 0, and
// compliant_2d vs distributed_2d_reference produce distinct tire loads.
void test_sweep_world_differences() {
    const auto sc = std::make_unique<BatchScenario>(
        make_batch_scenario(bike::physics_core::TyreBackendKind::compliant_2d));
    const auto sc_dist = std::make_unique<BatchScenario>(
        make_batch_scenario(bike::physics_core::TyreBackendKind::distributed_2d_reference));
    const GpuModelIndices &idx = sc->idx;
    const std::size_t nv = to_index(idx.nv);
    const std::span<const float> qfrc_rows{sc->qfrc_batch};
    const std::span<const float> qfrc_dist_rows{sc_dist->qfrc_batch};
    const std::span<const float> qfrc_w0 = qfrc_rows.first(nv);
    const std::span<const float> qfrc_w63 = qfrc_rows.subspan(63 * nv, nv);

    require(std::fabs(qfrc_w63[to_index(idx.fork_travel_dof)] -
                      qfrc_w0[to_index(idx.fork_travel_dof)]) > 10.0f,
            "fork DOF force differs across the pressure sweep");
    require(std::fabs(qfrc_w63[to_index(idx.shock_stroke_dof)]) >
                std::fabs(qfrc_w0[to_index(idx.shock_stroke_dof)]) + 50.0f,
            "shock DOF force differs across the rate sweep");
    require(std::fabs(qfrc_w63[to_index(idx.root_z_dof)] -
                      qfrc_w0[to_index(idx.root_z_dof)]) > 10.0f,
            "root_z force differs across the stiffness sweep");
    require(std::fabs(qfrc_dist_rows[to_index(idx.root_z_dof)] -
                      qfrc_w0[to_index(idx.root_z_dof)]) > 1.0f,
            "distributed_2d_reference and compliant_2d backends produce distinct tire loads");
}

// Section 9: the extern "C" host launch wrappers run on the CPU fallback and
// reject invalid bounds/descriptors with nonzero int without touching qfrc_batch.
void test_launch_wrappers_and_bounds() {
    const auto sc_ptr = std::make_unique<BatchScenario>(make_batch_scenario());
    BatchScenario &sc = *sc_ptr;
    const GpuModelIndices &idx = sc.idx;

    require(bike_gpu_launch_init_states(
                nullptr,
                kNWorld,
                &idx,
                sc.configs.data(),
                1,
                sc.states.data(),
                sc.qpos_batch.data()) == 0,
            "launch_init_states succeeds");
    require(bike_gpu_launch_writers(
                nullptr,
                kNWorld,
                kDt,
                &idx,
                &sc.tyre_model,
                &sc.terrain.desc,
                sc.terrain.f32.data(),
                sc.terrain.i32.data(),
                sc.configs.data(),
                1,
                sc.states.data(),
                sc.qpos_batch.data(),
                sc.qvel_batch.data(),
                sc.xpos_batch.data(),
                sc.xmat_batch.data(),
                sc.xanchor_batch.data(),
                sc.xaxis_batch.data(),
                sc.qfrc_batch.data(),
                sc.ctrl_batch.data(),
                sc.tendon_range_batch.data(),
                2) == 0,
            "launch_writers succeeds");
    require(std::isfinite(sc.qfrc_batch[to_index(idx.root_z_dof)]),
            "launch_writers leaves finite qfrc on the CPU fallback");

    // Reject oversized model dimensions, bad joint_id, oversized station_count, and non-monotonic px
    GpuModelIndices bad_idx = idx;
    bad_idx.nq = kMaxNq + 1;
    const float saved_root_z = sc.qfrc_batch[to_index(idx.root_z_dof)];
    sc.qfrc_batch[to_index(idx.root_z_dof)] = -12345.0f;
    require(bike_gpu_launch_step_writers(
                nullptr, kNWorld, kDt, &bad_idx, &sc.tyre_model, &sc.terrain.desc,
                sc.terrain.f32.data(), sc.terrain.i32.data(),
                sc.configs.data(), 1, sc.states.data(),
                sc.qpos_batch.data(), sc.qvel_batch.data(), sc.xpos_batch.data(),
                sc.xmat_batch.data(), sc.xanchor_batch.data(), sc.xaxis_batch.data(),
                sc.qfrc_batch.data(), sc.ctrl_batch.data(), sc.tendon_range_batch.data(), 2) != 0,
            "launch_step_writers rejects nq > kMaxNq");
    require(sc.qfrc_batch[to_index(idx.root_z_dof)] == -12345.0f,
            "rejected launch leaves qfrc_batch unmodified");

    {
        auto bad_tm = std::make_unique<bike::physics_core::GpuTyreModel>(sc.tyre_model);
        const std::span bad_wheels(bad_tm->wheels);
        const std::span bad_joints(bad_wheels[0].chain.joints);
        bad_joints[0].joint_id = kNjnt;
        require(bike_gpu_launch_writers(
                    nullptr, kNWorld, kDt, &idx, bad_tm.get(), &sc.terrain.desc,
                    sc.terrain.f32.data(), sc.terrain.i32.data(),
                    sc.configs.data(), 1, sc.states.data(),
                    sc.qpos_batch.data(), sc.qvel_batch.data(), sc.xpos_batch.data(),
                    sc.xmat_batch.data(), sc.xanchor_batch.data(), sc.xaxis_batch.data(),
                    sc.qfrc_batch.data(), sc.ctrl_batch.data(), sc.tendon_range_batch.data(), 2) != 0,
                "launch_writers rejects joint_id >= njnt");
    }
    {
        auto bad_tm = std::make_unique<bike::physics_core::GpuTyreModel>(sc.tyre_model);
        bad_tm->backend = bike::physics_core::TyreBackendKind::distributed_2d_reference;
        bad_tm->station_count = bike::physics_core::kMaxGpuTyreStations + 1;
        require(bike_gpu_launch_writers(
                    nullptr, kNWorld, kDt, &idx, bad_tm.get(), &sc.terrain.desc,
                    sc.terrain.f32.data(), sc.terrain.i32.data(),
                    sc.configs.data(), 1, sc.states.data(),
                    sc.qpos_batch.data(), sc.qvel_batch.data(), sc.xpos_batch.data(),
                    sc.xmat_batch.data(), sc.xanchor_batch.data(), sc.xaxis_batch.data(),
                    sc.qfrc_batch.data(), sc.ctrl_batch.data(), sc.tendon_range_batch.data(), 2) != 0,
                "launch_writers rejects station_count > 128");
    }
    {
        auto bad_f32 = sc.terrain.f32;
        bad_f32[to_index(sc.terrain.desc.px_offset + 10)] =
            bad_f32[to_index(sc.terrain.desc.px_offset + 9)];
        require(bike_gpu_launch_writers(
                    nullptr, kNWorld, kDt, &idx, &sc.tyre_model, &sc.terrain.desc,
                    bad_f32.data(), sc.terrain.i32.data(),
                    sc.configs.data(), 1, sc.states.data(),
                    sc.qpos_batch.data(), sc.qvel_batch.data(), sc.xpos_batch.data(),
                    sc.xmat_batch.data(), sc.xanchor_batch.data(), sc.xaxis_batch.data(),
                    sc.qfrc_batch.data(), sc.ctrl_batch.data(), sc.tendon_range_batch.data(), 2) != 0,
                "launch_writers rejects non-monotonic px");
    }
    sc.qfrc_batch[to_index(idx.root_z_dof)] = saved_root_z;

    {
        GpuModelIndices idx_t1 = idx;
        idx_t1.freehub_tendon = 1;
        std::vector<GpuWorldState> st_t1 = sc.states;
        std::vector<float> tendon_stride4(static_cast<std::size_t>(kNWorld) * 4, -99.0f);
        require(gpu_step_world_writers(
                    3, kDt, idx_t1, &sc.tyre_model, sc.terrain.desc,
                    sc.terrain.f32.data(), sc.terrain.i32.data(),
                    sc.configs.data(), 1, st_t1.data(),
                    sc.qpos_batch.data(), sc.qvel_batch.data(), sc.xpos_batch.data(),
                    sc.xmat_batch.data(), sc.xanchor_batch.data(), sc.xaxis_batch.data(),
                    sc.qfrc_batch.data(), sc.ctrl_batch.data(), tendon_stride4.data(), 4) ==
                    bike::physics_core::Status::ok,
                "stride 4 step succeeds");
        require(tendon_stride4[3 * 4 + 0] == -99.0f &&
                    tendon_stride4[3 * 4 + 1] == -99.0f &&
                    tendon_stride4[3 * 4 + 2] == -99.0f,
                "tendon_range_batch only writes 2 * freehub_tendon + 1");
        require_near(tendon_stride4[3 * 4 + 3],
                     st_t1[3].drivetrain.freehub.boundary,
                     1e-6f,
                     "tendon_range_batch writes freehub upper bound at 2 * freehub_tendon + 1");

        std::vector<float> tendon_small(static_cast<std::size_t>(kNWorld) * 2, -77.0f);
        sc.qfrc_batch[3 * to_index(idx_t1.nv) + to_index(idx_t1.root_z_dof)] = -54321.0f;
        require(bike_gpu_launch_step_writers(
                    nullptr, kNWorld, kDt, &idx_t1,
                    &sc.tyre_model, &sc.terrain.desc,
                    sc.terrain.f32.data(), sc.terrain.i32.data(),
                    sc.configs.data(), 1, st_t1.data(),
                    sc.qpos_batch.data(), sc.qvel_batch.data(), sc.xpos_batch.data(),
                    sc.xmat_batch.data(), sc.xanchor_batch.data(), sc.xaxis_batch.data(),
                    sc.qfrc_batch.data(), sc.ctrl_batch.data(), tendon_small.data(), 2) != 0,
                "launch_step_writers rejects small stride");
        require(sc.qfrc_batch[3 * to_index(idx_t1.nv) + to_index(idx_t1.root_z_dof)] == -54321.0f &&
                    tendon_small[3 * 2 + 1] == -77.0f,
                "rejected tendon_range_world_stride leaves qfrc and tendon untouched");
    }
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
