// native/tests/gpu/test_gpu_batch_runners.cpp — C++23 integration and regression test
// for native/src/gpu/cpu_batch_runner.cpp and native/src/gpu/metal_batch_runner.mm.
//
// Usage: bike_test_gpu_batch_runners [case-name|--list]
//        (the model is located relative to BIKE_REPO_ROOT or the cwd)

#include "../../src/gpu/batch_step.hpp"
#include "../../src/platform/argv.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

extern "C" {

int bike_gpu_sizeof_world_config();
int bike_gpu_sizeof_world_state();
int bike_gpu_sizeof_model_indices();
void bike_gpu_init_default_config(bike::gpu::GpuWorldConfig *out_cfg);
void bike_gpu_build_sweep_configs(
    bike::gpu::GpuWorldConfig *out_configs,
    int nworld,
    const float *fork_psi_scale,
    const float *rebound_scale,
    const float *shock_rate_scale,
    const float *tire_stiff_scale,
    int num_stations,
    int num_lateral_ribs
);

int bike_cpu_run_batch_sweep(
    const char *xml_path,
    int nworld,
    int n_steps,
    int num_threads,
    const bike::gpu::GpuModelIndices *h_idx,
    const float *d_hf_heights,
    int hf_num_samples,
    float hf_x_min_m,
    float hf_dx_m,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    const double *init_qpos,
    const double *init_qvel,
    float *out_peak_fork_mm,
    float *out_peak_shock_mm,
    float *out_final_root_x_m,
    int *out_all_finite,
    double *out_elapsed_ms
);

#if defined(__APPLE__)
int bike_metal_init(const char *gpu_src_dir);
const char *bike_metal_device_name();

int bike_metal_launch_init_states(
    int nworld,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos
);

int bike_metal_launch_writers(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const float *d_hf_heights,
    int hf_num_samples,
    float hf_x_min_m,
    float hf_dx_m,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    float *d_qfrc_out,
    float *d_ctrl_out,
    float *d_tendon_range_out,
    int tendon_range_world_stride
);

int bike_metal_run_hybrid_sweep(
    const char *xml_path,
    int nworld,
    int n_steps,
    int num_threads,
    const bike::gpu::GpuModelIndices *h_idx,
    const float *d_hf_heights,
    int hf_num_samples,
    float hf_x_min_m,
    float hf_dx_m,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    const double *init_qpos,
    const double *init_qvel,
    float *out_peak_fork_mm,
    float *out_peak_shock_mm,
    float *out_final_root_x_m,
    int *out_all_finite,
    double *out_elapsed_ms
);
#endif

} // extern "C"

namespace {

namespace fs = std::filesystem;

using TestFunction = void (*)();

struct TestCase {
    std::string_view name;
    TestFunction run;
};

void require(bool condition, std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string(message));
    }
}

// Numeric helper used instead of printf-style messages. Templated on the
// arithmetic type so float comparisons keep their original float semantics
// instead of silently promoting to double (same helper as the other gpu tests).
template <class T>
void require_near(T actual, T expected, T tol, std::string_view what) {
    if (!(std::abs(actual - expected) <= tol)) {
        throw std::runtime_error(std::string(what) +
                                 ": actual=" + std::to_string(actual) +
                                 " expected=" + std::to_string(expected) +
                                 " tol=" + std::to_string(tol));
    }
}

// One-sided bounds that report the measured value and the bound on failure.
void require_above(double actual, double bound, std::string_view what) {
    require(actual > bound,
            std::string(what) + ": actual=" + std::to_string(actual) +
                " bound=" + std::to_string(bound));
}

void require_below(double actual, double bound, std::string_view what) {
    require(actual < bound,
            std::string(what) + ": actual=" + std::to_string(actual) +
                " bound=" + std::to_string(bound));
}

// GpuModelIndices addresses are int for the kernel ABI (a disabled slot is
// -1); container subscripts take size_t, so the explicit conversion lives in
// one place (same helper as the other gpu tests).
constexpr std::size_t to_index(int field) {
    return static_cast<std::size_t>(field);
}

constexpr const char *kRelModelPath = "tools/fp32_probe/artifacts/common/model.xml";

fs::path find_probe_model_xml() {
    std::vector<fs::path> candidates;
#ifdef BIKE_REPO_ROOT
    candidates.emplace_back(fs::path(BIKE_REPO_ROOT) / kRelModelPath);
#endif
    candidates.emplace_back(kRelModelPath);
    candidates.emplace_back(fs::path("..") / kRelModelPath);
    candidates.emplace_back(fs::path("../..") / kRelModelPath);
    candidates.emplace_back(fs::path("../../..") / kRelModelPath);
    for (const auto &p : candidates) {
        std::error_code ec;
        if (fs::is_regular_file(p, ec)) {
            return fs::canonical(p, ec);
        }
    }
    return {};
}

#if defined(__APPLE__)
void test_metal_shader_and_buffer_reuse() {
    using namespace bike::gpu;

    require(bike_metal_init(nullptr) == 0,
            "bike_metal_init succeeds");
    const char *dev_name = bike_metal_device_name();
    require(dev_name != nullptr && !std::string_view(dev_name).empty(),
            "bike_metal_device_name returns a non-empty device name");

    constexpr int kNWorld = 8;
    const GpuModelIndices idx{};
    const std::size_t nq = to_index(idx.nq);
    const std::size_t nv = to_index(idx.nv);
    const std::size_t nu = to_index(idx.nu);
    const std::size_t nbody3 = 3 * to_index(idx.nbody);
    std::vector<GpuWorldConfig> configs(static_cast<std::size_t>(kNWorld));
    for (int w = 0; w < kNWorld; ++w) {
        bike_gpu_init_default_config(&configs[to_index(w)]);
    }
    std::vector<GpuWorldState> states(static_cast<std::size_t>(kNWorld));

    // Blocker 4 regression: verify Metal never serves a stale cached MTLBuffer
    // when the same host pointer address is reused with different data across
    // calls. The vector buffer keeps one address for both launches.
    std::vector<float> qpos(static_cast<std::size_t>(kNWorld) * nq, 0.0f);
    const std::span<float> qpos_rows{qpos};
    for (int w = 0; w < kNWorld; ++w) {
        const std::span<float> qpos_w = qpos_rows.subspan(to_index(w) * nq, nq);
        qpos_w[to_index(idx.crank_spin_qpos)] = 1.0f;
        qpos_w[to_index(idx.rear_wheel_spin_qpos)] = 0.0f;
    }
    require(bike_metal_launch_init_states(
                kNWorld, &idx, configs.data(), 1, states.data(), qpos.data()) == 0,
            "bike_metal_launch_init_states succeeds on the first upload");
    const float ratio = 22.0f / 51.0f;
    for (int w = 0; w < kNWorld; ++w) {
        require_near(states[to_index(w)].drivetrain.freehub_boundary_rad,
                     1.0f * ratio, 1e-5f,
                     "freehub boundary initialized from crank qpos");
    }

    // Mutate in place at the exact same address and re-launch
    for (int w = 0; w < kNWorld; ++w) {
        qpos_rows.subspan(to_index(w) * nq, nq)[to_index(idx.crank_spin_qpos)] = 10.0f;
    }
    require(bike_metal_launch_init_states(
                kNWorld, &idx, configs.data(), 1, states.data(), qpos.data()) == 0,
            "bike_metal_launch_init_states succeeds on the reused address");
    for (int w = 0; w < kNWorld; ++w) {
        require_near(states[to_index(w)].drivetrain.freehub_boundary_rad,
                     10.0f * ratio, 1e-5f,
                     "freehub boundary re-initialized from mutated qpos at the same address");
    }

    // Also verify bike_metal_launch_writers with 32x1 and 45x10 tire ray configurations
    std::vector<float> hf(512, 0.0f);
    std::vector<float> writers_qpos(static_cast<std::size_t>(kNWorld) * nq, 0.0f);
    std::vector<float> qvel(static_cast<std::size_t>(kNWorld) * nv, 0.0f);
    std::vector<float> xpos(static_cast<std::size_t>(kNWorld) * nbody3, 0.0f);
    std::vector<float> qfrc(static_cast<std::size_t>(kNWorld) * nv, 0.0f);
    std::vector<float> ctrl(static_cast<std::size_t>(kNWorld) * nu, 0.0f);
    std::vector<float> tendon(static_cast<std::size_t>(kNWorld) * 2, 0.0f);
    const std::span<float> writers_qpos_rows{writers_qpos};
    const std::span<float> xpos_rows{xpos};
    const std::span<const float> qfrc_rows{qfrc};

    for (int w = 0; w < kNWorld; ++w) {
        const std::size_t wi = to_index(w);
        const std::span<float> qpos_w = writers_qpos_rows.subspan(wi * nq, nq);
        const std::span<float> xpos_w = xpos_rows.subspan(wi * nbody3, nbody3);
        qpos_w[to_index(idx.fork_travel_qpos)] = 0.050f;
        qpos_w[to_index(idx.shock_stroke_qpos)] = 0.020f;
        xpos_w[3 * to_index(idx.frame_body) + 0] = 2.0f;
        xpos_w[3 * to_index(idx.frame_body) + 2] = 0.56f;
        xpos_w[3 * to_index(idx.steer_body) + 0] = 2.0f + 0.532605f;
        xpos_w[3 * to_index(idx.steer_body) + 2] = 0.56f + 0.538145f;
        xpos_w[3 * to_index(idx.front_wheel_body) + 0] = 2.60f;
        xpos_w[3 * to_index(idx.front_wheel_body) + 2] = 0.34f;
        xpos_w[3 * to_index(idx.rear_wheel_body) + 0] = 1.40f;
        xpos_w[3 * to_index(idx.rear_wheel_body) + 2] = 0.33f;
    }

    require(bike_metal_launch_writers(
                kNWorld, 0.001f, &idx, hf.data(), static_cast<int>(hf.size()),
                0.0f, 0.01f, configs.data(), 1, states.data(),
                writers_qpos.data(), qvel.data(), xpos.data(),
                qfrc.data(), ctrl.data(), tendon.data(), 2) == 0,
            "bike_metal_launch_writers succeeds");
    for (int w = 0; w < kNWorld; ++w) {
        const std::span<const float> qfrc_w = qfrc_rows.subspan(to_index(w) * nv, nv);
        require(std::isfinite(qfrc_w[to_index(idx.root_z_dof)]) &&
                    qfrc_w[to_index(idx.root_z_dof)] > 100.0f,
                "root_z tire ground reaction is strongly positive");
        require(std::isfinite(qfrc_w[to_index(idx.fork_travel_dof)]) &&
                    std::fabs(qfrc_w[to_index(idx.fork_travel_dof)]) > 10.0f,
                "fork DOF force is non-trivial");
    }
}

// Launch ABI argument validation: every invalid argument returns -1 and leaves
// the state buffer untouched.
void test_metal_launch_argument_validation() {
    using namespace bike::gpu;

    const GpuModelIndices idx{};
    GpuWorldConfig cfg{};
    bike_gpu_init_default_config(&cfg);
    std::vector<GpuWorldState> states(2);
    constexpr float kSentinel = 123.0f;
    for (auto &s : states) {
        s.drivetrain.freehub_boundary_rad = kSentinel;
    }
    std::vector<float> buf(4096, 0.0f);
    const float *qpos = buf.data();

    require(bike_metal_launch_init_states(0, &idx, &cfg, 1, states.data(), qpos) == -1,
            "init_states rejects nworld == 0");
    require(bike_metal_launch_init_states(-3, &idx, &cfg, 1, states.data(), qpos) == -1,
            "init_states rejects nworld < 0");
    require(bike_metal_launch_init_states(2, nullptr, &cfg, 1, states.data(), qpos) == -1,
            "init_states rejects a null model-index pointer");
    require(bike_metal_launch_init_states(2, &idx, nullptr, 1, states.data(), qpos) == -1,
            "init_states rejects a null config pointer");
    require(bike_metal_launch_init_states(2, &idx, &cfg, 1, nullptr, qpos) == -1,
            "init_states rejects a null state pointer");
    for (const auto &s : states) {
        require(s.drivetrain.freehub_boundary_rad == kSentinel,
                "rejected init_states leaves the state buffer untouched");
    }

    require(bike_metal_launch_writers(
                0, 0.001f, &idx, nullptr, 0, 0.0f, 0.01f, &cfg, 1, states.data(),
                qpos, qpos, qpos, buf.data(), buf.data(), buf.data(), 2) == -1,
            "writers rejects nworld == 0");
    require(bike_metal_launch_writers(
                -1, 0.001f, &idx, nullptr, 0, 0.0f, 0.01f, &cfg, 1, states.data(),
                qpos, qpos, qpos, buf.data(), buf.data(), buf.data(), 2) == -1,
            "writers rejects nworld < 0");
    require(bike_metal_launch_writers(
                2, 0.001f, nullptr, nullptr, 0, 0.0f, 0.01f, &cfg, 1, states.data(),
                qpos, qpos, qpos, buf.data(), buf.data(), buf.data(), 2) == -1,
            "writers rejects a null model-index pointer");
    require(bike_metal_launch_writers(
                2, 0.001f, &idx, nullptr, 0, 0.0f, 0.01f, nullptr, 1, states.data(),
                qpos, qpos, qpos, buf.data(), buf.data(), buf.data(), 2) == -1,
            "writers rejects a null config pointer");
    require(bike_metal_launch_writers(
                2, 0.001f, &idx, nullptr, 0, 0.0f, 0.01f, &cfg, 1, states.data(),
                nullptr, qpos, qpos, buf.data(), buf.data(), buf.data(), 2) == -1,
            "writers rejects a null qpos pointer");
    for (const auto &s : states) {
        require(s.drivetrain.freehub_boundary_rad == kSentinel,
                "rejected writers leaves the state buffer untouched");
    }
}
#endif // defined(__APPLE__)

void test_cpu_and_metal_hybrid_rollouts() {
    using namespace bike::gpu;

    const fs::path xml_path = find_probe_model_xml();
    if (xml_path.empty()) {
        std::cout << "SKIP model.xml rollout check (" << kRelModelPath << " not found)\n";
        return;
    }
    const std::string xml_str = xml_path.string();

    constexpr int kNWorld = 4;
    constexpr int kSteps = 400;
    const GpuModelIndices idx{};
    std::vector<GpuWorldConfig> configs(static_cast<std::size_t>(kNWorld));
    for (int w = 0; w < kNWorld; ++w) {
        bike_gpu_init_default_config(&configs[to_index(w)]);
    }

    // 1. Run CPU batch rollout using model's default heightfield (d_hf_heights = nullptr)
    std::vector<float> cpu_fork_mm(static_cast<std::size_t>(kNWorld), 0.0f);
    std::vector<float> cpu_shock_mm(static_cast<std::size_t>(kNWorld), 0.0f);
    std::vector<float> cpu_root_x_m(static_cast<std::size_t>(kNWorld), 0.0f);
    int cpu_finite = 0;
    double cpu_ms = 0.0;

    const int rc_cpu = bike_cpu_run_batch_sweep(
        xml_str.c_str(), kNWorld, kSteps, 2, &idx,
        /*d_hf_heights=*/nullptr, 0, 0.0f, 0.005f,
        configs.data(), 1,
        /*init_qpos=*/nullptr, /*init_qvel=*/nullptr,
        cpu_fork_mm.data(), cpu_shock_mm.data(), cpu_root_x_m.data(),
        &cpu_finite, &cpu_ms
    );
    require(rc_cpu == 0, "bike_cpu_run_batch_sweep returns 0");
    require(cpu_finite == 1, "CPU rollout stays finite in every world");

    // Rollout bounds derived from the captured start, replacing the earlier
    // invented 3.5 m/s assumption. With init_qpos/init_qvel null the runner
    // loads captured_bootstrap.json next to model.xml (vx = 0.058 m/s).
    // Measured on this harness (nworld 4, kSteps 400 = 0.20 s at dt = 0.0005 s,
    // null init): root_x = 2.055095 m (+0.039 m over the window),
    // fork peak = 59.88 mm, shock peak = 21.35 mm, Metal-CPU = 0 m.
    constexpr double kX0M = 2.0163204663006566; // captured_bootstrap.json integration_state[1] (qpos[0])
    constexpr double kMinAdvanceM = 0.01; // floor for measurable forward motion under pedal_demand 0.65; measured +0.039 m in 0.20 s
    constexpr double kMaxAdvanceM = 0.50; // 2.5 m/s max plausible speed x 0.20 s rollout window
    for (int w = 0; w < kNWorld; ++w) {
        const std::size_t wi = to_index(w);
        require_above(static_cast<double>(cpu_root_x_m[wi]), kX0M + kMinAdvanceM,
                      "cpu root_x advances past the captured start");
        require_below(static_cast<double>(cpu_root_x_m[wi]), kX0M + kMaxAdvanceM,
                      "cpu root_x stays below the max-plausible-speed bound");
        // Physical bands kept from the pre-port test: fork travel peak in
        // (20, 130) mm and shock stroke peak in (5, 60) mm bracket the
        // measured 59.88 mm / 21.35 mm against both a stalled suspension and
        // a bottomed-out one.
        require(cpu_fork_mm[wi] > 20.0f && cpu_fork_mm[wi] < 130.0f,
                "cpu peak fork travel inside the physical band");
        require(cpu_shock_mm[wi] > 5.0f && cpu_shock_mm[wi] < 60.0f,
                "cpu peak shock stroke inside the physical band");
    }

    // 2. Verify custom caller d_hf_heights is respected (not overwritten by model->hfield_data)
    std::vector<float> custom_hf_low(2000, -0.040f);
    std::vector<float> custom_hf_high(2000, +0.025f);
    std::vector<float> fork_low(static_cast<std::size_t>(kNWorld), 0.0f);
    std::vector<float> shock_low(static_cast<std::size_t>(kNWorld), 0.0f);
    std::vector<float> rx_low(static_cast<std::size_t>(kNWorld), 0.0f);
    std::vector<float> fork_high(static_cast<std::size_t>(kNWorld), 0.0f);
    std::vector<float> shock_high(static_cast<std::size_t>(kNWorld), 0.0f);
    std::vector<float> rx_high(static_cast<std::size_t>(kNWorld), 0.0f);
    int fin_low = 0, fin_high = 0;
    double ms_low = 0.0, ms_high = 0.0;

    require(bike_cpu_run_batch_sweep(
                xml_str.c_str(), kNWorld, 50, 2, &idx,
                custom_hf_low.data(), static_cast<int>(custom_hf_low.size()), -2.0f, 0.01f,
                configs.data(), 1, nullptr, nullptr,
                fork_low.data(), shock_low.data(), rx_low.data(), &fin_low, &ms_low) == 0,
            "custom heightfield (low) sweep returns 0");
    require(bike_cpu_run_batch_sweep(
                xml_str.c_str(), kNWorld, 50, 2, &idx,
                custom_hf_high.data(), static_cast<int>(custom_hf_high.size()), -2.0f, 0.01f,
                configs.data(), 1, nullptr, nullptr,
                fork_high.data(), shock_high.data(), rx_high.data(), &fin_high, &ms_high) == 0,
            "custom heightfield (high) sweep returns 0");
    require(fin_low == 1 && fin_high == 1,
            "custom heightfield sweeps stay finite");
    require(fork_high[0] > fork_low[0] + 1.0f,
            "caller's heightfield is used: higher terrain raises the fork peak");

#if defined(__APPLE__)
    // 3. Verify Metal hybrid rollout matches CPU rollout on the same 400-step scenario
    std::vector<float> mtl_fork_mm(static_cast<std::size_t>(kNWorld), 0.0f);
    std::vector<float> mtl_shock_mm(static_cast<std::size_t>(kNWorld), 0.0f);
    std::vector<float> mtl_root_x_m(static_cast<std::size_t>(kNWorld), 0.0f);
    int mtl_finite = 0;
    double mtl_ms = 0.0;

    const int rc_mtl = bike_metal_run_hybrid_sweep(
        xml_str.c_str(), kNWorld, kSteps, 2, &idx,
        /*d_hf_heights=*/nullptr, 0, 0.0f, 0.005f,
        configs.data(), 1,
        /*init_qpos=*/nullptr, /*init_qvel=*/nullptr,
        mtl_fork_mm.data(), mtl_shock_mm.data(), mtl_root_x_m.data(),
        &mtl_finite, &mtl_ms
    );
    require(rc_mtl == 0, "bike_metal_run_hybrid_sweep returns 0");
    require(mtl_finite == 1, "Metal rollout stays finite in every world");
    for (int w = 0; w < kNWorld; ++w) {
        const std::size_t wi = to_index(w);
        require_above(static_cast<double>(mtl_root_x_m[wi]), kX0M + kMinAdvanceM,
                      "metal root_x advances past the captured start");
        require_below(static_cast<double>(mtl_root_x_m[wi]), kX0M + kMaxAdvanceM,
                      "metal root_x stays below the max-plausible-speed bound");
        require_near(mtl_root_x_m[wi], cpu_root_x_m[wi], 1e-2f,
                     "metal root_x matches cpu root_x");
        require_near(mtl_fork_mm[wi], cpu_fork_mm[wi], 1.0f,
                     "metal fork peak matches cpu fork peak");
    }
#endif
}

#if defined(__APPLE__)
constexpr std::array<TestCase, 3> cases{{
    {.name = "metal_shader_and_buffer_reuse",
     .run = test_metal_shader_and_buffer_reuse},
    {.name = "metal_launch_argument_validation",
     .run = test_metal_launch_argument_validation},
    {.name = "cpu_and_metal_hybrid_rollouts",
     .run = test_cpu_and_metal_hybrid_rollouts},
}};
#else
constexpr std::array<TestCase, 1> cases{{
    {.name = "cpu_and_metal_hybrid_rollouts",
     .run = test_cpu_and_metal_hybrid_rollouts},
}};
#endif

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
            for (const TestCase &test_case : cases) {
                std::cout << test_case.name << '\n';
            }
            return 0;
        }
        if (arguments.size() > 2) {
            std::cerr << "usage: bike_test_gpu_batch_runners [case-name|--list]\n";
            return 2;
        }

        const std::string_view requested =
            arguments.size() == 1 ? "all" : std::string_view(arguments[1]);
        bool found = requested == "all";
        int status = 0;
        for (const TestCase &test_case : cases) {
            if (requested == "all" || requested == test_case.name) {
                found = true;
                if (run_case(test_case) != 0) {
                    status = 1;
                }
            }
        }
        if (!found) {
            std::cerr << "unknown gpu batch_runners case: " << requested << '\n';
            return 2;
        }
        return status;
    } catch (const std::exception &error) {
        // A throwing argument envelope is a harness failure, not a usage error.
        std::cerr << "FAIL argument handling: " << error.what() << '\n';
        return 1;
    }
}
