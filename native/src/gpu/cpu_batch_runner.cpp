// native/src/gpu/cpu_batch_runner.cpp — Multi-threaded C++ CPU batch runner
// executing `gpu_step_world_writers` + `mj_step` across all CPU cores with
// zero per-step heap allocations and zero Python/GIL overhead.

#include "batch_step.hpp"

#include <mujoco/mujoco.h>

#include "model_assets.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <span>
#include <string>
#include <thread>
#include <vector>

namespace {

// Path of a side-car asset next to the model XML (diagnostics only).
std::string asset_file_path(const char *xml_path, const char *file_name) {
    return (std::filesystem::path(bike::gpu::assets::asset_dir(xml_path)) / file_name).string();
}

// The single raw pointer -> std::span boundary of this TU. The extern "C" batch
// entry points receive (pointer, count) ranges whose extents the caller owns
// (d_configs/d_hf_heights/init_qpos/init_qvel/out_* sized by nworld, the model
// counts or the caller's sample count); MuJoCo likewise exposes mjModel/mjData
// arrays as raw pointers sized by the adjacent model counts (qpos/qvel/xpos/
// qfrc_applied/ctrl by nq/nv/nbody*3/nu, tendon_range by ntendon*2). Every such
// range is wrapped once through this helper; the (pointer, count) construction
// itself is -Wunsafe-buffer-usage-in-container — suppressed here, scoped
// exactly like native/src/gpu/model_assets.hpp's detail::model_span and
// native/src/platform/argv.hpp's main_arguments.
// Register: docs/TESTING.md suppression table.
template <typename T>
[[nodiscard]] std::span<T> boundary_span(T *ptr, std::size_t count) noexcept {
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    return std::span<T>(ptr, count);
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
}

// Container subscripts take size_t; the C ABI dims, GpuModelIndices fields and
// the (kMax-checked) mjModel counts are int, so the explicit conversion lives
// in one place (same helper shape as the gpu tests' to_index).
constexpr std::size_t to_index(int field) {
    return static_cast<std::size_t>(field);
}

// The three scalar int dims arrive as adjacent, easily swapped ABI slots;
// binding them once into a named aggregate keeps every use site self-labeled.
struct BatchDims {
    int nworld;
    int n_steps;
    int num_threads;
};

// The four optional float* outputs, each an nworld-sized range or empty when
// the caller passed nullptr. One aggregate init keeps the boundary staging in
// a single place; empty() replaces the null check at the write sites.
struct OutputViews {
    std::span<float> peak_fork_mm;
    std::span<float> peak_shock_mm;
    std::span<float> final_root_x_m;
    std::span<float> final_root_z_m;
};

} // namespace

extern "C" {

int bike_cpu_run_batch_ex(
    const char *xml_path,
    int nworld,
    int n_steps,
    int num_threads,
    const bike::gpu::GpuModelIndices *d_idx,
    const float *d_hf_heights,
    int hf_num_samples,
    float hf_x_min,
    float hf_dx,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    const double *init_qpos,
    const double *init_qvel,
    float *out_peak_fork_mm,
    float *out_peak_shock_mm,
    float *out_final_root_x_m,
    int *out_all_finite,
    double *out_elapsed_ms,
    float *out_final_root_z_m
);
int bike_cpu_run_batch(
    const char *xml_path,
    int nworld,
    int n_steps,
    int num_threads,
    const bike::gpu::GpuModelIndices *d_idx,
    const float *d_hf_heights,
    int hf_num_samples,
    float hf_x_min,
    float hf_dx,
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
int bike_cpu_run_batch_sweep(
    const char *xml_path,
    int nworld,
    int n_steps,
    int num_threads,
    const bike::gpu::GpuModelIndices *d_idx,
    const float *d_hf_heights,
    int hf_num_samples,
    float hf_x_min,
    float hf_dx,
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

// Return codes, shared by bike_cpu_run_batch_ex, bike_cpu_run_batch and bike_cpu_run_batch_sweep
// (the last two forward to bike_cpu_run_batch_ex):
//    0  success
//   -1  invalid arguments: xml_path or d_configs is null, or nworld or n_steps <= 0
//   -2  model load failure, or model / idx dimensions exceed the kMax bounds
//   -5  initial-state asset error, only when init_qpos or init_qvel is null: captured_bootstrap.json
//       next to xml_path is missing, malformed, or its nq/nv differ from the model
//   -6  heightfield asset error: hfield_data.bin next to xml_path is missing or does not hold exactly
//       nhfielddata binary64 values (required whenever the model has a heightfield), or no heights were
//       passed and the model has no usable 1D heightfield
// Errors -5 and -6 print the asset status name and the file path to stderr.

int bike_cpu_run_batch_ex(
    const char *xml_path,
    int nworld,
    int n_steps,
    int num_threads,
    const bike::gpu::GpuModelIndices *d_idx,
    const float *d_hf_heights,
    int hf_num_samples,
    float hf_x_min,
    float hf_dx,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    const double *init_qpos,
    const double *init_qvel,
    float *out_peak_fork_mm,
    float *out_peak_shock_mm,
    float *out_final_root_x_m,
    int *out_all_finite,
    double *out_elapsed_ms,
    float *out_final_root_z_m
) {
    using bike::gpu::kMaxNbody;
    using bike::gpu::kMaxNq;
    using bike::gpu::kMaxNu;
    using bike::gpu::kMaxNv;

    if (xml_path == nullptr || nworld <= 0 || n_steps <= 0 || d_configs == nullptr) {
        return -1;
    }
    const BatchDims dims{.nworld = nworld, .n_steps = n_steps, .num_threads = num_threads};

    std::array<char, 1024> err{};
    mjModel *m_base = mj_loadXML(xml_path, nullptr, err.data(), static_cast<int>(err.size()));
    if (m_base == nullptr) {
        return -2;
    }

    // The rollout's contacts read m->hfield_data, so the side-car is required for any model with a heightfield.
    const bike::gpu::assets::AssetStatus hf_bin_status =
        bike::gpu::assets::load_hfield_bin(m_base, xml_path);
    if (hf_bin_status != bike::gpu::assets::AssetStatus::kOk) {
        std::cerr << "bike_cpu_run_batch: hfield asset '"
                  << bike::gpu::assets::asset_status_name(hf_bin_status) << "' ("
                  << asset_file_path(xml_path, bike::gpu::assets::kHfieldBinName) << ")\n";
        mj_deleteModel(m_base);
        return -6;
    }

    const bike::gpu::GpuModelIndices idx =
        (d_idx != nullptr) ? *d_idx : bike::gpu::GpuModelIndices{};
    if (m_base->nq > kMaxNq || m_base->nv > kMaxNv || m_base->nu > kMaxNu || m_base->nbody > kMaxNbody) {
        std::cerr << "bike_cpu_run_batch: model dimensions exceed kMax bounds\n";
        mj_deleteModel(m_base);
        return -2;
    }
    if (idx.nq > kMaxNq || idx.nv > kMaxNv || idx.nu > kMaxNu || idx.nbody > kMaxNbody) {
        std::cerr << "bike_cpu_run_batch: idx dimensions exceed kMax bounds\n";
        mj_deleteModel(m_base);
        return -2;
    }

    // kMax bounds above keep the mjtSize -> int narrowing lossless.
    const int nq = static_cast<int>(m_base->nq);
    const int nv = static_cast<int>(m_base->nv);
    const int nu = static_cast<int>(m_base->nu);
    const int nbody3 = 3 * static_cast<int>(m_base->nbody);
    const int ntendon = static_cast<int>(m_base->ntendon);

    // Caller heights, when usable, are adopted as given; otherwise the model's own 1D
    // heightfield is extracted. The caller's three hf_* scalars bind into one descriptor.
    bike::gpu::assets::Heightfield1D model_hf;
    bike::gpu::GpuHeightfield1D hf{};
    if (d_hf_heights != nullptr && hf_num_samples > 0) {
        const float inv_dx = (hf_dx > 0.0f) ? (1.0f / hf_dx) : 1.0f;
        hf = bike::gpu::GpuHeightfield1D{
            .heights_m = d_hf_heights,
            .num_samples = hf_num_samples,
            .x_min_m = hf_x_min,
            .dx_m = hf_dx,
            .inv_dx_m = inv_dx,
        };
    } else {
        const bike::gpu::assets::AssetStatus hf_status =
            bike::gpu::assets::extract_heightfield_1d(m_base, model_hf);
        if (hf_status != bike::gpu::assets::AssetStatus::kOk) {
            std::cerr << "bike_cpu_run_batch: model heightfield extraction '"
                      << bike::gpu::assets::asset_status_name(hf_status) << "'\n";
            mj_deleteModel(m_base);
            return -6;
        }
        const float inv_dx = (model_hf.dx_m > 0.0f) ? (1.0f / model_hf.dx_m) : 1.0f;
        hf = bike::gpu::GpuHeightfield1D{
            .heights_m = model_hf.heights.data(),
            .num_samples = static_cast<int>(model_hf.heights.size()),
            .x_min_m = model_hf.x_min_m,
            .dx_m = model_hf.dx_m,
            .inv_dx_m = inv_dx,
        };
    }

    // Initial state: caller pointers are used as given; a null pointer takes the captured bootstrap value.
    const double *qpos_src = init_qpos;
    const double *qvel_src = init_qvel;
    std::vector<double> captured_qpos;
    std::vector<double> captured_qvel;
    if (init_qpos == nullptr || init_qvel == nullptr) {
        const bike::gpu::assets::AssetStatus init_status =
            bike::gpu::assets::load_captured_initial_state(xml_path, nq, nv,
                                                           captured_qpos, captured_qvel);
        if (init_status != bike::gpu::assets::AssetStatus::kOk) {
            std::cerr << "bike_cpu_run_batch: initial state '"
                      << bike::gpu::assets::asset_status_name(init_status) << "' ("
                      << asset_file_path(xml_path, bike::gpu::assets::kCapturedBootstrapName) << ")\n";
            mj_deleteModel(m_base);
            return -5;
        }
        if (qpos_src == nullptr) {
            qpos_src = captured_qpos.data();
        }
        if (qvel_src == nullptr) {
            qvel_src = captured_qvel.data();
        }
    }
    const std::span<const double> init_qpos_v = boundary_span(qpos_src, to_index(nq));
    const std::span<const double> init_qvel_v = boundary_span(qvel_src, to_index(nv));

    std::vector<double> qpos0(to_index(nq), 0.0);
    std::vector<double> qvel0(to_index(nv), 0.0);
    for (std::size_t i = 0; i < to_index(nq); ++i) {
        qpos0[i] = init_qpos_v[i];
    }
    for (std::size_t i = 0; i < to_index(nv); ++i) {
        qvel0[i] = init_qvel_v[i];
    }

    const std::span<const bike::gpu::GpuWorldConfig> configs_v =
        boundary_span(d_configs, (per_world_config != 0) ? to_index(dims.nworld) : 1);
    const std::size_t nworld_u = to_index(dims.nworld);
    const OutputViews out{
        .peak_fork_mm = (out_peak_fork_mm != nullptr)
                            ? boundary_span(out_peak_fork_mm, nworld_u)
                            : std::span<float>{},
        .peak_shock_mm = (out_peak_shock_mm != nullptr)
                             ? boundary_span(out_peak_shock_mm, nworld_u)
                             : std::span<float>{},
        .final_root_x_m = (out_final_root_x_m != nullptr)
                              ? boundary_span(out_final_root_x_m, nworld_u)
                              : std::span<float>{},
        .final_root_z_m = (out_final_root_z_m != nullptr)
                              ? boundary_span(out_final_root_z_m, nworld_u)
                              : std::span<float>{},
    };

    const int hw_threads = static_cast<int>(std::thread::hardware_concurrency());
    const int n_workers = std::max(
        1, std::min(dims.nworld,
                    (dims.num_threads > 0) ? dims.num_threads : ((hw_threads > 0) ? hw_threads : 4)));

    const auto make_data_fn = &mj_makeData;
    const auto reset_data_fn = &mj_resetData;
    const auto forward_fn = &mj_forward;
    const auto step_fn = &mj_step;

    std::vector<mjModel *> worker_models(to_index(n_workers), nullptr);
    std::vector<mjData *> worker_data(to_index(n_workers), nullptr);
    for (int t = 0; t < n_workers; ++t) {
        worker_models[to_index(t)] = mj_copyModel(nullptr, m_base);
        worker_data[to_index(t)] = make_data_fn(worker_models[to_index(t)]);
    }

    const float dt_s = static_cast<float>(m_base->opt.timestep);

    std::atomic<int> next_world{0};
    std::atomic<int> all_finite_flag{1};

    const auto t_start = std::chrono::steady_clock::now();

    const auto worker_fn = [&](int tid) {
        mjModel *m_t = worker_models[to_index(tid)];
        mjData *d_t = worker_data[to_index(tid)];
        // MuJoCo-owned worker arrays, wrapped once through the boundary helper;
        // the copies share m_base's dims.
        const std::span<mjtNum> d_qpos = boundary_span(d_t->qpos, to_index(nq));
        const std::span<mjtNum> d_qvel = boundary_span(d_t->qvel, to_index(nv));
        const std::span<mjtNum> d_xpos = boundary_span(d_t->xpos, to_index(nbody3));
        const std::span<mjtNum> d_qfrc_applied = boundary_span(d_t->qfrc_applied, to_index(nv));
        const std::span<mjtNum> d_ctrl = boundary_span(d_t->ctrl, to_index(nu));
        const std::span<mjtNum> t_tendon_range =
            boundary_span(m_t->tendon_range, 2 * to_index(ntendon));
        const std::span<const mjtNum> base_tendon_range =
            boundary_span(m_base->tendon_range, 2 * to_index(ntendon));

        while (true) {
            const int w = next_world.fetch_add(1, std::memory_order_relaxed);
            if (w >= dims.nworld) {
                break;
            }

            const bike::gpu::GpuWorldConfig &cfg_w =
                configs_v[(per_world_config != 0) ? to_index(w) : 0];

            reset_data_fn(m_t, d_t);
            for (std::size_t i = 0; i < to_index(nq); ++i) {
                d_qpos[i] = static_cast<mjtNum>(qpos0[i]);
            }
            for (std::size_t i = 0; i < to_index(nv); ++i) {
                d_qvel[i] = static_cast<mjtNum>(qvel0[i]);
            }
            alignas(16) std::array<float, 2> tendon_f32{0.0f, 0.0f};
            if (ntendon > 0 && idx.freehub_tendon >= 0 && idx.freehub_tendon < ntendon) {
                const std::size_t ft = 2 * to_index(idx.freehub_tendon);
                t_tendon_range[ft + 0] = base_tendon_range[ft + 0];
                t_tendon_range[ft + 1] = base_tendon_range[ft + 1];
                tendon_f32[0] = static_cast<float>(base_tendon_range[ft + 0]);
                tendon_f32[1] = static_cast<float>(base_tendon_range[ft + 1]);
            }
            forward_fn(m_t, d_t);

            bike::gpu::GpuWorldState st_w{};
            const float crank0 =
                (idx.crank_spin_qpos >= 0 && idx.crank_spin_qpos < nq)
                    ? static_cast<float>(d_qpos[to_index(idx.crank_spin_qpos)])
                    : 0.0f;
            const float rwheel0 =
                (idx.rear_wheel_spin_qpos >= 0 && idx.rear_wheel_spin_qpos < nq)
                    ? static_cast<float>(d_qpos[to_index(idx.rear_wheel_spin_qpos)])
                    : 0.0f;
            bike::gpu::gpu_init_world_state(st_w, cfg_w.drivetrain, crank0, rwheel0);

            float max_fork_m = 0.0f;
            float max_shock_m = 0.0f;

            const auto step_world = [&]() {
                alignas(64) std::array<float, kMaxNq> qpos_f32{};
                alignas(64) std::array<float, kMaxNv> qvel_f32{};
                alignas(64) std::array<float, 3 * to_index(kMaxNbody)> xpos_f32{};
                alignas(64) std::array<float, kMaxNv> qfrc_f32{};
                alignas(64) std::array<float, kMaxNu> ctrl_f32{};

                for (std::size_t i = 0; i < to_index(nq); ++i) {
                    qpos_f32[i] = static_cast<float>(d_qpos[i]);
                }
                for (std::size_t i = 0; i < to_index(nv); ++i) {
                    qvel_f32[i] = static_cast<float>(d_qvel[i]);
                }
                for (std::size_t i = 0; i < to_index(nbody3); ++i) {
                    xpos_f32[i] = static_cast<float>(d_xpos[i]);
                }

                bike::gpu::gpu_step_world_writers(
                    0,
                    dt_s,
                    idx,
                    hf,
                    &cfg_w,
                    0,
                    &st_w,
                    qpos_f32.data(),
                    qvel_f32.data(),
                    xpos_f32.data(),
                    qfrc_f32.data(),
                    ctrl_f32.data(),
                    tendon_f32.data(),
                    2
                );

                for (std::size_t i = 0; i < to_index(nv); ++i) {
                    d_qfrc_applied[i] = static_cast<mjtNum>(qfrc_f32[i]);
                }
                for (std::size_t i = 0; i < to_index(nu); ++i) {
                    d_ctrl[i] = static_cast<mjtNum>(ctrl_f32[i]);
                }
                if (ntendon > 0 && idx.freehub_tendon >= 0 && idx.freehub_tendon < ntendon) {
                    const std::size_t ft = 2 * to_index(idx.freehub_tendon);
                    t_tendon_range[ft + 0] = static_cast<mjtNum>(tendon_f32[0]);
                    t_tendon_range[ft + 1] = static_cast<mjtNum>(tendon_f32[1]);
                }

                step_fn(m_t, d_t);

                if (idx.fork_travel_qpos >= 0 && idx.fork_travel_qpos < nq) {
                    const float fk = static_cast<float>(d_qpos[to_index(idx.fork_travel_qpos)]);
                    if (fk > max_fork_m) {
                        max_fork_m = fk;
                    }
                }
                if (idx.shock_stroke_qpos >= 0 && idx.shock_stroke_qpos < nq) {
                    const float sh = static_cast<float>(d_qpos[to_index(idx.shock_stroke_qpos)]);
                    if (sh > max_shock_m) {
                        max_shock_m = sh;
                    }
                }
            };

            for (int step = 0; step < dims.n_steps; ++step) {
                step_world();
            }

            bool world_finite = true;
            for (std::size_t i = 0; i < to_index(nq); ++i) {
                if (!std::isfinite(static_cast<double>(d_qpos[i]))) {
                    world_finite = false;
                    break;
                }
            }
            if (world_finite) {
                for (std::size_t i = 0; i < to_index(nv); ++i) {
                    if (!std::isfinite(static_cast<double>(d_qvel[i]))) {
                        world_finite = false;
                        break;
                    }
                }
            }
            if (!world_finite) {
                all_finite_flag.store(0, std::memory_order_relaxed);
            }

            if (!out.peak_fork_mm.empty()) {
                out.peak_fork_mm[to_index(w)] = max_fork_m * 1000.0f;
            }
            if (!out.peak_shock_mm.empty()) {
                out.peak_shock_mm[to_index(w)] = max_shock_m * 1000.0f;
            }
            if (!out.final_root_x_m.empty()) {
                out.final_root_x_m[to_index(w)] =
                    (idx.root_x_qpos >= 0 && idx.root_x_qpos < nq)
                        ? static_cast<float>(d_qpos[to_index(idx.root_x_qpos)])
                        : 0.0f;
            }
            if (!out.final_root_z_m.empty()) {
                out.final_root_z_m[to_index(w)] =
                    (idx.root_z_qpos >= 0 && idx.root_z_qpos < nq)
                        ? static_cast<float>(d_qpos[to_index(idx.root_z_qpos)])
                        : 0.0f;
            }
        }
    };

    std::vector<std::thread> threads;
    threads.reserve(to_index(n_workers));
    for (int t = 0; t < n_workers; ++t) {
        threads.emplace_back(worker_fn, t);
    }
    for (auto &th : threads) {
        th.join();
    }

    const auto t_end = std::chrono::steady_clock::now();
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();

    if (out_all_finite != nullptr) {
        *out_all_finite = all_finite_flag.load(std::memory_order_relaxed);
    }
    if (out_elapsed_ms != nullptr) {
        *out_elapsed_ms = elapsed_ms;
    }

    for (int t = 0; t < n_workers; ++t) {
        mj_deleteData(worker_data[to_index(t)]);
        mj_deleteModel(worker_models[to_index(t)]);
    }
    mj_deleteModel(m_base);
    return 0;
}

int bike_cpu_run_batch(
    const char *xml_path,
    int nworld,
    int n_steps,
    int num_threads,
    const bike::gpu::GpuModelIndices *d_idx,
    const float *d_hf_heights,
    int hf_num_samples,
    float hf_x_min,
    float hf_dx,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    const double *init_qpos,
    const double *init_qvel,
    float *out_peak_fork_mm,
    float *out_peak_shock_mm,
    float *out_final_root_x_m,
    int *out_all_finite,
    double *out_elapsed_ms
) {
    return bike_cpu_run_batch_ex(
        xml_path,
        nworld,
        n_steps,
        num_threads,
        d_idx,
        d_hf_heights,
        hf_num_samples,
        hf_x_min,
        hf_dx,
        d_configs,
        per_world_config,
        init_qpos,
        init_qvel,
        out_peak_fork_mm,
        out_peak_shock_mm,
        out_final_root_x_m,
        out_all_finite,
        out_elapsed_ms,
        nullptr
    );
}

int bike_cpu_run_batch_sweep(
    const char *xml_path,
    int nworld,
    int n_steps,
    int num_threads,
    const bike::gpu::GpuModelIndices *d_idx,
    const float *d_hf_heights,
    int hf_num_samples,
    float hf_x_min,
    float hf_dx,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    const double *init_qpos,
    const double *init_qvel,
    float *out_peak_fork_mm,
    float *out_peak_shock_mm,
    float *out_final_root_x_m,
    int *out_all_finite,
    double *out_elapsed_ms
) {
    return bike_cpu_run_batch(
        xml_path,
        nworld,
        n_steps,
        num_threads,
        d_idx,
        d_hf_heights,
        hf_num_samples,
        hf_x_min,
        hf_dx,
        d_configs,
        per_world_config,
        init_qpos,
        init_qvel,
        out_peak_fork_mm,
        out_peak_shock_mm,
        out_final_root_x_m,
        out_all_finite,
        out_elapsed_ms
    );
}

} // extern "C"
