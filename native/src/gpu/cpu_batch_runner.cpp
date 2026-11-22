// native/src/gpu/cpu_batch_runner.cpp — Multi-threaded C++ CPU batch runner
// executing `gpu_step_world_writers` + MuJoCo step across CPU cores with
// zero per-step heap allocations and zero Python/GIL overhead.

#include "batch_step.hpp"

#include <mujoco/mujoco.h>

#include "model_assets.hpp"
#include "runner_spindle_setup.hpp"
#include "runner_tyre_setup.hpp"

#if defined(BIKE_GPU_STANDALONE_RUNNER)
#include "standalone_mujoco_shim.hpp"
#else
#include "../engine_call.hpp"
#endif

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

std::string asset_file_path(const char *xml_path, const char *file_name) {
    return (std::filesystem::path(bike::gpu::assets::asset_dir(xml_path)) / file_name).string();
}

// The single raw pointer -> std::span boundary of this TU. The extern "C" batch
// entry points receive (pointer, count) ranges whose extents the caller owns
// (d_configs/d_terrain_f32/d_terrain_i32/init_qpos/init_qvel/out_* sized by
// nworld, the model counts or the caller's terrain table counts); MuJoCo likewise
// exposes mjModel/mjData arrays as raw pointers sized by the adjacent model counts
// (qpos/qvel/xpos/xmat/xanchor/xaxis/qfrc_applied/ctrl by nq/nv/nbody*3/nbody*9/
// njnt*3/nu, tendon_range by ntendon*2). Every such range is wrapped once through
// this helper; the (pointer, count) construction itself is
// -Wunsafe-buffer-usage-in-container — suppressed here, scoped exactly like
// native/src/gpu/model_assets.hpp's detail::model_span and
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

constexpr std::size_t to_index(int field) {
    return static_cast<std::size_t>(field);
}

struct BatchDims {
    int nworld;
    int n_steps;
    int num_threads;
};

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
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
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
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
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
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
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

int bike_cpu_build_tyre_and_terrain(
    const char *xml_path,
    int backend_kind,
    int station_count,
    const bike::gpu::GpuModelIndices *d_idx,
    bike::physics_core::GpuTyreModel *out_tyre_model,
    bike::physics_core::GpuTerrainTableDesc *out_terrain_desc,
    float *out_terrain_f32,
    int *io_f32_count,
    bike::physics_core::i32 *out_terrain_i32,
    int *io_i32_count
);

int bike_cpu_build_tyre_and_terrain_for_surface(
    const char *xml_path,
    int backend_kind,
    int station_count,
    int surface_label,
    const bike::gpu::GpuModelIndices *d_idx,
    bike::physics_core::GpuTyreModel *out_tyre_model,
    bike::physics_core::GpuTerrainTableDesc *out_terrain_desc,
    float *out_terrain_f32,
    int *io_f32_count,
    bike::physics_core::i32 *out_terrain_i32,
    int *io_i32_count
);

int bike_cpu_build_spindle_params(
    const char *xml_path,
    const char *strength_json_path,
    const char *envelope_json_path,
    int control_period_steps,
    bike::physics_core::SpindleM2K2Params<float> *out_params
);

int bike_cpu_run_batch_ex(
    const char *xml_path,
    int nworld,
    int n_steps,
    int num_threads,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
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
    using bike::physics_core::kMaxNjnt;

    if (xml_path == nullptr || nworld <= 0 || n_steps <= 0 || d_configs == nullptr) {
        return -1;
    }
    if ((d_tyre_model == nullptr) != (d_terrain_desc == nullptr)) {
        return -1;
    }
    std::span<const float> caller_f32{};
    std::span<const bike::physics_core::i32> caller_i32{};
    if (d_terrain_desc != nullptr) {
        if (d_terrain_desc->f32_count < 0 || d_terrain_desc->i32_count < 0 ||
            (d_terrain_desc->f32_count > 0 && d_terrain_f32 == nullptr) ||
            (d_terrain_desc->i32_count > 0 && d_terrain_i32 == nullptr)) {
            return -1;
        }
        if (d_terrain_desc->f32_count > 0) {
            caller_f32 = boundary_span(d_terrain_f32, to_index(d_terrain_desc->f32_count));
        }
        if (d_terrain_desc->i32_count > 0) {
            caller_i32 = boundary_span(d_terrain_i32, to_index(d_terrain_desc->i32_count));
        }
    }

    const BatchDims dims{.nworld = nworld, .n_steps = n_steps, .num_threads = num_threads};

    std::array<char, 1024> err{};
    mjModel *m_base = mj_loadXML(xml_path, nullptr, err.data(), static_cast<int>(err.size()));
    if (m_base == nullptr) {
        return -2;
    }

    const bike::gpu::assets::AssetStatus hf_bin_status =
        bike::gpu::assets::load_hfield_bin(m_base, xml_path);
    if (hf_bin_status != bike::gpu::assets::AssetStatus::kOk) {
        std::cerr << "bike_cpu_run_batch: hfield asset '"
                  << bike::gpu::assets::asset_status_name(hf_bin_status) << "' ("
                  << asset_file_path(xml_path, bike::gpu::assets::kHfieldBinName) << ")\n";
        mj_deleteModel(m_base);
        return -6;
    }

    if (m_base->nq <= 0 || m_base->nq > kMaxNq ||
        m_base->nv <= 0 || m_base->nv > kMaxNv ||
        m_base->nu < 0 || m_base->nu > kMaxNu ||
        m_base->nbody <= 0 || m_base->nbody > kMaxNbody ||
        m_base->njnt <= 0 || m_base->njnt > kMaxNjnt) {
        std::cerr << "bike_cpu_run_batch: model dimensions exceed kMax bounds\n";
        mj_deleteModel(m_base);
        return -2;
    }

    const bike::gpu::GpuModelIndices idx =
        (d_idx != nullptr) ? *d_idx : bike::gpu::assets::resolve_gpu_model_indices(m_base);
    if (!bike::gpu::gpu_validate_model_indices(idx, static_cast<int>(m_base->ntendon)) ||
        idx.nq != static_cast<int>(m_base->nq) ||
        idx.nv != static_cast<int>(m_base->nv) ||
        idx.nu != static_cast<int>(m_base->nu) ||
        idx.nbody != static_cast<int>(m_base->nbody)) {
        std::cerr << "bike_cpu_run_batch: idx dimensions/indices invalid or mismatch model\n";
        mj_deleteModel(m_base);
        return -2;
    }

    const int nq = static_cast<int>(m_base->nq);
    const int nv = static_cast<int>(m_base->nv);
    const int nu = static_cast<int>(m_base->nu);
    const int nbody3 = 3 * static_cast<int>(m_base->nbody);
    const int nbody9 = 9 * static_cast<int>(m_base->nbody);
    const int njnt3 = 3 * static_cast<int>(m_base->njnt);
    const int ntendon = static_cast<int>(m_base->ntendon);

    // Initial state: caller pointers are used as given; null takes captured_bootstrap.json.
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

    bike::gpu::ResolvedRunnerTyre resolved_tyre{};
    {
        mjData *d_init = engine::make_data(m_base);
        const std::span<mjtNum> d_init_qpos = boundary_span(d_init->qpos, to_index(nq));
        const std::span<mjtNum> d_init_qvel = boundary_span(d_init->qvel, to_index(nv));
        for (std::size_t i = 0; i < to_index(nq); ++i) {
            d_init_qpos[i] = static_cast<mjtNum>(qpos0[i]);
        }
        for (std::size_t i = 0; i < to_index(nv); ++i) {
            d_init_qvel[i] = static_cast<mjtNum>(qvel0[i]);
        }
        engine::forward(m_base, d_init);
        const int rc_tyre = bike::gpu::resolve_runner_tyre_and_terrain(
            m_base,
            d_init,
            idx,
            d_tyre_model,
            d_terrain_desc,
            caller_f32,
            caller_i32,
            resolved_tyre);
        mj_deleteData(d_init);
        if (rc_tyre != 0) {
            mj_deleteModel(m_base);
            return -1;
        }
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

    std::vector<mjModel *> worker_models(to_index(n_workers), nullptr);
    std::vector<mjData *> worker_data(to_index(n_workers), nullptr);
    for (int t = 0; t < n_workers; ++t) {
        worker_models[to_index(t)] = mj_copyModel(nullptr, m_base);
        worker_data[to_index(t)] = engine::make_data(worker_models[to_index(t)]);
    }

    const float dt_s = static_cast<float>(m_base->opt.timestep);
    const float *terrain_f32_ptr =
        resolved_tyre.terrain.f32.empty() ? nullptr : resolved_tyre.terrain.f32.data();
    const bike::physics_core::i32 *terrain_i32_ptr =
        resolved_tyre.terrain.i32.empty() ? nullptr : resolved_tyre.terrain.i32.data();

    std::atomic<int> next_world{0};
    std::atomic<int> all_finite_flag{1};
    std::atomic<int> writer_failed_flag{0};

    const auto t_start = std::chrono::steady_clock::now();

    const auto worker_fn = [&](int tid) {
        mjModel *m_t = worker_models[to_index(tid)];
        mjData *d_t = worker_data[to_index(tid)];
        const std::span<mjtNum> d_qpos = boundary_span(d_t->qpos, to_index(nq));
        const std::span<mjtNum> d_qvel = boundary_span(d_t->qvel, to_index(nv));
        const std::span<mjtNum> d_xpos = boundary_span(d_t->xpos, to_index(nbody3));
        const std::span<mjtNum> d_xmat = boundary_span(d_t->xmat, to_index(nbody9));
        const std::span<mjtNum> d_xanchor = boundary_span(d_t->xanchor, to_index(njnt3));
        const std::span<mjtNum> d_xaxis = boundary_span(d_t->xaxis, to_index(njnt3));
        const std::span<mjtNum> d_qfrc_applied = boundary_span(d_t->qfrc_applied, to_index(nv));
        const std::span<mjtNum> d_ctrl = boundary_span(d_t->ctrl, to_index(nu));
        const std::span<mjtNum> t_tendon_range =
            boundary_span(m_t->tendon_range, 2 * to_index(ntendon));
        const std::span<const mjtNum> base_tendon_range =
            boundary_span(m_base->tendon_range, 2 * to_index(ntendon));

        std::unique_ptr<bike::physics_core::GpuPneumaticStepScratch<float>> pneum_scratch;
        if (resolved_tyre.model.backend == bike::physics_core::TyreBackendKind::pneumatic) {
            pneum_scratch =
                std::make_unique<bike::physics_core::GpuPneumaticStepScratch<float>>();
        }

        while (true) {
            const int w = next_world.fetch_add(1, std::memory_order_relaxed);
            if (w >= dims.nworld) {
                break;
            }

            const bike::gpu::GpuWorldConfig &cfg_w =
                configs_v[(per_world_config != 0) ? to_index(w) : 0];

            engine::reset_data(m_t, d_t);
            for (std::size_t i = 0; i < to_index(nq); ++i) {
                d_qpos[i] = static_cast<mjtNum>(qpos0[i]);
            }
            for (std::size_t i = 0; i < to_index(nv); ++i) {
                d_qvel[i] = static_cast<mjtNum>(qvel0[i]);
            }
            alignas(16) std::array<float, 4> tendon_f32{0.0f, 0.0f, 0.0f, 0.0f};
            if (ntendon > 0 && idx.freehub_tendon >= 0 && idx.freehub_tendon < ntendon) {
                const std::size_t ft = 2 * to_index(idx.freehub_tendon);
                t_tendon_range[ft + 0] = base_tendon_range[ft + 0];
                t_tendon_range[ft + 1] = base_tendon_range[ft + 1];
                tendon_f32[0] = static_cast<float>(base_tendon_range[ft + 0]);
                tendon_f32[1] = static_cast<float>(base_tendon_range[ft + 1]);
            }
            const bool has_ds =
                (idx.drive_shaft_spin_qpos >= 0 && idx.drive_shaft_spin_qpos < nq);
            const bool has_cc =
                has_ds && ntendon > 0 && idx.crank_clutch_tendon >= 0 && idx.crank_clutch_tendon < ntendon;
            if (has_cc) {
                const std::size_t ct_idx = 2 * to_index(idx.crank_clutch_tendon);
                t_tendon_range[ct_idx + 0] = base_tendon_range[ct_idx + 0];
                t_tendon_range[ct_idx + 1] = base_tendon_range[ct_idx + 1];
                tendon_f32[2] = static_cast<float>(base_tendon_range[ct_idx + 0]);
                tendon_f32[3] = static_cast<float>(base_tendon_range[ct_idx + 1]);
            }
            engine::forward(m_t, d_t);

            const auto st_w = std::make_unique<bike::gpu::GpuWorldState>();
            const float crank0 =
                (idx.crank_spin_qpos >= 0 && idx.crank_spin_qpos < nq)
                    ? static_cast<float>(d_qpos[to_index(idx.crank_spin_qpos)])
                    : 0.0f;
            const float ds0 =
                has_ds ? static_cast<float>(d_qpos[to_index(idx.drive_shaft_spin_qpos)]) : 0.0f;
            const float rwheel0 =
                (idx.rear_wheel_spin_qpos >= 0 && idx.rear_wheel_spin_qpos < nq)
                    ? static_cast<float>(d_qpos[to_index(idx.rear_wheel_spin_qpos)])
                    : 0.0f;
            bike::gpu::gpu_init_world_state(
                *st_w, cfg_w.drivetrain, crank0, rwheel0, ds0, has_ds);

            bike::gpu::GpuModelIndices local_idx = idx;
            if (ntendon > 0 && local_idx.freehub_tendon >= 0 && local_idx.freehub_tendon < ntendon) {
                local_idx.freehub_tendon = 0;
            } else {
                local_idx.freehub_tendon = -1;
            }
            local_idx.crank_clutch_tendon = has_cc ? 1 : -1;

            float max_fork_m = 0.0f;
            float max_shock_m = 0.0f;

            const auto step_world = [&]() {
                alignas(64) std::array<float, kMaxNq> qpos_f32{};
                alignas(64) std::array<float, kMaxNv> qvel_f32{};
                alignas(64) std::array<float, 3 * to_index(kMaxNbody)> xpos_f32{};
                alignas(64) std::array<float, 9 * to_index(kMaxNbody)> xmat_f32{};
                alignas(64) std::array<float, 3 * to_index(kMaxNjnt)> xanchor_f32{};
                alignas(64) std::array<float, 3 * to_index(kMaxNjnt)> xaxis_f32{};
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
                for (std::size_t i = 0; i < to_index(nbody9); ++i) {
                    xmat_f32[i] = static_cast<float>(d_xmat[i]);
                }
                for (std::size_t i = 0; i < to_index(njnt3); ++i) {
                    xanchor_f32[i] = static_cast<float>(d_xanchor[i]);
                    xaxis_f32[i] = static_cast<float>(d_xaxis[i]);
                }

                const bike::physics_core::Status st = bike::gpu::gpu_step_world_writers(
                    0,
                    dt_s,
                    local_idx,
                    &resolved_tyre.model,
                    resolved_tyre.terrain.desc,
                    terrain_f32_ptr,
                    terrain_i32_ptr,
                    &cfg_w,
                    0,
                    st_w.get(),
                    qpos_f32.data(),
                    qvel_f32.data(),
                    xpos_f32.data(),
                    xmat_f32.data(),
                    xanchor_f32.data(),
                    xaxis_f32.data(),
                    qfrc_f32.data(),
                    ctrl_f32.data(),
                    (local_idx.freehub_tendon >= 0 || local_idx.crank_clutch_tendon >= 0) ? tendon_f32.data() : nullptr,
                    4,
                    false,
                    pneum_scratch.get()
                );
                if (st != bike::physics_core::Status::ok) {
                    writer_failed_flag.store(1, std::memory_order_relaxed);
                    return;
                }

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
                if (has_cc) {
                    const std::size_t ct_idx = 2 * to_index(idx.crank_clutch_tendon);
                    t_tendon_range[ct_idx + 0] = static_cast<mjtNum>(tendon_f32[2]);
                    t_tendon_range[ct_idx + 1] = static_cast<mjtNum>(tendon_f32[3]);
                }

                engine::step(m_t, d_t);
                engine::kinematics(m_t, d_t);

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
                if (writer_failed_flag.load(std::memory_order_relaxed) != 0) {
                    break;
                }
            }

            bool world_finite = (writer_failed_flag.load(std::memory_order_relaxed) == 0);
            if (world_finite) {
                for (std::size_t i = 0; i < to_index(nq); ++i) {
                    if (!std::isfinite(static_cast<double>(d_qpos[i]))) {
                        world_finite = false;
                        break;
                    }
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
    return writer_failed_flag.load(std::memory_order_relaxed) != 0 ? -7 : 0;
}

int bike_cpu_run_batch(
    const char *xml_path,
    int nworld,
    int n_steps,
    int num_threads,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
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
        d_tyre_model,
        d_terrain_desc,
        d_terrain_f32,
        d_terrain_i32,
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
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
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
        d_tyre_model,
        d_terrain_desc,
        d_terrain_f32,
        d_terrain_i32,
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

int bike_cpu_build_tyre_and_terrain(
    const char *xml_path,
    int backend_kind,
    int station_count,
    const bike::gpu::GpuModelIndices *d_idx,
    bike::physics_core::GpuTyreModel *out_tyre_model,
    bike::physics_core::GpuTerrainTableDesc *out_terrain_desc,
    float *out_terrain_f32,
    int *io_f32_count,
    bike::physics_core::i32 *out_terrain_i32,
    int *io_i32_count
) {
    return bike::gpu::build_runner_tyre_and_terrain_buffers(
        xml_path, backend_kind, station_count, d_idx,
        out_tyre_model, out_terrain_desc,
        out_terrain_f32, io_f32_count,
        out_terrain_i32, io_i32_count
    );
}

int bike_cpu_build_tyre_and_terrain_for_surface(
    const char *xml_path,
    int backend_kind,
    int station_count,
    int surface_label,
    const bike::gpu::GpuModelIndices *d_idx,
    bike::physics_core::GpuTyreModel *out_tyre_model,
    bike::physics_core::GpuTerrainTableDesc *out_terrain_desc,
    float *out_terrain_f32,
    int *io_f32_count,
    bike::physics_core::i32 *out_terrain_i32,
    int *io_i32_count
) {
    return bike::gpu::build_runner_tyre_and_terrain_buffers_for_surface(
        xml_path, backend_kind, station_count, surface_label, d_idx,
        out_tyre_model, out_terrain_desc,
        out_terrain_f32, io_f32_count,
        out_terrain_i32, io_i32_count
    );
}

int bike_cpu_build_spindle_params(
    const char *xml_path,
    const char *strength_json_path,
    const char *envelope_json_path,
    int control_period_steps,
    bike::physics_core::SpindleM2K2Params<float> *out_params
) {
    return bike::gpu::build_runner_spindle_params(
        xml_path, strength_json_path, envelope_json_path, control_period_steps, out_params);
}

} // extern "C"
