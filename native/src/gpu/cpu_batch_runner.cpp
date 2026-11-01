// native/src/gpu/cpu_batch_runner.cpp — Multi-threaded C++ CPU batch runner
// executing `gpu_step_world_writers` + `mj_step` across all CPU cores with
// zero per-step heap allocations and zero Python/GIL overhead.

#include "cuda_kernels.cu"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>
#include <vector>

extern "C" {

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
    if (xml_path == nullptr || nworld <= 0 || n_steps <= 0 || d_configs == nullptr) {
        return -1;
    }

    char err[1024] = {0};
    mjModel *m_base = mj_loadXML(xml_path, nullptr, err, sizeof(err));
    if (m_base == nullptr) {
        return -2;
    }

    const int hw_threads = static_cast<int>(std::thread::hardware_concurrency());
    int n_workers = (num_threads > 0) ? num_threads : (hw_threads > 0 ? hw_threads : 4);
    n_workers = std::max(1, std::min(n_workers, nworld));

    std::vector<mjModel *> worker_models(static_cast<std::size_t>(n_workers), nullptr);
    std::vector<mjData *> worker_data(static_cast<std::size_t>(n_workers), nullptr);
    for (int t = 0; t < n_workers; ++t) {
        worker_models[static_cast<std::size_t>(t)] = mj_copyModel(nullptr, m_base);
        worker_data[static_cast<std::size_t>(t)] =
            mj_makeData(worker_models[static_cast<std::size_t>(t)]);
    }

    const bike::gpu::GpuModelIndices idx =
        (d_idx != nullptr) ? *d_idx : bike::gpu::GpuModelIndices{};
    const float hf_inv_dx = (hf_dx > 0.0f) ? (1.0f / hf_dx) : 1.0f;
    const bike::gpu::GpuHeightfield1D hf{
        d_hf_heights,
        hf_num_samples,
        hf_x_min,
        hf_dx,
        hf_inv_dx
    };
    const float dt_s = static_cast<float>(m_base->opt.timestep);

    std::atomic<int> next_world{0};
    std::atomic<int> all_finite_flag{1};

    const auto t_start = std::chrono::steady_clock::now();

    auto worker_fn = [&](int tid) {
        mjModel *m_t = worker_models[static_cast<std::size_t>(tid)];
        mjData *d_t = worker_data[static_cast<std::size_t>(tid)];
        const int nq = m_t->nq;
        const int nv = m_t->nv;
        const int nu = m_t->nu;
        const int nbody3 = m_t->nbody * 3;

        alignas(64) float qpos_f32[64] = {0.0f};
        alignas(64) float qvel_f32[64] = {0.0f};
        alignas(64) float xpos_f32[128] = {0.0f};
        alignas(64) float qfrc_f32[64] = {0.0f};
        alignas(64) float ctrl_f32[32] = {0.0f};
        alignas(16) float tendon_f32[2] = {0.0f, 0.0f};

        while (true) {
            const int w = next_world.fetch_add(1, std::memory_order_relaxed);
            if (w >= nworld) {
                break;
            }

            const bike::gpu::GpuWorldConfig &cfg_w = d_configs[per_world_config ? w : 0];

            mj_resetData(m_t, d_t);
            if (init_qpos != nullptr) {
                for (int i = 0; i < nq; ++i) {
                    d_t->qpos[i] = static_cast<mjtNum>(init_qpos[i]);
                }
            }
            if (init_qvel != nullptr) {
                for (int i = 0; i < nv; ++i) {
                    d_t->qvel[i] = static_cast<mjtNum>(init_qvel[i]);
                }
            }
            if (m_t->ntendon > 0) {
                m_t->tendon_range[0] = m_base->tendon_range[0];
                m_t->tendon_range[1] = m_base->tendon_range[1];
                tendon_f32[0] = static_cast<float>(m_base->tendon_range[0]);
                tendon_f32[1] = static_cast<float>(m_base->tendon_range[1]);
            }
            mj_forward(m_t, d_t);

            bike::gpu::GpuWorldState st_w{};
            const float crank0 =
                (idx.crank_spin_qpos >= 0 && idx.crank_spin_qpos < nq)
                    ? static_cast<float>(d_t->qpos[idx.crank_spin_qpos])
                    : 0.0f;
            const float rwheel0 =
                (idx.rear_wheel_spin_qpos >= 0 && idx.rear_wheel_spin_qpos < nq)
                    ? static_cast<float>(d_t->qpos[idx.rear_wheel_spin_qpos])
                    : 0.0f;
            bike::gpu::gpu_init_world_state(st_w, cfg_w.drivetrain, crank0, rwheel0);

            float peak_fork_m = 0.0f;
            float peak_shock_m = 0.0f;

            for (int step = 0; step < n_steps; ++step) {
                for (int i = 0; i < nq; ++i) {
                    qpos_f32[i] = static_cast<float>(d_t->qpos[i]);
                }
                for (int i = 0; i < nv; ++i) {
                    qvel_f32[i] = static_cast<float>(d_t->qvel[i]);
                }
                for (int i = 0; i < nbody3; ++i) {
                    xpos_f32[i] = static_cast<float>(d_t->xpos[i]);
                }

                bike::gpu::gpu_step_world_writers(
                    0,
                    dt_s,
                    idx,
                    hf,
                    &cfg_w,
                    0,
                    &st_w,
                    qpos_f32,
                    qvel_f32,
                    xpos_f32,
                    qfrc_f32,
                    ctrl_f32,
                    tendon_f32,
                    2
                );

                for (int i = 0; i < nv; ++i) {
                    d_t->qfrc_applied[i] = static_cast<mjtNum>(qfrc_f32[i]);
                }
                for (int i = 0; i < nu; ++i) {
                    d_t->ctrl[i] = static_cast<mjtNum>(ctrl_f32[i]);
                }

                mj_step(m_t, d_t);

                if (step >= 24) {
                    if (idx.fork_travel_qpos >= 0 && idx.fork_travel_qpos < nq) {
                        const float fk = static_cast<float>(d_t->qpos[idx.fork_travel_qpos]);
                        if (fk > peak_fork_m) {
                            peak_fork_m = fk;
                        }
                    }
                    if (idx.shock_stroke_qpos >= 0 && idx.shock_stroke_qpos < nq) {
                        const float sh = static_cast<float>(d_t->qpos[idx.shock_stroke_qpos]);
                        if (sh > peak_shock_m) {
                            peak_shock_m = sh;
                        }
                    }
                }
            }

            bool world_finite = true;
            for (int i = 0; i < nq; ++i) {
                if (!std::isfinite(static_cast<double>(d_t->qpos[i]))) {
                    world_finite = false;
                    break;
                }
            }
            if (world_finite) {
                for (int i = 0; i < nv; ++i) {
                    if (!std::isfinite(static_cast<double>(d_t->qvel[i]))) {
                        world_finite = false;
                        break;
                    }
                }
            }
            if (!world_finite) {
                all_finite_flag.store(0, std::memory_order_relaxed);
            }

            if (out_peak_fork_mm != nullptr) {
                out_peak_fork_mm[w] = peak_fork_m * 1000.0f;
            }
            if (out_peak_shock_mm != nullptr) {
                out_peak_shock_mm[w] = peak_shock_m * 1000.0f;
            }
            if (out_final_root_x_m != nullptr) {
                out_final_root_x_m[w] =
                    (idx.root_x_qpos >= 0 && idx.root_x_qpos < nq)
                        ? static_cast<float>(d_t->qpos[idx.root_x_qpos])
                        : 0.0f;
            }
        }
    };

    std::vector<std::thread> threads;
    threads.reserve(static_cast<std::size_t>(n_workers));
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
        mj_deleteData(worker_data[static_cast<std::size_t>(t)]);
        mj_deleteModel(worker_models[static_cast<std::size_t>(t)]);
    }
    mj_deleteModel(m_base);
    return 0;
}

} // extern "C"
