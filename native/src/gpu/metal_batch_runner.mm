// native/src/gpu/metal_batch_runner.mm — Apple Metal Unified-Memory GPU Force
// Writers & Hybrid (Metal GPU Writers FP32 + Multi-Threaded C++ CPU MuJoCo FP64)
// Batch Runner for Apple Silicon (M4 Max).

#if !defined(BIKE_METAL_OMIT_CUDA_KERNELS_TU)
#include "cuda_kernels.cu"
#else
#include "batch_step.hpp"
#include "sp3_autonomous_step.hpp"
#include "sp4_closed_loop_step.hpp"
#endif

#include <mujoco/mujoco.h>
#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include "model_assets.hpp"
#include "runner_spindle_setup.hpp"
#include "runner_tyre_setup.hpp"
#if defined(BIKE_GPU_STANDALONE_RUNNER)
#include "standalone_mujoco_shim.hpp"
#else
#include "../engine_call.hpp"
#endif
namespace assets = bike::gpu::assets;

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <mutex>
#include <span>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

// The single raw pointer -> std::span boundary of this TU. The extern "C" batch
// and launch entry points receive (pointer, count) ranges whose extents the caller
// owns (d_configs/d_terrain_f32/d_terrain_i32/init_qpos/init_qvel/d_states/d_qpos/
// d_qvel/d_xpos/d_xmat/d_xanchor/d_xaxis/d_qfrc_applied/d_ctrl/d_tendon_range/out_*
// sized by nworld, the idx dims or the caller's terrain table counts);
// [MTLBuffer contents] exposes each shared Metal buffer as a raw void* over the
// byte length the buffer was allocated with; MuJoCo likewise exposes mjModel/mjData
// arrays as raw pointers sized by the adjacent model counts. Every such range is
// wrapped once through this helper; the (pointer, count) construction itself is
// -Wunsafe-buffer-usage-in-container — suppressed here, scoped like
// native/src/gpu/cpu_batch_runner.cpp's boundary_span.
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

std::string read_file_str(const std::string &path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "Metal shader loader: failed to open header " << path << '\n';
        return "";
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::string strip_includes_and_pragmas(const std::string &src) {
    std::istringstream iss(src);
    std::ostringstream oss;
    std::string line;
    while (std::getline(iss, line)) {
        const std::size_t first = line.find_first_not_of(" \t\r");
        if (first != std::string::npos) {
            if (line.compare(first, 12, "#pragma once") == 0 ||
                line.compare(first, 8, "#include") == 0) {
                continue;
            }
        }
        oss << line << '\n';
    }
    return oss.str();
}

std::string build_msl_shader_source(const std::string &gpu_src_dir) {
    const std::string base = gpu_src_dir.empty() ? "." : gpu_src_dir;
    const std::vector<std::string> headers = {
        "../physics_core/common.hpp",
        "../physics_core/gpu_math.hpp",
        "../physics_core/gpu_toolkit.hpp",
        "../physics_core/tyre_types.hpp",
        "../physics_core/kinematics.hpp",
        "../physics_core/terrain_scanner.hpp",
        "../physics_core/terrain_distance.hpp",
        "../physics_core/tyre_contact.hpp",
        "../physics_core/tyre_tabulated.hpp",
        "../physics_core/tyre_compliant.hpp",
        "../physics_core/tyre_distributed.hpp",
        "../physics_core/tyre_pneumatic_geometry.hpp",
        "../physics_core/tyre_pneumatic_carcass.hpp",
        "../physics_core/tyre_pneumatic_brush.hpp",
        "../physics_core/tyre_pneumatic_model.hpp",
        "../physics_core/tyre_gpu_step.hpp",
        "../physics_core/suspension.hpp",
        "../physics_core/resistance.hpp",
        "../physics_core/brake_hold.hpp",
        "../physics_core/joint_envelope.hpp",
        "../physics_core/ratchet.hpp",
        "../physics_core/launch_plan.hpp",
        "../physics_core/front_lift.hpp",
        "../physics_core/rider_intent.hpp",
        "../physics_core/efc_monitors.hpp",
        "../physics_core/shifting.hpp",
        "../physics_core/pedaling.hpp",
        "../physics_core/assist.hpp",
        "../physics_core/cruise.hpp",
        "../physics_core/ride_program.hpp",
        "../physics_core/energy_ledger.hpp",
        "../physics_core/spindle_m2k2.hpp",
        "../physics_core/spindle_m0.hpp",
        "common.hpp",
        "batch_step.hpp",
        "sp3_autonomous_step.hpp",
        "sp4_closed_loop_step.hpp",
    };

    std::string combined = R"MSL(
#pragma METAL fp contract(off)
#pragma clang fp contract(off) reassociate(off)
#pragma clang diagnostic ignored "-Wc++17-extensions"
#include <metal_stdlib>
using namespace metal;
)MSL";

    for (const auto &fn : headers) {
        const std::string raw = read_file_str(base + "/" + fn);
        if (raw.empty()) {
            return "";
        }
        combined += "\n// === " + fn + " ===\n" + strip_includes_and_pragmas(raw);
    }

    combined += R"MSL(
inline void bike_metal_init_states_impl(
    int w,
    thread const bike::gpu::GpuModelIndices &idx,
    device const bike::gpu::GpuWorldConfig *configs,
    int per_world_config,
    device bike::gpu::GpuWorldState *states,
    device const float *qpos_batch
) {
    if (!bike::gpu::gpu_validate_model_indices(idx)) {
        return;
    }
    device const bike::gpu::GpuWorldConfig &cfg = configs[per_world_config ? w : 0];
    const bike::gpu::GpuDrivetrainParams dp = cfg.drivetrain;
    device const float *qpos_w = (qpos_batch != nullptr) ? (qpos_batch + static_cast<long>(w) * idx.nq) : nullptr;
    const bool has_drive_shaft =
        (idx.drive_shaft_spin_qpos >= 0 && idx.drive_shaft_spin_qpos < idx.nq);
    const float crank_qpos =
        (qpos_w != nullptr && idx.crank_spin_qpos >= 0 && idx.crank_spin_qpos < idx.nq)
            ? qpos_w[idx.crank_spin_qpos]
            : 0.0f;
    const float drive_shaft_qpos =
        (qpos_w != nullptr && has_drive_shaft)
            ? qpos_w[idx.drive_shaft_spin_qpos]
            : 0.0f;
    const float rear_wheel_qpos =
        (qpos_w != nullptr && idx.rear_wheel_spin_qpos >= 0 && idx.rear_wheel_spin_qpos < idx.nq)
            ? qpos_w[idx.rear_wheel_spin_qpos]
            : 0.0f;
    bike::gpu::GpuWorldState local_st{};
    bike::gpu::gpu_init_world_state(
        local_st, dp, crank_qpos, rear_wheel_qpos, drive_shaft_qpos, has_drive_shaft);
    states[w] = local_st;
}

inline void bike_metal_step_writers_world(
    int w,
    float dt_s,
    thread const bike::gpu::GpuModelIndices &idx,
    device const bike::physics_core::GpuTyreModel *tyre_model,
    thread const bike::physics_core::GpuTerrainTableDesc &terrain_desc,
    device const float *terrain_f32,
    device const bike::physics_core::i32 *terrain_i32,
    device const bike::gpu::GpuWorldConfig *configs,
    int per_world_config,
    device bike::gpu::GpuWorldState *states,
    device const float *qpos_batch,
    device const float *qvel_batch,
    device const float *xpos_batch,
    device const float *xmat_batch,
    device const float *xanchor_batch,
    device const float *xaxis_batch,
    device float *qfrc_batch,
    device float *ctrl_batch,
    device float *tendon_range_batch,
    int tendon_range_world_stride,
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch
) {
    if (!bike::gpu::gpu_validate_model_indices(idx) || tyre_model == nullptr) {
        return;
    }
    const bike::gpu::GpuWorldConfig cfg = configs[per_world_config ? w : 0];
    bike::gpu::GpuDrivetrainState dt_st = states[w].drivetrain;

    float qpos_w[bike::gpu::kMaxNq];
    float qvel_w[bike::gpu::kMaxNv];
    float qfrc_w[bike::gpu::kMaxNv];
    float ctrl_w[bike::gpu::kMaxNu];
    float tendon_w[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    const int nq = idx.nq;
    const int nv = idx.nv;
    const int nu = idx.nu;
    const long w64 = static_cast<long>(w);

    device const float *qpos_src = qpos_batch + w64 * nq;
    device const float *qvel_src = qvel_batch + w64 * nv;
    device const float *xpos_src = xpos_batch + w64 * idx.nbody * 3;
    device const float *xmat_src = xmat_batch + w64 * idx.nbody * 9;
    device const float *xanchor_src = xanchor_batch + w64 * tyre_model->njnt * 3;
    device const float *xaxis_src = xaxis_batch + w64 * tyre_model->njnt * 3;

    for (int i = 0; i < nq; ++i) qpos_w[i] = qpos_src[i];
    for (int i = 0; i < nv; ++i) qvel_w[i] = qvel_src[i];

    bike::gpu::GpuModelIndices local_idx = idx;
    if (local_idx.freehub_tendon >= 0) {
        local_idx.freehub_tendon = 0;
    }
    if (local_idx.crank_clutch_tendon >= 0 && local_idx.drive_shaft_spin_qpos >= 0) {
        local_idx.crank_clutch_tendon = 1;
    } else {
        local_idx.crank_clutch_tendon = -1;
    }

    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_w =
        (pneum_scratch != nullptr)
            ? &pneum_scratch[w]
            : static_cast<device bike::physics_core::GpuPneumaticStepScratch<float> *>(nullptr);

    const bike::physics_core::Status st = bike::gpu::gpu_step_world_writers_single(
        0,
        dt_s,
        local_idx,
        tyre_model,
        terrain_desc,
        terrain_f32,
        terrain_i32,
        cfg,
        dt_st,
        states[w].compliant_wheels,
        &states[w].distributed_wheels[0][0],
        qpos_w,
        qvel_w,
        xpos_src,
        xmat_src,
        xanchor_src,
        xaxis_src,
        qfrc_w,
        ctrl_w,
        (tendon_range_batch != nullptr && bike::gpu::gpu_max_active_tendon_idx(idx) >= 0)
            ? tendon_w
            : nullptr,
        4,
        false,
        states[w].pneumatic_wheels,
        pneum_w
    );
    if (st != bike::physics_core::Status::ok) {
        return;
    }

    states[w].drivetrain = dt_st;
    device float *qfrc_dst = qfrc_batch + w64 * nv;
    device float *ctrl_dst = ctrl_batch + w64 * nu;
    for (int i = 0; i < nv; ++i) qfrc_dst[i] = qfrc_w[i];
    for (int i = 0; i < nu; ++i) ctrl_dst[i] = ctrl_w[i];

    if (tendon_range_batch != nullptr) {
        if (idx.freehub_tendon >= 0) {
            bike::gpu::gpu_write_tendon_upper_bound(
                w, idx.freehub_tendon, tendon_w[1], tendon_range_batch, tendon_range_world_stride);
        }
        if (idx.drive_shaft_spin_qpos >= 0 && idx.crank_clutch_tendon >= 0 &&
            idx.crank_clutch_tendon != idx.freehub_tendon) {
            bike::gpu::gpu_write_tendon_upper_bound(
                w, idx.crank_clutch_tendon, tendon_w[3], tendon_range_batch, tendon_range_world_stride);
        }
    }
}

inline void bike_metal_step_writers_impl(
    constant int &world_offset,
    constant int &world_count,
    constant float &dt_s,
    constant bike::gpu::GpuModelIndices &idx_const,
    device const bike::physics_core::GpuTyreModel *tyre_model,
    constant bike::physics_core::GpuTerrainTableDesc &terrain_desc_const,
    device const float *terrain_f32,
    device const bike::physics_core::i32 *terrain_i32,
    device const bike::gpu::GpuWorldConfig *configs,
    constant int &per_world_config,
    device bike::gpu::GpuWorldState *states,
    device const float *qpos_batch,
    device const float *qvel_batch,
    device const float *xpos_batch,
    device const float *xmat_batch,
    device const float *xanchor_batch,
    device const float *xaxis_batch,
    device float *qfrc_batch,
    device float *ctrl_batch,
    device float *tendon_range_batch,
    constant int &tendon_range_world_stride,
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch,
    uint gid
) {
    const int local_w = static_cast<int>(gid);
    if (local_w >= world_count) {
        return;
    }
    const int w = world_offset + local_w;
    const bike::gpu::GpuModelIndices idx = idx_const;
    const bike::physics_core::GpuTerrainTableDesc terrain_desc = terrain_desc_const;
    bike_metal_step_writers_world(
        w, dt_s, idx, tyre_model, terrain_desc, terrain_f32, terrain_i32,
        configs, per_world_config, states, qpos_batch, qvel_batch,
        xpos_batch, xmat_batch, xanchor_batch, xaxis_batch,
        qfrc_batch, ctrl_batch, tendon_range_batch, tendon_range_world_stride,
        pneum_scratch
    );
}

struct BikeWpInitArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::gpu::GpuWorldConfig *configs;
    device bike::gpu::GpuWorldState *states;
    device const float *qpos;
    int nworld;
    int per_world_config;
};

struct BikeWpWritersArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::physics_core::GpuTyreModel *tyre_model;
    device const bike::physics_core::GpuTerrainTableDesc *terrain_desc;
    device const float *terrain_f32;
    device const bike::physics_core::i32 *terrain_i32;
    device const bike::gpu::GpuWorldConfig *configs;
    device bike::gpu::GpuWorldState *states;
    device const float *qpos;
    device const float *qvel;
    device const float *xpos;
    device const float *xmat;
    device const float *xanchor;
    device const float *xaxis;
    device float *qfrc_applied;
    device float *ctrl;
    device float *tendon_range;
    int nworld;
    float dt_s;
    int per_world_config;
    int tendon_range_world_stride;
};

struct BikeWpWritersPneumArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::physics_core::GpuTyreModel *tyre_model;
    device const bike::physics_core::GpuTerrainTableDesc *terrain_desc;
    device const float *terrain_f32;
    device const bike::physics_core::i32 *terrain_i32;
    device const bike::gpu::GpuWorldConfig *configs;
    device bike::gpu::GpuWorldState *states;
    device const float *qpos;
    device const float *qvel;
    device const float *xpos;
    device const float *xmat;
    device const float *xanchor;
    device const float *xaxis;
    device float *qfrc_applied;
    device float *ctrl;
    device float *tendon_range;
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch;
    int nworld;
    float dt_s;
    int per_world_config;
    int tendon_range_world_stride;
};

kernel void bike_wp_init_states_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpInitArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    bike_metal_init_states_impl(w, idx, args->configs, args->per_world_config, args->states, args->qpos);
}

inline void bike_metal_step_writers_fused_impl(
    constant ulong *bounds_buf,
    constant BikeWpWritersArgs *args,
    uint gid
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    if (args->terrain_desc == nullptr) {
        return;
    }
    const bike::physics_core::GpuTerrainTableDesc terrain_desc = *(args->terrain_desc);
    bike_metal_step_writers_world(
        w,
        args->dt_s,
        idx,
        args->tyre_model,
        terrain_desc,
        args->terrain_f32,
        args->terrain_i32,
        args->configs,
        args->per_world_config,
        args->states,
        args->qpos,
        args->qvel,
        args->xpos,
        args->xmat,
        args->xanchor,
        args->xaxis,
        args->qfrc_applied,
        args->ctrl,
        args->tendon_range,
        args->tendon_range_world_stride,
        static_cast<device bike::physics_core::GpuPneumaticStepScratch<float> *>(nullptr)
    );
}

kernel void bike_wp_writers_pneum_metal_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpWritersPneumArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    if (args->terrain_desc == nullptr) {
        return;
    }
    const bike::physics_core::GpuTerrainTableDesc terrain_desc = *(args->terrain_desc);
    bike_metal_step_writers_world(
        w,
        args->dt_s,
        idx,
        args->tyre_model,
        terrain_desc,
        args->terrain_f32,
        args->terrain_i32,
        args->configs,
        args->per_world_config,
        args->states,
        args->qpos,
        args->qvel,
        args->xpos,
        args->xmat,
        args->xanchor,
        args->xaxis,
        args->qfrc_applied,
        args->ctrl,
        args->tendon_range,
        args->tendon_range_world_stride,
        args->pneum_scratch
    );
}

kernel void bike_gpu_init_states_metal_kernel(
    constant int &nworld [[buffer(0)]],
    constant bike::gpu::GpuModelIndices &idx_const [[buffer(1)]],
    device const bike::gpu::GpuWorldConfig *configs [[buffer(2)]],
    constant int &per_world_config [[buffer(3)]],
    device bike::gpu::GpuWorldState *states [[buffer(4)]],
    device const float *qpos_batch [[buffer(5)]],
    uint gid [[thread_position_in_grid]]
) {
    const int w = static_cast<int>(gid);
    if (w >= nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx = idx_const;
    bike_metal_init_states_impl(w, idx, configs, per_world_config, states, qpos_batch);
}

kernel void bike_gpu_writers_metal_kernel(
    constant int &world_offset [[buffer(0)]],
    constant int &world_count [[buffer(1)]],
    constant float &dt_s [[buffer(2)]],
    constant bike::gpu::GpuModelIndices &idx_const [[buffer(3)]],
    device const bike::physics_core::GpuTyreModel *tyre_model [[buffer(4)]],
    constant bike::physics_core::GpuTerrainTableDesc &terrain_desc_const [[buffer(5)]],
    device const float *terrain_f32 [[buffer(6)]],
    device const bike::physics_core::i32 *terrain_i32 [[buffer(7)]],
    device const bike::gpu::GpuWorldConfig *configs [[buffer(8)]],
    constant int &per_world_config [[buffer(9)]],
    device bike::gpu::GpuWorldState *states [[buffer(10)]],
    device const float *qpos_batch [[buffer(11)]],
    device const float *qvel_batch [[buffer(12)]],
    device const float *xpos_batch [[buffer(13)]],
    device const float *xmat_batch [[buffer(14)]],
    device const float *xanchor_batch [[buffer(15)]],
    device const float *xaxis_batch [[buffer(16)]],
    device float *qfrc_batch [[buffer(17)]],
    device float *ctrl_batch [[buffer(18)]],
    device float *tendon_range_batch [[buffer(19)]],
    constant int &tendon_range_world_stride [[buffer(20)]],
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch [[buffer(21)]],
    uint gid [[thread_position_in_grid]]
) {
    bike_metal_step_writers_impl(
        world_offset, world_count, dt_s, idx_const, tyre_model, terrain_desc_const,
        terrain_f32, terrain_i32, configs, per_world_config, states,
        qpos_batch, qvel_batch, xpos_batch, xmat_batch, xanchor_batch, xaxis_batch,
        qfrc_batch, ctrl_batch, tendon_range_batch, tendon_range_world_stride,
        pneum_scratch, gid
    );
}

kernel void bike_wp_writers_metal_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpWritersArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    bike_metal_step_writers_fused_impl(bounds_buf, args, gid);
}

kernel void bike_metal_step_writers_kernel(
    constant int &world_offset [[buffer(0)]],
    constant int &world_count [[buffer(1)]],
    constant float &dt_s [[buffer(2)]],
    constant bike::gpu::GpuModelIndices &idx_const [[buffer(3)]],
    device const bike::physics_core::GpuTyreModel *tyre_model [[buffer(4)]],
    constant bike::physics_core::GpuTerrainTableDesc &terrain_desc_const [[buffer(5)]],
    device const float *terrain_f32 [[buffer(6)]],
    device const bike::physics_core::i32 *terrain_i32 [[buffer(7)]],
    device const bike::gpu::GpuWorldConfig *configs [[buffer(8)]],
    constant int &per_world_config [[buffer(9)]],
    device bike::gpu::GpuWorldState *states [[buffer(10)]],
    device const float *qpos_batch [[buffer(11)]],
    device const float *qvel_batch [[buffer(12)]],
    device const float *xpos_batch [[buffer(13)]],
    device const float *xmat_batch [[buffer(14)]],
    device const float *xanchor_batch [[buffer(15)]],
    device const float *xaxis_batch [[buffer(16)]],
    device float *qfrc_batch [[buffer(17)]],
    device float *ctrl_batch [[buffer(18)]],
    device float *tendon_range_batch [[buffer(19)]],
    constant int &tendon_range_world_stride [[buffer(20)]],
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch [[buffer(21)]],
    uint gid [[thread_position_in_grid]]
) {
    bike_metal_step_writers_impl(
        world_offset, world_count, dt_s, idx_const, tyre_model, terrain_desc_const,
        terrain_f32, terrain_i32, configs, per_world_config, states,
        qpos_batch, qvel_batch, xpos_batch, xmat_batch, xanchor_batch, xaxis_batch,
        qfrc_batch, ctrl_batch, tendon_range_batch, tendon_range_world_stride,
        pneum_scratch, gid
    );
}

kernel void bike_metal_step_writers_fused_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpWritersArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    bike_metal_step_writers_fused_impl(bounds_buf, args, gid);
}

inline void bike_metal_sp3_init_impl(
    int w,
    thread const bike::gpu::GpuModelIndices &idx,
    device const bike::gpu::GpuSp3Config *configs,
    int per_world_config,
    device bike::gpu::GpuSp3WorldState *sp3_states,
    int has_world_states,
    device bike::gpu::GpuWorldState *world_states,
    int has_qpos,
    device const float *qpos_batch,
    float init_grade,
    int ntendon,
    int has_tendon_range,
    device float *tendon_range_batch,
    int tendon_range_world_stride
) {
    if (!bike::gpu::gpu_validate_model_indices(idx, ntendon) ||
        configs == nullptr || sp3_states == nullptr) {
        return;
    }
    const long w64 = static_cast<long>(w);
    device const bike::gpu::GpuSp3Config *cfg_w = &configs[per_world_config ? w : 0];
    device bike::gpu::GpuWorldState *tyre_w =
        (has_world_states != 0 && world_states != nullptr)
            ? &world_states[w]
            : static_cast<device bike::gpu::GpuWorldState *>(nullptr);
    device const float *qpos_w =
        (has_qpos != 0 && qpos_batch != nullptr)
            ? (qpos_batch + w64 * idx.nq)
            : static_cast<device const float *>(nullptr);
    device float *tendon_w = static_cast<device float *>(nullptr);
    if (has_tendon_range != 0 && tendon_range_batch != nullptr && ntendon > 0) {
        if (tendon_range_world_stride >= 2 * ntendon) {
            tendon_w = tendon_range_batch + w64 * tendon_range_world_stride;
        } else if (tendon_range_world_stride == 0 && w == 0) {
            tendon_w = tendon_range_batch;
        } else {
            return;
        }
    }
    (void)bike::gpu::gpu_sp3_init_world(
        idx, cfg_w, &sp3_states[w], tyre_w, qpos_w, init_grade, ntendon, tendon_w);
}

inline void bike_metal_sp3_pre_solve_world(
    int w,
    float dt_s,
    thread const bike::gpu::GpuModelIndices &idx,
    device const bike::physics_core::GpuTyreModel *tyre_model,
    thread const bike::physics_core::GpuTerrainTableDesc &terrain_desc,
    device const float *terrain_f32,
    device const bike::physics_core::i32 *terrain_i32,
    device const bike::gpu::GpuSp3Config *configs,
    int per_world_config,
    device bike::gpu::GpuSp3WorldState *sp3_states,
    device bike::gpu::GpuWorldState *world_states,
    device const float *qpos_batch,
    device const float *qvel_batch,
    device const float *xpos_batch,
    device const float *xmat_batch,
    device const float *xanchor_batch,
    device const float *xaxis_batch,
    device float *qfrc_batch,
    device float *ctrl_batch,
    int ntendon,
    int has_tendon_range,
    device float *tendon_range_batch,
    int tendon_range_world_stride,
    int has_dof_frictionloss,
    device float *dof_frictionloss_batch,
    int dof_frictionloss_world_stride,
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch
) {
    if (!bike::gpu::gpu_validate_model_indices(idx, ntendon) || tyre_model == nullptr ||
        configs == nullptr || sp3_states == nullptr || world_states == nullptr) {
        return;
    }
    float qpos_w[bike::gpu::kMaxNq];
    float qvel_w[bike::gpu::kMaxNv];
    float qfrc_w[bike::gpu::kMaxNv];
    float ctrl_w[bike::gpu::kMaxNu];

    const int nq = idx.nq;
    const int nv = idx.nv;
    const int nu = idx.nu;
    const long w64 = static_cast<long>(w);

    device const float *qpos_src = qpos_batch + w64 * nq;
    device const float *qvel_src = qvel_batch + w64 * nv;
    device const float *xpos_src = xpos_batch + w64 * idx.nbody * 3;
    device const float *xmat_src = xmat_batch + w64 * idx.nbody * 9;
    device const float *xanchor_src = xanchor_batch + w64 * tyre_model->njnt * 3;
    device const float *xaxis_src = xaxis_batch + w64 * tyre_model->njnt * 3;

    for (int i = 0; i < nq; ++i) qpos_w[i] = qpos_src[i];
    for (int i = 0; i < nv; ++i) qvel_w[i] = qvel_src[i];

    device const bike::gpu::GpuSp3Config *cfg_w = &configs[per_world_config ? w : 0];
    device float *tendon_w = static_cast<device float *>(nullptr);
    if (has_tendon_range != 0 && tendon_range_batch != nullptr && ntendon > 0) {
        if (tendon_range_world_stride >= 2 * ntendon) {
            tendon_w = tendon_range_batch + w64 * tendon_range_world_stride;
        } else if (tendon_range_world_stride == 0 && w == 0) {
            tendon_w = tendon_range_batch;
        } else {
            return;
        }
    }
    device float *fl_w = static_cast<device float *>(nullptr);
    if (has_dof_frictionloss != 0 && dof_frictionloss_batch != nullptr) {
        if (dof_frictionloss_world_stride >= nv) {
            fl_w = dof_frictionloss_batch + w64 * dof_frictionloss_world_stride;
        } else if (dof_frictionloss_world_stride == 0 && w == 0) {
            fl_w = dof_frictionloss_batch;
        } else {
            return;
        }
    }

    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_w =
        (pneum_scratch != nullptr)
            ? &pneum_scratch[w]
            : static_cast<device bike::physics_core::GpuPneumaticStepScratch<float> *>(nullptr);

    const bike::physics_core::Status st = bike::gpu::gpu_sp3_pre_solve_world(
        w,
        dt_s,
        idx,
        tyre_model,
        terrain_desc,
        terrain_f32,
        terrain_i32,
        cfg_w,
        &sp3_states[w],
        world_states[w].compliant_wheels,
        &world_states[w].distributed_wheels[0][0],
        qpos_w,
        qvel_w,
        xpos_src,
        xmat_src,
        xanchor_src,
        xaxis_src,
        qfrc_w,
        ctrl_w,
        ntendon,
        tendon_w,
        fl_w,
        world_states[w].pneumatic_wheels,
        pneum_w);
    if (st != bike::physics_core::Status::ok) {
        return;
    }

    device float *qfrc_dst = qfrc_batch + w64 * nv;
    device float *ctrl_dst = ctrl_batch + w64 * nu;
    for (int i = 0; i < nv; ++i) qfrc_dst[i] = qfrc_w[i];
    for (int i = 0; i < nu; ++i) ctrl_dst[i] = ctrl_w[i];
}

inline void bike_metal_sp3_post_solve_world(
    int w,
    float dt_s,
    thread const bike::gpu::GpuModelIndices &idx,
    device const bike::gpu::GpuSp3Config *configs,
    int per_world_config,
    device bike::gpu::GpuSp3WorldState *sp3_states,
    device float *qpos_batch,
    device const float *qvel_batch,
    device const float *xmat_batch,
    int has_sensordata,
    int nsensordata,
    device const float *sensordata_batch,
    int has_efc,
    device const int *nefc_batch,
    int njmax,
    int efc_J_row_stride,
    device const int *efc_type_batch,
    device const int *efc_id_batch,
    device const float *efc_J_batch,
    device const float *efc_force_batch,
    int has_contacts,
    device const int *nacon_batch,
    int nconmax,
    int contact_efc_stride,
    int has_contact_worldid,
    device const int *contact_worldid_batch,
    device const int *contact_geom_batch,
    device const int *contact_efc_address_batch,
    int has_contact_dim,
    device const int *contact_dim_batch
) {
    if (!bike::gpu::gpu_validate_model_indices(idx) || configs == nullptr || sp3_states == nullptr) {
        return;
    }
    const long w64 = static_cast<long>(w);
    device const bike::gpu::GpuSp3Config *cfg_w = &configs[per_world_config ? w : 0];
    device const float *sd_w =
        (has_sensordata != 0 && sensordata_batch != nullptr && nsensordata > 0)
            ? (sensordata_batch + w64 * nsensordata)
            : static_cast<device const float *>(nullptr);

    int nefc_w = 0;
    device const int *efc_type_w = static_cast<device const int *>(nullptr);
    device const int *efc_id_w = static_cast<device const int *>(nullptr);
    device const float *efc_J_w = static_cast<device const float *>(nullptr);
    device const float *efc_force_w = static_cast<device const float *>(nullptr);
    if (has_efc != 0 && nefc_batch != nullptr && njmax > 0) {
        nefc_w = nefc_batch[w];
        if (nefc_w < 0) {
            nefc_w = 0;
        }
        if (nefc_w > njmax) {
            nefc_w = njmax;
        }
        efc_type_w = (efc_type_batch != nullptr)
                         ? (efc_type_batch + w64 * njmax)
                         : static_cast<device const int *>(nullptr);
        efc_id_w = (efc_id_batch != nullptr)
                       ? (efc_id_batch + w64 * njmax)
                       : static_cast<device const int *>(nullptr);
        efc_force_w = (efc_force_batch != nullptr)
                          ? (efc_force_batch + w64 * njmax)
                          : static_cast<device const float *>(nullptr);
        efc_J_w = (efc_J_batch != nullptr && efc_J_row_stride > 0)
                      ? (efc_J_batch + w64 * static_cast<long>(njmax) * efc_J_row_stride)
                      : static_cast<device const float *>(nullptr);
    }

    int ncon_or_total = 0;
    device const int *c_wid = static_cast<device const int *>(nullptr);
    device const int *c_geom = static_cast<device const int *>(nullptr);
    device const int *c_adr = static_cast<device const int *>(nullptr);
    device const int *c_dim = static_cast<device const int *>(nullptr);
    if (has_contacts != 0 && nacon_batch != nullptr && nconmax > 0) {
        if (has_contact_worldid != 0 && contact_worldid_batch != nullptr) {
            ncon_or_total = nacon_batch[0];
            if (ncon_or_total < 0) {
                ncon_or_total = 0;
            }
            if (ncon_or_total > nconmax) {
                ncon_or_total = nconmax;
            }
            c_wid = contact_worldid_batch;
            c_geom = contact_geom_batch;
            c_adr = contact_efc_address_batch;
            c_dim = (has_contact_dim != 0) ? contact_dim_batch
                                           : static_cast<device const int *>(nullptr);
        } else {
            ncon_or_total = nacon_batch[w];
            if (ncon_or_total < 0) {
                ncon_or_total = 0;
            }
            if (ncon_or_total > nconmax) {
                ncon_or_total = nconmax;
            }
            c_geom = (contact_geom_batch != nullptr)
                         ? (contact_geom_batch + w64 * 2 * nconmax)
                         : static_cast<device const int *>(nullptr);
            c_adr = (contact_efc_address_batch != nullptr)
                        ? (contact_efc_address_batch + w64 * nconmax * contact_efc_stride)
                        : static_cast<device const int *>(nullptr);
            c_dim = (has_contact_dim != 0 && contact_dim_batch != nullptr)
                        ? (contact_dim_batch + w64 * nconmax)
                        : static_cast<device const int *>(nullptr);
        }
    }

    (void)bike::gpu::gpu_sp3_post_solve_world(
        w,
        dt_s,
        idx,
        cfg_w,
        &sp3_states[w],
        qpos_batch + w64 * idx.nq,
        qvel_batch + w64 * idx.nv,
        xmat_batch + w64 * idx.nbody * 9,
        nsensordata,
        sd_w,
        nefc_w,
        efc_J_row_stride,
        efc_type_w,
        efc_id_w,
        efc_J_w,
        efc_force_w,
        ncon_or_total,
        contact_efc_stride,
        c_wid,
        c_geom,
        c_adr,
        c_dim);
}

struct BikeWpSp3InitArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::gpu::GpuSp3Config *configs;
    device bike::gpu::GpuSp3WorldState *sp3_states;
    device bike::gpu::GpuWorldState *world_states;
    device const float *qpos;
    device float *tendon_range;
    int nworld;
    int per_world_config;
    float init_grade;
    int ntendon;
    int tendon_range_world_stride;
    int has_world_states;
    int has_qpos;
    int has_tendon_range;
};

struct BikeWpSp3PreSolveArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::physics_core::GpuTyreModel *tyre_model;
    device const bike::physics_core::GpuTerrainTableDesc *terrain_desc;
    device const float *terrain_f32;
    device const bike::physics_core::i32 *terrain_i32;
    device const bike::gpu::GpuSp3Config *configs;
    device bike::gpu::GpuSp3WorldState *sp3_states;
    device bike::gpu::GpuWorldState *world_states;
    device const float *qpos;
    device const float *qvel;
    device const float *xpos;
    device const float *xmat;
    device const float *xanchor;
    device const float *xaxis;
    device float *qfrc_applied;
    device float *ctrl;
    device float *tendon_range;
    device float *dof_frictionloss;
    int nworld;
    float dt_s;
    int per_world_config;
    int ntendon;
    int tendon_range_world_stride;
    int dof_frictionloss_world_stride;
    int has_world_states;
    int has_tendon_range;
    int has_dof_frictionloss;
};

struct BikeWpSp3PneumPreSolveArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::physics_core::GpuTyreModel *tyre_model;
    device const bike::physics_core::GpuTerrainTableDesc *terrain_desc;
    device const float *terrain_f32;
    device const bike::physics_core::i32 *terrain_i32;
    device const bike::gpu::GpuSp3Config *configs;
    device bike::gpu::GpuSp3WorldState *sp3_states;
    device bike::gpu::GpuWorldState *world_states;
    device const float *qpos;
    device const float *qvel;
    device const float *xpos;
    device const float *xmat;
    device const float *xanchor;
    device const float *xaxis;
    device float *qfrc_applied;
    device float *ctrl;
    device float *tendon_range;
    device float *dof_frictionloss;
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch;
    int nworld;
    float dt_s;
    int per_world_config;
    int ntendon;
    int tendon_range_world_stride;
    int dof_frictionloss_world_stride;
    int has_world_states;
    int has_tendon_range;
    int has_dof_frictionloss;
    int has_pneum_scratch;
};

struct BikeWpSp3PostSolveArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::gpu::GpuSp3Config *configs;
    device bike::gpu::GpuSp3WorldState *sp3_states;
    device float *qpos;
    device const float *qvel;
    device const float *xmat;
    device const float *sensordata;
    device const int *nefc;
    device const int *efc_type;
    device const int *efc_id;
    device const float *efc_J;
    device const float *efc_force;
    device const int *nacon;
    device const int *contact_worldid;
    device const int *contact_geom;
    device const int *contact_efc_address;
    device const int *contact_dim;
    int nworld;
    float dt_s;
    int per_world_config;
    int nsensordata;
    int njmax;
    int efc_J_row_stride;
    int nconmax;
    int contact_efc_stride;
    int has_sensordata;
    int has_efc;
    int has_contacts;
    int has_contact_worldid;
    int has_contact_dim;
};

kernel void bike_wp_sp3_init_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpSp3InitArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    bike_metal_sp3_init_impl(
        w, idx, args->configs, args->per_world_config, args->sp3_states,
        args->has_world_states, args->world_states, args->has_qpos, args->qpos,
        args->init_grade, args->ntendon, args->has_tendon_range, args->tendon_range,
        args->tendon_range_world_stride);
}

kernel void bike_wp_sp3_pre_solve_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpSp3PreSolveArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld || args->terrain_desc == nullptr || args->has_world_states == 0) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    const bike::physics_core::GpuTerrainTableDesc terrain_desc = *(args->terrain_desc);
    bike_metal_sp3_pre_solve_world(
        w, args->dt_s, idx, args->tyre_model, terrain_desc, args->terrain_f32, args->terrain_i32,
        args->configs, args->per_world_config, args->sp3_states, args->world_states,
        args->qpos, args->qvel, args->xpos, args->xmat, args->xanchor, args->xaxis,
        args->qfrc_applied, args->ctrl, args->ntendon, args->has_tendon_range, args->tendon_range,
        args->tendon_range_world_stride, args->has_dof_frictionloss, args->dof_frictionloss,
        args->dof_frictionloss_world_stride,
        static_cast<device bike::physics_core::GpuPneumaticStepScratch<float> *>(nullptr));
}

kernel void bike_wp_sp3_pneum_pre_solve_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpSp3PneumPreSolveArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld || args->terrain_desc == nullptr || args->has_world_states == 0) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    const bike::physics_core::GpuTerrainTableDesc terrain_desc = *(args->terrain_desc);
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch =
        (args->has_pneum_scratch != 0 && args->pneum_scratch != nullptr)
            ? args->pneum_scratch
            : static_cast<device bike::physics_core::GpuPneumaticStepScratch<float> *>(nullptr);
    bike_metal_sp3_pre_solve_world(
        w, args->dt_s, idx, args->tyre_model, terrain_desc, args->terrain_f32, args->terrain_i32,
        args->configs, args->per_world_config, args->sp3_states, args->world_states,
        args->qpos, args->qvel, args->xpos, args->xmat, args->xanchor, args->xaxis,
        args->qfrc_applied, args->ctrl, args->ntendon, args->has_tendon_range, args->tendon_range,
        args->tendon_range_world_stride, args->has_dof_frictionloss, args->dof_frictionloss,
        args->dof_frictionloss_world_stride,
        pneum_scratch);
}

kernel void bike_wp_sp3_post_solve_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpSp3PostSolveArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    bike_metal_sp3_post_solve_world(
        w, args->dt_s, idx, args->configs, args->per_world_config, args->sp3_states,
        args->qpos, args->qvel, args->xmat, args->has_sensordata, args->nsensordata,
        args->sensordata, args->has_efc, args->nefc, args->njmax, args->efc_J_row_stride,
        args->efc_type, args->efc_id, args->efc_J, args->efc_force,
        args->has_contacts, args->nacon, args->nconmax, args->contact_efc_stride,
        args->has_contact_worldid, args->contact_worldid, args->contact_geom,
        args->contact_efc_address, args->has_contact_dim, args->contact_dim);
}

kernel void bike_gpu_sp3_init_metal_kernel(
    constant int &nworld [[buffer(0)]],
    constant bike::gpu::GpuModelIndices &idx_const [[buffer(1)]],
    device const bike::gpu::GpuSp3Config *configs [[buffer(2)]],
    constant int &per_world_config [[buffer(3)]],
    device bike::gpu::GpuSp3WorldState *sp3_states [[buffer(4)]],
    constant int &has_world_states [[buffer(5)]],
    device bike::gpu::GpuWorldState *world_states [[buffer(6)]],
    constant int &has_qpos [[buffer(7)]],
    device const float *qpos_batch [[buffer(8)]],
    constant float &init_grade [[buffer(9)]],
    constant int &ntendon [[buffer(10)]],
    constant int &has_tendon_range [[buffer(11)]],
    device float *tendon_range_batch [[buffer(12)]],
    constant int &tendon_range_world_stride [[buffer(13)]],
    uint gid [[thread_position_in_grid]]
) {
    const int w = static_cast<int>(gid);
    if (w >= nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx = idx_const;
    bike_metal_sp3_init_impl(
        w, idx, configs, per_world_config, sp3_states, has_world_states, world_states,
        has_qpos, qpos_batch, init_grade, ntendon, has_tendon_range, tendon_range_batch,
        tendon_range_world_stride);
}

kernel void bike_gpu_sp3_pre_solve_metal_kernel(
    constant int &nworld [[buffer(0)]],
    constant float &dt_s [[buffer(1)]],
    constant bike::gpu::GpuModelIndices &idx_const [[buffer(2)]],
    device const bike::physics_core::GpuTyreModel *tyre_model [[buffer(3)]],
    constant bike::physics_core::GpuTerrainTableDesc &terrain_desc_const [[buffer(4)]],
    device const float *terrain_f32 [[buffer(5)]],
    device const bike::physics_core::i32 *terrain_i32 [[buffer(6)]],
    device const bike::gpu::GpuSp3Config *configs [[buffer(7)]],
    constant int &per_world_config [[buffer(8)]],
    device bike::gpu::GpuSp3WorldState *sp3_states [[buffer(9)]],
    device bike::gpu::GpuWorldState *world_states [[buffer(10)]],
    device const float *qpos_batch [[buffer(11)]],
    device const float *qvel_batch [[buffer(12)]],
    device const float *xpos_batch [[buffer(13)]],
    device const float *xmat_batch [[buffer(14)]],
    device const float *xanchor_batch [[buffer(15)]],
    device const float *xaxis_batch [[buffer(16)]],
    device float *qfrc_batch [[buffer(17)]],
    device float *ctrl_batch [[buffer(18)]],
    constant int &ntendon [[buffer(19)]],
    constant int &has_tendon_range [[buffer(20)]],
    device float *tendon_range_batch [[buffer(21)]],
    constant int &tendon_range_world_stride [[buffer(22)]],
    constant int &has_dof_frictionloss [[buffer(23)]],
    device float *dof_frictionloss_batch [[buffer(24)]],
    constant int &dof_frictionloss_world_stride [[buffer(25)]],
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch [[buffer(26)]],
    uint gid [[thread_position_in_grid]]
) {
    const int w = static_cast<int>(gid);
    if (w >= nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx = idx_const;
    const bike::physics_core::GpuTerrainTableDesc terrain_desc = terrain_desc_const;
    bike_metal_sp3_pre_solve_world(
        w, dt_s, idx, tyre_model, terrain_desc, terrain_f32, terrain_i32,
        configs, per_world_config, sp3_states, world_states,
        qpos_batch, qvel_batch, xpos_batch, xmat_batch, xanchor_batch, xaxis_batch,
        qfrc_batch, ctrl_batch, ntendon, has_tendon_range, tendon_range_batch,
        tendon_range_world_stride, has_dof_frictionloss, dof_frictionloss_batch,
        dof_frictionloss_world_stride, pneum_scratch);
}

kernel void bike_gpu_sp3_post_solve_metal_kernel(
    constant int &nworld [[buffer(0)]],
    constant float &dt_s [[buffer(1)]],
    constant bike::gpu::GpuModelIndices &idx_const [[buffer(2)]],
    device const bike::gpu::GpuSp3Config *configs [[buffer(3)]],
    constant int &per_world_config [[buffer(4)]],
    device bike::gpu::GpuSp3WorldState *sp3_states [[buffer(5)]],
    device float *qpos_batch [[buffer(6)]],
    device const float *qvel_batch [[buffer(7)]],
    device const float *xmat_batch [[buffer(8)]],
    constant int &has_sensordata [[buffer(9)]],
    constant int &nsensordata [[buffer(10)]],
    device const float *sensordata_batch [[buffer(11)]],
    constant int &has_efc [[buffer(12)]],
    device const int *nefc_batch [[buffer(13)]],
    constant int &njmax [[buffer(14)]],
    constant int &efc_J_row_stride [[buffer(15)]],
    device const int *efc_type_batch [[buffer(16)]],
    device const int *efc_id_batch [[buffer(17)]],
    device const float *efc_J_batch [[buffer(18)]],
    device const float *efc_force_batch [[buffer(19)]],
    constant int &has_contacts [[buffer(20)]],
    device const int *nacon_batch [[buffer(21)]],
    constant int &nconmax [[buffer(22)]],
    constant int &contact_efc_stride [[buffer(23)]],
    constant int &has_contact_worldid [[buffer(24)]],
    device const int *contact_worldid_batch [[buffer(25)]],
    device const int *contact_geom_batch [[buffer(26)]],
    device const int *contact_efc_address_batch [[buffer(27)]],
    constant int &has_contact_dim [[buffer(28)]],
    device const int *contact_dim_batch [[buffer(29)]],
    uint gid [[thread_position_in_grid]]
) {
    const int w = static_cast<int>(gid);
    if (w >= nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx = idx_const;
    bike_metal_sp3_post_solve_world(
        w, dt_s, idx, configs, per_world_config, sp3_states,
        qpos_batch, qvel_batch, xmat_batch, has_sensordata, nsensordata,
        sensordata_batch, has_efc, nefc_batch, njmax, efc_J_row_stride,
        efc_type_batch, efc_id_batch, efc_J_batch, efc_force_batch,
        has_contacts, nacon_batch, nconmax, contact_efc_stride,
        has_contact_worldid, contact_worldid_batch, contact_geom_batch,
        contact_efc_address_batch, has_contact_dim, contact_dim_batch);
}
)MSL";
    combined += R"MSL(
inline void bike_metal_sp4_init_impl(
    int w,
    thread const bike::gpu::GpuModelIndices &idx,
    device const bike::gpu::GpuSp3Config *configs,
    int per_world_config,
    device bike::gpu::GpuSp3WorldState *sp3_states,
    device bike::gpu::GpuSp4WorldState *sp4_states,
    int has_world_states,
    device bike::gpu::GpuWorldState *world_states,
    int has_qpos,
    device const float *qpos_batch,
    float init_grade,
    int ntendon,
    int has_tendon_range,
    device float *tendon_range_batch,
    int tendon_range_world_stride
) {
    if (!bike::gpu::gpu_validate_model_indices(idx, ntendon) ||
        configs == nullptr || sp3_states == nullptr || sp4_states == nullptr) {
        return;
    }
    const long w64 = static_cast<long>(w);
    device const bike::gpu::GpuSp3Config *cfg_w = &configs[per_world_config ? w : 0];
    device bike::gpu::GpuWorldState *ws_w =
        (has_world_states != 0 && world_states != nullptr)
            ? &world_states[w]
            : static_cast<device bike::gpu::GpuWorldState *>(nullptr);
    device const float *qpos_w =
        (has_qpos != 0 && qpos_batch != nullptr)
            ? (qpos_batch + w64 * idx.nq)
            : static_cast<device const float *>(nullptr);
    device float *tr_w = static_cast<device float *>(nullptr);
    if (has_tendon_range != 0 && tendon_range_batch != nullptr && ntendon > 0) {
        if (tendon_range_world_stride >= 2 * ntendon) {
            tr_w = tendon_range_batch + w64 * tendon_range_world_stride;
        } else if (tendon_range_world_stride == 0 && w == 0) {
            tr_w = tendon_range_batch;
        } else {
            return;
        }
    }
    (void)bike::gpu::gpu_sp4_init_world(
        idx, cfg_w, &sp3_states[w], &sp4_states[w], ws_w, qpos_w, init_grade, ntendon, tr_w);
}

inline void bike_metal_sp4_pre_solve_and_pass0_impl(
    int w,
    float dt_s,
    thread const bike::gpu::GpuModelIndices &idx,
    device const bike::physics_core::GpuTyreModel *tyre_model,
    thread const bike::physics_core::GpuTerrainTableDesc &terrain_desc,
    device const float *terrain_f32,
    device const bike::physics_core::i32 *terrain_i32,
    device const bike::gpu::GpuSp3Config *configs,
    int per_world_config,
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params,
    device bike::gpu::GpuSp3WorldState *sp3_states,
    device bike::gpu::GpuSp4WorldState *sp4_states,
    device bike::gpu::GpuWorldState *world_states,
    device const float *qpos_batch,
    device const float *qvel_batch,
    device const float *qfrc_bias_batch,
    device const float *xpos_batch,
    device const float *xmat_batch,
    device const float *xanchor_batch,
    device const float *xaxis_batch,
    device const float *site_xpos_batch,
    device const float *subtree_com_batch,
    int has_body_subtreemass,
    device const float *body_subtreemass,
    int qM_row_stride,
    int M_world_stride,
    device const int *M_rowadr,
    device const int *M_rownnz,
    device const int *M_colind,
    device const float *d_M_batch,
    device float *qfrc_batch,
    device float *ctrl_batch,
    int ntendon,
    int has_tendon_range,
    device float *tendon_range_batch,
    int tendon_range_world_stride,
    int has_dof_frictionloss,
    device float *dof_frictionloss_batch,
    int dof_frictionloss_world_stride,
    int has_qacc_warmstart,
    device float *qacc_warmstart_batch,
    int has_pneum_scratch,
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch
) {
    if (!bike::gpu::gpu_validate_model_indices(idx, ntendon) || tyre_model == nullptr ||
        configs == nullptr || spindle_params == nullptr ||
        sp3_states == nullptr || sp4_states == nullptr || world_states == nullptr ||
        M_world_stride <= 0) {
        return;
    }
    float qpos_w[bike::gpu::kMaxNq];
    float qvel_w[bike::gpu::kMaxNv];
    float qfrc_w[bike::gpu::kMaxNv];
    float ctrl_w[bike::gpu::kMaxNu];

    const int nq = idx.nq;
    const int nv = idx.nv;
    const int nu = idx.nu;
    const long w64 = static_cast<long>(w);

    device const float *qpos_src = qpos_batch + w64 * nq;
    device const float *qvel_src = qvel_batch + w64 * nv;
    for (int i = 0; i < nq; ++i) qpos_w[i] = qpos_src[i];
    for (int i = 0; i < nv; ++i) qvel_w[i] = qvel_src[i];

    device const bike::gpu::GpuSp3Config *cfg_w = &configs[per_world_config ? w : 0];
    device float *tendon_w = static_cast<device float *>(nullptr);
    if (has_tendon_range != 0 && tendon_range_batch != nullptr && ntendon > 0) {
        if (tendon_range_world_stride >= 2 * ntendon) {
            tendon_w = tendon_range_batch + w64 * tendon_range_world_stride;
        } else if (tendon_range_world_stride == 0 && w == 0) {
            tendon_w = tendon_range_batch;
        } else {
            return;
        }
    }
    device float *fl_w = static_cast<device float *>(nullptr);
    if (has_dof_frictionloss != 0 && dof_frictionloss_batch != nullptr) {
        if (dof_frictionloss_world_stride >= nv) {
            fl_w = dof_frictionloss_batch + w64 * dof_frictionloss_world_stride;
        } else if (dof_frictionloss_world_stride == 0 && w == 0) {
            fl_w = dof_frictionloss_batch;
        } else {
            return;
        }
    }
    device const float *submass_ptr =
        (has_body_subtreemass != 0 && body_subtreemass != nullptr)
            ? body_subtreemass
            : static_cast<device const float *>(nullptr);
    device float *ws_w =
        (has_qacc_warmstart != 0 && qacc_warmstart_batch != nullptr)
            ? (qacc_warmstart_batch + w64 * nv)
            : static_cast<device float *>(nullptr);

    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_w =
        (has_pneum_scratch != 0 && pneum_scratch != nullptr)
            ? &pneum_scratch[w]
            : static_cast<device bike::physics_core::GpuPneumaticStepScratch<float> *>(nullptr);

    const bike::physics_core::Status st = bike::gpu::gpu_sp4_pre_solve_and_pass0_world(
        w,
        dt_s,
        idx,
        tyre_model,
        terrain_desc,
        terrain_f32,
        terrain_i32,
        cfg_w,
        spindle_params,
        &sp3_states[w],
        &sp4_states[w],
        world_states[w].compliant_wheels,
        &world_states[w].distributed_wheels[0][0],
        qpos_w,
        qvel_w,
        qfrc_bias_batch + w64 * nv,
        xpos_batch + w64 * idx.nbody * 3,
        xmat_batch + w64 * idx.nbody * 9,
        xanchor_batch + w64 * tyre_model->njnt * 3,
        xaxis_batch + w64 * tyre_model->njnt * 3,
        site_xpos_batch + w64 * spindle_params->nsite * 3,
        subtree_com_batch + w64 * idx.nbody * 3,
        submass_ptr,
        qM_row_stride,
        M_rowadr,
        M_rownnz,
        M_colind,
        d_M_batch + w64 * M_world_stride,
        qfrc_w,
        ctrl_w,
        ntendon,
        tendon_w,
        fl_w,
        ws_w,
        world_states[w].pneumatic_wheels,
        pneum_w);
    if (st != bike::physics_core::Status::ok) {
        return;
    }

    device float *qfrc_dst = qfrc_batch + w64 * nv;
    device float *ctrl_dst = ctrl_batch + w64 * nu;
    for (int i = 0; i < nv; ++i) qfrc_dst[i] = qfrc_w[i];
    for (int i = 0; i < nu; ++i) ctrl_dst[i] = ctrl_w[i];
}

inline void bike_metal_sp4_pass_stage_impl(
    int stage,
    int w,
    int nq,
    int nv,
    int nu,
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params,
    device bike::gpu::GpuSp4WorldState *sp4_states,
    device const float *qpos_batch,
    device const float *qvel_batch,
    device const float *qfrc_bias_batch,
    device const float *qacc_batch,
    int qM_row_stride,
    int M_world_stride,
    device const int *M_rowadr,
    device const int *M_rownnz,
    device const int *M_colind,
    device const float *d_M_batch,
    device float *ctrl_batch,
    int has_qacc_warmstart,
    device float *qacc_warmstart_batch
) {
    if (spindle_params == nullptr || sp4_states == nullptr || M_world_stride <= 0) {
        return;
    }
    const long w64 = static_cast<long>(w);
    device float *ws_w =
        (has_qacc_warmstart != 0 && qacc_warmstart_batch != nullptr)
            ? (qacc_warmstart_batch + w64 * nv)
            : static_cast<device float *>(nullptr);
    if (stage == 1) {
        (void)bike::gpu::gpu_sp4_pass1_world(
            spindle_params, &sp4_states[w],
            qpos_batch + w64 * nq, qvel_batch + w64 * nv,
            qfrc_bias_batch + w64 * nv, qacc_batch + w64 * nv,
            qM_row_stride, M_rowadr, M_rownnz, M_colind,
            d_M_batch + w64 * M_world_stride, ctrl_batch + w64 * nu, ws_w);
    } else if (stage == 2) {
        (void)bike::gpu::gpu_sp4_pass2_world(
            spindle_params, &sp4_states[w],
            qpos_batch + w64 * nq, qvel_batch + w64 * nv,
            qfrc_bias_batch + w64 * nv, qacc_batch + w64 * nv,
            qM_row_stride, M_rowadr, M_rownnz, M_colind,
            d_M_batch + w64 * M_world_stride, ctrl_batch + w64 * nu, ws_w);
    } else if (stage == 3) {
        (void)bike::gpu::gpu_sp4_pass3_world(
            spindle_params, &sp4_states[w],
            qpos_batch + w64 * nq, qvel_batch + w64 * nv,
            qfrc_bias_batch + w64 * nv, qacc_batch + w64 * nv,
            qM_row_stride, M_rowadr, M_rownnz, M_colind,
            d_M_batch + w64 * M_world_stride, ctrl_batch + w64 * nu, ws_w);
    }
}

inline void bike_metal_sp4_post_solve_impl(
    int w,
    float dt_s,
    thread const bike::gpu::GpuModelIndices &idx,
    device const bike::gpu::GpuSp3Config *configs,
    int per_world_config,
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params,
    device bike::gpu::GpuSp3WorldState *sp3_states,
    device bike::gpu::GpuSp4WorldState *sp4_states,
    device float *qpos_batch,
    device const float *qvel_batch,
    int has_xpos,
    device const float *xpos_batch,
    device const float *xmat_batch,
    int has_xanchor,
    device const float *xanchor_batch,
    int has_sensordata,
    int nsensordata,
    device const float *sensordata_batch,
    int has_efc,
    device const int *nefc_batch,
    int njmax,
    int efc_J_row_stride,
    device const int *efc_type_batch,
    device const int *efc_id_batch,
    device const float *efc_J_batch,
    device const float *efc_force_batch,
    int has_contacts,
    device const int *nacon_batch,
    int nconmax,
    int contact_efc_stride,
    int has_contact_worldid,
    device const int *contact_worldid_batch,
    device const int *contact_geom_batch,
    device const int *contact_efc_address_batch,
    int has_contact_dim,
    device const int *contact_dim_batch
) {
    if (!bike::gpu::gpu_validate_model_indices(idx) || configs == nullptr ||
        spindle_params == nullptr || sp3_states == nullptr || sp4_states == nullptr) {
        return;
    }
    const long w64 = static_cast<long>(w);
    device const bike::gpu::GpuSp3Config *cfg_w = &configs[per_world_config ? w : 0];
    device const float *sd_w =
        (has_sensordata != 0 && sensordata_batch != nullptr && nsensordata > 0)
            ? (sensordata_batch + w64 * nsensordata)
            : static_cast<device const float *>(nullptr);
    int nefc_w = 0;
    device const int *efc_type_w = static_cast<device const int *>(nullptr);
    device const int *efc_id_w = static_cast<device const int *>(nullptr);
    device const float *efc_J_w = static_cast<device const float *>(nullptr);
    device const float *efc_force_w = static_cast<device const float *>(nullptr);
    if (has_efc != 0 && nefc_batch != nullptr && njmax > 0) {
        nefc_w = nefc_batch[w];
        if (nefc_w < 0) nefc_w = 0;
        if (nefc_w > njmax) nefc_w = njmax;
        efc_type_w = (efc_type_batch != nullptr) ? (efc_type_batch + w64 * njmax)
                                                 : static_cast<device const int *>(nullptr);
        efc_id_w = (efc_id_batch != nullptr) ? (efc_id_batch + w64 * njmax)
                                             : static_cast<device const int *>(nullptr);
        efc_force_w = (efc_force_batch != nullptr) ? (efc_force_batch + w64 * njmax)
                                                   : static_cast<device const float *>(nullptr);
        efc_J_w = (efc_J_batch != nullptr && efc_J_row_stride > 0)
                      ? (efc_J_batch + w64 * static_cast<long>(njmax) * efc_J_row_stride)
                      : static_cast<device const float *>(nullptr);
    }
    int ncon_or_total = 0;
    device const int *c_wid = static_cast<device const int *>(nullptr);
    device const int *c_geom = static_cast<device const int *>(nullptr);
    device const int *c_adr = static_cast<device const int *>(nullptr);
    device const int *c_dim = static_cast<device const int *>(nullptr);
    if (has_contacts != 0 && nacon_batch != nullptr && nconmax > 0) {
        if (has_contact_worldid != 0 && contact_worldid_batch != nullptr) {
            ncon_or_total = nacon_batch[0];
            if (ncon_or_total < 0) ncon_or_total = 0;
            if (ncon_or_total > nconmax) ncon_or_total = nconmax;
            c_wid = contact_worldid_batch;
            c_geom = contact_geom_batch;
            c_adr = contact_efc_address_batch;
            c_dim = (has_contact_dim != 0) ? contact_dim_batch
                                           : static_cast<device const int *>(nullptr);
        } else {
            ncon_or_total = nacon_batch[w];
            if (ncon_or_total < 0) ncon_or_total = 0;
            if (ncon_or_total > nconmax) ncon_or_total = nconmax;
            c_geom = (contact_geom_batch != nullptr) ? (contact_geom_batch + w64 * 2 * nconmax)
                                                     : static_cast<device const int *>(nullptr);
            c_adr = (contact_efc_address_batch != nullptr)
                        ? (contact_efc_address_batch + w64 * nconmax * contact_efc_stride)
                        : static_cast<device const int *>(nullptr);
            c_dim = (has_contact_dim != 0 && contact_dim_batch != nullptr)
                        ? (contact_dim_batch + w64 * nconmax)
                        : static_cast<device const int *>(nullptr);
        }
    }
    const int nj = spindle_params->njnt;
    device const float *xpos_w =
        (has_xpos != 0 && xpos_batch != nullptr)
            ? (xpos_batch + w64 * idx.nbody * 3)
            : static_cast<device const float *>(nullptr);
    device const float *xanchor_w =
        (has_xanchor != 0 && xanchor_batch != nullptr && nj > 0)
            ? (xanchor_batch + w64 * nj * 3)
            : static_cast<device const float *>(nullptr);

    (void)bike::gpu::gpu_sp4_post_solve_world(
        w,
        dt_s,
        idx,
        cfg_w,
        spindle_params,
        &sp3_states[w],
        &sp4_states[w],
        qpos_batch + w64 * idx.nq,
        qvel_batch + w64 * idx.nv,
        xpos_w,
        (xmat_batch != nullptr) ? (xmat_batch + w64 * idx.nbody * 9)
                                : static_cast<device const float *>(nullptr),
        xanchor_w,
        nsensordata,
        sd_w,
        nefc_w,
        efc_J_row_stride,
        efc_type_w,
        efc_id_w,
        efc_J_w,
        efc_force_w,
        ncon_or_total,
        contact_efc_stride,
        c_wid,
        c_geom,
        c_adr,
        c_dim);
}

)MSL";
    combined += R"MSL(
struct BikeWpSp4InitArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::gpu::GpuSp3Config *configs;
    device bike::gpu::GpuSp3WorldState *sp3_states;
    device bike::gpu::GpuSp4WorldState *sp4_states;
    device bike::gpu::GpuWorldState *world_states;
    device const float *qpos;
    device float *tendon_range;
    int nworld;
    int per_world_config;
    float init_grade;
    int ntendon;
    int tendon_range_world_stride;
    int has_world_states;
    int has_qpos;
    int has_tendon_range;
};

struct BikeWpSp4PreSolveArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::physics_core::GpuTyreModel *tyre_model;
    device const bike::physics_core::GpuTerrainTableDesc *terrain_desc;
    device const float *terrain_f32;
    device const bike::physics_core::i32 *terrain_i32;
    device const bike::gpu::GpuSp3Config *configs;
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params;
    device bike::gpu::GpuSp3WorldState *sp3_states;
    device bike::gpu::GpuSp4WorldState *sp4_states;
    device bike::gpu::GpuWorldState *world_states;
    device const float *qpos;
    device const float *qvel;
    device const float *qfrc_bias;
    device const float *xpos;
    device const float *xmat;
    device const float *xanchor;
    device const float *xaxis;
    device const float *site_xpos;
    device const float *subtree_com;
    device const float *body_subtreemass;
    device const int *M_rowadr;
    device const int *M_rownnz;
    device const int *M_colind;
    device const float *d_M;
    device float *qfrc_applied;
    device float *ctrl;
    device float *tendon_range;
    device float *dof_frictionloss;
    device float *qacc_warmstart;
    int nworld;
    float dt_s;
    int per_world_config;
    int qM_row_stride;
    int M_world_stride;
    int ntendon;
    int tendon_range_world_stride;
    int dof_frictionloss_world_stride;
    int has_body_subtreemass;
    int has_tendon_range;
    int has_dof_frictionloss;
    int has_qacc_warmstart;
};

struct BikeWpSp4PneumPreSolveArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::physics_core::GpuTyreModel *tyre_model;
    device const bike::physics_core::GpuTerrainTableDesc *terrain_desc;
    device const float *terrain_f32;
    device const bike::physics_core::i32 *terrain_i32;
    device const bike::gpu::GpuSp3Config *configs;
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params;
    device bike::gpu::GpuSp3WorldState *sp3_states;
    device bike::gpu::GpuSp4WorldState *sp4_states;
    device bike::gpu::GpuWorldState *world_states;
    device const float *qpos;
    device const float *qvel;
    device const float *qfrc_bias;
    device const float *xpos;
    device const float *xmat;
    device const float *xanchor;
    device const float *xaxis;
    device const float *site_xpos;
    device const float *subtree_com;
    device const float *body_subtreemass;
    device const int *M_rowadr;
    device const int *M_rownnz;
    device const int *M_colind;
    device const float *d_M;
    device float *qfrc_applied;
    device float *ctrl;
    device float *tendon_range;
    device float *dof_frictionloss;
    device float *qacc_warmstart;
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch;
    int nworld;
    float dt_s;
    int per_world_config;
    int qM_row_stride;
    int M_world_stride;
    int ntendon;
    int tendon_range_world_stride;
    int dof_frictionloss_world_stride;
    int has_body_subtreemass;
    int has_tendon_range;
    int has_dof_frictionloss;
    int has_qacc_warmstart;
};

struct BikeWpSp4PassArgs {
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params;
    device bike::gpu::GpuSp4WorldState *sp4_states;
    device const float *qpos;
    device const float *qvel;
    device const float *qfrc_bias;
    device const float *qacc;
    device const int *M_rowadr;
    device const int *M_rownnz;
    device const int *M_colind;
    device const float *d_M;
    device float *ctrl;
    device float *qacc_warmstart;
    int nworld;
    int nq;
    int nv;
    int nu;
    int qM_row_stride;
    int M_world_stride;
    int has_qacc_warmstart;
};

struct BikeWpSp4PostSolveArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::gpu::GpuSp3Config *configs;
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params;
    device bike::gpu::GpuSp3WorldState *sp3_states;
    device bike::gpu::GpuSp4WorldState *sp4_states;
    device float *qpos;
    device const float *qvel;
    device const float *xpos;
    device const float *xmat;
    device const float *xanchor;
    device const float *sensordata;
    device const int *nefc;
    device const int *efc_type;
    device const int *efc_id;
    device const float *efc_J;
    device const float *efc_force;
    device const int *nacon;
    device const int *contact_worldid;
    device const int *contact_geom;
    device const int *contact_efc_address;
    device const int *contact_dim;
    int nworld;
    float dt_s;
    int per_world_config;
    int nsensordata;
    int njmax;
    int efc_J_row_stride;
    int nconmax;
    int contact_efc_stride;
    int has_xpos;
    int has_xanchor;
    int has_sensordata;
    int has_efc;
    int has_contacts;
    int has_contact_worldid;
    int has_contact_dim;
};

kernel void bike_wp_sp4_init_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpSp4InitArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    bike_metal_sp4_init_impl(
        w, idx, args->configs, args->per_world_config,
        args->sp3_states, args->sp4_states,
        args->has_world_states, args->world_states,
        args->has_qpos, args->qpos, args->init_grade, args->ntendon,
        args->has_tendon_range, args->tendon_range, args->tendon_range_world_stride);
}

kernel void bike_wp_sp4_pre_solve_and_pass0_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpSp4PreSolveArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld || args->terrain_desc == nullptr) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    const bike::physics_core::GpuTerrainTableDesc terrain_desc = *(args->terrain_desc);
    bike_metal_sp4_pre_solve_and_pass0_impl(
        w, args->dt_s, idx, args->tyre_model, terrain_desc, args->terrain_f32, args->terrain_i32,
        args->configs, args->per_world_config, args->spindle_params,
        args->sp3_states, args->sp4_states, args->world_states,
        args->qpos, args->qvel, args->qfrc_bias,
        args->xpos, args->xmat, args->xanchor, args->xaxis,
        args->site_xpos, args->subtree_com, args->has_body_subtreemass, args->body_subtreemass,
        args->qM_row_stride, args->M_world_stride,
        args->M_rowadr, args->M_rownnz, args->M_colind, args->d_M,
        args->qfrc_applied, args->ctrl,
        args->ntendon, args->has_tendon_range, args->tendon_range, args->tendon_range_world_stride,
        args->has_dof_frictionloss, args->dof_frictionloss, args->dof_frictionloss_world_stride,
        args->has_qacc_warmstart, args->qacc_warmstart,
        0, static_cast<device bike::physics_core::GpuPneumaticStepScratch<float> *>(nullptr));
}

kernel void bike_wp_sp4_pneum_pre_solve_and_pass0_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpSp4PneumPreSolveArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld || args->terrain_desc == nullptr) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    const bike::physics_core::GpuTerrainTableDesc terrain_desc = *(args->terrain_desc);
    bike_metal_sp4_pre_solve_and_pass0_impl(
        w, args->dt_s, idx, args->tyre_model, terrain_desc, args->terrain_f32, args->terrain_i32,
        args->configs, args->per_world_config, args->spindle_params,
        args->sp3_states, args->sp4_states, args->world_states,
        args->qpos, args->qvel, args->qfrc_bias,
        args->xpos, args->xmat, args->xanchor, args->xaxis,
        args->site_xpos, args->subtree_com, args->has_body_subtreemass, args->body_subtreemass,
        args->qM_row_stride, args->M_world_stride,
        args->M_rowadr, args->M_rownnz, args->M_colind, args->d_M,
        args->qfrc_applied, args->ctrl,
        args->ntendon, args->has_tendon_range, args->tendon_range, args->tendon_range_world_stride,
        args->has_dof_frictionloss, args->dof_frictionloss, args->dof_frictionloss_world_stride,
        args->has_qacc_warmstart, args->qacc_warmstart,
        (args->pneum_scratch != nullptr) ? 1 : 0, args->pneum_scratch);
}

kernel void bike_wp_sp4_pass1_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpSp4PassArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld) {
        return;
    }
    bike_metal_sp4_pass_stage_impl(
        1, w, args->nq, args->nv, args->nu,
        args->spindle_params, args->sp4_states,
        args->qpos, args->qvel, args->qfrc_bias, args->qacc,
        args->qM_row_stride, args->M_world_stride,
        args->M_rowadr, args->M_rownnz, args->M_colind, args->d_M, args->ctrl,
        args->has_qacc_warmstart, args->qacc_warmstart);
}

kernel void bike_wp_sp4_pass2_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpSp4PassArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld) {
        return;
    }
    bike_metal_sp4_pass_stage_impl(
        2, w, args->nq, args->nv, args->nu,
        args->spindle_params, args->sp4_states,
        args->qpos, args->qvel, args->qfrc_bias, args->qacc,
        args->qM_row_stride, args->M_world_stride,
        args->M_rowadr, args->M_rownnz, args->M_colind, args->d_M, args->ctrl,
        args->has_qacc_warmstart, args->qacc_warmstart);
}

kernel void bike_wp_sp4_pass3_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpSp4PassArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld) {
        return;
    }
    bike_metal_sp4_pass_stage_impl(
        3, w, args->nq, args->nv, args->nu,
        args->spindle_params, args->sp4_states,
        args->qpos, args->qvel, args->qfrc_bias, args->qacc,
        args->qM_row_stride, args->M_world_stride,
        args->M_rowadr, args->M_rownnz, args->M_colind, args->d_M, args->ctrl,
        args->has_qacc_warmstart, args->qacc_warmstart);
}

kernel void bike_wp_sp4_post_solve_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpSp4PostSolveArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    bike_metal_sp4_post_solve_impl(
        w, args->dt_s, idx, args->configs, args->per_world_config, args->spindle_params,
        args->sp3_states, args->sp4_states,
        args->qpos, args->qvel, args->has_xpos, args->xpos, args->xmat,
        args->has_xanchor, args->xanchor,
        args->has_sensordata, args->nsensordata, args->sensordata,
        args->has_efc, args->nefc, args->njmax, args->efc_J_row_stride,
        args->efc_type, args->efc_id, args->efc_J, args->efc_force,
        args->has_contacts, args->nacon, args->nconmax, args->contact_efc_stride,
        args->has_contact_worldid, args->contact_worldid, args->contact_geom,
        args->contact_efc_address, args->has_contact_dim, args->contact_dim);
}

struct BikeMetalSp4InitScalars {
    bike::gpu::GpuModelIndices idx;
    int nworld;
    int per_world_config;
    float init_grade;
    int ntendon;
    int tendon_range_world_stride;
    int has_world_states;
    int has_qpos;
    int has_tendon_range;
};

struct BikeMetalSp4PreSolveScalars {
    bike::gpu::GpuModelIndices idx;
    bike::physics_core::GpuTerrainTableDesc terrain_desc;
    int nworld;
    float dt_s;
    int per_world_config;
    int has_body_subtreemass;
    int qM_row_stride;
    int M_world_stride;
    int ntendon;
    int has_tendon_range;
    int tendon_range_world_stride;
    int has_dof_frictionloss;
    int dof_frictionloss_world_stride;
    int has_qacc_warmstart;
    int has_pneum_scratch;
};

struct BikeMetalSp4PassScalars {
    int stage;
    int nworld;
    int nq;
    int nv;
    int nu;
    int qM_row_stride;
    int M_world_stride;
    int has_qacc_warmstart;
};

struct BikeMetalSp4PostSolveScalars {
    bike::gpu::GpuModelIndices idx;
    int nworld;
    float dt_s;
    int per_world_config;
    int has_xpos;
    int has_xanchor;
    int has_sensordata;
    int nsensordata;
    int has_efc;
    int njmax;
    int efc_J_row_stride;
    int has_contacts;
    int nconmax;
    int contact_efc_stride;
    int has_contact_worldid;
    int has_contact_dim;
};

kernel void bike_gpu_sp4_init_metal_kernel(
    constant BikeMetalSp4InitScalars &sc [[buffer(0)]],
    device const bike::gpu::GpuSp3Config *configs [[buffer(1)]],
    device bike::gpu::GpuSp3WorldState *sp3_states [[buffer(2)]],
    device bike::gpu::GpuSp4WorldState *sp4_states [[buffer(3)]],
    device bike::gpu::GpuWorldState *world_states [[buffer(4)]],
    device const float *qpos_batch [[buffer(5)]],
    device float *tendon_range_batch [[buffer(6)]],
    uint gid [[thread_position_in_grid]]
) {
    const int w = static_cast<int>(gid);
    if (w >= sc.nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx = sc.idx;
    bike_metal_sp4_init_impl(
        w, idx, configs, sc.per_world_config, sp3_states, sp4_states,
        sc.has_world_states, world_states, sc.has_qpos, qpos_batch, sc.init_grade,
        sc.ntendon, sc.has_tendon_range, tendon_range_batch, sc.tendon_range_world_stride);
}

kernel void bike_gpu_sp4_pre_solve_and_pass0_metal_kernel(
    constant BikeMetalSp4PreSolveScalars &sc [[buffer(0)]],
    device const bike::physics_core::GpuTyreModel *tyre_model [[buffer(1)]],
    device const float *terrain_f32 [[buffer(2)]],
    device const bike::physics_core::i32 *terrain_i32 [[buffer(3)]],
    device const bike::gpu::GpuSp3Config *configs [[buffer(4)]],
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params [[buffer(5)]],
    device bike::gpu::GpuSp3WorldState *sp3_states [[buffer(6)]],
    device bike::gpu::GpuSp4WorldState *sp4_states [[buffer(7)]],
    device bike::gpu::GpuWorldState *world_states [[buffer(8)]],
    device const float *qpos_batch [[buffer(9)]],
    device const float *qvel_batch [[buffer(10)]],
    device const float *qfrc_bias_batch [[buffer(11)]],
    device const float *xpos_batch [[buffer(12)]],
    device const float *xmat_batch [[buffer(13)]],
    device const float *xanchor_batch [[buffer(14)]],
    device const float *xaxis_batch [[buffer(15)]],
    device const float *site_xpos_batch [[buffer(16)]],
    device const float *subtree_com_batch [[buffer(17)]],
    device const float *body_subtreemass [[buffer(18)]],
    device const int *M_rowadr [[buffer(19)]],
    device const int *M_rownnz [[buffer(20)]],
    device const int *M_colind [[buffer(21)]],
    device const float *d_M_batch [[buffer(22)]],
    device float *qfrc_batch [[buffer(23)]],
    device float *ctrl_batch [[buffer(24)]],
    device float *tendon_range_batch [[buffer(25)]],
    device float *dof_frictionloss_batch [[buffer(26)]],
    device float *qacc_warmstart_batch [[buffer(27)]],
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch [[buffer(28)]],
    uint gid [[thread_position_in_grid]]
) {
    const int w = static_cast<int>(gid);
    if (w >= sc.nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx = sc.idx;
    const bike::physics_core::GpuTerrainTableDesc terrain_desc = sc.terrain_desc;
    bike_metal_sp4_pre_solve_and_pass0_impl(
        w, sc.dt_s, idx, tyre_model, terrain_desc, terrain_f32, terrain_i32,
        configs, sc.per_world_config, spindle_params, sp3_states, sp4_states, world_states,
        qpos_batch, qvel_batch, qfrc_bias_batch,
        xpos_batch, xmat_batch, xanchor_batch, xaxis_batch,
        site_xpos_batch, subtree_com_batch, sc.has_body_subtreemass, body_subtreemass,
        sc.qM_row_stride, sc.M_world_stride, M_rowadr, M_rownnz, M_colind, d_M_batch,
        qfrc_batch, ctrl_batch, sc.ntendon, sc.has_tendon_range, tendon_range_batch,
        sc.tendon_range_world_stride, sc.has_dof_frictionloss, dof_frictionloss_batch,
        sc.dof_frictionloss_world_stride, sc.has_qacc_warmstart, qacc_warmstart_batch,
        sc.has_pneum_scratch, pneum_scratch);
}

kernel void bike_gpu_sp4_pass_stage_metal_kernel(
    constant BikeMetalSp4PassScalars &sc [[buffer(0)]],
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params [[buffer(1)]],
    device bike::gpu::GpuSp4WorldState *sp4_states [[buffer(2)]],
    device const float *qpos_batch [[buffer(3)]],
    device const float *qvel_batch [[buffer(4)]],
    device const float *qfrc_bias_batch [[buffer(5)]],
    device const float *qacc_batch [[buffer(6)]],
    device const int *M_rowadr [[buffer(7)]],
    device const int *M_rownnz [[buffer(8)]],
    device const int *M_colind [[buffer(9)]],
    device const float *d_M_batch [[buffer(10)]],
    device float *ctrl_batch [[buffer(11)]],
    device float *qacc_warmstart_batch [[buffer(12)]],
    uint gid [[thread_position_in_grid]]
) {
    const int w = static_cast<int>(gid);
    if (w >= sc.nworld) {
        return;
    }
    bike_metal_sp4_pass_stage_impl(
        sc.stage, w, sc.nq, sc.nv, sc.nu,
        spindle_params, sp4_states,
        qpos_batch, qvel_batch, qfrc_bias_batch, qacc_batch,
        sc.qM_row_stride, sc.M_world_stride, M_rowadr, M_rownnz, M_colind, d_M_batch,
        ctrl_batch, sc.has_qacc_warmstart, qacc_warmstart_batch);
}

kernel void bike_gpu_sp4_post_solve_metal_kernel(
    constant BikeMetalSp4PostSolveScalars &sc [[buffer(0)]],
    device const bike::gpu::GpuSp3Config *configs [[buffer(1)]],
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params [[buffer(2)]],
    device bike::gpu::GpuSp3WorldState *sp3_states [[buffer(3)]],
    device bike::gpu::GpuSp4WorldState *sp4_states [[buffer(4)]],
    device float *qpos_batch [[buffer(5)]],
    device const float *qvel_batch [[buffer(6)]],
    device const float *xpos_batch [[buffer(7)]],
    device const float *xmat_batch [[buffer(8)]],
    device const float *xanchor_batch [[buffer(9)]],
    device const float *sensordata_batch [[buffer(10)]],
    device const int *nefc_batch [[buffer(11)]],
    device const int *efc_type_batch [[buffer(12)]],
    device const int *efc_id_batch [[buffer(13)]],
    device const float *efc_J_batch [[buffer(14)]],
    device const float *efc_force_batch [[buffer(15)]],
    device const int *nacon_batch [[buffer(16)]],
    device const int *contact_worldid_batch [[buffer(17)]],
    device const int *contact_geom_batch [[buffer(18)]],
    device const int *contact_efc_address_batch [[buffer(19)]],
    device const int *contact_dim_batch [[buffer(20)]],
    uint gid [[thread_position_in_grid]]
) {
    const int w = static_cast<int>(gid);
    if (w >= sc.nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx = sc.idx;
    bike_metal_sp4_post_solve_impl(
        w, sc.dt_s, idx, configs, sc.per_world_config, spindle_params,
        sp3_states, sp4_states, qpos_batch, qvel_batch,
        sc.has_xpos, xpos_batch, xmat_batch, sc.has_xanchor, xanchor_batch,
        sc.has_sensordata, sc.nsensordata, sensordata_batch,
        sc.has_efc, nefc_batch, sc.njmax, sc.efc_J_row_stride,
        efc_type_batch, efc_id_batch, efc_J_batch, efc_force_batch,
        sc.has_contacts, nacon_batch, sc.nconmax, sc.contact_efc_stride,
        sc.has_contact_worldid, contact_worldid_batch, contact_geom_batch,
        contact_efc_address_batch, sc.has_contact_dim, contact_dim_batch);
}
)MSL";
    combined += R"MSL(
inline void bike_metal_sp4_m0_init_impl(
    int w,
    thread const bike::gpu::GpuModelIndices &idx,
    device const bike::gpu::GpuSp3Config *configs,
    int per_world_config,
    device bike::gpu::GpuSp3WorldState *sp3_states,
    device bike::gpu::GpuSp4WorldState *sp4_states,
    device bike::physics_core::SpindleM0Scratch<float> *m0_scratch,
    int has_world_states,
    device bike::gpu::GpuWorldState *world_states,
    int has_qpos,
    device const float *qpos_batch,
    float init_grade,
    int ntendon,
    int has_tendon_range,
    device float *tendon_range_batch,
    int tendon_range_world_stride
) {
    if (!bike::gpu::gpu_validate_model_indices(idx, ntendon) ||
        configs == nullptr || sp3_states == nullptr || sp4_states == nullptr ||
        m0_scratch == nullptr) {
        return;
    }
    const long w64 = static_cast<long>(w);
    device const bike::gpu::GpuSp3Config *cfg_w = &configs[per_world_config ? w : 0];
    device bike::gpu::GpuWorldState *ws_w =
        (has_world_states != 0 && world_states != nullptr)
            ? &world_states[w]
            : static_cast<device bike::gpu::GpuWorldState *>(nullptr);
    device const float *qpos_w =
        (has_qpos != 0 && qpos_batch != nullptr)
            ? (qpos_batch + w64 * idx.nq)
            : static_cast<device const float *>(nullptr);
    device float *tr_w = static_cast<device float *>(nullptr);
    if (has_tendon_range != 0 && tendon_range_batch != nullptr && ntendon > 0) {
        if (tendon_range_world_stride >= 2 * ntendon) {
            tr_w = tendon_range_batch + w64 * tendon_range_world_stride;
        } else if (tendon_range_world_stride == 0 && w == 0) {
            tr_w = tendon_range_batch;
        } else {
            return;
        }
    }
    (void)bike::gpu::gpu_sp4_m0_init_world(
        idx, cfg_w, &sp3_states[w], &sp4_states[w], &m0_scratch[w],
        ws_w, qpos_w, init_grade, ntendon, tr_w);
}

inline void bike_metal_sp4_m0_pre_solve_and_begin_impl(
    int w,
    float dt_s,
    thread const bike::gpu::GpuModelIndices &idx,
    device const bike::physics_core::GpuTyreModel *tyre_model,
    thread const bike::physics_core::GpuTerrainTableDesc &terrain_desc,
    device const float *terrain_f32,
    device const bike::physics_core::i32 *terrain_i32,
    device const bike::gpu::GpuSp3Config *configs,
    int per_world_config,
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params,
    device bike::gpu::GpuSp3WorldState *sp3_states,
    device bike::gpu::GpuSp4WorldState *sp4_states,
    device bike::physics_core::SpindleM0Scratch<float> *m0_scratch,
    device bike::gpu::GpuWorldState *world_states,
    int has_pneum_scratch,
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch,
    device const float *qpos_batch,
    device const float *qvel_batch,
    device const float *qfrc_bias_batch,
    device const float *xpos_batch,
    device const float *xmat_batch,
    device const float *xanchor_batch,
    device const float *xaxis_batch,
    device const float *site_xpos_batch,
    device const float *subtree_com_batch,
    int has_body_subtreemass,
    device const float *body_subtreemass,
    int qM_row_stride,
    int M_world_stride,
    device const int *M_rowadr,
    device const int *M_rownnz,
    device const int *M_colind,
    device const float *d_M_batch,
    device float *qfrc_batch,
    device float *ctrl_batch,
    int ntendon,
    int has_tendon_range,
    device float *tendon_range_batch,
    int tendon_range_world_stride,
    int has_dof_frictionloss,
    device float *dof_frictionloss_batch,
    int dof_frictionloss_world_stride,
    int has_qacc_warmstart,
    device float *qacc_warmstart_batch,
    device int *any_needs_solve
) {
    if (!bike::gpu::gpu_validate_model_indices(idx, ntendon) || tyre_model == nullptr ||
        configs == nullptr || spindle_params == nullptr ||
        sp3_states == nullptr || sp4_states == nullptr || m0_scratch == nullptr ||
        world_states == nullptr || M_world_stride <= 0) {
        return;
    }
    float qpos_w[bike::gpu::kMaxNq];
    float qvel_w[bike::gpu::kMaxNv];
    float qfrc_w[bike::gpu::kMaxNv];
    float ctrl_w[bike::gpu::kMaxNu];

    const int nq = idx.nq;
    const int nv = idx.nv;
    const int nu = idx.nu;
    const long w64 = static_cast<long>(w);

    device const float *qpos_src = qpos_batch + w64 * nq;
    device const float *qvel_src = qvel_batch + w64 * nv;
    for (int i = 0; i < nq; ++i) qpos_w[i] = qpos_src[i];
    for (int i = 0; i < nv; ++i) qvel_w[i] = qvel_src[i];

    device const bike::gpu::GpuSp3Config *cfg_w = &configs[per_world_config ? w : 0];
    device float *tendon_w = static_cast<device float *>(nullptr);
    if (has_tendon_range != 0 && tendon_range_batch != nullptr && ntendon > 0) {
        if (tendon_range_world_stride >= 2 * ntendon) {
            tendon_w = tendon_range_batch + w64 * tendon_range_world_stride;
        } else if (tendon_range_world_stride == 0 && w == 0) {
            tendon_w = tendon_range_batch;
        } else {
            return;
        }
    }
    device float *fl_w = static_cast<device float *>(nullptr);
    if (has_dof_frictionloss != 0 && dof_frictionloss_batch != nullptr) {
        if (dof_frictionloss_world_stride >= nv) {
            fl_w = dof_frictionloss_batch + w64 * dof_frictionloss_world_stride;
        } else if (dof_frictionloss_world_stride == 0 && w == 0) {
            fl_w = dof_frictionloss_batch;
        } else {
            return;
        }
    }
    device const float *submass_ptr =
        (has_body_subtreemass != 0 && body_subtreemass != nullptr)
            ? body_subtreemass
            : static_cast<device const float *>(nullptr);
    device float *ws_w =
        (has_qacc_warmstart != 0 && qacc_warmstart_batch != nullptr)
            ? (qacc_warmstart_batch + w64 * nv)
            : static_cast<device float *>(nullptr);
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_w =
        (has_pneum_scratch != 0 && pneum_scratch != nullptr)
            ? &pneum_scratch[w]
            : static_cast<device bike::physics_core::GpuPneumaticStepScratch<float> *>(nullptr);

    bike::physics_core::u8 needs_solve = 0;
    bike::physics_core::u8 done = 1;
    const bike::physics_core::Status st = bike::gpu::gpu_sp4_m0_pre_solve_and_begin_world(
        w,
        dt_s,
        idx,
        tyre_model,
        terrain_desc,
        terrain_f32,
        terrain_i32,
        cfg_w,
        spindle_params,
        &sp3_states[w],
        &sp4_states[w],
        &m0_scratch[w],
        world_states[w].compliant_wheels,
        &world_states[w].distributed_wheels[0][0],
        qpos_w,
        qvel_w,
        qfrc_bias_batch + w64 * nv,
        xpos_batch + w64 * idx.nbody * 3,
        xmat_batch + w64 * idx.nbody * 9,
        xanchor_batch + w64 * tyre_model->njnt * 3,
        xaxis_batch + w64 * tyre_model->njnt * 3,
        site_xpos_batch + w64 * spindle_params->nsite * 3,
        subtree_com_batch + w64 * idx.nbody * 3,
        submass_ptr,
        qM_row_stride,
        M_rowadr,
        M_rownnz,
        M_colind,
        d_M_batch + w64 * M_world_stride,
        qfrc_w,
        ctrl_w,
        ntendon,
        tendon_w,
        fl_w,
        ws_w,
        needs_solve,
        done,
        world_states[w].pneumatic_wheels,
        pneum_w);
    if (st != bike::physics_core::Status::ok) {
        return;
    }

    device float *qfrc_dst = qfrc_batch + w64 * nv;
    device float *ctrl_dst = ctrl_batch + w64 * nu;
    for (int i = 0; i < nv; ++i) qfrc_dst[i] = qfrc_w[i];
    for (int i = 0; i < nu; ++i) ctrl_dst[i] = ctrl_w[i];
    if (needs_solve != 0 && any_needs_solve != nullptr) {
        atomic_fetch_or_explicit((device atomic_int *)any_needs_solve, 1, memory_order_relaxed);
    }
}

inline void bike_metal_sp4_m0_step_impl(
    int w,
    thread const bike::gpu::GpuModelIndices &idx,
    device const bike::gpu::GpuSp3Config *configs,
    int per_world_config,
    device const bike::gpu::GpuSp3WorldState *sp3_states,
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params,
    device bike::gpu::GpuSp4WorldState *sp4_states,
    device bike::physics_core::SpindleM0Scratch<float> *m0_scratch,
    device const float *qpos_batch,
    device const float *qvel_batch,
    device const float *qfrc_bias_batch,
    device const float *qacc_batch,
    int nq,
    int nv,
    int nu,
    int qM_row_stride,
    int M_world_stride,
    device const int *M_rowadr,
    device const int *M_rownnz,
    device const int *M_colind,
    device const float *d_M_batch,
    device float *ctrl_batch,
    int has_qacc_warmstart,
    device float *qacc_warmstart_batch,
    device int *any_needs_solve
) {
    if (!bike::gpu::gpu_validate_model_indices(idx) || configs == nullptr ||
        sp3_states == nullptr || spindle_params == nullptr ||
        sp4_states == nullptr || m0_scratch == nullptr || M_world_stride <= 0) {
        return;
    }
    const long w64 = static_cast<long>(w);
    device const bike::gpu::GpuSp3Config *cfg_w = &configs[per_world_config ? w : 0];
    device float *ws_w =
        (has_qacc_warmstart != 0 && qacc_warmstart_batch != nullptr)
            ? (qacc_warmstart_batch + w64 * nv)
            : static_cast<device float *>(nullptr);
    bike::physics_core::u8 needs_solve = 0;
    bike::physics_core::u8 done = 1;
    const bike::physics_core::Status st = bike::gpu::gpu_sp4_m0_step_world(
        w,
        idx,
        cfg_w,
        spindle_params,
        &sp3_states[w],
        &sp4_states[w],
        &m0_scratch[w],
        qpos_batch + w64 * nq,
        qvel_batch + w64 * nv,
        qfrc_bias_batch + w64 * nv,
        qacc_batch + w64 * nv,
        qM_row_stride,
        M_rowadr,
        M_rownnz,
        M_colind,
        d_M_batch + w64 * M_world_stride,
        ctrl_batch + w64 * nu,
        ws_w,
        needs_solve,
        done);
    if (st == bike::physics_core::Status::ok && needs_solve != 0 && any_needs_solve != nullptr) {
        atomic_fetch_or_explicit((device atomic_int *)any_needs_solve, 1, memory_order_relaxed);
    }
}

inline void bike_metal_sp4_m0_post_solve_impl(
    int w,
    float dt_s,
    thread const bike::gpu::GpuModelIndices &idx,
    device const bike::gpu::GpuSp3Config *configs,
    int per_world_config,
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params,
    device bike::gpu::GpuSp3WorldState *sp3_states,
    device bike::gpu::GpuSp4WorldState *sp4_states,
    device const bike::physics_core::SpindleM0Scratch<float> *m0_scratch,
    device float *qpos_batch,
    device const float *qvel_batch,
    int has_xpos,
    device const float *xpos_batch,
    device const float *xmat_batch,
    int has_xanchor,
    device const float *xanchor_batch,
    int has_sensordata,
    int nsensordata,
    device const float *sensordata_batch,
    int has_efc,
    device const int *nefc_batch,
    int njmax,
    int efc_J_row_stride,
    device const int *efc_type_batch,
    device const int *efc_id_batch,
    device const float *efc_J_batch,
    device const float *efc_force_batch,
    int has_contacts,
    device const int *nacon_batch,
    int nconmax,
    int contact_efc_stride,
    int has_contact_worldid,
    device const int *contact_worldid_batch,
    device const int *contact_geom_batch,
    device const int *contact_efc_address_batch,
    int has_contact_dim,
    device const int *contact_dim_batch
) {
    if (!bike::gpu::gpu_validate_model_indices(idx) || configs == nullptr ||
        spindle_params == nullptr || sp3_states == nullptr ||
        sp4_states == nullptr || m0_scratch == nullptr) {
        return;
    }
    const long w64 = static_cast<long>(w);
    device const bike::gpu::GpuSp3Config *cfg_w = &configs[per_world_config ? w : 0];
    device const float *sd_w =
        (has_sensordata != 0 && sensordata_batch != nullptr && nsensordata > 0)
            ? (sensordata_batch + w64 * nsensordata)
            : static_cast<device const float *>(nullptr);
    int nefc_w = 0;
    device const int *efc_type_w = static_cast<device const int *>(nullptr);
    device const int *efc_id_w = static_cast<device const int *>(nullptr);
    device const float *efc_J_w = static_cast<device const float *>(nullptr);
    device const float *efc_force_w = static_cast<device const float *>(nullptr);
    if (has_efc != 0 && nefc_batch != nullptr && njmax > 0) {
        nefc_w = nefc_batch[w];
        if (nefc_w < 0) nefc_w = 0;
        if (nefc_w > njmax) nefc_w = njmax;
        efc_type_w = (efc_type_batch != nullptr) ? (efc_type_batch + w64 * njmax)
                                                 : static_cast<device const int *>(nullptr);
        efc_id_w = (efc_id_batch != nullptr) ? (efc_id_batch + w64 * njmax)
                                             : static_cast<device const int *>(nullptr);
        efc_force_w = (efc_force_batch != nullptr) ? (efc_force_batch + w64 * njmax)
                                                   : static_cast<device const float *>(nullptr);
        efc_J_w = (efc_J_batch != nullptr && efc_J_row_stride > 0)
                      ? (efc_J_batch + w64 * static_cast<long>(njmax) * efc_J_row_stride)
                      : static_cast<device const float *>(nullptr);
    }
    int ncon_or_total = 0;
    device const int *c_wid = static_cast<device const int *>(nullptr);
    device const int *c_geom = static_cast<device const int *>(nullptr);
    device const int *c_adr = static_cast<device const int *>(nullptr);
    device const int *c_dim = static_cast<device const int *>(nullptr);
    if (has_contacts != 0 && nacon_batch != nullptr && nconmax > 0) {
        if (has_contact_worldid != 0 && contact_worldid_batch != nullptr) {
            ncon_or_total = nacon_batch[0];
            if (ncon_or_total < 0) ncon_or_total = 0;
            if (ncon_or_total > nconmax) ncon_or_total = nconmax;
            c_wid = contact_worldid_batch;
            c_geom = contact_geom_batch;
            c_adr = contact_efc_address_batch;
            c_dim = (has_contact_dim != 0) ? contact_dim_batch
                                           : static_cast<device const int *>(nullptr);
        } else {
            ncon_or_total = nacon_batch[w];
            if (ncon_or_total < 0) ncon_or_total = 0;
            if (ncon_or_total > nconmax) ncon_or_total = nconmax;
            c_geom = (contact_geom_batch != nullptr) ? (contact_geom_batch + w64 * 2 * nconmax)
                                                     : static_cast<device const int *>(nullptr);
            c_adr = (contact_efc_address_batch != nullptr)
                        ? (contact_efc_address_batch + w64 * nconmax * contact_efc_stride)
                        : static_cast<device const int *>(nullptr);
            c_dim = (has_contact_dim != 0 && contact_dim_batch != nullptr)
                        ? (contact_dim_batch + w64 * nconmax)
                        : static_cast<device const int *>(nullptr);
        }
    }
    const int nj = spindle_params->njnt;
    device const float *xpos_w =
        (has_xpos != 0 && xpos_batch != nullptr)
            ? (xpos_batch + w64 * idx.nbody * 3)
            : static_cast<device const float *>(nullptr);
    device const float *xanchor_w =
        (has_xanchor != 0 && xanchor_batch != nullptr && nj > 0)
            ? (xanchor_batch + w64 * nj * 3)
            : static_cast<device const float *>(nullptr);

    (void)bike::gpu::gpu_sp4_post_solve_world(
        w,
        dt_s,
        idx,
        cfg_w,
        spindle_params,
        &sp3_states[w],
        &sp4_states[w],
        qpos_batch + w64 * idx.nq,
        qvel_batch + w64 * idx.nv,
        xpos_w,
        (xmat_batch != nullptr) ? (xmat_batch + w64 * idx.nbody * 9)
                                : static_cast<device const float *>(nullptr),
        xanchor_w,
        nsensordata,
        sd_w,
        nefc_w,
        efc_J_row_stride,
        efc_type_w,
        efc_id_w,
        efc_J_w,
        efc_force_w,
        ncon_or_total,
        contact_efc_stride,
        c_wid,
        c_geom,
        c_adr,
        c_dim,
        &m0_scratch[w]);
}

inline void bike_metal_sp4_energy_post_step_impl(
    int w,
    float dt_s,
    thread const bike::gpu::GpuModelIndices &idx,
    device const bike::physics_core::GpuTyreModel *tyre_model,
    thread const bike::physics_core::GpuTerrainTableDesc &terrain_desc,
    device const float *terrain_f32,
    device const bike::gpu::GpuSp3Config *configs,
    int per_world_config,
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params,
    device bike::gpu::GpuSp3WorldState *sp3_states,
    device bike::gpu::GpuSp4WorldState *sp4_states,
    device const bike::gpu::GpuWorldState *world_states,
    int has_pneum_scratch,
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch,
    int qM_row_stride,
    int M_world_stride,
    device const int *M_rowadr,
    device const int *M_rownnz,
    device const int *M_colind,
    device const float *d_M_batch,
    device const float *qpos_batch,
    device const float *qvel_batch,
    device const float *xpos_batch,
    device const float *ctrl_batch,
    device const float *body_mass,
    device const float *xipos_batch,
    device const float *gravity,
    int has_efc,
    device const int *nefc_batch,
    int njmax,
    int efc_J_row_stride,
    device const int *efc_type_batch,
    device const float *efc_J_batch,
    device const float *efc_force_batch
) {
    if (!bike::gpu::gpu_validate_model_indices(idx) || configs == nullptr ||
        spindle_params == nullptr || sp3_states == nullptr ||
        sp4_states == nullptr || world_states == nullptr || M_world_stride <= 0) {
        return;
    }
    const long w64 = static_cast<long>(w);
    device const bike::gpu::GpuSp3Config *cfg_w = &configs[per_world_config ? w : 0];
    int nefc_w = 0;
    device const int *efc_type_w = static_cast<device const int *>(nullptr);
    device const float *efc_J_w = static_cast<device const float *>(nullptr);
    device const float *efc_force_w = static_cast<device const float *>(nullptr);
    if (has_efc != 0 && nefc_batch != nullptr && njmax > 0) {
        nefc_w = nefc_batch[w];
        if (nefc_w < 0) nefc_w = 0;
        if (nefc_w > njmax) nefc_w = njmax;
        efc_type_w = (efc_type_batch != nullptr) ? (efc_type_batch + w64 * njmax)
                                                 : static_cast<device const int *>(nullptr);
        efc_force_w = (efc_force_batch != nullptr) ? (efc_force_batch + w64 * njmax)
                                                   : static_cast<device const float *>(nullptr);
        efc_J_w = (efc_J_batch != nullptr && efc_J_row_stride > 0)
                      ? (efc_J_batch + w64 * static_cast<long>(njmax) * efc_J_row_stride)
                      : static_cast<device const float *>(nullptr);
    }
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_w =
        (has_pneum_scratch != 0 && pneum_scratch != nullptr)
            ? &pneum_scratch[w]
            : static_cast<device bike::physics_core::GpuPneumaticStepScratch<float> *>(nullptr);
    device const float *xipos_w = xipos_batch + w64 * idx.nbody * 3;
    device const float *xpos_w =
        (xpos_batch != nullptr) ? (xpos_batch + w64 * idx.nbody * 3) : xipos_w;
    device const float *ctrl_w =
        (ctrl_batch != nullptr) ? (ctrl_batch + w64 * idx.nu)
                                : static_cast<device const float *>(nullptr);

    (void)bike::gpu::gpu_sp4_energy_post_step_world(
        dt_s,
        idx,
        tyre_model,
        terrain_desc,
        terrain_f32,
        cfg_w,
        spindle_params,
        &sp3_states[w],
        &sp4_states[w],
        &world_states[w],
        pneum_w,
        qM_row_stride,
        M_rowadr,
        M_rownnz,
        M_colind,
        d_M_batch + w64 * M_world_stride,
        qpos_batch + w64 * idx.nq,
        qvel_batch + w64 * idx.nv,
        xpos_w,
        ctrl_w,
        body_mass,
        xipos_w,
        gravity,
        nefc_w,
        efc_J_row_stride,
        efc_type_w,
        efc_J_w,
        efc_force_w);
}

struct BikeWpSp4M0InitArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::gpu::GpuSp3Config *configs;
    device bike::gpu::GpuSp3WorldState *sp3_states;
    device bike::gpu::GpuSp4WorldState *sp4_states;
    device bike::physics_core::SpindleM0Scratch<float> *m0_scratch;
    device bike::gpu::GpuWorldState *world_states;
    device const float *qpos;
    device float *tendon_range;
    int nworld;
    int per_world_config;
    float init_grade;
    int ntendon;
    int tendon_range_world_stride;
    int has_world_states;
    int has_qpos;
    int has_tendon_range;
};

struct BikeWpSp4M0PreSolveArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::physics_core::GpuTyreModel *tyre_model;
    device const bike::physics_core::GpuTerrainTableDesc *terrain_desc;
    device const float *terrain_f32;
    device const bike::physics_core::i32 *terrain_i32;
    device const bike::gpu::GpuSp3Config *configs;
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params;
    device bike::gpu::GpuSp3WorldState *sp3_states;
    device bike::gpu::GpuSp4WorldState *sp4_states;
    device bike::physics_core::SpindleM0Scratch<float> *m0_scratch;
    device bike::gpu::GpuWorldState *world_states;
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch;
    device const float *qpos;
    device const float *qvel;
    device const float *qfrc_bias;
    device const float *xpos;
    device const float *xmat;
    device const float *xanchor;
    device const float *xaxis;
    device const float *site_xpos;
    device const float *subtree_com;
    device const float *body_subtreemass;
    device const int *M_rowadr;
    device const int *M_rownnz;
    device const int *M_colind;
    device const float *d_M;
    device float *qfrc_applied;
    device float *ctrl;
    device float *tendon_range;
    device float *dof_frictionloss;
    device float *qacc_warmstart;
    device int *any_needs_solve;
    int nworld;
    float dt_s;
    int per_world_config;
    int qM_row_stride;
    int M_world_stride;
    int ntendon;
    int tendon_range_world_stride;
    int dof_frictionloss_world_stride;
    int has_body_subtreemass;
    int has_tendon_range;
    int has_dof_frictionloss;
    int has_qacc_warmstart;
    int has_pneum_scratch;
};

struct BikeWpSp4M0StepArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::gpu::GpuSp3Config *configs;
    device const bike::gpu::GpuSp3WorldState *sp3_states;
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params;
    device bike::gpu::GpuSp4WorldState *sp4_states;
    device bike::physics_core::SpindleM0Scratch<float> *m0_scratch;
    device const float *qpos;
    device const float *qvel;
    device const float *qfrc_bias;
    device const float *qacc;
    device const int *M_rowadr;
    device const int *M_rownnz;
    device const int *M_colind;
    device const float *d_M;
    device float *ctrl;
    device float *qacc_warmstart;
    device int *any_needs_solve;
    int nworld;
    int nq;
    int nv;
    int nu;
    int per_world_config;
    int qM_row_stride;
    int M_world_stride;
    int has_qacc_warmstart;
};

struct BikeWpSp4M0PostSolveArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::gpu::GpuSp3Config *configs;
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params;
    device bike::gpu::GpuSp3WorldState *sp3_states;
    device bike::gpu::GpuSp4WorldState *sp4_states;
    device const bike::physics_core::SpindleM0Scratch<float> *m0_scratch;
    device float *qpos;
    device const float *qvel;
    device const float *xpos;
    device const float *xmat;
    device const float *xanchor;
    device const float *sensordata;
    device const int *nefc;
    device const int *efc_type;
    device const int *efc_id;
    device const float *efc_J;
    device const float *efc_force;
    device const int *nacon;
    device const int *contact_worldid;
    device const int *contact_geom;
    device const int *contact_efc_address;
    device const int *contact_dim;
    int nworld;
    float dt_s;
    int per_world_config;
    int nsensordata;
    int njmax;
    int efc_J_row_stride;
    int nconmax;
    int contact_efc_stride;
    int has_xpos;
    int has_xanchor;
    int has_sensordata;
    int has_efc;
    int has_contacts;
    int has_contact_worldid;
    int has_contact_dim;
};

struct BikeWpSp4EnergyArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::physics_core::GpuTyreModel *tyre_model;
    device const bike::physics_core::GpuTerrainTableDesc *terrain_desc;
    device const float *terrain_f32;
    device const bike::gpu::GpuSp3Config *configs;
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params;
    device bike::gpu::GpuSp3WorldState *sp3_states;
    device bike::gpu::GpuSp4WorldState *sp4_states;
    device const bike::gpu::GpuWorldState *world_states;
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch;
    device const int *M_rowadr;
    device const int *M_rownnz;
    device const int *M_colind;
    device const float *d_M;
    device const float *qpos;
    device const float *qvel;
    device const float *xpos;
    device const float *ctrl;
    device const float *body_mass;
    device const float *xipos;
    device const float *gravity;
    device const int *nefc;
    device const int *efc_type;
    device const float *efc_J;
    device const float *efc_force;
    int nworld;
    float dt_s;
    int per_world_config;
    int qM_row_stride;
    int M_world_stride;
    int njmax;
    int efc_J_row_stride;
    int has_pneum_scratch;
    int has_efc;
};

kernel void bike_wp_sp4_m0_init_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpSp4M0InitArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    bike_metal_sp4_m0_init_impl(
        w, idx, args->configs, args->per_world_config,
        args->sp3_states, args->sp4_states, args->m0_scratch,
        args->has_world_states, args->world_states,
        args->has_qpos, args->qpos, args->init_grade, args->ntendon,
        args->has_tendon_range, args->tendon_range, args->tendon_range_world_stride);
}

kernel void bike_wp_sp4_m0_pre_solve_and_begin_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpSp4M0PreSolveArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld || args->terrain_desc == nullptr) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    const bike::physics_core::GpuTerrainTableDesc terrain_desc = *(args->terrain_desc);
    bike_metal_sp4_m0_pre_solve_and_begin_impl(
        w, args->dt_s, idx, args->tyre_model, terrain_desc, args->terrain_f32, args->terrain_i32,
        args->configs, args->per_world_config, args->spindle_params,
        args->sp3_states, args->sp4_states, args->m0_scratch, args->world_states,
        args->has_pneum_scratch, args->pneum_scratch,
        args->qpos, args->qvel, args->qfrc_bias,
        args->xpos, args->xmat, args->xanchor, args->xaxis,
        args->site_xpos, args->subtree_com, args->has_body_subtreemass, args->body_subtreemass,
        args->qM_row_stride, args->M_world_stride,
        args->M_rowadr, args->M_rownnz, args->M_colind, args->d_M,
        args->qfrc_applied, args->ctrl,
        args->ntendon, args->has_tendon_range, args->tendon_range, args->tendon_range_world_stride,
        args->has_dof_frictionloss, args->dof_frictionloss, args->dof_frictionloss_world_stride,
        args->has_qacc_warmstart, args->qacc_warmstart, args->any_needs_solve);
}

kernel void bike_wp_sp4_m0_step_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpSp4M0StepArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    bike_metal_sp4_m0_step_impl(
        w, idx, args->configs, args->per_world_config, args->sp3_states,
        args->spindle_params, args->sp4_states, args->m0_scratch,
        args->qpos, args->qvel, args->qfrc_bias, args->qacc,
        args->nq, args->nv, args->nu,
        args->qM_row_stride, args->M_world_stride,
        args->M_rowadr, args->M_rownnz, args->M_colind, args->d_M,
        args->ctrl, args->has_qacc_warmstart, args->qacc_warmstart,
        args->any_needs_solve);
}

kernel void bike_wp_sp4_m0_post_solve_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpSp4M0PostSolveArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    bike_metal_sp4_m0_post_solve_impl(
        w, args->dt_s, idx, args->configs, args->per_world_config, args->spindle_params,
        args->sp3_states, args->sp4_states, args->m0_scratch,
        args->qpos, args->qvel, args->has_xpos, args->xpos, args->xmat,
        args->has_xanchor, args->xanchor,
        args->has_sensordata, args->nsensordata, args->sensordata,
        args->has_efc, args->nefc, args->njmax, args->efc_J_row_stride,
        args->efc_type, args->efc_id, args->efc_J, args->efc_force,
        args->has_contacts, args->nacon, args->nconmax, args->contact_efc_stride,
        args->has_contact_worldid, args->contact_worldid, args->contact_geom,
        args->contact_efc_address, args->has_contact_dim, args->contact_dim);
}

kernel void bike_wp_sp4_energy_post_step_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpSp4EnergyArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid);
    if (w >= args->nworld || args->terrain_desc == nullptr) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    const bike::physics_core::GpuTerrainTableDesc terrain_desc = *(args->terrain_desc);
    bike_metal_sp4_energy_post_step_impl(
        w, args->dt_s, idx, args->tyre_model, terrain_desc, args->terrain_f32,
        args->configs, args->per_world_config, args->spindle_params,
        args->sp3_states, args->sp4_states, args->world_states,
        args->has_pneum_scratch, args->pneum_scratch,
        args->qM_row_stride, args->M_world_stride,
        args->M_rowadr, args->M_rownnz, args->M_colind, args->d_M,
        args->qpos, args->qvel, args->xpos, args->ctrl,
        args->body_mass, args->xipos, args->gravity,
        args->has_efc, args->nefc, args->njmax, args->efc_J_row_stride,
        args->efc_type, args->efc_J, args->efc_force);
}

struct BikeMetalSp4M0StepScalars {
    bike::gpu::GpuModelIndices idx;
    int nworld;
    int nq;
    int nv;
    int nu;
    int per_world_config;
    int qM_row_stride;
    int M_world_stride;
    int has_qacc_warmstart;
};

struct BikeMetalSp4EnergyScalars {
    bike::gpu::GpuModelIndices idx;
    bike::physics_core::GpuTerrainTableDesc terrain_desc;
    int nworld;
    float dt_s;
    int per_world_config;
    int qM_row_stride;
    int M_world_stride;
    int njmax;
    int efc_J_row_stride;
    int has_pneum_scratch;
    int has_efc;
    int has_ctrl;
};

kernel void bike_gpu_sp4_m0_init_metal_kernel(
    constant BikeMetalSp4InitScalars &sc [[buffer(0)]],
    device const bike::gpu::GpuSp3Config *configs [[buffer(1)]],
    device bike::gpu::GpuSp3WorldState *sp3_states [[buffer(2)]],
    device bike::gpu::GpuSp4WorldState *sp4_states [[buffer(3)]],
    device bike::physics_core::SpindleM0Scratch<float> *m0_scratch [[buffer(4)]],
    device bike::gpu::GpuWorldState *world_states [[buffer(5)]],
    device const float *qpos_batch [[buffer(6)]],
    device float *tendon_range_batch [[buffer(7)]],
    uint gid [[thread_position_in_grid]]
) {
    const int w = static_cast<int>(gid);
    if (w >= sc.nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx = sc.idx;
    bike_metal_sp4_m0_init_impl(
        w, idx, configs, sc.per_world_config, sp3_states, sp4_states, m0_scratch,
        sc.has_world_states, world_states, sc.has_qpos, qpos_batch, sc.init_grade,
        sc.ntendon, sc.has_tendon_range, tendon_range_batch, sc.tendon_range_world_stride);
}

kernel void bike_gpu_sp4_m0_pre_solve_and_begin_metal_kernel(
    constant BikeMetalSp4PreSolveScalars &sc [[buffer(0)]],
    device const bike::physics_core::GpuTyreModel *tyre_model [[buffer(1)]],
    device const float *terrain_f32 [[buffer(2)]],
    device const bike::physics_core::i32 *terrain_i32 [[buffer(3)]],
    device const bike::gpu::GpuSp3Config *configs [[buffer(4)]],
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params [[buffer(5)]],
    device bike::gpu::GpuSp3WorldState *sp3_states [[buffer(6)]],
    device bike::gpu::GpuSp4WorldState *sp4_states [[buffer(7)]],
    device bike::physics_core::SpindleM0Scratch<float> *m0_scratch [[buffer(8)]],
    device bike::gpu::GpuWorldState *world_states [[buffer(9)]],
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch [[buffer(10)]],
    device const float *qpos_batch [[buffer(11)]],
    device const float *qvel_batch [[buffer(12)]],
    device const float *qfrc_bias_batch [[buffer(13)]],
    device const float *xpos_batch [[buffer(14)]],
    device const float *xmat_batch [[buffer(15)]],
    device const float *xanchor_batch [[buffer(16)]],
    device const float *xaxis_batch [[buffer(17)]],
    device const float *site_xpos_batch [[buffer(18)]],
    device const float *subtree_com_batch [[buffer(19)]],
    device const float *body_subtreemass [[buffer(20)]],
    device const int *M_rowadr [[buffer(21)]],
    device const int *M_rownnz [[buffer(22)]],
    device const int *M_colind [[buffer(23)]],
    device const float *d_M_batch [[buffer(24)]],
    device float *qfrc_batch [[buffer(25)]],
    device float *ctrl_batch [[buffer(26)]],
    device float *tendon_range_batch [[buffer(27)]],
    device float *dof_frictionloss_batch [[buffer(28)]],
    device float *qacc_warmstart_batch [[buffer(29)]],
    device int *any_needs_solve [[buffer(30)]],
    uint gid [[thread_position_in_grid]]
) {
    const int w = static_cast<int>(gid);
    if (w >= sc.nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx = sc.idx;
    const bike::physics_core::GpuTerrainTableDesc td = sc.terrain_desc;
    bike_metal_sp4_m0_pre_solve_and_begin_impl(
        w, sc.dt_s, idx, tyre_model, td, terrain_f32, terrain_i32,
        configs, sc.per_world_config, spindle_params, sp3_states, sp4_states,
        m0_scratch, world_states, sc.has_pneum_scratch, pneum_scratch,
        qpos_batch, qvel_batch, qfrc_bias_batch,
        xpos_batch, xmat_batch, xanchor_batch, xaxis_batch,
        site_xpos_batch, subtree_com_batch, sc.has_body_subtreemass, body_subtreemass,
        sc.qM_row_stride, sc.M_world_stride, M_rowadr, M_rownnz, M_colind, d_M_batch,
        qfrc_batch, ctrl_batch, sc.ntendon, sc.has_tendon_range, tendon_range_batch,
        sc.tendon_range_world_stride, sc.has_dof_frictionloss, dof_frictionloss_batch,
        sc.dof_frictionloss_world_stride, sc.has_qacc_warmstart, qacc_warmstart_batch,
        any_needs_solve);
}

kernel void bike_gpu_sp4_m0_step_metal_kernel(
    constant BikeMetalSp4M0StepScalars &sc [[buffer(0)]],
    device const bike::gpu::GpuSp3Config *configs [[buffer(1)]],
    device const bike::gpu::GpuSp3WorldState *sp3_states [[buffer(2)]],
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params [[buffer(3)]],
    device bike::gpu::GpuSp4WorldState *sp4_states [[buffer(4)]],
    device bike::physics_core::SpindleM0Scratch<float> *m0_scratch [[buffer(5)]],
    device const float *qpos_batch [[buffer(6)]],
    device const float *qvel_batch [[buffer(7)]],
    device const float *qfrc_bias_batch [[buffer(8)]],
    device const float *qacc_batch [[buffer(9)]],
    device const int *M_rowadr [[buffer(10)]],
    device const int *M_rownnz [[buffer(11)]],
    device const int *M_colind [[buffer(12)]],
    device const float *d_M_batch [[buffer(13)]],
    device float *ctrl_batch [[buffer(14)]],
    device float *qacc_warmstart_batch [[buffer(15)]],
    device int *any_needs_solve [[buffer(16)]],
    uint gid [[thread_position_in_grid]]
) {
    const int w = static_cast<int>(gid);
    if (w >= sc.nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx = sc.idx;
    bike_metal_sp4_m0_step_impl(
        w, idx, configs, sc.per_world_config, sp3_states,
        spindle_params, sp4_states, m0_scratch,
        qpos_batch, qvel_batch, qfrc_bias_batch, qacc_batch,
        sc.nq, sc.nv, sc.nu,
        sc.qM_row_stride, sc.M_world_stride,
        M_rowadr, M_rownnz, M_colind, d_M_batch,
        ctrl_batch, sc.has_qacc_warmstart, qacc_warmstart_batch,
        any_needs_solve);
}

kernel void bike_gpu_sp4_m0_post_solve_metal_kernel(
    constant BikeMetalSp4PostSolveScalars &sc [[buffer(0)]],
    device const bike::gpu::GpuSp3Config *configs [[buffer(1)]],
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params [[buffer(2)]],
    device bike::gpu::GpuSp3WorldState *sp3_states [[buffer(3)]],
    device bike::gpu::GpuSp4WorldState *sp4_states [[buffer(4)]],
    device const bike::physics_core::SpindleM0Scratch<float> *m0_scratch [[buffer(5)]],
    device float *qpos_batch [[buffer(6)]],
    device const float *qvel_batch [[buffer(7)]],
    device const float *xpos_batch [[buffer(8)]],
    device const float *xmat_batch [[buffer(9)]],
    device const float *xanchor_batch [[buffer(10)]],
    device const float *sensordata_batch [[buffer(11)]],
    device const int *nefc_batch [[buffer(12)]],
    device const int *efc_type_batch [[buffer(13)]],
    device const int *efc_id_batch [[buffer(14)]],
    device const float *efc_J_batch [[buffer(15)]],
    device const float *efc_force_batch [[buffer(16)]],
    device const int *nacon_batch [[buffer(17)]],
    device const int *contact_worldid_batch [[buffer(18)]],
    device const int *contact_geom_batch [[buffer(19)]],
    device const int *contact_efc_address_batch [[buffer(20)]],
    device const int *contact_dim_batch [[buffer(21)]],
    uint gid [[thread_position_in_grid]]
) {
    const int w = static_cast<int>(gid);
    if (w >= sc.nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx = sc.idx;
    bike_metal_sp4_m0_post_solve_impl(
        w, sc.dt_s, idx, configs, sc.per_world_config, spindle_params,
        sp3_states, sp4_states, m0_scratch, qpos_batch, qvel_batch,
        sc.has_xpos, xpos_batch, xmat_batch, sc.has_xanchor, xanchor_batch,
        sc.has_sensordata, sc.nsensordata, sensordata_batch,
        sc.has_efc, nefc_batch, sc.njmax, sc.efc_J_row_stride,
        efc_type_batch, efc_id_batch, efc_J_batch, efc_force_batch,
        sc.has_contacts, nacon_batch, sc.nconmax, sc.contact_efc_stride,
        sc.has_contact_worldid, contact_worldid_batch, contact_geom_batch,
        contact_efc_address_batch, sc.has_contact_dim, contact_dim_batch);
}

kernel void bike_gpu_sp4_energy_post_step_metal_kernel(
    constant BikeMetalSp4EnergyScalars &sc [[buffer(0)]],
    device const bike::physics_core::GpuTyreModel *tyre_model [[buffer(1)]],
    device const float *terrain_f32 [[buffer(2)]],
    device const bike::gpu::GpuSp3Config *configs [[buffer(3)]],
    device const bike::physics_core::SpindleM2K2Params<float> *spindle_params [[buffer(4)]],
    device bike::gpu::GpuSp3WorldState *sp3_states [[buffer(5)]],
    device bike::gpu::GpuSp4WorldState *sp4_states [[buffer(6)]],
    device const bike::gpu::GpuWorldState *world_states [[buffer(7)]],
    device bike::physics_core::GpuPneumaticStepScratch<float> *pneum_scratch [[buffer(8)]],
    device const int *M_rowadr [[buffer(9)]],
    device const int *M_rownnz [[buffer(10)]],
    device const int *M_colind [[buffer(11)]],
    device const float *d_M_batch [[buffer(12)]],
    device const float *qpos_batch [[buffer(13)]],
    device const float *qvel_batch [[buffer(14)]],
    device const float *xpos_batch [[buffer(15)]],
    device const float *ctrl_batch [[buffer(16)]],
    device const float *body_mass [[buffer(17)]],
    device const float *xipos_batch [[buffer(18)]],
    device const float *gravity [[buffer(19)]],
    device const int *nefc_batch [[buffer(20)]],
    device const int *efc_type_batch [[buffer(21)]],
    device const float *efc_J_batch [[buffer(22)]],
    device const float *efc_force_batch [[buffer(23)]],
    uint gid [[thread_position_in_grid]]
) {
    const int w = static_cast<int>(gid);
    if (w >= sc.nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx = sc.idx;
    const bike::physics_core::GpuTerrainTableDesc td = sc.terrain_desc;
    device const float *eff_ctrl =
        (sc.has_ctrl != 0) ? ctrl_batch : static_cast<device const float *>(nullptr);
    bike_metal_sp4_energy_post_step_impl(
        w, sc.dt_s, idx, tyre_model, td, terrain_f32,
        configs, sc.per_world_config, spindle_params,
        sp3_states, sp4_states, world_states,
        sc.has_pneum_scratch, pneum_scratch,
        sc.qM_row_stride, sc.M_world_stride,
        M_rowadr, M_rownnz, M_colind, d_M_batch,
        qpos_batch, qvel_batch,
        xpos_batch,
        eff_ctrl, body_mass, xipos_batch, gravity,
        sc.has_efc, nefc_batch, sc.njmax, sc.efc_J_row_stride,
        efc_type_batch, efc_J_batch, efc_force_batch);
}
)MSL";
    return combined;
}

MTLCompileOptions *make_compile_options() {
    MTLCompileOptions *opts = [[MTLCompileOptions alloc] init];
#if defined(__MAC_OS_X_VERSION_MAX_ALLOWED) && __MAC_OS_X_VERSION_MAX_ALLOWED >= 270000
    if (@available(macOS 27.0, *)) {
        opts.languageVersion = MTLLanguageVersion4_1;
    } else
#endif
    if (@available(macOS 26.0, *)) {
        opts.languageVersion = MTLLanguageVersion4_0;
    }
    if (@available(macOS 15.0, *)) {
        opts.mathMode = MTLMathModeSafe;
        opts.mathFloatingPointFunctions = MTLMathFloatingPointFunctionsPrecise;
    }
    return opts;
}

bool wait_command_buffer_ok(id<MTLCommandBuffer> cb, const char *caller) {
    if (!cb) {
        std::cerr << '[' << caller << "] nil MTLCommandBuffer\n";
        return false;
    }
    [cb waitUntilCompleted];
    if ([cb status] != MTLCommandBufferStatusCompleted || [cb error] != nil) {
        NSError *err = [cb error];
        std::cerr << '[' << caller << "] MTLCommandBuffer failed (status="
                  << static_cast<unsigned long>([cb status]) << "): "
                  << (err ? [[err localizedDescription] UTF8String] : "unknown error") << '\n';
        return false;
    }
    return true;
}

struct BikeMetalSp4InitScalars {
    bike::gpu::GpuModelIndices idx;
    int nworld;
    int per_world_config;
    float init_grade;
    int ntendon;
    int tendon_range_world_stride;
    int has_world_states;
    int has_qpos;
    int has_tendon_range;
};

struct BikeMetalSp4PreSolveScalars {
    bike::gpu::GpuModelIndices idx;
    bike::physics_core::GpuTerrainTableDesc terrain_desc;
    int nworld;
    float dt_s;
    int per_world_config;
    int has_body_subtreemass;
    int qM_row_stride;
    int M_world_stride;
    int ntendon;
    int has_tendon_range;
    int tendon_range_world_stride;
    int has_dof_frictionloss;
    int dof_frictionloss_world_stride;
    int has_qacc_warmstart;
    int has_pneum_scratch;
};

struct BikeMetalSp4PassScalars {
    int stage;
    int nworld;
    int nq;
    int nv;
    int nu;
    int qM_row_stride;
    int M_world_stride;
    int has_qacc_warmstart;
};

struct BikeMetalSp4PostSolveScalars {
    bike::gpu::GpuModelIndices idx;
    int nworld;
    float dt_s;
    int per_world_config;
    int has_xpos;
    int has_xanchor;
    int has_sensordata;
    int nsensordata;
    int has_efc;
    int njmax;
    int efc_J_row_stride;
    int has_contacts;
    int nconmax;
    int contact_efc_stride;
    int has_contact_worldid;
    int has_contact_dim;
};

struct BikeMetalSp4M0StepScalars {
    bike::gpu::GpuModelIndices idx;
    int nworld;
    int nq;
    int nv;
    int nu;
    int per_world_config;
    int qM_row_stride;
    int M_world_stride;
    int has_qacc_warmstart;
};

struct BikeMetalSp4EnergyScalars {
    bike::gpu::GpuModelIndices idx;
    bike::physics_core::GpuTerrainTableDesc terrain_desc;
    int nworld;
    float dt_s;
    int per_world_config;
    int qM_row_stride;
    int M_world_stride;
    int njmax;
    int efc_J_row_stride;
    int has_pneum_scratch;
    int has_efc;
    int has_ctrl;
};

struct MetalWritersContext {
    id<MTLDevice> device = nil;
    id<MTLCommandQueue> queue = nil;
    id<MTLLibrary> library = nil;
    id<MTLComputePipelineState> pso_init_states = nil;
    id<MTLComputePipelineState> pso_1thread = nil;
    id<MTLComputePipelineState> pso_step_writers_fused = nil;
    id<MTLComputePipelineState> pso_sp3_init = nil;
    id<MTLComputePipelineState> pso_sp3_pre_solve = nil;
    id<MTLComputePipelineState> pso_sp3_post_solve = nil;
    id<MTLComputePipelineState> pso_sp4_init = nil;
    id<MTLComputePipelineState> pso_sp4_pre_solve = nil;
    id<MTLComputePipelineState> pso_sp4_pass_stage = nil;
    id<MTLComputePipelineState> pso_sp4_post_solve = nil;
    id<MTLComputePipelineState> pso_sp4_m0_init = nil;
    id<MTLComputePipelineState> pso_sp4_m0_pre_solve = nil;
    id<MTLComputePipelineState> pso_sp4_m0_step = nil;
    id<MTLComputePipelineState> pso_sp4_m0_post_solve = nil;
    id<MTLComputePipelineState> pso_sp4_energy_post_step = nil;
    id<MTLBuffer> b_pneum_scratch = nil;
    id<MTLBuffer> b_pneum_dummy = nil;
    int pneum_scratch_cap = 0;
    std::mutex pneum_mutex;
    bool ok = false;

    id<MTLBuffer> ensure_pneum_scratch(int nworld, bool is_pneumatic) {
        std::lock_guard<std::mutex> lock(pneum_mutex);
        if (!device) {
            return nil;
        }
        if (!is_pneumatic) {
            if (!b_pneum_dummy) {
                b_pneum_dummy = [device newBufferWithLength:16U options:MTLResourceStorageModeShared];
            }
            return b_pneum_dummy;
        }
        if (nworld <= 0) {
            return nil;
        }
        if (!b_pneum_scratch || nworld > pneum_scratch_cap) {
            const NSUInteger bytes =
                static_cast<NSUInteger>(nworld) *
                static_cast<NSUInteger>(sizeof(bike::physics_core::GpuPneumaticStepScratch<float>));
            id<MTLBuffer> buf = [device newBufferWithLength:bytes options:MTLResourceStorageModeShared];
            if (!buf) {
                return nil;
            }
            b_pneum_scratch = buf;
            pneum_scratch_cap = nworld;
        }
        return b_pneum_scratch;
    }

    bool init(const std::string &gpu_src_dir) {
        if (ok && pso_1thread != nil && pso_init_states != nil &&
            pso_sp3_init != nil && pso_sp3_pre_solve != nil && pso_sp3_post_solve != nil &&
            pso_sp4_init != nil && pso_sp4_pre_solve != nil &&
            pso_sp4_pass_stage != nil && pso_sp4_post_solve != nil &&
            pso_sp4_m0_init != nil && pso_sp4_m0_pre_solve != nil &&
            pso_sp4_m0_step != nil && pso_sp4_m0_post_solve != nil &&
            pso_sp4_energy_post_step != nil) {
            return true;
        }
        device = MTLCreateSystemDefaultDevice();
        if (!device) {
            std::cerr << "[MetalWritersContext] MTLCreateSystemDefaultDevice returned nil\n";
            return false;
        }
        queue = [device newCommandQueue];
        if (!queue) {
            std::cerr << "[MetalWritersContext] newCommandQueue returned nil\n";
            return false;
        }
        const std::string src = build_msl_shader_source(gpu_src_dir);
        if (src.empty()) {
            std::cerr << "[MetalWritersContext] Empty MSL shader source from dir '"
                      << gpu_src_dir << "'\n";
            return false;
        }
        NSError *err = nil;
        MTLCompileOptions *opts = make_compile_options();
        library = [device newLibraryWithSource:[NSString stringWithUTF8String:src.c_str()]
                                       options:opts
                                         error:&err];
        if (!library) {
            std::cerr << "[MetalWritersContext] MSL compile error:\n"
                      << (err ? [[err localizedDescription] UTF8String] : "unknown Metal error")
                      << '\n';
            return false;
        }
        id<MTLFunction> fn_init = [library newFunctionWithName:@"bike_gpu_init_states_metal_kernel"];
        id<MTLFunction> fn1 = [library newFunctionWithName:@"bike_gpu_writers_metal_kernel"];
        id<MTLFunction> fn_wp1 = [library newFunctionWithName:@"bike_wp_writers_metal_kernel"];
        id<MTLFunction> fn_sp3_init = [library newFunctionWithName:@"bike_gpu_sp3_init_metal_kernel"];
        id<MTLFunction> fn_sp3_pre = [library newFunctionWithName:@"bike_gpu_sp3_pre_solve_metal_kernel"];
        id<MTLFunction> fn_sp3_post = [library newFunctionWithName:@"bike_gpu_sp3_post_solve_metal_kernel"];
        id<MTLFunction> fn_sp4_init = [library newFunctionWithName:@"bike_gpu_sp4_init_metal_kernel"];
        id<MTLFunction> fn_sp4_pre = [library newFunctionWithName:@"bike_gpu_sp4_pre_solve_and_pass0_metal_kernel"];
        id<MTLFunction> fn_sp4_pass = [library newFunctionWithName:@"bike_gpu_sp4_pass_stage_metal_kernel"];
        id<MTLFunction> fn_sp4_post = [library newFunctionWithName:@"bike_gpu_sp4_post_solve_metal_kernel"];
        id<MTLFunction> fn_sp4_m0_init = [library newFunctionWithName:@"bike_gpu_sp4_m0_init_metal_kernel"];
        id<MTLFunction> fn_sp4_m0_pre = [library newFunctionWithName:@"bike_gpu_sp4_m0_pre_solve_and_begin_metal_kernel"];
        id<MTLFunction> fn_sp4_m0_step = [library newFunctionWithName:@"bike_gpu_sp4_m0_step_metal_kernel"];
        id<MTLFunction> fn_sp4_m0_post = [library newFunctionWithName:@"bike_gpu_sp4_m0_post_solve_metal_kernel"];
        id<MTLFunction> fn_sp4_energy = [library newFunctionWithName:@"bike_gpu_sp4_energy_post_step_metal_kernel"];
        if (!fn_init || !fn1 || !fn_wp1 || !fn_sp3_init || !fn_sp3_pre || !fn_sp3_post ||
            !fn_sp4_init || !fn_sp4_pre || !fn_sp4_pass || !fn_sp4_post ||
            !fn_sp4_m0_init || !fn_sp4_m0_pre || !fn_sp4_m0_step || !fn_sp4_m0_post ||
            !fn_sp4_energy) {
            std::cerr << "[MetalWritersContext] Failed to look up MSL kernel functions\n";
            return false;
        }

        pso_init_states = [device newComputePipelineStateWithFunction:fn_init error:&err];
        pso_1thread = [device newComputePipelineStateWithFunction:fn1 error:&err];
        pso_step_writers_fused = [device newComputePipelineStateWithFunction:fn_wp1 error:&err];
        pso_sp3_init = [device newComputePipelineStateWithFunction:fn_sp3_init error:&err];
        pso_sp3_pre_solve = [device newComputePipelineStateWithFunction:fn_sp3_pre error:&err];
        pso_sp3_post_solve = [device newComputePipelineStateWithFunction:fn_sp3_post error:&err];
        pso_sp4_init = [device newComputePipelineStateWithFunction:fn_sp4_init error:&err];
        pso_sp4_pre_solve = [device newComputePipelineStateWithFunction:fn_sp4_pre error:&err];
        pso_sp4_pass_stage = [device newComputePipelineStateWithFunction:fn_sp4_pass error:&err];
        pso_sp4_post_solve = [device newComputePipelineStateWithFunction:fn_sp4_post error:&err];
        pso_sp4_m0_init = [device newComputePipelineStateWithFunction:fn_sp4_m0_init error:&err];
        pso_sp4_m0_pre_solve = [device newComputePipelineStateWithFunction:fn_sp4_m0_pre error:&err];
        pso_sp4_m0_step = [device newComputePipelineStateWithFunction:fn_sp4_m0_step error:&err];
        pso_sp4_m0_post_solve = [device newComputePipelineStateWithFunction:fn_sp4_m0_post error:&err];
        pso_sp4_energy_post_step = [device newComputePipelineStateWithFunction:fn_sp4_energy error:&err];
        if (!pso_init_states || !pso_1thread || !pso_step_writers_fused ||
            !pso_sp3_init || !pso_sp3_pre_solve || !pso_sp3_post_solve ||
            !pso_sp4_init || !pso_sp4_pre_solve || !pso_sp4_pass_stage || !pso_sp4_post_solve ||
            !pso_sp4_m0_init || !pso_sp4_m0_pre_solve || !pso_sp4_m0_step ||
            !pso_sp4_m0_post_solve || !pso_sp4_energy_post_step) {
            std::cerr << "[MetalWritersContext] PSO error:\n"
                      << (err ? [[err localizedDescription] UTF8String] : "unknown Metal error")
                      << '\n';
            return false;
        }

        ok = true;
        return true;
    }
};

MetalWritersContext &get_metal_ctx(const char *gpu_src_dir = nullptr) {
    static MetalWritersContext g_ctx;
    static std::mutex g_ctx_mutex;
    std::lock_guard<std::mutex> lock(g_ctx_mutex);
    if (g_ctx.ok) {
        return g_ctx;
    }
    std::string dir_str;
    if (gpu_src_dir != nullptr && gpu_src_dir[0] != '\0') {
        dir_str = gpu_src_dir;
    } else {
        std::string file_path = __FILE__;
        const std::size_t slash = file_path.find_last_of("/\\");
        dir_str = (slash != std::string::npos) ? file_path.substr(0, slash) : ".";
    }
    g_ctx.init(dir_str);
    return g_ctx;
}

void encode_metal_writers_slice(
    id<MTLComputeCommandEncoder> enc,
    MetalWritersContext &ctx,
    int world_offset,
    int world_count,
    float dt_s,
    const bike::gpu::GpuModelIndices &idx,
    id<MTLBuffer> b_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc &terrain_desc,
    id<MTLBuffer> b_terrain_f32,
    id<MTLBuffer> b_terrain_i32,
    id<MTLBuffer> b_cfg,
    int per_world_config,
    id<MTLBuffer> b_st,
    id<MTLBuffer> b_qpos,
    id<MTLBuffer> b_qvel,
    id<MTLBuffer> b_xpos,
    id<MTLBuffer> b_xmat,
    id<MTLBuffer> b_xanchor,
    id<MTLBuffer> b_xaxis,
    id<MTLBuffer> b_qfrc,
    id<MTLBuffer> b_ctrl,
    id<MTLBuffer> b_tendon,
    int tendon_stride,
    id<MTLBuffer> b_pneum_scratch
) {
    [enc setComputePipelineState:ctx.pso_1thread];
    [enc setBytes:&world_offset length:sizeof(int) atIndex:0];
    [enc setBytes:&world_count length:sizeof(int) atIndex:1];
    [enc setBytes:&dt_s length:sizeof(float) atIndex:2];
    [enc setBytes:&idx length:sizeof(bike::gpu::GpuModelIndices) atIndex:3];
    [enc setBuffer:b_tyre_model offset:0 atIndex:4];
    [enc setBytes:&terrain_desc length:sizeof(bike::physics_core::GpuTerrainTableDesc) atIndex:5];
    [enc setBuffer:b_terrain_f32 offset:0 atIndex:6];
    [enc setBuffer:b_terrain_i32 offset:0 atIndex:7];
    [enc setBuffer:b_cfg offset:0 atIndex:8];
    [enc setBytes:&per_world_config length:sizeof(int) atIndex:9];
    [enc setBuffer:b_st offset:0 atIndex:10];
    [enc setBuffer:b_qpos offset:0 atIndex:11];
    [enc setBuffer:b_qvel offset:0 atIndex:12];
    [enc setBuffer:b_xpos offset:0 atIndex:13];
    [enc setBuffer:b_xmat offset:0 atIndex:14];
    [enc setBuffer:b_xanchor offset:0 atIndex:15];
    [enc setBuffer:b_xaxis offset:0 atIndex:16];
    [enc setBuffer:b_qfrc offset:0 atIndex:17];
    [enc setBuffer:b_ctrl offset:0 atIndex:18];
    [enc setBuffer:b_tendon offset:0 atIndex:19];
    [enc setBytes:&tendon_stride length:sizeof(int) atIndex:20];
    [enc setBuffer:b_pneum_scratch offset:0 atIndex:21];

    const NSUInteger total_threads = static_cast<NSUInteger>(world_count);
    const NSUInteger tg = std::min<NSUInteger>(64, std::max<NSUInteger>(1, total_threads));
    [enc dispatchThreads:MTLSizeMake(total_threads, 1, 1)
   threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
}

} // namespace

extern "C" {

int bike_metal_init(const char *gpu_src_dir) {
    MetalWritersContext &ctx = get_metal_ctx(gpu_src_dir);
    return ctx.ok ? 0 : -1;
}

const char *bike_metal_device_name() {
    static std::mutex dev_mutex;
    static std::string cached_name;
    std::lock_guard<std::mutex> lock(dev_mutex);
    if (cached_name.empty()) {
        id<MTLDevice> dev = MTLCreateSystemDefaultDevice();
        if (!dev || ![dev name]) {
            return "None";
        }
        cached_name = [[dev name] UTF8String];
    }
    return cached_name.c_str();
}

const char *bike_metal_get_shader_source(const char *gpu_src_dir) {
    std::string dir_str;
    if (gpu_src_dir != nullptr && gpu_src_dir[0] != '\0') {
        dir_str = gpu_src_dir;
    } else {
        std::string file_path = __FILE__;
        const std::size_t slash = file_path.find_last_of("/\\");
        dir_str = (slash != std::string::npos) ? file_path.substr(0, slash) : ".";
    }
    static std::mutex src_mutex;
    static std::string cached_src;
    std::lock_guard<std::mutex> lock(src_mutex);
    cached_src = build_msl_shader_source(dir_str);
    return cached_src.c_str();
}

int bike_metal_launch_init_states(
    int nworld,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos
) {
    if (nworld <= 0 || d_idx == nullptr || d_configs == nullptr || d_states == nullptr) {
        return -1;
    }
    const bike::gpu::GpuModelIndices idx_val = *d_idx;
    if (!bike::gpu::gpu_validate_model_indices(idx_val)) {
        std::cerr << "[bike_metal_launch_init_states] Invalid model indices or kMax bounds exceeded\n";
        return -1;
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_init_states) {
        std::cerr << "[bike_metal_launch_init_states] Metal context or pipeline not initialized\n";
        return -2;
    }
    const int n_cfg = per_world_config ? nworld : 1;
    const std::size_t cfg_bytes =
        static_cast<std::size_t>(n_cfg) * sizeof(bike::gpu::GpuWorldConfig);
    const std::size_t st_bytes =
        static_cast<std::size_t>(nworld) * sizeof(bike::gpu::GpuWorldState);
    const std::size_t qpos_bytes =
        static_cast<std::size_t>(nworld * std::max(1, idx_val.nq)) * sizeof(float);

    @autoreleasepool {
        id<MTLBuffer> b_cfg = [ctx.device newBufferWithBytes:d_configs
                                                      length:cfg_bytes
                                                     options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_st = [ctx.device newBufferWithLength:st_bytes
                                                     options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_qpos = (d_qpos != nullptr && idx_val.nq > 0)
            ? [ctx.device newBufferWithBytes:d_qpos
                                      length:qpos_bytes
                                     options:MTLResourceStorageModeShared]
            : nil;

        if (!b_cfg || !b_st || (d_qpos != nullptr && idx_val.nq > 0 && !b_qpos)) {
            std::cerr << "[bike_metal_launch_init_states] Failed to allocate Metal buffers\n";
            return -3;
        }

        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        [enc setComputePipelineState:ctx.pso_init_states];
        [enc setBytes:&nworld length:sizeof(int) atIndex:0];
        [enc setBytes:&idx_val length:sizeof(bike::gpu::GpuModelIndices) atIndex:1];
        [enc setBuffer:b_cfg offset:0 atIndex:2];
        [enc setBytes:&per_world_config length:sizeof(int) atIndex:3];
        [enc setBuffer:b_st offset:0 atIndex:4];
        [enc setBuffer:b_qpos offset:0 atIndex:5];
        const NSUInteger total_threads = static_cast<NSUInteger>(nworld);
        const NSUInteger tg = std::min<NSUInteger>(64, std::max<NSUInteger>(1, total_threads));
        [enc dispatchThreads:MTLSizeMake(total_threads, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_launch_init_states")) {
            return -4;
        }
        const std::size_t nworld_u = to_index(nworld);
        const std::span<const bike::gpu::GpuWorldState> st_src =
            boundary_span(static_cast<const bike::gpu::GpuWorldState *>([b_st contents]), nworld_u);
        const std::span<bike::gpu::GpuWorldState> st_dst = boundary_span(d_states, nworld_u);
        std::ranges::copy(st_src, st_dst.begin());
    }
    return 0;
}

int bike_metal_launch_writers(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    float *d_qfrc_applied,
    float *d_ctrl,
    float *d_tendon_range,
    int tendon_range_world_stride
) {
    if (nworld <= 0 || !std::isfinite(dt_s) || dt_s <= 0.0f ||
        tendon_range_world_stride < 0 ||
        d_idx == nullptr || d_tyre_model == nullptr || d_terrain_desc == nullptr ||
        d_configs == nullptr || d_states == nullptr ||
        d_qpos == nullptr || d_qvel == nullptr ||
        d_xpos == nullptr || d_xmat == nullptr ||
        d_xanchor == nullptr || d_xaxis == nullptr ||
        d_qfrc_applied == nullptr || d_ctrl == nullptr) {
        return -1;
    }
    const bike::gpu::GpuModelIndices idx_val = *d_idx;
    if (!bike::gpu::gpu_validate_model_indices(idx_val)) {
        std::cerr << "[bike_metal_launch_writers] Invalid model indices or kMax bounds exceeded\n";
        return -1;
    }
    const int max_active_tendon_idx = bike::gpu::gpu_max_active_tendon_idx(idx_val);
    if (d_tendon_range != nullptr && max_active_tendon_idx >= 0 &&
        tendon_range_world_stride > 0 &&
        static_cast<long>(2) * static_cast<long>(max_active_tendon_idx) + 1L >=
            static_cast<long>(tendon_range_world_stride)) {
        std::cerr << "[bike_metal_launch_writers] tendon_range_world_stride too small for active tendons\n";
        return -1;
    }
    if (bike::gpu::validate_host_tyre_and_terrain(
            d_tyre_model,
            idx_val.nq,
            idx_val.nv,
            idx_val.nbody,
            d_terrain_desc,
            d_terrain_f32,
            d_terrain_i32) != 0) {
        std::cerr << "[bike_metal_launch_writers] Invalid GpuTyreModel or GpuTerrainTableDesc\n";
        return -1;
    }

    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_1thread) {
        std::cerr << "[bike_metal_launch_writers] Metal context or pipeline not initialized\n";
        return -2;
    }

    const int nq = idx_val.nq;
    const int nv = idx_val.nv;
    const int nu = idx_val.nu;
    const int nbody3 = idx_val.nbody * 3;
    const int nbody9 = idx_val.nbody * 9;
    const int njnt3 = d_tyre_model->njnt * 3;
    const int n_cfg = per_world_config ? nworld : 1;

    const std::size_t cfg_bytes =
        static_cast<std::size_t>(n_cfg) * sizeof(bike::gpu::GpuWorldConfig);
    const std::size_t st_bytes =
        static_cast<std::size_t>(nworld) * sizeof(bike::gpu::GpuWorldState);
    const std::size_t qpos_bytes = static_cast<std::size_t>(nworld * nq) * sizeof(float);
    const std::size_t qvel_bytes = static_cast<std::size_t>(nworld * nv) * sizeof(float);
    const std::size_t xpos_bytes = static_cast<std::size_t>(nworld * nbody3) * sizeof(float);
    const std::size_t xmat_bytes = static_cast<std::size_t>(nworld * nbody9) * sizeof(float);
    const std::size_t xanchor_bytes = static_cast<std::size_t>(nworld * njnt3) * sizeof(float);
    const std::size_t xaxis_bytes = static_cast<std::size_t>(nworld * njnt3) * sizeof(float);
    const std::size_t qfrc_bytes = static_cast<std::size_t>(nworld * nv) * sizeof(float);
    const std::size_t ctrl_bytes = static_cast<std::size_t>(nworld * nu) * sizeof(float);
    const std::size_t tendon_floats =
        (tendon_range_world_stride > 0)
            ? static_cast<std::size_t>(nworld * tendon_range_world_stride)
            : ((max_active_tendon_idx >= 0)
                   ? static_cast<std::size_t>(2 * (max_active_tendon_idx + 1))
                   : 0ULL);
    const std::size_t tendon_bytes = tendon_floats * sizeof(float);

    @autoreleasepool {
        id<MTLBuffer> b_tyre = [ctx.device newBufferWithBytes:d_tyre_model
                                                       length:sizeof(bike::physics_core::GpuTyreModel)
                                                      options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_tf32 = (d_terrain_desc->f32_count > 0 && d_terrain_f32 != nullptr)
            ? [ctx.device newBufferWithBytes:d_terrain_f32
                                      length:static_cast<NSUInteger>(d_terrain_desc->f32_count) * sizeof(float)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(float) options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_ti32 = (d_terrain_desc->i32_count > 0 && d_terrain_i32 != nullptr)
            ? [ctx.device newBufferWithBytes:d_terrain_i32
                                      length:static_cast<NSUInteger>(d_terrain_desc->i32_count) * sizeof(bike::physics_core::i32)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(bike::physics_core::i32) options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_cfg = [ctx.device newBufferWithBytes:d_configs
                                                      length:cfg_bytes
                                                     options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_st = [ctx.device newBufferWithBytes:d_states
                                                     length:st_bytes
                                                    options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_qpos = [ctx.device newBufferWithBytes:d_qpos
                                                       length:qpos_bytes
                                                      options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_qvel = [ctx.device newBufferWithBytes:d_qvel
                                                       length:qvel_bytes
                                                      options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_xpos = [ctx.device newBufferWithBytes:d_xpos
                                                       length:xpos_bytes
                                                      options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_xmat = [ctx.device newBufferWithBytes:d_xmat
                                                       length:xmat_bytes
                                                      options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_xanchor = [ctx.device newBufferWithBytes:d_xanchor
                                                          length:xanchor_bytes
                                                         options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_xaxis = [ctx.device newBufferWithBytes:d_xaxis
                                                        length:xaxis_bytes
                                                       options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_qfrc = [ctx.device newBufferWithLength:qfrc_bytes
                                                       options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_ctrl = [ctx.device newBufferWithLength:ctrl_bytes
                                                       options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_tendon = (d_tendon_range != nullptr && tendon_floats > 0)
            ? [ctx.device newBufferWithBytes:d_tendon_range
                                      length:tendon_bytes
                                     options:MTLResourceStorageModeShared]
            : nil;
        const bool is_pneum =
            (d_tyre_model->backend == bike::physics_core::TyreBackendKind::pneumatic);
        id<MTLBuffer> b_pneum = ctx.ensure_pneum_scratch(nworld, is_pneum);

        if (!b_tyre || !b_tf32 || !b_ti32 || !b_cfg || !b_st ||
            !b_qpos || !b_qvel || !b_xpos || !b_xmat || !b_xanchor || !b_xaxis ||
            !b_qfrc || !b_ctrl || (d_tendon_range != nullptr && tendon_floats > 0 && !b_tendon) ||
            !b_pneum) {
            std::cerr << "[bike_metal_launch_writers] Failed to allocate Metal buffers\n";
            return -3;
        }

        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        encode_metal_writers_slice(
            enc,
            ctx,
            0,
            nworld,
            dt_s,
            idx_val,
            b_tyre,
            *d_terrain_desc,
            b_tf32,
            b_ti32,
            b_cfg,
            per_world_config,
            b_st,
            b_qpos,
            b_qvel,
            b_xpos,
            b_xmat,
            b_xanchor,
            b_xaxis,
            b_qfrc,
            b_ctrl,
            b_tendon,
            tendon_range_world_stride,
            b_pneum
        );
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_launch_writers")) {
            return -4;
        }

        const std::size_t nworld_u = to_index(nworld);
        const std::span<const bike::gpu::GpuWorldState> st_src =
            boundary_span(static_cast<const bike::gpu::GpuWorldState *>([b_st contents]), nworld_u);
        const std::span<const float> qfrc_src =
            boundary_span(static_cast<const float *>([b_qfrc contents]), nworld_u * to_index(nv));
        const std::span<const float> ctrl_src =
            boundary_span(static_cast<const float *>([b_ctrl contents]), nworld_u * to_index(nu));
        std::ranges::copy(st_src, boundary_span(d_states, nworld_u).begin());
        std::ranges::copy(qfrc_src, boundary_span(d_qfrc_applied, nworld_u * to_index(nv)).begin());
        std::ranges::copy(ctrl_src, boundary_span(d_ctrl, nworld_u * to_index(nu)).begin());
        if (d_tendon_range != nullptr && b_tendon != nil && tendon_floats > 0) {
            const std::span<const float> tendon_src =
                boundary_span(static_cast<const float *>([b_tendon contents]), tendon_floats);
            std::ranges::copy(tendon_src, boundary_span(d_tendon_range, tendon_floats).begin());
        }
    }
    return 0;
}

int bike_metal_benchmark_writers_only(
    int nworld,
    int n_steps,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    double *out_single_cb_ms,
    double *out_per_step_cb_ms
) {
    if (nworld <= 0 || n_steps <= 0 || !std::isfinite(dt_s) || dt_s <= 0.0f ||
        d_idx == nullptr || d_configs == nullptr ||
        d_tyre_model == nullptr || d_terrain_desc == nullptr) {
        return -1;
    }
    const bike::gpu::GpuModelIndices idx_val = *d_idx;
    if (!bike::gpu::gpu_validate_model_indices(idx_val)) {
        std::cerr << "[bike_metal_benchmark_writers_only] Invalid model indices or kMax bounds exceeded\n";
        return -1;
    }
    if (bike::gpu::validate_host_tyre_and_terrain(
            d_tyre_model,
            idx_val.nq,
            idx_val.nv,
            idx_val.nbody,
            d_terrain_desc,
            d_terrain_f32,
            d_terrain_i32) != 0) {
        return -1;
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_1thread) {
        return -2;
    }

    const int nq = idx_val.nq;
    const int nv = idx_val.nv;
    const int nu = idx_val.nu;
    const int nbody3 = idx_val.nbody * 3;
    const int nbody9 = idx_val.nbody * 9;
    const int njnt3 = d_tyre_model->njnt * 3;
    const int n_cfg = per_world_config ? nworld : 1;
    const std::span<const bike::gpu::GpuWorldConfig> configs_v =
        boundary_span(d_configs, to_index(n_cfg));

    id<MTLBuffer> b_tyre = [ctx.device newBufferWithBytes:d_tyre_model
                                                   length:sizeof(bike::physics_core::GpuTyreModel)
                                                  options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_tf32 = (d_terrain_desc->f32_count > 0 && d_terrain_f32 != nullptr)
        ? [ctx.device newBufferWithBytes:d_terrain_f32
                                  length:static_cast<NSUInteger>(d_terrain_desc->f32_count) * sizeof(float)
                                 options:MTLResourceStorageModeShared]
        : [ctx.device newBufferWithLength:sizeof(float) options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_ti32 = (d_terrain_desc->i32_count > 0 && d_terrain_i32 != nullptr)
        ? [ctx.device newBufferWithBytes:d_terrain_i32
                                  length:static_cast<NSUInteger>(d_terrain_desc->i32_count) * sizeof(bike::physics_core::i32)
                                 options:MTLResourceStorageModeShared]
        : [ctx.device newBufferWithLength:sizeof(bike::physics_core::i32) options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_cfg = [ctx.device newBufferWithBytes:d_configs
                                                  length:static_cast<NSUInteger>(n_cfg) * sizeof(bike::gpu::GpuWorldConfig)
                                                 options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_st = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld) * sizeof(bike::gpu::GpuWorldState)
                                                 options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_qpos = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * nq) * sizeof(float)
                                                   options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_qvel = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * nv) * sizeof(float)
                                                   options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_xpos = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * nbody3) * sizeof(float)
                                                   options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_xmat = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * nbody9) * sizeof(float)
                                                   options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_xanchor = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * njnt3) * sizeof(float)
                                                      options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_xaxis = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * njnt3) * sizeof(float)
                                                    options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_qfrc = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * nv) * sizeof(float)
                                                   options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_ctrl = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * nu) * sizeof(float)
                                                   options:MTLResourceStorageModeShared];
    const int max_active_tendon_idx = bike::gpu::gpu_max_active_tendon_idx(idx_val);
    const int tendon_slots_per_world =
        (max_active_tendon_idx >= 0) ? (2 * (max_active_tendon_idx + 1)) : 2;
    id<MTLBuffer> b_tendon = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * tendon_slots_per_world) * sizeof(float)
                                                     options:MTLResourceStorageModeShared];
    const bool is_pneum =
        (d_tyre_model->backend == bike::physics_core::TyreBackendKind::pneumatic);
    id<MTLBuffer> b_pneum = ctx.ensure_pneum_scratch(nworld, is_pneum);
    if (!b_tyre || !b_tf32 || !b_ti32 || !b_cfg || !b_st ||
        !b_qpos || !b_qvel || !b_xpos || !b_xmat || !b_xanchor || !b_xaxis ||
        !b_qfrc || !b_ctrl || !b_tendon || !b_pneum) {
        std::cerr << "[bike_metal_benchmark_writers_only] Failed to allocate Metal buffers\n";
        return -3;
    }

    const std::size_t nworld_u = to_index(nworld);
    const std::span<bike::gpu::GpuWorldState> st_v =
        boundary_span(static_cast<bike::gpu::GpuWorldState *>([b_st contents]), nworld_u);
    const std::span<float> qpos_v =
        boundary_span(static_cast<float *>([b_qpos contents]), nworld_u * to_index(nq));
    const std::span<float> qvel_v =
        boundary_span(static_cast<float *>([b_qvel contents]), nworld_u * to_index(nv));
    const std::span<float> xpos_v =
        boundary_span(static_cast<float *>([b_xpos contents]), nworld_u * to_index(nbody3));
    const std::span<float> xmat_v =
        boundary_span(static_cast<float *>([b_xmat contents]), nworld_u * to_index(nbody9));
    const std::span<float> xanchor_v =
        boundary_span(static_cast<float *>([b_xanchor contents]), nworld_u * to_index(njnt3));
    const std::span<float> xaxis_v =
        boundary_span(static_cast<float *>([b_xaxis contents]), nworld_u * to_index(njnt3));
    const std::span<float> tendon_v =
        boundary_span(static_cast<float *>([b_tendon contents]), to_index(tendon_slots_per_world) * nworld_u);

    std::ranges::fill(qpos_v, 0.0f);
    std::ranges::fill(qvel_v, 0.0f);
    std::ranges::fill(xpos_v, 0.0f);
    std::ranges::fill(xmat_v, 0.0f);
    std::ranges::fill(xanchor_v, 0.0f);
    std::ranges::fill(xaxis_v, 0.0f);
    std::ranges::fill(tendon_v, 0.0f);

    for (int w = 0; w < nworld; ++w) {
        const std::size_t wu = to_index(w);
        const std::span<float> qp = qpos_v.subspan(wu * to_index(nq), to_index(nq));
        const std::span<float> qv = qvel_v.subspan(wu * to_index(nv), to_index(nv));
        const std::span<float> xp = xpos_v.subspan(wu * to_index(nbody3), to_index(nbody3));
        const std::span<float> xm = xmat_v.subspan(wu * to_index(nbody9), to_index(nbody9));
        const std::span<float> xa = xaxis_v.subspan(wu * to_index(njnt3), to_index(njnt3));

        if (idx_val.root_x_qpos >= 0) qp[to_index(idx_val.root_x_qpos)] = 2.0f;
        if (idx_val.root_z_qpos >= 0) qp[to_index(idx_val.root_z_qpos)] = 0.56f;
        if (idx_val.fork_travel_qpos >= 0) qp[to_index(idx_val.fork_travel_qpos)] = 0.05f;
        if (idx_val.shock_stroke_qpos >= 0) qp[to_index(idx_val.shock_stroke_qpos)] = 0.02f;

        if (idx_val.root_x_dof >= 0) qv[to_index(idx_val.root_x_dof)] = 8.0f;
        if (idx_val.front_wheel_spin_dof >= 0) qv[to_index(idx_val.front_wheel_spin_dof)] = 22.0f;
        if (idx_val.rear_wheel_spin_dof >= 0) qv[to_index(idx_val.rear_wheel_spin_dof)] = 22.0f;

        if (idx_val.frame_body >= 0) {
            xp[3 * to_index(idx_val.frame_body) + 0] = 2.0f;
            xp[3 * to_index(idx_val.frame_body) + 2] = 0.56f;
        }
        if (idx_val.front_wheel_body >= 0) {
            xp[3 * to_index(idx_val.front_wheel_body) + 0] = 2.60f;
            xp[3 * to_index(idx_val.front_wheel_body) + 2] = 0.34f;
        }
        if (idx_val.rear_wheel_body >= 0) {
            xp[3 * to_index(idx_val.rear_wheel_body) + 0] = 1.40f;
            xp[3 * to_index(idx_val.rear_wheel_body) + 2] = 0.33f;
        }
        for (int b = 0; b < idx_val.nbody; ++b) {
            xm[9 * to_index(b) + 0] = 1.0f;
            xm[9 * to_index(b) + 4] = 1.0f;
            xm[9 * to_index(b) + 8] = 1.0f;
        }
        for (int j = 0; j < d_tyre_model->njnt; ++j) {
            xa[3 * to_index(j) + 1] = 1.0f;
        }
        const bike::gpu::GpuWorldConfig &cfg_w = configs_v[per_world_config ? wu : 0];
        const bool has_ds = (idx_val.drive_shaft_spin_qpos >= 0);
        const float crank_qpos =
            (idx_val.crank_spin_qpos >= 0) ? qp[to_index(idx_val.crank_spin_qpos)] : 0.0f;
        const float rw_qpos =
            (idx_val.rear_wheel_spin_qpos >= 0) ? qp[to_index(idx_val.rear_wheel_spin_qpos)] : 0.0f;
        const float ds_qpos = has_ds ? qp[to_index(idx_val.drive_shaft_spin_qpos)] : 0.0f;
        bike::gpu::gpu_init_world_state(
            st_v[wu], cfg_w.drivetrain, crank_qpos, rw_qpos, ds_qpos, has_ds);
    }

    {
        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        encode_metal_writers_slice(
            enc, ctx, 0, nworld, dt_s, idx_val, b_tyre, *d_terrain_desc, b_tf32, b_ti32,
            b_cfg, per_world_config, b_st, b_qpos, b_qvel, b_xpos, b_xmat, b_xanchor, b_xaxis,
            b_qfrc, b_ctrl, b_tendon, tendon_slots_per_world, b_pneum
        );
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_benchmark_writers_only")) {
            return -4;
        }
    }

    const auto t0 = std::chrono::steady_clock::now();
    {
        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        for (int s = 0; s < n_steps; ++s) {
            encode_metal_writers_slice(
                enc, ctx, 0, nworld, dt_s, idx_val, b_tyre, *d_terrain_desc, b_tf32, b_ti32,
                b_cfg, per_world_config, b_st, b_qpos, b_qvel, b_xpos, b_xmat, b_xanchor, b_xaxis,
                b_qfrc, b_ctrl, b_tendon, tendon_slots_per_world, b_pneum
            );
        }
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_benchmark_writers_only")) {
            return -4;
        }
    }
    const auto t1 = std::chrono::steady_clock::now();

    const auto t2 = std::chrono::steady_clock::now();
    for (int s = 0; s < n_steps; ++s) {
        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        encode_metal_writers_slice(
            enc, ctx, 0, nworld, dt_s, idx_val, b_tyre, *d_terrain_desc, b_tf32, b_ti32,
            b_cfg, per_world_config, b_st, b_qpos, b_qvel, b_xpos, b_xmat, b_xanchor, b_xaxis,
            b_qfrc, b_ctrl, b_tendon, tendon_slots_per_world, b_pneum
        );
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_benchmark_writers_only")) {
            return -4;
        }
    }
    const auto t3 = std::chrono::steady_clock::now();

    if (out_single_cb_ms != nullptr) {
        *out_single_cb_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    }
    if (out_per_step_cb_ms != nullptr) {
        *out_per_step_cb_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
    }
    return 0;
}

int bike_metal_run_hybrid_sweep(
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

    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_1thread) {
        return -2;
    }

    std::array<char, 1024> err{};
    mjModel *m_base = mj_loadXML(xml_path, nullptr, err.data(), static_cast<int>(err.size()));
    if (m_base == nullptr) {
        std::cerr << "[bike_metal_run_hybrid_sweep] mj_loadXML failed: " << err.data() << '\n';
        return -3;
    }

    if (m_base->nq <= 0 || m_base->nq > bike::gpu::kMaxNq ||
        m_base->nv <= 0 || m_base->nv > bike::gpu::kMaxNv ||
        m_base->nu < 0 || m_base->nu > bike::gpu::kMaxNu ||
        m_base->nbody <= 0 || m_base->nbody > bike::gpu::kMaxNbody ||
        m_base->njnt <= 0 || m_base->njnt > bike::physics_core::kMaxNjnt) {
        std::cerr << "[bike_metal_run_hybrid_sweep] Model dimensions exceed kMax bounds\n";
        mj_deleteModel(m_base);
        return -4;
    }

    const assets::AssetStatus hfield_bin_status = assets::load_hfield_bin(m_base, xml_path);
    if (hfield_bin_status != assets::AssetStatus::kOk) {
        std::cerr << "[bike_metal_run_hybrid_sweep] hfield_data.bin load failed: "
                  << assets::asset_status_name(hfield_bin_status) << '\n';
        mj_deleteModel(m_base);
        return -6;
    }

    const bike::gpu::GpuModelIndices idx =
        (d_idx != nullptr) ? *d_idx : assets::resolve_gpu_model_indices(m_base);
    if (!bike::gpu::gpu_validate_model_indices(idx, static_cast<int>(m_base->ntendon)) ||
        idx.nq != static_cast<int>(m_base->nq) ||
        idx.nv != static_cast<int>(m_base->nv) ||
        idx.nu != static_cast<int>(m_base->nu) ||
        idx.nbody != static_cast<int>(m_base->nbody)) {
        std::cerr << "[bike_metal_run_hybrid_sweep] idx dimensions/indices invalid or mismatch model\n";
        mj_deleteModel(m_base);
        return -4;
    }

    const int nq = static_cast<int>(m_base->nq);
    const int nv = static_cast<int>(m_base->nv);
    const int nu = static_cast<int>(m_base->nu);
    const int nbody3 = 3 * static_cast<int>(m_base->nbody);
    const int nbody9 = 9 * static_cast<int>(m_base->nbody);
    const int njnt3 = 3 * static_cast<int>(m_base->njnt);
    const int ntendon = static_cast<int>(m_base->ntendon);
    const float dt_s = static_cast<float>(m_base->opt.timestep);
    const int n_cfg = per_world_config ? nworld : 1;

    std::vector<double> qpos0(to_index(nq), 0.0);
    std::vector<double> qvel0(to_index(nv), 0.0);
    if (init_qpos == nullptr || init_qvel == nullptr) {
        std::vector<double> captured_qpos;
        std::vector<double> captured_qvel;
        const assets::AssetStatus init_status =
            assets::load_captured_initial_state(xml_path, nq, nv, captured_qpos, captured_qvel);
        if (init_status != assets::AssetStatus::kOk) {
            std::cerr << "[bike_metal_run_hybrid_sweep] captured_bootstrap.json load failed: "
                      << assets::asset_status_name(init_status) << '\n';
            mj_deleteModel(m_base);
            return -5;
        }
        qpos0 = std::move(captured_qpos);
        qvel0 = std::move(captured_qvel);
    }
    if (init_qpos != nullptr) {
        const std::span<const double> init_qpos_v = boundary_span(init_qpos, to_index(nq));
        std::ranges::copy(init_qpos_v, qpos0.begin());
    }
    if (init_qvel != nullptr) {
        const std::span<const double> init_qvel_v = boundary_span(init_qvel, to_index(nv));
        std::ranges::copy(init_qvel_v, qvel0.begin());
    }

    const auto resolved_tyre_ptr = std::make_unique<bike::gpu::ResolvedRunnerTyre>();
    bike::gpu::ResolvedRunnerTyre &resolved_tyre = *resolved_tyre_ptr;
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

    const int hw_threads = static_cast<int>(std::thread::hardware_concurrency());
    int n_workers = (num_threads > 0) ? num_threads : (hw_threads > 0 ? hw_threads : 4);
    n_workers = std::max(1, std::min(n_workers, nworld));
    const std::span<const bike::gpu::GpuWorldConfig> configs_v =
        boundary_span(d_configs, to_index(n_cfg));

    id<MTLBuffer> b_tyre = [ctx.device newBufferWithBytes:&resolved_tyre.model
                                                   length:sizeof(bike::physics_core::GpuTyreModel)
                                                  options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_tf32 = !resolved_tyre.terrain.f32.empty()
        ? [ctx.device newBufferWithBytes:resolved_tyre.terrain.f32.data()
                                  length:resolved_tyre.terrain.f32.size() * sizeof(float)
                                 options:MTLResourceStorageModeShared]
        : [ctx.device newBufferWithLength:sizeof(float) options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_ti32 = !resolved_tyre.terrain.i32.empty()
        ? [ctx.device newBufferWithBytes:resolved_tyre.terrain.i32.data()
                                  length:resolved_tyre.terrain.i32.size() * sizeof(bike::physics_core::i32)
                                 options:MTLResourceStorageModeShared]
        : [ctx.device newBufferWithLength:sizeof(bike::physics_core::i32) options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_cfg = [ctx.device newBufferWithBytes:d_configs
                                                  length:static_cast<NSUInteger>(n_cfg) * sizeof(bike::gpu::GpuWorldConfig)
                                                 options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_st = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld) * sizeof(bike::gpu::GpuWorldState)
                                                 options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_qpos = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * nq) * sizeof(float)
                                                   options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_qvel = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * nv) * sizeof(float)
                                                   options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_xpos = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * nbody3) * sizeof(float)
                                                   options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_xmat = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * nbody9) * sizeof(float)
                                                   options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_xanchor = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * njnt3) * sizeof(float)
                                                      options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_xaxis = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * njnt3) * sizeof(float)
                                                    options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_qfrc = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * nv) * sizeof(float)
                                                   options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_ctrl = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * nu) * sizeof(float)
                                                   options:MTLResourceStorageModeShared];
    const int tendon_slots_per_world = std::max(2, 2 * ntendon);
    id<MTLBuffer> b_tendon = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * tendon_slots_per_world) * sizeof(float)
                                                     options:MTLResourceStorageModeShared];
    const bool is_pneum =
        (resolved_tyre.model.backend == bike::physics_core::TyreBackendKind::pneumatic);
    id<MTLBuffer> b_pneum = ctx.ensure_pneum_scratch(nworld, is_pneum);
    if (!b_tyre || !b_tf32 || !b_ti32 || !b_cfg || !b_st ||
        !b_qpos || !b_qvel || !b_xpos || !b_xmat || !b_xanchor || !b_xaxis ||
        !b_qfrc || !b_ctrl || !b_tendon || !b_pneum) {
        std::cerr << "[bike_metal_run_hybrid_sweep] Failed to allocate Metal buffers\n";
        mj_deleteModel(m_base);
        return -8;
    }

    const std::size_t nworld_u = to_index(nworld);
    const std::span<bike::gpu::GpuWorldState> st_v =
        boundary_span(static_cast<bike::gpu::GpuWorldState *>([b_st contents]), nworld_u);
    const std::span<float> qpos_v =
        boundary_span(static_cast<float *>([b_qpos contents]), nworld_u * to_index(nq));
    const std::span<float> qvel_v =
        boundary_span(static_cast<float *>([b_qvel contents]), nworld_u * to_index(nv));
    const std::span<float> xpos_v =
        boundary_span(static_cast<float *>([b_xpos contents]), nworld_u * to_index(nbody3));
    const std::span<float> xmat_v =
        boundary_span(static_cast<float *>([b_xmat contents]), nworld_u * to_index(nbody9));
    const std::span<float> xanchor_v =
        boundary_span(static_cast<float *>([b_xanchor contents]), nworld_u * to_index(njnt3));
    const std::span<float> xaxis_v =
        boundary_span(static_cast<float *>([b_xaxis contents]), nworld_u * to_index(njnt3));
    const std::span<float> qfrc_v =
        boundary_span(static_cast<float *>([b_qfrc contents]), nworld_u * to_index(nv));
    const std::span<float> ctrl_v =
        boundary_span(static_cast<float *>([b_ctrl contents]), nworld_u * to_index(nu));
    const std::span<float> tendon_v =
        boundary_span(static_cast<float *>([b_tendon contents]), to_index(tendon_slots_per_world) * nworld_u);

    std::vector<mjModel *> worker_models(static_cast<std::size_t>(n_workers), nullptr);
    for (int t = 0; t < n_workers; ++t) {
        worker_models[static_cast<std::size_t>(t)] = mj_copyModel(nullptr, m_base);
    }

    std::vector<mjData *> world_data(nworld_u, nullptr);
    std::vector<float> max_fork_m(nworld_u, 0.0f);
    std::vector<float> max_shock_m(nworld_u, 0.0f);

    {
        std::atomic<int> init_cursor{0};
        auto init_worker = [&](int tid) {
            mjModel *m_t = worker_models[to_index(tid)];
            while (true) {
                const int w = init_cursor.fetch_add(1, std::memory_order_relaxed);
                if (w >= nworld) {
                    break;
                }
                const std::size_t wu = to_index(w);
                mjData *d_w = engine::make_data(m_t);
                world_data[wu] = d_w;
                engine::reset_data(m_t, d_w);
                const std::span<mjtNum> d_qpos = boundary_span(d_w->qpos, to_index(nq));
                const std::span<mjtNum> d_qvel = boundary_span(d_w->qvel, to_index(nv));
                const std::span<const mjtNum> d_xpos = boundary_span(d_w->xpos, to_index(nbody3));
                const std::span<const mjtNum> d_xmat = boundary_span(d_w->xmat, to_index(nbody9));
                const std::span<const mjtNum> d_xanchor = boundary_span(d_w->xanchor, to_index(njnt3));
                const std::span<const mjtNum> d_xaxis = boundary_span(d_w->xaxis, to_index(njnt3));
                for (std::size_t i = 0; i < to_index(nq); ++i) {
                    d_qpos[i] = static_cast<mjtNum>(qpos0[i]);
                }
                for (std::size_t i = 0; i < to_index(nv); ++i) {
                    d_qvel[i] = static_cast<mjtNum>(qvel0[i]);
                }
                engine::forward(m_t, d_w);

                const std::span<float> qp = qpos_v.subspan(wu * to_index(nq), to_index(nq));
                const std::span<float> qv = qvel_v.subspan(wu * to_index(nv), to_index(nv));
                const std::span<float> xp = xpos_v.subspan(wu * to_index(nbody3), to_index(nbody3));
                const std::span<float> xm = xmat_v.subspan(wu * to_index(nbody9), to_index(nbody9));
                const std::span<float> xanc = xanchor_v.subspan(wu * to_index(njnt3), to_index(njnt3));
                const std::span<float> xax = xaxis_v.subspan(wu * to_index(njnt3), to_index(njnt3));
                for (std::size_t i = 0; i < to_index(nq); ++i) qp[i] = static_cast<float>(d_qpos[i]);
                for (std::size_t i = 0; i < to_index(nv); ++i) qv[i] = static_cast<float>(d_qvel[i]);
                for (std::size_t i = 0; i < to_index(nbody3); ++i) xp[i] = static_cast<float>(d_xpos[i]);
                for (std::size_t i = 0; i < to_index(nbody9); ++i) xm[i] = static_cast<float>(d_xmat[i]);
                for (std::size_t i = 0; i < to_index(njnt3); ++i) {
                    xanc[i] = static_cast<float>(d_xanchor[i]);
                    xax[i] = static_cast<float>(d_xaxis[i]);
                }

                if (m_base->ntendon > 0) {
                    const std::span<const mjtNum> base_tendon_range =
                        boundary_span(m_base->tendon_range, 2 * to_index(ntendon));
                    const std::size_t tw = to_index(tendon_slots_per_world) * wu;
                    for (int ti = 0; ti < ntendon; ++ti) {
                        const std::size_t ft = 2 * to_index(ti);
                        tendon_v[tw + ft + 0] = static_cast<float>(base_tendon_range[ft + 0]);
                        tendon_v[tw + ft + 1] = static_cast<float>(base_tendon_range[ft + 1]);
                    }
                }

                const bike::gpu::GpuWorldConfig &cfg_w = configs_v[to_index(per_world_config ? w : 0)];
                const bool has_ds =
                    (idx.drive_shaft_spin_qpos >= 0 && idx.drive_shaft_spin_qpos < nq);
                const float crank0 =
                    (idx.crank_spin_qpos >= 0 && idx.crank_spin_qpos < nq) ? qp[to_index(idx.crank_spin_qpos)] : 0.0f;
                const float ds0 = has_ds ? qp[to_index(idx.drive_shaft_spin_qpos)] : 0.0f;
                const float rwheel0 =
                    (idx.rear_wheel_spin_qpos >= 0 && idx.rear_wheel_spin_qpos < nq) ? qp[to_index(idx.rear_wheel_spin_qpos)] : 0.0f;
                bike::gpu::gpu_init_world_state(st_v[wu], cfg_w.drivetrain, crank0, rwheel0, ds0, has_ds);
            }
        };
        std::vector<std::thread> init_threads;
        init_threads.reserve(static_cast<std::size_t>(n_workers));
        for (int t = 0; t < n_workers; ++t) {
            init_threads.emplace_back(init_worker, t);
        }
        for (auto &th : init_threads) {
            th.join();
        }
    }

    auto dispatch_gpu_slice_async = [&](int w_offset, int w_count) -> id<MTLCommandBuffer> {
        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        encode_metal_writers_slice(
            enc, ctx, w_offset, w_count, dt_s,
            idx, b_tyre, resolved_tyre.terrain.desc, b_tf32, b_ti32,
            b_cfg, per_world_config, b_st,
            b_qpos, b_qvel, b_xpos, b_xmat, b_xanchor, b_xaxis,
            b_qfrc, b_ctrl, b_tendon, tendon_slots_per_world, b_pneum
        );
        [enc endEncoding];
        [cb commit];
        return cb;
    };

    bool gpu_ok = true;
    {
        id<MTLCommandBuffer> cb = dispatch_gpu_slice_async(0, nworld);
        if (!wait_command_buffer_ok(cb, "bike_metal_run_hybrid_sweep")) {
            gpu_ok = false;
        }
        for (int w = 0; w < nworld; ++w) {
            const bike::gpu::GpuWorldConfig &cfg_w = configs_v[to_index(per_world_config ? w : 0)];
            const std::span<const float> qp =
                qpos_v.subspan(to_index(w) * to_index(nq), to_index(nq));
            const bool has_ds =
                (idx.drive_shaft_spin_qpos >= 0 && idx.drive_shaft_spin_qpos < nq);
            const float crank0 =
                (idx.crank_spin_qpos >= 0 && idx.crank_spin_qpos < nq) ? qp[to_index(idx.crank_spin_qpos)] : 0.0f;
            const float ds0 = has_ds ? qp[to_index(idx.drive_shaft_spin_qpos)] : 0.0f;
            const float rwheel0 =
                (idx.rear_wheel_spin_qpos >= 0 && idx.rear_wheel_spin_qpos < nq) ? qp[to_index(idx.rear_wheel_spin_qpos)] : 0.0f;
            bike::gpu::gpu_init_world_state(st_v[to_index(w)], cfg_w.drivetrain, crank0, rwheel0, ds0, has_ds);
        }
    }

    std::atomic<int> epoch{0};
    std::atomic<int> phase_w_begin{0};
    std::atomic<int> phase_w_end{0};
    std::atomic<int> phase_step{0};
    std::atomic<int> phase_cursor{0};
    std::atomic<int> done_count{0};
    std::atomic<bool> stop_pool{false};

    auto step_world_cpu = [&](int w, int t, int step) {
        (void)step;
        mjModel *m_t = worker_models[to_index(t)];
        mjData *d_t = world_data[to_index(w)];
        const std::size_t wu = to_index(w);
        const std::span<const float> qf = qfrc_v.subspan(wu * to_index(nv), to_index(nv));
        const std::span<const float> ct = ctrl_v.subspan(wu * to_index(nu), to_index(nu));
        const std::span<mjtNum> d_qfrc_applied = boundary_span(d_t->qfrc_applied, to_index(nv));
        const std::span<mjtNum> d_ctrl = boundary_span(d_t->ctrl, to_index(nu));
        for (std::size_t i = 0; i < to_index(nv); ++i) {
            d_qfrc_applied[i] = static_cast<mjtNum>(qf[i]);
        }
        for (std::size_t i = 0; i < to_index(nu); ++i) {
            d_ctrl[i] = static_cast<mjtNum>(ct[i]);
        }
        if (m_t->ntendon > 0) {
            const std::span<mjtNum> t_tendon_range =
                boundary_span(m_t->tendon_range, 2 * to_index(ntendon));
            const std::size_t tw = to_index(tendon_slots_per_world) * wu;
            for (int ti = 0; ti < ntendon; ++ti) {
                const std::size_t ft = 2 * to_index(ti);
                t_tendon_range[ft + 0] = static_cast<mjtNum>(tendon_v[tw + ft + 0]);
                t_tendon_range[ft + 1] = static_cast<mjtNum>(tendon_v[tw + ft + 1]);
            }
        }
        engine::step(m_t, d_t);
        engine::kinematics(m_t, d_t);

        const std::span<float> qp = qpos_v.subspan(wu * to_index(nq), to_index(nq));
        const std::span<float> qv = qvel_v.subspan(wu * to_index(nv), to_index(nv));
        const std::span<float> xp = xpos_v.subspan(wu * to_index(nbody3), to_index(nbody3));
        const std::span<float> xm = xmat_v.subspan(wu * to_index(nbody9), to_index(nbody9));
        const std::span<float> xanc = xanchor_v.subspan(wu * to_index(njnt3), to_index(njnt3));
        const std::span<float> xax = xaxis_v.subspan(wu * to_index(njnt3), to_index(njnt3));
        const std::span<const mjtNum> d_qpos = boundary_span(d_t->qpos, to_index(nq));
        const std::span<const mjtNum> d_qvel = boundary_span(d_t->qvel, to_index(nv));
        const std::span<const mjtNum> d_xpos = boundary_span(d_t->xpos, to_index(nbody3));
        const std::span<const mjtNum> d_xmat = boundary_span(d_t->xmat, to_index(nbody9));
        const std::span<const mjtNum> d_xanchor = boundary_span(d_t->xanchor, to_index(njnt3));
        const std::span<const mjtNum> d_xaxis = boundary_span(d_t->xaxis, to_index(njnt3));
        for (std::size_t i = 0; i < to_index(nq); ++i) qp[i] = static_cast<float>(d_qpos[i]);
        for (std::size_t i = 0; i < to_index(nv); ++i) qv[i] = static_cast<float>(d_qvel[i]);
        for (std::size_t i = 0; i < to_index(nbody3); ++i) xp[i] = static_cast<float>(d_xpos[i]);
        for (std::size_t i = 0; i < to_index(nbody9); ++i) xm[i] = static_cast<float>(d_xmat[i]);
        for (std::size_t i = 0; i < to_index(njnt3); ++i) {
            xanc[i] = static_cast<float>(d_xanchor[i]);
            xax[i] = static_cast<float>(d_xaxis[i]);
        }

        if (idx.fork_travel_qpos >= 0 && idx.fork_travel_qpos < nq) {
            const float fk = qp[to_index(idx.fork_travel_qpos)];
            if (fk > max_fork_m[wu]) {
                max_fork_m[wu] = fk;
            }
        }
        if (idx.shock_stroke_qpos >= 0 && idx.shock_stroke_qpos < nq) {
            const float sh = qp[to_index(idx.shock_stroke_qpos)];
            if (sh > max_shock_m[wu]) {
                max_shock_m[wu] = sh;
            }
        }
    };

    const int n_aux_threads = std::max(0, n_workers - 1);
    std::vector<std::thread> pool;
    pool.reserve(static_cast<std::size_t>(n_aux_threads));
    for (int t = 1; t < n_workers; ++t) {
        pool.emplace_back([&, t]() {
            int local_epoch = 0;
            while (true) {
                epoch.wait(local_epoch, std::memory_order_acquire);
                if (stop_pool.load(std::memory_order_acquire)) {
                    return;
                }
                local_epoch = epoch.load(std::memory_order_relaxed);
                const int w_end = phase_w_end.load(std::memory_order_relaxed);
                const int step = phase_step.load(std::memory_order_relaxed);
                while (true) {
                    const int w = phase_cursor.fetch_add(1, std::memory_order_relaxed);
                    if (w >= w_end) {
                        break;
                    }
                    step_world_cpu(w, t, step);
                }
                const int prev_done = done_count.fetch_add(1, std::memory_order_release);
                if (prev_done + 1 == n_aux_threads) {
                    done_count.notify_all();
                }
            }
        });
    }

    auto run_cpu_slice = [&](int w_begin, int w_end, int step) {
        phase_w_begin.store(w_begin, std::memory_order_relaxed);
        phase_w_end.store(w_end, std::memory_order_relaxed);
        phase_step.store(step, std::memory_order_relaxed);
        phase_cursor.store(w_begin, std::memory_order_relaxed);
        done_count.store(0, std::memory_order_relaxed);
        epoch.fetch_add(1, std::memory_order_release);
        epoch.notify_all();

        while (true) {
            const int w = phase_cursor.fetch_add(1, std::memory_order_relaxed);
            if (w >= w_end) {
                break;
            }
            step_world_cpu(w, 0, step);
        }
        while (true) {
            const int cur_done = done_count.load(std::memory_order_acquire);
            if (cur_done >= n_aux_threads) {
                break;
            }
            done_count.wait(cur_done, std::memory_order_acquire);
        }
    };

    const auto t_start = std::chrono::steady_clock::now();

    if (gpu_ok && nworld >= 64 && n_workers > 1) {
        const int half = nworld / 2;
        id<MTLCommandBuffer> cb_h0 = dispatch_gpu_slice_async(0, half);
        if (!wait_command_buffer_ok(cb_h0, "bike_metal_run_hybrid_sweep")) {
            gpu_ok = false;
        }

        for (int step = 0; step < n_steps && gpu_ok; ++step) {
            id<MTLCommandBuffer> cb_h1 = dispatch_gpu_slice_async(half, nworld - half);
            run_cpu_slice(0, half, step);
            if (!wait_command_buffer_ok(cb_h1, "bike_metal_run_hybrid_sweep")) {
                gpu_ok = false;
                break;
            }

            if (step + 1 < n_steps) {
                cb_h0 = dispatch_gpu_slice_async(0, half);
                run_cpu_slice(half, nworld, step);
                if (!wait_command_buffer_ok(cb_h0, "bike_metal_run_hybrid_sweep")) {
                    gpu_ok = false;
                    break;
                }
            } else {
                run_cpu_slice(half, nworld, step);
            }
        }
    } else if (gpu_ok) {
        for (int step = 0; step < n_steps; ++step) {
            id<MTLCommandBuffer> cb = dispatch_gpu_slice_async(0, nworld);
            if (!wait_command_buffer_ok(cb, "bike_metal_run_hybrid_sweep")) {
                gpu_ok = false;
                break;
            }
            run_cpu_slice(0, nworld, step);
        }
    }

    const auto t_end = std::chrono::steady_clock::now();
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();

    stop_pool.store(true, std::memory_order_release);
    epoch.fetch_add(1, std::memory_order_release);
    epoch.notify_all();
    for (auto &th : pool) {
        th.join();
    }

    if (gpu_ok) {
        const std::span<float> out_fork =
            (out_peak_fork_mm != nullptr) ? boundary_span(out_peak_fork_mm, nworld_u)
                                          : std::span<float>{};
        const std::span<float> out_shock =
            (out_peak_shock_mm != nullptr) ? boundary_span(out_peak_shock_mm, nworld_u)
                                           : std::span<float>{};
        const std::span<float> out_root_x =
            (out_final_root_x_m != nullptr) ? boundary_span(out_final_root_x_m, nworld_u)
                                            : std::span<float>{};
        int all_finite = 1;
        for (int w = 0; w < nworld; ++w) {
            const std::size_t wu = to_index(w);
            const std::span<const float> qp = qpos_v.subspan(wu * to_index(nq), to_index(nq));
            const std::span<const float> qv = qvel_v.subspan(wu * to_index(nv), to_index(nv));
            for (std::size_t i = 0; i < to_index(nq); ++i) {
                if (!std::isfinite(qp[i])) {
                    all_finite = 0;
                    break;
                }
            }
            for (std::size_t i = 0; i < to_index(nv); ++i) {
                if (!std::isfinite(qv[i])) {
                    all_finite = 0;
                    break;
                }
            }
            if (!out_fork.empty()) {
                out_fork[wu] = max_fork_m[wu] * 1000.0f;
            }
            if (!out_shock.empty()) {
                out_shock[wu] = max_shock_m[wu] * 1000.0f;
            }
            if (!out_root_x.empty()) {
                out_root_x[wu] =
                    (idx.root_x_qpos >= 0 && idx.root_x_qpos < nq) ? qp[to_index(idx.root_x_qpos)] : 0.0f;
            }
        }

        if (out_all_finite != nullptr) {
            *out_all_finite = all_finite;
        }
        if (out_elapsed_ms != nullptr) {
            *out_elapsed_ms = elapsed_ms;
        }
    } else {
        std::cerr << "[bike_metal_run_hybrid_sweep] GPU command buffer failed; run aborted\n";
    }

    for (int w = 0; w < nworld; ++w) {
        mj_deleteData(world_data[to_index(w)]);
    }
    for (int t = 0; t < n_workers; ++t) {
        mj_deleteModel(worker_models[to_index(t)]);
    }
    mj_deleteModel(m_base);
    return gpu_ok ? 0 : -7;
}

int bike_metal_launch_step_writers(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    float *d_qfrc_applied,
    float *d_ctrl,
    float *d_tendon_range,
    int tendon_range_world_stride
) {
    return bike_metal_launch_writers(
        nworld, dt_s, d_idx, d_tyre_model, d_terrain_desc, d_terrain_f32, d_terrain_i32,
        d_configs, per_world_config, d_states, d_qpos, d_qvel,
        d_xpos, d_xmat, d_xanchor, d_xaxis,
        d_qfrc_applied, d_ctrl, d_tendon_range, tendon_range_world_stride
    );
}

int bike_metal_run_hybrid_batch(
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
    return bike_metal_run_hybrid_sweep(
        xml_path, nworld, n_steps, num_threads, d_idx,
        d_tyre_model, d_terrain_desc, d_terrain_f32, d_terrain_i32,
        d_configs, per_world_config, init_qpos, init_qvel,
        out_peak_fork_mm, out_peak_shock_mm, out_final_root_x_m,
        out_all_finite, out_elapsed_ms
    );
}

int bike_metal_build_tyre_and_terrain(
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

int bike_metal_build_tyre_and_terrain_for_surface(
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

int bike_metal_sizeof_sp3_config() {
    return static_cast<int>(sizeof(bike::gpu::GpuSp3Config));
}

int bike_metal_sizeof_sp3_world_state() {
    return static_cast<int>(sizeof(bike::gpu::GpuSp3WorldState));
}

int bike_metal_init_default_sp3_config(bike::gpu::GpuSp3Config *cfg) {
    if (cfg == nullptr) {
        return -1;
    }
    bike::gpu::gpu_init_default_sp3_config(*cfg);
    return 0;
}

int bike_metal_launch_sp3_init(
    int nworld,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuWorldState *d_world_states,
    const float *d_qpos,
    float init_grade,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride
) {
    if (nworld <= 0 || ntendon < 0 || tendon_range_world_stride < 0 ||
        d_idx == nullptr || d_sp3_configs == nullptr || d_sp3_states == nullptr ||
        !std::isfinite(init_grade)) {
        return -1;
    }
    const bike::gpu::GpuModelIndices idx_val = *d_idx;
    if (!bike::gpu::gpu_validate_model_indices(idx_val, ntendon)) {
        return -1;
    }
    if (d_tendon_range != nullptr && ntendon > 0 &&
        ((tendon_range_world_stride > 0 && tendon_range_world_stride < 2 * ntendon) ||
         (tendon_range_world_stride == 0 && nworld > 1))) {
        return -1;
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_sp3_init) {
        return -2;
    }
    const int n_cfg = per_world_config ? nworld : 1;
    const int has_world_states = (d_world_states != nullptr) ? 1 : 0;
    const int has_qpos = (d_qpos != nullptr && idx_val.nq > 0) ? 1 : 0;
    const int has_tendon_range =
        (d_tendon_range != nullptr && ntendon > 0 &&
         (tendon_range_world_stride >= 2 * ntendon ||
          (tendon_range_world_stride == 0 && nworld == 1)))
            ? 1
            : 0;
    const std::size_t nworld_u = to_index(nworld);
    const std::size_t tr_elems = has_tendon_range
        ? (tendon_range_world_stride >= 2 * ntendon
               ? (nworld_u * to_index(tendon_range_world_stride))
               : to_index(2 * ntendon))
        : 1ULL;

    @autoreleasepool {
        id<MTLBuffer> b_cfg = [ctx.device newBufferWithBytes:d_sp3_configs
                                                      length:to_index(n_cfg) * sizeof(bike::gpu::GpuSp3Config)
                                                     options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_sp3 = [ctx.device newBufferWithBytes:d_sp3_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp3WorldState)
                                                     options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_ws = has_world_states
            ? [ctx.device newBufferWithBytes:d_world_states
                                      length:nworld_u * sizeof(bike::gpu::GpuWorldState)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(bike::gpu::GpuWorldState)
                                       options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_qpos = has_qpos
            ? [ctx.device newBufferWithBytes:d_qpos
                                      length:nworld_u * to_index(idx_val.nq) * sizeof(float)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(float) options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_tr = has_tendon_range
            ? [ctx.device newBufferWithBytes:d_tendon_range
                                      length:tr_elems * sizeof(float)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(float) options:MTLResourceStorageModeShared];

        if (!b_cfg || !b_sp3 || !b_ws || !b_qpos || !b_tr) {
            return -3;
        }

        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        [enc setComputePipelineState:ctx.pso_sp3_init];
        [enc setBytes:&nworld length:sizeof(int) atIndex:0];
        [enc setBytes:&idx_val length:sizeof(bike::gpu::GpuModelIndices) atIndex:1];
        [enc setBuffer:b_cfg offset:0 atIndex:2];
        [enc setBytes:&per_world_config length:sizeof(int) atIndex:3];
        [enc setBuffer:b_sp3 offset:0 atIndex:4];
        [enc setBytes:&has_world_states length:sizeof(int) atIndex:5];
        [enc setBuffer:b_ws offset:0 atIndex:6];
        [enc setBytes:&has_qpos length:sizeof(int) atIndex:7];
        [enc setBuffer:b_qpos offset:0 atIndex:8];
        [enc setBytes:&init_grade length:sizeof(float) atIndex:9];
        [enc setBytes:&ntendon length:sizeof(int) atIndex:10];
        [enc setBytes:&has_tendon_range length:sizeof(int) atIndex:11];
        [enc setBuffer:b_tr offset:0 atIndex:12];
        [enc setBytes:&tendon_range_world_stride length:sizeof(int) atIndex:13];
        const NSUInteger total_threads = static_cast<NSUInteger>(nworld);
        const NSUInteger tg = std::min<NSUInteger>(64, std::max<NSUInteger>(1, total_threads));
        [enc dispatchThreads:MTLSizeMake(total_threads, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_launch_sp3_init")) {
            return -4;
        }

        const auto sp3_src = boundary_span(
            static_cast<const bike::gpu::GpuSp3WorldState *>([b_sp3 contents]), nworld_u);
        std::ranges::copy(sp3_src, boundary_span(d_sp3_states, nworld_u).begin());
        if (has_world_states) {
            const auto ws_src = boundary_span(
                static_cast<const bike::gpu::GpuWorldState *>([b_ws contents]), nworld_u);
            std::ranges::copy(ws_src, boundary_span(d_world_states, nworld_u).begin());
        }
        if (has_tendon_range) {
            const auto tr_src = boundary_span(
                static_cast<const float *>([b_tr contents]), tr_elems);
            std::ranges::copy(tr_src, boundary_span(d_tendon_range, tr_elems).begin());
        }
    }
    return 0;
}

int bike_metal_launch_sp3_pre_solve(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuWorldState *d_world_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    float *d_qfrc_applied,
    float *d_ctrl,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride,
    float *d_dof_frictionloss,
    int dof_frictionloss_world_stride
) {
    if (nworld <= 0 || !std::isfinite(dt_s) || dt_s <= 0.0f ||
        ntendon < 0 || tendon_range_world_stride < 0 || dof_frictionloss_world_stride < 0 ||
        d_idx == nullptr || d_tyre_model == nullptr || d_terrain_desc == nullptr ||
        d_sp3_configs == nullptr || d_sp3_states == nullptr || d_world_states == nullptr ||
        d_qpos == nullptr || d_qvel == nullptr || d_xpos == nullptr || d_xmat == nullptr ||
        d_xanchor == nullptr || d_xaxis == nullptr || d_qfrc_applied == nullptr || d_ctrl == nullptr) {
        return -1;
    }
    const bike::gpu::GpuModelIndices idx_val = *d_idx;
    if (!bike::gpu::gpu_validate_model_indices(idx_val, ntendon)) {
        return -1;
    }
    if (d_tendon_range != nullptr && ntendon > 0 &&
        ((tendon_range_world_stride > 0 && tendon_range_world_stride < 2 * ntendon) ||
         (tendon_range_world_stride == 0 && nworld > 1))) {
        return -1;
    }
    if (d_dof_frictionloss != nullptr &&
        ((dof_frictionloss_world_stride > 0 && dof_frictionloss_world_stride < idx_val.nv) ||
         (dof_frictionloss_world_stride == 0 && nworld > 1))) {
        return -1;
    }
    if (bike::gpu::validate_host_tyre_and_terrain(
            d_tyre_model,
            idx_val.nq,
            idx_val.nv,
            idx_val.nbody,
            d_terrain_desc,
            d_terrain_f32,
            d_terrain_i32) != 0) {
        return -1;
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_sp3_pre_solve) {
        return -2;
    }

    const int nq = idx_val.nq;
    const int nv = idx_val.nv;
    const int nu = idx_val.nu;
    const int nbody = idx_val.nbody;
    const int njnt = d_tyre_model->njnt;
    const int n_cfg = per_world_config ? nworld : 1;
    const int has_tendon_range =
        (d_tendon_range != nullptr && ntendon > 0 &&
         (tendon_range_world_stride >= 2 * ntendon ||
          (tendon_range_world_stride == 0 && nworld == 1)))
            ? 1
            : 0;
    const int has_dof_frictionloss =
        (d_dof_frictionloss != nullptr &&
         (dof_frictionloss_world_stride >= nv ||
          (dof_frictionloss_world_stride == 0 && nworld == 1)))
            ? 1
            : 0;
    const std::size_t nworld_u = to_index(nworld);
    const std::size_t tr_elems = has_tendon_range
        ? (tendon_range_world_stride >= 2 * ntendon
               ? (nworld_u * to_index(tendon_range_world_stride))
               : to_index(2 * ntendon))
        : 1ULL;
    const std::size_t fl_elems = has_dof_frictionloss
        ? (dof_frictionloss_world_stride >= nv
               ? (nworld_u * to_index(dof_frictionloss_world_stride))
               : to_index(nv))
        : 1ULL;

    @autoreleasepool {
        id<MTLBuffer> b_tyre = [ctx.device newBufferWithBytes:d_tyre_model
                                                       length:sizeof(bike::physics_core::GpuTyreModel)
                                                      options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_tf32 = (d_terrain_desc->f32_count > 0 && d_terrain_f32 != nullptr)
            ? [ctx.device newBufferWithBytes:d_terrain_f32
                                      length:to_index(d_terrain_desc->f32_count) * sizeof(float)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(float) options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_ti32 = (d_terrain_desc->i32_count > 0 && d_terrain_i32 != nullptr)
            ? [ctx.device newBufferWithBytes:d_terrain_i32
                                      length:to_index(d_terrain_desc->i32_count) * sizeof(bike::physics_core::i32)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(bike::physics_core::i32) options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_cfg = [ctx.device newBufferWithBytes:d_sp3_configs
                                                      length:to_index(n_cfg) * sizeof(bike::gpu::GpuSp3Config)
                                                     options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_sp3 = [ctx.device newBufferWithBytes:d_sp3_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp3WorldState)
                                                     options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_ws = [ctx.device newBufferWithBytes:d_world_states
                                                     length:nworld_u * sizeof(bike::gpu::GpuWorldState)
                                                    options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_qpos = [ctx.device newBufferWithBytes:d_qpos
                                                       length:nworld_u * to_index(nq) * sizeof(float)
                                                      options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_qvel = [ctx.device newBufferWithBytes:d_qvel
                                                       length:nworld_u * to_index(nv) * sizeof(float)
                                                      options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_xpos = [ctx.device newBufferWithBytes:d_xpos
                                                       length:nworld_u * to_index(nbody * 3) * sizeof(float)
                                                      options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_xmat = [ctx.device newBufferWithBytes:d_xmat
                                                       length:nworld_u * to_index(nbody * 9) * sizeof(float)
                                                      options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_xanchor = [ctx.device newBufferWithBytes:d_xanchor
                                                          length:nworld_u * to_index(njnt * 3) * sizeof(float)
                                                         options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_xaxis = [ctx.device newBufferWithBytes:d_xaxis
                                                        length:nworld_u * to_index(njnt * 3) * sizeof(float)
                                                       options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_qfrc = [ctx.device newBufferWithLength:nworld_u * to_index(nv) * sizeof(float)
                                                       options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_ctrl = [ctx.device newBufferWithLength:nworld_u * to_index(std::max(1, nu)) * sizeof(float)
                                                       options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_tr = has_tendon_range
            ? [ctx.device newBufferWithBytes:d_tendon_range
                                      length:tr_elems * sizeof(float)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(float) options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_fl = has_dof_frictionloss
            ? [ctx.device newBufferWithBytes:d_dof_frictionloss
                                      length:fl_elems * sizeof(float)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(float) options:MTLResourceStorageModeShared];
        const bool is_pneum =
            (d_tyre_model->backend == bike::physics_core::TyreBackendKind::pneumatic);
        id<MTLBuffer> b_pneum = ctx.ensure_pneum_scratch(nworld, is_pneum);

        if (!b_tyre || !b_tf32 || !b_ti32 || !b_cfg || !b_sp3 || !b_ws ||
            !b_qpos || !b_qvel || !b_xpos || !b_xmat || !b_xanchor || !b_xaxis ||
            !b_qfrc || !b_ctrl || !b_tr || !b_fl || !b_pneum) {
            return -3;
        }

        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        [enc setComputePipelineState:ctx.pso_sp3_pre_solve];
        [enc setBytes:&nworld length:sizeof(int) atIndex:0];
        [enc setBytes:&dt_s length:sizeof(float) atIndex:1];
        [enc setBytes:&idx_val length:sizeof(bike::gpu::GpuModelIndices) atIndex:2];
        [enc setBuffer:b_tyre offset:0 atIndex:3];
        [enc setBytes:d_terrain_desc length:sizeof(bike::physics_core::GpuTerrainTableDesc) atIndex:4];
        [enc setBuffer:b_tf32 offset:0 atIndex:5];
        [enc setBuffer:b_ti32 offset:0 atIndex:6];
        [enc setBuffer:b_cfg offset:0 atIndex:7];
        [enc setBytes:&per_world_config length:sizeof(int) atIndex:8];
        [enc setBuffer:b_sp3 offset:0 atIndex:9];
        [enc setBuffer:b_ws offset:0 atIndex:10];
        [enc setBuffer:b_qpos offset:0 atIndex:11];
        [enc setBuffer:b_qvel offset:0 atIndex:12];
        [enc setBuffer:b_xpos offset:0 atIndex:13];
        [enc setBuffer:b_xmat offset:0 atIndex:14];
        [enc setBuffer:b_xanchor offset:0 atIndex:15];
        [enc setBuffer:b_xaxis offset:0 atIndex:16];
        [enc setBuffer:b_qfrc offset:0 atIndex:17];
        [enc setBuffer:b_ctrl offset:0 atIndex:18];
        [enc setBytes:&ntendon length:sizeof(int) atIndex:19];
        [enc setBytes:&has_tendon_range length:sizeof(int) atIndex:20];
        [enc setBuffer:b_tr offset:0 atIndex:21];
        [enc setBytes:&tendon_range_world_stride length:sizeof(int) atIndex:22];
        [enc setBytes:&has_dof_frictionloss length:sizeof(int) atIndex:23];
        [enc setBuffer:b_fl offset:0 atIndex:24];
        [enc setBytes:&dof_frictionloss_world_stride length:sizeof(int) atIndex:25];
        [enc setBuffer:b_pneum offset:0 atIndex:26];
        const NSUInteger total_threads = static_cast<NSUInteger>(nworld);
        const NSUInteger tg = std::min<NSUInteger>(64, std::max<NSUInteger>(1, total_threads));
        [enc dispatchThreads:MTLSizeMake(total_threads, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_launch_sp3_pre_solve")) {
            return -4;
        }

        const std::size_t qfrc_elems = nworld_u * to_index(nv);
        const std::size_t ctrl_elems = nworld_u * to_index(nu);
        std::ranges::copy(
            boundary_span(static_cast<const float *>([b_qfrc contents]), qfrc_elems),
            boundary_span(d_qfrc_applied, qfrc_elems).begin());
        if (nu > 0) {
            std::ranges::copy(
                boundary_span(static_cast<const float *>([b_ctrl contents]), ctrl_elems),
                boundary_span(d_ctrl, ctrl_elems).begin());
        }
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp3WorldState *>([b_sp3 contents]), nworld_u),
            boundary_span(d_sp3_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuWorldState *>([b_ws contents]), nworld_u),
            boundary_span(d_world_states, nworld_u).begin());
        if (has_tendon_range) {
            std::ranges::copy(
                boundary_span(static_cast<const float *>([b_tr contents]), tr_elems),
                boundary_span(d_tendon_range, tr_elems).begin());
        }
        if (has_dof_frictionloss) {
            std::ranges::copy(
                boundary_span(static_cast<const float *>([b_fl contents]), fl_elems),
                boundary_span(d_dof_frictionloss, fl_elems).begin());
        }
    }
    return 0;
}

int bike_metal_launch_sp3_post_solve(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    float *d_qpos,
    const float *d_qvel,
    const float *d_xmat,
    int nsensordata,
    const float *d_sensordata,
    const int *d_nefc,
    int njmax,
    int efc_J_row_stride,
    const int *d_efc_type,
    const int *d_efc_id,
    const float *d_efc_J,
    const float *d_efc_force,
    const int *d_nacon,
    int nconmax,
    int contact_efc_stride,
    const int *d_contact_worldid,
    const int *d_contact_geom,
    const int *d_contact_efc_address,
    const int *d_contact_dim
) {
    if (nworld <= 0 || !std::isfinite(dt_s) || dt_s <= 0.0f ||
        nsensordata < 0 || njmax < 0 || efc_J_row_stride < 0 ||
        nconmax < 0 || contact_efc_stride < 0 ||
        d_idx == nullptr || d_sp3_configs == nullptr || d_sp3_states == nullptr ||
        d_qpos == nullptr || d_qvel == nullptr || d_xmat == nullptr) {
        return -1;
    }
    const bike::gpu::GpuModelIndices idx_val = *d_idx;
    if (!bike::gpu::gpu_validate_model_indices(idx_val)) {
        return -1;
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_sp3_post_solve) {
        return -2;
    }

    const int n_cfg = per_world_config ? nworld : 1;
    const int has_sensordata = (d_sensordata != nullptr && nsensordata > 0) ? 1 : 0;
    const int has_efc = (d_nefc != nullptr && njmax > 0) ? 1 : 0;
    const int has_contacts = (d_nacon != nullptr && nconmax > 0 && d_contact_geom != nullptr &&
                              d_contact_efc_address != nullptr)
                                 ? 1
                                 : 0;
    const int has_contact_worldid = (has_contacts && d_contact_worldid != nullptr) ? 1 : 0;
    const int has_contact_dim = (has_contacts && d_contact_dim != nullptr) ? 1 : 0;

    const std::size_t nworld_u = to_index(nworld);
    const std::size_t qpos_elems = nworld_u * to_index(idx_val.nq);
    const std::size_t efc_rows = has_efc ? (nworld_u * to_index(njmax)) : 1ULL;
    const std::size_t efc_j_elems =
        (has_efc && d_efc_J != nullptr && efc_J_row_stride > 0)
            ? (efc_rows * to_index(efc_J_row_stride))
            : 1ULL;
    const std::size_t con_elems =
        has_contacts ? (has_contact_worldid ? to_index(nconmax) : (nworld_u * to_index(nconmax))) : 1ULL;
    const std::size_t nacon_elems = has_contacts ? (has_contact_worldid ? 1ULL : nworld_u) : 1ULL;

    @autoreleasepool {
        id<MTLBuffer> b_cfg = [ctx.device newBufferWithBytes:d_sp3_configs
                                                      length:to_index(n_cfg) * sizeof(bike::gpu::GpuSp3Config)
                                                     options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_sp3 = [ctx.device newBufferWithBytes:d_sp3_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp3WorldState)
                                                     options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_qpos = [ctx.device newBufferWithBytes:d_qpos
                                                       length:qpos_elems * sizeof(float)
                                                      options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_qvel = [ctx.device newBufferWithBytes:d_qvel
                                                       length:nworld_u * to_index(idx_val.nv) * sizeof(float)
                                                      options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_xmat = [ctx.device newBufferWithBytes:d_xmat
                                                       length:nworld_u * to_index(idx_val.nbody * 9) * sizeof(float)
                                                      options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_sd = has_sensordata
            ? [ctx.device newBufferWithBytes:d_sensordata
                                      length:nworld_u * to_index(nsensordata) * sizeof(float)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(float) options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_nefc = has_efc
            ? [ctx.device newBufferWithBytes:d_nefc
                                      length:nworld_u * sizeof(int)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(int) options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_et = (has_efc && d_efc_type != nullptr)
            ? [ctx.device newBufferWithBytes:d_efc_type
                                      length:efc_rows * sizeof(int)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(int) options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_eid = (has_efc && d_efc_id != nullptr)
            ? [ctx.device newBufferWithBytes:d_efc_id
                                      length:efc_rows * sizeof(int)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(int) options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_eJ = (has_efc && d_efc_J != nullptr && efc_J_row_stride > 0)
            ? [ctx.device newBufferWithBytes:d_efc_J
                                      length:efc_j_elems * sizeof(float)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(float) options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_ef = (has_efc && d_efc_force != nullptr)
            ? [ctx.device newBufferWithBytes:d_efc_force
                                      length:efc_rows * sizeof(float)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(float) options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_nacon = has_contacts
            ? [ctx.device newBufferWithBytes:d_nacon
                                      length:nacon_elems * sizeof(int)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(int) options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_cwid = has_contact_worldid
            ? [ctx.device newBufferWithBytes:d_contact_worldid
                                      length:con_elems * sizeof(int)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(int) options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_cgeom = has_contacts
            ? [ctx.device newBufferWithBytes:d_contact_geom
                                      length:con_elems * 2ULL * sizeof(int)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(int) * 2ULL options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_cadr = has_contacts
            ? [ctx.device newBufferWithBytes:d_contact_efc_address
                                      length:con_elems * to_index(std::max(1, contact_efc_stride)) * sizeof(int)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(int) options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_cdim = has_contact_dim
            ? [ctx.device newBufferWithBytes:d_contact_dim
                                      length:con_elems * sizeof(int)
                                     options:MTLResourceStorageModeShared]
            : [ctx.device newBufferWithLength:sizeof(int) options:MTLResourceStorageModeShared];

        if (!b_cfg || !b_sp3 || !b_qpos || !b_qvel || !b_xmat || !b_sd ||
            !b_nefc || !b_et || !b_eid || !b_eJ || !b_ef ||
            !b_nacon || !b_cwid || !b_cgeom || !b_cadr || !b_cdim) {
            return -3;
        }

        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        [enc setComputePipelineState:ctx.pso_sp3_post_solve];
        [enc setBytes:&nworld length:sizeof(int) atIndex:0];
        [enc setBytes:&dt_s length:sizeof(float) atIndex:1];
        [enc setBytes:&idx_val length:sizeof(bike::gpu::GpuModelIndices) atIndex:2];
        [enc setBuffer:b_cfg offset:0 atIndex:3];
        [enc setBytes:&per_world_config length:sizeof(int) atIndex:4];
        [enc setBuffer:b_sp3 offset:0 atIndex:5];
        [enc setBuffer:b_qpos offset:0 atIndex:6];
        [enc setBuffer:b_qvel offset:0 atIndex:7];
        [enc setBuffer:b_xmat offset:0 atIndex:8];
        [enc setBytes:&has_sensordata length:sizeof(int) atIndex:9];
        [enc setBytes:&nsensordata length:sizeof(int) atIndex:10];
        [enc setBuffer:b_sd offset:0 atIndex:11];
        [enc setBytes:&has_efc length:sizeof(int) atIndex:12];
        [enc setBuffer:b_nefc offset:0 atIndex:13];
        [enc setBytes:&njmax length:sizeof(int) atIndex:14];
        [enc setBytes:&efc_J_row_stride length:sizeof(int) atIndex:15];
        [enc setBuffer:b_et offset:0 atIndex:16];
        [enc setBuffer:b_eid offset:0 atIndex:17];
        [enc setBuffer:b_eJ offset:0 atIndex:18];
        [enc setBuffer:b_ef offset:0 atIndex:19];
        [enc setBytes:&has_contacts length:sizeof(int) atIndex:20];
        [enc setBuffer:b_nacon offset:0 atIndex:21];
        [enc setBytes:&nconmax length:sizeof(int) atIndex:22];
        [enc setBytes:&contact_efc_stride length:sizeof(int) atIndex:23];
        [enc setBytes:&has_contact_worldid length:sizeof(int) atIndex:24];
        [enc setBuffer:b_cwid offset:0 atIndex:25];
        [enc setBuffer:b_cgeom offset:0 atIndex:26];
        [enc setBuffer:b_cadr offset:0 atIndex:27];
        [enc setBytes:&has_contact_dim length:sizeof(int) atIndex:28];
        [enc setBuffer:b_cdim offset:0 atIndex:29];
        const NSUInteger total_threads = static_cast<NSUInteger>(nworld);
        const NSUInteger tg = std::min<NSUInteger>(64, std::max<NSUInteger>(1, total_threads));
        [enc dispatchThreads:MTLSizeMake(total_threads, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_launch_sp3_post_solve")) {
            return -4;
        }

        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp3WorldState *>([b_sp3 contents]), nworld_u),
            boundary_span(d_sp3_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const float *>([b_qpos contents]), qpos_elems),
            boundary_span(d_qpos, qpos_elems).begin());
    }
    return 0;
}

int bike_metal_build_spindle_params(
    const char *xml_path,
    const char *strength_json_path,
    const char *envelope_json_path,
    int control_period_steps,
    bike::physics_core::SpindleM2K2Params<float> *out_params
) {
    return bike::gpu::build_runner_spindle_params(
        xml_path, strength_json_path, envelope_json_path, control_period_steps, out_params);
}

int bike_metal_sizeof_sp4_world_state() {
    return static_cast<int>(sizeof(bike::gpu::GpuSp4WorldState));
}

int bike_metal_sizeof_spindle_params() {
    return static_cast<int>(sizeof(bike::physics_core::SpindleM2K2Params<float>));
}

int bike_metal_launch_sp4_init(
    int nworld,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    bike::gpu::GpuWorldState *d_world_states,
    const float *d_qpos,
    float init_grade,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride
) {
    if (nworld <= 0 || ntendon < 0 || tendon_range_world_stride < 0 ||
        d_idx == nullptr || d_sp3_configs == nullptr ||
        d_sp3_states == nullptr || d_sp4_states == nullptr ||
        !std::isfinite(init_grade)) {
        return -1;
    }
    const bike::gpu::GpuModelIndices idx_val = *d_idx;
    if (!bike::gpu::gpu_validate_model_indices(idx_val, ntendon)) {
        return -1;
    }
    const std::size_t nworld_u = to_index(nworld);
    const std::size_t n_cfg = per_world_config ? nworld_u : 1ULL;
    const int has_ws = (d_world_states != nullptr) ? 1 : 0;
    const int has_qp = (d_qpos != nullptr) ? 1 : 0;
    const int has_tr =
        (d_tendon_range != nullptr && ntendon > 0 &&
         (tendon_range_world_stride >= 2 * ntendon ||
          (tendon_range_world_stride == 0 && nworld == 1)))
            ? 1
            : 0;
    if (d_tendon_range != nullptr && ntendon > 0 && has_tr == 0) {
        return -1;
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_sp4_init) {
        return -2;
    }
    @autoreleasepool {
        const auto opt = MTLResourceStorageModeShared;
        id<MTLBuffer> b_cfg = [ctx.device newBufferWithBytes:d_sp3_configs
                                                      length:n_cfg * sizeof(bike::gpu::GpuSp3Config)
                                                     options:opt];
        id<MTLBuffer> b_sp3 = [ctx.device newBufferWithBytes:d_sp3_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp3WorldState)
                                                     options:opt];
        id<MTLBuffer> b_sp4 = [ctx.device newBufferWithBytes:d_sp4_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp4WorldState)
                                                     options:opt];
        id<MTLBuffer> b_ws = has_ws
            ? [ctx.device newBufferWithBytes:d_world_states
                                       length:nworld_u * sizeof(bike::gpu::GpuWorldState)
                                      options:opt]
            : [ctx.device newBufferWithLength:sizeof(bike::gpu::GpuWorldState) options:opt];
        const std::size_t qp_elems = nworld_u * to_index(idx_val.nq);
        id<MTLBuffer> b_qp = has_qp
            ? [ctx.device newBufferWithBytes:d_qpos length:qp_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        const std::size_t tr_elems = has_tr
            ? ((tendon_range_world_stride > 0) ? nworld_u * to_index(tendon_range_world_stride)
                                               : to_index(2 * ntendon))
            : 4U;
        id<MTLBuffer> b_tr = has_tr
            ? [ctx.device newBufferWithBytes:d_tendon_range length:tr_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        if (!b_cfg || !b_sp3 || !b_sp4 || !b_ws || !b_qp || !b_tr) {
            return -3;
        }
        BikeMetalSp4InitScalars sc{};
        sc.idx = idx_val;
        sc.nworld = nworld;
        sc.per_world_config = per_world_config;
        sc.init_grade = init_grade;
        sc.ntendon = ntendon;
        sc.tendon_range_world_stride = tendon_range_world_stride;
        sc.has_world_states = has_ws;
        sc.has_qpos = has_qp;
        sc.has_tendon_range = has_tr;

        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        [enc setComputePipelineState:ctx.pso_sp4_init];
        [enc setBytes:&sc length:sizeof(sc) atIndex:0];
        [enc setBuffer:b_cfg offset:0 atIndex:1];
        [enc setBuffer:b_sp3 offset:0 atIndex:2];
        [enc setBuffer:b_sp4 offset:0 atIndex:3];
        [enc setBuffer:b_ws offset:0 atIndex:4];
        [enc setBuffer:b_qp offset:0 atIndex:5];
        [enc setBuffer:b_tr offset:0 atIndex:6];
        const NSUInteger total_threads = static_cast<NSUInteger>(nworld);
        const NSUInteger tg = std::min<NSUInteger>(64, std::max<NSUInteger>(1, total_threads));
        [enc dispatchThreads:MTLSizeMake(total_threads, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_launch_sp4_init")) {
            return -4;
        }
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp3WorldState *>([b_sp3 contents]), nworld_u),
            boundary_span(d_sp3_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp4WorldState *>([b_sp4 contents]), nworld_u),
            boundary_span(d_sp4_states, nworld_u).begin());
        if (has_ws) {
            std::ranges::copy(
                boundary_span(static_cast<const bike::gpu::GpuWorldState *>([b_ws contents]), nworld_u),
                boundary_span(d_world_states, nworld_u).begin());
        }
        if (has_tr) {
            std::ranges::copy(
                boundary_span(static_cast<const float *>([b_tr contents]), tr_elems),
                boundary_span(d_tendon_range, tr_elems).begin());
        }
    }
    return 0;
}

int bike_metal_launch_sp4_pre_solve_and_pass0(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    bike::gpu::GpuWorldState *d_world_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_qfrc_bias,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    const float *d_site_xpos,
    const float *d_subtree_com,
    const float *d_body_subtreemass,
    int qM_row_stride,
    int M_world_stride,
    const int *d_M_rowadr,
    const int *d_M_rownnz,
    const int *d_M_colind,
    const float *d_M_or_qM,
    float *d_qfrc_applied,
    float *d_ctrl,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride,
    float *d_dof_frictionloss,
    int dof_frictionloss_world_stride,
    float *d_qacc_warmstart
) {
    if (nworld <= 0 || dt_s <= 0.0f || !std::isfinite(dt_s) ||
        d_idx == nullptr || d_tyre_model == nullptr || d_terrain_desc == nullptr ||
        d_sp3_configs == nullptr || d_spindle_params == nullptr ||
        d_sp3_states == nullptr || d_sp4_states == nullptr || d_world_states == nullptr ||
        d_qpos == nullptr || d_qvel == nullptr || d_qfrc_bias == nullptr ||
        d_xpos == nullptr || d_xmat == nullptr || d_xanchor == nullptr ||
        d_xaxis == nullptr || d_site_xpos == nullptr || d_subtree_com == nullptr ||
        d_M_or_qM == nullptr || d_qfrc_applied == nullptr || d_ctrl == nullptr ||
        M_world_stride <= 0 || ntendon < 0 ||
        tendon_range_world_stride < 0 || dof_frictionloss_world_stride < 0) {
        return -1;
    }
    const bike::gpu::GpuModelIndices idx_val = *d_idx;
    if (!bike::gpu::gpu_validate_model_indices(idx_val, ntendon) ||
        !bike::physics_core::validate_spindle_m2k2_params(d_spindle_params)) {
        return -1;
    }
    if (bike::gpu::validate_host_tyre_and_terrain(
            d_tyre_model, idx_val.nq, idx_val.nv, idx_val.nbody,
            d_terrain_desc, d_terrain_f32, d_terrain_i32) != 0) {
        return -1;
    }
    const int has_tr =
        (d_tendon_range != nullptr && ntendon > 0 &&
         (tendon_range_world_stride >= 2 * ntendon ||
          (tendon_range_world_stride == 0 && nworld == 1)))
            ? 1
            : 0;
    if (d_tendon_range != nullptr && ntendon > 0 && has_tr == 0) {
        return -1;
    }
    const int has_fl =
        (d_dof_frictionloss != nullptr &&
         (dof_frictionloss_world_stride >= idx_val.nv ||
          (dof_frictionloss_world_stride == 0 && nworld == 1)))
            ? 1
            : 0;
    if (d_dof_frictionloss != nullptr && has_fl == 0) {
        return -1;
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_sp4_pre_solve) {
        return -2;
    }
    const std::size_t nworld_u = to_index(nworld);
    const std::size_t n_cfg = per_world_config ? nworld_u : 1ULL;
    const std::size_t f32_cnt = std::max<std::size_t>(1U, to_index(d_terrain_desc->f32_count));
    const std::size_t i32_cnt = std::max<std::size_t>(1U, to_index(d_terrain_desc->i32_count));
    const std::size_t qpos_elems = nworld_u * to_index(idx_val.nq);
    const std::size_t nv_elems = nworld_u * to_index(idx_val.nv);
    const std::size_t nu_elems = nworld_u * to_index(idx_val.nu);
    const std::size_t xpos_elems = nworld_u * to_index(idx_val.nbody) * 3ULL;
    const std::size_t xmat_elems = nworld_u * to_index(idx_val.nbody) * 9ULL;
    const std::size_t j3_elems = nworld_u * to_index(d_tyre_model->njnt) * 3ULL;
    const std::size_t site_elems = nworld_u * to_index(d_spindle_params->nsite) * 3ULL;
    const std::size_t m_elems = nworld_u * to_index(M_world_stride);
    const std::size_t tr_elems = has_tr
        ? ((tendon_range_world_stride > 0) ? nworld_u * to_index(tendon_range_world_stride)
                                           : to_index(2 * ntendon))
        : 4U;
    const std::size_t fl_elems = has_fl
        ? ((dof_frictionloss_world_stride > 0) ? nworld_u * to_index(dof_frictionloss_world_stride)
                                               : to_index(idx_val.nv))
        : 4U;
    const int has_submass = (d_body_subtreemass != nullptr) ? 1 : 0;
    const int is_sparse = (qM_row_stride == 0 && d_M_rowadr != nullptr &&
                           d_M_rownnz != nullptr && d_M_colind != nullptr)
                              ? 1
                              : 0;
    if (qM_row_stride == 0 && is_sparse == 0) {
        return -1;
    }
    std::size_t colind_cnt = 1U;
    if (is_sparse != 0) {
        const auto ra = boundary_span(d_M_rowadr, to_index(idx_val.nv));
        const auto rn = boundary_span(d_M_rownnz, to_index(idx_val.nv));
        for (std::size_t r = 0; r < ra.size(); ++r) {
            if (ra[r] < 0 || rn[r] < 0 || rn[r] > idx_val.nv) {
                return -1;
            }
            const std::size_t end_idx = to_index(ra[r]) + to_index(rn[r]);
            if (end_idx > colind_cnt) {
                colind_cnt = end_idx;
            }
        }
    }

    @autoreleasepool {
        const auto opt = MTLResourceStorageModeShared;
        id<MTLBuffer> b_tyre = [ctx.device newBufferWithBytes:d_tyre_model
                                                       length:sizeof(bike::physics_core::GpuTyreModel)
                                                      options:opt];
        id<MTLBuffer> b_tf32 = (d_terrain_desc->f32_count > 0 && d_terrain_f32 != nullptr)
            ? [ctx.device newBufferWithBytes:d_terrain_f32 length:f32_cnt * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_ti32 = (d_terrain_desc->i32_count > 0 && d_terrain_i32 != nullptr)
            ? [ctx.device newBufferWithBytes:d_terrain_i32
                                       length:i32_cnt * sizeof(bike::physics_core::i32)
                                      options:opt]
            : [ctx.device newBufferWithLength:sizeof(bike::physics_core::i32) * 4U options:opt];
        id<MTLBuffer> b_cfg = [ctx.device newBufferWithBytes:d_sp3_configs
                                                      length:n_cfg * sizeof(bike::gpu::GpuSp3Config)
                                                     options:opt];
        id<MTLBuffer> b_sparams = [ctx.device newBufferWithBytes:d_spindle_params
                                                          length:sizeof(bike::physics_core::SpindleM2K2Params<float>)
                                                         options:opt];
        id<MTLBuffer> b_sp3 = [ctx.device newBufferWithBytes:d_sp3_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp3WorldState)
                                                     options:opt];
        id<MTLBuffer> b_sp4 = [ctx.device newBufferWithBytes:d_sp4_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp4WorldState)
                                                     options:opt];
        id<MTLBuffer> b_ws = [ctx.device newBufferWithBytes:d_world_states
                                                     length:nworld_u * sizeof(bike::gpu::GpuWorldState)
                                                    options:opt];
        id<MTLBuffer> b_qp = [ctx.device newBufferWithBytes:d_qpos length:qpos_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_qv = [ctx.device newBufferWithBytes:d_qvel length:nv_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_bias = [ctx.device newBufferWithBytes:d_qfrc_bias length:nv_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_xp = [ctx.device newBufferWithBytes:d_xpos length:xpos_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_xm = [ctx.device newBufferWithBytes:d_xmat length:xmat_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_xa = [ctx.device newBufferWithBytes:d_xanchor length:j3_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_xx = [ctx.device newBufferWithBytes:d_xaxis length:j3_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_sp = [ctx.device newBufferWithBytes:d_site_xpos length:site_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_sc = [ctx.device newBufferWithBytes:d_subtree_com length:xpos_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_sm = has_submass
            ? [ctx.device newBufferWithBytes:d_body_subtreemass
                                       length:to_index(idx_val.nbody) * sizeof(float)
                                      options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_ra = is_sparse
            ? [ctx.device newBufferWithBytes:d_M_rowadr length:to_index(idx_val.nv) * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_rn = is_sparse
            ? [ctx.device newBufferWithBytes:d_M_rownnz length:to_index(idx_val.nv) * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_ci = is_sparse
            ? [ctx.device newBufferWithBytes:d_M_colind length:colind_cnt * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_M = [ctx.device newBufferWithBytes:d_M_or_qM length:m_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_qfrc = [ctx.device newBufferWithBytes:d_qfrc_applied length:nv_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_ctrl = (nu_elems > 0U)
            ? [ctx.device newBufferWithBytes:d_ctrl length:nu_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_tr = has_tr
            ? [ctx.device newBufferWithBytes:d_tendon_range length:tr_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_fl = has_fl
            ? [ctx.device newBufferWithBytes:d_dof_frictionloss length:fl_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        const int has_ws = (d_qacc_warmstart != nullptr) ? 1 : 0;
        id<MTLBuffer> b_ws_qacc = has_ws
            ? [ctx.device newBufferWithBytes:d_qacc_warmstart length:nv_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        const bool is_pneum =
            (d_tyre_model->backend == bike::physics_core::TyreBackendKind::pneumatic);
        id<MTLBuffer> b_pneum = ctx.ensure_pneum_scratch(nworld, is_pneum);
        if (!b_tyre || !b_tf32 || !b_ti32 || !b_cfg || !b_sparams || !b_sp3 || !b_sp4 || !b_ws ||
            !b_qp || !b_qv || !b_bias || !b_xp || !b_xm || !b_xa || !b_xx || !b_sp || !b_sc ||
            !b_sm || !b_ra || !b_rn || !b_ci || !b_M || !b_qfrc || !b_ctrl || !b_tr || !b_fl ||
            !b_ws_qacc || !b_pneum) {
            return -3;
        }
        BikeMetalSp4PreSolveScalars sc{};
        sc.idx = idx_val;
        sc.terrain_desc = *d_terrain_desc;
        sc.nworld = nworld;
        sc.dt_s = dt_s;
        sc.per_world_config = per_world_config;
        sc.has_body_subtreemass = has_submass;
        sc.qM_row_stride = qM_row_stride;
        sc.M_world_stride = M_world_stride;
        sc.ntendon = ntendon;
        sc.has_tendon_range = has_tr;
        sc.tendon_range_world_stride = tendon_range_world_stride;
        sc.has_dof_frictionloss = has_fl;
        sc.dof_frictionloss_world_stride = dof_frictionloss_world_stride;
        sc.has_qacc_warmstart = has_ws;
        sc.has_pneum_scratch = is_pneum ? 1 : 0;

        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        [enc setComputePipelineState:ctx.pso_sp4_pre_solve];
        [enc setBytes:&sc length:sizeof(sc) atIndex:0];
        [enc setBuffer:b_tyre offset:0 atIndex:1];
        [enc setBuffer:b_tf32 offset:0 atIndex:2];
        [enc setBuffer:b_ti32 offset:0 atIndex:3];
        [enc setBuffer:b_cfg offset:0 atIndex:4];
        [enc setBuffer:b_sparams offset:0 atIndex:5];
        [enc setBuffer:b_sp3 offset:0 atIndex:6];
        [enc setBuffer:b_sp4 offset:0 atIndex:7];
        [enc setBuffer:b_ws offset:0 atIndex:8];
        [enc setBuffer:b_qp offset:0 atIndex:9];
        [enc setBuffer:b_qv offset:0 atIndex:10];
        [enc setBuffer:b_bias offset:0 atIndex:11];
        [enc setBuffer:b_xp offset:0 atIndex:12];
        [enc setBuffer:b_xm offset:0 atIndex:13];
        [enc setBuffer:b_xa offset:0 atIndex:14];
        [enc setBuffer:b_xx offset:0 atIndex:15];
        [enc setBuffer:b_sp offset:0 atIndex:16];
        [enc setBuffer:b_sc offset:0 atIndex:17];
        [enc setBuffer:b_sm offset:0 atIndex:18];
        [enc setBuffer:b_ra offset:0 atIndex:19];
        [enc setBuffer:b_rn offset:0 atIndex:20];
        [enc setBuffer:b_ci offset:0 atIndex:21];
        [enc setBuffer:b_M offset:0 atIndex:22];
        [enc setBuffer:b_qfrc offset:0 atIndex:23];
        [enc setBuffer:b_ctrl offset:0 atIndex:24];
        [enc setBuffer:b_tr offset:0 atIndex:25];
        [enc setBuffer:b_fl offset:0 atIndex:26];
        [enc setBuffer:b_ws_qacc offset:0 atIndex:27];
        [enc setBuffer:b_pneum offset:0 atIndex:28];
        const NSUInteger total_threads = static_cast<NSUInteger>(nworld);
        const NSUInteger tg = std::min<NSUInteger>(64, std::max<NSUInteger>(1, total_threads));
        [enc dispatchThreads:MTLSizeMake(total_threads, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_launch_sp4_pre_solve_and_pass0")) {
            return -4;
        }
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp3WorldState *>([b_sp3 contents]), nworld_u),
            boundary_span(d_sp3_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp4WorldState *>([b_sp4 contents]), nworld_u),
            boundary_span(d_sp4_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuWorldState *>([b_ws contents]), nworld_u),
            boundary_span(d_world_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const float *>([b_qfrc contents]), nv_elems),
            boundary_span(d_qfrc_applied, nv_elems).begin());
        if (nu_elems > 0U) {
            std::ranges::copy(
                boundary_span(static_cast<const float *>([b_ctrl contents]), nu_elems),
                boundary_span(d_ctrl, nu_elems).begin());
        }
        if (has_tr) {
            std::ranges::copy(
                boundary_span(static_cast<const float *>([b_tr contents]), tr_elems),
                boundary_span(d_tendon_range, tr_elems).begin());
        }
        if (has_fl) {
            std::ranges::copy(
                boundary_span(static_cast<const float *>([b_fl contents]), fl_elems),
                boundary_span(d_dof_frictionloss, fl_elems).begin());
        }
    }
    return 0;
}

namespace {

int bike_metal_launch_sp4_pass_stage(
    int stage,
    int nworld,
    int nq,
    int nv,
    int nu,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_qfrc_bias,
    const float *d_qacc,
    int qM_row_stride,
    int M_world_stride,
    const int *d_M_rowadr,
    const int *d_M_rownnz,
    const int *d_M_colind,
    const float *d_M_or_qM,
    float *d_ctrl,
    float *d_qacc_warmstart
) {
    if (nworld <= 0 || nq <= 0 || nv <= 0 || nu < 0 || M_world_stride <= 0 ||
        d_spindle_params == nullptr || d_sp4_states == nullptr ||
        d_qpos == nullptr || d_qvel == nullptr || d_qfrc_bias == nullptr ||
        d_qacc == nullptr || d_M_or_qM == nullptr || d_ctrl == nullptr ||
        !bike::physics_core::validate_spindle_m2k2_params(d_spindle_params)) {
        return -1;
    }
    const int is_sparse = (qM_row_stride == 0 && d_M_rowadr != nullptr &&
                           d_M_rownnz != nullptr && d_M_colind != nullptr)
                              ? 1
                              : 0;
    if (qM_row_stride == 0 && is_sparse == 0) {
        return -1;
    }
    std::size_t colind_cnt = 1U;
    if (is_sparse != 0) {
        const auto ra = boundary_span(d_M_rowadr, to_index(nv));
        const auto rn = boundary_span(d_M_rownnz, to_index(nv));
        for (std::size_t r = 0; r < ra.size(); ++r) {
            if (ra[r] < 0 || rn[r] < 0 || rn[r] > nv) {
                return -1;
            }
            const std::size_t end_idx = to_index(ra[r]) + to_index(rn[r]);
            if (end_idx > colind_cnt) {
                colind_cnt = end_idx;
            }
        }
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_sp4_pass_stage) {
        return -2;
    }
    const std::size_t nworld_u = to_index(nworld);
    const std::size_t qpos_elems = nworld_u * to_index(nq);
    const std::size_t nv_elems = nworld_u * to_index(nv);
    const std::size_t nu_elems = nworld_u * to_index(nu);
    const std::size_t m_elems = nworld_u * to_index(M_world_stride);
    @autoreleasepool {
        const auto opt = MTLResourceStorageModeShared;
        id<MTLBuffer> b_sparams = [ctx.device newBufferWithBytes:d_spindle_params
                                                          length:sizeof(bike::physics_core::SpindleM2K2Params<float>)
                                                         options:opt];
        id<MTLBuffer> b_sp4 = [ctx.device newBufferWithBytes:d_sp4_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp4WorldState)
                                                     options:opt];
        id<MTLBuffer> b_qp = [ctx.device newBufferWithBytes:d_qpos length:qpos_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_qv = [ctx.device newBufferWithBytes:d_qvel length:nv_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_bias = [ctx.device newBufferWithBytes:d_qfrc_bias length:nv_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_qacc = [ctx.device newBufferWithBytes:d_qacc length:nv_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_ra = is_sparse
            ? [ctx.device newBufferWithBytes:d_M_rowadr length:to_index(nv) * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_rn = is_sparse
            ? [ctx.device newBufferWithBytes:d_M_rownnz length:to_index(nv) * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_ci = is_sparse
            ? [ctx.device newBufferWithBytes:d_M_colind length:colind_cnt * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_M = [ctx.device newBufferWithBytes:d_M_or_qM length:m_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_ctrl = (nu_elems > 0U)
            ? [ctx.device newBufferWithBytes:d_ctrl length:nu_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        const int has_ws = (d_qacc_warmstart != nullptr) ? 1 : 0;
        id<MTLBuffer> b_ws = has_ws
            ? [ctx.device newBufferWithBytes:d_qacc_warmstart length:nv_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        if (!b_sparams || !b_sp4 || !b_qp || !b_qv || !b_bias || !b_qacc ||
            !b_ra || !b_rn || !b_ci || !b_M || !b_ctrl || !b_ws) {
            return -3;
        }
        BikeMetalSp4PassScalars sc{};
        sc.stage = stage;
        sc.nworld = nworld;
        sc.nq = nq;
        sc.nv = nv;
        sc.nu = nu;
        sc.qM_row_stride = qM_row_stride;
        sc.M_world_stride = M_world_stride;
        sc.has_qacc_warmstart = has_ws;

        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        [enc setComputePipelineState:ctx.pso_sp4_pass_stage];
        [enc setBytes:&sc length:sizeof(sc) atIndex:0];
        [enc setBuffer:b_sparams offset:0 atIndex:1];
        [enc setBuffer:b_sp4 offset:0 atIndex:2];
        [enc setBuffer:b_qp offset:0 atIndex:3];
        [enc setBuffer:b_qv offset:0 atIndex:4];
        [enc setBuffer:b_bias offset:0 atIndex:5];
        [enc setBuffer:b_qacc offset:0 atIndex:6];
        [enc setBuffer:b_ra offset:0 atIndex:7];
        [enc setBuffer:b_rn offset:0 atIndex:8];
        [enc setBuffer:b_ci offset:0 atIndex:9];
        [enc setBuffer:b_M offset:0 atIndex:10];
        [enc setBuffer:b_ctrl offset:0 atIndex:11];
        [enc setBuffer:b_ws offset:0 atIndex:12];
        const NSUInteger total_threads = static_cast<NSUInteger>(nworld);
        const NSUInteger tg = std::min<NSUInteger>(64, std::max<NSUInteger>(1, total_threads));
        [enc dispatchThreads:MTLSizeMake(total_threads, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_launch_sp4_pass_stage")) {
            return -4;
        }
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp4WorldState *>([b_sp4 contents]), nworld_u),
            boundary_span(d_sp4_states, nworld_u).begin());
        if (nu_elems > 0U) {
            std::ranges::copy(
                boundary_span(static_cast<const float *>([b_ctrl contents]), nu_elems),
                boundary_span(d_ctrl, nu_elems).begin());
        }
        if (has_ws) {
            std::ranges::copy(
                boundary_span(static_cast<const float *>([b_ws contents]), nv_elems),
                boundary_span(d_qacc_warmstart, nv_elems).begin());
        }
    }
    return 0;
}

} // namespace

int bike_metal_launch_sp4_pass1(
    int nworld, int nq, int nv, int nu,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    const float *d_qpos, const float *d_qvel, const float *d_qfrc_bias, const float *d_qacc,
    int qM_row_stride, int M_world_stride,
    const int *d_M_rowadr, const int *d_M_rownnz, const int *d_M_colind,
    const float *d_M_or_qM, float *d_ctrl, float *d_qacc_warmstart
) {
    return bike_metal_launch_sp4_pass_stage(
        1, nworld, nq, nv, nu, d_spindle_params, d_sp4_states,
        d_qpos, d_qvel, d_qfrc_bias, d_qacc,
        qM_row_stride, M_world_stride, d_M_rowadr, d_M_rownnz, d_M_colind,
        d_M_or_qM, d_ctrl, d_qacc_warmstart);
}

int bike_metal_launch_sp4_pass2(
    int nworld, int nq, int nv, int nu,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    const float *d_qpos, const float *d_qvel, const float *d_qfrc_bias, const float *d_qacc,
    int qM_row_stride, int M_world_stride,
    const int *d_M_rowadr, const int *d_M_rownnz, const int *d_M_colind,
    const float *d_M_or_qM, float *d_ctrl, float *d_qacc_warmstart
) {
    return bike_metal_launch_sp4_pass_stage(
        2, nworld, nq, nv, nu, d_spindle_params, d_sp4_states,
        d_qpos, d_qvel, d_qfrc_bias, d_qacc,
        qM_row_stride, M_world_stride, d_M_rowadr, d_M_rownnz, d_M_colind,
        d_M_or_qM, d_ctrl, d_qacc_warmstart);
}

int bike_metal_launch_sp4_pass3(
    int nworld, int nq, int nv, int nu,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    const float *d_qpos, const float *d_qvel, const float *d_qfrc_bias, const float *d_qacc,
    int qM_row_stride, int M_world_stride,
    const int *d_M_rowadr, const int *d_M_rownnz, const int *d_M_colind,
    const float *d_M_or_qM, float *d_ctrl, float *d_qacc_warmstart
) {
    return bike_metal_launch_sp4_pass_stage(
        3, nworld, nq, nv, nu, d_spindle_params, d_sp4_states,
        d_qpos, d_qvel, d_qfrc_bias, d_qacc,
        qM_row_stride, M_world_stride, d_M_rowadr, d_M_rownnz, d_M_colind,
        d_M_or_qM, d_ctrl, d_qacc_warmstart);
}

int bike_metal_launch_sp4_post_solve(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    int nsensordata,
    const float *d_sensordata,
    const int *d_nefc,
    int njmax,
    int efc_J_row_stride,
    const int *d_efc_type,
    const int *d_efc_id,
    const float *d_efc_J,
    const float *d_efc_force,
    const int *d_nacon,
    int nconmax,
    int contact_efc_stride,
    const int *d_contact_worldid,
    const int *d_contact_geom,
    const int *d_contact_efc_address,
    const int *d_contact_dim
) {
    if (nworld <= 0 || dt_s <= 0.0f || !std::isfinite(dt_s) ||
        d_idx == nullptr || d_sp3_configs == nullptr || d_spindle_params == nullptr ||
        d_sp3_states == nullptr || d_sp4_states == nullptr ||
        d_qpos == nullptr || d_qvel == nullptr || d_xmat == nullptr ||
        nsensordata < 0 || njmax < 0 || efc_J_row_stride < 0 || nconmax < 0 ||
        contact_efc_stride < 0 || (nconmax > 0 && contact_efc_stride == 0)) {
        return -1;
    }
    const bike::gpu::GpuModelIndices idx_val = *d_idx;
    if (!bike::gpu::gpu_validate_model_indices(idx_val)) {
        return -1;
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_sp4_post_solve) {
        return -2;
    }
    const std::size_t nworld_u = to_index(nworld);
    const std::size_t n_cfg = per_world_config ? nworld_u : 1ULL;
    const std::size_t qpos_elems = nworld_u * to_index(idx_val.nq);
    const std::size_t qvel_elems = nworld_u * to_index(idx_val.nv);
    const std::size_t xpos_elems = nworld_u * to_index(idx_val.nbody) * 3ULL;
    const std::size_t xmat_elems = nworld_u * to_index(idx_val.nbody) * 9ULL;
    const int nj = d_spindle_params->njnt;
    const int has_xp = (d_xpos != nullptr) ? 1 : 0;
    const int has_xa = (d_xanchor != nullptr && nj > 0) ? 1 : 0;
    const std::size_t xa_elems = has_xa ? (nworld_u * to_index(nj) * 3ULL) : 4U;
    const int has_sd = (d_sensordata != nullptr && nsensordata > 0) ? 1 : 0;
    const int has_efc = (d_nefc != nullptr && njmax > 0) ? 1 : 0;
    const int has_con =
        (d_nacon != nullptr && nconmax > 0 && d_contact_geom != nullptr &&
         d_contact_efc_address != nullptr)
            ? 1
            : 0;
    const int has_cwid = (has_con != 0 && d_contact_worldid != nullptr) ? 1 : 0;
    const int has_cdim = (has_con != 0 && d_contact_dim != nullptr) ? 1 : 0;

    @autoreleasepool {
        const auto opt = MTLResourceStorageModeShared;
        id<MTLBuffer> b_cfg = [ctx.device newBufferWithBytes:d_sp3_configs
                                                      length:n_cfg * sizeof(bike::gpu::GpuSp3Config)
                                                     options:opt];
        id<MTLBuffer> b_sp = [ctx.device newBufferWithBytes:d_spindle_params
                                                     length:sizeof(bike::physics_core::SpindleM2K2Params<float>)
                                                    options:opt];
        id<MTLBuffer> b_sp3 = [ctx.device newBufferWithBytes:d_sp3_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp3WorldState)
                                                     options:opt];
        id<MTLBuffer> b_sp4 = [ctx.device newBufferWithBytes:d_sp4_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp4WorldState)
                                                     options:opt];
        id<MTLBuffer> b_qpos = [ctx.device newBufferWithBytes:d_qpos length:qpos_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_qvel = [ctx.device newBufferWithBytes:d_qvel length:qvel_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_xp = has_xp
            ? [ctx.device newBufferWithBytes:d_xpos length:xpos_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_xmat = [ctx.device newBufferWithBytes:d_xmat length:xmat_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_xa = has_xa
            ? [ctx.device newBufferWithBytes:d_xanchor length:xa_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_sd = has_sd
            ? [ctx.device newBufferWithBytes:d_sensordata
                                       length:nworld_u * to_index(nsensordata) * sizeof(float)
                                      options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        const std::size_t efc_elems = has_efc ? (nworld_u * to_index(njmax)) : 4U;
        const std::size_t eJ_elems =
            (has_efc && d_efc_J != nullptr && efc_J_row_stride > 0)
                ? (efc_elems * to_index(efc_J_row_stride))
                : 4U;
        id<MTLBuffer> b_nefc = has_efc
            ? [ctx.device newBufferWithBytes:d_nefc length:nworld_u * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_et = (has_efc && d_efc_type != nullptr)
            ? [ctx.device newBufferWithBytes:d_efc_type length:efc_elems * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_eid = (has_efc && d_efc_id != nullptr)
            ? [ctx.device newBufferWithBytes:d_efc_id length:efc_elems * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_eJ = (has_efc && d_efc_J != nullptr && efc_J_row_stride > 0)
            ? [ctx.device newBufferWithBytes:d_efc_J length:eJ_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_ef = (has_efc && d_efc_force != nullptr)
            ? [ctx.device newBufferWithBytes:d_efc_force length:efc_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];

        const std::size_t nacon_cnt = has_con ? (has_cwid ? 1U : nworld_u) : 4U;
        const std::size_t con_tot =
            has_con ? (has_cwid ? to_index(nconmax) : (nworld_u * to_index(nconmax))) : 4U;
        id<MTLBuffer> b_nacon = has_con
            ? [ctx.device newBufferWithBytes:d_nacon length:nacon_cnt * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_cwid = has_cwid
            ? [ctx.device newBufferWithBytes:d_contact_worldid length:con_tot * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_cgeom = has_con
            ? [ctx.device newBufferWithBytes:d_contact_geom length:con_tot * 2U * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_cadr = has_con
            ? [ctx.device newBufferWithBytes:d_contact_efc_address
                                       length:con_tot * to_index(contact_efc_stride) * sizeof(int)
                                      options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_cdim = has_cdim
            ? [ctx.device newBufferWithBytes:d_contact_dim length:con_tot * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];

        if (!b_cfg || !b_sp || !b_sp3 || !b_sp4 || !b_qpos || !b_qvel || !b_xp || !b_xmat ||
            !b_xa || !b_sd || !b_nefc || !b_et || !b_eid || !b_eJ || !b_ef ||
            !b_nacon || !b_cwid || !b_cgeom || !b_cadr || !b_cdim) {
            return -3;
        }
        BikeMetalSp4PostSolveScalars sc{};
        sc.idx = idx_val;
        sc.nworld = nworld;
        sc.dt_s = dt_s;
        sc.per_world_config = per_world_config;
        sc.has_xpos = has_xp;
        sc.has_xanchor = has_xa;
        sc.has_sensordata = has_sd;
        sc.nsensordata = nsensordata;
        sc.has_efc = has_efc;
        sc.njmax = njmax;
        sc.efc_J_row_stride = efc_J_row_stride;
        sc.has_contacts = has_con;
        sc.nconmax = nconmax;
        sc.contact_efc_stride = contact_efc_stride;
        sc.has_contact_worldid = has_cwid;
        sc.has_contact_dim = has_cdim;

        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        [enc setComputePipelineState:ctx.pso_sp4_post_solve];
        [enc setBytes:&sc length:sizeof(sc) atIndex:0];
        [enc setBuffer:b_cfg offset:0 atIndex:1];
        [enc setBuffer:b_sp offset:0 atIndex:2];
        [enc setBuffer:b_sp3 offset:0 atIndex:3];
        [enc setBuffer:b_sp4 offset:0 atIndex:4];
        [enc setBuffer:b_qpos offset:0 atIndex:5];
        [enc setBuffer:b_qvel offset:0 atIndex:6];
        [enc setBuffer:b_xp offset:0 atIndex:7];
        [enc setBuffer:b_xmat offset:0 atIndex:8];
        [enc setBuffer:b_xa offset:0 atIndex:9];
        [enc setBuffer:b_sd offset:0 atIndex:10];
        [enc setBuffer:b_nefc offset:0 atIndex:11];
        [enc setBuffer:b_et offset:0 atIndex:12];
        [enc setBuffer:b_eid offset:0 atIndex:13];
        [enc setBuffer:b_eJ offset:0 atIndex:14];
        [enc setBuffer:b_ef offset:0 atIndex:15];
        [enc setBuffer:b_nacon offset:0 atIndex:16];
        [enc setBuffer:b_cwid offset:0 atIndex:17];
        [enc setBuffer:b_cgeom offset:0 atIndex:18];
        [enc setBuffer:b_cadr offset:0 atIndex:19];
        [enc setBuffer:b_cdim offset:0 atIndex:20];
        const NSUInteger total_threads = static_cast<NSUInteger>(nworld);
        const NSUInteger tg = std::min<NSUInteger>(64, std::max<NSUInteger>(1, total_threads));
        [enc dispatchThreads:MTLSizeMake(total_threads, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_launch_sp4_post_solve")) {
            return -4;
        }
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp3WorldState *>([b_sp3 contents]), nworld_u),
            boundary_span(d_sp3_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp4WorldState *>([b_sp4 contents]), nworld_u),
            boundary_span(d_sp4_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const float *>([b_qpos contents]), qpos_elems),
            boundary_span(d_qpos, qpos_elems).begin());
    }
    return 0;
}

int bike_metal_launch_sp4_m0_init(
    int nworld,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    bike::physics_core::SpindleM0Scratch<float> *d_m0_scratch,
    bike::gpu::GpuWorldState *d_world_states,
    const float *d_qpos,
    float init_grade,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride
) {
    if (nworld <= 0 || ntendon < 0 || tendon_range_world_stride < 0 ||
        d_idx == nullptr || d_sp3_configs == nullptr ||
        d_sp3_states == nullptr || d_sp4_states == nullptr || d_m0_scratch == nullptr ||
        !std::isfinite(init_grade)) {
        return -1;
    }
    const bike::gpu::GpuModelIndices idx_val = *d_idx;
    if (!bike::gpu::gpu_validate_model_indices(idx_val, ntendon)) {
        return -1;
    }
    const std::size_t nworld_u = to_index(nworld);
    const std::size_t n_cfg = per_world_config ? nworld_u : 1ULL;
    const int has_ws = (d_world_states != nullptr) ? 1 : 0;
    const int has_qp = (d_qpos != nullptr) ? 1 : 0;
    const int has_tr =
        (d_tendon_range != nullptr && ntendon > 0 &&
         (tendon_range_world_stride >= 2 * ntendon ||
          (tendon_range_world_stride == 0 && nworld == 1)))
            ? 1
            : 0;
    if (d_tendon_range != nullptr && ntendon > 0 && has_tr == 0) {
        return -1;
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_sp4_m0_init) {
        return -2;
    }
    @autoreleasepool {
        const auto opt = MTLResourceStorageModeShared;
        id<MTLBuffer> b_cfg = [ctx.device newBufferWithBytes:d_sp3_configs
                                                      length:n_cfg * sizeof(bike::gpu::GpuSp3Config)
                                                     options:opt];
        id<MTLBuffer> b_sp3 = [ctx.device newBufferWithBytes:d_sp3_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp3WorldState)
                                                     options:opt];
        id<MTLBuffer> b_sp4 = [ctx.device newBufferWithBytes:d_sp4_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp4WorldState)
                                                     options:opt];
        id<MTLBuffer> b_m0 = [ctx.device newBufferWithBytes:d_m0_scratch
                                                     length:nworld_u * sizeof(bike::physics_core::SpindleM0Scratch<float>)
                                                    options:opt];
        id<MTLBuffer> b_ws = has_ws
            ? [ctx.device newBufferWithBytes:d_world_states
                                       length:nworld_u * sizeof(bike::gpu::GpuWorldState)
                                      options:opt]
            : [ctx.device newBufferWithLength:sizeof(bike::gpu::GpuWorldState) options:opt];
        const std::size_t qp_elems = nworld_u * to_index(idx_val.nq);
        id<MTLBuffer> b_qp = has_qp
            ? [ctx.device newBufferWithBytes:d_qpos length:qp_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        const std::size_t tr_elems = has_tr
            ? ((tendon_range_world_stride > 0) ? nworld_u * to_index(tendon_range_world_stride)
                                               : to_index(2 * ntendon))
            : 4U;
        id<MTLBuffer> b_tr = has_tr
            ? [ctx.device newBufferWithBytes:d_tendon_range length:tr_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        if (!b_cfg || !b_sp3 || !b_sp4 || !b_m0 || !b_ws || !b_qp || !b_tr) {
            return -3;
        }
        BikeMetalSp4InitScalars sc{};
        sc.idx = idx_val;
        sc.nworld = nworld;
        sc.per_world_config = per_world_config;
        sc.init_grade = init_grade;
        sc.ntendon = ntendon;
        sc.tendon_range_world_stride = tendon_range_world_stride;
        sc.has_world_states = has_ws;
        sc.has_qpos = has_qp;
        sc.has_tendon_range = has_tr;

        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        [enc setComputePipelineState:ctx.pso_sp4_m0_init];
        [enc setBytes:&sc length:sizeof(sc) atIndex:0];
        [enc setBuffer:b_cfg offset:0 atIndex:1];
        [enc setBuffer:b_sp3 offset:0 atIndex:2];
        [enc setBuffer:b_sp4 offset:0 atIndex:3];
        [enc setBuffer:b_m0 offset:0 atIndex:4];
        [enc setBuffer:b_ws offset:0 atIndex:5];
        [enc setBuffer:b_qp offset:0 atIndex:6];
        [enc setBuffer:b_tr offset:0 atIndex:7];
        const NSUInteger total_threads = static_cast<NSUInteger>(nworld);
        const NSUInteger tg = std::min<NSUInteger>(64, std::max<NSUInteger>(1, total_threads));
        [enc dispatchThreads:MTLSizeMake(total_threads, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_launch_sp4_m0_init")) {
            return -4;
        }
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp3WorldState *>([b_sp3 contents]), nworld_u),
            boundary_span(d_sp3_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp4WorldState *>([b_sp4 contents]), nworld_u),
            boundary_span(d_sp4_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const bike::physics_core::SpindleM0Scratch<float> *>([b_m0 contents]), nworld_u),
            boundary_span(d_m0_scratch, nworld_u).begin());
        if (has_ws) {
            std::ranges::copy(
                boundary_span(static_cast<const bike::gpu::GpuWorldState *>([b_ws contents]), nworld_u),
                boundary_span(d_world_states, nworld_u).begin());
        }
        if (has_tr) {
            std::ranges::copy(
                boundary_span(static_cast<const float *>([b_tr contents]), tr_elems),
                boundary_span(d_tendon_range, tr_elems).begin());
        }
    }
    return 0;
}

int bike_metal_launch_sp4_m0_pre_solve_and_begin(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    bike::physics_core::SpindleM0Scratch<float> *d_m0_scratch,
    bike::gpu::GpuWorldState *d_world_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_qfrc_bias,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    const float *d_site_xpos,
    const float *d_subtree_com,
    const float *d_body_subtreemass,
    int qM_row_stride,
    int M_world_stride,
    const int *d_M_rowadr,
    const int *d_M_rownnz,
    const int *d_M_colind,
    const float *d_M_or_qM,
    float *d_qfrc_applied,
    float *d_ctrl,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride,
    float *d_dof_frictionloss,
    int dof_frictionloss_world_stride,
    float *d_qacc_warmstart,
    int *out_any_needs_solve
) {
    if (nworld <= 0 || dt_s <= 0.0f || !std::isfinite(dt_s) ||
        d_idx == nullptr || d_tyre_model == nullptr || d_terrain_desc == nullptr ||
        d_sp3_configs == nullptr || d_spindle_params == nullptr ||
        d_sp3_states == nullptr || d_sp4_states == nullptr ||
        d_m0_scratch == nullptr || d_world_states == nullptr ||
        d_qpos == nullptr || d_qvel == nullptr || d_qfrc_bias == nullptr ||
        d_xpos == nullptr || d_xmat == nullptr || d_xanchor == nullptr ||
        d_xaxis == nullptr || d_site_xpos == nullptr || d_subtree_com == nullptr ||
        d_M_or_qM == nullptr || d_qfrc_applied == nullptr || d_ctrl == nullptr ||
        out_any_needs_solve == nullptr ||
        M_world_stride <= 0 || ntendon < 0 ||
        tendon_range_world_stride < 0 || dof_frictionloss_world_stride < 0) {
        return -1;
    }
    *out_any_needs_solve = 0;
    const bike::gpu::GpuModelIndices idx_val = *d_idx;
    if (!bike::gpu::gpu_validate_model_indices(idx_val, ntendon) ||
        !bike::physics_core::validate_spindle_m2k2_params(d_spindle_params)) {
        return -1;
    }
    if (bike::gpu::validate_host_tyre_and_terrain(
            d_tyre_model, idx_val.nq, idx_val.nv, idx_val.nbody,
            d_terrain_desc, d_terrain_f32, d_terrain_i32) != 0) {
        return -1;
    }
    const int has_tr =
        (d_tendon_range != nullptr && ntendon > 0 &&
         (tendon_range_world_stride >= 2 * ntendon ||
          (tendon_range_world_stride == 0 && nworld == 1)))
            ? 1
            : 0;
    if (d_tendon_range != nullptr && ntendon > 0 && has_tr == 0) {
        return -1;
    }
    const int has_fl =
        (d_dof_frictionloss != nullptr &&
         (dof_frictionloss_world_stride >= idx_val.nv ||
          (dof_frictionloss_world_stride == 0 && nworld == 1)))
            ? 1
            : 0;
    if (d_dof_frictionloss != nullptr && has_fl == 0) {
        return -1;
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_sp4_m0_pre_solve) {
        return -2;
    }

    const std::size_t nworld_u = to_index(nworld);
    const std::size_t n_cfg = per_world_config ? nworld_u : 1ULL;
    const std::size_t f32_cnt = static_cast<std::size_t>(std::max(1, d_terrain_desc->f32_count));
    const std::size_t i32_cnt = static_cast<std::size_t>(std::max(1, d_terrain_desc->i32_count));
    const std::size_t qpos_elems = nworld_u * to_index(idx_val.nq);
    const std::size_t nv_elems = nworld_u * to_index(idx_val.nv);
    const std::size_t nu_elems = nworld_u * to_index(idx_val.nu);
    const std::size_t xpos_elems = nworld_u * to_index(idx_val.nbody) * 3ULL;
    const std::size_t xmat_elems = nworld_u * to_index(idx_val.nbody) * 9ULL;
    const std::size_t j3_elems = nworld_u * to_index(d_tyre_model->njnt) * 3ULL;
    const std::size_t site_elems = nworld_u * to_index(d_spindle_params->nsite) * 3ULL;
    const std::size_t m_elems = nworld_u * to_index(M_world_stride);
    const std::size_t tr_elems = has_tr
        ? ((tendon_range_world_stride > 0) ? nworld_u * to_index(tendon_range_world_stride)
                                           : to_index(2 * ntendon))
        : 4U;
    const std::size_t fl_elems = has_fl
        ? ((dof_frictionloss_world_stride > 0) ? nworld_u * to_index(dof_frictionloss_world_stride)
                                               : to_index(idx_val.nv))
        : 4U;
    const int has_submass = (d_body_subtreemass != nullptr) ? 1 : 0;
    const int is_sparse = (qM_row_stride == 0 && d_M_rowadr != nullptr &&
                           d_M_rownnz != nullptr && d_M_colind != nullptr)
                              ? 1
                              : 0;
    if (qM_row_stride == 0 && is_sparse == 0) {
        return -1;
    }
    std::size_t colind_cnt = 1U;
    if (is_sparse != 0) {
        const auto ra = boundary_span(d_M_rowadr, to_index(idx_val.nv));
        const auto rn = boundary_span(d_M_rownnz, to_index(idx_val.nv));
        for (std::size_t r = 0; r < ra.size(); ++r) {
            if (ra[r] < 0 || rn[r] < 0 || rn[r] > idx_val.nv) {
                return -1;
            }
            const std::size_t end_idx = to_index(ra[r]) + to_index(rn[r]);
            if (end_idx > colind_cnt) {
                colind_cnt = end_idx;
            }
        }
    }

    @autoreleasepool {
        const auto opt = MTLResourceStorageModeShared;
        id<MTLBuffer> b_tyre = [ctx.device newBufferWithBytes:d_tyre_model
                                                       length:sizeof(bike::physics_core::GpuTyreModel)
                                                      options:opt];
        id<MTLBuffer> b_tf32 = (d_terrain_desc->f32_count > 0 && d_terrain_f32 != nullptr)
            ? [ctx.device newBufferWithBytes:d_terrain_f32 length:f32_cnt * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_ti32 = (d_terrain_desc->i32_count > 0 && d_terrain_i32 != nullptr)
            ? [ctx.device newBufferWithBytes:d_terrain_i32
                                       length:i32_cnt * sizeof(bike::physics_core::i32)
                                      options:opt]
            : [ctx.device newBufferWithLength:sizeof(bike::physics_core::i32) * 4U options:opt];
        id<MTLBuffer> b_cfg = [ctx.device newBufferWithBytes:d_sp3_configs
                                                      length:n_cfg * sizeof(bike::gpu::GpuSp3Config)
                                                     options:opt];
        id<MTLBuffer> b_sparams = [ctx.device newBufferWithBytes:d_spindle_params
                                                          length:sizeof(bike::physics_core::SpindleM2K2Params<float>)
                                                         options:opt];
        id<MTLBuffer> b_sp3 = [ctx.device newBufferWithBytes:d_sp3_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp3WorldState)
                                                     options:opt];
        id<MTLBuffer> b_sp4 = [ctx.device newBufferWithBytes:d_sp4_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp4WorldState)
                                                     options:opt];
        id<MTLBuffer> b_m0 = [ctx.device newBufferWithBytes:d_m0_scratch
                                                     length:nworld_u * sizeof(bike::physics_core::SpindleM0Scratch<float>)
                                                    options:opt];
        id<MTLBuffer> b_ws = [ctx.device newBufferWithBytes:d_world_states
                                                     length:nworld_u * sizeof(bike::gpu::GpuWorldState)
                                                    options:opt];
        const bool is_pneum =
            (d_tyre_model->backend == bike::physics_core::TyreBackendKind::pneumatic);
        id<MTLBuffer> b_pneum = ctx.ensure_pneum_scratch(nworld, is_pneum);
        id<MTLBuffer> b_qp = [ctx.device newBufferWithBytes:d_qpos length:qpos_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_qv = [ctx.device newBufferWithBytes:d_qvel length:nv_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_bias = [ctx.device newBufferWithBytes:d_qfrc_bias length:nv_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_xp = [ctx.device newBufferWithBytes:d_xpos length:xpos_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_xm = [ctx.device newBufferWithBytes:d_xmat length:xmat_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_xa = [ctx.device newBufferWithBytes:d_xanchor length:j3_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_xx = [ctx.device newBufferWithBytes:d_xaxis length:j3_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_sp = [ctx.device newBufferWithBytes:d_site_xpos length:site_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_sc = [ctx.device newBufferWithBytes:d_subtree_com length:xpos_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_sm = has_submass
            ? [ctx.device newBufferWithBytes:d_body_subtreemass
                                       length:to_index(idx_val.nbody) * sizeof(float)
                                      options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_ra = is_sparse
            ? [ctx.device newBufferWithBytes:d_M_rowadr length:to_index(idx_val.nv) * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_rn = is_sparse
            ? [ctx.device newBufferWithBytes:d_M_rownnz length:to_index(idx_val.nv) * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_ci = is_sparse
            ? [ctx.device newBufferWithBytes:d_M_colind length:colind_cnt * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_M = [ctx.device newBufferWithBytes:d_M_or_qM length:m_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_qfrc = [ctx.device newBufferWithBytes:d_qfrc_applied length:nv_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_ctrl = (nu_elems > 0U)
            ? [ctx.device newBufferWithBytes:d_ctrl length:nu_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_tr = has_tr
            ? [ctx.device newBufferWithBytes:d_tendon_range length:tr_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_fl = has_fl
            ? [ctx.device newBufferWithBytes:d_dof_frictionloss length:fl_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        const int has_ws = (d_qacc_warmstart != nullptr) ? 1 : 0;
        id<MTLBuffer> b_ws_qacc = has_ws
            ? [ctx.device newBufferWithBytes:d_qacc_warmstart length:nv_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        const int zero_any = 0;
        id<MTLBuffer> b_any = [ctx.device newBufferWithBytes:&zero_any length:sizeof(int) options:opt];
        if (!b_tyre || !b_tf32 || !b_ti32 || !b_cfg || !b_sparams || !b_sp3 || !b_sp4 || !b_m0 ||
            !b_ws || !b_pneum || !b_qp || !b_qv || !b_bias || !b_xp || !b_xm || !b_xa || !b_xx ||
            !b_sp || !b_sc || !b_sm || !b_ra || !b_rn || !b_ci || !b_M || !b_qfrc || !b_ctrl ||
            !b_tr || !b_fl || !b_ws_qacc || !b_any) {
            return -3;
        }
        BikeMetalSp4PreSolveScalars sc{};
        sc.idx = idx_val;
        sc.terrain_desc = *d_terrain_desc;
        sc.nworld = nworld;
        sc.dt_s = dt_s;
        sc.per_world_config = per_world_config;
        sc.has_body_subtreemass = has_submass;
        sc.qM_row_stride = qM_row_stride;
        sc.M_world_stride = M_world_stride;
        sc.ntendon = ntendon;
        sc.has_tendon_range = has_tr;
        sc.tendon_range_world_stride = tendon_range_world_stride;
        sc.has_dof_frictionloss = has_fl;
        sc.dof_frictionloss_world_stride = dof_frictionloss_world_stride;
        sc.has_qacc_warmstart = has_ws;
        sc.has_pneum_scratch = is_pneum ? 1 : 0;

        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        [enc setComputePipelineState:ctx.pso_sp4_m0_pre_solve];
        [enc setBytes:&sc length:sizeof(sc) atIndex:0];
        [enc setBuffer:b_tyre offset:0 atIndex:1];
        [enc setBuffer:b_tf32 offset:0 atIndex:2];
        [enc setBuffer:b_ti32 offset:0 atIndex:3];
        [enc setBuffer:b_cfg offset:0 atIndex:4];
        [enc setBuffer:b_sparams offset:0 atIndex:5];
        [enc setBuffer:b_sp3 offset:0 atIndex:6];
        [enc setBuffer:b_sp4 offset:0 atIndex:7];
        [enc setBuffer:b_m0 offset:0 atIndex:8];
        [enc setBuffer:b_ws offset:0 atIndex:9];
        [enc setBuffer:b_pneum offset:0 atIndex:10];
        [enc setBuffer:b_qp offset:0 atIndex:11];
        [enc setBuffer:b_qv offset:0 atIndex:12];
        [enc setBuffer:b_bias offset:0 atIndex:13];
        [enc setBuffer:b_xp offset:0 atIndex:14];
        [enc setBuffer:b_xm offset:0 atIndex:15];
        [enc setBuffer:b_xa offset:0 atIndex:16];
        [enc setBuffer:b_xx offset:0 atIndex:17];
        [enc setBuffer:b_sp offset:0 atIndex:18];
        [enc setBuffer:b_sc offset:0 atIndex:19];
        [enc setBuffer:b_sm offset:0 atIndex:20];
        [enc setBuffer:b_ra offset:0 atIndex:21];
        [enc setBuffer:b_rn offset:0 atIndex:22];
        [enc setBuffer:b_ci offset:0 atIndex:23];
        [enc setBuffer:b_M offset:0 atIndex:24];
        [enc setBuffer:b_qfrc offset:0 atIndex:25];
        [enc setBuffer:b_ctrl offset:0 atIndex:26];
        [enc setBuffer:b_tr offset:0 atIndex:27];
        [enc setBuffer:b_fl offset:0 atIndex:28];
        [enc setBuffer:b_ws_qacc offset:0 atIndex:29];
        [enc setBuffer:b_any offset:0 atIndex:30];
        const NSUInteger total_threads = static_cast<NSUInteger>(nworld);
        const NSUInteger tg = std::min<NSUInteger>(64, std::max<NSUInteger>(1, total_threads));
        [enc dispatchThreads:MTLSizeMake(total_threads, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_launch_sp4_m0_pre_solve_and_begin")) {
            return -4;
        }
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp3WorldState *>([b_sp3 contents]), nworld_u),
            boundary_span(d_sp3_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp4WorldState *>([b_sp4 contents]), nworld_u),
            boundary_span(d_sp4_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const bike::physics_core::SpindleM0Scratch<float> *>([b_m0 contents]), nworld_u),
            boundary_span(d_m0_scratch, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuWorldState *>([b_ws contents]), nworld_u),
            boundary_span(d_world_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const float *>([b_qfrc contents]), nv_elems),
            boundary_span(d_qfrc_applied, nv_elems).begin());
        if (nu_elems > 0U) {
            std::ranges::copy(
                boundary_span(static_cast<const float *>([b_ctrl contents]), nu_elems),
                boundary_span(d_ctrl, nu_elems).begin());
        }
        if (has_tr) {
            std::ranges::copy(
                boundary_span(static_cast<const float *>([b_tr contents]), tr_elems),
                boundary_span(d_tendon_range, tr_elems).begin());
        }
        if (has_fl) {
            std::ranges::copy(
                boundary_span(static_cast<const float *>([b_fl contents]), fl_elems),
                boundary_span(d_dof_frictionloss, fl_elems).begin());
        }
        if (has_ws) {
            std::ranges::copy(
                boundary_span(static_cast<const float *>([b_ws_qacc contents]), nv_elems),
                boundary_span(d_qacc_warmstart, nv_elems).begin());
        }
        const int any_val = *static_cast<const int *>([b_any contents]);
        *out_any_needs_solve = (any_val != 0) ? 1 : 0;
    }
    return 0;
}

int bike_metal_launch_sp4_m0_step(
    int nworld,
    int nq,
    int nv,
    int nu,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    const bike::gpu::GpuSp3WorldState *d_sp3_states,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    bike::physics_core::SpindleM0Scratch<float> *d_m0_scratch,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_qfrc_bias,
    const float *d_qacc,
    int qM_row_stride,
    int M_world_stride,
    const int *d_M_rowadr,
    const int *d_M_rownnz,
    const int *d_M_colind,
    const float *d_M_or_qM,
    float *d_ctrl,
    float *d_qacc_warmstart,
    int *out_any_needs_solve
) {
    if (nworld <= 0 || nq <= 0 || nv <= 0 || nu < 0 ||
        nq > bike::gpu::kMaxNq || nv > bike::gpu::kMaxNv || nu > bike::gpu::kMaxNu ||
        d_idx == nullptr || d_sp3_configs == nullptr ||
        d_sp3_states == nullptr || d_spindle_params == nullptr ||
        d_sp4_states == nullptr || d_m0_scratch == nullptr ||
        d_qpos == nullptr || d_qvel == nullptr || d_qfrc_bias == nullptr ||
        d_qacc == nullptr || d_M_or_qM == nullptr || d_ctrl == nullptr ||
        out_any_needs_solve == nullptr || M_world_stride <= 0) {
        return -1;
    }
    *out_any_needs_solve = 0;
    const bike::gpu::GpuModelIndices idx_val = *d_idx;
    if (!bike::gpu::gpu_validate_model_indices(idx_val) ||
        !bike::physics_core::validate_spindle_m2k2_params(d_spindle_params)) {
        return -1;
    }
    const int is_sparse = (qM_row_stride == 0 && d_M_rowadr != nullptr &&
                           d_M_rownnz != nullptr && d_M_colind != nullptr)
                              ? 1
                              : 0;
    if (qM_row_stride == 0 && is_sparse == 0) {
        return -1;
    }
    std::size_t colind_cnt = 1U;
    if (is_sparse != 0) {
        const auto ra = boundary_span(d_M_rowadr, to_index(nv));
        const auto rn = boundary_span(d_M_rownnz, to_index(nv));
        for (std::size_t r = 0; r < ra.size(); ++r) {
            if (ra[r] < 0 || rn[r] < 0 || rn[r] > nv) {
                return -1;
            }
            const std::size_t end_idx = to_index(ra[r]) + to_index(rn[r]);
            if (end_idx > colind_cnt) {
                colind_cnt = end_idx;
            }
        }
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_sp4_m0_step) {
        return -2;
    }
    const std::size_t nworld_u = to_index(nworld);
    const std::size_t n_cfg = per_world_config ? nworld_u : 1ULL;
    const std::size_t qpos_elems = nworld_u * to_index(nq);
    const std::size_t nv_elems = nworld_u * to_index(nv);
    const std::size_t nu_elems = nworld_u * to_index(nu);
    const std::size_t m_elems = nworld_u * to_index(M_world_stride);
    @autoreleasepool {
        const auto opt = MTLResourceStorageModeShared;
        id<MTLBuffer> b_cfg = [ctx.device newBufferWithBytes:d_sp3_configs
                                                      length:n_cfg * sizeof(bike::gpu::GpuSp3Config)
                                                     options:opt];
        id<MTLBuffer> b_sp3 = [ctx.device newBufferWithBytes:d_sp3_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp3WorldState)
                                                     options:opt];
        id<MTLBuffer> b_sparams = [ctx.device newBufferWithBytes:d_spindle_params
                                                          length:sizeof(bike::physics_core::SpindleM2K2Params<float>)
                                                         options:opt];
        id<MTLBuffer> b_sp4 = [ctx.device newBufferWithBytes:d_sp4_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp4WorldState)
                                                     options:opt];
        id<MTLBuffer> b_m0 = [ctx.device newBufferWithBytes:d_m0_scratch
                                                     length:nworld_u * sizeof(bike::physics_core::SpindleM0Scratch<float>)
                                                    options:opt];
        id<MTLBuffer> b_qp = [ctx.device newBufferWithBytes:d_qpos length:qpos_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_qv = [ctx.device newBufferWithBytes:d_qvel length:nv_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_bias = [ctx.device newBufferWithBytes:d_qfrc_bias length:nv_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_qacc = [ctx.device newBufferWithBytes:d_qacc length:nv_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_ra = is_sparse
            ? [ctx.device newBufferWithBytes:d_M_rowadr length:to_index(nv) * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_rn = is_sparse
            ? [ctx.device newBufferWithBytes:d_M_rownnz length:to_index(nv) * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_ci = is_sparse
            ? [ctx.device newBufferWithBytes:d_M_colind length:colind_cnt * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_M = [ctx.device newBufferWithBytes:d_M_or_qM length:m_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_ctrl = (nu_elems > 0U)
            ? [ctx.device newBufferWithBytes:d_ctrl length:nu_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        const int has_ws = (d_qacc_warmstart != nullptr) ? 1 : 0;
        id<MTLBuffer> b_ws = has_ws
            ? [ctx.device newBufferWithBytes:d_qacc_warmstart length:nv_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        const int zero_any = 0;
        id<MTLBuffer> b_any = [ctx.device newBufferWithBytes:&zero_any length:sizeof(int) options:opt];
        if (!b_cfg || !b_sp3 || !b_sparams || !b_sp4 || !b_m0 || !b_qp || !b_qv || !b_bias ||
            !b_qacc || !b_ra || !b_rn || !b_ci || !b_M || !b_ctrl || !b_ws || !b_any) {
            return -3;
        }
        BikeMetalSp4M0StepScalars sc{};
        sc.idx = idx_val;
        sc.nworld = nworld;
        sc.nq = nq;
        sc.nv = nv;
        sc.nu = nu;
        sc.per_world_config = per_world_config;
        sc.qM_row_stride = qM_row_stride;
        sc.M_world_stride = M_world_stride;
        sc.has_qacc_warmstart = has_ws;

        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        [enc setComputePipelineState:ctx.pso_sp4_m0_step];
        [enc setBytes:&sc length:sizeof(sc) atIndex:0];
        [enc setBuffer:b_cfg offset:0 atIndex:1];
        [enc setBuffer:b_sp3 offset:0 atIndex:2];
        [enc setBuffer:b_sparams offset:0 atIndex:3];
        [enc setBuffer:b_sp4 offset:0 atIndex:4];
        [enc setBuffer:b_m0 offset:0 atIndex:5];
        [enc setBuffer:b_qp offset:0 atIndex:6];
        [enc setBuffer:b_qv offset:0 atIndex:7];
        [enc setBuffer:b_bias offset:0 atIndex:8];
        [enc setBuffer:b_qacc offset:0 atIndex:9];
        [enc setBuffer:b_ra offset:0 atIndex:10];
        [enc setBuffer:b_rn offset:0 atIndex:11];
        [enc setBuffer:b_ci offset:0 atIndex:12];
        [enc setBuffer:b_M offset:0 atIndex:13];
        [enc setBuffer:b_ctrl offset:0 atIndex:14];
        [enc setBuffer:b_ws offset:0 atIndex:15];
        [enc setBuffer:b_any offset:0 atIndex:16];
        const NSUInteger total_threads = static_cast<NSUInteger>(nworld);
        const NSUInteger tg = std::min<NSUInteger>(64, std::max<NSUInteger>(1, total_threads));
        [enc dispatchThreads:MTLSizeMake(total_threads, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_launch_sp4_m0_step")) {
            return -4;
        }
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp4WorldState *>([b_sp4 contents]), nworld_u),
            boundary_span(d_sp4_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const bike::physics_core::SpindleM0Scratch<float> *>([b_m0 contents]), nworld_u),
            boundary_span(d_m0_scratch, nworld_u).begin());
        if (nu_elems > 0U) {
            std::ranges::copy(
                boundary_span(static_cast<const float *>([b_ctrl contents]), nu_elems),
                boundary_span(d_ctrl, nu_elems).begin());
        }
        if (has_ws) {
            std::ranges::copy(
                boundary_span(static_cast<const float *>([b_ws contents]), nv_elems),
                boundary_span(d_qacc_warmstart, nv_elems).begin());
        }
        const int any_val = *static_cast<const int *>([b_any contents]);
        *out_any_needs_solve = (any_val != 0) ? 1 : 0;
    }
    return 0;
}

int bike_metal_launch_sp4_m0_post_solve(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    const bike::physics_core::SpindleM0Scratch<float> *d_m0_scratch,
    float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    int nsensordata,
    const float *d_sensordata,
    const int *d_nefc,
    int njmax,
    int efc_J_row_stride,
    const int *d_efc_type,
    const int *d_efc_id,
    const float *d_efc_J,
    const float *d_efc_force,
    const int *d_nacon,
    int nconmax,
    int contact_efc_stride,
    const int *d_contact_worldid,
    const int *d_contact_geom,
    const int *d_contact_efc_address,
    const int *d_contact_dim
) {
    if (nworld <= 0 || dt_s <= 0.0f || !std::isfinite(dt_s) ||
        d_idx == nullptr || d_sp3_configs == nullptr || d_spindle_params == nullptr ||
        d_sp3_states == nullptr || d_sp4_states == nullptr || d_m0_scratch == nullptr ||
        d_qpos == nullptr || d_qvel == nullptr || d_xmat == nullptr ||
        nsensordata < 0 || njmax < 0 || efc_J_row_stride < 0 || nconmax < 0 ||
        contact_efc_stride < 0 || (nconmax > 0 && contact_efc_stride == 0)) {
        return -1;
    }
    const bike::gpu::GpuModelIndices idx_val = *d_idx;
    if (!bike::gpu::gpu_validate_model_indices(idx_val)) {
        return -1;
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_sp4_m0_post_solve) {
        return -2;
    }
    const std::size_t nworld_u = to_index(nworld);
    const std::size_t n_cfg = per_world_config ? nworld_u : 1ULL;
    const std::size_t qpos_elems = nworld_u * to_index(idx_val.nq);
    const std::size_t qvel_elems = nworld_u * to_index(idx_val.nv);
    const std::size_t xpos_elems = nworld_u * to_index(idx_val.nbody) * 3ULL;
    const std::size_t xmat_elems = nworld_u * to_index(idx_val.nbody) * 9ULL;
    const int nj = d_spindle_params->njnt;
    const int has_xp = (d_xpos != nullptr) ? 1 : 0;
    const int has_xa = (d_xanchor != nullptr && nj > 0) ? 1 : 0;
    const std::size_t xa_elems = has_xa ? (nworld_u * to_index(nj) * 3ULL) : 4U;
    const int has_sd = (d_sensordata != nullptr && nsensordata > 0) ? 1 : 0;
    const int has_efc = (d_nefc != nullptr && njmax > 0) ? 1 : 0;
    const int has_con =
        (d_nacon != nullptr && nconmax > 0 && d_contact_geom != nullptr &&
         d_contact_efc_address != nullptr)
            ? 1
            : 0;
    const int has_cwid = (has_con != 0 && d_contact_worldid != nullptr) ? 1 : 0;
    const int has_cdim = (has_con != 0 && d_contact_dim != nullptr) ? 1 : 0;

    @autoreleasepool {
        const auto opt = MTLResourceStorageModeShared;
        id<MTLBuffer> b_cfg = [ctx.device newBufferWithBytes:d_sp3_configs
                                                      length:n_cfg * sizeof(bike::gpu::GpuSp3Config)
                                                     options:opt];
        id<MTLBuffer> b_sp = [ctx.device newBufferWithBytes:d_spindle_params
                                                     length:sizeof(bike::physics_core::SpindleM2K2Params<float>)
                                                    options:opt];
        id<MTLBuffer> b_sp3 = [ctx.device newBufferWithBytes:d_sp3_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp3WorldState)
                                                     options:opt];
        id<MTLBuffer> b_sp4 = [ctx.device newBufferWithBytes:d_sp4_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp4WorldState)
                                                     options:opt];
        id<MTLBuffer> b_m0 = [ctx.device newBufferWithBytes:d_m0_scratch
                                                     length:nworld_u * sizeof(bike::physics_core::SpindleM0Scratch<float>)
                                                    options:opt];
        id<MTLBuffer> b_qpos = [ctx.device newBufferWithBytes:d_qpos length:qpos_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_qvel = [ctx.device newBufferWithBytes:d_qvel length:qvel_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_xp = has_xp
            ? [ctx.device newBufferWithBytes:d_xpos length:xpos_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_xmat = [ctx.device newBufferWithBytes:d_xmat length:xmat_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_xa = has_xa
            ? [ctx.device newBufferWithBytes:d_xanchor length:xa_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_sd = has_sd
            ? [ctx.device newBufferWithBytes:d_sensordata
                                       length:nworld_u * to_index(nsensordata) * sizeof(float)
                                      options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        const std::size_t efc_elems = has_efc ? (nworld_u * to_index(njmax)) : 4U;
        const std::size_t eJ_elems =
            (has_efc && d_efc_J != nullptr && efc_J_row_stride > 0)
                ? (efc_elems * to_index(efc_J_row_stride))
                : 4U;
        id<MTLBuffer> b_nefc = has_efc
            ? [ctx.device newBufferWithBytes:d_nefc length:nworld_u * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_et = (has_efc && d_efc_type != nullptr)
            ? [ctx.device newBufferWithBytes:d_efc_type length:efc_elems * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_eid = (has_efc && d_efc_id != nullptr)
            ? [ctx.device newBufferWithBytes:d_efc_id length:efc_elems * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_eJ = (has_efc && d_efc_J != nullptr && efc_J_row_stride > 0)
            ? [ctx.device newBufferWithBytes:d_efc_J length:eJ_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_ef = (has_efc && d_efc_force != nullptr)
            ? [ctx.device newBufferWithBytes:d_efc_force length:efc_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];

        const std::size_t nacon_cnt = has_con ? (has_cwid ? 1U : nworld_u) : 4U;
        const std::size_t con_tot =
            has_con ? (has_cwid ? to_index(nconmax) : (nworld_u * to_index(nconmax))) : 4U;
        id<MTLBuffer> b_nacon = has_con
            ? [ctx.device newBufferWithBytes:d_nacon length:nacon_cnt * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_cwid = has_cwid
            ? [ctx.device newBufferWithBytes:d_contact_worldid length:con_tot * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_cgeom = has_con
            ? [ctx.device newBufferWithBytes:d_contact_geom length:con_tot * 2U * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_cadr = has_con
            ? [ctx.device newBufferWithBytes:d_contact_efc_address
                                       length:con_tot * to_index(contact_efc_stride) * sizeof(int)
                                      options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_cdim = has_cdim
            ? [ctx.device newBufferWithBytes:d_contact_dim length:con_tot * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];

        if (!b_cfg || !b_sp || !b_sp3 || !b_sp4 || !b_m0 || !b_qpos || !b_qvel ||
            !b_xp || !b_xmat || !b_xa || !b_sd || !b_nefc || !b_et || !b_eid ||
            !b_eJ || !b_ef || !b_nacon || !b_cwid || !b_cgeom || !b_cadr || !b_cdim) {
            return -3;
        }
        BikeMetalSp4PostSolveScalars sc{};
        sc.idx = idx_val;
        sc.nworld = nworld;
        sc.dt_s = dt_s;
        sc.per_world_config = per_world_config;
        sc.has_xpos = has_xp;
        sc.has_xanchor = has_xa;
        sc.has_sensordata = has_sd;
        sc.nsensordata = nsensordata;
        sc.has_efc = has_efc;
        sc.njmax = njmax;
        sc.efc_J_row_stride = efc_J_row_stride;
        sc.has_contacts = has_con;
        sc.nconmax = nconmax;
        sc.contact_efc_stride = contact_efc_stride;
        sc.has_contact_worldid = has_cwid;
        sc.has_contact_dim = has_cdim;

        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        [enc setComputePipelineState:ctx.pso_sp4_m0_post_solve];
        [enc setBytes:&sc length:sizeof(sc) atIndex:0];
        [enc setBuffer:b_cfg offset:0 atIndex:1];
        [enc setBuffer:b_sp offset:0 atIndex:2];
        [enc setBuffer:b_sp3 offset:0 atIndex:3];
        [enc setBuffer:b_sp4 offset:0 atIndex:4];
        [enc setBuffer:b_m0 offset:0 atIndex:5];
        [enc setBuffer:b_qpos offset:0 atIndex:6];
        [enc setBuffer:b_qvel offset:0 atIndex:7];
        [enc setBuffer:b_xp offset:0 atIndex:8];
        [enc setBuffer:b_xmat offset:0 atIndex:9];
        [enc setBuffer:b_xa offset:0 atIndex:10];
        [enc setBuffer:b_sd offset:0 atIndex:11];
        [enc setBuffer:b_nefc offset:0 atIndex:12];
        [enc setBuffer:b_et offset:0 atIndex:13];
        [enc setBuffer:b_eid offset:0 atIndex:14];
        [enc setBuffer:b_eJ offset:0 atIndex:15];
        [enc setBuffer:b_ef offset:0 atIndex:16];
        [enc setBuffer:b_nacon offset:0 atIndex:17];
        [enc setBuffer:b_cwid offset:0 atIndex:18];
        [enc setBuffer:b_cgeom offset:0 atIndex:19];
        [enc setBuffer:b_cadr offset:0 atIndex:20];
        [enc setBuffer:b_cdim offset:0 atIndex:21];
        const NSUInteger total_threads = static_cast<NSUInteger>(nworld);
        const NSUInteger tg = std::min<NSUInteger>(64, std::max<NSUInteger>(1, total_threads));
        [enc dispatchThreads:MTLSizeMake(total_threads, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_launch_sp4_m0_post_solve")) {
            return -4;
        }
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp3WorldState *>([b_sp3 contents]), nworld_u),
            boundary_span(d_sp3_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp4WorldState *>([b_sp4 contents]), nworld_u),
            boundary_span(d_sp4_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const float *>([b_qpos contents]), qpos_elems),
            boundary_span(d_qpos, qpos_elems).begin());
    }
    return 0;
}

int bike_metal_launch_sp4_energy_post_step(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    const bike::gpu::GpuWorldState *d_world_states,
    int qM_row_stride,
    int M_world_stride,
    const int *d_M_rowadr,
    const int *d_M_rownnz,
    const int *d_M_colind,
    const float *d_M_or_qM,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_ctrl,
    const float *d_body_mass,
    const float *d_xipos,
    const float *d_gravity,
    const int *d_nefc,
    int njmax,
    int efc_J_row_stride,
    const int *d_efc_type,
    const float *d_efc_J,
    const float *d_efc_force
) {
    if (nworld <= 0 || dt_s <= 0.0f || !std::isfinite(dt_s) ||
        d_idx == nullptr || d_tyre_model == nullptr || d_terrain_desc == nullptr ||
        d_sp3_configs == nullptr || d_spindle_params == nullptr ||
        d_sp3_states == nullptr || d_sp4_states == nullptr || d_world_states == nullptr ||
        d_M_or_qM == nullptr || d_qpos == nullptr || d_qvel == nullptr || d_xpos == nullptr ||
        d_body_mass == nullptr || d_xipos == nullptr || d_gravity == nullptr ||
        M_world_stride <= 0 || njmax < 0 || efc_J_row_stride < 0) {
        return -1;
    }
    const bike::gpu::GpuModelIndices idx_val = *d_idx;
    if (!bike::gpu::gpu_validate_model_indices(idx_val) ||
        !bike::physics_core::validate_spindle_m2k2_params(d_spindle_params)) {
        return -1;
    }
    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    if (bike::physics_core::validate_gpu_tyre_model_device(
            d_tyre_model, idx_val.nq, idx_val.nv, idx_val.nbody, err) !=
        bike::physics_core::Status::ok) {
        return -1;
    }
    if (qM_row_stride > 0 && qM_row_stride < idx_val.nv) {
        return -1;
    }
    if ((d_tyre_model->backend == bike::physics_core::TyreBackendKind::compliant_2d ||
         d_tyre_model->backend == bike::physics_core::TyreBackendKind::pneumatic) &&
        (d_terrain_desc->f32_count <= 0 || d_terrain_f32 == nullptr)) {
        return -1;
    }
    const int is_sparse = (qM_row_stride == 0 && d_M_rowadr != nullptr &&
                           d_M_rownnz != nullptr && d_M_colind != nullptr)
                              ? 1
                              : 0;
    if (qM_row_stride == 0 && is_sparse == 0) {
        return -1;
    }
    std::size_t colind_cnt = 1U;
    if (is_sparse != 0) {
        const auto ra = boundary_span(d_M_rowadr, to_index(idx_val.nv));
        const auto rn = boundary_span(d_M_rownnz, to_index(idx_val.nv));
        for (std::size_t r = 0; r < ra.size(); ++r) {
            if (ra[r] < 0 || rn[r] < 0 || rn[r] > idx_val.nv) {
                return -1;
            }
            const std::size_t end_idx = to_index(ra[r]) + to_index(rn[r]);
            if (end_idx > colind_cnt) {
                colind_cnt = end_idx;
            }
        }
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_sp4_energy_post_step) {
        return -2;
    }
    const std::size_t nworld_u = to_index(nworld);
    const std::size_t n_cfg = per_world_config ? nworld_u : 1ULL;
    const std::size_t f32_cnt = static_cast<std::size_t>(std::max(1, d_terrain_desc->f32_count));
    const std::size_t m_elems = nworld_u * to_index(M_world_stride);
    const std::size_t qpos_elems = nworld_u * to_index(idx_val.nq);
    const std::size_t nv_elems = nworld_u * to_index(idx_val.nv);
    const std::size_t nu_elems = nworld_u * to_index(idx_val.nu);
    const std::size_t xpos_elems = nworld_u * to_index(idx_val.nbody) * 3ULL;
    const std::size_t xipos_elems = nworld_u * to_index(idx_val.nbody) * 3ULL;
    const int has_ctrl = (d_ctrl != nullptr && idx_val.nu > 0) ? 1 : 0;
    const int has_efc = (d_nefc != nullptr && njmax > 0) ? 1 : 0;
    const std::size_t efc_elems = has_efc ? (nworld_u * to_index(njmax)) : 4U;
    const std::size_t eJ_elems =
        (has_efc && d_efc_J != nullptr && efc_J_row_stride > 0)
            ? (efc_elems * to_index(efc_J_row_stride))
            : 4U;

    @autoreleasepool {
        const auto opt = MTLResourceStorageModeShared;
        id<MTLBuffer> b_tyre = [ctx.device newBufferWithBytes:d_tyre_model
                                                       length:sizeof(bike::physics_core::GpuTyreModel)
                                                      options:opt];
        id<MTLBuffer> b_tf32 = (d_terrain_desc->f32_count > 0 && d_terrain_f32 != nullptr)
            ? [ctx.device newBufferWithBytes:d_terrain_f32 length:f32_cnt * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_cfg = [ctx.device newBufferWithBytes:d_sp3_configs
                                                      length:n_cfg * sizeof(bike::gpu::GpuSp3Config)
                                                     options:opt];
        id<MTLBuffer> b_sparams = [ctx.device newBufferWithBytes:d_spindle_params
                                                          length:sizeof(bike::physics_core::SpindleM2K2Params<float>)
                                                         options:opt];
        id<MTLBuffer> b_sp3 = [ctx.device newBufferWithBytes:d_sp3_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp3WorldState)
                                                     options:opt];
        id<MTLBuffer> b_sp4 = [ctx.device newBufferWithBytes:d_sp4_states
                                                      length:nworld_u * sizeof(bike::gpu::GpuSp4WorldState)
                                                     options:opt];
        id<MTLBuffer> b_ws = [ctx.device newBufferWithBytes:d_world_states
                                                     length:nworld_u * sizeof(bike::gpu::GpuWorldState)
                                                    options:opt];
        const bool is_pneum =
            (d_tyre_model->backend == bike::physics_core::TyreBackendKind::pneumatic);
        id<MTLBuffer> b_pneum = ctx.ensure_pneum_scratch(nworld, is_pneum);
        id<MTLBuffer> b_ra = is_sparse
            ? [ctx.device newBufferWithBytes:d_M_rowadr length:to_index(idx_val.nv) * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_rn = is_sparse
            ? [ctx.device newBufferWithBytes:d_M_rownnz length:to_index(idx_val.nv) * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_ci = is_sparse
            ? [ctx.device newBufferWithBytes:d_M_colind length:colind_cnt * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_M = [ctx.device newBufferWithBytes:d_M_or_qM length:m_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_qp = [ctx.device newBufferWithBytes:d_qpos length:qpos_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_qv = [ctx.device newBufferWithBytes:d_qvel length:nv_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_xp = [ctx.device newBufferWithBytes:d_xpos length:xpos_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_ctrl = has_ctrl
            ? [ctx.device newBufferWithBytes:d_ctrl length:nu_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_bm = [ctx.device newBufferWithBytes:d_body_mass
                                                     length:to_index(idx_val.nbody) * sizeof(float)
                                                    options:opt];
        id<MTLBuffer> b_xip = [ctx.device newBufferWithBytes:d_xipos length:xipos_elems * sizeof(float) options:opt];
        id<MTLBuffer> b_grav = [ctx.device newBufferWithBytes:d_gravity length:3U * sizeof(float) options:opt];
        id<MTLBuffer> b_nefc = has_efc
            ? [ctx.device newBufferWithBytes:d_nefc length:nworld_u * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_et = (has_efc && d_efc_type != nullptr)
            ? [ctx.device newBufferWithBytes:d_efc_type length:efc_elems * sizeof(int) options:opt]
            : [ctx.device newBufferWithLength:sizeof(int) * 4U options:opt];
        id<MTLBuffer> b_eJ = (has_efc && d_efc_J != nullptr && efc_J_row_stride > 0)
            ? [ctx.device newBufferWithBytes:d_efc_J length:eJ_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        id<MTLBuffer> b_ef = (has_efc && d_efc_force != nullptr)
            ? [ctx.device newBufferWithBytes:d_efc_force length:efc_elems * sizeof(float) options:opt]
            : [ctx.device newBufferWithLength:sizeof(float) * 4U options:opt];
        if (!b_tyre || !b_tf32 || !b_cfg || !b_sparams || !b_sp3 || !b_sp4 || !b_ws || !b_pneum ||
            !b_ra || !b_rn || !b_ci || !b_M || !b_qp || !b_qv || !b_xp || !b_ctrl || !b_bm || !b_xip ||
            !b_grav || !b_nefc || !b_et || !b_eJ || !b_ef) {
            return -3;
        }
        BikeMetalSp4EnergyScalars sc{};
        sc.idx = idx_val;
        sc.terrain_desc = *d_terrain_desc;
        sc.nworld = nworld;
        sc.dt_s = dt_s;
        sc.per_world_config = per_world_config;
        sc.qM_row_stride = qM_row_stride;
        sc.M_world_stride = M_world_stride;
        sc.njmax = njmax;
        sc.efc_J_row_stride = efc_J_row_stride;
        sc.has_pneum_scratch = is_pneum ? 1 : 0;
        sc.has_efc = has_efc;
        sc.has_ctrl = has_ctrl;

        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        [enc setComputePipelineState:ctx.pso_sp4_energy_post_step];
        [enc setBytes:&sc length:sizeof(sc) atIndex:0];
        [enc setBuffer:b_tyre offset:0 atIndex:1];
        [enc setBuffer:b_tf32 offset:0 atIndex:2];
        [enc setBuffer:b_cfg offset:0 atIndex:3];
        [enc setBuffer:b_sparams offset:0 atIndex:4];
        [enc setBuffer:b_sp3 offset:0 atIndex:5];
        [enc setBuffer:b_sp4 offset:0 atIndex:6];
        [enc setBuffer:b_ws offset:0 atIndex:7];
        [enc setBuffer:b_pneum offset:0 atIndex:8];
        [enc setBuffer:b_ra offset:0 atIndex:9];
        [enc setBuffer:b_rn offset:0 atIndex:10];
        [enc setBuffer:b_ci offset:0 atIndex:11];
        [enc setBuffer:b_M offset:0 atIndex:12];
        [enc setBuffer:b_qp offset:0 atIndex:13];
        [enc setBuffer:b_qv offset:0 atIndex:14];
        [enc setBuffer:b_xp offset:0 atIndex:15];
        [enc setBuffer:b_ctrl offset:0 atIndex:16];
        [enc setBuffer:b_bm offset:0 atIndex:17];
        [enc setBuffer:b_xip offset:0 atIndex:18];
        [enc setBuffer:b_grav offset:0 atIndex:19];
        [enc setBuffer:b_nefc offset:0 atIndex:20];
        [enc setBuffer:b_et offset:0 atIndex:21];
        [enc setBuffer:b_eJ offset:0 atIndex:22];
        [enc setBuffer:b_ef offset:0 atIndex:23];
        const NSUInteger total_threads = static_cast<NSUInteger>(nworld);
        const NSUInteger tg = std::min<NSUInteger>(64, std::max<NSUInteger>(1, total_threads));
        [enc dispatchThreads:MTLSizeMake(total_threads, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_launch_sp4_energy_post_step")) {
            return -4;
        }
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp3WorldState *>([b_sp3 contents]), nworld_u),
            boundary_span(d_sp3_states, nworld_u).begin());
        std::ranges::copy(
            boundary_span(static_cast<const bike::gpu::GpuSp4WorldState *>([b_sp4 contents]), nworld_u),
            boundary_span(d_sp4_states, nworld_u).begin());
    }
    return 0;
}

} // extern "C"
