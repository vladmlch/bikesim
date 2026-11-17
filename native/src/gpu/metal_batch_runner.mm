// native/src/gpu/metal_batch_runner.mm — Apple Metal Unified-Memory GPU Force
// Writers & Hybrid (Metal GPU Writers FP32 + Multi-Threaded C++ CPU MuJoCo FP64)
// Batch Runner for Apple Silicon (M4 Max).

#if !defined(BIKE_METAL_OMIT_CUDA_KERNELS_TU)
#include "cuda_kernels.cu"
#else
#include "batch_step.hpp"
#endif

#include <mujoco/mujoco.h>
#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include "model_assets.hpp"
#include "runner_tyre_setup.hpp"
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
        "../physics_core/tyre_compliant.hpp",
        "../physics_core/tyre_distributed.hpp",
        "../physics_core/tyre_gpu_step.hpp",
        "common.hpp",
        "suspension_kernel.hpp",
        "drivetrain_resistance_kernel.hpp",
        "rider_kernel.hpp",
        "batch_step.hpp",
    };

    std::string combined = R"MSL(
#pragma METAL fp contract(off)
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
    const int fh_qpos_idx =
        (idx.drive_shaft_spin_qpos >= 0) ? idx.drive_shaft_spin_qpos : idx.crank_spin_qpos;
    const float crank_qpos =
        (qpos_w != nullptr && fh_qpos_idx >= 0 && fh_qpos_idx < idx.nq)
            ? qpos_w[fh_qpos_idx]
            : 0.0f;
    const float rear_wheel_qpos =
        (qpos_w != nullptr && idx.rear_wheel_spin_qpos >= 0 && idx.rear_wheel_spin_qpos < idx.nq)
            ? qpos_w[idx.rear_wheel_spin_qpos]
            : 0.0f;
    bike::gpu::gpu_init_world_state(states[w], dp, crank_qpos, rear_wheel_qpos);
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
    int tendon_range_world_stride
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
    float tendon_w[2] = {0.0f, 0.0f};

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

    const bike::physics_core::Status st = bike::gpu::gpu_step_world_writers_single(
        w,
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
        (tendon_range_batch != nullptr && idx.freehub_tendon >= 0) ? tendon_w : nullptr,
        2,
        false
    );
    if (st != bike::physics_core::Status::ok) {
        return;
    }

    states[w].drivetrain = dt_st;
    device float *qfrc_dst = qfrc_batch + w64 * nv;
    device float *ctrl_dst = ctrl_batch + w64 * nu;
    for (int i = 0; i < nv; ++i) qfrc_dst[i] = qfrc_w[i];
    for (int i = 0; i < nu; ++i) ctrl_dst[i] = ctrl_w[i];

    if (tendon_range_batch != nullptr && idx.freehub_tendon >= 0) {
        const long slot =
            static_cast<long>(2) * static_cast<long>(idx.freehub_tendon) + static_cast<long>(1);
        if (tendon_range_world_stride > 0) {
            if (slot < static_cast<long>(tendon_range_world_stride)) {
                tendon_range_batch[w64 * static_cast<long>(tendon_range_world_stride) + slot] =
                    tendon_w[1];
            }
        } else if (w == 0) {
            tendon_range_batch[slot] = tendon_w[1];
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
        qfrc_batch, ctrl_batch, tendon_range_batch, tendon_range_world_stride
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
        args->tendon_range_world_stride
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
    uint gid [[thread_position_in_grid]]
) {
    bike_metal_step_writers_impl(
        world_offset, world_count, dt_s, idx_const, tyre_model, terrain_desc_const,
        terrain_f32, terrain_i32, configs, per_world_config, states,
        qpos_batch, qvel_batch, xpos_batch, xmat_batch, xanchor_batch, xaxis_batch,
        qfrc_batch, ctrl_batch, tendon_range_batch, tendon_range_world_stride, gid
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
    uint gid [[thread_position_in_grid]]
) {
    bike_metal_step_writers_impl(
        world_offset, world_count, dt_s, idx_const, tyre_model, terrain_desc_const,
        terrain_f32, terrain_i32, configs, per_world_config, states,
        qpos_batch, qvel_batch, xpos_batch, xmat_batch, xanchor_batch, xaxis_batch,
        qfrc_batch, ctrl_batch, tendon_range_batch, tendon_range_world_stride, gid
    );
}

kernel void bike_metal_step_writers_fused_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpWritersArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    bike_metal_step_writers_fused_impl(bounds_buf, args, gid);
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
    opts.mathMode = MTLMathModeSafe;
    opts.mathFloatingPointFunctions = MTLMathFloatingPointFunctionsPrecise;
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

struct MetalWritersContext {
    id<MTLDevice> device = nil;
    id<MTLCommandQueue> queue = nil;
    id<MTLLibrary> library = nil;
    id<MTLComputePipelineState> pso_init_states = nil;
    id<MTLComputePipelineState> pso_1thread = nil;
    id<MTLComputePipelineState> pso_step_writers_fused = nil;
    bool ok = false;

    bool init(const std::string &gpu_src_dir) {
        if (ok && pso_1thread != nil && pso_init_states != nil) {
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
        if (!fn_init || !fn1 || !fn_wp1) {
            std::cerr << "[MetalWritersContext] Failed to look up MSL kernel functions\n";
            return false;
        }

        pso_init_states = [device newComputePipelineStateWithFunction:fn_init error:&err];
        pso_1thread = [device newComputePipelineStateWithFunction:fn1 error:&err];
        pso_step_writers_fused = [device newComputePipelineStateWithFunction:fn_wp1 error:&err];
        if (!pso_init_states || !pso_1thread || !pso_step_writers_fused) {
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
    int tendon_stride
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
    if (d_tendon_range != nullptr && idx_val.freehub_tendon >= 0 &&
        tendon_range_world_stride > 0 &&
        static_cast<long>(2) * static_cast<long>(idx_val.freehub_tendon) + 1L >=
            static_cast<long>(tendon_range_world_stride)) {
        std::cerr << "[bike_metal_launch_writers] tendon_range_world_stride too small for freehub_tendon\n";
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
            : 2ULL;
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
        id<MTLBuffer> b_tendon = (d_tendon_range != nullptr)
            ? [ctx.device newBufferWithBytes:d_tendon_range
                                      length:tendon_bytes
                                     options:MTLResourceStorageModeShared]
            : nil;

        if (!b_tyre || !b_tf32 || !b_ti32 || !b_cfg || !b_st ||
            !b_qpos || !b_qvel || !b_xpos || !b_xmat || !b_xanchor || !b_xaxis ||
            !b_qfrc || !b_ctrl || (d_tendon_range != nullptr && !b_tendon)) {
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
            tendon_range_world_stride
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
        if (d_tendon_range != nullptr && b_tendon != nil) {
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
    id<MTLBuffer> b_tendon = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * 2) * sizeof(float)
                                                     options:MTLResourceStorageModeShared];
    if (!b_tyre || !b_tf32 || !b_ti32 || !b_cfg || !b_st ||
        !b_qpos || !b_qvel || !b_xpos || !b_xmat || !b_xanchor || !b_xaxis ||
        !b_qfrc || !b_ctrl || !b_tendon) {
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
        boundary_span(static_cast<float *>([b_tendon contents]), 2 * nworld_u);

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
        bike::gpu::gpu_init_world_state(st_v[wu], cfg_w.drivetrain, 0.0f, 0.0f);
    }

    {
        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        encode_metal_writers_slice(
            enc, ctx, 0, nworld, dt_s, idx_val, b_tyre, *d_terrain_desc, b_tf32, b_ti32,
            b_cfg, per_world_config, b_st, b_qpos, b_qvel, b_xpos, b_xmat, b_xanchor, b_xaxis,
            b_qfrc, b_ctrl, b_tendon, 2
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
                b_qfrc, b_ctrl, b_tendon, 2
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
            b_qfrc, b_ctrl, b_tendon, 2
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

    bike::gpu::ResolvedRunnerTyre resolved_tyre{};
    {
        mjData *d_init = mj_makeData(m_base);
        const std::span<mjtNum> d_init_qpos = boundary_span(d_init->qpos, to_index(nq));
        const std::span<mjtNum> d_init_qvel = boundary_span(d_init->qvel, to_index(nv));
        for (std::size_t i = 0; i < to_index(nq); ++i) {
            d_init_qpos[i] = static_cast<mjtNum>(qpos0[i]);
        }
        for (std::size_t i = 0; i < to_index(nv); ++i) {
            d_init_qvel[i] = static_cast<mjtNum>(qvel0[i]);
        }
        mj_forward(m_base, d_init);
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
    id<MTLBuffer> b_tendon = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * 2) * sizeof(float)
                                                     options:MTLResourceStorageModeShared];
    if (!b_tyre || !b_tf32 || !b_ti32 || !b_cfg || !b_st ||
        !b_qpos || !b_qvel || !b_xpos || !b_xmat || !b_xanchor || !b_xaxis ||
        !b_qfrc || !b_ctrl || !b_tendon) {
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
        boundary_span(static_cast<float *>([b_tendon contents]), 2 * nworld_u);

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
                mjData *d_w = mj_makeData(m_t);
                world_data[wu] = d_w;
                mj_resetData(m_t, d_w);
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
                mj_forward(m_t, d_w);

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

                if (m_base->ntendon > 0 && idx.freehub_tendon >= 0 && idx.freehub_tendon < m_base->ntendon) {
                    const std::span<const mjtNum> base_tendon_range =
                        boundary_span(m_base->tendon_range, 2 * to_index(ntendon));
                    const std::size_t ft = 2 * to_index(idx.freehub_tendon);
                    const std::size_t tw = 2 * wu;
                    tendon_v[tw + 0] = static_cast<float>(base_tendon_range[ft + 0]);
                    tendon_v[tw + 1] = static_cast<float>(base_tendon_range[ft + 1]);
                }

                const bike::gpu::GpuWorldConfig &cfg_w = configs_v[to_index(per_world_config ? w : 0)];
                const int fh_qpos_idx =
                    (idx.drive_shaft_spin_qpos >= 0) ? idx.drive_shaft_spin_qpos : idx.crank_spin_qpos;
                const float crank0 =
                    (fh_qpos_idx >= 0 && fh_qpos_idx < nq) ? qp[to_index(fh_qpos_idx)] : 0.0f;
                const float rwheel0 =
                    (idx.rear_wheel_spin_qpos >= 0 && idx.rear_wheel_spin_qpos < nq) ? qp[to_index(idx.rear_wheel_spin_qpos)] : 0.0f;
                bike::gpu::gpu_init_world_state(st_v[wu], cfg_w.drivetrain, crank0, rwheel0);
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
            b_qfrc, b_ctrl, b_tendon, 2
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
            const int fh_qpos_idx =
                (idx.drive_shaft_spin_qpos >= 0) ? idx.drive_shaft_spin_qpos : idx.crank_spin_qpos;
            const float crank0 =
                (fh_qpos_idx >= 0 && fh_qpos_idx < nq) ? qp[to_index(fh_qpos_idx)] : 0.0f;
            const float rwheel0 =
                (idx.rear_wheel_spin_qpos >= 0 && idx.rear_wheel_spin_qpos < nq) ? qp[to_index(idx.rear_wheel_spin_qpos)] : 0.0f;
            bike::gpu::gpu_init_world_state(st_v[to_index(w)], cfg_w.drivetrain, crank0, rwheel0);
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
        if (m_t->ntendon > 0 && idx.freehub_tendon >= 0 && idx.freehub_tendon < m_t->ntendon) {
            const std::span<mjtNum> t_tendon_range =
                boundary_span(m_t->tendon_range, 2 * to_index(ntendon));
            const std::size_t ft = 2 * to_index(idx.freehub_tendon);
            const std::size_t tw = 2 * wu;
            t_tendon_range[ft + 0] = static_cast<mjtNum>(tendon_v[tw + 0]);
            t_tendon_range[ft + 1] = static_cast<mjtNum>(tendon_v[tw + 1]);
        }
        mj_step(m_t, d_t);
        mj_kinematics(m_t, d_t);

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

} // extern "C"
