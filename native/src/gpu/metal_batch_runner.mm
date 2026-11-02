// native/src/gpu/metal_batch_runner.mm — Apple Metal Unified-Memory GPU Force
// Writers & Hybrid (Metal GPU Writers FP32 + Multi-Threaded C++ CPU mj_step FP64)
// Batch Runner for Apple Silicon (M4 Max).

#if !defined(BIKE_METAL_OMIT_CUDA_KERNELS_TU)
#include "cuda_kernels.cu"
#else
// The Metal runner has no CPU fallback: the CUDA host launchers are not referenced from this TU.
#include "batch_step.hpp"
#endif

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#include <mujoco/mujoco.h>

// Host-only asset loaders (hfield_data.bin, heightfield extraction, captured bootstrap state).
// Deliberately NOT in build_msl_shader_source()'s header list: the header #errors under Metal.
#include "model_assets.hpp"
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
#include <type_traits>
#include <vector>

namespace {

// The single raw pointer -> std::span boundary of this TU. The extern "C" batch
// and launch entry points receive (pointer, count) ranges whose extents the caller
// owns (d_configs/d_hf_heights/init_qpos/init_qvel/d_states/d_qpos/d_qvel/d_xpos/
// d_qfrc_applied/d_ctrl/d_tendon_range/out_* sized by nworld, the idx dims or the
// caller's sample count); [MTLBuffer contents] exposes each shared Metal buffer as
// a raw void* over the byte length the buffer was allocated with; MuJoCo likewise
// exposes mjModel/mjData arrays as raw pointers sized by the adjacent model counts
// (qpos/qvel/xpos/qfrc_applied/ctrl by nq/nv/nbody*3/nu, tendon_range by ntendon*2).
// Every such range is wrapped once through this helper; the (pointer, count)
// construction itself is -Wunsafe-buffer-usage-in-container — suppressed here,
// scoped exactly like native/src/gpu/cpu_batch_runner.cpp's boundary_span,
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

// Container subscripts take size_t; the C ABI dims, GpuModelIndices fields and
// the (kMax-checked) mjModel counts are int, so the explicit conversion lives
// in one place (same helper shape as cpu_batch_runner.cpp's to_index).
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
        "common.hpp",
        "suspension_kernel.hpp",
        "tire_kernel.hpp",
        "drivetrain_resistance_kernel.hpp",
        "rider_kernel.hpp",
        "batch_step.hpp"
    };

    std::string combined = R"MSL(
#pragma clang diagnostic ignored "-Wc++17-extensions"
#include <metal_stdlib>
#include <metal_simdgroup>
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
inline bike::gpu::GpuWheelContactOutput bike_metal_wheel_contact_simd32(
    float wheel_x_m,
    float wheel_z_m,
    float wheel_vx_mps,
    float wheel_vz_mps,
    float wheel_omega_rad_s,
    float dt_s,
    device const bike::gpu::GpuTireParams &params,
    thread const bike::gpu::GpuHeightfield1D &hf,
    device bike::gpu::GpuWheelStationState *station_state,
    uint lane
) {
    using namespace bike::gpu;
    const GpuTireParams local_params = params;
    const int n_stations = local_params.num_stations > 0 ? local_params.num_stations : 32;
    const int n_ribs = local_params.num_lateral_ribs > 0 ? local_params.num_lateral_ribs : 1;
    const float station_weight = kTwoPi / static_cast<float>(n_stations);
    const float rib_weight = station_weight / static_cast<float>(n_ribs);
    const float safe_dt = gpu_max(dt_s, 0.0f);
    const float sigma = gpu_max(local_params.relaxation_length_m, 1e-4f);

    GpuWheelRayAccum accum{};
    const int total_rays = n_stations * n_ribs;
    for (int ray_idx = static_cast<int>(lane); ray_idx < total_rays; ray_idx += 32) {
        const int s = ray_idx / n_ribs;
        const int rib = ray_idx - s * n_ribs;
        const bool track_bristle = (n_ribs == 1 && s < kMaxTreadStations && station_state != nullptr);
        float xi = track_bristle ? station_state->bristle_x[s] : 0.0f;
        gpu_evaluate_wheel_contact_ray(
            s,
            rib,
            n_stations,
            n_ribs,
            rib_weight,
            safe_dt,
            sigma,
            wheel_x_m,
            wheel_z_m,
            wheel_vx_mps,
            wheel_vz_mps,
            wheel_omega_rad_s,
            local_params,
            hf,
            track_bristle ? &xi : nullptr,
            accum
        );
        if (track_bristle) {
            station_state->bristle_x[s] = xi;
        }
    }

    GpuWheelRayAccum total_accum{};
    total_accum.fx_world_n = simd_sum(accum.fx_world_n);
    total_accum.fz_world_n = simd_sum(accum.fz_world_n);
    total_accum.spin_torque_nm = simd_sum(accum.spin_torque_nm);
    total_accum.normal_load_n = simd_sum(accum.normal_load_n);
    total_accum.active_stations = simd_sum(accum.active_stations);
    total_accum.weighted_drop_terms = simd_sum(accum.weighted_drop_terms);
    total_accum.weighted_drop_weights = simd_sum(accum.weighted_drop_weights);
    total_accum.unweighted_drop_terms = simd_sum(accum.unweighted_drop_terms);
    total_accum.unweighted_drop_count = simd_sum(accum.unweighted_drop_count);
    total_accum.weighted_cx = simd_sum(accum.weighted_cx);
    total_accum.weighted_cz = simd_sum(accum.weighted_cz);
    total_accum.unweighted_cx = simd_sum(accum.unweighted_cx);
    total_accum.unweighted_cz = simd_sum(accum.unweighted_cz);

    return gpu_finalize_wheel_contact_accum(
        wheel_x_m,
        wheel_z_m,
        local_params.nominal_radius_m,
        total_accum
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
    device const bike::gpu::GpuWorldConfig &cfg = configs[per_world_config ? w : 0];
    const bike::gpu::GpuDrivetrainParams dp = cfg.drivetrain;
    bike::gpu::GpuWorldState st{};
    device const float *qpos_w = (qpos_batch != nullptr) ? (qpos_batch + w * idx.nq) : nullptr;
    const float crank_qpos =
        (qpos_w != nullptr && idx.crank_spin_qpos >= 0 && idx.crank_spin_qpos < idx.nq)
            ? qpos_w[idx.crank_spin_qpos]
            : 0.0f;
    const float rear_wheel_qpos =
        (qpos_w != nullptr && idx.rear_wheel_spin_qpos >= 0 && idx.rear_wheel_spin_qpos < idx.nq)
            ? qpos_w[idx.rear_wheel_spin_qpos]
            : 0.0f;
    bike::gpu::gpu_init_world_state(st, dp, crank_qpos, rear_wheel_qpos);
    states[w] = st;
}

inline void bike_metal_step_writers_scalar_world(
    int w,
    float dt_s,
    thread const bike::gpu::GpuModelIndices &idx,
    thread const bike::gpu::GpuHeightfield1D &hf,
    device const bike::gpu::GpuWorldConfig *configs,
    int per_world_config,
    device bike::gpu::GpuWorldState *states,
    device const float *qpos_batch,
    device const float *qvel_batch,
    device const float *xpos_batch,
    device float *qfrc_batch,
    device float *ctrl_batch,
    device float *tendon_range_batch,
    int tendon_range_world_stride
) {
    if (idx.nq < 0 || idx.nq > bike::gpu::kMaxNq ||
        idx.nv < 0 || idx.nv > bike::gpu::kMaxNv ||
        idx.nu < 0 || idx.nu > bike::gpu::kMaxNu ||
        idx.nbody < 0 || idx.nbody > bike::gpu::kMaxNbody) {
        return;
    }
    const bike::gpu::GpuWorldConfig cfg = configs[per_world_config ? w : 0];
    bike::gpu::GpuWorldState st = states[w];

    float qpos_w[bike::gpu::kMaxNq];
    float qvel_w[bike::gpu::kMaxNv];
    float xpos_w[bike::gpu::kMaxNbody * 3];
    float qfrc_w[bike::gpu::kMaxNv];
    float ctrl_w[bike::gpu::kMaxNu];
    float tendon_w[2] = {0.0f, 0.0f};

    const int nq = idx.nq;
    const int nv = idx.nv;
    const int nu = idx.nu;
    const int nbody3 = idx.nbody * 3;

    device const float *qpos_src = qpos_batch + w * nq;
    device const float *qvel_src = qvel_batch + w * nv;
    device const float *xpos_src = xpos_batch + w * nbody3;
    for (int i = 0; i < nq; ++i) qpos_w[i] = qpos_src[i];
    for (int i = 0; i < nv; ++i) qvel_w[i] = qvel_src[i];
    for (int i = 0; i < nbody3; ++i) xpos_w[i] = xpos_src[i];

    bike::gpu::gpu_step_world_writers(
        0,
        dt_s,
        idx,
        hf,
        &cfg,
        0,
        &st,
        qpos_w,
        qvel_w,
        xpos_w,
        qfrc_w,
        ctrl_w,
        (tendon_range_batch != nullptr) ? tendon_w : nullptr,
        2
    );

    device float *qfrc_dst = qfrc_batch + w * nv;
    device float *ctrl_dst = ctrl_batch + w * nu;
    for (int i = 0; i < nv; ++i) qfrc_dst[i] = qfrc_w[i];
    for (int i = 0; i < nu; ++i) ctrl_dst[i] = ctrl_w[i];
    states[w] = st;

    if (tendon_range_batch != nullptr && idx.freehub_tendon >= 0) {
        if (tendon_range_world_stride > 0) {
            tendon_range_batch[w * tendon_range_world_stride + 1] = tendon_w[1];
        } else if (w == 0) {
            tendon_range_batch[1] = tendon_w[1];
        }
    }
}

inline void bike_metal_step_writers_simd32_world(
    int w,
    float dt_s,
    thread const bike::gpu::GpuModelIndices &idx,
    thread const bike::gpu::GpuHeightfield1D &hf,
    device const bike::gpu::GpuWorldConfig *configs,
    int per_world_config,
    device bike::gpu::GpuWorldState *states,
    device const float *qpos_batch,
    device const float *qvel_batch,
    device const float *xpos_batch,
    device float *qfrc_batch,
    device float *ctrl_batch,
    device float *tendon_range_batch,
    int tendon_range_world_stride,
    uint lane
) {
    if (idx.nq < 0 || idx.nq > bike::gpu::kMaxNq ||
        idx.nv < 0 || idx.nv > bike::gpu::kMaxNv ||
        idx.nu < 0 || idx.nu > bike::gpu::kMaxNu ||
        idx.nbody < 0 || idx.nbody > bike::gpu::kMaxNbody) {
        return;
    }
    device const bike::gpu::GpuWorldConfig &cfg = configs[per_world_config ? w : 0];
    device const float *qpos_src = qpos_batch + w * idx.nq;
    device const float *qvel_src = qvel_batch + w * idx.nv;
    device const float *xpos_src = xpos_batch + w * (idx.nbody * 3);

    const int nq = idx.nq;
    const int nv = idx.nv;
    const int nu = idx.nu;
    const int nbody3 = idx.nbody * 3;

    float xpos_w[bike::gpu::kMaxNbody * 3];
    for (int i = 0; i < nbody3; ++i) {
        xpos_w[i] = xpos_src[i];
    }

    const float com_x = xpos_w[3 * idx.frame_body + 0];
    const float com_z = xpos_w[3 * idx.frame_body + 2];
    const float front_x = xpos_w[3 * idx.front_wheel_body + 0];
    const float front_z = xpos_w[3 * idx.front_wheel_body + 2];
    const float rear_x = xpos_w[3 * idx.rear_wheel_body + 0];
    const float rear_z = xpos_w[3 * idx.rear_wheel_body + 2];

    const float root_pitch =
        (idx.root_pitch_qpos >= 0 && idx.root_pitch_qpos < nq) ? qpos_src[idx.root_pitch_qpos] : 0.0f;
    // GpuModelIndices has no main_pivot qpos field. main_pivot is a hinge, and every joint before it in
    // model.xml is single-DOF, so its qpos address equals main_pivot_dof (both 8).
    const float main_pivot_angle =
        (idx.main_pivot_dof >= 0 && idx.main_pivot_dof < nq) ? qpos_src[idx.main_pivot_dof] : 0.0f;
    const float root_vx = qvel_src[idx.root_x_dof];
    const float root_vz = qvel_src[idx.root_z_dof];
    const float pitch_rate = qvel_src[idx.root_pitch_dof];

    const float v_fork = (idx.fork_travel_dof >= 0) ? qvel_src[idx.fork_travel_dof] : 0.0f;
    const bike::gpu::GpuVec2 e_fork = bike::gpu::gpu_fork_axis_world(idx, xpos_w, root_pitch);
    const float front_vx = root_vx + pitch_rate * (front_z - com_z) + e_fork.x * v_fork;
    const float front_vz = root_vz - pitch_rate * (front_x - com_x) + e_fork.y * v_fork;
    const float front_omega =
        pitch_rate + (idx.front_wheel_spin_dof >= 0 ? qvel_src[idx.front_wheel_spin_dof] : 0.0f);

    const float v_main = (idx.main_pivot_dof >= 0) ? qvel_src[idx.main_pivot_dof] : 0.0f;
    const float v_horst = (idx.horst_pivot_dof >= 0) ? qvel_src[idx.horst_pivot_dof] : 0.0f;
    const bike::gpu::GpuVec2 a_main = bike::gpu::gpu_rear_pivot_anchor_world(
        idx.chainstay_body,
        xpos_w,
        com_x,
        com_z,
        bike::gpu::kMainPivotLocalDxM,
        bike::gpu::kMainPivotLocalDzM,
        root_pitch
    );
    const bike::gpu::GpuVec2 a_horst = bike::gpu::gpu_horst_pivot_anchor_world(
        idx.seatstay_body,
        xpos_w,
        com_x,
        com_z,
        root_pitch,
        main_pivot_angle
    );
    const float rear_vx = root_vx
        + pitch_rate * (rear_z - com_z)
        + v_main * (rear_z - a_main.y)
        + v_horst * (rear_z - a_horst.y);
    const float rear_vz = root_vz
        - pitch_rate * (rear_x - com_x)
        - v_main * (rear_x - a_main.x)
        - v_horst * (rear_x - a_horst.x);
    const float rear_omega =
        pitch_rate + v_main + v_horst +
        (idx.rear_wheel_spin_dof >= 0 ? qvel_src[idx.rear_wheel_spin_dof] : 0.0f);

    const bike::gpu::GpuWheelContactOutput front_out = bike_metal_wheel_contact_simd32(
        front_x, front_z, front_vx, front_vz, front_omega, dt_s, cfg.front_tire, hf, &states[w].front_wheel, lane
    );
    const bike::gpu::GpuWheelContactOutput rear_out = bike_metal_wheel_contact_simd32(
        rear_x, rear_z, rear_vx, rear_vz, rear_omega, dt_s, cfg.rear_tire, hf, &states[w].rear_wheel, lane
    );

    if (lane == 0) {
        float qpos_w[bike::gpu::kMaxNq];
        float qvel_w[bike::gpu::kMaxNv];
        float qfrc_w[bike::gpu::kMaxNv];
        float ctrl_w[bike::gpu::kMaxNu];
        for (int i = 0; i < nq; ++i) qpos_w[i] = qpos_src[i];
        for (int i = 0; i < nv; ++i) { qvel_w[i] = qvel_src[i]; qfrc_w[i] = 0.0f; }
        for (int i = 0; i < nu; ++i) ctrl_w[i] = 0.0f;

        {
            const bike::gpu::GpuSuspensionParams susp = cfg.suspension;
            bike::gpu::gpu_write_suspension_qfrc(susp, idx, qpos_w, qvel_w, qfrc_w);
        }

        bike::gpu::project_front_wheel_force_to_qfrc(
            idx,
            xpos_w,
            com_x,
            com_z,
            front_out.contact_x_m,
            front_out.contact_z_m,
            front_out.fx_world_n,
            front_out.fz_world_n,
            qfrc_w,
            root_pitch
        );
        if (idx.front_wheel_spin_dof >= 0) {
            qfrc_w[idx.front_wheel_spin_dof] += front_out.spin_torque_nm;
        }

        bike::gpu::project_rear_wheel_force_to_qfrc(
            idx,
            xpos_w,
            com_x,
            com_z,
            rear_out.contact_x_m,
            rear_out.contact_z_m,
            rear_out.fx_world_n,
            rear_out.fz_world_n,
            qfrc_w,
            root_pitch,
            main_pivot_angle
        );
        if (idx.rear_wheel_spin_dof >= 0) {
            qfrc_w[idx.rear_wheel_spin_dof] += rear_out.spin_torque_nm;
        }

        const bool include_rear_drive_torque =
            (tendon_range_batch == nullptr) || (idx.freehub_tendon < 0);
        bike::gpu::GpuDrivetrainOutput dt_out{};
        {
            const bike::gpu::GpuResistanceParams res = cfg.resistance;
            const bike::gpu::GpuDrivetrainParams dp = cfg.drivetrain;
            bike::gpu::GpuDrivetrainState dt_st = states[w].drivetrain;
            dt_out = bike::gpu::gpu_write_drivetrain_and_resistance_qfrc(
                res, dp, idx, dt_st, qpos_w, qvel_w,
                front_out.normal_load_n, rear_out.normal_load_n,
                front_out.effective_radius_m, rear_out.effective_radius_m,
                cfg.pedal_demand, cfg.front_brake_demand, cfg.rear_brake_demand,
                qfrc_w, include_rear_drive_torque
            );
            states[w].drivetrain = dt_st;
        }

        {
            const bike::gpu::GpuRiderParams rider = cfg.rider;
            bike::gpu::gpu_write_rider_qfrc_and_ctrl(rider, idx, qpos_w, qvel_w, qfrc_w, ctrl_w);
        }

        device float *qfrc_dst = qfrc_batch + w * idx.nv;
        device float *ctrl_dst = ctrl_batch + w * idx.nu;
        for (int i = 0; i < nv; ++i) qfrc_dst[i] = qfrc_w[i];
        for (int i = 0; i < nu; ++i) ctrl_dst[i] = ctrl_w[i];

        if (tendon_range_batch != nullptr && idx.freehub_tendon >= 0) {
            if (tendon_range_world_stride > 0) {
                tendon_range_batch[w * tendon_range_world_stride + 1] = dt_out.tendon_upper_bound_rad;
            } else if (w == 0) {
                tendon_range_batch[1] = dt_out.tendon_upper_bound_rad;
            }
        }
    }
}

// 1. Scalar step kernel (Scenario 3 Hybrid)
inline void bike_metal_step_writers_impl(
    constant int &world_offset,
    constant int &world_count,
    constant float &dt_s,
    constant bike::gpu::GpuModelIndices &idx_const,
    device const float *hf_heights,
    constant int &hf_num_samples,
    constant float &hf_x_min,
    constant float &hf_dx,
    device const bike::gpu::GpuWorldConfig *configs,
    constant int &per_world_config,
    device bike::gpu::GpuWorldState *states,
    device const float *qpos_batch,
    device const float *qvel_batch,
    device const float *xpos_batch,
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
    const float inv_dx = hf_dx > 0.0f ? (1.0f / hf_dx) : 1.0f;
    const bike::gpu::GpuHeightfield1D hf{hf_heights, hf_num_samples, hf_x_min, hf_dx, inv_dx};
    bike_metal_step_writers_scalar_world(
        w, dt_s, idx, hf, configs, per_world_config, states,
        qpos_batch, qvel_batch, xpos_batch, qfrc_batch, ctrl_batch,
        tendon_range_batch, tendon_range_world_stride
    );
}

// 2. SIMD32 step kernel (Scenario 3 Hybrid)
inline void bike_metal_step_writers_simd32_impl(
    constant int &world_offset,
    constant int &world_count,
    constant float &dt_s,
    constant bike::gpu::GpuModelIndices &idx_const,
    device const float *hf_heights,
    constant int &hf_num_samples,
    constant float &hf_x_min,
    constant float &hf_dx,
    device const bike::gpu::GpuWorldConfig *configs,
    constant int &per_world_config,
    device bike::gpu::GpuWorldState *states,
    device const float *qpos_batch,
    device const float *qvel_batch,
    device const float *xpos_batch,
    device float *qfrc_batch,
    device float *ctrl_batch,
    device float *tendon_range_batch,
    constant int &tendon_range_world_stride,
    uint gid,
    uint lane
) {
    const int local_w = static_cast<int>(gid >> 5);
    if (local_w >= world_count) {
        return;
    }
    const int w = world_offset + local_w;
    const bike::gpu::GpuModelIndices idx = idx_const;
    const float inv_dx = hf_dx > 0.0f ? (1.0f / hf_dx) : 1.0f;
    const bike::gpu::GpuHeightfield1D hf{hf_heights, hf_num_samples, hf_x_min, hf_dx, inv_dx};
    bike_metal_step_writers_simd32_world(
        w, dt_s, idx, hf, configs, per_world_config, states,
        qpos_batch, qvel_batch, xpos_batch, qfrc_batch, ctrl_batch,
        tendon_range_batch, tendon_range_world_stride, lane
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
    device const float *hf_heights;
    device const bike::gpu::GpuWorldConfig *configs;
    device bike::gpu::GpuWorldState *states;
    device const float *qpos;
    device const float *qvel;
    device const float *xpos;
    device float *qfrc_applied;
    device float *ctrl;
    device float *tendon_range;
    int nworld;
    float dt_s;
    int hf_num_samples;
    float hf_x_min;
    float hf_dx;
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
    device const bike::gpu::GpuWorldConfig &cfg = args->configs[args->per_world_config ? w : 0];
    const bike::gpu::GpuDrivetrainParams dp = cfg.drivetrain;
    bike::gpu::GpuWorldState st{};
    device const float *qpos_src = (args->qpos != nullptr) ? (args->qpos + w * idx.nq) : nullptr;
    const float crank_qpos =
        (qpos_src != nullptr && idx.crank_spin_qpos >= 0 && idx.crank_spin_qpos < idx.nq)
            ? qpos_src[idx.crank_spin_qpos]
            : 0.0f;
    const float rear_wheel_qpos =
        (qpos_src != nullptr && idx.rear_wheel_spin_qpos >= 0 && idx.rear_wheel_spin_qpos < idx.nq)
            ? qpos_src[idx.rear_wheel_spin_qpos]
            : 0.0f;
    bike::gpu::gpu_init_world_state(st, dp, crank_qpos, rear_wheel_qpos);
    args->states[w] = st;
}

// 3. Fused Scalar step kernel (Scenario 2 warp-metal)
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
    const float inv_dx = args->hf_dx > 0.0f ? (1.0f / args->hf_dx) : 1.0f;
    const bike::gpu::GpuHeightfield1D hf{
        args->hf_heights, args->hf_num_samples, args->hf_x_min, args->hf_dx, inv_dx
    };
    bike_metal_step_writers_scalar_world(
        w, args->dt_s, idx, hf, args->configs, args->per_world_config, args->states,
        args->qpos, args->qvel, args->xpos, args->qfrc_applied, args->ctrl,
        args->tendon_range, args->tendon_range_world_stride
    );
}

// 4. Fused SIMD32 step kernel (Scenario 2 warp-metal)
inline void bike_metal_step_writers_fused_simd32_impl(
    constant ulong *bounds_buf,
    constant BikeWpWritersArgs *args,
    uint gid,
    uint lane
) {
    (void)bounds_buf;
    const int w = static_cast<int>(gid >> 5);
    if (w >= args->nworld) {
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (args->idx != nullptr) ? *(args->idx) : bike::gpu::GpuModelIndices{};
    const float inv_dx = args->hf_dx > 0.0f ? (1.0f / args->hf_dx) : 1.0f;
    const bike::gpu::GpuHeightfield1D hf{
        args->hf_heights, args->hf_num_samples, args->hf_x_min, args->hf_dx, inv_dx
    };
    bike_metal_step_writers_simd32_world(
        w, args->dt_s, idx, hf, args->configs, args->per_world_config, args->states,
        args->qpos, args->qvel, args->xpos, args->qfrc_applied, args->ctrl,
        args->tendon_range, args->tendon_range_world_stride, lane
    );
}

kernel void bike_gpu_writers_metal_kernel(
    constant int &world_offset [[buffer(0)]],
    constant int &world_count [[buffer(1)]],
    constant float &dt_s [[buffer(2)]],
    constant bike::gpu::GpuModelIndices &idx_const [[buffer(3)]],
    device const float *hf_heights [[buffer(4)]],
    constant int &hf_num_samples [[buffer(5)]],
    constant float &hf_x_min [[buffer(6)]],
    constant float &hf_dx [[buffer(7)]],
    device const bike::gpu::GpuWorldConfig *configs [[buffer(8)]],
    constant int &per_world_config [[buffer(9)]],
    device bike::gpu::GpuWorldState *states [[buffer(10)]],
    device const float *qpos_batch [[buffer(11)]],
    device const float *qvel_batch [[buffer(12)]],
    device const float *xpos_batch [[buffer(13)]],
    device float *qfrc_batch [[buffer(14)]],
    device float *ctrl_batch [[buffer(15)]],
    device float *tendon_range_batch [[buffer(16)]],
    constant int &tendon_range_world_stride [[buffer(17)]],
    uint gid [[thread_position_in_grid]]
) {
    bike_metal_step_writers_impl(
        world_offset, world_count, dt_s, idx_const, hf_heights, hf_num_samples, hf_x_min, hf_dx,
        configs, per_world_config, states, qpos_batch, qvel_batch, xpos_batch, qfrc_batch, ctrl_batch,
        tendon_range_batch, tendon_range_world_stride, gid
    );
}

kernel void bike_gpu_writers_simd32_kernel(
    constant int &world_offset [[buffer(0)]],
    constant int &world_count [[buffer(1)]],
    constant float &dt_s [[buffer(2)]],
    constant bike::gpu::GpuModelIndices &idx_const [[buffer(3)]],
    device const float *hf_heights [[buffer(4)]],
    constant int &hf_num_samples [[buffer(5)]],
    constant float &hf_x_min [[buffer(6)]],
    constant float &hf_dx [[buffer(7)]],
    device const bike::gpu::GpuWorldConfig *configs [[buffer(8)]],
    constant int &per_world_config [[buffer(9)]],
    device bike::gpu::GpuWorldState *states [[buffer(10)]],
    device const float *qpos_batch [[buffer(11)]],
    device const float *qvel_batch [[buffer(12)]],
    device const float *xpos_batch [[buffer(13)]],
    device float *qfrc_batch [[buffer(14)]],
    device float *ctrl_batch [[buffer(15)]],
    device float *tendon_range_batch [[buffer(16)]],
    constant int &tendon_range_world_stride [[buffer(17)]],
    uint gid [[thread_position_in_grid]],
    uint lane [[thread_index_in_simdgroup]]
) {
    bike_metal_step_writers_simd32_impl(
        world_offset, world_count, dt_s, idx_const, hf_heights, hf_num_samples, hf_x_min, hf_dx,
        configs, per_world_config, states, qpos_batch, qvel_batch, xpos_batch, qfrc_batch, ctrl_batch,
        tendon_range_batch, tendon_range_world_stride, gid, lane
    );
}

kernel void bike_wp_writers_metal_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpWritersArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    bike_metal_step_writers_fused_impl(bounds_buf, args, gid);
}

kernel void bike_wp_writers_simd32_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpWritersArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]],
    uint lane [[thread_index_in_simdgroup]]
) {
    bike_metal_step_writers_fused_simd32_impl(bounds_buf, args, gid, lane);
}

kernel void bike_metal_step_writers_kernel(
    constant int &world_offset [[buffer(0)]],
    constant int &world_count [[buffer(1)]],
    constant float &dt_s [[buffer(2)]],
    constant bike::gpu::GpuModelIndices &idx_const [[buffer(3)]],
    device const float *hf_heights [[buffer(4)]],
    constant int &hf_num_samples [[buffer(5)]],
    constant float &hf_x_min [[buffer(6)]],
    constant float &hf_dx [[buffer(7)]],
    device const bike::gpu::GpuWorldConfig *configs [[buffer(8)]],
    constant int &per_world_config [[buffer(9)]],
    device bike::gpu::GpuWorldState *states [[buffer(10)]],
    device const float *qpos_batch [[buffer(11)]],
    device const float *qvel_batch [[buffer(12)]],
    device const float *xpos_batch [[buffer(13)]],
    device float *qfrc_batch [[buffer(14)]],
    device float *ctrl_batch [[buffer(15)]],
    device float *tendon_range_batch [[buffer(16)]],
    constant int &tendon_range_world_stride [[buffer(17)]],
    uint gid [[thread_position_in_grid]]
) {
    bike_metal_step_writers_impl(
        world_offset, world_count, dt_s, idx_const, hf_heights, hf_num_samples, hf_x_min, hf_dx,
        configs, per_world_config, states, qpos_batch, qvel_batch, xpos_batch, qfrc_batch, ctrl_batch,
        tendon_range_batch, tendon_range_world_stride, gid
    );
}

kernel void bike_metal_step_writers_simd32_kernel(
    constant int &world_offset [[buffer(0)]],
    constant int &world_count [[buffer(1)]],
    constant float &dt_s [[buffer(2)]],
    constant bike::gpu::GpuModelIndices &idx_const [[buffer(3)]],
    device const float *hf_heights [[buffer(4)]],
    constant int &hf_num_samples [[buffer(5)]],
    constant float &hf_x_min [[buffer(6)]],
    constant float &hf_dx [[buffer(7)]],
    device const bike::gpu::GpuWorldConfig *configs [[buffer(8)]],
    constant int &per_world_config [[buffer(9)]],
    device bike::gpu::GpuWorldState *states [[buffer(10)]],
    device const float *qpos_batch [[buffer(11)]],
    device const float *qvel_batch [[buffer(12)]],
    device const float *xpos_batch [[buffer(13)]],
    device float *qfrc_batch [[buffer(14)]],
    device float *ctrl_batch [[buffer(15)]],
    device float *tendon_range_batch [[buffer(16)]],
    constant int &tendon_range_world_stride [[buffer(17)]],
    uint gid [[thread_position_in_grid]],
    uint lane [[thread_index_in_simdgroup]]
) {
    bike_metal_step_writers_simd32_impl(
        world_offset, world_count, dt_s, idx_const, hf_heights, hf_num_samples, hf_x_min, hf_dx,
        configs, per_world_config, states, qpos_batch, qvel_batch, xpos_batch, qfrc_batch, ctrl_batch,
        tendon_range_batch, tendon_range_world_stride, gid, lane
    );
}

kernel void bike_metal_step_writers_fused_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpWritersArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    bike_metal_step_writers_fused_impl(bounds_buf, args, gid);
}

kernel void bike_metal_step_writers_fused_simd32_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpWritersArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]],
    uint lane [[thread_index_in_simdgroup]]
) {
    bike_metal_step_writers_fused_simd32_impl(bounds_buf, args, gid, lane);
}
)MSL";
    return combined;
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
    bool ok = false;
    id<MTLDevice> device = nil;
    id<MTLCommandQueue> queue = nil;
    id<MTLLibrary> library = nil;
    id<MTLComputePipelineState> pso_init_states = nil;
    id<MTLComputePipelineState> pso_1thread = nil;
    id<MTLComputePipelineState> pso_simd32 = nil;
    id<MTLComputePipelineState> pso_step_writers_simd32 = nil;
    id<MTLComputePipelineState> pso_step_writers_fused = nil;
    id<MTLComputePipelineState> pso_step_writers_fused_simd32 = nil;

    bool init(const std::string &gpu_src_dir) {
        if (ok && pso_1thread != nil && pso_simd32 != nil && pso_init_states != nil) {
            return true;
        }
        ok = false;
        if (!device) {
            device = MTLCreateSystemDefaultDevice();
        }
        if (!device) {
            std::cerr << "[MetalWritersContext] MTLCreateSystemDefaultDevice failed\n";
            return false;
        }
        if (!queue) {
            queue = [device newCommandQueue];
        }
        if (!queue) {
            std::cerr << "[MetalWritersContext] newCommandQueue failed\n";
            return false;
        }
        const std::string src = build_msl_shader_source(gpu_src_dir);
        if (src.empty()) {
            std::cerr << "[MetalWritersContext] Empty MSL shader source from dir '"
                      << gpu_src_dir << "'\n";
            return false;
        }
        NSError *err = nil;
        library = [device newLibraryWithSource:[NSString stringWithUTF8String:src.c_str()]
                                       options:nil
                                         error:&err];
        if (!library) {
            std::cerr << "[MetalWritersContext] MSL compile error:\n"
                      << (err ? [[err localizedDescription] UTF8String] : "unknown Metal error")
                      << '\n';
            return false;
        }
        id<MTLFunction> fn_init = [library newFunctionWithName:@"bike_gpu_init_states_metal_kernel"];
        id<MTLFunction> fn1 = [library newFunctionWithName:@"bike_gpu_writers_metal_kernel"];
        id<MTLFunction> fn32 = [library newFunctionWithName:@"bike_gpu_writers_simd32_kernel"];
        id<MTLFunction> fn_wp1 = [library newFunctionWithName:@"bike_wp_writers_metal_kernel"];
        id<MTLFunction> fn_wp32 = [library newFunctionWithName:@"bike_wp_writers_simd32_kernel"];
        if (!fn_init || !fn1 || !fn32 || !fn_wp1 || !fn_wp32) {
            std::cerr << "[MetalWritersContext] Failed to look up MSL kernel functions\n";
            return false;
        }

        pso_init_states = [device newComputePipelineStateWithFunction:fn_init error:&err];
        pso_1thread = [device newComputePipelineStateWithFunction:fn1 error:&err];
        pso_simd32 = [device newComputePipelineStateWithFunction:fn32 error:&err];
        pso_step_writers_simd32 = pso_simd32;
        pso_step_writers_fused = [device newComputePipelineStateWithFunction:fn_wp1 error:&err];
        pso_step_writers_fused_simd32 = [device newComputePipelineStateWithFunction:fn_wp32 error:&err];
        if (!pso_init_states || !pso_1thread || !pso_simd32 ||
            !pso_step_writers_fused || !pso_step_writers_fused_simd32) {
            std::cerr << "[MetalWritersContext] PSO error:\n"
                      << (err ? [[err localizedDescription] UTF8String] : "unknown Metal error")
                      << '\n';
            return false;
        }

        if (pso_step_writers_simd32.threadExecutionWidth != 32 ||
            pso_step_writers_fused_simd32.threadExecutionWidth != 32) {
            std::cerr << "[MetalWritersContext] Invalid threadExecutionWidth for simd32 kernels: "
                      << static_cast<unsigned long>(pso_step_writers_simd32.threadExecutionWidth)
                      << " and "
                      << static_cast<unsigned long>(pso_step_writers_fused_simd32.threadExecutionWidth)
                      << " (expected 32)\n";
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
    id<MTLBuffer> b_idx, NSUInteger off_idx,
    id<MTLBuffer> b_hf, NSUInteger off_hf,
    int hf_num_samples,
    float hf_x_min,
    float hf_dx,
    id<MTLBuffer> b_cfg, NSUInteger off_cfg,
    int per_world_config,
    id<MTLBuffer> b_st, NSUInteger off_st,
    id<MTLBuffer> b_qpos, NSUInteger off_qpos,
    id<MTLBuffer> b_qvel, NSUInteger off_qvel,
    id<MTLBuffer> b_xpos, NSUInteger off_xpos,
    id<MTLBuffer> b_qfrc, NSUInteger off_qfrc,
    id<MTLBuffer> b_ctrl, NSUInteger off_ctrl,
    id<MTLBuffer> b_tendon, NSUInteger off_tendon,
    int tendon_stride,
    bool use_simd32
) {
    [enc setComputePipelineState:(use_simd32 ? ctx.pso_simd32 : ctx.pso_1thread)];
    [enc setBytes:&world_offset length:sizeof(int) atIndex:0];
    [enc setBytes:&world_count length:sizeof(int) atIndex:1];
    [enc setBytes:&dt_s length:sizeof(float) atIndex:2];
    [enc setBuffer:b_idx offset:off_idx atIndex:3];
    [enc setBuffer:b_hf offset:off_hf atIndex:4];
    [enc setBytes:&hf_num_samples length:sizeof(int) atIndex:5];
    [enc setBytes:&hf_x_min length:sizeof(float) atIndex:6];
    [enc setBytes:&hf_dx length:sizeof(float) atIndex:7];
    [enc setBuffer:b_cfg offset:off_cfg atIndex:8];
    [enc setBytes:&per_world_config length:sizeof(int) atIndex:9];
    [enc setBuffer:b_st offset:off_st atIndex:10];
    [enc setBuffer:b_qpos offset:off_qpos atIndex:11];
    [enc setBuffer:b_qvel offset:off_qvel atIndex:12];
    [enc setBuffer:b_xpos offset:off_xpos atIndex:13];
    [enc setBuffer:b_qfrc offset:off_qfrc atIndex:14];
    [enc setBuffer:b_ctrl offset:off_ctrl atIndex:15];
    [enc setBuffer:b_tendon offset:off_tendon atIndex:16];
    [enc setBytes:&tendon_stride length:sizeof(int) atIndex:17];

    if (use_simd32) {
        const NSUInteger total_threads = static_cast<NSUInteger>(world_count) * 32;
        const NSUInteger tg = std::min<NSUInteger>(128, total_threads);
        [enc dispatchThreads:MTLSizeMake(total_threads, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
    } else {
        const NSUInteger total_threads = static_cast<NSUInteger>(world_count);
        const NSUInteger tg = std::min<NSUInteger>(64, std::max<NSUInteger>(1, total_threads));
        [enc dispatchThreads:MTLSizeMake(total_threads, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
    }
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

// Initializes per-world states on the Metal GPU. Returns 0 or a launch-ABI code (see the table above
// bike_metal_launch_writers). Outputs are copied back only after a successful command buffer.
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

// Dispatches `bike_gpu_writers_*_kernel` on Apple Metal GPU.
//
// Launch ABI shared by bike_metal_launch_init_states, bike_metal_launch_writers and
// bike_metal_launch_step_writers:
//   0   success; outputs are copied back only after the command buffer completed
//  -1   invalid arguments (nworld <= 0, a required pointer is null, or model dimensions exceed kMax*)
//  -2   Metal context or a required pipeline is not initialized (bike_metal_init failed or PSO missing)
//  -3   Metal buffer allocation failed
//  -4   command buffer failed (wait_command_buffer_ok returned false)
// There is no CPU fallback. On a non-zero return the caller's output arrays are left unmodified.
int bike_metal_launch_writers(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const float *d_hf_heights,
    int hf_num_samples,
    float hf_x_min,
    float hf_dx,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    float *d_qfrc_applied,
    float *d_ctrl,
    float *d_tendon_range,
    int tendon_range_world_stride
) {
    if (nworld <= 0 || d_idx == nullptr || d_configs == nullptr || d_states == nullptr ||
        d_qpos == nullptr || d_qvel == nullptr || d_xpos == nullptr ||
        d_qfrc_applied == nullptr || d_ctrl == nullptr) {
        return -1;
    }
    const bike::gpu::GpuModelIndices idx_val = *d_idx;
    if (idx_val.nq <= 0 || idx_val.nq > bike::gpu::kMaxNq ||
        idx_val.nv <= 0 || idx_val.nv > bike::gpu::kMaxNv ||
        idx_val.nu <= 0 || idx_val.nu > bike::gpu::kMaxNu ||
        idx_val.nbody <= 0 || idx_val.nbody > bike::gpu::kMaxNbody) {
        std::cerr << "[bike_metal_launch_writers] Model dimensions exceed kMax bounds (nq="
                  << idx_val.nq << ", nv=" << idx_val.nv << ", nu=" << idx_val.nu
                  << ", nbody=" << idx_val.nbody << ")\n";
        return -1;
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_1thread || !ctx.pso_simd32) {
        std::cerr << "[bike_metal_launch_writers] Metal context or pipeline not initialized\n";
        return -2;
    }

    const int nq = idx_val.nq;
    const int nv = idx_val.nv;
    const int nu = idx_val.nu;
    const int nbody3 = idx_val.nbody * 3;
    const int n_cfg = per_world_config ? nworld : 1;
    const bool use_simd32 = (d_configs[0].front_tire.num_lateral_ribs > 1) || (nworld < 64);

    const std::size_t hf_bytes =
        static_cast<std::size_t>(std::max(1, hf_num_samples)) * sizeof(float);
    const std::size_t cfg_bytes =
        static_cast<std::size_t>(n_cfg) * sizeof(bike::gpu::GpuWorldConfig);
    const std::size_t st_bytes =
        static_cast<std::size_t>(nworld) * sizeof(bike::gpu::GpuWorldState);
    const std::size_t qpos_bytes = static_cast<std::size_t>(nworld * nq) * sizeof(float);
    const std::size_t qvel_bytes = static_cast<std::size_t>(nworld * nv) * sizeof(float);
    const std::size_t xpos_bytes = static_cast<std::size_t>(nworld * nbody3) * sizeof(float);
    const std::size_t qfrc_bytes = static_cast<std::size_t>(nworld * nv) * sizeof(float);
    const std::size_t ctrl_bytes = static_cast<std::size_t>(nworld * nu) * sizeof(float);
    const std::size_t tendon_floats =
        (tendon_range_world_stride > 0)
            ? static_cast<std::size_t>(nworld * tendon_range_world_stride)
            : 2ULL;
    const std::size_t tendon_bytes = tendon_floats * sizeof(float);

    @autoreleasepool {
        id<MTLBuffer> b_idx = [ctx.device newBufferWithBytes:&idx_val
                                                      length:sizeof(bike::gpu::GpuModelIndices)
                                                     options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_hf = (d_hf_heights != nullptr && hf_num_samples > 0)
            ? [ctx.device newBufferWithBytes:d_hf_heights
                                      length:hf_bytes
                                     options:MTLResourceStorageModeShared]
            : nil;
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
        id<MTLBuffer> b_qfrc = [ctx.device newBufferWithLength:qfrc_bytes
                                                       options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_ctrl = [ctx.device newBufferWithLength:ctrl_bytes
                                                       options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_tendon = (d_tendon_range != nullptr)
            ? [ctx.device newBufferWithBytes:d_tendon_range
                                      length:tendon_bytes
                                     options:MTLResourceStorageModeShared]
            : nil;

        if (!b_idx || !b_cfg || !b_st || !b_qpos || !b_qvel || !b_xpos || !b_qfrc || !b_ctrl ||
            (d_hf_heights != nullptr && hf_num_samples > 0 && !b_hf) ||
            (d_tendon_range != nullptr && !b_tendon)) {
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
            b_idx, 0,
            b_hf, 0,
            hf_num_samples,
            hf_x_min,
            hf_dx,
            b_cfg, 0,
            per_world_config,
            b_st, 0,
            b_qpos, 0,
            b_qvel, 0,
            b_xpos, 0,
            b_qfrc, 0,
            b_ctrl, 0,
            b_tendon, 0,
            tendon_range_world_stride,
            use_simd32
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

// Benchmarks pure Apple Metal GPU writers execution time across `n_steps` for `nworld` worlds.
// Return codes: 0 success; -1 invalid arguments or model dimensions exceed kMax*;
// -2 Metal context or pipeline not initialized; -3 Metal buffer allocation failed;
// -4 command buffer failed. Timings are written only on success.
int bike_metal_benchmark_writers_only(
    int nworld,
    int n_steps,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const float *d_hf_heights,
    int hf_num_samples,
    float hf_x_min,
    float hf_dx,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    double *out_single_cb_ms,
    double *out_per_step_cb_ms
) {
    if (nworld <= 0 || n_steps <= 0 || d_idx == nullptr || d_configs == nullptr) {
        return -1;
    }
    const bike::gpu::GpuModelIndices idx_val = *d_idx;
    const bike::gpu::GpuModelIndices *norm_idx = &idx_val;
    if (norm_idx->nq > bike::gpu::kMaxNq ||
        norm_idx->nv > bike::gpu::kMaxNv ||
        norm_idx->nu > bike::gpu::kMaxNu ||
        norm_idx->nbody > bike::gpu::kMaxNbody) {
        return -1;
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_1thread || !ctx.pso_simd32) {
        return -2;
    }

    (void)d_hf_heights;
    const int nq = norm_idx->nq;
    const int nv = norm_idx->nv;
    const int nu = norm_idx->nu;
    const int nbody3 = norm_idx->nbody * 3;
    const int n_cfg = per_world_config ? nworld : 1;
    const std::span<const bike::gpu::GpuWorldConfig> configs_v =
        boundary_span(d_configs, to_index(n_cfg));
    const bool use_simd32 = (configs_v[0].front_tire.num_lateral_ribs > 1) || (nworld < 64);

    const int bench_hf_samples = (hf_num_samples > 0) ? hf_num_samples : 4000;
    const float bench_hf_x_min = (hf_num_samples > 0) ? hf_x_min : -10.0f;
    const float bench_hf_dx = (hf_dx > 0.0f) ? hf_dx : 0.005f;

    id<MTLBuffer> b_idx = [ctx.device newBufferWithBytes:norm_idx
                                                  length:sizeof(bike::gpu::GpuModelIndices)
                                                 options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_hf = [ctx.device newBufferWithLength:static_cast<NSUInteger>(bench_hf_samples) * sizeof(float)
                                                 options:MTLResourceStorageModeShared];
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
    id<MTLBuffer> b_qfrc = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * nv) * sizeof(float)
                                                   options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_ctrl = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * nu) * sizeof(float)
                                                   options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_tendon = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * 2) * sizeof(float)
                                                     options:MTLResourceStorageModeShared];
    if (!b_idx || !b_hf || !b_cfg || !b_st || !b_qpos || !b_qvel || !b_xpos || !b_qfrc || !b_ctrl || !b_tendon) {
        std::cerr << "[bike_metal_benchmark_writers_only] Failed to allocate Metal buffers\n";
        return -3;
    }

    // Initialize realistic in-contact state (`d_hf_heights` at 0.0f, wheels at z = 0.34f,
    // forward speed 8.0 m/s, wheel spin 22.0 rad/s) so ray-grid contact and bristle friction execute.
    const std::span<float> hf_v =
        boundary_span(static_cast<float *>([b_hf contents]), to_index(bench_hf_samples));
    for (std::size_t i = 0; i < hf_v.size(); ++i) {
        hf_v[i] = 0.0f;
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
    const std::span<float> tendon_v =
        boundary_span(static_cast<float *>([b_tendon contents]), 2 * nworld_u);

    for (int w = 0; w < nworld; ++w) {
        const std::size_t wu = to_index(w);
        const std::span<float> qp = qpos_v.subspan(wu * to_index(nq), to_index(nq));
        const std::span<float> qv = qvel_v.subspan(wu * to_index(nv), to_index(nv));
        const std::span<float> xp = xpos_v.subspan(wu * to_index(nbody3), to_index(nbody3));
        for (std::size_t i = 0; i < to_index(nq); ++i) qp[i] = 0.0f;
        for (std::size_t i = 0; i < to_index(nv); ++i) qv[i] = 0.0f;
        for (std::size_t i = 0; i < to_index(nbody3); ++i) xp[i] = 0.0f;

        if (norm_idx->root_z_qpos >= 0 && norm_idx->root_z_qpos < nq) qp[to_index(norm_idx->root_z_qpos)] = 0.55f;
        if (norm_idx->fork_travel_qpos >= 0 && norm_idx->fork_travel_qpos < nq) qp[to_index(norm_idx->fork_travel_qpos)] = 0.035f;
        if (norm_idx->shock_stroke_qpos >= 0 && norm_idx->shock_stroke_qpos < nq) qp[to_index(norm_idx->shock_stroke_qpos)] = 0.018f;

        if (norm_idx->root_x_dof >= 0 && norm_idx->root_x_dof < nv) qv[to_index(norm_idx->root_x_dof)] = 8.0f;
        if (norm_idx->rider_dof[6] >= 0 && norm_idx->rider_dof[6] < nv) qv[to_index(norm_idx->rider_dof[6])] = 8.0f;
        if (norm_idx->front_wheel_spin_dof >= 0 && norm_idx->front_wheel_spin_dof < nv) qv[to_index(norm_idx->front_wheel_spin_dof)] = 22.0f;
        if (norm_idx->rear_wheel_spin_dof >= 0 && norm_idx->rear_wheel_spin_dof < nv) qv[to_index(norm_idx->rear_wheel_spin_dof)] = 22.0f;
        if (norm_idx->crank_spin_dof >= 0 && norm_idx->crank_spin_dof < nv) {
            qv[to_index(norm_idx->crank_spin_dof)] = 22.0f * (51.0f / 22.0f);
        }

        if (norm_idx->frame_body >= 0 && 3 * norm_idx->frame_body + 2 < nbody3) {
            xp[3 * to_index(norm_idx->frame_body) + 0] = 0.0f;
            xp[3 * to_index(norm_idx->frame_body) + 2] = 0.55f;
        }
        if (norm_idx->steer_body >= 0 && 3 * norm_idx->steer_body + 2 < nbody3) {
            xp[3 * to_index(norm_idx->steer_body) + 0] = 0.42f;
            xp[3 * to_index(norm_idx->steer_body) + 2] = 0.72f;
        }
        if (norm_idx->fork_lower_body >= 0 && 3 * norm_idx->fork_lower_body + 2 < nbody3) {
            xp[3 * to_index(norm_idx->fork_lower_body) + 0] = 0.58f;
            xp[3 * to_index(norm_idx->fork_lower_body) + 2] = 0.39f;
        }
        if (norm_idx->front_wheel_body >= 0 && 3 * norm_idx->front_wheel_body + 2 < nbody3) {
            xp[3 * to_index(norm_idx->front_wheel_body) + 0] = 0.62f;
            xp[3 * to_index(norm_idx->front_wheel_body) + 2] = 0.34f;
        }
        if (norm_idx->chainstay_body >= 0 && 3 * norm_idx->chainstay_body + 2 < nbody3) {
            xp[3 * to_index(norm_idx->chainstay_body) + 0] = bike::gpu::kMainPivotLocalDxM;
            xp[3 * to_index(norm_idx->chainstay_body) + 2] = 0.55f + bike::gpu::kMainPivotLocalDzM;
        }
        if (norm_idx->seatstay_body >= 0 && 3 * norm_idx->seatstay_body + 2 < nbody3) {
            xp[3 * to_index(norm_idx->seatstay_body) + 0] = bike::gpu::kHorstPivotLocalDxM;
            xp[3 * to_index(norm_idx->seatstay_body) + 2] = 0.55f + bike::gpu::kHorstPivotLocalDzM;
        }
        if (norm_idx->rear_wheel_body >= 0 && 3 * norm_idx->rear_wheel_body + 2 < nbody3) {
            xp[3 * to_index(norm_idx->rear_wheel_body) + 0] = -0.44f;
            xp[3 * to_index(norm_idx->rear_wheel_body) + 2] = 0.34f;
        }

        const std::size_t tw = 2 * wu;
        tendon_v[tw + 0] = -100000.0f;
        tendon_v[tw + 1] = 0.0f;

        const bike::gpu::GpuWorldConfig &cfg_w = configs_v[to_index(per_world_config ? w : 0)];
        bike::gpu::gpu_init_world_state(st_v[wu], cfg_w.drivetrain, 0.0f, 0.0f);
    }

    // Warmup
    for (int w = 0; w < 5; ++w) {
        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        encode_metal_writers_slice(
            enc, ctx, 0, nworld, dt_s,
            b_idx, 0, b_hf, 0, bench_hf_samples, bench_hf_x_min, bench_hf_dx,
            b_cfg, 0, per_world_config, b_st, 0,
            b_qpos, 0, b_qvel, 0, b_xpos, 0, b_qfrc, 0, b_ctrl, 0, b_tendon, 0, 2,
            use_simd32
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
                enc, ctx, 0, nworld, dt_s,
                b_idx, 0, b_hf, 0, bench_hf_samples, bench_hf_x_min, bench_hf_dx,
                b_cfg, 0, per_world_config, b_st, 0,
                b_qpos, 0, b_qvel, 0, b_xpos, 0, b_qfrc, 0, b_ctrl, 0, b_tendon, 0, 2,
                use_simd32
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
            enc, ctx, 0, nworld, dt_s,
            b_idx, 0, b_hf, 0, bench_hf_samples, bench_hf_x_min, bench_hf_dx,
            b_cfg, 0, per_world_config, b_st, 0,
            b_qpos, 0, b_qvel, 0, b_xpos, 0, b_qfrc, 0, b_ctrl, 0, b_tendon, 0, 2,
            use_simd32
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

// Scenario 3: Hybrid Unified-Memory Batch Runner
// Runs `bike_gpu_writers_*_kernel` on the 40-core Apple Metal GPU in Unified Memory
// (`MTLResourceStorageModeShared`) overlapped with multi-threaded C++ CPU `mj_step` (FP64).
//
// Return codes (bike_metal_run_hybrid_batch forwards these unchanged):
//   0   success; all out_* arrays are written
//  -1   invalid arguments (xml_path, nworld, n_steps or d_configs)
//  -2   Metal context unavailable (bike_metal_init failed or a pipeline is missing)
//  -3   mj_loadXML failed for xml_path (the loader message is printed to stderr)
//  -4   model dimensions exceed the kMax* bounds
//  -5   initial state unavailable: init_qpos or init_qvel is null and captured_bootstrap.json next to
//       xml_path cannot be loaded (missing, unreadable, malformed, or nq/nv mismatch).
//  -6   heightfield asset failure: hfield_data.bin missing, wrong size or unreadable while the model has
//       a heightfield, or the 1D heightfield extraction fails when d_hf_heights is not supplied.
//  -7   GPU command buffer failed during the warmup or a step. The run is aborted; there is no CPU
//       fallback and the out_* arrays are not written.
//  -8   Metal buffer allocation failed before the simulation started; out_* arrays are not written.
// Initial state: a null init pointer selects the captured bootstrap state; caller arrays are used as given.
// Heights: d_hf_heights with hf_num_samples > 0 is used as given; otherwise the model heightfield is used.
// Every return path releases the mjModel it loaded.
int bike_metal_run_hybrid_sweep(
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
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.ok || !ctx.pso_1thread || !ctx.pso_simd32) {
        return -2;
    }

    std::array<char, 1024> err{};
    mjModel *m_base = mj_loadXML(xml_path, nullptr, err.data(), static_cast<int>(err.size()));
    if (m_base == nullptr) {
        std::cerr << "[bike_metal_run_hybrid_sweep] mj_loadXML failed: " << err.data() << '\n';
        return -3;
    }

    if (m_base->nq > bike::gpu::kMaxNq ||
        m_base->nv > bike::gpu::kMaxNv ||
        m_base->nu > bike::gpu::kMaxNu ||
        m_base->nbody > bike::gpu::kMaxNbody) {
        std::cerr << "[bike_metal_run_hybrid_sweep] Model dimensions (nq="
                  << static_cast<int>(m_base->nq) << ", nv=" << static_cast<int>(m_base->nv)
                  << ", nu=" << static_cast<int>(m_base->nu) << ", nbody="
                  << static_cast<int>(m_base->nbody) << ") exceed kMax bounds\n";
        mj_deleteModel(m_base);
        return -4;
    }

    // Model assets must be in place BEFORE cloning `worker_models`. hfield_data.bin is required whenever
    // the model has a heightfield (the CPU rollout collides with m_base->hfield_data). The 1D extraction
    // is needed only when the caller did not supply heights. Nothing else is allocated yet, so the
    // failure paths only release m_base.
    const assets::AssetStatus hfield_bin_status = assets::load_hfield_bin(m_base, xml_path);
    if (hfield_bin_status != assets::AssetStatus::kOk) {
        std::cerr << "[bike_metal_run_hybrid_sweep] hfield_data.bin load failed: "
                  << assets::asset_status_name(hfield_bin_status) << '\n';
        mj_deleteModel(m_base);
        return -6;
    }

    const bool caller_heights = (d_hf_heights != nullptr && hf_num_samples > 0);
    assets::Heightfield1D model_hf;
    if (!caller_heights) {
        const assets::AssetStatus hf_status = assets::extract_heightfield_1d(m_base, model_hf);
        if (hf_status != assets::AssetStatus::kOk) {
            std::cerr << "[bike_metal_run_hybrid_sweep] heightfield extraction failed: "
                      << assets::asset_status_name(hf_status) << '\n';
            mj_deleteModel(m_base);
            return -6;
        }
    }

    const float *resolved_hf_ptr = caller_heights ? d_hf_heights : model_hf.heights.data();
    const int resolved_hf_samples =
        caller_heights ? hf_num_samples : static_cast<int>(model_hf.heights.size());
    const float resolved_hf_x_min = caller_heights ? hf_x_min : model_hf.x_min_m;
    const float resolved_hf_dx = caller_heights ? hf_dx : model_hf.dx_m;

    const int hw_threads = static_cast<int>(std::thread::hardware_concurrency());
    int n_workers = (num_threads > 0) ? num_threads : (hw_threads > 0 ? hw_threads : 4);
    n_workers = std::max(1, std::min(n_workers, nworld));

    const bike::gpu::GpuModelIndices idx =
        (d_idx != nullptr) ? *d_idx : bike::gpu::GpuModelIndices{};
    // The kMax bounds above keep the mjtSize -> int narrowing lossless.
    const int nq = static_cast<int>(m_base->nq);
    const int nv = static_cast<int>(m_base->nv);
    const int nu = static_cast<int>(m_base->nu);
    const int nbody3 = 3 * static_cast<int>(m_base->nbody);
    const int ntendon = static_cast<int>(m_base->ntendon);
    const float dt_s = static_cast<float>(m_base->opt.timestep);
    const int n_cfg = per_world_config ? nworld : 1;
    const std::span<const bike::gpu::GpuWorldConfig> configs_v =
        boundary_span(d_configs, to_index(n_cfg));
    const bool use_simd32 = (configs_v[0].front_tire.num_lateral_ribs > 1) || (nworld < 64);

    // Initial state: if either pointer is null, the captured bootstrap state (captured_bootstrap.json
    // next to xml_path) supplies both vectors; non-null caller arrays then replace their vector.
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

    // Allocate `MTLResourceStorageModeShared` buffers. Model indices, heights and configs are copied in
    // once with newBufferWithBytes. The per-world state buffers are not copied: the CPU workers read and
    // write them in place through [buffer contents] while the GPU runs the writers on the same storage.
    id<MTLBuffer> b_idx = [ctx.device newBufferWithBytes:&idx
                                                  length:sizeof(bike::gpu::GpuModelIndices)
                                                 options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_hf = [ctx.device newBufferWithBytes:resolved_hf_ptr
                                                 length:static_cast<NSUInteger>(std::max(1, resolved_hf_samples)) * sizeof(float)
                                                options:MTLResourceStorageModeShared];
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
    id<MTLBuffer> b_qfrc = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * nv) * sizeof(float)
                                                   options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_ctrl = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * nu) * sizeof(float)
                                                   options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_tendon = [ctx.device newBufferWithLength:static_cast<NSUInteger>(nworld * 2) * sizeof(float)
                                                     options:MTLResourceStorageModeShared];
    if (!b_idx || !b_hf || !b_cfg || !b_st || !b_qpos || !b_qvel || !b_xpos || !b_qfrc || !b_ctrl || !b_tendon) {
        std::cerr << "[bike_metal_run_hybrid_sweep] Failed to allocate Metal buffers\n";
        mj_deleteModel(m_base);
        return -8;
    }

    // Each shared buffer's [contents] pointer is wrapped once into a typed span
    // through the boundary helper; all reads and writes below go through the spans.
    const std::size_t nworld_u = to_index(nworld);
    const std::span<bike::gpu::GpuWorldState> st_v =
        boundary_span(static_cast<bike::gpu::GpuWorldState *>([b_st contents]), nworld_u);
    const std::span<float> qpos_v =
        boundary_span(static_cast<float *>([b_qpos contents]), nworld_u * to_index(nq));
    const std::span<float> qvel_v =
        boundary_span(static_cast<float *>([b_qvel contents]), nworld_u * to_index(nv));
    const std::span<float> xpos_v =
        boundary_span(static_cast<float *>([b_xpos contents]), nworld_u * to_index(nbody3));
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

    // Parallel initialization of all `nworld` mjData instances and initial Unified Memory state
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
                // MuJoCo-owned per-world arrays, wrapped once through the boundary helper.
                const std::span<mjtNum> d_qpos = boundary_span(d_w->qpos, to_index(nq));
                const std::span<mjtNum> d_qvel = boundary_span(d_w->qvel, to_index(nv));
                const std::span<const mjtNum> d_xpos = boundary_span(d_w->xpos, to_index(nbody3));
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
                for (std::size_t i = 0; i < to_index(nq); ++i) qp[i] = static_cast<float>(d_qpos[i]);
                for (std::size_t i = 0; i < to_index(nv); ++i) qv[i] = static_cast<float>(d_qvel[i]);
                for (std::size_t i = 0; i < to_index(nbody3); ++i) xp[i] = static_cast<float>(d_xpos[i]);

                if (m_base->ntendon > 0 && idx.freehub_tendon >= 0 && idx.freehub_tendon < m_base->ntendon) {
                    const std::span<const mjtNum> base_tendon_range =
                        boundary_span(m_base->tendon_range, 2 * to_index(ntendon));
                    const std::size_t ft = 2 * to_index(idx.freehub_tendon);
                    const std::size_t tw = 2 * wu;
                    tendon_v[tw + 0] = static_cast<float>(base_tendon_range[ft + 0]);
                    tendon_v[tw + 1] = static_cast<float>(base_tendon_range[ft + 1]);
                }

                const bike::gpu::GpuWorldConfig &cfg_w = configs_v[to_index(per_world_config ? w : 0)];
                const float crank0 =
                    (idx.crank_spin_qpos >= 0 && idx.crank_spin_qpos < nq) ? qp[to_index(idx.crank_spin_qpos)] : 0.0f;
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
            b_idx, 0, b_hf, 0, resolved_hf_samples, resolved_hf_x_min, resolved_hf_dx,
            b_cfg, 0, per_world_config, b_st, 0,
            b_qpos, 0, b_qvel, 0, b_xpos, 0, b_qfrc, 0, b_ctrl, 0, b_tendon, 0, 2,
            use_simd32
        );
        [enc endEncoding];
        [cb commit];
        return cb;
    };

    bool gpu_ok = true;
    // Warmup GPU pipeline once before starting clock and reset states[w]
    {
        id<MTLCommandBuffer> cb = dispatch_gpu_slice_async(0, nworld);
        if (!wait_command_buffer_ok(cb, "bike_metal_run_hybrid_sweep")) {
            gpu_ok = false;
        }
        for (int w = 0; w < nworld; ++w) {
            const bike::gpu::GpuWorldConfig &cfg_w = configs_v[to_index(per_world_config ? w : 0)];
            const std::span<const float> qp =
                qpos_v.subspan(to_index(w) * to_index(nq), to_index(nq));
            const float crank0 =
                (idx.crank_spin_qpos >= 0 && idx.crank_spin_qpos < nq) ? qp[to_index(idx.crank_spin_qpos)] : 0.0f;
            const float rwheel0 =
                (idx.rear_wheel_spin_qpos >= 0 && idx.rear_wheel_spin_qpos < nq) ? qp[to_index(idx.rear_wheel_spin_qpos)] : 0.0f;
            bike::gpu::gpu_init_world_state(st_v[to_index(w)], cfg_w.drivetrain, crank0, rwheel0);
        }
    }

    // Persistent C++20 atomic wait/notify thread pool for zero-allocation per-phase CPU mj_step execution
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
        // MuJoCo-owned per-world arrays, wrapped once through the boundary helper.
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

        const std::span<float> qp = qpos_v.subspan(wu * to_index(nq), to_index(nq));
        const std::span<float> qv = qvel_v.subspan(wu * to_index(nv), to_index(nv));
        const std::span<float> xp = xpos_v.subspan(wu * to_index(nbody3), to_index(nbody3));
        const std::span<const mjtNum> d_qpos = boundary_span(d_t->qpos, to_index(nq));
        const std::span<const mjtNum> d_qvel = boundary_span(d_t->qvel, to_index(nv));
        const std::span<const mjtNum> d_xpos = boundary_span(d_t->xpos, to_index(nbody3));
        for (std::size_t i = 0; i < to_index(nq); ++i) qp[i] = static_cast<float>(d_qpos[i]);
        for (std::size_t i = 0; i < to_index(nv); ++i) qv[i] = static_cast<float>(d_qvel[i]);
        for (std::size_t i = 0; i < to_index(nbody3); ++i) xp[i] = static_cast<float>(d_xpos[i]);

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
        // Double-buffered Unified Memory pipeline:
        // Half 0 = [0, half), Half 1 = [half, nworld).
        // GPU computes force writers for Half 1 while the CPU workers run mj_step for Half 0, and vice versa.
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

    // Outputs are written only when every GPU command buffer completed. After a GPU failure the CPU
    // state is partial, so it is not reported.
    if (gpu_ok) {
        // The three optional float* outputs, each an nworld-sized range or empty when
        // the caller passed nullptr; empty() replaces the null check at the write sites.
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
    const float *d_hf_heights,
    int hf_num_samples,
    float hf_x_min,
    float hf_dx,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    float *d_qfrc_applied,
    float *d_ctrl,
    float *d_tendon_range,
    int tendon_range_world_stride
) {
    return bike_metal_launch_writers(
        nworld, dt_s, d_idx, d_hf_heights, hf_num_samples, hf_x_min, hf_dx,
        d_configs, per_world_config, d_states, d_qpos, d_qvel, d_xpos,
        d_qfrc_applied, d_ctrl, d_tendon_range, tendon_range_world_stride
    );
}

// Same contract and return codes as bike_metal_run_hybrid_sweep.
int bike_metal_run_hybrid_batch(
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
    return bike_metal_run_hybrid_sweep(
        xml_path, nworld, n_steps, num_threads, d_idx, d_hf_heights,
        hf_num_samples, hf_x_min, hf_dx, d_configs, per_world_config,
        init_qpos, init_qvel, out_peak_fork_mm, out_peak_shock_mm,
        out_final_root_x_m, out_all_finite, out_elapsed_ms
    );
}

} // extern "C"
