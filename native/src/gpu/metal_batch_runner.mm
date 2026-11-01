// native/src/gpu/metal_batch_runner.mm — Apple Metal Unified-Memory GPU Force
// Writers & Hybrid (Metal GPU Writers FP32 + Multi-Threaded C++ CPU mj_step FP64)
// Batch Runner for Apple Silicon (M4 Max).

#include "cuda_kernels.cu"

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

std::string read_text_file(const std::string &path) {
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

void replace_all_str(std::string &s, const std::string &from, const std::string &to) {
    std::size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) {
        s.replace(pos, from.length(), to);
        pos += to.length();
    }
}

std::string build_metal_shader_source(const std::string &gpu_src_dir) {
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
#include <metal_stdlib>
#include <metal_simdgroup>
using namespace metal;
#define BIKE_HD inline
#define BIKE_DEVICE inline
#define BIKE_GLOBAL kernel
inline float bike_msl_pow(float base, float exp) {
    if (base <= 0.0f) return (exp == 0.0f) ? 1.0f : 0.0f;
    return metal::pow(base, exp);
}
inline float bike_msl_tanh(float v) {
    const float c = v < -12.0f ? -12.0f : (v > 12.0f ? 12.0f : v);
    return metal::tanh(c);
}
)MSL";

    for (const auto &fn : headers) {
        std::string content = read_text_file(base + "/" + fn);
        replace_all_str(content, "#pragma once", "");
        replace_all_str(content, "#include <cmath>", "");
        replace_all_str(content, "#include <cstdint>", "");
        replace_all_str(content, "#include \"common.hpp\"", "");
        replace_all_str(content, "#include \"drivetrain_resistance_kernel.hpp\"", "");
        replace_all_str(content, "#include \"rider_kernel.hpp\"", "");
        replace_all_str(content, "#include \"suspension_kernel.hpp\"", "");
        replace_all_str(content, "#include \"tire_kernel.hpp\"", "");
        replace_all_str(content, "inline constexpr", "constant constexpr");
        replace_all_str(content, "::fabsf(", "metal::fabs(");
        replace_all_str(content, "::sqrtf(", "metal::sqrt(");
        replace_all_str(content, "::powf(", "bike_msl_pow(");
        replace_all_str(content, "::tanhf(", "bike_msl_tanh(");
        replace_all_str(content, "::sinf(", "metal::sin(");
        replace_all_str(content, "::cosf(", "metal::cos(");
        replace_all_str(content, "::atan2f(", "metal::atan2(");
        replace_all_str(content, "std::isfinite(", "metal::isfinite(");
        replace_all_str(content, " noexcept", "");
        replace_all_str(content, "[[maybe_unused]] ", "");
        replace_all_str(
            content,
            "const float *heights_m = nullptr;",
            "__BIKE_DEV_FLOAT_PTR__ heights_m = nullptr;"
        );
        replace_all_str(content, "const Gpu", "thread const Gpu");
        replace_all_str(content, "GpuSuspensionParams &", "thread GpuSuspensionParams &");
        replace_all_str(content, "GpuWheelStationState *", "thread GpuWheelStationState *");
        replace_all_str(content, "GpuWheelContactOutput *", "thread GpuWheelContactOutput *");
        replace_all_str(content, "GpuDrivetrainState &", "thread GpuDrivetrainState &");
        replace_all_str(content, "GpuWorldConfig &", "thread GpuWorldConfig &");
        replace_all_str(content, "GpuWorldConfig *", "thread GpuWorldConfig *");
        replace_all_str(content, "GpuWorldState &", "thread GpuWorldState &");
        replace_all_str(content, "GpuWorldState *", "thread GpuWorldState *");
        replace_all_str(content, "const float *", "thread const float *");
        replace_all_str(content, "    float *", "    thread float *");
        replace_all_str(content, "__BIKE_DEV_FLOAT_PTR__", "device const float *");
        combined += "\n// === " + fn + " ===\n" + content;
    }

    combined += R"MSL(
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
    const int local_w = static_cast<int>(gid);
    if (local_w >= world_count) {
        return;
    }
    const int w = world_offset + local_w;
    bike::gpu::GpuModelIndices idx = idx_const;
    const float inv_dx = hf_dx > 0.0f ? (1.0f / hf_dx) : 1.0f;
    bike::gpu::GpuHeightfield1D hf{hf_heights, hf_num_samples, hf_x_min, hf_dx, inv_dx};

    bike::gpu::GpuWorldConfig cfg = configs[per_world_config ? w : 0];
    bike::gpu::GpuWorldState st = states[w];

    float qpos_w[bike::gpu::kMaxNq];
    float qvel_w[bike::gpu::kMaxNv];
    float xpos_w[128];
    float qfrc_w[bike::gpu::kMaxNv];
    float ctrl_w[bike::gpu::kMaxNu];
    float tendon_w[2] = {0.0f, 0.0f};

    device const float *qpos_src = qpos_batch + w * idx.nq;
    device const float *qvel_src = qvel_batch + w * idx.nv;
    device const float *xpos_src = xpos_batch + w * (idx.nbody * 3);
    for (int i = 0; i < idx.nq; ++i) qpos_w[i] = qpos_src[i];
    for (int i = 0; i < idx.nv; ++i) qvel_w[i] = qvel_src[i];
    const int b0 = 3 * idx.frame_bid;
    const int b1 = 3 * idx.front_wheel_bid;
    const int b2 = 3 * idx.rear_wheel_bid;
    xpos_w[b0 + 0] = xpos_src[b0 + 0];
    xpos_w[b0 + 1] = xpos_src[b0 + 1];
    xpos_w[b0 + 2] = xpos_src[b0 + 2];
    xpos_w[b1 + 0] = xpos_src[b1 + 0];
    xpos_w[b1 + 1] = xpos_src[b1 + 1];
    xpos_w[b1 + 2] = xpos_src[b1 + 2];
    xpos_w[b2 + 0] = xpos_src[b2 + 0];
    xpos_w[b2 + 1] = xpos_src[b2 + 1];
    xpos_w[b2 + 2] = xpos_src[b2 + 2];

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
        tendon_w,
        2
    );

    device float *qfrc_dst = qfrc_batch + w * idx.nv;
    device float *ctrl_dst = ctrl_batch + w * idx.nu;
    for (int i = 0; i < idx.nv; ++i) qfrc_dst[i] = qfrc_w[i];
    for (int i = 0; i < idx.nu; ++i) ctrl_dst[i] = ctrl_w[i];
    states[w] = st;

    if (tendon_range_world_stride > 0) {
        tendon_range_batch[w * tendon_range_world_stride + 1] = tendon_w[1];
    } else if (w == 0) {
        tendon_range_batch[1] = tendon_w[1];
    }
}

inline bike::gpu::GpuWheelContactOutput gpu_evaluate_wheel_contact_simd32(
    float wheel_x_m,
    float wheel_z_m,
    float wheel_vx_mps,
    float wheel_vz_mps,
    float wheel_omega_rad_s,
    float dt_s,
    thread const bike::gpu::GpuTireParams &params,
    thread const bike::gpu::GpuHeightfield1D &hf,
    device bike::gpu::GpuWheelStationState *station_state,
    uint lane
) {
    using namespace bike::gpu;
    GpuWheelContactOutput out{};
    const int n_stations = params.num_stations > 0 ? params.num_stations : 32;
    const int n_ribs = params.num_lateral_ribs > 0 ? params.num_lateral_ribs : 1;
    const float station_weight = kTwoPi / static_cast<float>(n_stations);
    const float rib_weight = station_weight / static_cast<float>(n_ribs);
    const float safe_dt = gpu_max(dt_s, 0.0f);
    const float sigma = gpu_max(params.relaxation_length_m, 1e-4f);

    float fx_sum = 0.0f;
    float fz_sum = 0.0f;
    float tau_sum = 0.0f;
    float load_sum = 0.0f;
    int active_sum = 0;
    float weighted_drop_terms = 0.0f;
    float weighted_drop_weights = 0.0f;
    float unweighted_drop_terms = 0.0f;
    int unweighted_drop_count = 0;

    const int total_rays = n_stations * n_ribs;
    for (int ray_idx = static_cast<int>(lane); ray_idx < total_rays; ray_idx += 32) {
        const int s = ray_idx / n_ribs;
        const int rib = ray_idx - s * n_ribs;

        const float theta = kTwoPi * static_cast<float>(s) / static_cast<float>(n_stations);
        const float cos_theta = gpu_cos(theta);
        const float sin_theta = gpu_sin(theta);

        float r_rib = params.nominal_radius_m;
        if (n_ribs > 1) {
            const float u_lat = (2.0f * static_cast<float>(rib) /
                                 static_cast<float>(n_ribs - 1)) - 1.0f;
            const float psi = u_lat * kMaxRibCrownAngleRad;
            r_rib = params.nominal_radius_m -
                    params.section_radius_m * (1.0f - gpu_cos(psi));
        }

        const float rx = r_rib * cos_theta;
        const float rz = r_rib * sin_theta;
        const float pt_x = wheel_x_m + rx;
        const float pt_z = wheel_z_m + rz;

        const GpuVec3 surf = gpu_sample_heightfield_1d(hf, pt_x);
        const float terrain_z = surf.x;
        const float nx = surf.y;
        const float nz = surf.z;

        const float delta_m = (terrain_z - pt_z) * nz;
        if (delta_m <= 0.0f) {
            if (n_ribs == 1 && s < kMaxTreadStations && station_state != nullptr) {
                station_state->bristle_x[s] = 0.0f;
            }
            continue;
        }

        const float vx_pt = wheel_vx_mps + wheel_omega_rad_s * rz;
        const float vz_pt = wheel_vz_mps - wheel_omega_rad_s * rx;

        const float rate_mps = -(vx_pt * nx + vz_pt * nz);
        const GpuVec2 resp = gpu_station_response(
            delta_m, rate_mps, rib_weight, params.density);
        const float normal_force = resp.x;

        const float tx = nz;
        const float tz = -nx;
        const float v_slip = vx_pt * tx + vz_pt * tz;
        const float v_trans = gpu_abs(wheel_vx_mps * tx + wheel_vz_mps * tz);

        float xi_prev = 0.0f;
        if (n_ribs == 1 && s < kMaxTreadStations && station_state != nullptr) {
            xi_prev = station_state->bristle_x[s];
        }

        const float decay = 1.0f / (1.0f + (v_trans * safe_dt) / sigma);
        float xi_new = (xi_prev - v_slip * safe_dt) * decay;

        const float k_station = params.bristle_kx_n_m * rib_weight;
        const float c_station = params.bristle_cx_ns_m * rib_weight;
        const float ft_raw = k_station * xi_new - c_station * v_slip;

        const float slip_blend = gpu_min(gpu_abs(v_slip) / 0.5f, 1.0f);
        const float mu_eff = params.mu_kinetic +
            (params.mu_static - params.mu_kinetic) * (1.0f - slip_blend);
        const float max_ft = gpu_max(0.0f, mu_eff * normal_force);
        const float tangent_force = gpu_clamp(ft_raw, -max_ft, max_ft);

        if (k_station > 1e-8f) {
            const float max_xi = max_ft / k_station;
            xi_new = gpu_clamp(xi_new, -max_xi, max_xi);
        }
        if (n_ribs == 1 && s < kMaxTreadStations && station_state != nullptr) {
            station_state->bristle_x[s] = xi_new;
        }

        const float fx_station = normal_force * nx + tangent_force * tx;
        const float fz_station = normal_force * nz + tangent_force * tz;

        fx_sum += fx_station;
        fz_sum += fz_station;
        tau_sum += rz * fx_station - rx * fz_station;
        load_sum += normal_force;
        active_sum += 1;

        const float drop = -(rx * nx + rz * nz);
        if (normal_force > 0.0f) {
            weighted_drop_terms += normal_force * drop;
            weighted_drop_weights += normal_force;
        }
        unweighted_drop_terms += drop;
        unweighted_drop_count += 1;
    }

    out.fx_world_n = simd_sum(fx_sum);
    out.fz_world_n = simd_sum(fz_sum);
    out.spin_torque_nm = simd_sum(tau_sum);
    out.normal_load_n = simd_sum(load_sum);
    out.active_stations = simd_sum(active_sum);
    const float tot_w_drop = simd_sum(weighted_drop_terms);
    const float tot_w_weight = simd_sum(weighted_drop_weights);
    const float tot_u_drop = simd_sum(unweighted_drop_terms);
    const int tot_u_cnt = simd_sum(unweighted_drop_count);

    if (out.active_stations > 0) {
        if (tot_w_weight > 0.0f) {
            out.effective_radius_m = gpu_max(0.0f, tot_w_drop / tot_w_weight);
        } else if (tot_u_cnt > 0) {
            out.effective_radius_m = gpu_max(0.0f, tot_u_drop / static_cast<float>(tot_u_cnt));
        }
    }
    return out;
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
    const int local_w = static_cast<int>(gid >> 5);
    if (local_w >= world_count) {
        return;
    }
    const int w = world_offset + local_w;
    bike::gpu::GpuModelIndices idx = idx_const;
    const float inv_dx = hf_dx > 0.0f ? (1.0f / hf_dx) : 1.0f;
    bike::gpu::GpuHeightfield1D hf{hf_heights, hf_num_samples, hf_x_min, hf_dx, inv_dx};

    bike::gpu::GpuWorldConfig cfg = configs[per_world_config ? w : 0];
    device const float *qpos_w = qpos_batch + w * idx.nq;
    device const float *qvel_w = qvel_batch + w * idx.nv;
    device const float *xpos_w = xpos_batch + w * (idx.nbody * 3);

    const float com_x = xpos_w[3 * idx.frame_bid + 0];
    const float com_z = xpos_w[3 * idx.frame_bid + 2];
    const float front_x = xpos_w[3 * idx.front_wheel_bid + 0];
    const float front_z = xpos_w[3 * idx.front_wheel_bid + 2];
    const float rear_x = xpos_w[3 * idx.rear_wheel_bid + 0];
    const float rear_z = xpos_w[3 * idx.rear_wheel_bid + 2];

    const float root_vx = qvel_w[idx.root_x_dof];
    const float root_vz = qvel_w[idx.root_z_dof];
    const float pitch_rate = qvel_w[idx.root_pitch_dof];

    const float front_vx = root_vx + pitch_rate * (front_z - com_z);
    const float front_vz = root_vz - pitch_rate * (front_x - com_x);
    const float front_omega = pitch_rate + qvel_w[idx.front_wheel_spin_dof];

    const float rear_vx = root_vx + pitch_rate * (rear_z - com_z);
    const float rear_vz = root_vz - pitch_rate * (rear_x - com_x);
    const float rear_omega = pitch_rate + qvel_w[idx.rear_wheel_spin_dof];

    const bike::gpu::GpuWheelContactOutput front_out = gpu_evaluate_wheel_contact_simd32(
        front_x, front_z, front_vx, front_vz, front_omega, dt_s, cfg.front_tire, hf, &states[w].front_wheel, lane
    );
    const bike::gpu::GpuWheelContactOutput rear_out = gpu_evaluate_wheel_contact_simd32(
        rear_x, rear_z, rear_vx, rear_vz, rear_omega, dt_s, cfg.rear_tire, hf, &states[w].rear_wheel, lane
    );

    if (lane == 0) {
        float qpos_loc[bike::gpu::kMaxNq];
        float qvel_loc[bike::gpu::kMaxNv];
        float qfrc_loc[bike::gpu::kMaxNv];
        float ctrl_loc[bike::gpu::kMaxNu];
        for (int i = 0; i < idx.nq; ++i) qpos_loc[i] = qpos_w[i];
        for (int i = 0; i < idx.nv; ++i) { qvel_loc[i] = qvel_w[i]; qfrc_loc[i] = 0.0f; }
        for (int i = 0; i < idx.nu; ++i) ctrl_loc[i] = 0.0f;

        bike::gpu::gpu_write_suspension_qfrc(cfg.suspension, idx, qpos_loc, qvel_loc, qfrc_loc);

        bike::gpu::project_planar_force_to_root(
            front_out.fx_world_n, front_out.fz_world_n, front_x, front_z, com_x, com_z, idx, qfrc_loc
        );
        bike::gpu::project_planar_force_to_root(
            rear_out.fx_world_n, rear_out.fz_world_n, rear_x, rear_z, com_x, com_z, idx, qfrc_loc
        );
        qfrc_loc[idx.front_wheel_spin_dof] += front_out.spin_torque_nm;
        qfrc_loc[idx.rear_wheel_spin_dof] += rear_out.spin_torque_nm;

        bike::gpu::GpuDrivetrainState dt_st = states[w].drivetrain;
        const bike::gpu::GpuDrivetrainOutput dt_out = bike::gpu::gpu_write_drivetrain_and_resistance_qfrc(
            cfg.resistance, cfg.drivetrain, idx, dt_st, qpos_loc, qvel_loc,
            front_out.normal_load_n, rear_out.normal_load_n,
            front_out.effective_radius_m, rear_out.effective_radius_m,
            cfg.pedal_demand, cfg.front_brake_demand, cfg.rear_brake_demand, qfrc_loc
        );
        states[w].drivetrain = dt_st;

        bike::gpu::gpu_write_rider_qfrc_and_ctrl(cfg.rider, idx, qpos_loc, qvel_loc, qfrc_loc, ctrl_loc);

        device float *qfrc_dst = qfrc_batch + w * idx.nv;
        device float *ctrl_dst = ctrl_batch + w * idx.nu;
        for (int i = 0; i < idx.nv; ++i) qfrc_dst[i] = qfrc_loc[i];
        for (int i = 0; i < idx.nu; ++i) ctrl_dst[i] = ctrl_loc[i];

        if (tendon_range_world_stride > 0) {
            tendon_range_batch[w * tendon_range_world_stride + 1] = dt_out.tendon_upper_bound_rad;
        } else if (w == 0) {
            tendon_range_batch[1] = dt_out.tendon_upper_bound_rad;
        }
    }
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
    const int w = static_cast<int>(gid);
    if (w >= args->nworld) {
        return;
    }
    bike::gpu::GpuModelIndices idx = *(args->idx);
    bike::gpu::GpuWorldConfig cfg = args->configs[args->per_world_config ? w : 0];
    bike::gpu::GpuWorldState st{};
    device const float *qpos_src = args->qpos + w * idx.nq;
    bike::gpu::gpu_init_world_state(
        st,
        cfg.drivetrain,
        qpos_src[idx.crank_spin_qpos],
        qpos_src[idx.rear_wheel_spin_qpos]
    );
    args->states[w] = st;
}

kernel void bike_wp_writers_metal_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpWritersArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    const int w = static_cast<int>(gid);
    if (w >= args->nworld) {
        return;
    }
    bike::gpu::GpuModelIndices idx = *(args->idx);
    const float inv_dx = args->hf_dx > 0.0f ? (1.0f / args->hf_dx) : 1.0f;
    bike::gpu::GpuHeightfield1D hf{args->hf_heights, args->hf_num_samples, args->hf_x_min, args->hf_dx, inv_dx};

    bike::gpu::GpuWorldConfig cfg = args->configs[args->per_world_config ? w : 0];
    bike::gpu::GpuWorldState st = args->states[w];

    float qpos_w[bike::gpu::kMaxNq];
    float qvel_w[bike::gpu::kMaxNv];
    float xpos_w[128];
    float qfrc_w[bike::gpu::kMaxNv];
    float ctrl_w[bike::gpu::kMaxNu];
    float tendon_w[2] = {0.0f, 0.0f};

    device const float *qpos_src = args->qpos + w * idx.nq;
    device const float *qvel_src = args->qvel + w * idx.nv;
    device const float *xpos_src = args->xpos + w * (idx.nbody * 3);
    for (int i = 0; i < idx.nq; ++i) qpos_w[i] = qpos_src[i];
    for (int i = 0; i < idx.nv; ++i) qvel_w[i] = qvel_src[i];
    const int b0 = 3 * idx.frame_bid;
    const int b1 = 3 * idx.front_wheel_bid;
    const int b2 = 3 * idx.rear_wheel_bid;
    xpos_w[b0 + 0] = xpos_src[b0 + 0];
    xpos_w[b0 + 1] = xpos_src[b0 + 1];
    xpos_w[b0 + 2] = xpos_src[b0 + 2];
    xpos_w[b1 + 0] = xpos_src[b1 + 0];
    xpos_w[b1 + 1] = xpos_src[b1 + 1];
    xpos_w[b1 + 2] = xpos_src[b1 + 2];
    xpos_w[b2 + 0] = xpos_src[b2 + 0];
    xpos_w[b2 + 1] = xpos_src[b2 + 1];
    xpos_w[b2 + 2] = xpos_src[b2 + 2];

    bike::gpu::gpu_step_world_writers(
        0, args->dt_s, idx, hf, &cfg, 0, &st,
        qpos_w, qvel_w, xpos_w, qfrc_w, ctrl_w, tendon_w, 2
    );

    device float *qfrc_dst = args->qfrc_applied + w * idx.nv;
    device float *ctrl_dst = args->ctrl + w * idx.nu;
    for (int i = 0; i < idx.nv; ++i) qfrc_dst[i] = qfrc_w[i];
    for (int i = 0; i < idx.nu; ++i) ctrl_dst[i] = ctrl_w[i];
    args->states[w] = st;

    if (args->tendon_range != nullptr) {
        if (args->tendon_range_world_stride > 0) {
            args->tendon_range[w * args->tendon_range_world_stride + 1] = tendon_w[1];
        } else if (w == 0) {
            args->tendon_range[1] = tendon_w[1];
        }
    }
}

kernel void bike_wp_writers_simd32_kernel(
    constant ulong *bounds_buf [[buffer(0)]],
    constant BikeWpWritersArgs *args [[buffer(1)]],
    uint gid [[thread_position_in_grid]],
    uint lane [[thread_index_in_simdgroup]]
) {
    const int w = static_cast<int>(gid >> 5);
    if (w >= args->nworld) {
        return;
    }
    bike::gpu::GpuModelIndices idx = *(args->idx);
    const float inv_dx = args->hf_dx > 0.0f ? (1.0f / args->hf_dx) : 1.0f;
    bike::gpu::GpuHeightfield1D hf{args->hf_heights, args->hf_num_samples, args->hf_x_min, args->hf_dx, inv_dx};

    bike::gpu::GpuWorldConfig cfg = args->configs[args->per_world_config ? w : 0];
    device const float *qpos_w = args->qpos + w * idx.nq;
    device const float *qvel_w = args->qvel + w * idx.nv;
    device const float *xpos_w = args->xpos + w * (idx.nbody * 3);

    const float com_x = xpos_w[3 * idx.frame_bid + 0];
    const float com_z = xpos_w[3 * idx.frame_bid + 2];
    const float front_x = xpos_w[3 * idx.front_wheel_bid + 0];
    const float front_z = xpos_w[3 * idx.front_wheel_bid + 2];
    const float rear_x = xpos_w[3 * idx.rear_wheel_bid + 0];
    const float rear_z = xpos_w[3 * idx.rear_wheel_bid + 2];

    const float root_vx = qvel_w[idx.root_x_dof];
    const float root_vz = qvel_w[idx.root_z_dof];
    const float pitch_rate = qvel_w[idx.root_pitch_dof];

    const float front_vx = root_vx + pitch_rate * (front_z - com_z);
    const float front_vz = root_vz - pitch_rate * (front_x - com_x);
    const float front_omega = pitch_rate + qvel_w[idx.front_wheel_spin_dof];

    const float rear_vx = root_vx + pitch_rate * (rear_z - com_z);
    const float rear_vz = root_vz - pitch_rate * (rear_x - com_x);
    const float rear_omega = pitch_rate + qvel_w[idx.rear_wheel_spin_dof];

    const bike::gpu::GpuWheelContactOutput front_out = gpu_evaluate_wheel_contact_simd32(
        front_x, front_z, front_vx, front_vz, front_omega, args->dt_s, cfg.front_tire, hf, &args->states[w].front_wheel, lane
    );
    const bike::gpu::GpuWheelContactOutput rear_out = gpu_evaluate_wheel_contact_simd32(
        rear_x, rear_z, rear_vx, rear_vz, rear_omega, args->dt_s, cfg.rear_tire, hf, &args->states[w].rear_wheel, lane
    );

    if (lane == 0) {
        float qpos_loc[bike::gpu::kMaxNq];
        float qvel_loc[bike::gpu::kMaxNv];
        float qfrc_loc[bike::gpu::kMaxNv];
        float ctrl_loc[bike::gpu::kMaxNu];
        for (int i = 0; i < idx.nq; ++i) qpos_loc[i] = qpos_w[i];
        for (int i = 0; i < idx.nv; ++i) { qvel_loc[i] = qvel_w[i]; qfrc_loc[i] = 0.0f; }
        for (int i = 0; i < idx.nu; ++i) ctrl_loc[i] = 0.0f;

        bike::gpu::gpu_write_suspension_qfrc(cfg.suspension, idx, qpos_loc, qvel_loc, qfrc_loc);

        bike::gpu::project_planar_force_to_root(
            front_out.fx_world_n, front_out.fz_world_n, front_x, front_z, com_x, com_z, idx, qfrc_loc
        );
        bike::gpu::project_planar_force_to_root(
            rear_out.fx_world_n, rear_out.fz_world_n, rear_x, rear_z, com_x, com_z, idx, qfrc_loc
        );
        qfrc_loc[idx.front_wheel_spin_dof] += front_out.spin_torque_nm;
        qfrc_loc[idx.rear_wheel_spin_dof] += rear_out.spin_torque_nm;

        bike::gpu::GpuDrivetrainState dt_st = args->states[w].drivetrain;
        const bike::gpu::GpuDrivetrainOutput dt_out = bike::gpu::gpu_write_drivetrain_and_resistance_qfrc(
            cfg.resistance, cfg.drivetrain, idx, dt_st, qpos_loc, qvel_loc,
            front_out.normal_load_n, rear_out.normal_load_n,
            front_out.effective_radius_m, rear_out.effective_radius_m,
            cfg.pedal_demand, cfg.front_brake_demand, cfg.rear_brake_demand, qfrc_loc
        );
        args->states[w].drivetrain = dt_st;

        bike::gpu::gpu_write_rider_qfrc_and_ctrl(cfg.rider, idx, qpos_loc, qvel_loc, qfrc_loc, ctrl_loc);

        device float *qfrc_dst = args->qfrc_applied + w * idx.nv;
        device float *ctrl_dst = args->ctrl + w * idx.nu;
        for (int i = 0; i < idx.nv; ++i) qfrc_dst[i] = qfrc_loc[i];
        for (int i = 0; i < idx.nu; ++i) ctrl_dst[i] = ctrl_loc[i];

        if (args->tendon_range != nullptr) {
            if (args->tendon_range_world_stride > 0) {
                args->tendon_range[w * args->tendon_range_world_stride + 1] = dt_out.tendon_upper_bound_rad;
            } else if (w == 0) {
                args->tendon_range[1] = dt_out.tendon_upper_bound_rad;
            }
        }
    }
}
)MSL";
    return combined;
}

struct CachedBuffer {
    uintptr_t base = 0;
    uintptr_t total_len = 0;
    id<MTLBuffer> buffer = nil;
};

struct MetalWritersContext {
    id<MTLDevice> device = nil;
    id<MTLCommandQueue> queue = nil;
    id<MTLLibrary> library = nil;
    id<MTLComputePipelineState> pso_1thread = nil;
    id<MTLComputePipelineState> pso_simd32 = nil;
    id<MTLBuffer> dummy_buf = nil;
    std::vector<CachedBuffer> cache;

    bool init(const std::string &gpu_src_dir) {
        if (pso_1thread != nil && pso_simd32 != nil) {
            return true;
        }
        device = MTLCreateSystemDefaultDevice();
        if (!device) {
            std::fprintf(stderr, "[MetalWritersContext] MTLCreateSystemDefaultDevice failed\n");
            return false;
        }
        queue = [device newCommandQueue];
        std::string src = build_metal_shader_source(gpu_src_dir);
        NSError *err = nil;
        library = [device newLibraryWithSource:[NSString stringWithUTF8String:src.c_str()]
                                       options:nil
                                         error:&err];
        if (!library) {
            std::fprintf(
                stderr,
                "[MetalWritersContext] MSL compile error:\n%s\n",
                [[err localizedDescription] UTF8String]
            );
            return false;
        }
        id<MTLFunction> fn1 = [library newFunctionWithName:@"bike_gpu_writers_metal_kernel"];
        id<MTLFunction> fn32 = [library newFunctionWithName:@"bike_gpu_writers_simd32_kernel"];
        pso_1thread = [device newComputePipelineStateWithFunction:fn1 error:&err];
        pso_simd32 = [device newComputePipelineStateWithFunction:fn32 error:&err];
        if (!pso_1thread || !pso_simd32) {
            std::fprintf(
                stderr,
                "[MetalWritersContext] PSO error:\n%s\n",
                [[err localizedDescription] UTF8String]
            );
            return false;
        }
        dummy_buf = [device newBufferWithLength:64 options:MTLResourceStorageModeShared];
        return true;
    }

    id<MTLBuffer> wrap_ptr(const void *ptr, std::size_t byte_len, NSUInteger *out_offset) {
        if (ptr == nullptr || byte_len == 0) {
            *out_offset = 0;
            return dummy_buf;
        }
        constexpr uintptr_t kPageSize = 16384ULL;
        const uintptr_t raw = reinterpret_cast<uintptr_t>(ptr);
        const uintptr_t base = raw & ~(kPageSize - 1ULL);
        const uintptr_t offset = raw - base;
        const uintptr_t total = (offset + byte_len + kPageSize - 1ULL) & ~(kPageSize - 1ULL);
        *out_offset = static_cast<NSUInteger>(offset);

        for (auto &entry : cache) {
            if (entry.base == base && entry.total_len >= total) {
                return entry.buffer;
            }
        }
        id<MTLBuffer> buf = [device newBufferWithBytesNoCopy:reinterpret_cast<void *>(base)
                                                      length:static_cast<NSUInteger>(total)
                                                     options:MTLResourceStorageModeShared
                                                 deallocator:nil];
        if (!buf) {
            return dummy_buf;
        }
        cache.push_back(CachedBuffer{base, total, buf});
        return buf;
    }
};

MetalWritersContext &get_metal_ctx(const char *gpu_src_dir_opt = nullptr) {
    static MetalWritersContext ctx;
    static std::once_flag flag;
    std::string dir_str;
    if (gpu_src_dir_opt != nullptr && gpu_src_dir_opt[0] != '\0') {
        dir_str = gpu_src_dir_opt;
    } else {
        std::string file_path = __FILE__;
        std::size_t slash = file_path.find_last_of("/\\");
        dir_str = (slash != std::string::npos) ? file_path.substr(0, slash) : ".";
    }
    std::call_once(flag, [&]() { ctx.init(dir_str); });
    return ctx;
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
    return (ctx.pso_1thread != nil && ctx.pso_simd32 != nil) ? 0 : -1;
}

const char *bike_metal_device_name() {
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.device) {
        return "None";
    }
    static std::string name = [[ctx.device name] UTF8String];
    return name.c_str();
}

const char *bike_metal_get_shader_source(const char *gpu_src_dir) {
    std::string dir_str;
    if (gpu_src_dir != nullptr && gpu_src_dir[0] != '\0') {
        dir_str = gpu_src_dir;
    } else {
        std::string file_path = __FILE__;
        std::size_t slash = file_path.find_last_of("/\\");
        dir_str = (slash != std::string::npos) ? file_path.substr(0, slash) : ".";
    }
    static std::string cached_src;
    cached_src = build_metal_shader_source(dir_str);
    return cached_src.c_str();
}

// Initializes per-world states directly in Unified Memory (for Scenario 2 warp-metal).
void bike_metal_launch_init_states(
    int nworld,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos
) {
    bike_gpu_launch_init_states(
        nullptr, nworld, d_idx, d_configs, per_world_config, d_states, d_qpos
    );
}

// Dispatches `bike_gpu_writers_*_kernel` on Apple Metal GPU over zero-copy Unified Memory
// pointers (for Scenario 2 warp-metal + mujoco_warp).
void bike_metal_launch_writers(
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
    if (nworld <= 0 || d_idx == nullptr || d_configs == nullptr || d_states == nullptr) {
        return;
    }
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.pso_1thread || !ctx.pso_simd32) {
        return;
    }

    const int nq = d_idx->nq;
    const int nv = d_idx->nv;
    const int nu = d_idx->nu;
    const int nbody3 = d_idx->nbody * 3;
    const int n_cfg = per_world_config ? nworld : 1;
    const bool use_simd32 = (d_configs[0].front_tire.num_lateral_ribs > 1) || (nworld < 64);

    NSUInteger off_idx = 0, off_hf = 0, off_cfg = 0, off_st = 0;
    NSUInteger off_qpos = 0, off_qvel = 0, off_xpos = 0, off_qfrc = 0, off_ctrl = 0, off_tendon = 0;

    id<MTLBuffer> b_idx = ctx.wrap_ptr(d_idx, sizeof(bike::gpu::GpuModelIndices), &off_idx);
    id<MTLBuffer> b_hf = ctx.wrap_ptr(
        d_hf_heights, static_cast<std::size_t>(std::max(1, hf_num_samples)) * sizeof(float), &off_hf
    );
    id<MTLBuffer> b_cfg = ctx.wrap_ptr(
        d_configs, static_cast<std::size_t>(n_cfg) * sizeof(bike::gpu::GpuWorldConfig), &off_cfg
    );
    id<MTLBuffer> b_st = ctx.wrap_ptr(
        d_states, static_cast<std::size_t>(nworld) * sizeof(bike::gpu::GpuWorldState), &off_st
    );
    id<MTLBuffer> b_qpos = ctx.wrap_ptr(
        d_qpos, static_cast<std::size_t>(nworld * nq) * sizeof(float), &off_qpos
    );
    id<MTLBuffer> b_qvel = ctx.wrap_ptr(
        d_qvel, static_cast<std::size_t>(nworld * nv) * sizeof(float), &off_qvel
    );
    id<MTLBuffer> b_xpos = ctx.wrap_ptr(
        d_xpos, static_cast<std::size_t>(nworld * nbody3) * sizeof(float), &off_xpos
    );
    id<MTLBuffer> b_qfrc = ctx.wrap_ptr(
        d_qfrc_applied, static_cast<std::size_t>(nworld * nv) * sizeof(float), &off_qfrc
    );
    id<MTLBuffer> b_ctrl = ctx.wrap_ptr(
        d_ctrl, static_cast<std::size_t>(nworld * nu) * sizeof(float), &off_ctrl
    );
    const std::size_t tendon_floats =
        (tendon_range_world_stride > 0)
            ? static_cast<std::size_t>(nworld * tendon_range_world_stride)
            : 2ULL;
    id<MTLBuffer> b_tendon = ctx.wrap_ptr(
        d_tendon_range, tendon_floats * sizeof(float), &off_tendon
    );

    id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
    id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
    encode_metal_writers_slice(
        enc,
        ctx,
        0,
        nworld,
        dt_s,
        b_idx, off_idx,
        b_hf, off_hf,
        hf_num_samples,
        hf_x_min,
        hf_dx,
        b_cfg, off_cfg,
        per_world_config,
        b_st, off_st,
        b_qpos, off_qpos,
        b_qvel, off_qvel,
        b_xpos, off_xpos,
        b_qfrc, off_qfrc,
        b_ctrl, off_ctrl,
        b_tendon, off_tendon,
        tendon_range_world_stride,
        use_simd32
    );
    [enc endEncoding];
    [cb commit];
    [cb waitUntilCompleted];
}

// Benchmarks pure Apple Metal GPU writers execution time across `n_steps` for `nworld` worlds.
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
    MetalWritersContext &ctx = get_metal_ctx();
    if (!ctx.pso_1thread || !ctx.pso_simd32) {
        return -2;
    }

    const int nq = d_idx->nq;
    const int nv = d_idx->nv;
    const int nu = d_idx->nu;
    const int nbody3 = d_idx->nbody * 3;
    const int n_cfg = per_world_config ? nworld : 1;
    const bool use_simd32 = (d_configs[0].front_tire.num_lateral_ribs > 1) || (nworld < 64);

    id<MTLBuffer> b_idx = [ctx.device newBufferWithBytes:d_idx
                                                  length:sizeof(bike::gpu::GpuModelIndices)
                                                 options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_hf = [ctx.device newBufferWithBytes:d_hf_heights
                                                 length:static_cast<NSUInteger>(std::max(1, hf_num_samples)) * sizeof(float)
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

    // Warmup
    for (int w = 0; w < 5; ++w) {
        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        encode_metal_writers_slice(
            enc, ctx, 0, nworld, dt_s,
            b_idx, 0, b_hf, 0, hf_num_samples, hf_x_min, hf_dx,
            b_cfg, 0, per_world_config, b_st, 0,
            b_qpos, 0, b_qvel, 0, b_xpos, 0, b_qfrc, 0, b_ctrl, 0, b_tendon, 0, 2,
            use_simd32
        );
        [enc endEncoding];
        [cb commit];
        [cb waitUntilCompleted];
    }

    const auto t0 = std::chrono::steady_clock::now();
    {
        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        for (int s = 0; s < n_steps; ++s) {
            encode_metal_writers_slice(
                enc, ctx, 0, nworld, dt_s,
                b_idx, 0, b_hf, 0, hf_num_samples, hf_x_min, hf_dx,
                b_cfg, 0, per_world_config, b_st, 0,
                b_qpos, 0, b_qvel, 0, b_xpos, 0, b_qfrc, 0, b_ctrl, 0, b_tendon, 0, 2,
                use_simd32
            );
        }
        [enc endEncoding];
        [cb commit];
        [cb waitUntilCompleted];
    }
    const auto t1 = std::chrono::steady_clock::now();

    const auto t2 = std::chrono::steady_clock::now();
    for (int s = 0; s < n_steps; ++s) {
        id<MTLCommandBuffer> cb = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        encode_metal_writers_slice(
            enc, ctx, 0, nworld, dt_s,
            b_idx, 0, b_hf, 0, hf_num_samples, hf_x_min, hf_dx,
            b_cfg, 0, per_world_config, b_st, 0,
            b_qpos, 0, b_qvel, 0, b_xpos, 0, b_qfrc, 0, b_ctrl, 0, b_tendon, 0, 2,
            use_simd32
        );
        [enc endEncoding];
        [cb commit];
        [cb waitUntilCompleted];
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
    if (!ctx.pso_1thread || !ctx.pso_simd32) {
        return -2;
    }

    char err[1024] = {0};
    mjModel *m_base = mj_loadXML(xml_path, nullptr, err, sizeof(err));
    if (m_base == nullptr) {
        return -3;
    }

    const int hw_threads = static_cast<int>(std::thread::hardware_concurrency());
    int n_workers = (num_threads > 0) ? num_threads : (hw_threads > 0 ? hw_threads : 4);
    n_workers = std::max(1, std::min(n_workers, nworld));

    const bike::gpu::GpuModelIndices idx =
        (d_idx != nullptr) ? *d_idx : bike::gpu::GpuModelIndices{};
    const int nq = m_base->nq;
    const int nv = m_base->nv;
    const int nu = m_base->nu;
    const int nbody3 = m_base->nbody * 3;
    const float dt_s = static_cast<float>(m_base->opt.timestep);
    const int n_cfg = per_world_config ? nworld : 1;
    const bool use_simd32 = (d_configs[0].front_tire.num_lateral_ribs > 1) || (nworld < 64);

    // Allocate Unified Memory (`MTLResourceStorageModeShared`) buffers shared zero-copy between GPU and CPU
    id<MTLBuffer> b_idx = [ctx.device newBufferWithBytes:&idx
                                                  length:sizeof(bike::gpu::GpuModelIndices)
                                                 options:MTLResourceStorageModeShared];
    id<MTLBuffer> b_hf = [ctx.device newBufferWithBytes:d_hf_heights
                                                 length:static_cast<NSUInteger>(std::max(1, hf_num_samples)) * sizeof(float)
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

    auto *st_ptr = static_cast<bike::gpu::GpuWorldState *>([b_st contents]);
    auto *qpos_ptr = static_cast<float *>([b_qpos contents]);
    auto *qvel_ptr = static_cast<float *>([b_qvel contents]);
    auto *xpos_ptr = static_cast<float *>([b_xpos contents]);
    auto *qfrc_ptr = static_cast<float *>([b_qfrc contents]);
    auto *ctrl_ptr = static_cast<float *>([b_ctrl contents]);
    auto *tendon_ptr = static_cast<float *>([b_tendon contents]);

    std::vector<mjModel *> worker_models(static_cast<std::size_t>(n_workers), nullptr);
    for (int t = 0; t < n_workers; ++t) {
        worker_models[static_cast<std::size_t>(t)] = mj_copyModel(nullptr, m_base);
    }

    std::vector<mjData *> world_data(static_cast<std::size_t>(nworld), nullptr);
    std::vector<float> peak_fork_m(static_cast<std::size_t>(nworld), 0.0f);
    std::vector<float> peak_shock_m(static_cast<std::size_t>(nworld), 0.0f);

    // Parallel initialization of all `nworld` mjData instances and initial Unified Memory state
    {
        std::atomic<int> init_cursor{0};
        auto init_worker = [&](int tid) {
            mjModel *m_t = worker_models[static_cast<std::size_t>(tid)];
            while (true) {
                const int w = init_cursor.fetch_add(1, std::memory_order_relaxed);
                if (w >= nworld) {
                    break;
                }
                mjData *d_w = mj_makeData(m_t);
                world_data[static_cast<std::size_t>(w)] = d_w;
                mj_resetData(m_t, d_w);
                if (init_qpos != nullptr) {
                    for (int i = 0; i < nq; ++i) {
                        d_w->qpos[i] = static_cast<mjtNum>(init_qpos[i]);
                    }
                }
                if (init_qvel != nullptr) {
                    for (int i = 0; i < nv; ++i) {
                        d_w->qvel[i] = static_cast<mjtNum>(init_qvel[i]);
                    }
                }
                mj_forward(m_t, d_w);

                float *qp = qpos_ptr + w * nq;
                float *qv = qvel_ptr + w * nv;
                float *xp = xpos_ptr + w * nbody3;
                for (int i = 0; i < nq; ++i) qp[i] = static_cast<float>(d_w->qpos[i]);
                for (int i = 0; i < nv; ++i) qv[i] = static_cast<float>(d_w->qvel[i]);
                for (int i = 0; i < nbody3; ++i) xp[i] = static_cast<float>(d_w->xpos[i]);

                if (m_base->ntendon > 0) {
                    tendon_ptr[w * 2 + 0] = static_cast<float>(m_base->tendon_range[0]);
                    tendon_ptr[w * 2 + 1] = static_cast<float>(m_base->tendon_range[1]);
                }

                const bike::gpu::GpuWorldConfig &cfg_w = d_configs[per_world_config ? w : 0];
                const float crank0 =
                    (idx.crank_spin_qpos >= 0 && idx.crank_spin_qpos < nq) ? qp[idx.crank_spin_qpos] : 0.0f;
                const float rwheel0 =
                    (idx.rear_wheel_spin_qpos >= 0 && idx.rear_wheel_spin_qpos < nq) ? qp[idx.rear_wheel_spin_qpos] : 0.0f;
                bike::gpu::gpu_init_world_state(st_ptr[w], cfg_w.drivetrain, crank0, rwheel0);
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
            b_idx, 0, b_hf, 0, hf_num_samples, hf_x_min, hf_dx,
            b_cfg, 0, per_world_config, b_st, 0,
            b_qpos, 0, b_qvel, 0, b_xpos, 0, b_qfrc, 0, b_ctrl, 0, b_tendon, 0, 2,
            use_simd32
        );
        [enc endEncoding];
        [cb commit];
        return cb;
    };

    // Warmup GPU pipeline once before starting clock and reset states[w]
    {
        id<MTLCommandBuffer> cb = dispatch_gpu_slice_async(0, nworld);
        [cb waitUntilCompleted];
        for (int w = 0; w < nworld; ++w) {
            const bike::gpu::GpuWorldConfig &cfg_w = d_configs[per_world_config ? w : 0];
            const float *qp = qpos_ptr + w * nq;
            const float crank0 =
                (idx.crank_spin_qpos >= 0 && idx.crank_spin_qpos < nq) ? qp[idx.crank_spin_qpos] : 0.0f;
            const float rwheel0 =
                (idx.rear_wheel_spin_qpos >= 0 && idx.rear_wheel_spin_qpos < nq) ? qp[idx.rear_wheel_spin_qpos] : 0.0f;
            bike::gpu::gpu_init_world_state(st_ptr[w], cfg_w.drivetrain, crank0, rwheel0);
        }
    }

    // Persistent spin-barrier thread pool for zero-allocation per-phase CPU mj_step execution
    std::atomic<int> phase_id{0};
    std::atomic<int> phase_w_begin{0};
    std::atomic<int> phase_w_end{0};
    std::atomic<int> phase_step{0};
    std::atomic<int> phase_cursor{0};
    std::atomic<int> workers_done{0};
    std::atomic<bool> stop_pool{false};

    auto step_world_cpu = [&](mjModel *m_t, int w, int step) {
        mjData *d_w = world_data[static_cast<std::size_t>(w)];
        const float *qf = qfrc_ptr + w * nv;
        const float *ct = ctrl_ptr + w * nu;
        for (int i = 0; i < nv; ++i) {
            d_w->qfrc_applied[i] = static_cast<mjtNum>(qf[i]);
        }
        for (int i = 0; i < nu; ++i) {
            d_w->ctrl[i] = static_cast<mjtNum>(ct[i]);
        }
        mj_step(m_t, d_w);

        float *qp = qpos_ptr + w * nq;
        float *qv = qvel_ptr + w * nv;
        float *xp = xpos_ptr + w * nbody3;
        for (int i = 0; i < nq; ++i) qp[i] = static_cast<float>(d_w->qpos[i]);
        for (int i = 0; i < nv; ++i) qv[i] = static_cast<float>(d_w->qvel[i]);
        const int b0 = 3 * idx.frame_bid;
        const int b1 = 3 * idx.front_wheel_bid;
        const int b2 = 3 * idx.rear_wheel_bid;
        xp[b0 + 0] = static_cast<float>(d_w->xpos[b0 + 0]);
        xp[b0 + 1] = static_cast<float>(d_w->xpos[b0 + 1]);
        xp[b0 + 2] = static_cast<float>(d_w->xpos[b0 + 2]);
        xp[b1 + 0] = static_cast<float>(d_w->xpos[b1 + 0]);
        xp[b1 + 1] = static_cast<float>(d_w->xpos[b1 + 1]);
        xp[b1 + 2] = static_cast<float>(d_w->xpos[b1 + 2]);
        xp[b2 + 0] = static_cast<float>(d_w->xpos[b2 + 0]);
        xp[b2 + 1] = static_cast<float>(d_w->xpos[b2 + 1]);
        xp[b2 + 2] = static_cast<float>(d_w->xpos[b2 + 2]);

        if (step >= 24) {
            if (idx.fork_travel_qpos >= 0 && idx.fork_travel_qpos < nq) {
                const float fk = qp[idx.fork_travel_qpos];
                if (fk > peak_fork_m[static_cast<std::size_t>(w)]) {
                    peak_fork_m[static_cast<std::size_t>(w)] = fk;
                }
            }
            if (idx.shock_stroke_qpos >= 0 && idx.shock_stroke_qpos < nq) {
                const float sh = qp[idx.shock_stroke_qpos];
                if (sh > peak_shock_m[static_cast<std::size_t>(w)]) {
                    peak_shock_m[static_cast<std::size_t>(w)] = sh;
                }
            }
        }
    };

    const int n_aux_threads = std::max(0, n_workers - 1);
    std::vector<std::thread> pool;
    pool.reserve(static_cast<std::size_t>(n_aux_threads));
    for (int t = 1; t < n_workers; ++t) {
        pool.emplace_back([&, t]() {
            mjModel *m_t = worker_models[static_cast<std::size_t>(t)];
            int local_phase = 0;
            while (true) {
                while (phase_id.load(std::memory_order_acquire) == local_phase) {
                    if (stop_pool.load(std::memory_order_relaxed)) {
                        return;
                    }
                    std::this_thread::yield();
                }
                if (stop_pool.load(std::memory_order_relaxed)) {
                    return;
                }
                local_phase = phase_id.load(std::memory_order_relaxed);
                const int w_end = phase_w_end.load(std::memory_order_relaxed);
                const int step = phase_step.load(std::memory_order_relaxed);
                while (true) {
                    const int w = phase_cursor.fetch_add(1, std::memory_order_relaxed);
                    if (w >= w_end) {
                        break;
                    }
                    step_world_cpu(m_t, w, step);
                }
                workers_done.fetch_add(1, std::memory_order_release);
            }
        });
    }

    auto run_cpu_slice = [&](int w_begin, int w_end, int step) {
        phase_w_begin.store(w_begin, std::memory_order_relaxed);
        phase_w_end.store(w_end, std::memory_order_relaxed);
        phase_step.store(step, std::memory_order_relaxed);
        phase_cursor.store(w_begin, std::memory_order_relaxed);
        workers_done.store(0, std::memory_order_relaxed);
        phase_id.fetch_add(1, std::memory_order_release);

        mjModel *m_0 = worker_models[0];
        while (true) {
            const int w = phase_cursor.fetch_add(1, std::memory_order_relaxed);
            if (w >= w_end) {
                break;
            }
            step_world_cpu(m_0, w, step);
        }
        while (workers_done.load(std::memory_order_acquire) < n_aux_threads) {
            std::this_thread::yield();
        }
    };

    const auto t_start = std::chrono::steady_clock::now();

    if (nworld >= 64 && n_workers > 1) {
        // Double-buffered Unified Memory pipeline:
        // Half 0 = [0, half), Half 1 = [half, nworld).
        // GPU computes force writers for Half 1 while 16 CPU cores run mj_step for Half 0, and vice versa.
        const int half = nworld / 2;
        id<MTLCommandBuffer> cb_h0 = dispatch_gpu_slice_async(0, half);
        [cb_h0 waitUntilCompleted];

        for (int step = 0; step < n_steps; ++step) {
            id<MTLCommandBuffer> cb_h1 = dispatch_gpu_slice_async(half, nworld - half);
            run_cpu_slice(0, half, step);
            [cb_h1 waitUntilCompleted];

            if (step + 1 < n_steps) {
                cb_h0 = dispatch_gpu_slice_async(0, half);
                run_cpu_slice(half, nworld, step);
                [cb_h0 waitUntilCompleted];
            } else {
                run_cpu_slice(half, nworld, step);
            }
        }
    } else {
        for (int step = 0; step < n_steps; ++step) {
            id<MTLCommandBuffer> cb = dispatch_gpu_slice_async(0, nworld);
            [cb waitUntilCompleted];
            run_cpu_slice(0, nworld, step);
        }
    }

    const auto t_end = std::chrono::steady_clock::now();
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();

    stop_pool.store(true, std::memory_order_relaxed);
    phase_id.fetch_add(1, std::memory_order_release);
    for (auto &th : pool) {
        th.join();
    }

    int all_finite = 1;
    for (int w = 0; w < nworld; ++w) {
        const float *qp = qpos_ptr + w * nq;
        const float *qv = qvel_ptr + w * nv;
        for (int i = 0; i < nq; ++i) {
            if (!std::isfinite(qp[i])) {
                all_finite = 0;
                break;
            }
        }
        for (int i = 0; i < nv; ++i) {
            if (!std::isfinite(qv[i])) {
                all_finite = 0;
                break;
            }
        }
        if (out_peak_fork_mm != nullptr) {
            out_peak_fork_mm[w] = peak_fork_m[static_cast<std::size_t>(w)] * 1000.0f;
        }
        if (out_peak_shock_mm != nullptr) {
            out_peak_shock_mm[w] = peak_shock_m[static_cast<std::size_t>(w)] * 1000.0f;
        }
        if (out_final_root_x_m != nullptr) {
            out_final_root_x_m[w] =
                (idx.root_x_qpos >= 0 && idx.root_x_qpos < nq) ? qp[idx.root_x_qpos] : 0.0f;
        }
    }

    if (out_all_finite != nullptr) {
        *out_all_finite = all_finite;
    }
    if (out_elapsed_ms != nullptr) {
        *out_elapsed_ms = elapsed_ms;
    }

    for (int w = 0; w < nworld; ++w) {
        mj_deleteData(world_data[static_cast<std::size_t>(w)]);
    }
    for (int t = 0; t < n_workers; ++t) {
        mj_deleteModel(worker_models[static_cast<std::size_t>(t)]);
    }
    mj_deleteModel(m_base);
    return 0;
}

} // extern "C"
