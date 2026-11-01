// native/src/gpu/common.hpp — Shared __host__ __device__ primitives and POD layouts
// for the allocation-free C++/CUDA GPU force writers.
//
// Compiles cleanly under both:
//   1. Standard C++23 host compilers (clang++ / g++) for local unit tests.
//   2. NVIDIA CUDA nvcc (C++20/C++23) for device kernels fused with mujoco_warp.
#pragma once

#include <cmath>
#include <cstdint>

#if defined(__CUDACC__)
#define BIKE_HD __host__ __device__ __forceinline__
#define BIKE_DEVICE __device__ __forceinline__
#define BIKE_GLOBAL __global__
#else
#define BIKE_HD inline
#define BIKE_DEVICE inline
#define BIKE_GLOBAL
#endif

namespace bike::gpu {

inline constexpr int kMaxNq = 26;
inline constexpr int kMaxNv = 26;
inline constexpr int kMaxNu = 12;
inline constexpr int kMaxTreadStations = 64;
inline constexpr int kMaxHingeKnots = 8;

struct GpuVec2 {
    float x = 0.0f;
    float y = 0.0f;
};

struct GpuVec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct GpuMat33 {
    float m[9] = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    };
};

// Fixed model address table resolved once on the host from mjModel.
struct GpuModelIndices {
    int nq = 26;
    int nv = 26;
    int nu = 12;
    int nbody = 23;

    // Body IDs
    int frame_bid = 1;
    int front_wheel_bid = 7;
    int rear_wheel_bid = 10;

    // Joint qpos addresses
    int root_x_qpos = 0;
    int root_z_qpos = 1;
    int root_pitch_qpos = 2;
    int crank_spin_qpos = 3;
    int fork_travel_qpos = 6;
    int front_wheel_spin_qpos = 7;
    int rear_wheel_spin_qpos = 10;
    int shock_stroke_qpos = 13;
    int rider_qpos[6] = {16, 17, 18, 23, 24, 25};

    // Joint dof addresses
    int root_x_dof = 0;
    int root_z_dof = 1;
    int root_pitch_dof = 2;
    int crank_spin_dof = 3;
    int fork_travel_dof = 6;
    int front_wheel_spin_dof = 7;
    int rear_wheel_spin_dof = 10;
    int shock_stroke_dof = 13;
    int rider_dof[6] = {16, 17, 18, 23, 24, 25};

    // Actuator ctrl addresses
    int rider_ctrl[6] = {6, 7, 8, 9, 10, 11};
};

BIKE_HD float gpu_min(float a, float b) noexcept {
    return a < b ? a : b;
}

BIKE_HD float gpu_max(float a, float b) noexcept {
    return a < b ? b : a;
}

BIKE_HD float gpu_clamp(float v, float lo, float hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

BIKE_HD float gpu_abs(float v) noexcept {
    return ::fabsf(v);
}

BIKE_HD float gpu_sqrt(float v) noexcept {
    return ::sqrtf(v);
}

BIKE_HD float gpu_pow(float base, float exp) noexcept {
    return ::powf(base, exp);
}

BIKE_HD float gpu_tanh(float v) noexcept {
    return ::tanhf(v);
}

BIKE_HD float gpu_sin(float v) noexcept {
    return ::sinf(v);
}

BIKE_HD float gpu_cos(float v) noexcept {
    return ::cosf(v);
}

BIKE_HD bool gpu_isfinite(float v) noexcept {
#if defined(__CUDACC__)
    return ::isfinite(v) != 0;
#else
    return std::isfinite(v);
#endif
}

BIKE_HD float dot3(const GpuVec3 &a, const GpuVec3 &b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

BIKE_HD float norm3(const GpuVec3 &v) noexcept {
    return gpu_sqrt(dot3(v, v));
}

// Projects a planar world force (Fx, 0, Fz) applied at world point `pt` on a body
// onto the planar root DOFs (root_x, root_z, root_pitch) relative to root CoM `(rx, rz)`.
BIKE_HD void project_planar_force_to_root(
    float fx,
    float fz,
    float pt_x,
    float pt_z,
    float com_x,
    float com_z,
    const GpuModelIndices &idx,
    float *qfrc_world_row
) noexcept {
    qfrc_world_row[idx.root_x_dof] += fx;
    qfrc_world_row[idx.root_z_dof] += fz;
    // In MuJoCo planar X-Z with Y-axis pitch: torque_y = (pt_z - com_z)*fx - (pt_x - com_x)*fz
    const float dx = pt_x - com_x;
    const float dz = pt_z - com_z;
    qfrc_world_row[idx.root_pitch_dof] += dz * fx - dx * fz;
}

} // namespace bike::gpu
