// native/src/gpu/common.hpp — Shared __host__ __device__ primitives and POD layouts
// for the allocation-free C++/CUDA/Metal GPU force writers.
//
// Compiles cleanly under:
//   1. Standard C++ host compilers (clang++ / g++) for local unit tests.
//   2. NVIDIA CUDA nvcc (C++20) for device kernels fused with mujoco_warp.
//   3. Apple Metal Shading Language (MSL) for Metal GPU writers on Apple Silicon.
#pragma once

#if defined(__METAL_VERSION__)
#include <metal_stdlib>
#define noexcept
#define BIKE_HD inline
#define BIKE_INLINE inline
#define BIKE_THREAD thread
#define BIKE_DEVICE device
#define BIKE_CONSTANT constant
#define BIKE_GLOBAL kernel
#define BIKE_CONSTEXPR constant constexpr
#elif defined(__CUDACC__)
#include <cmath>
#include <cstdint>
#define BIKE_HD __host__ __device__ __forceinline__
#define BIKE_INLINE __forceinline__
#define BIKE_THREAD
#define BIKE_DEVICE
#define BIKE_CONSTANT
#define BIKE_GLOBAL __global__
#define BIKE_CONSTEXPR inline constexpr
#else
#include <cmath>
#include <cstdint>
#define BIKE_HD inline
#define BIKE_INLINE inline
#define BIKE_THREAD
#define BIKE_DEVICE
#define BIKE_CONSTANT
#define BIKE_GLOBAL
#define BIKE_CONSTEXPR inline constexpr
#endif

#if defined(__clang__) && !defined(__METAL_VERSION__) && !defined(__CUDA_ARCH__)
#define BIKE_GPU_SHARED_CODE_BEGIN \
    _Pragma("clang diagnostic push") \
    _Pragma("clang diagnostic ignored \"-Wunsafe-buffer-usage\"")
#define BIKE_GPU_SHARED_CODE_END _Pragma("clang diagnostic pop")
#else
#define BIKE_GPU_SHARED_CODE_BEGIN
#define BIKE_GPU_SHARED_CODE_END
#endif

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: MSL has no std::span/std::array and address-space-qualified raw pointers are the device ABI; positional float signatures mirror the CUDA/MSL kernels. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::gpu {

BIKE_CONSTEXPR int kMaxNq = 32;
BIKE_CONSTEXPR int kMaxNv = 32;
BIKE_CONSTEXPR int kMaxNu = 16;
BIKE_CONSTEXPR int kMaxNbody = 32;
BIKE_CONSTEXPR int kMaxTreadStations = 64;
BIKE_CONSTEXPR int kMaxHingeKnots = 8;

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

// Number of controlled rider joints in model.xml:
//   [0]: rider_hip_front      (qpos 22, dof 22, ctrl 8)
//   [1]: rider_knee_front     (qpos 23, dof 23, ctrl 9)
//   [2]: rider_hip_rear       (qpos 24, dof 24, ctrl 10)
//   [3]: rider_knee_rear      (qpos 25, dof 25, ctrl 11)
//   [4]: rider_torso_hinge    (qpos 17, dof 17, ctrl 3)
//   [5]: rider_shoulder_left  (qpos 18, dof 18, ctrl 4)
//   [6]: rider_elbow_left     (qpos 19, dof 19, ctrl 5)
//   [7]: rider_shoulder_right (qpos 20, dof 20, ctrl 6)
//   [8]: rider_elbow_right    (qpos 21, dof 21, ctrl 7)
BIKE_CONSTEXPR int kNumRiderJoints = 9;

// Fixed model address table resolved once on the host from mjModel.
// Total non-static int fields = 57 (sizeof(GpuModelIndices) == 228 bytes).
struct GpuModelIndices {
    int nq = 26;
    int nv = 26;
    int nu = 12;
    int nbody = 28;

    // Body indices matching tools/fp32_probe/artifacts/common/model.xml
    int frame_body = 1;
    int steer_body = 5;
    int fork_lower_body = 6;
    int front_wheel_body = 7;
    int chainstay_body = 8;
    int seatstay_body = 9;
    int rear_wheel_body = 10;

    // Joint qpos addresses
    int root_x_qpos = 0;
    int root_z_qpos = 1;
    int root_pitch_qpos = 2;
    int crank_spin_qpos = 3;
    int fork_travel_qpos = 6;
    int front_wheel_spin_qpos = 7;
    int rear_wheel_spin_qpos = 10;
    int shock_stroke_qpos = 13;
    int rider_qpos[kNumRiderJoints] = {22, 23, 24, 25, 17, 18, 19, 20, 21};

    // Joint dof addresses
    int root_x_dof = 0;
    int root_z_dof = 1;
    int root_pitch_dof = 2;
    int crank_spin_dof = 3;
    int fork_travel_dof = 6;
    int front_wheel_spin_dof = 7;
    int rear_wheel_spin_dof = 10;
    int shock_stroke_dof = 13;
    int rider_dof[kNumRiderJoints] = {22, 23, 24, 25, 17, 18, 19, 20, 21};

    // Actuator ctrl addresses
    int rider_ctrl[kNumRiderJoints] = {8, 9, 10, 11, 3, 4, 5, 6, 7};

    // Suspension pivot DOFs in model.xml (main_pivot=8, horst_pivot=9)
    int main_pivot_dof = 8;
    int horst_pivot_dof = 9;

    // Tendon index in model.xml (ideal_mid_drive_freehub=0)
    int freehub_tendon = 0;
};

static_assert(sizeof(GpuModelIndices) == 228, "GpuModelIndices must be 228 bytes");

#if defined(__METAL_VERSION__)
BIKE_HD float gpu_min(float a, float b) noexcept {
    return metal::min(a, b);
}

BIKE_HD float gpu_max(float a, float b) noexcept {
    return metal::max(a, b);
}

BIKE_HD float gpu_clamp(float v, float lo, float hi) noexcept {
    return metal::clamp(v, lo, hi);
}

BIKE_HD float gpu_abs(float v) noexcept {
    return metal::abs(v);
}

BIKE_HD float gpu_sqrt(float v) noexcept {
    return metal::sqrt(v);
}

BIKE_HD float gpu_pow(float base, float exp) noexcept {
    return metal::pow(base, exp);
}

BIKE_HD float gpu_tanh(float v) noexcept {
    return metal::tanh(gpu_clamp(v, -15.0f, 15.0f));
}

BIKE_HD float gpu_sin(float v) noexcept {
    return metal::sin(v);
}

BIKE_HD float gpu_cos(float v) noexcept {
    return metal::cos(v);
}

BIKE_HD float gpu_floor(float v) noexcept {
    return metal::floor(v);
}

BIKE_HD float gpu_fmod(float x, float y) noexcept {
    return metal::fmod(x, y);
}

BIKE_HD float gpu_atan2(float y, float x) noexcept {
    return metal::atan2(y, x);
}

BIKE_HD bool gpu_isfinite(float v) noexcept {
    return metal::isfinite(v);
}
#else
BIKE_HD float gpu_min(float a, float b) noexcept {
    return ::fminf(a, b);
}

BIKE_HD float gpu_max(float a, float b) noexcept {
    return ::fmaxf(a, b);
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
    return ::tanhf(gpu_clamp(v, -15.0f, 15.0f));
}

BIKE_HD float gpu_sin(float v) noexcept {
    return ::sinf(v);
}

BIKE_HD float gpu_cos(float v) noexcept {
    return ::cosf(v);
}

BIKE_HD float gpu_floor(float v) noexcept {
    return ::floorf(v);
}

BIKE_HD float gpu_fmod(float x, float y) noexcept {
    return ::fmodf(x, y);
}

BIKE_HD float gpu_atan2(float y, float x) noexcept {
    return ::atan2f(y, x);
}

BIKE_HD bool gpu_isfinite(float v) noexcept {
#if defined(__CUDACC__)
    return ::isfinite(v) != 0;
#else
    return std::isfinite(v);
#endif
}
#endif

BIKE_HD float dot3(const BIKE_THREAD GpuVec3 &a, const BIKE_THREAD GpuVec3 &b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

BIKE_HD float norm3(const BIKE_THREAD GpuVec3 &v) noexcept {
    return gpu_sqrt(dot3(v, v));
}

// Projects a planar world force (Fx, 0, Fz) applied at world point `(pt_x, pt_z)` on a body
// onto the planar root DOFs (root_x, root_z, root_pitch) relative to root CoM `(com_x, com_z)`.
BIKE_HD void project_planar_force_to_root(
    const BIKE_THREAD GpuModelIndices &idx,
    float com_x,
    float com_z,
    float pt_x,
    float pt_z,
    float fx,
    float fz,
    BIKE_THREAD float *qfrc_world_row
) noexcept {
    qfrc_world_row[idx.root_x_dof] += fx;
    qfrc_world_row[idx.root_z_dof] += fz;
    // In MuJoCo planar X-Z with Y-axis pitch: torque_y = (pt_z - com_z)*fx - (pt_x - com_x)*fz
    const float dx = pt_x - com_x;
    const float dz = pt_z - com_z;
    qfrc_world_row[idx.root_pitch_dof] += dz * fx - dx * fz;
}

// Legacy argument-order overload for callers passing (fx, fz, pt_x, pt_z, com_x, com_z, idx, qfrc).
BIKE_HD void project_planar_force_to_root(
    float fx,
    float fz,
    float pt_x,
    float pt_z,
    float com_x,
    float com_z,
    const BIKE_THREAD GpuModelIndices &idx,
    BIKE_THREAD float *qfrc_world_row
) noexcept {
    project_planar_force_to_root(
        idx, com_x, com_z, pt_x, pt_z, fx, fz, qfrc_world_row);
}

// Model-derived frame-local geometry constants (from tools/fp32_probe/artifacts/common/model.xml):
//   jnt_axis[fork_travel] = (-0.43837105, 0, 0.89879410)
//   body_pos[steer]       = (0.532605, 0, 0.538145), unit = (0.70343884, 0, 0.71075580)
//   body_pos[chainstay]   = (-0.044902, 0, 0.046393)
//   body_pos[seatstay]    = (-0.334216, 0, -0.054835) relative to chainstay
// root_pitch, main_pivot and horst_pivot are all hinges with jnt_axis (0, 1, 0), so a hinge angle
// theta rotates frame-local (x, z) by R(theta): (c*x + s*z, -s*x + c*z).
BIKE_CONSTEXPR float kForkAxisLocalX = -0.43837105f;
BIKE_CONSTEXPR float kForkAxisLocalZ = 0.89879410f;
BIKE_CONSTEXPR float kSteerToForkCos = 0.33045590f;
BIKE_CONSTEXPR float kSteerToForkSin = 0.94382143f;
BIKE_CONSTEXPR float kMainPivotLocalDxM = -0.0449020f;
BIKE_CONSTEXPR float kMainPivotLocalDzM = 0.0463930f;
// Seatstay origin relative to the chainstay origin, in the chainstay frame (model body_pos[seatstay]).
// The horst fallback rotates this offset by root_pitch + main_pivot.
BIKE_CONSTEXPR float kSeatstayFromChainstayDxM = -0.3342160f;
BIKE_CONSTEXPR float kSeatstayFromChainstayDzM = -0.0548350f;
// Seatstay origin in the frame at main_pivot = 0, i.e. kMainPivotLocal + kSeatstayFromChainstay.
// NOT the horst fallback for main_pivot != 0 (use gpu_horst_pivot_anchor_fallback). No production caller
// passes it to gpu_rear_pivot_anchor_world: tire_kernel.hpp and metal_batch_runner.mm use
// gpu_horst_pivot_anchor_world for the seatstay. It remains for the benchmark seatstay seed in
// metal_batch_runner.mm and the pre-fix anchor comparison in test_gpu_tire.cpp.
BIKE_CONSTEXPR float kHorstPivotLocalDxM = -0.3791180f;
BIKE_CONSTEXPR float kHorstPivotLocalDzM = -0.0084420f;

// Computes the unit vector e_fork = (e_x, e_z) of the fork_travel slide axis in world coordinates.
// Derives frame orientation from (xpos[steer] - xpos[frame]), which has fixed non-zero length
// (0.75714 m in model.xml) and never vanishes or flips sign when fork_travel <= 0 (top-out).
BIKE_HD GpuVec2 gpu_fork_axis_world(
    const BIKE_THREAD GpuModelIndices &idx,
    const BIKE_THREAD float *xpos_world_row,
    float root_pitch_rad = 0.0f
) noexcept {
    if (xpos_world_row != nullptr && idx.steer_body >= 0 && idx.frame_body >= 0) {
        const float dx = xpos_world_row[3 * idx.steer_body + 0] - xpos_world_row[3 * idx.frame_body + 0];
        const float dz = xpos_world_row[3 * idx.steer_body + 2] - xpos_world_row[3 * idx.frame_body + 2];
        const float len_sq = dx * dx + dz * dz;
        if (len_sq > 1e-10f) {
            const float inv_len = 1.0f / gpu_sqrt(len_sq);
            const float ux = dx * inv_len;
            const float uz = dz * inv_len;
            return GpuVec2{
                .x = kSteerToForkCos * ux - kSteerToForkSin * uz,
                .y = kSteerToForkSin * ux + kSteerToForkCos * uz
            };
        }
    }
    const float c = gpu_cos(root_pitch_rad);
    const float s = gpu_sin(root_pitch_rad);
    return GpuVec2{
        .x = c * kForkAxisLocalX + s * kForkAxisLocalZ,
        .y = -s * kForkAxisLocalX + c * kForkAxisLocalZ
    };
}

// Projects planar force (fx_world, fz_world) applied at (px_world, pz_world) on the front wheel
// into root_x_dof, root_z_dof, root_pitch_dof AND into fork_travel_dof.
BIKE_HD void project_front_wheel_force_to_qfrc(
    const BIKE_THREAD GpuModelIndices &idx,
    const BIKE_THREAD float *xpos_world_row,
    float com_x_world,
    float com_z_world,
    float px_world,
    float pz_world,
    float fx_world,
    float fz_world,
    BIKE_THREAD float *qfrc_world_row,
    float root_pitch_rad = 0.0f
) noexcept {
    project_planar_force_to_root(
        idx, com_x_world, com_z_world, px_world, pz_world, fx_world, fz_world, qfrc_world_row);
    if (idx.fork_travel_dof >= 0) {
        const GpuVec2 e_fork = gpu_fork_axis_world(idx, xpos_world_row, root_pitch_rad);
        qfrc_world_row[idx.fork_travel_dof] += e_fork.x * fx_world + e_fork.y * fz_world;
    }
}

// Computes the world anchor for a rear suspension pivot, using xpos_world_row if available
// or falling back to a frame-local offset rotated by root_pitch_rad. The fallback is exact only
// for a body whose origin is frame-local (the chainstay). For the seatstay use
// gpu_horst_pivot_anchor_world, whose fallback also needs main_pivot_rad.
BIKE_HD GpuVec2 gpu_rear_pivot_anchor_world(
    int pivot_body,
    const BIKE_THREAD float *xpos_world_row,
    float com_x_world,
    float com_z_world,
    float fallback_dx,
    float fallback_dz,
    float root_pitch_rad = 0.0f
) noexcept {
    if (xpos_world_row != nullptr && pivot_body >= 0) {
        const float ax = xpos_world_row[3 * pivot_body + 0];
        const float az = xpos_world_row[3 * pivot_body + 2];
        if (gpu_abs(ax) > 1e-6f || gpu_abs(az) > 1e-6f) {
            return GpuVec2{.x = ax, .y = az};
        }
    }
    const float c = gpu_cos(root_pitch_rad);
    const float s = gpu_sin(root_pitch_rad);
    return GpuVec2{
        .x = com_x_world + c * fallback_dx + s * fallback_dz,
        .y = com_z_world - s * fallback_dx + c * fallback_dz
    };
}

// Seatstay (horst pivot) origin without xpos. Model chain: frame --root_pitch--> chainstay
// --main_pivot--> seatstay. The chainstay origin is frame-local at R(root_pitch). The seatstay origin
// lies at kSeatstayFromChainstay in the chainstay frame, whose world rotation is
// R(root_pitch) * R(main_pivot) = R(root_pitch + main_pivot), since both hinges share axis (0, 1, 0).
// Hence horst = com + R(rp) * kMainPivotLocal + R(rp + mp) * kSeatstayFromChainstay.
// Verified against MuJoCo d->xanchor[horst_pivot] by test_gpu_model_geometry at main_pivot = +-0.05 rad.
BIKE_HD GpuVec2 gpu_horst_pivot_anchor_fallback(
    float com_x_world,
    float com_z_world,
    float root_pitch_rad,
    float main_pivot_rad
) noexcept {
    const GpuVec2 a_main = gpu_rear_pivot_anchor_world(
        -1, nullptr, com_x_world, com_z_world, kMainPivotLocalDxM, kMainPivotLocalDzM, root_pitch_rad);
    const float total_rad = root_pitch_rad + main_pivot_rad;
    const float c = gpu_cos(total_rad);
    const float s = gpu_sin(total_rad);
    return GpuVec2{
        .x = a_main.x + c * kSeatstayFromChainstayDxM + s * kSeatstayFromChainstayDzM,
        .y = a_main.y - s * kSeatstayFromChainstayDxM + c * kSeatstayFromChainstayDzM
    };
}

// Seatstay (horst pivot) world anchor: xpos[seatstay] when available (same validity rule as
// gpu_rear_pivot_anchor_world), otherwise gpu_horst_pivot_anchor_fallback.
BIKE_HD GpuVec2 gpu_horst_pivot_anchor_world(
    int seatstay_body,
    const BIKE_THREAD float *xpos_world_row,
    float com_x_world,
    float com_z_world,
    float root_pitch_rad,
    float main_pivot_rad
) noexcept {
    if (xpos_world_row != nullptr && seatstay_body >= 0) {
        const float ax = xpos_world_row[3 * seatstay_body + 0];
        const float az = xpos_world_row[3 * seatstay_body + 2];
        if (gpu_abs(ax) > 1e-6f || gpu_abs(az) > 1e-6f) {
            return GpuVec2{.x = ax, .y = az};
        }
    }
    return gpu_horst_pivot_anchor_fallback(com_x_world, com_z_world, root_pitch_rad, main_pivot_rad);
}

// Projects planar force (fx_world, fz_world) applied at (px_world, pz_world) on the rear wheel
// into root_x_dof, root_z_dof, root_pitch_dof AND into main_pivot_dof and horst_pivot_dof.
//
// Pivot anchors come from xpos_world_row when it is non-null (production callers always pass it).
// Without xpos the anchors are rebuilt from the rest chain, which is exact only if BOTH
// root_pitch_rad and main_pivot_rad are supplied: root_pitch_rad rotates the frame and
// main_pivot_rad rotates the chainstay about its origin. The defaults (0) are correct only at
// root_pitch = main_pivot = 0.
BIKE_HD void project_rear_wheel_force_to_qfrc(
    const BIKE_THREAD GpuModelIndices &idx,
    const BIKE_THREAD float *xpos_world_row,
    float com_x_world,
    float com_z_world,
    float px_world,
    float pz_world,
    float fx_world,
    float fz_world,
    BIKE_THREAD float *qfrc_world_row,
    float root_pitch_rad = 0.0f,
    float main_pivot_rad = 0.0f
) noexcept {
    project_planar_force_to_root(
        idx, com_x_world, com_z_world, px_world, pz_world, fx_world, fz_world, qfrc_world_row);
    if (idx.main_pivot_dof >= 0) {
        const GpuVec2 a_main = gpu_rear_pivot_anchor_world(
            idx.chainstay_body, xpos_world_row, com_x_world, com_z_world,
            kMainPivotLocalDxM, kMainPivotLocalDzM, root_pitch_rad);
        qfrc_world_row[idx.main_pivot_dof] +=
            (pz_world - a_main.y) * fx_world - (px_world - a_main.x) * fz_world;
    }
    if (idx.horst_pivot_dof >= 0) {
        const GpuVec2 a_horst = gpu_horst_pivot_anchor_world(
            idx.seatstay_body, xpos_world_row, com_x_world, com_z_world, root_pitch_rad, main_pivot_rad);
        qfrc_world_row[idx.horst_pivot_dof] +=
            (pz_world - a_horst.y) * fx_world - (px_world - a_horst.x) * fz_world;
    }
}

} // namespace bike::gpu

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
