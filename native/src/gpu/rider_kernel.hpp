// native/src/gpu/rider_kernel.hpp — Allocation-free __host__ __device__
// rider joint ROM envelope and 9-DOF posture control kernel. Legs are passive
// crank followers; only the upper body is actively driven.
#pragma once

#include "common.hpp"

#if !defined(__METAL_VERSION__) && !defined(__CUDA_ARCH__)
#include <numbers>
#endif

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: MSL has no std::span/std::array and address-space-qualified raw pointers are the device ABI; positional float signatures mirror the CUDA/MSL kernels. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::gpu {

#if !defined(__METAL_VERSION__) && !defined(__CUDA_ARCH__)
BIKE_CONSTEXPR float kDegToRad = std::numbers::pi_v<float> / 180.0f;
#else
BIKE_CONSTEXPR float kDegToRad = 3.14159265358979323846f / 180.0f;
#endif

// Per-joint anatomical ROM envelope, soft boundary impedance, and torque limits.
struct GpuJointEnvelopeSpec {
    float min_deg = -90.0f;
    float max_deg = 90.0f;
    float ref_deg = 0.0f;
    int sign_flip = 0;
    float margin_rad = 0.0872664626f;
    float k_edge_nm_rad = 200.0f;
    float c_edge_nm_s_rad = 5.0f;
    float max_ext_torque_nm = 250.0f;
    float max_flex_torque_nm = 150.0f;
};

// Complete 9-DOF rider parameter table paired 1-to-1 with model.xml:
//   joints[0..8] = {
//     0: rider_hip_front      (qpos 22, dof 22, ctrl 8)
//     1: rider_knee_front     (qpos 23, dof 23, ctrl 9)
//     2: rider_hip_rear       (qpos 24, dof 24, ctrl 10)
//     3: rider_knee_rear      (qpos 25, dof 25, ctrl 11)
//     4: rider_torso_hinge    (qpos 17, dof 17, ctrl 3)
//     5: rider_shoulder_left  (qpos 18, dof 18, ctrl 4)
//     6: rider_elbow_left     (qpos 19, dof 19, ctrl 5)
//     7: rider_shoulder_right (qpos 20, dof 20, ctrl 6)
//     8: rider_elbow_right    (qpos 21, dof 21, ctrl 7)
//   }
//
// Design: the 4 leg joints (slots 0..3) are passive followers of the crank. Each foot is
// tied to its pedal by a connect_foot_* equality constraint in model.xml, so the leg joint
// angles follow the crank angle through the closed loop. This kernel does not drive the legs:
// their kp_posture and kd_posture are 0, and no crank-phase target or pedaling torque is
// applied, so their actuator ctrl is exactly 0. Their ROM soft-edge envelope torque and
// envelope energy are still computed. Crank effort is applied once, by the drivetrain kernel
// (drivetrain_resistance_kernel.hpp, rider_crank_torque added to crank_spin). Upper-body
// joints (slots 4..8) are held by posture PD toward target_qpos_rad.
struct GpuRiderParams {
    GpuJointEnvelopeSpec joints[kNumRiderJoints] = {
        // 0: rider_hip_front (model.xml q_range = [-1.326989, +1.383011] rad = [-76.03, +79.24] deg)
        {
            .min_deg = -120.0f, .max_deg = 35.27f, .ref_deg = -43.97f, .sign_flip = 0,
            .margin_rad = 0.0872664626f, .k_edge_nm_rad = 220.0f, .c_edge_nm_s_rad = 8.0f,
            .max_ext_torque_nm = 280.0f, .max_flex_torque_nm = 140.0f,
        },
        // 1: rider_knee_front (model.xml q_range = [-1.108838, +1.331162] rad = [-63.53, +76.27] deg)
        {
            .min_deg = -139.80f, .max_deg = 0.0f, .ref_deg = -76.27f, .sign_flip = 0,
            .margin_rad = 0.0872664626f, .k_edge_nm_rad = 240.0f, .c_edge_nm_s_rad = 8.0f,
            .max_ext_torque_nm = 300.0f, .max_flex_torque_nm = 130.0f,
        },
        // 2: rider_hip_rear (model.xml q_range = [-1.621450, +1.088550] rad = [-92.90, +62.37] deg)
        {
            .min_deg = -120.0f, .max_deg = 35.27f, .ref_deg = -27.10f, .sign_flip = 0,
            .margin_rad = 0.0872664626f, .k_edge_nm_rad = 220.0f, .c_edge_nm_s_rad = 8.0f,
            .max_ext_torque_nm = 280.0f, .max_flex_torque_nm = 140.0f,
        },
        // 3: rider_knee_rear (model.xml q_range = [-1.420899, +1.019101] rad = [-81.41, +58.39] deg)
        {
            .min_deg = -139.80f, .max_deg = 0.0f, .ref_deg = -58.39f, .sign_flip = 0,
            .margin_rad = 0.0872664626f, .k_edge_nm_rad = 240.0f, .c_edge_nm_s_rad = 8.0f,
            .max_ext_torque_nm = 300.0f, .max_flex_torque_nm = 130.0f,
        },
        // 4: rider_torso_hinge (model.xml q_range = [-1.232562, +0.567438] rad = [-70.62, +32.51] deg)
        {
            .min_deg = -38.11f, .max_deg = 65.02f, .ref_deg = 32.51f, .sign_flip = 0,
            .margin_rad = 0.0872664626f, .k_edge_nm_rad = 200.0f, .c_edge_nm_s_rad = 6.0f,
            .max_ext_torque_nm = 220.0f, .max_flex_torque_nm = 180.0f,
        },
        // 5: rider_shoulder_left (model.xml q_range = [-1.773614, +2.056386] rad = [-101.62, +117.82] deg)
        {
            .min_deg = -60.0f, .max_deg = 159.44f, .ref_deg = 41.62f, .sign_flip = 0,
            .margin_rad = 0.0872664626f, .k_edge_nm_rad = 140.0f, .c_edge_nm_s_rad = 4.0f,
            .max_ext_torque_nm = 140.0f, .max_flex_torque_nm = 120.0f,
        },
        // 6: rider_elbow_left (model.xml q_range = [-2.379773, +0.240227] rad = [-136.35, +13.76] deg)
        {
            .min_deg = 0.0f, .max_deg = 150.11f, .ref_deg = 13.76f, .sign_flip = 1,
            .margin_rad = 0.0872664626f, .k_edge_nm_rad = 120.0f, .c_edge_nm_s_rad = 3.5f,
            .max_ext_torque_nm = 110.0f, .max_flex_torque_nm = 110.0f,
        },
        // 7: rider_shoulder_right (model.xml q_range = [-1.773614, +2.056386] rad = [-101.62, +117.82] deg)
        {
            .min_deg = -60.0f, .max_deg = 159.44f, .ref_deg = 41.62f, .sign_flip = 0,
            .margin_rad = 0.0872664626f, .k_edge_nm_rad = 140.0f, .c_edge_nm_s_rad = 4.0f,
            .max_ext_torque_nm = 140.0f, .max_flex_torque_nm = 120.0f,
        },
        // 8: rider_elbow_right (model.xml q_range = [-2.379773, +0.240227] rad = [-136.35, +13.76] deg)
        {
            .min_deg = 0.0f, .max_deg = 150.11f, .ref_deg = 13.76f, .sign_flip = 1,
            .margin_rad = 0.0872664626f, .k_edge_nm_rad = 120.0f, .c_edge_nm_s_rad = 3.5f,
            .max_ext_torque_nm = 110.0f, .max_flex_torque_nm = 110.0f,
        },
    };
    // Posture PD gains. Legs (0..3) are zero, so their ctrl is exactly 0 and they follow the
    // crank passively. Upper body (4..8) gains hold posture.
    float kp_posture[kNumRiderJoints] = {
        0.0f, 0.0f, 0.0f, 0.0f, 180.0f, 95.0f, 75.0f, 95.0f, 75.0f
    };
    float kd_posture[kNumRiderJoints] = {
        0.0f, 0.0f, 0.0f, 0.0f, 18.0f, 9.5f, 7.5f, 9.5f, 7.5f
    };
    // Posture targets (rad). Entries 0..3 (legs) are unused while kp = kd = 0; they are kept so
    // the table stays 9-DOF and paired with joints[].
    float target_qpos_rad[kNumRiderJoints] = {
        -0.02f, 0.05f, -0.23f, -0.25f, -0.14185f, 0.08543f, -0.14391f, 0.08543f, -0.14391f
    };
    int rider_enabled = 1;
};

// Output of a single rider evaluation step.
struct GpuRiderOutput {
    float envelope_torque_nm[kNumRiderJoints] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };
    float actuator_ctrl_nm[kNumRiderJoints] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };
    float envelope_energy_j = 0.0f;
};

// Port of spindle::anatomical_angle from native/src/rider/spindle_math.cpp:
// maps generalized coordinate `q_rad` into the wrapped anatomical angle (rad).
BIKE_HD float gpu_anatomical_angle(
    float q_rad,
    float ref_deg,
    int sign_flip
) noexcept {
    const float dir = (sign_flip != 0) ? -1.0f : 1.0f;
    const float ref_rad = ref_deg * kDegToRad;
    const float value = ref_rad + dir * q_rad;
    return gpu_atan2(gpu_sin(value), gpu_cos(value));
}

// Port of spindle::joint_q_range from native/src/rider/spindle_math.cpp:
// maps anatomical [min_deg, max_deg] and neutral ref_deg into generalized
// coordinate bounds (q_lo, q_hi) in radians.
BIKE_HD GpuVec2 gpu_joint_q_range(
    float min_deg,
    float max_deg,
    float ref_deg,
    int sign_flip
) noexcept {
    const float dir = (sign_flip != 0) ? -1.0f : 1.0f;
    const float min_rad = min_deg * kDegToRad;
    const float max_rad = max_deg * kDegToRad;
    const float ref_rad = ref_deg * kDegToRad;
    const float a = (min_rad - ref_rad) / dir;
    const float b = (max_rad - ref_rad) / dir;
    return GpuVec2{.x = gpu_min(a, b), .y = gpu_max(a, b)};
}

// Port of the restoring torque in spindle::soft_edge_scalar from
// native/src/rider/spindle_math.cpp, with smooth boundary damping inside the
// active edge margin. Zero inside the safe interior [q_lo + margin, q_hi - margin].
BIKE_HD float gpu_soft_edge_torque(
    float q_rad,
    float qd_rad_s,
    float q_lo_rad,
    float q_hi_rad,
    float margin_rad,
    float k_edge,
    float c_edge
) noexcept {
    if (q_hi_rad <= q_lo_rad || (k_edge <= 0.0f && c_edge <= 0.0f)) {
        return 0.0f;
    }
    const float k = gpu_max(k_edge, 0.0f);
    const float c = gpu_max(c_edge, 0.0f);
    const float m = gpu_max(margin_rad, 0.0f);
    const float margin = gpu_min(m, 0.5f * (q_hi_rad - q_lo_rad));
    const float left = gpu_max(q_lo_rad + margin - q_rad, 0.0f);
    const float right = gpu_max(q_rad - q_hi_rad + margin, 0.0f);
    float torque = k * (left - right);
    if (left > 0.0f || right > 0.0f) {
        const float penetration = gpu_max(left, right);
        const float ramp = (margin > 1e-6f) ? gpu_min(penetration / margin, 1.0f) : 1.0f;
        torque -= c * ramp * qd_rad_s;
    }
    return torque;
}

// Port of the elastic potential energy in spindle::soft_edge_scalar from
// native/src/rider/spindle_math.cpp. Zero inside [q_lo + margin, q_hi - margin].
BIKE_HD float gpu_soft_edge_potential(
    float q_rad,
    float q_lo_rad,
    float q_hi_rad,
    float margin_rad,
    float k_edge
) noexcept {
    if (q_hi_rad <= q_lo_rad || k_edge <= 0.0f) {
        return 0.0f;
    }
    const float k = gpu_max(k_edge, 0.0f);
    const float m = gpu_max(margin_rad, 0.0f);
    const float margin = gpu_min(m, 0.5f * (q_hi_rad - q_lo_rad));
    const float left = gpu_max(q_lo_rad + margin - q_rad, 0.0f);
    const float right = gpu_max(q_rad - q_hi_rad + margin, 0.0f);
    return 0.5f * k * (left * left + right * right);
}

// Port of Hill-type velocity scaling in spindle::directional_capacity from
// native/src/rider/spindle_math.cpp.
BIKE_HD float gpu_directional_capacity(
    float iso_torque_nm,
    float velocity_rad_s,
    int direction,
    float vmax_rad_s = 12.0f,
    float hill_c = 0.25f,
    float eccentric_ratio = 1.4f
) noexcept {
    const float dir = (direction >= 0) ? 1.0f : -1.0f;
    const float vmax = gpu_max(vmax_rad_s, 1e-6f);
    const float hc = gpu_max(hill_c, 1e-6f);
    const float ecc = gpu_max(eccentric_ratio, 1.0f);
    const float speed = dir * velocity_rad_s / vmax;
    float factor = 0.0f;
    if (speed >= 0.0f) {
        factor = gpu_max(0.0f, (1.0f - speed) / (1.0f + speed / hc));
    } else {
        factor = 1.0f + (ecc - 1.0f) * (-speed) / (1.0f - speed);
    }
    return gpu_max(iso_torque_nm, 0.0f) * factor;
}

// Evaluates the 9-DOF rider ROM soft-edge torques, stored boundary energy, and
// posture PD actuator commands for one world. Every joint gets its envelope torque
// (accumulated into qfrc) and envelope energy. Only joints with non-zero posture gains
// produce ctrl. The legs (slots 0..3) have zero gains and are passive followers of the
// crank through the connect_foot_* constraints, so their ctrl stays exactly 0.0f. The crank
// phase is not read here: crank effort is applied only by the drivetrain kernel, never as a
// second drive path through the legs.
BIKE_HD GpuRiderOutput gpu_compute_rider_step(
    const BIKE_THREAD GpuRiderParams &rp,
    const BIKE_THREAD GpuModelIndices &idx,
    const BIKE_THREAD float *qpos_world,
    const BIKE_THREAD float *qvel_world
) noexcept {
    GpuRiderOutput out{};
    if (qpos_world == nullptr || qvel_world == nullptr) {
        return out;
    }

    float total_energy = 0.0f;
    for (int j = 0; j < kNumRiderJoints; ++j) {
        const int qpos_id = idx.rider_qpos[j];
        const int dof_id = idx.rider_dof[j];
        if (qpos_id < 0 || dof_id < 0 || qpos_id >= idx.nq || dof_id >= idx.nv) {
            out.envelope_torque_nm[j] = 0.0f;
            out.actuator_ctrl_nm[j] = 0.0f;
            continue;
        }

        const BIKE_THREAD GpuJointEnvelopeSpec &spec = rp.joints[j];
        const float q = qpos_world[qpos_id];
        const float qd = qvel_world[dof_id];

        const GpuVec2 q_range = gpu_joint_q_range(
            spec.min_deg, spec.max_deg, spec.ref_deg, spec.sign_flip);

        const float env_tau = gpu_soft_edge_torque(
            q, qd, q_range.x, q_range.y,
            spec.margin_rad, spec.k_edge_nm_rad, spec.c_edge_nm_s_rad);
        const float env_e = gpu_soft_edge_potential(
            q, q_range.x, q_range.y,
            spec.margin_rad, spec.k_edge_nm_rad);

        out.envelope_torque_nm[j] = gpu_isfinite(env_tau) ? env_tau : 0.0f;
        total_energy += gpu_isfinite(env_e) ? env_e : 0.0f;

        if (rp.rider_enabled != 0) {
            // Legs (kp = kd = 0) produce exactly 0 here; their target_qpos_rad is unused.
            const float pos_err = rp.target_qpos_rad[j] - q;
            float ctrl_tau = rp.kp_posture[j] * pos_err - rp.kd_posture[j] * qd;

            const float max_pos = (spec.sign_flip != 0)
                ? gpu_max(spec.max_flex_torque_nm, 0.0f)
                : gpu_max(spec.max_ext_torque_nm, 0.0f);
            const float max_neg = (spec.sign_flip != 0)
                ? gpu_max(spec.max_ext_torque_nm, 0.0f)
                : gpu_max(spec.max_flex_torque_nm, 0.0f);

            ctrl_tau = gpu_clamp(ctrl_tau, -max_neg, max_pos);
            // Zero, including -0.0f from a zero-gain leg with negative error, is written as +0.0f.
            out.actuator_ctrl_nm[j] = (gpu_isfinite(ctrl_tau) && ctrl_tau != 0.0f) ? ctrl_tau : 0.0f;
        } else {
            out.actuator_ctrl_nm[j] = 0.0f;
        }
    }
    out.envelope_energy_j = total_energy;
    return out;
}

// Computes the rider step and writes:
//   - envelope_torque_nm[j] into qfrc_world[idx.rider_dof[j]] via += accumulation
//   - clamped actuator_ctrl_nm[j] into ctrl_world[idx.rider_ctrl[j]]
BIKE_HD void gpu_write_rider_qfrc_and_ctrl(
    const BIKE_THREAD GpuRiderParams &rp,
    const BIKE_THREAD GpuModelIndices &idx,
    const BIKE_THREAD float *qpos_world,
    const BIKE_THREAD float *qvel_world,
    BIKE_THREAD float *qfrc_world,
    BIKE_THREAD float *ctrl_world
) noexcept {
    if (qpos_world == nullptr || qvel_world == nullptr) {
        return;
    }
    const GpuRiderOutput out = gpu_compute_rider_step(rp, idx, qpos_world, qvel_world);
    for (int j = 0; j < kNumRiderJoints; ++j) {
        if (qfrc_world != nullptr) {
            const int dof = idx.rider_dof[j];
            if (dof >= 0 && dof < idx.nv) {
                qfrc_world[dof] += out.envelope_torque_nm[j];
            }
        }
        if (ctrl_world != nullptr) {
            const int ctrl_id = idx.rider_ctrl[j];
            if (ctrl_id >= 0 && ctrl_id < idx.nu) {
                ctrl_world[ctrl_id] = out.actuator_ctrl_nm[j];
            }
        }
    }
}

} // namespace bike::gpu

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
