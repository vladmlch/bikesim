// native/src/gpu/rider_kernel.hpp — Allocation-free __host__ __device__
// rider joint ROM envelope and 6-DOF posture + pedaling control kernel.
#pragma once

#include "common.hpp"

namespace bike::gpu {

inline constexpr int kNumRiderJoints = 6;
inline constexpr int kNumPedalingJoints = 3;
inline constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;

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

// Complete 6-DOF rider parameter table:
//   joints[0..5] = {hip, knee, ankle, waist_pitch, shoulder_pitch, elbow}
struct GpuRiderParams {
    GpuJointEnvelopeSpec joints[6] = {
        // 0: hip
        {-120.0f, 15.0f, 0.0f, 0, 0.0872664626f, 220.0f, 8.0f, 280.0f, 140.0f},
        // 1: knee
        {-135.0f, 0.0f, 0.0f, 0, 0.0872664626f, 240.0f, 8.0f, 300.0f, 130.0f},
        // 2: ankle
        {-35.0f, 45.0f, 0.0f, 0, 0.0698131701f, 150.0f, 4.0f, 160.0f, 80.0f},
        // 3: waist_pitch
        {-45.0f, 75.0f, 0.0f, 0, 0.0872664626f, 200.0f, 6.0f, 220.0f, 180.0f},
        // 4: shoulder_pitch
        {-60.0f, 140.0f, 0.0f, 0, 0.0872664626f, 140.0f, 4.0f, 140.0f, 120.0f},
        // 5: elbow
        {0.0f, 145.0f, 0.0f, 0, 0.0872664626f, 120.0f, 3.5f, 110.0f, 110.0f},
    };
    float kp_posture[6] = {120.0f, 140.0f, 80.0f, 180.0f, 95.0f, 75.0f};
    float kd_posture[6] = {12.0f, 14.0f, 8.0f, 18.0f, 9.5f, 7.5f};
    float target_qpos_rad[6] = {-0.75f, -0.90f, 0.10f, 0.35f, 0.85f, 0.55f};
    float pedaling_amp_nm[3] = {45.0f, 55.0f, 18.0f};
    int rider_enabled = 1;
};

// Output of a single rider evaluation step.
struct GpuRiderOutput {
    float envelope_torque_nm[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float actuator_ctrl_nm[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
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
    return ::atan2f(gpu_sin(value), gpu_cos(value));
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
    return GpuVec2{gpu_min(a, b), gpu_max(a, b)};
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

// Evaluates the 6-DOF rider ROM soft-edge torques, stored boundary energy,
// and posture PD + phase-locked pedaling actuator commands for one world.
BIKE_HD GpuRiderOutput gpu_compute_rider_step(
    const GpuRiderParams &rp,
    const GpuModelIndices &idx,
    const float *qpos_world,
    const float *qvel_world
) noexcept {
    GpuRiderOutput out{};
    if (qpos_world == nullptr || qvel_world == nullptr) {
        return out;
    }

    const float crank_phase =
        (idx.crank_spin_qpos >= 0 && idx.crank_spin_qpos < idx.nq)
            ? qpos_world[idx.crank_spin_qpos]
            : 0.0f;
    const float sin_phase = gpu_sin(crank_phase);
    const float cos_phase = gpu_cos(crank_phase);

    float total_energy = 0.0f;
    for (int j = 0; j < kNumRiderJoints; ++j) {
        const GpuJointEnvelopeSpec &spec = rp.joints[j];
        const int qpos_adr = idx.rider_qpos[j];
        const int dof_adr = idx.rider_dof[j];
        const float q = (qpos_adr >= 0 && qpos_adr < idx.nq) ? qpos_world[qpos_adr] : 0.0f;
        const float qd = (dof_adr >= 0 && dof_adr < idx.nv) ? qvel_world[dof_adr] : 0.0f;

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
            const float pos_err = rp.target_qpos_rad[j] - q;
            float ctrl_tau = rp.kp_posture[j] * pos_err - rp.kd_posture[j] * qd;

            if (j == 0) {
                ctrl_tau += rp.pedaling_amp_nm[0] * sin_phase;
            } else if (j == 1) {
                ctrl_tau += rp.pedaling_amp_nm[1] * cos_phase;
            } else if (j == 2) {
                ctrl_tau += rp.pedaling_amp_nm[2] * sin_phase;
            }

            const float max_pos = (spec.sign_flip != 0)
                ? gpu_max(spec.max_flex_torque_nm, 0.0f)
                : gpu_max(spec.max_ext_torque_nm, 0.0f);
            const float max_neg = (spec.sign_flip != 0)
                ? gpu_max(spec.max_ext_torque_nm, 0.0f)
                : gpu_max(spec.max_flex_torque_nm, 0.0f);

            ctrl_tau = gpu_clamp(ctrl_tau, -max_neg, max_pos);
            out.actuator_ctrl_nm[j] = gpu_isfinite(ctrl_tau) ? ctrl_tau : 0.0f;
        } else {
            out.actuator_ctrl_nm[j] = 0.0f;
        }
    }
    out.envelope_energy_j = total_energy;
    return out;
}

// Computes the rider step and writes:
//   - envelope_torque_nm[j] into qfrc_world[idx.rider_dof[j]]
//   - clamped actuator_ctrl_nm[j] into ctrl_world[idx.rider_ctrl[j]]
BIKE_HD void gpu_write_rider_qfrc_and_ctrl(
    const GpuRiderParams &rp,
    const GpuModelIndices &idx,
    const float *qpos_world,
    const float *qvel_world,
    float *qfrc_world,
    float *ctrl_world
) noexcept {
    if (qpos_world == nullptr || qvel_world == nullptr) {
        return;
    }
    const GpuRiderOutput out = gpu_compute_rider_step(rp, idx, qpos_world, qvel_world);
    for (int j = 0; j < kNumRiderJoints; ++j) {
        if (qfrc_world != nullptr) {
            const int dof = idx.rider_dof[j];
            if (dof >= 0 && dof < idx.nv) {
                qfrc_world[dof] = out.envelope_torque_nm[j];
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
