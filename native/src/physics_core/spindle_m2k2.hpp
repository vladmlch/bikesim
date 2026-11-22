// native/src/physics_core/spindle_m2k2.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL 2.4..3.1+)
// closed-form analytical 2-link leg kinematics, subtree_com leg weights, symmetric CSR M*v,
// anatomical strength curves & power-budget bisection, upper-body IK/PD, and 4-pass M2_K2
// Newton pedal-press solver for SpindleController.
//
// Strict tri-target hygiene:
// - Zero `double` literals (always `BIKE_REAL(Real, ...)`).
// - Zero `std::` headers/calls inside BIKE_GPU_SHARED_CODE_BEGIN..END.
// - Zero `if constexpr` (MSL 2.4 is C++14; type dispatch uses function overloading).
// - Zero heap allocations, zero lambdas, zero member methods on POD structs.
// - All reference parameters carry explicit BIKE_REF / BIKE_CREF / BIKE_THREAD.
// - Distinct template pointer types for every pointer parameter so `constant`, `device`,
//   and `thread` address spaces never alias the same deduced type on MSL < 3.2.
#pragma once

#include "common.hpp"
#include "gpu_math.hpp"
#include "rider_intent.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: POD structs and templated pointers match the CUDA/MSL ABI without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

BIKE_CONSTEXPR i32 kNumSpindleJoints = 9;
BIKE_CONSTEXPR i32 kNumUpperSpindleJoints = 5;
BIKE_CONSTEXPR i32 kMaxStrengthKnots = 8;
BIKE_CONSTEXPR i32 kMaxNsite = 64;
BIKE_CONSTEXPR i32 kSpindlePowerBisectIters = 60;
BIKE_CONSTEXPR i32 kSpindleLeanBisectIters = 21;

// Canonical joint slot indices matching SpindleController::joints_ on welded.xml:
//   0: rider_torso_hinge
//   1: rider_shoulder_left,  2: rider_elbow_left
//   3: rider_shoulder_right, 4: rider_elbow_right
//   5: rider_hip_front,      6: rider_knee_front
//   7: rider_hip_rear,       8: rider_knee_rear
BIKE_CONSTEXPR i32 kSpindleSlotTorso = 0;
BIKE_CONSTEXPR i32 kSpindleSlotShoulderLeft = 1;
BIKE_CONSTEXPR i32 kSpindleSlotElbowLeft = 2;
BIKE_CONSTEXPR i32 kSpindleSlotShoulderRight = 3;
BIKE_CONSTEXPR i32 kSpindleSlotElbowRight = 4;
BIKE_CONSTEXPR i32 kSpindleSlotHipFront = 5;
BIKE_CONSTEXPR i32 kSpindleSlotKneeFront = 6;
BIKE_CONSTEXPR i32 kSpindleSlotHipRear = 7;
BIKE_CONSTEXPR i32 kSpindleSlotKneeRear = 8;

BIKE_HD float spindle_copysign(float mag, float sign_src) noexcept {
    const u32 m_bits = pf_f32_to_u32(mag) & 0x7fffffffu;
    const u32 s_bits = pf_f32_to_u32(sign_src) & 0x80000000u;
    return pf_u32_to_f32(m_bits | s_bits);
}

#if !defined(__METAL_VERSION__)
BIKE_HD double spindle_copysign(double mag, double sign_src) noexcept {
    const double amag = (mag < static_cast<double>(0)) ? -mag : mag;
    return (sign_src < static_cast<double>(0)) ? -amag : amag;
}
#endif

struct SpindlePortableTrig {
    BIKE_HD static float flush_subnorm(float x) noexcept {
        return PortableTrig::flush_subnorm(x);
    }
#if !defined(__METAL_VERSION__)
    BIKE_HD static double flush_subnorm(double x) noexcept {
        return PortableTrig::flush_subnorm(x);
    }
#endif

    template <class Real, class TK>
    BIKE_HD static Real sin_val(Real x) noexcept {
        return PortableTrig::sin_val<Real, TK>(x);
    }

    template <class Real, class TK>
    BIKE_HD static Real cos_val(Real x) noexcept {
        return PortableTrig::cos_val<Real, TK>(x);
    }

    template <class Real, class TK>
    BIKE_HD static Real atan_val(Real x) noexcept {
        return PortableTrig::atan_val<Real, TK>(x);
    }

    template <class Real, class TK>
    BIKE_HD static Real atan2_val(Real y, Real x) noexcept {
        return PortableTrig::atan2_val<Real, TK>(y, x);
    }

    template <class Real, class TK>
    BIKE_HD static Real acos_val(Real x_in) noexcept {
        const Real x = flush_subnorm(x_in);
        const Real c = TK::clamp(x, BIKE_REAL(Real, -1.0), BIKE_REAL(Real, 1.0));
        const Real s_sq = TK::max(
            BIKE_REAL(Real, 0.0),
            (BIKE_REAL(Real, 1.0) - c) * (BIKE_REAL(Real, 1.0) + c));
        return PortableTrig::atan2_val<Real, TK>(TK::sqrt(s_sq), c);
    }
};

template <class Real>
struct TorqueCurveSpec {
    Real angles_rad[kMaxStrengthKnots]{};
    Real torques_nm[kMaxStrengthKnots]{};
    Real vmax_rad_s = BIKE_REAL(Real, 10.0);
    Real hill_c = BIKE_REAL(Real, 0.25);
    Real eccentric_ratio = BIKE_REAL(Real, 1.4);
    i32 knot_count = 0;
};

template <class Real>
struct JointStrengthSpec {
    TorqueCurveSpec<Real> positive{};
    TorqueCurveSpec<Real> negative{};
    Real neutral_anatomical_rad = BIKE_REAL(Real, 0.0);
    i32 direction = 1;
};

template <class Real>
struct SpindleIkResult {
    Real angle0 = BIKE_REAL(Real, 0.0);
    Real angle1 = BIKE_REAL(Real, 0.0);
    u8 saturated = 0;
    u8 pad[3]{};
};

template <class Real>
struct LegKinematicsResult {
    Real jac[2]{};
    Real djac[2]{};
    Real gamma_z[2]{};
    Real gamma_x[2]{};
    Real spindle_xz[2]{};
    u8 valid = 0;
    u8 pad[3]{};
};

template <class Real>
struct PressStateCore {
    Real f_passive[2]{};
    Real max_total_press_n = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct SpindleCommand {
    Real mean_crank_torque_nm = BIKE_REAL(Real, 0.0);
    Real torso_lean_rad = BIKE_REAL(Real, 0.0);
    Real crank_target_rate_rad_s = BIKE_REAL(Real, 0.0);
    Real weld_crank_torque_nm = BIKE_REAL(Real, 0.0);
    u8 has_weld_crank_torque = 0;
    u8 enabled = 1;
    u8 crank_reposition = 0;
    u8 steady_state = 0;
    u8 advance = 1;
    u8 pad[3]{};
};

template <class Real>
struct SpindleM2K2Params {
    i32 nq = 27;
    i32 nv = 27;
    i32 nu = 12;
    i32 nbody = 29;
    i32 njnt = 21;
    i32 nsite = 8;

    i32 frame_body = -1;
    i32 pelvis_body = -1;
    i32 torso_body = -1;
    i32 steer_body = -1;
    i32 crank_body = -1;
    i32 pedal_bodies[2]{-1, -1};
    i32 upper_arm_bodies[2]{-1, -1};
    i32 forearm_bodies[2]{-1, -1};
    i32 hip_bodies[2]{-1, -1};
    i32 knee_bodies[2]{-1, -1};

    i32 crank_joint = -1;
    i32 torso_joint = -1;
    i32 frame_pitch_joint = -1;
    i32 rider_pitch_joint = -1;
    i32 hip_joints[2]{-1, -1};
    i32 knee_joints[2]{-1, -1};
    i32 grip_sites[2]{-1, -1};

    i32 frame_root_x_dof = -1;
    i32 frame_root_z_dof = -1;
    i32 frame_pitch_dof = -1;
    i32 rider_root_x_dof = -1;
    i32 rider_root_z_dof = -1;
    i32 rider_pitch_dof = -1;
    i32 crank_spin_dof = -1;

    i32 joint_qpos_adr[kNumSpindleJoints]{};
    i32 joint_dof_adr[kNumSpindleJoints]{};
    i32 joint_actuator_id[kNumSpindleJoints]{};
    Real joint_range_lo[kNumSpindleJoints]{};
    Real joint_range_hi[kNumSpindleJoints]{};
    u8 joint_has_range[kNumSpindleJoints]{};
    u8 pad_range[3]{};

    JointStrengthSpec<Real> strength[kNumSpindleJoints]{};

    Real gravity[3]{BIKE_REAL(Real, 0.0), BIKE_REAL(Real, 0.0), BIKE_REAL(Real, -9.81)};
    Real hip_subtreemass[2]{BIKE_REAL(Real, 0.0), BIKE_REAL(Real, 0.0)};
    Real knee_subtreemass[2]{BIKE_REAL(Real, 0.0), BIKE_REAL(Real, 0.0)};

    Real thigh_m[2]{BIKE_REAL(Real, 0.0), BIKE_REAL(Real, 0.0)};
    Real link_m[2]{BIKE_REAL(Real, 0.0), BIKE_REAL(Real, 0.0)};
    Real thigh_angle0_rad[2]{BIKE_REAL(Real, 0.0), BIKE_REAL(Real, 0.0)};
    Real link_angle0_rad[2]{BIKE_REAL(Real, 0.0), BIKE_REAL(Real, 0.0)};
    Real knee_sign[2]{BIKE_REAL(Real, 1.0), BIKE_REAL(Real, 1.0)};
    Real side_sign[2]{BIKE_REAL(Real, 1.0), BIKE_REAL(Real, -1.0)};
    Real crank_length_m = BIKE_REAL(Real, 0.165);

    Real arm_upper_m = BIKE_REAL(Real, 0.0);
    Real arm_lower_m = BIKE_REAL(Real, 0.0);
    Real arm_upper_angle0 = BIKE_REAL(Real, 0.0);
    Real arm_lower_angle0 = BIKE_REAL(Real, 0.0);
    i32 arm_branch = 1;
    Real neutral_torso_q = BIKE_REAL(Real, 0.0);
    Real weld_grip_offset[2][3]{};
    Real pose_grip[3]{};
    Real lean_limit_rad = BIKE_REAL(Real, 0.8);

    Real joint_kp_nm_rad = BIKE_REAL(Real, 600.0);
    Real joint_kd_nms_rad = BIKE_REAL(Real, 15.0);
    Real leg_active_damping_nms_rad = BIKE_REAL(Real, 35.0);
    Real joint_limit_nm = BIKE_REAL(Real, 100.0);
    Real joint_speed_limit_rad_s = BIKE_REAL(Real, 20.0);
    Real joint_power_limit_w = BIKE_REAL(Real, 250.0);
    Real active_positive_power_limit_w = BIKE_REAL(Real, 450.0);
    Real pedal_torque_ripple = BIKE_REAL(Real, 0.5);
    Real return_foot_preload_n = BIKE_REAL(Real, 40.0);
    Real pedal_press_margin_n = BIKE_REAL(Real, 25.0);
    Real weld_servo_gain = BIKE_REAL(Real, 0.05);
    Real weld_servo_limit_nm = BIKE_REAL(Real, 25.0);
    Real reposition_damping_nms_rad = BIKE_REAL(Real, 12.0);
    Real reposition_torque_limit_nm = BIKE_REAL(Real, 60.0);

    Real ik_reach_eps = BIKE_REAL(Real, 1e-6);
    Real ik_dir_eps = BIKE_REAL(Real, 1e-12);
    Real det_eps = BIKE_REAL(Real, 1e-6);
    Real newton_eps = BIKE_REAL(Real, 0.1);
    Real conv_eps = BIKE_REAL(Real, 1e-6);
    i32 control_period_steps = 1;

    u8 has_lean_limit = 1;
    u8 has_active_positive_power_limit = 1;
    u8 has_strength = 1;
    u8 grip_is_spring = 0;
    u8 neutral_torso_saturated = 0;
    u8 pad_flags[3]{};
};

template <class ParamsPtr>
BIKE_HD bool validate_spindle_m2k2_params(ParamsPtr params) noexcept {
    if (params == nullptr) {
        return false;
    }
    if (params->nq <= 0 || params->nq > kMaxNq ||
        params->nv <= 0 || params->nv > kMaxNv ||
        params->nu < 0 || params->nu > kMaxNu ||
        params->nbody <= 0 || params->nbody > kMaxNbody ||
        params->njnt <= 0 || params->njnt > kMaxNjnt ||
        params->nsite < 0 || params->nsite > kMaxNsite) {
        return false;
    }
    const i32 nb = params->nbody;
    const i32 nj = params->njnt;
    const i32 ns = params->nsite;
    const i32 nq = params->nq;
    const i32 nv = params->nv;
    const i32 nu = params->nu;

    if (params->frame_body < 0 || params->frame_body >= nb ||
        params->pelvis_body < 0 || params->pelvis_body >= nb ||
        params->torso_body < 0 || params->torso_body >= nb ||
        params->steer_body < 0 || params->steer_body >= nb ||
        params->crank_body < 0 || params->crank_body >= nb) {
        return false;
    }
    for (i32 s = 0; s < 2; ++s) {
        if (params->pedal_bodies[s] < 0 || params->pedal_bodies[s] >= nb ||
            params->upper_arm_bodies[s] < 0 || params->upper_arm_bodies[s] >= nb ||
            params->forearm_bodies[s] < 0 || params->forearm_bodies[s] >= nb ||
            params->hip_bodies[s] < 0 || params->hip_bodies[s] >= nb ||
            params->knee_bodies[s] < 0 || params->knee_bodies[s] >= nb ||
            params->hip_joints[s] < 0 || params->hip_joints[s] >= nj ||
            params->knee_joints[s] < 0 || params->knee_joints[s] >= nj ||
            params->grip_sites[s] < 0 || params->grip_sites[s] >= ns) {
            return false;
        }
    }
    if (params->crank_joint < 0 || params->crank_joint >= nj ||
        params->torso_joint < 0 || params->torso_joint >= nj ||
        params->frame_pitch_joint < 0 || params->frame_pitch_joint >= nj ||
        params->rider_pitch_joint < 0 || params->rider_pitch_joint >= nj) {
        return false;
    }
    if (params->frame_pitch_dof < 0 || params->frame_pitch_dof >= nv ||
        params->rider_pitch_dof < 0 || params->rider_pitch_dof >= nv ||
        params->crank_spin_dof < 0 || params->crank_spin_dof >= nv) {
        return false;
    }
    if ((params->frame_root_x_dof != -1 && (params->frame_root_x_dof < 0 || params->frame_root_x_dof >= nv)) ||
        (params->frame_root_z_dof != -1 && (params->frame_root_z_dof < 0 || params->frame_root_z_dof >= nv)) ||
        (params->rider_root_x_dof != -1 && (params->rider_root_x_dof < 0 || params->rider_root_x_dof >= nv)) ||
        (params->rider_root_z_dof != -1 && (params->rider_root_z_dof < 0 || params->rider_root_z_dof >= nv))) {
        return false;
    }
    for (i32 i = 0; i < kNumSpindleJoints; ++i) {
        if (params->joint_qpos_adr[i] < 0 || params->joint_qpos_adr[i] >= nq ||
            params->joint_dof_adr[i] < 0 || params->joint_dof_adr[i] >= nv) {
            return false;
        }
        const i32 aid = params->joint_actuator_id[i];
        if (aid != -1 && (aid < 0 || aid >= nu)) {
            return false;
        }
    }
    return true;
}

template <class Real>
struct SpindleM2K2State {
    Real coast_hold_nm = BIKE_REAL(Real, 0.0);
    Real held_torques[kNumSpindleJoints]{};
    PressStateCore<Real> prev_ps{};
    u8 enabled = 1;
    u8 command_enabled = 1;
    u8 has_held_torques = 0;
    u8 has_prev_ps = 0;
};

template <class Real>
struct SpindleM2K2StepScratch {
    Real frame_r[9]{};
    Real crank_world[3]{};
    Real frame_pitch_anchor[3]{};
    Real rider_pitch_anchor[3]{};
    Real hip_world[2][3]{};
    Real delta_hip[2][3]{};

    Real phase = BIKE_REAL(Real, 0.0);
    Real rate = BIKE_REAL(Real, 0.0);
    Real pelvis_pitch = BIKE_REAL(Real, 0.0);
    Real shares[2]{BIKE_REAL(Real, 0.5), BIKE_REAL(Real, 0.5)};

    Real jacobians[2][2]{};
    Real djacs[2][2]{};
    Real gammas[2][2]{};
    Real gammas_x[2][2]{};
    Real f_weight[2]{};
    Real qdot_leg_kin[2][2]{};
    Real leg_cap_pos[2][2]{};
    Real leg_cap_neg[2][2]{};
    Real press_per_nm[2]{};

    Real upper_req[kNumUpperSpindleJoints]{};
    Real arm_gamma_sh[2]{};
    Real arm_gamma_el[2]{};
    Real arm_tau_sh[2]{};
    Real arm_tau_el[2]{};
    u8 arm_valid[2]{};
    u8 has_jacobian[2]{};

    Real incoming_coast_hold_nm = BIKE_REAL(Real, 0.0);
    Real mean_crank_torque_nm = BIKE_REAL(Real, 0.0);
    Real crank_target_rate_rad_s = BIKE_REAL(Real, 0.0);
    Real weld_crank_torque_nm = BIKE_REAL(Real, 0.0);
    u8 has_weld_crank_torque = 0;
    u8 command_enabled = 1;
    u8 crank_reposition = 0;
    u8 steady_state = 0;
    u8 advance = 1;
    u8 active_newton = 0;
    u8 converged = 0;
    u8 perturbed0 = 0;
    u8 perturbed1 = 0;
    u8 adaptive_mode = 0;
    u8 torso_saturated = 0;
    u8 arms_saturated = 0;
    u8 pad_flags[4]{};

    PressStateCore<Real> ps0{};
    PressStateCore<Real> P{};
    PressStateCore<Real> P0{};
    PressStateCore<Real> ps_trial{};
    Real base_leg_req[4]{};
    Real base_leg_prio[4]{};
    Real crank_task_nm = BIKE_REAL(Real, 0.0);
    Real last_req[kNumSpindleJoints]{};
    Real last_exc[kNumSpindleJoints]{};
};

namespace detail {

template <class Real, class TK, class Trig>
BIKE_HD Real spindle_wrap_angle(Real angle_rad) noexcept {
    return Trig::template atan2_val<Real, TK>(
        Trig::template sin_val<Real, TK>(angle_rad),
        Trig::template cos_val<Real, TK>(angle_rad));
}

template <class Real, class PtrMat, class PtrVec>
BIKE_HD void spindle_mat_vec(PtrMat a, PtrVec x, BIKE_THREAD Real (&out)[3]) noexcept {
    out[0] = a[0] * x[0] + a[1] * x[1] + a[2] * x[2];
    out[1] = a[3] * x[0] + a[4] * x[1] + a[5] * x[2];
    out[2] = a[6] * x[0] + a[7] * x[1] + a[8] * x[2];
}

template <class Real, class PtrMat, class PtrVec>
BIKE_HD void spindle_mat_t_vec_xz(PtrMat a, PtrVec x, BIKE_THREAD Real (&out)[2]) noexcept {
    out[0] = a[0] * x[0] + a[3] * x[1] + a[6] * x[2];
    out[1] = a[2] * x[0] + a[5] * x[1] + a[8] * x[2];
}

template <class Real, class PtrA, class PtrB>
BIKE_HD void spindle_mat_mul(PtrA a, PtrB b, BIKE_THREAD Real (&out)[9]) noexcept {
    for (i32 r = 0; r < 3; ++r) {
        const i32 rb = r * 3;
        for (i32 c = 0; c < 3; ++c) {
            out[rb + c] = a[rb + 0] * b[0 + c] + a[rb + 1] * b[3 + c] + a[rb + 2] * b[6 + c];
        }
    }
}

template <class Real, class TK, class Trig, class CurvePtr>
BIKE_HD Real eval_torque_curve(
    CurvePtr curve,
    Real angle_rad,
    Real velocity_rad_s,
    i32 direction
) noexcept {
    const i32 n = curve->knot_count;
    if (n <= 0) {
        return BIKE_REAL(Real, 0.0);
    }
    const Real a = TK::clamp(angle_rad, curve->angles_rad[0], curve->angles_rad[n - 1]);
    Real iso = curve->torques_nm[0];
    if (n >= 2) {
        if (a <= curve->angles_rad[0]) {
            iso = curve->torques_nm[0];
        } else if (a >= curve->angles_rad[n - 1]) {
            iso = curve->torques_nm[n - 1];
        } else {
            i32 hi = 1;
            while (hi + 1 < n && a >= curve->angles_rad[hi]) {
                ++hi;
            }
            const Real x0 = curve->angles_rad[hi - 1];
            const Real x1 = curve->angles_rad[hi];
            const Real y0 = curve->torques_nm[hi - 1];
            const Real y1 = curve->torques_nm[hi];
            const Real slope = (y1 - y0) / (x1 - x0);
            iso = TK::interp_segment(slope, a, x0, y0, x1, y1);
        }
    }
    const Real sp = Trig::flush_subnorm(
        static_cast<Real>(direction) * Trig::flush_subnorm(velocity_rad_s) / curve->vmax_rad_s);
    Real fac = BIKE_REAL(Real, 0.0);
    if (sp >= BIKE_REAL(Real, 0.0)) {
        fac = TK::max(
            BIKE_REAL(Real, 0.0),
            (BIKE_REAL(Real, 1.0) - sp) / (BIKE_REAL(Real, 1.0) + sp / curve->hill_c));
    } else {
        fac = BIKE_REAL(Real, 1.0) +
              (curve->eccentric_ratio - BIKE_REAL(Real, 1.0)) * (-sp) / (BIKE_REAL(Real, 1.0) - sp);
    }
    return Trig::flush_subnorm(iso * fac);
}

template <class Real, class TK, class Trig, class ParamsPtr>
BIKE_HD Real spindle_directional_limit(
    ParamsPtr params,
    i32 slot,
    Real q_in,
    Real qd_in,
    Real sign
) noexcept {
    if (params->has_strength == 0) {
        return params->joint_limit_nm;
    }
    const Real q = Trig::flush_subnorm(q_in);
    const Real qd = Trig::flush_subnorm(qd_in);
    const Real val = params->strength[slot].neutral_anatomical_rad +
                     static_cast<Real>(params->strength[slot].direction) * q;
    const Real ang = spindle_wrap_angle<Real, TK, Trig>(val);
    if (sign >= BIKE_REAL(Real, 0.0)) {
        return eval_torque_curve<Real, TK, Trig>(&params->strength[slot].positive, ang, qd, 1);
    }
    return eval_torque_curve<Real, TK, Trig>(&params->strength[slot].negative, ang, qd, -1);
}

template <class Real, class TK, class Trig, class ParamsPtr>
BIKE_HD Real spindle_usable_slot_capacity(
    ParamsPtr params,
    i32 slot,
    Real q_in,
    Real qd_in,
    Real sign
) noexcept {
    const Real q = Trig::flush_subnorm(q_in);
    const Real qd = Trig::flush_subnorm(qd_in);
    const Real prod = Trig::flush_subnorm(sign * qd);
    if (TK::abs(qd) >= params->joint_speed_limit_rad_s && prod > BIKE_REAL(Real, 0.0)) {
        return BIKE_REAL(Real, 0.0);
    }
    return spindle_directional_limit<Real, TK, Trig>(params, slot, q, qd, sign);
}

template <class Real, class TK>
BIKE_HD void split_joint_torques_2(
    Real tau_leg_nm,
    const BIKE_THREAD Real (&jac)[2],
    const BIKE_THREAD Real (&cap_pos)[2],
    const BIKE_THREAD Real (&cap_neg)[2],
    BIKE_THREAD Real (&out_torques)[2]
) noexcept {
    Real limits[2] = {
        TK::max(cap_pos[0], cap_neg[0]),
        TK::max(cap_pos[1], cap_neg[1])
    };
    Real t0 = BIKE_REAL(Real, 0.0);
    Real t1 = BIKE_REAL(Real, 0.0);
    for (i32 iter = 0; iter < 2; ++iter) {
        const Real w0 = limits[0] * limits[0] * jac[0];
        const Real w1 = limits[1] * limits[1] * jac[1];
        const Real denom = jac[0] * w0 + jac[1] * w1;
        if (denom < BIKE_REAL(Real, 1e-9)) {
            out_torques[0] = BIKE_REAL(Real, 0.0);
            out_torques[1] = BIKE_REAL(Real, 0.0);
            return;
        }
        t0 = tau_leg_nm * w0 / denom;
        t1 = tau_leg_nm * w1 / denom;
        limits[0] = (t0 >= BIKE_REAL(Real, 0.0)) ? cap_pos[0] : cap_neg[0];
        limits[1] = (t1 >= BIKE_REAL(Real, 0.0)) ? cap_pos[1] : cap_neg[1];
    }
    Real ts[2] = {t0, t1};
    for (i32 i = 0; i < 2; ++i) {
        if (TK::abs(ts[i]) > limits[i]) {
            ts[i] = TK::clamp(ts[i], -limits[i], limits[i]);
            const i32 o = 1 - i;
            if (TK::abs(jac[o]) > BIKE_REAL(Real, 1e-9)) {
                const Real rem = tau_leg_nm - jac[i] * ts[i];
                ts[o] = rem / jac[o];
                limits[o] = (ts[o] >= BIKE_REAL(Real, 0.0)) ? cap_pos[o] : cap_neg[o];
            }
        }
    }
    out_torques[0] = TK::clamp(ts[0], -limits[0], limits[0]);
    out_torques[1] = TK::clamp(ts[1], -limits[1], limits[1]);
}

template <class Real, class TK>
BIKE_HD void downstroke_joint_torques_2(
    Real tau_leg_nm,
    const BIKE_THREAD Real (&jac)[2],
    const BIKE_THREAD Real (&cap_pos)[2],
    const BIKE_THREAD Real (&cap_neg)[2],
    BIKE_THREAD Real (&out_torques)[2]
) noexcept {
    if (tau_leg_nm <= BIKE_REAL(Real, 0.0)) {
        split_joint_torques_2<Real, TK>(tau_leg_nm, jac, cap_pos, cap_neg, out_torques);
        return;
    }
    const Real c0 = cap_pos[0];
    const Real c1 = cap_neg[1];
    const Real w0 = c0 * c0 * jac[0];
    const Real w1 = c1 * c1 * jac[1];
    const Real denom = jac[0] * w0 + jac[1] * w1;
    if (denom < BIKE_REAL(Real, 1e-9)) {
        out_torques[0] = BIKE_REAL(Real, 0.0);
        out_torques[1] = BIKE_REAL(Real, 0.0);
        return;
    }
    out_torques[0] = (jac[0] > BIKE_REAL(Real, 0.0)) ? TK::min(c0, tau_leg_nm * w0 / denom) : BIKE_REAL(Real, 0.0);
    out_torques[1] = (jac[1] < BIKE_REAL(Real, 0.0)) ? TK::max(-c1, tau_leg_nm * w1 / denom) : BIKE_REAL(Real, 0.0);
}

template <class Real, class TK, class Trig = SpindlePortableTrig>
BIKE_HD void scale_to_power_budget_9(
    const BIKE_THREAD Real (&torques)[kNumSpindleJoints],
    const BIKE_THREAD Real (&speeds)[kNumSpindleJoints],
    Real per_joint_w,
    Real total_w,
    BIKE_THREAD Real (&out_torques)[kNumSpindleJoints]
) noexcept {
    Real pos_sum = BIKE_REAL(Real, 0.0);
    for (i32 i = 0; i < kNumSpindleJoints; ++i) {
        const Real t = Trig::flush_subnorm(torques[i]);
        const Real v = Trig::flush_subnorm(speeds[i]);
        const Real tv = Trig::flush_subnorm(t * v);
        out_torques[i] = (tv > per_joint_w) ? Trig::flush_subnorm(per_joint_w / v) : t;
        const Real ov = Trig::flush_subnorm(out_torques[i] * v);
        pos_sum += TK::max(ov, BIKE_REAL(Real, 0.0));
    }
    if (pos_sum > total_w) {
        const Real sc = total_w / pos_sum;
        for (i32 i = 0; i < kNumSpindleJoints; ++i) {
            const Real v = Trig::flush_subnorm(speeds[i]);
            if (Trig::flush_subnorm(out_torques[i] * v) > BIKE_REAL(Real, 0.0)) {
                out_torques[i] = Trig::flush_subnorm(out_torques[i] * sc);
            }
        }
    }
}

template <class Real, class ParamsPtr>
BIKE_HD Real spindle_read_subtreemass(
    ParamsPtr params,
    i32 s,
    i32 idx_j,
    i32 /*bid*/,
    decltype(nullptr) /*unused*/
) noexcept {
    return (idx_j == 0) ? params->hip_subtreemass[s] : params->knee_subtreemass[s];
}

template <class Real, class ParamsPtr, class PtrSubtreeMass>
BIKE_HD Real spindle_read_subtreemass(
    ParamsPtr params,
    i32 s,
    i32 idx_j,
    i32 bid,
    PtrSubtreeMass body_subtreemass
) noexcept {
    if (body_subtreemass == nullptr) {
        return (idx_j == 0) ? params->hip_subtreemass[s] : params->knee_subtreemass[s];
    }
    return body_subtreemass[bid];
}

template <class Real, class TK, class Trig, class ParamsPtr, class PtrPos, class PtrMat>
BIKE_HD void compute_grips_world(
    ParamsPtr params,
    PtrPos body_xpos,
    PtrMat body_xmat,
    BIKE_THREAD Real (&out_grips)[2][3]
) noexcept {
    if (params->grip_is_spring != 0) {
        const i32 fb9 = params->frame_body * 9;
        const i32 fb3 = params->frame_body * 3;
        for (i32 s = 0; s < 2; ++s) {
            Real local[3] = {
                params->pose_grip[0],
                params->pose_grip[1] + (s == 0 ? BIKE_REAL(Real, -0.10) : BIKE_REAL(Real, 0.10)),
                params->pose_grip[2]
            };
            Real rotated[3]{};
            spindle_mat_vec<Real>(&body_xmat[fb9], local, rotated);
            for (i32 c = 0; c < 3; ++c) {
                out_grips[s][c] = body_xpos[fb3 + c] + rotated[c];
            }
        }
        return;
    }
    const i32 sb9 = params->steer_body * 9;
    const i32 sb3 = params->steer_body * 3;
    for (i32 s = 0; s < 2; ++s) {
        Real local[3] = {
            params->weld_grip_offset[s][0],
            params->weld_grip_offset[s][1],
            params->weld_grip_offset[s][2]
        };
        Real rotated[3]{};
        spindle_mat_vec<Real>(&body_xmat[sb9], local, rotated);
        for (i32 c = 0; c < 3; ++c) {
            out_grips[s][c] = body_xpos[sb3 + c] + rotated[c];
        }
    }
}

} // namespace detail

// ---- 1. Upper-body two-link IK & torso lean limits -------------------------

template <class Real, class TK, class Trig = SpindlePortableTrig>
BIKE_HD SpindleIkResult<Real> two_link_ik(
    Real target_x,
    Real target_z,
    Real upper_m,
    Real lower_m,
    i32 elbow_sign,
    Real reach_eps = BIKE_REAL(Real, 1e-6),
    Real dir_eps = BIKE_REAL(Real, 1e-12)
) noexcept {
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kHalfPi = BIKE_REAL(Real, 1.57079632679489661923);
    const Real distance = TK::hypot2(target_x, target_z);
    const Real diff_ab = TK::abs(upper_m - lower_m);
    const Real lo = diff_ab + reach_eps;
    const Real hi = upper_m + lower_m - reach_eps;
    SpindleIkResult<Real> out{};
    out.saturated = (lo <= distance && distance <= hi) ? 0u : 1u;
    const Real d = TK::clamp(distance, lo, hi);
    const Real direction = (distance > dir_eps)
        ? Trig::template atan2_val<Real, TK>(target_z, target_x)
        : -kHalfPi;
    const Real cosine = TK::clamp(
        (d * d - upper_m * upper_m - lower_m * lower_m) / (BIKE_REAL(Real, 2.0) * upper_m * lower_m),
        BIKE_REAL(Real, -1.0),
        BIKE_REAL(Real, 1.0));
    const Real knee = static_cast<Real>(elbow_sign) * Trig::template acos_val<Real, TK>(cosine);
    const Real hip = direction - Trig::template atan2_val<Real, TK>(
        lower_m * Trig::template sin_val<Real, TK>(knee),
        upper_m + lower_m * Trig::template cos_val<Real, TK>(knee));
    out.angle0 = hip;
    out.angle1 = knee;
    return out;
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class PtrQpos, class PtrPos, class PtrMat, class PtrAnchor>
BIKE_HD Real ik_target_elbow_flexion_rad(
    ParamsPtr params,
    PtrQpos qpos,
    PtrPos body_xpos,
    PtrMat body_xmat,
    PtrAnchor jnt_xanchor,
    Real torso_lean_rad
) noexcept {
    if (!validate_spindle_m2k2_params(params)) {
        return BIKE_REAL(Real, 0.0);
    }
    const i32 pb9 = params->pelvis_body * 9;
    const i32 fb9 = params->frame_body * 9;
    const i32 tb9 = params->torso_body * 9;
    const i32 tj3 = params->torso_joint * 3;
    Real grips[2][3]{};
    detail::compute_grips_world<Real, TK, Trig>(params, body_xpos, body_xmat, grips);

    const Real frame_pitch = Trig::template atan2_val<Real, TK>(
        body_xmat[fb9 + 2], body_xmat[fb9 + 0]);
    const Real pelvis_pitch = Trig::template atan2_val<Real, TK>(
        body_xmat[pb9 + 2], body_xmat[pb9 + 0]);
    const Real torso_q = params->neutral_torso_q + torso_lean_rad +
                         detail::spindle_wrap_angle<Real, TK, Trig>(frame_pitch - pelvis_pitch);
    Real delta = detail::spindle_wrap_angle<Real, TK, Trig>(
        torso_q - Trig::flush_subnorm(qpos[params->joint_qpos_adr[kSpindleSlotTorso]]));
    if (params->grip_is_spring != 0) {
        delta = BIKE_REAL(Real, 0.0);
    }
    const Real cd = Trig::template cos_val<Real, TK>(delta);
    const Real sd = Trig::template sin_val<Real, TK>(delta);
    const Real rot[9] = {
        cd, BIKE_REAL(Real, 0.0), sd,
        BIKE_REAL(Real, 0.0), BIKE_REAL(Real, 1.0), BIKE_REAL(Real, 0.0),
        -sd, BIKE_REAL(Real, 0.0), cd
    };
    Real torso_r[9]{};
    detail::spindle_mat_mul<Real>(rot, &body_xmat[tb9], torso_r);

    Real min_flexion = BIKE_REAL(Real, 1e30);
    for (i32 s = 0; s < 2; ++s) {
        const i32 ua3 = params->upper_arm_bodies[s] * 3;
        Real rel_ua[3] = {
            body_xpos[ua3 + 0] - jnt_xanchor[tj3 + 0],
            body_xpos[ua3 + 1] - jnt_xanchor[tj3 + 1],
            body_xpos[ua3 + 2] - jnt_xanchor[tj3 + 2]
        };
        Real rot_ua[3]{};
        detail::spindle_mat_vec<Real>(rot, rel_ua, rot_ua);
        Real diff[3] = {
            grips[s][0] - (jnt_xanchor[tj3 + 0] + rot_ua[0]),
            grips[s][1] - (jnt_xanchor[tj3 + 1] + rot_ua[1]),
            grips[s][2] - (jnt_xanchor[tj3 + 2] + rot_ua[2])
        };
        Real arm_target[2]{};
        detail::spindle_mat_t_vec_xz<Real>(torso_r, diff, arm_target);
        const SpindleIkResult<Real> ik = two_link_ik<Real, TK, Trig>(
            arm_target[0], arm_target[1],
            params->arm_upper_m, params->arm_lower_m, params->arm_branch,
            params->ik_reach_eps, params->ik_dir_eps);
        const Real flex = TK::abs(ik.angle1);
        if (flex < min_flexion) {
            min_flexion = flex;
        }
    }
    return min_flexion;
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class PtrQpos, class PtrPos, class PtrMat, class PtrAnchor>
BIKE_HD void spindle_m2k2_min_torso_lean_rad(
    ParamsPtr params,
    PtrQpos qpos,
    PtrPos body_xpos,
    PtrMat body_xmat,
    PtrAnchor jnt_xanchor,
    Real min_elbow_flexion_rad,
    BIKE_THREAD u8 &out_has_min_lean,
    BIKE_THREAD Real &out_min_lean_rad
) noexcept {
    out_has_min_lean = 0;
    out_min_lean_rad = BIKE_REAL(Real, 0.0);
    if (!validate_spindle_m2k2_params(params) ||
        !TK::isfinite(min_elbow_flexion_rad) || min_elbow_flexion_rad <= BIKE_REAL(Real, 0.0) ||
        params->grip_is_spring != 0) {
        return;
    }
    Real lo = BIKE_REAL(Real, -0.8);
    Real hi = (params->has_lean_limit != 0) ? params->lean_limit_rad : BIKE_REAL(Real, 0.8);
    if (ik_target_elbow_flexion_rad<Real, TK, Trig>(
            params, qpos, body_xpos, body_xmat, jnt_xanchor, lo) >= min_elbow_flexion_rad) {
        out_has_min_lean = 1;
        out_min_lean_rad = lo;
        return;
    }
    if (ik_target_elbow_flexion_rad<Real, TK, Trig>(
            params, qpos, body_xpos, body_xmat, jnt_xanchor, hi) < min_elbow_flexion_rad) {
        out_has_min_lean = 1;
        out_min_lean_rad = hi;
        return;
    }
    for (i32 iter = 0; iter < kSpindleLeanBisectIters; ++iter) {
        const Real mid = BIKE_REAL(Real, 0.5) * (lo + hi);
        if (ik_target_elbow_flexion_rad<Real, TK, Trig>(
                params, qpos, body_xpos, body_xmat, jnt_xanchor, mid) >= min_elbow_flexion_rad) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    out_has_min_lean = 1;
    out_min_lean_rad = hi;
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class PtrQpos, class PtrPos, class PtrMat, class PtrAnchor,
          class PtrOut>
BIKE_HD void upper_targets_core(
    ParamsPtr params,
    PtrQpos qpos,
    PtrPos body_xpos,
    PtrMat body_xmat,
    PtrAnchor jnt_xanchor,
    Real torso_lean_rad,
    PtrOut out_targets,
    BIKE_THREAD u8 &out_torso_sat,
    BIKE_THREAD u8 &out_arms_sat
) noexcept {
    if (!validate_spindle_m2k2_params(params)) {
        for (i32 k = 0; k < kNumUpperSpindleJoints; ++k) {
            out_targets[k] = BIKE_REAL(Real, 0.0);
        }
        out_torso_sat = 0;
        out_arms_sat = 0;
        return;
    }
    const i32 pb9 = params->pelvis_body * 9;
    const i32 fb9 = params->frame_body * 9;
    const i32 tb9 = params->torso_body * 9;
    const i32 tj3 = params->torso_joint * 3;
    Real grips[2][3]{};
    detail::compute_grips_world<Real, TK, Trig>(params, body_xpos, body_xmat, grips);

    const Real frame_pitch = Trig::template atan2_val<Real, TK>(
        body_xmat[fb9 + 2], body_xmat[fb9 + 0]);
    const Real pelvis_pitch = Trig::template atan2_val<Real, TK>(
        body_xmat[pb9 + 2], body_xmat[pb9 + 0]);
    const Real torso_q = params->neutral_torso_q + torso_lean_rad +
                         detail::spindle_wrap_angle<Real, TK, Trig>(frame_pitch - pelvis_pitch);
    out_targets[0] = torso_q;

    Real delta = detail::spindle_wrap_angle<Real, TK, Trig>(
        torso_q - Trig::flush_subnorm(qpos[params->joint_qpos_adr[kSpindleSlotTorso]]));
    if (params->grip_is_spring != 0) {
        delta = BIKE_REAL(Real, 0.0);
    }
    const Real cd = Trig::template cos_val<Real, TK>(delta);
    const Real sd = Trig::template sin_val<Real, TK>(delta);
    const Real rot[9] = {
        cd, BIKE_REAL(Real, 0.0), sd,
        BIKE_REAL(Real, 0.0), BIKE_REAL(Real, 1.0), BIKE_REAL(Real, 0.0),
        -sd, BIKE_REAL(Real, 0.0), cd
    };
    Real torso_r[9]{};
    detail::spindle_mat_mul<Real>(rot, &body_xmat[tb9], torso_r);

    u8 arm_sat = 0;
    for (i32 s = 0; s < 2; ++s) {
        const i32 ua3 = params->upper_arm_bodies[s] * 3;
        Real rel_ua[3] = {
            body_xpos[ua3 + 0] - jnt_xanchor[tj3 + 0],
            body_xpos[ua3 + 1] - jnt_xanchor[tj3 + 1],
            body_xpos[ua3 + 2] - jnt_xanchor[tj3 + 2]
        };
        Real rot_ua[3]{};
        detail::spindle_mat_vec<Real>(rot, rel_ua, rot_ua);
        Real diff[3] = {
            grips[s][0] - (jnt_xanchor[tj3 + 0] + rot_ua[0]),
            grips[s][1] - (jnt_xanchor[tj3 + 1] + rot_ua[1]),
            grips[s][2] - (jnt_xanchor[tj3 + 2] + rot_ua[2])
        };
        Real arm_target[2]{};
        detail::spindle_mat_t_vec_xz<Real>(torso_r, diff, arm_target);
        const SpindleIkResult<Real> ik = two_link_ik<Real, TK, Trig>(
            arm_target[0], arm_target[1],
            params->arm_upper_m, params->arm_lower_m, params->arm_branch,
            params->ik_reach_eps, params->ik_dir_eps);
        if (ik.saturated != 0) {
            arm_sat = 1;
        }
        out_targets[1 + 2 * s] = params->arm_upper_angle0 - ik.angle0;
        out_targets[2 + 2 * s] = (params->arm_lower_angle0 - params->arm_upper_angle0) - ik.angle1;
    }

    u8 torso_sat = params->neutral_torso_saturated;
    for (i32 k = 0; k < kNumUpperSpindleJoints; ++k) {
        const Real current = Trig::flush_subnorm(qpos[params->joint_qpos_adr[k]]);
        Real t = current + detail::spindle_wrap_angle<Real, TK, Trig>(out_targets[k] - current);
        if (params->joint_has_range[k] != 0) {
            const Real lo = params->joint_range_lo[k];
            const Real hi = params->joint_range_hi[k];
            const Real limited = TK::clamp(t, lo, hi);
            if (limited != t) {
                if (k == 0) {
                    torso_sat = 1;
                } else {
                    arm_sat = 1;
                }
            }
            t = limited;
        }
        out_targets[k] = t;
    }
    out_torso_sat = torso_sat;
    out_arms_sat = arm_sat;
}

// ---- 2. Closed-form analytical 2-link leg loop J, dJ/dphi, gammas ----------

template <class Real, class TK, class Trig = SpindlePortableTrig>
BIKE_HD LegKinematicsResult<Real> analytic_leg_kinematics(
    Real thigh_m,
    Real link_m,
    Real crank_m,
    Real side_sign,
    Real knee_sign,
    Real hip_x,
    Real hip_z,
    Real phase_rad,
    Real det_eps = BIKE_REAL(Real, 1e-6),
    Real reach_eps = BIKE_REAL(Real, 1e-5)
) noexcept {
    LegKinematicsResult<Real> res{};
    const Real c = Trig::template cos_val<Real, TK>(phase_rad);
    const Real s = Trig::template sin_val<Real, TK>(phase_rad);
    const Real sp0 = side_sign * crank_m * c;
    const Real sp1 = -side_sign * crank_m * s;
    const Real dsp0 = -side_sign * crank_m * s;
    const Real dsp1 = -side_sign * crank_m * c;
    const Real ddsp0 = -side_sign * crank_m * c;
    const Real ddsp1 = side_sign * crank_m * s;
    res.spindle_xz[0] = sp0;
    res.spindle_xz[1] = sp1;

    const Real dx = sp0 - hip_x;
    const Real dz = sp1 - hip_z;
    const Real L2 = dx * dx + dz * dz;
    const Real L = TK::sqrt(L2);
    if (L <= reach_eps || L > thigh_m + link_m + reach_eps ||
        L < TK::abs(thigh_m - link_m) - reach_eps) {
        return res;
    }
    const Real hip_cos = TK::clamp(
        (thigh_m * thigh_m + L2 - link_m * link_m) / (BIKE_REAL(Real, 2.0) * thigh_m * L),
        BIKE_REAL(Real, -1.0),
        BIKE_REAL(Real, 1.0));
    const Real pa = Trig::template atan2_val<Real, TK>(-dx, -dz);
    const Real theta = pa - knee_sign * Trig::template acos_val<Real, TK>(hip_cos);

    const Real r1x = -thigh_m * Trig::template sin_val<Real, TK>(theta);
    const Real r1z = -thigh_m * Trig::template cos_val<Real, TK>(theta);
    const Real r2x = dx - r1x;
    const Real r2z = dz - r1z;

    const Real det = -dz * r2x + r2z * dx;
    if (TK::abs(det) <= det_eps) {
        return res;
    }

    res.gamma_x[0] = -r2x / det;
    res.gamma_x[1] = dx / det;
    res.gamma_z[0] = -r2z / det;
    res.gamma_z[1] = dz / det;

    const Real dq0 = (-r2x * dsp0 - r2z * dsp1) / det;
    const Real dq1 = (dx * dsp0 + dz * dsp1) / det;
    res.jac[0] = dq0;
    res.jac[1] = dq1;

    const Real dr1x = r1z * dq0;
    const Real dr1z = -r1x * dq0;
    const Real dr2x = dsp0 - dr1x;
    const Real dr2z = dsp1 - dr1z;
    const Real rhs0 = ddsp0 - (dsp1 * dq0 + dr2z * dq1);
    const Real rhs1 = ddsp1 - (-dsp0 * dq0 - dr2x * dq1);
    res.djac[0] = (-r2x * rhs0 - r2z * rhs1) / det;
    res.djac[1] = (dx * rhs0 + dz * rhs1) / det;
    res.valid = 1;
    return res;
}

template <class Real, class TK, class Trig = SpindlePortableTrig, class PtrOut>
BIKE_HD void leg_joint_q_core(
    Real thigh_m,
    Real link_m,
    Real knee_sign,
    Real thigh_angle0_rad,
    Real link_angle0_rad,
    Real hip_x,
    Real hip_z,
    Real spindle_x,
    Real spindle_z,
    Real pelvis_pitch_rad,
    PtrOut out_q,
    BIKE_THREAD u8 &out_valid,
    Real reach_eps = BIKE_REAL(Real, 1e-5)
) noexcept {
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const Real kPi = BIKE_REAL(Real, 3.14159265358979323846);
    out_q[0] = BIKE_REAL(Real, 0.0);
    out_q[1] = BIKE_REAL(Real, 0.0);
    out_valid = 0;
    const Real dx = spindle_x - hip_x;
    const Real dz = spindle_z - hip_z;
    const Real L = TK::sqrt(dx * dx + dz * dz);
    if (L <= reach_eps || L > thigh_m + link_m + reach_eps ||
        L < TK::abs(thigh_m - link_m) - reach_eps) {
        return;
    }
    const Real knee_cos = TK::clamp(
        (thigh_m * thigh_m + link_m * link_m - L * L) / (BIKE_REAL(Real, 2.0) * thigh_m * link_m),
        BIKE_REAL(Real, -1.0),
        BIKE_REAL(Real, 1.0));
    const Real hip_cos = TK::clamp(
        (thigh_m * thigh_m + L * L - link_m * link_m) / (BIKE_REAL(Real, 2.0) * thigh_m * L),
        BIKE_REAL(Real, -1.0),
        BIKE_REAL(Real, 1.0));
    const Real pa = Trig::template atan2_val<Real, TK>(-dx, -dz);
    const Real thigh_angle = pa - knee_sign * Trig::template acos_val<Real, TK>(hip_cos);
    const Real bend = knee_sign * (kPi - Trig::template acos_val<Real, TK>(knee_cos));
    out_q[0] = detail::spindle_wrap_angle<Real, TK, Trig>(
        thigh_angle - pelvis_pitch_rad - thigh_angle0_rad);
    out_q[1] = detail::spindle_wrap_angle<Real, TK, Trig>(
        bend - (link_angle0_rad - thigh_angle0_rad));
    out_valid = 1;
}

template <class Real, class TK, class Trig = SpindlePortableTrig>
BIKE_HD void leg_shares_core(
    Real phase_rad,
    BIKE_THREAD Real (&out_shares)[2]
) noexcept {
    const Real kHandoverCosine = BIKE_REAL(Real, 0.25);
    const Real downward = Trig::template cos_val<Real, TK>(phase_rad);
    if (TK::abs(downward) <= BIKE_REAL(Real, 1e-12)) {
        out_shares[0] = BIKE_REAL(Real, 0.5);
        out_shares[1] = BIKE_REAL(Real, 0.5);
        return;
    }
    if (downward >= kHandoverCosine) {
        out_shares[0] = BIKE_REAL(Real, 1.0);
        out_shares[1] = BIKE_REAL(Real, 0.0);
        return;
    }
    if (downward <= -kHandoverCosine) {
        out_shares[0] = BIKE_REAL(Real, 0.0);
        out_shares[1] = BIKE_REAL(Real, 1.0);
        return;
    }
    const Real u = downward / kHandoverCosine;
    const Real front = BIKE_REAL(Real, 0.5) +
                       BIKE_REAL(Real, 0.25) * u * (BIKE_REAL(Real, 3.0) - u * u);
    out_shares[0] = front;
    out_shares[1] = BIKE_REAL(Real, 1.0) - front;
}

// ---- 3. Closed-form subtree_com leg_weights_at_pedal ----------------------

template <class Real, class ParamsPtr, class PtrSubtreeMass,
          class PtrSubtreeCom, class PtrXanchor, class PtrXaxis, class PtrOut>
BIKE_HD void leg_weights_at_pedal_core(
    ParamsPtr params,
    PtrSubtreeMass body_subtreemass,
    PtrSubtreeCom subtree_com,
    PtrXanchor xanchor,
    PtrXaxis xaxis,
    Real gamma_z_front0,
    Real gamma_z_front1,
    Real gamma_z_rear0,
    Real gamma_z_rear1,
    PtrOut out_f_weight
) noexcept {
    const Real gx = params->gravity[0];
    const Real gy = params->gravity[1];
    const Real gz = params->gravity[2];
    const Real gz_arr[2][2] = {
        {gamma_z_front0, gamma_z_front1},
        {gamma_z_rear0, gamma_z_rear1}
    };
    for (i32 s = 0; s < 2; ++s) {
        const i32 jids[2] = {params->hip_joints[s], params->knee_joints[s]};
        const i32 bids[2] = {params->hip_bodies[s], params->knee_bodies[s]};
        Real leg_g[2]{};
        for (i32 idx_j = 0; idx_j < 2; ++idx_j) {
            const i32 jid = jids[idx_j];
            const i32 bid = bids[idx_j];
            const Real ms = detail::spindle_read_subtreemass<Real>(
                params, s, idx_j, bid, body_subtreemass);
            const i32 b3 = bid * 3;
            const i32 j3 = jid * 3;
            const Real dx = subtree_com[b3 + 0] - xanchor[j3 + 0];
            const Real dy = subtree_com[b3 + 1] - xanchor[j3 + 1];
            const Real dz = subtree_com[b3 + 2] - xanchor[j3 + 2];
            const Real ax0 = xaxis[j3 + 0];
            const Real ax1 = xaxis[j3 + 1];
            const Real ax2 = xaxis[j3 + 2];
            const Real cr0 = ax1 * dz - ax2 * dy;
            const Real cr1 = ax2 * dx - ax0 * dz;
            const Real cr2 = ax0 * dy - ax1 * dx;
            leg_g[idx_j] = -ms * (gx * cr0 + gy * cr1 + gz * cr2);
        }
        out_f_weight[s] = gz_arr[s][0] * leg_g[0] + gz_arr[s][1] * leg_g[1];
    }
}

// ---- 4. Symmetric sparse CSR & dense M * v matvec ---------------------------

template <class Real, class PtrRowAdr, class PtrRowNnz, class PtrColInd,
          class PtrM, class PtrVec, class PtrOut>
BIKE_HD void mul_m_csr(
    i32 nv,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M,
    PtrVec vec,
    PtrOut out_inertia
) noexcept {
    if (nv <= 0 || nv > kMaxNv) {
        return;
    }
    for (i32 i = 0; i < nv; ++i) {
        out_inertia[i] = BIKE_REAL(Real, 0.0);
    }
    for (i32 i = 0; i < nv; ++i) {
        const i32 adr = M_rowadr[i];
        const i32 nnz = M_rownnz[i];
        if (adr < 0 || nnz <= 0) {
            continue;
        }
        Real s = BIKE_REAL(Real, 0.0);
        const Real vi = vec[i];
        for (i32 k = 0; k < nnz; ++k) {
            const i32 j = M_colind[adr + k];
            if (j < 0 || j > i) {
                continue;
            }
            const Real val = d_M[adr + k];
            s += val * vec[j];
            if (j < i) {
                out_inertia[j] += val * vi;
            }
        }
        out_inertia[i] += s;
    }
}

template <class Real, class PtrM, class PtrVec, class PtrOut>
BIKE_HD void mul_m_csr(
    i32 /*nv*/,
    decltype(nullptr) /*M_rowadr*/,
    decltype(nullptr) /*M_rownnz*/,
    decltype(nullptr) /*M_colind*/,
    PtrM /*d_M*/,
    PtrVec /*vec*/,
    PtrOut /*out_inertia*/
) noexcept {}

template <class Real, class QmPtr, class PtrVec, class PtrOut>
BIKE_HD void mul_m_dense(
    i32 nv,
    i32 qM_row_stride,
    QmPtr d_qM_w,
    PtrVec vec,
    PtrOut out_inertia
) noexcept {
    if (nv <= 0 || nv > kMaxNv || qM_row_stride < nv) {
        return;
    }
    for (i32 i = 0; i < nv; ++i) {
        const i32 row = i * qM_row_stride;
        Real s = BIKE_REAL(Real, 0.0);
        for (i32 j = 0; j < nv; ++j) {
            s += d_qM_w[row + j] * vec[j];
        }
        out_inertia[i] = s;
    }
}

// ---- 5. PressState computation (with generic MulM, CSR d_M, or dense qM) ---

template <class Real, class ParamsPtr, class ScratchPtr, class PtrQacc, class PtrImposed>
BIKE_HD void fill_press_imposed(
    ParamsPtr params,
    ScratchPtr scratch,
    PtrQacc qacc,
    PtrImposed imposed
) noexcept {
    const i32 nv = params->nv;
    if (nv <= 0 || nv > kMaxNv) {
        return;
    }
    for (i32 i = 0; i < nv; ++i) {
        imposed[i] = BIKE_REAL(Real, 0.0);
    }
    const Real c_acc = (params->crank_spin_dof >= 0) ? qacc[params->crank_spin_dof] : BIKE_REAL(Real, 0.0);
    const Real fx = (params->frame_root_x_dof >= 0) ? qacc[params->frame_root_x_dof] : BIKE_REAL(Real, 0.0);
    const Real fz = (params->frame_root_z_dof >= 0) ? qacc[params->frame_root_z_dof] : BIKE_REAL(Real, 0.0);
    const Real fp = (params->frame_pitch_dof >= 0) ? qacc[params->frame_pitch_dof] : BIKE_REAL(Real, 0.0);
    const Real px = (params->rider_root_x_dof >= 0) ? qacc[params->rider_root_x_dof] : BIKE_REAL(Real, 0.0);
    const Real pz = (params->rider_root_z_dof >= 0) ? qacc[params->rider_root_z_dof] : BIKE_REAL(Real, 0.0);
    const Real pp = (params->rider_pitch_dof >= 0) ? qacc[params->rider_pitch_dof] : BIKE_REAL(Real, 0.0);

    if (params->rider_root_x_dof >= 0) {
        imposed[params->rider_root_x_dof] = px;
    }
    if (params->rider_root_z_dof >= 0) {
        imposed[params->rider_root_z_dof] = pz;
    }
    if (params->rider_pitch_dof >= 0) {
        imposed[params->rider_pitch_dof] = pp;
    }
    for (i32 k = 0; k < kNumUpperSpindleJoints; ++k) {
        const i32 d = params->joint_dof_adr[k];
        if (d >= 0) {
            imposed[d] = qacc[d];
        }
    }

    const Real a_cx = fx + (scratch->crank_world[2] - scratch->frame_pitch_anchor[2]) * fp;
    const Real a_cz = fz - (scratch->crank_world[0] - scratch->frame_pitch_anchor[0]) * fp;
    const Real pelvis_rel_acc = pp - fp;
    const Real rate2 = scratch->rate * scratch->rate;

    for (i32 s = 0; s < 2; ++s) {
        const Real a_hx = px + (scratch->hip_world[s][2] - scratch->rider_pitch_anchor[2]) * pp;
        const Real a_hz = pz - (scratch->hip_world[s][0] - scratch->rider_pitch_anchor[0]) * pp;
        const Real a_rel[3] = {
            a_hx - a_cx - scratch->delta_hip[s][2] * fp,
            BIKE_REAL(Real, 0.0),
            a_hz - a_cz + scratch->delta_hip[s][0] * fp
        };
        Real hip_acc_xz[2]{};
        detail::spindle_mat_t_vec_xz<Real>(scratch->frame_r, a_rel, hip_acc_xz);
        const i32 d0 = params->joint_dof_adr[5 + 2 * s];
        const i32 d1 = params->joint_dof_adr[6 + 2 * s];
        imposed[d0] = scratch->jacobians[s][0] * c_acc + scratch->djacs[s][0] * rate2 -
                      scratch->gammas_x[s][0] * hip_acc_xz[0] -
                      scratch->gammas[s][0] * hip_acc_xz[1] - pelvis_rel_acc;
        imposed[d1] = scratch->jacobians[s][1] * c_acc + scratch->djacs[s][1] * rate2 -
                      scratch->gammas_x[s][1] * hip_acc_xz[0] -
                      scratch->gammas[s][1] * hip_acc_xz[1];
    }
}

template <class Real, class ParamsPtr, class ScratchPtr, class PtrImposed>
BIKE_HD void fill_press_imposed_zero_qacc(
    ParamsPtr params,
    ScratchPtr scratch,
    PtrImposed imposed
) noexcept {
    const i32 nv = params->nv;
    if (nv <= 0 || nv > kMaxNv) {
        return;
    }
    for (i32 i = 0; i < nv; ++i) {
        imposed[i] = BIKE_REAL(Real, 0.0);
    }
    const Real rate2 = scratch->rate * scratch->rate;
    for (i32 s = 0; s < 2; ++s) {
        const i32 d0 = params->joint_dof_adr[5 + 2 * s];
        const i32 d1 = params->joint_dof_adr[6 + 2 * s];
        imposed[d0] = scratch->djacs[s][0] * rate2;
        imposed[d1] = scratch->djacs[s][1] * rate2;
    }
}

template <class Real, class ParamsPtr, class ScratchPtr, class PtrInertia, class PtrBias>
BIKE_HD PressStateCore<Real> finish_press_state(
    ParamsPtr params,
    ScratchPtr scratch,
    PtrInertia inertia,
    PtrBias qfrc_bias
) noexcept {
    Real f_grips_up = BIKE_REAL(Real, 0.0);
    for (i32 s = 0; s < 2; ++s) {
        if (scratch->arm_valid[s] == 0) {
            continue;
        }
        const i32 sd = params->joint_dof_adr[1 + 2 * s];
        const i32 ed = params->joint_dof_adr[2 + 2 * s];
        f_grips_up +=
            scratch->arm_gamma_sh[s] * (qfrc_bias[sd] + inertia[sd] - scratch->arm_tau_sh[s]) +
            scratch->arm_gamma_el[s] * (qfrc_bias[ed] + inertia[ed] - scratch->arm_tau_el[s]);
    }
    const Real root_rx = (params->rider_root_x_dof < 0)
        ? BIKE_REAL(Real, 0.0)
        : (qfrc_bias[params->rider_root_x_dof] + inertia[params->rider_root_x_dof]);
    const Real root_rz = (params->rider_root_z_dof < 0)
        ? BIKE_REAL(Real, 0.0)
        : (qfrc_bias[params->rider_root_z_dof] + inertia[params->rider_root_z_dof]);

    PressStateCore<Real> out{};
    out.max_total_press_n =
        scratch->frame_r[2] * root_rx + scratch->frame_r[8] * root_rz - f_grips_up;
    for (i32 s = 0; s < 2; ++s) {
        const i32 d0 = params->joint_dof_adr[5 + 2 * s];
        const i32 d1 = params->joint_dof_adr[6 + 2 * s];
        out.f_passive[s] =
            scratch->gammas[s][0] * (qfrc_bias[d0] + inertia[d0]) +
            scratch->gammas[s][1] * (qfrc_bias[d1] + inertia[d1]);
    }
    return out;
}

template <class Real, class ParamsPtr, class ScratchPtr,
          class PtrRowAdr, class PtrRowNnz, class PtrColInd, class PtrM,
          class PtrQacc, class PtrBias>
BIKE_HD PressStateCore<Real> compute_press_state_m(
    ParamsPtr params,
    ScratchPtr scratch,
    i32 qM_row_stride,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M_or_qM,
    PtrQacc qacc,
    PtrBias qfrc_bias
) noexcept {
    if (params->nv <= 0 || params->nv > kMaxNv) {
        return PressStateCore<Real>{};
    }
    Real imposed[kMaxNv]{};
    Real inertia[kMaxNv]{};
    fill_press_imposed<Real>(params, scratch, qacc, imposed);
    if (qM_row_stride >= params->nv) {
        mul_m_dense<Real>(params->nv, qM_row_stride, d_M_or_qM, imposed, inertia);
    } else {
        mul_m_csr<Real>(params->nv, M_rowadr, M_rownnz, M_colind, d_M_or_qM, imposed, inertia);
    }
    return finish_press_state<Real>(params, scratch, inertia, qfrc_bias);
}

template <class Real, class ParamsPtr, class ScratchPtr,
          class PtrRowAdr, class PtrRowNnz, class PtrColInd, class PtrM,
          class PtrBias>
BIKE_HD PressStateCore<Real> compute_press_state_zero_qacc_m(
    ParamsPtr params,
    ScratchPtr scratch,
    i32 qM_row_stride,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M_or_qM,
    PtrBias qfrc_bias
) noexcept {
    if (params->nv <= 0 || params->nv > kMaxNv) {
        return PressStateCore<Real>{};
    }
    Real imposed[kMaxNv]{};
    Real inertia[kMaxNv]{};
    fill_press_imposed_zero_qacc<Real>(params, scratch, imposed);
    if (qM_row_stride >= params->nv) {
        mul_m_dense<Real>(params->nv, qM_row_stride, d_M_or_qM, imposed, inertia);
    } else {
        mul_m_csr<Real>(params->nv, M_rowadr, M_rownnz, M_colind, d_M_or_qM, imposed, inertia);
    }
    return finish_press_state<Real>(params, scratch, inertia, qfrc_bias);
}

template <class Real, class ParamsPtr, class ScratchPtr,
          class PtrRowAdr, class PtrRowNnz, class PtrColInd, class PtrM,
          class PtrQacc, class PtrBias>
BIKE_HD PressStateCore<Real> compute_press_state_csr(
    ParamsPtr params,
    ScratchPtr scratch,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M,
    PtrQacc qacc,
    PtrBias qfrc_bias
) noexcept {
    return compute_press_state_m<Real>(
        params, scratch, 0, M_rowadr, M_rownnz, M_colind, d_M, qacc, qfrc_bias);
}

template <class Real, class ParamsPtr, class ScratchPtr,
          class PtrRowAdr, class PtrRowNnz, class PtrColInd, class PtrM,
          class PtrBias>
BIKE_HD PressStateCore<Real> compute_press_state_zero_qacc_csr(
    ParamsPtr params,
    ScratchPtr scratch,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M,
    PtrBias qfrc_bias
) noexcept {
    return compute_press_state_zero_qacc_m<Real>(
        params, scratch, 0, M_rowadr, M_rownnz, M_colind, d_M, qfrc_bias);
}

template <class Real, class ParamsPtr, class ScratchPtr, class QmPtr,
          class PtrQacc, class PtrBias>
BIKE_HD PressStateCore<Real> compute_press_state_dense(
    ParamsPtr params,
    ScratchPtr scratch,
    i32 qM_row_stride,
    QmPtr d_qM_w,
    PtrQacc qacc,
    PtrBias qfrc_bias
) noexcept {
    if (params->nv <= 0 || params->nv > kMaxNv || qM_row_stride < params->nv) {
        return PressStateCore<Real>{};
    }
    Real imposed[kMaxNv]{};
    Real inertia[kMaxNv]{};
    fill_press_imposed<Real>(params, scratch, qacc, imposed);
    mul_m_dense<Real>(params->nv, qM_row_stride, d_qM_w, imposed, inertia);
    return finish_press_state<Real>(params, scratch, inertia, qfrc_bias);
}

template <class Real, class ParamsPtr, class ScratchPtr, class QmPtr,
          class PtrBias>
BIKE_HD PressStateCore<Real> compute_press_state_zero_qacc_dense(
    ParamsPtr params,
    ScratchPtr scratch,
    i32 qM_row_stride,
    QmPtr d_qM_w,
    PtrBias qfrc_bias
) noexcept {
    if (params->nv <= 0 || params->nv > kMaxNv || qM_row_stride < params->nv) {
        return PressStateCore<Real>{};
    }
    Real imposed[kMaxNv]{};
    Real inertia[kMaxNv]{};
    fill_press_imposed_zero_qacc<Real>(params, scratch, imposed);
    mul_m_dense<Real>(params->nv, qM_row_stride, d_qM_w, imposed, inertia);
    return finish_press_state<Real>(params, scratch, inertia, qfrc_bias);
}

template <class Real, class MulMFn, class ParamsPtr, class ScratchPtr,
          class PtrQacc, class PtrBias>
BIKE_HD PressStateCore<Real> compute_press_state(
    ParamsPtr params,
    ScratchPtr scratch,
    PtrQacc qacc,
    PtrBias qfrc_bias,
    BIKE_REF(MulMFn) mul_m_fn
) noexcept {
    if (params->nv <= 0 || params->nv > kMaxNv) {
        return PressStateCore<Real>{};
    }
    Real imposed[kMaxNv]{};
    Real inertia[kMaxNv]{};
    fill_press_imposed<Real>(params, scratch, qacc, imposed);
    mul_m_fn(imposed, inertia);
    return finish_press_state<Real>(params, scratch, inertia, qfrc_bias);
}

// ---- 6. Torques, limits, and speed gate ------------------------------------

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class ScratchPtr, class PtrQvel, class StatePtr>
BIKE_HD void populate_leg_torques_core(
    ParamsPtr params,
    ScratchPtr scratch,
    PressStateCore<Real> ps,
    u8 commit_hold,
    PtrQvel qvel,
    StatePtr state,
    BIKE_THREAD Real (&out_req)[kNumSpindleJoints],
    BIKE_THREAD Real (&out_prio)[kNumSpindleJoints]
) noexcept {
    for (i32 k = 0; k < kNumUpperSpindleJoints; ++k) {
        out_req[k] = scratch->upper_req[k];
        out_prio[k] = scratch->upper_req[k];
    }

    const Real margin = params->pedal_press_margin_n;
    const u8 steady = scratch->steady_state;
    Real f_base[2]{};
    Real f_floor[2]{};
    Real f_min[2]{};
    for (i32 s = 0; s < 2; ++s) {
        const Real f_upper = TK::max(margin, scratch->f_weight[s]);
        f_base[s] = TK::min(ps.f_passive[s], f_upper);
        f_floor[s] = (steady != 0) ? ps.f_passive[s] : f_base[s];
        f_min[s] = (steady != 0) ? ps.f_passive[s] : TK::max(margin, f_base[s]);
    }

    Real f_task_press_max[2]{};
    Real f_rest_n[2]{};
    Real crank_press_ceiling_nm = BIKE_REAL(Real, 1e30);
    for (i32 s = 0; s < 2; ++s) {
        f_task_press_max[s] = TK::max(
            BIKE_REAL(Real, 0.0), ps.max_total_press_n - f_min[1 - s] - f_floor[s]);
        f_rest_n[s] = TK::max(scratch->f_weight[s], BIKE_REAL(Real, 0.0));
        if (scratch->shares[s] > BIKE_REAL(Real, 0.0) &&
            scratch->press_per_nm[s] > BIKE_REAL(Real, 1e-12)) {
            const Real lim_s = f_task_press_max[s] / (scratch->press_per_nm[s] * scratch->shares[s]);
            if (lim_s < crank_press_ceiling_nm) {
                crank_press_ceiling_nm = lim_s;
            }
        }
    }

    Real hold_nm = scratch->incoming_coast_hold_nm;
    Real total = BIKE_REAL(Real, 0.0);
    const Real mt = scratch->mean_crank_torque_nm;
    if (mt > BIKE_REAL(Real, 0.0)) {
        const Real c2p = Trig::template cos_val<Real, TK>(
            BIKE_REAL(Real, 2.0) * scratch->phase);
        const Real raw_task = mt * (BIKE_REAL(Real, 1.0) + params->pedal_torque_ripple * c2p);
        const Real task = TK::min(raw_task, crank_press_ceiling_nm);
        const bool at_press_limit =
            (raw_task + TK::max(BIKE_REAL(Real, 0.0), hold_nm)) >= crank_press_ceiling_nm;
        if (scratch->has_weld_crank_torque != 0 && scratch->advance != 0 &&
            scratch->rate >= scratch->crank_target_rate_rad_s &&
            scratch->weld_crank_torque_nm <= raw_task + params->weld_servo_limit_nm &&
            !at_press_limit) {
            const Real held = hold_nm + params->weld_servo_gain *
                                            (task - scratch->weld_crank_torque_nm);
            hold_nm = TK::clamp(held, -params->weld_servo_limit_nm, params->weld_servo_limit_nm);
        }
        const Real max_hold = TK::min(task, crank_press_ceiling_nm - task);
        const Real applied_hold = (hold_nm > max_hold) ? max_hold
                                : (hold_nm < -task)    ? -task
                                                       : hold_nm;
        total = task + applied_hold;
    } else if (scratch->crank_reposition != 0) {
        const Real raw = params->reposition_damping_nms_rad *
                         (scratch->crank_target_rate_rad_s - scratch->rate);
        total = TK::clamp(raw, -params->reposition_torque_limit_nm, params->reposition_torque_limit_nm);
    } else {
        hold_nm = BIKE_REAL(Real, 0.0);
        total = BIKE_REAL(Real, 0.0);
    }

    if (commit_hold != 0) {
        state->coast_hold_nm = hold_nm;
    }
    scratch->crank_task_nm = total;

    for (i32 s = 0; s < 2; ++s) {
        const Real leg_task_nm = scratch->shares[s] * total;
        const Real jac[2] = {scratch->jacobians[s][0], scratch->jacobians[s][1]};
        const Real cap_pos[2] = {scratch->leg_cap_pos[s][0], scratch->leg_cap_pos[s][1]};
        const Real cap_neg[2] = {scratch->leg_cap_neg[s][0], scratch->leg_cap_neg[s][1]};
        Real task_torques[2]{};
        if (mt > BIKE_REAL(Real, 0.0)) {
            detail::downstroke_joint_torques_2<Real, TK>(
                leg_task_nm, jac, cap_pos, cap_neg, task_torques);
        } else {
            detail::split_joint_torques_2<Real, TK>(
                leg_task_nm, jac, cap_pos, cap_neg, task_torques);
        }

        Real f_task_press = -(scratch->gammas[s][0] * task_torques[0] +
                              scratch->gammas[s][1] * task_torques[1]);
        if (f_task_press > f_task_press_max[s] && f_task_press > BIKE_REAL(Real, 1e-12)) {
            const Real sc = f_task_press_max[s] / f_task_press;
            task_torques[0] *= sc;
            task_torques[1] *= sc;
            f_task_press = f_task_press_max[s];
        } else if (f_task_press < -f_rest_n[s]) {
            const Real sc = -f_rest_n[s] / f_task_press;
            task_torques[0] *= sc;
            task_torques[1] *= sc;
            f_task_press = -f_rest_n[s];
        }

        Real delta_f_press = BIKE_REAL(Real, 0.0);
        if (steady != 0) {
            delta_f_press = BIKE_REAL(Real, 0.0);
        } else if (mt <= BIKE_REAL(Real, 0.0)) {
            delta_f_press = scratch->f_weight[s] - ps.f_passive[s];
        } else {
            delta_f_press = TK::max(margin - f_task_press, f_base[s]) - ps.f_passive[s];
        }

        Real press_torques[2] = {BIKE_REAL(Real, 0.0), BIKE_REAL(Real, 0.0)};
        if (TK::abs(delta_f_press) > BIKE_REAL(Real, 1e-9)) {
            const Real rem_pos[2] = {
                TK::max(BIKE_REAL(Real, 0.0), cap_pos[0] - task_torques[0]),
                TK::max(BIKE_REAL(Real, 0.0), cap_pos[1] - task_torques[1])
            };
            const Real rem_neg[2] = {
                TK::max(BIKE_REAL(Real, 0.0), cap_neg[0] + task_torques[0]),
                TK::max(BIKE_REAL(Real, 0.0), cap_neg[1] + task_torques[1])
            };
            const Real neg_gamma[2] = {-scratch->gammas[s][0], -scratch->gammas[s][1]};
            detail::split_joint_torques_2<Real, TK>(
                delta_f_press, neg_gamma, rem_pos, rem_neg, press_torques);
        }

        for (i32 i = 0; i < 2; ++i) {
            const i32 slot = 5 + 2 * s + i;
            const i32 dof = params->joint_dof_adr[slot];
            out_prio[slot] = Trig::flush_subnorm(press_torques[i]);
            const Real damp_nm = -params->leg_active_damping_nms_rad *
                                 (Trig::flush_subnorm(qvel[dof]) - scratch->qdot_leg_kin[s][i]);
            out_req[slot] = Trig::flush_subnorm(task_torques[i] + press_torques[i] + damp_nm);
        }
    }
    if (commit_hold != 0) {
        state->coast_hold_nm = Trig::flush_subnorm(state->coast_hold_nm);
    }
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class PtrQpos, class PtrQvel>
BIKE_HD void limit_torques_core(
    ParamsPtr params,
    const BIKE_THREAD Real (&requested)[kNumSpindleJoints],
    const BIKE_THREAD Real (&priority_requested)[kNumSpindleJoints],
    PtrQpos qpos,
    PtrQvel qvel,
    BIKE_THREAD Real (&out_limited)[kNumSpindleJoints]
) noexcept {
    Real prio[kNumSpindleJoints]{};
    Real speeds[kNumSpindleJoints]{};
    const Real jlim_w = params->joint_power_limit_w;
    const Real spd_lim = params->joint_speed_limit_rad_s;

    for (i32 i = 0; i < kNumSpindleJoints; ++i) {
        const Real q = Trig::flush_subnorm(qpos[params->joint_qpos_adr[i]]);
        const Real v = Trig::flush_subnorm(qvel[params->joint_dof_adr[i]]);
        speeds[i] = v;

        const Real tr = Trig::flush_subnorm(requested[i]);
        const Real cap_r = detail::spindle_directional_limit<Real, TK, Trig>(
            params, i, q, v, (tr >= BIKE_REAL(Real, 0.0)) ? BIKE_REAL(Real, 1.0) : BIKE_REAL(Real, -1.0));
        Real vr = TK::clamp(tr, -cap_r, cap_r);
        if (TK::abs(v) >= spd_lim && Trig::flush_subnorm(vr * v) > BIKE_REAL(Real, 0.0)) {
            vr = BIKE_REAL(Real, 0.0);
        }
        if (jlim_w >= BIKE_REAL(Real, 0.0) && Trig::flush_subnorm(vr * v) > jlim_w) {
            vr = Trig::flush_subnorm(spindle_copysign(jlim_w / TK::abs(v), vr));
        }
        out_limited[i] = Trig::flush_subnorm(vr);

        const Real tp = Trig::flush_subnorm(priority_requested[i]);
        const Real cap_p = detail::spindle_directional_limit<Real, TK, Trig>(
            params, i, q, v, (tp >= BIKE_REAL(Real, 0.0)) ? BIKE_REAL(Real, 1.0) : BIKE_REAL(Real, -1.0));
        Real vp = TK::clamp(tp, -cap_p, cap_p);
        if (TK::abs(v) >= spd_lim && Trig::flush_subnorm(vp * v) > BIKE_REAL(Real, 0.0)) {
            vp = BIKE_REAL(Real, 0.0);
        }
        if (jlim_w >= BIKE_REAL(Real, 0.0) && Trig::flush_subnorm(vp * v) > jlim_w) {
            vp = Trig::flush_subnorm(spindle_copysign(jlim_w / TK::abs(v), vp));
        }
        prio[i] = Trig::flush_subnorm(vp);
    }

    const Real total_w = (params->has_active_positive_power_limit != 0)
        ? params->active_positive_power_limit_w
        : (jlim_w * static_cast<Real>(kNumSpindleJoints));

    Real req_power = BIKE_REAL(Real, 0.0);
    for (i32 i = 0; i < kNumSpindleJoints; ++i) {
        const Real p_i = Trig::flush_subnorm(out_limited[i] * speeds[i]);
        req_power += TK::max(p_i, BIKE_REAL(Real, 0.0));
    }
    if (req_power <= total_w || total_w < BIKE_REAL(Real, 0.0)) {
        return;
    }

    Real prio_power = BIKE_REAL(Real, 0.0);
    for (i32 i = 0; i < kNumSpindleJoints; ++i) {
        const Real p_i = Trig::flush_subnorm(prio[i] * speeds[i]);
        prio_power += TK::max(p_i, BIKE_REAL(Real, 0.0));
    }
    if (prio_power >= total_w) {
        Real scaled[kNumSpindleJoints]{};
        detail::scale_to_power_budget_9<Real, TK, Trig>(prio, speeds, jlim_w, total_w, scaled);
        for (i32 i = 0; i < kNumSpindleJoints; ++i) {
            out_limited[i] = scaled[i];
        }
        return;
    }

    Real lo = BIKE_REAL(Real, 0.0);
    Real hi = BIKE_REAL(Real, 1.0);
    for (i32 iter = 0; iter < kSpindlePowerBisectIters; ++iter) {
        const Real mid = BIKE_REAL(Real, 0.5) * (lo + hi);
        Real s = BIKE_REAL(Real, 0.0);
        for (i32 i = 0; i < kNumSpindleJoints; ++i) {
            const Real p_prio = TK::max(
                Trig::flush_subnorm(prio[i] * speeds[i]), BIKE_REAL(Real, 0.0));
            const Real p_lim = TK::max(
                Trig::flush_subnorm(out_limited[i] * speeds[i]), BIKE_REAL(Real, 0.0));
            const Real cand = (p_lim > p_prio)
                ? (prio[i] + mid * (out_limited[i] - prio[i]))
                : out_limited[i];
            s += TK::max(Trig::flush_subnorm(cand * speeds[i]), BIKE_REAL(Real, 0.0));
        }
        if (s <= total_w) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    for (i32 i = 0; i < kNumSpindleJoints; ++i) {
        const Real p_prio = TK::max(
            Trig::flush_subnorm(prio[i] * speeds[i]), BIKE_REAL(Real, 0.0));
        const Real p_lim = TK::max(
            Trig::flush_subnorm(out_limited[i] * speeds[i]), BIKE_REAL(Real, 0.0));
        if (p_lim > p_prio) {
            out_limited[i] = Trig::flush_subnorm(prio[i] + lo * (out_limited[i] - prio[i]));
        }
    }
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class PtrQvel, class PtrCtrl>
BIKE_HD void write_ctrl_with_speed_gate(
    ParamsPtr params,
    const BIKE_THREAD Real (&exc)[kNumSpindleJoints],
    PtrQvel qvel,
    u8 command_enabled,
    PtrCtrl ctrl
) noexcept {
    if (!validate_spindle_m2k2_params(params)) {
        return;
    }
    const Real spd_lim = params->joint_speed_limit_rad_s;
    for (i32 i = 0; i < kNumSpindleJoints; ++i) {
        const i32 aid = params->joint_actuator_id[i];
        if (aid < 0) {
            continue;
        }
        if (command_enabled == 0) {
            ctrl[aid] = BIKE_REAL(Real, 0.0);
            continue;
        }
        const Real speed = Trig::flush_subnorm(qvel[params->joint_dof_adr[i]]);
        Real delivered = Trig::flush_subnorm(exc[i]);
        if (TK::abs(speed) >= spd_lim &&
            Trig::flush_subnorm(delivered * speed) > BIKE_REAL(Real, 0.0)) {
            delivered = BIKE_REAL(Real, 0.0);
        }
        ctrl[aid] = delivered;
    }
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class StatePtr, class PtrQpos, class PtrQvel, class PtrCtrl>
BIKE_HD void spindle_m2k2_write_held_ctrl(
    ParamsPtr params,
    StatePtr state,
    PtrQpos qpos,
    PtrQvel qvel,
    u8 command_enabled,
    PtrCtrl ctrl
) noexcept {
    if (!validate_spindle_m2k2_params(params)) {
        return;
    }
    const Real spd_lim = params->joint_speed_limit_rad_s;
    for (i32 i = 0; i < kNumSpindleJoints; ++i) {
        const i32 aid = params->joint_actuator_id[i];
        if (aid < 0) {
            continue;
        }
        if (command_enabled == 0) {
            ctrl[aid] = BIKE_REAL(Real, 0.0);
            continue;
        }
        const Real t = Trig::flush_subnorm(state->held_torques[i]);
        const Real q = Trig::flush_subnorm(qpos[params->joint_qpos_adr[i]]);
        const Real speed = Trig::flush_subnorm(qvel[params->joint_dof_adr[i]]);
        const Real cap = detail::spindle_directional_limit<Real, TK, Trig>(
            params, i, q, speed,
            (t >= BIKE_REAL(Real, 0.0)) ? BIKE_REAL(Real, 1.0) : BIKE_REAL(Real, -1.0));
        Real clipped = TK::clamp(t, -cap, cap);
        if (TK::abs(speed) >= spd_lim &&
            Trig::flush_subnorm(clipped * speed) > BIKE_REAL(Real, 0.0)) {
            clipped = BIKE_REAL(Real, 0.0);
        }
        ctrl[aid] = Trig::flush_subnorm(clipped);
    }
}

// ---- 7. 4-Pass M2_K2 Newton Pedal Press Solver ----------------------------

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class StatePtr, class ScratchPtr,
          class PtrQpos, class PtrQvel, class PtrBias,
          class PtrXmat, class PtrPos, class PtrAnchor, class PtrAxis,
          class PtrSitePos, class PtrSubtreeCom, class PtrSubtreeMass>
BIKE_HD void spindle_populate_step_scratch_invariants(
    ParamsPtr params,
    SpindleCommand<Real> cmd,
    PtrQpos qpos,
    PtrQvel qvel,
    PtrBias qfrc_bias,
    PtrXmat body_xmat,
    PtrPos body_xpos,
    PtrAnchor jnt_xanchor,
    PtrAxis jnt_xaxis,
    PtrSitePos site_xpos,
    PtrSubtreeCom subtree_com,
    PtrSubtreeMass body_subtreemass,
    StatePtr state,
    ScratchPtr scratch
) noexcept {
    // Populate step scratch invariants from current state
    const i32 fb9 = params->frame_body * 9;
    const i32 pb9 = params->pelvis_body * 9;
    for (i32 k = 0; k < 9; ++k) {
        scratch->frame_r[k] = body_xmat[fb9 + k];
    }
    const i32 cj3 = params->crank_joint * 3;
    const i32 fpj3 = params->frame_pitch_joint * 3;
    const i32 rpj3 = params->rider_pitch_joint * 3;
    for (i32 c = 0; c < 3; ++c) {
        scratch->crank_world[c] = jnt_xanchor[cj3 + c];
        scratch->frame_pitch_anchor[c] = jnt_xanchor[fpj3 + c];
        scratch->rider_pitch_anchor[c] = jnt_xanchor[rpj3 + c];
    }

    const Real rel00 =
        scratch->frame_r[0] * body_xmat[pb9 + 0] +
        scratch->frame_r[3] * body_xmat[pb9 + 3] +
        scratch->frame_r[6] * body_xmat[pb9 + 6];
    const Real rel02 =
        scratch->frame_r[0] * body_xmat[pb9 + 2] +
        scratch->frame_r[3] * body_xmat[pb9 + 5] +
        scratch->frame_r[6] * body_xmat[pb9 + 8];
    const Real pelvis_pitch = Trig::template atan2_val<Real, TK>(rel02, rel00);
    scratch->pelvis_pitch = pelvis_pitch;

    const i32 ped0_3 = params->pedal_bodies[0] * 3;
    const Real dped0[3] = {
        body_xpos[ped0_3 + 0] - scratch->crank_world[0],
        body_xpos[ped0_3 + 1] - scratch->crank_world[1],
        body_xpos[ped0_3 + 2] - scratch->crank_world[2]
    };
    Real sp0_xz[2]{};
    detail::spindle_mat_t_vec_xz<Real>(scratch->frame_r, dped0, sp0_xz);
    const Real phase = Trig::template atan2_val<Real, TK>(-sp0_xz[1], sp0_xz[0]);
    const Real rate = Trig::flush_subnorm(qvel[params->crank_spin_dof]);
    scratch->phase = phase;
    scratch->rate = rate;

    Real shares[2]{};
    leg_shares_core<Real, TK, Trig>(phase, shares);
    scratch->shares[0] = shares[0];
    scratch->shares[1] = shares[1];

    const Real fx_v = (params->frame_root_x_dof >= 0) ? Trig::flush_subnorm(qvel[params->frame_root_x_dof]) : BIKE_REAL(Real, 0.0);
    const Real fz_v = (params->frame_root_z_dof >= 0) ? Trig::flush_subnorm(qvel[params->frame_root_z_dof]) : BIKE_REAL(Real, 0.0);
    const Real fp_v = (params->frame_pitch_dof >= 0) ? Trig::flush_subnorm(qvel[params->frame_pitch_dof]) : BIKE_REAL(Real, 0.0);
    const Real px_v = (params->rider_root_x_dof >= 0) ? Trig::flush_subnorm(qvel[params->rider_root_x_dof]) : BIKE_REAL(Real, 0.0);
    const Real pz_v = (params->rider_root_z_dof >= 0) ? Trig::flush_subnorm(qvel[params->rider_root_z_dof]) : BIKE_REAL(Real, 0.0);
    const Real pp_v = (params->rider_pitch_dof >= 0) ? Trig::flush_subnorm(qvel[params->rider_pitch_dof]) : BIKE_REAL(Real, 0.0);

    const Real v_crank_x = fx_v + (scratch->crank_world[2] - scratch->frame_pitch_anchor[2]) * fp_v;
    const Real v_crank_z = fz_v - (scratch->crank_world[0] - scratch->frame_pitch_anchor[0]) * fp_v;
    const Real pelvis_rel_rate = pp_v - fp_v;

    for (i32 s = 0; s < 2; ++s) {
        const i32 hj3 = params->hip_joints[s] * 3;
        for (i32 c = 0; c < 3; ++c) {
            scratch->hip_world[s][c] = jnt_xanchor[hj3 + c];
            scratch->delta_hip[s][c] = scratch->hip_world[s][c] - scratch->crank_world[c];
        }
        Real hip_xz[2]{};
        detail::spindle_mat_t_vec_xz<Real>(scratch->frame_r, scratch->delta_hip[s], hip_xz);
        const LegKinematicsResult<Real> lk = analytic_leg_kinematics<Real, TK, Trig>(
            params->thigh_m[s], params->link_m[s], params->crank_length_m,
            params->side_sign[s], params->knee_sign[s],
            hip_xz[0], hip_xz[1], phase,
            params->det_eps, params->ik_reach_eps * BIKE_REAL(Real, 10.0));
        scratch->has_jacobian[s] = lk.valid;
        for (i32 j = 0; j < 2; ++j) {
            scratch->jacobians[s][j] = lk.jac[j];
            scratch->djacs[s][j] = lk.djac[j];
            scratch->gammas[s][j] = lk.gamma_z[j];
            scratch->gammas_x[s][j] = lk.gamma_x[j];
        }

        const Real v_hx = px_v + (scratch->hip_world[s][2] - scratch->rider_pitch_anchor[2]) * pp_v;
        const Real v_hz = pz_v - (scratch->hip_world[s][0] - scratch->rider_pitch_anchor[0]) * pp_v;
        const Real v_rel[3] = {
            v_hx - v_crank_x - scratch->delta_hip[s][2] * fp_v,
            BIKE_REAL(Real, 0.0),
            v_hz - v_crank_z + scratch->delta_hip[s][0] * fp_v
        };
        Real hv_xz[2]{};
        detail::spindle_mat_t_vec_xz<Real>(scratch->frame_r, v_rel, hv_xz);
        scratch->qdot_leg_kin[s][0] =
            lk.jac[0] * rate - lk.gamma_x[0] * hv_xz[0] - lk.gamma_z[0] * hv_xz[1] - pelvis_rel_rate;
        scratch->qdot_leg_kin[s][1] =
            lk.jac[1] * rate - lk.gamma_x[1] * hv_xz[0] - lk.gamma_z[1] * hv_xz[1];

        for (i32 j = 0; j < 2; ++j) {
            const i32 slot = 5 + 2 * s + j;
            const Real qj = Trig::flush_subnorm(qpos[params->joint_qpos_adr[slot]]);
            const Real v_j = Trig::flush_subnorm(qvel[params->joint_dof_adr[slot]]);
            scratch->leg_cap_pos[s][j] = detail::spindle_usable_slot_capacity<Real, TK, Trig>(
                params, slot, qj, v_j, BIKE_REAL(Real, 1.0));
            scratch->leg_cap_neg[s][j] = detail::spindle_usable_slot_capacity<Real, TK, Trig>(
                params, slot, qj, v_j, BIKE_REAL(Real, -1.0));
        }

        Real pp = BIKE_REAL(Real, 0.0);
        if (shares[s] > BIKE_REAL(Real, 0.0)) {
            const Real c_hip = scratch->leg_cap_pos[s][0];
            const Real c_knee = scratch->leg_cap_neg[s][1];
            const Real w_hip = c_hip * c_hip * lk.jac[0];
            const Real w_knee = c_knee * c_knee * lk.jac[1];
            const Real denom = lk.jac[0] * w_hip + lk.jac[1] * w_knee;
            if (denom >= BIKE_REAL(Real, 1e-9)) {
                const Real u_hip = (lk.jac[0] > BIKE_REAL(Real, 0.0)) ? (w_hip / denom) : BIKE_REAL(Real, 0.0);
                const Real u_knee = (lk.jac[1] < BIKE_REAL(Real, 0.0)) ? (w_knee / denom) : BIKE_REAL(Real, 0.0);
                pp = -(lk.gamma_z[0] * u_hip + lk.gamma_z[1] * u_knee);
            }
        }
        scratch->press_per_nm[s] = pp;
    }

    Real fw[2]{};
    leg_weights_at_pedal_core<Real>(
        params, body_subtreemass, subtree_com, jnt_xanchor, jnt_xaxis,
        scratch->gammas[0][0], scratch->gammas[0][1],
        scratch->gammas[1][0], scratch->gammas[1][1], fw);
    scratch->f_weight[0] = fw[0];
    scratch->f_weight[1] = fw[1];

    Real up_targets[kNumUpperSpindleJoints]{};
    u8 torso_sat = 0;
    u8 arms_sat = 0;
    upper_targets_core<Real, TK, Trig>(
        params, qpos, body_xpos, body_xmat, jnt_xanchor, cmd.torso_lean_rad,
        up_targets, torso_sat, arms_sat);
    scratch->torso_saturated = torso_sat;
    scratch->arms_saturated = arms_sat;

    for (i32 k = 0; k < kNumUpperSpindleJoints; ++k) {
        const i32 dof = params->joint_dof_adr[k];
        Real torque = BIKE_REAL(Real, 0.0);
        if (k == 0) {
            torque = params->joint_kp_nm_rad * pelvis_pitch +
                     params->joint_kd_nms_rad * (pp_v - fp_v);
        } else {
            const Real qk = Trig::flush_subnorm(qpos[params->joint_qpos_adr[k]]);
            const Real vk = Trig::flush_subnorm(qvel[dof]);
            torque = params->joint_kp_nm_rad *
                         detail::spindle_wrap_angle<Real, TK, Trig>(up_targets[k] - qk) -
                     params->joint_kd_nms_rad * vk;
        }
        scratch->upper_req[k] = Trig::flush_subnorm(torque + qfrc_bias[dof]);
    }

    for (i32 s = 0; s < 2; ++s) {
        const i32 ua3 = params->upper_arm_bodies[s] * 3;
        const i32 fa3 = params->forearm_bodies[s] * 3;
        const i32 gs3 = params->grip_sites[s] * 3;
        const Real d_sh[3] = {
            body_xpos[ua3 + 0] - scratch->crank_world[0],
            body_xpos[ua3 + 1] - scratch->crank_world[1],
            body_xpos[ua3 + 2] - scratch->crank_world[2]
        };
        const Real d_el[3] = {
            body_xpos[fa3 + 0] - scratch->crank_world[0],
            body_xpos[fa3 + 1] - scratch->crank_world[1],
            body_xpos[fa3 + 2] - scratch->crank_world[2]
        };
        const Real d_gr[3] = {
            site_xpos[gs3 + 0] - scratch->crank_world[0],
            site_xpos[gs3 + 1] - scratch->crank_world[1],
            site_xpos[gs3 + 2] - scratch->crank_world[2]
        };
        Real sh_xz[2]{};
        Real el_xz[2]{};
        Real gr_xz[2]{};
        detail::spindle_mat_t_vec_xz<Real>(scratch->frame_r, d_sh, sh_xz);
        detail::spindle_mat_t_vec_xz<Real>(scratch->frame_r, d_el, el_xz);
        detail::spindle_mat_t_vec_xz<Real>(scratch->frame_r, d_gr, gr_xz);
        const Real det =
            -(gr_xz[1] - sh_xz[1]) * (gr_xz[0] - el_xz[0]) +
            (gr_xz[1] - el_xz[1]) * (gr_xz[0] - sh_xz[0]);
        if (TK::abs(det) <= params->det_eps) {
            scratch->arm_valid[s] = 0;
            scratch->arm_gamma_sh[s] = BIKE_REAL(Real, 0.0);
            scratch->arm_gamma_el[s] = BIKE_REAL(Real, 0.0);
            scratch->arm_tau_sh[s] = BIKE_REAL(Real, 0.0);
            scratch->arm_tau_el[s] = BIKE_REAL(Real, 0.0);
            continue;
        }
        scratch->arm_valid[s] = 1;
        scratch->arm_gamma_sh[s] = -(gr_xz[1] - el_xz[1]) / det;
        scratch->arm_gamma_el[s] = (gr_xz[1] - sh_xz[1]) / det;

        const i32 sh_slot = 1 + 2 * s;
        const i32 el_slot = 2 + 2 * s;
        const Real req_sh = scratch->upper_req[sh_slot];
        const Real req_el = scratch->upper_req[el_slot];
        const Real q_sh = Trig::flush_subnorm(qpos[params->joint_qpos_adr[sh_slot]]);
        const Real v_sh = Trig::flush_subnorm(qvel[params->joint_dof_adr[sh_slot]]);
        const Real q_el = Trig::flush_subnorm(qpos[params->joint_qpos_adr[el_slot]]);
        const Real v_el = Trig::flush_subnorm(qvel[params->joint_dof_adr[el_slot]]);
        const Real cap_sh = detail::spindle_usable_slot_capacity<Real, TK, Trig>(
            params, sh_slot, q_sh, v_sh,
            (req_sh >= BIKE_REAL(Real, 0.0)) ? BIKE_REAL(Real, 1.0) : BIKE_REAL(Real, -1.0));
        const Real cap_el = detail::spindle_usable_slot_capacity<Real, TK, Trig>(
            params, el_slot, q_el, v_el,
            (req_el >= BIKE_REAL(Real, 0.0)) ? BIKE_REAL(Real, 1.0) : BIKE_REAL(Real, -1.0));
        scratch->arm_tau_sh[s] = TK::clamp(req_sh, -cap_sh, cap_sh);
        scratch->arm_tau_el[s] = TK::clamp(req_el, -cap_el, cap_el);
    }

    scratch->incoming_coast_hold_nm = state->coast_hold_nm;
    scratch->mean_crank_torque_nm = cmd.mean_crank_torque_nm;
    scratch->crank_target_rate_rad_s = cmd.crank_target_rate_rad_s;
    scratch->weld_crank_torque_nm = cmd.weld_crank_torque_nm;
    scratch->has_weld_crank_torque = cmd.has_weld_crank_torque;
    scratch->crank_reposition = cmd.crank_reposition;
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class StatePtr, class ScratchPtr,
          class PtrQpos, class PtrQvel, class PtrBias,
          class PtrXmat, class PtrPos, class PtrAnchor, class PtrAxis,
          class PtrSitePos, class PtrSubtreeCom, class PtrSubtreeMass,
          class PtrRowAdr, class PtrRowNnz, class PtrColInd, class PtrM,
          class PtrCtrl>
BIKE_HD void spindle_m2k2_pass0_init_and_base(
    ParamsPtr params,
    SpindleCommand<Real> cmd,
    i32 step_index,
    PtrQpos qpos,
    PtrQvel qvel,
    PtrBias qfrc_bias,
    PtrXmat body_xmat,
    PtrPos body_xpos,
    PtrAnchor jnt_xanchor,
    PtrAxis jnt_xaxis,
    PtrSitePos site_xpos,
    PtrSubtreeCom subtree_com,
    PtrSubtreeMass body_subtreemass,
    i32 qM_row_stride,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M_or_qM,
    StatePtr state,
    ScratchPtr scratch,
    PtrCtrl ctrl
) noexcept {
    (void)step_index;
    scratch->active_newton = 0;
    scratch->converged = 1;
    scratch->perturbed0 = 0;
    scratch->perturbed1 = 0;
    scratch->adaptive_mode = 0;
    if (!validate_spindle_m2k2_params(params)) {
        return;
    }
    const u8 cmd_en = (cmd.enabled != 0 && state->enabled != 0) ? 1u : 0u;
    state->command_enabled = cmd_en;
    scratch->command_enabled = cmd_en;
    scratch->advance = cmd.advance;
    scratch->steady_state = cmd.steady_state;

    if (cmd_en == 0) {
        scratch->torso_saturated = 0;
        scratch->arms_saturated = 0;
        scratch->crank_task_nm = BIKE_REAL(Real, 0.0);
        for (i32 i = 0; i < kNumSpindleJoints; ++i) {
            scratch->last_req[i] = BIKE_REAL(Real, 0.0);
            scratch->last_exc[i] = BIKE_REAL(Real, 0.0);
        }
        if (cmd.advance != 0) {
            state->coast_hold_nm = BIKE_REAL(Real, 0.0);
            for (i32 i = 0; i < kNumSpindleJoints; ++i) {
                state->held_torques[i] = BIKE_REAL(Real, 0.0);
            }
            state->has_held_torques = 1;
            state->has_prev_ps = 0;
            state->prev_ps = PressStateCore<Real>{};
        }
        for (i32 i = 0; i < kNumSpindleJoints; ++i) {
            const i32 aid = params->joint_actuator_id[i];
            if (aid >= 0) {
                ctrl[aid] = BIKE_REAL(Real, 0.0);
            }
        }
        return;
    }

    const i32 period = (params->control_period_steps > 0) ? params->control_period_steps : 1;
    const u8 is_control_tick =
        (cmd.advance == 0 || cmd.steady_state != 0 || state->has_held_torques == 0 ||
         (step_index % period == 0))
            ? 1u
            : 0u;
    if (is_control_tick == 0) {
        spindle_m2k2_write_held_ctrl<Real, TK, Trig>(
            params, state, qpos, qvel, cmd_en, ctrl);
        return;
    }

    spindle_populate_step_scratch_invariants<Real, TK, Trig>(
        params, cmd, qpos, qvel, qfrc_bias,
        body_xmat, body_xpos, jnt_xanchor, jnt_xaxis,
        site_xpos, subtree_com, body_subtreemass, state, scratch);

    const PressStateCore<Real> ps0 =
        (state->has_prev_ps != 0 && cmd.steady_state == 0 && state->enabled != 0)
        ? state->prev_ps
        : compute_press_state_zero_qacc_m<Real>(
              params, scratch, qM_row_stride, M_rowadr, M_rownnz, M_colind, d_M_or_qM, qfrc_bias);
    scratch->ps0 = ps0;

    Real req[kNumSpindleJoints]{};
    Real prio[kNumSpindleJoints]{};
    Real exc[kNumSpindleJoints]{};

    if (cmd.steady_state != 0) {
        populate_leg_torques_core<Real, TK, Trig>(
            params, scratch, ps0, cmd.advance, qvel, state, req, prio);
        limit_torques_core<Real, TK, Trig>(params, req, prio, qpos, qvel, exc);
        for (i32 i = 0; i < kNumSpindleJoints; ++i) {
            scratch->last_req[i] = req[i];
            scratch->last_exc[i] = exc[i];
        }
        if (cmd.advance != 0) {
            for (i32 i = 0; i < kNumSpindleJoints; ++i) {
                state->held_torques[i] = exc[i];
            }
            state->has_held_torques = 1;
            state->prev_ps = ps0;
            state->has_prev_ps = 1;
        }
        write_ctrl_with_speed_gate<Real, TK, Trig>(params, exc, qvel, 1u, ctrl);
        return;
    }

    populate_leg_torques_core<Real, TK, Trig>(
        params, scratch, ps0, 0u, qvel, state, req, prio);
    for (i32 j = 0; j < 4; ++j) {
        scratch->base_leg_req[j] = Trig::flush_subnorm(req[5 + j]);
        scratch->base_leg_prio[j] = Trig::flush_subnorm(prio[5 + j]);
    }
    limit_torques_core<Real, TK, Trig>(params, req, prio, qpos, qvel, exc);
    write_ctrl_with_speed_gate<Real, TK, Trig>(params, exc, qvel, 1u, ctrl);

    scratch->active_newton = 1;
    scratch->converged = 0;
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class StatePtr, class ScratchPtr,
          class PtrQpos, class PtrQvel, class PtrBias,
          class PtrXmat, class PtrPos, class PtrAnchor, class PtrAxis,
          class PtrSitePos, class PtrSubtreeCom, class PtrSubtreeMass,
          class PtrRowAdr, class PtrRowNnz, class PtrColInd, class PtrM,
          class PtrCtrl>
BIKE_HD void spindle_m2k2_pass0_init_and_base(
    ParamsPtr params,
    SpindleCommand<Real> cmd,
    i32 step_index,
    PtrQpos qpos,
    PtrQvel qvel,
    PtrBias qfrc_bias,
    PtrXmat body_xmat,
    PtrPos body_xpos,
    PtrAnchor jnt_xanchor,
    PtrAxis jnt_xaxis,
    PtrSitePos site_xpos,
    PtrSubtreeCom subtree_com,
    PtrSubtreeMass body_subtreemass,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M,
    StatePtr state,
    ScratchPtr scratch,
    PtrCtrl ctrl
) noexcept {
    spindle_m2k2_pass0_init_and_base<Real, TK, Trig>(
        params, cmd, step_index, qpos, qvel, qfrc_bias,
        body_xmat, body_xpos, jnt_xanchor, jnt_xaxis, site_xpos, subtree_com, body_subtreemass,
        0, M_rowadr, M_rownnz, M_colind, d_M, state, scratch, ctrl);
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class StatePtr, class ScratchPtr,
          class PtrQpos, class PtrQvel, class PtrBias, class PtrQacc,
          class PtrRowAdr, class PtrRowNnz, class PtrColInd, class PtrM,
          class PtrCtrl>
BIKE_HD void spindle_m2k2_pass1_perturb0(
    ParamsPtr params,
    PtrQpos qpos,
    PtrQvel qvel,
    PtrBias qfrc_bias,
    PtrQacc qacc,
    i32 qM_row_stride,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M_or_qM,
    StatePtr state,
    ScratchPtr scratch,
    PtrCtrl ctrl
) noexcept {
    if (!validate_spindle_m2k2_params(params) || scratch->active_newton == 0) {
        return;
    }
    const PressStateCore<Real> P = compute_press_state_m<Real>(
        params, scratch, qM_row_stride, M_rowadr, M_rownnz, M_colind, d_M_or_qM, qacc, qfrc_bias);
    scratch->P = P;

    const Real r0 = P.f_passive[0] - scratch->ps0.f_passive[0];
    const Real r1 = P.f_passive[1] - scratch->ps0.f_passive[1];
    const Real r2 = P.max_total_press_n - scratch->ps0.max_total_press_n;
    const Real err = TK::max(TK::abs(r0), TK::max(TK::abs(r1), TK::abs(r2)));
    if (err <= params->conv_eps) {
        scratch->converged = 1;
        scratch->perturbed0 = 0;
        return;
    }

    Real req[kNumSpindleJoints]{};
    Real prio[kNumSpindleJoints]{};
    Real exc[kNumSpindleJoints]{};

    const bool sign_flip_kink =
        ((scratch->ps0.f_passive[0] > BIKE_REAL(Real, 0.0) && P.f_passive[0] < BIKE_REAL(Real, 0.0)) ||
         (scratch->ps0.f_passive[0] < BIKE_REAL(Real, 0.0) && P.f_passive[0] > BIKE_REAL(Real, 0.0)) ||
         (scratch->ps0.f_passive[1] > BIKE_REAL(Real, 0.0) && P.f_passive[1] < BIKE_REAL(Real, 0.0)) ||
         (scratch->ps0.f_passive[1] < BIKE_REAL(Real, 0.0) && P.f_passive[1] > BIKE_REAL(Real, 0.0)));
    if (state->has_prev_ps != 0 &&
        (err > BIKE_REAL(Real, 100.0) || sign_flip_kink)) {
        scratch->adaptive_mode = 1;
        const Real alpha = BIKE_REAL(Real, 0.55);
        PressStateCore<Real> ps_mid{};
        ps_mid.f_passive[0] = scratch->ps0.f_passive[0] + alpha * r0;
        ps_mid.f_passive[1] = scratch->ps0.f_passive[1] + alpha * r1;
        ps_mid.max_total_press_n = scratch->ps0.max_total_press_n + alpha * r2;
        scratch->ps_trial = ps_mid;
        populate_leg_torques_core<Real, TK, Trig>(
            params, scratch, ps_mid, 0u, qvel, state, req, prio);
        limit_torques_core<Real, TK, Trig>(params, req, prio, qpos, qvel, exc);
        write_ctrl_with_speed_gate<Real, TK, Trig>(params, exc, qvel, 1u, ctrl);
        return;
    }

    PressStateCore<Real> ps_p0 = scratch->ps0;
    ps_p0.f_passive[0] += params->newton_eps;
    populate_leg_torques_core<Real, TK, Trig>(
        params, scratch, ps_p0, 0u, qvel, state, req, prio);

    u8 same = 1;
    for (i32 j = 0; j < 4; ++j) {
        if (Trig::flush_subnorm(req[5 + j]) != scratch->base_leg_req[j] ||
            Trig::flush_subnorm(prio[5 + j]) != scratch->base_leg_prio[j]) {
            same = 0;
        }
    }
    if (same != 0 && state->has_prev_ps != 0) {
        PressStateCore<Real> ps_chk = scratch->ps0;
        ps_chk.f_passive[0] = P.f_passive[0];
        populate_leg_torques_core<Real, TK, Trig>(
            params, scratch, ps_chk, 0u, qvel, state, req, prio);
        u8 diff_at_P = 0;
        for (i32 j = 0; j < 4; ++j) {
            if (Trig::flush_subnorm(req[5 + j]) != scratch->base_leg_req[j] ||
                Trig::flush_subnorm(prio[5 + j]) != scratch->base_leg_prio[j]) {
                diff_at_P = 1;
            }
        }
        if (diff_at_P != 0) {
            scratch->adaptive_mode = 1;
            const Real alpha = BIKE_REAL(Real, 0.55);
            PressStateCore<Real> ps_mid{};
            ps_mid.f_passive[0] = scratch->ps0.f_passive[0] + alpha * r0;
            ps_mid.f_passive[1] = scratch->ps0.f_passive[1] + alpha * r1;
            ps_mid.max_total_press_n = scratch->ps0.max_total_press_n + alpha * r2;
            scratch->ps_trial = ps_mid;
            populate_leg_torques_core<Real, TK, Trig>(
                params, scratch, ps_mid, 0u, qvel, state, req, prio);
            limit_torques_core<Real, TK, Trig>(params, req, prio, qpos, qvel, exc);
            write_ctrl_with_speed_gate<Real, TK, Trig>(params, exc, qvel, 1u, ctrl);
            return;
        }
    }
    if (same != 0) {
        scratch->perturbed0 = 0;
        return;
    }

    scratch->perturbed0 = 1;
    limit_torques_core<Real, TK, Trig>(params, req, prio, qpos, qvel, exc);
    write_ctrl_with_speed_gate<Real, TK, Trig>(params, exc, qvel, 1u, ctrl);
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class StatePtr, class ScratchPtr,
          class PtrQpos, class PtrQvel, class PtrBias, class PtrQacc,
          class PtrRowAdr, class PtrRowNnz, class PtrColInd, class PtrM,
          class PtrCtrl>
BIKE_HD void spindle_m2k2_pass1_perturb0(
    ParamsPtr params,
    PtrQpos qpos,
    PtrQvel qvel,
    PtrBias qfrc_bias,
    PtrQacc qacc,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M,
    StatePtr state,
    ScratchPtr scratch,
    PtrCtrl ctrl
) noexcept {
    spindle_m2k2_pass1_perturb0<Real, TK, Trig>(
        params, qpos, qvel, qfrc_bias, qacc, 0, M_rowadr, M_rownnz, M_colind, d_M,
        state, scratch, ctrl);
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class StatePtr, class ScratchPtr,
          class PtrQpos, class PtrQvel, class PtrBias, class PtrQacc,
          class PtrRowAdr, class PtrRowNnz, class PtrColInd, class PtrM,
          class PtrCtrl>
BIKE_HD void spindle_m2k2_pass2_perturb1(
    ParamsPtr params,
    PtrQpos qpos,
    PtrQvel qvel,
    PtrBias qfrc_bias,
    PtrQacc qacc,
    i32 qM_row_stride,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M_or_qM,
    StatePtr state,
    ScratchPtr scratch,
    PtrCtrl ctrl
) noexcept {
    if (!validate_spindle_m2k2_params(params) ||
        scratch->active_newton == 0 || scratch->converged != 0) {
        return;
    }
    if (scratch->adaptive_mode != 0) {
        const PressStateCore<Real> P_mid = compute_press_state_m<Real>(
            params, scratch, qM_row_stride, M_rowadr, M_rownnz, M_colind, d_M_or_qM,
            qacc, qfrc_bias);
        const Real rm0 = P_mid.f_passive[0] - scratch->ps_trial.f_passive[0];
        const Real rm1 = P_mid.f_passive[1] - scratch->ps_trial.f_passive[1];
        const Real rm2 = P_mid.max_total_press_n - scratch->ps_trial.max_total_press_n;
        const Real err_m = TK::max(TK::abs(rm0), TK::max(TK::abs(rm1), TK::abs(rm2)));
        if (err_m <= params->conv_eps) {
            scratch->ps0 = scratch->ps_trial;
            scratch->P = P_mid;
            scratch->converged = 1;
            return;
        }
        PressStateCore<Real> ps2{};
        const Real dx0 = scratch->ps_trial.f_passive[0] - scratch->ps0.f_passive[0];
        const Real dx1 = scratch->ps_trial.f_passive[1] - scratch->ps0.f_passive[1];
        const Real dP0 = P_mid.f_passive[0] - scratch->P.f_passive[0];
        const Real dP1 = P_mid.f_passive[1] - scratch->P.f_passive[1];
        const Real den0 = (TK::abs(dx0) > BIKE_REAL(Real, 0.5))
            ? (TK::clamp(dP0 / dx0, -BIKE_REAL(Real, 0.6), BIKE_REAL(Real, 0.5)) - BIKE_REAL(Real, 1.0))
            : -BIKE_REAL(Real, 1.0);
        const Real den1 = (TK::abs(dx1) > BIKE_REAL(Real, 0.5))
            ? (TK::clamp(dP1 / dx1, -BIKE_REAL(Real, 0.6), BIKE_REAL(Real, 0.5)) - BIKE_REAL(Real, 1.0))
            : -BIKE_REAL(Real, 1.0);
        ps2.f_passive[0] = scratch->ps_trial.f_passive[0] +
            TK::clamp(-rm0 / den0, -BIKE_REAL(Real, 400.0), BIKE_REAL(Real, 400.0));
        ps2.f_passive[1] = scratch->ps_trial.f_passive[1] +
            TK::clamp(-rm1 / den1, -BIKE_REAL(Real, 400.0), BIKE_REAL(Real, 400.0));
        ps2.max_total_press_n = P_mid.max_total_press_n;

        scratch->ps0 = scratch->ps_trial;
        scratch->P = P_mid;
        scratch->ps_trial = ps2;

        Real req[kNumSpindleJoints]{};
        Real prio[kNumSpindleJoints]{};
        Real exc[kNumSpindleJoints]{};
        populate_leg_torques_core<Real, TK, Trig>(
            params, scratch, ps2, 0u, qvel, state, req, prio);
        limit_torques_core<Real, TK, Trig>(params, req, prio, qpos, qvel, exc);
        write_ctrl_with_speed_gate<Real, TK, Trig>(params, exc, qvel, 1u, ctrl);
        return;
    }

    if (scratch->perturbed0 != 0) {
        scratch->P0 = compute_press_state_m<Real>(
            params, scratch, qM_row_stride, M_rowadr, M_rownnz, M_colind, d_M_or_qM,
            qacc, qfrc_bias);
    } else {
        scratch->P0 = scratch->P;
    }

    PressStateCore<Real> ps_p1 = scratch->ps0;
    ps_p1.f_passive[1] += params->newton_eps;
    Real req[kNumSpindleJoints]{};
    Real prio[kNumSpindleJoints]{};
    populate_leg_torques_core<Real, TK, Trig>(
        params, scratch, ps_p1, 0u, qvel, state, req, prio);

    u8 same = 1;
    for (i32 j = 0; j < 4; ++j) {
        if (Trig::flush_subnorm(req[5 + j]) != scratch->base_leg_req[j] ||
            Trig::flush_subnorm(prio[5 + j]) != scratch->base_leg_prio[j]) {
            same = 0;
        }
    }
    if (same != 0) {
        scratch->perturbed1 = 0;
        return;
    }

    scratch->perturbed1 = 1;
    Real exc[kNumSpindleJoints]{};
    limit_torques_core<Real, TK, Trig>(params, req, prio, qpos, qvel, exc);
    write_ctrl_with_speed_gate<Real, TK, Trig>(params, exc, qvel, 1u, ctrl);
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class StatePtr, class ScratchPtr,
          class PtrQpos, class PtrQvel, class PtrBias, class PtrQacc,
          class PtrRowAdr, class PtrRowNnz, class PtrColInd, class PtrM,
          class PtrCtrl>
BIKE_HD void spindle_m2k2_pass2_perturb1(
    ParamsPtr params,
    PtrQpos qpos,
    PtrQvel qvel,
    PtrBias qfrc_bias,
    PtrQacc qacc,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M,
    StatePtr state,
    ScratchPtr scratch,
    PtrCtrl ctrl
) noexcept {
    spindle_m2k2_pass2_perturb1<Real, TK, Trig>(
        params, qpos, qvel, qfrc_bias, qacc, 0, M_rowadr, M_rownnz, M_colind, d_M,
        state, scratch, ctrl);
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class StatePtr, class ScratchPtr,
          class PtrQpos, class PtrQvel, class PtrBias, class PtrQacc,
          class PtrRowAdr, class PtrRowNnz, class PtrColInd, class PtrM,
          class PtrCtrl>
BIKE_HD void spindle_m2k2_pass3_commit(
    ParamsPtr params,
    PtrQpos qpos,
    PtrQvel qvel,
    PtrBias qfrc_bias,
    PtrQacc qacc,
    i32 qM_row_stride,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M_or_qM,
    StatePtr state,
    ScratchPtr scratch,
    PtrCtrl ctrl
) noexcept {
    if (!validate_spindle_m2k2_params(params) || scratch->active_newton == 0) {
        return;
    }
    PressStateCore<Real> ps_final = scratch->ps0;
    if (scratch->converged == 0 && scratch->adaptive_mode != 0) {
        const PressStateCore<Real> P2 = compute_press_state_m<Real>(
            params, scratch, qM_row_stride, M_rowadr, M_rownnz, M_colind, d_M_or_qM,
            qacc, qfrc_bias);
        const Real r2_0 = P2.f_passive[0] - scratch->ps_trial.f_passive[0];
        const Real r2_1 = P2.f_passive[1] - scratch->ps_trial.f_passive[1];
        const Real r2_2 = P2.max_total_press_n - scratch->ps_trial.max_total_press_n;
        const Real err2 = TK::max(TK::abs(r2_0), TK::max(TK::abs(r2_1), TK::abs(r2_2)));
        if (err2 <= params->conv_eps) {
            ps_final = scratch->ps_trial;
        } else {
            const Real dx0 = scratch->ps_trial.f_passive[0] - scratch->ps0.f_passive[0];
            const Real dx1 = scratch->ps_trial.f_passive[1] - scratch->ps0.f_passive[1];
            const Real dP0 = P2.f_passive[0] - scratch->P.f_passive[0];
            const Real dP1 = P2.f_passive[1] - scratch->P.f_passive[1];
            const Real den0 = (TK::abs(dx0) > BIKE_REAL(Real, 0.25))
                ? (TK::clamp(dP0 / dx0, -BIKE_REAL(Real, 0.8), BIKE_REAL(Real, 0.6)) - BIKE_REAL(Real, 1.0))
                : -BIKE_REAL(Real, 1.0);
            const Real den1 = (TK::abs(dx1) > BIKE_REAL(Real, 0.25))
                ? (TK::clamp(dP1 / dx1, -BIKE_REAL(Real, 0.8), BIKE_REAL(Real, 0.6)) - BIKE_REAL(Real, 1.0))
                : -BIKE_REAL(Real, 1.0);
            ps_final.f_passive[0] = scratch->ps_trial.f_passive[0] +
                TK::clamp(-r2_0 / den0, -BIKE_REAL(Real, 400.0), BIKE_REAL(Real, 400.0));
            ps_final.f_passive[1] = scratch->ps_trial.f_passive[1] +
                TK::clamp(-r2_1 / den1, -BIKE_REAL(Real, 400.0), BIKE_REAL(Real, 400.0));
            ps_final.max_total_press_n = P2.max_total_press_n;
        }
    } else if (scratch->converged == 0) {
        PressStateCore<Real> P1 = scratch->P;
        if (scratch->perturbed1 != 0) {
            P1 = compute_press_state_m<Real>(
                params, scratch, qM_row_stride, M_rowadr, M_rownnz, M_colind, d_M_or_qM,
                qacc, qfrc_bias);
        }
        const PressStateCore<Real> P = scratch->P;
        const PressStateCore<Real> P0 = scratch->P0;
        const Real eps = params->newton_eps;
        const Real r0 = P.f_passive[0] - ps_final.f_passive[0];
        const Real r1 = P.f_passive[1] - ps_final.f_passive[1];
        const Real r2 = P.max_total_press_n - ps_final.max_total_press_n;
        const Real J00 = (P0.f_passive[0] - P.f_passive[0]) / eps - BIKE_REAL(Real, 1.0);
        const Real J10 = (P0.f_passive[1] - P.f_passive[1]) / eps;
        const Real J20 = (P0.max_total_press_n - P.max_total_press_n) / eps;
        const Real J01 = (P1.f_passive[0] - P.f_passive[0]) / eps;
        const Real J11 = (P1.f_passive[1] - P.f_passive[1]) / eps - BIKE_REAL(Real, 1.0);
        const Real J21 = (P1.max_total_press_n - P.max_total_press_n) / eps;
        const Real det = J00 * J11 - J01 * J10;
        if (TK::abs(det) >= params->det_eps) {
            const Real dp0 = -(J11 * r0 - J01 * r1) / det;
            const Real dp1 = -(-J10 * r0 + J00 * r1) / det;
            const Real dp2 = r2 + J20 * dp0 + J21 * dp1;
            ps_final.f_passive[0] += dp0;
            ps_final.f_passive[1] += dp1;
            ps_final.max_total_press_n = TK::max(
                ps_final.max_total_press_n + dp2, BIKE_REAL(Real, 0.0));
        }
    }

    Real req[kNumSpindleJoints]{};
    Real prio[kNumSpindleJoints]{};
    Real exc[kNumSpindleJoints]{};
    populate_leg_torques_core<Real, TK, Trig>(
        params, scratch, ps_final, scratch->advance, qvel, state, req, prio);
    limit_torques_core<Real, TK, Trig>(params, req, prio, qpos, qvel, exc);
    for (i32 i = 0; i < kNumSpindleJoints; ++i) {
        scratch->last_req[i] = req[i];
        scratch->last_exc[i] = exc[i];
    }
    if (scratch->advance != 0) {
        for (i32 i = 0; i < kNumSpindleJoints; ++i) {
            state->held_torques[i] = exc[i];
        }
        state->has_held_torques = 1;
        if (TK::isfinite(ps_final.f_passive[0]) &&
            TK::isfinite(ps_final.f_passive[1]) &&
            TK::isfinite(ps_final.max_total_press_n)) {
            state->prev_ps.f_passive[0] = ps_final.f_passive[0];
            state->prev_ps.f_passive[1] = ps_final.f_passive[1];
            state->prev_ps.max_total_press_n =
                TK::max(ps_final.max_total_press_n, BIKE_REAL(Real, 0.0));
            state->has_prev_ps = 1;
        } else {
            state->has_prev_ps = 0;
        }
    }
    write_ctrl_with_speed_gate<Real, TK, Trig>(params, exc, qvel, 1u, ctrl);
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class StatePtr, class ScratchPtr,
          class PtrQpos, class PtrQvel, class PtrBias, class PtrQacc,
          class PtrRowAdr, class PtrRowNnz, class PtrColInd, class PtrM,
          class PtrCtrl>
BIKE_HD void spindle_m2k2_pass3_commit(
    ParamsPtr params,
    PtrQpos qpos,
    PtrQvel qvel,
    PtrBias qfrc_bias,
    PtrQacc qacc,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M,
    StatePtr state,
    ScratchPtr scratch,
    PtrCtrl ctrl
) noexcept {
    spindle_m2k2_pass3_commit<Real, TK, Trig>(
        params, qpos, qvel, qfrc_bias, qacc, 0, M_rowadr, M_rownnz, M_colind, d_M,
        state, scratch, ctrl);
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
