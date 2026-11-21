// native/src/physics_core/spindle_m2k2_upload.hpp — Standalone host-only
// parameter loader populating SpindleM2K2Params<Real> from mjModel + caller
// fk-evaluated mjData(qpos=0) and JSON strength/envelope tables.
// Links ONLY libmujoco (zero bike_core, bike_io, or numeric_numpy.hpp deps).
#pragma once

#if defined(__METAL_VERSION__) || defined(__CUDA_ARCH__)
#error "spindle_m2k2_upload.hpp is host-only"
#endif

#include "../model_access.hpp"
#include "spindle_m2k2.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace bike::physics_core {

inline constexpr std::array<const char *, kNumSpindleJoints> kSpindleJointNames{
    "rider_torso_hinge",
    "rider_shoulder_left",
    "rider_elbow_left",
    "rider_shoulder_right",
    "rider_elbow_right",
    "rider_hip_front",
    "rider_knee_front",
    "rider_hip_rear",
    "rider_knee_rear",
};

struct SpindleM2K2ScalarConfig {
    double joint_kp_nm_rad = 600.0;
    double joint_kd_nms_rad = 15.0;
    double leg_active_damping_nms_rad = 35.0;
    double joint_limit_nm = 100.0;
    double joint_speed_limit_rad_s = 20.0;
    double joint_power_limit_w = 250.0;
    bool has_active_positive_power_limit = true;
    double active_positive_power_limit_w = 450.0;
    double pedal_torque_ripple = 0.5;
    double return_foot_preload_n = 40.0;
    double pedal_press_margin_n = 25.0;
    double weld_servo_gain = 0.05;
    double weld_servo_limit_nm = 25.0;
    double reposition_damping_nms_rad = 12.0;
    double reposition_torque_limit_nm = 60.0;
    double arm_reach_fraction = 0.92;
    double crank_length_m = 0.165;
    int control_period_steps = 1;
    bool grip_is_spring = false;
    bool has_strength = true;
    bool compute_lean_limit = true;
};

struct SpindleUploadTiming {
    double crank_length_m = 0.165;
    int control_period_steps = 1;
};

struct TorqueCurveInput {
    std::vector<double> angles;
    std::vector<double> torques;
    double vmax = 0.0;
    double hill_c = 0.0;
    double eccentric_ratio = 0.0;
};

struct SpindleJointStrengthInput {
    int direction = 1;
    TorqueCurveInput positive;
    TorqueCurveInput negative;
};

struct SpindleJointEnvelopeInput {
    int direction = 1;
    double minimum_anatomical_rad = 0.0;
    double maximum_anatomical_rad = 0.0;
};

struct SpindleReferenceTables {
    std::array<SpindleJointStrengthInput, kNumSpindleJoints> strength{};
    std::array<SpindleJointEnvelopeInput, kNumSpindleJoints> envelopes{};
};

struct SpindleReferenceJsonTexts {
    std::string_view strength_json;
    std::string_view envelope_json;
};

[[nodiscard]] inline SpindleM2K2ScalarConfig make_viewer_welded_spindle_scalar_config(
    int control_period_steps = 1
) {
    SpindleM2K2ScalarConfig cfg{};
    cfg.control_period_steps = std::max(1, control_period_steps);
    return cfg;
}

namespace upload_detail {

// Standalone host FP64 toolkit using standard <cmath> (zero BIKE_NUMPY_* macros).
struct UploadDoubleToolkit {
    using Real = double;

    [[nodiscard]] static double sqrt(double x) noexcept {
        return std::sqrt(x);
    }
    [[nodiscard]] static double abs(double x) noexcept {
        return std::abs(x);
    }
    [[nodiscard]] static bool isfinite(double x) noexcept {
        return std::isfinite(x);
    }
    [[nodiscard]] static bool isnan(double x) noexcept {
        return std::isnan(x);
    }
    [[nodiscard]] static double min(double a, double b) noexcept {
        return std::min(a, b);
    }
    [[nodiscard]] static double max(double a, double b) noexcept {
        return std::max(a, b);
    }
    [[nodiscard]] static double clamp(double x, double lo, double hi) noexcept {
        return x < lo ? lo : (x > hi ? hi : x);
    }
    [[nodiscard]] static double hypot2(double a, double b) noexcept {
        return std::hypot(a, b);
    }
};

struct UploadDoubleTrig {
    static double flush_subnorm(double x) noexcept {
        return x;
    }
    template <class Real, class TK>
    static Real sin_val(Real x) noexcept {
        return static_cast<Real>(std::sin(static_cast<double>(x)));
    }
    template <class Real, class TK>
    static Real cos_val(Real x) noexcept {
        return static_cast<Real>(std::cos(static_cast<double>(x)));
    }
    template <class Real, class TK>
    static Real atan_val(Real x) noexcept {
        return static_cast<Real>(std::atan(static_cast<double>(x)));
    }
    template <class Real, class TK>
    static Real atan2_val(Real y, Real x) noexcept {
        return static_cast<Real>(std::atan2(static_cast<double>(y), static_cast<double>(x)));
    }
    template <class Real, class TK>
    static Real acos_val(Real x) noexcept {
        const double c = std::clamp(static_cast<double>(x), -1.0, 1.0);
        return static_cast<Real>(std::acos(c));
    }
};

[[nodiscard]] inline std::size_t to_idx(int id) noexcept {
    return static_cast<std::size_t>(id);
}

[[nodiscard]] inline int require_obj(const mjModel *m, int obj_type, const char *name) {
    const int id = mj_name2id(m, obj_type, name);
    if (id < 0) {
        throw std::invalid_argument(std::string("spindle_m2k2_upload missing model object: ") + name);
    }
    return id;
}

[[nodiscard]] inline double planar_angle_f64(double dx, double dz) noexcept {
    return std::atan2(-dx, -dz);
}

inline void validate_scalar_config(const SpindleM2K2ScalarConfig &cfg) {
    const auto require_nonneg = [](double v, const char *name) {
        if (!std::isfinite(v) || v < 0.0) {
            throw std::invalid_argument(std::string("spindle_m2k2_upload invalid scalar ") + name);
        }
    };
    require_nonneg(cfg.joint_kp_nm_rad, "joint_kp_nm_rad");
    require_nonneg(cfg.joint_kd_nms_rad, "joint_kd_nms_rad");
    require_nonneg(cfg.leg_active_damping_nms_rad, "leg_active_damping_nms_rad");
    require_nonneg(cfg.joint_limit_nm, "joint_limit_nm");
    require_nonneg(cfg.joint_speed_limit_rad_s, "joint_speed_limit_rad_s");
    require_nonneg(cfg.joint_power_limit_w, "joint_power_limit_w");
    if (cfg.has_active_positive_power_limit) {
        require_nonneg(cfg.active_positive_power_limit_w, "active_positive_power_limit_w");
    }
    if (!std::isfinite(cfg.pedal_torque_ripple)) {
        throw std::invalid_argument("spindle_m2k2_upload invalid pedal_torque_ripple");
    }
    require_nonneg(cfg.return_foot_preload_n, "return_foot_preload_n");
    require_nonneg(cfg.pedal_press_margin_n, "pedal_press_margin_n");
    require_nonneg(cfg.weld_servo_gain, "weld_servo_gain");
    require_nonneg(cfg.weld_servo_limit_nm, "weld_servo_limit_nm");
    require_nonneg(cfg.reposition_damping_nms_rad, "reposition_damping_nms_rad");
    require_nonneg(cfg.reposition_torque_limit_nm, "reposition_torque_limit_nm");
    if (!std::isfinite(cfg.arm_reach_fraction) ||
        cfg.arm_reach_fraction <= 0.0 || cfg.arm_reach_fraction > 1.0) {
        throw std::invalid_argument("spindle_m2k2_upload invalid arm_reach_fraction");
    }
    if (!std::isfinite(cfg.crank_length_m) || cfg.crank_length_m <= 0.0) {
        throw std::invalid_argument("spindle_m2k2_upload invalid crank_length_m");
    }
}

template <class Real>
void pack_torque_curve(
    const TorqueCurveInput &in,
    TorqueCurveSpec<Real> &out
) {
    if (in.angles.size() < 2 || in.angles.size() != in.torques.size() ||
        in.angles.size() > static_cast<std::size_t>(kMaxStrengthKnots)) {
        throw std::invalid_argument("invalid torque curve knot count");
    }
    if (!std::isfinite(in.vmax) || in.vmax <= 0.0) {
        throw std::invalid_argument("torque curve vmax_rad_s must be finite and > 0");
    }
    if (!std::isfinite(in.hill_c) || in.hill_c <= 0.0) {
        throw std::invalid_argument("torque curve hill_c must be finite and > 0");
    }
    if (!std::isfinite(in.eccentric_ratio) || in.eccentric_ratio < 1.0) {
        throw std::invalid_argument("torque curve eccentric_ratio must be finite and >= 1.0");
    }
    for (std::size_t k = 0; k < in.angles.size(); ++k) {
        if (!std::isfinite(in.angles[k]) || !std::isfinite(in.torques[k])) {
            throw std::invalid_argument("torque curve knots must be finite");
        }
        if (in.torques[k] < 0.0) {
            throw std::invalid_argument("torque curve torques_nm must be >= 0");
        }
        if (k > 0 && !(in.angles[k] > in.angles[k - 1])) {
            throw std::invalid_argument("torque curve angles_rad must be strictly increasing");
        }
    }
    out.knot_count = static_cast<i32>(in.angles.size());
    out.vmax_rad_s = static_cast<Real>(in.vmax);
    out.hill_c = static_cast<Real>(in.hill_c);
    out.eccentric_ratio = static_cast<Real>(in.eccentric_ratio);
    const std::span<Real> dst_a(out.angles_rad);
    const std::span<Real> dst_t(out.torques_nm);
    for (std::size_t k = 0; k < in.angles.size(); ++k) {
        dst_a[k] = static_cast<Real>(in.angles[k]);
        dst_t[k] = static_cast<Real>(in.torques[k]);
    }
}

struct EnvelopeRange {
    double lo_q = 0.0;
    double hi_q = 0.0;
    double neutral_anatomical_rad = 0.0;
    int direction = 1;
};

struct LeanLimitSearchOptions {
    double crank_m = 0.165;
    double tdc_phase_rad = -std::numbers::pi / 2.0;
    double max_rad = 0.8;
    double tolerance = 1e-3;
};

struct JointEnvelopeValidationInput {
    std::size_t slot = 0;
    int env_direction = 1;
    double min_anat_rad = 0.0;
    double max_anat_rad = 0.0;
    int strength_direction = 1;
    bool check_strength_direction = false;
    u8 joint_has_range = 0;
    double joint_lo_q = 0.0;
    double joint_hi_q = 0.0;
};

inline void validate_joint_envelope(const JointEnvelopeValidationInput &in) {
    if (in.env_direction != 1 && in.env_direction != -1) {
        throw std::invalid_argument("joint envelope direction must be +1 or -1");
    }
    if (!std::isfinite(in.min_anat_rad) || !std::isfinite(in.max_anat_rad) ||
        !(in.min_anat_rad < in.max_anat_rad)) {
        throw std::invalid_argument("joint envelope anatomical bounds must be finite with min < max");
    }
    if (in.check_strength_direction && in.env_direction != in.strength_direction) {
        throw std::invalid_argument(
            std::string("joint strength/envelope direction mismatch for ") +
            kSpindleJointNames[in.slot]);
    }
    if (in.joint_has_range == 0 || !std::isfinite(in.joint_lo_q) ||
        !std::isfinite(in.joint_hi_q) || !(in.joint_lo_q < in.joint_hi_q)) {
        throw std::invalid_argument(
            std::string("joint must have finite limited range for envelope on ") +
            kSpindleJointNames[in.slot]);
    }
}

[[nodiscard]] inline double compute_lean_limit_from_pose(
    const std::array<double, 2> &hip,
    const std::array<double, 2> &shoulder,
    const std::array<double, 2> &elbow,
    const std::array<double, 2> &grip,
    const std::array<std::array<double, 2>, 2> &knee,
    const std::array<std::array<double, 2>, 2> &pedal,
    const std::array<EnvelopeRange, kNumSpindleJoints> &envelopes,
    const LeanLimitSearchOptions &opts
) {
    const double trunk_x = shoulder[0] - hip[0];
    const double trunk_z = shoulder[1] - hip[1];
    const double upper_x = elbow[0] - shoulder[0];
    const double upper_z = elbow[1] - shoulder[1];
    const double lower_x = grip[0] - elbow[0];
    const double lower_z = grip[1] - elbow[1];
    const double arm_upper = std::hypot(upper_x, upper_z);
    const double arm_lower = std::hypot(lower_x, lower_z);
    const double upper0 = std::atan2(upper_z, upper_x);
    const double lower0 = std::atan2(lower_z, lower_x);
    const int branch = std::sin(lower0 - upper0) >= 0.0 ? 1 : -1;
    const double crank_x = 0.5 * (pedal[0][0] + pedal[1][0]);
    const double crank_z = 0.5 * (pedal[0][1] + pedal[1][1]);

    std::array<double, 2> hips_at_tdc{};
    for (std::size_t s = 0; s < 2; ++s) {
        const double thigh_x = knee[s][0] - hip[0];
        const double thigh_z = knee[s][1] - hip[1];
        const double link_x = pedal[s][0] - knee[s][0];
        const double link_z = pedal[s][1] - knee[s][1];
        const double chord_x = pedal[s][0] - hip[0];
        const double chord_z = pedal[s][1] - hip[1];
        const double ahead = chord_x * thigh_z - chord_z * thigh_x;
        const double thigh_m = std::hypot(thigh_x, thigh_z);
        const double link_m = std::hypot(link_x, link_z);
        const double knee_sign = ahead >= 0.0 ? 1.0 : -1.0;
        const double thigh_a0 = planar_angle_f64(thigh_x, thigh_z);
        const double link_a0 = planar_angle_f64(link_x, link_z);
        const double side_sign = (s == 0) ? 1.0 : -1.0;
        const double ph = opts.tdc_phase_rad + (s == 0 ? 0.0 : std::numbers::pi);
        const double sp_x = crank_x + side_sign * opts.crank_m * std::cos(ph);
        const double sp_z = crank_z - side_sign * opts.crank_m * std::sin(ph);
        std::array<double, 2> q{};
        u8 valid = 0;
        leg_joint_q_core<double, UploadDoubleToolkit, UploadDoubleTrig>(
            thigh_m, link_m, knee_sign, thigh_a0, link_a0,
            hip[0], hip[1], sp_x, sp_z, 0.0, q.data(), valid, 1e-9);
        if (valid == 0) {
            throw std::invalid_argument("unreachable TDC leg pose in lean_limit");
        }
        hips_at_tdc[s] = q[0];
    }

    const auto within = [&](std::size_t slot, double q) {
        return envelopes[slot].lo_q - 1e-12 <= q && q <= envelopes[slot].hi_q + 1e-12;
    };
    const auto feasible = [&](double lean) {
        const double c = std::cos(lean);
        const double s = std::sin(lean);
        const double sh_x = hip[0] + c * trunk_x + s * trunk_z;
        const double sh_z = hip[1] - s * trunk_x + c * trunk_z;
        const double gx = grip[0] - sh_x;
        const double gz = grip[1] - sh_z;
        const double agx = c * gx - s * gz;
        const double agz = s * gx + c * gz;
        const auto ik = two_link_ik<double, UploadDoubleToolkit, UploadDoubleTrig>(
            agx, agz, arm_upper, arm_lower, branch, 1e-10, 1e-15);
        if (ik.saturated != 0 || !within(kSpindleSlotTorso, lean)) {
            return false;
        }
        const double sh_q = upper0 - ik.angle0;
        const double el_q = (lower0 - upper0) - ik.angle1;
        if (!within(kSpindleSlotShoulderLeft, sh_q) || !within(kSpindleSlotElbowLeft, el_q) ||
            !within(kSpindleSlotShoulderRight, sh_q) || !within(kSpindleSlotElbowRight, el_q)) {
            return false;
        }
        return within(kSpindleSlotHipFront, hips_at_tdc[0] - lean) &&
               within(kSpindleSlotHipRear, hips_at_tdc[1] - lean);
    };

    if (!feasible(0.0)) {
        throw std::invalid_argument("neutral seated posture lies outside reachable joint ROM");
    }
    if (feasible(opts.max_rad)) {
        return opts.max_rad;
    }
    double low = 0.0;
    double high = opts.max_rad;
    while (high - low > opts.tolerance) {
        const double mid = 0.5 * (low + high);
        if (feasible(mid)) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return low;
}

template <class Real>
void populate_model_topology_common(
    const mjModel *m,
    const SpindleM2K2ScalarConfig &cfg,
    SpindleM2K2Params<Real> &out
) {
    if (m == nullptr) {
        throw std::invalid_argument("build_spindle_m2k2_params requires non-null mjModel");
    }
    validate_scalar_config(cfg);

    out.nq = static_cast<i32>(m->nq);
    out.nv = static_cast<i32>(m->nv);
    out.nu = static_cast<i32>(m->nu);
    out.nbody = static_cast<i32>(m->nbody);
    out.njnt = static_cast<i32>(m->njnt);
    out.nsite = static_cast<i32>(m->nsite);
    if (out.nv <= 0 || out.nv > kMaxNv ||
        out.nq <= 0 || out.nq > kMaxNq ||
        out.nu < 0 || out.nu > kMaxNu ||
        out.nbody <= 0 || out.nbody > kMaxNbody ||
        out.njnt <= 0 || out.njnt > kMaxNjnt ||
        out.nsite < 0 || out.nsite > kMaxNsite) {
        throw std::invalid_argument("mjModel dimensions out of physics_core kMax* bounds");
    }

    out.frame_body = require_obj(m, mjOBJ_BODY, "frame");
    out.pelvis_body = require_obj(m, mjOBJ_BODY, "rider_pelvis");
    out.torso_body = require_obj(m, mjOBJ_BODY, "rider_torso");
    out.steer_body = require_obj(m, mjOBJ_BODY, "steer");
    out.crank_body = require_obj(m, mjOBJ_BODY, "crank");
    out.pedal_bodies[0] = require_obj(m, mjOBJ_BODY, "pedal_front");
    out.pedal_bodies[1] = require_obj(m, mjOBJ_BODY, "pedal_rear");
    out.upper_arm_bodies[0] = require_obj(m, mjOBJ_BODY, "rider_upper_arm_left");
    out.upper_arm_bodies[1] = require_obj(m, mjOBJ_BODY, "rider_upper_arm_right");
    out.forearm_bodies[0] = require_obj(m, mjOBJ_BODY, "rider_forearm_left");
    out.forearm_bodies[1] = require_obj(m, mjOBJ_BODY, "rider_forearm_right");

    out.crank_joint = require_obj(m, mjOBJ_JOINT, "crank_spin");
    out.torso_joint = require_obj(m, mjOBJ_JOINT, "rider_torso_hinge");
    out.frame_pitch_joint = require_obj(m, mjOBJ_JOINT, "root_pitch");
    out.rider_pitch_joint = require_obj(m, mjOBJ_JOINT, "rider_root_pitch");
    out.hip_joints[0] = require_obj(m, mjOBJ_JOINT, "rider_hip_front");
    out.hip_joints[1] = require_obj(m, mjOBJ_JOINT, "rider_hip_rear");
    out.knee_joints[0] = require_obj(m, mjOBJ_JOINT, "rider_knee_front");
    out.knee_joints[1] = require_obj(m, mjOBJ_JOINT, "rider_knee_rear");

    out.grip_sites[0] = require_obj(m, mjOBJ_SITE, "site_rider_grip_left");
    out.grip_sites[1] = require_obj(m, mjOBJ_SITE, "site_rider_grip_right");

    const auto qpos_adrs = model_access::readonly_buffer(m->jnt_qposadr, m->njnt, "jnt_qposadr");
    const auto dof_adrs = model_access::readonly_buffer(m->jnt_dofadr, m->njnt, "jnt_dofadr");
    const auto jnt_bodyid = model_access::readonly_buffer(m->jnt_bodyid, m->njnt, "jnt_bodyid");
    const auto jnt_limited = model_access::readonly_buffer(m->jnt_limited, m->njnt, "jnt_limited");
    const auto jnt_range = model_access::readonly_buffer(m->jnt_range, m->njnt * 2, "jnt_range");
    const auto submass = model_access::readonly_buffer(m->body_subtreemass, m->nbody, "body_subtreemass");
    const auto gravity = model_access::readonly_buffer(std::data(m->opt.gravity), 3, "opt.gravity");

    for (std::size_t c = 0; c < 3; ++c) {
        std::span{out.gravity}[c] = static_cast<Real>(gravity[c]);
    }
    for (std::size_t s = 0; s < 2; ++s) {
        std::span{out.hip_bodies}[s] = jnt_bodyid[to_idx(std::span{out.hip_joints}[s])];
        std::span{out.knee_bodies}[s] = jnt_bodyid[to_idx(std::span{out.knee_joints}[s])];
        std::span{out.hip_subtreemass}[s] =
            static_cast<Real>(submass[to_idx(std::span{out.hip_bodies}[s])]);
        std::span{out.knee_subtreemass}[s] =
            static_cast<Real>(submass[to_idx(std::span{out.knee_bodies}[s])]);
    }

    const int fx_j = mj_name2id(m, mjOBJ_JOINT, "root_x");
    const int fz_j = mj_name2id(m, mjOBJ_JOINT, "root_z");
    const int rx_j = mj_name2id(m, mjOBJ_JOINT, "rider_root_x");
    const int rz_j = mj_name2id(m, mjOBJ_JOINT, "rider_root_z");
    out.frame_root_x_dof = (fx_j < 0) ? -1 : dof_adrs[to_idx(fx_j)];
    out.frame_root_z_dof = (fz_j < 0) ? -1 : dof_adrs[to_idx(fz_j)];
    out.frame_pitch_dof = dof_adrs[to_idx(out.frame_pitch_joint)];
    out.rider_root_x_dof = (rx_j < 0) ? -1 : dof_adrs[to_idx(rx_j)];
    out.rider_root_z_dof = (rz_j < 0) ? -1 : dof_adrs[to_idx(rz_j)];
    out.rider_pitch_dof = dof_adrs[to_idx(out.rider_pitch_joint)];
    out.crank_spin_dof = dof_adrs[to_idx(out.crank_joint)];

    const std::span<i32> dst_qadr(out.joint_qpos_adr);
    const std::span<i32> dst_dadr(out.joint_dof_adr);
    const std::span<i32> dst_aid(out.joint_actuator_id);
    const std::span<u8> dst_has_rng(out.joint_has_range);
    const std::span<Real> dst_rlo(out.joint_range_lo);
    const std::span<Real> dst_rhi(out.joint_range_hi);
    for (std::size_t i = 0; i < static_cast<std::size_t>(kNumSpindleJoints); ++i) {
        const int jid = require_obj(m, mjOBJ_JOINT, kSpindleJointNames[i]);
        const std::string act_name = std::string("act_") + kSpindleJointNames[i];
        const int aid = require_obj(m, mjOBJ_ACTUATOR, act_name.c_str());
        dst_qadr[i] = qpos_adrs[to_idx(jid)];
        dst_dadr[i] = dof_adrs[to_idx(jid)];
        dst_aid[i] = aid;
        dst_has_rng[i] = (jnt_limited[to_idx(jid)] != 0) ? 1u : 0u;
        if (dst_has_rng[i] != 0) {
            dst_rlo[i] = static_cast<Real>(jnt_range[to_idx(jid) * 2 + 0]);
            dst_rhi[i] = static_cast<Real>(jnt_range[to_idx(jid) * 2 + 1]);
        }
    }

    // Extract weld_grip_offset from connect_grip_{left,right} equalities
    const std::array<const char *, 2> grip_eq_names{"connect_grip_left", "connect_grip_right"};
    const auto eq_data = model_access::readonly_buffer(m->eq_data, m->neq * mjNEQDATA, "eq_data");
    for (std::size_t s = 0; s < 2; ++s) {
        const int eq = mj_name2id(m, mjOBJ_EQUALITY, grip_eq_names[s]);
        if (eq < 0) {
            if (!cfg.grip_is_spring) {
                throw std::invalid_argument(
                    std::string("spindle_m2k2_upload missing required grip weld equality: ") +
                    grip_eq_names[s]);
            }
            continue;
        }
        const std::size_t base = to_idx(eq) * mjNEQDATA;
        for (std::size_t c = 0; c < 3; ++c) {
            const double val = eq_data[base + 3 + c];
            if (!std::isfinite(val)) {
                throw std::invalid_argument("spindle_m2k2_upload non-finite grip equality anchor");
            }
            std::span{std::span{out.weld_grip_offset}[s]}[c] = static_cast<Real>(val);
        }
    }

    out.crank_length_m = static_cast<Real>(cfg.crank_length_m);
    out.side_sign[0] = static_cast<Real>(1.0);
    out.side_sign[1] = static_cast<Real>(-1.0);
    out.joint_kp_nm_rad = static_cast<Real>(cfg.joint_kp_nm_rad);
    out.joint_kd_nms_rad = static_cast<Real>(cfg.joint_kd_nms_rad);
    out.leg_active_damping_nms_rad = static_cast<Real>(cfg.leg_active_damping_nms_rad);
    out.joint_limit_nm = static_cast<Real>(cfg.joint_limit_nm);
    out.joint_speed_limit_rad_s = static_cast<Real>(cfg.joint_speed_limit_rad_s);
    out.joint_power_limit_w = static_cast<Real>(cfg.joint_power_limit_w);
    out.has_active_positive_power_limit = cfg.has_active_positive_power_limit ? 1u : 0u;
    out.active_positive_power_limit_w = static_cast<Real>(cfg.active_positive_power_limit_w);
    out.pedal_torque_ripple = static_cast<Real>(cfg.pedal_torque_ripple);
    out.return_foot_preload_n = static_cast<Real>(cfg.return_foot_preload_n);
    out.pedal_press_margin_n = static_cast<Real>(cfg.pedal_press_margin_n);
    out.weld_servo_gain = static_cast<Real>(cfg.weld_servo_gain);
    out.weld_servo_limit_nm = static_cast<Real>(cfg.weld_servo_limit_nm);
    out.reposition_damping_nms_rad = static_cast<Real>(cfg.reposition_damping_nms_rad);
    out.reposition_torque_limit_nm = static_cast<Real>(cfg.reposition_torque_limit_nm);
    out.has_strength = cfg.has_strength ? 1u : 0u;
    out.grip_is_spring = cfg.grip_is_spring ? 1u : 0u;
    out.control_period_steps = std::max(1, cfg.control_period_steps);
}

} // namespace upload_detail

[[nodiscard]] SpindleReferenceTables parse_spindle_reference_json(
    const SpindleReferenceJsonTexts &texts
);

[[nodiscard]] SpindleReferenceTables load_spindle_reference_files(
    const std::filesystem::path &strength_json_path,
    const std::filesystem::path &envelope_json_path
);

// Build SpindleM2K2Params<Real> from mjModel + caller-prepared fk mjData(qpos=0)
// and validated reference tables. Zero mj_makeData / mj_resetData calls.
template <class Real>
[[nodiscard]] SpindleM2K2Params<Real> build_spindle_m2k2_params_from_tables(
    const mjModel *m,
    const mjData *d_qpos0,
    const SpindleReferenceTables &tables,
    const SpindleM2K2ScalarConfig &cfg
) {
    if (d_qpos0 == nullptr) {
        throw std::invalid_argument(
            "build_spindle_m2k2_params_from_tables requires non-null fk-evaluated mjData");
    }
    SpindleM2K2Params<Real> out{};
    upload_detail::populate_model_topology_common(m, cfg, out);

    const auto xanchor = model_access::readonly_buffer(d_qpos0->xanchor, m->njnt * 3, "xanchor");
    const auto xpos = model_access::readonly_buffer(d_qpos0->xpos, m->nbody * 3, "xpos");
    const auto site_xpos = model_access::readonly_buffer(d_qpos0->site_xpos, m->nsite * 3, "site_xpos");
    const auto xmat = model_access::readonly_buffer(d_qpos0->xmat, m->nbody * 9, "xmat");

    const auto xz_anchor = [&](int jid) -> std::array<double, 2> {
        return {xanchor[upload_detail::to_idx(jid) * 3 + 0],
                xanchor[upload_detail::to_idx(jid) * 3 + 2]};
    };
    const auto xz_body = [&](int bid) -> std::array<double, 2> {
        return {xpos[upload_detail::to_idx(bid) * 3 + 0],
                xpos[upload_detail::to_idx(bid) * 3 + 2]};
    };
    const auto xz_site = [&](int sid) -> std::array<double, 2> {
        return {site_xpos[upload_detail::to_idx(sid) * 3 + 0],
                site_xpos[upload_detail::to_idx(sid) * 3 + 2]};
    };

    const std::array<double, 2> hip_xz = xz_anchor(out.hip_joints[0]);
    std::array<std::array<double, 2>, 2> knee_xz{};
    std::array<std::array<double, 2>, 2> pedal_xz{};
    for (std::size_t s = 0; s < 2; ++s) {
        const auto h = xz_anchor(std::span{out.hip_joints}[s]);
        knee_xz[s] = xz_anchor(std::span{out.knee_joints}[s]);
        pedal_xz[s] = xz_body(std::span{out.pedal_bodies}[s]);
        const double thigh_x = knee_xz[s][0] - h[0];
        const double thigh_z = knee_xz[s][1] - h[1];
        const double link_x = pedal_xz[s][0] - knee_xz[s][0];
        const double link_z = pedal_xz[s][1] - knee_xz[s][1];
        const double chord_x = pedal_xz[s][0] - h[0];
        const double chord_z = pedal_xz[s][1] - h[1];
        const double ahead = chord_x * thigh_z - chord_z * thigh_x;
        std::span{out.thigh_m}[s] = static_cast<Real>(std::hypot(thigh_x, thigh_z));
        std::span{out.link_m}[s] = static_cast<Real>(std::hypot(link_x, link_z));
        std::span{out.knee_sign}[s] = static_cast<Real>(ahead >= 0.0 ? 1.0 : -1.0);
        std::span{out.thigh_angle0_rad}[s] =
            static_cast<Real>(upload_detail::planar_angle_f64(thigh_x, thigh_z));
        std::span{out.link_angle0_rad}[s] =
            static_cast<Real>(upload_detail::planar_angle_f64(link_x, link_z));
    }

    const auto shoulder_xz = xz_body(out.upper_arm_bodies[0]);
    const auto elbow_xz = xz_body(out.forearm_bodies[0]);
    const auto grip_xz = xz_site(out.grip_sites[0]);
    const double trunk_x = shoulder_xz[0] - hip_xz[0];
    const double trunk_z = shoulder_xz[1] - hip_xz[1];
    const double upper_x = elbow_xz[0] - shoulder_xz[0];
    const double upper_z = elbow_xz[1] - shoulder_xz[1];
    const double lower_x = grip_xz[0] - elbow_xz[0];
    const double lower_z = grip_xz[1] - elbow_xz[1];
    const double upper_a0 = std::atan2(upper_z, upper_x);
    const double lower_a0 = std::atan2(lower_z, lower_x);
    out.arm_upper_m = static_cast<Real>(std::hypot(upper_x, upper_z));
    out.arm_lower_m = static_cast<Real>(std::hypot(lower_x, lower_z));
    out.arm_upper_angle0 = static_cast<Real>(upper_a0);
    out.arm_lower_angle0 = static_cast<Real>(lower_a0);
    out.arm_branch = std::sin(lower_a0 - upper_a0) >= 0.0 ? 1 : -1;

    {
        const double goal_x = grip_xz[0] - hip_xz[0];
        const double goal_z = grip_xz[1] - hip_xz[1];
        const double reach =
            (std::hypot(upper_x, upper_z) + std::hypot(lower_x, lower_z)) * cfg.arm_reach_fraction;
        const double trunk_norm = std::hypot(trunk_x, trunk_z);
        const auto first = two_link_ik<double, upload_detail::UploadDoubleToolkit,
                                       upload_detail::UploadDoubleTrig>(
            goal_x, goal_z, trunk_norm, reach, -1, 1e-10, 1e-15);
        const auto second = two_link_ik<double, upload_detail::UploadDoubleToolkit,
                                        upload_detail::UploadDoubleTrig>(
            goal_x, goal_z, trunk_norm, reach, 1, 1e-10, 1e-15);
        const auto &neutral =
            std::sin(first.angle0) >= std::sin(second.angle0) ? first : second;
        out.neutral_torso_saturated = neutral.saturated;
        out.neutral_torso_q = static_cast<Real>(0.0);
    }

    if (cfg.grip_is_spring) {
        const auto fb3 = upload_detail::to_idx(out.frame_body) * 3;
        const auto fb9 = upload_detail::to_idx(out.frame_body) * 9;
        const auto gs3 = upload_detail::to_idx(out.grip_sites[0]) * 3;
        const double dx = site_xpos[gs3 + 0] - xpos[fb3 + 0];
        const double dy = site_xpos[gs3 + 1] - xpos[fb3 + 1];
        const double dz = site_xpos[gs3 + 2] - xpos[fb3 + 2];
        out.pose_grip[0] = static_cast<Real>(xmat[fb9 + 0] * dx + xmat[fb9 + 3] * dy + xmat[fb9 + 6] * dz);
        out.pose_grip[1] = static_cast<Real>(0.0);
        out.pose_grip[2] = static_cast<Real>(xmat[fb9 + 2] * dx + xmat[fb9 + 5] * dy + xmat[fb9 + 8] * dz);
    }

    std::array<upload_detail::EnvelopeRange, kNumSpindleJoints> env_ranges{};
    const std::span<JointStrengthSpec<Real>> dst_str(out.strength);
    for (std::size_t i = 0; i < static_cast<std::size_t>(kNumSpindleJoints); ++i) {
        const auto &env = tables.envelopes[i];
        const auto &str = tables.strength[i];
        if (str.direction != 1 && str.direction != -1) {
            throw std::invalid_argument("joint strength direction must be +1 or -1");
        }
        const double lo_q = static_cast<double>(std::span{out.joint_range_lo}[i]);
        const double hi_q = static_cast<double>(std::span{out.joint_range_hi}[i]);
        upload_detail::validate_joint_envelope(upload_detail::JointEnvelopeValidationInput{
            .slot = i,
            .env_direction = env.direction,
            .min_anat_rad = env.minimum_anatomical_rad,
            .max_anat_rad = env.maximum_anatomical_rad,
            .strength_direction = str.direction,
            .check_strength_direction = cfg.has_strength,
            .joint_has_range = std::span{out.joint_has_range}[i],
            .joint_lo_q = lo_q,
            .joint_hi_q = hi_q,
        });

        const double neutral_anat = (env.direction > 0)
            ? (env.minimum_anatomical_rad - lo_q)
            : (env.maximum_anatomical_rad + lo_q);
        env_ranges[i] = upload_detail::EnvelopeRange{
            .lo_q = lo_q,
            .hi_q = hi_q,
            .neutral_anatomical_rad = neutral_anat,
            .direction = env.direction,
        };

        dst_str[i].direction = str.direction;
        dst_str[i].neutral_anatomical_rad = static_cast<Real>(neutral_anat);
        if (cfg.has_strength) {
            upload_detail::pack_torque_curve(str.positive, dst_str[i].positive);
            upload_detail::pack_torque_curve(str.negative, dst_str[i].negative);
        }
    }

    if (cfg.compute_lean_limit) {
        out.has_lean_limit = 1u;
        out.lean_limit_rad = static_cast<Real>(upload_detail::compute_lean_limit_from_pose(
            hip_xz, shoulder_xz, elbow_xz, grip_xz, knee_xz, pedal_xz,
            env_ranges, upload_detail::LeanLimitSearchOptions{.crank_m = cfg.crank_length_m}));
    } else {
        out.has_lean_limit = 0u;
    }

    if (!validate_spindle_m2k2_params(&out)) {
        throw std::invalid_argument("spindle_m2k2_upload produced invalid params");
    }
    return out;
}

template <class Real>
[[nodiscard]] SpindleM2K2Params<Real> build_spindle_m2k2_params_from_json(
    const mjModel *m,
    const mjData *d_qpos0,
    const SpindleReferenceJsonTexts &texts,
    const SpindleM2K2ScalarConfig &cfg
) {
    const SpindleReferenceTables tables = parse_spindle_reference_json(texts);
    return build_spindle_m2k2_params_from_tables<Real>(m, d_qpos0, tables, cfg);
}

template <class Real>
[[nodiscard]] SpindleM2K2Params<Real> build_spindle_m2k2_params_from_json_files(
    const mjModel *m,
    const mjData *d_qpos0,
    const std::filesystem::path &strength_json_path,
    const std::filesystem::path &envelope_json_path,
    const SpindleM2K2ScalarConfig &cfg
) {
    const SpindleReferenceTables tables =
        load_spindle_reference_files(strength_json_path, envelope_json_path);
    return build_spindle_m2k2_params_from_tables<Real>(m, d_qpos0, tables, cfg);
}

} // namespace bike::physics_core
