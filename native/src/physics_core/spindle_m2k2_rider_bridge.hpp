// native/src/physics_core/spindle_m2k2_rider_bridge.hpp — Host-only adapter
// connecting bike_core types (SimulationPhysicsConfig, bike::io::Value,
// rider::SpindlePose, rider::SpindleConfig) to standalone spindle_m2k2_upload.
#pragma once

#if defined(__METAL_VERSION__) || defined(__CUDA_ARCH__)
#error "spindle_m2k2_rider_bridge.hpp is host-only"
#endif

#include "../configuration/model_config.hpp"
#include "../io/value.hpp"
#include "../rider/spindle_controller.hpp"
#include "cpu_exact_toolkit.hpp"
#include "spindle_m2k2_upload.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace bike::physics_core {

[[nodiscard]] inline SpindleM2K2ScalarConfig make_spindle_scalar_config(
    const bike::configuration::SimulationPhysicsConfig &phys,
    double crank_length_m = 0.165
) {
    SpindleM2K2ScalarConfig cfg{};
    cfg.joint_kp_nm_rad = phys.articulated.joint_kp_nm_rad;
    cfg.joint_kd_nms_rad = phys.articulated.joint_kd_nms_rad;
    cfg.leg_active_damping_nms_rad = phys.articulated.leg_active_damping_nms_rad;
    cfg.joint_limit_nm = phys.articulated.joint_limit_nm;
    cfg.joint_speed_limit_rad_s = phys.articulated.joint_speed_limit_rad_s;
    cfg.joint_power_limit_w = phys.articulated.joint_power_limit_w;
    cfg.has_active_positive_power_limit =
        phys.articulated.active_positive_power_limit_w.has_value();
    cfg.active_positive_power_limit_w =
        phys.articulated.active_positive_power_limit_w.value_or(
            phys.articulated.joint_power_limit_w * static_cast<double>(kNumSpindleJoints));
    cfg.pedal_torque_ripple = phys.articulated.pedal_torque_ripple;
    cfg.return_foot_preload_n = phys.articulated.return_foot_preload_n;
    cfg.weld_servo_gain = phys.articulated.weld_servo_gain;
    cfg.weld_servo_limit_nm = phys.articulated.weld_servo_limit_nm;
    cfg.reposition_damping_nms_rad = phys.drive.pedaling.reposition_damping_nms_rad;
    cfg.reposition_torque_limit_nm = phys.seated_climb.max_crank_torque_nm;
    cfg.arm_reach_fraction = phys.articulated.arm_reach_fraction;
    cfg.crank_length_m = crank_length_m;
    cfg.grip_is_spring = (phys.articulated.grip_attachment == "spring");
    if (phys.timestep_s > 0.0 && phys.control_period_s > 0.0) {
        cfg.control_period_steps = std::max(
            1, static_cast<int>(std::llround(phys.control_period_s / phys.timestep_s)));
    }
    return cfg;
}

namespace bridge_detail {

[[nodiscard]] inline const bike::io::Value *find_obj_member(
    const bike::io::Value &obj,
    std::string_view key
) noexcept {
    const auto *map = std::get_if<bike::io::Value::Object>(&obj.data);
    if (map == nullptr) {
        return nullptr;
    }
    for (const auto &[k, v] : *map) {
        if (k == key) {
            return &v;
        }
    }
    return nullptr;
}

[[nodiscard]] inline const bike::io::Value &require_obj_member(
    const bike::io::Value &obj,
    std::string_view key
) {
    const auto *v = find_obj_member(obj, key);
    if (v == nullptr) {
        throw std::invalid_argument(std::string("missing JSON field: ") + std::string(key));
    }
    return *v;
}

[[nodiscard]] inline double json_real(const bike::io::Value &v) {
    const auto *num = std::get_if<bike::io::Number>(&v.data);
    if (num == nullptr) {
        throw std::invalid_argument("expected JSON number");
    }
    const double d = std::stod(num->lexeme);
    if (!std::isfinite(d)) {
        throw std::invalid_argument("expected finite JSON number");
    }
    return d;
}

[[nodiscard]] inline int json_direction(const bike::io::Value &v) {
    const double d = json_real(v);
    if (d == 1.0) {
        return 1;
    }
    if (d == -1.0) {
        return -1;
    }
    throw std::invalid_argument("JSON direction must be +1 or -1");
}

[[nodiscard]] inline std::vector<double> json_real_array(const bike::io::Value &v) {
    const auto *arr = std::get_if<bike::io::Value::Array>(&v.data);
    if (arr == nullptr) {
        throw std::invalid_argument("expected JSON array");
    }
    std::vector<double> out;
    out.reserve(arr->size());
    for (const auto &el : *arr) {
        out.push_back(json_real(el));
    }
    return out;
}

[[nodiscard]] inline TorqueCurveInput torque_curve_from_value(const bike::io::Value &v) {
    return TorqueCurveInput{
        .angles = json_real_array(require_obj_member(v, "angles_rad")),
        .torques = json_real_array(require_obj_member(v, "torques_nm")),
        .vmax = json_real(require_obj_member(v, "vmax_rad_s")),
        .hill_c = json_real(require_obj_member(v, "hill_c")),
        .eccentric_ratio = json_real(require_obj_member(v, "eccentric_ratio")),
    };
}

} // namespace bridge_detail

template <class Real>
[[nodiscard]] SpindleM2K2Params<Real> build_spindle_m2k2_params_from_json_docs(
    const mjModel *m,
    const mjData *d_qpos0,
    const bike::io::Value &strength_doc,
    const bike::io::Value &envelope_doc,
    const SpindleM2K2ScalarConfig &cfg
) {
    const auto &s_joints = bridge_detail::require_obj_member(strength_doc, "joints");
    const auto &e_joints = bridge_detail::require_obj_member(envelope_doc, "joints");
    SpindleReferenceTables tables{};
    for (std::size_t i = 0; i < static_cast<std::size_t>(kNumSpindleJoints); ++i) {
        const char *jname = kSpindleJointNames[i];
        const auto &ej = bridge_detail::require_obj_member(e_joints, jname);
        tables.envelopes[i].direction =
            bridge_detail::json_direction(bridge_detail::require_obj_member(ej, "direction"));
        tables.envelopes[i].minimum_anatomical_rad =
            bridge_detail::json_real(bridge_detail::require_obj_member(ej, "minimum_anatomical_rad"));
        tables.envelopes[i].maximum_anatomical_rad =
            bridge_detail::json_real(bridge_detail::require_obj_member(ej, "maximum_anatomical_rad"));

        const auto &sj = bridge_detail::require_obj_member(s_joints, jname);
        const auto &coord = bridge_detail::require_obj_member(sj, "coordinate");
        tables.strength[i].direction =
            bridge_detail::json_direction(bridge_detail::require_obj_member(coord, "direction"));
        const auto &dirs = bridge_detail::require_obj_member(sj, "directions");
        tables.strength[i].positive =
            bridge_detail::torque_curve_from_value(bridge_detail::require_obj_member(dirs, "positive"));
        tables.strength[i].negative =
            bridge_detail::torque_curve_from_value(bridge_detail::require_obj_member(dirs, "negative"));
    }
    return build_spindle_m2k2_params_from_tables<Real>(m, d_qpos0, tables, cfg);
}

// Build SpindleM2K2Params<Real> from authoritative rider::SpindlePose + rider::SpindleConfig
// (preserves exact neutral_anatomical_rad, envelopes, and pose geometry).
template <class Real>
[[nodiscard]] SpindleM2K2Params<Real> build_spindle_m2k2_params_from_spindle_config(
    const mjModel *m,
    const ::rider::SpindlePose &pose,
    const ::rider::SpindleConfig &config,
    const SpindleUploadTiming &timing = {}
) {
    SpindleM2K2ScalarConfig cfg{};
    cfg.joint_kp_nm_rad = config.joint_kp_nm_rad;
    cfg.joint_kd_nms_rad = config.joint_kd_nms_rad;
    cfg.leg_active_damping_nms_rad = config.leg_active_damping_nms_rad;
    cfg.joint_limit_nm = config.joint_limit_nm;
    cfg.joint_speed_limit_rad_s = config.joint_speed_limit_rad_s;
    cfg.joint_power_limit_w = config.joint_power_limit_w;
    cfg.has_active_positive_power_limit = config.active_positive_power_limit_w.has_value();
    cfg.active_positive_power_limit_w = config.active_positive_power_limit_w.value_or(
        config.joint_power_limit_w * static_cast<double>(kNumSpindleJoints));
    cfg.pedal_torque_ripple = config.pedal_torque_ripple;
    cfg.return_foot_preload_n = config.return_foot_preload_n;
    cfg.weld_servo_gain = config.weld_servo_gain;
    cfg.weld_servo_limit_nm = config.weld_servo_limit_nm;
    cfg.reposition_damping_nms_rad = config.reposition_damping_nms_rad;
    cfg.reposition_torque_limit_nm = config.reposition_torque_limit_nm;
    cfg.arm_reach_fraction = config.arm_reach_fraction;
    cfg.crank_length_m = timing.crank_length_m;
    cfg.control_period_steps = timing.control_period_steps;
    cfg.grip_is_spring = config.legacy && config.legacy->grip_attachment == "spring";
    cfg.has_strength = config.has_strength;
    cfg.compute_lean_limit = !config.envelopes.empty() &&
                             (!config.legacy || config.legacy->pedal_attachment == "spindle");

    SpindleM2K2Params<Real> out{};
    upload_detail::populate_model_topology_common(m, cfg, out);

    const std::array<double, 2> hip{pose.hip[0], pose.hip[2]};
    const std::array<double, 2> shoulder{pose.shoulder[0], pose.shoulder[2]};
    const std::array<double, 2> elbow{pose.elbow[0], pose.elbow[2]};
    const std::array<double, 2> grip{pose.grip[0], pose.grip[2]};
    const std::array<std::array<double, 2>, 2> knee{
        std::array<double, 2>{pose.knee_front[0], pose.knee_front[2]},
        std::array<double, 2>{pose.knee_rear[0], pose.knee_rear[2]}};
    const std::array<std::array<double, 2>, 2> pedal{
        std::array<double, 2>{pose.pedal_front[0], pose.pedal_front[2]},
        std::array<double, 2>{pose.pedal_rear[0], pose.pedal_rear[2]}};

    for (std::size_t s = 0; s < 2; ++s) {
        const double thigh_x = knee[s][0] - hip[0];
        const double thigh_z = knee[s][1] - hip[1];
        const double link_x = pedal[s][0] - knee[s][0];
        const double link_z = pedal[s][1] - knee[s][1];
        const double chord_x = pedal[s][0] - hip[0];
        const double chord_z = pedal[s][1] - hip[1];
        const double ahead = chord_x * thigh_z - chord_z * thigh_x;
        std::span{out.thigh_m}[s] = static_cast<Real>(std::hypot(thigh_x, thigh_z));
        std::span{out.link_m}[s] = static_cast<Real>(std::hypot(link_x, link_z));
        std::span{out.knee_sign}[s] = static_cast<Real>(ahead >= 0.0 ? 1.0 : -1.0);
        std::span{out.thigh_angle0_rad}[s] =
            static_cast<Real>(upload_detail::planar_angle_f64(thigh_x, thigh_z));
        std::span{out.link_angle0_rad}[s] =
            static_cast<Real>(upload_detail::planar_angle_f64(link_x, link_z));
    }

    const double trunk_x = shoulder[0] - hip[0];
    const double trunk_z = shoulder[1] - hip[1];
    const double upper_x = elbow[0] - shoulder[0];
    const double upper_z = elbow[1] - shoulder[1];
    const double lower_x = grip[0] - elbow[0];
    const double lower_z = grip[1] - elbow[1];
    const double upper_a0 = std::atan2(upper_z, upper_x);
    const double lower_a0 = std::atan2(lower_z, lower_x);
    out.arm_upper_m = static_cast<Real>(std::hypot(upper_x, upper_z));
    out.arm_lower_m = static_cast<Real>(std::hypot(lower_x, lower_z));
    out.arm_upper_angle0 = static_cast<Real>(upper_a0);
    out.arm_lower_angle0 = static_cast<Real>(lower_a0);
    out.arm_branch = std::sin(lower_a0 - upper_a0) >= 0.0 ? 1 : -1;

    {
        const double goal_x = grip[0] - hip[0];
        const double goal_z = grip[1] - hip[1];
        const double reach =
            (std::hypot(upper_x, upper_z) + std::hypot(lower_x, lower_z)) * cfg.arm_reach_fraction;
        const double trunk_norm = std::hypot(trunk_x, trunk_z);
        const auto first = two_link_ik<double, CpuExactToolkit, upload_detail::UploadDoubleTrig>(
            goal_x, goal_z, trunk_norm, reach, -1, 1e-10, 1e-15);
        const auto second = two_link_ik<double, CpuExactToolkit, upload_detail::UploadDoubleTrig>(
            goal_x, goal_z, trunk_norm, reach, 1, 1e-10, 1e-15);
        const auto &neutral =
            std::sin(first.angle0) >= std::sin(second.angle0) ? first : second;
        out.neutral_torso_saturated = neutral.saturated;
        out.neutral_torso_q =
            (config.legacy && config.legacy->pedal_attachment != "spindle")
                ? static_cast<Real>(std::atan2(trunk_z, trunk_x) - neutral.angle0)
                : static_cast<Real>(0.0);
    }

    for (std::size_t c = 0; c < 3; ++c) {
        std::span{out.pose_grip}[c] = static_cast<Real>(std::span{pose.grip}[c]);
    }

    if (config.has_strength) {
        const std::span<JointStrengthSpec<Real>> dst_str(out.strength);
        for (std::size_t i = 0; i < static_cast<std::size_t>(kNumSpindleJoints); ++i) {
            const std::string name(kSpindleJointNames[i]);
            const auto &coord = config.strength_coordinates.at(name);
            if (coord.direction != 1 && coord.direction != -1) {
                throw std::invalid_argument("SpindleConfig strength coordinate direction must be +1 or -1");
            }
            if (!std::isfinite(coord.neutral_anatomical_rad)) {
                throw std::invalid_argument("SpindleConfig neutral_anatomical_rad must be finite");
            }
            const auto &str = config.strength.at(name);
            dst_str[i].direction = coord.direction;
            dst_str[i].neutral_anatomical_rad = static_cast<Real>(coord.neutral_anatomical_rad);
            upload_detail::pack_torque_curve(
                TorqueCurveInput{
                    .angles = str.positive.angles_rad,
                    .torques = str.positive.torques_nm,
                    .vmax = str.positive.vmax_rad_s,
                    .hill_c = str.positive.hill_c,
                    .eccentric_ratio = str.positive.eccentric_ratio,
                },
                dst_str[i].positive);
            upload_detail::pack_torque_curve(
                TorqueCurveInput{
                    .angles = str.negative.angles_rad,
                    .torques = str.negative.torques_nm,
                    .vmax = str.negative.vmax_rad_s,
                    .hill_c = str.negative.hill_c,
                    .eccentric_ratio = str.negative.eccentric_ratio,
                },
                dst_str[i].negative);
        }
    }

    if (cfg.compute_lean_limit) {
        const std::span<const JointStrengthSpec<Real>> dst_str_ro(out.strength);
        const std::span<const std::uint8_t> has_rng_ro(out.joint_has_range);
        const std::span<const Real> rng_lo_ro(out.joint_range_lo);
        const std::span<const Real> rng_hi_ro(out.joint_range_hi);
        std::array<upload_detail::EnvelopeRange, kNumSpindleJoints> env_ranges{};
        for (std::size_t i = 0; i < static_cast<std::size_t>(kNumSpindleJoints); ++i) {
            const auto &env = config.envelopes.at(std::string(kSpindleJointNames[i]));
            const int strength_dir = config.has_strength
                ? dst_str_ro[i].direction
                : env.direction;
            upload_detail::validate_joint_envelope(upload_detail::JointEnvelopeValidationInput{
                .slot = i,
                .env_direction = env.direction,
                .min_anat_rad = env.minimum_anatomical_rad,
                .max_anat_rad = env.maximum_anatomical_rad,
                .strength_direction = strength_dir,
                .check_strength_direction = config.has_strength,
                .joint_has_range = has_rng_ro[i],
                .joint_lo_q = static_cast<double>(rng_lo_ro[i]),
                .joint_hi_q = static_cast<double>(rng_hi_ro[i]),
            });
            if (!std::isfinite(env.neutral_anatomical_rad)) {
                throw std::invalid_argument("SpindleConfig envelope neutral_anatomical_rad must be finite");
            }
            const double a = (env.minimum_anatomical_rad - env.neutral_anatomical_rad) / env.direction;
            const double b = (env.maximum_anatomical_rad - env.neutral_anatomical_rad) / env.direction;
            env_ranges[i] = upload_detail::EnvelopeRange{
                .lo_q = std::min(a, b),
                .hi_q = std::max(a, b),
                .neutral_anatomical_rad = env.neutral_anatomical_rad,
                .direction = env.direction,
            };
        }
        out.has_lean_limit = 1u;
        out.lean_limit_rad = static_cast<Real>(upload_detail::compute_lean_limit_from_pose(
            hip, shoulder, elbow, grip, knee, pedal, env_ranges,
            upload_detail::LeanLimitSearchOptions{.crank_m = timing.crank_length_m}));
    } else {
        out.has_lean_limit = 0u;
    }

    if (!validate_spindle_m2k2_params(&out)) {
        throw std::invalid_argument("build_spindle_m2k2_params_from_spindle_config invalid params");
    }
    return out;
}

} // namespace bike::physics_core
