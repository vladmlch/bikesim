// rider/spindle_wire.cpp — wire decoders/emitters for the spindle
// controller. See the header for the shape contract.
#include "spindle_wire.hpp"

#include <array>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <nanobind/stl/string.h>

#include "../binding_arrays.hpp"
#include "../binding_readers.hpp"

namespace rider {
namespace {

using wire::keys;

[[nodiscard]] nb::dict child_dict(const nb::dict &parent, const char *key,
                                  std::string_view path) {
    const nb::handle value = parent[key];
    if (!value.is_valid() ||
        !nb::isinstance<nb::dict>(value))
        wire::invalid(std::string(path) + "." + key, "expected dict");
    return nb::borrow<nb::dict>(value);
}

[[nodiscard]] nb::dict nullable_dict(nb::handle value,
                                     std::string_view path) {
    if (value.is_none())
        return {};
    return wire::mapping(value, path);
}

[[nodiscard]] std::array<double, 3> point3(nb::handle value,
                                           std::string_view path) {
    const auto v = wire::fixed<3>(value, path);
    return {v[0], v[1], v[2]};
}

[[nodiscard]] spindle::TorqueCurve parse_curve(nb::handle value,
                                               std::string_view path) {
    const auto d = wire::mapping(value, path);
    wire::exact_keys(
        d,
        keys("angles_rad", "torques_nm", "vmax_rad_s", "hill_c",
             "eccentric_ratio", "source"),
        {}, path);
    spindle::TorqueCurve curve{
        .angles_rad = wire::vector(d["angles_rad"], path),
        .torques_nm = wire::vector(d["torques_nm"], path),
        .vmax_rad_s = wire::finite_real(d["vmax_rad_s"], path),
        .hill_c = wire::finite_real(d["hill_c"], path),
        .eccentric_ratio = wire::finite_real(d["eccentric_ratio"], path),
        .source = wire::string(d["source"], path),
    };
    spindle::validate_torque_curve(curve, "strength curve");
    return curve;
}

[[nodiscard]] spindle::JointEnvelope parse_envelope(nb::handle value,
                                                    std::string_view path) {
    const auto d = wire::mapping(value, path);
    wire::exact_keys(d,
                     keys("neutral_anatomical_rad", "direction",
                          "minimum_anatomical_rad",
                          "maximum_anatomical_rad", "provenance"),
                     {}, path);
    spindle::JointEnvelope envelope{
        .neutral_anatomical_rad =
            wire::finite_real(d["neutral_anatomical_rad"], path),
        .direction = wire::integer32(d["direction"], path),
        .minimum_anatomical_rad =
            wire::finite_real(d["minimum_anatomical_rad"], path),
        .maximum_anatomical_rad =
            wire::finite_real(d["maximum_anatomical_rad"], path),
        .provenance = wire::string(d["provenance"], path),
    };
    spindle::validate_joint_envelope(envelope, "joint envelope");
    return envelope;
}

[[nodiscard]] std::map<std::string, spindle::JointEnvelope>
parse_envelope_map(nb::handle value, std::string_view path) {
    std::map<std::string, spindle::JointEnvelope> out;
    const auto d = nullable_dict(value, path);
    for (const auto item : d) {
        const auto name = wire::string(item.first, path);
        const auto sub = std::string(path) + "." + name;
        out.emplace(name, parse_envelope(item.second, sub));
    }
    return out;
}

[[nodiscard]] NamedEntries<bool>
parse_named_bools(nb::handle value, std::string_view path,
                  std::span<const std::string_view> allowed) {
    NamedEntries<bool> out;
    const auto d = wire::mapping(value, path);
    for (const auto item : d) {
        const auto name = wire::string(item.first, path);
        if (std::ranges::find(allowed, name) == allowed.end())
            wire::invalid(std::string(path) + "." + name, "unknown key");
        const auto sub = std::string(path) + "." + name;
        out.emplace_back(name, wire::boolean(item.second, sub));
    }
    return out;
}

[[nodiscard]] std::vector<std::string>
parse_name_list(nb::handle value, std::string_view path) {
    std::vector<std::string> out;
    for (const nb::handle item : wire::sequence(value, path))
        out.push_back(wire::string(item, path));
    return out;
}

[[nodiscard]] std::optional<double>
parse_optional_real(nb::handle value, std::string_view path) {
    return wire::optional_real(value, path);
}

[[nodiscard]] PedalRecoveryState
parse_pedal_recovery(nb::handle value, std::string_view path) {
    const auto d = wire::mapping(value, path);
    wire::exact_keys(
        d, keys("stage", "direction", "release_offset_x_m"), {}, path);
    PedalRecoveryState out;
    out.stage = wire::string(d["stage"], path);
    if (out.stage != "none" && out.stage != "release" &&
        out.stage != "escape" && out.stage != "raise" &&
        out.stage != "return")
        wire::invalid(path, "unknown pedal recovery stage");
    out.direction = wire::finite_real(d["direction"], path);
    out.release_offset_x_m =
        wire::finite_real(d["release_offset_x_m"], path);
    return out;
}

[[nodiscard]] EffortDiagnostics
parse_effort_diagnostics(nb::handle value, std::string_view path) {
    const auto d = nullable_dict(value, path);
    if (d.size() == 0)
        return {};
    EffortDiagnostics out;
    wire::exact_keys(
        d,
        keys("rider_active_request_nm", "rider_active_delivered_nm",
             "rider_positive_power_w", "rider_passive_power_w",
             "rider_activation_saturated", "rider_strength_limited",
             "rider_effort_budget_exceeded",
             "rider_active_positive_power_limit_w",
             "rider_effort_observation"),
        keys("rider_joint_positive_power_w",
             "rider_joint_power_violations",
             "rider_joint_speed_violations",
             "rider_positive_work_step_j", "rider_passive_work_step_j",
             "rider_strength_violations"),
        path);
    out.present = true;
    out.active_request_nm =
        parse_named_reals(d["rider_active_request_nm"], path);
    out.active_delivered_nm =
        parse_named_reals(d["rider_active_delivered_nm"], path);
    out.positive_power_w =
        wire::finite_real(d["rider_positive_power_w"], path);
    out.passive_power_w =
        wire::finite_real(d["rider_passive_power_w"], path);
    out.activation_saturated =
        wire::boolean(d["rider_activation_saturated"], path);
    out.strength_limited =
        parse_name_list(d["rider_strength_limited"], path);
    out.budget_exceeded =
        wire::boolean(d["rider_effort_budget_exceeded"], path);
    out.positive_power_limit_w = parse_optional_real(
        d["rider_active_positive_power_limit_w"], path);
    out.observation =
        wire::string(d["rider_effort_observation"], path);
    if (d.contains("rider_joint_positive_power_w"))
        out.joint_positive_power_w = parse_named_reals(
            d["rider_joint_positive_power_w"], path);
    if (d.contains("rider_joint_power_violations"))
        out.joint_power_violations = parse_name_list(
            d["rider_joint_power_violations"], path);
    if (d.contains("rider_joint_speed_violations"))
        out.joint_speed_violations = parse_name_list(
            d["rider_joint_speed_violations"], path);
    if (d.contains("rider_positive_work_step_j"))
        out.positive_work_step_j = wire::finite_real(
            d["rider_positive_work_step_j"], path);
    if (d.contains("rider_passive_work_step_j"))
        out.passive_work_step_j = wire::finite_real(
            d["rider_passive_work_step_j"], path);
    if (d.contains("rider_strength_violations"))
        out.strength_violations = parse_name_list(
            d["rider_strength_violations"], path);
    return out;
}

[[nodiscard]] AllocationDiagnostics
parse_allocation_diagnostics(nb::handle value, std::string_view path) {
    const auto d = nullable_dict(value, path);
    if (d.size() == 0)
        return {};
    AllocationDiagnostics out;
    wire::exact_keys(d,
                     keys("invalid_controller", "crank_task_nm",
                          "crank_task_shortfall_nm",
                          "solution_excitation_nm"),
                     {}, path);
    out.present = true;
    out.invalid_controller = wire::boolean(d["invalid_controller"], path);
    out.crank_task_nm = wire::finite_real(d["crank_task_nm"], path);
    out.crank_task_shortfall_nm =
        wire::finite_real(d["crank_task_shortfall_nm"], path);
    out.solution_excitation_nm =
        wire::vector(d["solution_excitation_nm"], path);
    return out;
}

[[nodiscard]] SupportDiagnostics
parse_support_diagnostics(nb::handle value, std::string_view path) {
    const auto d = nullable_dict(value, path);
    if (d.size() == 0)
        return {};
    SupportDiagnostics out;
    wire::exact_keys(d, keys("stance"), {}, path);
    const auto stance = child_dict(d, "stance", path);
    wire::exact_keys(stance, keys("front", "rear"), {},
                     std::string(path) + ".stance");
    out.present = true;
    out.stance_front =
        wire::boolean(stance["front"],
                      std::string(path) + ".stance.front");
    out.stance_rear =
        wire::boolean(stance["rear"], std::string(path) + ".stance.rear");
    return out;
}

[[nodiscard]] NamedEntries<SoleGoalDiagnostics>
parse_sole_goal_diagnostics(nb::handle value, std::string_view path) {
    NamedEntries<SoleGoalDiagnostics> out;
    const auto d = nullable_dict(value, path);
    for (const auto item : d) {
        const auto name = wire::string(item.first, path);
        if (name != "front" && name != "rear")
            wire::invalid(std::string(path) + "." + name, "unknown key");
        const auto sub = std::string(path) + "." + name;
        const auto entry = wire::mapping(item.second, sub);
        wire::exact_keys(entry,
                         keys("spindle", "saturated", "limiting_reasons"),
                         {}, sub);
        SoleGoalDiagnostics goal;
        goal.spindle = wire::boolean(entry["spindle"], sub);
        goal.saturated = wire::boolean(entry["saturated"], sub);
        for (const nb::handle reason :
             wire::sequence(entry["limiting_reasons"], sub))
            goal.limiting_reasons.push_back(wire::string(reason, sub));
        out.emplace_back(name, std::move(goal));
    }
    return out;
}

// Emission helpers ---------------------------------------------------------

[[nodiscard]] nb::dict named_reals_dict(
    const NamedEntries<double> &values) {
    nb::dict out;
    for (const auto &[name, value] : values) out[name.c_str()] = value;
    return out;
}

[[nodiscard]] nb::dict named_bools_dict(
    const NamedEntries<bool> &values) {
    nb::dict out;
    for (const auto &[name, value] : values) out[name.c_str()] = value;
    return out;
}

[[nodiscard]] nb::list names_list(const std::vector<std::string> &names) {
    nb::list out;
    for (const auto &name : names) out.append(name);
    return out;
}

void set_or_none(const nb::dict &out, const char *key,
                 const std::optional<double> &value) {
    if (value.has_value())
        out[key] = *value;
    else
        out[key] = nb::none();
}

// Kept out of parse_spindle_config's frame: the table decode plus the
// scalar section together exceed the 8 KiB stack guard.
void parse_optional_path(const nb::dict &d, const char *key,
                         std::string_view path) {
    const nb::handle v = d[key];
    if (!v.is_none())
        (void)wire::string(v, path);
}

void parse_strength_tables(const nb::dict &d, std::string_view path,
                           SpindleConfig &config) {
    const nb::handle strength = d["strength"];
    if (!strength.is_none()) {
        config.has_strength = true;
        const auto table = wire::mapping(strength, path);
        for (const auto item : table) {
            const auto name = wire::string(item.first, path);
            const auto sub = std::string(path) + "." + name;
            const auto pair = wire::mapping(item.second, sub);
            wire::exact_keys(pair, keys("positive", "negative"), {}, sub);
            DirectionalStrength curves{
                .positive = parse_curve(pair["positive"],
                                        sub + ".positive"),
                .negative = parse_curve(pair["negative"],
                                        sub + ".negative")};
            config.strength.emplace(name, std::move(curves));
        }
    }
    config.strength_coordinates =
        parse_envelope_map(d["strength_coordinates"], path);
    config.envelopes = parse_envelope_map(d["envelopes"], path);
}

} // namespace

// A joint-name -> number dict preserving wire insertion order — Python
// dict ordering is part of the contract. Exported for the runtime
// bootstrap's held_control decode.
NamedEntries<double> parse_named_reals(nb::handle value,
                                       std::string_view path) {
    NamedEntries<double> out;
    const auto d = wire::mapping(value, path);
    for (const auto item : d) {
        const auto name = wire::string(item.first, path);
        const auto sub = std::string(path) + "." + name;
        out.emplace_back(name, wire::finite_real(item.second, sub));
    }
    return out;
}

// Exported for the runtime bootstrap's held_rider_terms decode.
JointTerms parse_joint_terms(nb::handle value, std::string_view path) {
    const auto d = wire::mapping(value, path);
    wire::exact_keys(
        d,
        keys("requested_nm", "command_nm", "posture_nm", "pedaling_nm",
             "saturated"),
        keys("active_request_nm", "active_delivered_nm",
             "passive_damping_nm", "solved_force_nm", "solved_active_nm",
             "solved_passive_nm"),
        path);
    JointTerms out;
    out.requested_nm = wire::finite_real(d["requested_nm"], path);
    out.command_nm = wire::finite_real(d["command_nm"], path);
    out.posture_nm = wire::finite_real(d["posture_nm"], path);
    out.pedaling_nm = wire::finite_real(d["pedaling_nm"], path);
    out.saturated = wire::boolean(d["saturated"], path);
    const auto optional = [&](const char *key) {
        if (!d.contains(key) || d[key].is_none())
            return std::optional<double>{};
        return std::optional<double>(
            wire::finite_real(d[key], path));
    };
    out.active_request_nm = optional("active_request_nm");
    out.active_delivered_nm = optional("active_delivered_nm");
    out.passive_damping_nm = optional("passive_damping_nm");
    out.solved_force_nm = optional("solved_force_nm");
    out.solved_active_nm = optional("solved_active_nm");
    out.solved_passive_nm = optional("solved_passive_nm");
    return out;
}

SpindlePose parse_spindle_pose(nb::handle value, std::string_view path) {
    const auto d = wire::mapping(value, path);
    wire::exact_keys(d,
                     keys("saddle", "hip", "shoulder", "elbow", "grip",
                          "head_center", "knee_front", "knee_rear",
                          "ankle_front", "ankle_rear", "pedal_front",
                          "pedal_rear"),
                     {}, path);
    SpindlePose pose;
    pose.hip = point3(d["hip"], path);
    pose.shoulder = point3(d["shoulder"], path);
    pose.elbow = point3(d["elbow"], path);
    pose.grip = point3(d["grip"], path);
    pose.knee_front = point3(d["knee_front"], path);
    pose.knee_rear = point3(d["knee_rear"], path);
    pose.pedal_front = point3(d["pedal_front"], path);
    pose.pedal_rear = point3(d["pedal_rear"], path);
    // saddle/head_center/ankle_* are validated present by exact_keys but
    // unused by the spindle path.
    return pose;
}

SpindleConfig parse_spindle_config(nb::handle value,
                                   std::string_view path) {
    const auto d = wire::mapping(value, path);
    // Every ArticulatedConfig field, plus the resolved tables appended by
    // the setup boundary. All fields are required; presence is the wire
    // contract even when the spindle path ignores a value.
    constexpr auto required = keys(
        "activation_tau_s", "active_positive_power_limit_w",
        "arm_reach_fraction", "balance_dwell_s", "balance_floor_kmh",
        "balance_grace_s", "bar_support_fraction",
        "coasting_brake_d_nm_s_rad", "coasting_brake_limit_nm",
        "coupled_task_control", "foot_mu", "grip_attachment",
        "grip_c_ns_m", "grip_capture_distance_m", "grip_capture_speed_mps",
        "grip_k_n_m", "grip_pair_force_limit_n", "grip_pull_per_hand_n",
        "grip_release_distance_m", "joint_envelope_path",
        "joint_envelope_soft_k_nm_rad", "joint_envelope_soft_margin_rad",
        "joint_kd_nms_rad", "joint_kp_nm_rad", "joint_limit_nm",
        "joint_passive_damping_nms_rad", "joint_power_limit_w",
        "joint_speed_limit_rad_s", "joint_strength_path", "link_max_gap_m",
        "pedal_ankle_amplitude_rad", "pedal_attachment", "pedal_c_ns_m",
        "pedal_min_normal_n", "pedal_patch_half_length_m",
        "pedal_scrape_fraction", "pedal_support_fraction",
        "pedal_torque_ripple", "posture_pitch_d_nms_rad",
        "posture_pitch_k_nm_rad", "posture_pitch_limit_nm",
        "posture_sole_depth_m", "posture_translation_d_ns_m",
        "posture_translation_k_n_m", "posture_translation_limit_n",
        "return_foot_preload_n", "road_lookahead_m", "saddle_attachment",
        "saddle_mu", "saddle_patch_half_length_m",
        "saddle_reserve_weight_fraction", "stance_blend_load_n",
        "support_c_ns_m", "support_k_n_m", "support_length_m",
        "support_mu", "support_pad_radius_m", "support_tangent_k_n_m",
        "swing_clearance_m", "strength", "strength_coordinates",
        "envelopes");
    wire::exact_keys(d, required, {}, path);
    SpindleConfig config;
    config.joint_kp_nm_rad = wire::finite_real(d["joint_kp_nm_rad"], path);
    config.joint_kd_nms_rad =
        wire::finite_real(d["joint_kd_nms_rad"], path);
    config.joint_limit_nm = wire::finite_real(d["joint_limit_nm"], path);
    config.joint_speed_limit_rad_s =
        wire::finite_real(d["joint_speed_limit_rad_s"], path);
    config.joint_power_limit_w =
        wire::finite_real(d["joint_power_limit_w"], path);
    config.active_positive_power_limit_w =
        wire::optional_real(d["active_positive_power_limit_w"], path);
    config.joint_passive_damping_nms_rad =
        wire::optional_real(d["joint_passive_damping_nms_rad"], path);
    config.activation_tau_s =
        wire::finite_real(d["activation_tau_s"], path);
    config.pedal_torque_ripple =
        wire::finite_real(d["pedal_torque_ripple"], path);
    config.return_foot_preload_n =
        wire::finite_real(d["return_foot_preload_n"], path);
    config.coasting_brake_d_nm_s_rad =
        wire::finite_real(d["coasting_brake_d_nm_s_rad"], path);
    config.coasting_brake_limit_nm =
        wire::finite_real(d["coasting_brake_limit_nm"], path);
    config.joint_envelope_soft_k_nm_rad =
        wire::finite_real(d["joint_envelope_soft_k_nm_rad"], path);
    config.joint_envelope_soft_margin_rad =
        wire::finite_real(d["joint_envelope_soft_margin_rad"], path);
    config.arm_reach_fraction =
        wire::finite_real(d["arm_reach_fraction"], path);
    config.pedal_patch_half_length_m =
        wire::finite_real(d["pedal_patch_half_length_m"], path);
    config.support_pad_radius_m =
        wire::finite_real(d["support_pad_radius_m"], path);
    // Consumed-but-unmodeled fields are still type-validated; the values
    // belong to surfaces the spindle path does not reach.
    for (const char *key :
         {"pedal_support_fraction", "bar_support_fraction",
          "posture_sole_depth_m", "swing_clearance_m",
          "stance_blend_load_n", "posture_pitch_k_nm_rad",
          "posture_pitch_d_nms_rad", "posture_pitch_limit_nm",
          "posture_translation_k_n_m", "posture_translation_d_ns_m",
          "posture_translation_limit_n", "saddle_patch_half_length_m",
          "support_k_n_m", "support_c_ns_m", "pedal_c_ns_m",
          "support_tangent_k_n_m", "support_mu", "support_length_m",
          "grip_k_n_m", "grip_c_ns_m", "grip_release_distance_m",
          "grip_capture_distance_m", "grip_capture_speed_mps",
          "grip_pull_per_hand_n", "foot_mu", "saddle_mu",
          "pedal_min_normal_n", "saddle_reserve_weight_fraction",
          "pedal_ankle_amplitude_rad", "pedal_scrape_fraction",
          "link_max_gap_m", "road_lookahead_m", "balance_floor_kmh",
          "balance_dwell_s", "balance_grace_s"})
        (void)wire::finite_real(d[key], path);
    (void)wire::optional_real(d["grip_pair_force_limit_n"], path);
    if (wire::boolean(d["coupled_task_control"], path))
        wire::invalid(std::string(path) + ".coupled_task_control",
                      "coupled task control is a legacy allocator path");
    // The class implements exactly the supported spindle plant; other
    // attachments route through paths that do not exist natively.
    const auto pedal_attachment = wire::string(d["pedal_attachment"], path);
    if (pedal_attachment != "spindle")
        wire::invalid(std::string(path) + ".pedal_attachment",
                      "supported: 'spindle'");
    const auto saddle_attachment = wire::string(d["saddle_attachment"], path);
    if (saddle_attachment != "pin")
        wire::invalid(std::string(path) + ".saddle_attachment",
                      "supported: 'pin'");
    const auto grip_attachment = wire::string(d["grip_attachment"], path);
    if (grip_attachment != "connect")
        wire::invalid(std::string(path) + ".grip_attachment",
                      "supported: 'connect'");
    parse_optional_path(d, "joint_strength_path", path);
    parse_optional_path(d, "joint_envelope_path", path);
    parse_strength_tables(d, path, config);
    return config;
}

RiderPosture parse_rider_posture(nb::handle value, std::string_view path) {
    const auto d = wire::mapping(value, path);
    wire::exact_keys(d,
                     keys("torso_lean_rad", "pelvis_pitch_rad",
                          "pelvis_offset_m", "use_saddle"),
                     {}, path);
    RiderPosture posture;
    posture.torso_lean_rad =
        wire::finite_real(d["torso_lean_rad"], path);
    posture.pelvis_pitch_rad =
        wire::finite_real(d["pelvis_pitch_rad"], path);
    if (std::abs(posture.torso_lean_rad) > .8)
        wire::invalid(path, "torso_lean_rad must lie in [-0.8, 0.8]");
    if (std::abs(posture.pelvis_pitch_rad) > .5)
        wire::invalid(path, "pelvis_pitch_rad must lie in [-0.5, 0.5]");
    const nb::handle offset = d["pelvis_offset_m"];
    if (!offset.is_none()) {
        const auto v = wire::fixed<2>(offset, path);
        if (std::abs(v[0]) > .25 || v[1] < -.15 || v[1] > .25)
            wire::invalid(path,
                          "hip offset exceeds the posture command envelope");
        posture.pelvis_offset_m = spindle::Vec2{v[0], v[1]};
    }
    posture.use_saddle = wire::boolean(d["use_saddle"], path);
    return posture;
}

RiderCommand parse_rider_command(nb::handle value, std::string_view path) {
    const auto d = wire::mapping(value, path);
    wire::exact_keys(d,
                     keys("mean_crank_torque_nm", "enabled", "posture",
                          "crank_target_phase_rad",
                          "crank_target_rate_rad_s"),
                     {}, path);
    RiderCommand command;
    command.mean_crank_torque_nm =
        wire::finite_real(d["mean_crank_torque_nm"], path);
    if (command.mean_crank_torque_nm < 0.)
        wire::invalid(path, "rider effort must be nonnegative");
    command.enabled = wire::boolean(d["enabled"], path);
    command.posture = parse_rider_posture(d["posture"],
                                          std::string(path) + ".posture");
    command.crank_target_phase_rad =
        wire::optional_real(d["crank_target_phase_rad"], path);
    command.crank_target_rate_rad_s =
        wire::finite_real(d["crank_target_rate_rad_s"], path);
    if (command.crank_target_phase_rad.has_value() &&
        command.mean_crank_torque_nm > 0.)
        wire::invalid(path,
                      "coasting goals cannot request pedaling effort");
    return command;
}

SpindleController::State parse_spindle_state(nb::handle value,
                                             std::string_view path) {
    const auto d = wire::mapping(value, path);
    wire::exact_keys(
        d,
        keys("enabled", "command_enabled", "active_state",
             "activation_time_s", "last_terms", "pd_split", "saturated_ik",
             "ik_reach_limited", "joint_torques_nm", "joint_capacity_nm",
             "lean_limit_rad", "last_branch", "last_solution",
             "pedal_recovery", "effort_diagnostics",
             "allocation_diagnostics", "support_diagnostics",
             "sole_goal_diagnostics"),
        {}, path);
    SpindleController::State state;
    state.enabled = wire::boolean(d["enabled"], path);
    state.command_enabled = wire::boolean(d["command_enabled"], path);
    state.active_state = wire::vector(d["active_state"], path);
    state.activation_time_s =
        wire::optional_real(d["activation_time_s"], path);
    {
        const auto terms = wire::mapping(d["last_terms"], path);
        for (const auto item : terms) {
            const auto name = wire::string(item.first, path);
            const auto sub = std::string(path) + ".last_terms." + name;
            state.last_terms.emplace_back(name,
                                          parse_joint_terms(item.second,
                                                            sub));
        }
    }
    {
        const auto split = wire::mapping(d["pd_split"], path);
        if (split.size() != 0)
            wire::invalid(std::string(path) + ".pd_split",
                          "spindle controller carries no PD warm split");
    }
    constexpr auto ik_names = keys("front", "rear", "torso", "arms");
    state.saturated_ik =
        parse_named_bools(d["saturated_ik"], path, ik_names);
    state.ik_reach_limited = parse_named_bools(
        d["ik_reach_limited"], path, keys("front", "rear"));
    state.joint_torques_nm =
        parse_named_reals(d["joint_torques_nm"], path);
    state.joint_capacity_nm =
        parse_named_reals(d["joint_capacity_nm"], path);
    state.lean_limit_rad =
        wire::optional_real(d["lean_limit_rad"], path);
    if (!d["last_branch"].is_none())
        wire::invalid(std::string(path) + ".last_branch",
                      "spindle controller carries no allocator branch");
    if (!d["last_solution"].is_none())
        wire::invalid(std::string(path) + ".last_solution",
                      "spindle controller carries no allocator warm start");
    {
        const auto recovery = wire::mapping(d["pedal_recovery"], path);
        wire::exact_keys(recovery, keys("front", "rear"), {},
                         std::string(path) + ".pedal_recovery");
        for (const char *side : {"front", "rear"})
            state.pedal_recovery.emplace_back(
                side,
                parse_pedal_recovery(recovery[side],
                                     std::string(path) +
                                         ".pedal_recovery." + side));
    }
    state.effort_diagnostics = parse_effort_diagnostics(
        d["effort_diagnostics"], std::string(path) + ".effort_diagnostics");
    state.allocation_diagnostics = parse_allocation_diagnostics(
        d["allocation_diagnostics"],
        std::string(path) + ".allocation_diagnostics");
    state.support_diagnostics = parse_support_diagnostics(
        d["support_diagnostics"],
        std::string(path) + ".support_diagnostics");
    state.sole_goal_diagnostics = parse_sole_goal_diagnostics(
        d["sole_goal_diagnostics"],
        std::string(path) + ".sole_goal_diagnostics");
    return state;
}

nb::dict
effort_diagnostics_dict(const EffortDiagnostics &diagnostics) {
    nb::dict out;
    if (!diagnostics.present)
        return out;
    // Emission order follows Python's dict-update order: finalize keys
    // first, then the solved_effort additions.
    out["rider_active_request_nm"] =
        named_reals_dict(diagnostics.active_request_nm);
    out["rider_active_delivered_nm"] =
        named_reals_dict(diagnostics.active_delivered_nm);
    out["rider_positive_power_w"] = diagnostics.positive_power_w;
    out["rider_passive_power_w"] = diagnostics.passive_power_w;
    out["rider_activation_saturated"] = diagnostics.activation_saturated;
    out["rider_strength_limited"] =
        names_list(diagnostics.strength_limited);
    out["rider_effort_budget_exceeded"] = diagnostics.budget_exceeded;
    set_or_none(out, "rider_active_positive_power_limit_w",
                diagnostics.positive_power_limit_w);
    out["rider_effort_observation"] = diagnostics.observation;
    if (diagnostics.joint_positive_power_w.has_value())
        out["rider_joint_positive_power_w"] =
            named_reals_dict(*diagnostics.joint_positive_power_w);
    if (diagnostics.joint_power_violations.has_value())
        out["rider_joint_power_violations"] =
            names_list(*diagnostics.joint_power_violations);
    if (diagnostics.joint_speed_violations.has_value())
        out["rider_joint_speed_violations"] =
            names_list(*diagnostics.joint_speed_violations);
    if (diagnostics.positive_work_step_j.has_value())
        out["rider_positive_work_step_j"] =
            *diagnostics.positive_work_step_j;
    if (diagnostics.passive_work_step_j.has_value())
        out["rider_passive_work_step_j"] =
            *diagnostics.passive_work_step_j;
    if (diagnostics.strength_violations.has_value())
        out["rider_strength_violations"] =
            names_list(*diagnostics.strength_violations);
    return out;
}

nb::dict
allocation_diagnostics_dict(const AllocationDiagnostics &diagnostics) {
    nb::dict out;
    if (!diagnostics.present)
        return out;
    out["invalid_controller"] = diagnostics.invalid_controller;
    out["crank_task_nm"] = diagnostics.crank_task_nm;
    out["crank_task_shortfall_nm"] = diagnostics.crank_task_shortfall_nm;
    out["solution_excitation_nm"] = wire::owned_array<double>(
        std::span<const double>(diagnostics.solution_excitation_nm));
    return out;
}

nb::dict
support_diagnostics_dict(const SupportDiagnostics &diagnostics) {
    nb::dict out;
    if (!diagnostics.present)
        return out;
    nb::dict stance;
    stance["front"] = diagnostics.stance_front;
    stance["rear"] = diagnostics.stance_rear;
    out["stance"] = stance;
    return out;
}

nb::dict sole_goal_diagnostics_dict(
    const NamedEntries<SoleGoalDiagnostics> &diagnostics) {
    nb::dict out;
    for (const auto &[name, goal] : diagnostics) {
        nb::dict entry;
        entry["spindle"] = goal.spindle;
        entry["saturated"] = goal.saturated;
        entry["limiting_reasons"] = names_list(goal.limiting_reasons);
        out[name.c_str()] = entry;
    }
    return out;
}

nb::dict last_terms_dict(const NamedEntries<JointTerms> &terms) {
    nb::dict out;
    for (const auto &[name, term] : terms) {
        nb::dict entry;
        // Python dict-literal order: requested, command, posture,
        // pedaling, saturated; update() keys append afterwards.
        entry["requested_nm"] = term.requested_nm;
        entry["command_nm"] = term.command_nm;
        entry["posture_nm"] = term.posture_nm;
        entry["pedaling_nm"] = term.pedaling_nm;
        entry["saturated"] = term.saturated;
        if (term.active_request_nm.has_value())
            entry["active_request_nm"] = *term.active_request_nm;
        if (term.active_delivered_nm.has_value())
            entry["active_delivered_nm"] = *term.active_delivered_nm;
        if (term.passive_damping_nm.has_value())
            entry["passive_damping_nm"] = *term.passive_damping_nm;
        if (term.solved_force_nm.has_value())
            entry["solved_force_nm"] = *term.solved_force_nm;
        if (term.solved_active_nm.has_value())
            entry["solved_active_nm"] = *term.solved_active_nm;
        if (term.solved_passive_nm.has_value())
            entry["solved_passive_nm"] = *term.solved_passive_nm;
        out[name.c_str()] = entry;
    }
    return out;
}

nb::dict spindle_state_dict(const SpindleController::State &state) {
    nb::dict out;
    out["enabled"] = state.enabled;
    out["command_enabled"] = state.command_enabled;
    out["active_state"] =
        wire::owned_array<double>(state.active_state);
    set_or_none(out, "activation_time_s", state.activation_time_s);
    out["last_terms"] = last_terms_dict(state.last_terms);
    out["pd_split"] = nb::dict();
    out["saturated_ik"] = named_bools_dict(state.saturated_ik);
    out["ik_reach_limited"] = named_bools_dict(state.ik_reach_limited);
    out["joint_torques_nm"] = named_reals_dict(state.joint_torques_nm);
    out["joint_capacity_nm"] = named_reals_dict(state.joint_capacity_nm);
    set_or_none(out, "lean_limit_rad", state.lean_limit_rad);
    out["last_branch"] = nb::none();
    out["last_solution"] = nb::none();
    nb::dict recovery;
    for (const auto &[name, pedal] : state.pedal_recovery) {
        nb::dict entry;
        entry["stage"] = pedal.stage;
        entry["direction"] = pedal.direction;
        entry["release_offset_x_m"] = pedal.release_offset_x_m;
        recovery[name.c_str()] = entry;
    }
    out["pedal_recovery"] = recovery;
    out["effort_diagnostics"] =
        effort_diagnostics_dict(state.effort_diagnostics);
    out["allocation_diagnostics"] =
        allocation_diagnostics_dict(state.allocation_diagnostics);
    out["support_diagnostics"] =
        support_diagnostics_dict(state.support_diagnostics);
    out["sole_goal_diagnostics"] =
        sole_goal_diagnostics_dict(state.sole_goal_diagnostics);
    return out;
}

} // namespace rider
