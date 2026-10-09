// rider/intent_wire.cpp — wire decoders/emitters for the seated-climb
// intent stack. See the header for the shape contract.
//
// Decode validation mirrors the dataclass __post_init__ order of each
// Python type so error precedence matches; resolve() arguments stay
// lazily validated (the oracle never checks road_grade/lean_limit_rad on
// non-tick steps, so the wire accepts non-finite reals there and lets
// policy.update reject them exactly where Python does).
#include "intent_wire.hpp"

#include <array>
#include <optional>
#include <string>
#include <utility>

#include <nanobind/stl/string.h>

#include "../binding_readers.hpp"
#include "spindle_wire.hpp"

namespace rider {
namespace {

using wire::keys;

[[nodiscard]] IntentSignals parse_signals(nb::handle value,
                                          std::string_view path) {
    const auto d = wire::mapping(value, path);
    wire::exact_keys(d,
                     keys("pitch_rate_up_rad_s", "specific_force_body_mps2",
                          "crank_rate_rad_s", "human_crank_torque_nm",
                          "front_load_share"),
                     {}, path);
    IntentSignals signals;
    // SeatedClimbSignals.__post_init__ order: scalars, force array, share.
    signals.pitch_rate_up_rad_s =
        wire::finite_real(d["pitch_rate_up_rad_s"], path);
    signals.crank_rate_rad_s =
        wire::finite_real(d["crank_rate_rad_s"], path);
    signals.human_crank_torque_nm =
        wire::finite_real(d["human_crank_torque_nm"], path);
    signals.specific_force_body_mps2 =
        wire::fixed<3>(d["specific_force_body_mps2"], path);
    const nb::handle share = d["front_load_share"];
    if (!share.is_none()) {
        const double parsed = wire::finite_real(share, path);
        if (parsed < 0.)
            wire::invalid(path, "invalid front load share");
        if (parsed > 1.)
            wire::invalid(path,
                          "front load share must not exceed one");
        signals.front_load_share = parsed;
    }
    return signals;
}

[[nodiscard]] nb::object emit_optional_real(std::optional<double> value) {
    return value.has_value() ? nb::cast(*value) : nb::none();
}

[[nodiscard]] std::pair<double, IntentSignals>
parse_policy_sample(nb::handle value, std::string_view path) {
    const auto row = wire::sequence(value, path);
    if (row.size() != 2)
        wire::invalid(path, "incorrect sequence width");
    return {wire::finite_real(row[0], path),
            parse_signals(row[1], std::string(path) + ".signals")};
}

[[nodiscard]] std::pair<double, std::optional<double>>
parse_load_sample(nb::handle value, std::string_view path) {
    const auto row = wire::sequence(value, path);
    if (row.size() != 2)
        wire::invalid(path, "incorrect sequence width");
    return {wire::finite_real(row[0], path),
            wire::optional_real(row[1], path)};
}

[[nodiscard]] std::array<double, 3>
parse_pulse(nb::handle value, std::string_view path) {
    return wire::fixed<3>(value, path);
}

} // namespace

IntentConfig parse_intent_config(nb::handle value, std::string_view path) {
    const auto d = wire::mapping(value, path);
    wire::exact_keys(
        d,
        keys("enabled", "period_s", "reaction_delay_s",
             "target_crank_power_w", "max_crank_torque_nm",
             "torque_slew_nm_s", "lean_gain", "max_forward_lean_rad",
             "max_backward_lean_rad", "lean_rate_rad_s",
             "orientation_tau_s", "surge_power_w", "surge_grade",
             "surge_budget_s", "surge_recovery_rate",
             "front_load_share_target", "lean_trim_gain_rad_s",
             "lean_trim_limit_rad", "trim_dead_time_s"),
        {}, path);
    IntentConfig config;
    // SeatedClimbConfig.__post_init__ order: enabled, positive fields,
    // nonnegative fields, share bound, posture envelope bound.
    config.enabled = wire::boolean(d["enabled"], path);
    struct Bound {
        const char *key;
        double IntentConfig::*field;
        bool positive;
    };
    const std::array<Bound, 18> bounds{{
        {.key = "period_s", .field = &IntentConfig::period_s,
         .positive = true},
        {.key = "target_crank_power_w",
         .field = &IntentConfig::target_crank_power_w, .positive = true},
        {.key = "max_crank_torque_nm",
         .field = &IntentConfig::max_crank_torque_nm, .positive = true},
        {.key = "torque_slew_nm_s", .field = &IntentConfig::torque_slew_nm_s,
         .positive = true},
        {.key = "lean_rate_rad_s", .field = &IntentConfig::lean_rate_rad_s,
         .positive = true},
        {.key = "orientation_tau_s", .field = &IntentConfig::orientation_tau_s,
         .positive = true},
        {.key = "surge_power_w", .field = &IntentConfig::surge_power_w,
         .positive = true},
        {.key = "surge_budget_s", .field = &IntentConfig::surge_budget_s,
         .positive = true},
        {.key = "reaction_delay_s", .field = &IntentConfig::reaction_delay_s,
         .positive = false},
        {.key = "lean_gain", .field = &IntentConfig::lean_gain,
         .positive = false},
        {.key = "max_forward_lean_rad",
         .field = &IntentConfig::max_forward_lean_rad, .positive = false},
        {.key = "max_backward_lean_rad",
         .field = &IntentConfig::max_backward_lean_rad, .positive = false},
        {.key = "surge_grade", .field = &IntentConfig::surge_grade,
         .positive = false},
        {.key = "surge_recovery_rate",
         .field = &IntentConfig::surge_recovery_rate, .positive = false},
        {.key = "front_load_share_target",
         .field = &IntentConfig::front_load_share_target, .positive = false},
        {.key = "lean_trim_gain_rad_s",
         .field = &IntentConfig::lean_trim_gain_rad_s, .positive = false},
        {.key = "lean_trim_limit_rad",
         .field = &IntentConfig::lean_trim_limit_rad, .positive = false},
        {.key = "trim_dead_time_s", .field = &IntentConfig::trim_dead_time_s,
         .positive = false},
    }};
    for (const auto &[key, field, positive] : bounds) {
        const double parsed = wire::finite_real(d[key], path);
        if (positive ? parsed <= 0. : parsed < 0.)
            wire::invalid(std::string(path) + "." + key,
                          "invalid value or out of range");
        config.*field = parsed;
    }
    if (config.front_load_share_target > 1.)
        wire::invalid(path, "front load share target must not exceed one");
    if (std::max(config.max_forward_lean_rad,
                 config.max_backward_lean_rad) > .8)
        wire::invalid(path,
                      "seated lean exceeds the posture envelope");
    return config;
}

IntentSignals parse_intent_signals(nb::handle value, std::string_view path) {
    return parse_signals(value, path);
}

std::int64_t parse_intent_step(nb::handle value, std::string_view path) {
    // rider_intent.py resolve(): `type(step) is not int` — bools, numpy
    // integers and int subclasses are all rejected by the oracle.
    if (!value.is_valid() || !PyLong_CheckExact(value.ptr()))
        wire::invalid(path, "expected integer");
    const auto result = PyLong_AsLongLong(value.ptr());
    if (PyErr_Occurred()) wire::conversion_error(path);
    return result;
}

IntentControl parse_intent_control(nb::handle value, std::string_view path) {
    const auto d = wire::mapping(value, path);
    wire::exact_keys(d,
                     keys("motor_torque_nm", "motor_limit_nm",
                          "human_torque_nm", "crank_target_rate_rad_s",
                          "posture", "rider_enabled"),
                     {}, path);
    IntentControl control;
    // RideControl.__post_init__ order: the four optional nonneg reals,
    // posture type, rider_enabled bool.
    for (const auto &[key, field] :
         std::array<std::pair<const char *,
                              std::optional<double> IntentControl::*>,
                    4>{{{"motor_torque_nm", &IntentControl::motor_torque_nm},
                        {"motor_limit_nm", &IntentControl::motor_limit_nm},
                        {"human_torque_nm",
                         &IntentControl::human_torque_nm},
                        {"crank_target_rate_rad_s",
                         &IntentControl::crank_target_rate_rad_s}}}) {
        const auto parsed = wire::optional_real(d[key], path);
        if (parsed.has_value() && *parsed < 0.)
            wire::invalid(std::string(path) + "." + key,
                          "invalid value or out of range");
        control.*field = parsed;
    }
    const nb::handle posture = d["posture"];
    if (!posture.is_none())
        control.posture = parse_rider_posture(
            posture, std::string(path) + ".posture");
    control.rider_enabled = wire::boolean(d["rider_enabled"], path);
    return control;
}

namespace {

// Per-section decoders — the stack-budget sweep bounds each function's
// frame even on sanitizer builds, so the state sections decode in their
// own scopes.
Intent parse_intent_block(nb::handle value, const std::string &path) {
    const auto intent = wire::mapping(value, path);
    wire::exact_keys(intent, keys("posture", "effort_ceiling_nm"), {},
                     path);
    Intent block;
    block.posture =
        parse_rider_posture(intent["posture"], path + ".posture");
    block.effort_ceiling_nm =
        wire::finite_real(intent["effort_ceiling_nm"], path);
    return block;
}

SeatedClimbPolicy::State
parse_intent_policy(nb::handle value, const std::string &policy_path) {
    const auto policy = wire::mapping(value, policy_path);
    wire::exact_keys(policy,
                     keys("time_s", "inclination_rad", "lean_rad",
                          "effort_nm", "samples", "delayed",
                          "surge_budget_s_left"),
                     {}, policy_path);
    SeatedClimbPolicy::State out;
    out.time_s = wire::finite_real(policy["time_s"], policy_path);
    out.inclination_rad =
        wire::finite_real(policy["inclination_rad"], policy_path);
    out.lean_rad = wire::finite_real(policy["lean_rad"], policy_path);
    out.effort_nm = wire::finite_real(policy["effort_nm"], policy_path);
    for (nb::handle const item :
         wire::sequence(policy["samples"], policy_path))
        out.samples.push_back(
            parse_policy_sample(item, policy_path + ".samples"));
    const nb::handle delayed = policy["delayed"];
    if (!delayed.is_none())
        out.delayed = parse_signals(delayed, policy_path + ".delayed");
    out.surge_budget_s_left =
        wire::finite_real(policy["surge_budget_s_left"], policy_path);
    return out;
}

SeatedPostureProgram::State
parse_intent_program(nb::handle value, const std::string &program_path) {
    const auto program = wire::mapping(value, program_path);
    wire::exact_keys(program,
                     keys("lean_rad", "trim_rad", "load_samples",
                          "delayed_load_share", "pulses"),
                     {}, program_path);
    SeatedPostureProgram::State out;
    out.lean_rad = wire::finite_real(program["lean_rad"], program_path);
    out.trim_rad = wire::finite_real(program["trim_rad"], program_path);
    for (nb::handle const item :
         wire::sequence(program["load_samples"], program_path))
        out.load_samples.push_back(
            parse_load_sample(item, program_path + ".load_samples"));
    out.delayed_load_share =
        wire::optional_real(program["delayed_load_share"], program_path);
    for (nb::handle const item :
         wire::sequence(program["pulses"], program_path))
        out.pulses.push_back(
            parse_pulse(item, program_path + ".pulses"));
    return out;
}

} // namespace

RiderIntent::State parse_intent_state(nb::handle value,
                                      std::string_view path) {
    const auto d = wire::mapping(value, path);
    wire::exact_keys(d, keys("last_tick_step", "intent", "policy", "program"),
                     {}, path);
    RiderIntent::State state;
    state.last_tick_step =
        parse_intent_step(d["last_tick_step"],
                          std::string(path) + ".last_tick_step");
    state.intent = parse_intent_block(d["intent"],
                                      std::string(path) + ".intent");
    state.policy = parse_intent_policy(d["policy"],
                                       std::string(path) + ".policy");
    state.program = parse_intent_program(d["program"],
                                         std::string(path) + ".program");
    return state;
}

nb::dict intent_signals_dict(const IntentSignals &signals) {
    nb::dict out;
    out["pitch_rate_up_rad_s"] = signals.pitch_rate_up_rad_s;
    // seated_climb.py _signals_dict emits tuple(...) — the container type is
    // observable through dict equality.
    out["specific_force_body_mps2"] = nb::make_tuple(
        signals.specific_force_body_mps2[0],
        signals.specific_force_body_mps2[1],
        signals.specific_force_body_mps2[2]);
    out["crank_rate_rad_s"] = signals.crank_rate_rad_s;
    out["human_crank_torque_nm"] = signals.human_crank_torque_nm;
    out["front_load_share"] =
        emit_optional_real(signals.front_load_share);
    return out;
}

nb::dict rider_posture_dict(const RiderPosture &posture) {
    nb::dict out;
    out["torso_lean_rad"] = posture.torso_lean_rad;
    out["pelvis_pitch_rad"] = posture.pelvis_pitch_rad;
    if (posture.pelvis_offset_m.has_value()) {
        nb::list offset;
        offset.append((*posture.pelvis_offset_m)[0]);
        offset.append((*posture.pelvis_offset_m)[1]);
        out["pelvis_offset_m"] = offset;
    } else {
        out["pelvis_offset_m"] = nb::none();
    }
    out["use_saddle"] = posture.use_saddle;
    return out;
}

nb::dict seated_intent_dict(const Intent &intent) {
    nb::dict out;
    out["posture"] = rider_posture_dict(intent.posture);
    out["effort_ceiling_nm"] = intent.effort_ceiling_nm;
    return out;
}

nb::dict intent_control_dict(const IntentControl &control) {
    nb::dict out;
    out["motor_torque_nm"] = emit_optional_real(control.motor_torque_nm);
    out["motor_limit_nm"] = emit_optional_real(control.motor_limit_nm);
    out["human_torque_nm"] = emit_optional_real(control.human_torque_nm);
    out["crank_target_rate_rad_s"] =
        emit_optional_real(control.crank_target_rate_rad_s);
    out["posture"] = control.posture.has_value()
                         ? nb::cast(rider_posture_dict(*control.posture))
                         : nb::none();
    out["rider_enabled"] = control.rider_enabled;
    return out;
}

nb::dict intent_state_dict(const RiderIntent::State &state) {
    nb::dict out;
    out["last_tick_step"] = state.last_tick_step;
    out["intent"] = seated_intent_dict(state.intent);
    {
        nb::dict policy;
        policy["time_s"] = state.policy.time_s;
        policy["inclination_rad"] = state.policy.inclination_rad;
        policy["lean_rad"] = state.policy.lean_rad;
        policy["effort_nm"] = state.policy.effort_nm;
        nb::list samples;
        for (const auto &[time_s, signals] : state.policy.samples) {
            nb::tuple row = nb::make_tuple(time_s, intent_signals_dict(signals));
            samples.append(row);
        }
        policy["samples"] = samples;
        policy["delayed"] =
            state.policy.delayed.has_value()
                ? nb::cast(intent_signals_dict(*state.policy.delayed))
                : nb::none();
        policy["surge_budget_s_left"] = state.policy.surge_budget_s_left;
        out["policy"] = policy;
    }
    {
        nb::dict program;
        program["lean_rad"] = state.program.lean_rad;
        program["trim_rad"] = state.program.trim_rad;
        nb::list load_samples;
        for (const auto &[time_s, share] : state.program.load_samples)
            load_samples.append(
                nb::make_tuple(time_s, emit_optional_real(share)));
        program["load_samples"] = load_samples;
        program["delayed_load_share"] =
            emit_optional_real(state.program.delayed_load_share);
        nb::list pulses;
        for (const auto &[start, duration, amplitude] :
             state.program.pulses)
            pulses.append(nb::make_tuple(start, duration, amplitude));
        program["pulses"] = pulses;
        out["program"] = program;
    }
    return out;
}

} // namespace rider
