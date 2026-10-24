#include "evidence.hpp"
#include "../runtime/setup_values.hpp"
#include <algorithm>
#include <cmath>
#include <ranges>

namespace bike::research {
namespace {
using namespace runtime::setupvalue;
bool status_flag(const runtime::WireObject& s, std::string_view key) {
    return runtime::sample_wire::boolean(runtime::sample_wire::required(s, key));
}
const Value* member(const Value& object, std::string_view key) {
    const auto* values = std::get_if<Object>(&object.data);
    return values ? io::find_member(*values, key) : nullptr;
}
bool metadata_flag(const Value& metadata, std::string_view key) {
    const auto* value = member(metadata, key);
    return value && std::holds_alternative<bool>(value->data) && std::get<bool>(value->data);
}
std::string_view capture_name(runtime::ResearchCaptureMode mode) {
    switch (mode) {
        case runtime::ResearchCaptureMode::off: return "off";
        case runtime::ResearchCaptureMode::full: return "full";
        case runtime::ResearchCaptureMode::verify: return "verify";
        case runtime::ResearchCaptureMode::legacy: return "legacy";
    }
    throw std::invalid_argument("unknown research capture mode");
}
} // namespace

io::Value physical_summary(const runtime::OwnedResearchEpisode& episode) {
    namespace wire = runtime::sample_wire;
    Value out = std::holds_alternative<Object>(episode.configuration.metadata.data)
        ? episode.configuration.metadata : Value{Object{}};
    const auto& s = episode.evaluation;
    const auto& accounting = wire::object(wire::required(s, "accounting"));
    const auto& history = wire::object(wire::required(accounting, "history"));
    const auto& balance = wire::required(s, "balance_event");
    Value outcome{Object{{"reason", wire_value(wire::required(s, "reason"))},
        {"time_s", wire_value(wire::required(s, "time_s"))}, {"position_m", wire_value(wire::required(s, "position_m"))},
        {"steps", integer(episode.final_snapshot.step)}, {"crashed", Value{!wire::is_none(wire::required(s, "crash"))}},
        {"balance_lost", Value{!wire::is_none(balance)}},
        {"balance_lost_at_m", wire::is_none(balance) ? Value{} : wire_value(wire::required(wire::object(balance), "position_m"))}}};
    set(out, "outcome", std::move(outcome));
    if(const auto* spatial=wire::find(s,"spatial"))set(out,"spatial",wire_value(*spatial));
    set(out, "first_failure", episode.final_snapshot.first_failure ? wire_value(episode.final_snapshot.first_failure->as_wire()) : Value{});
    set(out, "energy", wire_value(wire::required(accounting, "energy")));
    set(out, "model_status", wire_value(runtime::Wire{episode.final_snapshot.model_status}));
    set(out, "battery_energy_j", wire_value(wire::required(s, "battery_energy_j")));
    for (const auto& [source, destination] : {std::pair{"work_j", "component_work_j"},
                                           std::pair{"airtime_s", "airtime_threshold_s"}, std::pair{"duration_s", "duration_s"}})
        if (const auto* value = wire::find(history, source)) set(out, destination, wire_value(*value));
    if (const auto* setup = member(episode.configuration.metadata, "setup"))
        if (const auto* equilibrium = member(*setup, "equilibrium")) set(out, "equilibrium", *equilibrium);
    return out;
}

io::Value summarize(const runtime::OwnedResearchEpisode& episode) {
    namespace wire = runtime::sample_wire;
    const auto& s = episode.evaluation;
    const auto& config = episode.configuration;
    auto out = physical_summary(episode);
    const auto* saved_experiment = member(config.metadata, "experiment");
    const auto* saved_sensors = member(config.metadata, "sensor_config");
    const auto* execution = member(config.metadata, "execution");
    const bool error = !wire::is_none(wire::required(s, "error"));
    const bool launch_failure = member(episode.run_metadata, "launch_failure") != nullptr;
    const auto* seal = member(episode.run_metadata, "seal_reason");
    const auto seal_reason = seal ? io::as_string(*seal, "seal reason") : std::string{};
    const bool failure_seal = seal_reason == "simulation_error" || seal_reason == "policy_error" ||
        seal_reason == "invalid_controller";
    Value research{Object{{"config", saved_experiment ? *saved_experiment : Value{}},
        {"sensor_config", saved_sensors ? *saved_sensors : Value{}},
        {"model_status", wire_value(runtime::Wire{episode.final_snapshot.model_status})},
        {"numerically_valid", wire_value(wire::required(s, "numerically_valid"))},
        {"max_energy_residual_ratio", wire_value(wire::required(s, "max_energy_residual_ratio"))},
        {"actual_sensor_seed", Value{io::Number{validate_seed(config.seed_decimal)}}},
        {"control_steps", integer(config.control_steps)}, {"actuator_delay_steps", integer(config.delay_steps)},
        {"metrics", wire_value(wire::required(s, "metrics"))}, {"terminated", wire_value(wire::required(s, "terminated"))},
        {"truncated", wire_value(wire::required(s, "truncated"))}, {"error", wire_value(wire::required(s, "error"))},
        {"execution", execution ? *execution : Value{}}, {"run_metadata", episode.run_metadata},
        {"policy_inputs", text("proper acceleration, pitch gyro, wheel/crank encoders, motor/human torque sensors")},
        {"privileged_outputs", text("contact loads/clearances, road-relative pitch, CoM and simulator speed")},
        {"timestep_alignment", text("truth/sensor source is the last incoming physical state, not the endpoint")},
        {"parameters_validated_against_measurements", Value{false}}, {"capture_mode", text(capture_name(config.capture_mode))},
        {"measurement_decimation", integer(config.record_decimation)}, {"physical_downsampling", Value{false}},
        {"valid_for_learning", Value{status_flag(s, "numerically_valid") && status_flag(s, "model_valid") && !error && !launch_failure}}}};
    if(const auto* spatial=wire::find(s,"spatial")) {
        set(research,"spatial",wire_value(*spatial));
        if(!wire::boolean(wire::required(wire::object(*spatial),"valid")))set(research,"valid_for_learning",Value{false});
    }
    bool identities_unchanged = true;
    const auto* receipt = member(episode.run_metadata, "identity_checks");
    for (const auto key : {"source_changed_during_run", "configuration_changed_during_run", "execution_changed_during_run"}) {
        const auto* value = receipt ? member(*receipt, key) : nullptr;
        set(research, key, value ? *value : Value{});
        identities_unchanged = identities_unchanged && value && std::holds_alternative<bool>(value->data) && !std::get<bool>(value->data);
    }
    if (receipt)
        if (const auto* error_text = member(*receipt, "identity_check_error")) set(research, "identity_check_error", *error_text);
    if (!config.programs.demand_frames.empty()) set(research, "demand_program", demand_program_value(config.programs));
    if (!config.programs.rider_frames.empty()) {
        set(research, "rider_program", rider_program_value(config.programs));
        set(research, "rider_command_recording", text("program_inputs"));
    }
    const auto requested_count = wire::integer(wire::required(s, "requested_count"));
    const auto completed_count = wire::integer(wire::required(s, "completed_count"));
    const bool finite = std::ranges::all_of(episode.initial_integration_state, [](double x) { return std::isfinite(x); }) &&
        std::ranges::all_of(episode.final_snapshot.integration_state, [](double x) { return std::isfinite(x); });
    const bool complete = config.capture_mode == runtime::ResearchCaptureMode::full &&
        !status_flag(s, "active") && !status_flag(s, "delivery_pending") && requested_count == completed_count &&
        std::cmp_equal(episode.requested.size(), requested_count) && std::cmp_equal(episode.transitions.size(), completed_count) &&
        std::cmp_equal(episode.physical.samples.size(), episode.final_snapshot.step);
    const bool eligible = complete && finite && !episode.initial_integration_state.empty() && !error && !launch_failure && !failure_seal && identities_unchanged &&
        !metadata_flag(episode.run_metadata, "policy_source_changed_during_run");
    const auto& accounting = wire::object(wire::required(s, "accounting"));
    const auto& history = wire::object(wire::required(accounting, "history"));
    std::int64_t accounted = 0;
    if (const auto* last = wire::find(history, "last_id"); last && !wire::is_none(*last)) accounted = wire::integer(*last) + 1;
    Value extents{Object{{"physical_rows", integer(static_cast<std::int64_t>(episode.physical.samples.size()))},
        {"physical_steps", integer(episode.final_snapshot.step)}, {"accounted_steps", integer(accounted)},
        {"requested_controls", integer(static_cast<std::int64_t>(episode.requested.size()))},
        {"complete_control_steps", integer(static_cast<std::int64_t>(episode.transitions.size()))},
        {"accepted_control_count", integer(requested_count)}, {"completed_control_count", integer(completed_count)},
        {"observations", integer(static_cast<std::int64_t>(episode.observations.size()))},
        {"requested_duration_s", real(static_cast<double>(config.max_steps) * config.timestep_s)},
        {"achieved_duration_s", real(episode.final_snapshot.time_s)}, {"replay_eligible", Value{eligible}}}};
    set(out, "research", std::move(research)); set(out, "episode_metrics", episode_metrics(episode));
    set(out, "recording_extents", std::move(extents)); set(out, "research_replay_contract", integer(3));
    return out;
}

runtime::WireArray recording_batch(const runtime::OwnedResearchEpisode& episode, std::string_view stream,
                                   std::int64_t start, std::int64_t limit) {
    auto rows = episode.batch(stream, start, limit);
    for (std::size_t i = 0; i < rows.size(); ++i) {
        auto& row = std::get<runtime::WireObject>(rows[i].value);
        const auto index = start + static_cast<std::int64_t>(i);
        if (stream == "commands_requested") {
            runtime::sample_wire::set(row, "request_id", index);
            runtime::sample_wire::set(row, "observation_id", index);
        } else if (stream == "commands_applied") runtime::sample_wire::set(row, "applied_id", index);
        else if (stream == "observations") runtime::sample_wire::set(row, "observation_id", index);
        else if (stream == "transitions") {
            runtime::sample_wire::set(row, "transition_id", index);
            runtime::sample_wire::set(row, "request_id", index);
            runtime::sample_wire::set(row, "previous_observation_id", index);
            runtime::sample_wire::set(row, "observation_id", index + 1);
        }
    }
    return rows;
}
} // namespace bike::research
