#include "report.hpp"
#include "analysis_detail.hpp"
#include <array>
#include <functional>
#include <format>

namespace bike::recording {
namespace {
using namespace detail;
using Query = std::function<std::vector<Object>(std::string_view, std::span<const Value>)>;
std::string display(const Value& value) {
    if (io::is_null(value)) return "unavailable / null";
    const auto source = io::recording_json(value);
    std::string out;
    for (char const c : source) { if (c == '|') out += '\\'; out += c == '\n' ? ' ' : c; }
    return out;
}
std::string raw(const Object& row, std::string_view name) {
    const auto& value = field(row, name);
    if (const auto* text_value = std::get_if<std::string>(&value.data)) return *text_value;
    if (const auto* boolean = std::get_if<bool>(&value.data)) return *boolean ? "True" : "False";
    if (io::is_null(value)) return "None";
    return io::recording_json(value);
}
Value select_optional(const Value& value, std::initializer_list<std::string_view> keys) {
    Object out; for (auto key : keys) out.emplace_back(key, maybe(value, key)); return Value{std::move(out)};
}
std::string render(const Query& query) {
    const auto runs = query("SELECT * FROM runs", {});
    if (runs.size() != 1) throw RecordingError("report requires one run");
    const auto& run = runs[0];
    const auto episodes = query("SELECT * FROM episodes ORDER BY episode_id", {});
    const auto fields = query("SELECT * FROM field_catalog WHERE dtype='DOUBLE' AND "
        "(table_name='physics' OR table_name LIKE 'channel_%') ORDER BY table_name,field_name", {});
    std::string out = "# Run recording\n\nRun UUID: `" + raw(run, "run_id") + "`\n\nStarted (UTC): `" + raw(run, "started_at") +
        "`. Backend: `" + raw(run, "backend") + "`. Mode: `" + raw(run, "mode") + "`.\n\n## Invocation and provenance\n\n"
        "Evidence: `runs.metadata_json`; per-episode setup identity is in `episodes.summary_json`.\n\n```json\n" +
        pretty_json(io::parse_recording_json(string(field(run, "metadata_json")))) + "\n```\n\n## Interpretation and limits\n\n"
        "This is saved numerical evidence, not a claim that the model is calibrated. Recording retains the launch in RAM and publishes at finalization; "
        "crashes, SIGKILL and out-of-memory termination have no durability guarantee. NULL, absence and nonfinite values are separate states. "
        "No cause is inferred from temporal coincidence.\n\nA forwarded endpoint measurement is not an instantaneous peak at every interval. "
        "The extrema below exclude forwarded energy rows; cumulative fields are reported as final values and are not summed. "
        "Missing/nonfinite coverage is reported explicitly.\n\n";
    if (episodes.empty()) out += "No episode was sealed. No physical or checked-replay claim is available.\n\n";
    for (const auto& episode : episodes) {
        const auto id = count(field(episode, "episode_id")); const std::array params{integer(id)};
        const auto saved = io::parse_recording_json(string(field(episode, "summary_json")));
        const auto extent = query("SELECT count(*) AS rows,min(interval_id) AS first_interval,max(interval_id) AS last_interval,"
            "min(time_s) AS first_time_s,max(interval_end_s) AS last_time_s FROM physics WHERE episode_id=?", params).at(0);
        out += "## Episode " + std::to_string(id) + "\n\nNative generation: `" + raw(episode, "generation") + "`. Physical prefix: `" +
            display(Value{extent}) + "`.\n\nCompleted control transitions: `" + raw(episode, "complete_control_steps") +
            "`. Checked research replay eligible: `" + raw(episode, "replay_eligible") + "` (contract `" + raw(episode, "replay_contract_version") +
            "`).\n\n### Outcome, validity and first failure\n\nEvidence: `episodes.summary_json`, selected by `episode_id`.\n\n```json\n" +
            pretty_json(select_optional(saved, {"outcome", "model_status", "first_failure", "failure", "tail_failure",
                "source_changed_during_run", "execution_changed_during_run"})) +
            "\n```\n\n### Important events\n\nSource events keep their recorded classification. `derived_transition` events describe retained channel changes "
            "under a named rule; they are not controller explanations.\n\nClock semantics: `received_wall_s` is the monotonic receipt timestamp "
            "(a wall clock, not simulation time); `interval_time_s` is the simulation start time of the interval the input was processed on, "
            "and is null when no interval carries that `processed_step`.\n\n"
            "| Event | Kind | Received wall (s) | Sim time (s) | Processed step | Applied step | Recorded detail |\n"
            "| --- | --- | --- | --- | --- | --- | --- |\n";
        const auto events = query("SELECT e.event_id,e.kind,e.received_wall_s,e.processed_step,e.applied_step,e.values_json,p.time_s AS interval_time_s "
            "FROM events e LEFT JOIN physics p ON p.episode_id=e.episode_id AND p.interval_id=e.processed_step WHERE e.episode_id=? "
            "ORDER BY e.event_id LIMIT 201", params);
        for (std::size_t i = 0; i < std::min<std::size_t>(events.size(), 200); ++i) {
            const auto& event = events[i];
            out += "| " + raw(event, "event_id") + " | " + raw(event, "kind") + " | " + display(field(event, "received_wall_s")) +
                " | " + display(field(event, "interval_time_s")) + " | " + raw(event, "processed_step") + " | " + display(field(event, "applied_step")) +
                " | " + display(io::parse_recording_json(string(field(event, "values_json")))) + " |\n";
        }
        if (events.empty()) out += "| - | No source events recorded | - | - | - | - | - |\n";
        if (events.size() > 200) out += "\nEvent display truncated at 200; remaining evidence is available with `SELECT * FROM events WHERE episode_id=" +
            std::to_string(id) + " ORDER BY event_id`.\n";
        out += "\n### Scalar coverage and extrema\n\n| Field | Unit / phase | Finite | Missing | Nonfinite | Minimum / final | Maximum |\n"
            "| --- | --- | --- | --- | --- | --- | --- |\n";
        for (const auto& field_row : fields) {
            const auto table = string(field(field_row, "table_name")), name = string(field(field_row, "field_name"));
            std::string domain = "episode_id=?";
            if (table == "channel_energy" && string(field(field_row, "measurement")) == "forwarded") domain += " AND measured IS TRUE";
            const auto column = quote(name);
            const auto values = query(std::format(
                "SELECT count(*) AS n,count({0}) AS present,count(*) FILTER(WHERE isfinite({0})) AS finite,"
                "min({0}) FILTER(WHERE isfinite({0})) AS minimum,max({0}) FILTER(WHERE isfinite({0})) AS maximum"
                " FROM {1} WHERE {2}", column, quote(table), domain), params).at(0);
            Value minimum = field(values, "minimum"), maximum = field(values, "maximum");
            if (string(field(field_row, "phase")) == "episode") {
                const auto last = query(std::format("SELECT {} AS value FROM {} WHERE {} ORDER BY interval_id DESC LIMIT 1", column, quote(table), domain), params);
                minimum = last.empty() ? Value{} : field(last[0], "value"); maximum = text("cumulative: not a peak");
            }
            out += std::format("| `{}`.{} | {} / {} | {} | {} | {} | {} | {} |\n", table, name,
                raw(field_row, "unit"), raw(field_row, "phase"), raw(values, "finite"),
                count(field(values, "n")) - count(field(values, "present")),
                count(field(values, "present")) - count(field(values, "finite")), display(minimum), display(maximum));
        }
        out += "\nReproduce each extrema row as `SELECT min(FIELD), max(FIELD) FROM TABLE WHERE episode_id=" + std::to_string(id) +
            " AND isfinite(FIELD)`; for `channel_energy` fields marked `forwarded` in `field_catalog`, also require `measured IS TRUE`. "
            "For phase `episode`, select the last `interval_id` instead of aggregating cumulative values.\n\n"
            "### Recorded component power integrals\n\nRule `left_rectangle_recorded_power.v1`: sum of recorded `power_w * dt_s`. "
            "This is a derived finite-subset integral when any power is missing/nonfinite; it does not replace the engine's recorded cumulative accounting.\n\n"
            "| Component | Intervals | Finite powers | Derived work (J) |\n| --- | --- | --- | --- |\n";
        const auto powers = query("SELECT f.component,count(*) AS n,count(*) FILTER(WHERE isfinite(f.power_w)) AS finite,"
            "sum(f.power_w*p.dt_s) FILTER(WHERE isfinite(f.power_w)) AS work_j FROM forces f JOIN physics p USING(episode_id,interval_id) "
            "WHERE episode_id=? GROUP BY f.component ORDER BY f.component", params);
        for (const auto& power : powers) out += "| `" + raw(power, "component") + "` | " + raw(power, "n") + " | " + raw(power, "finite") + " | " + display(field(power, "work_j")) + " |\n";
        if (powers.empty()) out += "| - | 0 | 0 | unavailable |\n";
        out += "\nEvidence query: `forces JOIN physics USING(episode_id,interval_id)`, `episode_id=" + std::to_string(id) +
            "`, grouped by `component`.\n\n### Episode metrics and source identity\n\n```json\n" +
            pretty_json(select_optional(saved, {"episode_metrics", "research", "resolved_config", "model_source_sha256", "execution"})) + "\n```\n\n";
    }
    out += "## Field and coordinate coverage\n\nThe complete finite field inventory is `field_catalog`; model-address identities are `coordinate_catalog`. "
        "`_present` columns describe existence independently of nullable values. Parent `_valid` flags distinguish an explicitly null parent from an empty "
        "or absent parent. Structured children carry their own optionality flags.\n\nDiagnostics not represented by this catalog were not recorded and "
        "cannot be reconstructed as source facts. Observation delivery and acquisition timestamps are separate; transitions reference observations "
        "rather than duplicating them.\n";
    return out;
}
} // namespace
std::string render_summary(const Recording& recording) {
    return render([&](std::string_view sql, std::span<const Value> parameters) { return recording.rows(sql, parameters); });
}
std::string render_report_database(const std::filesystem::path& path) {
    Database database(path, true);
    return render([&](std::string_view sql, std::span<const Value> parameters) { return database.connection().query(sql, parameters); });
}
} // namespace bike::recording
