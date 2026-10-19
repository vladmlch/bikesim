#include "input_detail.hpp"
#include "../recording/catalog.hpp"
#include "../recording/encoding.hpp"

namespace bike::replay {
namespace {
using namespace detail;
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
Value decoded(const Value& value, const recording::FieldSpec& spec) {
    if (io::is_null(value)) return value;
    if (spec.dtype == "struct") {
        Object out;
        for (const auto& child : spec.children) {
            const auto* item = find(value, child.key); const auto* present = find(value, child.key + "_present");
            if (child.optional && (present ? !io::as_bool(*present, "presence") : !item || io::is_null(*item))) continue;
            if (!item && !child.optional) throw recording::RecordingError("missing research structure field: " + child.key);
            out.emplace_back(child.key, item ? decoded(*item, child) : Value{});
        }
        return Value{std::move(out)};
    }
    if (spec.dtype == "struct[]") {
        auto child = spec; child.dtype = "struct"; Array out;
        for (const auto& item : io::as_array(value, "struct[]")) out.push_back(decoded(item, child));
        return Value{std::move(out)};
    }
    if (spec.dtype == "map<struct>") {
        auto child = spec; child.dtype = "struct"; Object out;
        for (const auto& [name, item] : io::as_object(value, "map<struct>")) out.emplace_back(name, decoded(item, child));
        return Value{std::move(out)};
    }
    return value;
}
std::vector<Value> stream(const recording::Recording& reader, const recording::TableSpec& spec, std::int64_t episode) {
    const auto& identity = spec.keys.at(1).key; const std::array params{integer(episode)};
    auto cursor = reader.stream("SELECT * FROM " + quote(spec.name) + " WHERE episode_id=? ORDER BY " + quote(identity), params);
    std::vector<Value> out;
    for (;;) {
        const auto batch = cursor.next_batch(); if (batch.empty()) break;
        std::vector<Object> rows;
        for (const auto& row : batch) {
            Object decoded_row;
            auto const take = [&](const recording::FieldSpec& item) {
                const auto* value = find(row, item.key); if (!value) throw recording::RecordingError("missing recorded stream field: " + item.key);
                if (item.optional && !io::as_bool(field(row, item.key + "_present"), "recorded presence")) return;
                decoded_row.emplace_back(item.key, decoded(*value, item));
            };
            for (const auto& item : spec.keys) take(item);
            for (const auto& item : spec.fields) take(item);
            rows.push_back(std::move(decoded_row));
        }
        // Re-encoding is the source's semantic presence/shape validation.
        (void)recording::encode_rows(spec, rows, episode);
        for (auto& row : rows) { (void)pop(row, "episode_id"); (void)pop(row, identity); out.push_back(Value{std::move(row)}); }
    }
    return out;
}
ReplayInput database_input(const std::filesystem::path& path, std::optional<std::int64_t> episode) {
    const recording::Recording reader(path);
    const auto selected = reader.episode(episode, true);
    if (!io::as_bool(field(selected, "replay_eligible"), "replay_eligible"))
        throw recording::RecordingError("failure/incomplete-prefix evidence is analyzable but not completed replay");
    ReplayInput out; out.episode_id = count(field(selected, "episode_id")); out.container_schema_version = 1; out.replay_contract_version = 3;
    out.summary = io::parse_recording_json(string(field(selected, "summary_json")));
    const std::array params{integer(out.episode_id)};
    for (const auto& row : reader.rows("SELECT name,content FROM assets WHERE episode_id=? ORDER BY name", params))
        out.assets.emplace(string(field(row, "name")), std::get<io::Bytes>(field(row, "content").data).value);
    const auto recipe = io::parse_recording_json(asset_text(out, "recipe.json"));
    std::set<std::string> expected_recipe{"replay_contract_version", "start_x_m", "heightfield", "state_signature", "execution"};
    for (auto key : {"joint_envelope_file", "joint_strength_file"}) if (find(recipe, key)) expected_recipe.insert(key);
    exact_keys(recipe, expected_recipe, "replay recipe");
    if (count(field(recipe, "replay_contract_version")) != 3 || string(field(recipe, "state_signature")) != "mjSTATE_INTEGRATION")
        throw recording::RecordingError("invalid replay contract 3 recipe");
    (void)io::as_double(field(recipe, "start_x_m"), "start_x_m"); (void)io::as_object(field(recipe, "heightfield"), "heightfield");
    validate_execution(field(recipe, "execution"));
    const auto& research = field(out.summary, "research");
    if (!equal(field(research, "execution"), field(recipe, "execution"))) throw recording::RecordingError("summary and recipe execution identities disagree");
    for (auto key : {"source_changed_during_run", "configuration_changed_during_run", "execution_changed_during_run"})
        if (!find(research, key) || !std::holds_alternative<bool>(field(research, key).data) || io::as_bool(field(research, key), key))
            throw recording::RecordingError(std::string("exact replay requires a checked unchanged identity: ") + key);
    if (const auto* error = find(research, "error"); error && !io::is_null(*error) && (!std::holds_alternative<std::string>(error->data) || !string(*error).empty()))
        throw recording::RecordingError("failed recording is not replay eligible");
    if (const auto* metadata = find(research, "run_metadata"))
        if (const auto* changed = find(*metadata, "policy_source_changed_during_run"); changed && !io::is_null(*changed) && io::as_bool(*changed, "policy changed"))
            throw recording::RecordingError("policy identity changed during recording");
    const auto resolved = io::parse_recording_json(asset_text(out, "resolved_config.json"));
    if (!equal(resolved, field(out.summary, "resolved_config"))) throw recording::RecordingError("bundled and summarized resolved configurations disagree");
    const auto& articulated = field(field(resolved, "physics"), "articulated");
    std::set<std::string> expected_assets{"track.toml", "terrain_vertices.npy", "resolved_config.json", "recipe.json"};
    for (auto [key, name] : {std::pair{"joint_envelope", "rider_joint_envelope.json"}, std::pair{"joint_strength", "rider_joint_strength.json"}}) {
        const auto* declared = find(articulated, std::string(key) + "_path"); const auto* bundled = find(recipe, std::string(key) + "_file");
        if ((declared && !io::is_null(*declared)) != (bundled != nullptr) || (bundled && (string(*bundled) != name || !out.assets.contains(name) ||
            io::sha256(out.assets.at(name)) != string(field(resolved, std::string(key) + "_sha256")))))
            throw recording::RecordingError(std::string("inconsistent recorded ") + key + " bundle");
        if (bundled) expected_assets.insert(name);
    }
    std::set<std::string> actual_assets; for (const auto& [name, ignored] : out.assets) { (void)ignored; actual_assets.insert(name); }
    if (actual_assets != expected_assets) throw recording::RecordingError("replay recipe must own exactly its fixed input assets");
    const auto& extents = field(out.summary, "recording_extents");
    for (auto key : {"physical_rows", "complete_control_steps", "replay_eligible"})
        if (!equal(field(extents, key), field(selected, key))) throw recording::RecordingError(std::string("summary and database extents disagree: ") + key);
    out.manifest = Value{Object{{"schema_version", integer(3)}, {"start_x_m", field(recipe, "start_x_m")}, {"heightfield", field(recipe, "heightfield")},
        {"state_signature", field(recipe, "state_signature")}, {"execution", field(recipe, "execution")}, {"container_schema_version", integer(1)},
        {"container_manifest", reader.manifest()}, {"episode_id", integer(out.episode_id)}}};
    for (auto key : {"joint_envelope_file", "joint_strength_file"}) if (const auto* value = find(recipe, key)) set(out.manifest, key, *value);
    out.requested = stream(reader, recording::table_spec("commands_requested"), out.episode_id);
    out.applied = stream(reader, recording::table_spec("commands_applied"), out.episode_id);
    out.observations = stream(reader, recording::table_spec("observations"), out.episode_id);
    out.transitions = stream(reader, recording::table_spec("transitions"), out.episode_id);
    for (std::size_t i = 0; i < out.transitions.size(); ++i) {
        auto& row = std::get<Object>(out.transitions[i].data);
        if (count(pop(row, "request_id")) != static_cast<std::int64_t>(i) || count(pop(row, "previous_observation_id")) != static_cast<std::int64_t>(i) ||
            count(pop(row, "observation_id")) != static_cast<std::int64_t>(i) + 1 || i + 1 >= out.observations.size())
            throw recording::RecordingError("inconsistent transition observation/request references");
        set(row, "observation", out.observations[i + 1]);
    }
    command_shapes(out);
    std::set<std::string> phases;
    for (const auto& row : reader.rows("SELECT phase,state FROM integration_states WHERE episode_id=?", params)) {
        const auto phase = string(field(row, "phase")); if (!phases.insert(phase).second) throw recording::RecordingError("duplicate integration state phase");
        if (phase == "initial") out.initial_state = double_vector(field(row, "state"), "initial state");
        else if (phase == "final") out.final_state = double_vector(field(row, "state"), "final state");
        else throw recording::RecordingError("unknown integration state phase");
    }
    if (out.initial_state.empty() || out.final_state.size() != out.initial_state.size()) throw recording::RecordingError("invalid initial/final integration state for checked replay");
    const auto vertices = io::parse_npy(out.assets.at("terrain_vertices.npy"));
    if ((vertices.dtype != "<f8" && vertices.dtype != "=f8" && vertices.dtype != ">f8") || vertices.shape.size() != 2 || vertices.shape[1] != 2)
        throw recording::RecordingError("terrain must be a finite float64 N-by-2 array");
    out.terrain_vertices = vertices.doubles();
    const auto canonical = io::double_array(out.terrain_vertices, {out.terrain_vertices.size() / 2, 2});
    if (io::sha256(canonical.bytes) != string(field(out.summary, "terrain_sha256"))) throw recording::RecordingError("terrain identity differs from the bundled compiled road");
    std::int64_t completed = 0;
    for (std::size_t i = 0; i < out.transitions.size(); ++i) {
        if (count(field(out.requested[i], "step")) != completed) throw recording::RecordingError("requested controls do not cover consecutive physical windows");
        const auto steps = count(field(out.transitions[i], "physics_steps"));
        if (steps <= 0 || completed > std::numeric_limits<std::int64_t>::max() - steps) throw recording::RecordingError("invalid transition physics extent");
        completed += steps;
    }
    if (completed != count(field(selected, "physical_rows")) || completed != count(field(field(out.summary, "outcome"), "steps")))
        throw recording::RecordingError("completed controls do not cover the saved physical prefix");
    out.metrics = maybe(out.summary, "episode_metrics");
    return out;
}
} // namespace
ReplayInput load_replay_input(const std::filesystem::path& path, std::optional<std::int64_t> episode) {
    for (auto name : {"manifest.json", "run.duckdb", "UNSEALED"})
        if (std::filesystem::exists(path / name) || std::filesystem::is_symlink(path / name)) return database_input(path, episode);
    return load_legacy_input(path, episode);
}
} // namespace bike::replay
