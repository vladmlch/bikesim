#include "database.hpp"
#include "value_util.hpp"
#include "../io/hash.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <limits>
#include <map>
#include <set>
#include <format>

namespace bike::recording {
namespace {
using io::Value;
using Object = Value::Object;
std::int64_t count(Connection const& c, std::string_view sql, std::span<const Value> params = {}) {
    const auto rows = c.query(sql, params);
    if (rows.size() != 1 || rows.front().size() != 1) throw std::invalid_argument("recording validation count has invalid shape");
    return io::as_int64(rows.front().front().second, "recording count");
}
void require_zero(const Connection& c, const std::string& sql, std::string_view error, std::span<const Value> params = {}) {
    if (count(c, sql, params) != 0) throw std::invalid_argument(std::string(error));
}
std::string normalize(std::string value) {
    std::erase_if(value, [](unsigned char ch) { return ch == '"' || std::isspace(ch); });
    for (auto& ch : value) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    return value;
}
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
std::string nested_invalid(const FieldSpec& field, const std::string& expression, unsigned depth = 0) {
    if (depth > 64) throw std::invalid_argument("recording schema nesting exceeds validation bound");
    std::vector<std::string> invalid;
    if (field.dtype == "struct") {
        for (const auto& child : field.children) {
            const auto v = "(" + expression + ")." + quote(child.key);
            if (child.optional) {
                const auto present = std::format("({}).{}", expression, quote(child.key + "_present"));
                invalid.push_back(present + " IS NULL");
                invalid.push_back(std::format("(NOT {} AND {} IS NOT NULL)", present, v));
            } else invalid.push_back(v + " IS NULL");
            const auto nested = nested_invalid(child, v, depth+1);
            if (!nested.empty()) invalid.push_back(nested);
        }
    } else if (field.dtype.ends_with("[]") || field.dtype.starts_with("map<")) {
        const auto variable = "recording_element_" + std::to_string(depth);
        auto item = field;
        const bool map = field.dtype.starts_with("map<");
        item.dtype = map ? field.dtype.substr(4, field.dtype.size()-5) : field.dtype.substr(0, field.dtype.size()-2);
        const auto list = map ? "map_values(" + expression + ")" : expression;
        auto predicate = variable + " IS NULL";
        const auto nested = nested_invalid(item, variable, depth+1);
        if (!nested.empty()) predicate += " OR " + nested;
        invalid.push_back("len(list_filter(" + list + ", " + variable + " -> (" + predicate + "))) > 0");
        if (!map && field.shape && !field.shape->empty()) invalid.push_back("len(" + expression + ") != " + std::to_string(field.shape->front()));
        if (map) invalid.push_back("len(list_filter(map_keys(" + expression + "), " + variable + " -> (" + variable + " IS NULL OR " + variable + " = ''))) > 0");
    }
    return invalid.empty() ? std::string{} : "(" + expression + " IS NOT NULL AND (" + value::join(invalid, " OR ") + "))";
}
void presence(const Connection& c, const TableSpec& spec) {
    std::vector<std::string> invalid;
    const auto parents = parent_paths(spec);
    for (const auto& path : parents) {
        const auto marker = std::format("__{}", value::join(path));
        const auto present = quote(marker + "_present"), valid = quote(marker + "_valid");
        invalid.push_back(present + " IS NULL"); invalid.push_back(valid + " IS NULL"); invalid.push_back(std::format("({} AND NOT {})", valid, present));
        if (path.size() > 1) {
            const auto ancestor = quote(std::format("__{}_valid", value::join(std::span<const std::string>(path).first(path.size()-1))));
            invalid.push_back(std::format("({} AND NOT {})", present, ancestor));
        }
    }
    auto const field = [&](const FieldSpec& f) {
        const auto v = quote(f.key); std::vector<std::string> ancestors;
        for (std::size_t depth = 1; depth < f.path.size(); ++depth) ancestors.push_back(quote("__" + value::join(std::span<const std::string>(f.path).first(depth)) + "_valid"));
        auto active = ancestors.empty() ? std::string("TRUE") : value::join(ancestors, " AND ");
        const bool root = f.path.size() == 1 && std::ranges::find(spec.claims, f.path.front()) != spec.claims.end();
        const bool routed = root && f.optional && (f.dtype.starts_with("map<") || f.dtype == "struct" || f.dtype == "struct[]");
        const auto empty_remainder = routed ? "(" + quote("__" + f.path.front() + "_valid") + " AND NOT " + quote(f.key + "_present") + ")" : std::string("FALSE");
        if (f.optional) {
            const auto present = quote(f.key + "_present");
            invalid.push_back(present + " IS NULL"); invalid.push_back("(NOT " + present + " AND " + v + " IS NOT NULL)");
            invalid.push_back("(NOT (" + active + ") AND (" + present + " OR " + v + " IS NOT NULL))");
            if (root) invalid.push_back("(NOT " + empty_remainder + " AND " + present + " != " + quote("__" + f.path.front() + "_present") + ")");
        } else {
            if (root) active = quote("__" + f.path.front() + "_valid");
            invalid.push_back("((" + active + ") AND " + v + " IS NULL)");
            invalid.push_back("(NOT (" + active + ") AND " + v + " IS NOT NULL)");
        }
        // The encoder's empty routed map/struct remainder has no terminal
        // leaf. Preserve its source root masks; do not invent an empty value.
        if (root) invalid.push_back("(NOT " + empty_remainder + " AND " + quote("__" + f.path.front() + "_valid") + " != (" + v + " IS NOT NULL))");
        const auto nested = nested_invalid(f, v);
        if (!nested.empty()) invalid.push_back(nested);
    };
    for (const auto& f : spec.keys) field(f);
    for (const auto& f : spec.fields) field(f);
    if (!invalid.empty()) require_zero(c, "SELECT count(*) FROM " + quote(spec.name) + " WHERE " + value::join(invalid, " OR "), spec.name + ": inconsistent presence or missing required value");
}
void consecutive(Connection const& c, std::string_view table, std::string_view key, std::span<const Value> params, bool zero) {
    const auto rows = c.query("SELECT count(*) AS n, min(" + quote(key) + ") AS first, max(" + quote(key) + ") AS last FROM " + quote(table) + " WHERE episode_id=?", params);
    const auto& row = rows.at(0); const auto n = io::as_int64(value::require(row, "n"), "n");
    if (!n) return;
    const auto first = io::as_int64(value::require(row, "first"), "first"), last = io::as_int64(value::require(row, "last"), "last");
    if (first < 0 || last < first || last - first != n-1 || (zero && first != 0)) throw std::invalid_argument(std::string(table) + ": identities are not consecutive");
}
std::int64_t table_count(const Connection& c, std::string_view table, std::span<const Value> params) { return count(c, "SELECT count(*) FROM " + quote(table) + " WHERE episode_id=?", params); }
void check_keys(const Connection& c, std::string_view table, std::span<const std::string> keys) {
    std::vector<std::string> quoted, missing;
    for (const auto& key : keys) { quoted.push_back(quote(key)); missing.push_back(quote(key) + " IS NULL"); }
    const auto identity = value::join(quoted, ",");
    require_zero(c, "SELECT count(*) FROM " + quote(table) + " WHERE " + value::join(missing, " OR "), std::string(table) + ": null identity");
    require_zero(c, "SELECT count(*) FROM (SELECT " + identity + " FROM " + quote(table) + " GROUP BY " + identity + " HAVING count(*) != 1)", std::string(table) + ": duplicate identity");
}
std::set<std::string> validate_container_schema(const Connection& c) {
    const std::set<std::string> metadata{"recording_meta", "runs", "episodes", "events", "field_catalog", "coordinate_catalog", "alias_catalog", "assets", "integration_states", "metrics"};
    auto expected = metadata; for (const auto& spec : all_tables()) expected.insert(spec.name);
    std::set<std::string> actual;
    for (const auto& row : c.query("SELECT table_name FROM information_schema.tables WHERE table_schema='main' AND table_type='BASE TABLE'")) actual.insert(io::as_string(value::require(row, "table_name"), "table_name"));
    if (actual != expected) throw std::invalid_argument("recording table set differs from container schema 1");
    const auto meta = c.query("SELECT schema_version,run_id,codec FROM recording_meta");
    const auto runs = c.query("SELECT run_id FROM runs");
    if (meta.size() != 1 || runs.size() != 1 || io::as_int64(value::require(meta.front(), "schema_version"), "schema_version") != schema_version ||
        io::recording_json(value::require(meta.front(), "run_id")) != io::recording_json(value::require(runs.front(), "run_id")) ||
        io::as_string(value::require(meta.front(), "codec"), "codec") != "bike_recording.json.v1") throw std::invalid_argument("invalid recording identity/schema");
    for (const auto& spec : all_tables()) {
        const auto columns = c.query("DESCRIBE " + quote(spec.name)); const auto types = column_types(spec);
        if (columns.size() != types.size()) throw std::invalid_argument(spec.name + ": columns differ from schema 1");
        for (std::size_t i = 0; i < columns.size(); ++i) if (io::as_string(value::require(columns[i], "column_name"), "column_name") != types[i].first ||
            normalize(io::as_string(value::require(columns[i], "column_type"), "column_type")) != normalize(types[i].second)) throw std::invalid_argument(spec.name + ": column order/types differ from schema 1");
        presence(c, spec);
        std::vector<std::string> keys; keys.reserve(spec.keys.size());
        for (const auto& f : spec.keys) keys.push_back(f.key);
        check_keys(c, spec.name, keys);
    }
    const std::map<std::string, std::vector<std::string>> meta_keys{{"runs", {"run_id"}}, {"episodes", {"episode_id"}}, {"events", {"episode_id", "event_id"}},
        {"field_catalog", {"table_name", "field_name"}}, {"coordinate_catalog", {"episode_id", "kind", "coordinate_index"}},
        {"alias_catalog", {"episode_id", "alias"}}, {"assets", {"episode_id", "name"}}, {"integration_states", {"episode_id", "phase"}}, {"metrics", {"episode_id", "name"}}};
    for (const auto& [table, keys] : meta_keys) check_keys(c, table, keys);
    return actual;
}
void validate_declared_catalog(Connection const& c) {
    auto declared = catalog_rows();
    std::ranges::sort(declared, [](const auto& a, const auto& b) {
        const auto first = io::as_string(a[0], "table").compare(io::as_string(b[0], "table"));
        return first < 0 || (first == 0 && io::as_string(a[1], "field") < io::as_string(b[1], "field"));
    });
    const auto catalog = c.query("SELECT * FROM field_catalog ORDER BY table_name,field_name");
    if (catalog.size() != declared.size()) throw std::invalid_argument("field catalog differs from reviewed schema");
    for (std::size_t i = 0; i < catalog.size(); ++i) {
        Value::Array row; for (const auto& [name, v] : catalog[i]) { (void)name; row.push_back(v); }
        if (io::recording_json(Value{row}) != io::recording_json(Value{declared[i]})) throw std::invalid_argument("field catalog descriptor differs from reviewed schema");
    }
}
void validate_relationships_and_assets(const Connection& c) {
    for (auto table : {"assets", "integration_states", "metrics", "coordinate_catalog", "alias_catalog", "events"}) require_zero(c,
        "SELECT count(*) FROM " + quote(table) + " t LEFT JOIN episodes e USING(episode_id) WHERE e.episode_id IS NULL", std::string(table) + ": orphan episode");
    require_zero(c, "SELECT count(*) FROM episodes WHERE episode_id<0 OR generation<0 OR physical_rows<0 OR complete_control_steps<0", "negative episode identity/extent");
    require_zero(c, "SELECT count(*) FROM events WHERE event_id<0 OR processed_step<0 OR applied_step<processed_step OR NOT isfinite(received_wall_s)", "invalid event identity/clock");
    for (const auto& spec : all_tables()) {
        require_zero(c, "SELECT count(*) FROM " + quote(spec.name) + " t LEFT JOIN episodes e USING(episode_id) WHERE e.episode_id IS NULL", spec.name + ": orphan episode");
        if (spec.name == "forces" || spec.name.starts_with("channel_")) require_zero(c, "SELECT count(*) FROM " + quote(spec.name) + " t LEFT JOIN physics p USING(episode_id,interval_id) WHERE p.interval_id IS NULL", spec.name + ": orphan physical interval");
    }
    for (const auto& row : c.query("SELECT episode_id,name,sha256,content FROM assets")) {
        const auto* bytes = std::get_if<io::Bytes>(&value::require(row, "content").data);
        if (!bytes || io::sha256(bytes->value) != io::as_string(value::require(row, "sha256"), "asset.sha256")) throw std::invalid_argument("recording asset checksum differs");
    }
}
void validate_episode_row(const Connection& c, const Object& row) {
    const auto id = io::as_int64(value::require(row, "episode_id"), "episode_id");
    const std::array<Value,1> params{value::integer(id)};
    const auto physical_rows = io::as_int64(value::require(row, "physical_rows"), "physical_rows");
    const auto controls = io::as_int64(value::require(row, "complete_control_steps"), "complete_control_steps");
    const bool eligible = io::as_bool(value::require(row, "replay_eligible"), "replay_eligible");
    consecutive(c, "events", "event_id", params, true); consecutive(c, "physics", "interval_id", params, false);
    if (table_count(c, "physics", params) != physical_rows) throw std::invalid_argument("episode physical prefix incomplete");
    require_zero(c, "SELECT count(*) FROM physics WHERE episode_id=? AND (time_s IS NULL OR interval_end_s IS NULL OR dt_s IS NULL OR NOT isfinite(time_s) OR NOT isfinite(interval_end_s) OR NOT isfinite(dt_s) OR dt_s<=0 OR interval_end_s<=time_s OR dt_s!=interval_end_s-time_s)", "invalid physical interval clock", params);
    require_zero(c, "SELECT count(*) FROM (SELECT time_s,lag(interval_end_s) OVER (ORDER BY interval_id) AS previous_end FROM physics WHERE episode_id=?) WHERE previous_end IS NOT NULL AND time_s!=previous_end", "discontinuous physical clock", params);
    for (const auto& spec : all_tables()) if (spec.name.starts_with("channel_") && table_count(c, spec.name, params) != physical_rows) throw std::invalid_argument(spec.name + ": channel coverage differs from physical prefix");
    std::map<std::string,std::int64_t> dimensions;
    for (const auto& coordinate : c.query("SELECT kind,coordinate_index FROM coordinate_catalog WHERE episode_id=? ORDER BY kind,coordinate_index", params)) {
        const auto kind = io::as_string(value::require(coordinate, "kind"), "coordinate.kind");
        if (kind != "qpos" && kind != "dof") throw std::invalid_argument("invalid coordinate catalog kind");
        const auto index = io::as_int64(value::require(coordinate, "coordinate_index"), "coordinate_index");
        if (index != dimensions[kind]++) throw std::invalid_argument("invalid coordinate catalog identity");
    }
    for (const auto& [vector, kind] : {std::pair{"qpos", "qpos"}, std::pair{"qvel", "dof"}}) {
        const std::array<Value,2> width{value::integer(id), value::integer(dimensions[kind])};
        require_zero(c, "SELECT count(*) FROM physics WHERE episode_id=? AND (" + quote(vector) + " IS NULL OR len(" + quote(vector) + ")!=?)", "physical coordinate width differs", width);
    }
    const std::array<Value,2> dof{value::integer(id), value::integer(dimensions["dof"])};
    require_zero(c, "SELECT count(*) FROM forces WHERE episode_id=? AND (qfrc IS NULL OR len(qfrc)!=?)", "force coordinate width differs", dof);
    std::map<std::string,std::int64_t> counts;
    for (auto table : {"commands_requested", "commands_applied", "observations", "transitions", "integration_states"}) counts[table] = table_count(c, table, params);
    const auto& version = value::require(row, "replay_contract_version");
    if (io::is_null(version)) {
        if (eligible || controls || counts["observations"] || counts["transitions"] || counts["integration_states"]) throw std::invalid_argument("ordinary evidence cannot claim checked research replay");
        return;
    }
    if (io::as_int64(version, "replay_contract_version") != replay_contract_version || controls == std::numeric_limits<std::int64_t>::max() ||
        counts["transitions"] != controls || counts["observations"] != controls+1 ||
        (counts["commands_requested"] != controls && counts["commands_requested"] != controls+1) || counts["integration_states"] != 2)
        throw std::invalid_argument("incomplete research identities/extents");
    for (const auto& [table, key] : {std::pair{"commands_requested", "request_id"}, std::pair{"commands_applied", "application_id"}, std::pair{"observations", "observation_id"}, std::pair{"transitions", "transition_id"}}) consecutive(c, table, key, params, true);
    require_zero(c, "SELECT count(*) FROM transitions WHERE episode_id=? AND (request_id!=transition_id OR previous_observation_id!=transition_id OR observation_id!=transition_id+1)", "transition observation/request references disagree", params);
    require_zero(c, "SELECT count(*) FROM transitions WHERE episode_id=? AND (physics_steps IS NULL OR physics_steps<=0)", "completed transition must span positive physics steps", params);
    for (auto table : {"commands_requested", "commands_applied"}) require_zero(c, "SELECT count(*) FROM " + quote(table) + " WHERE episode_id=? AND (step IS NULL OR step<0 OR time_s IS NULL OR time_s<0 OR NOT isfinite(time_s))", "invalid research control clock", params);
    require_zero(c, "SELECT count(*) FROM observations WHERE episode_id=? AND (time_s IS NULL OR source_time_s IS NULL OR NOT isfinite(time_s) OR NOT isfinite(source_time_s) OR time_s<0 OR source_time_s<0 OR source_time_s>time_s+1e-12 OR specific_force_body_mps2 IS NULL OR len(specific_force_body_mps2)!=3)", "invalid observation acquisition/delivery clock or IMU vector", params);
    if (!counts["commands_applied"]) throw std::invalid_argument("research capture requires startup applied command");
    const auto states = c.query("SELECT phase,signature,len(state) AS width FROM integration_states WHERE episode_id=? ORDER BY phase", params);
    if (states.size() != 2 || io::as_string(value::require(states[0], "phase"), "phase") != "final" || io::as_string(value::require(states[1], "phase"), "phase") != "initial" ||
        io::as_string(value::require(states[0], "signature"), "signature") != "mjSTATE_INTEGRATION" || io::as_string(value::require(states[1], "signature"), "signature") != "mjSTATE_INTEGRATION" ||
        io::as_int64(value::require(states[0], "width"), "width") != io::as_int64(value::require(states[1], "width"), "width")) throw std::invalid_argument("invalid integration state pair");
    if (eligible && (counts["commands_requested"] != controls || count(c, "SELECT coalesce(sum(physics_steps),0) FROM transitions WHERE episode_id=?", params) != physical_rows)) throw std::invalid_argument("incomplete control/physical suffix cannot be replay eligible");
}
void validate_episode_rows(const Connection& c) {
    const auto episodes = c.query("SELECT episode_id,physical_rows,complete_control_steps,replay_contract_version,replay_eligible FROM episodes ORDER BY episode_id");
    std::optional<std::int64_t> previous;
    for (const auto& row : episodes) {
        const auto id = io::as_int64(value::require(row, "episode_id"), "episode_id");
        if (previous && (*previous == std::numeric_limits<std::int64_t>::max() || id != *previous+1)) throw std::invalid_argument("episode identities are not consecutive");
        previous = id;
        validate_episode_row(c, row);
    }
}
} // namespace
io::Value::Object validate_database(const Connection& c) {
    const auto actual = validate_container_schema(c);
    validate_declared_catalog(c);
    validate_relationships_and_assets(c);
    validate_episode_rows(c);
    Object counts;
    for (const auto& table : actual) counts.emplace_back(table, value::integer(count(c, "SELECT count(*) FROM " + quote(table))));
    return counts;
}
} // namespace bike::recording
