#include "catalog.hpp"
#include "versioned.hpp"
#include "duckdb.hpp"
#include "value_util.hpp"
#include <algorithm>
#include <map>
#include <set>
#include <array>
#include <format>

namespace bike::recording {
namespace {
// Preserve the schema2 metadata on every nested struct member, not only the
// outer database column. describe_field remains the schema1 serialization.
// NOLINTNEXTLINE(misc-no-recursion) follows the finite catalog field tree
io::Value describe_spatial_field(const FieldSpec& field) {
    auto definition = describe_field(field);
    auto& object = std::get<io::Value::Object>(definition.data);
    value::set(object, "name", value::text(field.key));
    value::set(object, "type", value::text(field.dtype));
    value::set(object, "classification", value::text(field.measurement == "measured" ? "measurement" : "truth"));
    value::set(object, "nullable", io::Value{field.optional});
    value::set(object, "presence", value::text("explicit"));
    io::Value::Array children;
    for (const auto& child : field.children) children.push_back(describe_spatial_field(child));
    value::set(object, "children", io::Value{std::move(children)});
    return definition;
}
} // namespace
std::string quote(std::string_view name) {
    if (name.empty() || name.find('\0') != std::string_view::npos) throw std::invalid_argument("invalid SQL identifier");
    std::string out = "\"";
    for (char const c : name) { out += c; if (c == '"') out += c; }
    return out + '"';
}
std::vector<std::vector<std::string>> parent_paths(const TableSpec& spec) {
    std::set<std::vector<std::string>> parents;
    for (const auto& f : spec.fields) for (std::size_t depth = 1; depth < f.path.size(); ++depth)
        parents.emplace(f.path.begin(), f.path.begin() + static_cast<std::ptrdiff_t>(depth));
    for (const auto& root : spec.claims) parents.insert({root});
    return {parents.begin(), parents.end()};
}
std::vector<std::string> parent_masks(const TableSpec& spec) {
    std::vector<std::string> out;
    for (const auto& path : parent_paths(spec)) for (auto suffix : {"_present", "_valid"}) out.push_back("__" + value::join(path) + suffix);
    return out;
}
std::vector<std::pair<std::string, std::string>> column_types(const TableSpec& spec) {
    std::vector<std::pair<std::string, std::string>> out;
    auto const add = [&](const FieldSpec& f) { out.emplace_back(f.key, f.duckdb_type()); if (f.optional) out.emplace_back(f.key + "_present", "BOOLEAN"); };
    for (const auto& f : spec.keys) add(f);
    for (const auto& f : spec.fields) add(f);
    for (const auto& mask : parent_masks(spec)) out.emplace_back(mask, "BOOLEAN");
    return out;
}
std::vector<io::Value::Array> catalog_rows() { return catalog_rows(CatalogVersion::planar1); }
std::vector<io::Value::Array> catalog_rows(CatalogVersion version) {
    std::vector<io::Value::Array> rows;
    for (const auto& spec : all_tables(version)) {
        auto const add = [&](const FieldSpec& f) {
            const auto definition = version == CatalogVersion::spatial2 ? describe_spatial_field(f) : describe_field(f);
            rows.push_back({value::text(spec.name), value::text(f.key), value::text(f.duckdb_type()), value::text(f.unit), value::text(f.phase), io::Value{f.optional}, value::text(f.measurement), value::text(f.source), value::text(f.frame), value::text(io::recording_json(definition))});
            if (f.optional) rows.push_back({value::text(spec.name), value::text(f.key + "_present"), value::text("BOOLEAN"), value::text("1"), value::text(f.phase), io::Value{false}, value::text("declared"), value::text(f.source), value::text("n/a"), value::text(io::recording_json(io::Value{io::Value::Object{{"presence_of", value::text(f.key)}, {"description", value::text("field existed in source payload")}}}))});
        };
        for (const auto& f : spec.keys) add(f);
        for (const auto& f : spec.fields) add(f);
        for (const auto& mask : parent_masks(spec)) rows.push_back({value::text(spec.name), value::text(mask), value::text("BOOLEAN"), value::text("1"), value::text("interval"), io::Value{false}, value::text("declared"), value::text(spec.source), value::text("n/a"), value::text(io::recording_json(io::Value{io::Value::Object{{"parent_mask", value::text(mask)}}}))});
    }
    return rows;
}
void create_metadata(Connection const& connection) {
    static const auto definitions = std::to_array<std::pair<std::string_view, std::string_view>>({
        {"recording_meta", "schema_version INTEGER NOT NULL, run_id UUID NOT NULL, codec VARCHAR NOT NULL"},
        {"runs", "run_id UUID PRIMARY KEY, started_at TIMESTAMPTZ NOT NULL, backend VARCHAR NOT NULL, mode VARCHAR NOT NULL, metadata_json VARCHAR NOT NULL"},
        {"episodes", "episode_id BIGINT PRIMARY KEY, generation BIGINT NOT NULL, physical_rows BIGINT NOT NULL, complete_control_steps BIGINT NOT NULL, replay_contract_version INTEGER, replay_eligible BOOLEAN NOT NULL, summary_json VARCHAR NOT NULL"},
        {"events", "episode_id BIGINT NOT NULL, event_id BIGINT NOT NULL, kind VARCHAR NOT NULL, received_wall_s DOUBLE NOT NULL, processed_step BIGINT NOT NULL, applied_step BIGINT, values_json VARCHAR NOT NULL, PRIMARY KEY(episode_id,event_id)"},
        {"field_catalog", "table_name VARCHAR NOT NULL, field_name VARCHAR NOT NULL, dtype VARCHAR NOT NULL, unit VARCHAR NOT NULL, phase VARCHAR NOT NULL, optional BOOLEAN NOT NULL, measurement VARCHAR NOT NULL, source VARCHAR NOT NULL, frame VARCHAR NOT NULL, definition_json VARCHAR NOT NULL, PRIMARY KEY(table_name,field_name)"},
        {"coordinate_catalog", "episode_id BIGINT NOT NULL, kind VARCHAR NOT NULL, coordinate_index BIGINT NOT NULL, joint VARCHAR NOT NULL, component VARCHAR NOT NULL, dof_kind VARCHAR NOT NULL, unit VARCHAR NOT NULL, PRIMARY KEY(episode_id,kind,coordinate_index)"},
        {"alias_catalog", "episode_id BIGINT NOT NULL, alias VARCHAR NOT NULL, canonical VARCHAR NOT NULL, vector_index BIGINT, parameter VARCHAR, note VARCHAR NOT NULL, PRIMARY KEY(episode_id,alias)"},
        {"assets", "episode_id BIGINT NOT NULL, name VARCHAR NOT NULL, media_type VARCHAR NOT NULL, sha256 VARCHAR NOT NULL, content BLOB NOT NULL, PRIMARY KEY(episode_id,name)"},
        {"integration_states", "episode_id BIGINT NOT NULL, phase VARCHAR NOT NULL, signature VARCHAR NOT NULL, state DOUBLE[] NOT NULL, PRIMARY KEY(episode_id,phase)"},
        {"metrics", "episode_id BIGINT NOT NULL, name VARCHAR NOT NULL, kind VARCHAR NOT NULL, double_value DOUBLE, bigint_value BIGINT, boolean_value BOOLEAN, text_value VARCHAR, list_value DOUBLE[], json_value VARCHAR, PRIMARY KEY(episode_id,name)"}});
    for (const auto& [name, definition] : definitions) connection.execute("CREATE TABLE " + quote(name) + " (" + std::string(definition) + ")");
}
void create_streams(Connection const& connection) { create_streams(connection, CatalogVersion::planar1); }
void create_streams(Connection const& connection, CatalogVersion version) {
    for (const auto& spec : all_tables(version)) {
        std::string sql = "CREATE TABLE " + quote(spec.name) + " (";
        bool first = true;
        for (const auto& [name, dtype] : column_types(spec)) { if (!first) sql += ", "; first = false; sql += quote(name) + " " + dtype; }
        sql += ", PRIMARY KEY ("; first = true;
        for (const auto& key : spec.keys) { if (!first) sql += ", "; first = false; sql += quote(key.key); }
        connection.execute(sql + "))");
    }
}
void create_views(Connection const& connection) { create_views(connection, CatalogVersion::planar1); }
void create_views(Connection const& connection, CatalogVersion version) {
    std::string select = "p.episode_id, p.interval_id", joins;
    std::size_t index = 0;
    for (const auto& spec : schema_catalog(version).table_order()) {
        const auto current = index++;
        if (spec.name == "forces") continue;
        const std::string alias = spec.name == "physics" ? "p" : "c" + std::to_string(current);
        if (alias != "p") joins += " LEFT JOIN " + quote(spec.name) + " " + alias + " USING (episode_id, interval_id)";
        for (const auto& [name, type] : column_types(spec)) { (void)type; select += ", " + alias + "." + quote(name) + " AS " + quote(spec.name + "." + name); }
    }
    connection.execute("CREATE VIEW telemetry AS SELECT " + select + " FROM physics p" + joins);
    connection.execute("CREATE VIEW legacy_intervals AS SELECT episode_id, interval_id, time_s, interval_end_s AS end_time_s, dt_s, qpos, qvel FROM physics");
    connection.execute("CREATE VIEW legacy_observations AS SELECT * EXCLUDE(episode_id, observation_id) FROM observations");
}
void create_legacy_view(Connection const& connection) {
    std::map<std::string, std::vector<std::string>, std::less<>> expressions;
    for (const auto& row : connection.query("SELECT episode_id,alias,canonical,vector_index,parameter FROM alias_catalog ORDER BY alias,episode_id")) {
        const auto& alias = io::as_string(value::require(row, "alias"), "alias");
        const auto& canonical = io::as_string(value::require(row, "canonical"), "canonical");
        if (alias.find('{') != std::string::npos || !io::is_null(value::require(row, "parameter")) || canonical.starts_with("forces.") || std::count(canonical.begin(), canonical.end(), '.') != 1) continue;
        std::string expression = quote(canonical);
        const auto& index = value::require(row, "vector_index");
        if (!io::is_null(index)) expression = std::format("({})[{}]", expression, io::as_int64(index, "vector_index") + 1);
        expressions[alias].push_back(std::format("WHEN episode_id={} THEN {}", io::as_int64(value::require(row, "episode_id"), "episode_id"), expression));
    }
    std::string columns = "episode_id, interval_id";
    for (const auto& [name, cases] : expressions) columns += ", CASE " + value::join(cases, " ") + " END AS " + quote(name);
    connection.execute("CREATE VIEW legacy_telemetry AS SELECT " + columns + " FROM telemetry");
}
} // namespace bike::recording
