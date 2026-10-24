#include "database.hpp"
#include "versioned.hpp"
#include "capture.hpp"
#include "value_util.hpp"
#include "../io/hash.hpp"
#include "../platform/files.hpp"
#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <exception>
#include <set>
#include <system_error>
#include <unistd.h>

namespace bike::recording {
namespace {
using io::Value;
using Row = Value::Array;
void insert(Connection const& c, std::string_view table, const Row& row) {
    if (row.empty()) throw std::invalid_argument("recording insert has no columns");
    std::string sql = "INSERT INTO " + quote(table) + " VALUES (";
    for (std::size_t i = 0; i < row.size(); ++i) { if (i) sql += ','; sql += '?'; }
    c.execute(sql + ')', row);
}
void metrics(const Connection& c, std::int64_t id, const Value& summary) {
    const auto* metrics = io::find_member(io::as_object(summary, "summary"), "episode_metrics");
    if (!metrics) return;
    for (const auto& [name, v] : io::as_object(*metrics, "episode_metrics")) {
        Row row{value::integer(id), value::text(name), value::text("null"), Value{}, Value{}, Value{}, Value{}, Value{}, Value{}};
        if (std::holds_alternative<bool>(v.data)) { row[2] = value::text("boolean"); row[5] = v; }
        else if (const auto* n = std::get_if<io::Number>(&v.data)) {
            if (io::is_integer_lexeme(n->lexeme)) { (void)io::as_int64(v, name); row[2] = value::text("bigint"); row[4] = v; }
            else { (void)io::as_double(v, name); row[2] = value::text("double"); row[3] = v; }
        } else if (const auto* tag = std::get_if<io::Tagged>(&v.data); tag && tag->kind == "float64") {
            (void)io::tagged_double(v, name); row[2] = value::text("double"); row[3] = v;
        } else if (std::holds_alternative<std::string>(v.data)) { row[2] = value::text("text"); row[6] = v; }
        else if (!io::is_null(v)) { row[2] = value::text("metadata"); row[8] = value::text(io::recording_json(v)); }
        insert(c, "metrics", row);
    }
}
void episode(Connection& c, const EpisodeEvidence& evidence, std::size_t maximum, const std::function<void(std::string_view)>& before) {
    evidence.validate(); const auto id = evidence.episode_id; const auto& r = evidence.research;
    insert(c, "episodes", {value::integer(id), value::integer(evidence.generation), value::integer(evidence.physical->row_count()),
        value::integer(r ? r->complete_control_steps : 0), r ? value::integer(r->replay_contract_version) : Value{}, Value{r && r->replay_eligible}, value::text(io::recording_json(evidence.summary))});
    const auto* catalog = evidence.physical->catalog();
    const auto version = catalog ? catalog_version(*catalog) : CatalogVersion::planar1;
    if (!catalog && evidence.physical->row_count()) throw std::invalid_argument("nonempty physical evidence must own its coordinate catalog");
    if (catalog) {
        auto const coordinates = [&](const std::vector<Coordinate>& sequence) { for (const auto& coordinate : sequence) insert(c, "coordinate_catalog", {
            value::integer(id), value::text(coordinate.kind), value::integer(coordinate.index), value::text(coordinate.joint),
            value::text(coordinate.component), value::text(coordinate.dof_kind), value::text(coordinate.unit)}); };
        coordinates(catalog->coordinates().qpos); coordinates(catalog->coordinates().dof);
        for (const auto& alias : catalog->aliases()) insert(c, "alias_catalog", {value::integer(id), value::text(alias.alias), value::text(alias.canonical),
            alias.index ? value::integer(*alias.index) : Value{}, alias.parameter ? value::text(*alias.parameter) : Value{}, value::text(alias.note)});
    }
    for (const auto& a : evidence.assets) {
        if (io::sha256(a.content) != a.sha256) throw std::invalid_argument("asset checksum mismatch: " + a.name);
        insert(c, "assets", {value::integer(id), value::text(a.name), value::text(a.media_type), value::text(a.sha256), Value{io::Bytes{a.content}}});
    }
    const auto physical_batch = [&](std::string_view name, runtime::EncodedTable&& batch) {  // NOLINT(cppcoreguidelines-rvalue-reference-param-not-moved) callback signature is fixed by EncodedTables::tables; the batch is consumed by append_pieces, not moved
        const auto& spec = table_spec(name, version);
        const bool control = name == "commands_requested" || name == "commands_applied" || name == "observations" || name == "transitions";
        if (control && !(name == "commands_applied" && !r && evidence.physical->ordinary_commands())) throw std::invalid_argument("physical source emitted research table " + std::string(name));
        if (batch.rows < 0 || std::cmp_greater(batch.rows, maximum)) throw std::invalid_argument("physical source exceeded batch_rows bound");
        if (before) before("append");
        append_pieces(c, spec, batch, id, before);
    };
    evidence.physical->tables(maximum, physical_batch);
    if (r) {
        r->history->tables(maximum, [&](std::string_view name, runtime::EncodedTable&& batch) {  // NOLINT(cppcoreguidelines-rvalue-reference-param-not-moved) callback signature is fixed by EncodedTables::tables; the batch is consumed by append_pieces, not moved
            if (name != "commands_requested" && name != "commands_applied" && name != "observations" && name != "transitions") throw std::invalid_argument("research source emitted physical table");
            if (batch.rows < 0 || std::cmp_greater(batch.rows, maximum)) throw std::invalid_argument("research source exceeded batch_rows bound");
            if (before) before("append");
            append_pieces(c, table_spec(name, version), batch, id, before);
        });
        insert(c, "integration_states", {value::integer(id), value::text("initial"), value::text("mjSTATE_INTEGRATION"), value::doubles(r->initial_integration_state)});
        insert(c, "integration_states", {value::integer(id), value::text("final"), value::text("mjSTATE_INTEGRATION"), value::doubles(r->final_integration_state)});
    }
    metrics(c, id, evidence.summary);
}
std::filesystem::path temporary_directory(const std::filesystem::path& parent) {
    auto text = (parent / ".recording-db-XXXXXX").string(); std::vector<char> bytes(text.begin(), text.end()); bytes.push_back('\0');
    if (!::mkdtemp(bytes.data())) throw std::system_error(errno, std::generic_category(), "exclusive recording database staging");
    return {bytes.data()};
}
} // namespace
io::Value::Object write_database(const std::filesystem::path& file, const RunCapture& capture, std::size_t maximum,
                                 const std::function<void(std::string_view)>& before) {
    if (!capture.enabled()) throw std::invalid_argument("disabled capture cannot write a database");
    if (!maximum) throw std::invalid_argument("batch_rows must be positive");
    const auto version = capture_version(capture);
    const auto parent = file.has_parent_path() ? file.parent_path() : std::filesystem::path(".");
    const auto owned = temporary_directory(parent), payload = owned / database_name;
    Value::Object counts;
    // DuckDB itself has no EXCL-open option. Create inside an owned private
    // directory, then hard-link the CHECKPOINTed, closed file exclusively.
    // An existing output (even an empty file) is never opened by the engine.
    {
        if (before) before("database_open");
        Database const database(payload, false); auto& c = database.connection();
        bool committed = false;
        try {
            c.execute("BEGIN TRANSACTION");
            if (before) before("schema_write");
            create_metadata(c);
            create_streams(c, version);
            const auto& metadata = capture.metadata();
            if (before) before("metadata_write");
            insert(c, "recording_meta", {value::integer(static_cast<int>(version)), value::text(metadata.run_uuid), value::text("bike_recording.json.v1")});
            insert(c, "runs", {value::text(metadata.run_uuid), value::text(metadata.started_utc), value::text(metadata.backend), value::text(metadata.mode), value::text(io::recording_json(metadata.values))});
            if (before) before("catalog_write");
            for (const auto& row : catalog_rows(version)) insert(c, "field_catalog", row);
            for (const auto& evidence : capture.episodes()) episode(c, *evidence, maximum, before);
            for (const auto& e : capture.events()) insert(c, "events", {value::integer(e.episode_id), value::integer(e.event_id), value::text(e.kind), value::real(e.received_wall_s), value::integer(e.processed_step), e.applied_step ? value::integer(*e.applied_step) : Value{}, value::text(io::recording_json(e.values))});
            if (before) before("views_write");
            create_views(c, version);
            create_legacy_view(c);
            if (before) before("validate_database");
            counts = validate_database(c);
            if (before) before("commit");
            c.execute("COMMIT");
            committed = true;
            if (before) before("checkpoint");
            c.execute("CHECKPOINT");
            if (before) before("database_close");
        } catch (...) {
            const auto primary = std::current_exception();
            if (!committed) { try { c.execute("ROLLBACK"); } catch (const std::exception& cleanup) {
                try { std::rethrow_exception(primary); } catch (const std::exception& error) {
                    throw std::runtime_error(std::string(error.what()) + "; rollback also failed: " + cleanup.what() + "; unsealed database staging: " + owned.string());
                }
            } }
            std::rethrow_exception(primary);
        }
    }
    if (std::filesystem::exists(payload.string() + ".wal")) throw std::runtime_error("recording database retains a WAL after close");
    auto opened = platform::open_regular(payload); platform::sync(opened); opened.close();
    if (before) before("database_link");
    platform::link_new(payload, file);
    platform::sync_directory(parent);
    std::error_code cleanup; std::filesystem::remove(payload, cleanup); std::filesystem::remove(owned, cleanup);
    return counts;
}
void write_database(const std::filesystem::path& file, const RunCapture& capture) { (void)write_database(file, capture, batch_rows); }
void validate_database(const std::filesystem::path& file) { Database const database(file, true); (void)validate_database(database.connection()); }
} // namespace bike::recording
