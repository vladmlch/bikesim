#include "reader.hpp"
#include "analysis_detail.hpp"
#include "database.hpp"
#include "../io/hash.hpp"
#include "../platform/files.hpp"
#include <array>
#include <iterator>
#include <regex>
#include <set>

namespace bike::recording {
RecordingError::~RecordingError() = default;
namespace {
using namespace detail;
void exact_keys(const Object& object, const std::set<std::string>& expected, std::string_view error) {
    std::set<std::string> actual;
    for (const auto& [key, ignored] : object) { (void)ignored; actual.insert(key); }
    if (actual != expected || object.size() != actual.size()) throw RecordingError(std::string(error));
}
void hash_payloads(const std::filesystem::path& directory, const Value& hashes) {
    for (const auto& [name, digest] : io::as_object(hashes, "file_sha256")) {
        const auto hash = io::as_string(digest, "file_sha256." + name);
        if (!std::regex_match(hash, std::regex("[0-9a-f]{64}"))) throw RecordingError("invalid checksum: " + name);
        if (platform::file_sha256(directory / name) != hash) throw RecordingError("recording checksum mismatch: " + name);
    }
}
} // namespace

io::Value read_manifest(const std::filesystem::path& directory) {
    Value manifest;
    try { manifest = io::parse_recording_json(read_text(directory / "manifest.json")); }
    catch (const std::exception& error) {
        throw RecordingError("recording is unsealed: a regular valid manifest.json is required: " + std::string(error.what()));
    }
    const auto& object = io::as_object(manifest, "manifest");
    exact_keys(object, {"schema_version", "sealed", "run_id", "database", "file_sha256", "row_counts", "report_status", "report_error"},
        "unsupported recording manifest/container schema");
    if (count(field(object, "schema_version")) != 1 || !io::as_bool(field(object, "sealed"), "manifest.sealed") ||
        string(field(object, "database")) != "run.duckdb") throw RecordingError("unsupported recording manifest/container schema");
    if (!std::regex_match(string(field(object, "run_id")), std::regex("[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}")))
        throw RecordingError("invalid recording run UUID");
    const auto status = string(field(object, "report_status"));
    if (status != "complete" && status != "failed") throw RecordingError("invalid report status");
    const auto& hashes = field(object, "file_sha256");
    exact_keys(io::as_object(hashes, "file_sha256"), status == "complete" ? std::set<std::string>{"run.duckdb", "summary.md"} :
        std::set<std::string>{"run.duckdb"}, "manifest must cover exactly its fixed payload files");
    const auto& error = field(object, "report_error");
    if (status == "complete" && !io::is_null(error)) throw RecordingError("successful report cannot have an error");
    if (status == "failed" && (io::is_null(error) || string(error).empty())) throw RecordingError("failed report must describe its error");
    for (const auto& [name, value] : io::as_object(field(object, "row_counts"), "row_counts")) {
        if (name.empty() || count(value) < 0) throw RecordingError("invalid manifest row counts");
    }
    hash_payloads(directory, hashes);
    return manifest;
}

Recording::Recording(std::filesystem::path path) : path_(std::move(path)), manifest_(read_manifest(path_)),
    database_(std::make_unique<Database>(path_ / "run.duckdb", true)) {
    const auto expected_database = string(field(field(manifest_, "file_sha256"), "run.duckdb"));
    if (database_->opened_sha256() != expected_database) throw RecordingError("opened database checksum differs from manifest");
    const auto counts = validate_database(database_->connection());
    const auto identity = rows("SELECT run_id FROM runs");
    if (identity.size() != 1 || !equal(field(identity.front(), "run_id"), field(manifest_, "run_id")) ||
        !equal(Value{counts}, field(manifest_, "row_counts")))
        throw RecordingError("manifest and database identity/extents disagree");
    auto assets = stream("SELECT name,sha256,content FROM assets ORDER BY episode_id,name");
    for (;;) {
        auto const batch = assets.next_batch(64);
        if (batch.empty()) break;
        for (const auto& row : batch) {
            const auto* bytes = std::get_if<io::Bytes>(&field(row, "content").data);
            if (!bytes || io::sha256(bytes->value) != string(field(row, "sha256")))
                throw RecordingError("bundled asset checksum mismatch: " + string(field(row, "name")));
        }
    }
    // Close the validate/open race: a leaf replacement or changed payload
    // cannot become a silently accepted recording after the initial hash.
    hash_payloads(path_, field(manifest_, "file_sha256"));
    if (database_->opened_sha256() != expected_database) throw RecordingError("opened database changed during validation");
}
Recording::~Recording() = default;
void Recording::close() noexcept { database_.reset(); }
std::vector<io::Value::Object> Recording::rows(std::string_view sql, std::span<const io::Value> parameters) const {
    auto cursor=stream(sql,parameters);
    std::vector<Object> rows;
    for(;;) {
        auto batch=cursor.next_batch();if(batch.empty())break;
        rows.insert(rows.end(),std::make_move_iterator(batch.begin()),std::make_move_iterator(batch.end()));
    }
    return rows;
}
Cursor Recording::stream(std::string_view sql, std::span<const io::Value> parameters) const {
    if(!database_)throw RecordingError("recording reader is closed");
    return database_->connection().stream(sql, parameters, true);
}
io::Value Recording::metadata() const {
    auto result = rows("SELECT * FROM runs");
    auto row = std::move(result.at(0));
    set(row, "values", io::parse_recording_json(string(pop(row, "metadata_json"))));
    return Value{std::move(row)};
}
std::vector<std::int64_t> Recording::episode_ids() const {
    std::vector<std::int64_t> out;
    for (const auto& row : rows("SELECT episode_id FROM episodes ORDER BY episode_id")) out.push_back(count(field(row, "episode_id")));
    return out;
}
io::Value Recording::episode(std::optional<std::int64_t> episode_id, bool research) const {
    if (episode_id && *episode_id < 0) throw RecordingError("episode must be a nonnegative integer");
    std::vector<Object> selected;
    if (episode_id) {
        const std::array params{integer(*episode_id)};
        selected = rows("SELECT * FROM episodes WHERE episode_id=?", params);
        if (selected.empty()) throw RecordingError("no episode " + std::to_string(*episode_id));
    } else {
        selected = rows("SELECT * FROM episodes ORDER BY episode_id LIMIT 2");
        if (selected.size() != 1) throw RecordingError("recording has multiple or no episodes; select --episode explicitly");
    }
    if (selected.size() != 1) throw RecordingError("duplicate recording episode");
    if (research && (io::is_null(field(selected[0], "replay_contract_version")) || count(field(selected[0], "replay_contract_version")) != 3))
        throw RecordingError("ordinary physical recordings are not checked research replay recipes");
    return Value{std::move(selected[0])};
}
std::unique_ptr<Recording> open_recording(const std::filesystem::path& directory) { return std::make_unique<Recording>(directory); }
} // namespace bike::recording
