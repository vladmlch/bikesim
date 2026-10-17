#include "export.hpp"
#include "input_detail.hpp"
#include "../platform/files.hpp"
#include "../research/evidence.hpp"
#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdlib>
#include <limits>
#include <system_error>

namespace bike::replay {
namespace {
using namespace detail;
constexpr std::array fixed_files{"summary.json", "commands_requested.jsonl", "commands_applied.jsonl", "observations.jsonl",
    "transitions.jsonl", "states.npz", "terrain_vertices.npy", "track.toml", "trace.csv", "episode_metrics.json", "telemetry.csv", "intervals.jsonl"};
void plain(const Value& value) {
    if (std::holds_alternative<io::Tagged>(value.data) || std::holds_alternative<io::Bytes>(value.data))
        throw recording::RecordingError("legacy JSON cannot represent tagged/nonfinite/byte evidence; use the lossless recording export");
    if (const auto* rows = std::get_if<Array>(&value.data)) for (const auto& item : *rows) plain(item);
    else if (const auto* object = std::get_if<Object>(&value.data)) for (const auto& [key, item] : *object) { (void)key; plain(item); }
}
// Python json.dumps(sort_keys=True, ensure_ascii=True, allow_nan=False)
// spelling. Legacy command hashes retain their historical spaced separators.
std::string json(const Value& value, bool pretty = false) {
    plain(value); const auto compact = io::canonical_json(value);
    constexpr std::string_view hex = "0123456789abcdef";
    std::string escaped_ascii;
    const auto unicode = [&](std::uint32_t code) {
        escaped_ascii += "\\u";
        for (int shift = 12; shift >= 0; shift -= 4) escaped_ascii += hex[(code >> static_cast<unsigned>(shift)) & 15U];
    };
    for (std::size_t i = 0; i < compact.size(); ++i) {
        const auto first = static_cast<unsigned char>(compact[i]);
        if (first < 128) { escaped_ascii += compact[i]; continue; }
        unsigned remaining = first < 224 ? 1U : first < 240 ? 2U : 3U;
        std::uint32_t code = first & ((1U << (6U - remaining)) - 1U);
        for (unsigned j = 0; j < remaining; ++j) code = (code << 6U) | (static_cast<unsigned char>(compact[++i]) & 63U);
        if (code < 0x10000U) unicode(code);
        else { code -= 0x10000U; unicode(0xd800U + (code >> 10U)); unicode(0xdc00U + (code & 1023U)); }
    }
    std::string output; std::size_t depth = 0; bool quoted = false, escaped = false;
    for (std::size_t i = 0; i < escaped_ascii.size(); ++i) {
        const char c = escaped_ascii[i];
        if (quoted) {
            output += c;
            if (escaped) escaped = false;
            else if (c == '\\') escaped = true;
            else if (c == '"') quoted = false;
        } else if (c == '"') { quoted = true; output += c; }
        else if (c == '{' || c == '[') {
            output += c; ++depth;
            if (pretty && i + 1 < escaped_ascii.size() && escaped_ascii[i+1] != '}' && escaped_ascii[i+1] != ']') output += '\n' + std::string(2 * depth, ' ');
        } else if (c == '}' || c == ']') {
            --depth;
            if (pretty && i && escaped_ascii[i-1] != '{' && escaped_ascii[i-1] != '[') output += '\n' + std::string(2 * depth, ' ');
            output += c;
        } else if (c == ',') output += pretty ? ",\n" + std::string(2 * depth, ' ') : ", ";
        else if (c == ':') output += ": ";
        else output += c;
    }
    return output;
}
void write(const std::filesystem::path& path, const std::string& source) { platform::write_exclusive(path, std::as_bytes(std::span(source.data(), source.size()))); }
void lines(const std::filesystem::path& path, const std::vector<Value>& rows) {
    auto file = platform::open_exclusive(path);
    for (const auto& row : rows) {
        const auto source = json(row) + '\n'; platform::write_all(file, std::as_bytes(std::span(source.data(), source.size())));
    }
    platform::sync(file); file.close();
}
std::string csv_row(const std::vector<std::string>& cells) {
    std::string out;
    for (std::size_t i = 0; i < cells.size(); ++i) {
        if (i) out += ',';
        const auto& cell = cells[i]; const bool quote = cell.find_first_of(",\"\r\n") != std::string::npos || (cells.size() == 1 && cell.empty());
        if (quote) out += '"';
        for (char c : cell) { out += c; if (c == '"') out += '"'; }
        if (quote) out += '"';
    }
    return out + "\r\n";
}
std::string csv_scalar(const Value& value) {
    if (io::is_null(value)) return {};
    if (const auto* b = std::get_if<bool>(&value.data)) return *b ? "True" : "False";
    if (const auto* n = std::get_if<io::Number>(&value.data)) return n->lexeme;
    if (const auto* s = std::get_if<std::string>(&value.data)) return *s;
    throw recording::RecordingError("legacy trace contains a non-scalar field");
}
void trace(const std::filesystem::path& path, const std::vector<Value>& transitions) {
    auto file = platform::open_exclusive(path); std::vector<std::string> names;
    for (const auto& transition : transitions) {
        Object row;
        const auto& observation = field(transition, "observation");
        const auto acceleration = double_vector(field(observation, "specific_force_body_mps2"), "specific force");
        if (acceleration.size() != 3) throw recording::RecordingError("legacy trace acceleration must have width 3");
        for (const auto& [name, item] : io::as_object(observation, "observation"))
            if (name != "specific_force_body_mps2") row.emplace_back("sensor_" + name, item);
        constexpr std::string_view axes="xyz";
        for (std::size_t i = 0; i < 3; ++i) row.emplace_back("sensor_specific_force_" + std::string(1, axes[i]) + "_mps2", real(acceleration[i]));
        if (const auto* truth = find(transition, "truth"); truth && !io::is_null(*truth))
            for (const auto& [name, item] : io::as_object(*truth, "truth")) row.emplace_back("truth_" + name, item);
        for (auto name : {"contact_state", "demand_nm", "terminated", "truncated", "reason", "numerically_valid"}) row.emplace_back(name, field(transition, name));
        if (names.empty()) {
            for (const auto& [name, item] : row) { (void)item; names.push_back(name); }
            const auto header = csv_row(names); platform::write_all(file, std::as_bytes(std::span(header.data(), header.size())));
        }
        for (const auto& [name, item] : row) { (void)item; if (std::ranges::find(names, name) == names.end()) throw recording::RecordingError("legacy trace field set changed after first row"); }
        std::vector<std::string> cells;
        for (const auto& name : names) { const auto* item = find(row, name); cells.push_back(item ? csv_scalar(*item) : ""); }
        const auto line = csv_row(cells); platform::write_all(file, std::as_bytes(std::span(line.data(), line.size())));
    }
    if (names.empty()) {
        const auto header = csv_row({"sensor_time_s"}); platform::write_all(file, std::as_bytes(std::span(header.data(), header.size())));
    }
    platform::sync(file); file.close();
}
void telemetry(const std::filesystem::path& path, const runtime::SampleColumns& columns) {
    std::vector<std::string> names;
    const std::size_t count = columns.empty() ? 0 : columns.begin()->second.size();
    for (const auto& [name, values] : columns) {
        if (values.size() != count) throw recording::RecordingError("legacy telemetry column lengths disagree");
        names.push_back(name);
    }
    if (names.empty()) names = {"schema_version", "time_s"};
    auto file = platform::open_exclusive(path);
    auto source = csv_row(names); platform::write_all(file, std::as_bytes(std::span(source.data(), source.size())));
    for (std::size_t i = 0; i < count; ++i) {
        std::vector<std::string> row;
        for (const auto& name : names) {
            const auto value = columns.at(name)[i];
            row.push_back(std::isnan(value) ? "" : std::isinf(value) ? (value < 0. ? "-inf" : "inf") : csv_scalar(real(value)));
        }
        source = csv_row(row); platform::write_all(file, std::as_bytes(std::span(source.data(), source.size())));
    }
    platform::sync(file); file.close();
}

void le(std::vector<std::byte>& output, std::uint32_t value, unsigned bytes) {
    for (unsigned i = 0; i < bytes; ++i) { output.push_back(static_cast<std::byte>(value & 255U)); value >>= 8U; }
}
std::uint32_t crc(std::span<const std::byte> bytes) {
    std::uint32_t value = 0xffffffffU;
    for (const auto byte : bytes) {
        value ^= std::to_integer<std::uint8_t>(byte);
        for (unsigned bit = 0; bit < 8; ++bit) value = (value >> 1U) ^ ((value & 1U) ? 0xedb88320U : 0U);
    }
    return value ^ 0xffffffffU;
}
std::vector<std::byte> states(const ReplayInput& input) {
    if (input.initial_state.empty() || input.initial_state.size() != input.final_state.size()) throw recording::RecordingError("legacy initial/final integration state dimensions disagree");
    struct Member { std::string name; std::vector<std::byte> bytes; std::uint32_t crc = 0, offset = 0; };
    std::array members{Member{"initial.npy", io::encode_npy(io::double_array(input.initial_state, {input.initial_state.size()}))},
        Member{"final.npy", io::encode_npy(io::double_array(input.final_state, {input.final_state.size()}))}};
    std::vector<std::byte> output;
    // A standards-compliant stored ZIP: no compression/provider dependency.
    // The .npy member bytes, shape, endian and values are the source contract;
    // checksums always identify the archive actually written.
    for (auto& member : members) {
        if (member.bytes.size() > io::kMaxNumpyBytes || output.size() > io::kMaxNumpyBytes - member.bytes.size()) throw recording::RecordingError("legacy state archive exceeds byte limit");
        member.offset = static_cast<std::uint32_t>(output.size()); member.crc = crc(member.bytes);
        le(output, 0x04034b50U, 4); le(output, 20, 2); le(output, 0, 2); le(output, 0, 2); le(output, 0, 2); le(output, 33, 2);
        le(output, member.crc, 4); le(output, static_cast<std::uint32_t>(member.bytes.size()), 4); le(output, static_cast<std::uint32_t>(member.bytes.size()), 4);
        le(output, static_cast<std::uint32_t>(member.name.size()), 2); le(output, 0, 2);
        for (char c : member.name) output.push_back(static_cast<std::byte>(c));
        output.insert(output.end(), member.bytes.begin(), member.bytes.end());
    }
    const auto central = static_cast<std::uint32_t>(output.size());
    for (const auto& member : members) {
        le(output, 0x02014b50U, 4); le(output, 20, 2); le(output, 20, 2); le(output, 0, 2); le(output, 0, 2); le(output, 0, 2); le(output, 33, 2);
        le(output, member.crc, 4); le(output, static_cast<std::uint32_t>(member.bytes.size()), 4); le(output, static_cast<std::uint32_t>(member.bytes.size()), 4);
        le(output, static_cast<std::uint32_t>(member.name.size()), 2); le(output, 0, 2); le(output, 0, 2); le(output, 0, 2); le(output, 0, 2); le(output, 0, 4); le(output, member.offset, 4);
        for (char c : member.name) output.push_back(static_cast<std::byte>(c));
    }
    const auto central_size = static_cast<std::uint32_t>(output.size()) - central;
    le(output, 0x06054b50U, 4); le(output, 0, 2); le(output, 0, 2); le(output, 2, 2); le(output, 2, 2); le(output, central_size, 4); le(output, central, 4); le(output, 0, 2);
    return output;
}
class Staging {
public:
    explicit Staging(const std::filesystem::path& parent) {
        auto pattern = (parent / ".research-export-XXXXXX").string(); std::vector<char> name(pattern.begin(), pattern.end()); name.push_back('\0');
        const auto* result = ::mkdtemp(name.data()); if (!result) throw std::system_error(errno, std::generic_category(), "create legacy export staging"); path = result;
    }
    ~Staging() { std::error_code ignored; std::filesystem::remove_all(path, ignored); }
    std::filesystem::path path;
};
void destination_check(const std::filesystem::path& destination, bool overwrite) {
    const auto status = std::filesystem::symlink_status(destination);
    if (std::filesystem::is_symlink(status) || (std::filesystem::exists(status) && !std::filesystem::is_directory(status)))
        throw recording::RecordingError("legacy output must be a real directory");
    if (std::filesystem::exists(status) && !overwrite && !std::filesystem::is_empty(destination))
        throw recording::RecordingError("refusing to overwrite nonempty legacy output");
}
} // namespace

LegacySnapshot capture_legacy(runtime::OwnedResearchEpisode episode) {
    const auto& c = episode.configuration;
    if (c.capture_mode != runtime::ResearchCaptureMode::legacy && c.capture_mode != runtime::ResearchCaptureMode::full)
        throw recording::RecordingError("legacy export requires retained legacy or full capture");
    const auto evaluation = research::wire_value(runtime::Wire{episode.evaluation});
    if (io::as_bool(field(evaluation, "active"), "active") || io::as_bool(field(evaluation, "delivery_pending"), "delivery_pending"))
        throw recording::RecordingError("finish the external control interval before exporting its boundary");
    if (!c.assets || c.record_decimation < 1) throw recording::RecordingError("legacy capture has no immutable input assets or invalid decimation");
    LegacySnapshot output; auto& data = output.data;
    if (!std::holds_alternative<Object>(episode.run_metadata.data)) episode.run_metadata = Value{Object{}};
    set(episode.run_metadata, "identity_checks", research::identity_receipt(c.metadata));
    data.summary = research::summarize(episode); data.metrics = research::episode_metrics(episode);
    data.initial_state = episode.initial_integration_state; data.final_state = episode.final_snapshot.integration_state;
    data.replay_contract_version = 2;
    for (const auto& asset : *c.assets) {
        if (io::sha256(asset.content) != asset.sha256 || !data.assets.emplace(asset.name, asset.content).second)
            throw recording::RecordingError("invalid immutable legacy asset");
    }
    data.manifest = io::parse_recording_json(asset_text(data, "recipe.json"));
    const auto take = [&](std::string_view name, std::vector<Value>& rows) {
        const auto count = episode.row_count(name);
        for (std::size_t first = 0; first < count; first += std::min<std::size_t>(4096, count - first)) {
            const auto batch = episode.batch(name, static_cast<std::int64_t>(first), static_cast<std::int64_t>(std::min<std::size_t>(4096, count - first)));
            for (const auto& row : batch) rows.push_back(research::wire_value(row));
        }
    };
    take("commands_requested", data.requested); take("commands_applied", data.applied);
    take("observations", data.observations); take("transitions", data.transitions); command_shapes(data);
    std::vector<runtime::SamplePtr> retained;
    for (const auto& sample : episode.physical.samples) {
        if (!sample) throw recording::RecordingError("null legacy physical sample");
        if (sample->interval_id % c.record_decimation == 0) { retained.push_back(sample); output.intervals.push_back(research::wire_value(runtime::Wire{sample->as_wire()})); }
    }
    output.columns = runtime::make_columns(retained, episode.physical.columns);
    auto research_metadata = field(data.summary, "research"); const auto commands = json(Value{data.requested});
    set(research_metadata, "commands_sha256", text(io::sha256(std::as_bytes(std::span(commands.data(), commands.size())))));
    set(data.summary, "research", std::move(research_metadata));
    return output;
}
LegacySnapshot capture_legacy(runtime::NativeResearchRuntime& environment) { return capture_legacy(environment.legacy_snapshot()); }

io::Value export_legacy(const LegacySnapshot& snapshot, const std::filesystem::path& destination, bool overwrite) {
    if (destination.empty()) throw std::invalid_argument("legacy output directory is required");
    destination_check(destination, overwrite);
    const auto& data = snapshot.data; const auto& recipe = data.manifest;
    validate_execution(field(recipe, "execution")); command_shapes(data);
    if (!equal(field(field(data.summary, "research"), "execution"), field(recipe, "execution"))) throw recording::RecordingError("legacy summary and recipe execution differ");
    const bool native_executable = find(field(recipe, "execution"), "execution_identity") != nullptr;
    auto summary = data.summary;
    const Value compatibility{Object{{"layout_schema_version", integer(2)},
        {"execution_profile", text(native_executable ? "native_executable" : "historical")},
        {"historical_python_reader_accepts_execution", Value{!native_executable}},
        {"execution_provenance_preserved", Value{true}},
        {"checked_replay_qualified_by_export", Value{false}}}};
    set(summary, "compatibility_export", compatibility);
    const auto parent = destination.parent_path().empty() ? std::filesystem::path(".") : destination.parent_path();
    std::filesystem::create_directories(parent); Staging staging(parent);
    write(staging.path / "summary.json", json(summary, true) + '\n');
    lines(staging.path / "commands_requested.jsonl", data.requested); lines(staging.path / "commands_applied.jsonl", data.applied);
    lines(staging.path / "observations.jsonl", data.observations); lines(staging.path / "transitions.jsonl", data.transitions);
    lines(staging.path / "intervals.jsonl", snapshot.intervals);
    trace(staging.path / "trace.csv", data.transitions); telemetry(staging.path / "telemetry.csv", snapshot.columns);
    write(staging.path / "episode_metrics.json", json(data.metrics, true) + '\n');
    platform::write_exclusive(staging.path / "states.npz", states(data));
    for (auto name : {"terrain_vertices.npy", "track.toml"}) {
        const auto it = data.assets.find(name); if (it == data.assets.end()) throw recording::RecordingError(std::string("missing legacy asset ") + name);
        platform::write_exclusive(staging.path / name, it->second);
    }
    Value manifest{Object{{"schema_version", integer(2)}, {"start_x_m", field(recipe, "start_x_m")}, {"heightfield", field(recipe, "heightfield")},
        {"state_signature", text("mjSTATE_INTEGRATION")}, {"execution", field(recipe, "execution")}}};
    Object hashes;
    for (auto name : fixed_files) hashes.emplace_back(name, text(platform::file_sha256(staging.path / name)));
    for (auto [key, name] : {std::pair{"joint_envelope_file", "rider_joint_envelope.json"}, std::pair{"joint_strength_file", "rider_joint_strength.json"}}) {
        if (const auto* declared = find(recipe, key)) {
            if (string(*declared) != name || !data.assets.contains(name)) throw recording::RecordingError("invalid legacy profile filename");
            platform::write_exclusive(staging.path / name, data.assets.at(name)); set(manifest, key, text(name)); hashes.emplace_back(name, text(platform::file_sha256(staging.path / name)));
        }
    }
    set(manifest, "file_sha256", Value{hashes}); write(staging.path / "replay.json", json(manifest, true) + '\n');
    // Validate the staged bundle with the same strict, hash-checking reader.
    // This loads data only and never initializes a simulator or policy.
    (void)load_legacy_input(staging.path);
    destination_check(destination, overwrite);
    if (!std::filesystem::exists(destination)) platform::create_directory(destination);
    if (overwrite) std::filesystem::remove(destination / "replay.json");
    for (const auto& [name, ignored] : hashes) {
        (void)ignored;
        if (overwrite) std::filesystem::rename(staging.path / name, destination / name);
        else platform::link_new(staging.path / name, destination / name);
    }
    platform::sync_directory(destination);
    platform::link_new(staging.path / "replay.json", destination / "replay.json"); platform::sync_directory(destination); platform::sync_directory(parent);
    return Value{Object{{"output", text(destination.string())}, {"schema_version", integer(2)}, {"compatibility", compatibility}}};
}
io::Value export_legacy(runtime::NativeResearchRuntime& environment, const std::filesystem::path& path, bool overwrite) {
    const auto snapshot = capture_legacy(environment); return export_legacy(snapshot, path, overwrite);
}
} // namespace bike::replay
