#include "input_detail.hpp"
#include "../platform/files.hpp"
#include <array>

namespace bike::replay {
namespace {
using namespace detail;
constexpr std::array files{"summary.json", "commands_requested.jsonl", "commands_applied.jsonl", "observations.jsonl",
    "transitions.jsonl", "states.npz", "terrain_vertices.npy", "track.toml"};
constexpr std::array more_files{"trace.csv", "episode_metrics.json", "telemetry.csv", "intervals.jsonl"};
}
void validate_execution(const io::Value& execution) {
    const auto backend = string(field(execution, "backend"));
    if (find(execution, "execution_identity")) {
        std::set<std::string> expected{"backend", "native_source_sha256", "execution_identity", "mujoco_library_sha256", "numeric_provider",
            "executable_sha256", "source_identity_scope"};
        if (find(execution, "allocation_solver")) { expected.insert("allocation_solver"); expected.insert("allocation_solver_qualification"); }
        if(const auto* threads=find(execution,"tyre_threads")) {
            expected.insert("tyre_threads");
            const auto value=count(*threads);
            if(value<1 || value>64)throw recording::RecordingError("execution.tyre_threads: expected [1,64]");
        }
        exact_keys(execution, expected, "native executable execution");
        if (backend != "native") throw recording::RecordingError("native executable profile requires native backend");
        for (auto key : {"native_source_sha256", "execution_identity", "mujoco_library_sha256", "executable_sha256"}) digest(field(execution, key), key);
        for (auto key : {"numeric_provider", "source_identity_scope"}) if (string(field(execution, key)).empty()) throw recording::RecordingError(std::string(key) + " must be nonempty");
        return;
    }
    exact_keys(execution, {"backend", "runtime_schema", "python_source_sha256", "native_source_sha256", "extension_sha256", "mujoco_version",
        "mujoco_library_sha256", "build_context"}, "schema-2 execution");
    if (backend != "native" && backend != "python") throw recording::RecordingError("unsupported recorded execution backend");
    if (count(field(execution, "runtime_schema")) != 1 || string(field(execution, "mujoco_version")).empty())
        throw recording::RecordingError("unsupported recorded runtime schema/version");
    for (auto key : {"python_source_sha256", "native_source_sha256", "extension_sha256", "mujoco_library_sha256"}) {
        const bool expected_null = backend == "python" && std::string_view(key) != "python_source_sha256";
        if (expected_null) { if (!io::is_null(field(execution, key))) throw recording::RecordingError(std::string("Python execution.") + key + " must be null"); }
        else digest(field(execution, key), key);
    }
    const auto& context = field(execution, "build_context");
    if (backend == "python") { if (!io::is_null(context)) throw recording::RecordingError("Python execution.build_context must be null"); }
    else {
        exact_keys(context, {"build_type", "compiler_id", "compiler_version", "numerical_flags"}, "build_context");
        for (const auto& [name, value] : io::as_object(context, "build_context"))
            if (string(value).find_first_not_of(" \t\r\n") == std::string::npos) throw recording::RecordingError("empty build_context." + name);
    }
}
ReplayInput load_legacy_input(const std::filesystem::path& path, std::optional<std::int64_t> episode) {
    if (episode && *episode != 0) throw recording::RecordingError("legacy directories contain only episode 0");
    ReplayInput out;
    out.manifest = io::parse_json(read_text(path / "replay.json"));
    const auto version = count(field(out.manifest, "schema_version"));
    if (version != 1 && version != 2) throw recording::RecordingError("unsupported replay schema");
    out.replay_contract_version = static_cast<int>(version);
    std::set<std::string> expected{"schema_version", "start_x_m", "heightfield", "state_signature", "file_sha256"};
    if (version == 2) expected.insert("execution");
    if (find(out.manifest, "joint_envelope_file")) expected.insert("joint_envelope_file");
    if (version == 2 && find(out.manifest, "joint_strength_file")) expected.insert("joint_strength_file");
    exact_keys(out.manifest, expected, "replay manifest");
    if (string(field(out.manifest, "state_signature")) != "mjSTATE_INTEGRATION") throw recording::RecordingError("unsupported replay state signature");
    (void)io::as_double(field(out.manifest, "start_x_m"), "replay.start_x_m");
    (void)io::as_object(field(out.manifest, "heightfield"), "replay.heightfield");
    if (version == 2) validate_execution(field(out.manifest, "execution"));
    std::set<std::string> required(files.begin(), files.end());
    if (version == 2) required.insert(more_files.begin(), more_files.end());
    for (auto [key, name] : {std::pair{"joint_envelope_file", "rider_joint_envelope.json"}, std::pair{"joint_strength_file", "rider_joint_strength.json"}}) {
        if (const auto* value = find(out.manifest, key)) {
            if (string(*value) != name) throw recording::RecordingError(std::string("unsupported replay ") + key + " path");
            required.insert(name);
        }
    }
    exact_keys(field(out.manifest, "file_sha256"), required, "replay fixed file set");
    for (const auto& name : required) {
        const auto& hash = field(field(out.manifest, "file_sha256"), name); digest(hash, name);
        auto bytes = io::read_numpy_bytes(path / name); // no-follow, regular opened file, bounded
        if (io::sha256(bytes) != string(hash)) throw recording::RecordingError("recording checksum mismatch: " + name);
        out.assets.emplace(name, std::move(bytes));
    }
    out.summary = io::parse_json(asset_text(out, "summary.json")); (void)io::as_object(out.summary, "summary");
    if (version == 2) {
        const auto& research = field(out.summary, "research");
        if (!equal(field(research, "execution"), field(out.manifest, "execution"))) throw recording::RecordingError("summary and manifest execution identities disagree");
        const auto& resolved = field(out.summary, "resolved_config");
        const auto& articulated = field(field(resolved, "physics"), "articulated");
        for (auto [key, name] : {std::pair{"joint_envelope", "rider_joint_envelope.json"}, std::pair{"joint_strength", "rider_joint_strength.json"}}) {
            const auto* declared = find(articulated, std::string(key) + "_path"); const bool bundled = find(out.manifest, std::string(key) + "_file") != nullptr;
            if ((declared && !io::is_null(*declared)) != bundled || (bundled &&
                string(field(resolved, std::string(key) + "_sha256")) != io::sha256(out.assets.at(name))))
                throw recording::RecordingError(std::string("inconsistent recorded ") + key + " bundle");
        }
    }
    out.requested = json_lines(asset_text(out, "commands_requested.jsonl"));
    out.applied = json_lines(asset_text(out, "commands_applied.jsonl"));
    out.observations = json_lines(asset_text(out, "observations.jsonl"));
    out.transitions = json_lines(asset_text(out, "transitions.jsonl")); command_shapes(out);
    const auto states = io::parse_npz(out.assets.at("states.npz"));
    const io::NpyArray* initial = nullptr; const io::NpyArray* final = nullptr;
    for (const auto& [name, value] : states) { if (name == "initial") initial = &value; else if (name == "final") final = &value; }
    if (!initial || !final || initial->shape.size() != 1 || final->shape != initial->shape) throw recording::RecordingError("incompatible initial/final state shape");
    out.initial_state = initial->doubles(); out.final_state = final->doubles();
    const auto terrain = io::parse_npy(out.assets.at("terrain_vertices.npy"));
    if (terrain.shape.size() != 2 || terrain.shape[1] != 2) throw recording::RecordingError("terrain must be an N-by-2 array");
    out.terrain_vertices = terrain.doubles();
    if (version == 2) out.metrics = io::parse_json(asset_text(out, "episode_metrics.json"));
    return out;
}
} // namespace bike::replay
