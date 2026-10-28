// T53 replay/view ports — shared fixtures for real native record/replay
// contracts plus the synthetic schema-1/2 manifest bundles of
// test_replay_manifest_v2.py. The real path mirrors test_t44's
// native-produced recording round-trip; synthetic bundles exercise the
// data-only reader without a plant.
#pragma once
#include "support/migration_test.hpp"
#include "support/golden.hpp"
#include "migration/recording_fixture.hpp"
#include "engine_call.hpp"
#include "io/hash.hpp"
#include "io/json.hpp"
#include "io/npy.hpp"
#include "platform/files.hpp"
#include "recording/analysis_detail.hpp"
#include "recording/capture.hpp"
#include "recording/duckdb.hpp"
#include "recording/publish.hpp"
#include "recording/reader.hpp"
#include "recording/value_util.hpp"
#include "replay/export.hpp"
#include "replay/input.hpp"
#include "replay/playback.hpp"
#include "replay/rebuild.hpp"
#include "replay/session.hpp"
#include "research/environment.hpp"
#include "research/evidence.hpp"
#include "runtime/control.hpp"
#include "runtime/research.hpp"
#include "runtime/research_io.hpp"
#include "terrain/profile.hpp"
#include <array>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <numeric>
#include <system_error>
#include <unistd.h>

namespace replay_reference {
namespace io = bike::io;
namespace rec = bike::recording;
namespace replay = bike::replay;
namespace research = bike::research;
namespace platform = bike::platform;
using rec::detail::count;
using rec::detail::equal;
using rec::detail::field;
using rec::detail::find;
using rec::detail::integer;
using rec::detail::maybe;
using rec::detail::real;
using rec::detail::set;
using rec::detail::text;

struct Directory {
    std::filesystem::path path;
    Directory() {
        auto pattern = (std::filesystem::temp_directory_path() / "bike-replay-reference-XXXXXX").string();
        std::vector<char> bytes(pattern.begin(), pattern.end());
        bytes.push_back('\0');
        const auto* made = ::mkdtemp(bytes.data());
        if (!made) throw std::system_error(errno, std::generic_category(), "test temporary directory");
        path = made;
    }
    ~Directory() { std::error_code ignored; std::filesystem::remove_all(path, ignored); }
    Directory(const Directory&) = delete;
    Directory& operator=(const Directory&) = delete;
    Directory(Directory&&) = delete;
    Directory& operator=(Directory&&) = delete;
};

[[nodiscard]] inline std::filesystem::path repo_root() {
    // bt::oracle::root() is <repo>/tests/fixtures/golden.
    return bt::oracle::root().parent_path().parent_path().parent_path();
}

[[nodiscard]] inline std::string read_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    bt::require(static_cast<bool>(in), "test fixture file opens for reading");
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}
inline void write_file(const std::filesystem::path& path, std::string_view content) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    bt::require(static_cast<bool>(out), "test fixture file opens for writing");
    out << content;
    out.close();
    bt::require(static_cast<bool>(out), "test fixture write completes");
}
[[nodiscard]] inline std::string digest(const std::filesystem::path& path) { return platform::file_sha256(path); }
inline void write_json(const std::filesystem::path& path, const io::Value& value) {
    write_file(path, io::recording_json(value) + "\n");
}

// ---------------------------------------------------------------------------
// Real native run → sealed container / exported legacy bundle (T44 recipe).
// ---------------------------------------------------------------------------
[[nodiscard]] inline std::unique_ptr<research::ResearchRequest> roundtrip_request() {
    auto owner = std::make_unique<research::ResearchRequest>();
    auto& request = *owner;
    bike::configuration::SimulationPhysicsConfig physics;
    physics.physics_mode = "physical";
    physics.drive_mode = "crank_effort";
    physics.timestep_s = .0005;
    physics.initial_speed_mps = .1;
    physics.equilibrium_cache_enabled = false;
    physics.tires.backend = "compliant_2d";
    physics.tires.surface_mode = "track";
    bike::configuration::RiderSpecs rider;
    rider.variant = "lumped";
    request.setup.model.physics = physics;
    request.setup.model.rider = rider;
    request.setup.model.include_rider = true;
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.track = bike::terrain::flat(12.);
    request.experiment.duration_s = .006;
    request.experiment.control_period_s = .002;
    request.experiment.actuator_delay_s = .001;
    request.experiment.record_decimation = 1;
    request.experiment.stop_on_model_violation = false;
    request.sensors = research::SensorConfig::ideal();
    request.sensors.sample_period_s = .001;
    request.road_resolution_m = .05;
    request.capture_mode = runtime::ResearchCaptureMode::full;
    return owner;
}

struct Environment {
    bike::model::ModelOwner model;
    std::optional<rec::Catalog> catalog;
    std::unique_ptr<runtime::NativeResearchRuntime> runtime;
};
// Heap-owned chain: prepared environment, model image and catalog stay alive
// together so episodes can be sealed and evidence built against the same
// catalog as the model the runtime runs.
[[nodiscard]] inline std::unique_ptr<Environment> make_environment(
    std::unique_ptr<research::ResearchRequest> request = {}) {
    auto prepared = research::prepare_environment(request ? std::move(request) : roundtrip_request());
    auto environment = std::make_unique<Environment>();
    environment->model = bike::model::ModelOwner{engine::load_model_image(prepared->bootstrap.ride.model.mjb)};
    environment->catalog.emplace(rec::build_catalog(*environment->model));
    environment->runtime = runtime::make_research(std::make_unique<runtime::ResearchBootstrap>(std::move(prepared->bootstrap)));
    return environment;
}

// One full control window: begin → advance to the transition → acknowledge.
inline void step_control(runtime::NativeResearchRuntime& environment) {
    runtime::RideControl command;
    command.motor_torque_nm = 0.;
    command.human_torque_nm = 0.;
    environment.begin_control(command, 0., 0.);
    for (std::size_t guard = 0; guard < 4096; ++guard) {
        if (environment.advance_control(std::nullopt, std::nullopt).has_value()) break;
        bt::require(guard + 1 < 4096, "unbounded native control completes a physical window");
    }
    environment.acknowledge_control();
}

[[nodiscard]] inline io::Value sealed_summary(runtime::OwnedResearchEpisode& owner) {
    set(owner.run_metadata, "identity_checks", research::identity_receipt(owner.configuration.metadata));
    return research::summarize(owner);
}

// save_run(): seal one (or two) completed episodes into a sealed container.
[[nodiscard]] inline rec::Publication record_container(
    const std::filesystem::path& root, bool two_episodes = false,
    std::optional<std::string> policy_reference = {}) {
    auto environment = make_environment();
    rec::RunCapture run(recording_fixture::metadata("00000000-0000-4000-8000-000000000053"));
    {
        for (;;) {
            const auto status = research::wire_value(runtime::Wire{environment->runtime->status()});
            if (io::as_bool(field(status, "terminated"), "terminated") ||
                io::as_bool(field(status, "truncated"), "truncated")) break;
            step_control(*environment->runtime);
        }
        auto owner = environment->runtime->seal_episode();
        if (policy_reference)
            set(owner.run_metadata, "policy",
                io::Value{io::Value::Object{{"reference", text(*policy_reference)}}});
        const auto summary = sealed_summary(owner);
        run.add_episode(rec::make_research_evidence(0, std::move(owner), *environment->catalog, summary));
    }
    if (two_episodes) {
        environment->runtime->reset();
        step_control(*environment->runtime);
        environment->runtime->stop();
        auto owner = environment->runtime->seal_episode();
        const auto summary = sealed_summary(owner);
        run.add_episode(rec::make_research_evidence(1, std::move(owner), *environment->catalog, summary));
    }
    auto publication = rec::finalize(run, root / "runs");
    environment->runtime->close();
    return publication;
}

// record_prefix(): a stopped (or empty) run captured then exported to the
// legacy replay.json layout — the schema-2 file set.
[[nodiscard]] inline std::filesystem::path record_prefix(
    const std::filesystem::path& path, std::size_t controls = 2,
    std::optional<std::string> policy_reference = {}) {
    auto environment = make_environment();
    for (std::size_t i = 0; i < controls; ++i) step_control(*environment->runtime);
    if (controls) environment->runtime->stop();
    auto snapshot = replay::capture_legacy(*environment->runtime);
    if (policy_reference) {
        auto research_section = field(snapshot.data.summary, "research");
        set(research_section, "run_metadata",
            io::Value{io::Value::Object{{"policy", io::Value{io::Value::Object{{"reference", text(*policy_reference)}}}}}});
        set(snapshot.data.summary, "research", std::move(research_section));
    }
    (void)replay::export_legacy(snapshot, path, true);
    environment->runtime->close();
    return path;
}

// A policy-failure container: analyzable and playable, never replayable.
[[nodiscard]] inline rec::Publication record_failed(const std::filesystem::path& root) {
    auto environment = make_environment();
    step_control(*environment->runtime);
    environment->runtime->fail_policy("policy exploded");
    auto owner = environment->runtime->seal_episode();
    const auto summary = sealed_summary(owner);
    rec::RunCapture run(recording_fixture::metadata("00000000-0000-4000-8000-000000000054"));
    run.add_episode(rec::make_research_evidence(0, std::move(owner), *environment->catalog, summary));
    auto publication = rec::finalize(run, root / "failed-runs");
    environment->runtime->close();
    return publication;
}

// An ordinary physical capture (no research replay contract).
[[nodiscard]] inline rec::Publication record_ordinary(const std::filesystem::path& root) {
    return recording_fixture::publish(root / "ordinary");
}

// ---------------------------------------------------------------------------
// Sealed-container mutations — checksum-honest edits (recompute the manifest
// digest for changed bytes, exactly like a transported/forged recording).
// ---------------------------------------------------------------------------
inline void reseal_manifest(const std::filesystem::path& directory, std::string_view member) {
    auto manifest = io::parse_recording_json(read_file(directory / "manifest.json"));
    auto hashes = field(manifest, "file_sha256");
    set(hashes, std::string(member), text(digest(directory / std::string(member))));
    set(manifest, "file_sha256", std::move(hashes));
    write_json(directory / "manifest.json", manifest);
}

// UPDATE inside run.duckdb then reseal manifest.json's run.duckdb digest —
// the checksum stays honest while the payload changes.
template <class Mutate>
inline void patch_database(const std::filesystem::path& directory, Mutate&& mutate) {
    {
        rec::Database database(directory / "run.duckdb");
        mutate(database.connection());
        database.connection().execute("CHECKPOINT");
    }
    reseal_manifest(directory, "run.duckdb");
}

// _patch_execution(): rewrite the recorded execution identity inside both the
// episode summary_json and the bundled recipe.json asset (they must agree),
// then reseal — the transported-recording mutation.
template <class Mutate>
inline void patch_execution(const std::filesystem::path& directory, Mutate&& mutate) {
    patch_database(directory, [&](rec::Connection& connection) {
        const std::array params{integer(0)};
        const auto episode_rows = connection.query(
            "SELECT summary_json FROM episodes WHERE episode_id=?", params);
        auto summary = io::parse_recording_json(
            rec::detail::string(field(episode_rows.at(0), "summary_json")));
        auto research_section = field(summary, "research");
        set(research_section, "execution", mutate(field(research_section, "execution")));
        set(summary, "research", std::move(research_section));
        const auto summary_bytes = io::recording_json(summary);
        const std::array update{io::Value{summary_bytes}, params.front()};
        connection.execute("UPDATE episodes SET summary_json=? WHERE episode_id=?", update);
        const auto asset_rows = connection.query(
            "SELECT content FROM assets WHERE episode_id=? AND name='recipe.json'", params);
        const auto& blob = std::get<io::Bytes>(field(asset_rows.at(0), "content").data).value;
        const std::string_view recipe_text(reinterpret_cast<const char*>(blob.data()), blob.size());
        auto recipe = io::parse_recording_json(recipe_text);
        set(recipe, "execution", mutate(field(recipe, "execution")));
        const auto recipe_bytes = io::recording_json(recipe);
        const auto recipe_view = std::as_bytes(std::span{recipe_bytes});
        const std::array asset_update{
            io::Value{io::Bytes{std::vector<std::byte>(recipe_view.begin(), recipe_view.end())}},
            text(io::sha256(recipe_view)), params.front()};
        connection.execute(
            "UPDATE assets SET content=?, sha256=? WHERE episode_id=? AND name='recipe.json'", asset_update);
    });
}

// Resealed final-state tamper: integration_states.final gets a shifted lane;
// the manifest digest is recomputed so only the final comparison boundary
// can notice.
inline void tamper_final_state(const std::filesystem::path& directory) {
    patch_database(directory, [](rec::Connection& connection) {
        const auto rows = connection.query(
            "SELECT state FROM integration_states WHERE phase='final'");
        const auto& lanes = io::as_array(field(rows.at(0), "state"), "final state");
        std::vector<double> state;
        state.reserve(lanes.size());
        for (const auto& lane : lanes) state.push_back(io::as_double(lane, "final state lane"));
        bt::require(state.size() > 1, "recorded final state has at least two lanes");
        state[1] += .01;
        const std::array update{rec::value::doubles(state)};
        connection.execute("UPDATE integration_states SET state=? WHERE phase='final'", update);
    });
}

// ---------------------------------------------------------------------------
// Legacy-bundle mutations (replay.json + file set).
// ---------------------------------------------------------------------------
inline void reseal(const std::filesystem::path& path, std::string_view name) {
    auto manifest = io::parse_json(read_file(path / "replay.json"));
    auto hashes = field(manifest, "file_sha256");
    set(hashes, std::string(name), text(digest(path / std::string(name))));
    set(manifest, "file_sha256", std::move(hashes));
    write_file(path / "replay.json", io::recording_json(manifest) + "\n");
}

// Legacy-bundle execution patch: summary.json's research.execution and
// replay.json's manifest.execution must agree (the loader cross-checks), so
// the same mutation is applied to both and summary.json resealed.
template <class Mutate>
inline void patch_legacy_execution(const std::filesystem::path& path, Mutate&& mutate) {
    auto summary = io::parse_json(read_file(path / "summary.json"));
    auto research_section = field(summary, "research");
    set(research_section, "execution", mutate(field(research_section, "execution")));
    set(summary, "research", std::move(research_section));
    write_file(path / "summary.json", io::recording_json(summary) + "\n");
    reseal(path, "summary.json");
    auto manifest = io::parse_json(read_file(path / "replay.json"));
    set(manifest, "execution", mutate(field(manifest, "execution")));
    write_file(path / "replay.json", io::recording_json(manifest) + "\n");
}

// Checksum-honest states.npz tamper: lane `lane` of member `member` shifts by
// `delta`; the manifest digest is recomputed so only the semantic boundary
// can notice.
inline void tamper_npz_state(const std::filesystem::path& path, std::string_view member,
    std::size_t lane, double delta) {
    const auto full = path / "states.npz";
    const auto bytes = io::read_numpy_bytes(full);
    auto archive = io::parse_npz(std::span<const std::byte>(bytes.data(), bytes.size()));
    for (auto& [name, array] : archive) {
        if (name != member) continue;
        auto values = array.doubles();
        bt::require(lane < values.size(), "tamper lane exists");
        values[lane] += delta;
        const auto bytes_view = std::as_bytes(std::span(values));
        array.bytes.assign(bytes_view.begin(), bytes_view.end());
    }
    const auto packed = io::encode_npz(archive);
    write_file(full, std::string_view(reinterpret_cast<const char*>(packed.data()), packed.size()));
    reseal(path, "states.npz");
}

// Checksum-honest JSONL semantic tamper: row[index]'s key gains +1, then the
// manifest digest is recomputed — the format layer stays valid and only the
// semantic comparison can notice.
inline void tamper_jsonl_row(const std::filesystem::path& path, std::string_view file,
    std::size_t index, std::string_view key) {
    const auto full = path / std::string(file);
    const auto source = read_file(full);
    std::vector<std::string> lines;
    for (std::size_t begin = 0; begin < source.size();) {
        const auto end = source.find('\n', begin);
        lines.push_back(std::string(source.substr(begin, end == std::string::npos ? end : end - begin)));
        if (end == std::string::npos) break;
        begin = end + 1;
    }
    bt::require(index < lines.size(), "tamper target row exists");
    auto row = io::parse_json(lines[index]);
    const auto& current = field(row, std::string(key));
    if (const auto* number = std::get_if<io::Number>(&current.data); number && io::is_integer_lexeme(number->lexeme))
        set(row, std::string(key), integer(count(current) + 1));
    else set(row, std::string(key), real(io::as_double(current, key) + 1.));
    lines[index] = io::recording_json(row);
    std::string joined;
    for (const auto& line : lines) { joined += line; joined += '\n'; }
    write_file(full, joined);
    reseal(path, file);
}

// ---------------------------------------------------------------------------
// Synthetic schema-1/2 fixtures — test_replay_manifest_v2.py's recording().
// Data-only bundles: opaque file bytes, no physical rollout.
// ---------------------------------------------------------------------------
inline constexpr std::array SCHEMA1_FILES{"summary.json", "commands_requested.jsonl",
    "commands_applied.jsonl", "observations.jsonl", "transitions.jsonl",
    "states.npz", "terrain_vertices.npy", "track.toml"};
inline constexpr std::array SCHEMA2_EXTRA{"trace.csv", "episode_metrics.json",
    "telemetry.csv", "intervals.jsonl"};
inline constexpr std::string_view ENVELOPE_FILE = "rider_joint_envelope.json";
inline constexpr std::string_view STRENGTH_FILE = "rider_joint_strength.json";

// identity(backend, versioned): legacy 8-field or identity_schema=2 record.
[[nodiscard]] inline io::Value identity(std::string_view backend = "native", bool versioned = false) {
    const bool native = backend == "native";
    io::Value record{io::Value::Object{
        {"backend", text(backend)},
        {"runtime_schema", integer(1)},
        {"python_source_sha256", text("1111111111111111111111111111111111111111111111111111111111111111")},
        {"native_source_sha256", native ? text("2222222222222222222222222222222222222222222222222222222222222222") : io::Value{}},
        {"extension_sha256", native ? text("3333333333333333333333333333333333333333333333333333333333333333") : io::Value{}},
        {"mujoco_version", text("3.12.0")},
        {"mujoco_library_sha256", native ? text("4444444444444444444444444444444444444444444444444444444444444444") : io::Value{}},
        {"build_context", native ? io::Value{io::Value::Object{
            {"build_type", text("Release")}, {"compiler_id", text("AppleClang")},
            {"compiler_version", text("21.0.0")}, {"numerical_flags", text("-O3 -ffp-contract=off")}}} : io::Value{}}}};
    if (versioned) {
        io::Value platform{io::Value::Object{
            {"system", text("Linux")}, {"machine", text("aarch64")},
            {"libc", io::Value{io::Value::Object{{"name", text("glibc")}, {"version", text("2.39")}}}},
            {"os_build", text("6.8.0-87-generic")}, {"python_version", text("3.13.16")},
            {"python_soabi", text("cpython-313-aarch64-linux-gnu")}}};
        io::Value numerics{io::Value::Object{
            {"backend", text("NUMPY_OPENBLAS")}, {"numpy_version", text("2.5.2")},
            {"blas_integer_bits", integer(64)}, {"lapack_integer_bits", integer(64)},
            {"cpu_profile", text("neoversen1")},
            {"threading", io::Value{io::Value::Object{
                {"implementation", text("openblas-pthreads")}, {"num_threads", integer(16)}}}},
            {"libraries", io::Value{io::Value::Array{io::Value{io::Value::Object{
                {"role", text("blas_lapack")}, {"identity_kind", text("file")},
                {"sha256", text("5555555555555555555555555555555555555555555555555555555555555555")},
                {"build_id", io::Value{}}}}}}}}};
        io::Value::Object context;
        // The native record keeps its four legacy build-context members plus
        // the platform/numerics pair; a python record carries exactly the pair.
        if (native) for (auto key : {"build_type", "compiler_id", "compiler_version", "numerical_flags"})
            context.emplace_back(key, field(field(record, "build_context"), key));
        context.emplace_back("platform", std::move(platform));
        context.emplace_back("numerics", std::move(numerics));
        set(record, "build_context", io::Value{std::move(context)});
        set(record, "identity_schema", integer(2));
    }
    return record;
}

// The recording() fixture: a data-only bundle of authenticated bytes.
[[nodiscard]] inline std::filesystem::path synthetic_recording(
    const std::filesystem::path& root, int version = 2,
    std::string_view backend = "native", bool bundles = true, bool versioned = false) {
    auto path = root / (std::string("v") + std::to_string(version) + "-" +
        std::string(backend) + "-" + (bundles ? "bundled" : "bare") + (versioned ? "-versioned" : ""));
    std::filesystem::create_directories(path);
    std::vector<std::string> files(SCHEMA1_FILES.begin(), SCHEMA1_FILES.end());
    if (version == 2) files.insert(files.end(), SCHEMA2_EXTRA.begin(), SCHEMA2_EXTRA.end());
    // The native reader eagerly decodes the four JSONL streams, states.npz,
    // terrain_vertices.npy and (schema-2) episode_metrics.json: those members
    // carry the minimal valid payload (zero commands, one observation row)
    // while the opaque members stay inert bytes.
    const auto zero_f8 = [](std::vector<std::size_t> shape) {
        io::NpyArray array;
        array.dtype = "<f8"; array.shape = std::move(shape);
        array.bytes.assign(std::accumulate(array.shape.begin(), array.shape.end(),
            std::size_t{1}, std::multiplies<>()) * 8, std::byte{});
        return array;
    };
    for (const auto& name : files) {
        if (name == "observations.jsonl") write_file(path / name, "{}\n");
        else if (name == "states.npz") {
            const auto packed = io::encode_npz(io::NpzArchive{{"initial", zero_f8({2})}, {"final", zero_f8({2})}});
            write_file(path / name, std::string_view(reinterpret_cast<const char*>(packed.data()), packed.size()));
        } else if (name == "terrain_vertices.npy") {
            const auto packed = io::encode_npy(zero_f8({1, 2}));
            write_file(path / name, std::string_view(reinterpret_cast<const char*>(packed.data()), packed.size()));
        } else if (name == "episode_metrics.json") write_file(path / name, "{}\n");
        else if (name.ends_with(".jsonl")) write_file(path / name, "");
        else write_file(path / name, "opaque data-only reader fixture\n");
    }
    const auto execution = identity(backend, versioned);
    io::Value articulated{io::Value::Object{
        {"joint_envelope_path", io::Value{}}, {"joint_strength_path", io::Value{}}}};
    io::Value resolved{io::Value::Object{
        {"physics", io::Value{io::Value::Object{{"articulated", io::Value{}}}}}}};
    io::Value summary{io::Value::Object{
        {"resolved_config", io::Value{}},
        {"research", io::Value{io::Value::Object{
            {"execution", execution},
            {"run_metadata", io::Value{io::Value::Object{
                {"policy", io::Value{io::Value::Object{
                    {"reference", text("malicious_recorded_module:factory")}}}}}}}}}}}};
    io::Value manifest{io::Value::Object{
        {"schema_version", integer(version)}, {"start_x_m", real(.25)},
        {"heightfield", io::Value{io::Value::Object{}}},
        {"state_signature", text("mjSTATE_INTEGRATION")}}};
    if (version == 2) set(manifest, "execution", execution);
    std::vector<std::string> optional;
    if (bundles) {
        std::vector<std::pair<std::string, std::string>> choices{{"joint_envelope", std::string(ENVELOPE_FILE)}};
        if (version == 2) choices.emplace_back("joint_strength", std::string(STRENGTH_FILE));
        for (const auto& [key, name] : choices) {
            write_file(path / name, "{\"data_only\":true}\n");
            set(articulated, key + "_path", text("/untrusted/recipe/path/" + name));
            set(resolved, key + "_sha256", text(digest(path / name)));
            set(manifest, key + "_file", text(name));
            optional.push_back(name);
        }
    }
    auto physics = field(resolved, "physics");
    set(physics, "articulated", std::move(articulated));
    set(resolved, "physics", std::move(physics));
    set(summary, "resolved_config", std::move(resolved));
    write_json(path / "summary.json", summary);
    io::Value::Object hashes;
    for (const auto& name : files) hashes.emplace_back(name, text(digest(path / name)));
    for (const auto& name : optional) hashes.emplace_back(name, text(digest(path / name)));
    set(manifest, "file_sha256", io::Value{std::move(hashes)});
    write_json(path / "replay.json", manifest);
    return path;
}

// ---------------------------------------------------------------------------
// Small assertion helpers.
// ---------------------------------------------------------------------------
template <class Action> inline void rejects(Action&& action, std::string_view needle = {}) {
    bool rejected = false;
    std::string message;
    try { std::forward<Action>(action)(); } catch (const std::exception& failure) {
        rejected = true; message = failure.what(); }
    bt::require(rejected, "operation must reject invalid input");
    if (!needle.empty())
        bt::require(message.find(needle) != std::string::npos,
            "rejection names the expected contract: " + std::string(needle) + " (got: " + message + ")");
}
template <class Action> inline void recording_error(Action&& action, std::string_view needle = {}) {
    bool rejected = false;
    std::string message;
    try { std::forward<Action>(action)(); } catch (const rec::RecordingError& failure) {
        rejected = true; message = failure.what(); }
    bt::require(rejected, "operation raises a recording-domain error");
    if (!needle.empty())
        bt::require(message.find(needle) != std::string::npos,
            "rejection names the expected contract: " + std::string(needle) + " (got: " + message + ")");
}
template <class Action> inline void incompatible(Action&& action, std::string_view needle = {}) {
    bool rejected = false;
    std::string message;
    try { std::forward<Action>(action)(); } catch (const replay::IncompatibleExecution& failure) {
        rejected = true; message = failure.what(); }
    bt::require(rejected, "replay rejects the incompatible execution identity");
    if (!needle.empty())
        bt::require(message.find(needle) != std::string::npos,
            "rejection names the mismatched field: " + std::string(needle) + " (got: " + message + ")");
}
[[nodiscard]] inline bool flag(const io::Value& value, std::string_view key) {
    return io::as_bool(field(value, std::string(key)), std::string(key));
}
[[nodiscard]] inline std::string_view text_of(const io::Value& value, std::string_view key) {
    const auto& child = field(value, std::string(key));
    const auto* result = std::get_if<std::string>(&child.data);
    bt::require(result != nullptr, "diagnostic field is a string");
    return *result;
}
// ReplaySession report() must refuse before completed verification.
inline void report_incomplete(const replay::ReplaySession& session) {
    bool rejected = false;
    try { (void)session.report(); } catch (const std::runtime_error& failure) {
        rejected = std::string_view(failure.what()).find("incomplete") != std::string_view::npos;
    }
    bt::require(rejected, "incomplete replay reports must refuse");
}
} // namespace replay_reference
