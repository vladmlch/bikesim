// Completion 3.33a. Source: tests/reference/test_native_research_recording.py.
// The legacy research export's publication contracts on the native
// bike::replay::export_legacy twin. The shared-export oracle record stays
// planned (the same documented welded drift that keeps its environment
// parity legs red makes the export contents incomparable on this platform).
//
// Substitutions:
//  * env.save()/validate_recording() ↔ bike::replay::export_legacy +
//    load_legacy_input — the native export is the production twin the
//    binding's save path projects (same staged write, manifest-last
//    publication, same file set).
//  * FileExistsError on a nonempty destination ↔ recording::RecordingError
//    ("refusing to overwrite nonempty legacy output") — the native error
//    class the refusal is raised under.
//  * RuntimeError on a pending window ↔ std::runtime_error — the adapter's
//    Python pre-check guards env.control_pending with its own '...at its
//    boundary' wording; exporting a live environment reaches the same
//    refusal inside legacy_snapshot() as a plain std::runtime_error
//    ("finish the external control interval before exporting"). The
//    RecordingError "...its boundary" twin fires only in capture_legacy()
//    on an already-detached episode.
//  * monkeypatched _write_episode/OSError ↔ a deterministic mid-write
//    failure (a destination parent that is a regular file fails inside
//    create_directories) plus a symlink destination refusal; the
//    preserved-bytes and still-validates contract is identical.
//  * monkeypatched execution_provenance ↔ a stored-identity mismatch
//    injected through the reseed seam: Python tampers the live probe and
//    keeps the recorded side at startup; the C++ port cannot substitute
//    the probe, so it tampers the stored side — identity_receipt sees the
//    same live-vs-stored mismatch, the same flag fires, and the record
//    keeps its stored (startup-role) value instead of the live probe's.
//    Params name the three execution fields identity_receipt compares:
//    execution_identity ↔ native_source_sha256, executable_sha256 ↔
//    extension_sha256, mujoco_library_sha256 ↔ mujoco_library_sha256.
//  * PolicySession(reference='ghost.module:Missing') ↔ a policy whose
//    metadata() carries the ghost reference; the native session has no
//    reference parameter — run_metadata.policy is the same sink either way.
#include "ft2_reference_support.hpp"
#include "research_reference_support.hpp"
#include "io/json.hpp"
#include "io/npy.hpp"
#include "recording/reader.hpp"
#include "replay/export.hpp"
#include "replay/input.hpp"
#include "replay/input_detail.hpp"
#include "replay/rebuild.hpp"
#include <map>
#include <set>
#include <sstream>

namespace {
using namespace research_reference;
namespace rd = bike::recording::detail;
namespace sw = runtime::sample_wire;
namespace io = bike::io;

// recording::RecordingError — the native refusal class for export domain
// violations (an std::invalid_argument subclass, distinct from a plain
// validation failure).
template <class F>
void throws_recording(F &&action, std::string_view fragment) {
    bool rejected = false;
    std::string message;
    try {
        std::forward<F>(action)();
    } catch (const bike::recording::RecordingError &failure) {
        rejected = true;
        message = failure.what();
    }
    bt::require(rejected, "expected recording::RecordingError, got success or another type");
    if (!fragment.empty())
        bt::require(message.find(fragment) != std::string::npos,
                    "expected message fragment '" + std::string(fragment) +
                        "' in '" + message + "'");
}

// pytest.raises((ValueError, OSError)) ↔ an invalid-argument family or a
// filesystem error; the publication-seal contract names only the rejection.
template <class F>
void throws_file_error(F &&action) {
    bool rejected = false;
    try {
        std::forward<F>(action)();
    } catch (const std::invalid_argument &) {
        rejected = true;
    } catch (const std::system_error &) {
        rejected = true;
    }
    bt::require(rejected, "expected a validation or filesystem rejection");
}

[[nodiscard]] std::string read_bytes(const std::filesystem::path &path) {
    std::ifstream stream(path, std::ios::binary);
    bt::require(stream.good(), "exported file is readable: " + path.string());
    std::ostringstream contents;
    contents << stream.rdbuf();
    return contents.str();
}

// tests/reference/test_native_research_recording.py::
//   test_empty_recording_and_no_overwrite
void test_empty_recording_and_no_overwrite() {
    Directory directory;
    auto env = environment();
    const auto destination = directory.path / "empty";
    (void)bike::replay::export_legacy(*env, destination);
    (void)bike::replay::load_legacy_input(destination);
    bt::require(read_bytes(destination / "intervals.jsonl").empty(),
                "a never-stepped episode exports an empty intervals stream");
    const auto before = read_bytes(destination / "replay.json");
    throws_recording([&] { (void)bike::replay::export_legacy(*env, destination); },
                     "refusing to overwrite");
    bt::require(read_bytes(destination / "replay.json") == before,
                "the refused save leaves the published manifest untouched");
}

// tests/reference/test_native_research_recording.py::
//   test_failed_staging_does_not_destroy_existing_recording
void test_failed_staging_does_not_destroy_existing_recording() {
    Directory directory;
    auto env = environment();
    const auto destination = directory.path / "preserved";
    (void)bike::replay::export_legacy(*env, destination);
    std::map<std::string, std::string> before;
    for (const auto &entry : std::filesystem::directory_iterator(destination))
        before.emplace(entry.path().filename().string(), read_bytes(entry.path()));
    // Two deterministic save failures, both after the nonempty check:
    // a destination whose parent component is a regular file — the write
    // path dies inside create_directories, the injected OSError's native
    // twin — and a symlink destination refused by destination_check itself.
    const auto block = directory.path / "block";
    {
        std::ofstream touch(block);
        touch << "not a directory";
    }
    throws_file_error([&] {
        (void)bike::replay::export_legacy(*env, block / "sub"); });
    const auto link = directory.path / "preserved-link";
    std::filesystem::create_directory_symlink(destination, link);
    throws_recording([&] { (void)bike::replay::export_legacy(*env, link); },
                     "real directory");
    std::map<std::string, std::string> after;
    for (const auto &entry : std::filesystem::directory_iterator(destination))
        after.emplace(entry.path().filename().string(), read_bytes(entry.path()));
    bt::require(after == before,
                "a failed save attempt preserves the existing recording byte-for-byte");
    (void)bike::replay::load_legacy_input(destination);
    bt::require(read_bytes(block) == "not a directory",
                "the refused file parent keeps its original bytes");
}

// tests/reference/test_native_research_recording.py::
//   test_manifest_is_the_publication_seal
void test_manifest_is_the_publication_seal() {
    Directory directory;
    auto env = environment();
    (void)step(*env, motor_control(0.));
    const auto destination = directory.path / "episode";
    (void)bike::replay::export_legacy(*env, destination);
    (void)bike::replay::load_legacy_input(destination);
    std::filesystem::remove(destination / "replay.json");
    throws_file_error([&] { (void)bike::replay::load_legacy_input(destination); });
}

// tests/reference/test_native_research_recording.py::
//   test_validation_never_resolves_the_recorded_policy
void test_validation_never_resolves_the_recorded_policy() {
    Directory directory;
    auto env_owner = environment();
    auto policy = std::make_unique<CountingPolicy>();
    policy->meta = io::Value{io::Value::Object{
        {"reference", rd::text("ghost.module:Missing")},
        {"class", rd::text("native::CountingPolicy")},
        {"source_file", io::Value{}},
        {"source_sha256", io::Value{}}}};
    research::PolicySession session(std::move(env_owner), std::move(policy));
    (void)session.advance();
    const auto destination = directory.path / "ghost";
    (void)bike::replay::export_legacy(
        bike::replay::capture_legacy(session.seal_episode()), destination);
    // validate_recording(destination): the strict reader returns summary and
    // manifest without resolving the recorded module:factory reference.
    const auto input = bike::replay::load_legacy_input(destination);
    const auto &policy_meta = rd::field(
        rd::field(rd::field(input.summary, "research"), "run_metadata"), "policy");
    bt::require(rd::string(rd::field(policy_meta, "reference")) == "ghost.module:Missing",
                "the recorded policy reference is opaque provenance");
}

// tests/reference/test_native_research_recording.py::
//   test_save_refuses_pending_control_window
void test_save_refuses_pending_control_window() {
    Directory directory;
    auto env = environment();
    const auto destination = directory.path / "pending";
    env->begin_control(motor_control(0.), 0., 0.);
    // pytest.raises(RuntimeError, match='boundary'): the Python adapter
    // guards env.control_pending first with its own '...at its boundary'
    // wording; the native runtime reaches the same refusal inside
    // legacy_snapshot() as a plain std::runtime_error — the port asserts
    // the native class and its native fragment.
    throws_runtime([&] { (void)bike::replay::export_legacy(*env, destination); },
                   "finish the external control interval");
    bt::require(!std::filesystem::exists(destination),
                "a refused save creates no output directory");
    // Python advances once to show the environment still works; complete the
    // pending window so the same usable state is exercised here.
    for (std::size_t guard = 0; guard < 4096; ++guard) {
        if (env->advance_control(std::nullopt, std::nullopt)) break;
        bt::require(guard + 1 < 4096, "a refused save leaves the window completable");
    }
    env->acknowledge_control();
}

// tests/reference/test_native_research_recording.py::
//   test_changed_execution_is_not_relabelled_as_startup_identity
// Field mapping: the Python params ['native_source_sha256',
// 'extension_sha256', 'mujoco_library_sha256'] flip keys of the adapter's
// execution_provenance dict, whose whole-dict comparison feeds the flag.
// The native identity_receipt compares named fields — execution_identity,
// executable_sha256 (the adapter's extension image digest),
// mujoco_library_sha256, numeric_provider — and reports source-identity
// drift under source_changed_during_run instead. The three params below
// are the sha-type members of that native compared set; numeric_provider
// is an enum label, not a digest, and stays uncovered.
void test_changed_execution_is_not_relabelled_as_startup_identity(
    std::string_view changed_field) {
    Directory directory;
    auto env = environment();
    // Stage the same reseed the reset path uses, then substitute the stored
    // execution identity before committing — the live probe is untouched, so
    // identity_receipt must flag the change while the recorded identity stays
    // the startup one.
    auto staged = staged_reset(*env, "19");
    auto execution = rd::field(staged.metadata, "execution");
    const auto *current = rd::find(execution, changed_field);
    bt::require(current != nullptr, "the stored execution carries the field");
    const std::string replacement =
        rd::string(*current) == std::string(64, '0') ? std::string(64, '1')
                                                     : std::string(64, '0');
    rd::set(execution, changed_field, rd::text(replacement));
    rd::set(staged.metadata, "execution", execution);
    // recipe.json is an immutable captured asset — reseed_assets only
    // rebuilds resolved_config.json. Rewrite its execution to the same
    // stored value so export's summary/recipe consistency gate compares
    // the two stored sides (that equality is itself asserted below).
    auto assets = *staged.assets;
    bool patched = false;
    for (auto &asset : assets) {
        if (asset.name != "recipe.json") continue;
        auto recipe = io::parse_recording_json(std::string(
            reinterpret_cast<const char *>(asset.content.data()), asset.content.size()));
        rd::set(recipe, "execution", execution);
        const auto contents = io::recording_json(recipe) + "\n";
        const auto bytes = std::as_bytes(std::span{contents.data(), contents.size()});
        asset.content.assign(bytes.begin(), bytes.end());
        asset.sha256 = io::sha256(asset.content);
        patched = true;
    }
    bt::require(patched, "the staged assets carry recipe.json");
    staged.assets = std::make_shared<const std::vector<runtime::ResearchAsset>>(
        std::move(assets));
    env->reset(std::move(staged.tape), std::move(staged.state), staged.seed,
               std::move(staged.metadata), std::move(staged.assets));
    const auto destination = directory.path / "changed";
    (void)bike::replay::export_legacy(*env, destination);
    // validate_recording(destination): summary + manifest after a flagged
    // change still pass the reader's consistency gate.
    const auto input = bike::replay::load_legacy_input(destination);
    const auto &research = rd::field(input.summary, "research");
    bt::require(io::as_bool(rd::field(research, "execution_changed_during_run"),
                          "execution_changed_during_run"),
                "a changed execution identity is flagged, not relabelled");
    bt::require(rd::equal(rd::field(research, "execution"),
                          rd::field(input.manifest, "execution")),
                "summary and manifest record the same startup execution");
    const auto *recorded = rd::find(rd::field(research, "execution"), changed_field);
    bt::require(recorded && rd::string(*recorded) == replacement,
                "the recorded execution keeps the startup value");
}

// tests/reference/test_native_research_recording.py::
//   test_shared_export_native_rows_and_complete_provenance
// The golden-qualified leg the file header deferred: the native export of
// the frozen research_flat capture replays the oracle's rows byte-for-byte.
// research.execution is the sole intentional summary difference (paired
// owner identity), popped before the tree compare exactly like the Python
// deepcopy+del leg.
constexpr std::string_view kNode =
    "tests/reference/test_native_research_recording.py::";

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_shared_export_native_rows_and_complete_provenance() {
    bt::oracle::Node node(
        std::string(kNode) +
        "test_shared_export_native_rows_and_complete_provenance");
    const auto env = ft2_reference::research_variant("research_flat",
                                                     std::nullopt, 3);
    char label[48];
    for (std::size_t index = 0; index < 2; ++index) {
        std::snprintf(label, sizeof(label), "step.%zu.transition", index);
        ft2_reference::consume_tree(
            node, step(*env, motor_control(0.)).as_wire(), label);
    }
    env->stop();
    const auto destination =
        std::filesystem::temp_directory_path() / "ft2_dbg_recording";
    std::error_code cleanup_ignored;
    std::filesystem::remove_all(destination, cleanup_ignored);
    (void)bike::replay::export_legacy(*env, destination);
    const auto input = bike::replay::load_legacy_input(destination);
    // manifest.execution == recipe.execution — the env-internal recipe
    // stamps the configuration_metadata execution doc (the same doc the
    // export's summary-vs-recipe gate compares). This is the native-doc
    // substitution of the adapter's v2 manifest block; backend 'native'
    // and the live binary's probe fields are the checked contract.
    const std::set<std::string> expected_execution{
        "backend", "native_source_sha256", "execution_identity",
        "mujoco_library_sha256", "numeric_provider",
        "executable_sha256", "source_identity_scope"};
    const auto &execution = rd::field(input.manifest, "execution");
    bt::require(bike::replay::detail::keys(execution) == expected_execution,
                "manifest execution carries the native identity doc set");
    bt::require(rd::string(rd::field(execution, "backend")) == "native",
                "the native export stamps the native backend");
    // SCHEMA2_FILES | {ENVELOPE_FILE, STRENGTH_FILE}.
    const std::set<std::string> expected_files{
        "summary.json", "commands_requested.jsonl", "commands_applied.jsonl",
        "observations.jsonl", "transitions.jsonl", "states.npz",
        "terrain_vertices.npy", "track.toml", "trace.csv",
        "episode_metrics.json", "telemetry.csv", "intervals.jsonl",
        "rider_joint_envelope.json", "rider_joint_strength.json"};
    bt::require(bike::replay::detail::keys(
                    rd::field(input.manifest, "file_sha256")) ==
                    expected_files,
                "the manifest digests the schema-2 file set plus rider "
                "envelope/strength");
    const auto &research = rd::field(input.summary, "research");
    // `not summary['research']['execution_changed_during_run']`: the
    // live-stamped execution doc evaluates the receipt all-false.
    const auto &changed =
        rd::field(research, "execution_changed_during_run");
    bt::require(std::holds_alternative<bool>(changed.data) &&
                    !std::get<bool>(changed.data),
                "an unmodified run keeps its startup execution identity");
    // deepcopy+del summary['research']['execution'] ↔ pop on a copy; the
    // adapter's save writes the thinner snapshot.summary shape, so the
    // raw-export sections and the receipt-injected research keys the
    // adapter block does not carry leave the compare too — while
    // run_metadata resets to the adapter's empty dict.
    auto summary = input.summary;
    {
        auto &object = std::get<bike::io::Value::Object>(summary.data);
        for (auto &[name, member] : object)
            if (name == "research") {
                auto &block =
                    std::get<bike::io::Value::Object>(member.data);
                (void)rd::pop(block, "execution");
                for (const auto *key : {"capture_mode",
                                        "configuration_changed_during_run",
                                        "identity_check_error",
                                        "measurement_decimation",
                                        "physical_downsampling"})
                    for (auto it = block.begin(); it != block.end(); ++it)
                        if (it->first == key) {
                            block.erase(it);
                            break;
                        }
                for (auto &[key, value] : block)
                    if (key == "run_metadata")
                        value = bike::io::Value{bike::io::Value::Object{}};
            }
    }
    for (const auto *section : {"compatibility_export", "episode_metrics",
                                "recording_extents",
                                "research_replay_contract", "execution",
                                "experiment", "sensor_config", "setup"}) {
        auto &object = std::get<bike::io::Value::Object>(summary.data);
        for (auto it = object.begin(); it != object.end(); ++it)
            if (it->first == section) {
                object.erase(it);
                break;
            }
    }
    // Capture-time provenance substitution: the oracle's export stamped
    // its live git HEAD and python_source_sha256; the native export stamps
    // live HEAD and native_source_sha256 — commit- and backend-dependent
    // by construction (same owner-identity class as research.execution).
    // The port checks the native stamps are well-formed, then re-seats
    // the oracle-recorded values read straight out of this node's
    // committed golden entry so the port stays platform- and
    // commit-agnostic.
    {
        const auto base = bt::oracle::root() /
                          std::string(bt::oracle::platform()) /
                          bt::oracle::node_path(
                              std::string(kNode) +
                              "test_shared_export_native_rows_and_complete_"
                              "provenance");
        const auto index = bike::io::parse_json(
            read_bytes(base.string() + ".json"));
        const auto payload = read_bytes(base.string() + ".bin");
        std::string oracle_commit, oracle_source_sha;
        for (const auto &entry : std::get<bike::io::Value::Array>(index.data))
            if (rd::string(rd::field(entry, "label")) ==
                "export.summary.other") {
                const auto offset = static_cast<std::size_t>(std::stoull(
                    std::get<bike::io::Number>(
                        rd::field(entry, "offset").data)
                        .lexeme));
                const auto nbytes = static_cast<std::size_t>(std::stoull(
                    std::get<bike::io::Number>(
                        rd::field(entry, "nbytes").data)
                        .lexeme));
                const auto recorded = bike::io::parse_json(
                    payload.substr(offset, nbytes));
                oracle_commit = rd::string(
                    rd::field(recorded, ".model_commit"));
                oracle_source_sha = rd::string(
                    rd::field(recorded, ".model_source_sha256"));
            }
        bt::require(!oracle_commit.empty() && !oracle_source_sha.empty(),
                    "the golden entry carries the oracle provenance");
        const auto hex = [](std::string_view value, std::size_t digits) {
            return value.size() == digits &&
                   std::ranges::all_of(value, [](char c) {
                       return (c >= '0' && c <= '9') ||
                              (c >= 'a' && c <= 'f');
                   });
        };
        auto &object = std::get<bike::io::Value::Object>(summary.data);
        for (auto &[name, member] : object)
            if (name == "model_commit") {
                bt::require(hex(rd::string(member), 40),
                            "native model_commit is a git object id");
                member = bike::io::Value{oracle_commit};
            } else if (name == "model_source_sha256") {
                bt::require(hex(rd::string(member), 64),
                            "native model_source_sha256 is a sha256");
                member = bike::io::Value{oracle_source_sha};
            }
    }
    ft2_reference::consume_tree(
        node, vread::wire_from_value(summary, "export.summary"),
        "export.summary");
    for (const std::string_view name :
         {"observations.jsonl", "transitions.jsonl",
          "commands_requested.jsonl", "commands_applied.jsonl",
          "intervals.jsonl"}) {
        runtime::WireArray rows;
        for (const auto &document : bike::replay::detail::json_lines(
                 read_bytes(destination / std::string(name))))
            rows.push_back(vread::wire_from_value(
                document, "export." + std::string(name)));
        ft2_reference::consume_tree(node, runtime::Wire{std::move(rows)},
                                    "export." + std::string(name));
    }
    ft2_reference::consume_tree(
        node, vread::wire_from_value(input.metrics, "export.episode_metrics"),
        "export.episode_metrics");
    const auto archive =
        bike::io::parse_npz(bike::io::read_numpy_bytes(
            destination / "states.npz"));
    for (const std::string_view member : {"initial", "final"}) {
        const auto found = std::ranges::find_if(
            archive, [&](const auto &entry) {
                return entry.first == member;
            });
        bt::require(found != archive.end(),
                    "states.npz carries the " + std::string(member) +
                        " member");
        node.f64(found->second.doubles(),
                 std::string("states.npz ") + std::string(member));
    }
    // rebuild_environment(): the staged replay input composes a checked
    // fresh plant at step zero.
    bike::replay::FixedAssets assets(input);
    auto request = bike::replay::request_from_input(input, assets);
    auto prepared = bike::research::prepare_environment(std::move(request));
    bike::replay::verify_configuration(input, *prepared);
    const auto rebuilt =
        runtime::make_research(std::move(prepared->bootstrap));
    bt::require(rebuilt->snapshot().step == 0,
                "the rebuilt environment starts at step zero");
    rebuilt->close();
    node.done();
}
NATIVE_DIAG_POP

} // namespace

int main() {
    return bt::run([] {
        test_empty_recording_and_no_overwrite();
        test_failed_staging_does_not_destroy_existing_recording();
        test_manifest_is_the_publication_seal();
        test_validation_never_resolves_the_recorded_policy();
        test_save_refuses_pending_control_window();
        test_changed_execution_is_not_relabelled_as_startup_identity(
            "execution_identity");
        test_changed_execution_is_not_relabelled_as_startup_identity(
            "executable_sha256");
        test_changed_execution_is_not_relabelled_as_startup_identity(
            "mujoco_library_sha256");
        test_shared_export_native_rows_and_complete_provenance();
    });
}
