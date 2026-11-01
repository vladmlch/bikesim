// Port of tests/test_recording_platform_compatibility.py — the portability
// split: bike-recording inspection commands and bike-replay --view
// recorded-frame playback stay portable (no execution-identity comparison,
// no plant construction, no physical stepping), while bike-replay remains
// checked physical replay gated on the recorded execution identity.
//
// Marker divergences from the Python surface, all documented in the ledger
// evidence: the native --view --dry-run prints only "frames=... ok" (the
// Python CLI prints 'unverified' on stderr); the native gate's compared
// axis is the native-executable profile, so "the other host" is a foreign
// execution_identity/mujoco_library_sha256 digest rather than a
// build_context.platform.system string; and the poisoned-symbol subprocess
// is observationally covered by a foreign-identity recording — any
// consulted gate would refuse.
#include "../reference/replay_reference_support.hpp"
#include "platform/process.hpp"
#include "platform/provenance.hpp"
#include <set>

namespace {
namespace rr = replay_reference;
using rr::count;
using rr::field;
using rr::set;
using rr::text;
using bike::io::Value;
using Object = Value::Object;

[[nodiscard]] std::filesystem::path binary(std::string_view name) {
    // CLI artifacts land at <build-root>/bin; this test's binary is four
    // levels down in native/tests/integration.
    return bike::platform::executable_path()
        .parent_path().parent_path().parent_path().parent_path() / "bin" / name;
}
[[nodiscard]] bike::platform::ProcessResult cli(const std::filesystem::path& tool,
        std::initializer_list<std::string> arguments, const std::filesystem::path& cwd,
        double timeout_s = 300.) {
    std::vector<std::string> argv{tool.string()};
    argv.insert(argv.end(), arguments.begin(), arguments.end());
    bike::platform::ProcessOptions options;
    options.timeout_s = timeout_s;
    return bike::platform::run(argv, cwd, options);
}
// A structurally valid foreign identity: the digests remain 64-hex so the
// recorded profile validates and stays readable — only the gate refuses.
[[nodiscard]] Value foreign_execution(const Value& execution, std::string_view key) {
    auto patched = execution;
    set(patched, key, text(std::string(64, '0')));
    return patched;
}
[[nodiscard]] Value legacy_execution() {
    return Value{Object{{"backend", text("native")}, {"runtime_schema", rr::integer(1)},
        {"python_source_sha256", text(std::string(64, 'a'))},
        {"native_source_sha256", text(std::string(64, 'b'))},
        {"extension_sha256", text(std::string(64, 'c'))},
        {"mujoco_version", text("3.12.0")},
        {"mujoco_library_sha256", text(std::string(64, 'd'))},
        {"build_context", Value{Object{{"build_type", text("Release")},
            {"compiler_id", text("Clang")}, {"compiler_version", text("21.0.0")},
            {"numerical_flags", text("-O3 -ffp-contract=off")}}}}}};
}
[[nodiscard]] Value schema99_execution() {
    return Value{Object{{"backend", text("python")}, {"runtime_schema", rr::integer(1)},
        {"python_source_sha256", text(std::string(64, 'a'))},
        {"native_source_sha256", Value{}}, {"extension_sha256", Value{}},
        {"mujoco_version", text("3.12.0")}, {"mujoco_library_sha256", Value{}},
        {"build_context", Value{}}, {"identity_schema", rr::integer(99)}}};
}
[[nodiscard]] std::map<std::string, std::string> file_hashes(
        const std::filesystem::path& directory) {
    std::map<std::string, std::string> out;
    for (const auto& entry : std::filesystem::directory_iterator(directory))
        if (entry.is_regular_file())
            out.emplace(entry.path().filename().string(),
                bike::platform::file_sha256(entry.path()));
    return out;
}
void incompatible_check(const std::filesystem::path& directory,
        std::string_view needle) {
    const auto result = rr::replay::check(directory, 0);
    bt::require(!result.matched &&
        rr::text_of(result.details, "status") == "incompatible-execution" &&
        result.reason.find(needle) != std::string::npos,
        "checked replay refuses the foreign identity by name: " + result.reason);
}

// test_inspection_commands_open_recordings_without_any_native_artifact —
// every inspection command is exit-0 on a sealed container; the export
// payload declares itself analytics, not a checked-replay bundle.
void inspection_commands() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  const auto tool = binary("bike-recording");
  bt::require(std::filesystem::exists(tool),
              "the bike-recording artifact is built");
  const auto &dir = publication.directory;
  const std::vector<std::vector<std::string>> commands{
      {"overview", dir.string()},
      {"fields", dir.string()},
      {"events", dir.string(), "--episode", "0"},
      {"window", dir.string(), "--episode", "0", "--start", "0", "--end", "1",
       "--fields", "physics.time_s"},
      {"sql", dir.string(), "--query", "SELECT count(*) AS n FROM physics",
       "--json"},
      {"compare", dir.string(), dir.string(), "--left-episode", "0",
       "--right-episode", "0"},
      {"compare", dir.string(), dir.string(), "--mode", "summary",
       "--left-episode", "0", "--right-episode", "0"},
  };
  for (const auto &arguments : commands) {
    std::vector<std::string> argv{tool.string()};
    argv.insert(argv.end(), arguments.begin(), arguments.end());
    bike::platform::ProcessOptions options;
    options.timeout_s = 60.;
    const auto ran = bike::platform::run(argv, tmp.path, options);
    bt::require(ran.exit_code == 0,
                "inspection command exits 0: " + arguments.front() + " (" +
                    ran.err + ")");
  }
    const auto out = tmp.path / "exported.jsonl";
    const auto exported = cli(tool, {"export", dir.string(), "--episode", "0",
        "--format", "jsonl", "--output", out.string(), "--table", "metrics"}, tmp.path, 60.);
    bt::require(exported.exit_code == 0 &&
        exported.out.find("not a checked-replay bundle") != std::string::npos,
        "the export payload declares analytics-only semantics");
    bt::require(std::filesystem::file_size(out) > 0, "the export writes rows");
}

// test_inspection_reads_a_recording_authored_on_another_host — a foreign
// identity is still fully inspectable.
void inspection_reads_foreign() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  rr::patch_execution(publication.directory, [](const Value &execution) {
    return foreign_execution(execution, "execution_identity");
  });
  const auto tool = binary("bike-recording");
  const auto overview =
      cli(tool, {"overview", publication.directory.string(), "--json"},
          tmp.path, 60.);
  bt::require(overview.exit_code == 0,
              "overview reads the foreign recording: " + overview.err);
  const auto parsed = bike::io::parse_json(overview.out);
  bt::require(!rr::io::as_array(field(field(parsed, "episodes"), "rows"),
                                "episode rows")
                   .empty(),
              "the foreign recording's episode rows are inspectable");
  const auto sql = cli(tool,
                       {"sql", publication.directory.string(), "--query",
                        "SELECT count(*) AS n FROM physics", "--json"},
                       tmp.path, 60.);
  bt::require(sql.exit_code == 0,
              "sql reads the foreign recording: " + sql.err);
  const auto recording = rr::rec::open_recording(publication.directory);
  bt::require(recording->schema_version() == 1,
              "the database still opens against its manifest");
}

// test_checked_replay_still_demands_an_explicit_episode — a multi-episode
// container refuses the implicit selection with the --episode hint, and each
// episode stays individually selectable.
void explicit_episode_demanded() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path, true);
  rr::rejects(
      [&] { (void)rr::replay::load_replay_input(publication.directory); },
      "--episode");
  rr::rejects([&] { (void)rr::replay::load_playback(publication.directory); },
              "--episode");
  const auto playback = rr::replay::load_playback(publication.directory, 1);
  bt::require(playback.episode_id == 1, "episode 1 remains selectable");
}

// test_recorded_playback_supports_a_foreign_host_recording — the same file
// checked replay refuses is still fully playable.
void playback_supports_foreign() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  rr::patch_execution(publication.directory, [](const Value &execution) {
    return foreign_execution(execution, "execution_identity");
  });
  const auto playback = rr::replay::load_playback(publication.directory);
  bt::require(playback.times_s.size() >= 2 &&
                  playback.qpos.size() == playback.qvel.size() &&
                  playback.qpos.size() == playback.times_s.size() &&
                  std::isfinite(playback.duration_s) &&
                  playback.duration_s > 0.,
              "the foreign recording still loads a playable frame table");
  const auto pose = rr::replay::pose_at(playback, playback.duration_s * .5);
  bt::require(pose.size() == playback.qpos.front().size(),
              "an interpolated pose keeps the stored qpos row shape");
}

// test_recorded_playback_never_claims_physical_verification — the view path
// completes and never prints a verification claim.
void playback_never_claims_verification() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  const auto playback = rr::replay::load_playback(publication.directory);
  bt::require(playback.times_s.size() >= 2,
              "the recording loads playable frames");
  const auto viewed =
      cli(binary("bike-replay"),
          {publication.directory.string(), "--view", "--dry-run"}, tmp.path);
  bt::require(viewed.exit_code == 0 &&
                  viewed.out.find("frames=") != std::string::npos,
              "the dry-run view walks the stored trajectory");
  bt::require(viewed.out.find("verified") == std::string::npos &&
                  viewed.err.find("verified") == std::string::npos &&
                  viewed.out.find("passed") == std::string::npos,
              "the dry-run view never claims verification");
}

// test_failed_runs_stay_playable_but_never_replayable — failure evidence is
// analyzable and playable, never replayable.
void failed_runs_playable_not_replayable() {
  rr::Directory const tmp;
  const auto publication = rr::record_failed(tmp.path);
  rr::rejects(
      [&] { (void)rr::replay::load_replay_input(publication.directory); },
      "not completed replay");
  const auto playback = rr::replay::load_playback(publication.directory);
  bt::require(playback.times_s.size() >= 2,
              "the failed run still loads playable frames");
  const auto viewed =
      cli(binary("bike-replay"),
          {publication.directory.string(), "--view", "--dry-run"}, tmp.path);
  bt::require(viewed.exit_code == 0 &&
                  viewed.out.find("verified") == std::string::npos,
              "the failed run still plays without a verification claim");
  const auto overview =
      cli(binary("bike-recording"),
          {"overview", publication.directory.string()}, tmp.path, 60.);
  bt::require(overview.exit_code == 0, "the failed run still inspects");
}

// test_playback_rejects_recordings_without_the_authored_contract — an
// ordinary capture owns no replay recipe; playback says so instead of
// manufacturing one, and inspection is unaffected.
void playback_requires_recipe() {
  rr::Directory const tmp;
  const auto publication = rr::record_ordinary(tmp.path);
  rr::rejects([&] { (void)rr::replay::load_playback(publication.directory); },
              "recipe");
  const auto recording = rr::rec::open_recording(publication.directory);
  const auto overview = rr::rec::overview(*recording);
  bt::require(!rr::io::as_array(field(field(overview, "episodes"), "rows"),
                                "episode rows")
                   .empty(),
              "inspection of the ordinary capture is unaffected");
}

// test_inspection_and_playback_never_mutate_the_source — every regular file
// keeps its bytes across inspection, playback and the dry-run view.
void source_never_mutated() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  const auto before = file_hashes(publication.directory);
  bt::require(!before.empty(), "the sealed container carries files");
  const auto overview =
      cli(binary("bike-recording"),
          {"overview", publication.directory.string()}, tmp.path, 60.);
  bt::require(overview.exit_code == 0, "overview runs");
  const auto sql = cli(binary("bike-recording"),
                       {"sql", publication.directory.string(), "--query",
                        "SELECT count(*) AS n FROM physics"},
                       tmp.path, 60.);
  bt::require(sql.exit_code == 0, "sql runs");
  (void)rr::replay::load_playback(publication.directory);
  const auto viewed =
      cli(binary("bike-replay"),
          {publication.directory.string(), "--view", "--dry-run"}, tmp.path);
  bt::require(viewed.exit_code == 0, "the dry-run view runs");
  bt::require(file_hashes(publication.directory) == before,
              "inspection and playback never mutate the source");
}

// test_recording_survives_a_directory_move — the container is
// self-describing: a relocate returns identical semantic inspection counts
// and identical playback.
void directory_move_survives() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  const auto recording = rr::rec::open_recording(publication.directory);
  const auto before_overview = rr::rec::overview(*recording);
  const auto before_sql = recording->query("SELECT count(*) AS n FROM physics");
  recording->close();
  const auto destination = tmp.path / "moved" / "run";
  std::filesystem::create_directories(destination.parent_path());
  std::filesystem::rename(publication.directory, destination);
  const auto moved = rr::rec::open_recording(destination);
  const auto after_overview = rr::rec::overview(*moved);
  const auto after_sql = moved->query("SELECT count(*) AS n FROM physics");
  bt::require(
      rr::equal(field(after_overview, "episodes"),
                field(before_overview, "episodes")) &&
          rr::equal(field(field(after_overview, "manifest"), "row_counts"),
                    field(field(before_overview, "manifest"), "row_counts")) &&
          rr::equal(after_sql, before_sql),
      "semantic inspection results are identical after the move");
  moved->close();
  const auto playback = rr::replay::load_playback(destination);
  bt::require(playback.times_s.size() >= 2, "the moved container still plays");
}

// test_corrupted_assets_fail_closed — a byte-level edit inside the
// container is a checksum failure for inspection, playback and checked
// replay alike.
void corrupted_assets_fail_closed() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  const auto database = publication.directory / "run.duckdb";
  auto blob = rr::read_file(database);
  blob.back() = static_cast<char>(blob.back() ^ 0xFF); // NOLINT(bugprone-signed-bitwise) fixed-width literal shift is the fixture contract
  rr::write_file(database, blob);
  for (const auto &probe : {0, 1, 2}) {
    bool rejected = false;
    std::string message;
    try {
      if (probe == 0) {
        const auto r = rr::rec::open_recording(publication.directory);
      } else if (probe == 1) {
        (void)rr::replay::load_playback(publication.directory);
      } else {
        (void)rr::replay::load_replay_input(publication.directory);
      }
    } catch (const std::exception &error) {
      rejected = true;
      message = error.what();
    }
    bt::require(rejected && (message.find("checksum") != std::string::npos ||
                             message.find("Checksum") != std::string::npos ||
                             message.find("sha256") != std::string::npos),
                "corrupted bytes fail closed on the checksum: " + message);
  }
}

// test_missing_episode_is_a_named_error — a nonexistent episode selection
// names itself for both loaders.
void missing_episode_named() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  rr::rejects(
      [&] { (void)rr::replay::load_replay_input(publication.directory, 7); },
      "no episode 7");
  rr::rejects(
      [&] { (void)rr::replay::load_playback(publication.directory, 7); },
      "no episode 7");
}

// test_foreign_identity_cannot_be_checked_replayed — the literal
// field-level gate in either direction: a foreign recorded digest or a
// foreign provider digest each names itself before any physics.
void foreign_identity_never_checked_replayed() {
    for (const char* key : {"execution_identity", "mujoco_library_sha256"}) {
      rr::Directory const tmp;
      const auto publication = rr::record_container(tmp.path);
      rr::patch_execution(publication.directory, [&](const Value &execution) {
        return foreign_execution(execution, key);
      });
      incompatible_check(publication.directory, key);
    }
}

// test_legacy_identity_report_names_the_missing_fields — a pre-versioned
// (no identity_schema, no execution_identity) recorded identity is refused
// by name: the verdict reports the missing native-executable fixture
// wholesale.
void legacy_identity_named() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  rr::patch_execution(publication.directory,
                      [](const Value &) { return legacy_execution(); });
  const auto result = rr::replay::check(publication.directory, 0);
  bt::require(
      !result.matched &&
          rr::text_of(result.details, "status") == "incompatible-execution" &&
          result.reason.find("historical") != std::string::npos,
      "a legacy recorded identity is refused by name: " + result.reason);
}

// test_foreign_identity_rejects_checked_replay_with_the_named_field — the
// constructor-time gate names the field before a plant exists; the data
// stay readable.
void foreign_identity_named_before_plant() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  rr::patch_execution(publication.directory, [](const Value &execution) {
    return foreign_execution(execution, "execution_identity");
  });
  rr::incompatible(
      [&] {
        rr::replay::ReplaySession const session(publication.directory, 0);
      },
      "execution_identity");
  const auto recording = rr::rec::open_recording(publication.directory);
  bt::require(recording->schema_version() == 1,
              "inspection does not apply the gate");
}

// test_unknown_identity_schema_fails_the_recipe_before_stepping — an
// unknown identity_schema inside the recipe is rejected by name before any
// stepping.
void unknown_schema_fails_recipe() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  rr::patch_execution(publication.directory,
                      [](const Value &) { return schema99_execution(); });
  rr::rejects(
      [&] { (void)rr::replay::load_replay_input(publication.directory); },
      "identity_schema");
}

// test_inspection_and_dry_run_run_without_artifact_or_stepping — the
// clean-subprocess contract: on a foreign-identity recording (where a
// consulted gate would refuse) every inspection command and the dry-run
// view still complete.
void inspection_subprocess_portable() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  rr::patch_execution(publication.directory, [](const Value &execution) {
    return foreign_execution(execution, "execution_identity");
  });
  const auto recording_tool = binary("bike-recording");
  for (const auto &arguments :
       {std::vector<std::string>{"overview", publication.directory.string(),
                                 "--json"},
        {"fields", publication.directory.string()},
        {"sql", publication.directory.string(), "--query",
         "SELECT count(*) AS n FROM physics", "--json"}}) {
    std::vector<std::string> argv{recording_tool.string()};
    argv.insert(argv.end(), arguments.begin(), arguments.end());
    bike::platform::ProcessOptions options;
    options.timeout_s = 60.;
    const auto ran = bike::platform::run(argv, tmp.path, options);
    bt::require(ran.exit_code == 0,
                "inspection completes without verify machinery: " +
                    arguments.front());
  }
    const auto viewed = cli(binary("bike-replay"),
        {publication.directory.string(), "--view", "--dry-run"}, tmp.path);
    bt::require(viewed.exit_code == 0,
        "the dry-run view completes without verify machinery");
}

// test_platform_recording_fixture_manifests_are_wellformed — the recorded
// provenance documents stay well-formed: pinned container/replay schema
// versions, a qualified host pairing and a complete digest table.
void fixture_manifests_wellformed() {
    const auto directory =
        rr::repo_root() / "tests" / "fixtures" / "platform-recordings";
    std::vector<std::filesystem::path> fixtures;
    for (const auto& entry : std::filesystem::directory_iterator(directory))
        if (entry.path().extension() == ".json") fixtures.push_back(entry.path());
    bt::require(!fixtures.empty(),
        "tests/fixtures/platform-recordings must carry manifests");
    for (const auto& file : fixtures) {
        const auto manifest = bike::io::parse_json(rr::read_file(file));
        bt::require(rr::text_of(manifest, "fixture") == "platform-recordings",
            "the manifest declares the platform-recordings fixture");
        const auto& recording = field(manifest, "recording");
        bt::require(count(field(recording, "container_schema_version")) == 1 &&
            count(field(recording, "replay_contract_version")) == 3,
            "the manifest pins container schema 1 / replay contract 3");
        const auto& host = field(field(manifest, "generator"), "host");
        const std::string system(rr::text_of(host, "system"));
        bt::require(system == "Darwin" || system == "Linux",
            "the recorded generator host is a qualified system");
        const auto& files = field(recording, "files");
        std::set<std::string> names;
        for (const auto& [name, entry] : rr::io::as_object(files, "files")) {
            names.insert(name);
            bt::require(rr::text_of(entry, "sha256").size() == 64 &&
                count(field(entry, "bytes")) > 0,
                "every recorded file carries a digest and size");
        }
        bt::require(names == std::set<std::string>{"manifest.json", "run.duckdb", "summary.md"},
            "the file table is exactly the sealed set");
        const auto& identity = field(manifest, "execution_identity");
        bt::require(count(field(identity, "identity_schema")) >= 2 &&
            rr::text_of(field(field(identity, "build_context"), "platform"), "system") == system,
            "the recorded identity matches the generator host");
    }
}
} // namespace

int main() {
    return bt::run([] {
        inspection_commands();
        inspection_reads_foreign();
        explicit_episode_demanded();
        playback_supports_foreign();
        playback_never_claims_verification();
        failed_runs_playable_not_replayable();
        playback_requires_recipe();
        source_never_mutated();
        directory_move_survives();
        corrupted_assets_fail_closed();
        missing_episode_named();
        foreign_identity_never_checked_replayed();
        legacy_identity_named();
        foreign_identity_named_before_plant();
        unknown_schema_fails_recipe();
        inspection_subprocess_portable();
        fixture_manifests_wellformed();
    });
}
