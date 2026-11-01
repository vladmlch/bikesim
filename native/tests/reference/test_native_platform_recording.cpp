// Port of tests/reference/test_native_platform_recording.py — the
// cross-platform run-recording qualification against the real storage
// seam. The file's viewer-pump records are Python frontend mechanics;
// their seal/publish/interrupt contracts are already landed by the
// native headless twins in test_native_recording_lifecycle.cpp and are
// recorded as covered-by in the ledger. The records ported here are the
// ones that qualify filesystem behavior itself: exclusive-name
// collision, crash staging, EACCES-and-retry, the spaced-unicode output
// path, the deep real-run payload shape, the strict invalid prefix, and
// the research publication contract.
#include "../support/migration_test.hpp"
#include "../support/temporary_directory.hpp"
#include "recording_test_support.hpp"
#include "replay_reference_support.hpp"
#include "cli/application.hpp"
#include "cli/arguments.hpp"
#include "recording/publish.hpp"
#include "recording/reader.hpp"
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#ifdef __linux__
#include <grp.h>  // glibc declares setgroups here, not in unistd.h
#endif
#include <pwd.h>
#include <set>

namespace {
namespace rt = recording_test;
namespace rr = replay_reference;
namespace rec = bike::recording;
namespace cli = bike::cli;
using bike::io::Value;
using runtime::SamplePtr;

const Value& req(const Value::Object& object, std::string_view key) {
    const Value* slot = rt::member(object, key);
    // The guard must be visible in this translation unit: bt::require's
    // throw lives in the harness TU, so the analyzer cannot prove the
    // dereference below is unreachable on the null path. The failure mode
    // is identical — std::runtime_error with the same message.
    if (slot == nullptr)
        throw std::runtime_error("member ." + std::string(key) + " is present");
    return *slot;
}
const Value& reqv(const Value& value, std::string_view key) {
    return req(rt::as_object(value), key);
}
void assert_published(const std::filesystem::path& dir) {
    for (const auto& name : {"run.duckdb", "manifest.json", "summary.md"})
        bt::require(std::filesystem::is_regular_file(dir / name),
            std::string("the sealed run carries ") + name);
}
[[nodiscard]] std::vector<std::string> event_kinds(const rec::Recording& saved) {
    std::vector<std::string> out;
    const std::array params{rr::integer(0)};
    for (const auto& row : saved.rows(
            "SELECT kind FROM events WHERE episode_id=? ORDER BY event_id",
            params))
        out.push_back(rt::as_text(req(row, "kind")));
    return out;
}
[[nodiscard]] Value summary_of(const rec::Recording& saved) {
    return bike::io::parse_recording_json(
        rt::as_text(reqv(saved.episode(), "summary_json")));
}
[[nodiscard]] std::filesystem::path published_run(const std::filesystem::path& root) {
    std::filesystem::path found;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(root))
        if (entry.path().filename() == "manifest.json") {
            bt::require(found.empty(), "exactly one run was published");
            found = entry.path().parent_path();
        }
    bt::require(!found.empty(), "a run was published");
    return found;
}
[[nodiscard]] cli::Arguments headless_args(const std::filesystem::path& physics,
        const std::filesystem::path& out, bool record,
        const char* duration = "0.01", const char* policy = "diagnostic") {
    const auto track = rt::repo_file("examples/research/rough_uphill_savage.toml");
    const auto physics_str = physics.string(), track_str = track.string(),
        out_str = out.string();
    std::vector<std::string_view> args{
        "--physics-config", physics_str, "--track", track_str,
        "--headless", "--duration", duration, "--no-plots",
        "--check-policy", policy, "--decimate", "1", "--out", out_str};
    if (record) args.emplace_back("--record");
    return cli::parse_arguments("bike-ride", args);
}

// test_exclusive_names_collide_without_overwrite — the exclusive
// destination loop suffixes rather than adopting or overwriting, even a
// pre-existing empty collision.
[[gnu::noinline]] void exclusive_names_collide() {
  bt::TemporaryDirectory const root("platform-recording-collide");
  const std::string base =
      "20260101T000000.000000Z-00000000-0000-4000-8000-000000000001";
  const auto squatter = root.path() / base;
  std::filesystem::create_directories(squatter);
  auto first_capture = rt::synthetic_capture({rt::sample_row(0, 0., .001)},
                                             rt::SyntheticCapture{});
  const auto first = rec::finalize(*first_capture, root.path());
  bt::require(first.directory.filename() == base + "-1",
              "the squatter is skipped to the first suffix, never adopted");
  bt::require(std::filesystem::is_empty(squatter),
              "somebody else's directory is never touched");
  auto second_capture = rt::synthetic_capture({rt::sample_row(0, 0., .001)},
                                              rt::SyntheticCapture{});
  const auto second = rec::finalize(*second_capture, root.path());
  bt::require(second.directory.filename() == base + "-2",
              "the second publication takes the next free suffix");
  for (const auto &dir : {first.directory, second.directory}) {
    assert_published(dir);
    const auto saved = rec::open_recording(dir);
    bt::require(saved->episode_ids() == std::vector<std::int64_t>{0},
                "each suffixed run reads back its episode");
  }
}

// test_finalization_crash_leaves_no_readable_run — a process dying
// between payload link and manifest link leaves a destination the
// reader refuses and an unsealed staging directory carrying its
// UNSEALED marker.
[[gnu::noinline]] void crash_leaves_no_readable_run() {
  bt::TemporaryDirectory const root("platform-recording-crash");
  const auto pid = ::fork();
  bt::require(pid >= 0, "fork for the crash probe");
  if (pid == 0) {
    auto capture = rt::synthetic_capture({rt::sample_row(0, 0., .001)},
                                         rt::SyntheticCapture{});
    rec::FinalizeOptions options;
    options.before_stage = [](std::string_view stage) {
      if (stage == "manifest_link")
        ::_exit(111);
    };
    (void)rec::finalize(*capture, root.path(), options);
    ::_exit(112); // publication completed — must never happen
  }
    int status = 0;
    bt::require(::waitpid(pid, &status, 0) == pid && WIFEXITED(status) &&
        WEXITSTATUS(status) == 111, "the child dies inside the manifest link");
    for (const auto& entry :
            std::filesystem::recursive_directory_iterator(root.path()))
        bt::require(entry.path().filename() != "manifest.json",
            "no readable run was published: " + entry.path().string());
    bool unsealed_seen = false, rejected_seen = false;
    for (const auto& entry :
            std::filesystem::recursive_directory_iterator(root.path())) {
        if (entry.path().filename() != "run.duckdb") continue;
        const auto& dir = entry.path().parent_path();
        if (dir.filename().string().starts_with(".run-recording-unsealed-")) {
            unsealed_seen = true;
            bt::require(std::filesystem::is_regular_file(dir / "UNSEALED"),
                "surviving staging keeps its UNSEALED marker");
        } else {
            bool rejected = false;
            try { (void)rec::open_recording(dir); }
            catch (const std::exception&) { rejected = true; }
            bt::require(rejected, "a payload-only directory is not a run");
            rejected_seen = true;
        }
    }
    bt::require(unsealed_seen && rejected_seen,
        "the crash leaves both the staging marker and the refused destination");
}

// The EACCES probe as a plain function — shared by the unprivileged
// direct path and the root-nobody child.
[[gnu::noinline]] bool eacces_probe(const std::filesystem::path& parent) {
    const auto denied = parent / "denied";
    std::filesystem::create_directories(denied);
    auto capture = rt::synthetic_capture(
        {rt::sample_row(0, 0., .001)}, rt::SyntheticCapture{});
    std::filesystem::permissions(denied,
        std::filesystem::perms::owner_read | std::filesystem::perms::owner_exec,
        std::filesystem::perm_options::replace);
    bool refused = false;
    try { (void)rec::finalize(*capture, denied); }
    catch (const std::exception&) { refused = true; }
    if (!refused || capture->state() != rec::RunCapture::State::publication_failed)
        return false;
    if (!std::filesystem::is_empty(denied)) return false;
    std::filesystem::permissions(denied, std::filesystem::perms::owner_all,
        std::filesystem::perm_options::replace);
    const auto published = rec::finalize(*capture, denied);
    const auto saved = rec::open_recording(published.directory);
    return saved->episode_ids() == std::vector<std::int64_t>{0};
}

// test_publication_permission_denied_then_retry — EACCES fails closed
// into publication_failed and an explicit retry publishes once. Under
// euid 0 the mode bits are bypassed, so the probe runs in a child that
// drops to nobody first (the oracle's setuid twin).
[[gnu::noinline]] void permission_denied_then_retry() {
  bt::TemporaryDirectory const root("platform-recording-eacces");
  if (::geteuid() != 0) {
    bt::require(eacces_probe(root.path()),
                "EACCES fails closed and the retry publishes exactly once");
    return;
  }
    const auto* account = ::getpwnam("nobody"); // NOLINT(concurrency-mt-unsafe) test environment mutation runs single-threaded by construction
    bt::require(account != nullptr, "the nobody account exists for the drop");
    const auto sandbox = root.path() / "unprivileged";
    std::filesystem::create_directories(sandbox);
    std::filesystem::permissions(sandbox, std::filesystem::perms::all,
        std::filesystem::perm_options::replace);
    const auto pid = ::fork();
    bt::require(pid >= 0, "fork for the nobody probe");
    if (pid == 0) {
        if (::setgroups(0, nullptr) != 0 || ::setgid(account->pw_gid) != 0 ||
            ::setuid(account->pw_uid) != 0) {
            ::_exit(4);
        }
        if (::geteuid() == 0) ::_exit(3);
        ::_exit(eacces_probe(sandbox) ? 0 : 1);
    }
    int status = 0;
    bt::require(::waitpid(pid, &status, 0) == pid && WIFEXITED(status) &&
        WEXITSTATUS(status) == 0, "the unprivileged EACCES probe passed");
}

// test_headless_native_record_publishes_real_run — a real native
// --record ride publishes the deep payload: contiguous physics rows,
// the honest invalid verdict, schema-2 digests, source assets and the
// channel tables.
[[gnu::noinline]] void real_run_publishes_deep_payload() {
  bt::TemporaryDirectory const root("platform-recording-run");
  const auto physics = rt::violating_physics(root.path() / "physics.toml");
  const auto args = headless_args(physics, root.path() / "runs", true);
  bt::require(cli::ride_command(args) == 2,
              "the violating recipe reports its honest exit code");
  const auto run_dir = published_run(root.path() / "runs");
  assert_published(run_dir);
  const auto saved = rec::open_recording(run_dir);
  bt::require(saved->episode_ids() == std::vector<std::int64_t>{0},
              "one sealed episode");
  const auto meta = saved->metadata();
  bt::require(rt::as_text(reqv(meta, "backend")) == "native",
              "the run metadata is the native launch");
  const auto summary = summary_of(*saved);
  bt::require(rt::as_text(reqv(reqv(summary, "outcome"), "reason")) ==
                  "duration_reached",
              "the diagnostic ride reaches the duration");
  bt::require(!rt::as_bool(reqv(reqv(summary, "model_status"), "model_valid")),
              "the weld-envelope violation keeps the model invalid");
  const auto &failure =
      bike::io::as_array(reqv(summary, "first_failure"), "first_failure");
  bt::require(failure.size() == 2,
              "first_failure carries its measured violations");
  const auto rows = saved->rows("SELECT min(interval_id) lo, max(interval_id) "
                                "hi, count(*) n FROM physics");
  const auto physical = rt::as_integer(reqv(saved->episode(), "physical_rows"));
  bt::require(physical > 0 &&
                  rt::as_integer(req(rows.at(0), "n")) == physical &&
                  rt::as_integer(req(rows.at(0), "hi")) == physical - 1,
              "physics rows are the contiguous committed prefix");
  // The native execution record carries the executable/provider identity
  // keys (research::physical_configuration_metadata), not the Python
  // extension's identity_schema shape — the divergence is ledgered.
  const auto &execution = rt::as_object(reqv(summary, "execution"));
  bt::require(rt::as_text(req(execution, "backend")) == "native" &&
                  !rt::as_text(req(execution, "numeric_provider")).empty(),
              "the native execution identity is sealed");
  for (const char *key : {"execution_identity", "mujoco_library_sha256",
                          "native_source_sha256", "executable_sha256"})
    bt::require(rt::as_text(req(execution, key)).size() == 64,
                std::string(key) + " is a 64-hex digest");
  // Native journals emit "episode_started" for physical rides (the
  // Python journal names it "episode_start") — a naming divergence the
  // ledger records; the boundary contract is identical.
  const auto kinds = event_kinds(*saved);
  bt::require(!kinds.empty() && kinds.front() == "episode_started",
              "the journal opens with the episode boundary");
  std::set<std::string> assets;
  for (const auto &row : saved->rows("SELECT name FROM assets"))
    assets.insert(rt::as_text(req(row, "name")));
  bt::require(assets.count("track.toml") &&
                  assets.count("resolved_config.json"),
              "typed source assets are bundled with the episode");
  for (const char *table :
       {"channel_tires", "channel_energy", "channel_control"})
    bt::require(
        rt::as_integer(req(
            saved->rows(std::string("SELECT count(*) n FROM ") + table).at(0),
            "n")) > 0,
        std::string(table) + " carries rows");
}

// test_headless_native_record_strict_stop_keeps_invalid_prefix — the
// strict policy stops inside the duration and publishes the committed
// prefix under its real invalid outcome.
[[gnu::noinline]] void strict_stop_keeps_invalid_prefix() {
  bt::TemporaryDirectory const root("platform-recording-strict");
  const auto physics = rt::violating_physics(root.path() / "physics.toml");
  const auto args =
      headless_args(physics, root.path() / "runs", true, "0.01", "strict");
  // ride_command rethrows the primary strict rejection after sealing and
  // publishing; the bike-ride wrapper is what maps it to exit 2.
  std::string primary;
  try {
    (void)cli::ride_command(args);
  } catch (const std::exception &error) {
    primary = error.what();
  }
    bt::require(primary.find("foot_front.normal") != std::string::npos ||
        primary.find("invalid") != std::string::npos,
        "the strict rejection propagates after publishing: " + primary);
    const auto run_dir = published_run(root.path() / "runs");
    const auto saved = rec::open_recording(run_dir);
    const auto summary = summary_of(*saved);
    const auto& outcome = rt::as_object(reqv(summary, "outcome"));
    // Native ride.cpp seals an advance-level rejection as
    // "simulation_error" (the Python frontend names the same contract
    // "invalid_controller") — the honest-prefix contract is identical
    // and the naming divergence is ledgered.
    bt::require(rt::as_text(req(outcome, "reason")) == "simulation_error",
        "the strict outcome keeps its real invalid reason");
    const auto time_s = rt::as_real(req(outcome, "time_s"));
    const auto& failure = bike::io::as_array(reqv(summary, "first_failure"),
        "first_failure");
    bt::require(rt::as_real(failure.at(0)) <= time_s && time_s < .01,
        "the measured failure lands inside the requested duration");
    const auto rows = rt::as_integer(reqv(saved->episode(), "physical_rows"));
    bt::require(rows > 0 && rows < 16,
        "the committed prefix is honest — never lengthened");
}

// test_headless_native_record_off_with_out_writes_nothing — --out is a
// location, never an opt-in.
[[gnu::noinline]] void record_off_writes_nothing() {
  bt::TemporaryDirectory const root("platform-recording-off");
  const auto physics = rt::violating_physics(root.path() / "physics.toml");
  const auto args = headless_args(physics, root.path() / "runs", false);
  bt::require(cli::ride_command(args) == 2,
              "the run still reports the violation");
  bt::require(!std::filesystem::exists(root.path() / "runs") ||
                  std::filesystem::is_empty(root.path() / "runs"),
              "no run files exist without --record");
}

// test_headless_native_record_under_spaced_unicode_path — publication
// and read-back survive spaces and non-ASCII in the --out path.
[[gnu::noinline]] void spaced_unicode_path() {
  bt::TemporaryDirectory const root("platform-recording-unicode");
  const auto physics = rt::violating_physics(root.path() / "physics.toml");
  const auto out = root.path() / "runs dir" / "ünïcodé ✓";
  const auto args = headless_args(physics, out, true);
  bt::require(cli::ride_command(args) == 2, "the unicode path ride completes");
  const auto run_dir = published_run(out);
  const auto saved = rec::open_recording(run_dir);
  bt::require(saved->episode_ids() == std::vector<std::int64_t>{0} &&
                  rt::as_integer(reqv(saved->episode(), "physical_rows")) > 0,
              "the unicode-path run reads back");
}

// test_research_native_run_publishes_sealed_recording +
// test_research_policy_failure_after_capture_publishes_evidence — the
// real research publication: a completed episode is checked-replay
// eligible (contract v3, research tables); a failed launch publishes
// its evidence and is never eligible.
[[gnu::noinline]] void research_run_sealed_and_failed() {
  bt::TemporaryDirectory const root("platform-recording-research");
  const auto publication = rr::record_container(root.path());
  assert_published(publication.directory);
  const auto saved = rec::open_recording(publication.directory);
  const auto episode_row = saved->episode();
  const auto &episode = rt::as_object(episode_row);
  bt::require(rt::as_integer(req(episode, "replay_contract_version")) == 3 &&
                  rt::as_bool(req(episode, "replay_eligible")) &&
                  rt::as_integer(req(episode, "physical_rows")) > 0,
              "the completed research episode is checked-replay eligible");
  for (const char *table : {"commands_requested", "commands_applied",
                            "observations", "transitions"})
    bt::require(
        rt::as_integer(req(
            saved->rows(std::string("SELECT count(*) n FROM ") + table).at(0),
            "n")) > 0,
        std::string(table) + " carries research evidence");
  saved->close();
  const auto failed = rr::record_failed(root.path());
  const auto rejected = rec::open_recording(failed.directory);
  const auto failed_row = rejected->episode();
  const auto &failed_episode = rt::as_object(failed_row);
  bt::require(!rt::as_bool(req(failed_episode, "replay_eligible")),
              "the failed launch is published, never replay eligible");
}
} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        exclusive_names_collide();
        crash_leaves_no_readable_run();
        permission_denied_then_retry();
        real_run_publishes_deep_payload();
        strict_stop_keeps_invalid_prefix();
        record_off_writes_nothing();
        spaced_unicode_path();
        research_run_sealed_and_failed();
    });
}
