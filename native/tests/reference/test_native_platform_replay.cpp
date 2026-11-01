// Port of tests/reference/test_native_platform_replay.py — the checked
// replay boundary against the recorded execution identity: a same-host
// recording verifies, a foreign-identity recording is refused with the
// named field before plant construction, recorded-frame playback of the
// same file stays available, and the inspection/playback branch never
// invokes the gate.
//
// Native mapping: the historical build_context.platform.system diagnostic
// path has no counterpart in the native-executable profile — the gate
// (replay::check_execution, run inside the ReplaySession constructor before
// any plant exists) names the first mismatched compared key:
// native_source_sha256, execution_identity, mujoco_library_sha256 or
// numeric_provider. The Python "foreign runtime" direction is the same
// observed comparison with a different compared field mutated; the live
// side cannot be monkeypatched in a statically linked process, so the
// foreign direction is authored on the recorded axis.
//
// 'unverified' marker divergence: the Python CLI prints the word on stderr;
// the native --view --dry-run prints only "frames=... ok" and never claims
// verification — the port asserts the strictest reading (no 'verified'
// marker anywhere) plus a refused checked replay of the identical file.
#include "replay_reference_support.hpp"
#include "platform/process.hpp"
#include "platform/provenance.hpp"

namespace {
namespace rr = replay_reference;
using rr::field;
using rr::set;
using rr::text;

[[nodiscard]] std::filesystem::path replay_binary() {
    // bike-replay lands at <build-root>/bin; this test's binary is four
    // levels down in native/tests/reference.
    return bike::platform::executable_path()
        .parent_path().parent_path().parent_path().parent_path() / "bin" / "bike-replay";
}
// A structurally valid foreign identity: still a 64-hex digest, so the
// recorded profile validates and stays readable — only the gate refuses.
[[nodiscard]] bike::io::Value foreign_execution(const bike::io::Value& execution,
        std::string_view key = "execution_identity") {
    auto patched = execution;
    set(patched, key, text(std::string(64, '0')));
    return patched;
}

// test_native_checked_replay_passes_for_a_same_host_recording — the control
// case: the gate admits the identity this host recorded.
void same_host_checked_replay() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  const auto result = rr::replay::check(publication.directory, 0);
  bt::require(result.matched && rr::flag(result.details, "passed") &&
                  rr::text_of(result.details, "backend") == "native",
              "checked replay of a same-host recording reports passed/native");
}

// test_native_replay_rejects_a_recording_authored_on_another_host — a
// transported recording carries its recorded identity; the session
// constructor names the mismatched field before plant construction, and the
// file stays openable read-only.
void foreign_recording_rejected() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  rr::patch_execution(publication.directory,
                      [](const bike::io::Value &execution) {
                        return foreign_execution(execution);
                      });
  rr::incompatible(
      [&] {
        rr::replay::ReplaySession const session(publication.directory, 0);
      },
      "execution_identity");
  const auto recording = rr::rec::open_recording(publication.directory);
  bt::require(recording->schema_version() == 1 &&
                  !recording->episode_ids().empty(),
              "the rejected recording still opens read-only with its episodes");
}

// test_native_replay_rejects_a_foreign_runtime_with_the_named_field — the
// symmetric direction: a runtime whose provider differs from the record is
// the same field-level mismatch; here the recorded numeric_provider axis is
// foreign so the gate names it.
void foreign_runtime_rejected() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  rr::patch_execution(
      publication.directory, [](const bike::io::Value &execution) {
        auto patched = execution;
        set(patched, "numeric_provider", text("OTHER_PROVIDER"));
        return patched;
      });
  rr::incompatible(
      [&] {
        rr::replay::ReplaySession const session(publication.directory, 0);
      },
      "numeric_provider");
}

// test_native_replay_recheck_still_fires_on_drift_during_replay — the later
// gate cannot be weakened by the construction-time pass: a recording whose
// identity drifts underneath a live session is refused at the next gate
// entry point (restart loads the drifted manifest and runs the identical
// construction gate), while the already-completed session keeps its
// honestly earned certificate.
void drift_recheck_fires() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  rr::replay::ReplaySession session(publication.directory, 0);
  while (!session.advance()) {
  }
    bt::require(rr::flag(session.report(), "passed"),
        "the session completed before the drift");
    rr::patch_execution(publication.directory,
        [](const bike::io::Value& execution) { return foreign_execution(execution); });
    rr::incompatible([&] { session.restart(); }, "execution_identity");
    session.close();
}

// test_native_recorded_playback_supports_the_foreign_recording — the file
// checked replay refuses is still fully playable, and the dry-run output
// never claims verification.
void foreign_recording_still_plays() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  rr::patch_execution(publication.directory,
                      [](const bike::io::Value &execution) {
                        return foreign_execution(execution);
                      });
  const auto playback = rr::replay::load_playback(publication.directory);
  bt::require(playback.times_s.size() >= 2 && playback.duration_s > 0.,
              "the foreign recording still loads playable frames");
  const auto binary = replay_binary();
  bt::require(std::filesystem::exists(binary),
              "the bike-replay artifact is built");
  bike::platform::ProcessOptions options;
  options.timeout_s = 300.;
  const auto viewed = bike::platform::run(
      {binary.string(), publication.directory.string(), "--view", "--dry-run"},
      tmp.path, options);
  bt::require(viewed.exit_code == 0 &&
                  viewed.out.find("frames=") != std::string::npos,
              "the foreign recording still walks its stored trajectory");
  bt::require(viewed.out.find("verified") == std::string::npos &&
                  viewed.err.find("verified") == std::string::npos &&
                  viewed.out.find("passed") == std::string::npos,
              "the dry-run view never claims verification");
  const auto checked = rr::replay::check(publication.directory, 0);
  bt::require(
      !checked.matched &&
          rr::text_of(checked.details, "status") == "incompatible-execution" &&
          checked.reason.find("execution_identity") != std::string::npos,
      "checked replay of the same foreign file still refuses by name");
}

// test_native_verify_machinery_is_never_called_for_inspection_or_playback —
// a foreign-identity recording proves non-invocation observationally: every
// inspection and playback path succeeds where a consulted gate would
// refuse, and only the checked path produces the incompatible-execution
// verdict.
void gate_never_fires_for_inspection_or_playback() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  rr::patch_execution(publication.directory,
                      [](const bike::io::Value &execution) {
                        return foreign_execution(execution);
                      });
  const auto recording = rr::rec::open_recording(publication.directory);
  const auto overview = rr::rec::overview(*recording);
  bt::require(
      !rr::io::as_array(rr::field(rr::field(overview, "episodes"), "rows"),
                        "episode rows")
           .empty(),
      "overview reads the foreign recording");
  const auto queried = recording->query("SELECT count(*) AS n FROM episodes");
  bt::require(
      !rr::io::as_array(rr::field(queried, "rows"), "query rows").empty(),
      "sql inspection reads the foreign recording");
  const auto playback = rr::replay::load_playback(publication.directory);
  bt::require(playback.times_s.size() >= 2,
              "playback loads the foreign recording without the gate");
  const auto binary = replay_binary();
  bike::platform::ProcessOptions options;
  options.timeout_s = 300.;
  const auto viewed = bike::platform::run(
      {binary.string(), publication.directory.string(), "--view", "--dry-run"},
      tmp.path, options);
  bt::require(viewed.exit_code == 0,
              "dry-run inspection never consults the compatibility gate");
  const auto checked = rr::replay::check(publication.directory, 0);
  bt::require(!checked.matched && rr::text_of(checked.details, "status") ==
                                      "incompatible-execution",
              "the checked path alone carries the gate's verdict");
}
} // namespace

int main() {
    return bt::run([] {
        same_host_checked_replay();
        foreign_recording_rejected();
        foreign_runtime_rejected();
        drift_recheck_fires();
        foreign_recording_still_plays();
        gate_never_fires_for_inspection_or_playback();
    });
}
