// Port of tests/reference/test_native_replay_session.py — same-backend
// session lifecycle on a native-produced recording, the execution gate at
// both boundaries, and foreign-identity rejection before plant construction.
//
// Dropped source contract (no native counterpart):
//   test_native_replay_rejects_a_missing_selected_extension — Python selects
//   a nanobind artifact through BIKE_NATIVE_BUILD_PATH; a statically linked
//   native runtime has no extension artifact or build path to consult.
//   Recorded-identity rejection is covered by foreign_identity() below.
#include "replay_reference_support.hpp"

namespace {
namespace rr = replay_reference;
using rr::count;
using rr::field;
using rr::text;

// test_native_recording_replays_same_backend_and_restarts — the legacy
// bundle written from a native run replays on the same backend; restart
// rebuilds identical evidence under the next generation and never reuses
// command streams.
void same_backend_restarts() {
  rr::Directory const tmp;
  const auto path = rr::record_prefix(tmp.path / "native");
  const auto expected = rr::replay::check(path, 0);
  bt::require(expected.matched && rr::flag(expected.details, "passed") &&
                  rr::text_of(expected.details, "backend") == "native",
              "native-produced legacy replay reports the native backend");
  rr::replay::ReplaySession session(path);
  (void)session.advance(0.);
  bt::require(session.pending() && session.snapshot().step == 0,
              "a zero-budget yield leaves the first control pending");
  while (!session.advance(std::nullopt, session.snapshot().step + 1)) {
  }
    bt::require(rr::equal(session.report(), expected.details),
        "incremental report is the headless certificate");
    const auto final = session.snapshot();
    const auto before = final.integration_state;
    session.restart();
    bt::require(session.snapshot().generation == final.generation + 1,
        "restart rebuilds under the next generation");
    while (!session.advance()) {}
    bt::require(rr::equal(session.report(), expected.details),
        "the restarted session reaches the identical certificate");
    bt::require(final.integration_state == before,
        "a captured snapshot is independent of the restarted runtime");
    session.close();
}

// test_native_session_gate_invoked_with_the_recorded_identity — the gate is
// a direct call in the session implementation: construction (before any
// plant exists) and the terminal boundary. A statically linked build cannot
// spy on the call; the observable certificate carries the gate's result.
void gate_invoked_with_recorded_identity() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  rr::replay::ReplaySession session(publication.directory, 0);
  while (!session.advance()) {
  }
    const auto report = session.report();
    bt::require(rr::flag(report, "passed") && rr::text_of(report, "backend") == "native" &&
        rr::count(field(report, "episode_id")) == 0,
        "the completed certificate is produced under the recorded-identity gate");
    session.close();
}

// test_native_session_rejects_foreign_identity_before_any_plant — a
// transported recording names the mismatched identity field; the rejection
// fires in the session constructor before plant/backend/physics exist.
void foreign_identity() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  rr::patch_execution(
      publication.directory, [](const bike::io::Value &execution) {
        auto patched = execution;
        rr::set(patched, "execution_identity",
            text("0000000000000000000000000000000000000000000000000000000000000000"));
        return patched;
      });
  rr::incompatible(
      [&] {
        rr::replay::ReplaySession const session(publication.directory, 0);
      },
      "execution_identity");
}
} // namespace
int main() {
    return bt::run([] {
        same_backend_restarts();
        gate_invoked_with_recorded_identity();
        foreign_identity();
    });
}
