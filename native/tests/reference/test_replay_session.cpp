// Port of tests/reference/test_replay_session.py — incremental ReplaySession
// contracts on the legacy (replay.json) layout: shared checks with headless
// replay, early close, verified-frame rollback, no recorded-policy loading,
// checksum-honest semantic tamper, and the identity gates before plant
// construction / at revalidation boundaries.
#include "replay_reference_support.hpp"

namespace {
namespace rr = replay_reference;
using bike::io::Value;
using rr::text;

// test_python_headless_and_incremental_replay_share_checks (count ∈ {0,2}) —
// the incremental session verifies the same evidence as headless replay.
void headless_incremental(std::size_t controls) {
  rr::Directory const tmp;
  const auto path = rr::record_prefix(tmp.path / "episode", controls);
  const auto expected = rr::replay::check(path, 0);
  rr::replay::ReplaySession session(path);
  if (controls) {
    bt::require(
        !session.advance(0.) && !session.advance(0.),
        "a zero wall-budget advance yields without consuming a control");
    bt::require(session.snapshot().step == 0,
                "zero-budget yields preserve the pending request state");
  }
    while (!session.advance(std::nullopt, session.snapshot().step + 1)) {
        bt::require(!session.done(), "mid-replay states are never terminal");
    }
    const auto report = session.report();
    bt::require(expected.matched && rr::flag(report, "passed"),
        "incremental and headless replay share the same checks");
    bt::require(rr::equal(expected.details, report),
        "incremental report is the headless verification certificate");
    const auto old = session.snapshot();
    const auto held_state = old.integration_state;
    session.restart();
    bt::require(session.snapshot().step == 0 &&
        session.snapshot().generation == old.generation + 1 && !session.done(),
        "restart rebuilds the same evidence under the next generation");
    rr::report_incomplete(session);
    bt::require(old.integration_state == held_state,
        "a captured snapshot is independent of later session state");
    session.close();
}

// test_early_close_never_finishes_a_pending_replay.
void early_close() {
  rr::Directory const tmp;
  const auto path = rr::record_prefix(tmp.path / "episode");
  rr::replay::ReplaySession session(path);
  bt::require(!session.advance(std::nullopt, 1),
              "one control consumes without finishing");
  bt::require(session.snapshot().step == 1 && session.pending(),
              "a pending request survives the committed step");
  bt::require(!rr::flag(session.diagnostics(), "passed") && !session.error(),
              "an incomplete session reports no verified result");
  const auto frame = session.snapshot();
  session.close();
  bt::require(frame.step == 1, "the detached snapshot keeps its step");
  rr::report_incomplete(session);
}

// test_last_frame_remains_unverified_until_final_state_comparison — the
// resealed states.npz edit passes format loading; only the terminal
// comparison notices, and the session rolls back to the verified frame.
void last_frame_unverified() {
  rr::Directory const tmp;
  const auto path = rr::record_prefix(tmp.path / "episode");
  rr::tamper_npz_state(path, "final", 1, .01);
  rr::replay::ReplaySession session(path);
  bt::require(!session.advance(), "the first control still verifies");
  const auto verified = session.snapshot();
  rr::rejects([&] { (void)session.advance(); }, "final integration state");
  const auto &now = session.snapshot();
  bt::require(
      now.step == verified.step && now.generation > verified.generation,
      "a terminal mismatch restores the verified frame under a new generation");
  bt::require(!session.done() && session.error().has_value() &&
                  session.error()->find("final integration state") !=
                      std::string::npos,
              "the failure records the final-state mismatch");
  bt::require(!rr::flag(session.diagnostics(), "passed"),
              "diagnostics stay unverified after the mismatch");
  rr::report_incomplete(session);
  session.close();
}

// test_recorded_policy_reference_is_never_loaded — an absent recorded policy
// reference stays inert text; replay passes without any loader to invoke.
void policy_never_loaded() {
  rr::Directory const tmp;
  const auto path = rr::record_prefix(
      tmp.path / "episode", 2, "recorded_untrusted_missing_module:factory");
  bt::require(rr::replay::check(path, 0).matched,
              "replay never resolves recorded policy references");
}

// test_resealed_semantic_tamper_still_fails_verification — three streams.
void semantic_tamper(std::string_view file, std::size_t index, std::string_view key, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    std::string_view label) {
  rr::Directory const tmp;
  const auto path = rr::record_prefix(tmp.path / "episode");
  rr::tamper_jsonl_row(path, file, index, key);
  rr::replay::ReplaySession session(path);
  rr::rejects(
      [&] {
        while (!session.advance()) {
        }
      },
      label);
  bt::require(!session.done() && session.error().has_value(),
              "a checksum-honest semantic edit fails the comparison");
  bt::require(!rr::flag(session.diagnostics(), "passed"),
              "the session records its failure");
  rr::report_incomplete(session);
  session.close();
}
void semantic_tampers() {
    semantic_tamper("commands_applied.jsonl", 0, "step", "applied commands");
    semantic_tamper("transitions.jsonl", 0, "physics_steps", "transition");
    semantic_tamper("observations.jsonl", 1, "time_s", "observation");
}

// test_identity_mismatch_rejected_before_plant_construction — the recorded
// engine identity mismatching live provenance is rejected by check_execution
// inside the session constructor, which runs before any plant is built.
// (Python patches mujoco_version on the legacy profile; the native-executable
// profile's analogous bound field is mujoco_library_sha256.)
void identity_before_plant() {
  rr::Directory const tmp;
  const auto path = rr::record_prefix(tmp.path / "episode");
  rr::patch_legacy_execution(path, [](const Value &execution) {
    auto patched = execution;
    rr::set(patched, "mujoco_library_sha256",
            text("0000000000000000000000000000000000000000000000000000000000000"
                 "000"));
    return patched;
  });
  rr::incompatible([&] { rr::replay::ReplaySession const session(path); },
                   "mujoco_library_sha256");
}

// test_identity_recheck_failure_advances_no_physics — native revalidation
// fires at each fresh input load; patching the recorded execution while a
// session is live makes restart() reject at that boundary (before plant
// construction) and keep the old verified frame. The live-process provenance
// is immutable in native builds — the Python drift-through-monkeypatch has
// no native patch point — so the post-mismatch no-physics clause is verified
// on a session that recorded a real mid-run mismatch.
void identity_recheck() {
  rr::Directory const tmp;
  const auto path = rr::record_prefix(tmp.path / "episode");
  rr::replay::ReplaySession session(path);
  bt::require(!session.advance(), "one control verifies before the boundary");
  const auto verified = session.snapshot();
  rr::patch_legacy_execution(path, [](const Value &execution) {
    auto patched = execution;
    rr::set(patched, "mujoco_library_sha256",
            text("0000000000000000000000000000000000000000000000000000000000000"
                 "000"));
    return patched;
  });
  rr::incompatible([&] { session.restart(); }, "mujoco_library_sha256");
  bt::require(
      session.snapshot().step == verified.step &&
          session.snapshot().generation == verified.generation &&
          !session.done(),
      "a failed identity revalidation keeps the verified frame, un-advanced");
  session.close();
}
void mismatch_refusal() {
  rr::Directory const tmp;
  const auto path = rr::record_prefix(tmp.path / "episode");
  rr::tamper_jsonl_row(path, "transitions.jsonl", 0, "physics_steps");
  rr::replay::ReplaySession session(path);
  rr::rejects(
      [&] {
        while (!session.advance()) {
        }
      },
      "transition");
  const auto stopped = session.snapshot();
  bt::require(session.error().has_value() && !session.done(),
              "the mismatch is recorded");
  rr::rejects([&] { (void)session.advance(); },
              "replay stopped after mismatch");
  bt::require(session.snapshot().step == stopped.step,
              "post-mismatch advance commits no further physics");
  rr::report_incomplete(session);
  session.close();
}
} // namespace
int main() {
    return bt::run([] {
        headless_incremental(0);
        headless_incremental(2);
        early_close();
        last_frame_unverified();
        policy_never_loaded();
        semantic_tampers();
        identity_before_plant();
        identity_recheck();
        mismatch_refusal();
    });
}
