// Port of tests/reference/test_replay_recording_v3.py — the sealed
// container (contract-3) replay input: explicit episode selection,
// ordinary-recording rejection, recorded policies never loaded, and
// checksum/semantic boundaries in the right order.
#include "replay_reference_support.hpp"

namespace {
namespace rr = replay_reference;
using rr::count;
using rr::field;

// test_new_recording_requires_all_verification_before_report — the loaded
// container input is the contract-3 recipe; the session refuses a report
// until every comparison completes, and the completed report names the
// schema pair it verified under. (The Python read-only-buffer assertion is
// a numpy flag detail: natively the loaded state is an owned copy.)
void requires_all_verification() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  const auto data = rr::replay::load_replay_input(publication.directory);
  bt::require(data.container_schema_version &&
                  *data.container_schema_version == 1 &&
                  data.replay_contract_version == 3 && data.episode_id == 0,
              "container input carries schema-1/contract-3 for episode 0");
  rr::replay::ReplaySession session(publication.directory, 0);
  rr::report_incomplete(session);
  bt::require(!session.advance(0.) && session.pending() &&
                  session.snapshot().step == 0,
              "zero-budget yield keeps one accepted pending control without "
              "integrating");
  while (!session.advance()) {
  }
    const auto result = session.report();
    bt::require(rr::flag(result, "passed") && rr::count(field(result, "replay_contract_version")) == 3 &&
        rr::count(field(result, "container_schema_version")) == 1,
        "saved evidence passes only after transitions, observations, metrics, outcome and final state agree");
    session.close();
    bt::require(rr::replay::check(publication.directory, 0).matched,
        "headless checked replay passes the same evidence");
}

// test_multi_episode_selection_is_explicit — both episodes replay, a bare
// constructor refuses to guess, and an absent id names itself.
void multi_episode() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path, true);
  rr::rejects(
      [&] { rr::replay::ReplaySession const session(publication.directory); },
      "episode");
  for (std::int64_t episode = 0; episode < 2; ++episode)
    bt::require(rr::replay::check(publication.directory, episode).matched,
                "each recorded episode replays when selected explicitly");
  rr::rejects(
      [&] { (void)rr::replay::load_replay_input(publication.directory, 2); },
      "no episode");
}

// test_ordinary_recording_does_not_claim_checked_replay — an ordinary
// physical capture has no research contract to verify against.
void ordinary_rejected() {
  rr::Directory const tmp;
  const auto publication = rr::record_ordinary(tmp.path);
  rr::recording_error(
      [&] { (void)rr::replay::load_replay_input(publication.directory); },
      "ordinary physical");
}

// test_new_replay_does_not_load_recorded_policy — an absent recorded
// policy reference stays inert metadata: replay passes without any policy
// loader existing to resolve it.
void policy_never_loaded() {
  rr::Directory const tmp;
  const auto publication =
      rr::record_container(tmp.path, false, "absent_untrusted_policy:factory");
  bt::require(
      rr::replay::check(publication.directory, 0).matched,
      "recorded policy references are never resolved during checked replay");
}

// test_resealed_final_state_tamper_fails_last_boundary_not_format_loading —
// format loading stays honest (the manifest digest is recomputed); only the
// terminal integration-state comparison can see the edit, and the session
// rolls back to the last verified frame.
void final_state_boundary() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  rr::tamper_final_state(publication.directory);
  rr::replay::ReplaySession session(publication.directory, 0);
  // The verified checkpoint is the frame after the last non-final
  // transition; capture it from the last iteration before the throw.
  runtime::RuntimeSnapshot verified;
  bool verified_once = false;
  try {
    while (!session.advance()) {
      verified = session.snapshot();
      verified_once = true;
    }
    bt::require(false, "a tampered final state must reject");
  } catch (const std::exception &failure) {
    bt::require(
        std::string_view(failure.what()).find("final integration state") !=
            std::string_view::npos,
        "the terminal boundary names the final integration state");
  }
    const auto diagnostics = session.diagnostics();
    bt::require(!session.done() && session.error().has_value() &&
        !rr::flag(diagnostics, "passed"),
        "the session records the failed final boundary");
    bt::require(verified_once && session.snapshot().step == verified.step &&
        session.snapshot().generation > verified.generation,
        "a terminal mismatch restores the last verified frame under a new generation");
    rr::report_incomplete(session);
    session.close();
}

// test_new_checksum_failure_precedes_plant_construction — appended bytes
// change the payload digest, so the manifest check fires before any model
// or runtime exists (the session constructor itself rejects).
void checksum_before_plant() {
  rr::Directory const tmp;
  const auto publication = rr::record_container(tmp.path);
  {
    std::ofstream out(publication.directory / "run.duckdb",
                      std::ios::binary | std::ios::app);
    out << "not the recorded checksum";
  }
  rr::rejects(
      [&] {
        rr::replay::ReplaySession const session(publication.directory, 0);
      },
      "checksum");
}

// The battery energy model was removed; legacy recordings still carry a
// resolved drive.battery section. The shipped disabled section is inert and
// replays, but an enabled one could have clamped assist torque once its
// energy ran out, so the legacy-bundle rebuild refuses it.
void removed_battery_gate() {
    auto input = rr::replay::load_legacy_input(bt::fixture("replay/legacy_v2"));
    const auto& recorded_drive = field(field(field(input.summary, "resolved_config"), "physics"), "drive");
    bt::require(rr::io::find_member(rr::io::as_object(recorded_drive, "drive"), "battery") != nullptr,
                "the legacy fixture still records a drive.battery section");
    {
        const rr::replay::FixedAssets assets(input);
        bt::require(
            static_cast<bool>(rr::replay::request_from_input(input, assets)),
            "a recorded disabled battery is inert and replays");
    }
    auto resolved = field(input.summary, "resolved_config");
    auto physics = field(resolved, "physics");
    auto drive = field(physics, "drive");
    auto battery = field(drive, "battery");
    rr::set(battery, "enabled", rr::io::Value{true});
    rr::set(drive, "battery", std::move(battery));
    rr::set(physics, "drive", std::move(drive));
    rr::set(resolved, "physics", std::move(physics));
    rr::set(input.summary, "resolved_config", std::move(resolved));
    const rr::replay::FixedAssets assets(input);
    rr::incompatible(
        [&] { (void)rr::replay::request_from_input(input, assets); },
        "drive.battery");
}
} // namespace
int main() {
    return bt::run([] {
        requires_all_verification();
        multi_episode();
        ordinary_rejected();
        policy_never_loaded();
        final_state_boundary();
        checksum_before_plant();
        removed_battery_gate();
    });
}
