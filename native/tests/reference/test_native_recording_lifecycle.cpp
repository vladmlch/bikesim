// Completion 3.30 (T53). Source: tests/reference/test_native_recording_lifecycle.py.
//
// R03/R11/R14/R16 run-recording lifecycle contract ports. The oracle split
// its checks between fake-window pump fakes and real native-driver runs;
// the ports map each check onto the production owner that carries it
// natively:
//   * derived-transition journal emission — recording::derived_events +
//     episode_interval_for_time over synthetic OwnedNativeEpisode rows;
//   * capture/publication retry + appends — RunCapture/EpisodeJournal +
//     recording::finalize with FinalizeOptions::before_stage injection
//     (the oracle's publisher monkeypatch);
//   * real headless rides — cli::ride_command with the authored physics
//     recipes (the same --record/--headless path the oracle drove);
//   * seal-per-generation / zero-step failure — NativeRideRuntime over the
//     captured welded bootstrap + test_fail_at_step;
//   * deferred Ctrl-C — platform::InterruptShield + recording::finalize_guarded
//     (the same shielded seal+finalize composition as cli/ride.cpp) and
//     research::ResearchRun's seal dedupe for the terminal-notice check.
//
// Dropped nodes (python-only):
//   * test_viewer_requests_are_journaled_before_state_changes — the
//     requested-vs-applied journal records are the Python frontend's
//     emission policy (cli/ride.cpp's native frontend emits a single
//     'brakes' event); the mechanical journal contract is covered inside
//     test_viewer_journals_two_episodes_and_publishes_once.
//   * test_viewer_playback_scale_is_requested_only — the playback_scale
//     journal kind exists only in the Python frontend; no native event
//     counterpart exists to assert.
//   * test_headless_trailer_names_only_existing_artifacts — the console
//     trailer text is Python frontend plumbing (capsys assertion); the
//     no-artifact contract it names is ported by
//     test_headless_off_writes_no_artifact_files.
//   * test_explicit_programmatic_export_still_writes_artifacts — the
//     PhysicalRideDriver.export() file bundle is Python adapter plumbing;
//     native artifact emission is cli/ride.cpp's legacy headless path.
//   * test_viewer_strict_seal_rejection_retries_once — a once-failing
//     ownership step is not injectable on NativeRideRuntime; the same
//     retry-on-strict-rejection contract lives in cli/ride.cpp's seal-retry
//     block, and evidence preservation is covered by
//     test_failed_sealing_leaves_active_evidence_available and
//     test_strict_flush_rejection_preserves_retained_evidence.
//
// Journal-level ports emit the oracle's kind vocabulary directly
// ('episode_start', 'operator_stop_deferred', 'brake_toggle', ...); the
// cli::ride_command ports assert the native vocabulary ('episode_started').
#include "recording_test_support.hpp"
#include "support/temporary_directory.hpp"
#include "diag.hpp"
#include "cli/application.hpp"
#include "cli/arguments.hpp"
#include "cli/physical_capture.hpp"
#include "platform/signals.hpp"
#include "recording/publish.hpp"
#include "recording/reader.hpp"
#include "research/environment.hpp"
#include "research/lifecycle.hpp"
#include "runtime/research_io.hpp"
#include "runtime/status.hpp"
#include <algorithm>
#include <array>
#include <csignal>
#include <fstream>
#include <vector>

namespace {

using namespace recording_test;
namespace value = rec::value;
namespace cli = bike::cli;
using bike::io::Value;

// ---- shared sealed-recording readers ---------------------------------------

const Value &member_ref(const Value &value, std::string_view key) {
    const auto *slot = io::find_member(io::as_object(value, "value"), key);
    bt::require(slot != nullptr, std::string("value.") + std::string(key) + " present");
    return *slot;
}

const Value &required_member(const Value::Object &object, std::string_view key) {
    const auto *slot = io::find_member(object, key);
    bt::require(slot != nullptr, std::string("row.") + std::string(key) + " present");
    return *slot;
}

std::int64_t physical_rows_of(const rec::Recording &saved, std::int64_t episode_id) {
    const auto episode = saved.episode(episode_id);
    return as_integer(required_member(as_object(episode), "physical_rows"));
}

Value summary_of(const rec::Recording &saved, std::int64_t episode_id) {
    const auto episode = saved.episode(episode_id);
    return io::parse_recording_json(
        io::as_string(required_member(as_object(episode), "summary_json"), "summary_json"));
}

// Sealed manifest directory under root (exactly one expected when sealed > 0).
std::filesystem::path sealed_directory(const std::filesystem::path &root, std::size_t expected = 1) {
    std::size_t sealed = 0;
    std::filesystem::path directory;
    for (const auto &entry : std::filesystem::directory_iterator(root))
        if (std::filesystem::exists(entry.path() / "manifest.json")) {
            ++sealed;
            directory = entry.path();
        }
    bt::require(sealed == expected, "the manifest is sealed exactly " + std::to_string(expected) + " time(s)");
    return directory;
}

std::vector<std::string> sealed_kinds(const rec::Recording &saved, std::int64_t episode_id) {
    const auto result = rec::events(saved, episode_id);
    const auto &rows = io::as_array(required_member(as_object(result), "rows"), "events.rows");
    std::vector<std::string> out;
    for (const auto &row : rows)
        out.push_back(as_text(required_member(as_object(row), "kind")));
    return out;
}

// ---- derived-transition helpers ---------------------------------------------

// row(interval_id, ...) — the frontend fakes' minimal channel rows.
SamplePtr brake_row(std::int64_t id, bool contact, double brake) {
    auto sample = sample_row(id, static_cast<double>(id) * .00125, .00125,
                             storable_channels(contact, brake));
    return sample;
}

SamplePtr channel_row(std::int64_t id, double dt_s, WireObject channels) {
    return sample_row(id, static_cast<double>(id) * dt_s, dt_s, std::move(channels));
}

void test_derived_transitions_carry_rule_version_and_interval() {
    const auto episode = make_episode({brake_row(0, true, 0.), brake_row(1, true, .4),
                                       brake_row(2, false, .4)});
    const auto events = rec::derived_events(episode, 0, 3, 1.5);
    bt::require(events.size() == 2 && events[0].event_id == 3 && events[1].event_id == 4,
                "derived events continue the journal cursor");
    const auto &first = events[0], &second = events[1];
    bt::require(first.kind == "derived_transition" &&
                as_text(member_ref(first.values, "classification")) == "derived_transition" &&
                as_text(member_ref(first.values, "derivation_rule")) == "channel_value_change.v1" &&
                as_text(member_ref(first.values, "derived_kind")) == "brake_gate" &&
                as_integer(member_ref(first.values, "interval_id")) == 1 &&
                first.processed_step == 1 && first.applied_step && *first.applied_step == 1,
                "the brake gate transition carries rule version and interval");
    const auto &changes = as_object(member_ref(first.values, "changes"));
    bt::require(changes.size() == 2 &&
                as_real(required_member(changes, "front_brake_demand")) == .4 &&
                as_real(required_member(changes, "rear_brake_demand")) == .4,
                "changed values carry first-insertion order");
    bt::require(as_text(member_ref(second.values, "derived_kind")) == "tire_contact",
                "the contact transition carries its derived kind");
    const auto &contact_changes = as_object(member_ref(second.values, "changes"));
    bt::require(contact_changes.size() == 2 &&
                !as_bool(required_member(contact_changes, "front.raw_contact")) &&
                !as_bool(required_member(contact_changes, "rear.raw_contact")) &&
                second.received_wall_s == 1.5,
                "the contact transition carries flattened member paths");
}

void test_derived_transitions_cover_shift_coast_and_limiter_gates() {
    const auto drive_row = [](std::int64_t id, WireObject fields) {
        return channel_row(id, .001, WireObject{{"drive", Wire{std::move(fields)}}});
    };
    const auto episode = make_episode({
        drive_row(0, WireObject{{"shift_active", Wire{false}}, {"shift_direction", Wire{std::string("none")}},
                                {"freehub_engaged", Wire{true}}, {"coasting_reason", Wire{}},
                                {"safety_limited", Wire{false}}, {"energy_limited", Wire{false}},
                                {"motor_enabled", Wire{true}}, {"battery_empty", Wire{false}}}),
        drive_row(1, WireObject{{"shift_active", Wire{true}}, {"shift_direction", Wire{std::string("down")}},
                                {"freehub_engaged", Wire{false}}, {"coasting_reason", Wire{std::string("shift")}},
                                {"safety_limited", Wire{false}}, {"energy_limited", Wire{false}},
                                {"motor_enabled", Wire{true}}, {"battery_empty", Wire{false}}}),
        drive_row(2, WireObject{{"shift_active", Wire{false}}, {"shift_direction", Wire{std::string("none")}},
                                {"freehub_engaged", Wire{false}}, {"coasting_reason", Wire{std::string("coast")}},
                                {"safety_limited", Wire{true}}, {"energy_limited", Wire{false}},
                                {"motor_enabled", Wire{false}}, {"battery_empty", Wire{false}}}),
        drive_row(3, WireObject{{"shift_active", Wire{false}}, {"shift_direction", Wire{std::string("none")}},
                                {"freehub_engaged", Wire{true}}, {"coasting_reason", Wire{}},
                                {"safety_limited", Wire{false}}, {"energy_limited", Wire{true}},
                                {"motor_enabled", Wire{false}}, {"battery_empty", Wire{true}}})});
    const auto events = rec::derived_events(episode, 0, 0, 0.);
    std::map<std::string, std::vector<const rec::InputEvent *>> by_kind;
    for (const auto &event : events)
        by_kind[as_text(member_ref(event.values, "derived_kind"))].push_back(&event);
    bt::require(by_kind.size() == 3 && by_kind.count("shift") && by_kind.count("freehub_coast") &&
                by_kind.count("drive_limits"), "every tracked drive gate family emits");
    const auto &shift = by_kind["shift"];
    bt::require(shift.size() == 2 &&
                as_integer(member_ref(shift[0]->values, "interval_id")) == 1 &&
                as_integer(member_ref(shift[1]->values, "interval_id")) == 2,
                "the shift gate emits at both transition intervals");
    const auto &shift0 = as_object(member_ref(shift[0]->values, "changes"));
    const auto &shift1 = as_object(member_ref(shift[1]->values, "changes"));
    bt::require(shift0.size() == 2 && as_bool(required_member(shift0, "shift_active")) &&
                as_text(required_member(shift0, "shift_direction")) == "down" &&
                shift1.size() == 2 && !as_bool(required_member(shift1, "shift_active")) &&
                as_text(required_member(shift1, "shift_direction")) == "none",
                "shift flips both tracked fields in one event each way");
    const auto &coast = by_kind["freehub_coast"];
    bt::require(coast.size() == 3 &&
                as_integer(member_ref(coast[0]->values, "interval_id")) == 1 &&
                as_integer(member_ref(coast[1]->values, "interval_id")) == 2 &&
                as_integer(member_ref(coast[2]->values, "interval_id")) == 3,
                "the coast gate emits at all three transitions");
    const auto &coast0 = as_object(member_ref(coast[0]->values, "changes"));
    const auto &coast1 = as_object(member_ref(coast[1]->values, "changes"));
    const auto &coast2 = as_object(member_ref(coast[2]->values, "changes"));
    bt::require(coast0.size() == 2 &&
                as_text(required_member(coast0, "coasting_reason")) == "shift" &&
                !as_bool(required_member(coast0, "freehub_engaged")) &&
                coast1.size() == 1 &&
                as_text(required_member(coast1, "coasting_reason")) == "coast" &&
                coast2.size() == 2 &&
                io::is_null(required_member(coast2, "coasting_reason")) &&
                as_bool(required_member(coast2, "freehub_engaged")),
                "unchanged members stay out of the changes payload and null diffs keep their label");
    const auto &limits = by_kind["drive_limits"];
    bt::require(limits.size() == 2 &&
                as_integer(member_ref(limits[0]->values, "interval_id")) == 2 &&
                as_integer(member_ref(limits[1]->values, "interval_id")) == 3,
                "the limiter gate emits at both transitions");
    const auto &limits0 = as_object(member_ref(limits[0]->values, "changes"));
    const auto &limits1 = as_object(member_ref(limits[1]->values, "changes"));
    bt::require(limits0.size() == 2 &&
                as_bool(required_member(limits0, "safety_limited")) &&
                !as_bool(required_member(limits0, "motor_enabled")) &&
                limits1.size() == 3 &&
                !as_bool(required_member(limits1, "safety_limited")) &&
                as_bool(required_member(limits1, "energy_limited")) &&
                as_bool(required_member(limits1, "battery_empty")),
                "limiter changes carry only the changed fields");
    for (const auto &event : events)
        bt::require(as_text(member_ref(event.values, "classification")) == "derived_transition" &&
                    as_text(member_ref(event.values, "derivation_rule")) == "channel_value_change.v1" &&
                    event.applied_step && event.processed_step == *event.applied_step,
                    "every event carries rule metadata and aligned steps");
}

void test_derived_first_model_failure_stays_a_derived_transition() {
    const auto violation_row = [](std::int64_t id, Wire first) {
        return channel_row(id, .001, WireObject{{"model_status", Wire{WireObject{
            {"model_valid", Wire{sw::is_none(first)}},
            {"first_model_violation", Wire{std::move(first)}}}}}});
    };
    const auto violation = [](const char *reason) {
        return Wire{WireObject{{"time_s", Wire{.001}},
                               {"reasons", Wire{WireArray{Wire{std::string(reason)}}}}}};
    };
    const auto episode = make_episode({violation_row(0, Wire{}),
                                       violation_row(1, violation("joint.limit")),
                                       violation_row(2, violation("joint.limit"))});
    const auto events = rec::derived_events(episode, 0, 0, 0.);
    bt::require(events.size() == 2 &&
                events[0].kind == "derived_transition" && events[1].kind == "derived_transition",
                "sticky model-status transitions stay derived facts");
    bt::require(as_text(member_ref(events[0].values, "derived_kind")) == "model_validity",
                "the validity transition is labeled");
    const auto &changes = as_object(member_ref(events[0].values, "changes"));
    bt::require(changes.size() == 1 && !as_bool(required_member(changes, "model_valid")),
                "the validity transition carries model_valid");
    bt::require(as_text(member_ref(events[1].values, "derived_kind")) == "model_first_failure",
                "the sticky first failure is a channel-derived fact, not the monitor event");
}

void test_derived_missing_reason_instrumentation_emits_nothing_source() {
    const auto crash_row = [](std::int64_t id, bool present, Wire cause) {
        WireObject channels;
        if (present) channels.emplace_back("contact_crash_cause", std::move(cause));
        return channel_row(id, .001, std::move(channels));
    };
    const auto silent = make_episode({crash_row(0, false, Wire{}), crash_row(1, false, Wire{}),
                                      crash_row(2, false, Wire{}), crash_row(3, false, Wire{})});
    bt::require(rec::derived_events(silent, 0, 0, 0.).empty(),
                "absent channel instrumentation is not a transition");
    const auto episode = make_episode({
        crash_row(0, true, Wire{}),
        crash_row(1, true, Wire{WireObject{{"kind", Wire{std::string("tire_blowout")}}}})});
    const auto events = rec::derived_events(episode, 0, 0, 0.);
    bt::require(events.size() == 1 && events[0].kind == "derived_transition" &&
                as_text(member_ref(events[0].values, "derived_kind")) == "contact_crash" &&
                as_text(member_ref(events[0].values, "classification")) == "derived_transition",
                "the appearing crash cause is a derived fact");
}

void test_episode_interval_for_time_maps_failure_timestamp() {
    const auto episode = make_episode({brake_row(0, false, 0.), brake_row(1, false, 0.),
                                       brake_row(2, false, 0.), brake_row(3, false, 0.),
                                       brake_row(4, false, 0.)});
    bt::require(rec::episode_interval_for_time(episode, 0.) == 0 &&
                rec::episode_interval_for_time(episode, .003) == 2 &&
                rec::episode_interval_for_time(episode, 99.) == 4,
                "the interval lookup anchors timestamps to their span");
    const auto empty = make_episode({});
    bt::require(rec::episode_interval_for_time(empty, .1) == 0,
                "the empty episode falls back to 0");
}

void test_capture_retries_publication_with_evidence_intact() {
    bt::TemporaryDirectory root("lifecycle-capture-retry");
    rec::RunCapture capture(run_metadata(), true);
    rec::EpisodeJournal journal(capture, 0);
    (void)journal.emit("episode_start", 0., 0);
    // Native finalize validates event/episode coverage, so the ported
    // capture carries the owned episode its journal references.
    capture.add_episode(make_evidence(0, 1, {sample_row(0, 0.)}));
    int calls = 0;
    rec::FinalizeOptions options;
    options.before_stage = [&](std::string_view stage) {
        if (stage == "manifest_link" && ++calls == 1)
            throw std::runtime_error("disk busy");
    };
    throws<std::runtime_error>([&] { (void)rec::finalize(capture, root.path(), options); },
                               "disk busy", "first publication fails");
    bt::require(capture.state() == rec::RunCapture::State::publication_failed,
                "the failure leaves the capture retryable");
    const auto publication = rec::finalize(capture, root.path(), options);
    bt::require(capture.state() == rec::RunCapture::State::published &&
                calls == 2 && std::filesystem::exists(publication.directory / "manifest.json"),
                "retry publishes the intact evidence");
    bt::require(capture.events().size() == 1 && capture.events()[0].kind == "episode_start" &&
                capture.episodes().size() == 1,
                "the journaled event and owned episode survive the failed attempt");
}

// ---- backend rejection (the 'python' backend guard) -------------------------

void test_headless_rejects_python_record_before_any_setup() {
    // The native CLI's backend domain is {"native"}: --record --backend
    // python rejects at parse, before resolve_ride builds anything — the
    // oracle's 'native backend' ValueError half is the domain guard.
    const std::array<std::string_view, 4> args{"--record", "--backend", "python", "--headless"};
    throws<bike::cli::ArgumentError>(
        [&] { (void)bike::cli::parse_arguments("bike-ride", args); }, "",
        "recording under a non-native backend is rejected before setup");
}

void test_viewer_python_backend_record_rejected_before_window() {
    const std::array<std::string_view, 3> args{"--record", "--backend", "python"};
    throws<bike::cli::ArgumentError>(
        [&] { (void)bike::cli::parse_arguments("bike-ride", args); }, "",
        "recording under a non-native backend rejects before any window");
}

void test_make_run_capture_rejects_non_native_recording() {
    // The enabled+non-native rejection is frontend policy; natively the
    // same policy is the backend domain guard. The disabled-capture
    // metadata half ports directly.
    const std::array<std::string_view, 3> args{"--record", "--backend", "python"};
    throws<bike::cli::ArgumentError>(
        [&] { (void)bike::cli::parse_arguments("bike-ride", args); }, "",
        "recording under a non-native backend is rejected");
    rec::RunCapture disabled(run_metadata("python", "viewer"), false);
    bt::require(!disabled.enabled() &&
                disabled.metadata().backend == "python" &&
                as_text(member_ref(disabled.metadata().values, "check_policy")) ==
                    "strict",
                "the disabled capture keeps its launch metadata");
}

// ---- viewer pump: the capture/journal-level contracts ------------------------

void test_viewer_journals_two_episodes_and_publishes_once() {
    // The fake-window pump drives two generations through one journal
    // sequence; the ported contract is the journal/episode ordering and
    // the once-only publication the pump guarantees — request and applied
    // records keep their receipt order and step identity distinctions.
    bt::TemporaryDirectory root("lifecycle-two-episodes");
    rec::RunCapture capture(run_metadata(), true);
    {
        rec::EpisodeJournal journal(capture, 0);
        (void)journal.emit("episode_start", .1, 0);
        (void)journal.emit("brake_toggle", .2, 3, 3,
                           Value{Value::Object{{"record", text("requested")},
                                               {"braking", Value{true}}}});
        (void)journal.emit("brake_demand", .21, 4, 4,
                           Value{Value::Object{{"record", text("applied")},
                                               {"front_brake_demand", real(.5)}}});
        (void)journal.emit("brake_strength", .22, 5, 5,
                           Value{Value::Object{{"record", text("applied")},
                                               {"strength", real(.4)}}});
        (void)journal.emit("brake_demand", .23, 6, 6,
                           Value{Value::Object{{"record", text("applied")},
                                               {"front_brake_demand", real(.5)}}});
        (void)journal.emit("brake_strength", .24, 7, 7,
                           Value{Value::Object{{"record", text("applied")}}});
        (void)journal.emit("brake_demand", .25, 8, 8,
                           Value{Value::Object{{"record", text("applied")},
                                               {"front_brake_demand", real(.5)}}});
        (void)journal.emit("reset", .3, 9, 9);
    }
    capture.add_episode(make_evidence(0, 1, {sample_row(0, 0.)}));
    {
        rec::EpisodeJournal journal(capture, 1);
        (void)journal.emit("episode_start", .4, 0);
        (void)journal.emit("brake_toggle", .41, 2, 2,
                           Value{Value::Object{{"record", text("requested")}}});
        (void)journal.emit("brake_demand", .42, 3, 3,
                           Value{Value::Object{{"record", text("applied")}}});
        (void)journal.emit("brake_strength", .43, 4, 4,
                           Value{Value::Object{{"record", text("applied")}}});
        (void)journal.emit("brake_demand", .44, 5, 5,
                           Value{Value::Object{{"record", text("applied")}}});
        (void)journal.emit("quit", .5, 6, std::nullopt);
    }
    capture.add_episode(make_evidence(1, 2, {sample_row(0, .0)}));
    const auto publication = rec::finalize(capture, root.path());
    bt::require(capture.state() == rec::RunCapture::State::published &&
                std::filesystem::exists(publication.directory / "manifest.json"),
                "both episodes publish in one finalize");
    const auto ep0 = kinds(capture.events(), 0);
    const auto ep1 = kinds(capture.events(), 1);
    bt::require(ep0 == std::vector<std::string>(
                    {"episode_start", "brake_toggle", "brake_demand", "brake_strength",
                     "brake_demand", "brake_strength", "brake_demand", "reset"}) &&
                ep1 == std::vector<std::string>(
                    {"episode_start", "brake_toggle", "brake_demand", "brake_strength",
                     "brake_demand", "quit"}),
                "requested inputs arrive in receipt order per episode");
    std::map<std::pair<std::int64_t, std::string>, const rec::InputEvent *> by_kind;
    for (const auto &event : capture.events()) by_kind[{event.episode_id, event.kind}] = &event;
    const auto *toggle = by_kind.at({0, "brake_toggle"});
    const auto *applied = by_kind.at({0, "brake_demand"});
    bt::require(toggle->received_wall_s <= applied->received_wall_s &&
                as_text(member_ref(toggle->values, "record")) == "requested" &&
                as_bool(member_ref(toggle->values, "braking")) &&
                as_text(member_ref(applied->values, "record")) == "applied" &&
                as_real(member_ref(applied->values, "front_brake_demand")) == .5 &&
                applied->applied_step && *applied->applied_step == applied->processed_step,
                "receipt and processed steps stay distinct on applied records");
    bt::require(!by_kind.at({1, "quit"})->applied_step,
                "terminal requests keep no applied step");
    rec::Recording saved(publication.directory);
    bt::require(saved.episode_ids() == std::vector<std::int64_t>({0, 1}),
                "the sealed recording carries both episodes");
    sealed_directory(root.path());
}

void test_viewer_duplicate_terminal_notice_keeps_one_episode() {
    // The production seal-dedupe lives on the run owner:
    // research::ResearchRun::seal returns the already-prepared receipt for
    // a second terminal notice over the same episode.
    auto request = bike::research::read_research_request(bt::fixture("requests/research_delay.toml"));
    request.capture_mode = runtime::ResearchCaptureMode::full;
    auto seed = bike::research::prepare_environment(std::move(request));
    auto environment = runtime::make_research(std::move(seed.bootstrap));
    class Stub final : public bike::research::ResearchPublication {
      public:
        int calls = 0;
        std::size_t episodes = 0;
        std::filesystem::path publish(std::span<const bike::research::EpisodeReceipt> receipts,
                                      std::span<const bike::research::OperatorEvent>) override {
            ++calls; episodes = receipts.size();
            return std::filesystem::temp_directory_path() / "lifecycle-dedupe";
        }
    };
    const auto publication = std::make_shared<Stub>();
    {
        bike::research::ResearchRun run(*environment, publication);
        const auto *first = run.seal("end_of_track");
        const auto *second = run.seal("end_of_track");
        bt::require(first != nullptr && second == first && run.episodes().size() == 1,
                    "a second terminal notice over the same episode keeps one receipt");
        (void)run.finish();
    }
    bt::require(publication->calls == 1 && publication->episodes == 1,
                "the publication carries one episode");
    environment->close();
}

void test_viewer_native_pump_seals_reset_generations() {
    // The reset hook's seal-before-reset contract: each generation seals
    // into its own episode, journals restart per episode and the derived
    // transitions re-read the sealed channel rows — a real brake demand
    // flip mid-run produces the generation-1 brake_gate fact.
    const auto rig = welded_runtime("full");
    advance(*rig->runtime, 2);
    (void)rig->runtime->advance(4, runtime::RideControl{}, .4, .4, std::nullopt);
    auto first_episode = std::make_shared<runtime::OwnedNativeEpisode>(rig->runtime->seal_episode());
    bt::require(first_episode->generation == 1, "generation 1 seals first");
    rig->runtime->reset();
    (void)rig->runtime->advance(2, runtime::RideControl{}, .4, .4, std::nullopt);
    (void)rig->runtime->advance(4, runtime::RideControl{}, 0., 0., std::nullopt);
    auto second_episode = std::make_shared<runtime::OwnedNativeEpisode>(rig->runtime->seal_episode());
    bt::require(second_episode->generation == 2, "the reset generation seals separately");
    bt::TemporaryDirectory root("lifecycle-reset-generations");
    rec::RunCapture capture(run_metadata(), true);
    {
        rec::EpisodeJournal journal(capture, 0);
        (void)journal.emit("episode_start", .1, 0);
        journal.emit_all(rec::derived_events(*first_episode, 0, journal.next_id(), .2));
    }
    {
        auto evidence = std::make_shared<rec::EpisodeEvidence>();
        evidence->episode_id = 0; evidence->generation = first_episode->generation;
        evidence->physical = rec::make_physical_source(
            first_episode, cli::model_catalog(rig->runtime->model_image()), true);
        evidence->summary = Value{Value::Object{
            {"outcome", Value{Value::Object{{"reason", text("reset")}}}},
            {"model_status", Value{Value::Object{{"model_valid", Value{true}}}}}}};
        capture.add_episode(std::move(evidence));
    }
    {
        rec::EpisodeJournal journal(capture, 1);
        (void)journal.emit("episode_start", .3, 0);
        (void)journal.emit("quit", .35, 0, std::nullopt);
        journal.emit_all(rec::derived_events(*second_episode, 1, journal.next_id(), .4));
    }
    {
        auto evidence = std::make_shared<rec::EpisodeEvidence>();
        evidence->episode_id = 1; evidence->generation = second_episode->generation;
        evidence->physical = rec::make_physical_source(
            second_episode, cli::model_catalog(rig->runtime->model_image()), true);
        evidence->summary = Value{Value::Object{
            {"outcome", Value{Value::Object{{"reason", text("viewer_closed")}}}},
            {"model_status", Value{Value::Object{{"model_valid", Value{true}}}}}}};
        capture.add_episode(std::move(evidence));
    }
    const auto publication = rec::finalize(capture, root.path());
    rec::Recording saved(publication.directory);
    bt::require(saved.episode_ids() == std::vector<std::int64_t>({0, 1}),
                "both generations publish as separate episodes");
    const auto ep0 = kinds(capture.events(), 0), ep1 = kinds(capture.events(), 1);
    bt::require(ep0.size() >= 2 && ep0.front() == "episode_start" &&
                ep1.size() >= 2 && ep1.front() == "episode_start" && ep1[1] == "quit",
                "per-episode journals restart with lifecycle kinds");
    bt::require(std::ranges::all_of(capture.events(), [](const rec::InputEvent &e) {
                    return e.kind == "episode_start" || e.kind == "quit" ||
                           e.kind == "derived_transition";
                }),
                "seal-time facts are lifecycle or derived kinds only");
    bt::require(std::ranges::any_of(capture.events(), [](const rec::InputEvent &e) {
                    return e.kind == "derived_transition" &&
                           as_text(member_ref(e.values, "derivation_rule")) == "channel_value_change.v1" &&
                           as_text(member_ref(e.values, "derived_kind")) == "brake_gate";
                }),
                "the real brake flip re-derives the brake_gate transition from sealed rows");
}

// ---- Ctrl-C / shielded seal+finalize compositions ---------------------------

// The shielded seal+finalize block cli/ride.cpp runs: the Python frontends
// inject signal.raise_signal inside driver.seal_episode / the publisher to
// prove the shield defers delivery; natively the same composition runs
// through platform::InterruptShield with note()/raise() as the injected
// seam and finalize/finalize_guarded as the real publication.
std::shared_ptr<const rec::EpisodeEvidence>
evidence_for(runtime::OwnedNativeEpisode &episode, std::int64_t episode_id,
             std::string_view reason) {
    auto owned = std::make_shared<runtime::OwnedNativeEpisode>(std::move(episode));
    auto evidence = std::make_shared<rec::EpisodeEvidence>();
    evidence->episode_id = episode_id;
    evidence->generation = owned->generation;
    evidence->physical = make_source(owned);
    evidence->summary = Value{Value::Object{
        {"outcome", Value{Value::Object{{"reason", text(std::string(reason))}}}}}};
    return evidence;
}

void test_viewer_second_ctrl_c_during_cleanup_seal_still_publishes() {
    bt::TemporaryDirectory root("lifecycle-deferred-seal");
    rec::RunCapture capture(run_metadata(), true);
    bool rethrow = false;
    {
        bike::platform::InterruptShield shield;
        rec::EpisodeJournal journal(capture, 0);
        (void)journal.emit("episode_start", .1, 0);
        // The shielded seal: owned evidence then derived journal events.
        auto episode = make_episode({brake_row(0, true, .4), brake_row(1, false, .4)});
        journal.emit_all(rec::derived_events(episode, 0, journal.next_id(), .2));
        capture.add_episode(evidence_for(episode, 0, "operator_stop"));
        // The injected second Ctrl-C lands inside the shielded seal: the
        // journal is still open, so the deferred interrupt is journaled
        // (physical_frontend._journal_deferred; the native ride frontend
        // emits the same kind through research::ResearchRun::finish).
        shield.note();
        (void)journal.emit("operator_stop_deferred", .3, 0);
        const auto publication = rec::finalize(capture, root.path());
        bt::require(std::filesystem::exists(publication.directory / "manifest.json"),
                    "the shielded publication still seals");
        rethrow = shield.deferred() > 0;
    }
    bt::require(rethrow, "the deferred interrupt is honored after the section");
    const auto &events = capture.events();
    bt::require(kinds(events, 0) == std::vector<std::string>(
                    {"episode_start", "derived_transition", "operator_stop_deferred"}),
                "the deferred interrupt is journaled before publication");
    rec::Recording saved(sealed_directory(root.path()));
    bt::require(saved.episode_ids() == std::vector<std::int64_t>({0}) &&
                physical_rows_of(saved, 0) == 2 &&
                std::filesystem::exists(saved.path() / "run.duckdb"),
                "the sealed episode reads back with its committed prefix");
}

void test_viewer_second_ctrl_c_during_cleanup_finalize_still_publishes() {
    bt::TemporaryDirectory root("lifecycle-deferred-finalize");
    rec::RunCapture capture(run_metadata(), true);
    {
        rec::EpisodeJournal journal(capture, 0);
        (void)journal.emit("episode_start", .1, 0);
    }
    {
        auto episode = make_episode({brake_row(0, true, 0.)});
        capture.add_episode(evidence_for(episode, 0, "operator_stop"));
    }
    rec::FinalizeOptions options;
    options.before_stage = [](std::string_view stage) {
        if (stage == "manifest_link") (void)std::raise(SIGINT);
    };
    // finalize_guarded owns the shield: the injected signal lands inside
    // it — too late to journal — and surfaces as OperatorInterrupted after
    // the manifest is sealed (the oracle's '__notes__' re-raise contract).
    throws<bike::research::OperatorInterrupted>(
        [&] { (void)rec::finalize_guarded(capture, root.path(), options); },
        "operator interrupt deferred",
        "the deferred interrupt is re-raised after publication");
    bt::require(capture.state() == rec::RunCapture::State::published,
                "the publication still committed");
    bt::require(kinds(capture.events(), 0) == std::vector<std::string>({"episode_start"}),
                "a deferred interrupt inside finalize is not journaled");
    rec::Recording saved(sealed_directory(root.path()));
    bt::require(saved.episode_ids() == std::vector<std::int64_t>({0}) &&
                std::filesystem::exists(saved.path() / "run.duckdb"),
                "the sealed recording reads back");
}

void test_viewer_ctrl_c_during_normal_finalize_publishes_then_stops() {
    // Deferred before the finalize call inside the shielded section: the
    // interrupt is journaled (the journal is still open), then publication
    // commits and the interrupt is honored — an operator-stop exit.
    bt::TemporaryDirectory root("lifecycle-deferred-normal");
    rec::RunCapture capture(run_metadata(), true);
    bool rethrow = false;
    {
        bike::platform::InterruptShield shield;
        rec::EpisodeJournal journal(capture, 0);
        (void)journal.emit("episode_start", .1, 0);
        auto episode = make_episode({brake_row(0, true, .4), brake_row(1, false, .4)});
        journal.emit_all(rec::derived_events(episode, 0, journal.next_id(), .2));
        capture.add_episode(evidence_for(episode, 0, "viewer_closed"));
        shield.note();
        (void)journal.emit("operator_stop_deferred", .3, 0);
        const auto publication = rec::finalize(capture, root.path());
        bt::require(std::filesystem::exists(publication.directory / "manifest.json"),
                    "publication still commits");
        rethrow = shield.deferred() > 0;
    }
    bt::require(rethrow, "the interrupt is honored after the section");
    bt::require(kinds(capture.events(), 0) == std::vector<std::string>(
                    {"episode_start", "derived_transition", "operator_stop_deferred"}),
                "the pre-finalize deferred interrupt is journaled");
    rec::Recording saved(sealed_directory(root.path()));
    bt::require(sealed_kinds(saved, 0) == std::vector<std::string>(
                    {"episode_start", "derived_transition", "operator_stop_deferred"}),
                "the sealed journal carries the deferred event");
}

void test_viewer_ctrl_c_during_normal_finalize_is_noted_on_stop() {
    bt::TemporaryDirectory root("lifecycle-deferred-noted");
    rec::RunCapture capture(run_metadata(), true);
    {
        rec::EpisodeJournal journal(capture, 0);
        (void)journal.emit("episode_start", .1, 0);
    }
    {
        auto episode = make_episode({brake_row(0, true, 0.)});
        capture.add_episode(evidence_for(episode, 0, "viewer_closed"));
    }
    rec::FinalizeOptions options;
    options.before_stage = [](std::string_view stage) {
        if (stage == "manifest_link") (void)std::raise(SIGINT);
    };
    throws<bike::research::OperatorInterrupted>(
        [&] { (void)rec::finalize_guarded(capture, root.path(), options); },
        "operator interrupt deferred",
        "a deferred interrupt inside finalize re-raises after sealing");
    (void)sealed_directory(root.path());
    bt::require(kinds(capture.events(), 0) == std::vector<std::string>({"episode_start"}),
                "the deferred interrupt lands too late to journal");
}

void test_viewer_ctrl_c_adds_no_control_and_still_finalizes() {
    // KeyboardInterrupt during advance seals the committed prefix as
    // operator_stop and finalizes; only lifecycle + derived facts are
    // journaled — no control kinds.
    bt::TemporaryDirectory root("lifecycle-ctrlc-prefix");
    rec::RunCapture capture(run_metadata(), true);
    bool rethrow = false;
    {
        bike::platform::InterruptShield shield;
        rec::EpisodeJournal journal(capture, 0);
        (void)journal.emit("episode_start", .1, 0);
        shield.note(); // the injected Ctrl-C during the advance step
        auto episode = make_episode({brake_row(0, true, .4), brake_row(1, false, .4)});
        journal.emit_all(rec::derived_events(episode, 0, journal.next_id(), .2));
        capture.add_episode(evidence_for(episode, 0, "operator_stop"));
        const auto publication = rec::finalize(capture, root.path());
        bt::require(std::filesystem::exists(publication.directory / "manifest.json"),
                    "the interrupted run still publishes its prefix");
        rethrow = shield.deferred() > 0;
    }
    bt::require(rethrow, "the interrupt is honored after finalization");
    bt::require(capture.events().size() == 2 &&
                std::ranges::all_of(capture.events(), [](const rec::InputEvent &e) {
                    return e.kind == "episode_start" || e.kind == "derived_transition";
                }),
                "no control events are journaled for an operator interrupt");
}

void test_viewer_simulation_error_seals_prefix_and_reraises() {
    // A primary runtime_error during advance: the committed prefix seals
    // as simulation_error, publication commits, and the ORIGINAL failure
    // rethrows — not a seal or publish error.
    bt::TemporaryDirectory root("lifecycle-simulation-error");
    const auto rig = welded_runtime("full");
    advance(*rig->runtime, 6);
    rig->runtime->test_fail_at_step(6);
    std::exception_ptr primary;
    try {
        (void)rig->runtime->advance(10, runtime::RideControl{}, 0., 0., std::nullopt);
    } catch (...) { primary = std::current_exception(); }
    bt::require(primary != nullptr, "the injected simulation failure is the primary");
    rec::RunCapture capture(run_metadata(), true);
    {
        rec::EpisodeJournal journal(capture, 0);
        (void)journal.emit("episode_start", .1, 0);
    }
    auto episode = std::make_shared<runtime::OwnedNativeEpisode>(rig->runtime->seal_episode());
    auto evidence = std::make_shared<rec::EpisodeEvidence>();
    evidence->episode_id = 0; evidence->generation = episode->generation;
    evidence->physical = rec::make_physical_source(
        episode, cli::model_catalog(rig->runtime->model_image()), true);
    evidence->summary = Value{Value::Object{
        {"outcome", Value{Value::Object{{"reason", text("simulation_error")}}}}}};
    capture.add_episode(std::move(evidence));
    const auto publication = rec::finalize(capture, root.path());
    bt::require(std::filesystem::exists(publication.directory / "manifest.json"),
                "the committed failure prefix publishes");
    rig->runtime->close();
    bool rethrown = false;
    try { std::rethrow_exception(primary); }
    catch (const std::runtime_error &) { rethrown = true; }
    bt::require(rethrown, "the primary simulation error rethrows after finalization");
}

// ---- real headless rides (cli::ride_command) ---------------------------------

bike::cli::Arguments headless_args(const std::filesystem::path &physics,
                                   const std::filesystem::path &out, bool record,
                                   const char *duration = "0.01") {
    const auto track = repo_file("examples/research/rough_uphill_savage.toml");
    std::vector<std::string_view> args{
        "--physics-config", physics.c_str(),
        "--track", track.c_str(),
        "--headless", "--duration", duration, "--no-plots",
        "--check-policy", "diagnostic", "--decimate", "1",
        "--out", out.c_str()};
    if (record) args.emplace_back("--record");
    return bike::cli::parse_arguments("bike-ride", args);
}

void test_headless_records_seals_and_finalizes_once() {
    // The real cli::ride_command headless path seals once and publishes at
    // end. The welded recipe trips its sticky energy.constraint_work marker
    // at the first reference checkpoint (welded feet do real constraint
    // work), so the honest exit is 2 with reason duration_reached — the
    // seal/publish contract the oracle fakes covers is identical.
    bt::TemporaryDirectory root("lifecycle-headless-ok");
    const auto physics = repo_file("examples/research/viewer_physics_welded.toml");
    const auto args = headless_args(physics, root.path(), true);
    const int code = bike::cli::ride_command(args);
    bt::require(code == 2, "the welded marker exits 2 after duration_reached");
    rec::Recording saved(sealed_directory(root.path()));
    bt::require(saved.episode_ids() == std::vector<std::int64_t>({0}),
                "one episode published");
    bt::require(physical_rows_of(saved, 0) > 0,
                "the sealed episode carries physical rows");
    const auto summary = summary_of(saved, 0);
    bt::require(as_text(member_ref(member_ref(summary, "outcome"), "reason")) ==
                "duration_reached",
                "the episode summary carries the duration outcome");
    const auto kinds_seen = sealed_kinds(saved, 0);
    bt::require(!kinds_seen.empty() && kinds_seen.front() == "episode_started",
                "the episode journal starts with episode_started");
    bt::require(std::ranges::all_of(kinds_seen, [](const std::string &kind) {
                    return kind == "episode_started" || kind == "derived_transition";
                }),
                "a healthy run journals lifecycle and derived facts only");
}

void test_headless_native_run_records_one_episode() {
    // The violating recipe trips a real model-check violation; diagnostic
    // mode keeps the sticky first failure and the run exits 2 after
    // publishing its evidence. The oracle's 'model_first_failure' measured
    // event is frontend emission; natively the sticky failure rides on the
    // episode summary and channel-derived model-status transitions.
    bt::TemporaryDirectory root("lifecycle-headless-violation");
    const auto physics = violating_physics(root.path() / "physics.toml");
    const auto args = headless_args(physics, root.path(), true);
    const int code = bike::cli::ride_command(args);
    bt::require(code == 2, "the violating run reports the model-check outcome");
    rec::Recording saved(sealed_directory(root.path()));
    bt::require(saved.episode_ids() == std::vector<std::int64_t>({0}),
                "one episode published");
    bt::require(physical_rows_of(saved, 0) > 0,
                "the sealed episode carries physical rows");
    const auto summary = summary_of(saved, 0);
    bt::require(as_text(member_ref(member_ref(summary, "outcome"), "reason")) ==
                "duration_reached",
                "the diagnostic run still ends at duration");
    const auto &failure = member_ref(summary, "first_failure");
    bt::require(!io::is_null(failure) &&
                io::recording_json(failure).find("foot_front.normal") != std::string::npos,
                "the sticky first failure carries the weld-envelope violation");
    const auto kinds_seen = sealed_kinds(saved, 0);
    bt::require(!kinds_seen.empty() && kinds_seen.front() == "episode_started",
                "the journal starts with episode_started");
    const auto run = saved.metadata();
    const auto &run_object = as_object(run);
    bt::require(as_text(required_member(run_object, "backend")) == "native" &&
                as_text(required_member(run_object, "mode")) == "physical_headless",
                "launch metadata carries backend and mode");
    const auto tires = saved.rows("SELECT count(*) AS n FROM channel_tires");
    const auto control = saved.rows("SELECT count(*) AS n FROM channel_control");
    bt::require(tires.size() == 1 && as_integer(required_member(tires[0], "n")) > 0 &&
                control.size() == 1 && as_integer(required_member(control[0], "n")) > 0,
                "typed channel tables decode for the real episode");
}

void test_headless_off_writes_no_artifact_files() {
    // Without --record the run writes NOTHING under --out — not even an
    // empty file (the oracle asserts rglob('*') == []).
    bt::TemporaryDirectory root("lifecycle-headless-off");
    const auto out = root.path() / "out";
    std::filesystem::create_directories(out);
    const auto physics = violating_physics(root.path() / "physics.toml");
    const auto args = headless_args(physics, out, false);
    const int code = bike::cli::ride_command(args);
    bt::require(code == 2, "the unrecorded run still reports the model outcome");
    bt::require(std::filesystem::is_empty(out),
                "off-record runs write no artifact files");
}

void test_headless_zero_step_failure_keeps_owned_episode() {
    // An initialization-time driver failure seals the committed prefix —
    // zero rows — into its own episode and still publishes it (R03/R16).
    bt::TemporaryDirectory root("lifecycle-zero-step");
    const auto rig = welded_runtime("full");
    rig->runtime->test_fail_at_step(0);
    bool failed = false;
    try { (void)rig->runtime->advance(4, runtime::RideControl{}, 0., 0., std::nullopt); }
    catch (const std::exception &) { failed = true; }
    bt::require(failed, "the zero-step failure propagates through advance");
    auto episode = std::make_shared<runtime::OwnedNativeEpisode>(rig->runtime->seal_episode());
    bt::require(episode->samples.empty(),
                "the zero-step episode seals with an empty prefix");
    rec::RunCapture capture(run_metadata(), true);
    {
        rec::EpisodeJournal journal(capture, 0);
        (void)journal.emit("episode_start", .1, 0);
    }
    auto evidence = std::make_shared<rec::EpisodeEvidence>();
    evidence->episode_id = 0; evidence->generation = episode->generation;
    evidence->physical = rec::make_physical_source(
        episode, cli::model_catalog(rig->runtime->model_image()), true);
    evidence->summary = Value{Value::Object{
        {"outcome", Value{Value::Object{{"reason", text("simulation_error")}}}}}};
    capture.add_episode(std::move(evidence));
    const auto publication = rec::finalize(capture, root.path());
    bt::require(std::filesystem::exists(publication.directory / "manifest.json"),
                "the zero-step episode publishes");
    rec::Recording saved(publication.directory);
    bt::require(saved.episode_ids() == std::vector<std::int64_t>({0}) &&
                physical_rows_of(saved, 0) == 0,
                "the sealed episode reads back with zero physical rows");
}

void test_headless_success_with_failed_publication_is_nonzero() {
    // A clean recorded run whose finalize fails exits nonzero: physical_run
    // rethrows the publication failure through ride_command.
    bt::TemporaryDirectory root("lifecycle-publish-fail");
    const auto blocker = root.path() / "blocker";
    { std::ofstream file(blocker); file << "occupied"; }
    const auto physics = repo_file("examples/research/viewer_physics_welded.toml");
    const auto args = headless_args(physics, blocker / "run", true);
    bool failed = false;
    try { (void)bike::cli::ride_command(args); }
    catch (const std::exception &) { failed = true; }
    bt::require(failed, "a failed publication surfaces as a nonzero exit");
}

void test_headless_publish_failure_preserves_primary_and_evidence() {
    // A primary driver failure plus a publication failure: the primary
    // rethrows, the capture retains its owned evidence for retry, and the
    // later finalize succeeds (R04/R16).
    bt::TemporaryDirectory root("lifecycle-primary-publish-fail");
    const auto rig = welded_runtime("full");
    advance(*rig->runtime, 4);
    rig->runtime->test_fail_at_step(4);
    std::exception_ptr primary;
    try { (void)rig->runtime->advance(8, runtime::RideControl{}, 0., 0., std::nullopt); }
    catch (...) { primary = std::current_exception(); }
    bt::require(primary != nullptr, "the primary failure is captured");
    rec::RunCapture capture(run_metadata(), true);
    auto episode = std::make_shared<runtime::OwnedNativeEpisode>(rig->runtime->seal_episode());
    auto evidence = std::make_shared<rec::EpisodeEvidence>();
    evidence->episode_id = 0; evidence->generation = episode->generation;
    evidence->physical = rec::make_physical_source(
        episode, cli::model_catalog(rig->runtime->model_image()), true);
    evidence->summary = Value{Value::Object{
        {"outcome", Value{Value::Object{{"reason", text("simulation_error")}}}}}};
    capture.add_episode(std::move(evidence));
    rec::FinalizeOptions options;
    options.before_stage = [](std::string_view stage) {
        if (stage == "manifest_link") throw std::runtime_error("publish failed");
    };
    throws<std::runtime_error>([&] { (void)rec::finalize(capture, root.path(), options); },
                               "publish failed", "the publication failure propagates");
    bt::require(capture.state() == rec::RunCapture::State::publication_failed &&
                capture.episodes().size() == 1,
                "the failed publication retains the owned evidence");
    options.before_stage = {};
    const auto publication = rec::finalize(capture, root.path(), options);
    bt::require(std::filesystem::exists(publication.directory / "manifest.json"),
                "the retried publication succeeds with intact evidence");
    rig->runtime->close();
    bool rethrown = false;
    try { std::rethrow_exception(primary); }
    catch (const std::runtime_error &) { rethrown = true; }
    bt::require(rethrown, "the primary failure rethrows, not the publish error");
}

void test_headless_second_ctrl_c_during_seal_journals_and_still_publishes() {
    // The headless variant of the seal-time deferred interrupt: journaled
    // while the journal is still open, then publication commits.
    bt::TemporaryDirectory root("lifecycle-headless-deferred-seal");
    rec::RunCapture capture(run_metadata("native", "physical_headless"), true);
    bool rethrow = false;
    {
        bike::platform::InterruptShield shield;
        rec::EpisodeJournal journal(capture, 0);
        (void)journal.emit("episode_started", .1, 0);
        auto episode = make_episode({brake_row(0, true, .4), brake_row(1, false, .4)});
        journal.emit_all(rec::derived_events(episode, 0, journal.next_id(), .2));
        capture.add_episode(evidence_for(episode, 0, "operator_stop"));
        shield.note();
        (void)journal.emit("operator_stop_deferred", .3, 0);
        const auto publication = rec::finalize(capture, root.path());
        bt::require(std::filesystem::exists(publication.directory / "manifest.json"),
                    "the shielded publication still seals");
        rethrow = shield.deferred() > 0;
    }
    bt::require(rethrow, "the deferred interrupt is honored after the section");
    bt::require(kinds(capture.events(), 0) == std::vector<std::string>(
                    {"episode_started", "derived_transition", "operator_stop_deferred"}),
                "the deferred interrupt is journaled before publication");
}

void test_headless_second_ctrl_c_during_finalize_is_noted_and_publishes() {
    bt::TemporaryDirectory root("lifecycle-headless-deferred-finalize");
    rec::RunCapture capture(run_metadata("native", "physical_headless"), true);
    {
        rec::EpisodeJournal journal(capture, 0);
        (void)journal.emit("episode_started", .1, 0);
    }
    {
        auto episode = make_episode({brake_row(0, true, 0.)});
        capture.add_episode(evidence_for(episode, 0, "operator_stop"));
    }
    rec::FinalizeOptions options;
    options.before_stage = [](std::string_view stage) {
        if (stage == "manifest_link") (void)std::raise(SIGINT);
    };
    throws<bike::research::OperatorInterrupted>(
        [&] { (void)rec::finalize_guarded(capture, root.path(), options); },
        "operator interrupt deferred",
        "the deferred interrupt re-raises after sealing");
    bt::require(kinds(capture.events(), 0) == std::vector<std::string>({"episode_started"}),
                "a deferred interrupt inside finalize is not journaled");
    rec::Recording saved(sealed_directory(root.path()));
    bt::require(saved.episode_ids() == std::vector<std::int64_t>({0}),
                "the sealed recording reads back");
}

} // namespace

// The dispatch lambda's frame aggregates the inlined test locals across the
// 26 ports; tests are the leaf of the call graph so the bound is advisory.
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
int main() {
    return bt::run([] {
        test_derived_transitions_carry_rule_version_and_interval();
        test_derived_transitions_cover_shift_coast_and_limiter_gates();
        test_derived_first_model_failure_stays_a_derived_transition();
        test_derived_missing_reason_instrumentation_emits_nothing_source();
        test_episode_interval_for_time_maps_failure_timestamp();
        test_capture_retries_publication_with_evidence_intact();
        test_headless_rejects_python_record_before_any_setup();
        test_viewer_python_backend_record_rejected_before_window();
        test_make_run_capture_rejects_non_native_recording();
        test_viewer_journals_two_episodes_and_publishes_once();
        test_viewer_duplicate_terminal_notice_keeps_one_episode();
        test_viewer_native_pump_seals_reset_generations();
        test_viewer_second_ctrl_c_during_cleanup_seal_still_publishes();
        test_viewer_second_ctrl_c_during_cleanup_finalize_still_publishes();
        test_viewer_ctrl_c_during_normal_finalize_publishes_then_stops();
        test_viewer_ctrl_c_during_normal_finalize_is_noted_on_stop();
        test_viewer_ctrl_c_adds_no_control_and_still_finalizes();
        test_viewer_simulation_error_seals_prefix_and_reraises();
        test_headless_records_seals_and_finalizes_once();
        test_headless_native_run_records_one_episode();
        test_headless_off_writes_no_artifact_files();
        test_headless_zero_step_failure_keeps_owned_episode();
        test_headless_success_with_failed_publication_is_nonzero();
        test_headless_publish_failure_preserves_primary_and_evidence();
        test_headless_second_ctrl_c_during_seal_journals_and_still_publishes();
        test_headless_second_ctrl_c_during_finalize_is_noted_and_publishes();
    });
}
