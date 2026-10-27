// Completion 3.30 (T53). Source: tests/reference/test_recording_capture.py.
//
// RunCapture / EpisodeJournal / InterruptShield contract ports. Every
// comparison here is a self-contained contract or invariant — the oracle
// asserted against literal structure, never captured values — so no
// platform goldens exist for this file.
//
// Dropped nodes (python-only): test_metadata_must_be_a_run_metadata —
// the rejected inputs are Python runtime type states (dict where a
// RunMetadata belongs, str where a bool belongs); the C++ signature
// carries both as types. Field/domain validation ports stay.
#include "recording_test_support.hpp"
#include "support/temporary_directory.hpp"
#include "diag.hpp"
#include "platform/signals.hpp"
#include "recording/publish.hpp"
#include "recording/reader.hpp"
#include "research/lifecycle.hpp"
#include <cmath>
#include <csignal>
#include <thread>

namespace {

using namespace recording_test;
namespace rt = recording_test;
namespace value = rec::value;

// The exception RunCapture raises for appends outside `collecting`
// (physical_frontend.py's RuntimeError surface).
template <class F> void logic_rejects(F &&action, std::string_view message) {
    throws<std::logic_error>(std::forward<F>(action), "", message);
}

void test_disabled_capture_has_no_persistent_side_effect() {
    bt::TemporaryDirectory root("capture-disabled");
    const auto target = root.path() / "not-created";
    rec::RunCapture capture(run_metadata(), false);
    const auto publication = rec::finalize(capture, target);
    bt::require(publication.directory.empty() && publication.report_status == "disabled",
                "disabled finalize publishes nothing");
    bt::require(!std::filesystem::exists(target),
                "disabled finalize creates no output");
    bt::require(capture.state() == rec::RunCapture::State::published,
                "disabled capture reports published without a path");
}

void test_disabled_capture_still_validates_but_retains_nothing() {
    bt::TemporaryDirectory root("capture-disabled-validate");
    rec::RunCapture capture(run_metadata(), false);
    capture.event(input_event(0, 0));
    capture.add_episode(make_evidence(0, 1, {sample_row(0, 0.)}));
    bt::require(capture.events().empty() && capture.episodes().empty(),
                "disabled capture retains nothing");
    throws<std::invalid_argument>([&] { capture.event(input_event(0, 0)); }, "",
                                  "ordering rules apply when disabled too");
    const auto publication = rec::finalize(capture, root.path() / "run");
    bt::require(publication.directory.empty() &&
                !std::filesystem::exists(root.path() / "run"),
                "disabled finalize still creates nothing");
}

void test_conflicting_episode_ids_reject_atomically() {
    rec::RunCapture capture(run_metadata(), true);
    const auto first = make_evidence(0, 1, {sample_row(0, 0.)});
    capture.add_episode(first);
    const auto conflicting = make_evidence(0, 1, {sample_row(0, 0.), sample_row(1, .00125)});
    throws<std::invalid_argument>([&] { capture.add_episode(conflicting); }, "conflict",
                                  "conflicting evidence rejected");
    // The capture freezes its own immutable copy; the original evidence
    // stays intact and its physical source is the retained one.
    bt::require(capture.episodes().size() == 1 &&
                capture.episodes()[0]->episode_id == first->episode_id &&
                capture.episodes()[0]->physical.get() == first->physical.get(),
                "the failed addition changed nothing");
}

void test_identical_episode_readdition_is_idempotent() {
    rec::RunCapture capture(run_metadata(), true);
    const auto evidence = make_evidence(0, 1, {sample_row(0, 0.)});
    capture.add_episode(evidence);
    capture.add_episode(evidence);
    bt::require(capture.episodes().size() == 1 &&
                capture.episodes()[0]->episode_id == 0 &&
                capture.episodes()[0]->physical.get() == evidence->physical.get(),
                "same evidence pointer re-adds idempotently");
}

void test_episode_identifiers_are_never_reused() {
    rec::RunCapture capture(run_metadata(), true);
    capture.add_episode(make_evidence(1, 1, {sample_row(0, 0.)}));
    throws<std::invalid_argument>([&] {
        capture.add_episode(make_evidence(0, 1, {sample_row(0, 0.)})); }, "",
        "an identifier before the committed base is never reused");
    bt::require(capture.episodes().size() == 1, "rejection committed nothing");
}

void test_episode_prefix_continuity_rejects_gaps_atomically() {
    rec::RunCapture capture(run_metadata(), true);
    const auto first = make_evidence(1, 1, {sample_row(0, 0.)});
    capture.add_episode(first);
    throws<std::invalid_argument>([&] {
        capture.add_episode(make_evidence(3, 1, {sample_row(0, 0.)})); }, "continuity",
        "a gap in the prefix is rejected");
    bt::require(capture.episodes().size() == 1 &&
                capture.episodes()[0]->episode_id == first->episode_id &&
                capture.episodes()[0]->physical.get() == first->physical.get(),
                "the failed addition committed nothing");
}

void test_episode_ids_need_not_start_at_one() {
    rec::RunCapture capture(run_metadata(), true);
    for (const std::int64_t id : {5, 6, 7})
        capture.add_episode(make_evidence(id, 1, {sample_row(0, 0.)}));
    bt::require(capture.episodes().size() == 3 &&
                capture.episodes()[0]->episode_id == 5 &&
                capture.episodes()[2]->episode_id == 7,
                "the committed base can be nonzero");
}

void test_event_identity_and_order_are_validated() {
    rec::RunCapture capture(run_metadata(), true);
    capture.event(input_event(0, 0));
    capture.event(input_event(0, 1));
    capture.event(input_event(1, 0)); // a reset-separated episode restarts ids
    capture.event(input_event(1, 3));
    bt::require(capture.events().size() == 4 && capture.events()[3].event_id == 3,
                "per-episode event cursors with gaps accept monotonic order");
    throws<std::invalid_argument>([&] { capture.event(input_event(0, 5)); }, "",
                                  "an earlier episode is closed");
    throws<std::invalid_argument>([&] { capture.event(input_event(1, 3)); }, "",
                                  "duplicate identity rejected");
    throws<std::invalid_argument>([&] { capture.event(input_event(1, 2)); }, "",
                                  "out-of-order event id rejected");
}

void test_episode_journal_owns_contiguous_ids_per_episode() {
    rec::RunCapture capture(run_metadata(), true);
    rec::EpisodeJournal journal(capture, 0);
    bt::require(journal.episode_id() == 0 && journal.next_id() == 0,
                "journal starts its cursor at 0");
    const auto emitted = journal.emit("episode_start", 1., 0);
    bt::require(emitted.event_id == 0 && emitted.kind == "episode_start",
                "emit assigns the journal's own id");
    (void)journal.emit("brake_toggle", 1.1, 4, 4,
                       Value{Value::Object{{"record", text("requested")}}});
    bt::require(journal.next_id() == 2 &&
                capture.events()[0].event_id == 0 && capture.events()[1].event_id == 1,
                "journal ids stay contiguous");
    {
        const std::array bulk{input_event(0, 2, "derived_transition"),
                              input_event(0, 3, "derived_transition")};
        journal.emit_all(bulk);
    }
    bt::require(journal.next_id() == 4 &&
                capture.events()[2].kind == "derived_transition" &&
                capture.events()[3].kind == "derived_transition",
                "emit_all journals a pre-built contiguous sequence");
    rec::EpisodeJournal other(capture, 1);
    (void)other.emit("episode_start", 2., 0);
    bt::require(capture.events().back().episode_id == 1,
                "a new episode's journal restarts its own cursor");
}

void test_episode_journal_rejects_non_contiguous_bulk_emission() {
    rec::RunCapture capture(run_metadata(), true);
    rec::EpisodeJournal journal(capture, 0);
    (void)journal.emit("episode_start", 1., 0);
    throws<std::invalid_argument>([&] {
            const std::array wrong{input_event(1, 1)};
            journal.emit_all(wrong); },
        "continue journal", "wrong episode rejected");
    throws<std::invalid_argument>([&] {
            const std::array skipped{input_event(0, 2)};
            journal.emit_all(skipped); },
        "continue journal", "skipped id rejected");
    bt::require(journal.next_id() == 1, "rejection leaves the cursor unchanged");
    throws<std::invalid_argument>([&] {
            const std::array bulk{input_event(0, 1), input_event(0, 3)};
            journal.emit_all(bulk); },
        "continue journal", "mid-sequence rejection commits nothing past the prefix");
    bt::require(journal.next_id() == 2, "the valid prefix still committed");
}

void test_episode_journal_validates_its_construction() {
    rec::RunCapture capture(run_metadata(), true);
    // `EpisodeJournal(object(), 0)` and `EpisodeJournal(capture, True)` are
    // Python type-domain rejections; C++ carries them as types.
    throws<std::invalid_argument>([&] { rec::EpisodeJournal bad(capture, -1); }, "",
                                  "negative episode id rejected");
}

void test_event_field_validation() {
    rec::RunCapture capture(run_metadata(), true);
    throws<std::invalid_argument>([&] {
        capture.event(input_event(0, 0, "brake", std::nan(""))); }, "",
        "received_wall_s must be finite");
    auto applied_negative = input_event(0, 0);
    applied_negative.applied_step = -1;
    throws<std::invalid_argument>([&] { capture.event(applied_negative); }, "",
                                  "applied_step must be nonnegative");
    auto applied_before = input_event(0, 0, "brake", 0., 4);
    applied_before.applied_step = 2;
    throws<std::invalid_argument>([&] { capture.event(applied_before); }, "",
                                  "applied_step precedes processed_step");
    throws<std::invalid_argument>([&] {
        rec::InputEvent empty_kind;
        empty_kind.episode_id = 0; empty_kind.event_id = 0;
        empty_kind.received_wall_s = 0.; empty_kind.processed_step = 0;
        empty_kind.validate(); }, "", "kind must be nonempty");
    throws<std::invalid_argument>([&] { capture.event(input_event(-1, 0)); }, "",
                                  "episode_id must be nonnegative");
    capture.event(input_event(0, 0)); // requested but never applied
}

void test_zero_interval_episode_evidence_is_valid() {
    rec::RunCapture capture(run_metadata(), true);
    const auto empty = make_evidence(0, 1, {});
    bt::require(empty->physical->row_count() == 0, "zero-row evidence is valid");
    capture.add_episode(empty);
    bt::require(capture.episodes().size() == 1 &&
                capture.episodes()[0]->physical.get() == empty->physical.get(),
                "zero-interval evidence commits");
}

void test_episode_evidence_contract_is_validated() {
    rec::RunCapture capture(run_metadata(), true);
    auto negative = std::make_shared<rec::EpisodeEvidence>(*make_evidence(0));
    negative->episode_id = -1;
    throws<std::invalid_argument>([&] { capture.add_episode(negative); }, "",
                                  "negative episode id rejected");
    auto null_source = std::make_shared<rec::EpisodeEvidence>(*make_evidence(1));
    null_source->physical = nullptr;
    throws<std::invalid_argument>([&] { capture.add_episode(null_source); }, "",
                                  "evidence requires a physical owner");
    auto bogus_research = std::make_shared<rec::EpisodeEvidence>(*make_evidence(2));
    bogus_research->research = rec::ResearchEvidence{}; // invalid ownership/domain
    throws<std::invalid_argument>([&] { capture.add_episode(bogus_research); }, "",
                                  "research evidence validates its contract");
}

void test_nested_mutable_source_input_cannot_modify_captured_metadata() {
    // io::Value is an owned tree: the capture copies it at construction, so
    // mutating the source document can never rewrite captured metadata
    // (the oracle's MappingProxyType freeze is static in C++).
    const auto terrain_slot = [](Value::Object &object) -> Value & {
        return const_cast<Value &>(*io::find_member(object, "terrain"));
    };
    const auto grade_slot = [](Value &terrain) -> Value::Array & {
        return const_cast<Value::Array &>(io::as_array(
            *io::find_member(io::as_object(terrain, "terrain"), "grade"), "grade"));
    };
    Value nested{Value::Object{
        {"terrain", Value{Value::Object{{"grade", Value{Value::Array{Value{io::Number{"0.1"}},
                                                                       Value{io::Number{"0.2"}}}}}}}},
        {"seed", Value{io::Number{"19"}}}}};
    rec::RunMetadata metadata = run_metadata("native", "ride");
    metadata.values = nested;
    rec::RunCapture capture(std::move(metadata), true);
    // Mutate the caller-side tree after construction input was staged.
    auto &nested_object = const_cast<Value::Object &>(io::as_object(nested, "nested"));
    grade_slot(terrain_slot(nested_object))[0] = Value{io::Number{"9.9"}};
    const_cast<Value &>(*io::find_member(nested_object, "seed")) = Value{io::Number{"-1"}};
    nested_object.emplace_back("new_key", Value{true});
    const auto &captured = io::as_object(capture.metadata().values, "metadata.values");
    const auto &captured_grade = io::as_array(*io::find_member(
        io::as_object(*io::find_member(captured, "terrain"), "terrain"), "grade"), "grade");
    bt::require(as_real(captured_grade[0]) == 0.1 &&
                as_integer(*io::find_member(captured, "seed")) == 19 &&
                io::find_member(captured, "new_key") == nullptr,
                "captured metadata is an owned snapshot");
}

void test_event_and_summary_values_are_frozen() {
    Value values{Value::Object{
        {"request", Value{Value::Object{{"brake", Value{Value::Array{Value{io::Number{"0.5"}}}}}}}}}};
    rec::InputEvent event = input_event(0, 0, "brake", 0., 0, values);
    // Mutate the source after staging — the event owns its copy.
    auto &request = const_cast<Value::Object &>(io::as_object(
        *io::find_member(const_cast<Value::Object &>(io::as_object(values, "values")), "request"), "request"));
    const_cast<Value::Array &>(io::as_array(*io::find_member(request, "brake"), "brake"))
        .push_back(Value{io::Number{"1.0"}});
    const auto &captured = io::as_array(*io::find_member(
        io::as_object(*io::find_member(io::as_object(event.values, "event.values"), "request"), "request"),
        "brake"), "brake");
    bt::require(captured.size() == 1 && as_real(captured[0]) == 0.5,
                "event values are an owned snapshot");
}

void test_finalize_publishes_once_and_retries_failed_publication() {
    bt::TemporaryDirectory root("capture-finalize-once");
    rec::RunCapture capture(run_metadata(), true);
    capture.add_episode(make_evidence(0, 1, {sample_row(0, 0.)}));
    bt::require(capture.state() == rec::RunCapture::State::collecting,
                "pre-finalize state is collecting");
    const auto first = rec::finalize(capture, root.path());
    bt::require(capture.state() == rec::RunCapture::State::published &&
                !first.directory.empty() &&
                std::filesystem::exists(first.directory / "manifest.json"),
                "first finalize seals a manifest");
    const auto second = rec::finalize(capture, root.path());
    std::size_t sealed = 0;
    for (const auto &entry : std::filesystem::directory_iterator(root.path()))
        if (std::filesystem::exists(entry.path() / "manifest.json")) ++sealed;
    bt::require(second.directory == first.directory &&
                second.report_status == first.report_status && sealed == 1,
                "repeated finalize returns the cached publication without re-publishing");
}

void test_failed_publication_keeps_capture_retryable() {
    bt::TemporaryDirectory root("capture-finalize-retry");
    rec::RunCapture capture(run_metadata(), true);
    capture.add_episode(make_evidence(0, 1, {sample_row(0, 0.)}));
    int attempts = 0;
    rec::FinalizeOptions options;
    options.before_stage = [&](std::string_view stage) {
        if (stage == "manifest_link" && ++attempts == 1)
            throw std::runtime_error("injected publication failure");
    };
    throws<std::runtime_error>([&] { (void)rec::finalize(capture, root.path(), options); },
                               "injected", "first publication fails");
    bt::require(capture.state() == rec::RunCapture::State::publication_failed,
                "failed publication is retryable");
    const auto publication = rec::finalize(capture, root.path(), options);
    bt::require(capture.state() == rec::RunCapture::State::published &&
                std::filesystem::exists(publication.directory / "manifest.json"),
                "the retried finalize seals with intact evidence");
}

void test_finalize_uses_default_duckdb_publisher() {
    bt::TemporaryDirectory root("capture-finalize-duckdb");
    rec::RunCapture capture(run_metadata(), true);
    // The oracle sealed a zero-episode launch; the default DuckDB
    // publication still writes a readable recording.
    const auto publication = rec::finalize(capture, root.path() / "run");
    bt::require(capture.state() == rec::RunCapture::State::published, "finalize publishes");
    rec::Recording saved(publication.directory);
    bt::require(saved.episode_ids().empty(),
                "the sealed recording exposes the empty episode set");
}

void test_appends_are_rejected_after_publication() {
    bt::TemporaryDirectory root("capture-append-after-publish");
    rec::RunCapture capture(run_metadata(), true);
    capture.add_episode(make_evidence(0, 1, {sample_row(0, 0.)}));
    (void)rec::finalize(capture, root.path());
    logic_rejects([&] { capture.event(input_event(0, 0)); },
                  "events reject after publication");
    logic_rejects([&] { capture.add_episode(make_evidence(1, 1, {})); },
                  "episodes reject after publication");
    rec::RunCapture failed(run_metadata(), true);
    failed.add_episode(make_evidence(0, 1, {}));
    rec::FinalizeOptions options;
    options.before_stage = [](std::string_view stage) {
        if (stage == "manifest_link") throw std::runtime_error("injected publisher failure");
    };
    throws<std::runtime_error>([&] { (void)rec::finalize(failed, root.path() / "x", options); },
                               "injected", "publication failure propagates");
    logic_rejects([&] { failed.event(input_event(0, 0)); },
                  "events reject after a failed publication too");
}

void test_interrupt_shield_defers_sigint_and_restores_handler() {
    // The counting handler is installed on the main thread; raising SIGINT
    // inside the scope records instead of raising. `previous` is the
    // disposition the shield must restore on exit.
    const auto previous = std::signal(SIGINT, SIG_DFL);
    {
        bike::platform::InterruptShield shield;
        bt::require(shield.installed(), "main-thread shield installs the counting handler");
        (void)std::raise(SIGINT);
        bt::require(shield.deferred() == 1, "the shielded Ctrl-C is counted, not raised");
        (void)std::raise(SIGINT);
        bt::require(shield.deferred() == 2, "repeated SIGINT only defers again");
    }
    bt::require(std::signal(SIGINT, previous) == SIG_DFL,
                "the previous disposition is restored on exit");
}

void test_interrupt_shield_restores_on_body_failure() {
    const auto previous = std::signal(SIGINT, SIG_DFL);
    bool raised = false;
    try {
        bike::platform::InterruptShield shield;
        throw std::runtime_error("boom");
    } catch (const std::runtime_error &) { raised = true; }
    bt::require(raised, "the shielded body still propagates its failure");
    bt::require(std::signal(SIGINT, previous) == SIG_DFL,
                "the previous disposition is restored on unwind");
}

void test_interrupt_shield_is_a_flag_holder_off_main_thread() {
    // Off the main thread the shield degrades to a flag holder: no handler
    // is claimed, note() still records, and the section still runs.
    const auto previous = std::signal(SIGINT, SIG_DFL);
    std::atomic<bool> installed{true}, done{false};
    volatile std::sig_atomic_t deferred = -1;
    std::thread worker([&] {
        bike::platform::InterruptShield shield;
        installed = shield.installed();
        shield.note();
        deferred = static_cast<std::sig_atomic_t>(shield.deferred());
        done = true;
    });
    worker.join();
    bt::require(done && !installed.load() && deferred == 1,
                "off-main shield is a flag holder that still records");
    bt::require(std::signal(SIGINT, previous) == SIG_DFL,
                "the process disposition was never claimed off-main");
}

} // namespace

int main() {
    return bt::run([] {
        test_disabled_capture_has_no_persistent_side_effect();
        test_disabled_capture_still_validates_but_retains_nothing();
        test_conflicting_episode_ids_reject_atomically();
        test_identical_episode_readdition_is_idempotent();
        test_episode_identifiers_are_never_reused();
        test_episode_prefix_continuity_rejects_gaps_atomically();
        test_episode_ids_need_not_start_at_one();
        test_event_identity_and_order_are_validated();
        test_episode_journal_owns_contiguous_ids_per_episode();
        test_episode_journal_rejects_non_contiguous_bulk_emission();
        test_episode_journal_validates_its_construction();
        test_event_field_validation();
        test_zero_interval_episode_evidence_is_valid();
        test_episode_evidence_contract_is_validated();
        test_nested_mutable_source_input_cannot_modify_captured_metadata();
        test_event_and_summary_values_are_frozen();
        test_finalize_publishes_once_and_retries_failed_publication();
        test_failed_publication_keeps_capture_retryable();
        test_finalize_uses_default_duckdb_publisher();
        test_appends_are_rejected_after_publication();
        test_interrupt_shield_defers_sigint_and_restores_handler();
        test_interrupt_shield_restores_on_body_failure();
        test_interrupt_shield_is_a_flag_holder_off_main_thread();
    });
}
