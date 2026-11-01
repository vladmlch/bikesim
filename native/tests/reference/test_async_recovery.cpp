// Port of tests/test_async_recovery.py — recovery ownership and cleanup
// controls at the PeriodAccounting/AsyncCloseRunner boundary. The Python
// doubles model publish_close; the native contract is detach_async():
// every preserved period is replayed once into a fresh evaluator seeded
// from the committed prefix, a strict rejection stops after applying its
// own period, a transport failure recovers pending + current exactly
// once, and teardown keeps the recovered rows.
//
// Divergences, all recorded in the ledger: the Python "interval
// constraints" map is the publication-local period_violations staging
// (cleared per applied publication, never carried from an old window);
// "failure before publication" maps to the strict-rejection throw —
// detached_at_ preserves the unapplied tail and collect() resumes it
// without repeating the committed prefix; and the multiprocessing
// _close_queues records are dropped (the native runner owns a single
// thread whose close() is noexcept — there is no multi-handle transport
// to drain).
#include "../support/migration_test.hpp"
#include "runtime/accounting.hpp"
#include "runtime/async_close.hpp"
#include "runtime/status.hpp"
#include <set>
#include <stdexcept>
#include <vector>

namespace {
using runtime::AccountingConfig;
using runtime::AccountingState;
using runtime::AsyncCloseRunner;
using runtime::AsyncPublicationError;
using runtime::InvalidReferenceRun;
using runtime::RawStep;
using runtime::ReferenceFailure;

// The synthetic rider-free raw — the test_async_close.cpp twin: no effort
// channel on a minimal accounting configuration.
[[nodiscard]] RawStep raw(std::int64_t i, double dt = .0005) {
    RawStep step;
    step.interval_id = i;
    step.time_s = static_cast<double>(i) * dt;
    step.end_time_s = static_cast<double>(i + 1) * dt;
    step.qpos.assign(4, 0.);
    step.qvel.assign(4, 0.);
    step.components = {{"chain", {1., 1., 1., 1.}}};
    step.qfrc_passive.assign(4, 0.);
    step.loss_step_j = .1;
    step.mechanical_energy_j = 99.5;
    step.elastic_energy_j = runtime::WireObject{};
    step.battery_energy_j = 0.;
    step.electrical_power_w = runtime::Wire{0.};
    step.numerical_constraint_power_w = {{"equality:0", .01}};
    step.diagnostics = {
        {"rider", runtime::Wire{runtime::WireObject{}}},
        {"rider_welds", runtime::Wire{runtime::WireObject{}}},
        {"endpoint_mass", runtime::Wire{runtime::WireObject{}}},
        {"rider_intent", runtime::Wire{nullptr}},
        {"inclination_rad", runtime::Wire{0.}},
        {"rider_allocation", runtime::Wire{runtime::WireObject{}}},
        {"rider_ik_saturation", runtime::Wire{runtime::WireObject{}}},
        {"rider_support_targets", runtime::Wire{runtime::WireObject{}}},
    };
    runtime::WireObject tires;
    const auto patch = [](double load) {
        return runtime::Wire{runtime::WireArray{runtime::Wire{runtime::WireObject{
            {"normal_load_n", runtime::Wire{load}},
            {"source_geom", runtime::Wire{"terrain"}}}}}};
    };
    tires.emplace_back("front", runtime::Wire{runtime::WireObject{{"patches", patch(50.)}}});
    tires.emplace_back("rear", runtime::Wire{runtime::WireObject{{"patches", patch(60.)}}});
    step.channels = {
        {"tires", runtime::Wire{std::move(tires)}},
        {"sensors", runtime::Wire{runtime::WireObject{}}},
    };
    step.constraint_snapshot.interval_start_s = step.time_s;
    step.constraint_snapshot.interval_end_s = step.end_time_s;
    step.constraint_snapshot.qvel_start = step.qvel;
    step.full = i % 10 == 9;
    return step;
}
[[nodiscard]] AccountingConfig config(bool asynchronous) {
    AccountingConfig cfg;
    cfg.timestep_s = .0005;
    cfg.period_steps = 10;
    cfg.measurement_decimation = 1;
    cfg.retention = runtime::RetentionMode::legacy;
    cfg.max_qpos = 4;
    cfg.max_dofs = 4;
    cfg.async_telemetry = asynchronous;
    return cfg;
}
[[nodiscard]] AccountingState state() {
    AccountingState initial;
    initial.energy.initial_energy_j = 100.;
    initial.energy.mechanical_energy_j = 100.;
    initial.energy.energy_scale_j = 50.;
    initial.energy.initial_battery_j = 0.;
    initial.energy.electrical_work_j = 0.;
    initial.energy.loss_j = 0.;
    initial.energy.active_work_j = 0.;
    initial.energy.external_work_j = 0.;
    initial.energy.solver_work_j = 0.;
    initial.energy.muscle_signed_j = 0.;
    initial.energy.muscle_positive_j = 0.;
    initial.energy.motor_signed_j = 0.;
    initial.energy.motor_positive_j = 0.;
    initial.energy.constraint_absolute_j = 0.;
    return initial;
}
[[noreturn]] inline void fail_worker(std::uint64_t, const char* message) {
    throw std::runtime_error(message);
}
void close_period(runtime::PeriodAccounting& accounting, std::int64_t first,
                  std::string_view failure = {}) {
    for (std::int64_t i = first; i < first + 10; ++i) {
        auto step = raw(i);
        if (!failure.empty()) step.attachment_errors = {std::string(failure)};
        accounting.push(std::move(step));
    }
    accounting.close_period();
}
[[nodiscard]] std::vector<std::int64_t> ids(
        std::span<const runtime::SamplePtr> samples) {
    std::vector<std::int64_t> out;
    out.reserve(samples.size());
    for (const auto& sample : samples) out.push_back(sample->interval_id);
    return out;
}
[[nodiscard]] std::vector<std::int64_t> range(std::int64_t count) {
    std::vector<std::int64_t> out(static_cast<std::size_t>(count));
    for (std::int64_t i = 0; i < count; ++i) out[static_cast<std::size_t>(i)] = i;
    return out;
}

// test_replay_retains_every_period_and_previously_published_prefix —
// detach_async() replays each preserved period once on top of the
// acknowledged prefix; the committed index keeps contiguous ids and the
// worker is gone.
[[gnu::noinline]] void replay_retains_every_period_and_prefix() {
    runtime::PeriodAccounting accounting(config(true), state(), 1, nullptr, nullptr);
    close_period(accounting, 0);
    accounting.collect(true);
    bt::require(ids(accounting.pending_samples()) == range(10),
        "the acknowledged prefix publishes first");
    close_period(accounting, 10);
    close_period(accounting, 20);
    close_period(accounting, 30);
    accounting.detach_async();
    bt::require(!accounting.asynchronous(),
        "detach disables async telemetry permanently");
    bt::require(ids(accounting.pending_samples()) == range(40),
        "every preserved period applies exactly once, in order");
    bt::require(accounting.latest_sample()->interval_id == 39,
        "the replay tail is the closing interval");
}

// test_replay_does_not_carry_constraints_from_an_old_publication_window —
// period_violations is publication-local: a stale entry seeded into the
// committed state is replaced, never merged, by the replay.
[[gnu::noinline]] void replay_drops_old_publication_window() {
    auto initial = state();
    initial.period_violations.push_back(
        ReferenceFailure{.time_s = -1., .violations = {"old-window"}});
    runtime::PeriodAccounting accounting(config(true), initial, 1, nullptr, nullptr);
    close_period(accounting, 0);
    close_period(accounting, 10);
    accounting.detach_async();
    bt::require(ids(accounting.pending_samples()) == range(20),
        "the replayed prefix applies once");
    bt::require(accounting.state().period_violations.empty(),
        "the old window's violations are not carried into the new publication");
}

// test_empty_replay_leaves_publication_state_untouched — detach with
// nothing pending neither publishes nor clears the committed window.
[[gnu::noinline]] void empty_replay_untouched() {
    auto initial = state();
    initial.period_violations.push_back(
        ReferenceFailure{.time_s = -1., .violations = {"old-window"}});
    runtime::PeriodAccounting accounting(config(true), initial, 1, nullptr, nullptr);
    accounting.detach_async();
    bt::require(accounting.pending_samples().empty(),
        "an empty replay publishes nothing");
    bt::require(accounting.state().period_violations.size() == 1 &&
        accounting.state().period_violations.front().violations ==
            std::vector<std::string>{"old-window"},
        "the committed window is left untouched");
    bt::require(!accounting.asynchronous(), "the worker is still released");
}

// test_strict_rejection_keeps_published_prefix_and_stops_before_later_periods
// + test_failure_before_publication_restores_prefix_without_repeating_it —
// a strict rejection surfaces where the bounded submit collect applies
// results: the rejected period's own publication commits (the monitor's
// first_failure is measured there) and the throw happens before the
// later period is submitted, so the buffered tail flushes through the
// live worker without ever re-publishing the prefix.
[[gnu::noinline]] void strict_rejection_keeps_prefix_and_resume() {
    auto initial = state();
    initial.monitor.strict = true;
    runtime::PeriodAccounting accounting(config(true), initial, 1, nullptr, nullptr);
    close_period(accounting, 0);
    close_period(accounting, 10, "forced-budget");
    bool rejected = false;
    try { close_period(accounting, 20); }
    catch (const InvalidReferenceRun& error) {
        rejected = std::string(error.what()).find("forced-budget") != std::string::npos;
    }
    bt::require(rejected, "the strict rejection propagates the measured failure");
    bt::require(ids(accounting.pending_samples()) == range(20),
        "the rejected period's own publication stays; the tail does not apply");
    const auto& first = accounting.state().monitor.first_failure;
    bt::require(first.has_value() &&
        first->violations == std::vector<std::string>{"forced-budget"},
        "the monitor records the rejected period's failure");
    accounting.flush(); // the buffered tail publishes — prefix never repeats
    bt::require(ids(accounting.pending_samples()) == range(30),
        "the resumed tail completes the period exactly once");
    bt::require(accounting.asynchronous(),
        "a strict rejection is not a transport failure — the worker lives on");
    accounting.detach_async();
    bt::require(!accounting.asynchronous(), "explicit detach still releases it");
}

// test_submission_recovery_publishes_pending_and_current_periods_once —
// the worker dies on its first job; the next submit hits the transport
// refusal (AsyncWorkerLost), the publication error still surfaces
// through collect, and detach_async() publishes the failed and current
// periods exactly once before synchronous closes continue.
[[gnu::noinline]] void submission_recovery_publishes_once() {
    runtime::PeriodAccounting accounting(config(true), state(), 1, nullptr, nullptr,
        [](std::uint64_t sequence) { fail_worker(sequence, "context serialization failed"); });
    close_period(accounting, 0); // queued; the worker fails inside it
    std::string caught;
    try { accounting.collect(true); } // synchronizes on the worker failure
    catch (const std::exception& error) { caught = error.what(); }
    bt::require(caught.find("async close worker failed") != std::string::npos,
        "the worker publication error surfaces once: " + caught);
    caught.clear();
    try { close_period(accounting, 10); } // submit refuses — worker lost
    catch (const std::exception& error) { caught = error.what(); }
    bt::require(caught.find("async close worker failed") != std::string::npos,
        "the preserved error surfaces again through the failed submit: " + caught);
    accounting.detach_async();
    bt::require(ids(accounting.pending_samples()) == range(10),
        "the failed job replays exactly once; the buffered current waits");
    accounting.flush(); // the still-buffered current period publishes
    bt::require(ids(accounting.pending_samples()) == range(20),
        "the current period publishes exactly once");
    bt::require(!accounting.asynchronous(), "the runner is released once");
    close_period(accounting, 20); // fully synchronous now
    bt::require(ids(accounting.pending_samples()) == range(30),
        "synchronous closes continue after recovery");
}

// Same recovery after an acknowledged prefix — the 'registration' phase:
// the worker dies on its second job and recovery replays from the point
// of failure without re-publishing the acknowledged window.
[[gnu::noinline]] void submission_recovery_after_prefix() {
    runtime::PeriodAccounting accounting(config(true), state(), 1, nullptr, nullptr,
        [](std::uint64_t sequence) {
            if (sequence == 1) throw std::runtime_error("period serialization failed");
        });
    close_period(accounting, 0);
    accounting.collect(true);
    bt::require(ids(accounting.pending_samples()) == range(10),
        "the acknowledged prefix commits");
    close_period(accounting, 10); // the worker fails inside this job
    std::string caught;
    try { accounting.collect(true); } // synchronizes on the job-1 failure
    catch (const std::exception& error) { caught = error.what(); }
    bt::require(caught.find("async close worker failed") != std::string::npos,
        "the job-1 publication failure surfaces: " + caught);
    caught.clear();
    try { close_period(accounting, 20); } // submit refuses — worker lost
    catch (const std::exception& error) { caught = error.what(); }
    bt::require(caught.find("async close worker failed") != std::string::npos,
        "the failure surfaces at the next submit: " + caught);
    accounting.detach_async();
    bt::require(ids(accounting.pending_samples()) == range(20),
        "the failed job replays once after the acknowledged prefix");
    accounting.flush();
    bt::require(ids(accounting.pending_samples()) == range(30),
        "the buffered current publishes — recovery never re-publishes");
}

// test_transport_recovery_preserves_every_period_without_a_second_replay
// + test_transport_cleanup_error_still_preserves_recovered_rows — after
// transport loss every preserved period lands exactly once, a second
// drain is a no-op, and runner teardown (noexcept — the Python OSError
// surface has no native counterpart) never disturbs the recovered rows.
[[gnu::noinline]] void transport_recovery_replays_once_and_teardown_keeps_rows() {
    runtime::PeriodAccounting accounting(config(true), state(), 1, nullptr, nullptr,
        [](std::uint64_t sequence) { fail_worker(sequence, "worker died"); });
    close_period(accounting, 0);
    std::string caught;
    try { accounting.collect(true); } // synchronizes on the worker failure
    catch (const std::exception& error) { caught = error.what(); }
    bt::require(caught.find("async close worker failed") != std::string::npos,
        "the transport failure surfaces: " + caught);
    caught.clear();
    try { close_period(accounting, 10); } // submit refuses — worker lost
    catch (const std::exception& error) { caught = error.what(); }
    bt::require(caught.find("async close worker failed") != std::string::npos,
        "the preserved failure surfaces at the refused submit: " + caught);
    accounting.detach_async();
    bt::require(ids(accounting.pending_samples()) == range(10),
        "the recovered job is preserved through teardown");
    accounting.flush(); // the buffered current publishes synchronously
    bt::require(ids(accounting.pending_samples()) == range(20),
        "every preserved period lands exactly once");
    accounting.collect(true); // the second drain publishes nothing
    bt::require(ids(accounting.pending_samples()) == range(20),
        "no second replay happens after recovery");
}
} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        replay_retains_every_period_and_prefix();
        replay_drops_old_publication_window();
        empty_replay_untouched();
        strict_rejection_keeps_prefix_and_resume();
        submission_recovery_publishes_once();
        submission_recovery_after_prefix();
        transport_recovery_replays_once_and_teardown_keeps_rows();
    });
}
