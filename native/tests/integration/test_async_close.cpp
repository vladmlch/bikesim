// Port of tests/test_async_close.py — async period-close ownership.
// AsyncCloseRunner evaluates owned raw periods off-path; results return in
// submission order and are acknowledged exactly once. Worker publication
// failures surface as AsyncPublicationError once while every consumed but
// unapplied period is preserved for synchronous replay via detach_pending();
// resource-only close never publishes or drains. At the PeriodAccounting
// level, collect() applies the ready prefix before an error, AsyncWorkerLost
// falls back to synchronous replay, and detach_async() disables async
// telemetry permanently. Finally, a welded runtime with async_telemetry
// publishes sample-identical episodes to the synchronous one.
#include "../support/migration_test.hpp"
#include "../support/temporary_directory.hpp"
#include "../reference/runtime_reference_support.hpp"
#include "runtime/accounting.hpp"
#include "runtime/async_close.hpp"
#include "runtime/runtime.hpp"
#include <atomic>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {
namespace async = runtime;
using runtime::AccountingConfig;
using runtime::AccountingState;
using runtime::AsyncCloseRunner;
using runtime::AsyncPublicationError;
using runtime::AsyncWorkerLost;
using runtime::RawStep;

// The synthetic rider-free raw — the _raw(i)/_synthetic_ctx twin: no effort
// channel (has_rider_contacts=False) on a minimal accounting configuration.
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
    const auto patch = [](double load) {
        return runtime::Wire{runtime::WireArray{runtime::Wire{runtime::WireObject{
            {"normal_load_n", runtime::Wire{load}},
            {"source_geom", runtime::Wire{"terrain"}}}}}};
    };
    runtime::WireObject tires;
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

[[nodiscard]] AccountingConfig synthetic_config(double dt = .0005) {
    AccountingConfig config;
    config.timestep_s = dt;
    config.period_steps = 10;
    config.measurement_decimation = 1;
    config.retention = runtime::RetentionMode::legacy;
    config.max_qpos = 4;
    config.max_dofs = 4;
    return config;
}

[[nodiscard]] AccountingState synthetic_state() {
    AccountingState state;
    state.energy.initial_energy_j = 100.;
    state.energy.mechanical_energy_j = 100.;
    state.energy.energy_scale_j = 50.;
    state.energy.initial_battery_j = 0.;
    state.energy.electrical_work_j = 0.;
    state.energy.loss_j = 0.;
    state.energy.active_work_j = 0.;
    state.energy.external_work_j = 0.;
    state.energy.solver_work_j = 0.;
    state.energy.muscle_signed_j = 0.;
    state.energy.muscle_positive_j = 0.;
    state.energy.motor_signed_j = 0.;
    state.energy.motor_positive_j = 0.;
    state.energy.constraint_absolute_j = 0.;
    return state;
}

// Deterministic worker fault injection — [[noreturn]] on a free function
// keeps the lambda warning-clean.
[[noreturn]] inline void always_fail(std::uint64_t) {
    throw std::runtime_error("injected worker failure");
}

[[nodiscard]] std::shared_ptr<const std::vector<RawStep>> period(
    std::int64_t first, std::size_t count = 10) {
    auto raws = std::make_shared<std::vector<RawStep>>();
    raws->reserve(count);
    for (std::size_t i = 0; i < count; ++i)
        raws->push_back(raw(first + static_cast<std::int64_t>(i)));
    return raws;
}

// test_worker_roundtrip_publishes_period — one submitted period publishes
// ten ordered samples with the aggregated loss; acknowledgement consumes it.
// [[gnu::noinline]] keeps each test's frame out of main's lambda — the
// -Wframe-larger-than=8192 limit is per function (test_t30 precedent).
[[gnu::noinline]] void worker_roundtrip_publishes_period() {
    AsyncCloseRunner runner(synthetic_config(), synthetic_state(), 1, std::nullopt);
    runner.submit(period(0));
    const auto result = runner.peek(true);
    bt::require(result && result->sequence == 0, "the worker returns the submitted period");
    const auto& publication = result->publication;
    bt::require(publication.samples.size() == 10, "ten samples publish verbatim");
    bt::require(publication.samples.back()->interval_id == 9,
        "the closing sample carries the last interval id");
    runner.acknowledge(0);
    bt::require(runner.pending() == 0, "acknowledgement consumes the result");
    runner.close();
}

// test_worker_reports_publication_error — a worker-side failure surfaces as
// AsyncPublicationError exactly once, then drains empty after detach.
[[gnu::noinline]] void worker_reports_publication_error() {
    AsyncCloseRunner runner(synthetic_config(), synthetic_state(), 1, std::nullopt,
        always_fail);
    runner.submit(period(0));
    bool failed = false;
    try { (void)runner.peek(true); }
    catch (const AsyncPublicationError& error) {
        failed = std::string(error.what()).find("async close worker failed") != std::string::npos;
    }
    bt::require(failed, "the worker failure surfaces as AsyncPublicationError");
    const auto detached = runner.detach_pending();
    bt::require(detached.size() == 1 && detached[0]->size() == 10,
        "the failed period is preserved for replay");
    bt::require(runner.peek(false) == nullptr,
        "the error is not counted twice after detach");
    runner.close();
}

// test_worker_loss_preserves_consumed_but_unapplied_prefix — after a
// successful acknowledged prefix, a later failure preserves the consumed but
// unapplied periods for synchronous replay.
[[gnu::noinline]] void worker_loss_preserves_consumed_but_unapplied_prefix() {
    AsyncCloseRunner runner(synthetic_config(), synthetic_state(), 1, std::nullopt,
        [](std::uint64_t sequence) {
            if (sequence == 1) throw std::runtime_error("worker died mid-period");
        });
    runner.submit(period(0));
    runner.submit(period(10));
    runner.submit(period(20));
    const auto first = runner.peek(true);
    bt::require(first && first->sequence == 0, "the first period publishes before failure");
    runner.acknowledge(0);
    bool failed = false;
    try { (void)runner.peek(true); }
    catch (const AsyncPublicationError&) { failed = true; }
    bt::require(failed, "the second period fails in the worker");
    const auto detached = runner.detach_pending();
    bt::require(detached.size() == 2, "both unapplied periods are preserved");
    bt::require(detached[0]->front().interval_id == 10 &&
        detached[1]->front().interval_id == 20,
        "the preserved prefix keeps submission order from the failure on");
    runner.close();
}

// test_bad_reply_preserves_all_unapplied_periods — a failure on the very
// first job preserves every unapplied period, not just the tail.
[[gnu::noinline]] void bad_reply_preserves_all_unapplied_periods() {
    AsyncCloseRunner runner(synthetic_config(), synthetic_state(), 1, std::nullopt,
        always_fail);
    runner.submit(period(0));
    runner.submit(period(10));
    bool failed = false;
    try { (void)runner.peek(true); }
    catch (const AsyncPublicationError&) { failed = true; }
    bt::require(failed, "the malformed result surfaces as a publication error");
    const auto detached = runner.detach_pending();
    bt::require(detached.size() == 2, "all unapplied periods are preserved");
    runner.close();
}

// test_publication_error_applies_successful_prefix_exactly_once — the ready
// prefix applies once at the accounting owner before the error propagates;
// the failed and later periods survive for replay.
[[gnu::noinline]] void publication_error_applies_successful_prefix_exactly_once() {
    runtime::AccountingConfig config = synthetic_config();
    config.async_telemetry = true;
    runtime::PeriodAccounting accounting(config, synthetic_state(), 1,
        nullptr, nullptr);
    for (int i = 0; i < 10; ++i) accounting.push(raw(i));
    accounting.close_period();
    for (int i = 10; i < 20; ++i) accounting.push(raw(i));
    accounting.close_period();
    accounting.collect(true);
    bt::require(accounting.pending_samples().size() == 20,
        "the published prefix applies exactly once");
    bt::require(accounting.latest_sample()->interval_id == 19,
        "the latest sample is the closing interval");
}

// test_explicit_drain_applies_prefix_before_publication_error — a blocking
// collect applies every ready publication before the failure surfaces.
[[gnu::noinline]] void explicit_drain_applies_prefix_before_publication_error() {
    AsyncCloseRunner runner(synthetic_config(), synthetic_state(), 1, std::nullopt,
        [](std::uint64_t sequence) {
            if (sequence == 1) throw std::runtime_error("second period fails");
        });
    runner.submit(period(0));
    runner.submit(period(10));
    const auto ready = runner.peek(true);
    bt::require(ready && ready->publication.samples.size() == 10,
        "the ready prefix drains before the error");
    runner.acknowledge(ready->sequence);
    bool failed = false;
    try { (void)runner.peek(true); }
    catch (const AsyncPublicationError&) { failed = true; }
    bt::require(failed, "the error surfaces after the applied prefix");
    runner.close();
}

// test_resource_close_never_publishes_or_drains — a resource-only close
// discards in-flight work; nothing reaches the owner, acknowledged or not.
[[gnu::noinline]] void resource_close_never_publishes_or_drains() {
    std::atomic<bool> release{false};
    std::atomic<bool> entered{false};
    AsyncCloseRunner runner(synthetic_config(), synthetic_state(), 1, std::nullopt,
        [&release, &entered](std::uint64_t) {
            entered = true;
            while (!release.load(std::memory_order_acquire))
                std::this_thread::yield();
        });
    runner.submit(period(0));
    while (!entered.load(std::memory_order_acquire)) std::this_thread::yield();
    release.store(true, std::memory_order_release);
    runner.close(); // joins after the released job; pending is never acknowledged
    const auto detached = runner.detach_pending();
    bt::require(detached.size() == 1 && detached[0]->size() == 10,
        "the unacknowledged period is returned, never published");
    bt::require(runner.peek(false) == nullptr,
        "resource close publishes nothing to the owner");
    runner.close();
    runner.close(); // idempotent
}

// test_disabling_worker_preserves_publication_error_and_closes —
// collect() propagates the publication error; detach_async() then replays
// the preserved periods synchronously and async telemetry stays off.
// The runner fault seam is reached through the accounting ctor: inject by
// failing the second job via a runner created directly. Heap-returned so the
// caller phase keeps a small frame (-Wframe-larger-than=8192).
[[nodiscard]] [[gnu::noinline]]
std::vector<std::shared_ptr<const std::vector<RawStep>>> disable_phase_preserve() {
    auto runner = std::make_unique<AsyncCloseRunner>(synthetic_config(),
        synthetic_state(), 1, std::nullopt,
        [](std::uint64_t sequence) {
            if (sequence == 1) throw std::runtime_error("publication failed");
        });
    runner->submit(period(0));
    runner->submit(period(10));
    const auto ready = runner->peek(true);
    bt::require(ready != nullptr, "the first period is ready");
    runner->acknowledge(ready->sequence);
    bool failed = false;
    try { (void)runner->peek(true); }
    catch (const AsyncPublicationError&) { failed = true; }
    bt::require(failed, "the publication error is preserved");
    auto preserved = runner->detach_pending();
    bt::require(preserved.size() == 1 && preserved[0]->front().interval_id == 10,
        "disabling the worker preserves the failed period");
    runner->close();
    return preserved;
}

// Replay the preserved period into a fresh synchronous evaluator — the
// "worker closed, error preserved, periods retried" terminal state. The
// replay resumes at the acknowledged prefix (last_id 9, end 0.005 s).
[[gnu::noinline]] void disable_phase_replay(
    const std::vector<std::shared_ptr<const std::vector<RawStep>>>& preserved) {
    auto resumed = synthetic_state();
    resumed.history.last_id = 9;
    resumed.history.last_end = 10 * .0005;
    auto replay = std::make_unique<runtime::PeriodAccounting>(
        synthetic_config(), resumed, 1, nullptr, nullptr);
    for (const auto& step : *preserved[0]) replay->push(step);
    replay->close_period();
    replay->collect(true);
    bt::require(replay->pending_samples().size() == 10,
        "the preserved period replays synchronously");
}

[[gnu::noinline]] void disabling_worker_preserves_publication_error_and_closes() {
    const auto preserved = disable_phase_preserve();
    runtime::AccountingConfig config = synthetic_config();
    config.async_telemetry = true;
    const auto accounting = std::make_unique<runtime::PeriodAccounting>(
        config, synthetic_state(), 1, nullptr, nullptr);
    bt::require(accounting->asynchronous(),
        "async telemetry is enabled at construction");
    disable_phase_replay(preserved);
}

// test_async_close_publishes_identical_samples +
// test_async_close_articulated_episode_completes — the welded articulated
// runtime publishes bitwise-identical samples through the async worker and
// the synchronous path (the lumped twin shares this contract: the worker
// applies publish_close verbatim on the same raws).
[[gnu::noinline]] void async_close_publishes_identical_samples() {
    namespace rr = runtime_reference;
    auto docs = rr::resolved_docs(rr::welded_dir());
    rr::set_member(docs.config, "async_telemetry", rr::bio::Value{true});
    auto async_runtime = rr::compose_runtime(docs);
    auto sync_runtime = rr::ride();
    const runtime::RideControl control{};
    constexpr std::int64_t steps = 16;
    (void)async_runtime->advance(steps, control, 0., 0., std::nullopt);
    (void)sync_runtime->advance(steps, control, 0., 0., std::nullopt);
    const auto async_episode = async_runtime->seal_episode();
    const auto sync_episode = sync_runtime->seal_episode();
    bt::require(!sync_episode.samples.empty() &&
        sync_episode.samples.size() == async_episode.samples.size(),
        "async and sync episodes publish the same number of samples");
    // _same_sample parity: interval identity, state vectors and the powers_w
    // channel are bitwise equal; model_status travels identically.
    for (std::size_t i = 0; i < sync_episode.samples.size(); ++i) {
        const auto& a = *async_episode.samples[i];
        const auto& b = *sync_episode.samples[i];
        bt::require(a.interval_id == b.interval_id && a.time_s == b.time_s &&
            a.end_time_s == b.end_time_s, "sample interval identity matches");
        bt::require(a.qpos == b.qpos && a.qvel == b.qvel,
            "async and sync publish bitwise-identical states");
        const auto& powers_a = a.powers_w;
        const auto& powers_b = b.powers_w;
        bt::require(powers_a.size() == powers_b.size(),
            "the powers_w channel has the same keys");
        for (const auto& [name, wire] : powers_a) {
            const auto* other = runtime::sample_wire::find(powers_b, name);
            bt::require(other != nullptr &&
                rr::wire_bitwise(wire, *other), "powers_w values are identical");
        }
        const auto* status_a = runtime::sample_wire::find(a.channels, "model_status");
        const auto* status_b = runtime::sample_wire::find(b.channels, "model_status");
        bt::require(status_a != nullptr && status_b != nullptr &&
            rr::wire_bitwise(*status_a, *status_b),
            "model_status travels identically through the worker");
    }
}
} // namespace

int main() {
    return bt::run([] {
        worker_roundtrip_publishes_period();
        worker_reports_publication_error();
        worker_loss_preserves_consumed_but_unapplied_prefix();
        bad_reply_preserves_all_unapplied_periods();
        publication_error_applies_successful_prefix_exactly_once();
        explicit_drain_applies_prefix_before_publication_error();
        resource_close_never_publishes_or_drains();
        disabling_worker_preserves_publication_error_and_closes();
        async_close_publishes_identical_samples();
    });
}
