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
#include "research/environment.hpp"
#include "research/lifecycle.hpp"
#include "research/policy_session.hpp"
#include "runtime/accounting.hpp"
#include "runtime/async_close.hpp"
#include "runtime/research_io.hpp"
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

// --- research owner / driver close records -----------------------------------
// The Python adapter's _window/_closed/_replay_assets_owner session state maps
// onto PolicySession (policy + environment owners), the unconditional
// NativeResearchRuntime::close discard, and the FailureReport note plumbing in
// research/lifecycle. AsyncCloseRunner is constructed inside PeriodAccounting
// only when async_telemetry is set — that is the "owns only needed worker"
// boundary the [native-False] driver record pins.
namespace res = bike::research;

// The failing worker-close owner: close() throws once; the call count proves
// whether a close path retried the failed owner.
class FailingClosePolicy final : public res::Policy {
public:
    explicit FailingClosePolicy(std::shared_ptr<int> closes) : closes_(std::move(closes)) {}
    ~FailingClosePolicy() override;
    FailingClosePolicy(const FailingClosePolicy&) = delete;
    FailingClosePolicy& operator=(const FailingClosePolicy&) = delete;
    FailingClosePolicy(FailingClosePolicy&&) = delete;
    FailingClosePolicy& operator=(FailingClosePolicy&&) = delete;
    void reset(std::string_view) override {}
    [[nodiscard]] runtime::RideControl act(const runtime::SensorObservation&,
                                           std::optional<double>) override {
        runtime::RideControl control;
        control.motor_torque_nm = 0.;
        return control;
    }
    void close() override {
        ++*closes_;
        throw std::runtime_error("worker cleanup");
    }
private:
    std::shared_ptr<int> closes_;
};
FailingClosePolicy::~FailingClosePolicy() = default;

class ResetFailurePolicy final : public res::Policy {
public:
    ~ResetFailurePolicy() override;
    ResetFailurePolicy(const ResetFailurePolicy&) = delete;
    ResetFailurePolicy& operator=(const ResetFailurePolicy&) = delete;
    ResetFailurePolicy(ResetFailurePolicy&&) = delete;
    ResetFailurePolicy& operator=(ResetFailurePolicy&&) = delete;
    ResetFailurePolicy() = default;
    [[noreturn]] void reset(std::string_view) override {
        throw std::runtime_error("injected constructor reset failure");
    }
    [[noreturn]] runtime::RideControl act(const runtime::SensorObservation&,
                                          std::optional<double>) override {
        throw std::runtime_error("act must not run after reset failure");
    }
};
ResetFailurePolicy::~ResetFailurePolicy() = default;

[[nodiscard]] [[gnu::noinline]]
std::unique_ptr<res::PreparedEnvironment> research_seed(bool async_telemetry) {
    auto request = std::make_unique<res::ResearchRequest>(
        res::read_research_request(bt::fixture("requests/research_delay.toml")));
    request->experiment.async_telemetry = async_telemetry;
    return res::prepare_environment(std::move(request));
}

// test_research_close_keeps_active_window_guard_and_discards_without_steps —
// the adapter's 'active interval' guard lives on PolicySession::close(): an
// accepted but unfinished window makes close() throw before either owner runs,
// the window and worker survive, and the unconditional environment close is
// the discard_pending twin — it abandons the pending interval without
// consuming steps.
[[gnu::noinline]] void research_close_keeps_active_window_guard_and_discards_without_steps() {
    const auto closes = std::make_shared<int>(0);
    auto seed = research_seed(false);
    res::PolicySession session(runtime::make_research(seed->bootstrap),
        std::make_unique<FailingClosePolicy>(closes));
    session.begin_advance();
    bt::require(session.pending(), "the accepted policy interval is active");
    bool guarded = false;
    try { session.close(); }
    catch (const std::runtime_error& error) {
        guarded = std::string(error.what()).find("policy interval") != std::string::npos;
    }
    bt::require(guarded && *closes == 0,
        "the window guard fires before either owner close runs");
    bt::require(session.pending() && !session.environment().closed(),
        "the window and the environment survive the rejected close");
    bt::require(session.environment().snapshot().step == 0,
        "the pending window consumed no steps");
    session.environment().close();
    bt::require(session.environment().closed(),
        "the discard close releases the environment without finishing the window");
}

// test_research_close_attempts_both_owners_preserving_first_failure —
// PolicySession::close() attempts policy->close() then environment->close(),
// rethrows the first failure and records a second as a cleanup note. The
// secondary-note arm ("environment close also failed") is reachable only
// through a re-entrant environment close (ResearchGuard) — no public seam
// injects it; the close_environment wrapper proves primary preservation and
// the cleanup_notes->secondary merge instead. Where the oracle retries a
// failed owner, the native close latches closed: failure is recorded, not
// retried.
// close_environment lifts a failing session close into FailureReport: the
// first failure stays rethrowable and session cleanup notes append as
// secondaries (the oracle's exception __notes__ twin). noinline keeps the
// session frame out of the caller (-Wframe-larger-than=8192).
[[gnu::noinline]] void close_environment_wraps_first_failure() {
    const auto closes = std::make_shared<int>(0);
    auto seed = research_seed(false);
    res::PolicySession session(runtime::make_research(seed->bootstrap),
        std::make_unique<FailingClosePolicy>(closes));
    res::FailureReport report;
    res::close_environment(session, report);
    bool preserved = false;
    try { report.rethrow_if_failed(); }
    catch (const std::runtime_error& error) {
        preserved = std::string(error.what()).find("worker cleanup") != std::string::npos;
    }
    bt::require(preserved && session.environment().closed(),
        "the report preserves the first failure after attempting both owners");
}

[[gnu::noinline]] void research_close_attempts_both_owners_preserving_first_failure() {
    const auto closes = std::make_shared<int>(0);
    auto seed = research_seed(false);
    res::PolicySession session(runtime::make_research(seed->bootstrap),
        std::make_unique<FailingClosePolicy>(closes));
    bool primary = false;
    try { session.close(); }
    catch (const std::runtime_error& error) {
        primary = std::string(error.what()).find("worker cleanup") != std::string::npos;
    }
    bt::require(primary, "the policy-owner failure is the preserved primary");
    bt::require(session.environment().closed() && *closes == 1,
        "the environment owner close is still attempted exactly once");
    session.close();
    bt::require(*closes == 1,
        "a latched close does not retry the failed owner");
    close_environment_wraps_first_failure();
}

// test_research_constructor_failure_releases_worker — PolicySession restores
// named environment ownership when start_policy throws, so the caller still
// closes the worker-owning runtime (async_telemetry makes the worker live at
// construction; the close joins it inside accounting teardown).
[[gnu::noinline]] void research_constructor_failure_releases_worker() {
    auto seed = research_seed(true);
    auto environment = runtime::make_research(seed->bootstrap);
    bool primary = false;
    try {
        res::PolicySession const failed(std::move(environment),
            std::make_unique<ResetFailurePolicy>());
    } catch (const std::runtime_error& error) {
        primary = std::string(error.what()).find("injected constructor reset failure")
            != std::string::npos;
    }
    bt::require(primary, "the constructor failure propagates");
    // NOLINTNEXTLINE(bugprone-use-after-move) asserting restored ownership is the test's purpose
    bt::require(environment != nullptr,
        "caller environment ownership is restored on constructor failure");
    environment->close();
    bt::require(environment->closed(),
        "the restored environment releases its async worker on close");
}

// test_physical_driver_construction_owns_only_needed_worker[native-False] —
// a synchronous session owns no AsyncCloseRunner; async_telemetry constructs
// exactly the one worker it needs. The [python-True] parametrization pins the
// python-backend adapter seam and is a documented drop.
[[gnu::noinline]] void construction_owns_only_needed_worker() {
    runtime::PeriodAccounting sync_only(synthetic_config(), synthetic_state(), 1,
        nullptr, nullptr);
    bt::require(!sync_only.asynchronous(),
        "a synchronous session owns no async worker");
    runtime::AccountingConfig async_config = synthetic_config();
    async_config.async_telemetry = true;
    runtime::PeriodAccounting async_owner(async_config, synthetic_state(), 1,
        nullptr, nullptr);
    bt::require(async_owner.asynchronous(),
        "async telemetry constructs exactly the worker it needs");
}

// test_python_driver_close_releases_worker_once — the driver-close twin on
// the owned runtime: close() latches and a second close is a no-op, so the
// async worker is released exactly once.
[[gnu::noinline]] void driver_close_releases_worker_once() {
    namespace rr = runtime_reference;
    auto docs = rr::resolved_docs(rr::welded_dir());
    rr::set_member(docs.config, "async_telemetry", rr::bio::Value{true});
    auto ride = rr::compose_runtime(docs);
    ride->close();
    bool latched = false;
    try { (void)ride->snapshot(); }
    catch (const std::logic_error& error) {
        latched = std::string(error.what()).find("closed") != std::string::npos;
    }
    bt::require(latched, "the first close latches the runtime");
    ride->close(); // idempotent — the worker was released exactly once
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
        research_close_keeps_active_window_guard_and_discards_without_steps();
        research_close_attempts_both_owners_preserving_first_failure();
        research_constructor_failure_releases_worker();
        construction_owns_only_needed_worker();
        driver_close_releases_worker_once();
    });
}
