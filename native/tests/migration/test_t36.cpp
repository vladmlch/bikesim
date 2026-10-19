// T36 source-derived ownership/pacing/failure regressions. Fatal-engine and
// SIGINT probes run in separate native child processes only when this test is
// explicitly executed later. Nothing was compiled or run during authoring.
#include "support/migration_test.hpp"
#include "diag.hpp"
#include "research/lifecycle.hpp"
#include "research/evidence.hpp"
#include "research/preview_csv.hpp"
#include "platform/process.hpp"
#include "platform/provenance.hpp"
#include "runtime/setup_values.hpp"
#include "io/json.hpp"
#include <csignal>
#include <exception>
#include <ranges>
#include <sstream>

namespace {
using namespace bike::research;
namespace wire = runtime::sample_wire;
struct Calls { int acts = 0, resets = 0; };
class CountingPolicy final : public Policy {
public:
    explicit CountingPolicy(std::shared_ptr<Calls> calls) : calls_(std::move(calls)) {}
    ~CountingPolicy() override;
    CountingPolicy(const CountingPolicy&) = delete;
    CountingPolicy& operator=(const CountingPolicy&) = delete;
    CountingPolicy(CountingPolicy&&) = delete;
    CountingPolicy& operator=(CountingPolicy&&) = delete;
    void reset(std::string_view) override { ++calls_->resets; }
    runtime::RideControl act(const runtime::SensorObservation&, std::optional<double>) override {
        ++calls_->acts; runtime::RideControl control; control.motor_torque_nm = 0.; return control;
    }
private:
    std::shared_ptr<Calls> calls_;
};
CountingPolicy::~CountingPolicy() = default;
class ResetFailurePolicy final : public Policy {
public:
    ResetFailurePolicy() = default;
    ~ResetFailurePolicy() override;
    ResetFailurePolicy(const ResetFailurePolicy&) = delete;
    ResetFailurePolicy& operator=(const ResetFailurePolicy&) = delete;
    ResetFailurePolicy(ResetFailurePolicy&&) = delete;
    ResetFailurePolicy& operator=(ResetFailurePolicy&&) = delete;
    [[noreturn]] void reset(std::string_view) override { throw std::runtime_error("injected initial policy reset failure"); }
    [[noreturn]] runtime::RideControl act(const runtime::SensorObservation&, std::optional<double>) override {
        throw std::runtime_error("act must not run after reset failure");
    }
};
ResetFailurePolicy::~ResetFailurePolicy() = default;
class Publication final : public ResearchPublication {
public:
    Publication() = default;
    ~Publication() override;
    Publication(const Publication&) = delete;
    Publication& operator=(const Publication&) = delete;
    Publication(Publication&&) = delete;
    Publication& operator=(Publication&&) = delete;
    int calls = 0;
    bool fail_first = false, interrupt = false;
    std::size_t episodes = 0;
    std::filesystem::path publish(std::span<const EpisodeReceipt> receipts, std::span<const OperatorEvent> events) override {
        ++calls; episodes = receipts.size();
        bt::require(!receipts.empty() && !events.empty(), "publication receives immutable evidence and event rows");
        if (fail_first && calls == 1) throw std::runtime_error("injected publication failure");
        if (interrupt) (void)std::raise(SIGINT);
        return std::filesystem::temp_directory_path() / "t36-publication-receipt";
    }
};
Publication::~Publication() = default;
template<class F> void runtime_failure(F&& call, std::string_view contains) {
    bool matched = false;
    try { std::forward<F>(call)(); }
    catch (const std::runtime_error& error) { matched = std::string(error.what()).find(contains) != std::string::npos; }
    bt::require(matched, std::string("expected failure: ") + std::string(contains));
}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")

[[gnu::noinline]] std::unique_ptr<PreparedEnvironment> prepared() {
    auto request = std::make_unique<ResearchRequest>(read_research_request(bt::fixture("requests/research_delay.toml")));
    request->capture_mode = runtime::ResearchCaptureMode::full;
    request->experiment.control_period_s = .06; // exceeds the 50 ms pacing debt cap
    request->experiment.duration_s = .12;
    request->experiment.stop_on_model_violation = false;
    request->experiment.maximum_energy_residual_ratio = 1e12; // timing test: numeric outcome tested in T37
    request->setup.model.physics.value().initial_speed_mps = 0.; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
    return std::make_unique<PreparedEnvironment>(prepare_environment(std::move(*request)));
}
void clock_contract() {
    PlaybackClock clock(.001);
    clock.rebase(1., 0);
    const auto capped = clock.target(2., 0);
    bt::require(capped == 50 && clock.debt_s() == .05, "wall debt is capped without shortening an external command");
    bt::require(clock.target(2., 10) == 50, "actual completed physics steps pay pacing debt");
    clock.set_paused(true, 2., 10);
    bt::require(clock.target(20., 10) == 10 && clock.debt_s() == 0., "pause discards debt");
    clock.set_paused(false, 20., 10);
    clock.set_scale(4, 20., 10);
    bt::require(clock.target(21., 10) == 210, "scale rebase preserves dt and caps scaled wall debt");
    clock.rebase(21., 0);
    bt::require(clock.target(21., 0) == 0, "episode reset rebases pacing");
    bt::invalid([&] { clock.set_scale(3, 21., 0); });
    bt::invalid([] { PlaybackClock const invalid(0.); });
}
void boundary_and_retry(const PreparedEnvironment& seed) {
    const auto calls = std::make_shared<Calls>();
    PolicySession session(runtime::make_research(seed.bootstrap), std::make_unique<CountingPolicy>(calls));
    const auto publisher = std::make_shared<Publication>(); publisher->fail_first = true;
    ResearchRun run(session, publisher);
    session.begin_advance();
    for (int i = 0; i < 4; ++i) bt::require(!session.advance_pending(0.), "zero wall budget yields the accepted window");
    bt::require(calls->acts == 1 && session.environment().snapshot().step == 0, "yield never asks policy for another command");
    bt::require(!session.advance_pending(std::nullopt, 1), "target step yields inside a long window");
    session.pause(); session.set_brakes(.2, .3);
    bt::require(!session.paused(), "pause remains queued until the accepted window ends");
    drain_active(session);
    bt::require(calls->acts == 1 && session.paused() && !session.pending(), "stop drains only the existing command");
    bt::require(session.environment().snapshot().step == seed.bootstrap.config.control_steps,
        "pacing debt cap did not truncate the 60 ms control window");
    const auto status = session.environment().status();
    bt::require(wire::string(wire::required(status, "reason")) == "operator_stop", "operator stop terminal reason");
    runtime_failure([&] { (void)run.finish(); }, "publication failure");
    bt::require(run.finalization_attempted() && run.episodes().size() == 1, "failed publication retains the sealed receipt");
    runtime_failure([&] { session.reset("1"); }, "finalized");
    const auto summary = bike::io::canonical_json(run.episodes().front().summary);
    session.close();
    bt::require(run.finish().has_value() && publisher->calls == 2 && publisher->episodes == 1,
        "publication retries immutable receipts after closing the engine");
    (void)run.finish();
    bt::require(publisher->calls == 2 && summary == bike::io::canonical_json(run.episodes().front().summary),
        "repeat finish is idempotent and does not reseal live state");
}
void queued_reset(const PreparedEnvironment& seed) {
    const auto calls = std::make_shared<Calls>();
    PolicySession session(runtime::make_research(seed.bootstrap), std::make_unique<CountingPolicy>(calls));
    const auto generation = session.environment().snapshot().generation;
    session.begin_advance();
    (void)session.advance_pending(std::nullopt, 1);
    session.reset("184467440737095516160001");
    bt::require(calls->resets == 1, "queued reset does not restart policy inside a window");
    while (session.pending()) (void)session.advance_pending();
    bt::require(calls->acts == 1 && calls->resets == 2 && session.environment().snapshot().step == 0 &&
        session.environment().snapshot().generation > generation, "reset occurs after exactly one accepted interval");
    bt::require(session.environment().configuration().seed_decimal == "184467440737095516160001",
        "new sensor entropy is retained without narrowing");
    session.begin_advance(); session.request_stop(); drain_active(session);
    bt::require(calls->acts == 2, "reset does not call act before explicit acceptance");
    session.close();
}
void loader_failure_receipt(const PreparedEnvironment& seed) {
    auto environment = runtime::make_research(seed.bootstrap);
    const auto publisher = std::make_shared<Publication>(); publisher->fail_first = true;
    ResearchRun run(*environment, publisher);
    std::exception_ptr primary;
    try { (void)load_policy("missing_module:factory"); }
    catch (...) { primary = std::current_exception(); environment->fail_policy("injected policy load failure"); }
    bool preserved = false;
    try { (void)run.finish(primary); }
    catch (const std::invalid_argument& error) { preserved = std::string(error.what()).find("missing_module") != std::string::npos; }
    bt::require(preserved && !run.failures().secondary.empty(), "primary policy-load error survives publication failure");
    bt::require(run.episodes().size() == 1 && run.episodes().front().episode.physical.samples.empty() &&
        run.episodes().front().episode.requested.empty() && run.episodes().front().episode.observations.size() == 1,
        "policy loading failure retains zero-step startup evidence");
    environment->close();
    // NOLINTNEXTLINE(bugprone-empty-catch) deliberate: the result is advisory and the failure path is handled separately
    try { (void)run.finish(); } catch (const std::invalid_argument&) {}
    bt::require(publisher->calls == 2, "primary failure still permits an explicit owned-data publication retry");
}
void preview_contract() {
    std::ostringstream stream;
    PhysicalLiveCsv csv(stream, .01, 2);
    csv.write(0., 1, {{"time_s", 0.}, {"label", "a,b"}});
    bt::require(!csv.due(.005, 1), "preview cadence does not emit extra rows");
    csv.write(.01, 1, {{"time_s", .01}, {"label", "quoted\""}, {"later_column", 9.}});
    csv.write(0., 2, {{"time_s", 0.}, {"label", "reset"}});
    csv.write_marker("duration"); csv.close(); csv.close();
    bt::require(stream.str() == "generation,event,time_s,label\r\n1,,0.0,\"a,b\"\r\n1,,0.01,\"quoted\"\"\"\r\n2,reset,0.0,reset\r\n2,duration,,\r\n",
        "preview fixes initial columns, quotes CSV and marks reset/end without retaining history");
    runtime_failure([&] { csv.write_marker("late"); }, "closed");
    bt::invalid([&] { PhysicalLiveCsv const invalid(stream, 0.); });
}
void reset_failure_receipt(const PreparedEnvironment& seed) {
    auto environment = runtime::make_research(seed.bootstrap);
    std::exception_ptr primary;
    try { PolicySession const failed(std::move(environment), std::make_unique<ResetFailurePolicy>()); }
    catch (const std::runtime_error&) { primary = std::current_exception(); }
    // Deliberate use-after-move: asserting restored ownership is the test's purpose.
    // cppcheck-suppress accessMoved
    // NOLINTNEXTLINE(bugprone-use-after-move) asserting the restored ownership is the test's purpose
    bt::require(primary && environment && environment->snapshot().step == 0,
        "constructor reset failure restores named caller environment ownership");
    const auto publisher = std::make_shared<Publication>();
    ResearchRun run(*environment, publisher);
    runtime_failure([&] { (void)run.finish(primary); }, "initial policy reset failure");
    bt::require(publisher->calls == 1 && run.episodes().size() == 1 &&
        run.episodes().front().episode.observations.size() == 1,
        "failed policy reset publishes the original zero-step evidence before rethrowing");
    environment->close();
}
void exit_contract() {
    for (const bool numeric : {false, true})
        for (const bool model : {false, true}) {
            const runtime::WireObject status{{"error", nullptr}, {"numerically_valid", numeric}, {"model_valid", model}};
            bt::require(episode_exit_code(status) == (numeric && model ? 0 : 2), "invalid model and numerics share exit 2");
            auto failed = status; wire::set(failed, "error", "policy failed");
            bt::require(episode_exit_code(failed) == 3, "error exit 3 overrides validity flags");
        }
}
void failure_child() {
    const auto seed = prepared();
    const auto calls = std::make_shared<Calls>();
    PolicySession session(runtime::make_research(seed->bootstrap), std::make_unique<CountingPolicy>(calls));
    session.environment().test_fail_at_step(2);
    session.begin_advance();
    bool failed = false;
    try { (void)session.advance_pending(); } catch (const std::exception&) { failed = true; }
    const auto status = session.environment().status();
    bt::require(failed && calls->acts == 1 && !session.pending(), "physical suffix failure leaves no accepted continuation");
    bt::require(session.environment().snapshot().step == 2 && session.environment().transitions().empty(),
        "failed suffix preserves physical prefix without a synthetic transition");
    bt::require(episode_exit_code(status) == 3 && wire::integer(wire::required(status, "completed_count")) == 0,
        "failed control remains distinct from completed physical steps");
    session.close();
}
void interrupt_child() {
    const auto seed = prepared();
    auto environment = runtime::make_research(seed->bootstrap);
    auto const publication = std::make_shared<Publication>(); publication->interrupt = true;
    ResearchRun run(*environment, publication);
    bool deferred = false;
    try { (void)run.finish(); } catch (const OperatorInterrupted&) { deferred = true; }
    bt::require(deferred && publication->calls == 1 && run.episodes().size() == 1,
        "second SIGINT is honored after publishing the owned episode");
    bt::require(run.finish().has_value() && publication->calls == 1, "deferred interrupt does not duplicate publication");
    environment->close();
}
} // namespace
int main(int argc, char** argv) {
    return bt::run([&] {
        const auto arguments = std::views::counted(argv, argc);
        if (argc == 2 && std::string_view(arguments[1]) == "--failure-child") { failure_child(); return; }
        if (argc == 2 && std::string_view(arguments[1]) == "--interrupt-child") { interrupt_child(); return; }
        bt::require(argc == 1, "unexpected T36 arguments");
        clock_contract(); preview_contract(); exit_contract();
        const auto seed = prepared();
        boundary_and_retry(*seed); queued_reset(*seed); loader_failure_receipt(*seed); reset_failure_receipt(*seed);
        for (const auto child : {"--failure-child", "--interrupt-child"}) {
            const auto result = bike::platform::run({bike::platform::executable_path().string(), child},
                std::filesystem::temp_directory_path(), {.cancellation = {}, .timeout_s = 300., .max_output_bytes = std::size_t{1024U} * std::size_t{1024U}});
            bt::require(result.exit_code == 0, std::string(child) + ": " + result.err + result.out);
        }
    });
}
NATIVE_DIAG_POP
