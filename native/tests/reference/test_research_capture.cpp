// Completion 3.33b. Source: tests/reference/test_research_capture.py.
// Capture-mode parity, sealed-evidence ownership, the verify cursor's
// consume-after-freeze contract and the four final-only lifecycle rows.
//
// Substitution notes:
//  * _noise() monkeypatches the factory's SensorConfig; the port injects the
//    same delivery bias/dropout fields on the shared request recipe, so both
//    envs exercise a nontrivial (identical) delivery path.
//  * env.seal_episode(0, reason='reset') is an adapter pair — id + reason —
//    over the owner seal; the port stamps seal_reason on run_metadata and the
//    summarize() output exactly where ResearchRun::seal writes them.
//  * 'not_finished' is the Python summary's display fallback for a null
//    terminal reason; the native summary keeps the null, and the port pins
//    that representation (replay/session.cpp translates it back).
//  * The qpos.setflags / initial_integration_state.setflags ValueError legs
//    are numpy flag refusals. On the native owner the payload is const
//    (SamplePtr is shared_ptr<const PhysicalSampleData>) and
//    initial_integration_state is a by-value copy — mutation can never reach
//    the live episode, so the contract is type-level and static_assert'd.
//  * env.begin_control raising RuntimeError('sealed') after seal is an
//    adapter-side frozen flag; the native owner deliberately re-serves seals
//    as fresh identical snapshots, and finality is owned by ResearchRun —
//    that leg has no native twin.
//  * contracts.freeze MemoryError injection has no native twin (already
//    documented in the 3.33a capture port): prepare_applied_since is a pure
//    const read. This test's remaining legs pin the consume-after-ack cursor
//    semantics the freeze failure was protecting.
//  * launch.capture.finalize(tmp_path) in the incomplete-window test is the
//    raw publication seam; publication->publish(launch.episodes(),
//    launch.events()) is the identical native owner call.
//  * finish(primary=..., stop=True) returns the path in the Python adapter;
//    the native owner rethrows the primary after publishing, so the port
//    catches the rethrow and reads the idempotent finish() — the published
//    bytes are identical either way.
#include "research_reference_support.hpp"
#include "replay/input.hpp"
#include "runtime/setup_values.hpp"
#include <exception>
#include <type_traits>

namespace {
using namespace research_reference;
namespace sw = runtime::sample_wire;
namespace rec = bike::recording;
namespace io = bike::io;
namespace rd = bike::recording::detail;
namespace setup = runtime::setupvalue;

// tests/reference/test_research_capture.py::
//   test_python_on_off_capture_does_not_change_state_noise_or_results
void test_python_on_off_capture_does_not_change_state_noise_or_results() {
    // The monkeypatched SensorConfig adds delivery bias + dropout on both
    // envs; capture mode must not touch any of it.
    const auto noisy = [](runtime::ResearchCaptureMode mode) {
        auto req = request({.capture = mode});
        req->sensors.delivery.acceleration_bias = {.1, -.2, .3};
        req->sensors.delivery.gyro_bias = .01;
        req->sensors.delivery.dropout_probability = .25;
        return research::make_environment(*req);
    };
    auto off = noisy(runtime::ResearchCaptureMode::off);
    auto full = noisy(runtime::ResearchCaptureMode::full);
    for (const double torque : {0., 2., 1.}) {
        const auto a = step(*off, motor_control(torque));
        const auto b = step(*full, motor_control(torque));
        // asdict(a) == asdict(b): every carried transition field.
        bt::require(a.observation.time_s == b.observation.time_s &&
                    a.observation.source_time_s == b.observation.source_time_s &&
                    a.observation.valid == b.observation.valid &&
                    a.observation.values == b.observation.values &&
                    a.truth.has_value() == b.truth.has_value() &&
                    (!a.truth || wire_equal(runtime::Wire{a.truth->as_wire()},
                                            runtime::Wire{b.truth->as_wire()})) &&
                    a.contact_state == b.contact_state &&
                    a.terminated == b.terminated && a.truncated == b.truncated &&
                    a.numerically_valid == b.numerically_valid &&
                    a.model_valid == b.model_valid && a.reason == b.reason &&
                    a.physics_steps == b.physics_steps && a.demand_nm == b.demand_nm,
                    "capture mode does not change the delivered transition");
        // integration_state(sim) equality — the owned snapshot vector copy.
        bt::require(off->snapshot().integration_state == full->snapshot().integration_state,
                    "capture mode does not change the integration state");
        const auto off_status = off->status();
        const auto full_status = full->status();
        bt::require(sw::integer(sw::required(off_status, "samples_attempted")) ==
                        sw::integer(sw::required(full_status, "samples_attempted")) &&
                    sw::integer(sw::required(off_status, "samples_dropped")) ==
                        sw::integer(sw::required(full_status, "samples_dropped")),
                    "the sensor pipeline counters agree across capture modes");
        bt::require(wire_equal(runtime::Wire{sw::required(off_status, "metrics")},
                               runtime::Wire{sw::required(full_status, "metrics")}),
                    "tracker metrics agree across capture modes");
    }
    bt::require(off->commands_requested().empty() && off->commands_applied().empty(),
                "off capture keeps no command history");
    bt::require(off->observations().empty() && off->transitions().empty(),
                "off capture keeps no observation or transition history");
    bt::require(sw::integer(sw::required(off->status(), "recorded_rows")) == 0,
                "off capture retains no physical samples");
    bt::require(full->commands_requested().size() == 3 && full->transitions().size() == 3,
                "full capture keeps every requested command and transition");
    // The adapter's env.recorder.rows counts finalize-staged rows (always 0
    // pre-seal); the native 'recorded_rows' status field is the retained
    // capture list itself — Python's _capture_samples twin.
    bt::require(full->observations().size() == 4,
                "full capture keeps the startup row plus one observation per window");
    bt::require(sw::integer(sw::required(full->status(), "recorded_rows")) ==
                    full->snapshot().step,
                "the captured prefix covers every committed physical step");
    off->close();
    full->close();
}

// tests/reference/test_research_capture.py::
//   test_sealed_python_source_survives_reset_close_and_forbids_array_mutation
void test_sealed_python_source_survives_reset_close_and_forbids_array_mutation() {
    auto env = environment({.decimation = 3,
                            .capture = runtime::ResearchCaptureMode::full});
    (void)step(*env, motor_control(0.));
    auto evidence = env->seal_episode();
    // env.seal_episode(0, reason='reset'): the adapter stamps the reason onto
    // the sealed metadata; the native owner writes it identically in
    // ResearchRun::seal — stamp it here to model the same record.
    setup::set(evidence.run_metadata, "seal_reason", setup::text("reset"));
    bt::require(!evidence.physical.samples.empty(), "the sealed episode retains the committed prefix");
    const auto held = evidence.physical.samples[0]->qpos;
    // first.qpos.setflags(write=True) -> ValueError: sample payloads are
    // const through SamplePtr — refusal is a type-level contract, no run leg.
    static_assert(std::is_const_v<typename runtime::SamplePtr::element_type>,
                  "sealed sample payloads are immutable");
    // initial_integration_state.setflags(write=True) -> ValueError: the
    // native field is an owned by-value copy; mutating it can never reach the
    // live episode the way the numpy flag refusal protects against.
    evidence.initial_integration_state.front() = 12345.;
    bt::require(evidence.initial_integration_state.front() == 12345. &&
                    env->snapshot().integration_state.front() != 12345.,
                "the sealed startup state is an owned copy isolated from the runtime");
    // env.seal_episode(0, reason='reset') is evidence: the adapter caches the
    // seal object; the native owner re-serves an identical snapshot. Value
    // equality is the port's twin of the Python identity assertion.
    const auto again = env->seal_episode();
    bt::require(again.physical.samples.size() == evidence.physical.samples.size() &&
                    again.physical.samples[0]->qpos == held,
                "a repeated seal returns identical evidence");
    env->reset();
    env->close();
    bt::require(evidence.physical.samples[0]->qpos == held,
                "sealed evidence survives reset and close");
    // tables(batch_rows=3): page the retained physics stream in threes and
    // sum the batches back to the row count.
    const auto rows = static_cast<std::int64_t>(evidence.physical.samples.size());
    std::int64_t paged = 0;
    for (std::int64_t start = 0; start < rows; start += 3)
        paged += static_cast<std::int64_t>(evidence.physical.sample_batch(start, 3).samples.size());
    bt::require(paged == rows, "the paged physics tables cover every retained row");
    auto summary = research::summarize(evidence);
    setup::set(summary, "seal_reason", setup::text("reset"));
    bt::require(std::holds_alternative<std::monostate>(
                    rd::field(rd::field(summary, "outcome"), "reason").data),
                "an unfinished seal records a null terminal reason ('not_finished' in the adapter)");
    bt::require(io::as_string(rd::field(summary, "seal_reason"), "seal_reason") == "reset",
                "the seal reason is stamped on the summary");
}

// tests/reference/test_research_capture.py::
//   test_python_verify_suffix_is_consumed_only_after_freezing
void test_python_verify_suffix_is_consumed_only_after_freezing() {
    auto env = environment({.capture = runtime::ResearchCaptureMode::verify});
    // The freeze-injection leg is covered by the sibling 3.33a capture port:
    // a pure const prepare cannot lose un-acked rows. What remains here is
    // the consumed-cursor contract the freeze failure was protecting.
    // env.applied_since(0) = prepare + freeze + acknowledge in one adapter
    // call; the port performs the same three steps explicitly.
    const auto initial = env->prepare_applied_since(0);
    const auto cursor = sw::integer(sw::required(initial, "next_cursor"));
    bt::require(sw::array(sw::required(initial, "rows")).size() == 1 && cursor == 1,
                "the startup applied row is delivered once");
    env->acknowledge_applied(cursor);
    bt::require(env->commands_applied().empty(),
                "a consumed suffix leaves no un-acked prefix visible");
    const auto empty = env->prepare_applied_since(cursor);
    bt::require(sw::array(sw::required(empty, "rows")).empty() &&
                    sw::integer(sw::required(empty, "next_cursor")) == cursor,
                "the consumed cursor yields the empty suffix");
    invalid_with([&] { (void)env->prepare_applied_since(0); }, "stale");
    // env.applied_since(True): the bool cursor rejection is a Python
    // type-domain check — a bool cannot reach the int64 cursor parameter.
    (void)step(*env, motor_control(2.));
    const auto suffix = env->prepare_applied_since(cursor);
    const auto &tail = sw::array(sw::required(suffix, "rows"));
    const auto end = sw::integer(sw::required(suffix, "next_cursor"));
    bt::require(end == cursor + static_cast<std::int64_t>(tail.size()) && !tail.empty(),
                "the suffix advances the cursor by exactly its row count");
    env->acknowledge_applied(end);
    bt::require(env->commands_requested().empty() && env->observations().empty() &&
                    env->transitions().empty(),
                "verify retention keeps no stream history");
    const auto last = env->last_requested();
    const auto &control = sw::object(sw::required(sw::object(last), "control"));
    bt::require(sw::number(sw::required(control, "motor_torque_nm")) == 2.,
                "the last requested command carries the submitted motor torque");
    env->close();
}

// tests/reference/test_research_capture.py::
//   test_two_episode_launch_writes_only_after_finalization
void test_two_episode_launch_writes_only_after_finalization() {
    Directory directory;
    const auto output = directory.path / "recording";
    auto [env, publication] = recording_env(output);
    research::ResearchRun launch(*env, publication);
    (void)step(*env, motor_control(0.));
    launch.reset();
    bt::require(!std::filesystem::exists(output),
                "a mid-run seal writes nothing before finalization");
    (void)step(*env, motor_control(1.));
    env->stop();
    (void)launch.seal(*text(env->status(), "reason"));
    bt::require(!std::filesystem::exists(output),
                "a requested seal still writes nothing before finalization");
    const auto path = launch.finish();
    env->close();
    bt::require(path.has_value(), "finish publishes the run directory");
    {
        rec::Recording reader(*path);
        bt::require(reader.episode_ids() == std::vector<std::int64_t>({0, 1}),
                    "both sealed episodes publish");
        invalid_with([&] { (void)reader.episode(); }, "select --episode");
        for (const std::int64_t identity : {0, 1})
            bt::require(io::as_int64(rd::field(reader.episode(identity), "complete_control_steps"),
                                     "complete_control_steps") == 1,
                        "each episode retains one completed control step");
        bt::require(row_count(reader, "SELECT count(*) AS n FROM observations") == 4,
                    "two episodes publish startup + one delivered observation each");
    }
    bt::require(launch.finish() == path, "repeat finish is idempotent");
}

// tests/reference/test_research_capture.py::
//   test_incomplete_window_and_failed_policy_remain_analyzable
void test_incomplete_window_and_failed_policy_remain_analyzable() {
    Directory directory;
    auto [env, publication] = recording_env(directory.path);
    env->begin_control(motor_control(0.), 0., 0.);
    (void)env->advance_control(std::nullopt, 1);
    research::ResearchRun launch(*env, publication);
    // A failure seal owns the actual committed prefix, not a fabricated
    // completed step. launch.capture.finalize(tmp_path) is the raw
    // publication seam — no drain, no finish bookkeeping.
    (void)launch.seal("simulation_error");
    const auto path = publication->publish(launch.episodes(), launch.events());
    env->close(); // discard_pending — the native close has no pending guard
    {
        rec::Recording reader(path);
        const auto episode = reader.episode();
        bt::require(io::as_int64(rd::field(episode, "complete_control_steps"),
                                 "complete_control_steps") == 0,
                    "a mid-window seal records no completed control step");
        bt::require(io::as_int64(rd::field(episode, "physical_rows"), "physical_rows") == 1,
                    "the seal retains exactly the committed physical prefix");
        bt::require(!io::as_bool(rd::field(episode, "replay_eligible"), "replay_eligible"),
                    "a failure-sealed prefix is not replay eligible");
        bt::require(row_count(reader, "SELECT count(*) AS n FROM commands_requested") == 1,
                    "the requested command of the incomplete window is retained");
        bt::require(row_count(reader, "SELECT count(*) AS n FROM observations") == 1,
                    "only the startup observation is retained");
    }
    invalid_with([&] { (void)bike::replay::load_replay_input(path); }, "incomplete-prefix");
}

// tests/reference/test_research_capture.py::
//   test_policy_failure_before_first_request_seals_zero_step_evidence
void test_policy_failure_before_first_request_seals_zero_step_evidence() {
    Directory directory;
    auto [env, publication] = recording_env(directory.path);
    research::ResearchRun launch(*env, publication);
    env->fail_policy("policy setup injection");
    // finish(primary=error, stop=True): the native owner rethrows the primary
    // after publishing; the adapter returned the path. Catch, then read the
    // idempotent second call.
    throws_runtime([&] {
        (void)launch.finish(std::make_exception_ptr(std::runtime_error("policy setup injection")), true); },
        "policy setup injection");
    const auto path = launch.finish();
    env->close();
    bt::require(path.has_value(), "the failure seal still publishes");
    rec::Recording reader(*path);
    const auto episode = reader.episode();
    bt::require(io::as_int64(rd::field(episode, "physical_rows"), "physical_rows") == 0,
                "a policy failure before the first request retains no physical rows");
    bt::require(!io::as_bool(rd::field(episode, "replay_eligible"), "replay_eligible"),
                "zero-step policy failure evidence is not replay eligible");
    bt::require(row_count(reader, "SELECT count(*) AS n FROM observations") == 1,
                "only the startup observation is retained");
}

// tests/reference/test_research_capture.py::
//   test_interruption_drains_only_the_already_accepted_control
void test_interruption_drains_only_the_already_accepted_control() {
    Directory directory;
    auto [env, publication] = recording_env(directory.path);
    auto counting = std::make_unique<CountingPolicy>();
    auto *const policy = counting.get();
    // ResearchRun(env, ...) + launch.attach(session) — the session-form
    // constructor is the same ownership pair in one step.
    research::PolicySession session(std::move(env), std::move(counting));
    auto *const environment_ptr = &session.environment();
    research::ResearchRun launch(session, publication);
    session.begin_advance();
    (void)session.advance_pending(std::nullopt, 1);
    throws_runtime([&] {
        (void)launch.finish(std::make_exception_ptr(research::OperatorInterrupted{}), true); },
        "operator interrupt");
    const auto path = launch.finish();
    bt::require(policy->calls.size() == 1 &&
                    environment_ptr->snapshot().step ==
                        environment_ptr->configuration().control_steps,
                "the interrupt drains only the already accepted window");
    bt::require(!session.pending(), "no pending window remains after the drain");
    environment_ptr->close();
    bt::require(path.has_value(), "the interrupt seal still publishes");
    rec::Recording reader(*path);
    bt::require(io::as_int64(rd::field(reader.episode(), "complete_control_steps"),
                             "complete_control_steps") == 1,
                "the drained window records one completed control step");
    bool interrupted = false;
    for (const auto &row : reader.rows("SELECT kind FROM events"))
        if (io::as_string(rd::field(row, "kind"), "kind") == "launch_interrupted")
            interrupted = true;
    bt::require(interrupted, "the launch_interrupted event is journaled");
}

} // namespace

int main() {
    return bt::run([] {
        test_python_on_off_capture_does_not_change_state_noise_or_results();
        test_sealed_python_source_survives_reset_close_and_forbids_array_mutation();
        test_python_verify_suffix_is_consumed_only_after_freezing();
        test_two_episode_launch_writes_only_after_finalization();
        test_incomplete_window_and_failed_policy_remain_analyzable();
        test_policy_failure_before_first_request_seals_zero_step_evidence();
        test_interruption_drains_only_the_already_accepted_control();
    });
}
