// Completion 3.33a. Source: tests/reference/test_native_research_environment.py.
// The recipe-agnostic environment contracts; the oracle-parity legs
// (assert_tree step/transition/recorder comparisons, first_failure pin) stay
// planned — see task-C333a-report.md for the red-node inventory and reasons.
//
// Substitutions:
//  * environment_factory's welded physics → the established flat-track
//    crank_effort recipe (research_reference_support.hpp); every assertion
//    below names steps/reasons/state, never physics trajectories.
//  * `front_brake_demand=True` and `wall_budget_s=True` are nanobind type
//    gates with no C++ spelling; the expressible legs (out-of-domain brake,
//    negative and NaN budgets) hit the same validators.
//  * `env.close()` while pending is the Python adapter's own guard
//    (NativeResearch.close raises before the native call); the native
//    runtime deliberately permits deferred close — documented, not ported.
//  * `test_native_loop_never_calls_python_interval_or_rng` exists to prove
//    no Python interval/RNG code runs inside a native window — vacuous in a
//    pure C++ test; the retained half is the committed physics_steps count.
//  * BoundaryRider.act() returns RiderPosture() on the welded oracle
//    recipe; the lumped recipe's begin_control rejects posture commands
//    (rider commands require articulated_planar), so the ported callback
//    returns std::nullopt. The asserted contract — act runs once per
//    staged boundary, never on retry/poll, and reset reseeds — is
//    unchanged.
#include "ft2_reference_support.hpp"
#include "research_reference_support.hpp"
#include "recording/analysis_detail.hpp"

#include <array>
#include <cstdio>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>

namespace {
using namespace research_reference;
namespace rd = bike::recording::detail;
namespace sw = runtime::sample_wire;

[[nodiscard]] std::string read_bytes(const std::filesystem::path &path) {
    std::ifstream stream(path, std::ios::binary);
    bt::require(stream.good(), "golden file is readable: " + path.string());
    std::ostringstream contents;
    contents << stream.rdbuf();
    return contents.str();
}

// tests/reference/test_native_research_environment.py::
//   test_native_loop_never_calls_python_interval_or_rng
void test_native_loop_never_calls_python_interval_or_rng() {
    auto env = environment();
    const auto transition = step(*env, motor_control(0.));
    bt::require(transition.physics_steps == env->configuration().control_steps,
                "one external step commits exactly control_steps physics steps");
}

// tests/reference/test_native_research_environment.py::
//   test_invalid_boundary_calls_do_not_change_state
void test_invalid_boundary_calls_do_not_change_state() {
    auto env = environment();
    const auto before = env->snapshot();
    // begin_control(RideControl(), front_brake_demand=True) — the bool is a
    // binding gate; the expressible leg is the same validator's domain check.
    bt::invalid([&] { env->begin_control(runtime::RideControl{}, 1.5, 0.); });
    bt::require(!flag(env->status(), "active"),
                "a rejected boundary leaves no pending control");
    env->begin_control(motor_control(0.), 0., 0.);
    throws_runtime([&] { env->begin_control(motor_control(0.), 0., 0.); },
                   "an external research window is already active");
    for (const double budget : {-.1, std::numeric_limits<double>::quiet_NaN()}) {
        bt::invalid([&] { (void)env->advance_control(budget, std::nullopt); });
        bt::require(flag(env->status(), "active"),
                    "a rejected budget keeps the pending control");
    }
    throws_runtime([&] { env->reset(); },
                   "finish the active external interval before reset");
    bt::require(env->snapshot().integration_state == before.integration_state,
                "rejected boundary calls never touch integration state");
    env->stop();
    const auto status = env->status();
    bt::require(!flag(status, "terminated") && !flag(status, "truncated"),
                "a queued stop waits for the external boundary");
    (void)env->advance_control(std::nullopt, std::nullopt);
    env->acknowledge_control();
    const auto done = env->status();
    bt::require(flag(done, "truncated") && text(done, "reason") == "operator_stop",
                "the queued stop resolves to operator_stop at the boundary");
}

// tests/reference/test_native_research_environment.py::
//   test_zero_budget_retains_single_window_and_no_tail_flush
void test_zero_budget_retains_single_window_and_no_tail_flush() {
    auto env = environment();
    const auto before = env->snapshot();
    env->begin_control(motor_control(0.), 0., 0.);
    for (int poll = 0; poll < 5; ++poll) {
        bt::require(!env->advance_control(0., std::nullopt).has_value(),
                    "a zero wall budget is a pure poll: no physics progress");
        const auto status = env->status();
        bt::require(flag(status, "active") &&
                        sw::integer(sw::required(status, "step")) == 0 &&
                        sw::integer(sw::required(status, "recorded_rows")) == 0,
                    "the retained single window never touches steps or rows");
        bt::require(env->commands_requested().size() == 1 &&
                        env->observations().size() == 1 &&
                        env->transitions().empty(),
                    "the held window publishes its startup rows and no tail flush");
    }
    bt::require(env->snapshot().integration_state == before.integration_state,
                "zero-budget polling leaves the integration state untouched");
    // assert_tree(native.advance_control(), reference.step(command)) — the
    // transition-equality leg is welded-oracle physics; the port substitutes
    // the recipe-agnostic completion bookkeeping: the window commits,
    // pending clears, and exactly one transition lands in the trace.
    const auto completed = env->advance_control(std::nullopt, std::nullopt);
    bt::require(completed.has_value(), "the unbounded advance completes the window");
    env->acknowledge_control();
    bt::require(!flag(env->status(), "active") && env->transitions().size() == 1,
                "the completed window clears pending and publishes one transition");
    throws_runtime([&] { (void)env->advance_control(std::nullopt, std::nullopt); },
                   "begin_control must precede advance_control");
}

// tests/reference/test_native_research_environment.py::
//   test_stop_on_ended_episode_preserves_original_reason
void test_stop_on_ended_episode_preserves_original_reason() {
    auto env = environment({.duration_s = .02});
    for (;;) {
        const auto status = env->status();
        if (flag(status, "terminated") || flag(status, "truncated")) break;
        (void)step(*env, motor_control(0.));
    }
    const auto done = env->status();
    bt::require(text(done, "reason") == "duration",
                "the flat run ends by duration");
    env->stop();
    bt::require(text(env->status(), "reason") == "duration",
                "an operator stop after the episode keeps the real outcome");
}

// tests/reference/test_native_research_environment.py::BoundaryRider —
// reset() records the seed and clears the call log; act() appends the
// staged observation time. The oracle twin returns RiderPosture() for the
// welded plant; the lumped recipe cannot carry posture commands, so the
// port returns std::nullopt (see the header substitution note).
class BoundaryRider : public research::RiderBehavior {
public:
    BoundaryRider() = default;
    ~BoundaryRider() override = default;
    BoundaryRider(const BoundaryRider &) = delete;
    BoundaryRider &operator=(const BoundaryRider &) = delete;
    BoundaryRider(BoundaryRider &&) = delete;
    BoundaryRider &operator=(BoundaryRider &&) = delete;
    void reset(std::string_view seed_decimal) override {
        seeds.emplace_back(seed_decimal);
        times.clear();
    }
    [[nodiscard]] std::optional<::rider::RiderPosture>
    act(double time_s, const research::RiderSignals &) override {
        times.push_back(time_s);
        return std::nullopt;
    }
    std::vector<std::string> seeds;
    std::vector<double> times;
};

// tests/reference/test_native_research_environment.py::
//   test_custom_rider_callback_runs_once_per_external_boundary
// The staged-command contract of PolicySession::begin_advance
// (policy_session.cpp): the rider behavior's act() fires exactly once while
// the command is staged; a zero-budget advance_pending is a pure poll that
// never re-fires it, and completing the window adds no call. reset(seed=59)
// reseeds policy and behavior through start_policy — the seeds keep every
// reset and the recorded call times clear.
[[gnu::noinline]] void
test_custom_rider_callback_runs_once_per_external_boundary() {
    auto policy = std::make_unique<CountingPolicy>();
    auto *counted = policy.get();
    auto behavior = std::make_unique<BoundaryRider>();
    auto *rider = behavior.get();
    research::PolicySession session(environment(), std::move(policy),
                                    std::move(behavior));
    bt::require(rider->seeds == std::vector<std::string>{"19"} &&
                    counted->seeds == std::vector<std::string>{"19"},
                "session construction resets policy and behavior once");
    session.begin_advance();
    bt::require(rider->times == std::vector<double>{0.},
                "staging a boundary fires the rider act exactly once");
    for (int poll = 0; poll < 3; ++poll)
        bt::require(!session.advance_pending(0.).has_value(),
                    "a zero-budget poll leaves the window pending");
    bt::require(rider->times.size() == 1 && counted->calls.size() == 1,
                "polling never re-fires policy or rider act");
    const auto completed = session.advance_pending();
    bt::require(completed.has_value(),
                "the unbounded poll completes the staged window");
    bt::require(rider->times.size() == 1 && counted->calls.size() == 1,
                "the completed boundary still cost exactly one act");
    session.reset("59");
    bt::require(rider->seeds == std::vector<std::string>({"19", "59"}) &&
                    counted->seeds == std::vector<std::string>({"19", "59"}),
                "reset reseeds policy and behavior to the chosen seed");
    bt::require(rider->times.empty(),
                "the behavior call log clears on reset");
    // The source ends here; the port keeps the once-per-boundary pin live
    // across the reseed — the first staged window of the new episode fires
    // act exactly once at t=0 and no poll repeats it.
    session.begin_advance();
    bt::require(rider->times == std::vector<double>{0.},
                "the reseeded boundary stages a fresh act at t=0");
    for (int poll = 0; poll < 3; ++poll)
        bt::require(!session.advance_pending(0.).has_value(),
                    "a reseeded window polls without a repeat act");
    const auto reseeded = session.advance_pending();
    bt::require(reseeded.has_value() && rider->times.size() == 1,
                "the reseeded window completes with exactly one act");
}

// ===========================================================================
// Golden-qualified legs (FT-2): every record below replays the frozen
// research_flat/research_savage bootstraps against the committed platform
// trees captured under the pinned oracle kernels.
// ===========================================================================
constexpr std::string_view kNode =
    "tests/reference/test_native_research_environment.py::";

[[nodiscard]] bool done(const runtime::NativeResearchRuntime &env) {
    const auto status = env.status();
    return flag(status, "terminated") || flag(status, "truncated");
}
[[nodiscard]] std::int64_t sim_steps(
    const runtime::NativeResearchRuntime &env) {
    return sw::integer(sw::required(env.status(), "step"));
}
[[nodiscard]] std::optional<std::string> reason(
    const runtime::NativeResearchRuntime &env) {
    return text(env.status(), "reason");
}

// bike_sim.sim.research.metrics.episode_metrics — the compact outcome record
// re-assembled from the status/snapshot wires the native runtime publishes
// (status.metrics is the wheelie tracker doc; model_status rides the
// snapshot). run_metadata is empty in every record below, so
// operator_intervention is the oracle's .get() default.
[[nodiscard]] runtime::WireObject episode_metrics(
    const runtime::NativeResearchRuntime &env) {
    const auto status = env.status();
    const auto outcome = text(status, "reason");
    const double duration = sw::number(sw::required(status, "time_s"));
    const double progress = sw::number(sw::required(status, "position_m")) -
                            env.configuration().start_position_m;
    const double requested =
        sw::number(sw::required(status, "torque_requested_nms"));
    const double delivered =
        sw::number(sw::required(status, "torque_delivered_nms"));
    const auto &balance = sw::required(status, "balance_event");
    const bool lost = !sw::is_none(balance);
    runtime::WireObject out;
    sw::set(out, "outcome", sw::required(status, "reason"));
    sw::set(out, "duration_s", duration);
    sw::set(out, "progress_m", progress);
    sw::set(out, "mean_speed_mps", duration > 0. ? progress / duration : 0.);
    sw::set(out, "finish_time_s",
            outcome && *outcome == "finish" ? runtime::Wire(duration)
                                            : runtime::Wire(nullptr));
    sw::set(out, "torque_delivered_nms", delivered);
    sw::set(out, "torque_requested_nms", requested);
    sw::set(out, "demand_integral_nms",
            sw::required(status, "demand_integral_nms"));
    sw::set(out, "operator_intervention", false);
    sw::set(out, "motor_pass_fraction",
            requested > 0. ? runtime::Wire(delivered / requested)
                           : runtime::Wire(nullptr));
    sw::set(out, "balance_lost", lost);
    sw::set(out, "balance_lost_at_m",
            lost ? sw::required(sw::object(balance), "position_m")
                 : runtime::Wire(nullptr));
    sw::set(out, "loop_out", outcome && *outcome == "crash:loop_out");
    sw::set(out, "endo", outcome && *outcome == "crash:endo");
    sw::set(out, "max_shock_stroke_m",
            sw::required(status, "max_shock_stroke_m"));
    sw::set(out, "max_fork_travel_m",
            sw::required(status, "max_fork_travel_m"));
    sw::set(out, "numerically_valid",
            sw::required(status, "numerically_valid"));
    sw::set(out, "max_energy_residual_ratio",
            sw::required(status, "max_energy_residual_ratio"));
    sw::set(out, "model_status", env.snapshot().model_status);
    sw::set(out, "wheelie", sw::required(status, "metrics"));
    return out;
}

// recorder.columns() — the channel table keyed by column name.
[[nodiscard]] runtime::WireObject columns_wire(
    const runtime::NativeResearchRuntime &env) {
    runtime::WireObject out;
    for (const auto &[name, column] : env.recorded_columns())
        sw::set(out, name, runtime::wire_array(column));
    return out;
}

// tests/reference/test_native_research_environment.py::
//   test_native_research_matches_every_external_transition
// The paired gate on the rough-track assist fixture: every external
// transition consumes its recorded tree boundary-for-boundary.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_native_research_matches_every_external_transition() {
    bt::oracle::Node node(
        std::string(kNode) +
        "test_native_research_matches_every_external_transition");
    const auto env = ft2_reference::research("research_savage");
    ft2_reference::consume_tree(
        node, sw::required(env->status(), "observation"),
        "startup.observation");
    std::int64_t boundary = 0;
    char label[40];
    while (!done(*env)) {
        std::snprintf(label, sizeof(label), "transition.%08lld",
                      static_cast<long long>(boundary));
        ft2_reference::consume_tree(
            node, step(*env, runtime::RideControl{}).as_wire(), label);
        ++boundary;
    }
    bt::require(sim_steps(*env) == env->configuration().max_steps,
                "the savage episode completes every physics step");
    const auto outcome = reason(*env);
    bt::require(outcome && *outcome == "duration",
                "the savage run ends by duration");
    ft2_reference::consume_tree(node, env->commands_applied(),
                                "commands_applied");
    ft2_reference::consume_tree(node, env->commands_requested(),
                                "commands_requested");
    node.done();
}
NATIVE_DIAG_POP

// tests/reference/test_native_research_environment.py::
//   test_one_second_pair_with_one_e_minus_nine_gate
// The release gate: 800 physics steps, transition + integration-state rows
// at every boundary, then commands/metrics/columns trailers.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_one_second_pair_with_one_e_minus_nine_gate() {
    bt::oracle::Node node(
        std::string(kNode) +
        "test_one_second_pair_with_one_e_minus_nine_gate");
    const auto env = ft2_reference::research_variant(
        "research_flat", std::nullopt, 20, 800);
    ft2_reference::consume_tree(
        node, sw::required(env->status(), "observation"),
        "startup.observation");
    std::int64_t boundary = 0;
    char label[48];
    while (!done(*env)) {
        std::snprintf(label, sizeof(label), "transition.%08lld",
                      static_cast<long long>(boundary));
        ft2_reference::consume_tree(
            node, step(*env, motor_control(0.)).as_wire(), label);
        const std::vector<double> state =
            env->snapshot().integration_state;
        node.f64(state, "integration state at step " +
                            std::to_string(env->snapshot().step));
        ++boundary;
    }
    bt::require(sim_steps(*env) == env->configuration().max_steps,
                "the one-second gate completes every physics step");
    ft2_reference::consume_tree(node, env->commands_requested(),
                                "commands_requested");
    ft2_reference::consume_tree(node, env->commands_applied(),
                                "commands_applied");
    ft2_reference::consume_tree(node, episode_metrics(*env),
                                "episode_metrics");
    ft2_reference::consume_tree(node, columns_wire(*env),
                                "recorder.columns");
    node.done();
}
NATIVE_DIAG_POP

// tests/reference/test_native_research_environment.py::
//   test_one_second_pair_complete_episode_gate
// The same episode under the raised 1e9 residual tolerance — the measured
// residual still lands under the default .05, so the relaxed gate ran the
// identical physics.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_one_second_pair_complete_episode_gate() {
    bt::oracle::Node node(
        std::string(kNode) +
        "test_one_second_pair_complete_episode_gate");
    const auto env = ft2_reference::research_variant(
        "research_flat", std::nullopt, 20, 800, std::nullopt,
        std::nullopt, std::nullopt, std::nullopt, 1e9);
    ft2_reference::consume_tree(
        node, sw::required(env->status(), "observation"),
        "startup.observation");
    std::int64_t boundary = 0;
    char label[48];
    while (!done(*env)) {
        std::snprintf(label, sizeof(label), "transition.%08lld",
                      static_cast<long long>(boundary));
        ft2_reference::consume_tree(
            node, step(*env, motor_control(0.)).as_wire(), label);
        const std::vector<double> state =
            env->snapshot().integration_state;
        node.f64(state, "integration state at step " +
                            std::to_string(env->snapshot().step));
        ++boundary;
    }
    bt::require(sim_steps(*env) == 800 &&
                    sim_steps(*env) == env->configuration().max_steps,
                "the complete gate runs all 800 steps");
    const auto outcome = reason(*env);
    bt::require(outcome && *outcome == "duration",
                "the complete gate ends by duration");
    const double residual = sw::number(sw::required(
        env->status(), "max_energy_residual_ratio"));
    const std::vector<double> residual_row{residual};
    node.f64_close(residual_row, "max_energy_residual_ratio");
    bt::require(residual <= .05,
                "the raised gate ran the default-gate physics");
    ft2_reference::consume_tree(node, env->commands_requested(),
                                "commands_requested");
    ft2_reference::consume_tree(node, env->commands_applied(),
                                "commands_applied");
    ft2_reference::consume_tree(node, episode_metrics(*env),
                                "episode_metrics");
    ft2_reference::consume_tree(node, columns_wire(*env),
                                "recorder.columns");
    node.done();
}
NATIVE_DIAG_POP

// tests/reference/test_native_research_environment.py::
//   test_delay_full_commands_immediate_brakes_and_recording[decimation]
// The four-command delay/delayed-motor leg under each recorded decimation.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_delay_full_commands_immediate_brakes_and_recording(
    int decimation) {
    bt::oracle::Node node(
        std::string(kNode) +
        "test_delay_full_commands_immediate_brakes_and_recording[" +
        std::to_string(decimation) + "]");
    const auto env = ft2_reference::research_variant(
        "research_flat", std::nullopt, decimation);
    // RideControl(motor_torque_nm=None, motor_limit_nm=0., human_torque_nm=0.,
    //             crank_target_rate_rad_s=0., posture=RiderPosture())
    runtime::RideControl explicit_zero;
    explicit_zero.motor_limit_nm = 0.;
    explicit_zero.human_torque_nm = 0.;
    explicit_zero.crank_target_rate_rad_s = 0.;
    explicit_zero.posture = rider::RiderPosture{};
    // RideControl(0., None, 1., None, RiderPosture(.01), True)
    runtime::RideControl immediate;
    immediate.motor_torque_nm = 0.;
    immediate.human_torque_nm = 1.;
    rider::RiderPosture leaned{};
    leaned.torso_lean_rad = .01;
    immediate.posture = leaned;
    // RideControl(2., .5, 0., 0., None, False)
    runtime::RideControl bounded;
    bounded.motor_torque_nm = 2.;
    bounded.motor_limit_nm = .5;
    bounded.human_torque_nm = 0.;
    bounded.crank_target_rate_rad_s = 0.;
    bounded.rider_enabled = false;
    const std::array<runtime::RideControl, 4> commands{
        explicit_zero, immediate, bounded, runtime::RideControl{}};
    char label[40];
    for (std::size_t index = 0; index < commands.size(); ++index) {
        const double front = index == 1 ? .2 : 0.;
        const double rear = index == 2 ? .1 : 0.;
        std::snprintf(label, sizeof(label), "transition.%08zu", index);
        ft2_reference::consume_tree(
            node, step(*env, commands.at(index), front, rear).as_wire(),
            label);
        std::snprintf(label, sizeof(label), "commands_applied.%zu", index);
        ft2_reference::consume_tree(node, env->commands_applied(), label);
        std::snprintf(label, sizeof(label), "commands_requested.%zu",
                      index);
        ft2_reference::consume_tree(node, env->commands_requested(), label);
        if (done(*env)) break;
    }
    ft2_reference::consume_tree(node, columns_wire(*env),
                                "recorder.columns");
    ft2_reference::consume_tree(node, episode_metrics(*env),
                                "episode_metrics");
    const auto intervals = env->recorded_intervals();
    const auto requested = env->commands_requested();
    bt::require(!intervals.empty() && !requested.empty(),
                "the delay leg publishes recorded and requested rows");
    const auto &first_control =
        sw::object(sw::required(sw::object(intervals.front()), "control"));
    bt::require(sw::is_none(sw::required(
                    sw::object(sw::required(
                        sw::object(requested.front()), "control")),
                    "motor_torque_nm")),
                "the startup requested row keeps the None motor field");
    const double first_sample[2] = {
        sw::number(sw::required(first_control, "motor_torque_nm")),
        sw::number(sw::required(first_control, "human_torque_nm"))};
    node.f64(first_sample, "first_sample.control");
    bt::require(first_sample[0] == 0. && first_sample[1] == 0.,
                "the first recorded interval carries the safe bootstrap "
                "command");
    node.done();
}
NATIVE_DIAG_POP

// tests/reference/test_native_research_environment.py::
//   test_scripted_rider_and_demand_are_sampled_at_physics_rate
// RiderProgram/DemandProgram keyframes replay through the physics-rate
// sampler; demand_nm is the program's value at the current sim time.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_scripted_rider_and_demand_are_sampled_at_physics_rate() {
    bt::oracle::Node node(
        std::string(kNode) +
        "test_scripted_rider_and_demand_are_sampled_at_physics_rate");
    runtime::ResearchPrograms programs;
    programs.reaction_delay_s = .0025;
    rider::RiderPosture leaned{};
    leaned.torso_lean_rad = .01;
    programs.rider_frames = {
        {.time_s = 0., .posture = rider::RiderPosture{},
         .human_torque_nm = 0.},
        {.time_s = .021,
         .posture = leaned,
         .human_torque_nm = 1.},
        {.time_s = .04, .posture = rider::RiderPosture{},
         .human_torque_nm = 0.}};
    programs.demand_frames = {{0., 0.}, {.013, 5.}, {.04, 1.}};
    const auto env = ft2_reference::research_variant(
        "research_flat", std::nullopt, 3, std::nullopt, std::nullopt,
        std::nullopt, std::nullopt, std::nullopt, std::nullopt,
        std::nullopt, std::nullopt, std::nullopt, std::nullopt,
        std::move(programs));
    char label[40];
    for (std::size_t index = 0; index < 4; ++index) {
        const auto status = env->status();
        const auto demand = env->configuration().programs.demand_at(
            sw::number(sw::required(status, "time_s")));
        runtime::RideControl control;
        control.motor_torque_nm = demand;
        std::snprintf(label, sizeof(label), "transition.%08zu", index);
        ft2_reference::consume_tree(
            node, step(*env, control).as_wire(), label);
        if (done(*env)) break;
    }
    ft2_reference::consume_tree(node, env->commands_applied(),
                                "commands_applied");
    bt::require(env->commands_applied().size() >
                    env->commands_requested().size(),
                "program-applied rows outnumber the requested commands");
    ft2_reference::consume_tree(node, episode_metrics(*env),
                                "episode_metrics");
    node.done();
}
NATIVE_DIAG_POP

// tests/reference/test_native_research_environment.py::
//   test_reset_seed_generation_snapshot_and_facade_ownership
// reset(seed=73) reseats tape/startup/metadata through the reseed seam;
// the facade legs (immutable snapshot view, read-only sim/terminated
// attributes) are binding-gated — the port keeps the value-semantics half:
// a snapshot copy the caller mutates can never reach the runtime.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_reset_seed_generation_snapshot_and_facade_ownership() {
    bt::oracle::Node node(
        std::string(kNode) +
        "test_reset_seed_generation_snapshot_and_facade_ownership");
    const auto env = ft2_reference::research("research_flat");
    const auto before = env->snapshot();
    auto caller_copy = before;
    caller_copy.integration_state[0] = 1.;
    bt::require(env->snapshot().integration_state ==
                    before.integration_state,
                "a mutated snapshot copy never touches the runtime");
    (void)step(*env, motor_control(0.));
    reseed(*env, "73");
    const auto delivered = env->observations();
    bt::require(!delivered.empty(),
                "reset re-seats the delivered startup observation");
    ft2_reference::consume_tree(node, delivered.back(),
                                "reset.observation");
    const auto status = env->status();
    bt::require(env->snapshot().generation > before.generation,
                "reset bumps the snapshot generation");
    bt::require(sim_steps(*env) == 0 &&
                    sw::integer(sw::required(status, "recorded_rows")) ==
                        0 &&
                    env->commands_requested().empty() &&
                    env->transitions().empty(),
                "reset clears steps, rows and queues");
    bt::require(sw::integer(sw::required(status, "noise_cursor")) == 1,
                "the reseated tape starts at cursor one");
    bt::require(
        sw::number(sw::required(status, "torque_delivered_nms")) == 0. &&
            sw::number(sw::required(status, "torque_requested_nms")) ==
                0. &&
            sw::number(sw::required(status,
                                    "max_energy_residual_ratio")) == 0.,
        "reset clears the accumulated metrics");
    bt::require(sw::is_none(sw::required(status, "demand_integral_nms")),
                "the flat recipe carries no demand integral after reset");
    bt::require(env->commands_applied().size() == 1,
                "the startup applied row is re-seated");
    // env.metadata on the oracle is the configuration_metadata doc minus
    // the native-only members the binding never projects: the execution
    // block wholesale (native binary provenance), the native-only
    // experiment/sensor/setup additions, and the capture-time source
    // identity (re-seated from the committed golden like the recording
    // port's model_commit/model_source_sha256 substitution).
    auto metadata = env->configuration().metadata;
    {
        const auto base = bt::oracle::root() /
                          std::string(bt::oracle::platform()) /
                          bt::oracle::node_path(
                              std::string(kNode) +
                              "test_reset_seed_generation_snapshot_and_"
                              "facade_ownership");
        const auto index = bike::io::parse_json(
            read_bytes(base.string() + ".json"));
        const auto payload = read_bytes(base.string() + ".bin");
        // Capture-time provenance is string-or-null across platforms: a
        // checkout without .git (the Linux container's /src copy) records
        // null model_commit/model_source_dirty. The recorded Value is
        // re-seated verbatim so the port stays capture-environment-agnostic.
        bike::io::Value oracle_commit, oracle_source_sha;
        bool provenance_found = false;
        for (const auto &entry : std::get<bike::io::Value::Array>(index.data))
            if (rd::string(rd::field(entry, "label")) ==
                "metadata.other") {
                const auto offset = static_cast<std::size_t>(std::stoull(
                    std::get<bike::io::Number>(
                        rd::field(entry, "offset").data)
                        .lexeme));
                const auto nbytes = static_cast<std::size_t>(std::stoull(
                    std::get<bike::io::Number>(
                        rd::field(entry, "nbytes").data)
                        .lexeme));
                const auto recorded = bike::io::parse_json(
                    payload.substr(offset, nbytes));
                oracle_commit = rd::field(recorded, ".model_commit");
                oracle_source_sha =
                    rd::field(recorded, ".model_source_sha256");
                provenance_found = true;
            }
        bt::require(provenance_found,
                    "the golden entry carries the oracle provenance");
        const auto string_or_null = [](const bike::io::Value &value) {
            return bike::io::is_null(value) ||
                   std::holds_alternative<std::string>(value.data);
        };
        bt::require(
            string_or_null(oracle_commit) && string_or_null(oracle_source_sha),
            "oracle provenance is string-or-null");
        auto &object = std::get<bike::io::Value::Object>(metadata.data);
        for (auto &[name, member] : object) {
            if (name == "model_commit")
                member = oracle_commit;
            else if (name == "model_source_sha256")
                member = oracle_source_sha;
        }
        for (const std::string_view section :
             {"execution", "experiment", "sensor_config", "setup"})
            for (auto it = object.begin(); it != object.end(); ++it)
                if (it->first == section) {
                    object.erase(it);
                    break;
                }
    }
    ft2_reference::consume_tree(
        node,
        vread::wire_from_value(metadata, "metadata"),
        "metadata");
    ft2_reference::consume_tree(
        node, step(*env, motor_control(0.)).as_wire(),
        "post_reset.transition");
    node.done();
}
NATIVE_DIAG_POP

// tests/reference/test_native_research_environment.py::
//   test_strict_and_diagnostic_outcomes_remain_visible[strict]
// strict=False replays four diagnostic transitions; strict=True pins the
// weld-floor violation (weld_foot_min_normal_n=100) so the first window
// raises InvalidReferenceRun with the recorded first_failure.
void test_strict_and_diagnostic_outcomes_remain_visible(bool strict) {
    bt::oracle::Node node(
        std::string(kNode) +
        "test_strict_and_diagnostic_outcomes_remain_visible[" +
        std::string(strict ? "True" : "False") + "]");
    const auto env =
        strict ? ft2_reference::research_variant(
                     "research_flat", std::nullopt, std::nullopt,
                     std::nullopt, std::nullopt, std::nullopt,
                     std::nullopt, true, std::nullopt, std::nullopt,
                     std::nullopt, std::nullopt, std::nullopt,
                     std::nullopt, 100.)
               : ft2_reference::research("research_flat");
    char label[40];
    for (std::size_t window = 0; window < 4; ++window) {
        try {
            std::snprintf(label, sizeof(label), "transition.%08zu",
                          window);
            ft2_reference::consume_tree(
                node, step(*env, motor_control(0.)).as_wire(), label);
        } catch (const runtime::InvalidReferenceRun &) {
            bt::require(strict, "a diagnostic run never raises the strict "
                                "reference gate");
            const auto outcome = reason(*env);
            bt::require(outcome && *outcome == "invalid_controller",
                        "the strict stop is invalid_controller");
            const auto &failure = env->snapshot().first_failure;
            bt::require(failure.has_value(),
                        "the monitor publishes its first failure");
            ft2_reference::consume_tree(node, failure->as_wire(),
                                        "first_failure");
            node.done();
            return;
        }
        if (done(*env)) break;
    }
    node.done();
}

// tests/reference/test_native_research_environment.py::
//   test_differing_control_and_sensor_periods_match_python[params]
// Each (control_steps, sensor_steps) pair runs the full flat episode;
// pipeline.counters pins the acquisition gate arithmetic.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_differing_control_and_sensor_periods_match_python(
    std::int64_t control_steps, std::int64_t sensor_steps,
    std::string_view params) {
    bt::oracle::Node node(
        std::string(kNode) +
        "test_differing_control_and_sensor_periods_match_python[" +
        std::string(params) + "]");
    const auto env = ft2_reference::research_variant(
        "research_flat", std::nullopt, std::nullopt, std::nullopt,
        control_steps, sensor_steps);
    bt::require(env->configuration().control_steps == control_steps &&
                    env->configuration().sensor_steps == sensor_steps,
                "the period wiring seats the requested step counts");
    std::int64_t boundary = 0;
    char label[48];
    while (!done(*env)) {
        std::snprintf(label, sizeof(label), "transition.%08lld",
                      static_cast<long long>(boundary));
        ft2_reference::consume_tree(
            node, step(*env, motor_control(0.)).as_wire(), label);
        const std::vector<double> state =
            env->snapshot().integration_state;
        node.f64(state, "integration state at step " +
                            std::to_string(env->snapshot().step));
        ++boundary;
    }
    const auto outcome = reason(*env);
    bt::require(outcome && *outcome == "duration",
                "the period sweep ends by duration");
    const auto status = env->status();
    const auto attempted =
        sw::integer(sw::required(status, "samples_attempted"));
    const auto dropped =
        sw::integer(sw::required(status, "samples_dropped"));
    bt::require(attempted ==
                        1 + (env->snapshot().step - 1) / sensor_steps,
                "one push per sensor_steps boundary plus the startup row");
    const std::vector<double> counters{
        static_cast<double>(attempted), static_cast<double>(dropped)};
    node.f64(counters, "pipeline.counters");
    ft2_reference::consume_tree(node, env->commands_requested(),
                                "commands_requested");
    ft2_reference::consume_tree(node, env->commands_applied(),
                                "commands_applied");
    node.done();
}
NATIVE_DIAG_POP

} // namespace

int main() {
    return bt::run([] {
        test_native_research_matches_every_external_transition();
        test_one_second_pair_with_one_e_minus_nine_gate();
        test_one_second_pair_complete_episode_gate();
        test_delay_full_commands_immediate_brakes_and_recording(1);
        test_delay_full_commands_immediate_brakes_and_recording(7);
        test_delay_full_commands_immediate_brakes_and_recording(20);
        test_delay_full_commands_immediate_brakes_and_recording(80);
        test_scripted_rider_and_demand_are_sampled_at_physics_rate();
        test_native_loop_never_calls_python_interval_or_rng();
        test_custom_rider_callback_runs_once_per_external_boundary();
        test_reset_seed_generation_snapshot_and_facade_ownership();
        test_invalid_boundary_calls_do_not_change_state();
        test_zero_budget_retains_single_window_and_no_tail_flush();
        test_strict_and_diagnostic_outcomes_remain_visible(false);
        test_strict_and_diagnostic_outcomes_remain_visible(true);
        test_differing_control_and_sensor_periods_match_python(
            8, 4, "0.01-0.005-8-4");
        test_differing_control_and_sensor_periods_match_python(
            10, 2, "0.0125-0.0025-10-2");
        test_differing_control_and_sensor_periods_match_python(
            4, 3, "0.005-0.00375-4-3");
        test_differing_control_and_sensor_periods_match_python(
            3, 5, "0.00375-0.00625-3-5");
        test_stop_on_ended_episode_preserves_original_reason();
    });
}
