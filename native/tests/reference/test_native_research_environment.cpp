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
#include "research_reference_support.hpp"

namespace {
using namespace research_reference;
namespace sw = runtime::sample_wire;

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

} // namespace

int main() {
    return bt::run([] {
        test_native_loop_never_calls_python_interval_or_rng();
        test_invalid_boundary_calls_do_not_change_state();
        test_zero_budget_retains_single_window_and_no_tail_flush();
        test_stop_on_ended_episode_preserves_original_reason();
    });
}
