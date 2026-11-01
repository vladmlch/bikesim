// Completion 3.33a. Source: tests/reference/test_native_research_failures.py.
// The injected engine-failure boundary: prefix preservation at the failure
// step and full reset recovery.
//
// Substitution: the Python source wraps the probe in a child process because
// the adapter must prove a *fatal* engine path is isolated from the
// interpreter. The native production boundary is already in-process safe —
// test_fail_at_step runs the same solver-99 forward() probe under an
// AdvancementGuard, and T10/T36 exercise it in-process. Every assertion in
// the child's JSON report is pinned here against the same runtime object.
#include "research_reference_support.hpp"

namespace {
using namespace research_reference;
namespace sw = runtime::sample_wire;

// tests/reference/test_native_research_failures.py::
//   test_engine_failure_preserves_prefix_and_reset_recovers
void test_engine_failure_preserves_prefix_and_reset_recovers() {
    auto env = environment();
    const auto initial = env->snapshot();
    env->test_fail_at_step(3);
    env->begin_control(motor_control(0.), 0., 0.);
    throws_runtime([&] {
        (void)env->advance_control(std::nullopt, std::nullopt); });
    const auto status = env->status();
    bt::require(sw::integer(sw::required(status, "step")) == 3,
                "the committed prefix stops at the injected step");
    bt::require(text(status, "reason") == "simulation_error",
                "an engine failure records simulation_error");
    const auto &error = sw::required(status, "error");
    bt::require(!sw::is_none(error) &&
                    !sw::string(error).empty(),
                "the failure evidence carries an error message");
    bt::require(sw::string(error).starts_with("RuntimeError: "),
                "the stored error preserves the runtime prefix");
    bt::require(!flag(status, "delivery_pending") && !flag(status, "active"),
                "no pending control is manufactured for a failed window");
    bt::require(env->transitions().empty(),
                "no successful transition is fabricated");
    bt::require(env->commands_requested().size() == 1,
                "the single requested command stays in the prefix");
    bt::require(sw::integer(sw::required(status, "recorded_rows")) == 3,
                "the recorder holds exactly the committed sample prefix");
    bt::require(env->snapshot().step == 3,
                "the frozen snapshot keeps the failure step");
    reseed(*env, "29");
    const auto recovered = env->status();
    bt::require(env->snapshot().step == 0 &&
                    !flag(recovered, "terminated") && !flag(recovered, "truncated"),
                "reset clears the failed episode to step zero");
    bt::require(env->snapshot().generation > initial.generation,
                "reset bumps the runtime generation");
    const auto transition = step(*env, motor_control(0.));
    bt::require(transition.physics_steps == env->configuration().control_steps,
                "the recovered runtime commits a full control window");
    env->close();
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        test_engine_failure_preserves_prefix_and_reset_recovers();
    });
}
