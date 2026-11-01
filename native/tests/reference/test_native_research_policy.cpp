// Completion 3.33a. Source: tests/reference/test_native_research_policy.py.
// The seven external-policy lifecycle contracts, all self-contained: the
// parametrized policy_environment fixture runs each contract on the Python
// backend and the native backend separately — there is no oracle comparison
// to freeze — so the C++ suite re-asserts each contract directly on the
// native session.
//
// Substitutions:
//  * env.seed ↔ env.configuration().seed_decimal (a decimal string: '19'
//    seeds the fixture, so seeds == {"19", "47", "53"}).
//  * session.stop() ↔ request_stop(); env.trace ↔ transitions().
//  * env.error == 'RuntimeError: policy sentinel' ↔ status("error") ==
//    "policy sentinel" — the 'RuntimeError: ' decoration is the Python
//    adapter's exception-name prefix; the native layer stores the raw
//    policy message (an engine failure's stored text carries it).
//  * begin_advance(front_brake_demand=True) and reset(seed=True) are nanobind
//    type gates with no C++ spelling; the same validators are exercised
//    through their expressible domain rejections (out-of-range brake demand,
//    non-decimal seed).
#include "research_reference_support.hpp"

namespace {
using namespace research_reference;
namespace sw = runtime::sample_wire;

[[nodiscard]] bool done(const runtime::WireObject &status) {
    return flag(status, "terminated") || flag(status, "truncated");
}
[[nodiscard]] std::int64_t steps(const runtime::NativeResearchRuntime &env) {
    return sw::integer(sw::required(env.status(), "step"));
}
[[nodiscard]] double sim_time(const runtime::NativeResearchRuntime &env) {
    return sw::number(sw::required(env.status(), "time_s"));
}
[[nodiscard]] double brake(const runtime::Wire &command, std::string_view key) {
    return sw::number(sw::required(sw::object(command), key));
}
[[nodiscard]] std::unique_ptr<research::PolicySession>
session_of(CountingPolicy *&handle, const Knobs &knobs = {}) {
    auto policy = std::make_unique<CountingPolicy>();
    handle = policy.get();
    return std::make_unique<research::PolicySession>(
        environment(knobs), std::move(policy));
}

// tests/reference/test_native_research_policy.py::
//   test_one_act_per_window_and_queued_pause_brakes
void test_one_act_per_window_and_queued_pause_brakes() {
    CountingPolicy *policy;
    auto session = session_of(policy);
    auto &env = session->environment();
    bt::require(policy->seeds.size() == 1 &&
                    policy->seeds[0] == env.configuration().seed_decimal,
                "the policy reset carries the environment seed");
    session->begin_advance();
    for (int i = 0; i < 4; ++i)
        bt::require(!session->advance_pending(0.).has_value(),
                    "a zero-budget advance_pending is a pure poll");
    bt::require(policy->calls.size() == 1,
                "one window costs exactly one act call");
    bt::require(env.commands_requested().size() == 1,
                "one window publishes exactly one requested command");
    session->pause();
    session->set_brakes(.2, .1);
    bt::require(!session->paused(),
                "a queued pause is not latched while the window is in flight");
    const auto result = session->advance_pending();
    bt::require(result.has_value() && session->paused(),
                "the completed window applies the queued pause at the boundary");
    bt::require(policy->calls.size() == 1,
                "the queued actions never re-invoke the policy");
    bt::require(brake(env.commands_requested()[0], "front_brake_demand") == 0.,
                "the first window keeps the brakes it was begun with");
    throws_runtime([&] { session->begin_advance(); });
    bt::require(policy->calls.size() == 1,
                "a paused begin_advance does not reach the policy");
    session->resume();
    (void)session->advance();
    bt::require(policy->calls.size() == 2,
                "resume permits the next window's single act call");
    const auto commands = env.commands_requested();
    bt::require(brake(commands[1], "front_brake_demand") == .2 &&
                    brake(commands[1], "rear_brake_demand") == .1,
                "the queued brakes apply to the next requested command");
    const auto meta = session->run_metadata();
    const auto *intervention = bike::io::find_member(
        bike::io::as_object(meta, "run_metadata"), "operator_intervention");
    bt::require(intervention && bike::io::as_bool(*intervention, "operator_intervention"),
                "accepted brake demands record operator intervention");
}

// tests/reference/test_native_research_policy.py::
//   test_one_act_per_window_across_budget_yields
void test_one_act_per_window_across_budget_yields() {
    CountingPolicy *policy;
    auto session = session_of(policy);
    auto &env = session->environment();
    for (std::size_t window = 0; window < 3 && !done(env.status()); ++window) {
        session->begin_advance();
        bt::require(!session->advance_pending(0.).has_value(),
                    "a zero-budget call is a pure poll: no progress, no re-act");
        std::optional<runtime::ResearchTransition> result;
        for (int guard = 0; guard < 200 && !result; ++guard)
            result = session->advance_pending(.008);
        bt::require(result.has_value(), "the window completes under budget yields");
        bt::require(policy->calls.size() == window + 1 &&
                        env.commands_requested().size() == window + 1 &&
                        env.transitions().size() == window + 1,
                    "each completed window costs exactly one act/request/transition");
    }
}

// tests/reference/test_native_research_policy.py::
//   test_pause_and_resume_do_not_advance_simulated_time
void test_pause_and_resume_do_not_advance_simulated_time() {
    CountingPolicy *policy;
    auto session = session_of(policy);
    auto &env = session->environment();
    (void)session->advance();
    const auto before_steps = steps(env);
    const auto before_calls = policy->calls.size();
    const double before_time = sim_time(env);
    session->pause();
    bt::require(session->paused(), "pause latches when no window is pending");
    throws_runtime([&] { (void)session->advance_pending(); });
    throws_runtime([&] { (void)session->advance(); });
    session->resume();
    bt::require(!session->paused(), "resume clears the latch");
    bt::require(steps(env) == before_steps && sim_time(env) == before_time,
                "pause/resume never advances simulated time");
    bt::require(policy->calls.size() == before_calls,
                "pause/resume never re-invokes the policy");
    (void)session->advance();
    bt::require(steps(env) == before_steps + env.configuration().control_steps,
                "the next window advances exactly control_steps");
    bt::require(policy->calls.size() == before_calls + 1,
                "the next window costs exactly one act call");
}

// tests/reference/test_native_research_policy.py::
//   test_stop_and_reset_are_deferred_to_external_boundary
void test_stop_and_reset_are_deferred_to_external_boundary() {
    CountingPolicy *policy;
    auto session = session_of(policy);
    auto &env = session->environment();
    session->begin_advance();
    bt::require(!session->advance_pending(0.).has_value(),
                "the fresh window accepts a zero-budget poll");
    session->request_stop();
    bt::require(!done(env.status()),
                "a queued stop does not end the episode mid-window");
    (void)session->advance_pending();
    bt::require(text(env.status(), "reason") == "operator_stop" &&
                    !session->pending(),
                "the queued stop truncates at the external boundary");
    session->reset("47");
    bt::require(steps(env) == 0 && !done(env.status()),
                "an unqueued reset restarts the episode immediately");
    bt::require(policy->seeds == std::vector<std::string>({"19", "47"}),
                "each boundary reset re-seeds the policy");
    session->begin_advance();
    session->reset("53");
    session->set_brakes(0., .3);
    bt::require(env.configuration().seed_decimal == "47",
                "a queued reset does not touch the running episode");
    const auto previous = session->advance_pending();
    bt::require(previous && previous->physics_steps == env.configuration().control_steps,
                "the in-flight window still completes before the reset applies");
    bt::require(env.configuration().seed_decimal == "53" && steps(env) == 0,
                "the queued reset applies at the external boundary");
    bt::require(policy->seeds == std::vector<std::string>({"19", "47", "53"}),
                "the boundary reset re-seeds the policy a third time");
    (void)session->advance();
    bt::require(brake(env.commands_requested()[0], "rear_brake_demand") == .3,
                "the queued brakes carry into the post-reset window");
    bt::require(policy->calls.size() == 3,
                "three windows, three act calls — resets never call act");
}

// tests/reference/test_native_research_policy.py::
//   test_invalid_operator_values_do_not_call_policy
void test_invalid_operator_values_do_not_call_policy() {
    CountingPolicy *policy;
    auto session = session_of(policy);
    auto &env = session->environment();
    // front_brake_demand=True is a nanobind bool-gate; the expressible leg is
    // the same domain validator on an out-of-range demand.
    bt::invalid([&] { session->begin_advance(1.5); });
    bt::invalid([&] { session->set_brakes(0., 2.); });
    // reset(seed=True) is the same nanobind gate; the expressible leg is a
    // non-decimal seed rejected by validate_seed.
    bt::invalid([&] { session->reset("not-a-seed"); });
    bt::require(policy->calls.empty(),
                "rejected operator actions never reach the policy");
    bt::require(!done(env.status()) && !session->pending(),
                "rejected operator actions leave the session idle");
}

// tests/reference/test_native_research_policy.py::
//   test_policy_failure_has_no_manufactured_transition
void test_policy_failure_has_no_manufactured_transition() {
    class BrokenPolicy final : public CountingPolicy {
    public:
        [[nodiscard]] runtime::RideControl act(
            const runtime::SensorObservation &observation,
            std::optional<double> demand_nm) override {
            (void)observation;
            (void)demand_nm;
            throw std::runtime_error("policy sentinel");
        }
    };
    auto policy = std::make_unique<BrokenPolicy>();
    auto *broken = policy.get();
    auto session = std::make_unique<research::PolicySession>(
        environment(), std::move(policy));
    auto &env = session->environment();
    throws_runtime([&] { session->begin_advance(); }, "policy sentinel");
    const auto status = env.status();
    bt::require(flag(status, "terminated") &&
                    text(status, "reason") == "policy_error" &&
                    text(status, "error") == "policy sentinel",
                "the act failure terminates with the policy_error contract");
    bt::require(steps(env) == 0,
                "the failed policy never produced simulated time");
    bt::require(env.commands_requested().empty() && env.transitions().empty(),
                "a failed policy yields no manufactured command or transition");
    env.stop();
    bt::require(text(env.status(), "reason") == "policy_error",
                "an operator stop cannot relabel the policy_error outcome");
    (void)broken;
}

// tests/reference/test_native_research_policy.py::
//   test_policy_cannot_smuggle_rider_inputs
void test_policy_cannot_smuggle_rider_inputs() {
    class RiderPolicy final : public CountingPolicy {
    public:
        RiderPolicy() { command.human_torque_nm = 0.; }
    };
    auto policy = std::make_unique<RiderPolicy>();
    auto session = std::make_unique<research::PolicySession>(
        environment(), std::move(policy));
    auto &env = session->environment();
    invalid_with([&] { (void)session->advance(); }, "rider inputs");
    bt::require(text(env.status(), "reason") == "policy_error",
                "the smuggled rider input terminates as policy_error");
    bt::require(!session->pending(),
                "no window survives the rejected policy command");
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        test_one_act_per_window_and_queued_pause_brakes();
        test_one_act_per_window_across_budget_yields();
        test_pause_and_resume_do_not_advance_simulated_time();
        test_stop_and_reset_are_deferred_to_external_boundary();
        test_invalid_operator_values_do_not_call_policy();
        test_policy_failure_has_no_manufactured_transition();
        test_policy_cannot_smuggle_rider_inputs();
    });
}
