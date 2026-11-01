// Completion 3.33a. Source: tests/reference/test_native_research_frontend.py.
// Partial ticks latch the policy command until the external boundary; a
// queued operator stop resolves to operator_stop at that boundary. The
// viewer's advance_playback/stop_at_boundary slices map to the same
// PolicySession calls the Python wrappers issue.
#include "research_reference_support.hpp"

namespace {
using namespace research_reference;
namespace sw = runtime::sample_wire;

// tests/reference/test_native_research_frontend.py::
//   test_native_partial_ticks_latch_policy_until_external_boundary
void test_native_partial_ticks_latch_policy_until_external_boundary() {
    auto env = environment();
    auto const *const environment_ptr = env.get();
    auto counting = std::make_unique<CountingPolicy>();
    auto const *const policy = counting.get();
    research::PolicySession session(std::move(env), std::move(counting));
    // advance_playback(session, 1): one bounded slice — begin the window,
    // deliver at most to step 1 and keep the pending latch.
    session.begin_advance();
    (void)session.advance_pending(.008, 1);
    bt::require(environment_ptr->snapshot().step == 1 && session.pending(),
                "a partial tick keeps the window latched");
    for (std::size_t poll = 0; poll < 3; ++poll)
        (void)session.advance_pending(0., std::nullopt);
    bt::require(environment_ptr->snapshot().step == 1 && policy->calls.size() == 1,
                "zero-budget polls are pure: no steps, no extra act");
    session.set_brakes(.5, .5);
    // stop_at_boundary: queue the stop, then drain only the active window.
    session.request_stop();
    while (session.pending()) (void)session.advance_pending(.008, std::nullopt);
    bt::require(environment_ptr->snapshot().step ==
                    environment_ptr->configuration().control_steps,
                "the stop completes only the active window");
    const auto requested = environment_ptr->commands_requested();
    bt::require(policy->calls.size() == 1 && requested.size() == 1,
                "exactly one policy act and one requested command");
    const auto &first = sw::object(requested.front());
    bt::require(sw::number(sw::required(first, "front_brake_demand")) == 0.,
                "the latched command predates the queued brakes");
    bt::require(text(environment_ptr->status(), "reason") == "operator_stop",
                "the queued stop resolves at the boundary");
}

} // namespace

int main() {
    return bt::run([] {
        test_native_partial_ticks_latch_policy_until_external_boundary();
    });
}
