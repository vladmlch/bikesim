// Completion 3.33b. Source: tests/reference/test_research_frontend.py.
// The policy-session pacing contract: partial ticks call act() once and a
// queued stop drains only the already accepted window.
//
// Substitution notes: advance_playback(session, N) is the frontend's
// (.008 s wall budget, target_step N) helper pair; stop_at_boundary(session)
// is request_stop plus a drain loop. Both are spelled out over the native
// session so each latched-window step stays explicit.
#include "research_reference_support.hpp"

namespace {
using namespace research_reference;
namespace sw = runtime::sample_wire;

// tests/reference/test_research_frontend.py::
//   test_partial_ticks_call_policy_once_and_stop_does_not_start_another
void test_partial_ticks_call_policy_once_and_stop_does_not_start_another() {
    auto env = environment();
    auto *const environment_ptr = env.get();
    auto counting = std::make_unique<CountingPolicy>();
    auto *const policy = counting.get();
    research::PolicySession session(std::move(env), std::move(counting));
    // advance_playback(session, 1): begin the window and deliver at most to
    // step 1 — the window stays latched and act() ran exactly once.
    session.begin_advance();
    bt::require(!session.advance_pending(.008, 1).has_value(),
                "a partial tick returns before the control boundary");
    bt::require(policy->calls.size() == 1 && session.pending() &&
                    environment_ptr->snapshot().step == 1,
                "one act, one delivered step, the pending latch held");
    // advance_playback(session, 2): the same latched command, no second act.
    bt::require(!session.advance_pending(.008, 2).has_value(),
                "the second partial tick still holds the window");
    bt::require(environment_ptr->snapshot().step == 2 && policy->calls.size() == 1,
                "no second act while the accepted window advances");
    session.pause();
    // stop_at_boundary(session): queue the stop and drain only the window
    // already accepted.
    session.request_stop();
    while (session.pending()) (void)session.advance_pending(.008);
    bt::require(!session.pending(), "the drained window releases the latch");
    bt::require(environment_ptr->snapshot().step ==
                    environment_ptr->configuration().control_steps,
                "the stop completes exactly the accepted window");
    bt::require(policy->calls.size() == 1 &&
                    environment_ptr->commands_requested().size() == 1 &&
                    environment_ptr->transitions().size() == 1,
                "one policy act, one requested command, one transition");
    bt::require(text(environment_ptr->status(), "reason") == "operator_stop",
                "the queued stop resolves at the boundary");
    session.close();
}

} // namespace

int main() {
    return bt::run([] {
        test_partial_ticks_call_policy_once_and_stop_does_not_start_another();
    });
}
