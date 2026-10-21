// Source successor of all seven test_playback_clock.py functions (3.20).
#include "support/migration_test.hpp"
#include "research/policy_session.hpp"
#include "viewer/input.hpp"
#include <array>
#include <limits>
#include <type_traits>

namespace {
using bike::research::PlaybackClock;
namespace view = bike::viewer;
void test_budget_yield_preserves_uncompleted_debt() {
    PlaybackClock clock(.00125); clock.set_scale(2, 0., 0); clock.rebase(0., 0);
    bt::require(clock.target(.01, 0) == 16, "2x pacing requests sixteen steps");
    bt::require(clock.target(.01, 8) == 16, "budget yield retains eight outstanding steps");
    clock.set_scale(4, .01, 8);
    bt::require(clock.target(.02, 8) == 40, "speed change rebases completed prefix");
}
void test_catchup_is_capped_and_counted_from_completed_steps() {
    PlaybackClock clock(.00125); clock.set_scale(8, 0., 10); clock.rebase(0., 10);
    bt::require(clock.target(10., 10) == 330, "catchup cap is scaled from the completed prefix");
    bt::require(clock.target(10., 20) == 330, "completed steps reduce debt once");
    clock.rebase(10., 0);
    bt::require(clock.target(10., 0) == 0, "reset clears old catchup debt");
}
void test_fractional_remainder_and_regressing_wall_time() {
    PlaybackClock clock(.01); clock.rebase(0., 0);
    bt::require(clock.target(.007, 0) == 0, "fractional first step remains debt");
    bt::require(clock.target(.012, 0) == 1, "fraction accumulates to one step");
    bt::require(clock.target(.009, 1) == 1, "regressing wall time creates no negative progress");
    bt::require(clock.target(.019, 1) == 1, "regressed interval is never double-counted");
    bt::require(clock.target(.022, 1) == 2, "remaining fraction reaches the next step");
}
void test_pause_resume_and_scale_rebase_discard_backlog() {
    PlaybackClock clock(.00125); clock.rebase(0., 0);
    bt::require(clock.target(1., 0) == 40, "normal catchup is capped at fifty milliseconds");
    clock.set_paused(true, 1., 0);
    bt::require(clock.target(100., 0) == 0, "pause discards backlog");
    clock.set_paused(false, 100., 0);
    bt::require(clock.target(100., 0) == 0, "resume rebases wall time");
    clock.set_scale(2, 101., 0);
    bt::require(clock.target(101., 0) == 0, "scale change discards backlog");
    clock.rebase(102., 20);
    bt::invalid([&] { (void)clock.target(102., 0); });
}
void test_invalid_timestep() {
    for (double dt : {0., -1., std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
        bt::invalid([&] { PlaybackClock invalid(dt); });
    bt::require(!std::is_constructible_v<PlaybackClock, bool> && !std::is_constructible_v<PlaybackClock, const char*>, "typed pacing API rejects bool and text timesteps");
}
template<class T> concept ScaleAccepted = requires(PlaybackClock& clock, T scale) { clock.set_scale(scale, 0., 0); };
void test_invalid_scale() {
    PlaybackClock clock(.00125);
    for (int scale : {0, 3, 16}) bt::invalid([&] { clock.set_scale(scale, 0., 0); });
    bt::require(ScaleAccepted<int> && !ScaleAccepted<bool> && !ScaleAccepted<double> && !ScaleAccepted<const char*>, "only an integer scale reaches runtime validation");
}
void test_speed_keys_do_not_reuse_physical_controls() {
    const auto speed = [](int key, unsigned current) { return view::speed_for_action(view::action_for_key(view::Mode::physical, key), current); };
    bt::require(speed(295, 2) == 1 && speed(296, 2) == 4 && speed(297, 8) == 1, "F6/F7/F8 are speed controls");
    bt::require(speed(295, 1) == 1 && speed(296, 8) == 8, "speed keys saturate at supported limits");
    for (int key : std::array<int, 7>{32, 44, 46, 'B', 'R', 'T', 'G'}) {
        const auto action = view::action_for_key(view::Mode::physical, key);
        bt::require(action != view::Action::slower && action != view::Action::faster && action != view::Action::normal_speed, "physical controls are separate from pacing controls");
        bt::require(speed(key, 2) == 2, "non-speed key preserves time scale");
    }
}
} // namespace
int main() { return bt::run([] {
    test_budget_yield_preserves_uncompleted_debt(); test_catchup_is_capped_and_counted_from_completed_steps();
    test_fractional_remainder_and_regressing_wall_time(); test_pause_resume_and_scale_rebase_discard_backlog();
    test_invalid_timestep(); test_invalid_scale(); test_speed_keys_do_not_reuse_physical_controls();
}); }
