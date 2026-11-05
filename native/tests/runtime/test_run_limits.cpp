// test_run_limits.cpp — RunLimits::for_track sizes the physical ride's
// step cap and wall-clock watchdog. It moved out of the removed legacy
// controls unchanged; these cases pin the same expression and errors.
#include "support/fixture_test.hpp"
#include "runtime/run_limits.hpp"

#include <cstdint>

namespace {
using runtime::RunLimits;

// The cap is twice the remaining distance at the minimum speed, truncated.
void cap_matches_the_formula() {
    const double dt = .0005, speed = 15. / 3.6;
    const auto limits = RunLimits::for_track(120., 5., dt);
    bt::require(limits.finish_x_m == 120., "finish is the track length");
    bt::require(limits.max_steps == static_cast<std::int64_t>(2. * (120. - 5.) / (speed * dt)),
                "step cap is 2*(length-start)/(min_speed*dt), truncated");
    bt::require(limits.max_wall_clock_s == 300., "default wall-clock cap is 300 s");
}

// Non-positive inputs and a start at or past the finish are rejected.
void rejects_bad_inputs() {
    bt::invalid([] { (void)RunLimits::for_track(120., 5., 0.); });
    bt::invalid([] { (void)RunLimits::for_track(120., 5., .0005, 300., 0.); });
    bt::invalid([] { (void)RunLimits::for_track(5., 5., .0005); });
    bt::invalid([] { (void)RunLimits::for_track(120., 5., .0005, 0.); });
}
} // namespace

int main() { return bt::run([] { cap_matches_the_formula(); rejects_bad_inputs(); }); }
