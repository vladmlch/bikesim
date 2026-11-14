// test_rider_launch_rides_normal.cpp — launch part 4, task 6: the normal
// rider (viewer_physics_welded.toml) launch rides against decision 10 and
// P11/P13 thresholds.
#include "launch_ride_metrics.hpp"
#include "support/fixture_test.hpp"

namespace {

[[nodiscard]] launch_ride::LaunchRule normal_rule(const launch_ride::LaunchCase &c) {
    if (c.name == "g00") return launch_ride::LaunchRule::clean_with_t20;
    if (c.name == "g20") return launch_ride::LaunchRule::short_lift;
    if (c.name == "g22" || c.name == "g25") return launch_ride::LaunchRule::info;
    return launch_ride::LaunchRule::clean;
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        launch_ride::run_launch_suite(
            "normal", "examples/research/viewer_physics_welded.toml",
            normal_rule);
    });
}
