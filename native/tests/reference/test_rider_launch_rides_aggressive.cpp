// test_rider_launch_rides_aggressive.cpp — launch part 4, task 6: the
// aggressive rider (rider_type_aggressive.toml) launch rides against decision 10
// and P10/P13 thresholds.
#include "launch_ride_metrics.hpp"
#include "support/fixture_test.hpp"

namespace {

[[nodiscard]] launch_ride::LaunchRule aggressive_rule(const launch_ride::LaunchCase &c) {
    if (c.name == "g15" || c.name == "g20" || c.name == "g17" ||
        c.name == "wet17" || c.name == "loose17") {
        return launch_ride::LaunchRule::controller_window;
    }
    if (c.name == "g22" || c.name == "g25" || c.name == "loose25") {
        return launch_ride::LaunchRule::steep_20s;
    }
    return launch_ride::LaunchRule::seated_only;
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        launch_ride::run_launch_suite(
            "aggressive", "examples/research/rider_type_aggressive.toml",
            aggressive_rule);
    });
}
