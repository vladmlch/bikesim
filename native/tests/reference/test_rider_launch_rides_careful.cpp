// test_rider_launch_rides_careful.cpp — launch part 4, task 6: the careful
// rider (rider_type_careful.toml) launch rides against decision 10 and
// P12/P13 thresholds.
#include <print>

#include "launch_ride_metrics.hpp"
#include "support/fixture_test.hpp"

namespace {

[[nodiscard]] launch_ride::LaunchRule careful_rule(const launch_ride::LaunchCase &c) {
    if (c.name == "g20") return launch_ride::LaunchRule::short_lift;
    if (c.name == "g22" || c.name == "g25" || c.name == "loose25") {
        return launch_ride::LaunchRule::steep_20s;
    }
    return launch_ride::LaunchRule::clean;
}

void test_careful_rider_restart_25_percent() {
    const auto r = launch_ride::ride_restart(
        "examples/research/rider_type_careful.toml", .25, "asphalt");
    std::println(
        "[restart careful g25] pre_min={:.3f} pre_teeth={} stop_min={:.4f} "
        "hold={:.3f} rel={:.3f} teeth={} ho={:.3f} post_max={:.3f} "
        "post_lifts={} sad_min={:.1f} bal={}",
        r.pre_stop_cruise_min_speed_mps, r.pre_stop_rear_teeth,
        r.min_abs_speed_during_stop_mps, r.launch.hold_s.value_or(-1.),
        r.launch.release_s.value_or(-1.), r.hold_rear_teeth_at_release,
        r.launch.handover_s.value_or(-1.), r.post_restart_max_speed_mps,
        r.launch.post_lifts.size(), r.launch.whole_ride_saddle_min_n,
        r.launch.whole_ride_balance_lost ? 1 : 0);
    bt::require(r.pre_stop_cruise_min_speed_mps > 2.0,
                "careful restart g25 pre-stop cruise > 2.0 m/s");
    bt::require(r.launch.hold_s.has_value(),
                "careful restart g25 enters launch hold");
    bt::require(r.min_abs_speed_during_stop_mps <= 0.1,
                "careful restart g25 comes to a full stop");
    bt::require(r.hold_rear_teeth_at_release == 39,
                "careful restart g25 downshifts to 39T climbing gear before release");
    bt::require(r.launch.release_s.has_value() && r.launch.handover_s.has_value(),
                "careful restart g25 releases and hands over cleanly");
    const auto failures = launch_ride::seated_limit_failures(
        {.name = "restart_g25", .grade = .25, .surface = "asphalt"}, r.launch);
    bt::require(failures.empty(),
                "careful restart g25 satisfies seated limit checks");
    bt::require(r.launch.whole_ride_saddle_min_n >= -6. &&
                    r.launch.whole_ride_front_foot_min_n >= -6. &&
                    r.launch.whole_ride_rear_foot_min_n >= -6.,
                "careful restart g25 whole-ride supports remain >= -6 N");
    bt::require(!r.launch.whole_ride_balance_lost,
                "careful restart g25 preserves balance");
    bt::require(r.launch.post_lifts.empty(),
                "careful restart g25 has no post-handover front lifts");
    bt::require(r.post_restart_max_speed_mps > 3.0,
                "careful restart g25 recovers speed > 3.0 m/s after handover");
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        launch_ride::run_launch_suite(
            "careful", "examples/research/rider_type_careful.toml",
            careful_rule);
        test_careful_rider_restart_25_percent();
    });
}
