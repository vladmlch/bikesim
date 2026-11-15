// test_rider_launch_rides_normal.cpp — launch part 4, task 6: the normal
// rider (viewer_physics_welded.toml) launch rides against decision 10 and
// P11/P13 thresholds.
#include <array>
#include <format>
#include <print>
#include <string>
#include <vector>

#include "launch_ride_metrics.hpp"
#include "support/fixture_test.hpp"

namespace {

[[nodiscard]] launch_ride::LaunchRule normal_rule(const launch_ride::LaunchCase &c) {
    if (c.name == "g00") return launch_ride::LaunchRule::clean_with_t20;
    if (c.name == "g20") return launch_ride::LaunchRule::short_lift;
    if (c.name == "g22" || c.name == "g25" || c.name == "loose25") {
        return launch_ride::LaunchRule::steep_20s;
    }
    return launch_ride::LaunchRule::clean;
}

void test_normal_rider_restart_after_a_stop() {
    struct RestartSpec {
        const char *name;
        double grade;
        int expected_cog;
    };
    constexpr std::array<RestartSpec, 2> kSpecs{{
        {.name = "restart_g05", .grade = .05, .expected_cog = 33},
        {.name = "restart_g10", .grade = .10, .expected_cog = 39},
    }};

    std::vector<std::string> all_failures;
    std::println(stderr, "=== normal rider mid-ride stop & restart ===");
    std::println(
        stderr,
        "{:<12} {:>7} {:>9} {:>7} {:>8} {:>9} {:<16} {:<10} {:>7} {:>8} {:>7} {:>20} {:<24} {}",
        "case", "cruise", "stop_spd", "cog(p/r)", "stop_fN", "post_spd",
        "lifts", "slips", "slip_pk", "rb_mm", "cad_rpm", "sad/fF/fR_N",
        "phases(h/r/a/ho)", "verdict");

    for (const auto &spec : kSpecs) {
        const auto rm = launch_ride::ride_restart(
            "examples/research/viewer_physics_welded.toml", spec.grade, "asphalt");
        const launch_ride::LaunchCase c{
            .name = spec.name, .grade = spec.grade, .surface = "asphalt"};
        auto failures = launch_ride::clean_start_failures(c, rm.launch);
        if (!(rm.pre_stop_cruise_min_speed_mps > 2.0)) {
            failures.push_back(
                std::format("{}: pre-stop cruise min speed {:.3f} <= 2.0 m/s",
                            spec.name, rm.pre_stop_cruise_min_speed_mps));
        }
        if (!rm.launch.hold_s.has_value()) {
            failures.push_back(std::string(spec.name) + ": never entered hold during stop");
        }
        if (!(rm.min_abs_speed_during_stop_mps <= 0.1)) {
            failures.push_back(
                std::format("{}: did not reach standstill (min |speed| {:.3f} > 0.1 m/s)",
                            spec.name, rm.min_abs_speed_during_stop_mps));
        }
        if (rm.hold_rear_teeth_at_release != spec.expected_cog) {
            failures.push_back(
                std::format("{}: hold_rear_teeth_at_release {} != expected {}",
                            spec.name, rm.hold_rear_teeth_at_release, spec.expected_cog));
        }
        if (!rm.launch.release_s.has_value() || !rm.launch.handover_s.has_value()) {
            failures.push_back(
                std::string(spec.name) + ": restart did not reach release and handover");
        } else if (*rm.launch.handover_s - *rm.launch.release_s >= 6.0) {
            failures.push_back(
                std::format("{}: restart handover took {:.3f} s (>= 6.0 s)",
                            spec.name, *rm.launch.handover_s - *rm.launch.release_s));
        }
        if (!(rm.post_restart_max_speed_mps > 5.0)) {
            failures.push_back(
                std::format("{}: post-restart max speed {:.3f} <= 5.0 m/s",
                            spec.name, rm.post_restart_max_speed_mps));
        }

        const std::string verdict =
            failures.empty()
                ? "PASS"
                : "FAIL: " + launch_ride::detail::join_failures(failures);
        for (auto &f : failures) all_failures.push_back(std::move(f));

        const std::string cogs_str = std::format(
            "{}/{}", rm.pre_stop_rear_teeth, rm.hold_rear_teeth_at_release);
        const std::string lifts_str = launch_ride::detail::format_episodes(rm.launch.lifts);
        const std::string slips_str = launch_ride::detail::format_episodes(rm.launch.slips);
        const std::string supports_str = std::format(
            "{:+.1f}/{:+.1f}/{:+.1f}", rm.launch.saddle_min_n,
            rm.launch.front_foot_min_n, rm.launch.rear_foot_min_n);
        const std::string phases_str = std::format(
            "{}/{}/{}/{}", launch_ride::detail::format_opt(rm.launch.hold_s),
            launch_ride::detail::format_opt(rm.launch.release_s),
            launch_ride::detail::format_opt(rm.launch.accelerate_s),
            launch_ride::detail::format_opt(rm.launch.handover_s));
        std::println(
            stderr,
            "{:<12} {:>7.3f} {:>9.4f} {:>7} {:>8.1f} {:>9.3f} {:<16} {:<10} {:>7.3f} {:>+8.2f} {:>7.1f} {:>20} {:<24} {}",
            spec.name, rm.pre_stop_cruise_min_speed_mps,
            rm.min_abs_speed_during_stop_mps, cogs_str,
            rm.stop_min_front_normal_n, rm.post_restart_max_speed_mps,
            lifts_str, slips_str, rm.launch.slip_peak,
            rm.launch.rollback_m * 1000., rm.launch.leg_ema_peak_rpm,
            supports_str, phases_str, verdict);
    }

    bt::require(all_failures.empty(),
                launch_ride::detail::join_failures(all_failures));
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        launch_ride::run_launch_suite(
            "normal", "examples/research/viewer_physics_welded.toml",
            normal_rule);
        test_normal_rider_restart_after_a_stop();
        launch_ride::verify_restart_25_percent(
            {.label = "normal",
             .profile = "examples/research/viewer_physics_welded.toml",
             .expected_cog = 39});
    });
}
