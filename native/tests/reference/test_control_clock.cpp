// Three pure control-clock functions from test_control_clock.py (3.20).
// The fourth, solved-effort physical-step test, remains a separate unported
// integration assertion; these unit contracts must not claim its coverage.
#include "support/migration_test.hpp"
#include "runtime/control.hpp"
#include "configuration/model_config.hpp"
#include <array>
#include <limits>
#include <type_traits>

namespace {
void test_period_must_be_an_integer_multiple_of_the_timestep() {
    runtime::ControlClock clock(.0005, .005);
    bt::require(clock.steps_per_period == 10, "period contains ten physics steps");
    constexpr std::array<bool, 12> ticks{true, false, false, false, false, false, false, false, false, false, true, false};
    for (std::size_t i = 0; i < ticks.size(); ++i) bt::require(clock.is_tick(static_cast<std::int64_t>(i)) == ticks[i], "control clock exact tick sequence");
    bt::invalid([] { runtime::ControlClock invalid(.0005, .0012); });
    bike::configuration::SimulationPhysicsConfig config;
    config.physics_mode = "physical"; config.timestep_s = .0005; config.control_period_s = .0012;
    bt::invalid([&] { bike::configuration::validate(config); });
}
void test_held_command_is_returned_unchanged_between_ticks() {
    runtime::ControlClock clock(.0005, .005);
    const auto empty_rejected = [&] {
        bool caught = false; try { (void)clock.held(); } catch (const std::runtime_error& error) { caught = std::string_view(error.what()).find("no command") != std::string_view::npos; }
        bt::require(caught, "held command fails explicitly before first hold or after reset");
    };
    empty_rejected();
    rider::NamedEntries<double> command{{"rider_knee_front", 12.5}, {"rider_hip_front", -3.}};
    clock.hold(command); command.front().second = 1.;
    auto held_copy = clock.held(); held_copy.front().second = 0.;
    const rider::NamedEntries<double> expected{{"rider_knee_front", 12.5}, {"rider_hip_front", -3.}};
    bt::require(clock.held() == expected, "hold owns its input and returned snapshots do not mutate it");
    bt::require(std::is_const_v<std::remove_reference_t<decltype(clock.held())>>, "borrowed C++ command view is const");
    clock.reset(); empty_rejected();
}
void test_clock_rejects_invalid_durations() {
    const double inf = std::numeric_limits<double>::infinity();
    for (const auto& [dt, period] : std::array{std::pair{0., .005}, std::pair{.0005, inf}, std::pair{.0005, -.005}})
        bt::invalid([&] { runtime::ControlClock invalid(dt, period); });
    // Additional native negative control: the quotient overflows although
    // both durations are finite; no out-of-range floating-to-int conversion.
    bt::invalid([] { runtime::ControlClock invalid(1e-300, 1e300); });
}
} // namespace
int main() { return bt::run([] {
    test_period_must_be_an_integer_multiple_of_the_timestep();
    test_held_command_is_returned_unchanged_between_ticks(); test_clock_rejects_invalid_durations();
}); }
