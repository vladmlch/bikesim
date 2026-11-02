// The four control-clock functions from test_control_clock.py (3.20).
// The solved-effort physical-step test is ported through the frozen
// env_request bundle (<platform>/control_clock/env_request/) and the
// golden capture seam: the monkeypatched compute-call counting has no
// native analog, but the observable contract — the ctrl vector held
// byte-identical on non-tick physics steps, the per-step solved-effort
// telemetry channel, the interval cadence and the reset semantics — is
// asserted on the episode artifact the native runtime produces.
#include "support/fixture_test.hpp"
#include "support/golden.hpp"
#include "runtime/control.hpp"
#include "configuration/model_config.hpp"
#include "golden/episode.hpp"
#include "research/environment.hpp"
#include "runtime/research.hpp"
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>

namespace {
void test_period_must_be_an_integer_multiple_of_the_timestep() {
    runtime::ControlClock const clock(.0005, .005);
    bt::require(clock.steps_per_period == 10, "period contains ten physics steps");
    constexpr std::array<bool, 12> ticks{true, false, false, false, false, false, false, false, false, false, true, false};
    for (std::size_t i = 0; i < ticks.size(); ++i) bt::require(clock.is_tick(static_cast<std::int64_t>(i)) == ticks[i], "control clock exact tick sequence");
    bt::invalid([] { runtime::ControlClock const invalid(.0005, .0012); });
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
        bt::invalid([&] { runtime::ControlClock const invalid(dt, period); });
    // Additional native negative control: the quotient overflows although
    // both durations are finite; no out-of-range floating-to-int conversion.
    bt::invalid([] { runtime::ControlClock const invalid(1e-300, 1e300); });
}

// control_clock/runtime_holds_ctrl_but_observes_solved_effort_each_step
// 403 physics steps over the flat-track welded env: ticks at steps
// 0,10,...,400 (41 advancing control windows — the oracle's monkeypatch
// counting of ArticulatedRiderController.compute has no native seam; the
// cadence is observed through the applied command count), ctrl held
// bitwise on non-tick steps, a solved-effort channel row per physics
// step, and reset returning the runtime to step 0.
[[gnu::noinline]] void
test_runtime_holds_ctrl_but_observes_solved_effort_each_step() {
    const auto env = bike::research::make_environment(
            bt::oracle::root() / bt::oracle::platform() / "control_clock" /
                    "env_request" / "request.toml");
    const runtime::RideControl zero{
            .motor_torque_nm = 0., .motor_limit_nm = std::nullopt,
            .human_torque_nm = 0., .crank_target_rate_rad_s = std::nullopt,
            .posture = std::nullopt, .rider_enabled = true};
    const auto episode = bike::golden::capture_episode(*env, 403, zero);
    bt::require(episode.controls.size() == 403,
                "one applied intent per physics step");
    // Every applied control encodes the held zero command.
    for (const auto &control : episode.controls) {
        const auto &object = bike::io::as_object(control, "controls[]");
        const auto *human = bike::io::find_member(object, "human_torque_nm");
        const auto *motor = bike::io::find_member(object, "motor_torque_nm");
        bt::require(human != nullptr && motor != nullptr,
                    "control rows carry the torque channels");
        bt::require(bike::io::as_double(*human, "human_torque_nm") == 0. &&
                            bike::io::as_double(*motor, "motor_torque_nm") == 0.,
                    "held zero command applied every step");
    }
    // The ctrl vector is written only on control ticks: for every
    // non-tick step the captured post-step ctrl row is byte-identical to
    // the previous step's.
    const auto ctrl = episode.ctrl_written.doubles(true);
    const std::size_t nu = episode.ctrl_written.shape.at(1);
    for (std::size_t i = 1; i < episode.controls.size(); ++i) {
        if (i % 10 == 0) continue;
        const auto now = std::span(ctrl).subspan(i * nu, nu);
        const auto prev = std::span(ctrl).subspan((i - 1) * nu, nu);
        bt::require(std::ranges::equal(std::as_bytes(now),
                                       std::as_bytes(prev)),
                    "ctrl held byte-identical on a non-tick step");
    }
    // Solved-effort telemetry: every published sample row carries the
    // solved_actuator_force_at_incoming_interval marker and a finite
    // rider_positive_power_w (the oracle drained 400 + 3 flushed).
    std::size_t effort_rows = 0;
    for (const auto &row : episode.rows) {
        bool marker = false, finite_power = false;
        for (const auto &[name, leaf] : row) {
            if (name.ends_with("rider_effort_observation"))
                marker = std::holds_alternative<std::string>(leaf) &&
                        std::get<std::string>(leaf) ==
                                "solved_actuator_force_at_incoming_interval";
            if (name.ends_with("rider_positive_power_w"))
                finite_power = std::holds_alternative<double>(leaf) &&
                        std::isfinite(std::get<double>(leaf));
        }
        if (marker) {
            bt::require(finite_power,
                        "effort sample carries a finite power channel");
            ++effort_rows;
        }
    }
    bt::require(effort_rows >= 400,
                "solved effort observed at the physics rate");
    // reset() clears the committed clock; the first post-reset step is a
    // fresh control tick (ctrl row free to move).
    env->reset();
    bt::require(env->snapshot().step == 0, "reset returns to step 0");
    const auto tail = bike::golden::capture_episode(*env, 10, zero);
    bt::require(tail.controls.size() == 10,
                "post-reset stepping resumes the control clock");
}
} // namespace
int main() { return bt::run([] {
    test_period_must_be_an_integer_multiple_of_the_timestep();
    test_held_command_is_returned_unchanged_between_ticks(); test_clock_rejects_invalid_durations();
    test_runtime_holds_ctrl_but_observes_solved_effort_each_step();
}); }
