// The four control-clock functions from test_control_clock.py (3.20).
// The solved-effort physical-step test is ported through the frozen
// env_request bundle (<platform>/control_clock/env_request/) and the
// golden capture seam: the monkeypatched compute-call counting has no
// native analog, but the observable contract — the ctrl channels held
// byte-identical on non-tick physics steps wherever the writer
// reproduces the tick value, the every-step rider channels observed
// to update between ticks, the per-step solved-effort telemetry
// channel, the interval cadence and the reset semantics — is asserted
// on the episode artifact the native runtime produces.
#include "support/fixture_test.hpp"
#include "support/golden.hpp"
#include "support/record_initial_state.hpp"
#include "runtime/control.hpp"
#include "configuration/model_config.hpp"
#include "engine_call.hpp"
#include "golden/episode.hpp"
#include "model/builder.hpp"
#include "research/environment.hpp"
#include "runtime/research.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
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
    config.monitors.grounded_hold_s = .005; config.monitors.contact_dropout_s = .005;
    config.seated_climb.period_s = .00125;
    config.seated_climb.reaction_delay_s = .15;
    config.seated_climb.lean_rate_rad_s = .5;
    config.seated_climb.orientation_tau_s = .5;
    config.seated_climb.surge_budget_s = 15.;
    config.seated_climb.trim_dead_time_s = .2;
    config.seated_climb.min_elbow_flexion_deg = 15.0;
    config.rider_launch = {.start_cog_grade_breaks = {}, .start_cog_teeth = {24},
                           .start_pedal_angle_rad = 0., .start_lean_grades = {0.},
                           .start_lean_rad = {0.},
                           .plan_enabled = false, .standstill_speed_mps = .1,
                           .standstill_crank_rpm = 5., .effort_rate_nm_s = 40.,
                           .effort_cap_asphalt_nm = 60., .effort_cap_hardpack_nm = 60.,
                           .effort_cap_loose_nm = 60., .effort_cap_wet_nm = 60.,
                           .effort_cap_scree_nm = 60., .effort_cap_grades = {0.},
                           .effort_cap_grade_nm = {1000.}, .effort_rise_speed_mps = 2.5,
                           .release_torque_grades = {0., .2}, .release_torque_nm = {0., 30.},
                           .brake_release_s = .15, .max_hold_s = 1.5,
                           .upshift_crank_rpm = 82., .shift_retry_s = .25,
                           .end_speed_mps = 3.,
                           .end_crank_rpm = 60., .end_hold_s = .3,
                           .max_launch_s = 6., .handover_s = .9};
    config.front_lift_reaction = {.hand_window_s = .5, .hand_lag_s = .5, .hand_drop_ratio = .7,
                                  .pitch_rate_up_rad_s = .5,
                                  .pitch_rad = .17, .decision_s = .1, .ease_enabled = false,
                                  .lean_enabled = false, .rear_brake_tap_enabled = false,
                                  .ease_fraction = .7, .ease_rate_per_s = 5., .lean_add_rad = .1,
                                  .lean_rate_rad_s = .5, .clear_s = .3};
    config.drive.assist.release_hold_s = .3;
    config.drive.pedaling.stop_time_s = .35;
    config.drive.pedaling.reposition_on_stall = false;
    config.drive.pedaling.reposition_min_effort_nm = 20.;
    config.drive.pedaling.reposition_stall_window_s = 1.;
    config.drive.pedaling.reposition_stall_progress_rad = .5;
    config.drive.pedaling.reposition_cooldown_s = 1.;
    config.drive.pedaling.reposition_back_rate_rad_s = 3.;
    config.drive.pedaling.reposition_timeout_s = 3.;
    config.drive.pedaling.reposition_phase_tolerance_rad = .05;
    config.drive.pedaling.reposition_noop_rad = .12;
    config.drive.motor_clutch = false;
    config.drive.shifting.shift_cooldown_s = .4; config.drive.shifting.shift_cut_duration_s = .2;
    config.articulated.balance_dwell_s = .5; config.articulated.balance_grace_s = 3.; config.articulated.joint_kp_nm_rad = 600.; config.articulated.joint_kd_nms_rad = 15.; config.articulated.leg_active_damping_nms_rad = 35.;
    config.equilibrium_crank_hold_kp_nm_rad = 20000.; config.equilibrium_crank_hold_kd_nms_rad = 88.;
    config.research.control_period_s = .01; config.research.actuator_delay_s = .005;
    config.research.sensor_latency_s = .01; config.research.sensor_maximum_age_s = .1;
    config.research.sample_period_s = .005; config.research.record_decimation = 80;
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
// cadence is observed through the applied command count), the tick-held
// ctrl channels bitwise on non-tick steps while the every-step rider
// channels update between ticks, a solved-effort channel row per
// physics step, and reset returning the runtime to step 0.
[[gnu::noinline]] void
test_runtime_holds_ctrl_but_observes_solved_effort_each_step() {
    bt::record_initial_state(
        bt::oracle::root() / bt::oracle::platform() / "control_clock" /
        "env_request" / "request.toml");
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
    // PhysicalStep clears d->ctrl every step and every writer
    // republishes its channels, but the ctrl values still split by
    // update cadence: the brake and drive channels track the held
    // RideControl demands (writers/brake.cpp apply, writers/
    // drivetrain.cpp commit) and the rider entries whose held command
    // stays inside the per-step clip reproduce the tick value
    // verbatim, so on a non-tick step those channels are
    // byte-identical to the previous step's. Since 7019cd11 (leg
    // controller every step) and 8e8b0b0d (rider intent every step)
    // the rider write re-derives clip(held, directional_limit(qvel))
    // each step (runtime/step.cpp write site -> rider/
    // spindle_controller.cpp); the rider channels whose cap binds —
    // act_rider_elbow_left/right and act_rider_hip_rear — update on
    // non-tick steps instead of holding.
    const auto ctrl = episode.ctrl_written.doubles(true);
    const std::size_t nu = episode.ctrl_written.shape.at(1);
    const bike::model::ModelOwner model{
        engine::load_model_image(env->model_image().mjb)};
    bt::require(model->nu == static_cast<int>(nu),
                "ctrl row width is the model actuator count");
    constexpr std::array<const char *, 3> every_step_actuators{
        "act_rider_elbow_left", "act_rider_elbow_right",
        "act_rider_hip_rear"};
    std::array<std::size_t, every_step_actuators.size()> every_step{};
    for (std::size_t k = 0; k < every_step_actuators.size(); ++k) {
        const int id =
            mj_name2id(model.get(), mjOBJ_ACTUATOR, every_step_actuators[k]);
        bt::require(id >= 0,
                    "every-step rider actuator present in the ctrl row");
        every_step[k] = static_cast<std::size_t>(id);
    }
    const auto is_every_step = [&](const std::size_t channel) {
        return std::ranges::find(every_step, channel) != every_step.end();
    };
    for (std::size_t i = 1; i < episode.controls.size(); ++i) {
        if (i % 10 == 0) continue;
        const auto now = std::span(ctrl).subspan(i * nu, nu);
        const auto prev = std::span(ctrl).subspan((i - 1) * nu, nu);
        for (std::size_t channel = 0; channel < nu; ++channel) {
            if (is_every_step(channel)) continue;
            bt::require(std::ranges::equal(
                            std::as_bytes(now.subspan(channel, 1)),
                            std::as_bytes(prev.subspan(channel, 1))),
                        "ctrl held byte-identical on a non-tick step");
        }
    }
    const auto bits = [](const double value) {
        return std::bit_cast<std::uint64_t>(value);
    };
    for (const std::size_t channel : every_step) {
        bool updated = false;
        for (std::size_t i = 1; i < episode.controls.size(); ++i) {
            if (i % 10 == 0) continue;
            updated = updated || bits(ctrl[i * nu + channel]) !=
                                    bits(ctrl[(i - 1) * nu + channel]);
        }
        bt::require(updated,
                    "every-step rider ctrl channel updates on a "
                    "non-tick step");
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
