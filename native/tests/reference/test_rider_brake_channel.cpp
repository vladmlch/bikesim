// test_rider_brake_channel.cpp — launch part 1 task 4, the rider-owned
// brake channel. Part 1 produces the empty command set in production, so
// the test seam is bike::validation::RuntimeRigAccess::set_rider_commands — the
// injected commands and the (equally) resolved empty set feed one
// BrakeResolution per physical step. The four predicates stay the single
// old boolean when nothing is commanded and diverge exactly on rider
// brakes: legs_coast keeps the caller-only semantics, motor_braking /
// shift_blocked / riding see the merged rider demand. The motor
// assertions inspect AssistController state, not only torque — a partial
// brake + instant release is invisible in torque but the reset snapshot
// and the first released tick's fresh-controller equality are not.
#include "support/fixture_test.hpp"
#include "launch_ride_support.hpp"
#include "configuration/load.hpp"
#include "drivetrain/motor.hpp"
#include "model/rider_geometry.hpp"
#include "research/environment.hpp"
#include "rider/intent.hpp"
#include "runtime/research.hpp"
#include "runtime/research_io.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "runtime/step.hpp"
#include "stepper.hpp"
#include "terrain/trackfile.hpp"
#include "validation/runtime_access.hpp"
#include "writers/drivetrain.hpp"
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace {
namespace sw = runtime::sample_wire;
namespace terrain = bike::terrain;
namespace research = bike::research;

[[nodiscard]] const runtime::WireObject &channel_of(const runtime::SamplePtr &sample,
                                                    const char *name) {
    return sw::object(sw::required(sample->channels, name));
}

[[nodiscard]] double channel_number(const runtime::SamplePtr &sample,
                                    const char *channel, const char *key) {
    return sw::number(sw::required(channel_of(sample, channel), key));
}

[[nodiscard]] bool channel_boolean(const runtime::SamplePtr &sample,
                                   const char *channel, const char *key) {
    return sw::boolean(sw::required(channel_of(sample, channel), key));
}

[[nodiscard]] std::string channel_string(const runtime::SamplePtr &sample,
                                         const char *channel, const char *key) {
    return sw::string(sw::required(channel_of(sample, channel), key));
}

[[nodiscard]] double root_x(launch_ride::PedelecEnv &env) {
    // Before the first committed interval latest_sample is null — read
    // the live qpos through the rig instead.
    const auto &sample = env.runtime->snapshot().latest_sample;
    if (sample)
        return sample->qpos[static_cast<std::size_t>(env.root_x_qpos)];
    double x = 0.;
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &s, const runtime::PhysicalStep &) {
            x = model_access::readonly_buffer(s.data()->qpos, s.model()->nq)
                    [static_cast<std::size_t>(env.root_x_qpos)];
        });
    return x;
}

void ride_intervals(launch_ride::PedelecEnv &env, std::int64_t count,
                    const runtime::RideControl &control, double front = 0.,
                    double rear = 0.) {
    const auto committed = env.runtime->snapshot().step;
    const auto result = env.runtime->advance(committed + count, control,
                                             front, rear, std::nullopt);
    bt::require(!result.outcome, "ride did not crash: " +
                                 (result.outcome ? *result.outcome : ""));
}

// Full-object comparison for the P4 evidence seam: every declared leaf of
// the rider_commands record compares field-for-field across modes.
void require_same_commands(const runtime::WireObject &actual,
                           const char *what,
                           const runtime::WireObject &expected) {
    for (const auto &[key, expected_wire] : expected) {
        const auto *seen = sw::find(actual, key);
        bt::require(seen != nullptr, std::string(what) + ": key " + key);
        const auto &e = expected_wire.value;
        if (const auto *b = std::get_if<bool>(&e))
            bt::require(sw::boolean(*seen) == *b,
                        std::string(what) + ": flag " + key);
        else if (const auto *d = std::get_if<double>(&e))
            bt::require(sw::number(*seen) == *d,
                        std::string(what) + ": demand " + key);
        else
            bt::require(sw::string(*seen) == std::get<std::string>(e),
                        std::string(what) + ": label " + key);
    }
}

// Replays the recorded rider_commands record through a fresh controller
// exactly once — the first released tick equals a controller that never
// saw braking stepping on the same inputs.
void test_validate_rejects_out_of_range() {
    rider::validate(rider::RiderCommands{});
    rider::validate(rider::RiderCommands{.front_brake = 1., .rear_brake = .5,
        .shift_request = std::nullopt});
    bt::invalid([&] {
        rider::validate(rider::RiderCommands{.front_brake = 1.5,
            .shift_request = std::nullopt});
    });
    bt::invalid([&] {
        rider::validate(rider::RiderCommands{.rear_brake = -.1,
            .shift_request = std::nullopt});
    });
    bt::invalid([&] {
        rider::validate(rider::RiderCommands{
            .front_brake = std::numeric_limits<double>::quiet_NaN(),
            .shift_request = std::nullopt});
    });
    bt::invalid([&] {
        rider::validate(rider::RiderCommands{
            .rear_brake = std::numeric_limits<double>::infinity(),
            .shift_request = std::nullopt});
    });
    // Launch part 1, task 5: a shift request without ownership is an
    // inconsistent command set — the request would be silently dropped.
    rider::validate(rider::RiderCommands{
        .shift_request = rider::ShiftRequest::up, .owns_shifting = true});
    bt::invalid([&] {
        rider::validate(rider::RiderCommands{
            .shift_request = rider::ShiftRequest::down});
    });
}

void test_rider_brake_holds_while_legs_press() {
    const auto track = terrain::load_track(launch_ride::repo_file(
        "tests/fixtures/tracks/climb_15pct_standstill.toml"));
    auto env = launch_ride::environment(track, 2.);
    bike::validation::RuntimeRigAccess::set_rider_commands(
        *env.runtime, rider::RiderCommands{.front_brake = 1., .rear_brake = 1.,
            .shift_request = std::nullopt});
    runtime::RideControl control;
    control.human_torque_nm = 30.;
    const double start_x = root_x(env);
    ride_intervals(env, 400, control); // 0.5 s at dt = .00125 s
    const auto &samples = env.runtime->recorded_intervals();
    bt::require(!samples.empty(), "braked ride records intervals");
    for (const auto &sample : samples) {
        bt::require(channel_number(sample, "drive", "motor_torque_nm") == 0.,
                    "rider braking delivers no motor torque");
        bt::require(channel_boolean(sample, "drive", "motor_braking"),
                    "the merged braking predicate reaches the drive lane");
        bt::require(channel_number(sample, "drive", "rider_front_brake_demand") == 1. &&
                        channel_number(sample, "drive", "rider_rear_brake_demand") == 1.,
                    "the rider brake demands ride the drive lane");
        bt::require(channel_number(sample, "drive", "front_brake_demand") == 1. &&
                        channel_number(sample, "drive", "rear_brake_demand") == 1.,
                    "the merged demands reach the effective brake fields");
        bt::require(channel_number(sample, "drive", "human_sensor_nm") > 0.,
                    "the rider's legs keep pressing while the rider brakes");
        bt::require(channel_string(sample, "drive", "rider_mode") == "pedaling",
                    "rider braking never coasts the rider's own legs");
        const auto &commands = channel_of(sample, "rider_commands");
        bt::require(sw::number(sw::required(commands, "front_brake")) == 1. &&
                        sw::number(sw::required(commands, "rear_brake")) == 1.,
                    "the commanded brakes ride the rider_commands channel");
        bt::require(sw::string(sw::required(commands, "shift_request")) == "none",
                    "no shift request was commanded");
        bt::require(!sw::boolean(sw::required(commands, "owns_shifting")),
                    "the rider does not own shifting in this test");
        bt::require(sw::string(sw::required(commands, "source")) == "test_override",
                    "the override marks its provenance");
        bt::require(!sw::boolean(sw::required(commands, "legs_coast")),
                    "rider-owned braking keeps the legs pressing");
        bt::require(sw::boolean(sw::required(commands, "motor_braking")),
                    "rider-owned braking declares motor braking");
        bt::require(sw::boolean(sw::required(commands, "shift_blocked")),
                    "rider-owned braking blocks shift admission");
        bt::require(!sw::boolean(sw::required(commands, "riding")),
                    "full rider braking suppresses the riding predicate");
    }
    bt::require(std::abs(root_x(env) - start_x) < .01,
                "the braked bike holds within a centimetre");
}

// The assist-controller assertions go through inspect(): braking resets
// the committed snapshot wholesale and the first released tick equals a
// never-braked controller stepping once on the same interval inputs.
void test_motor_state_resets_on_rider_brake() {
    const auto track = terrain::load_track(launch_ride::repo_file(
        "tests/fixtures/tracks/climb_15pct_standstill.toml"));
    auto env = launch_ride::environment(track, 2.);
    runtime::RideControl control;
    control.human_torque_nm = 30.;
    ride_intervals(env, 400, control);
    drivetrain::AssistSnapshot seeded;
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &s, const runtime::PhysicalStep &) {
            seeded = s.drive().state().assist;
        });
    bt::require(seeded.torque > 0. && seeded.pedaling,
                "unbraked pedaling seeds a nonzero assist state");
    bike::validation::RuntimeRigAccess::set_rider_commands(
        *env.runtime, rider::RiderCommands{.front_brake = 1., .rear_brake = 1.,
            .shift_request = std::nullopt});
    ride_intervals(env, 32, control);
    drivetrain::AssistSnapshot braked;
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &s, const runtime::PhysicalStep &) {
            braked = s.drive().state().assist;
        });
    bt::require(braked.torque == 0. && braked.last_gain == 0. &&
                    braked.hold_power_w == 0. && braked.hold_torque_nm == 0. &&
                    braked.release_hold_left_s == 0. && !braked.pedaling,
                "rider braking resets the full assist snapshot, run-on included");
    bike::validation::RuntimeRigAccess::set_rider_commands(*env.runtime, std::nullopt);
    // The first released tick, recomputed by a controller that never saw
    // braking and stepped once on that step's own recorded inputs
    // (record_decimation = 1 — one interval is one physical step).
    ride_intervals(env, 1, control);
    const auto &released = env.runtime->recorded_intervals().back();
    const double speed_mps =
        released->qvel[static_cast<std::size_t>(env.root_x_dof)];
    const double assist_sensor =
        channel_number(released, "drive", "assist_sensor_nm");
    const double cadence_rpm =
        channel_number(released, "drive", "cadence_rpm");
    const double shaft_rad_s =
        channel_number(released, "drive", "drive_shaft_rad_s");
    drivetrain::AssistSnapshot rebuilt;
    double fresh_delivered = 0.;
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &s, const runtime::PhysicalStep &) {
            drivetrain::AssistController fresh(s.drive().config().policies.assist);
            fresh_delivered =
                fresh.step(assist_sensor, cadence_rpm, speed_mps, false,
                           .00125, std::nullopt, shaft_rad_s * 60. / (2. * M_PI));
            rebuilt = fresh.state();
        });
    drivetrain::AssistSnapshot committed;
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &s, const runtime::PhysicalStep &) {
            committed = s.drive().state().assist;
        });
    bt::require(committed.torque == rebuilt.torque &&
                    committed.last_gain == rebuilt.last_gain &&
                    committed.hold_power_w == rebuilt.hold_power_w &&
                    committed.hold_torque_nm == rebuilt.hold_torque_nm &&
                    committed.release_hold_left_s == rebuilt.release_hold_left_s &&
                    committed.pedaling == rebuilt.pedaling,
                "the released tick equals a fresh controller's first step — "
                "no stored pre-brake state leaks through");
    bt::require(channel_number(released, "drive", "motor_torque_nm") ==
                    fresh_delivered,
                "the delivered torque equals the fresh controller's request");
    ride_intervals(env, 320, control);
    bt::require(channel_number(env.runtime->recorded_intervals().back(),
                               "drive", "motor_torque_nm") > 0.,
                "the assist ramp recovers after the brake is released");
}

void test_caller_braking_still_coasts_the_legs() {
    const auto track = terrain::load_track(launch_ride::repo_file(
        "tests/fixtures/tracks/climb_15pct_standstill.toml"));
    auto env = launch_ride::environment(track, 2.);
    runtime::RideControl control;
    control.human_torque_nm = 30.;
    ride_intervals(env, 32, control, 1., 1.);
    const auto &sample = env.runtime->recorded_intervals().back();
    // The ~3.7 Nm sensor floor is the weld's return-foot preload — zero
    // commanded effort plus the coasting mode are the leg-coast signals.
    bt::require(channel_number(sample, "drive", "human_command_nm") == 0.,
                "caller braking zeroes the commanded effort as before");
    bt::require(channel_number(sample, "drive", "human_sensor_nm") < 10.,
                "caller braking drops the sensed torque to the weld floor");
    bt::require(channel_string(sample, "drive", "coasting_reason") == "braking",
                "caller braking keeps reporting its coasting reason");
    bt::require(channel_string(sample, "drive", "rider_mode") == "coasting",
                "caller braking still flips the rider mode to coasting");
    const auto &commands = channel_of(sample, "rider_commands");
    bt::require(sw::boolean(sw::required(commands, "legs_coast")),
                "caller braking keeps legs_coast set");
    bt::require(sw::boolean(sw::required(commands, "motor_braking")) &&
                    sw::boolean(sw::required(commands, "shift_blocked")),
                "caller braking keeps blocking the motor and shifting");
    bt::require(sw::string(sw::required(commands, "source")) == "intent",
                "with no override the commands come from the intent");
}

void test_flags_match_old_semantics_without_commands() {
    const auto track = terrain::load_track(launch_ride::repo_file(
        "tests/fixtures/tracks/climb_15pct_standstill.toml"));
    // Pre-launch-plan fallback semantics: run with the launch plan off so the
    // intent does not hold brakes or own shifting on the 15 % standstill start.
    auto env = launch_ride::environment(
        track, 2.,
        [](auto &physics) { physics.rider_launch.plan_enabled = false; }, 1,
        runtime::ResearchCaptureMode::legacy);
    runtime::RideControl control;
    control.human_torque_nm = 30.;
    ride_intervals(env, 160, control);          // unbraked press
    ride_intervals(env, 160, control, .8, .6);  // caller braking
    ride_intervals(env, 160, control);          // released
    const auto &samples = env.runtime->recorded_intervals();
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const auto &sample = samples[i];
        const bool caller_brake = i >= 160 && i < 320;
        const auto &commands = channel_of(sample, "rider_commands");
        bt::require(sw::number(sw::required(commands, "front_brake")) == 0. &&
                        sw::number(sw::required(commands, "rear_brake")) == 0. &&
                        sw::string(sw::required(commands, "shift_request")) == "none" &&
                        !sw::boolean(sw::required(commands, "owns_shifting")),
                    "the default command set stays empty");
        bt::require(sw::string(sw::required(commands, "source")) == "intent",
                    "production commands come from the intent");
        bt::require(sw::boolean(sw::required(commands, "legs_coast")) == caller_brake &&
                        sw::boolean(sw::required(commands, "motor_braking")) == caller_brake &&
                        sw::boolean(sw::required(commands, "shift_blocked")) == caller_brake,
                    "every resolved flag equals the single old predicate");
        bt::require(channel_number(sample, "drive", "rider_front_brake_demand") == 0. &&
                        channel_number(sample, "drive", "rider_rear_brake_demand") == 0.,
                    "no rider demand is emitted without commands");
        bt::require(channel_boolean(sample, "drive", "motor_braking") == caller_brake,
                    "motor_braking equals the caller predicate by default");
    }
    const std::vector<double> first = samples.back()->qpos;
    env.runtime->reset();
    control.human_torque_nm = 30.;
    ride_intervals(env, 160, control);
    ride_intervals(env, 160, control, .8, .6);
    ride_intervals(env, 160, control);
    bt::require(env.runtime->recorded_intervals().back()->qpos == first,
                "the empty-command ride replays bitwise identically");
}

void test_partial_rider_brake_flags() {
    const auto track = terrain::load_track(launch_ride::repo_file(
        "tests/fixtures/tracks/climb_15pct_standstill.toml"));
    auto env = launch_ride::environment(track, 2.);
    bike::validation::RuntimeRigAccess::set_rider_commands(
        *env.runtime, rider::RiderCommands{.front_brake = .5, .rear_brake = .5,
            .shift_request = std::nullopt});
    runtime::RideControl control;
    control.human_torque_nm = 30.;
    ride_intervals(env, 200, control);
    const auto &sample = env.runtime->recorded_intervals().back();
    const auto &commands = channel_of(sample, "rider_commands");
    bt::require(!sw::boolean(sw::required(commands, "legs_coast")),
                "partial rider braking is still rider-owned for the legs");
    bt::require(sw::boolean(sw::required(commands, "motor_braking")) &&
                    sw::boolean(sw::required(commands, "shift_blocked")),
                "a nonzero rider brake blocks the motor and shifting");
    bt::require(!sw::boolean(sw::required(commands, "riding")),
                "a partial rider brake is not balance riding");
    bt::require(channel_number(sample, "drive", "human_sensor_nm") > 20. &&
                    channel_number(sample, "drive", "motor_torque_nm") == 0.,
                "legs press, motor off, under a partial rider brake");
    bt::require(channel_number(sample, "drive", "front_brake_demand") == .5 &&
                    channel_number(sample, "drive", "rider_front_brake_demand") == .5,
                "the partial demand merges into the effective lane");
}

void test_rider_and_caller_brakes_overlap() {
    const auto track = terrain::load_track(launch_ride::repo_file(
        "tests/fixtures/tracks/climb_15pct_standstill.toml"));
    auto env = launch_ride::environment(track, 2.);
    bike::validation::RuntimeRigAccess::set_rider_commands(
        *env.runtime, rider::RiderCommands{.rear_brake = 1.,
            .shift_request = std::nullopt});
    runtime::RideControl control;
    control.human_torque_nm = 30.;
    ride_intervals(env, 32, control, 1., 0.);
    const auto &sample = env.runtime->recorded_intervals().back();
    const auto &commands = channel_of(sample, "rider_commands");
    bt::require(sw::boolean(sw::required(commands, "legs_coast")) &&
                    sw::boolean(sw::required(commands, "motor_braking")),
                "the caller brake still coasts the legs; the merge still "
                "blocks the motor");
    bt::require(channel_number(sample, "drive", "front_brake_demand") == 1. &&
                    channel_number(sample, "drive", "rear_brake_demand") == 1.,
                "each wheel takes the max of caller and rider demand");
    bt::require(channel_number(sample, "drive", "human_command_nm") == 0. &&
                    channel_number(sample, "drive", "human_sensor_nm") < 10.,
                "the caller brake still drops the effort to the weld floor");
}

void test_rollback_does_not_set_motor_braking() {
    const auto track = terrain::load_track(launch_ride::repo_file(
        "tests/fixtures/tracks/climb_15pct_standstill.toml"));
    auto env = launch_ride::environment(track, 2.);
    runtime::RideControl control;
    // Explicit zero effort keeps the rider enabled (the rollback reflex
    // is gated on it) while no leg force reaches the cranks — the bike
    // rolls back and the hold engages.
    control.human_torque_nm = 0.;
    ride_intervals(env, 800, control);
    // The hold engages below -0.25 m/s and releases at standstill, so it
    // oscillates — assert on every engaged tick, not only the last one.
    bool saw_hold = false;
    for (const auto &sample : env.runtime->recorded_intervals()) {
        if (channel_number(sample, "drive", "rollback_brake_demand") <= 0.)
            continue;
        saw_hold = true;
        bt::require(!channel_boolean(sample, "drive", "motor_braking"),
                    "rollback hold alone does not declare motor braking");
        bt::require(!sw::boolean(sw::required(
                        channel_of(sample, "rider_commands"), "motor_braking")),
                    "rollback hold alone never marks the rider channel");
    }
    bt::require(saw_hold, "the standstill rollback hold engaged");
    bike::validation::RuntimeRigAccess::set_rider_commands(
        *env.runtime, rider::RiderCommands{.front_brake = 1.,
            .shift_request = std::nullopt});
    ride_intervals(env, 16, control);
    const auto &ridden = env.runtime->recorded_intervals().back();
    bt::require(channel_boolean(ridden, "drive", "motor_braking"),
                "a rider brake on top of rollback declares motor braking");
}

// The research twin of launch_ride::environment — identical request, but
// the whole ResearchBootstrap goes to make_research so the command journal
// and the recorded channel are observable in the second runtime mode.
struct ResearchEnv {
    std::unique_ptr<runtime::NativeResearchRuntime> runtime;
    std::int64_t control_steps = 0;
};

[[nodiscard]] ResearchEnv research_environment(const terrain::TrackSpec &track,
                                               double duration_s) {
    auto owner = std::make_unique<research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = bike::configuration::load_physics(
        launch_ride::repo_file("examples/research/viewer_physics_welded.toml"));
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.rider =
        bike::model::rider_from_args("articulated_planar");
    request.setup.track = track;
    request.setup.model.physics->initial_speed_mps = 0.;
    request.experiment.duration_s = duration_s;
    request.experiment.record_decimation = 1;
    request.experiment.maximum_energy_residual_ratio = 1e9;
    request.experiment.stop_on_model_violation = false;
    request.sensors = research::SensorConfig{};
    request.experiment.control_period_s =
        request.setup.model.physics->research.control_period_s;
    request.experiment.actuator_delay_s =
        request.setup.model.physics->research.actuator_delay_s;
    request.sensors.delivery.latency_s =
        request.setup.model.physics->research.sensor_latency_s;
    request.sensors.delivery.maximum_age_s =
        request.setup.model.physics->research.sensor_maximum_age_s;
    request.sensors.sample_period_s = .005;
    // Full capture: recorded_intervals() then carries the channel
    // records the parity loop compares field-for-field.
    request.capture_mode = runtime::ResearchCaptureMode::full;
    const auto prepared = research::prepare_environment(std::move(owner));
    ResearchEnv env;
    env.control_steps = prepared->bootstrap.config.control_steps;
    env.runtime = runtime::make_research(std::move(prepared->bootstrap));
    return env;
}

void test_research_mode_records_the_same_journal() {
    const auto track = terrain::load_track(launch_ride::repo_file(
        "tests/fixtures/tracks/climb_15pct_standstill.toml"));
    auto env = launch_ride::environment(track, 2.);
    bike::validation::RuntimeRigAccess::set_rider_commands(
        *env.runtime, rider::RiderCommands{.front_brake = .5, .rear_brake = .5,
            .shift_request = std::nullopt});
    auto research_env = research_environment(track, 2.);
    bike::validation::RuntimeRigAccess::set_rider_commands(
        *research_env.runtime,
        rider::RiderCommands{.front_brake = .5, .rear_brake = .5,
            .shift_request = std::nullopt});

    runtime::RideControl control;
    control.human_torque_nm = 30.;
    const std::int64_t phase_steps = env.control_steps * 10; // ten windows
    ride_intervals(env, phase_steps, control);
    ride_intervals(env, phase_steps, control, .7, .7);
    ride_intervals(env, phase_steps, control);
    const auto windows = [&](double front, double rear) {
        for (int i = 0; i < 10; ++i) {
            research_env.runtime->begin_control(control, front, rear);
            bt::require(
                research_env.runtime->advance_control(std::nullopt)
                    .has_value(),
                "research window completes");
            research_env.runtime->acknowledge_control();
        }
    };
    windows(0., 0.);
    windows(.7, .7);
    windows(0., 0.);

    const auto page = research_env.runtime->prepare_rider_commands_since(0);
    const auto &rows = sw::array(sw::required(page, "rows"));
    bt::require(!rows.empty(), "the journal records the commanded channel");
    const auto &samples = env.runtime->recorded_intervals();
    // Journal rows are deduplicated command records {time_s, step,
    // commands}; each must equal the ride recording's channel at its step
    // and the coverage must be gap-free.
    std::int64_t covered = -1;
    for (const auto &row_wire : rows) {
        const auto &row = sw::object(row_wire);
        const auto step = sw::integer(sw::required(row, "step"));
        bt::require(step > covered, "journal rows advance monotonically");
        covered = step;
        const auto &expected =
            channel_of(samples[static_cast<std::size_t>(step)], "rider_commands");
        require_same_commands(sw::object(sw::required(row, "commands")),
                              "journal row vs ride channel", expected);
    }
    // The research recording carries the same channel shape.
    const auto intervals = research_env.runtime->recorded_intervals();
    bt::require(intervals.size() == samples.size(),
                "both modes recorded the same number of intervals");
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const auto &row = sw::object(intervals[i]);
        // PhysicalSampleData::as_wire() merges the channels at top level.
        require_same_commands(
            sw::object(sw::required(row, "rider_commands")),
            "research channel vs ride channel",
            channel_of(samples[i], "rider_commands"));
    }
}

void test_set_rider_commands_validates() {
    const auto track = terrain::load_track(launch_ride::repo_file(
        "tests/fixtures/tracks/climb_15pct_standstill.toml"));
    auto env = launch_ride::environment(track, 2.);
    bt::invalid([&] {
        bike::validation::RuntimeRigAccess::set_rider_commands(
            *env.runtime, rider::RiderCommands{.front_brake = 2.,
                .shift_request = std::nullopt});
    });
    bike::validation::RuntimeRigAccess::set_rider_commands(*env.runtime, std::nullopt);
}

// Launch part 1, task 5: the production transport — a commanded request
// reaches the drivetrain's cadence shifter and the closed rider_response
// label is recorded on the drive channel. Three deterministic phases:
// the cassette end rejects, the resolved shift_blocked defers, and a
// flat-track run finds a real in-band landing to accept.
void test_rider_shift_request_flows_to_the_shifter() {
    // One flat environment serves all three outcomes: at low speed there
    // is no rollback latch, so the only gate phase 2 trips is the
    // commanded rider brake itself. The welded gear starts at 51T.
    const auto track = terrain::load_track(launch_ride::repo_file(
        "tests/fixtures/tracks/flat.toml"));
    // Launch part 4 moved the welded profile's flat start cog to 24T and
    // enabled the launch plan; keep this transport test on its original
    // 51T plan-off baseline so phase 1 still tests 'down' past 51T.
    auto env = launch_ride::environment(
        track, 12.,
        [](auto &physics) {
            physics.rider_launch.plan_enabled = false;
            physics.rider_launch.start_cog_grade_breaks.clear();
            physics.rider_launch.start_cog_teeth = {51};
        },
        1, runtime::ResearchCaptureMode::legacy);
    runtime::RideControl control;
    control.human_torque_nm = 30.;
    ride_intervals(env, 8, control);   // 10 ms — the gear is still 51T.
    const auto cassette = std::vector<int>{10, 12, 14, 16, 18, 21, 24, 28,
                                           33, 39, 45, 51};
    const auto response_of = [&](const runtime::SamplePtr &sample) {
        return channel_string(sample, "drive", "shift_rider_response");
    };
    // Uncommanded intervals carry the reset label.
    bt::require(response_of(env.runtime->recorded_intervals().back()) ==
                    "none",
                "the recorded response defaults to none");
    // 1) Rejected at the cassette end: 'down' asks past 51T.
    bike::validation::RuntimeRigAccess::set_rider_commands(
        *env.runtime, rider::RiderCommands{
            .shift_request = rider::ShiftRequest::down,
            .owns_shifting = true});
    ride_intervals(env, 1, control);
    const auto &rejected = env.runtime->recorded_intervals().back();
    bt::require(response_of(rejected) == "rejected",
                "the cassette end rejects the request");
    bt::require(channel_number(rejected, "drive", "gear_rear_teeth") == 51.,
                "the rejected request keeps the cog");
    const auto &asked = channel_of(rejected, "rider_commands");
    bt::require(sw::string(sw::required(asked, "shift_request")) == "down" &&
                    sw::boolean(sw::required(asked, "owns_shifting")),
                "the commanded request rides the rider_commands channel");
    // 2) Deferred through the resolved shift_blocked: a rider brake on
    //    the same step wins the shared admission gate.
    bike::validation::RuntimeRigAccess::set_rider_commands(
        *env.runtime, rider::RiderCommands{.front_brake = 1., .rear_brake = 1.,
            .shift_request = rider::ShiftRequest::up, .owns_shifting = true});
    ride_intervals(env, 1, control);
    const auto &deferred = env.runtime->recorded_intervals().back();
    bt::require(response_of(deferred) == "deferred",
                "rider braking defers through shift_blocked");
    bt::require(channel_number(deferred, "drive", "gear_rear_teeth") == 51.,
                "the deferred request keeps the cog");
    bt::require(sw::boolean(sw::required(
                    channel_of(deferred, "rider_commands"), "shift_blocked")),
                "the resolved predicate marks the deferral's cause");
    // 3) Accepted: the same flat run accelerates into a window whose
    //    landing sits inside the band; the persistent command
    //    re-evaluates every tick — a deferred request keeps waiting
    //    while the ride continues, a rejected one (the automaton raced
    //    the request or the landing drifted out) is cleared and the
    //    poll resumes. The phase-2 command set is latched state — clear
    //    it or the brakes hold the bike at standstill forever.
    bike::validation::RuntimeRigAccess::set_rider_commands(
        *env.runtime, rider::RiderCommands{});
    control.human_torque_nm = 80.;
    bool latched = false, accepted = false;
    int asked_smaller = 0;
    for (int round = 0; round < 4800 && !accepted; ++round) {
        ride_intervals(env, 1, control);
        const auto &sample = env.runtime->recorded_intervals().back();
        const auto response = response_of(sample);
        if (latched) {
            if (response == "accepted") {
                bt::require(channel_number(sample, "drive",
                                           "gear_rear_teeth") ==
                                static_cast<double>(asked_smaller),
                            "the accepted request moves exactly one cog smaller");
                bt::require(channel_number(sample, "drive",
                                           "shift_torque_factor") < 1.,
                            "the accepted shift arms the torque cut");
                accepted = true;
                break;
            }
            if (response == "rejected") {
                bike::validation::RuntimeRigAccess::set_rider_commands(
                    *env.runtime, rider::RiderCommands{});
                latched = false;
            }
            continue;
        }
        const double required =
            channel_number(sample, "drive", "required_cadence_rpm");
        const int teeth = static_cast<int>(
            channel_number(sample, "drive", "gear_rear_teeth"));
        const auto at = std::ranges::find(cassette, teeth);
        bt::require(at != cassette.end(), "the recorded cog is on the cassette");
        // up = one cog smaller: landing = required * smaller/current must
        // sit strictly inside [75, 110] (margin against the EMA lag and
        // the one-tick delay between the read sample and the request).
        if (at == cassette.begin())
            continue;
        const int smaller = *std::prev(at);
        const double landing = required * smaller / teeth;
        if (landing < 80. || landing > 105.)
            continue;
        bike::validation::RuntimeRigAccess::set_rider_commands(
            *env.runtime, rider::RiderCommands{
                .shift_request = rider::ShiftRequest::up,
                .owns_shifting = true});
        asked_smaller = smaller;
        latched = true;
    }
    bt::require(accepted,
                "the flat run reached an admissible in-band window");
    // Ownership without a request suppresses the band automaton and
    // reports none.
    bike::validation::RuntimeRigAccess::set_rider_commands(
        *env.runtime, rider::RiderCommands{.owns_shifting = true});
    const auto count_at_hold = channel_number(
        env.runtime->recorded_intervals().back(), "drive", "shift_count");
    ride_intervals(env, 400, control);
    const auto &held = env.runtime->recorded_intervals().back();
    bt::require(channel_number(held, "drive", "shift_count") == count_at_hold,
                "the band never selects while the rider owns");
    bt::require(response_of(held) == "none",
                "ownership without a request reports none");
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        test_validate_rejects_out_of_range();
        test_rider_brake_holds_while_legs_press();
        test_motor_state_resets_on_rider_brake();
        test_caller_braking_still_coasts_the_legs();
        test_flags_match_old_semantics_without_commands();
        test_partial_rider_brake_flags();
        test_rider_and_caller_brakes_overlap();
        test_rollback_does_not_set_motor_braking();
        test_research_mode_records_the_same_journal();
        test_set_rider_commands_validates();
        test_rider_shift_request_flows_to_the_shifter();
    });
}
