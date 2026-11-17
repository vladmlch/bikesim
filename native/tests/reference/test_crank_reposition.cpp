// 2026-11-16 plan task 7 — the crank-reposition plant check (S3 + R3/R5)
// and the request's one-shot contract: a
// NativeRideRuntime::request_crank_reposition() reaches the pedaling
// policy through the writer-level RideControl, backpedals the
// spindle-welded crank to the nearest lower power phase while the motor
// clutch and rear freehub keep the wheel and drive shaft from being
// dragged backward, and the pedal welds stay inside their safe envelope.
//
// The environment follows the weld_env pattern of
// integration/test_rider_welds.cpp:65-94 — the live welded example on a
// 30 m flat track, initial speed 0, diagnostic limits — with the start
// pedal angle moved to a dead spot. The plan suggests mutating
// physics->drive.crank_phase_rad, but initialization::prepare_request
// overwrites that field from rider_launch.start_pedal_angle_rad
// (launch_preparation.cpp:20, launch_config.cpp:135), so the dead spot
// is authored on the launch table — the same value the settle then holds
// the crank at through equilibrium_crank_hold_*.
#include "support/fixture_test.hpp"
#include "support/golden.hpp"
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "drivetrain/pedaling.hpp"
#include "rider/spindle_controller.hpp"
#include "model_access.hpp"
#include "research/environment.hpp"
#include "runtime/control.hpp"
#include "runtime/runtime.hpp"
#include "runtime/step.hpp"
#include "terrain/trackfile.hpp"
#include "validation/runtime_access.hpp"
#include "writers/rider_contacts.hpp"
#include "stepper.hpp"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <variant>

namespace {

namespace research = bike::research;
namespace cfg = bike::configuration;

std::filesystem::path repo_file(std::string_view relative) {
    return bt::oracle::root().parent_path().parent_path().parent_path() /
           std::filesystem::path(relative);
}

// weld_env variant: flat 30 m, standstill, diagnostic limits, crank
// parked at a dead spot (kPowerPhaseOffsetRad + pi/2 + 0.6). The lower
// power phase sits ~pi/2 + 0.6 rad of backpedal away. When given,
// control_period_s receives the physics-level control period — the
// cadence PhysicalStep's ControlClock ticks at — so a test can schedule
// around the advancing control ticks. control_period_steps widens the
// profile's 1-step cadence for tests that need non-tick intervals.
std::unique_ptr<runtime::NativeRideRuntime> reposition_env(
    double *control_period_s = nullptr, int control_period_steps = 0) {
    auto owner = std::make_unique<research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = cfg::load_physics(
        repo_file("examples/research/viewer_physics_welded.toml"));
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.rider =
        bike::model::rider_from_args("articulated_planar");
    if (control_period_steps > 0)
        request.setup.model.physics->control_period_s =
            control_period_steps *
            request.setup.model.physics->timestep_s;
    const std::string track_text =
        "name = \"probe\"\nlength_m = 30.0\nsurface = \"hardpack\"\n"
        "grade_profile = { knots = [[0.0, 0.0], [30.0, 0.0]] }\n";
    request.setup.track = bike::terrain::track_from_dict(
        bike::terrain::parse_track_toml(track_text));
    request.setup.model.physics->initial_speed_mps = 0.;
    request.setup.model.physics->rider_launch.start_pedal_angle_rad =
        drivetrain::kPowerPhaseOffsetRad + std::numbers::pi / 2. + 0.6;
    request.experiment.duration_s = 6.;
    request.experiment.record_decimation = 400;
    request.experiment.maximum_energy_residual_ratio = 1e9;
    request.experiment.stop_on_model_violation = false;
    request.sensors = research::SensorConfig{};
    // [research] is profile-owned now: copy the loaded physics
    // section across instead of relying on struct defaults.
    request.experiment.control_period_s =
        request.setup.model.physics->research.control_period_s;
    if (control_period_s != nullptr)
        *control_period_s = request.setup.model.physics->control_period_s;
    request.experiment.actuator_delay_s =
        request.setup.model.physics->research.actuator_delay_s;
    request.sensors.delivery.latency_s =
        request.setup.model.physics->research.sensor_latency_s;
    request.sensors.delivery.maximum_age_s =
        request.setup.model.physics->research.sensor_maximum_age_s;
    request.sensors.sample_period_s = .005;
    const auto prepared = research::prepare_environment(std::move(owner));
    auto bootstrap = std::make_unique<runtime::RuntimeBootstrap>(
        std::move(prepared->bootstrap.ride));
    return std::make_unique<runtime::NativeRideRuntime>(
        std::move(*bootstrap));
}

struct SpinJoint {
    int qpos = -1, dof = -1;
};

SpinJoint spin_joint(const mjModel *m, const char *name) {
    const int id = mj_name2id(m, mjOBJ_JOINT, name);
    bt::require(id >= 0,
                std::string(name) + " exists in the welded plant");
    return {.qpos = model_access::readonly_buffer(m->jnt_qposadr, m->njnt)
                        [static_cast<std::size_t>(id)],
            .dof = model_access::readonly_buffer(m->jnt_dofadr, m->njnt)
                       [static_cast<std::size_t>(id)]};
}

// Telemetry lane reads — absent lanes and explicit-None values both
// report no sample so a topology that omits them cannot silently satisfy
// the clutch assertions.
std::optional<double> lane_real(const drivetrain::DriveTelemetry &lanes,
                                std::string_view name) {
    if (!lanes.present(name)) return std::nullopt;
    const drivetrain::DiagnosticValue value = lanes.at(name);
    if (const auto *real = std::get_if<double>(&value)) return *real;
    return std::nullopt;
}

std::optional<bool> lane_flag(const drivetrain::DriveTelemetry &lanes,
                            std::string_view name) {
    if (!lanes.present(name)) return std::nullopt;
    const drivetrain::DiagnosticValue value = lanes.at(name);
    if (const auto *flag = std::get_if<bool>(&value)) return *flag;
    return std::nullopt;
}

const writers::RiderSettledEntry *
settled_entry(const writers::RiderSettledWelds &welds,
              std::string_view name) {
    for (const auto &[key, entry] : welds.entries)
        if (key == name) return &entry;
    return nullptr;
}

// test_requested_reposition_reaches_the_lower_power_phase — R3:
// the request engages a reposition goal (reason 'requested'), the crank
// moves only backward and arrives inside the 0.05 rad tolerance well
// before the 3 s timeout.
void test_requested_reposition_reaches_the_lower_power_phase() {
    auto runtime = reposition_env();
    const runtime::RideControl control{.motor_torque_nm = 0.,
                                       .motor_limit_nm = std::nullopt,
                                       .human_torque_nm = 0.,
                                       .crank_target_rate_rad_s =
                                           std::nullopt,
                                       .posture = std::nullopt};
    const auto outcome_free = [&] {
        return !runtime->snapshot().outcome.has_value();
    };
    (void)runtime->advance(40, control, 0., 0., std::nullopt);
    bt::require(runtime->snapshot().step == 40 && outcome_free(),
                "the dead-spot plant reaches the request step");

    double phase_at_request = 0., timestep = 0.;
    SpinJoint crank;
    bike::validation::RuntimeRigAccess::inspect(
        *runtime, [&](const Stepper &stepper, const runtime::PhysicalStep &) {
            crank = spin_joint(stepper.model(), "crank_spin");
            timestep = stepper.model()->opt.timestep;
            phase_at_request = model_access::readonly_buffer(
                stepper.data()->qpos,
                stepper.model()->nq)[static_cast<std::size_t>(crank.qpos)];
        });

    // The one-shot request is consumed by the next advancing control
    // tick; the manoeuvre then backpedals toward the goal.
    runtime->request_crank_reposition();

    // 4 s: a hard bound far past the ~1 s the arrival takes — the loop
    // must not hang if the manoeuvre ever regresses to waiting out the
    // 3 s timeout.
    const int window = static_cast<int>(std::llround(4. / timestep));

    bool goal_seen = false, cleared = false;
    std::uint8_t goal_reason = 0;
    double goal_rad = 0., crank_at_goal = 0., phase_at_clear = 0.;
    double elapsed_at_clear = 0.;
    double max_crank_rad = -std::numeric_limits<double>::infinity();
    double min_crank_rad = std::numeric_limits<double>::infinity();
    for (int i = 0; i < window && outcome_free() && !cleared; ++i) {
        (void)runtime->advance(runtime->snapshot().step + 1, control, 0.,
                               0., std::nullopt);
        bike::validation::RuntimeRigAccess::inspect(
            *runtime,
            [&](const Stepper &stepper, const runtime::PhysicalStep &) {
                const double phase = model_access::readonly_buffer(
                    stepper.data()->qpos, stepper.model()->nq)
                    [static_cast<std::size_t>(crank.qpos)];
                const auto &reposition =
                    stepper.drive().state().pedaling.reposition;
                if (reposition.has_value() &&
                    reposition->goal_rad.has_value()) {
                    if (!goal_seen) {
                        goal_seen = true;
                        goal_rad = *reposition->goal_rad;
                        goal_reason = reposition->reason;
                        crank_at_goal = phase;
                    }
                    max_crank_rad = std::max(max_crank_rad, phase);
                    min_crank_rad = std::min(min_crank_rad, phase);
                    elapsed_at_clear = reposition->elapsed_s;
                } else if (goal_seen) {
                    cleared = true;
                    phase_at_clear = phase;
                    if (reposition.has_value())
                        elapsed_at_clear = reposition->elapsed_s;
                }
            });
    }

    bt::require(goal_seen,
                "the one-shot request engages a reposition goal "
                "(crank phase at request: " +
                    std::to_string(phase_at_request) + " rad)");
    bt::require(goal_reason == 1,
                "the goal reports the operator-request reason, not the "
                "stall reflex (reason " +
                    std::to_string(static_cast<int>(goal_reason)) + ")");
    bt::require(cleared,
                "the manoeuvre completes inside the window "
                "(goal " + std::to_string(goal_rad) +
                    " rad, last phase " +
                    std::to_string(max_crank_rad) + " rad)");
    // R3's "nearest power phase below": the goal is exactly a multiple
    // of pi away from kPowerPhaseOffsetRad — not any lower phase —
    // and never above the crank it started from.
    bt::require(std::fmod(std::abs(goal_rad -
                                   drivetrain::kPowerPhaseOffsetRad),
                          std::numbers::pi) < 1e-6,
                "the goal is a horizontal power phase (goal " +
                    std::to_string(goal_rad) + " rad, offset " +
                    std::to_string(drivetrain::kPowerPhaseOffsetRad) +
                    " rad)");
    bt::require(goal_rad <= crank_at_goal,
                "the goal is a lower power phase (goal " +
                    std::to_string(goal_rad) + " rad, crank at goal start " +
                    std::to_string(crank_at_goal) + " rad)");
    bt::require(max_crank_rad <= crank_at_goal + .01,
                "the crank moves only backward during the manoeuvre "
                "(peak " + std::to_string(max_crank_rad) +
                    " rad vs start " + std::to_string(crank_at_goal) +
                    " rad)");
    bt::require(crank_at_goal - phase_at_clear >= .1,
                "the manoeuvre actually travels (start " +
                    std::to_string(crank_at_goal) + " rad, end " +
                    std::to_string(phase_at_clear) + " rad)");
    // Clear fires at remaining <= tolerance (0.05 rad) or at the 3 s
    // timeout. The reposition branch's rate damper
    // (drive.pedaling.reposition_damping_nms_rad) holds a ~0.014 rad steady-state residual
    // against the leg-weight lever at the horizontal pose — inside the
    // tolerance — so the manoeuvre clears by arrival, ~0.7-0.9 s on
    // this plant. The first require pins the plan's 0.05 rad
    // acceptance bound; the second pins arrival over a timeout exit.
    bt::require(phase_at_clear - goal_rad >= -.01 &&
                    phase_at_clear - goal_rad <= .05,
                "the crank arrives inside the 0.05 rad tolerance (end " +
                    std::to_string(phase_at_clear) + " rad vs goal " +
                    std::to_string(goal_rad) + " rad; approach min " +
                    std::to_string(min_crank_rad) + " rad)");
    bt::require(elapsed_at_clear < 1.5,
                "the manoeuvre clears on tolerance well before the 3 s "
                "timeout (clear at " + std::to_string(elapsed_at_clear) +
                    " s)");
}

// test_reposition_backpedal_respects_the_clutch_topology — R5 + S3:
// while the crank runs backward the rear wheel is not dragged back
// (freehub), the crank clutch stays open and applies no backward torque
// to the motor-side shaft, the delivered crank torque keeps inside the
// reposition branch's bound, and both pedal weld normals stay safe.
void test_reposition_backpedal_respects_the_clutch_topology() {
    auto runtime = reposition_env();
    const runtime::RideControl control{.motor_torque_nm = 0.,
                                       .motor_limit_nm = std::nullopt,
                                       .human_torque_nm = 0.,
                                       .crank_target_rate_rad_s =
                                           std::nullopt,
                                       .posture = std::nullopt};
    (void)runtime->advance(40, control, 0., 0., std::nullopt);
    bt::require(runtime->snapshot().step == 40,
                "the dead-spot plant reaches the request step");
    runtime->request_crank_reposition();

    // The reposition torque cap is the profile's seated_climb crank
    // torque ceiling (2026-11-16 plan task 2), not a code constant.
    const double crank_torque_cap_nm =
        cfg::load_physics(
            repo_file("examples/research/viewer_physics_welded.toml"))
            .seated_climb.max_crank_torque_nm;

    // Joint addresses are model constants — resolve them once outside
    // the inspected-step loop instead of paying an mj_name2id (and a
    // non-assertion bt::require) per step.
    double timestep = 0.;
    SpinJoint crank, wheel, shaft;
    bike::validation::RuntimeRigAccess::inspect(
        *runtime, [&](const Stepper &stepper, const runtime::PhysicalStep &) {
            const mjModel *m = stepper.model();
            timestep = m->opt.timestep;
            crank = spin_joint(m, "crank_spin");
            wheel = spin_joint(m, "rear_wheel_spin");
            shaft = spin_joint(m, "drive_shaft_spin");
        });
    const int window = static_cast<int>(std::llround(4. / timestep));

    bool goal_seen = false, cleared = false;
    double wheel_at_goal = 0., shaft_at_goal = 0.;
    double min_wheel_rad = std::numeric_limits<double>::infinity();
    double min_shaft_rad_s = std::numeric_limits<double>::infinity();
    double min_shaft_rad = std::numeric_limits<double>::infinity();
    int shaft_samples = 0;
    int backpedal_steps = 0, backpedal_clutch_engaged = 0,
        backpedal_clutch_samples = 0, backpedal_clutch_torque_samples = 0;
    double min_backpedal_clutch_torque_nm =
        std::numeric_limits<double>::infinity();
    double min_pedal_normal_n = std::numeric_limits<double>::infinity();
    double max_abs_crank_torque_nm = 0.;
    int pedal_samples = 0;
    for (int i = 0; i < window &&
                    !runtime->snapshot().outcome.has_value() && !cleared;
         ++i) {
        (void)runtime->advance(runtime->snapshot().step + 1, control, 0.,
                               0., std::nullopt);
        bike::validation::RuntimeRigAccess::inspect(
            *runtime,
            [&](const Stepper &stepper, const runtime::PhysicalStep &) {
                const mjModel *m = stepper.model();
                const mjData *d = stepper.data();
                const auto qpos =
                    model_access::readonly_buffer(d->qpos, m->nq);
                const auto qvel =
                    model_access::readonly_buffer(d->qvel, m->nv);
                const auto &reposition =
                    stepper.drive().state().pedaling.reposition;
                const bool active = reposition.has_value() &&
                                    reposition->goal_rad.has_value();
                if (active) {
                    if (!goal_seen) {
                        goal_seen = true;
                        wheel_at_goal =
                            qpos[static_cast<std::size_t>(wheel.qpos)];
                        shaft_at_goal =
                            qpos[static_cast<std::size_t>(shaft.qpos)];
                    }
                    min_wheel_rad = std::min(
                        min_wheel_rad,
                        qpos[static_cast<std::size_t>(wheel.qpos)]);
                    min_shaft_rad = std::min(
                        min_shaft_rad,
                        qpos[static_cast<std::size_t>(shaft.qpos)]);
                    if (const auto shaft_rate =
                            lane_real(stepper.drive().state().last,
                                      "drive_shaft_rad_s")) {
                        ++shaft_samples;
                        min_shaft_rad_s =
                            std::min(min_shaft_rad_s, *shaft_rate);
                    }
                    // The R5 mechanism itself: once the crank actually
                    // backpedals, the motor-side clutch must be open and
                    // apply no torque to the shaft. (The shaft is a free
                    // rotor — no bearing drag exists in the model — so
                    // its own residual drift is not evidence of crank
                    // backdrive; the clutch force is.)
                    if (qvel[static_cast<std::size_t>(crank.dof)] < -.5) {
                        ++backpedal_steps;
                        if (const auto engaged =
                                lane_flag(stepper.drive().state().last,
                                          "crank_clutch_engaged")) {
                            ++backpedal_clutch_samples;
                            if (*engaged) ++backpedal_clutch_engaged;
                        }
                        if (const auto torque =
                                lane_real(stepper.drive().state().last,
                                          "crank_clutch_torque_nm")) {
                            ++backpedal_clutch_torque_samples;
                            min_backpedal_clutch_torque_nm = std::min(
                                min_backpedal_clutch_torque_nm, *torque);
                        }
                    }
                } else if (goal_seen) {
                    cleared = true;
                }
                // S3: weld normals and delivered crank torque, over the
                // whole post-request window (the manoeuvre plus the tick
                // the request consumed).
                const auto &contacts = stepper.rider_contacts().state();
                if (contacts.settled_welds.has_value()) {
                    const auto &welds = *contacts.settled_welds;
                    const auto *front = settled_entry(welds, "front_pedal");
                    const auto *rear = settled_entry(welds, "rear_pedal");
                    bt::require(front != nullptr && rear != nullptr,
                                "both pedal weld entries are published");
                    ++pedal_samples;
                    min_pedal_normal_n = std::min(
                        {min_pedal_normal_n, front->normal_n,
                         rear->normal_n});
                    max_abs_crank_torque_nm = std::max(
                        max_abs_crank_torque_nm,
                        std::abs(contacts.delivered_crank_torque_nm));
                }
            });
    }

    bt::require(goal_seen && cleared,
                "the manoeuvre ran (goal engaged and cleared)");
    bt::require(pedal_samples > 0,
                "the settled weld entries published during the window");
    // R5: the freehub/shaft side never follows the crank backward.
    bt::require(wheel_at_goal - min_wheel_rad <= .02,
                "the rear wheel's backward displacement stays under "
                "0.02 rad (measured " +
                    std::to_string(wheel_at_goal - min_wheel_rad) +
                    " rad)");
    bt::require(shaft_samples > 0,
                "the drive_shaft_rad_s lane publishes on this topology");
    bt::require(backpedal_steps > 0,
                "the crank genuinely backpedals in the window");
    bt::require(backpedal_clutch_samples > 0 &&
                    backpedal_clutch_engaged == 0,
                "the crank clutch stays open while the crank backpedals "
                "(engaged on " + std::to_string(backpedal_clutch_engaged) +
                    " of " + std::to_string(backpedal_clutch_samples) +
                    " sampled backpedal steps; shaft drift " +
                    std::to_string(shaft_at_goal - min_shaft_rad) +
                    " rad, min shaft rate " +
                    std::to_string(min_shaft_rad_s) + " rad/s)");
    bt::require(backpedal_clutch_torque_samples > 0 &&
                    min_backpedal_clutch_torque_nm >= -1e-6,
                "the clutch applies no backward torque to the shaft "
                "while the crank backpedals (min " +
                    std::to_string(min_backpedal_clutch_torque_nm) +
                    " N m over " +
                    std::to_string(backpedal_clutch_torque_samples) +
                    " sampled backpedal steps)");
    // S3: the weld loads stay inside their envelopes — the manoeuvre is
    // a bounded leg push, not a pedal pull.
    bt::require(min_pedal_normal_n >= -20.,
                "the pedal weld normals never pull past 20 N "
                "(min " + std::to_string(min_pedal_normal_n) + " N)");
    // The cap is the rider's crank torque ceiling
    // seated_climb.max_crank_torque_nm (decision 11), carried into the
    // reposition branch by SpindleConfig (2026-11-16 plan task 2).
    bt::require(max_abs_crank_torque_nm <= crank_torque_cap_nm,
                "the delivered crank torque stays inside the reposition "
                "bound (max |tau| " +
                    std::to_string(max_abs_crank_torque_nm) + " N m)");
}

// test_reposition_request_waits_for_the_advancing_tick — the one-shot
// contract itself: request_crank_reposition() queues the edge, and only
// the first advancing control tick consumes it into the pedaling
// policy's goal — probe_step_inputs() and non-tick advances leave it
// pending, and restore() (reached through reset()) drops a still-queued
// edge so it cannot fire after the reset.
void test_reposition_request_waits_for_the_advancing_tick() {
    double control_period_s = 0.;
    // Ticks every other interval (steps_per_period = 2): the welded
    // profile's physics control_period_s equals one timestep, which
    // would make every advancing interval a tick and leave nothing for
    // the non-tick invariant to bite on.
    auto runtime = reposition_env(&control_period_s, 2);
    const runtime::RideControl control{.motor_torque_nm = 0.,
                                       .motor_limit_nm = std::nullopt,
                                       .human_torque_nm = 0.,
                                       .crank_target_rate_rad_s =
                                           std::nullopt,
                                       .posture = std::nullopt};
    double timestep = 0.;
    bike::validation::RuntimeRigAccess::inspect(
        *runtime, [&](const Stepper &stepper, const runtime::PhysicalStep &) {
            timestep = stepper.model()->opt.timestep;
        });
    const auto period = std::llround(control_period_s / timestep);
    bt::require(period >= 2,
                "the control period spans several physics steps");

    const auto goal_engaged = [&] {
        bool engaged = false;
        bike::validation::RuntimeRigAccess::inspect(
            *runtime,
            [&](const Stepper &stepper, const runtime::PhysicalStep &) {
                const auto &reposition =
                    stepper.drive().state().pedaling.reposition;
                engaged = reposition.has_value() &&
                          reposition->goal_rad.has_value();
            });
        return engaged;
    };

    // Interval 0 is a control tick, so after advance(1) the next
    // period - 1 intervals are all non-tick: nothing in between can
    // legitimately consume the request.
    (void)runtime->advance(1, control, 0., 0., std::nullopt);
    bt::require(runtime->snapshot().step == 1 && !goal_engaged(),
                "the dead-spot plant advances without a goal");

    runtime->request_crank_reposition();
    // The probe computes the interval's force fold without committing —
    // the pending exchange never runs on a non-advancing pass.
    (void)runtime->probe_step_inputs(control, 0., 0.);
    (void)runtime->advance(period, control, 0., 0., std::nullopt);
    bt::require(runtime->snapshot().step == period && !goal_engaged(),
                "probe and the non-tick advances leave the request pending");
    // The next interval is the control tick: the edge is consumed
    // there — and only there — and the goal engages.
    (void)runtime->advance(period + 1, control, 0., 0., std::nullopt);
    bt::require(goal_engaged(),
                "the advancing control tick consumes the request");

    // A request queued but never consumed does not survive a reset:
    // restore() clears the pending edge before the next tick can see it.
    runtime->request_crank_reposition();
    runtime->reset();
    (void)runtime->advance(1, control, 0., 0., std::nullopt);
    bt::require(!goal_engaged(), "reset clears the queued request");
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        test_requested_reposition_reaches_the_lower_power_phase();
        test_reposition_backpedal_respects_the_clutch_topology();
        test_reposition_request_waits_for_the_advancing_tick();
    });
}
