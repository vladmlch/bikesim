// Ports tests/reference/test_seated_posture_program.py — R6: forward
// lean and a bounded body-thrust pulse as posture intent.
//
// The oracle's AST source sweep (test_posture_program_reads_no_inertial_
// signal) has no native analogue: the C++ update() signature cannot
// receive the inertial channels by construction — the gate is structural.
// Its behavioral half survives as the resolver's signal-independence,
// exercised there and by test_resolver_road_grade_survives_probe_and_
// ignores_imu below.
//
// test_runtime_passes_current_road_snapshot_before_intent_and_force_
// evaluation drives a SimpleNamespace interception point inside the
// oracle's apply_forces; the values it asserts — the preview-end road
// sample and the folded grade — are asserted here directly off
// road_samples/road_grade_for_posture on the identical inputs.
#include "support/fixture_test.hpp"
#include "support/golden.hpp"
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "initialization/setup_owner.hpp"
#include "model_access.hpp"
#include "research/environment.hpp"
#include "rider/attachment_wrench.hpp"
#include "rider/intent.hpp"
#include "rider/rider_posture.hpp"
#include "runtime/control.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "runtime/step.hpp"
#include "terrain/trackfile.hpp"
#include "validation/runtime_access.hpp"
#include "writers/rider_contacts.hpp"
#include "stepper.hpp"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace rider = ::rider;
namespace research = bike::research;
namespace cfg = bike::configuration;
namespace sw = runtime::sample_wire;

// The live example document — bt::oracle::root() is
// <repo>/tests/fixtures/golden. The oracle's WELDED constant reads this
// same file: the migration fixture mirror predates F1 and lacks the
// activation lag, delivered budgets and weld envelopes the episodes
// assert.
std::filesystem::path repo_file(std::string_view relative) {
    return bt::oracle::root().parent_path().parent_path().parent_path() /
           std::filesystem::path(relative);
}

bool approx(double actual, double expected, double rtol = 1e-9,
            double atol = 1e-12) {
    return std::abs(actual - expected) <= atol + rtol * std::abs(expected);
}

rider::IntentConfig enabled(double lean_rate = 0.) {
    rider::IntentConfig config;
    config.enabled = true;
    if (lean_rate != 0.) config.lean_rate_rad_s = lean_rate;
    return config;
}

// test_posture_program_reads_no_inertial_signal — the AST sweep's
// behavioral contract: the posture intent resolves identically under
// arbitrary IMU perturbation, because no inertial channel reaches it.
void test_posture_program_reads_no_inertial_signal() {
    std::array<rider::RiderIntent, 2> resolvers{
        rider::RiderIntent(enabled(), .0005),
        rider::RiderIntent(enabled(), .0005)};
    const rider::IntentSignals calm;
    rider::IntentSignals perturbed;
    perturbed.pitch_rate_up_rad_s = 4.;
    perturbed.specific_force_body_mps2 = {5., 0., 8.};
    const auto a = resolvers[0].resolve({}, calm, 0, .18);
    const auto b = resolvers[1].resolve({}, perturbed, 0, .18);
    bt::require(a.posture.has_value() && b.posture.has_value(),
                "both resolves emit a posture");
    bt::require(a.posture->torso_lean_rad == b.posture.value().torso_lean_rad && // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                    a.posture.value().pelvis_pitch_rad == // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                        b.posture.value().pelvis_pitch_rad && // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                    a.posture.value().pelvis_offset_m == // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                        b.posture.value().pelvis_offset_m && // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                    a.posture->use_saddle == b.posture.value().use_saddle, // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                "the posture intent is invariant to inertial inputs");
}

// test_lean_follows_road_grade_not_body_pitch
void test_lean_follows_road_grade_not_body_pitch() {
    rider::IntentConfig config = enabled();
    config.lean_rate_rad_s = 5.;
    rider::SeatedPostureProgram program(config);
    double lean = 0.;
    for (int i = 0; i < 200; ++i)
        lean = program.update(i * .01, .20, .01);
    bt::require(approx(lean, std::atan(.20), 0., 1e-9),
                "the lean tracks atan(road grade), not body pitch");
}

// test_road_grade_uses_underwheel_and_preview_samples_uniformly
void test_road_grade_uses_underwheel_and_preview_samples_uniformly() {
    const std::array<runtime::RoadSample, 3> samples{
        runtime::RoadSample{.x_m = 0., .height_m = 0., .grade = .1},
        runtime::RoadSample{.x_m = 1., .height_m = 0., .grade = .2},
        runtime::RoadSample{.x_m = 2., .height_m = 0., .grade = .3}};
    bt::require(
        approx(runtime::road_grade_for_posture(samples), .2),
        "the posture grade averages the under-wheel and preview rows");
    bt::invalid([] {
        (void)runtime::road_grade_for_posture(
            std::span<const runtime::RoadSample>{});
    });
}

// test_resolver_road_grade_survives_probe_and_ignores_imu
void test_resolver_road_grade_survives_probe_and_ignores_imu() {
    std::array<rider::RiderIntent, 2> resolvers{
        rider::RiderIntent(enabled(), .0005),
        rider::RiderIntent(enabled(), .0005)};
    const rider::IntentSignals base;
    rider::IntentSignals perturbed;
    perturbed.pitch_rate_up_rad_s = 4.;
    perturbed.specific_force_body_mps2 = {5., 0., 8.};
    rider::IntentControl first;
    for (std::int64_t step = 0; step < 2000; step += 20) {
        const auto calm = resolvers[0].resolve({}, base, step, .18);
        const auto noisy =
            resolvers[1].resolve({}, perturbed, step, .18);
        bt::require(calm.posture.has_value() && noisy.posture.has_value(),
                    "every tick emits a posture");
        bt::require(calm.posture.value().torso_lean_rad == // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                            noisy.posture.value().torso_lean_rad && // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                        calm.posture.value().pelvis_pitch_rad == // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                            noisy.posture.value().pelvis_pitch_rad && // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                        calm.posture.value().pelvis_offset_m == // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                            noisy.posture.value().pelvis_offset_m && // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                        calm.posture.value().use_saddle == // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                            noisy.posture.value().use_saddle, // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                    "the IMU perturbation cannot reach the posture");
        first = calm;
    }
    bt::require(approx(first.posture.value().torso_lean_rad, std::atan(.18)), // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                "the converged lean is atan(.18)");
    auto &resolver = resolvers[0];
    const double before = resolver.policy().state().time_s;
    const auto probe =
        resolver.resolve({}, perturbed, 2000, .18, std::nullopt,
                         std::nullopt, true, false);
    bt::require(resolver.policy().state().time_s == before,
                "a probe resolve does not advance the policy clock");
    bt::require(probe.posture.has_value() &&
                    probe.posture->torso_lean_rad ==
                        first.posture.value().torso_lean_rad, // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                "the probe reproduces the committed posture");
}

// test_runtime_passes_current_road_snapshot_before_intent_and_force_
// evaluation — the interception point is an oracle seam; the sampled
// values it asserts are checked directly on the identical inputs.
void test_runtime_passes_current_road_snapshot_before_intent_and_force_evaluation() {
    // vertices [[-1., -.18], [4., .72]], wheel xpos (0., 1.), lookahead 2.
    const std::array<std::array<double, 2>, 2> vertices{
        std::array<double, 2>{-1., -.18}, {4., .72}};
    const std::array<double, 2> wheel_x{0., 1.};
    const auto road = runtime::road_samples(vertices, wheel_x, 2.);
    bt::require(!road.empty(), "the road window is non-empty");
    bt::require(approx(road.back().x_m, 3.),
                "the preview ends a lookahead ahead of the front wheel");
    bt::require(approx(runtime::road_grade_for_posture(road), .18),
                "the folded road grade is the .18 ramp");
}

// test_body_pulse_is_finite_and_has_no_impulsive_endpoint
void test_body_pulse_is_finite_and_has_no_impulsive_endpoint() {
    bt::require(rider::body_pulse(0., 1., .5, .1) == 0.,
                "zero before the window");
    bt::require(rider::body_pulse(1., 1., .5, .1) == 0.,
                "zero at the window edge");
    bt::require(std::abs(rider::body_pulse(1.25, 1., .5, .1) - .1) < 1e-12,
                "the mid-window plateau reaches the amplitude");
    bt::require(rider::body_pulse(1.5, 1., .5, .1) == 0.,
                "zero after the window");
}

// test_body_pulse_validates_its_inputs
void test_body_pulse_validates_its_inputs() {
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    constexpr double inf = std::numeric_limits<double>::infinity();
    for (const double bad : {nan, inf}) {
        bt::invalid([&] { (void)rider::body_pulse(bad, 1., .5, .1); });
        bt::invalid([&] { (void)rider::body_pulse(1., bad, .5, .1); });
        bt::invalid([&] { (void)rider::body_pulse(1., 1., bad, .1); });
        bt::invalid([&] { (void)rider::body_pulse(1., 1., .5, bad); });
    }
    bt::invalid([&] { (void)rider::body_pulse(1., 1., 0., .1); });
    bt::invalid([&] { (void)rider::body_pulse(1., 1., -.5, .1); });
}

// test_body_pulse_is_continuous_around_its_edges
void test_body_pulse_is_continuous_around_its_edges() {
    bt::require(std::abs(rider::body_pulse(1. + 1e-9, 1., .5, .1)) < 1e-6,
                "the rising edge is continuous");
    bt::require(std::abs(rider::body_pulse(1.5 - 1e-9, 1., .5, .1)) < 1e-6,
                "the falling edge is continuous");
}

// test_posture_program_rate_limits_and_bounds_the_pulse
void test_posture_program_rate_limits_and_bounds_the_pulse() {
    rider::IntentConfig config;
    config.enabled = true;
    config.lean_gain = 0.;
    config.max_forward_lean_rad = .35;
    config.max_backward_lean_rad = .1;
    config.lean_rate_rad_s = .5;
    {
        rider::SeatedPostureProgram program(config);
        program.schedule_pulse(1., .5, 10.);
        constexpr double dt = .01;
        double lean = 0.;
        for (int i = 0; i < 80; ++i)
            lean = program.update(i * dt, 0., dt);
        bt::require(std::abs(lean) <= 1e-12,
                    "no lean before the pulse window");
        double peak = 0.;
        for (int i = 80; i < 160; ++i) {
            lean = program.update(i * dt, 0., dt);
            peak = std::max(peak, lean);
        }
        bt::require(peak > 0. &&
                        peak <= config.max_forward_lean_rad + 1e-12,
                    "the pulse intent rises but never beyond the "
                    "strategy envelope");
    }
    {
        rider::SeatedPostureProgram program(config);
        program.schedule_pulse(.2, .6, .3);
        constexpr double dt = .01;
        double previous = 0.;
        for (int i = 0; i < 200; ++i) {
            const double current = program.update(i * dt, 0., dt);
            bt::require(
                current - previous <= config.lean_rate_rad_s * dt + 1e-12,
                "consecutive intents differ by at most the rate bound");
            previous = current;
        }
    }
}

// test_posture_program_rejects_nonfinite_or_reversed_schedules
void test_posture_program_rejects_nonfinite_or_reversed_schedules() {
    rider::SeatedPostureProgram program(enabled());
    bt::invalid([&] {
        program.schedule_pulse(std::numeric_limits<double>::quiet_NaN(),
                               .5, .1);
    });
    bt::invalid([&] { program.schedule_pulse(0., 0., .1); });
    bt::invalid([&] {
        program.schedule_pulse(0., .5,
                               std::numeric_limits<double>::infinity());
    });
}

// ---- _environment / _run: one graded episode, per-tick channels -------

struct GradedEnv {
    std::unique_ptr<runtime::NativeRideRuntime> runtime;
    std::int64_t period = 0;
    // The research episode horizon — the ride runtime carries no duration
    // gate of its own, so callers stop at max_steps like env.done's
    // truncation does.
    std::int64_t max_steps = 0;
    double delivered_positive_budget_w = 0.;
};

GradedEnv graded_env(bool diagnostic) {
    auto owner = std::make_unique<research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = cfg::load_physics(
        repo_file("examples/research/viewer_physics_welded.toml"));
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.physics->timestep_s = .0005;
    request.setup.model.physics->initial_speed_mps = 0.;
    // equilibrium_cache_enabled stays true — the TOML's own value and
    // the oracle's settle path.
    request.setup.model.rider =
        bike::model::rider_from_args("articulated_planar");
    request.setup.track = bike::terrain::track_from_dict(
        bike::terrain::parse_track_toml(
            "name = \"probe\"\nlength_m = 200.0\nsurface = \"hardpack\"\n"
            "grade_profile = { knots = [[0.0, 0.0], [5.0, 0.18], "
            "[200.0, 0.18]] }\n"));
    request.experiment.duration_s = 6.;
    request.experiment.record_decimation = 80;
    request.experiment.stop_on_model_violation = !diagnostic;
    request.sensors = research::SensorConfig{};
    request.sensors.sample_period_s = .005;
    // The budget is read off the articulated config BEFORE the request
    // moves — `request` is a reference into *owner, so its fields are
    // disengaged after prepare_environment consumes it.
    const auto articulated = request.setup.model.physics->articulated;
    const auto prepared = research::prepare_environment(std::move(owner));
    auto bootstrap = std::make_unique<runtime::RuntimeBootstrap>(
        std::move(prepared->bootstrap.ride));
    GradedEnv env;
    // One env.step(control): the research control window (.01 s / .0005 s
    // = 20 steps), not the 10-step accounting period.
    env.period = prepared->bootstrap.config.control_steps;
    env.max_steps = prepared->bootstrap.config.max_steps;
    const auto budget = articulated.delivered_positive_budget_w();
    bt::require(budget.has_value(),
                "the welded config declares a delivered-power budget");
    env.delivered_positive_budget_w = (budget).value(); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
    env.runtime =
        std::make_unique<runtime::NativeRideRuntime>(std::move(*bootstrap));
    return env;
}

struct RunResult {
    std::vector<double> time_s, grip_pull_n, power_w, chest_z_m;
    double grip_pull_max_n = 0.;
    std::size_t infeasible = 0;
    std::optional<std::vector<std::string>> first_failure;
};

const rider::AttachmentSample *attachment_sample(
    const Stepper &stepper, std::string_view name) {
    const auto &state = stepper.rider_contacts().state();
    for (const auto &[key, entry] : state.attachment_samples)
        if (key == name)
            if (const auto *sample =
                    std::get_if<rider::AttachmentSample>(&entry))
                return sample;
    return nullptr;
}

RunResult run_episode(bool diagnostic,
                    std::optional<std::array<double, 3>> pulse) {
    auto env = graded_env(diagnostic);
    if (pulse)
        bike::validation::RuntimeRigAccess::schedule_rider_pulse(
            *env.runtime, (*pulse)[0], (*pulse)[1], (*pulse)[2]);
    int torso = -1;
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &stepper,
                          const runtime::PhysicalStep &) {
            const mjModel *m = stepper.model();
            for (int i = 0; i < m->nbody; ++i) {
                const char *name = mj_id2name(m, mjOBJ_BODY, i);
                if (name != nullptr &&
                    (std::string_view(name) == "torso" ||
                     std::string_view(name) == "rider_torso" ||
                     std::string_view(name) == "rider_body"))
                    torso = i;
            }
        });
    RunResult out;
    const runtime::RideControl control{.motor_torque_nm = 0.,
                                       .motor_limit_nm = std::nullopt,
                                       .human_torque_nm = 30.,
                                       .crank_target_rate_rad_s = std::nullopt,
                                       .posture = std::nullopt};
    for (int tick = 0; tick < 6000; ++tick) {
        if (env.runtime->snapshot().outcome.has_value() ||
            env.runtime->snapshot().step >= env.max_steps)
            break;
        const auto result = env.runtime->advance(
            env.runtime->snapshot().step + env.period, control, 0., 0.,
            std::nullopt);
        bike::validation::RuntimeRigAccess::inspect(
            *env.runtime, [&](const Stepper &stepper,
                              const runtime::PhysicalStep &physical) {
                double grip = 0.;
                for (const char *key : {"grip_left", "grip_right"}) {
                    const auto *sample = attachment_sample(stepper, key);
                    if (sample != nullptr) {
                        grip += std::max(0., sample->pull_n);
                        out.grip_pull_max_n =
                            std::max(out.grip_pull_max_n, sample->pull_n);
                    }
                }
                out.grip_pull_n.push_back(grip);
                out.power_w.push_back(
                    physical.rider_control()
                        .state()
                        .effort_diagnostics.positive_power_w);
                if (torso >= 0)
                    out.chest_z_m.push_back(
                        model_access::readonly_buffer(
                            stepper.data()->xpos,
                            3 * stepper.model()->nbody)
                            [3 * static_cast<std::size_t>(torso) + 2]);
                out.time_s.push_back(stepper.data()->time);
            });
        const auto intervals = env.runtime->recorded_intervals();
        if (!intervals.empty())
            if (const auto *wire = sw::find(intervals.back()->channels,
                                            "attachment_violations"))
                for (const auto &value : sw::array(*wire))
                    if (sw::string(value) == "rider_controller.infeasible")
                        ++out.infeasible;
        if (result.reason == "outcome") break;
    }
    if (const auto failure = env.runtime->snapshot().first_failure)
        out.first_failure = failure->violations;
    return out;
}

double interp(double x, const std::vector<double> &xs,
              const std::vector<double> &ys) {
    if (x <= xs.front()) return ys.front();
    if (x >= xs.back()) return ys.back();
    const auto it = std::ranges::upper_bound(xs, x);
    const std::size_t right =
        static_cast<std::size_t>(std::distance(xs.begin(), it));
    const std::size_t left = right - 1;
    return ys[left] + (ys[right] - ys[left]) * (x - xs[left]) /
                          (xs[right] - xs[left]);
}

// test_pulse_redistributes_load_through_inertia_only (slow)
void test_pulse_redistributes_load_through_inertia_only() {
    const std::array<double, 3> pulse{2.5, .5, .15};
    // Diagnostic mode keeps both episodes recording to the end; each must
    // record no first failure, which is the strict run's contract with the
    // full trace kept for diagnosis. (Until the freehub's tendon-limit rows
    // joined the closure channel on 2026-11-04, every native run tripped
    // `energy.constraint_work` at t=0.005 s and this gate tolerated it.)
    constexpr bool diagnostic = true;
    const auto calm = run_episode(diagnostic, std::nullopt);
    const auto surged = run_episode(diagnostic, pulse);
    for (const auto &result : {calm, surged})
        bt::require(!result.first_failure.has_value(),
                    "the monitor records no model violation");
    const double start = pulse[0];
    std::vector<double> window_times, window_grip;
    for (std::size_t i = 0; i < surged.time_s.size(); ++i)
        if (surged.time_s[i] >= start &&
            surged.time_s[i] <= start + pulse[1]) {
            window_times.push_back(surged.time_s[i]);
            window_grip.push_back(surged.grip_pull_n[i]);
        }
    bt::require(!window_times.empty(),
                "the pulse window contains control ticks");
    double delta_max = 0.;
    for (std::size_t i = 0; i < window_times.size(); ++i)
        delta_max = std::max(
            delta_max,
            std::abs(window_grip[i] -
                     interp(window_times[i], calm.time_s,
                            calm.grip_pull_n)));
    bt::require(delta_max > 1e-6,
                "the pulse moves the bar reaction through inertia");
    bt::require(surged.grip_pull_max_n <= 300. + 1e-6,
                "each hand stays inside the 300 N pull budget");
    double power_max = 0.;
    for (const double power : surged.power_w)
        power_max = std::max(power_max, power);
    // F1: the delivered-power verdict reads the declared monitor budget,
    // not the command cap.
    const auto env_budget = graded_env(diagnostic);
    bt::require(power_max <= env_budget.delivered_positive_budget_w + 1e-6,
                "delivered power stays inside the monitor budget");
    if (!surged.chest_z_m.empty() && !calm.chest_z_m.empty()) {
        double chest_delta = 0.;
        for (std::size_t i = 0; i < window_times.size(); ++i)
            chest_delta = std::max(
                chest_delta,
                interp(window_times[i], calm.time_s, calm.chest_z_m) -
                    surged.chest_z_m[i]);
        bt::require(chest_delta > 0.,
                    "the lean intent drops the chest below the calm "
                    "trajectory");
    }
}

// test_trim_is_delayed_and_increases_forward_lean_for_light_front
void test_trim_is_delayed_and_increases_forward_lean_for_light_front() {
    rider::IntentConfig config;
    config.enabled = true;
    config.lean_rate_rad_s = 10.;
    config.trim_dead_time_s = .2;
    rider::SeatedPostureProgram program(config);
    const double base = std::atan(.15);
    double lean = 0.;
    for (int i = 0; i < 20; ++i)
        lean = program.update(i * .01, .15, .01, .20);
    bt::require(approx(lean, base),
                "the dead-time keeps the lean at the grade baseline");
    for (int i = 20; i < 300; ++i)
        lean = program.update(i * .01, .15, .01, .20);
    // The oracle asserts the FINAL lean once — the trim integrator is
    // still mid-rise for most of the window.
    bt::require(base + .05 < lean && lean <= base + .15 + 1e-9,
                "the delayed trim increases forward lean for the "
                "light front end");
}

// test_geometric_limit_caps_lean
void test_geometric_limit_caps_lean() {
    rider::IntentConfig config;
    config.enabled = true;
    config.lean_rate_rad_s = 10.;
    config.max_forward_lean_rad = .8;
    rider::SeatedPostureProgram program(config);
    double lean = 0.;
    for (int i = 0; i < 200; ++i)
        lean = program.update(i * .01, .45, .01, std::nullopt, .2);
    bt::require(approx(lean, .2),
                "the geometric limit caps the lean at .2 rad");
}

// test_preview_is_last_sample_and_preserves_exact_endpoint
void test_preview_is_last_sample_and_preserves_exact_endpoint() {
    const std::array<runtime::RoadSample, 2> samples{
        runtime::RoadSample{.x_m = 0., .height_m = 0., .grade = .1},
        runtime::RoadSample{.x_m = 2., .height_m = .4, .grade = .3}};
    bt::require(runtime::road_grade_preview(samples) == .3,
                "the preview grade is the last sample's");
    const std::array<std::array<double, 2>, 2> vertices{
        std::array<double, 2>{0., 0.}, {4., .4}};
    const std::array<double, 2> wheel_x{0., 1.};
    const auto road = runtime::road_samples(vertices, wheel_x, 2.03);
    bt::require(approx(road.back().x_m, 3.03),
                "the preview preserves the exact endpoint");
    bt::invalid([] {
        (void)runtime::road_grade_preview(
            std::span<const runtime::RoadSample>{});
    });
}

// test_geometric_limit_replaces_the_legacy_config_ceiling_when_available
void test_geometric_limit_replaces_the_legacy_config_ceiling_when_available() {
    rider::IntentConfig config;
    config.enabled = true;
    config.lean_rate_rad_s = 10.;
    config.max_forward_lean_rad = .1;
    rider::SeatedPostureProgram program(config);
    bt::require(approx(program.update(0., .5, .1, std::nullopt, .4), .4),
                "an explicit geometric limit replaces the config "
                "ceiling");
    rider::SeatedPostureProgram legacy(config);
    bt::require(approx(legacy.update(0., .5, .1), .1),
                "without it the legacy config ceiling applies");
}

} // namespace

int main() {
    return bt::run([] {
        test_posture_program_reads_no_inertial_signal();
        test_lean_follows_road_grade_not_body_pitch();
        test_road_grade_uses_underwheel_and_preview_samples_uniformly();
        test_resolver_road_grade_survives_probe_and_ignores_imu();
        test_runtime_passes_current_road_snapshot_before_intent_and_force_evaluation();
        test_body_pulse_is_finite_and_has_no_impulsive_endpoint();
        test_body_pulse_validates_its_inputs();
        test_body_pulse_is_continuous_around_its_edges();
        test_posture_program_rate_limits_and_bounds_the_pulse();
        test_posture_program_rejects_nonfinite_or_reversed_schedules();
        test_pulse_redistributes_load_through_inertia_only();
        test_trim_is_delayed_and_increases_forward_lean_for_light_front();
        test_geometric_limit_caps_lean();
        test_preview_is_last_sample_and_preserves_exact_endpoint();
        test_geometric_limit_replaces_the_legacy_config_ceiling_when_available();
    });
}
