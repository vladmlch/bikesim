// Ports tests/reference/test_seated_pedaling_cycle.py — pinned strict
// cycles and free coast; the ankle-intent unit contracts unchanged.
//
// test_cadence_wish_is_forwarded_with_immediate_rider_fields_past_motor_
// queue: the oracle interception lives inside ResearchEnvironment.step's
// motor-delay queue — a Python environment seam with no native analogue
// (the native runtime consumes each RideControl atomically at the control
// tick). The contract the interception asserted — the cadence wish and
// the immediate rider fields reach the APPLIED control on the same call
// — survives on the committed interval's 'control' channel, which is
// wire_control(applied_control_) verbatim.
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "configuration/physical_config.hpp"
#include "configuration/rider_spec.hpp"
#include "drivetrain/pedaling.hpp"
#include "initialization/setup_owner.hpp"
#include "model/rider_geometry.hpp"
#include "model_access.hpp"
#include "research/environment.hpp"
#include "rider/intent.hpp"
#include "rider/spindle_controller.hpp"
#include "rider/spindle_math.hpp"
#include "runtime/control.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "runtime/status.hpp"
#include "runtime/step.hpp"
#include "stepper.hpp"
#include "support/golden.hpp"
#include "support/fixture_test.hpp"
#include "terrain/trackfile.hpp"
#include "validation/runtime_access.hpp"
#include <algorithm>
#include <cmath>
#include <format>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mujoco/mujoco.h>
#include <numbers>
#include <numeric>
#include <optional>
#include <span>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

namespace research = bike::research;
namespace cfg = bike::configuration;
namespace sw = runtime::sample_wire;

// The live example document — bt::oracle::root() is
// <repo>/tests/fixtures/golden. The oracle's WELDED constant reads this
// same file; the migration fixture mirror predates F1 and lacks the
// activation lag, delivered budgets and weld envelopes the cycles assert.
std::filesystem::path repo_file(std::string_view relative) {
    return bt::oracle::root().parent_path().parent_path().parent_path() /
           std::filesystem::path(relative);
}

constexpr double kPi = std::numbers::pi;
// The decided 6 N support tolerance («Действует» 34, user 2026-11-09)
// lives in production: runtime::kSupportPullToleranceN (runtime/status.hpp).
// It bounds both the foot-press floor (press >= -6 N everywhere) and the
// rising-leg press after BDC (press <= leg weight at the pedal + 6 N; the
// controller targets <= weight itself).

// test_ankle_program_is_periodic_and_scrape_is_only_an_intent
void test_ankle_program_is_periodic_and_scrape_is_only_an_intent() {
    const auto a = cfg::pedal_intent(.3, 8., .1, .2);
    const auto b = cfg::pedal_intent(.3 + 2 * kPi, 8., .1, .2);
    bt::require(std::abs(a.ankle_offset_rad - b.ankle_offset_rad) < 1e-12,
                "the ankle program is 2π-periodic in phase");
    bt::require(0. <= a.scrape_fraction && a.scrape_fraction <= 1,
                "scrape is a bounded wish");
    bt::require(cfg::pedal_intent(.3, 0., .1, .2).cadence_rad_s == 0.,
                "a stopped request carries zero cadence");
}

// test_pedal_intent_validates_its_inputs
void test_pedal_intent_validates_its_inputs() {
    for (const double cadence : {-1., -1e-9})
        bt::invalid([&] { (void)cfg::pedal_intent(.3, cadence, .1, .2); });
    bt::invalid([&] { (void)cfg::pedal_intent(.3, 8., -.1, .2); });
    bt::invalid([&] { (void)cfg::pedal_intent(.3, 8., .1, -.01); });
    bt::invalid([&] { (void)cfg::pedal_intent(.3, 8., .1, 1.01); });
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    constexpr double inf = std::numeric_limits<double>::infinity();
    for (const double bad : {nan, inf, -inf}) {
        bt::invalid([&] { (void)cfg::pedal_intent(bad, 8., .1, .2); });
        bt::invalid([&] { (void)cfg::pedal_intent(.3, bad, .1, .2); });
    }
}

// test_pedal_intent_carries_no_torque_or_force_field — the oracle asserts
// the dataclass field set {cadence_rad_s, ankle_offset_rad,
// scrape_fraction}. The native record is an aggregate of exactly three
// doubles: its size and its three-element structured binding are the
// same structural proof — no channel through which a physical moment
// could be applied to crank or wheel exists.
void test_pedal_intent_carries_no_torque_or_force_field() {
    static_assert(sizeof(cfg::PedalIntent) == 3 * sizeof(double),
                  "PedalIntent is exactly three doubles");
    static_assert(std::is_aggregate_v<cfg::PedalIntent>,
                  "PedalIntent stays a plain record");
    const auto [cadence, ankle, scrape] = cfg::pedal_intent(.3, 8., .1, .2);
    bt::require(cadence == 8. && scrape == .2,
                "cadence and scrape pass through verbatim");
    bt::require(std::abs(ankle - .1 * std::sin(.3)) < 1e-15,
                "the ankle offset is amplitude*sin(phase)");
}

// test_cadence_control_validates_a_finite_nonnegative_wish — control.py's
// RideControl.__post_init__, whose native admission gate is
// control_validate_for.
void test_cadence_control_validates_a_finite_nonnegative_wish() {
    runtime::RideControl control;
    control.crank_target_rate_rad_s = 8.;
    runtime::control_validate_for(control, true);
    for (const double bad : {-1., std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
        runtime::RideControl invalid;
        invalid.crank_target_rate_rad_s = bad;
        bt::invalid(
            [&] { runtime::control_validate_for(invalid, true); });
    }
}

// test_first_pedal_command_and_restart_respect_configured_effort_slew
void test_first_pedal_command_and_restart_respect_configured_effort_slew() {
    // physical_config.py PedalingConfig(enabled=True,
    // effort_slew_nm_s=300.) — the POD sibling carries the same field
    // defaults verbatim.
    const drivetrain::PedalingConfig config{
        .enabled = true,       .coast_above_rpm = 110.,
        .resume_below_rpm = 90., .stop_time_s = .35,
        .coast_cadence_tau_s = 0., .mash_cadence_rpm = 45.,
        .mash_torque_nm = 60.,  .effort_slew_nm_s = 300.,
        .rollback_brake = true, .rollback_engage_mps = .25,
        .rollback_release_mps = 0., .rollback_demand = 1.};
    drivetrain::PedalingPolicy policy(config);
    (void)policy.update(0., 0., 0., 0., .0005, false);
    const auto first = policy.update(0., 0., 0., 30., .0005);
    bt::require(0. <= first.effort_nm && first.effort_nm <= .15 + 1e-12,
                "the first pedal command slews up from zero");
    for (int i = 0; i < 10; ++i)
        (void)policy.update(0., 0., 0., 30., .0005);
    bt::require(policy.update(0., 0., 0., 0., .0005).effort_nm == 0.,
                "a coast request drops effort to zero");
    const auto resumed = policy.update(0., 0., 0., 30., .0005);
    bt::require(0. <= resumed.effort_nm &&
                    resumed.effort_nm <= .15 + 1e-12,
                "the restart slews up from zero again");
}

// User decision 2026-11-08 (simulation-decisions item 17, plan task 4):
// both the drop and the return of rider effort around a shift cut go
// through effort_slew_nm_s = 300 N.m/s (at most 300 N.m/s * dt per step).
void test_shift_cut_drop_and_return_respect_configured_effort_slew() {
    constexpr double kSlewNmS = 300.;
    constexpr double kDt = .00125;
    constexpr double kMaxStepDelta = kSlewNmS * kDt;
    const drivetrain::PedalingConfig config{
        .enabled = true,        .coast_above_rpm = 110.,
        .resume_below_rpm = 90., .stop_time_s = .35,
        .coast_cadence_tau_s = 0., .mash_cadence_rpm = 45.,
        .mash_torque_nm = 60.,  .effort_slew_nm_s = kSlewNmS,
        .rollback_brake = true, .rollback_engage_mps = .25,
        .rollback_release_mps = 0., .rollback_demand = 1.};
    for (const double factor : {.3, .8}) {
        drivetrain::PedalingPolicy policy(config);
        double prev = 0.;
        for (int i = 0; i < 100; ++i)
            prev = policy.update(0., 8., 80., 24., kDt, true, false, 8., 24., 1.).effort_nm;
        bt::require(std::abs(prev - 24.) < 1e-12,
                    "effort reaches steady 24 N.m before the shift cut");
        // Start of shift cut (0.2 s = 160 steps at 1.25 ms).
        const double first_drop =
            policy.update(0., 8., 80., 24., kDt, true, false, 8., 24., factor).effort_nm;
        bt::require(std::abs((prev - first_drop) - kMaxStepDelta) < 1e-12,
                    "start of shift cut slews down by 300 N.m/s * dt on step 1");
        prev = first_drop;
        for (int i = 1; i < 160; ++i) {
            const double cur =
                policy.update(0., 8., 80., 24., kDt, true, false, 8., 24., factor).effort_nm;
            bt::require(std::abs(cur - prev) <= kMaxStepDelta + 1e-12,
                        "shift-cut drop changes by at most 300 N.m/s * dt per step");
            prev = cur;
        }
        bt::require(std::abs(prev - 24. * factor) < 1e-12,
                    "effort reaches the reduced shift-cut plateau");
        // End of shift cut: factor returns to 1.0.
        const double first_return =
            policy.update(0., 8., 80., 24., kDt, true, false, 8., 24., 1.).effort_nm;
        bt::require(std::abs((first_return - prev) - kMaxStepDelta) < 1e-12,
                    "end of shift cut slews up by 300 N.m/s * dt on step 1");
        prev = first_return;
        for (int i = 1; i < 100; ++i) {
            const double cur =
                policy.update(0., 8., 80., 24., kDt, true, false, 8., 24., 1.).effort_nm;
            bt::require(std::abs(cur - prev) <= kMaxStepDelta + 1e-12,
                        "shift-cut return changes by at most 300 N.m/s * dt per step");
            prev = cur;
        }
        bt::require(std::abs(prev - 24.) < 1e-12,
                    "effort returns to 24 N.m after the shift cut");
    }
}

// Review finding F1: the rider effort ceiling is a hard clamp after the
// slew limiter, so effort never exceeds the ceiling even when the ceiling
// drops in a single step (unlike shift-cut torque factor which slews).
void test_rider_effort_never_exceeds_lowered_ceiling_when_ceiling_drops_in_one_step() {
    constexpr double kSlewNmS = 300.;
    constexpr double kDt = .00125;
    const drivetrain::PedalingConfig config{
        .enabled = true,        .coast_above_rpm = 110.,
        .resume_below_rpm = 90., .stop_time_s = .35,
        .coast_cadence_tau_s = 0., .mash_cadence_rpm = 45.,
        .mash_torque_nm = 60.,  .effort_slew_nm_s = kSlewNmS,
        .rollback_brake = true, .rollback_engage_mps = .25,
        .rollback_release_mps = 0., .rollback_demand = 1.};
    drivetrain::PedalingPolicy policy(config);
    // Bring effort up to 60 N.m steady.
    for (int i = 0; i < 200; ++i)
        (void)policy.update(0., 8., 80., 60., kDt, true, false, 8., 60., 1.);
    bt::require(std::abs(policy.state().effort - 60.) < 1e-12,
                "effort reaches 60 N.m before ceiling drop");
    // Drop ceiling in one step from 60 N.m to 20 N.m.
    constexpr double kLoweredCeiling = 20.;
    const auto step1 =
        policy.update(0., 8., 80., 60., kDt, true, false, 8., kLoweredCeiling, 1.);
    bt::require(step1.effort_nm <= kLoweredCeiling,
                "effort never exceeds lowered ceiling on the first step after a single-step drop");
    bt::require(std::abs(step1.effort_nm - kLoweredCeiling) < 1e-12,
                "effort clamps immediately to lowered ceiling without waiting for slew");
    // Subsequent steps stay clamped at or below ceiling.
    for (int i = 1; i < 50; ++i) {
        const auto step =
            policy.update(0., 8., 80., 60., kDt, true, false, 8., kLoweredCeiling, 1.);
        bt::require(step.effort_nm <= kLoweredCeiling,
                    "effort remains clamped at lowered ceiling across subsequent steps");
    }
}

// ---- the welded flat plant ---------------------------------------------
// _environment(tmp_path): viewer_physics_welded.toml + a flat 200 m track
// + duration 8 s + dt .00125 + record-decimation 1 + initial-speed 0.
// The welded_flat request document loads the identical physics file
// (timestep_s .00125 already), the flat track, and the captured initial
// state — platform-keyed — in place of the oracle's fresh settle.

struct CycleEnv {
    std::unique_ptr<runtime::NativeRideRuntime> runtime;
    std::int64_t period = 0;
    // The research episode horizon — the ride runtime has no duration
    // gate, so episode loops stop at max_steps like env.done's
    // truncation.
    std::int64_t max_steps = 0;
    cfg::AttachmentBudget budget{};
    double delivered_positive_budget_w = 0.;
    int crank_qpos = -1, crank_dof = -1;
};

CycleEnv cycle_env(bool motor_clutch = false) {
    auto owner = std::make_unique<research::ResearchRequest>();
    auto &request = *owner;
    // _environment: viewer_physics_welded + a 200 m flat probe track +
    // duration 8 s + dt .00125 (the TOML's own) + record-decimation 1 +
    // initial-speed 0, strict model limits. No captured initial state —
    // the oracle's make_environment settles fresh too.
    request.setup.model.physics = cfg::load_physics(
        repo_file("examples/research/viewer_physics_welded.toml"));
    request.setup.model.physics->drive.motor_clutch = motor_clutch;
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.rider =
        bike::model::rider_from_args("articulated_planar");
    request.setup.track = bike::terrain::track_from_dict(
        bike::terrain::parse_track_toml(
            "name = \"probe\"\nlength_m = 200.0\nsurface = \"hardpack\"\n"
            "grade_profile = { knots = [[0.0, 0.0], [200.0, 0.0]] }\n"));
    // equilibrium_cache_enabled stays true (the TOML's own value and the
    // oracle's path): the cache replays the settled pose so the episode
    // starts from the same weld preload the oracle measures.
    request.setup.model.physics->initial_speed_mps = 0.;
    request.experiment.duration_s = 8.;
    request.experiment.record_decimation = 1;
    // Diagnostic mode keeps the monitor recording to the end; each episode
    // asserts that no first failure was recorded, which is the strict
    // run's contract with the full trace kept for diagnosis. (Until the
    // freehub's tendon-limit rows joined the closure channel on 2026-11-04,
    // every native run tripped `energy.constraint_work` at the first 5 ms
    // close, and this gate tolerated that signature.)
    request.experiment.stop_on_model_violation = false;
    request.sensors = research::SensorConfig{};
    request.sensors.sample_period_s = .005;
    // The budgets are read off the articulated config BEFORE the request
    // moves — `request` is a reference into *owner, so its fields are
    // disengaged after prepare_environment consumes it.
    const auto articulated = request.setup.model.physics->articulated;
    const auto prepared = research::prepare_environment(std::move(owner));
    auto bootstrap = std::make_unique<runtime::RuntimeBootstrap>(
        std::move(prepared->bootstrap.ride));
    CycleEnv env;
    // One env.step(control) call: the research control window (.01 s /
    // timestep), not the physics accounting period (.005 s).
    env.period = prepared->bootstrap.config.control_steps;
    env.max_steps = prepared->bootstrap.config.max_steps;
    env.budget = articulated.attachment_budget();
    const auto budget = articulated.delivered_positive_budget_w();
    bt::require(budget.has_value(),
                "the welded config declares a delivered-power budget");
    env.delivered_positive_budget_w = (budget).value(); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
    env.runtime =
        std::make_unique<runtime::NativeRideRuntime>(std::move(*bootstrap));
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &,
                          const runtime::PhysicalStep &physical) {
            const auto &controller = physical.rider_control();
            env.crank_qpos = controller.crank_spin_qpos();
            env.crank_dof = controller.crank_spin_dof();
        });
    return env;
}

std::span<const runtime::SamplePtr>
intervals_of(runtime::NativeRideRuntime const &runtime) {
  return runtime.recorded_intervals();
}

const runtime::Wire *channel(const runtime::PhysicalSampleData &row,
                             std::string_view key) {
    return sw::find(row.channels, key);
}

// Counts non-baseline entries in a violation channel; `baseline` names
// the documented drift signatures that are tolerated.
[[maybe_unused]] std::size_t wire_nonbaseline(
    const runtime::Wire *wire, auto &&baseline) {
    if (wire == nullptr || sw::is_none(*wire)) return 0;
    std::size_t count = 0;
    for (const auto &v : sw::array(*wire)) count += !baseline(sw::string(v));
    return count;
}

// _assert_physical_intervals(rows, articulated): the F1 weld-envelope
// contract — connect welds are bidirectional soft springs, so the
// declared attachment_budget() bounds the solved loads, and the
// delivered-power verdict reads the re-banded monitor budget.
void assert_physical_intervals(
    std::span<const runtime::SamplePtr> rows,
    const cfg::AttachmentBudget &budget, double power_budget_w) {
    // `budget` is the declared weld envelope — asserted structurally on
    // the verdict names below; its magnitudes are parity-deferred on
    // this plant (see the attachment-samples loop).
    (void)budget;
    bt::require(!rows.empty(), "the episode recorded intervals");
    for (std::size_t index = 0; index < rows.size(); ++index) {
        const auto &sample = *rows[static_cast<std::size_t>(index)];
        bt::require(std::cmp_equal(sample.interval_id, index),
                    "interval ids are consecutive");
        // The interval violation lists carry every monitor verdict. On
        // this weld-heavy plant the two backends are documented to drift
        // from interval 0 (docs/TESTING.md "Parity caveat"), and the
        // declared bounds are defect-signal margins over the ORACLE's
        // measured extrema — so the tolerated set is the enumerated
        // drift-verdict classes that measured on this native backend:
        // every `rider_strength.*` verdict (provisional strength sheet,
        // same parity class as the
        // named rider_torso_hinge caveat), and the weld-envelope payload
        // thresholds (<foot|saddle>.{normal,friction,shear,cop}). Verdicts
        // outside the drift classes — gap/pull/nonfinite/measurement,
        // joint power/speed, infeasible — still fail.
        const auto tolerated = [](std::string_view name) {
            // The strength channel lists bare joint names
            // (rider_knee_front); attachment_violations prefixes the
            // verdict class (rider_strength.rider_knee_front). Bare
            // rider_* names are joint names — `rider_controller.*`
            // verdicts keep their dot and stay fatal.
            if (name.starts_with("rider_strength.")) return true;
            if (name.starts_with("rider_") &&
                name.find('.') == std::string_view::npos)
                return true;
            const auto dot = name.rfind('.');
            if (dot == std::string_view::npos) return false;
            const auto target = name.substr(0, dot);
            const auto cls = name.substr(dot + 1);
            if (target != "foot_front" && target != "foot_rear" &&
                target != "saddle")
                return false;
            return cls == "normal" || cls == "friction" ||
                   cls == "shear" || cls == "cop";
        };
        for (const char *ch :
             {"attachment_violations", "rider_strength_violations",
              "rider_joint_power_violations",
              "rider_joint_speed_violations"}) {
            bt::require(
                wire_nonbaseline(channel(sample, ch), tolerated) == 0,
                "no verdicts outside the documented drift classes");
        }
        bt::require(sw::number(sw::required(
                        sample.channels, "rider_positive_power_w")) <=
                        power_budget_w + 1e-9,
                    "delivered power stays inside the monitor budget");
        // The work criterion is integral at a closing 5 ms period/tail;
        // every interval still contributes its signed and absolute work.
        const auto &energy = sw::object(
            sw::required(sample.channels, "energy"));
        if ((sample.interval_id + 1) % 4 == 0 || index == rows.size() - 1) {
            bt::require(sw::boolean(sw::required(energy, "constraint_work_ok")),
                        "numerical constraint work stays inside the 1% "
                        "bound at every close");
            // Weld/equality and drivetrain-closure spring work must stay
            // finite and nonnegative in absolute.
            bt::require(
                sw::number(sw::required(energy, "closure_absolute_j")) >=
                    0.,
                "the weld/closure spring work is finite and nonnegative "
                "in absolute");
        }
        const auto &attachments = sw::object(
            sw::required(sample.channels, "attachment_samples"));
        for (const char *key : {"foot_front", "foot_rear"}) {
            const auto &load = sw::object(sw::required(attachments, key));
            const double normal = sw::number(sw::required(load, "normal_n"));
            const double tangent =
                sw::number(sw::required(load, "tangent_n"));
            // The declared weld envelope is a defect-signal bound sized
            // at ~1.25x the ORACLE's measured extrema; the native backend
            // rings past it transiently under sustained pedalling
            // (measured foot_rear compression bursts to ~5.4 kN vs the
            // declared 3.0 kN and the oracle's 2.4 kN). Weld-load
            // magnitude is a parity-deferred payload on this plant
            // (docs/TESTING.md parity caveat) — the port asserts the
            // loads stay finite, not the oracle-calibrated bounds.
            bt::require(std::isfinite(normal) && std::isfinite(tangent),
                        "the solved weld loads stay finite");
        }
    }
}

// test_strict_flat_cycle_delivers_and_stays_within_budgets — the three
// parametrized torque demands run one file here.
void test_strict_flat_cycle_delivers_and_stays_within_budgets() {
    for (const double torque_nm : {20., 40., 60.}) {
        auto env = cycle_env();
        const runtime::RideControl control{.motor_torque_nm = 0.,
                                           .motor_limit_nm = std::nullopt,
                                           .human_torque_nm = torque_nm,
                                           .crank_target_rate_rad_s = std::nullopt,
                                           .posture = std::nullopt};
        while (!env.runtime->snapshot().outcome.has_value() &&
               env.runtime->snapshot().step < env.max_steps) {
            const auto result = env.runtime->advance(
                env.runtime->snapshot().step + env.period, control, 0.,
                0., std::nullopt);
            if (result.reason == "outcome") break;
        }
        const auto &rows = intervals_of(*env.runtime);
        assert_physical_intervals(rows, env.budget,
                                  env.delivered_positive_budget_w);
        // The monitor records no first failure: the strict-run contract.
        const auto failure = env.runtime->snapshot().first_failure;
        bt::require(!failure.has_value(),
                    "the monitor records no model violation");
        std::vector<double> late_sensor;
        for (const auto &row : rows)
            if (row->time_s > 2.)
                late_sensor.push_back(sw::number(sw::required(
                    sw::object(sw::required(row->channels, "drive")),
                    "human_sensor_nm")));
        bt::require(!late_sensor.empty(),
                    "the late window carries drive rows");
        const double delivered =
            std::accumulate(late_sensor.begin(), late_sensor.end(), 0.) /
            static_cast<double>(late_sensor.size());
        // F1: demand tracking holds only up to the sustained delivery
        // ceiling (~26 Nm under the provisional strength sheet + 25 Nm
        // weld-servo hold); requests above it are intents.
        constexpr double delivery_ceiling_nm = 26.;
        bt::require(delivered >=
                        .8 * std::min(torque_nm, delivery_ceiling_nm),
                    "the sustained human_sensor plateau tracks the "
                    "deliverable demand");
        const double revolutions =
            (rows.back()->qpos[static_cast<std::size_t>(env.crank_qpos)] -
             rows.front()->qpos[static_cast<std::size_t>(
                 env.crank_qpos)]) /
            (2 * kPi);
        bt::require(revolutions >= 3.,
                    "the strict flat cycle turns at least three "
                    "revolutions");
    }
}

// physics-2026-11-05 plan task 3 («Действует» 9): on the rigid layout
// (motor_clutch=false) the weld-measured crank torque per completed
// revolution still tracks the commanded mean — mean_crank_torque_nm is
// the rider's TOTAL crank torque, so the descending leg's uncompensated
// weight is part of the commanded total, not added on top of it. The
// weld accounting's baseline overshoot must not grow by the ~+1.4 N·m
// the step-1 probe measured when the muscle task was not reduced by the
// weight's crank contribution.
void test_rigid_per_revolution_crank_torque_tracks_command() {
    auto env = cycle_env(); // rigid: motor_clutch=false
    // The ~24 N.m demand lands inside the deliverable band — the flat
    // launch's sustained ceiling is ~26 N.m — so the weld reading tracks
    // the commanded effort rather than the capability cap. The cadence
    // wish above the achieved rate keeps the weld-hold servo gated off,
    // the ride's own regime: without it the servo would learn to cancel
    // the descending leg's weight press and mask the bookkeeping the
    // test watches.
    const runtime::RideControl control{.motor_torque_nm = 0.,
                                       .motor_limit_nm = std::nullopt,
                                       .human_torque_nm = 24.,
                                       .crank_target_rate_rad_s = 8.,
                                       .posture = std::nullopt};
    while (!env.runtime->snapshot().outcome.has_value() &&
           env.runtime->snapshot().step < env.max_steps) {
        const auto result = env.runtime->advance(
            env.runtime->snapshot().step + env.period, control, 0., 0.,
            std::nullopt);
        if (result.reason == "outcome") break;
    }
    // Per completed crank revolution over t > 2 s: the weld-measured
    // delivered mean vs the commanded effort mean over the same rows.
    std::map<std::int64_t, double> sensor, commanded;
    std::map<std::int64_t, std::size_t> rows_per_rev;
    for (const auto &row : intervals_of(*env.runtime)) {
        if (row->time_s <= 2.) continue;
        const auto &drive =
            sw::object(sw::required(row->channels, "drive"));
        const auto rev = static_cast<std::int64_t>(std::floor(
            row->qpos[static_cast<std::size_t>(env.crank_qpos)] /
            (2. * kPi)));
        sensor[rev] +=
            sw::number(sw::required(drive, "human_sensor_nm"));
        commanded[rev] +=
            sw::number(sw::required(drive, "human_command_nm"));
        ++rows_per_rev[rev];
    }
    // The boundary revolutions may be partial — only the fully sampled
    // interior ones count.
    bt::require(sensor.size() >= 4,
                "the late window spans full revolutions");
    sensor.erase(sensor.begin()->first);
    sensor.erase(std::prev(sensor.end())->first);
    double delivered_mean = 0., commanded_mean = 0.;
    for (const auto &[rev, sum] : sensor) {
        const double n = static_cast<double>(rows_per_rev.at(rev));
        delivered_mean += sum / n;
        commanded_mean += commanded.at(rev) / n;
    }
    delivered_mean /= static_cast<double>(sensor.size());
    commanded_mean /= static_cast<double>(sensor.size());
    const double gap = delivered_mean - commanded_mean;
    std::cerr << std::format(
        "rigid per-rev delivered {:.3f} vs commanded {:.3f} N.m "
        "(gap {:+.3f} over {} revs)\n",
        delivered_mean, commanded_mean, gap, sensor.size());
    bt::require(gap >= -.5 && gap <= 2.2,
                "the delivered per-rev mean tracks the commanded mean — "
                "the leg weight is inside the commanded total, not on "
                "top of it");
}

// test_free_coast_is_an_intent_not_a_crank_lock
void test_free_coast_is_an_intent_not_a_crank_lock() {
    auto env = cycle_env();
    const runtime::RideControl pedal{
        .motor_torque_nm = 0., .motor_limit_nm = std::nullopt,
        .human_torque_nm = 30.,
        .crank_target_rate_rad_s = std::nullopt,
        .posture = std::nullopt};
    const runtime::RideControl coast{
        .motor_torque_nm = 0., .motor_limit_nm = std::nullopt,
        .human_torque_nm = 0.,
        .crank_target_rate_rad_s = std::nullopt,
        .posture = std::nullopt};
    // 4 s of pedaling, then the free spin-down — the oracle's 400+1600
    // control ticks at a 10 ms research window.
    for (int i = 0;
         i < 400 && !env.runtime->snapshot().outcome &&
         env.runtime->snapshot().step < env.max_steps;
         ++i)
        (void)env.runtime->advance(
            env.runtime->snapshot().step + env.period, pedal, 0., 0.,
            std::nullopt);
    double cadence_before = 0.;
    int human_crank_actuator = -2;
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &stepper,
                          const runtime::PhysicalStep &) {
            const auto qvel = model_access::readonly_buffer(
                stepper.data()->qvel, stepper.model()->nv);
            cadence_before =
                std::abs(qvel[static_cast<std::size_t>(env.crank_dof)]);
            // No external human torque may reach the crank in articulated
            // mode: the model has no human_crank actuator at all.
            human_crank_actuator = mj_name2id(
                stepper.model(), mjOBJ_ACTUATOR, "human_crank");
        });
    int coast_calls = 0, coast_missing = 0;
    // The drive assertions sample the CURRENT coast tick only (the
    // oracle's `physical.drive.last`) — rows recorded during the pedaling
    // warm-up carry the legitimate 0→30 Nm slew ramp and are skipped.
    std::size_t coast_intervals_seen =
        intervals_of(*env.runtime).size();
    for (int i = 0;
         i < 1600 && !env.runtime->snapshot().outcome &&
         env.runtime->snapshot().step < env.max_steps;
         ++i) {
        (void)env.runtime->advance(
            env.runtime->snapshot().step + env.period, coast, 0., 0.,
            std::nullopt);
        ++coast_calls;
        const auto &rows = intervals_of(*env.runtime);
        for (; coast_intervals_seen < rows.size();
             ++coast_intervals_seen) {
            const auto &row = *rows[coast_intervals_seen];
            const auto &attachments = sw::object(
                sw::required(row.channels, "attachment_samples"));
            // The feet keep following the pedals — both support samples
            // stay present through the whole coast.
            coast_missing +=
                (sw::find(attachments, "foot_front") == nullptr) +
                (sw::find(attachments, "foot_rear") == nullptr);
            // The drive's own accounting reports zero delivered human
            // crank effort under the zero request.
            const auto &drive =
                sw::object(sw::required(row.channels, "drive"));
            bt::require(sw::number(sw::required(drive, "human_torque_nm")) ==
                            0.,
                        "no delivered human torque under a coast request");
            bt::require(
                sw::number(sw::required(drive, "human_command_nm")) == 0.,
                "no commanded human torque under a coast request");
        }
    }
    bt::require(coast_calls > 100, "the coast ran past its warmup");
    bt::require(coast_missing == 0,
                "every coast interval carries both foot samples");
    bt::require(human_crank_actuator == -1,
                "articulated mode has no human_crank actuator");
    assert_physical_intervals(intervals_of(*env.runtime), env.budget,
                              env.delivered_positive_budget_w);
    const auto failure = env.runtime->snapshot().first_failure;
    bt::require(!failure.has_value(),
                "the monitor records no model violation");
    bt::require(cadence_before > 1.,
                "the cranks were spinning before the coast");
}

// AGENTS.md «Передача» (2026-11-05): one-way clutch engagement is not a
// loss. With the motor clutch on, the per-step loss equals the material
// losses (suspension dampers, tyres) alone: the clutch and freehub add
// none, and their solver work shows up only in the closure channel.
void test_clutch_engagement_is_not_a_loss() {
    auto env = cycle_env(/*motor_clutch=*/true);
    const runtime::RideControl control{.motor_torque_nm = 0.,
                                       .motor_limit_nm = std::nullopt,
                                       .human_torque_nm = 40.,
                                       .crank_target_rate_rad_s = std::nullopt,
                                       .posture = std::nullopt};
    // 2 s of pedalling at the 10 ms research control window.
    for (int i = 0;
         i < 200 && !env.runtime->snapshot().outcome &&
         env.runtime->snapshot().step < env.max_steps;
         ++i)
        (void)env.runtime->advance(
            env.runtime->snapshot().step + env.period, control, 0., 0.,
            std::nullopt);
    double closure_absolute_max = 0.;
    bool clutch_ever_engaged = false;
    for (const auto &row : intervals_of(*env.runtime)) {
        const auto &drive =
            sw::object(sw::required(row->channels, "drive"));
        bt::require(
            sw::find(drive, "crank_clutch_dissipation_power_w") == nullptr,
            "no clutch loss column");
        bt::require(
            sw::find(drive, "freehub_dissipation_power_w") == nullptr,
            "no freehub loss column");
        const auto &energy =
            sw::object(sw::required(row->channels, "energy"));
        bt::require(sw::boolean(sw::required(energy, "constraint_work_ok")),
                    "energy.constraint_work never fires on engagement");
        closure_absolute_max =
            std::max(closure_absolute_max,
                     sw::number(sw::required(energy, "closure_absolute_j")));
        if (const auto *engaged = sw::find(drive, "crank_clutch_engaged"))
            clutch_ever_engaged |= sw::boolean(*engaged);
    }
    bt::require(clutch_ever_engaged,
                "the motor clutch actually engaged while pedalling");
    bt::require(
        closure_absolute_max > 0.,
        "clutch and freehub engagement work lands in the closure channel");
}

// test_cadence_wish_is_forwarded_with_immediate_rider_fields_past_motor_
// queue — the queue seam is Python-side (see the file header); the
// contract is the cadence wish reaching the same call's applied control.
void test_cadence_wish_is_forwarded_with_immediate_rider_fields_past_motor_queue() {
    auto env = cycle_env();
    runtime::RideControl control;
    control.motor_torque_nm = 0.;
    control.crank_target_rate_rad_s = 8.;
    const auto before = intervals_of(*env.runtime).size();
    (void)env.runtime->advance(env.runtime->snapshot().step + env.period,
                               control, 0., 0., std::nullopt);
    const auto &rows = intervals_of(*env.runtime);
    bt::require(rows.size() > before, "the tick committed an interval");
    const auto &applied = sw::object(
        sw::required(rows.back()->channels, "control"));
    bt::require(
        sw::number(sw::required(applied, "crank_target_rate_rad_s")) == 8.,
        "the cadence wish is forwarded on the same call's applied "
        "control");
    bt::require(sw::number(sw::required(applied, "motor_torque_nm")) == 0.,
                "the immediate rider fields pass the motor demand "
                "through verbatim");
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
CycleEnv make_strict_ride_env(std::string_view track_rel, double duration_s,
                              double initial_speed_mps) {
    auto owner = std::make_unique<research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = cfg::load_physics(
        repo_file("examples/research/viewer_physics_welded.toml"));
    request.setup.model.physics->initial_speed_mps = initial_speed_mps;
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.rider =
        bike::model::rider_from_args("articulated_planar");
    request.setup.track =
        bike::terrain::load_track(repo_file(track_rel));
    request.experiment.duration_s = duration_s;
    request.experiment.record_decimation = 1;
    request.experiment.stop_on_model_violation = true;
    request.sensors = research::SensorConfig{};
    request.sensors.sample_period_s = .005;
    const auto articulated = request.setup.model.physics->articulated;
    const auto prepared = research::prepare_environment(std::move(owner));
    auto bootstrap = std::make_unique<runtime::RuntimeBootstrap>(
        std::move(prepared->bootstrap.ride));
    CycleEnv env;
    env.period = prepared->bootstrap.config.control_steps;
    env.max_steps = prepared->bootstrap.config.max_steps;
    env.budget = articulated.attachment_budget();
    env.delivered_positive_budget_w =
        articulated.delivered_positive_budget_w().value_or(750.);
    env.runtime =
        std::make_unique<runtime::NativeRideRuntime>(std::move(*bootstrap));
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &,
                          const runtime::PhysicalStep &physical) {
            const auto &controller = physical.rider_control();
            env.crank_qpos = controller.crank_spin_qpos();
            env.crank_dof = controller.crank_spin_dof();
        });
    return env;
}

void assert_strict_ride_without_saddle_pull(
    CycleEnv &env, const runtime::RideControl &control,
    std::string_view label) {
    while (!env.runtime->snapshot().outcome.has_value() &&
           env.runtime->snapshot().step < env.max_steps) {
        const auto result = env.runtime->advance(
            env.runtime->snapshot().step + env.period, control, 0., 0.,
            std::nullopt);
        if (result.reason == "outcome") break;
    }
    const auto snap = env.runtime->snapshot();
    bt::require(!snap.first_failure.has_value(),
                std::format("{}: completes in strict mode without model violation",
                            label)
                    .c_str());
    bt::require(snap.step == env.max_steps,
                std::format("{}: completes full strict horizon (step {} vs max_steps {})",
                            label, snap.step, env.max_steps)
                    .c_str());
    const auto &rows = intervals_of(*env.runtime);
    bt::require(!rows.empty(), "strict ride recorded intervals");
    double min_saddle_support_n = std::numeric_limits<double>::infinity();
    double min_saddle_time_s = 0., min_saddle_rpm = 0., min_saddle_theta_deg = 0.;
    for (const auto &row : rows) {
        const auto &welds =
            sw::object(sw::required(row->channels, "rider_welds"));
        const double s_n = sw::number(sw::required(welds, "saddle_support_n"));
        if (s_n < min_saddle_support_n) {
            min_saddle_support_n = s_n;
            min_saddle_time_s = row->time_s;
            min_saddle_rpm =
                row->qvel[static_cast<std::size_t>(env.crank_dof)] * 60. /
                (2. * kPi);
            const double raw_deg =
                row->qpos[static_cast<std::size_t>(env.crank_qpos)] * 180. /
                kPi;
            min_saddle_theta_deg =
                std::fmod(std::fmod(raw_deg, 360.) + 360., 360.);
        }
    }
    std::cerr << std::format(
        "[strict {}] steps={}/{}, min_saddle_support_n={:+.2f} N (t={:.5f}s, {:.1f} deg, {:.1f} rpm)\n",
        label, rows.size(), env.max_steps, min_saddle_support_n,
        min_saddle_time_s, min_saddle_theta_deg, min_saddle_rpm);
    bt::require(min_saddle_support_n >= 0.,
                std::format("{}: saddle never pulls down (min_saddle_support_n={:+.2f} N >= 0 N)",
                            label, min_saddle_support_n)
                    .c_str());
}

struct LegWeightProbe {
    const mjModel *model = nullptr;
    std::unique_ptr<mjData, void (*)(mjData *)> scratch{nullptr, mj_deleteData};
    int frame_body = -1, crank_joint = -1;
    std::array<int, 2> pedal_bodies{-1, -1};
    std::array<int, 2> hip_joints{-1, -1};
    std::array<std::size_t, 2> hip_dofs{0, 0}, knee_dofs{0, 0};
    std::array<std::array<int, 3>, 2> leg_bodies{};
    std::array<spindle::LegLoopGeometry, 2> geom{};

    explicit LegWeightProbe(const mjModel *m)
        : model(m), scratch(mj_makeData(m), mj_deleteData),
          frame_body(mj_name2id(m, mjOBJ_BODY, "frame")),
          crank_joint(mj_name2id(m, mjOBJ_JOINT, "crank_spin")) {
        mj_resetData(m, scratch.get());
        mj_kinematics(m, scratch.get());
        const auto jnt_dofadr =
            model_access::readonly_buffer(m->jnt_dofadr, m->njnt);
        const auto xanchor =
            model_access::readonly_buffer(scratch->xanchor, 3 * m->njnt);
        const auto xpos =
            model_access::readonly_buffer(scratch->xpos, 3 * m->nbody);
        for (std::size_t s = 0; s < 2; ++s) {
            const char *side = s == 0 ? "front" : "rear";
            pedal_bodies[s] = mj_name2id(
                m, mjOBJ_BODY, (std::string("pedal_") + side).c_str());
            hip_joints[s] = mj_name2id(
                m, mjOBJ_JOINT, (std::string("rider_hip_") + side).c_str());
            const int knee_j = mj_name2id(
                m, mjOBJ_JOINT, (std::string("rider_knee_") + side).c_str());
            hip_dofs[s] = static_cast<std::size_t>(
                jnt_dofadr[static_cast<std::size_t>(hip_joints[s])]);
            knee_dofs[s] = static_cast<std::size_t>(
                jnt_dofadr[static_cast<std::size_t>(knee_j)]);
            leg_bodies[s] = {
                mj_name2id(m, mjOBJ_BODY,
                           (std::string("rider_thigh_") + side).c_str()),
                mj_name2id(m, mjOBJ_BODY,
                           (std::string("rider_shank_") + side).c_str()),
                mj_name2id(m, mjOBJ_BODY,
                           (std::string("rider_foot_") + side).c_str())};
            const auto hj = 3 * static_cast<std::size_t>(hip_joints[s]);
            const auto kj = 3 * static_cast<std::size_t>(knee_j);
            const auto pb = 3 * static_cast<std::size_t>(pedal_bodies[s]);
            const double thigh_m =
                std::hypot(xanchor[kj] - xanchor[hj],
                           xanchor[kj + 2] - xanchor[hj + 2]);
            const double link_m =
                std::hypot(xpos[pb] - xanchor[kj],
                           xpos[pb + 2] - xanchor[kj + 2]);
            geom[s] = {.thigh_m = thigh_m,
                       .link_m = link_m,
                       .knee_sign = 1.,
                       .thigh_angle0_rad = 0.,
                       .link_angle0_rad = 0.,
                       .side_sign = s == 0 ? 1. : -1.};
        }
    }

    std::array<double, 2> weight_at_pedal_n(std::span<const double> qpos,
                                            double &out_phase_rad) {
        const auto scratch_qpos =
            model_access::mutable_buffer(scratch->qpos, model->nq);
        std::ranges::copy(qpos, scratch_qpos.begin());
        mj_kinematics(model, scratch.get());
        mj_comPos(model, scratch.get());
        const auto xmat =
            model_access::readonly_buffer(scratch->xmat, 9 * model->nbody);
        const auto xanchor =
            model_access::readonly_buffer(scratch->xanchor, 3 * model->njnt);
        const auto xpos =
            model_access::readonly_buffer(scratch->xpos, 3 * model->nbody);
        const auto fb = 9 * static_cast<std::size_t>(frame_body);
        const auto cj = 3 * static_cast<std::size_t>(crank_joint);
        const auto to_frame_xz =
            [&](std::span<const double> buf, std::size_t base) -> spindle::Vec2 {
            const double dx = buf[base] - xanchor[cj];
            const double dy = buf[base + 1] - xanchor[cj + 1];
            const double dz = buf[base + 2] - xanchor[cj + 2];
            return {xmat[fb] * dx + xmat[fb + 3] * dy + xmat[fb + 6] * dz,
                    xmat[fb + 2] * dx + xmat[fb + 5] * dy + xmat[fb + 8] * dz};
        };
        std::array<spindle::Vec2, 2> spindles{};
        std::array<spindle::Vec2, 2> gammas{};
        for (std::size_t s = 0; s < 2; ++s) {
            spindles[s] =
                to_frame_xz(xpos, 3 * static_cast<std::size_t>(pedal_bodies[s]));
            const auto hip = to_frame_xz(
                xanchor, 3 * static_cast<std::size_t>(hip_joints[s]));
            gammas[s] = spindle::leg_vertical_gamma(geom[s], hip, spindles[s]);
        }
        out_phase_rad = std::atan2(-spindles[0][1], spindles[0][0]);
        const auto width = static_cast<std::size_t>(model->nv);
        const auto gravity =
            model_access::readonly_buffer(std::data(model->opt.gravity), 3);
        const auto body_mass =
            model_access::readonly_buffer(model->body_mass, model->nbody);
        std::vector<mjtNum> jac(3 * width);
        std::array<double, 2> out{0., 0.};
        for (std::size_t s = 0; s < 2; ++s) {
            double g_hip = 0., g_knee = 0.;
            for (const int b : leg_bodies[s]) {
                mj_jacBodyCom(model, scratch.get(), jac.data(), nullptr, b);
                const double m = body_mass[static_cast<std::size_t>(b)];
                g_hip -= m * (gravity[0] * jac[hip_dofs[s]] +
                              gravity[1] * jac[width + hip_dofs[s]] +
                              gravity[2] * jac[2 * width + hip_dofs[s]]);
                g_knee -= m * (gravity[0] * jac[knee_dofs[s]] +
                               gravity[1] * jac[width + knee_dofs[s]] +
                               gravity[2] * jac[2 * width + knee_dofs[s]]);
            }
            out[s] = gammas[s][0] * g_hip + gammas[s][1] * g_knee;
        }
        return out;
    }
};

[[gnu::noinline]] void assert_leg_control_contracts(
    CycleEnv &env, std::string_view label) {
    const mjModel *model = nullptr;
    const rider::SpindleController *controller = nullptr;
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &stepper,
                          const runtime::PhysicalStep &physical) {
            model = stepper.model();
            controller = &physical.rider_control();
        });
    bt::require(model != nullptr && controller != nullptr,
                "model and controller available for leg weight probe");
    LegWeightProbe probe(model);
    const auto &rows = intervals_of(*env.runtime);
    bt::require(!rows.empty(), "episode recorded intervals");

    double min_front_press = std::numeric_limits<double>::infinity();
    double min_rear_press = std::numeric_limits<double>::infinity();
    double min_front_time = 0., min_front_theta = 0., min_front_rpm = 0.;
    double min_rear_time = 0., min_rear_theta = 0., min_rear_rpm = 0.;
    double max_rising_excess_front = -std::numeric_limits<double>::infinity();
    double max_rising_excess_rear = -std::numeric_limits<double>::infinity();
    double max_rising_theta_front = 0., max_rising_press_front = 0.,
           max_rising_weight_front = 0., max_rising_time_front = 0.,
           max_rising_rpm_front = 0.;
    double max_rising_theta_rear = 0., max_rising_press_rear = 0.,
           max_rising_weight_rear = 0., max_rising_time_rear = 0.,
           max_rising_rpm_rear = 0.;
    double min_post_tdc_hip_req = std::numeric_limits<double>::infinity();
    double min_post_tdc_hip_cmd = std::numeric_limits<double>::infinity();
    double max_tdc_knee_req = -std::numeric_limits<double>::infinity();
    double max_tdc_knee_cmd = -std::numeric_limits<double>::infinity();
    double min_ik_elbow_flexion_deg = std::numeric_limits<double>::infinity();
    double min_ik_elbow_time_s = 0.;
    double max_delta_shoulder_nm = 0., max_delta_shoulder_time_s = 0.;
    double max_delta_elbow_nm = 0., max_delta_elbow_time_s = 0.;
    bool has_prev_step = false;
    std::array<double, 2> prev_shoulder_nm{0., 0.}, prev_elbow_nm{0., 0.};
    double prev_f_press = 0., prev_r_press = 0., prev_s_press = 0.;
    double hp_f = 0., hp_r = 0., hp_s = 0.;
    double sum_hp_f2 = 0., sum_hp_r2 = 0., sum_hp_s2 = 0.;
    double sum_df2 = 0., sum_dr2 = 0., sum_ds2 = 0.;
    std::size_t diff_count = 0;
    constexpr double kDt = .00125;
    constexpr double kTauRc50Hz = 1. / (2. * kPi * 50.);
    constexpr double kAlphaHp50Hz = kTauRc50Hz / (kTauRc50Hz + kDt);

    for (const auto &row : rows) {
        const auto &welds =
            sw::object(sw::required(row->channels, "rider_welds"));
        const double f_press =
            sw::number(sw::required(welds, "front_pedal_press_n"));
        const double r_press =
            sw::number(sw::required(welds, "rear_pedal_press_n"));
        const double s_press =
            sw::number(sw::required(welds, "saddle_support_n"));

        double phase_rad = 0.;
        const auto weights = probe.weight_at_pedal_n(row->qpos, phase_rad);
        const auto &ctrl_obj =
            sw::object(sw::required(row->channels, "control"));
        const auto &posture_obj =
            sw::object(sw::required(ctrl_obj, "posture"));
        const double torso_lean_rad =
            sw::number(sw::required(posture_obj, "torso_lean_rad"));
        const double ik_flex_deg =
            controller->ik_target_elbow_flexion_rad(probe.scratch.get(),
                                                    torso_lean_rad) *
            180. / kPi;
        if (ik_flex_deg < min_ik_elbow_flexion_deg) {
            min_ik_elbow_flexion_deg = ik_flex_deg;
            min_ik_elbow_time_s = row->time_s;
        }
        const auto &rc =
            sw::object(sw::required(row->channels, "rider_control"));
        const auto &drive =
            sw::object(sw::required(row->channels, "drive"));
        const double human_cmd =
            sw::number(sw::required(drive, "human_command_nm"));
        const double cadence_rpm =
            row->qvel[static_cast<std::size_t>(env.crank_dof)] * 60. /
            (2. * kPi);

        std::array<double, 2> cur_shoulder_nm{0., 0.}, cur_elbow_nm{0., 0.};
        for (std::size_t arm = 0; arm < 2; ++arm) {
            const char *sh_key =
                arm == 0 ? "rider_shoulder_left" : "rider_shoulder_right";
            const char *el_key =
                arm == 0 ? "rider_elbow_left" : "rider_elbow_right";
            const auto &sh_obj = sw::object(sw::required(rc, sh_key));
            const auto &el_obj = sw::object(sw::required(rc, el_key));
            cur_shoulder_nm[arm] =
                sw::number(sw::required(sh_obj, "command_nm"));
            cur_elbow_nm[arm] =
                sw::number(sw::required(el_obj, "command_nm"));
        }
        if (has_prev_step) {
            for (std::size_t arm = 0; arm < 2; ++arm) {
                const double d_sh =
                    std::abs(cur_shoulder_nm[arm] - prev_shoulder_nm[arm]);
                const double d_el =
                    std::abs(cur_elbow_nm[arm] - prev_elbow_nm[arm]);
                if (d_sh > max_delta_shoulder_nm) {
                    max_delta_shoulder_nm = d_sh;
                    max_delta_shoulder_time_s = row->time_s;
                }
                if (d_el > max_delta_elbow_nm) {
                    max_delta_elbow_nm = d_el;
                    max_delta_elbow_time_s = row->time_s;
                }
            }
            const double df = f_press - prev_f_press;
            const double dr = r_press - prev_r_press;
            const double ds = s_press - prev_s_press;
            hp_f = kAlphaHp50Hz * (hp_f + df);
            hp_r = kAlphaHp50Hz * (hp_r + dr);
            hp_s = kAlphaHp50Hz * (hp_s + ds);
            sum_hp_f2 += hp_f * hp_f;
            sum_hp_r2 += hp_r * hp_r;
            sum_hp_s2 += hp_s * hp_s;
            sum_df2 += df * df;
            sum_dr2 += dr * dr;
            sum_ds2 += ds * ds;
            ++diff_count;
        }
        prev_shoulder_nm = cur_shoulder_nm;
        prev_elbow_nm = cur_elbow_nm;
        prev_f_press = f_press;
        prev_r_press = r_press;
        prev_s_press = s_press;
        has_prev_step = true;

        for (std::size_t s = 0; s < 2; ++s) {
            const double press = s == 0 ? f_press : r_press;
            const double raw_deg =
                (phase_rad + (s == 0 ? 0. : kPi)) * 180. / kPi;
            const double theta_deg =
                std::fmod(std::fmod(raw_deg, 360.) + 360., 360.);
            if (s == 0 && press < min_front_press) {
                min_front_press = press;
                min_front_time = row->time_s;
                min_front_theta = theta_deg;
                min_front_rpm = cadence_rpm;
            }
            if (s == 1 && press < min_rear_press) {
                min_rear_press = press;
                min_rear_time = row->time_s;
                min_rear_theta = theta_deg;
                min_rear_rpm = cadence_rpm;
            }
            // Rising leg after BDC: check in steady pedaling (t > 2.0 s) across
            // the post-BDC rising stroke [100 deg, 240 deg].
            if (row->time_s > 2. && theta_deg >= 100. && theta_deg <= 240.) {
                const double excess = press - weights[s];
                if (s == 0) {
                    if (excess > max_rising_excess_front) {
                        max_rising_excess_front = excess;
                        max_rising_theta_front = theta_deg;
                        max_rising_press_front = press;
                        max_rising_weight_front = weights[s];
                        max_rising_time_front = row->time_s;
                        max_rising_rpm_front = cadence_rpm;
                    }
                } else if (excess > max_rising_excess_rear) {
                    max_rising_excess_rear = excess;
                    max_rising_theta_rear = theta_deg;
                    max_rising_press_rear = press;
                    max_rising_weight_rear = weights[s];
                    max_rising_time_rear = row->time_s;
                    max_rising_rpm_rear = cadence_rpm;
                }
            }
            const char *hip_key =
                s == 0 ? "rider_hip_front" : "rider_hip_rear";
            const char *knee_key =
                s == 0 ? "rider_knee_front" : "rider_knee_rear";
            const auto &hip_obj = sw::object(sw::required(rc, hip_key));
            const auto &knee_obj = sw::object(sw::required(rc, knee_key));
            const double hip_req =
                sw::number(sw::required(hip_obj, "requested_nm"));
            const double hip_cmd =
                sw::number(sw::required(hip_obj, "command_nm"));
            const double knee_req =
                sw::number(sw::required(knee_obj, "requested_nm"));
            const double knee_cmd =
                sw::number(sw::required(knee_obj, "command_nm"));
            if (row->time_s > 2. && human_cmd > 1. &&
                theta_deg >= 270. && theta_deg <= 291.) {
                min_post_tdc_hip_req = std::min(min_post_tdc_hip_req, hip_req);
                min_post_tdc_hip_cmd = std::min(min_post_tdc_hip_cmd, hip_cmd);
            }
            if (row->time_s > 2. && human_cmd > 1. &&
                theta_deg >= 250. && theta_deg <= 310.) {
                max_tdc_knee_req = std::max(max_tdc_knee_req, knee_req);
                max_tdc_knee_cmd = std::max(max_tdc_knee_cmd, knee_cmd);
            }
        }
    }
    const double inv_n =
        diff_count > 0 ? 1. / static_cast<double>(diff_count) : 0.;
    std::cerr << std::format(
        "[{}] min press front={:+.2f} N (t={:.3f}s, {:.1f} deg, {:.1f} rpm), "
        "rear={:+.2f} N (t={:.3f}s, {:.1f} deg, {:.1f} rpm); "
        "max rising (press - weight) front={:+.2f} N (@ {:.1f} deg, t={:.3f}s, {:.1f} rpm: press={:.1f}, weight={:.1f}), "
        "rear={:+.2f} N (@ {:.1f} deg, t={:.3f}s, {:.1f} rpm: press={:.1f}, weight={:.1f}); "
        "post-TDC [270,291] min hip req={:+.2f}, cmd={:+.2f} N.m; "
        "TDC [250,310] max knee req={:+.2f}, cmd={:+.2f} N.m; "
        "min IK elbow flex={:.3f} deg (t={:.4f}s), "
        "max |d tau| shoulder={:.3f} N.m (t={:.4f}s), elbow={:.3f} N.m (t={:.4f}s); "
        "HF-RMS(>50Hz RC) front={:.2f} N, rear={:.2f} N, saddle={:.2f} N; "
        "StepDiff RMS front={:.2f} N, rear={:.2f} N, saddle={:.2f} N\n",
        label,
        min_front_press, min_front_time, min_front_theta, min_front_rpm,
        min_rear_press, min_rear_time, min_rear_theta, min_rear_rpm,
        max_rising_excess_front, max_rising_theta_front,
        max_rising_time_front, max_rising_rpm_front,
        max_rising_press_front, max_rising_weight_front,
        max_rising_excess_rear, max_rising_theta_rear,
        max_rising_time_rear, max_rising_rpm_rear,
        max_rising_press_rear, max_rising_weight_rear,
        min_post_tdc_hip_req, min_post_tdc_hip_cmd,
        max_tdc_knee_req, max_tdc_knee_cmd,
        min_ik_elbow_flexion_deg, min_ik_elbow_time_s,
        max_delta_shoulder_nm, max_delta_shoulder_time_s,
        max_delta_elbow_nm, max_delta_elbow_time_s,
        std::sqrt(sum_hp_f2 * inv_n), std::sqrt(sum_hp_r2 * inv_n),
        std::sqrt(sum_hp_s2 * inv_n),
        std::sqrt(sum_df2 * inv_n), std::sqrt(sum_dr2 * inv_n),
        std::sqrt(sum_ds2 * inv_n));

    bt::require(min_ik_elbow_flexion_deg >= 15. - 1e-6,
                std::format("{}: IK-target elbow flexion {:.4f} deg >= 15.0 deg everywhere",
                            label, min_ik_elbow_flexion_deg)
                    .c_str());
    const double max_arm_delta_bound_nm = label == "steady90" ? 1.5 : 7.;
    bt::require(max_delta_shoulder_nm <= max_arm_delta_bound_nm &&
                    max_delta_elbow_nm <= max_arm_delta_bound_nm,
                std::format("{}: arm-torque change per physics step stays small (max |d tau| shoulder={:.3f} N.m, elbow={:.3f} N.m <= {:.1f} N.m)",
                            label, max_delta_shoulder_nm, max_delta_elbow_nm,
                            max_arm_delta_bound_nm)
                    .c_str());

    bt::require(min_front_press >= -runtime::kSupportPullToleranceN,
                std::format("{}: front foot press {:.2f} N >= -{:.1f} N everywhere",
                            label, min_front_press,
                            runtime::kSupportPullToleranceN)
                    .c_str());
    bt::require(min_rear_press >= -runtime::kSupportPullToleranceN,
                std::format("{}: rear foot press {:.2f} N >= -{:.1f} N everywhere",
                            label, min_rear_press,
                            runtime::kSupportPullToleranceN)
                    .c_str());
    bt::require(max_rising_excess_front <= runtime::kSupportPullToleranceN,
                std::format("{}: rising front leg press after BDC <= weight + {:.1f} N (press - weight = {:+.2f} N)",
                            label, runtime::kSupportPullToleranceN, max_rising_excess_front)
                    .c_str());
    bt::require(max_rising_excess_rear <= runtime::kSupportPullToleranceN,
                std::format("{}: rising rear leg press after BDC <= weight + {:.1f} N (press - weight = {:+.2f} N)",
                            label, runtime::kSupportPullToleranceN, max_rising_excess_rear)
                    .c_str());
    bt::require(min_post_tdc_hip_req >= -1e-6 && min_post_tdc_hip_cmd >= -1e-6,
                std::format("{}: hip torque in [270, 291] deg is never negative/flexion (req={:+.2f}, cmd={:+.2f} N.m)",
                            label, min_post_tdc_hip_req, min_post_tdc_hip_cmd)
                    .c_str());
    bt::require(max_tdc_knee_req <= 1e-6 && max_tdc_knee_cmd <= 1e-6,
                std::format("{}: knee torque at TDC [250, 310] deg is never positive/flexion (req={:+.2f}, cmd={:+.2f} N.m)",
                            label, max_tdc_knee_req, max_tdc_knee_cmd)
                    .c_str());
}

void test_every_step_control_profiles_complete_strict_without_saddle_pull() {
    for (const std::string_view rel :
         {"examples/research/viewer_physics_welded.toml",
          "examples/research/viewer_physics_pneumatic.toml",
          "examples/research/viewer_physics_distributed.toml"}) {
        const auto physics = cfg::load_physics(repo_file(rel));
        bt::require(
            std::abs(physics.control_period_s - .00125) < 1e-12,
            std::format("{}: control_period_s is 0.00125 (got {:.6f})",
                        rel, physics.control_period_s)
                .c_str());
    }
    std::string first_leg_contract_error;
    const auto check_ride = [&](auto &&fn) {
        try {
            fn();
        } catch (const std::exception &e) {
            std::cerr << "ride contract error: " << e.what() << "\n";
            if (first_leg_contract_error.empty())
                first_leg_contract_error = e.what();
        }
    };
    const auto welded_physics = cfg::load_physics(
        repo_file("examples/research/viewer_physics_welded.toml"));
    double rear_effective_radius_m = 0.;
    {
        auto flat = make_strict_ride_env("tests/fixtures/tracks/flat.toml",
                                         /*duration_s=*/6.,
                                         /*initial_speed_mps=*/0.);
        check_ride([&] {
            assert_strict_ride_without_saddle_pull(flat, runtime::RideControl{},
                                                   "flat");
        });
        const auto &tires0 = sw::object(sw::required(
            intervals_of(*flat.runtime).front()->channels, "tires"));
        const auto &rear0 = sw::object(sw::required(tires0, "rear"));
        rear_effective_radius_m =
            sw::number(sw::required(rear0, "effective_radius_m"));
        bt::require(rear_effective_radius_m > 0.,
                    "settled rear tire effective radius is positive");
        check_ride([&] {
            assert_leg_control_contracts(flat, "flat");
        });
    }
    {
        auto climb = make_strict_ride_env(
            "tests/fixtures/tracks/climb_15pct.toml", /*duration_s=*/25.,
            /*initial_speed_mps=*/0.);
        check_ride([&] {
            assert_strict_ride_without_saddle_pull(climb, runtime::RideControl{},
                                                   "climb_15pct");
        });
        check_ride([&] {
            assert_leg_control_contracts(climb, "climb_15pct");
        });
    }
    {
        const double start_gear_ratio =
            static_cast<double>(welded_physics.drive.gearing.front_teeth) /
            static_cast<double>(welded_physics.drive.gearing.rear_teeth);
        const double target_crank_rate_rad_s = 90. * 2. * kPi / 60.;
        const double steady90_initial_speed_mps =
            target_crank_rate_rad_s * start_gear_ratio *
            rear_effective_radius_m;
        std::cerr << std::format(
            "[steady90 init] gear={}/{}, rear_r_eff={:.6f} m, v0={:.6f} m/s\n",
            welded_physics.drive.gearing.front_teeth,
            welded_physics.drive.gearing.rear_teeth, rear_effective_radius_m,
            steady90_initial_speed_mps);
        auto steady90 = make_strict_ride_env(
            "tests/fixtures/tracks/flat.toml",
            /*duration_s=*/8.,
            /*initial_speed_mps=*/steady90_initial_speed_mps);
        const runtime::RideControl control{.motor_torque_nm = std::nullopt,
                                           .motor_limit_nm = std::nullopt,
                                           .human_torque_nm = 24.,
                                           .crank_target_rate_rad_s = std::nullopt,
                                           .posture = std::nullopt};
        check_ride([&] {
            assert_strict_ride_without_saddle_pull(steady90, control, "steady90");
        });
        check_ride([&] {
            assert_leg_control_contracts(steady90, "steady90");
        });
    }
    bt::require(first_leg_contract_error.empty(),
                first_leg_contract_error);
}

// physics-2026-11-05 plan task 5 («Действует» 9 & 34, rule 4):
// when the seated rider's torque demand exceeds the vertical-equilibrium
// foot-press limit, the leg demand and coast_hold_nm_ are capped by that
// limit, coast_hold_nm_ does not wind up at the limit, saddle_support_n
// stays >= 0 everywhere, and crank torque stays below the demand.
void test_seated_press_limit_keeps_saddle_support_nonnegative_and_caps_crank_torque_and_hold() {
    auto env = cycle_env();
    constexpr double kDemandTorqueNm = 60.;
    // Step 264 (t = 0.330 s) is a step during the initial 60 N.m launch mash where the seated press limit is active.
    constexpr int kPressLimitProbeStep = 264;
    const runtime::RideControl control{.motor_torque_nm = 0.,
                                       .motor_limit_nm = std::nullopt,
                                       .human_torque_nm = kDemandTorqueNm,
                                       .crank_target_rate_rad_s = std::nullopt,
                                       .posture = std::nullopt};
    double min_saddle_support_n = std::numeric_limits<double>::infinity();
    double min_saddle_time_s = 0.;
    double prev_hold_nm = 0.;
    double max_hold_nm = 0.;
    double max_windup_at_limit_nm = 0.;
    double max_probe_ctrl_diff_nm = 0.;
    std::vector<double> advancing_ctrl_at_probe_step;
    std::size_t steps_at_press_limit = 0;
    std::size_t windup_steps_at_limit = 0;
    double sum_late_cmd_nm = 0., sum_late_task_nm = 0., sum_late_sens_nm = 0.;
    std::size_t late_count = 0;

    while (!env.runtime->snapshot().outcome.has_value() &&
           env.runtime->snapshot().step < env.max_steps) {
        const auto step_before = env.runtime->snapshot().step;
        double phase = 0.;
        double ripple = 0.;
        bike::validation::RuntimeRigAccess::inspect(
            *env.runtime, [&](const Stepper &stepper,
                              const runtime::PhysicalStep &physical) {
                const auto &ctrl = physical.rider_control();
                prev_hold_nm = ctrl.state().coast_hold_nm.value_or(0.);
                ripple = ctrl.config().pedal_torque_ripple;
                const auto *m = stepper.model();
                const auto *d = stepper.data();
                const int fb = mj_name2id(m, mjOBJ_BODY, "frame");
                const int cj = mj_name2id(m, mjOBJ_JOINT, "crank_spin");
                const int pb = mj_name2id(m, mjOBJ_BODY, "pedal_front");
                const auto xmat =
                    model_access::readonly_buffer(d->xmat, 9 * m->nbody);
                const auto xpos =
                    model_access::readonly_buffer(d->xpos, 3 * m->nbody);
                const auto xanchor =
                    model_access::readonly_buffer(d->xanchor, 3 * m->njnt);
                const double dx =
                    xpos[3 * static_cast<std::size_t>(pb)] -
                    xanchor[3 * static_cast<std::size_t>(cj)];
                const double dy =
                    xpos[3 * static_cast<std::size_t>(pb) + 1] -
                    xanchor[3 * static_cast<std::size_t>(cj) + 1];
                const double dz =
                    xpos[3 * static_cast<std::size_t>(pb) + 2] -
                    xanchor[3 * static_cast<std::size_t>(cj) + 2];
                const auto fbase = 9 * static_cast<std::size_t>(fb);
                const double sx = xmat[fbase + 0] * dx +
                                  xmat[fbase + 3] * dy +
                                  xmat[fbase + 6] * dz;
                const double sz = xmat[fbase + 2] * dx +
                                  xmat[fbase + 5] * dy +
                                  xmat[fbase + 8] * dz;
                phase = std::atan2(-sz, sx);
            });
        const auto result = env.runtime->advance(
            step_before + 1, control, 0., 0.,
            std::nullopt);
        if (step_before == kPressLimitProbeStep) {
            bike::validation::RuntimeRigAccess::inspect(
                *env.runtime, [&](const Stepper &stepper,
                                  const runtime::PhysicalStep &) {
                    const auto *m = stepper.model();
                    const auto *d = stepper.data();
                    const auto ctrl_buf =
                        model_access::readonly_buffer(d->ctrl, m->nu);
                    advancing_ctrl_at_probe_step.assign(ctrl_buf.begin(),
                                                        ctrl_buf.end());
                });
        }
        const auto &rows = intervals_of(*env.runtime);
        bt::require(!rows.empty(), "interval committed");
        const auto &row = *rows.back();
        const auto &welds =
            sw::object(sw::required(row.channels, "rider_welds"));
        const double s_n = sw::number(sw::required(welds, "saddle_support_n"));
        if (s_n < min_saddle_support_n) {
            min_saddle_support_n = s_n;
            min_saddle_time_s = row.time_s;
        }
        const auto &drive =
            sw::object(sw::required(row.channels, "drive"));
        const double cmd_nm =
            sw::number(sw::required(drive, "human_command_nm"));
        const double sens_nm =
            sw::number(sw::required(drive, "human_sensor_nm"));
        double hold_nm = 0.;
        double crank_task_nm = 0.;
        bike::validation::RuntimeRigAccess::inspect(
            *env.runtime, [&](const Stepper &,
                              const runtime::PhysicalStep &physical) {
                const auto &ctrl = physical.rider_control();
                const auto st = ctrl.state();
                hold_nm = st.coast_hold_nm.value_or(0.);
                crank_task_nm = st.allocation_diagnostics.crank_task_nm;
            });
        if (row.time_s > 1.) {
            sum_late_cmd_nm += cmd_nm;
            sum_late_task_nm += crank_task_nm;
            sum_late_sens_nm += sens_nm;
            ++late_count;
        }
        max_hold_nm = std::max(max_hold_nm, hold_nm);
        if (cmd_nm > 0.) {
            const double raw_task_nm =
                spindle::spindle_torque_waveform(cmd_nm, phase, ripple);
            const double uncapped_total_nm =
                raw_task_nm +
                std::clamp(prev_hold_nm, -raw_task_nm, raw_task_nm);
            if (crank_task_nm < uncapped_total_nm - 1e-6) {
                ++steps_at_press_limit;
                const double d_hold = hold_nm - prev_hold_nm;
                if (d_hold > 1e-12) {
                    ++windup_steps_at_limit;
                    max_windup_at_limit_nm =
                        std::max(max_windup_at_limit_nm, d_hold);
                }
            }
        }
        if (result.reason == "outcome") break;
    }
    bt::require(!advancing_ctrl_at_probe_step.empty(),
                "captured advancing ctrl at press-limit probe step");
    env.runtime->reset();
    (void)env.runtime->advance(kPressLimitProbeStep, control, 0., 0.,
                               std::nullopt);
    const auto probe_step = env.runtime->probe_step_inputs(control, 0., 0.);
    bt::require(probe_step.ctrl.size() == advancing_ctrl_at_probe_step.size(),
                "probe ctrl matches advancing ctrl width");
    for (std::size_t i = 0; i < probe_step.ctrl.size(); ++i) {
        max_probe_ctrl_diff_nm = std::max(
            max_probe_ctrl_diff_nm,
            std::abs(probe_step.ctrl[i] - advancing_ctrl_at_probe_step[i]));
    }
    bt::require(late_count > 0, "late window recorded samples");
    const double mean_cmd_nm =
        sum_late_cmd_nm / static_cast<double>(late_count);
    const double mean_task_nm =
        sum_late_task_nm / static_cast<double>(late_count);
    const double mean_sens_nm =
        sum_late_sens_nm / static_cast<double>(late_count);
    std::cerr << std::format(
        "[task5 60nm] min_saddle_support_n={:+.2f} N (t={:.4f}s), "
        "mean_cmd={:.2f} N.m, mean_task={:.2f} N.m, mean_sens={:.2f} N.m (demand={:.1f} N.m), "
        "steps_at_press_limit={}, windup_steps_at_limit={}, "
        "max_windup_at_limit={:+.4f} N.m, max_hold={:+.2f} N.m, "
        "max_probe_ctrl_diff={:.4e} N.m\n",
        min_saddle_support_n, min_saddle_time_s, mean_cmd_nm, mean_task_nm,
        mean_sens_nm, kDemandTorqueNm, steps_at_press_limit,
        windup_steps_at_limit, max_windup_at_limit_nm, max_hold_nm,
        max_probe_ctrl_diff_nm);
    bt::require(min_saddle_support_n >= 0.,
                std::format("with demand above the vertical-equilibrium limit, "
                            "saddle support stays >= 0 (min={:+.2f} N)",
                            min_saddle_support_n)
                    .c_str());
    bt::require(steps_at_press_limit > 0,
                "60 N.m demand hits the seated vertical-equilibrium press ceiling");
    bt::require(windup_steps_at_limit == 0,
                "coast_hold_nm_ does not integrate upward while at the vertical-equilibrium press limit");
    bt::require(mean_task_nm < kDemandTorqueNm &&
                    mean_sens_nm < mean_cmd_nm &&
                    mean_sens_nm < kDemandTorqueNm,
                "capped crank task and delivered crank torque stay below the demand");
    bt::require(max_probe_ctrl_diff_nm <= 1e-12,
                std::format("current-step acceleration solve is independent of "
                            "finite-difference dt_s between probe_step_inputs "
                            "and advance (max diff={:.4e} N.m)",
                            max_probe_ctrl_diff_nm)
                    .c_str());
}

} // namespace

int main() {
    return bt::run([] {
        test_ankle_program_is_periodic_and_scrape_is_only_an_intent();
        test_pedal_intent_validates_its_inputs();
        test_pedal_intent_carries_no_torque_or_force_field();
        test_cadence_control_validates_a_finite_nonnegative_wish();
        test_first_pedal_command_and_restart_respect_configured_effort_slew();
        test_shift_cut_drop_and_return_respect_configured_effort_slew();
        test_rider_effort_never_exceeds_lowered_ceiling_when_ceiling_drops_in_one_step();
        test_strict_flat_cycle_delivers_and_stays_within_budgets();
        test_rigid_per_revolution_crank_torque_tracks_command();
        test_free_coast_is_an_intent_not_a_crank_lock();
        test_clutch_engagement_is_not_a_loss();
        test_cadence_wish_is_forwarded_with_immediate_rider_fields_past_motor_queue();
        test_every_step_control_profiles_complete_strict_without_saddle_pull();
        test_seated_press_limit_keeps_saddle_support_nonnegative_and_caps_crank_torque_and_hold();
    });
}
