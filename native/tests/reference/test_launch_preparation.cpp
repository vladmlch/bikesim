// Launch part 2 — the rider's launch table and the start preparation it
// drives: the pure table rules (task 1), the setup wiring of the cog and
// pedal angle (task 2) and of the lean (task 3).
#include "support/fixture_test.hpp"
#include "support/temporary_directory.hpp"
#include "diag.hpp"
#include "rider/launch_config.hpp"
#include "initialization/setup_owner.hpp"
#include "initialization/launch_preparation.hpp"
#include "research/environment.hpp"
#include "configuration/load.hpp"
#include "launch_ride_support.hpp"
#include "validation/scenarios.hpp"
#include "cli/application.hpp"
#include "io/value.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

NATIVE_DIAG_PUSH
// NATIVE_DIAG_IGNORE is the project diagnostic-pragma macro from diag.hpp; cppcheck cannot resolve it.
// cppcheck-suppress unknownMacro
NATIVE_DIAG_IGNORE("-Wframe-larger-than")

namespace {

// Runs `action`, which must throw std::invalid_argument naming `needle`.
template <typename Action>
void rejects(Action &&action, std::string_view needle, std::string_view what) {
    bool threw = false;
    try {
        std::forward<Action>(action)();
    } catch (const std::invalid_argument &failure) {
        threw = true;
        bt::require(std::string_view(failure.what()).find(needle) != std::string_view::npos,
                    std::string(what) + " names " + std::string(needle) + " (" + failure.what() + ")");
    }
    bt::require(threw, std::string(what) + " is rejected");
}

// A valid three-cog table with a two-knot lean ramp — the control every
// rejection below changes in exactly one place.
rider::LaunchConfig ramp_table() {
    return {.start_cog_grade_breaks = {.03, .08},
            .start_cog_teeth = {24, 33, 51},
            .start_pedal_angle_rad = .6,
            .start_lean_grades = {0., .2},
            .start_lean_rad = {0., .2}};
}

// The cog is stepwise and each interval is closed below: a grade equal to a
// break already takes the next cog; below the first break the first cog,
// above the last break the last one.
void test_cog_is_stepwise_by_grade() {
    const auto table = ramp_table();
    bt::require(rider::prepare_launch(table, -.05).rear_teeth == 24, "downhill takes the first cog");
    bt::require(rider::prepare_launch(table, .0299).rear_teeth == 24, "below the first break");
    bt::require(rider::prepare_launch(table, .03).rear_teeth == 33, "a break belongs to the next cog");
    bt::require(rider::prepare_launch(table, .0799).rear_teeth == 33, "below the second break");
    bt::require(rider::prepare_launch(table, .08).rear_teeth == 51, "the second break takes the last cog");
    bt::require(rider::prepare_launch(table, .3).rear_teeth == 51, "steep takes the last cog");
    rider::LaunchConfig single = table;
    single.start_cog_grade_breaks = {};
    single.start_cog_teeth = {51};
    bt::require(rider::prepare_launch(single, -.2).rear_teeth == 51 &&
                    rider::prepare_launch(single, .2).rear_teeth == 51,
                "a one-cog table starts on that cog at every grade");
}

// The lean interpolates linearly between knots and holds the end values
// outside them; the pedal angle is the table's, whatever the grade.
void test_lean_interpolates_and_angle_is_fixed() {
    const auto table = ramp_table();
    bt::require(std::abs(rider::prepare_launch(table, .05).lean_rad - .05) < 1e-15, "a quarter of the way up the ramp");
    bt::require(rider::prepare_launch(table, -.1).lean_rad == 0., "below the first knot holds the first lean");
    bt::require(rider::prepare_launch(table, .5).lean_rad == .2, "above the last knot holds the last lean");
    bt::require(rider::prepare_launch(table, .15).crank_phase_rad == .6, "the pedal angle comes from the table");
    rider::LaunchConfig flat = table;
    flat.start_lean_grades = {0.};
    flat.start_lean_rad = {.04};
    bt::require(rider::prepare_launch(flat, .3).lean_rad == .04, "one knot is a constant lean");
}

// Every shape or domain error names its field; the valid control passes.
void test_table_rules_name_the_field() {
    (void)rider::prepare_launch(ramp_table(), 0.);
    auto with = [](auto change) { auto t = ramp_table(); change(t); return t; };
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_cog_teeth = {}; t.start_cog_grade_breaks = {}; })); },
            "start_cog_teeth", "an empty cog list");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_cog_teeth = {24, 2, 51}; })); },
            "start_cog_teeth", "a two-tooth cog");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_cog_grade_breaks = {.03}; })); },
            "start_cog_grade_breaks", "one break too few");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_cog_grade_breaks = {.03, .03}; })); },
            "start_cog_grade_breaks", "equal breaks");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_cog_grade_breaks = {.08, .03}; })); },
            "start_cog_grade_breaks", "descending breaks");
    rejects([&] { rider::validate_launch_table(with([](auto &t) {
                t.start_cog_grade_breaks = {std::numeric_limits<double>::quiet_NaN(), .08}; })); },
            "start_cog_grade_breaks", "a NaN break");
    rejects([&] { rider::validate_launch_table(with([](auto &t) {
                t.start_pedal_angle_rad = std::numeric_limits<double>::infinity(); })); },
            "start_pedal_angle_rad", "an infinite pedal angle");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_lean_grades = {}; t.start_lean_rad = {}; })); },
            "start_lean_grades", "no lean knot");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_lean_rad = {0.}; })); },
            "start_lean_rad", "a lean count that differs from the knots");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_lean_grades = {.2, 0.}; })); },
            "start_lean_grades", "descending lean knots");
    rejects([&] { rider::validate_launch_table(with([](auto &t) {
                t.start_lean_rad = {0., std::numeric_limits<double>::quiet_NaN()}; })); },
            "start_lean_rad", "a NaN lean");
    rejects([&] { (void)rider::prepare_launch(ramp_table(), std::numeric_limits<double>::infinity()); },
            "grade", "an infinite preparation grade");
    const rider::LaunchConfig unset;
    rejects([&] { rider::validate_launch_table(unset); }, "start_cog_teeth", "a default-constructed table");
}

namespace init = bike::initialization;
namespace cfg = bike::configuration;

// A member of a JSON-like document; a missing member fails the test.
const bike::io::Value &member(const bike::io::Value &object, std::string_view key) {
    const auto *found = bike::io::find_member(bike::io::as_object(object, key), key);
    bt::require(found != nullptr, "the document names " + std::string(key));
    return *found;
}

// The welded flat request with a real settle on a constant grade: no
// transferred initial state, no cache, held brakes (the
// slope_brake_and_crank recipe of test_setup.cpp). Heap-owned: a
// SetupRequest is several KB.
std::unique_ptr<init::SetupRequest> settle_request(double grade) {
    auto request = std::make_unique<init::SetupRequest>(
        init::read_setup_request(bt::fixture("requests/welded_flat.toml")));
    auto &physics = request->model.physics.value(); // NOLINT(bugprone-unchecked-optional-access) the fixture request carries physics
    physics.equilibrium_cache_enabled = false;
    physics.equilibrium_state_file.reset();
    physics.initial_speed_mps = 0.;
    physics.initial_front_brake = 1.;
    physics.initial_rear_brake = 1.;
    request->initial_state.reset();
    request->track.grade_profile = bike::terrain::GradeProfile{{{0., grade}, {request->track.length_m, grade}}};
    request->model.field = bike::terrain::HeightFieldSpec::for_track(request->track);
    return request;
}

// The 24/39 table switching at 3 %, pedal at .6 rad.
void stepwise_cog(init::SetupRequest &request) {
    auto &launch = request.model.physics->rider_launch; // NOLINT(bugprone-unchecked-optional-access) set by settle_request
    launch.start_cog_grade_breaks = {.03};
    launch.start_cog_teeth = {24, 39};
    launch.start_pedal_angle_rad = .6;
}

// The rider judges the grade road_lookahead_m ahead of the start point on
// the track's grade profile; a track without one is flat.
void test_start_grade_looks_ahead_of_the_start() {
    auto track = bike::terrain::flat(40.);
    bt::require(init::rider_start_grade(track, 2., 2.) == 0., "no grade profile is flat");
    track.grade_profile = bike::terrain::GradeProfile{{{0., 0.}, {10., 0.}, {10.5, .1}, {40., .1}}};
    bt::require(init::rider_start_grade(track, 2., 2.) == 0., "flat ahead of a flat start");
    bt::require(init::rider_start_grade(track, 9., 2.) == .1, "the climb two metres ahead");
    rejects([&] { (void)init::rider_start_grade(track, 2., -1.); }, "lookahead", "a negative lookahead");
}

// Setup prepares the start from the table at the start grade before the
// model exists: on a 4 % start the 24/39 table compiles the 39T cog, and
// cog and angle reach the setup metadata and the sealed bootstrap. The
// crank coordinate itself follows the table: the same start with the pedal
// at angle 0 settles to a different crank phase.
[[gnu::noinline]] void test_setup_prepares_cog_and_angle_for_the_start_grade() {
    auto request = settle_request(.04);
    stepwise_cog(*request);
    auto owner = init::make_setup(*request);
    const auto result = init::settle(*owner);
    bt::require(result.initial_crank_phase_rad == .6, "the settle starts from the table's pedal angle");
    const auto metadata = owner->metadata();
    const auto &drive = member(member(metadata, "physics"), "drive");
    bt::require(bike::io::as_int64(member(member(drive, "gearing"), "rear_teeth"), "rear_teeth") == 39,
                "the setup physics names the prepared cog");
    const auto bootstrap = std::make_unique<runtime::RuntimeBootstrap>(owner->seal_bootstrap());
    bt::require(bootstrap->state.drive.shifting.rear_teeth == 39, "the ride starts on the prepared cog");
    request->model.physics->rider_launch.start_pedal_angle_rad = 0.; // NOLINT(bugprone-unchecked-optional-access) set by settle_request
    auto upright_pedal = init::make_setup(*request);
    bt::require(init::settle(*upright_pedal).settled_crank_phase_rad != result.settled_crank_phase_rad,
                "the pedal angle moves the actual crank coordinate");
}

// A programmatic configuration need not repair the computed cog: with a
// cassette that lacks the stale default 24, setup prepares the table's cog
// before any cassette check runs.
[[gnu::noinline]] void test_setup_prepares_before_validating() {
    auto request = settle_request(0.);
    auto &physics = request->model.physics.value(); // NOLINT(bugprone-unchecked-optional-access) set by settle_request
    physics.drive.shifting.cassette = {39, 51};
    physics.drive.gearing.rear_teeth = 24;   // the stale struct default
    physics.rider_launch.start_cog_teeth = {51};
    auto owner = init::make_setup(*request);
    const auto metadata = owner->metadata();   // owned: member() borrows from it
    const auto &drive = member(member(metadata, "physics"), "drive");
    bt::require(bike::io::as_int64(member(member(drive, "gearing"), "rear_teeth"), "rear_teeth") == 51,
                "setup prepared 51T from the table");
}

// The research recording and the research ride name the prepared cog: the
// research environment prepares its own request before anything reads it.
[[gnu::noinline]] void test_research_records_the_prepared_cog() {
    auto owner = std::make_unique<bike::research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = cfg::load_physics(
        launch_ride::repo_file("examples/research/viewer_physics_welded.toml"));
    auto &physics = *request.setup.model.physics;
    physics.initial_speed_mps = 0.;
    physics.rider_launch.start_cog_grade_breaks = {.03};
    physics.rider_launch.start_cog_teeth = {24, 39};
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.rider = bike::model::rider_from_args("articulated_planar");
    request.setup.track = launch_ride::track("[[0.0, 0.04], [60.0, 0.04]]", 60.);
    request.experiment.duration_s = .01;
    request.experiment.record_decimation = 1;
    request.experiment.maximum_energy_residual_ratio = 1e9;
    request.experiment.stop_on_model_violation = false;
    request.sensors = bike::research::SensorConfig{};
    request.experiment.control_period_s = physics.research.control_period_s;
    request.experiment.actuator_delay_s = physics.research.actuator_delay_s;
    request.sensors.delivery.latency_s = physics.research.sensor_latency_s;
    request.sensors.delivery.maximum_age_s = physics.research.sensor_maximum_age_s;
    request.sensors.sample_period_s = .005;
    const auto prepared = bike::research::prepare_environment(std::move(owner));
    const auto &resolved = member(prepared->metadata, "resolved_config");
    const auto &drive = member(member(resolved, "physics"), "drive");
    bt::require(bike::io::as_int64(member(member(drive, "gearing"), "rear_teeth"), "rear_teeth") == 39,
                "the recording names the prepared cog");
    const auto &table = member(member(resolved, "physics"), "rider_launch");
    bt::require(bike::io::as_array(member(table, "start_cog_teeth"), "start_cog_teeth").size() == 2,
                "the recording keeps the authored table");
    bt::require(prepared->bootstrap.ride.state.drive.shifting.rear_teeth == 39,
                "the research ride starts on it");
}

// The validation rig captures the initial state of the start that was
// actually prepared: a transfer captured through validation::Simulation on
// the 4 % stepwise start validates when the same request reuses it.
[[gnu::noinline]] void test_validation_capture_matches_the_prepared_start() {
    auto request = settle_request(.04);
    stepwise_cog(*request);
    auto simulation = std::make_unique<bike::validation::Simulation>(*request);
    auto transfer = std::make_unique<init::SetupRequest>(*request);
    transfer->initial_state = simulation->initial_state();
    auto reused = init::make_setup(*transfer);
    bt::require(init::settle(*reused).initial_state_sha256.has_value(),
                "the rig capture validates for the same prepared start");
}

// The research profile path holds the same rule as load_physics: a profile
// file that authors a computed member is rejected.
[[gnu::noinline]] void test_research_profile_cannot_author_the_cog() {
    const bt::TemporaryDirectory dir("research-authored-cog");
    const auto path = dir.path() / "physics.toml";
    { std::ofstream out(path); out << "[drive.gearing]\nrear_teeth = 39\n"; }
    rejects([&] { (void)bike::research::resolve_research_physics(
                cfg::default_physics(), path, bike::io::Value{bike::io::Value::Object{}}); },
            "start_cog_teeth", "a research profile that authors the rear cog");
}

} // namespace

int main() {
    return bt::run([] {
        test_cog_is_stepwise_by_grade();
        test_lean_interpolates_and_angle_is_fixed();
        test_table_rules_name_the_field();
        test_start_grade_looks_ahead_of_the_start();
        test_setup_prepares_cog_and_angle_for_the_start_grade();
        test_setup_prepares_before_validating();
        test_research_records_the_prepared_cog();
        test_validation_capture_matches_the_prepared_start();
        test_research_profile_cannot_author_the_cog();
    });
}
NATIVE_DIAG_POP
