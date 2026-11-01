// Ports tests/reference/test_seated_climb_surge.py — the finite rider
// surge budget and the front-load-share posture channel.
//
// SeatedClimbPolicy's power_target_w is the identical record the oracle's
// seated_climb.py runs: the same deplete/refill arithmetic against the
// same IntentConfig fields, asserted bitwise like the Python equality
// checks.
//
// The front-share node calls rider_intent.py's signals_from_channels:
// natively that free-function fold lives as PhysicalStep::update_intent_
// signals, so RuntimeRigAccess::fold_intent_signals runs it on the
// oracle's own channel dict — 'tires.front/rear.normal_load_n' in,
// the optional share out. The rig hosting the PhysicalStep is the
// welded-plant fixture request; the fold itself reads no physics state,
// so the numbers are the dict's literals like the Python assertion.
#include "support/migration_test.hpp"
#include "support/golden.hpp"
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "initialization/setup_owner.hpp"
#include "model/rider_geometry.hpp"
#include "rider/intent.hpp"
#include "runtime/control.hpp"
#include "runtime/runtime.hpp"
#include "research/environment.hpp"
#include "terrain/trackfile.hpp"
#include "validation/runtime_access.hpp"
#include <limits>
#include <memory>
#include <optional>
#include <string>

namespace {

namespace rider = ::rider;
namespace research = bike::research;
namespace cfg = bike::configuration;

// The live example document — bt::oracle::root() is
// <repo>/tests/fixtures/golden. The batch's welded physics loads the
// same file the oracle's WELDED constant reads; the migration fixture
// mirror predates F1 and lacks the activation lag, delivered budgets
// and weld envelopes.
std::filesystem::path repo_file(std::string_view relative) {
    return bt::oracle::root().parent_path().parent_path().parent_path() /
           std::filesystem::path(relative);
}

// test_surge_budget_depletes_and_recovers
void test_surge_budget_depletes_and_recovers() {
    rider::IntentConfig config;
    config.enabled = true;
    config.target_crank_power_w = 250.;
    rider::SeatedClimbPolicy policy(config);
    for (int i = 0; i < 150; ++i)
        bt::require(policy.power_target_w(.25, .1) == 400.,
                    "surge holds at 400 W while the 15 s budget lasts");
    bt::require(policy.power_target_w(.25, .1) == 250.,
                "the depleted budget returns the sustained target");
    for (int i = 0; i < 20; ++i)
        bt::require(policy.power_target_w(.25, .1) == 250.,
                    "the depleted budget stays at the sustained target");
    for (int i = 0; i < 450; ++i)
        bt::require(policy.power_target_w(0., .1) == 250.,
                    "recovery calls keep the sustained target");
    bt::require(policy.power_target_w(.25, .1) == 400.,
                "the recovered budget surges again");
}

// test_below_threshold_uses_sustained_power
void test_below_threshold_uses_sustained_power() {
    rider::IntentConfig config;
    config.enabled = true;
    config.target_crank_power_w = 250.;
    rider::SeatedClimbPolicy policy(config);
    bt::require(policy.power_target_w(.15, .1) == 250.,
                "grade .15 is below the .20 surge threshold");
}

// A welded-plant rig hosts the PhysicalStep the fold runs on — the fold
// itself reads only the supplied channel dict, never physics state, so
// any prepared runtime works and the welded request is the batch's rig.
// The fixture request's captured initial-state contract predates F1
// physics, so the rig builds the same setup document the batch's other
// welded envs use: the live TOML on a flat probe track, settling fresh.
std::unique_ptr<runtime::NativeRideRuntime> welded_runtime() {
    auto owner = std::make_unique<research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = cfg::load_physics(
        repo_file("examples/research/viewer_physics_welded.toml"));
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.rider =
        bike::model::rider_from_args("articulated_planar");
    request.setup.track = bike::terrain::track_from_dict(
        bike::terrain::parse_track_toml(
            "name = \"probe\"\nlength_m = 30.0\nsurface = \"hardpack\"\n"
            "grade_profile = { knots = [[0.0, 0.0], [30.0, 0.0]] }\n"));
    request.setup.model.physics->equilibrium_cache_enabled = false;
    request.experiment.duration_s = .01;
    request.experiment.record_decimation = 80;
    request.experiment.stop_on_model_violation = false;
    request.sensors = research::SensorConfig{};
    request.sensors.sample_period_s = .005;
    const auto prepared = research::prepare_environment(std::move(owner));
    auto bootstrap = std::make_unique<runtime::RuntimeBootstrap>(
        std::move(prepared->bootstrap.ride));
    return std::make_unique<runtime::NativeRideRuntime>(
        std::move(*bootstrap));
}

// test_front_share_is_posture_only_and_unavailable_when_unloaded —
// signals_from_channels(base) on the oracle's hand-built channel dict,
// loaded then unloaded.
void test_front_share_is_posture_only_and_unavailable_when_unloaded() {
    auto runtime = welded_runtime();
    // base = {'frame_gyro_body_rad_s': (0,0,0),
    //         'frame_specific_force_body_mps2': (0,0,0),
    //         'encoders_rad_s': {'crank': 0}, 'human_torque_nm': 0,
    //         'tires': {'front': {'normal_load_n': 200},
    //                   'rear': {'normal_load_n': 800}}}
    runtime::WireObject base{
        {"frame_gyro_body_rad_s", runtime::WireArray{0., 0., 0.}},
        {"frame_specific_force_body_mps2", runtime::WireArray{0., 0., 0.}},
        {"encoders_rad_s",
         runtime::WireObject{{"crank", 0.}}},
        {"human_torque_nm", 0.},
        {"tires", runtime::WireObject{
                      {"front",
                       runtime::WireObject{{"normal_load_n", 200.}}},
                      {"rear",
                       runtime::WireObject{{"normal_load_n", 800.}}}}}};
    const auto loaded =
        bike::validation::RuntimeRigAccess::fold_intent_signals(
            *runtime, base);
    bt::require(loaded.front_load_share == .2,
                "200/(200+800) folds to a .2 front share");
    // base['tires']['front']['normal_load_n'] = 0. and the rear — both
    // below the 1 N gate, the share reads None.
    auto &tires = std::get<runtime::WireObject>(base[4].second.value);
    std::get<runtime::WireObject>(tires[0].second.value)[0].second = 0.;
    std::get<runtime::WireObject>(tires[1].second.value)[0].second = 0.;
    const auto unloaded =
        bike::validation::RuntimeRigAccess::fold_intent_signals(
            *runtime, base);
    bt::require(!unloaded.front_load_share.has_value(),
                "unloaded tires report no posture share");
}

// test_surge_reset_restores_budget_and_rejects_nonfinite_inputs
void test_surge_reset_restores_budget_and_rejects_nonfinite_inputs() {
    rider::IntentConfig config;
    config.enabled = true;
    rider::SeatedClimbPolicy policy(config);
    bt::require(policy.power_target_w(.25, 15.) == config.surge_power_w,
                "one 15 s call drains the whole surge budget");
    bt::require(policy.power_target_w(.25, .1) ==
                    policy.config().target_crank_power_w,
                "the depleted policy returns the configured target");
    policy.reset();
    bt::require(policy.power_target_w(.25, .1) == 400.,
                "reset restores the surge budget");
    bt::invalid([&] { (void)policy.power_target_w(
                          std::numeric_limits<double>::quiet_NaN(), .1); });
    bt::invalid([&] { (void)policy.power_target_w(.25, 0.); });
    bt::invalid([&] { (void)policy.power_target_w(
                          .25, std::numeric_limits<double>::infinity()); });
}

} // namespace

int main() {
    return bt::run([] {
        test_surge_budget_depletes_and_recovers();
        test_below_threshold_uses_sustained_power();
        test_front_share_is_posture_only_and_unavailable_when_unloaded();
        test_surge_reset_restores_budget_and_rejects_nonfinite_inputs();
    });
}
