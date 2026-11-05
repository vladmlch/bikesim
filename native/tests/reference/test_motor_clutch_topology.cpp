// Completion 3.18; source: tests/reference/test_motor_freewheel_topology.py.
// The original inertias, intervals and pytest.approx limits are retained.
// The rotor-freewheel cases were removed with the rotor topology
// (2026-11-05): the drivetrain keeps exactly two one-way clutches — the
// rear freehub and the crank clutch.
#include "drive_reference_support.hpp"
#include "policy_reference_support.hpp"
#include "support/model_fixture.hpp"
#include "initialization/capture.hpp"
#include "initialization/setup_owner.hpp"
#include "model/mapping.hpp"
#include "stepper.hpp"

namespace {
namespace cfg = bike::configuration;

struct FullPlant {
    cfg::PhysicalDriveConfig raw;
    std::unique_ptr<Stepper> engine;
};
// NOLINTBEGIN(modernize-make-unique) make_unique's inlined forwarding path keeps each
// by-value result on this frame and overflows the 8192-byte budget; new T(f()) elides.
FullPlant full_plant(bool clutch = false) { // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API.
    auto request = std::make_unique<bike::initialization::SetupRequest>();
    request->model.mode = "ride"; request->model.rider = cfg::RiderSpecs{};
    request->model.rider->variant = "articulated_planar";
    const auto physics_owner = std::unique_ptr<cfg::SimulationPhysicsConfig>{new cfg::SimulationPhysicsConfig(
        cfg::load_physics(bt::fixture("config/viewer_physics_welded.toml")))};
    request->model.physics = std::move(*physics_owner);
    auto& physics = *request->model.physics;
    // _config replaces the complete [drive] table, retaining its child tables.
    const auto old = std::make_unique<cfg::PhysicalDriveConfig>(physics.drive);
    physics.drive = cfg::PhysicalDriveConfig{};
    physics.drive.pedaling = old->pedaling; physics.drive.assist = old->assist;
    physics.drive.gearing = old->gearing;
    physics.drive.shifting = old->shifting;
    physics.drive.transmission_model = "ideal_mid_drive";
    physics.drive.motor_clutch = clutch;
    physics.drive.assist.profile = "bosch_cx_gen4"; physics.drive.assist.max_torque = 1.;
    cfg::post_init(physics);
    request->track = bike::terrain::track_from_dict(bike::terrain::parse_track_toml(
        "name = \"probe\"\nlength_m = 30.0\nsurface = \"hardpack\"\n"
        "grade_profile = { knots = [[0.0, 0.0], [30.0, 0.0]] }\n"));
    auto raw = std::make_unique<cfg::PhysicalDriveConfig>(physics.drive);
    auto owner = bike::initialization::make_setup(std::move(*request));
    (void)bike::initialization::settle(*owner);
    auto bootstrap = std::unique_ptr<runtime::RuntimeBootstrap>{new runtime::RuntimeBootstrap(owner->seal_bootstrap())};
    const auto writer = std::unique_ptr<nativecfg::WriterConfig>{new nativecfg::WriterConfig(
        nativecfg::decode(bootstrap->config.writer_config))};
    auto engine = std::make_unique<Stepper>(bootstrap->model.mjb, *writer);
    engine::set_state(engine->model(), engine->data(), bootstrap->state.integration_state.data(),
        static_cast<int>(mjSTATE_INTEGRATION));
    engine->note_posvel_stale(); engine->forward(); engine->drive().restore(bootstrap->state.drive);
    return {.raw=std::move(*raw), .engine=std::move(engine)};
}
// NOLINTEND(modernize-make-unique)
int actuator(const Stepper& engine) {
    return bike::model::required_id(*engine.model(), mjOBJ_ACTUATOR, "mid_drive");
}
void actuator_joint(const Stepper& engine, std::string_view name) {
    const auto ids = model_access::readonly_buffer(engine.model()->actuator_trnid, 2 * engine.model()->nu);
    bt::require(ids[2U * static_cast<std::size_t>(actuator(engine))] ==
        bike::model::required_id(*engine.model(), mjOBJ_JOINT, name), "motor actuator targets the required joint");
}
// Source: test_default_topology_is_rigid_crank_with_motor_on_the_crank.
void test_default_topology_is_rigid_crank_with_motor_on_the_crank() {
    const auto plant = full_plant(); const auto& engine = *plant.engine;
    for (auto name : {"crank_clutch", "motor_freewheel"})
        bt::require(mj_name2id(engine.model(), mjOBJ_TENDON, name) == -1, "rigid crank has no auxiliary tendon");
    bt::require(mj_name2id(engine.model(), mjOBJ_JOINT, "drive_shaft_spin") == -1, "rigid crank has no separate drive shaft");
    actuator_joint(engine, "crank_spin");
    const auto state = std::make_unique<drivetrain::DriveSnapshot>(engine.drive().state());
    bt::require(!state->clutch, "rigid drive owns no auxiliary transmission");
    const auto range = model_access::readonly_buffer(engine.model()->actuator_ctrlrange, 2 * engine.model()->nu);
    const auto index = 2U * static_cast<std::size_t>(actuator(engine));
    bt::require(range[index] == 0. && range[index + 1] == 85., "profile sets full motor actuator range");
    bt::require(plant.raw.assist.max_torque == 1., "raw conflicting torque remains distinct from resolved profile");
}
// Source: test_legacy_crank_clutch_still_builds_for_regression_ab.
void test_legacy_crank_clutch_still_builds_for_regression_ab() {
    const auto plant = full_plant(true); const auto& engine = *plant.engine;
    bt::require(mj_name2id(engine.model(), mjOBJ_TENDON, "crank_clutch") >= 0, "legacy crank clutch tendon remains");
    actuator_joint(engine, "drive_shaft_spin");
    bt::require(engine.drive().state().clutch.has_value(), "legacy drive owns its clutch state");
}
} // namespace
int main() {
    return bt::run([] {
        test_default_topology_is_rigid_crank_with_motor_on_the_crank();
        test_legacy_crank_clutch_still_builds_for_regression_ab();
    });
}
