// Completion 3.18; source: tests/reference/test_motor_freewheel_topology.py.
// The original inertias, intervals and pytest.approx limits are retained.
#include "drive_reference_support.hpp"
#include "policy_reference_support.hpp"
#include "support/model_fixture.hpp"
#include "initialization/capture.hpp"
#include "initialization/setup_owner.hpp"
#include "model/mapping.hpp"
#include "stepper.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <numeric>
#include <vector>

namespace {
using drive_reference::near;
namespace cfg = bike::configuration;
constexpr std::string_view two_inertias = R"(<mujoco>
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <body name="rotor"><joint name="rotor_spin" type="hinge" axis="0 1 0"/>
      <geom type="sphere" size="0.05" mass="0.3"/>
      <inertial pos="0 0 0" mass="0.3" diaginertia="0.075 0.15 0.075"/></body>
    <body name="crank"><joint name="crank_spin" type="hinge" axis="0 1 0"/>
      <geom type="sphere" size="0.05" mass="1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.5 1.0 0.5"/></body>
  </worldbody>
  <tendon><fixed name="motor_freewheel" limited="true" range="-1e12 0" margin="0" solreflimit="0.001 1">
    <joint joint="rotor_spin" coef="1"/><joint joint="crank_spin" coef="-1"/></fixed></tendon>
  <actuator><motor name="mid_drive" joint="rotor_spin" gear="1" ctrllimited="true" ctrlrange="0 85"/></actuator>
</mujoco>)";

struct Plant {
    bt::ModelFixture engine{two_inertias};
    std::unique_ptr<drivetrain::Transmission> hub = std::make_unique<drivetrain::Transmission>(
        engine.model.get(), drivetrain::GearingConfig{3, 3, .0127}, false,
        "motor_freewheel", "rotor_spin", "crank_spin");
    Plant() { engine::forward(engine.model.get(), engine.data.get()); hub->reset(engine.data.get()); }
    [[nodiscard]] auto velocity() { return model_access::mutable_buffer(engine.data->qvel, engine.model->nv); }
    [[nodiscard]] double relative_angle() const {
        const auto q = model_access::readonly_buffer(engine.data->qpos, engine.model->nq);
        return q[0] - q[1];
    }
    std::vector<double> advance(int steps, double control) {
        std::vector<double> result; result.reserve(static_cast<std::size_t>(steps));
        for (int step = 0; step < steps; ++step) {
            hub->prepare(engine.data.get());
            model_access::mutable_buffer(engine.data->ctrl, engine.model->nu)[0] = control;
            engine::step(engine.model.get(), engine.data.get());
            result.push_back(hub->solved(engine.data.get())[1]);
        }
        return result;
    }
};
double mean(std::span<const double> values) {
    bt::require(!values.empty(), "mean uses a nonempty source interval");
    return std::accumulate(values.begin(), values.end(), 0.) / static_cast<double>(values.size());
}

// Source: test_rotor_torque_drives_the_crank_through_the_engaged_freewheel.
void test_rotor_torque_drives_the_crank_through_the_engaged_freewheel() {
    auto plant = std::make_unique<Plant>(); const auto torques = plant->advance(200, 10.);
    near(plant->velocity()[0], plant->velocity()[1], "rotor and crank accelerate together", 1e-3);
    near(plant->velocity()[1], 10. / 1.15 * .2, "combined inertia acceleration", .02);
    bt::require(mean(std::span(torques).subspan(20)) > 0., "engaged freewheel drives the crank");
}
// Source: test_crank_overruns_a_coasting_rotor_without_backdriving_it.
void test_crank_overruns_a_coasting_rotor_without_backdriving_it() {
    auto plant = std::make_unique<Plant>(); (void)plant->advance(200, 10.);
    const double before = plant->velocity()[0]; plant->velocity()[1] += 5.;
    const auto forces = plant->advance(200, 0.);
    for (double force : forces) bt::require(std::abs(force) < 1e-6, "overrunning crank does not backdrive rotor");
    near(plant->velocity()[0], before, "rotor coasts", 0., 1e-9);
    near(plant->velocity()[1], before + 5., "crank keeps its overrun velocity", 0., 1e-9);
}
// Source: test_rotor_catches_up_and_reengages_without_lash.
void test_rotor_catches_up_and_reengages_without_lash() {
    auto plant = std::make_unique<Plant>(); (void)plant->advance(200, 10.);
    plant->velocity()[1] += 2.; (void)plant->advance(50, 0.);
    bt::require(plant->hub->relative_rate(plant->engine.data.get()) < 0., "crank is overrunning");
    plant->hub->prepare(plant->engine.data.get());
    near(plant->hub->state().boundary.value(), plant->relative_angle(), "ratchet tracks the overrun angle", 0., 1e-6);
    const auto torques = plant->advance(400, 30.);
    near(plant->velocity()[0], plant->velocity()[1], "rotor catches the crank", 1e-3);
    near(mean(std::span(torques).last(20)), 30. / 1.15, "reengaged mean crank torque", .02);
    (void)plant->advance(50, 0.); plant->hub->prepare(plant->engine.data.get());
    near(plant->hub->state().boundary.value(), plant->relative_angle(), "unloaded ratchet has no retained lash", 0., 1e-6);
    const auto reloaded = plant->advance(20, 30.);
    bt::require(*std::ranges::max_element(reloaded) > 20., "ratchet reengages immediately on reload");
}

struct FullPlant {
    cfg::PhysicalDriveConfig raw;
    std::unique_ptr<Stepper> engine;
};
FullPlant full_plant(bool clutch = false, double inertia = 0., double speed_kph = 0.) {
    auto request = std::make_unique<bike::initialization::SetupRequest>();
    request->model.mode = "ride"; request->model.rider = cfg::RiderSpecs{};
    request->model.rider->variant = "articulated_planar";
    request->model.physics = cfg::load_physics(bt::fixture("config/viewer_physics_welded.toml"));
    auto& physics = *request->model.physics;
    // _config replaces the complete [drive] table, retaining its child tables.
    const auto old = physics.drive;
    physics.drive = cfg::PhysicalDriveConfig{};
    physics.drive.pedaling = old.pedaling; physics.drive.assist = old.assist;
    physics.drive.battery = old.battery; physics.drive.gearing = old.gearing;
    physics.drive.shifting = old.shifting;
    physics.drive.transmission_model = "ideal_mid_drive";
    physics.drive.motor_clutch = clutch; physics.drive.rotor_inertia_kgm2 = inertia;
    if (inertia > 0.) physics.drive.human_torque_nm = 20.;
    physics.drive.assist.profile = "bosch_cx_gen4"; physics.drive.assist.max_torque = 1.;
    physics.initial_speed_mps = speed_kph / 3.6;
    cfg::post_init(physics);
    request->track = bike::terrain::track_from_dict(bike::terrain::parse_track_toml(
        "name = \"probe\"\nlength_m = 30.0\nsurface = \"hardpack\"\n"
        "grade_profile = { knots = [[0.0, 0.0], [30.0, 0.0]] }\n"));
    auto raw = physics.drive;
    auto owner = bike::initialization::make_setup(std::move(*request));
    (void)bike::initialization::settle(*owner);
    auto bootstrap = std::make_unique<runtime::RuntimeBootstrap>(owner->seal_bootstrap());
    auto engine = std::make_unique<Stepper>(bootstrap->model.mjb, nativecfg::decode(bootstrap->config.writer_config));
    engine::set_state(engine->model(), engine->data(), bootstrap->state.integration_state.data(),
        static_cast<int>(mjSTATE_INTEGRATION));
    engine->note_posvel_stale(); engine->forward(); engine->drive().restore(bootstrap->state.drive);
    return {std::move(raw), std::move(engine)};
}
int actuator(const Stepper& engine) {
    return bike::model::required_id(*engine.model(), mjOBJ_ACTUATOR, "mid_drive");
}
void actuator_joint(const Stepper& engine, std::string_view name) {
    const auto ids = model_access::readonly_buffer(engine.model()->actuator_trnid, 2 * engine.model()->nu);
    bt::require(ids[2U * static_cast<std::size_t>(actuator(engine))] ==
        bike::model::required_id(*engine.model(), mjOBJ_JOINT, name), "motor actuator targets the required joint");
}
double diagnostic(const Stepper& engine, std::string_view name) {
    return std::get<double>(engine.drive().diagnostics(false).at(name));
}
// Source: test_default_topology_is_rigid_crank_with_motor_on_the_crank.
void test_default_topology_is_rigid_crank_with_motor_on_the_crank() {
    const auto plant = full_plant(); const auto& engine = *plant.engine;
    for (auto name : {"crank_clutch", "motor_freewheel"})
        bt::require(mj_name2id(engine.model(), mjOBJ_TENDON, name) == -1, "rigid crank has no auxiliary tendon");
    bt::require(mj_name2id(engine.model(), mjOBJ_JOINT, "drive_shaft_spin") == -1, "rigid crank has no separate drive shaft");
    actuator_joint(engine, "crank_spin");
    const auto state = std::make_unique<drivetrain::DriveSnapshot>(engine.drive().state());
    bt::require(!state->clutch && !state->freewheel, "rigid drive owns neither auxiliary transmission");
    const auto range = model_access::readonly_buffer(engine.model()->actuator_ctrlrange, 2 * engine.model()->nu);
    const auto index = 2U * static_cast<std::size_t>(actuator(engine));
    bt::require(range[index] == 0. && range[index + 1] == 85., "profile sets full motor actuator range");
    bt::require(plant.raw.assist.max_torque == 1., "raw conflicting torque remains distinct from resolved profile");
}
// Source: test_rotor_inertia_adds_a_freewheeled_rotor_body.
void test_rotor_inertia_adds_a_freewheeled_rotor_body() {
    auto plant = full_plant(false, .15, 1.); auto& engine = *plant.engine;
    const auto body = static_cast<std::size_t>(bike::model::required_id(*engine.model(), mjOBJ_BODY, "motor_rotor"));
    const auto inertia = model_access::readonly_buffer(engine.model()->body_inertia, 3 * engine.model()->nbody).subspan(3 * body, 3);
    near(*std::ranges::max_element(inertia), .15, "rotor principal inertia");
    bt::require(model_access::readonly_buffer(engine.model()->body_mass, engine.model()->nbody)[body] < 1., "reflected rotor has low material mass");
    bt::require(mj_name2id(engine.model(), mjOBJ_TENDON, "motor_freewheel") >= 0, "rotor has a freewheel tendon");
    actuator_joint(engine, "rotor_spin");
    const auto rotor = static_cast<std::size_t>(bike::model::joint_address(*engine.model(), "rotor_spin").second);
    const auto crank = static_cast<std::size_t>(bike::model::joint_address(*engine.model(), "crank_spin").second);
    auto velocity = model_access::mutable_buffer(engine.data()->qvel, engine.model()->nv);
    near(velocity[rotor], velocity[crank], "native initial speed synchronizes rotor and crank");
    bt::require(velocity[rotor] > 0., "initial forward speed reaches the rotor");
    bt::require(model_access::readonly_buffer(engine.model()->actuator_ctrlrange, 2 * engine.model()->nu)[
        2U * static_cast<std::size_t>(actuator(engine)) + 1] == 85., "rotor actuator retains full profile torque");
    auto state = std::make_unique<drivetrain::DriveSnapshot>(engine.drive().state());
    bt::require(state->freewheel.has_value(), "native writer owns the rotor freewheel");
    state->assist.torque = 85.; engine.drive().restore(*state);
    velocity[rotor] = 150. * drive_reference::rpm_to_rad_s;
    velocity[crank] = 60. * drive_reference::rpm_to_rad_s;
    engine.note_posvel_stale(); engine.forward();
    const drivetrain::PedalingState pedaling{.mode=drivetrain::PedalMode::pedaling,
        .reason=drivetrain::CoastReason::none, .effort_nm=20., .required_cadence_rpm=60.};
    (void)engine.drive().components({}, .001, 3., false, true, true, 100., pedaling, true, std::nullopt);
    // Independent linear interpolation of the source fixture's 120.1/180 rpm segment.
    const double expected = 47.7 + (150. - 120.1) * (0. - 47.7) / (180. - 120.1);
    near(diagnostic(engine, "motor_torque_nm"), expected, "rotor speed selects its own torque curve limit");
    near(diagnostic(engine, "motor_shaft_power_w"), expected * velocity[rotor], "motor shaft power uses rotor speed");
    bt::require(diagnostic(engine, "motor_shaft_power_w") <= 600., "rotor respects the profile power ceiling");
    engine.step();
    for (double force : engine.drive().settle()) bt::require(std::isfinite(force), "freewheel settles finite forces");
    bt::require(diagnostic(engine, "motor_freewheel_torque_nm") >= 0., "freewheel never transmits reverse torque");
    bt::require(diagnostic(engine, "motor_freewheel_dissipation_power_w") >= 0., "freewheel dissipation is nonnegative");
}
// Source: test_legacy_crank_clutch_still_builds_for_regression_ab.
void test_legacy_crank_clutch_still_builds_for_regression_ab() {
    const auto plant = full_plant(true); const auto& engine = *plant.engine;
    bt::require(mj_name2id(engine.model(), mjOBJ_TENDON, "crank_clutch") >= 0, "legacy crank clutch tendon remains");
    actuator_joint(engine, "drive_shaft_spin");
    bt::require(engine.drive().state().clutch.has_value(), "legacy drive owns its clutch state");
}
// Source: test_rotor_and_legacy_clutch_are_exclusive.
void test_rotor_and_legacy_clutch_are_exclusive() {
    for (int index = 0; index < 3; ++index) {
        cfg::PhysicalDriveConfig drive;
        drive.transmission_model = index == 1 ? "elastic_chain" : "ideal_mid_drive";
        drive.motor_clutch = index == 0; drive.rotor_inertia_kgm2 = index == 2 ? -.1 : .1;
        bt::invalid([&] { cfg::detail::post_init(drive); });
    }
}
// Source: test_rotor_inertia_requires_a_finite_nonnegative_scalar (nan, inf, bool).
void test_rotor_inertia_requires_a_finite_nonnegative_scalar() {
    using namespace runtime::setupvalue;
    for (const auto& bad : std::array{Value{bike::io::Number{"nan"}}, Value{bike::io::Number{"inf"}}, Value{true}}) {
        const Value input{Value::Object{{"physics_mode", text("physical")}, {"drive_mode", text("crank_effort")},
            {"drive", Value{Value::Object{{"transmission_model", text("ideal_mid_drive")}, {"rotor_inertia_kgm2", bad}}}}}};
        bt::invalid([&] { (void)cfg::detail::construct_config(input); });
    }
}
// Source: test_rotor_requires_an_effort_drive_mode.
void test_rotor_requires_an_effort_drive_mode() {
    auto physics = std::make_unique<cfg::SimulationPhysicsConfig>(); physics->physics_mode = "physical";
    physics->drive_mode = "coast"; physics->drive.transmission_model = "ideal_mid_drive";
    physics->drive.rotor_inertia_kgm2 = .15;
    bt::invalid([&] { cfg::post_init(*physics); });
}
} // namespace
int main() {
    return bt::run([] {
        test_rotor_torque_drives_the_crank_through_the_engaged_freewheel();
        test_crank_overruns_a_coasting_rotor_without_backdriving_it();
        test_rotor_catches_up_and_reengages_without_lash();
        test_default_topology_is_rigid_crank_with_motor_on_the_crank();
        test_rotor_inertia_adds_a_freewheeled_rotor_body();
        test_legacy_crank_clutch_still_builds_for_regression_ab();
        test_rotor_and_legacy_clutch_are_exclusive();
        test_rotor_inertia_requires_a_finite_nonnegative_scalar();
        test_rotor_requires_an_effort_drive_mode();
    });
}
