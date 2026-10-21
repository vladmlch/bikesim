// Completion 3.21b: all ten numeric-domain functions. The suspension fixture
// keeps the recorded resolved coefficients and isolates the two slide joints;
// invalid inputs, exception classes and 61.3/181/30 mm probes are unchanged.
#include "support/model_fixture.hpp"
#include "drive_reference_support.hpp"
#include "runtime/setup_values.hpp"
#include "writer_config.hpp"
#include "writers/suspension.hpp"
#include "writers/brake.hpp"
#include "writers/cruise.hpp"
#include "writers/resistance.hpp"
#include "model/suspension_setup.hpp"
#include "drivetrain/policy_set.hpp"
#include <array>
#include <limits>
#include <tuple>

namespace {
namespace io = bike::io;
using namespace runtime::setupvalue;
template<class F> void overflow(F&& action) {
    bool caught = false;
    try { action(); } catch (const std::overflow_error&) { caught = true; }
    bt::require(caught, "finite inputs whose computed result overflows raise overflow_error");
}
Value& member(Value& value, std::string_view key) {
    auto* object = std::get_if<Object>(&value.data);
    if (!object) throw std::invalid_argument("fixture expected object");
    for (auto& [name, item] : *object) if (name == key) return item;
    throw std::invalid_argument("fixture missing " + std::string(key));
}
Value suspension_wire() {
    const auto captured = io::parse_json(bt::text(bt::fixture("bootstrap/welded/config.json")));
    auto suspension = field(field(captured, "writer_config"), "suspension");
    set(suspension, "fork_joint", text("fork_slide")); set(suspension, "shock_joint", text("shock_stroke"));
    return Value{Object{{"schema", integer(2)}, {"suspension", std::move(suspension)}}};
}
constexpr std::string_view suspension_xml = R"(<mujoco><worldbody>
  <body><joint name="fork_slide" type="slide" axis="0 0 1" limited="true" range="0 .20"/><geom size=".1" mass="1"/></body>
  <body pos="1 0 0"><joint name="shock_stroke" type="slide" axis="0 0 1" limited="true" range="-.01 .08"/><geom size=".1" mass="1"/></body>
</worldbody></mujoco>)";
nativecfg::SuspensionConfig suspension(const Value& wire) {
    const auto decoded = nativecfg::decode(wire);
    bt::require(decoded.suspension.has_value(), "fixture decodes a real suspension config");
    return *decoded.suspension;
}
void test_suspension_domain_is_shared() {
    bt::ModelFixture fixture(suspension_xml);
    const auto original = suspension_wire();
    SuspensionWriter valid(fixture.model.get(), suspension(original));
    bt::require(valid.fork_dofadr() != valid.shock_dofadr(), "valid control reaches both suspension joints");
    const std::array positive_bad{real(0.), real(-1.), Value{io::Number{"nan"}}, Value{io::Number{"inf"}}, Value{true}};
    const std::array nonnegative_bad{real(-1.), Value{io::Number{"nan"}}, Value{io::Number{"inf"}}, Value{true}};
    const auto reject = [&](std::string_view section, std::string_view key, const auto& bad_values) {
        for (const auto& bad : bad_values) {
            auto candidate = original; set(member(member(candidate, "suspension"), section), key, bad);
            bt::invalid([&] { SuspensionWriter rejected(fixture.model.get(), suspension(candidate)); });
        }
    };
    for (auto key : {"stanchion_inner_diam_mm", "total_travel_mm", "pos_chamber_length_mm", "neg_chamber_length_mm", "gamma", "atm_pressure_pa"}) reject("air_spring", key, positive_bad);
    for (auto key : {"token_volume_cm3", "gauge_pressure_psi"}) reject("air_spring", key, nonnegative_bad);
    for (auto section : {"fork_damper", "shock_damper"}) {
        for (auto key : {"max_hsc", "max_lsc", "max_reb", "v_knee_comp", "v_knee_reb"}) reject(section, key, positive_bad);
        for (auto key : {"c_lsc_min", "c_lsc_max", "c_hsc_min", "c_hsc_max", "c_reb_min", "c_reb_max"}) reject(section, key, nonnegative_bad);
    }
    for (auto key : {"rate_n_m", "stroke_mm", "bumper_length_mm", "bumper_peak_n"}) reject("coil", key, positive_bad);
    reject("coil", "preload_mm", nonnegative_bad);
    for (auto key : {"stiffness_n_m", "damping_n_s_m"}) reject("end_stops", key, nonnegative_bad);
}
void test_damper_invalid_parameters_rejected() {
    for (auto key : {"max_lsc", "v_knee_comp", "c_hsc_min"}) {
        auto cfg = suspension(suspension_wire()).fork_damper;
        if (std::string_view(key) == "max_lsc") cfg.core.max_lsc = 0;
        else if (std::string_view(key) == "v_knee_comp") cfg.core.v_knee_comp = 0.;
        else cfg.core.c_hsc_min = -1.;
        bt::invalid([&] { nativecfg::validate(cfg); });
        bt::invalid([&] { bike::configuration::domain::damper(cfg); });
    }
}
void test_air_pressure_intermediate_overflow_is_shared() {
    bt::ModelFixture fixture(suspension_xml); auto config = suspension(suspension_wire());
    config.air_spring.gauge_pressure_psi = 1e308;
    SuspensionWriter writer(fixture.model.get(), config);
    model_access::mutable_buffer(fixture.data->qpos, fixture.model->nq)[static_cast<std::size_t>(writer.fork_qposadr())] = .0613;
    bike::model::ForkAirSpring spring(std::nullopt, 1e308);
    overflow([&] { (void)spring.compute_axial_force(61.3); });
    overflow([&] { (void)writer.components(fixture.data.get()); });
}
void test_activated_zero_hbo_denominator_rejected() {
    for (bool legacy : {false, true}) {
        bike::model::Charger3Damper damper(2, 7, 9, 180., legacy);
        damper.hbo_start_mm = damper.total_travel_mm;
        bt::invalid([&] { (void)damper.compute_damping_force(.7, 181.); });
        bt::ModelFixture fixture(suspension_xml); auto config = suspension(suspension_wire());
        config.fork_damper.legacy_behavior = legacy;
        config.fork_damper.hbo_start_mm = config.fork_damper.total_travel_mm;
        if (!legacy) bt::invalid([&] { SuspensionWriter rejected(fixture.model.get(), config); });
        else {
            SuspensionWriter writer(fixture.model.get(), config);
            model_access::mutable_buffer(fixture.data->qpos, fixture.model->nq)[static_cast<std::size_t>(writer.fork_qposadr())] = .181;
            model_access::mutable_buffer(fixture.data->qvel, fixture.model->nv)[static_cast<std::size_t>(writer.fork_dofadr())] = .7;
            bt::invalid([&] { (void)writer.components(fixture.data.get()); });
        }
    }
}
void test_mutated_policy_configs_are_rejected() {
    const auto original = drive_reference::projected_policies(drive_reference::turbo_source());
    for (int i = 0; i < 11; ++i) {
        auto candidate = original;
        switch (i) {
        case 0: candidate.pedaling.stop_time_s = 0.; break;
        case 1: candidate.pedaling.coast_cadence_tau_s = -1.; break;
        case 2: candidate.pedaling.effort_slew_nm_s = -1.; break;
        case 3: candidate.shifting.shift_cooldown_s = -1.; break;
        case 4: candidate.shifting.torque_factor = 1.1; break;
        case 5: candidate.shifting.cadence_smoothing_tau_s = -1.; break;
        case 6: candidate.assist.tau = 0.; break;
        case 7: candidate.assist.slew = 0.; break;
        case 8: candidate.assist.gain = -1.; break;
        case 9: candidate.assist.taper_width_mps = 0.; break;
        case 10: candidate.battery.energy_j = -1.; break;
        default: throw std::logic_error("invalid fixture case");
        }
        bt::invalid([&] { drivetrain::validate(candidate); });
        bt::invalid([&] { drivetrain::PolicySet rejected(candidate); });
    }
}
void test_policy_derived_overflow_is_shared() {
    drivetrain::PolicySet policies(drive_reference::projected_policies(drive_reference::turbo_source()));
    overflow([&] { (void)policies.assist.step(1e308, 80., 0., false, .001); });
    overflow([&] { (void)policies.battery.draw(1e308, 10.); });
    const auto& battery = policies.config.battery;
    overflow([&] { (void)drivetrain::motor_electrical_power(1e308, 1., battery.copper_w_per_nm2, battery.speed_w_per_rad_s2, battery.idle_w, true); });
    policies.hub.set_state({.boundary = 0., .energy_j = 0., .torque_nm = 0.});
    overflow([&] { (void)policies.hub.update(1e308, 0., 0., 0.); });
}
void test_cruise_longitudinal_topology_is_shared() {
    for (const auto& [kind, axis] : std::array{std::pair{"hinge", "1 0 0"}, std::pair{"slide", "0 1 0"}, std::pair{"ball", "0 0 1"}}) {
        bt::ModelFixture fixture("<mujoco><worldbody><body><joint name=\"root_x\" type=\"" + std::string(kind) + "\" axis=\"" + axis + "\"/><geom size=\".1\" mass=\"1\"/></body></worldbody></mujoco>");
        bt::invalid([&] { CruiseWriter rejected(fixture.model.get(), {25., 180., 150., 150.}); });
    }
}
void test_brake_wheel_topology_and_actuator_target_are_shared() {
    using Row = std::tuple<const char*, const char*, const char*>;
    for (const auto& [kind, axis, target] : std::array{Row{"slide", "0 1 0", "front_wheel_spin"}, Row{"ball", "0 1 0", "front_wheel_spin"}, Row{"hinge", "0 0 1", "front_wheel_spin"}, Row{"hinge", "0 1 0", "rear_wheel_spin"}}) {
        bt::ModelFixture fixture("<mujoco><worldbody><body><joint name=\"front_wheel_spin\" type=\"" + std::string(kind) + "\" axis=\"" + axis + "\"/><geom name=\"geom_front_contact\" type=\"sphere\" size=\".3\" mass=\"1\"/></body><body><joint name=\"rear_wheel_spin\" type=\"hinge\" axis=\"0 1 0\"/><geom name=\"geom_rear_contact\" type=\"sphere\" size=\".3\" mass=\"1\"/></body></worldbody><actuator><motor name=\"front_brake\" joint=\"" + target + "\"/><motor name=\"rear_brake\" joint=\"rear_wheel_spin\"/></actuator></mujoco>");
        bt::invalid([&] { BrakeWriter rejected(fixture.model.get(), {200., 1.}); });
    }
}
void test_coil_force_overflow_before_publication() {
    bt::ModelFixture fixture(suspension_xml); auto config = suspension(suspension_wire()); config.coil.rate_n_m = 1e308;
    SuspensionWriter writer(fixture.model.get(), config);
    model_access::mutable_buffer(fixture.data->qpos, fixture.model->nq)[static_cast<std::size_t>(writer.shock_qposadr())] = .030;
    bike::model::CoilShockSpecs spec; spec.rate_n_m = 1e308; bike::model::CoilShock coil(spec);
    overflow([&] { (void)coil.compute_spring_force(30.); }); overflow([&] { (void)writer.components(fixture.data.get()); });
}
void test_rolling_and_drag_computed_overflow() {
    bt::ModelFixture fixture(R"(<mujoco><worldbody><body name="frame"><joint name="root_x" type="slide" axis="1 0 0"/><geom size=".1" mass="1"/>
      <body name="front_wheel"><joint name="front_wheel_spin" type="hinge" axis="0 1 0"/><geom size=".3" mass="1"/></body>
      <body name="rear_wheel"><joint name="rear_wheel_spin" type="hinge" axis="0 1 0"/><geom size=".3" mass="1"/></body>
      </body></worldbody></mujoco>)");
    engine::forward(fixture.model.get(), fixture.data.get());
    nativecfg::ResistanceConfig config{.crr=1e308, .rolling_taper_rad_s=.2, .rho_kg_m3=1., .cda_m2=1.,
        .wind_world_mps={}, .point_body_m={}, .frame_body="frame", .front_wheel_body="front_wheel", .rear_wheel_body="rear_wheel"};
    const std::array<double, 1> load{10.}, none{0.}; const std::array<bool, 1> working{true};
    ResistanceWriter rolling(fixture.model.get(), config);
    auto velocity = model_access::mutable_buffer(fixture.data->qvel, fixture.model->nv);
    velocity[static_cast<std::size_t>(fixture.dof("front_wheel_spin"))] = 1.;
    overflow([&] { (void)rolling.components(fixture.data.get(), {load, working, .3}, {none, working, .3}); });
    config.crr = 0.; ResistanceWriter drag(fixture.model.get(), config);
    std::ranges::fill(velocity, 0.); velocity[static_cast<std::size_t>(fixture.dof("root_x"))] = 1e308;
    overflow([&] { (void)drag.components(fixture.data.get(), {none, working, .3}, {none, working, .3}); });
}
} // namespace
int main() { return bt::run([] {
    test_suspension_domain_is_shared(); test_damper_invalid_parameters_rejected();
    test_air_pressure_intermediate_overflow_is_shared(); test_activated_zero_hbo_denominator_rejected();
    test_mutated_policy_configs_are_rejected(); test_policy_derived_overflow_is_shared();
    test_cruise_longitudinal_topology_is_shared(); test_brake_wheel_topology_and_actuator_target_are_shared();
    test_coil_force_overflow_before_publication(); test_rolling_and_drag_computed_overflow();
}); }
