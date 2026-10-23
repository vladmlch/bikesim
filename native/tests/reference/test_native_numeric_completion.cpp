// Completion 3.21a: numeric-completion reference port. Wire/typed domain
// rejections and overflow-before-publication controls are re-asserted
// against the native boundary; the recorded oracle comparisons consume the
// two-platform goldens captured from the Python file (no live oracle).
#include "support/model_fixture.hpp"
#include "support/temporary_directory.hpp"
#include "drivetrain_reference_support.hpp"
#include "policy_reference_support.hpp"
#include "runtime/setup_values.hpp"
#include "writer_config.hpp"
#include "config_validation.hpp"
#include "configuration/model_config.hpp"
#include "configuration/physical_config.hpp"
#include "model/suspension_setup.hpp"
#include "writers/cruise.hpp"
#include "model_access.hpp"
#include "initialization/capture.hpp"
#include "drivetrain/policy_set.hpp"
#include "drivetrain/chain.hpp"
#include "drivetrain/motor.hpp"
#include "runtime/runtime.hpp"
#include "runtime/bootstrap_io.hpp"
#include "runtime/control.hpp"
#include "runtime/samples.hpp"
#include "engine_call.hpp"
#include "stepper.hpp"
#include "validation.hpp"
#include "io/json.hpp"
#include "diag.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace {
namespace io = bike::io;
namespace cfg = bike::configuration;
using namespace runtime::setupvalue;
using policy_reference::put;
// The oracle's pytest.raises(OverflowError) maps onto both the shared
// derived() overflow_error and the drivetrain writer's ArithmeticError.
template<class F> void overflow(F&& action) {
    bool rejected = false;
    try { std::forward<F>(action)(); }
    catch (const std::overflow_error&) { rejected = true; }
    catch (const drivetrain::ArithmeticError&) { rejected = true; }
    bt::require(rejected, "unrepresentable derived value throws overflow_error");
}

// ---- document plumbing ----------------------------------------------------

Value& member(Value& value, std::string_view key) {
    auto* object = std::get_if<Value::Object>(&value.data);
    if (!object) throw std::invalid_argument("fixture expected object");
    for (auto& [name, item] : *object) if (name == key) return item;
    throw std::invalid_argument("fixture missing " + std::string(key));
}
// The Python file's `at` helper: dict keys descend into objects, integer
// keys index arrays (the 'paths.0' location).
Value& at(Value& value, std::span<const std::string_view> path) {
    Value* cursor = &value;
    for (const auto key : path) {
        if (std::holds_alternative<Value::Object>(cursor->data)) {
            cursor = &member(*cursor, key);
        } else {
            auto* array = std::get_if<Value::Array>(&cursor->data);
            if (!array) throw std::invalid_argument("fixture path needs object/array");
            int index = 0;
            for (const char c : key) index = index * 10 + (c - '0');
            cursor = &(*array)[static_cast<std::size_t>(index)];
        }
    }
    return *cursor;
}
// Call-site literal lists are safe to pass by initializer_list (the backing
// array outlives the call); stored Case paths use vectors instead.
Value& at(Value& value, std::initializer_list<std::string_view> path) {
    return at(value, std::span<const std::string_view>(path));
}

// ---- the resolved() fixture ------------------------------------------------
// test_native_input_contracts.resolved: _pinned_config physics (spindle/pin/
// connect welded plant — the welded_pinned bootstrap), project(env) plus the
// cruise/assist/rider-path/surface-section overrides the fixture applies.
std::string pinned_path() {
    return bt::fixture("bootstrap/welded_pinned/model.mjb").string();
}
const std::vector<double>& pinned_integration() {
    static const std::vector<double> state =
        bt::read_f64(bt::fixture("bootstrap/welded_pinned/integration.f64"));
    return state;
}
// mjSTATE_INTEGRATION layout: [time, qpos(26), qvel(26), act(0),
// qacc_warmstart(26), plugin] for the nq=nv=26, na=0 plant.
std::vector<double> pinned_qpos() {
    const auto& v = pinned_integration();
    return {v.begin() + 1, v.begin() + 27};
}
std::vector<double> pinned_qvel() {
    const auto& v = pinned_integration();
    return {v.begin() + 27, v.begin() + 53};
}
std::vector<double> pinned_warmstart() {
    const auto& v = pinned_integration();
    return {v.begin() + 53, v.begin() + 79};
}
int pinned_adr(const std::string& joint) {
    static const bike::model::ModelOwner model(
        engine::load_model(pinned_path().c_str()));
    return bike::model::joint_address(*model, joint).first;
}
int pinned_dof(const std::string& joint) {
    static const bike::model::ModelOwner model(
        engine::load_model(pinned_path().c_str()));
    return bike::model::joint_address(*model, joint).second;
}

Value resolved_wire() {
    auto wire = field(io::parse_json(
        bt::text(bt::fixture("bootstrap/welded_pinned/config.json"))), "writer_config");
    set(wire, "cruise", io::parse_json(
        R"({"target_speed_kmh":25.0,"kp_nm_per_mps":1.0,"ki_nm_per_mps_s":0.1,"torque_ceiling_nm":20.0})"));
    auto& assist = member(member(wire, "drive"), "assist");
    set(assist, "profile", io::parse_json(
        R"({"mode_gains":{"eco":1.0,"tour":2.0,"emtb":[1.0,3.0],"turbo":3.0},"emtb_full_gain_at_nm":20.0})"));
    set(assist, "mode", text("eco"));
    set(wire, "rider_forces", io::parse_json(
        R"({"paths":[{"name":"probe_path","joint":"root_x","stiffness_n_m":1.0,"damping_ns_m":0.0,"preload_deflection_m":0.0,"offset_m":0.0,"unilateral":false}]})"));
    auto& surface_map = member(member(wire, "tire"), "surface_map");
    auto& sections = member(surface_map, "sections");
    if (io::as_array(sections, "tire.surface_map.sections").empty()) {
        // surface=copy.deepcopy(config['tire']['surface_map']['surface'])
        const Value surface = field(surface_map, "surface");
        Value entry{Value::Object{{"start_m", real(1.)}, {"end_m", real(2.)},
                                  {"surface", surface}}};
        sections = Value{Value::Array{std::move(entry)}};
    }
    return wire;
}
std::unique_ptr<Stepper> resolved_stepper(const Value& wire) {
    return std::make_unique<Stepper>(pinned_path(), nativecfg::decode(wire));
}
// Live env.sim.data replay: captured qpos/qvel, empty act, captured
// warmstart — the shared-state tests (shock overflow, unload clamp).
void set_live(Stepper& stepper, std::vector<double> qpos,
              std::vector<double> qvel) {
    stepper.set_state(std::move(qpos), std::move(qvel), {}, pinned_warmstart(), 0.);
}
// Fresh MjData replay: qpos taken from the live state, zero qvel/act/
// warmstart (mujoco.MjData(model) zero-initializes them).
void set_fresh(Stepper& stepper, std::vector<double> qpos,
               std::vector<double> qvel) {
    stepper.set_state(std::move(qpos), std::move(qvel), {},
                      std::vector<double>(pinned_qpos().size(), 0.), 0.);
}
// python_wire_config()'s typed half: every decoded section re-validates
// through the shared nativecfg/drivetrain validate faces.
void typed_wire_config(const nativecfg::WriterConfig& config) {
    if (config.suspension) nativecfg::validate(*config.suspension);
    if (config.brake) nativecfg::validate(*config.brake);
    if (config.cruise) nativecfg::validate(*config.cruise);
    if (config.resistance) nativecfg::validate(*config.resistance);
    if (config.tire) nativecfg::validate(*config.tire);
    if (config.drive) drivetrain::validate(*config.drive);
    if (config.rider_forces) nativecfg::validate(*config.rider_forces);
}

// ---- golden plumbing --------------------------------------------------------

bt::oracle::Node node(std::string_view function, std::string_view id = {}) {
    auto name = "tests/reference/test_native_numeric_completion.py::" +
                std::string(function);
    if (!id.empty()) name += "[" + std::string(id) + "]";
    return bt::oracle::Node(name);
}
void scalar(bt::oracle::Node& golden, double value) {
    const std::array values{value};
    golden.f64(values, std::span<const std::size_t>{}, "");
}
void array(bt::oracle::Node& golden, std::span<const double> values) {
    const std::array shape{values.size()};
    golden.f64(values, shape, "");
}
// _record_tree_close's Wire twin: object → #keys (sorted), array → #length
// + indexed leaves, numbers → close f64, other leaves → canonical JSON.
void close_scalar(bt::oracle::Node& golden, double value, const std::string& path) {
    const std::array values{value};
    golden.f64_close(values, std::span<const std::size_t>{}, path);
}
void wire_close(bt::oracle::Node& golden, const runtime::Wire& wire,
                const std::string& path) {
    if (const auto* object = std::get_if<runtime::WireObject>(&wire.value)) {
        std::vector<std::string> names;
        for (const auto& [key, ignored] : *object) { (void)ignored; names.push_back(key); }
        std::ranges::sort(names);
        Value::Array keys;
        for (const auto& name : names) keys.push_back(text(name));
        policy_reference::json_bytes(golden, Value{std::move(keys)}, path + "#keys");
        for (const auto& name : names)
            for (const auto& [key, child] : *object)
                if (key == name) wire_close(golden, child, path + "." + name);
    } else if (const auto* items = std::get_if<runtime::WireArray>(&wire.value)) {
        policy_reference::json_bytes(golden,
            Value{bike::io::Number{std::to_string(items->size())}}, path + "#length");
        for (std::size_t i = 0; i < items->size(); ++i)
            wire_close(golden, (*items)[i], path + "[" + std::to_string(i) + "]");
    } else if (const auto* number = std::get_if<double>(&wire.value)) {
        close_scalar(golden, *number, path);
    } else if (const auto* count = std::get_if<std::int64_t>(&wire.value)) {
        close_scalar(golden, static_cast<double>(*count), path);
    } else if (const auto* flag = std::get_if<bool>(&wire.value)) {
        policy_reference::json_bytes(golden, Value{*flag}, path);
    } else if (const auto* string = std::get_if<std::string>(&wire.value)) {
        policy_reference::json_bytes(golden, text(*string), path);
    } else {
        policy_reference::json_bytes(golden, Value{}, path);
    }
}
const runtime::Wire& wire_member(const runtime::WireObject& object,
                                 std::string_view name) {
    for (const auto& [key, value] : object)
        if (key == name) return value;
    throw std::invalid_argument("wire missing " + std::string(name));
}

// ---- tests ----------------------------------------------------------------

// test_air_specs_mutated_geometry_rejected[key]
void test_air_specs_mutated_geometry_rejected() {
    std::fprintf(stderr, ">> test_air_specs_mutated_geometry_rejected\n");
    for (const auto key : {"piston_area_m2", "token_volume_m3",
                           "base_pos_volume_m3", "base_neg_volume_m3"}) {
        const bike::model::AirSpringSpecs specs{};
        double derived_value;
        if (std::string_view(key) == "piston_area_m2") derived_value = specs.piston_area_m2();
        else if (std::string_view(key) == "token_volume_m3") derived_value = specs.token_volume_m3();
        else if (std::string_view(key) == "base_pos_volume_m3") derived_value = specs.base_pos_volume_m3();
        else derived_value = specs.base_neg_volume_m3();
        bt::require(std::isfinite(derived_value), "valid spec derived property is finite");
        auto mutated = specs;
        mutated.neg_chamber_length_mm = -1.;
        bt::invalid([&] { bike::model::validate_air_spring_specs(mutated); });
        bt::invalid([&] { bike::model::AirSpringSpecs rejected{.neg_chamber_length_mm = -1.};
                          bike::model::validate_air_spring_specs(rejected); });
        bt::invalid([&] { bike::model::ForkAirSpring rejected(mutated); });
    }
}

// test_air_pressure_properties_revalidate[key]
void test_air_pressure_properties_revalidate() {
    std::fprintf(stderr, ">> test_air_pressure_properties_revalidate\n");
    for (const auto key : {"abs_pressure_pa", "gauge_pressure_bar"}) {
        bike::model::ForkAirSpring spring;
        // The oracle stores a non-finite gauge pressure (the strict-type
        // rejection is the wire half); the property getter revalidates.
        spring.set_gauge_pressure_psi(std::numeric_limits<double>::quiet_NaN());
        bt::invalid([&] {
            if (std::string_view(key) == "abs_pressure_pa") (void)spring.abs_pressure_pa();
            else (void)spring.gauge_pressure_bar();
        });
    }
}

// test_air_specs_mutated_token_limit_rejected
void test_air_specs_mutated_token_limit_rejected() {
    std::fprintf(stderr, ">> test_air_specs_mutated_token_limit_rejected\n");
    bike::model::AirSpringSpecs specs;
    specs.max_tokens = -1;
    for (const auto& construct : std::array<std::function<void()>, 3>{
        [&] { bike::model::validate_air_spring_specs(specs); },
        [&] { bike::model::ForkAirSpring rejected(specs); },
        [&] { bike::model::AirSpringSpecs rejected{.max_tokens = -1};
              bike::model::validate_air_spring_specs(rejected); }})
        bt::invalid(construct);
}

// test_invalid_tire_material_rejected[key-value]
void test_invalid_tire_material_rejected() {
    std::fprintf(stderr, ">> test_invalid_tire_material_rejected\n");
    const nativecfg::TireMaterial base{.radial_k_n_m = 1e5, .radial_c_ns_m = 100.,
        .pressure_pa_gauge = 1e5, .provenance = "synthetic",
        .valid_load_range_n = {0., 2000.}, .table = std::nullopt};
    for (const auto key : {"pressure_pa_gauge", "provenance",
                           "valid_load_range_n", "radial_k_n_m"}) {
        // radial_k_n_m mutates to the oracle's bool — a strict-type wire
        // rejection; the other mutations are typed-domain re-validations.
        if (std::string_view(key) == "radial_k_n_m") {
            auto wire = resolved_wire();
            set(at(wire, {"tire", "front", "material"}), "radial_k_n_m", Value{true});
            bt::invalid([&] { (void)nativecfg::decode(wire); });
            bt::invalid([&] { (void)resolved_stepper(wire); });
            continue;
        }
        auto mutated = base;
        if (std::string_view(key) == "pressure_pa_gauge") mutated.pressure_pa_gauge = -1.;
        else if (std::string_view(key) == "provenance") mutated.provenance = "";
        else mutated.valid_load_range_n = {2., 1.};
        bt::invalid([&] { nativecfg::validate(mutated); });
    }
}

// test_mutated_surface_map_rejected[mutation]
void test_mutated_surface_map_rejected() {
    std::fprintf(stderr, ">> test_mutated_surface_map_rejected\n");
    const auto config = nativecfg::decode(resolved_wire());
    bt::require(config.tire.has_value() && config.tire->surface_map.has_value(),
                "resolved wire carries a track surface map");
    for (const auto mutation : {"surface", "section", "overlap"}) {
        auto tire = *config.tire;
        if (std::string_view(mutation) == "surface") {
            tire.surface_map->surface.mu_slide = 2.;
            bt::invalid([&] { nativecfg::validate(tire.surface_map->surface); });
        } else if (std::string_view(mutation) == "section") {
            tire.surface_map->sections[0].start_m = -1.;
            bt::invalid([&] { nativecfg::validate(tire); });
        } else {
            const auto surface = tire.surface_map->surface;
            tire.surface_map->sections = {nativecfg::SurfaceSection{0., 2., surface},
                                          nativecfg::SurfaceSection{1., 3., surface}};
            bt::invalid([&] { nativecfg::validate(tire); });
        }
    }
}

// test_mutated_drive_config_rejected_before_model_writes[key-value]
void test_mutated_drive_config_rejected_before_model_writes() {
    std::fprintf(stderr, ">> test_mutated_drive_config_rejected_before_model_writes\n");
    struct Case { std::string_view id, key; double numeric; bool wire_text; };
    const std::array cases{
        Case{"chain_k_n_m-0.0", "chain_k_n_m", 0., false},
        Case{"bearing_c_nms_rad--1.0", "bearing_c_nms_rad", -1., false},
        Case{"motor_clutch-no", "motor_clutch", 0., true},
        Case{"torque_ripple-1.0", "torque_ripple", 1., false}};
    for (const auto& item : cases) {
        auto golden = node("test_mutated_drive_config_rejected_before_model_writes", item.id);
        bt::ModelFixture fixture(drivetrain_reference::model_xml("ideal_mid_drive"));
        const auto image = bike::initialization::capture_model_image(*fixture.model);
        auto wire = drivetrain_reference::writer_config(drivetrain_reference::source());
        if (item.wire_text) {
            // The oracle setattr's 'no' onto the bool — the wire half is the
            // strict-type rejection at decode.
            put(wire, "drive", "motor_clutch", text("no"));
            bt::invalid([&] { (void)nativecfg::decode(wire); });
            bt::invalid([&] { Stepper rejected(image.mjb, nativecfg::decode(wire)); });
        } else {
            auto typed = nativecfg::decode(wire);
            auto drive = *typed.drive;
            if (std::string_view(item.key) == "chain_k_n_m") drive.chain_k_n_m = item.numeric;
            else if (std::string_view(item.key) == "bearing_c_nms_rad") drive.bearing_c_nms_rad = item.numeric;
            else drive.torque_ripple = item.numeric;
            bt::invalid([&] { drivetrain::validate(drive); });
            auto mutated = typed;
            mutated.drive = drive;
            bt::invalid([&] { Stepper rejected(image.mjb, mutated); });
        }
        const std::array shape{static_cast<std::size_t>(fixture.model->nwrap)};
        golden.f64(model_access::readonly_buffer(
                       fixture.model->wrap_prm, fixture.model->nwrap),
                   shape, "");
        golden.done();
    }
}

// test_drivetrain_wheel_topology_shared[axis-kind]
void test_drivetrain_wheel_topology_shared() {
    std::fprintf(stderr, ">> test_drivetrain_wheel_topology_shared\n");
    for (const auto& [axis, kind] : std::array<std::pair<const char*, const char*>, 2>{
        std::pair{"0 0 1", "hinge"}, std::pair{"0 1 0", "slide"}}) {
        auto text_xml = drivetrain_reference::model_xml("elastic_chain");
        const std::string needle = "<joint name=\"front_wheel_spin\" axis=\"0 1 0\"/>";
        const auto position = text_xml.find(needle);
        bt::require(position != std::string::npos, "drivetrain xml carries the front-wheel joint");
        text_xml.replace(position, needle.size(),
            "<joint name=\"front_wheel_spin\" type=\"" + std::string(kind) +
                "\" axis=\"" + axis + "\"/>");
        bt::ModelFixture fixture(text_xml);
        const auto image = bike::initialization::capture_model_image(*fixture.model);
        auto wire = drivetrain_reference::writer_config(drivetrain_reference::source("elastic_chain"));
        put(wire, "drive", "transmission_model", text("elastic_chain"));
        drivetrain_reference::error<std::invalid_argument>(
            [&] { Stepper rejected(image.mjb, nativecfg::decode(wire)); }, "front_wheel_spin");
    }
}

// test_drivetrain_actuator_target_shared[actuator]
void test_drivetrain_actuator_target_shared() {
    std::fprintf(stderr, ">> test_drivetrain_actuator_target_shared\n");
    for (const auto actuator : {"human_crank", "mid_drive"}) {
        auto text_xml = drivetrain_reference::model_xml("ideal_mid_drive");
        const std::string needle = "<motor name=\"" + std::string(actuator) + "\" joint=\"crank_spin\"/>";
        const auto position = text_xml.find(needle);
        bt::require(position != std::string::npos, "drivetrain xml carries the actuator");
        text_xml.replace(position, needle.size(),
            "<motor name=\"" + std::string(actuator) + "\" joint=\"front_wheel_spin\"/>");
        bt::ModelFixture fixture(text_xml);
        const auto image = bike::initialization::capture_model_image(*fixture.model);
        const auto wire = drivetrain_reference::writer_config(drivetrain_reference::source());
        drivetrain_reference::error<std::invalid_argument>(
            [&] { Stepper rejected(image.mjb, nativecfg::decode(wire)); }, actuator);
    }
}

// test_shock_component_overflow_shared[legacy]
void test_shock_component_overflow_shared() {
    std::fprintf(stderr, ">> test_shock_component_overflow_shared\n");
    const auto shock_dof = pinned_dof("shock_stroke");
    for (const bool legacy : {false, true}) {
        auto wire = resolved_wire();
        auto& shock = member(member(wire, "suspension"), "shock_damper");
        set(shock, "legacy_behavior", Value{legacy});
        set(shock, "lockout_firm", Value{true});
        set(shock, "c_hsc_min", real(1e308)); set(shock, "c_hsc_max", real(1e308));
        bike::model::SuperDeluxeDamper damper;
        damper.legacy_behavior = legacy; damper.lockout_firm = true;
        damper.c_hsc_min = damper.c_hsc_max = 1e308;
        overflow([&] { (void)damper.compute_damping_force(10., 30.); });
        auto stepper = resolved_stepper(wire);
        auto velocity = pinned_qvel();
        velocity[static_cast<std::size_t>(shock_dof)] = 10.;
        set_live(*stepper, pinned_qpos(), velocity);
        overflow([&] { (void)stepper->suspension_components(); });
    }
}

// test_chain_computation_overflow_shared[name]
void test_chain_computation_overflow_shared() {
    std::fprintf(stderr, ">> test_chain_computation_overflow_shared\n");
    for (const auto name : {"chain_tension", "chain_geometry"}) {
        if (std::string_view(name) == "chain_tension") {
            auto source = drivetrain_reference::source("elastic_chain");
            source.chain_k_n_m = 1e308; source.chain_c_ns_m = 1e308;
            auto owner = drivetrain_reference::pair(source);
            const auto crank = bike::model::joint_address(*owner->model(), "crank_spin");
            drivetrain_reference::position(*owner, "crank_spin",
                owner->qpos()[static_cast<std::size_t>(crank.first)] + 1.);
            drivetrain_reference::velocity(*owner, "crank_spin", 1000.);
            drivetrain_reference::reset_engine_state(*owner);
            overflow([&] { (void)drivetrain_reference::components(*owner, .001, 0.); });
        } else {
            overflow([&] {
                (void)drivetrain::chain_geometry(drivetrain::Vec2{0., 0.},
                    drivetrain::Vec2{1e308, 0.}, .1, .05, drivetrain::Vec2{0., 1.}, std::nullopt); });
        }
    }
}

// test_policy_intermediate_overflow_shared[policy]
void test_policy_intermediate_overflow_shared() {
    std::fprintf(stderr, ">> test_policy_intermediate_overflow_shared\n");
    for (const auto policy : {"assist", "pedaling", "shifting"}) {
        auto wire = policy_reference::config(policy_reference::source());
        if (std::string_view(policy) == "assist") {
            put(wire, "assist", "slew", real(1e308));
            auto policies = policy_reference::policies(wire);
            overflow([&] { (void)policies.assist.step(10., 80., 0., false, 10.); });
        } else if (std::string_view(policy) == "pedaling") {
            put(wire, "pedaling", "stop_time_s", real(1e-308));
            auto policies = policy_reference::policies(wire);
            overflow([&] { (void)policies.pedaling.update(0., 10., 80., 0., .001); });
        } else {
            auto policies = policy_reference::policies(wire);
            auto state = policies.snapshot();
            state.shifting.cadence_ema = 1e308; state.shifting.required_ema = 1e308;
            policies.restore(state);
            overflow([&] { (void)policies.shifting.update(-1e308, -1e308, .001); });
        }
    }
}

// test_cruise_overflow_preserves_finite_telemetry
void test_cruise_overflow_preserves_finite_telemetry() {
    std::fprintf(stderr, ">> test_cruise_overflow_preserves_finite_telemetry\n");
    bt::ModelFixture fixture(R"(<mujoco><option timestep="1e308"/><worldbody><body><joint name="root_x" type="slide" axis="1 0 0"/><geom size=".1" mass="1"/></body></worldbody></mujoco>)");
    const auto image = bike::initialization::capture_model_image(*fixture.model);
    const auto wire = io::parse_json(
        R"({"schema":2,"cruise":{"target_speed_kmh":25.0,"kp_nm_per_mps":1e-308,"ki_nm_per_mps_s":1e-308,"torque_ceiling_nm":20.0}})");
    const auto config = nativecfg::decode(wire);
    Stepper stepper(image.mjb, config);
    const CruiseState before = stepper.cruise_state();
    overflow([&] { (void)stepper.cruise_compute(true, false, std::nullopt); });
    CruiseWriter writer(fixture.model.get(), *config.cruise);
    overflow([&] { (void)writer.compute(fixture.data.get(), true, false, std::nullopt); });
    const CruiseState after = stepper.cruise_state();
    bt::require(std::bit_cast<std::uint64_t>(before.target_speed_mps) ==
                    std::bit_cast<std::uint64_t>(after.target_speed_mps) &&
                std::bit_cast<std::uint64_t>(before.integral_mps_s) ==
                    std::bit_cast<std::uint64_t>(after.integral_mps_s) &&
                std::bit_cast<std::uint64_t>(before.torque_nm) ==
                    std::bit_cast<std::uint64_t>(after.torque_nm) &&
                before.engaged == after.engaged &&
                std::bit_cast<std::uint64_t>(before.gain_scale) ==
                    std::bit_cast<std::uint64_t>(after.gain_scale),
                "cruise compute overflow preserves the standalone PI state");
}

// test_native_damper_errors_name_full_field_path[section-key]
void test_native_damper_errors_name_full_field_path() {
    std::fprintf(stderr, ">> test_native_damper_errors_name_full_field_path\n");
    for (const auto& [section, key] : std::array<std::pair<const char*, const char*>, 2>{
        std::pair{"fork_damper", "v_knee_comp"}, std::pair{"shock_damper", "max_lsc"}}) {
        auto wire = resolved_wire();
        set(member(member(wire, "suspension"), section), key, real(0.));
        drivetrain_reference::error<std::invalid_argument>(
            [&] { (void)resolved_stepper(wire); },
            "config.suspension." + std::string(section) + "." + key);
    }
}

// test_nested_numeric_errors_name_full_field_path[case]
void test_nested_numeric_errors_name_full_field_path() {
    std::fprintf(stderr, ">> test_nested_numeric_errors_name_full_field_path\n");
    struct Case { std::vector<std::string_view> path; double bad; std::string_view expected; };
    const std::array cases{
        Case{{"tire", "front", "material", "radial_c_ns_m"}, -1., "config.tire.front.material.radial_c_ns_m"},
        Case{{"tire", "rear", "relaxation_length_m"}, 0., "config.tire.rear.relaxation_length_m"},
        Case{{"drive", "assist", "profile", "mode_gains", "eco"}, -1., "config.drive.assist.profile.mode_gains.eco"},
        Case{{"rider_forces", "paths", "0", "stiffness_n_m"}, -1., "config.rider_forces.paths.0.stiffness_n_m"}};
    for (const auto& item : cases) {
        auto wire = resolved_wire();
        at(wire, item.path) = real(item.bad);
        drivetrain_reference::error<std::invalid_argument>(
            [&] { (void)resolved_stepper(wire); }, item.expected);
    }
}

// test_end_stop_strict_types_and_computation_overflow — the oracle calls
// end_stop() directly; the native end_stop is the suspension writer's
// internal, so the port exercises its wire/config boundary: a bool stiffness
// is the strict-type ValueError, a 1e308 stiffness with a deep deflection is
// the same k*low_depth overflow the direct call reaches.
void test_end_stop_strict_types_and_computation_overflow() {
    std::fprintf(stderr, ">> test_end_stop_strict_types_and_computation_overflow\n");
    auto wire = resolved_wire();
    auto& stops = member(member(wire, "suspension"), "end_stops");
    set(stops, "stiffness_n_m", Value{true});
    bt::invalid([&] { (void)nativecfg::decode(wire); });
    bt::invalid([&] { (void)resolved_stepper(wire); });
    auto wire2 = resolved_wire();
    set(member(member(wire2, "suspension"), "end_stops"), "stiffness_n_m", real(1e308));
    auto stepper = resolved_stepper(wire2);
    auto position = pinned_qpos();
    position[static_cast<std::size_t>(pinned_adr("shock_stroke"))] = -10.;
    set_fresh(*stepper, position, std::vector<double>(pinned_qpos().size(), 0.));
    overflow([&] { (void)stepper->suspension_components(); });
}

struct FieldCase { std::vector<std::string_view> path; };
const std::initializer_list<FieldCase> kPositiveFields = {
    {{"suspension", "fork_damper", "total_travel_mm"}},
    {{"suspension", "shock_damper", "total_stroke_mm"}},
    {{"suspension", "shock_damper", "max_hbo"}},
    {{"brake", "taper_radps"}},
    {{"cruise", "kp_nm_per_mps"}}, {{"cruise", "ki_nm_per_mps_s"}},
    {{"cruise", "torque_ceiling_nm"}},
    {{"resistance", "rolling_taper_rad_s"}},
    {{"tire", "front", "tangent_k_n_m"}}, {{"tire", "front", "relaxation_length_m"}},
    {{"tire", "rear", "tangent_k_n_m"}}, {{"tire", "rear", "relaxation_length_m"}},
    {{"tire", "front", "material", "radial_k_n_m"}},
    {{"tire", "rear", "material", "radial_k_n_m"}},
    {{"tire", "surface_map", "surface", "mu_peak"}},
    {{"tire", "surface_map", "surface", "mu_slide"}},
    {{"tire", "surface_map", "surface", "slip_stiffness_per_load"}},
    {{"tire", "surface_map", "surface", "stribeck_speed_mps"}},
    {{"drive", "gearing", "chain_pitch_m"}},
    {{"drive", "chain_k_n_m"}},
    {{"drive", "hub_stiffness_nm_rad"}},
    {{"drive", "pedaling", "coast_above_rpm"}},
    {{"drive", "pedaling", "stop_time_s"}},
    {{"drive", "shifting", "target_cadence_min_rpm"}},
    {{"drive", "shifting", "target_cadence_max_rpm"}},
    {{"drive", "assist", "tau"}},
    {{"drive", "assist", "slew"}},
    {{"drive", "assist", "taper_width_mps"}},
};
const std::initializer_list<FieldCase> kNonnegativeFields = {
    {{"suspension", "fork_damper", "hbo_start_mm"}},
    {{"suspension", "fork_damper", "c_hbo_base"}},
    {{"suspension", "shock_damper", "hbo_start_mm"}},
    {{"suspension", "shock_damper", "c_hbo_min"}},
    {{"suspension", "shock_damper", "c_hbo_max"}},
    {{"suspension", "shock_damper", "lockout_preload_n"}},
    {{"suspension", "shock_damper", "lockout_stiffness"}},
    {{"brake", "torque_ceiling_nm"}},
    {{"resistance", "crr"}}, {{"resistance", "rho_kg_m3"}}, {{"resistance", "cda_m2"}},
    {{"tire", "front", "mu"}}, {{"tire", "rear", "mu"}},
    {{"tire", "front", "material", "radial_c_ns_m"}},
    {{"tire", "front", "material", "pressure_pa_gauge"}},
    {{"tire", "rear", "material", "radial_c_ns_m"}},
    {{"tire", "rear", "material", "pressure_pa_gauge"}},
    {{"tire", "significant_delta_m"}},
    {{"tire", "significance_fraction"}},
    {{"drive", "human_torque_nm"}}, {{"drive", "torque_ripple"}},
    {{"drive", "chain_c_ns_m"}}, {{"drive", "bearing_c_nms_rad"}},
    {{"drive", "rotor_inertia_kgm2"}}, {{"drive", "hub_damping_nm_s"}},
    {{"drive", "pedaling", "resume_below_rpm"}},
    {{"drive", "pedaling", "coast_cadence_tau_s"}},
    {{"drive", "pedaling", "mash_cadence_rpm"}},
    {{"drive", "pedaling", "mash_torque_nm"}},
    {{"drive", "pedaling", "effort_slew_nm_s"}},
    {{"drive", "shifting", "shift_cooldown_s"}},
    {{"drive", "shifting", "shift_cut_duration_s"}},
    {{"drive", "shifting", "torque_factor"}},
    {{"drive", "shifting", "cadence_smoothing_tau_s"}},
    {{"drive", "shifting", "upshift_slip_limit_mps"}},
    {{"drive", "assist", "gain"}}, {{"drive", "assist", "max_torque"}},
    {{"drive", "assist", "max_power"}}, {{"drive", "assist", "engage_torque_nm"}},
    {{"drive", "assist", "gate_min_crank_rad_s"}}, {{"drive", "assist", "cutoff_mps"}},
    {{"drive", "battery", "energy_j"}}, {{"drive", "battery", "copper_w_per_nm2"}},
    {{"drive", "battery", "speed_w_per_rad_s2"}}, {{"drive", "battery", "idle_w"}},
};

Value without_profile() {
    auto wire = resolved_wire();
    set(member(member(wire, "drive"), "assist"), "profile", Value{});
    return wire;
}
// test_remaining_s4_scalar_field_matrix[location-bad]
void test_remaining_s4_scalar_field_matrix() {
    std::fprintf(stderr, ">> test_remaining_s4_scalar_field_matrix\n");
    const std::array bad_values{real(-1.), Value{bike::io::Number{"nan"}},
        Value{bike::io::Number{"inf"}}, Value{bike::io::Number{"-inf"}}, Value{true}};
    for (const auto* list : {&kPositiveFields, &kNonnegativeFields})
        for (const auto& item : *list) for (const auto& bad : bad_values) {
            auto wire = without_profile();
            at(wire, item.path) = bad;
            bt::invalid([&] { typed_wire_config(nativecfg::decode(wire)); });
            bt::invalid([&] { (void)resolved_stepper(wire); });
        }
}
// test_remaining_s4_positive_field_zero[location]
void test_remaining_s4_positive_field_zero() {
    std::fprintf(stderr, ">> test_remaining_s4_positive_field_zero\n");
    for (const auto& item : kPositiveFields) {
        auto wire = without_profile();
        at(wire, item.path) = integer(0);
        bt::invalid([&] { typed_wire_config(nativecfg::decode(wire)); });
        bt::invalid([&] { (void)resolved_stepper(wire); });
    }
}
// test_s4_matrix_valid_configuration
void test_s4_matrix_valid_configuration() {
    std::fprintf(stderr, ">> test_s4_matrix_valid_configuration\n");
    const auto config = nativecfg::decode(without_profile());
    typed_wire_config(config);
    (void)resolved_stepper(without_profile());
}

// test_s4_relationship_matrix[location-value]
void test_s4_relationship_matrix() {
    std::fprintf(stderr, ">> test_s4_relationship_matrix\n");
    struct Case { std::vector<std::string_view> path; const char* value; };
    const std::array cases{
        Case{{"drive", "assist", "taper_width_mps"}, "100.0"},
        Case{{"drive", "assist", "torque_curve"}, "[[0.0,1.0],[0.0,2.0]]"},
        Case{{"drive", "shifting", "cassette"}, "[24,24]"},
        Case{{"drive", "shifting", "target_cadence_min_rpm"}, "200.0"},
        Case{{"drive", "shifting", "shift_cut_duration_s"}, "100.0"},
        Case{{"drive", "shifting", "torque_factor"}, "1.1"},
        Case{{"drive", "pedaling", "resume_below_rpm"}, "200.0"},
        Case{{"tire", "significance_fraction"}, "1.1"},
        Case{{"tire", "distinct_normal_deg"}, "180.0"},
        Case{{"tire", "front", "material", "valid_load_range_n"}, "[2000.0,1000.0]"},
        Case{{"tire", "surface_map", "surface", "mu_slide"}, "20.0"},
        Case{{"suspension", "fork_damper", "c_lsc_min"}, "1e6"},
        Case{{"suspension", "shock_damper", "c_hbo_min"}, "1e6"},
        Case{{"suspension", "coil", "bumper_length_mm"}, "100.0"}};
    for (const auto& item : cases) {
        auto wire = without_profile();
        at(wire, item.path) = io::parse_json(item.value);
        bt::invalid([&] { typed_wire_config(nativecfg::decode(wire)); });
        bt::invalid([&] { (void)resolved_stepper(wire); });
    }
}

// test_unicode_blank_tire_provenance_shared[side]
void test_unicode_blank_tire_provenance_shared() {
    std::fprintf(stderr, ">> test_unicode_blank_tire_provenance_shared\n");
    for (const auto side : {"front", "rear"}) {
        auto wire = resolved_wire();
        set(at(wire, {"tire", side, "material"}), "provenance", text("\u2003"));
        bt::invalid([&] { typed_wire_config(nativecfg::decode(wire)); });
        drivetrain_reference::error<std::invalid_argument>(
            [&] { (void)resolved_stepper(wire); }, "provenance");
    }
}

// test_resistance_requires_physical_bodies[side]
void test_resistance_requires_physical_bodies() {
    std::fprintf(stderr, ">> test_resistance_requires_physical_bodies\n");
    for (const auto side : {"frame", "front_wheel", "rear_wheel"}) {
        auto wire = resolved_wire();
        set(at(wire, {"resistance", "bodies"}), side, text("world"));
        bt::invalid([&] { (void)resolved_stepper(wire); });
    }
}

// test_cassette_ordered_integer_contract[field-value]
void test_cassette_ordered_integer_contract() {
    std::fprintf(stderr, ">> test_cassette_ordered_integer_contract\n");
    struct Case { Value value; bool valid; };
    // {12, 24} is a set (non-sequence → an object wire), (24, True) and
    // (24, 24) are the bool/non-ordered rejections; the np.int64 tuple and
    // the plain list are the accepted controls.
    const std::array cases{
        Case{io::parse_json(R"({"12":null,"24":null})"), false},
        Case{io::parse_json("[24,true]"), false},
        Case{io::parse_json("[24,24]"), false},
        Case{io::parse_json("[24,32]"), true},
        Case{io::parse_json("[12,24]"), true}};
    for (const auto& item : cases) {
        auto wire = policy_reference::config(policy_reference::source());
        put(wire, "shifting", "cassette", item.value);
        if (item.valid) (void)policy_reference::policies(wire);
        else bt::invalid([&] { (void)policy_reference::policies(wire); });
    }
}

// test_mutated_tire_parameters_rejected[name-key]
void test_mutated_tire_parameters_rejected() {
    std::fprintf(stderr, ">> test_mutated_tire_parameters_rejected\n");
    const auto config = nativecfg::decode(resolved_wire());
    for (const auto& [side, key] : std::array<std::pair<const char*, const char*>, 2>{
        std::pair{"front", "mu"}, std::pair{"rear", "tangent_k_n_m"}}) {
        auto parameters = std::string_view(side) == "front" ? config.tire->front : config.tire->rear;
        if (std::string_view(key) == "mu") parameters.mu = -1.;
        else parameters.tangent_k_n_m = -1.;
        bt::invalid([&] { nativecfg::validate(parameters); });
    }
}

// test_damper_mutated_denominator_rejected_by_setter
void test_damper_mutated_denominator_rejected_by_setter() {
    std::fprintf(stderr, ">> test_damper_mutated_denominator_rejected_by_setter\n");
    bike::model::Charger3Damper damper;
    damper.max_hsc = 0;
    bt::invalid([&] { damper.set_clicks(1, std::nullopt, std::nullopt); });
}

// test_rider_path_scalar_matrix_shared[key-bad]
void test_rider_path_scalar_matrix_shared() {
    std::fprintf(stderr, ">> test_rider_path_scalar_matrix_shared\n");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const std::array bad_values{
        Value{bike::io::Number{"nan"}}, Value{bike::io::Number{"inf"}},
        Value{bike::io::Number{"-inf"}}, Value{true}};
    for (const auto key : {"stiffness_n_m", "damping_ns_m", "preload_deflection_m", "offset_m"})
        for (const auto& bad : bad_values) {
            auto wire = resolved_wire();
            set(at(wire, {"rider_forces", "paths", "0"}), key, bad);
            bt::invalid([&] { (void)resolved_stepper(wire); });
            auto typed = *nativecfg::decode(resolved_wire()).rider_forces;
            auto& row = typed.paths[0];
            if (std::string_view(key) == "stiffness_n_m") row.stiffness_n_m = nan;
            else if (std::string_view(key) == "damping_ns_m") row.damping_ns_m = nan;
            else if (std::string_view(key) == "preload_deflection_m") row.preload_deflection_m = nan;
            else row.offset_m = nan;
            bt::invalid([&] { nativecfg::validate(row); });
        }
}

// test_invalid_motor_profile_relationships_rejected[attribute-value]
void test_invalid_motor_profile_relationships_rejected() {
    std::fprintf(stderr, ">> test_invalid_motor_profile_relationships_rejected\n");
    for (const auto& [attribute, value] : std::array<std::pair<const char*, double>, 2>{
        std::pair{"rated_power_w", 1e6}, std::pair{"taper_width_mps", 100.}}) {
        auto profile = cfg::kBoschCxGen4;
        if (std::string_view(attribute) == "rated_power_w") profile.rated_power_w = value;
        else profile.taper_width_mps = value;
        bt::invalid([&] { cfg::detail::post_init(profile); });
    }
}

// test_bearing_force_overflow_before_return_and_telemetry
void test_bearing_force_overflow_before_return_and_telemetry() {
    std::fprintf(stderr, ">> test_bearing_force_overflow_before_return_and_telemetry\n");
    auto source = drivetrain_reference::source();
    source.bearing_c_nms_rad = 1e308;
    auto owner = drivetrain_reference::pair(source);
    drivetrain_reference::velocity(*owner, "crank_spin", 100.);
    drivetrain_reference::reset_engine_state(*owner);
    overflow([&] { (void)drivetrain_reference::components(*owner, .001, 0.); });
}

// test_chain_dissipation_overflow_before_telemetry
void test_chain_dissipation_overflow_before_telemetry() {
    std::fprintf(stderr, ">> test_chain_dissipation_overflow_before_telemetry\n");
    auto source = drivetrain_reference::source("elastic_chain");
    source.chain_k_n_m = 1e308; source.chain_c_ns_m = 1e108;
    source.battery.enabled = false;
    auto owner = drivetrain_reference::pair(source);
    const auto crank = bike::model::joint_address(*owner->model(), "crank_spin");
    drivetrain_reference::position(*owner, "crank_spin",
        owner->qpos()[static_cast<std::size_t>(crank.first)] + 1.);
    drivetrain_reference::velocity(*owner, "crank_spin", -1e200);
    drivetrain_reference::reset_engine_state(*owner);
    overflow([&] {
        (void)drivetrain_reference::components(*owner, .001, 0., {}, false); });
}

// test_suspension_total_overflow_before_telemetry
void test_suspension_total_overflow_before_telemetry() {
    std::fprintf(stderr, ">> test_suspension_total_overflow_before_telemetry\n");
    auto wire = resolved_wire();
    auto& air = member(member(wire, "suspension"), "air_spring");
    set(air, "stanchion_inner_diam_mm", real(1100.));
    set(air, "num_tokens", integer(0));
    set(air, "gauge_pressure_psi", real(1e304));
    auto& fork = member(member(wire, "suspension"), "fork_damper");
    set(fork, "c_lsc_min", real(1e308)); set(fork, "c_lsc_max", real(1e308));
    set(fork, "v_knee_comp", real(1.));
    auto stepper = resolved_stepper(wire);
    auto position = pinned_qpos();
    position[static_cast<std::size_t>(pinned_adr("fork_travel"))] = .1;
    auto velocity = std::vector<double>(pinned_qpos().size(), 0.);
    velocity[static_cast<std::size_t>(pinned_dof("fork_travel"))] = .9;
    set_fresh(*stepper, position, velocity);
    overflow([&] { (void)stepper->suspension_components(); });
}

// test_end_stop_mutated_config_rejected
void test_end_stop_mutated_config_rejected() {
    std::fprintf(stderr, ">> test_end_stop_mutated_config_rejected\n");
    // replace(EndStopConfig(), reference_deflection_m=-1.) reaches post_init.
    bt::invalid([&] {
        cfg::EndStopConfig rejected{.reference_deflection_m = -1.};
        cfg::detail::post_init(rejected); });
    auto config = cfg::EndStopConfig{};
    config.reference_deflection_m = -1.;
    bt::invalid([&] { cfg::detail::post_init(config); });
    bt::require(std::isfinite(cfg::EndStopConfig{}.stiffness_n_m()),
                "declared end-stop config derives a finite stiffness");
}

// test_tire_two_wheel_loss_overflow_before_publication
void test_tire_two_wheel_loss_overflow_before_publication() {
    std::fprintf(stderr, ">> test_tire_two_wheel_loss_overflow_before_publication\n");
    auto stepper = resolved_stepper(resolved_wire());
    auto position = pinned_qpos();
    position[static_cast<std::size_t>(pinned_adr("root_z"))] += 10.;
    set_fresh(*stepper, position, std::vector<double>(pinned_qpos().size(), 0.));
    stepper->forward();
    const auto names = stepper->tire_state_names();
    std::vector<double> row(names.size(), std::numeric_limits<double>::quiet_NaN());
    row[0] = row[11] = 1.1e152;
    stepper->set_tire_state(names, row);
    overflow([&] { (void)stepper->tire_qfrc(.001); });
}

// test_integer_normalization_and_legacy_hbo_bitwise_controls[tokens-clicks]
void test_integer_normalization_and_legacy_hbo_bitwise_controls() {
    std::fprintf(stderr, ">> test_integer_normalization_and_legacy_hbo_bitwise_controls\n");
    const auto fork_adr = pinned_adr("fork_travel");
    const auto fork_dof = pinned_dof("fork_travel");
    for (const auto& [id, tokens, clicks] : std::array<std::tuple<const char*, int, int>, 2>{
        std::tuple{"-100--100", -100, -100}, std::tuple{"100-100", 100, 100}}) {
        auto golden = node("test_integer_normalization_and_legacy_hbo_bitwise_controls", id);
        auto wire = resolved_wire();
        set(member(member(wire, "suspension"), "air_spring"), "num_tokens", integer(tokens));
        auto& fork = member(member(wire, "suspension"), "fork_damper");
        set(fork, "total_travel_mm", real(150.));
        set(fork, "hbo_start_mm", real(160.));
        set(fork, "legacy_behavior", Value{true});
        set(fork, "hsc_clicks", integer(clicks));
        set(fork, "lsc_clicks", integer(clicks));
        set(fork, "rebound_clicks", integer(clicks));
        auto stepper = resolved_stepper(wire);
        auto position = pinned_qpos();
        position[static_cast<std::size_t>(fork_adr)] = .17;
        auto velocity = std::vector<double>(pinned_qpos().size(), 0.);
        velocity[static_cast<std::size_t>(fork_dof)] = .7;
        set_fresh(*stepper, position, velocity);
        const auto components = stepper->suspension_components();
        for (const auto* wanted : {"fork_spring", "fork_damper"}) {
            const auto found = std::ranges::find_if(components,
                [&](const auto& item) { return item.first == wanted; });
            bt::require(found != components.end(), "suspension components carry the fork rows");
            array(golden, found->second);
        }
        golden.done();
    }
}

// test_air_unreachable_declared_travel_remains_usable
void test_air_unreachable_declared_travel_remains_usable() {
    std::fprintf(stderr, ">> test_air_unreachable_declared_travel_remains_usable\n");
    bike::model::AirSpringSpecs specs;
    specs.total_travel_mm = 1000.;
    bike::model::ForkAirSpring spring(specs, 0);
    bt::require(std::isfinite(spring.compute_axial_force(50.)),
                "unreachable declared travel remains a usable spring");
}

// test_air_calibration_denominator_overflow_before_pressure_commit
void test_air_calibration_denominator_overflow_before_pressure_commit() {
    std::fprintf(stderr, ">> test_air_calibration_denominator_overflow_before_pressure_commit\n");
    bike::model::AirSpringSpecs specs;
    specs.stanchion_inner_diam_mm = 2e103; specs.total_travel_mm = 1e-100;
    specs.pos_chamber_length_mm = 2e-100; specs.neg_chamber_length_mm = 1e-100;
    specs.token_volume_cm3 = 0.; specs.max_tokens = 0; specs.default_tokens = 0;
    specs.gamma = 500.;
    bike::model::ForkAirSpring spring(specs, 0, 82.);
    overflow([&] { (void)spring.calibrate_psi_for_sag(1e-100, 322.2); });
    bt::require(spring.gauge_pressure_psi() == 82.,
                "calibration overflow leaves the stored pressure untouched");
}

// test_air_force_curve_energy_overflow_before_publication
void test_air_force_curve_energy_overflow_before_publication() {
    std::fprintf(stderr, ">> test_air_force_curve_energy_overflow_before_publication\n");
    bike::model::AirSpringSpecs specs;
    specs.stanchion_inner_diam_mm = 2500.;
    bike::model::ForkAirSpring spring(specs, 0, 1e303);
    bt::require(std::isfinite(spring.compute_axial_force(180.)),
                "the force call itself stays finite");
    overflow([&] { (void)spring.compute_force_curve(101); });
}

// test_air_calibration_and_curve_valid_controls
void test_air_calibration_and_curve_valid_controls() {
    std::fprintf(stderr, ">> test_air_calibration_and_curve_valid_controls\n");
    bike::model::ForkAirSpring spring;
    const double pressure = spring.calibrate_psi_for_sag(54., 322.2);
    bt::require(std::isfinite(pressure), "valid calibration returns a finite pressure");
    bt::require(spring.gauge_pressure_psi() == pressure,
                "calibration commits the returned pressure");
    const auto curve = spring.compute_force_curve(11);
    bt::require(!curve.energy_j.empty() &&
                    std::ranges::all_of(curve.energy_j,
                        [](double v) { return std::isfinite(v); }),
                "valid curve publishes finite energies");
    bt::require(curve.max_energy_j == curve.energy_j.back(),
                "reported maximum is the terminal energy");
}

// test_battery_cap_numerator_overflow_before_saturation[native]
void test_battery_cap_numerator_overflow_before_saturation() {
    std::fprintf(stderr, ">> test_battery_cap_numerator_overflow_before_saturation\n");
    auto wire = policy_reference::config(policy_reference::source());
    put(wire, "battery", "copper_w_per_nm2", real(1e-306));
    put(wire, "battery", "speed_w_per_rad_s2", real(0.));
    put(wire, "battery", "idle_w", real(0.));
    const auto policies = policy_reference::policies(wire);
    overflow([&] { (void)policy_reference::energy_limit(policies, 1e308, 0., 1e308); });
}

// test_mutated_zero_dimensional_cassette_is_value_error[native]
void test_mutated_zero_dimensional_cassette_is_value_error() {
    std::fprintf(stderr, ">> test_mutated_zero_dimensional_cassette_is_value_error\n");
    auto wire = policy_reference::config(policy_reference::source());
    put(wire, "shifting", "cassette", integer(24));
    drivetrain_reference::error<std::invalid_argument>(
        [&] { (void)policy_reference::policies(wire); }, "cassette");
}

// test_brake_and_cruise_numpy_numeric_controls
void test_brake_and_cruise_numpy_numeric_controls() {
    std::fprintf(stderr, ">> test_brake_and_cruise_numpy_numeric_controls\n");
    auto golden = node("test_brake_and_cruise_numpy_numeric_controls");
    const auto qpos = golden.f64_input("input.qpos");
    const auto qvel = golden.f64_input("input.qvel");
    const auto warmstart = golden.f64_input("input.qacc_warmstart");
    auto stepper = resolved_stepper(resolved_wire());
    stepper->set_state(qpos, qvel, {}, warmstart, 0.);
    const auto [front, rear] = stepper->brake_torques(-.5, 1.5);
    const std::array torques{front, rear};
    array(golden, torques);
    stepper->apply_brake(-.5, 1.5);
    scalar(golden, stepper->cruise_set_assist_compensation(2.));
    stepper->cruise_set_target_speed(30.);
    scalar(golden, stepper->cruise_state().target_speed_mps);
    golden.done();
}

// test_end_stop_unloading_overflow_before_clamp[branch-native]
void test_end_stop_unloading_overflow_before_clamp() {
    std::fprintf(stderr, ">> test_end_stop_unloading_overflow_before_clamp\n");
    const auto shock_adr = pinned_adr("shock_stroke");
    const auto shock_dof = pinned_dof("shock_stroke");
    const auto resolved = resolved_wire();
    const auto stroke_m = io::as_double(
        field(field(field(resolved, "suspension"), "coil"), "stroke_mm"),
        "coil.stroke_mm") / 1000.;
    for (const auto branch : {"lower", "upper"}) {
        auto wire = resolved_wire();
        auto& stops = member(member(wire, "suspension"), "end_stops");
        set(stops, "stiffness_n_m", real(0.));
        set(stops, "damping_n_s_m", real(1e308));
        auto stepper = resolved_stepper(wire);
        auto position = pinned_qpos();
        position[static_cast<std::size_t>(shock_adr)] =
            std::string_view(branch) == "lower" ? -.01 : stroke_m + .001;
        auto velocity = std::vector<double>(pinned_qpos().size(), 0.);
        velocity[static_cast<std::size_t>(shock_dof)] =
            std::string_view(branch) == "lower" ? 100. : -100.;
        set_live(*stepper, position, velocity);
        overflow([&] { (void)stepper->suspension_components(); });
    }
}

// test_damper_constructor_original_type_class[section-field-bad-native]
void test_damper_constructor_original_type_class() {
    std::fprintf(stderr, ">> test_damper_constructor_original_type_class\n");
    for (const auto& [section, key] : std::array<std::pair<const char*, const char*>, 2>{
        std::pair{"fork_damper", "total_travel_mm"},
        std::pair{"shock_damper", "total_stroke_mm"}})
        for (const Value& bad : {text("180"),
            Value{bike::io::Number{std::string("1") + std::string(1000, '0')}}}) {
            auto wire = resolved_wire();
            set(member(member(wire, "suspension"), section), key, bad);
            bt::invalid([&] { (void)nativecfg::decode(wire); });
            bt::invalid([&] { (void)resolved_stepper(wire); });
        }
}

// test_assist_constructor_revalidates_mutated_profile — the oracle's
// mode_gains=None is the absent-profile / invalid-gains boundary; the
// controller constructor revalidates its own profile block.
void test_assist_constructor_revalidates_mutated_profile() {
    std::fprintf(stderr, ">> test_assist_constructor_revalidates_mutated_profile\n");
    auto wire = policy_reference::config(policy_reference::source());
    put(wire, "assist", "profile", io::parse_json(
        R"({"mode_gains":{"eco":1.0,"tour":2.0,"emtb":[1.0,3.0],"turbo":3.0},"emtb_full_gain_at_nm":20.0})"));
    auto decoded = nativecfg::decode_policy_config(wire);
    auto mutated = decoded;
    mutated.assist.profile->emtb_high = mutated.assist.profile->emtb_low - 1.;
    bt::invalid([&] { drivetrain::validate(mutated.assist); });
    bt::invalid([&] { drivetrain::AssistController rejected(mutated.assist); });
}

// test_assist_constructor_profile_slew_overflow_before_publication — the
// constructor's profile slew is peak_torque_nm / torque_tau_s; the mutated
// profile overflows that derived division before any state is published.
void test_assist_constructor_profile_slew_overflow_before_publication() {
    std::fprintf(stderr, ">> test_assist_constructor_profile_slew_overflow_before_publication\n");
    const auto profile = [] {
        auto value = cfg::kBoschCxGen4;
        value.peak_torque_nm = 1e308; value.torque_tau_s = 1e-308;
        return value;
    }();
    overflow([&] {
        (void)validation::derived(profile.peak_torque_nm / profile.torque_tau_s,
                                  "AssistController.profile_slew"); });
}

// test_zero_disable_policy_fields_remain_bitwise_valid
void test_zero_disable_policy_fields_remain_bitwise_valid() {
    std::fprintf(stderr, ">> test_zero_disable_policy_fields_remain_bitwise_valid\n");
    auto golden = node("test_zero_disable_policy_fields_remain_bitwise_valid");
    auto source = drivetrain_reference::source();
    source.pedaling.enabled = true;
    source.pedaling.coast_cadence_tau_s = 0.; source.pedaling.effort_slew_nm_s = 0.;
    source.pedaling.mash_torque_nm = 0.; source.pedaling.mash_cadence_rpm = 0.;
    source.shifting.enabled = true;
    source.shifting.shift_cooldown_s = 0.; source.shifting.shift_cut_duration_s = 0.;
    source.shifting.cadence_smoothing_tau_s = 0.; source.shifting.upshift_slip_limit_mps = 0.;
    source.battery.energy_j = 0.; source.battery.copper_w_per_nm2 = 0.;
    source.battery.speed_w_per_rad_s2 = 0.; source.battery.idle_w = 0.;
    source.assist.gain = 0.; source.assist.max_torque = 0.; source.assist.max_power = 0.;
    source.assist.engage_torque_nm = 0.; source.assist.gate_min_crank_rad_s = 0.;
    source.bearing_c_nms_rad = 0.; source.freehub_c_nms_rad = 0.; source.chain_c_ns_m = 0.;
    auto stepper = drivetrain_reference::pair(source);
    drivetrain_reference::components(golden,
        drivetrain_reference::components(*stepper, .001, 0.));
    golden.done();
}

// test_tire_channel_parity_across_backends — the recorded tree is the
// oracle's tire_channels wire; the native runtime's own wire is walked in
// the same sorted-keys/close-scalar order, so any schema or value drift
// consumes the wrong golden entry.
//
// The oracle's own t=0 bootstrap travels through the golden as input
// entries: the live reset's settled state is platform-dependent (~1e-6
// between darwin-arm64 and linux-aarch64), so neither committed bootstrap
// fixture is the oracle's seed. The port materializes the recorded
// canonical config.json/state.json documents beside the committed
// model.mjb — model-field drift measures ~1e-13 in the trajectory — and
// replays the same platform-faithful start.
std::string input_bytes(std::span<const double> payload) {
    const auto view = std::as_bytes(payload);
    return {reinterpret_cast<const char*>(view.data()), view.size()};
}
void test_tire_channel_parity_across_backends() {
    std::fprintf(stderr, ">> test_tire_channel_parity_across_backends\n");
    auto golden = node("test_tire_channel_parity_across_backends");
    for (const auto& [side, fixture_name] : std::array<std::pair<const char*, const char*>, 2>{
        std::pair{"compliant", "bootstrap/welded"},
        std::pair{"distributed", "bootstrap/distributed_rocky"}}) {
        const auto fixed_dir = bt::fixture(fixture_name);
        bt::TemporaryDirectory stage("nc-tires");
        std::filesystem::copy_file(fixed_dir / "model.mjb",
                                   stage.path() / "model.mjb");
        std::string manifest_text =
            input_bytes(golden.f64_input(std::string(side) + ".files"));
        while (!manifest_text.empty() && manifest_text.back() == ' ')
            manifest_text.pop_back();
        const Value manifest = io::parse_json(manifest_text);
        for (const Value& item : io::as_array(manifest, "manifest")) {
            const std::string name =
                io::as_string(field(item, "name"), "manifest.name");
            const auto nbytes = static_cast<std::size_t>(io::number_i64(
                io::as_number(field(item, "bytes"), "manifest.bytes"),
                "manifest.bytes"));
            const auto payload =
                golden.f64_input(std::string(side) + ".file::" + name);
            bt::require(payload.size() * 8 >= nbytes,
                        "recorded file payload truncated");
            const std::filesystem::path path = stage.path() / name;
            std::filesystem::create_directories(path.parent_path());
            std::ofstream out(path, std::ios::binary);
            out.write(reinterpret_cast<const char*>(payload.data()),
                      static_cast<std::streamsize>(nbytes));
            bt::require(out.good(), "recorded file materialization failed");
        }
        const auto rt = std::make_unique<runtime::NativeRideRuntime>(
            runtime::read_bootstrap(stage.path()));
        for (std::int64_t target = 1; target <= 4; ++target)
            (void)rt->advance(target, runtime::RideControl{}, 0., 0., std::nullopt);
        const auto snapshot = rt->snapshot();
        bt::require(snapshot.latest_sample != nullptr,
                    "advanced runtime publishes a latest sample");
        wire_close(golden, wire_member(snapshot.latest_sample->channels, "tires"), side);
    }
    golden.done();
}

// test_python_free_typed_numeric_validators — the source compiles and runs
// this exact validator battery; the port keeps it inline.
void test_python_free_typed_numeric_validators() {
    std::fprintf(stderr, ">> test_python_free_typed_numeric_validators\n");
    const auto invalid_config = [](const auto& config) {
        try { validate(config); }  // unqualified: ADL picks nativecfg/drivetrain
        catch (const std::invalid_argument&) { return; }
        throw std::runtime_error("typed config accepted invalid domain");
    };
    invalid_config(nativecfg::AirSpringSpec{});
    invalid_config(nativecfg::AirSpringConfig{});
    invalid_config(nativecfg::DamperCore{});
    invalid_config(nativecfg::Charger3Config{});
    invalid_config(nativecfg::SuperDeluxeConfig{});
    invalid_config(nativecfg::CoilConfig{});
    invalid_config(nativecfg::EndStopConfig{.stiffness_n_m = -1., .damping_n_s_m = 0.});
    invalid_config(nativecfg::BrakeConfig{});
    invalid_config(nativecfg::CruiseConfig{});
    invalid_config(nativecfg::RiderPathConfig{.name = "rider_leg_front",
        .joint = "rider_leg_front", .stiffness_n_m = -1., .damping_ns_m = 0.,
        .preload_deflection_m = 0., .offset_m = 0., .unilateral = false});
    invalid_config(nativecfg::ResistanceConfig{});
    invalid_config(nativecfg::TireMaterial{});
    invalid_config(nativecfg::TireParams{});
    invalid_config(nativecfg::SurfaceSpec{});
    invalid_config(nativecfg::TireConfig{});
    invalid_config(drivetrain::GearingConfig{});
    invalid_config(drivetrain::PedalingConfig{});
    invalid_config(drivetrain::ShiftingConfig{});
    invalid_config(drivetrain::MotorProfile{});
    invalid_config(drivetrain::AssistConfig{});
    invalid_config(drivetrain::DrivePolicyConfig{});
    invalid_config(drivetrain::DriveConfig{});
    invalid_config(drivetrain::BatteryConfig{.energy_j = -1.});
    nativecfg::validate(nativecfg::EndStopConfig{});
    nativecfg::validate(nativecfg::BrakeConfig{.torque_ceiling_nm = 0., .taper_radps = 1.});
    drivetrain::validate(drivetrain::BatteryConfig{});
}

} // namespace

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
int main() { return bt::run([] {
    test_air_specs_mutated_geometry_rejected();
    test_air_pressure_properties_revalidate();
    test_air_specs_mutated_token_limit_rejected();
    test_invalid_tire_material_rejected();
    test_mutated_surface_map_rejected();
    test_mutated_drive_config_rejected_before_model_writes();
    test_drivetrain_wheel_topology_shared();
    test_drivetrain_actuator_target_shared();
    test_shock_component_overflow_shared();
    test_chain_computation_overflow_shared();
    test_policy_intermediate_overflow_shared();
    test_cruise_overflow_preserves_finite_telemetry();
    test_native_damper_errors_name_full_field_path();
    test_nested_numeric_errors_name_full_field_path();
    test_end_stop_strict_types_and_computation_overflow();
    test_remaining_s4_scalar_field_matrix();
    test_remaining_s4_positive_field_zero();
    test_s4_matrix_valid_configuration();
    test_unicode_blank_tire_provenance_shared();
    test_resistance_requires_physical_bodies();
    test_cassette_ordered_integer_contract();
    test_mutated_tire_parameters_rejected();
    test_damper_mutated_denominator_rejected_by_setter();
    test_s4_relationship_matrix();
    test_rider_path_scalar_matrix_shared();
    test_invalid_motor_profile_relationships_rejected();
    test_bearing_force_overflow_before_return_and_telemetry();
    test_chain_dissipation_overflow_before_telemetry();
    test_suspension_total_overflow_before_telemetry();
    test_end_stop_mutated_config_rejected();
    test_tire_two_wheel_loss_overflow_before_publication();
    test_integer_normalization_and_legacy_hbo_bitwise_controls();
    test_air_unreachable_declared_travel_remains_usable();
    test_air_calibration_denominator_overflow_before_pressure_commit();
    test_air_force_curve_energy_overflow_before_publication();
    test_air_calibration_and_curve_valid_controls();
    test_battery_cap_numerator_overflow_before_saturation();
    test_mutated_zero_dimensional_cassette_is_value_error();
    test_brake_and_cruise_numpy_numeric_controls();
    test_end_stop_unloading_overflow_before_clamp();
    test_damper_constructor_original_type_class();
    test_assist_constructor_revalidates_mutated_profile();
    test_assist_constructor_profile_slew_overflow_before_publication();
    test_zero_disable_policy_fields_remain_bitwise_valid();
    test_tire_channel_parity_across_backends();
    test_python_free_typed_numeric_validators();
}); }
NATIVE_DIAG_POP
