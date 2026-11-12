// Ports all thirteen source test functions in test_pneumatic_tire_config.py.
// The two Python dynamic-type constructions are impossible for the typed
// member; their native contracts are compile-time assignment rejection plus
// the same malformed-table rejection at the real configuration ingress.
#include "support/fixture_test.hpp"
#include "diag.hpp"
#include "configuration/load.hpp"
#include "io/json.hpp"
#include "research/environment.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>

namespace {
namespace cfg = bike::configuration;
namespace io = bike::io;
// Decoded config records are multi-kilobyte; test functions are noinline so
// main's lambda stays under -Wframe-larger-than, and the pragma covers the
// few individual frames that still exceed 8192 bytes.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
cfg::SimulationPhysicsConfig resolve(std::string_view document) {
    auto value = io::parse_json(document);
    if (auto *obj = std::get_if<io::Value::Object>(&value.data)) {
        if (!io::find_member(*obj, "seated_climb")) {
            obj->emplace_back("seated_climb", io::Value{io::Value::Object{
                {"period_s", io::Value{io::Number{"0.00125"}}},
                {"reaction_delay_s", io::Value{io::Number{"0.15"}}},
                {"lean_rate_rad_s", io::Value{io::Number{"0.5"}}},
                {"orientation_tau_s", io::Value{io::Number{"0.5"}}},
                {"surge_budget_s", io::Value{io::Number{"15"}}},
                {"trim_dead_time_s", io::Value{io::Number{"0.2"}}},
                {"min_elbow_flexion_deg", io::Value{io::Number{"15.0"}}},
                {"launch_effort_ramp_s", io::Value{io::Number{"1.5"}}},
                {"launch_min_grade", io::Value{io::Number{"0.03"}}},
                {"launch_standstill_crank_rate_rad_s",
                 io::Value{io::Number{"1.0"}}}}});
        }
        if (!io::find_member(*obj, "rider_launch")) {
            obj->emplace_back("rider_launch", io::Value{io::Value::Object{
                {"start_cog_grade_breaks", io::Value{io::Value::Array{}}},
                {"start_cog_teeth", io::Value{io::Value::Array{io::Value{io::Number{"24"}}}}},
                {"start_pedal_angle_rad", io::Value{io::Number{"0.0"}}},
                {"start_lean_grades", io::Value{io::Value::Array{io::Value{io::Number{"0.0"}}}}},
                {"start_lean_rad", io::Value{io::Value::Array{io::Value{io::Number{"0.0"}}}}},
                {"plan_enabled", io::Value{false}},
                {"standstill_speed_mps", io::Value{io::Number{"0.1"}}},
                {"standstill_crank_rpm", io::Value{io::Number{"5.0"}}},
                {"effort_rate_nm_s", io::Value{io::Number{"40.0"}}},
                {"effort_cap_asphalt_nm", io::Value{io::Number{"60.0"}}},
                {"effort_cap_hardpack_nm", io::Value{io::Number{"60.0"}}},
                {"effort_cap_loose_nm", io::Value{io::Number{"60.0"}}},
                {"effort_cap_wet_nm", io::Value{io::Number{"60.0"}}},
                {"effort_cap_scree_nm", io::Value{io::Number{"60.0"}}},
                {"release_torque_grades", io::Value{io::Value::Array{
                     io::Value{io::Number{"0.0"}}, io::Value{io::Number{"0.2"}}}}},
                {"release_torque_nm", io::Value{io::Value::Array{
                     io::Value{io::Number{"0.0"}}, io::Value{io::Number{"30.0"}}}}},
                {"brake_release_s", io::Value{io::Number{"0.15"}}},
                {"max_hold_s", io::Value{io::Number{"1.5"}}},
                {"upshift_crank_rpm", io::Value{io::Number{"82.0"}}},
                {"end_speed_mps", io::Value{io::Number{"3.0"}}},
                {"end_crank_rpm", io::Value{io::Number{"60.0"}}},
                {"end_hold_s", io::Value{io::Number{"0.3"}}},
                {"max_launch_s", io::Value{io::Number{"6.0"}}},
                {"handover_s", io::Value{io::Number{"0.9"}}}}});
        }
        if (!io::find_member(*obj, "front_lift_reaction")) {
            obj->emplace_back("front_lift_reaction", io::Value{io::Value::Object{
                {"hand_unload_n", io::Value{io::Number{"20.0"}}},
                {"unload_dwell_s", io::Value{io::Number{"0.05"}}},
                {"pitch_rate_up_rad_s", io::Value{io::Number{"0.5"}}},
                {"pitch_rad", io::Value{io::Number{"0.17"}}},
                {"decision_s", io::Value{io::Number{"0.1"}}},
                {"ease_enabled", io::Value{false}},
                {"lean_enabled", io::Value{false}},
                {"rear_brake_tap_enabled", io::Value{false}},
                {"ease_fraction", io::Value{io::Number{"0.7"}}},
                {"ease_rate_per_s", io::Value{io::Number{"5.0"}}},
                {"lean_add_rad", io::Value{io::Number{"0.1"}}},
                {"lean_rate_rad_s", io::Value{io::Number{"0.5"}}},
                {"clear_s", io::Value{io::Number{"0.3"}}}}});
        }
        // These documents exercise the tire section only — fill the
        // required timing keys the oracle's defaults used to supply, with
        // the 10-step dropout expressed at the document's own tick.
        const auto *timestep = io::find_member(*obj, "timestep_s");
        const double dt = timestep != nullptr
                              ? io::as_double(*timestep, "timestep_s")
                              : 0.0005;
        if (timestep == nullptr)
            obj->emplace_back("timestep_s", io::Value{io::Number{"0.0005"}});
        if (!io::find_member(*obj, "control_period_s"))
            obj->emplace_back("control_period_s",
                              io::Value{io::Number{"0.005"}});
        if (!io::find_member(*obj, "monitors")) {
            const auto hold = cfg::detail::repr_double(10. * dt);
            obj->emplace_back("monitors", io::Value{io::Value::Object{
                {"grounded_hold_s", io::Value{io::Number{hold}}},
                {"contact_dropout_s", io::Value{io::Number{hold}}}}});
        }
        // The required crank-reposition knobs ride inside [drive.pedaling] —
        // these tire documents never carry a drive section, so the former
        // code defaults are injected explicitly (reposition stays off). The
        // 1 s window/cooldown and 3 s timeout must be whole multiples of the
        // document's own tick, so each is rounded to the nearest one (at
        // the 0.3 ms tick of one document, 1 s becomes 3333 steps).
        const auto window =
            cfg::detail::repr_double(std::rint(1. / dt) * dt);
        const auto timeout =
            cfg::detail::repr_double(std::rint(3. / dt) * dt);
        // motor_clutch is a required [drive] member: the clutch is the bike
        // wherever the rider pedals, so the injected section engages it for
        // effort modes and stays rigid elsewhere (where it must be false).
        const auto *mode = io::find_member(*obj, "drive_mode");
        const bool effort =
            mode != nullptr &&
            (io::as_string(*mode, "drive_mode") == "crank_effort" ||
             io::as_string(*mode, "drive_mode") == "articulated_effort");
        // The drive holds and balance windows follow the same rule: every
        // declared duration must land on a whole number of the document's
        // ticks, so each default is rounded the same way. The smoothing
        // defaults (.05 s assist lag, .35 s cadence EMA, .2 s rate
        // relaxation, disabled coast filter) clear two ticks at every
        // timestep these documents use.
        const auto tick = [&](double nominal) {
            return cfg::detail::repr_double(std::rint(nominal / dt) * dt);
        };
        if (!io::find_member(*obj, "drive"))
            obj->emplace_back("drive", io::Value{io::Value::Object{
                {"motor_clutch", io::Value{effort}},
                {"assist", io::Value{io::Value::Object{
                    {"release_hold_s", io::Value{io::Number{tick(.3)}}}}}},
                {"pedaling", io::Value{io::Value::Object{
                    {"reposition_on_stall", io::Value{false}},
                    {"reposition_min_effort_nm", io::Value{io::Number{"20.0"}}},
                    {"reposition_stall_window_s", io::Value{io::Number{window}}},
                    {"reposition_stall_progress_rad", io::Value{io::Number{"0.5"}}},
                    {"reposition_cooldown_s", io::Value{io::Number{window}}},
                    {"reposition_back_rate_rad_s", io::Value{io::Number{"3.0"}}},
                    {"reposition_timeout_s", io::Value{io::Number{timeout}}},
                    {"reposition_phase_tolerance_rad", io::Value{io::Number{"0.05"}}},
                    {"reposition_noop_rad", io::Value{io::Number{"0.12"}}},
                    {"stop_time_s", io::Value{io::Number{tick(.35)}}}}}},
                {"shifting", io::Value{io::Value::Object{
                    {"shift_cooldown_s", io::Value{io::Number{tick(.4)}}},
                    {"shift_cut_duration_s", io::Value{io::Number{tick(.2)}}}}}}}});
        if (!io::find_member(*obj, "articulated"))
            obj->emplace_back("articulated", io::Value{io::Value::Object{
                {"balance_dwell_s", io::Value{io::Number{tick(.5)}}},
                {"balance_grace_s", io::Value{io::Number{tick(3.)}}}}});
        // The required [research] timings likewise express their former
        // defaults at the document's own tick; latency and maximum age are
        // timestamp cutoffs, so they keep their literal values.
        const auto delay =
            cfg::detail::repr_double(std::rint(0.005 / dt) * dt);
        const auto period =
            cfg::detail::repr_double(std::rint(0.01 / dt) * dt);
        if (!io::find_member(*obj, "research"))
            obj->emplace_back("research", io::Value{io::Value::Object{
                {"control_period_s", io::Value{io::Number{period}}},
                {"actuator_delay_s", io::Value{io::Number{delay}}},
                {"sensor_latency_s", io::Value{io::Number{"0.01"}}},
                {"sensor_maximum_age_s", io::Value{io::Number{"0.1"}}},
                {"sample_period_s", io::Value{io::Number{delay}}},
                {"record_decimation", io::Value{io::Number{"80"}}}}});
    }
    return cfg::detail::construct_config(value);
}
template<class F> void rejects(F &&operation, std::string_view message) {
    bool rejected = false;
    try { std::forward<F>(operation)(); }
    catch (const std::invalid_argument &error) {
        rejected = true;
        bt::require(std::string_view(error.what()).find(message) != std::string_view::npos,
            "configuration rejection identifies " + std::string(message) + ": " + error.what());
    }
    bt::require(rejected, "invalid configuration rejected");
}
void stock(const cfg::TyreSpecs &value, const cfg::TyreSpecs &expected) {
    bt::require(io::canonical_json(cfg::detail::as_object(cfg::detail::kTyreSpecsSchema, &value)) ==
        io::canonical_json(cfg::detail::as_object(cfg::detail::kTyreSpecsSchema, &expected)), "all stock tyre fields agree");
}
[[gnu::noinline]] void test_pneumatic_section_defaults_to_fast_stock_tyres() {
    // pneumatic_tire_config/pneumatic_section_defaults_to_fast_stock_tyres
    cfg::PneumaticTireConfig config;
    cfg::detail::post_init(config);
    bt::require(config.tier == "fast" && !config.surface, "default fast, no named surface");
    stock(config.front, cfg::kFrontTyre); stock(config.rear, cfg::kRearTyre);
    bt::require(cfg::find_tire_tier(config.tier) != nullptr, "default tier is registered");
}
[[gnu::noinline]] void test_pneumatic_is_opt_in_and_the_section_always_resolves() {
    // pneumatic_tire_config/pneumatic_is_opt_in_and_the_section_always_resolves
    cfg::TireBackendConfig config; cfg::detail::post_init(config);
    bt::require(config.backend == "native_reference", "pneumatic is opt in");
    bt::require((std::is_same_v<decltype(config.pneumatic), cfg::PneumaticTireConfig>), "section always has resolved type");
    stock(config.pneumatic.front, cfg::kFrontTyre);
}
[[gnu::noinline]] void test_pneumatic_backend_resolves_from_toml() {
    // pneumatic_tire_config/pneumatic_backend_resolves_from_toml
    const auto config = resolve(R"({"physics_mode":"physical","tires":{"backend":"pneumatic","pneumatic":{"surface":"asphalt"}}})");
    bt::require(config.tires.backend == "pneumatic" && config.tires.pneumatic.surface == "asphalt" &&
        config.tires.pneumatic.tier == "fast", "pneumatic section resolves");
    stock(config.tires.pneumatic.front, cfg::kFrontTyre); stock(config.tires.pneumatic.rear, cfg::kRearTyre);
}
[[gnu::noinline]] void test_pneumatic_backend_resolves_under_track_surface_mode() {
    // pneumatic_tire_config/pneumatic_backend_resolves_under_track_surface_mode
    const auto config = resolve(R"({"physics_mode":"physical","timestep_s":0.00025,"tires":{"backend":"pneumatic","surface_mode":"track","pneumatic":{"tier":"detailed"}}})");
    bt::require(config.tires.surface_mode == "track" && !config.tires.pneumatic.surface &&
        config.tires.pneumatic.tier == "detailed", "detailed track material resolution");
}
[[gnu::noinline]] void test_detailed_pneumatic_requires_the_tier_timestep() {
    // pneumatic_tire_config/detailed_pneumatic_requires_the_tier_timestep
    rejects([] { (void)resolve(R"({"physics_mode":"physical","tires":{"backend":"pneumatic","surface_mode":"track","pneumatic":{"tier":"detailed"}}})"); },
        "detailed pneumatic tyre requires timestep_s");
    // Multi-kilobyte resolved configs live on the heap; this frame stays
    // under the 8192-byte budget.
    const auto config = std::make_unique<cfg::SimulationPhysicsConfig>(
        resolve(R"({"physics_mode":"physical","timestep_s":0.00025,"tires":{"backend":"pneumatic","surface_mode":"track","pneumatic":{"tier":"detailed"}}})"));
    bt::require(config->timestep_s == cfg::find_tire_tier("detailed")->timestep_s, "exact declared detailed timestep");
    const auto fast = std::make_unique<cfg::SimulationPhysicsConfig>(
        resolve(R"({"physics_mode":"physical","timestep_s":0.00025,"tires":{"backend":"pneumatic","surface_mode":"track","pneumatic":{"tier":"fast"}}})"));
    bt::require(fast->tires.pneumatic.tier == "fast", "fast tier accepts a different timestep");
}
[[gnu::noinline]] void test_partial_tyre_tables_extend_the_default_specs() {
    // pneumatic_tire_config/partial_tyre_tables_extend_the_default_specs
    const auto config = resolve(R"({"physics_mode":"physical","tires":{"backend":"pneumatic","pneumatic":{"surface":"asphalt","front":{"pressure_bar":1.9},"rear":{"pressure_bar":2.1,"name":"custom rear"}}}})");
    const auto &front = config.tires.pneumatic.front, &rear = config.tires.pneumatic.rear;
    bt::require(front.pressure_bar == 1.9 && front.name == cfg::kFrontTyre.name &&
        front.outer_radius_mm == cfg::kFrontTyre.outer_radius_mm && front.area_factor == cfg::kFrontTyre.area_factor,
        "front partial table extends stock");
    bt::require(rear.pressure_bar == 2.1 && rear.name == "custom rear" && rear.outer_radius_mm == cfg::kRearTyre.outer_radius_mm,
        "rear partial table extends stock");
}
[[gnu::noinline]] void test_pneumatic_rejects_configured_mode_without_a_surface() {
    // pneumatic_tire_config/pneumatic_rejects_configured_mode_without_a_surface
    rejects([] { (void)resolve(R"({"tires":{"backend":"pneumatic"}})"); }, "pneumatic");
    rejects([] { (void)resolve(R"({"tires":{"backend":"pneumatic","surface_mode":"configured","pneumatic":{"tier":"fast"}}})"); }, "surface");
}
[[gnu::noinline]] void test_pneumatic_rejects_an_unknown_tier() {
    // pneumatic_tire_config/pneumatic_rejects_an_unknown_tier
    rejects([] { (void)resolve(R"({"tires":{"backend":"pneumatic","pneumatic":{"surface":"asphalt","tier":"ultra"}}})"); }, "tier");
    rejects([] { cfg::PneumaticTireConfig config; config.tier = "ultra"; cfg::detail::post_init(config); }, "tier");
}
[[gnu::noinline]] void test_pneumatic_rejects_an_unknown_surface_name() {
    // pneumatic_tire_config/pneumatic_rejects_an_unknown_surface_name
    rejects([] { (void)resolve(R"({"tires":{"backend":"pneumatic","pneumatic":{"surface":"ice"}}})"); }, "surface");
    rejects([] { cfg::PneumaticTireConfig config; config.surface = "ice"; cfg::detail::post_init(config); }, "surface");
}
[[gnu::noinline]] void test_pneumatic_rejects_an_unresolved_section() {
    // pneumatic_tire_config/pneumatic_rejects_an_unresolved_section
    static_assert(!std::is_assignable_v<cfg::PneumaticTireConfig &, std::nullptr_t>);
    static_assert(!std::is_assignable_v<cfg::PneumaticTireConfig &, io::Value>);
    bt::require((!std::is_assignable_v<cfg::PneumaticTireConfig &, io::Value>), "Python None/dict cannot inhabit typed pneumatic member");
    rejects([] { (void)resolve(R"({"tires":{"backend":"pneumatic","pneumatic":"asphalt"}})"); }, "requires a TOML table");
    rejects([] { (void)resolve(R"({"tires":{"backend":"pneumatic","pneumatic":null}})"); }, "requires a TOML table");
}
[[gnu::noinline]] void test_pneumatic_rejects_unknown_tyre_keys() {
    // pneumatic_tire_config/pneumatic_rejects_unknown_tyre_keys
    rejects([] { (void)resolve(R"({"tires":{"backend":"pneumatic","pneumatic":{"surface":"asphalt","front":{"pressure_baar":1.9}}}})"); }, "unknown TyreSpecs parameter");
}
[[gnu::noinline]] void test_research_environment_rejects_a_named_pneumatic_surface() {
    // pneumatic_tire_config/research_environment_rejects_a_named_pneumatic_surface
    const auto request = [](bool named) {
        auto input = std::make_unique<bike::research::ResearchRequest>();
        // control_period_s keeps the config-level integer-step gate satisfied
        // (2 x timestep) so a named surface reaches environment.cpp's
        // track-material gate — the same ordering as ResearchEnvironment,
        // where SimulationPhysicsConfig.__post_init__ precedes the tire
        // check and the experiment control period divides last. The
        // experiment's default 0.01 s period is deliberately not a multiple
        // of 0.0003 s, giving the unnamed control its downstream rejection.
        input->setup.model.physics = resolve(R"({"physics_mode":"physical","drive_mode":"articulated_effort","timestep_s":0.0003,"control_period_s":0.0006,"tires":{"backend":"pneumatic","surface_mode":"track"}})");
        if (named) input->setup.model.physics->tires.pneumatic.surface = "asphalt";
        input->setup.track.name = "gate-control"; input->setup.track.length_m = 10.;
        input->setup.model.rider = cfg::RiderSpecs{}; input->setup.model.rider->variant = "articulated_planar";
        const auto &timings = input->setup.model.physics->research;
        input->experiment.control_period_s = .01;
        input->experiment.actuator_delay_s = timings.actuator_delay_s;
        input->experiment.record_decimation = timings.record_decimation;
        input->sensors.delivery.latency_s = timings.sensor_latency_s;
        input->sensors.delivery.maximum_age_s = timings.sensor_maximum_age_s;
        input->sensors.sample_period_s = timings.sample_period_s;
        return input;
    };
    rejects([&] { (void)bike::research::prepare_environment(request(true)); }, "track-material");
    // C++ has no SimpleNamespace with an absent model. A deliberately
    // nonintegral control period is the analogous downstream control:
    // it proves unnamed pneumatic passed the real research admission gate,
    // and rejects before any engine setup could obscure that evidence.
    rejects([&] { (void)bike::research::prepare_environment(request(false)); }, "control period must be an integer multiple");
}
[[gnu::noinline]] void test_existing_backend_gates_are_unchanged() {
    // pneumatic_tire_config/existing_backend_gates_are_unchanged
    rejects([] { cfg::TireBackendConfig config; config.backend = "pneumtic"; cfg::detail::post_init(config); }, "unknown physical tire backend");
    rejects([] { cfg::TireBackendConfig config; config.surface_mode = "track"; cfg::detail::post_init(config); },
        "track material resolution requires a compliant_2d, distributed_2d_reference or pneumatic tire backend");
    rejects([] { (void)resolve(R"({"tires":{"pneumtic":{}}})"); }, "unknown TireBackendConfig parameter");
}
NATIVE_DIAG_POP
} // namespace
int main() { return bt::run([] {
    test_pneumatic_section_defaults_to_fast_stock_tyres();
    test_pneumatic_is_opt_in_and_the_section_always_resolves();
    test_pneumatic_backend_resolves_from_toml();
    test_pneumatic_backend_resolves_under_track_surface_mode();
    test_detailed_pneumatic_requires_the_tier_timestep();
    test_partial_tyre_tables_extend_the_default_specs();
    test_pneumatic_rejects_configured_mode_without_a_surface();
    test_pneumatic_rejects_an_unknown_tier();
    test_pneumatic_rejects_an_unknown_surface_name();
    test_pneumatic_rejects_an_unresolved_section();
    test_pneumatic_rejects_unknown_tyre_keys();
    test_research_environment_rejects_a_named_pneumatic_surface();
    test_existing_backend_gates_are_unchanged();
}); }
