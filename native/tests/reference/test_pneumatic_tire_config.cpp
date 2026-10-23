// Ports all thirteen source test functions in test_pneumatic_tire_config.py.
// The two Python dynamic-type constructions are impossible for the typed
// member; their native contracts are compile-time assignment rejection plus
// the same malformed-table rejection at the real configuration ingress.
#include "support/migration_test.hpp"
#include "diag.hpp"
#include "configuration/load.hpp"
#include "io/json.hpp"
#include "research/environment.hpp"

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
    return cfg::detail::construct_config(io::parse_json(document));
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
    // test_pneumatic_tire_config.py::test_pneumatic_section_defaults_to_fast_stock_tyres
    cfg::PneumaticTireConfig config;
    cfg::detail::post_init(config);
    bt::require(config.tier == "fast" && !config.surface, "default fast, no named surface");
    stock(config.front, cfg::kFrontTyre); stock(config.rear, cfg::kRearTyre);
    bt::require(cfg::find_tire_tier(config.tier) != nullptr, "default tier is registered");
}
[[gnu::noinline]] void test_pneumatic_is_opt_in_and_the_section_always_resolves() {
    // test_pneumatic_tire_config.py::test_pneumatic_is_opt_in_and_the_section_always_resolves
    cfg::TireBackendConfig config; cfg::detail::post_init(config);
    bt::require(config.backend == "native_reference", "pneumatic is opt in");
    bt::require((std::is_same_v<decltype(config.pneumatic), cfg::PneumaticTireConfig>), "section always has resolved type");
    stock(config.pneumatic.front, cfg::kFrontTyre);
}
[[gnu::noinline]] void test_pneumatic_backend_resolves_from_toml() {
    // test_pneumatic_tire_config.py::test_pneumatic_backend_resolves_from_toml
    const auto config = resolve(R"({"physics_mode":"physical","tires":{"backend":"pneumatic","pneumatic":{"surface":"asphalt"}}})");
    bt::require(config.tires.backend == "pneumatic" && config.tires.pneumatic.surface == "asphalt" &&
        config.tires.pneumatic.tier == "fast", "pneumatic section resolves");
    stock(config.tires.pneumatic.front, cfg::kFrontTyre); stock(config.tires.pneumatic.rear, cfg::kRearTyre);
}
[[gnu::noinline]] void test_pneumatic_backend_resolves_under_track_surface_mode() {
    // test_pneumatic_tire_config.py::test_pneumatic_backend_resolves_under_track_surface_mode
    const auto config = resolve(R"({"physics_mode":"physical","timestep_s":0.00025,"tires":{"backend":"pneumatic","surface_mode":"track","pneumatic":{"tier":"detailed"}}})");
    bt::require(config.tires.surface_mode == "track" && !config.tires.pneumatic.surface &&
        config.tires.pneumatic.tier == "detailed", "detailed track material resolution");
}
[[gnu::noinline]] void test_detailed_pneumatic_requires_the_tier_timestep() {
    // test_pneumatic_tire_config.py::test_detailed_pneumatic_requires_the_tier_timestep
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
    // test_pneumatic_tire_config.py::test_partial_tyre_tables_extend_the_default_specs
    const auto config = resolve(R"({"physics_mode":"physical","tires":{"backend":"pneumatic","pneumatic":{"surface":"asphalt","front":{"pressure_bar":1.9},"rear":{"pressure_bar":2.1,"name":"custom rear"}}}})");
    const auto &front = config.tires.pneumatic.front, &rear = config.tires.pneumatic.rear;
    bt::require(front.pressure_bar == 1.9 && front.name == cfg::kFrontTyre.name &&
        front.outer_radius_mm == cfg::kFrontTyre.outer_radius_mm && front.area_factor == cfg::kFrontTyre.area_factor,
        "front partial table extends stock");
    bt::require(rear.pressure_bar == 2.1 && rear.name == "custom rear" && rear.outer_radius_mm == cfg::kRearTyre.outer_radius_mm,
        "rear partial table extends stock");
}
[[gnu::noinline]] void test_pneumatic_rejects_configured_mode_without_a_surface() {
    // test_pneumatic_tire_config.py::test_pneumatic_rejects_configured_mode_without_a_surface
    rejects([] { (void)resolve(R"({"tires":{"backend":"pneumatic"}})"); }, "pneumatic");
    rejects([] { (void)resolve(R"({"tires":{"backend":"pneumatic","surface_mode":"configured","pneumatic":{"tier":"fast"}}})"); }, "surface");
}
[[gnu::noinline]] void test_pneumatic_rejects_an_unknown_tier() {
    // test_pneumatic_tire_config.py::test_pneumatic_rejects_an_unknown_tier
    rejects([] { (void)resolve(R"({"tires":{"backend":"pneumatic","pneumatic":{"surface":"asphalt","tier":"ultra"}}})"); }, "tier");
    rejects([] { cfg::PneumaticTireConfig config; config.tier = "ultra"; cfg::detail::post_init(config); }, "tier");
}
[[gnu::noinline]] void test_pneumatic_rejects_an_unknown_surface_name() {
    // test_pneumatic_tire_config.py::test_pneumatic_rejects_an_unknown_surface_name
    rejects([] { (void)resolve(R"({"tires":{"backend":"pneumatic","pneumatic":{"surface":"ice"}}})"); }, "surface");
    rejects([] { cfg::PneumaticTireConfig config; config.surface = "ice"; cfg::detail::post_init(config); }, "surface");
}
[[gnu::noinline]] void test_pneumatic_rejects_an_unresolved_section() {
    // test_pneumatic_tire_config.py::test_pneumatic_rejects_an_unresolved_section
    static_assert(!std::is_assignable_v<cfg::PneumaticTireConfig &, std::nullptr_t>);
    static_assert(!std::is_assignable_v<cfg::PneumaticTireConfig &, io::Value>);
    bt::require((!std::is_assignable_v<cfg::PneumaticTireConfig &, io::Value>), "Python None/dict cannot inhabit typed pneumatic member");
    rejects([] { (void)resolve(R"({"tires":{"backend":"pneumatic","pneumatic":"asphalt"}})"); }, "requires a TOML table");
    rejects([] { (void)resolve(R"({"tires":{"backend":"pneumatic","pneumatic":null}})"); }, "requires a TOML table");
}
[[gnu::noinline]] void test_pneumatic_rejects_unknown_tyre_keys() {
    // test_pneumatic_tire_config.py::test_pneumatic_rejects_unknown_tyre_keys
    rejects([] { (void)resolve(R"({"tires":{"backend":"pneumatic","pneumatic":{"surface":"asphalt","front":{"pressure_baar":1.9}}}})"); }, "unknown TyreSpecs parameter");
}
[[gnu::noinline]] void test_research_environment_rejects_a_named_pneumatic_surface() {
    // test_pneumatic_tire_config.py::test_research_environment_rejects_a_named_pneumatic_surface
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
    // test_pneumatic_tire_config.py::test_existing_backend_gates_are_unchanged
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
