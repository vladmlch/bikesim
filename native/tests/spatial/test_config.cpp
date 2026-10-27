// W05 authoring: no simulation, engine compilation or fixture generation is
// needed by these cold configuration tests. Execution belongs to V01.
#include "support/migration_test.hpp"
#include "spatial/support/spatial_fixture.hpp"
#include "fixture_paths.hpp"
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "configuration/rider_spec.hpp"
#include "io/float_format.hpp"
#include "io/json.hpp"
#include "runtime/capabilities.hpp"
#include "runtime/project_config.hpp"
#include "runtime/spatial_tire_factory.hpp"
#include "spatial/config.hpp"
#include "spatial/types.hpp"
#include "writer_config.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <variant>

namespace {
namespace cfg = bike::configuration;
namespace sp = bike::spatial;
namespace io = bike::io;
using io::Value;
using Object = Value::Object;

Value text(std::string_view value) { return Value{std::string(value)}; }
Value number(double value) { return Value{io::Number{io::repr_binary64(value)}}; }
Value integer(std::int64_t value) { return Value{io::Number{std::to_string(value)}}; }
const Value& field(const Value& object, std::string_view name) {
    const auto* found = io::find_member(io::as_object(object, name), name);
    if (found == nullptr) throw std::invalid_argument("test field missing: " + std::string(name));
    return *found;
}
Value& field(Value& object, std::string_view name) {
    for (auto& [key, value] : std::get<Object>(object.data)) if (key == name) return value;
    throw std::invalid_argument("test field missing: " + std::string(name));
}
void erase(Value& object, std::string_view name) {
    std::erase_if(std::get<Object>(object.data), [&](const auto& item) { return item.first == name; });
}
template<class Action>
void invalid_path(Action&& action, std::string_view path) {
    bt::invalid([&] {
        try { std::forward<Action>(action)(); }
        catch (const std::invalid_argument& error) {
            bt::require(std::string_view(error.what()).find(path) != std::string_view::npos,
                        "error must name field " + std::string(path) + ": " + error.what());
            throw;
        }
    });
}
using ConfigOwner = std::unique_ptr<cfg::SimulationPhysicsConfig>;
template<class Factory>
ConfigOwner own_config(Factory&& factory) {
    // Direct prvalue construction avoids a full stack temporary at the factory
    // boundary; the test keeps only an owning pointer in its frame.
    // NOLINTNEXTLINE(modernize-make-unique) make_unique would materialize the factory result before moving it
    return ConfigOwner(new cfg::SimulationPhysicsConfig(std::forward<Factory>(factory)()));
}
ConfigOwner load(std::string_view name) {
    return own_config([&] {
        return cfg::load_physics(spatial_tests::fixture("config/" + std::string(name)));
    });
}

void same_config(const sp::Config& a, const sp::Config& b) {
    bt::require(std::tie(a.mode, a.root, a.corridor_half_width_m, a.capsize_roll_rad) ==
                std::tie(b.mode, b.root, b.corridor_half_width_m, b.capsize_roll_rad), "config scalars round trip");
    bt::require(std::tie(a.steer.range_deg, a.steer.damping_nms_rad, a.steer.torque_cap_nm,
                        a.steer.reaction_delay_s, a.steer.controller, a.steer.profile_id, a.steer.profile_sha256) ==
                std::tie(b.steer.range_deg, b.steer.damping_nms_rad, b.steer.torque_cap_nm,
                         b.steer.reaction_delay_s, b.steer.controller, b.steer.profile_id, b.steer.profile_sha256),
                "every steer field round trips");
    bt::require(std::tie(a.bootstrap.settle_time_s, a.bootstrap.release_force_limit_n,
                        a.bootstrap.release_moment_limit_nm, a.bootstrap.hold_stiffness_n_per_m,
                        a.bootstrap.hold_damping_ns_per_m, a.bootstrap.hold_rot_stiffness_nm_per_rad,
                        a.bootstrap.hold_rot_damping_nms_per_rad) ==
                std::tie(b.bootstrap.settle_time_s, b.bootstrap.release_force_limit_n,
                         b.bootstrap.release_moment_limit_nm, b.bootstrap.hold_stiffness_n_per_m,
                         b.bootstrap.hold_damping_ns_per_m, b.bootstrap.hold_rot_stiffness_nm_per_rad,
                         b.bootstrap.hold_rot_damping_nms_per_rad), "every bootstrap field round trips");
    bt::require(std::tie(a.imu.mount_pos, a.imu.mount_rpy_rad, a.imu.axis_misalignment_rad, a.imu.calibration_revision) ==
                std::tie(b.imu.mount_pos, b.imu.mount_rpy_rad, b.imu.axis_misalignment_rad, b.imu.calibration_revision),
                "every IMU field round trips");
    bt::require(std::tie(a.terrain.kind, a.terrain.path, a.terrain.corridor_half_width_m,
                        a.terrain.max_asset_bytes, a.terrain.dx_m, a.terrain.dy_m) ==
                std::tie(b.terrain.kind, b.terrain.path, b.terrain.corridor_half_width_m,
                         b.terrain.max_asset_bytes, b.terrain.dx_m, b.terrain.dy_m), "every terrain field round trips");
    bt::require(std::tie(a.tyre.tier, a.tyre.lateral_coefficients_id, a.tyre.lateral_coefficients_sha256) ==
                std::tie(b.tyre.tier, b.tyre.lateral_coefficients_id, b.tyre.lateral_coefficients_sha256),
                "every tyre field round trips");
}

void enum_and_defaults() {
    for (const auto mode : {sp::Mode::planar, sp::Mode::spatial3d})
        bt::require(sp::parse_mode(sp::to_string(mode)) == mode, "mode spelling round trip");
    for (const auto root : {sp::RootKind::planar3, sp::RootKind::constrained4, sp::RootKind::free6})
        bt::require(sp::parse_root_kind(sp::to_string(root)) == root, "root spelling round trip");
    for (const auto tier : {sp::TyreTier::fast, sp::TyreTier::detailed})
        bt::require(sp::parse_tyre_tier(sp::to_string(tier)) == tier, "tier spelling round trip");
    bt::require(!sp::parse_mode("3D") && !sp::parse_root_kind("free6") && !sp::parse_tyre_tier("preview"),
                "unknown enum spellings are not aliases");
    const auto defaults = sp::parse_spatial_config(Value{Object{}});
    bt::require(defaults.mode == sp::Mode::planar && defaults.root == sp::RootKind::planar3,
                "absent spatial selects planar3");
    const auto free_defaults = sp::parse_spatial_config(Value{Object{{"spatial", text("3d")}}});
    bt::require(free_defaults.root == sp::RootKind::free6, "3D root default is free");
    bt::require(free_defaults.steer.range_deg == 60. && free_defaults.bootstrap.settle_time_s == 80.,
                "declared C1 starter defaults");
    bt::require(std::get<std::string>(free_defaults.imu.mount_pos) == "site_BB", "IMU mount default is explicit");
}

void positive_and_roundtrip() {
    for (const auto name : {"valid_free.toml", "valid_constrained.toml"}) {
        const auto config = load(name);
        bt::require(config->spatial.has_value(), "3D owns its extension");
        const auto report = sp::check_spatial_capability(*config->spatial, *config);
        bt::require(report.implemented && report.qualification_pending && report.rejections.empty(),
                    "W25/W31 source dispatch is connected while numerical qualification remains pending");
        const auto runtime_report = runtime::implemented_capabilities(*config, cfg::RiderSpecs{});
        bt::require(runtime_report.implemented && !runtime_report.supported && runtime_report.qualification_pending,
                    "connected runtime does not promote implementation into qualified support");
        const auto wire = sp::spatial_config_to_wire(*config->spatial);
        const auto restored = sp::spatial_config_from_wire(wire);
        same_config(*config->spatial, restored);
        bt::require(io::canonical_json(wire) == io::canonical_json(sp::spatial_config_to_wire(restored)), "canonical wire round trip");
        bt::require(sp::spatial_config_fingerprint_hex(*config->spatial) == sp::spatial_config_fingerprint_hex(restored), "fingerprint round trip");

        const auto emitted = cfg::write_physics_toml(*config);
        const auto input = cfg::detail::parse_toml(emitted, "spatial emitted TOML");
        const auto reloaded = own_config([&] { return cfg::detail::construct_config(input); });
        bt::require(reloaded->spatial.has_value(), "TOML emitter retains the extension");
        same_config(*config->spatial, *reloaded->spatial);
        const auto resolved = own_config([&] { return cfg::resolve_physics(Value{Object{}}, *config); });
        bt::require(resolved->spatial.has_value(), "empty override retains the extension");
        same_config(*config->spatial, *resolved->spatial);
    }

    auto altered = *load("valid_free.toml")->spatial;
    const auto before = sp::spatial_config_fingerprint_hex(altered);
    altered.steer.range_deg = 55.;
    altered.steer.damping_nms_rad = .75;
    altered.steer.torque_cap_nm = 17.;
    altered.steer.reaction_delay_s = .02;
    altered.steer.profile_id = "synthetic-steer";
    altered.steer.profile_sha256 = std::string(64, 'a'); // synthetic parser identity, not a measured digest
    altered.bootstrap = {70., .4, .5, 1900., 180., 350., 30.};
    altered.imu.mount_pos = sp::Vec3{.1, .2, .3};
    altered.imu.mount_rpy_rad = {.2, .3, .4};
    altered.imu.axis_misalignment_rad = {.001, -.002, .003};
    altered.imu.calibration_revision = "synthetic-calibration";
    altered.corridor_half_width_m = 2.5;
    altered.terrain = {sp::TerrainRequestKind::genspec, "synthetic.toml", 2.5, 123456, .02, .03};
    altered.capsize_roll_rad = .7;
    altered.tyre = {sp::TyreTier::detailed, "synthetic-lateral", std::string(64, 'b')};
    same_config(altered, sp::spatial_config_from_wire(sp::spatial_config_to_wire(altered)));
    bt::require(before != sp::spatial_config_fingerprint_hex(altered), "physical config changes change identity");
    auto owner = load("valid_free.toml");
    owner->spatial = altered;
    const auto reloaded = own_config([&] {
        return cfg::detail::construct_config(cfg::detail::parse_toml(
            cfg::write_physics_toml(*owner), "all spatial fields emitted TOML"));
    });
    bt::require(reloaded->spatial.has_value(), "nondefault TOML retains the extension");
    same_config(altered, *reloaded->spatial);
    const auto resolved = own_config([&] { return cfg::resolve_physics(Value{Object{}}, *owner); });
    bt::require(resolved->spatial.has_value(), "nondefault empty override retains the extension");
    same_config(altered, *resolved->spatial);

    // Nested input is a documented equivalent form; conflicts never pick a winner.
    auto nested = cfg::detail::parse_toml(
        "spatial='3d'\n[spatial_3d.tires.pneumatic_3d]\ntier='detailed'\n"
        "[tires.pneumatic_3d]\ntier='detailed'\n", "nested tyre control");
    bt::require(sp::parse_spatial_config(nested).tyre.tier == sp::TyreTier::detailed, "consistent aliases agree");
    field(field(field(nested, "tires"), "pneumatic_3d"), "tier") = text("fast");
    invalid_path([&] { (void)sp::parse_spatial_config(nested); }, "tires.pneumatic_3d.tier");
}

void override_resolution() {
    auto base = load("valid_free.toml");
    base->spatial->tyre.lateral_coefficients_sha256 = std::string(64, 'b'); // synthetic parser identity
    const auto resolve = [&](std::string_view source) {
        return own_config([&] {
            return cfg::resolve_physics(cfg::detail::parse_toml(source, "spatial override"), *base);
        });
    };
    const auto corridor = resolve("[spatial_3d]\ncorridor_half_width_m=2.0\n");
    bt::require(corridor->spatial->corridor_half_width_m == 2. &&
                corridor->spatial->terrain.corridor_half_width_m == 2.,
                "root corridor override updates the derived terrain width");
    const auto tier = resolve("[spatial_3d.tires.pneumatic_3d]\ntier='detailed'\n");
    bt::require(tier->spatial->tyre.tier == sp::TyreTier::detailed &&
                tier->spatial->tyre.lateral_coefficients_id == base->spatial->tyre.lateral_coefficients_id &&
                tier->spatial->tyre.lateral_coefficients_sha256 == base->spatial->tyre.lateral_coefficients_sha256,
                "nested tier override keeps other inherited tyre fields");
    const auto id = resolve("[spatial_3d]\nlateral_coefficients='synthetic-replacement'\n");
    bt::require(id->spatial->tyre.lateral_coefficients_id == "synthetic-replacement" &&
                id->spatial->tyre.lateral_coefficients_sha256 == base->spatial->tyre.lateral_coefficients_sha256,
                "string lateral alias overrides only its id");
    const auto identity = resolve(
        "[spatial_3d.tires.pneumatic_3d]\n"
        "lateral_coefficients={id='synthetic-replacement',sha256='" + std::string(64, 'c') + "'}\n");
    bt::require(identity->spatial->tyre.lateral_coefficients_id == "synthetic-replacement" &&
                identity->spatial->tyre.lateral_coefficients_sha256 == std::string(64, 'c'),
                "object lateral alias replaces the complete profile identity");
    invalid_path([&] {
        (void)resolve("[spatial_3d.tires.pneumatic_3d]\ntier='detailed'\n"
                      "[tires.pneumatic_3d]\ntier='fast'\n");
    }, "tires.pneumatic_3d.tier");
    const auto planar = resolve("spatial='planar'\n[tires]\nbackend='native_reference'\nsurface_mode='configured'\n");
    bt::require(!planar->spatial && planar->tires.backend == "native_reference", "explicit transition removes inherited extension");
    const auto planar_value = cfg::to_value(*planar);
    bt::require(io::find_member(io::as_object(planar_value, "planar"), "spatial_3d") == nullptr &&
                io::find_member(io::as_object(field(planar_value, "tires"), "tires"), "pneumatic_3d") == nullptr,
                "planar output contains no inherited spatial sections");
    // The shoulder profile lives in the planar articulated table but is
    // spatial-only: it must survive a 3D round trip and be dropped, not
    // inherited, by an explicit planar transition.
    const auto shouldered = own_config([&] { return cfg::SimulationPhysicsConfig(*base); });
    shouldered->articulated.shoulder_profile = "synthetic-shoulder.json";
    const auto kept = own_config([&] { return cfg::resolve_physics(Value{Object{}}, *shouldered); });
    bt::require(kept->articulated.shoulder_profile == std::optional<std::string>("synthetic-shoulder.json"),
                "3D resolution round-trips the shoulder profile");
    const auto shoulder_planar = own_config([&] {
        return cfg::resolve_physics(cfg::detail::parse_toml(
            "spatial='planar'\n[tires]\nbackend='native_reference'\nsurface_mode='configured'\n", "planar shoulder"),
            *shouldered);
    });
    bt::require(!shoulder_planar->spatial && !shoulder_planar->articulated.shoulder_profile &&
                !shoulder_planar->articulated.shoulder_profile_sha256,
                "explicit planar transition drops the inherited shoulder profile");
    invalid_path([&] {
        (void)own_config([&] {
            return cfg::resolve_physics(cfg::detail::parse_toml(
                "spatial='planar'\n[tires]\nbackend='native_reference'\nsurface_mode='configured'\n"
                "[articulated]\nshoulder_profile='explicit.json'\n", "planar shoulder override"), *shouldered);
        });
    }, "articulated.shoulder_profile");
    invalid_path([&] { (void)resolve("spatial='planar'\n"); }, "tires.backend");
    invalid_path([&] {
        (void)resolve("spatial='planar'\n[spatial_3d]\nroot='free'\n");
    }, "spatial_3d");
    const auto legacy = own_config([] { return cfg::default_physics(); });
    const auto legacy_wire = io::canonical_json(cfg::to_value(*legacy));
    for (const auto& override_doc : {
             Value{Object{{"spatial", text("planar")}, {"tires", Value{}}}},
             Value{Object{{"spatial", text("planar")}, {"tires", Value{Object{{"backend", Value{}}}}}}}}) {
        const auto retained = own_config([&] { return cfg::resolve_physics(override_doc, *legacy); });
        bt::require(io::canonical_json(cfg::to_value(*retained)) == legacy_wire,
                    "explicit planar preserves legacy null-as-absent override semantics");
        invalid_path([&] {
            (void)own_config([&] { return cfg::resolve_physics(override_doc, *base); });
        }, "tires.backend");
    }
    invalid_path([&] {
        (void)own_config([&] {
            return cfg::resolve_physics(Value{Object{{"spatial", text("planar")}, {"spatial_3d", Value{}}}}, *legacy);
        });
    }, "spatial_3d");

    const auto filename = spatial_tests::fixture("config/valid_constrained.toml");
    auto document = cfg::detail::parse_toml_file(filename);
    cfg::detail::resolve_config_paths(document, filename.parent_path());
    const auto request = own_config([&] { return cfg::detail::construct_config(document); });
    bt::require(request->spatial->terrain.path ==
                std::filesystem::weakly_canonical(filename.parent_path() / "synthetic-grid.genspec.toml"),
                "shared path resolver anchors spatial assets to the declaring TOML");
}

void negative_fixtures() {
    constexpr std::array cases{
        std::pair{"neg_spatial_table_without_mode.toml", "spatial_3d"},
        std::pair{"neg_3d_planar_tyre.toml", "tires.backend"},
        std::pair{"neg_planar_pneumatic3d.toml", "tires.backend"},
        std::pair{"neg_constrained_controller.toml", "spatial_3d.steer.controller"},
        std::pair{"neg_nan_grid.toml", "spatial_3d.terrain.dx_m"},
        std::pair{"neg_unknown_key.toml", "spatial_3d.steer.telepathy"},
        std::pair{"neg_unsupported_schema.toml", "spatial_3d.spatial_contract"}
    };
    for (const auto& [name, path] : cases) invalid_path([&] { (void)load(name); }, path);
    const auto bad_combo = load("neg_rider_drive_combo.toml");
    auto rider = cfg::RiderSpecs{};
    rider.variant = "lumped";
    invalid_path([&] { runtime::validate_physical_capabilities(*bad_combo, rider); }, "rider.variant");

    for (const auto value : {0., -1., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        auto bad = *load("valid_constrained.toml")->spatial;
        bad.terrain.dx_m = value;
        invalid_path([&] { (void)sp::spatial_config_to_wire(bad); }, "spatial_3d.terrain.dx_m");
    }
    for (const auto range : {0., 90., 91., std::numeric_limits<double>::infinity()}) {
        auto bad = *load("valid_free.toml")->spatial;
        bad.steer.range_deg = range;
        invalid_path([&] { (void)sp::spatial_config_to_wire(bad); }, "spatial_3d.steer.range_deg");
    }
    auto typed = *load("valid_free.toml")->spatial;
    typed.terrain.max_asset_bytes = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) + 1;
    invalid_path([&] { (void)sp::spatial_config_to_wire(typed); }, "spatial_3d.terrain.max_asset_bytes");
    typed = *load("valid_free.toml")->spatial;
    typed.steer.profile_id = std::string("bad\0profile", 11);
    invalid_path([&] { (void)sp::spatial_config_to_wire(typed); }, "spatial_3d.steer.profile_id");
    typed = *load("valid_free.toml")->spatial;
    typed.imu.calibration_revision = std::string("bad\0revision", 12);
    invalid_path([&] { (void)sp::spatial_config_to_wire(typed); }, "spatial_3d.imu.calibration_revision");
    typed = *load("valid_free.toml")->spatial;
    typed.tyre.lateral_coefficients_id = std::string("bad\0profile", 11);
    invalid_path([&] { (void)sp::spatial_config_to_wire(typed); }, "tires.pneumatic_3d.lateral_coefficients");
    typed = *load("valid_constrained.toml")->spatial;
    typed.terrain.path = std::string("bad\0path", 8);
    invalid_path([&] { (void)sp::spatial_config_to_wire(typed); }, "spatial_3d.terrain.path");
    const auto good = sp::spatial_config_to_wire(*load("valid_free.toml")->spatial);
    auto bad = good;
    field(bad, "spatial_contract") = integer(2);
    invalid_path([&] { (void)sp::spatial_config_from_wire(bad); }, "config.spatial.spatial_contract");
    field(bad, "spatial_contract") = number(1.0);
    invalid_path([&] { (void)sp::spatial_config_from_wire(bad); }, "config.spatial.spatial_contract");
    bad = good;
    erase(bad, "profile");
    invalid_path([&] { (void)sp::spatial_config_from_wire(bad); }, "config.spatial.profile");
    bad = good;
    std::get<Object>(field(bad, "imu").data).emplace_back("unknown", Value{false});
    invalid_path([&] { (void)sp::spatial_config_from_wire(bad); }, "config.spatial.imu.unknown");
    bad = good;
    std::get<Object>(bad.data).emplace_back("mode", text("3d"));
    invalid_path([&] { (void)sp::spatial_config_from_wire(bad); }, "config.spatial.mode");
    bad = good;
    field(field(bad, "profile"), "capsize_roll_rad") = number(0.);
    invalid_path([&] { (void)sp::spatial_config_from_wire(bad); }, "config.spatial.profile.capsize_roll_rad");
    bad = good;
    field(field(bad, "profile"), "lateral_coefficients_sha256") = text("not-a-digest");
    invalid_path([&] { (void)sp::spatial_config_from_wire(bad); }, "config.spatial.profile.lateral_coefficients_sha256");
    bad = good;
    field(field(bad, "terrain"), "max_asset_bytes") = Value{io::Number{"9223372036854775808"}};
    invalid_path([&] { (void)sp::spatial_config_from_wire(bad); }, "config.spatial.terrain.max_asset_bytes");
}

void planar_compatibility() {
    const auto before = own_config([] { return cfg::default_physics(); });
    const auto after = load("planar_explicit.toml");
    bt::require(!before->spatial && !after->spatial, "explicit planar is normalized to absent");
    const auto legacy_wire_before = io::canonical_json(cfg::to_value(*before));
    const auto legacy_wire_after = io::canonical_json(cfg::to_value(*after));
    bt::require(legacy_wire_before == legacy_wire_after, "legacy_wire_before == legacy_wire_after");
    const auto frozen_schema = io::parse_json(bt::text(bt::fixture("config/schema.json")));
    bt::require(io::canonical_json(cfg::schema()) == io::canonical_json(frozen_schema), "legacy field schema matches existing frozen data");

    std::size_t toml_count = 0, physics_count = 0;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(
             std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT) / "examples")) {
        if (!entry.is_regular_file() || entry.path().extension() != ".toml") continue;
        const auto document = cfg::detail::parse_toml_file(entry.path());
        const auto& root = io::as_object(document, "example");
        if (io::find_member(root, "spatial") != nullptr) continue; // explicit future spatial example has its own contract
        ++toml_count;
        bt::require(sp::parse_spatial_config(document).mode == sp::Mode::planar, "old TOML has no spatial extension");
        // Track/generator/batch TOMLs retain their own schemas; only physics
        // documents belong to load_physics. Every TOML was parsed above.
        if (io::find_member(root, "physics_mode") == nullptr) continue;
        ++physics_count;
        const auto legacy = own_config([&] { return cfg::load_physics(entry.path()); });
        const auto explicit_planar = own_config([&] {
            return cfg::resolve_physics(Value{Object{{"spatial", text("planar")}}}, *legacy);
        });
        bt::require(!legacy->spatial && !explicit_planar->spatial, "old physics stays planar");
        bt::require(io::canonical_json(cfg::to_value(*legacy)) == io::canonical_json(cfg::to_value(*explicit_planar)),
                    "every existing physics profile preserves canonical bytes with explicit planar");
        bt::require(cfg::write_physics_toml(*legacy) == cfg::write_physics_toml(*explicit_planar), "planar TOML output is byte-identical");
    }
    bt::require(toml_count > 0 && physics_count > 0, "example sweep must execute real checks");
}

Value spatial_writer(const cfg::SimulationPhysicsConfig& config) {
    // Synthetic hashes are intentionally confined to a cold parser control.
    return Value{Object{{"schema", integer(3)}, {"spatial", Value{Object{
        {"spatial_contract", integer(1)}, {"root_kind", text(sp::to_string(config.spatial->root))},
        {"topology_fingerprint", text(std::string(64, 'a'))}, {"terrain_sha256", text(std::string(64, 'b'))},
        {"numeric_profile", text("synthetic-parser-control")}}}},
        {"drive", runtime::project_drive_config(config)},
        {"tire", Value{Object{{"backend", text("pneumatic_3d")}, {"pneumatic_3d", Value{Object{}}}}}}}};
}
Value resolved_fields(const cfg::SimulationPhysicsConfig& config) {
    return Value{Object{
        {"timestep_s", number(config.timestep_s)}, {"control_period_s", number(config.control_period_s)},
        {"control_period_steps", integer(10)}, {"physics_mode", text("physical")}, {"drive_mode", text(*config.drive_mode)},
        {"strict", Value{false}}, {"measurement_decimation", integer(1)}, {"record_retention", text("off")},
        {"writer_config", spatial_writer(config)}, {"rider_controller", Value{Object{}}}, {"rider_intent", Value{Object{}}},
        {"monitors", Value{Object{{"balance_floor_mps", number(.5)}, {"balance_dwell_s", number(1.)},
            {"balance_grace_s", number(0.)}, {"grounded_hold_s", number(0.)}}}},
        {"geometry", Value{Object{{"crank_length_m", number(.17)},
            {"brake_dofs", Value{Object{
                {"front", Value{Object{{"joint", text("front_wheel_spin")}, {"dof_index", integer(0)}}}},
                {"rear", Value{Object{{"joint", text("rear_wheel_spin")}, {"dof_index", integer(1)}}}}}}},
            {"wheel_bodies", Value{Object{{"front", text("front_wheel")}, {"rear", text("rear_wheel")}}}},
            {"com_marker_site", Value{}}, {"pose", Value{}}}}}
    }};
}

void envelope_dispatch() {
    const auto config = load("valid_free.toml");
    const auto fields = resolved_fields(*config);
    const auto original = io::canonical_json(fields);
    const auto projected = runtime::project_spatial_runtime_envelope(*config, fields);
    bt::require(io::as_int64(field(projected, "runtime_schema"), "schema") == 2, "3D envelope is runtime2");
    bt::require(io::as_int64(field(field(projected, "writer_config"), "schema"), "schema") == 3, "3D envelope is writer3");
    same_config(*config->spatial, sp::spatial_config_from_wire(field(projected, "spatial")));
    bt::require(original == io::canonical_json(fields), "projection does not mutate caller-owned input");
    invalid_path([&] { (void)nativecfg::decode(field(projected, "writer_config")); }, "config.tire.pneumatic_3d");
    auto connected=field(projected,"writer_config");
    field(field(connected,"tire"),"pneumatic_3d")=runtime::project_spatial_tire_config(*config);
    (void)nativecfg::decode(connected);

    auto bad = fields;
    std::get<Object>(bad.data).emplace_back("runtime_schema", integer(1));
    invalid_path([&] { (void)runtime::project_spatial_runtime_envelope(*config, bad); }, "config.runtime_schema");
    bad = fields;
    field(field(bad, "writer_config"), "schema") = integer(2);
    invalid_path([&] { (void)runtime::project_spatial_runtime_envelope(*config, bad); }, "config.schema");
    bad = fields;
    field(field(field(bad, "writer_config"), "spatial"), "root_kind") = text("constrained");
    invalid_path([&] { (void)runtime::project_spatial_runtime_envelope(*config, bad); }, "config.writer_config.spatial.root_kind");
    auto writer = spatial_writer(*config);
    erase(field(writer, "spatial"), "terrain_sha256");
    invalid_path([&] { nativecfg::validate_spatial_writer_envelope(writer); }, "config.spatial.terrain_sha256");
    writer = spatial_writer(*config);
    field(field(writer, "tire"), "backend") = text("pneumatic");
    invalid_path([&] { nativecfg::validate_spatial_writer_envelope(writer); }, "config.tire.backend");
    writer = spatial_writer(*config);
    field(writer, "schema") = number(3.0);
    invalid_path([&] { (void)nativecfg::decode(writer); }, "config.schema");
    writer = spatial_writer(*config);
    std::get<Object>(field(writer, "spatial").data).emplace_back("mystery", Value{true});
    invalid_path([&] { (void)nativecfg::decode(writer); }, "config.spatial.mystery");
    (void)nativecfg::decode(Value{Object{{"schema", integer(1)}}});
    (void)nativecfg::decode(Value{Object{{"schema", integer(2)}}});
    bt::require(runtime::kRuntimeSchema == 1, "legacy runtime version is unchanged");
}
} // namespace

int main() {
    return bt::run([] {
        enum_and_defaults();
        positive_and_roundtrip();
        override_resolution();
        negative_fixtures();
        planar_compatibility();
        envelope_dispatch();
    });
}
