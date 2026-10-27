#include "config.hpp"

#include "../configuration/model_config.hpp"
#include "../io/float_format.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <set>
#include <span>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <utility>

namespace bike::spatial {
namespace {
using bike::io::Value;
using Object = Value::Object;
using Keys = std::initializer_list<std::string_view>;

[[noreturn]] void fail(std::string_view path, std::string_view message) {
    throw std::invalid_argument(std::string(path) + ": " + std::string(message));
}
std::string child(std::string_view path, std::string_view key) {
    return path.empty() ? std::string(key) : std::string(path) + "." + std::string(key);
}
const Value* find(const Object& object, std::string_view key) noexcept {
    return bike::io::find_member(object, key);
}
const Value& required(const Object& object, std::string_view key, std::string_view path) {
    if (const auto* value = find(object, key)) return *value;
    fail(child(path, key), "missing required field");
}
const Object& table(const Value& value, std::string_view path, Keys allowed, bool exact = false) {
    const auto& object = bike::io::as_object(value, path);
    std::set<std::string_view> seen;
    for (const auto& [key, unused] : object) {
        (void)unused;
        if (!seen.insert(key).second) fail(child(path, key), "duplicate key");
        if (std::ranges::find(allowed, key) == allowed.end())
            fail(child(path, key), "unknown key");
    }
    if (exact) for (const auto key : allowed) (void)required(object, key, path);
    return object;
}
double finite(const Value& value, std::string_view path) {
    return bike::io::as_double(value, path);
}
void positive(double value, std::string_view path) {
    if (!std::isfinite(value) || value <= 0.) fail(path, "must be finite and > 0");
}
void nonnegative(double value, std::string_view path) {
    if (!std::isfinite(value) || value < 0.) fail(path, "must be finite and >= 0");
}
std::uint64_t unsigned_integer(const Value& value, std::string_view path) {
    const auto& number = bike::io::as_number(value, path);
    if (!bike::io::is_integer_lexeme(number.lexeme) || number.lexeme.front() == '-')
        fail(path, "expected unsigned integer");
    std::uint64_t result = 0;
    const auto* end = std::to_address(number.lexeme.cend());
    const auto parsed = std::from_chars(number.lexeme.data(), end, result);
    if (parsed.ec != std::errc{} || parsed.ptr != end) fail(path, "integer out of range");
    return result;
}
Vec3 vector3(const Value& value, std::string_view path) {
    const auto& array = bike::io::as_array(value, path);
    if (array.size() != 3) fail(path, "expected exactly three coordinates");
    return {finite(array[0], child(path, "0")), finite(array[1], child(path, "1")),
            finite(array[2], child(path, "2"))};
}
void finite_vector(const Vec3& value, std::string_view path) {
    for (std::size_t index = 0; index < value.size(); ++index)
        if (!std::isfinite(value[index])) fail(child(path, std::to_string(index)), "must be finite");
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (value, path) matches the file's helper convention
void string_domain(std::string_view value, std::string_view path) {
    if (value.find('\0') != std::string_view::npos) fail(path, "embedded NUL is forbidden");
}
std::string string(const Value& value, std::string_view path) {
    auto result = bike::io::as_string(value, path);
    string_domain(result, path);
    return result;
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (value, path) matches the file's helper convention
void digest(std::string_view value, std::string_view path) {
    if (value.empty()) return; // No acquired profile identity: capability stays closed.
    if (value.size() != 64 || !std::ranges::all_of(value, [](char ch) {
            return (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f');
        })) fail(path, "expected 64 lowercase hexadecimal characters");
}
void read_real(const Object& object, std::string_view key, double& target, std::string_view path) {
    if (const auto* value = find(object, key)) target = finite(*value, child(path, key));
}
void read_string(const Object& object, std::string_view key, std::string& target, std::string_view path) {
    if (const auto* value = find(object, key)) target = string(*value, child(path, key));
}

void steer(const Value& value, SteerConfig& out, std::string_view path, bool exact = false) {
    const auto& object = table(value, path, {"range_deg", "damping_nms_rad", "torque_cap_nm",
        "reaction_delay_s", "controller", "profile_id", "profile_sha256"}, exact);
    read_real(object, "range_deg", out.range_deg, path);
    read_real(object, "damping_nms_rad", out.damping_nms_rad, path);
    read_real(object, "reaction_delay_s", out.reaction_delay_s, path);
    if (const auto* cap = find(object, "torque_cap_nm")) {
        if (const auto* text = std::get_if<std::string>(&cap->data)) {
            if (*text != "strength") fail(child(path, "torque_cap_nm"), "expected positive real or 'strength'");
            out.torque_cap_nm.reset();
        } else out.torque_cap_nm = finite(*cap, child(path, "torque_cap_nm"));
    }
    if (const auto* controller = find(object, "controller")) {
        const auto text = string(*controller, child(path, "controller"));
        if (text == "off") out.controller = SteerController::off;
        else if (text == "roll_heading") out.controller = SteerController::roll_heading;
        else fail(child(path, "controller"), "expected 'off' or 'roll_heading'");
    }
    read_string(object, "profile_id", out.profile_id, path);
    read_string(object, "profile_sha256", out.profile_sha256, path);
}
void bootstrap(const Value& value, BootstrapConfig& out, std::string_view path, bool exact = false) {
    const auto& object = table(value, path, {"settle_time_s", "release_force_limit_n", "release_moment_limit_nm",
        "hold_stiffness_n_per_m", "hold_damping_ns_per_m", "hold_rot_stiffness_nm_per_rad",
        "hold_rot_damping_nms_per_rad"}, exact);
    read_real(object, "settle_time_s", out.settle_time_s, path);
    read_real(object, "release_force_limit_n", out.release_force_limit_n, path);
    read_real(object, "release_moment_limit_nm", out.release_moment_limit_nm, path);
    read_real(object, "hold_stiffness_n_per_m", out.hold_stiffness_n_per_m, path);
    read_real(object, "hold_damping_ns_per_m", out.hold_damping_ns_per_m, path);
    read_real(object, "hold_rot_stiffness_nm_per_rad", out.hold_rot_stiffness_nm_per_rad, path);
    read_real(object, "hold_rot_damping_nms_per_rad", out.hold_rot_damping_nms_per_rad, path);
}
void imu(const Value& value, ImuMountConfig& out, std::string_view path, bool exact = false) {
    const auto& object = table(value, path, {"mount_pos_m", "mount_rpy_rad", "axis_misalignment_rad",
                                          "calibration_revision"}, exact);
    if (const auto* mount = find(object, "mount_pos_m")) {
        if (std::holds_alternative<std::string>(mount->data)) out.mount_pos = string(*mount, child(path, "mount_pos_m"));
        else out.mount_pos = vector3(*mount, child(path, "mount_pos_m"));
    }
    if (const auto* rpy = find(object, "mount_rpy_rad")) out.mount_rpy_rad = vector3(*rpy, child(path, "mount_rpy_rad"));
    if (const auto* axis = find(object, "axis_misalignment_rad")) out.axis_misalignment_rad = vector3(*axis, child(path, "axis_misalignment_rad"));
    read_string(object, "calibration_revision", out.calibration_revision, path);
}
std::string_view terrain_kind(TerrainRequestKind value) {
    switch (value) {
    case TerrainRequestKind::none: return "none";
    case TerrainRequestKind::grid_asset: return "grid_asset";
    case TerrainRequestKind::genspec: return "genspec";
    case TerrainRequestKind::profile_extrusion: return "profile_extrusion";
    }
    fail("spatial_3d.terrain.kind", "unknown terrain request kind");
}
void terrain(const Value& value, TerrainRequest3d& out, std::string_view path, bool exact = false) {
    const auto& object = table(value, path, {"kind", "path", "corridor_half_width_m", "max_asset_bytes", "dx_m", "dy_m"}, exact);
    if (const auto* kind = find(object, "kind")) {
        const auto name = string(*kind, child(path, "kind"));
        if (name == "none") out.kind = TerrainRequestKind::none;
        else if (name == "grid_asset") out.kind = TerrainRequestKind::grid_asset;
        else if (name == "genspec") out.kind = TerrainRequestKind::genspec;
        else if (name == "profile_extrusion") out.kind = TerrainRequestKind::profile_extrusion;
        else fail(child(path, "kind"), "unknown terrain request kind");
    }
    if (const auto* item = find(object, "path")) out.path = string(*item, child(path, "path"));
    read_real(object, "corridor_half_width_m", out.corridor_half_width_m, path);
    if (const auto* item = find(object, "max_asset_bytes")) out.max_asset_bytes = unsigned_integer(*item, child(path, "max_asset_bytes"));
    for (const auto& [key, target] : {std::pair{"dx_m", &out.dx_m}, std::pair{"dy_m", &out.dy_m}}) {
        if (const auto* item = find(object, key)) {
            if (exact && bike::io::is_null(*item)) target->reset();
            else *target = finite(*item, child(path, key));
        }
    }
}

struct TyreInput {
    std::optional<TyreTier> tier;
    std::optional<std::string> id, sha256;
};
template<class T>
void consistent(std::optional<T>& target, T value, std::string_view path) {
    if (target && *target != value) fail(path, "conflicting spatial tyre declarations");
    target = std::move(value);
}
void lateral(const Value& value, TyreInput& out, std::string_view path) {
    if (std::holds_alternative<std::string>(value.data)) {
        auto id = string(value, path);
        if (id.empty()) fail(path, "profile id must not be empty");
        consistent(out.id, std::move(id), path);
    } else {
        const auto& object = table(value, path, {"id", "sha256"}, true);
        auto id = string(required(object, "id", path), child(path, "id"));
        if (id.empty()) fail(child(path, "id"), "profile id must not be empty");
        consistent(out.id, std::move(id), path);
        consistent(out.sha256, string(required(object, "sha256", path), child(path, "sha256")), path);
    }
}
void tyre(const Value& value, TyreInput& out, std::string_view path) {
    const auto& object = table(value, path, {"tier", "lateral_coefficients", "lateral_coefficients_sha256"});
    if (const auto* item = find(object, "tier")) {
        const auto parsed = parse_tyre_tier(string(*item, child(path, "tier")));
        if (!parsed) fail(child(path, "tier"), "expected 'fast' or 'detailed'");
        consistent(out.tier, *parsed, child(path, "tier"));
    }
    if (const auto* item = find(object, "lateral_coefficients")) lateral(*item, out, child(path, "lateral_coefficients"));
    if (const auto* item = find(object, "lateral_coefficients_sha256"))
        consistent(out.sha256, string(*item, child(path, "lateral_coefficients_sha256")), child(path, "lateral_coefficients_sha256"));
}

void shape(const Config& cfg) {
    if (to_string(cfg.mode).empty()) fail("spatial", "unknown mode");
    if (to_string(cfg.root).empty()) fail("spatial_3d.root", "unknown root kind");
    if (cfg.mode == Mode::planar) {
        if (cfg.root != RootKind::planar3) fail("spatial_3d.root", "planar mode requires planar root");
        return;
    }
    if (cfg.root == RootKind::planar3) fail("spatial_3d.root", "3d mode requires 'constrained' or 'free'");
    string_domain(cfg.steer.profile_id, "spatial_3d.steer.profile_id");
    string_domain(cfg.imu.calibration_revision, "spatial_3d.imu.calibration_revision");
    string_domain(cfg.terrain.path.generic_string(), "spatial_3d.terrain.path");
    string_domain(cfg.tyre.lateral_coefficients_id, "tires.pneumatic_3d.lateral_coefficients");
    positive(cfg.corridor_half_width_m, "spatial_3d.corridor_half_width_m");
    if (!std::isfinite(cfg.capsize_roll_rad) || cfg.capsize_roll_rad <= 0. || cfg.capsize_roll_rad >= std::numbers::pi / 2.)
        fail("spatial_3d.capsize_roll_rad", "must be in (0,pi/2)");
    if (!std::isfinite(cfg.steer.range_deg) || cfg.steer.range_deg <= 0. || cfg.steer.range_deg >= 90.)
        fail("spatial_3d.steer.range_deg", "must be in (0,90)");
    nonnegative(cfg.steer.damping_nms_rad, "spatial_3d.steer.damping_nms_rad");
    nonnegative(cfg.steer.reaction_delay_s, "spatial_3d.steer.reaction_delay_s");
    if (cfg.steer.torque_cap_nm) positive(*cfg.steer.torque_cap_nm, "spatial_3d.steer.torque_cap_nm");
    if (cfg.steer.controller != SteerController::off && cfg.steer.controller != SteerController::roll_heading)
        fail("spatial_3d.steer.controller", "unknown controller");
    if (cfg.root == RootKind::constrained4 && cfg.steer.controller != SteerController::off)
        fail("spatial_3d.steer.controller", "constrained root requires 'off'");
    digest(cfg.steer.profile_sha256, "spatial_3d.steer.profile_sha256");
    if (!cfg.steer.profile_sha256.empty() && cfg.steer.profile_id.empty())
        fail("spatial_3d.steer.profile_id", "profile hash requires an id");
    const auto& boot = cfg.bootstrap;
    positive(boot.settle_time_s, "spatial_3d.bootstrap.settle_time_s");
    positive(boot.release_force_limit_n, "spatial_3d.bootstrap.release_force_limit_n");
    positive(boot.release_moment_limit_nm, "spatial_3d.bootstrap.release_moment_limit_nm");
    positive(boot.hold_stiffness_n_per_m, "spatial_3d.bootstrap.hold_stiffness_n_per_m");
    nonnegative(boot.hold_damping_ns_per_m, "spatial_3d.bootstrap.hold_damping_ns_per_m");
    positive(boot.hold_rot_stiffness_nm_per_rad, "spatial_3d.bootstrap.hold_rot_stiffness_nm_per_rad");
    nonnegative(boot.hold_rot_damping_nms_per_rad, "spatial_3d.bootstrap.hold_rot_damping_nms_per_rad");
    if (const auto* name = std::get_if<std::string>(&cfg.imu.mount_pos)) {
        if (*name != "site_BB") fail("spatial_3d.imu.mount_pos_m", "expected 'site_BB' or three finite coordinates");
    } else finite_vector(std::get<Vec3>(cfg.imu.mount_pos), "spatial_3d.imu.mount_pos_m");
    finite_vector(cfg.imu.mount_rpy_rad, "spatial_3d.imu.mount_rpy_rad");
    finite_vector(cfg.imu.axis_misalignment_rad, "spatial_3d.imu.axis_misalignment_rad");
    (void)terrain_kind(cfg.terrain.kind);
    positive(cfg.terrain.corridor_half_width_m, "spatial_3d.terrain.corridor_half_width_m");
    if (cfg.terrain.corridor_half_width_m != cfg.corridor_half_width_m)
        fail("spatial_3d.terrain.corridor_half_width_m", "must equal spatial_3d.corridor_half_width_m");
    if (cfg.terrain.max_asset_bytes == 0) fail("spatial_3d.terrain.max_asset_bytes", "must be > 0");
    if (cfg.terrain.max_asset_bytes > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
        fail("spatial_3d.terrain.max_asset_bytes", "must fit the signed 64-bit TOML integer domain");
    if (cfg.terrain.kind != TerrainRequestKind::none && cfg.terrain.path.empty())
        fail("spatial_3d.terrain.path", "required for selected terrain request");
    if (cfg.terrain.kind == TerrainRequestKind::none && !cfg.terrain.path.empty())
        fail("spatial_3d.terrain.path", "kind 'none' cannot carry an asset path");
    if (cfg.terrain.dx_m) positive(*cfg.terrain.dx_m, "spatial_3d.terrain.dx_m");
    if (cfg.terrain.dy_m) positive(*cfg.terrain.dy_m, "spatial_3d.terrain.dy_m");
    if ((cfg.terrain.dx_m || cfg.terrain.dy_m) && cfg.terrain.kind != TerrainRequestKind::genspec)
        fail("spatial_3d.terrain.kind", "grid step overrides require 'genspec'");
    if (to_string(cfg.tyre.tier).empty()) fail("tires.pneumatic_3d.tier", "unknown tier");
    digest(cfg.tyre.lateral_coefficients_sha256, "tires.pneumatic_3d.lateral_coefficients_sha256");
    if (!cfg.tyre.lateral_coefficients_sha256.empty() && cfg.tyre.lateral_coefficients_id.empty())
        fail("tires.pneumatic_3d.lateral_coefficients", "profile hash requires an id");
}

Value text(std::string_view value) { return Value{std::string(value)}; }
Value real(double value) { return Value{bike::io::Number{bike::io::repr_binary64(value)}}; }
Value integer(std::uint64_t value) { return Value{bike::io::Number{std::to_string(value)}}; }
Value optional_real(const std::optional<double>& value) { return value ? real(*value) : Value{}; }
Value vector_value(const Vec3& value) {
    return Value{Value::Array{real(value[0]), real(value[1]), real(value[2])}};
}
} // namespace

Config parse_spatial_config(const Value& physics_table) {
    const auto& physics = bike::io::as_object(physics_table, "physics");
    Config out;
    if (const auto* mode = find(physics, "spatial")) {
        const auto parsed = parse_mode(string(*mode, "spatial"));
        if (!parsed) fail("spatial", "expected 'planar' or '3d'");
        out.mode = *parsed;
    }
    const auto* extension = find(physics, "spatial_3d");
    const Object* tires = nullptr;
    if (const auto* value = find(physics, "tires")) tires = &bike::io::as_object(*value, "tires");
    if (out.mode == Mode::planar) {
        if (extension != nullptr) fail("spatial_3d", "requires spatial='3d'");
        if (tires != nullptr) {
            if (const auto* backend = find(*tires, "backend"); backend != nullptr &&
                string(*backend, "tires.backend") == "pneumatic_3d")
                fail("tires.backend", "pneumatic_3d requires spatial='3d'");
            if (find(*tires, "pneumatic_3d")) fail("tires.pneumatic_3d", "requires spatial='3d'");
        }
        return out;
    }
    out.root = RootKind::free6;
    TyreInput tyre_input;
    if (extension != nullptr) {
        const auto& object = table(*extension, "spatial_3d", {"root", "corridor_half_width_m", "capsize_roll_rad",
            "steer", "bootstrap", "imu", "terrain", "tires", "lateral_coefficients"});
        if (const auto* root = find(object, "root")) {
            const auto parsed = parse_root_kind(string(*root, "spatial_3d.root"));
            if (!parsed || *parsed == RootKind::planar3) fail("spatial_3d.root", "expected 'constrained' or 'free'");
            out.root = *parsed;
        }
        read_real(object, "corridor_half_width_m", out.corridor_half_width_m, "spatial_3d");
        read_real(object, "capsize_roll_rad", out.capsize_roll_rad, "spatial_3d");
        out.terrain.corridor_half_width_m = out.corridor_half_width_m;
        if (const auto* section = find(object, "steer")) steer(*section, out.steer, "spatial_3d.steer");
        if (const auto* section = find(object, "bootstrap")) bootstrap(*section, out.bootstrap, "spatial_3d.bootstrap");
        if (const auto* section = find(object, "imu")) imu(*section, out.imu, "spatial_3d.imu");
        if (const auto* section = find(object, "terrain")) terrain(*section, out.terrain, "spatial_3d.terrain");
        if (const auto* section = find(object, "lateral_coefficients")) lateral(*section, tyre_input, "spatial_3d.lateral_coefficients");
        if (const auto* section = find(object, "tires")) {
            const auto& nested = table(*section, "spatial_3d.tires", {"pneumatic_3d"});
            if (const auto* tire = find(nested, "pneumatic_3d")) tyre(*tire, tyre_input, "spatial_3d.tires.pneumatic_3d");
        }
    }
    if (tires != nullptr) if (const auto* section = find(*tires, "pneumatic_3d"))
        tyre(*section, tyre_input, "tires.pneumatic_3d");
    if (tyre_input.tier) out.tyre.tier = *tyre_input.tier;
    if (tyre_input.id) out.tyre.lateral_coefficients_id = *tyre_input.id;
    if (tyre_input.sha256) out.tyre.lateral_coefficients_sha256 = *tyre_input.sha256;
    shape(out);
    return out;
}

void validate_spatial_config(const Config& config, const bike::configuration::SimulationPhysicsConfig& physics) {
    shape(config);
    if (config.mode == Mode::planar) {
        if (physics.tires.backend == "pneumatic_3d") fail("tires.backend", "pneumatic_3d requires spatial='3d'");
        return;
    }
    if (physics.physics_mode != "physical") fail("physics_mode", "spatial='3d' requires physical mode");
    if (physics.tires.backend != "pneumatic_3d") fail("tires.backend", "spatial='3d' requires pneumatic_3d");
    positive(physics.timestep_s, "timestep_s");
    positive(physics.control_period_s, "control_period_s");
    if (physics.drive_mode && *physics.drive_mode != "articulated_effort" &&
        *physics.drive_mode != "crank_effort" && *physics.drive_mode != "coast" &&
        *physics.drive_mode != "ideal_speed_control") fail("drive_mode", "unknown native drive dispatch");
    // RiderSpecs is owned by BuildRequest, so rider/drive/attachment combinations
    // are checked by runtime::implemented_capabilities at that actual boundary.
}

Value spatial_config_to_wire(const Config& config) {
    shape(config);
    if (config.mode != Mode::spatial3d) fail("config.spatial.mode", "planar has no spatial wire extension");
    const auto& t = config.terrain;
    const auto& s = config.steer;
    const auto& b = config.bootstrap;
    const auto& i = config.imu;
    const Value mount = std::holds_alternative<std::string>(i.mount_pos)
        ? text(std::get<std::string>(i.mount_pos)) : vector_value(std::get<Vec3>(i.mount_pos));
    return Value{Object{
        {"spatial_contract", integer(kSpatialContract)}, {"mode", text(to_string(config.mode))},
        {"root", text(to_string(config.root))},
        {"terrain", Value{Object{{"kind", text(terrain_kind(t.kind))}, {"path", text(t.path.generic_string())},
            {"corridor_half_width_m", real(t.corridor_half_width_m)}, {"max_asset_bytes", integer(t.max_asset_bytes)},
            {"dx_m", optional_real(t.dx_m)}, {"dy_m", optional_real(t.dy_m)}}}},
        {"steer", Value{Object{{"range_deg", real(s.range_deg)}, {"damping_nms_rad", real(s.damping_nms_rad)},
            {"torque_cap_nm", s.torque_cap_nm ? real(*s.torque_cap_nm) : text("strength")},
            {"reaction_delay_s", real(s.reaction_delay_s)},
            {"controller", text(s.controller == SteerController::off ? "off" : "roll_heading")},
            {"profile_id", text(s.profile_id)}, {"profile_sha256", text(s.profile_sha256)}}}},
        {"bootstrap", Value{Object{{"settle_time_s", real(b.settle_time_s)},
            {"release_force_limit_n", real(b.release_force_limit_n)}, {"release_moment_limit_nm", real(b.release_moment_limit_nm)},
            {"hold_stiffness_n_per_m", real(b.hold_stiffness_n_per_m)}, {"hold_damping_ns_per_m", real(b.hold_damping_ns_per_m)},
            {"hold_rot_stiffness_nm_per_rad", real(b.hold_rot_stiffness_nm_per_rad)},
            {"hold_rot_damping_nms_per_rad", real(b.hold_rot_damping_nms_per_rad)}}}},
        {"imu", Value{Object{{"mount_pos_m", mount}, {"mount_rpy_rad", vector_value(i.mount_rpy_rad)},
            {"axis_misalignment_rad", vector_value(i.axis_misalignment_rad)}, {"calibration_revision", text(i.calibration_revision)}}}},
        {"profile", Value{Object{{"corridor_half_width_m", real(config.corridor_half_width_m)},
            {"capsize_roll_rad", real(config.capsize_roll_rad)}, {"tyre_tier", text(to_string(config.tyre.tier))},
            {"lateral_coefficients_id", text(config.tyre.lateral_coefficients_id)},
            {"lateral_coefficients_sha256", text(config.tyre.lateral_coefficients_sha256)}}}}
    }};
}

Config spatial_config_from_wire(const Value& value) {
    constexpr std::string_view path = "config.spatial";
    const auto& object = table(value, path, {"spatial_contract", "mode", "root", "terrain", "steer", "bootstrap", "imu", "profile"}, true);
    if (unsigned_integer(required(object, "spatial_contract", path), "config.spatial.spatial_contract") != kSpatialContract)
        fail("config.spatial.spatial_contract", "unsupported spatial contract");
    Config out;
    const auto mode = parse_mode(string(required(object, "mode", path), "config.spatial.mode"));
    if (!mode || *mode != Mode::spatial3d) fail("config.spatial.mode", "expected '3d'");
    out.mode = *mode;
    const auto root = parse_root_kind(string(required(object, "root", path), "config.spatial.root"));
    if (!root || *root == RootKind::planar3) fail("config.spatial.root", "expected 'constrained' or 'free'");
    out.root = *root;
    terrain(required(object, "terrain", path), out.terrain, "config.spatial.terrain", true);
    steer(required(object, "steer", path), out.steer, "config.spatial.steer", true);
    bootstrap(required(object, "bootstrap", path), out.bootstrap, "config.spatial.bootstrap", true);
    imu(required(object, "imu", path), out.imu, "config.spatial.imu", true);
    const auto& profile = table(required(object, "profile", path), "config.spatial.profile",
        {"corridor_half_width_m", "capsize_roll_rad", "tyre_tier", "lateral_coefficients_id", "lateral_coefficients_sha256"}, true);
    out.corridor_half_width_m = finite(required(profile, "corridor_half_width_m", "config.spatial.profile"), "config.spatial.profile.corridor_half_width_m");
    out.capsize_roll_rad = finite(required(profile, "capsize_roll_rad", "config.spatial.profile"), "config.spatial.profile.capsize_roll_rad");
    const auto tier = parse_tyre_tier(string(required(profile, "tyre_tier", "config.spatial.profile"), "config.spatial.profile.tyre_tier"));
    if (!tier) fail("config.spatial.profile.tyre_tier", "expected 'fast' or 'detailed'");
    out.tyre.tier = *tier;
    out.tyre.lateral_coefficients_id = string(required(profile, "lateral_coefficients_id", "config.spatial.profile"), "config.spatial.profile.lateral_coefficients_id");
    out.tyre.lateral_coefficients_sha256 = string(required(profile, "lateral_coefficients_sha256", "config.spatial.profile"), "config.spatial.profile.lateral_coefficients_sha256");
    // Reuse exactly the same domains, but report the wire path at this boundary.
    try { shape(out); }
    catch (const std::invalid_argument& error) {
        std::string message = error.what();
        // Validation is shared, but diagnostics at a wire boundary use wire
        // field paths rather than the public TOML aliases.
        for (const auto& [source, target] : {
            std::pair<std::string_view, std::string_view>{"spatial_3d.corridor_half_width_m", "config.spatial.profile.corridor_half_width_m"},
            {"spatial_3d.capsize_roll_rad", "config.spatial.profile.capsize_roll_rad"},
            {"tires.pneumatic_3d.lateral_coefficients_sha256", "config.spatial.profile.lateral_coefficients_sha256"},
            {"tires.pneumatic_3d.lateral_coefficients", "config.spatial.profile.lateral_coefficients_id"},
            {"tires.pneumatic_3d.tier", "config.spatial.profile.tyre_tier"},
            {"spatial_3d.", "config.spatial."}}) {
            if (message.starts_with(source)) {
                message.replace(0, source.size(), target);
                throw std::invalid_argument(message);
            }
        }
        fail(path, message);
    }
    return out;
}

CapabilityReport check_spatial_capability(const Config& config, const bike::configuration::SimulationPhysicsConfig& physics) {
    validate_spatial_config(config, physics);
    CapabilityReport report;
    if (config.mode == Mode::planar) {
        report.implemented = true; // no extension; the existing runtime checks the planar row
        return report;
    }
    report.implemented=report.rejections.empty();
    report.qualification_pending=true;
    return report;
}

std::string spatial_config_fingerprint_hex(const Config& config) {
    const auto canonical = bike::io::canonical_json(spatial_config_to_wire(config));
    return bike::io::sha256(std::as_bytes(std::span{canonical.data(), canonical.size()}));
}

} // namespace bike::spatial
