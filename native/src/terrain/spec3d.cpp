#include "terrain/spec3d.hpp"
#include "terrain/detail.hpp"
#include "terrain/trackfile.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <string_view>
#include <utility>

namespace bike::terrain {
namespace {
using bike::io::Value;

const Value::Object& keys(const Value& value, std::string_view path,
                         std::initializer_list<std::string_view> allowed) {
    const auto& object = bike::io::as_object(value, path);
    std::vector<std::string_view> seen;
    for (const auto& [key, unused] : object) {
        (void)unused;
        if (std::ranges::find(allowed, key) == allowed.end())
            throw std::invalid_argument(std::string(path) + "." + key + ": unknown key");
        if (std::ranges::find(seen, key) != seen.end())
            throw std::invalid_argument(std::string(path) + "." + key + ": duplicate key");
        seen.push_back(key);
    }
    return object;
}
const Value& required(const Value::Object& object, std::string_view key,
                      std::string_view path) {
    const auto* result = bike::io::find_member(object, key);
    if (!result) throw std::invalid_argument(std::string(path) + "." + std::string(key) + ": required");
    return *result;
}
double real(const Value::Object& object, std::string_view key, std::string_view path) {
    return bike::io::as_double(required(object, key, path), std::string(path) + "." + std::string(key));
}
std::uint32_t id(const Value::Object& object, std::string_view key, std::string_view path) {
    const auto value = bike::io::as_int64(required(object, key, path), std::string(path) + "." + std::string(key));
    if (value < 0 || std::cmp_greater(value, std::numeric_limits<std::uint32_t>::max()))
        throw std::invalid_argument(std::string(path) + "." + std::string(key) + ": outside uint32 range");
    return static_cast<std::uint32_t>(value);
}
void text_domain(const std::string& value, std::string_view path) {
    if (value.empty() || value.find('\0') != std::string::npos)
        throw std::invalid_argument(std::string(path) + ": expected nonempty text without NUL");
}
void validate(const Track3dSpec& spec) {
    detail::positive(spec.dx_m, "terrain3d.dx_m");
    detail::positive(spec.dy_m, "terrain3d.dy_m");
    detail::positive(spec.half_width_m, "terrain3d.half_width_m");
    detail::nonnegative(spec.lead_in_m, "terrain3d.lead_in_m");
    detail::nonnegative(spec.lead_out_m, "terrain3d.lead_out_m");
    detail::nonnegative(spec.roughness.sigma_m, "terrain3d.roughness.sigma_m");
    detail::positive(spec.roughness.ell_m, "terrain3d.roughness.ell_m");
    (void)detail::seed_plus(spec.seed_decimal, 0);
    if (spec.generator_version != "grid3d-gen/1")
        throw std::invalid_argument("terrain3d.generator_version: unsupported generator");
    const auto base = track_from_dict(spec.base_track_spec);
    if (!std::isfinite(spec.lead_in_m + spec.lead_out_m + 1.) ||
        base.length_m < spec.lead_in_m + spec.lead_out_m + 1.)
        throw std::invalid_argument("terrain3d.base_track.length_m: shorter than lead_in + lead_out + 1 m");
    for (std::size_t i = 0; i < spec.banks.size(); ++i) {
        const auto& bank = spec.banks[i];
        const auto path = "terrain3d.banks[" + std::to_string(i) + "]";
        detail::nonnegative(bank.start_x_m, path + ".start_x_m");
        detail::positive(bank.ramp_m, path + ".ramp_m");
        detail::finite(bank.angle_deg, path + ".angle_deg");
        if (std::abs(bank.angle_deg) > 15.) throw std::invalid_argument(path + ".angle_deg: exceeds 15 degrees");
        if (bank.start_x_m > base.length_m) throw std::invalid_argument(path + ".start_x_m: outside track");
    }
    for (std::size_t i = 0; i < spec.features.size(); ++i) {
        const auto& feature = spec.features[i];
        const auto path = "terrain3d.features[" + std::to_string(i) + "]";
        if (feature.kind != "rock" && feature.kind != "rut" && feature.kind != "root")
            throw std::invalid_argument(path + ".kind: expected rock, rut or root");
        detail::finite(feature.x_m, path + ".x_m"); detail::finite(feature.y_m, path + ".y_m");
        detail::positive(feature.radius_x_m, path + ".radius_x_m");
        detail::positive(feature.radius_y_m, path + ".radius_y_m");
        detail::positive(feature.amplitude_m, path + ".amplitude_m");
        detail::finite(feature.yaw_rad, path + ".yaw_rad");
        if (feature.x_m < 0. || feature.x_m > base.length_m || std::abs(feature.y_m) > spec.half_width_m)
            throw std::invalid_argument(path + ": centre outside physical domain");
        if ((feature.kind == "rock" && (feature.amplitude_m < .04 || feature.amplitude_m > .15)) ||
            (feature.kind == "rut" && (feature.amplitude_m < .02 || feature.amplitude_m > .06)))
            throw std::invalid_argument(path + ".amplitude_m: outside author range (positive magnitude)");
    }
    if (spec.surfaces.empty()) throw std::invalid_argument("terrain3d.surfaces: nonempty table required");
    for (std::size_t i = 0; i < spec.surfaces.size(); ++i) {
        const auto& surface = spec.surfaces[i];
        const auto path = "terrain3d.surfaces[" + std::to_string(i) + "]";
        text_domain(surface.name, path + ".name");
        detail::nonnegative(surface.mu_peak, path + ".mu_peak");
        detail::nonnegative(surface.mu_slide, path + ".mu_slide");
        detail::nonnegative(surface.slip_stiffness_per_load, path + ".slip_stiffness_per_load");
        detail::positive(surface.stribeck_speed_mps, path + ".stribeck_speed_mps");
        if (surface.description.find('\0') != std::string::npos) throw std::invalid_argument(path + ".description: NUL");
        for (std::size_t j = 0; j < i; ++j) if (spec.surfaces[j].id == surface.id)
            throw std::invalid_argument(path + ".id: duplicate surface ID");
    }
    for (std::size_t i = 0; i < spec.zones.size(); ++i) {
        const auto& zone = spec.zones[i];
        const auto path = "terrain3d.zones[" + std::to_string(i) + "]";
        detail::finite(zone.x0_m, path + ".x0_m"); detail::finite(zone.x1_m, path + ".x1_m");
        detail::finite(zone.y0_m, path + ".y0_m"); detail::finite(zone.y1_m, path + ".y1_m");
        if (!(0. <= zone.x0_m && zone.x0_m < zone.x1_m && zone.x1_m <= base.length_m &&
              -spec.half_width_m <= zone.y0_m && zone.y0_m < zone.y1_m && zone.y1_m <= spec.half_width_m))
            throw std::invalid_argument(path + ": invalid bounds or outside domain");
        if (std::ranges::none_of(spec.surfaces, [&](const auto& surface) { return surface.id == zone.surface_id; }))
            throw std::invalid_argument(path + ".surface_id: unknown surface ID");
    }
}
}

Track3dSpec read_track3d_spec(const Value& document) {
    const auto& root = keys(document, "terrain3d.document", {"base_track", "terrain3d"});
    Track3dSpec out;
    out.base_track_spec = required(root, "base_track", "terrain3d.document");
    if (std::holds_alternative<std::string>(out.base_track_spec.data))
        throw std::invalid_argument("terrain3d.base_track: unresolved path; use load_track3d_spec at the file boundary");
    const auto& table = keys(required(root, "terrain3d", "terrain3d.document"), "terrain3d",
        {"seed", "dx_m", "dy_m", "half_width_m", "lead_in_m", "lead_out_m", "generator_version",
         "banks", "roughness", "features", "surfaces", "zones"});
    out.seed_decimal = "0";
    out.roughness.ell_m = .1;
    if (const auto* seed = bike::io::find_member(table, "seed"))
        out.seed_decimal = detail::seed_plus(bike::io::as_number(*seed, "terrain3d.seed").lexeme, 0);
    for (const auto key : {"dx_m", "dy_m", "half_width_m", "lead_in_m", "lead_out_m"}) {
        const auto* value = bike::io::find_member(table, key);
        if (!value) continue;
        const double number = bike::io::as_double(*value, std::string("terrain3d.") + key);
        if (key == std::string_view("dx_m")) out.dx_m = number;
        else if (key == std::string_view("dy_m")) out.dy_m = number;
        else if (key == std::string_view("half_width_m")) out.half_width_m = number;
        else if (key == std::string_view("lead_in_m")) out.lead_in_m = number;
        else out.lead_out_m = number;
    }
    if (const auto* version = bike::io::find_member(table, "generator_version"))
        out.generator_version = bike::io::as_string(*version, "terrain3d.generator_version");
    if (const auto* roughness = bike::io::find_member(table, "roughness")) {
        const auto& object = keys(*roughness, "terrain3d.roughness", {"sigma_m", "ell_m"});
        out.roughness = {.sigma_m = real(object, "sigma_m", "terrain3d.roughness"),
                         .ell_m = real(object, "ell_m", "terrain3d.roughness")};
    }
    const auto entries = [&](std::string_view key) -> const Value::Array& {
        static const Value::Array empty;
        const auto* value = bike::io::find_member(table, key);
        return value ? bike::io::as_array(*value, "terrain3d." + std::string(key)) : empty;
    };
    for (const auto& entry : entries("banks")) {
        const auto path = "terrain3d.banks[" + std::to_string(out.banks.size()) + "]";
        const auto& object = keys(entry, path, {"start_x_m", "ramp_m", "angle_deg"});
        out.banks.push_back({.start_x_m = real(object, "start_x_m", path), .ramp_m = real(object, "ramp_m", path),
                             .angle_deg = real(object, "angle_deg", path)});
    }
    for (const auto& entry : entries("features")) {
        const auto path = "terrain3d.features[" + std::to_string(out.features.size()) + "]";
        const auto& object = keys(entry, path, {"kind", "x_m", "y_m", "amplitude_m", "radius_x_m", "radius_y_m", "yaw_rad"});
        out.features.push_back({.kind = bike::io::as_string(required(object, "kind", path), path + ".kind"),
            .x_m = real(object, "x_m", path), .y_m = real(object, "y_m", path),
            .amplitude_m = real(object, "amplitude_m", path), .radius_x_m = real(object, "radius_x_m", path),
            .radius_y_m = real(object, "radius_y_m", path), .yaw_rad = real(object, "yaw_rad", path)});
    }
    for (const auto& entry : entries("surfaces")) {
        const auto path = "terrain3d.surfaces[" + std::to_string(out.surfaces.size()) + "]";
        const auto& object = keys(entry, path, {"id", "name", "mu_peak", "mu_slide", "slip_stiffness_per_load", "stribeck_speed_mps", "description"});
        SurfaceSpec3d surface{.id = id(object, "id", path),
            .name = bike::io::as_string(required(object, "name", path), path + ".name"),
            .mu_peak = real(object, "mu_peak", path), .mu_slide = real(object, "mu_slide", path),
            .slip_stiffness_per_load = real(object, "slip_stiffness_per_load", path),
            .stribeck_speed_mps = real(object, "stribeck_speed_mps", path), .description = {}};
        if (const auto* description = bike::io::find_member(object, "description")) surface.description = bike::io::as_string(*description, path + ".description");
        out.surfaces.push_back(std::move(surface));
    }
    for (const auto& entry : entries("zones")) {
        const auto path = "terrain3d.zones[" + std::to_string(out.zones.size()) + "]";
        const auto& object = keys(entry, path, {"surface_id", "x0_m", "x1_m", "y0_m", "y1_m"});
        out.zones.push_back({.surface_id = id(object, "surface_id", path), .x0_m = real(object, "x0_m", path),
                             .x1_m = real(object, "x1_m", path), .y0_m = real(object, "y0_m", path),
                             .y1_m = real(object, "y1_m", path)});
    }
    validate(out);
    return out;
}

Value write_track3d_spec(const Track3dSpec& spec) {
    validate(spec);
    Value::Array banks, features, surfaces, zones;
    for (const auto& bank : spec.banks) banks.push_back({Value::Object{
        {"start_x_m", detail::number(bank.start_x_m)}, {"ramp_m", detail::number(bank.ramp_m)}, {"angle_deg", detail::number(bank.angle_deg)}}});
    for (const auto& feature : spec.features) features.push_back({Value::Object{
        {"kind", Value{feature.kind}}, {"x_m", detail::number(feature.x_m)}, {"y_m", detail::number(feature.y_m)},
        {"amplitude_m", detail::number(feature.amplitude_m)}, {"radius_x_m", detail::number(feature.radius_x_m)},
        {"radius_y_m", detail::number(feature.radius_y_m)}, {"yaw_rad", detail::number(feature.yaw_rad)}}});
    for (const auto& surface : spec.surfaces) surfaces.push_back({Value::Object{
        {"id", detail::integer(surface.id)}, {"name", Value{surface.name}}, {"mu_peak", detail::number(surface.mu_peak)},
        {"mu_slide", detail::number(surface.mu_slide)}, {"slip_stiffness_per_load", detail::number(surface.slip_stiffness_per_load)},
        {"stribeck_speed_mps", detail::number(surface.stribeck_speed_mps)}, {"description", Value{surface.description}}}});
    for (const auto& zone : spec.zones) zones.push_back({Value::Object{
        {"surface_id", detail::integer(zone.surface_id)}, {"x0_m", detail::number(zone.x0_m)}, {"x1_m", detail::number(zone.x1_m)},
        {"y0_m", detail::number(zone.y0_m)}, {"y1_m", detail::number(zone.y1_m)}}});
    return {Value::Object{{"base_track", spec.base_track_spec}, {"terrain3d", Value{Value::Object{
        {"seed", detail::seed_value(spec.seed_decimal)}, {"dx_m", detail::number(spec.dx_m)}, {"dy_m", detail::number(spec.dy_m)},
        {"half_width_m", detail::number(spec.half_width_m)}, {"lead_in_m", detail::number(spec.lead_in_m)},
        {"lead_out_m", detail::number(spec.lead_out_m)}, {"generator_version", Value{spec.generator_version}},
        {"banks", Value{std::move(banks)}}, {"roughness", Value{Value::Object{
            {"sigma_m", detail::number(spec.roughness.sigma_m)}, {"ell_m", detail::number(spec.roughness.ell_m)}}}},
        {"features", Value{std::move(features)}}, {"surfaces", Value{std::move(surfaces)}}, {"zones", Value{std::move(zones)}}}}}}};
}
} // namespace bike::terrain
