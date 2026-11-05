#pragma once
// Reference tyres for the pneumatic tyre tests: the two catalogue tyres,
// the fast/detailed tiers and the world-grid road profile (ground plane +
// build_profile, before MuJoCo's binary32 heightfield quantization). Moved
// verbatim from the removed legacy tyre owner (2026-11-04); physical runs
// take their tyres from [tires.pneumatic] in the physics config instead.
#include "config_types.hpp"
#include "geometry/specs.hpp"
#include "pneumatic/model.hpp"
#include "terrain/profile.hpp"
#include "terrain/types.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace bt::tyres {
[[nodiscard]] inline nativecfg::PneumaticTyreSpec front_tyre() {
    return {.name="Schwalbe Magic Mary 29x2.4", .outer_radius_mm=372., .rim_radius_mm=320., .section_height_mm=57., .pressure_bar=1.5, .crr_reference=.011,
        .casing_width_mm=60., .compressed_casing_mm=6., .area_factor=.4435, .carcass_stiffness_n_mm2=.1194, .contact_length_factor=.8589, .rate_stiffening=.2484, .rate_relaxation_s=.2, .loss_factor=.0662, .hysteresis_rate_eps_mps=.01, .relaxation_length_mm=60., .rim_stiffness_n_mm2=28., .tread_loss_crr=.00586};
}
[[nodiscard]] inline nativecfg::PneumaticTyreSpec rear_tyre() {
    return {.name="Schwalbe Hans Dampf 27.5x2.4", .outer_radius_mm=352., .rim_radius_mm=300., .section_height_mm=55., .pressure_bar=1.7, .crr_reference=.0103,
        .casing_width_mm=60., .compressed_casing_mm=6., .area_factor=.4576, .carcass_stiffness_n_mm2=.1225, .contact_length_factor=.8801, .rate_stiffening=.2498, .rate_relaxation_s=.2, .loss_factor=.0690, .hysteresis_rate_eps_mps=.01, .relaxation_length_mm=60., .rim_stiffness_n_mm2=28., .tread_loss_crr=.00481};
}
struct TierSpec {
    std::string name;
    std::int64_t n_rays;
    double half_angle_deg, timestep_s;
    bool discretised_brush;
};
inline constexpr std::array<std::string_view, 2> kTiers{"fast", "detailed"};
[[nodiscard]] inline TierSpec tier_spec(std::string_view name) {
    if (name == "fast") return {.name="fast", .n_rays=64, .half_angle_deg=75., .timestep_s=.0005, .discretised_brush=false};
    if (name == "detailed") return {.name="detailed", .n_rays=256, .half_angle_deg=75., .timestep_s=.00025, .discretised_brush=true};
    throw std::invalid_argument("unknown reference tyre tier '" + std::string(name) + "'");
}
[[nodiscard]] inline biketyre::RoadProfile build_road_profile(const bike::terrain::TrackSpec& track,
        const bike::terrain::HeightFieldSpec& field, const bike::geometry::BikeSpecs& specs) {
    const auto x = field.track_x();
    auto z = bike::terrain::build_profile(track, x);
    const double ground = bike::geometry::compute_ground_z(specs) / 1000.;
    for (auto& height : z) height = ground + height;
    return {x.front(), x[1] - x[0], std::move(z)};
}
} // namespace bt::tyres
