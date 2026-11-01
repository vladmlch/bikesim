// test_tyre_road_profile.cpp — C++ port of
// tests/reference/test_tyre_road_profile.py (batch 3.13).
//
// The shared build_road_profile helper is bitwise-locked to the inline
// construction it replaced (ground_z/1000 + build_profile on the
// HeightFieldSpec track_x raster), and the pneumatic patch slip is the
// signed tread slip v_x - omega * R_eff per patch — bitwise against the
// recorded oracle entries, never tolerance.
#include "tyre_reference_support.hpp"

#include "geometry/specs.hpp"
#include "legacy/tyre.hpp"
#include "pneumatic/model.hpp"
#include "pneumatic/profile.hpp"
#include "terrain/generators.hpp"
#include "terrain/profile.hpp"

#include <cmath>
#include <vector>

namespace {

using namespace tyre_reference;
constexpr std::string_view kFile = "test_tyre_road_profile.py";
namespace terrain = bike::terrain;

// SurfaceMap.uniform("asphalt") — the resolved registry spec, no sections.
[[nodiscard]] nativecfg::SurfaceMap uniform_asphalt() {
    const auto &registered = terrain::get_surface("asphalt");
    return {.surface = {.name = registered.name,
                        .mu_peak = registered.mu_peak,
                        .mu_slide = registered.mu_slide,
                        .slip_stiffness_per_load =
                                registered.slip_stiffness_per_load,
                        .stribeck_speed_mps =
                                registered.stribeck_speed_mps},
            .sections = {}};
}

// tests/reference/test_tyre_road_profile.py::test_build_road_profile_matches_the_former_inline_construction_bitwise
// Capture: expected.x0_m, expected.dx_m, expected.z_m — the oracle's
// inline-construction RoadProfile the helper must reproduce bitwise.
void test_build_road_profile_matches_the_former_inline_construction_bitwise() {
    auto golden = node(
            kFile,
            "test_build_road_profile_matches_the_former_inline_construction_bitwise");
    const auto track = terrain::get_preset(terrain::DEFAULT_PRESET);
    const auto field = terrain::HeightFieldSpec::for_track(track);
    const bike::geometry::BikeSpecs specs;

    const auto road = bike::legacy::build_road_profile(track, field, specs);
    scalar(golden, road.x0_m(), "expected x0_m");
    scalar(golden, road.dx_m(), "expected dx_m");
    vector(golden, road.z_m(), "expected z_m");

    // Port-side: the same inline construction the oracle ran before the
    // helper was extracted — track_x grid, ground_z/1000 + build_profile.
    const auto track_x = field.track_x();
    const auto profile_z = terrain::build_profile(track, track_x);
    const double ground_z = bike::geometry::compute_ground_z(specs) / 1000.;
    bt::require(road.size() == track_x.size(),
                "road profile length equals the field raster");
    for (std::size_t i = 0; i < road.size(); ++i) {
        bt::require(road.x_m()[i] == track_x[i],
                    "road x[" + std::to_string(i) +
                            "] is the field track_x grid");
        bt::require(road.z_m()[i] == ground_z + profile_z[i],
                    "road z[" + std::to_string(i) +
                            "] is bitwise ground_z + build_profile");
    }
    golden.done();
}

// tests/reference/test_tyre_road_profile.py::test_patch_slip_speed_mps_is_the_signed_tread_slip[v_x-omega-id]
// _loaded_outputs: flat 20 m road at 5 mm, hub dropped 6 mm below the
// outer radius, constant forward velocity — dt is the tier's own step.
// Capture per node: patch count, then expected_v_s per patch.
void test_patch_slip_speed_mps_is_the_signed_tread_slip(
        std::string_view parameters, std::string_view tier_name, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
        double v_x_mps, double omega_radps, double sign) { // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    auto golden = node(kFile,
                       "test_patch_slip_speed_mps_is_the_signed_tread_slip",
                       parameters);
    const auto tyre = bike::legacy::front_tyre();
    const auto tier = bike::legacy::tier_spec(tier_name);
    const double radius_m = tyre.outer_radius_mm / 1000.;
    // RoadProfile.from_samples(linspace(0, 20, 4001), zeros) — x0 0, dx 5 mm.
    biketyre::RoadProfile const road(0., .005, std::vector<double>(4001, 0.));
    biketyre::PneumaticTyreModel model(
            tyre, tier.n_rays, tier.half_angle_deg, tier.discretised_brush);
    const std::array<double, 3> hub{5., 0., radius_m - .006};
    const std::array<double, 3> velocity{v_x_mps, 0., 0.};
    const auto surface = uniform_asphalt();
    const auto &out = model.evaluate(hub, velocity, omega_radps, road,
                                     surface, tier.timestep_s);

    bt::require(!out.airborne, std::string(tier_name) + " is not airborne");
    bt::require(out.patch_count > 0,
                std::string(tier_name) + " produced patches");

    count(golden, static_cast<std::int64_t>(out.patch_count), "patch count");
    for (std::size_t i = 0; i < out.patch_count; ++i) {
        const auto &patch = out.patches()[i];
        scalar(golden, patch.slip_speed_mps,
               "expected_v_s patch " + std::to_string(i));
        // Port-side re-derivation of the oracle's analytic law:
        // v_s = v_x . tangent - omega * max(R - deflection/3, 1e-6).
        const double v_x = velocity[0] * patch.tangent_world[0] +
                velocity[1] * patch.tangent_world[1] +
                velocity[2] * patch.tangent_world[2];
        const double effective_radius_m = std::max(
                radius_m - patch.mean_deflection_m / 3., 1e-6);
        const double expected_v_s = v_x - omega_radps * effective_radius_m;
        bt::require(patch.slip_speed_mps == expected_v_s,
                    "patch " + std::to_string(i) +
                            " slip is the signed tread slip bitwise");
        bt::require(patch.slip_speed_mps * sign > 0.,
                    "patch " + std::to_string(i) + " slip sign");
        bt::require(patch.sliding_speed_mps == std::abs(patch.slip_speed_mps),
                    "patch " + std::to_string(i) +
                            " sliding speed is |slip|");
    }
    golden.done();
}

} // namespace

int main() {
    return bt::run([] {
        test_build_road_profile_matches_the_former_inline_construction_bitwise();
        // Parametrize order follows the pytest ids:
        // [v_x,omega case id]-[tier name] — forward-slide then
        // spinning-wheel, fast then detailed.
        test_patch_slip_speed_mps_is_the_signed_tread_slip(
                "forward-slide-fast", "fast", 5., 0., 1.);
        test_patch_slip_speed_mps_is_the_signed_tread_slip(
                "forward-slide-detailed", "detailed", 5., 0., 1.);
        test_patch_slip_speed_mps_is_the_signed_tread_slip(
                "spinning-wheel-fast", "fast", 2., 20., -1.);
        test_patch_slip_speed_mps_is_the_signed_tread_slip(
                "spinning-wheel-detailed", "detailed", 2., 20., -1.);
    });
}
