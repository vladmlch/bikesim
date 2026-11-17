#include "support/fixture_test.hpp"
#include "terrain/generator3d.hpp"
#include "terrain/profile.hpp"
#include "terrain/trackfile.hpp"

#include <array>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <string_view>

int main() {
    return bt::run([] {
        using namespace bike::terrain;
        const std::filesystem::path worktree_root =
            bt::fixture("config").parent_path().parent_path().parent_path();

        // 1. Both tracks that failed under 3d-new (rough_uphill_savage 100 mm SquareEdge
        // and alpine_rocky scree + SquareEdge) generate cleanly with a 5-deg cross-slope
        // and preserve lead-in/lead-out flatness and surface sections.
        constexpr std::array<std::string_view, 2> kTracks{
            "examples/research/rough_uphill_savage.toml",
            "examples/research/alpine_rocky.toml",
        };
        for (const std::string_view rel : kTracks) {
            const TrackSpec track = load_track(worktree_root / rel);
            Terrain3dOverlaySpec ov{};
            ov.half_width_m = 1.5;
            ov.dx_m = 0.005;
            ov.dy_m = 0.020;
            ov.lead_in_m = 4.0;
            ov.lead_out_m = 4.0;
            ov.cross_slopes.push_back(CrossSlopeSegment3d{
                .start_m = 10.0,
                .end_m = 60.0,
                .transition_m = 3.0,
                .bank_rad = 5.0 * std::numbers::pi / 180.0,
            });

            const Grid3d grid = build_overlay_grid3d(track, ov);
            bt::require(grid.spec().ny == 151, "151 transverse nodes at dy=20 mm");

            // Lead-in zone [0, 4.0 m] and lead-out zone [length_m - 4.0, length_m] have exact zero
            // lateral slope everywhere across [-1.5, 1.5].
            const TerrainSample3d s_left = grid.sample(2.0, 1.2);
            const TerrainSample3d s_right = grid.sample(2.0, -1.2);
            bt::require(s_left.status == QueryStatus3d::ok && s_right.status == QueryStatus3d::ok,
                        "lead-in samples ok");
            bt::require(s_left.height_m == s_right.height_m && s_left.normal_world[1] == 0.0,
                        "lead-in halo keeps interpolated surface strictly flat laterally");

            const TerrainSample3d out_left = grid.sample(track.length_m - 1.0, 1.1);
            const TerrainSample3d out_right = grid.sample(track.length_m - 1.0, -0.7);
            bt::require(out_left.status == QueryStatus3d::ok && out_right.status == QueryStatus3d::ok,
                        "lead-out samples ok");
            bt::require(std::abs(out_left.height_m - out_right.height_m) <= 1e-15 &&
                            out_left.normal_world[1] == 0.0,
                        "lead-out halo keeps interpolated surface strictly flat laterally");

            // Inside full cross-slope at x = 30 m: dz/dy == tan(5 deg).
            const TerrainSample3d bank_pos = grid.sample(30.0, 0.5);
            const TerrainSample3d bank_neg = grid.sample(30.0, -0.5);
            const double expected_diff = std::tan(5.0 * std::numbers::pi / 180.0) * 1.0;
            bt::require(std::abs((bank_pos.height_m - bank_neg.height_m) - expected_diff) <= 1e-12,
                        "5-deg cross-slope elevation difference exact");

            // Surface sections of base_track (e.g. scree on alpine_rocky, asphalt on savage) preserved.
            const SurfaceMap smap = track.surface_map();
            constexpr std::array<double, 4> kSampleXs{1.0, 25.0, 55.0, 90.0};
            for (const double x : kSampleXs) {
                const auto expected = rider::surface_label(smap.at(x).name);
                bt::require(expected.has_value(), "expected valid surface label");
                bt::require(grid.sample(x, 0.4).surface == expected.value(),
                            "base_track surface sections preserved in 3D overlay");
            }
        }

        // 2. Mirror symmetry: flipping bank_rad -> -bank_rad mirrors y -> -y.
        const TrackSpec flat_track = flat(40.0);
        const Terrain3dOverlaySpec pos_ov{
            .half_width_m = 1.5,
            .dx_m = 0.01,
            .dy_m = 0.05,
            .lead_in_m = 4.0,
            .lead_out_m = 4.0,
            .cross_slopes = {{.start_m = 8., .end_m = 30., .transition_m = 2., .bank_rad = 0.1}},
        };
        Terrain3dOverlaySpec neg_ov = pos_ov;
        neg_ov.cross_slopes[0].bank_rad = -0.1;

        const Grid3d g_pos = build_overlay_grid3d(flat_track, pos_ov);
        const Grid3d g_neg = build_overlay_grid3d(flat_track, neg_ov);
        constexpr std::array<double, 4> kMirrorXs{6.0, 9.0, 18.5, 31.0};
        constexpr std::array<double, 5> kMirrorYs{-1.1, -0.35, 0.0, 0.72, 1.3};
        for (const double x : kMirrorXs) {
            for (const double y : kMirrorYs) {
                const TerrainSample3d a = g_pos.sample(x, y);
                const TerrainSample3d b = g_neg.sample(x, -y);
                bt::require(std::abs(a.height_m - b.height_m) <= 1e-15, "mirror height symmetry");
                bt::require(std::abs(a.normal_world[1] + b.normal_world[1]) <= 1e-15,
                            "mirror lateral normal antisymmetry");
            }
        }

        // 3. Deterministic seeded lateral roughness (using NumpyRng::normal01) stays zero in lead-in/out.
        const Terrain3dOverlaySpec rough_a{
            .half_width_m = 1.5,
            .dx_m = 0.02,
            .dy_m = 0.1,
            .lead_in_m = 4.0,
            .lead_out_m = 4.0,
            .lateral_roughness_amplitude_m = 0.015,
            .seed = "42",
        };
        Terrain3dOverlaySpec rough_b = rough_a;
        rough_b.seed = "43";
        const Grid3d gr_a1 = build_overlay_grid3d(flat_track, rough_a);
        const Grid3d gr_a2 = build_overlay_grid3d(flat_track, rough_a);
        const Grid3d gr_b = build_overlay_grid3d(flat_track, rough_b);
        bt::require(gr_a1.sha256_hex() == gr_a2.sha256_hex(),
                    "seeded lateral roughness is bitwise deterministic");
        bt::require(gr_a1.sha256_hex() != gr_b.sha256_hex(),
                    "different seed produces distinct lateral roughness");
        bt::require(gr_a1.sample(2.0, 0.8).height_m == 0.0 &&
                        gr_a1.sample(38.0, -0.8).height_m == 0.0,
                    "roughness is strictly zeroed in lead-in and lead-out zones");
        bt::require(gr_a1.sample(20.0, 0.5).height_m != 0.0,
                    "roughness applies in interior");

        // 4. Invalid overlay specs fail closed with std::invalid_argument.
        bt::invalid([&] {
            Terrain3dOverlaySpec bad = pos_ov;
            bad.half_width_m = 0.1;
            validate_overlay_spec(flat_track, bad);
        });
        bt::invalid([&] {
            Terrain3dOverlaySpec bad = pos_ov;
            bad.dy_m = 0.07; // 3.0 not a multiple of 0.07
            validate_overlay_spec(flat_track, bad);
        });
        bt::invalid([&] {
            Terrain3dOverlaySpec bad = pos_ov;
            bad.dy_m = 1.0; // ny = 4 < 5
            validate_overlay_spec(flat_track, bad);
        });
        bt::invalid([&] {
            Terrain3dOverlaySpec bad = pos_ov;
            bad.cross_slopes[0].bank_rad = std::numbers::pi / 4.0;
            validate_overlay_spec(flat_track, bad);
        });
        bt::invalid([&] {
            Terrain3dOverlaySpec bad = pos_ov;
            bad.cross_slopes[0].end_m = bad.cross_slopes[0].start_m;
            validate_overlay_spec(flat_track, bad);
        });
        bt::invalid([&] {
            Terrain3dOverlaySpec bad = pos_ov;
            bad.lateral_roughness_amplitude_m = -0.01;
            validate_overlay_spec(flat_track, bad);
        });
    });
}
