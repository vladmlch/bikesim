#include "support/fixture_test.hpp"
#include "terrain/extrusion3d.hpp"
#include "terrain/trackfile.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <vector>

int main() {
    return bt::run([] {
        using namespace bike::terrain;

        // Worktree root is three levels above <worktree>/tests/fixtures/config.
        const std::filesystem::path repo_root =
            bt::fixture("config").parent_path().parent_path().parent_path();
        const std::array<const char*, 5> track_files{
            "examples/research/rough_uphill.toml",
            "examples/research/rough_uphill_extreme.toml",
            "examples/research/rough_uphill_savage.toml",
            "examples/research/alpine_scree.toml",
            "examples/research/alpine_rocky.toml",
        };

        for (const char* rel : track_files) {
            const TrackSpec spec = load_track(repo_root / rel);
            const ExtrudedTrack3d ext(spec, 1.5, -0.402);
            bt::require(ext.sha256_hex().size() == 64U, "64-hex SHA-256 digest");

            // Check bitwise height parity with 1D biketyre::np_interp and Y-invariance across [-1.5, 1.5].
            const auto pts1d = profile(spec, DEFAULT_RESOLUTION_M);
            std::vector<double> xs(pts1d.size()), zs(pts1d.size());
            for (std::size_t i = 0; i < pts1d.size(); ++i) {
                xs[i] = pts1d[i][0];
                zs[i] = pts1d[i][1] - 0.402;
            }
            const SurfaceMap smap = spec.surface_map();

            for (const double x :
                 {0.0, 1.234, 12.5, 26.0, spec.length_m * 0.5, spec.length_m - 0.321}) {
                if (x > spec.length_m) {
                    continue;
                }
                const double expected_z = biketyre::np_interp(x, xs, zs);
                const auto expected_lbl = rider::surface_label(smap.at(x).name);
                bt::require(expected_lbl.has_value(), "known surface label");

                for (const double y : {-1.5, -0.37, 0.0, 0.89, 1.5}) {
                    const TerrainSample3d s = ext.sample(x, y);
                    bt::require(s.status == QueryStatus3d::ok, "sample inside corridor ok");
                    bt::require(s.height_m == expected_z, "bitwise height match with 1D np_interp");
                    bt::require(s.normal_world[1] == 0.0, "zero lateral normal on extruded track");
                    bt::require(s.surface == *expected_lbl,
                                "surface section preserved (including scree)");
                }
            }

            bt::require(ext.sample(-0.01, 0.0).status == QueryStatus3d::outside, "x < 0 outside");
            bt::require(ext.sample(ext.bounds().x_max_m + 0.01, 0.0).status == QueryStatus3d::outside,
                        "x > x_max outside");
            bt::require(ext.sample(1.0, 1.51).status == QueryStatus3d::outside,
                        "|y| > half_width outside");

            const double nan = std::numeric_limits<double>::quiet_NaN();
            bt::require(ext.sample(nan, 0.0).status == QueryStatus3d::invalid_input,
                        "NaN x rejected as invalid_input");
            bt::require(ext.closest_point({1.0, 0.0, 0.0}, 0.0).status == QueryStatus3d::invalid_input,
                        "nonpositive search_radius rejected");

            // Check closest_point parity with 1D SignedProfile on sharp obstacles (e.g. SquareEdge).
            std::vector<Point> verts(xs.size());
            for (std::size_t i = 0; i < xs.size(); ++i) {
                verts[i] = {xs[i], zs[i]};
            }
            const SignedProfile sp(std::move(verts));
            const Point probe_xz{spec.length_m * 0.4, zs[zs.size() * 2U / 5U] + 0.035};
            const std::array<Point, 1> probe_arr{probe_xz};
            const ProfileDistance d1 = sp.query(probe_arr);
            const TorusClosestPoint3d d3 = ext.closest_point({probe_xz[0], -0.42, probe_xz[1]}, 0.08);
            bt::require(d3.status == QueryStatus3d::ok, "closest_point ok");
            bt::require(std::abs(d3.signed_distance_m - d1.distance[0]) <= 1e-14,
                        "3D closest_point matches 1D SignedProfile distance");
            bt::require(std::abs(d3.normal_world[0] - d1.normal[0][0]) <= 1e-14 &&
                            d3.normal_world[1] == 0.0 &&
                            std::abs(d3.normal_world[2] - d1.normal[0][1]) <= 1e-14,
                        "3D closest_point normal matches 1D SignedProfile normal");
        }

        const TrackSpec flat_track = flat(20.0);
        bt::invalid([&] { (void)ExtrudedTrack3d(flat_track, 0.0); });
        bt::invalid([&] { (void)ExtrudedTrack3d(flat_track, 1.5, 0.0, 0.0); });
        bt::invalid([&] {
            (void)ExtrudedTrack3d(flat_track, 1.5, std::numeric_limits<double>::quiet_NaN());
        });
    });
}
