#include "support/fixture_test.hpp"
#include "terrain/source3d.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace {

// NOLINTBEGIN(bugprone-easily-swappable-parameters) (x, y, eps, tol) test helper
void check_finite_diff_normal(const bike::terrain::Grid3d& grid, double x, double y,
                              double eps, double tol, const char* label) {
// NOLINTEND(bugprone-easily-swappable-parameters)
    using namespace bike::terrain;
    const TerrainSample3d s = grid.sample(x, y);
    bt::require(s.status == QueryStatus3d::ok, label);
    const TerrainSample3d sx_p = grid.sample(x + eps, y);
    const TerrainSample3d sx_m = grid.sample(x - eps, y);
    const TerrainSample3d sy_p = grid.sample(x, y + eps);
    const TerrainSample3d sy_m = grid.sample(x, y - eps);
    bt::require(sx_p.status == QueryStatus3d::ok && sx_m.status == QueryStatus3d::ok &&
                    sy_p.status == QueryStatus3d::ok && sy_m.status == QueryStatus3d::ok,
                "finite-difference stencil inside bounds");

    const double fd_dz_dx = (sx_p.height_m - sx_m.height_m) / (2. * eps);
    const double fd_dz_dy = (sy_p.height_m - sy_m.height_m) / (2. * eps);
    const double n_len = std::sqrt(s.normal_world[0] * s.normal_world[0] +
                                   s.normal_world[1] * s.normal_world[1] +
                                   s.normal_world[2] * s.normal_world[2]);
    bt::require(std::abs(n_len - 1.) <= 1e-13, "normal_world is unit length");
    bt::require(s.normal_world[2] > 0., "normal_world points upward");

    const double s_dz_dx = -s.normal_world[0] / s.normal_world[2];
    const double s_dz_dy = -s.normal_world[1] / s.normal_world[2];
    bt::require(std::abs(s_dz_dx - fd_dz_dx) <= tol, "dz/dx matches centered finite difference");
    bt::require(std::abs(s_dz_dy - fd_dz_dy) <= tol, "dz/dy matches centered finite difference");
}

} // namespace

int main() {
    return bt::run([] {
        using namespace bike::terrain;

        // 0. Invalid default-constructed Terrain3dQueryView fails closed.
        const Terrain3dQueryView empty_view{};
        bt::require(!empty_view.valid(), "default view is invalid");
        bt::require(empty_view.sha256_hex().empty(), "default view digest empty");
        bt::require(empty_view.sample(0., 0.).status == QueryStatus3d::invalid_input,
                    "default view sample returns invalid_input");
        bt::require(empty_view.closest_point({0., 0., 0.}, 0.1).status == QueryStatus3d::invalid_input,
                    "default view closest_point returns invalid_input");

        // 1. Exact reproduction of tilted plane z = 0.18 * x - 0.09 * y + 0.5 and layout ix * ny + iy.
        const Grid3dSpec plane_spec{.nx = 41, .ny = 21, .x0_m = 0., .y0_m = -1., .dx_m = 0.05, .dy_m = 0.1};
        std::vector<double> plane_z(plane_spec.nx * plane_spec.ny);
        for (std::size_t ix = 0; ix < plane_spec.nx; ++ix) {
            const double x = plane_spec.x0_m + static_cast<double>(ix) * plane_spec.dx_m;
            for (std::size_t iy = 0; iy < plane_spec.ny; ++iy) {
                const double y = plane_spec.y0_m + static_cast<double>(iy) * plane_spec.dy_m;
                plane_z[ix * plane_spec.ny + iy] = 0.18 * x - 0.09 * y + 0.5;
            }
        }
        std::vector<rider::SurfaceLabel> plane_surf((plane_spec.nx - 1) * (plane_spec.ny - 1),
                                                    rider::SurfaceLabel::scree);
        const Terrain3dSource plane_src(Grid3d(plane_spec, std::move(plane_z), std::move(plane_surf)));
        const Terrain3dQueryView view = plane_src.view();
        bt::require(view.valid(), "source view is valid");
        bt::require(view.sha256_hex().size() == 64, "64-hex SHA-256 digest");

        const double inv_norm = 1. / std::sqrt(1. + 0.18 * 0.18 + 0.09 * 0.09);
        for (const double x : {0.173, 0.85, 1.42}) {
            for (const double y : {-0.73, 0.0, 0.64}) {
                const TerrainSample3d s = view.sample(x, y);
                bt::require(s.status == QueryStatus3d::ok, "plane sample ok");
                bt::require(std::abs(s.height_m - (0.18 * x - 0.09 * y + 0.5)) <= 1e-13,
                            "Steffen bicubic reproduces tilted plane to machine precision");
                bt::require(std::abs(s.normal_world[0] - (-0.18 * inv_norm)) <= 1e-12 &&
                                std::abs(s.normal_world[1] - (0.09 * inv_norm)) <= 1e-12 &&
                                std::abs(s.normal_world[2] - inv_norm) <= 1e-12,
                            "plane normal exact");
                bt::require(s.surface == rider::SurfaceLabel::scree, "surface preserved");
            }
        }

        // Nonfinite and out-of-bounds queries fail closed.
        const double nan = std::numeric_limits<double>::quiet_NaN();
        bt::require(view.sample(nan, 0.).status == QueryStatus3d::invalid_input, "NaN x rejected");
        bt::require(view.sample(-0.01, 0.).status == QueryStatus3d::outside, "x < x_min outside");
        bt::require(view.sample(0.5, 1.01).status == QueryStatus3d::outside, "y > y_max outside");
        bt::require(view.closest_point({0.5, 0., 0.5}, 0.).status == QueryStatus3d::invalid_input,
                    "zero search radius rejected");
        bt::require(view.closest_point({-0.1, 0., 0.5}, 0.2).status == QueryStatus3d::outside,
                    "outside center rejected");

        // 2. Normal consistency with centered finite differences on smooth curved surface & across seams.
        const Grid3dSpec curve_spec{.nx = 51, .ny = 31, .x0_m = -0.5, .y0_m = -0.75, .dx_m = 0.04, .dy_m = 0.05};
        std::vector<double> curve_z(curve_spec.nx * curve_spec.ny);
        for (std::size_t ix = 0; ix < curve_spec.nx; ++ix) {
            const double x = curve_spec.x0_m + static_cast<double>(ix) * curve_spec.dx_m;
            for (std::size_t iy = 0; iy < curve_spec.ny; ++iy) {
                const double y = curve_spec.y0_m + static_cast<double>(iy) * curve_spec.dy_m;
                curve_z[ix * curve_spec.ny + iy] =
                    0.08 * std::sin(2.5 * x) * std::cos(1.8 * y) + 0.05 * x * y - 0.12 * y * y + 0.2;
            }
        }
        std::vector<rider::SurfaceLabel> curve_surf((curve_spec.nx - 1) * (curve_spec.ny - 1),
                                                    rider::SurfaceLabel::hardpack);
        const Grid3d curve_grid(curve_spec, std::move(curve_z), std::move(curve_surf));
        bt::require(curve_grid.sha256_hex() != view.sha256_hex(), "distinct grids have distinct SHA-256");

        // Interior points, X-seams (x = -0.5 + 15*0.04 = 0.10), Y-seams (y = -0.75 + 15*0.05 = 0.00),
        // and cell corners.
        for (const auto& [qx, qy] : std::array<std::pair<double, double>, 5>{{
                 {0.117, 0.023},
                 {0.10, 0.023},
                 {0.117, 0.00},
                 {0.10, 0.00},
                 {0.73, -0.31},
             }}) {
            check_finite_diff_normal(curve_grid, qx, qy, 1e-6, 1e-6, "curved surface finite-diff normal");
        }
        // C^1 normal continuity across both X-seam and Y-seam on curved surface.
        const TerrainSample3d cx_l = curve_grid.sample(0.10 - 1e-8, 0.023);
        const TerrainSample3d cx_r = curve_grid.sample(0.10 + 1e-8, 0.023);
        const TerrainSample3d cy_l = curve_grid.sample(0.117, 0.00 - 1e-8);
        const TerrainSample3d cy_r = curve_grid.sample(0.117, 0.00 + 1e-8);
        for (std::size_t k = 0; k < 3; ++k) {
            bt::require(std::abs(cx_l.normal_world[k] - cx_r.normal_world[k]) <= 1e-6,
                        "curved C^1 normal across X-seam");
            bt::require(std::abs(cy_l.normal_world[k] - cy_r.normal_world[k]) <= 1e-6,
                        "curved C^1 normal across Y-seam");
        }

        // 3. Regression against 3d-new Gibbs overshoot on 100 mm SquareEdge at dx = 5 mm AND 140 mm Pothole!
        const Grid3dSpec step_spec{.nx = 201, .ny = 11, .x0_m = 0., .y0_m = -0.1, .dx_m = 0.005, .dy_m = 0.02};
        std::vector<double> step_z(step_spec.nx * step_spec.ny, 0.0);
        for (std::size_t ix = 0; ix < step_spec.nx; ++ix) {
            const double x = static_cast<double>(ix) * step_spec.dx_m;
            double z = 0.0;
            if (x >= 0.30 && x <= 0.55) {
                z = 0.100; // 100 mm SquareEdge (slope = 20.0 >> tan(60 deg)!)
            } else if (x >= 0.70 && x <= 0.85) {
                z = -0.140; // 140 mm sharp Pothole
            }
            for (std::size_t iy = 0; iy < step_spec.ny; ++iy) {
                step_z[ix * step_spec.ny + iy] = z;
            }
        }
        std::vector<rider::SurfaceLabel> step_surf((step_spec.nx - 1) * (step_spec.ny - 1),
                                                   rider::SurfaceLabel::asphalt);
        const Grid3d step_grid(step_spec, std::move(step_z), std::move(step_surf));

        // Sample every 0.5 mm across both plateaus and transitions: ZERO overshoot/undershoot!
        for (int k = 0; k <= 2000; ++k) {
            const double x = 0.0005 * static_cast<double>(k);
            const TerrainSample3d s = step_grid.sample(x, 0.0);
            bt::require(s.status == QueryStatus3d::ok, "step sample ok");
            if (x <= 0.60) {
                bt::require(s.height_m >= -1e-15 && s.height_m <= 0.100 + 1e-15,
                            "zero Gibbs pit/lip around 100 mm SquareEdge");
            } else {
                bt::require(s.height_m >= -0.140 - 1e-15 && s.height_m <= 1e-15,
                            "zero Gibbs overshoot around 140 mm Pothole");
            }
        }

        // 4. C^1 seam continuity across cell boundary and frontal -Fx normal on 100 mm SquareEdge!
        const TerrainSample3d left_seam = step_grid.sample(0.2975 - 1e-9, 0.01);
        const TerrainSample3d right_seam = step_grid.sample(0.2975 + 1e-9, 0.01);
        bt::require(std::abs(left_seam.normal_world[0] - right_seam.normal_world[0]) <= 1e-6,
                    "continuous C^1 normal across cell boundary");

        // Torus station at (x = 0.275 m, z = 0.055 m) approaching the 100 mm step at x = 0.300 m:
        // must see the step face with negative normal_x (frontal impact resistance), never origin_below_surface!
        const TorusClosestPoint3d hit = step_grid.closest_point({0.275, 0.0, 0.055}, 0.06);
        bt::require(hit.status == QueryStatus3d::ok, "closest_point near 100 mm step succeeds");
        bt::require(hit.signed_distance_m > 0. && hit.signed_distance_m < 0.035 &&
                        hit.normal_world[0] < -0.5,
                    "torus station sees frontal -X normal of 100 mm SquareEdge");

        // Below-surface query returns negative signed_distance_m and upward normal.
        const TorusClosestPoint3d below = step_grid.closest_point({0.10, 0.0, -0.02}, 0.06);
        bt::require(below.status == QueryStatus3d::ok, "below-surface closest_point ok");
        bt::require(below.signed_distance_m < 0. && std::abs(below.signed_distance_m + 0.02) <= 1e-12 &&
                        below.normal_world[2] > 0.99,
                    "below-surface query returns negative distance and upward normal");
    });
}
