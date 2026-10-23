#include "support/migration_test.hpp"
#include "spatial/support/spatial_fixture.hpp"
#include "spatial/support/terrain_fixture.hpp"
#include "terrain/interpolation3d.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <numbers>

namespace {
using namespace bike::terrain;

void close(double a, double b, double tolerance, const char* message) {
    bt::require(std::isfinite(a) && std::abs(a - b) <= tolerance, message);
}

double polynomial(const std::array<double, 10>& c, double x, double y) {
    return c[0] + c[1] * x + c[2] * y + c[3] * x * x + c[4] * x * y + c[5] * y * y +
        c[6] * x * x * x + c[7] * x * x * y + c[8] * x * y * y + c[9] * y * y * y;
}
} // namespace

int main() {
    return bt::run([] {
        const auto grid = spatial_tests::plane_grid(.1, -.2, .3, 16, 12, .25, .5);
        const auto& d = grid.descriptor();
        const double normal_length = std::sqrt(1.05);
        const std::array<double, 3> normal{-.1 / normal_length, .2 / normal_length, 1. / normal_length};
        const auto check = [&](double x, double y) {
            const auto sample = grid.sample(x, y);
            bt::require(sample.status == QueryStatus3d::ok, "plane node/interior/closed boundary samples");
            close(sample.height_m, .1 * x - .2 * y + .3, 1e-12, "plane height within V02 tolerance");
            close(std::hypot(sample.normal_world[0], sample.normal_world[1], sample.normal_world[2]), 1., 1e-12, "normal is unit");
            for (std::size_t k = 0; k < 3; ++k) close(sample.normal_world[k], normal[k], 1e-12, "analytic normal component");
        };
        for (std::uint64_t iy = 0; iy < grid.ny(); ++iy)
            for (std::uint64_t ix = 0; ix < grid.nx(); ++ix)
                check(d.x0_m + static_cast<double>(ix) * d.dx_m, d.y0_m + static_cast<double>(iy) * d.dy_m);
        for (double x : {d.x0_m, .137, 1.671, grid.x_max_m()})
            for (double y : {d.y0_m, -.713, .411, grid.y_max_m()}) check(x, y);
        const double infinity = std::numeric_limits<double>::infinity();
        bt::require(grid.sample(std::nextafter(d.x0_m, -infinity), 0.).status == QueryStatus3d::outside, "left nextafter is outside");
        bt::require(grid.sample(std::nextafter(grid.x_max_m(), infinity), 0.).status == QueryStatus3d::outside, "right nextafter is outside");
        bt::require(grid.sample(1., std::nextafter(d.y0_m, -infinity)).status == QueryStatus3d::outside, "bottom nextafter is outside");
        bt::require(grid.sample(1., std::nextafter(grid.y_max_m(), infinity)).status == QueryStatus3d::outside, "top nextafter is outside");
        bt::require(grid.sample(std::numeric_limits<double>::quiet_NaN(), 0.).status == QueryStatus3d::invalid_input, "NaN query rejected");
        bt::require(grid.sample(0., infinity).status == QueryStatus3d::invalid_input, "infinite query rejected");
        for (std::uint64_t ix = 1; ix + 1 < grid.nx(); ++ix) {
            const double seam = d.x0_m + static_cast<double>(ix) * d.dx_m;
            const auto a = grid.sample(seam - 1e-9, .31), b = grid.sample(seam + 1e-9, .31);
            close(a.height_m, b.height_m, 1e-9, "height continuous across x seam");
            for (std::size_t k = 0; k < 3; ++k) close(a.normal_world[k], b.normal_world[k], 1e-9, "normal continuous across x seam");
        }
        for (std::uint64_t iy = 1; iy + 1 < grid.ny(); ++iy) {
            const double seam = d.y0_m + static_cast<double>(iy) * d.dy_m;
            const auto a = grid.sample(.71, seam - 1e-9), b = grid.sample(.71, seam + 1e-9);
            close(a.height_m, b.height_m, 1e-9, "height continuous across y seam");
            for (std::size_t k = 0; k < 3; ++k) close(a.normal_world[k], b.normal_world[k], 1e-9, "normal continuous across y seam");
        }
        const auto asym = spatial_tests::asymmetric_4x4();
        bt::require(asym.sample(std::nextafter(1., 0.), .5).surface_id == 1 && asym.sample(1., .5).surface_id == 7,
                    "material x boundary is half open");
        bt::require(asym.sample(.5, std::nextafter(1., 0.)).surface_id == 1 && asym.sample(.5, 1.).surface_id == 7,
                    "material y boundary is half open");
        bt::require(asym.sample(3., 3.).surface_id == 1, "outer corner belongs to last material cell");
        const std::array<double, 10> c{.3, .1, -.2, .03, .04, .02, .17, -.11, .09, .13};
        double previous_error = infinity;
        for (std::uint64_t n : {17U, 33U, 65U}) {
            const double h = 2. / static_cast<double>(n - 1);
            const auto p = spatial_tests::polynomial_grid(c, n, n, h, h);
            const double x = .5 + .37 * h, y = .25 + .29 * h;
            const double error = std::abs(p.sample(x, y).height_m - polynomial(c, x, y));
            bt::require(error < previous_error, "cubic sampling error decreases under h/2 refinement");
            previous_error = error;
        }
        const auto mirrored = spatial_tests::plane_grid(.1, .2, .3, 16, 12, .25, .5);
        for (double x : {.13, 1.43, 3.21}) {
            const auto a = grid.sample(x, .71), b = mirrored.sample(x, -.71);
            close(a.height_m, b.height_m, 1e-12, "mirror height covariance");
            close(a.normal_world[1], -b.normal_world[1], 1e-12, "mirror normal sign");
        }
        const double yaw = 37. * std::numbers::pi / 180.;
        const double a = .1 * std::cos(yaw) - (-.2) * std::sin(yaw);
        const double b = .1 * std::sin(yaw) + (-.2) * std::cos(yaw);
        const auto rotated = spatial_tests::plane_grid(a, b, .3, 16, 12, .25, .5).sample(1.17, -.43);
        close(rotated.normal_world[0], normal[0] * std::cos(yaw) - normal[1] * std::sin(yaw), 1e-12, "yaw rotation transforms normal x");
        close(rotated.normal_world[1], normal[0] * std::sin(yaw) + normal[1] * std::cos(yaw), 1e-12, "yaw rotation transforms normal y");
        close(Catmull::value(-1., 0., 1., 2., .4), .4, 1e-15, "Catmull affine value");
        close(Catmull::derivative(-1., 0., 1., 2., .4), 1., 1e-15, "Catmull analytic derivative");
    });
}
