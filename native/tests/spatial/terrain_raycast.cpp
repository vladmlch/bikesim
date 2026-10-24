#include "support/migration_test.hpp"
#include "spatial/support/spatial_fixture.hpp"
#include "spatial/support/terrain_fixture.hpp"
#include "terrain/raycast3d.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

namespace {
using namespace bike::terrain;

void hit(const RayHit3d& value, double expected, double tolerance = 1e-6) {
    bt::require(value.status == QueryStatus3d::ok, "ray must hit");
    bt::require(std::abs(value.distance_m - expected) <= tolerance, "first-hit distance meets V02 tolerance");
    bt::require(std::abs(std::hypot(value.normal_world[0], value.normal_world[1], value.normal_world[2]) - 1.) <= 1e-12,
                "hit normal is unit");
}
} // namespace

int main() {
    return bt::run([] {
        const auto flat = spatial_tests::plane_grid(0., 0., 0., 16, 12, .25, .5);
        hit(flat.ray_cast({0., 0., 1.}, {0., 0., -1.}, 2.), 1.);
        bt::require(flat.ray_cast({0., 0., 1.}, {0., 0., 1.}, 2.).status == QueryStatus3d::miss, "upward vertical ray misses");
        bt::require(flat.ray_cast({0., 0., -.01}, {0., 0., 1.}, 2.).status == QueryStatus3d::origin_below_surface, "below surface is distinct from miss");
        hit(flat.ray_cast({0., 0., 0.}, {0., 0., -1.}, 0.), 0.);
        hit(flat.ray_cast({0., 0., 0.}, {0., 0., 1.}, 0.), 0.);
        hit(flat.ray_cast({0., 0., 0.}, {0., 0., 1.}, 2.), 0.);
        // Despite a tiny horizontal component, this ray leaves the grid long
        // before reaching z=0. Treating it as vertical fabricated an edge hit.
        bt::require(flat.ray_cast({0., 0., 1e17}, {5e-16, 0., -1.}, 1e17).status == QueryStatus3d::miss,
                    "near-vertical rays retain their finite XY query domain");
        bt::require(flat.ray_cast({-1., 0., 1e17}, {5e-16, 0., -1.}, 1e17).status == QueryStatus3d::miss,
                    "near-vertical rays can enter and leave the XY domain");
        const double yaw = 37. * std::numbers::pi / 180.;
        const double pitch = -30. * std::numbers::pi / 180.;
        const std::array<double, 3> oblique{std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)};
        hit(flat.ray_cast({.2, -.5, 1.}, oblique, 3.), -1. / oblique[2]);
        const double diagonal = std::sqrt(.5);
        hit(flat.ray_cast({-1., 0., 1.}, {diagonal, 0., -diagonal}, 3.), 1. / diagonal);
        hit(flat.ray_cast({.25, 0., .1}, {0., diagonal, -diagonal}, 1.), .1 / diagonal);
        hit(flat.ray_cast({1., 0., .25}, {-diagonal, 0., -diagonal}, 1.), .25 / diagonal);
        hit(flat.ray_cast({1., 0., .25}, {-.5, -.5, -diagonal}, 1.), .25 / diagonal);
        bt::require(flat.ray_cast({-1., 0., 1.}, {-1., 0., 0.}, 3.).status == QueryStatus3d::outside, "empty clipped domain is outside");
        bt::require(flat.ray_cast({0., 0., 1.}, {0., 0., -2.}, 2.).status == QueryStatus3d::invalid_input, "nonunit direction rejected");
        bt::require(flat.ray_cast({0., 0., 1.}, {0., 0., -1.}, -1.).status == QueryStatus3d::invalid_input, "negative distance rejected");
        bt::require(flat.ray_cast({0., 0., 1.}, {std::numeric_limits<double>::quiet_NaN(), 0., -1.}, 2.).status == QueryStatus3d::invalid_input, "NaN direction rejected");
        const std::array<double, 10> coefficients{-5.25, 5., 0., -1., 0., 0., 0., 0., 0., 0.};
        const auto hump = spatial_tests::polynomial_grid(coefficients, 6, 4, 1., 1.);
        const auto patch = bicubic_patch(hump, 2, 1);
        const auto p = line_minus_surface(patch, {0., .5}, {1., 0.}, .9, 0.);
        const auto roots = spatial_tests::polynomial_ray_roots_highprecision(p);
        bt::require(roots.size() == 2, "independent oracle finds two same-sign-endpoint roots inside one cell");
        bt::require(p.a[0] > 0., "fixture starts above surface");
        double endpoint = 0.; for (double a : p.a) endpoint += a;
        bt::require(endpoint > 0., "fixture ends above surface");
        hit(hump.ray_cast({2., 0., .9}, {1., 0., 0.}, 1.), roots.front());
        const auto isolated = earliest_root_bernstein(p, {});
        bt::require(isolated.status == QueryStatus3d::ok && std::abs(isolated.s - roots.front()) <= 1e-6,
                    "Bernstein search returns the earliest root");
        hit(hump.ray_cast({2., 0., 1.}, {1., 0., 0.}, 1.), .5);
        bt::require(hump.ray_cast({2., 0., 1. + 2e-8}, {1., 0., 0.}, 1.).status == QueryStatus3d::miss,
                    "tangent near-miss above residual tolerance remains miss");
        const auto bump = spatial_tests::bump_grid(2., .4, 9, 9, .25, .25);
        hit(bump.ray_cast({0., 0., 2.}, {1., 0., 0.}, 2.), 1.);
        bt::require(bump.ray_cast({0., 0., 2. + 2e-8}, {1., 0., 0.}, 2.).status == QueryStatus3d::miss,
                    "Gaussian bump near-miss is not a tangent hit");
        Poly6 tangent;
        tangent.a[0] = .371 * .371; tangent.a[1] = -2. * .371; tangent.a[2] = 1.;
        const auto tangent_root = earliest_root_bernstein(tangent, {});
        bt::require(tangent_root.status == QueryStatus3d::ok && std::abs(tangent_root.s - .371) <= 1e-6,
                    "non-dyadic stationary tangency is isolated");
        // p=(s-1/4)(s-3/4)(s+3/16): both endpoint values are positive,
        // p'(0)=0, and the open interval has a second stationary point below
        // zero. The boundary stationary point must not prove a false miss.
        Poly6 endpoint_stationary;
        endpoint_stationary.a[0] = 9. / 256.;
        endpoint_stationary.a[2] = -13. / 16.;
        endpoint_stationary.a[3] = 1.;
        auto coarse_budget = RayBudget{};
        coarse_budget.root_interval_m = 1.;
        const auto endpoint_roots = earliest_root_bernstein(endpoint_stationary, coarse_budget);
        bt::require(endpoint_roots.status == QueryStatus3d::ok && std::abs(endpoint_roots.s - .25) <= 1e-6,
                    "stationary boundary preserves the earliest interior crossing");
        auto budget = flat.budget(); budget.max_cells = 1;
        auto bounded = Grid3d::create(flat.descriptor(), {flat.elevation().begin(), flat.elevation().end()},
            {flat.materials().begin(), flat.materials().end()}, {flat.surfaces().begin(), flat.surfaces().end()}, flat.metadata(), budget);
        bt::require(bounded.has_value(), "bounded fixture constructed");
        bt::require(bounded->ray_cast({0., 0., 2.}, {1., 0., 0.}, 3.).status == QueryStatus3d::budget_exhausted,
                    "cell budget exhaustion is not miss");
        budget = {}; budget.max_subdivisions = 0;
        bt::require(earliest_root_bernstein(p, budget).status == QueryStatus3d::budget_exhausted, "subdivision exhaustion explicit");
        budget = {}; budget.stack_capacity = 1;
        bt::require(earliest_root_bernstein(p, budget).status == QueryStatus3d::budget_exhausted, "fixed stack exhaustion explicit");
        const auto general = line_minus_surface(patch, {.1, .2}, {.7, .6}, 1.3, -.4);
        for (double s : {0., .1, .37, .71, 1.}) {
            double power_value = general.a[6];
            for (std::size_t k = 6; k != 0; --k) power_value = power_value * s + general.a[k - 1];
            const auto surface = evaluate(patch, .1 + .7 * s, .2 + .6 * s);
            bt::require(std::abs(power_value - (1.3 - .4 * s - surface.z)) <= 1e-12,
                        "sixth-degree line composition agrees with surface evaluation");
        }
    });
}
