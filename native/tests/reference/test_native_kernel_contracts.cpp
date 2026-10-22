// Task 3.21b: retain native kernel contracts when the Numba-vs-Python twin
// comparison loses its subject in phase 4. The four Hypothesis equivalence
// functions are not represented as 1000 native oracle passes by these tests.
#include "support/migration_test.hpp"
#include "contact/laws.hpp"
#include "rider/spindle_math.hpp"
#include "rider/support_geometry.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <tuple>
#include <utility>

namespace {
constexpr double rtol = 1e-12, atol = 1e-12;
void close(double actual, double expected, std::string_view label) {
    bt::require(std::isfinite(actual) && std::isfinite(expected) &&
        std::abs(actual - expected) <= atol + rtol * std::abs(expected), label);
}
template<class F> void overflow(F&& operation) {
    bool caught = false;
    try { std::forward<F>(operation)(); }
    catch (const std::overflow_error&) { caught = true; }
    bt::require(caught, "native arithmetic overflow remains an explicit error");
}
spindle::TorqueCurve curve() {
    return {.angles_rad={-1., 0., 1.}, .torques_nm={50., 100., 50.}, .vmax_rad_s=10., .hill_c=.25, .eccentric_ratio=1.4,
        .source="synthetic-equivalence-probe"};
}
void test_directional_capacity_error_paths_match() {
    const auto c = curve();
    for (const auto& [angle, velocity, direction] : std::array{
        std::tuple{0., 0., 0}, std::tuple{0., 0., 2}, std::tuple{2., 0., 1},
        std::tuple{0., std::numeric_limits<double>::infinity(), 1},
        std::tuple{0., std::numeric_limits<double>::quiet_NaN(), -1}})
        bt::invalid([&] { (void)spindle::directional_capacity(c, angle, velocity, direction); });
}
void directional_capacity_physical_boundaries() {
    const auto c = curve();
    spindle::validate_torque_curve(c, "native kernel control");
    for (int const direction : {-1, 1}) {
        close(spindle::directional_capacity(c, -1., 0., direction), 50., "left documented strength knot");
        close(spindle::directional_capacity(c, 0., 0., direction), 100., "isometric central strength knot");
        close(spindle::directional_capacity(c, 1., 0., direction), 50., "right documented strength knot");
        close(spindle::directional_capacity(c, .5, 0., direction), 75., "linear interpolation at the midpoint");
        close(spindle::directional_capacity(c, 0., direction * 10., direction), 0., "concentric unloaded speed has no remaining torque");
        close(spindle::directional_capacity(c, 0., direction * 20., direction), 0., "above unloaded speed cannot create negative capacity");
        close(spindle::directional_capacity(c, 0., direction * -10., direction), 120., "one maximum speed eccentrically lies halfway to the asymptote");
        double previous = 100.;
        for (double const speed : {0., .001, .1, 1., 5., 10., 100., 1e6}) {
            const double concentric = spindle::directional_capacity(c, 0., direction * speed, direction);
            bt::require(concentric >= 0. && concentric <= previous, "concentric capacity decreases from the isometric bound");
            previous = concentric;
            const double eccentric = spindle::directional_capacity(c, 0., -direction * speed, direction);
            bt::require(eccentric >= 100. && eccentric <= 140., "eccentric capacity stays between isometric and documented asymptote");
        }
    }
}
void test_brush_step_error_paths_match() {
    overflow([] { (void)contactlaw::brush_step(1e200, 0., 0., 1., 1e200, 1., 1., 1.); });
    overflow([] { (void)contactlaw::brush_step(0., 0., 1., 1e200, 1., 1e200, 1., 1.); });
    // The existing energy must be validated before an airborne early return.
    overflow([] { (void)contactlaw::brush_step(1e200, 0., 0., 0., 1e200, 1., 1., 1.); });
}
void brush_energy_and_friction_boundaries() {
    for (const auto& [load, friction] : std::array{std::pair{0., .8}, std::pair{500., 0.}}) {
        const auto [xi, force, lost] = contactlaw::brush_step(.01, 1., 2., load, 10000., friction, .04, .001);
        bt::require(xi == 0. && force == 0., "loss of normal support or friction releases all shear");
        close(lost, .5, "released shear reports the entire independently known spring energy");
    }
    for (double const slip : {-50., -2., -.1, 0., .1, 2., 50.}) {
        const auto [xi, force, lost] = contactlaw::brush_step(0., slip, 2., 500., 10000., .8, .04, .001);
        bt::require(std::isfinite(xi) && std::isfinite(force) && std::isfinite(lost), "valid brush update is finite");
        bt::require(std::abs(force) <= 400. + atol && lost >= 0., "contact respects its Coulomb bound and dissipates energy");
        bt::require(force * slip <= 0., "fresh brush opposes slip");
    }
}
void test_normal_contact_error_paths_match() {
    overflow([] { (void)contactlaw::normal_contact(1e200, 0., 1e200, 0.); });
}
void normal_contact_physical_boundaries() {
    bt::require(contactlaw::normal_contact(-1., 100., 1000., 20.) == std::pair{0., 0.}, "separated surfaces have no force or stored energy");
    bt::require(contactlaw::normal_contact(0., 100., 1000., 20.) == std::pair{0., 0.}, "zero penetration belongs to the open-contact boundary");
    const auto compressed = contactlaw::normal_contact(.125, 0., 1024., 16.);
    bt::require(compressed == std::pair{128., 8.}, "dyadic spring compression gives independently exact force and energy");
    const auto unloaded = contactlaw::normal_contact(.125, -16., 1024., 16.);
    bt::require(unloaded == std::pair{0., 8.}, "unloading cannot pull surfaces together or erase stored elastic energy");
}
void test_box_pad_contact_inside_and_tie_cases() {
    const ::rider::Vec3 origin{0., 0., 0.}, half{.15, .05, .02};
    const ::rider::Mat3 identity{1.,0.,0.,0.,1.,0.,0.,0.,1.};
    struct Case { ::rider::Vec3 center, point, normal; double signed_distance; bool footprint; };
    const double corner_distance = std::sqrt(.05 * .05 + .03 * .03);
    const std::array cases{
        Case{.center={0.,0.,0.}, .point={0.,0.,.02}, .normal={0.,0.,1.}, .signed_distance=-.02, .footprint=true},
        Case{.center={.15,0.,0.}, .point={.15,0.,0.}, .normal={1.,0.,0.}, .signed_distance=0., .footprint=true},
        Case{.center={0.,0.,.02}, .point={0.,0.,.02}, .normal={0.,0.,1.}, .signed_distance=0., .footprint=true},
        Case{.center={.05,0.,-.01}, .point={.05,0.,-.02}, .normal={0.,0.,-1.}, .signed_distance=-.01, .footprint=true},
        Case{.center={.2,0.,0.}, .point={.15,0.,0.}, .normal={1.,0.,0.}, .signed_distance=.05, .footprint=false},
        Case{.center={.2,0.,.05}, .point={.15,0.,.02}, .normal={.05/corner_distance,0.,.03/corner_distance}, .signed_distance=corner_distance, .footprint=false}
    };
    for (const auto& c : cases) for (double const radius : {.01, .03}) {
        const auto contact = ::rider::box_pad_contact(c.center, radius, origin, identity, half);
        for (std::size_t axis = 0; axis < 3; ++axis) {
            close(contact.point_m[axis], c.point[axis], "analytical nearest box face or corner");
            close(contact.normal[axis], c.normal[axis], "outward face/corner normal including medial tie");
        }
        close(contact.gap_m, c.signed_distance - radius, "sphere radius offsets signed distance from finite box");
        bt::require(contact.within_width && contact.within_footprint == c.footprint, "finite width and footprint remain separate decisions");
    }
}
} // namespace
int main() { return bt::run([] {
    test_directional_capacity_error_paths_match(); directional_capacity_physical_boundaries();
    test_brush_step_error_paths_match(); brush_energy_and_friction_boundaries();
    test_normal_contact_error_paths_match(); normal_contact_physical_boundaries();
    test_box_pad_contact_inside_and_tie_cases();
}); }
