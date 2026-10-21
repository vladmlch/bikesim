// Task 3.19: tests/reference/test_pedal_waveform.py, all ten functions and
// original phase/friction/tilt parameter tables. No captured values invented.
#include "drive_reference_support.hpp"
#include "rider/force_requests.hpp"

#include <array>
#include <limits>
#include <numeric>
#include <vector>

namespace {
namespace rider = bike::rider;
namespace cfg = bike::configuration;
using drive_reference::near;
using rider::Force3;
constexpr std::array<Force3, 2> upright{{{0., 0., 1.}, {0., 0., 1.}}};
constexpr std::array<double, 2> loaded{1e4, 1e4};

double norm(const Force3& force) { return std::sqrt(force[0] * force[0] + force[1] * force[1] + force[2] * force[2]); }
double dot(const Force3& a, const Force3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
void finite(const Force3& value) {
    for (double component : value) bt::require(std::isfinite(component), "finite pedal request component");
}
double net_torque(double phase, const rider::PedalingRequests& request) {
    double torque = 0.;
    for (std::size_t side = 0; side < 2; ++side) {
        const double angle = phase + (side == 0 ? 0. : std::numbers::pi);
        const Force3 lever{.175 * std::cos(angle), 0., -.175 * std::sin(angle)};
        const auto& force = request.force[side];
        torque += lever[2] * force[0] - lever[0] * force[2];
    }
    return torque;
}
void test_waveform_mean_and_ripple() {
    std::vector<double> values(3600);
    for (std::size_t i = 0; i < values.size(); ++i) {
        const double phase = static_cast<double>(i) * (2. * std::numbers::pi / static_cast<double>(values.size()));
        values[i] = rider::pedal_torque_waveform(40., phase, .5);
    }
    near(std::accumulate(values.begin(), values.end(), 0.) / static_cast<double>(values.size()), 40., "cycle mean torque", 1e-6);
    near(*std::max_element(values.begin(), values.end()), 60., "ripple peak");
    near(*std::min_element(values.begin(), values.end()), 20., "ripple trough");
    bt::require(rider::pedal_torque_waveform(40., 0., 0.) == 40., "zero ripple is constant torque");
}
void test_waveform_rejects_invalid_ripple() {
    for (double bad : {-.1, 1., 1.5, std::numeric_limits<double>::quiet_NaN()})
        bt::invalid([&] { (void)rider::pedal_torque_waveform(40., 0., bad); });
}
void test_force_requests_honour_the_ripple_argument() {
    const auto lo = rider::pedaling_force_requests(0., 40., .175, upright, loaded, 5., 0.);
    const auto hi = rider::pedaling_force_requests(0., 40., .175, upright, loaded, 5., .5);
    bt::require(norm(hi.force[0]) + norm(hi.force[1]) > 1.3 * (norm(lo.force[0]) + norm(lo.force[1])),
                "ripple changes force request magnitude");
}
void test_horizontal_power_stroke_requests_the_finite_crank_torque() {
    for (double phase : {0., std::numbers::pi}) for (double mu : {0., .9, 5.}) {
        const auto forces = rider::pedaling_force_requests(phase, 40., .175, upright, loaded, mu, 0.);
        for (const auto& force : forces.force) finite(force);
        near(net_torque(phase, forces), 40., "horizontal power stroke net torque");
    }
}
void test_articulated_config_exposes_the_ripple() {
    auto config = std::make_unique<cfg::ArticulatedConfig>();
    bt::require(config->pedal_torque_ripple == .35, "declared articulated ripple");
    config->pedal_torque_ripple = .5; cfg::detail::post_init(*config);
    bt::require(config->pedal_torque_ripple == .5, "valid configured ripple");
    config->pedal_torque_ripple = 1.; bt::invalid([&] { cfg::detail::post_init(*config); });
}
void test_return_foot_preload_is_a_nonnegative_force() {
    auto config = std::make_unique<cfg::ArticulatedConfig>();
    bt::require(config->return_foot_preload_n == 0., "declared preload is zero");
    config->return_foot_preload_n = 40.; cfg::detail::post_init(*config);
    bt::require(config->return_foot_preload_n == 40., "positive preload is accepted");
    config->return_foot_preload_n = -1.; bt::invalid([&] { cfg::detail::post_init(*config); });
}
void test_recovery_preload_preserves_the_requested_net_crank_waveform() {
    for (double phase : {0., .35, std::numbers::pi, std::numbers::pi + .35}) for (bool tilted : {false, true}) {
        const std::array<Force3, 2> normals = tilted
            ? std::array<Force3, 2>{{{.1, 0., std::sqrt(.99)}, {-.1, 0., std::sqrt(.99)}}}
            : upright;
        const auto requests = rider::pedaling_force_requests(phase, 40., .175, normals, loaded, .9, .5, 40.);
        for (std::size_t side = 0; side < 2; ++side)
            if (requests.weights[side] <= 1e-6)
                near(-dot(requests.force[side], normals[side]), 40., "recovery foot normal preload");
        near(net_torque(phase, requests), 40. * (1. + .5 * std::cos(2. * phase)), "preload preserves net drive waveform");
    }
}
void test_recovering_foot_can_be_excluded_from_preload() {
    const auto requests = rider::pedaling_force_requests(0., 40., .175, upright, loaded, .9, .5, 40., {true, false});
    bt::require(requests.force[1] == Force3{}, "excluded rear recovery foot has zero force");
    near(-.175 * requests.force[0][2], 60., "front drive torque survives exclusion");
}
void test_infeasible_preload_geometry_never_requests_negative_drive() {
    const std::array<Force3, 2> normals{{{0., 0., 1.}, {0., 0., -1.}}};
    const auto requests = rider::pedaling_force_requests(0., .1, .175, normals, loaded, .9, 0., 40.);
    bt::require(requests.force[0] == Force3{}, "inverted recovery preload suppresses drive wish");
    near(requests.force[1][2], 40., "inverted recovery foot keeps mandatory preload");
}
void test_preload_inputs_are_validated_and_zero_load_stays_bounded() {
    const std::array<Force3, 2> normals{{{1., 0., 0.}, {1., 0., 0.}}};
    const std::array<double, 2> zero_load{};
    for (double bad : {-1., std::numeric_limits<double>::quiet_NaN()})
        bt::invalid([&] { (void)rider::pedaling_force_requests(0., 40., .175, normals, zero_load, .9, .35, bad); });
    const auto requests = rider::pedaling_force_requests(0., 40., .175, normals, zero_load, .9, .35, 40.);
    for (const auto& force : requests.force) {
        finite(force);
        near(dot(force, {0., 0., -1.}), 0., "zero load cannot request tangential pedal force");
    }
}
} // namespace
int main() { return bt::run([] {
    test_waveform_mean_and_ripple();
    test_waveform_rejects_invalid_ripple();
    test_force_requests_honour_the_ripple_argument();
    test_horizontal_power_stroke_requests_the_finite_crank_torque();
    test_articulated_config_exposes_the_ripple();
    test_return_foot_preload_is_a_nonnegative_force();
    test_recovery_preload_preserves_the_requested_net_crank_waveform();
    test_recovering_foot_can_be_excluded_from_preload();
    test_infeasible_preload_geometry_never_requests_negative_drive();
    test_preload_inputs_are_validated_and_zero_load_stays_bounded();
}); }
