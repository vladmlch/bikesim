#include "support/migration_test.hpp"
#include "support/startup_fixture.hpp"
#include "support/terrain_fixture.hpp"
#include "engine_call.hpp"
#include "model_access.hpp"
#include "runtime/control.hpp"
#include "terrain/source3d.hpp"

#include <array>
#include <cmath>

namespace {
using Point = std::array<double, 3>;
void grades(const std::vector<runtime::RoadSample>& samples, double expected) {
    bt::require(!samples.empty(), "physical preview has supported samples");
    for (const auto& sample : samples)
        bt::require(std::abs(sample.grade - expected) < 1e-12,
                    "preview follows the analytic directional gradient");
    bt::require(std::abs(runtime::road_grade_for_posture(samples) - expected) < 1e-12 &&
                std::abs(runtime::road_grade_preview(samples) - expected) < 1e-12,
                "both posture and effort receive the physical directional grade");
}
}

int main() { return bt::run([] {
    const bike::terrain::Terrain3dSource bank(
        spatial_tests::plane_grid(0., .2, 0., 41, 41, .25, .25));
    const std::array wheels{Point{4., -.5, 1.}, Point{4., .5, 1.}};
    const auto uphill = runtime::spatial_road_samples(bank.query_view(), wheels, {0., 1., 0.}, .75, .25);
    grades(uphill, .2);
    bt::require(std::abs(uphill.front().height_m + .1) < 1e-12 &&
                std::abs(uphill.back().height_m - .25) < 1e-12,
                "wheel positions and forward endpoint sample their actual lateral coordinates");
    const auto downhill = runtime::spatial_road_samples(bank.query_view(), wheels, {0., -1., 0.}, .75, .25);
    grades(downhill, -.2);
    bt::require(std::abs(downhill.back().height_m + .25) < 1e-12,
                "reverse heading chooses the other leading wheel and preview endpoint");
    grades(runtime::spatial_road_samples(bank.query_view(), wheels, {1., 0., .3}, .5), 0.);

    // z=.1*x*y has d(z)/dx=.1*y. The two rider poses share the
    // same X station but must not use the y=0 centreline's zero grade.
    const std::array<double, 10> polynomial{0., 0., 0., 0., .1, 0., 0., 0., 0., 0.};
    const bike::terrain::Terrain3dSource changing(
        spatial_tests::polynomial_grid(polynomial, 41, 41, .25, .25));
    const std::array left{Point{3., 1., 1.}, Point{4., 1., 1.}};
    const std::array right{Point{3., -1., 1.}, Point{4., -1., 1.}};
    grades(runtime::spatial_road_samples(changing.query_view(), left, {1., 0., 0.}, .5), .1);
    grades(runtime::spatial_road_samples(changing.query_view(), right, {1., 0., 0.}, .5), -.1);

    const std::array edge{Point{4., 4., 1.}, Point{4., 4.5, 1.}};
    const auto truncated = runtime::spatial_road_samples(bank.query_view(), edge, {0., 1., 0.}, 2., .25);
    bt::require(truncated.size() == 4 && std::abs(truncated.back().height_m - 1.) < 1e-12,
                "optional lookahead ends at the finite physical edge without failing or extrapolating");
    bt::invalid([&] { (void)runtime::spatial_road_samples({}, wheels, {1., 0., 0.}, .5); });
    bt::invalid([&] { (void)runtime::spatial_road_samples(bank.query_view(), wheels, {0., 0., 1.}, .5); });
    bt::invalid([&] { (void)runtime::spatial_road_samples(bank.query_view(), wheels, {1., 0., 0.}, 1e100); });
    bt::invalid([&] { (void)runtime::spatial_road_samples(bank.query_view(), wheels, {1., 0., 0.}, .5, 0.); });

    // The actual state adapter must use independent qpos/dof addresses and
    // current geometry, including yaw, rather than its legacy X-Z cache.
    auto rig = spatial_tests::mixed_joint_rig();
    auto qpos = model_access::mutable_buffer(rig.data->qpos, rig.model->nq);
    const auto q = static_cast<std::size_t>(rig.topology.root.free_qpos_adr);
    qpos[q] = 4.; qpos[q + 1] = 0.; qpos[q + 2] = 1.;
    qpos[q + 3] = std::sqrt(.5); qpos[q + 4] = qpos[q + 5] = 0.; qpos[q + 6] = std::sqrt(.5);
    engine::kinematics(rig.model, rig.data);
    const auto state = runtime::spatial_rider_kinematic_state(
        rig.model, rig.data, rig.topology, bank.query_view(), .5);
    grades(state.road, .2);
    bt::require(state.qpos.size() == static_cast<std::size_t>(rig.model->nq) &&
                state.qvel.size() == static_cast<std::size_t>(rig.model->nv),
                "spatial rider snapshots preserve independent state dimensions");
}); }
