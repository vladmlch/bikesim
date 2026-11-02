// Completion 3.23. Source: tests/reference/test_lean_limit.py.
// Geometric lean ceiling over the articulated pose and the anatomical
// envelopes: elbow ROM can lower the attainable lean and the torso hinge
// caps it at its own residual range.
#include "support/fixture_test.hpp"
#include "support/golden.hpp"
#include "configuration/rider_spec.hpp"
#include "geometry/specs.hpp"
#include "model/rider_geometry.hpp"
#include "rider/spindle_controller.hpp"
#include <cmath>
#include <filesystem>
#include <map>
#include <numbers>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace model = bike::model;

namespace {

// The live anatomical envelope document — the oracle's test path resolves
// the same repo-relative file.
std::filesystem::path repo_file(std::string_view relative) {
    return bt::oracle::root().parent_path().parent_path().parent_path() /
           std::filesystem::path(relative);
}

std::map<std::string, spindle::JointEnvelope> envelopes() {
    const auto loaded = bike::configuration::load_joint_envelopes(
        repo_file("examples/research/rider_joint_envelope_anatomical.json"));
    return {loaded.begin(), loaded.end()};
}

rider::SpindlePose pose() {
    bike::configuration::RiderSpecs rider;
    rider.variant = "articulated_planar";
    const auto articulated =
        model::geometry_pose(rider, bike::geometry::BikeSpecs{});
    return {.hip = articulated.hip,
            .shoulder = articulated.shoulder,
            .elbow = articulated.elbow,
            .grip = articulated.grip,
            .knee_front = articulated.knee_front,
            .knee_rear = articulated.knee_rear,
            .pedal_front = articulated.pedal_front,
            .pedal_rear = articulated.pedal_rear,
            .ankle_front = articulated.ankle_front,
            .ankle_rear = articulated.ankle_rear};
}

double crank_radius(const rider::SpindlePose &p) {
    // np.linalg.norm(3-vec) / 2 — the full spatial crank diameter, not the
    // in-plane xz the controller feeds leg loops.
    double squared = 0.;
    for (std::size_t i = 0; i < 3; ++i) {
        const double delta = p.pedal_front[i] - p.pedal_rear[i];
        squared += delta * delta;
    }
    return std::sqrt(squared) / 2.;
}

// test_lean_limit.py::test_lean_limit_is_geometric_and_elbow_rom_can_lower_it
void test_lean_limit_is_geometric_and_elbow_rom_can_lower_it() {
    const auto p = pose();
    const auto base = envelopes();
    const double radius = crank_radius(p);
    const double limit = rider::lean_limit_rad(
        p, base, radius, -std::numbers::pi / 2.);
    bt::require(.1 < limit && limit < .8,
                "the anatomical envelope leaves a feasible lean band");
    auto tight = base;
    for (const char *name : {"rider_elbow_left", "rider_elbow_right"}) {
        auto &envelope = tight.at(name);
        envelope.maximum_anatomical_rad =
            envelope.neutral_anatomical_rad + .2;
    }
    bt::require(rider::lean_limit_rad(p, tight, radius,
                                    -std::numbers::pi / 2.) < limit,
                "tighter elbow ROM lowers the geometric lean limit");
}

// test_lean_limit.py::test_torso_rom_caps_the_limit
void test_torso_rom_caps_the_limit() {
    const auto p = pose();
    auto tight = envelopes();
    auto &torso = tight.at("rider_torso_hinge");
    torso.maximum_anatomical_rad = torso.neutral_anatomical_rad + .3;
    const double limit = rider::lean_limit_rad(
        p, tight, crank_radius(p), -std::numbers::pi / 2.);
    bt::require(std::abs(limit - .3) <= .001,
                "the torso hinge caps the lean at its residual .3 rad");
}

} // namespace

int main() {
    return bt::run([] {
        test_lean_limit_is_geometric_and_elbow_rom_can_lower_it();
        test_torso_rom_caps_the_limit();
    });
}
