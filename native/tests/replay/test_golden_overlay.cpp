// Golden overlay contracts: ghost pose interpolation, running-max Δq HUD,
// and contact-point tyre-force arrows (front orange, rear blue, zero omitted).
#include "replay/golden_overlay.hpp"
#include "replay/playback.hpp"
#include "support/fixture_test.hpp"
#include <cmath>
#include <limits>
#include <vector>

namespace {
namespace replay = bike::replay;

replay::PlaybackData two_frames(double x1) {
    replay::PlaybackData d;
    d.times_s = {0.0, 0.1};
    d.ends_s = {0.1, 0.2};
    d.qpos = {{0.0, 0.0}, {x1, 0.0}};
    d.qvel = {{0.0, 0.0}, {0.0, 0.0}};
    d.duration_s = 0.2;
    return d;
}

void ghost_pose_and_delta() {
    const auto ghost = two_frames(1.0);
    const auto shown = two_frames(1.01);
    const std::vector<std::array<double, 8>> wheels(2, std::array<double, 8>{});
    double running_max = 0.0;
    const auto dec = replay::golden_decorations(
        shown, ghost, wheels, {"root_x", "root_z"}, 0.1, running_max);
    bt::require(dec.ghost_qpos.has_value() && *dec.ghost_qpos == replay::pose_at(ghost, 0.1),
                "ghost is the recording pose");
    bt::require(std::abs(running_max - 0.01) < 1e-12, "running max tracks the worst delta");
    bt::require(dec.overlay.find("root_x") != std::string::npos, "overlay names the worst dof");
    bt::require(dec.overlay.find("1.000e-02") != std::string::npos, "overlay prints the delta");
    bt::require(dec.arrows.empty(), "no contact force, no arrows");

    // Moving back to t=0 keeps running_max monotonic.
    const auto earlier = replay::golden_decorations(
        shown, ghost, wheels, {"root_x", "root_z"}, 0.0, running_max);
    bt::require(std::abs(running_max - 0.01) < 1e-12, "running max is monotonic across seeks");
    bt::require(earlier.overlay.find("0.000e+00") != std::string::npos, "instantaneous delta is zero at t=0");
}

void arrows_from_contact_points() {
    const auto data = two_frames(1.0);
    const std::array<double, 8> front_only{1.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 0.0};
    const std::array<double, 8> both_wheels{1.2, 0.05, 100.0, 800.0, -0.8, 0.02, -50.0, 600.0};
    const std::vector<std::array<double, 8>> wheels{front_only, both_wheels};
    double running_max = 0.0;

    const auto dec0 = replay::golden_decorations(
        data, data, wheels, {"root_x", "root_z"}, 0.0, running_max);
    bt::require(dec0.arrows.size() == 1, "only the front wheel is in contact at t=0");
    const auto& front0 = dec0.arrows.front();
    bt::require(front0.from == std::array<double, 3>{1.0, 0.0, 0.0}, "arrow starts at the contact point");
    bt::require(std::abs(front0.to[2] - 0.5) < 1e-12 && front0.to[0] == 1.0,
                "0.0005 m per N along the force");

    const auto dec1 = replay::golden_decorations(
        data, data, wheels, {"root_x", "root_z"}, 0.1, running_max);
    bt::require(dec1.arrows.size() == 2, "both wheels draw arrows when both have nonzero force");
    bt::require(dec1.arrows[0].rgba[0] > dec1.arrows[0].rgba[2], "front arrow is orange-tinted");
    bt::require(dec1.arrows[1].rgba[2] > dec1.arrows[1].rgba[0], "rear arrow is blue-tinted");
    bt::require(std::abs(dec1.arrows[1].from[0] - (-0.8)) < 1e-12 &&
                    std::abs(dec1.arrows[1].from[2] - 0.02) < 1e-12,
                "rear arrow starts at rear contact point");
    bt::require(std::abs(dec1.arrows[1].to[0] - (-0.825)) < 1e-12 &&
                    std::abs(dec1.arrows[1].to[2] - 0.32) < 1e-12,
                "rear arrow scales 5e-4 m per N");
}

void guards_and_fallbacks() {
    const auto data = two_frames(1.0);
    const std::vector<std::array<double, 8>> wheels(2, std::array<double, 8>{});
    double running_max = 0.0;

    // Fewer dof names than nq falls back cleanly without OOB.
    const auto dec = replay::golden_decorations(data, two_frames(1.5), wheels, {}, 0.1, running_max);
    bt::require(dec.overlay.find("q0") != std::string::npos, "missing dof name falls back to q<index>");

    bt::invalid([&] {
        (void)replay::golden_decorations(
            data, data, wheels, {"root_x", "root_z"}, std::numeric_limits<double>::quiet_NaN(), running_max);
    });
    auto mismatch = two_frames(1.0);
    mismatch.qpos[0].push_back(0.0);
    mismatch.qpos[1].push_back(0.0);
    bt::invalid([&] {
        (void)replay::golden_decorations(data, mismatch, wheels, {"root_x", "root_z"}, 0.0, running_max);
    });
}
} // namespace

int main() {
    return bt::run([] {
        ghost_pose_and_delta();
        arrows_from_contact_points();
        guards_and_fallbacks();
    });
}
