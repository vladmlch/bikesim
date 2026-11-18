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

    // Nonfinite running_max is rejected at entry without mutating it.
    double bad_max = std::numeric_limits<double>::quiet_NaN();
    bt::invalid([&] {
        (void)replay::golden_decorations(data, data, wheels, {"root_x", "root_z"}, 0.0, bad_max);
    });
    bt::require(std::isnan(bad_max), "rejected running_max remains untouched");

    // Stage-before-commit: a throw on wheel telemetry leaves running_max untouched.
    double committed_max = 0.005;
    const std::vector<std::array<double, 8>> bad_wheels{
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0, 0.0, 0.0},
    };
    bt::invalid([&] {
        (void)replay::golden_decorations(
            data, two_frames(1.5), bad_wheels, {"root_x", "root_z"}, 0.1, committed_max);
    });
    bt::require(committed_max == 0.005, "running_max is not updated when golden_decorations throws");
}

void velocity_interpolates_across_strides() {
    constexpr double kFineDt = 0.00125;
    constexpr std::size_t kFineFrames = 41; // 0.000 .. 0.050 s
    constexpr double kAccel = 20.0;

    replay::PlaybackData ghost;
    replay::PlaybackData shown;
    for (std::size_t i = 0; i < kFineFrames; ++i) {
        const double t = static_cast<double>(i) * kFineDt;
        const std::vector<double> q{0.5 * kAccel * t * t, 0.0};
        const std::vector<double> v{kAccel * t, 1.0 - 4.0 * t};
        ghost.times_s.push_back(t);
        ghost.ends_s.push_back(t + kFineDt);
        ghost.qpos.push_back(q);
        ghost.qvel.push_back(v);
        if (i % 10 == 0) {
            shown.times_s.push_back(t);
            shown.ends_s.push_back(t + kFineDt * 10.0);
            shown.qpos.push_back(q);
            shown.qvel.push_back(v);
        }
    }
    ghost.duration_s = ghost.ends_s.back();
    shown.duration_s = shown.ends_s.back();

    const std::vector<std::array<double, 8>> wheels(shown.times_s.size(), std::array<double, 8>{});
    double running_max = 0.0;

    // Sample off-grid at 1/60 s: linear qvel(t) must agree across strides.
    const auto dec = replay::golden_decorations(
        shown, ghost, wheels, {"root_x", "root_z"}, 1.0 / 60.0, running_max);
    const auto pos = dec.overlay.find("max|Δv| ");
    bt::require(pos != std::string::npos, "overlay includes max|Δv|");
    const double max_dv = std::stod(dec.overlay.substr(pos + 9));
    bt::require(max_dv < 1e-12, "interpolated qvel matches across 10x stride mismatch");

    // Exact knot sample produces exact zero velocity delta.
    const auto knot = replay::golden_decorations(
        shown, ghost, wheels, {"root_x", "root_z"}, 0.025, running_max);
    bt::require(knot.overlay.find("max|Δv| 0.000e+00") != std::string::npos,
                "aligned knot produces exact zero max|Δv|");

    // Clamped boundaries and single-frame PlaybackData do not go out of bounds.
    replay::PlaybackData single;
    single.times_s = {0.0};
    single.ends_s = {0.01};
    single.qpos = {{0.0, 0.0}};
    single.qvel = {{2.0, -1.0}};
    single.duration_s = 0.01;
    const auto low = replay::golden_decorations(single, single, wheels, {"root_x", "root_z"}, -0.1, running_max);
    const auto high = replay::golden_decorations(single, single, wheels, {"root_x", "root_z"}, 10.0, running_max);
    bt::require(low.overlay.find("max|Δv| 0.000e+00") != std::string::npos &&
                    high.overlay.find("max|Δv| 0.000e+00") != std::string::npos,
                "single-frame and clamped boundary velocity lookup succeeds");
}
} // namespace

int main() {
    return bt::run([] {
        ghost_pose_and_delta();
        arrows_from_contact_points();
        guards_and_fallbacks();
        velocity_interpolates_across_strides();
    });
}
