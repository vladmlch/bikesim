#include "golden_overlay.hpp"
#include <algorithm>
#include <cmath>
#include <format>
#include <span>
#include <stdexcept>

namespace bike::replay {
namespace {
[[nodiscard]] std::size_t wheel_row_index(
    const PlaybackData& data,
    std::span<const std::array<double, 8>> wheels,
    double time_s) {
    const double clamped = std::clamp(time_s, 0., data.duration_s);
    const auto after = std::ranges::upper_bound(data.times_s, clamped);
    const std::size_t raw = after == data.times_s.begin()
        ? 0
        : static_cast<std::size_t>(after - data.times_s.begin() - 1);
    return std::min(raw, wheels.size() - 1);
}

[[nodiscard]] std::size_t velocity_row_index(const PlaybackData& data, double time_s) {
    const double clamped = std::clamp(time_s, 0., data.duration_s);
    const auto found = std::ranges::lower_bound(data.times_s, clamped);
    return std::min(static_cast<std::size_t>(found - data.times_s.begin()), data.qvel.size() - 1);
}
} // namespace

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the task specification
viewer::Decorations golden_decorations(
    const PlaybackData& shown,
    const PlaybackData& ghost,
    const std::vector<std::array<double, 8>>& wheels,
    const std::vector<std::string>& dof_names,
    double time_s,
    double& running_max) {
    if (!std::isfinite(time_s)) {
        throw std::invalid_argument("golden_decorations requires a finite time");
    }

    auto ghost_q = pose_at(ghost, time_s);
    const auto shown_q = pose_at(shown, time_s);
    if (shown_q.size() != ghost_q.size() || shown_q.empty()) {
        throw std::invalid_argument("golden_decorations: trajectory pose dimensions differ");
    }

    std::size_t best_k = 0;
    double max_dq = -1.;
    for (std::size_t i = 0; i < shown_q.size(); ++i) {
        const double delta = std::abs(shown_q[i] - ghost_q[i]);
        if (delta > max_dq) {
            max_dq = delta;
            best_k = i;
        }
    }
    running_max = std::max(running_max, max_dq);

    const std::string dof_label = best_k < dof_names.size()
        ? dof_names[best_k]
        : ("q" + std::to_string(best_k));

    viewer::Decorations out;
    out.overlay = std::format("GPU vs CPU  max|Δq| {:.3e} ({})  run max {:.3e}",
                              max_dq, dof_label, running_max);

    if (!shown.qvel.empty() && !ghost.qvel.empty() && !shown.times_s.empty() && !ghost.times_s.empty()) {
        const auto& shown_v = shown.qvel[velocity_row_index(shown, time_s)];
        const auto& ghost_v = ghost.qvel[velocity_row_index(ghost, time_s)];
        if (shown_v.size() == ghost_v.size() && !shown_v.empty()) {
            double max_dv = 0.;
            for (std::size_t i = 0; i < shown_v.size(); ++i) {
                max_dv = std::max(max_dv, std::abs(shown_v[i] - ghost_v[i]));
            }
            out.overlay += std::format("  max|Δv| {:.3e}", max_dv);
        }
    }

    std::string named;
    for (const std::string_view key : {"root_x", "root_z", "root_pitch"}) {
        const auto it = std::ranges::find(dof_names, key);
        if (it == dof_names.end()) continue;
        const auto idx = static_cast<std::size_t>(it - dof_names.begin());
        if (idx >= shown_q.size()) continue;
        const double delta = std::abs(shown_q[idx] - ghost_q[idx]);
        if (!named.empty()) named += "  ";
        named += std::format("|Δ{}| {:.3e}",
                             key.substr(5), delta);
    }
    if (!named.empty()) {
        out.overlay += "\n" + named;
    }

    if (!wheels.empty() && !shown.times_s.empty()) {
        const auto& row = wheels[wheel_row_index(shown, wheels, time_s)];
        constexpr double kArrowScale = 5e-4;
        constexpr std::array<std::array<float, 4>, 2> kColors{{
            {1.f, .45f, .1f, .95f}, // front wheel: orange
            {.2f, .55f, 1.f, .95f}, // rear wheel: blue
        }};
        for (std::size_t w = 0; w < 2; ++w) {
            const std::size_t off = w * 4;
            const double cx = row[off];
            const double cz = row[off + 1];
            const double fx = row[off + 2];
            const double fz = row[off + 3];
            if (!std::isfinite(cx) || !std::isfinite(cz) || !std::isfinite(fx) || !std::isfinite(fz)) {
                throw std::invalid_argument("golden_decorations: nonfinite wheel contact or force");
            }
            if (fx == 0. && fz == 0.) continue;
            out.arrows.push_back(viewer::Arrow{
                .from = {cx, 0., cz},
                .to = {cx + kArrowScale * fx, 0., cz + kArrowScale * fz},
                .rgba = kColors[w],
            });
        }
    }

    out.ghost_qpos = std::move(ghost_q);
    return out;
}
} // namespace bike::replay
