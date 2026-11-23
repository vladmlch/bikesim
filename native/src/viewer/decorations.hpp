#pragma once
#include <array>
#include <optional>
#include <string>
#include <vector>

namespace bike::viewer {
struct Arrow {
    std::array<double, 3> from{};
    std::array<double, 3> to{};
    std::array<float, 4> rgba{1.f, .3f, .1f, 1.f};
};

struct SurfaceBand {
    std::array<double, 3> from{};
    std::array<double, 3> to{};
    double radius_m = .018;
    std::array<float, 4> rgba{.35f, .35f, .38f, .9f};
};

// Extra scene content for one frame: a translucent ghost pose of the same
// model, force arrows, surface-material ribbon capsules, and text appended to
// the overlay. Presentation only.
struct Decorations {
    std::optional<std::vector<double>> ghost_qpos;
    std::vector<Arrow> arrows;
    std::vector<SurfaceBand> surface_bands{};
    std::string overlay;
};
} // namespace bike::viewer
