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

// Extra scene content for one frame: a translucent ghost pose of the same
// model, force arrows, and text appended to the overlay. Presentation only.
struct Decorations {
    std::optional<std::vector<double>> ghost_qpos;
    std::vector<Arrow> arrows;
    std::string overlay;
};
} // namespace bike::viewer
