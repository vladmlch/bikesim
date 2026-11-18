#pragma once
#include "playback.hpp"
#include "../viewer/decorations.hpp"
#include <array>
#include <string>
#include <vector>

namespace bike::replay {
[[nodiscard]] viewer::Decorations golden_decorations(
    const PlaybackData& shown,
    const PlaybackData& ghost,
    const std::vector<std::array<double, 8>>& wheels,
    const std::vector<std::string>& dof_names,
    double time_s,
    double& running_max);
} // namespace bike::replay
