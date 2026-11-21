#pragma once
#include "playback.hpp"
#include <array>
#include <filesystem>
#include <vector>

namespace bike::replay {
// Externally produced trajectory (GPU golden replay). Presentation only: it is
// not a recording, carries no seal and never certifies anything.
struct ExternalTrajectory {
    PlaybackData playback;
    std::vector<std::array<double, 8>> wheels;
};

[[nodiscard]] ExternalTrajectory load_external_trajectory(const std::filesystem::path& dir);
} // namespace bike::replay
