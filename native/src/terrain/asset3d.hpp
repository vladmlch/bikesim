#pragma once

#include "terrain/grid3d.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <vector>

namespace bike::terrain {

inline constexpr std::array<char, 4> kTerrain3dMagic{'B', 'K', 'T', '3'};
inline constexpr std::uint32_t kTerrain3dVersion = 1;
inline constexpr std::uint32_t kTerrain3dHeaderBytes = 112;

struct Terrain3dLoadLimits {
    std::uint64_t max_total_bytes = 1ull << 30u;
    std::uint64_t max_nodes = 100'000'000;
};

// BKT3/v1: little endian header, heights, cell material IDs, canonical
// surface JSON, canonical metadata JSON; no trailing bytes (contract C3.1).
std::expected<Grid3d, std::string> read_terrain3d(
    std::span<const std::byte>, Terrain3dLoadLimits = {});
std::vector<std::byte> write_terrain3d(const Grid3d&);

} // namespace bike::terrain
