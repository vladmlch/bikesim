#pragma once

#include "../io/value.hpp"
#include "../terrain/source3d.hpp"

#include <cstdint>
#include <string>

namespace bike::replay {

inline constexpr std::uint32_t kSpatialReplayContract = 4;

struct SpatialRecipe4 {
    std::uint32_t contract = kSpatialReplayContract;
    bike::terrain::Terrain3dKind terrain_kind = bike::terrain::Terrain3dKind::grid;
    std::string terrain_sha256_hex;
    bike::io::Value config_spatial;
    bike::io::Value checkpoint_initial, checkpoint_final;
    std::string execution_identity_sha256;
};

SpatialRecipe4 read_spatial_recipe(const bike::io::Value&);
bike::io::Value write_spatial_recipe(const SpatialRecipe4&);

} // namespace bike::replay
