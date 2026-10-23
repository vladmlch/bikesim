#pragma once

#include "../io/value.hpp"

#include <cstdint>

namespace bike::golden {

inline constexpr std::uint32_t kSpatialManifestVersion = 1;
// Invalid/missing provenance and empty selected fixture sets fail closed.
void validate_spatial_manifest(const bike::io::Value&);

} // namespace bike::golden
