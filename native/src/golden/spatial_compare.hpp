#pragma once
// Private W44 comparison policy. The frozen spatial_registry.hpp API remains
// the structural manifest validator; file/provenance checks live here.
#include "episode.hpp"

namespace bike::golden {
struct SpatialReferenceStatus {
    bool pending = false;
    std::string platform{};
};
[[nodiscard]] SpatialReferenceStatus inspect_spatial_reference(const io::Value& entry,
    const std::filesystem::path& directory, std::string_view current_platform);
[[nodiscard]] std::vector<std::string> compare_spatial_artifacts(const EpisodeArtifact& expected,
    const EpisodeArtifact& actual, const io::Value& entry, std::string_view current_platform);
[[nodiscard]] std::vector<std::string> compare_spatial_episode(const EpisodeArtifact& expected,
    runtime::NativeResearchRuntime&, const io::Value& entry, std::string_view current_platform);
}
