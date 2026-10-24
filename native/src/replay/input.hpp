#pragma once
#include "../io/value.hpp"
#include "spatial_recipe.hpp"
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>
namespace bike::replay {
struct ReplayInput {
    io::Value summary, manifest, metrics;
    std::map<std::string, std::vector<std::byte>> assets;
    std::vector<io::Value> requested, applied, observations, transitions;
    std::vector<double> initial_state, final_state, terrain_vertices;
    std::int64_t episode_id = 0;
    std::optional<int> container_schema_version;
    int replay_contract_version = 0;
    std::optional<SpatialRecipe4> spatial_recipe;
};
// Validates either the historical schema-2 execution object or the explicitly
// distinct native-executable identity. This is data-only and loads no policy.
void validate_execution(const io::Value&);
[[nodiscard]] ReplayInput load_legacy_input(const std::filesystem::path&, std::optional<std::int64_t> episode = {});
[[nodiscard]] ReplayInput load_replay_input(const std::filesystem::path&, std::optional<std::int64_t> episode = {});
} // namespace bike::replay
