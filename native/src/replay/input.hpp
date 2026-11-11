#pragma once
#include "../io/value.hpp"
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
    // Launch part 1, task 4 — the deduplicated rider_commands evidence
    // (rows shaped {step, commands}) beside the applied commands. The
    // flag records whether the recording owns the channel table at all:
    // older containers predate it and skip the compare entirely.
    std::vector<io::Value> rider_commands;
    bool has_rider_commands = false;
    std::vector<double> initial_state, final_state, terrain_vertices;
    std::int64_t episode_id = 0;
    std::optional<int> container_schema_version;
    int replay_contract_version = 0;
};
// Validates either the historical schema-2 execution object or the explicitly
// distinct native-executable identity. This is data-only and loads no policy.
void validate_execution(const io::Value&);
[[nodiscard]] ReplayInput load_legacy_input(const std::filesystem::path&, std::optional<std::int64_t> episode = {});
[[nodiscard]] ReplayInput load_replay_input(const std::filesystem::path&, std::optional<std::int64_t> episode = {});
} // namespace bike::replay
