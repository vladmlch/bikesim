// Linux root-build identity support for the relocatable bin/lib layout.
// This cold-path helper never changes the loaded-image inspection rules.
#pragma once

#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace runtime {
struct InstalledProviderIdentity {
    std::string policy_json;
    std::vector<std::string> required_images;
};

// Resolve only the exact adjacent lib/<recorded filename> candidates, and
// verify each regular file against the compiled policy's SHA-256. Require
// complete agreement with the original required-image list. No provider is
// selected from PATH, the environment or a global basename search.
// The caller must additionally verify that these exact files are loaded.
// Invalid policy/layout input throws invalid_argument; missing, escaping or
// modified installed files throw runtime_error/system_error. Nothing mutates
// the supplied policy or any file, including on a partially checked failure.
[[nodiscard]] InstalledProviderIdentity installed_provider_identity(
    std::string_view policy_json, std::span<const std::string> required_images,
    const std::filesystem::path& executable);
} // namespace runtime
