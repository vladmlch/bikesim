#pragma once

#include "../../../src/io/value.hpp"

#include <cstddef>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace spatial_tests {

// Bounded fixture root; reject empty/absolute/traversal and symlink escapes.
std::filesystem::path fixture(std::string_view relative);
std::string text(const std::filesystem::path&);
std::vector<std::byte> bytes(const std::filesystem::path&);
bike::io::Value json(const std::filesystem::path&);
std::string sha256_hex(std::span<const std::byte>);

} // namespace spatial_tests
