#pragma once
#include "reader.hpp"
#include <filesystem>
#include <optional>
#include <string_view>
namespace bike::recording {
[[nodiscard]] io::Value export_data(const Recording&, std::int64_t episode, std::string_view table,
    std::string_view format, const std::filesystem::path& output, std::optional<std::uint64_t> limit = {});
void export_table(const Recording&, std::int64_t episode, std::string_view table, std::string_view format,
    const std::filesystem::path& output, std::optional<std::uint64_t> limit = {});
} // namespace bike::recording
