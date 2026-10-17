#pragma once
#include "catalog.hpp"
#include "duckdb.hpp"
#include <filesystem>
#include <functional>

namespace bike::recording {
class RunCapture;
[[nodiscard]] io::Value::Object validate_database(Connection& connection);
void validate_database(const std::filesystem::path& file);
void write_database(const std::filesystem::path& file, const RunCapture& capture);
[[nodiscard]] io::Value::Object write_database(const std::filesystem::path& file,
    const RunCapture& capture, std::size_t maximum_batch_rows,
    const std::function<void(std::string_view stage)>& before_stage = {});
} // namespace bike::recording
