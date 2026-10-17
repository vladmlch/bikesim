#pragma once
#include "reader.hpp"
#include <filesystem>
#include <string>
namespace bike::recording {
[[nodiscard]] std::string render_summary(const Recording&);
// Used only by final publication after the DB has committed/checkpointed.
// This does not relax the manifest requirement on any analyst entry point.
[[nodiscard]] std::string render_report_database(const std::filesystem::path& database);
} // namespace bike::recording
