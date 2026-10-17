#pragma once
#include <iosfwd>
#include <span>
#include <string_view>
namespace bike::cli {
int recording_command(std::span<const std::string_view> arguments);
int recording_command(std::span<const std::string_view> arguments, std::ostream& out, std::ostream& error);
} // namespace bike::cli
