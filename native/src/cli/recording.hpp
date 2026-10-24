#pragma once
#include <iosfwd>
#include <span>
#include <string_view>
namespace bike::cli {
// The same parser as recording_command, without opening input or output.
void validate_recording_arguments(std::span<const std::string_view> arguments);
int recording_command(std::span<const std::string_view> arguments);
int recording_command(std::span<const std::string_view> arguments, std::ostream& out, std::ostream& error);
} // namespace bike::cli
