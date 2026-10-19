#pragma once
#include "../io/value.hpp"
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace bike::cli {
class ArgumentError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
    ~ArgumentError() override;
    ArgumentError(const ArgumentError&) = default;
    ArgumentError& operator=(const ArgumentError&) = default;
    ArgumentError(ArgumentError&&) = default;
    ArgumentError& operator=(ArgumentError&&) = default;
};
struct OptionSpec {
    std::vector<std::string> names, choices;
    std::string dest, type;
    std::optional<std::string> default_value;
    bool flag = false, required = false, positional = false;
};
struct Arguments {
    std::string command;
    std::vector<std::pair<std::string, std::string>> values;
    std::vector<std::string> supplied;
    bool help = false;
    [[nodiscard]] bool present(std::string_view key) const;
    [[nodiscard]] bool has(std::string_view key) const;
    [[nodiscard]] bool flag(std::string_view key) const;
    [[nodiscard]] std::string text(std::string_view key, const std::string& fallback = {}) const;
    [[nodiscard]] double real(std::string_view key, double fallback = 0.) const;
    [[nodiscard]] std::int64_t integer(std::string_view key, std::int64_t fallback = 0) const;
    [[nodiscard]] std::optional<std::string> optional_text(std::string_view key) const;
    [[nodiscard]] std::optional<double> optional_real(std::string_view key) const;
    void set(std::string key, std::string value, bool explicit_value = false);
};
[[nodiscard]] std::vector<OptionSpec> option_specs(std::string_view command);
[[nodiscard]] Arguments parse_arguments(std::string_view command, std::span<const std::string_view> arguments);
[[nodiscard]] std::string help_text(std::string_view command);
[[nodiscard]] std::vector<std::string> command_names();
} // namespace bike::cli
