#pragma once
#include "input.hpp"
#include "../recording/analysis_detail.hpp"
#include "../recording/reader.hpp"
#include "../io/hash.hpp"
#include <array>
#include <regex>
#include <set>
namespace bike::replay::detail {
using namespace recording::detail;
inline std::string asset_text(const ReplayInput& input, const std::string& name) {
    const auto it = input.assets.find(name);
    if (it == input.assets.end()) throw recording::RecordingError("missing replay asset: " + name);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) byte-to-char aliasing is the designed idiom for raw file bytes
    return {reinterpret_cast<const char*>(it->second.data()), it->second.size()};
}
inline std::set<std::string> keys(const Value& value) {
    std::set<std::string> out;
    for (const auto& [name, ignored] : io::as_object(value, "replay")) { (void)ignored; out.insert(name); }
    return out;
}
inline void exact_keys(const Value& value, const std::set<std::string>& expected, std::string_view path) {
    if (keys(value) != expected) throw recording::RecordingError(std::string(path) + ": unexpected fields");
}
inline void digest(const Value& value, std::string_view label) {
    if (!std::regex_match(string(value), std::regex("[0-9a-f]{64}"))) throw recording::RecordingError(std::string(label) + " must be a lowercase SHA-256 digest");
}
inline std::vector<double> double_vector(const Value& value, std::string_view label) {
    std::vector<double> out;
    for (const auto& item : io::as_array(value, label)) out.push_back(io::as_double(item, label));
    return out;
}
inline void command_shapes(const ReplayInput& input) {
    if (input.requested.size() != input.transitions.size() || input.observations.size() != input.requested.size() + 1)
        throw recording::RecordingError("inconsistent recorded command/transition counts");
    for (const auto& command : input.requested) {
        exact_keys(command, {"step", "time_s", "control", "front_brake_demand", "rear_brake_demand"}, "requested command");
        if (count(field(command, "step")) < 0) throw recording::RecordingError("negative requested command step");
        (void)io::as_object(field(command, "control"), "command.control");
        (void)io::as_double(field(command, "time_s"), "command.time_s");
    }
}
inline std::vector<Value> json_lines(std::string_view source) {
    std::vector<Value> result;
    for (std::size_t begin = 0; begin < source.size();) {
        const auto end = source.find('\n', begin);
        const auto line = source.substr(begin, end == std::string_view::npos ? end : end - begin);
        if (line.find_first_not_of(" \t\r") != std::string_view::npos) result.push_back(io::parse_json(line));
        if (end == std::string_view::npos) break;
        begin = end + 1;
    }
    return result;
}
} // namespace bike::replay::detail
