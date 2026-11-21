#pragma once

#include <array>
#include <string>
#include <string_view>
#include <utility>

namespace bt {

// Keep legacy inline test profiles valid when they exercise unrelated
// behavior. Tests of missing seated_climb keys must stage their raw TOML.
[[nodiscard]] inline std::string complete_seated_climb_test_profile(
    std::string_view toml) {
    constexpr std::string_view header = "[seated_climb]";
    const std::size_t start = toml.find(header);
    if (start == std::string_view::npos) return std::string(toml);
    const std::size_t body = start + header.size();
    const std::size_t next = toml.find("\n[", body);
    const std::size_t end = next == std::string_view::npos ? toml.size() : next;
    const std::string_view section = toml.substr(body, end - body);
    static constexpr std::array<std::pair<std::string_view,
                                          std::string_view>, 10> defaults{{
        {"enabled", "false"},
        {"target_crank_power_w", "225.0"},
        {"lean_gain", "1.0"},
        {"max_backward_lean_rad", "0.1"},
        {"surge_power_w", "400.0"},
        {"surge_grade", "0.2"},
        {"surge_recovery_rate", "0.3333333333333333"},
        {"front_load_share_target", "0.3"},
        {"lean_trim_gain_rad_s", "0.3"},
        {"lean_trim_limit_rad", "0.15"},
    }};
    std::string additions;
    for (const auto &[key, value] : defaults) {
        const std::string assignment = std::string(key) + " = ";
        if (section.find(assignment) == std::string_view::npos)
            additions += assignment + std::string(value) + "\n";
    }
    if (additions.empty()) return std::string(toml);
    std::string result(toml);
    if (end == result.size()) {
        if (result.empty() || result.back() != '\n')
            additions.insert(additions.begin(), '\n');
    } else {
        additions.insert(additions.begin(), '\n');
    }
    result.insert(end, additions);
    return result;
}

} // namespace bt
