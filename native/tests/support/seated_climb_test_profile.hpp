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
    std::string result(toml);
    const std::size_t start = toml.find(header);
    std::size_t end = 0;
    std::string_view section;
    if (start == std::string_view::npos) {
        if (!result.empty() && result.back() != '\n') result.push_back('\n');
        result += header;
        result.push_back('\n');
        end = result.size();
    } else {
        const std::size_t body = start + header.size();
        const std::size_t next = toml.find("\n[", body);
        end = next == std::string_view::npos ? toml.size() : next;
        section = toml.substr(body, end - body);
    }
    static constexpr std::array<std::pair<std::string_view,
                                          std::string_view>, 23> defaults{{
        {"enabled", "false"},
        {"period_s", "0.00125"},
        {"reaction_delay_s", "0.15"},
        {"target_crank_power_w", "225.0"},
        {"max_crank_torque_nm", "60.0"},
        {"lean_gain", "1.0"},
        {"max_forward_lean_rad", "0.55"},
        {"max_backward_lean_rad", "0.1"},
        {"lean_rate_rad_s", "0.5"},
        {"orientation_tau_s", "0.5"},
        {"surge_power_w", "400.0"},
        {"surge_grade", "0.2"},
        {"surge_budget_s", "15.0"},
        {"surge_recovery_rate", "0.3333333333333333"},
        {"front_load_share_target", "0.3"},
        {"lean_trim_gain_rad_s", "0.3"},
        {"lean_trim_limit_rad", "0.15"},
        {"trim_dead_time_s", "0.2"},
        {"min_elbow_flexion_deg", "15.0"},
        {"climb_effort_rise_speed_mps", "2.5"},
        {"climb_effort_full_speed_mps", "4.0"},
        {"climb_effort_cap_grades", "[0.0]"},
        {"climb_effort_cap_grade_nm", "[1000.0]"},
    }};
    const auto has_assignment = [](std::string_view text,
                                   std::string_view key) {
        while (!text.empty()) {
            const std::size_t newline = text.find('\n');
            const std::string_view line =
                text.substr(0, newline == std::string_view::npos
                                   ? text.size()
                                   : newline);
            const std::size_t first = line.find_first_not_of(" \t\r");
            if (first != std::string_view::npos &&
                line.substr(first, key.size()) == key) {
                std::size_t equals = first + key.size();
                while (equals < line.size() &&
                       (line[equals] == ' ' || line[equals] == '\t' ||
                        line[equals] == '\r'))
                    ++equals;
                if (equals < line.size() && line[equals] == '=') return true;
            }
            if (newline == std::string_view::npos) break;
            text.remove_prefix(newline + 1);
        }
        return false;
    };
    std::string additions;
    for (const auto &[key, value] : defaults) {
        if (!has_assignment(section, key))
            additions += std::string(key) + " = " + std::string(value) + "\n";
    }
    if (additions.empty()) return result;
    if (end != result.size() || result.empty() || result.back() != '\n')
        additions.insert(additions.begin(), '\n');
    result.insert(end, additions);
    return result;
}

} // namespace bt
