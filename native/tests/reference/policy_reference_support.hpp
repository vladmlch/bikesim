#pragma once
#include "drive_reference_support.hpp"
#include "support/golden.hpp"
#include "drivetrain/policy_set.hpp"
#include "drivetrain/state_codec.hpp"
#include "io/json.hpp"
#include "random/numpy_compat.hpp"
#include "runtime/setup_values.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace policy_reference {
using bike::io::Value;
using namespace runtime::setupvalue;
using drivetrain::PolicySet;

inline bike::configuration::PhysicalDriveConfig source(bool fixture = true) {
    bike::configuration::PhysicalDriveConfig result;
    result.transmission_model = "ideal_mid_drive";
    if (fixture) {
        result.pedaling.enabled = true; result.pedaling.coast_cadence_tau_s = .03;
        result.shifting.enabled = true; result.shifting.cadence_smoothing_tau_s = 0.;
    }
    return result;
}
inline Value config(const bike::configuration::PhysicalDriveConfig& input) {
    const auto writer = drive_reference::projected_writer(input);
    return select(field(writer, "drive"), {"gearing", "pedaling", "shifting", "assist", "battery",
                                          "hub_stiffness_nm_rad", "hub_damping_nm_s"});
}
inline PolicySet policies(const Value& value) { return PolicySet(nativecfg::decode_policy_config(value)); }
inline Value state(const PolicySet& policy) { return drivetrain::encode_policy_snapshot(policy.snapshot()); }
inline void restore(PolicySet& policy, const Value& value) { policy.restore(drivetrain::decode_policy_snapshot(value)); }
inline std::string state_text(const PolicySet& policy) { return bike::io::canonical_json(state(policy)); }

inline Value& member(Value& value, std::string_view key) {
    auto* object = std::get_if<Value::Object>(&value.data);
    if (object == nullptr) throw std::invalid_argument("test document needs an object");
    for (auto& [name, child] : *object) if (name == key) return child;
    throw std::invalid_argument("missing test document member " + std::string(key));
}
inline void erase(Value& value, std::string_view key) {
    auto* object = std::get_if<Value::Object>(&value.data);
    if (object == nullptr) throw std::invalid_argument("test document needs an object");
    const auto removed = std::erase_if(*object, [&](const auto& item) { return item.first == key; });
    bt::require(removed == 1, "test mutation removes an existing member");
}
inline void put(Value& value, std::string_view section, std::string_view key, Value child) {
    set(member(value, section), key, std::move(child));
}
inline std::vector<std::string> sorted_keys(const Value& value) {
    std::vector<std::string> result;
    for (const auto& [key, ignored] : bike::io::as_object(value, "tree")) { (void)ignored; result.push_back(key); }
    std::ranges::sort(result);
    return result;
}

// Mirrors the source helper's stable, labelled capture traversal. Every
// discrete leaf is compact JSON UTF-8 bytes; every double retains raw bits.
inline void json_bytes(bt::oracle::Node& node, const Value& value, std::string_view label) {
    const auto bytes = bike::io::canonical_json(value);
    const std::array shape{bytes.size()};
    node.bytes(std::as_bytes(std::span(bytes.data(), bytes.size())), "|u1", shape, label);
}
inline void keys(bt::oracle::Node& node, const Value& value, const std::string& path) {
    Value::Array names; for (const auto& key : sorted_keys(value)) names.push_back(text(key));
    json_bytes(node, Value{std::move(names)}, path + "#keys");
}
inline void length(bt::oracle::Node& node, std::size_t count, const std::string& path) {
    json_bytes(node, Value{bike::io::Number{std::to_string(count)}}, path + "#length");
}
// NOLINTNEXTLINE(misc-no-recursion) recursive JSON value tree walk
inline void tree(bt::oracle::Node& node, const Value& value, const std::string& path = "$") {
    if (std::holds_alternative<Value::Object>(value.data)) {
        keys(node, value, path);
        for (const auto& key : sorted_keys(value)) { std::string next(path); next += '.'; next += key; tree(node, field(value, key), next); }
    } else if (const auto* array = std::get_if<Value::Array>(&value.data)) {
        length(node, array->size(), path);
        for (std::size_t i = 0; i < array->size(); ++i) tree(node, (*array)[i], path + "[" + std::to_string(i) + "]");
    } else if (const auto* number = std::get_if<bike::io::Number>(&value.data);
               number && !bike::io::is_integer_lexeme(number->lexeme)) {
        const std::array values{bike::io::as_double(value, path)};
        node.f64(values, std::span<const std::size_t>{}, path);
    } else {
        json_bytes(node, value, path);
    }
}
inline void scalar(bt::oracle::Node& node, double value, std::string_view label = "") {
    const std::array values{value}; node.f64(values, std::span<const std::size_t>{}, label);
}
inline void pair(bt::oracle::Node& node, std::pair<double, double> value) {
    const std::array values{value.first, value.second}; const std::array<std::size_t, 1> shape{2};
    node.f64(values, shape, "");
}
inline void shifted(bt::oracle::Node& node, bool value) {
    const std::array bytes{static_cast<std::byte>(value ? 1U : 0U)};
    node.bytes(bytes, "|b1", std::span<const std::size_t>{}, "shifted");
}
inline Value pedaling_result(const drivetrain::PedalingState& value) {
    return Value{Value::Object{{"mode", text(drivetrain::pedal_mode_name(value.mode))},
        {"reason", text(drivetrain::coast_reason_name(value.reason))}, {"effort_nm", real(value.effort_nm)},
        {"required_cadence_rpm", real(value.required_cadence_rpm)},
        {"target_phase_rad", value.target_phase_rad ? real(*value.target_phase_rad) : Value{}},
        {"target_rate_rad_s", real(value.target_rate_rad_s)}}};
}
inline Value shifting_result(const PolicySet& value) {
    auto result = field(state(value), "shifting");
    set(result, "gear_ratio", real(value.shifting.gear_ratio()));
    set(result, "torque_factor", real(value.shifting.torque_factor()));
    return result;
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) mirrors the Python oracle-node helper signature
inline bt::oracle::Node node(std::string_view function, std::string_view parameter = {}) {
    auto name = "tests/reference/test_native_drive_policies.py::" + std::string(function);
    if (!parameter.empty()) name += "[" + std::string(parameter) + "]";
    return bt::oracle::Node(name);
}
inline double electrical(const PolicySet& policy, double torque, double omega, bool enabled) {
    const auto& b = policy.config.battery;
    return drivetrain::motor_electrical_power(torque, omega, b.copper_w_per_nm2, b.speed_w_per_rad_s2, b.idle_w, enabled);
}
inline double energy_limit(const PolicySet& policy, double torque, double omega, double budget) {
    const auto& b = policy.config.battery;
    return drivetrain::limit_torque_by_energy(torque, omega, b.copper_w_per_nm2, b.speed_w_per_rad_s2, b.idle_w, budget);
}
template<class F> void overflow(F&& action) {
    bool rejected = false;
    try { std::forward<F>(action)(); } catch (const std::overflow_error&) { rejected = true; }
    bt::require(rejected, "unrepresentable derived value throws overflow_error");
}
} // namespace policy_reference
