#pragma once

// Cold validation boundary. These helpers never enter the realtime loop.
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../io/npy.hpp"
#include "../platform/files.hpp"
#include "../numeric_sum.hpp"
#include "../recording/value_util.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <set>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace bike::validation {
using Value = io::Value;
using Object = Value::Object;
using Array = Value::Array;
namespace value = recording::value;

inline const Value* find(const Value& object, std::string_view key) {
    const auto* members = std::get_if<Object>(&object.data);
    return members ? io::find_member(*members, key) : nullptr;
}
inline const Value& get(const Value& object, std::string_view key) {
    if (const auto* item = find(object, key)) return *item;
    throw std::invalid_argument("validation: missing " + std::string(key));
}
inline Value optional(const Value& object, std::string_view key, const Value& fallback = {}) {
    const auto* item = find(object, key);
    return item ? *item : fallback;
}
inline bool true_value(const Value& item) {
    const auto* flag = std::get_if<bool>(&item.data);
    return flag && *flag;
}
inline bool false_value(const Value& item) {
    const auto* flag = std::get_if<bool>(&item.data);
    return flag && !*flag;
}
inline bool flag(const Value& object, std::string_view key) {
    const auto* item = find(object, key);
    return item && true_value(*item);
}
inline bool checks_passed(const Value& checks) {
    const auto& entries=io::as_object(checks,"checks");
    return std::ranges::all_of(entries,[](const auto& item){return true_value(item.second);});
}
inline double real(const Value& object, std::string_view key) {
    return io::as_double(get(object, key), key);
}
inline double real(const Value& object, std::string_view key, double fallback) {
    const auto* item = find(object, key);
    return item ? io::as_double(*item, key) : fallback;
}
inline std::string text(const Value& object, std::string_view key, std::string_view fallback = {}) {
    const auto* item = find(object, key);
    return item ? io::as_string(*item, key) : std::string(fallback);
}
inline double scalar(double x, std::string_view name) {
    if (!std::isfinite(x)) throw std::invalid_argument(std::string(name) + " must be finite");
    return x;
}
inline double positive(double x, std::string_view name) {
    scalar(x, name);
    if (x <= 0.) throw std::invalid_argument(std::string(name) + " must be positive");
    return x;
}
inline double nonnegative(double x, std::string_view name) {
    scalar(x, name);
    if (x < 0.) throw std::invalid_argument(std::string(name) + " must be nonnegative");
    return x;
}
inline std::string trim(const std::string& s) {
    constexpr std::string_view whitespace = " \t\r\n\v\f";
    const auto first = s.find_first_not_of(whitespace);
    if (first == std::string::npos) return {};
    return s.substr(first, s.find_last_not_of(whitespace) - first + 1);
}
inline std::string identifier(const Value& item, std::string_view name) {
    const auto result = io::as_string(item, name);
    if (trim(result).empty()) throw std::invalid_argument(std::string(name) + " needs a nonempty identifier");
    return result;
}
inline std::set<std::string> keys(const Value& item) {
    std::set<std::string> result;
    for (const auto& [key, unused] : io::as_object(item, "validation")) {
        (void)unused;
        result.insert(key);
    }
    return result;
}
inline std::vector<double> numbers(const Value& item, std::string_view name) {
    std::vector<double> result;
    for (const auto& entry : io::as_array(item, name)) result.push_back(io::as_double(entry, name));
    return result;
}
inline std::vector<std::string> strings(const Value& item, std::string_view name) {
    std::vector<std::string> result;
    for (const auto& entry : io::as_array(item, name)) result.push_back(io::as_string(entry, name));
    return result;
}
inline std::string read_text(const std::filesystem::path& file) {
    // Reuse the opened-object/no-follow and bounded-read contract. The byte
    // reader is format-neutral; JSON/TOML validation belongs to the caller.
    const auto bytes = io::read_numpy_bytes(file);
    if (bytes.empty()) return {};
    return {reinterpret_cast<const char*>(bytes.data()), bytes.size()}; // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast) byte storage is reinterpreted as char for the string contract
}
inline std::string hash_text(std::string_view source) {
    return io::sha256(std::as_bytes(std::span(source.data(), source.size())));
}
inline std::string file_hash(const std::filesystem::path& file) { return platform::file_sha256(file); }
inline Value read_json(const std::filesystem::path& file) { return io::parse_json(read_text(file)); }
void write_report(const std::filesystem::path& file, const Value& report);
void write_report_text(const std::filesystem::path& file, std::string_view serialized);
[[nodiscard]] std::size_t steps(double duration, double dt, double tolerance = 1e-10);
inline double mean(std::span<const double> input) {
    if (input.empty()) throw std::invalid_argument("validation: mean of empty data");
    return numeric::numpy_sum(input) / static_cast<double>(input.size());
}
inline Value pair(double first, double second) { return Value{Array{value::real(first), value::real(second)}}; }
inline Value empty_object() { return Value{Object{}}; }
} // namespace bike::validation
