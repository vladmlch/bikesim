#pragma once
#include "../io/json.hpp"
#include "../io/float_format.hpp"
#include "../io/npy.hpp"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace bike::recording::detail {
using Value = io::Value;
using Object = Value::Object;
using Array = Value::Array;
inline Value text(std::string_view value) { return Value{std::string(value)}; }
inline Value integer(std::int64_t value) { return Value{io::Number{std::to_string(value)}}; }
inline Value real(double value) {
    if (!std::isfinite(value))
        return Value{io::Tagged{"float64", {{"value", std::isnan(value) ? "nan" : value < 0. ? "-inf" : "+inf"}}}};
    return Value{io::Number{io::repr_binary64(value)}};
}
inline const Value* find(const Object& object, std::string_view name) { return io::find_member(object, name); }
inline const Value* find(const Value& value, std::string_view name) {
    return find(io::as_object(value, "recording"), name);
}
inline const Value& field(const Object& object, std::string_view name) {
    if (const auto* value = find(object, name)) return *value;
    throw std::invalid_argument("recording: missing field '" + std::string(name) + "'");
}
inline const Value& field(const Value& value, std::string_view name) { return field(io::as_object(value, "recording"), name); }
inline Value maybe(const Value& value, std::string_view name) {
    const auto* child = find(value, name); return child ? *child : Value{};
}
inline void set(Object& object, std::string_view name, Value value) {
    for (auto& [key, child] : object) if (key == name) { child = std::move(value); return; }
    object.emplace_back(name, std::move(value));
}
inline void set(Value& object, std::string_view name, Value value) { set(std::get<Object>(object.data), name, std::move(value)); }
inline Value pop(Object& object, std::string_view name) {
    for (auto it = object.begin(); it != object.end(); ++it)
        if (it->first == name) { auto value = std::move(it->second); object.erase(it); return value; }
    throw std::invalid_argument("recording: missing field '" + std::string(name) + "'");
}
inline bool equal(const Value& left, const Value& right) { return io::recording_json(left) == io::recording_json(right); }
inline std::string string(const Value& value) { return io::as_string(value, "recording"); }
inline std::int64_t count(const Value& value) { return io::as_int64(value, "recording count"); }
inline std::string quote(std::string_view name) {
    if (name.empty() || name.find('\0') != std::string_view::npos) throw std::invalid_argument("invalid SQL identifier");
    std::string out = "\"";
    for (char c : name) { out += c; if (c == '"') out += '"'; }
    return out + '"';
}
inline std::string literal(std::string_view value) {
    if (value.find('\0') != std::string_view::npos) throw std::invalid_argument("SQL value contains NUL");
    std::string out = "'";
    for (char c : value) { out += c; if (c == '\'') out += '\''; }
    return out + '\'';
}
inline void limit(std::uint64_t value) {
    // Exact signed SQL bounds, including the mandatory look-ahead row.
    if (value == 0 || value >= static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
        throw std::invalid_argument("limit must be a positive integer below INT64_MAX");
}
inline Array values(std::vector<Object> rows) {
    Array out; out.reserve(rows.size());
    for (auto& row : rows) out.push_back(Value{std::move(row)});
    return out;
}
inline Value bounded(std::vector<Object> rows, std::uint64_t maximum, std::optional<std::int64_t> total = {}) {
    limit(maximum); const bool more = rows.size() > maximum;
    if (more) rows.resize(static_cast<std::size_t>(maximum));
    Object out{{"rows", Value{values(std::move(rows))}}, {"limit", integer(static_cast<std::int64_t>(maximum))},
        {"truncated", Value{more}}, {"has_more", Value{more}}};
    if (total) out.emplace_back("total", integer(*total));
    return Value{std::move(out)};
}
inline std::string read_text(const std::filesystem::path& path) {
    auto bytes = io::read_numpy_bytes(path);
    return std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}
inline std::string pretty_json(const Value& value) {
    const auto compact = io::recording_json(value);
    std::string out; std::size_t depth = 0; bool quoted = false, escaped = false;
    for (std::size_t i = 0; i < compact.size(); ++i) {
        const char c = compact[i];
        if (quoted) {
            out += c;
            if (escaped) escaped = false;
            else if (c == '\\') escaped = true;
            else if (c == '"') quoted = false;
            continue;
        }
        if (c == '"') { quoted = true; out += c; }
        else if (c == '{' || c == '[') {
            out += c; ++depth;
            if (i + 1 < compact.size() && compact[i + 1] != '}' && compact[i + 1] != ']')
                out += '\n' + std::string(depth * 2, ' ');
        } else if (c == '}' || c == ']') {
            --depth;
            if (i && compact[i - 1] != '{' && compact[i - 1] != '[') out += '\n' + std::string(depth * 2, ' ');
            out += c;
        } else if (c == ',') out += ",\n" + std::string(depth * 2, ' ');
        else if (c == ':') out += ": ";
        else out += c;
    }
    return out;
}
} // namespace bike::recording::detail
