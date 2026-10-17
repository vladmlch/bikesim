#pragma once

#include <algorithm>
#include <cstdint>
#include <initializer_list>
#include <span>
#include <string>
#include <string_view>
#include "../configuration/fields.hpp"
#include "../io/value.hpp"

// Small cold-path constructors shared by projection and bootstrap capture.
// Typed physics remains in the existing writers, never in these IO values.
namespace runtime::setupvalue {
using Value = bike::io::Value;
using Object = Value::Object;
using Array = Value::Array;
inline Value real(double x) {
    return Value{bike::io::Number{bike::configuration::detail::repr_double(x)}};
}
inline Value integer(std::int64_t x) { return Value{bike::io::Number{std::to_string(x)}}; }
inline Value text(std::string_view x) { return Value{std::string(x)}; }
inline Value reals(std::span<const double> values) {
    Array out;
    out.reserve(values.size());
    for (double x : values) out.push_back(real(x));
    return Value{std::move(out)};
}
inline const Value* find(const Value& value, std::string_view key) {
    for (const auto& [name, child] : bike::io::as_object(value, "setup"))
        if (name == key) return &child;
    return nullptr;
}
inline const Value& field(const Value& value, std::string_view key) {
    if (const auto* child = find(value, key)) return *child;
    throw std::invalid_argument("setup: missing field '" + std::string(key) + "'");
}
inline void set(Value& value, std::string_view key, Value child) {
    auto* object = std::get_if<Object>(&value.data);
    if (object == nullptr) throw std::invalid_argument("setup: expected object");
    for (auto& [name, current] : *object) {
        if (name == key) { current = std::move(child); return; }
    }
    object->emplace_back(key, std::move(child));
}
inline Value select(const Value& value, std::initializer_list<std::string_view> keys) {
    Object out;
    out.reserve(keys.size());
    for (auto key : keys) out.emplace_back(key, field(value, key));
    return Value{std::move(out)};
}
} // namespace runtime::setupvalue
