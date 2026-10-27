#pragma once

// Cold, owned continuation codecs. These helpers never run in a physics tick.
#include "setup_values.hpp"
#include "value_readers.hpp"
#include "samples.hpp"
#include "../io/json.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <type_traits>

namespace runtime::checkpoint_io {
using namespace setupvalue;
inline void exact(const Value& value, std::initializer_list<std::string_view> keys,
                  std::string_view path = "checkpoint") {
    const auto& object = bike::io::as_object(value, path);
    std::set<std::string_view> seen;
    if (object.size() != keys.size()) vread::invalid(path, "wrong field set");
    for (const auto& [name, unused] : object) {
        (void)unused;
        if (!seen.insert(name).second || std::ranges::find(keys, name) == keys.end())
            vread::invalid(path, "duplicate or unknown field");
    }
}
inline Value wire(const Wire& source) {
    return std::visit([](const auto& value) -> Value {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, std::monostate>) return {};
        else if constexpr (std::is_same_v<T, double>) {
            if (std::isfinite(value)) return real(value);
            return Value{bike::io::Tagged{"float64", {{"value", std::isnan(value) ? "nan" : value < 0. ? "-inf" : "+inf"}}}};
        } else if constexpr (std::is_same_v<T, std::int64_t>) return integer(value);
        else if constexpr (std::is_same_v<T, WireArray>) {
            Array out; out.reserve(value.size());
            for (const auto& item : value) out.push_back(wire(item));
            return Value{std::move(out)};
        } else if constexpr (std::is_same_v<T, WireObject>) {
            Object out; out.reserve(value.size());
            for (const auto& [key, item] : value) out.emplace_back(key, wire(item));
            return Value{std::move(out)};
        } else return Value{value};
    }, source.value);
}
inline WireObject object(const Value& value) {
    return sample_wire::object(vread::wire_from_value(value, "checkpoint.object"));
}
inline std::int64_t count(const Value& value) {
    return vread::integer(vread::Node{&value}, "checkpoint.integer");
}
inline double number(const Value& value) { return bike::io::as_double(value, "checkpoint.real"); }
inline bool boolean(const Value& value) { return bike::io::as_bool(value, "checkpoint.boolean"); }
inline std::string string(const Value& value) { return bike::io::as_string(value, "checkpoint.string"); }
inline std::vector<double> numbers(const Value& value, std::optional<std::size_t> width = {}) {
    const auto& entries = bike::io::as_array(value, "checkpoint.vector");
    if (width && entries.size() != *width) vread::invalid("checkpoint.vector", "wrong model-bound width");
    std::vector<double> out; out.reserve(entries.size());
    for (const auto& item : entries) out.push_back(number(item));
    return out;
}
inline Value strings(std::span<const std::string> values) {
    Array out; out.reserve(values.size());
    for (const auto& item : values) out.push_back(text(item));
    return Value{std::move(out)};
}
inline std::vector<std::string> strings(const Value& value) {
    std::vector<std::string> out;
    for (const auto& item : bike::io::as_array(value, "checkpoint.strings")) out.push_back(string(item));
    return out;
}
inline Value optional(std::optional<double> value) { return value ? real(*value) : Value{}; }
inline std::optional<double> optional(const Value& value) {
    return bike::io::is_null(value) ? std::nullopt : std::optional<double>{number(value)};
}
inline Value components(const ForceComponents& values) {
    // Force folds have insertion-order binary64 semantics. JSON objects are
    // canonically key-sorted, so preserve this owner as an ordered row array.
    Array out;out.reserve(values.size());
    for (const auto& [name, row] : values)
        out.push_back(Value{Object{{"name",text(name)},{"values",reals(row)}}});
    return Value{std::move(out)};
}
inline ForceComponents components(const Value& value, std::size_t width) {
    ForceComponents out; std::set<std::string> names;
    for (const auto& entry : bike::io::as_array(value, "checkpoint.forces")) {
        exact(entry,{"name","values"},"checkpoint.forces");
        const auto name=string(field(entry,"name"));
        if (!names.insert(name).second) vread::invalid("checkpoint.forces", "duplicate component");
        out.emplace_back(name, numbers(field(entry,"values"), width));
    }
    return out;
}
} // namespace runtime::checkpoint_io
