#pragma once
#include "../io/float_format.hpp"
#include "../io/json.hpp"
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <span>

namespace bike::recording::value {
using io::Value;
inline Value integer(std::int64_t n) { return Value{io::Number{std::to_string(n)}}; }
inline Value text(std::string_view s) { return Value{std::string(s)}; }
inline Value real(double x) {
    if (!std::isfinite(x)) return Value{io::Tagged{.kind="float64", .fields={{"value", std::isnan(x) ? "nan" : x < 0. ? "-inf" : "+inf"}}}};
    // Python repr()/json.dumps spelling: shortest round-trip digits stay
    // fixed for -4 <= e < 16 and integral values retain ".0". A raw
    // to_chars general pass instead shortens e.g. 0.0001 to "1e-04" and
    // 1e15 to "1e+15", diverging from the oracle's canonical JSON hash.
    return Value{io::Number{io::repr_binary64(x)}};
}
inline Value strings(std::span<const std::string> input) {
    Value::Array out; out.reserve(input.size());
    for (const auto& s : input) out.push_back(text(s));
    return Value{std::move(out)};
}
inline Value doubles(std::span<const double> input) {
    Value::Array out; out.reserve(input.size());
    for (double const n : input) out.push_back(real(n));
    return Value{std::move(out)};
}
inline const Value& require(const Value::Object& object, std::string_view key) {
    if (const auto* v = io::find_member(object, key)) return *v;
    throw std::invalid_argument("recording: missing " + std::string(key));
}
inline const Value& require(const Value& object, std::string_view key) {
    return require(io::as_object(object, "recording"), key);
}
inline void set(Value::Object& object, std::string key, Value v) {
    for (auto& [k, old] : object) if (k == key) { old = std::move(v); return; }
    object.emplace_back(std::move(key), std::move(v));
}
inline std::string join(std::span<const std::string> path, std::string_view sep = ".") {
    std::string out;
    for (const auto& part : path) { if (!out.empty()) out += sep; out += part; }
    return out;
}
} // namespace bike::recording::value
