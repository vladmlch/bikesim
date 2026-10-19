#pragma once
#include "../io/json.hpp"
#include <array>
#include <charconv>
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
    std::array<char,64> bytes{};
    const auto out = std::to_chars(bytes.data(), std::to_address(bytes.end()), x);
    if (out.ec != std::errc{}) throw std::runtime_error("recording: could not format double");
    const auto size = static_cast<std::size_t>(out.ptr - bytes.data());
    std::string lexeme(bytes.data(), size);
    if (lexeme.find_first_of(".eE") == std::string::npos) lexeme += ".0";
    return Value{io::Number{std::move(lexeme)}};
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
