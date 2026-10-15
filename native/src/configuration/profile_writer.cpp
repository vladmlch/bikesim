// configuration/profile_writer.cpp — plant_profile.py write_physics_toml,
// the document half. asdict-equivalent tree -> TOML text: the banner
// comment, bare-key [dotted] tables in declaration order, scalars first and
// subtables after within each section, json.dumps(ensure_ascii=True,
// allow_nan=False) scalar spellings. The source's exclusive non-overwriting
// file step is a caller concern (platform::write_exclusive carries the same
// contract).

#include "configuration/load.hpp"

#include <array>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace bike::configuration {
namespace {

using bike::io::Number;
using bike::io::Value;

// str.isidentifier() restricted to the ASCII identifier domain — every
// reachable key is a declared snake_case field name; a non-ASCII or leading
// digit key is 'invalid configuration key' in the source too (Python's
// broader Unicode identifiers cannot occur inside a config document).
bool is_identifier(std::string_view key) {
    if (key.empty()) return false;
    const auto lead = [](char c) {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
    };
    if (!lead(key.front())) return false;
    for (const char c : key)
        if (!lead(c) && !(c >= '0' && c <= '9')) return false;
    return true;
}

void hex4(std::string& out, unsigned value) {
    constexpr std::array<char, 16> digits{'0', '1', '2', '3', '4', '5',
                                          '6', '7', '8', '9', 'a', 'b',
                                          'c', 'd', 'e', 'f'};
    out += "\\u";
    out.push_back(digits.at((value >> 12U) & 0xfU));
    out.push_back(digits.at((value >> 8U) & 0xfU));
    out.push_back(digits.at((value >> 4U) & 0xfU));
    out.push_back(digits.at(value & 0xfU));
}

// json.dumps(ensure_ascii=True) string body: escapes ", \, the five
// shorthand controls, \u00XX below 0x20, and every non-ASCII code point —
// \uXXXX inside the BMP, surrogate pairs beyond it (the source spelling,
// byte for byte). Invalid UTF-8 tails escape per raw byte — unreachable
// over the decoded input domain.
std::string json_body(std::string_view text) {
    std::string out;
    out.reserve(text.size() + 2);
    for (std::size_t i = 0; i < text.size();) {
        const unsigned char c = static_cast<unsigned char>(text[i]);
        if (c < 0x80) {
            ++i;
            switch (c) {
            case '"': out += "\\\""; continue;
            case '\\': out += "\\\\"; continue;
            case '\b': out += "\\b"; continue;
            case '\f': out += "\\f"; continue;
            case '\n': out += "\\n"; continue;
            case '\r': out += "\\r"; continue;
            case '\t': out += "\\t"; continue;
            default:
                if (c < 0x20)
                    hex4(out, c);
                else
                    out.push_back(static_cast<char>(c));
                continue;
            }
        }
        // UTF-8 continuation: 2-, 3- and 4-byte sequences.
        unsigned code = 0;
        std::size_t len = 0;
        if (c >= 0xc2 && c < 0xe0) {
            code = static_cast<unsigned>(c) & 0x1fU;
            len = 2;
        } else if (c >= 0xe0 && c < 0xf0) {
            code = static_cast<unsigned>(c) & 0x0fU;
            len = 3;
        } else if (c >= 0xf0 && c < 0xf5) {
            code = static_cast<unsigned>(c) & 0x07U;
            len = 4;
        }
        bool valid = len != 0 && i + len <= text.size();
        for (std::size_t k = 1; valid && k < len; ++k) {
            const unsigned t =
                static_cast<unsigned char>(text[i + k]);
            if ((t & 0xc0U) != 0x80U) {
                valid = false;
            } else {
                code = (code << 6U) | (t & 0x3fU);
            }
        }
        // Reject overlongs/surrogates/out-of-range the same way a strict
        // decoder does; anything odd degrades to per-byte escaping.
        if (valid &&
            (code < (len == 2 ? 0x80U : len == 3 ? 0x800U : 0x10000U) ||
             (code >= 0xd800U && code <= 0xdfffU) || code > 0x10ffffU))
            valid = false;
        if (!valid) {
            hex4(out, c);
            ++i;
            continue;
        }
        i += len;
        if (code <= 0xffffU) {
            hex4(out, code);
        } else {
            const unsigned hi = 0xd800U + ((code - 0x10000U) >> 10U);
            const unsigned lo = 0xdc00U + ((code - 0x10000U) & 0x3ffU);
            hex4(out, hi);
            hex4(out, lo);
        }
    }
    return out;
}

[[noreturn]] void writer_error(const std::string& message) {
    throw std::invalid_argument(message);
}

// plant_profile.py _toml_value.
// NOLINTNEXTLINE(misc-no-recursion) nested arrays recurse through elements
std::string toml_value(const Value& value) {
    if (const auto* items = std::get_if<Value::Array>(&value.data)) {
        std::string out = "[";
        for (std::size_t i = 0; i < items->size(); ++i) {
            if (i != 0) out += ", ";
            out += toml_value(items->at(i));
        }
        out += ']';
        return out;
    }
    if (const auto* text = std::get_if<std::string>(&value.data))
        return '"' + json_body(*text) + '"';
    if (const auto* flag = std::get_if<bool>(&value.data))
        return *flag ? "true" : "false";
    if (const auto* number = std::get_if<Number>(&value.data)) {
        // json.dumps(allow_nan=False) on a finite repr spelling; the
        // {"$nonfinite": ...} decode feeds back the spelling — a nonfinite
        // member fails with the source's ValueError text.
        bool ok = false;
        const double parsed = bike::io::detail::float_from_lexeme(
            number->lexeme, ok);
        if (!ok || !std::isfinite(parsed))
            writer_error(
                "Out of range float values are not JSON compliant");
        return number->lexeme;
    }
    writer_error("unsupported physics configuration scalar");
}

// NOLINTNEXTLINE(misc-no-recursion) mirrors the asdict nesting
void emit_table(const Value::Object& obj, const std::string& prefix,
                std::vector<std::string>& lines) {
    if (!prefix.empty()) {
        lines.emplace_back();
        std::string heading("[");
        heading += prefix;
        heading += ']';
        lines.push_back(std::move(heading));
    }
    // Scalars and null-skipping in member order, then subtable sections —
    // the source's two-pass table().
    for (const auto& [key, value] : obj) {
        if (!is_identifier(key)) writer_error("invalid configuration key");
        if (!bike::io::is_null(value) &&
            !std::holds_alternative<Value::Object>(value.data)) {
            std::string line(key);
            line += " = ";
            line += toml_value(value);
            lines.push_back(std::move(line));
        }
    }
    for (const auto& [key, value] : obj) {
        if (const auto* nested =
                std::get_if<Value::Object>(&value.data)) {
            std::string next(prefix);
            if (!next.empty()) next += '.';
            next += key;
            emit_table(*nested, next, lines);
        }
    }
}

} // namespace

std::string write_physics_toml(const SimulationPhysicsConfig& config) {
    std::vector<std::string> lines{
        "# Explicit candidate; not real-world safety validation."};
    const Value document =
        detail::plain_document(detail::as_object(
            detail::kSimulationPhysicsConfigSchema, &config));
    emit_table(std::get<Value::Object>(document.data), "", lines);
    std::string out;
    for (const std::string& line : lines) out += line + '\n';
    return out;
}

} // namespace bike::configuration
