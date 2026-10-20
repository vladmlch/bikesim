// configuration/wire_decode.hpp — the CPython float()/int()/str()
// conversion surfaces shared by the T12 validators. Both the typed-domain
// JSON profile loaders (validate.cpp) and the resolved-wire validator
// (schema.cpp) decode members exactly the way Python's constructors do:
// float() accepts numbers, bools and parseable strings; int() truncates
// floats and parses digit strings; str() spells None/True/False.
#pragma once

#include <algorithm>
#include <array>
#include <charconv>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "../io/json.hpp"
#include "../io/value.hpp"

namespace bike::configuration::detail {

// str.strip() over the Python whitespace set at the UTF-8 boundary — the
// same 29-sequence table nativecfg::nonblank carries, applied both ends.
[[nodiscard]] inline std::string py_strip(std::string_view text) {
    constexpr std::array<std::string_view, 29> spaces{
        "\x09", "\x0a", "\x0b", "\x0c", "\x0d", "\x1c", "\x1d", "\x1e",
        "\x1f", " ", "\xc2\x85", "\xc2\xa0", "\xe1\x9a\x80", "\xe2\x80\x80",
        "\xe2\x80\x81", "\xe2\x80\x82", "\xe2\x80\x83", "\xe2\x80\x84",
        "\xe2\x80\x85", "\xe2\x80\x86", "\xe2\x80\x87", "\xe2\x80\x88",
        "\xe2\x80\x89", "\xe2\x80\x8a", "\xe2\x80\xa8", "\xe2\x80\xa9",
        "\xe2\x80\xaf", "\xe2\x81\x9f", "\xe3\x80\x80"};
    for (;;) {
        const auto found = std::ranges::find_if(
            spaces, [&](std::string_view s) { return text.starts_with(s); });
        if (found == spaces.end()) break;
        text.remove_prefix(found->size());
    }
    for (;;) {
        const auto found = std::ranges::find_if(
            spaces, [&](std::string_view s) { return text.ends_with(s); });
        if (found == spaces.end()) break;
        text.remove_suffix(found->size());
    }
    return std::string(text);
}

// repr() of a str — the single-quote spelling CPython 3.13 emits inside
// f"...{value!r}" and error text (float/int conversion diagnostics).
// Non-ASCII runs are decoded as UTF-8 so that codepoints only printable
// under the Unicode-16.0 tables take 3.13's non-printable escape
// (io/detail::kUnicode16OnlyPrintable); other multi-byte sequences and
// malformed bytes pass through verbatim — the same pass-through the
// pre-existing implementation had for codepoints both versions agree on.
[[nodiscard]] inline std::string py_repr_str(std::string_view text) {
    const bool has_sq = text.find('\'') != std::string_view::npos;
    const bool has_dq = text.find('"') != std::string_view::npos;
    const char quote = (has_sq && !has_dq) ? '"' : '\'';
    std::string out;
    out.push_back(quote);
    for (std::size_t i = 0; i < text.size();) {
        const char ch = text[i];
        const unsigned char u = static_cast<unsigned char>(ch);
        if (u < 0x80) {
            if (ch == '\\')
                out += "\\\\";
            else if (ch == quote)
                out += std::string("\\") + quote;
            else if (ch == '\n')
                out += "\\n";
            else if (ch == '\r')
                out += "\\r";
            else if (ch == '\t')
                out += "\\t";
            else if (u < 0x20 || u == 0x7f) {
                char buf[5];
                std::snprintf(buf, sizeof buf, "\\x%02x", u);
                out += buf;
            } else
                out.push_back(ch);
            ++i;
            continue;
        }
        std::size_t pos = i;
        const char32_t cp = io::detail::utf8_decode_one(text, pos);
        if (cp != io::detail::kInvalidCodepoint) {
            if (io::detail::unicode16_only_printable(cp))
                io::detail::repr_escape_nonprintable(out, cp);
            else
                out.append(text, i, pos - i);
            i = pos;
        } else {
            out.push_back(ch); // malformed UTF-8: no CPython analogue
            ++i;
        }
    }
    out.push_back(quote);
    return out;
}

// type(value).__name__ for the float()/int() "not '...'" suffixes.
[[nodiscard]] inline std::string_view
python_type_name(const bike::io::Value &value) {
    using bike::io::Value;
    if (std::holds_alternative<Value::Object>(value.data)) return "dict";
    if (std::holds_alternative<Value::Array>(value.data)) return "list";
    if (std::holds_alternative<io::Bytes>(value.data)) return "bytes";
    if (std::holds_alternative<io::Tagged>(value.data)) return "set";
    if (std::holds_alternative<std::string>(value.data)) return "str";
    if (std::holds_alternative<std::monostate>(value.data)) return "NoneType";
    if (std::holds_alternative<bool>(value.data)) return "bool";
    return "float";
}

// Python's single-underscore-between-digits literal rule.
[[nodiscard]] inline std::string strip_python_underscores(
    std::string_view text, bool &ok) {
    std::string out;
    out.reserve(text.size());
    ok = true;
    for (std::size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '_') {
            const bool between =
                i > 0 && i + 1 < text.size() &&
                std::isdigit(static_cast<unsigned char>(text[i - 1])) &&
                std::isdigit(static_cast<unsigned char>(text[i + 1]));
            if (!between) {
                ok = false;
                return out;
            }
            continue;
        }
        out.push_back(text[i]);
    }
    return out;
}

// Python float(v) over a decoded JSON member: numbers keep their decimal
// semantics (nonfinite lexemes already rejected by the parser but huge
// magnitudes become +/-inf, matching float()); bool maps to 1.0/0.0; str is
// parsed like CPython (strip, digit underscores, inf/nan spellings, no hex);
// anything else is TypeError — the port throws std::invalid_argument either
// way and keeps the CPython message text.
[[nodiscard]] inline double py_float(const bike::io::Value &value) {
    using bike::io::Value;
    if (const auto *number = std::get_if<io::Number>(&value.data)) {
        bool ok = false;
        const double parsed =
            bike::io::detail::float_from_lexeme(number->lexeme, ok);
        if (!ok)
            throw std::invalid_argument("malformed number lexeme");
        return parsed;
    }
    if (const auto *flag = std::get_if<bool>(&value.data))
        return *flag ? 1. : 0.;
    if (const auto *text = std::get_if<std::string>(&value.data)) {
        const std::string trimmed = py_strip(*text);
        bool underscores_ok = true;
        const std::string candidate =
            strip_python_underscores(trimmed, underscores_ok);
        // strtod accepts C99 '0x..' hex and '0X'; Python float() does not.
        std::string_view probe = candidate;
        if (!probe.empty() && (probe.front() == '+' || probe.front() == '-'))
            probe.remove_prefix(1);
        const bool hex = probe.size() > 2 &&
                         (probe.starts_with("0x") ||
                          probe.starts_with("0X"));
        if (underscores_ok && !candidate.empty() && !hex) {
            // from_chars instead of strtod (-Wunsafe-buffer-usage-in-libc-call):
            // identical grammar for this domain — 'inf'/'infinity'/'nan(...)',
            // '1.'/'.5', exponent signs all parse — except a leading '+' which
            // from_chars rejects, so it is consumed by hand. Range errors map
            // to +/-inf or signed 0 inside float_from_lexeme, exactly like
            // CPython float() overflow/underflow.
            std::string_view view = candidate;
            if (view.front() == '+') view.remove_prefix(1);
            bool consumed = false;
            const double parsed =
                bike::io::detail::float_from_lexeme(view, consumed);
            if (consumed) return parsed;
        }
        throw std::invalid_argument("could not convert string to float: " +
                                    py_repr_str(*text));
    }
    throw std::invalid_argument(
        "float() argument must be a string or a real number, not '" +
        std::string(python_type_name(value)) + "'");
}

// Python int(v): bool -> 0/1, float truncates toward zero (inf ->
// OverflowError 'cannot convert float infinity to integer', nan ->
// ValueError), str parses as a base-10 literal with underscores/whitespace.
// Python ints are arbitrary precision; consumers here compare against
// int-domain bounds, so saturating past int32 keeps every outcome.
[[nodiscard]] inline int py_int(const bike::io::Value &value) {
    using bike::io::Value;
    if (const auto *flag = std::get_if<bool>(&value.data))
        return *flag ? 1 : 0;
    if (const auto *number = std::get_if<io::Number>(&value.data)) {
        bool ok = false;
        const double parsed =
            bike::io::detail::float_from_lexeme(number->lexeme, ok);
        if (!ok)
            throw std::invalid_argument("malformed number lexeme");
        if (std::isinf(parsed))
            throw std::overflow_error(
                "cannot convert float infinity to integer");
        if (std::isnan(parsed))
            throw std::invalid_argument(
                "cannot convert float NaN to integer");
        if (parsed > 2147483647.) return std::numeric_limits<int>::max();
        if (parsed < -2147483648.) return std::numeric_limits<int>::min();
        return static_cast<int>(parsed);
    }
    if (const auto *text = std::get_if<std::string>(&value.data)) {
        const std::string trimmed = py_strip(*text);
        bool underscores_ok = true;
        const std::string candidate =
            strip_python_underscores(trimmed, underscores_ok);
        // string_view indexing — raw digit/end pointers trip
        // -Wunsafe-buffer-usage.
        std::string_view digits = candidate;
        bool negative = false;
        if (!digits.empty() &&
            (digits.front() == '+' || digits.front() == '-')) {
            negative = digits.front() == '-';
            digits.remove_prefix(1);
        }
        if (underscores_ok && !digits.empty() &&
            std::ranges::all_of(digits, [](char c) {
                return std::isdigit(static_cast<unsigned char>(c));
            })) {
            long long v = 0;
            const auto result = std::from_chars(
                digits.data(), std::to_address(digits.cend()), v);
            if (result.ec == std::errc::result_out_of_range)
                v = negative ? static_cast<long long>(
                                   std::numeric_limits<int>::min())
                             : std::numeric_limits<int>::max();
            v = negative ? -v : v;
            if (v >= std::numeric_limits<int>::min() &&
                v <= std::numeric_limits<int>::max())
                return static_cast<int>(v);
            return negative ? std::numeric_limits<int>::min()
                            : std::numeric_limits<int>::max();
        }
        throw std::invalid_argument("invalid literal for int() with base 10: " +
                                    py_repr_str(*text));
    }
    throw std::invalid_argument(
        "int() argument must be a string or a real number, not '" +
        std::string(python_type_name(value)) + "'");
}

// str(v) over a decoded JSON member: strings pass through, numbers keep
// their lexeme, bools spell 'True'/'False', null 'None'; containers take a
// canonical-JSON spelling (a stable provenance text — list/dict repr layout
// is not part of the validated domain).
[[nodiscard]] inline std::string py_str(const bike::io::Value &value) {
    using bike::io::Value;
    if (const auto *s = std::get_if<std::string>(&value.data)) return *s;
    if (const auto *n = std::get_if<io::Number>(&value.data))
        return n->lexeme;
    if (const auto *b = std::get_if<bool>(&value.data))
        return *b ? "True" : "False";
    if (std::holds_alternative<std::monostate>(value.data)) return "None";
    return bike::io::canonical_json(value);
}

// `iter(value)` over a decoded JSON member with Python iteration semantics:
// arrays iterate elements, strings iterate characters (codepoints — at the
// UTF-8 boundary one substring per leading byte group), objects iterate
// keys, and every other kind raises TypeError ("'X' object is not
// iterable" — the generator-inner spelling; direct unpack sites use
// "cannot unpack non-iterable X object").
inline std::vector<io::Value> iterate_items(const io::Value &value) {
    using io::Value;
    if (const auto *items = std::get_if<Value::Array>(&value.data))
        return *items;
    if (const auto *text = std::get_if<std::string>(&value.data)) {
        std::vector<Value> chars;
        for (std::size_t i = 0; i < text->size();) {
            const unsigned char lead =
                static_cast<unsigned char>((*text)[i]);
            const std::size_t len =
                lead < 0x80 ? 1 : lead < 0xE0 ? 2 : lead < 0xF0 ? 3 : 4;
            Value ch;
            ch.data = text->substr(i, len);
            chars.push_back(std::move(ch));
            i += len;
        }
        return chars;
    }
    if (const auto *object = std::get_if<Value::Object>(&value.data)) {
        std::vector<Value> keys;
        keys.reserve(object->size());
        for (const auto &[key, member] : *object) {
            Value k;
            k.data = key;
            keys.push_back(std::move(k));
        }
        return keys;
    }
    throw std::invalid_argument("'" +
                                std::string(python_type_name(value)) +
                                "' object is not iterable");
}

// `a, b = value` — the raw two-item unpack: iterates like `iter()` above
// (but reports scalars as "cannot unpack non-iterable X object"), then the
// cardinality errors ("too many/not enough values to unpack").
inline std::pair<io::Value, io::Value>
unpack2(const io::Value &value) {
    if (!std::holds_alternative<io::Value::Array>(value.data) &&
        !std::holds_alternative<std::string>(value.data) &&
        !std::holds_alternative<io::Value::Object>(value.data))
        throw std::invalid_argument("cannot unpack non-iterable " +
                                    std::string(python_type_name(value)) +
                                    " object");
    const auto items = iterate_items(value);
    if (items.size() < 2)
        throw std::invalid_argument(
            "not enough values to unpack (expected 2, got " +
            std::to_string(items.size()) + ")");
    if (items.size() > 2)
        throw std::invalid_argument(
            "too many values to unpack (expected 2)");
    return {items[0], items[1]};
}

} // namespace bike::configuration::detail
