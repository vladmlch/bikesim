// value.hpp — the owned cold-path value tree (task T05;
// migration/interface-contracts.md §2 verbatim shape). `Value` is a
// lossless JSON/recording AST: numbers stay validated lexemes so integer
// precision and integer-versus-float spelling survive; `Bytes` and
// `Tagged` carry the decoded bike_recording.json.v1 tag payloads. This is
// the cold input/output boundary — the hot path keeps the existing
// runtime::WireValue/WireObject types.
//
// Domain mapping of the decoded recording codec:
//   * JSON null          -> std::monostate (an *explicit* null member; a
//                           missing member is an absent Object entry, not a
//                           monostate entry)
//   * true/false         -> bool (boolean never travels through Number)
//   * number token       -> Number{exact lexeme}; integers of arbitrary
//                           magnitude and float spellings such as "-0.0"
//                           are preserved verbatim
//   * string             -> std::string (UTF-8)
//   * array/object       -> Array / Object (Object is insertion-ordered;
//                           duplicate keys are rejected during parsing)
//   * tagged bytes       -> Bytes{decoded payload}
//   * tagged float64/decimal/date/time/datetime/timedelta
//                        -> Tagged{kind, fields}; fields preserve the
//                           recorded lexemes ("value", or
//                           days/seconds/microseconds as normalized
//                           decimal integers)
//   * escaped object     -> Object (the $bike_recording escape is a
//                           wire-level form; a decoded escaped object is an
//                           ordinary Object that may itself contain the
//                           reserved key)
#pragma once

#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

namespace bike::io {

struct Number {
    std::string lexeme;
};
struct Bytes {
    std::vector<std::byte> value;
};
struct Tagged {
    using Field = std::pair<std::string, std::string>;
    std::string kind;
    std::vector<Field> fields;
};
struct Value {
    using Array = std::vector<Value>;
    using Object = std::vector<std::pair<std::string, Value>>;
    std::variant<std::monostate, bool, Number, std::string, Array, Object,
                 Bytes, Tagged>
        data;
};

// ---- typed extraction -----------------------------------------------------
// Cold-path accessors: every check runs before conversion; a kind mismatch,
// an out-of-range integer or a nonfinite double is std::invalid_argument
// naming the API and the complete field path (shared exception class).
// `field` is the caller-supplied public path, e.g. "config.track.length_m".

inline bool is_null(const Value &value) noexcept {
    return std::holds_alternative<std::monostate>(value.data);
}

// Member lookup that keeps absence distinct from explicit null: returns a
// pointer to the stored entry or nullptr when the member is absent.
inline const Value *find_member(const Value::Object &object,
                                std::string_view key) noexcept {
    for (const auto &member: object)
        if (member.first == key) return &member.second;
    return nullptr;
}

namespace detail {
    [[noreturn]] inline void kind_error(std::string_view api,
                                        std::string_view field,
                                        std::string_view expected) {
        throw std::invalid_argument(
            std::string(api) + ": " + std::string(field) + ": expected " +
            std::string(expected));
    }

    // Direction probe for a std::from_chars range error on a JSON number
    // lexeme: the standard leaves `value` unspecified on
    // result_out_of_range, so the magnitude is recovered from the lexeme —
    // true iff |value| < 1 (the underflow direction, float() -> signed 0.0).
    // Only consulted when from_chars already validated the token as a
    // number; the explicit exponent is clamped, never overflowing.
    inline bool lexeme_magnitude_below_one(
        std::string_view lexeme) noexcept {
        std::size_t pos = 0;
        if (pos < lexeme.size() && lexeme[pos] == '-') ++pos;
        std::size_t int_digits = 0;
        bool int_nonzero = false;
        while (pos < lexeme.size() && lexeme[pos] >= '0' &&
               lexeme[pos] <= '9') {
            if (lexeme[pos] != '0') int_nonzero = true;
            if (int_nonzero) ++int_digits;
            ++pos;
        }
        std::size_t frac_zeros = 0;
        if (pos < lexeme.size() && lexeme[pos] == '.') {
            ++pos;
            while (pos < lexeme.size() && lexeme[pos] >= '0' &&
                   lexeme[pos] <= '9') {
                if (!int_nonzero && lexeme[pos] == '0')
                    ++frac_zeros;
                else
                    int_nonzero = true;
                ++pos;
            }
        }
        long long exponent = 0;
        if (pos < lexeme.size() && (lexeme[pos] == 'e' || lexeme[pos] == 'E')) {
            ++pos;
            bool exp_negative = false;
            if (pos < lexeme.size() &&
                (lexeme[pos] == '+' || lexeme[pos] == '-')) {
                exp_negative = lexeme[pos] == '-';
                ++pos;
            }
            for (; pos < lexeme.size() && lexeme[pos] >= '0' &&
                   lexeme[pos] <= '9';
                 ++pos) {
                exponent = exponent * 10 + (lexeme[pos] - '0');
                if (exponent > 1000000) exponent = 1000000; // clamp: only the direction matters
            }
            if (exp_negative) exponent = -exponent;
        }
        // 0.INT-part zero: value = 0.frac x 10^E with the first nonzero
        // frac digit at index frac_zeros, so magnitude < 1 iff E <=
        // frac_zeros. INT-part nonzero: magnitude < 1 iff
        // int_digits + E <= 0.
        if (int_digits == 0)
            return exponent <= static_cast<long long>(frac_zeros);
        return static_cast<long long>(int_digits) + exponent <= 0;
    }

    // Decimal float() semantics for a lexeme already known to be a JSON
    // number: from_chars result value is used when representable; a range
    // error resolves to +inf/-inf or signed zero by the lexeme direction —
    // matching CPython float(): overflow -> inf, underflow -> signed 0.
    // `ok` reports a fully consumed number token (callers gate the grammar
    // separately; this guards against malformed constructed lexemes).
    inline double float_from_lexeme(std::string_view lexeme,
                                    bool &ok) noexcept {
        ok = false;
        double result = 0.0;
        const char *first = lexeme.data();
        const char *last = std::to_address(lexeme.cend());
        const auto parsed = std::from_chars(first, last, result);
        if (parsed.ec == std::errc::invalid_argument || parsed.ptr != last)
            return 0.0;
        ok = true;
        if (parsed.ec == std::errc::result_out_of_range) {
            const bool negative =
                !lexeme.empty() && lexeme.front() == '-';
            if (lexeme_magnitude_below_one(lexeme))
                return negative ? -0.0 : 0.0;
            return negative ? -std::numeric_limits<double>::infinity()
                            : std::numeric_limits<double>::infinity();
        }
        return result;
    }
} // namespace detail

inline const Value::Object &as_object(const Value &value,
                                      std::string_view field) {
    const auto *result = std::get_if<Value::Object>(&value.data);
    if (result == nullptr)
        detail::kind_error("bike::io::as_object", field, "object");
    return *result;
}

inline const Value::Array &as_array(const Value &value,
                                    std::string_view field) {
    const auto *result = std::get_if<Value::Array>(&value.data);
    if (result == nullptr)
        detail::kind_error("bike::io::as_array", field, "array");
    return *result;
}

inline const std::string &as_string(const Value &value,
                                    std::string_view field) {
    const auto *result = std::get_if<std::string>(&value.data);
    if (result == nullptr)
        detail::kind_error("bike::io::as_string", field, "string");
    return *result;
}

inline bool as_bool(const Value &value, std::string_view field) {
    const auto *result = std::get_if<bool>(&value.data);
    if (result == nullptr)
        detail::kind_error("bike::io::as_bool", field, "boolean");
    return *result;
}

inline const Number &as_number(const Value &value, std::string_view field) {
    const auto *result = std::get_if<Number>(&value.data);
    if (result == nullptr)
        detail::kind_error("bike::io::as_number", field, "number");
    return *result;
}

// A JSON integer token: optional '-' followed by digits only. Fraction or
// exponent spellings are floating tokens even when their value is integral
// (int-versus-float spelling is load-bearing in the source codec).
inline bool is_integer_lexeme(std::string_view lexeme) noexcept {
    if (!lexeme.empty() && lexeme.front() == '-') lexeme.remove_prefix(1);
    if (lexeme.empty()) return false;
    for (const char ch: lexeme)
        if (ch < '0' || ch > '9') return false;
    return true;
}

// Integer conversion into the signed 64-bit storage domain (recording SQL
// columns): the lexeme must be an integer token in range — overflowing
// metadata integers fail rather than narrowing through binary64.
inline std::int64_t number_i64(const Number &number, std::string_view field) {
    if (!is_integer_lexeme(number.lexeme))
        detail::kind_error("bike::io::number_i64", field,
                           "integer number lexeme");
    std::int64_t result = 0;
    const auto *end = std::to_address(number.lexeme.cend());
    const auto parsed =
        std::from_chars(number.lexeme.data(), end, result, 10);
    if (parsed.ec == std::errc::invalid_argument || parsed.ptr != end)
        detail::kind_error("bike::io::number_i64", field,
                           "integer number lexeme");
    if (parsed.ec == std::errc::result_out_of_range)
        detail::kind_error("bike::io::number_i64", field,
                           "integer in int64 range");
    return result;
}

inline std::int64_t as_int64(const Value &value, std::string_view field) {
    return number_i64(as_number(value, field), field);
}

// Finite-binary64 extraction: the locale-independent conversion (correctly
// rounded, CPython float() semantics — overflow becomes nonfinite, underflow
// becomes signed zero); a lexeme that does not produce a finite binary64 is
// rejected here rather than coerced (the recording tag carries nonfinite
// values through Tagged float64, never through a bare number).
inline double number_double(const Number &number, std::string_view field) {
    bool ok = false;
    const double result = detail::float_from_lexeme(number.lexeme, ok);
    if (!ok || !std::isfinite(result))
        detail::kind_error("bike::io::number_double", field,
                           "finite binary64 number lexeme");
    return result;
}

inline double as_double(const Value &value, std::string_view field) {
    return number_double(as_number(value, field), field);
}

// Double extraction through the recording domain: finite Number lexemes
// convert directly; Tagged{kind:"float64"} supplies the named nonfinite
// sentinels the recording codec permits. This is the opt-in accessor —
// as_double stays finite-only for physical domains.
inline double tagged_double(const Value &value, std::string_view field) {
    if (const auto *number = std::get_if<Number>(&value.data))
        return number_double(*number, field);
    if (const auto *tagged = std::get_if<Tagged>(&value.data)) {
        if (tagged->kind == "float64" && tagged->fields.size() == 1 &&
            tagged->fields.front().first == "value") {
            const std::string &spelling = tagged->fields.front().second;
            if (spelling == "nan")
                return std::numeric_limits<double>::quiet_NaN();
            if (spelling == "+inf")
                return std::numeric_limits<double>::infinity();
            if (spelling == "-inf")
                return -std::numeric_limits<double>::infinity();
        }
    }
    detail::kind_error("bike::io::tagged_double", field,
                       "finite number or tagged float64 sentinel");
}

} // namespace bike::io
