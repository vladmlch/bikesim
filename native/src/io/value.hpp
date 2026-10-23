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

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <ranges>
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

// libstdc++ 13 checks Value's default-constructibility while defining
// pair<string, Value>. An inline constexpr destructor chain then reaches
// variant -> vector<pair> -> is_destructible<pair> before pair is complete.
// Declare ObjectList's special members here and define their non-constexpr
// bodies after Value so destruction is instantiated only with complete types.
// The inherited vector interface keeps object ordering and ownership intact.
template <class V>
struct ObjectList : std::vector<std::pair<std::string, V>> {
    using base = std::vector<std::pair<std::string, V>>;
    using base::base;
    ObjectList();
    explicit ObjectList(base members);
    ObjectList(const ObjectList &other);
    ObjectList(ObjectList &&other) noexcept;
    ObjectList &operator=(const ObjectList &other);
    ObjectList &operator=(ObjectList &&other) noexcept;
    ~ObjectList();
};

struct Value {
    using Array = std::vector<Value>;
    using Object = ObjectList<Value>;
    std::variant<std::monostate, bool, Number, std::string, Array, Object,
                 Bytes, Tagged>
        data;
};

// clang-tidy modernize-use-equals-default would restore the eager constexpr
// destruction cycle above. Completion-plan Task 1.13 records the libstdc++ 13
// reproducer; remove this exception only when the qualified Linux compilers
// accept the defaulted form and the JSON/config copy/move guards pass.
// NOLINTBEGIN(modernize-use-equals-default)
template <class V>
ObjectList<V>::ObjectList() {}
template <class V>
ObjectList<V>::ObjectList(base members) : base(std::move(members)) {}
template <class V>
ObjectList<V>::ObjectList(const ObjectList &other) : base(other) {}
template <class V>
ObjectList<V>::ObjectList(ObjectList &&other) noexcept : base(std::move(other)) {}
template <class V>
ObjectList<V> &ObjectList<V>::operator=(const ObjectList &other) {
    if (this != &other) base::operator=(other);
    return *this;
}
template <class V>
ObjectList<V> &ObjectList<V>::operator=(ObjectList &&other) noexcept {
    base::operator=(std::move(other));
    return *this;
}
template <class V>
ObjectList<V>::~ObjectList() {}
// NOLINTEND(modernize-use-equals-default)

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

// Codepoints that are printable under CPython 3.14's Unicode 16.0.0
// tables but unassigned — hence non-printable — under 3.13's Unicode
// 15.1.0 (UNIDATA_VERSION in each tree's Modules/unicodedata_db.h).
// Derived by decoding both Objects/unicodetype_db.h index1/index2/
// TypeRecord tries and diffing the PRINTABLE_MASK bit over every
// codepoint: 5185 codepoints in the 48 ranges below. str repr() under
// 3.13 must escape them (\xhh/\uxxxx/\Uxxxxxxxx, Py_hexdigits
// lowercase); 3.14 copies them verbatim.
inline constexpr std::array<std::pair<char32_t, char32_t>, 48>
    kUnicode16OnlyPrintable{{
        {0x00897u, 0x00897u}, {0x01b4eu, 0x01b4fu}, {0x01b7fu, 0x01b7fu},
        {0x01c89u, 0x01c8au}, {0x02427u, 0x02429u}, {0x031e4u, 0x031e5u},
        {0x0a7cbu, 0x0a7cdu}, {0x0a7dau, 0x0a7dcu}, {0x105c0u, 0x105f3u},
        {0x10d40u, 0x10d65u}, {0x10d69u, 0x10d85u}, {0x10d8eu, 0x10d8fu},
        {0x10ec2u, 0x10ec4u}, {0x10efcu, 0x10efcu}, {0x11380u, 0x11389u},
        {0x1138bu, 0x1138bu}, {0x1138eu, 0x1138eu}, {0x11390u, 0x113b5u},
        {0x113b7u, 0x113c0u}, {0x113c2u, 0x113c2u}, {0x113c5u, 0x113c5u},
        {0x113c7u, 0x113cau}, {0x113ccu, 0x113d5u}, {0x113d7u, 0x113d8u},
        {0x113e1u, 0x113e2u}, {0x116d0u, 0x116e3u}, {0x11bc0u, 0x11be1u},
        {0x11bf0u, 0x11bf9u}, {0x11f5au, 0x11f5au}, {0x13460u, 0x143fau},
        {0x16100u, 0x16139u}, {0x16d40u, 0x16d79u}, {0x18cffu, 0x18cffu},
        {0x1cc00u, 0x1ccf9u}, {0x1cd00u, 0x1ceb3u}, {0x1e5d0u, 0x1e5fau},
        {0x1e5ffu, 0x1e5ffu}, {0x1f8b2u, 0x1f8bbu}, {0x1f8c0u, 0x1f8c1u},
        {0x1fa89u, 0x1fa89u}, {0x1fa8fu, 0x1fa8fu}, {0x1fabeu, 0x1fabeu},
        {0x1fac6u, 0x1fac6u}, {0x1fadcu, 0x1fadcu}, {0x1fadfu, 0x1fadfu},
        {0x1fae9u, 0x1fae9u}, {0x1fbcbu, 0x1fbefu},
    }};

[[nodiscard]] inline bool unicode16_only_printable(char32_t cp) {
    const auto it = std::ranges::lower_bound(
        kUnicode16OnlyPrintable, cp, {},
        [](const auto &range) { return range.second; });
    return it != kUnicode16OnlyPrintable.end() && cp >= it->first;
}

// repr() of one codepoint that 3.13 marks non-printable: CPython
// unicode_repr's \xhh/\uxxxx/\Uxxxxxxxx escapes (lowercase
// Py_hexdigits). Only called after unicode16_only_printable(cp).
inline void repr_escape_nonprintable(std::string &out, char32_t cp) {
    static constexpr std::string_view kHex = "0123456789abcdef";
    out.push_back('\\');
    if (cp <= 0xffu) {
        out.push_back('x');
        out.push_back(kHex[(cp >> 4U) & 0xfU]);
        out.push_back(kHex[cp & 0xfU]);
    } else if (cp <= 0xffffu) {
        out.push_back('u');
        for (int shift = 12; shift >= 0; shift -= 4)
            out.push_back(kHex[(static_cast<std::uint32_t>(cp) >> static_cast<unsigned>(shift)) & 0xfU]);
    } else {
        out.push_back('U');
        for (int shift = 28; shift >= 0; shift -= 4)
            out.push_back(kHex[(static_cast<std::uint32_t>(cp) >> static_cast<unsigned>(shift)) & 0xfU]);
    }
}

// utf8_decode_one's malformed-sequence sentinel, spelled as a named
// constant: char32_t is unsigned, so comparing a returned codepoint
// against the literal -1 (or any `cp >= 0` check) is wrong — the
// sentinel must be named to be tested.
inline constexpr char32_t kInvalidCodepoint = static_cast<char32_t>(-1);

// Decodes one UTF-8 codepoint from `text` at `pos`; on success returns
// the codepoint and advances `pos` past the sequence, on a malformed
// sequence returns kInvalidCodepoint and leaves `pos` unchanged.
[[nodiscard]] inline char32_t
utf8_decode_one(std::string_view text, std::size_t &pos) noexcept {
    const auto byte = [&text](std::size_t i) {
        return static_cast<unsigned char>(text[i]);
    };
    const unsigned char lead = byte(pos);
    char32_t cp = 0;
    std::size_t len = 0;
    if (lead < 0x80) {
        cp = lead;
        len = 1;
    } else if (lead >= 0xc2 && lead <= 0xdf) {
        cp = lead & 0x1fU;
        len = 2;
    } else if (lead >= 0xe0 && lead <= 0xef) {
        cp = lead & 0x0fU;
        len = 3;
    } else if (lead >= 0xf0 && lead <= 0xf4) {
        cp = lead & 0x07U;
        len = 4;
    } else {
        return kInvalidCodepoint;
    }
    if (pos + len > text.size()) return kInvalidCodepoint;
    for (std::size_t k = 1; k < len; ++k) {
        const unsigned char cont = byte(pos + k);
        if ((cont & 0xc0U) != 0x80U) return kInvalidCodepoint;
        cp = (cp << 6U) | (cont & 0x3fU);
    }
    pos += len;
    return static_cast<char32_t>(cp);
}

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
            return std::cmp_less_equal(exponent, frac_zeros);
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
