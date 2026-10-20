// json.cpp — strict lossless JSON boundary (task T05).
//
// Parser: RapidJSON SAX with kParseNumbersAsStringsFlag so number lexemes
// survive verbatim (no binary64 round-trip), UTF-8 source validation,
// explicit duplicate-key rejection and a nesting bound. Codec: the
// bike_recording.json.v1 tag scheme ported from the source authority
// src/bike_sim/recording/json_codec.py:1-126, mirroring CPython 3.13
// semantics for strict base64 (Modules/binascii.c), Decimal(str)
// canonicalization, datetime/date/time fromisoformat+isoformat and
// timedelta normalization.
#include "io/json.hpp"

#include <rapidjson/encodedstream.h>
#include <rapidjson/error/en.h>
#include <rapidjson/memorystream.h>
#include <rapidjson/reader.h>

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <span>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

namespace bio = bike::io;
using bio::Value;

[[noreturn]] void fail_at(std::string_view caller, const std::string &detail) {
    throw std::invalid_argument(std::string(caller) + ": " + detail);
}
[[noreturn]] void fail_parse(const std::string &detail) {
    fail_at("bike::io::parse_json", detail);
}
[[noreturn]] void fail_codec(const std::string &detail) {
    fail_at("bike::io::parse_recording_json", detail);
}
[[noreturn]] void fail_emit(std::string_view caller, const std::string &detail) {
    fail_at(caller, detail);
}

// ---------------------------------------------------------------------------
// UTF-8 / number-lexeme primitives

// RFC 3629 well-formed UTF-8: rejects overlongs, surrogates and >U+10FFFF.
bool valid_utf8(std::string_view text) {
    const auto bytes = [&text](std::size_t i) {
        return static_cast<unsigned char>(text[i]);
    };
    std::size_t i = 0;
    while (i < text.size()) {
        const unsigned char lead = bytes(i);
        if (lead < 0x80) {
            ++i;
        } else if (lead >= 0xc2 && lead <= 0xdf) {
            if (i + 1 >= text.size() ||
                (bytes(i + 1) & 0xc0U) != 0x80)
                return false;
            i += 2;
        } else if (lead >= 0xe0 && lead <= 0xef) {
            if (i + 2 >= text.size() ||
                (bytes(i + 1) & 0xc0U) != 0x80 ||
                (bytes(i + 2) & 0xc0U) != 0x80 ||
                (lead == 0xe0 && bytes(i + 1) < 0xa0) ||
                (lead == 0xed && bytes(i + 1) > 0x9f))
                return false;
            i += 3;
        } else if (lead >= 0xf0 && lead <= 0xf4) {
            if (i + 3 >= text.size() ||
                (bytes(i + 1) & 0xc0U) != 0x80 ||
                (bytes(i + 2) & 0xc0U) != 0x80 ||
                (bytes(i + 3) & 0xc0U) != 0x80 ||
                (lead == 0xf0 && bytes(i + 1) < 0x90) ||
                (lead == 0xf4 && bytes(i + 1) > 0x8f))
                return false;
            i += 4;
        } else {
            return false;
        }
    }
    return true;
}

// Decodes one codepoint; text must already be valid UTF-8 (checked by the
// caller path — parse inputs are parser-validated and emit paths call
// valid_utf8 first).
char32_t utf8_decode(std::string_view text, std::size_t &pos) {
    const auto byte = [&text](std::size_t i) {
        return static_cast<unsigned char>(text[i]);
    };
    const unsigned char lead = byte(pos);
    if (lead < 0x80) {
        ++pos;
        return lead;
    }
    if (lead < 0xe0) {
        const char32_t cp = static_cast<char32_t>(
            ((lead & 0x1fU) << 6U) | (byte(pos + 1) & 0x3fU));
        pos += 2;
        return cp;
    }
    if (lead < 0xf0) {
        const char32_t cp = static_cast<char32_t>(
            ((lead & 0x0fU) << 12U) | ((byte(pos + 1) & 0x3fU) << 6U) |
            (byte(pos + 2) & 0x3fU));
        pos += 3;
        return cp;
    }
    const char32_t cp = static_cast<char32_t>(
        ((lead & 0x07U) << 18U) | ((byte(pos + 1) & 0x3fU) << 12U) |
        ((byte(pos + 2) & 0x3fU) << 6U) | (byte(pos + 3) & 0x3fU));
    pos += 4;
    return cp;
}

bool ascii_digit(char c) { return c >= '0' && c <= '9'; }

// Exact JSON number grammar: -?(0|[1-9][0-9]*)(\.[0-9]+)?([eE][+-]?[0-9]+)?
bool is_json_number(std::string_view lexeme) {
    std::size_t pos = 0;
    const auto need = [&](std::size_t count) {
        return pos + count <= lexeme.size();
    };
    const auto take_digits = [&]() {
        const std::size_t start = pos;
        while (pos < lexeme.size() && ascii_digit(lexeme[pos])) ++pos;
        return pos > start;
    };
    if (need(1) && lexeme[pos] == '-') ++pos;
    if (!need(1)) return false;
    if (lexeme[pos] == '0') {
        ++pos;
    } else if (lexeme[pos] >= '1' && lexeme[pos] <= '9') {
        ++pos;
        take_digits();
    } else {
        return false;
    }
    if (need(1) && lexeme[pos] == '.') {
        ++pos;
        if (!take_digits()) return false;
    }
    if (need(1) && (lexeme[pos] == 'e' || lexeme[pos] == 'E')) {
        ++pos;
        if (need(1) && (lexeme[pos] == '+' || lexeme[pos] == '-')) ++pos;
        if (!take_digits()) return false;
    }
    return pos == lexeme.size();
}

bool float_token(std::string_view lexeme) {
    return lexeme.find_first_of(".eE") != std::string_view::npos;
}

// float()-equivalent range probe: a float-spelled lexeme that rounds to a
// nonfinite binary64 is nonfinite input (loads()' finite() check,
// json_codec.py:108-112). Underflow to zero/denormal stays finite, like
// float('1e-999') -> 0.0; overflow -> inf, mirroring CPython.
bool finite_lexeme(std::string_view lexeme) {
    if (!is_json_number(lexeme)) return false;
    bool ok = false;
    const double value = bio::detail::float_from_lexeme(lexeme, ok);
    return ok && std::isfinite(value);
}

void check_lexeme(std::string_view lexeme, bool require_finite,
                  std::string_view caller) {
    if (!is_json_number(lexeme))
        fail_emit(caller, "invalid JSON number lexeme: " +
                              std::string(lexeme));
    if (require_finite && float_token(lexeme) && !finite_lexeme(lexeme))
        fail_emit(caller, "nonfinite JSON number lexeme: " +
                              std::string(lexeme));
}

// ---------------------------------------------------------------------------
// SAX handler -> owned value tree

struct SaxHandler {
    struct Frame {
        bool is_object;
        Value::Array elements;
        Value::Object members;
        std::unordered_set<std::string> keys;
        std::string pending_key;
    };

    std::vector<Frame> stack;
    Value root;
    std::string failure;

    bool fail(std::string message) {
        failure = std::move(message);
        return false;
    }

    bool attach(Value value) {
        if (stack.empty()) {
            root = std::move(value);
            return true;
        }
        Frame &frame = stack.back();
        if (frame.is_object) {
            frame.members.emplace_back(std::move(frame.pending_key),
                                       std::move(value));
        } else {
            frame.elements.push_back(std::move(value));
        }
        return true;
    }

    bool Null() { return attach(Value{std::monostate{}}); }
    bool Bool(bool value) { return attach(Value{value}); }
    bool RawNumber(const char *text, rapidjson::SizeType length, bool) {
        return attach(Value{bio::Number{std::string(text, length)}});
    }
    // Dead under kParseNumbersAsStringsFlag; implemented so the handler is
    // total if the flag set ever changes.
    bool Int(int value) { return attach(Value{bio::Number{std::to_string(value)}}); }
    bool Uint(unsigned value) {
        return attach(Value{bio::Number{std::to_string(value)}});
    }
    bool Int64(std::int64_t value) {
        return attach(Value{bio::Number{std::to_string(value)}});
    }
    bool Uint64(std::uint64_t value) {
        return attach(Value{bio::Number{std::to_string(value)}});
    }
    bool Double(double value) {
        return attach(Value{bio::Number{std::to_string(value)}});
    }
    bool String(const char *text, rapidjson::SizeType length, bool) {
        // RapidJSON transcoder accepts unpaired \udXXX escapes and emits
        // their CESU-8/WTF-8 byte forms — not storable in a UTF-8 string
        // domain; reject them here (fail-closed, no silent lossy decode).
        if (!valid_utf8(std::string_view(text, length)))
            return fail("invalid UTF-8 in JSON string "
                        "(unpaired surrogate escape)");
        return attach(Value{std::string(text, length)});
    }
    bool Key(const char *text, rapidjson::SizeType length, bool) {
        if (!valid_utf8(std::string_view(text, length)))
            return fail("invalid UTF-8 in JSON object key "
                        "(unpaired surrogate escape)");
        Frame &frame = stack.back();
        std::string key(text, length);
        if (!frame.keys.insert(key).second)
            return fail("duplicate JSON field: " + key);
        frame.pending_key = std::move(key);
        return true;
    }
    bool StartObject() {
        if (stack.size() >= bio::kMaxDepth)
            return fail("JSON nesting exceeds the " +
                        std::to_string(bio::kMaxDepth) + "-level limit");
        stack.push_back(Frame{.is_object = true,
                              .elements = {},
                              .members = {},
                              .keys = {},
                              .pending_key = {}});
        return true;
    }
    bool EndObject(rapidjson::SizeType) {
        Value value{std::move(stack.back().members)};
        stack.pop_back();
        return attach(std::move(value));
    }
    bool StartArray() {
        if (stack.size() >= bio::kMaxDepth)
            return fail("JSON nesting exceeds the " +
                        std::to_string(bio::kMaxDepth) + "-level limit");
        stack.push_back(Frame{.is_object = false,
                              .elements = {},
                              .members = {},
                              .keys = {},
                              .pending_key = {}});
        return true;
    }
    bool EndArray(rapidjson::SizeType) {
        Value value{std::move(stack.back().elements)};
        stack.pop_back();
        return attach(std::move(value));
    }
};

// ---------------------------------------------------------------------------
// Field-path diagnostics: $["key"][3] — json_codec.py's path vocabulary.
void path_key(std::string &path, std::string_view key) {
    path += "[\"";
    for (const char c : key) {
        if (c == '"' || c == '\\') path += '\\';
        path += c;
    }
    path += "\"]";
}
void path_index(std::string &path, std::size_t index) {
    path += '[';
    path += std::to_string(index);
    path += ']';
}

// Member lookup rides the public bio::find_member (ADL picks it up for
// Value::Object arguments — no local twin to shadow).
const std::string *member_string(const Value::Object &object,
                                 std::string_view key) {
    const Value *found = find_member(object, key);
    if (found == nullptr) return nullptr;
    return std::get_if<std::string>(&found->data);
}

// ---------------------------------------------------------------------------
// Strict base64 — port of CPython 3.13 Modules/binascii.c
// binascii_a2b_base64_impl(strict_mode=1), which is what
// base64.b64decode(value, validate=True) runs. CPython 3.14 rewrote
// the loop (padding_started flag, goto-done early exit, excess-data
// check at padding completion) but the strict-mode accept/output
// behaviour — including every error message selection — is equivalent;
// this implementation deliberately keeps the 3.13 pads-counting shape
// and its "Excess data after padding"/"Discontinuous padding not
// allowed" choice point at the next non-'=' character.

int b64_digit(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

std::vector<std::byte> b64_decode_strict(std::string_view text,
                                       const std::string &path) {
    const auto invalid = [&](std::string_view message) -> void {
        fail_codec("invalid base64 recording data at " + path + " (" +
                   std::string(message) + ")");
    };
    std::vector<std::byte> out;
    out.reserve((text.size() / 4) * 3 + 3);
    int quad_pos = 0;
    int pads = 0;
    unsigned int leftchar = 0;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const char ch = text[i];
        if (ch == '=') {
            ++pads;
            if (quad_pos >= 2 && quad_pos + pads <= 4) continue;
            if (quad_pos == 1) break; // falls through to the count error
            invalid(quad_pos == 0 && i == 0 ? "Leading padding not allowed"
                                            : "Excess padding not allowed");
        }
        const int digit = b64_digit(ch);
        if (digit < 0) invalid("Only base64 data is allowed");
        if (pads != 0)
            invalid(quad_pos + pads == 4 ? "Excess data after padding"
                                         : "Discontinuous padding not allowed");
        pads = 0;
        switch (quad_pos) {
        case 0:
            quad_pos = 1;
            leftchar = static_cast<unsigned int>(digit);
            break;
        case 1:
            quad_pos = 2;
            out.push_back(static_cast<std::byte>(
                (leftchar << 2U) |
                (static_cast<unsigned int>(digit) >> 4U)));
            leftchar = static_cast<unsigned int>(digit) & 0x0fU;
            break;
        case 2:
            quad_pos = 3;
            out.push_back(static_cast<std::byte>(
                (leftchar << 4U) |
                (static_cast<unsigned int>(digit) >> 2U)));
            leftchar = static_cast<unsigned int>(digit) & 0x03U;
            break;
        default:
            quad_pos = 0;
            out.push_back(static_cast<std::byte>(
                (leftchar << 6U) | static_cast<unsigned int>(digit)));
            leftchar = 0;
            break;
        }
    }
    if (quad_pos == 1)
        invalid("number of data characters cannot be 1 more than a "
                "multiple of 4");
    if (quad_pos != 0 && quad_pos + pads < 4) invalid("Incorrect padding");
    return out;
}

constexpr std::string_view kB64Alphabet =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string b64_encode(std::span<const std::byte> bytes) {
    std::string out;
    out.reserve(((bytes.size() + 2) / 3) * 4);
    std::size_t i = 0;
    while (i + 3 <= bytes.size()) {
        const unsigned int triple =
            (static_cast<unsigned int>(bytes[i]) << 16U) |
            (static_cast<unsigned int>(bytes[i + 1]) << 8U) |
            static_cast<unsigned int>(bytes[i + 2]);
        out += kB64Alphabet[(triple >> 18U) & 63U];
        out += kB64Alphabet[(triple >> 12U) & 63U];
        out += kB64Alphabet[(triple >> 6U) & 63U];
        out += kB64Alphabet[triple & 63U];
        i += 3;
    }
    const std::size_t rem = bytes.size() - i;
    if (rem == 1) {
        const unsigned int a = static_cast<unsigned int>(bytes[i]);
        out += kB64Alphabet[(a >> 2U) & 63U];
        out += kB64Alphabet[(a << 4U) & 63U];
        out += "==";
    } else if (rem == 2) {
        const unsigned int a = static_cast<unsigned int>(bytes[i]);
        const unsigned int b = static_cast<unsigned int>(bytes[i + 1]);
        out += kB64Alphabet[(a >> 2U) & 63U];
        out += kB64Alphabet[((a << 4U) | (b >> 4U)) & 63U];
        out += kB64Alphabet[(b << 2U) & 63U];
        out += '=';
    }
    return out;
}

// ---------------------------------------------------------------------------
// Decimal(str) validation + str(Decimal) canonicalization.
// Mirrors CPython _decimal string parsing: edge Unicode-whitespace strip,
// global '_' removal, optional sign, specials (inf/infinity/[s]nan[digits],
// case-insensitive), then digits[.digits][e[+-]digits] with Unicode
// category-Nd digits. Exponents are bounded by libmpdec's 64-bit
// MPD_MIN_EMIN/MPD_MAX_EMAX (+-999999999999999999). Output is the exact
// str(Decimal) spelling.

// Unicode category Nd ranges (UCD 15.1.0 — the CPython 3.13 unicodetype_db.h
// UNIDATA_VERSION — generated, 680 codepoints). Unicode 16.0.0 (CPython
// 3.14's table) added seven Nd blocks that 3.13's Decimal()/digit paths do
// not know; they were removed from this table so the ranges below reject
// exactly the codepoints the 3.13 oracle rejects:
//   0x10D40-10D49 Garay, 0x116D0-116E3 Myanmar Extended-C,
//   0x11BF0-11BF9 Sunuwar, 0x16130-16139 Gurung Khema,
//   0x16D70-16D79 Kirat Rai, 0x1CCF0-1CCF9 Outlined, 0x1E5F1-1E5FA Ol Onal.
// (Method: decoded both trees' Objects/unicodetype_db.h index1/index2/
// TypeRecord trie and diffed the DECIMAL_MASK bit over every codepoint.)
constexpr std::array<std::pair<char32_t, char32_t>, 64> kNdRanges{{
    {0x0030u, 0x0039u}, {0x0660u, 0x0669u}, {0x06f0u, 0x06f9u},
    {0x07c0u, 0x07c9u}, {0x0966u, 0x096fu}, {0x09e6u, 0x09efu},
    {0x0a66u, 0x0a6fu}, {0x0ae6u, 0x0aefu}, {0x0b66u, 0x0b6fu},
    {0x0be6u, 0x0befu}, {0x0c66u, 0x0c6fu}, {0x0ce6u, 0x0cefu},
    {0x0d66u, 0x0d6fu}, {0x0de6u, 0x0defu}, {0x0e50u, 0x0e59u},
    {0x0ed0u, 0x0ed9u}, {0x0f20u, 0x0f29u}, {0x1040u, 0x1049u},
    {0x1090u, 0x1099u}, {0x17e0u, 0x17e9u}, {0x1810u, 0x1819u},
    {0x1946u, 0x194fu}, {0x19d0u, 0x19d9u}, {0x1a80u, 0x1a89u},
    {0x1a90u, 0x1a99u}, {0x1b50u, 0x1b59u}, {0x1bb0u, 0x1bb9u},
    {0x1c40u, 0x1c49u}, {0x1c50u, 0x1c59u}, {0xa620u, 0xa629u},
    {0xa8d0u, 0xa8d9u}, {0xa900u, 0xa909u}, {0xa9d0u, 0xa9d9u},
    {0xa9f0u, 0xa9f9u}, {0xaa50u, 0xaa59u}, {0xabf0u, 0xabf9u},
    {0xff10u, 0xff19u}, {0x104a0u, 0x104a9u}, {0x10d30u, 0x10d39u},
    {0x11066u, 0x1106fu}, {0x110f0u, 0x110f9u},
    {0x11136u, 0x1113fu}, {0x111d0u, 0x111d9u}, {0x112f0u, 0x112f9u},
    {0x11450u, 0x11459u}, {0x114d0u, 0x114d9u}, {0x11650u, 0x11659u},
    {0x116c0u, 0x116c9u}, {0x11730u, 0x11739u},
    {0x118e0u, 0x118e9u}, {0x11950u, 0x11959u},
    {0x11c50u, 0x11c59u}, {0x11d50u, 0x11d59u}, {0x11da0u, 0x11da9u},
    {0x11f50u, 0x11f59u}, {0x16a60u, 0x16a69u},
    {0x16ac0u, 0x16ac9u}, {0x16b50u, 0x16b59u},
    {0x1d7ceu, 0x1d7ffu}, {0x1e140u, 0x1e149u},
    {0x1e2f0u, 0x1e2f9u}, {0x1e4f0u, 0x1e4f9u},
    {0x1e950u, 0x1e959u}, {0x1fbf0u, 0x1fbf9u},
}};

bool is_nd_digit(char32_t cp) {
    if (cp < 0x80) return cp >= '0' && cp <= '9';
    std::size_t begin = 0;
    std::size_t end = kNdRanges.size();
    while (begin != end) {
        const std::size_t mid = begin + (end - begin) / 2;
        if (cp < kNdRanges[mid].first)
            end = mid;
        else if (cp > kNdRanges[mid].second)
            begin = mid + 1;
        else
            return true;
    }
    return false;
}

// Py_UNICODE_ISSPACE set used by CPython for str.strip()/Decimal trimming.
bool is_unicode_space(char32_t cp) {
    if (cp >= 0x09 && cp <= 0x0d) return true;
    if (cp >= 0x1c && cp <= 0x1f) return true;
    if (cp == 0x20 || cp == 0x85 || cp == 0xa0 || cp == 0x1680) return true;
    if (cp >= 0x2000 && cp <= 0x200a) return true;
    if (cp == 0x2028 || cp == 0x2029 || cp == 0x202f || cp == 0x205f)
        return true;
    return cp == 0x3000;
}

// Lowercase an ASCII letter (only applied to A-Z/a-z inputs).
char ascii_lower(char c) {
    if (c >= 'A' && c <= 'Z') return static_cast<char>(c + ('a' - 'A'));
    return c;
}

struct DecimalParts {
    bool negative = false;
    // kind: 0 numeric, 1 infinity, 2 nan, 3 snan
    int kind = 0;
    std::string digits; // ASCII coefficient digits (no sign, no dot)
    std::int64_t exponent = 0; // value = digits * 10^exponent
    std::string nan_payload; // ASCII digits for nan payload (may be empty)
};

// Preprocess: strip edge unicode whitespace then remove every '_'.
bool decimal_preprocess(std::string_view input, std::string &body) {
    if (!valid_utf8(input)) return false;
    // codepoint offsets of leading/trailing whitespace
    std::size_t first = 0;
    while (first < input.size()) {
        std::size_t pos = first;
        const char32_t cp = utf8_decode(input, pos);
        if (!is_unicode_space(cp)) break;
        first = pos;
    }
    std::size_t last = input.size();
    while (last > first) {
        // find the codepoint starting before `last`
        std::size_t pos = first;
        std::size_t prev = pos;
        while (pos < last) {
            prev = pos;
            utf8_decode(input, pos);
        }
        std::size_t scan = prev;
        const char32_t cp = utf8_decode(input, scan);
        if (!is_unicode_space(cp)) break;
        last = prev;
    }
    body.clear();
    body.reserve(last - first);
    for (std::size_t i = first; i < last; ++i)
        if (input[i] != '_') body += input[i];
    return !body.empty();
}

// Parses a run of Unicode digits -> ASCII value digits. Returns the count
// of consumed BYTES (0 on failure: a non-digit codepoint stops the run —
// the caller decides whether the run is acceptable).
std::size_t decimal_digits(std::string_view text, std::size_t start,
                           std::string &digits) {
    std::size_t pos = start;
    while (pos < text.size()) {
        std::size_t scan = pos;
        const char32_t cp = utf8_decode(text, scan);
        if (!is_nd_digit(cp)) break;
        pos = scan;
        // value = cp - range start
        char32_t value = cp;
        for (const auto &[lo, hi] : kNdRanges) {
            if (cp >= lo && cp <= hi) {
                value = cp - lo;
                break;
            }
        }
        digits += static_cast<char>('0' + static_cast<unsigned char>(value));
    }
    return pos;
}

// str(Decimal) formatting of (sign, digits, exp10) — the mpdecimal
// scientific/fixed-point rule: fixed notation iff exp <= 0 and
// adjusted >= -6, else scientific 'd[.ddd]E±adj'.
std::string decimal_format(bool negative, const std::string &digits_in,
                           std::int64_t exp10) {
    std::string digits = digits_in;
    const std::size_t first_nonzero = digits.find_first_not_of('0');
    if (first_nonzero == std::string::npos) {
        digits = "0";
    } else {
        digits.erase(0, first_nonzero);
    }
    // exponent arithmetic uses widening intermediates guarded by the
    // parse-time bound; digit counts are bounded by the 512 MiB input cap.
    const long long len = static_cast<long long>(digits.size());
    const long long exp_ll = static_cast<long long>(exp10);
    const long long adj = exp_ll + len - 1;
    std::string out;
    if (negative) out += '-';
    if (exp_ll <= 0 && adj >= -6) {
        const long long point = len + exp_ll;
        if (exp_ll == 0) {
            out += digits;
        } else if (point > 0) {
            out += digits.substr(0, static_cast<std::size_t>(point));
            out += '.';
            out += digits.substr(static_cast<std::size_t>(point));
        } else {
            out += "0.";
            out.append(static_cast<std::size_t>(-point), '0');
            out += digits;
        }
    } else {
        out += digits[0];
        if (digits.size() > 1) {
            out += '.';
            out += digits.substr(1);
        }
        out += 'E';
        out += (adj < 0) ? '-' : '+';
        out += std::to_string(adj < 0 ? -adj : adj);
    }
    return out;
}

// Validates + canonicalizes a Decimal('...') input. Returns false on
// InvalidOperation; on success `canonical` is the exact str(Decimal)
// spelling. `reject` reports through fail_codec when a path is given.
bool decimal_canonical(std::string_view input, std::string &canonical,
                       std::string *diagnostic) {
    std::string body;
    if (!decimal_preprocess(input, body)) {
        if (diagnostic != nullptr) *diagnostic = "empty decimal lexeme";
        return false;
    }
    std::size_t pos = 0;
    bool negative = false;
    if (body[pos] == '+' || body[pos] == '-') {
        negative = body[pos] == '-';
        ++pos;
    }
    if (pos >= body.size()) {
        if (diagnostic != nullptr) *diagnostic = "bare decimal sign";
        return false;
    }
    // Case-insensitive specials; Python compares the WHOLE remaining token.
    std::string rest = body.substr(pos);
    std::string lower;
    lower.reserve(rest.size());
    for (const char c : rest) lower += ascii_lower(c);
    if (lower == "inf" || lower == "infinity") {
        canonical = negative ? "-Infinity" : "Infinity";
        return true;
    }
    bool snan = false;
    std::size_t nan_pos = 0;
    if (lower.size() >= 1 && lower[0] == 's') {
        snan = true;
        nan_pos = 1;
    }
    if (lower.size() >= nan_pos + 3 &&
        lower.compare(nan_pos, 3, "nan") == 0) {
        // nan payload: Unicode digits, value-normalized; an all-zero /
        // empty payload disappears (str(Decimal('-NaN0')) == '-NaN').
        std::string payload;
        const std::size_t end =
            decimal_digits(rest, nan_pos + 3, payload);
        if (end != rest.size()) {
            if (diagnostic != nullptr)
                *diagnostic = "invalid decimal nan payload";
            return false;
        }
        const std::size_t nz = payload.find_first_not_of('0');
        std::string canon = negative ? "-" : "";
        canon += snan ? "sNaN" : "NaN";
        if (nz != std::string::npos) canon += payload.substr(nz);
        canonical = std::move(canon);
        return true;
    }
    // Numeric: digits ('.' digits?)? | '.' digits ; optional exponent.
    std::string int_digits;
    std::size_t cur = decimal_digits(rest, 0, int_digits);
    std::string frac_digits;
    if (cur < rest.size() && rest[cur] == '.') {
        ++cur;
        cur = decimal_digits(rest, cur, frac_digits);
    }
    if (int_digits.empty() && frac_digits.empty()) {
        if (diagnostic != nullptr)
            *diagnostic = "decimal lexeme has no digits";
        return false;
    }
    long long explicit_exp = 0;
    if (cur < rest.size() && (rest[cur] == 'e' || rest[cur] == 'E')) {
        ++cur;
        bool exp_negative = false;
        if (cur < rest.size() && (rest[cur] == '+' || rest[cur] == '-')) {
            exp_negative = rest[cur] == '-';
            ++cur;
        }
        std::string exp_digits;
        cur = decimal_digits(rest, cur, exp_digits);
        if (exp_digits.empty()) {
            if (diagnostic != nullptr)
                *diagnostic = "decimal exponent has no digits";
            return false;
        }
        // libmpdec accumulates then bounds the stored exponent; values
        // past ~4e18 saturate — both bound checks below reject them
        // either way (MAX_EMAX 999999999999999999, MIN_ETINY
        // -1999999999999999997).
        for (const char c : exp_digits) {
            explicit_exp = explicit_exp * 10 + (c - '0');
            if (explicit_exp > 4000000000000000000LL) {
                explicit_exp = 4000000000000000000LL;
                break;
            }
        }
        if (exp_negative) explicit_exp = -explicit_exp;
    }
    if (cur != rest.size()) {
        if (diagnostic != nullptr)
            *diagnostic = "trailing garbage in decimal lexeme";
        return false;
    }
    std::string digits = int_digits + frac_digits;
    const std::size_t first_nonzero = digits.find_first_not_of('0');
    if (first_nonzero == std::string::npos) {
        digits = "0";
    } else {
        digits.erase(0, first_nonzero);
    }
    const long long frac_len =
        static_cast<long long>(frac_digits.size());
    const long long stored_exp = explicit_exp - frac_len;
    const long long sig_len = static_cast<long long>(digits.size());
    // libmpdec 64-bit string-parse bounds: adjusted exponent
    // (stored + len(digits) - 1) <= MPD_MAX_EMAX and
    // stored >= MPD_MIN_ETINY (-1999999999999999997).
    if (stored_exp < -1999999999999999997LL ||
        stored_exp > 999999999999999999LL - (sig_len - 1)) {
        if (diagnostic != nullptr)
            *diagnostic = "decimal exponent out of range";
        return false;
    }
    canonical = decimal_format(negative, digits,
                               static_cast<std::int64_t>(stored_exp));
    return true;
}

// Minimal base-1e9 bignum for the exact decimal expansion of a binary64
// (the Decimal(float) path): value = mantissa * 2^exp2.
struct BigDecimal {
    std::vector<std::uint32_t> limbs; // little-endian base 1e9

    void mul(std::uint32_t factor) {
        std::uint64_t carry = 0;
        for (auto &limb : limbs) {
            const std::uint64_t product =
                static_cast<std::uint64_t>(limb) * factor + carry;
            limb = static_cast<std::uint32_t>(product % 1000000000ULL);
            carry = product / 1000000000ULL;
        }
        while (carry != 0) {
            limbs.push_back(static_cast<std::uint32_t>(carry % 1000000000ULL));
            carry /= 1000000000ULL;
        }
    }
    [[nodiscard]] std::string digits() const {
        if (limbs.empty()) return "0";
        std::string out = std::to_string(limbs.back());
        for (std::size_t i = limbs.size() - 1; i-- > 0;) {
            const std::string chunk = std::to_string(limbs[i]);
            out.append(9 - chunk.size(), '0');
            out += chunk;
        }
        return out;
    }
};

// Exact str(Decimal(d)) for a finite binary64 d, mirroring
// Decimal(float) -> str: exact expansion, canonical format.
std::string decimal_from_f64(double value) {
    const std::uint64_t bits = std::bit_cast<std::uint64_t>(value);
    const bool negative = (bits >> 63U) != 0;
    const std::uint64_t exponent = (bits >> 52U) & 0x7ffU;
    const std::uint64_t fraction = bits & ((1ULL << 52U) - 1U);
    std::uint64_t mantissa = 0;
    std::int64_t exp2 = 0;
    if (exponent == 0) {
        mantissa = fraction;
        exp2 = -1074;
    } else {
        mantissa = fraction | (1ULL << 52U);
        exp2 = static_cast<std::int64_t>(exponent) - 1075;
    }
    // Decimal(+0.0) == Decimal('0'), Decimal(-0.0) == Decimal('-0') —
    // float zero canonicalizes without a fraction, unlike Decimal('0.0').
    if (mantissa == 0)
        return negative ? "-0" : "0";
    // libmpdec normalizes the mantissa's trailing zero bits first —
    // m * 2^e with an even m is the same value as (m>>1) * 2^(e+1).
    // This is what makes Decimal(1200.0) == Decimal('1200') (an integer,
    // not '1200.000...'): 1200.0 is 75 * 2^4 after normalization.
    if (exp2 < 0) {
        const auto zeros =
            static_cast<unsigned>(std::countr_zero(mantissa));
        mantissa >>= zeros;
        exp2 += static_cast<std::int64_t>(zeros);
    }
    BigDecimal big;
    big.limbs.push_back(static_cast<std::uint32_t>(mantissa % 1000000000ULL));
    big.limbs.push_back(
        static_cast<std::uint32_t>((mantissa / 1000000000ULL) % 1000000000ULL));
    big.limbs.push_back(
        static_cast<std::uint32_t>((mantissa / 1000000000000000000ULL)));
    while (!big.limbs.empty() && big.limbs.back() == 0) big.limbs.pop_back();
    std::int64_t exp10 = 0;
    if (exp2 >= 0) {
        std::int64_t shift = exp2;
        while (shift >= 30) {
            big.mul(1U << 30U);
            shift -= 30;
        }
        if (shift > 0) big.mul(1U << static_cast<unsigned>(shift));
    } else {
        // m * 2^e = (m * 5^-e) * 10^e
        std::int64_t powers = -exp2;
        while (powers >= 9) {
            big.mul(1953125U); // 5^9
            powers -= 9;
        }
        if (powers > 0) {
            std::uint32_t factor = 1;
            for (std::int64_t i = 0; i < powers; ++i) factor *= 5U;
            big.mul(factor);
        }
        exp10 = exp2;
    }
    const std::string digits = big.digits();
    return decimal_format(negative, digits, exp10);
}

// Decimal(int lexeme) — integer input keeps its digits; '-0' is the
// integer 0 ('0'), unlike the float -0.0 ('-0').
std::string decimal_from_int_lexeme(std::string_view lexeme) {
    std::string_view digits = lexeme;
    bool negative = false;
    if (!digits.empty() && digits[0] == '-') {
        negative = true;
        digits.remove_prefix(1);
    }
    const std::size_t nz = digits.find_first_not_of('0');
    if (nz == std::string_view::npos) return "0";
    std::string out;
    if (negative) out += '-';
    out += digits.substr(nz);
    return out;
}

// Decimal(number lexeme) — Python from_json_value runs Decimal(member),
// where a JSON number arrives as int (arbitrary precision, exact digits)
// or float (Decimal(float)'s exact expansion, including nonfinite
// spellings -> 'Infinity'/'-Infinity' and underflow -> signed zero).
std::string decimal_from_number_lexeme(std::string_view lexeme) {
    if (!float_token(lexeme)) return decimal_from_int_lexeme(lexeme);
    bool ok = false;
    const double value = bio::detail::float_from_lexeme(lexeme, ok);
    if (!ok || std::isnan(value))
        fail_codec("invalid decimal number lexeme: " + std::string(lexeme));
    if (std::isinf(value))
        return std::signbit(value) ? "-Infinity" : "Infinity";
    return decimal_from_f64(value);
}

// ---------------------------------------------------------------------------
// ISO 8601 date/time/datetime — ports of CPython 3.13 _datetimemodule's
// _find_isoformat_datetime_separator, parse_isoformat_date,
// parse_hh_mm_ss_ff and parse_isoformat_time plus the matching
// isoformat() renderings. Accept/reject behavior mirrors the C accelerator
// (strict ASCII digits; 'T24:00' is rejected by the hour range check —
// the 3.14 next-day wrap is not implemented).

constexpr std::array<int, 13> kDaysInMonth{0,  31, 28, 31, 30, 31, 30,
                                           31, 31, 30, 31, 30, 31};

bool is_leap_year(int year) {
    return year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
}

int days_in_month(int year, int month) {
    return kDaysInMonth[static_cast<std::size_t>(month)] +
           (month == 2 && is_leap_year(year) ? 1 : 0);
}

long long days_before_year(int year) {
    const long long y = year - 1;
    return y * 365 + y / 4 - y / 100 + y / 400;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional order mirrors CPython _isoweek/yday order
long long ymd_to_ord(int year, int month, int day) {
    long long before = 0;
    for (int m = 1; m < month; ++m) before += days_in_month(year, m);
    return days_before_year(year) + before + day;
}

// civil_from_days (Hinnant): z = days since 1970-01-01.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) out-params mirror the civil_from_days decomposition
void ord_to_ymd(long long ord, int &year, int &month, int &day) {
    const long long z = ord - 719163 + 719468;
    const long long era = (z >= 0 ? z : z - 146096) / 146097;
    const long long doe = z - era * 146097;
    const long long yoe =
        (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    long long y = yoe + era * 400;
    const long long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const long long mp = (5 * doy + 2) / 153;
    day = static_cast<int>(doy - (153 * mp + 2) / 5 + 1);
    month = static_cast<int>(mp < 10 ? mp + 3 : mp - 9);
    y += month <= 2;
    year = static_cast<int>(y);
}

long long isoweek1_monday(int year) {
    const long long firstday = ymd_to_ord(year, 1, 1);
    const long long firstweekday = (firstday + 6) % 7; // 0=Monday
    long long monday = firstday - firstweekday;
    if (firstweekday > 3) monday += 7;
    return monday;
}

struct IsoDate {
    int year = 0, month = 0, day = 0;
};

bool isoweek_to_gregorian(int year, int week, int day, IsoDate &out) {
    if (year < 1 || year > 9999) return false;
    if (week < 1 || week > 53) return false;
    if (week == 53) {
        const long long first_weekday = ymd_to_ord(year, 1, 1) % 7;
        if (!(first_weekday == 4 ||
              (first_weekday == 3 && is_leap_year(year))))
            return false;
    }
    if (day < 1 || day > 7) return false;
    const long long ord =
        isoweek1_monday(year) + static_cast<long long>(week - 1) * 7 +
        (day - 1);
    ord_to_ymd(ord, out.year, out.month, out.day);
    return out.year >= 1 && out.year <= 9999;
}

// Strict n-digit ASCII slice parse (the C accelerator's digit rule — no
// signs/whitespace/underscores/Unicode digits).
bool digits_at(std::string_view text, std::size_t pos, std::size_t count,
               int &out) {
    if (pos + count > text.size()) return false;
    int value = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const char c = text[pos + i];
        if (!ascii_digit(c)) return false;
        value = value * 10 + (c - '0');
    }
    out = value;
    return true;
}

// _parse_isoformat_date — assumes ASCII text of length 7/8/10 (gated).
bool parse_iso_date(std::string_view text, IsoDate &out) {
    if (text.size() != 7 && text.size() != 8 && text.size() != 10)
        return false;
    for (const char c : text)
        if (static_cast<unsigned char>(c) > 0x7f) return false;
    int year = 0;
    if (!digits_at(text, 0, 4, year)) return false;
    const bool has_sep = text[4] == '-';
    std::size_t pos = 4 + (has_sep ? 1 : 0);
    if (pos < text.size() && text[pos] == 'W') {
        ++pos;
        int week = 0;
        if (!digits_at(text, pos, 2, week)) return false;
        pos += 2;
        int day = 1;
        if (text.size() > pos) {
            const bool dash = text[pos] == '-';
            if (dash != has_sep) return false;
            pos += has_sep ? 1 : 0;
            if (!digits_at(text, pos, 1, day)) return false;
        }
        return isoweek_to_gregorian(year, week, day, out);
    }
    int month = 0;
    if (!digits_at(text, pos, 2, month)) return false;
    pos += 2;
    const bool dash = pos < text.size() && text[pos] == '-';
    if (dash != has_sep) return false;
    pos += has_sep ? 1 : 0;
    int day = 0;
    if (!digits_at(text, pos, 2, day)) return false;
    if (month < 1 || month > 12 || day < 1 ||
        day > days_in_month(year, month))
        return false;
    if (year < 1 || year > 9999) return false;
    out = IsoDate{.year = year, .month = month, .day = day};
    return true;
}

struct IsoTime {
    int hour = 0, minute = 0, second = 0, micro = 0;
    // nanoseconds offset from UTC; null when the input had no zone.
    bool has_offset = false;
    long long offset_us = 0;
};

// parse_hh_mm_ss_ff — HH[:?MM[:?SS[{.,}fff[fff]]]] (CPython 3.13
// Modules/_datetimemodule.c:994-1062). Returns 0 on success at
// end-of-input, 1 when a non-NUL character remains (the caller tolerates
// that only when a timezone marker follows), -3 on a failed time
// component and -4 on a malformed time separator — the same rv
// convention as the C routine.
int parse_hh_mm_ss_ff(std::string_view text, std::array<int, 4> &comps) {
    comps = {0, 0, 0, 0};
    std::size_t pos = 0;
    bool has_sep = false;
    for (int comp = 0; comp < 3; ++comp) {
        if (text.size() - pos < 2) return -3;
        if (!digits_at(text, pos, 2, comps[static_cast<std::size_t>(comp)]))
            return -3;
        pos += 2;
        const char c = pos < text.size() ? text[pos] : '\0';
        ++pos;  // consume c like C's *(p++); may step one past the end
        if (comp == 0) has_sep = c == ':';
        // 3.13 checks the decimal mark FIRST: '.',',' after any component
        // (hour, minute or second) breaks into the fraction tail, so
        // '12.5' -> 12:00:00.500000 and '12:34,5' -> 12:34:00.500000.
        if (c == '.' || c == ',') {
            if (pos >= text.size()) return -3;  // mark not followed by a digit
            break;
        } else if (pos >= text.size()) {
            return c != '\0';
        } else if (has_sep && c == ':') {
            // At comp == 2 the loop simply ends, so a ':' after the
            // seconds falls into fraction parsing ('12:34:56:78' ->
            // 12:34:56.780000) — accepted, unlike the 3.14 -4 reject.
            continue;
        } else if (!has_sep) {
            --pos;
        } else {
            return -4;  // malformed time separator
        }
    }
    // Fractional component: the first min(remainder, 6) digits scale to
    // microseconds, surplus digits are skipped, and only NUL/end
    // terminates the tail (mirrors "while (is_digit(*p)) ++p; return
    // *p != '\0'").
    const std::size_t take =
        std::min(text.size() - pos, static_cast<std::size_t>(6));
    int micro = 0;
    if (!digits_at(text, pos, take, micro)) return -3;
    if (take < 6) {
        static constexpr std::array<int, 6> kFractionCorrection{
            0, 100000, 10000, 1000, 100, 10};
        micro *= kFractionCorrection[take];
    }
    pos += take;
    while (pos < text.size() && ascii_digit(text[pos])) ++pos;
    comps[3] = micro;
    return pos < text.size() && text[pos] != '\0';
}

// _parse_isoformat_time — HH[:MM[:SS[.ffffff]]][{+|-|Z}offset]
bool parse_iso_time(std::string_view text, IsoTime &out) {
    if (text.size() < 2) return false;
    std::size_t tz_pos = 0;
    const std::size_t minus = text.find('-');
    const std::size_t plus = text.find('+');
    const std::size_t zulu = text.find('Z');
    if (minus != std::string_view::npos)
        tz_pos = minus + 1;
    else if (plus != std::string_view::npos)
        tz_pos = plus + 1;
    else if (zulu != std::string_view::npos)
        tz_pos = zulu + 1;
    const std::string_view timestr =
        tz_pos > 0 ? text.substr(0, tz_pos - 1) : text;
    std::array<int, 4> comps{};
    const int rv = parse_hh_mm_ss_ff(timestr, comps);
    if (rv < 0) return false;
    out.hour = comps[0];
    out.minute = comps[1];
    out.second = comps[2];
    out.micro = comps[3];
    // No 3.13 hour-24 special case: '24:xx' parses here and the callers'
    // field check rejects it ("hour must be in 0..23"-class).
    if (tz_pos == text.size() && text.back() == 'Z') {
        out.has_offset = true;
        out.offset_us = 0;
    } else if (tz_pos > 0) {
        const std::string_view tzstr = text.substr(tz_pos);
        const char sign_char = text[tz_pos - 1];
        if (tzstr.size() == 0 || tzstr.size() == 1 || tzstr.size() == 3 ||
            sign_char == 'Z')
            return false;
        std::array<int, 4> tz_comps{};
        if (parse_hh_mm_ss_ff(tzstr, tz_comps) != 0) return false;
        if (tz_comps[0] == 0 && tz_comps[1] == 0 && tz_comps[2] == 0 &&
            tz_comps[3] == 0) {
            out.has_offset = true;
            out.offset_us = 0;
        } else {
            const long long offset =
                (static_cast<long long>(tz_comps[0]) * 3600 +
                 static_cast<long long>(tz_comps[1]) * 60 +
                 tz_comps[2]) *
                    1000000LL +
                tz_comps[3];
            if (offset >= 86400LL * 1000000LL) return false;
            out.has_offset = true;
            out.offset_us = sign_char == '-' ? -offset : offset;
        }
    } else if (rv != 0) {
        // Characters remain after the time component and there is no
        // timezone marker to explain them — parse_isoformat_time's -5.
        return false;
    }
    return true;
}

// _find_isoformat_datetime_separator — codepoint-indexed port (the C
// version indexes chars; everything before the separator is ASCII in any
// acceptable input).
int find_datetime_separator(const std::vector<char32_t> &text) {
    const std::size_t len = text.size();
    if (len == 7) return 7;
    if (text[4] == U'-') {
        if (text[5] == U'W') {
            if (len < 8) return -1;
            if (len > 8 && text[8] == U'-') {
                if (len == 9) return -1;
                if (len > 10 && text[10] >= U'0' && text[10] <= U'9')
                    return 8;
                return 10;
            }
            return 8;
        }
        return 10;
    }
    if (text[4] == U'W') {
        std::size_t idx = 7;
        while (idx < len && text[idx] >= U'0' && text[idx] <= U'9') ++idx;
        if (idx < 9) return static_cast<int>(idx);
        return idx % 2 == 0 ? 7 : 8;
    }
    return 8;
}

struct IsoDateTime {
    IsoDate date;
    IsoTime time;
};

bool valid_time_fields(const IsoTime &t) {
    return t.hour >= 0 && t.hour <= 23 && t.minute >= 0 && t.minute <= 59 &&
           t.second >= 0 && t.second <= 59 && t.micro >= 0 &&
           t.micro <= 999999;
}

bool valid_date_fields(const IsoDate &d) {
    return d.year >= 1 && d.year <= 9999 && d.month >= 1 && d.month <= 12 &&
           d.day >= 1 && d.day <= days_in_month(d.year, d.month);
}

bool decode_cps(std::string_view text, std::vector<char32_t> &out) {
    if (!valid_utf8(text)) return false;
    out.clear();
    std::size_t pos = 0;
    while (pos < text.size()) out.push_back(utf8_decode(text, pos));
    return true;
}

std::string cps_to_utf8(std::span<const char32_t> cps) {
    std::string out;
    for (const char32_t cp : cps) {
        if (cp < 0x80) {
            out += static_cast<char>(cp);
        } else if (cp < 0x800) {
            out += static_cast<char>(0xc0U | (cp >> 6U));
            out += static_cast<char>(0x80U | (cp & 0x3fU));
        } else if (cp < 0x10000) {
            out += static_cast<char>(0xe0U | (cp >> 12U));
            out += static_cast<char>(0x80U | ((cp >> 6U) & 0x3fU));
            out += static_cast<char>(0x80U | (cp & 0x3fU));
        } else {
            out += static_cast<char>(0xf0U | (cp >> 18U));
            out += static_cast<char>(0x80U | ((cp >> 12U) & 0x3fU));
            out += static_cast<char>(0x80U | ((cp >> 6U) & 0x3fU));
            out += static_cast<char>(0x80U | (cp & 0x3fU));
        }
    }
    return out;
}

// datetime.fromisoformat — full CPython 3.13 port including the
// date-only, week-date and timezone paths. '24:xx' time components parse
// and are then rejected by valid_time_fields (the 3.13 "hour must be in
// 0..23" path); 3.14's next-day wrap does not exist here.
bool parse_iso_datetime(std::string_view text, IsoDateTime &out) {
    std::vector<char32_t> cps;
    if (!decode_cps(text, cps) || cps.size() < 7) return false;
    const int separator = find_datetime_separator(cps);
    if (separator < 0) return false;
    const std::span<const char32_t> all{cps};
    // CPython slices iso_str[0:separator] / iso_str[separator+1:] which
    // clamp at the string end; the separator index can exceed the input
    // length for short strings (e.g. '2000-01-0'), so clamp here too.
    const std::size_t sep_index = std::min(
        static_cast<std::size_t>(separator), cps.size());
    const std::string dstr = cps_to_utf8(all.subspan(0, sep_index));
    IsoDate date{};
    if (!parse_iso_date(dstr, date)) return false;
    IsoTime time{};
    if (sep_index < cps.size()) {
        // A separator codepoint was consumed: the time part is mandatory —
        // '2000-01-01T' (empty tstr) is an error, as is a trailing sep.
        const std::string tstr =
            cps_to_utf8(all.subspan(sep_index + 1));
        if (!parse_iso_time(tstr, time)) return false;
    }
    if (!valid_date_fields(date) || !valid_time_fields(time)) return false;
    out.date = date;
    out.time = time;
    return true;
}

// time.fromisoformat — one leading 'T' stripped, then _parse_isoformat_time
// and the hour/minute/second/microsecond field bound.
bool parse_iso_time_field(std::string_view text, IsoTime &out) {
    std::string_view body = text;
    if (!body.empty() && body.front() == 'T') body.remove_prefix(1);
    if (!parse_iso_time(body, out)) return false;
    return valid_time_fields(out);
}

// isoformat() renderings — the spellings to_json_value emits.
std::string format_2d(int value) {
    std::string out;
    out += static_cast<char>('0' + value / 10);
    out += static_cast<char>('0' + value % 10);
    return out;
}
std::string format_4d(int value) {
    std::string out;
    out += static_cast<char>('0' + value / 1000 % 10);
    out += static_cast<char>('0' + value / 100 % 10);
    out += static_cast<char>('0' + value / 10 % 10);
    out += static_cast<char>('0' + value % 10);
    return out;
}
std::string format_6d(int value) {
    std::string out;
    out += static_cast<char>('0' + value / 100000 % 10);
    out += static_cast<char>('0' + value / 10000 % 10);
    out += static_cast<char>('0' + value / 1000 % 10);
    out += static_cast<char>('0' + value / 100 % 10);
    out += static_cast<char>('0' + value / 10 % 10);
    out += static_cast<char>('0' + value % 10);
    return out;
}

std::string isoformat_time(const IsoTime &t) {
    std::string out = format_2d(t.hour) + ':' + format_2d(t.minute) + ':' +
                      format_2d(t.second);
    if (t.micro != 0) out += '.' + format_6d(t.micro);
    if (t.has_offset) {
        long long off = t.offset_us;
        out += off < 0 ? '-' : '+';
        if (off < 0) off = -off;
        const long long total_s = off / 1000000LL;
        const long long us = off % 1000000LL;
        out += format_2d(static_cast<int>(total_s / 3600));
        out += ':';
        out += format_2d(static_cast<int>((total_s / 60) % 60));
        const long long seconds = total_s % 60;
        if (seconds != 0 || us != 0) {
            out += ':';
            out += format_2d(static_cast<int>(seconds));
            if (us != 0) out += '.' + format_6d(static_cast<int>(us));
        }
    }
    return out;
}

std::string isoformat_date(const IsoDate &d) {
    return format_4d(d.year) + '-' + format_2d(d.month) + '-' +
           format_2d(d.day);
}

std::string isoformat_datetime(const IsoDateTime &dt) {
    return isoformat_date(dt.date) + 'T' + isoformat_time(dt.time);
}

// ---------------------------------------------------------------------------
// Recording tag decode — from_json_value (json_codec.py:54-87).

long long floor_div(long long a, long long b) {
    const long long q = a / b;
    const long long r = a % b;
    return (r != 0 && ((r < 0) != (b < 0))) ? q - 1 : q;
}

// timedelta(days, seconds, microseconds) -> normalized components, with
// CPython's |days| <= 999999999 bound on the RESULT. Pure int64
// arithmetic: the floor-div cascade keeps every partial within range.
// NOLINTBEGIN(bugprone-easily-swappable-parameters) component order mirrors timedelta(days,seconds,microseconds)
bool timedelta_normalize(std::int64_t d, std::int64_t s, std::int64_t u,
                         std::int64_t &out_days, std::int64_t &out_seconds,
                         std::int64_t &out_micros) {
// NOLINTEND(bugprone-easily-swappable-parameters)
    const long long q2 = floor_div(u, 1000000);
    const long long u1 = u - q2 * 1000000;
    const long long q1 = floor_div(s, 86400);
    const long long s1 = s - q1 * 86400;
    const long long s_join = s1 + q2;
    const long long q4 = floor_div(s_join, 86400);
    const long long s2 = s_join - q4 * 86400;
    // |q1|, |q4| <= ~1.1e14; |d| <= 9.2e18. The final bound check (+-1e9
    // days) rejects any overflow candidate before int64 addition could.
    if (d > 999999999LL * 86400LL || d < -999999999LL * 86400LL ||
        q1 + q4 > 999999999LL * 86400LL ||
        q1 + q4 < -999999999LL * 86400LL)
        return false;
    const long long total_days = d + q1 + q4;
    if (total_days > 999999999LL || total_days < -999999999LL) return false;
    out_days = total_days;
    out_seconds = s2;
    out_micros = u1;
    return true;
}

bool timedelta_components(const bio::Number &days, const bio::Number &seconds,
                          const bio::Number &micros, std::int64_t &out_days,
                          std::int64_t &out_seconds, std::int64_t &out_micros) {
    std::int64_t d = 0, s = 0, u = 0;
    try {
        d = bio::number_i64(days, "timedelta.days");
        s = bio::number_i64(seconds, "timedelta.seconds");
        u = bio::number_i64(micros, "timedelta.microseconds");
    } catch (const std::invalid_argument &) {
        return false;
    }
    return timedelta_normalize(d, s, u, out_days, out_seconds, out_micros);
}

Value decode_tagged(const Value::Object &object, const std::string &path);

// NOLINTNEXTLINE(misc-no-recursion) tree fold mirrors the recursive json_codec to_json_value walk
void fold_tags(Value &node, std::string &path) {
    if (auto *elements = std::get_if<Value::Array>(&node.data)) {
        for (std::size_t i = 0; i < elements->size(); ++i) {
            const std::size_t mark = path.size();
            path_index(path, i);
            fold_tags(elements->at(i), path);
            path.resize(mark);
        }
    } else if (auto *object = std::get_if<Value::Object>(&node.data)) {
        if (find_member(*object, bio::kRecordingTag) != nullptr) {
            node = decode_tagged(*object, path);
        } else {
            for (auto &[key, member] : *object) {
                const std::size_t mark = path.size();
                path_key(path, key);
                fold_tags(member, path);
                path.resize(mark);
            }
        }
    }
}

// loads()' parse_float=finite check (json_codec.py:108-112): every
// float-spelled lexeme in the tree must land inside finite binary64 —
// integer lexemes are unconstrained.
// NOLINTNEXTLINE(misc-no-recursion) finite walk mirrors the recursive loads() parse_float check
void enforce_finite(const Value &node, std::string &path) {
    if (const auto *number = std::get_if<bio::Number>(&node.data)) {
        if (float_token(number->lexeme) && !finite_lexeme(number->lexeme))
            fail_codec("nonfinite JSON token must use a recording tag: " +
                       number->lexeme + " at " + path);
    } else if (const auto *elements =
                   std::get_if<Value::Array>(&node.data)) {
        for (std::size_t i = 0; i < elements->size(); ++i) {
            const std::size_t mark = path.size();
            path_index(path, i);
            enforce_finite(elements->at(i), path);
            path.resize(mark);
        }
    } else if (const auto *object =
                   std::get_if<Value::Object>(&node.data)) {
        for (const auto &[key, member] : *object) {
            const std::size_t mark = path.size();
            path_key(path, key);
            enforce_finite(member, path);
            path.resize(mark);
        }
    }
}

bool has_only_members(const Value::Object &object,
                      std::initializer_list<std::string_view> names) {
    if (object.size() != names.size()) return false;
    for (const std::string_view name : names)
        if (find_member(object, name) == nullptr) return false;
    return true;
}

// NOLINTNEXTLINE(misc-no-recursion) decode recurses through fold_tags for escaped-object items
Value decode_tagged(const Value::Object &object, const std::string &path) {
    const std::string *kind = member_string(object, bio::kRecordingTag);
    if (kind == nullptr)
        fail_codec("invalid recording JSON tag at " + path);
    if (*kind == "float64" &&
        has_only_members(object, {bio::kRecordingTag, "value"})) {
        const std::string *value = member_string(object, "value");
        if (value != nullptr &&
            (*value == "nan" || *value == "+inf" || *value == "-inf"))
            return Value{bio::Tagged{.kind = "float64", .fields = {{"value", *value}}}};
        fail_codec("invalid tagged float64 at " + path);
    }
    if (*kind == "decimal" &&
        has_only_members(object, {bio::kRecordingTag, "value"})) {
        const Value *member = find_member(object, "value");
        std::string canonical;
        std::string diagnostic;
        if (const auto *text = std::get_if<std::string>(&member->data)) {
            if (!decimal_canonical(*text, canonical, &diagnostic))
                fail_codec("invalid tagged decimal at " + path + " (" +
                           diagnostic + ")");
        } else if (const auto *number =
                       std::get_if<bio::Number>(&member->data)) {
            canonical = decimal_from_number_lexeme(number->lexeme);
        } else if (const auto *boolean = std::get_if<bool>(&member->data)) {
            // Python Decimal(True) -> Decimal('1'), Decimal(False) -> '0'.
            canonical = *boolean ? "1" : "0";
        } else {
            fail_codec("invalid tagged decimal at " + path);
        }
        return Value{bio::Tagged{.kind = "decimal", .fields = {{"value", canonical}}}};
    }
    if ((*kind == "datetime" || *kind == "date" || *kind == "time") &&
        has_only_members(object, {bio::kRecordingTag, "value"})) {
        const std::string *value = member_string(object, "value");
        if (value == nullptr)
            fail_codec("invalid tagged " + *kind + " at " + path);
        if (*kind == "datetime") {
            IsoDateTime parsed{};
            if (!parse_iso_datetime(*value, parsed))
                fail_codec("invalid tagged datetime at " + path);
            return Value{bio::Tagged{
                .kind = "datetime",
                .fields = {{"value", isoformat_datetime(parsed)}}}};
        }
        if (*kind == "date") {
            IsoDate parsed{};
            if (!parse_iso_date(*value, parsed))
                fail_codec("invalid tagged date at " + path);
            return Value{
                bio::Tagged{.kind = "date",
                            .fields = {{"value", isoformat_date(parsed)}}}};
        }
        IsoTime parsed{};
        if (!parse_iso_time_field(*value, parsed))
            fail_codec("invalid tagged time at " + path);
        return Value{
            bio::Tagged{.kind = "time",
                        .fields = {{"value", isoformat_time(parsed)}}}};
    }
    if (*kind == "timedelta" &&
        has_only_members(object,
                         {bio::kRecordingTag, "days", "seconds",
                          "microseconds"})) {
        const Value *days = find_member(object, "days");
        const Value *seconds = find_member(object, "seconds");
        const Value *micros = find_member(object, "microseconds");
        const auto *days_num = std::get_if<bio::Number>(&days->data);
        const auto *seconds_num =
            std::get_if<bio::Number>(&seconds->data);
        const auto *micros_num = std::get_if<bio::Number>(&micros->data);
        std::int64_t d = 0, s = 0, u = 0;
        if (days_num == nullptr || seconds_num == nullptr ||
            micros_num == nullptr ||
            !timedelta_components(*days_num, *seconds_num, *micros_num, d,
                                  s, u))
            fail_codec("invalid tagged timedelta at " + path);
        return Value{bio::Tagged{
            .kind = "timedelta",
            .fields = {{"days", std::to_string(d)},
                       {"seconds", std::to_string(s)},
                       {"microseconds", std::to_string(u)}}}};
    }
    if (*kind == "bytes" &&
        has_only_members(object, {bio::kRecordingTag, "value"})) {
        const std::string *value = member_string(object, "value");
        if (value == nullptr)
            fail_codec("invalid tagged bytes at " + path);
        return Value{bio::Bytes{b64_decode_strict(*value, path)}};
    }
    if (*kind == "object" &&
        has_only_members(object, {bio::kRecordingTag, "items"})) {
        const Value *items_value = find_member(object, "items");
        const auto *items = std::get_if<Value::Array>(&items_value->data);
        if (items == nullptr)
            fail_codec("invalid escaped JSON object at " + path);
        Value::Object result;
        std::unordered_set<std::string> keys;
        for (std::size_t i = 0; i < items->size(); ++i) {
            const Value &pair_value = items->at(i);
            const auto *pair = std::get_if<Value::Array>(&pair_value.data);
            if (pair == nullptr || pair->size() != 2 ||
                std::get_if<std::string>(&pair->at(0).data) == nullptr)
                fail_codec("invalid escaped JSON object at " + path);
            const std::string &key =
                std::get<std::string>(pair->at(0).data);
            if (!keys.insert(key).second)
                fail_codec("duplicate JSON field: " + key);
            Value item = pair->at(1);
            std::string item_path = path;
            path_index(item_path, i);
            fold_tags(item, item_path);
            result.emplace_back(key, std::move(item));
        }
        return Value{std::move(result)};
    }
    fail_codec("invalid recording JSON tag at " + path);
}

// ---------------------------------------------------------------------------
// Serialization — dumps(value, sort_keys=True, ensure_ascii=False,
// separators=(',',':'), allow_nan=False) mirrored on the owned tree; the
// escape flag selects to_json_value's reserved-key escaping.

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (text, caller) order is a fixed emit convention
void emit_json_string(std::string &out, std::string_view text,
                      std::string_view caller) {
    if (!valid_utf8(text))
        fail_emit(caller, "invalid UTF-8 in JSON string");
    out += '"';
    for (const char ch : text) {
        const unsigned char c = static_cast<unsigned char>(ch);
        if (c == '"') {
            out += "\\\"";
        } else if (c == '\\') {
            out += "\\\\";
        } else if (c == '\b') {
            out += "\\b";
        } else if (c == '\f') {
            out += "\\f";
        } else if (c == '\n') {
            out += "\\n";
        } else if (c == '\r') {
            out += "\\r";
        } else if (c == '\t') {
            out += "\\t";
        } else if (c < 0x20) {
            constexpr std::string_view hex = "0123456789abcdef";
            const unsigned int code = static_cast<unsigned char>(c);
            out += "\\u00";
            out += hex[(code >> 4U) & 0x0fU];
            out += hex[code & 0x0fU];
        } else {
            out += static_cast<char>(c);
        }
    }
    out += '"';
}

const bio::Tagged::Field *tagged_field(const bio::Tagged &tagged,
                                       std::string_view name) {
    for (const auto &field : tagged.fields)
        if (field.first == name) return &field;
    return nullptr;
}

void emit_value(std::string &out, const Value &value, bool escape,
                std::string_view caller);

void emit_tagged(std::string &out, const bio::Tagged &tagged,
                 std::string_view caller) {
    const auto single_value = [&]() -> const std::string * {
        if (tagged.fields.size() != 1) return nullptr;
        const bio::Tagged::Field *field = tagged_field(tagged, "value");
        return field == nullptr ? nullptr : &field->second;
    };
    const std::string &kind = tagged.kind;
    const std::string fail_message =
        "unsupported recording tag '" + kind + "'";
    if (kind == "float64") {
        const std::string *value = single_value();
        if (value == nullptr ||
            (*value != "nan" && *value != "+inf" && *value != "-inf"))
            fail_emit(caller, fail_message);
        out += R"({"$bike_recording":"float64","value":)";
        emit_json_string(out, *value, caller);
        out += '}';
        return;
    }
    if (kind == "decimal" || kind == "datetime" || kind == "date" ||
        kind == "time" || kind == "bytes") {
        const std::string *value = single_value();
        if (value == nullptr) fail_emit(caller, fail_message);
        // Validate the stored domain spelling and emit the canonical form
        // the domain object would produce (str(Decimal), isoformat()) —
        // decoded Tags already store the canonical spelling; constructed
        // ones are normalized here. bytes keeps its lexeme verbatim.
        std::string canonical;
        const std::string *emitted = value;
        if (kind == "bytes") {
            for (const char c : *value)
                if (c != '=' && b64_digit(c) < 0)
                    fail_emit(caller, fail_message);
            // Full strict-shape checking happens on decode; emit needs the
            // alphabet guarantee only — a malformed padding field still
            // serializes losslessly for canonical hashing.
        } else if (kind == "decimal") {
            if (!decimal_canonical(*value, canonical, nullptr))
                fail_emit(caller, fail_message);
            emitted = &canonical;
        } else if (kind == "datetime") {
            IsoDateTime parsed{};
            if (!parse_iso_datetime(*value, parsed))
                fail_emit(caller, fail_message);
            canonical = isoformat_datetime(parsed);
            emitted = &canonical;
        } else if (kind == "date") {
            IsoDate parsed{};
            if (!parse_iso_date(*value, parsed))
                fail_emit(caller, fail_message);
            canonical = isoformat_date(parsed);
            emitted = &canonical;
        } else {
            IsoTime parsed{};
            if (!parse_iso_time_field(*value, parsed))
                fail_emit(caller, fail_message);
            canonical = isoformat_time(parsed);
            emitted = &canonical;
        }
        out += R"({"$bike_recording":")";
        out += kind;
        out += R"(","value":)";
        emit_json_string(out, *emitted, caller);
        out += '}';
        return;
    }
    if (kind == "timedelta") {
        const bio::Tagged::Field *days = tagged_field(tagged, "days");
        const bio::Tagged::Field *seconds = tagged_field(tagged, "seconds");
        const bio::Tagged::Field *micros =
            tagged_field(tagged, "microseconds");
        if (days == nullptr || seconds == nullptr || micros == nullptr ||
            tagged.fields.size() != 3)
            fail_emit(caller, fail_message);
        const auto integer = [](const std::string &field,
                                std::int64_t &out_value) {
            std::size_t pos = 0;
            bool negative = false;
            if (pos < field.size() && field[pos] == '-') {
                negative = true;
                ++pos;
            }
            const std::size_t digits = field.size() - pos;
            if (digits == 0 || digits > 19) return false;
            std::uint64_t accum = 0;
            for (; pos < field.size(); ++pos) {
                const char c = field[pos];
                if (!ascii_digit(c)) return false;
                accum = accum * 10U + static_cast<std::uint64_t>(c - '0');
            }
            if (negative) {
                if (accum > 9223372036854775808ULL) return false;
                out_value =
                    accum == 9223372036854775808ULL
                        ? std::numeric_limits<std::int64_t>::min()
                        : -static_cast<std::int64_t>(accum);
            } else {
                if (accum > 9223372036854775807ULL) return false;
                out_value = static_cast<std::int64_t>(accum);
            }
            return true;
        };
        std::int64_t d = 0, s = 0, u = 0;
        if (!integer(days->second, d) || !integer(seconds->second, s) ||
            !integer(micros->second, u))
            fail_emit(caller, fail_message);
        // A Tag stores a normalized timedelta (the domain object always
        // is); re-run the constructor normalization + bound on
        // constructed fields so non-canonical fields fail closed.
        std::int64_t nd = 0, ns = 0, nu = 0;
        if (!timedelta_normalize(d, s, u, nd, ns, nu))
            fail_emit(caller, fail_message);
        out += R"({"$bike_recording":"timedelta","days":)";
        out += std::to_string(nd);
        out += R"(,"microseconds":)";
        out += std::to_string(nu);
        out += R"(,"seconds":)";
        out += std::to_string(ns);
        out += '}';
        return;
    }
    fail_emit(caller, fail_message);
}

// NOLINTNEXTLINE(misc-no-recursion) emit recurses through emit_value for nested members
void emit_object(std::string &out, const Value::Object &object, bool escape,
                 std::string_view caller) {
    // Duplicate keys cannot exist in a parsed Object but a constructed one
    // may carry them; serialization fails closed either way.
    std::unordered_set<std::string> seen;
    for (const auto &member : object)
        if (!seen.insert(member.first).second)
            fail_emit(caller, "duplicate JSON field: " + member.first);
    if (escape && find_member(object, bio::kRecordingTag) != nullptr) {
        // to_json_value's escape form; 'items' is a list, so member order
        // is the insertion order, not sorted.
        out += R"({"$bike_recording":"object","items":[)";
        bool first = true;
        for (const auto &[key, member] : object) {
            if (!first) out += ',';
            first = false;
            out += '[';
            emit_json_string(out, key, caller);
            out += ',';
            emit_value(out, member, escape, caller);
            out += ']';
        }
        out += "]}";
        return;
    }
    // Sort indices, not member pointers: the comparator output is the
    // deterministic sort_keys contract and no pointer ordering leaks in.
    std::vector<std::size_t> order;
    order.reserve(object.size());
    for (std::size_t i = 0; i < object.size(); ++i) order.push_back(i);
    std::ranges::stable_sort(order, [&object](std::size_t a, std::size_t b) {
        return object[a].first < object[b].first;
    });
    out += '{';
    bool first = true;
    for (const std::size_t index : order) {
        const auto &[key, member] = object[index];
        if (!first) out += ',';
        first = false;
        emit_json_string(out, key, caller);
        out += ':';
        emit_value(out, member, escape, caller);
    }
    out += '}';
}

// NOLINTNEXTLINE(misc-no-recursion) emit recurses for nested arrays/objects
void emit_value(std::string &out, const Value &value, bool escape,
                std::string_view caller) {
    if (std::holds_alternative<std::monostate>(value.data)) {
        out += "null";
    } else if (const auto *boolean = std::get_if<bool>(&value.data)) {
        out += *boolean ? "true" : "false";
    } else if (const auto *number = std::get_if<bio::Number>(&value.data)) {
        check_lexeme(number->lexeme, escape, caller);
        out += number->lexeme;
    } else if (const auto *text = std::get_if<std::string>(&value.data)) {
        emit_json_string(out, *text, caller);
    } else if (const auto *elements =
                   std::get_if<Value::Array>(&value.data)) {
        out += '[';
        bool first = true;
        for (const Value &element : *elements) {
            if (!first) out += ',';
            first = false;
            emit_value(out, element, escape, caller);
        }
        out += ']';
    } else if (const auto *object =
                   std::get_if<Value::Object>(&value.data)) {
        emit_object(out, *object, escape, caller);
    } else if (const auto *bytes = std::get_if<bio::Bytes>(&value.data)) {
        out += R"({"$bike_recording":"bytes","value":")";
        out += b64_encode(bytes->value);
        out += R"("})";
    } else if (const auto *tagged = std::get_if<bio::Tagged>(&value.data)) {
        emit_tagged(out, *tagged, caller);
    }
}

} // namespace

namespace bike::io {

Value parse_json(std::string_view source) {
    if (source.size() > kMaxJsonBytes)
        fail_parse("input exceeds the 512 MiB boundary limit");
    // A raw NUL byte is never valid JSON (the \u0000 escape is the only
    // legal form) but RapidJSON's stream reports it as end-of-input — a
    // document followed by NUL + garbage would parse as singular. Reject
    // it explicitly; Python's json.loads rejects the same bytes.
    if (source.find('\0') != std::string_view::npos)
        fail_parse("raw NUL byte in JSON input");
    // EncodedInputStream would skip a UTF-8 BOM; Python's loads on str
    // input raises for a leading ﻿ too — reject it as malformed.
    if (source.size() >= 3 &&
        static_cast<unsigned char>(source[0]) == 0xefU &&
        static_cast<unsigned char>(source[1]) == 0xbbU &&
        static_cast<unsigned char>(source[2]) == 0xbfU)
        fail_parse("UTF-8 BOM prefix is not valid JSON input");
    rapidjson::MemoryStream memory(source.data(), source.size());
    rapidjson::EncodedInputStream<rapidjson::UTF8<>, rapidjson::MemoryStream>
        stream(memory);
    SaxHandler handler;
    rapidjson::Reader reader;
    constexpr unsigned kFlags = rapidjson::kParseNumbersAsStringsFlag |
                                rapidjson::kParseValidateEncodingFlag;
    const auto result = reader.Parse<kFlags>(stream, handler);
    if (!result) {
        if (!handler.failure.empty()) fail_parse(handler.failure);
        // Error positions are the RapidJSON scanner's byte offsets, not
        // CPython JSONDecodeError positions — no end/end-1 mirroring is
        // carried (W1.2x item 9: the 3.13-vs-3.14 py_scanstring offset
        // difference has no ported surface; the C accelerator reports
        // the control character's own index in both versions anyway).
        fail_parse("invalid JSON at offset " +
                   std::to_string(result.Offset()) + ": " +
                   rapidjson::GetParseError_En(result.Code()));
    }
    return std::move(handler.root);
}

Value parse_recording_json(std::string_view source) {
    Value root = parse_json(source);
    std::string path = "$";
    enforce_finite(root, path);
    fold_tags(root, path);
    return root;
}

std::string canonical_json(const Value &value) {
    std::string out;
    out.reserve(256);
    emit_value(out, value, false, "bike::io::canonical_json");
    return out;
}

std::string recording_json(const Value &value) {
    std::string out;
    out.reserve(256);
    emit_value(out, value, true, "bike::io::recording_json");
    return out;
}

} // namespace bike::io
