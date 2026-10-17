// Decimal arithmetic for batch identities/ranges. The public seed domain is
// wider than int64; only the number of generated jobs has a machine bound.
#pragma once
#include <algorithm>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace bike::batch::integer_detail {
inline std::string normalize(std::string_view value) {
    bool negative = false;
    if (!value.empty() && (value.front() == '+' || value.front() == '-')) {
        negative = value.front() == '-'; value.remove_prefix(1);
    }
    if (value.empty() || !std::ranges::all_of(value, [](char c) { return c >= '0' && c <= '9'; }))
        throw std::invalid_argument("batch integer must contain decimal digits");
    const auto first = value.find_first_not_of('0');
    if (first == std::string_view::npos) return "0";
    return (negative ? "-" : "") + std::string(value.substr(first));
}
inline std::string_view magnitude(std::string_view value) {
    return value.starts_with('-') ? value.substr(1) : value;
}
inline int compare_magnitude(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) return a.size() < b.size() ? -1 : 1;
    return a == b ? 0 : a < b ? -1 : 1;
}
// Arguments to arithmetic functions are normalized decimal strings.
inline int compare(std::string_view a, std::string_view b) {
    const bool negative_a = a.starts_with('-'), negative_b = b.starts_with('-');
    if (negative_a != negative_b) return negative_a ? -1 : 1;
    const auto result = compare_magnitude(magnitude(a), magnitude(b));
    return negative_a ? -result : result;
}
inline std::string add_magnitude(std::string_view a, std::string_view b) {
    std::string result; result.reserve(std::max(a.size(), b.size()) + 1);
    std::size_t ai = a.size(), bi = b.size(); unsigned carry = 0;
    while (ai || bi || carry) {
        unsigned sum = carry;
        if (ai) sum += static_cast<unsigned>(a[--ai] - '0');
        if (bi) sum += static_cast<unsigned>(b[--bi] - '0');
        result.push_back(static_cast<char>('0' + sum % 10U)); carry = sum / 10U;
    }
    std::ranges::reverse(result); return result;
}
inline std::string subtract_magnitude(std::string_view a, std::string_view b) {
    std::string result; result.reserve(a.size());
    std::size_t ai = a.size(), bi = b.size(); int borrow = 0;
    while (ai) {
        int digit = (a[--ai] - '0') - borrow;
        if (bi) digit -= b[--bi] - '0';
        borrow = digit < 0 ? 1 : 0; if (borrow) digit += 10;
        result.push_back(static_cast<char>('0' + digit));
    }
    while (result.size() > 1 && result.back() == '0') result.pop_back();
    std::ranges::reverse(result); return result;
}
inline std::string add(std::string_view a, std::string_view b) {
    const bool negative_a = a.starts_with('-'), negative_b = b.starts_with('-');
    const auto ma = magnitude(a), mb = magnitude(b);
    if (negative_a == negative_b) {
        auto result = add_magnitude(ma, mb);
        if (negative_a && result != "0") result.insert(result.begin(), '-');
        return result;
    }
    const auto order = compare_magnitude(ma, mb);
    if (order == 0) return "0";
    auto result = order > 0 ? subtract_magnitude(ma, mb) : subtract_magnitude(mb, ma);
    if (order > 0 ? negative_a : negative_b) result.insert(result.begin(), '-');
    return result;
}
inline unsigned digit_value(char c) {
    if (c >= '0' && c <= '9') return static_cast<unsigned>(c - '0');
    if (c >= 'a' && c <= 'f') return static_cast<unsigned>(c - 'a') + 10U;
    if (c >= 'A' && c <= 'F') return static_cast<unsigned>(c - 'A') + 10U;
    return 16U;
}
// Recognize exactly a TOML integer value token. Malformed integers are left
// untouched so the authoritative TOML parser supplies the syntax error.
inline std::optional<std::string> toml_token(std::string_view token) {
    if (token.empty()) return {};
    bool signed_token = false, negative = false;
    if (token.front() == '+' || token.front() == '-') {
        signed_token = true; negative = token.front() == '-'; token.remove_prefix(1);
    }
    if (token.empty()) return {};
    unsigned radix = 10;
    if (token.size() > 1 && token.front() == '0') {
        if (token[1] == 'x') radix = 16;
        else if (token[1] == 'o') radix = 8;
        else if (token[1] == 'b') radix = 2;
        if (radix != 10) { if (signed_token) return {}; token.remove_prefix(2); }
    }
    std::string digits; bool previous_digit = false;
    for (const char c : token) {
        if (c == '_') { if (!previous_digit) return {}; previous_digit = false; continue; }
        if (digit_value(c) >= radix) return {};
        digits.push_back(c); previous_digit = true;
    }
    if (!previous_digit || digits.empty() || (radix == 10 && digits.size() > 1 && digits.front() == '0')) return {};
    if (radix == 10) return normalize((negative ? "-" : "") + digits);
    std::string decimal = "0";
    for (const char c : digits) {
        unsigned carry = digit_value(c);
        for (std::size_t i = decimal.size(); i > 0; --i) {
            const unsigned value = static_cast<unsigned>(decimal[i-1] - '0') * radix + carry;
            decimal[i-1] = static_cast<char>('0' + value % 10U); carry = value / 10U;
        }
        while (carry) { decimal.insert(decimal.begin(), static_cast<char>('0' + carry % 10U)); carry /= 10U; }
    }
    return normalize(decimal);
}
} // namespace bike::batch::integer_detail
