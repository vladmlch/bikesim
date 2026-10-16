// random/seed_sequence.cpp — bit_generator.pyx SeedSequence ported
// statement-for-statement: entropy coercion to little-endian uint32 words,
// get_assembled_entropy's spawn-key zero-padding, mix_entropy's
// INIT_A/MULT_A hashmix pool fill, generate_state(uint32|uint64) and
// spawn()'s key-extension bookkeeping. Constant order and unsigned
// wraparound are the bitwise contract; every value stays in uint32 lanes.

#include "random/numpy_compat.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bike::random {
namespace {

// bit_generator.pyx module constants (DEFAULT_POOL_SIZE is the fixed 4).
constexpr std::uint32_t kInitA = 0x43b0d7e5U;
constexpr std::uint32_t kMultA = 0x931e8875U;
constexpr std::uint32_t kInitB = 0x8b51f9ddU;
constexpr std::uint32_t kMultB = 0x58f38dedU;
constexpr std::uint32_t kMixMultL = 0xca01f9ddU;
constexpr std::uint32_t kMixMultR = 0x4973f715U;
constexpr std::size_t kPoolSize = 4;
constexpr std::uint32_t kXshift = 16;

// cdef uint32_t hashmix(value, hash_const): the multiplier carries forward
// between calls — the const is input-output, matching hash_const[0].
std::uint32_t hashmix(std::uint32_t value, std::uint32_t &hash_const) {
    value ^= hash_const;
    hash_const *= kMultA;
    value *= hash_const;
    value ^= value >> kXshift;
    return value;
}

// cdef uint32_t mix(x, y).
std::uint32_t mix(std::uint32_t x, std::uint32_t y) {
    const std::uint32_t result = kMixMultL * x - kMixMultR * y;
    return result ^ (result >> kXshift);
}

// _int_to_uint32_array(n) for a uint64 input: little-endian words, [0] for
// zero, never empty.
std::vector<std::uint32_t> uint64_words(std::uint64_t value) {
    std::vector<std::uint32_t> words{static_cast<std::uint32_t>(value)};
    const std::uint32_t high = static_cast<std::uint32_t>(value >> 32U);
    if (high != 0U)
        words.push_back(high);
    return words;
}

// CPython's "invalid literal for int() with base N" ValueError shape.
[[noreturn]] void invalid_literal(std::string_view text, int base) {
    throw std::invalid_argument("invalid literal for int() with base " +
                                std::to_string(base) + ": '" +
                                std::string(text) + "'");
}

// int(x, base) after _coerce_to_uint32_array's string gates selected it.
// CPython strips ASCII whitespace, takes an optional sign and (base 16) an
// optional '0x'/'0X' prefix, then digits separated by single underscores —
// so '42 ', '4_2' and '0x5_5' all parse. The caller's gates make signs
// unreachable, but the grammar is honored anyway so a '-' still lands on
// _int_to_uint32_array's "expected non-negative integer" ValueError.
std::vector<std::uint32_t> python_int_words(std::string_view text, int base) {
    const auto is_space = [](char c) {
        return c == ' ' || c == '\t' || c == '\n' || c == '\r' ||
               c == '\v' || c == '\f';
    };
    std::size_t begin = 0;
    std::size_t end = text.size();
    while (begin < end && is_space(text[begin]))
        ++begin;
    while (end > begin && is_space(text[end - 1U]))
        --end;
    const std::string_view body = text.substr(begin, end - begin);
    if (body.empty())
        invalid_literal(text, base);
    std::size_t pos = 0;
    bool negative = false;
    if (body[pos] == '+' || body[pos] == '-') {
        negative = body[pos] == '-';
        ++pos;
    }
    std::vector<std::uint32_t> words{0U};
    bool prev_digit = false;
    std::size_t digit_count = 0;
    if (base == 16 && pos + 2U <= body.size() && body[pos] == '0' &&
        (body[pos + 1U] == 'x' || body[pos + 1U] == 'X')) {
        pos += 2U;
        // CPython also accepts a single underscore immediately after a
        // base prefix ('0x_5' == 5); it still must be followed by a digit.
        prev_digit = true;
    }
    for (; pos < body.size(); ++pos) {
        const char c = body[pos];
        if (c == '_') {
            // Underscores only sit between digits.
            if (!prev_digit)
                invalid_literal(text, base);
            prev_digit = false;
            continue;
        }
        int digit = -1;
        if (c >= '0' && c <= '9')
            digit = c - '0';
        else if (base == 16 && c >= 'a' && c <= 'f')
            digit = c - 'a' + 10;
        else if (base == 16 && c >= 'A' && c <= 'F')
            digit = c - 'A' + 10;
        if (digit < 0)
            invalid_literal(text, base);
        prev_digit = true;
        ++digit_count;
        // Arbitrary-precision little-endian accumulation — the same
        // decomposition _int_to_uint32_array performs on the int value.
        std::uint64_t carry = static_cast<std::uint64_t>(digit);
        for (std::uint32_t &word : words) {
            const std::uint64_t next =
                static_cast<std::uint64_t>(word) *
                    static_cast<std::uint64_t>(base) +
                carry;
            word = static_cast<std::uint32_t>(next);
            carry = next >> 32U;
        }
        if (carry != 0U)
            words.push_back(static_cast<std::uint32_t>(carry));
    }
    // '0x', '0x_', '_' and '5_' all fail here exactly like CPython.
    if (digit_count == 0U || !prev_digit)
        invalid_literal(text, base);
    if (negative)
        throw std::invalid_argument("expected non-negative integer");
    return words;
}

// _coerce_to_uint32_array's string element grammar: a '0x'-prefixed string
// parses via int(x, 16); a DECIMAL_RE match ([0-9]+ anchored at position 0)
// parses via int(x); anything else is ValueError("unrecognized seed
// string"). The decimal gate accepts any string whose FIRST character is an
// ASCII digit — trailing junk then fails inside int() instead.
std::vector<std::uint32_t> string_words(std::string_view text) {
    if (text.empty())
        throw std::invalid_argument("unrecognized seed string");
    if (text.starts_with("0x"))
        return python_int_words(text, 16);
    if (text.front() >= '0' && text.front() <= '9')
        return python_int_words(text, 10);
    throw std::invalid_argument("unrecognized seed string");
}

// get_assembled_entropy(): run entropy is zero-padded to the pool size only
// when a spawn key is present (the gh-16539 fix preserving the unspawned
// small-entropy streams).
// NOLINTBEGIN(bugprone-easily-swappable-parameters) (run_entropy, spawn_key) mirrors get_assembled_entropy's concat order
std::vector<std::uint32_t>
assembled_entropy(const std::vector<std::uint32_t> &run_entropy,
                  const std::vector<std::uint32_t> &spawn_key) {
// NOLINTEND(bugprone-easily-swappable-parameters)
    std::vector<std::uint32_t> assembled = run_entropy;
    if (!spawn_key.empty() && assembled.size() < kPoolSize)
        assembled.resize(kPoolSize, 0U);
    assembled.insert(assembled.end(), spawn_key.begin(), spawn_key.end());
    return assembled;
}

// mix_entropy(mixer, entropy_array): one hash_const accumulator threads
// through every hashmix call in source order.
std::array<std::uint32_t, kPoolSize>
mix_entropy(std::span<const std::uint32_t> entropy) {
    std::array<std::uint32_t, kPoolSize> mixer{};
    std::uint32_t hash_const = kInitA;
    for (std::size_t i = 0; i < kPoolSize; ++i)
        mixer.at(i) =
            hashmix(i < entropy.size() ? entropy[i] : 0U, hash_const);
    for (std::size_t i_src = 0; i_src < kPoolSize; ++i_src)
        for (std::size_t i_dst = 0; i_dst < kPoolSize; ++i_dst)
            if (i_src != i_dst)
                mixer.at(i_dst) =
                    mix(mixer.at(i_dst),
                        hashmix(mixer.at(i_src), hash_const));
    for (std::size_t i_src = kPoolSize; i_src < entropy.size(); ++i_src)
        for (std::size_t i_dst = 0; i_dst < kPoolSize; ++i_dst)
            mixer.at(i_dst) =
                mix(mixer.at(i_dst),
                    hashmix(entropy[i_src], hash_const));
    return mixer;
}

} // namespace

SeedSequence::SeedSequence(std::vector<std::uint32_t> entropy_words,
                           std::vector<std::uint32_t> spawn_key)
    : entropy_words_(std::move(entropy_words)),
      spawn_key_(std::move(spawn_key)),
      pool_(mix_entropy(assembled_entropy(entropy_words_, spawn_key_))) {}

SeedSequence::SeedSequence(std::uint64_t entropy)
    : SeedSequence(uint64_words(entropy), {}) {}

SeedSequence::SeedSequence(std::string_view entropy)
    : SeedSequence(string_words(entropy), {}) {}

SeedSequence::SeedSequence(std::string_view entropy,
                           std::span<const std::uint32_t> spawn_key)
    : SeedSequence(string_words(entropy),
                   {spawn_key.begin(), spawn_key.end()}) {}

std::vector<std::uint32_t>
SeedSequence::generate_state_u32(std::size_t n_words) const {
    // generate_state(..., np.uint32): the INIT_B/MULT_B cycle over the pool.
    std::vector<std::uint32_t> state(n_words);
    std::uint32_t hash_const = kInitB;
    for (std::size_t i = 0; i < n_words; ++i) {
        std::uint32_t data_val = pool_.at(i % kPoolSize);
        data_val ^= hash_const;
        hash_const *= kMultB;
        data_val *= hash_const;
        data_val ^= data_val >> kXshift;
        state.at(i) = data_val;
    }
    return state;
}

std::vector<std::uint64_t>
SeedSequence::generate_state_u64(std::size_t n_words) const {
    // np.uint64 requests twice the uint32 words, viewed little-endian in
    // pairs (state.astype('<u4').view('<u8')).
    const std::vector<std::uint32_t> words =
        generate_state_u32(n_words * 2U);
    std::vector<std::uint64_t> state(n_words);
    for (std::size_t i = 0; i < n_words; ++i)
        state.at(i) =
            static_cast<std::uint64_t>(words.at(2U * i)) |
            (static_cast<std::uint64_t>(words.at(2U * i + 1U)) << 32U);
    return state;
}

std::vector<SeedSequence> SeedSequence::spawn(std::uint32_t n_children) {
    std::vector<SeedSequence> children;
    children.reserve(n_children);
    for (std::uint32_t i = n_children_spawned_;
         i < n_children_spawned_ + n_children; ++i) {
        std::vector<std::uint32_t> key = spawn_key_;
        key.push_back(i);
        // Construct in member scope: the (words, spawn_key) ctor is private,
        // so vector::emplace_back's allocator-side construction cannot reach
        // it. The implicit move into push_back is public.
        children.push_back(SeedSequence(entropy_words_, std::move(key)));
    }
    n_children_spawned_ += n_children;
    return children;
}

} // namespace bike::random
