// hash.cpp — FIPS 180-4 SHA-256 (task T05). Owned home for the digest every
// config/recording/fixture consumer and the engine-image identity path
// share; engine_call.cpp keeps only its file-reader around this class.
// Wrapping arithmetic is required by the spec and stays on std::uint32_t.
#include "io/hash.hpp"

#include <algorithm>
#include <bit>
#include <string_view>

namespace bike::io {

void Sha256::update(std::span<const std::byte> bytes) noexcept {
    total_ += bytes.size();
    while (!bytes.empty()) {
        const std::size_t take =
            std::min(bytes.size(), block_.size() - pending_);
        for (std::size_t i = 0; i < take; ++i)
            block_[pending_ + i] = static_cast<std::uint8_t>(bytes[i]);
        pending_ += take;
        bytes = bytes.subspan(take);
        if (pending_ == block_.size()) {
            compress();
            pending_ = 0;
        }
    }
}

std::array<std::uint8_t, 32> Sha256::finish() noexcept {
    const std::uint64_t bit_count = total_ * 8U;
    block_[pending_++] = 0x80U;
    if (pending_ > 56) {
        for (std::size_t i = pending_; i < block_.size(); ++i) block_[i] = 0;
        compress();
        pending_ = 0;
    }
    for (std::size_t i = pending_; i < 56; ++i) block_[i] = 0;
    for (std::size_t i = 0; i < 8; ++i)
        block_[56 + i] =
            static_cast<std::uint8_t>(bit_count >> ((7 - i) * 8));
    compress();
    std::array<std::uint8_t, 32> digest{};
    for (std::size_t i = 0; i < state_.size(); ++i)
        for (std::size_t j = 0; j < 4; ++j)
            digest[4 * i + j] =
                static_cast<std::uint8_t>(state_[i] >> ((3 - j) * 8));
    return digest;
}

void Sha256::compress() noexcept {
    // The FIPS 180-4 K constants — the first 32 bits of the fractional
    // parts of the cube roots of the first 64 primes.
    static constexpr std::array<std::uint32_t, 64> round_constants{
        0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
        0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
        0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
        0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
        0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
        0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
        0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
        0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
        0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
        0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
        0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
        0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
        0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
        0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
        0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
        0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};
    std::array<std::uint32_t, 64> w{};
    for (std::size_t i = 0; i < 16; ++i)
        w[i] = (static_cast<std::uint32_t>(block_[4 * i]) << 24U) |
               (static_cast<std::uint32_t>(block_[4 * i + 1]) << 16U) |
               (static_cast<std::uint32_t>(block_[4 * i + 2]) << 8U) |
               static_cast<std::uint32_t>(block_[4 * i + 3]);
    for (std::size_t i = 16; i < w.size(); ++i) {
        const std::uint32_t s0 = std::rotr(w[i - 15], 7) ^
                                 std::rotr(w[i - 15], 18) ^
                                 (w[i - 15] >> 3U);
        const std::uint32_t s1 = std::rotr(w[i - 2], 17) ^
                                 std::rotr(w[i - 2], 19) ^
                                 (w[i - 2] >> 10U);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    std::array<std::uint32_t, 8> work = state_;
    const auto rotate_round = [&work, &w](std::size_t i) noexcept {
        const std::uint32_t s1 = std::rotr(work[4], 6) ^
                                 std::rotr(work[4], 11) ^
                                 std::rotr(work[4], 25);
        const std::uint32_t ch = (work[4] & work[5]) ^ (~work[4] & work[6]);
        const std::uint32_t t1 =
            work[7] + s1 + ch + round_constants[i] + w[i];
        const std::uint32_t s0 = std::rotr(work[0], 2) ^
                                 std::rotr(work[0], 13) ^
                                 std::rotr(work[0], 22);
        const std::uint32_t maj = (work[0] & work[1]) ^
                                 (work[0] & work[2]) ^ (work[1] & work[2]);
        work[7] = work[6];
        work[6] = work[5];
        work[5] = work[4];
        work[4] = work[3] + t1;
        work[3] = work[2];
        work[2] = work[1];
        work[1] = work[0];
        work[0] = t1 + s0 + maj;
    };
    for (std::size_t i = 0; i < round_constants.size(); ++i)
        rotate_round(i);
    for (std::size_t i = 0; i < state_.size(); ++i) state_[i] += work[i];
}

std::string sha256(std::span<const std::byte> bytes) {
    static constexpr std::string_view digits = "0123456789abcdef";
    Sha256 hash;
    hash.update(bytes);
    std::string hex;
    hex.reserve(64);
    for (const std::uint8_t byte : hash.finish()) {
        hex += digits[byte >> 4U];
        hex += digits[byte & 0x0fU];
    }
    return hex;
}

} // namespace bike::io
