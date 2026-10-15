// hash.hpp — FIPS 180-4 SHA-256 (task T05). The owned integrity/
// content-identity primitive for fixture/provenance manifests, the shared
// contract-test harness and the engine-image identity path in
// engine_call.cpp — a digest of content, never a claim of authenticity.
// The streaming type exists so file-hash consumers (the engine image is
// read in bounded chunks) share the identical compression code.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace bike::io {

class Sha256 {
  public:
    void update(std::span<const std::byte> bytes) noexcept;
    [[nodiscard]] std::array<std::uint8_t, 32> finish() noexcept;

  private:
    void compress() noexcept;

    std::array<std::uint32_t, 8> state_{
        0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
        0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};
    std::array<std::uint8_t, 64> block_{};
    std::uint64_t total_ = 0;
    std::size_t pending_ = 0;
};

// Lowercase-hex digest of the byte span (64 characters).
[[nodiscard]] std::string sha256(std::span<const std::byte> bytes);

} // namespace bike::io
