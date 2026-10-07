// state_format.hpp — versioned BIKEST02 bundle state reader with validation.
//
// tools/proto_native_bench state.bin layout (little-endian, exactly 80 bytes):
//   offset  0  magic8         "BIKEST02"
//   offset  8  version        u32 = 2
//   offset 12  flags          u32 = 0
//   offset 16  nq, nv, na, nu, nbody   u64 each
//   offset 56  payload_bytes  u64
//   offset 64  model_crc64    u64  (CRC64-ECMA over model.mjb bytes)
//   offset 72  state_crc64    u64  (CRC64-ECMA over the payload)
//
// Payload (binary64 little-endian, in order):
//   time(1), qpos[nq], qvel[nv], act[na], ctrl[nu], qfrc_applied[nv],
//   xfrc_applied[6*nbody], qacc_warmstart[nv]
// Required payload = 8 * (1 + nq + 3*nv + na + nu + 6*nbody) bytes, computed
// with checked arithmetic; the file must end exactly at the declared extent.
//
// CRC64-ECMA parameters: polynomial 0x42F0E1EBA9EA3693, init 0, no
// reflection, no final xor — check vector "123456789" -> 0x6C40DF5F0B497347
// (exercised through `native_bench --selftest`). The checksums detect
// corruption and cross-generation mixing; they are an integrity signal, not
// an authenticity/security guarantee.
//
// Every check runs BEFORE payload allocation or copy, in this order: size
// floor, magic (the retired legacy 0xBEA0 magic gets a regeneration
// instruction), version, flags, u64 field domains, dimension equality against
// the loaded model, declared-vs-required-vs-actual payload length, payload
// CRC64, model.mjb CRC64, then per-section finiteness during decode. A
// corrupt file therefore never sizes storage and never reaches an engine
// array. The header is decoded field-by-field from little-endian bytes — no
// packed-struct cast.
#pragma once

#include <mujoco/mujoco.h>

#include "../../native/src/model_access.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace proto_native_bench {

struct State {
    mjtNum time = 0;
    std::vector<mjtNum> qpos;
    std::vector<mjtNum> qvel;
    std::vector<mjtNum> act;
    std::vector<mjtNum> ctrl;
    std::vector<mjtNum> qfrc;
    std::vector<mjtNum> xfrc;
    std::vector<mjtNum> qacc_warmstart;
    // Declared header checksums, retained for run provenance reporting.
    std::uint64_t model_crc64 = 0;
    std::uint64_t state_crc64 = 0;
};

namespace detail {

constexpr std::array<char, 8> kStateMagic{'B', 'I', 'K', 'E',
                                          'S', 'T', '0', '2'};
constexpr std::int32_t kLegacyStateMagic = 0xBEA0;
constexpr std::size_t kHeaderBytes = 80;
constexpr std::size_t kWrenchElements = 6;
constexpr std::uint32_t kStateVersion = 2;
constexpr std::uint32_t kStateFlags = 0;
constexpr std::uint64_t kCrc64Poly = 0x42F0E1EBA9EA3693ULL;
constexpr std::uint64_t kMjtSizeMax =
    static_cast<std::uint64_t>(std::numeric_limits<mjtSize>::max());

// Element ceiling for every file/model-derived count: element counts must
// stay representable as mjtNum bytes, which also keeps them inside the
// signed mjtSize extent domain used by mjData arrays.
constexpr std::size_t kMaxElements =
    std::numeric_limits<std::size_t>::max() / sizeof(mjtNum);

// The payload decode states the little-endian contract instead of aliasing
// bytes blindly; the platform scope is little-endian hosts.
static_assert(std::endian::native == std::endian::little,
              "BIKEST02 state decode assumes a little-endian host");

// ---------------------------------------------------------------------------
// CRC64-ECMA (MSB-first, init 0, no reflection, no final xor)
// ---------------------------------------------------------------------------

// Compile-time CRC table: the consteval constructor fills the member array
// in place, so no large object ever crosses a function boundary
// (-Wlarge-by-value-copy stays satisfied without a suppression).
struct Crc64Table {
    std::array<std::uint64_t, 256> values{};
    consteval Crc64Table() {
        for (std::size_t i = 0; i < values.size(); ++i) {
            std::uint64_t crc = static_cast<std::uint64_t>(i) << 56U;
            for (int bit = 0; bit < 8; ++bit)
                crc = (crc & (std::uint64_t{1} << 63U)) != 0
                          ? (crc << 1U) ^ kCrc64Poly
                          : crc << 1U;
            values[i] = crc;
        }
    }
};

inline constexpr Crc64Table kCrc64Table{};

[[nodiscard]] inline std::uint64_t
crc64_ecma(std::span<const std::byte> data) noexcept {
    std::uint64_t crc = 0;
    for (const std::byte element : data)
        crc = kCrc64Table.values[((crc >> 56U) ^
                                  static_cast<std::uint64_t>(element)) &
                                 0xFFU] ^
              (crc << 8U);
    return crc;
}

// ---------------------------------------------------------------------------
// Explicit little-endian field decode (no packed-struct casts)
// ---------------------------------------------------------------------------

[[nodiscard]] inline std::uint32_t
read_u32(std::span<const char> field) noexcept {
    return static_cast<std::uint32_t>(static_cast<unsigned char>(field[0])) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(field[1]))
            << 8U) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(field[2]))
            << 16U) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(field[3]))
            << 24U);
}

[[nodiscard]] inline std::uint64_t
read_u64(std::span<const char> field) noexcept {
    std::uint64_t value = 0;
    for (std::size_t i = 0; i < 8; ++i)
        value |= static_cast<std::uint64_t>(
                     static_cast<unsigned char>(field[i]))
                 << (8U * i);
    return value;
}

[[nodiscard]] inline std::int32_t
read_i32(std::span<const char> field) noexcept {
    return std::bit_cast<std::int32_t>(read_u32(field));
}

// A header u64 dimension must fit the signed mjtSize domain and equal the
// loaded model's dimension; state/model mismatches are rejected here, before
// any buffer is sized.
[[nodiscard]] inline mjtSize header_dimension(std::span<const char> field,
                                              mjtSize model_dimension,
                                              std::string_view name,
                                              const std::string &label) {
    const std::uint64_t value = read_u64(field);
    if (value > kMjtSizeMax)
        throw std::invalid_argument(label + ": " + std::string(name) +
                                    " header field is outside the mjtSize "
                                    "domain");
    // std::cmp_* compares mixed-sign operands without a sign-hiding cast:
    // a negative model dimension (impossible for a loaded model) can never
    // equal an in-range u64, so the mismatch rejection below stays sound.
    if (std::cmp_not_equal(value, model_dimension))
        throw std::invalid_argument(label + ": state " + std::string(name) +
                                    "=" + std::to_string(value) +
                                    " does not match model dimension " +
                                    std::to_string(model_dimension));
    return model_dimension;
}

// Whole-file read with checked length accounting (replaces the two-read
// header/payload pattern now that CRCs cover the full byte stream anyway).
[[nodiscard]] inline std::vector<char>
read_file(const std::filesystem::path &path, const std::string &label) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file)
        throw std::runtime_error(label + ": file is not readable");
    const std::streamoff file_size = file.tellg();
    if (file_size < 0)
        throw std::runtime_error(label + ": file size is not readable");
    std::vector<char> bytes(static_cast<std::size_t>(file_size));
    file.seekg(0);
    if (!bytes.empty() &&
        !file.read(bytes.data(), static_cast<std::streamsize>(bytes.size())))
        throw std::runtime_error(label + ": truncated file read");
    return bytes;
}

// Copy `count` little-endian f64 elements out of `payload` at `offset`,
// advancing the cursor. The byte product and the end offset are checked
// against the declared payload extent before any vector is sized, and the
// section must be finite — a stored NaN is corruption, not state. The byte
// copy runs span-to-span over std::byte views: no (pointer, count) spans and
// no memcpy reach the buffers.
[[nodiscard]] inline std::vector<mjtNum>
read_section(std::span<const char> payload, std::size_t &offset,
             std::size_t count, std::string_view field,
             const std::string &label) {
    const std::size_t bytes =
        model_access::checked_product(count, sizeof(mjtNum), payload.size());
    const std::size_t end =
        model_access::checked_sum(offset, bytes, payload.size());
    std::vector<mjtNum> section(count);
    if (bytes != 0) {
        const std::span<const std::byte> chunk =
            std::as_bytes(payload.subspan(offset, bytes));
        const std::span<std::byte> destination =
            std::as_writable_bytes(std::span<mjtNum>{section});
        std::ranges::copy(chunk, destination.begin());
    }
    offset = end;
    if (!std::ranges::all_of(section,
                             [](mjtNum value) { return std::isfinite(value); }))
        throw std::invalid_argument(label + ": non-finite " +
                                    std::string(field) + " data");
    return section;
}

} // namespace detail

// Read and validate a BIKEST02 state.bin against `model` and the bundle's
// model.mjb at `model_path`. The model file bytes feed the model_crc64
// integrity check, which is what catches a state.bin paired with a different
// generation's model.mjb when dimensions happen to be equal.
[[nodiscard]] inline State load_state(const std::filesystem::path &state_path,
                                      const std::filesystem::path &model_path,
                                      const mjModel &model) {
    const std::string label = state_path.string();
    const std::vector<char> raw = detail::read_file(state_path, label);
    if (raw.size() < detail::kHeaderBytes)
        throw std::runtime_error(label + ": truncated state header");

    const std::span<const char> bytes{raw};
    const std::span<const char> header = bytes.first<detail::kHeaderBytes>();
    if (!std::ranges::equal(header.first<8>(),
                            std::span<const char>{detail::kStateMagic})) {
        if (detail::read_i32(header.subspan(0, 4)) == detail::kLegacyStateMagic)
            throw std::runtime_error(
                label + ": legacy 0xBEA0 state.bin is retired; regenerate the "
                        "bundle with tools/proto_native_bench/dump_model.py");
        throw std::runtime_error(label + ": unsupported state format");
    }
    if (detail::read_u32(header.subspan(8, 4)) != detail::kStateVersion)
        throw std::runtime_error(label + ": unsupported state version " +
                                 std::to_string(
                                     detail::read_u32(header.subspan(8, 4))));
    if (detail::read_u32(header.subspan(12, 4)) != detail::kStateFlags)
        throw std::runtime_error(label + ": unsupported state flags " +
                                 std::to_string(
                                     detail::read_u32(header.subspan(12, 4))));

    const mjtSize nq = detail::header_dimension(
        header.subspan(16, 8), model.nq, "nq", label);
    const mjtSize nv = detail::header_dimension(
        header.subspan(24, 8), model.nv, "nv", label);
    const mjtSize na = detail::header_dimension(
        header.subspan(32, 8), model.na, "na", label);
    const mjtSize nu = detail::header_dimension(
        header.subspan(40, 8), model.nu, "nu", label);
    const mjtSize nbody = detail::header_dimension(
        header.subspan(48, 8), model.nbody, "nbody", label);
    const std::uint64_t payload_field = detail::read_u64(header.subspan(56, 8));
    const std::uint64_t model_crc64 = detail::read_u64(header.subspan(64, 8));
    const std::uint64_t state_crc64 = detail::read_u64(header.subspan(72, 8));

    // Required payload: 8 * (1 + nq + 3*nv + na + nu + 6*nbody), checked.
    // Header dimensions already equal the model's, so these products are
    // bounded by construction; the checked forms keep the accounting honest.
    std::size_t elements = 1;
    elements = model_access::checked_sum(
        elements, static_cast<std::size_t>(nq), detail::kMaxElements);
    elements = model_access::checked_sum(
        elements,
        model_access::checked_product(std::size_t{3},
                                      static_cast<std::size_t>(nv),
                                      detail::kMaxElements),
        detail::kMaxElements);
    elements = model_access::checked_sum(
        elements, static_cast<std::size_t>(na), detail::kMaxElements);
    elements = model_access::checked_sum(
        elements, static_cast<std::size_t>(nu), detail::kMaxElements);
    elements = model_access::checked_sum(
        elements,
        model_access::checked_product(detail::kWrenchElements,
                                      static_cast<std::size_t>(nbody),
                                      detail::kMaxElements),
        detail::kMaxElements);
    const std::size_t payload_bytes = model_access::checked_product(
        elements, sizeof(mjtNum), std::numeric_limits<std::size_t>::max());
    if (payload_field != payload_bytes)
        throw std::runtime_error(
            label + ": payload_bytes field " + std::to_string(payload_field) +
            " does not match required " + std::to_string(payload_bytes));
    if (bytes.size() - detail::kHeaderBytes != payload_bytes)
        throw std::runtime_error(
            label + ": state payload length is " +
            std::to_string(bytes.size() - detail::kHeaderBytes) +
            " bytes; expected " + std::to_string(payload_bytes));

    const std::span<const char> payload = bytes.subspan(detail::kHeaderBytes);
    if (detail::crc64_ecma(std::as_bytes(payload)) != state_crc64)
        throw std::runtime_error(label + ": state payload checksum mismatch");
    const std::vector<char> model_bytes =
        detail::read_file(model_path, model_path.string());
    if (detail::crc64_ecma(std::as_bytes(
            std::span<const char>{model_bytes})) != model_crc64)
        throw std::runtime_error(
            label + ": model checksum mismatch (bundle members from different "
                    "generations?)");

    State state;
    state.model_crc64 = model_crc64;
    state.state_crc64 = state_crc64;
    std::size_t offset = 0;
    const std::vector<mjtNum> time =
        detail::read_section(payload, offset, 1, "time", label);
    state.time = time.front();
    state.qpos = detail::read_section(payload, offset,
                                      static_cast<std::size_t>(nq), "qpos", label);
    state.qvel = detail::read_section(payload, offset,
                                      static_cast<std::size_t>(nv), "qvel", label);
    state.act = detail::read_section(payload, offset,
                                     static_cast<std::size_t>(na), "act", label);
    state.ctrl = detail::read_section(payload, offset,
                                      static_cast<std::size_t>(nu), "ctrl", label);
    state.qfrc = detail::read_section(payload, offset,
                                      static_cast<std::size_t>(nv),
                                      "qfrc_applied", label);
    state.xfrc = detail::read_section(
        payload, offset,
        model_access::checked_product(detail::kWrenchElements,
                                      static_cast<std::size_t>(nbody),
                                      detail::kMaxElements),
        "xfrc_applied", label);
    state.qacc_warmstart =
        detail::read_section(payload, offset, static_cast<std::size_t>(nv),
                             "qacc_warmstart", label);
    if (offset != payload.size())
        throw std::logic_error(label + ": state payload accounting drift");
    return state;
}

} // namespace proto_native_bench
