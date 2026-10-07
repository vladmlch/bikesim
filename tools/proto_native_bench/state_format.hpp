// state_format.hpp — legacy (0xBEA0) benchmark state reader with validation.
//
// tools/proto_native_bench state.bin layout (little-endian):
//   i32 magic = 0xBEA0
//   i32 nq, i32 nv, i32 na, i32 nu, i32 nbody
//   f64 time
//   f64 qpos[nq], f64 qvel[nv], f64 act[na], f64 ctrl[nu],
//   f64 qfrc_applied[nv], f64 xfrc_applied[6*nbody]
//
// Every check runs BEFORE payload allocation or copy: header counts must be
// nonnegative and equal the loaded model's dimensions, the file must contain
// exactly the declared payload (no truncation, no trailing bytes), and every
// stored scalar must be finite. A corrupt file therefore never sizes storage
// and never reaches an engine array.
//
// Task R3 of the native-safety runtime plan replaces this reader with the
// versioned BIKEST02 bundle; the reader stays isolated in this header so the
// swap touches no benchmark code.
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
    int nq = 0;
    int nv = 0;
    int na = 0;
    int nu = 0;
    int nbody = 0;
    mjtNum time = 0;
    std::vector<mjtNum> qpos;
    std::vector<mjtNum> qvel;
    std::vector<mjtNum> act;
    std::vector<mjtNum> ctrl;
    std::vector<mjtNum> qfrc;
    std::vector<mjtNum> xfrc;
};

namespace detail {

constexpr std::int32_t kLegacyStateMagic = 0xBEA0;
constexpr std::size_t kHeaderBytes = 24;
constexpr std::size_t kWrenchElements = 6;

// Element ceiling for every file/model-derived count: element counts must
// stay representable as mjtNum bytes, which also keeps them inside the
// signed mjtSize extent domain used by mjData arrays.
constexpr std::size_t kMaxElements =
    std::numeric_limits<std::size_t>::max() / sizeof(mjtNum);

// The legacy writer stored native little-endian scalars (the artifacts were
// produced on arm64); the payload decode states that contract instead of
// aliasing bytes blindly.
static_assert(std::endian::native == std::endian::little,
              "legacy state decode assumes a little-endian host");

[[nodiscard]] inline std::int32_t read_i32(std::span<const char> field) noexcept {
    const std::uint32_t value =
        static_cast<std::uint32_t>(static_cast<unsigned char>(field[0])) |
        (static_cast<std::uint32_t>(static_cast<unsigned char>(field[1])) << 8U) |
        (static_cast<std::uint32_t>(static_cast<unsigned char>(field[2])) << 16U) |
        (static_cast<std::uint32_t>(static_cast<unsigned char>(field[3])) << 24U);
    return std::bit_cast<std::int32_t>(value);
}

// A header count must be nonnegative and equal the loaded model's dimension;
// state/model mismatches are rejected here, before any buffer is sized.
[[nodiscard]] inline int header_dimension(std::span<const char> field,
                                          mjtSize model_dimension,
                                          std::string_view name,
                                          const std::string &label) {
    const std::int32_t value = read_i32(field);
    if (value < 0)
        throw std::invalid_argument(label + ": negative " + std::string(name) +
                                    " header count");
    if (static_cast<std::int64_t>(value) != model_dimension)
        throw std::invalid_argument(label + ": state " + std::string(name) + "=" +
                                    std::to_string(value) +
                                    " does not match model dimension " +
                                    std::to_string(model_dimension));
    return value;
}

// Copy `count` little-endian f64 elements out of `payload` at `offset`,
// advancing the cursor. The byte product and the end offset are checked
// against the declared payload extent before any vector is sized, and the
// section must be finite — a stored NaN is corruption, not state. The byte
// copy runs span-to-span over std::byte views: no (pointer, count) spans and
// no memcpy reach the buffers.
[[nodiscard]] inline std::vector<mjtNum> read_section(std::span<const char> payload,
                                                    std::size_t &offset,
                                                    std::size_t count,
                                                    std::string_view field,
                                                    const std::string &label) {
    const std::size_t bytes = model_access::checked_product(
        count, sizeof(mjtNum), payload.size());
    const std::size_t end = model_access::checked_sum(offset, bytes, payload.size());
    std::vector<mjtNum> section(count);
    if (bytes != 0) {
        const std::span<const std::byte> chunk =
            std::as_bytes(payload.subspan(offset, bytes));
        const std::span<std::byte> destination =
            std::as_writable_bytes(std::span<mjtNum>{section});
        std::ranges::copy(chunk, destination.begin());
    }
    offset = end;
    if (!std::ranges::all_of(section, [](mjtNum value) { return std::isfinite(value); }))
        throw std::invalid_argument(label + ": non-finite " + std::string(field) +
                                    " data");
    return section;
}

} // namespace detail

// Read and validate a legacy state.bin against `model`. Both header order
// and payload extent are enforced against the loaded model dimensions; any
// failure throws before storage is sized or bytes are copied.
[[nodiscard]] inline State load_state(const std::filesystem::path &path,
                                      const mjModel &model) {
    const std::string label = path.string();
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file)
        throw std::runtime_error(label + ": state file is not readable");
    const std::streamoff file_size = file.tellg();
    if (file_size < 0 || std::cmp_less(file_size, detail::kHeaderBytes))
        throw std::runtime_error(label + ": truncated state header");

    file.seekg(0);
    std::array<char, detail::kHeaderBytes> header{};
    if (!file.read(header.data(), static_cast<std::streamsize>(header.size())))
        throw std::runtime_error(label + ": truncated state header");
    const std::span<const char> header_bytes{header};
    if (detail::read_i32(header_bytes.subspan(0, 4)) != detail::kLegacyStateMagic)
        throw std::runtime_error(
            label + ": unsupported state format; expected the legacy 0xBEA0 "
                    "layout");

    State state;
    state.nq = detail::header_dimension(header_bytes.subspan(4, 4), model.nq, "nq", label);
    state.nv = detail::header_dimension(header_bytes.subspan(8, 4), model.nv, "nv", label);
    state.na = detail::header_dimension(header_bytes.subspan(12, 4), model.na, "na", label);
    state.nu = detail::header_dimension(header_bytes.subspan(16, 4), model.nu, "nu", label);
    state.nbody = detail::header_dimension(header_bytes.subspan(20, 4), model.nbody,
                                           "nbody", label);

    // Exact payload extent: time + qpos + qvel + act + ctrl + qfrc + xfrc.
    // Header dimensions already equal the model's, so these products are
    // bounded by construction; the checked forms keep the accounting honest.
    std::size_t elements = 1;
    elements = model_access::checked_sum(
        elements, static_cast<std::size_t>(state.nq), detail::kMaxElements);
    elements = model_access::checked_sum(
        elements, static_cast<std::size_t>(state.nv), detail::kMaxElements);
    elements = model_access::checked_sum(
        elements, static_cast<std::size_t>(state.na), detail::kMaxElements);
    elements = model_access::checked_sum(
        elements, static_cast<std::size_t>(state.nu), detail::kMaxElements);
    elements = model_access::checked_sum(
        elements, static_cast<std::size_t>(state.nv), detail::kMaxElements);
    elements = model_access::checked_sum(
        elements,
        model_access::checked_product(detail::kWrenchElements,
                                      static_cast<std::size_t>(state.nbody),
                                      detail::kMaxElements),
        detail::kMaxElements);
    const std::size_t payload_bytes = model_access::checked_product(
        elements, sizeof(mjtNum), std::numeric_limits<std::size_t>::max());
    const std::size_t expected_total = model_access::checked_sum(
        detail::kHeaderBytes, payload_bytes,
        std::numeric_limits<std::size_t>::max());
    if (std::cmp_not_equal(file_size, expected_total))
        throw std::runtime_error(label + ": state payload length is " +
                                 std::to_string(static_cast<std::uintmax_t>(file_size) -
                                                detail::kHeaderBytes) +
                                 " bytes; expected " +
                                 std::to_string(payload_bytes));

    std::vector<char> payload(payload_bytes);
    if (!file.read(payload.data(), static_cast<std::streamsize>(payload_bytes)))
        throw std::runtime_error(label + ": truncated state payload");

    const std::span<const char> bytes{payload};
    std::size_t offset = 0;
    const std::vector<mjtNum> time =
        detail::read_section(bytes, offset, 1, "time", label);
    state.time = time.front();
    state.qpos = detail::read_section(bytes, offset, static_cast<std::size_t>(state.nq),
                                      "qpos", label);
    state.qvel = detail::read_section(bytes, offset, static_cast<std::size_t>(state.nv),
                                      "qvel", label);
    state.act = detail::read_section(bytes, offset, static_cast<std::size_t>(state.na),
                                     "act", label);
    state.ctrl = detail::read_section(bytes, offset, static_cast<std::size_t>(state.nu),
                                      "ctrl", label);
    state.qfrc = detail::read_section(bytes, offset, static_cast<std::size_t>(state.nv),
                                      "qfrc_applied", label);
    state.xfrc = detail::read_section(
        bytes, offset,
        model_access::checked_product(detail::kWrenchElements,
                                      static_cast<std::size_t>(state.nbody),
                                      detail::kMaxElements),
        "xfrc_applied", label);
    if (offset != bytes.size())
        throw std::logic_error(label + ": state payload accounting drift");
    return state;
}

} // namespace proto_native_bench
