#include "trajectory.hpp"
#include "../io/hash.hpp"
#include "../io/npy.hpp"
#include "../model_access.hpp"
#include "../recording/reader.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <utility>

namespace bike::replay {
namespace {
constexpr std::string_view kMagic = "BGTRAJ01";
constexpr std::uint32_t kVersion = 1;
constexpr std::uint32_t kWheelCols = 8;
constexpr std::size_t kHeaderBytes = 96;
constexpr std::size_t kShaHexLength = 64;

[[nodiscard]] std::uint32_t read_u32_le(std::span<const std::byte> bytes, std::size_t offset) noexcept {
    std::uint32_t value = 0;
    for (std::size_t i = 0; i < 4; ++i) {
        value |= static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(bytes[offset + i])) << (8U * i);
    }
    return value;
}

[[nodiscard]] std::uint64_t read_u64_le(std::span<const std::byte> bytes, std::size_t offset) noexcept {
    std::uint64_t value = 0;
    for (std::size_t i = 0; i < 8; ++i) {
        value |= static_cast<std::uint64_t>(std::to_integer<std::uint8_t>(bytes[offset + i])) << (8U * i);
    }
    return value;
}

[[nodiscard]] double read_f64_le(std::span<const std::byte> bytes, std::size_t offset) noexcept {
    return std::bit_cast<double>(read_u64_le(bytes, offset));
}

[[nodiscard]] std::string read_ascii(std::span<const std::byte> slice) {
    std::string out;
    out.reserve(slice.size());
    for (const std::byte byte : slice) {
        out.push_back(static_cast<char>(std::to_integer<unsigned char>(byte)));
    }
    return out;
}

[[nodiscard]] std::vector<std::byte> read_regular_bytes(const std::filesystem::path& path, std::string_view label) {
    try {
        return io::read_numpy_bytes(path);
    } catch (const std::exception& error) {
        throw recording::RecordingError("cannot read " + std::string(label) + ": " + error.what());
    }
}
} // namespace

ExternalTrajectory load_external_trajectory(const std::filesystem::path& dir) {
    auto mjb_bytes = read_regular_bytes(dir / "model.mjb", "trajectory model.mjb");
    const auto traj_bytes = read_regular_bytes(dir / "trajectory.bin", "trajectory.bin");

    if (traj_bytes.size() < kHeaderBytes) {
        throw recording::RecordingError("trajectory size: file smaller than 96-byte BGTRAJ01 header");
    }
    const std::span<const std::byte> raw{traj_bytes};
    if (read_ascii(raw.subspan(0, kMagic.size())) != kMagic) {
        throw recording::RecordingError("trajectory magic mismatch (expected BGTRAJ01)");
    }
    const std::uint32_t version = read_u32_le(raw, 8);
    if (version != kVersion) {
        throw recording::RecordingError("unsupported trajectory version: " + std::to_string(version));
    }
    const std::uint32_t nq = read_u32_le(raw, 12);
    const std::uint32_t nv = read_u32_le(raw, 16);
    const std::uint32_t wheel_cols = read_u32_le(raw, 20);
    if (wheel_cols != kWheelCols) {
        throw recording::RecordingError("trajectory wheel_cols must be 8, got " + std::to_string(wheel_cols));
    }
    const std::uint64_t frames = read_u64_le(raw, 24);
    const std::string header_sha = read_ascii(raw.subspan(32, kShaHexLength));

    std::string actual_sha = io::sha256(mjb_bytes);
    if (header_sha != actual_sha) {
        throw recording::RecordingError("trajectory model hash mismatch");
    }

    const std::size_t payload_bytes = raw.size() - kHeaderBytes;
    const std::size_t max_cols = payload_bytes / sizeof(double);
    std::size_t row_cols = 0;
    try {
        row_cols = model_access::checked_sum(
            static_cast<std::size_t>(2) + static_cast<std::size_t>(kWheelCols),
            model_access::checked_sum(static_cast<std::size_t>(nq), static_cast<std::size_t>(nv), max_cols),
            max_cols);
    } catch (const std::invalid_argument&) {
        throw recording::RecordingError("trajectory size mismatch");
    }
    const std::size_t row_bytes = row_cols * sizeof(double);
    if (frames > payload_bytes / row_bytes || static_cast<std::size_t>(frames) * row_bytes != payload_bytes) {
        throw recording::RecordingError("trajectory size mismatch");
    }
    if (frames < 2) {
        throw recording::RecordingError("stored-state playback needs at least two recorded frames");
    }

    const auto frame_count = static_cast<std::size_t>(frames);
    ExternalTrajectory out;
    out.playback.intervals_hash_verified = false;
    out.playback.model = {.mjb = std::move(mjb_bytes), .sha256 = std::move(actual_sha)};
    out.playback.times_s.reserve(frame_count);
    out.playback.ends_s.reserve(frame_count);
    out.playback.qpos.reserve(frame_count);
    out.playback.qvel.reserve(frame_count);
    out.wheels.reserve(frame_count);

    for (std::size_t f = 0; f < frame_count; ++f) {
        std::size_t offset = kHeaderBytes + f * row_bytes;
        const auto next_f64 = [&]() noexcept {
            const double val = read_f64_le(raw, offset);
            offset += sizeof(double);
            return val;
        };
        out.playback.times_s.push_back(next_f64());
        out.playback.ends_s.push_back(next_f64());

        std::vector<double> q(nq);
        for (std::size_t i = 0; i < nq; ++i) q[i] = next_f64();
        out.playback.qpos.push_back(std::move(q));

        std::vector<double> v(nv);
        for (std::size_t i = 0; i < nv; ++i) v[i] = next_f64();
        out.playback.qvel.push_back(std::move(v));

        std::array<double, 8> wheel{};
        for (double& w : wheel) {
            w = next_f64();
            if (!std::isfinite(w)) {
                throw recording::RecordingError("trajectory wheel telemetry contains nonfinite value");
            }
        }
        out.wheels.push_back(wheel);
    }

    const auto model = make_playback_model(out.playback);
    if (model->nq < 0 || model->nv < 0 ||
        std::cmp_not_equal(model->nq, nq) ||
        std::cmp_not_equal(model->nv, nv)) {
        throw recording::RecordingError("trajectory dimensions differ from model.mjb");
    }
    fill_quaternion_starts(out.playback, *model);
    out.playback.duration_s = out.playback.ends_s.back();
    validate_playback(out.playback, static_cast<std::size_t>(model->nq), static_cast<std::size_t>(model->nv));
    return out;
}
} // namespace bike::replay
