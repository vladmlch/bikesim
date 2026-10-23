#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

namespace bike::spatial {

using Vec3 = std::array<double, 3>;
// wxyz, world_from_body. Raw checkpoint quaternion signs are preserved.
using Quat = std::array<double, 4>;
enum class Mode : std::uint8_t { planar, spatial3d };
enum class RootKind : std::uint8_t { planar3, constrained4, free6 };
enum class TyreTier : std::uint8_t { fast, detailed };

struct BodyPose {
    Vec3 position_world_m{};
    Quat quat_world_from_body_wxyz{1., 0., 0., 0.};
    Vec3 linear_velocity_world_mps{};
    Vec3 angular_velocity_world_rad_s{};
};

inline constexpr std::uint32_t kSpatialContract = 1;
inline constexpr std::uint32_t kSpatialRuntimeSchema = 2;
inline constexpr std::uint32_t kSpatialBootstrapSchema = 2;
inline constexpr std::uint32_t kSpatialWriterSchema = 3;

std::string_view to_string(Mode) noexcept;
std::optional<Mode> parse_mode(std::string_view) noexcept;
std::string_view to_string(RootKind) noexcept;
std::optional<RootKind> parse_root_kind(std::string_view) noexcept;
std::string_view to_string(TyreTier) noexcept;
std::optional<TyreTier> parse_tyre_tier(std::string_view) noexcept;

} // namespace bike::spatial
