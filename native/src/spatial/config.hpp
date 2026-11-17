#pragma once

#include "io/value.hpp"
#include "spatial/frames.hpp"

#include <cstdint>
#include <optional>

namespace bike::spatial {

enum class SpatialTireBackend : std::uint8_t { compliant_3d, distributed_3d };
enum class SpatialTirePreset : std::uint8_t { mtb_2p8_1p2bar, mtb_2p4_standard };
enum class SpatialStartMode : std::uint8_t { moving, standstill };

struct SpatialSteerSpec {
    double range_deg = -1.;
    double control_period_s = -1.;
    double reaction_delay_s = -1.;
    double max_torque_nm = -1.;
};

struct SpatialStartSupportSpec {
    double release_speed_mps = -1.;
    double lateral_stiffness_n_per_m = -1.;
    double lateral_damping_ns_per_m = -1.;
    double roll_stiffness_nm_per_rad = -1.;
    double roll_damping_nms_per_rad = -1.;
};

struct SpatialConfig {
    SpatialTireBackend tire_backend = SpatialTireBackend::compliant_3d;
    SpatialTirePreset tire_preset = SpatialTirePreset::mtb_2p8_1p2bar;
    SpatialStartMode start_mode = SpatialStartMode::moving;
    double corridor_half_width_m = -1.;
    double capsize_roll_rad = -1.;
    SpatialSteerSpec steer{};
    std::optional<SpatialStartSupportSpec> start_support{};
    Vec3 imu_mount_pos_m{0., 0., 0.};
    Vec3 imu_mount_rpy_rad{0., 0., 0.};
    Vec3 imu_misalignment_rad{0., 0., 0.};
};

// Returns std::nullopt when root_object has no "spatial" key (planar mode).
[[nodiscard]] std::optional<SpatialConfig> parse_spatial_config(
    const bike::io::Value::Object& root_object, double timestep_s);

void validate_spatial_config(const SpatialConfig& config, double timestep_s);
[[nodiscard]] bike::io::Value spatial_config_to_value(const SpatialConfig& config);

// Exact integer-multiple check (Decision 12): ratio = duration_s / step_s must be
// an integer >= min_ticks within 1e-12 relative tolerance; rejects silent rounding.
[[nodiscard]] std::int64_t require_exact_ticks(
    double duration_s, double step_s, std::int64_t min_ticks, const char* field_path);

} // namespace bike::spatial
