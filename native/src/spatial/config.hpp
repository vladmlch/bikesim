#pragma once

#include "types.hpp"
#include "../io/value.hpp"

#include <cstdint>
#include <filesystem>
#include <numbers>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace bike::configuration { struct SimulationPhysicsConfig; }

namespace bike::spatial {

enum class SteerController : std::uint8_t { off, roll_heading };
struct SteerConfig {
    double range_deg = 60.;
    double damping_nms_rad = .5;
    std::optional<double> torque_cap_nm; // nullopt means profile strength
    double reaction_delay_s = .1;
    SteerController controller = SteerController::roll_heading;
    std::string profile_id;
    std::string profile_sha256;
};
struct BootstrapConfig {
    double settle_time_s = 80.;
    double release_force_limit_n = 1.;
    double release_moment_limit_nm = 1.;
    double hold_stiffness_n_per_m = 2000.;
    double hold_damping_ns_per_m = 200.;
    double hold_rot_stiffness_nm_per_rad = 400.;
    double hold_rot_damping_nms_per_rad = 40.;
};
struct ImuMountConfig {
    std::variant<std::string, Vec3> mount_pos = std::string("site_BB");
    Vec3 mount_rpy_rad{};
    Vec3 axis_misalignment_rad{};
    std::string calibration_revision;
};
enum class TerrainRequestKind : std::uint8_t { none, grid_asset, genspec, profile_extrusion };
struct TerrainRequest3d {
    TerrainRequestKind kind = TerrainRequestKind::none;
    std::filesystem::path path;
    double corridor_half_width_m = 1.5;
    std::uint64_t max_asset_bytes = 1ull << 30; // validated in [1, INT64_MAX] for TOML round trip
    // Optional generation overrides; absent for an immutable imported asset.
    // Frozen in W00 so W05 never accepts a grid step it cannot preserve.
    std::optional<double> dx_m;
    std::optional<double> dy_m;
};
struct Pneumatic3dConfig {
    TyreTier tier = TyreTier::fast;
    std::string lateral_coefficients_id;
    std::string lateral_coefficients_sha256;
};
struct Config {
    Mode mode = Mode::planar;
    RootKind root = RootKind::planar3;
    double corridor_half_width_m = 1.5;
    double capsize_roll_rad = std::numbers::pi / 3.;
    SteerConfig steer;
    BootstrapConfig bootstrap;
    ImuMountConfig imu;
    TerrainRequest3d terrain;
    Pneumatic3dConfig tyre;
};
struct CapabilityReport {
    bool implemented = false;
    bool qualification_pending = true;
    std::vector<std::string> rejections;
};

// Cold, owned boundaries. Errors are invalid_argument with a full field path.
Config parse_spatial_config(const bike::io::Value& physics_table);
void validate_spatial_config(const Config&, const bike::configuration::SimulationPhysicsConfig&);
bike::io::Value spatial_config_to_wire(const Config&);
Config spatial_config_from_wire(const bike::io::Value&);
CapabilityReport check_spatial_capability(const Config&, const bike::configuration::SimulationPhysicsConfig&);
std::string spatial_config_fingerprint_hex(const Config&);

} // namespace bike::spatial
