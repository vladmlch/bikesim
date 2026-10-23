#pragma once

#include "io/value.hpp"
#include "terrain/grid3d.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace bike::terrain {

struct Bank3dSpec {
    double start_x_m = 0., ramp_m = 0., angle_deg = 0.; // |angle| <= 15.
};

struct Roughness3dSpec {
    double sigma_m = 0., ell_m = 0.;
};

struct Feature3dSpec {
    std::string kind; // "rock", "rut", "root".
    double x_m = 0., y_m = 0., amplitude_m = 0.;
    double radius_x_m = 0., radius_y_m = 0., yaw_rad = 0.;
};

struct MaterialZone3dSpec {
    std::uint32_t surface_id = 0;
    double x0_m = 0., x1_m = 0., y0_m = 0., y1_m = 0.;
    // Half-open at right/top, except the final physical domain boundary.
};

struct Track3dSpec {
    bike::io::Value base_track_spec;
    std::string seed_decimal;
    double dx_m = .005, dy_m = .010, half_width_m = 1.5;
    std::vector<Bank3dSpec> banks;
    Roughness3dSpec roughness;
    std::vector<Feature3dSpec> features;
    std::vector<SurfaceSpec3d> surfaces;
    std::vector<MaterialZone3dSpec> zones;
    double lead_in_m = 4., lead_out_m = 5.;
    std::string generator_version = "grid3d-gen/1";
};

Track3dSpec read_track3d_spec(const bike::io::Value& toml_or_json);
bike::io::Value write_track3d_spec(const Track3dSpec&);

} // namespace bike::terrain
