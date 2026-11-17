#pragma once

#include "terrain/grid3d.hpp"
#include "terrain/types.hpp"

#include <string>
#include <vector>

namespace bike::terrain {

struct CrossSlopeSegment3d {
    double start_m = 0.;
    double end_m = 0.;
    double transition_m = 1.0;
    // Positive bank_rad raises +Y (left) and lowers -Y (right): dz = tan(bank_rad) * y.
    double bank_rad = 0.;
};

struct Terrain3dOverlaySpec {
    double half_width_m = 1.5;
    double dx_m = DEFAULT_RESOLUTION_M; // 0.005 m (matches 1D profile nodes)
    double dy_m = 0.020;                // 0.020 m (151 transverse nodes -> ~32.6 MB on 120 m track)
    double datum_z_offset_m = 0.;
    double lead_in_m = 4.0;
    double lead_out_m = 4.0;
    double lateral_roughness_amplitude_m = 0.;
    std::string seed = "0";
    std::vector<CrossSlopeSegment3d> cross_slopes{};
};

void validate_overlay_spec(const TrackSpec& base_track, const Terrain3dOverlaySpec& overlay);
[[nodiscard]] Grid3d build_overlay_grid3d(const TrackSpec& base_track,
                                          const Terrain3dOverlaySpec& overlay);

} // namespace bike::terrain
