#pragma once

#include "terrain/extrusion3d.hpp"
#include "terrain/grid3d.hpp"
#include "terrain/source3d.hpp"
#include "terrain/types.hpp"

#include <mujoco/mujoco.h>

namespace bike::terrain {

struct HeightField3dBuild {
    HeightFieldSpec spec{};
    TerrainBounds3d bounds{};
    double ground_z_m = 0.;
    double catch_plane_z_m = 0.;
};

[[nodiscard]] Grid3dSpec make_grid3d_spec(const TerrainBounds3d& bounds,
                                          double dx_m, double dy_m);

[[nodiscard]] Grid3d sample_to_grid3d(Terrain3dQueryView view, const Grid3dSpec& spec);
[[nodiscard]] Grid3d sample_to_grid3d(const ExtrudedTrack3d& extruded, const Grid3dSpec& spec);

[[nodiscard]] HeightField3dBuild build_heightfield_3d(
    const Grid3d& grid, double ground_z_m = 0., double catch_clearance_m = 1.0);
[[nodiscard]] HeightField3dBuild build_heightfield_3d(
    Terrain3dQueryView view, const Grid3dSpec& spec,
    double ground_z_m = 0., double catch_clearance_m = 1.0);
[[nodiscard]] HeightField3dBuild build_heightfield_3d(
    const ExtrudedTrack3d& extruded, const Grid3dSpec& spec,
    double ground_z_m = 0., double catch_clearance_m = 1.0);

void populate_heightfield_3d(mjModel& model, const Grid3d& grid,
                             const HeightField3dBuild& hf);

} // namespace bike::terrain
