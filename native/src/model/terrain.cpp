#include "builder_internal.hpp"
#include <limits>
#include <stdexcept>

namespace bike::model::detail {
// mujoco/terrain.py. Elevation is populated by T21 into the owned model;
// generating XML does not bake a Python-created heightfield into the asset.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
void build_terrain(Element& root,Element& world,const BuildContext& c) {
    const auto& field=c.request.field;
    if (c.spatial()) {
        if (!c.request.terrain3d)
            throw std::invalid_argument("model.terrain3d: physical source missing");
        if (field.nrow < 2 || field.ncol < 2 ||
            static_cast<std::uint64_t>(field.nrow)>std::numeric_limits<std::uint32_t>::max() ||
            static_cast<std::uint64_t>(field.ncol)>std::numeric_limits<std::uint32_t>::max())
            throw std::invalid_argument("model.terrain3d.render_resolution: invalid dimensions");
        const auto grid=terrain::make_render_grid(*c.request.terrain3d,
            {static_cast<std::uint32_t>(field.nrow),static_cast<std::uint32_t>(field.ncol)});
        build_terrain(root,world,c,grid);
        return;
    }
    block(root,"asset").add("hfield",{{"name","road"},{"nrow",std::to_string(field.nrow)},
        {"ncol",std::to_string(field.ncol)},{"size",field.size_attr()}});
    const auto ground=geometry::compute_ground_z(c.request.bike)/1000.;
    geom(world,"terrain","hfield",{{"hfield","road"},{"pos",f(field.geom_x_m())+" 0 "+f(field.geom_z_m(ground))},
        {"condim","3"},{"friction","1.2 0.005 0.0001"},{"solref","-130000 -800"},{"material","mat_floor"}});
}

void build_terrain(Element& root,Element& world,const BuildContext&,
                   const terrain::RenderGrid3d& grid) {
    if(grid.nrow<2 || grid.ncol<2 || !std::isfinite(grid.x0_m) ||
       !std::isfinite(grid.x1_m) || !std::isfinite(grid.y0_m) ||
       !std::isfinite(grid.y1_m) || !std::isfinite(grid.z_min_m) ||
       !std::isfinite(grid.z_max_m) || !(grid.x1_m>grid.x0_m) ||
       !(grid.y1_m>grid.y0_m) || grid.z_max_m<grid.z_min_m)
        throw std::invalid_argument("model.terrain3d.render: invalid bounds");
    // MuJoCo requires a positive vertical scale even for a flat source.
    // Its normalized samples are all zero, so this leaves the surface at z_min.
    const double height=grid.z_max_m>grid.z_min_m?grid.z_max_m-grid.z_min_m:1.;
    block(root,"asset").add("hfield",{{"name","road"},{"nrow",std::to_string(grid.nrow)},
        {"ncol",std::to_string(grid.ncol)}, {"size",repr((grid.x1_m-grid.x0_m)*.5)+" "+
        repr((grid.y1_m-grid.y0_m)*.5)+" "+repr(height)+" 0.1"}});
    geom(world,"terrain","hfield",{{"hfield","road"},
        {"pos",repr((grid.x0_m+grid.x1_m)*.5)+" "+repr((grid.y0_m+grid.y1_m)*.5)+" "+repr(grid.z_min_m)},
        {"contype",std::to_string(kSpatialCrashMask)}, {"conaffinity",std::to_string(kSpatialCrashMask)},
        {"condim","3"},{"friction","1.2 0.005 0.0001"},{"solref","-130000 -800"},{"material","mat_floor"}});
}
} // namespace bike::model::detail

namespace bike::model {
void populate_render_heightfield(mjModel& model,const terrain::RenderGrid3d& grid) {
    const int id=mj_name2id(&model,mjOBJ_HFIELD,"road");
    if(id<0)throw std::invalid_argument("model.terrain3d.render: road heightfield missing");
    if(grid.nrow<2 || grid.ncol<2 ||
       model.hfield_nrow[id]!=static_cast<mjtSize>(grid.nrow) ||
       model.hfield_ncol[id]!=static_cast<mjtSize>(grid.ncol))
        throw std::invalid_argument("model.terrain3d.render: dimension mismatch");
    const auto count=static_cast<std::uint64_t>(grid.nrow)*grid.ncol;
    const auto address=model.hfield_adr[id];
    if(address<0 || count!=grid.normalized_height.size() ||
       static_cast<std::uint64_t>(address)>static_cast<std::uint64_t>(model.nhfielddata) ||
       count>static_cast<std::uint64_t>(model.nhfielddata-address))
        throw std::invalid_argument("model.terrain3d.render: heightfield storage mismatch");
    for(float z:grid.normalized_height)
        if(!std::isfinite(z) || z<0.f || z>1.f)
            throw std::invalid_argument("model.terrain3d.render: sample outside [0,1]");
    // Validate the complete matrix before mutating any row. Physics never reads
    // this visual downsample; it retains the immutable Terrain3dSource owner.
    std::copy(grid.normalized_height.begin(),grid.normalized_height.end(),model.hfield_data+address);
}
} // namespace bike::model
