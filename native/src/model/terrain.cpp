#include "builder_internal.hpp"
#include "../model_access.hpp"
#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

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
            std::cmp_greater(field.nrow,std::numeric_limits<std::uint32_t>::max()) ||
            std::cmp_greater(field.ncol,std::numeric_limits<std::uint32_t>::max()))
            throw std::invalid_argument("model.terrain3d.render_resolution: invalid dimensions");
        const auto grid=terrain::make_render_grid(*c.request.terrain3d,
            {.nrow=static_cast<std::uint32_t>(field.nrow),.ncol=static_cast<std::uint32_t>(field.ncol)});
        build_terrain(root,world,c,grid);
        return;
    }
    block(root,"asset").add("hfield",{{"name","road"},{"nrow",std::to_string(field.nrow)},
        {"ncol",std::to_string(field.ncol)},{"size",field.size_attr()}});
    const auto ground=geometry::compute_ground_z(c.request.bike)/1000.;
    geom(world,"terrain","hfield",{{"hfield","road"},{"pos",f(field.geom_x_m())+" 0 "+f(field.geom_z_m(ground))},
        {"condim","3"},{"friction","1.2 0.005 0.0001"},{"solref","-130000 -800"},{"material","mat_floor"}});
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
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
void populate_render_heightfield(const mjModel& model,const terrain::RenderGrid3d& grid) {
    const int id=mj_name2id(&model,mjOBJ_HFIELD,"road");
    if(id<0)throw std::invalid_argument("model.terrain3d.render: road heightfield missing");
    // Checked extents: raw mjModel pointer indexing/arithmetic is rejected by
    // -Wunsafe-buffer-usage under the clang native policy (-Werror).
    const auto rows=model_access::readonly_buffer(model.hfield_nrow,model.nhfield);
    const auto columns=model_access::readonly_buffer(model.hfield_ncol,model.nhfield);
    const auto addresses=model_access::readonly_buffer(model.hfield_adr,model.nhfield);
    const auto slot=static_cast<std::size_t>(id);
    if(grid.nrow<2 || grid.ncol<2 ||
       std::cmp_not_equal(rows[slot],grid.nrow) ||
       std::cmp_not_equal(columns[slot],grid.ncol))
        throw std::invalid_argument("model.terrain3d.render: dimension mismatch");
    const auto count=static_cast<std::size_t>(grid.nrow)*grid.ncol;
    const auto address=addresses[slot];
    if(address<0 || count!=grid.normalized_height.size() ||
       static_cast<std::uint64_t>(address)>static_cast<std::uint64_t>(model.nhfielddata) ||
       std::cmp_greater(count,model.nhfielddata-address))
        throw std::invalid_argument("model.terrain3d.render: heightfield storage mismatch");
    for(const float z:grid.normalized_height)
        if(!std::isfinite(z) || z<0.f || z>1.f)
            throw std::invalid_argument("model.terrain3d.render: sample outside [0,1]");
    // Validate the complete matrix before mutating any row. Physics never reads
    // this visual downsample; it retains the immutable Terrain3dSource owner.
    const auto data=model_access::mutable_buffer(model.hfield_data,model.nhfielddata);
    std::ranges::copy(grid.normalized_height,
        data.subspan(static_cast<std::size_t>(address),count).begin());
}
} // namespace bike::model
