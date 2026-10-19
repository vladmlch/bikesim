#include "builder_internal.hpp"

namespace bike::model::detail {
// mujoco/terrain.py. Elevation is populated by T21 into the owned model;
// generating XML does not bake a Python-created heightfield into the asset.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
void build_terrain(Element& root,Element& world,const BuildContext& c) {
    const auto& field=c.request.field;
    block(root,"asset").add("hfield",{{"name","road"},{"nrow",std::to_string(field.nrow)},
        {"ncol",std::to_string(field.ncol)},{"size",field.size_attr()}});
    const auto ground=geometry::compute_ground_z(c.request.bike)/1000.;
    geom(world,"terrain","hfield",{{"hfield","road"},{"pos",f(field.geom_x_m())+" 0 "+f(field.geom_z_m(ground))},
        {"condim","3"},{"friction","1.2 0.005 0.0001"},{"solref","-130000 -800"},{"material","mat_floor"}});
}
} // namespace bike::model::detail
