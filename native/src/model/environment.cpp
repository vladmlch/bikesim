#include "builder_internal.hpp"

namespace bike::model::detail {
void build_environment(Element& world,const BuildContext& c) {
    const auto& mode=c.request.mode;
    const auto ground=geometry::compute_ground_z(c.request.bike)/1000.;
    world.add("light",{{"directional","true"},{"diffuse","0.85 0.85 0.85"},{"specular","0.25 0.25 0.25"},{"pos","1.5 -2.5 3.5"},{"dir","-0.3 0.6 -1.0"}});
    world.add("light",{{"directional","true"},{"diffuse","0.45 0.45 0.45"},{"specular","0.1 0.1 0.1"},{"pos","-2.5 2.5 3.0"},{"dir","0.6 -0.6 -1.0"}});
    if(mode=="ride") {
        if(c.is_spatial() && c.request.spatial.has_value()) {
            const auto& spatial=*c.request.spatial;
            const double half_x=std::max(50.,c.request.field.radius_x_m+10.);
            const double half_y=std::max(50.,std::max(c.request.field.radius_y_m,spatial.corridor_half_width_m)+10.);
            geom(world,"catch_plane","plane",{
                {"pos",f(c.request.field.geom_x_m())+" 0 "+f(c.request.field.catch_plane_z_m(ground))},
                {"size",f(half_x)+" "+f(half_y)+" 0.1"},
                {"material","mat_floor"}});
            return;
        }
        geom(world,"catch_plane","plane",{{"pos","0 0 "+f(c.request.field.catch_plane_z_m(ground))},{"size","50 50 0.1"},{"material","mat_floor"}});
        return;
    }
    geom(world,"floor","plane",{{"pos","0 0 "+f(ground)},{"size","50 50 0.1"},{"material","mat_floor"},{"conaffinity","1"},{"contype","1"},{"friction","1.3 0.005 0.0001"}});
}
} // namespace bike::model::detail
