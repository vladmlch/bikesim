#include "builder_internal.hpp"

namespace bike::model::detail {
void build_environment(Element& world,const BuildContext& c) {
    const auto& mode=c.request.mode;
    const auto ground=geometry::compute_ground_z(c.request.bike)/1000.;
    const auto p10=div(c.fixed.P10,1000.);
    world.add("light",{{"directional","true"},{"diffuse","0.85 0.85 0.85"},{"specular","0.25 0.25 0.25"},{"pos","1.5 -2.5 3.5"},{"dir","-0.3 0.6 -1.0"}});
    world.add("light",{{"directional","true"},{"diffuse","0.45 0.45 0.45"},{"specular","0.1 0.1 0.1"},{"pos","-2.5 2.5 3.0"},{"dir","0.6 -0.6 -1.0"}});
    if(mode=="ride") {
        geom(world,"catch_plane","plane",{{"pos","0 0 "+f(c.request.field.catch_plane_z_m(ground))},{"size","50 50 0.1"},{"material","mat_floor"}});
        return;
    }
    geom(world,"floor","plane",{{"pos","0 0 "+f(ground)},{"size","50 50 0.1"},{"material","mat_floor"},{"conaffinity","1"},{"contype","1"},{"friction","1.3 0.005 0.0001"}});
    if(mode=="stand" || mode=="playground") {
        geom(world,"geom_stand_base","box",{{"pos",f(p10[0])+" -0.35 "+f(ground+.015)},{"size","0.25 0.20 0.015"},{"material","mat_stand"}});
        geom(world,"geom_stand_post","cylinder",{{"fromto",f(p10[0])+" -0.35 "+f(ground+.03)+" "+f(p10[0])+" -0.35 "+f(p10[2])},{"size","0.026"},{"material","mat_stand"}});
        geom(world,"geom_stand_arm","cylinder",{{"fromto",f(p10[0])+" -0.35 "+f(p10[2])+" "+f(p10[0])+" -0.05 "+f(p10[2])},{"size","0.022"},{"material","mat_stand"}});
        geom(world,"geom_stand_clamp","cylinder",{{"fromto",f(p10[0])+" -0.055 "+f(p10[2])+" "+f(p10[0])+" 0.055 "+f(p10[2])},{"size","0.028"},{"material","mat_metal"}});
    }
}
} // namespace bike::model::detail
