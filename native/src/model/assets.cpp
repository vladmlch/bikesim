#include "builder_internal.hpp"

namespace bike::model::detail {
// assets.py: authored texture/material tables, in source insertion order.
void build_visual_and_assets(Element& root) {
    auto& visual=root.add("visual");
    visual.add("global",{{"elevation","-20"},{"azimuth","140"},{"offwidth","1920"},{"offheight","1080"}});
    visual.add("quality",{{"shadowsize","4096"}});
    auto& asset=root.add("asset");
    asset.add("texture",{{"name","grid_tex"},{"type","2d"},{"builtin","checker"},{"width","512"},{"height","512"},{"rgb1","0.85 0.85 0.88"},{"rgb2","0.75 0.75 0.78"}});
    asset.add("texture",{{"name","skybox"},{"type","skybox"},{"builtin","gradient"},{"rgb1","0.4 0.6 0.8"},{"rgb2","0.1 0.15 0.25"},{"width","512"},{"height","512"}});
    asset.add("material",{{"name","mat_floor"},{"texture","grid_tex"},{"texrepeat","25 25"},{"reflectance","0.08"}});
    struct Material { const char* name; const char* rgba; const char* specular; const char* shininess; };
    constexpr Material materials[]{
        {"mat_frame","0.14 0.18 0.22 1.0","0.4","0.5"},
        {"mat_linkage","0.88 0.22 0.16 1.0","0.7","0.8"},
        {"mat_yoke","0.55 0.22 0.75 1.0","0.6","0.7"},
        {"mat_kashima","0.82 0.60 0.28 1.0","0.95","0.95"},
        {"mat_damper_body","0.12 0.12 0.12 1.0","0.8","0.85"},
        {"mat_fork_lower","0.16 0.16 0.18 1.0","0.3","0.4"},
        {"mat_tire","0.08 0.08 0.08 1.0","0.05","0.1"},
        {"mat_rim","0.22 0.24 0.26 1.0","0.5","0.6"},
        {"mat_metal","0.75 0.78 0.82 1.0","0.85","0.9"},
        {"mat_saddle","0.06 0.06 0.06 1.0","0.1","0.2"},
        {"mat_handlebar","0.12 0.12 0.14 1.0","0.4","0.5"},
        {"mat_pivot_yellow","0.98 0.85 0.12 1.0","0.9","0.9"},
        {"mat_stand","0.22 0.25 0.30 1.0","0.5","0.6"},
        {"mat_obstacle","0.85 0.50 0.15 1.0","0.4","0.4"},
        {"mat_pit","0.22 0.35 0.55 1.0","0.4","0.5"},
        {"mat_motor","0.10 0.11 0.13 1.0","0.6","0.7"},
        {"mat_battery","0.08 0.08 0.10 1.0","0.5","0.6"},
        {"mat_cg_marker","0.95 0.20 0.85 0.9","0.9","0.9"},
        {"mat_rider","0.18 0.38 0.68 0.95","0.4","0.5"},
    };
    for(const auto& material:materials) asset.add("material",{{"name",material.name},{"rgba",material.rgba},{"specular",material.specular},{"shininess",material.shininess}});
}
} // namespace bike::model::detail
