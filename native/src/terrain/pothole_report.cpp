#include "terrain/pothole_report.hpp"
#include "terrain/profile.hpp"
#include <variant>

namespace bike::terrain {
std::vector<PotholeReport> pothole_reports(const TrackSpec& track){
    std::vector<PotholeReport> out;
    for(const auto& obstacle:track.sorted_obstacles()){
        if(obstacle.is_pothole()){
            const double depth=std::visit([](const auto& shape){if constexpr(requires{shape.depth_m;})return shape.depth_m;else return 0.;},obstacle.shape);
            out.push_back({.label=obstacle.label(),.kind=std::string(obstacle.kind()),.start_m=obstacle.start_m(),.length_m=obstacle.length_m(),.declared_depth_mm=depth*1000.,
                .effective_drop_front_mm=effective_drop_m(obstacle,FRONT_WHEEL_RADIUS_M)*1000.,.effective_drop_rear_mm=effective_drop_m(obstacle,REAR_WHEEL_RADIUS_M)*1000.});
        }
    }
    return out;
}
} // namespace bike::terrain
