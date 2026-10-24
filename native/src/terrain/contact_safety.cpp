#include "terrain/profile.hpp"
#include "terrain/detail.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <type_traits>

namespace bike::terrain {
double min_bump_length_m(double height,double radius){return std::numbers::pi*std::sqrt(2.*height*radius);}
double max_bowl_depth_m(double length,double radius){return length*length/(2.*(std::numbers::pi*std::numbers::pi)*radius);}
namespace {
template<class T> double concave(const T& obstacle,double window){
    const auto [x,z]=obstacle.nodes();return concave_span_deg(x,z,window);
}
}
double concave_span_deg(std::span<const double> x,std::span<const double> z,double window){
    if(x.size()!=z.size())throw std::invalid_argument("concave_span_deg: x/z length mismatch");
    if(x.size()<2)return 0.;
    std::vector<double> mid,slopes;mid.reserve(x.size()-1);slopes.reserve(x.size()-1);
    for(std::size_t i=0;i+1<x.size();++i){mid.push_back((x[i]+x[i+1])/2.);slopes.push_back(detail::gd::np_degrees(std::atan((z[i+1]-z[i])/(x[i+1]-x[i]))));}
    double worst=0.;
    for(std::size_t i=0;i+1<slopes.size();++i)for(std::size_t j=i+1;j<slopes.size()&&mid[j]-mid[i]<=window;++j)worst=std::max(worst,slopes[j]-slopes[i]);
    for(std::size_t i=0;i<slopes.size();++i){if(mid[i]<=x.front()+window)worst=std::max(worst,slopes[i]);if(mid[i]>=x.back()-window)worst=std::max(worst,-slopes[i]);}
    return worst;
}
double concave_span_deg(const RockGarden& o,double window){return concave(o,window);}
double concave_span_deg(const RoadRoughness& o,double window){return concave(o,window);}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
std::vector<std::string> single_contact_hazards(const TrackSpec& track,double radius,double kink,double window){
    std::vector<std::string> out;
    for(const auto& obstacle:track.sorted_obstacles())std::visit([&](const auto& o){
        using T=std::decay_t<decltype(o)>;
        if constexpr(std::is_same_v<T,RockGarden>||std::is_same_v<T,RoadRoughness>){
            const double span=concave_span_deg(o,window);
            if(span>=kink)out.push_back(obstacle.label()+": concave span "+detail::gd::py_fixed(span,1)+" deg >= "+detail::gd::py_g(kink)+" deg");
        }else if constexpr(std::is_same_v<T,Bump>){
            const double need=min_bump_length_m(o.height_m,radius);
            if(o.bump_length_m<need-1e-9)out.push_back(obstacle.label()+": length "+detail::gd::py_fixed(o.bump_length_m,3)+" m < "+detail::gd::py_fixed(need,3)+" m");
        }else if constexpr(std::is_same_v<T,BowlPothole>||std::is_same_v<T,GOut>){
            const double limit=max_bowl_depth_m(obstacle.length_m(),radius);
            if(o.depth_m>limit+1e-9)out.push_back(obstacle.label()+": depth "+detail::gd::py_fixed(o.depth_m,3)+" m > "+detail::gd::py_fixed(limit,3)+" m");
        }else out.push_back(obstacle.label()+": "+std::string(obstacle.kind())+" has faces the 2D tyre cannot ride");
    },obstacle.shape);
    return out;
}
} // namespace bike::terrain
