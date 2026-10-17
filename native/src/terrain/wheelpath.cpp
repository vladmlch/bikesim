#include "terrain/profile.hpp"
#include "terrain/detail.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace bike::terrain {
std::pair<std::vector<double>,std::vector<double>> wheel_centre_path(const Obstacle& obstacle,double radius,double resolution){
    detail::positive(radius,"wheel_centre_path.wheel_radius_m");detail::positive(resolution,"wheel_centre_path.resolution_m");obstacle.validate();
    const double length=obstacle.length_m();
    auto surface=detail::arange(-2.*radius,length+2.*radius+resolution,resolution);
    surface.insert(surface.end(),{-1e-9,0.,std::max(length-1e-9,0.),length});
    std::sort(surface.begin(),surface.end());surface.erase(std::unique(surface.begin(),surface.end()),surface.end());
    std::vector<double> z_surface(surface.size(),0.),local;std::vector<std::size_t> indices;
    for(std::size_t i=0;i<surface.size();++i)if(surface[i]>=0.&&surface[i]<length){local.push_back(surface[i]);indices.push_back(i);}
    if(!local.empty()){const auto heights=obstacle.elevation(local);for(std::size_t i=0;i<indices.size();++i)z_surface[indices[i]]=heights[i];}
    auto center=detail::arange(-radius,length+radius+resolution,resolution);std::vector<double> z_center(center.size());
    for(std::size_t i=0;i<center.size();++i){
        // The maximum ignores every candidate outside the circle. Use a
        // conservative one-node halo; reach retains the exact source test.
        const auto lo_it=std::lower_bound(surface.begin(),surface.end(),center[i]-radius);
        const auto hi_it=std::upper_bound(surface.begin(),surface.end(),center[i]+radius);
        const auto lo=lo_it==surface.begin()?0:static_cast<std::size_t>(lo_it-surface.begin()-1);
        const auto hi=std::min(surface.size(),static_cast<std::size_t>(hi_it-surface.begin())+1);
        double top=-std::numeric_limits<double>::infinity();
        for(std::size_t j=lo;j<hi;++j){const double dx=center[i]-surface[j],reach=radius*radius-dx*dx;if(reach>=0.)top=std::max(top,z_surface[j]+std::sqrt(std::max(reach,0.)));}
        z_center[i]=top;
    }return {std::move(center),std::move(z_center)};
}
double effective_drop_m(const Obstacle& o,double radius,double resolution){const auto result=wheel_centre_path(o,radius,resolution);return std::max(0.,radius-*std::min_element(result.second.begin(),result.second.end()));}
double effective_rise_m(const Obstacle& o,double radius,double resolution){const auto result=wheel_centre_path(o,radius,resolution);return std::max(0.,*std::max_element(result.second.begin(),result.second.end())-radius);}
double bridged_drop_m(double length,double radius){const double half=length/2.;if(half>=radius)return std::numeric_limits<double>::infinity();return radius-std::sqrt(radius*radius-half*half);}
} // namespace bike::terrain
