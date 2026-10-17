#include "terrain/generators.hpp"
#include "terrain/detail.hpp"
#include <algorithm>
#include <cmath>
#include <tuple>

namespace bike::terrain {
namespace {
void check_weights(const Weights& values,std::span<const std::string_view> allowed,std::string_view name){
    numeric::PythonSum sum;
    for(const auto& [key,value]:values){if(std::ranges::find(allowed,key)==allowed.end())throw std::invalid_argument(std::string(name)+": unknown shape '"+key+"'");detail::nonnegative(value,name);sum.add(value);}
    if(values.empty()||sum.total()<=0.)throw std::invalid_argument(std::string(name)+": weights must include at least one positive entry");
}
double draw(bike::random::NumpyRng& rng,Range bounds){const double value=bounds[1]<=bounds[0]?bounds[0]:rng.uniform(bounds[0],bounds[1]);return rounded(value/.001,0)*.001;}
std::string choose(bike::random::NumpyRng& rng,const Weights& weights){
    auto sorted=weights;std::sort(sorted.begin(),sorted.end(),[](const auto&a,const auto&b){return a.first<b.first;});
    std::vector<double> p;for(const auto& [key,value]:sorted){(void)key;p.push_back(value);}const double sum=numeric::numpy_pairwise_sum(p);for(double& v:p)v/=sum;
    return sorted.at(static_cast<std::size_t>(rng.choice(p))).first;
}
double position(bike::random::NumpyRng& rng,double length,Interval span,std::span<const Interval> occupied,double gap,std::string_view label){
    std::vector<Interval> slots;std::vector<double> widths;
    for(const auto& [a,b]:free_intervals(span,occupied,gap))if(b-a>=length){slots.emplace_back(a,b-length);widths.push_back(((b-length)-a)+.01);}
    if(slots.empty())throw std::invalid_argument("no room left for a "+detail::gd::py_fixed(length,2)+" m "+std::string(label)+": lower the defect rates, shorten the sizes or lengthen the track");
    const double sum=numeric::numpy_pairwise_sum(widths);for(double& v:widths)v/=sum;
    const auto [a,b]=slots.at(static_cast<std::size_t>(rng.choice(widths)));
    const double start=b>a?rng.uniform(a,b):a;return std::min(std::max(rounded(start/.01,0)*.01,a),b);
}
std::size_t defect_count(double rate,double usable){return detail::count(std::floor(rate*usable/100.+.5),"road defect count");}
Obstacle make_pothole(std::string_view edge,double start,double depth,double length,double edge_m){
    if(edge=="sharp")return {Pothole{start,depth,length}};
    if(edge=="sloped")return {SlopedPothole{start,depth,length,edge_m}};
    return {BowlPothole{start,depth,length}};
}
Obstacle make_bump(std::string_view shape,double start,double height,double length,double fraction){
    if(shape=="cosine")return {Bump{start,height,length}};
    const double ramp=length*fraction;return {TrapezoidBump{start,height,ramp,length-2.*ramp}};
}
}
void RoadGeneratorSpec::validate()const{
    (void)detail::seed_plus(seed,0);detail::nonnegative(potholes_per_100m,"potholes_per_100m");detail::nonnegative(bumps_per_100m,"bumps_per_100m");
    detail::range(pothole_depth_m,"pothole_depth_m");detail::range(pothole_length_m,"pothole_length_m");detail::range(bump_height_m,"bump_height_m");detail::range(bump_length_m,"bump_length_m");
    constexpr std::array<std::string_view,3> holes{"sharp","sloped","bowl"};constexpr std::array<std::string_view,2> bumps{"cosine","trapezoid"};
    check_weights(pothole_edge,holes,"pothole_edge");check_weights(bump_shape,bumps,"bump_shape");
    detail::nonnegative(roughness_m,"roughness_m");detail::finite(bump_ramp_fraction,"bump_ramp_fraction");
    if(!(bump_ramp_fraction>0.&&bump_ramp_fraction<=.5))throw std::invalid_argument("bump_ramp_fraction must be in (0, 0.5]");
    detail::finite(runup_m,"runup_m");detail::finite(runout_m,"runout_m");detail::finite(min_gap_m,"min_gap_m");
    detail::finite(pothole_edge_m,"pothole_edge_m");detail::positive(roughness_correlation_m,"roughness_correlation_m");
}
std::vector<Obstacle> generate_road(const RoadGeneratorSpec& spec,double length,std::span<const Interval> reserved){
    spec.validate();detail::positive(length,"road.length_m");const Interval span{spec.runup_m,length-spec.runout_m};const double usable=span.second-span.first;
    if(usable<=0.)throw std::invalid_argument("track leaves no room between runup and runout");
    bike::random::NumpyRng rng(spec.seed,{});std::vector<Interval> occupied(reserved.begin(),reserved.end());std::vector<Obstacle> obstacles;
    struct Defect{bool pothole;std::string shape;double height,length;};std::vector<Defect> defects;
    const auto holes=defect_count(spec.potholes_per_100m,usable),bumps=defect_count(spec.bumps_per_100m,usable);
    for(std::size_t i=0;i<holes;++i){const auto edge=choose(rng,spec.pothole_edge);const double depth=draw(rng,spec.pothole_depth_m);const double size=draw(rng,spec.pothole_length_m);defects.push_back({true,edge,depth,size});}
    for(std::size_t i=0;i<bumps;++i){const auto shape=choose(rng,spec.bump_shape);const double height=draw(rng,spec.bump_height_m);const double size=draw(rng,spec.bump_length_m);defects.push_back({false,shape,height,size});}
    std::stable_sort(defects.begin(),defects.end(),[](const auto&a,const auto&b){return a.length>b.length;});
    for(const auto& d:defects){const double start=position(rng,d.length,span,occupied,spec.min_gap_m,d.pothole?"pothole":"bump");
        Obstacle o=d.pothole?make_pothole(d.shape,start,d.height,d.length,spec.pothole_edge_m):make_bump(d.shape,start,d.height,d.length,spec.bump_ramp_fraction);
        occupied.emplace_back(o.start_m(),o.end_m());obstacles.push_back(std::move(o));
    }
    if(spec.roughness_m>0.)for(const auto& [a,b]:free_intervals(span,occupied,0.))if(b-a>=4.*spec.roughness_correlation_m){
        double size=rounded(b-a,6);while(a+size>b)size=rounded(size-1e-6,6);
        const auto seed=std::to_string(rng.integers(0,2147483647));obstacles.push_back({RoadRoughness{a,size,spec.roughness_m,spec.roughness_correlation_m,seed}});
    }return obstacles;
}
TrackSpec build_road(std::string name,const RoadGeneratorSpec& spec,double length,std::span<const Obstacle> hand,std::string description,std::string surface){
    std::vector<Interval> reserved;for(const auto&o:hand)reserved.emplace_back(o.start_m(),o.end_m());
    auto generated=generate_road(spec,length,reserved);std::vector<Obstacle> all(hand.begin(),hand.end());all.insert(all.end(),generated.begin(),generated.end());
    TrackSpec track{std::move(name),length,std::move(all),std::move(description),std::move(surface),std::nullopt,{}};track.validate();return track;
}
RoadGeneratorSpec road_smooth_spec(){RoadGeneratorSpec s;s.potholes_per_100m=1.;s.pothole_depth_m={.030,.060};s.pothole_length_m={.300,.500};s.bumps_per_100m=2.;s.bump_height_m={.020,.040};s.bump_length_m={.300,.500};s.roughness_m=.002;return s;}
RoadGeneratorSpec road_worn_spec(){return {};}
RoadGeneratorSpec road_broken_spec(){RoadGeneratorSpec s;s.potholes_per_100m=8.;s.pothole_depth_m={.060,.150};s.pothole_length_m={.400,1.};s.pothole_edge={{"sharp",.6},{"sloped",.3},{"bowl",.1}};s.bumps_per_100m=10.;s.bump_height_m={.050,.100};s.bump_length_m={.300,.700};s.bump_shape={{"cosine",.7},{"trapezoid",.3}};s.roughness_m=.005;s.roughness_correlation_m=.150;return s;}
TrackSpec road_smooth(){return build_road("road_smooth",road_smooth_spec(),100.,{},"Well-kept asphalt.");}
TrackSpec road_worn(){return build_road("road_worn",road_worn_spec(),100.,{},"Worn urban asphalt.");}
TrackSpec road_broken(){return build_road("road_broken",road_broken_spec(),100.,{},"Broken-up road.");}
} // namespace bike::terrain
