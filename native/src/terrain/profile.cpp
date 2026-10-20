#include "terrain/profile.hpp"
#include "terrain/detail.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace bike::terrain {
void GradeProfile::validate() const {
    if(knots.size()<2) throw std::invalid_argument("grade needs at least two station/grade pairs");
    for(const auto& p:knots) {detail::finite(p[0],"grade.station");detail::finite(p[1],"grade.grade");}
    if(knots.front()[0]!=0.) throw std::invalid_argument("grade stations must start at zero and strictly increase");
    for(std::size_t i=1;i<knots.size();++i) if(knots[i][0]<=knots[i-1][0]) throw std::invalid_argument("grade stations must start at zero and strictly increase");
}
double GradeProfile::slope(double x) const {
    validate(); detail::finite(x,"grade query");
    std::vector<double> stations,grades; stations.reserve(knots.size());grades.reserve(knots.size());
    for(const auto& p:knots) {stations.push_back(p[0]);grades.push_back(p[1]);}
    return detail::gd::np_interp(x,stations,grades);
}
double GradeProfile::elevation(double x) const {
    validate(); detail::finite(x,"grade query");
    std::vector<double> h(knots.size(),0.);
    for(std::size_t i=1;i<knots.size();++i) h[i]=h[i-1]+(knots[i][0]-knots[i-1][0])*(knots[i-1][1]+knots[i][1])*.5;
    if(x<0.) return knots.front()[1]*x;
    if(x>knots.back()[0]) return h.back()+knots.back()[1]*(x-knots.back()[0]);
    const auto right=std::ranges::upper_bound(knots,x,{},[](const Point& p){return p[0];});
    const auto index=std::ranges::min(static_cast<std::size_t>(right-knots.begin()-1),knots.size()-2);
    const double dx=x-knots[index][0];
    const double change=(knots[index+1][1]-knots[index][1])/(knots[index+1][0]-knots[index][0]);
    const double value=h[index]+knots[index][1]*dx+.5*change*dx*dx;
    detail::finite(value,"grade elevation"); return value;
}
std::vector<double> GradeProfile::slope(std::span<const double> x) const {
    std::vector<double> out;out.reserve(x.size());for(double const v:x)out.push_back(slope(v));return out;
}
std::vector<double> GradeProfile::elevation(std::span<const double> x) const {
    std::vector<double> out;out.reserve(x.size());for(double const v:x)out.push_back(elevation(v));return out;
}
SurfaceMap TrackSpec::surface_map() const { return SurfaceMap(get_surface(surface),surface_sections); }
std::vector<Obstacle> TrackSpec::sorted_obstacles() const {
    auto out=obstacles;std::ranges::stable_sort(out,[](const auto& a,const auto& b){return a.start_m()<b.start_m();});return out;
}
double TrackSpec::datum_shift_m() const {
    // sum(o.datum_shift_m ...) — the oracle's items are the float returns
    // of each obstacle's length/shift property, so the compensated float
    // branch is exact.
    numeric::PythonSum total;for(const auto& o:obstacles)total.add(o.datum_shift_m());
    return total.total()+(grade_profile?grade_profile->elevation(length_m):0.);
}
std::vector<std::pair<double,std::string>> TrackSpec::markers() const {
    std::vector<std::pair<double,std::string>> out;for(const auto& o:sorted_obstacles())if(o.annotate())out.emplace_back(o.start_m(),o.label());return out;
}
void TrackSpec::validate() const {
    if(!std::isfinite(length_m)||length_m<=0.)throw std::invalid_argument("track '"+name+"' has non-positive length "+detail::gd::py_repr(length_m));
    (void)get_surface(surface);
    if(grade_profile){grade_profile->validate();if(grade_profile->knots.back()[0]>length_m)throw std::invalid_argument("grade stations must fit inside the track");}
    const auto mapping=surface_map();
    for(const auto& s:mapping.sections())if(s.end_m>length_m)throw std::invalid_argument("material intervals must fit inside the track");
    const auto ordered=sorted_obstacles();
    for(const auto& o:ordered){o.validate();if(o.start_m()<0.||o.end_m()>length_m)throw std::invalid_argument("track '"+name+"': "+o.label()+" falls outside the track");}
    for(std::size_t i=1;i<ordered.size();++i)if(ordered[i].start_m()<ordered[i-1].end_m())throw std::invalid_argument("track '"+name+"': obstacles may not overlap");
}
std::vector<double> build_profile(const TrackSpec& track,std::span<const double> x) {
    track.validate();for(double const v:x)detail::finite(v,"build_profile.x");
    std::vector<double> z(x.size(),0.);double datum=0.;
    for(const auto& o:track.sorted_obstacles()) {
        for(std::size_t i=0;i<x.size();++i)if(x[i]>=o.start_m())z[i]=datum;
        if(o.length_m()>0.){
            std::vector<std::size_t> indices;std::vector<double> local;
            for(std::size_t i=0;i<x.size();++i)if(x[i]>=o.start_m()&&x[i]<o.end_m()){indices.push_back(i);local.push_back(x[i]-o.start_m());}
            if(!indices.empty()){const auto values=o.elevation(local);for(std::size_t i=0;i<indices.size();++i)z[indices[i]]=datum+values[i];}
        }
        datum+=o.datum_shift_m();
        for(std::size_t i=0;i<x.size();++i)if(x[i]>=o.end_m())z[i]=datum;
    }
    if(track.grade_profile)for(std::size_t i=0;i<x.size();++i)z[i]+=track.grade_profile->elevation(std::clamp(x[i],0.,track.length_m));
    for(double const v:z)detail::finite(v,"build_profile.z");
    return z;
}
std::vector<Point> profile(const TrackSpec& track,double resolution) {
    detail::positive(resolution,"profile.resolution_m");
    const auto x=detail::arange(0.,track.length_m+resolution,resolution);const auto z=build_profile(track,x);
    std::vector<Point> out;out.reserve(x.size());for(std::size_t i=0;i<x.size();++i)out.push_back({x[i],z[i]});return out;
}
Interval profile_extent(const TrackSpec& track,double resolution) {
    const auto vertices=profile(track,resolution);double lo=vertices.front()[1],hi=lo;
    for(const auto& p:vertices){lo=std::min(lo,p[1]);hi=std::max(hi,p[1]);}return {lo,hi};
}
ProfileQuery::ProfileQuery(std::vector<Point> vertices,double significant,double fraction,double angle):query_([&]{
    std::vector<double>x,z;x.reserve(vertices.size());z.reserve(vertices.size());
    for(const auto& p:vertices){x.push_back(p[0]);z.push_back(p[1]);}
    return biketyre::ProfileQuery(std::move(x),std::move(z),significant,fraction,angle);
}()) {}
ProfileContact ProfileQuery::contact(Point center,double radius,std::optional<int> previous) const {return query_.contact(center,radius,previous);}
ProfileContact validated_profile_contact(Point point,Point normal,double delta,int segment,bool multi){
    for(double const v:point)detail::finite(v,"contact point");
    for(double const v:normal)detail::finite(v,"contact normal");
    detail::finite(delta,"penetration");
    if(std::abs(biketyre::py_hypot(normal[0],normal[1])-1.)>1e-10)throw std::invalid_argument("contact normal must be a unit vector");
    return {.point=point,.normal=normal,.delta=delta,.segment_id=segment,.multi_support=multi};
}
ProfileContact closest_profile_contact(std::vector<Point> vertices,Point center,double radius,std::optional<int> previous){return ProfileQuery(std::move(vertices)).contact(center,radius,previous);}
ProfileContact closest_profile_contact(Point center,double radius,std::vector<Point> vertices,std::optional<int> previous){return ProfileQuery(std::move(vertices)).contact(center,radius,previous);}
SignedProfile::SignedProfile(std::vector<Point> vertices):vertices_(std::move(vertices)),query_(vertices_){}
double SignedProfile::height(double x) const {return query_.height(x);}
std::vector<double> SignedProfile::height(std::span<const double>x)const{std::vector<double>out;out.reserve(x.size());for(double const v:x)out.push_back(height(v));return out;}
ProfileDistance SignedProfile::query(std::span<const Point> points)const{
    ProfileDistance out;out.distance.reserve(points.size());out.normal.reserve(points.size());out.point.reserve(points.size());out.segment.reserve(points.size());out.nonsmooth.reserve(points.size());
    // Batch validation precedes queries, matching the source array boundary.
    for(const auto& p:points){detail::finite(p[0],"distance queries.x");detail::finite(p[1],"distance queries.z");}
    for(const auto& p:points){
        const auto row=query_.query(p);out.distance.push_back(row.distance);out.normal.push_back(row.normal);
        out.point.push_back(row.point);out.segment.push_back(static_cast<std::size_t>(row.segment));out.nonsmooth.push_back(row.nonsmooth);
    }return out;
}
ProfileDistance signed_profile_distance(std::vector<Point>vertices,std::span<const Point>points){return SignedProfile(std::move(vertices)).query(points);}
} // namespace bike::terrain
