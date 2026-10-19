#include "terrain/types.hpp"
#include "terrain/detail.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bike::terrain {
namespace {
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
const std::array<SurfaceSpec,5> registry{{
    {.name="asphalt",.mu_peak=1.05,.mu_slide=.75,.slip_stiffness_per_load=15.,.stribeck_speed_mps=4.5,.description="dry asphalt"},
    {.name="hardpack",.mu_peak=.80,.mu_slide=.60,.slip_stiffness_per_load=12.,.stribeck_speed_mps=4.5,.description="dry hard-packed dirt"},
    {.name="loose",.mu_peak=.55,.mu_slide=.45,.slip_stiffness_per_load=7.,.stribeck_speed_mps=4.5,.description="loose gravel over hardpack"},
    {.name="wet",.mu_peak=.50,.mu_slide=.40,.slip_stiffness_per_load=10.,.stribeck_speed_mps=4.5,.description="wet dirt"},
    {.name="scree",.mu_peak=.45,.mu_slide=.38,.slip_stiffness_per_load=4.5,.stribeck_speed_mps=4.5,.description="alpine scree: loose rock debris over bedrock"}
}};
}
void SurfaceSpec::validate() const {
    detail::positive(mu_peak,"SurfaceSpec.mu_peak");
    detail::positive(mu_slide,"SurfaceSpec.mu_slide");
    detail::positive(slip_stiffness_per_load,"SurfaceSpec.slip_stiffness_per_load");
    detail::positive(stribeck_speed_mps,"SurfaceSpec.stribeck_speed_mps");
    if(mu_slide>mu_peak) throw std::invalid_argument("SurfaceSpec.mu_slide exceeds mu_peak");
}
double SurfaceSpec::mu(double speed) const {
    validate(); detail::finite(speed,"SurfaceSpec.sliding_speed_mps");
    const double decay=std::exp(-std::abs(speed)/stribeck_speed_mps);
    const double value=mu_slide+(mu_peak-mu_slide)*decay;
    detail::finite(value,"SurfaceSpec.mu"); return value;
}
std::vector<double> SurfaceSpec::mu(std::span<const double> speeds) const {
    std::vector<double> out; out.reserve(speeds.size());
    for(const double value:speeds) out.push_back(mu(value));
    return out;
}
std::vector<std::string> available_surfaces() {
    std::vector<std::string> out; out.reserve(registry.size());
    for(const auto& s:registry) out.push_back(s.name);
    return out;
}
const SurfaceSpec& get_surface(std::string_view name) {
    for(const auto& s:registry) if(s.name==name) return s;
    throw std::invalid_argument("unknown surface '"+std::string(name)+"'; available: asphalt, hardpack, loose, wet, scree");
}
void SurfaceSection::validate() const {
    detail::finite(start_m,"SurfaceSection.start_m"); detail::finite(end_m,"SurfaceSection.end_m");
    if(!(0.<=start_m && start_m<end_m)) throw std::invalid_argument("surface interval must satisfy 0 <= start < end");
    (void)get_surface(surface);
}
SurfaceMap::SurfaceMap(SurfaceSpec surface,std::vector<SurfaceSection> sections):surface_(std::move(surface)),sections_(std::move(sections)) {
    std::ranges::stable_sort(sections_,[](const auto& a,const auto& b){return a.start_m<b.start_m;});
    validate();
}
SurfaceMap SurfaceMap::uniform(std::string_view name) { return SurfaceMap(get_surface(name)); }
const SurfaceSpec& SurfaceMap::at(double x) const {
    detail::finite(x,"SurfaceMap.x_m");
    for(const auto& s:sections_) if(s.start_m<=x && x<s.end_m) return get_surface(s.surface);
    return surface_;
}
void SurfaceMap::validate() const {
    surface_.validate(); double previous=0.;
    for(const auto& s:sections_) { s.validate(); if(s.start_m<previous) throw std::invalid_argument("SurfaceMap.sections: unordered or overlapping intervals"); previous=s.end_m; }
}
std::vector<SurfaceSpec> SurfaceMap::surfaces() const {
    std::vector<SurfaceSpec> out{surface_};
    for(const auto& s:sections_) {
        const auto it=std::ranges::find_if(out,[&](const auto& p){return p.name==s.surface;});
        if(it==out.end()) out.push_back(get_surface(s.surface)); else *it=get_surface(s.surface);
    }
    return out;
}
std::string SurfaceMap::name() const { return surface_.name+(sections_.empty()?"":"+zones"); }
double slip_stiffness_n(const SurfaceSpec& surface,double load) {
    if(!(load>0.) || !std::isfinite(load)) return 0.;
    return surface.slip_stiffness_per_load*load;
}
} // namespace bike::terrain
