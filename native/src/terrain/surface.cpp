#include "terrain/types.hpp"
#include "terrain/detail.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bike::terrain {
namespace {
// Stage A calibrated 5-surface registry (Spec §2.1):
// Note that `wet` (wet hard-packed dirt) has lower mu_peak than `loose` gravel (0.50 vs 0.55)
// due to water-film lubrication, yet remains a consolidated substrate with higher longitudinal
// shear stiffness (10.0 > 7.0) and lower micro-rolling loss (crr_delta 0.0035 < 0.0060).
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
const std::array<SurfaceSpec,5> registry{{
    {.name="asphalt",.mu_peak=1.05,.mu_slide=.75,.slip_stiffness_per_load=15.,.stribeck_speed_mps=4.5,.crr_delta=0.,.load_sensitivity=.06,.description="dry asphalt"},
    {.name="hardpack",.mu_peak=.80,.mu_slide=.60,.slip_stiffness_per_load=12.,.stribeck_speed_mps=4.5,.crr_delta=.0025,.load_sensitivity=.08,.description="dry hard-packed dirt"},
    {.name="loose",.mu_peak=.55,.mu_slide=.45,.slip_stiffness_per_load=7.,.stribeck_speed_mps=4.5,.crr_delta=.0060,.load_sensitivity=.12,.description="loose gravel over hardpack"},
    {.name="wet",.mu_peak=.50,.mu_slide=.40,.slip_stiffness_per_load=10.,.stribeck_speed_mps=4.5,.crr_delta=.0035,.load_sensitivity=.12,.description="wet dirt"},
    {.name="scree",.mu_peak=.45,.mu_slide=.38,.slip_stiffness_per_load=4.5,.stribeck_speed_mps=4.5,.crr_delta=.0080,.load_sensitivity=.15,.description="alpine scree: loose rock debris over bedrock"}
}};
}
void SurfaceSpec::validate() const {
    detail::positive(mu_peak,"SurfaceSpec.mu_peak");
    detail::positive(mu_slide,"SurfaceSpec.mu_slide");
    detail::positive(slip_stiffness_per_load,"SurfaceSpec.slip_stiffness_per_load");
    detail::positive(stribeck_speed_mps,"SurfaceSpec.stribeck_speed_mps");
    if(mu_slide>mu_peak) throw std::invalid_argument("SurfaceSpec.mu_slide exceeds mu_peak");
    detail::nonnegative(crr_delta,"SurfaceSpec.crr_delta");
    detail::nonnegative(load_sensitivity,"SurfaceSpec.load_sensitivity");
    if(load_sensitivity>0.5) throw std::invalid_argument("SurfaceSpec.load_sensitivity exceeds 0.5");
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
    detail::nonnegative(blend_m,"SurfaceSection.blend_m");
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
    surface_.validate();
    const std::size_t n=sections_.size();
    for(std::size_t i=0;i<n;++i) {
        const auto& sec=sections_[i];
        sec.validate();
        double b_left=0.;
        if(i==0) {
            if(sec.start_m>0.) {
                b_left=sec.blend_m;
                if(sec.start_m<.5*b_left)
                    throw std::invalid_argument("SurfaceMap.sections[0].blend_m: transition extends before x = 0");
            }
        } else {
            const auto& prev=sections_[i-1];
            if(sec.start_m<prev.end_m)
                throw std::invalid_argument("SurfaceMap.sections: unordered or overlapping intervals");
            if(sec.start_m==prev.end_m) {
                if(prev.blend_m!=sec.blend_m)
                    throw std::invalid_argument("SurfaceMap.sections: adjacent sections must have equal blend_m");
                b_left=sec.blend_m;
            } else {
                if(sec.start_m-prev.end_m<.5*prev.blend_m+.5*sec.blend_m)
                    throw std::invalid_argument("SurfaceMap.sections: transition windows overlap across gap");
                b_left=sec.blend_m;
            }
        }
        const double b_right=sec.blend_m;
        if(sec.end_m-sec.start_m<.5*b_left+.5*b_right)
            throw std::invalid_argument("SurfaceMap.sections: section shorter than transition half-widths");
    }
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
