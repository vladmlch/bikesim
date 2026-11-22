#include "terrain/types.hpp"
#include "terrain/detail.hpp"
#include "numeric_sincos.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <type_traits>

namespace bike::terrain {
namespace {
constexpr double pi=std::numbers::pi;
template<class T> constexpr bool faceted=std::is_same_v<T,RockGarden> || std::is_same_v<T,RoadRoughness>;
template<class T> auto make_nodes(const T& o) {
    detail::positive(o.correlation_length_m,"RockGarden.correlation_length_m");
    detail::nonnegative(o.section_length_m,"RockGarden.section_length_m");
    const double spacing=o.correlation_length_m/2.;
    const auto n=detail::count(std::max(std::ceil(o.section_length_m/spacing)+1.,4.),"RockGarden.nodes");
    auto x=detail::gd::np_linspace(0.,o.section_length_m,n);
    bike::random::NumpyRng rng(o.seed,{});
    std::vector<double> raw(n); rng.normal01(raw);
    // scipy.ndimage.gaussian_filter1d(sigma=1,order=0,truncate=4,
    // mode='nearest'): normalized exp(-x*x/2), then symmetric correlate1d.
    std::array<double,9> weights{};
    for(std::size_t i=0;i<weights.size();++i) {
        const double k=static_cast<double>(i)-4.; weights[i]=std::exp(-.5*(k*k));
    }
    const double total=numeric::numpy_pairwise_sum(weights);
    for(double& w:weights) w/=total;
    std::vector<double> shaped(n);
    const double taper_len=std::min(o.correlation_length_m*2.,o.section_length_m/2.);
    double peak=0.;
    for(std::size_t i=0;i<n;++i) {
        double smooth=raw[i]*weights[4];
        // NI_Correlate1D's symmetric loop walks negative offsets -radius
        // through -1: outermost pair first, after the center contribution.
        // Whether `out += (l+r)*w` contracts to fmadd is a wheel-build
        // property — numerics.scipy_correlate.lowering is measured per
        // wheel (AppleClang: contracted; manylinux: separate); spell the
        // recorded lowering because the project build pins
        // -ffp-contract=off.
        for(std::size_t j=4;j>0;--j) {
            const std::size_t left=i<j?0:i-j;
            const std::size_t right=std::min(i+j,n-1);
#if BIKE_SCIPY_CORRELATE_CONTRACTED
            smooth=std::fma(raw[left]+raw[right],weights[4-j],smooth);
#else
            smooth+=(raw[left]+raw[right])*weights[4-j];
#endif
        }
        double taper=1.;
        if(x[i]<taper_len) taper=.5*(1.-numeric::np_cos(pi*x[i]/taper_len));
        if(x[i]>o.section_length_m-taper_len) taper=.5*(1.-numeric::np_cos(pi*(o.section_length_m-x[i])/taper_len));
        shaped[i]=smooth*taper; peak=std::max(peak,std::abs(shaped[i]));
    }
    if(peak>0.) for(double& value:shaped) value=value*(o.amplitude_m/peak);
    return std::pair{std::move(x),std::move(shaped)};
}
// ISO 8608 harmonic synthesis: M cosine modes with NumpyRng phases uniform on
// [0, 2*pi) and amplitudes ~ n^(-waviness/2) over the spatial-frequency band
// [max(1/section_length, 0.2), 1/(2*correlation_length)] cycles/metre. The
// edge taper matches make_nodes; the untapered interior RMS — not the peak —
// is normalized to amplitude_m/3 so the level is section-length invariant.
std::pair<std::vector<double>,std::vector<double>> iso8608_nodes(const RoadRoughness& o) {
    detail::positive(o.correlation_length_m,"RoadRoughness.correlation_length_m");
    detail::nonnegative(o.section_length_m,"RoadRoughness.section_length_m");
    if(o.spectrum!="gaussian"&&o.spectrum!="iso8608")throw std::invalid_argument("RoadRoughness.spectrum: expected 'gaussian' or 'iso8608'");
    detail::finite(o.waviness,"RoadRoughness.waviness");
    if(o.waviness<1.||o.waviness>4.)throw std::invalid_argument("RoadRoughness.waviness must lie in [1,4]");
    const double spacing=o.correlation_length_m/2.;
    const auto n=detail::count(std::max(std::ceil(o.section_length_m/spacing)+1.,4.),"RoadRoughness.nodes");
    auto x=detail::gd::np_linspace(0.,o.section_length_m,n);
    constexpr std::size_t modes=64;
    const double n_min=std::max(o.section_length_m>0.?1./o.section_length_m:0.,0.2);
    const double n_max=1./(2.*o.correlation_length_m);
    const double dn=n_max>n_min?(n_max-n_min)/static_cast<double>(modes-1):0.;
    bike::random::NumpyRng rng(o.seed,{});
    std::vector<double> freq(modes),phase(modes),weight(modes);
    rng.uniform(0.,2.*pi,phase);
    for(std::size_t m=0;m<modes;++m){freq[m]=n_min+static_cast<double>(m)*dn;weight[m]=std::pow(freq[m],-.5*o.waviness);}
    const double taper_len=std::min(o.correlation_length_m*2.,o.section_length_m/2.);
    std::vector<double> z(n,0.);
    for(std::size_t i=0;i<n;++i) {
        double sum=0.;
        for(std::size_t m=0;m<modes;++m)sum+=weight[m]*numeric::np_cos(2.*pi*freq[m]*x[i]+phase[m]);
        double taper=1.;
        if(x[i]<taper_len)taper=.5*(1.-numeric::np_cos(pi*x[i]/taper_len));
        if(x[i]>o.section_length_m-taper_len)taper=.5*(1.-numeric::np_cos(pi*(o.section_length_m-x[i])/taper_len));
        z[i]=sum*taper;
    }
    double sumsq=0.;std::size_t interior=0;
    for(std::size_t i=0;i<n;++i)if(x[i]>=taper_len&&x[i]<=o.section_length_m-taper_len){sumsq+=z[i]*z[i];++interior;}
    if(interior==0){for(double const v:z)sumsq+=v*v;interior=n;}
    const double rms=interior>0?std::sqrt(sumsq/static_cast<double>(interior)):0.;
    if(rms>0.)for(double& v:z)v*=o.amplitude_m/3./rms;
    return {std::move(x),std::move(z)};
}
template<class T> double length(const T& s) {
    if constexpr(std::is_same_v<T,SquareEdge>) return s.ledge_length_m;
    else if constexpr(std::is_same_v<T,Pothole> || std::is_same_v<T,SlopedPothole> || std::is_same_v<T,BowlPothole>) return s.hole_length_m;
    else if constexpr(std::is_same_v<T,Washboard>) return s.wavelength_m*static_cast<double>(s.n_waves);
    else if constexpr(std::is_same_v<T,GOut>) return s.dip_length_m;
    else if constexpr(std::is_same_v<T,Drop>||std::is_same_v<T,StepUp>) return s.ramp_m;
    else if constexpr(std::is_same_v<T,Kicker>) return s.ramp_m+s.gap_m+s.landing_m;
    else if constexpr(std::is_same_v<T,Roots> || faceted<T>) return s.section_length_m;
    else if constexpr(std::is_same_v<T,Bump>) return s.bump_length_m;
    else if constexpr(std::is_same_v<T,TrapezoidBump>) return 2.*s.ramp_m+s.plateau_m;
    else {
        if(s.steps.empty()) return 0.;
        // sum(length for _, length in self.steps): the qualified presets
        // use float literals, so the compensated float branch is exact.
        // A hand-written Python config could mix int step lengths (3.13
        // adds those via naive +=); the C++ steps are std::array<double,2>
        // so the item-kind distinction is already erased — residual edge
        // documented in the W1.2x report.
        numeric::PythonSum total; for(const auto& step:s.steps) total.add(step[1]);
        return total.total()+static_cast<double>(s.steps.size())*s.transition_m;
    }
}
}
double Kicker::landing_drop_m() const { return landing_m*std::tan(detail::gd::np_radians(landing_angle_deg)); }
double Kicker::launch_angle_deg() const { return detail::gd::np_degrees(std::atan2(height_m,ramp_m)); }
double SlopedPothole::effective_edge_m() const { return std::min(edge_m,hole_length_m/2.); }
std::vector<double> Roots::bump_centres_m() const {
    if(n_bumps<=0) throw std::invalid_argument("Roots.n_bumps must be positive");
    const auto n=detail::count(static_cast<double>(n_bumps),"Roots.n_bumps");
    std::vector<double> out(n);
    const double span=section_length_m-2.*width_m, spacing=span/static_cast<double>(n);
    bike::random::NumpyRng rng(seed,{});
    for(std::size_t i=0;i<n;++i) {
        const double even=width_m+span*(static_cast<double>(i)+.5)/static_cast<double>(n);
        const double offset=rng.uniform(-jitter,jitter)*spacing;
        out[i]=detail::gd::np_clip_scalar(even+offset,width_m,section_length_m-width_m);
    }
    return out;
}
std::pair<std::vector<double>,std::vector<double>> RockGarden::nodes() const { return make_nodes(*this); }
std::pair<std::vector<double>,std::vector<double>> RoadRoughness::nodes() const {
    if(spectrum=="gaussian")return make_nodes(*this);
    if(spectrum=="iso8608")return iso8608_nodes(*this);
    throw std::invalid_argument("RoadRoughness.spectrum: expected 'gaussian' or 'iso8608'");
}
std::string_view Obstacle::kind() const {
    constexpr std::array<std::string_view,15> names{"SquareEdge","Pothole","Washboard","GOut","Drop","Kicker","Roots","RockGarden","Bump","TrapezoidBump","SlopedPothole","BowlPothole","RoadRoughness","SteppedClimb","StepUp"};
    return names.at(shape.index());
}
double Obstacle::start_m() const { return std::visit([](const auto& s){return s.start_m;},shape); }
void Obstacle::set_start_m(double value) { std::visit([&](auto& s){s.start_m=value;},shape); }
double Obstacle::length_m() const { return std::visit([](const auto& s){return length(s);},shape); }
double Obstacle::end_m() const { return start_m()+length_m(); }
double Obstacle::datum_shift_m() const {
    return std::visit([](const auto& s)->double {
        using T=std::decay_t<decltype(s)>;
        if constexpr(std::is_same_v<T,Drop>) return -s.height_m;
        else if constexpr(std::is_same_v<T,Kicker>) return -s.landing_drop_m();
        else if constexpr(std::is_same_v<T,StepUp>) return s.height_m;
        else if constexpr(std::is_same_v<T,SteppedClimb>) {
            double total=0.,previous=0.;
            for(const auto& step:s.steps) { const double g=step[0]/100.; total+=s.transition_m*(previous+g)*.5+step[1]*g; previous=g; }
            return total;
        } else return 0.;
    },shape);
}
std::string Obstacle::label() const { return std::string(kind())+"@"+detail::gd::py_fixed(start_m(),0)+"m"; }
bool Obstacle::annotate() const { return !std::holds_alternative<RoadRoughness>(shape); }
bool Obstacle::is_pothole() const { return std::holds_alternative<Pothole>(shape)||std::holds_alternative<SlopedPothole>(shape)||std::holds_alternative<BowlPothole>(shape); }
bool Obstacle::is_bump() const { return std::holds_alternative<Bump>(shape)||std::holds_alternative<TrapezoidBump>(shape); }
void Obstacle::validate() const {
    detail::finite(start_m(),"Obstacle.start_m"); detail::nonnegative(length_m(),"Obstacle.length_m");
    detail::finite(end_m(),"Obstacle.end_m"); detail::finite(datum_shift_m(),"Obstacle.datum_shift_m");
    std::visit([](const auto& s) {
        using T=std::decay_t<decltype(s)>;
        if constexpr(requires { s.height_m; }) detail::finite(s.height_m,"Obstacle.height_m");
        if constexpr(requires { s.depth_m; }) detail::finite(s.depth_m,"Obstacle.depth_m");
        if constexpr(requires { s.amplitude_m; }) detail::finite(s.amplitude_m,"Obstacle.amplitude_m");
        if constexpr(requires { s.seed; }) (void)detail::seed_plus(s.seed,0);
        if constexpr(faceted<T>) detail::positive(s.correlation_length_m,"Obstacle.correlation_length_m");
        if constexpr(std::is_same_v<T,Roots>) { detail::finite(s.width_m,"Roots.width_m"); detail::nonnegative(s.jitter,"Roots.jitter"); if(s.n_bumps<=0) throw std::invalid_argument("Roots.n_bumps must be positive"); }
        if constexpr(std::is_same_v<T,Washboard>) { detail::positive(s.wavelength_m,"Washboard.wavelength_m"); if(s.n_waves<0) throw std::invalid_argument("Washboard.n_waves must be nonnegative"); }
        if constexpr(std::is_same_v<T,SteppedClimb>) { detail::finite(s.transition_m,"SteppedClimb.transition_m"); for(const auto& step:s.steps) {detail::finite(step[0],"SteppedClimb.grade_pct");detail::finite(step[1],"SteppedClimb.length_m");} }
        if constexpr(std::is_same_v<T,StepUp>) { detail::positive(s.height_m,"StepUp.height_m"); detail::positive(s.ramp_m,"StepUp.ramp_m"); }
        if constexpr(std::is_same_v<T,RoadRoughness>) {
            if(s.spectrum!="gaussian"&&s.spectrum!="iso8608")throw std::invalid_argument("RoadRoughness.spectrum: expected 'gaussian' or 'iso8608'");
            detail::finite(s.waviness,"RoadRoughness.waviness");
            if(s.waviness<1.||s.waviness>4.)throw std::invalid_argument("RoadRoughness.waviness must lie in [1,4]");
        }
    },shape);
}
std::vector<double> Obstacle::elevation(std::span<const double> points) const {
    validate(); for(double const x:points) detail::finite(x,"Obstacle.local_x");
    return std::visit([&](const auto& s) {
        using T=std::decay_t<decltype(s)>;
        std::vector<double> out(points.size(),0.);
        if constexpr(faceted<T>) {
            const auto [x,z]=s.nodes(); for(std::size_t i=0;i<out.size();++i) out[i]=detail::gd::np_interp(points[i],x,z);
        } else if constexpr(std::is_same_v<T,Roots>) {
            const double half=s.width_m/2.;
            for(double const center:s.bump_centres_m()) for(std::size_t i=0;i<out.size();++i) {
                const double d=std::abs(points[i]-center);
                if(d<half) out[i]+=s.height_m*.5*(1.+numeric::np_cos(pi*d/half));
            }
        } else if constexpr(std::is_same_v<T,SteppedClimb>) {
            if(s.steps.empty() || length(s)<=0.) return out;
            double current_x=0.,current_z=0.,previous=0.;
            for(const auto& step:s.steps) {
                const double grade=step[0]/100.,tlen=s.transition_m;
                if(tlen>0.) {
                    for(std::size_t i=0;i<out.size();++i) if(points[i]>=current_x && points[i]<current_x+tlen) {
                        const double u=points[i]-current_x,t=u/tlen;
                        out[i]=current_z+tlen*(previous*t+(grade-previous)*(std::pow(t,3.)-.5*std::pow(t,4.)));
                    }
                    current_x+=tlen; current_z+=tlen*(previous+grade)*.5;
                }
                if(step[1]>0.) {
                    for(std::size_t i=0;i<out.size();++i) if(points[i]>=current_x && points[i]<current_x+step[1]) out[i]=current_z+grade*(points[i]-current_x);
                    current_x+=step[1];current_z+=grade*step[1];
                }
                previous=grade;
            }
            for(std::size_t i=0;i<out.size();++i) if(points[i]>=length(s)) out[i]=current_z;
        } else {
            for(std::size_t i=0;i<out.size();++i) {
                const double x=points[i];
                if constexpr(std::is_same_v<T,SquareEdge>) out[i]=s.height_m;
                else if constexpr(std::is_same_v<T,Pothole>) out[i]=-s.depth_m;
                else if constexpr(std::is_same_v<T,Washboard>) out[i]=s.amplitude_m*numeric::np_sin(2.*pi*x/s.wavelength_m);
                else if constexpr(std::is_same_v<T,GOut>) out[i]=-s.depth_m*.5*(1.-numeric::np_cos(2.*pi*x/s.dip_length_m));
                else if constexpr(std::is_same_v<T,Drop>) { if(s.ramp_m>0.) out[i]=-s.height_m*(x/s.ramp_m); }
                else if constexpr(std::is_same_v<T,Kicker>) { if(x<s.ramp_m) out[i]=s.height_m*(x/s.ramp_m); if(x>=s.ramp_m+s.gap_m) out[i]=-(x-(s.ramp_m+s.gap_m))*std::tan(detail::gd::np_radians(s.landing_angle_deg)); }
                else if constexpr(std::is_same_v<T,Bump>) out[i]=s.height_m*.5*(1.-numeric::np_cos(2.*pi*x/s.bump_length_m));
                else if constexpr(std::is_same_v<T,BowlPothole>) out[i]=-s.depth_m*.5*(1.-numeric::np_cos(2.*pi*x/s.hole_length_m));
                else if constexpr(std::is_same_v<T,TrapezoidBump>) {
                    double z=s.height_m;
                    if(s.ramp_m>0.) { if(x<s.ramp_m) z=s.height_m*(x/s.ramp_m); if(x>s.ramp_m+s.plateau_m) z=s.height_m*((length(s)-x)/s.ramp_m); }
                    out[i]=detail::gd::np_clip_scalar(z,0.,s.height_m);
                } else if constexpr(std::is_same_v<T,SlopedPothole>) {
                    const double edge=s.effective_edge_m(); double z=-s.depth_m;
                    if(edge>0.) { if(x<edge) z=-s.depth_m*(x/edge); if(x>s.hole_length_m-edge) z=-s.depth_m*((s.hole_length_m-x)/edge); }
                    out[i]=detail::gd::np_clip_scalar(z,-s.depth_m,0.);
                } else if constexpr(std::is_same_v<T,StepUp>) {
                    // C1 cosine ramp on [0, ramp_m); the deck past the ramp is
                    // carried by datum_shift_m, so the local profile holds h.
                    if(s.ramp_m>0.)out[i]=x<s.ramp_m?s.height_m*.5*(1.-numeric::np_cos(pi*x/s.ramp_m)):s.height_m;
                }
            }
        }
        for(double const value:out) detail::finite(value,"Obstacle.elevation");
        return out;
    },shape);
}
} // namespace bike::terrain
