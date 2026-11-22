// T20 regressions. Authored without executing a project binary. B comparisons
// consume immutable oracle files; missing binary payloads fail, never skip.
#include "support/fixture_test.hpp"
#include "terrain/profile.hpp"
#include "terrain/generators.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace {
namespace t=bike::terrain;
void exact(double actual,double expected,std::string_view label){bt::require(std::bit_cast<std::uint64_t>(actual)==std::bit_cast<std::uint64_t>(expected),label);}
void profile_boundaries(){
    auto track=t::flat(10.);track.obstacles={{.shape=t::SquareEdge{.start_m=2.,.height_m=.125,.ledge_length_m=.5}},{.shape=t::Drop{.start_m=4.,.height_m=.25,.ramp_m=0.}},{.shape=t::Pothole{.start_m=6.,.depth_m=.125,.hole_length_m=.5}}};
    const std::array x{-1.,0.,2.,std::nextafter(2.5,0.),2.5,4.,6.,6.5,10.,15.};
    const std::array expected{0.,0.,.125,.125,0.,-.25,-.375,-.25,-.25,-.25};
    const auto z=t::build_profile(track,x);for(std::size_t i=0;i<z.size();++i)exact(z[i],expected[i],"half-open obstacle and permanent datum boundaries");
    exact(track.datum_shift_m(),-.25,"net datum shift");bt::require(track.markers().size()==3,"feature markers");
    track.obstacles.push_back({.shape=t::SquareEdge{.start_m=2.25,.height_m=.125,.ledge_length_m=.5}});bt::invalid([&]{track.validate();});
    auto duplicate=t::flat(10.);duplicate.obstacles={{.shape=t::Drop{.start_m=2.,.height_m=.25,.ramp_m=0.}},{.shape=t::Drop{.start_m=2.,.height_m=.5,.ramp_m=0.}}};
    exact(t::build_profile(duplicate,std::array{2.})[0],-.75,"stable equal-start zero-width drops compose");
    bt::invalid([]{(void)t::profile(t::flat(10.),0.);});bt::invalid([]{t::flat(-1.).validate();});
}
void grade_and_field(){
    const t::GradeProfile grade{{{0.,0.},{2.,.25},{4.,.25},{6.,0.}}};
    exact(grade.elevation(0.),0.,"grade origin");exact(grade.elevation(2.),.25,"integrated ramp");exact(grade.elevation(4.),.75,"integrated plateau");exact(grade.elevation(6.),1.,"integrated ramp down");
    auto track=t::flat(6.);track.grade_profile=grade;
    const auto z=t::build_profile(track,std::array{-2.,0.,6.,9.});exact(z[0],0.,"flat leadin");exact(z[2],1.,"finish elevation");exact(z[3],1.,"clipped grade runout");
    bt::invalid([]{t::GradeProfile{{{0.,0.},{0.,.1}}}.validate();});
    bt::invalid([]{t::GradeProfile{{{1.,0.},{2.,.1}}}.validate();});
    bt::invalid([&]{(void)grade.slope(std::numeric_limits<double>::quiet_NaN());});
    const t::HeightFieldSpec field;
    bt::require(field.is_default()&&field.nrow==2&&field.ncol==24001,"shipped fixed field dimensions");
    exact(field.track_length_m(),120.,"field length");exact(field.resolution_m(),.005,"field resolution");exact(field.geom_x_m(),60.,"track origin placement");
    const auto x=field.track_x();exact(x.front(),0.,"first field station");exact(x.back(),120.,"last field station pinned");
    const auto normalized=t::build_field_data(t::flat(),field);
    bt::require(normalized.size()==48002,"row-major field sample count");
    bt::require(std::equal(normalized.begin(),normalized.begin()+24001,normalized.begin()+24001),"heightfield rows are identical");
    bt::require(std::ranges::all_of(normalized,[](double v){return v>=0.&&v<=1.;}),"normalized field range");
    bt::require(t::HeightFieldSpec::for_track_length(115.).is_default(),"five-metre runout exact threshold");
    const auto expanded=t::HeightFieldSpec::for_track_length(116.);exact(expanded.track_length_m(),130.,"expanded field length rounds to ten metres");exact(expanded.resolution_m(),.005,"expanded grid retains resolution");
    const auto raised=t::HeightFieldSpec::for_track(track);bt::require(raised.fits(t::profile_extent(track)),"expanded vertical envelope fits grade");
    exact(raised.catch_plane_z_m(0.),raised.geom_z_m(0.)-1.,"catch plane is below field floor");
    bt::invalid([&]{(void)field.normalize(std::array{0.,1.});});bt::invalid([&]{(void)field.with_length(.001);});
    bt::invalid([&]{t::assert_track_fits(t::flat(121.),field);});
}
void obstacles_and_wheelpath(){
    const std::array<double,1> middle{.5};
    exact(t::Obstacle{.shape=t::Bump{.start_m=0.,.height_m=.125,.bump_length_m=1.}}.elevation(middle)[0],.125,"raised cosine crest");
    exact(t::Obstacle{.shape=t::BowlPothole{.start_m=0.,.depth_m=.125,.hole_length_m=1.}}.elevation(middle)[0],-.125,"bowl trough");
    exact(t::Obstacle{.shape=t::SlopedPothole{.start_m=0.,.depth_m=.125,.hole_length_m=1.,.edge_m=1.}}.elevation(middle)[0],-.125,"overlapping chamfers form a V");
    exact(t::Obstacle{.shape=t::TrapezoidBump{.start_m=0.,.height_m=.125,.ramp_m=.5,.plateau_m=0.}}.elevation(middle)[0],.125,"zero-plateau triangular bump");
    const t::Obstacle kicker{.shape=t::Kicker{.start_m=0.,.height_m=.25,.ramp_m=1.,.gap_m=1.,.landing_angle_deg=0.,.landing_m=1.}};
    const auto ramp=kicker.elevation(std::array{.5,1.,2.});exact(ramp[0],.125,"kicker takeoff ramp");exact(ramp[1],0.,"kicker lip steps down");
    const t::Obstacle roots{.shape=t::Roots{.start_m=0.,.n_bumps=7,.height_m=.08,.width_m=.25,.section_length_m=7.,.jitter=.35,.seed="2029"}};
    const std::array coarse{0.,1.,2.,3.,4.,5.,6.,7.};const auto a=roots.elevation(coarse);
    const std::array fine{0.,.5,1.,1.5,2.,2.5,3.,3.5,4.,4.5,5.,5.5,6.,6.5,7.};const auto b=roots.elevation(fine);
    for(std::size_t i=0;i<a.size();++i)exact(a[i],b[i*2],"root shape is independent of sample grid");
    const t::RockGarden garden{.start_m=0.,.section_length_m=8.,.amplitude_m=.06,.correlation_length_m=.25,.seed="1701"};const auto [nodes,heights]=garden.nodes();
    bt::require(nodes.size()==65&&heights.size()==65,"correlation sets internal rock node grid");bt::require(heights.front()==0.&&heights.back()==0.,"rock entry and exit taper, allowing the source signed zero");
    double peak=0.;for(double const v:heights)peak=std::max(peak,std::abs(v));bt::require(std::abs(peak-.06)<1e-15,"rock amplitude normalization");
    const auto climb=t::climb_steps();exact(climb.obstacles[0].length_m(),90.,"all stepped climb transitions included");exact(climb.datum_shift_m(),13.125,"stepped climb integral");
    const auto climb_z=t::build_profile(climb,std::array{0.,15.,105.,115.});exact(climb_z[0],0.,"stepped climb leadin");exact(climb_z[2],climb.datum_shift_m(),"stepped climb downstream datum");
    const t::Obstacle hole{.shape=t::Pothole{.start_m=0.,.depth_m=.25,.hole_length_m=.6}};
    const double drop=t::effective_drop_m(hole,.5,.0005);
    bt::require(std::abs(drop-.1)<.001,"rigid wheel bridges the rectangular hole at the analytic sag, within the documented 0.5 mm grid bound");
    bt::require(std::isinf(t::bridged_drop_m(1.,.5)),"diameter-wide hole cannot be bridged");
    bt::require(std::abs(t::effective_rise_m(t::Obstacle{.shape=t::Bump{.start_m=0.,.height_m=.125,.bump_length_m=1.}},.5,.001)-.125)<.001,"wheel rises by the bump height");
}
void materials_and_contacts(){
    bt::require(t::available_surfaces()==std::vector<std::string>{"asphalt","hardpack","loose","wet","scree"},"surface registry order");
    const t::SurfaceMap map(t::get_surface("hardpack"),{{.start_m=2.,.end_m=3.,.surface="wet"},{.start_m=4.,.end_m=5.,.surface="scree"}});
    bt::require(map.at(-1.).name=="hardpack"&&map.at(2.).name=="wet"&&map.at(3.).name=="hardpack"&&map.at(5.).name=="hardpack","half-open surface sections and default outside map");
    bt::require(map.name()=="hardpack+zones"&&map.surfaces().size()==3,"surface map identity and inventory");
    exact(t::get_surface("scree").mu(0.),.45,"scree peak friction");
    exact(t::slip_stiffness_n(t::get_surface("scree"),100.),450.,"slip stiffness per normal load");
    exact(t::slip_stiffness_n(t::get_surface("scree"),std::numeric_limits<double>::infinity()),0.,"nonfinite load sentinel remains local");
    bt::invalid([]{(void)t::SurfaceMap(t::get_surface("hardpack"),{{.start_m=1.,.end_m=3.,.surface="wet"},{.start_m=2.,.end_m=4.,.surface="loose"}});});
    bt::invalid([&]{(void)map.at(std::numeric_limits<double>::quiet_NaN());});
    bt::invalid([]{t::SurfaceSpec{.name="bad",.mu_peak=.5,.mu_slide=.6,.slip_stiffness_per_load=1.}.validate();});
    // Stage A SurfaceSpec + SurfaceMap blend_m validation
    const auto& asphalt = t::get_surface("asphalt");
    const auto& hardpack = t::get_surface("hardpack");
    const auto& loose = t::get_surface("loose");
    const auto& wet = t::get_surface("wet");
    const auto& scree = t::get_surface("scree");
    bt::require(asphalt.crr_delta == 0.0 && asphalt.load_sensitivity == 0.06, "asphalt stage-a fields");
    bt::require(hardpack.crr_delta == 0.0025 && hardpack.load_sensitivity == 0.08, "hardpack stage-a fields");
    bt::require(loose.crr_delta == 0.0060 && loose.load_sensitivity == 0.12, "loose stage-a fields");
    bt::require(wet.crr_delta == 0.0035 && wet.load_sensitivity == 0.12, "wet stage-a fields");
    bt::require(scree.crr_delta == 0.0080 && scree.load_sensitivity == 0.15, "scree stage-a fields");
    bt::invalid([] {
        t::SurfaceSpec{.name = "bad", .mu_peak = 0.8, .mu_slide = 0.6,
                       .slip_stiffness_per_load = 10.0, .crr_delta = -0.001}.validate();
    });
    bt::invalid([] {
        t::SurfaceSpec{.name = "bad", .mu_peak = 0.8, .mu_slide = 0.6,
                       .slip_stiffness_per_load = 10.0, .load_sensitivity = -0.01}.validate();
    });
    bt::invalid([] {
        t::SurfaceSpec{.name = "bad", .mu_peak = 0.8, .mu_slide = 0.6,
                       .slip_stiffness_per_load = 10.0, .load_sensitivity = 0.55}.validate();
    });
    t::SurfaceSpec{.name = "edge", .mu_peak = 0.8, .mu_slide = 0.6,
                   .slip_stiffness_per_load = 10.0, .crr_delta = 0.0,
                   .load_sensitivity = 0.5}.validate();
    // Adjacent touching sections with mismatched blend_m reject; equal blend_m accepts.
    bt::invalid([] {
        (void)t::SurfaceMap(t::get_surface("hardpack"),
            {{.start_m = 10.0, .end_m = 20.0, .surface = "loose", .blend_m = 0.4},
             {.start_m = 20.0, .end_m = 30.0, .surface = "scree", .blend_m = 0.8}});
    });
    (void)t::SurfaceMap(t::get_surface("hardpack"),
        {{.start_m = 0.0, .end_m = 0.5, .surface = "asphalt", .blend_m = 1.0},
         {.start_m = 0.5, .end_m = 12.0, .surface = "scree", .blend_m = 1.0}});
    bt::invalid([] {
        (void)t::SurfaceMap(t::get_surface("hardpack"),
            {{.start_m = 0.0, .end_m = 0.4, .surface = "asphalt", .blend_m = 1.0}});
    });
    bt::invalid([] {
        (void)t::SurfaceMap(t::get_surface("hardpack"),
            {{.start_m = 0.2, .end_m = 5.0, .surface = "asphalt", .blend_m = 0.6}});
    });
    bt::invalid([] {
        (void)t::SurfaceMap(t::get_surface("hardpack"),
            {{.start_m = 2.0, .end_m = 5.0, .surface = "asphalt", .blend_m = -0.1}});
    });
    // Separated gap smaller than 0.5 * (b_0 + b_1) rejects; exact boundary accepts.
    bt::invalid([] {
        (void)t::SurfaceMap(t::get_surface("hardpack"),
            {{.start_m = 2.0, .end_m = 5.0, .surface = "loose", .blend_m = 1.0},
             {.start_m = 5.8, .end_m = 10.0, .surface = "scree", .blend_m = 1.0}});
    });
    (void)t::SurfaceMap(t::get_surface("hardpack"),
        {{.start_m = 2.0, .end_m = 5.0, .surface = "loose", .blend_m = 1.0},
         {.start_m = 6.0, .end_m = 10.0, .surface = "scree", .blend_m = 1.0}});
    // Interior section shorter than 0.5 * b_left + 0.5 * b_right rejects.
    bt::invalid([] {
        (void)t::SurfaceMap(t::get_surface("hardpack"),
            {{.start_m = 2.0, .end_m = 2.8, .surface = "loose", .blend_m = 1.0}});
    });
    // Stage A alpine surface_blend_m bounds: 1.0 m feature clearance and
    // half-blend inside the lead-in; exact boundaries stay valid.
    bt::invalid([] {
        t::AlpineGenSpec s{};
        s.surface_blend_m = 1.5;
        s.validate();
    });
    bt::invalid([] {
        t::AlpineGenSpec s{};
        s.surface_blend_m = 1.0;
        s.lead_in_m = 0.4;
        s.validate();
    });
    t::AlpineGenSpec clearance_boundary{};
    clearance_boundary.surface_blend_m = 1.0;
    clearance_boundary.validate();
    t::AlpineGenSpec lead_in_boundary{};
    lead_in_boundary.surface_blend_m = 1.0;
    lead_in_boundary.lead_in_m = 0.5;
    lead_in_boundary.validate();
    t::ProfileQuery road({{0.,0.},{1.,0.},{2.,0.}});
    const auto contact=road.contact({.5,.49},.5);exact(contact.point[0],.5,"flat contact station");exact(contact.point[1],0.,"flat contact height");bt::require(contact.delta>0.&&!contact.multi_support,"single flat support");
    bt::require(road.contact({.5,2.},.5).delta<0.,"airborne exact signed gap");
    bt::invalid([&]{(void)road.contact({.5,0.},.5);});bt::invalid([&]{(void)road.contact({-1.,1.},.5);});bt::invalid([&]{(void)road.contact({.5,1.},.5,4);});
    bt::invalid([]{(void)t::ProfileQuery({{0.,0.},{0.,1.}});});
    bt::invalid([]{(void)t::ProfileQuery({{0.,0.},{1.,std::numeric_limits<double>::quiet_NaN()}});});
    bt::invalid([]{(void)t::validated_profile_contact({0.,0.},{0.,2.},.1,0,false);});
    t::SignedProfile const signed_road({{0.,0.},{1.,0.},{2.,1.}});
    const auto distance=signed_road.query(std::array<t::Point,3>{{{.5,.25},{.5,-.25},{1.,0.}}});
    exact(distance.distance[0],.25,"positive signed distance above road");exact(distance.distance[1],-.25,"negative signed distance inside road");bt::require(distance.nonsmooth[2],"true corner marked nonsmooth");
    bt::require(std::abs(t::min_bump_length_m(.10)-.9425)<1e-4,"contact safety source spike");
    bt::require(std::abs(t::max_bowl_depth_m(.8)-.07205)<1e-5,"bowl contact safety source spike");
    bt::require(t::concave_span_deg(t::RockGarden{.start_m=0.,.section_length_m=8.,.amplitude_m=.07,.correlation_length_m=.45,.seed="5"})>=20.,"source dirty rock seed detected");
    bt::require(t::concave_span_deg(t::RockGarden{.start_m=0.,.section_length_m=8.,.amplitude_m=.07,.correlation_length_m=.45,.seed="0"})<20.,"source clean rock seed accepted");
}
void step_up_and_iso8608(){
    // StepUp: a C1 cosine ramp that permanently lifts the datum, unlike the
    // two-sided SquareEdge block.
    const t::Obstacle step{.shape=t::StepUp{.start_m=5.,.height_m=.08,.ramp_m=.40}};
    bt::require(step.kind()==std::string_view("StepUp"),"step up reports its own kind");
    exact(step.length_m(),.40,"step up length is its ramp");
    bt::require(std::abs(step.end_m()-5.40)<1e-12,"step up end station");
    exact(step.datum_shift_m(),.08,"step up lifts the downstream datum");
    const auto ez=step.elevation(std::array{0.,.1,.2,.3,.4,.5,1.});
    exact(ez[0],0.,"step foot is flush with the approach");
    bt::require(std::abs(ez[1]-.011716)<1e-6,"step ramp quarter point");
    bt::require(std::abs(ez[2]-.04)<1e-12,"step ramp midpoint is half height");
    bt::require(std::abs(ez[3]-.068284)<1e-6,"step ramp three-quarter point");
    bt::require(std::abs(ez[4]-.08)<1e-12&&std::abs(ez[5]-.08)<1e-12&&std::abs(ez[6]-.08)<1e-12,"step deck holds past the ramp");
    bt::invalid([]{const t::Obstacle o{.shape=t::StepUp{.start_m=0.,.height_m=0.,.ramp_m=.4}};o.validate();});
    bt::invalid([]{const t::Obstacle o{.shape=t::StepUp{.start_m=0.,.height_m=.06,.ramp_m=0.}};o.validate();});
    bt::invalid([]{const t::Obstacle o{.shape=t::StepUp{.start_m=0.,.height_m=.06,.ramp_m=-.1}};o.validate();});
    auto track=t::flat(20.);track.obstacles={{.shape=t::StepUp{.start_m=5.,.height_m=.08,.ramp_m=.40}}};
    const auto pz=t::build_profile(track,std::array{4.99,5.,5.2,5.4,6.});
    exact(pz[0],0.,"approach before the step stays on datum");
    exact(pz[1],0.,"step foot is flush in the profile");
    bt::require(std::abs(pz[2]-.04)<1e-12,"profile midpoint of the ramp");
    bt::require(std::abs(pz[3]-.08)<1e-12&&std::abs(pz[4]-.08)<1e-12,"deck lifted by datum and stays lifted");
    exact(track.datum_shift_m(),.08,"track datum picks up the step lift");
    bt::require(track.markers().size()==1&&track.markers().front().second=="StepUp@5m","step marker label");
    // Curvature gate: ramp_m >= pi*sqrt(0.5*height*radius).
    auto gentle=t::flat(20.);gentle.obstacles={{.shape=t::StepUp{.start_m=5.,.height_m=.08,.ramp_m=.45}}};
    bt::require(t::single_contact_hazards(gentle,t::FRONT_WHEEL_RADIUS_M).empty(),"gentle step ramp passes the wheel curvature gate");
    auto sharp=t::flat(20.);sharp.obstacles={{.shape=t::StepUp{.start_m=5.,.height_m=.08,.ramp_m=.15}}};
    const auto hazards=t::single_contact_hazards(sharp,t::FRONT_WHEEL_RADIUS_M);
    bt::require(hazards.size()==1&&hazards.front().starts_with("StepUp@5m"),"sharp step ramp reports a hazard");
    // SteppedClimb curvature replaces the unconditional rejection.
    bt::require(t::single_contact_hazards(t::climb_steps()).empty(),"gentle stepped transitions pass the curvature gate");
    auto tight=t::flat(120.);tight.obstacles={{.shape=t::SteppedClimb{.start_m=15.,.steps={{5.,15.},{20.,15.}},.transition_m=.05}}};
    bt::require(!t::single_contact_hazards(tight).empty(),"tight stepped transition reports a hazard");
    auto abrupt=t::flat(120.);abrupt.obstacles={{.shape=t::SteppedClimb{.start_m=15.,.steps={{5.,15.},{10.,15.}},.transition_m=0.}}};
    bt::require(!t::single_contact_hazards(abrupt).empty(),"zero-length stepped transition reports a hazard");
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) x/z spans and lo/hi bounds mirror the sampled-profile call shape
double interior_rms(std::span<const double> x,std::span<const double> z,double lo,double hi){
    double sum=0.;std::size_t n=0;
    for(std::size_t i=0;i<x.size();++i)if(x[i]>=lo&&x[i]<=hi){sum+=z[i]*z[i];++n;}
    return n>0?std::sqrt(sum/static_cast<double>(n)):std::numeric_limits<double>::quiet_NaN();
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) x/z spans mirror the sampled-profile call shape
double psd_loglog_slope(std::span<const double> x,std::span<const double> z){
    // Hann-windowed direct DFT at log-spaced frequencies inside the excited
    // band; least-squares slope of log10(power) vs log10(f).
    const std::size_t n=z.size();double mean=0.;
    for(double const v:z)mean+=v;
    mean/=static_cast<double>(n);
    std::vector<double> w(n);for(std::size_t i=0;i<n;++i)w[i]=.5*(1.-std::cos(2.*std::numbers::pi*static_cast<double>(i)/static_cast<double>(n-1)));
    constexpr int bins=24;constexpr double f_lo=.3,f_hi=2.2;
    double sx=0.,sy=0.,sxx=0.,sxy=0.;
    for(int k=0;k<bins;++k){
        const double f=f_lo*std::pow(f_hi/f_lo,static_cast<double>(k)/static_cast<double>(bins-1));
        double re=0.,im=0.;
        for(std::size_t j=0;j<n;++j){const double a=-2.*std::numbers::pi*f*x[j];const double d=(z[j]-mean)*w[j];re+=d*std::cos(a);im+=d*std::sin(a);}
        const double lx=std::log10(f),ly=std::log10(re*re+im*im);
        sx+=lx;sy+=ly;sxx+=lx*lx;sxy+=lx*ly;
    }
    const double nf=static_cast<double>(bins);
    return (nf*sxy-sx*sy)/(nf*sxx-sx*sx);
}
void iso8608_roughness(){
    const t::RoadRoughness probe{.start_m=0.,.section_length_m=40.,.amplitude_m=.03,.correlation_length_m=.20,.seed="0",.spectrum="iso8608",.waviness=2.};
    bt::require(probe.spectrum=="iso8608"&&probe.waviness==2.,"iso8608 fields on the record");
    for(const double length:{20.,40.,80.}){
        const t::RoadRoughness rough{.start_m=0.,.section_length_m=length,.amplitude_m=.03,.correlation_length_m=.20,.seed="0",.spectrum="iso8608",.waviness=2.};
        const auto [x,z]=rough.nodes();
        bt::require(x.size()==z.size()&&x.size()>2,"iso8608 node grid");
        const double hi=std::min(35.,length-5.);
        const double rms=interior_rms(x,z,5.,hi);
        bt::require(std::abs(rms-.01)<=.0025,"iso8608 interior RMS is length invariant on a "+std::to_string(length)+" m section");
    }
    const t::RoadRoughness wide{.start_m=0.,.section_length_m=80.,.amplitude_m=.03,.correlation_length_m=.20,.seed="0",.spectrum="iso8608",.waviness=2.};
    const auto [x,z]=wide.nodes();
    const auto repeat=wide.nodes();
    bt::require(repeat.second==z,"iso8608 synthesis is seed deterministic");
    std::vector<double> xs,zs;for(std::size_t i=0;i<x.size();++i)if(x[i]>=5.&&x[i]<=35.){xs.push_back(x[i]);zs.push_back(z[i]);}
    const double slope=psd_loglog_slope(xs,zs);
    bt::require(slope>=-2.5&&slope<=-1.5,"iso8608 PSD log-log slope tracks waviness");
    const t::RoadRoughness narrow{.start_m=0.,.section_length_m=20.,.amplitude_m=.03,.correlation_length_m=.20,.seed="0",.spectrum="iso8608",.waviness=2.};
    const auto [nx,nz]=narrow.nodes();
    std::vector<double> nxs,nzs;for(std::size_t i=0;i<nx.size();++i)if(nx[i]>=5.&&nx[i]<=15.){nxs.push_back(nx[i]);nzs.push_back(nz[i]);}
    const double narrow_slope=psd_loglog_slope(nxs,nzs);
    bt::require(narrow_slope>=-2.5&&narrow_slope<=-1.5,"iso8608 PSD slope holds on a 20 m section");
    const t::RoadRoughness gauss{.start_m=0.,.section_length_m=10.,.amplitude_m=.003,.correlation_length_m=.2,.seed="0"};
    bt::require(gauss.spectrum=="gaussian"&&gauss.waviness==2.,"gaussian defaults preserved");
    // carve_roughness leaves 0.5-0.8 m iso8608 pieces around defects; with a
    // one- or two-sample interior a realized-RMS fit divides by a near-zero
    // draw, so the analytic normalization must keep every seed bounded. The
    // correlation_length 0.4 m cases also collapse the band floor onto the
    // Nyquist ceiling (n_min == n_max, dn == 0).
    const std::array<std::pair<double,double>,4> short_cases{{{.6,.20},{.8,.20},{.5,.40},{.6,.40}}};
    for(int seed=0;seed<20;++seed)for(const auto& [length,corr]:short_cases){
        const t::RoadRoughness piece{.start_m=0.,.section_length_m=length,.amplitude_m=.03,.correlation_length_m=corr,.seed=std::to_string(seed),.spectrum="iso8608",.waviness=2.};
        const auto [px,pz]=piece.nodes();
        bt::require(px.size()==pz.size()&&pz.size()>2,"short iso8608 piece keeps its node grid");
        double peak=0.;for(double const v:pz)peak=std::max(peak,std::abs(v));
        bt::require(peak<=2.*.03,"iso8608 "+std::to_string(length)+" m piece seed "+std::to_string(seed)+" stays within twice the amplitude");
    }
    bt::invalid([]{const t::RoadRoughness r{.section_length_m=1.,.amplitude_m=std::numeric_limits<double>::quiet_NaN(),.spectrum="iso8608"};(void)r.nodes();});
    bt::invalid([]{const t::Obstacle o{.shape=t::RoadRoughness{.spectrum="bogus"}};o.validate();});
    bt::invalid([]{const t::Obstacle o{.shape=t::RoadRoughness{.spectrum="iso8608",.waviness=.5}};o.validate();});
    bt::invalid([]{const t::Obstacle o{.shape=t::RoadRoughness{.waviness=4.5}};o.validate();});
    bt::invalid([]{const t::Obstacle o{.shape=t::RoadRoughness{.spectrum="iso8608",.waviness=.5}};(void)o.elevation(std::array{0.});});
}
void immutable_profiles(){
    for(const auto& name:t::available_presets()){
        const auto stem="tracks/preset_"+name;
        const auto expected=bt::read_f64(bt::fixture(stem+"_profile.f64"));bt::require(expected.size()%2==0,"oracle profile has X/Z pairs");
        std::vector<double>x;for(std::size_t i=0;i<expected.size();i+=2)x.push_back(expected[i]);
        const auto z=t::build_profile(t::get_preset(name),x);std::vector<double> actual;actual.reserve(expected.size());for(std::size_t i=0;i<x.size();++i){actual.push_back(x[i]);actual.push_back(z[i]);}
        // Capture_reference.py used vertical extent as X bounds. Retain that
        // historical fixture exactly; full longitudinal coverage is above.
        bt::golden_f64(stem+"_profile.f64",actual);
    }
}
}
int main(){return bt::run([]{profile_boundaries();grade_and_field();obstacles_and_wheelpath();materials_and_contacts();step_up_and_iso8608();iso8608_roughness();immutable_profiles();});}
