// T20 regressions. Authored without executing a project binary. B comparisons
// consume immutable oracle files; missing binary payloads fail, never skip.
#include "support/migration_test.hpp"
#include "terrain/profile.hpp"
#include "terrain/generators.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace {
namespace t=bike::terrain;
void exact(double actual,double expected,std::string_view label){bt::require(std::bit_cast<std::uint64_t>(actual)==std::bit_cast<std::uint64_t>(expected),label);}
void profile_boundaries(){
    auto track=t::flat(10.);track.obstacles={{t::SquareEdge{2.,.125,.5}},{t::Drop{4.,.25,0.}},{t::Pothole{6.,.125,.5}}};
    const std::array x{-1.,0.,2.,std::nextafter(2.5,0.),2.5,4.,6.,6.5,10.,15.};
    const std::array expected{0.,0.,.125,.125,0.,-.25,-.375,-.25,-.25,-.25};
    const auto z=t::build_profile(track,x);for(std::size_t i=0;i<z.size();++i)exact(z[i],expected[i],"half-open obstacle and permanent datum boundaries");
    exact(track.datum_shift_m(),-.25,"net datum shift");bt::require(track.markers().size()==3,"feature markers");
    track.obstacles.push_back({t::SquareEdge{2.25,.125,.5}});bt::invalid([&]{track.validate();});
    auto duplicate=t::flat(10.);duplicate.obstacles={{t::Drop{2.,.25,0.}},{t::Drop{2.,.5,0.}}};
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
    exact(t::Obstacle{t::Bump{0.,.125,1.}}.elevation(middle)[0],.125,"raised cosine crest");
    exact(t::Obstacle{t::BowlPothole{0.,.125,1.}}.elevation(middle)[0],-.125,"bowl trough");
    exact(t::Obstacle{t::SlopedPothole{0.,.125,1.,1.}}.elevation(middle)[0],-.125,"overlapping chamfers form a V");
    exact(t::Obstacle{t::TrapezoidBump{0.,.125,.5,0.}}.elevation(middle)[0],.125,"zero-plateau triangular bump");
    const t::Obstacle kicker{t::Kicker{0.,.25,1.,1.,0.,1.}};
    const auto ramp=kicker.elevation(std::array{.5,1.,2.});exact(ramp[0],.125,"kicker takeoff ramp");exact(ramp[1],0.,"kicker lip steps down");
    const t::Obstacle roots{t::Roots{0.,7,.08,.25,7.,.35,"2029"}};
    const std::array coarse{0.,1.,2.,3.,4.,5.,6.,7.};const auto a=roots.elevation(coarse);
    const std::array fine{0.,.5,1.,1.5,2.,2.5,3.,3.5,4.,4.5,5.,5.5,6.,6.5,7.};const auto b=roots.elevation(fine);
    for(std::size_t i=0;i<a.size();++i)exact(a[i],b[i*2],"root shape is independent of sample grid");
    const t::RockGarden garden{0.,8.,.06,.25,"1701"};const auto [nodes,heights]=garden.nodes();
    bt::require(nodes.size()==65&&heights.size()==65,"correlation sets internal rock node grid");bt::require(heights.front()==0.&&heights.back()==0.,"rock entry and exit taper, allowing the source signed zero");
    double peak=0.;for(double v:heights)peak=std::max(peak,std::abs(v));bt::require(std::abs(peak-.06)<1e-15,"rock amplitude normalization");
    const auto climb=t::climb_steps();exact(climb.obstacles[0].length_m(),90.,"all stepped climb transitions included");exact(climb.datum_shift_m(),13.125,"stepped climb integral");
    const auto climb_z=t::build_profile(climb,std::array{0.,15.,105.,115.});exact(climb_z[0],0.,"stepped climb leadin");exact(climb_z[2],climb.datum_shift_m(),"stepped climb downstream datum");
    const t::Obstacle hole{t::Pothole{0.,.25,.6}};
    const double drop=t::effective_drop_m(hole,.5,.0005);
    bt::require(std::abs(drop-.1)<.001,"rigid wheel bridges the rectangular hole at the analytic sag, within the documented 0.5 mm grid bound");
    bt::require(std::isinf(t::bridged_drop_m(1.,.5)),"diameter-wide hole cannot be bridged");
    bt::require(std::abs(t::effective_rise_m(t::Obstacle{t::Bump{0.,.125,1.}},.5,.001)-.125)<.001,"wheel rises by the bump height");
}
void materials_and_contacts(){
    bt::require(t::available_surfaces()==std::vector<std::string>{"asphalt","hardpack","loose","wet","scree"},"surface registry order");
    const t::SurfaceMap map(t::get_surface("hardpack"),{{2.,3.,"wet"},{4.,5.,"scree"}});
    bt::require(map.at(-1.).name=="hardpack"&&map.at(2.).name=="wet"&&map.at(3.).name=="hardpack"&&map.at(5.).name=="hardpack","half-open surface sections and default outside map");
    bt::require(map.name()=="hardpack+zones"&&map.surfaces().size()==3,"surface map identity and inventory");
    exact(t::get_surface("scree").mu(0.),.45,"scree peak friction");
    exact(t::slip_stiffness_n(t::get_surface("scree"),100.),450.,"slip stiffness per normal load");
    exact(t::slip_stiffness_n(t::get_surface("scree"),std::numeric_limits<double>::infinity()),0.,"nonfinite load sentinel remains local");
    bt::invalid([]{(void)t::SurfaceMap(t::get_surface("hardpack"),{{1.,3.,"wet"},{2.,4.,"loose"}});});
    bt::invalid([&]{(void)map.at(std::numeric_limits<double>::quiet_NaN());});
    bt::invalid([]{t::SurfaceSpec{"bad",.5,.6,1.}.validate();});
    t::ProfileQuery road({{0.,0.},{1.,0.},{2.,0.}});
    const auto contact=road.contact({.5,.49},.5);exact(contact.point[0],.5,"flat contact station");exact(contact.point[1],0.,"flat contact height");bt::require(contact.delta>0.&&!contact.multi_support,"single flat support");
    bt::require(road.contact({.5,2.},.5).delta<0.,"airborne exact signed gap");
    bt::invalid([&]{(void)road.contact({.5,0.},.5);});bt::invalid([&]{(void)road.contact({-1.,1.},.5);});bt::invalid([&]{(void)road.contact({.5,1.},.5,4);});
    bt::invalid([]{(void)t::ProfileQuery({{0.,0.},{0.,1.}});});
    bt::invalid([]{(void)t::ProfileQuery({{0.,0.},{1.,std::numeric_limits<double>::quiet_NaN()}});});
    bt::invalid([]{(void)t::validated_profile_contact({0.,0.},{0.,2.},.1,0,false);});
    t::SignedProfile signed_road({{0.,0.},{1.,0.},{2.,1.}});
    const auto distance=signed_road.query(std::array<t::Point,3>{{{.5,.25},{.5,-.25},{1.,0.}}});
    exact(distance.distance[0],.25,"positive signed distance above road");exact(distance.distance[1],-.25,"negative signed distance inside road");bt::require(distance.nonsmooth[2],"true corner marked nonsmooth");
    bt::require(std::abs(t::min_bump_length_m(.10)-.9425)<1e-4,"contact safety source spike");
    bt::require(std::abs(t::max_bowl_depth_m(.8)-.07205)<1e-5,"bowl contact safety source spike");
    bt::require(t::concave_span_deg(t::RockGarden{0.,8.,.07,.45,"5"})>=20.,"source dirty rock seed detected");
    bt::require(t::concave_span_deg(t::RockGarden{0.,8.,.07,.45,"0"})<20.,"source clean rock seed accepted");
}
void immutable_profiles(){
    for(const auto& name:t::available_presets()){
        const auto stem="tracks/preset_"+name;bt::manifest_verify_files(bt::manifest_entry(stem));
        const auto expected=bt::read_f64(bt::fixture(stem+"_profile.f64"));bt::require(expected.size()%2==0,"oracle profile has X/Z pairs");
        std::vector<double>x;for(std::size_t i=0;i<expected.size();i+=2)x.push_back(expected[i]);
        const auto z=t::build_profile(t::get_preset(name),x);std::vector<double> actual;actual.reserve(expected.size());for(std::size_t i=0;i<x.size();++i){actual.push_back(x[i]);actual.push_back(z[i]);}
        // Capture_reference.py used vertical extent as X bounds. Retain that
        // historical fixture exactly; full longitudinal coverage is above.
        bt::golden_f64(stem+"_profile.f64",actual);
    }
}
}
int main(){return bt::run([]{profile_boundaries();grade_and_field();obstacles_and_wheelpath();materials_and_contacts();immutable_profiles();});}
