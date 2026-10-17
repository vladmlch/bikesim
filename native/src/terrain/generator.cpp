#include "terrain/generators.hpp"
#include "terrain/profile.hpp"
#include "terrain/spec_io.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace bike::terrain {
void TerrainGenSpec::validate()const{
    detail::range(length_m,"length_m");detail::range(roughness_amp_m,"roughness_amp_m");detail::range(roughness_len_m,"roughness_len_m");detail::range(roughness_corr_m,"roughness_corr_m");
    detail::range(bump_height_m,"bump_height_m");detail::range(bump_length_m,"bump_length_m");detail::range(edge_height_m,"edge_height_m");detail::range(drop_height_m,"drop_height_m");detail::range(roots_len_m,"roots_len_m");detail::range(section_len_m,"section_len_m");
    detail::range(n_roughness,"n_roughness");detail::range(n_features,"n_features");detail::range(n_surface_sections,"n_surface_sections");
    detail::nonnegative(grade_max,"grade_max");detail::nonnegative(grade_peak,"grade_peak");detail::nonnegative(lead_in_m,"lead_in_m");detail::nonnegative(lead_out_m,"lead_out_m");detail::nonnegative(grade_peak_prob,"grade_peak_prob");
    if(grade_peak<grade_max)throw std::invalid_argument("grade_peak must not be below grade_max");
    if(grade_peak_prob>1.)throw std::invalid_argument("grade_peak_prob must lie in [0, 1]");
    if(base_surfaces.empty()||section_surfaces.empty()||feature_types.empty())throw std::invalid_argument("surface/feature selections must be nonempty");
    for(const auto& v:base_surfaces)(void)get_surface(v);for(const auto& v:section_surfaces)(void)get_surface(v);
    for(const auto& v:feature_types)if(v!="bump"&&v!="edge"&&v!="drop"&&v!="roots")throw std::invalid_argument("unknown terrain feature type: "+v);
    if(length_m[0]<lead_in_m+lead_out_m+2.*CLEARANCE_M)throw std::invalid_argument("shortest track must leave a usable body after lead-in/out");
}
TerrainGenSpec TerrainGenSpec::from_dict(const bike::io::Value& value){
    TerrainGenSpec out;
    for(const auto& [key,v]:bike::io::as_object(value,"TerrainGenSpec")){
        bool found=false;
#define READ_FIELD(name) if(key==#name){detail::read_field(out.name,v,key);found=true;}
        BIKE_TERRAIN_SPEC_FIELDS(READ_FIELD)
#undef READ_FIELD
        if(!found)throw std::invalid_argument("unknown terrain generator key: "+key);
        if((key=="grade_max"||key=="grade_peak"||key=="grade_peak_prob"||key=="lead_in_m"||key=="lead_out_m")&&bike::io::is_integer_lexeme(bike::io::as_number(v,key).lexeme))out.integer_scalar_fields.push_back(key);
    }out.validate();return out;
}
bike::io::Value TerrainGenSpec::to_dict()const{
    bike::io::Value::Object out;
#define WRITE_FIELD(name) out.emplace_back(#name,detail::field_value(name));
    BIKE_TERRAIN_SPEC_FIELDS(WRITE_FIELD)
#undef WRITE_FIELD
    for(auto& [key,value]:out)if(std::ranges::find(integer_scalar_fields,key)!=integer_scalar_fields.end()){
        const double v=bike::io::as_double(value,key);if(std::trunc(v)==v)value={bike::io::Number{detail::gd::py_fixed(v,0)}};
    }return {std::move(out)};
}
std::string TerrainGenSpec::sha256()const{validate();return detail::source_spec_hash(to_dict());}
namespace {
GradeProfile grade_for(const TerrainGenSpec& s,bike::random::NumpyRng& rng,double length){
    const auto n=rng.integers(3,6);const double a=s.lead_in_m,b=length-s.lead_out_m,width=(b-a)/static_cast<double>(n+1);
    std::vector<double> stations,grades;stations.reserve(static_cast<std::size_t>(n));grades.reserve(static_cast<std::size_t>(n));
    for(std::int64_t i=0;i<n;++i)stations.push_back(rounded(a+static_cast<double>(i+1)*width+(rng.uniform01()-.5)*.6*width,2));
    for(std::int64_t i=0;i<n;++i)grades.push_back(rounded(rng.uniform(0.,s.grade_max)));
    if(rng.uniform01()<s.grade_peak_prob){const auto i=static_cast<std::size_t>(rng.integers(0,n-1));grades[i]=rounded(rng.uniform(s.grade_max,s.grade_peak));grades[i+1]=rounded(rng.uniform(s.grade_max,s.grade_peak));}
    GradeProfile grade{{{0.,0.},{s.lead_in_m,0.}}};for(std::size_t i=0;i<stations.size();++i)grade.knots.push_back({stations[i],grades[i]});grade.knots.push_back({rounded(b,2),0.});grade.knots.push_back({length,0.});grade.validate();return grade;
}
std::optional<Obstacle> feature(std::string_view kind,bike::random::NumpyRng& rng,const TerrainGenSpec& s,const GradeProfile& grade,std::span<const Interval> taken,double lo,double hi){
    double size=0.,height=0.;
    if(kind=="bump"){
        size=rounded(rng.uniform(s.bump_length_m[0],s.bump_length_m[1]),2);height=rounded(rng.uniform(s.bump_height_m[0],s.bump_height_m[1]));
        size=std::max(size,rounded(std::ceil(std::numbers::pi*std::sqrt(2.*height*.45)*100.)/100.,2));
    }else if(kind=="edge"){size=rounded(rng.uniform(.2,.6),2);height=rounded(rng.uniform(s.edge_height_m[0],s.edge_height_m[1]));}
    else if(kind=="drop"){height=rounded(rng.uniform(s.drop_height_m[0],s.drop_height_m[1]));}
    else{size=rounded(rng.uniform(s.roots_len_m[0],s.roots_len_m[1]),2);height=rounded(rng.uniform(s.bump_height_m[0],std::min(s.bump_height_m[1],.08)));}
    const auto placed=place(rng,std::max(size,.01),lo,hi,taken);if(!placed)return std::nullopt;const double start=placed->first;
    if(kind=="drop"&&grade.slope(start)<0.)return std::nullopt;
    if(kind=="bump")return Obstacle{Bump{start,height,size}};
    if(kind=="edge")return Obstacle{SquareEdge{start,height,size}};
    if(kind=="drop")return Obstacle{Drop{start,height,0.}};
    const auto n=rng.integers(4,9);const auto seed=std::to_string(rng.integers(0,2147483648LL));
    Roots roots;roots.start_m=start;roots.n_bumps=n;roots.height_m=height;roots.section_length_m=size;roots.seed=seed;return Obstacle{std::move(roots)};
}
}
TrackSpec generate_track(const TerrainGenSpec& s,std::string_view seed,std::optional<std::string> name){
    s.validate();const auto seed_text=detail::seed_plus(std::string(seed),0);bike::random::NumpyRng rng(seed_text,{});
    const double length=rounded(rng.uniform(s.length_m[0],s.length_m[1]),1),lo=s.lead_in_m,hi=length-s.lead_out_m;
    auto grade=grade_for(s,rng,length);std::vector<Interval> taken;std::vector<Obstacle> obstacles;
    const auto rough_count=detail::count_draw(rng,s.n_roughness);
    for(std::int64_t i=0;i<rough_count;++i){
        const double wanted=rounded(rng.uniform(s.roughness_len_m[0],s.roughness_len_m[1]),2);
        const double amp=rounded(rng.uniform(s.roughness_amp_m[0],s.roughness_amp_m[1]));
        const double corr=rounded(rng.uniform(s.roughness_corr_m[0],s.roughness_corr_m[1]),3);
        const auto rough_seed=std::to_string(rng.integers(0,2147483648LL));const auto placed=place(rng,wanted,lo,hi,taken,true);
        if(placed){const auto [start,size]=*placed;obstacles.push_back({RoadRoughness{start,size,amp,corr,rough_seed}});taken.emplace_back(start,start+size);}
    }
    const auto feature_count=detail::count_draw(rng,s.n_features);
    for(std::int64_t i=0;i<feature_count;++i){
        const auto& kind=s.feature_types.at(static_cast<std::size_t>(rng.integers(0,static_cast<std::int64_t>(s.feature_types.size()))));
        auto o=feature(kind,rng,s,grade,taken,lo,hi);if(o){taken.emplace_back(o->start_m(),o->end_m());obstacles.push_back(std::move(*o));}
    }
    const auto surface=s.base_surfaces.at(static_cast<std::size_t>(rng.integers(0,static_cast<std::int64_t>(s.base_surfaces.size()))));
    std::vector<SurfaceSection> sections;std::vector<Interval> used;const auto section_count=detail::count_draw(rng,s.n_surface_sections);
    for(std::int64_t i=0;i<section_count;++i){
        const double wanted=rounded(rng.uniform(s.section_len_m[0],s.section_len_m[1]),2);
        const auto material=s.section_surfaces.at(static_cast<std::size_t>(rng.integers(0,static_cast<std::int64_t>(s.section_surfaces.size()))));
        const auto placed=place(rng,wanted,lo,hi,used,true);if(placed){const auto [start,size]=*placed;sections.push_back({start,rounded(start+size,2),material});used.emplace_back(start,start+size);}
    }
    std::stable_sort(sections.begin(),sections.end(),[](const auto&a,const auto&b){return a.start_m<b.start_m;});
    TrackSpec track{(name&&!name->empty())?*name:"generated_"+seed_text,length,std::move(obstacles),"Generated rough climb; seed="+seed_text+"; spec_sha256="+s.sha256(),surface,std::move(grade),std::move(sections)};
    track.validate();return track;
}
TrackSpec generate_track(const TerrainGenSpec& spec,std::uint64_t seed,std::optional<std::string> name){return generate_track(spec,std::to_string(seed),std::move(name));}
TrackSpec generate(const bike::io::Value& spec,std::uint64_t seed){return generate_track(TerrainGenSpec::from_dict(spec),seed);}
TrackSpec build_research_track(std::string_view name,std::string_view seed){
    if(name!="flat"&&name!="uphill"&&name!="rough_uphill"&&name!="crest"&&name!="low_grip"&&name!="step_up")throw std::invalid_argument("unknown research scenario: "+std::string(name));
    const auto sd=detail::seed_plus(std::string(seed),0);
    TrackSpec track{"research_"+std::string(name),35.,{},"Synthetic longitudinal torque-control challenge; seed="+sd,"asphalt",std::nullopt,{}};
    if(name=="uphill"||name=="rough_uphill"||name=="low_grip"){
        const double grade=name=="low_grip"?.12:.22;track.grade_profile=GradeProfile{{{0.,0.},{4.,0.},{8.,grade},{24.,grade},{30.,0.},{35.,0.}}};
    }
    if(name=="rough_uphill"){
        track.surface="hardpack";track.obstacles={{RoadRoughness{4.,4.,.015,.35,sd}},{Bump{8.5,.06,.7}},{RoadRoughness{10.,5.,.025,.45,detail::seed_plus(sd,1)}},{SquareEdge{16.,.04,.25}},{RoadRoughness{18.,11.,.018,.3,detail::seed_plus(sd,2)}}};
    }else if(name=="crest"){
        track.grade_profile=GradeProfile{{{0.,0.},{4.,0.},{6.,.28},{8.,.28},{10.,0.},{12.,-.35},{16.,-.35},{20.,0.},{35.,0.}}};track.obstacles={{RoadRoughness{5.,3.,.008,.4,sd}}};
    }else if(name=="low_grip"){
        track.surface_sections={{6.,10.,"wet"},{14.,21.,"loose"}};track.obstacles={{RoadRoughness{4.,23.,.008,.4,sd}}};
    }else if(name=="step_up")track.obstacles={{SquareEdge{5.,.07,.5}},{Bump{9.,.10,1.}}};
    track.validate();return track;
}
} // namespace bike::terrain
