#include "terrain/generators.hpp"
#include "terrain/profile.hpp"
#include "terrain/spec_io.hpp"
#include <algorithm>
#include <cmath>

namespace bike::terrain {
void AlpineGenSpec::validate()const{
    const std::array<std::pair<std::string_view,Range>,20> ranges{{
        {"ramp_len_m",ramp_len_m},{"ramp_grade",ramp_grade},{"shelf_len_m",shelf_len_m},{"shelf_grade",shelf_grade},{"wall_grade",wall_grade},
        {"counter_len_m",counter_len_m},{"scree_len_m",scree_len_m},{"scree_amp_m",scree_amp_m},{"scree_corr_m",scree_corr_m},{"wet_len_m",wet_len_m},
        {"background_amp_m",background_amp_m},{"background_corr_m",background_corr_m},{"boulder_height_m",boulder_height_m},{"boulder_len_m",boulder_len_m},
        {"rock_len_m",rock_len_m},{"rock_amp_m",rock_amp_m},{"rock_corr_m",rock_corr_m},{"water_bar_depth_m",water_bar_depth_m},{"water_bar_len_m",water_bar_len_m},{"sharp_edge_height_m",sharp_edge_height_m}
    }};
    constexpr std::array<std::string_view,11> grid{"ramp_len_m","shelf_len_m","counter_len_m","scree_len_m","scree_corr_m","wet_len_m","background_corr_m","boulder_len_m","rock_len_m","rock_corr_m","water_bar_len_m"};
    for(const auto& [name,value]:ranges)detail::range(value,name,std::ranges::find(grid,name)==grid.end()?0.:.01);
    detail::range(counter_grade,"counter_grade",-1.);if(counter_grade[1]>=0.)throw std::invalid_argument("counter_grade must be negative: it is a short descent");
    detail::range(n_scree,"n_scree");detail::range(n_wet,"n_wet");detail::range(n_boulders,"n_boulders");detail::range(n_rock_gardens,"n_rock_gardens");detail::range(n_water_bars,"n_water_bars");detail::range(n_sharp_edges,"n_sharp_edges");
    detail::positive(length_m,"length_m");detail::positive(lead_in_m,"lead_in_m");detail::positive(lead_out_m,"lead_out_m");detail::positive(transition_m,"transition_m");detail::positive(min_contact_radius_m,"min_contact_radius_m");detail::positive(max_kink_deg,"max_kink_deg");detail::positive(contact_window_m,"contact_window_m");
    detail::nonnegative(counter_prob,"counter_prob");detail::nonnegative(scree_ramp_bias,"scree_ramp_bias");
    if(counter_prob>1.||scree_ramp_bias>1.)throw std::invalid_argument("alpine probabilities must lie in [0, 1]");
    if(n_walls<0||n_walls>1)throw std::invalid_argument("n_walls must be 0 or 1");
    if(wall_grade[0]<ramp_grade[1])throw std::invalid_argument("wall_grade must start at or above the top of ramp_grade");
    (void)get_surface(base_surface);if(transition_m<.01)throw std::invalid_argument("transition_m must be at least 0.01 m: segments are spaced on a centimetre grid");
    const double cursor=rounded(lead_in_m+transition_m,2),limit=rounded(rounded(length_m-lead_out_m,2)-transition_m,2);
    if(rounded(limit-cursor,2)<ramp_len_m[0])throw std::invalid_argument("the body must hold at least one shortest ramp between lead-in and lead-out");
    if(single_contact_safe&&n_sharp_edges[1]>0)throw std::invalid_argument("sharp edges always put the 2D tyre on two supports; set single_contact_safe = false to opt in");
}
AlpineGenSpec AlpineGenSpec::from_dict(const bike::io::Value& value){
    AlpineGenSpec out;for(const auto& [key,v]:bike::io::as_object(value,"AlpineGenSpec")){
        bool found=false;
#define READ_FIELD(name) if(key==#name){detail::read_field(out.name,v,key);found=true;}
        // X-macro taking the callback name, defined in terrain/spec_io.hpp;
        // cppcheck cannot expand this idiom.
        // cppcheck-suppress unknownMacro
        BIKE_ALPINE_SPEC_FIELDS(READ_FIELD)
#undef READ_FIELD
        if(!found)throw std::invalid_argument("unknown alpine generator key: "+key);
    }out.validate();return out;
}
bike::io::Value AlpineGenSpec::to_dict()const{
    bike::io::Value::Object out;
#define WRITE_FIELD(name) out.emplace_back(#name,detail::field_value(name));
    // X-macro taking the callback name, defined in terrain/spec_io.hpp;
    // cppcheck cannot expand this idiom.
    // cppcheck-suppress unknownMacro
    BIKE_ALPINE_SPEC_FIELDS(WRITE_FIELD)
#undef WRITE_FIELD
    return {std::move(out)};
}
std::string AlpineGenSpec::sha256()const{validate();return detail::source_spec_hash(to_dict());}
namespace {
struct Segment{std::string kind;double start,end,grade;};
std::vector<Segment> segments(const AlpineGenSpec& s,bike::random::NumpyRng& rng){
    const double body_end=rounded(s.length_m-s.lead_out_m,2),limit=rounded(body_end-s.transition_m,2);
    double cursor=rounded(s.lead_in_m+s.transition_m,2);std::vector<Segment> out;
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    const auto push=[&](std::string kind,double length,double grade){
        if(cursor>=limit)return false;
        const double end=rounded(std::min(cursor+length,limit),2);out.push_back({.kind=std::move(kind),.start=cursor,.end=end,.grade=grade});cursor=rounded(end+s.transition_m,2);return end<limit;
    };
    while(cursor<limit){
        const double length=rounded(rng.uniform(s.ramp_len_m[0],s.ramp_len_m[1]),2);
        const double grade=rounded(rng.uniform(s.ramp_grade[0],s.ramp_grade[1]));
        if(rounded(limit-cursor,2)<s.ramp_len_m[0]){out.back().end=limit;break;}
        if(!push("ramp",length,grade))break;
        const double shelf_length=rounded(rng.uniform(s.shelf_len_m[0],s.shelf_len_m[1]),2);
        const double shelf_grade=rounded(rng.uniform(s.shelf_grade[0],s.shelf_grade[1]));
        if(!push("shelf",shelf_length,shelf_grade))break;
        if(rng.uniform01()<s.counter_prob){
            const double counter_length=rounded(rng.uniform(s.counter_len_m[0],s.counter_len_m[1]),2);
            const double counter_grade=rounded(rng.uniform(s.counter_grade[0],s.counter_grade[1]));
            if(!push("counter",counter_length,counter_grade))break;
        }
    }
    if(s.n_walls!=0){std::vector<std::size_t> ramps;for(std::size_t i=0;i<out.size();++i)if(out[i].kind=="ramp")ramps.push_back(i);
        const auto index=ramps.at(static_cast<std::size_t>(rng.integers(0,static_cast<std::int64_t>(ramps.size()))));out[index].kind="wall";out[index].grade=rounded(rng.uniform(s.wall_grade[0],s.wall_grade[1]));}
    return out;
}
std::pair<GradeProfile,std::vector<Interval>> rhythm(const AlpineGenSpec& s,bike::random::NumpyRng& rng){
    const auto rows=segments(s,rng);GradeProfile grade{{{0.,0.},{s.lead_in_m,0.}}};std::vector<Interval> ramps;
    for(const auto& row:rows){grade.knots.push_back({row.start,row.grade});grade.knots.push_back({row.end,row.grade});if(row.kind=="ramp"||row.kind=="wall")ramps.emplace_back(row.start,row.end);}
    grade.knots.push_back({rounded(s.length_m-s.lead_out_m,2),0.});grade.knots.push_back({s.length_m,0.});grade.validate();return {std::move(grade),std::move(ramps)};
}
template<class T> Obstacle faceted(const AlpineGenSpec& s,bike::random::NumpyRng& rng,double start,double length,double amp,double corr){
    for(int redraw=0;redraw<25;++redraw){
        for(int attempt=0;attempt<PLACEMENT_TRIES;++attempt){
            const auto seed=std::to_string(rng.integers(0,2147483648LL));T o{start,length,amp,corr,seed};
            if(!s.single_contact_safe||concave_span_deg(o,s.contact_window_m)<s.max_kink_deg)return {std::move(o)};
        }amp=rounded(amp*.8);
    }
    throw std::invalid_argument("concave-span redraw exhausted at "+detail::gd::py_g(start)+" m: amplitude reached the grid floor without passing max_kink_deg");
}
}
TrackSpec generate_alpine_track(const AlpineGenSpec& s,std::string_view seed,std::optional<std::string> name){
    s.validate();const auto seed_text=detail::seed_plus(std::string(seed),0);bike::random::NumpyRng rng(seed_text,{});
    const double lo=s.lead_in_m,hi=rounded(s.length_m-s.lead_out_m,2);auto [grade,ramps]=rhythm(s,rng);
    std::vector<Interval> taken,used;std::vector<Obstacle> obstacles;std::vector<SurfaceSection> sections;
    const auto rock_count=detail::count_draw(rng,s.n_rock_gardens);
    for(std::int64_t i=0;i<rock_count;++i){
        const double wanted=rounded(rng.uniform(s.rock_len_m[0],s.rock_len_m[1]),2);
        const double amp=rounded(rng.uniform(s.rock_amp_m[0],s.rock_amp_m[1]));
        const double corr=rounded(rng.uniform(s.rock_corr_m[0],s.rock_corr_m[1]),3);
        const auto placed=place(rng,wanted,lo,hi,taken,true);if(placed){const auto [start,length]=*placed;obstacles.push_back(faceted<RockGarden>(s,rng,start,length,amp,corr));taken.emplace_back(start,start+length);}
    }
    const auto scree_count=detail::count_draw(rng,s.n_scree);
    for(std::int64_t i=0;i<scree_count;++i){
        const double wanted=rounded(rng.uniform(s.scree_len_m[0],s.scree_len_m[1]),2);
        const double amp=rounded(rng.uniform(s.scree_amp_m[0],s.scree_amp_m[1]));
        const double corr=rounded(rng.uniform(s.scree_corr_m[0],s.scree_corr_m[1]),3);
        Interval span{lo,hi};if(rng.uniform01()<s.scree_ramp_bias)span=ramps.at(static_cast<std::size_t>(rng.integers(0,static_cast<std::int64_t>(ramps.size()))));
        const auto placed=place(rng,wanted,span.first,span.second,taken,true);if(placed){
            const auto [start,length]=*placed;obstacles.push_back(faceted<RoadRoughness>(s,rng,start,length,amp,corr));sections.push_back({.start_m=start,.end_m=rounded(start+length,2),.surface="scree"});taken.emplace_back(start,start+length);used.emplace_back(start,start+length);
        }
    }
    const auto boulder_count=detail::count_draw(rng,s.n_boulders);
    for(std::int64_t i=0;i<boulder_count;++i){
        const double height=rounded(rng.uniform(s.boulder_height_m[0],s.boulder_height_m[1]));double length=rounded(rng.uniform(s.boulder_len_m[0],s.boulder_len_m[1]),2);
        if(s.single_contact_safe)length=std::max(length,rounded(std::ceil(min_bump_length_m(height,s.min_contact_radius_m)*100.)/100.,2));
        const auto placed=place(rng,length,lo,hi,taken);if(placed){const double start=placed->first;obstacles.push_back({.shape=Bump{.start_m=start,.height_m=height,.bump_length_m=length}});taken.emplace_back(start,start+length);}
    }
    const auto bar_count=detail::count_draw(rng,s.n_water_bars);
    for(std::int64_t i=0;i<bar_count;++i){
        double depth=rounded(rng.uniform(s.water_bar_depth_m[0],s.water_bar_depth_m[1]));const double length=rounded(rng.uniform(s.water_bar_len_m[0],s.water_bar_len_m[1]),2);
        if(s.single_contact_safe)depth=std::min(depth,rounded(max_bowl_depth_m(length,s.min_contact_radius_m)*.98));
        const auto placed=place(rng,length,lo,hi,taken);if(placed){const double start=placed->first;obstacles.push_back({.shape=BowlPothole{.start_m=start,.depth_m=depth,.hole_length_m=length}});taken.emplace_back(start,start+length);}
    }
    const auto edge_count=detail::count_draw(rng,s.n_sharp_edges);
    for(std::int64_t i=0;i<edge_count;++i){
        const double height=rounded(rng.uniform(s.sharp_edge_height_m[0],s.sharp_edge_height_m[1]));const double length=rounded(rng.uniform(.2,.6),2);
        const auto placed=place(rng,length,lo,hi,taken);if(placed){const double start=placed->first;obstacles.push_back({.shape=SquareEdge{.start_m=start,.height_m=height,.ledge_length_m=length}});taken.emplace_back(start,start+length);}
    }
    const auto wet_count=detail::count_draw(rng,s.n_wet);
    for(std::int64_t i=0;i<wet_count;++i){
        const double wanted=rounded(rng.uniform(s.wet_len_m[0],s.wet_len_m[1]),2);const auto placed=place(rng,wanted,lo,hi,used,true);
        if(placed){const auto [start,length]=*placed;sections.push_back({.start_m=start,.end_m=rounded(start+length,2),.surface="wet"});used.emplace_back(start,start+length);}
    }
    const double amp=rounded(rng.uniform(s.background_amp_m[0],s.background_amp_m[1]));
    const double corr=rounded(rng.uniform(s.background_corr_m[0],s.background_corr_m[1]),3);
    for(const auto& [a,b]:free_intervals({lo,hi},taken,0.))if(b-a>=4.*corr){
        double length=rounded(b-a,6);while(a+length>b)length=rounded(length-1e-6,6);obstacles.push_back(faceted<RoadRoughness>(s,rng,a,length,amp,corr));
    }
    std::ranges::stable_sort(sections,[](const auto&a,const auto&b){return a.start_m<b.start_m;});
    TrackSpec track{.name=(name&&!name->empty())?*name:"alpine_"+seed_text,.length_m=s.length_m,.obstacles=std::move(obstacles),.description="Alpine scree climb; seed="+seed_text+"; spec_sha256="+s.sha256(),.surface=s.base_surface,.grade_profile=std::move(grade),.surface_sections=std::move(sections)};
    track.validate();return track;
}
TrackSpec generate_alpine_track(const AlpineGenSpec& spec,std::uint64_t seed,std::optional<std::string>name){return generate_alpine_track(spec,std::to_string(seed),std::move(name));}
TrackSpec alpine_scree(std::string_view seed){const auto sd=detail::seed_plus(std::string(seed),0);return generate_alpine_track(AlpineGenSpec{},sd,sd=="41"?"alpine_scree":"alpine_scree_"+sd);}
TrackSpec alpine_rocky(std::string_view seed){
    // Exact contents of the pinned first-party alpine_rocky_genspec.toml.
    // The preset is owned native data, so it does not require the Python
    // source module's parents[3] path to exist in an installed application.
    AlpineGenSpec s;s.single_contact_safe=false;s.rock_amp_m={.06,.12};s.rock_corr_m={.15,.25};s.n_rock_gardens={2,3};
    s.scree_amp_m={.015,.030};s.scree_corr_m={.10,.15};s.boulder_height_m={.08,.20};s.boulder_len_m={.4,.9};
    s.water_bar_depth_m={.10,.18};s.water_bar_len_m={.6,1.};s.n_sharp_edges={1,2};s.sharp_edge_height_m={.04,.10};
    const auto sd=detail::seed_plus(std::string(seed),0);return generate_alpine_track(s,sd,sd=="0"?"alpine_rocky":"alpine_rocky_"+sd);
}
} // namespace bike::terrain
