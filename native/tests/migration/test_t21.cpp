// T21 source schemas, all generators/presets, exact authored source hashes,
// and immutable TOML fixtures. No fixture regeneration or skip-on-missing.
#include "support/migration_test.hpp"
#include "terrain/trackfile.hpp"
#include "io/hash.hpp"
#include "io/json.hpp"
#include <array>
#include <filesystem>
#include <string>

namespace {
namespace t=bike::terrain;namespace io=bike::io;
io::Value toml(std::string_view text){return t::parse_track_toml(text,"T21 authored input");}
std::string hash(std::string_view text){return io::sha256(std::as_bytes(std::span(text.data(),text.size())));}
void schemas(){
    const auto track=t::track_from_dict(toml("name='units'\nlength_m=20\n[[obstacles]]\ntype='pothole'\nedge='sloped'\nstart_m=2\ndepth_mm=80\nlength_m=0.5\nedge_mm=100\n"));
    const auto& hole=std::get<t::SlopedPothole>(track.obstacles[0].shape);bt::require(hole.depth_m==0.08&&hole.hole_length_m==0.5&&hole.edge_m==0.1,"millimetres and generic length aliases");
    const auto dumped=t::dump_track(track);bt::require(dumped.find("start_m = 2\n")!=std::string::npos,"authored integer field spelling retained");
    bt::require(t::dump_track(t::track_from_dict(toml(dumped)))==dumped,"materialized typed track round trip");
    for(const std::string body:{
        "type='hovercraft'\nstart_m=1\n", "type='__import__'\nstart_m=1\n", "type='pothole'\nedge='unknown'\nstart_m=1\n", "type='bump'\nshape='unknown'\nstart_m=1\n",
        "type='bump'\nstart_m=1\nheight_m=0.1\nheight_mm=100\n", "type='roots'\nstart_m=1\nseed=true\n", "type='washboard'\nstart_m=1\nn_waves=1.5\n", "type='drop'\nlength_m=1\nstart_m=1\n", "type='bump'\nheight_m=0.1\n"}){
        bt::invalid([&]{(void)t::track_from_dict(toml("name='bad'\nlength_m=30\n[[obstacles]]\n"+body));});
    }
    bt::invalid([]{(void)t::track_from_dict(toml("name='bad'\nlength_m=-1\n"));});
    bt::invalid([]{(void)t::track_from_dict(toml("name='bad'\nlength_m=30\n"),std::string("1"));});
    const auto zone=t::track_from_dict(toml("name='zones'\nlength_m=10\ngrade_profile={knots=[[0.0,0.0],[10.0,0.1]]}\nsurface_sections=[{start_m=2,end_m=3,surface='wet'}]\n"));
    bt::require(t::dump_track(t::track_from_dict(toml(t::dump_track(zone))))==t::dump_track(zone),"grade and material zones retained");
    const auto big=t::obstacle_from_dict(io::parse_json(R"({"type":"roots","start_m":1.0,"seed":184467440737095516170})"));
    bt::require(std::get<t::Roots>(big.shape).seed=="184467440737095516170","seed exceeds uint64 without precision loss");
    const auto climb=t::climb_steps();bt::invalid([&]{(void)t::dump_track(climb);});
    bt::require(io::canonical_json(t::identity_value(climb)).find("SteppedClimb")!=std::string::npos,"full native identity includes source nonserializable preset");
    const auto described=t::track_from_dict(toml("name='description'\nlength_m=5\ndescription={z=['text',true],a=2}\n"));
    bt::require(described.description=="{'z': ['text', True], 'a': 2}","source mapping order and Python description coercion");
}
void carving_and_placement(){
    const auto pieces=t::carve_roughness({{.shape=t::RoadRoughness{.start_m=1.0,.section_length_m=8.0,.amplitude_m=0.01,.correlation_length_m=0.2,.seed="99999999999999999999999"}},{.shape=t::SquareEdge{.start_m=4.0,.height_m=0.1,.ledge_length_m=1.}}});
    bt::require(pieces.size()==3,"roughness is split around hand placed defect");
    const auto& left=std::get<t::RoadRoughness>(pieces[0].shape);const auto& right=std::get<t::RoadRoughness>(pieces[2].shape);
    bt::require(left.start_m==1.&&left.section_length_m==3.&&right.start_m==5.&&right.section_length_m==4.0,"split geometry preserves exact neighbors");
    bt::require(right.seed=="100000000000000001000002","split seed addition is arbitrary precision");
    const std::array<t::Interval,2> taken{{{2.0,3.},{6.0,7.}}};const auto free=t::free_intervals({0.0,10.},taken,1.);
    bt::require(free==std::vector<t::Interval>{{0.0,1.},{4.0,5.},{8.0,10.}},"free intervals honor exact gap");
    bike::random::NumpyRng rng(0);bt::require(!t::place(rng,100.0,0.0,5.0,{},false),"impossible placement terminates");
    const auto shrink=t::place(rng,8.0,0.0,5.0,{},true);bt::require(shrink&&shrink->second==4.0,"shrinking placement retains source half-length rule");
    auto bad=t::road_worn_spec();bad.potholes_per_100m=10000.;bt::invalid([&]{(void)t::generate_road(bad,30.);});
    bad=t::road_worn_spec();bad.pothole_depth_m={0.2,0.1};bt::invalid([&]{bad.validate();});
    bad=t::road_worn_spec();bad.pothole_edge={{"sharp",0.}};bt::invalid([&]{bad.validate();});
}
void integer_seeds(){
    const std::array<std::pair<std::string_view,std::string_view>,7> spellings{{
        {"9223372036854775808","9223372036854775808"},
        {"184467440737095516170","184467440737095516170"},
        {"+18_446_744_073_709_551_6170","184467440737095516170"},
        {"0x1_0000_0000_0000_0000","18446744073709551616"},
        {"0o2_000_000_000_000_000_000_000","18446744073709551616"},
        {"0b1_00000000_00000000_00000000_00000000_00000000_00000000_00000000_00000000","18446744073709551616"},
        {"-0","0"}}};
    for(const auto& [token,expected]:spellings){
        const auto document=toml("name='seed tokens'\nlength_m=20\n[[obstacles]]\ntype='roots'\nstart_m=2\nseed="+std::string(token)+" # preserved exactly\n");
        const auto track=t::track_from_dict(document);
        bt::require(std::get<t::Roots>(track.obstacles[0].shape).seed==expected,"arbitrary TOML integer seed precision");
        const auto round=t::track_from_dict(toml(t::dump_track(track)));
        bt::require(std::get<t::Roots>(round.obstacles[0].shape).seed==expected,"arbitrary seed dump and read roundtrip");
    }
    const auto quoted=toml("name='quoted seed'\nlength_m=20\n# seed=99999999999999999999999999\ndescription='seed=88888888888888888888888888'\nobstacles=[{type='roots',start_m=2,\"s\\u0065ed\"=18446744073709551617}]\n");
    bt::require(std::get<t::Roots>(t::track_from_dict(quoted).obstacles[0].shape).seed=="18446744073709551617","quoted escaped seed key, inline table, and comment/string lookalikes");
    for(const auto token:{"-1","1.0","true","'1'","01","1_","0x_1","+0x1"}){
        bt::invalid([&]{(void)t::track_from_dict(toml("name='bad seed'\nlength_m=20\n[[obstacles]]\ntype='roots'\nstart_m=2\nseed="+std::string(token)+"\n"));});
    }
    bt::invalid([]{(void)toml("seed=18446744073709551617\nseed=18446744073709551618\n");});
    bt::invalid([]{(void)toml("length_m=18446744073709551617\n");});
    const auto overridden=t::track_from_dict(toml("name='overridden seed'\nlength_m=30\n[generator]\nseed=-1\npotholes_per_100m=0\nbumps_per_100m=0\nroughness_m=0\n"),std::string("3"));
    bt::require(overridden.obstacles.empty(),"override replaces invalid original seed before generator validation");
}
void generators(){
    // Exact source assertions from test_terrain_placement_regression.py.
    bt::require(t::TerrainGenSpec{}.sha256()=="45d11d386fb651dff55d7af3292ea3e2a972d9354e1bef677227f2399bfcf8e7","default terrain spec hash");
    const std::array<std::string_view,4> hashes{"b298d8170a8391daf50b671f41b9dc7e33855354fe770d3cb894af70400f544d","7ac544f21b89f27a03a176e5d017e0a0e175af8ec80119601dd05a652133848a","4837293048585ed234201595833d3e8ed7851b6c6e8a384ab41745840c7f5622","be8c67f11a7961b2bcb012612b6d16ed0b72394edcae09f9b12fc0e1b73857c0"};
    for(std::size_t seed=0;seed<hashes.size();++seed)bt::require(hash(t::dump_track(t::generate_track(t::TerrainGenSpec{},static_cast<std::uint64_t>(seed))))==hashes[seed],"source fixed-seed terrain bytes");
    bt::require(t::AlpineGenSpec::from_dict(toml("length_m=120\n")).sha256()==t::AlpineGenSpec{}.sha256(),"alpine scalar int/float normalization");
    bt::require(t::AlpineGenSpec::from_dict(t::AlpineGenSpec{}.to_dict()).sha256()==t::AlpineGenSpec{}.sha256(),"complete alpine spec roundtrip");
    for(const std::string input:{"ramp_len_m=[0,15]\n","transition_m=0.001\n","counter_prob=2\n","n_walls=2\n","n_boulders=[1.5,2]\n","base_surface='ice'\n","n_sharp_edges=[1,1]\n","single_contact_safe='yes'\n","unknown=1\n"})bt::invalid([&]{(void)t::AlpineGenSpec::from_dict(toml(input));});
    bt::invalid([]{(void)t::TerrainGenSpec::from_dict(toml("length_m=[1,2]\n"));});
    bt::invalid([]{(void)t::TerrainGenSpec::from_dict(toml("n_features=[1.5,2]\n"));});
    bt::invalid([]{(void)t::TerrainGenSpec::from_dict(toml("feature_types=['unknown']\n"));});
    bt::invalid([]{(void)t::generate_track(t::TerrainGenSpec{},std::string_view("-1"));});
    for(const auto name:{"flat","uphill","rough_uphill","crest","low_grip","step_up"}){
        const auto track=t::build_research_track(name,"7");track.validate();bt::require(t::dump_track(t::track_from_dict(toml(t::dump_track(track))))==t::dump_track(track),"research scenario roundtrip");
    }
    const auto safe=t::alpine_scree();bt::require(t::single_contact_hazards(safe).empty(),"default alpine obeys one-support geometry");
    bt::require(!t::single_contact_hazards(t::alpine_rocky()).empty(),"rocky preset preserves multi-contact opt-in");
}
void examples_and_oracles(){
    auto root=bt::fixture("tracks/flat.toml");for(int i=0;i<5;++i)root=root.parent_path();
    for(const auto name:{"rough_uphill","rough_uphill_extreme","rough_uphill_savage","alpine_scree","alpine_rocky"}){
        const auto track=t::load_track(root/"examples"/"research"/(std::string(name)+".toml"));
        bt::require(t::dump_track(t::track_from_dict(toml(t::dump_track(track))))==t::dump_track(track),"all authored example tracks roundtrip");
    }
    bt::manifest_verify_files(bt::manifest_entry("tracks/rough_uphill_savage.toml"));bt::manifest_verify_files(bt::manifest_entry("tracks/rough_uphill_savage.canonical.toml"));
    bt::golden_text("tracks/rough_uphill_savage.canonical.toml",t::dump_track(t::load_track(bt::fixture("tracks/rough_uphill_savage.toml"))));
    bt::require(t::available_presets().size()==10,"all ten source presets present");
    for(const auto& name:t::available_presets()){
        const auto stem="tracks/preset_"+name;bt::manifest_verify_files(bt::manifest_entry(stem));const auto track=t::get_preset(name);
        if(name=="climb_steps"){bt::invalid([&]{(void)t::track_to_dict(track);});continue;}
        bt::golden_text(stem+".toml",t::dump_track(track));
        const auto expected=io::parse_json(bt::text(bt::fixture(stem+".json")));bt::require(io::canonical_json(t::track_to_dict(track))==io::canonical_json(expected),"preset semantic fixture");
    }
    bt::invalid([]{(void)t::load_track(bt::fixture("tracks/unknown_obstacle.toml"));});bt::invalid([]{(void)t::load_track(bt::fixture("tracks/degenerate.toml"));});
}
}
int main(){return bt::run([]{schemas();carving_and_placement();integer_seeds();generators();examples_and_oracles();});}
