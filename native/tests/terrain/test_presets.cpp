// T21 source schemas, all generators/presets, exact authored source hashes,
// and immutable TOML fixtures. No fixture regeneration or skip-on-missing.
#include "support/fixture_test.hpp"
#include "terrain/trackfile.hpp"
#include "io/hash.hpp"
#include "io/json.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

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
    bt::require(t::dump_track(zone).find("blend_m")==std::string::npos&&io::canonical_json(t::identity_value(zone)).find("blend_m")==std::string::npos,"zero blend_m omitted from TOML and track identity");
    const auto blended=t::track_from_dict(toml("name='blended'\nlength_m=20\nsurface_sections=[{start_m=0.0,end_m=5.0,surface='asphalt',blend_m=0.4},{start_m=5.0,end_m=12.0,surface='scree',blend_m=0.4}]\n"));
    bt::require(blended.surface_sections.size()==2&&blended.surface_sections[0].blend_m==0.4&&blended.surface_sections[1].blend_m==0.4,"trackfile parses optional blend_m");
    bt::require(t::dump_track(blended).find("blend_m = 0.4")!=std::string::npos&&t::dump_track(t::track_from_dict(toml(t::dump_track(blended))))==t::dump_track(blended),"nonzero blend_m roundtrips through TOML");
    bt::require(io::canonical_json(t::identity_value(blended)).find("\"blend_m\":0.4")!=std::string::npos,"nonzero blend_m enters track identity");
    {
        bool caught_unknown=false;
        try{(void)t::track_from_dict(toml("name='bad'\nlength_m=10\nsurface_sections=[{start_m=2,end_m=4,surface='wet',extra=1}]\n"));}
        catch(const t::TrackFileError&){caught_unknown=true;}
        bt::require(caught_unknown,"unknown surface section key raises TrackFileError");
    }
    {
        bool caught_missing=false;
        try{(void)t::track_from_dict(toml("name='bad'\nlength_m=10\nsurface_sections=[{start_m=2,end_m=4,blend_m=0.2}]\n"));}
        catch(const t::TrackFileError&){caught_missing=true;}
        bt::require(caught_missing,"missing surface section required key raises TrackFileError");
    }
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
    // An iso8608 roughness section carves around a defect like any other;
    // the spectral fields propagate to the pieces and the sub-metre tail
    // piece stays bounded under the analytic normalization.
    const auto spectral_carve=t::track_from_dict(toml("name='iso carve'\nlength_m=20\n[[obstacles]]\ntype='roughness'\nstart_m=2\nsection_length_m=10\namplitude_m=0.03\ncorrelation_length_m=0.2\nspectrum='iso8608'\nwaviness=2.2\nseed=7\n[[obstacles]]\ntype='square_edge'\nstart_m=11\nheight_m=0.1\nledge_length_m=0.5\n"));
    std::vector<const t::RoadRoughness*> iso_pieces;
    for(const auto& o:spectral_carve.obstacles)if(const auto* rr=std::get_if<t::RoadRoughness>(&o.shape))iso_pieces.push_back(rr);
    bt::require(iso_pieces.size()==2,"iso8608 roughness splits around the defect");
    const t::RoadRoughness* tail=nullptr;
    for(const auto* rr:iso_pieces){
        bt::require(rr->spectrum=="iso8608"&&rr->waviness==2.2,"carved pieces keep the spectral fields");
        if(rr->section_length_m<1.)tail=rr;
    }
    bt::require(tail&&tail->start_m==11.5&&tail->section_length_m==.5,"defect leaves a 0.5 m iso8608 tail piece");
    const auto [tx,tz]=tail->nodes();
    bt::require(tx.size()==tz.size()&&tz.size()>2,"carved tail keeps its node grid");
    double iso_peak=0.;for(double const v:tz)iso_peak=std::max(iso_peak,std::abs(v));
    bt::require(iso_peak<=.06,"0.5 m carved iso8608 piece stays within twice the amplitude");
    bt::require(t::dump_track(t::track_from_dict(toml(t::dump_track(spectral_carve))))==t::dump_track(spectral_carve),"carved iso8608 track round trips");
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
    bt::require(io::canonical_json(t::AlpineGenSpec{}.to_dict()).find("surface_blend_m")==std::string::npos,"default alpine spec omits surface_blend_m for hash stability");
    {
        const auto blended_spec=t::AlpineGenSpec::from_dict(toml("surface_blend_m=0.4\nn_wet=[1,1]\n"));
        bt::require(blended_spec.surface_blend_m==0.4&&blended_spec.sha256()!=t::AlpineGenSpec{}.sha256(),"nonzero alpine surface_blend_m participates in spec hash");
        bt::require(t::AlpineGenSpec::from_dict(blended_spec.to_dict()).sha256()==blended_spec.sha256(),"alpine surface_blend_m roundtrips through dict");
        const auto gen_track=t::generate_alpine_track(blended_spec,"41");
        bt::require(!gen_track.surface_sections.empty(),"alpine track generates surface sections");
        for(const auto& sec:gen_track.surface_sections)bt::require(sec.blend_m>0.&&sec.blend_m<=0.4,"alpine generated sections carry clamped blend_m");
        bt::invalid([&]{(void)t::AlpineGenSpec::from_dict(toml("surface_blend_m=-0.1\n"));});
    }
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
void spectral_serialization(){
    const auto step=t::track_from_dict(toml("name='step'\nlength_m=20\n[[obstacles]]\ntype='step_up'\nstart_m=5.0\nheight_m=0.08\nramp_m=0.40\n"));
    const auto& ledge=std::get<t::StepUp>(step.obstacles[0].shape);
    bt::require(ledge.start_m==5.&&ledge.height_m==0.08&&ledge.ramp_m==0.4,"step_up fields parse from TOML");
    const auto step_dump=t::dump_track(step);
    bt::require(step_dump.find("type = \"step_up\"")!=std::string::npos,"step_up dumps its type tag");
    bt::require(t::dump_track(t::track_from_dict(toml(step_dump)))==step_dump,"step_up TOML round trip");
    bt::require(io::canonical_json(t::identity_value(step)).find("StepUp")!=std::string::npos,"step up enters track identity");
    bt::require(t::dump_track(step).find("ramp_m = 0.4")!=std::string::npos,"step up dumps its ramp length");
    const auto rough=t::track_from_dict(toml("name='rough'\nlength_m=60\n[[obstacles]]\ntype='roughness'\nstart_m=4\nsection_length_m=40\namplitude_m=0.03\ncorrelation_length_m=0.2\nspectrum='iso8608'\nwaviness=2.5\nseed=7\n"));
    const auto& rr=std::get<t::RoadRoughness>(rough.obstacles[0].shape);
    bt::require(rr.spectrum=="iso8608"&&rr.waviness==2.5,"iso8608 spectrum fields parse");
    const auto rough_dump=t::dump_track(rough);
    bt::require(rough_dump.find("spectrum = \"iso8608\"")!=std::string::npos&&rough_dump.find("waviness = 2.5")!=std::string::npos,"non-default spectrum fields dump");
    bt::require(t::dump_track(t::track_from_dict(toml(rough_dump)))==rough_dump,"iso8608 roughness TOML round trip");
    bt::require(io::canonical_json(t::identity_value(rough)).find("iso8608")!=std::string::npos,"iso8608 spectrum enters track identity");
    const auto plain=t::track_from_dict(toml("name='plain'\nlength_m=60\n[[obstacles]]\ntype='roughness'\nstart_m=4\nsection_length_m=40\namplitude_m=0.03\ncorrelation_length_m=0.2\nseed=7\n"));
    const auto plain_dump=t::dump_track(plain);
    bt::require(plain_dump.find("spectrum")==std::string::npos&&plain_dump.find("waviness")==std::string::npos,"default gaussian fields omitted from TOML");
    const auto plain_identity=io::canonical_json(t::identity_value(plain));
    bt::require(plain_identity.find("spectrum")==std::string::npos&&plain_identity.find("waviness")==std::string::npos,"default gaussian fields omitted from identity");
    const auto w2=t::track_from_dict(toml("name='w2'\nlength_m=60\n[[obstacles]]\ntype='roughness'\nstart_m=4\nsection_length_m=40\namplitude_m=0.03\ncorrelation_length_m=0.2\nspectrum='iso8608'\nseed=7\n"));
    const auto w2_dump=t::dump_track(w2);
    bt::require(w2_dump.find("spectrum = \"iso8608\"")!=std::string::npos&&w2_dump.find("waviness")==std::string::npos,"waviness 2.0 omitted while spectrum kept");
    bt::invalid([&]{(void)t::track_from_dict(toml("name='bad'\nlength_m=60\n[[obstacles]]\ntype='roughness'\nstart_m=4\nsection_length_m=40\nspectrum='bogus'\nseed=7\n"));});
    bt::invalid([&]{(void)t::track_from_dict(toml("name='bad'\nlength_m=60\n[[obstacles]]\ntype='roughness'\nstart_m=4\nsection_length_m=40\nwaviness=0.5\nseed=7\n"));});
    bt::invalid([&]{(void)t::track_from_dict(toml("name='bad'\nlength_m=60\n[[obstacles]]\ntype='roughness'\nstart_m=4\nsection_length_m=40\nwaviness=4.5\nseed=7\n"));});
    bt::invalid([&]{(void)t::track_from_dict(toml("name='bad'\nlength_m=20\n[[obstacles]]\ntype='step_up'\nstart_m=5\nheight_m=0.08\nramp_m=0.4\nbogus=1\n"));});
    bt::invalid([&]{(void)t::track_from_dict(toml("name='bad'\nlength_m=20\n[[obstacles]]\ntype='step_up'\nstart_m=5\nheight_m=0.08\nramp_m=0.4\nspectrum='iso8608'\n"));});
}
void examples_and_oracles(){
    auto root=bt::fixture("tracks/flat.toml");for(int i=0;i<4;++i)root=root.parent_path();
    // Optional spectral fields must leave every shipped track's canonical
    // identity bitwise identical; the digests below were captured on the
    // pre-change serialization and pin that contract.
    const std::array<std::pair<std::string_view,std::string_view>,5> identity_digests{{
        {"rough_uphill","3c518b7aa48716c2c7731c18483423c7fff883bf659250d0643583dac4de3d78"},
        {"rough_uphill_extreme","a8c51fb261b1866b44fca5895fcb42a16d413b9c8e438140248a3d6e894d3d4f"},
        {"rough_uphill_savage","1158c7a440363d33acdaae059e9962b62cb2a5a5389dd80352f93bb30ff046be"},
        {"alpine_scree","03dcdfa597e7511903ae22fb613c1a1998fd358e0af8d6767d015dc7b214dcf6"},
        {"alpine_rocky","1d5cd4059995f4b0388e4af0ce850765ffd092ea30e8122b04fac772c5d91f7a"}}};
    std::vector<std::pair<std::string_view,std::string>> shipped;
    for(const auto name:{"rough_uphill","rough_uphill_extreme","rough_uphill_savage","alpine_scree","alpine_rocky"}){
        const auto track=t::load_track(root/"examples"/"research"/(std::string(name)+".toml"));
        bt::require(t::dump_track(t::track_from_dict(toml(t::dump_track(track))))==t::dump_track(track),"all authored example tracks roundtrip");
        const auto identity=io::canonical_json(t::identity_value(track));
        bt::require(identity.find("spectrum")==std::string::npos&&identity.find("waviness")==std::string::npos,"shipped track "+std::string(name)+" identity carries no spectral keys");
        shipped.emplace_back(name,hash(identity));
    }
    std::string actuals;for(const auto& [n,d]:shipped){actuals+=n;actuals+='=';actuals+=d;actuals+=';';}
    for(const auto& [name,digest]:shipped){
        const auto expected=std::ranges::find_if(identity_digests,[name](const auto& e){return e.first==name;});
        std::string message="shipped track ";message+=name;message+=" identity digest ";message+=digest;message+="; all=";message+=actuals;
        bt::require(expected!=identity_digests.end()&&digest==expected->second,message);
    }
    
    bt::golden_text("tracks/rough_uphill_savage.canonical.toml",t::dump_track(t::load_track(bt::fixture("tracks/rough_uphill_savage.toml"))));
    bt::require(t::available_presets().size()==10,"all ten source presets present");
    for(const auto& name:t::available_presets()){
        const auto stem="tracks/preset_"+name;const auto track=t::get_preset(name);
        if(name=="climb_steps"){bt::invalid([&]{(void)t::track_to_dict(track);});continue;}
        bt::golden_text(stem+".toml",t::dump_track(track));
        const auto expected=io::parse_json(bt::text(bt::fixture(stem+".json")));bt::require(io::canonical_json(t::track_to_dict(track))==io::canonical_json(expected),"preset semantic fixture");
    }
    bt::invalid([]{(void)t::load_track(bt::fixture("tracks/unknown_obstacle.toml"));});bt::invalid([]{(void)t::load_track(bt::fixture("tracks/degenerate.toml"));});
}
}
int main(){return bt::run([]{schemas();carving_and_placement();integer_seeds();generators();spectral_serialization();examples_and_oracles();});}
