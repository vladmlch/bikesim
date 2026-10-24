// Parse and resolve the actual documented argv. No simulation/GLFW/output.
#include "support/migration_test.hpp"
#include "support/temporary_directory.hpp"
#include "fixture_paths.hpp"
#include "cli/application.hpp"
#include "cli/recording.hpp"
#include "batch/plan.hpp"
#include "io/json.hpp"
#include "recording/value_util.hpp"
#include <array>
#include <map>
#include <set>

namespace {
using bike::io::Value;
std::string expanded(std::string value,const std::filesystem::path& examples,const std::filesystem::path& out) {
    for(const auto& [from,to]:std::array<std::pair<std::string,std::string>,3>{{
        {"${EXAMPLES}",examples.string()},{"${OUT}",out.string()},{"${RUN}",(out/"sealed-run-placeholder").string()}}})
        for(auto pos=value.find(from);pos!=std::string::npos;pos=value.find(from,pos+to.size()))value.replace(pos,from.size(),to);
    return value;
}
void check(const bike::cli::Arguments& args,const Value& expected) {
    using namespace bike::recording::value;
    std::optional<bike::spatial::Config> spatial;double duration=0.;
    if(args.command=="bike-ride") {
        const auto resolved=bike::cli::resolve_ride(args);spatial=resolved->setup.model.spatial;duration=resolved->arguments.real("duration");
    } else {
        const auto resolved=bike::cli::resolve_research(args);spatial=resolved->setup.model.spatial;duration=resolved->experiment.duration_s;
        if(spatial)bt::require(resolved->sensors_v2.sample_period_s>0.,"documented spatial research owns a validated V2 sensor request");
    }
    const auto mode=spatial?"3d":"planar";
    const auto root=spatial?(spatial->root==bike::spatial::RootKind::free6?"free":"constrained"):"planar";
    bt::require(bike::io::as_string(require(expected,"mode"),"mode")==mode,"example mode resolves as documented");
    bt::require(bike::io::as_string(require(expected,"root"),"root")==root,"example root resolves as documented");
    bt::require(bike::io::as_int64(require(expected,"sensor_schema"),"sensor schema")== (spatial?2:1),"sensor schema follows mode");
    bt::require(bike::io::as_int64(require(expected,"container_schema"),"container schema")== (spatial?2:1),"container schema follows mode");
    bt::require(duration>0.&&duration==bike::io::as_double(require(expected,"duration_s"),"duration"),"every producer has a finite bounded positive duration");
    bt::require(args.flag("record")==bike::io::as_bool(require(expected,"record"),"record"),"recording remains an explicit flag");
    if(spatial)bt::require(args.text("spatial")=="3d","documented spatial command explicitly selects the mode");
}
}
int main(){return bt::run([]{
    using namespace bike::recording::value;
    const bt::TemporaryDirectory output("spatial-examples");
    const auto examples=std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT)/"examples/spatial";
    const auto manifest=bike::io::parse_json(bt::text(examples/"examples_manifest.json"));
    const auto& entries=bike::io::as_array(require(manifest,"examples"),"examples");
    std::map<std::string,Value> producer_expectations;std::set<std::string> ids;
    for(const auto& entry:entries) {
        const auto id=bike::io::as_string(require(entry,"id"),"id");
        bt::require(ids.insert(id).second,"example ids are unique");
        if(bike::io::as_string(require(entry,"kind"),"kind")=="producer")producer_expectations.emplace(id,require(entry,"expected"));
    }
    for(const auto& entry:entries) {
        const auto program=bike::io::as_string(require(entry,"program"),"program");
        std::vector<std::string> owned;for(const auto& arg:bike::io::as_array(require(entry,"argv"),"argv"))owned.push_back(expanded(bike::io::as_string(arg,"arg"),examples,output.path()));
        std::vector<std::string_view> argv;for(const auto& arg:owned)argv.push_back(arg);
        const auto& expected=require(entry,"expected");
        if(program=="bike-ride"||program=="bike-research") {
            const auto parsed=bike::cli::parse_arguments(program,argv);check(parsed,expected);
            if(parsed.has("terrain3d"))bt::require(argv.size()>3&&parsed.text("terrain3d")==argv[3],"documented terrain path is preserved by the canonical parser");
        } else if(program=="bike-research-sweep") {
            const auto sweep=bike::batch::parse_sweep_arguments(argv);
            bt::require(sweep.seeds.size()==4&&sweep.workers==2,"documented sweep resolves a process manifest");
            std::vector<std::string_view> child;for(const auto& arg:sweep.passthrough)child.push_back(arg);
            check(bike::cli::parse_arguments("bike-research",child),expected);
        } else {
            if(program=="bike-recording")bike::cli::validate_recording_arguments(argv);
            else {bt::require(program=="bike-replay","all companion programs have an authoritative parser");const auto parsed=bike::cli::parse_arguments(program,argv);bt::require(parsed.flag("checked"),"checked replay is explicit");}
            const auto source=bike::io::as_string(require(entry,"source"),"source");
            bt::require(producer_expectations.contains(source),"analysis command references a documented recording producer");
            const auto& producer=producer_expectations.at(source);
            for(const auto field:{"mode","root","sensor_schema","container_schema","duration_s"})
                bt::require(bike::io::canonical_json(require(expected,field))==bike::io::canonical_json(require(producer,field)),"analysis expectations match the actual producer mode/schema/root/duration");
        }
    }
    bt::require(ids.size()==10&&std::filesystem::is_empty(output.path()),"all ten documented workflows resolve without creating output");
});}
