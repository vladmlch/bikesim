#include "validation/acceptance.hpp"
#include "validation/cli_options.hpp"
#include "configuration/load.hpp"
#include "io/json.hpp"
#include "platform/argv.hpp"
#include <iostream>
#include <map>
#include <set>

namespace {
namespace bv=bike::validation;
namespace io=bike::io;
using bv::Value;using bv::Object;using bv::Array;
namespace value=bv::value;
std::string normalize(std::string name){std::ranges::replace(name,'-','_');return name;}
void help(){std::cout<<
    "bike-validate list | discover\n"
    "bike-validate run CASE [--config FILE.json|FILE.toml] [--root DIRECTORY] [options]\n"
    "bike-validate MODULE [options]\n\n"
    "Modules retain their Python argument names:\n"
    "  rider-replay: --physics-config --track --duration --timestep --mode --activation-tau --output\n"
    "  rider-factor-matrix: --physics-config --output\n"
    "  drive-suspension-rig, contact-resolution: --output\n"
    "  fidelity-sweep: --output --cases NAME... --backend --transmission --stations --jobs\n"
    "                  --time-steps N... --road-steps N... --station-counts N... --physics-config\n"
    "  reference-release: --output --physics-config --dt --dx --scenarios NAME...\n"
    "  climbing-benchmark: --physics-config --track --mode --duration --repeats --output\n"
    "  antiwheelie-bench: --physics-config --output --phase mechanics\n"
    "  benchmarks: --output --cases NAME... --dt-values N... --jobs\n\n"
    "Time-dependent individual rigs accept --dt or a JSON/TOML configuration. --timestep is\n"
    "an alias of --dt wherever that operation consumes dt_s; supplying both is an error.\n"
    "Scalar/postprocessing operations\n"
    "take their complete inputs through --config. --report FILE writes the full CLI envelope.\n"
    "Paths in module options resolve under --root (default: current directory).\n"
    "jobs=1 is implemented; isolated parallel workers remain unavailable and are rejected.\n"
    "Reports carry physical validity, numerical validity, numerical parity and performance\n"
    "independently. Null means that this operation did not establish that kind of evidence.\n"
    "Exit: 0 accepted/diagnostic completion, 1 failed declared criteria, 2 invalid inputs/runtime error.\n";}
Value numeric_value(std::string_view token,std::string_view name,bool integer){ // NOLINT(bugprone-easily-swappable-parameters) call sites keep the declared parameter order.
    Value result=io::parse_json(token);if(integer)(void)io::as_int64(result,name);else (void)io::as_double(result,name);return result;
}
int report_exit(std::string_view id,const bv::Verdict& verdict){
    if(id=="reference_release"){for(const auto& row:io::as_array(bv::get(verdict.evidence,"scenarios"),"scenarios")){
        const auto outcome=bv::text(bv::get(row,"outcome"),"outcome");if(outcome=="model_invalid"||outcome=="numerical_failure")return 2;}return 0;}
    const std::set<std::string_view> diagnostics{
        "holdout_metrics","radial_rig","native_radial_rig","radial_native_reference","wheel_inertia_rig","incline_rig","brake_stand",
        "chain_locked_rig","chain_geometry","compiled_chain_motion","quasistatic_front_load","grade_feasibility","distributed_road_trace",
        "rider_work","trace_integral","interval_integral","coasting_evidence","resume_evidence","timing_metrics","paired_summary",
        "classify_outcome","persistent_events","metrics_from_rows","scenarios","automatic_coast","profile_identity","source_fingerprint"};
    return verdict.accepted||diagnostics.contains(id)?0:1;
}
} // namespace

// NOLINTNEXTLINE(bugprone-exception-escape) main catches std::exception and catch-all below emit the error envelope; residual escapes are stream failures at process exit
int main(int argc,char** argv){
    try{
        if(argc<2){help();return 2;}
        std::vector<std::string> args;for(char* argument:bike::platform::main_arguments(argc,argv).subspan(1))args.emplace_back(argument);
        if(args[0]=="--help"||args[0]=="-h"||args[0]=="help"){help();return 0;}
        if(args[0]=="list"||args[0]=="discover"||args[0]=="--list"){
            if(args.size()!=1)throw std::invalid_argument("list/discover accepts no simulation options");Array rows;
            for(const auto& item:bv::cases())rows.push_back(Value{Object{{"id",value::text(item.id)},{"source",value::text(item.source)},
                {"description",value::text(item.description)},{"slow",Value{item.slow}}}});
            std::cout<<io::recording_json(Value{Object{{"schema_version",value::integer(1)},{"cases",Value{std::move(rows)}}}})<<'\n';return 0;
        }
        std::size_t at=0;if(args[0]=="run")++at;if(at==args.size())throw std::invalid_argument("run requires a case id");const auto original=args[at++],id=normalize(original);
        if(std::ranges::none_of(bv::cases(),[&](const auto& c){return normalize(std::string(c.id))==id;}))throw std::invalid_argument("unknown validation case: "+original);
        Value config{Object{}},overrides{Object{}};std::filesystem::path root=std::filesystem::current_path();std::optional<std::filesystem::path> input,report;
        const std::map<std::string,std::string> strings{{"--physics-config","physics_profile"},{"--track","track"},{"--mode","mode"},
            {"--backend","backend"},{"--transmission","transmission"},{"--output","output"},{"--phase","phase"}};
        const std::map<std::string,std::string> reals{{"--dt","dt_s"},{"--timestep","dt_s"},{"--dx","dx_m"},{"--duration","duration_s"},{"--activation-tau","activation_tau_s"}};
        const std::map<std::string,std::string> integers{{"--jobs","jobs"},{"--repeats","repeats"}};
        const std::map<std::string,std::string> lists{{"--cases","cases"},{"--scenarios","names"},{"--time-steps","time_steps"},
            {"--road-steps","road_steps"},{"--station-counts","station_counts"},{"--dt-values","dt_values_s"}};
        bv::CliOptions options(id);
        while(at<args.size()){
            const auto option=args[at++];if(option=="--help"||option=="-h"){help();return 0;}
            options.accept(option);
            auto const next=[&]{if(at==args.size()||args[at].starts_with("--"))throw std::invalid_argument("missing value for "+option);return args[at++];};
            if(option=="--config"){input=next();continue;}if(option=="--root"){root=next();continue;}if(option=="--report"){report=next();continue;}
            if(option=="--stations"){value::set(std::get<Object>(overrides.data),"include_stations",Value{true});continue;}
            if(const auto it=strings.find(option);it!=strings.end()){value::set(std::get<Object>(overrides.data),it->second,value::text(next()));continue;}
            if(const auto it=reals.find(option);it!=reals.end()){value::set(std::get<Object>(overrides.data),it->second,numeric_value(next(),option,false));continue;}
            if(const auto it=integers.find(option);it!=integers.end()){value::set(std::get<Object>(overrides.data),it->second,numeric_value(next(),option,true));continue;}
            if(const auto it=lists.find(option);it!=lists.end()){Array entries;const bool named=option=="--cases"||option=="--scenarios";
                while(at<args.size()&&!args[at].starts_with("--")){const auto& token=args[at++];entries.push_back(named?value::text(token):numeric_value(token,option,option=="--station-counts"));}
                if(entries.empty())throw std::invalid_argument(option+" requires at least one value");value::set(std::get<Object>(overrides.data),it->second,Value{std::move(entries)});continue;}
            throw std::invalid_argument("unknown argument: "+option);
        }
        if(!std::filesystem::is_directory(root))throw std::invalid_argument("--root must name an existing directory");root=std::filesystem::absolute(root);
        if(input){const auto path=root/ *input;config=path.extension()==".toml"?bike::configuration::detail::parse_toml_file(path):bv::read_json(path);(void)io::as_object(config,"case configuration");}
        for(const auto& [key,item]:io::as_object(overrides,"options"))value::set(std::get<Object>(config.data),key,item);
        if(id=="rider_replay")(void)bv::identifier(bv::get(config,"output"),"output");
        if(id=="antiwheelie_bench"||id=="climbing_benchmark")for(const char* field:{"physics_profile","output"})(void)bv::identifier(bv::get(config,field),field);
        if(id=="climbing_benchmark")for(const char* field:{"track","mode"})(void)bv::identifier(bv::get(config,field),field);
        const auto verdict=bv::run_case(original,config,root);
        auto envelope=bv::report_axes(verdict.evidence);auto& fields=std::get<Object>(envelope.data);
        fields.emplace_back("case_id",value::text(original));fields.emplace_back("accepted",Value{verdict.accepted});fields.emplace_back("configuration",config);
        fields.emplace_back("evidence",verdict.evidence);fields.emplace_back("acceptance_scope",value::text("declared case criteria or input schema; independent evidence axes are explicit"));
        const std::set<std::string> own_output{"rider_replay","rider_factor_matrix","drive_suspension_rig","fidelity_sweep","reference_release",
            "contact_resolution","benchmarks","climbing_benchmark","antiwheelie_bench","release_manifest"};
        if(const auto* file=bv::find(config,"output");file&&!own_output.contains(id))bv::write_report(root/io::as_string(*file,"output"),verdict.evidence);
        if(report)bv::write_report(root/ *report,envelope);
        std::cout<<io::recording_json(envelope)<<'\n';return report_exit(id,verdict);
    }catch(const std::exception& error){
        std::cerr<<io::recording_json(Value{Object{{"accepted",Value{false}},{"error",value::text(error.what())},
            {"physical_validity",Value{}},{"numerical_validity",Value{}},{"numerical_parity",Value{}},{"runtime_performance",Value{}}}})<<'\n';return 2;
    }catch(...){
        std::cerr<<io::recording_json(Value{Object{{"accepted",Value{false}},{"error",value::text("unrecognized exception")},
            {"physical_validity",Value{}},{"numerical_validity",Value{}},{"numerical_parity",Value{}},{"runtime_performance",Value{}}}})<<'\n';return 2;
    }
}
