// Direct analyst API and standalone-process regressions; authored, not run.
#include "recording_reader_test_support.hpp"
#include "recording_fixture.hpp"
#include "cli/recording.hpp"
#include "platform/argv.hpp"
#include "platform/process.hpp"
#include <sstream>

namespace {
using namespace bike::recording::detail;
struct Result { int code; std::string out,error; };
Result command(const std::vector<std::string>& args) {
    std::vector<std::string_view> views; views.reserve(args.size());
for(const auto& arg:args)views.push_back(arg);
    std::ostringstream out,error; const int code=bike::cli::recording_command(views,out,error);
    return {.code=code,.out=out.str(),.error=error.str()};
}
// The standalone process receives the same genuine sealed fixture. These
// cases verify dispatch/stdio/exit status beyond the direct C++ entry point.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
void analyst_process(const std::filesystem::path& executable, const std::string& path,
                     const std::string& expected_json, const std::filesystem::path& cwd) {
    const auto process=[&](std::vector<std::string> args) {
        args.insert(args.begin(),executable.string());return bike::platform::run(args,cwd);
    };
    const auto help=process({"--help"});
    bt::require(help.exit_code==0&&help.out.find("usage: bike-recording")!=std::string::npos&&help.err.empty(),"standalone analyst help runs without a simulator or display request");
    const auto overview=process({"overview",path,"--json"});
    bt::require(overview.exit_code==0&&overview.err.empty()&&equal(bike::io::parse_recording_json(overview.out),bike::io::parse_recording_json(expected_json)),
        "native executable and direct analyst API return the same sealed evidence");
    const auto invalid=process({"sql",path,"--query","DELETE FROM physics"});
    bt::require(invalid.exit_code==1&&invalid.out.empty()&&invalid.err.find("RECORDING ERROR")!=std::string::npos,"standalone SQL rejection keeps stdout and stderr separate");
    const auto process_export=cwd/"process-metrics.csv";
    const auto exported_process=process({"export",path,"--episode","0","--table","metrics","--format","csv","--output",process_export.string()});
    bt::require(exported_process.exit_code==0&&bt::text(process_export).find("9007199254740993")!=std::string::npos,"standalone export writes actual exact-integer CSV output");
}
void analyst(const std::filesystem::path& executable) {
    recording_reader_test::Directory const tmp;
    bt::require(command({"--help"}).code==0,"analyst help does not open a recording");
    bt::require(command({}).code==2&&command({"missing-command"}).code==2,"required command/parser failures return status 2");
    bt::require(command({"window","missing","--episode","0","--start","0","--end","1"}).code==2,"window requires explicit fields before recording access");
    bt::require(command({"overview","missing","--limit","bad"}).code==2,"bad integer is rejected before opening missing input");
    bt::require(command({"export","missing","--episode","0","--format","parquet","--output","wrong"}).code==2,"format choices reject unsupported convenience export");
    const auto missing=command({"overview",(tmp.path/"missing").string()});
    bt::require(missing.code==1&&missing.out.empty()&&missing.error.find("RECORDING ERROR")!=std::string::npos,"missing/unsealed recording is a status 1 diagnostic on stderr");
    const auto published=recording_fixture::publish(tmp.path/"runs",3); const auto path=published.directory.string();
    const auto plain=command({"overview",path}),json=command({"overview",path,"--json"});
    bt::require(plain.code==0&&json.code==0&&plain.error.empty()&&json.error.empty()&&
        equal(bike::io::parse_recording_json(plain.out),bike::io::parse_recording_json(json.out)),"normal/JSON output has identical bounded semantics");
    const auto sql=command({"sql",path,"--query","SELECT 9007199254740993::BIGINT AS exact","--json"});
    bt::require(sql.code==0&&sql.out.find("9007199254740993")!=std::string::npos,"analyst SQL preserves bigint precision");
    bt::require(command({"sql",path,"--query","SELECT 1; DELETE FROM events"}).code==1,"multiple statements never execute through the analyst command");
    bt::require(command({"events",path,"--episode","0","--kind","episode_ended","--json"}).code==0,"events command selects source classifications");
    bt::require(command({"window",path,"--episode=0","--start=0","--end=.003","--fields=x_m,powers_w.chain","--json"}).code==0,"equals-form options work for interval aliases");
    bt::require(command({"compare",path,path,"--left-episode","0","--right-episode","0","--fields","physics.qpos","--json"}).code==0,"exact comparison is available without simulation or policies");
    const auto output=tmp.path/"export.jsonl";
    const auto exported=command({"export",path,"--episode","0","--table","metrics","--format","jsonl","--output",output.string()});
    bt::require(exported.code==0&&count(field(bike::io::parse_recording_json(exported.out),"rows"))==3,"CLI export defaults to all rows");
    bt::require(command({"overview",path,"--limit","0"}).code==1,"nonpositive bounded limit is a semantic status 1 failure");
    bt::require(command({"overview",path,"--limit"," +0_2 "}).code==0,"integer options preserve source plus, whitespace and separator syntax");
    bt::require(command({"overview",path,"--limit","999999999999999999999999"}).code==1,"unrepresentable integer limits are semantic range failures, not malformed integer syntax");
    bt::require(command({"window",path,"--episode","0","--start","1e-9999","--end",".003","--fields","x_m"}).code==0,"source float underflow becomes zero before interval selection");
    bt::require(command({"window",path,"--episode","0","--start","0","--end","1e9999","--fields","x_m"}).code==1,"source float overflow is rejected by the finite interval domain, not the parser");
    bt::require(command({"window",path,"--episode","0","--start","nan(payload)","--end",".003","--fields","x_m"}).code==2,"implementation NaN payload syntax is not a source CLI float token");
    analyst_process(executable,path,json.out,tmp.path);
}
}
int main(int argc,char** argv){return bt::run([&]{
    const auto arguments=bike::platform::main_arguments(argc,argv);
    bt::require(arguments.size()==2,"standalone analyst executable path supplied by CTest");
    analyst(arguments[1]);
});}
