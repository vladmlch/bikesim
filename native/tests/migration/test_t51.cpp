#include "support/migration_test.hpp"
#include "platform/argv.hpp"
#include "batch/plan.hpp"
#include "io/json.hpp"
#include "platform/files.hpp"
#include "../../src/diag.hpp"
#include <chrono>
#include <filesystem>
#include <random>
#include <stop_token>
#include <thread>

namespace {
const bike::io::Value& member(const bike::io::Value& value,std::string_view key) {
    const auto* result=bike::io::find_member(bike::io::as_object(value,"batch test"),key);
    if(!result)throw std::invalid_argument("missing batch test member");
    return *result;
}
void wide_toml_and_ranges() {
    namespace batch=bike::batch;namespace io=bike::io;
    const auto wide=batch::expand_spec(batch::parse_spec_toml(R"TOML(
[base]
scenario = "rough_uphill"
[grid] # A quote in this comment is not a multiline string: "
seed = { start = 18446744073709551616, count = 2 }
rider-mass = [60.0, 80.0]
)TOML"));
    bt::require(wide.size()==4&&wide[0].argv==std::vector<std::string>{"--scenario","rough_uphill","--seed","18446744073709551616","--rider-mass","60.0"}&&
        wide[2].argv[3]=="18446744073709551617","TOML wide count range preserves arbitrary integers and Cartesian flag order");
    const auto descending=batch::expand_spec(batch::parse_spec_toml(R"TOML(
[grid]
motor-torque = { start = 18446744073709551617, stop = -18446744073709551618, step = -18446744073709551617 }
)TOML"));
    bt::require(descending.size()==3&&descending[0].argv.back()=="18446744073709551617"&&
        descending[1].argv.back()=="0"&&descending[2].argv.back()=="-18446744073709551617",
        "signed decimal range crosses zero with an arbitrary-width negative step and exclusive stop");
    const auto negative=batch::expand_spec(batch::parse_spec_toml("[grid]\nmotor-torque={start=-18446744073709551617,count=3}\n"));
    bt::require(negative.size()==3&&negative[1].argv.back()=="-18446744073709551616"&&negative[2].argv.back()=="-18446744073709551615",
        "negative arbitrary-width start increments exactly");
    const auto radix=batch::expand_spec(batch::parse_spec_toml(
        "[grid]\nseed=[0x1_0000_0000_0000_0000,0o2"+std::string(21,'0')+",0b1"+std::string(64,'0')+",+18_446_744_073_709_551_616]\n"));
    bt::require(radix.size()==4,"wide hexadecimal/octal/binary/underscored decimal values survive TOML arrays");
    for(const auto& job:radix)bt::require(job.argv.back()=="18446744073709551616","integer spelling converts to canonical decimal without binary64");
    const auto fractional=batch::expand_spec(batch::parse_spec_toml("[grid]\nrider-mass={start=60.5,count=2}\n"));
    bt::require(fractional.size()==2&&fractional[0].argv.back()=="60.5"&&fractional[1].argv.back()=="61.5","float start/count keeps source addition semantics");
    const auto empty=batch::expand_spec(batch::parse_spec_toml("[grid]\nseed={start=18446744073709551616,count=-18446744073709551616}\n"));
    bt::require(empty.empty(),"a nonpositive arbitrary-width count is an empty source range");
    bt::require(batch::expand_spec(batch::parse_spec_toml("# no sections\n")).size()==1,"empty TOML remains one default job");
    const auto explicit_jobs=batch::expand_spec(batch::parse_spec_toml(R"TOML(
[base]
duration = 1.0
[[job]]
seed = 9999999999999999999999999999999999999999
[[job]]
seed = 10000000000000000000000000000000000000000
)TOML"));
    bt::require(explicit_jobs.size()==2&&explicit_jobs[0].argv==std::vector<std::string>{"--duration","1.0","--seed","9999999999999999999999999999999999999999"}&&
        explicit_jobs[1].argv.back()=="10000000000000000000000000000000000000000","array-of-table jobs preserve source order and integers beyond a fixed 128-bit width");
    const auto carry=batch::expand_spec(batch::parse_spec_toml("[grid]\nseed={start=9999999999999999999999999999999999999999,count=2}\n"));
    bt::require(carry.size()==2&&carry[1].argv.back()=="10000000000000000000000000000000000000000","decimal range addition carries into an arbitrarily wide new digit");
    const auto lexical=batch::parse_spec_toml(R"TOML(
["18446744073709551616"]
123 = { 456 = 18446744073709551616, "789" = "123" }
text = "numbers stay text: [0xffffffffffffffffffff] = 123 #"
multiline = '''
[123]
start = 18446744073709551616
'''
enabled = true
real = 1.0
date = 1979-05-27T07:32:00Z
items = [1, # numeric comment 18446744073709551616
    18446744073709551616]
nested = [{123 = [18446744073709551616, {456 = -18446744073709551617}]}, []]
)TOML");
    const auto& table=member(lexical,"18446744073709551616");
    bt::require(io::as_number(member(member(table,"123"),"456"),"wide").lexeme=="18446744073709551616"&&
        io::as_string(member(member(table,"123"),"789"),"text")=="123","numeric table/inline keys and quoted value types are unchanged");
    bt::require(io::as_string(member(table,"multiline"),"multiline")=="[123]\nstart = 18446744073709551616\n"&&
        io::as_string(member(table,"text"),"text")=="numbers stay text: [0xffffffffffffffffffff] = 123 #","multiline strings and quoted syntax are never scanned as integers");
    bt::require(io::as_bool(member(table,"enabled"),"bool")&&!io::is_integer_lexeme(io::as_number(member(table,"real"),"float").lexeme)&&
        std::holds_alternative<io::Tagged>(member(table,"date").data),"booleans, floats and dates use the unchanged authoritative TOML parser");
    bt::require(io::as_number(io::as_array(member(table,"items"),"array")[1],"wide").lexeme=="18446744073709551616","array comments preserve integer value context");
    const auto& nested=io::as_array(member(table,"nested"),"nested");
    const auto& nested_values=io::as_array(member(nested[0],"123"),"nested values");
    bt::require(io::as_number(nested_values[0],"wide").lexeme=="18446744073709551616"&&
        io::as_number(member(nested_values[1],"456"),"negative").lexeme=="-18446744073709551617"&&
        io::as_array(nested[1],"empty nested array").empty(),"nested array/inline-table transitions preserve integer values, numeric keys and empty arrays");
    const auto escaped=batch::parse_spec_toml(R"TOML(seed = 18446744073709551616
text = "\u005f_bike_batch_integer_0_0"
)TOML");
    bt::require(io::as_number(member(escaped,"seed"),"seed").lexeme=="18446744073709551616"&&
        io::as_string(member(escaped,"text"),"text")=="__bike_batch_integer_0_0","decoded string collisions choose a new marker prefix without changing authored strings");
    for(const std::string_view invalid:{"seed=01","seed=0x_1","seed=1__0","seed=+0x1","seed=[1 2]","seed=\"unterminated"})
        bt::invalid([&]{(void)batch::parse_spec_toml(invalid);});
    bt::invalid([]{(void)batch::expand_spec(batch::parse_spec_toml("[grid]\nseed={start=0,stop=18446744073709551616,step=0}\n"));});
    bt::invalid([]{(void)batch::expand_spec(batch::parse_spec_toml("[grid]\nseed={start=18446744073709551616,count=18446744073709551616}\n"));});
}
struct Directory {
    std::filesystem::path path;
    Directory() {
        std::random_device entropy;
        for (int attempt = 0; attempt < 100; ++attempt) {
            path = std::filesystem::temp_directory_path() / ("bike-t51-" + std::to_string(entropy()));
            if (std::filesystem::create_directory(path)) return;
        }
        throw std::runtime_error("cannot create test directory");
    }
    ~Directory() { std::error_code ignored; std::filesystem::remove_all(path, ignored); }
    Directory(const Directory&) = delete;
    Directory& operator=(const Directory&) = delete;
    Directory(Directory&&) = delete;
    Directory& operator=(Directory&&) = delete;
};
}
int main(int argc, char** argv) {
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
    // bt::run wraps the test body in try/catch(...) — failures are reported, nothing escapes main.
    // cppcheck-suppress throwInEntryPoint
    return bt::run([&] {
        namespace batch = bike::batch;
        wide_toml_and_ranges();
        const auto grid = batch::expand_spec(bike::io::parse_json(R"({"base":{"scenario":"rough_uphill"},"grid":{"seed":[1,2],"rider-mass":[60.0,80.0]}})"));
        bt::require(grid.size() == 4 && grid[0].argv == std::vector<std::string>{"--scenario", "rough_uphill", "--seed", "1", "--rider-mass", "60.0"}, "source Cartesian order and decimal flag spelling");
        bt::require(grid[2].id == "0002-seed2-rider-mass60-0", "identity derives from input order");
        const auto flags = batch::expand_spec(bike::io::parse_json(R"({"base":{"assist":true,"ideal-sensors":false},"job":[{"seed":7,"motor-torque":120.0}]})"));
        bt::require(flags[0].argv == std::vector<std::string>{"--assist", "--seed", "7", "--motor-torque", "120.0"}, "bool false omitted and explicit job merged");
        const auto ranges = batch::expand_spec(bike::io::parse_json(R"({"grid":{"seed":{"start":10,"count":3},"rear-teeth":{"start":40,"stop":52,"step":5}}})"));
        bt::require(ranges.size() == 9 && ranges.back().argv.back() == "50", "range tables use stop-exclusive order");
        bt::require(batch::expand_spec(bike::io::parse_json("{}")).size() == 1, "empty spec is one default episode");
        const auto wide = batch::expand_spec(bike::io::parse_json(R"({"base":{"seed":9007199254740993},"job":[{"seed":9007199254740992.0}]})"));
        bt::require(wide[0].id != "0000", "integer/float equality never collapses a distinguishing wide seed");
        const auto equal = batch::expand_spec(bike::io::parse_json(R"({"base":{"seed":9007199254740992},"job":[{"seed":9007199254740992.0}]})"));
        bt::require(equal[0].id == "0000", "exactly equal int/float overrides retain source slug semantics");
        bt::invalid([] { (void)batch::expand_spec(bike::io::parse_json(R"({"base":{"out":"x"}})")); });
        bt::invalid([] { (void)batch::expand_spec(bike::io::parse_json(R"({"grid":{"seed":[]}})")); });
        bt::invalid([] { (void)batch::expand_spec(bike::io::parse_json(R"({"junk":{}})")); });
        bt::require(batch::parse_seeds("1-3,7,2") == std::vector<std::string>{"1", "2", "3", "7"}, "sweep seed order deduplicates without scheduling");
        bt::require(batch::parse_seeds("18446744073709551616-18446744073709551617") == std::vector<std::string>{"18446744073709551616", "18446744073709551617"}, "wide seeds do not pass through double/int64");
        bt::require(bike::io::canonical_json(batch::tail_json("diagnostic\n{\"outcome\":\"ok\"}\n")) == R"({"outcome":"ok"})", "worker JSON tail extraction");
        bt::require(argc == 2, "native test child is supplied by CTest");
        Directory const output;
        const auto supplied=bike::platform::main_arguments(argc,argv);
        const std::string child = supplied[1];
        const std::vector<batch::Job> jobs{{.id="a", .argv={child, "exit", "0"}}, {.id="b", .argv={child, "exit", "3"}}};
        batch::ExecuteOptions options; options.cwd = output.path;
        const auto serial = batch::execute(jobs, 1, options), parallel = batch::execute(jobs, 2, options);
        bt::require(serial.size() == 2 && parallel[0].id == "a" && parallel[1].id == "b", "worker completion never changes result order");
        bt::require(serial[0].exit_code == parallel[0].exit_code && serial[1].exit_code == 3 && parallel[1].exit_code == 3, "parallel nonzero outcome is preserved");
        const std::string literal = "Путь с пробелами `literal` $(literal)";
        const std::vector<batch::Job> echo{{.id="literal", .argv={child, "echo", literal}}, {.id="missing", .argv={(output.path / "missing-binary").string()}}};
        const auto echoed = batch::execute(echo, 2, options);
        bt::require(echoed[0].out == literal + "\n" && echoed[1].exit_code == -1, "argv is literal and spawn failure is a result");
        const auto file = output.path / "exclusive output.txt";
        const std::vector<batch::Job> write{{.id="write", .argv={child, "write", file.string(), "first"}}};
        bt::require(batch::execute(write, 1, options)[0].exit_code == 0, "exclusive child output succeeds once");
        bt::require(batch::execute(write, 1, options)[0].exit_code == 2 && bt::text(file) == "first", "collision preserves prior output");
        std::stop_source cancellation; options.cancellation = cancellation.get_token();
        std::jthread const cancel([&] { std::this_thread::sleep_for(std::chrono::milliseconds(50)); cancellation.request_stop(); });
        const std::vector<batch::Job> sleepers{{.id="running", .argv={child, "sleep", "1000"}}, {.id="queued", .argv={child, "sleep", "1000"}}};
        const auto stopped = batch::execute(sleepers, 1, options);
        bt::require(stopped[0].exit_code == 130 && stopped[1].exit_code == 130, "cancellation signals/reaps active child and prevents queued launch");
        bt::require(std::filesystem::current_path() != output.path, "process pool leaves parent cwd unchanged");
    });
NATIVE_DIAG_POP
}
