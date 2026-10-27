#include "recording.hpp"
#include "recording_golden.hpp"
#include "../recording/reader.hpp"
#include "../recording/export.hpp"
#include "../recording/spatial_analysis.hpp"
#include "../viz/spatial.hpp"
#include "../recording/analysis_detail.hpp"
#include <algorithm>
#include <charconv>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace bike::cli {
namespace {
using namespace recording::detail;
struct ParseError : std::invalid_argument { using std::invalid_argument::invalid_argument; ~ParseError() override; ParseError(const ParseError&) = default; ParseError& operator=(const ParseError&) = default; ParseError(ParseError&&) = default; ParseError& operator=(ParseError&&) = default; };
ParseError::~ParseError() = default;
constexpr std::string_view help =
    "usage: bike-recording [-h] {inspect,overview,fields,events,window,sql,export,compare,report} ...\n\n"
    "Inspect a sealed run without constructing a simulator or importing a policy.\n\n"
    "positional arguments:\n  {inspect,overview,fields,events,window,sql,export,compare,report}\n\noptions:\n  -h, --help  show this help message and exit\n";
std::string command_help(std::string_view name) {
    std::string out = "usage: bike-recording " + std::string(name);
    if (name == "compare") return out + " left right --left-episode N --right-episode N [--mode {exact,summary}] [--fields NAMES] "
        "[--atol FLOAT] [--rtol FLOAT] [--limit N] [--json]\n";
    out += " run";
    if (name == "report") return out + " --episode N --output PATH.{html,svg,png} [--kind {all,attitude,contact,path,imu,energy}]\n";
    if (name == "events" || name == "window" || name == "export") out += " --episode N";
    if (name == "fields") out += " [--pattern PATTERN]";
    if (name == "events") out += " [--kind KIND]";
    if (name == "window") out += " --start FLOAT --end FLOAT --fields NAMES";
    if (name == "sql") out += " --query SELECT";
    if (name == "export") out += " --format {csv,jsonl} --output PATH [--table {telemetry,legacy_telemetry,forces,channel_spatial,channel_spatial_energy,channel_spatial_geometry,channel_spatial_outcome,commands_requested,commands_applied,observations,transitions,events,metrics}] [--limit N]\n\noptional explicit export row limit; default exports all rows\n";
    else out += " [--limit N] [--json]\n";
    return out;
}
struct Arguments {
    std::string command;
    std::vector<std::string> positions;
    std::map<std::string, std::string> options;
    bool json = false;
    [[nodiscard]] std::string required(std::string_view name) const {
        const auto it = options.find(std::string(name));
        if (it == options.end()) throw ParseError("the following argument is required: --" + std::string(name));
        return it->second;
    }
    [[nodiscard]] std::optional<std::string> optional(std::string_view name) const {
        const auto it = options.find(std::string(name)); return it == options.end() ? std::nullopt : std::optional(it->second);
    }
    [[nodiscard]] std::string get(std::string_view name, const std::string& fallback) const { const auto value = optional(name); return value ? *value : fallback; }
};
std::string numeric_token(const std::string& source,std::string_view name,bool integral) {
    constexpr std::string_view whitespace=" \t\n\r\v\f";
    const auto first=source.find_first_not_of(whitespace),last=source.find_last_not_of(whitespace);
    if(first==std::string::npos)throw ParseError("invalid numeric value for --"+std::string(name)+": "+source);
    std::string_view raw(source);raw=raw.substr(first,last-first+1);
    std::string token;token.reserve(raw.size());
    const auto digit=[](char c){return c>='0'&&c<='9';};
    for(std::size_t i=0;i<raw.size();++i) {
        if(raw[i]=='_') {
            if(i==0||i+1==raw.size()||!digit(raw[i-1])||!digit(raw[i+1]))throw ParseError("invalid digit separator for --"+std::string(name));
        } else token+=raw[i];
    }
    if(token.starts_with('+'))token.erase(0,1);
    if(integral) {
        const std::string_view digits=std::string_view(token).substr(token.starts_with('-')?1:0);
        if(digits.empty()||!std::ranges::all_of(digits,digit))throw ParseError("invalid integer value for --"+std::string(name)+": "+source);
    }
    return token;
}
std::int64_t integer_argument(const std::string& source, std::string_view name) {
    std::int64_t value = 0;
    const auto token=numeric_token(source,name,true);
    const auto end=std::to_address(token.end());
    const auto parsed = std::from_chars(token.data(), end, value);
    if (parsed.ec != std::errc{} || parsed.ptr != end) throw std::invalid_argument("integer value for --" + std::string(name) + " exceeds the recording index/limit domain");
    return value;
}
double real_argument(const std::string& source, std::string_view name) {
    const auto token=numeric_token(source,name,false);
    bool valid=false;
    const double value=io::detail::float_from_lexeme(token,valid);
    // Python float accepts overflow/underflow as inf/signed zero; the
    // operation's finite/range contract rejects inappropriate values later.
    // from_chars additionally accepts implementation NaN payload syntax,
    // which is not part of the source command's float grammar.
    if (!valid || token.find_first_of("()")!=std::string::npos) throw ParseError("invalid float value for --" + std::string(name) + ": " + source);
    return value;
}
Arguments parse(std::span<const std::string_view> arguments) {
    if (arguments.empty()) throw ParseError("the following argument is required: command");
    Arguments out; out.command = arguments[0];
    const std::set<std::string> commands{"inspect", "overview", "fields", "events", "window", "sql", "export", "compare", "report"};
    if (!commands.contains(out.command)) throw ParseError("invalid command: " + out.command);
    std::set<std::string> allowed{"limit"};
    if (out.command == "events" || out.command == "window" || out.command == "export") allowed.insert("episode");
    if (out.command == "fields") allowed.insert("pattern");
    if (out.command == "events") allowed.insert("kind");
    if (out.command == "window") allowed.insert({"start", "end", "fields"});
    if (out.command == "sql") allowed.insert("query");
    if (out.command == "export") allowed.insert({"format", "output", "table"});
    if (out.command == "report") allowed.insert({"episode", "output", "kind"});
    if (out.command == "compare") allowed.insert({"left-episode", "right-episode", "mode", "fields", "atol", "rtol"});
    bool positions_only = false;
    for (std::size_t i = 1; i < arguments.size(); ++i) {
        const std::string token(arguments[i]);
        if (!positions_only && token == "--") { positions_only = true; continue; }
        if (!positions_only && token == "--json" && out.command != "export") { out.json = true; continue; }
        if (!positions_only && token.starts_with('-')) {
            if (!token.starts_with("--")) throw ParseError("unrecognized argument: " + token);
            const auto equal = token.find('='); const auto name = token.substr(2, equal == std::string::npos ? equal : equal - 2);
            if (!allowed.contains(name)) throw ParseError("unrecognized argument: --" + name);
            if (equal != std::string::npos) out.options[name] = token.substr(equal + 1);
            else {
                ++i;
                if (i == arguments.size() || arguments[i].starts_with("--")) throw ParseError("argument --" + name + " expected one value");
                out.options[name] = arguments[i];
            }
        } else out.positions.push_back(token);
    }
    const std::size_t required = out.command == "compare" ? 2 : 1;
    if (out.positions.size() != required) throw ParseError("expected " + std::to_string(required) + " recording path argument(s)");
    if (out.command == "events" || out.command == "window" || out.command == "export") (void)out.required("episode");
    if (out.command == "report") {
        (void)out.required("episode"); (void)out.required("output");
        const std::set<std::string> kinds{"all","attitude","contact","path","imu","energy"};
        if (!kinds.contains(out.get("kind","all"))) throw ParseError("invalid report --kind");
    }
    if (out.command == "window") for (auto key : {"start", "end", "fields"}) (void)out.required(key);
    if (out.command == "sql") (void)out.required("query");
    if (out.command == "export") {
        (void)out.required("output"); const auto format = out.required("format");
        if (format != "csv" && format != "jsonl") throw ParseError("invalid --format: choose csv or jsonl");
        const std::set<std::string> tables{"telemetry", "legacy_telemetry", "forces", "commands_requested", "commands_applied", "observations", "transitions", "events", "metrics", "channel_spatial", "channel_spatial_energy", "channel_spatial_geometry", "channel_spatial_outcome"};
        if (!tables.contains(out.get("table", "telemetry"))) throw ParseError("invalid --table");
    }
    if (out.command == "compare") {
        (void)out.required("left-episode"); (void)out.required("right-episode");
        const auto mode = out.get("mode", "exact"); if (mode != "exact" && mode != "summary") throw ParseError("invalid --mode: choose exact or summary");
    }
    // argparse parses option values before opening the source recording.
    for (auto name : {"limit", "episode", "left-episode", "right-episode"}) if (const auto value = out.optional(name)) (void)numeric_token(*value, name, true);
    for (auto name : {"start", "end", "atol", "rtol"}) if (const auto value = out.optional(name)) (void)real_argument(*value, name);
    return out;
}
} // namespace
void validate_recording_arguments(std::span<const std::string_view> arguments) { (void)parse(arguments); }
int recording_command(std::span<const std::string_view> arguments, std::ostream& out, std::ostream& error) {
    if (!arguments.empty() && arguments[0] == "golden")
        return golden_command(arguments.subspan(1), out, error);
    if (arguments.size() == 1 && (arguments[0] == "--help" || arguments[0] == "-h")) { out << help; return 0; }
    if (arguments.size() > 1 && std::ranges::any_of(arguments.subspan(1), [](auto arg) { return arg == "-h" || arg == "--help"; })) {
        const std::set<std::string_view> known{"inspect", "overview", "fields", "events", "window", "sql", "export", "compare", "report"};
        if (known.contains(arguments[0])) { out << command_help(arguments[0]); return 0; }
    }
    try {
        const auto args = parse(arguments);
        const auto limit_value = integer_argument(args.get("limit", "200"), "limit");
        const auto maximum = limit_value > 0 ? static_cast<std::uint64_t>(limit_value) : 0;
        Value result;
        if (args.command == "compare") {
            const auto left = recording::open_recording(args.positions[0]), right = recording::open_recording(args.positions[1]);
            result = recording::compare(*left, *right, integer_argument(args.required("left-episode"), "left-episode"),
                integer_argument(args.required("right-episode"), "right-episode"), args.get("mode", "exact"), args.optional("fields"),
                real_argument(args.get("atol", "1e-9"), "atol"), real_argument(args.get("rtol", "1e-9"), "rtol"), maximum);
        } else {
            const auto reader = recording::open_recording(args.positions[0]);
            if (args.command == "overview" || args.command == "inspect") result = recording::overview(*reader, maximum);
            else if (args.command == "fields") result = recording::fields(*reader, args.optional("pattern"), maximum);
            else if (args.command == "events") result = recording::events(*reader, integer_argument(args.required("episode"), "episode"), args.optional("kind"), maximum);
            else if (args.command == "window") result = recording::window(*reader, integer_argument(args.required("episode"), "episode"),
                real_argument(args.required("start"), "start"), real_argument(args.required("end"), "end"), args.optional("fields"), maximum);
            else if (args.command == "sql") result = reader->query(args.required("query"), maximum);
            else if (args.command == "report") {
                const auto channels = recording::spatial_report_channels(*reader, integer_argument(args.required("episode"),"episode"));
                const auto kind = args.get("kind","all");
                viz::Report report{.title="Spatial recording", .panels={}, .columns=1, .width=1280, .height=1600};
                const auto append = [&](viz::Report part) {
                    for(auto& panel:part.panels)report.panels.push_back(std::move(panel));
                };
                if (kind == "all" || kind == "attitude") append(viz::spatial_attitude_plot(channels));
                if (kind == "all" || kind == "contact") append(viz::spatial_contact_plot(channels));
                if (kind == "all" || kind == "energy") append(viz::spatial_energy_plot(channels));
                if (kind == "path" || (kind == "all" && viz::numeric(channels,"spatial.front_wheel_x_m")))
                    append(viz::spatial_wheel_path_plot(channels));
                if (kind == "imu" || (kind == "all" && viz::numeric(channels,"observations.time_s")))
                    append(viz::spatial_imu_plot(channels));
                if (kind == "all" && !viz::numeric(channels,"spatial.front_wheel_x_m"))
                    report.panels.front().notes.emplace_back("Wheel paths N/A: accepted force-stage axle transforms were not recorded.");
                if (kind == "all" && !viz::numeric(channels,"observations.time_s"))
                    report.panels.front().notes.emplace_back("IMU N/A: no research observations were recorded.");
                viz::write_artifact(report,args.required("output"));
                result = Value{Object{{"output",text(args.required("output"))},{"semantics",text("recorded spatial evidence; no simulator constructed")}}};
            }
            else result = recording::export_data(*reader, integer_argument(args.required("episode"), "episode"), args.get("table", "telemetry"),
                args.required("format"), args.required("output"), args.optional("limit") ? std::optional(maximum) : std::nullopt);
        }
        out << (args.json ? io::recording_json(result) : pretty_json(result)) << '\n';
        if (!out) throw std::runtime_error("recording output stream failed");
        return 0;
    } catch (const ParseError& failure) { error << help << "bike-recording: error: " << failure.what() << '\n'; return 2; }
    catch (const recording::RecordingError& failure) { error << "RECORDING ERROR: RecordingError: " << failure.what() << '\n'; return 1; }
    catch (const std::invalid_argument& failure) { error << "RECORDING ERROR: ValueError: " << failure.what() << '\n'; return 1; }
    catch (const std::exception& failure) { error << "RECORDING ERROR: " << failure.what() << '\n'; return 1; }
}
int recording_command(std::span<const std::string_view> arguments) { return recording_command(arguments, std::cout, std::cerr); }
} // namespace bike::cli
