#include "recording_golden.hpp"
#include "../golden/episode.hpp"
#include "../golden/spatial_registry.hpp"
#include "../golden/spatial_compare.hpp"
#include "../runtime/setup_values.hpp"
#include "../platform/files.hpp"
#include "../platform/provenance.hpp"
#include "../io/npy.hpp"
#include "../research/environment.hpp"
#include "../runtime/research.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <filesystem>
#include <span>
#include <ostream>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>

namespace bike::cli {
namespace {
constexpr std::string_view help =
    "usage: bike-recording golden freeze --request FILE --steps N --out DIR [--human-torque NM]\n"
    "       bike-recording golden compare --golden DIR --request FILE [--rtol R --atol A]\n\n"
    "Freeze direct physical steps or replay all recorded applied controls.\n"
    "freeze uses 35 N m human torque by default; existing output paths are refused.\n"
    "compare defaults to rtol=1e-12 and atol=1e-12. No Python is invoked.\n"
    "Spatial operations require --spatial-manifest FILE --test spatial.NAME.\n"
    "Spatial freeze also requires --accept-reviewed; --out is the golden root.\n"
    "Spatial compare is bitwise within its platform; pending fixtures return 3.\n";
struct ParseError final : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
    ~ParseError() override;
    ParseError(const ParseError &) = default;
    ParseError &operator=(const ParseError &) = default;
    ParseError(ParseError &&) = default;
    ParseError &operator=(ParseError &&) = default;
};
ParseError::~ParseError() = default;
struct Arguments {
    std::string command;
    std::map<std::string, std::string> options;
    std::string required(const std::string &name) const {
        const auto found = options.find(name);
        if (found == options.end() || found->second.empty()) throw ParseError("missing --" + name);
        return found->second;
    }
    std::string get(const std::string &name, const std::string& fallback) const { // NOLINT(bugprone-easily-swappable-parameters) call sites keep the declared parameter order.
        const auto found = options.find(name);
        return found == options.end() ? fallback : found->second;
    }
};
Arguments parse(std::span<const std::string_view> args) {
    if (args.empty() || (args[0] != "freeze" && args[0] != "compare")) throw ParseError("expected freeze or compare");
    Arguments out; out.command = args[0];
    const std::set<std::string> allowed = out.command == "freeze"
        ? std::set<std::string>{"request", "steps", "out", "human-torque", "accept-reviewed", "spatial-manifest", "test"}
        : std::set<std::string>{"request", "golden", "rtol", "atol", "spatial-manifest", "test"};
    for (std::size_t i = 1; i < args.size(); ++i) {
        const auto token = args[i];
        if (!token.starts_with("--")) throw ParseError("unexpected argument " + std::string(token));
        const auto equal = token.find('=');
        const std::string key(token.substr(2, equal == std::string_view::npos ? equal : equal - 2));
        if (!allowed.contains(key)) throw ParseError("unknown option --" + key);
        std::string value;
        if (key=="accept-reviewed") { if(equal!=std::string_view::npos)throw ParseError("--accept-reviewed takes no value"); value="true"; }
        else if (equal == std::string_view::npos) {
            ++i;
            if (i == args.size() || args[i].starts_with("--")) throw ParseError("--" + key + " expects a value");
            value = args[i];
        } else value = token.substr(equal + 1);
        if (!out.options.emplace(key, std::move(value)).second) throw ParseError("duplicate --" + key);
    }
    (void)out.required("request");
    if (out.command == "freeze") { (void)out.required("steps"); (void)out.required("out"); }
    else (void)out.required("golden");
    return out;
}
std::size_t steps(const std::string &text) {
    std::size_t value = 0;
    const auto end = std::to_address(text.end());
    const auto parsed = std::from_chars(text.data(), end, value);
    if (parsed.ec != std::errc{} || parsed.ptr != end || value > 1000000)
        throw ParseError("--steps must be an integer in [0, 1000000]");
    return value;
}
double real(const std::string &text, const char *name) {
    double value = 0.;
    const auto end = std::to_address(text.end());
    const auto parsed = std::from_chars(text.data(), end, value);
    if (parsed.ec != std::errc{} || parsed.ptr != end || !std::isfinite(value) || value < 0.)
        throw ParseError(std::string("--") + name + " must be finite and nonnegative");
    return value;
}
std::string spatial_platform() {
#if defined(__APPLE__) && defined(__aarch64__)
    return "macos-arm64";
#elif defined(__linux__) && defined(__aarch64__)
    return "linux-aarch64";
#else
    throw ParseError("spatial goldens require a qualified macos-arm64 or linux-aarch64 host");
#endif
}
bike::io::Value registry_entry(const Arguments& args) {
    const auto bytes=io::read_numpy_bytes(args.required("spatial-manifest"));
    const auto manifest=io::parse_json(std::string(reinterpret_cast<const char*>(bytes.data()),bytes.size()));
    golden::validate_spatial_manifest(manifest);
    const auto name=args.required("test");
    for(const auto& entry:io::as_array(runtime::setupvalue::field(manifest,"entries"),"entries"))
        if(io::as_string(runtime::setupvalue::field(entry,"test"),"test")==name)return entry;
    throw ParseError("spatial manifest has no selected test "+name);
}
} // namespace
int golden_command(std::span<const std::string_view> arguments, std::ostream &out, std::ostream &error) {
    if (std::ranges::any_of(arguments, [](auto arg) { return arg == "--help" || arg == "-h"; })) { out << help; return 0; }
    try {
        const auto args = parse(arguments);
        auto request=std::make_unique<research::ResearchRequest>(research::read_research_request(args.required("request")));
        const bool spatial=request->setup.model.physics && request->setup.model.physics->spatial &&
            request->setup.model.physics->spatial->mode==bike::spatial::Mode::spatial3d;
        std::optional<io::Value> entry;
        if(spatial) {
            entry=registry_entry(args);
            if(args.command=="freeze" && !args.options.contains("accept-reviewed"))
                throw ParseError("spatial golden freeze requires explicit --accept-reviewed");
            if(args.command=="compare") {
                const auto reference=golden::inspect_spatial_reference(*entry,args.required("golden"),spatial_platform());
                if(reference.pending){out<<"pending: spatial reference has no reviewed platform evidence\n";return 3;}
            }
        }
        if (args.command == "freeze") {
            const auto count = steps(args.required("steps"));
            const auto platform=spatial?spatial_platform():std::string{};
            runtime::RideControl control;
            control.human_torque_nm = real(args.get("human-torque", "35"), "human-torque");
            const auto environment = research::make_environment(std::move(*request));
            const auto episode = golden::capture_episode(*environment, count, control);
            auto destination=std::filesystem::path(args.required("out"));
            if(spatial) {
                destination/=platform;destination/="spatial";
                std::filesystem::create_directories(destination);
                destination/=args.required("test");
            }
            golden::save(episode, destination);
            out << "captured " << episode.rows.size() << " rows from " << count << " physics steps\n";
        } else {
            auto rtol = real(args.get("rtol", "1e-12"), "rtol");
            auto atol = real(args.get("atol", "1e-12"), "atol");
            if(spatial) {
                if(args.options.contains("rtol")||args.options.contains("atol"))throw ParseError("spatial tolerances belong to the reviewed manifest");
                rtol=0.;atol=0.;
            }
            const auto artifact = golden::load_episode(args.required("golden"));
            const auto environment = research::make_environment(std::move(*request));
            const auto mismatches = spatial ? golden::compare_spatial_episode(artifact,*environment,*entry,spatial_platform())
                : golden::compare_episode(artifact, *environment, rtol, atol);
            for (const auto &line : mismatches) out << line << '\n';
            if (!mismatches.empty()) return 1;
            out << "golden episode matches\n";
        }
        if (!out) throw std::runtime_error("golden output stream failed");
        return 0;
    } catch (const ParseError &failure) {
        error << help << "bike-recording golden: " << failure.what() << '\n'; return 2;
    } catch (const std::exception &failure) {
        error << "GOLDEN ERROR: " << failure.what() << '\n'; return 1;
    }
}
} // namespace bike::cli
