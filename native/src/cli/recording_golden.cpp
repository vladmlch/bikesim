#include "recording_golden.hpp"
#include "../golden/episode.hpp"
#include "../research/environment.hpp"
#include "../runtime/research.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <map>
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
    "compare defaults to rtol=1e-12 and atol=1e-12. No Python is invoked.\n";
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
    std::string get(const std::string &name, std::string fallback) const {
        const auto found = options.find(name);
        return found == options.end() ? std::move(fallback) : found->second;
    }
};
Arguments parse(std::span<const std::string_view> args) {
    if (args.empty() || (args[0] != "freeze" && args[0] != "compare")) throw ParseError("expected freeze or compare");
    Arguments out; out.command = args[0];
    const std::set<std::string> allowed = out.command == "freeze"
        ? std::set<std::string>{"request", "steps", "out", "human-torque"}
        : std::set<std::string>{"request", "golden", "rtol", "atol"};
    for (std::size_t i = 1; i < args.size(); ++i) {
        const auto token = args[i];
        if (!token.starts_with("--")) throw ParseError("unexpected argument " + std::string(token));
        const auto equal = token.find('=');
        const std::string key(token.substr(2, equal == std::string_view::npos ? equal : equal - 2));
        if (!allowed.contains(key)) throw ParseError("unknown option --" + key);
        std::string value;
        if (equal == std::string_view::npos) {
            if (++i == args.size() || args[i].starts_with("--")) throw ParseError("--" + key + " expects a value");
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
    const auto end = text.data() + text.size();
    const auto parsed = std::from_chars(text.data(), end, value);
    if (parsed.ec != std::errc{} || parsed.ptr != end || value > 1000000)
        throw ParseError("--steps must be an integer in [0, 1000000]");
    return value;
}
double real(const std::string &text, const char *name) {
    double value = 0.;
    const auto end = text.data() + text.size();
    const auto parsed = std::from_chars(text.data(), end, value);
    if (parsed.ec != std::errc{} || parsed.ptr != end || !std::isfinite(value) || value < 0.)
        throw ParseError(std::string("--") + name + " must be finite and nonnegative");
    return value;
}
} // namespace
int golden_command(std::span<const std::string_view> arguments, std::ostream &out, std::ostream &error) {
    if (std::ranges::any_of(arguments, [](auto arg) { return arg == "--help" || arg == "-h"; })) { out << help; return 0; }
    try {
        const auto args = parse(arguments);
        if (args.command == "freeze") {
            const auto count = steps(args.required("steps"));
            runtime::RideControl control;
            control.human_torque_nm = real(args.get("human-torque", "35"), "human-torque");
            const auto environment = research::make_environment(args.required("request"));
            const auto episode = golden::capture_episode(*environment, count, control);
            golden::save(episode, args.required("out"));
            out << "captured " << episode.rows.size() << " rows from " << count << " physics steps\n";
        } else {
            const auto rtol = real(args.get("rtol", "1e-12"), "rtol");
            const auto atol = real(args.get("atol", "1e-12"), "atol");
            const auto artifact = golden::load_episode(args.required("golden"));
            const auto environment = research::make_environment(args.required("request"));
            const auto mismatches = golden::compare_episode(artifact, *environment, rtol, atol);
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
