#include "cli/gpu_export.hpp"

#include "cli/application.hpp"
#include "configuration/load.hpp"
#include "golden/step_closure_bundle.hpp"
#include "io/json.hpp"
#include "platform/argv.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <ostream>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace bike::cli {
namespace {

constexpr std::string_view kHelpText =
    "usage: bike-gpu-export --physics-config TOML --track PRESET_OR_TOML --out DIR "
    "[--duration SEC] [--steps N] [--seed DEC] [--strict] [--verify] [-h|--help]\n\n"
    "Export a self-contained GPU Golden Bundle (model.xml, model_12gears.xml,\n"
    "hfield_data.bin, physics_config.json, step_closure.bin, manifest.json)\n"
    "from a deterministic NativeRideRuntime run.\n";

struct ParseError final : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
    ~ParseError() override;
    ParseError(const ParseError &) = default;
    ParseError &operator=(const ParseError &) = default;
    ParseError(ParseError &&) = default;
    ParseError &operator=(ParseError &&) = default;
};
ParseError::~ParseError() = default;

struct ParsedArgs {
    std::string physics_config;
    std::string track;
    std::string out_dir;
    std::optional<double> duration_s;
    std::optional<std::size_t> max_steps;
    std::optional<std::string> seed;
    bool strict = false;
    bool verify = false;
};

[[nodiscard]] double parse_positive_real(std::string_view text,
                                         const char *option) {
    double value = 0.;
    const auto *begin = text.data();
    const auto *end = std::to_address(text.end());
    const auto parsed = std::from_chars(begin, end, value);
    if (parsed.ec != std::errc{} || parsed.ptr != end ||
        !std::isfinite(value) || value <= 0.) {
        throw ParseError("option --" + std::string(option) +
                         " must be finite and positive");
    }
    return value;
}

[[nodiscard]] std::size_t parse_positive_steps(std::string_view text) {
    const std::size_t max_steps_bound =
        golden::max_step_closure_steps(1U, 1U, 0U, 0U, 1U);
    std::size_t value = 0;
    const auto *begin = text.data();
    const auto *end = std::to_address(text.end());
    const auto parsed = std::from_chars(begin, end, value);
    if (parsed.ec != std::errc{} || parsed.ptr != end || value == 0 ||
        value > max_steps_bound) {
        throw ParseError("option --steps must be an integer in [1, " +
                         std::to_string(max_steps_bound) + "]");
    }
    return value;
}

[[nodiscard]] ParsedArgs parse_args(std::span<const std::string_view> args) {
    static const std::set<std::string, std::less<>> kValueOptions{
        "physics-config", "track", "out", "duration", "steps", "seed",
    };
    static const std::set<std::string, std::less<>> kFlags{
        "strict", "verify",
    };
    std::map<std::string, std::string, std::less<>> values;
    std::set<std::string, std::less<>> flags;

    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string_view token = args[i];
        if (!token.starts_with("--")) {
            throw ParseError("unexpected argument: " + std::string(token));
        }
        const auto eq = token.find('=');
        const std::string_view key =
            token.substr(2, eq == std::string_view::npos ? eq : eq - 2);
        if (kFlags.contains(key)) {
            if (eq != std::string_view::npos) {
                throw ParseError("flag --" + std::string(key) +
                                 " does not take a value");
            }
            if (!flags.emplace(key).second) {
                throw ParseError("duplicate flag: --" + std::string(key));
            }
            continue;
        }
        if (!kValueOptions.contains(key)) {
            throw ParseError("unknown option: --" + std::string(key));
        }
        std::string value;
        if (eq == std::string_view::npos) {
            ++i;
            if (i >= args.size() || args[i].starts_with("--")) {
                throw ParseError("option --" + std::string(key) +
                                 " requires a value");
            }
            value = std::string(args[i]);
        } else {
            value = std::string(token.substr(eq + 1));
        }
        if (value.empty()) {
            throw ParseError("option --" + std::string(key) +
                             " cannot be empty");
        }
        if (!values.emplace(std::string(key), std::move(value)).second) {
            throw ParseError("duplicate option: --" + std::string(key));
        }
    }

    const auto require_value = [&](std::string_view name) -> std::string {
        const auto it = values.find(name);
        if (it == values.end()) {
            throw ParseError("the following argument is required: --" +
                             std::string(name));
        }
        return it->second;
    };

    ParsedArgs parsed;
    parsed.physics_config = require_value("physics-config");
    parsed.track = require_value("track");
    parsed.out_dir = require_value("out");
    if (const auto it = values.find("duration"); it != values.end()) {
        parsed.duration_s = parse_positive_real(it->second, "duration");
    }
    if (const auto it = values.find("steps"); it != values.end()) {
        parsed.max_steps = parse_positive_steps(it->second);
    }
    if (const auto it = values.find("seed"); it != values.end()) {
        parsed.seed = it->second;
    }
    parsed.strict = flags.contains("strict");
    parsed.verify = flags.contains("verify");
    return parsed;
}

} // namespace

int run_gpu_export(int argc, const char *const *argv,
                   std::ostream &out, std::ostream &err) {
    try {
        if (argc <= 0 || argv == nullptr) {
            err << kHelpText
                << "bike-gpu-export: error: invalid argument vector\n";
            return 2;
        }
        const auto raw = platform::main_arguments(argc, argv);
        std::vector<std::string_view> args;
        if (raw.size() > 1) {
            args.reserve(raw.size() - 1U);
            for (std::size_t i = 1; i < raw.size(); ++i) {
                if (raw[i] == nullptr) {
                    throw ParseError("null argument at index " +
                                     std::to_string(i));
                }
                args.emplace_back(raw[i]);
            }
        }

        if (std::ranges::any_of(args, [](std::string_view arg) {
                return arg == "-h" || arg == "--help";
            })) {
            out << kHelpText;
            return out ? 0 : 2;
        }

        const ParsedArgs parsed = parse_args(args);
        const std::filesystem::path physics_path(parsed.physics_config);
        const golden::StepClosureCaptureSpec spec{
            .physics = configuration::load_physics(physics_path),
            .track = resolve_track(parsed.track, parsed.seed),
            .duration_s = parsed.duration_s,
            .max_steps = parsed.max_steps,
            .seed_decimal = parsed.seed.value_or("0"),
            .physics_config_label = parsed.physics_config,
            .track_label = parsed.track,
            .asset_root = std::filesystem::absolute(physics_path).parent_path(),
            .strict = parsed.strict,
        };
        const auto bundle = golden::capture_step_closure_bundle(spec);
        const std::filesystem::path out_dir(parsed.out_dir);
        golden::write_step_closure_bundle(*bundle, out_dir);

        if (parsed.verify) {
            const auto v = golden::verify_bundle_closure(out_dir);
            if (!v.ok()) {
                err << "bike-gpu-export: closure verification failed at step "
                    << v.first_fail_step << " (max_qpos_err=" << v.max_qpos_err
                    << ", max_qvel_err=" << v.max_qvel_err << ")\n";
                return 1;
            }
        }

        io::Value::Object summary{
            {"bundle", io::Value{out_dir.string()}},
            {"steps", io::Value{io::Number{std::to_string(bundle->rows.size())}}},
            {"nq", io::Value{io::Number{std::to_string(bundle->nq)}}},
            {"nv", io::Value{io::Number{std::to_string(bundle->nv)}}},
            {"nu", io::Value{io::Number{std::to_string(bundle->nu)}}},
            {"ntendon", io::Value{io::Number{std::to_string(bundle->ntendon)}}},
        };
        out << io::canonical_json(io::Value{std::move(summary)}) << '\n';
        if (!out) {
            err << "bike-gpu-export: output stream write failed\n";
            return 2;
        }
        return 0;
    } catch (const ParseError &failure) {
        err << kHelpText << "bike-gpu-export: error: " << failure.what()
            << '\n';
        return 2;
    } catch (const std::exception &failure) {
        err << "bike-gpu-export: error: " << failure.what() << '\n';
        return 1;
    }
}

} // namespace bike::cli
