#include "cli/recording_digest.hpp"

#include "io/json.hpp"
#include "platform/argv.hpp"
#include "recording/analysis_detail.hpp"
#include "recording/digest.hpp"
#include "recording/reader.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
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
    "usage: bike-recording-digest --run PATH [--out PATH] [--verify PATH] [-h|--help]\n\n"
    "Compute or verify a deterministic Schema v1 SHA-256 digest over physics and forces tables.\n";

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
    std::string run;
    std::optional<std::string> out_path;
    std::optional<std::string> verify_path;
};

ParsedArgs parse_args(std::span<const std::string_view> args) {
    static const std::set<std::string, std::less<>> kAllowed{"run", "out", "verify"};
    std::map<std::string, std::string, std::less<>> options;

    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string_view token = args[i];
        if (!token.starts_with("--")) {
            throw ParseError("unexpected argument: " + std::string(token));
        }
        const auto eq = token.find('=');
        const std::string_view key =
            token.substr(2, eq == std::string_view::npos ? eq : eq - 2);
        if (!kAllowed.contains(key)) {
            throw ParseError("unknown option: --" + std::string(key));
        }
        std::string value;
        if (eq == std::string_view::npos) {
            ++i;
            if (i >= args.size() || args[i].starts_with("--")) {
                throw ParseError("option --" + std::string(key) + " requires a value");
            }
            value = std::string(args[i]);
        } else {
            value = std::string(token.substr(eq + 1));
        }
        if (value.empty()) {
            throw ParseError("option --" + std::string(key) + " cannot be empty");
        }
        if (!options.emplace(std::string(key), std::move(value)).second) {
            throw ParseError("duplicate option: --" + std::string(key));
        }
    }

    const auto run_it = options.find("run");
    if (run_it == options.end()) {
        throw ParseError("the following argument is required: --run");
    }

    ParsedArgs parsed;
    parsed.run = run_it->second;
    if (const auto it = options.find("out"); it != options.end()) {
        parsed.out_path = it->second;
    }
    if (const auto it = options.find("verify"); it != options.end()) {
        parsed.verify_path = it->second;
    }
    return parsed;
}

std::filesystem::path normalize_run_path(std::string_view raw) {
    std::filesystem::path path(raw);
    if (path.filename() == "run.duckdb") {
        path = path.parent_path();
        if (path.empty()) path = ".";
    }
    return path;
}

} // namespace

int run_recording_digest(int argc, const char *const *argv,
                         std::ostream &out, std::ostream &err) {
    try {
        if (argc <= 0 || argv == nullptr) {
            err << kHelpText << "bike-recording-digest: error: invalid argument vector\n";
            return 2;
        }
        const auto raw = platform::main_arguments(argc, argv);
        std::vector<std::string_view> args;
        if (raw.size() > 1) {
            args.reserve(raw.size() - 1);
            for (std::size_t i = 1; i < raw.size(); ++i) {
                if (raw[i] == nullptr) {
                    throw ParseError("null argument at index " + std::to_string(i));
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
        const std::filesystem::path run_dir = normalize_run_path(parsed.run);
        const auto rec = recording::open_recording(run_dir);
        const recording::DigestReport report = recording::compute_recording_digest(*rec);
        const std::string json_text = io::canonical_json(recording::digest_to_json(report));

        if (parsed.out_path) {
            std::ofstream file(*parsed.out_path, std::ios::binary | std::ios::trunc);
            if (!file.is_open()) {
                err << "bike-recording-digest: cannot open output file: " << *parsed.out_path
                    << '\n';
                return 2;
            }
            file << json_text << '\n';
            file.flush();
            if (!file.good()) {
                err << "bike-recording-digest: failed writing output file: " << *parsed.out_path
                    << '\n';
                return 2;
            }
        }

        if (parsed.verify_path) {
            const std::string golden_text = recording::detail::read_text(*parsed.verify_path);
            const recording::DigestReport golden =
                recording::digest_from_json(io::parse_json(golden_text));
            const recording::DigestDiff diff =
                recording::compare_recording_digests(golden, report);
            if (!diff.equal) {
                err << "DIGEST MISMATCH: table=" << diff.table << " column=" << diff.column
                    << " interval_id=" << diff.interval_id << " detail=" << diff.detail << '\n';
                return 1;
            }
            if (!parsed.out_path) {
                out << "DIGEST OK: " << report.overall_sha256 << '\n';
            }
        } else if (!parsed.out_path) {
            out << json_text << '\n';
        }

        if (!out) {
            err << "bike-recording-digest: output stream write failed\n";
            return 2;
        }
        return 0;
    } catch (const ParseError &failure) {
        err << kHelpText << "bike-recording-digest: error: " << failure.what() << '\n';
        return 2;
    } catch (const std::exception &failure) {
        err << "bike-recording-digest: error: " << failure.what() << '\n';
        return 2;
    }
}

} // namespace bike::cli
