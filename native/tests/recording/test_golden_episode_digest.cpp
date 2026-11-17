#include "io/json.hpp"
#include "platform/argv.hpp"
#include "platform/process.hpp"
#include "recording/digest.hpp"
#include "recording/reader.hpp"
#include "support/fixture_test.hpp"
#include "support/temporary_directory.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
namespace fs = std::filesystem;
namespace io = bike::io;
namespace rec = bike::recording;

enum class EpisodeGroup : std::uint8_t { all, welded, distributed };

struct EpisodeSpec {
    std::string_view name;
    EpisodeGroup group;
    std::string_view track;
    std::string_view physics_config;
    std::string_view golden_relative;
};

constexpr std::array<EpisodeSpec, 4> kEpisodes{{
    {
        .name = "welded_flat",
        .group = EpisodeGroup::welded,
        .track = "flat",
        .physics_config = "examples/research/viewer_physics_welded.toml",
        .golden_relative = "episode-digest/darwin-arm64/welded_flat.json",
    },
    {
        .name = "welded_rough_uphill_extreme",
        .group = EpisodeGroup::welded,
        .track = "examples/research/rough_uphill_extreme.toml",
        .physics_config = "examples/research/viewer_physics_welded.toml",
        .golden_relative =
            "episode-digest/darwin-arm64/welded_rough_uphill_extreme.json",
    },
    {
        .name = "distributed_flat",
        .group = EpisodeGroup::distributed,
        .track = "flat",
        .physics_config = "examples/research/viewer_physics_distributed.toml",
        .golden_relative = "episode-digest/darwin-arm64/distributed_flat.json",
    },
    {
        .name = "distributed_rough_uphill_extreme",
        .group = EpisodeGroup::distributed,
        .track = "examples/research/rough_uphill_extreme.toml",
        .physics_config = "examples/research/viewer_physics_distributed.toml",
        .golden_relative =
            "episode-digest/darwin-arm64/distributed_rough_uphill_extreme.json",
    },
}};

struct ParsedOptions {
    fs::path bike_ride_bin;
    fs::path repo_root;
    EpisodeGroup filter = EpisodeGroup::all;
    bool help = false;
};

EpisodeGroup parse_episode_filter(std::string_view value) {
    if (value == "all") return EpisodeGroup::all;
    if (value == "welded") return EpisodeGroup::welded;
    if (value == "distributed") return EpisodeGroup::distributed;
    throw std::invalid_argument(
        "unknown --episode filter '" + std::string(value) +
        "' (expected all, welded, or distributed)");
}

ParsedOptions parse_options(std::span<const char *const> raw_args) {
    ParsedOptions out;
    std::vector<std::string_view> positional;
    for (std::size_t i = 1; i < raw_args.size(); ++i) {
        if (raw_args[i] == nullptr) {
            throw std::invalid_argument("null argument at index " +
                                        std::to_string(i));
        }
        const std::string_view arg(raw_args[i]);
        if (arg == "-h" || arg == "--help") {
            out.help = true;
            return out;
        }
        if (arg == "--episode") {
            ++i;
            if (i >= raw_args.size() || raw_args[i] == nullptr) {
                throw std::invalid_argument("--episode requires a value");
            }
            out.filter = parse_episode_filter(raw_args[i]);
            continue;
        }
        if (arg.starts_with("--episode=")) {
            out.filter = parse_episode_filter(arg.substr(10));
            continue;
        }
        if (arg.starts_with('-')) {
            throw std::invalid_argument("unknown option: " + std::string(arg));
        }
        positional.push_back(arg);
    }
    if (positional.size() > 2) {
        throw std::invalid_argument(
            "expected at most 2 positional arguments: [<bike-ride> [<repo-root>]]");
    }
    if (!positional.empty()) {
        out.bike_ride_bin = fs::path(positional[0]);
    } else {
        const fs::path fixture_dir =
            bt::fixture("episode-digest/darwin-arm64/welded_flat.json")
                .parent_path();
        out.repo_root =
            fs::weakly_canonical(fixture_dir / ".." / ".." / ".." / "..");
        out.bike_ride_bin =
            out.repo_root / "build" / "native-release" / "bin" / "bike-ride";
    }
    if (positional.size() >= 2) {
        out.repo_root = fs::path(positional[1]);
    } else if (out.repo_root.empty()) {
        const fs::path fixture_dir =
            bt::fixture("episode-digest/darwin-arm64/welded_flat.json")
                .parent_path();
        out.repo_root =
            fs::weakly_canonical(fixture_dir / ".." / ".." / ".." / "..");
    }
    return out;
}

std::vector<EpisodeSpec> selected_episodes(EpisodeGroup filter) {
    std::vector<EpisodeSpec> out;
    for (const auto &ep : kEpisodes) {
        if (filter == EpisodeGroup::all || ep.group == filter) {
            out.push_back(ep);
        }
    }
    return out;
}

fs::path find_recording_dir(const fs::path &out_dir) {
    std::optional<fs::path> found;
    for (const auto &entry : fs::directory_iterator(out_dir)) {
        if (!entry.is_directory()) continue;
        if (fs::exists(entry.path() / "manifest.json") &&
            fs::exists(entry.path() / "run.duckdb")) {
            if (found.has_value()) {
                throw std::runtime_error("multiple recording directories in " +
                                         out_dir.string());
            }
            found = entry.path();
        }
    }
    if (!found.has_value()) {
        throw std::runtime_error("no sealed recording directory found under " +
                                 out_dir.string());
    }
    return *found;
}

struct RideRunSpec {
    fs::path bike_ride_bin;
    fs::path repo_root;
    EpisodeSpec episode;
    fs::path episode_tmp;
};

fs::path run_headless_bike_ride(const RideRunSpec &run_spec) {
    const fs::path cache_dir = run_spec.episode_tmp / "cache";
    const fs::path out_dir = run_spec.episode_tmp / "out";
    fs::create_directories(cache_dir);
    fs::create_directories(out_dir);

    const std::string track_arg =
        run_spec.episode.track == "flat"
            ? std::string("flat")
            : (run_spec.repo_root / run_spec.episode.track).string();
    const std::string phys_arg =
        (run_spec.repo_root / run_spec.episode.physics_config).string();
    const std::string cache_env =
        "BIKE_SIM_EQUILIBRIUM_CACHE_DIR=" + cache_dir.string();

    bike::platform::ProcessOptions options;
    options.timeout_s = 600.0;
    const std::vector<std::string> argv{
        "/usr/bin/env",
        cache_env,
        run_spec.bike_ride_bin.string(),
        "--physics-config",
        phys_arg,
        "--track",
        track_arg,
        "--headless",
        "--duration",
        "30",
        "--check-policy",
        "diagnostic",
        "--record",
        "--out",
        out_dir.string(),
        "--no-plots",
    };
    const auto result =
        bike::platform::run(argv, run_spec.repo_root, options);
    bt::require(result.exit_code == 0 || result.exit_code == 2,
                std::string(run_spec.episode.name) +
                    " bike-ride failed with exit code " +
                    std::to_string(result.exit_code) + ": " + result.err);
    return find_recording_dir(out_dir);
}

void verify_episodes(const ParsedOptions &opts) {
    bt::require(fs::exists(opts.bike_ride_bin),
                "bike-ride binary not found: " + opts.bike_ride_bin.string());
    bt::require(fs::is_directory(opts.repo_root),
                "repo root directory not found: " + opts.repo_root.string());

    const bt::TemporaryDirectory tmp("golden-episode-digest");
    for (const auto &ep : selected_episodes(opts.filter)) {
        const fs::path rec_dir = run_headless_bike_ride({
            .bike_ride_bin = opts.bike_ride_bin,
            .repo_root = opts.repo_root,
            .episode = ep,
            .episode_tmp = tmp.path() / ep.name,
        });
        const auto recording = rec::open_recording(rec_dir);
        const auto live = rec::compute_recording_digest(*recording);
        const auto golden_path = bt::fixture(ep.golden_relative);
        const auto golden =
            rec::digest_from_json(io::parse_json(bt::text(golden_path)));
        const auto diff = rec::compare_recording_digests(golden, live);
        bt::require(diff.equal,
                    std::string(ep.name) + " drift at " + diff.table + "." +
                        diff.column +
                        " interval=" + std::to_string(diff.interval_id) + ": " +
                        diff.detail);
    }
}

} // namespace

int main(int argc, char **argv) {
    try {
        const auto raw = bike::platform::main_arguments(argc, argv);
        const auto opts = parse_options(raw);
        if (opts.help) {
            std::cout << "usage: golden_episode_digest [<bike-ride> [<repo-root>]] "
                         "[--episode all|welded|distributed]\n";
            return 0;
        }
        return bt::run([&] { verify_episodes(opts); });
    } catch (const std::exception &err) {
        std::cerr << "FAIL " << err.what() << '\n';
        return 1;
    }
}
