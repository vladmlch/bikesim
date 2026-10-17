// A small native-startup executable. The full CLI/viewer/recording migration
// belongs to T38–T51; this target exercises the public prepare/make_ride path.
#include "physical_session.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include "../configuration/load.hpp"
#include "../io/json.hpp"
#include "../legacy/ride.hpp"
#include "../model/rider_geometry.hpp"
#include "../runtime/setup_values.hpp"
#include "../terrain/trackfile.hpp"

namespace {
struct Options {
    std::optional<std::filesystem::path> request, physics, track_file, legacy_source, cache_directory;
    std::optional<std::string> track, rider, seed, legacy_drive, legacy_tyre, legacy_tier, assist_mode;
    std::optional<double> speed_kph, duration_s;
    std::optional<std::int64_t> steps, equilibrium_steps;
    bool fresh = false, strict = true, help = false;
};

template<class T> T number(std::string_view text, std::string_view option) {
    T value{};
    const auto end = std::to_address(text.end());
    const auto parsed = std::from_chars(text.data(), end, value);
    if (text.empty() || parsed.ec != std::errc{} || parsed.ptr != end)
        throw std::invalid_argument(std::string(option) + ": invalid numeric value");
    return value;
}

Options parse(std::span<char*> arguments) {
    Options result;
    for (std::size_t i = 1; i < arguments.size(); ++i) {
        const std::string_view key(arguments[i]);
        auto value = [&]() -> std::string_view {
            if (i + 1 >= arguments.size())
                throw std::invalid_argument(std::string(key) + ": missing value");
            return arguments[++i];
        };
        if (key == "--help" || key == "-h") result.help = true;
        else if (key == "--request") result.request = value();
        else if (key == "--physics-config") result.physics = value();
        else if (key == "--track-config") result.track_file = value();
        else if (key == "--track") result.track = value();
        else if (key == "--rider") result.rider = value();
        else if (key == "--seed") result.seed = value();
        else if (key == "--legacy-drive") result.legacy_drive = value();
        else if (key == "--legacy-tyre") result.legacy_tyre = value();
        else if (key == "--legacy-tyre-tier") result.legacy_tier = value();
        else if (key == "--assist-mode") result.assist_mode = value();
        else if (key == "--initial-speed-kph") result.speed_kph = number<double>(value(), key);
        else if (key == "--duration") result.duration_s = number<double>(value(), key);
        else if (key == "--steps") result.steps = number<std::int64_t>(value(), key);
        else if (key == "--max-equilibrium-steps") result.equilibrium_steps = number<std::int64_t>(value(), key);
        else if (key == "--legacy-source-root") result.legacy_source = value();
        else if (key == "--cache-dir") result.cache_directory = value();
        else if (key == "--fresh") result.fresh = true;
        else if (key == "--no-strict") result.strict = false;
        else throw std::invalid_argument("unknown option '" + std::string(key) + "'");
    }
    if (result.help) return result;
    if (result.request && (result.physics || result.track || result.track_file || result.rider || result.seed))
        throw std::invalid_argument("--request is exclusive with configuration/track/rider/seed inputs");
    if (!result.request && !result.physics)
        throw std::invalid_argument("provide --physics-config FILE or --request FILE");
    if (result.track && result.track_file)
        throw std::invalid_argument("choose either --track or --track-config");
    if (result.steps && result.duration_s)
        throw std::invalid_argument("choose either --steps or --duration");
    if (result.steps && *result.steps < 0)
        throw std::invalid_argument("--steps must be nonnegative");
    if (result.equilibrium_steps && *result.equilibrium_steps <= 0)
        throw std::invalid_argument("--max-equilibrium-steps must be positive");
    if (result.duration_s && (!std::isfinite(*result.duration_s) || *result.duration_s < 0.))
        throw std::invalid_argument("--duration must be finite and nonnegative");
    if (result.speed_kph && !std::isfinite(*result.speed_kph))
        throw std::invalid_argument("--initial-speed-kph must be finite");
    if (result.seed && (result.seed->empty() || !std::ranges::all_of(*result.seed,
            [](char c) { return c >= '0' && c <= '9'; })))
        throw std::invalid_argument("--seed must be a nonnegative decimal integer");
    return result;
}

bike::app::SetupRequest request(const Options& options) {
    bike::app::SetupRequest result;
    if (options.request)
        result = bike::initialization::read_setup_request(*options.request, options.fresh);
    else {
        result.origin = std::filesystem::absolute(*options.physics).string();
        result.asset_root = std::filesystem::absolute(*options.physics).parent_path();
        result.model.physics = bike::configuration::load_physics(*options.physics);
        result.model.mode = "ride";
        result.model.rider = bike::model::rider_from_args(options.rider.value_or(
            result.model.physics->physics_mode == "legacy" ? "seated" : "articulated_planar"));
        result.model.crank_joint = true;
        result.seed_decimal = options.seed.value_or("0");
        result.track = options.track_file
            ? bike::initialization::load_track_for_setup(*options.track_file, result.seed_decimal)
            : bike::terrain::get_preset(options.track.value_or("flat"));
        result.model.field = bike::terrain::HeightFieldSpec::for_track(result.track);
    }
    result.runtime.strict = options.strict;
    result.legacy_source_root = options.legacy_source;
    result.cache_directory = options.cache_directory;
    result.equilibrium.max_steps = options.equilibrium_steps;
    if (options.speed_kph) result.model.physics->initial_speed_mps = *options.speed_kph / 3.6;
    if (options.fresh) {
        result.model.physics->equilibrium_cache_enabled = false;
        result.model.physics->equilibrium_state_file.reset();
        result.initial_state.reset();
    }
    bike::configuration::post_init(*result.model.physics);
    return result;
}

void usage() {
    std::cout <<
        "Native bike ride preparation and bounded execution\n"
        "  bike-native-ride --physics-config FILE [--track-config FILE | --track PRESET]\n"
        "                   [--rider VARIANT] [--seed DECIMAL]\n"
        "  bike-native-ride --request FILE\n"
        "  Common: [--steps N | --duration SECONDS] [--initial-speed-kph VALUE]\n"
        "          [--fresh] [--max-equilibrium-steps N] [--no-strict]\n"
        "          [--legacy-source-root DIR] [--cache-dir DIR]\n"
        "  Legacy mode: [--legacy-drive motor|pedal|pedelec] [--assist-mode MODE]\n"
        "               [--legacy-tyre sphere|pneumatic] [--legacy-tyre-tier TIER]\n"
        "Default duration: 1 second. --steps 0 prepares t=0 only.\n"
        "--fresh requests a new equilibrium, overriding saved state and cache reuse.\n";
}
} // namespace

int main(int argc, char** argv) {
    try {
        const auto options = parse(std::span<char*>{argv, static_cast<std::size_t>(argc)});
        if (options.help) { usage(); return 0; }
        auto setup_request = request(options);
        if (setup_request.model.physics->physics_mode == "legacy") {
            bike::legacy::RideOptions legacy_options;
            legacy_options.drive_mode = options.legacy_drive.value_or("motor");
            legacy_options.assist_mode = options.assist_mode.value_or("tour");
            legacy_options.tyre.model = options.legacy_tyre.value_or("sphere");
            legacy_options.tyre.tier = options.legacy_tier.value_or("fast");
            legacy_options.setup = std::move(setup_request);
            auto ride = bike::legacy::make_ride(std::move(legacy_options));
            std::int64_t steps = options.steps.value_or(0);
            if (!options.steps) {
                const double count = std::ceil(options.duration_s.value_or(1.) / ride->timestep_s());
                if (!std::isfinite(count) || count >= 0x1p63)
                    throw std::invalid_argument("duration exceeds the native step-count domain");
                steps = static_cast<std::int64_t>(count);
            }
            auto limits = ride->default_limits();
            limits.max_steps = steps;
            const auto outcome = steps == 0
                ? bike::legacy::RunOutcome{"target", 0, ride->time_s(), 0., ride->position_m(), std::nullopt}
                : ride->run(limits);
            using namespace runtime::setupvalue;
            const Value result{Object{{"schema", integer(1)}, {"physics_mode", text("legacy")},
                {"step", integer(outcome.steps)}, {"time_s", real(outcome.sim_time_s)},
                {"position_m", real(outcome.position_m)}, {"reason", text(outcome.reason)},
                {"outcome", outcome.crash ? text(outcome.crash->cause) : Value{}}}};
            std::cout << bike::io::canonical_json(result) << '\n';
            return outcome.crash ? 3 : 0;
        }
        if (options.legacy_drive || options.legacy_tyre || options.legacy_tier || options.assist_mode)
            throw std::invalid_argument("legacy drive/tyre/assist options require physics_mode='legacy'");
        const double dt = setup_request.model.physics->timestep_s;
        const double finish_m = setup_request.track.length_m;
        const std::string track = setup_request.track.name;
        std::int64_t steps = options.steps.value_or(0);
        if (!options.steps) {
            const double count = std::ceil(options.duration_s.value_or(1.) / dt);
            if (!std::isfinite(count) || count >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
                throw std::invalid_argument("duration exceeds the native step-count domain");
            steps = static_cast<std::int64_t>(count);
        }
        auto prepared = bike::app::prepare(std::move(setup_request));
        const auto& equilibrium = runtime::setupvalue::field(prepared.metadata, "equilibrium");
        const auto residual = runtime::setupvalue::field(equilibrium, "residual_qacc");
        auto ride = bike::app::make_ride(std::move(prepared));
        ride->set_finish_position(finish_m);
        const auto advanced = ride->advance(steps, runtime::RideControl{}, 0., 0., std::nullopt);
        using namespace runtime::setupvalue;
        const Value result{Object{{"schema", integer(1)}, {"track", text(track)},
            {"step", integer(advanced.step)}, {"time_s", real(advanced.time_s)},
            {"reason", text(advanced.reason)}, {"equilibrium_residual_qacc", residual},
            {"outcome", advanced.outcome ? text(*advanced.outcome) : Value{}}}};
        std::cout << bike::io::canonical_json(result) << '\n';
        ride->close();
        return advanced.outcome && advanced.outcome->starts_with("crash:") ? 3 : 0;
    } catch (const std::exception& error) {
        std::cerr << "bike-native-ride: " << error.what() << '\n';
        return 2;
    }
}
