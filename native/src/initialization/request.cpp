#include "setup_owner.hpp"
#include "../diag.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <ranges>
#include <stdexcept>
#include "../configuration/load.hpp"
#include "../io/json.hpp"
#include "../io/npy.hpp"
#include "../runtime/setup_values.hpp"
#include "../terrain/trackfile.hpp"

namespace bike::initialization {
namespace {
using namespace runtime::setupvalue;

std::filesystem::path path(const Value& document, std::string_view key,
                          const std::filesystem::path& base) {
    const auto& name = io::as_string(field(document, key), key);
    if (name.empty() || name.find('\0') != std::string::npos)
        throw std::invalid_argument(std::string(key) + ": expected a nonempty path");
    const std::filesystem::path file(name);
    return std::filesystem::weakly_canonical(file.is_absolute() ? file : base / file);
}
}

terrain::TrackSpec load_track_for_setup(const std::filesystem::path& file,
                                      std::string_view seed_decimal) {
    if (seed_decimal.empty() || !std::ranges::all_of(seed_decimal,
            [](char c) { return c >= '0' && c <= '9'; }))
        throw std::invalid_argument("seed_decimal must be a nonnegative decimal integer string");
    const auto bytes = io::read_numpy_bytes(file);
    std::string source;
    source.reserve(bytes.size());
    for (auto const byte : bytes)
        source.push_back(static_cast<char>(std::to_integer<unsigned char>(byte)));
    const auto document = terrain::parse_track_toml(source, file.string());
    const std::optional<std::string> override = find(document, "generator")
        ? std::optional<std::string>{seed_decimal} : std::nullopt;
    return terrain::track_from_dict(document, override);
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
SetupRequest read_setup_request(const std::filesystem::path& file, bool skip_initial_state) {
    try {
        const auto document = configuration::detail::parse_toml_file(file);
        constexpr auto keys = std::to_array<std::string_view>({"request_schema", "physics_file",
            "track_file", "model_mode", "rider_variant", "speed_kph", "seed_decimal",
            "use_equilibrium_cache", "initial_state_file", "overrides"});
        for (const auto& [name, ignored] : io::as_object(document, "request")) {
            (void)ignored;
            if (std::ranges::find(keys, name) == keys.end())
                throw std::invalid_argument("unknown request field '" + name + "'");
        }
        if (io::as_int64(field(document, "request_schema"), "request_schema") != 1)
            throw std::invalid_argument("unsupported request_schema");
        const auto base = std::filesystem::absolute(file).parent_path();
        auto physics = configuration::load_physics(path(document, "physics_file", base));
        if (const auto* overrides = find(document, "overrides")) {
            auto owned = *overrides;
            configuration::detail::resolve_config_paths(owned, base);
            physics = configuration::resolve_physics(owned, physics);
        }
        SetupRequest out;
        out.origin = std::filesystem::absolute(file).string();
        out.asset_root = base;
        out.seed_decimal = io::as_string(field(document, "seed_decimal"), "seed_decimal");
        if (out.seed_decimal.empty() || !std::ranges::all_of(out.seed_decimal,
                [](char c) { return c >= '0' && c <= '9'; }))
            throw std::invalid_argument("seed_decimal must be a nonnegative decimal integer string");
        out.track = load_track_for_setup(path(document, "track_file", base), out.seed_decimal);
        out.model.field = terrain::HeightFieldSpec::for_track(out.track);
        out.model.mode = io::as_string(field(document, "model_mode"), "model_mode");
        if (out.model.mode != "ride")
            throw std::invalid_argument("native ride preparation requires model_mode='ride'");
        out.model.rider = model::rider_from_args(
            io::as_string(field(document, "rider_variant"), "rider_variant"));
        const double speed = io::as_double(field(document, "speed_kph"), "speed_kph");
        if (!std::isfinite(speed)) throw std::invalid_argument("speed_kph must be finite");
        physics.initial_speed_mps = speed / 3.6;
        physics.equilibrium_cache_enabled =
            io::as_bool(field(document, "use_equilibrium_cache"), "use_equilibrium_cache");
        if (!skip_initial_state && find(document, "initial_state_file")) {
            const auto state_file = path(document, "initial_state_file", base);
            const auto bytes = io::read_numpy_bytes(state_file);
            std::string source;
            source.reserve(bytes.size());
            for (auto const byte : bytes) source.push_back(static_cast<char>(std::to_integer<unsigned char>(byte)));
            const auto first = source.find_first_not_of(" \t\r\n");
            bool transfer = false;
            if (first != std::string::npos && source[first] == '{') {
                const auto state = io::parse_json(source);
                transfer = find(state, "contract_sha256") != nullptr;
            }
            if (transfer) {
                out.initial_state.emplace(std::move(source));
                physics.equilibrium_state_file.reset();
            } else physics.equilibrium_state_file = state_file.string();
        }
        configuration::post_init(physics);
        out.model.physics = std::move(physics);
        out.model.crank_joint = true;
        return out;
    } catch (const std::invalid_argument& error) {
        throw std::invalid_argument(file.string() + ": " + error.what());
    }
}

} // namespace bike::initialization
NATIVE_DIAG_POP
