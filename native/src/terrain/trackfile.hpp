#pragma once
#include "terrain/generators.hpp"
#include "terrain/profile.hpp"
#include <filesystem>
#include <stdexcept>

namespace bike::terrain {
class TrackFileError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
    ~TrackFileError() override;
    TrackFileError(const TrackFileError&) = default;
    TrackFileError& operator=(const TrackFileError&) = default;
    TrackFileError(TrackFileError&&) = default;
    TrackFileError& operator=(TrackFileError&&) = default;
};
[[nodiscard]] Obstacle obstacle_from_dict(const bike::io::Value& entry,std::size_t index=0);
[[nodiscard]] RoadGeneratorSpec generator_from_dict(const bike::io::Value& entry);
[[nodiscard]] std::vector<Obstacle> carve_roughness(std::vector<Obstacle> obstacles);
[[nodiscard]] TrackSpec track_from_dict(const bike::io::Value& data,
    std::optional<std::string> seed=std::nullopt,std::optional<double> length_m=std::nullopt);
// The pinned TOML parser is int64-based. This scoped adapter preserves the
// source's arbitrary-precision integer *seed* values without broadening any
// other numeric field. It also retains authored table insertion order,
// including description mappings, instead of toml++'s sorted iteration.
[[nodiscard]] bike::io::Value parse_track_toml(std::string_view text,std::string_view origin="track");
[[nodiscard]] TrackSpec load_track(const std::filesystem::path& path,
    std::optional<std::string> seed=std::nullopt,std::optional<double> length_m=std::nullopt);
[[nodiscard]] bike::io::Value obstacle_to_dict(const Obstacle& obstacle);
[[nodiscard]] bike::io::Value track_to_dict(const TrackSpec& track);
[[nodiscard]] std::string dump_track(const TrackSpec& track);
[[nodiscard]] std::filesystem::path save_track(const TrackSpec& track,const std::filesystem::path& path);
// Owned, complete native startup/cache identity. This supports every typed
// obstacle including SteppedClimb, which the historical TOML writer rejects.
// It is deliberately separate from the user track-file serialization contract.
[[nodiscard]] bike::io::Value identity_value(const TrackSpec& track);
} // namespace bike::terrain
