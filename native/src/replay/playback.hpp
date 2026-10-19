#pragma once
#include "input.hpp"
#include "../model/builder.hpp"
#include "../runtime/runtime.hpp"
#include <memory>

namespace bike::replay {
// Stored incoming states are presentation evidence. No controller, policy,
// replay qualification or integration loop is involved in this surface.
struct PlaybackData {
    std::vector<double> times_s, ends_s;
    std::vector<std::vector<double>> qpos, qvel;
    std::vector<std::size_t> quaternion_starts;
    runtime::ModelImage model;
    io::Value summary;
    std::int64_t episode_id = 0;
    double duration_s = 0.;
    bool intervals_hash_verified = true;
};
void validate_playback(const PlaybackData&, std::size_t nq, std::size_t nv);
[[nodiscard]] PlaybackData load_playback(const std::filesystem::path&, std::optional<std::int64_t> episode = {});
[[nodiscard]] std::vector<double> pose_at(const PlaybackData&, double time_s);
[[nodiscard]] model::ModelOwner make_playback_model(const PlaybackData&);

class PlaybackSession {
public:
    explicit PlaybackSession(const std::filesystem::path&, std::optional<std::int64_t> episode = {});
    explicit PlaybackSession(PlaybackData);
    ~PlaybackSession();
    PlaybackSession(const PlaybackSession&) = delete;
    PlaybackSession& operator=(const PlaybackSession&) = delete;
    PlaybackSession(PlaybackSession&&) = delete;
    PlaybackSession& operator=(PlaybackSession&&) = delete;
    [[nodiscard]] runtime::RenderFrame frame(double time_s, int generation = 0);
    [[nodiscard]] const runtime::ModelImage& model_image() const noexcept;
    [[nodiscard]] double duration_s() const noexcept;
    [[nodiscard]] const PlaybackData& data() const noexcept;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace bike::replay
