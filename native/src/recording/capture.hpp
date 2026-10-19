#pragma once
#include "contracts.hpp"
#include <utility>

namespace bike::recording {
struct Publication;
struct FinalizeOptions;
class RunCapture {
public:
    // NOLINTNEXTLINE(performance-enum-size) enum width matches the reference ABI layout
    enum class State { collecting, publishing, published, publication_failed };
    explicit RunCapture(RunMetadata metadata, bool enabled = true);
    RunCapture(const RunCapture&) = delete;
    RunCapture& operator=(const RunCapture&) = delete;
    RunCapture(RunCapture&&) = delete;
    RunCapture& operator=(RunCapture&&) = delete;
    ~RunCapture() = default;
    [[nodiscard]] const RunMetadata& metadata() const noexcept { return metadata_; }
    [[nodiscard]] bool enabled() const noexcept { return enabled_; }
    [[nodiscard]] State state() const noexcept { return state_; }
    [[nodiscard]] std::string_view state_name() const noexcept;
    void event(InputEvent event);
    void add_episode(const std::shared_ptr<const EpisodeEvidence>& evidence);
    [[nodiscard]] const std::vector<std::shared_ptr<const EpisodeEvidence>>& episodes() const noexcept { return episodes_; }
    [[nodiscard]] const std::vector<InputEvent>& events() const noexcept { return events_; }
private:
    void require_collecting(std::string_view operation) const;
    RunMetadata metadata_;
    bool enabled_;
    State state_ = State::collecting;
    std::vector<std::shared_ptr<const EpisodeEvidence>> episodes_;
    std::vector<std::weak_ptr<const EpisodeEvidence>> episode_origins_;
    std::vector<InputEvent> events_;
    std::optional<std::pair<std::int64_t,std::int64_t>> last_event_;
    std::filesystem::path published_path_;
    std::string published_report_status_;
    friend Publication finalize(RunCapture&, const std::filesystem::path&, const FinalizeOptions&);
};
class EpisodeJournal {
public:
    EpisodeJournal(RunCapture& capture, std::int64_t episode_id);
    [[nodiscard]] std::int64_t next_id() const noexcept { return next_id_; }
    [[nodiscard]] std::int64_t episode_id() const noexcept { return episode_id_; }
    InputEvent emit(std::string kind, double received_wall_s, std::int64_t processed_step,
                    std::optional<std::int64_t> applied_step = {}, io::Value values = io::Value{io::Value::Object{}});
    void emit_all(std::span<const InputEvent> events);
private:
    RunCapture* capture_;
    std::int64_t episode_id_, next_id_ = 0;
};
[[nodiscard]] std::vector<InputEvent> derived_events(const runtime::OwnedNativeEpisode&,
    std::int64_t episode_id, std::int64_t start_event_id, double received_wall_s);
[[nodiscard]] std::int64_t episode_interval_for_time(const runtime::OwnedNativeEpisode&, double time_s);
} // namespace bike::recording
