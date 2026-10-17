#pragma once
#include "policy_session.hpp"
#include <exception>
#include <filesystem>
#include <span>
#include <stdexcept>

namespace bike::research {
class OperatorInterrupted : public std::runtime_error {
public:
    OperatorInterrupted() : std::runtime_error("operator interrupt deferred during research publication") {}
    ~OperatorInterrupted() override;
};
struct FailureReport {
    std::exception_ptr primary;
    std::vector<std::string> secondary;
    void add(std::string_view operation, std::exception_ptr failure) noexcept;
    void note(std::string_view message) noexcept;
    void rethrow_if_failed() const;
};
struct EpisodeReceipt {
    std::int64_t episode_id = 0;
    std::string seal_reason;
    runtime::OwnedResearchEpisode episode;
    io::Value summary;
};
class ResearchPublication {
public:
    virtual ~ResearchPublication();
    // The recording owner implements one exclusive final publication over
    // immutable RAM receipts. An exception leaves receipts available for retry.
    [[nodiscard]] virtual std::filesystem::path publish(std::span<const EpisodeReceipt>,
                                                       std::span<const OperatorEvent>) = 0;
};

void drain_active(PolicySession&);
void drain_active(runtime::NativeResearchRuntime&);
void close_environment(PolicySession&, FailureReport&);

class ResearchRun {
public:
    // Session must outlive this owner. A null publication keeps launch
    // capture disabled and never creates files or persistent launch histories.
    explicit ResearchRun(PolicySession&, std::shared_ptr<ResearchPublication> publication = {});
    // A setup owner can publish zero-step evidence when policy loading fails
    // before a PolicySession exists. The environment must outlive this owner.
    explicit ResearchRun(runtime::NativeResearchRuntime&, std::shared_ptr<ResearchPublication> publication = {});
    ~ResearchRun();
    ResearchRun(const ResearchRun&) = delete;
    ResearchRun& operator=(const ResearchRun&) = delete;
    void reset(std::optional<std::string> seed_decimal = {});
    [[nodiscard]] const EpisodeReceipt* seal(std::string reason);
    [[nodiscard]] std::optional<std::filesystem::path> finish(std::exception_ptr primary = {}, bool stop = false);
    [[nodiscard]] bool finalization_attempted() const noexcept { return finalization_attempted_; }
    [[nodiscard]] const FailureReport& failures() const noexcept { return failures_; }
    [[nodiscard]] const std::vector<EpisodeReceipt>& episodes() const noexcept { return episodes_; }
    [[nodiscard]] const std::vector<OperatorEvent>& events() const noexcept { return events_; }
    void event(std::string kind, io::Value values = io::Value{io::Value::Object{}});
private:
    void before_reset();
    void new_episode();
    void annotate_failure();
    PolicySession* session_ = nullptr;
    runtime::NativeResearchRuntime* environment_;
    std::shared_ptr<ResearchPublication> publication_;
    std::vector<EpisodeReceipt> episodes_;
    std::vector<OperatorEvent> events_;
    std::optional<EpisodeReceipt> prepared_;
    std::optional<std::vector<OperatorEvent>> prepared_events_;
    std::int64_t episode_id_ = 0;
    bool finalization_attempted_ = false, finalizing_ = false, draining_ = false;
    bool failure_event_recorded_ = false;
    std::optional<std::string> final_reason_;
    FailureReport failures_;
    std::optional<std::filesystem::path> path_;
};
} // namespace bike::research
