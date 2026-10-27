#include "research_publication.hpp"
#include "value_util.hpp"
#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace bike::recording {
namespace {
class ResearchRecording final : public research::ResearchPublication {
public:
    ~ResearchRecording() override;
    ResearchRecording(const ResearchRecording&) = delete;
    ResearchRecording& operator=(const ResearchRecording&) = delete;
    ResearchRecording(ResearchRecording&&) = delete;
    ResearchRecording& operator=(ResearchRecording&&) = delete;
    ResearchRecording(RunMetadata metadata, Catalog catalog, std::filesystem::path root, FinalizeOptions options)
        : metadata_(std::move(metadata)), catalog_(std::move(catalog)), root_(std::move(root)), options_(std::move(options)) { metadata_.validate(); }
    std::filesystem::path publish(std::span<const research::EpisodeReceipt> receipts,
                                  std::span<const research::OperatorEvent> events) override {
        if (!capture_) {
            // Stage the entire adaptation transaction before retaining it: a
            // source/validation/allocation failure can retry without duplicates.
            auto staged = std::make_unique<RunCapture>(metadata_, true);
            for (const auto& receipt : receipts) {
                auto summary = io::as_object(receipt.summary, "research summary");
                value::set(summary, "input_identity_at_finalization", input_identity_at_finalization(receipt.summary));
                const auto evidence = make_research_evidence(receipt.episode_id, receipt.episode, catalog_, io::Value{std::move(summary)});
                staged->add_episode(evidence); EpisodeJournal journal(*staged, receipt.episode_id);
                std::vector<const research::OperatorEvent*> tail;
                double seal_wall = metadata_.wall_origin_s;
                const auto append_event = [&](const research::OperatorEvent& e) {
                    auto values = io::as_object(e.values, "operator event values");
                    value::set(values, "time_s", value::real(e.time_s));
                    if (e.queued) value::set(values, "queued", io::Value{true});
                    if (e.terminal) value::set(values, "terminal", io::Value{true});
                    journal.emit(e.kind, e.received_wall_s, e.processed_step,
                        e.applied ? std::optional<std::int64_t>(e.processed_step) : std::nullopt, io::Value{std::move(values)});
                };
                for (const auto& e : events) if (e.episode_id == receipt.episode_id) {
                    seal_wall = std::max(seal_wall, e.received_wall_s);
                    if (e.kind == "episode_ended" || e.kind == "model_first_failure") tail.push_back(&e);
                    else append_event(e);
                }
                journal.emit_all(derived_events(receipt.episode.physical, receipt.episode_id, journal.next_id(), seal_wall));
                for (const auto* e : tail) append_event(*e);
            }
            capture_ = std::move(staged);
        }
        return finalize(*capture_, root_, options_).directory;
    }
private:
    RunMetadata metadata_;
    Catalog catalog_;
    std::filesystem::path root_;
    FinalizeOptions options_;
    std::unique_ptr<RunCapture> capture_;
};
ResearchRecording::~ResearchRecording() = default;
} // namespace
std::shared_ptr<research::ResearchPublication> make_research_publication(RunMetadata metadata, Catalog catalog,
    std::filesystem::path root, FinalizeOptions options) {
    return std::make_shared<ResearchRecording>(std::move(metadata), std::move(catalog), std::move(root), std::move(options));
}
} // namespace bike::recording
