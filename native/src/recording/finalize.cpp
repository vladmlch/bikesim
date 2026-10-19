#include "publish.hpp"

namespace bike::recording {
Publication finalize(RunCapture& capture, const std::filesystem::path& root, const FinalizeOptions& options) {
    if (!capture.enabled_) {
        if (capture.state_ == RunCapture::State::collecting) capture.state_ = RunCapture::State::published;
        return {.directory={}, .report_status="disabled"};
    }
    if (capture.state_ == RunCapture::State::published) return {.directory=capture.published_path_, .report_status=capture.published_report_status_};
    if (capture.state_ == RunCapture::State::publishing) throw std::logic_error("finalize re-entered during publication");
    capture.state_ = RunCapture::State::publishing;
    try {
        auto result = publish(capture, root, options);
        // Move the returned path/status before marking publication complete.
        // Copying to the caller after this commit can be retried idempotently.
        capture.published_path_ = std::move(result.directory);
        capture.published_report_status_ = std::move(result.report_status);
        capture.state_ = RunCapture::State::published;
    } catch (...) { capture.state_ = RunCapture::State::publication_failed; throw; }
    return {.directory=capture.published_path_, .report_status=capture.published_report_status_};
}
Publication finalize(RunCapture& capture, const std::filesystem::path& root) { return finalize(capture, root, FinalizeOptions{}); }
} // namespace bike::recording
