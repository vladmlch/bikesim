#include "async_close.hpp"
#include <limits>
#include <utility>

namespace runtime {
AsyncWorkerLost::~AsyncWorkerLost() = default;
AsyncPublicationError::~AsyncPublicationError() = default;
AsyncCloseRunner::AsyncCloseRunner(AccountingConfig config, AccountingState state,
    int generation, std::optional<EffortContext> effort, BeforeClose before)
    : before_close_(std::move(before)) {
    // A worker retains only its current accounting state and an unacknowledged
    // result. Growing episode history belongs exclusively to the main owner.
    config.async_telemetry = false;
    config.retention = RetentionMode::off;
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) boundary code holds a non-owning handle by design
    evaluator_.reset(new PeriodAccounting(config, std::move(state),
                                         generation, std::move(effort)));
    worker_ = std::thread([this] { work(); });
}
AsyncCloseRunner::~AsyncCloseRunner() { close(); }

void AsyncCloseRunner::submit(std::shared_ptr<const std::vector<RawStep>> raws) {
    if (!raws || raws->empty()) throw std::invalid_argument("async close requires an owned nonempty period");
    std::scoped_lock const lock(mutex_);
    if (stopping_ || finished_ || failure_)
        throw AsyncWorkerLost("async close worker is unavailable");
    if (sequence_ == std::numeric_limits<std::uint64_t>::max())
        throw std::overflow_error("async close sequence overflow");
    Job job{.sequence=sequence_, .raws=std::move(raws)};
    pending_.push_back(job);
    try { jobs_.push_back(std::move(job)); }
    catch (...) { pending_.pop_back(); throw; }
    ++sequence_;
    changed_.notify_all();
}

std::shared_ptr<const AsyncCloseRunner::Result> AsyncCloseRunner::peek(bool block) {
    std::unique_lock lock(mutex_);
    if (pending_.empty()) return {};
    if (block) changed_.wait(lock, [this] { return !results_.empty() || failure_ || finished_; });
    if (!results_.empty()) {
        if (results_.front()->sequence != pending_.front().sequence)
            throw AsyncWorkerLost("async close returned a result out of submission order");
        return results_.front();
    }
    if (failure_) {
        if (worker_lost_) throw AsyncWorkerLost("async close worker lost its publication context");
        try { std::rethrow_exception(failure_); }
        catch (const std::exception& error) {
            std::throw_with_nested(AsyncPublicationError(std::string("async close worker failed: ") + error.what()));
        } catch (...) {
            std::throw_with_nested(AsyncPublicationError("async close worker failed with a nonstandard exception"));
        }
    }
    if (finished_) throw AsyncWorkerLost("async close worker exited with unacknowledged periods");
    return {};
}

void AsyncCloseRunner::acknowledge(std::uint64_t sequence) {
    std::scoped_lock const lock(mutex_);
    if (pending_.empty() || results_.empty() ||
        pending_.front().sequence != sequence || results_.front()->sequence != sequence)
        throw std::invalid_argument("async close acknowledgement is not the ready prefix");
    results_.pop_front();
    pending_.pop_front();
}
std::size_t AsyncCloseRunner::pending() const {
    std::scoped_lock const lock(mutex_);
    return pending_.size();
}
std::size_t AsyncCloseRunner::pending_intervals() const {
    std::scoped_lock const lock(mutex_);
    std::size_t count = 0;
    for (const auto& item : pending_) count += item.raws->size();
    return count;
}

std::vector<std::shared_ptr<const std::vector<RawStep>>> AsyncCloseRunner::detach_pending() {
    close();
    std::scoped_lock const lock(mutex_);
    std::vector<std::shared_ptr<const std::vector<RawStep>>> result;
    result.reserve(pending_.size());
    for (const auto& item : pending_) result.push_back(item.raws);
    // Clear only after all allocations succeeded. Replaying these originals
    // from the caller's last committed state cannot double-count a period.
    pending_.clear();
    jobs_.clear();
    results_.clear();
    return result;
}

void AsyncCloseRunner::close() noexcept {
    {
        std::scoped_lock const lock(mutex_);
        stopping_ = true;
        changed_.notify_all();
    }
    if (worker_.joinable()) worker_.join();
}

void AsyncCloseRunner::work() noexcept {
    try {
        while (true) {
            Job job;
            {
                std::unique_lock lock(mutex_);
                changed_.wait(lock, [this] { return stopping_ || !jobs_.empty(); });
                if (stopping_) break;
                job = std::move(jobs_.front());
                jobs_.pop_front();
            }
            try {
                if (before_close_) before_close_(job.sequence);
                for (const auto& raw : *job.raws) evaluator_->push(raw);
                std::optional<std::string> strict_error;
                try { evaluator_->flush_local(); }
                catch (const InvalidReferenceRun& error) { strict_error = error.what(); }
                auto result = std::make_shared<Result>(Result{
                    .sequence=job.sequence, .publication=evaluator_->take_publication()});
                result->publication.strict_error = std::move(strict_error);
                {
                    std::scoped_lock const lock(mutex_);
                    results_.push_back(std::move(result));
                    changed_.notify_all();
                }
            } catch (...) {
                std::scoped_lock const lock(mutex_);
                failure_ = std::current_exception();
                changed_.notify_all();
                break;
            }
        }
    } catch (...) {
        std::scoped_lock const lock(mutex_);
        failure_ = std::current_exception();
        worker_lost_ = true;
    }
    std::scoped_lock const lock(mutex_);
    finished_ = true;
    changed_.notify_all();
}
} // namespace runtime
