#include "runtime/spatial_worker_pool.hpp"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace runtime::spatial {

struct WorkerPool::Impl {
    explicit Impl(unsigned count) : count_(count), ranges_(count), statuses_(count, 0) {
        if (count == 0) throw std::invalid_argument("WorkerPool.threads: must be positive");
        workers_.reserve(count - 1U);
        try {
            for (unsigned i = 1; i < count; ++i)
                workers_.emplace_back([this, i] { worker(i); });
            std::unique_lock lock(mutex_);
            completed_.wait(lock, [this] { return ready_ == workers_.size(); });
        } catch (...) {
            shutdown();
            throw;
        }
    }
    ~Impl() { shutdown(); }
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;

    struct Operation {
        explicit Operation(std::atomic_flag& busy) : busy_(busy) {
            if (busy_.test_and_set(std::memory_order_acquire))
                throw std::logic_error("WorkerPool: concurrent run/stop is not permitted");
        }
        ~Operation() { busy_.clear(std::memory_order_release); }
        Operation(const Operation&) = delete;
        Operation& operator=(const Operation&) = delete;
        Operation(Operation&&) = delete;
        Operation& operator=(Operation&&) = delete;
        std::atomic_flag& busy_;
    };

    void worker(unsigned index) {
        std::unique_lock lock(mutex_);
        std::uint64_t observed = 0;
        ++ready_;
        completed_.notify_one();
        for (;;) {
            published_.wait(lock, [this, &observed] { return stopping_ || generation_ != observed; });
            if (stopping_) return;
            observed = generation_;
            const auto range = ranges_[index];
            const auto work = work_;
            void* const context = context_;
            lock.unlock();
            work(context, range.begin, range.end);
            // Each byte has exactly one writer. The mutex publication below
            // establishes completion before the owner's status read.
            statuses_[index] = 1;
            lock.lock();
            --remaining_;
            if (remaining_ == 0) completed_.notify_one();
        }
    }

    void run(std::span<const IndexRange> ranges, ElementWork work, void* context) {
        const Operation operation(busy_);
        if (!work || ranges.size() != count_)
            throw std::invalid_argument("WorkerPool.run: one range per thread and a work callback are required");
        for (std::size_t i = 0; i < ranges.size(); ++i) {
            if (ranges[i].begin > ranges[i].end ||
                (i != 0 && ranges[i - 1].end != ranges[i].begin))
                throw std::invalid_argument("WorkerPool.run.ranges: ordered contiguous ranges required");
        }
        std::unique_lock lock(mutex_);
        if (stopping_) throw std::logic_error("WorkerPool.run: pool has been stopped");
        if (generation_ == std::numeric_limits<std::uint64_t>::max())
            throw std::logic_error("WorkerPool.run: generation exhausted");
        std::ranges::copy(ranges, ranges_.begin());
        std::ranges::fill(statuses_, std::uint8_t{0});
        work_ = work;
        context_ = context;
        remaining_ = workers_.size();
        ++generation_;
        lock.unlock();
        published_.notify_all();
        work(context, ranges_[0].begin, ranges_[0].end);
        statuses_[0] = 1;
        lock.lock();
        completed_.wait(lock, [this] { return remaining_ == 0; });
        for (const auto status : statuses_)
            if (status != 1) throw std::logic_error("WorkerPool.run: incomplete generation");
        work_ = nullptr;
        context_ = nullptr;
    }

    void stop() {
        const Operation operation(busy_);
        shutdown();
    }

    void shutdown() noexcept {
        {
            const std::scoped_lock lock(mutex_);
            stopping_ = true;
        }
        published_.notify_all();
        for (auto& worker_thread : workers_)
            if (worker_thread.joinable()) worker_thread.join();
    }

    const unsigned count_;
    std::vector<std::thread> workers_;
    std::vector<IndexRange> ranges_;
    std::vector<std::uint8_t> statuses_;
    std::mutex mutex_;
    std::condition_variable published_, completed_;
    // C++20 default-initializes atomic_flag clear; ATOMIC_FLAG_INIT is deprecated.
    std::atomic_flag busy_;
    std::size_t ready_ = 0, remaining_ = 0;
    std::uint64_t generation_ = 0;
    ElementWork work_ = nullptr;
    void* context_ = nullptr;
    bool stopping_ = false;
};

WorkerPool::WorkerPool(unsigned threads) : impl_(std::make_unique<Impl>(threads)) {}
WorkerPool::~WorkerPool() = default;
unsigned WorkerPool::worker_count() const noexcept { return impl_->count_ - 1U; }
void WorkerPool::run(std::span<const IndexRange> ranges, ElementWork work, void* context) {
    impl_->run(ranges, work, context);
}
void WorkerPool::stop() { impl_->stop(); }

} // namespace runtime::spatial
