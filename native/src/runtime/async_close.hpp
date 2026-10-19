// T42: native, data-only off-path period evaluation. There is no engine or
// filesystem work on the worker; publication always belongs to the caller.
#pragma once
#include "accounting.hpp"
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace runtime {
class AsyncWorkerLost : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
    ~AsyncWorkerLost() override;
    AsyncWorkerLost(const AsyncWorkerLost&) = default;
    AsyncWorkerLost& operator=(const AsyncWorkerLost&) = default;
    AsyncWorkerLost(AsyncWorkerLost&&) = default;
    AsyncWorkerLost& operator=(AsyncWorkerLost&&) = default;
};
class AsyncPublicationError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
    ~AsyncPublicationError() override;
    AsyncPublicationError(const AsyncPublicationError&) = default;
    AsyncPublicationError& operator=(const AsyncPublicationError&) = default;
    AsyncPublicationError(AsyncPublicationError&&) = default;
    AsyncPublicationError& operator=(AsyncPublicationError&&) = default;
};
class AsyncCloseRunner {
public:
    struct Result {
        std::uint64_t sequence{};
        AccountingPublication publication;
    };
    // The optional hook supplies deterministic worker fault injection to the
    // native regression executable. Production passes an empty hook.
    using BeforeClose = std::function<void(std::uint64_t)>;
    AsyncCloseRunner(AccountingConfig, AccountingState, int,
                     std::optional<EffortContext>, BeforeClose = {});
    ~AsyncCloseRunner();
    AsyncCloseRunner(const AsyncCloseRunner&) = delete;
    AsyncCloseRunner& operator=(const AsyncCloseRunner&) = delete;
    AsyncCloseRunner(AsyncCloseRunner&&) = delete;
    AsyncCloseRunner& operator=(AsyncCloseRunner&&) = delete;
    void submit(std::shared_ptr<const std::vector<RawStep>>);
    [[nodiscard]] std::shared_ptr<const Result> peek(bool block);
    void acknowledge(std::uint64_t);
    [[nodiscard]] std::size_t pending() const;
    [[nodiscard]] std::size_t pending_intervals() const;
    [[nodiscard]] std::vector<std::shared_ptr<const std::vector<RawStep>>> detach_pending();
    void close() noexcept;
private:
    struct Job { std::uint64_t sequence{}; std::shared_ptr<const std::vector<RawStep>> raws; };
    void work() noexcept;
    std::unique_ptr<PeriodAccounting> evaluator_;
    BeforeClose before_close_;
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    std::deque<Job> jobs_, pending_;
    std::deque<std::shared_ptr<const Result>> results_;
    std::exception_ptr failure_;
    bool worker_lost_ = false, stopping_ = false, finished_ = false;
    std::uint64_t sequence_ = 0;
    std::thread worker_;
};
} // namespace runtime
