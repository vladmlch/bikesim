#pragma once

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

namespace runtime::spatial {

struct IndexRange { std::size_t begin = 0, end = 0; };

// workers == 0 is invalid. The owner preserves deterministic reduction order.
std::vector<IndexRange> partition_ranges(std::size_t count, unsigned workers);
using ElementWork = void (*)(void* context, std::size_t begin,
                            std::size_t end) noexcept;

class WorkerPool {
public:
    explicit WorkerPool(unsigned threads);
    ~WorkerPool();
    WorkerPool(const WorkerPool&) = delete;
    WorkerPool& operator=(const WorkerPool&) = delete;
    unsigned worker_count() const noexcept;
    void run(std::span<const IndexRange>, ElementWork, void* context);
    void stop();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace runtime::spatial
