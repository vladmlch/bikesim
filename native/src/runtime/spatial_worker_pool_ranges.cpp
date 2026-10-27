#include "runtime/spatial_worker_pool.hpp"

#include <cstdint>
#include <limits>
#include <stdexcept>

namespace runtime::spatial {

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) call sites pass (count, workers) positionally
std::vector<IndexRange> partition_ranges(std::size_t count, unsigned workers) {
    if (workers == 0) throw std::invalid_argument("partition_ranges.workers: must be positive");
    static_assert(sizeof(std::size_t) <= sizeof(std::uint64_t));
    const auto n = static_cast<std::uint64_t>(count);
    const auto divisor = static_cast<std::uint64_t>(workers);
    std::vector<IndexRange> ranges;
    ranges.reserve(workers);
    if (n <= std::numeric_limits<std::uint64_t>::max() / divisor) {
        for (std::uint64_t i = 0; i < divisor; ++i)
            ranges.push_back({.begin = static_cast<std::size_t>((n * i) / divisor),
                              .end = static_cast<std::size_t>((n * (i + 1)) / divisor)});
    } else {
        const auto quotient = n / divisor, remainder = n % divisor;
        std::uint64_t begin = 0;
        for (std::uint64_t i = 0; i < divisor; ++i) {
            const auto end = begin + quotient + (i < remainder ? 1U : 0U);
            ranges.push_back({.begin = static_cast<std::size_t>(begin), .end = static_cast<std::size_t>(end)});
            begin = end;
        }
    }
    // Canonical fan index: ((wheel * n_theta_fan) + it) * n_phi + ip.
    // Dispatch assigns slots only. Reductions always traverse these indices,
    // independently of completion order and of the number of workers.
    return ranges;
}

} // namespace runtime::spatial
