// check_pool.cpp — bounded worker pool for the per-file sweeps.
//
// Child processes go through platform::run, which serializes pipe creation
// (process.cpp) so concurrent spawns never leak sibling pipe ends; per-file
// logs and scratch paths are keyed by the entry index.
#include "checks_internal.hpp"

#include <algorithm>
#include <atomic>
#include <exception>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <vector>

namespace bike::checks {

std::size_t check_jobs(const std::map<std::string, std::string> &environment) {
    if (const auto found = environment.find("BIKE_CHECK_JOBS");
        found != environment.end()) {
        const std::optional<std::int64_t> parsed = parse_int64(found->second);
        if (!parsed.has_value() || *parsed < 1)
            throw CheckError("BIKE_CHECK_JOBS must be a positive integer, got '" +
                             found->second + "'");
        return static_cast<std::size_t>(*parsed);
    }
    const long online = ::sysconf(_SC_NPROCESSORS_ONLN);
    return online > 0 ? static_cast<std::size_t>(online) : std::size_t{1};
}

void for_each_index(std::size_t count, std::size_t workers,
                    const std::function<void(std::size_t)> &task) {
    if (count == 0) return;
    const std::size_t threads = std::clamp<std::size_t>(workers, 1, count);
    std::atomic<std::size_t> next{0};
    std::exception_ptr first_error;
    std::mutex error_mutex;
    const auto drain = [&] {
        for (std::size_t index = next.fetch_add(1); index < count;
             index = next.fetch_add(1)) {
            try {
                task(index);
            } catch (...) {
                const std::scoped_lock<std::mutex> lock(error_mutex);
                if (!first_error) first_error = std::current_exception();
                next.store(count); // hand out no further indexes
            }
        }
    };
    {
        std::vector<std::jthread> pool;
        pool.reserve(threads - 1);
        for (std::size_t worker = 1; worker < threads; ++worker)
            pool.emplace_back(drain);
        drain();
    } // every jthread joins here
    if (first_error) std::rethrow_exception(first_error);
}

} // namespace bike::checks
