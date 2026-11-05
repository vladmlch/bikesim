#pragma once
// RunLimits — the physical ride's finish position, step cap and wall-clock
// watchdog. Moved unchanged from the removed legacy controls (2026-11-04).

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

namespace runtime {
struct RunLimits {
    double finish_x_m{};
    std::int64_t max_steps{};
    double max_wall_clock_s = 300.;

    void validate() const {
        positive(finish_x_m, "finish_x_m");
        positive(max_wall_clock_s, "max_wall_clock_s");
        if (max_steps <= 0) throw std::invalid_argument("max_steps must be positive");
    }
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    [[nodiscard]] static RunLimits for_track(double length_m, double start_x_m, double timestep_s,
            double max_wall_clock_s = 300., double min_speed_mps = 15. / 3.6) {
        positive(timestep_s, "timestep_s");
        positive(min_speed_mps, "min_speed_mps");
        const double distance = length_m - start_x_m;
        positive(distance, "remaining track length");
        const double count = 2. * distance / (min_speed_mps * timestep_s);
        if (!std::isfinite(count) || count >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
            throw std::invalid_argument("run step cap exceeds int64 range");
        RunLimits result{.finish_x_m = length_m, .max_steps = static_cast<std::int64_t>(count),
                         .max_wall_clock_s = max_wall_clock_s};
        result.validate();
        return result;
    }

private:
    static void positive(double value, std::string_view name) {
        if (!std::isfinite(value) || value <= 0.)
            throw std::invalid_argument(std::string(name) + " must be finite and positive");
    }
};
} // namespace runtime
