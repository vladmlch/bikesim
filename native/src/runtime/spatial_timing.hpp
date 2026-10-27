#pragma once

// Private, observational instrumentation. The collector belongs to the caller
// of advance/flush, never to a physics checkpoint or a worker's mutable state.
// Owner-side scopes include dispatch/barrier wall time. Workers do not inherit
// the thread-local binding and do not update shared floating-point counters.
#include "spatial_performance.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace runtime::spatial::timing {
enum class Region : std::uint8_t {
    terrain_rays, tyre_elements, engine_step, rider_steer, accounting,
    sensors, recording, ui, count
};
class Scope;
struct Collector {
    std::array<double, static_cast<std::size_t>(Region::count)> seconds{};
    std::array<std::uint64_t, static_cast<std::size_t>(Region::count)> calls{};
    bool valid = true;
    double clock_read_cost_s = 0.;
    Scope* active_scope = nullptr;

    // Cold calibration estimates clock reads only. It is not subtracted from
    // measured RTF and is not advertised as the full instrumentation cost.
    void calibrate() noexcept {
        double total = 0.;
        constexpr unsigned repetitions = 128;
        for (unsigned i = 0; i < repetitions; ++i) {
            const auto start = std::chrono::steady_clock::now();
            total += std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        }
        clock_read_cost_s = total / repetitions;
        if (!std::isfinite(clock_read_cost_s) || clock_read_cost_s < 0.) valid = false;
    }
    [[nodiscard]] RegionTimings regions() const noexcept {
        return {.terrain_rays_s = seconds[0], .tyre_elements_s = seconds[1],
            .engine_step_s = seconds[2], .rider_steer_s = seconds[3],
            .accounting_s = seconds[4], .sensors_s = seconds[5],
            .recording_s = seconds[6], .ui_s = seconds[7],
            .sensors_pending = calls[5] == 0, .recording_pending = calls[6] == 0,
            .ui_pending = calls[7] == 0};
    }
    [[nodiscard]] std::uint64_t clock_reads() const noexcept {
        std::uint64_t result = 0;
        for (const auto count : calls) {
            if (count > (std::numeric_limits<std::uint64_t>::max() - result) / 2)
                return std::numeric_limits<std::uint64_t>::max();
            result += 2 * count;
        }
        return result;
    }
};
inline thread_local Collector* current = nullptr; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables) the Scope RAII binds through this thread-local slot

class Binding {
public:
    explicit Binding(Collector* collector) noexcept : previous_(current) { current = collector; }
    ~Binding() { current = previous_; }
    Binding(const Binding&) = delete;
    Binding& operator=(const Binding&) = delete;
    Binding(Binding&&) = delete;
    Binding& operator=(Binding&&) = delete;
private:
    Collector* previous_;
};

class Scope {
public:
    explicit Scope(Region region) noexcept : collector_(current), index_(static_cast<std::size_t>(region)) {
        if (collector_) {
            start_ = std::chrono::steady_clock::now();
            parent_ = collector_->active_scope;
            if (parent_) parent_->pause(start_);
            collector_->active_scope = this;
        }
    }
    ~Scope() { stop(); }
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
    Scope(Scope&&) = delete;
    Scope& operator=(Scope&&) = delete;
    void stop() noexcept {
        if (!collector_) return;
        const auto end = std::chrono::steady_clock::now();
        pause(end);
        auto& count = collector_->calls[index_];
        auto& total = collector_->seconds[index_];
        if (!std::isfinite(elapsed_) || elapsed_ < 0. ||
            count == std::numeric_limits<std::uint64_t>::max() || !std::isfinite(total + elapsed_)) {
            collector_->valid = false;
        } else { total += elapsed_; ++count; }
        collector_->active_scope = parent_;
        if (parent_) parent_->start_ = end;
        collector_ = nullptr;
    }
private:
    Collector* collector_;
    std::size_t index_;
    Scope* parent_ = nullptr;
    double elapsed_ = 0.;
    std::chrono::steady_clock::time_point start_{};
    void pause(std::chrono::steady_clock::time_point now) noexcept {
        const double part = std::chrono::duration<double>(now - start_).count();
        if (!std::isfinite(part) || part < 0.) collector_->valid = false;
        elapsed_ += part;
    }
};
} // namespace runtime::spatial::timing
