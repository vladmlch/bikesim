#pragma once

#include <cmath>
#include <stdexcept>

namespace spatial_gpu {

inline bool latency_goal_met(double cpu_s, double gpu_s,
                             bool accuracy_passed, bool deterministic) {
    if (!std::isfinite(cpu_s) || !std::isfinite(gpu_s) || cpu_s <= 0. || gpu_s <= 0.)
        throw std::invalid_argument("positive finite measured times required");
    return accuracy_passed && deterministic && gpu_s <= cpu_s / 2.;
}

inline bool batch_goal_met(double cpu_episodes_per_s, double gpu_episodes_per_s,
                           bool same_physics) {
    if (!std::isfinite(cpu_episodes_per_s) || !std::isfinite(gpu_episodes_per_s) ||
        cpu_episodes_per_s <= 0. || gpu_episodes_per_s <= 0.)
        throw std::invalid_argument("positive finite throughput required");
    return same_physics && gpu_episodes_per_s >= 20. * cpu_episodes_per_s;
}

inline bool offline_goal_met(double cpu_task_s, double gpu_task_s,
                             bool exact_checks_passed) {
    if (!std::isfinite(cpu_task_s) || !std::isfinite(gpu_task_s) ||
        cpu_task_s <= 0. || gpu_task_s <= 0.)
        throw std::invalid_argument("positive finite task times required");
    return exact_checks_passed && gpu_task_s * 3. <= cpu_task_s;
}

// H1 needs both the twofold section gain and >=10% of the COMPLETE ride
// wall time, including dispatch/upload/readback/synchronization. Passing
// latency_goal_met alone must never select a production offload design.
inline bool interactive_goal_met(double cpu_section_s, double gpu_section_s,
                                 double cpu_ride_s, double gpu_ride_s,
                                 bool accuracy_passed, bool deterministic) {
    const bool section = latency_goal_met(cpu_section_s, gpu_section_s,
                                          accuracy_passed, deterministic);
    if (!std::isfinite(cpu_ride_s) || !std::isfinite(gpu_ride_s) ||
        cpu_ride_s <= 0. || gpu_ride_s <= 0.)
        throw std::invalid_argument("positive finite complete-ride times required");
    return section && gpu_ride_s <= 0.90 * cpu_ride_s;
}

} // namespace spatial_gpu
