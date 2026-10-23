#pragma once

#include "../io/value.hpp"
#include <array>
#include <cstdint>

namespace bike::spatial {

struct BrakeWheelLatch {
    bool engaged = false;
    double latch_angle_rad = 0.;
    double unwrapped_angle_rad = 0.;
    double stored_energy_j = 0.;
    double previous_demand = 0.;
};
struct BrakeState {
    std::uint32_t schema = 1;
    std::array<BrakeWheelLatch, 2> wheel{};
    std::int64_t interval_index = -1;
    std::uint8_t valid_mask = 0;
};
bike::io::Value encode_brake(const BrakeState&);
BrakeState decode_brake(const bike::io::Value&);
BrakeState canonical_released_brake() noexcept;

} // namespace bike::spatial
