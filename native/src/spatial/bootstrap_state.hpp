#pragma once

#include "config.hpp"
#include "types.hpp"
#include "../io/value.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace bike::spatial {

enum class BootstrapPhase : std::uint8_t { settling, release_check, armed, scored };
struct HoldState {
    std::uint32_t schema = 1;
    bool active = false;
    Vec3 reference_position_world_m{};
    Quat reference_quat_wxyz{1., 0., 0., 0.};
    Vec3 last_force_world_n{};
    Vec3 last_couple_world_nm{};
    double stored_energy_j = 0.;
    double work_j = 0.;
    BootstrapPhase phase = BootstrapPhase::settling;
    std::uint8_t valid_mask = 0;
};
struct SteerState {
    std::uint32_t schema = 1;
    SteerController controller = SteerController::off;
    std::int64_t next_tick = 0;
    std::vector<double> delay_fifo_nm;
    std::size_t fifo_head = 0;
    std::size_t fifo_count = 0;
    std::string profile_sha256;
    double last_command_nm = 0.;
    double last_applied_nm = 0.;
    std::int64_t saturated_ticks = 0;
    double work_j = 0.;
    std::uint8_t valid_mask = 0;
};
enum class SpatialReason : std::uint8_t {
    none, capsize, off_corridor, balance_lost, terrain_invalid,
    terrain_outside, numerical_failure, requested_completion
};
struct Outcome {
    SpatialReason reason = SpatialReason::none;
    bool terminal = false;
    bool valid = true;
    std::int64_t tick = -1;
    double time_s = 0.;
    Vec3 position_world_m{};
    std::string diagnostic;
};
struct OutcomeState {
    std::uint32_t schema = 1;
    Outcome latched;
    std::int64_t capsize_dwell_ticks = 0;
    std::int64_t corridor_dwell_ticks = 0;
    std::int64_t balance_dwell_ticks = 0;
    std::uint8_t valid_mask = 0;
};

HoldState canonical_inactive_hold() noexcept;
SteerState canonical_inactive_steer();
OutcomeState canonical_unlatched_outcome() noexcept;
bike::io::Value encode_hold(const HoldState&);
HoldState decode_hold(const bike::io::Value&);
bike::io::Value encode_steer(const SteerState&);
SteerState decode_steer(const bike::io::Value&);
bike::io::Value encode_outcome(const OutcomeState&);
OutcomeState decode_outcome(const bike::io::Value&);
std::string_view to_string(BootstrapPhase) noexcept;
std::optional<BootstrapPhase> parse_phase(std::string_view) noexcept;
std::string_view to_string(SpatialReason) noexcept;

} // namespace bike::spatial
