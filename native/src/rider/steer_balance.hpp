#pragma once

#include "../io/value.hpp"
#include "../spatial/bootstrap_state.hpp"
#include "../writers/writer_types.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace bike::rider {

// Other nonempty identifiers name external profile files. Keep acquisition,
// recording and replay on the same versioned built-in/source distinction.
inline constexpr bool builtin_steer_profile(std::string_view id) noexcept {
    return id.empty() || id == "roll-heading-v1" || id == "steer-provisional-v1";
}

struct SteerInput {
    std::int64_t control_tick = 0;
    double roll_rad = 0.;
    double roll_rate_rad_s = 0.;
    double heading_error_rad = 0.;
    double lateral_error_m = 0.;
    double steer_angle_rad = 0.;
    double steer_rate_rad_s = 0.;
    double speed_signed_mps = 0.;
    bool enabled = true;
};
struct SteerProfile {
    std::uint32_t schema = 1;
    std::vector<double> speed_nodes_mps;
    // roll, roll rate, heading, lateral, steer, steer rate at each speed node
    std::vector<std::array<double, 6>> gains;
    double reference_torque_nm = 0.;
    double cap_nm = 40.;
    double delay_s = .1;
    std::string sha256_hex;
    std::string provenance_status = "provisional";
};
SteerProfile read_steer_profile(const bike::io::Value&);
bike::io::Value write_steer_profile(const SteerProfile&);
class SteerBalance {
public:
    SteerBalance(SteerProfile, double control_period_s);
    double tick(const SteerInput&) noexcept;
    const SteerProfile& profile() const noexcept { return profile_; }
    const bike::spatial::SteerState& state() const noexcept;
    CoreStatus restore(const bike::spatial::SteerState&) noexcept;
    void reset() noexcept;
    double resolved_delay_s() const noexcept;
    std::size_t delay_ticks() const noexcept;
private:
    SteerProfile profile_;
    double period_s_;
    bike::spatial::SteerState state_;
};

} // namespace bike::rider
