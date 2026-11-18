#pragma once

#include <array>
#include <span>

#include "../rtsan.hpp"
#include "../spatial/types.hpp"

namespace runtime {

struct LaunchSupportConfig {
    bike::spatial::SpatialStartSupportSpec spec{};
    int root_free_qposadr = -1;
    int root_free_dofadr = -1;
    double standstill_speed_mps = 0.1;
    bool initially_armed = true;
};

struct LaunchSupportState {
    bool armed = false;
    bool reference_latched = false;
    std::array<double, 3> ref_pos_m{0.0, 0.0, 0.0};
    double ref_yaw_rad = 0.0;
};

struct LaunchSupportInput {
    std::span<const double> qpos;
    std::span<const double> qvel;
    std::span<const double, 3> frame_pos_world_m;
    double forward_speed_mps = 0.0;
    bool motor_braking = false;
    bool advance = true;
};

// Orthogonal standstill lateral + roll launch fixture for spatial 3D starts.
// Applies lateral point force at BB (`xpos[frame]`) along world-horizontal
// e_lat(psi) = (-sin(psi), cos(psi), 0) and roll torque through the ZYX
// roll-rate covector j_phi(phi, theta) = (1, sin(phi)*tan(theta), cos(phi)*tan(theta)),
// yielding identically zero virtual power on forward/uphill motion, +Z heave,
// sagittal pitch rate, and yaw rate.
class LaunchSupport {
public:
    explicit LaunchSupport(const LaunchSupportConfig &config);

    void reset(bool initially_armed = true) noexcept BIKE_NONBLOCKING;

    void latch_reference(std::span<const double, 3> frame_pos_world_m,
                         double yaw_rad) noexcept BIKE_NONBLOCKING;

    [[nodiscard]] LaunchSupportState state() const noexcept BIKE_NONBLOCKING {
        return state_;
    }

    void restore(const LaunchSupportState &state);

    [[nodiscard]] bool armed() const noexcept BIKE_NONBLOCKING {
        return state_.armed;
    }

    [[nodiscard]] const bike::spatial::SpatialStartSupportSpec &
    spec() const noexcept BIKE_NONBLOCKING {
        return config_.spec;
    }

    [[nodiscard]] double standstill_speed_mps() const noexcept BIKE_NONBLOCKING {
        return config_.standstill_speed_mps;
    }

    // Smooth C^2/C^1 quintic fade-out weight w(v_fwd) over [0.5 * v_rel, v_rel].
    [[nodiscard]] double
    fade_weight(double forward_speed_mps) const noexcept BIKE_NONBLOCKING;

    // Writes generalized force into `out_qfrc` (zero-filled first) and updates
    // armed/latched state when `input.advance == true`.
    // Returns active fade weight w in [0, 1] (0.0 when disarmed or invalid attitude).
    double compute_into(const LaunchSupportInput &input,
                        std::span<double> out_qfrc) noexcept BIKE_NONBLOCKING;

private:
    LaunchSupportConfig config_{};
    LaunchSupportState state_{};
};

} // namespace runtime
