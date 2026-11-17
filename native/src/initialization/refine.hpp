#pragma once

#include <mujoco/mujoco.h>

#include <optional>

#include "equilibrium.hpp"

namespace bike::initialization {

struct FreeJointAddress {
    int joint = -1;
    int qpos = -1;
    int dof = -1;
};

[[nodiscard]] std::optional<FreeJointAddress> find_free_root(const mjModel* m);

class PhaseAHoldScope {
public:
    explicit PhaseAHoldScope(SetupContext& context);
    ~PhaseAHoldScope();
    PhaseAHoldScope(const PhaseAHoldScope&) = delete;
    PhaseAHoldScope& operator=(const PhaseAHoldScope&) = delete;
    PhaseAHoldScope(PhaseAHoldScope&&) = delete;
    PhaseAHoldScope& operator=(PhaseAHoldScope&&) = delete;

private:
    bool active_ = false;
};

[[nodiscard]] bool phase_a_hold_active(const SetupContext& context) noexcept;

struct SpatialEquilibriumMoments {
    double roll_nm = 0.;
    double pitch_nm = 0.;
    double steer_nm = 0.;
};

[[nodiscard]] SpatialEquilibriumMoments spatial_equilibrium_moments(const SetupContext& context);

} // namespace bike::initialization
