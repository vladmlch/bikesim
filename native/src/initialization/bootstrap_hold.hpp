#pragma once

#include "spatial_equilibrium.hpp"
#include "../spatial/bootstrap_state.hpp"
#include "../spatial/config.hpp"
#include "../spatial/topology.hpp"
#include "../writers/writer_types.hpp"
#include <mujoco/mujoco.h>
#include <span>
#include <vector>

namespace bike::initialization {

class BootstrapHold {
public:
    BootstrapHold(const bike::spatial::BootstrapConfig&,
                  const bike::spatial::RootBinding&, const mjModel&);
    // phase >= armed: exact zero contribution; cumulative work is unchanged.
    CoreStatus try_compute_into(const mjModel&, const mjData&,
                                bike::spatial::BootstrapPhase,
                                std::span<double> qfrc) noexcept;
    const bike::spatial::HoldState& state() const noexcept;
    CoreStatus restore(const bike::spatial::HoldState&) noexcept;
    // acceleration_tolerance is the tolerance the settle request accepted
    // (SetupContext::equilibrium_tolerance); the default keeps callers that do
    // not carry the request on the historical bound.
    bool release_check(const SpatialEquilibriumReport&, double force_limit_n,
                       double moment_limit_nm,
                       double acceleration_tolerance = .05) const noexcept;
private:
    bike::spatial::BootstrapConfig cfg_;
    bike::spatial::RootBinding root_;
    bike::spatial::HoldState state_;
    std::vector<double> jacp_, jacr_;
};

} // namespace bike::initialization
