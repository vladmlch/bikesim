// writers/resistance.hpp — port of ExternalResistanceApplier
// (src/bike_sim/sim/ride/physical_resistance.py) plus the helpers it calls:
// _rolling_moment/_drag_force (physics/external_resistance.py) and
// point_jacobian_into (physical_mapping.py). Every `@`/`np.dot` in the
// source lowers to an Apple Accelerate CBLAS call by numpy — the writer
// invokes the same symbols, the same argument that lets mj_jac port
// directly. Scalar expressions keep their Python order verbatim.
#pragma once

#include <mujoco/mujoco.h>

#include <span>
#include <string>
#include <utility>
#include <vector>

#include "../config.hpp"

// One side's tire snapshot, flattened for the binding boundary: the fields
// compute_components reads (per-patch normal_load_n / working_surface in
// patch order, plus effective_radius_m).
struct TireSideInput {
    std::span<const double> patch_loads;
    std::span<const bool> patch_working;
    double effective_radius_m;
};

class ResistanceWriter {
public:
    // Mirrors ExternalResistanceApplier.__init__ (physical_resistance.py:
    // 10-18): resolve_id for the frame and both wheel bodies, plus the
    // shared (3,nv) Jacobian scratch. The ResistanceConfig.__post_init__
    // validation (physical_config.py:275-284) lives here too — config
    // object construction is where Python puts it.
    ResistanceWriter(const mjModel* m, nativecfg::ResistanceConfig config);

    using Component = std::pair<std::string, std::vector<double>>;

    // compute_components (physical_resistance.py:20-42): 'road_rolling'
    // then 'aerodynamic', the Python dict's insertion order.
    [[nodiscard]] std::vector<Component> components(
        const mjData* d, const TireSideInput& front,
        const TireSideInput& rear) const;

private:
    const mjModel* m_;   // non-owning; the Stepper outlives the writer
    nativecfg::ResistanceConfig cfg_;
    int nv_;
    int frame_, front_wheel_, rear_wheel_;
    // self._jr / self._jp — per-step scratch shared by the rolling and drag
    // maps, fully rewritten by mj_jac on every use.
    mutable std::vector<mjtNum> jr_, jp_;
};
