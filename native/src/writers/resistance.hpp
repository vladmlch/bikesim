// writers/resistance.hpp — port of ExternalResistanceApplier
// (src/bike_sim/sim/ride/physical_resistance.py) plus the helpers it calls:
// _rolling_moment/_drag_force (physics/external_resistance.py) and
// point_jacobian_into (physical_mapping.py). Every `@`/`np.dot` in the
// source lowers to an Apple Accelerate CBLAS call by numpy — the writer
// invokes the same symbols, the same argument that lets mj_jac port
// directly. Scalar expressions keep their Python order verbatim.
#pragma once

#include <mujoco/mujoco.h>

#include <cstddef>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "../config_types.hpp"
#include "../rtsan.hpp"
#include "writer_types.hpp"

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
    ResistanceWriter(const mjModel *m, nativecfg::ResistanceConfig config);

    using Component = std::pair<std::string, std::vector<double> >;

    // compute_components (physical_resistance.py:20-42): 'road_rolling'
    // then 'aerodynamic', the Python dict's insertion order.
    [[nodiscard]] std::vector<Component> components(
        const mjData *d, const TireSideInput &front,
        const TireSideInput &rear) const;

    // R1 allocation-free face. components_into fills `out` with one
    // ForceComponentView per named component — kComponentCount views in
    // compute_components' insertion order, each ForceKind::resistance —
    // over persistent member storage valid until the next compute.
    // components_into is the checked convenience boundary:
    // std::invalid_argument on a null mjData or wrong view count, then
    // components()' arithmetic verbatim. try_components_into is the
    // prevalidated warm-core entry: same preconditions (plus the
    // per-side patch-shape gate) through CoreStatus, no throw/string
    // construction or allocation in its own code (residual kernel
    // rejections are caught and mapped — see the .cpp), so it is proved
    // noexcept.
    static constexpr std::size_t kComponentCount = 2;
    void components_into(const mjData *d, const TireSideInput &front,
                         const TireSideInput &rear,
                         std::span<ForceComponentView> out) const;
    [[nodiscard]] CoreStatus
    try_components_into(const mjData *d, const TireSideInput &front,
                        const TireSideInput &rear,
                        std::span<ForceComponentView> out)
            const noexcept BIKE_NONBLOCKING;

    // The serialized component names in compute_components' insertion
    // order — Stepper's boxed convenience surface serializes views under
    // these names instead of calling the allocating components().
    [[nodiscard]] static std::span<const std::string_view>
    component_names() noexcept;

private:
    const mjModel *m_ = nullptr; // non-owning; the Stepper outlives the writer
    nativecfg::ResistanceConfig cfg_;
    int nv_ = 0;
    // Resolved unconditionally in the ctor via resolve_id (throws on a
    // missing body); -1 is the declared "not resolved" state — 0 is the
    // world body and must never mean absent.
    int frame_ = -1, front_wheel_ = -1, rear_wheel_ = -1;
    // self._jr / self._jp — per-step scratch shared by the rolling and drag
    // maps, fully rewritten by mj_jac on every use. Ownership: per-instance
    // scratch of this writer's owning Stepper context — components() is
    // const yet mutates them, which stays correct only because calls arrive
    // through the nanobind boundary while the GIL is held. Sharing one
    // writer across contexts requires external synchronization outside
    // realtime code.
    mutable std::vector<mjtNum> jr_, jp_;

    // components()'s arithmetic verbatim on the member buffers — the
    // patch-shape gate, rolling-moment and drag computation, and the
    // derived() validation — filling rolling_/aerodynamic_ (row_ is the
    // shared per-side row). Kept throwing so both boundaries share one
    // kernel and the original rejection types reach the caller.
    void compute_validated(const mjData *d, const TireSideInput &front,
                           const TireSideInput &rear) const;

    // Publish the insertion-ordered views over the member components.
    // noexcept: spans over construction-sized storage only.
    void export_views(std::span<ForceComponentView> out) const noexcept;

    // The per-call 'rolling'/'aerodynamic'/per-side 'row' vectors of
    // compute_components — construction-sized (nv), rewritten in place
    // every compute. export_views borrows rolling_/aerodynamic_, so they
    // must never resize; row_ is shared scratch fully overwritten by
    // each side's gemv (beta = 0).
    mutable std::vector<double> rolling_, aerodynamic_, row_;
};
