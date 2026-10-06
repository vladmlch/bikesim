// writers/tire.hpp — port of TireForceApplier (src/bike_sim/sim/ride/
// tire_forces.py) — the only stateful force writer: per-wheel _BrushState
// evolves inside compute_qfrc and commits only after BOTH wheels evaluate.
// The flattened-state schema is the artifact's `tire_state_names` layout
// (flatten_row(asdict(_BrushState)) per side, NaN for unset fields).
#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "../config_types.hpp"
#include "../tyre/profile.hpp"

// ContactPatch (contact_state.py:19-76) — tire patches always carry
// source 'terrain' (working_surface=true), no couple, no native force.
struct TirePatch {
    std::array<double, 3> point_m;
    std::array<double, 3> normal;
    double normal_load_n;
    double tangent_force_n;
    double slip_mps;
    bool working_surface;
};

// WheelContactSnapshot (contact_state.py:79-190) for one side, plus the
// effective_radius_m property precomputed the way contact_state.py
// computes it (load-weighted mean contact drop; unit weights when every
// patch is unloaded; 0 when there is no patch).
struct TireSnapshot {
    double time_s{};
    std::int64_t interval_id{};
    std::string backend;
    bool geometric_contact{};
    std::array<double, 3> wheel_axis_m{};
    std::vector<TirePatch> patches;
    double effective_radius_m{};
};

// The per-side diagnostics dict the Python writer assembles inside the
// loop (tire_forces.py:190-199) — exposed for parity checks/debugging.
struct TireDiagnostics {
    bool multi_support;
    double penetration_m;
    double normal_speed_mps;
    double slip_mps;
    double normal_load_n;
    double tangent_force_n;
    double friction_coefficient;
    std::string surface;
    double branch_release_loss_j;
    double brush_loss_j;
    double radial_energy_j;
    double shear_energy_j;
    bool outside_material_load_range;
};

// _BrushState (tire_forces.py:47-53). `xi` is a plain float in Python —
// the dataclass default is 0.0 and a restored NaN reaches _brush_step's
// _finite_result gate exactly like a Python NaN xi does.
struct BrushState {
    double xi = 0.0;
    std::optional<std::array<double, 3> > tangent;
    std::optional<std::array<double, 3> > point;
    std::optional<int> segment;
    std::optional<std::array<double, 3> > center;
};

class TireWriter {
public:
    // TireForceApplier.__init__ (tire_forces.py:65-89): profile built from
    // the compiled heightfield, geom/body/radius resolution, the wheel
    // collision gate, and fresh brush states. `d` supplies the geom
    // transforms the profile compile reads — it must be forwarded already.
    TireWriter(const mjModel *m, const mjData *d,
               nativecfg::TireConfig config);

    // Reset persistent brush state, diagnostics, energy and the advance clock
    // when an owning Stepper recovers from a fatal engine operation.
    void reset();

    // compute_qfrc(advance=True) (tire_forces.py:116-206). Mutates the
    // brush states; raises std::invalid_argument on the same-timestamp
    // double-advance, and leaves state untouched when either wheel's
    // evaluation throws.
    [[nodiscard]] std::vector<double> qfrc(const mjData *d, double dt);

    // Artifact-row restore: names follow `tire_state_names` ('front.xi',
    // 'front.tangent.0', ..., leaf 'front.tangent' tolerated as a NaN
    // column). All 22 canonical columns are required — missing ones throw
    // naming the column (the artifact contract is canonical-schema, not
    // partial decode). Resets the timestamp clock — a restored state has
    // not yet been advanced under any interval (mirrors the Python test
    // oracle's `last_time_s = None`).
    void set_state(std::span<const std::string> names,
                   std::span<const double> row);

    // Canonical 22-name schema this writer emits (tangent/point/center
    // component columns only — leaf None-columns decode but do not emit).
    [[nodiscard]] static const std::vector<std::string> &state_names();

    // Flattened current states in state_names() order — NaN for unset.
    [[nodiscard]] std::vector<double> state() const;

    // self.snapshots / self.diagnostics — empty until the first advancing
    // compute commits them.
    [[nodiscard]] const std::array<std::optional<TireSnapshot>, 2> &
    snapshots() const { return snapshots_; }

    [[nodiscard]] const std::array<std::optional<TireDiagnostics>, 2> &
    diagnostics() const { return diagnostics_; }

    // Post-commit counters (tire_forces.py:203-205).
    [[nodiscard]] double elastic_energy_j() const {
        return elastic_energy_j_;
    }

    [[nodiscard]] double brush_loss_step_j() const {
        return brush_loss_step_j_;
    }

    [[nodiscard]] double radial_dissipation_power_w() const {
        return radial_dissipation_power_w_;
    }

private:
    const mjModel *m_; // non-owning; the Stepper outlives it
    nativecfg::TireConfig cfg_;
    int nv_;
    // optional so the ctor can honor __init__'s validation order: the
    // surface_map gate (tire_forces.py:68-70) precedes the profile build.
    std::optional<biketyre::ProfileQuery> profile_;
    std::optional<nativecfg::SurfaceMap> surface_map_;
    // geoms/bodies/radii per ('front','rear') insertion order.
    std::array<int, 2> geoms_{}, bodies_{};
    std::array<double, 2> radii_{};
    // self._jac_contact / self._jac_center (3,nv) scratch.
    std::vector<mjtNum> jac_contact_, jac_center_;
    // Persistent writer state (reset() — tire_forces.py:91-98).
    std::array<BrushState, 2> states_;
    std::array<std::optional<TireSnapshot>, 2> snapshots_;
    std::array<std::optional<TireDiagnostics>, 2> diagnostics_;
    double elastic_energy_j_ = 0.0;
    double brush_loss_step_j_ = 0.0;
    double radial_dissipation_power_w_ = 0.0;
    std::optional<double> last_time_s_;
};
