// writers/tire.hpp — port of TireForceApplier (src/bike_sim/sim/ride/
// tire_forces.py) — the only stateful force writer: per-wheel _BrushState
// evolves inside compute_qfrc and commits only after BOTH wheels evaluate.
// The flattened-state schema is the artifact's `tire_state_names` layout
// (flatten_row(asdict(_BrushState)) per side, NaN for unset fields).
#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "../config_types.hpp"
#include "../rtsan.hpp"
#include "../tyre/profile.hpp"
#include "writer_types.hpp"

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

    // R1 allocation-free face. compute_into is the checked convenience
    // boundary — std::invalid_argument on a null sink, a null mjData, or
    // a wrong output width — then runs qfrc()'s advance+commit verbatim
    // on the caller-owned span. try_compute_into is the prevalidated
    // warm-core entry: identical preconditions (width, dt domain,
    // double-advance) reported through CoreStatus, and no throw/string
    // construction or allocation in its own code (residual kernel
    // rejections are caught and mapped — see the .cpp), so it is proved
    // noexcept. biketyre::ProfileQuery::contact now refills
    // construction-sized member scratch, so a valid tick is allocation-free
    // end to end and try_compute_into carries BIKE_NONBLOCKING.
    void compute_into(const mjData *d, double dt, std::span<double> out);
    [[nodiscard]] CoreStatus
    try_compute_into(const mjData *d, double dt,
                     std::span<double> out) noexcept BIKE_NONBLOCKING;

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
    // compute commits them. `committed_` carries that observable empty
    // state: the storage below stays engaged and construction-sized, so
    // the marked commit assigns into capacity instead of constructing the
    // contained object inside a disengaged optional — the former
    // first-commit engagement allocated the patch vector and the surface
    // string inside the marked warm core.
    [[nodiscard]] std::array<std::optional<TireSnapshot>, 2>
    snapshots() const {
        std::array<std::optional<TireSnapshot>, 2> out{};
        if (committed_)
            for (std::size_t i = 0; i < 2; ++i)
                out[i] = snapshots_[i];
        return out;
    }

    [[nodiscard]] std::array<std::optional<TireDiagnostics>, 2>
    diagnostics() const {
        std::array<std::optional<TireDiagnostics>, 2> out{};
        if (committed_)
            for (std::size_t i = 0; i < 2; ++i)
                out[i] = diagnostics_[i];
        return out;
    }

    // ---- probe path (compute_qfrc advance=False, tire_forces.py:108-113) --
    // A detached writer copy evaluates one ADVANCING compute on a cleared
    // clock (copy.copy + deepcopy(states) + last_time_s=None in the
    // oracle); only its snapshots cross back into probe_snapshots. The
    // caller's brush states, clock, energies, committed snapshots and
    // diagnostics stay untouched — including when the detached eval
    // throws. This is a cold path (init probe + advance=False steps):
    // the detached copy duplicates the construction-sized scratch, so no
    // try_* nonblocking face exists here.
    [[nodiscard]] std::vector<double> probe_qfrc(const mjData *d, double dt);
    void probe_compute_into(const mjData *d, double dt,
                            std::span<double> out);
    [[nodiscard]] std::array<std::optional<TireSnapshot>, 2>
    probe_snapshots() const {
        std::array<std::optional<TireSnapshot>, 2> out{};
        if (probe_committed_)
            for (std::size_t i = 0; i < 2; ++i)
                out[i] = probe_snapshots_[i];
        return out;
    }

    // self.radii — {'front':.., 'rear':..} in that order
    // (tire_forces.py:126).
    [[nodiscard]] const std::array<double, 2> &radii() const {
        return radii_;
    }

    // restart_clock() (tire_forces.py:169-171): `last_time_s = None`
    // without touching brush state — the init/restore re-arm.
    void restart_clock() noexcept { last_time_s_.reset(); }

    // stored_energy(model, data) (tire_forces.py:173-184): elastic +
    // shear energy at the CURRENT geometry — the patch delta comes from
    // a fresh profile contact, the shear term from the stored brush.
    [[nodiscard]] double stored_energy(const mjData *d) const;

    // Raw committed storage for internal readers (the runtime step
    // borrows these spans — the boxed accessors above stay the copying
    // boundary). The committed_ flag is the publication gate.
    [[nodiscard]] const std::array<TireSnapshot, 2> &
    snapshot_storage() const noexcept { return snapshots_; }
    [[nodiscard]] bool snapshots_committed() const noexcept {
        return committed_;
    }
    [[nodiscard]] const std::array<TireDiagnostics, 2> &
    diagnostic_storage() const noexcept { return diagnostics_; }
    [[nodiscard]] const std::array<TireSnapshot, 2> &
    probe_snapshot_storage() const noexcept { return probe_snapshots_; }
    [[nodiscard]] bool probe_committed() const noexcept {
        return probe_committed_;
    }

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
    const mjModel *m_ = nullptr; // non-owning; the Stepper outlives it
    nativecfg::TireConfig cfg_;
    int nv_ = 0;
    // optional so the ctor can honor __init__'s validation order: the
    // surface_map gate (tire_forces.py:68-70) precedes the profile build.
    std::optional<biketyre::ProfileQuery> profile_;
    std::optional<nativecfg::SurfaceMap> surface_map_;
    // geoms/bodies/radii per ('front','rear') insertion order.
    std::array<int, 2> geoms_{}, bodies_{};
    std::array<double, 2> radii_{};
    // self._jac_contact / self._jac_center (3,nv) scratch.
    std::vector<mjtNum> jac_contact_, jac_center_;
    // qfrc()'s body verbatim on caller-owned storage and the persistent
    // stage members below: the interval/double-advance gates, per-side
    // force evaluation, staged snapshot/diagnostics builds, then the
    // atomic commit. Kept throwing so both boundaries share one kernel
    // and the original rejection types reach the caller.
    void accumulate_validated(const mjData *d, double dt,
                              std::span<double> out);

    // R1: per-tick scratch moved out of compute_qfrc — tmp_ is the
    // jac_contact.T @ force_world gemv target and qfrc_out_ the boxing
    // surface's zeros(nv) buffer; both are construction-sized (nv) and
    // filled, never resized, each tick.
    std::vector<double> tmp_, qfrc_out_;
    // Atomic staging for the committed slots: the per-side snapshot /
    // diagnostics are built here during evaluation and copied into
    // snapshots_/diagnostics_ only after BOTH wheels succeed — the same
    // commit boundary the original locals gave. Stage members persist so
    // their string/vector capacity is reused across ticks instead of
    // reallocating (the copy-assign into the committed slots reuses
    // capacity too).
    std::array<TireSnapshot, 2> snapshot_stage_{};
    std::array<TireDiagnostics, 2> diagnostics_stage_{};
    // Patch capacity the snapshot stage reserves at construction: the
    // compliant_2d backend emits at most one contact patch per wheel per
    // tick, so the guard in accumulate_validated can never trip today —
    // it exists so a future multi-patch backend must raise this bound
    // deliberately rather than silently reallocate inside compute.
    static constexpr std::size_t kMaxSnapshotPatches = 1;
    // Persistent writer state (reset() — tire_forces.py:91-98). The
    // committed slots stay engaged for the writer's whole life so the
    // marked commit is a pure assign-into-capacity; `committed_` is the
    // "empty until the first advancing compute" state the accessors
    // project into the optional view.
    std::array<BrushState, 2> states_;
    std::array<TireSnapshot, 2> snapshots_{};
    std::array<TireDiagnostics, 2> diagnostics_{};
    bool committed_ = false;
    // self.probe_snapshots — published only by the advance=False probe;
    // reset() deliberately does not touch it (the oracle never clears it
    // either — the attribute persists across reset()).
    std::array<TireSnapshot, 2> probe_snapshots_{};
    bool probe_committed_ = false;
    double elastic_energy_j_ = 0.0;
    double brush_loss_step_j_ = 0.0;
    double radial_dissipation_power_w_ = 0.0;
    std::optional<double> last_time_s_;
};
