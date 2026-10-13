// writers/pneumatic_tire.hpp — native port of `PneumaticTireForceApplier`
// (src/bike_sim/sim/ride/pneumatic_tire_forces.py) — the radial-carcass
// pneumatic tyre backend (tire.backend == "pneumatic"). Task 11 of the
// native tyre plan: the Stepper selects this writer for the pneumatic
// backend; it composes the Task-10 per-wheel kernels
// (pneumatic/model.hpp) instead of reimplementing tyre mechanics.
//
// Divergence guide for the surrounding modules:
//  - writers/pneumatic_tire.cpp — this implementation; notes that
//    affect its external contract say "CONTRACT".
//  - src/bike_sim/sim/ride/pneumatic_tire_forces.py — the behavioral
//    oracle (line numbers cited below were ported at that revision).
//  - tests/reference/test_native_pneumatic_tire.py — the bitwise
//    arbiter: qfrc, snapshots, diagnostics, stored energy and the state
//    wire against the live oracle.
#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

#include "../config_types.hpp"
#include "../pneumatic/model.hpp"   // PneumaticTyreModel, PatchBrushState
#include "../pneumatic/profile.hpp" // RoadProfile
#include "../rtsan.hpp"
#include "tire_common.hpp"
#include "writer_types.hpp"

// Pneumatic tyre writer. Shares the TireWriter/DistributedTireWriter
// scaffolding conventions — interval/clock gates, staged
// snapshot/diagnostics committed only after BOTH wheels evaluate, probe
// isolation, the stored-energy surface, CoreStatus warm entry — while the
// per-wheel state lives in two PneumaticTyreModel instances plus their
// construction-sized rollback/probe clones.
//
// The writer is non-copyable: the models own lazily-built brush engines
// (unique_ptr rows) and the probe path evaluates on dedicated member
// clones instead of duplicating the writer (the oracle's
// copy.copy + deepcopy(states) decomposed — the probe clones carry the
// deep copy and the writer's own scaffolding is immutable during probe).
class PneumaticTireWriter {
public:
    // __init__ (pneumatic_tire_forces.py:77-122): the pneumatic-config,
    // backend, profile and surface-map gates in the oracle's order, then
    // _resolve_wheel per side (body/joint/geom resolution, sphere and
    // radius cross-checks), the all-wheel-geoms collision gate, and the
    // PneumaticTyre state objects — here the live/backup/probe model
    // triplets. `d` is unused by construction (the oracle reads no
    // mjData fields at build time either) but is kept in the signature
    // for writer-surface symmetry.
    PneumaticTireWriter(const mjModel *m, const mjData *d,
                        nativecfg::TireConfig config);

    PneumaticTireWriter(const PneumaticTireWriter &) = delete;
    PneumaticTireWriter &operator=(const PneumaticTireWriter &) = delete;
    // The writer never moves either: Stepper anchors it by unique_ptr
    // and the probe path works on dedicated model clones instead of
    // relocating writer state, so no use site exists — rule-of-5
    // declarations keep the whole family explicit.
    PneumaticTireWriter(PneumaticTireWriter &&) = delete;
    PneumaticTireWriter &operator=(PneumaticTireWriter &&) = delete;
    ~PneumaticTireWriter() = default;

    // CONTRACT — the pneumatic state wire (mirrors Python
    // set_state/state_dict/state_names):
    //
    // Canonical columns per side in ('front','rear') order —
    // `f'{side}.ray.{i:03d}.{leaf}'` for leaf in
    // (delta_prev_m, elastic_prev_n, maxwell_n, bristle_m), then
    // `{side}.carcass.initialized`, `{side}.rim.{active|x_m|speed_mps|
    // peak_load_n|peak_force_n|energy_j}`, `{side}.patch.count`, and
    // `{side}.patch.{k:02d}.{start|stop|kappa_prime}` for
    // k < max_patches — `2 * (4*n_rays + 8 + 3*max_patches)` names,
    // cached at construction.
    //
    // `set_state` accepts a canonical row only: every canonical column
    // exactly once (unknown/duplicate/missing all reject — the oracle's
    // shape/coverage checks admit some malformed rows incidentally; this
    // port takes the strictest reading like DistributedTireWriter).
    // Decode validates every cell before a single model commit runs
    // (two-phase), so a rejected row leaves the carcass/patch state and
    // the interval clock byte-identical. A successful restore clears
    // `last_time_s_` like the oracle's `self.last_time_s = None`.
    //
    // Column semantics: ray delta/elastic/maxwell cells must be finite;
    // bristle cells carry the detailed brush's per-ray deflection (NaN
    // outside patches, everywhere-NaN on the lumped tier);
    // carcass.initialized and rim.active are {0,1} flags; rim fields
    // finite iff active, NaN iff inactive; patch.count is a nonnegative
    // integer <= max_patches with disjoint ascending (start, stop) runs
    // and NaN past-count slots. Every rejection is std::runtime_error —
    // the pinned RuntimeError surface of the native wire contract — with
    // the oracle's ValueError message texts preserved.
    void set_state(std::span<const std::string> names,
                   std::span<const double> row);
    [[nodiscard]] const std::vector<std::string> &state_names() const;
    [[nodiscard]] std::vector<double> state() const;

    // reset (pneumatic_tire_forces.py:124-130): per-tyre reset, cleared
    // snapshots/diagnostics, zeroed energy counters, disarmed clock. The
    // probe publication stays untouched, like the oracle.
    void reset();

    // restart_clock (pneumatic_tire_forces.py:132-133) —
    // `last_time_s = None` without touching tyre state.
    void restart_clock() noexcept { last_time_s_.reset(); }

    // CONTRACT — the pneumatic tick. `compute_into` validates the
    // live-data/width preconditions like the other writers, then runs
    // compute_qfrc(advance=True) verbatim: dt/once-per-timestamp gates,
    // a semantic backup of both tyre models, the front-then-rear wheel
    // loop (mj_objectVelocity → PneumaticTyreModel::evaluate →
    // patch-centroid mj_jac force mapping → staged ContactPatch /
    // WheelContactSnapshot / diagnostics / energy folds), the derived
    // overflow gates, and the atomic commit. `try_compute_into` reports
    // the same preconditions through CoreStatus, catches residual
    // rejections and maps them, so the valid warm path is noexcept and
    // allocation-free (BIKE_NONBLOCKING) end to end — the brushes are
    // prewarmed at construction and every buffer is construction-sized.
    [[nodiscard]] std::vector<double> qfrc(const mjData *d, double dt);
    void compute_into(const mjData *d, double dt, std::span<double> out);
    [[nodiscard]] CoreStatus try_compute_into(
            const mjData *d, double dt,
            std::span<double> out) noexcept BIKE_NONBLOCKING;

    // compute_qfrc(advance=False) (pneumatic_tire_forces.py:349-355):
    // evaluates on construction-made probe clones freshly assign()ed from
    // the live models, on a cleared clock; only the committed snapshots
    // cross back. Live state, clock, energies and committed artifacts
    // stay untouched — including when the probe evaluation throws. Cold
    // path: the clone assignment and lazy brush creation allocate.
    [[nodiscard]] std::vector<double> probe_qfrc(const mjData *d,
                                                 double dt);
    void probe_compute_into(const mjData *d, double dt,
                            std::span<double> out);

    // stored_energy (pneumatic_tire_forces.py:332-346): per side in
    // front/rear order, a fresh ray/road intersect at the CURRENT pose
    // feeds `_radial_energy_j`; `_shear_energy_j` folds committed-patch
    // bristle energy on the detailed tier. Same expression the committed
    // diagnostics carry, so post-commit `elastic_energy_j` agrees
    // bitwise.
    [[nodiscard]] double stored_energy(const mjData *d) const;

    // self.snapshots / self.diagnostics — empty until the first
    // advancing compute commits them; `committed_` is the publication
    // gate (the TireWriter convention: storage stays engaged and
    // construction-sized).
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

    // self.probe_snapshots — published only by the advance=False probe.
    [[nodiscard]] std::array<std::optional<TireSnapshot>, 2>
    probe_snapshots() const {
        std::array<std::optional<TireSnapshot>, 2> out{};
        if (probe_committed_)
            for (std::size_t i = 0; i < 2; ++i)
                out[i] = probe_snapshots_[i];
        return out;
    }

    // Post-commit counters (pneumatic_tire_forces.py:472-474).
    [[nodiscard]] double elastic_energy_j() const noexcept {
        return elastic_energy_j_;
    }
    [[nodiscard]] double brush_loss_step_j() const noexcept {
        return brush_loss_step_j_;
    }
    [[nodiscard]] double radial_dissipation_power_w() const noexcept {
        return radial_dissipation_power_w_;
    }

    // self.radii — {'front':.., 'rear':..} geom sphere radii in that
    // order (pneumatic_tire_forces.py:113-114).
    [[nodiscard]] const std::array<double, 2> &radii() const noexcept {
        return radii_;
    }

private:
    // The advancing compute_qfrc body after its precondition gates:
    // backup, two-wheel evaluation, derived gates, rollback, commit.
    void accumulate_validated(const mjData *d, double dt,
                              std::span<double> out);

    // The oracle's per-side loop (pneumatic_tire_forces.py:368-463) plus
    // its tail derived_* gates (:464-467) — shared verbatim by the
    // advancing path (live models) and the probe (clones). Writes the
    // staged snapshots/diagnostics members and reports the three energy
    // folds through the out-references; throws where the oracle raises.
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) out-params mirror the oracle's three accumulators
    void evaluate_wheels(
            const mjData *d, double dt, double time,
            std::span<double> out,
            const std::array<std::unique_ptr<biketyre::PneumaticTyreModel>,
                             2> &tyres,
            double &energy_out, double &brush_loss_out,
            double &radial_power_out);
    // NOLINTEND(bugprone-easily-swappable-parameters)

    // _radial_energy_j (pneumatic_tire_forces.py:57-66) over a fresh
    // intersect's delta row — the `0.5*(k_c*δ² + k_rim*overlap²)*ds`
    // elementwise pair folded by np.sum.
    [[nodiscard]] double radial_energy_j(
            const biketyre::PneumaticTyreModel &tyre,
            std::span<const double> delta_m) const;

    // _shear_energy_j (pneumatic_tire_forces.py:305-330): the detailed
    // tier's committed-patch bristle energy; 0 on the lumped tier and
    // for patch ranges without a committed output.
    [[nodiscard]] double
    shear_energy_j(const biketyre::PneumaticTyreModel &tyre) const;

    // _intersect (pneumatic_tire_forces.py:295-303) into the shared
    // hit-buffer scratch — a fresh intersect at the given hub.
    void intersect_at(const biketyre::PneumaticTyreModel &tyre,
                      double hub_x_m, double hub_z_m,
                      biketyre::RayHitBuffers &out) const;

    const mjModel *m_ = nullptr; // non-owning; the Stepper outlives it
    nativecfg::TireConfig cfg_;
    int nv_ = 0;
    std::int64_t n_rays_ = 0;
    std::size_t max_patches_ = 0;
    bool discretised_ = false;
    // The resolved road profile + surface map (uniform from the
    // configured pneumatic surface, else the track map). The profile is
    // engaged once at construction — the writer never runs without it.
    std::optional<biketyre::RoadProfile> profile_;
    nativecfg::SurfaceMap surface_map_;
    // _WheelBinding fields per ('front','rear') insertion order.
    std::array<int, 2> bodies_{}, geoms_{}, spin_dofs_{};
    std::array<double, 2> radii_{};
    // self.states plus the two construction-sized clone banks: backup_
    // is the deepcopy rollback the oracle makes before every advancing
    // step; probe_ is the detached-copy evaluation bank the probe path
    // reassigns from the live models on every call.
    std::array<std::unique_ptr<biketyre::PneumaticTyreModel>, 2> tyres_,
            backup_, probe_;

    // Wire scaffolding — sized once at construction.
    std::vector<std::string> state_names_{};
    std::unordered_map<std::string, std::size_t> canonical_{};
    std::vector<double> wire_cols_{};
    std::vector<char> wire_seen_{};
    // set_state decode staging: per side the four ray rows, the covered
    // mask and the patch-state row — decode-only members, committed via
    // reset()+writes after full validation.
    std::array<std::vector<double>, 2> dec_delta_, dec_elastic_,
            dec_maxwell_, dec_bristle_;
    std::array<std::vector<char>, 2> dec_covered_;
    std::array<std::vector<biketyre::PatchBrushState>, 2> patch_stage_;

    // Tick scratch — construction-sized; the mutable rows also serve the
    // const stored_energy path.
    std::array<mjtNum, 6> velocity6_{};
    std::vector<mjtNum> jacp_{};  // 3*nv patch-centroid Jacobian
    std::vector<double> tmp_{};   // nv gemv target
    std::vector<double> qfrc_out_{};
    mutable std::vector<double> hit_r_, hit_delta_, hit_rx_, hit_rz_;
    mutable std::vector<double> energy_scratch_;  // radial terms (n)
    mutable std::vector<double> shear_scratch_;   // bristle² row (n)
    mutable std::vector<double> bristle_row_;     // state() assembly (n)

    // Commit staging — the oracle builds fresh dicts in locals; these
    // stage members persist so their capacities are reused across ticks
    // and the marked commit stays a pure assign-into-capacity.
    std::array<TireSnapshot, 2> snapshot_stage_{};
    std::array<TireDiagnostics, 2> diagnostics_stage_{};

    std::array<TireSnapshot, 2> snapshots_{};
    std::array<TireDiagnostics, 2> diagnostics_{};
    bool committed_ = false;
    std::array<TireSnapshot, 2> probe_snapshots_{};
    bool probe_committed_ = false;
    double elastic_energy_j_ = 0.0;
    double brush_loss_step_j_ = 0.0;
    double radial_dissipation_power_w_ = 0.0;
    std::optional<double> last_time_s_;
};
