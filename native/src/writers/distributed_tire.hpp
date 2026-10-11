// writers/distributed_tire.hpp — native port of `DistributedTireForceApplier`
// (src/bike_sim/sim/ride/distributed_tire_forces.py) — the distributed
// per-station tyre backend. Task 4 of
// docs/superpowers/plans/2026-10-11-distributed-tire-b-native-port.md:
// construction, the canonical state wire, stored energy, and the
// reset/clock/snapshot/diagnostic scaffolding. The station tick
// (`compute_into`/`try_compute_into`/`probe_*`) is a stub here and lands in
// Task 5; `Stepper` dispatch is Task 5 as well — until then the debug factory
// `bike_native._debug_distributed_writer` in binding.cpp is the only entry
// point.
//
// Divergence guide for the surrounding modules:
//  - writers/distributed_tire.cpp — this implementation; notes that affect
//    its external contract say "CONTRACT".
//  - src/bike_sim/sim/ride/distributed_tire_forces.py — the behavioral oracle.
//  - distributed-wire parity lives in
//    tests/reference/test_native_distributed_tire.py — if construction, the
//    wire, or stored energy disagree with the oracle, that suite is the
//    arbiter.
#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

#include "../config_types.hpp"
#include "../rtsan.hpp"
#include "../tyre/distance.hpp"
#include "../tyre/station.hpp"
#include "tire_common.hpp"
#include "writer_types.hpp"

// Per-station state — the C++ equivalent of the Python `_StationState`
// dataclass. `active` mirrors presence/absence in the Python `states` dict:
// an inactive station carries no `xi`/`tangent` at all and writes four NaNs
// on the wire. `tangent` is the unit scale direction from the previous tick;
// it is *not* a state variable in the Python applier — the tick computes the
// tangent — but the state wire carries it so a restore can reproduce it.
struct StationState {
    bool active = false;
    double xi = 0.0;
    std::optional<std::array<double, 3>> tangent;
};

// Distributed tyre writer. Shares the `TireWriter` scaffolding conventions
// (interval/clock gates, snapshot/diagnostic staging, probe isolation,
// stored-energy surface) but replaces the single compliant contact with
// `station_count` per-wheel stations under their own materials and density —
// the Python `DistributedTireForceApplier` class contract.
class DistributedTireWriter {
public:
    // Resolves the contact geoms, wheel bodies, wheel-axis radii, and the
    // station grid exactly like `TireWriter`, then builds the per-wheel local
    // station coordinates `r * (cos(angle), 0, sin(angle))` and sizes every
    // scratch container from `station_count`. `d` seeds the terrain profile
    // (`mj_forward` first) — the writer does not own the model or data, so the
    // context must outlive it.
    DistributedTireWriter(const mjModel *m, const mjData *d,
                          nativecfg::TireConfig config);

    // CONTRACT — distributed state wire (mirrors Python `set_state`/`state_dict`):
    //
    // Canonical columns are `f'{side}.s{index:03d}.{leaf}'` for
    // side ∈ {front, rear}, index ∈ 0..N-1, leaf ∈ {xi, tangent.0..2} —
    // exactly `2 * station_count * 4` names in side-major, station-major,
    // leaf-major order, cached at construction by `state_names()`.
    //
    // `set_state` accepts a *canonical row only*: `names` must be the full
    // canonical set (any order), `row` must have the same length, and every
    // canonical column must be present exactly once — unknown columns,
    // duplicate columns, and missing columns all reject. The Python oracle is
    // asymmetric (its shape check compares the row to the *unique-name*
    // count, which accidentally accepts some duplicate/short rows); this port
    // implements the strictest reading, rejecting every input the oracle
    // could not meaningfully restore. Every rejection preserves the existing
    // station states and clock — decode happens on a staged copy and commits
    // only after full validation. A successful restore restarts the interval
    // clock like the Python `last_time_s = None`.
    //
    // Column semantics: NaN `xi` means the station is inactive; a finite `xi`
    // activates it. A tangent triple of all NaN means unset; a fully finite
    // triple is one unit-scale direction. STRICHER THAN PYTHON: a partially
    // nonfinite tangent (any component nonfinite but not all three NaN)
    // rejects the entire row — the oracle would store NaN components as-is.
    // Validation is unconditional: a partial tangent rejects even beside a
    // NaN `xi`, and a non-NaN nonfinite `xi` (±inf) rejects rather than
    // storing a degenerate station.
    //
    // `state()` emits `state_names()`' canonical ordering: four NaNs for an
    // inactive station; finite `xi` plus three NaNs for an unset tangent;
    // finite `xi` plus the three stored components for a set tangent.
    void set_state(std::span<const std::string> names,
                   std::span<const double> row);
    [[nodiscard]] const std::vector<std::string> &state_names() const;
    [[nodiscard]] std::vector<double> state() const;

    void reset();

    // restart_clock() (distributed_tire_forces.py:64-65): `last_time_s = None`
    // without touching station state — the init/restore re-arm.
    void restart_clock() noexcept { last_time_s_.reset(); }

    // CONTRACT — the distributed tick is Task 5. `compute_into` validates the
    // live-data/width preconditions like `TireWriter` then throws
    // `std::logic_error` until the station loop lands; `try_compute_into`
    // keeps the final noexcept shape (precondition failures return
    // `CoreStatus::invalid_input`) and maps the stub throw. Both signatures
    // and the allocation-free structure are final — Task 5 fills only the
    // `accumulate_validated` kernel.
    [[nodiscard]] std::vector<double> qfrc(const mjData *d, double dt);
    void compute_into(const mjData *d, double dt, std::span<double> out);
    [[nodiscard]] CoreStatus try_compute_into(
        const mjData *d, double dt,
        std::span<double> out) noexcept BIKE_NONBLOCKING;

    // Probe evaluates qfrc on an isolated copy: the live writer's states and
    // clock are untouched while `probe_snapshots()` exposes what the probe
    // would have committed. Stages patch capacity for the real tick.
    [[nodiscard]] std::vector<double> probe_qfrc(const mjData *d, double dt);
    void probe_compute_into(const mjData *d, double dt, std::span<double> out);

    // CONTRACT — distributed stored energy. Mirrors
    // `DistributedTireForceApplier.stored_energy`: per side in {front, rear},
    // each wheel's stations are lifted to world points
    // (`center + local @ R.T`), the radial density energy
    // `weights[i] * density(interp(-deflection))` accumulates for every
    // penetrator, then each active station's shear energy
    // `0.5 * tangent_k * weights[i] * xi**2` accumulates — in the documented
    // side-major, station-ascending scalar order.
    [[nodiscard]] double stored_energy(const mjData *d) const;

    // self.snapshots / self.diagnostics — empty until the first advancing
    // compute commits them (same `committed_` publication gate as
    // TireWriter). Before the Task-5 tick nothing commits, so these project
    // two disengaged slots.
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

    // Raw committed storage for internal readers — `committed_` is the
    // publication gate (the TireWriter convention).
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

    // Diagnostic surfaces the outer step accounting reads. Each holds the
    // value from the most recently committed advance; resets to zero on
    // `reset()`.
    [[nodiscard]] double elastic_energy_j() const noexcept {
        return elastic_energy_j_;
    }
    [[nodiscard]] double brush_loss_step_j() const noexcept {
        return brush_loss_step_j_;
    }
    [[nodiscard]] double radial_dissipation_power_w() const noexcept {
        return radial_dissipation_power_w_;
    }

    // self.radii — {'front':.., 'rear':..} in that order
    // (distributed_tire_forces.py:49).
    [[nodiscard]] const std::array<double, 2> &radii() const noexcept {
        return radii_;
    }

private:
    void accumulate_validated(const mjData *d, double dt,
                              std::span<double> out);
    // The per-wheel geometry transform shared by stored_energy and the
    // future tick: lift the local station points to world coordinates into
    // `points_`, evaluate the terrain height at each into `heights_`, and
    // collect the penetrating station indices into `active_ids_` (prefix
    // length returned). CONST — used by the const stored_energy path, so
    // the scratch it fills is mutable.
    [[nodiscard]] std::size_t station_points(const mjData *d,
                                             std::size_t side) const;

    const mjModel *m_ = nullptr;
    nativecfg::TireConfig cfg_;
    int nv_ = 0;
    std::optional<biketyre::SignedProfile> profile_;
    std::optional<nativecfg::SurfaceMap> surface_map_;
    std::array<int, 2> geoms_{};
    std::array<int, 2> bodies_{};
    std::array<double, 2> radii_{};
    biketyre::StationAngles stations_;
    // Per-side station local coordinates `(r cos a, 0, r sin a)` — owned, no
    // self-referential pointers so the writer stays copyable for the probe.
    std::array<std::vector<std::array<double, 3>>, 2> local_{};
    std::array<std::vector<StationState>, 2> states_{};
    std::array<std::vector<StationState>, 2> states_stage_{};

    // Wire scaffolding — sized once at construction.
    std::vector<std::string> state_names_{};
    std::unordered_map<std::string, std::size_t> canonical_{};
    std::vector<double> wire_cols_{};
    std::vector<char> wire_seen_{};

    // Tick/stored-energy scratch — construction-sized so the Task-5 warm core
    // and the stored-energy path never allocate. Mutable because
    // `stored_energy` is a const accessor.
    mutable std::vector<double> points_{};    // N*3 world station points
    mutable std::vector<double> heights_{};   // N terrain heights
    mutable std::vector<int> active_ids_{};   // N penetrating ids (prefix)
    std::vector<mjtNum> jac_center_{};        // 3*nv (mj_jac position half)
    std::vector<mjtNum> jr_{};                // 3*nv (mj_jac rotation half)
    std::vector<mjtNum> jac_station_{};       // 3*nv
    std::vector<double> loads_{};             // N
    std::vector<double> energies_{};          // N
    std::vector<double> depths_{};            // N
    std::vector<double> normals_{};           // N*3
    std::vector<double> tmp_{};               // nv
    std::vector<double> qfrc_out_{};          // nv
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
