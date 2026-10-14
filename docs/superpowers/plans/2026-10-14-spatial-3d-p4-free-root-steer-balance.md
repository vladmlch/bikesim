# Spatial 3D — Phase 4 (Track D): Free Root, Steering, Balance Controller, C++ Equilibrium Implementation Plan

> **For agentic workers:** Execute task-by-task with your runtime's plan-execution skill; steps use checkbox (`- [ ]`) syntax. **Prerequisites:** Phases 1 and 3 merged. `examples/benchmarks/meijaard2007.toml` pinned (P0 Task 2). Read spec §7.2, §8.2–8.3, §9.1–9.2 and ADR 0002 Layer A before Task 1. This phase introduces the first physically unstable configuration; keep every test's perturbations small and durations short.

**Goal:** `spatial = "3d", root = "free"`: freejoint chassis and rider root, `steer_joint` with the `steer_torque` actuator, a speed-scheduled roll/heading balance controller in the rider module with configurable reaction delay and torque cap, a C++ equilibrium/bootstrap with a bootstrap-only hold and an explicit settle budget, new episode outcomes `crash:capsize`/`crash:off_corridor`, and the two external benchmarks passing: **Meijaard 2007** (3 % eigenvalues, ±0.2 m/s stability boundaries) and **steady cornering** (1° lean, 2 % lateral force). Mirror symmetry is bitwise.

**Architecture:** `topology::Spatial::free6` activates the `nq = nv + 1` branches declared in P1 (`mj_integratePos` already handles quaternions; the only new code is address handling and ISO Euler extraction in `samples.cpp`). `rider/steer_balance.{hpp,cpp}` computes the steer torque per control tick, with a FIFO delay in ticks and a cap; it writes `ctrl[steer_act]`. `runtime/equilibrium.{hpp,cpp}` ports the Python settle/refine loop (Accelerate `DGELSD` for the least squares); `runtime/bootstrap_hold.{hpp,cpp}` applies the roll/yaw/lateral virtual spring **only while bootstrap time < 0** and asserts zero afterwards. The builder gains `mode = "benchmark_whipple"`, assembling the Meijaard bicycle from the 27 parameters with the same 3D machinery. Mode identification for the benchmark uses a matrix-pencil fit in Python (test-side, NumPy).

**Tech Stack:** as master plan; `native` per task, `full` at gate; NumPy/SciPy only in tests.

**Spec:** §4.1–4.2, §7.2, §8.1–8.3, §9.1–9.2, §9.4, §9.6. **ADR:** 0002 Layers A, B, D.

## Global Constraints

- All master-plan constraints. Branch `feat/spatial-3d-p4-free-root`.
- Planar and constrained-root bytes unchanged (P1 regression bridge and lateral-zero tests stay green at every task).
- **No world forces during scored steps:** `bootstrap.hold_active` is asserted `0` for every `t ≥ 0` sample in every test of this phase; the energy ledger shows zero bootstrap-hold work after `t = 0`.
- Controller gains are config, defaults from spec §4.1; tests pin the defaults and must be updated explicitly if defaults change.
- Benchmark values are read from `meijaard2007.toml`; tests never embed literature numbers.
- dt study (Task 7) decides the fast-tier dt for free-root runs; until then tests use dt 0.25 ms for benchmarks and 1.25 ms for the ride model.

---

## File Structure

| File | Responsibility |
|---|---|
| Modify `src/bike_sim/mujoco/frame.py`, `articulated_rider.py` | `root = "free"`: `<freejoint name="root_free"/>`, `rider_root_free` |
| Modify `src/bike_sim/mujoco/steering_fork.py:36-39` | free root keeps `steer_joint` (range ±`range_deg`, damping), adds `steer_torque` motor actuator |
| Modify `src/bike_sim/mujoco/physical_topology.py:38-49` | steer removal only for `root != "free"` |
| Create `src/bike_sim/mujoco/benchmark_whipple.py` | Meijaard bicycle MJCF from 27 parameters |
| Modify `src/bike_sim/mujoco/builder.py` | `mode = "benchmark_whipple"` |
| Modify `src/bike_sim/physics/model_config.py` | free-root validation; `steer` config complete |
| Modify `native/src/spatial_topology.cpp` | `free6` addresses, `root_pitch` from quaternion (ISO), `root_x` from `qpos[adr]` |
| Modify `native/src/runtime/samples.cpp` | `root.quat_wxyz`, ISO roll/pitch/yaw, `steer.*`, `bootstrap.hold_active` |
| Create `native/src/rider/steer_balance.{hpp,cpp}` | Controller with schedule, delay FIFO, cap |
| Create `native/src/runtime/equilibrium.{hpp,cpp}` | Settle + refine port with time budget |
| Create `native/src/runtime/bootstrap_hold.{hpp,cpp}` | Bootstrap-only virtual hold writer (distinct from the existing `runtime/bootstrap.hpp`, which is the t=0 state-inventory wire; do not merge them) |
| Modify `native/src/runtime/step.cpp`, `status.cpp`, `research.cpp`, `research_metrics.cpp` | Controller tick; outcomes `crash:capsize`, `crash:off_corridor`; hold gating |
| Modify `native/src/binding.cpp`, `src/bike_sim/native/{runtime,setup,research}.py` | Bootstrap entry moves to native (`bootstrap(config) -> state`); Python equilibrium bypassed for 3D; cache fingerprint fields |
| Modify `src/bike_sim/sim/ride/equilibrium_cache.py` | 3D fingerprint (`spatial`, terrain sha256, tyre tier/coefficients, imu mount); stores the 3D state wire |
| Modify `src/bike_sim/sim/research/metrics.py`, `sim/ride/wheelie.py` | New outcomes; truth roll/yaw (produced here, consumed by P5) |
| Create `tests/reference/test_spatial3d_free_root.py` | Topology, steer actuator, controller unit behaviour, outcomes, bootstrap hold zero |
| Create `tests/reference/test_spatial3d_equilibrium.py` | Settle convergence on fast/detailed tiers; cache round-trip; budget respected |
| Create `tests/reference/test_spatial3d_meijaard.py` | Layer A gate |
| Create `tests/reference/test_spatial3d_cornering.py` | Layer A gate |
| Modify `tests/reference/test_spatial3d_invariants.py` | Mirror symmetry (bitwise) |
| Create `tests/reference/golden_manifests/spatial3d/{free_rocky_10s,meijaard_v6_3s}.json` | Self-golden manifests |
| Create `tools/modal_fit.py` | Matrix-pencil modal identification (test helper) |
| Create `docs/superpowers/audits/2026-10-14-spatial-3d-p4.md` | Evidence |

---

### Task 1: Free root in builder and native topology

**Files:** builder files above; `native/src/spatial_topology.cpp`, `runtime/samples.cpp`; `tests/reference/test_spatial3d_free_root.py`

- [ ] **Step 1 (failing tests):** builder with `root = "free"` emits `root_free`, `rider_root_free`, `steer_joint` (range ±60°, damping 0.5, no stiffness), actuator `steer_torque` with `ctrlrange = ±cap`; `nq == nv + 2` for bike + rider (two free joints); native `Spatial` reports `free6`, `expected_nq()` correct; `samples.cpp` emits `root.quat_wxyz.*`, right-handed ZYX Euler angles in the world frame (identity → all zero; a pure 10° **nose-up** pitch → `pitch_rad = −0.1745` and `pitch_up_rad = +0.1745`; a 10° lean to the right → `roll_rad = +0.1745`), `steer.angle_rad/rate_rad_s/torque_nm`.
- [ ] **Step 2:** Implement; writers that still reject `free6` get their 3D branch now (grep the P1 audit table for sites marked "P4").
- [ ] **Step 3:** `native` green; P1 bridge and lateral-zero green. Commit `feat: free-root chassis and steering joint for spatial 3d`.

### Task 2: Balance controller `steer_balance`

**Files:** `native/src/rider/steer_balance.{hpp,cpp}`, `runtime/step.cpp` (control tick), config; tests

**Interfaces:** `class SteerBalance { explicit SteerBalance(const nativecfg::SteerConfig&, double control_period_s); double tick(double roll, double roll_rate, double yaw, double y, double steer_rate, double speed) noexcept; /* returns torque after schedule, delay, cap */ void reset(); bool enabled() const; }` with the law of spec §8.3; `delay_ticks = round(reaction_delay_s / control_period_s)` (FIFO of that length, zero-filled on reset); cap from the config number or from `joint_strength_path` key `"steer"` (add the key to `examples/research/rider_strength_reference.json` with provenance "engineering estimate 40 N·m, unverified" — label it so).

- [ ] **Step 1 (failing tests, unit):** zero state → zero torque; positive roll (lean right, right-handed about +x) → **negative** steer torque (steer right, into the fall; the steer axis points up the head tube so positive steer turns the bar left) — document the sign convention in the test; delay of 0.1 s at 200 Hz shifts the response by exactly 20 ticks; cap clamps; schedule interpolates (`s(1) = 2.5`, `s(10) = 0.8`, `s(0.5) = 2.5` clamped).
- [ ] **Step 2 (failing test, closed loop):** flat `Grid3D`, free root, bike + rider, v = 3 m/s, initial roll 3°: roll returns under 0.5° within 3 s and stays; `|y| < 0.3 m` throughout; steer torque never saturates at the default cap.
- [ ] **Step 3:** Implement; controller runs inside the existing control tick after the rider program; `rider_enabled = false` disables it (then the bike falls — covered by the capsize test).
- [ ] **Step 4:** Commit `feat(native): speed-scheduled steer balance controller with delay and torque cap`.

### Task 3: Episode outcomes and truth extension

**Files:** `native/src/runtime/{status,research_metrics}.cpp`, `src/bike_sim/sim/research/metrics.py`, `sim/ride/wheelie.py`; tests

- [ ] **Step 1 (failing tests):** `rider_enabled = false` at v = 2 m/s → episode ends `crash:capsize` within 3 s with `|roll| ≥ capsize_roll_rad`; a lateral initial velocity of 1 m/s with controller gains zeroed → `crash:off_corridor` when `|y| > W − 0.3`; `WheelieTruth` gains `roll_rad, yaw_rad, heading_rad, lateral_offset_m, front/rear_lateral_slip_mps` (planar truth unchanged — fields default `0.0`, and the planar research tests stay byte-identical).
- [ ] **Step 2:** Implement in native status and the Python mirror.
- [ ] **Step 3:** Commit `feat(research): capsize and off-corridor outcomes; 3D truth fields`.

### Task 4: C++ equilibrium (settle + refine) and bootstrap hold

**Files:** `native/src/runtime/equilibrium.{hpp,cpp}`, `bootstrap_hold.{hpp,cpp}`, `binding.cpp`, `src/bike_sim/native/{runtime,setup}.py`, `sim/ride/equilibrium_cache.py`; `tests/reference/test_spatial3d_equilibrium.py`

**Interfaces:** `Equilibrium::run(const nativecfg::BootstrapConfig&, Stepper&) -> std::expected<EquilibriumResult, Diag>` (named to stay clear of the existing `runtime::Bootstrap*` state-inventory wire in `runtime/bootstrap.hpp`) — phases: (1) initial pose: root on the terrain at `x0` with pitch = −atan(longitudinal slope) and roll = −atan(lateral slope) at the contact line; brakes on; hold active; (2) damped relaxation (qvel zeroed every 20 ms) until `max|qacc| ≤ 0.05` or `settle_time_s` budget → failure diagnostic listing the worst five coordinates (as the Python message does); (3) refine: least squares over scalar `qpos` (free-root quaternion excluded; the hold keeps roll/yaw/y) with residual `M·qacc/√diag M` via `DGELSD`; (4) `t = 0⁻`: the hold wrench must be below 1 N / 1 N·m, otherwise `Diag("bootstrap hold still loaded at release: start zone is not laterally flat")`; `t = 0`: hold force forced to zero, controller armed. `BootstrapHoldWriter::try_compute_into` adds `−k·e − c·ė` on roll, yaw, y of the frame when `bootstrap_active`, else writes nothing and sets `hold_active = 0`. Energy ledger: hold work booked under `energy.bootstrap_hold_j`, must be constant after `t = 0`.

- [ ] **Step 1 (failing tests):** planar config still takes the Python equilibrium path and stays byte-identical (assert the native bootstrap is not invoked for `spatial = "planar"`); constrained-root 3D: native settle converges on `rough_uphill_savage_3d` fast tier within the budget and the resulting `qpos` matches the Python planar equilibrium's shared coordinates within 1e-6 (same physics, different solver — tolerance, not bytes); free-root 3D: converges with the hold; hold wrench at `t = 0⁻` below threshold (and a banked synthetic start fails with the diagnostic); `bootstrap.hold_active == 0` for all `t ≥ 0`; `energy.bootstrap_hold_j` constant after `t = 0`; detailed tier with `settle_time_s = 80` converges (mark `slow`); budget exhaustion gives the diagnostic.
- [ ] **Step 2:** Equilibrium cache: fingerprint includes `spatial`, `.terrain3d` sha256, tier, lateral coefficient set, imu mount; stores the 3D tyre state wire; cache hit skips settle (test: second bootstrap < 1 s).
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Commit `feat(native): C++ equilibrium bootstrap with settle budget and bootstrap-only hold`.

### Task 5: Meijaard 2007 benchmark fixture and gate

**Files:** `src/bike_sim/mujoco/benchmark_whipple.py`, `builder.py`, `tools/modal_fit.py`, `tests/reference/test_spatial3d_meijaard.py`, `golden_manifests/spatial3d/meijaard_v6_3s.json`

**Interfaces:** `benchmark_whipple.py` builds: rear frame + rider as one body (mass, CoM, inertia tensor from Table 1), front assembly body (handlebar/fork), rear and front wheels (mass, radius, I_xx/I_yy) on spin hinges, steer hinge along the steering axis defined by head angle λ and trail c; no suspension, no drivetrain, no rider tree; `pneumatic_3d` with `knife_edge = true` and `rim_radius` set so the torus is a thin disc approximation (section radius 10 mm); `controller = "off"`; dt 0.25 ms. `tools/modal_fit.py`: `fit_modes(t, roll, steer, n_modes=4) -> [(sigma, omega)]` by matrix pencil on the two signals; `stability_boundary(f, v_lo, v_hi, step=0.05)` scanning for the sign change of the weave real part / capsize real part.

- [ ] **Step 1 (failing tests):** parameters round-trip from the TOML into the MJCF (masses, inertias as `fullinertia`); the compiled model's total mass equals the paper's sum; for v ∈ {2, 4, 6, 8, 10} m/s: launch upright at speed, roll perturbation `δ₀ = 0.5°` at stable speeds and `0.05°` where Table 2 says the weave is unstable, record until `|roll|` reaches 2° or 3 s elapse (the linear regime; at v = 2 m/s the published σ ≈ +1.7 s⁻¹ leaves ≈ 2 s of usable window from 0.05°), identified weave `(σ, ω)` and capsize `σ` by matrix pencil on roll and steer within **3 %** (floor 0.05 s⁻¹) of the TOML Table 2 values; boundaries within **±0.2 m/s** of the TOML speeds.
- [ ] **Step 2:** Implement builder mode; `validate_supported` admits the benchmark mode (no rider, no drive) as a distinct whitelist row.
- [ ] **Step 3:** If the gate fails, investigate in this order: inertia tensor frame (MuJoCo `fullinertia` is about the body frame at the CoM), steer axis direction and trail construction, wheel gyroscopic terms (wheel spin inertia), tyre knife-edge stiffness too low (check lateral slip < 0.01 m/s), integrator (`implicitfast` vs RK4 at dt 0.25 ms). Never adjust tolerances.
- [ ] **Step 4:** Write the manifest `meijaard_v6_3s.json` (the v = 6 run). Commit `test: Meijaard 2007 benchmark fixture and eigenvalue gate`.

### Task 6: Steady cornering gate

**Files:** `tests/reference/test_spatial3d_cornering.py`; `steer_balance` gains a test-only `yaw_rate_ref` input (exposed through the binding, rejected in production configs)

- [ ] **Step 1 (failing test):** flat `Grid3D`, free root, bike + rider, for ρ ∈ {10, 20, 40} m, v ∈ {3, 5, 8} m/s (skip combinations with `tan φ > 0.7`): controller with `yaw_rate_ref = v/ρ`; after 5 s, mean lean angle of the combined CoM within **1°** of `atan(v²/(gρ))` and `Σ lateral tyre force` within **2 %** of `m v²/ρ`; path radius from `(x, y)` within 5 %.
- [ ] **Step 2:** Commit `test: steady-cornering gate for spatial 3d`.

### Task 7: dt study and mirror-symmetry invariant

**Files:** `tests/reference/test_spatial3d_invariants.py`; audit

- [ ] **Step 1:** Mirror test: `alpine_rocky_3d` mirrored in y (`z(x, −y)`, features mirrored), initial state mirrored (y, roll, yaw, steer signs), control identical; run 5 s; every lateral channel is the exact negation (bitwise) and every longitudinal channel identical. A mismatch is a sign or ordering bug — fix it under the spec §10.5 rules; only if a patch-level reduction provably cannot be made mirror-invariant, apply the pre-registered fallback (≤ 1e-12 relative on patch sums/qfrc) and name the reduction in the audit.
- [ ] **Step 2:** dt study: free root + rider + fast tier on `alpine_rocky_3d` at dt ∈ {1.25, 0.625, 0.5} ms for 10 s: compare outcome, contact timeline, max roll, energy residual; choose the largest dt whose results are within the Layer C tolerances of the 0.5 ms run; record in the audit and in `examples/research/viewer_physics_pneumatic_3d.toml` (new example config).
- [ ] **Step 3:** Write `free_rocky_10s.json` manifest at the chosen dt. Commit `test: mirror-symmetry invariant; dt study; pneumatic_3d example config`.

### Task 8: Phase audit and gate

- [ ] **Step 1:** `full` green; planar/constrained bytes unchanged; both benchmarks pass; `hold_active` zero everywhere.
- [ ] **Step 2:** `docs/superpowers/audits/2026-10-14-spatial-3d-p4.md`: eigenvalue table (published vs identified), boundary speeds, cornering table, settle times per tier, dt study table, mirror evidence.
- [ ] **Step 3:** `docs/RIDE.md` capability sentence (free root, balance, benchmarks); `CONTEXT.md` terms. Merge; tag `spatial-3d-p4`.
