# Spatial 3D — Phase 3 (Track C): `pneumatic_3d` Tyre Implementation Plan

> **For agentic workers:** Execute task-by-task with your runtime's plan-execution skill; steps use checkbox (`- [ ]`) syntax. **Prerequisites:** Phase 0 merged (contracts 5.1, 5.2; `tyre_lateral_reference.json` pinned — if its status is `unpinned`, Tasks 6–7 run with `xfail` and the phase gate records it); Phases 1 and 2 merged for integration tests (Tasks 5–8: constrained root, research runtime, regression bridge, 3D tracks). Read `native/src/pneumatic/*` and `native/src/writers/pneumatic_tire.cpp` first — the 3D tyre **reuses** those kernels through an element adapter and must not copy them. Read `docs/agents/native-engineering.md`.

**Goal:** A native `Pneumatic3dTireWriter` (backend `pneumatic_3d`): ray grid over a circular-section torus against `Grid3D`, per-element carcass law reusing the pneumatic kernels, 2D brush (fast: lumped combined-slip + camber thrust; detailed: discretised 2D bristles), `PatchWrench` output injected via `mj_jac`, canonical state wire, zero-allocation warm core, element partition hooks for the P7 worker pool (single-threaded here), validated by flat-road consistency (2 %) with the planar backend, T1–T4, lateral/camber rigs inside the pinned literature bands, and the physical camber ceiling.

**Architecture:** `pneumatic/element_adapter.{hpp,cpp}` exposes the planar carcass law per element (`element_response(delta, delta_prev, …, dA)`), Maxwell/hysteresis/advection along θ, and rim strike — thin wrappers over the existing kernels with areal weights. `writers/pneumatic_3d_tire.{hpp,cpp}` owns geometry (torus rays in a world-fixed wheel-plane fan), the `Grid3D` batch ray cast, element arrays, patch labelling (8-connectivity), per-patch aggregation, the brush (`pneumatic3d/brush2d.{hpp,cpp}`), and the qfrc injection. Partitioning: elements are split into `n_parts` contiguous ranges; `compute_elements(range)` is pure per range; the aggregation over ranges is fixed-order. This phase runs `n_parts = 1`; P7 calls the same functions from the pool.

**Tech Stack:** C++23, nanobind, `native` profile; Python rigs under `validation/` (`quick`).

**Spec:** `docs/superpowers/specs/2026-10-14-spatial-3d-design.md` §10, §5.2, §9.5. **ADR:** 0002 Layer A (T1–T4, lateral targets), B (mirror, determinism), D.

## Global Constraints

- All master-plan constraints. Branch `feat/spatial-3d-p3-pneumatic-3d`.
- `PneumaticTireWriter` (planar) bytes, state schema and tests unchanged; `test_native_pneumatic_tire.py` green at every task. Any change to `native/src/pneumatic/*.cpp` must be additive (new entry points) — existing functions keep their bodies.
- Warm core `try_compute_into` is `noexcept BIKE_NONBLOCKING`, allocation-free; scratch sized from `(n_circ, n_lat, kMaxPatches3d)` at construction; RTSan passes.
- Determinism rule §10.5: per-element outputs to arrays; reductions on the calling thread in fixed element order; **lateral reductions use the mirrored-pair tree** (pairs `j, n_lat−1−j` first, then outside-in, centre last) so the mirror test can be bitwise; no accumulation order depends on `n_parts`. If a reduction cannot be made exactly mirror-invariant, use the pre-registered fallback (≤ 1e-12 relative on patch sums/qfrc) and name it in the audit.
- The P1 `Pneumatic3dFlatStub` is deleted in Task 8 of this phase; until then both coexist in the writer variant.

---

## File Structure

| File | Responsibility |
|---|---|
| Create `native/src/pneumatic/element_adapter.{hpp,cpp}` | Areal per-element carcass response reusing `carcass.cpp` kernels; advection along θ; rim strike |
| Create `native/src/pneumatic3d/geometry.{hpp,cpp}` | Torus ray grid: origins/directions per (θ_i, φ_j), element areas, wheel-plane fan orientation |
| Create `native/src/pneumatic3d/patches.{hpp,cpp}` | 8-connected labelling on the (i, j) grid; per-patch aggregates (F_z, centroid, normal, half-length, camber) |
| Create `native/src/pneumatic3d/brush2d.{hpp,cpp}` | Lumped combined-slip brush with relaxation (κ′, α′), camber thrust, aligning couple; discretised 2D bristles |
| Modify `native/src/writers/pneumatic_3d_tire.hpp`; create `pneumatic_3d_tire.cpp` | The writer: compute, inject, state wire, snapshot/diagnostics, partition hooks |
| Modify `native/src/stepper.{hpp,cpp}`, `runtime/step.cpp` | Variant dispatch; `supports_multiple_contacts = true`; diagnostics on the wire |
| Modify `native/src/config_types.hpp`, `config_validation.hpp` | `Pneumatic3dConfig` fields incl. lateral coefficients and `knife_edge` |
| Modify `src/bike_sim/native/config.py`, `schema.py` | Project `tire.pneumatic_3d` with the pinned lateral set resolved from `tyre_lateral_reference.json` |
| Modify `src/bike_sim/physics/tyre.py` | `Pneumatic3dSpecs` (tier table 64×7 / 256×15, lat_half_angle 70°) and `LateralCoefficients` loader |
| Create `src/bike_sim/validation/contact_manifold_rigs_3d.py` | Flat/camber/slip-angle/combined/shoulder-rock/rut/bank rigs driving the native writer directly through the binding |
| Modify `src/bike_sim/validation/contact_acceptance.py` | `contact_matrix_3d` |
| Create `tests/reference/test_native_pneumatic_3d.py` | Unit, state wire, flat consistency, T1–T4, lateral bands, ceiling, determinism, RTSan |
| Modify `native/tests/test_contracts.cpp` | Geometry/patch/brush unit cases |
| Delete `native/src/writers/pneumatic_3d_flat_stub.{hpp,cpp}` | Task 8 |
| Modify `docs/RIDE.md` | Capability sentence; §3.1 gains the 3D subsection |
| Create `docs/superpowers/audits/2026-10-14-spatial-3d-p3.md` | Evidence |

---

### Task 1: Torus ray grid geometry

**Files:** `native/src/pneumatic3d/geometry.{hpp,cpp}`; `native/tests/test_contracts.cpp`

**Interfaces:** `struct RayGrid { int n_circ, n_lat; double half_angle, lat_half_angle, ring_radius, section_radius; }`; `void rays(const RayGrid&, const WheelPose& /*hub pos, axle axis world, wheel-plane "down" = projection of world −Z*/, std::span<double> origins /*3·N*/, std::span<double> dirs /*3·N*/, std::span<double> areas /*N*/) noexcept;` Element order: `k = i·n_lat + j` (θ outer, φ inner) — this order is the contract for the state wire and reductions. `dA_ij = R_ring·dθ · r_s·dφ · cos φ_j`.

- [ ] **Step 1 (failing tests):** for an upright wheel on a flat plane, rays with `j = n_lat/2` (φ = 0) lie in the wheel plane and reproduce the planar fan directions bitwise (`sin θ, 0, −cos θ` after rotation into world); `Σ dA` over the fan equals the analytic torus patch area to 1e-9; a 10° cambered wheel's rays are the upright rays rotated about the forward axis (orthogonality check).
- [ ] **Step 2:** Implement with `numeric_sincos.hpp` conventions.
- [ ] **Step 3:** Commit `feat(native): torus ray grid for pneumatic_3d`.

### Task 2: Element adapter over the pneumatic carcass kernels

**Files:** `native/src/pneumatic/element_adapter.{hpp,cpp}`; tests in `test_contracts.cpp` and `tests/reference/test_native_pneumatic_3d.py`

**Interfaces:** `struct ElementState { double delta_prev_m, elastic_prev_n, maxwell_n; }`; `ElementResult element_response(const CarcassParams&, double delta, double omega, double dt, double dA, double section_radius, ElementState& st) noexcept` returning `{force_n, energy_j, loss_j, rim_strike}`; `void advect_theta(std::span<ElementState>, int n_circ, int n_lat, double shift_elements) noexcept` (semi-Lagrangian along θ per φ-row, reusing the interpolation kernel of `carcass.cpp`). The areal pressure term uses the normalised chord function `g(δ)` such that `∫ g dφ` over the loaded φ-range equals the planar chord law — derive `g` in the header comment with the integral worked out.

- [ ] **Step 1 (failing tests):** with `n_lat = 1` and `dA = R·dθ` the element response equals the planar ray response **bitwise** for the same inputs (this pins the adapter to the existing kernel); energy bookkeeping sums match; `advect_theta` with `n_lat = 1` equals the planar advection bitwise.
- [ ] **Step 2:** Implement; add any new entry points to `carcass.cpp` additively.
- [ ] **Step 3:** `test_native_pneumatic_tire.py` still bitwise. Commit `feat(native): areal element adapter over the pneumatic carcass kernels`.

### Task 3: Patch labelling and aggregation

**Files:** `native/src/pneumatic3d/patches.{hpp,cpp}`

**Interfaces:** `int label_patches(std::span<const double> deflection, int n_circ, int n_lat, std::span<std::int16_t> label_out, int max_patches) noexcept` (8-connectivity, two-pass union-find on a fixed-size table, deterministic label order = first element index); `void aggregate(...)` → per patch `F_z` (sum of element forces along element normals, fixed order), load-weighted centroid and normal, longitudinal half-length `a` (θ extent × R_ring / 2), camber `γ = angle(wheel plane, patch normal)`, contact velocity at centroid.

- [ ] **Step 1 (failing tests):** synthetic deflection maps: one blob → 1 patch; two blobs separated by an unloaded column → 2; diagonal touch → 1 (8-connectivity); more than `kMaxPatches3d` → the writer reports `patch_overflow` (diagnostic, no abort) and keeps the `kMaxPatches3d` largest by element count, deterministically; **mirror rule (spec §10.5):** patches are emitted in the canonical order `(θ_min, |y_centroid|, sign(y_centroid))`, and a j-mirrored deflection map yields the mirrored patch list in the same canonical positions (bitwise aggregates with y-signs flipped).
- [ ] **Step 2:** Implement allocation-free.
- [ ] **Step 3:** Commit `feat(native): deterministic patch labelling and aggregation for pneumatic_3d`.

### Task 4: 2D brush (lumped and discretised)

**Files:** `native/src/pneumatic3d/brush2d.{hpp,cpp}`; `src/bike_sim/physics/tyre.py` (`LateralCoefficients`); tests

**Interfaces:**
- `struct LateralCoefficients { double c_alpha_per_load, c_gamma_per_load, sigma_y_over_sigma_x, load_exponent, pressure_exponent, f_ref_n, p_ref_bar; };`
- Lumped: `LumpedState { double kappa_prime, alpha_prime; }`; `BrushResult lumped_step(const BrushParams&, const PatchAggregate&, LumpedState&, double dt) noexcept` → `{fx, fy, mz, loss_j}`: relaxation `σ·dκ'/dt + |Vx|κ' = −Vsx`, `σ_y·dα'/dt + |Vx|α' = −Vsy` (reusing the planar relaxation integrator for κ′), combined slip `σ_eq = √(κ'² + (C_α/C_κ · tan α')²)`, parabolic brush magnitude `F = μF_z(1 − (1 − θ σ_eq)³)` for `θσ_eq < 1` else `μF_z`, direction along the combined slip; camber thrust `F_γ = min(C_γ(F_z, p)·γ, F_z·tan γ)` added to `fy` and included in the friction ellipse clipping; aligning couple from the parabolic brush `m_z = −F_y·a/3·(1 − θσ_eq)…` (write the full expression in the header). Friction from `SurfaceSpec` as today (Stribeck on sliding speed).
- Discretised: `Bristle { double x_m, y_m; }` per element; `discretised_step(...)`: bristles advect along θ at tread speed, 2D deflection from relative velocity, clipped to `μ·f_elem` in the combined direction, dashpot `BRISTLE_DAMPING_S`; camber thrust emerges (no explicit term).
- `knife_edge` scaling applied at config resolution, not in the kernel.

- [ ] **Step 1 (failing tests):** pure longitudinal slip with `n_lat = 1` → lumped `fx` equals the planar lumped brush bitwise and `fy = mz = 0`; pure slip angle at small α → `fy ≈ C_α·F_z·α` within 1 % (linear regime); large α → `|F| = μF_z` sliding; combined: `√(fx² + fy²) ≤ μF_z` always; camber: `fy ≤ F_z·tan γ` always; `α = 0, γ = 0` → `fy = 0.0` **exactly** (bitwise, for the lateral-zero invariant); discretised at α = 10° cambered wheel produces `fy` with the correct sign.
- [ ] **Step 2:** Implement; property tests (Hypothesis) over `(α, κ, γ, F_z, μ)` for the inequalities.
- [ ] **Step 3:** Commit `feat(native): 2D brush with combined slip and camber thrust for pneumatic_3d`.

### Task 5: The writer — compute, inject, state wire, diagnostics, partition hooks

**Files:** `native/src/writers/pneumatic_3d_tire.{hpp,cpp}`, `stepper.*`, `runtime/step.cpp`, config files, `src/bike_sim/native/{config,schema}.py`

**Interfaces:** spec §5.2. `try_compute_into(qfrc, const mjModel*, const mjData*, const Grid3D&, dt)`: (1) poses → rays for both wheels; (2) `compute_elements(range)` for each of `n_parts` ranges: `Grid3D::ray_cast_many`, deflections, element responses, element normals; (3) label/aggregate per wheel (calling thread); (4) brush per patch; (5) `PatchWrench` → `qfrc` via `mj_jac` at `point_m` on the wheel body plus `mj_jac`'s rotational block for `couple_nm`; (6) energy/loss into the accumulator. `ω_spin = (ω_wheel − ω_hub)·axle_world` from `mj_objectVelocity` on wheel and fork/chainstay bodies. State wire names from P0 generator; `state_row`/`set_state_row`; snapshot/diagnostics: `patch_count`, `airborne`, `rim_strike`, `max_camber_rad`, `max_slip_angle_rad`, `lateral_force_n`, `patch_overflow`, `supports_multiple_contacts = true`.

- [ ] **Step 1 (failing tests):** on the planar `alpine_rocky.toml` extruded laterally flat, with the constrained root: `tires.*.world_force_n.1 == 0.0` bitwise on every step (the lateral-zero invariant now holds with the real writer); state wire round-trip; snapshot diagnostics present; `n_parts ∈ {1, 2, 3, 7}` produce identical bytes **single-threaded** (this proves the reduction-order rule before P7 adds threads); RTSan build passes the tick; no allocation after construction (ASan + malloc-count hook as in existing tests).
- [ ] **Step 2:** Implement; config projection resolves `lateral_coefficients` by name from `tyre_lateral_reference.json` into numbers on the wire (native never reads JSON).
- [ ] **Step 3:** Commit `feat(native): Pneumatic3dTireWriter with deterministic partitioned element pipeline`.

### Task 6: Flat-road consistency and T1–T4 transfer

**Files:** `src/bike_sim/validation/contact_manifold_rigs_3d.py`, `src/bike_sim/validation/contact_acceptance.py`, `tests/reference/test_native_pneumatic_3d.py`

- [ ] **Step 1:** Rig `flat_load_sweep`: upright wheel, flat `Grid3D`, loads 200–900 N, 1.38 and 1.72 bar, both tyres: deflection and contact length vs the planar `pneumatic` backend on the same `RoadProfile` — **within 2 %**. If the areal pressure term misses, adjust `g(δ)` (Task 2) — never a fudge factor on the planar side.
- [ ] **Step 2:** T1–T4 checks re-run on the 3D writer at γ = 0 with the planar tolerances (`±15 %`); drop-sled dynamic/static ratio and damping ratio within the T2 bands.
- [ ] **Step 3:** Commit `test: pneumatic_3d flat-road consistency and T1–T4 transfer`.

### Task 7: Lateral and camber rigs, literature bands, physical ceiling

**Files:** as Task 6

- [ ] **Step 1:** Rig `slip_angle_sweep` (α ∈ {0, 1, 2, 4, 8}°, v = 5 m/s, loads 350/450/600 N, 1.5 bar): identified `C_α/F_z` from the linear fit over α ≤ 2° lies inside `tyre_lateral_reference.json["default"].C_alpha_per_load_per_rad`; saturation at large α at `μF_z` ± 3 %.
- [ ] **Step 2:** Rig `camber_sweep` (γ ∈ {0, 5, 10, 20}°, α = 0): `C_γ/F_z` inside the band; `F_y ≤ F_z·tan γ` for every sample (hard assert).
- [ ] **Step 3:** Rig `combined`: friction ellipse never exceeded; `shoulder_rock`, `rut`, `bank_15deg`: finite forces, no NaN, energy residual within the planar threshold, patch count ≤ `kMaxPatches3d` (overflow diagnostic counted and reported in the audit).
- [ ] **Step 4:** Detailed tier at dt 0.25 ms on the slip-angle sweep: `C_α` within 15 % of the fast tier (consistency between tiers, both inside the band).
- [ ] **Step 5:** Commit `test: pneumatic_3d lateral and camber acceptance against pinned literature bands`.

### Task 8: Remove the P1 stub; `validate_supported`; docs

- [ ] **Step 1:** Delete `pneumatic_3d_flat_stub.*`; variant dispatch routes `pneumatic_3d` to the real writer; P1 regression bridge re-run (`full`) — must still pass with identical numbers reported in the audit (the real writer on a flat extrusion must behave like the planar backend within the Layer C tolerances; the 2 % consistency gate makes this expected).
- [ ] **Step 2:** `docs/RIDE.md` §3.1 3D subsection + capability sentence; `CONTEXT.md` term *element grid*.
- [ ] **Step 3:** Commit `feat(native): pneumatic_3d replaces the flat stub; docs`.

### Task 9: Phase audit and gate

- [ ] **Step 1:** `bash tools/run_tests.sh native` and `full` green; planar pneumatic bitwise; RTSan/ASan builds green.
- [ ] **Step 2:** `docs/superpowers/audits/2026-10-14-spatial-3d-p3.md`: consistency table, T1–T4 table, lateral/camber identified values vs bands, tier consistency, `n_parts` byte-equality evidence, single-thread µs/step for fast and detailed tiers (expected fast ≈ 2 × 448 rays ≈ 100 µs/step before threading).
- [ ] **Step 3:** Merge; tag `spatial-3d-p3`.
