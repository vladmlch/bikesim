# Spatial 3D — Phase 0: Foundation (ADR, contracts, reference pins, doc repair) Implementation Plan

> **For agentic workers:** Execute task-by-task with your runtime's plan-execution skill; steps use checkbox (`- [ ]`) syntax. **Prerequisites:** `feat/native-pneumatic-tire` merged into `main`; user has accepted `docs/adr/0002-spatial-3d-correctness-method.md` (flip its status to "accepted" in Task 1). Read `docs/agents/native-engineering.md` before Task 3.

**Goal:** Everything the parallel tracks need before they can start without talking to each other: the five frozen interface contracts as compilable headers / importable dataclasses with failing stub tests, the external reference values pinned from their sources with provenance, the self-golden manifest tool, and the documentation repairs that the 3D work will otherwise trip over.

**Architecture:** Nothing in this phase changes physics or bytes. Headers declare types and function signatures; `.cpp` files contain bodies that `throw std::logic_error("spatial-3d P<N>: <symbol> not implemented")` (never silently return; throwing — not aborting — keeps the nanobind extension and pytest alive); Python stubs raise `NotImplementedError`. Stub tests are marked `pytest.mark.xfail(strict=True, reason="spatial-3d P<N>")` so a phase that implements the contract must flip them to passing in the same commit.

**Tech Stack:** as master plan. Python-only tasks run `quick`; header tasks run `native` (they must compile under both compilers and pass all sweeps).

**Spec:** `docs/superpowers/specs/2026-10-14-spatial-3d-design.md` §4, §5, §9, §13. **Master:** `docs/superpowers/plans/2026-10-14-spatial-3d-master.md`.

## Global Constraints

- All master-plan constraints.
- Branch: `feat/spatial-3d-p0-foundation`.
- No behavioural change anywhere; `bash tools/run_tests.sh native` green after Tasks 3–5; `full` at the end must be byte-identical to `main` for every existing golden test (the stubs must not be reachable from planar configs).
- Reference numbers are copied from the primary sources with page/table citations into the pin files; **never typed from memory**. If a source cannot be obtained, the pin file records `status: "unpinned"` and the dependent gate test is `xfail` until it is — do not invent values.

---

## File Structure

| File | Responsibility |
|---|---|
| Modify `docs/adr/0002-spatial-3d-correctness-method.md` | Status → accepted; date |
| Modify `CONTEXT.md` | C++23; channel count; new terms |
| Modify `docs/RIDE.md` | Add pneumatic section (§3.1) and literature list (§11.2) cited by code; capability sentence placeholder for 3D |
| Modify `docs/agents/native-engineering.md` | Fix dangling links to non-existent spec/plan files |
| Create `native/src/terrain/grid3d.hpp`, `grid3d.cpp` | Contract 5.1 types and signatures; unimplemented bodies |
| Create `native/src/writers/pneumatic_3d_tire.hpp` | Contract 5.2 `PatchWrench`, `TyreOutput`, state-wire name generator |
| Create `native/src/spatial_topology.hpp` | `topology::Spatial` descriptor declaration (filled in P1) |
| Modify `native/src/config_types.hpp`, `config_validation.hpp`, `config.hpp` | `nativecfg::SpatialConfig`, `Pneumatic3dConfig` parse + validate; rejected unless `spatial.mode == "3d"` |
| Modify `src/bike_sim/physics/model_config.py` | `spatial: str = "planar"`, `spatial_3d: Spatial3dConfig` dataclass; validation rules of spec §4.1 |
| Modify `src/bike_sim/physics/physical_config.py` | `tires.backend` accepts `"pneumatic_3d"`; `Pneumatic3dTierConfig` |
| Modify `src/bike_sim/native/config.py`, `schema.py` | Project and validate `config['spatial']`, `config['tire']['pneumatic_3d']` |
| Modify `src/bike_sim/native/setup.py:48-62` | `validate_supported`: `spatial` pairing rules |
| Modify `src/bike_sim/sim/research/sensors.py` | `SensorObservationV2` dataclass (side by side with schema 1 `SensorObservation`); `SensorConfig` 6-axis fields |
| Create `tests/test_spatial_config.py`, `tests/test_reference_pins.py` | Config rules; pin files |
| Modify `src/bike_sim/recording/catalog.py` | Declarations for spec §5.5 fields (type, unit, frame, source) — declared, not yet produced |
| Create `examples/benchmarks/meijaard2007.toml` | 27 benchmark parameters + Table 2 eigenvalues + weave/capsize speeds, with citations |
| Create `examples/research/tyre_lateral_reference.json` | Lateral/camber stiffness bands with provenance |
| Create `tools/golden_manifest.py` | `write`, `check`, CLI |
| Create `tests/reference/golden_manifests/spatial3d/README.md` | Update policy |
| Create `tests/reference/test_spatial3d_contracts.py` | Stub tests (xfail strict) for the five contracts |
| Create `tests/test_golden_manifest.py` | Manifest tool tests (real, passing) |
| Modify `native/CMakeLists.txt` | New sources in library and sweeps |
| Create `docs/superpowers/audits/2026-10-14-spatial-3d-p0.md` | Evidence |

---

### Task 1: Accept ADR 0002, repair documentation drift

**Files:**
- Modify: `docs/adr/0002-spatial-3d-correctness-method.md`, `CONTEXT.md`, `docs/RIDE.md`, `docs/agents/native-engineering.md`
- Test: `quick` (doc-only), plus `grep -n "RIDE.md §" src/bike_sim/physics/tyre.py src/bike_sim/sim/ride/tyre/*.py` to list every cited section

- [ ] **Step 1:** Flip ADR 0002 status to `accepted`, add the acceptance date.
- [ ] **Step 2:** `CONTEXT.md`: replace "C++17" with "C++23" (fact: `native/CMakeLists.txt:124`), replace "~100-channel contract" with "schema-2 channel tree (924 flattened columns on the pneumatic welded run; backend-dependent; see `recording/catalog.py`)", add terms: *spatial mode*, *constrained root*, *free root*, *knife-edge setting*, *bootstrap hold*, *self-golden manifest*, *mirror test*, *observation schema 2*.
- [ ] **Step 3:** `docs/RIDE.md`: write the pneumatic section the code cites as §3.1 (model summary from `sim/ride/tyre/{geometry,carcass,brush,model}.py` docstrings: ray fan, carcass law, Maxwell branch, hysteresis, rim strike, lumped vs discretised brush, tiers) and §11.2 literature list ([T1] Dressel & Sadauckas 2020 and the T2–T4 sources as named in `physics/tyre.py:245-273` and the tyre docstrings; if a citation is incomplete in code, write "incomplete in source, see `physics/tyre.py:<line>`"). Renumber nothing else; add anchors so the `§3.1`/`§11.2` references resolve.
- [ ] **Step 4:** `docs/agents/native-engineering.md`: the links to `docs/superpowers/specs/2026-10-06-native-safety.md` and `plans/2026-10-06-native-safety-master.md` point to files that do not exist; replace with the nearest existing documents (`docs/TESTING.md` sanitizer section) and add ADR 0002 to the "read before native changes" list.
- [ ] **Step 5:** Commit `docs: accept ADR 0002 and repair CONTEXT/RIDE/native-engineering drift`.

### Task 2: Pin external references with provenance

**Files:**
- Create: `examples/benchmarks/meijaard2007.toml`, `examples/research/tyre_lateral_reference.json`
- Create: `tests/test_reference_pins.py`
- Test: `quick`

**Interfaces:**
- `meijaard2007.toml`: `[source] doi = "10.1098/rspa.2007.1857"`, `[parameters]` (27 named values with units, Table 1), `[eigenvalues]` rows `v_mps = …, weave_re, weave_im, capsize, caster` for v = 0…10 (Table 2), `[speeds] weave_mps, capsize_mps` (as printed in the paper), `[checks] cross_check = "bicycleparameters"` with the version used.
- `tyre_lateral_reference.json`: `{"default": {"C_alpha_per_load_per_rad": [lo, hi], "C_gamma_per_load_per_rad": [lo, hi], "sigma_y_over_sigma_x": value, "load_exponent": value, "pressure_exponent": value, "valid_load_n": [lo, hi], "valid_pressure_bar": [lo, hi], "provenance": [{"source": "...", "where": "...", "note": "..."}], "status": "pinned"|"unpinned"}}`.

- [ ] **Step 1:** Obtain Meijaard et al. 2007 (Proc. R. Soc. A 463, 1955–1982). Transcribe Table 1 (parameters) and Table 2 (eigenvalues v = 0…10 m/s) and the printed weave/capsize speeds into the TOML with table/page citations. Independently compute the eigenvalues with `uv run --with bicycleparameters python -c …` using the same parameters and record the max relative difference (must be < 1e-6; if `bicycleparameters` cannot be installed, record the attempt and skip the cross-check — the paper remains the pin).
- [ ] **Step 2:** Obtain Dressel (2013, UWM thesis "Measuring and Modeling the Mechanical Properties of Bicycle Tires"), Doria et al. (2013, Vehicle System Dynamics 51(3), "Identification of the mechanical properties of bicycle tyres for modelling of bicycle dynamics"), Cole & Khoo (2001, Int. J. Vehicle Design). Extract normalized cornering stiffness and camber stiffness bands for 50–65 mm tyres at 1.2–2.0 bar, 350–600 N; convert everything to per-rad; record the ceiling check `C_gamma ≤ 1/rad` (= F_z·tan γ/γ small-angle). Write the JSON with per-value provenance. If a source is unavailable, set `"status": "unpinned"` for the affected band and note it.
- [ ] **Step 3:** `tests/test_reference_pins.py`: files parse; every value has provenance; eigenvalue rows are ordered and the weave/capsize speeds lie where the sign changes in Table 2 (consistency only); lateral bands are ordered `lo < hi` and the camber ceiling holds.
- [ ] **Step 4:** Commit `data: pin Meijaard 2007 benchmark and bicycle tyre lateral stiffness references`.

### Task 3: Native contract headers (5.1, 5.2) and `topology::Spatial` declaration

**Files:**
- Create: `native/src/terrain/grid3d.hpp`, `native/src/terrain/grid3d.cpp`, `native/src/writers/pneumatic_3d_tire.hpp`, `native/src/spatial_topology.hpp`
- Modify: `native/CMakeLists.txt`
- Test: `native` (compiles both compilers, all sweeps); `native/tests/test_contracts.cpp` gains "unimplemented bodies abort" cases

**Interfaces:** exactly spec §5.1 and §5.2. `grid3d.cpp` bodies throw `std::logic_error("spatial-3d P2: Grid3D::sample not implemented")` (helper `[[noreturn]] void unimplemented(std::string_view)` in a new `native/src/unimplemented.hpp`; `diag.hpp` is a pragma helper and is not touched). `from_profile_extrusion` is part of the contract and is implemented in P1. `pneumatic_3d_tire.hpp` declares `PatchWrench`, `TyreOutput`, `constexpr int kMaxPatches3d = 8`, `std::vector<std::string> state_wire_names(const nativecfg::Pneumatic3dConfig&)` (implemented now — it is pure naming), and the class `Pneumatic3dTireWriter` declaration with the same facets as `PneumaticTireWriter` (`try_compute_into`, `state_row`, `set_state_row`, `snapshot`).

- [ ] **Step 1:** Write the headers; add to CMake library and to every sweep's source list (`native_sources.json` generation if used — check `tools/native_checks.py --kind context`).
- [ ] **Step 2:** `test_contracts.cpp`: `state_wire_names` for fast tier has `2 × (64×7×3 + 1 + rim_fields + 1 + 8×2)` entries in the canonical order (compute from the formula, assert the first/last names); `Grid3D::load` on an empty span returns `std::unexpected`; `Grid3D::sample` on a default-constructed grid throws `std::logic_error` (caught in-process — no death test needed).
- [ ] **Step 3:** `bash tools/run_tests.sh native` green.
- [ ] **Step 4:** Commit `feat(native): declare spatial-3d contracts (Grid3D, PatchWrench, Spatial topology)`.

### Task 4: Config schema (5.3) — Python dataclasses, wire projection, native parse/validate

**Files:**
- Modify: `src/bike_sim/physics/model_config.py`, `src/bike_sim/physics/physical_config.py:80`, `src/bike_sim/native/config.py`, `src/bike_sim/native/schema.py`, `src/bike_sim/native/setup.py:48-62`
- Modify: `native/src/config_types.hpp`, `native/src/config_validation.hpp`, `native/src/config.hpp`
- Test: `tests/reference/test_native_runtime_config.py` (extend), new `tests/test_spatial_config.py`; profile `native`

**Interfaces:** spec §4.1 keys and rules; wire shape spec §5.3.

- [ ] **Step 1 (failing tests):** `tests/test_spatial_config.py`: default `spatial == "planar"` and the projected wire has `spatial.mode == "planar"` with everything else `None`; `[spatial_3d]` without `spatial = "3d"` → `ValueError`; `backend = "pneumatic_3d"` with planar → `ValueError`; `spatial = "3d"` with `backend = "pneumatic"` → `ValueError`; `root = "constrained"` with `[spatial_3d.steer]` → `ValueError`; a full valid 3D TOML round-trips through `src/bike_sim/native/config.py` projection and `src/bike_sim/native/schema.py` validation.
- [ ] **Step 2:** Implement the dataclasses (`Spatial3dConfig`, `SteerConfig`, `SteerGains`, `BootstrapConfig`, `ImuMountConfig`, `Pneumatic3dTierConfig`) with `scalar(...)` validation in the repository style; defaults from spec §4.1.
- [ ] **Step 3:** Projection + schema; `validate_supported` pairing rules; native `nativecfg::SpatialConfig`/`Pneumatic3dConfig` parse with `exact_keys`-equivalent strictness and the same rejections.
- [ ] **Step 4:** Prove planar bytes unchanged: run `bash tools/run_tests.sh native` and `tests/reference/test_native_runtime_rollout.py` — all green; the projected wire for every `examples/research/*.toml` is **byte-identical** to the pre-change wire (the `spatial` key is emitted only for `spatial = "3d"`; native treats its absence as planar). Add a test that diffs the projection against a stored sha256 of the pre-change wire for each example TOML.
- [ ] **Step 5:** Commit `feat(config): spatial mode schema with planar default and pneumatic_3d pairing rules`.

### Task 5: Observation schema 2 (5.4) and telemetry field declarations (5.5)

**Files:**
- Modify: `src/bike_sim/sim/research/sensors.py`, `src/bike_sim/recording/catalog.py`
- Test: `tests/reference/test_spatial3d_contracts.py` (stubs), extend the existing sensors tests; profile `quick`

- [ ] **Step 1:** Add `SensorObservationV2` (spec §5.4) alongside the existing class; `SensorConfig` gains `gyro_std_rad_s: tuple[3]`, `accel_std_mps2: tuple[3]`, `gyro_bias_rad_s`, `accel_bias_mps2` (3-vectors) while the scalar legacy fields remain for schema 1; `build_noise_tape(..., schema=1|2)` signature extended with schema 1 output byte-identical (test by comparing tapes for a fixed seed before/after).
- [ ] **Step 2:** `catalog.py`: declare every spec §5.5 field with type/unit/frame/source and `produced_by = "spatial-3d P<N>"`; add a catalog test that fields declared-but-not-produced are reported, not failed, until the producing phase sets `produced = True`.
- [ ] **Step 3:** `test_spatial3d_contracts.py`: five xfail(strict) stubs, one per contract, each importing the symbol and calling the unimplemented path.
- [ ] **Step 4:** Commit `feat(research): observation schema 2 and 3D telemetry catalog declarations`.

### Task 6: Self-golden manifest tool

**Files:**
- Create: `tools/golden_manifest.py`, `tests/test_golden_manifest.py`, `tests/reference/golden_manifests/spatial3d/README.md`
- Test: `quick`

**Interfaces:** `write(episode: Episode, path: Path, window_steps: int = 100) -> dict` producing `{"schema": 1, "window_steps", "groups": {"<channel-root>": ["sha256-hex per window", ...]}, "state": {"first": sha256(qpos|qvel|act|time bytes), "last": ...}, "provenance": {"model_sha256", "config_sha256", "terrain_sha256", "mujoco_version", "git_head"}}`; `check(episode, manifest) -> list[Mismatch]` with window index and group; CLI `python tools/golden_manifest.py {write,check} …`. Uses `tools/golden_episode.py::load_episode` as input.

- [ ] **Step 1 (failing tests):** manifest round-trip on a 40-step planar golden captured in-test (reuse `_golden` from `tests/reference/test_native_suspension.py`); flipping one byte of one channel in one window is reported with the right window/group; changing `window_steps` is rejected on check.
- [ ] **Step 2:** Implement; hash with `hashlib.sha256` over the raw `float64` bytes in canonical channel order (the catalog order).
- [ ] **Step 3:** README: "A manifest changes only by an explicit commit whose message states the physical reason."
- [ ] **Step 4:** Commit `tools: self-golden sha256 manifests for spatial-3d episodes`.

### Task 7: Phase audit and gate

- [ ] **Step 1:** `bash tools/run_tests.sh full` on the branch; confirm every pre-existing golden test is byte-identical (no manifest or golden diffs; rollout test green).
- [ ] **Step 2:** Write `docs/superpowers/audits/2026-10-14-spatial-3d-p0.md`: contract header list, pin provenance table (with `unpinned` items highlighted), projection snapshot diff, sweep results.
- [ ] **Step 3:** Merge to `main`; tag `spatial-3d-p0`.
