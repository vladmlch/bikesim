# Tracks and Surfaces Simulation (Stages B, C, D, Meso-Obstacles, Viewer Ribbon & Calibrated Tracks) Implementation Plan

> **For agentic workers:** Executed via `/swarm` with strict model routing agreed with the user (2026-11-23):
> - Lightweight / track / fixture / gate tasks: `gemini-4-hydrogen-high`
> - Complex C++ physics / runtime / perception / viewer tasks: `gemini-4-neon-medium`
> - Task reviews, scoped re-reviews, and final whole-branch review: `gemini-4-carbon-medium`

**Goal:** Deliver physically faithful track obstacle primitives (`StepUp`, `ISO 8608` roughness), local obstacle micro-surfaces (Stage C), finite-thickness deformable granular layers with 1D rut memory and separate soil energy/loss accounting (Stage B), rough-terrain rider perception + strict `surface_rear` / `initial_grade_rad` codecs + viewer HUD & 3D surface color ribbon (Stage D), and fully strict calibrated research tracks without breaking frozen tracks or `compliant_2d` parity.

**Architecture:** Additive, zero-allocation (`BIKE_NONBLOCKING`) extensions layered cleanly atop Stage A: (1) terrain-generation primitives & curvature gates in `bike_terrain`, (2) pre-expanded `MicroSurfaceSpan` binary-search overlay in `tirecommon`, (3) pre-allocated 1D `RutMemory` grid coupled to `PneumaticTireWriter` with exact §4.1 constants and separate `"soil"` elastic + `ground_plastic_loss_step_j` accounting lines, and (4) chord-based `road_grade_chord_preview`, trail-relative wheelie pitch in `Perception`, strict `surface_rear` in `RiderSenses`, and `Decorations::surface_bands` 3D track ribbon rendering in `bike_viewer`.

**Tech Stack:** C++23, MuJoCo 3.12 (`mjtNum = double`), CMake/CTest (`build/native-release` and `build/gui`), Accelerate BLAS.

**Spec:** `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18/docs/superpowers/specs/2026-11-22-tracks-and-surfaces-design.md`

## Global Constraints

- Frozen acceptance tracks (`examples/research/rough_uphill.toml`, `rough_uphill_extreme.toml`, `rough_uphill_savage.toml`, `alpine_scree`, `alpine_rocky`) MUST NOT be modified, and their `terrain::identity_value` / `track_sha256` MUST stay bitwise identical.
- `compliant_2d` (`TireWriter`, `viewer_physics_welded.toml`) and `distributed_2d_reference` physics laws MUST NOT be recalibrated; serialized `config.json` for `compliant_2d` remains bitwise unchanged when new optional track fields are absent.
- Zero heap allocations on the hot path (`BIKE_NONBLOCKING` in `PneumaticTireWriter::try_compute_into` and `DistributedTireWriter::try_compute_into`).
- Strict Ban #39: zero linear viscous damping `c * z_dot` or `c * delta_dot` anywhere in tire or soil laws; energy balance residual `|residual_j| <= 0.05 J` on all measured intervals.
- `advance == false` probes (`PhysicalStep::probe_step_inputs`, `SpindleController` Newton iterations) MUST NEVER mutate `RutMemory` or brush states.
- Strict wire codecs (`ncd::exact`): `RiderSenses` (`surface_rear`) and `Perception::State` (`initial_grade_rad`) MUST use strict `ncd::exact` validation with no silent legacy fallback; affected `darwin-arm64` fixtures are refreshed cleanly in Task 5.
- Viewer 3D surface coloring MUST use `viewer::Decorations` (`mjvScene` geoms in `RenderOwner::draw`), NEVER mutating `mjModel` or changing `model_sha256`.
- Information boundary (`test_native_rider_information_boundary`): motor and research policies MUST NEVER access `RoadProfile`, `SurfaceMap`, `MicroSurfaceSpan`, `RutMemory`, or `RiderSenses`.
- Work strictly inside `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18` reusing `build/native-release` and `build/gui`. Never run `rm -rf`, `git reset --hard`, `git clean`, `git stash`, `git add -A`, `git add .`, or `git push`.

---

### Task 1: Research Docs Commit, `StepUp` Ledge Primitive, `ISO 8608` Roughness, and Curvature Gates — **COMPLETE (`1d243969..589a7059`, reviewed clean)**

- [x] **Step 1–5:** Implemented `StepUp`, `RoadRoughness(spectrum="iso8608", waviness)`, `contact_safety.cpp` curvature gates, and committed research/spec/plan docs (`e0502f1b`, `589a7059`).

---

### Task 2: Stage C — Local Obstacle Micro-Surfaces (`wet_wood`, `mud_lens`, `wet_rock`) and Zero-Allocation Sub-Patch Overlay — **COMPLETE (`589a7059..e37ec5a0`, reviewed clean)**

- [x] **Step 1–5:** Implemented 5 micro-surfaces (`dry_wood`, `wet_wood`, `polished_rock`, `wet_rock`, `mud_lens`), `Obstacle::micro_surface_spans()`, `nativecfg::SurfaceMap::micro_spans`, zero-allocation $O(\log M)$ Hermite collar evaluation in `tire_common.cpp`, and Stage C tests (`8d7f577b`, `005c4564`, `256008c8`, `e37ec5a0`).

---

### Task 3: Stage B — Deformable Thin Granular Layer (`loose`, `scree`), Spec §4.1 Constants, and Separate Soil Energy Ledger

**Model Tier:** `gemini-4-neon-medium` (author) + `gemini-4-carbon-medium` (review & re-review)

**Current State (`9cfc3722`):** Core `RutMemory`, `layer_thickness_m`, coupled tire/soil solve, 16-point Gauss–Legendre work integration, probe purity, and sparse `rut.node.*` state serialization are implemented in `9cfc3722`, awaiting Task 3 review (`e37ec5a0..9cfc3722`) plus the two user-mandated Stage B refinements (2026-11-23 rulings A1/A2/A3).

**Files:**
- Review baseline: `e37ec5a0..9cfc3722` (`native/src/tyre/rut_memory.{hpp,cpp}`, `native/src/pneumatic/model.{hpp,cpp}`, `native/src/writers/pneumatic_tire.{hpp,cpp}`, `native/src/config_*`, `native/src/terrain/*`, `native/tests/test_rut_memory.cpp`, `native/tests/reference/test_pneumatic_energy_balance.cpp`, `native/tests/reference/test_native_input_contracts.cpp`)
- Modify in fix round:
  - `docs/superpowers/specs/2026-11-22-tracks-and-surfaces-design.md`, `docs/superpowers/plans/2026-11-22-tracks-and-surfaces.md`
  - `native/src/tyre/rut_memory.cpp`
  - `native/src/writers/pneumatic_tire.hpp`, `native/src/writers/pneumatic_tire.cpp`
  - `native/src/stepper.hpp`, `native/src/stepper.cpp` (if `TireWriterVariant` / helper forwards `has_deformable_layer()`, `soil_elastic_energy_j()`, `ground_plastic_loss_step_j()`)
  - `native/src/runtime/accounting.hpp`, `native/src/runtime/accounting.cpp`
  - `native/src/runtime/step.cpp`
  - `native/src/recording/catalog_data.inc`
  - `native/tests/test_rut_memory.cpp`, `native/tests/reference/test_pneumatic_energy_balance.cpp`, `native/tests/recording/test_catalog.cpp`

**Interfaces & Exact Requirements:**
1. **Exact Spec §4.1 Soil Constants & $C^1$ Toe Regularization (`rut_memory.cpp`):**
   - Set `loose` params verbatim to: `k0_pa = 180e3, n = 0.80, zeta_o = 0.45, zeta_w = 0.30, b_hp_pa = 500e3, c_hp = 4.5, beta_u = 6.0, mu_basal = 0.52, rho_b_kg_m3 = 1650.0, alpha_tau = 1.0, alpha_n = 0.55`.
   - Set `scree` params verbatim to: `k0_pa = 130e3, n = 0.85, zeta_o = 0.40, zeta_w = 0.30, b_hp_pa = 650e3, c_hp = 5.0, beta_u = 5.0, mu_basal = 0.42, rho_b_kg_m3 = 1750.0, alpha_tau = 1.0, alpha_n = 0.45`.
   - Regularize the Bekker base term at $z_\varepsilon = 10^{-4}\text{ m}$ (`0.1 mm`): `p_base(z) = k0_pa * (std::pow((z + z_eps) / b, n) - std::pow(z_eps / b, n))` and `dp_base/dz = k0_pa * n * std::pow((z + z_eps) / b, n - 1.0) / b` so $p_{\text{virgin}}(0) = 0$ and $p'_{\text{virgin}}(z)$ is finite, positive, and $C^1$-continuous for all $z \in [0, H_s)$ (matching centered finite differences at $z = 0$ and across $z_o$).
2. **Separate Soil Elastic & Ground Plastic Dissipation Ledger Lines (`pneumatic_tire.*`, `step.cpp`, `accounting.*`, `catalog_data.inc`):**
   - `PneumaticTireWriter::stored_energy(d)` and `PneumaticTireWriter::elastic_energy_j()` return **tire-only** elastic energy (radial carcass + rim + Maxwell + bristle shear), keeping `stored_energy(d) == elastic_energy_j()` bitwise; `PneumaticTireWriter::soil_elastic_energy_j()` returns soil elastic energy $U_{\text{soil},e} \ge 0$.
   - `PneumaticTireWriter::brush_loss_step_j()` returns **tire-only** brush/slip dissipation (do NOT add soil plastic work into `brush_loss_step_j` or `diagnostics_[side].brush_loss_j`); `PneumaticTireWriter::ground_plastic_loss_step_j()` returns soil plastic step work $\Delta D_{\text{soil},p} \ge 0$ (`0.0` when `has_deformable_layer() == false`), and `clear_settled_loss()` clears settled step losses cleanly.
   - In `step.cpp`:
     - When building `elastic` (`WireObject` for `elastic_energy_j`) and `total`: `"tires"` holds tire-only elastic energy; when `stepper_->has_tire() && stepper_->tire_writer().has_deformable_layer()`, append `"soil"` with `soil_elastic_energy_j()` and add it into `total` (so `total` = mechanical + all `elastic_energy_j` terms!). When `has_deformable_layer() == false`, `"soil"` is NOT emitted.
     - Pass `raw.ground_plastic_loss_step_j` (`0.0` when no deformable layer) and include `ground_plastic_loss_step_j` in `raw.loss_step_j` (`loss_increment`).
   - In `accounting.hpp`, `accounting.cpp`, and `recording/catalog_data.inc`:
     - Mirror `rider_end_stop_loss_step_j` verbatim: emit mandatory `"ground_plastic_loss_step_j": Wire(raw.ground_plastic_loss_step_j)` in `account_energy`'s `result` right after `"rider_end_stop_loss_step_j"` (`optional: false` in `catalog_data.inc`, description `"deformable ground plastic compaction dissipation of this interval (included in loss_step_j; Stage B)"`). Note: since `raw.loss_step_j` already includes `ground_plastic_loss_step_j`, do NOT add it a second time to `e.loss_j`!

- [x] **Step 1–2 (Completed in `9cfc3722`):** Core `RutMemory` implementation & tests.
- [ ] **Step 3: Run Task 3 Review (`e37ec5a0..9cfc3722`) on `gemini-4-carbon-medium`**
  Verify `9cfc3722` against the brief and write `.superpowers/sdd/2026-11-22-tracks-and-surfaces/task-3-review.md`.
- [ ] **Step 4: Implement Spec §4.1 Constants, Separate Soil Energy/Loss Channels, and Any Review Fixes (`gemini-4-neon-medium`)**
  Update `test_rut_memory.cpp`, `test_pneumatic_energy_balance.cpp` (explicitly asserting `energy["elastic_energy_j"]["soil"] > 0`, `energy["ground_plastic_loss_step_j"] >= 0`, `stored_energy(d) == elastic_energy_j()` for tire-only, and `|residual_j| <= 0.05 J`), and `test_catalog.cpp`. Run targeted tests (`native_rut_memory_tests`, `reference.pneumatic_energy_balance`, `reference.native_pneumatic_model`, `reference.native_pneumatic_tire`, `reference.native_input_contracts`, `recording_catalog`) + `bike-native-checks --kind tidy/cppcheck/headers` on touched TUs, and commit (Note: golden tests reading frozen `energy` JSON rows before Task 5's fixture refresh will be refreshed in Task 5).
- [ ] **Step 5: Scoped Re-Review on `gemini-4-carbon-medium`**

---

### Task 4: Stage D — Rough-Terrain Rider Perception Fixes, Strict `surface_rear` / `initial_grade_rad` Codecs, Viewer HUD & 3D Surface Ribbon

**Model Tier:** `gemini-4-neon-medium` (author) + `gemini-4-carbon-medium` (review)

**Files:**
- Modify: `native/src/runtime/control.hpp`, `native/src/runtime/control.cpp`
- Modify: `native/src/runtime/step.cpp`
- Modify: `native/src/runtime/runtime.hpp`, `native/src/runtime/runtime.cpp`
- Modify: `native/src/rider/senses.hpp`, `native/src/rider/senses_codec.cpp`
- Modify: `native/src/rider/perception.hpp`, `native/src/rider/perception.cpp`
- Modify: `native/src/rider/launch_plan.cpp`
- Modify: `native/src/recording/catalog_data.inc`
- Modify: `native/src/viewer/hud.cpp`
- Modify: `native/src/viewer/decorations.hpp`, `native/src/viewer/render_owner.cpp`, `native/src/viewer/ride.cpp`
- Test: `native/tests/reference/test_rider_senses.cpp`, `native/tests/reference/test_rider_perception.cpp`, `native/tests/reference/test_seated_posture_program.cpp`, `native/tests/reference/test_launch_plan.cpp`, `native/tests/reference/test_native_rider_information_boundary.cpp`, `native/tests/recording/test_catalog.cpp`, `native/tests/viewer/test_hud.cpp`, `native/tests/viewer/test_render.cpp`

**Interfaces & Exact Requirements:**
1. **`runtime::road_grade_chord_preview(std::span<const RoadSample> road, double front_wheel_x_m)` (`control.{hpp,cpp}`, `step.cpp`):**
   - Interpolates/locates `h_front` from `road` at `front_wheel_x_m` and returns `(road.back().height_m - h_front) / (road.back().x_m - front_wheel_x_m)` when `road.back().x_m - front_wheel_x_m > 1e-9` (falling back to `road_grade_preview(road)` otherwise). Keep `road_grade_preview(road)` intact for unit tests; switch `step.cpp:1290` and `step.cpp:2920` to `road_grade_chord_preview(road, wheel_x_m_[0])`.
2. **Trail-Relative `Perception::pitch_rad` with Strict `initial_grade_rad` State (`perception.{hpp,cpp}`, `senses_codec.cpp`):**
   - Add `std::optional<double> initial_grade_rad{};` to `Perception::State`.
   - In `Perception::update`: when `delayed != nullptr`, compute `const double trail_rad = std::atan(delayed->senses.grade_ahead);`, latch `const double init_rad = s_.initial_grade_rad.value_or(trail_rad);`, compute `const double pitch_rel = pitch - (trail_rad - init_rad);` (check `std::isfinite(pitch_rel)` before mutating `s_`), publish `.pitch_rad = pitch_rel` in `Percepts`, and commit `s_.initial_grade_rad = init_rad;` alongside `s_.pitch_rad = pitch;`.
   - In `Perception::restore`: enforce `if (state.initial_grade_rad.has_value() != (state.tick > delay_ticks_)) throw std::invalid_argument(...);` and `if (state.initial_grade_rad && !std::isfinite(*state.initial_grade_rad)) throw std::invalid_argument(...);`.
   - In `senses_codec.cpp` (`decode_perception_state` / `encode_perception_state`): **STRICT `ncd::exact(d, keys("tick", "samples", "delayed", "pitch_rad", "initial_grade_rad"))`** (`null` when disengaged, `real_value` when engaged; no legacy fallback!).
3. **Strict `RiderSenses::surface_rear` & `LaunchPlan` Traction Cap (`senses.hpp`, `senses_codec.cpp`, `step.cpp`, `launch_plan.cpp`, `catalog_data.inc`):**
   - Add `SurfaceLabel surface_rear = SurfaceLabel::hardpack;` to `RiderSenses`, populate in `step.cpp:2914` from `config_.surface_labels.at(wheel_x_m[1])`, validate in `perception.cpp:check_sample`, emit in `step.cpp:2091` (`"surface_rear"` right before `"surface_front"`) and `catalog_data.inc`, and enforce **STRICT `ncd::exact(d, keys("hand_press_n", "speed_mps", "rear_teeth", "surface_rear", "surface_front", "surface_ahead", "grade_ahead"))`** in `senses_codec.cpp`.
   - In `LaunchPlan::ramp` (`launch_plan.cpp:107`): `const double surface = std::min(cap(p.senses.surface_rear), cap(p.senses.surface_ahead));`.
4. **Viewer HUD Column & 3D Track Surface Color Ribbon (`viewer/hud.cpp`, `decorations.hpp`, `render_owner.cpp`, `ride.cpp`, `runtime.{hpp,cpp}`):**
   - `hud.cpp`: in `physical_columns`, read `rider_senses` (`surface_rear`, `surface_front`, `surface_ahead`) and append a compact column `add("surf r/f/a", "surf=" + abbr(r) + "/" + abbr(f) + "/" + abbr(a))` (`asp`, `hpk`, `wet`, `lse`, `scr`, or `---` when absent).
   - `decorations.hpp`: add `struct SurfaceBand { std::array<double, 3> from{}; std::array<double, 3> to{}; double radius_m = .018; std::array<float, 4> rgba{.35f, .35f, .38f, .9f}; };` and `std::vector<SurfaceBand> surface_bands{};` on `Decorations`.
   - `render_owner.cpp`: validate finiteness + positive `radius_m` of `surface_bands` in `set_decorations`, and draw each band in `RenderOwner::draw` via `mjv_initGeom` + `mjv_connector(geom, mjGEOM_CAPSULE, band.radius_m, band.from.data(), band.to.data())` into `owner->scene.geoms` (presentation-only, `mjModel` and `model_sha256` untouched!).
   - `runtime.{hpp,cpp}` & `viewer/ride.cpp`: add `std::vector<viewer::SurfaceBand> surface_bands_near(double center_x_m, double back_m = 8.0, double ahead_m = 18.0) const;` (or free helper over `NativeRideRuntime`'s owned `RoadProfile` + `SurfaceLabels` + `SurfaceMap::micro_spans`), sampling ~0.20 m segments (finer over micro-spans) slightly offset on the road edge (`y = 0.22 m`, or full strip) with distinct RGBA colors for all 5 macro surfaces (`asphalt` dark slate, `hardpack` warm tan, `wet` cool blue-slate, `loose` amber gravel, `scree` pale limestone) and 5 Stage C micro-surfaces (`dry_wood` oak brown, `wet_wood` glossy dark umber/cyan tint, `polished_rock` light quartz, `wet_rock` teal-grey, `mud_lens` deep mud brown), wired into `hooks.decorations` in `viewer::run_ride`.
5. **Sensor Boundary Test (`test_native_rider_information_boundary.cpp`):**
   - Assert research `Observation` is bitwise identical across two runs whose track differs in `surface_ahead` (`asphalt` vs `scree` 1.5 m ahead of the front wheel).

- [ ] **Step 1: Write failing unit tests across perception, senses, launch_plan, HUD, render, catalog, and information_boundary**
- [ ] **Step 2: Build and run targeted tests to verify RED**
- [ ] **Step 3: Implement Stage D perception, strict codecs, `LaunchPlan` cap, HUD, and 3D viewer `surface_bands`**
- [ ] **Step 4: Build and run targeted tests + static checks (`tidy`, `cppcheck`, `headers`) to verify GREEN**
- [ ] **Step 5: Commit and pass Task 4 review (`gemini-4-carbon-medium`)**

---

### Task 5: Strictly Validated Calibrated Research Tracks, Darwin Fixture Refresh, Savage Benchmark, `simulation-decisions.md`, and `build/gui`

**Model Tier:** `gemini-4-hydrogen-high` (author) + `gemini-4-carbon-medium` (review)

**Files:**
- Create: `examples/research/rough_uphill_trail_calibrated.toml`
- Create: `examples/research/alpine_scree_deformable.toml`
- Create: `native/tests/reference/test_calibrated_surface_tracks.cpp` (registered in `native/tests/reference/Phase3Ports.cmake`)
- Modify: `docs/agents/simulation-decisions.md`
- Refresh genuinely affected `darwin-arm64` fixtures & poses (`tests/fixtures/config/poses/savage_pneumatic.npz` via `bike-settle`, plus `tests/fixtures/**` touched by `ground_plastic_loss_step_j`, `surface_rear`, `initial_grade_rad`, and `road_grade_chord_preview`)

**Interfaces & Exact Requirements:**
1. **Two Calibrated Research Tracks Passing Full Model Strictness on `viewer_physics_pneumatic.toml`:**
   - `examples/research/rough_uphill_trail_calibrated.toml`: realistic seated e-MTB climb (`0 -> 14% -> 22% -> 28% -> 0%`, smooth transitions, `hardpack` + `wet` transition with `blend_m`, `RoadRoughness` with `spectrum = "iso8608"` and `correlation_length_m >= 0.30`, `StepUp` bedrock ledges passing `single_contact_hazards`, `Roots` with `surface = "wet_wood", surface_blend_m = 0.03`, `SlopedPothole`/`GOut` with `surface = "mud_lens"`).
   - `examples/research/alpine_scree_deformable.toml`: alpine climb (`hardpack -> loose (layer_thickness_m = 0.035, blend_m = 0.5) -> scree (layer_thickness_m = 0.050, blend_m = 0.5) -> hardpack`, gentle `iso8608` roughness + `StepUp` ledge).
   - `native/tests/reference/test_calibrated_surface_tracks.cpp`: runs both tracks end-to-end on `viewer_physics_pneumatic.toml` and asserts **full acceptance strictness**:
     - `outcome == "end_of_track"`, `!crashed`, `!balance_lost`;
     - `model_valid == true` and `first_failure == std::nullopt`;
     - `saddle.min_n >= -6.0`, `foot_left.min_n >= -6.0`, `foot_right.min_n >= -6.0`, `grip_left.max_pull_n <= 300.0`, `grip_right.max_pull_n <= 300.0`;
     - `research::energy_quality(energy, 0.05)` has `acceptable == true` and `residual_ratio <= 0.05`, and every measured interval has `|residual_j| <= 0.05`;
     - on `alpine_scree_deformable.toml`, cumulative `ground_plastic_loss_step_j > 0.0` and peak `"soil"` in `elastic_energy_j > 0.0`.
2. **Diagnostic Progress Benchmark on Frozen `rough_uphill_savage.toml`:**
   - Without modifying `examples/research/rough_uphill_savage.toml` (`track_sha256` verified unchanged!), run headless diagnostic rides on `viewer_physics_welded.toml` and `viewer_physics_pneumatic.toml`, recording achieved `position_m`, `time_s`, `reason`, and `balance_lost` in `task-5-report.md` and `docs/agents/simulation-decisions.md`.
3. **Darwin-arm64 Fixture Refresh & `build/gui` Rebuild:**
   - Regenerate `tests/fixtures/config/poses/savage_pneumatic.npz` via `bike-settle` and refresh only the `darwin-arm64` golden/episode/bootstrap fixtures genuinely affected by `ground_plastic_loss_step_j`, `surface_rear`, `initial_grade_rad`, and `road_grade_chord_preview` using the repository's official capture/golden tools (never weakening any physical tolerance!).
   - Rebuild `cmake --build build/gui --target bike-ride -j8`.
   - Update `docs/agents/simulation-decisions.md` (items 12, 23, 32, 34, 42) with exact 2026-11-23 user decisions (session `eea0aaf4`) and commit hashes.
   - Run `bash tools/check_change.sh 1d243969` until 100% exit 0.

- [ ] **Step 1: Author both calibrated TOML tracks and `test_calibrated_surface_tracks.cpp` and verify full strictness**
- [ ] **Step 2: Refresh affected `darwin-arm64` fixtures & `savage_pneumatic.npz`, run `savage` diagnostic benchmark, and pass `bash tools/check_change.sh 1d243969`**
- [ ] **Step 3: Rebuild `build/gui` (`bike-ride`) and update `docs/agents/simulation-decisions.md`**
- [ ] **Step 4: Commit and pass Task 5 review (`gemini-4-carbon-medium`)**

---

### Task 6: Final Whole-Branch Review & Extended `full` Gate (`1d243969..HEAD`)

**Model Tier:** `gemini-4-carbon-medium` (final whole-branch review) + `gemini-4-neon-medium` (if a fix wave is needed) + `gemini-4-hydrogen-high` (`full` gate runner)

- [ ] **Step 1: Generate whole-branch review package (`1d243969..HEAD`) and run Final Whole-Branch Review on `gemini-4-carbon-medium` (including triaging deferred minors from `progress.md`)**
- [ ] **Step 2: If any Critical/Important finding arises, execute ONE fix wave (`gemini-4-neon-medium`) + ONE scoped re-review (`gemini-4-carbon-medium`)**
- [ ] **Step 3: Run `bash tools/run_tests.sh full` inside `.worktrees/surface-stage-a-2026-11-18` until complete exit 0 (`native` static sweeps + ordinary CTest + `slow` + `package` labels), and stop before merging into `main` for user review.**
