# Tracks and Surfaces Simulation (Stages B, C, D, Meso-Obstacles & Rider Perception) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver physically faithful track obstacle primitives (`StepUp`, `ISO 8608` roughness), local obstacle micro-surfaces (Stage C), finite-thickness deformable granular layers with 1D rut memory and exact energy accounting (Stage B), and rough-terrain rider perception + `surface_rear` awareness (Stage D) without breaking frozen tracks or `compliant_2d` parity.

**Architecture:** Additive, zero-allocation (`BIKE_NONBLOCKING`) extensions layered cleanly atop Stage A: (1) terrain-generation primitives & curvature gates in `bike_terrain`, (2) pre-expanded `MicroSurfaceSpan` binary-search overlay in `tirecommon`, (3) pre-allocated 1D `RutMemory` SoA grid in `bike_core` coupled to `PneumaticTireWriter` via local Newton equilibrium and `accounting.cpp`, and (4) chord-based `road_grade_chord_preview`, trail-relative wheelie pitch in `Perception`, and `surface_rear` in `RiderSenses`.

**Tech Stack:** C++23, MuJoCo 3.12 (`mjtNum = double`), CMake/CTest (`build/native-release`), Accelerate BLAS.

**Spec:** `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18/docs/superpowers/specs/2026-11-22-tracks-and-surfaces-design.md`

## Global Constraints

- Frozen acceptance tracks (`examples/research/rough_uphill.toml`, `rough_uphill_extreme.toml`, `rough_uphill_savage.toml`, `alpine_scree`, `alpine_rocky`) MUST NOT be modified, and their `terrain::identity_value` / `track_sha256` MUST stay bitwise identical.
- `compliant_2d` (`TireWriter`, `viewer_physics_welded.toml`) behavior and serialized `config.json` MUST remain bitwise unchanged when new optional track fields are absent.
- Zero heap allocations on the hot path (`BIKE_NONBLOCKING` in `PneumaticTireWriter::try_compute_into` and `DistributedTireWriter::try_compute_into`).
- Strict Ban #39: zero linear viscous damping `c * z_dot` or `c * delta_dot` anywhere in tire or soil laws; energy balance residual `|residual_j| <= 0.05 J` on all measured intervals.
- `advance == false` probes (`PhysicalStep::probe_step_inputs`, `SpindleController` Newton iterations) MUST NEVER mutate `RutMemory` or brush states.
- Information boundary (`test_native_rider_information_boundary`): motor and research policies MUST NEVER access `RoadProfile`, `SurfaceMap`, `MicroSurfaceSpan`, `RutMemory`, or `RiderSenses`.
- Build & run targeted tests inside `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18` reusing `build/native-release`. Never run `rm -rf`, `git reset --hard`, `git clean`, `git stash`, or `git push`.

---

### Task 1: Research Docs Commit, `StepUp` Ledge Primitive, `ISO 8608` Roughness, and Curvature Gates

**Model Tier:** `swe-2-flash-max` (`sdd-luna`)

**Files:**
- Commit existing untracked docs: `docs/research/2026-11-22-tracks-and-surfaces/00..08-*.md`, `docs/superpowers/specs/2026-11-22-tracks-and-surfaces-design.md`
- Modify: `native/src/terrain/types.hpp`
- Modify: `native/src/terrain/obstacles.cpp`
- Modify: `native/src/terrain/contact_safety.cpp`
- Modify: `native/src/terrain/trackfile.cpp`
- Modify: `native/src/terrain/spec_io.cpp`
- Test: `native/tests/terrain/test_generators.cpp`, `native/tests/terrain/test_presets.cpp`

**Interfaces:**
- Produces:
  - `struct bike::terrain::StepUp { double start_m = 0., height_m = .060, ramp_m = .120; };` added to `Obstacle::Shape` variant (at the end of `std::variant` so existing variant indices `0..13` do not shift).
  - `RoadRoughness` optional fields: `std::string spectrum = "gaussian"; double waviness = 2.0;` (omitted in `spec_io.cpp` / `trackfile.cpp` `obstacle_fields` when `spectrum == "gaussian" && waviness == 2.0`).
  - `StepUp` TOML key `type = "step_up"`, `length_m() == ramp_m`, `datum_shift_m() == +height_m`, `elevation(x) = 0.5 * height_m * (1.0 - numeric::np_cos(pi * x / ramp_m))` on `[0, ramp_m)`.
  - `single_contact_hazards` checks `StepUp` (`ramp_m < pi * std::sqrt(0.5 * height_m * radius) - 1e-9`) and `SteppedClimb` (`tlen <= 0 || 1.5 * std::abs(dg) / tlen > 1.0 / radius + 1e-9`).

- [ ] **Step 1: Write failing unit tests in `native/tests/terrain/test_generators.cpp` and `test_presets.cpp`**
  Add tests verifying:
  1. `StepUp` (`type = "step_up"`, `start_m = 5.0, height_m = 0.08, ramp_m = 0.40`): `datum_shift_m() == 0.08`, profile stays `+0.08 m` higher for `x >= 5.40`, `z(5.0) == 0`, `z(5.20) == 0.04`, TOML round-trip `dump_track(load_track(...))` works, and `single_contact_hazards` passes for `ramp_m = 0.45` (`> pi * sqrt(0.5 * 0.08 * 0.372) = 0.383 m`) and flags hazard for `ramp_m = 0.15`.
  2. `RoadRoughness` with `spectrum = "iso8608", waviness = 2.0, amplitude_m = 0.03, correlation_length_m = 0.20, section_length_m = 40.0`: RMS of interior `[5.0, 35.0]` matches `amplitude_m / 3.0` within `25%` across both `20 m` and `80 m` sections (length-invariant RMS), log-log PSD slope is within `[-2.5, -1.5]`, invalid `spectrum = "bogus"` or `waviness = 0.5` throws `std::invalid_argument`, and default `RoadRoughness` (`spectrum == "gaussian"`) keeps bitwise-identical output and `track_sha256` on all 5 shipped tracks in `test_presets.cpp`.

- [ ] **Step 2: Build and run tests to verify RED**
  Run: `cmake --build build/native-release --target test_terrain_generators test_terrain_presets -j8 && ctest --test-dir build/native-release -R 'terrain\.(generators|presets)' --output-on-failure`
  Expected: FAIL (missing `StepUp` / `spectrum` fields).

- [ ] **Step 3: Implement `StepUp`, `RoadRoughness` `iso8608`, TOML/identity serialization, and `contact_safety.cpp` checks**
  - In `types.hpp`: append `StepUp` to `Obstacle::Shape`; add `std::string spectrum = "gaussian"; double waviness = 2.0;` to `RoadRoughness`.
  - In `obstacles.cpp`:
    - Update `names` array to size `15` (`"StepUp"` at index `14`), `length(const StepUp&)` -> `s.ramp_m`, `datum_shift_m()` -> `+s.height_m`, `validate()` -> `detail::positive(s.height_m, "StepUp.height_m"); detail::positive(s.ramp_m, "StepUp.ramp_m");` and for `RoadRoughness` validate `spectrum == "gaussian" || spectrum == "iso8608"` and `1.0 <= waviness && waviness <= 4.0`.
    - For `RoadRoughness::nodes()` when `spectrum == "iso8608"`: generate deterministic harmonic superposition over frequencies $n_m = n_{\min} + m \Delta n$ ($M = 64$ log-spaced or uniform modes in $[\max(1.0/\text{section\_length\_m}, 0.2), \frac{1}{2\,\text{correlation\_length\_m}}]$), phases $\phi_m = \text{rng.uniform}(0, 2\pi)$, weights $a_m = n_m^{-w/2}$, apply the same edge cosine taper `taper_len`, normalize interior RMS to `amplitude_m / 3.0`!
  - In `trackfile.cpp` & `spec_io.cpp`: parse `"step_up"` and optional `"spectrum"`, `"waviness"` on `RoadRoughness`; emit `"spectrum"` only when `!= "gaussian"` and `"waviness"` only when `!= 2.0` so existing canonical TOML & `identity_value` hashes remain 100% unchanged!
  - In `contact_safety.cpp`: add `StepUp` and `SteppedClimb` curvature checks.

- [ ] **Step 4: Build and run targeted tests to verify GREEN**
  Run: `cmake --build build/native-release --target test_terrain_generators test_terrain_presets -j8 && ctest --test-dir build/native-release -R 'terrain\.' --output-on-failure`
  Expected: PASS, 0 warnings.

- [ ] **Step 5: Commit**
  Stage explicit paths (`docs/research/2026-11-22-tracks-and-surfaces/`, `docs/superpowers/specs/2026-11-22-tracks-and-surfaces-design.md`, `docs/superpowers/plans/2026-11-22-tracks-and-surfaces.md`, and the modified `native/src/terrain/*` + `native/tests/terrain/*` files) and commit.

---

### Task 2: Stage C — Local Obstacle Micro-Surfaces (`wet_wood`, `mud_lens`, `wet_rock`) and Zero-Allocation Sub-Patch Overlay

**Model Tier:** `swe-2-max` (`sdd-terra`)

**Files:**
- Modify: `native/src/terrain/types.hpp`
- Modify: `native/src/terrain/surface.cpp`
- Modify: `native/src/terrain/obstacles.cpp`
- Modify: `native/src/terrain/trackfile.cpp`
- Modify: `native/src/terrain/spec_io.cpp`
- Modify: `native/src/config_types.hpp`
- Modify: `native/src/config_validation.hpp`
- Modify: `native/src/configuration/schema.cpp`
- Modify: `native/src/writer_config.hpp`
- Modify: `native/src/runtime/project_config.cpp`
- Modify: `native/src/writers/tire_common.hpp`
- Modify: `native/src/writers/tire_common.cpp`
- Test: `native/tests/terrain/test_surface_eval.cpp`, `native/tests/reference/test_pneumatic_energy_balance.cpp`, `native/tests/reference/test_native_input_contracts.cpp`

**Interfaces:**
- Produces:
  - `bike::terrain::available_micro_surfaces()` and `bike::terrain::get_obstacle_surface(std::string_view name)` resolving the 5 macro surfaces + 5 micro-surfaces (`dry_wood`, `wet_wood`, `polished_rock`, `wet_rock`, `mud_lens`). `available_surfaces()` and `get_surface()` keep the exact 5 macro surfaces!
  - `Obstacle` fields: `std::string surface{}; double surface_blend_m = 0.0;` and `struct MicroSpan { double start_m{}, end_m{}, blend_m{}; std::string surface{}; }; std::vector<MicroSpan> Obstacle::micro_surface_spans() const;`
  - `nativecfg::MicroSurfaceSpan { double start_m = 0., end_m = 0., blend_m = 0.; SurfaceSpec surface{}; };` and `std::vector<MicroSurfaceSpan> micro_spans{};` on `nativecfg::SurfaceMap` (omitted in `project_config.cpp` for `compliant_2d` and when `micro_spans.empty()`!).
  - `tirecommon::eval_surface` and `tirecommon::uniform_surface_for_span` evaluate `map.micro_spans` via $O(\log M)$ `std::ranges::upper_bound` with Hermite edge collars `[start_m - 0.5*blend_m, start_m + 0.5*blend_m]` and `[end_m - 0.5*blend_m, end_m + 0.5*blend_m]`, `0` heap allocations.

- [ ] **Step 1: Write failing tests**
  - In `test_surface_eval.cpp`: test all 5 new micro-surfaces via `get_obstacle_surface`, `Roots` generating individual per-bump `micro_surface_spans()` leaving inter-root gaps as `hardpack`, `Pothole` generating `[start+0.2L, start+0.8L]` `mud_lens`, `eval_surface` Hermite blending at micro-span collars, and `uniform_surface_for_span` returning `nullptr` across collars and `&span.surface` inside pure micro-span cores.
  - In `test_pneumatic_energy_balance.cpp`: test a `PneumaticTireWriter` (`fast` and `detailed`) + `DistributedTireWriter` rolling over `Roots` with `surface = "wet_wood", surface_blend_m = 0.02` on a `hardpack` track, asserting: (a) friction drops on each root and recovers in dirt gaps between roots, (b) `stored_energy(d) == elastic_energy_j()` and strict energy residual `|residual_j| <= 0.05 J`, and (c) `0` heap allocations during `try_compute_into`.

- [ ] **Step 2: Build and run tests to verify RED**
  Run: `cmake --build build/native-release --target test_surface_eval test_pneumatic_energy_balance -j8 && ctest --test-dir build/native-release -R 'surface_eval|pneumatic_energy_balance' --output-on-failure`

- [ ] **Step 3: Implement Stage C micro-surfaces end-to-end**
  - Implement micro-surface registry in `terrain/surface.cpp`, `Obstacle::micro_surface_spans()` in `obstacles.cpp`, validation in `TrackSpec::validate()` (including checking non-overlap of expanded micro-spans `s - 0.5*b .. e + 0.5*b` within `[0, length_m]` and `e - s >= b`), optional TOML/JSON serialization in `trackfile.cpp`, `spec_io.cpp`, `project_config.cpp`, `schema.cpp`, `writer_config.hpp`, `test_native_input_contracts.cpp`, and $O(\log M)$ zero-allocation evaluation in `tire_common.cpp`.

- [ ] **Step 4: Build and run targeted tests to verify GREEN**
  Run: `cmake --build build/native-release --target test_surface_eval test_terrain_generators test_terrain_presets test_native_input_contracts test_pneumatic_energy_balance test_native_distributed_tire -j8 && ctest --test-dir build/native-release -R 'terrain\.|surface_eval|native_input_contracts|pneumatic_energy_balance|native_distributed_tire' --output-on-failure`

- [ ] **Step 5: Commit**
  Stage explicit changed files and commit.

---

### Task 3: Stage B — Deformable Thin Granular Layer (`loose`, `scree`) with 1D `RutMemory` and Exact Energy Accounting

**Model Tier:** `swe-2-max` (`sdd-terra`)

**Files:**
- Create: `native/src/tyre/rut_memory.hpp`, `native/src/tyre/rut_memory.cpp`
- Modify: `CMakeLists.txt` (or target source list where `tyre/station.cpp` is compiled — verify exact CMake file first)
- Modify: `native/src/terrain/types.hpp`, `native/src/terrain/surface.cpp`, `native/src/terrain/trackfile.cpp`, `native/src/terrain/spec_io.cpp`
- Modify: `native/src/config_types.hpp`, `native/src/config_validation.hpp`, `native/src/configuration/schema.cpp`, `native/src/writer_config.hpp`, `native/src/runtime/project_config.cpp`
- Modify: `native/src/writers/pneumatic_tire.hpp`, `native/src/writers/pneumatic_tire.cpp`
- Modify: `native/src/runtime/step.cpp`
- Test: `native/tests/reference/test_pneumatic_energy_balance.cpp`, `native/tests/reference/test_native_input_contracts.cpp`

**Interfaces:**
- Produces:
  - `SurfaceSection::layer_thickness_m = 0.0` (optional in TOML/wire; when `> 0`, must lie in `[0.012, 0.150]` m and `surface` must be `"loose"` or `"scree"`, else `std::invalid_argument`; omitted when `0.0` so all Stage A tracks/fixtures stay bitwise identical).
  - `class biketyre::RutMemory`:
    - Pre-allocated 1D grid matching `RoadProfile` (`x0_m`, `dx_m`, `n_nodes`), with per-node thickness `h_s_m_[g]` interpolated smoothly across `SurfaceMap` `TransitionZone`, `z_max_m_[g]`, `z_p_m_[g]`, `z_e_m_[g]`, `soil_elastic_energy_j_`, and `step_plastic_loss_j_`.
    - $C^1$-smooth finite-depth normal pressure law (Bekker–Reece + smoothstep bedrock hardening $B_{hp}[\exp(c_{hp}(z - z_o)_+/b)-1]$) + linear unloading branch $k_u = \beta_u p'_{\text{virgin}}(z_{\max})$ (zero viscous `c * z_dot`!).
    - Slope-coupled basal-slip traction ceiling $\mu_{\text{basal,avail}}(\theta, F_N)$ on steep climbs.
    - `probe` (`advance == false`) vs `commit` (`advance == true`) separation: `probe_compute_into` takes `const RutMemory&` and mutates ZERO state; `compute_into` processes `"front"` wheel before `"rear"` wheel so rear wheel rides in front wheel's compacted rut (`z_max_m_ > 0`, lower plastic sinkage loss on second pass!), folds `soil_elastic_energy_j()` into `PneumaticTireWriter::stored_energy` / `elastic_energy_j`, and folds `step_plastic_loss_j() >= 0` into `loss_step_j`!
    - Sparse `RutStateDiff` snapshot/restore wired into `PneumaticTireWriter` `reset()` / state restore so episodes and `equilibrium` settle/restore cleanly!

- [ ] **Step 1: Write failing tests in `test_pneumatic_energy_balance.cpp`**
  Test:
  1. Validation rejects `layer_thickness_m = 0.05` on `"asphalt"` and `layer_thickness_m = 0.005` (`< 0.012`) or `0.5` (`> 0.150`) on `"scree"`.
  2. Finite-depth hardening & bedrock bound: under increasing wheel load up to `1800 N` on `scree` (`H_s = 0.04 m`), sinkage stays strictly `< H_s` with $C^1$-monotone force and `0` heap allocations.
  3. Multipass rut memory: front wheel passing over virgin `scree` (`H_s = 0.05 m`) leaves `z_p > 0`, and rear wheel following `1.23 m` behind incurs strictly less plastic compaction loss than front wheel at equal load!
  4. Probe purity: calling `probe_step_inputs` (`advance = false`) leaves `RutMemory` bitwise unchanged.
  5. Full `PhysicalStep` ride across `asphalt -> scree (H_s = 0.05 m, blend_m = 0.4) -> hardpack` holds strict energy balance `|residual_j| <= 0.05 J` on every measured step!

- [ ] **Step 2: Build and run tests to verify RED**
  Run: `cmake --build build/native-release --target test_pneumatic_energy_balance -j8 && ctest --test-dir build/native-release -R 'pneumatic_energy_balance' --output-on-failure`

- [ ] **Step 3: Implement `RutMemory` and wire into `PneumaticTireWriter`, config schema, and `step.cpp`**
  Keep `layer_thickness_m == 0.0` fast-path bypassing `RutMemory` completely so all Stage A tests remain bitwise identical.

- [ ] **Step 4: Build and run targeted tests to verify GREEN**
  Run: `cmake --build build/native-release --target test_pneumatic_energy_balance test_native_pneumatic_tire test_native_pneumatic_model test_native_input_contracts test_equilibrium_state_file -j8 && ctest --test-dir build/native-release -R 'pneumatic_energy_balance|native_pneumatic_model|native_input_contracts|equilibrium_state_file' --output-on-failure`

- [ ] **Step 5: Commit**
  Stage explicit changed files and commit.

---

### Task 4: Stage D — Rough-Terrain Rider Perception Fixes, `surface_rear` Channel, Viewer HUD, and Sensor Boundary

**Model Tier:** `swe-2-max` (`sdd-terra`)

**Files:**
- Modify: `native/src/runtime/control.hpp`, `native/src/runtime/control.cpp`
- Modify: `native/src/runtime/step.cpp`
- Modify: `native/src/rider/senses.hpp`, `native/src/rider/senses_codec.cpp`
- Modify: `native/src/rider/perception.hpp`, `native/src/rider/perception.cpp`, `native/src/rider/state_codec.cpp`
- Modify: `native/src/rider/launch_plan.cpp`
- Modify: `native/src/recording/catalog_data.inc`
- Modify: `native/src/viewer/hud.cpp`
- Test: `native/tests/reference/test_rider_senses.cpp`, `native/tests/reference/test_rider_perception.cpp`, `native/tests/reference/test_seated_posture_program.cpp`, `native/tests/reference/test_native_rider_information_boundary.cpp`, `native/tests/recording/test_catalog.cpp`

**Interfaces:**
- Produces:
  - `double runtime::road_grade_chord_preview(std::span<const RoadSample> road, double front_wheel_x_m)`: computes `(road.back().height_m - h_front) / (road.back().x_m - x_front)` where `(x_front, h_front)` is interpolated/located from `road` at `front_wheel_x_m` (falls back to `road_grade_preview(road)` when `road.back().x_m - front_wheel_x_m <= 1e-9`). `step.cpp:1290` and `step.cpp:2920` call `road_grade_chord_preview(road, wheel_x_m_[0])`. Existing `road_grade_preview(road)` stays unchanged!
  - `Perception`: latches `std::optional<double> initial_grade_rad_` on the first delivered `delayed` sample (`std::atan(delayed->senses.grade_ahead)`), and sets `out->pitch_rad = pitch - (std::atan(delayed->senses.grade_ahead) - *initial_grade_rad_)`! When `grade_ahead` is constant (`0.0` in unit tests or constant grade in `rider_launch_rides_*`), `std::atan(grade_ahead) - *initial_grade_rad_ == 0.0` **bitwise**, preserving every existing `Perception`, `FrontLiftReaction`, and constant-grade launch test! Note: serialize `initial_grade_rad_` cleanly (or derive from state if needed — check `state_codec.cpp` / `test_native_state_restore.cpp` so existing frozen `state.json` fixtures decode cleanly via optional default!).
  - `RiderSenses::surface_rear`: added as the last member `SurfaceLabel surface_rear = SurfaceLabel::hardpack;` in `senses.hpp`, populated in `step.cpp:2914` from `config_.surface_labels.at(wheel_x_m[1])`, validated in `perception.cpp`, decoded with optional fallback to `surface_front` if absent in legacy frozen `state.json` fixtures (`senses_codec.cpp`!), emitted in `step.cpp:2093` & `catalog_data.inc`, consumed in `LaunchPlan::ramp` as `const double surface = std::min(cap(p.senses.surface_rear), cap(p.senses.surface_ahead));`, and displayed in `viewer/hud.cpp`.

- [ ] **Step 1: Write failing unit tests**
  - `test_seated_posture_program.cpp`: test `road_grade_chord_preview` on a 25% ramp containing a `140 mm` sharp pothole at `x_front + 2.0 m` — `road_grade_preview` spikes to `~ -27.7`, whereas `road_grade_chord_preview` stays smoothly within `[0.15, 0.30]`, and equals `0.25` to `<= 2 ULP` on a smooth 25% slope.
  - `test_rider_perception.cpp`: test a flat-to-25%-climb transition where `pitch_rate_up_rad_s` integrates `+0.245 rad` as the road grade rises `0 -> 0.25`: `percepts->pitch_rad` returns to `~0.0 rad` (no false wheelie latch!), while a true `+0.12 rad` wheelie at constant `0.25` grade reports `+0.12 rad`.
  - `test_rider_senses.cpp` & `test_launch_plan.cpp`: verify `surface_rear` tracks `wheel_x_m[1]` (`1.23 m` behind `surface_front`) across surface transitions and `LaunchPlan::ramp` caps effort when either `surface_rear` or `surface_ahead` is `"wet"`.
  - `test_native_rider_information_boundary.cpp`: assert that research `Observation` is bitwise identical across two runs whose track differs in `surface_ahead` (`asphalt` vs `scree` 1.5 m ahead of the front wheel).

- [ ] **Step 2: Build and run tests to verify RED**
  Run: `cmake --build build/native-release --target test_seated_posture_program test_rider_perception test_rider_senses test_launch_plan test_native_rider_information_boundary test_catalog -j8 && ctest --test-dir build/native-release -R 'seated_posture_program|rider_perception|rider_senses|launch_plan|information_boundary|catalog' --output-on-failure`

- [ ] **Step 3: Implement Stage D & rough-track perception fixes**
  Carefully ensure `senses_codec.cpp` and `state_codec.cpp` accept legacy frozen episode fixtures (`pinned_savage_40_f2`) without breaking `test_native_state_restore`!

- [ ] **Step 4: Build and run targeted tests (including `state_restore` and `realistic_pedelec_acceptance`) to verify GREEN**
  Run: `cmake --build build/native-release --target test_seated_posture_program test_rider_perception test_rider_senses test_launch_plan test_launch_intent test_front_lift_reaction test_native_rider_information_boundary test_native_state_restore test_catalog test_realistic_pedelec_acceptance -j8 && ctest --test-dir build/native-release -R 'seated_posture_program|rider_perception|rider_senses|launch_plan|launch_intent|front_lift_reaction|information_boundary|state_restore|catalog|realistic_pedelec_acceptance' --output-on-failure`

- [ ] **Step 5: Commit**
  Stage explicit changed files and commit.

---

### Task 5: Calibrated Research Tracks, Fixture Refresh, `simulation-decisions.md` Update, and Stage Gate

**Model Tier:** `swe-2-flash-max` (`sdd-luna`)

**Files:**
- Create: `examples/research/rough_uphill_trail_calibrated.toml`
- Create: `examples/research/alpine_scree_deformable.toml`
- Create: `native/tests/reference/test_calibrated_surface_tracks.cpp` (registered in `native/tests/reference/Phase3Ports.cmake`)
- Modify: `docs/agents/simulation-decisions.md`
- Refresh if needed: any darwin-arm64 golden fixtures touched by `RiderSenses::surface_rear` wire addition or `road_grade_chord_preview` (verify via `bash tools/check_change.sh 1d243969`!)

**Interfaces:**
- Consumes: `StepUp`, `RoadRoughness(spectrum="iso8608")`, `Obstacle::surface` (`wet_wood`, `mud_lens`), `SurfaceSection::layer_thickness_m`, `PneumaticTireWriter` `RutMemory`, Stage D rider perception.
- Produces:
  - Two shipped research tracks exercising all new physics end-to-end on `viewer_physics_pneumatic.toml` (`rough_uphill_trail_calibrated.toml`: completes full track `end_of_track` with `|residual_j| <= 0.05 J` and `0` loop-outs; `alpine_scree_deformable.toml`: demonstrates multipass rutting & strict energy balance `|residual_j| <= 0.05 J`), plus integration test `reference.calibrated_surface_tracks`.
  - Updated `docs/agents/simulation-decisions.md` item 23 (marking Stages B, C, D implemented with commit hashes & dates) and perception notes, while leaving `examples/research/rough_uphill_savage.toml` 100% untouched!

- [ ] **Step 1: Write `examples/research/rough_uphill_trail_calibrated.toml`, `examples/research/alpine_scree_deformable.toml`, and `test_calibrated_surface_tracks.cpp`**
- [ ] **Step 2: Build and run `test_calibrated_surface_tracks` plus `bash tools/check_change.sh 1d243969`**
  Inspect any golden fixture diffs caused by `surface_rear` in `rider_senses` or `road_grade_chord_preview`, refresh only genuinely affected darwin-arm64 fixtures via the project's golden recorder tools (never weakening any physical threshold!), and re-run `bash tools/check_change.sh 1d243969` until 100% exit 0.
- [ ] **Step 3: Rebuild `build/gui` (`cmake --build build/gui --target bike-ride -j8`) and update `docs/agents/simulation-decisions.md`**
- [ ] **Step 4: Commit**
  Stage explicit changed files and commit.
