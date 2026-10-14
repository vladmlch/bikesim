# Spatial 3D — Phase 1 (Track A): Native Topology Generalisation and the Constrained Root Implementation Plan

> **For agentic workers:** Execute task-by-task with your runtime's plan-execution skill; steps use checkbox (`- [ ]`) syntax. **Prerequisites:** Phase 0 merged (`spatial-3d-p0` tag). Read `docs/agents/native-engineering.md` and ADR 0002 Layer C before Task 1. This phase is the riskiest for planar bytes: **every `tests/reference/test_native_*.py` must be bitwise green after every task.**

**Goal:** The native core and the MJCF builder accept `spatial = "3d"` with `root = "constrained"` (x, y, z, pitch; steering removed): all planar-only invariants in `native/src` are replaced by rules driven by a `topology::Spatial` descriptor, with the planar path byte-identical; a laterally flat extruded track runs end-to-end with a **temporary flat-road stub** of the 3D tyre; the planar regression bridge (ADR 0002 Layer C) passes; and lateral channels are bitwise zero on flat tracks.

**Architecture:** `topology::Spatial` (declared in P0) is computed once in `Stepper`/runtime entry from the loaded `mjModel` and passed by const reference to every writer and to the rider controller. Each planar invariant becomes a `switch (spatial.root)` with the `planar3` branch textually unchanged. New 3D branches generalise addressing (`jnt_qposadr`/`jnt_dofadr`), angular-velocity projections (axle axis instead of world Y), and the rider support/attachment Jacobians (full 3D). The constrained root is four scalar joints, so `nq == nv` still holds in this phase — the free-root (`nq = nv + 1`) branch is declared, unit-tested on a synthetic model, and left disabled until P4. The temporary tyre stub `Pneumatic3dFlatStub` evaluates the planar pneumatic carcass/brush along the wheel-plane ray fan against `Grid3D::sample` at `y = hub_y`, producing `PatchWrench` output through the P0 contract so the whole plumbing (state wire, telemetry, energy) is exercised before P3 lands.

**Tech Stack:** as master plan. Profile `native` after every task; `full` at the gate.

**Spec:** `docs/superpowers/specs/2026-10-14-spatial-3d-design.md` §4.2, §7.1, §7.3, §8.1, §9.3, §9.4. **ADR:** 0002 Layers B, C, D.

## Global Constraints

- All master-plan constraints. Branch `feat/spatial-3d-p1-native-topology`.
- **Byte-preservation proof per task:** run `bash tools/run_tests.sh native` and additionally `uv run --frozen --group native pytest tests/reference/test_native_runtime_rollout.py tests/reference/test_native_pneumatic_tire.py tests/reference/test_native_distributed_tire.py tests/reference/test_native_tire.py -x` before committing. If a task cannot keep bytes, stop and split the change behind `Spatial::planar3`.
- No new physics in this phase except the throwaway `Pneumatic3dFlatStub` (deleted in P3). The stub lives in `native/src/writers/pneumatic_3d_flat_stub.{hpp,cpp}` so its removal is a clean file deletion.
- `Grid3D` is implemented in P2. Until P2 merges, this phase implements `Grid3D::from_profile_extrusion(profile, half_width)` (part of contract 5.1): a 3-row grid (y = −W, 0, +W), all rows equal to the profile; P2 keeps it (it is the regression-bridge path).

---

## File Structure

| File | Responsibility |
|---|---|
| Modify `src/bike_sim/mujoco/frame.py:252-257` | `root = "constrained"`: add `root_y` slide; order x, y, z, pitch |
| Modify `src/bike_sim/mujoco/articulated_rider.py:45-47` | `rider_root_y` for constrained root |
| Modify `src/bike_sim/mujoco/builder.py`, `mujoco/sensors.py` | `spatial` argument; terrain hfield from 3D grid (render) or extrusion; `site_imu` + `sensor_imu_accel/gyro` added for 3D only — the existing `site_BB` sensors are untouched |
| Modify `src/bike_sim/mujoco/physical_topology.py:34-160` | Steer removal unchanged; wheel geoms contype 0 for `pneumatic_3d`; structural collision bits unchanged |
| Modify `src/bike_sim/mujoco/terrain.py:29-54` | hfield rows/cols from grid; radius_y from corridor |
| Create `src/bike_sim/terrain/extrude.py` | Planar track → laterally flat `.terrain3d` + render hfield (byte-equal along y = 0) |
| Modify `native/src/spatial_topology.hpp`, create `spatial_topology.cpp` | `topology::Spatial::from_model(const mjModel*)` |
| Modify `native/src/model_topology.hpp` | `scalar_joint` accepts qposadr/dofadr separately; `root_accessors` |
| Modify `native/src/stepper.{hpp,cpp}` | Hold `Spatial`; writer variant gains `Pneumatic3dFlatStub` |
| Modify `native/src/runtime/step.cpp:700-720, 1020-1040, 1440-1460` | Spring accounting over scalar joints by address; root position/pitch via accessors |
| Modify `native/src/writers/drivetrain.cpp:100-130`, `drivetrain/chain.cpp:185-205`, `drivetrain/transmission.cpp:60-70, 100-110` | `nq != nv` allowed when `spatial.root == free6`; axis checks body-local (unchanged text, new comment) |
| Modify `native/src/writers/brake.cpp:30-40` | same |
| Modify `native/src/writers/cruise.cpp:35-45` | root x accessor |
| Modify `native/src/writers/resistance.cpp:95-125, 310-320` | drag along body-x velocity for 3D; planar branch unchanged |
| Modify `native/src/rider/support_geometry.cpp:270-320`, `spindle_controller.cpp:200-250, 520-530`, `attachment_wrench.cpp:310-330, 575-600`, `attachment_binding.cpp` | 3D ancestor rule and 3D Jacobians behind `Spatial`; planar text unchanged |
| Modify `native/src/runtime/research_binding.cpp:150-180` | root addresses via accessors |
| Modify `native/src/runtime/samples.cpp`, `runtime/step.cpp` | Produce `root.lateral_offset_m`, `root.roll/pitch/yaw_rad` (constrained: roll = yaw = 0 exactly), `imu.*` truth fields from `sensor_imu_*` (the `frame_*` channels from `site_BB` stay byte-identical) |
| Create `native/src/writers/pneumatic_3d_flat_stub.{hpp,cpp}` | Temporary stub (deleted in P3) |
| Modify `native/src/binding.cpp`, `src/bike_sim/native/runtime.py`, `src/bike_sim/native/setup.py` | Route 3D configs; `validate_supported` now admits `spatial = "3d"` with `root = "constrained"` only |
| Create `tests/reference/test_spatial3d_topology.py` | Descriptor, addressing, synthetic free-joint model acceptance (unit level) |
| Create `tests/reference/test_spatial3d_planar_regression.py` | ADR 0002 Layer C harness (2 s in `native`, 10 s in `full`) |
| Create `tests/reference/test_spatial3d_invariants.py` | Lateral-zero bitwise test (mirror/thread tests added in P4/P7) |
| Modify `native/tests/test_contracts.cpp:1272` | The "free-joint fixture must be rejected" contract becomes "rejected unless `spatial.root == free6`" |
| Create `docs/superpowers/audits/2026-10-14-spatial-3d-p1.md` | Evidence |

---

### Task 1: `topology::Spatial::from_model` and addressing helpers

**Files:**
- Modify: `native/src/spatial_topology.hpp`; Create: `native/src/spatial_topology.cpp`; Modify: `native/src/model_topology.hpp`
- Test: `native/tests/test_contracts.cpp`, `tests/reference/test_spatial3d_topology.py`

**Interfaces:** `struct Spatial { Root root; int root_jnt_first; int root_qposadr; int root_dofadr; int steer_jnt = -1; int steer_act = -1; int imu_acc_sensor = -1; int imu_gyro_sensor = -1; bool has_lateral() const; double root_x(const mjData*) const; double root_pitch(const mjData*) const; /* planar3/constrained4: hinge value; free6: ISO pitch from quaternion */ };` `from_model` detects `root_x/root_z/root_pitch` (planar3), `root_x/root_y/root_z/root_pitch` (constrained4) or `root_free` (free6) by `mj_name2id`; anything else is rejected with the existing diagnostic style.

- [ ] **Step 1 (failing tests):** C++ contract tests on three synthetic MJCF strings (planar, constrained, free) → correct `Root`, addresses, `nq/nv` relation; a model with `root_free` **and** `root_x` is rejected. Python test loads the welded planar model and asserts `planar3` and that `root_x(d)` equals `qpos[jnt_qposadr(root_x)]` bitwise.
- [ ] **Step 2:** Implement; `Stepper` and the research runtime construct it once and store it.
- [ ] **Step 3:** `native` green; planar byte-proof list green. Commit `feat(native): topology::Spatial descriptor with planar/constrained/free root detection`.

### Task 2: Replace planar invariants behind `Spatial` (drivetrain, brake, cruise, resistance, step accounting, research binding)

**Files:** as listed in File Structure for those units.

- [ ] **Step 1:** For each site in the table, wrap the existing check in `if (spatial.root == Root::planar3) { <unchanged text> } else { <3D rule from spec §8.1> }`. Keep the planar branch character-identical (diff must show only indentation and the new `else`).
- [ ] **Step 2:** `nq != nv` checks: replace with `spatial.expected_nq() == m->nq`; scalar joint access via `jnt_qposadr`/`jnt_dofadr` everywhere (grep `qpos[` and `qvel[` in `native/src` and audit each index origin; record the audit table in the phase audit file).
- [ ] **Step 3:** Resistance: 3D branch computes aerodynamic drag along the frame body-x velocity (`mj_objectVelocity` on the frame, body frame); planar branch unchanged.
- [ ] **Step 4:** Unit tests on the synthetic constrained model: drivetrain/brake/cruise writers construct without rejection; the free-root synthetic model is still rejected by writers not yet generalised (rider) with a clear message naming P4.
- [ ] **Step 5:** `native` green; planar byte-proof green. Commit `refactor(native): planar invariants dispatched through topology::Spatial (planar path byte-identical)`.

### Task 3: Rider support geometry, attachment wrench and spindle controller in 3D

**Files:** `native/src/rider/support_geometry.cpp`, `attachment_wrench.cpp`, `attachment_binding.cpp`, `spindle_controller.cpp`
- Test: existing `test_native_rider_*.py` (bitwise), new cases in `test_spatial3d_topology.py`

**Interfaces:** `relative_planar_jacobian` keeps its name and bytes for `planar3`; new `relative_jacobian_3d(point, body_a, body_b) -> 3×nv` via `mj_jac` difference; support contact math (`_box_pad_contact_core` port) receives full 3D contact frames; the ancestor rule becomes "all rider-internal joints are hinges about body-Y; root joints are whatever `Spatial` says".

- [ ] **Step 1 (failing tests):** on the constrained synthetic model with the rider tree: controller constructs; the attachment wrench of a static pose on flat ground has `y` components exactly `0.0`; on the planar model the wrench equals the pre-change bytes: **before editing**, run the helper `tools/snapshot_wrench.py` (new, ~30 lines: fixed welded pose from the equilibrium cache of `viewer_physics_welded.toml`, dumps sha256 of the wrench array) and commit the hash as a constant in `test_spatial3d_topology.py`; after editing, the test recomputes and compares.
- [ ] **Step 2:** Implement 3D branches; keep planar text unchanged.
- [ ] **Step 3:** `native` green; all `test_native_rider_*`, `test_native_spindle_*`, `test_native_attachment_*` bitwise. Commit `feat(native): rider supports and attachment wrench generalised to 3D behind Spatial`.

### Task 4: Builder — constrained root, `spatial` argument, IMU site, hfield from grid/extrusion

**Files:** `src/bike_sim/mujoco/{frame,articulated_rider,builder,physical_topology,terrain}.py`, create `src/bike_sim/terrain/extrude.py`
- Test: `tests/test_rider_welds.py` (unchanged), new `tests/test_builder_spatial.py`; profile `quick`

- [ ] **Step 1 (failing tests):** `generate_mujoco_xml(..., spatial="planar")` output is byte-identical to the pre-change output for every `examples/research/*.toml` (store sha256 of the XML for each before the change); `spatial="3d", root="constrained"` yields joints `root_x, root_y, root_z, root_pitch`, `rider_root_x/y/z/pitch`, no `steer_joint`, the untouched `site_BB` sensors **plus** `site_imu` with `sensor_imu_accel`/`sensor_imu_gyro`, hfield rows = render `ny`.
- [ ] **Step 2:** Implement. `extrude.py`: `extrude_profile(track) -> (Terrain3dFile, render_hfield)` with `ny = 3` rows (y = −W, 0, +W) all equal to `h(x)`; test that row 1 equals `compute_ground_z/1000 + build_profile(...)` bitwise.
- [ ] **Step 3:** Commit `feat(builder): spatial 3d constrained root, IMU site, extruded terrain`.

### Task 5: `Grid3D::from_profile_extrusion` and the temporary `Pneumatic3dFlatStub`

**Files:** `native/src/terrain/grid3d.cpp` (constructor only), create `native/src/writers/pneumatic_3d_flat_stub.{hpp,cpp}`, modify `native/src/stepper.{hpp,cpp}`, `native/src/binding.cpp`, `src/bike_sim/native/setup.py`, `runtime.py`
- Test: `tests/reference/test_spatial3d_topology.py` (stub runs), profile `native`

**Interfaces:** the stub consumes `Pneumatic3dConfig` (tier ignored), reuses `native/src/pneumatic/{geometry,carcass,brush,model}` by building a `RoadProfile` view from `Grid3D` row at `y = hub_y` (nearest row) and emits one `PatchWrench` per patch with `force_n` in the x-z plane, `couple_nm = 0`, `camber_rad = 0`, `slip_angle_rad = 0`; state wire = the P0 canonical names with lateral slots NaN.

- [ ] **Step 1 (failing test):** constrained-root model + extruded `rough_uphill_savage` runs 400 steps through the research runtime without rejection; `tires.front.world_force_n.1 == 0.0` bitwise on every step; state wire round-trips.
- [ ] **Step 2:** Implement; `validate_supported` admits `spatial = "3d"` only with `root = "constrained"` (free root rejected with message "spatial-3d P4").
- [ ] **Step 3:** `native` green; planar byte-proof green (the stub is unreachable from planar configs — add a test asserting the writer variant index for planar configs is unchanged). Commit `feat(native): temporary flat-road pneumatic_3d stub and extruded Grid3D for the regression bridge`.

### Task 6: Telemetry 3D fields for the constrained root; IMU truth channels

**Files:** `native/src/runtime/samples.cpp`, `src/bike_sim/recording/catalog.py` (`produced = True` for the fields), `src/bike_sim/sim/ride/physical_runtime.py` (Python-side sample tree only if the research path still formats samples in Python — check `make_physical_view`)
- Test: `tests/reference/test_native_recording*.py` (planar unchanged), new assertions in `test_spatial3d_topology.py`

- [ ] **Step 1:** Produce `root.lateral_offset_m`, `root.roll_rad = 0.0`, `root.pitch_rad` (= `qpos[root_pitch]`, right-handed about +y, nose-down positive; `pitch_up_rad = −pitch_rad` elsewhere), `root.yaw_rad = 0.0`, `imu.specific_force.{0,1,2}`, `imu.angular_rate.{0,1,2}` from `sensor_imu_*`; planar samples unchanged (store the pre-change column count per example config — 924 for the pneumatic welded run — and assert it).
- [ ] **Step 2:** Flat extruded track: `imu.specific_force.1 == 0.0` and `imu.angular_rate.{0,2} == 0.0` bitwise; with the default mount, `imu.*` equals `frame_specific_force_body_mps2`/`frame_gyro_body_rad_s` bitwise.
- [ ] **Step 3:** Commit `feat(telemetry): root pose and IMU truth channels for spatial 3d`.

### Task 7: Planar regression bridge (ADR 0002 Layer C) and lateral-zero invariant

**Files:** create `tests/reference/test_spatial3d_planar_regression.py`, `tests/reference/test_spatial3d_invariants.py`
- Test: `native` (2 s per track), `full` (10 s per track)

**Interfaces:** `run_pair(track, physics_toml, duration) -> (planar_episode, spatial_episode)` using `viewer_physics_pneumatic.toml` for planar and the same file with `spatial = "3d"`, `[spatial_3d] root = "constrained"`, `backend = "pneumatic_3d"` for 3D; identical `RideControl` program (human torque 35 N·m, passthrough motor).

- [ ] **Step 1:** Implement the comparisons with the exact numbers from ADR 0002 Layer C: outcome and `model_status` equal; `classify_contact` per step equal with transitions within ±1 step; per-wheel normal-load RMS difference < 2 % of total weight (`mass.mass_kg × 9.81`); distance and ∫motor torque < 1 %; `WheelieTracker` metrics equal within one `persistence_s`.
- [ ] **Step 2:** Tracks: `examples/research/rough_uphill_savage.toml`, `alpine_rocky.toml`, and the research scenario `flat` (`build_research_track('flat', seed=0)`), all with `viewer_physics_pneumatic.toml` physics. Mark the 10-s variants `slow`.
- [ ] **Step 3:** `test_spatial3d_invariants.py::test_constrained_root_lateral_channels_are_bitwise_zero`: the explicit list `LATERAL_ZERO_CHANNELS = [root.lateral_offset_m, root.roll_rad, root.yaw_rad, qpos.<root_y>, qvel.<root_y>, tires.{front,rear}.world_force_n.1, tires.{front,rear}.patches.*.normal.1, tires.{front,rear}.patches.*.lateral_force_n, imu.specific_force.1, imu.angular_rate.0, imu.angular_rate.2, rider.*.force_on_bike_n.1]` is exactly `0.0` for all steps on the extruded tracks (the list lives in the test and grows with the catalog).
- [ ] **Step 4:** Run `full`. If the bridge fails, the stub or the topology refactor is wrong — do not loosen the numbers; fix and re-run. Commit `test: planar regression bridge and lateral-zero invariant for spatial 3d`.

### Task 8: Phase audit and gate

- [ ] **Step 1:** `bash tools/run_tests.sh full` green; byte-proof list green; the `qpos[`/`qvel[` audit table complete.
- [ ] **Step 2:** `docs/superpowers/audits/2026-10-14-spatial-3d-p1.md`: list of every planar invariant site and its 3D rule; bridge numbers per track; column counts; sweep results.
- [ ] **Step 3:** `docs/RIDE.md`: capability sentence "spatial 3d (constrained root) — regression configuration only; no lateral physics yet". Merge; tag `spatial-3d-p1`.
