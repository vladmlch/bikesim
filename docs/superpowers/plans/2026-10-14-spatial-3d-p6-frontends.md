# Spatial 3D — Phase 6 (Track F): Frontends (Viewer, HUD, Replay, CLI) Implementation Plan

> **For agentic workers:** Execute task-by-task with your runtime's plan-execution skill; steps use checkbox (`- [ ]`) syntax. **Prerequisites:** Phases 4 and 5 merged. Read `docs/RIDE.md` viewer section (replica-mjData, pacing, mjcb_time whitelist), `src/bike_sim/sim/camera.py`, `sim/ride/hud.py`, `sim/ride/physical_view.py`, `sim/playback.py`, `cli/ride.py`, `cli/replay.py`.

**Goal:** The interactive viewer, `bike-ride` and checked replay work with the free-root 3D model: a filtered chase camera that follows yaw, a world-fixed side view, HUD rows for roll, steer angle, steer torque, lateral offset and capsize margin, replica frames carrying the quaternion root, 1× pacing on the fast tier, and `bike-replay --viewer` reproducing a recorded free-root episode to 1e-9.

**Architecture:** The viewer's replica model/data already receive full `mjSTATE_INTEGRATION`, so a free root needs no new wire — only the camera (`CameraManager`) and HUD (`RideHUD`) learn 3D. The camera reads yaw from the sample channel `root.yaw_rad`, not from the replica (the replica never steers anything, including the camera's notion of state — keep the one-way rule). Replay reuses `ReplaySession`; the quaternion state is part of the integration state already checked.

**Tech Stack:** Python (`quick`), manual viewer runs under `mjpython` with screenshots in the audit; `native` only if the binding changes (it should not).

**Spec:** §12. **ADR:** 0002 Layer D.

## Global Constraints

- All master-plan constraints. Branch `feat/spatial-3d-p6-frontends`.
- The replica cannot steer physics (RIDE.md:101-106) — the camera and HUD read sample channels, never write anything into the runtime.
- Planar viewer behaviour unchanged: camera presets for planar configs produce identical `lookat/azimuth/elevation/distance` sequences (snapshot test on a recorded planar episode).
- The `mjcb_time` whitelist (`native/src/stepper.cpp:170-237`) is not touched.

---

## File Structure

| File | Responsibility |
|---|---|
| Modify `src/bike_sim/sim/camera.py:23-99` | `3d` preset = filtered chase (azimuth = yaw + 135° with 0.5 s EMA, elevation −15°, distance 3.2 m); `2d` world-fixed side view with lookat following x and y; planar path unchanged |
| Modify `src/bike_sim/sim/ride/hud.py` | New segment "attitude": roll (deg), steer angle (deg), steer torque (N·m), lateral offset (m), capsize margin (deg) — only when `spatial == "3d"` |
| Modify `src/bike_sim/sim/ride/physical_view.py` | No wire change; assert free-root `qpos` width in the frame |
| Modify `src/bike_sim/cli/ride.py`, `cli/replay.py` | `--spatial`, `--terrain3d`, `--threads` flags (threads consumed by P7) |
| Modify `src/bike_sim/sim/ride/input.py`, `sim/input_handler.py` | Key `V` (86/118, currently unbound): toggle `rider_enabled` (rider intent **and** balance controller together) for demonstrations — routed through the existing control path, never through the replica. `B` stays unbound (`input.py:98-99`) |
| Create `tests/test_camera_3d.py`, `tests/test_hud_3d.py` | Unit tests on recorded samples |
| Modify `tests/reference/test_native_replay*.py` or create `tests/reference/test_spatial3d_replay.py` | Checked replay of a free-root episode to 1e-9 |
| Modify `docs/RIDE.md` | Viewer keys, 3D capability sentences, example commands |
| Create `docs/superpowers/audits/2026-10-14-spatial-3d-p6.md` | Evidence incl. screenshots |

---

### Task 1: Camera

- [ ] **Step 1 (failing tests):** `CameraManager.update_viewer` on a recorded 3D sample stream: `3d` azimuth tracks `yaw + 135°` with the EMA time constant (step response reaches 63 % in 0.5 s); `2d` lookat follows `(x, y)`; planar snapshot unchanged.
- [ ] **Step 2:** Implement; read yaw from `sample.channels['root']['yaw_rad']` when present.
- [ ] **Step 3:** Commit `feat(viewer): chase camera following yaw for spatial 3d`.

### Task 2: HUD

- [ ] **Step 1 (failing tests):** `RideHUD` renders the attitude segment with the right units and blanks when channels are absent (planar); capsize margin = `capsize_roll_rad − |roll|` in degrees.
- [ ] **Step 2:** Implement; `make_physical_view` captures the row as today.
- [ ] **Step 3:** Commit `feat(viewer): attitude HUD segment`.

### Task 3: CLI flags, key binding, interactive run

- [ ] **Step 1:** `bike-ride --physics-config examples/research/viewer_physics_pneumatic_3d.toml --track-file examples/research/alpine_rocky_3d.toml` runs under `mjpython`; `V` toggles the rider (expect capsize when off — the episode ends and the viewer shows the outcome as it does for other crashes).
- [ ] **Step 2:** Record 20 s at 1× and note mean achieved RTF (expect ≥ 1.0 on the fast tier single-threaded if P3's µs/step allows; otherwise P7 closes the gap — record either way).
- [ ] **Step 3:** Commit `feat(cli): spatial 3d flags and rider toggle key`.

### Task 4: Checked replay

- [ ] **Step 1 (failing test):** record a 5-s free-root episode (`R` export), then `bike-replay --viewer` headless mode replays with max state deviation < 1e-9 (quaternion normalisation deterministic; if a deviation appears, it is a non-determinism bug in the step — investigate, do not loosen).
- [ ] **Step 2:** Commit `test: checked replay of a free-root spatial episode`.

### Task 5: Docs, audit, gate

- [ ] **Step 1:** `docs/RIDE.md`: keys table (`V`), 3D sentences, commands; screenshots of the `3d` and `2d` presets in the audit; RTF numbers.
- [ ] **Step 2:** `quick` and `native` green. Merge; tag `spatial-3d-p6`.
