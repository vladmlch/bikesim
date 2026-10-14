# Spatial 3D — Phase 5 (Track E): IMU Observation v2, Research Environment and Truth/Metrics in 3D Implementation Plan

> **For agentic workers:** Execute task-by-task with your runtime's plan-execution skill; steps use checkbox (`- [ ]`) syntax. **Prerequisites:** Phase 1 merged (IMU truth channels exist; constrained root runs). Tasks 1–3 run on the constrained root; Task 4 needs Phase 4 (free root). Read `src/bike_sim/sim/research/{environment,policy_session,sensors}.py`, `src/bike_sim/native/research.py`, `native/src/runtime/research*.cpp` first.

**Goal:** The research path works in 3D end-to-end with the observation the future motor controller will see: a 6-axis IMU at a configurable mount with the existing latency/noise/bias/dropout model on six channels (schema 2), wheel/crank/torque channels unchanged; `WheelieTruth` and the episode metrics carry the 3D fields; `bike-research` runs a 3D episode on `alpine_rocky_3d` with the demo policies; the planar research path stays byte-identical (schema 1).

**Architecture:** Native already computes IMU truth from `sensordata` (P1). The noise tape (`build_noise_tape`) grows to six IMU channels for schema 2; native's `research.cpp` applies the tape per sensor sample as today, now on six values, and fills `SensorObservationV2`. `axis_misalignment_rad` is a fixed small rotation applied to the truth before noise (native). Truth and metrics gain 3D fields in both the Python mirror (`wheelie.py`, `metrics.py`) and native (`research_metrics.cpp`); `classify_contact` is unchanged. The research CLI picks the schema from `spatial`.

**Tech Stack:** Python `quick` for the research layer; `native` for `research.cpp`/`research_metrics.cpp`.

**Spec:** §5.4, §5.5, §8.4, §12 (research CLI). **ADR:** 0002 Layer D (planar bytes), B (lateral zeros on flat).

## Global Constraints

- All master-plan constraints. Branch `feat/spatial-3d-p5-imu-research`.
- Planar research path byte-identical: `tests/reference/test_native_research*.py` and the noise-tape snapshot (P0 Task 5) green at every task.
- The observation never contains truth or terrain information (only what a real controller can sense). Review every new field against this rule.
- Demo policies stay "plumbing, not anti-wheelie solutions" (docstring in `policies.py` unchanged in spirit).

---

## File Structure

| File | Responsibility |
|---|---|
| Modify `src/bike_sim/sim/research/sensors.py` | `build_noise_tape(schema=2)`; `observe_v2` formatting; `SensorConfig` 6-axis validation |
| Modify `src/bike_sim/sim/research/environment.py`, `policy_session.py`, `configuration.py` | Schema selection; observation construction; `PolicySession` passes schema |
| Modify `src/bike_sim/native/research.py` | Tape width; `SensorObservationV2` from native dict |
| Modify `native/src/runtime/research.cpp`, `research_binding.cpp`, `samples.cpp` | Six-channel tape application; misalignment rotation; observation dict v2 |
| Modify `native/src/runtime/research_metrics.cpp`, `src/bike_sim/sim/research/metrics.py`, `sim/ride/wheelie.py` | 3D truth fields → tracker metrics: `max_abs_roll_rad`, `lateral_rms_m`, `steer_torque_integral_nms`, `capsize_count`, per-wheel `lateral_slip_distance_m` |
| Modify `src/bike_sim/cli/research.py` | `--spatial`, `--terrain3d`, `--threads` (threads consumed in P7; accepted now) |
| Create `tests/reference/test_spatial3d_research.py` | Observation v2, tape, misalignment, truth/metrics, CLI episode |
| Modify `src/bike_sim/recording/catalog.py` | `produced = True` for truth fields |
| Create `docs/superpowers/audits/2026-10-14-spatial-3d-p5.md` | Evidence |

---

### Task 1: Noise tape and observation v2 (Python side)

**Files:** `sensors.py`, `environment.py`, `policy_session.py`, `configuration.py`, `native/research.py`; tests

**Interfaces:** `build_noise_tape(config, seed, count, schema)` → schema 1 unchanged bytes; schema 2 rows: `[acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, enc_front, enc_rear, enc_crank, tq_motor, tq_human, dropout_u]` with per-axis std/bias from `SensorConfig`. `observe_v2(truth_imu6, wheels, torques, tape_row, latency…) -> SensorObservationV2`.

- [ ] **Step 1 (failing tests):** schema 1 tape for seed 0 equals the P0 snapshot bitwise; schema 2 tape has the documented width; ideal config (`SensorConfig.ideal()`) yields observation equal to truth; latency/dropout semantics identical to schema 1 (same `maximum_age_s` handling, `valid` flag).
- [ ] **Step 2:** Implement; `PolicySession` exposes `observation_schema`.
- [ ] **Step 3:** Commit `feat(research): six-axis IMU noise tape and observation schema 2`.

### Task 2: Native tape application, misalignment, observation dict v2

**Files:** `native/src/runtime/research.cpp`, `research_binding.cpp`; `src/bike_sim/native/research.py`; tests in `test_spatial3d_research.py`, `native` profile

- [ ] **Step 1 (failing tests):** on the constrained root over a flat extrusion with the ideal sensor config, with the default mount, native observation `angular_rate_body_rad_s = (0, q, 0)` where `q == −pitch_rate_up_rad_s` of the schema-1 observation **bitwise** (schema 1 defines `pitch_rate_up = −frame_gyro[1]`, `observations.py:16`; schema 2 reports the raw right-handed body rate); `specific_force_body_mps2` on a stationary bike reads `(0, 0, +g)` in the mount frame; misalignment of 1° about z rotates the vector accordingly (1e-12 against NumPy); with noise, native and Python `observe_v2` agree bitwise given the same tape row (the Python formatter is the mirror of the native one — keep both and test equality, as the project does for samples).
- [ ] **Step 2:** Implement in native; Python mirror formatting in `native/research.py`.
- [ ] **Step 3:** Commit `feat(native): observation v2 with six-channel noise tape and IMU misalignment`.

### Task 3: Truth and metrics 3D (Python mirror + native)

**Files:** `wheelie.py`, `metrics.py`, `native/src/runtime/research_metrics.cpp`, `catalog.py`; tests

- [ ] **Step 1 (failing tests):** `WheelieTracker` metrics gain `max_abs_roll_rad`, `lateral_rms_m`, `capsize_count`, `front/rear_lateral_slip_distance_m`; `episode_metrics` gains `steer_torque_integral_nms`; on planar episodes all new metrics are exactly `0.0` and the existing metric dict (minus new keys) is byte-identical; native and Python metrics agree bitwise on a 2-s constrained-root 3D episode (the existing native/Python metrics parity test extended).
- [ ] **Step 2:** Implement both sides.
- [ ] **Step 3:** Commit `feat(research): 3D wheelie-truth metrics in Python and native`.

### Task 4: Research CLI in 3D and the demo policies end-to-end

**Files:** `cli/research.py`, track files, `terrain/research.py`; tests (requires P4 for the free root; constrained-root variant runs without it)

- [ ] **Step 1 (failing tests):** `bike-research --physics-config examples/research/viewer_physics_pneumatic_3d.toml --track-file examples/research/alpine_rocky_3d.toml --duration 5 --dt <P4 dt> --policy bike_sim.sim.research.policies:passthrough` produces `summary.json`, `telemetry.csv` with the 3D columns, `intervals.jsonl`; `TractionControlPolicy` runs with schema 2 (it reads encoders only); a schema-1 policy that reads `pitch_rate_up_rad_s` gets a clear `AttributeError` message pointing to schema 2 (test).
- [ ] **Step 2:** `tools/research_sweep.py` runs two seeds of the 3D track in parallel processes (smoke).
- [ ] **Step 3:** Commit `feat(cli): bike-research in spatial 3d`.

### Task 5: Phase audit and gate

- [ ] **Step 1:** `native`/`full` green; planar research bytes unchanged; observation rule review (no truth leakage) recorded.
- [ ] **Step 2:** `docs/superpowers/audits/2026-10-14-spatial-3d-p5.md`: observation v2 field table with frames and signs, tape layout, metrics table, a sample 5-s 3D episode summary.
- [ ] **Step 3:** `docs/RIDE.md` research section update. Merge; tag `spatial-3d-p5`.
