# Native Research (Track B) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Supply B1-B3 source changes without executing project code or compiling.

**Architecture:** An owned C++ research scheduler wraps the existing physical runtime. NumPy produces an immutable setup-time noise tape; all interval scheduling, sensor delivery, program interpolation, truth, metrics and recording storage run in C++. Python adapts external policy boundaries and exports recordings.

**Tech Stack:** C++23, nanobind, existing MuJoCo runtime, Python/NumPy setup and pytest regression sources.

**Spec:** `docs/superpowers/specs/2026-10-08-native-visual-runtime-design.md`, sections 3, 4, 6, 7; original B1-B3 plan `2026-10-08-native-visual-runtime-research.md`.

## Global Constraints

- User explicitly prohibits execution and compilation. No builds, imports, tests, lint, static analyzers, benchmarks or application runs. Read/edit/package files only.
- A4 verification is outstanding in the supplied archive; source work may proceed, but the A4/B completion gates cannot be claimed.
- Preserve full optional RideControl fields, integer deadlines, external-boundary callbacks, incoming-state samples, binary64 association and explicit invalid-run outcomes.
- Do not port NumPy RNG approximately. Use the supplied NumPy sources as the reference for setup-time draw order and interpolation.
- No source Git repository, tools directory, root packaging files or prior Python reference tests were supplied. Do not invent their execution history.

---

### Task 1: Deterministic sensor source and native delivery

**Files:** create `sensor_noise.py`, `runtime/sensors.{hpp,cpp}`, sensor binding and regression sources; modify `sensors.py`.

**Interfaces:** `SensorNoiseSource.draw() -> (noise[9], uniform)`; `build_noise_tape(config, *, seed, count)`; native `SensorPipeline.reset/push/read/import_state`.

- [x] Pin independent NumPy call-order and sensor delivery regression sources, including zero deviations, disabled IMU, all-dropout, invalid timestamps and exhausted tapes.
- [x] Extract four unchanged normal calls and separate uniform stream; export owned startup state after row zero.
- [x] Implement staged native push/read and validated import. Invalid inputs preserve queue, clocks and cursor.
- [ ] Execution gate deferred: `tests/reference/test_sensor_noise.py` and `test_native_research_sensors.py` must pass on the selected artifact before B1 is verified.

### Task 2: Owned research windows, programs and metrics

**Files:** create `runtime/{programs,research_metrics,research,recording}.{hpp,cpp}`, research binding, `native/research.py`, `sim/research/backend.py`; modify runtime private stepping seam, CMake, configuration, environment and policy session.

**Interfaces:** `begin_control(control, brakes)` stages one interval; `advance_control(wall_budget_s)` returns no transition on a budget yield; `step` synchronously completes that same transaction.

- [x] Reuse the exact physical one-interval primitive, not a second integrator or Python loop.
- [x] Keep actuator transport, immediate brakes, rider quintic program and demand association, sensor deadlines, wheelie hysteresis, energy checks and all-interval torque/peak metrics native.
- [x] Retain owned decimated samples/columns; expose boundary snapshots and explicit export only.
- [x] Adapt external policy lifecycle, queued operator stop/reset and fresh-seed setup. Preserve primary exceptions and invalid-run evidence.
- [x] Add paired Python/native environment and counting-policy regression sources.
- [ ] Execution gate deferred: complete one-second 1e-9 comparison, failure contracts and sanitizers must pass.

### Task 3: Common export and strict schema-2 provenance

**Files:** create `native/recording.py`; modify artifact metadata, CMake build identity, replay validation, physical summary adapter; add recording/schema regression sources.

**Interfaces:** `execution_provenance(backend, artifact=None)` emits exactly design section-6 fields; `export_recording(env, directory, overwrite=False)` writes fixed files and publishes `replay.json` last.

- [x] Capture source/extension/loaded-engine content identity at startup and compare it at save; native build context comes from compiled metadata, not guessed flags.
- [x] Preserve schema-1 fixed file set; schema 2 covers telemetry, intervals, trace, episode metrics, states, commands, observations and referenced envelope/strength bundles.
- [x] Reject malformed execution records, path substitution, missing/extra checksum fields and changed bundle content. Validation never imports recorded policies.
- [x] Add tamper and compatibility regression sources. Exact checked native replay/viewer integration remains C3, outside Track B.
- [ ] Execution gate deferred: export parity, schema compatibility and tamper regressions must pass.

### Handoff

- [x] Inspect changed source.
- [x] Package a clean project plus a changes-only patch.
- [x] Record source-reference paths, implementation scope, missing upstream files and every unexecuted gate in the delivery audit. No red/green or numerical/performance claim.
