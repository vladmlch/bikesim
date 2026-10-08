# Native Research Implementation Plan — Phase B

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Execute complete research experiments through the native physical runtime while preserving policy, sensor, recording and validity contracts.

**Architecture:** A native research context owns one Phase A runtime and the experiment scheduler. Python external policies remain at control boundaries. Typed native output is exported into the existing recording formats after computation; rendering does not change experiment chronology.

**Tech Stack:** Phase A runtime, C++23, NumPy random streams, current Python research dataclasses and pytest.

**Spec:** [Design](../specs/2026-10-08-native-visual-runtime-design.md), sections 3, 4, 6 and 7.

## Global Constraints

Apply [the master constraints](2026-10-08-native-visual-runtime.md#global-constraints). Begin after A4's accounted physical rollout gate. This phase adds no alternate tire model or non-spindle solver.

## B1. Extract deterministic noise and port sensor delivery

**Files:**

- Create `src/bike_sim/sim/research/sensor_noise.py`.
- Modify `src/bike_sim/sim/research/sensors.py`.
- Create `native/src/runtime/sensors.hpp`, `native/src/runtime/sensors.cpp`.
- Extend native research bootstrap and runtime binding.
- Create `tests/reference/test_sensor_noise.py`, `tests/reference/test_native_research_sensors.py`.

**Interfaces:**

```text
SensorNoiseSource(config: SensorConfig, *, seed: int)
SensorNoiseSource.draw() -> tuple[np.ndarray, float]
build_noise_tape(config: SensorConfig, *, seed: int, count: int) -> NoiseTape
```

`NoiseTape` is defined in sensor_noise: owned read-only float64 `noise` of shape `(count, 9)` and `dropout_uniform` of shape `(count,)`. Native sensor state has its own cursor, acquisition/delivery clocks, startup sample, delayed queue and attempted/dropped counts. Tape generation is setup work.

- [ ] **Pin random draws before refactoring SensorPipeline.**

```python
import numpy as np
from bike_sim.sim.research.sensors import SensorConfig
from bike_sim.sim.research.sensor_noise import build_noise_tape

def test_tape_matches_existing_numpy_call_order():
    cfg = SensorConfig()
    tape = build_noise_tape(cfg, seed=210, count=5)
    noise_seed, drop_seed = np.random.SeedSequence(210).spawn(2)
    noise_rng = np.random.default_rng(noise_seed)
    drop_rng = np.random.default_rng(drop_seed)
    for index in range(5):
        expected = np.concatenate((
            noise_rng.normal(0.0, cfg.acceleration_std_mps2, 3),
            np.atleast_1d(noise_rng.normal(0.0, cfg.gyro_std_rad_s)),
            noise_rng.normal(0.0, cfg.encoder_std_rad_s, 3),
            noise_rng.normal(0.0, cfg.torque_std_nm, 2),
        ))
        np.testing.assert_array_equal(tape.noise[index], expected)
        assert tape.dropout_uniform[index] == drop_rng.random()
```

This test encodes the current external RNG contract independently of the new noise helper. Add a pre-refactor golden sensor trace with fixed raw observations and compare the refactored pipeline against it.

Run:

```bash
uv run --frozen --group native python -m pytest tests/reference/test_sensor_noise.py -q
```

Red criterion: missing tape API or wrong sequence, not a native build issue.

- [ ] **Extract the source and generate immutable tapes.**

Implement draw with the exact four current normal calls and separate dropout draw. Reuse it from Python SensorPipeline. Keep original arithmetic association: acceleration raw + bias + noise; gyro raw + bias + noise; encoders/torques raw + noise. Draw discarded IMU noise before masking.

The construction loop is:

```python
source = SensorNoiseSource(config, seed=seed)
noise = np.empty((count, 9), dtype=np.float64)
dropout = np.empty(count, dtype=np.float64)
for index in range(count):
    noise[index], dropout[index] = source.draw()
noise.setflags(write=False)
dropout.setflags(write=False)
return NoiseTape(noise=noise, dropout_uniform=dropout)
```

Validate seed/count and configuration before allocation. Changing seed resets both streams; changing IMU enablement does not shift encoder/torque noise. Zero standard deviation still follows the same calls.

- [ ] **Implement native sensor push/read/reset.**

Port `SensorPipeline.push/read` timestamps, latency cutoff tolerance, stale/invalid behavior, dropout counters and idempotent reads. Consume one tape row for each attempted push, including dropped samples. Validate tape shape/finiteness/uniform domain before copying.

For a standalone native sensor test, reset(raw_initial) consumes row zero. For full research import, Python setup has already produced the initial noisy queue; import that queue and start the tape cursor at one. Do not apply noise to the initial sample twice.

- [ ] **Verify B1 and checkpoint.**

Compare Python/native traces with nonzero bias/noise, 0%/100% dropout, IMU disabled, irregular read times, latency crossing, maximum age, repeated read, out-of-order input, future input and tape exhaustion. Invalid pushes preserve cursor/queue state. Verify same-seed resets repeat and different seeds differ.

```bash
uv run --frozen --group native cmake --build native/build/release -j4
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native python -m pytest tests/reference/test_sensor_noise.py tests/reference/test_native_research_sensors.py -q
```

Completion: independent reference sequence and full sensor-delivery traces agree; no Python RNG call occurs during native advancement. Checkpoint: `feat: preserve deterministic sensors in native research`.

## B2. Implement research scheduling and policy-boundary adaptation

**Files:**

- Create `native/src/runtime/research.hpp`, `native/src/runtime/research.cpp`, `native/src/runtime/programs.hpp`, `native/src/runtime/programs.cpp`, `native/src/runtime/research_metrics.hpp`, `native/src/runtime/research_metrics.cpp`.
- Create `src/bike_sim/native/research.py`, `src/bike_sim/sim/research/backend.py`.
- Modify `src/bike_sim/sim/research/environment.py`, `src/bike_sim/sim/research/configuration.py`, `src/bike_sim/sim/research/policy_session.py` and runtime binding.
- Extend `tests/reference/_native_runtime_support.py`.
- Create `tests/reference/test_native_research_environment.py`, `tests/reference/test_native_research_policy.py`.
- Read `src/bike_sim/sim/research/environment.py`, `src/bike_sim/sim/research/rider_program.py`, `src/bike_sim/sim/research/demand.py`, `src/bike_sim/sim/research/rider_behavior.py`, `src/bike_sim/sim/research/observations.py`, `src/bike_sim/sim/research/quality.py`, `src/bike_sim/sim/research/metrics.py` and `src/bike_sim/sim/ride/wheelie.py`.

**Interfaces:**

- `create_native_research(env: ResearchEnvironment) -> NativeResearchEnvironment` captures a fresh initialized t=0 Python environment once.
- `ResearchBackend` protocol in backend defines the shared values/operations from design section 4; Python ResearchEnvironment and NativeResearchEnvironment implement it.
- `build_environment(..., backend: str = "python")` keeps current named arguments and constructs the selected implementation after shared configuration/setup validation.
- `begin_control(control, *, front_brake_demand=0., rear_brake_demand=0.)` validates/stages one external interval.
- `advance_control(*, wall_budget_s: float | None = None) -> ResearchStep | None` resumes it; a partial compute-budget return has no published external transition.
- `PolicySession.begin_advance(...)` / `advance_pending(*, wall_budget_s)` support viewer pumping. Existing `advance()` remains synchronous on top of these operations.

- [ ] **Add a paired environment fixture and regression.**

Append this helper to the shared support file:

```python
def make_python_research(*, duration=0.1, seed=210):
    from bike_sim.cli import research
    args = research.parser().parse_args([
        "--physics-config", str(PHYSICS), "--track-file", str(TRACK),
        "--duration", str(duration), "--seed", str(seed),
        "--diagnostic-model-limits", "--assist",
    ])
    return research.make_environment(args)
```

The test uses one initialized Python environment and an independent native owner:

```python
from dataclasses import asdict
import pytest
from _native_runtime_support import make_python_research, assert_tree_close
from bike_sim.native.research import create_native_research
from bike_sim.sim.ride.control import RideControl

@pytest.mark.slow
def test_native_research_matches_every_external_transition():
    reference = make_python_research()
    native = create_native_research(reference)
    assert_tree_close(asdict(native.observation), asdict(reference.observation))
    while not reference.done:
        wanted = reference.step(RideControl())
        got = native.step(RideControl())
        assert_tree_close(asdict(got), asdict(wanted))
        assert native.sim.steps == reference.sim.steps
    assert native.done and native.reason == reference.reason
    assert_tree_close(native.commands_applied, reference.commands_applied)
```

Run selected-artifact pytest on the new environment test. A runtime physics mismatch is investigated in A3/A4; do not mask it in the adapter.

- [ ] **Port the external-window scheduler and programs.**

Represent all acquisition, actuator and controller deadlines as integer physics steps validated with current integer-multiple tolerances. Native research owns one physical runtime. Apply the existing order:

```text
validate command/program ownership and both brake demands
record requested command at current external boundary
enqueue delivered motor command for current_step + actuator_delay_steps
for each physics step in this external window:
    dequeue eligible motor commands
    evaluate current rider program at current simulation time
    compose delayed motor fields with current rider fields
    record an applied-command change when the full effective control changes
    advance physical runtime once with the window's brake demands
    consume each newly completed sample exactly once
    acquire sensors, update truth/metrics/quality and evaluate stop conditions
at completed external window or terminal path:
    flush any physical tail as the current Python implementation does
    consume the flushed samples
    deliver the current sensor observation and demand
    publish one ResearchStep/trace entry
```

A compute-budget yield stays inside this transaction's active window and does not repeat request recording or sensor reads. Keep current `seen_demand_nm` semantics: the transition records what the policy saw before stepping.

Port quintic rider-keyframe interpolation, reaction delay, optional field ownership and saddle-request switching. Port demand interpolation and wheelie persistence/state transitions. Keep interval-rate torque integrals and suspension peaks even when record decimation is high.

Preserve error classification, invalid-run status, primary exception with cleanup notes, and first-failure evidence. A terminal condition evaluated from a completed period may refer to an earlier interval; match Python's actual period-consumption/stop order.

- [ ] **Adapt PolicySession and complete stop/reset behavior.**

A policy's `act` sees only SensorObservation/demand and returns RideControl. Existing motor-policy rejection of rider-owned fields remains. Optional custom RiderBehavior is called at the existing external boundary before entering native advancement.

Use this state flow:

```python
def advance(self, *, front_brake_demand=0.0, rear_brake_demand=0.0):
    self.begin_advance(front_brake_demand=front_brake_demand,
                       rear_brake_demand=rear_brake_demand)
    result = self.advance_pending(wall_budget_s=None)
    while result is None:
        result = self.advance_pending(wall_budget_s=None)
    return result
```

The begin method owns policy invocation/validation; the pending method never calls `act` again. Reject a second begin while a window is active. Queue interactive pause/brake/reset/stop requests until the active external interval completes so requested-command/transition cardinalities remain valid. Reset regenerates the tape for a new seed and imports a fresh equivalent research startup state.

Provide `env.stop()` to record operator stop without requiring consumers to mutate implementation-specific attributes. Adapt PolicySession.stop to it for both backends. For an already failed/finished episode, stop preserves the original reason.

- [ ] **Verify B2 and checkpoint.**

Test actuator delay, immediate window brakes, program-controlled rider fields, explicit null/zero commands, differing sensor/internal/external periods, decimation 1 versus 80, policy errors, unsupported policy rider control, and strict/diagnostic model outcomes.

Use a counting policy to assert one reset and exactly one act per completed/requested external window even across repeated 8 ms budget yields. Pause/resume must not advance simulated time. Reset must clear command queues, pending windows, RNG cursors, operator events and metrics.

```bash
uv run --frozen --group native cmake --build native/build/release -j4
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native python -m pytest tests/reference/test_native_research_environment.py tests/reference/test_native_research_policy.py tests/reference/test_native_research_sensors.py -q
```

Completion: every external transition and observation agrees for the supported fixture, with native physics-rate scheduling and Python callbacks only at declared boundaries. Checkpoint: `feat: execute research windows in the native runtime`.

## B3. Export recordings with backend and artifact identity

**Files:**

- Create `src/bike_sim/native/recording.py`, `native/src/runtime/recording.hpp`, `native/src/runtime/recording.cpp`.
- Modify `src/bike_sim/sim/research/environment.py`, `src/bike_sim/sim/research/replay.py`, `src/bike_sim/sim/ride/physical_session.py`, `src/bike_sim/validation/environment.py`, native artifact module and CMake metadata generation.
- Create `tests/reference/test_native_research_recording.py`, `tests/reference/test_replay_manifest_v2.py`.

**Interfaces:**

- `execution_provenance(backend: str, *, artifact=None) -> dict` in artifact, with exactly the execution fields defined by design section 6.
- `export_recording(env, directory: Path, *, overwrite: bool = False) -> Path` in recording; consumes immutable native/Python episode data and writes the current recording file set.
- `validate_recording(directory)` accepts schema 1 or 2 and verifies each schema's exact file/hash set before rebuilding.
- Native recorder stores the same decimated rows/full interval detail chosen by current PhysicalRecorder; native all-interval accounting remains independent.

- [ ] **Add recording parity and tamper regressions.**

```python
import json
import pytest
from _native_runtime_support import make_python_research
from bike_sim.native.research import create_native_research
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.replay import validate_recording

@pytest.mark.slow
def test_native_recording_identifies_backend_and_checksums(tmp_path):
    env = create_native_research(make_python_research(duration=0.02))
    while not env.done:
        env.step(RideControl())
    destination = tmp_path / "native"
    env.save(destination)
    summary, manifest = validate_recording(destination)
    assert manifest["schema_version"] == 2
    assert manifest["execution"]["backend"] == "native"
    assert manifest["execution"]["mujoco_version"] == "3.12.0"
    assert len(manifest["execution"]["extension_sha256"]) == 64
    commands = destination / "commands_requested.jsonl"
    commands.write_text(commands.read_text() + "{}\n")
    with pytest.raises(ValueError, match="checksum"):
        validate_recording(destination)
```

Add schema-1 fixture compatibility tests, Python schema-2 provenance with null native fields, strength/envelope bundle tampering and changed native artifact/source identity.

- [ ] **Implement record storage and common export.**

Store native interval columns and decimated detailed channel records without Python objects. Keep the same CSV column names/units, JSONL fields and finite-value rules. Materialize Python dictionaries only during explicit export/debug comparisons. Save initial/final integration states and per-policy observations/transitions.

Export preserves overwrite protection and output-directory semantics. Hash files after their final bytes are written, then publish replay.json last so a partial save cannot appear to be a valid recording. Capture source/artifact identity at start and compare at save; a changed source invalidates exact replay claims.

Include fixed `rider_joint_envelope.json` and `rider_joint_strength.json` files when referenced. Relocate their paths only during reconstruction; identity is content-based.

The manifest branch is:

```python
version = manifest.get("schema_version")
if version == 1:
    execution_backend = "python"
elif version == 2:
    execution_backend = manifest["execution"]["backend"]
    if execution_backend not in {"python", "native"}:
        raise ValueError("unsupported recorded execution backend")
else:
    raise ValueError("unsupported replay schema")
```

Use strict schema readers for the full execution object and exact checksum set. No importable policy reference or recorded path is executed during validation.

- [ ] **Verify B3 and the phase gate.**

Compare Python/native exported channels, requested/applied commands, observation count, transition count, summaries and metrics for the same controls. Exclude intentionally different provenance/performance fields from cross-backend numeric comparisons by explicit path, not blanket dictionary filtering.

```bash
uv run --frozen --group native cmake --build native/build/release -j4
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native python -m pytest tests/reference/test_native_research_recording.py tests/reference/test_replay_manifest_v2.py tests/reference/test_native_research_environment.py -q
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh full
```

Completion: both backends write complete schema-2 recordings and schema-1 parsing keeps its old strict contract. Checked replay execution is implemented in C3, so do not claim replay success yet. Checkpoint: `feat: record native research provenance and trajectories`. Continue with [Phase C](2026-10-08-native-visual-runtime-frontends.md).
