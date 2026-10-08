# Native Physical Runtime Implementation Plan — Phase A

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a supported native physical rollout with complete state ownership, rider control and accounted samples.

**Architecture:** Extend the existing Stepper/writer ownership with typed runtime operations. A new NativeRideRuntime composes those components and ports the Python physical sequencing/accounting; the Python bridge performs setup and marshaling outside the running loop.

**Tech Stack:** C++23, nanobind, MuJoCo 3.12.0, uv, pytest, existing native safety and parity tooling.

**Spec:** [Design](../specs/2026-10-08-native-visual-runtime-design.md), especially sections 2–3 and 7.

## Global Constraints

Apply the [master constraints and A0 baseline](2026-10-08-native-visual-runtime.md#global-constraints). This phase requires A0. The capability table and ownership/publication semantics in the design are authoritative.

## A1. Package the native boundary and define complete setup

**Files:**

- Create `src/bike_sim/native/__init__.py`, `src/bike_sim/native/artifact.py`, `src/bike_sim/native/config.py`, `src/bike_sim/native/schema.py`, `src/bike_sim/native/setup.py`, `src/bike_sim/native/contracts.py`.
- Modify `tools/native_config.py`, `tools/native_schema.py`, `tests/reference/native_loader.py`.
- Create `native/src/runtime/config.hpp`, `native/src/runtime/bootstrap.hpp`, `native/src/runtime/runtime_binding.hpp`, `native/src/runtime/runtime_binding.cpp`.
- Modify `native/src/stepper.hpp`, `native/src/stepper.cpp`, `native/src/binding.cpp`, `native/CMakeLists.txt` and guarded engine helpers when adding engine operations.
- Create `tests/reference/test_native_runtime_config.py`, `tests/reference/test_native_runtime_bootstrap.py`.

**Interfaces:**

- `validate_supported(cfg: SimulationPhysicsConfig, rider: RiderSpecs) -> None` in setup.
- `capture_bootstrap(sim, directory: Path) -> RuntimeBootstrap` in setup. Requires t=0; writes `directory/model.mjb` after initialization.
- `RuntimeBootstrap.model_path: Path`, `.config: dict`, `.state: dict`; the config contains `runtime_schema=1`, unchanged nested writer schema-2 config, controller/intent/monitor settings and resolved geometry.
- `load_native_extension(build_dir: Path | None = None) -> ModuleType` in artifact. Explicit argument wins; otherwise use the application's selector from the design.
- Native extension constructor `NativeRideRuntime(mjb_path: str, runtime_config: dict, bootstrap: dict)`. Initial methods: `snapshot()`, `reset()`, `close()`; stepping arrives in A3.
- `FrameSnapshot`, `AdvanceResult`, `SampleBatch` and `PhysicalViewState` are defined in contracts. Fields are owning/read-only values; the design defines their required content. Frame exposes `generation`, `step`, `time_s`, `integration_state`, `latest_sample`, `view`, `outcome`, `first_failure`. Result exposes `step`, `time_s`, `reason`, `outcome`. Batch exposes `as_dict_rows()` and `interval_ids`.

- [ ] **Add the support-matrix regression before the bridge.**

```python
from dataclasses import replace
import pytest
from bike_sim.physics.resolution import load_physics_config
from bike_sim.physics.rider import RiderSpecs
from bike_sim.native.setup import validate_supported

def test_native_capability_uses_fields_not_filename():
    cfg = load_physics_config("examples/research/viewer_physics_welded.toml")
    rider = RiderSpecs(variant="articulated_planar")
    validate_supported(cfg, rider)
    different_attachment = replace(
        cfg, articulated=replace(cfg.articulated, pedal_attachment="weld"))
    with pytest.raises(ValueError, match=r"articulated.pedal_attachment"):
        validate_supported(different_attachment, rider)
```

Add parameterized cases for every structural predicate in design section 1, valid scalar tuning, disabled seated-climb/battery/shifting, explicit zero/null command values and wrong native dependency version. Assert rejection precedes output creation and extension construction.

Run:

```bash
uv run --frozen --group native python -m pytest tests/reference/test_native_runtime_config.py -q
```

Red criterion: the new boundary is missing or accepts an unsupported structural setting; unrelated equilibrium failure is not the red condition.

- [ ] **Move shared code and add the strict runtime envelope.**

Move writer projection/schema implementations into the installed package. Keep tool imports as explicit forwarding shims so existing tests/tools use the same implementation. Extract the loader's artifact-path/import verification into the package; retain `NATIVE_TEST_BUILD_PATH`, legacy test-selector conflict handling, sanitizer verification and reporter behavior in the test wrapper.

The capability validator implements this checked map before projection:

```python
checks = {
    "physics_mode": (cfg.physics_mode, "physical"),
    "drive_mode": (cfg.drive_mode, "articulated_effort"),
    "rider.variant": (rider.variant, "articulated_planar"),
    "pitch_assist": (cfg.pitch_assist, False),
    "articulated.pedal_attachment": (cfg.articulated.pedal_attachment, "spindle"),
    "articulated.saddle_attachment": (cfg.articulated.saddle_attachment, "pin"),
    "articulated.grip_attachment": (cfg.articulated.grip_attachment, "connect"),
    "tires.backend": (cfg.tires.backend, "compliant_2d"),
    "tires.surface_mode": (cfg.tires.surface_mode, "track"),
    "drive.transmission_model": (cfg.drive.transmission_model, "ideal_mid_drive"),
    "drive.motor_clutch": (cfg.drive.motor_clutch, False),
    "drive.rotor_inertia_kgm2": (cfg.drive.rotor_inertia_kgm2, 0.0),
}
for field, (actual, supported) in checks.items():
    if actual != supported:
        raise ValueError(f"native runtime: {field}={actual!r}; requires {supported!r}")
```

Use existing dataclass/readers for numeric domains and separately require analytic `TireSpec` material. Resolve envelopes/strength profiles at setup and send their numerical tables and provenance. The nested writer schema retains its existing version; the new runtime envelope has its own version.

Completion: application imports do not depend on tests/tools, Python-only CLI imports no native extension, and existing loader/config tests remain green.

- [ ] **Implement bootstrap staging and ownership.**

Use `mj_saveModel` after setup and `mj_getState(..., mjSTATE_INTEGRATION)` for data. The baseline model includes current tendon/friction coefficients. Project the complete state inventory from design section 3.1, with explicit field readers on both sides. Use the existing state exporters where available; add a missing-state exporter beside its Python owner instead of serializing arbitrary `__dict__`.

Native runtime contains an owning Stepper and typed controller/accounting members. Add typed borrowed accessors/typed configuration constructors to Stepper where required by composition; every borrowed writer refers to that Stepper's own model/data. Use existing `drive()` and staged writer APIs. Do not create two authoritative tire/drive instances for one model.

Stage decoded bootstrap values, validate model identity/shapes/domains, then restore model coefficients, mechanical snapshots and full integration state in a defined order. Recompute only derived caches that Python setup also recomputes; do not add a settling step or consume a running clock.

Add this ownership regression:

```python
import numpy as np
import pytest
from native_loader import load_native
from _native_runtime_support import make_python_ride
from bike_sim.native.setup import capture_bootstrap

@pytest.mark.slow
def test_bootstrap_owns_input_and_restores_complete_start(tmp_path):
    sim = make_python_ride()
    setup = capture_bootstrap(sim, tmp_path)
    native = load_native().NativeRideRuntime(
        str(setup.model_path), setup.config, setup.state)
    before = native.snapshot()
    setup.state["integration_state"][:] = 0.0
    sim.data.qpos[:] = sim.data.qpos + 0.01
    np.testing.assert_array_equal(native.snapshot().integration_state,
                                  before.integration_state)
    native.reset()
    after = native.snapshot()
    np.testing.assert_array_equal(after.integration_state, before.integration_state)
    assert after.step == 0
    assert after.generation == before.generation + 1
```

Use owning float64 arrays in the Python bootstrap; validate original values before numeric conversion. Also test dimensions/model mismatch, malformed nested material state, caller-object destruction, conflicting selected artifacts and reset twice. Defer first-force equivalence to A3; record that dependency instead of declaring the whole bootstrap behavior qualified here.

- [ ] **Verify A1 and checkpoint.**

```bash
uv run --frozen --group native cmake --build native/build/release -j4
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native python -m pytest tests/reference/test_native_runtime_config.py tests/reference/test_native_runtime_bootstrap.py tests/reference/test_native_loader.py tests/reference/test_native_state_restore.py -q
```

Register every added translation unit in the existing CMake targets/manifests and sweep inputs. Run IDEA inspections on modified source. Completion: strict configuration/ownership tests pass; existing Stepper loading and state contracts remain unchanged. Checkpoint: `feat: define owned native runtime setup boundary`.

## A2. Port physical braking, spindle control and automatic intent

**Files:**

- Create `native/src/runtime/static_brake.hpp`, `native/src/runtime/static_brake.cpp`.
- Create `native/src/rider/spindle_math.hpp`, `native/src/rider/spindle_math.cpp`, `native/src/rider/spindle_controller.hpp`, `native/src/rider/spindle_controller.cpp`, `native/src/rider/intent.hpp`, `native/src/rider/intent.cpp`.
- Extend runtime config/bootstrap/binding and CMake sources.
- Create `tests/reference/test_native_static_brake.py`, `tests/reference/test_native_spindle_controller.py`, `tests/reference/test_native_rider_intent.py`.
- Read Python `src/bike_sim/sim/ride/static_braking.py`, `src/bike_sim/sim/ride/leg_loop.py`, `src/bike_sim/sim/ride/crank_split.py`, `src/bike_sim/sim/ride/rider_control.py`, `src/bike_sim/sim/ride/rider_intent.py`, `src/bike_sim/physics/joint_strength.py`, `src/bike_sim/physics/rider_envelope.py`, `src/bike_sim/physics/rider_program.py`, `src/bike_sim/physics/seated_climb.py`.

**Interfaces:**

- `StaticBrake`: validated front/rear DOF IDs and ceiling; apply demands to model frictionloss; capture solved front/rear generalized forces.
- `SpindleController`: initialization, pure/probe computation, advancing computation, held outputs, state/restore, envelope forces and solved-effort diagnostics.
- `RiderIntent`: resolve the full runtime control using integer step, sensor inputs, road window and advance/probe mode; state/restore/reset.
- Narrow binding probes `spindle_torque_waveform(mean_nm, phase_rad, ripple)` and `crank_effort_ceiling(power_w, torque_limit_nm, crank_rate_rad_s)` are production kernels exposed for parity.
- A native-only test adapter binds controller/intent state probes, keeping its interfaces separate from the application adapter.

- [ ] **Add independent per-call regressions.**

```python
import math
import pytest
from native_loader import load_native
from bike_sim.sim.ride.rider_control import pedal_torque_waveform
from bike_sim.physics.seated_climb import crank_effort_ceiling

@pytest.mark.parametrize("phase", [0.0, math.pi / 2, math.pi, 2 * math.pi - 1e-7])
def test_spindle_waveform_matches_python(phase):
    native = load_native()
    actual = native.spindle_torque_waveform(60.0, phase, 0.5)
    assert actual == pytest.approx(
        pedal_torque_waveform(60.0, phase, 0.5), rel=1e-12, abs=1e-12)

@pytest.mark.parametrize("rate", [-3.0, 0.0, 8.0, 20.0])
def test_effort_ceiling_matches_python(rate):
    assert load_native().crank_effort_ceiling(250.0, 60.0, rate) == pytest.approx(
        crank_effort_ceiling(250.0, 60.0, rate), rel=1e-12, abs=1e-12)
```

Add static-brake comparisons against Python on a stationary supported model, including a nonzero externally loaded wheel, both demand bounds and independent front/rear rows. Assert model frictionloss and solved `mjCNSTR_FRICTION_DOF` work, not a requested signed torque.

Run new tests against the selected artifact and observe absent kernels/incorrect braking behavior.

- [ ] **Implement pure primitives and physical brake law.**

Reuse existing support geometry and Accelerate-backed algebra. Preserve the finite-difference leg Jacobian, branch unwraps, directional torque split, curve interpolation, strength/speed/power limits and current floating-point expression order.

Physical brake core:

```cpp
// front and rear are validated, distinct wheel DOFs in this owning model.
// Validate both demands before either write.
model.dof_frictionloss[front] = ceiling_nm * std::clamp(front_demand, 0.0, 1.0);
model.dof_frictionloss[rear] = ceiling_nm * std::clamp(rear_demand, 0.0, 1.0);
```

Use the repository's checked model-access spans in the actual implementation. Capture solved forces through the existing guarded `mj_mulJacTVec` boundary and selected EFC friction rows. The legacy BrakeWriter remains dedicated to legacy semantics.

- [ ] **Implement the stateful spindle branch and intent clock.**

Port the spindle branch of `ArticulatedRiderController.compute` and the helpers it reaches, including upper-body PD/bias compensation, recovery preload, active-state evolution, anatomical envelope forces and held command-term diagnostics. Resolve names and profile tables once.

The intent scheduler follows:

```cpp
if (enabled && active && step % period_steps == 0 && step != last_tick_step) {
    const auto expected = last_tick_step < 0 ? 0 : last_tick_step + period_steps;
    require(step == expected, "rider intention clock skipped an acquisition");
    intent = policy.update(signals, period_s, road_grade, preview_grade, lean_limit);
    last_tick_step = step;
}
if (rider_enabled) {
    if (!control.posture.has_value()) control.posture = intent.posture;
    if (!control.human_torque_nm.has_value()) control.human_torque_nm = intent.effort_ceiling_nm;
}
```

Translate `require` into the existing validation/status convention; the snippet fixes scheduling, not a new generic assertion library. Probe calls work on staged state and preserve clocks, surge budget and activation.

Port posture-program rate limits, delay queues, inclination filter, surge spend/recovery and finite-budget boundaries. Optional scheduled intent pulses retain reset semantics.

- [ ] **Verify complete controller behavior and checkpoint.**

Use the existing `test_pinned_topology._compiled` fixture for a cheap nine-actuator model, and actual captured states for coupled cases. Compare all nine actuator requests and diagnostics; verify free-root forces are absent. Test phase branch cuts, reversed crank rates, zero effort, explicit override versus automatic effort, controller/intent periods that differ, probe non-mutation, saturation, surge exhaustion/recovery and reset.

```bash
uv run --frozen --group native cmake --build native/build/release -j4
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native python -m pytest tests/reference/test_native_static_brake.py tests/reference/test_native_spindle_controller.py tests/reference/test_native_rider_intent.py tests/reference/test_leg_loop.py tests/reference/test_crank_split.py tests/reference/test_joint_strength.py -q
```

Completion: per-call numeric/categorical contracts pass at 1e-12 for the supported branch; all stateful parity scenarios pass. Checkpoint: `feat: port spindle rider control and physical braking`.

## A3. Assemble the owned physical step and bounded advancement

**Files:**

- Create `native/src/runtime/ride_runtime.hpp`, `native/src/runtime/ride_runtime.cpp`, `native/src/runtime/step.hpp`, `native/src/runtime/step.cpp`, `native/src/runtime/control.hpp`.
- Create `src/bike_sim/native/runtime.py`.
- Extend runtime binding/bootstrap and Stepper typed accessors.
- Create `tests/reference/test_native_runtime_step.py`, `tests/reference/test_native_runtime_commands.py`.
- Read `src/bike_sim/sim/ride/physical_runtime.py:apply_forces/_advance_physics`, `src/bike_sim/sim/ride/control_clock.py`, `src/bike_sim/sim/ride/contact_filter.py`, `src/bike_sim/sim/ride/balance_monitor.py`, `src/bike_sim/sim/ride/physical_crash.py`, `src/bike_sim/sim/ride/force_accumulator.py`.

**Interfaces:** implement the design's `create_native_ride`, `advance`, `snapshot`, reset and close. Add `probe_step_inputs(control, *, front_brake_demand, rear_brake_demand)` to the test adapter, returning ordered components, actuator input, model brake bounds and staged diagnostics without advancement. A4 supplies completed accounted rows.

- [ ] **Add first-step and chunking regressions.**

```python
import numpy as np
import pytest
from _native_runtime_support import make_python_ride
from bike_sim.native.runtime import create_native_ride
from bike_sim.sim.ride.control import RideControl

@pytest.mark.slow
def test_physics_result_is_independent_of_call_chunks():
    sim = make_python_ride()
    one = create_native_ride(sim, strict=False, record_decimation=1)
    chunks = create_native_ride(sim, strict=False, record_decimation=1)
    command = RideControl()
    one.advance(40, command)
    for target in (1, 3, 9, 17, 40):
        chunks.advance(target, command)
    a, b = one.snapshot(), chunks.snapshot()
    assert a.step == b.step == 40
    np.testing.assert_array_equal(a.integration_state, b.integration_state)

@pytest.mark.slow
def test_native_first_step_uses_settled_material_state():
    sim = make_python_ride()
    native = create_native_ride(sim, strict=False, record_decimation=1)
    command = RideControl()
    sim.step(control=command)
    native.advance(1, command)
    import mujoco
    expected = np.empty(mujoco.mj_stateSize(sim.model, mujoco.mjtState.mjSTATE_INTEGRATION))
    mujoco.mj_getState(sim.model, sim.data, expected, mujoco.mjtState.mjSTATE_INTEGRATION)
    np.testing.assert_allclose(native.snapshot().integration_state, expected,
                               atol=1e-9, rtol=1e-9)
```

Red criterion: `advance` is absent or first-step state differs; repeated bare `mj_step` passing on an unforced toy model is insufficient.

- [ ] **Implement one authoritative physical step.**

Port in this order, as private functions on the owning runtime:

```text
validate/resolve current command and internal intent tick
capture incoming control/filter state
apply static brake bounds; prepare pedaling and drivetrain state
mj_forward; clear/fold ordered force components
compute tire material/contact forces and external resistance
compute rider contact forces and drivetrain sensing/components
apply anatomical envelope; compute or hold rider actuator command
publish accumulated forces; incoming-state mj_forward
capture incoming q/v, attachment geometry and raw work inputs
guarded mj_step
capture solved EFC, actuator/passive forces and sensor channels
settle transmission and solved contact/drive state
endpoint mj_forward and endpoint kinematics
append RawStep for A4 and update integer step/control clocks
```

Match each forward/capture site to its Python source before changing order. Include rollback brake intent, grounded filters, balance/crash checks and exact named force-fold order. Use per-context scratch with construction-sized buffers where the existing kernels permit it. Existing coefficient-changing ticks may allocate; retain that distinction in RTSan/allocation assertions.

Use full command fields; reject unknown fields before state changes. Restrict externally injected generalized-force arrays to internal tests until an existing frontend requests them; if ported for the oracle, validate width/ownership and keep them out of the user CLI.

- [ ] **Implement bounded advancement and safe nanobind boundaries.**

The loop advances one committed step at a time until target, terminal outcome, strict error or budget. Check the wall clock between steps. Rendering yields do not flush accounting. Holding a command across calls is explicit at the adapter, not a hidden latching change to physical stepping.

Binding flow:

```text
GIL held: parse/stage Python arguments and validate public domains
GIL released: execute owned native advancement, preserving status/evidence
GIL held: translate status and construct immutable return objects
```

Retain callback-policy refresh before entering the loop. Prevent concurrent/reentrant advance/reset on the same context. Do not touch Python refs while GIL is released. Preserve existing native fatal-engine exception handling; completed earlier steps are a committed prefix, not rolled back.

- [ ] **Verify A3 and checkpoint.**

Add a control schedule with brake application/release, explicit zero versus automatic rider effort, motor ceiling changes, gear shifts and coast/resume. Compare native forces against recorded Python inputs at each transition. Test budget yields, invalid target/command, call reentrancy, reset after advance and process-isolated engine failure.

```bash
uv run --frozen --group native cmake --build native/build/release -j4
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native python -m pytest tests/reference/test_native_runtime_step.py tests/reference/test_native_runtime_commands.py tests/reference/test_native_runtime_bootstrap.py tests/reference/test_native_engine_errors.py -q
```

Completion: first-force/first-step bootstrap equivalence and deterministic chunking pass; no Python step/controller callback appears inside native advancement. Checkpoint: `feat: assemble native physical ride stepping`.

## A4. Account every interval and publish complete native samples

**Files:**

- Create `native/src/runtime/period_buffer.hpp`, `native/src/runtime/period_buffer.cpp`, `native/src/runtime/accounting.hpp`, `native/src/runtime/accounting.cpp`, `native/src/runtime/samples.hpp`, `native/src/runtime/samples.cpp`, `native/src/runtime/status.hpp`, `native/src/runtime/status.cpp`.
- Extend ride runtime, binding, contracts and runtime adapter.
- Create `tests/reference/test_native_runtime_accounting.py`, `tests/reference/test_native_runtime_rollout.py`, `tests/reference/test_native_runtime_ownership.py`.
- Read `src/bike_sim/sim/ride/period_buffer.py`, `src/bike_sim/sim/ride/physical_energy.py`, `src/bike_sim/sim/ride/physical_observations.py`, `src/bike_sim/sim/ride/physical_samples.py`, `src/bike_sim/sim/ride/physical_recorder.py`, `src/bike_sim/sim/ride/reference_monitor.py`, `src/bike_sim/sim/ride/model_status.py`, `src/bike_sim/physics/energy_ledger.py` and the complete `PhysicalRuntime._close_period`.

**Interfaces:** `flush()`, `drain_samples() -> SampleBatch`, latest owned sample/view, first failure/model status and recorded native columns. Scalar diagnostic export uses existing schema-2 `PhysicalSample.as_dict()` keys/units.

- [ ] **Add complete-row and flush regressions.**

```python
import pytest
from _native_runtime_support import make_python_ride, advance_python, assert_tree_close
from bike_sim.native.runtime import create_native_ride
from bike_sim.sim.ride.control import RideControl

@pytest.mark.slow
def test_native_full_rows_match_python_for_non_aligned_tail():
    sim = make_python_ride()
    native = create_native_ride(sim, strict=False, record_decimation=1)
    command = RideControl()
    expected = advance_python(sim, 41, command)
    native.advance(41, command)
    native.flush()
    actual = native.drain_samples().as_dict_rows()
    assert len(actual) == len(expected) == 41
    assert [row["interval_id"] for row in actual] == list(range(41))
    for got, wanted in zip(actual, expected):
        assert_tree_close(got, wanted)
    assert native.drain_samples().as_dict_rows() == []
```

Add explicit flush after a partial period followed by more steps. Compare it against Python with the identical explicit flush schedule; compare display/budget-only chunking without inserting flushes. This distinction matters for full-detail publication and period-level integral checks.

- [ ] **Implement native period evaluation and work history.**

Represent incoming RawStep fields with owned native storage. Reuse native equality reaction and attachment-wrench primitives with the current observability/rank checks. Port effort reconstruction and EFC work partitions; keep all thresholds from Python.

At period close:

```text
evaluate every raw interval's attachment/effort evidence
accumulate signed/positive source work and absolute/signed constraint work
evaluate the period-closing integral criterion
build all mandatory channels and requested detailed channels
update model status/history for each interval
publish the complete period and its ordered first-failure evidence
apply strict ReferenceMonitor rejection using the originating interval time
```

A latest-frame sample is an owned snapshot, not a view of reusable scratch. Use acknowledgement-based batch extraction: box/export before removing pending rows. Native research/ride recorders consume native columns, not round trips through per-interval Python dictionaries.

- [ ] **Implement independent failure and ownership tests.**

Port meaningful scenarios from `test_period_buffer.py` and `test_energy_ledger.py`: inter-tick strength/power violation, independent opposing constraint work, zero-source absolute budget, nonaligned flush, record decimation, missing/duplicated interval protection and strict versus diagnostic behavior.

Add source-state lifetime tests, retained snapshots across reset/close, bootstrap alias mutation, failed batch boxing then retry, and engine-failure committed-prefix evidence. Run fatal probes in subprocesses. Compare a one-second 800-step golden episode with all channels and first-failure identity; use diagnostic mode consistently on both sides where the current physical model reports a violation.

- [ ] **Run the phase gate and checkpoint.**

```bash
uv run --frozen --group native cmake --build native/build/release -j4
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native python -m pytest tests/reference/test_native_runtime_accounting.py tests/reference/test_native_runtime_rollout.py tests/reference/test_native_runtime_ownership.py tests/reference/test_period_buffer.py tests/reference/test_energy_ledger.py -q
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh full
```

Run the targeted ownership/fatal-path suite against the selected ASan/UBSan build using the repository's documented loader/runtime recipe; verify the actual loaded artifact/runtime. Update the audit with full output identities and any baseline failures.

Completion: full physical rollout, all-row accounting, strict failure semantics, state ownership and required phase verification are evidenced. No research/viewer claim is made yet. Checkpoint: `feat: account and publish native physical intervals`. Continue with [Phase B](2026-10-08-native-visual-runtime-research.md).
