# Native visual runtime design

Status: agreed product scope; implementation has not started.
Date: 2026-10-08.
Planning baseline: `7a12394ed22e0657dbb02b29f23f76d5f8308dd5`, branch `impl/cpp-port-p2`.
Target checkout: `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/cpp-port-p2`.

Implementation starts at [the master plan](../plans/2026-10-08-native-visual-runtime.md).
Read [native engineering rules](../../agents/native-engineering.md) before changing native code, mirrored physics, bridges, or verification tools. The accepted [native-core ADR](../../adr/0001-native-port-mujoco-core.md) remains the architectural basis.

## 1. Product contract

Python prepares terrain, MJCF, resolved configuration and static equilibrium. C++ owns the running physical state, all physics-rate work, internal rider control, accounting and research scheduling. Python retains the existing MuJoCo passive viewer and external research policy plugins.

Supported entry points:

| Entry point | Backend selection | Presentation |
|---|---|---|
| `bike-ride` | `--backend python\|native`, default `python` | Existing interactive physical ride or `--headless` |
| `bike-ride --research` | Same flag and default | Existing research window or headless experiment |
| `bike-research` | Same flag and default | Headless research |
| `bike-replay RECORDING` | Selected from recording provenance | Existing checked headless replay |
| `bike-replay RECORDING --viewer` | Selected from recording provenance | New checked visual replay |

Interactive entry points accept `--time-scale {1,2,4,8}`, default `1`. Explicit time-scale use on a headless invocation is an argument error. Backend selection does not alter resolved physical settings. Missing native artifacts and unsupported native configurations fail explicitly.

The first native release accepts the following capability set, independently of configuration filename:

- `physics_mode=physical`, `drive_mode=articulated_effort`, `rider.variant=articulated_planar`, `pitch_assist=false`.
- `articulated.pedal_attachment=spindle`, `saddle_attachment=pin`, `grip_attachment=connect`.
- `tires.backend=compliant_2d`, `surface_mode=track`, existing analytic `TireSpec` materials.
- `drive.transmission_model=ideal_mid_drive`, `motor_clutch=false`, `rotor_inertia_kgm2=0`.
- Supported configuration values include seated-climb enabled/disabled, configured assist/battery/shifting settings, numerical tuning, valid rider geometry, TOML/generated tracks and supported rider/demand programs.
- Disabling a currently supported controller feature remains supported. A topology or material change outside this table is rejected with the full configuration field path.

Support is shared by ride, research and replay. Other tire laws, non-spindle allocators, standalone C++ graphics, and cross-backend replay are separate work.

## 2. Global constraints

- Use C++23, nanobind, the existing CMake/toolchain conventions, and the pinned MuJoCo 3.12.0 native dependency group.
- Invoke Python and Python tools through `uv`.
- Keep the Python backend and 1x presentation as defaults.
- Preserve physical timestep, numerical operation order, controller periods, physical checks and existing tolerances.
- The supplied welded profile uses `timestep_s=0.00125`; all comparative measurements use that effective value.
- Keep setup/equilibrium in Python and all running physics/accounting in C++.
- Keep external `module:factory` motor policies in Python, called once per external policy boundary.
- Keep the same recorded channels and first-failure semantics. Record decimation changes publication detail, not physics/accounting coverage.
- Use one owner thread per runtime. Release the GIL only around code that touches no Python objects.
- Reuse the existing guarded engine calls, strict readers, selected-artifact validation and numerical parity infrastructure.
- Performance evidence comes from an explicit Release artifact executing the complete supported runtime.
- Preserve user-owned untracked plans, skills and output. This design does not resume unrelated unfinished safety-plan tasks.

## 3. Runtime boundary

### 3.1 Setup and reset ownership

Add a `bike_sim.native` Python package. Its setup boundary consumes an initialized Python ride or research environment and produces a versioned `RuntimeBootstrap`. This is a new format, separate from benchmark `BIKEST02`, partial `Stepper.set_state`, and the t=0-only `PhysicalInitialState`.

The bootstrap contains:

| Owned section | Contents |
|---|---|
| Compiled model | MJB saved after equilibrium and construction-time model changes; model identity digest; resolved named IDs and geometry |
| MuJoCo state | Complete `mjSTATE_INTEGRATION` vector, signature, width and dtype |
| Mutable model state | Every running coefficient changed by physical code, including brake frictionloss, tendon coefficients/ranges and any visualization marker updated by the runtime |
| Mechanical state | Existing tire, drive/transmission/policy, contact/grip and material snapshots |
| Rider state | Controller activation and held torques/terms; intent/posture/surge state; observations and contact filters |
| Runtime state | Integer clocks, incoming sensor state, balance/crash state, energy baselines/history, strictness, model status, generation and record settings |
| Research state | Present only for research: initial acquisition state, command transport, programs, experiment settings, sensor noise tape and its cursor |

Capture at t=0 after the correct frontend's initialization. Research initialization changes startup forces, warmstart and initial observations; capture it once and do not run a second research initializer over the imported state.

Construction validates and copies all inputs before publishing a runtime. A bootstrap is reusable and independent of caller array lifetimes. Reset restores the complete startup state and increments generation; it clears completed output, pending commands, clocks and playback debt. Runtime bootstrap is not an arbitrary user-facing resume/checkpoint format.

Use native typed state internally. Extend existing writer snapshots only where their present coverage is insufficient. Document each projected field's Python owner and C++ owner in the projection code.

### 3.2 Commands and advancement

The full command is the existing Python `RideControl` plus front/rear brake demands. Preserve every field, especially `None` versus numeric zero, optional posture and crank-rate target. Existing native `drivetrain::RideControl` is only a subset and is not the new public runtime command.

Public Python adapter contracts:

```text
create_native_ride(sim, *, strict: bool, record_decimation: int) -> NativeRideDriver
NativeRideDriver.advance(target_step: int, control: RideControl, *,
                         front_brake_demand: float = 0.0,
                         rear_brake_demand: float = 0.0,
                         wall_budget_s: float | None = None) -> AdvanceResult
NativeRideDriver.snapshot() -> FrameSnapshot
NativeRideDriver.drain_samples() -> SampleBatch
NativeRideDriver.flush() -> None
NativeRideDriver.reset() -> FrameSnapshot
NativeRideDriver.close() -> None
```

`target_step` is an absolute integer physics-step boundary. `AdvanceResult` carries actual step/time, a reason (`target`, `budget`, or `outcome`) and the latched outcome. Invalid input changes nothing. A wall budget is checked between completed physics steps; reaching it preserves a partial period and returns `budget`. One indivisible step may exceed the budget. Only explicit flush, physical-period close, external research boundary, terminal handling or reset closes accounting; a display frame or budget return does not.

Successful earlier steps remain committed if a later engine call fails. Preserve poisoned-context behavior and recover by rebuilding/resetting through the existing supported engine recovery path. Validation errors precede mutation; a fatal engine error is not represented as a successful transition. Completed evidence can be drained after a strict monitor rejection. Publication/boxing failure must retain an unacknowledged native batch so retry cannot lose rows.

`SampleBatch` owns column buffers and structured channel data. `as_dict_rows()` materializes the existing schema-2 rows for tests and export. Production code uses native aggregation, native recording buffers and the latest published sample; it does not box every raw interval into Python dictionaries while stepping.

### 3.3 Physical semantics that must survive the port

- Physical brakes use `StaticBrakeApplier`: `mjModel.dof_frictionloss` constraints and solved EFC forces. The existing native `BrakeWriter` implements legacy actuator braking and is not this operation.
- Match `PhysicalRuntime.apply_forces` and `_advance_physics` in order, including drivetrain preparation, forward calls, ordered component accumulation, tire/contact advancement, held controller output and transmission settlement.
- Capture incoming geometry/velocities and solved EFC, actuator, passive and sensor values before the endpoint `mj_forward`.
- Use integer physics-step clocks. Existing `interval_clock::interval_id` is a time/dt conversion helper, not a controller scheduler.
- Port spindle rider control expression-for-expression. `leg_loop_jacobian` currently uses centered finite differences with `delta_rad=1e-4` and wrapped angle differences; retain that algorithm.
- Period accounting examines every interval. Strict reference rejection occurs after the complete period has been accounted and published, while retaining the original violating interval timestamp.
- Preserve signed work, positive source work, physical loss and absolute constraint work as distinct channels. Preserve no-cancellation checks for independent constraint rows.
- Preserve the current strict research and diagnostic viewer policies. Existing physically invalid outcomes remain visible; optimization does not qualify the physical model.

## 4. Research boundary

Keep `PolicySession` lifecycle and `RideControl` / `SensorObservation` / `ResearchStep` meanings. Factory functions return a Python or native environment with this shared behavior:

- `step(control, *, front_brake_demand=0., rear_brake_demand=0.) -> ResearchStep`.
- `reset(*, seed=None)`, `save(directory, *, overwrite=False)`, and explicit `stop()`.
- Existing observation, demand, done/reason/error, validity, seed, metrics, metadata and trace access.
- A read-only simulation view supplies step, time, position, setup metadata and snapshots to frontend code. Backend-neutral consumers use these values instead of mutating `sim.physical` internals.

For incremental viewer execution add `begin_control(control, *, front_brake_demand, rear_brake_demand)` and `advance_control(*, wall_budget_s) -> ResearchStep | None`. Only one external interval is active at once. `None` means the interval is still in progress. Synchronous `step` uses these same operations until complete. A Python motor policy is invoked once before `begin_control`; yielding to render does not call it again. Operator brake/pause events are applied between external intervals, preserving the current command recording granularity.

Native research owns transport delays, rider-program interpolation, sensor acquisition/delivery, truth metrics, validity and completion. Preserve the existing environment's exact field composition: motor command fields follow its transport queue, current rider fields/program outputs are composed separately, and brake demands bypass that queue. The drive's safety-ceiling ordering within its delivered command remains unchanged.

### Deterministic sensors

Extract a shared `SensorNoiseSource` from `SensorPipeline`. Its `draw()` returns nine scaled noise components (acceleration 3, gyro 1, encoders 3, torques 2) and one dropout uniform draw. Preserve the two child streams from `SeedSequence(seed).spawn(2)` and the current draw order, including discarded IMU noise.

For native research, prepare `1 + floor(max_steps / sensor_steps)` noise rows before stepping. This includes startup and a harmless spare final acquisition. The tape is immutable, belongs to the episode and is regenerated on seed reset. C++ owns latency queues, validity, dropout decisions and cursor advancement. Exhaustion is an explicit invalid-run error, never a changed random seed. `read()` remains idempotent and consumes no randomness.

Python callbacks, including optional custom `RiderBehavior`, execute only at the same external policy boundary where the Python implementation executes them. The native loop performs no callback while the GIL is released.

## 5. Presentation boundary

Use a shared `PlaybackClock` for physical ride, research view and replay. Keep legacy ride behavior outside this physical feature.

- Allowed scale ladder: `(1, 2, 4, 8)`.
- F6 decreases, F7 increases, F8 selects 1x; clamp at ladder ends.
- Accumulate elapsed wall time multiplied by scale and retain sub-step remainder.
- Cap catch-up debt at `0.05 * scale` simulation seconds. Discard excess pacing debt, never state transitions or physical integration steps.
- Change scale, pause/resume and reset by rebasing the wall clock and clearing pacing debt. Preserve already-completed simulation work.
- Use an 8 ms compute budget before returning to events/rendering. Check completion and close events between calls.
- Render at up to 60 Hz wall time; HUD at its existing refresh cadence. Playback speed does not multiply rendering frequency.
- Display requested scale and achieved simulated-time/wall-time RTF separately. Rebase the measurement on pause/reset.

`FrameSnapshot` owns generation, step/time, integration state needed by the render replica, latest physical sample, outcome/validity and explicit view channels. The native runtime also publishes endpoint quantities required by the existing HUD/CSV, so the display does not infer physical diagnostics from replica solver results.

Build a Python render model/data replica from the same MJB. Copy a frame while the native owner is idle; recompute only replica kinematics. Keep all calculations and mutations on the authoritative runtime separate from viewer data. Model marker edits are presentation changes; existing run-time physical tuning remains fixed per run.

Refactor physical HUD formatting to consume a `PhysicalViewState` value instead of live Python writer objects. Supply the same value from both backends and compare formatted channels. Reuse the camera, physical preview CSV/HTML, livery and existing physical keyboard behavior.

On macOS call `ensure_macos_mjpython()` before model/runtime creation for every graphical entry point, including replay. Preserve re-exec arguments and the selected artifact path.

## 6. Recording and replay

New recordings use replay manifest schema 2. Schema 1 stays readable under its existing Python runtime/source requirements and is interpreted as `backend=python`.

Schema 2 adds a required `execution` object:

| Execution field | Generated value |
|---|---|
| `backend` | `python` or `native` |
| `runtime_schema` | Integer `1` |
| `python_source_sha256` | Existing Python source fingerprint |
| `native_source_sha256` | Native source/header/CMake fingerprint, null for Python |
| `extension_sha256` | Selected extension bytes digest, null for Python |
| `mujoco_version` | Installed/runtime MuJoCo version |
| `mujoco_library_sha256` | Loaded MuJoCo library bytes digest, null for Python |
| `build_context` | Measured build type, compiler ID/version and effective numerical flags; null for Python |

Python recordings use `backend=python` and null native-only fields. Paths are informational; content identity and source/runtime checks govern compatibility.

Hash native source/header/CMake inputs and the actual selected extension/MuJoCo library. Preserve current Python source/version checks. Detect source changes during recording. Bundle both referenced rider envelope and strength documents under fixed filenames; include them in the manifest's exact file/checksum set. Configuration reconstruction must reproduce their content fingerprints.

Refactor replay around `ReplaySession`: `advance(*, wall_budget_s=None)`, `snapshot()`, `restart()`, `close()`, `done`, and `report()`. It owns validation, environment reconstruction, command/observation/transition cursors and final checks. Both headless replay and the new window consume this object.

Replay uses recorded inputs without importing or calling the recorded policy factory. Compare every existing recorded observation/transition, applied-command sequence, outcome, metrics and final integration state. Preserve `atol=rtol=1e-9` for existing replay numeric comparisons. Success is emitted only after all final checks pass.

Visual replay controls: Space pause, R restart, C camera, Q/Escape exit, shared F6/F7/F8 speed. Physics-changing ride controls are unavailable. Mismatch stops advancement, retains the last consistent frame and reports the mismatching path. A user closing early receives an incomplete result, not `passed=true`. CLI exit codes: 0 verified, 1 incompatible/diverged, 2 interrupted before verification.

## 7. Artifact loading and evidence

Move reusable loader logic into `bike_sim.native.artifact`; test selectors and sanitizer instrumentation remain in the test wrapper. Application selection is `BIKE_NATIVE_BUILD_PATH` when explicitly set to an absolute directory, otherwise checkout `native/build/release`. An application process imports exactly one selected extension and rejects a conflicting pre-imported module. Python-only commands do not import the extension.

Expose artifact identity through startup diagnostics and recorded provenance. Reuse CMake manifests/static sweeps for each added translation unit. Compile-only and sanitizer checks are correctness evidence, not RTF evidence.

Acceptance:

1. Existing bitwise writer gates remain bitwise. New scalar/controller parity uses the ADR's per-call tolerance of 1e-12, with exact categorical/shape/key equality.
2. Native execution is invariant to caller chunking, display refresh and scale when controls are applied at identical simulation steps.
3. Compare Python/native complete channels and states over a fixed one-second golden horizon at 1e-9; preserve identical termination/failure identity. Longer comparisons report measured divergence separately and never relax exact same-backend replay.
4. Record/replay each backend under its own compatible artifact/runtime with existing 1e-9 checks.
5. Run required native/full verification through the repository launcher, plus targeted ASan/UBSan ownership and failure-path tests using the selected artifact.
6. On the same machine, sequential runs of the supplied profile/track at dt=0.00125 show measured native improvement and unpaced native RTF >= 1.0. Three measured runs per backend follow one warm-up; report medians and all outcomes.
7. Measure a 30 simulated-second prefix or the shared natural terminal prefix, excluding setup but including accounted stepping and final flush. If termination leaves too little timing evidence, label performance inconclusive rather than changing the physics.
8. With a real window, the supported scene sustains mean achieved RTF >= 0.95 at requested 1x. Higher scale settings report achieved speed without claiming hardware can meet every target.
9. Required gates must pass for an unqualified completion claim. Record baseline failures by test ID and original error; do not treat an unchanged red suite as green.

Planning performed source inspection only. Test examples and launch commands in the plans are implementation instructions, not executed validation evidence.
