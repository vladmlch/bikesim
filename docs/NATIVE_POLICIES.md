# Native research policies

The T34–T37 implementation constructs a research environment from native configuration, owns its motor policy, and produces immutable episode evidence through the existing physical runtime. The sources and regressions were written on 2026-10-17 under an explicit **no compilation or execution** instruction. Build, numerical parity, plugin loading, simulator behavior, and performance qualification remain pending.

The entry points are [environment.hpp](../native/src/research/environment.hpp), [policy.hpp](../native/src/research/policy.hpp), [policy_session.hpp](../native/src/research/policy_session.hpp), [lifecycle.hpp](../native/src/research/lifecycle.hpp), and [evidence.hpp](../native/src/research/evidence.hpp). The public plugin header is [bike/policy_v1.h](../native/include/bike/policy_v1.h).

## Constructing an environment

`read_research_request(path)` reads a `request_schema = 1` TOML assembly document. Relative physics, track, profile and program files resolve against their declaring document. Unknown request, experiment, sensor, rider-program and posture fields are rejected. A demand keyframe may retain extra source annotations; the native interpolator consumes `time_s` and `torque_nm`.

`prepare_environment(request)` validates the declarative inputs, resolves native terrain and optional seeded rider generation, calls the existing `make_setup`/`settle` path, performs the research-only zero-torque force probe, and seals an owned `runtime::ResearchBootstrap`. `make_environment(request)` and the path overload also create `NativeResearchRuntime`. No Python bootstrap or policy import is involved.

The physical domain follows the research source: effort drive (`crank_effort` or `articulated_effort`), no pitch assist, and track-material `compliant_2d`, `distributed_2d_reference`, or `pneumatic` tires. The native setup and physical configuration validators still apply. An authored `track_file` takes precedence over scenario generation and preserves its authored generator seed. Without one, the named native research scenario or generated/alpine specification uses the request seed.

Two defaults deliberately remain distinct:

| Input surface | Sensor sample period | Energy/telemetry measurement decimation |
|---|---:|---:|
| Typed `SensorConfig` / `ExperimentConfig` | 0.001 s | 20 |
| File assembly request using source CLI defaults | 0.005 s | 80 |

Both surfaces default to a 0.01 s control period, 0.005 s actuator delay, 3 s duration, and seed zero. Every physical/control/acquisition deadline must be an integral number of physics steps within the source alignment tolerance. Explicit zero actuator and sensor latency is supported. Capture selection remains explicit in the frontend; the typed compatibility default is `legacy`, while a no-record frontend must select `off`. An output path does not select full capture.

`async_telemetry = true` currently fails during `ExperimentConfig::validate`, before model construction or cache access, with `requires native asynchronous publication (T42)`. The native period-accounting kernel is synchronous. Its Python worker, `sim/ride/async_close.py`, belongs to downstream T42. Recording the flag without an effect or creating a second asynchronous physics implementation would violate the ownership boundary.

### Seeds, programs and reset

Seeds are nonnegative decimal integers stored without narrowing to 64 bits. The sensor tape uses the existing native NumPy-compatible `SeedSequence`/PCG64 transforms: separate spawned noise and dropout streams, with the source normal-draw order retained even when a noise scale is zero or IMU output is disabled. Startup acquires exactly one row, applies delivery latency/dropout, and starts the runtime with the cursor already at one.

Native rider generation uses the existing T19 sampler. `rider_seed_decimal` can select an independent rider seed. Rider and demand TOML documents become owned `ResearchPrograms`; the existing quintic interpolation, delayed rider reaction, discrete saddle requests and ownership checks remain authoritative. A rider behavior and a posture-owning program are exclusive.

Reset regenerates sensor delivery/noise and resets the policy with the chosen seed. It restores the same initial physical state, terrain and rider. `experiment.seed` retains the original request; `resolved_config.seed` and `actual_sensor_seed` identify the current episode. A historical captured bootstrap without raw startup sensors can repeat its existing seed but cannot synthesize a different one.

## Builtin motor policies

`load_policy(reference, configuration)` accepts explicit builtin names and `plugin:/absolute/path/to/module`. Unknown `module:factory` strings produce an actionable native-policy error; they are never imported.

| Native name | Behavior |
|---|---|
| `passthrough`, `builtin:passthrough` | Pass demand; absent demand becomes an explicit zero torque, matching the historical batch callable. |
| `zero`, `builtin:zero` | Command explicit zero motor torque. |
| `fixed_limit_40`, `builtin:fixed_limit_40` | Pass demand with a 40 Nm ceiling; absent demand becomes explicit zero. |
| `traction_control`, `builtin:traction_control` | Source encoder-slip controller; absent demand retains automatic assistance. |

Known source factory references under `bike_sim.sim.research.policies:` are also recognized: `passthrough_factory`, `zero_factory`, `fixed_limit_40_factory`, and `traction_control_factory`. The corresponding no-argument class aliases are `PassthroughPolicy`, `ZeroPolicy`, `FixedLimitPolicy`, and `TractionControlPolicy`. Factory/class passthrough and fixed-limit policies preserve absent demand as automatic assistance, matching their source classes. A source callable requiring `(observation, demand)` is not accepted as a no-argument factory.

The three simple policies accept an empty configuration object. Traction configuration accepts the following source parameters, plus optional `base` and `base_config` for an owned wrapped policy:

| Parameter | Default |
|---|---:|
| `front_radius_m` | 0.372 |
| `rear_radius_m` | 0.352 |
| `slip_target_mps` | 0.2 |
| `cut_gain_nm_s_per_mps` | 200 |
| `recover_nm_s` | 25 |
| `min_limit_nm` | 8 |
| `standstill_wheel_rad_s` | 0.3 |
| `standstill_release_s` | 0.8 |

Radii and cut/recovery rates must be finite and positive; thresholds and the minimum limit must be finite and nonnegative. Slip is derived only from delivered front/rear wheel encoders. The implementation preserves source order for elapsed time, cut/recovery and ceiling clamping. Invalid observations advance the previous time without updating the slip limit. The standstill timeout clears the explicit motor torque request, allowing automatic assistance as in the source implementation; it does not invent a separate plant actuator.

## Policy ABI v1

Modules export `const BikePolicyV1* bike_policy_v1(void)` with default visibility. The descriptor must outlive the instance and every callback. It contains ABI version 1, a descriptor size at least `sizeof(BikePolicyV1)`, zero unsupported required-capability bits, sensor schema version 1, and nonnull `create`, `reset`, `act`, and `destroy` callbacks. Observation and returned control structures use the exact v1 sizes. This ABI is for a compatible native platform/toolchain; it is not a packed file or network format.

`create` receives canonical JSON bytes and length. `reset` receives the full decimal seed and length. Neither input is promised to be NUL-terminated. `act` receives only the delivered observation and optional motor demand:

| `values` index | Sensor value | Unit |
|---:|---|---|
| 0–2 | Frame proper acceleration, body axes | m/s² |
| 3 | Pitch-up gyro rate | rad/s |
| 4 | Front wheel encoder rate | rad/s |
| 5 | Rear wheel encoder rate | rad/s |
| 6 | Crank encoder rate | rad/s |
| 7 | Measured motor torque | Nm |
| 8 | Measured human torque | Nm |

`time_s` is delivery time and `source_time_s` is acquisition time. `valid` is a canonical byte boolean. The current source sensor schema is all-or-none: `valid_mask` is `BIKE_SENSOR_ALL_FIELDS` (nine bits) for a valid row and zero otherwise. Even an invalid row carries finite, held sensor values. Simulator state pointers, contacts, road geometry, CoM, true velocity, recording data and force channels are absent from the ABI.

`BikeControlV1.present_mask` specifies which optional values are present. Absent fields are not interpreted as zero. Motor torque and motor limit must be finite and nonnegative when present. The conversion also validates canonical booleans, posture shape, known mask bits and optional-field dependencies. The motor-policy session subsequently rejects human torque, crank targets, rider posture and rider disabling: these belong to the rider behavior/program. Output brakes are separate operator inputs to the session.

Every callback must contain its exceptions and return zero on success. A failure uses a nonzero status and writes at most `BikeErrorV1.capacity` bytes into the host-owned error buffer, then sets `written`; no NUL terminator is required. The host verifies that its buffer pointer/capacity are unchanged and that `written` is bounded. It also catches callback exceptions as a defensive boundary. `destroy` must not throw.

The module owns the opaque instance and destroys it exactly once before the host unloads the library. It must not retain host observation, output, seed, JSON or error-buffer storage. An instance returned alongside a failed `create` is still cleaned up. Explicit close surfaces a destroy failure; destructors perform best-effort cleanup without replacing an active exception.

The loader requires an absolute path, resolves the `bike_policy_v1` symbol, and records SHA-256 of the module bytes and canonical configuration. It checks the module hash again after creation and at episode sealing. A wrapped plugin participates in recursive policy identity checks. Native policies are trusted code in the host process; this ABI does not sandbox a malicious or corrupted library.

[examples/policies/traction_control.cpp](../examples/policies/traction_control.cpp) is a standalone v1 module with the source traction defaults. Its configuration is deliberately restricted to an empty JSON object; the builtin supports configuration. CMake declares the optional `BIKE_BUILD_POLICY_EXAMPLES` target `bike_traction_policy` and installs the ABI header. Neither the example nor the failure-fixture modules were compiled or loaded in this delivery.

## Session, operator and failure ownership

`PolicySession` owns the environment, motor policy and optional rider behavior. It calls `act` once when accepting a new external control window. `advance_pending` may yield repeatedly at wall budgets or target steps; it keeps the same accepted command and uses the existing `NativeResearchRuntime` loop. A staged command survives a native acceptance allocation failure, so a retry does not call the policy again. A completed transition is acknowledged exactly once before queued operator actions are applied.

Pause, resume, brakes, reset and stop received during a window are queued until that window completes. `drain_active` finishes only the accepted window and never obtains another policy command. `PlaybackClock` converts wall time to completed-physics-step debt with pause/reset rebasing and scales 1, 2, 4 and 8; the debt cap does not shorten a physical control window.

The summary's `operator_events` preserves the source accepted-brake records. Requested/applied/after-terminal operator actions are a separate ordered lifecycle journal with processed step, simulation time, received wall time and episode ID. Operator intervention is latched even when optional row retention is off.

Keep a named environment owner until policy startup succeeds:

```cpp
auto environment = bike::research::make_environment(request);
auto policy = bike::research::load_policy("builtin:zero");
auto session = std::make_unique<bike::research::PolicySession>(
    std::move(environment), std::move(policy));
```

If initial policy reset or metadata collection throws, the constructor restores the named `environment` owner and rethrows the original error. The failed environment can still provide zero-step evidence and be closed. A module-load failure can be handled the same way before constructing a session. Later policy failures mark the current episode invalid; cleanup diagnostics cannot replace the primary exception.

`ResearchRun` borrows a live session or an environment and must be destroyed before its borrower. It seals before reset, retains immutable receipts across publication errors, prevents reset after finalization, and publishes at most once after a successful finish. Repeating a failed publication retries the same owned receipts, including after the engine closes. Finalization defers SIGINT through the publication boundary and then honors the interruption. Secondary recording/cleanup failures remain diagnostics alongside the original exception.

## Evidence, capture and downstream recording

`OwnedResearchEpisode` contains an immutable physical-sample index, owned requested/applied command rows, observations, transitions, final state, initial integration state, evaluation counters and metadata. Its summary remains readable after reset, close and runtime destruction. `recording_batch` adds stable stream IDs without changing the historical T10 batch schema.

Capture mode `off` retains the live accounting, validity and outcome state required for evaluation, while omitting optional trajectories and rebuild assets. `full` retains every physical row independently of measurement decimation. `legacy` retains the historical optional recorder projection. `verify` retains verification state without enabling persistent launch output. File publication requires an explicitly supplied `ResearchPublication` implementation and full capture. A null publisher retains no launch receipt/event history and creates no files.

`ResearchPublication::publish` is the integration boundary for T38–T42. This wave supplies immutable recipe/assets/summary and final-only ownership; it does not implement DuckDB, replay verification, recording manifests or the asynchronous publisher. `PhysicalLiveCsv` is a separately requested preview sink with stable first-row columns, cadence, reset/end markers and explicit close; it does not enable persistent research capture.

Metadata includes resolved configuration, terrain/model/profile hashes, compiled contact/equality solver parameters, experiment/sensor/program inputs, native source-manifest identity, executable and MuJoCo provider identity, calibration status and limitations. For an articulated rider using a non-spindle pedal attachment, `execution.allocation_solver` records `bike::rider::allocation_numeric_identity`, including dense SQP settings, and `allocation_solver_qualification` is `pending`. This is a distinct native numerical identity, with no SciPy parity claim.

Native metadata is not presented as a byte-exact Python oracle artifact. Runtime/toolchain identity differs deliberately; unavailable historical commit/dirty/locked-environment verification remains null. The source identity denotes the compiled first-party manifest; executable/provider hashes are checked separately. Referenced profile bytes and policy module bytes are rehashed when sealing. Missing or failed identity checks remain unknown or changed and cannot qualify replay.

Model applicability, numerical acceptance, calibration and replay completeness remain separate fields. Numerical energy scale uses initial energy plus external and positive source work; loss and constraint-work magnitudes cannot normalize their own defect. Model failures and the first measured rejection remain latched until reset. `valid_for_learning` requires valid model/numerics and no execution failure. Replay eligibility additionally requires full, finite, complete owned evidence, equal accepted/completed control counts, no unfinished physical suffix and unchanged identities; it is a candidacy flag, not a passed replay test. A diagnostic model violation can remain replayable while being unsuitable for learning.

## Pending qualification

Authored tests `migration_T34`–`migration_T37` cover request/program/seed/startup domains, analytic policy traces, real shared-module failure controls, queued control windows, publication retry and ownership, isolated fatal-engine/SIGINT probes, energy/status behavior, and frozen full/off evidence. CMake declares those tests and both T35 failure modules. These are authored checks, not successful results.

The next authorized validation must compile under the existing strict flags, execute the focused regressions, acquire and compare the declared source-specific oracle fixtures, and qualify same-provider numerical/bitwise behavior. The full no-record/recording, replay, Linux/installation and removal gates remain owned by their migration packages. Python source removal remains T63.
