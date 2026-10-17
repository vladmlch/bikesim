# T34–T37: source delivery and static review

Date: 2026-10-17. Scope: native research construction, motor policies/ABI, session/lifecycle ownership, and owned evaluation evidence. This report records **source inspection and authored regression coverage only**. The user's instruction prohibited compilation and execution. No CMake configure, compiler, syntax-only compiler check, project script, plugin, test, simulator, oracle capture, benchmark or sanitizer was run.

The implementation plan is [2026-10-17-physics-research-policies.md](../docs/superpowers/plans/2026-10-17-physics-research-policies.md); public behavior and constraints are in [NATIVE_POLICIES.md](../docs/NATIVE_POLICIES.md). Common integration and migration status are maintained in [physics-research-review.md](physics-research-review.md) and the root-owned ledgers. These packages are not marked runtime-qualified or deletion-ready.

## Source coverage and ownership

| Cards / source responsibility | Native implementation or retained seam | Authored checks and outstanding qualification |
|---|---|---|
| T34 F047, F121, F122: native research adapter, package/backend surface | `research/environment.*`; owned `runtime/ResearchConfig`, `OwnedResearchEpisode`, `NativeResearchRuntime::configuration/evidence_snapshot/reset`; existing `research_io.*` bootstrap and stream methods. Dynamic Python attribute wrappers become typed owned records. | T34 native configuration/startup/reset checks; T37 owned status and stable recording IDs. Render frontend and persistent file export remain their downstream owners. |
| T34 F123: research configuration, terrain, environment construction | `configuration.cpp::resolve_research_physics/research_field/resolve_research_track/read_research_request`; `environment.cpp::prepare_environment`; existing model/terrain/setup owners. | T34 integer clock/domain/track-precedence and native startup assertions. Source-specific model/terrain/configuration parity not run. |
| T34 F124, F137: demand/rider programs | `program_loader.cpp` strict parsing/serialization, constant demand; existing `runtime/programs.cpp` ownership, reaction delay and interpolation. | T34 quintic midpoint, endpoint, delay, discrete saddle, ordering, ownership, malformed program and serialization cases. |
| T34 F125, F127: experiment, observations and episode start | `ExperimentConfig`, `initialization/research_startup.cpp`, existing `raw_sensor_observation`, `NativeResearchRuntime`; `ResearchTransition::valid_for_learning`. | T34 acquisition cursor one, latency/dropout/zero delay; T36 window/drain/reset; T37 transition eligibility. `async_telemetry=true` rejects before setup pending T42. |
| T34 F136: rider behavior | `RiderBehavior`, `RiderSignals`, `signals_from_sample`, `PolicySession` ownership. Only body IMU and rider support measurements enter this API. | T37 sensor/IMU sign/load projection and startup zero state. Behavior and posture-owning programs are exclusive. |
| T34 F140: sensors | `SensorConfig`, `rider_random.cpp::make_sensor_tape/startup_sensor_state`; existing `runtime/sensors.*` acquisition, latency, hold, quantization, noise cursor/state. | T34 arbitrary-precision entropy, repeat/different seed, disabled IMU draw order, startup read and dropout. Native T19 RNG is reused, not reimplemented. |
| T35 F129: source callable/class/factory policies | `builtin_policies.cpp`: passthrough, zero, fixed 40 Nm ceiling, traction with owned optional base and source missing-demand semantics. | T35 analytic cut/recovery/invalid-observation/standstill trace, aliases, constructor domains; no source numerical trace comparison executed. |
| T35 extension and T36 F130 loader | `bike/policy_v1.h`, `policy.cpp`, `plugin_policy.cpp`, standalone traction example. ABI/header sizes, sensor schema/masks, callbacks, errors, output domains, library/instance lifetime and binary/config hash identity. | T35 tests plus actual CMake MODULE fixture with create/reset/act/destroy failures and separate missing-symbol module. No library built or loaded. |
| T36 F130: policy session | `PolicySession`: one act per accepted external window; staged acceptance retry; exactly-once acknowledgement; queued operators/reset; failure and policy identity metadata. A failed initial reset restores named caller ownership. | T36 counting policy, repeated yield, target inside a long window, queue/reset/drain, zero-step load/reset failures. Host policy errors preserve physical prefixes rather than manufacturing transitions. |
| T36 F132: launch recording lifecycle | `ResearchRun`, `EpisodeReceipt`, `ResearchPublication`, `FailureReport`, `drain_active/close_environment`; owned before-reset seal, publication retry, interrupt shield, primary exception preservation. | T36 fake final publisher exercises immutable arguments and retry/idempotency. Actual DuckDB publication/replay/async remain T38–T42. Fatal engine and SIGINT checks are separate child-process test branches, authored but unexecuted. |
| T36 F191: physical session/preview/metadata | `PhysicalLiveCsv`, `configuration_metadata/physical_run_dir_name/physical_summary`; existing native setup and app/ride driver integration. | T36 CSV quoting/cadence/reset/end/idempotent close; T37 physical/research summary ownership. Interactive frontend behavior remains downstream. |
| T37 F126, F131, F141, F177: metrics, energy, model validity/status | `metrics.cpp`, `quality.cpp`, `evidence.cpp`; reused `research_quality/channel_violations/ModelStatus`, explicit `ModelStatus::reset`, empty-energy no-evaluation branch. | T37 denominator exclusions, finite/zero/electrical limits, ordered violations, sticky status/first failure, reset, full/off accounting and frozen summary behavior. Calibration remains distinct from numerical acceptance. |

T19 rider/noise primitives and T32 wheelie/contact metrics remain single shared implementations. No Python source, immutable oracle manifest or captured fixture was edited to claim parity.

## Static evidence

- Construction uses `initialization::make_setup` → `settle` → research-only safe probe → `SetupOwner::seal_bootstrap` → existing `runtime::make_research`. The ordinary ride startup branch is unchanged by the research hook. `prepare_research_startup` requires completed time-zero equilibrium, clears warm start, evaluates zero motor/human torque without advancing, performs one forward and captures one sensor observation. No second physics loop was introduced.
- `make_sensor_tape` uses native T19 child streams and transforms. `startup_sensor_state` consumes acquisition row zero once, then transfers its owned delivery queue/last-delivered state. Reset stages new tape/state/metadata before mutating the live runtime. Same-seed captured bootstraps remain usable; reseeding without raw startup data rejects explicitly.
- `PolicySession::begin_advance` calls only `Policy::act(observation, demand)` and optional rider behavior at an external command boundary. Force/accounting loops cannot call a plugin. `advance_pending` retains and acknowledges a completed transition before processing queued operations. A command staged before a native acceptance allocation error remains available without an extra policy call.
- The C ABI contains fixed sensor values and POD control/error records, not runtime/model/truth pointers or C++ containers. The host owns callback buffers; the module owns only its instance. `PluginPolicy` declares its `Library` before `Instance`, so instance destruction precedes unload. Failed creates with a returned instance, callback exceptions, malformed output and explicit-close exceptions have defined cleanup paths.
- Evidence owns values and a vector of `shared_ptr<const PhysicalSampleData>`; rebuild assets are `shared_ptr<const vector<ResearchAsset>>` containing their own bytes. Summaries read the sealed evaluation/configuration, not live runtime pointers. `recording_batch` adds stable IDs while preserving historical T10 `batch` output.
- `ResearchRun` stages both receipts and event rows before their commit. Failed publication retains the immutable receipt for explicit retry after engine close. A primary exception remains primary when seal/publication/close also fails. Finalization uses the existing platform interrupt shield and does not start a new policy interval.
- `configuration_metadata` records native source-manifest, executable, provider, configuration, terrain and profile identity. Non-spindle articulated allocation records `bike::rider::allocation_numeric_identity` and `allocation_solver_qualification = "pending"`. This identifies the new dense SQP arithmetic separately from historical SciPy behavior. Identity lookup failures leave qualification unknown; they do not invent matching provenance.
- File requests retain source CLI defaults of 5 ms sensors / 80-step measurement cadence; typed dataclass equivalents retain 1 ms / 20. Metadata is not claimed byte-identical to the captured Python environment. Missing historical commit/dirty/locked-environment status stays null.
- A static `rg` scan found no Python/nanobind runtime call, TODO/FIXME, merge marker or trailing whitespace in the new research, startup, ABI/example, T34–T37 and fixture sources at the review point. Global `::rider::` qualifications were checked against the new `bike::rider` allocation namespace. Public method signatures and CMake registrations were read; this is not compiler validation.

## Build integration inventory

The root-owned `native/cmake/NativePhysicsResearch.cmake` declares these production translation units:

```text
native/src/initialization/research_startup.cpp
native/src/research/environment.cpp
native/src/research/configuration.cpp
native/src/research/program_loader.cpp
native/src/research/rider_random.cpp
native/src/research/policy.cpp
native/src/research/builtin_policies.cpp
native/src/research/plugin_policy.cpp
native/src/research/policy_session.cpp
native/src/research/rider_behavior.cpp
native/src/research/lifecycle.cpp
native/src/research/exit_codes.cpp
native/src/research/metrics.cpp
native/src/research/quality.cpp
native/src/research/evidence.cpp
native/src/research/metadata.cpp
native/src/research/preview_csv.cpp
```

Shared edits are confined to the research startup additions in `initialization/setup_owner.hpp/.cpp`, `initialization/finalize.hpp`, the research friend declaration in `runtime/step.hpp`, owned research metadata/reset/evidence additions in `runtime/research.hpp/.cpp`, explicit reset/empty-energy handling in `runtime/status.hpp/.cpp`, and the finite positive residual-ratio guard in `runtime/research_metrics.cpp`. The two actual plant control validations in `runtime/research.cpp` pass `PhysicalStep::drive_mode()`.

Regression translation units are `test_t34.cpp`, `test_t35.cpp`, `test_t36.cpp`, and `test_t37.cpp`. `policy_fixture.cpp` is compiled into two MODULE targets, `migration_policy_fixture` and `migration_policy_missing_symbol`. T35 receives both paths through generator expressions and depends on both targets. Existing strict flags, MuJoCo identity registration and test harness remain in place. The optional standalone example is `examples/policies/traction_control.cpp`.

## Concrete remaining gates and limitations

1. **Compilation and execution are unqualified.** Strict warning, standalone-header, link/ODR, sanitizer, engine safety, exception/allocation-fault and all regression results are pending. Source-level defects may remain; nothing in this report is a passed build claim.
2. **Captured research binary inputs are incomplete in the supplied snapshot.** `research/delayed_bootstrap/research.json` references `research.arrays/a000.f64` through `a002.f64`; `state.json` references `state.arrays/a000.f64` through `a016.f64`. Those arrays and the manifest-declared `model.mjb` are absent from the extracted fixture directory. Present JSON and a repeat-seed self-comparison cannot substitute for the missing oracle bytes. No fixture was regenerated or skipped to close a gate.
3. **Asynchronous telemetry is downstream T42.** There is no native `set_async_close` worker in this snapshot. `async_telemetry=true` is a controlled pre-setup rejection, with an authored regression checking this occurs even before a valid physics/model/track exists. The supported synchronous configuration records `false` accurately.
4. **Persistent recording and replay are downstream integrations.** This wave supplies typed immutable assets, summary, extent/ID data and a final-only publication interface. DuckDB layout/transactions/manifests, replay rebuild/verification, batch/viewer and final record/no-record equivalence remain their owner packages. `replay_eligible` is a completeness candidacy check and does not mean replay passed.
5. **Numeric identity requires new qualification.** Non-spindle articulated allocation has an explicitly recorded native solver identity and pending qualification. Source settings/operation-order review does not establish SciPy or same-provider bitwise parity. Sensor, policy, trajectory, wheelie, energy and model-status oracle comparisons are still required.
6. **One referenced safety document is missing.** `docs/agents/native-engineering.md` points to `docs/superpowers/specs/2026-10-06-native-safety.md`, absent from the uploaded tree. Available common/interface/native-engineering instructions were retained; no missing gate was silently waived.

The common migration cards remain unchecked for their run-dependent gates. T63 retains exclusive ownership of Python removal.
