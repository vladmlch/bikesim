# mujoco_clean Stage 5 — `Rfinal` Adjudication

Inspected revision: `0502660dd14bd1021b3da90a61e20dcb1d5805a1`

## Adjudication Rule

Final compliance = most severe cited verdict across R1 (independent research) and R2 (adversarial audit); a violation backed by `file:line` evidence stands unless directly refuted. Justification maps the strictest product-relevance call onto {relevant, redundant, contrary_to_confirmed_goals, unresolved}.

## Property Verdicts

| prop | title | compliance_verdict | justification_verdict | r1 | r2 |
|------|-------|--------------------|-----------------------|----|----|
| P01a | Lossless Drivetrain & Unilateral Clutch Invariant | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P01b | Autonomous Stall-Reposition Motor-Clutch Config Guard | violated | relevant | partially_violated/mixed_subclauses | violated/relevant_engineering_contract |
| P01c | `Transmission` Tendon-Range Boundary Coherence | partially_violated | relevant | partially_violated/mixed_subclauses | holds/relevant_engineering_contract |
| P02a | Cross-Boundary State-Space Coherence (Runtime, `set_state`, Wire Codec) | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P02b | Bounded Crank-Reposition Termination & Post-State | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/relevant_engineering_contract |
| P03a | Pedaling Effort Slew & Coasting Zero-Effort Contract | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/relevant_confirmed_requirement |
| P03b | Cadence Hysteresis Coasting Semantics (Product Decision) | partially_violated | unresolved | partially_violated/mixed_subclauses | partially_violated/unresolved_product_decision |
| P03c | Motor Assist Brake Reset, Power-Capped Run-On & Snapshot Relational Invariant | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P04a | Shifting Snapshot & Cut-Order Invariant | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P04b | Unified Torque-Cut Active Predicate | violated | relevant | partially_violated/mixed_subclauses | violated/mixed_subclauses |
| P04c | Launch Stationary/Upshift Protocol | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/relevant_engineering_contract |
| P05a | Plan-Disabled Phase Quiescence | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/relevant_confirmed_requirement |
| P05b | Launch/Front-Lift/Intent/Posture Restore Sentinel & Bounds Coherence | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P05c | Automatic Rider-Command Domain & Clamp Bounds | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/relevant_confirmed_requirement |
| P05d | Bounded Launch & Front-Lift Termination | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/relevant_engineering_contract |
| P06a | Rider Unilateral Support, Spindle Anti-Windup & Zero Joint Damping | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P06b | `PedalRecoveryState` Closed Enum & Escape-Direction Integrity | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/relevant_engineering_contract |
| P06c | Cohesive Two-Hand Spring-Grip Release (Product Decision) | violated | unresolved | partially_violated/mixed_subclauses | violated/unresolved_product_decision |
| P07a | Observational Purity of Non-Advancing Probe Evaluation | violated | relevant | violated/mixed_subclauses | violated/mixed_subclauses |
| P07b | Hill-Hold Rollback-Brake Parity on Active Probes | violated | relevant | violated/mixed_subclauses | violated/relevant_engineering_contract |
| P08a | `engine::invoke` TLS Error Containment | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P08b | `Stepper` Fail-Closed Poisoning Boundary | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/relevant_engineering_contract |
| P08c | `BIKE_NONBLOCKING` Realtime Status Path | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/relevant_engineering_contract |
| P08d | `ControlClock` Alignment & Hold Gate | partially_violated | relevant | partially_violated/mixed_subclauses | holds/relevant_engineering_contract |
| P09 | `NativeRideRuntime` Single-Owner Exclusion, Committed-Prefix Preservation & Balance/Brake/Crash Monitors | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P10a | `ModelStatus` Verdict Lattice & Wire/Accounting Hardening | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P10b | `ReferenceMonitor` Episode-Scoped Warn-Once & First-Failure Latch | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/relevant_confirmed_requirement |
| P10c | `PeriodAccounting` Rollback & `AsyncCloseRunner` Recovery Semantics | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P10d | `SensorPipeline` Cursor & Delivery-Time Contract | partially_violated | relevant | partially_violated/mixed_subclauses | holds/relevant_confirmed_requirement |
| P11 | `NativeResearchRuntime` Control-Window Staging, Retry Idempotency & Exclusive Episode Outcome Machine | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P12a | `measure_run` Whole-Boundary Protocol & Termination Bound | partially_violated | relevant | holds/relevant_confirmed_requirement | partially_violated/relevant_engineering_contract |
| P12b | Benchmark Valid-Prefix Isolation & Monitor-into-Model-Status Merge | holds | relevant | holds/relevant_confirmed_requirement | holds/relevant_confirmed_requirement |
| P12c | Installed-Provider Non-Rescue & Provenance Gate | partially_violated | relevant | holds/relevant_confirmed_requirement | partially_violated/mixed_subclauses |
| P13 | `SetupOwner` Multi-Stage Bootstrap Lifecycle, Setup-Only Crank Hold & Equilibrium Cache Atomicity | partially_violated | relevant | partially_violated/relevant_engineering_contract | partially_violated/mixed_subclauses |
| P14a | Single `act` Per Accepted Control Window | partially_violated | relevant | partially_violated/mixed_subclauses | holds/relevant_confirmed_requirement |
| P14b | Throwing `apply` Consumes Command & Releases Transition | violated | relevant | partially_violated/mixed_subclauses | violated/relevant_engineering_contract |
| P14c | Post-Terminal Reset Journaled With Success Truth | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P14d | `PlaybackClock` Wiring Status (Dead-Code Disposition) | violated | relevant | partially_violated/mixed_subclauses | violated/relevant_engineering_contract |
| P15 | `ResearchRun` Multi-Episode Finalization, `PluginPolicy` ABI v1 RAII Order, `TractionControl` & `PhysicalLiveCsv` Schema Safety | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P16 | `RunCapture`, `ResearchRecording` & DuckDB Atomic Sealed-Container Publication and Post-Seal Immutability | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P17a | Deferred-SIGINT Shield Mechanics & Scope Registry | partially_violated | relevant | partially_violated/relevant_engineering_contract | partially_violated/relevant_engineering_contract |
| P17b | Exit-130 on Deferred Interrupt at Seal/Publication Boundaries | violated | relevant | partially_violated/relevant_engineering_contract | violated/relevant_confirmed_requirement |
| P17c | `operator_stop_deferred` Journaling Boundary | violated | relevant | partially_violated/relevant_engineering_contract | violated/relevant_confirmed_requirement |
| P18 | `ReplaySession` Checked-Replay Certificate Soundness, Mismatch Halt & Command-0 Verified-Frame Rollback | partially_violated | relevant | partially_violated/relevant_engineering_contract | partially_violated/relevant_engineering_contract |
| P19 | Viewer Replica Isolation, Monotone `(generation, step)` Presentation, Pacing & GPU Destruction Order | partially_violated | relevant | holds/relevant_confirmed_requirement | partially_violated/relevant_confirmed_requirement |
| P20a | Batch Worker-Pool & Subprocess Group Lifecycle | partially_violated | relevant | partially_violated/relevant_engineering_contract | partially_violated/mixed_subclauses |
| P20b | CLI Cross-Mode Validation & Jobs Grammar | violated | relevant | partially_violated/relevant_engineering_contract | violated/relevant_engineering_contract |
| P20c | Evidence Fingerprint & Source-Immutability Gate Scope | partially_violated | relevant | partially_violated/relevant_engineering_contract | partially_violated/relevant_engineering_contract |
| P21a | `Sha256` Single-Finish Protocol | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/relevant_engineering_contract |
| P21b | Lossless IO Decoders & FD Ownership Boundary | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P21c | `SeedSequence`/`NumpyRng` Spawn Counter Integrity | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/relevant_confirmed_requirement |
| P21d | `slsqp`/Least-Squares/Linear-Feasibility Status Soundness | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/relevant_engineering_contract |
| P22 | Configuration Mode Gates, Integer-Tick Quantization, Two-Phase Model Builders & Terrain/Kinematics Invariants | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P23 | Tyre Carcass/Bristle Lifecycles, Shared Writer Clock/Restore Protocol, `CruiseWriter` & GPU Batch Parity | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/mixed_subclauses |
| P24a | Qualification Sweep Verdict Semantics & Summary Publication | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/relevant_engineering_contract |
| P24b | Container/`link_deps` Lifecycle & Harness Self-Test Wiring | partially_violated | relevant | partially_violated/mixed_subclauses | partially_violated/relevant_engineering_contract |

## Evidence Citations

- `P01a`: `native/src/model/drivetrain.cpp:15-58`; `native/src/model/physical_topology.cpp:12-14`; `native/src/drivetrain/motor_clutch.cpp:8-16`; `native/src/drivetrain/transmission.hpp:29-289`; `native/src/drivetrain/transmission.cpp:68-147`; `native/src/writers/drivetrain.hpp:442-533`
- `P01b`: `native/src/model/drivetrain.cpp:15-58`; `native/src/model/physical_topology.cpp:12-14`; `native/src/drivetrain/motor_clutch.cpp:8-16`; `native/src/drivetrain/transmission.hpp:29-289`; `native/src/drivetrain/transmission.cpp:68-147`; `native/src/writers/drivetrain.hpp:442-533`
- `P01c`: `native/src/model/drivetrain.cpp:15-58`; `native/src/model/physical_topology.cpp:12-14`; `native/src/drivetrain/motor_clutch.cpp:8-16`; `native/src/drivetrain/transmission.hpp:29-289`; `native/src/drivetrain/transmission.cpp:68-147`; `native/src/writers/drivetrain.hpp:442-533`
- `P02a`: `native/src/drivetrain/stall_detector.hpp:13-17`; `native/src/config_types.hpp:66-74`; `native/src/config_types.hpp:76-82`; `native/src/drivetrain/pedaling.hpp:126`; `native/src/drivetrain/policy_set.hpp:14-18`; `native/src/writers/drivetrain.hpp:62-78`
- `P02b`: `native/src/drivetrain/stall_detector.hpp:13-17`; `native/src/config_types.hpp:66-74`; `native/src/config_types.hpp:76-82`; `native/src/drivetrain/pedaling.hpp:126`; `native/src/drivetrain/policy_set.hpp:14-18`; `native/src/writers/drivetrain.hpp:62-78`
- `P03a`: `native/src/config_types.hpp:51-59`; `native/src/drivetrain/pedaling.hpp:126`; `native/src/drivetrain/policy_set.hpp:61`; `native/src/writers/drivetrain.hpp:442`; `native/src/config_types.hpp:97-105`; `native/src/drivetrain/motor.hpp:99`
- `P03b`: `native/src/config_types.hpp:51-59`; `native/src/drivetrain/pedaling.hpp:126`; `native/src/drivetrain/policy_set.hpp:61`; `native/src/writers/drivetrain.hpp:442`; `native/src/config_types.hpp:97-105`; `native/src/drivetrain/motor.hpp:99`
- `P03c`: `native/src/config_types.hpp:51-59`; `native/src/drivetrain/pedaling.hpp:126`; `native/src/drivetrain/policy_set.hpp:61`; `native/src/writers/drivetrain.hpp:442`; `native/src/config_types.hpp:97-105`; `native/src/drivetrain/motor.hpp:99`
- `P04a`: `native/src/config_types.hpp:84-93`; `native/src/drivetrain/shifting.hpp:179`; `native/src/drivetrain/policy_set.hpp:63`; `native/src/writers/drivetrain.hpp:502`; `native/src/writers/drivetrain.cpp:274-278`; `native/src/rider/launch_plan.hpp:40-53`
- `P04b`: `native/src/config_types.hpp:84-93`; `native/src/drivetrain/shifting.hpp:179`; `native/src/drivetrain/policy_set.hpp:63`; `native/src/writers/drivetrain.hpp:502`; `native/src/writers/drivetrain.cpp:274-278`; `native/src/rider/launch_plan.hpp:40-53`
- `P04c`: `native/src/config_types.hpp:84-93`; `native/src/drivetrain/shifting.hpp:179`; `native/src/drivetrain/policy_set.hpp:63`; `native/src/writers/drivetrain.hpp:502`; `native/src/writers/drivetrain.cpp:274-278`; `native/src/rider/launch_plan.hpp:40-53`
- `P05a`: `CMakeLists.txt:116-175`; `native/src/rider/launch_plan.hpp:51-106`; `native/src/rider/front_lift_reaction.hpp:48-112`; `native/src/rider/perception.hpp:42-83`; `native/src/rider/intent.hpp:132-182`; `native/src/rider/intent.hpp:265-333`
- `P05b`: `CMakeLists.txt:116-175`; `native/src/rider/launch_plan.hpp:51-106`; `native/src/rider/front_lift_reaction.hpp:48-112`; `native/src/rider/perception.hpp:42-83`; `native/src/rider/intent.hpp:132-182`; `native/src/rider/intent.hpp:265-333`
- `P05c`: `CMakeLists.txt:116-175`; `native/src/rider/launch_plan.hpp:51-106`; `native/src/rider/front_lift_reaction.hpp:48-112`; `native/src/rider/perception.hpp:42-83`; `native/src/rider/intent.hpp:132-182`; `native/src/rider/intent.hpp:265-333`
- `P05d`: `CMakeLists.txt:116-175`; `native/src/rider/launch_plan.hpp:51-106`; `native/src/rider/front_lift_reaction.hpp:48-112`; `native/src/rider/perception.hpp:42-83`; `native/src/rider/intent.hpp:132-182`; `native/src/rider/intent.hpp:265-333`
- `P06a`: `native/src/rider/mode_search.cpp:9-34`; `native/src/contact/laws.hpp:28-38`; `native/src/rider/spindle_controller.hpp:489`; `native/src/rider/spindle_controller.cpp:1388`; `native/src/configuration/physical_config.hpp:437`; `native/src/rider/state_codec.cpp:774`
- `P06b`: `native/src/rider/mode_search.cpp:9-34`; `native/src/contact/laws.hpp:28-38`; `native/src/rider/spindle_controller.hpp:489`; `native/src/rider/spindle_controller.cpp:1388`; `native/src/configuration/physical_config.hpp:437`; `native/src/rider/state_codec.cpp:774`
- `P06c`: `native/src/rider/mode_search.cpp:9-34`; `native/src/contact/laws.hpp:28-38`; `native/src/rider/spindle_controller.hpp:489`; `native/src/rider/spindle_controller.cpp:1388`; `native/src/configuration/physical_config.hpp:437`; `native/src/rider/state_codec.cpp:774`
- `P07a`: `native/src/runtime/runtime.cpp:816-828`; `native/src/runtime/step.cpp:2953-3012`; `native/src/cli/ride.cpp:120-152`; `native/src/viewer/ride.cpp:170-189`; `native/src/replay/session.cpp:213-253`; `native/src/initialization/research_startup.cpp:50-66`
- `P07b`: `native/src/runtime/runtime.cpp:816-828`; `native/src/runtime/step.cpp:2953-3012`; `native/src/cli/ride.cpp:120-152`; `native/src/viewer/ride.cpp:170-189`; `native/src/replay/session.cpp:213-253`; `native/src/initialization/research_startup.cpp:50-66`
- `P08a`: `native/src/engine_call.cpp:78`; `native/src/engine_call.cpp:66-74`; `native/src/engine_abi_312.hpp:15-17`; `native/src/engine_call.cpp:398-434`; `native/src/engine_call.cpp:483-776`; `native/src/writers/rider_contacts.cpp:1330-1344`
- `P08b`: `native/src/engine_call.cpp:78`; `native/src/engine_call.cpp:66-74`; `native/src/engine_abi_312.hpp:15-17`; `native/src/engine_call.cpp:398-434`; `native/src/engine_call.cpp:483-776`; `native/src/writers/rider_contacts.cpp:1330-1344`
- `P08c`: `native/src/engine_call.cpp:78`; `native/src/engine_call.cpp:66-74`; `native/src/engine_abi_312.hpp:15-17`; `native/src/engine_call.cpp:398-434`; `native/src/engine_call.cpp:483-776`; `native/src/writers/rider_contacts.cpp:1330-1344`
- `P08d`: `native/src/engine_call.cpp:78`; `native/src/engine_call.cpp:66-74`; `native/src/engine_abi_312.hpp:15-17`; `native/src/engine_call.cpp:398-434`; `native/src/engine_call.cpp:483-776`; `native/src/writers/rider_contacts.cpp:1330-1344`
- `P09`: `native/src/runtime/runtime.hpp:236`; `native/src/runtime/runtime.cpp:74-86`; `native/src/validation/runtime_access.cpp:12-17`; `native/src/runtime/research.cpp:14-28`; `native/src/runtime/runtime.cpp:74-90`; `native/src/validation/runtime_access.cpp:99-104`
- `P10a`: `native/src/runtime/status.hpp:41-59`; `native/src/runtime/status.hpp:42-49`; `native/src/runtime/status.cpp:186-219`; `native/src/runtime/accounting.cpp:739-740`; `native/src/runtime/accounting.cpp:436`; `native/src/runtime/runtime.cpp:580-584`
- `P10b`: `native/src/runtime/status.hpp:41-59`; `native/src/runtime/status.hpp:42-49`; `native/src/runtime/status.cpp:186-219`; `native/src/runtime/accounting.cpp:739-740`; `native/src/runtime/accounting.cpp:436`; `native/src/runtime/runtime.cpp:580-584`
- `P10c`: `native/src/runtime/status.hpp:41-59`; `native/src/runtime/status.hpp:42-49`; `native/src/runtime/status.cpp:186-219`; `native/src/runtime/accounting.cpp:739-740`; `native/src/runtime/accounting.cpp:436`; `native/src/runtime/runtime.cpp:580-584`
- `P10d`: `native/src/runtime/status.hpp:41-59`; `native/src/runtime/status.hpp:42-49`; `native/src/runtime/status.cpp:186-219`; `native/src/runtime/accounting.cpp:739-740`; `native/src/runtime/accounting.cpp:436`; `native/src/runtime/runtime.cpp:580-584`
- `P11`: `native/tests/research/test_bootstrap.cpp:523-556`; `native/tests/reference/test_native_viewer_snapshot.cpp:183-219`; `docs/RECORDING.md:72-75`; `docs/RECORDING.md:74-75`
- `P12a`: `CMakeLists.txt:239-284`; `docs/superpowers/plans/2026-11-16-audit-fixes.md:169`; `CMakeLists.txt:327-338`; `CMakeLists.txt:282-285`; `CMakeLists.txt:610-618`; `native/src/runtime/realtime_measurement.cpp:159-165`
- `P12b`: `CMakeLists.txt:239-284`; `docs/superpowers/plans/2026-11-16-audit-fixes.md:169`; `CMakeLists.txt:327-338`; `CMakeLists.txt:282-285`; `CMakeLists.txt:610-618`
- `P12c`: `CMakeLists.txt:239-284`; `docs/superpowers/plans/2026-11-16-audit-fixes.md:169`; `CMakeLists.txt:327-338`; `CMakeLists.txt:282-285`; `CMakeLists.txt:610-618`; `native/src/runtime/provider_images.cpp:133-145`
- `P13`: `native/src/initialization/setup_owner.cpp:145-164`; `native/src/initialization/context.hpp:52-53`; `native/src/app/prepare.cpp:6-24`; `native/src/research/environment.cpp:62-69`; `native/src/app/physical_session.cpp:4-9`; `native/src/cli/settle.cpp:39-50`
- `P14a`: `docs/NATIVE_POLICIES.md:94`; `native/src/research/policy_session.cpp:200`; `native/src/runtime/research.cpp:356-359`
- `P14b`: `docs/NATIVE_POLICIES.md:94`; `native/src/research/policy_session.cpp:154-159`; `native/src/runtime/research_io.cpp:179-237`; `native/src/research/lifecycle.cpp:87`; `native/src/research/plugin_policy.cpp:76-80`; `native/src/runtime/research.cpp:624`
- `P14c`: `docs/NATIVE_POLICIES.md:94`; `native/src/research/policy_session.cpp:138`; `native/src/viewer/research.cpp:20`; `native/src/research/plugin_policy.cpp:102-104`; `native/tests/research/test_lifecycle.cpp:135-151`
- `P14d`: `docs/NATIVE_POLICIES.md:94`; `native/src/research/policy_session.hpp:58-76`; `native/src/research/lifecycle.cpp:263-292`; `native/cmake/NativePhysicsResearch.cmake:48`; `native/src/viewer/ride.cpp:29`; `native/src/viewer/research.cpp:11-15`
- `P15`: `AGENTS.md:1`
- `P16`: `native/src/recording/capture.hpp:8-38`; `native/src/recording/publish.cpp:43`; `native/src/platform/files.cpp:111-112`; `native/src/recording/export.cpp:77`; `native/src/golden/episode.cpp:361-371`; `native/tests/recording/test_publish.cpp:88-96`
- `P17a`: `native/src/platform/signals.cpp:41-49`; `native/src/cli/research.cpp:142`; `native/src/research/lifecycle.cpp:223`; `native/src/recording/interrupt.cpp:6-15`; `native/src/cli/ride.cpp:67`; `native/src/cli/settle.cpp:41`
- `P17b`: `native/src/platform/signals.cpp:41-49`; `native/src/cli/research.cpp:142`; `native/src/research/lifecycle.cpp:223`; `native/src/recording/interrupt.cpp:6-15`; `native/src/cli/ride.cpp:67`; `native/src/cli/settle.cpp:41`
- `P17c`: `native/src/platform/signals.cpp:41-49`; `native/src/cli/research.cpp:142`; `native/src/research/lifecycle.cpp:223`; `native/src/recording/interrupt.cpp:6-15`; `native/src/cli/ride.cpp:67`; `native/src/cli/settle.cpp:41`
- `P18`: `native/src/replay/input.cpp:203-207`; `native/src/replay/legacy.cpp:119-177`; `native/src/replay/session.cpp:74-92`; `native/src/cli/replay.cpp:83-122`; `native/src/viewer/ride.cpp:24-168`; `native/src/replay/session.cpp:95`
- `P19`: `CMakePresets.json:53-59`; `docs/RIDE.md:94-97`; `docs/RIDE.md:98-104`
- `P20a`: `native/src/cli/batch.cpp:25-28`; `native/src/batch/sweep.cpp:38-48`; `native/tests/batch/test_plan.cpp:146-161`; `native/src/platform/process.cpp:240-245`; `native/tests/batch/test_plan.cpp:148-164`; `native/tests/platform/test_primitives.cpp:107-213`
- `P20b`: `native/src/cli/batch.cpp:25-28`; `native/src/batch/sweep.cpp:38-48`; `native/tests/batch/test_plan.cpp:146-161`; `native/src/platform/process.cpp:240-245`; `native/tests/batch/test_plan.cpp:148-164`; `native/tests/platform/test_primitives.cpp:107-213`
- `P20c`: `native/src/cli/batch.cpp:25-28`; `native/src/batch/sweep.cpp:38-48`; `native/tests/batch/test_plan.cpp:146-161`; `native/src/platform/process.cpp:240-245`; `native/tests/batch/test_plan.cpp:148-164`; `native/tests/platform/test_primitives.cpp:107-213`
- `P21a`: `native/src/io/hash.cpp:13-27`; `native/src/platform/files.cpp:207-223`; `native/src/engine_call.cpp:387`; `native/src/io/npy.cpp:143-150`; `native/src/random/noise_tape.cpp:98-114`; `native/src/research/rider_random.cpp:19-39`
- `P21b`: `native/src/io/hash.cpp:13-27`; `native/src/platform/files.cpp:207-223`; `native/src/engine_call.cpp:387`; `native/src/io/npy.cpp:143-150`; `native/src/random/noise_tape.cpp:98-114`; `native/src/research/rider_random.cpp:19-39`
- `P21c`: `native/src/io/hash.cpp:13-27`; `native/src/platform/files.cpp:207-223`; `native/src/engine_call.cpp:387`; `native/src/io/npy.cpp:143-150`; `native/src/random/noise_tape.cpp:98-114`; `native/src/research/rider_random.cpp:19-39`
- `P21d`: `native/src/io/hash.cpp:13-27`; `native/src/platform/files.cpp:207-223`; `native/src/engine_call.cpp:387`; `native/src/io/npy.cpp:143-150`; `native/src/random/noise_tape.cpp:98-114`; `native/src/research/rider_random.cpp:19-39`
- `P22`: `AGENTS.md:1`
- `P23`: `native/src/writers/tire.cpp:620-621`; `native/src/writers/cruise.cpp:48-83`; `native/src/pneumatic/carcass.cpp:221-241`; `native/src/pneumatic/brush.cpp:371-390`; `native/src/pneumatic/model.cpp:36-52`; `native/src/pneumatic/geometry.cpp:309-339`
- `P24a`: `native/src/tools/check_frontends.cpp:361-720`; `native/src/tools/check_contracts.cpp:106-471`; `native/src/tools/check_reports.cpp:297-313`; `tools/run_native_checks.sh:56-77`; `tools/run_tests.sh:117-120`; `tools/check_change.sh:48-50`
- `P24b`: `native/src/tools/check_frontends.cpp:361-720`; `native/src/tools/check_contracts.cpp:106-471`; `native/src/tools/check_reports.cpp:297-313`; `tools/run_native_checks.sh:56-77`; `tools/run_tests.sh:117-120`; `tools/check_change.sh:48-50`

## Missing Requirements (WHOLE_REGISTRY)

- M1 — Reposition gating is split-brained (product-level): config validation demands `spindle` attachment, runtime demands `spindle` AND `motor_clutch`, and the stall reflex (P02b) inherits whichever path armed it. There is no single product statement of which configurations may legally reposition; simulation-decisions ruling #44 ties it to the one-way clutch physics, but the config boundary doesn't encode it (see C1). Missing requirement: a uniform `reposition_possible(config)` predicate applied at validate, decode, and runtime — one answer, not three.
- M2 — Open product decisions carried by the registry itself: (a) P03b — whether `PedalingSnapshot::coasting` splits into cadence-cutoff latch vs coast-profile activation, or `stop_time_s` is retired — explicitly unresolved; current behavior is Python-oracle parity, an engineering artifact. (b) P14d — `PlaybackClock` (research/lifecycle.cpp:263-292) is defined with a seven-test contract but has no production caller found in `native/src` (only its own definition sites); the 'pace a viewer path OR mark parity-dead' decision is still open, and no comment marks it dead. (c) P22(c-2) — whether `drivetrain::validate` is a trusted-secondary validator or a defective omission is undecided. (d) P23(c) — whether `traction_limited` has a real producer or is intentionally reserved remains unverified in the code paths inspected.
- M3 — Assist-hold state coherence has no declared product semantics: P03c wants `set_state` to reject incoherent holds, but the deeper question — is `hold_power_w>0 && hold_torque_nm<=0` physically meaningless (reject) or merely a degenerate zero-cap (accept-and-zero)? — is an unwritten product rule. The same for `release_hold_left_s>0` with both stored terms zero: reject vs treat-as-expired. C3 shows code treats them silently; the product decision is absent.
- M4 — Reposition retry/abort policy missing: nothing states how many automatic stall-repositions are acceptable before the system should surface 'drivetrain jammed' as a terminal condition (G1). Real products would bound retries or escalate; here the only bound is the indirect balance/crash gate.
- M5 — `pending_reposition_` restore semantics undocumented at product level: dropping a queued operator request on restore (G4) is coded as deliberate ('transient input') but no product requirement states whether an operator's reposition request should survive a checkpoint — for a replayed/checked episode the answer affects replay determinism (P18 records `rider_commands`; an operator reposition is NOT a rider command, so it escapes the evidence battery entirely).
- M6 — `-6 N` rider-support rule scope is open (P06a): 'wired into PeriodAccounting/ModelStatus' vs 'scoped to ride-level acceptance gating' — the property itself offers both; which one is the product contract is unresolved.
- M7 — `operator_stop_deferred` journaling scope (P17c): research journal emits it (lifecycle.cpp:241); the physical `bike-ride` frontend path was not found to journal the kind at all — the property's alternative ('or RECORDING.md scopes the kind to research journals only') is the operative leg, but the doc text ('the frontend never journals an interrupt it delivered immediately') is generic, not scoped — the scoping decision is implicit, not written.
- M8 — GPU↔CPU numerical parity acceptance criteria: P23(d) pins JSON depth, worker-pool sync, and `gpu_ok` gating for batch runners, and P01a asserts `loss_j` semantics across CPU and GPU, but no property states the numerical tolerance/parity contract for GPU drivetrain vs native (bitwise? tolerance-based? per-step?) — the batch-runner machinery is specified without the comparison rule it exists to serve.

## WHOLE_REGISTRY Findings

C1 — P01b vs the live validators (configuration boundary inconsistency): the registry requires `reposition_on_stall=true` to be rejected unless `motor_clutch=true` at BOTH `bike::configuration::validate` and the wire decode boundary, matching `request_crank_reposition()`. In code, `native/src/configuration/validate.cpp:1330` gates reposition_on_stall only on `articulated.pedal_attachment=="spindle"`; `writer_config.hpp` `decode_drive_policies` similarly lacks the motor_clutch gate. Only `NativeRideRuntime::request_crank_reposition()` (runtime.cpp:830-857) enforces both conditions. Consequence: `reposition_on_stall=true && motor_clutch=false` validates end-to-end, so the stall reflex (P02b) can arm and drive a rigidly-coupled rotor backward — the exact backdriving the property forbids — while the manual reposition path refuses the same configuration. P01b over-claims the boundary coverage.

C2 — P02a/P03a vs `PedalingPolicy::set_state` (validation-boundary split): the property requires decode_policies AND set_state to jointly enforce (a') `!stall_elapsed_s => crank_rad==0 && wheel_rad==0`, (b') goal⇔reason∈{1,2}, goal⊥cooldown>0, goal⇒(!coasting && effort==0), and (P03a) reject `(coasting || reposition-goal) && effort!=0`. The implemented set_state (pedaling.cpp:20-44) checks only per-field finite/nonnegative, `reason<=2`, and `coasting⇒target_phase.has_value()`. None of the cross-field implications (a'), (b'), or the P03a rejection are enforced at either boundary, so a wire snapshot asserting `goal_rad` with `reason==0`, or `coasting` with nonzero effort, is stageable — violating the very invariants the advancing step assumes (armed/goal/cooldown interlock). decode_policies duplicates the scalar checks but not the relational ones.

C3 — P03c vs `AssistController::set_state` (same defect class): the property requires `set_state` to reject `release_hold_left_s>0 && (hold_power_w<=0 || hold_torque_nm<=0)` and `hold_power_w>0 && hold_torque_nm<=0`; the implementation (motor.cpp set_state) validates only per-field nonnegativity. A snapshot can carry an active hold with zero stored power/torque — which the step then interprets as `omega>0 ? min({0/omega, ...}) : 0` = immediate expiry-like behavior instead of a rejected corrupt state.

C4 — P07b is formulated around machinery that does not exist: the property names `peek_rollback_brake_demand(speed, control)` as the non-mutating hysteresis evaluator and requires `apply_forces_body` with `resolved==nullptr` to pass `rollback_demand = active ? peek_... : 0`. In code there is no peek function at all; the `resolved==nullptr` branch passes `rollback_demand = 0.` unconditionally (step.cpp:1313 region). `probe_step_inputs` therefore resolves brakes WITHOUT the rollback contribution even when `rollback_hold_` is engaged, so under an active rollback hold the probe's reported `final_front/final_rear`, `dof_frictionloss` bounds, and brake components diverge from what `advance_physics` (which mutates `rollback_hold_` via `rollback_brake_demand`, step.cpp:2367-2383) computes for identical inputs. Probe-step parity is claimed but not delivered for the rollback lane.

C5 — P07a vs probe model purity: `probe_step_inputs` runs the full `apply_forces` fold inside `stepper_->mutate`, which calls `brake_.apply(m, final_front, final_rear)` — a WRITE to `model->dof_frictionloss` (static_brake.cpp:44-70) that also calls `note_posvel_stale()` when the bounds change. I found no snapshot/restore of `dof_frictionloss` or `posvel_fresh_` around the probe in `NativeRideRuntime::probe_step_inputs` (runtime.cpp:815-828) or `PhysicalStep::probe_step_inputs` (step.cpp:2953+). The probe leaves the model's static-friction bounds at the probe's clamped demands (which exclude rollback — see C4); correctness is only incidentally preserved because every advancing `apply_forces` rewrites the bounds before solve, but the 'persistent model buffers stay pristine' half of P07a is not implemented as stated — the contract is sav

G1 — Stall→reposition retry lattice is unbounded: `PedalingPolicy::step` re-arms the stall detector on every disengage when `reposition_on_stall && !armed` (pedaling.cpp:195-200). A persistent mechanical stall (jammed wheel/chain) therefore cycles `stall_window → reposition goal → timeout → cooldown → re-arm → stall_window…` indefinitely. P02b bounds ONE manoeuvre to `reposition_timeout_s`, but no property or code bounds the retry count; the episode only terminates via balance-loss/crash detection (P09) — an indirect gate that assumes the stalled bike falls over. If the stalled bike remains balanced (e.g. held upright by test fixtures or a braked standstill), the liveness gap is real: no `max_reposition_attempts` exists anywhere.

G2 — Mid-grace braked standstill protection asymmetry: `BalanceStop::braked` re-arms grace only when `launch.phase==hold || grace_spent`. A controlled braked stop taken BEFORE grace expiry does not re-arm; `started_s` keeps burning and once `grace_left_s<=1e-12`, `|speed|<floor` + `!riding` arms `low_started_s` → `balance_lost` fires after `dwell` at a held standstill, while an identical stop one tick later (grace spent) earns a fresh 3 s. Reachable end-to-end: rider brakes to a stop inside the launch window and holds → flagged `balance_lost` though never falling. Property-faithful (it describes exactly this) but a product-level progress quirk worth re-checking.

G3 — Probe/advance divergence under rollback (see C4/C5): with `rollback_hold_` engaged (bike rolling back > `rollback_engage_mps`), `advance_physics` merges `rollback_demand` into `final_front/final_rear` and thus brake bounds; `probe_step_inputs` for the same state resolves `rollback_demand=0` → different `brake` bounds, different `components`, different `front_brake_bound_nm` reported. Any verification flow that probes-then-advances (equilibrium probes, research truth lanes) reads a weaker brake demand than the advancing step will apply — a reachability inconsistency in the observation contract, not just an internal detail.

G4 — Queued operator reposition is silently dropped on restore: `PhysicalStep::restore` (step.cpp:3084) clears `pending_reposition_` deliberately ('transient input'). A `request_crank_reposition()` acknowledged between save and restore is lost — the operator's intent does not survive a checkpoint. For interactive ride sessions this is a small but real recovery gap: the wire contract preserves all FSM state except this queued edge.

G5 — `delivery_pending` liveness verified: `advance_control` returns the identical `pending_transition` while pending (research.cpp:350), the physics loop is bounded by `window->target_step - committed_step` (line 355), and `acknowledge_control` resets pending/window in one shot (396-406). No deadlock found in the control-window protocol itself; the only residue is the stale `stop_requested` flag (C7).

G6 — Publication path verified sound: `publish` stages exclusively via `mkdtemp` under the output root, writ

## Stage 6 Ranked FSM Dispositions

Dispositions: {'convert': 88, 'investigate': 31, 'simplify': 4}. Order: canonical FSM id. Policy: violated parent -> `investigate`; vacuous/unexercised parent -> `investigate`; otherwise fsm-map disposition hint (convert-to-explicit/tighten -> `convert`, simplify/derive/delete -> `simplify`, retain -> `retain` — upgraded to `investigate` under a partially violated parent); implicit classification without a hint -> `convert`.

| fsm_id | disposition | rationale |
|--------|-------------|-----------|
| FSM-001 | convert | parent P02a partially_violated; map classification '`implicit`' per fsm-map disposition guidance |
| FSM-002 | convert | parent P02b partially_violated; map classification '`hybrid` (projects onto explicit output enums `pedalmode::coasting` and `coastreason::{repos...' per fsm-map disposition guidance |
| FSM-003 | convert | parent P03b partially_violated; map classification '`hybrid` (emits explicit `pedalmode` and `coastreason` enums in `pedalingstate`, while persi...' per fsm-map disposition guidance |
| FSM-004 | investigate | parent P04b violated; named counterexamples must be resolved or the machine quarantined |
| FSM-005 | convert | parent P03c partially_violated; map classification '`implicit`' per fsm-map disposition guidance |
| FSM-006 | investigate | parent P01a partially violated; machine (Standalone Freehub Ratchet Machine (`Freehub` / `FreehubSnap) needs follow-up on named gaps |
| FSM-007 | simplify | parent P01c partially_violated; map classification '`implicit`' per fsm-map disposition guidance |
| FSM-008 | simplify | parent P03c partially_violated; map classification '`implicit` (wire/snapshot protocol boundary owned in d01 by `state_codec.{hpp,cpp}` and `pol...' per fsm-map disposition guidance |
| FSM-009 | investigate | parent P05d partially violated; machine (`LaunchPlan` Primary Standstill-Launch Lifecycle (`LaunchPha) needs follow-up on named gaps |
| FSM-010 | convert | parent P04c partially_violated; map classification '`implicit` — encoded without an enum across five fields of `launchplan::state` (`bool shift_...' per fsm-map disposition guidance |
| FSM-011 | convert | parent P05b partially_violated; map classification '`implicit` — encoded via sentinel counter `std::int64_t rise_ticks` (`-1` vs `>= 0`) coupled...' per fsm-map disposition guidance |
| FSM-012 | investigate | parent P05b partially violated; machine (`FrontLiftReaction` Lifecycle & Criterion-B Lagged Hand-Unlo) needs follow-up on named gaps |
| FSM-013 | convert | parent P05b partially_violated; map classification '`implicit` — integer-tick fifo delay line with pre-delivery / steady-delivery phases and ove...' per fsm-map disposition guidance |
| FSM-014 | convert | parent P05b partially_violated; map classification '`implicit` — encoded via `config_.enabled`, `last_tick_step` (`-1` vs `k * period_steps_`), ...' per fsm-map disposition guidance |
| FSM-015 | convert | parent P05c partially_violated; map classification '`implicit` — encoded across `surge_budget_s_left_`, `time_s_`, `samples_` queue, `delayed_` ...' per fsm-map disposition guidance |
| FSM-016 | convert | parent P05c partially_violated; map classification '`implicit` — encoded across `lean_rad_`, `trim_rad_`, `load_samples_` queue, `delayed_load_s...' per fsm-map disposition guidance |
| FSM-017 | investigate | parent P07a violated; named counterexamples must be resolved or the machine quarantined |
| FSM-018 | investigate | parent P06b partially violated; machine (Flat-Pedal Foot-Slip Recovery FSM (`PedalRecoveryState`)) needs follow-up on named gaps |
| FSM-019 | investigate | parent P07a violated; named counterexamples must be resolved or the machine quarantined |
| FSM-020 | investigate | parent P06a partially violated; machine (Soft-Constraint Active-Set & Contact/Limit Mode-Search Autom) needs follow-up on named gaps |
| FSM-021 | investigate | parent P08a partially violated; machine (Test-Only Thread-Local Allocation Fault-Injection & Activity) needs follow-up on named gaps |
| FSM-022 | convert | parent P08a partially_violated; map classification 'implicit tls stack-disciplined protocol fsm + process-global callback provenance classifier' per fsm-map disposition guidance |
| FSM-023 | convert | parent P08c partially_violated; map classification 'implicit 3-axis product fsm (`health × stagefreshness × timecallbackcache`)' per fsm-map disposition guidance |
| FSM-024 | convert | parent P09 partially_violated; map classification 'implicit composite lifecycle & concurrency-exclusion fsm' per fsm-map disposition guidance |
| FSM-025 | investigate | parent P07a violated; named counterexamples must be resolved or the machine quarantined |
| FSM-026 | convert | parent P09 partially_violated; map classification 'implicit 2-state speed-hysteresis latch (`disengaged` / `holding`) + 4-predicate brake merge...' per fsm-map disposition guidance |
| FSM-027 | convert | parent P09 partially_violated; map classification 'implicit 2-wheel independent hysteresis & slip-reseat latch fsm (`frontlatch × rearlatch`)' per fsm-map disposition guidance |
| FSM-028 | convert | parent P08d partially_violated; map classification 'implicit 2-state initialization latch (`empty` vs `holding`) + modular step divider' per fsm-map disposition guidance |
| FSM-029 | convert | parent P09 partially_violated; map classification 'coupled implicit monotone-clock hold filter (`groundedfilter`) + discrete step-dropout count...' per fsm-map disposition guidance |
| FSM-030 | convert | parent P09 partially_violated; map classification 'implicit 6-phase temporal hysteresis & dual-latch fsm (implements simulation decision 14 & 2...' per fsm-map disposition guidance |
| FSM-031 | convert | parent P09 partially_violated; map classification 'implicit monotone first-cause terminal latch (`upright` -> `crashed(cause)`)' per fsm-map disposition guidance |
| FSM-032 | convert | parent P10a partially_violated; map classification 'implicit monotone interval ledger + sticky tri-state validity machine (`not_evaluated` -> `t...' per fsm-map disposition guidance |
| FSM-033 | convert | parent P10b partially_violated; map classification 'implicit 3-state diagnostic warn-once / strict-rejection latch fsm (simulation decision 30)' per fsm-map disposition guidance |
| FSM-034 | convert | parent P10c partially_violated; map classification 'implicit multi-stage transactional buffer/donor/commit pipeline + transport-mode degradation...' per fsm-map disposition guidance |
| FSM-035 | simplify | parent P10c partially_violated; map classification 'explicit-mutex concurrent worker lifecycle & fifo sequence-matching protocol fsm' per fsm-map disposition guidance |
| FSM-036 | convert | parent P10d partially_violated; map classification 'implicit 3-phase (`uninitialized` -> `active` -> `tapeexhausted`) monotone-clock delay queue...' per fsm-map disposition guidance |
| FSM-037 | convert | parent P11 partially_violated; map classification 'implicit hierarchical fsm (atomic owner/reentrancy envelope + boolean/optional control-windo...' per fsm-map disposition guidance |
| FSM-038 | convert | parent P11 partially_violated; map classification 'implicit coupled cursor / queue fsm (operating inside `researchepisode`, driven by `nativere...' per fsm-map disposition guidance |
| FSM-039 | convert | parent P11 partially_violated; map classification 'implicit append-only monotonic-cursor fsm (`runtime::researchrecorder`)' per fsm-map disposition guidance |
| FSM-040 | investigate | parent P11 partially violated; machine (`WheelieTracker` Hysteresis & Persistence Contact-State / Ep) needs follow-up on named gaps |
| FSM-041 | convert | parent P12b holds; map classification 'implicit protocol + concrete driver fsm (`runtime::measurement::driver`, `nativedriver`, `me...' per fsm-map disposition guidance |
| FSM-042 | convert | parent P12c partially_violated; map classification 'implicit sequential phase orchestrator ([realtime_command.cpp](file:///users/vladislav.molch...' per fsm-map disposition guidance |
| FSM-043 | convert | parent P13 partially_violated; map classification 'implicit fsm** (object lifecycle + multi-stage pipeline spread across three coupled structs ...' per fsm-map disposition guidance |
| FSM-044 | convert | parent P13 partially_violated; map classification 'implicit ownership-handoff & cli session protocol** (spanning `app/prepare.*`, `app/physical...' per fsm-map disposition guidance |
| FSM-045 | convert | parent P13 partially_violated; map classification 'implicit multi-phase solver & force-attachment fsm' per fsm-map disposition guidance |
| FSM-046 | convert | parent P13 partially_violated; map classification 'implicit two-phase staging & atomic filesystem publication fsm' per fsm-map disposition guidance |
| FSM-047 | convert | parent P13 partially_violated; map classification 'implicit artifact dispatch & two-stage state-transfer protocol' per fsm-map disposition guidance |
| FSM-048 | convert | parent P14c partially_violated; map classification 'implicit product fsm with an explicit internal operator-command tag (`enum class action : st...' per fsm-map disposition guidance |
| FSM-049 | investigate | parent P14d violated; named counterexamples must be resolved or the machine quarantined |
| FSM-050 | convert | parent P15 partially_violated; map classification 'implicit fsm (8 boolean/optional coordination latches + monotone `episode_id_` + `failurerep...' per fsm-map disposition guidance |
| FSM-051 | convert | parent P15 partially_violated; map classification 'implicit raii + pointer-latch fsm (`library` + `instance` + `pluginpolicy`)' per fsm-map disposition guidance |
| FSM-052 | investigate | parent P15 partially violated; machine (`TractionControlPolicy` Slip-Limit & Standstill-Release Hybr) needs follow-up on named gaps |
| FSM-053 | convert | parent P15 partially_violated; map classification 'implicit fsm (`closed_`, `failed_`, `columns_.empty()`, `generation_`, `next_time_s_`, `rows...' per fsm-map disposition guidance |
| FSM-054 | investigate | parent P16 partially violated; machine (`RunCapture` Launch Evidence & Finalization State Machine) needs follow-up on named gaps |
| FSM-055 | convert | parent P16 partially_violated; map classification 'implicit fsm candidate** (monotone per-episode counter `next_id_` coupled with `runcapture::...' per fsm-map disposition guidance |
| FSM-056 | convert | parent P16 partially_violated; map classification 'implicit fsm candidate** (`std::unique_ptr<runcapture> capture_` acting as `{unadapted, adap...' per fsm-map disposition guidance |
| FSM-057 | investigate | parent P16 partially violated; machine (Two-Level Exclusive Publication, DuckDB Transaction/Appender) needs follow-up on named gaps |
| FSM-058 | investigate | parent P17c violated; named counterexamples must be resolved or the machine quarantined |
| FSM-059 | convert | parent P16 partially_violated; map classification 'implicit fsm candidate** (raii mode-dependent `database` + streaming `cursor::impl` chunk st...' per fsm-map disposition guidance |
| FSM-060 | convert | parent P16 partially_violated; map classification 'implicit fsm candidate** (`{open (database_ != nullptr), closed (database_ == nullptr)}` gat...' per fsm-map disposition guidance |
| FSM-061 | convert | parent P16 partially_violated; map classification 'implicit fsm candidate** (dual `std::atomic<bool>` reentrancy guard + runtime state precondi...' per fsm-map disposition guidance |
| FSM-062 | convert | parent P16 partially_violated; map classification 'implicit fsm candidate (test support infrastructure)' per fsm-map disposition guidance |
| FSM-063 | convert | parent P18 partially_violated; map classification 'implicit fsm** (multi-field boolean/optional/cursor/generation product state coordinated wit...' per fsm-map disposition guidance |
| FSM-064 | convert | parent P19 partially_violated; map classification 'implicit fsm** (3 boolean flags `paused`, `initialized`, `rebase_pending` + discrete `scale`...' per fsm-map disposition guidance |
| FSM-065 | convert | parent P19 partially_violated; map classification 'implicit 2-state estimator** (`std::optional<double> value_` + monotone clock/sim-time basel...' per fsm-map disposition guidance |
| FSM-066 | investigate | parent P19 partially violated; machine (`InputState` Mode-Dependent Operator Latch & Speed-Scale Mac) needs follow-up on named gaps |
| FSM-067 | convert | parent P19 partially_violated; map classification 'implicit orchestration fsm** (loop control state distributed across local flags `request_fra...' per fsm-map disposition guidance |
| FSM-068 | convert | parent P19 partially_violated; map classification 'implicit fsm** (`is_closed`, `poisoned`, `frame.has_value()`, `graphics` + monotone `(genera...' per fsm-map disposition guidance |
| FSM-069 | convert | parent P19 partially_violated; map classification 'implicit resource & fault-latch fsm** (process-global `context_count` + per-instance `regist...' per fsm-map disposition guidance |
| FSM-070 | convert | parent P19 partially_violated; map classification 'implicit 2-state irreversible global latch** (mutex-guarded `std::optional<std::filesystem::...' per fsm-map disposition guidance |
| FSM-071 | investigate | parent P20b violated; named counterexamples must be resolved or the machine quarantined |
| FSM-072 | convert | parent P20a partially_violated; map classification 'implicit fsm** (concurrent parent-coordinator + worker-pool + per-job lifecycle coordinated ...' per fsm-map disposition guidance |
| FSM-073 | investigate | parent P20b violated; named counterexamples must be resolved or the machine quarantined |
| FSM-074 | investigate | parent P17c violated; named counterexamples must be resolved or the machine quarantined |
| FSM-075 | investigate | parent P17b violated; named counterexamples must be resolved or the machine quarantined |
| FSM-076 | convert | parent P18 partially_violated; map classification 'implicit fsm** (two sub-machines in `cli/replay.cpp`: (a) `inspect` presentation cursor/gene...' per fsm-map disposition guidance |
| FSM-077 | convert | parent P20c partially_violated; map classification 'implicit fsm** (stream protocol verifier over drained `physicalsampledata` intervals)' per fsm-map disposition guidance |
| FSM-078 | convert | parent P20c partially_violated; map classification 'implicit fsm** (absorbing valid-prefix latch in `validprefix` + tri-state `{true, false, nul...' per fsm-map disposition guidance |
| FSM-079 | convert | parent P20c partially_violated; map classification 'implicit fsms** (three temporal dwell/sequence classifiers over interval histories: `persist...' per fsm-map disposition guidance |
| FSM-080 | convert | parent P20c partially_violated; map classification 'implicit fsm** (raii `std::atomic<bool>` compare-exchange lock `guard` + runtime lifecycle/s...' per fsm-map disposition guidance |
| FSM-081 | investigate | parent P20b violated; named counterexamples must be resolved or the machine quarantined |
| FSM-082 | convert | parent P20c partially_violated; map classification 'implicit fsm** (atomic staging-rename checkpoint lifecycle + pre/post source-tree sha-256 im...' per fsm-map disposition guidance |
| FSM-083 | convert | parent P17a partially_violated; map classification 'implicit fsm candidate (raii scope object coupled to a process-global 16-slot lock-free regi...' per fsm-map disposition guidance |
| FSM-084 | convert | parent P20a partially_violated; map classification 'implicit fsm candidate (multi-variable os subprocess, process-group signaling, and dual-pipe...' per fsm-map disposition guidance |
| FSM-085 | convert | parent P21b partially_violated; map classification 'implicit fsm candidate (move-only os handle lifecycle with distinct silent-destructor vs thr...' per fsm-map disposition guidance |
| FSM-086 | convert | parent P21a partially_violated; map classification 'implicit fsm candidate (streaming 64-byte block buffer + destructive finalizer **missing a `...' per fsm-map disposition guidance |
| FSM-087 | convert | parent P21b partially_violated; map classification 'implicit fsm candidate (bounded-depth pushdown transducer over rapidjson's sax token stream)' per fsm-map disposition guidance |
| FSM-088 | investigate | parent P21b partially violated; machine (Strict Base64 Quad/Padding Scanner (`b64_decode_strict`)) needs follow-up on named gaps |
| FSM-089 | convert | parent P21b partially_violated; map classification 'implicit fsm candidate (transient format-decoding state machines)' per fsm-map disposition guidance |
| FSM-090 | convert | parent P21c partially_violated; map classification 'implicit fsm candidate (persistent prng state machine combining move-only pimpl ownership wi...' per fsm-map disposition guidance |
| FSM-091 | convert | parent P21c partially_violated; map classification 'implicit fsm candidate (monotonic child-index counter producing disjoint hierarchical spawn ...' per fsm-map disposition guidance |
| FSM-092 | convert | parent P21d partially_violated; map classification 'implicit fsm candidate (synchronous closed-call solver mode/status machine; explicitly `bike...' per fsm-map disposition guidance |
| FSM-093 | convert | parent P21d partially_violated; map classification 'implicit fsm candidate (synchronous trf solver status machine + stateful `evaluator` residua...' per fsm-map disposition guidance |
| FSM-094 | investigate | parent P21d partially violated; machine (Three-Valued Epistemic Linear Feasibility Pipeline (`linear_) needs follow-up on named gaps |
| FSM-095 | investigate | parent P22 partially violated; machine (`BikeSuspensionSystem` & `SuperDeluxeDamper` Preset / Click ) needs follow-up on named gaps |
| FSM-096 | convert | parent P22 partially_violated; map classification 'implicit two-phase builder/accumulator protocol (**phase 1: monotonic unique-key registratio...' per fsm-map disposition guidance |
| FSM-097 | convert | parent P22 partially_violated; map classification 'implicit 2-state initialization protocol encoded via a floating-point `quiet_nan()` sentinel...' per fsm-map disposition guidance |
| FSM-098 | convert | parent P22 partially_violated; map classification 'implicit fsm' per fsm-map disposition guidance |
| FSM-099 | convert | parent P23 partially_violated; map classification 'implicit fsm (4-mode lifecycle encoded across `bool initialized_`, array zero-ness predicate...' per fsm-map disposition guidance |
| FSM-100 | investigate | parent P23 partially violated; machine (Pneumatic Rim-Strike Event Tracker) needs follow-up on named gaps |
| FSM-101 | simplify | parent P23 partially_violated; map classification 'implicit dynamic multi-slot & per-bristle fsm' per fsm-map disposition guidance |
| FSM-102 | convert | parent P23 partially_violated; map classification 'implicit fsm (`brushstate` optional-group presence + geometric branch-continuity classifier ...' per fsm-map disposition guidance |
| FSM-103 | convert | parent P23 partially_violated; map classification 'implicit per-station 2-mode + stick/slide fsm (`stationstate` array of width `n ∈ [16, 512]`...' per fsm-map disposition guidance |
| FSM-104 | convert | parent P23 partially_violated; map classification 'implicit 4-aspect protocol fsm duplicated identically across `tirewriter` (`compliant_2d`), ...' per fsm-map disposition guidance |
| FSM-105 | convert | parent P23 partially_violated; map classification 'implicit 3-regime controller fsm (`cruisestate::engaged` + anti-windup saturation/traction h...' per fsm-map disposition guidance |
| FSM-106 | investigate | parent P07a violated; named counterexamples must be resolved or the machine quarantined |
| FSM-107 | investigate | parent P07a violated; named counterexamples must be resolved or the machine quarantined |
| FSM-108 | convert | parent P23 partially_violated; map classification 'implicit per-world fsm (`gpuworldstate` stepped in-place on cpu/cuda/metal)' per fsm-map disposition guidance |
| FSM-109 | convert | parent P23 partially_violated; map classification 'implicit concurrent coordination & double-buffered pipeline fsm' per fsm-map disposition guidance |
| FSM-110 | investigate | parent P23 partially violated; machine (Iterative JSON Pushdown Validator (`detail::scan_value` in `) needs follow-up on named gaps |
| FSM-111 | convert | parent P24a partially_violated; map classification 'implicit concurrent coordination fsm' per fsm-map disposition guidance |
| FSM-112 | convert | parent P24a partially_violated; map classification 'implicit multi-phase orchestration & monotone-latch outcome fsm' per fsm-map disposition guidance |
| FSM-113 | investigate | parent P24a partially violated; machine (Explicit Lexical Quote/Comment Scanner Mini-FSMs (`shlex_spl) needs follow-up on named gaps |
| FSM-114 | convert | parent P24a partially_violated; map classification 'implicit pipeline/latch fsm + filesystem-backed multi-process coordination fsm' per fsm-map disposition guidance |
| FSM-115 | convert | parent P24b partially_violated; map classification 'implicit resource lifecycle fsm gated by an unconditional top-level circuit breaker' per fsm-map disposition guidance |
| FSM-116 | convert | parent P24b partially_violated; map classification 'implicit two-phase commit / filesystem rollback transaction fsm' per fsm-map disposition guidance |
| FSM-117 | convert | parent P24b partially_violated; map classification 'implicit persistent cache + staged-directory / marker-file fsm' per fsm-map disposition guidance |
| FSM-118 | convert | parent P24a partially_violated; map classification 'implicit configure-time fail-closed qualification pipeline with persistent `cmakecache.txt` ...' per fsm-map disposition guidance |
| FSM-119 | investigate | parent P24a partially violated; machine (Artifact Relocation, Code-Signing & Staged Report Publicatio) needs follow-up on named gaps |
| FSM-120 | convert | parent P24b partially_violated; map classification 'implicit fsm' per fsm-map disposition guidance |
| FSM-121 | convert | parent P24b partially_violated; map classification 'implicit fsm' per fsm-map disposition guidance |
| FSM-122 | convert | parent P24b partially_violated; map classification 'implicit fsm' per fsm-map disposition guidance |
| FSM-123 | convert | parent P24b partially_violated; map classification 'implicit fsm' per fsm-map disposition guidance |

## Appendix: Post-Snapshot Delta vs Merged Stage A

`0502660dd14bd1021b3da90a61e20dcb1d5805a1..main` contains `0` commit(s) — main has NOT moved past the audited snapshot; no fix merge exists yet. The Stage A feature branch `feat/surface-stage-a-2026-11-18` adds:

```
a0bb59e1 feat(writers): add C1-smooth SurfaceMap evaluator and wheel-load friction law
5cbe72a4 feat(terrain,config): extend SurfaceSpec/SurfaceMap with crr_delta, load_sensitivity and blend_m
f502d012 docs(plan): add Stage A surface simulation implementation plan
37946646 docs(spec): add Stage A surface simulation design spec and record rulings
```

Touch summary (read-only `git diff --stat`):

```
docs/agents/simulation-decisions.md                |  11 +-
 .../plans/2026-11-18-surface-stage-a.md            | 449 +++++++++++++++++
 .../specs/2026-11-18-surface-stage-a-design.md     | 224 +++++++++
 examples/research/viewer_physics_pneumatic.toml    |   5 +
 native/src/config_types.hpp                        |   2 +
 native/src/config_validation.hpp                   |  77 ++-
 native/src/configuration/schema.cpp                |   4 +-
 native/src/platform/process.cpp                    |   7 +-
 native/src/runtime/project_config.cpp              |  26 +-
 native/src/terrain/alpine.cpp                      |  16 +-
 native/src/terrain/generators.hpp                  |   1 +
 native/src/terrain/surface.cpp                     |  50 +-
 native/src/terrain/trackfile.cpp                   |  28 +-
 native/src/terrain/types.hpp                       |   3 +
 native/src/writer_config.hpp                       |  12 +-
 native/src/writers/tire_common.cpp                 | 230 +++++++++
 native/src/writers/tire_common.hpp                 |  37 ++
 native/tests/config/test_schema.cpp                |  19 +
 .../reference/pneumatic_reference_support.hpp      |   4 +-
 native/tests/reference/pneumatic_tire_fixture.hpp  |   3 +-
 .../reference/test_native_input_contracts.cpp      |  98 +++-
 native/tests/reference/test_tyre_road_profile.cpp  |   4 +-
 native/tests/terrain/CMakeLists.txt                |   2 +-
 native/tests/terrain/test_generators.cpp           |  61 +++
 native/tests/terrain/test_presets.cpp              |  27 +
 native/tests/terrain/test_surface_eval.cpp         | 551 +++++++++++++++++++++
 tests/fixtures/config/resolved/pneumatic.json      |   2 +-
 .../fixtures/config/viewer_physics_pneumatic.toml  |   5 +
 28 files changed, 1899 insertions(+), 59 deletions(-)
```

Mapping to audited properties: `native/src/terrain/*` + `writers/tire_common.*` + config schema changes land inside the P22 (configuration/kinematics/terrain) and P23 (tyre/contactlaw/writers) linked-FSM scopes as an additive feature, not a violation fix; P20 (batch/process) is untouched. No Stage-A commit contradicts or resolves an R1/R2-named counterexample.
