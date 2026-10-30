# Naming and diagnostic registry — 2026-10-30

Companion registry to `docs/superpowers/plans/2026-10-30-cpp-naming-logging-unification.md`
(task C0). Each confirmed rename records the concept, the C++ symbols, the external spelling
that stays fixed, the owning slice and the covering test. Status values: `rename` (scheduled),
`fixed` (applied, focused gate green), `exempt` (external contract), `reviewed/no change`
(audited, consistent), `deferred` (open slice).

## Confirmed renames

| Concept | Old symbol → new symbol | External spelling (unchanged) | Owner | Producer | Consumers | Test | Status |
|---|---|---|---|---|---|---|---|
| Bounded raw file read (128 MiB, no-follow) | `io::read_numpy_bytes` → `io::read_bounded_file_bytes` | none (internal API) | N2 | `native/src/io/npy.hpp:34`, `npy.cpp:143–149` | `cli/physical_capture.cpp`, `cli/batch.cpp`, `cli/ride.cpp`, `replay/legacy.cpp`, `research/configuration.cpp`, `research/metadata.cpp`, `initialization/request.cpp`, `initialization/state_file.cpp`, `golden/episode.cpp`, `validation/common.hpp`, `runtime/realtime_provenance.cpp`, `runtime/realtime_command.cpp`, `recording/analysis_detail.hpp` | `reference.measure_realtime`, `reference.golden_episode_codec`, `migration_T51` | fixed |
| Overwrite (not accumulate) of output buffers | `BrakeWriter::accumulate_validated`, `RiderForcesWriter::accumulate_validated` → `compute_validated_into` | none (private) | N3 | `writers/brake.hpp:71`, `writers/rider_forces.hpp:89` + impl | owning class `compute_into` paths only | `reference.native_brake_resistance`, `reference.native_rider_forces` | fixed |
| Reentrancy exclusion of all runtime operations | `AdvancementGuard` → `RuntimeOperationGuard`, `advancing_` → `busy_` | error text `"advance already in progress on this runtime"` kept | N4a | `runtime/runtime.hpp:232`, `runtime.cpp:69–88` | all guarded calls in `runtime.cpp` (advance/snapshot/render/image/state/reset/close) | `reference.native_runtime_commands` | fixed |
| Brake demand fractions in research control | `ResearchCommand.front/rear`, `ResearchWindow.front/rear` → `front_brake_demand`, `rear_brake_demand` | binding keywords `front_brake_demand`/`rear_brake_demand` (already canonical) | N4b | `runtime/research.hpp:43–82`, `research.cpp`, `research_binding.cpp:269–273` | research environment + binding adapter | `reference.native_research_environment` | fixed |
| Observed torque demand (Nm) | `ResearchWindow.seen_demand` → `observed_demand_nm` | none (internal) | N4b | `runtime/research.hpp`, `research.cpp` | research window/delivery | `reference.native_research_environment` | fixed |
| Queue of delayed controls (not command history) | `ResearchEpisode.commands` → `pending_delayed_controls` | requested/applied wire encoding unchanged | N4b | `runtime/research.hpp`, `research.cpp` | episode drain/delivery paths | `reference.native_research_environment` | fixed |
| Default worker count (platform policy) | `batch::performance_cores()` → `batch::default_worker_count()` | none (internal) | N1 | `batch/process_pool.cpp:18–28`, `batch/plan.hpp` | `cli/batch.cpp:23` (lazy default; explicit `--jobs`/`--serial` bypasses discovery) | `migration_T51`, `migration_T47` | fixed |
| Child stdout/stderr capture | `platform::ProcessResult.out/err`, `batch::Result.out/err` → `stdout_text/stderr_text` | result record `"stderr_tail"`/`"error"` keys, `stdout.log`/`stderr.log` filenames | N1 | `platform/process.hpp:29–33`, `batch/plan.hpp:22–28`, `batch/process_pool.cpp:59–66` | `tools/check_contracts.cpp`, `check_reports.cpp`, `python_audit_main.cpp`, `runtime/realtime_provenance.cpp`, `batch/summary.cpp`, `batch/sweep.cpp`, `cli/batch.cpp`; tests T06/T36/T46/T47/T51, `integration/test_batch.cpp`, `test_replay_playback.cpp`, reference sweep/lifecycle/CLI/engine/benchmark/realtime consumers | `migration_T06`, `migration_T51`, `reference.batch`, `reference.native_research_sweep` | fixed |
| Monotonic job duration | `batch::Result.wall_s` → `wall_duration_s` | serialized `"wall_s"` key and sweep `"returncode"`/`"wall_s"` unchanged | N1 | `batch/plan.hpp`, `process_pool.cpp` | `summary.cpp` (`"wall_s"` ← field), `sweep.cpp`, `cli/batch.cpp`, result readers | `migration_T51`, `reference.native_research_sweep` | fixed |
| Ideal-transmission model family predicate | `simplified_`/`simplified()` → `uses_ideal_transmission_`/`uses_ideal_transmission()` | transmission model strings `elastic_chain`/`ideal_mid_drive`/`geometric_ideal_mid_drive` | N5a | `writers/drivetrain.hpp:374,405`, `drivetrain.cpp` | force staging / solved interval paths | `reference.native_drivetrain` | fixed |
| Contact reference position (world, m) — front/rear/handlebar `geom_xpos` | `TireSnapshot.wheel_axis_m` → `reference_position_world_m` | wire key `wheel_axis_m` unchanged; pneumatic `hub_position_world_m` (real hub) unchanged; `wheel_axis_moment_nm` untouched | N5b | `writers/tire_common.hpp:51`, `tire.cpp`, `distributed_tire.cpp`, `pneumatic_tire.cpp`, `runtime/native_contacts.cpp`, `runtime/step.cpp`, `legacy/controls.cpp`, `validation/runtime_access.cpp` | snapshot producers/consumers, wire encoders | `reference.native_encode_columns`, `reference.native_tire`, `reference.native_distributed_tire`, `reference.native_pneumatic_tire`, `native_contract_tests` | fixed |

## External-spelling exemptions (pinned, not renamed)

| Spelling | Scope | Reason |
|---|---|---|
| batch result record `name`/`index`/`out`/`exit`/`wall_s`/`params`/`argv`/`stderr_tail`/`error` | `batch::result_record` JSONL | serialized contract (`native/src/batch/summary.cpp`); `"out"` is the job output *path*, `"wall_s"` the rounded monotonic duration — internal names may unify, keys stay |
| sweep `sweep.json` `jobs`/`passthrough`/`runs[].seed`/`returncode`/`wall_s` | `batch::research_sweep` artifact | serialized contract |
| wire key `wheel_axis_m` | `runtime/step.cpp` encoder + golden schema | external channel name; C++ member renamed separately |
| `received_wall_s`, measurement/recording clock fields | recording/research schemas | simulation time vs monotonic wall vs absolute timestamps are distinct concepts; existing spellings stay |
| policy C ABI `bike/policy_v1.h` (`create`/`reset`/`act`/`destroy`, owned error buffer) | `native/include/bike/policy_v1.h` | ABI v1 contract |
| exported binding names (`front_brake_demand`, `rear_brake_demand`, module/symbol names) | `research_binding.cpp`, `binding*.cpp` | already canonical / Python-facing contract |
| CLI flags (`--jobs`, `--serial`, `--out`, …) and help text | `cli/arguments*` | user-facing contract |
| `qpos`, `qvel`, `mjModel`, `mjData`, BLAS/LAPACK names | engine/MuJoCo boundary | vendor spelling; MuJoCo object ids are not UUIDs |
| `tyre` spelling | `biketyre` namespace, tyre modules | established project domain spelling |
| STL pairs `erase`/`remove`, `delete`; filesystem `remove` | all | idiom exemption |
| `Job.id` / `Job.index` | `batch/plan.hpp` | id = plan-scope identity slug, index = position; documented, not a UUID to invent |

## Reviewed / no change required at this audit

| Area | Verdict |
|---|---|
| `io::parse_npy` / `io::encode_npy` | genuinely NumPy codec operations — `read`/`parse`/`decode` distinction is correct here |
| `tire` / `distributed_tire` / `pneumatic_tire` `accumulate_validated` | real accumulation into shared force buffers (unlike the writer overwrite bug); semantics verified in N3/G1, not renamed |
| namespaces `runtime`, `rider`, `drivetrain`, `biketyre` | interface contract `migration/interface-contracts.md:22`; no migration to `bike::*` |
| `io::Value`, generic container `value`, `mjData`, opaque ABI `void*`, short local `result` | context-sufficient names, §2 vague-name clause |
| `Config`/`Options`/`State`/`Snapshot`/`Result`/`Owner` roles | audit found no confirmed role-suffix violation; §2 rules apply to new classes |
| `get_*` accessors (`size()`, `state()`, `config()`) | idiomatic, no hidden lifecycle/I/O; not converted |

## Directory coverage marks

| Directory | Mark |
|---|---|
| `src/io/` | rename in slice (N2) |
| `src/writers/` | rename in slice (N3, N5a, N5b) |
| `src/runtime/` | rename in slice (N4a, N4b, N5b); `realtime_provenance`/`realtime_command` touched by N1/N2 consumer updates |
| `src/batch/` | rename in slice (N1); diagnostic slice (L4) |
| `src/cli/` | consumer updates (N1, N2); diagnostic slices (L1–L4) |
| `src/platform/` | rename in slice (N1 `ProcessResult`) |
| `src/research/` | consumer updates (N2); diagnostic slices (L2, L3) |
| `src/tools/` | consumer updates (N1); diagnostic slice (L5) |
| `src/recording/` | consumer updates (N2 `analysis_detail.hpp`) |
| `src/initialization/` | consumer updates (N2) |
| `src/golden/` | consumer updates (N2) |
| `src/replay/` | consumer updates (N2) |
| `src/validation/` | consumer updates (N2 `common.hpp`, N5b `runtime_access.cpp`); JSON stderr contract pinned (L5) |
| `src/legacy/` | consumer updates (N2, N5b) |
| `src/app/` | reviewed/no change |
| `src/configuration/` | reviewed/no change |
| `src/contact/` | reviewed/no change |
| `src/drivetrain/` | reviewed/no change (binding keywords already canonical) |
| `src/geometry/` | reviewed/no change |
| `src/kinematics/` | reviewed/no change |
| `src/model/` | reviewed/no change |
| `src/numerics/` | reviewed/no change (vendor/BLAS names exempt) |
| `src/pneumatic/` | reviewed/no change (`hub_position_world_m` is a real hub) |
| `src/random/` | reviewed/no change |
| `src/rider/` | reviewed/no change |
| `src/terrain/` | reviewed/no change |
| `src/tyre/` | reviewed/no change (project `tyre` spelling) |
| `src/viewer/` | diagnostic consumer (L3 HUD presentation evidence) |
| `src/viz/` | reviewed/no change |
| `include/bike/` | ABI contract exemption |
| `src/*.hpp/.cpp` root files | reviewed/no change except `binding*.cpp` exported-name exemptions |
| `native/tests/` | characterization/companion edits in each owning slice |
| `tools/proto_native_bench` | reviewed/no change this audit |

Generated/vendor trees (`.deps`, `native/build*`, `build`, package snapshots, reference checkouts)
are excluded by provenance, not by name.

## Diagnostic event ownership (G1)

One semantic owner per event; all human diagnostics flow through `io::write_diagnostic`
(`native/src/io/diagnostics.*`). Schema-bearing outputs are evidence, not human diagnostics.

| Event / output | Owner | Slice | Status |
|---|---|---|---|
| `physical.reference.warning` | `cli/ride.cpp` cold-path drain (advance end, seal/flush, pre-reset, pre-close) | L3 | fixed |
| `research.reference.warning` | `cli/research.cpp` cold-path drain (stop/finalization boundaries) | L3 | fixed |
| `research.policy.rejected` | `cli/application.cpp` `emit_failure_diagnostic`; fixed phrase + `error_code` + `field_path`, plugin `what()` never reaches the human stream | L1 | fixed |
| run `error` events (dispatcher) | `cli/application.cpp` `emit_failure_diagnostic` — one terminal event per captured failure | L1 | fixed |
| physical finalization secondary failures | `cli/physical_capture.cpp` `finalize_physical_run` → `FailureReport` notes; primary rethrown once, printed once by dispatcher | L2 | fixed |
| `batch.summary.malformed` | `cli/batch.cpp` — one warning per malformed expected research summary; missing optional summaries silent | L4 | fixed |
| sweep per-job `*.stdout.log`/`*.stderr.log` | `batch/sweep.cpp` via exclusive/no-follow `write_log` — evidence artifacts, not diagnostics | L4 | fixed |
| native-check `version` record / `tool_health` | `tools/check_frontends.cpp` — record appended to final `records`; raw stdout/stderr log paths preserved | L5 | fixed |
| alpha findings console marker | `tools/check_frontends.cpp` — single reporting point after all records formed; no loop+final double print | L5 | fixed |
| `bike-validate` JSON stderr envelope | `cli/` validator — machine channel, exempt from human formatter by design | C0/L1 | contract exemption |
| viewer HUD `[FIRST FAILURE]` | `viewer/` — presentation evidence, not a second operational event | L3 | contract exemption |
| recorded evidence (episode/recording schemas, result JSON, `sweep.json`) | owning serializers — separate channel, unchanged keys | C0/L4 | contract exemption |

Deferred with reason: Linux aarch64 verification gate (environment is macOS arm64 — deferred to CI);
narrow `readability-identifier-naming` AST rollout (separate agreed slice per plan G1); real GUI
smoke for viewer warning telemetry (headless session, presentation path audited only).
