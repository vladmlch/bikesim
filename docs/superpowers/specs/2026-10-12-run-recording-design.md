# Run recording for post-run analysis

Status: approved design. Source implementation for ordinary native and both research backends is supplied in the 2026-10-13 delivery; see [source-completion and requirement evidence](../audits/2026-10-13-run-recording-source-completion.md). Compilation, tests, GUI, simulations and performance measurements are NOT RUN under the explicit execution prohibition. This specification does not claim runtime acceptance.

## 1. Outcome and authority

An agent working in the checkout must be able to investigate why a ride stopped, rolled backwards or crashed; explain recorded controller decisions; examine forces, contacts, work and validity; and compare experiments. It has shell, Python and SQL access. The primary workload is individual short runs and small comparison sets on a machine with ample RAM.

The final user decision is **collect in RAM and write at the end of the entire launch**. It supersedes the earlier discussion of periodic commits, bounded writer queues and crash-readable on-disk prefixes. A hard process failure before finalization loses the in-memory recording. No incremental persistence, background database writer or disk-I/O pipeline is required.

Implementation is ordered: ordinary native physical ride first; research recording and replay migration second. Both stages are required. Existing ordinary Python execution remains supported, but adding its new recording adapter is outside stage A. Research stage B preserves its existing Python and native backends.

Normative requirements are R01–R24 below. The accompanying master plan maps every requirement to tasks. These remain the normative implementation contracts. Current source-level behavior and usage are documented in [RECORDING](../../RECORDING.md); runtime acceptance is tracked separately.

## 2. Global constraints

- Invoke Python and Python tools through `uv`.
- Use Python >=3.13, C++23, nanobind, and MuJoCo 3.12.0 for native execution.
- Preserve valid-input binary64 operation order, physics timestep, controller/sensor scheduling, and existing comparison tolerances.
- Store physical samples at every accounted physics interval when recording is enabled; do not downsample or quantize the canonical recording.
- Collect the whole launch in RAM and write only during launch finalization.
- Make persistent recording opt-in with `--record`, default false.
- Keep database access, Python boxing, report generation, and disk I/O outside realtime kernels.
- Preserve user-owned changes, unrelated plans, skills, and output; do not delete or overwrite an existing run.

Read [native engineering rules](../../agents/native-engineering.md), [TESTING](../../TESTING.md), and current `AGENTS.md` before implementation. The specification does not reopen unrelated native-safety work.

## 3. Vocabulary and lifecycle

**R01 — Run identity.** A run is one CLI launch, with a UUID, UTC start timestamp and a monotonic wall-time origin. A recording contains one run. Configuration hashes identify configuration, never a run. An episode is a reset-separated physical timeline within that launch. Its key is `(run_id, episode_id)`; native generation is retained separately. Interval and step counters are local to an episode. A reset may reuse time zero but never an episode identifier.

**R02 — Opt-in.** Add `--record` to physical `bike-ride`, `bike-ride --research` and `bike-research`. Without it, retain live presentation, accounting, checks and the console outcome, but avoid accumulating optional full histories and writing ordinary/research recording bundles, preview CSVs or recording-derived plots. Necessary sensor delay queues, controller state and running metrics remain available. Keep explicit programmatic legacy `save()`/`export()` compatibility until the replacement adapters are verified; a caller explicitly requesting a save has opted in. Legacy nonphysical ride modes retain their current behavior; reject the new flag on an unsupported mode/backend before stepping, naming the unsupported combination.

For stage A, `--record` is supported for ordinary physical native headless/viewer. Stage B adds both research backends and both research entrypoints. Ordinary physical Python plus `--record` reports a clear unsupported-combination error until separately implemented; ordinary Python without it continues to run. Do not route it silently to native.

**R03 — Finalization.** All episodes, including those ended by viewer reset, stay in RAM. Headless completion, viewer close, Ctrl-C and a caught simulation/policy error trigger one launch finalization. A terminal physical state while its viewer remains open finishes the episode in RAM; publication occurs when the launch closes. A reset closes/captures the outgoing episode before resetting its owner. Existing research rules for finishing an active control window remain authoritative; finalization must not request another policy action.

**R04 — Failure evidence.** Capture the latest owned committed prefix, completed-control prefix, primary failure, cleanup notes and attempted outcome. A caught engine failure may leave physical evidence beyond the last completed research transition: store both extents explicitly. Never synthesize a successful transition for that suffix. Error handling must not mask the simulation's original exception or exit classification with a secondary export error. A successful simulation whose requested recording cannot be published exits nonzero and reports the publication error. Finalization is idempotent; a retry cannot duplicate rows or simulate more work.

Abrupt process termination, a native fatal signal, power loss before finalization and allocation failure that prevents any usable export are not durability guarantees. Large retained memory is acceptable for this workload. Exhaustion raises an explicit error rather than silently dropping samples.

## 4. Recording as observation

**R05 — Passive capture.** For fixed inputs, seed, measurement settings and stop policy, recording enabled/disabled produces the same committed physical trajectory, commands, sensor sequence, validity and terminal outcome. A real-time wall-clock cap is the documented exception: capture overhead can consume more wall time. Test numerical passivity with an explicit physical duration and nonbinding wall cap. User interaction tied to wall time is not expected to produce identical commands; compare a prerecorded step-indexed input sequence instead.

**R06 — Separate measurement and retention.** Current `record_decimation` also gates expensive mass/energy measurement (`PhysicalStep` in `native/src/runtime/step.cpp`, Python `physical_runtime.py`). Split this into independent measurement scheduling and evidence retention. Recording opt-in never changes the configured schedule of authoritative checks. Preserve the old defaults for those checks and the legacy argument as a documented compatibility mapping to measurement scheduling; record its effective meaning. Every committed physical interval is retained when recording is on, independently of that schedule.

Full capture means every interval and all available diagnostic evidence, not a fabricated measurement on an unmeasured step. Record per-field/group measurement status and original source interval/time when the existing algorithm carries a value forward. Extra measurements may be added only as diagnostics isolated from plant/control/check decisions, with passivity demonstrated. Do not present forwarded or unmeasured values as fresh instantaneous measurements.

**R07 — Ownership and transaction safety.** Native stepping retains owned C++ samples and compact event facts; it does not cross into Python each physics step. Native consumers share sample payloads and one canonical retained sample index. Boxing and typed ingestion happen at an external seam or finalization. Completed batches survive reset/close. Prepare owned export values before acknowledging/releasing their delivery; preserve retry behavior on conversion/allocation failure. Runtime advance, accounting and control acknowledgement keep their existing exception-safety contracts. Store an episode's metadata and owned state before destroying/resetting its owner.

Current `PhysicalRideDriver.advance()/flush()` discard pending delivery; current research has a second recorder on top of accounting. The plans replace these retention paths deliberately rather than attaching a late callback to already released data.

## 5. Evidence contract

**R08 — Physical evidence.** For every accounted interval retain integer ID, start/end/dt, incoming `qpos` and `qvel`, component generalized-force vectors and corresponding powers, tire/drive/suspension/rider/control channels, contact state, solver/energy diagnostics, validity and available decision diagnostics. Generalized forces currently exist in memory but are absent from the legacy `as_dict()` representation; expose them through the new recording adapter. Reuse immutable payloads rather than deriving forces from rounded exported powers. Numerical values stay binary64, identifiers stay integers, categories stay strings and booleans stay booleans. Do not retain aliases as additional canonical columns.

**R09 — Field catalog.** Each canonical field has a stable key, DuckDB type, description, unit, physical frame/reference, phase, authoritative source location, optionality and measurement semantics. The catalog includes vector shape, axes and model-specific qpos/DOF address/name mappings. Mixed generalized-force units follow the DOF map. Explain incoming state, solved-before-endpoint sensor values, endpoint values, sensor acquisition time, observation delivery time, crank-side versus wheel-side torque, and command ceiling versus target. Unknown fields fail a schema-completeness check; they are not silently omitted or assigned guessed units. A physically inapplicable unit is explicitly marked dimensionless/not applicable with its meaning.

**R10 — Presence and exceptional values.** Distinguish missing key, explicit null, false, zero, empty vector, NaN and signed infinities. Optional typed values use a corresponding presence bit/boolean; nullable parent objects have presence too. Floating NaN stays a floating NaN, not SQL NULL. JSON used for metadata has an explicit tagged representation for nonfinite leaves; it is not the numeric source of truth. For typed scalar/vector payloads use DuckDB numeric/list/struct types and null validity. Preserve signed zero in canonical storage and test a round trip. Alias resolution is catalog metadata or a SQL view, not duplicated samples.

**R11 — Events and commands.** Retain ordered source events with `(episode_id, event_id)`, source kind, received monotonic time, processed/applied step/time and payload. Ordinary viewer input records preserve raw request order, including toggles that cancel before a step, reset, brake strength changes and playback changes. An input request and effective applied demand are distinct; rollback-hold composition must not erase the requested value. Headless startup/commands use the same model without fictitious keyboard inputs.

Capture first violations, validity transitions, crash causes, contact transitions, gear shifts, coasting and assistance/limiter changes where supported by source. A controller decision event includes the actual selected branch/gates and thresholds when instrumented. Offline state differences are marked `derived_transition` and identify their derivation rule; they never claim an unavailable internal reason. Keep source evidence and later analysis interpretations distinct. Wall/presentation events do not affect physical clocks.

**R12 — Provenance and assets.** Retain requested and resolved configurations, seed, terrain, effective timestep/periods, stop/check policies, limits, source revision/dirty/source digest, extension/library/build identity, runtime versions and model limitations. Bundle recipe assets as bytes with media type and hash, including terrain/track and rider parameter inputs required by replay. Snapshot required asset content when its identity is established; at finalization also record whether source/artifact identity changed. Do not reread changed external files and relabel them as original inputs. Ordinary recordings support rerunning a recipe, without promising deterministic checked replay.

**R13 — Research streams.** Store requested commands, applied command events, initial and subsequent policy observations, completed transitions, evaluation truth, episode metrics and initial/final integration state. Keep physics, control, sensor-acquisition and observation-delivery clocks explicit. Preserve sensor noise/bias/dropout draw ordering, latency, actuator delay, program composition and the policy's access to sensor observations only. Do not pass privileged truth into a policy. Recording captures this experimental semantics; it does not merge research orchestration into ordinary ride orchestration.

## 6. In-memory and database interfaces

**R14 — One launch owner.** New `bike_sim.recording` owns a `RunCapture` that retains completed `EpisodeEvidence` values plus ordered input events. Its enabled/disabled policy is established before stepping. Ordinary native and research adapters attach owned native evidence; the Python research adapter attaches its owned values. A native episode may be sealed into a shared owned backing before reset, with lazy boxing during finalization. A replay verifier can request transient bounded/suffix evidence even when persistent recording is disabled; it must not repeatedly copy the entire applied history.

The shared interface is:

```text
RunCapture(metadata: RunMetadata, *, enabled: bool)
RunCapture.event(event: InputEvent) -> None
RunCapture.add_episode(evidence: EpisodeEvidence) -> None
RunCapture.finalize(out_root: Path) -> Path | None

write_recording(capture: RunCapture, out_root: Path) -> Path
open_recording(path: Path) -> RecordingReader
```

Disabled `finalize()` returns `None`; enabled finalization returns the sealed run directory. `add_episode` validates episode identity/prefix continuity before commit and is idempotent for the same owned evidence, rejecting conflicting repeats. These are small interfaces over capture, schema, publication and readers; frontends do not issue SQL or manage file sets.

**R15 — Storage.** Use DuckDB as the canonical persisted payload, with Python Arrow ingestion of typed batches at finalization. Add `duckdb==1.5.6` and `pyarrow==26.0.0` to project dependencies and update `uv.lock` during implementation; imports are lazy outside execution kernels. These were the scratch-research versions; verify installation with the project's Python and supported platform before committing the dependency change. There is no native DuckDB/C++ dependency. Process retained data in batches without first boxing the whole run into a giant list of Python dictionaries.

Logical schema version is `1` for the new recording container; this is independent of legacy physical schema `2` and checked replay contract versions. Core tables:

| Relation | Key and role |
|---|---|
| `recording_meta` | One row: container/writer versions and integrity/coverage declarations |
| `runs` | `run_id`; provenance, configuration, capture/check policies and timestamps |
| `episodes` | `(run_id, episode_id)`; generation, outcome, validity, physical/control/replay extents, finalization status |
| `physics` | `(episode_id, interval_id)`; integer keys, interval times, qpos/qvel and measurement provenance |
| `forces` | `(episode_id, interval_id, component)`; generalized-force list, crank/DOF unit mapping and scalar power |
| `channel_<domain>` | `(episode_id, interval_id)`; typed canonical leaves/structures for drive, tires, suspension, rider, energy, physical sensors and model status |
| `events` | `(episode_id, event_id)`; source/derived kind, time/step association and typed core payload |
| `commands_requested`, `commands_applied` | Ordered command IDs and research application times; ordinary commands use the same distinction |
| `observations`, `transitions` | Research control-step keys, acquisition/delivery times, typed observations/truth and observation references |
| `field_catalog`, `coordinate_catalog` | Field schema/meaning/aliases, optional-presence columns and model coordinate mapping |
| `assets`, `integration_states` | Named hashed bytes and typed initial/final state vectors |
| `metrics` | Episode-scoped named typed metrics and definition/version |

`channel_<domain>` tables are a finite schema generated from the reviewed field catalog. Store typed fields directly, not one key/value row per numeric cell. Presence columns cover optional leaves and parents. Research observations are stored once and referenced by transitions; analytic views reconstruct convenient flattened traces. The `telemetry` SQL view joins physical domains with unambiguous catalog names. A separate compatibility view/export supplies historical aliases. Foreign keys/logical integrity checks reject dangling episode, observation and command references. No automatic retention deletion is introduced.

**R16 — Publication.** At finalization create an exclusive staging directory under `--out`, build `run.duckdb` with all episodes in a transaction, validate counts/coverage/relationships, commit, checkpoint and close. Generate the report from that closed database. Compute its hash and write a manifest identifying the fixed container version, run UUID, database hash and optional report hash/error. Reserve a unique final directory with exclusive creation; move the staged files there and publish `manifest.json` last. A directory without a valid manifest is an incomplete publication, never a successful recording. Readers verify the payload hash before querying. Never overwrite another run, including an empty pre-existing destination.

If report rendering fails after the database is valid, publish the database with `report_status=failed` and its error in the manifest; report regeneration is possible. If database construction/publication fails, keep the owned capture for retry while the process lives, expose the primary export error, and leave any incomplete owned staging/publication directory clearly unsealed. Do not delete unrelated destination files. This is final-export integrity, not incremental run durability.

## 7. Agent-facing analysis

**R17 — Report.** Generate `summary.md` containing invocation/provenance, outcomes and extents by episode, validity/first failure, important event times, key metric extrema/integrals, field coverage and limitations. Link each finding to an episode/interval/event or a reproducible query. Numerical invalidity is not physical calibration evidence. Report unavailable diagnostics explicitly. Build this deterministically from data, without invoking an LLM or inferring a cause as fact.

**R18 — CLI.** Add `bike-recording` with these operations:

```text
bike-recording overview RUN [--json]
bike-recording fields RUN [--pattern PATTERN] [--json]
bike-recording events RUN --episode ID [--kind KIND] [--json]
bike-recording window RUN --episode ID --start S --end S --fields FIELD,... [--json]
bike-recording compare LEFT RIGHT --left-episode ID --right-episode ID --mode exact|summary [--fields FIELD,...] [--atol N --rtol N] [--json]
bike-recording sql RUN --query SQL [--limit N]
bike-recording export RUN --episode ID --format csv|jsonl --output PATH
```

`RUN` is a sealed run directory. Querying and report generation open the database read-only. Default row output is capped at 200 with an explicit truncation marker and total/more indicator; `--limit` can raise it. Window bounds are simulation seconds, half-open `[start,end)`, selected by interval start; include exact interval IDs and boundary-overlap information rather than silently interpolating. Field lookup and default compact domain selections avoid printing 900 columns. Human output and JSON share the same typed result model; JSON explicitly tags nonfinite values.

**R19 — Comparison.** `exact` requires compatible time grids, field units/phases, coordinate identities and measurement provenance. Compare integers/categories exactly and floats with explicit reported tolerances, default `atol=rtol=1e-9`; report first mismatch, maximum deviations, unmatched extents and validity. Matching NaNs are classified as matching invalid evidence, never proof of a valid physical result; null/presence mismatches are separate. Do not silently interpolate or align different coordinates. `summary` compares outcomes, metrics, event counts/times and surrounding windows for parameter-changing experiments. Automatic distance alignment is outside this version because rollback makes it ambiguous.

## 8. Research migration and compatibility

**R20 — Preserve checked replay.** New research recordings use `run.duckdb` and the common manifest, with replay contract version `3` stored in recording metadata. `bike-replay RUN --episode ID` selects a new recorded episode; omission selects its sole episode or errors when ambiguous. Legacy episode directories with replay schema `1`/`2` keep their current reader/checker and backend identity semantics. Never rewrite or upgrade an old recording in place. An explicit exporter can produce legacy analytics, but normal new recording does not duplicate CSV and JSONL histories.

Legacy compatibility is format compatibility: historical data remains readable under its schema; physically replaying it still requires the recorded source/runtime/artifact identity. Modifying the implementation does not authorize bypassing the old source-hash check. Current-identity legacy exports exercise end-to-end legacy replay; historical fixtures can exercise parsing/hash validation independently.

The version-3 reader exposes the same logical inputs to `ReplaySession`: recipe/assets, backend/source/runtime identity, initial state, requested/applied commands, observations/transitions, metrics, outcome and final state. It verifies schema, hashes, required assets, extents and completeness before building a plant. Policy metadata remains descriptive data; it is not imported/executed. Preserve recorded-backend selection, each transition check, observed/applied sequence checks, final `1e-9` state tolerances and source-change rejection. Saved failure evidence remains readable for analysis but cannot be advertised as completed successful replay.

**R21 — Optimize at the shared seam.** Remove redundant native research sample/column retention after a common immutable episode view replaces it; sample payload sharing must remain explicit. Export each history once per finalization and reuse typed batches for projections. Replay consumes only newly applied events through a cursor/suffix interface, instead of copying the complete growing vector at every control boundary. Retain initial/final state and the precomputed sensor noise sequence as needed by existing semantics. Changing RNG generation, controller scheduling or physical models is a separate project.

**R22 — Stop policy.** Preserve current ordinary headless/viewer default check policies. Add an explicit `--check-policy strict|diagnostic` for ordinary physical and research entrypoints, resolving to existing stop-on-model-violation settings without changing fatal engine-error handling. Diagnostic continuation records sticky invalid status and original first-failure time. Research defaults and recorded policy remain reproducible. Report the resolved policy in every recording; the flag is independent of `--record`.

## 9. Acceptance and evidence

**R23 — Verification.** Stage A and B each require focused regressions plus the repository `full` profile for final native/realtime integration. Current `tools/run_tests.sh full` includes native preflight; verify that integration again at execution. Use the `native` profile for native-only intermediate changes and `quick` for isolated Python storage/analysis work. Missing mandatory analyzers are installed, not skipped. Keep simulations and build/test profiles sequential. Green means final required profiles pass against the selected artifact, not a document checkbox or scratch benchmark.

Required regression families: on/off trajectory/check/sensor passivity; complete consecutive physical capture; retention across reset/close; failed boxing/allocation retry; requested versus applied control ordering; types/presence/nonfinite/vector round trips; all known field catalog coverage; empty/error/interrupt/multiple-episode exports; publication failures and no-overwrite; bounded query output and read-only access; exact versus summary comparisons; old and new research replay, wrong source/artifact/hash/assets, incomplete windows and failure evidence. No in-flight disk recording is expected or tested as a feature.

**R24 — Performance and delivery.** Measure capture-on/off wall time, peak RSS, final export time, output size and representative queries separately on a short fixed-input native Release rollout at the effective profile timestep. Record dataset, compiler/engine/dependency identity and memory/CPU conditions; do not change the configured timestep for a favorable comparison. Increased memory and slowdown are permitted; silent omissions are not. Existing scratch timing is exploratory numerical-storage evidence, not an integration acceptance threshold.

Deliver stage A, then stage B with reviewable checkpoints and source/test evidence. Keep every unrun or failed required check visible. The planning task only writes these documents; implementation begins under an execution task.

## 10. Source and research references

Planning snapshot: HEAD `c5dafe06b5ce4c4f0b5020586eeec40a6973cad3`, tracked tree clean when inspected; unrelated untracked skills/docs/output exist. Recheck at execution, since the checkout advanced during discussion.

- Ordinary orchestration/export: `src/bike_sim/sim/ride/physical_driver.py`, `physical_frontend.py`, `physical_session.py`.
- Native ownership and evidence: `src/bike_sim/native/contracts.py`, `runtime.py`; `native/src/runtime/accounting.*`, `samples.*`, `runtime_binding.cpp`.
- Measurement coupling: `native/src/runtime/step.cpp`, `src/bike_sim/sim/ride/physical_runtime.py`.
- Research: `src/bike_sim/native/research.py`, `recording.py`; `src/bike_sim/sim/research/environment.py`, `viewer.py`, `policy_session.py`, `replay.py`, `replay_session.py`; `native/src/runtime/research.*`, `recording.*`.
- Library references: `/Users/vladislav.molchanov/Desktop/Projects_downloaded/mujoco/include/mujoco/mjtype.h` (`mjSTATE_INTEGRATION`), `src/engine/engine_support.c` (`mj_getState`/`mj_setState`); `/Users/vladislav.molchanov/Desktop/Projects_downloaded/numpy/numpy/_core/_add_newdocs.py` (owned/read-only arrays), `numpy/lib/_format_impl.py` (`allow_pickle=False`). An integration vector alone is not a universal controller/RNG checkpoint.
- DuckDB [transactions](https://duckdb.org/docs/current/sql/statements/transactions), [file-format tradeoffs](https://duckdb.org/docs/current/guides/performance/file_formats); Arrow [IPC](https://arrow.apache.org/docs/python/ipc.html), [typed columnar format](https://arrow.apache.org/docs/format/Columnar.html).
- Scratch exploration: `/private/tmp/bike_storage_bench.py`, `/private/tmp/bike-storage-bench-results/summary.json`. The reproducible measurement description is preserved in the master plan, so these temporary files are not execution dependencies.

The user's example names `examples/research/viewer_physics_distributed.toml`, which was absent at this snapshot. Use a user-supplied existing supported profile for acceptance. `viewer_physics_welded.toml` is an available source-backed reference, not an assertion that it is the same model or a silent substitute for the missing file.
