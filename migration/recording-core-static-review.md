# T38–T42: recording core — static implementation review

Date: 2026-10-17. Scope: the recording and application wave requested by the user, specifically T38–T42. This note records the implementation and the static review of the source snapshot. **No project executable, compiler, build system, test, Python bootstrap, or source oracle was run.** The user's explicit prohibition on compilation and execution overrides the run/commit checkboxes in the migration cards. None of the checks below is a passing runtime gate; the existing source authorities remain present.

## Source authorities and native boundaries

The implementation was checked against the common specification, `migration/interface-contracts.md`, the recording/replay owner plan, and the declarations listed in `migration/cards/T38.md` through `T42.md`. Responsibilities are divided by ownership and dependency boundaries rather than by reproducing Python module names.

| Task / source cards | Native implementation and integration |
|---|---|
| T38 / F098 catalog | `native/src/recording/catalog.hpp`, `catalog.cpp`, `coordinates.cpp`, `catalog_model.cpp`; compiled full descriptor asset `catalog_data.inc` and inspectable `native/resources/recording/catalog-v1.json` |
| T38 / F099 immutable contracts | `recording/contracts.hpp`, validation and freezing at `capture.cpp`; encoded batch validation in `encoding.cpp`; owning physical/research source interfaces |
| T38 / F109 container schema | `recording/schema.cpp`; schema version 1 is independent of research replay version 3; typed streams, masks, full field descriptors, coordinate/alias catalog, views |
| T38 / F215 frozen forces | `recording::ForceSample` in `contracts.hpp` / `capture.cpp`; value-owned incoming state and components; power uses the incoming velocity |
| T39 / F100, F104 typed encoding | `recording/encoding.hpp`, `encoding.cpp`, `plan.cpp`, `duckdb_chunks.cpp`; existing `runtime/encode.cpp` remains the physical numerical projector |
| T40 / F096 assets | `recording/assets.cpp`; setup snapshot adapters in `cli/physical_capture.cpp` and existing `research/metadata.cpp`; captured bytes and final identity diagnostics are separate values |
| T40 / F097 capture | `recording/capture.hpp`, `capture.cpp`, `finalize.cpp`; metadata/event/episode value ownership, journal ordering, finalization state machine |
| T40 / F103 native source | `recording/physical_source.cpp`, `catalog_model.cpp`, `events.cpp`; owned episode batches, ordinary applied commands, derived event labeling and interval lookup |
| T40 / F108 research source | `recording/research_source.cpp`, `research_publication.hpp/.cpp`; existing research evidence summarization supplies completion and identity receipts |
| T40 / F187, F196 recorder integration | Existing `runtime/samples.cpp` and accounting retention provide physical owned rows/columns; existing `legacy/recorder.cpp` and `legacy/telemetry.cpp` retain legacy sampling/event behavior. CLI and T46 exports use those owned values. No new Python recorder or physical force computation is introduced. |
| T41 / F110 storage | `recording/database.hpp/.cpp`, `database_validate.cpp`, `publish.hpp/.cpp`; typed DB transaction and exclusive terminal publication |
| T42 / F101 interrupt | Existing `platform::InterruptShield`, `recording/interrupt.cpp`, frontend lifecycle critical section; deferred interrupt is delivered after publication status is committed |
| T42 / F145 async close | `runtime/async_close.hpp/.cpp`, accounting ownership changes in `runtime/accounting.hpp/.cpp`, boundary collection in `runtime/runtime.cpp`; configuration/projector/bootstrap/environment/CLI flag wiring |

The generic DuckDB ownership/conversion boundary `recording/duckdb.hpp/.cpp` and the read-only report generator belong to T43–T46. The core uses `Database`, `Connection`, and `render_report_database()` through those public interfaces. The CMake integrator registers the additional translation units and the thread dependency.

The final include review found that the old `runtime/encode.hpp` reached MuJoCo/controller headers through `samples.hpp`. The unchanged `Piece`/`EncodedTable` value definitions now live in `runtime/encoding_values.hpp`; the runtime projector includes that header, while generic recording encoding includes only these standard-library value types. Native plan/projector callers explicitly include `runtime/encode.hpp`. This removes that engine-header edge from the standalone analyst without duplicating a payload representation.

## Catalog and schema

The catalog asset contains **11 physical/catalog tables and 4 research tables, with 495 field descriptors counting keys and repeated nested declarations**. The descriptor transcription preserves order and every source descriptor property: type, unit, frame, phase, optionality, measurement meaning, source, description, path, aliases, shape, axes, exceptions, and nested children. It was produced by static syntax transcription; no source module was imported or executed to generate it. The C++ parser validates the compiled asset at use time.

The coordinate builder accepts model joint facts explicitly. Free, ball, slide, and hinge layouts retain their respective qpos/DOF widths. Coordinates are dense and ordered, units distinguish translational from rotational DOFs, and scalar aliases require a scalar coordinate instead of guessing an index. Alias resolution handles parameter templates and nested map/struct fields. MuJoCo inspection is isolated in `catalog_model.cpp`; catalog and SQL validation headers do not import the engine. The analyst include/link closure can therefore use catalog, coordinate and schema code without runtime/model construction.

Metadata tables retain the container identity, runs, episodes, events, complete field descriptors, per-episode coordinates and aliases, byte assets, integration-state pairs, and typed metrics. Stream columns follow reviewed catalog order. A metric integer such as 9007199254740993 uses a BIGINT cell, never a double intermediary. Metadata JSON remains the existing tagged, lossless native value codec.

## Encoding and typed append

Physical history uses `runtime::encode_table_native` with compiled plans. Generic value projection covers the reviewed primitive, vector, map and nested struct shapes; a future `map<bigint>` physical field can use the explicit source adapter instead of silently losing a field. There are no `map<bigint>` fields in the shipped physical descriptor set. Unknown roots, members, duplicate names, invalid types, shapes and bounds reject their batch.

Before DuckDB receives a chunk, recursive validation checks piece kind, scalar lane extent, validity extent, child/member sets, list/map offsets, int32 offset bounds, nested required fields, optional presence, root masks, null-parent consistency, and declared shapes. Slicing rebuilds owned recursive pieces and clears pointer caches; it does not retain spans into a released original batch. The optimized double/int/bool lanes are copied directly into DuckDB vectors. Numeric physical rows do not pass through JSON. Generic metadata nonfinite values use the existing tagged codec; arbitrary physical NaN payload bits are preserved by the optimized physical path.

Appending creates the reviewed logical types and nested vectors, reserves child extents, appends bounded chunks, flushes, closes and destroys the appender with explicit error handling. Public batches are limited by the configured maximum; internal chunks are additionally limited to DuckDB's vector size. Fault seams exist before append, appender flush/close/destroy, and the database transaction stages. The writer deliberately replaces a batch's episode key with the owning `EpisodeEvidence.episode_id`, matching source `schema.arrow_batch`; adapter generations are not assumed to equal launch episode IDs.

### Reviewed correction: empty routed root versus source storage validation

A concrete inconsistency exists between the source encoder and the source database validator:

* `src/bike_sim/recording/encoding.py::_distribute` does not assign an optional terminal root leaf when a claimed-root mapping has an empty routed remainder. Deeper-only members can also exhaust that remainder.
* `tests/reference/test_native_encode_columns.py::test_map_channel_tables_bitwise` explicitly specifies this case for `component_work_j={}`: the source root is present and valid, while the terminal leaf is absent and NULL. The existing native T11 encoder preserves that distinction.
* `src/bike_sim/recording/storage.py::_validate_presence` unconditionally equates the root and terminal-leaf masks. This rejects the encoder's own empty-root output. Empty rider control/intent and component maps are reachable runtime outputs, so reproducing that rejection would prevent actual recordings from being published.

The integrator reviewed and accepted the following narrow validator correction. An optional **claimed-root terminal field** with type `map<…>`, `struct`, or `struct[]` may have precisely:

| Root present | Root valid | Terminal present | Terminal value | Meaning |
|---|---|---|---|---|
| true | true | false | NULL | The source root exists; its routed terminal remainder is empty. |

The encoder is unchanged. No empty map or struct is synthesized. Normal parent checks still require valid ⇒ present; leaf checks still require absent ⇒ NULL. The exception does not apply to primitive roots, required roots, deeper leaves, or a null/invalid root. All other root/leaf equalities remain enforced. Both in-memory piece validation and SQL database validation use the same rule.

`test_t39.cpp` contains positive mask/NULL checks and rejects the same absent leaf after clearing root validity. `test_t41.cpp::empty_routed_root` writes the real encoder output, then rejects a corrupted root-valid mask. **These regression sources were authored but not run.** This is an explicit correction to the source validator's accepted-state domain, not a claim of bitwise parity with the erroneous rejection.

## Capture, setup assets and derived facts

`RunCapture` owns metadata values, immutable source histories, frozen summary values and byte assets. It snapshots metadata/assets even if a caller retains a mutable `shared_ptr` alias. Re-adding the identical original episode owner is idempotent; a conflicting object with the same ID rejects. Episode IDs are consecutive from the first committed nonnegative ID. Event identities increase lexicographically across reset-separated episodes; journals restart the event counter for each episode and validate receive/apply clocks.

Capture is opt-in. A disabled capture validates arguments and journal ordering but retains no growing event or episode history and does not create an output directory. Runtime retention remains independently configured; publication consumes owned sealed episodes without keeping live model/data/policy aliases.

Physical/research adapters retain immutable episode owners and emit bounded batches. Research transitions explicitly reference their requested action and previous/next observation; `application_id` is the schema's applied-command identity. Full capture is required for persistence, and replay eligibility also requires the complete control/physical suffix, finite matching integration states and the existing summary identity gate. An incomplete physical tail remains evidence without becoming a completed control transition.

Asset creation hashes the exact captured bytes. The setup adapters compare referenced rider-profile bytes to the already established setup hashes. `input_identity_at_finalization()` records each declared profile as unchanged, changed, or unavailable, retaining the setup hash and either the current hash or the read error. Those reads never replace the captured assets. The ordinary summary and the research publication adapter attach the resulting map. Existing research metadata supplies the broader executable/provider/configuration identity receipts.

Derived events are emitted only from retained rows, carry the source derivation rule/version, retain original receive and physical-interval clocks, and stay labeled `derived_transition`. They are not presented as source input events. The research publication adapter inserts them before the terminal episode/first-failure tail and reindexes the launch journal consistently.

## Database validation and final-only publication

The database writer creates a private, exclusive directory before letting DuckDB open a new payload. It never opens an existing requested destination. One transaction writes schema, metadata, assets and typed streams, adds views, runs relational validation and commits. It then checkpoints and closes. A rollback failure is secondary to the original failure; the diagnostic retains both. A surviving WAL prevents publication.

Validation checks the exact table set, schema/codec/run identity, stream column order/types, full field catalog, keys and orphan references, optional/root/nested masks, asset hashes, channel coverage, coordinate widths, continuous interval clocks and contiguous identities. It also checks research N/N+1 extents, positive completed step windows, request/observation references, observation delivery clocks, startup applied command, integration-state phases/signatures and replay-eligible suffix completeness. Ordinary command mirroring is accepted without claiming checked research replay.

The terminal publisher follows this sequence:

1. Create an owned 0700 staging directory and explicit unsealed marker.
2. Write, validate, commit, checkpoint, close and sync the DB.
3. Generate the report using the actual read-only reporting pipeline. A report exception becomes explicit `report_status=failed` and `report_error`; the valid database can still be sealed without a report payload.
4. Hash all successful payloads and exclusively write/sync the candidate manifest.
5. Exclusively create a fresh final destination, skipping existing files, directories and symlinks without adopting them.
6. Link payloads exclusively, verify opened regular-file/inode identities, rehash the destination payloads and sync directory names.
7. Link `manifest.json` **last** as the sole sealing operation.

There are no fallible validation steps after the sealing link. Postcommit directory syncing/cleanup is best effort and cannot turn a sealed success into a reported failure. Before the commit marker, any failure leaves unsealed staging/destination evidence and a retryable in-memory capture. A retry picks a fresh destination; a repeated successful finalization returns the cached path. RAM capture lost to a killed process is not claimed to be recoverable.

The path primitives validate opened regular files with no-follow semantics. Source and destination payload identities are compared before sealing. Qualification of the OS-specific reader descriptor path and crash/signal behavior is pending actual execution; this note does not assert resistance to arbitrary concurrent replacement of ancestor directories by a separate process with the same credentials.

## Real asynchronous telemetry publication (F145)

The previous unconditional `async_telemetry` rejection was removed only after adding an actual worker and boundary wiring. The flag controls period publication, rather than aliasing synchronous behavior.

`AsyncCloseRunner` owns a `std::thread`, mutex/condition-variable queues, immutable original raw periods, ordered result objects and a worker-local accounting evaluator. It receives only value-owned accounting state, raw samples and an `EffortContext` containing copied controller configuration/joint facts and dimensions. The worker never owns or touches live `mjModel`, `mjData`, runtime, controller or filesystem state. It runs the same period-close calculation used synchronously, with the same numerical statement order.

The stepping owner submits a complete period and collects ready results at a subsequent boundary. A slow worker can expose older accounted observations; this is an actual delayed mode and must not be described as trajectory-equivalent for policies that consume such observations. At most two periods remain unacknowledged before the next submission drains, so recording-off mode cannot accumulate unbounded raw transport history.

Main-thread application stages throwing state/diagnostic copies and storage reservations first. State, sample retention and diagnostics commit before queue acknowledgement. Strict reference failure is transported with its already committed samples and original failure time, then thrown after acknowledgement. Pending warnings are merged so asynchronous scheduling neither repeats nor loses an unconsumed warning. Ordinary worker calculation failures propagate as `AsyncPublicationError`; they do not silently switch calculation modes.

Lost-worker transport can detach and replay every unacknowledged original from the main owner's last committed state. Detachment joins the worker before releasing its owners. Replay advances its cursor only after a successful main-thread commit, so failure remains retryable without duplicate history. Explicit flush/seal/reset boundaries drain the partial tail and pending results. Reset commits the new generation only after the old accounting path drains; close destroys/joins accounting before dependent runtime/controller state. Close is idempotent and never leaves a detached thread. Polymorphic exception/source destructors are out of line for the repository's strict Clang vtable policy.

## Authored regression sources and remaining qualification

| Source | Meaningful contracts represented; **not executed** |
|---|---|
| `native/tests/migration/test_t38.cpp` | Full table/field inventory; acquired welded catalog structure; aliases; free/ball dimensions and units; invalid coordinate/address/descriptor domains |
| `test_t39.cpp` | Absent/null/empty distinctions; large integers; nested values; slices/fragments; nonfinite physical bits and signed zero; malformed offsets/members; empty routed-root acceptance and corruption rejection |
| `test_t40.cpp` | Frozen metadata/assets; idempotent versus conflicting owners; episode/event identity and apply clocks; disabled history; ForceSample incoming powers/domains; current profile unchanged/changed/unavailable |
| `test_t41.cpp` | Typed persisted counts/metrics/BLOBs; absent-root coverage; exclusive refusal of an existing DB; masks/hash corruption; zero-row episode; real empty routed-root output |
| `test_t42.cpp` | Twenty-four injected publication failure stages; no premature sealing; retained evidence and retry; report failure; concurrent finalizers; existing/symlink destination collisions; authored fork/SIGKILL and repeated-SIGINT cases |
| `async_close_contract.hpp` | Synchronous/worker publication values and signed zero; partial tail; sealed owner survives acknowledgement; detach exact replay; strict failure time and committed rows; injected worker failure; retained original owners and idempotent close |
| `recording_fixture.hpp` | Shared small valid evidence/publication fixture for T38–T46; synthetic owned rows, deliberately absent roots and typed metadata; no simulator execution is hidden in the fixture |

Pending gates require explicit future authorization to run project code. They include compiler/strict-warning qualification of all added translation units, the pinned DuckDB 1.5.6 C API and typed nested append/read compatibility, native regression executables, source-specific acquired oracle comparisons, articulated-effort asynchronous numeric parity, OS-specific no-follow/descriptor opening, real repeated-signal and child-kill publication tests, and broader integrated CLI/replay/application acceptance. The snapshot does not contain the pinned DuckDB SDK headers, and no dependency was acquired or build configured during this work.

This source review is implementation evidence only. It does not close migration execution gates, authorize old-code deletion, or claim a clean build.
