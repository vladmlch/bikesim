# Task C331 — test-port 3.31 report (recording catalog, CLI, report, storage, values, analysis, analyst CLI)

**Batch**: 3.31 — `tests/reference/test_recording_catalog.py`,
`test_recording_cli.py`, `test_recording_report.py`,
`test_recording_storage.py`, `test_recording_values.py`,
`test_recording_analysis.py`, `test_recording_analyst_cli.py`
(66 records / 80 collected nodes; parametrized records expand the count).
CTest ids `reference.{recording_catalog,recording_cli,recording_report,
recording_storage,recording_values,recording_analysis,recording_analyst_cli}`;
labels `t53 recording`; port-map rows carry `owner_task: T53`.

**Lane**: `.worktrees/pfc-lane-a`, branch `lane/a`, base `2077ec72`
(merge of lane/e batch 3.30 recording lifecycle/retention/capture/
publication). Reuses the 3.30 `recording_test_support.hpp` seam, extended
with the 3.31 surface (8-arg `sample_row`, `energy_channels` overrides,
`value_case_rows`, `SyntheticCapture`/`synthetic_capture`, `sealed`,
`mutate_database` manifest resealing, `with_forces`/`forwarded`,
`object_ref`/`member_ref`/`wire_erase`, `throws<Ex>`).

## Status: landed — 65 `port`, 1 `drop`, 0 `planned`

65 of 66 records ported into seven new CTest executables. Every ported
check compares native state against native state, native contract text,
or literal structure — **no golden fixtures were needed or created**
(zero `_golden` / live-oracle comparison sites in the seven source
files; all recording content is authored synthetically through the
production encoders and sealed via `RunCapture`/`finalize`). One record
is `drop` (`python-only`): `test_exact_comparison_rejects_incompatible_
units_and_phases` — units/phases derive from `joint_type` inside
`build_catalog` and no native seam authors divergent field metadata
between two sealed recordings; the schema-coherence check is exercised
natively through the coordinate-identity record instead. No record
stays `planned`: the source suite is fully green (80/80) and no
live-oracle red family exists in this batch.

## Triage

| Source file | records | collected nodes | port | planned | drop |
|---|---|---|---|---|---|
| `test_recording_catalog.py` | 15 | 15 | 15 | 0 | 0 |
| `test_recording_cli.py` | 12 | 12 | 12 | 0 | 0 |
| `test_recording_report.py` | 1 | 1 | 1 | 0 | 0 |
| `test_recording_storage.py` | 11 | 11 | 11 | 0 | 0 |
| `test_recording_values.py` | 5 | 5 | 5 | 0 | 0 |
| `test_recording_analysis.py` | 16 | 30 | 15 | 0 | 1 |
| `test_recording_analyst_cli.py` | 6 | 6 | 6 | 0 | 0 |
| **total** | **66** | **80** | **65** | **0** | **1** |

## What the ports reproduce

- **`reference.recording_catalog`** (15 tests): finite fully-described
  table/field catalog, qpos/DOF coordinate addressing, legacy flat-alias
  → canonical resolution, R10 presence/exceptional-value round trip
  through real `EncodedTable` pieces and a DuckDB read-back,
  false/zero/empty/signed survival, nonfinite ≠ NULL, nullable-parent
  presence masks, empty-list vs absent, vector shape/axes/units,
  unknown-field named rejection, source-backed catalog metadata,
  `PhysicalSource` typed batches, per-component force explosion
  (qfrc vector + power), batch-argument/bad-interval rejection, malformed
  joint descriptors.
- **`reference.recording_cli`** (12 tests): `--record` default-off,
  `--out` never enables it, acceptance for physical-native/research,
  rejection for python backend / legacy physics / preview, check-policy
  resolution ordering and scope gating, interactive-only `time-scale`,
  self-naming unsupported combinations — all through `parse_arguments`
  + `resolve_ride` with effective mode read off
  `ResolvedRide::setup.model.physics->physics_mode` (`--physics-config`
  does not set `Arguments::text("physics")`).
- **`reference.recording_report`** (1 test): sealed recording →
  `render_summary` regeneration is byte-identical and mutates nothing
  (read-only + idempotent markers verified).
- **`reference.recording_storage`** (11 tests): typed nonfinite/presence
  round trip with owned asset, empty-run/zero-step publication, disabled
  capture no-op, publication collision not adopted, report-failure
  publishes valid DB with explicit status, manifest-last ordering with
  retryable evidence on failed link, unsealed/tampered payload
  rejection (missing manifest, checksum mismatch, symlinked payload
  refused via `O_NOFOLLOW`/`ELOOP`), resealed catalog tamper rejected
  before analysis, asset checksum verified before publication,
  multi-episode extents + isolated event journals, per-component forces
  round trip + qfrc-width rejection.
- **`reference.recording_values`** (5 tests): the reviewed optional-leaf
  / nullable-parent mask matrix (`NULL + leaf present`, `absent leaf`,
  `parent NULL → __valid cleared`, `absent parent → both masks
  cleared`), the nine-row nonfinite/signed-zero matrix, a resealed
  presence-flag contradiction rejected on open, per-batch export
  bounds with no legacy columns, `commands_applied` tolerating
  explicit-null optional drive.
- **`reference.recording_analysis`** (15 tests): recording-JSON boundary
  (NaN/±inf/−0.0/null/tag/`decimal`/`object`-escape literals), window
  start-inclusive/end-exclusive selection with `overlap_before_start`,
  bounded query truncation + full export totals, read-only SQL surface,
  clock-mismatch refusal vs summary availability, matching-nonfinite
  evidence not silently valid, report excluding forwarded extrema,
  first-mismatch stable keys, prefix-extent mismatch, incompatible DOF
  coordinate identity (`build_catalog(joints, scalar_aliases={})`),
  forwarded measurement provenance as incompatibility, exact numeric
  tolerance boundary (`|a−b| ≤ atol + rtol·max`), explicit-null vs
  absent, categorical-change reporting, summary-mode parameter
  changes without alignment.
- **`reference.recording_analyst_cli`** (6 tests): stable JSON envelopes
  on empty result sets, `RECORDING ERROR` diagnostics without
  tracebacks, episode-scoped multi-episode selection, bounded CSV
  export that refuses overwrite, nonzero exits for bad SQL / missing
  recordings, SQL truncation indicator — all through
  `bike::cli::recording_command` with captured streams.

## Confirmed traps found during the port

1. **`EncodedTable` is `{rows, columns}`**, not row-shaped — column
   `Piece`s carry numeric/string/bool lanes + `valid` presence bytes +
   list offsets; `_present` companion columns and `__<path>_present` /
   `__<path>_valid` parent masks live alongside.
2. **`io::Value` does not take raw scalars.** Numbers need
   `rec::value::real` / `rec::value::integer` / `text`; `Wire` explicit
   nulls are `Wire{nullptr}` (no `std::monostate` constructor).
3. **`PhysicalSource::tables(maximum, sink)` is eager** — unlike the
   oracle's lazy `tables()` generator it encodes every catalog table
   through the sink, so rows carrying partial channel structs trip the
   claimed root's required members (`channel_drive[0].drive.
   transmission_model: required`). Exercising only `commands_applied`
   still requires a storage-valid `channels.drive` (complete
   `drive_channels`) when a drive root is present.
4. **Window semantics**: `rec::window` `rows` are intervals *opening*
   inside `[start,end)`; an interval already open at `start` reports
   under `overlap_before_start` and the edge row carries
   `overlaps_end`.
5. **Exception surfaces**: `finalize` wraps inner failures as
   `std::runtime_error` ("…; owned evidence retained; unsealed
   staging"), symlinked payloads surface `std::system_error` (ELOOP
   via `O_NOFOLLOW`), reader/manifest faults surface `RecordingError`
   (an `invalid_argument`), and DuckDB `SELECT 1.25` returns a
   `decimal`-tagged value — `tagged_double` wants `float64`, so the
   SQL probe casts `CAST(1.25 AS DOUBLE)`.
6. **Divergent-catalog authoring**: `build_catalog(joints)` alone
   cannot build a renamed-DOF catalog — the default scalar aliases
   bind fixed joint names (`scalar alias coordinate missing: x_m`);
   the `build_catalog(joints, scalar_aliases={})` overload is required,
   matching the oracle's `scalar_aliases={}` keyword.
7. **Test-local helper names shadow `using namespace recording_test`**
   imports — same-name locals in the anonymous namespace hide
   `rec::value::{integer,real,text}`; the files now qualify value
   constructors explicitly and keep member accessors under distinct
   names (`field_int`/`field_real`/`field_bool`/`field_str`).

## Verification evidence

- **Oracle green (darwin-arm64)**: `uv run pytest
  tests/reference/test_recording_{catalog,cli,report,storage,values,
  analysis,analyst_cli}.py` — **80 passed** (38.88 s). The linux oracle
  run was not required: no record compares against the live oracle and
  no golden capture sites exist in this batch.
- **CTest darwin-arm64**: `ctest -R
  'reference\.(recording_|native_recording)'` — **11/11 Passed**
  (44.55 s; includes the 3.30 lifecycle/retention/capture/publication
  regression scope recompiled against the extended support header).
- **CTest linux-aarch64** (`bike-dev-pfc-lane-a`, committed tree synced
  via `git archive`): same scope — **11/11 Passed** (47.42 s).
- **Red→green proof (no goldens in batch → byte-flip N/A)**: genuine
  red→green iterations during development — window-selection semantics
  corrected to match the oracle (`rows == [1]` + `overlap_before_start`),
  presence-contradiction required a non-null authored residual, partial
  `drive` root tripped `channel_drive` required members. Asserts fire.
- **Static registration**: `Phase3Ports.cmake` — seven
  `bike_add_reference_test` entries, `LABELS t53 recording`.

## Ledger

- `migration/test-port-map.json`: 65 `port` (contract), 1 `drop`
  (python-only) — measured `collected_node_ids` (80 nodes),
  `assertions_capture` and per-record evidence for all 66 rows.
  Headers unchanged: `static_test_functions`=2391,
  `top_level_test_functions`=2375, records=2391.
- `migration/python-removal.json`: the seven source rows →
  `status: replacement_landed`, `verification_ids` set to the CTest
  ids; deletion stays gated at T63.

## Commits

| SHA | Unit |
|---|---|
| `483f4985` | 3.31 T53: native recording catalog/CLI/report/storage/values/analysis/analyst-cli ports — 65 port + 1 drop, darwin-arm64 green |
| (this commit) | ledger dispositions + linux-aarch64 qualification evidence, C3.31 report |
