# Task C332 — test-port 3.32 report (replay, session, manifest, playback, view)

**Batch**: 3.32 — `tests/reference/test_replay_manifest_v2.py`,
`test_replay_recording_v3.py`, `test_replay_session.py`,
`test_native_replay_recording_v3.py`, `test_native_replay_session.py`,
`test_pedelec_replay_contract.py`, `test_native_view_contracts.py`,
`test_native_viewer_snapshot.py`, `tests/test_replay_playback.py`
(51 records / 85 collected nodes; parametrized records expand the count).
CTest ids `reference.{replay_manifest_v2,replay_recording_v3,
replay_session,native_replay_recording_v3,native_replay_session,
pedelec_replay_contract,native_view_contracts,replay_playback,
native_viewer_snapshot}`; labels `t53 replay`; port-map rows carry
`owner_task: T53`.

**Lane**: `.worktrees/pfc-lane-d`, branch `lane/d`, base `1601b5a7`
(3.27 merge-back). No merge, no push.

## Status: landed — 48 `port`, 1 `drop`, 2 `planned` (snapshot parity reds)

48 of 51 records ported into eight new reference CTest executables plus
one integration executable, all over the shared
`replay_reference_support.hpp` fixture layer (real native research
environments, sealed `run.duckdb` containers, schema-2 legacy exports,
checksum-honest resealing, execution-identity patching and semantic
tampering). One record is `drop` (`python-only`): the
`BIKE_NATIVE_BUILD_PATH` nanobind artifact-selection check has no
counterpart in a statically linked build — recorded-identity rejection
is covered by the foreign-identity port. Two records stay `planned`:
`test_native_viewer_snapshot.py`'s cross-oracle parity halves
(preview_row/endpoint/integration_state vs the live Python HUD oracle)
— the native-side ownership and pacing-slice halves landed and are
green on both platforms; the parity assertions remain the batch's two
documented reds pending two-platform goldens.

## Triage

| Source file | records | collected nodes | port | planned | drop |
|---|---|---|---|---|---|
| `test_replay_manifest_v2.py` | 10 | 30 | 10 | 0 | 0 |
| `test_replay_recording_v3.py` | 6 | 6 | 6 | 0 | 0 |
| `test_replay_session.py` | 7 | 9 | 7 | 0 | 0 |
| `test_native_replay_recording_v3.py` | 3 | 3 | 3 | 0 | 0 |
| `test_native_replay_session.py` | 4 | 4 | 3 | 0 | 1 |
| `test_pedelec_replay_contract.py` | 5 | 8 | 5 | 0 | 0 |
| `test_native_view_contracts.py` | 9 | 9 | 9 | 0 | 0 |
| `test_native_viewer_snapshot.py` | 2 | 2 | 0 | 2 | 0 |
| `test_replay_playback.py` | 5 | 5 | 5 | 0 | 0 |
| **total** | **51** | **85** | **48** | **2** | **1** |

## Implementation change

`native/src/replay/legacy.cpp` — `load_legacy_input` now validates the
`identity_schema=2` `execution.build_context` record end to end: exact
object keys at every level, normalized platform machine tokens,
nonempty libc/os_build/python identity strings, per-backend
BLAS/LAPACK integer ABI (ACCELERATE 32/64, NUMPY_OPENBLAS 64/64),
`file` vs `apple-system` library identities matched to the numerics
backend (file → sha256 + null build_id; apple-system → null sha256 +
nonempty build_id), and rejection of impossible Darwin↔Linux
platform/provider combinations — a transported recording's
platform/numerics pair must stay self-consistent with a qualified
host pair.

## What the ports reproduce

- **`reference.replay_manifest_v2`** (10 records / 30 nodes): the full
  schema matrix (v1/v2 × python/native backend × versioned), every
  authenticated schema-2 file and the envelope/strength JSONs,
  six unlisted-field/path mutations, the schema-1 checksum-set gate,
  summary↔bundle cross-checks (missing bundle, digest, execution,
  malformed physics rows), leaf-symlink rejection, strict-JSON
  duplicate/nonfinite rejection, exact-shape execution records for
  both backends, the v2 nullability schema, and the stable
  native-source manifest with tracked headers.
- **`reference.replay_recording_v3`** (6): container input verification
  before report, explicit multi-episode selection, ordinary-recording
  rejection (no checked-replay claim), recorded-policy never loaded,
  checksum-honest final-state tamper failing at the last boundary, and
  checksum failure preceding plant construction.
- **`reference.replay_session`** (7): headless/incremental report
  parity, early close never finishing a pending replay, last-frame
  unverified until final state comparison, no-policy loading, three
  semantic tampers (commands_applied step, transitions physics_steps,
  observations time_s) that reseal honestly yet still fail
  verification, identity mismatch rejected before plant construction,
  and identity recheck failure advancing no physics.
- **`reference.native_replay_recording_v3`** (3): v3 + suffix capture,
  failed recordings staying analyzable and frame-playable without
  being replayable, and playback never touching artifact/identity
  machinery.
- **`reference.native_replay_session`** (3): same-backend replay +
  restart generation growth with snapshot independence, the recorded-
  identity gate at construction and terminal boundaries, and
  foreign-identity rejection in the session constructor.
- **`reference.pedelec_replay_contract`** (5): `open_loop_schedule`
  rejecting at 0.0/1.5/3.0/5.0 s, `run_replay`'s
  `unsupported_experiment` report shape (empty rows, no learning-valid
  support evidence), the real `bike-validate rider-replay --mode
  open-loop` subprocess exiting 1 with the retained error, and
  `fidelity_sweep`'s `motor_ramp_grade` initialization rejection with
  every series row an `initialization_failure` and `passed == false`.
- **`reference.native_view_contracts`** (9): views as `std::span<const
  double>` read-only windows of engine widths (Python's refcount
  retention ports as the const-view/scope contract — documented),
  live-window bitwise tracking across `set_state`+`forward` against a
  parallel mj_forward reference, `efc_force` arena-relocation snapshot
  ownership, prepared-storage generation/capacity counters (never
  addresses), the non-geometric nullptr report, and the drive-core
  oracle dispatch's closed kind set / unknown operation / installed-
  writer rejections before any model mutation.
- **`reference.native_viewer_snapshot`** (2 planned reds): owned-side
  halves landed — retained frames survive replica mutation, reset and
  close; pacing yields latch the window without duplicating requests or
  transitions; stale targets reject non-strictly. The cross-oracle
  preview/endpoint/integration-state parity halves stay planned red.
- **`reference.replay_playback`** (integration, 5): real-container
  playback load (strictly increasing times, nq/nv widths, endpoint
  pose exactness), midpoint interpolation with linear dofs exact to
  1e-12 and quaternion blocks normalized, bad-directory rejections,
  `bike-replay <dir> --view --dry-run` exiting 0 with `frames=` output
  and 1 for a missing recording, and the default checked path failing
  nonzero on bare/missing invocations while verifying a sealed
  container to 0.

## Port-only support

`native/tests/reference/replay_reference_support.hpp` —
`roundtrip_request()`/`make_environment()` (lumped-rider physical
research runtime), `record_container`/`record_failed`/`record_ordinary`
publications, `record_prefix` schema-2 legacy export, `reseal` +
`patch_container` digest-honest edits, `patch_execution` container +
recipe-asset identity mutation, `tamper_final_state` NPZ/JSONL semantic
mutation after checksum updates, and the expected-exception helpers
(`rejects`, `incompatible`, `flag`/`text_of`/`count` accessors).

## Confirmed defects / staging traps found during the port

1. **DuckDB `Connection::execute` takes a parameter span**, not a bare
   `Value` — all UPDATE statements use `std::array` parameter lists.
2. **`Environment::catalog` is `std::optional`** — `rec::Catalog` has
   no default ctor; it is emplaced after model construction and
   dereferenced at `make_research_evidence` call sites.
3. **Safe-buffer policy**: pointer arithmetic and two-parameter
   `std::span` construction are errors on both compilers — NPZ/recipe
   byte copies go through `std::as_bytes(std::span{...})`, buffer views
   through `model_access::readonly_buffer`/`mutable_buffer`, and byte
   equality through `std::ranges::equal` (no `memcmp`).
4. **`ProcessOptions` designated initializers are darwin-only**:
   skipping `cancellation`/`cancellation_grace_s` trips Linux clang's
   `-Wmissing-field-initializers` — the subprocess calls name an
   options variable and set `timeout_s` explicitly.
5. **`run_replay`/`fidelity_sweep` fingerprint `root/"native/src"`
   before the try block** — the contract's failure reports only exist
   when `root` is a real checkout; the ports pass `repo_root()` with
   absolute fixture paths.
6. **`bike-validate` / `bike-replay` location**: the CLIs live at
   `<build-root>/bin/`; the tests resolve them four `parent_path()`
   levels up from `platform::executable_path()` and declare
   `add_dependencies` on the tool targets so a bare target build keeps
   the artifacts fresh.
7. **`status()` is a `WireObject`, not `io::Value`** — pending/active
   reads go through `research::wire_value(Wire{...})`.
8. **`efc_force` is a span in native**, not an owned snapshot: the
   arena-relocation record keeps a caller-owned `std::vector` copy —
   the exact ownership the binding's detached ndarray emulates.

## Verification evidence

- **CTest darwin-arm64** (`native/build/migration_T03_work/no-python`,
  Apple Clang): `ctest -L t53` filtered to the nine replay targets —
  **9/9 Passed** (35.3 s): `replay_manifest_v2` 0.24 s,
  `replay_recording_v3` 8.65 s, `replay_session` 10.63 s,
  `native_replay_recording_v3` 5.47 s, `native_replay_session` 4.25 s,
  `pedelec_replay_contract` 0.43 s, `native_view_contracts` 0.03 s,
  `replay_playback` 4.62 s, `native_viewer_snapshot` 0.72 s.
  Per-binary check counts: 31 (pedelec), 66 (view), 220 (playback),
  12 (snapshot), 45+14 (native replay pair), 112 (manifest/recording/
  session grouped invocation).
- **CTest linux-aarch64** (`bike-dev-pfc-lane-d`, `/build/root-clang`,
  clang++-18, Release): same scope — **9/9 Passed** (40.85 s).
- **Python oracle collection**: `uv run pytest <9 files>
  --collect-only -q` — 85 nodes measured into `collected_node_ids`.
- **Subprocess coverage**: `reference.pedelec_replay_contract` spawns
  `bin/bike-validate` (exit 1 + retained error); `reference.
  replay_playback` spawns `bin/bike-replay` (`--view --dry-run` exit 0/
  1, default verify exit 0) — no test-only reimplementation.
- **Static registration**: `Phase3Ports.cmake` — nine
  `bike_add_reference_test` entries (`ENGINE` where MuJoCo runtime
  linkage is required), `LABELS t53 replay`, plus `add_dependencies`
  on `bike-validate`/`bike-replay`.

## Ledger

- `migration/test-port-map.json`: 48 `port`, 1 `drop`
  (BIKE_NATIVE_BUILD_PATH artifact selection — python-only), 2
  `planned` (viewer-snapshot cross-oracle parity reds) — measured
  `collected_node_ids`, `assertions_capture`, two-platform evidence on
  every row.
- `migration/python-removal.json`: eight source rows →
  `status: replacement_landed`, `verification_ids` set to the CTest
  ids. `test_native_viewer_snapshot.py` stays `planned` until the two
  parity reds land; deletion stays gated at T63.

## Commits

| SHA | Unit |
|---|---|
| `322deec3` | feat(replay): strict schema-2 execution identity validation (C3.32) |
| `433d39f7` | test(reference): port T53 replay/session/manifest/playback/view suite (C3.32) |
| (next) | fix(reference): explicit ProcessOptions for linux missing-field-initializers |
| (next) | docs(migration): 3.32 port-map/removal ledgers + C332 report |
