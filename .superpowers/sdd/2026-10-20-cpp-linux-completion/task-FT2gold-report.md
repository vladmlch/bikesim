# Task FT-2 — golden-capture batch: 28 golden ports, two platforms, ledger + fixtures committed

**Lane**: `.worktrees/pfc-lane-f`, branch `lane/f2`, base `3901bdb4`.
No merge, no push. Commits on lane/f2 (post-rebase SHAs): `96f9e2c9`
(partial: capture harness + ft2 fixtures + runtime step/commands/rollout
ports), `ae7c8007` (research capture port), `a7f05d85` (research
recording port), `881b21a4` (provenance equilibrium+raw_initial),
`9cab7702` (research environment ports), `beff9745` (viewer snapshot
ports), `91732afb` (linux-aarch64 goldens + ledger + platform provenance
fix). The `Devin-Session: .../placeholder` trailer on the original
checkpoint `9b4c3ee1` was removed by rebase — the repo convention is no
such trailer (1026 commits clean; `Co-Authored-By` appears on one).

## Scope

Complete the remaining ADR-0003 golden-capture batch: 7 pytest files /
51 ledger records. Outcome: **28 records `golden`** (24 planned→golden,
2 earlier contract `port`→golden upgrades), **22 records stay `port`**
(contract ports without per-record goldens), **1 `drop`**, **1 stays
`planned`**.

## Per-file dispositions (test-port-map.json)

| pytest file | golden | port | drop | planned | removal status |
|---|---|---|---|---|---|
| test_native_runtime_step.py | 7 | 3 | | | replacement_landed |
| test_native_runtime_commands.py | 4 | 6 | | | replacement_landed |
| test_native_runtime_rollout.py | 5 | 1 | | | replacement_landed |
| test_native_viewer_snapshot.py | 2 | | | | replacement_landed |
| test_native_research_capture.py | 1 | 1 | | | replacement_landed |
| test_native_research_environment.py | 8 | 4 | 1 | 1 | **planned** (see concerns) |
| test_native_research_recording.py | 1 | 6 | | | replacement_landed |

Adapter-seam reconciliation (lane/d2 `61001d7e` — not an ancestor of
lane/f2; dispositions replicated canonically, evidence preserved):

- `test_publication_failure_retry_does_not_advance_twice` → **drop**,
  `comparison_class=python-only`, native anchors nulled. Covered by the
  ported twins `test_zero_budget_retains_single_window_and_no_tail_flush`
  + `test_invalid_boundary_calls_do_not_change_state`; the injected
  `adapter._transition` failure is a Python-only seam deleted at T63.
- `test_custom_rider_callback_runs_once_per_external_boundary` → stays
  **planned**; once-per-external-boundary callback has no native test
  leg (needs a counting-`RiderBehavior` through `begin_advance` +
  zero-budget polls + reset).

## Golden fixtures committed

Per platform (`tests/fixtures/golden/<platform>/`), 35 node dirs —
identical layout on both platforms:

| file dir | nodes |
|---|---|
| tests_reference_test_native_runtime_step.py | 7 |
| tests_reference_test_native_runtime_commands.py | 4 |
| tests_reference_test_native_runtime_rollout.py | 5 |
| tests_reference_test_native_viewer_snapshot.py | 2 |
| tests_reference_test_native_research_capture.py | 1 |
| tests_reference_test_native_research_environment.py | 15 |
| tests_reference_test_native_research_recording.py | 1 |

Migration roots `tests/fixtures/migration/ft2/<platform>/{ride,
research_flat,research_savage}` (model.mjb, config/state +
`*.arrays/`, research.json, provenance.json incl. `equilibrium` and
`raw_initial`, native-stamped `assets/` + `assets.json`; 19 MiB per
platform) captured via `tools/migration/_ft2_capture.py`.

## Verification

- **Darwin capture**: `BIKE_GOLDEN_CAPTURE=tests/fixtures/golden
  PYTHONPATH=tests/reference` pytest over the 7 files — all nodes pass
  (26 + 34).
- **Linux capture**: `bike-dev-pfc-lane-f` container, `/opt/bike-venv`,
  `NATIVE_TEST_BUILD_PATH` + `BIKE_NATIVE_BUILD_PATH=/build/native` —
  both selectors are required; the conftest session fixture forwards to
  the application selector only when the `native_module` fixture runs.
  Same two batches green (26 passed / 34 passed; warnings are the known
  `energy.constraint_work` diagnostic notices).
- **Scoped CTest darwin-arm64** (`build/native-release`): 9/9 pass —
  `reference.native_{runtime_step,runtime_commands,runtime_rollout,
  viewer_snapshot,research_capture,research_environment,
  research_recording,research_capture_contracts,
  research_recording_lifecycle}`, 36.57 sec*proc total, exit 0.
- **Scoped CTest linux-aarch64** (`/src/build/native-release`, root
  preset `release`, Clang 18): same 9/9 pass, 31.48 sec*proc, exit 0.
- **Corrupted-golden proof (darwin-arm64)**: byte 100 XOR 0xFF inside
  `test_native_target_slice_does_not_flush_or_duplicate_request.bin`
  (entry-0 `target slice integration state` payload) →
  `reference.native_viewer_snapshot` FAIL `comparison 0 ... element 12:
  actual=0x3fcc3a23c2bab8d7 expected=0x3fcc3adcc2bab8d7`, exit 1.
  Byte restored → `PASS 949 checks`, exit 0; fixture byte-identical.
- **Final native gate** (`bash tools/run_tests.sh native`, darwin-arm64,
  2026-10-31): build green; all 9 sweeps green — frontends none, tidy
  333 srcs/334 TUs (alpha findings report-only), analyzer report-only,
  cppcheck none, odr none, scripts clean, headers 269 hdrs/9 570 TUs
  none, diagnostic-controls 6/6, context all passed. CTest (legacy
  `native/build` graph): **60/62, exit 8** — the only reds are
  `migration_T09` (`post-advance integration vector matches the oracle`)
  + `migration_T10` (`first transition matches the Python oracle`),
  the **documented baseline pair**: identical signatures to the
  pre-session red set recorded 2026-10-29 in
  `native/build/Testing/Temporary/LastTestsFailed.log` (also identical
  in the 05:45 same-day run before this gate), and to the frozen-oracle
  drift pair documented throughout `migration/cpp-linux-completion-
  evidence.md` (e.g. lines ~6385, ~7273). `git log 3901bdb4..HEAD`
  touches neither `test_t09.cpp`/`test_t10.cpp` nor their fixtures
  (`migration/bootstrap/*`, `migration/research/delayed_bootstrap/*`)
  nor `native/src/{rider,physics,sim}` — FT-2 cannot be causal. The
  script stops at the ctest leg on failure, so the in-gate pytest leg
  did not run; pytest qualification for this batch is the two-platform
  capture runs above (26 + 34 nodes green each).

## Platform-parity fixes landed this batch

- `test_native_research_environment.cpp` + `test_native_research_recording.cpp`:
  Clang 18 `-Werror,-Wmissing-field-initializers` — `RiderPosture`
  designated initializer replaced by a defaulted temporary + member
  assignment (Darwin's warning batch tolerated the partial init).
- Same files: `model_commit` / `model_source_sha256` provenance
  substitution now copies the recorded `Value` **verbatim**
  (string-or-null) instead of `rd::string`. The container's `/src`
  carries no `.git`, so the Linux oracle records `model_commit=None`
  (`model_source_dirty=None` likewise); native stamps `null` for both
  by construction (`native/src/research/metadata.cpp:155`). `model_source_sha256`
  stays a real sha on both platforms — it is the first-party source
  manifest hash, not a git object id.

## Ledgers

- `migration/test-port-map.json`: 28 records → `golden` with real
  `native_test_id` (`reference.<ctest>::<cpp-symbol>`; viewer legs use
  the actual C++ symbols `preview_owned` / `target_slice`) and
  `golden_platforms=[darwin-arm64, linux-aarch64]`; drop/planned as
  above; evidence appended per record.
- `migration/python-removal.json`: the six fully-covered files →
  `replacement_landed`; `test_native_research_environment.py` stays
  `planned`. Snapshot fields are intentionally not refreshed (stale
  snapshots are the established convention — e.g. sensors 154→199).

## Concerns / follow-ups

1. **`test_native_research_environment.py` keeps one planned record**
   (custom rider callback) — the file cannot reach `replacement_landed`
   or be deleted until a counting-`RiderBehavior` native leg exists.
2. **Container sync is destructive to captured-only trees**: `container.sh
   sync` runs `rsync -a --delete`; a mid-task re-sync wiped the first
   linux capture. Capture → `pull` (or commit) before any later sync.
3. Viewer startup-row substitution (4 construction-time
   accumulator/contact fields) and the recording/env provenance
   substitutions are documented seams — native asserts the values are
   well-formed before re-seating the oracle-recorded bytes.
4. Linux full gate (`container.sh gate native`) was not run; the scoped
   9-registration sweep plus host `tools/run_tests.sh native` gate
   (sweeps green; ctest stops at the documented baseline T09/T10 pair)
   are the qualification evidence for this batch.
