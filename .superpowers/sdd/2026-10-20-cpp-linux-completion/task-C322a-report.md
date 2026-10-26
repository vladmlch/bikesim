# Task C322a — test-port 3.22a report (encode columns, recording)

**Batch**: 3.22a — `tests/reference/test_native_encode_columns.py`
(40 records / 40 collected nodes; no parametrization — `pytestmark`
binds only the `native_module` fixture). CTest id
`reference.native_encode_columns`; labels `t52 recording golden slow`;
port-map rows carry `owner_task: T52`.

**Lane**: `.worktrees/pfc-lane-c`, branch `lane/c`, base `c4cf470f`.
No merge, no push, no subagents.

## Status: landed, all 40 records green

Commit on `lane/c`: 39 of 40 records ported into
`native/tests/reference/test_native_encode_columns.cpp` (2649 lines,
single TU, `main()` drives the same source-file order), 22 golden
records with both-platform fixtures, 1 documented drop, ledger rows,
evidence section.

## Triage

| File | port | golden | drop | planned |
|---|---|---|---|---|
| `test_native_encode_columns.py` | 17 | 22 | 1 | 0 |

The drop is `test_make_episode_rejects_malformed_channels`: the rejected
shapes (a sample row without `channels`, or `channels` holding a list)
exist only at the Python binding boundary — the typed C++
`PhysicalSampleData::channels` (`WireObject`) cannot express them, so
the gate lives entirely inside `python_to_wire`.

No record was baseline-red; no expected-fail ports (D16).

## What the port reproduces

- **`column_bits` byte identity over `EncodedTable` pieces** — validity
  bitmap (LSB-first, allocated iff Arrow materializes a null) + int32
  offsets + payload lanes, recursing list/map/struct children in catalog
  order with generated `<child>_present` companions; float lanes keep
  `-0.0` and NaN payloads bitwise-visible.
- **Arrow's `pa.array` normalization**: every recorded batch is the
  `arrow_batch` side (flat columns → `pa.array`), so a member slot under
  a null parent struct element is stored *valid* with default payload —
  the mirror carries an `arrow_validity` context,
  `masked(i) = piece_invalid(i) && parent_arrow_valid(i)`. Verified
  directly against pyarrow and pinned by the `commands_applied`
  `control` struct golden (absent `posture` → nested members emit raw
  zero lanes, no bitmaps).
- **Move-only `Piece` handling**: recursive `copy_piece`, and
  `concat_tables` mirroring `pa.concat_batches` — mask widening where a
  fragment is all-valid, offsets rebasing.
- **The `freeze` typed-tree serializer** for `iter_derived_events` /
  `derived_rows` comparisons, plus per-leaf float consumption matching
  `assert_bitwise_equal`'s golden record order.
- **Bootstrap replay**: the whole t=0 runtime inventory —
  `config.json`, `state.json`, `state.arrays/*` and the live
  `model.mjb` — travels as `ride.file::<path>` golden inputs and is
  materialized into a temp stage for `read_bootstrap` +
  `NativeRideRuntime.advance(65)`.

## Findings worth recording

1. **`mj_saveModel` bytes are platform-dependent.** Live linux-aarch64
   model sha256 `26a9dfb9…` vs committed darwin-arm64 `3f1d0d41…`
   (~1e-13 in model fields). Tolerance compares absorb the drift; the
   bitwise `physics/bits/qpos` lanes do not — the first linux run failed
   there until the live model joined the recorded manifest. The
   committed fixture
   `tests/fixtures/migration/bootstrap/encode_columns_welded/`
   (`model.mjb` + `meta.json`) remains the canonical reference image.
2. **`np.linspace(0,1,8)` ≠ `k/7`**: intermediates are `k*fl(1/7)` with
   the endpoint pinned at exactly `1.0`; the staged qpos rows in
   `test_physics_and_forces_bitwise_vs_oracle` needed that lowering.
3. **clang-18 strictness** (linux lane): `-Wmissing-field-initializers`
   on eight `FieldSpec`/`TableSpec` designated initializers, and a
   `-Wframe-larger-than=8192` trip in `drive_channels` — the ~110-pair
   `WireObject` initializer-list became `emplace_back` calls.

## Verification

- **Parity pytest** (golden capture runs, full file):
  `BIKE_GOLDEN_CAPTURE=tests/fixtures/golden
  NATIVE_TEST_BUILD_PATH=<build> uv run --frozen --no-sync pytest -q
  tests/reference/test_native_encode_columns.py`
  → **40 passed, 3 warnings** on darwin-arm64 (`native/build`, 9.72 s)
  and **40 passed, 3 warnings** on linux-aarch64 (`/build/native-clang`,
  `bike-dev-pfc-lane-c`, 3.49 s). The warning is the known
  `invalid reference step at 0.005000000: ('energy.constraint_work',)`
  RuntimeWarning in the three real-episode nodes.
- **Focused CTest**: `ctest -R reference.native_encode_columns`
  → **PASS 7834 checks** on darwin-arm64 (`build/native-release`,
  0.41 s) and **PASS 7834 checks** on linux-aarch64
  (`/build/root-clang`, clang++-18, 0.45 s).
- **Golden fixtures**: 22 node pairs (`.json`+`.bin`) per platform under
  `tests/fixtures/golden/{darwin-arm64,linux-aarch64}/
  tests_reference_test_native_encode_columns.py/`; linux sidecars pulled
  via `container.sh pull`.
- **Byte-flip proof**: byte 1024 of
  `test_physics_and_forces_bitwise_vs_oracle.bin` (darwin-arm64) flipped
  → FAIL `physics/bits/qvel: bytes differ`; restored byte-identical →
  PASS. The bitwise comparator detects single-byte fixture corruption.
- **Static checks**: the TU compiles warning-free under both toolchains
  (AppleClang `-Wframe-larger-than=8192`, `-Wunsafe-buffer-usage`, …;
  clang++-18 same policy); no sweep findings added.

## Final gate — `bash tools/run_tests.sh native` (darwin-arm64)

Two runs. Run 1 stopped at `check_tidy` on a single pre-existing finding
— `alpine.cpp:84 misc-use-internal-linkage` on `faceted`, left with
incidental external linkage by `5df6c45e` (lane/d merge, inside base
`c4cf470f`) and undeclared in `detail.hpp`. Fixed per the checker's own
prescription (anonymous namespace inside `detail`); standalone
`check_tidy` rerun green.

Run 2 (completion gate): exit 8 — build green, all nine static-analysis
sweeps green, CTest 60/62 on the documented baseline pair
`migration_T09` + `migration_T29` (recorded at HEAD repeatedly in
`cpp-linux-completion-evidence.md`, including a stash+rerun
reproduction; neither file nor its code path is touched by this batch).
The pytest tail is not reached after the ctest stop — same abort
behavior the earlier dated entries record; the per-batch required gates
(focused CTest on both platforms, 40-node oracle pytest on both
platforms, two-platform goldens, corruption proof) are all green.

## Ledger

- `migration/test-port-map.json`: all 40 rows dispositioned — 22
  `golden` (comparison `oracle-derived comparison`,
  `golden_platforms: [darwin-arm64, linux-aarch64]`), 17 `port`
  (`native contract port`), 1 `drop` (`python-only`); `line`/`end_line`
  refreshed to the live source; dated evidence strings appended.
- `migration/python-removal.json`: `test_native_encode_columns.py` →
  `replacement_landed` (replacement
  `native/tests/reference/test_native_encode_columns.cpp`, verification
  `reference.native_encode_columns`; `sha256`/`line_count` refreshed to
  the recording-instrumented source, snapshot hash + line count kept).
- `native/tests/reference/Phase3Ports.cmake`: registered as
  `bike_add_reference_test(native_encode_columns ENGINE … LIBRARIES
  bike_recording LABELS t52 recording golden slow)`.
- `migration/cpp-linux-completion-evidence.md`: dated 3.22a section
  appended.

## Concerns / notes for merge-back

1. The three real-episode goldens each carry a full ~5.8 MiB recorded
   `model.mjb` input (platform-faithful replay is required bitwise); the
   linux sidecars total ~18 MiB. That is the honest bitwise cost — the
   earlier repoint-and-skip scheme cannot distinguish platform model
   drift.
2. `test_make_episode_rejects_malformed_channels` is the file's only
   drop (binding-only gate); the typed-boundary equivalents are covered
   by the ported reject tests (`unknown_channel_root/member`,
   `map_leaf_rejects_non_dict_and_bad_member`, …).
3. The `_record_bootstrap` helper still takes `committed_name` — it now
   only asserts the committed reference image exists (the digest
   repoint is gone); kept so the call sites document which committed
   fixture each capture corresponds to.
