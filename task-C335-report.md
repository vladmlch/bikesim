# Task C335 report — batch 3.35 "platform files" (T53, lane/d)

Branch: `lane/d`, base tip `5308c55a`. Scope: migrate the ten
platform-oriented Python test sources of batch 3.35 to native C++
reference tests, capture linux-aarch64 goldens to match the existing
darwin-arm64 set, and update the migration ledgers. Python sources and
implementation remain in place; deletion stays gated by T63 per
ADR-0003.

## Record disposition totals

132 batch records: **70 ported, 34 golden, 28 dropped**.

| Python source | Native replacement | Records (port/golden/drop) |
|---|---|---|
| tests/test_execution_identity.py | `reference.execution_identity` | 26 / 0 / 0 |
| tests/reference/test_native_numeric_portability.py | `reference.native_numeric_portability` | 2 / 21 / 1 |
| tests/reference/test_native_platform_blas.py | `reference.native_platform_blas` | 1 / 11 / 0 |
| tests/reference/test_native_platform_linkage.py | `reference.native_platform_linkage` | 4 / 0 / 0(3) |
| tests/reference/test_native_linux_headless.py | `reference.native_linux_headless` | 4 / 0 / 0 |
| tests/reference/test_native_platform_recording.py | `reference.native_platform_recording` | 9 / 0 / 9 |
| tests/reference/test_native_platform_replay.py | `reference.native_platform_replay` | 6 / 0 / 0 |
| tests/test_recording_platform_compatibility.py | `reference.recording_platform_compat` | 17 / 0 / 0 |
| tests/test_async_recovery.py | `reference.async_recovery` (+`reference.async_close`) | 8 / 0 / 3 |
| tests/reference/test_golden_hook.py | — (delete_with_python) | 0 / 0 / 5 |

(Counts per `migration/test-port-map.json`; the linkage/headless table
rows above reflect the per-record dispositions recorded there.)

## What was ported and how

- **execution_identity** (`native/tests/reference/test_execution_identity.cpp`)
  — full v2 identity record construction, fail-closed field, nested-field
  and Linux file-identity mutation tables, named-failure diagnostics, and
  the `require_execution_compatible` gate. 26/26 records, no drops.
- **numeric_portability** — golden-replayed bitwise checks over
  `_debug_numeric_contract`/`_debug_pneumatic_math`/`_debug_interp`/
  `_debug_rider_linalg`: tanh/hypot (scalar, strided, adversarial,
  layout views), min/max (incl. signed zero + NaN class fixtures),
  multiply broadcast, python_hypot, sum, sin/cos edge+nonfinite
  fixtures, interp incl. reject paths via production `np_interp`,
  rider linalg 2-D/3-D/norm + reject paths. One drop: a Python
  import-ordering mechanic. 34 records carry `golden` disposition with
  `golden_platforms: [darwin-arm64, linux-aarch64]` after this task.
- **platform_blas** — golden-replayed dot/matvec/matmul over C and F
  layouts, zero-size shapes, transposed views, rectangular selections,
  and all provider reject paths (`_debug_platform_blas`).
- **platform_linkage** — subprocess link-table inspection instead of
  Python ctypes probes: `otool -L` on Darwin; `readelf -d` + `ldd` on
  Linux. Runs the real published binaries under a scrubbed
  `LD_/DYLD_*` environment; absolute install-name assertions scoped to
  `libmujoco` (`libduckdb` legitimately keeps `@rpath`); searches for
  `libmujoco` (not `mujoco`) to avoid matching the `mujoco_clean`
  checkout path. `bike-test-child` is exercised as
  `bike-test-child echo ok`.
- **linux_headless** — real `bike-ride` subprocess stepping under a
  display-free scrubbed environment; diagnostic path exits 0 with
  `duration_reached: false` + model violation, strict path exits 2 on
  the invalid reference step; recording cleanup verified.
- **platform_recording** — publication/sealing/crash-staging,
  exclusive-name suffixing past squatters, EACCES fail-closed + retry
  (forked `nobody` probe), Unicode paths, strict stop preserving owned
  rows, record-disabled path, real-run deep payload, research
  publication with `replay_contract_version == 3` + replay eligibility;
  physical recordings keep nullable replay metadata and assert
  non-eligibility. 9 covered-by/drop records are Python-harness
  mechanics already covered by shared support.
- **platform_replay** — explicit-episode requirements, platform
  metadata, replay compatibility across physical/research records.
- **recording_platform_compatibility** — cross-representation reader
  contracts; documented native/Python journal-name deltas
  (`episode_started` vs `episode_start`) and outcome names
  (`simulation_error` vs `invalid_controller`) in source comments.
- **async_recovery** — `PeriodAccounting` and `AsyncCloseRunner`
  recovery, pending/current publication, no-duplicate-replay,
  strict-rejection prefix retention, ownership. 3 drops are Python
  multiprocessing queue-transport cleanup mechanics with no native
  counterpart.
- **golden_hook** — dropped (5 records): it tests Python-only
  `_golden` capture-hook internals. Native golden-reader behavior is
  covered by `reference.golden_support`; the file stays
  `delete_with_python` under the T63 gate.

## Linux aarch64 evidence (this task)

- Container `bike-dev-pfc-lane-d` (Ubuntu 24.04, Docker linux/arm64),
  root build `/src/build/native-release` — the earlier "No tests
  found" report came from CTest'ing `/build/native-clang`, which is the
  binding-only subtree build; the reference tests live in the root
  build.
- All nine batch CTests pass: `reference.execution_identity`,
  `native_platform_replay`, `recording_platform_compat`,
  `async_recovery`, `native_platform_recording`, `native_linux_headless`,
  `native_platform_linkage`, `native_numeric_portability`,
  `native_platform_blas`.
- linux-aarch64 goldens captured via
  `BIKE_GOLDEN_CAPTURE=/src/tests/fixtures/golden` against the live
  oracle (`169 passed` for the two numerics files) and pulled into
  `tests/fixtures/golden/linux-aarch64/` (commit `43f8c4b1`).
- Portability fixes required by the Linux lane (same commit):
  explicit `std::pair<Value(*)(), const char*>` (CTAD deduced `char[N]`
  per literal), `emplace_back` table construction plus a per-case
  helper (the 27-entry `initializer_list` backing array exceeded the
  8 KiB `-Wframe-larger-than` cap), `<grp.h>` for glibc `setgroups`,
  and checked `setgid/setuid` results (`-Wunused-result`).
- Darwin re-verified after those edits: both touched tests green on
  `build/native-release`.

## Ledger updates

- `migration/test-port-map.json`: 34 records now carry
  `golden_platforms: [darwin-arm64, linux-aarch64]`; "linux-aarch64
  pending" evidence rewritten to the landed container evidence; all 70
  ported records carry the linux-green evidence line citing commit
  `43f8c4b1`.
- `migration/python-removal.json`: all nine ported files
  `replacement_landed`; `test_golden_hook.py` stays `planned` /
  `delete_with_python` under T63. No Python deletion performed.

## Commits (this task)

- `b63435b2` platform_replay port (prior)
- `b129f9fb` async_recovery + recording_platform_compat
- `9b1f9070` native_platform_recording
- `4f922ff0` linux_headless + platform_linkage
- `24efb1c8` numeric_portability + platform_blas (darwin goldens)
- `43f8c4b1` linux portability fixes + linux-aarch64 goldens

## Remaining gates

- `bash tools/run_tests.sh native` final gate (see below / latest run).
- Python deletion: T63 after T61/T62 and row-specific evidence.
