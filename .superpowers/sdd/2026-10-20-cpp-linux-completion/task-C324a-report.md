# Task C324a — test-port 3.24a report (native benchmark, episode)

**Batch**: 3.24a — `tests/reference/test_native_benchmark.py`
(56 records / 90 collected nodes). CTest id
`reference.native_benchmark`; labels `t52 episode golden`; port-map
rows carry `owner_task: T52`.

**Lane**: `.worktrees/pfc-lane-a`, branch `lane/a`, base `c35ebfae`
(+ lane commit `2fe70226` carrying the capture site and the
`bytes_input` accessor). No merge, no push.

## Status: landed, all 56 records green

All 56 records ported into
`native/tests/reference/test_native_benchmark.cpp` (~1650 lines,
single TU, `main()` drives the same source-file order). 55 `port` +
1 `golden` (the conformance symbol covering 3 parametrized nodes);
both-platform fixtures captured; ledger rows and evidence section
landed.

## Triage

| File | port | golden | drop | planned |
|---|---|---|---|---|
| `test_native_benchmark.py` | 55 | 1 | 0 | 0 |

No drops; no record was baseline-red; no expected-fail ports.

## What the port reproduces

- **The `native_bench` subprocess boundary**: every check execs the
  real binary (`BIKE_NATIVE_BENCH_PATH` from the CMake-generated
  `bike_native_bench_path.hpp`, `add_dependencies` on `native_bench`),
  asserts exit codes, and matches stderr/stdout fragments where the
  Python assertions inspect diagnostics.
- **CLI contract**: `--help` usage, `--selftest` CRC64-ECMA check
  vector (`"123456789" → 0x6C40DF5F0B497347`), required mode, unknown
  modes (`Glue`, `spin`, `0`, `--mode`), malformed `steps`
  (8 cases incl. `MAX_COUNT+1` and uint64 overflow text), invalid
  `rewind` (7 cases), missing/duplicate/dangling `--artifacts`,
  extra positionals.
- **Bundle resolution**: concrete bundle dirs, `current.json` pointer
  roots, malformed/missing/escaping pointers, and per-member readable
  checks (`model.mjb`, `state.bin`, `state.npz`, `manifest.json`) —
  the ordering that a per-test fresh `tmp_path` exposes.
- **`BIKEST02` state validation**: legacy `0xBEA0` magic with the
  regeneration hint, bad magic, unknown version/flags, truncation
  inside header and payload, trailing bytes, oversized `n*` fields,
  per-field model-dimension mismatch (`nq`/`nv`/`na`/`nu`/`nbody`),
  payload-length lies, payload- and declared-CRC corruption,
  foreign-generation and foreign-dimension models, non-finite
  scalars. Fixtures serialize natively with the same field order
  (`time, qpos, qvel, act, ctrl, qfrc_applied, xfrc_applied,
  qacc_warmstart`) and CRC64-ECMA as
  `tools/proto_native_bench/state_format.py`; `state.npz` via
  `bike::io::encode_npz`; `model.mjb` via `mj_saveModel`.
- **Workload domains**: `bare` accepting `nv=0`, `glue`/`heavy`
  empty-model rejection, no-actuator `glue` rejection, `heavy` body
  count (`nbody<10` at dofs 6 and 9) and narrow-model rejection, the
  `nv ∈ {0,1,33,65,128}` bare and `{1,33,65,128}` glue sweeps,
  `nu>nv`, `heavy` nv=33 paired Jacobians, the nv=10 boundary model,
  `operation_mix`, rewind==steps / rewind>steps boundaries, and the
  tracked `tools/proto_native_bench/artifacts` smoke across all four
  mode/arg pairs.
- **Report contract**: emitted JSON shape (`workload.claim`,
  `operation_counts`, `provenance` crcs, stages, checksum) and the
  unwritable `--emit-json` rejection.
- **Oracle conformance**: the recorded `input.report` byte document —
  canonical compact sorted JSON of the Python emission — replays via
  `bt::oracle::bytes_input`; the port re-asserts every comparison in
  the pytest body verbatim (exact integer stages + `1e-12`-close
  floats; timing fields intentionally uncompared).

## Findings worth recording

1. **`std::as_chars` does not exist.** The original draft used it for
   byte→char views; C++23 has only `std::as_bytes`. Fixed with an
   `append_bytes`/`write_bytes` byte-loop pair (the same put-per-byte
   idiom `test_golden_support.cpp` uses) and a byte→string loop for
   the replayed report.
2. **`-Wunsafe-buffer-usage` forbids `string_view` iterator
   arithmetic** (`raw.end() - 16`, `raw.begin() + 20`); `substr`
   windows replaced both.
3. **clang-18 `-Wmissing-field-initializers`** fires on partial
   designated initializers where AppleClang stays quiet — both
   `BundleSpec` sites now spell all three fields.
4. **pytest `tmp_path` isolation is load-bearing.** The first green
   run shared one `TemporaryDirectory` across sibling tests;
   `test_missing_state_bin`'s `model.mjb` survived into
   `test_missing_model_mjb`, which then failed on `state.npz`
   instead. `with_tmp_path` gives every check function a fresh dir,
   matching the per-test fixture exactly.
5. **`container.sh sync` is `rsync --delete`.** Goldens captured
   in-container are erased by the next sync before they are pulled —
   the linux capture ran before a re-sync and had to be re-captured;
   pull first (or sync, capture, never sync again).

## Verification

- **Parity pytest** (full file): **90 passed** darwin-arm64
  (`build/native-release`; capture run 4.26 s, plain rerun 4.50 s)
  and **90 passed** linux-aarch64 (`/build/native-clang`,
  `bike-dev-pfc-lane-a`, `NATIVE_TEST_BUILD_PATH=/build/native-clang`,
  2.42–2.47 s, `BIKE_GOLDEN_CAPTURE=tests/fixtures/golden`).
- **Focused CTest**: `ctest -R reference.native_benchmark` → **PASS
  18167 checks** darwin-arm64 (`build/native-release`, 3.25 s) and
  **Passed** linux-aarch64 (`/build/root-clang`, clang++-18, 1.01 s).
- **Golden fixtures**: 3 conformance node pairs per platform under
  `tests/fixtures/golden/{darwin-arm64,linux-aarch64}/
  tests_reference_test_native_benchmark.py/`; linux sidecars pulled
  via `container.sh pull`.
- **Byte-flip proof**: byte 844 of
  `test_matched_workload_python_native_conformance_bare_.bin`
  (darwin-arm64) flipped → FAIL `bike::io::parse_json: invalid JSON
  at offset 844`, exit 1; restored → PASS 18167 checks, exit 0.
- **Static checks**: the TU compiles warning-free under both
  toolchains (AppleClang + clang++-18 strict policies).

## Final gate — `bash tools/run_tests.sh native` (darwin-arm64)

Run status recorded in `migration/cpp-linux-completion-evidence.md`
(3.24a section); see the ledger entry below for the outcome.

## Ledger

- `migration/test-port-map.json`: all 56 rows qualified — 55 `port`
  (`native contract port`), 1 `golden` (`oracle-derived comparison`,
  `golden_platforms: [darwin-arm64, linux-aarch64]`); measured
  `collected_node_ids` (90), dated `assertions_capture`, 3.24a lane/a
  evidence strings.
- `migration/python-removal.json`: `test_native_benchmark.py` →
  `replacement_landed` (replacement
  `native/tests/reference/test_native_benchmark.cpp`, verification
  `reference.native_benchmark`; `sha256`/`line_count` refreshed to
  the capture-instrumented 1027-line source, snapshot kept).
- `native/tests/reference/Phase3Ports.cmake`: registered as
  `bike_add_reference_test(native_benchmark ENGINE SOURCES
  test_native_benchmark.cpp LIBRARIES bike_app LABELS t52 episode
  golden)` + `add_dependencies(reference.native_benchmark
  native_bench)` + `file(GENERATE)` emitting
  `support/bike_native_bench_path.hpp`.
- `native/tests/support/golden.{hpp,cpp}`:
  `bt::oracle::bytes_input(label)` — next captured entry, bitwise
  `|u1`-only, cursor-advancing (landed earlier in `2fe70226`).
- `tests/reference/test_native_benchmark.py`: canonical compact JSON
  `input.report` record in the conformance node (landed earlier in
  `2fe70226`).
- `migration/cpp-linux-completion-evidence.md`: dated 3.24a section
  appended.

## Concerns / notes for merge-back

1. The conformance replay trusts the recorded Python emission bytewise
   and re-derives every comparison on the native side; platform model
   drift (`mj_saveModel` byte differences) does not affect the check
   because both sides of the comparison are generated live in the same
   process on the same platform.
2. `test_tracked_artifacts_smoke` runs the real
   `tools/proto_native_bench/artifacts` bundle read-only — it depends
   on the tracked bundle staying in sync with `state_format` v2.
3. `with_tmp_path` intentionally re-creates one `TemporaryDirectory`
   per check rather than sharing: keep that discipline if more checks
   are added, or the missing-member tests can silently mask again.
