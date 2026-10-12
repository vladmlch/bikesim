# Run recording implementation audit — 2026-10-12

Master plan: `docs/superpowers/plans/2026-10-12-run-recording.md`
(ordinary stage: `...-ordinary.md`, research stage: `...-research.md`,
spec: `docs/superpowers/specs/2026-10-12-run-recording-design.md`, R01–R24).

This ledger records what was actually run and observed. A documented
failure is not a pass; open rows keep their tasks unclaimed.

## Execution baseline (A0)

| Fact | Value |
|---|---|
| Checkout | `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/cpp-port-p2` |
| Branch | `impl/cpp-port-p2` |
| HEAD | `e3b1d9e5758fda1940d21e547fc8809954d7ca32` (`feat(tools): port research_sweep/step_profile + perf log from bikesim_new`) |
| Tracked tree | clean — `git status --short` shows zero modified tracked entries |
| Untracked | 70 entries, all user-owned and preserved: `.claude/skills/` (55), `docs/superpowers/` plan+spec files (11), `docs/agents/`, `docs/reference/simulation-audit-2026-10-07.md`, `output/`, `tools/proto_native_bench/artifacts/fixture-v3/` |
| Selected artifact | `native/build/release` via `NATIVE_TEST_BUILD_PATH`; rebuilt at HEAD by the run (`bike_native.cpython-314-darwin.so`) |
| Toolchain | Python 3.14.3, uv 0.12.9, NumPy 2.5.2, MuJoCo 3.12.0, nanobind 3.1.0, pytest 9.1.1; Apple clang 21.0.0 (`/usr/bin/c++`); clang-tidy/clang++ LLVM 23.1.2 (keg `/opt/homebrew/opt/llvm/bin`); cppcheck 2.22.0; g++-16 GCC 16.2.0; shellcheck 0.11.0 |
| Profiles | `examples/research/viewer_physics_distributed.toml` ABSENT (confirmed, not manufactured); `examples/research/viewer_physics_welded.toml` present (5947 B) and is the source-backed reference profile |
| Recording deps | `duckdb`/`pyarrow` absent from `pyproject.toml`/`uv.lock`; no `bike-recording` entrypoint — expected pre-A1 state |

### Launcher inspection (what `full` does on this checkout)

`tools/run_tests.sh`: `full` shares the identical native preflight with
`native` (`preflight_native`): MuJoCo 3.12.0 pin check, `cmake -S/-B`
configure, full build, sweeps `check_frontends|tidy|analyzer|cppcheck|odr|
scripts`, controls `headers|diagnostic-controls|context`, sanitizer-runtime
resolution (inactive for the regular release build), CTest, exact
selected-extension import + provenance, `check_native_contracts`, then one
pytest invocation with `-p native_test_reporter`. The only difference:
`native` restricts pytest to `tests/reference/test_native_*.py` +
`test_golden_episode*.py` with `MallocScribble`/`MallocPreScribble`/
`MallocGuardEdges` exported; `full` runs the entire suite without malloc
scribbling (it measures realtime). `full` therefore already contains the
`native` gate — a separate `native` run adds nothing but the marker
selection. All profiles run serially; unknown profile → exit 2.

### Baseline `full` run — verbatim result

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh full
# log: /tmp/a0-full-baseline.log ; exit code 8
```

Green before the abort: configure+build (all targets), `check_frontends`
(g++-16, findings none), `check_tidy` (findings none; suppressions
`clang-analyzer-cplusplus.Move=1, clang-analyzer-security.ArrayBound=1`),
`check_analyzer` (tool health ok; findings report-only alpha;
suppressions `cplusplus.Move=2`), `check_cppcheck` (findings none),
`check_odr` (findings none), `check_scripts`, `headers` (findings none),
`diagnostic-controls` (6 controls, 0 failed), `context` (tool health ok;
SDK controls explicit/missing/default/fallback/hardening all pass).
No mandatory tool was missing; nothing was skipped.

**ABORT — CTest:** `75% tests passed, 1 tests failed out of 4` →
`Errors while running CTest`, exit 8 under `set -e`. `native_bench_bare`,
`native_bench_glue`, `native_bench_heavy` pass (they resolve
`tools/proto_native_bench/artifacts` → `current.json` → `fixture-v3`).
`native_contract_tests` reports 30 `PASS` case lines plus exactly:

```text
FAIL settlement_sparse_last_atomicity: std::bad_alloc
FAIL drivetrain_allocation_enumeration: std::bad_alloc
```

Selected-extension import, `check_native_contracts` and pytest were never
reached. Since `native` shares this preflight, it aborts identically at
CTest (deterministic — both cases fail `exit=1` run standalone against the
fresh release binary).

**Root cause (disassembly-verified, Release-only test-codegen defect):**
`sweep_allocations` (`native/tests/test_contracts.cpp:2063-2075`) arms the
fault injector with `arm(0)` and probes it with
`std::vector<double> probe(8, 0.); static_cast<void>(probe);` inside
`try`/`catch(std::bad_alloc)`. In the `-O3` release build the probe's
`operator new` is absent from the instruction stream — the dead allocation
is elided, `threw` folds to false, and `fail("the allocation injector
failed to arm")` then constructs its message `std::string` (a 40-byte
`operator new` observed at `sweep_allocations+1904`) while the injector is
still armed at `remaining == 0`. The injected `std::bad_alloc` escapes the
case; the `Guard` disarms on unwind so later cases pass. Exactly one C++
throw occurs per failing process (`lldb` `-E c++`, then exit). In the
unoptimized `native/build` default build the probe allocation is not
elided and both cases pass (`positions` counts 0–9). `native/tests/` is
unchanged since the last recorded green CTest gate (`a743548`, the C4
`full` gate-of-record); the elision drifted in through changed inlined
production headers in `a743548..e3b1d9e` (5 `native/` commits). A
test-harness defect, not a physics/production defect — but a hard red
gate: `full`/`native` cannot reach pytest until it is resolved.

**fixture-v3 resolution:** a prior ledger recorded `full`/`native`
aborting because tracked `current.json` pointed at the then-absent
untracked bundle `fixture-v3`. The bundle directory now exists on disk
(still untracked, user-owned, preserved — manifest: model nq=nv=26,
nu=12, welded config, commit `e3b1d9e`), `current.json` resolves, and all
three `native_bench_*` CTest cases pass against it. That abort cause no
longer reproduces; the current abort is the contract-test defect above.
The pointer dangle risk is unchanged in principle: `current.json` is
tracked while `fixture-v3/` is not, so a clean clone of HEAD still lacks
the bundle. Unresolved, recorded — not "fixed" by committing user files.

## Task ledger

Requirement IDs are taken from the master plan's coverage table.
"Selected artifact" is `native/build/release` unless a row states
otherwise.

| Task | Requirements | Exact commands | Result | Unresolved errors / notes |
|---|---|---|---|---|
| A0 — establish baseline | R23 (baseline portion) | `git rev-parse --show-toplevel`, `git branch --show-current`, `git rev-parse HEAD`, `git status --short`; `NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh full` | Baseline recorded (table above). `full` **exit 8**: preflight green through `context`; CTest 3/4 — `native_contract_tests` fails 2 cases; pytest not reached | `settlement_sparse_last_atomicity` + `drivetrain_allocation_enumeration` escape injected `std::bad_alloc` — Release-only elision of the arm(0) probe's dead `std::vector` allocation; `native` profile equally blocked; `fixture-v3` untracked-but-resolving (clean clones still lack it); plan-file checkboxes intentionally left open |
| A1 — capture contracts, catalog, deps | R01–R03, R08–R10, R12, R14, R15 | (per plan: `bash tools/run_tests.sh quick tests/reference/test_recording_capture.py tests/reference/test_recording_catalog.py`) | not started | |
| A2 — native retention vs measurement, sealing | R04–R10, R14 | (per plan: `NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh native -k 'recording_retention or runtime_accounting or ride_frontend'`) | not started | depends on the A0-recorded CTest gate being green or explicitly scoped |
| A3 — ordinary CLI, lifecycle, journal, check policy | R01–R04, R07, R11, R12, R14, R22 | (per plan: `... run_tests.sh native -k 'recording_lifecycle or recording_cli or ride_frontend'` + `quick` for parser-only tests) | not started | same preflight dependency as A2 |
| A4 — DuckDB storage, reader, publication | R04, R08–R10, R12, R14–R16 | (per plan: `bash tools/run_tests.sh quick tests/reference/test_recording_*.py`) | not started | |
| A5 — report and analyst CLI | R17–R19 | (per plan: `bash tools/run_tests.sh quick tests/reference/test_recording_analysis.py ...`) | not started | |
| A6 — ordinary acceptance + docs | R05, R06, R17–R19, R23, R24 | (per plan: `NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh full`) | not started | final gate currently red per A0 row |
| B1 — research evidence owner + suffix | R04–R11, R13, R14, R21 | (per plan: `... run_tests.sh native -k 'research_capture or ...'` + `quick test_research_capture.py`) | not started | |
| B2 — research CLI finalization + normalized streams | R01–R04, R11–R16, R21, R22 | (per plan: `... run_tests.sh native -k 'research_recording_lifecycle or ...'` + `quick` files) | not started | |
| B3 — versioned replay + legacy compatibility | R04, R12, R13, R20, R21 | (per plan: `... run_tests.sh native -k 'replay_recording_v3 or ...'` + `quick` files) | not started | |
| B4 — cross-mode acceptance + handoff | R05, R06, R13, R17–R21, R23, R24 | (per plan: `NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh full`) | not started | final gate currently red per A0 row |

## Standing notes

- `full` includes the complete `native` preflight on this checkout, so the
  A6/B4 `full` gates also evidence the native gate; a red `full` keeps
  both open.
- The A0 CTest failure is recorded, not waived: no gate was weakened and
  no untracked user file was committed. Any later "green" claim must cite
  a rerun against the final code.
- Prior documented platform-baseline pytest failures (24 IDs in
  `docs/superpowers/audits/2026-10-08-native-visual-runtime.md`) could not
  be re-evaluated: pytest was never reached. They remain a historical
  expectation, not current evidence.
