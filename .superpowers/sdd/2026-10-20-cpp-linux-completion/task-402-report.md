# Task 4.02 — `tools/run_tests.sh` on the native graph only

**Lane**: `.worktrees/pfc-lane-e`, branch `lane/e3`, integration tip
`907687f1d4c6d6b7c112c97cd05bc08a260c47a7`. DARWIN-ONLY (macOS arm64,
AppleClang, Accelerate numerics). No container use.

## Profile → target mapping (as implemented)

`tools/run_tests.sh <profile> [preset]` (134 lines, `set -euo pipefail`,
usage text kept). `preset` defaults to `release`; `<build>` is the preset's
binaryDir — `build/<preset>`, `release` → `build/native-release` (Task 3.41).
`native asan` runs the same stages on `build/asan`; sanitizer runtimes come
from the preset's link flags, not a preload.

| profile | stages |
|---|---|
| `quick` | `cmake --workflow --preset <preset>` — configure + build + test step; the `release` test preset inherits `_tests` (`exclude.label: slow\|audit\|package`, `outputOnFailure`, `noTestsAction: error`, JUnit → `build/native-release/ctest-junit.xml`) |
| `native` | `cmake --preset <preset>` → `cmake --build --preset <preset>` → `tools/run_native_checks.sh <build>` (every produced sweep kind + the `contracts` kind, via `bike-native-checks`) → `ctest --test-dir <build> -LE 'slow\|audit\|package' -j8 --no-tests=error --output-on-failure --output-junit <build>/Testing/junit.xml` |
| `full` | `native`, then explicit `ctest --test-dir <build> -L slow`, `-L audit`, `-L package`, each `--no-tests=error --output-on-failure` so an absent registration fails instead of passing silently |

Darwin allocator debugging (`MallocScribble=1 MallocPreScribble=1
MallocGuardEdges=1`) wraps only the ordinary `-LE` CTest leg in a subshell via
`run_ordinary_leg` — in `quick` that is the workflow invocation (its test step
*is* the `-LE` leg; configure/build ignore the variables), in `native`/`full`
it is the literal `ctest --test-dir <build> -LE …` command. Never exported on
Linux, never around `ctest -L slow` which contains realtime measurements
(Task 3.10).

`NATIVE_TEST_BUILD_PATH` is still exported to the selected build directory
before the sweep/CTest legs: consumers honoring the selection convention see a
consistent environment and the `contracts` kind rejects a mismatched value.
The `[preset]` argument replaces `NATIVE_TEST_BUILD_PATH` /
`NATIVE_TEST_BUILD_DIR=asan|rtsan|coverage` as the build selector.

Header-comment successor map (required by the plan's review focus):

- extension configure/build → `cmake --preset` / `--build --preset` on the
  root graph (no bindings extension remains to select or import);
- `check_<name>` sweep targets → `tools/run_native_checks.sh <build>` driving
  the produced `bike-native-checks` binary in fixed contract order;
- `native_runtime` preload → preset sanitizer link flags (Task 3.42);
- native/golden pytest legs → the `reference` CTest label set;
- `NATIVE_TEST_PROVENANCE_PATH` / `native_test_reporter` plugin → CTest's
  `<build>/Testing/` tree plus `<build>/Testing/junit.xml` (Task 3.45 CI lanes
  upload both).

No `python`, `pytest`, `uv run` or `uv sync` token remains in the script —
the plan's Step-3 grep returns no match, and the produced
`bike-python-audit`/`scripts` sweep inside `migration_T56` (boundary-matched
`invocation_tokens`) ran green over this exact text.

## `slow` labels added (measured >30 s)

Measured under `-j8` on this host from the full `-LE` run; the plan's `-j1`
method gives the same set — every labeled test exceeds 30 s by >2× while the
next-highest test (`migration_T23`, 17.90 s) is comfortably under, so no
borderline cases exist.

| test | measured s | registration site | change |
|---|---:|---|---|
| `migration_T56` | 1926.95 | `native/cmake/Checks.cmake:193` | `LABELS "migration;t56;slow"` |
| `migration_T53_portability` | 527.02 | `native/cmake/Checks.cmake:100` | `LABELS "migration;t53;portability;slow"` |
| `migration_T03` | 339.13 | `native/cmake/Provenance.cmake:410` | `+LABELS "migration;t03;slow"` (existing `TIMEOUT 1200` kept) |
| `migration_T53_numerics` | 110.02 | `native/cmake/Checks.cmake:119` | `+set_tests_properties LABELS "migration;t53;numerics;slow"` |
| `reference.native_engine_errors` | 69.77 | `native/tests/reference/Phase3Ports.cmake:211` | `LABELS t53 checks slow` |

- `native/tests/migration/CMakeLists.txt`: **no change needed** — every
  executable registered there measured <30 s (max `migration_T23` 17.90 s).
  The slow script tests in the migration family register from
  `Checks.cmake`/`Provenance.cmake` instead; edits went to the actual
  registration sites.
- `CMakePresets.json`: **no change needed** — the `_tests` base preset
  already carries `"filter": {"exclude": {"label": "slow|audit|package"}}`
  and `release` inherits it. `-L slow` went 44 → 49 tests; the `-LE` set
  went 186 → 181.

## `AGENTS.md` Verification section (plan Step 4)

Profiles now describe the C++ gate; sweep reruns are
`build/native-release/bin/bike-native-checks --kind <kind> --build
build/native-release` or `cmake --build build/native-release --target
check_<name>`; the Numba paragraph was removed per Step 4 and replaced with
the required one-line note ("removed with Python, see ADR-0003").

## Ledger flips (`migration/python-removal.json`)

T57-owned rows whose replacements are verifiably landed at this tip
(plan maps T57 → 3.42/3.43/4.02):

- `tools/native_runtime.py` → `replacement_landed`; verification_ids:
  `migration_T336_portability`, `sanitizer_clean_control`,
  `sanitizer_negative_control`, `rtsan_clean_control`,
  `rtsan_negative_control`.
- `tools/dev.py` → `replacement_landed`; verification_ids:
  `migration_T336r_portability`, `install_smoke`, `package_smoke`,
  `coverage_smoke`.

Not flipped: `tools/native_checks.py` (its T56 row covers a broader surface
than this task; remains `planned`), `tools/native_coverage.py` and the
conftest/`native_loader`/`native_test_reporter` rows (their declared
`replacement_paths` name files not yet landed). Counts: 175
`replacement_landed` (+2), 360 `planned` (−2).

## Verification transcript (darwin-arm64)

```
grep -nE 'python|pytest|uv run|uv sync' tools/run_tests.sh   → no match (exit 1)
bash -n tools/run_tests.sh                                    → clean
shellcheck tools/run_tests.sh                                 → clean
git diff --check                                              → clean
bash tools/run_tests.sh bogus / quick extra / native badpreset→ exit 2 + usage
cmake --preset release                                        → ok (14.2 s)
ctest --test-dir build/native-release -N -L slow              → Total Tests: 49 (>0)
bash tools/run_tests.sh quick                                 → exit 8
  cmake --workflow --preset release: configure → build → test,
  181 tests in 73.16 s, 9 failures (list below)
bash tools/run_tests.sh native                                → exit 1
  configure ok → build ok (778 targets) → run_native_checks.sh
  fails at the first sweep kind (frontends, see blockers)
ctest -LE 'slow|audit|package' -j8 … (the script's exact leg) → exit 8
  186 tests, 9 failures — the same set `quick` reports
```

## Terminal state vs the expected baseline

Expected terminal per the task: `native` exit 8 with **only** T09/T10 red.
Actual at integration tip `907687f1` — honest result, not the target state:

1. `native` exits **1** before CTest: the `frontends` sweep (first produced
   kind in `run_native_checks.sh` order) reports **123 diagnostics across 37
   translation units** (`misleading-indentation` ×52,
   `missing-field-initializers` ×34, `useless-cast` ×26,
   `dangling-reference` ×7, `type-limits`/`redundant-tags`/
   `range-loop-construct` ×1 each, plus one untagged error) —
   "tool health: failed", "findings: blocking". The debt lives in
   `native/src/validation/*`, `validate_main.cpp` and reference test/support
   TUs (`rider_control_reference_support.hpp`,
   `replay_reference_support.hpp`, …) introduced by the PHASE-3 merges that
   were never swept. The gate correctly blocks at the first failing stage —
   the same fail-fast semantics the previous script had; logs land under
   `build/native-release/native_check_logs/frontends/<utc>/`.
2. The CTest leg (script-exact invocation, run standalone) exits **8** with
   9 failures / 186:
   - `migration_T09`, `migration_T10` — the documented baseline pair.
   - `reference.native_numeric_completion` — config fixture missing
     `rider_controller.leg_track_damping_nms_rad`.
   - `reference.pneumatic_tire_oracle` — "fixture is not a regular file"
     (documented pneumatic fixture gap).
   - `reference.native_drive_policies` — golden mismatch
     `$.target_rate_rad_s`.
   - `reference.native_atomicity` — atomic-shift golden mismatch.
   - `reference.native_runtime_commands` / `…_rollout` — interval-120
     golden mismatches.
   - `reference.native_research_environment` — transition-87 golden
     mismatch; FT-2 green, committed golden stale after FT-3/FT-4
     trajectory changes.

   All seven extra reds are pre-existing at the tip: this diff touches no
   physics, test, or golden code — only the gate script, three label
   registrations, `AGENTS.md` and two ledger status fields.

## Blockers (not caused by this task; owned elsewhere)

- **frontends sweep**: 123 `-Werror` diagnostics across 37 TUs in the
  PHASE-3 validation sources must be cleared by the owning lane before
  `native`/`full` can reach their CTest leg. Blocking by policy
  (tool-health/stability class).
- **7 non-T09/T10 CTest reds**: stale committed goldens + the pneumatic
  fixture gap + the numeric-completion config key — owning tasks pending.
- T09/T10 remain the sanctioned baseline pair until their own task lands.

## Commit

`feat(4.02): run_tests.sh drives the native graph only`
(+ `Co-Authored-By: Devin`)
