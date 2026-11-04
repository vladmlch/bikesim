# Verification speed-up — baseline audit (task 0)

Date: 2026-11-03. Branch: `main3`. Plan:
[docs/superpowers/plans/2026-11-03-verification-speed.md](../plans/2026-11-03-verification-speed.md).
Design: [docs/superpowers/specs/2026-11-03-verification-speed-design.md](../specs/2026-11-03-verification-speed-design.md).

## Working copy

- HEAD: `352f51e0fbf4f2850fa8fb9a35a9e182fd201fd4` (`feat(pedaling): restore the reposition config knobs (default off)`)
- Untracked: `.agents/skills/cass/`, `.devin/skills/`, `.junie/`,
  `docs/superpowers/plans/2026-11-03-crank-reposition-native.md`,
  `docs/superpowers/plans/2026-11-03-verification-speed.md`,
  `docs/superpowers/specs/` — left alone, as expected.
- Host: macOS (Darwin), 16 online cores.

## Ordinary tests (ctest -LE 'slow|package' -j8)

Command: `cmake --preset release && cmake --build --preset release && ctest --test-dir build/native-release -LE 'slow|package' -j8 --output-on-failure`

Result: `99% tests passed, 2 tests failed out of 183`, `Total Test time (real) = 46.81 sec`.

Failures (the two the plan names — pre-existing, fixed in tasks 1 and 2):

- `208 - reference.native_research_recording (Failed)`
- `230 - reference.native_numeric_portability (Failed)`

## Tooling tests

Command: `ctest --test-dir build/native-release -R '^(tooling_checks|tooling_platform|tooling_toolchain|tooling_install)$' --output-on-failure`

| Test | Result | Time |
|---|---|---|
| tooling_platform | Passed | 137.66 s |
| tooling_toolchain | Passed | 2.54 s |
| tooling_install | Passed | 0.41 s |
| tooling_checks | **Failed** | 1401.99 s |

`75% tests passed, 1 tests failed out of 4`; `Total Test time (real) = 1542.62 sec`.

`tooling_checks` fixture-contract failures, verbatim (pre-existing — recorded, not
fixed; tasks 3, 5, 7 compare against this list):

    t56_ctx_binding_null_interpreter: stderr missing 'configured Python executable is missing or is not a path':

    t56_ctx_binding_missing_interpreter: stderr missing 'configured Python executable is missing or is not a path':

    t56_ctx_binding_null_identity: stderr missing 'binding native check context python must be an object':

    t56_contracts_stale_extension: stderr missing 'expected exactly one bike_native extension':
    native contract checks failed: cannot read CTest registration file
    build/native-release/tooling_checks/contracts-stale-extension/CTestTestfile.cmake: unreadable

    b29_ext_zero: stderr missing 'expected exactly one bike_native extension':
    native contract checks passed: build/native-release/tooling_checks/b29fx-extzero/build
    (contract list printed; engine_call_wrappers: pending)

    b29_rtsan_qualified: control build failed (2):
    make: *** No rule to make target `native_rtsan_control'.  Stop.

Also observed: `b29_cov_non_clang: GNU driver on Darwin is rejected before the
coverage-family gate — skipped` (a skip, not a failure).

## After task 5

Command: `time build/native-release/bin/bike-native-checks --kind analyzer --build build/native-release`

Result (single stable pass; alpha checkers removed): `real 27m33.112s`
(`user 26m15.122s`, `sys 0m49.598s`), exit 0 —
`tool health: ok`, `findings: none` (suppressions: cplusplus.NewDelete=1).
Roughly half the old ~60 min two-pass run, matching the ~27 min estimate.

### Task 7 sweep timings

Command per kind: `/usr/bin/time -p build/native-release/bin/bike-native-checks
--kind <k> --build build/native-release`. Default workers (BIKE_CHECK_JOBS
unset → 16 online cores). Every kind ended `tool health: ok`,
`findings: none`, `jobs: 16`; per-TU record counts match the serial shape
(odr includes the trailing link record).

| kind | wall (s) | records |
|---|---|---|
| cppcheck | 11.76 | 512 |
| frontends | 54.74 | 512 |
| headers | 90.68 | 2380 |
| odr | 123.72 | 513 |
| tidy | 220.29 | 512 |
| analyzer | 199.39 | 512 |
| **total** | **~700.6 (~11.7 min)** | |

Serial phase was ~137 min; the pool phase is ~11.7 min (~11.7x). cppcheck
and headers were re-run on the final binary after a source fix (a
`by-value HeaderProbe` + `std::scoped_lock` + designated-initializer
adjustment silenced one cppcheck `objectIndex` and two clang-tidy
diagnostics on the new probe code); the table shows the final runs.
Swap: `vm.swapusage` reported 2.4 G of 4 G in use during the sweep phase —
the machine did not throttle the default 16 workers.

### Task 9 header dedup timing

Command: `/usr/bin/time -p build/native-release/bin/bike-native-checks
--kind headers --build build/native-release`. Default workers (16).

Selection line: `native headers selection: 260 headers, 751 header
translation units (2380 contexts, 1629 duplicates skipped)`;
`tool health: ok`, `findings: none`.

| kind | wall (s) | records |
|---|---|---|
| headers (task 7) | 90.68 | 2380 |
| headers (task 9) | 33.27 | 751 |

### Task 10 ccache

Command: `ccache -z && cmake --preset release -B /tmp/bike-ccache-second &&
time cmake --build /tmp/bike-ccache-second && ccache -s`. The first build in
`build/native-release` (launcher rule change → full 783-step rebuild,
cold misses) took `real 1m32.812s`; the second build into
`/tmp/bike-ccache-second` took `real 0m25.317s`.

| metric | value |
|---|---|
| Cacheable calls | 513 / 513 (100.0%) |
| Hits | 491 / 513 (95.71%) |
| Misses | 22 / 513 (4.29%) |
| First build (cold) | 92.8 s |
| Second build (warm) | 25.3 s (~3.7x) |

The 22 misses are stable across fresh build dirs (re-measured with
`CCACHE_LOGFILE` on a third dir): exactly the TUs whose compile line embeds
the build directory — `-DBIKE_BINARY=<builddir>/...` defines and
`-I<builddir>/...` include dirs. The path leaks into the preprocessed
output (macro expansion and `#` linemarkers), so a different `-B` can never
hit them: `gpu/*` + `tests/gpu/*` (BIKE_BINARY), `platform/{blas,lapack}_accelerate`
(`-I bike_numerics`), `runtime/{provenance,realtime_options,realtime_provenance}`
(`-I research_identity|measurement`; `measurement/bike_measurement_paths.hpp`
also genuinely differs), `viz/html` (`-I viz`), and the test TUs under
`-I native/tests/{support,reference/support}` (the generated
`*_paths.hpp` headers there embed build paths too).

### Task 12 level-1 change check

Command: `time bash tools/check_change.sh main3~3` (warm tree; changed set =
3 listed files — `checks_main.cpp`, `checks_internal.hpp`,
`check_frontends.cpp` — expanding to 6 TUs once the four readers of
`checks_internal.hpp` join through the include walk). Result: `real
1m16.613s` (`user 4m31.705s`, `sys 0m36.287s`), exit 0 — inside the ~5 min
level-1 budget. Script-reported `check_change: passed in 77s`.

| part | wall (s) |
|---|---|
| `cmake --build --preset release` (incremental, ~211 steps: 1 TU + relinks) | ~8 |
| `ctest --preset release` (185/185 passed, `Total Test time (real) = 46.52 sec`) | 46.5 |
| `cppcheck --only-sources` (6 TUs, health ok, findings none) | ~3 |
| `frontends --only-sources` (6 TUs, health ok, findings none) | ~4 |
| `tidy --only-sources` (6 TUs, health ok, findings none) | ~15 |
| **total** | **76.6** |

Partial summaries land in `native_check_summaries/partial-<kind>.json` with
`"selection":"partial"`; the full `<kind>.json` files were not touched.

## After

Final gate (task 13, 2026-11-04, HEAD `279ed980`): one complete
`bash tools/run_tests.sh native` run, exit 0 — **831 s (~13.8 min)** on an
idle 16-core machine. The task-10 gate on the same code took 982 s; both
sit well inside the ≤30 min target. Log: `/tmp/verification-speed-final.log`.

| stage | before (serial baseline, design doc) | after (this run, 16 workers) |
|---|---|---|
| configure | — | 11 s |
| build (incremental) | — | 0 s |
| ctest (ordinary; runs first now) | ~1 min | 44 s (185/185 passed) |
| scripts (shellcheck) | not in baseline table | 1 s |
| cppcheck | ~1 min | 11 s |
| context | not in baseline table | 52 s |
| diagnostic-controls | not in baseline table | 3 s |
| contracts | not in baseline table | 4 s |
| frontends | ~9 min | 57 s |
| headers | ~15 min (2380 contexts) | 43 s (751 deduplicated TUs) |
| odr | ~19 min | 147 s |
| tidy | ~33 min | 241 s |
| analyzer | ~60 min (stable + alpha halves) | 217 s (single stable pass) |
| **total** | **sweep phase ~137 min serial** | **831 s (~13.8 min)** |

`tooling_checks` (the CTest fixture that re-runs every sweep kind on a toy
build): 1402 s at baseline → 385 s after the pool.

Level-1 change check (task 12): `bash tools/check_change.sh` measured
**76.6 s** on a warm tree — inside the ≤5 min budget (table above).

Journal line appended by this run
(`$(git rev-parse --git-common-dir)/bike-gate/timings.tsv`):

    2026-11-04T08:28:29Z	/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/main3	279ed980	native	release	-	16	passed	831	configure=11,build=0,ctest=44,scripts=1,cppcheck=11,context=52,diagnostic-controls=3,contracts=4,frontends=57,headers=43,odr=147,tidy=241,analyzer=217

Deferred:

- **Task 10, variant Б (ccache off by default).** The owner ruling took
  variant A: `BIKE_COMPILER_CACHE=ON` auto-enables ccache when installed
  and records it as `compiler.build_launcher`. The off-by-default variant
  stays reachable through `-DBIKE_COMPILER_CACHE=OFF`; flipping the
  default is a policy call, not a code change, so it is left to the owner.
- **Task 11 standalone analyzer decision.** The measurement report
  [2026-11-03-analyzer-vs-tidy.md](2026-11-03-analyzer-vs-tidy.md) found
  the analyzer-only checker set empty, yet the standalone pass still
  surfaces the T15 `cplusplus.NewDelete` control that clang-tidy misses
  on this toolchain — for ~4.1 min per gate. Options A (keep), B (drop),
  C (`full` only) are tabled; the decision is pending the owner.
- **`slow`-label parallelization.** Out of scope per the design's
  not-included list: it first needs a "measures machine speed / does not"
  labeling pass over the 49 slow tests (~46 min serial today).

Open tracked failures (the reason `AGENTS.md`'s "no tolerated baseline"
rule points here): the six `tooling_checks` fixture cases recorded
verbatim in the baseline section above (`t56_ctx_binding_*` ×3,
`t56_contracts_stale_extension`, `b29_ext_zero`, `b29_rtsan_qualified`)
remain unfixed — pre-existing defects in the fixture contracts, outside
this plan's scope. They keep `tooling_checks` red and therefore gate the
`full` profile's `slow` stage; fixing them is tracked by this record
until a dedicated task takes them.
