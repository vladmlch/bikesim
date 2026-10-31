# Task C329b2 — test-port 3.29b2 report (check-tools / contract-checks remainder, T53)

**Batch**: 3.29b2 — the remaining records of
`tests/reference/test_native_check_tools.py` (51) and
`tests/reference/test_native_contract_checks.py` (5), all ported into the
`cmake -P` CTest script fixture `native/tests/migration/migration_T56_check.cmake`
(CTest `migration_T56`, labels `migration;t56`).

**Lane**: `.worktrees/pfc-lane-e`, branch `lane/e`, base `d20ea74c`
(`fix(deps): loosen requires-python to >=3.13`). No merge, no push.

## Status: landed — 56 `port`, 0 `drop`, 0 `planned`

All 56 remaining records ported into `migration_T56` run_case blocks; both
source files now hold `port` on every record (88 + 22). **No golden
fixtures were needed or created**: the batch is contract-only — tool
resolution, schema-2 context validation, compile-database selection,
contract-checker provenance and RTSan/coverage configure gates are all
asserted against produced `bike-native-checks`/`bike-python-audit` output,
not live-oracle comparisons.

## Triage

| Source file | records | port | drop |
|---|---|---|---|
| `test_native_check_tools.py` | 51 | 51 | 0 |
| `test_native_contract_checks.py` | 5 | 5 | 0 |
| **total** | **56** | **56** | **0** |

## Port map (`migration_T56::*`, batch block at `migration_T56_check.cmake:2658+`)

- **Tool override matrix** — `b29_override_{tidy,analyzer,cppcheck,frontends,odr}`:
  a nonexistent required override fails resolution before an empty sweep can
  be reported passing (`invalid <kind> override`, summary `status=failed`,
  `translation_unit_count=0`). ← `test_broken_required_override_fails_before_sweep`
- **Wrong-family rejection** — `b29_wrong_family`. ← `test_wrong_family_tool_is_a_selection_error`
- **Compiler-sibling preference** — `b29_tidy_sibling`, `b29_gxx_sibling`
  (versioned `clang++-18`/`g++-13` sibling chosen over PATH defaults).
  ← `test_tidy_prefers_configured_compiler_sibling_over_default`,
  `test_gxx_prefers_gnu_compiler_sibling_over_default`
- **Schema contexts** — `b29_schema1_supported`,
  `b29_schema2_host` (Darwin stdlib `libc++`, `sdk == sysroot`; Linux stdlib
  membership `libc++|libstdc++`, `sdk`/`sysroot` null), plus the fail-closed
  battery `b29_schema2_*`/`t56_ctx_schema2_*` (missing platform, host
  mismatch, bad stdlib, stdlib-mode mismatch, missing numerics, Linux SDK
  `not-applicable`, Darwin sysroot drift, sanitizer record/runtime/compiler-
  identity consistency, empty evidence objects).
  ← `test_schema1_context_remains_supported`, `test_schema2_context_loads_on_matching_host`,
  `test_schema2_linux_context_loads_and_records_no_sdk`,
  `test_schema2_linux_context_must_not_record_sdk`, `test_schema2_*` rejections
- **Linux compile-database selection** — `b29_linux_*`: absent db, missing /
  duplicate TU by name, stale compiler argv[0], wrong `-o` target context,
  GNU-dialect `-std`, omitted required option, relative include path,
  malformed `arguments`/`command`/no-cmd entries, entries-without-sysroot
  acceptance, explicit-sysroot-directory acceptance. ← `test_schema2_linux_*`
- **Library-path validation** — `b29_library_path_{missing,dir}`.
  ← `test_schema2_linux_invalid_library_path_fails`
- **Support-module parity** — `b29_support_modules` (4-module parity with the
  Python reference list). ← `test_native_support_modules_match_the_python_reference`
- **Contract-checker provenance** — `b29_prov_malformed` (`malformed native
  loader provenance`), `b29_prov_wrong_build` (`selected build does not
  match`). ← `test_coverage_rejects_malformed_provenance`,
  `test_coverage_rejects_a_provenance_for_a_different_build`
- **Coverage gates** — `b29_cov_flag_reject` (clang-family flag-reject
  shadow), `b29_cov_non_clang` (GNU driver rejected at the family gate —
  Linux-only; Darwin skips by contract), `b29_cov_bad_override`
  (unresolvable `LLVM_PROFDATA`), `b29_cov_merge_fail` (broken profdata
  reports command+log), `b29_cov_qualified` + `b29_cov_tool_evidence`
  (qualified lane records resolved llvm-profdata/llvm-cov evidence),
  `b29_run_coverage_no_tests` (`RunCoverage.cmake` fails when the selected
  CTest set produces no run). ← `test_requested_coverage_*`,
  `test_qualified_coverage_configure_records_tool_evidence`,
  `test_coverage_records_tool_resolution_evidence`,
  `test_coverage_failing_merge_tool_reports_command_and_log`,
  `test_coverage_requires_zero_excluded_native_tests`
- **RTSan gates** — `b29_rtsan_flag_reject`, `b29_rtsan_lacking`,
  `b29_rtsan_unrequested` (unrequested lane records `requested:false,
  supported:false, enabled:false, runtime:null` with probe evidence),
  `b29_rtsan_no_runtime`, `b29_rtsan_qualified`, `b29_rtsan_control`
  (violation diagnostic contains `RealtimeSanitizer`+`malloc`; skipped with
  an explicit message when no RTSan-capable compiler exists).
  ← `test_requested_rtsan_*`, `test_qualified_rtsan_configure_enables_the_lane`,
  `test_unrequested_rtsan_capability_is_explicitly_unsupported`,
  `test_rtsan_control_source_reports_the_sanitizer_diagnostic`

## Fixture repairs made while qualifying

- `_ctx` reuse clobber (pre-existing block wrote JSON content over the
  context-path variable): dedicated `_b29_ctx` path variable for the new
  schema-2 host assertions.
- Compiler candidate dedup changed from `file(REAL_PATH)` to the invoked
  path: REAL_PATH collapses `clang++` onto the `clang`/`clang-23` real
  binary, and the driver is argv[0]-sensitive — `clang` does not link the
  C++ stdlib, so a symlink-resolved "compiler" cannot pass the CMake
  compiler test on either platform.
- `b29_linux_case` callers write `compile_commands.json` per case;
  `b29-linux-missing` was missing its DB write (only `b29-linux-no-db`
  intentionally has none).
- Legacy `expect_json*` helpers gained the same `EXISTS` guard the new
  helpers already had — a missing file now records a fixture failure
  instead of aborting aggregation.
- `b29_tidy_wrapper_missing_build` pins `BIKE_NATIVE_CHECKS=${CHECKS_BINARY}`
  so the wrapper case does not depend on the repo-root fallback existing
  inside the lane container (build trees live under `/build`, not `/src/build`).
- Coverage-driver selection now requires a compile+link probe of
  `-fprofile-instr-generate -fcoverage-mapping`: resolved llvm-profdata/
  llvm-cov alone do not qualify the lane when the profile runtime is absent
  (the container's llvm-18 lacks `libclang_rt.profile-aarch64.a`; the four
  qualified-lane cases skip explicitly there and run on Darwin).
- `native/cmake/Checks.cmake`: `migration_T56` gained
  `LABELS "migration;t56" TIMEOUT 3600` (the real-tree headers sweep inside
  the fixture exceeds the 1500 s default).

## Verification

| Command | Result |
|---|---|
| `ctest --test-dir build/native-release -R "^migration_T56$" --output-on-failure` (darwin-arm64, AppleClang/libc++) | **PASS — 1630.55 s** |
| `container.sh ctest root-clang -R '^migration_T56$'` (linux-aarch64 container `bike-dev-pfc-lane-e`, `/build/root-clang`, clang++-18) | **PASS — 1451.05 s** |
| `bash tools/run_tests.sh native` (final gate, darwin-arm64, run 1) | exit 8 — **environmental**: `/System/Volumes/Data` at 100% capacity; CTest could not write `LastTestsFailed.log`, `migration_T03` scratch writes hit `No space left on device` (T09/T10 oracle reds also present but untrusted under disk exhaustion) |
| `bash tools/run_tests.sh native` (final gate, darwin-arm64, run 2 — after freeing ~3 GiB of this lane's scratch) | exit 8 — **baseline-blocked**: build + all 9 static-analysis sweeps green (analyzer report-only); `ctest --test-dir native/build` **60/62** — only the documented baseline pair `migration_T09` (`post-advance integration vector matches the oracle`) + `migration_T10` (`first transition matches the Python oracle`), identical signatures to the `cpp-linux-completion-evidence.md` §3.31 final-gate baseline; `migration_T03` passed 187.63 s; zero unexpected reds |
| `pytest tests/reference/test_native_check_tools.py tests/reference/test_native_contract_checks.py` (scoped gate-tail resume; the two source files remain live until T63) | first attempt: 63 passed, 3 failed, 60 errors — **every** failure/error was `OSError: [Errno 28] No space left on device` inside sweep log writes / `work_root.mkdir` while parallel lanes re-exhausted the volume (verified in tracebacks, e.g. `native_check_logs/headers/…/5906-project_config-bike_viz.stdout.log`, `native_check_work/context/20261031T135651Z`); `--lf` rerun of the identical 63-node set with free space restored: **62 passed, 1 skipped** (platform-gated) in 2:00:08 — zero real assertion failures |

Linux notes:

- `/build/root-ref` (GNU 13.3.0) cannot build `native_contract_tests`:
  `bike_core` TUs (`drivetrain/state_codec.cpp`, `writer_config.cpp`,
  `runtime/wire.cpp`, `control.cpp`, `bootstrap_io.cpp`, …) trip the
  documented GCC-13 `-Werror=null-dereference` false-positive class inside
  libstdc++ inlining — the same class recorded in
  `migration/cpp-linux-completion-evidence.md` (11 production TUs; the
  qualified lane uses `clang++-18`). Verification therefore ran on
  `/build/root-clang` (`configure root-clang root
  -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18`).
- `/opt/bike-venv` had to be provisioned (`container.sh venv`) so scratch
  configures get a Python ≥3.13 with the project dependency surface —
  `/usr/bin/python3` is 3.12 and fails the `find_package(Python)` gate.
- RTSan-capable compiler: none on the container (clang-18 predates upstream
  RTSan) → `b29_rtsan_{no_runtime,qualified,control}` skip with the explicit
  message; `b29_rtsan_unrequested` runs with the lacking driver. Coverage
  qualified-lane cases likewise skip (no `libclang_rt.profile`). Both run on
  Darwin.

## Final gate detail

Single-command exit-0 is unreachable while the documented baseline pair
`migration_T09`/`migration_T10` fails — the same 60/62 signature recorded
for every lane gate since the pair was documented (see
`migration/cpp-linux-completion-evidence.md` "Final gate" entries). The
first run of this batch's gate was corrupted by a full filesystem
(423 MiB free → `No space left on device` in the CTest phase, T03
collateral); after reclaiming ~3 GiB of this lane's own scratch
(`build/native-release/native_check_work`, `migration_T03_work`,
`native/build/tests`, `/private/tmp/t53-*`, source tarball) the rerun
completed cleanly: both graphs configured and built, all sweeps green,
and the failure set is exactly `{T09, T10}` — identical assertions to
the documented baseline, so **zero unexpected reds attributable to this
batch**. This batch changes no production behavior (fixture script,
CTest registration, ledger JSON, one `NOLINT` comment), so the baseline
mapping is sound. The gate's pytest tail does not run after the CTest
abort; the batch-relevant portion — the two source test files, still
live until T63 — was resumed standalone and is reported above.

## Ledgers

- `migration/test-port-map.json`: 56 records → `disposition: port`,
  `native_test_id: migration_T56::<case-list>`, `native_source:
  native/tests/migration/migration_T56_check.cmake`,
  `comparison_class: contract`, measured collected nodes + assert counts,
  evidence lines citing the green CTest runs above.
- `migration/python-removal.json`: both rows → `status: replacement_landed`,
  `replacement_paths` pointing at the actual carrier
  (`native/tests/migration/migration_T56_check.cmake`,
  `native/tests/rtsan_control.cpp`), `verification_ids` include
  `migration_T56`.
- `migration/ctest-graph-map.json`: no change — it maps binding→root target
  names, not pytest files.
