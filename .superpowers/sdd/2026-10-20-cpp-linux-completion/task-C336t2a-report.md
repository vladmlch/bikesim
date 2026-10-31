# Task C336t2a — test-port 3.36 tranche A report (portability tooling; T53)

**Batch**: 3.36t2a — the six remaining `tests/portability/` tooling files:
`test_native_toolchain.py` (29), `test_native_runtime.py` (39),
`test_offline_bundle.py` (35), `test_offline_install.py` (28),
`test_offline_tools.py` (14), `test_qualification_report.py` (43) —
188 records total, all `planned` at batch start.

**Lane**: `.worktrees/pfc-lane-a`, branch `lane/a`, base `d20ea74c`
(post 3.36b merge-back tip). No merge, no push. Commits on `lane/a`:
`fe5f65ac` (stash provenance: native-checks launcher preserve +
inspect_tool surface + warning-flag probe), `44af459e` (native contract
+ Checks registration), `175d4ff8` (ledgers), `a24d26f6` (documented
analysis suppression for a baseline-red tidy finding).

## Status: 22 `port`, 0 `golden`, 166 `drop`, all 188 records classified

| file | records | port | drop |
|---|---|---|---|
| test_native_toolchain.py | 29 | 9 | 20 |
| test_native_runtime.py | 39 | 13 | 26 |
| test_offline_bundle.py | 35 | 0 | 35 |
| test_offline_install.py | 28 | 0 | 28 |
| test_offline_tools.py | 14 | 0 | 14 |
| test_qualification_report.py | 43 | 0 | 43 |

All six `python-removal.json` rows are `replacement_landed`.

Ported surface: **one `cmake -P` CTest contract script** —
`native/tests/migration/migration_T336_portability.cmake` (625 lines,
12 cases) — registered as `migration_T336_portability` in
`native/cmake/Checks.cmake` (root graph only, like the T53 scripts;
`migration;t336;portability` labels, 1200 s timeout, convenience target
DEPENDS `bike-native-checks`), plus three **pre-existing registered
controls** the runtime records map to (`sanitizer_clean_control`,
`rtsan_clean_control`, `sanitizer_negative_control` —
`native/tests/{sanitizer,rtsan}_control.cpp`, wired in
`native/cmake/Sanitizers.cmake`).

The script drives real surfaces, exactly like the Python tests it
replaces:

- `--inspect-tool PATH` on the produced `bike-native-checks` (the new
  CLI surface landed in `fe5f65ac`) — schema-1 inspection record with
  family/resolved_path/version query/target evidence; rejections of a
  missing tool, a silent stub, a non-tool (`python3`/`sh`), and a
  wrong-family driver, each pinning the oracle's wording.
- Real negative-compile fixtures through the host compiler: `-Werror=
  unused-result` warning diagnostic, non-self-sufficient header
  diagnostic, invalid-source rejection — the same observable contract
  as the Python control runs.
- stdlib-hardening out-of-bounds controls as **two faithful probes**:
  `_GLIBCXX_ASSERTIONS` libstdc++ (hardened binary must trap — signal
  or assertion-bearing diagnostic; the unhardened binary's UB outcome
  is recorded, never asserted) and libc++ `EXTENSIVE` hardening
  (Darwin contract; benign run must pass, violation run must not exit
  0) — mirroring the Python sources' distinct stdlib controls.
- Sanitizer-lane invariants read from the emitted
  `native_check_context.json` (schema 2): the ASan lane's recorded
  runtime must be absolute, platform-correct (`asan_osx_dynamic.dylib`
  on Darwin, `.so` on Linux) with `supported=ON` + compiler_id/target
  evidence — or the lane is coherently `supported=OFF` with no runtime;
  the RTSan lane is either supported with its runtime recorded and
  `enabled == requested`, or fails closed (`enabled=OFF`, null
  runtime) — the preload resolver's gate, now at configure time.
- `NATIVE_SANITIZE=ON` + `NATIVE_RTSAN=ON` is a configure-time
  `FATAL_ERROR` — the mutual exclusion the removed env composer and
  cache-staleness checks enforced — via a scratch `project(... CXX)`
  configure including the real `BikeCompilerPolicy.cmake`.

## Case map (test → native case)

- `test_inspect_tool_records_version_target_and_resource` →
  `inspect_tool_records_version_target_and_resource`
- `test_inspect_tool_rejects_unrelated_tool` →
  `inspect_tool_rejects_unrelated_tool`
- `test_inspect_tool_rejects_missing_and_silent` →
  `inspect_tool_rejects_missing_and_silent`
- `test_inspect_tool_rejects_wrong_family_words` →
  `inspect_tool_rejects_wrong_family_words`
- `test_warning_control_fires_real_diagnostic` →
  `warning_control_fires_real_diagnostic`
- `test_header_self_sufficiency_control_fails_with_diagnostic` →
  `header_self_sufficiency_control_fails_with_diagnostic`
- `test_invalid_source_control_is_rejected_with_diagnostics` →
  `invalid_source_control_is_rejected_with_diagnostics`
- `test_libstdcxx_assertions_abort_out_of_bounds` →
  `libstdcxx_assertions_abort_out_of_bounds`
- `test_libcpp_extensive_assertion_traps_out_of_bounds` →
  `libcpp_extensive_assertion_traps_out_of_bounds`
- `test_{appleclang,clang,gnu}_driver_query_selects_*` →
  `asan_lane_records_platform_runtime` (3 records, one case: the
  platform-correct absolute-runtime + supported/compiler/target
  contract covers all three driver families)
- `test_darwin_rtsan_driver_query_selects_the_osx_dylib` +
  `test_gnu_rtsan_lane_never_resolves_a_runtime` →
  `rtsan_lane_records_supported_runtime_or_reports_unsupported` (2
  records, one case: supported⇒recorded-runtime / unsupported⇒fail-
  closed covers both outcomes)
- `test_conflicting_asan_rtsan_fails` +
  `test_cli_cache_with_both_sanitizers_fails` →
  `conflicting_asan_rtsan_configure_fails` (2 records, one case)
- `test_cli_enabled_asan_injects_the_real_runtime` +
  `test_instrumented_clean_control_runs` +
  `test_selected_asan_extension_loads_in_a_fresh_process` →
  `sanitizer_clean_control` (3 records)
- `test_cli_enabled_rtsan_injects_the_real_runtime` +
  `test_selected_rtsan_extension_loads_in_a_fresh_process` →
  `rtsan_clean_control` (2 records)
- `test_instrumented_use_after_free_reports_asan` →
  `sanitizer_negative_control` (1 record)

## Dropped surface (166 records, `python-only` + named successor)

- **test_offline_bundle/install/tools (77)**: synthetic-wheel/venv/uv/
  tool-lock installer machinery for a Python-wheel offline bundle has
  no analog — the Python-free distribution is the CPack TGZ verified by
  `package_smoke` (clean-env `--help` on every packaged binary,
  headless ride, sha256-verified source deps) and `install_smoke`.
- **test_qualification_report (43)**: pytest/JUnit report classification
  + baseline failure-key matching is a removed-Python report generator;
  the qualification verdict now lives in `tools/ci_presets.sh` lanes +
  `.github/workflows/portability.yml` + the command/exit/commit rows of
  `docs/releases/python-free-qualification.md`. No pytest document
  remains to classify.
- **test_native_toolchain drops (20)**: `resolve_tool`,
  `sibling_candidates`, `darwin_tool_candidates`, `_gcc_context_args`/
  `_tool_argv`, `_stdlib_probe_spec` internals live inside
  `bike-native-checks` (`check_{context,frontends,reports}.cpp`); the
  resolved tools are recorded in the context and the argv/stdlib
  adaptations are exercised by the produced `check_frontends`,
  `check_odr`, `check_analyzer`, diagnostic-controls sweeps and
  `migration_T56`.
- **test_native_runtime drops (26)**: preload-env composition,
  driver-query/stub-image validation, cache-staleness gates and the
  `run --build` wrapper — `BikeCompilerPolicy.cmake` resolves/records
  the lanes in `capabilities.{asan,rtsan}` and fails closed at
  configure; `Sanitizers.cmake` links the runtime directly.

No golden fixtures: contract/structural surface only, no numerical
output to capture per platform.

## Stash disposition

Preserved stash `stash@{0}` ("stale check_* crumbs from dead agent")
was audited and landed as provenance commit `fe5f65ac`: it adds
`compiler_launcher(Value&)` (launcher-preserving argv construction that
rejects malformed launchers), `inspect_tool`, `inspect_sweep_tool`,
frontend warning-flag probing, and the `--inspect-tool PATH` CLI — the
toolchain contract surface this batch's ports depend on. Regression
evidence on the landed code: `migration_T53_portability` 306.97 s green,
`migration_T56` 1463.87 s green, `native/build` + `build/native-release`
rebuilt clean. The stash entry itself remains (unmodified, content now
committed).

## Verification

| gate | result |
|---|---|
| `cmake --build native/build` (darwin-arm64) | clean, 59 targets |
| `cmake --build build/native-release` (darwin-arm64) | clean |
| `ctest --test-dir build/native-release -R '^migration_T336_portability$'` | **Passed, 4.02 s** (#16), `passed (12 cases)`, 0 skips |
| `ctest ... -R '^(install_smoke|sanitizer_clean_control|rtsan_clean_control|package_smoke)$'` | all Passed (47.25 s / 0.03 s / 0.02 s / 83.63 s) |
| `cmake --preset asan` + `ctest --test-dir build/asan -R ...` | `sanitizer_negative_control` **Passed, 0.25 s**; clean controls also green in the asan tree |
| `ctest --test-dir build/native-release -R '^migration_T53_portability$'` | Passed, 306.97 s |
| `ctest --test-dir build/native-release -R '^migration_T56$'` | Passed, 1463.87 s |
| oracle `uv run pytest tests/portability/test_native_toolchain.py test_native_runtime.py` | 66 passed, 2 skipped (the two `NATIVE_TEST_BUILD_PATH`-gated instrumented tests — natively replaced by the unconditional controls) |
| linux-aarch64 (`bike-dev-pfc-lane-a`): `container.sh up`/`sync`/`configure native-release root`/`build` | all clean; deps sha256-verified, `linux-aarch64` record |
| linux-aarch64 `ctest -R '^(migration_T336_portability|sanitizer_clean_control|rtsan_clean_control)$'` | all Passed — T336 **0.41 s, `passed (12 cases)`**, 1 by-design skip (`libcpp_extensive_assertion_traps_out_of_bounds` is a Darwin libc++ contract); controls green |
| `bash tools/run_tests.sh native` (darwin-arm64), run 1 | **FAILED at `check_tidy`** — see baseline finding below |
| `cmake --build native/build --target check_tidy` after suppression | **green** — `findings: none`, `suppressions: clang-analyzer-cplusplus.Move=1, concurrency-mt-unsafe=1` |
| `bash tools/run_tests.sh native` (darwin-arm64), run 2 | **exit 8 at the CTest phase** — build + all sweeps green, CTest **60/62**; the 2 reds are `migration_T09` + `migration_T10`, baseline-verbatim frozen-oracle failures (see below) |
| post-abort phase: extension resolution (`load_native`) | green — `native/build/bike_native.cpython-313-darwin.so` |
| post-abort phase: `bike-native-checks --kind contracts --build native/build` | **green — 12/12** (provider_binding_boundary, manifest_selection, loader_provenance, iso_language_mode_and_hardening, analysis_policy, selected_artifact_assertions, contract_source_manifest_membership, ctest_contract_cases, engine_call_wrappers, benchmark_manifest_membership, pure_core_python_free, hot_kernel_dependency_free) |
| post-abort phase: `pytest tests/reference/test_native_*.py tests/reference/test_golden_episode*.py` (`run_with_native_runtime`, 3,086 items) | **in flight at report commit** — progressed past `test_native_check_tools.py` with no failures emitted; currently inside `test_standalone_header_sweep_compiles_first_party_headers` (the designed ~85–90 min serial full-header sweep, `timeout=7200`), which is running long on a load-avg-13 host |

Notes on the Linux run: sanitizer_negative_control is registered only
under `NATIVE_SANITIZE=ON` (verified in the Darwin asan tree instead);
the container's release configure does not register it — expected.
RTSan is unsupported on this Darwin host's AppleClang toolchain (probe
recorded `supported:false` in the context); `rtsan_clean_control`
asserts the fail-closed contract and `rtsan_negative_control` runs on
the upstream-clang lanes per `Sanitizers.cmake`.

## Baseline-red finding (not this batch's), suppressed with provenance

`run_tests.sh native` run 1 stopped at `check_tidy` with one unsuppressed
diagnostic: `native/src/batch/summary.cpp:51:28: warning: function is not
thread safe [concurrency-mt-unsafe]` on the read-only
`std::getenv("OPENBLAS_NUM_THREADS")` telemetry probe. The line was
introduced at `29509355` (3.34 lane/d, 2026-10-28), after the manifest's
last green baseline; **an independent worktree (`pfc-gate`) at
`d20ea74c` — this batch's exact base, carrying none of this batch's
changes — produced the identical blocking diagnostic** on this host,
proving the failure pre-existing rather than induced by this batch.
`getenv` is MT-unsafe only against concurrent mutation, and no
`setenv`/`putenv` exists anywhere in `native/src`; recorded in
`native/analysis_suppressions.json` (commit `a24d26f6`) with checker /
origin / message / `tool_version=23.1.2` / reason / reproducer /
`remove_when` per the manifest schema and the native-engineering
suppression rule. `check_tidy` standalone re-run: green
(`findings: none`; suppressions `clang-analyzer-cplusplus.Move=1,
concurrency-mt-unsafe=1`).

## Baseline-red CTest pair (not this batch's), frozen-oracle documented

`run_tests.sh native` run 2 passed the build and every static-analysis
phase — `check_frontends` (334 TUs), `check_tidy` (334 stable, 0
blocking after the `a24d26f6` suppression), `check_analyzer`
(report-only findings), `check_cppcheck`, `check_odr`, `check_scripts`
(shellcheck clean), produced `headers` (268 headers / 9,508 header
TUs), `diagnostic-controls` (6/6), `context` — then ran the 62-test
`native/build` CTest selection to **60/62** and exited 8 per
`set -euo pipefail`:

- `migration_T09` — `FAIL post-advance integration vector matches the
  oracle` (reproduced standalone: 18.68 s, identical signature)
- `migration_T10` — `FAIL first transition matches the Python oracle`

Both are **documented darwin-arm64 baseline reds, signature-verbatim**:
`migration/cpp-linux-completion-evidence.md` records exactly
`T09` (`post-advance integration vector matches the oracle`) and
`T10` (`first transition matches the Python oracle`) as "frozen-oracle
comparisons" in the documented darwin baseline-red set (the
`ctest -R 'migration_T(09|10|56)$'` → 0/3 row, and a prior `full`-gate
"exit 8, baseline-verbatim {T09,T10} ctest reds" entry).
**Independently A/B-verified on this host**: the `pfc-gate` worktree at
`d20ea74c` — this batch's exact base, carrying none of this batch's
changes — fails the same two tests with byte-identical signatures
(`PASS 319 checks` → `FAIL post-advance integration vector matches the
oracle`; `FAIL first transition matches the Python oracle`). The oracles are
vectors committed inside `test_t09.cpp`/`test_t10.cpp` (last touched
2026-10-15/16, `cbe9b8d5`/`c9366654`) — deterministic comparisons; this
batch's diff (`d20ea74c..HEAD`) touches only
`native/src/tools/check_{context,frontends,reports}.cpp`,
`checks_internal.hpp`, `checks_main.cpp`, `Checks.cmake`, the new T336
script, `analysis_suppressions.json`, and the ledgers — none of which
link into or alter T09/T10's inputs, so the pair cannot be attributed
to this batch. Per the AGENTS.md baseline protocol and the project's
documented practice (F4 lane: "Unrelated documented reds reported, not
fixed"), they are reported, not fixed.

Because `set -e` aborts the gate at the CTest failure, the three
remaining gate phases were run standalone under the identical
`NATIVE_TEST_BUILD_PATH=native/build` / `native_runtime.py run`
environment: extension resolution green, `--kind contracts` **12/12
green**, and the `test_native_*`/`test_golden_episode*` pytest phase —
see the table row above.

## Port-authored bugs found and fixed during red→green

- `find_program` results are cached: a prior compiler path leaked into
  later negative cases — each probe now unsets a sanitized cache
  variable per candidate.
- Absolute compiler candidates (e.g. `/opt/homebrew/opt/llvm/bin/
  clang++`) failed `_find_first`'s name lookup — absolute paths are now
  accepted directly when they exist.
- The first stdlib OOB probe was libstdc++-specific and did not
  faithfully port the libc++ control — replaced with distinct
  `_GLIBCXX_ASSERTIONS` (hardened-trap + recorded-UB) and
  `_LIBCPP_HARDENING_MODE_EXTENSIVE` (benign-run + nonzero-violation)
  probes matching the Python sources.
- `CMAKE_HOST_SYSTEM_PROCESSOR` is unset in `cmake -P`; host arch comes
  from `uname(1)` normalized to `aarch64`/`x86_64`.
- Accumulators for `_fail`/`_skip` live on GLOBAL properties — helper-
  function scope would silently drop failures via PARENT_SCOPE.
- Message needles compare squashed (whitespace-collapsed) since CMake
  wraps `FATAL_ERROR` text at ~80 columns (same rule as migration_T53).

## Ledger

- `migration/test-port-map.json`: all 188 records updated —
  `disposition` 22 `port` / 166 `drop`; `native_test_id` =
  `migration_T336_portability::<case>` or the control test name;
  `native_source` = the carrier script/.cpp; `collected_node_ids` =
  measured `pytest --collect-only` ids (every drop carries real ids);
  `assertions_capture` = measured node counts; `comparison_class` =
  `contract` (22) / `python-only` (166); evidence = dated
  `2026-10-31 3.36t2a lane/a` entries naming the native successor.
  Canonical serialization (`indent=2, ensure_ascii=True` + trailing LF)
  byte-verified.
- `migration/python-removal.json`: the six rows →
  `status: replacement_landed`. `replacement_paths` now name the actual
  carriers — `native/tests/migration/migration_T336_portability.cmake`
  (+ `sanitizer_control.cpp`/`rtsan_control.cpp` for runtime), the
  packaging/package_smoke/install_smoke artifacts for the offline
  files, and `portability.yml`/`ci_presets.sh`/`python-free-
  qualification.md` for the report file — replacing the inventory's
  never-created placeholder `native/tests/reference/test_*.cpp` paths.
  `verification_ids` = the CTest names verified green above
  (`migration_T336_portability`, `sanitizer_clean_control`,
  `rtsan_negative_control`→`sanitizer_negative_control`,
  `install_smoke`, `package_smoke`); the all-drop qualification file
  carries the ledger anchors `source_assertion_map` +
  `release_capability_matrix` per the all-drop landed convention.
  Inventory-note scratch fields retained per the tranche-1 portability
  sibling convention. Canonical bytes verified. Only the six target
  rows were touched — tool-file rows owned by T56–T63 tasks untouched.

## Unresolved concerns

- `rtsan_negative_control` has no executed evidence on this host
  (AppleClang lacks RTSan; the context records `supported:false`). No
  ledger record routes to it; it runs on the upstream-clang lanes.
- The six Python files remain in place — deletion is the T63 gate.
- Qualification lanes in `docs/releases/python-free-qualification.md`
  remain "Not run" — this batch lands the report-generator's native
  successor (checklist + workflow), not release qualification itself.
