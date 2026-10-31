# Tooling replacement map — 2026-10-31 (audit of plan tasks 3.41–3.45)

Read-only audit at `d20ea74c`. Verdict: all five tasks are **source-complete but
unexecuted** — no preset configure, sanitizer lane, coverage run, CPack build,
offline rehearsal, or CI run has ever been recorded;
`docs/releases/python-free-qualification.md` lists every lane "Not run".
Nothing flips to a terminal ledger status until the lanes execute green on
macOS arm64 + Linux aarch64.

## Replacement map (`tools/` ledger rows)

Dispositions: `replacement_landed`, `replace_with_cmake_ctest_cpack`,
`replace_with_ci_checklist`, `delete_with_python`, `deferred`.

### Dev-driver layer

| Path | Purpose | Verified native replacement | Disposition |
|---|---|---|---|
| `tools/dev.py` | dev driver: bootstrap/configure/build/doctor/test | CMakePresets.json presets (`release|debug|gui|asan|rtsan|coverage`); doctor → configure probes (`native_check_context.json`, `bike_policy_report.json`) | `replace_with_cmake_ctest_cpack` — partially; never run |
| `tools/native_runtime.py` | sanitizer dylib preload + exec | `native/cmake/Sanitizers.cmake`, `BikeCompilerPolicy.cmake` runtime resolution; `asan`/`rtsan` presets | `replace_with_cmake_ctest_cpack` — still called by `run_tests.sh:110` (dies with it) |
| `tools/native_coverage.py` | llvm-profdata/llvm-cov post-run | `coverage` preset+target, `Coverage.cmake`/`RunCoverage.cmake`/`CoverageTools.cmake`, `coverage-report` preset | `replace_with_cmake_ctest_cpack` — partially; resolver fixtures open |
| `tools/native_coverage.sh` | shell front-end (not a ledger row) | same | dies with Python |
| `tools/ci_portability.py` | qualified CI lane runner | `.github/workflows/portability.yml` + `tools/ci_presets.sh` + `tools/linux/container.sh` | `replace_with_ci_checklist` — ledger cites nonexistent `native.yml` |
| `tools/portability_qualification.py` | result classification + release-gate aggregation | `docs/releases/python-free-qualification.md` checklist + `ci_presets.sh` evidence trap | `replace_with_ci_checklist` — manual by design (D18) |
| `tools/portability_baseline.py` | baseline row marking | `python-free-qualification.md` + `toolchain-matrix.json` | `replace_with_ci_checklist` — same stale path |
| `tools/offline_bundle.py` | schema-1 offline bundles | CPack TGZ (`packaging/CMakeLists.txt`), `StageSource.cmake`, `package_smoke.cmake`, `extract_deps.sh` | `replace_with_cmake_ctest_cpack` — stale paths |
| `tools/offline_manifest.py` | offline-manifest schema | package digests `packaging/CMakeLists.txt:59-60` + `package_smoke.cmake:90-108` | `replace_with_cmake_ctest_cpack` |
| `tools/install_offline.py` | offline install | untar + `extract_deps.sh` + `cmake --workflow --preset release`; `ci_presets.sh offline` | `replace_with_cmake_ctest_cpack` — rehearsal never run |
| `tools/prepare_offline_tools.py` | freeze tool wheels | obsolete: distributions ship no Python tools | `replace_with_cmake_ctest_cpack` |
| `tools/check_offline_sources.py` | tree vs wheel diff | `bike-python-audit --mode package` in `package_smoke.cmake:85` | `replace_with_cmake_ctest_cpack` |
| `tools/native_dependencies.py` | provider probe, numerics record | `BikeDependencyLock.cmake`, `dependencies.lock.json`, `BikeNumerics.cmake`, `numerics.lock.json`, `acquire/extract_deps.sh`, `migration_T53_numerics` | `delete_with_python` — still called by binding graph |
| `tools/numeric_portability_probe.py` | bitwise probe vs NumPy | `native_platform_{blas,lapack,sincos}` CTests + goldens + `migration_T53_numerics` + `reference.native_numeric_*` | `delete_with_python` |
| `tools/native_toolchain.py` | analysis-tool resolution | `check_context.cpp` + `CoverageTools.cmake` | `delete_with_python` |

### Check orchestrator (T56)

| Path | Purpose | Replacement | Disposition |
|---|---|---|---|
| `tools/native_checks.py` | 9-sweep orchestrator | `bike-native-checks --kind {tidy,analyzer,cppcheck,frontends,odr,headers,diagnostic-controls,context,scripts,contracts}` + `run_native_checks.sh` + `check_<kind>` targets + `migration_T56_check` | `replacement_landed` — needs execution pass |
| `tools/check_native_contracts.py` | structural contract checks | `bike-native-checks --kind contracts`; still invoked at `native/CMakeLists.txt:556` | `replacement_landed` |
| `tools/native_config.py` | 15-line shim | `bike_configuration` lib | `replacement_landed` |
| `tools/native_schema.py` | 12-line shim | same | `replacement_landed` |

### Perf/diagnostic drivers

| Path | Purpose | Replacement | Disposition |
|---|---|---|---|
| `tools/research_sweep.py` | parallel seed sweep | `bike-research-sweep` + `bike-batch` + `migration_T51` | `replacement_landed` |
| `tools/measure_realtime.py` | realtime measurement | `bike-measure-realtime` + `reference.realtime_*` | `replacement_landed` |
| `tools/golden_episode.py` | episode oracle capture | `bike-recording golden freeze` + `golden/{episode,capture}.cpp` + fixtures + `reference.golden_episode_*` | `replacement_landed` — ledger paths stale |
| `tools/episode_compare.py` | golden replay+diff | `bike-recording golden compare` / `compare` | `replacement_landed` — stale paths |
| `tools/diag_run.py` | ad-hoc headless ride + npz dump | `bike-ride --headless --no-plots` + `bike-recording overview/fields/export` | `replacement_landed` — npz format intentionally unported |
| `tools/step_profile.py` | ride-loop timing + cProfile | `bike-measure-realtime` p50/p95/p99 + `native_bench` + `reference.native_benchmark` | `replacement_landed` — partial (cProfile view unported) |
| `tools/divergence_horizon.py` | chaos-divergence horizon | **NO REPLACEMENT** (ledger cites nonexistent `divergence_horizon_main.cpp`) | `delete_with_python` — decide explicitly |
| `tools/recording_finalize_bench.py` | encode→Arrow→DuckDB stage timing | **NO REPLACEMENT** | `delete_with_python` — decide explicitly |
| `tools/proto_native_bench/bench.py` | matched-workload benchmark twin | `native_bench` + `native_bench_{bare,glue,heavy}` + `reference.native_benchmark` | `replacement_landed` — twin retires with oracle (D12) |
| `tools/proto_native_bench/state_format.py` | BIKEST02 bundle r/w | `state_format.{hpp,cpp}` (reader only) | `replacement_landed` (reader); writer gap → freeze artifacts |
| `tools/proto_native_bench/dump_model.py` | regenerates BIKEST02 bundles | **partial gap** — no native writer | `delete_with_python` — freeze committed `artifacts/` |

### `tools/migration/`

| Path | Purpose | Replacement | Disposition |
|---|---|---|---|
| `capture_reference.py` | oracle golden capture driver | `bike-recording golden freeze` + `capture.cpp` + fixtures | `delete_with_python` |
| `_t10_capture.py`, `_f5_spindle_capture.py`, `_t25_capture.py`, `_t52_capture.py` | one-off captures | same | `delete_with_python` — **3 of these lack ledger rows** |

### `tools/research/**` (22 rows)

All cite `docs/reference/README-provenance.md` (does not exist — a 4.05
deliverable). Per user ruling OD-1 (2026-10-31): **all deferred** — keep as
explicit exception candidates in the no-python policy.

## Gaps / defects to fix

1. `divergence_horizon.py`, `recording_finalize_bench.py` — no native
   replacement; explicit `delete_with_python` justification or a small port task.
2. `dump_model.py`/`state_format.py` writer side — freeze committed artifacts
   or port the writer.
3. Missing ledger rows: `tools/migration/{_f5_spindle,_t25,_t52}_capture.py`.
4. Stale workflow path: rows cite `.github/workflows/native.yml`; actual is
   `portability.yml`.
5. Stale T55/T57/T59 replacement paths cite plan-08 artifacts that D18 cut
   (`golden_episode_main.cpp`, `episode_compare_main.cpp`,
   `divergence_horizon_main.cpp`, `step_profile_main.cpp`, `diag_main.cpp`,
   `finalize_bench_main.cpp`, `benchmark_main.cpp`, `test_runner_main.cpp`,
   `coverage_main.cpp`, `package_verify_main.cpp`, `install_main.cpp`,
   `packaging/manifest.schema.json`, `packaging/install.sh`). Real paths:
   `recording_golden.cpp`, `golden/*.cpp`, `RealtimeMeasurement.cmake`,
   `native_bench`, `CMakePresets.json`, `packaging/CMakeLists.txt`,
   `package_smoke.cmake`, `extract_deps.sh`, `bike-python-audit --mode package`.
6. `docs/reference/README-provenance.md` must be authored in task 4.05.
7. Transition dependents to remove with the drop commits: `run_tests.sh:110`,
   `native_coverage.sh`, `make_offline_bundle.sh`, `install_offline.sh`,
   `native/CMakeLists.txt:556`, `native/cmake/BikeDependencies.cmake:27`.

## Per-task status

| Task | Verdict |
|---|---|
| 3.41 CMakePresets | source-complete, unverified — never configured |
| 3.42 sanitizer/RTSan | source-complete, unverified — never executed |
| 3.43 coverage | source-complete, unverified — never run; resolver fixtures open |
| 3.44 CPack distributions | source-complete, unverified — no package/offline run |
| 3.45 CI on presets | source-complete, unverified — no workflow validation/run |

Next actions: fix stale ledger paths, add missing `tools/migration` rows,
decide the three gap tools, run the qualification lanes green on both
platforms, then flip rows.
