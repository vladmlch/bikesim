# Portability requirement-to-test matrix

Date: 2026-10-18. Task T27 of `docs/superpowers/plans/2026-10-14-linux-macos-07-validation.md`.

This matrix maps every release-blocking requirement of the portability
design (`docs/superpowers/specs/2026-10-14-linux-macos-portability-design.md`)
to the exact test files, node IDs, or commands that enforce it, and to the
hosts where the evidence must be produced.

**Required hosts** (the supported-platform contract, spec section 3):

- **macOS** — the existing Apple Silicon macOS environment (the recovered
  baseline: AppleClang, Accelerate, ARM64).
- **Linux** — Ubuntu 24.04 x86_64, glibc. Clang and GCC lanes are both
  release-blocking for the strict C++23 build and the native
  contract/oracle tests.
- **both** — a row marked *both* must produce green evidence on each host;
  a platform is never qualified by another platform's green run.

**Profile column** — `quick`/`native`/`full` name the `tools/run_tests.sh`
profile that executes the row; `lane:` names a separate build/test lane.

| Requirement | Exact test / command | Profile / lane | Required hosts |
|---|---|---|---|
| Environment identity recorded and verified | `tests/portability/test_baseline.py` (e.g. `test_real_checkout_baseline`); `tools/portability_baseline.py --output PATH` | quick | both |
| Platform pre-project dispatch | `tests/portability/test_cmake_platform.py` (`test_linux_configure_never_invokes_apple_tools`, `test_linux_rejects_osx_sysroot_option`, `test_linux_rejects_foreign_system_name`) | quick | both |
| Apple SDK discovery/fallback preserved | `tests/portability/test_cmake_platform.py` (`test_macos_default_sdk_resolves_via_xcrun`, `test_macos_explicit_sdk_name_is_resolved`, `test_macos_explicit_sdk_alias_is_canonicalized`) | quick | macOS |
| MuJoCo 3.12.0 image + private error ABI | `tests/reference/test_native_engine_errors.py` (e.g. `test_native_fatal_constructor_preserves_process`, `test_runtime_fatal_poisons_stepper_until_reset`); `tools/run_tests.sh` preflight version check | native, full | both |
| Engine linkage / image identity | `tests/reference/test_native_platform_linkage.py` (`test_every_ctest_executable_runs_directly`, `test_engine_consumers_record_resolvable_engine_image`, `test_extension_imports_without_environment_rescue`) | native, full | both |
| BLAS/LAPACK provider selection | `tests/portability/test_numerics_discovery.py`; `tools/native_dependencies.py --python PATH` probe | quick | both |
| BLAS/LAPACK bitwise-vs-local-NumPy lowering | `tests/reference/test_native_platform_blas.py` (`test_blas_dot_matches_numpy`, `test_blas_matvec_matches_numpy`, `test_blas_matmul_matches_numpy`) | native, full | both |
| Numerical portability (ufuncs/libm boundary) | `tests/reference/test_native_numeric_portability.py` (`test_tanh_and_hypot_match_local_array_ufunc`, `test_tanh_and_hypot_adversarial_fixture`); `tools/numeric_portability_probe.py --build PATH --output PATH` | native, full | both |
| Numeric input domains shared | `tests/reference/test_native_numeric_domains.py`, `tests/reference/test_native_numeric_completion.py` | native, full | both |
| Selected-artifact loader + provenance | `tests/reference/test_native_loader.py` (`test_selected_build_loads_the_exact_imported_extension`, `test_load_native_records_selected_build_and_imported_extension`, `test_pytest_reporter_records_selected_native_count_and_provenance`); provenance JSON consumed by `tools/portability_qualification.py` | native, full | both |
| Install-mode selection / no implicit build | `tests/portability/test_install_modes.py` | quick | both |
| Compiler flag policy + hardening | `tests/portability/test_compiler_policy.py`; `tools/native_checks.py --kind diagnostic-controls --build PATH` | quick; native preflight | both; Linux: Clang **and** GCC |
| Dependency pin policy | `tests/portability/test_dependency_policy.py` | quick | both |
| Source membership / target manifest | `tests/reference/test_native_loader.py::test_contract_executable_source_is_in_cmake_manifest`; `tests/reference/test_native_check_tools.py` (compilation-database and `native_sources.json` cases); `tools/native_checks.py --kind headers --build PATH`; `cmake --build <build> --target check_native_contracts` | native, full | both |
| Static-analysis sweeps (frontends, tidy, analyzer, cppcheck, ODR, scripts) | `cmake --build <build> --target check_frontends check_tidy check_analyzer check_cppcheck check_odr check_scripts` (driven by `tools/check_*.sh`); `tests/portability/test_native_toolchain.py` | native preflight | both |
| Sanitizer runtime selection + ASan lane | `tests/portability/test_native_runtime.py` (e.g. `test_linux_asan_uses_recorded_compiler_not_filename`, `test_darwin_asan_prepends_dyld_insert_libraries`); `tools/native_runtime.py run --build PATH -- COMMAND`; `native/tests/sanitizer_control.cpp` via `ctest --test-dir <build>` | quick (composition); lane: `-DNATIVE_SANITIZE=ON` build + `tools/run_tests.sh native` with `NATIVE_TEST_BUILD_PATH=<asan-build>` | both |
| RTSan lane | `tests/portability/test_native_runtime.py` (`test_rtsan_enabled_uses_the_rtsan_runtime`, `test_gnu_rtsan_lane_never_resolves_a_runtime`); `native/tests/` RTSan control binaries (`native_rtsan_control` CTest target) | quick (composition); lane: `-DNATIVE_RTSAN=ON` clang build | both (clang lane only — GCC provides no RTSan runtime) |
| Coverage | `tests/portability/test_native_coverage_tools.py` (`test_real_toolchain_resolves_and_passes_the_format_probe`, `test_linux_tool_search_never_invokes_xcrun`); `tools/native_coverage.sh` / `tools/native_coverage.py` | quick (controls); lane: coverage build + export | both (clang-family lanes) |
| Headless execution (no display) | `tests/portability/test_headless_imports.py` (`test_import_without_display`, `test_cli_help_without_display`); `tests/reference/test_native_linux_headless.py` (`test_four_step_driver_without_rendering`, `test_native_cli_without_display_retains_real_outcome`) | quick; native, full | both |
| GUI launch policy | `tests/portability/test_viewer_launch_platform.py` (`test_linux_never_looks_up_or_reexecutes_mjpython`, `test_darwin_reexec_preserves_argv_and_fallback`); `tests/reference/test_frontend_windows.py`; real viewer acceptance per `docs/development/viewer-platform-acceptance.md` | quick; native, full; manual desktop acceptance | both |
| Async / batch workers | `tests/test_async_close.py` (`test_worker_roundtrip_publishes_period`), `tests/test_async_recovery.py`, `tests/test_batch.py`; `tests/portability/test_worker_platform.py` (`test_linux_cpu_count_does_not_query_apple`, `test_darwin_performance_core_query`) | quick; slow episodes in full | both |
| Recording (opt-in / final-only) | `tests/reference/test_recording_*.py` (`test_recording_capture`, `test_recording_lifecycle`-family files); `tests/reference/test_native_platform_recording.py` (e.g. `test_viewer_ctrl_c_seals_operator_stop_and_publishes`); `tests/reference/test_native_recording_lifecycle.py`, `test_native_recording_retention.py` | quick (Python controls); native, full | both |
| Replay — checked physical vs portable inspection | `tests/test_recording_platform_compatibility.py` (`test_checked_replay_still_demands_an_explicit_episode`, `test_inspection_reads_a_recording_authored_on_another_host`); `tests/reference/test_native_platform_replay.py` (`test_native_checked_replay_passes_for_a_same_host_recording`, `test_native_replay_rejects_a_recording_authored_on_another_host`); `tests/test_replay_playback.py` | quick; native, full | both |
| Offline install (source bundle) | `tests/portability/test_offline_install.py` (T25 — bundle preflight + e2e installer paths; fixture bundles use the shipping `packaging/install_{full,slim}.sh`); `tests/portability/test_offline_bundle.py`, `tests/portability/test_offline_tools.py`; `tools/check_offline_sources.py` | quick | both |
| Result qualification / exact baseline comparison | `tests/portability/test_qualification_report.py`; `python tools/portability_qualification.py --profile PROFILE --build PATH --output PATH [--baseline PATH]` | quick (controls); qualification runs wrap every profile | both |

## Golden traces and fixture provenance

Golden traces (e.g. `tests/reference/test_golden_episode.py`,
`test_golden_episode_v2.py`, and the recorded-episode fixtures under
`tests/fixtures/`) belong to their **originating environment**. They are
authoritative oracles only for the platform/toolchain/provider identity
recorded with them — the checked-in manifests carry the execution identity
(see `tests/fixtures/platform-recordings/darwin-arm64.json`), and the
`identity_schema=2` execution identities compare platform and numerics
records field by field.

A fixture provenance mismatch — a recording or golden trace authored on
another host, OS, CPU, interpreter, or numerics provider — is **reported
with a field-level incompatibility explanation, never silently
regenerated** (enforced by `test_native_replay_rejects_a_recording_authored_on_another_host`,
`test_native_replay_rejects_a_foreign_runtime_with_the_named_field`, and
the checked-replay cases in `tests/test_recording_platform_compatibility.py`).
A macOS binary trace is not a Linux bitwise oracle: each host keeps its own
local exact-oracle tests (`tests/reference/_bits.py` pinning) plus the
portable semantic invariants that compare row counts and identities rather
than raw container bytes.

## Result-document rules (T27)

- Every required test executes against the **selected** artifact:
  `NATIVE_TEST_BUILD_PATH` selects one build, `native_test_provenance.json`
  records the imported extension, and `tools/portability_qualification.py`
  verifies freshness, build/extension identity, platform identity, and a
  nonzero executed direct-oracle set — tooling-only `test_native_*`
  modules never qualify.
- New failures, skips, xfails, or errors cannot hide behind the approved
  macOS `full` baseline: `classify_result` compares exact
  `(nodeid, signature)` keys and event node sets on the identical
  environment only. Linux required gates are pass-only.
- Empty collections, deselected-all runs, collection/import errors,
  timeouts, signal exits, and incomplete evidence are always `fail` —
  never a green or baseline_unchanged suite.
