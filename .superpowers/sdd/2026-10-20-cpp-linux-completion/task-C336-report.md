# Task C336 — test-port 3.36 report (portability tooling; `portability`, T53)

**Batch**: 3.36 — the 24 `tests/portability/test_*.py` files (558
test-port-map records, 542 `planned` at batch start). This lane session
landed the **five plan-named files** — `test_cmake_platform.py`,
`test_compiler_policy.py`, `test_dependency_policy.py`,
`test_provider_identity.py`, `test_root_configure.py` — 49 records total.
Per the shared-procedure split rule ("split it by file at the boundary
reached"), the remaining 19 files + 1 red record (494 `planned`) remain are left to a
follow-on session; the boundary is exactly by file.

**Lane**: `.worktrees/pfc-lane-c`, branch `lane/c`, base `842c2585`
(post-3.29b/3.33a integration tip). No merge, no push.

## Status: 43 `port`, 5 `drop`, 1 `planned` (live-oracle red), 4 files `replacement_landed`

**Live-oracle red kept `planned`**:
`test_dependency_policy.py::test_preserved_scientific_pins_and_python_floor`
asserts `requires-python == ">=3.13"` verbatim while `pyproject.toml`
carries `">=3.13,<3.14"` — `uv run pytest` reproduces the failure at HEAD
(`'>=3.13,<3.14' != '>=3.13'`). Per the batch rule, the record stays
`planned` with the verbatim signature; the four declared-pin checks that
do hold are ported (`preserved_scientific_pins`). The file's removal row
stays `planned` accordingly; the other four files are
`replacement_landed`.

Ported surface: **one `cmake -P` CTest contract script** —
`native/tests/migration/migration_T53_portability.cmake` — registered as
CTest `migration_T53_portability` in `native/cmake/Checks.cmake`
(labels `migration;t53;portability`). Fixture style follows the plan's
3.36 note: scratch configures against the real `native/cmake/Bike*.cmake`
modules (migration_T03-style), plus `run_case`-style execute_process
harnesses (migration_T56 precedent) for the provider-identity cmake -P
surfaces and the produced pin/record contracts.

| Source file | records | port | drop | landing |
|---|---|---|---|---|
| `test_cmake_platform.py` | 12 | 12 | 0 | `migration_T53_portability::*` (14 cases: 3 Linux-only, 7 Darwin-only, 4 both) |
| `test_compiler_policy.py` | 10 | 10 | 0 | same (10 cases incl. per-compiler loop + glibc-only fortify) |
| `test_dependency_policy.py` | 7 | 2 | 4 (+1 planned-red) | declared-pin file checks ported; `artifact.*` enforcement dropped (interpreter-scoped, T63); python-floor record red at HEAD |
| `test_provider_identity.py` | 16 | 15 | 1 | pure cmake -P + Darwin probe cases ported; Python build-id spelling dropped |
| `test_root_configure.py` | 4 | 4 | 0 | shim control, no-interpreter configure, cross-toolchain refusal, bindings fail-closed |
| **total** | **49** | **43** | **5** (+1 planned-red) | |

No golden fixtures needed: the batch is contract/structural only (no
live-oracle value comparisons).

## Case map (source_symbol → ctest case)

- `test_cmake_platform`: Linux gate (`linux_configure_never_invokes_apple_tools`
  with planted `xcrun`/`xcode-select` stubs, `linux_rejects_osx_sysroot_option`,
  `linux_rejects_foreign_system_name`); Darwin SDK contract
  (`macos_default_sdk_resolves_via_xcrun`, `macos_explicit_sdk_name_is_resolved`,
  `macos_explicit_sdk_alias_is_canonicalized`, `macos_missing_explicit_sdk_path_fails`,
  `macos_unavailable_sdk_name_fails`,
  `macos_default_falls_back_from_link_incompatible_sdk` — link-rejecting
  c++ wrapper + fake xcrun verbatim — and
  `macos_explicit_link_incompatible_sdk_is_fatal`); cross-compilation
  refusal through both includers
  (`cross_compiling_configure_is_refused_{BikePlatform,BikeHostTarget}`)
  and the positive `host_dispatch_names_the_configured_host`.
- `test_compiler_policy`: real configure+build probe applying
  `bike_apply_native_policy` then reading `compile_commands.json` +
  `bike_policy_report.json`: linux-x86 flag hygiene, strict
  `-std=c++23`/`-ffp-contract=off`/`-Werror`/banned-flag sweep over every
  entry, unsupported-warning omission record, missing-required-flag
  fail-closed, host dispatch (aarch64 branch-protection / Darwin
  headerpad), simulated-aarch64, stdlib hardening dispatch
  (`_LIBCPP_HARDENING_MODE_EXTENSIVE` vs `_GLIBCXX_ASSERTIONS` and never
  `_GLIBCXX_DEBUG`), C++23 library probes, glibc `_FORTIFY_SOURCE=3`
  Release-only, and `policy_under_each_available_compiler` iterating the
  deduped-by-realpath host compiler set.
- `test_provider_identity`: committed-lock eight-op record check
  (`committed_provider_records_qualify_bound_operations`), the LC_UUID
  probe-text positives/negatives (`probe_text_yields_the_reported_uuid`,
  `probe_text_fails_for_unreported_image`, `probe_text_fails_for_no_lc_uuid`,
  `probe_text_rejects_malformed_uuid` over the three verbatim vectors),
  declarations generation (`declarations_reproduce_accelerate_identity`,
  `declarations_missing_build_id_as_null` with the `<none>`→null path,
  `declarations_reject_build_id_count_mismatch`,
  `declarations_reject_record_lists_other_providers`), the Darwin probe
  (`probe_reports_the_loaded_accelerate_images` with 8-4-4-4-12 group
  length checks — CMake regex lacks `{n}`,
  `probe_fails_when_required_image_not_loaded`), build-id vector
  coverage + `cmake_build_id_matches_the_shared_vectors` over all 11
  fixture vectors, `build_id_spelling_has_one_home_per_language` repo
  scan, and `root_identity_equals_the_legacy_graph` (root vs legacy
  configure identity equality; Darwin + `.deps` + importable scientific
  env, same gate as the Python skip).
- `test_root_configure`: `interpreter_shim_records_a_spawn_by_name`
  (positive shim control), `root_configure_starts_no_interpreter`
  (stub-PATH configure + `^_?P(ython|YTHON)\d*_` cache scan),
  `root_refuses_a_cross_compiling_toolchain` (verbatim refusal +
  `BikeHostTarget.cmake` attribution + no "Configuring done"),
  `root_fails_closed_on_python_bindings` (verbatim fail-closed needle).
- `test_dependency_policy`: declared pins read from `pyproject.toml`
  (`mujoco==3.12.0`, `numpy==2.5.2`, `scipy/duckdb/pyarrow` exact pins,
  `numba>=0.68`, `requires-python = ">=3.13"`, `threadpoolctl>=3.6`
  floor / no `==` pin). The four `bike_sim.native.artifact`
  enforcement records are `drop`/`python-only`: that module is
  interpreter-side enforcement and dies at T63; no C++ subject exists
  (evidence strings name this per record).
- `drop` also covers
  `test_python_build_id_matches_the_shared_vectors` — the Python spelling
  of the build-id formula (`provider_build_id.py`) is deleted with the
  interpreter; the surviving CMake spelling is asserted over the same
  shared vectors.

## Verification

| gate | result |
|---|---|
| darwin-arm64 `cmake -P` (standalone) | passed, 44 cases, 1 linux-gated skip |
| `ctest --test-dir build/native-release -R ^migration_T53_portability$` | **Passed, 340.80 s** (test #14) |
| linux-aarch64 `cmake -P` (bike-dev-pfc-lane-c) | passed, 38 cases, 1 darwin-gated skip |
| `ctest --test-dir /build/root-clang -R ^migration_T53_portability$` (container) | **Passed, 102.56 s** (test #15) |

Negative proof: a tampered copy (`mujoco==9.12.0`) fails with
`native_scientific_versions_are_pinned: mujoco==9.12.0 not pinned` in
the failure list — the accumulator records helper-routed failures.

### Port-authored bugs found and fixed during red→green

- `_fail`/`_skip` originally wrote `PARENT_SCOPE`, which only reached
  the `_expect_*` helper's own frame — helper-routed failures were
  silently dropped (proved by a tamper run passing). Accumulators are
  now GLOBAL properties; the rerun tamper fails as required.
- `CMAKE_HOST_SYSTEM_PROCESSOR` is unset in `cmake -P` mode → host
  arch now comes from `uname -m`.
- `CMAKE_MATCH_{n}` quantifiers unsupported → UUID well-formedness uses
  capture groups + explicit 8-4-4-4-12 lengths.
- `find_program` cache + stale normal-var reuse in the compiler
  enumeration (produced a `TRUE` compiler) → dedicated var + explicit
  unsets.
- `CMP0057 NEW` required for `IN_LIST` on the container's CMake 3.28.
- Legacy-graph configure needs `VIRTUAL_ENV` (FindPython3 prefers the
  venv only when set — the Python test inherited it from `uv run`) and
  `/usr/sbin` on PATH (`sysctl` used by the interpreter probe).
- `string(JSON GET)` renders booleans `ON`/`OFF`, not `true`/`false`.
- Long FATAL-message needles must match whitespace-collapsed
  (`_expect_squashed`, the port of the Python tests' `squashed()`).
- The explicit-incompatible-SDK case needs a resolved sysroot path:
  the module canonicalizes `-isysroot` before the wrapper compares it.

## Ledger

- `migration/test-port-map.json`: 49 records updated — `native_test_id`
  = `migration_T53_portability::<case>` (comma-joined multi-case like
  T56), `native_source` = the script path, `collected_node_ids` =
  measured `pytest --collect-only` ids on this host (64 nodes across the
  five files; parametrize tables verbatim), `assertions_capture` =
  measured node/assert counts, `comparison_class` = contract (43) /
  structural (1) / python-only (5), evidence = dated lane entry with
  both-platform ctest facts.
- `migration/python-removal.json`: four file rows →
  `status: replacement_landed`, `replacement_paths` = the cmake script,
  `verification_ids` = `migration_T53_portability`, live sha256/line_count
  verified unchanged; `test_dependency_policy.py` stays `planned` for
  the live-oracle red above.

## Remaining scope (explicit split)

19 files + 1 red record (494 `planned`) remain remain in batch 3.36 and need per-function
triage in a follow-on session — per D18, many of the tool-surface tests
(`test_native_runtime.py` 39, `test_offline_bundle.py` 35,
`test_ci_lanes.py` 34, `test_release_gate.py` 34, `test_native_toolchain.py`
29, `test_install_modes.py` 28, `test_native_dependencies.py` 28,
`test_offline_install.py` 28, `test_dev_commands.py` 18,
`test_offline_tools.py` 14, `test_native_coverage_tools.py` 12,
`test_viewer_launch_platform.py` 11, `test_worker_platform.py` 11,
`test_development_docs.py` 9, `test_baseline.py` 4,
`test_headless_imports.py` 3) decide per function between a named
preset/CTest/CPack successor and a justified drop; the plan calls out
`test_numerics_discovery.py` (113) explicitly — its record-only
`BikeNumerics.cmake`, provider identity/path, ABI-width and symbol guards
are retained native contracts requiring CTest successors (the D18
tooling review's per-function table is the working list).
`test_qualification_report.py` (43) is CI-checklist successor territory
(T58/3.45). `test_performance_evidence.py` (16) already carried `port`
dispositions at batch start.
