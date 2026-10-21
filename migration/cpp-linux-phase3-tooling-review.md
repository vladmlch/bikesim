# Phase 3 tooling source review — 2026-10-21

Scope: completion-plan Tasks 3.41–3.45, plus source checks of the supplied
agent report about the preceding phases. The user prohibited compilation
and execution. No CMake configure, compiler/syntax-only invocation,
CTest/pytest, sanitizer, coverage tool, application, container, package
command or project shell script was run. The observations below are
source-supported; prior recorded executions are attributed to the supplied
archive, not claimed as new results.

## Source changes

| Task | Implementation | Qualification still required |
| --- | --- | --- |
| 3.41 | Root version-6 presets, host-specific toolchain selection, separate build trees, explicit `slow/audit/package` test exclusions and JUnit output; active development commands updated | Both-platform configure/build/CTest workflows and compiler-cache evidence |
| 3.42 | Root clean and conditional negative controls using the existing policy flags; root build rpaths and configure controls use the linked sanitizer runtime without an interpreter preload | Clean/defect control pairs on every supported instrumentation lane; unsanitized registration check |
| 3.43 | CMake LLVM tool resolution with fail-closed overrides, matching compiler major and real profile-format probe; fresh-profile coverage target, LCOV/text outputs and native-source smoke | Actual configure-time probe, test run, profile merge/export and coverage smoke; binding-graph compatibility of the shared policy change |
| 3.44 | CPack binary/source distributions, exact locked-archive staging, filtered CPack-time source snapshot, package help/ride/audit checks; shared offline extractor reused by image acquisition and network-disabled rehearsal | Binary/install/source smokes and fresh network-disabled rebuild on the target platform |
| 3.45 | Preset-based shell CI lanes, SHA-pinned actions, protected dispatch-only trusted lanes, ARM64 Linux container requirement, release checklist and retained historical matrix | Workflow syntax tooling, actual hosted/trusted lanes and manual macOS GUI acceptance |

The root includes `Sanitizers.cmake` before `Checks.cmake` so the new
control targets enter the first-party source manifest. `Coverage.cmake`
comes after all executables exist. `packaging` is added last; root project
version is initially `0.0.0`, with release version changes explicitly a
separate decision.

The CI evidence trap is installed before dependency acquisition and Linux
image/synchronization. Early failures therefore preserve the original lane
exit code as well as failures during configure/build/test. A Linux lane
must supply its run-unique container name. The source review did not run
the trap or demonstrate its behavior in a shell.

Coverage excludes the nested install build and `slow`, `package`, `audit`
and coverage-smoke tests while collecting profiles, then validates the
newly exported report separately. Required tests must pass before a final
report is published. Coverage aggregation enumerates actual executable
and shared-library target outputs, including the reference test binaries.
It never substitutes an empty profile/report for absent instrumentation.

The sanitizer change adds no instrumentation option or sanitizer flag.
The root graph's shared-runtime lookup needs a build rpath because there
is no interpreter launcher to preload the compiler's runtime. The binding
graph retains its existing preload qualification contract until removal.
Install/package qualification continues to use release builds.

## Packaging decisions found during source review

1. The proposed direct source tree plus staged `.deps` overlay interacts
   incorrectly with broad `/build` exclusions when `source-deps` lives
   inside that same build directory. The implementation instead stages a
   filtered source snapshot at CPack time and adds exactly the selected
   archives. It excludes generated/private trees from that snapshot and
   verifies both names and digests in the extracted source package.
2. `bike-python-audit` falls back to an empty policy object when no policy
   exists beneath the audited root. A binary install does not contain
   `migration/no-python-policy.json`; therefore package smoke explicitly
   passes the authoritative source policy with `--policy`. Calling the
   tool without it would leave several Python-artifact checks empty.
3. The supplied `gui_smoke.sh` rejects every nonzero diagnostic exit. No
   measured acceptance of a nonzero canned-ride status is supplied, so
   package smoke also preserves every nonzero status as a failure.
4. `extract_deps.sh` contains no network branch. Its caller downloads
   archives separately, then both Docker acquisition and offline rebuild
   invoke this same extraction code, validate the lock's relative paths,
   verify each archive digest, and check required extracted files against
   their locked digests. It additionally
   supports the lock's Darwin layouts for the corresponding source package.
5. The old full/slim Python installers remain until Phase 4. The new
   installation guide describes the native distributions and explicitly
   distinguishes their transition status.

## Recheck of the supplied earlier agent report

| Earlier assertion | Current archive evidence | Result of this source review |
| --- | --- | --- |
| Numerics lock has only Darwin/four operations | `native/numerics.lock.json` now has Darwin and Linux records with all eight required symbol names; `cpp-linux-completion-evidence.md` has a dated measured regeneration block, source revision `cb057ef` | Stale for this archive. The lock was not regenerated or edited here. |
| Task 1.03 is only a text import | The archive adds an “actual-checkout integration” evidence block from `fc848de…` and cites merged branch `64c0a500…`; it records restoration/checks of nine compressed datasets | Prior evidence reports a real merge. A Repomix text export does not carry the original Git objects or those binary dataset bytes, so actual ancestry/binary equality cannot be independently re-established here. |
| Nonfinite sin/cos tests only check `isnan` | `native/tests/test_platform_sincos.cpp` now uses `bit_cast<uint64_t>` against per-platform measured tables for ±infinity and signed canonical quiet NaNs; evidence contains the probe source and dated measurements | Stale. No additional unmeasured payload/signaling-NaN expectations were invented. |
| `container.sh pull` accepts only files | The archive now has recursive tree validation, exact placement, staged publication and rollback, plus later source/focused-execution evidence | Stale. The directory-pull implementation was retained; only image-context staging was extended to include the offline extractor. |
| Phase checkpoints, full builds and parity/acceptance remain open | The later evidence explicitly says the focused merge/numerics/sincos/pull work does not close L1/L2/L3 or phase checkpoints | Still correct; this source-only change closes none of those gates. |

The four-input nonfinite table does not claim coverage of other NaN
payloads or signaling NaNs. The original report's demand for complete
bits on the chosen inputs is now met in source; numerical qualification
remains bounded by the retained actual measurements and future full gates.

## Static review performed here

Read the exact plan tasks, native engineering rules, both CMake graphs,
compiler policy/runtime probes, dependency lock/acquisition recipe,
install and GUI smokes, package-audit policy loader, old CI gating and
coverage resolver. Reviewed the changed source interfaces and used text
and JSON inspection plus `git diff --check`; these are not compilation or
execution of the project. No task checkbox, pass count or acceptance row
was promoted on the basis of this work.

Current GitHub ARM64 runner labels were checked against the official
[runner reference](https://docs.github.com/en/actions/reference/runners/github-hosted-runners)
before choosing `ubuntu-24.04-arm`; runtime availability and actual lane
results remain CI evidence, not a source-review conclusion.

## Task 3.36 / D18: retained contracts and explicit test gaps

D18 is not authority to drop every assertion in a Python tooling test file.
The new presets, scripts and CPack definitions are production replacements;
their existence does not port every regression that happened to invoke
them through Python. This review did not change the migration maps or
delete Python. The following distinctions are source-supported, and all
future executions remain unperformed.

### Direct numerics CMake tests that must survive

All functions below are in `tests/portability/test_numerics_discovery.py`.
The record-only fixture creates a CMake `project(... NONE)` and includes
the real `BikeNumerics.cmake`. Failing interpreter/uv shims prove absence of
discovery calls; the synthetic provider bytes are only hashed, never
loaded. These tests exercise the root native contract, not a Python
binding API. No corresponding per-case CTest exists in this change.

| Source function | Retained native contract / remaining successor gap |
| --- | --- |
| `test_record_only_binds_the_committed_facts_without_python` | Darwin committed definitions/provider binding with interpreter discovery forbidden. Release preset covers normal configuration only, not the shim assertion. |
| `test_record_only_requires_an_explicit_mode` | Missing explicit numerics mode must fail instead of silently choosing live discovery. Needs a negative CMake fixture. |
| `test_record_only_linux_binds_acquired_provider_without_python` | Acquired Linux provider, including both ordinary and symlinked dependency roots, must bind without interpreter calls. Needs both fixture variants. |
| `test_record_only_linux_requires_the_acquired_provider` | Both missing-root and missing-library cases must fail even when an alternative live provider exists. Needs two negative fixtures. |
| `test_record_only_linux_requires_matching_glibc` | glibc/musl/absent-family mismatches must fail with host-probe evidence. Needs all three original cases. |
| `test_record_only_linux_rejects_changed_provider_bytes` | Provider SHA mismatch must fail. Production lock verification exists; the regression is not replaced by a successful release preset. |
| `test_record_only_linux_rejects_an_escaping_provider_symlink` | Identical provider bytes outside the acquired tree must fail containment. Needs a symlink-escape negative fixture. |
| `test_record_only_names_a_missing_platform_entry` | Missing platform entry must name the regeneration command. Needs exact failure/diagnostic assertion. |
| `test_record_only_rejects_an_entry_that_describes_another_host` | Platform fields inconsistent with the selected host must fail. Needs tampered-record fixture. |
| `test_record_only_keeps_the_structural_guards` | RECORD_ONLY retains qualified integer-width guards. Needs the original width mutation. |
| `test_record_only_does_not_compare_with_a_measurement` | Changed committed interpolation lowering is consumed without an unauthorized measurement. Needs positive fixture with deliberately changed committed fact and expected compile definition. |
| `test_cmake_guard_rejects_an_unqualified_width_in_the_record` | Shared CMake ABI-width guards must reject unqualified widths. Needs independent mutation regression. |
| `test_cmake_guard_rejects_a_tampered_symbol_in_the_record` | Shared CMake symbol guards must reject tampering. Needs independent mutation regression. |

The remaining live discovery, record-generation, staleness and provider
tests in that file still need per-function decisions. In particular,
retiring interpreter orchestration does not by itself retire qualified
ABI widths, provider identity, eight-operation completeness or committed
record integrity. This table does not claim a complete triage of the
entire numerics-discovery file.

### Compiler and analyzer controls

Source file: `tests/portability/test_native_toolchain.py`. Existing produced
commands are `bike-native-checks --kind ...`, wired as `check_context`,
`check_frontends`, `check_headers`, `check_diagnostic_controls`, etc. The
CTest `migration_T56` contains fixture tests for the produced orchestrator.
Those are concrete successor locations, but their exact scope matters.

| Source functions | Current native location / exact remaining distinction |
| --- | --- |
| `test_warning_control_fires_real_diagnostic` | `check_diagnostic_controls` has `discarded_nodiscard` with real diagnostic markers. It intentionally compiles with `-Wno-error=unused-result`, whereas this source asserts nonzero under `-Werror=unused-result`; the promoted-warning rejection is not an exact port yet. |
| `test_header_self_sufficiency_control_fails_with_diagnostic` | `check_headers` compiles reachable real headers. The deliberate missing `<vector>` fixture, nonzero status and named diagnostic still need a native regression. |
| `test_invalid_source_control_is_rejected_with_diagnostics` | Real frontend sweeps exist. This explicitly invalid C++ source and non-launch-failure diagnostic control has no identified exact fixture successor. |
| `test_libstdcxx_assertions_abort_out_of_bounds` | `check_context`, `check_contracts.cpp`'s `stdlib_hardening` control retains the compile guard, real SIGABRT, benign success, and unhardened compile with UB outcome recorded without a success assertion. Still needs selected-artifact execution. |
| `test_libcpp_extensive_assertion_traps_out_of_bounds` | `check_context` has Darwin libc++ hardening controls and Linux libc++ mode handling; actual mode/signal/benign evidence must be matched to the source before qualification. |
| `test_stdlib_probe_spec_libstdcxx_assertions` | Native context control selects `_GLIBCXX_ASSERTIONS`, rejects `_GLIBCXX_DEBUG`, and requires SIGABRT. The old Python returned-dict shape is language-specific; the underlying hardening contract is retained. |
| `test_stdlib_probe_spec_libcpp_mode_and_stdlib_flag` | Native context control emits the chosen hardening mode and sends `-stdlib=libc++` only to Clang-family drivers. Per-family argument fixture remains to be mapped; a valid host execution alone does not test GNU exclusion. |
| `test_gcc_args_preserve_compiler_launcher` | Production frontend command construction retains launcher prefixes. Needs a fixture asserting the exact launcher/compiler ordering. |
| `test_gcc_args_forward_linux_sysroot_pair_and_joined` | `check_context.cpp` handles paired and joined sysroots. Both forms still need exact argv regression coverage. |
| `test_gcc_args_forward_darwin_isysroot` | Native context/secondary frontend paths retain the selected SDK. Needs the original argv fixture. |
| `test_gcc_args_never_forward_apple_sdk_to_linux_gcc` | Context and secondary frontend paths reject unrecorded sysroot drift; `migration_T56` checks Linux SDK metadata. The secondary-frontend argv fixture is still a separate gap. |
| `test_gcc_args_reject_sysroot_drift` | Production validation exists; needs the source's differing configured/entry sysroot pair. |
| `test_gcc_args_preserve_defines_language_and_abi` | Native secondary-frontend filtering retains semantic defines/C++23/fp-contract/hardening/ABI flags and drops driver-specific optimization/warnings. Exact original argv expectation remains unmapped. |
| `test_analyzer_argv_preserves_launcher_and_drops_output` | Native analyzer argv assembly is present; explicit launcher and removed object-output assertions still need a regression. |
| `test_broken_override_is_not_hidden`, `test_missing_required_tool_lists_checked_candidates` | `migration_T56` includes `t56_tool_override_missing` / `t56_tool_missing`; compare exact source diagnostic/provenance requirements before calling these duplicate tests. |
| `test_clang_sibling_candidates_follow_version_suffix`, `test_gcc_sibling_candidates_only_for_gxx`, `test_triplet_prefixed_gcc_sibling_names`, `test_appleclang_contributes_no_siblings`, `test_darwin_candidates_stay_homebrew_prefixed` | Production candidate logic exists in `check_context.cpp`. No per-function candidate-order fixture mapping is established by this review. Keep planned. |

The root also preserves three concrete native-loader contracts as
`reference.native_loader.test_native_contract_require_control_fails_in_selected_build`,
`reference.native_loader.test_native_contract_executable_lists_registered_cases`
and
`reference.native_loader.test_contract_executable_source_is_in_cmake_manifest`.
They are implemented in `test_native_loader_contracts.cmake`; their source
presence is not an executed pass. The other assertions in the Python
loader file must receive their own binding/native disposition.

### Coverage: resolver behavior is not the LCOV smoke

Source file: `tests/portability/test_native_coverage_tools.py`.
`CoverageTools.cmake` replaces the production resolver and format probe.
`coverage_smoke` only requires a newly generated nonempty LCOV report
containing native source paths. It does not exercise fake-tool search
precedence or error controls.

| Source function | Production replacement / remaining regression |
| --- | --- |
| `test_explicit_missing_llvm_cov_override_fails` | Explicit environment override fails closed; missing-override fixture still needed. |
| `test_linux_tool_search_never_invokes_xcrun` | Linux resolution has no xcrun branch; call-recording negative fixture still needed. |
| `test_compiler_sibling_tools_precede_path_candidates` | CMake checks compiler siblings before PATH; competing-tool fixture still needed. |
| `test_explicit_override_wins_over_sibling` | Explicit override is first; competing override/sibling fixture still needed. |
| `test_xcrun_is_a_darwin_only_last_resort` | CMake Darwin last-resort branch exists; earlier-candidate and Darwin-only controls still needed. |
| `test_non_clang_compiler_is_rejected` | CMake rejects non-Clang coverage configuration; source failure regression not yet ported. |
| `test_missing_compiler_fails` | CMake compiler setup fails closed; missing-compiler failure fixture still needs exact mapping. |
| `test_unknown_override_keys_fail` | Closed Python dictionary-argument API retires with that helper. CMake has named environment variables, not that dictionary; eligible for an explicit language/API drop, not a fabricated native assertion. |
| `test_failing_merge_tool_fails_with_command_evidence` | Probe reports merge command/tool/output on nonzero exit; original exit-7 fake tool regression still needed. |
| `test_export_without_valid_control_data_fails` | Probe validates coverage JSON type, data array and version; malformed export fixture still needed. |
| `test_probe_result_records_tool_versions_and_export` | `coverage_tool_resolution.json` and shared policy context expose resolved tool/version/format evidence; exact schema/provenance comparison still needed. |
| `test_real_toolchain_resolves_and_passes_the_format_probe` | `coverage` preset invokes the real compile/run/merge/export probe. This is a concrete future execution gate; it was not run here. |

### Dependencies, offline packaging and CI

The following representative functions were read specifically to avoid
discarding native assertions embedded in old environment tooling. This is
not a claim that every D18 file has been fully triaged.

| Source functions | Native contract / concrete replacement and remaining gap |
| --- | --- |
| `test_native_dependencies.py::test_numerics_ops_include_solve_eigh_and_ilp64_dots`, `test_op_symbol_tail_does_not_confuse_dgesvd_with_dgesv` | Eight qualified operations and distinct solve/SVD/LP64/ILP64 names remain native requirements. `BikeNumerics.cmake` has production guards; exact malformed-symbol/operation regressions still need native fixtures. |
| `test_native_dependencies.py::test_linux_library_selection_is_not_a_dylib_glob`, `test_darwin_selects_the_versioned_dylib`, `test_missing_exact_engine_is_fatal`, `test_wrong_version_is_rejected_before_lookup` | Exact platform image and engine version remain required. `BikeDependencyLock.cmake`/`BikeDependencies.cmake` enforce locked paths/digests/version, but normal presets cover only valid inputs. |
| `test_native_dependencies.py::test_unversioned_alias_resolves_to_the_same_image`, `test_versioned_name_itself_may_be_a_symlink`, `test_conflicting_real_images_are_fatal`, `test_unversioned_real_library_cannot_prove_the_version`, `test_alias_to_a_different_version_is_fatal` | Real-image identity and allowed/rejected aliases are native dependency contracts. Each needs a documented root-acquisition counterpart or an explicit obsolete-discovery-path decision; successful install/package smoke is not coverage of these mutations. |
| `test_native_dependencies.py::test_validate_dependencies_enforces_the_engine_pin`, `test_validate_dependencies_requires_absolute_existing_paths`, `test_validate_dependencies_checks_library_name_against_platform` | The Python probe envelope can retire, but engine pin/path/platform requirements must remain covered at the locked dependency boundary. Negative fixtures remain open. |
| `test_offline_install.py::test_preflight_wrong_system_fails_before_mutation`, `test_preflight_missing_native_source_fails`, `test_preflight_missing_compiler_fails` | Supported host, complete native source and tool availability survive Python removal. Presets, source staging and CI preflight enforce the new path; exact failure-before-publication regressions still need mapping. |
| `test_offline_install.py::test_preflight_corrupted_wheel_fails_before_mutation` | Locked archive integrity survives: extractor checks archive and required-file SHA-256; `package_smoke` checks packaged locked archives. No corrupted-archive negative CTest is authored here. |
| `test_offline_install.py::test_native_fixture_fails_without_success_claim` | Failed native configure/build must not report successful installation. New scripts use fail-closed process status, but this source's injected failure/no-success-publication control has no exact CTest successor yet. |
| `test_dev_commands.py::test_subprocess_failure_exit_code_propagates`, `test_ci_lanes.py::test_failed_command_stops_the_lane`, `test_lane_failure_exit_code_is_preserved` | `tools/ci_presets.sh` stops on first failure and its EXIT handler preserves status, including acquisition failure. Controlled failure-injection tests remain to be ported. |
| `test_ci_lanes.py::test_workflow_yaml_parses_and_every_use_is_full_sha`, `test_workflow_triggers_and_permissions`, `test_workflow_never_runs_fork_code_on_trusted_runners` | `.github/workflows/portability.yml` retains SHA pins, read-only permissions and dispatch/protected-environment trusted gates. Static source inspection happened; YAML parsing and equivalent policy regression were not run or ported. |
| `test_ci_lanes.py::test_workflow_runs_the_lane_runner_and_uploads_evidence` | Workflow calls `ci_presets.sh` and uploads `output/ci/<lane>` with `always()`. Existing Python test still names the retired driver and requires an explicit successor, not a blanket drop. |
| `test_ci_lanes.py::test_offline_lane_uses_checked_in_offline_tools` | Concrete replacement is `ci_presets.sh release offline`: CPack source archive → fresh network-disabled container → `extract_deps.sh` → release workflow. It does not replace tests of legacy full/slim Python installers until their deliberate retirement. |

Interpreter selection, virtualenv activation, uv-wheel selection and
Python import-shadowing checks may become obsolete when their exact
product surfaces are removed in Phase 4. That decision does not apply to
the native assertions listed above. Unreviewed D18 functions remain
planned; no broad passing or dropped status is inferred from this review.

## Final root-graph source integration check

Inspected `CMakeLists.txt`, `NativeValidation.cmake`,
`RealtimeMeasurement.cmake`, `native/tests/reference/CMakeLists.txt` and
`Phase3Ports.cmake`, following the existing `NativeStartup` /
`RecordingApplication` target declarations. A literal source-path
comparison with the file inventory found no missing concrete source among
the reviewed registrations. Source-manifest globs, include directories and
generated headers were distinguished from required checkout files.

- `bike_cli`, `bike_app`, `bike_viewer`, `bike_legacy`, `bike_random`,
  `bike_golden` and `bike_recording_cli` exist before the new reference
  registrations. `RecordingApplication.cmake` resolves `Threads::Threads`
  before root contract-test registration.
- `bike_realtime_measurement` lists its six implementation translation
  units, links `bike_cli`, and supplies its private generated path header
  to both consumers. `bike-measure-realtime` links that library and has the
  root executable/install relocation rules.
- `bike_validation` lists eleven implementation translation units and
  links `bike_cli`; `bike-validate`, `migration_T54` and
  `reference.system_momentum_drive` have concrete link edges. The
  `migration_T54` target also depends on the executable it exercises.
- The migration harness and allocation-fault object library exist before
  root portable contract tests; the golden test support library exists
  before all reference executables. All six 3.19 and four 3.18 source
  files have registrations, including their extra random/CLI/validation
  edges and `slow`/`golden` labels.
- Sanitizer controls precede the shared check-context/source-manifest
  emission; coverage enumeration follows the check executables; packaging
  is last. Ordinary preset test filters exclude `audit` explicitly.

This check inspected source declarations and filesystem paths only. It
does not establish that CMake generation, compilation, static archive
ordering, linkage, installed loader paths or any test execution succeeds.
