# Task C329b — test-port 3.29b report (contract checks, check tools; `checks`, T53)

**Batch**: 3.29b — `tests/reference/test_native_check_tools.py` (37 records)
and `tests/reference/test_native_contract_checks.py` (17 records);
**54 records / 62 collected nodes / 157 source-level `assert` statements**.
All 54 port into the `cmake -P` fixture harness
`native/tests/migration/migration_T56_check.cmake`, surfaced as a single
CTest node `migration_T56` whose `native_test_id` names are
`migration_T56::<comma-separated b29_* case names>` (68 distinct cases).
Port-map rows carry `owner_task: T53`, `comparison_class: contract`.

**Lane**: `.worktrees/pfc-lane-e`, branch `lane/e`, base `fe849692`
(post-3.29a merge-back integration tip). No merge, no push.

## Status: landed — 54 `port`, 0 `drop`, 56 `planned` remain in the two files

All 54 batch records are `port` into `migration_T56` fixture cases.
Unlike 3.29a, **neither source file is fully migrated**:
`test_native_check_tools.py` keeps 51 `planned` rows and
`test_native_contract_checks.py` keeps 5 `planned` rows (later batches).
Consequently `migration/python-removal.json` is intentionally **not**
modified — both source rows keep their pre-batch status and the Python
files stay authoritative-oracle until T63.

## Triage

| Source file | records | collected nodes | port | drop | planned (remaining) |
|---|---|---|---|---|---|
| `test_native_check_tools.py` | 88 total / 37 in batch | (batch nodes incl. below) | 37 | 0 | 51 |
| `test_native_contract_checks.py` | 22 total / 17 in batch | (batch nodes incl. below) | 17 | 0 | 5 |
| **batch total** | **54** | **62** | **54** | **0** | — |

No golden fixtures were needed or created: the batch is contract-only —
every ported behavior asserts process exit codes, stable diagnostics, or
structural JSON emitted by `bike-native-checks` on synthetic scratch
fixtures; no Python oracle value is captured.

## Port map (`migration_T56::b29_*`, 68 cases)

The harness extends the pre-existing T56 `run_case` runner (parses
`EXPECT_EXIT`/`EXPECT_STDOUT`/`EXPECT_STDERR`, runs each fixture in an
isolated scratch dir, aggregates failures) with `b29_*` helpers that
generate scratch repositories, compile databases, source manifests,
context JSON, fake tool binaries, fake extensions, CTest registrations,
and provenance records, plus JSON assertions (`expect_json`,
`expect_json_item_names`, `expect_json_contains`, `expect_json_length`,
`expect_json_member_set`, `expect_json_equal`).

Coverage, grouped by the oracle behaviors they mirror:

- **Compile-database ingestion**: `b29_entries_arguments`,
  `b29_entries_command`, `b29_entries_relative` (both JSON forms,
  entry-directory-relative file resolution); `b29_empty_db`,
  `b29_missing_source`, `b29_no_target`, `b29_unexpected_tu`,
  `b29_dup_tu` (fail-closed on duplicate identical TU);
  `b29_shared_source`, `b29_multi_context` (same source under several
  target contexts selected once per context);
  `b29_compiler_mismatch`, `b29_compiler_symlink`,
  `b29_sysroot_mismatch`, `b29_iso23`.
- **Include/library selection**: `b29_include_paths`,
  `b29_library_paths`, `b29_missing_include`, `b29_joined_I`,
  `b29_joined_isystem`, `b29_joined_iquote`, `b29_joined_idirafter`.
- **Sweep frontends**: `b29_headers`, `b29_gxx_frontends`,
  `b29_gxx_odr`, `b29_tidy_default`, `b29_tidy_missing_build`,
  `b29_tidy_wrapper_missing_build`, `b29_folded_tidy`,
  `b29_llvm_report` (capability-gated: skipped where the toolchain
  reports coverage unsupported), `b29_run_coverage_empty`,
  `b29_hfilter_main`, `b29_hfilter_original-root`,
  `b29_hfilter_renamed-root`.
- **Tool health / finding classification**: `b29_stable_rc127`,
  `b29_stable_rc2`, `b29_stable_rc139`, `b29_stable_warn`,
  `b29_alpha_report` (analyzer alpha findings report-only),
  `b29_alpha_crash` (analyzer crash fails tool health),
  `b29_positive`.
- **Suppressions / diagnostic controls**: `b29_suppression_manifest`,
  `b29_suppression_match`, `b29_suppression_version`,
  `b29_move_suppression`, `b29_move_disabled`, `b29_diag_controls`
  (sorted control-name assertion).
- **Context / SDK / hardening**: `b29_context_control`,
  `b29_sdk_alias`, `b29_sdk_default`, `b29_sdk_missing`,
  `b29_hardening_default`, `b29_hardening_fast`,
  `b29_hardening_invalid`, `b29_hardening_def` — darwin asserts SDK
  controls plus `EXTENSIVE`/`FAST` `sizeof` equality, non-empty
  `timings_ns.*` arrays and assertion-activity fields; linux asserts
  libstdc++ hardening evidence.
- **Contract fixtures** (`b29_contract_fixture`/`b29_contract_run`):
  `b29_contract_base`, `b29_ctest_registration`,
  `b29_empty_registry`, `b29_bench`, `b29_prov_count`,
  `b29_prov_imported`, `b29_ext_zero`, `b29_ext_two`,
  `b29_loader_bypass`, `b29_engine_loadmodel`,
  `b29_engine_forward`, `b29_engine_unguarded_loadmodel`,
  `b29_engine_unguarded_forward`, `b29_engine_unmanifested` —
  manifest/source membership, compile-database selection, CTest
  registration, benchmark membership, loader provenance, extension
  count/provenance, engine boundary files, hardening definitions.

## Checker fix (in-batch, required by `b29_multi_context`)

`native/src/tools/check_contracts.cpp` —
`manifest_selection.source_count` now counts **unique resolved source
paths** via an `expected_seen` set: one source selected under several
target contexts counts once, matching the oracle's accounting instead of
inflating per context.

## Port substitutions and limitations

Disclosures from the 3.29b review round (PASS-with-reservations); no
code semantics changed by this round.

- **Coverage-provenance records are partial ports onto a substituted
  surface.** `test_coverage_provenance_supports_a_custom_build_and_exact_report_object`
  → `b29_contract_base` exercises only ~3 of the 9 oracle asserts: the
  exit code plus `selected_build`/`imported_extension` — read through
  the **`--kind contracts`** `loader_provenance` surface
  (`native_test_provenance.json`), **not** the coverage path. The
  `coverage_provenance.json` surface (`report_object`, `profile_count`,
  `native_test_count == 5`, `report_paths.{text,export}` existence, the
  `TOTAL` line in the text report) is **unexercised**: native
  `RunCoverage.cmake` writes no coverage-provenance file at all (zero
  `provenance` references; the file is emitted only by
  `tools/native_coverage.py`), so there is no native provenance
  validation on the coverage path to port against. The record keeps
  `disposition: port` with the partial-port coverage stated in its
  ledger `evidence` string.
- **`b29_prov_count` weakened trigger.** The oracle's case rejects a
  *missing* `native_test_count`; the C++ record treats the field as
  optional-but-validated, so the fixture plants `"unknown"` to reach the
  same reject path (disclosed inline at
  `migration_T56_check.cmake` ~2281). Rejection of an *absent*
  `native_test_count` is not covered natively.
- **Produced-diagnostic convention.** Where the produced checker emits
  its own stable diagnostic instead of the oracle's Python wording, the
  fixture asserts the produced needle:

  | oracle needle / field | produced needle / field |
  |---|---|
  | `CTest registration is incomplete` | `native_contract_tests is not registered with CTest` |
  | `missing stdlib hardening definition` | `missing libc++ hardening definition` |
  | `imported extension does not match selected build artifact` | `imported extension does not match selected artifact` |
  | `no nonempty profraw files` | `no raw coverage profiles` |
  | `controls[].diagnostics` | `controls[].matched_diagnostics` |

## Verification evidence

- **Oracle green (darwin-arm64)**: `uv run pytest
  tests/reference/test_native_check_tools.py
  tests/reference/test_native_contract_checks.py -x -q` — **125 passed,
  1 capability-gated skip** in 5524.25 s. (The contract file alone:
  24 passed in 14.41 s.) Collection measured for the 54 batch records:
  62 node ids recorded verbatim in `collected_node_ids`; 157 source
  asserts recorded in `assertions_capture`. Linux oracle run not
  required (no live-oracle/golden comparisons in the batch).
- **CTest darwin-arm64** (`build/native-release`, AppleClang, Release):
  `ctest -R '^migration_T56$'` — **PASS**; 1213.50 s on the run recorded
  in ledger evidence (1274.68 s on the final post-fix rerun).
- **CTest linux-aarch64** (container `bike-dev-pfc-lane-e`, committed
  tree synced via `container.sh sync` → `/build/root-ref`, clang++-18
  Ninja Release): `ctest -R '^migration_T56$'` — **PASS** in 1291.08 s;
  the LLVM-coverage fixture reports capability-unsupported and the case
  is explicitly skipped, not failed.
- **Static registration**: `native/cmake/Checks.cmake` —
  `add_test(NAME migration_T56 … -P migration_T56_check.cmake)` under
  `BUILD_TESTING` with the `migration_T56` build target wrapper; the
  node lives only in the checks-enabled graphs, so it runs standalone
  via `ctest -R`, not inside the gate's 62-test `native/build` set.
- **Outer gate** `bash tools/run_tests.sh native` (darwin-arm64,
  `native/build`): build + **all nine sweeps green** (frontends none,
  tidy none + documented suppressions, analyzer alpha report-only,
  cppcheck none, odr none, shellcheck clean, headers 265 headers /
  9322 probes clean, context + diagnostic-controls clean); **CTest
  60/62 — exit 8** with exactly the documented baseline reds
  `migration_T09` (`post-advance integration vector matches the
  oracle`) + `migration_T10` (`first transition matches the Python
  oracle`, the T01 `transitions.json` frozen-oracle family), matching
  the recorded gate signature verbatim; **zero unexpected reds**.
  Profile stops at the first red stage per documented gate-abort
  behavior. `migration_T56` is not in that 62-test set (verified
  separately, above).
- **Red→green discipline**: every fixture negative control asserts a
  specific nonzero exit or diagnostic needle (e.g. `shellcheck: rc=1`,
  `missing source file`); harness `_failures` aggregation verified by
  construction during bring-up (malformed-JSON and ordering faults each
  surfaced as named failures before their fixes).
- **Run-log retention limitation**: no in-tree `migration_T56` run logs
  are retained — the ~20-minute dual-platform runs' evidence lives in
  the ledger `evidence` strings and the timings recorded above, so
  downstream verification rests on the lane runs themselves.

## Ledger

- `migration/test-port-map.json`: 54 records → `port` /
  `comparison_class: contract`, `native_source:
  native/tests/migration/migration_T56_check.cmake`, measured
  `collected_node_ids` (62 nodes) + `assertions_capture` (157 asserts),
  per-record dual-platform `evidence` rows. Headers unchanged: **2391
  static records / 2375 top-level test functions**. Totals: `port`
  858 → **912**, `planned` 1096 → **1042**, `golden` 349, `drop` 88 —
  diff verified to touch exactly the 54 batch records; canonical
  serializer formatting byte-stable under load/dump round-trip.
  Review round: the `b29_contract_base` record's `evidence` string was
  amended to state its partial-port coverage (see above); still
  canonical (indent=2, ensure_ascii, trailing LF).
- `migration/python-removal.json`: **unchanged** — both source files
  retain `planned` rows (51 + 5), so neither row qualifies for
  `replacement_landed`. Deletion stays gated at T63.

## Defects found

All fixed in-batch (inside `feec6f54`); none outstanding.

1. **CMake `;`-list JSON corruption** — multi-argument `set()` calls
   stored semicolon lists that broke generated fixture JSON; all JSON
   fragments now single-argument strings.
2. **Compile-database `"command"` form unsupported** — `b29_ccdb_args`
   accepts both `"arguments"` arrays and `"command"` strings via
   `separate_arguments`.
3. **Scratch configure picked an unsuitable system Python** — the
   harness prefers `${REPO_ROOT}/.venv/bin/python` (fallback
   `python3`/`python`; `/opt/bike-venv` in the container) via
   `-DPython_EXECUTABLE`.
4. **`expect_json_equal` argument bleed** — function name/file args
   leaked into member paths; path construction now compares only real
   member args.
5. **Context timing JSON misuse** — summary doc needed `file(READ)`
   before `string(JSON … LENGTH)`; timings asserted as non-empty
   arrays, not scalars.
6. **Diagnostic-control ordering** — expected names sorted to match the
   checker's emitted order.
7. **Shellcheck-0.11 diagnostic needle** — `SC2168 (error):` does not
   match the native `warning:|error:` regex, so the fixture asserts the
   stable `shellcheck: rc=1` instead.
8. **Linux joined-include fixtures** — empty `_b29_def` produced a
   `[""]` compile-definitions array the checker rejects; fixtures now
   use the platform hardening-definition representation
   (`_b29_defs_json`), fixing `b29_joined_{I,isystem,iquote,idirafter}`
   on libstdc++.
9. **`check_contracts` unique-source accounting** — see checker fix
   above; without it `b29_multi_context` (and the real orchestrator's
   multi-context builds) overcounts `source_count`.

Out-of-batch edits to **pre-existing** T56 cases, also inside
`feec6f54` — legitimate repairs, disclosed in the review round:

- **`t56_scripts_depth2_bad`** — planted trigger changed
  `for f in $UNQUOTED; do echo $f; done` → `local unquoted=$UNQUOTED`
  (deterministic SC2168 under shellcheck 0.11) and the asserted needle
  changed `linux/bad.sh` → `shellcheck: rc=1` (the shellcheck-0.11
  diagnostic-format drift already documented inline at
  `migration_T56_check.cmake` ~831–837).
- **`t56_audit_doc_bad` / `t56_audit_doc_clean`** — `\`` escaped
  backticks in the planted markdown fences → literal backticks: CMake
  3.x keeps the escape while 4.x strips it, which produced different
  fence bytes per platform; written unescaped for byte parity.

Review-round harness nits fixed (no run semantics change; verified by
extracted-block `cmake -P` drivers plus a full-script `cmake -P` parse
check):

- **`expect_json_equal` separator guard** — `if(_sep … EQUAL ${ARGC})`
  was unreachable (`_sep` is bounded to `ARGC-1` by the scan loop); the
  guard now compares against `ARGC-1`, so a trailing `=` (empty
  path-b) is rejected as intended.
- **`b29_suppression_manifest` RANGE guard** — `foreach(_i RANGE 0 -1)`
  iterates indices `0` and `-1` on an empty `suppressions` list, so the
  loop is now guarded by `_sup_count GREATER 0` (the convention already
  used by `expect_json_item_names`). The in-repo manifest is non-empty,
  so the bug never fired in the lane runs.

## Commits

| SHA | Unit |
|---|---|
| `feec6f54` | 3.29b T53: 54-record port into migration_T56 + `check_contracts` unique-source fix + ledger update (darwin + linux `ctest -R migration_T56` green) |
| `7f32cf8a` | task-C329b-report.md (gate evidence + triage/dispositions) |
| (this commit) | docs(3.29b): review round — coverage substitutions + out-of-batch edits disclosed; `expect_json_equal`/`b29_suppression_manifest` guard nits |
