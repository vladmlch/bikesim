# Task C336b — test-port 3.36b report (numerics discovery; `portability`, T53)

**Batch**: 3.36b — `tests/portability/test_numerics_discovery.py` (113
test-port-map records, all `planned` at batch start). The plan calls this
file out explicitly: the numerics surface cannot be dropped wholesale
because it carries retained native contracts — `BikeNumerics.cmake`
record-only consumption, provider identity/paths, ABI integer widths,
exported-symbol spellings, libc family, and the identity-header
projection.

**Lane**: `.worktrees/pfc-lane-e`, branch `lane/e`, base `4fe5ce7e`
(post-3.29b/3.33a integration tip). No merge, no push. Commits on
`lane/e`: `e4109aea` (fixture + registration + pytest fixture repair),
`e3a60fe7` (host-portable record keys in live cases), `07a8f5cd`
(ledgers).

## Status: 70 `port`, 43 `drop`, 0 `planned`, file `replacement_landed`

Ported surface: **one `cmake -P` CTest contract script** —
`native/tests/migration/migration_T53_numerics.cmake` (1528 lines, 109
cases) — registered as CTest `migration_T53_numerics` in
`native/cmake/Checks.cmake` with a same-named convenience target.

A **dedicated sibling fixture** (not `migration_T53_portability.cmake`)
was chosen: the subject is exclusively `BikeNumerics.cmake` /
`BikeProviderIdentity.cmake` / `numerics.lock.json` contracts, while the
tranche-1 fixture covers platform/policy/root-configure tooling — and it
lives on `lane/c`, so there is nothing on this lane to extend. Fixture
style follows both precedents: scratch `project(... NONE)` configures
that include the real `BikeNumerics.cmake` in RECORD_ONLY and LIVE modes
(migration_T53 pattern) plus file-level committed-lock invariants and
`-P` identity-declaration harnesses (migration_T56 pattern). The LIVE
checker's interpreter is replaced by a stub whose verdict echoes the
record key — the same stand-in the Python tests use
(`_stub_checker`), which is exactly what lets the CMake guards be
reached with checker-rejectable data.

| surface | records | landing |
|---|---|---|
| committed-lock invariants (schema, both platform entries, identity declarations) | 7 | `committed_lock_schema_is_v1`, `committed_lock_{darwin,linux}_record_facts`, `identity_declarations_take_facts_from_committed_record` |
| RECORD_ONLY scratch configures | 30 | mode-forgotten refusal, darwin/linux positive binds (incl. symlinked root), missing root/library, libc family ×4, sha mismatch ×2, misshapen digests, escaping symlink, directory provider, missing entry, other-host entry, structural guards, no-compare, unknown backend, non-object entry, missing lock file |
| width/symbol/lowering/member/parametrize vectors | 22 records → 63 expanded cases | `record_only_{openblas,accelerate}_width`, `record_only_symbol_identifier_{accepts,rejects,rejects_non_string}`, `record_only_symbol_spelling`, `record_only_lowering`, `record_only_missing_record_field`, `record_only_provider_identity`, both path-form cases |
| LIVE scratch configures (stub checker) | 11 | `live_resolves_auto_backend[{Darwin,Linux}]`, wheel-relative provider, outside-site-packages, backend mismatch, missing record, non-spawnable interpreter, stale verdict propagation, missing platform entry, both cmake-guard rejections, `live_apple_system_requires_build_id[0,1]` |
| **ported total** | **70** (62 contract + 8 structural) | `migration_T53_numerics::<case>` |

Dropped surface (43 records, all `python-only` with per-record
justifications): `validate_numerics` live-evidence members
(threading/numpy_imports/provider_config/evidence counts/numpy_package),
`validate_numerics_record_entry` member-shape rejections, the
`require_same_library` resolved-identity helper, duplicate-provider
validator rejections, `_numerics_abi_probe_source` C-probe generation,
`validate_dependencies` document boundary, probe orchestration
(real-env, scipy-interposition, repeated-probe), CLI plumbing
(stdout/mutual-exclusion/checker exit codes), record writer/projection
determinism and staleness comparison, `check_numerics_record` verdict
API, libc-family classifier, and the two checker-side live-document
rejections whose committed-record twins are ported separately. These
die with the interpreter at T63; where a committed invariant survives
it is named in the record's evidence.

No golden fixtures needed: the batch is contract/structural only.

## Case map (test → native case, abridged)

- Identifier grammar: `test_checked_symbol_accepts_valid_identifiers` →
  `record_only_symbol_identifier_accepts[6]`; `..._rejects_unsafe_identifiers`
  + `test_symbol_names_must_be_checked_identifiers` →
  `record_only_symbol_identifier_rejects` (same 11 vectors);
  `..._rejects_non_strings` → `[null]`,`[42]`,`[true]`,`[["cblas_ddot"]]`
  (JSON `true` renders `ON`, so it is rejected at the spelling stage —
  documented CMake-specific difference, same rejection contract).
- Widths: `test_openblas_requires_ilp64_on_both_interfaces` → 6 of 7
  `record_only_openblas_width` vectors; `test_integer_widths_must_be_ints_not_bools`
  → `[true-64]`; `test_accelerate_requires_the_32_64_split` → 4
  `record_only_accelerate_width` vectors; the writer-side width check →
  `record_only_accelerate_width[32-32]`.
- Symbol spellings: six spelling tests → the 7-vector
  `record_only_symbol_spelling` loop (LP64-vs-ILP64 suffixes, legacy-vs-
  NEWLAPACK interfaces, same-width substitution rejected on both
  backends); `test_accelerate_legacy_dot_keeps_its_separate_32_bit_contract`
  → `committed_lock_darwin_record_facts`.
- Lowerings: `test_interp_lowering_spellings` +
  `test_multiply_broadcast_nan_winner_spellings` → the 18-vector
  `record_only_lowering` loop (incl. the newer distributions /
  scipy_correlate facts, same guard and needle).
- Provider identity: `test_file_identity_*` → `..._sha_mismatch[changed,
  forged]` + `..._misshapen_provider_digests` + `committed_lock_linux_record_facts`
  (no committed `build_id`); `test_apple_system_identity_*` →
  `committed_lock_darwin_record_facts` (sha256 NULL, framework paths,
  `build_id` absent) + `live_apple_system_requires_build_id[0,1]`;
  role/kind rejections → the 5-vector `record_only_provider_identity`;
  `test_library_roles_must_cover_blas_and_lapack` → the role vectors.
- RECORD_ONLY mode: all 11 record-only tests map 1:1 (incl. the
  parametrized acquired-provider and glibc vectors);
  `test_record_only_does_not_compare_with_a_measurement` keeps the
  positive semantics (flipped fact configures — LIVE is the comparator).
- LIVE mode: AUTO resolution on both platforms, wheel-relative provider,
  backend mismatch, missing record/platform entry, non-spawnable
  interpreter, stale-verdict propagation (also the legacy-graph
  configure twin), both verbatim cmake-guard rejections, apple-system
  `build_id` evidence.
- Committed record/projection/libc/legacy: `committed_lock_*_record_facts`
  pin both platform entries' exact member sets; `..._requires_matching_glibc`
  covers family-change, unknown-family and committed-version rejections;
  `identity_declarations_take_facts_from_committed_record` runs the real
  `BikeProviderIdentity.cmake` on both committed entries and compares the
  generated policy JSON field-by-field (build_id stays live).
- Drops with named surviving coverage: the smuggled-member validators
  note `committed_lock_*_record_facts`' exact member-set pins; the
  checker-side live rejections note their committed-record twins
  (`live_cmake_guard_rejects_*`).

## Verification

| gate | result |
|---|---|
| darwin-arm64 `cmake -P` standalone | **passed (109 cases), 12.7 s** |
| `ctest --test-dir build/native-release -R '^migration_T53_numerics$'` | **Passed, 12.61 s** (test #14), 0 skips |
| linux-aarch64 `ctest --test-dir /src/build/native-release -R '^migration_T53_numerics$'` (container `bike-dev-pfc-lane-e`) | **Passed, 1.56 s** (test #15), 0 skips, `passed (109 cases)` |
| `uv run pytest tests/portability/test_numerics_discovery.py` (oracle, darwin-arm64) | **168 passed in 96.62 s** |

Sync to the container was a non-git tar/`docker cp` into `/src` per the
lane's container workflow; `cmake -S . -B build/native-release` +
`cmake --build` (clang-18/Ninja, root `CMakeLists.txt`) preceded ctest.
`tools/run_tests.sh` was not used, per the batch instruction.

Negative proof: tampering the committed `ddot` spelling in
`native/numerics.lock.json` (`cblas_ddot_TAMPERED`) fails the fixture at
`committed_lock_darwin_record_facts` with the exact diff —
`'cblas_ddot_TAMPERED' != 'cblas_ddot'` (verified red, then restored).

### Port-authored bugs found and fixed during red→green

- `string(JSON TYPE ... build_id)` errors on an absent member —
  absence is proven via `GET` + `ERROR_VARIABLE` instead (the committed
  record must not carry `build_id`).
- `string(JSON GET ... library_paths)` returns the whole array; member
  counts need `string(JSON LENGTH ...)`.
- JSON `true` renders as `ON` (identifier-grammar-passing) — the
  non-string symbol case asserts the backend-spelling rejection for
  that vector and the grammar rejection for null/number/array.
- `get_filename_component(REALPATH)` canonicalizes `/tmp` →
  `/private/tmp` on Darwin — provider-path assertions compare against
  the canonicalized path.
- `CMAKE_HOST_SYSTEM_PROCESSOR` is unset in `cmake -P`; `_uname_m`
  comes from `uname(1)` — and, found by the Linux run: `_darwin_record`
  locks are keyed `darwin-${_uname_m}`, so hardcoding `darwin-arm64` as
  the stub's verdict key failed four cases at the record lookup on
  aarch64 (`committed numerics record has no entry`). The stub now
  echoes `darwin-${_uname_m}`, matching the lock on every host
  (commit `e3a60fe7`); LIVE mode's entry lookup precedes backend and
  provider checks, which is also why the checker-failure cases
  (`stale_verdict`, `non_spawnable`, `missing_numerics_record`) were
  already host-portable.
- Python oracle fixture repair at batch start: the projection-key
  expectation at ~line 1534 now includes `distributions` and
  `scipy_correlate` (committed facts added since the test was written);
  the file is green end-to-end (168 passed).

## Ledger

- `migration/test-port-map.json`: all 113 records updated —
  `disposition` 70 `port` / 43 `drop`; `native_test_id` =
  `migration_T53_numerics::<case>` (comma-joined multi-case, no
  placeholders); `native_source` = the script path; `collected_node_ids`
  = measured `pytest --collect-only` ids on this host (168 nodes across
  the file, parametrize tables verbatim); `assertions_capture` =
  measured node/assert + `pytest.raises` counts with the green oracle
  run; `comparison_class` = `contract` (62) / `structural` (8) /
  `python-only` (43); `line`/`end_line` refreshed to current AST
  positions; evidence = dated lane entries citing both-platform ctest
  facts. File ends with LF (byte-verified).
- `migration/python-removal.json`: the file's row →
  `status: replacement_landed`, `replacement_paths` =
  `native/tests/migration/migration_T53_numerics.cmake`,
  `verification_ids` = `migration_T53_numerics`, live
  `sha256`=`fb9ecb1d…` / `line_count`=1981 (snapshot fields frozen),
  `deletion_gate` aligned to the landed-row text; the inventory-note
  scratch fields were dropped per the 128-row landed convention. LF
  verified. The Python file itself stays in place — deletion is the T63
  gate.
