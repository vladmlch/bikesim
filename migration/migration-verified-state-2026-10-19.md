# Verified migration state — 2026-10-19

Ground truth established by direct code/test verification (not by reports).
Use this when updating tracking documents. Do NOT over-claim: these results
cover build + registered migration tests + static sweeps only. Runtime
qualification, per-source oracle equivalence, T61/T62 acceptance and T63
deletion gates remain open unless listed otherwise.

## Executed evidence (this checkout, today)

- Build: `native/build/bin/*` produced 2026-10-19 01:13 (9 CLI binaries +
  bike-native-ride + bike-test-child + bike-research-sweep). `bike-sim
  --help` runs; `otool -L` shows NO libpython (links libmujoco, Accelerate,
  libduckdb, libc++).
- CTest `native/build`: **57/57 PASS** (2026-10-19). Registered tests:
  native_contract_tests, native_distance_tests, native_station_tests,
  native_bench_{bare,glue,heavy}, migration_T02..T51 (all), migration_T03,
  migration_T30_native_contacts. NO migration tests exist for T52-T63.
  Earlier 03:42 run had migration_T05+T11 red; both pass now.
- Static sweeps (native/build/native_check_summaries/*.json, all today):
  contracts, diagnostic-controls, frontends, headers, odr, tidy, cppcheck,
  analyzer, context — all `status: passed`.
- pytest `.pytest_cache/lastfailed` (10:01 today): exactly the 24-failure
  documented baseline listed in docs/TESTING.md:717-743. No new failures.
- Working tree: 288 modified files uncommitted (+3451/-2244), the
  T38-T51 recording/application integration + post-review fixes are in
  flight and NOT committed. Branch: impl/python-free-cpp, last commit
  4f630db (Oct 17).

## Open review findings (native-waves-commit-review.md) — re-verified today

- F1 FIXED: initialization/capture.cpp now guards tire_state() behind
  stepper.has_tire() (was the native_reference bootstrap crash).
- F2 OPEN (P1): runtime/step.cpp ~1449-1455 still writes 0. to rear_drive
  for drive_mode=="ideal_speed_control"; capabilities.cpp:20-21 admits the
  mode. Oracle: src/bike_sim/sim/ride/physical_runtime.py:389-394 calls
  cruise.compute(). Misleading comments claim the branch is unreachable.
- F3 OPEN (P2): step.cpp ~1295-1310 skips resistance for native_reference
  ("engine-contact-only"), but oracle applies resistance from contact
  snapshots when tire is None (physical_runtime.py:384-386).
- F4 FIXED: research/policy.cpp policy_control_from_abi() calls
  runtime::control_validate_for + translates to PolicyFailure;
  migration_T35 green.
- F5 OPEN (P2): step.cpp ~1420 passes `active && !initializing_` as
  pedal_recovery to rider controller compute(); oracle passes
  cfg.seated_climb.enabled (physical_runtime.py:362-367).

## Per-task verified status (task-graph vocabulary:
## planned -> ready -> in_progress -> review -> verified -> merged)

- verified: T00,T01,T02,T03,T04,T05,T06,T07,T08,T09,T10,T11,T12,T13,T14,
  T15,T19  (cards checked, recorded green gates, migration tests green)
- review:   T16,T17,T18,T20,T21,T22,T23,T24,T25,T26,T27,T28,T29,T30,T31,
  T32,T33,T34,T35,T36,T37,T38,T39,T40,T41,T42,T43,T44,T45,T46,T47,T48,
  T49,T50,T51  (wave-delivered; migration tests green 2026-10-19; wave
  acceptance reviews still open; open findings live in the T26-T33 block;
  T38-T51 code uncommitted)
- in_progress: T56 (bike-native-checks exists and runs
  headers/diagnostic-controls/context/contracts kinds; bash+pytest
  orchestration and tools/*.py remain)
- planned: T52,T53 (1611 test-port records, 0 ported, no
  native/tests/reference/), T54,T55,T57,T58,T59,T60,T61,T62,T63

## Ledger facts

- python-removal.json: 459 rows, all status "planned" (correct — deletion
  is T63). 213 rows carry `implementation` records covering ALL rows of
  T16-T18,T20-T51; ZERO implementation rows for T52-T59. Every impl record
  says execution=not_run_user_prohibited_compilation_and_execution — STALE,
  tests have since run.
- file-work-units.json: 459 rows mirroring the same files.
- test-port-map.json: 1611 records, all disposition "planned" — CORRECT
  (T52/T53 not started; native/tests/reference/ does not exist).
- Wave manifests with stale execution=not_run: native-startup-
  implementation.json (9 tasks T16-T18,T20-T25), physics-research-
  implementation.json (12 tasks T26-T37), recording-application-
  implementation.json (14 tasks T38-T51). All status implemented_unverified,
  packages_closed [].
- migration/cards/T*.md: only T11,T12,T13,T14,T15,T19 contain checked
  boxes. Cards T16-T51 list per-file (Fxxx) work units implemented by the
  waves but never marked.

## Editing rules for status updates

- Date everything "2026-10-19". Keep historical sections; append dated
  status blocks rather than rewriting history.
- Vocabulary: use honest labels — "tests green" means the registered
  ctest node(s) listed above passed; it does NOT mean runtime-qualified,
  oracle-verified, or acceptance-closed.
- Do not mark any task "merged" (nothing merges to main here) and do not
  mark T52-T63 beyond planned/in_progress as evidenced above.
- Do not edit docs/superpowers/plans/* (historical plans), baseline logs,
  or oracle manifests' data payload.
- No git commits — the tree already has unrelated in-flight changes.
