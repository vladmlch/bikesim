# Task F5 — spindle rear-leg-chain effort: investigation, native port, two-platform qualification

**Lane**: `.worktrees/pfc-lane-e`, branch `lane/e`, base `d8242c88`.
No merge, no push.

## Defect hypothesis and verdict

**Reported**: in the enabled-compute spindle path, rear-leg-chain native
efforts collapse to ≈ 0 while the Python oracle returns nonzero values;
the disabled-compute legacy `efc` path was green.

**Verdict: does not reproduce.** The native spindle path emits nonzero,
oracle-matching rear hip/knee torques through the rear-driving crank
half on both qualified platforms. The permanent regression
`test_spindle_rear_chain_effort_matches_python` (4.19 / 3.67 / 0.0 /
π / π/2 commanded cases) freezes that: it stages the pinned plant,
runs `compute()` through the enable path, and asserts nonzero
oracle-matching rear-chain torque. The existing coupled-rider runtime
test (`reference.native_coupled_rider_task`) already passes on darwin
and directly validates the F4 forward-refresh ordering this hypothesis
blamed — consistent with non-reproduction.

## Confirmed defects found and fixed during the port

1. **`solved_effort` malformed-state OOB read (native, real).**
   `native/src/rider/spindle_controller.cpp` indexed
   `incoming_velocity[dof_adr]` before any width validation — a
   truncated state span was an out-of-bounds read where the oracle
   raises `IndexError`. Fixed with an upfront nq/nv width gate throwing
   `std::invalid_argument`, matching `strength_limited` /
   `strength_violations`. Commit `7f70dfab`.
2. **`_damped_config` duplicate TOML key (test, unrelated to physics).**
   The helper appended `activation_tau_s = 0.05` over the file's
   existing `0.025` — duplicate-key parse failure in two nodes. Fixed by
   rewriting the assignment in place. Bundled in `3336515f`.

## Port delivered

- `native/tests/reference/test_native_spindle_controller.cpp`
  (registered `reference.native_spindle_controller` in
  `Phase3Ports.cmake`) — ports the 22-symbol oracle family plus the
  rear-chain regression: waveform/ceiling kernels, compute torque+state
  parity, initialization, envelope/write/state codec, solved_effort,
  reset, strength probes, malformed-input rejection (2754 checks).
- Python golden capture added across the family
  (`tests/reference/test_native_spindle_controller.py`); pinned plant
  fixture `tests/fixtures/migration/spindle/pinned/` (the pre-existing
  `welded_pinned/model.mjb` does not hash-match the fresh pinned plant
  and was not reused). Capture script
  `tools/migration/_f5_spindle_capture.py`.

## Cross-platform comparison bound

Oracle numpy/libm math is **not bitwise-stable** between darwin-arm64
and linux-aarch64: the linux capture diverges ~2e-9 in leg-kinematics
trig (`rider_hip_front`, 1.66e-9 rel) and 1.06e-8 rel through the
excitation solve (`solution_excitation_nm`), while darwin matches
bit-for-bit. All f64 records are `kind='close'` at **rtol=1e-7 /
atol=1e-8** — ~three decades below any physically meaningful deviation,
one decade of headroom over the widest observed platform spread.
Canonical `|u1` state documents stay platform-agnostic fixtures; the
C++ side consumes them via `bytes_input` and deep-compares — keys,
order, array lengths and literal leaves exact, numeric leaves within
the same bound. Bool-only violation/limit documents remain bitwise.
`initialize_velocity` keeps its earlier `close` record (activation
warmup is not bitwise identical even on one platform).

## Verification

- darwin-arm64: `pytest tests/reference/test_native_spindle_controller.py`
  — **47 passed**; `reference.native_spindle_controller` — **PASS 2754
  checks** (build/native-release + ctest).
- linux-aarch64 (`bike-dev-pfc-lane-e`): same pytest — **47 passed**;
  CTest `reference.native_spindle_controller` in `/build/root-ref`
  (root graph — `/build/native-clang` is the `/src/native` binding
  graph and carries no reference tests) — **Passed**; binary PASS 2754.
  `/build/native-clang` ctest run: `migration_T51` Not Run,
  `migration_T03` Failed — both pre-existing baseline entries,
  unrelated to this change (see gate classification).
- Full gate: `bash tools/run_tests.sh full` (darwin-arm64, completed to
  its tail, log `output/full_gate_f5.log`) — **exit 8** at the
  binding-graph CTest, identical to the documented baseline gate
  (F1-era/F4 records: exit 8, CTest 60/62, reds {T09, T10}).
  - Extension configure+build at `native/build`: green (MuJoCo 3.12.0
    pin); produced orchestrator `build/native-release/bin/bike-native-checks`.
  - All nine analysis stages green: `check_frontends` g++-16 (333
    sources/334 TUs, findings none), `check_tidy` clang-tidy (334 TUs,
    findings none + 1 documented `clang-analyzer-cplusplus.Move`
    suppression), `check_analyzer` clang++ (report-only alpha,
    pre-existing patterns), `check_cppcheck` none, `check_odr`
    g++-16 -flto -Wodr none, `check_scripts` shellcheck clean;
    produced kinds `headers` (260 headers / 9012 header TUs, findings
    none), `diagnostic-controls` 6/6, `context` all SDK controls
    passed; runtime resolution sanitizer=none.
  - `ctest --test-dir native/build`: **60/62** (961.82 s) — reds
    `migration_T09` (`FAIL post-advance integration vector matches the
    oracle`) + `migration_T10` (`FAIL first transition matches the
    Python oracle`), verbatim the documented frozen-oracle baseline
    pair. Zero unexpected reds; pytest phase never reached (gate
    aborts at the first red stage, same as the recorded baseline).
  - Contracts checker run standalone per the documented procedure
    (after a provenance-seeding `load_native()` import):
    `bike-native-checks --kind contracts` → **12/12 assertions passed**
    incl. `pure_core_python_free`, `hot_kernel_dependency_free`,
    `ctest_contract_cases`, `loader_provenance`.
- Post-edit spindle re-runs (after `e85a9de5` comment edit + `9df2575e`
  ledger edit): darwin `build/native-release` rebuilt +
  `ctest -R 'spindle'` → `reference.native_spindle_controller` **Passed**
  (binary **PASS 2754 checks**); `pytest
  tests/reference/test_native_spindle_controller.py` under
  `native_runtime` → **47 passed**. Linux-aarch64
  `bike-dev-pfc-lane-e`, `/build/root-ref` (root graph — the
  `native-clang` binding graph carries no reference tests): sources
  synced (test TU md5-verified), rebuilt +
  `ctest -R 'spindle'` → **Passed**, binary **PASS 2754 checks** —
  identical count to darwin.

## Migration ledger

`migration/test-port-map.json`: **23 records** for the spindle family
(the 22 planned source symbols + the new rear-chain record), each with
collected node ids, `native_test_id`
`reference.native_spindle_controller::<symbol>` and an evidence line.
17 golden records carry `comparison_class=tolerance-oracle` +
`golden_platforms=[darwin-arm64, linux-aarch64]`; the six reject-only
records carry `comparison_class=contract` + `disposition=port` (no
captured artifacts). Commits `13c021f1`, fix-round `9df2575e`.
No other records touched.

## Commits

- `7f70dfab` fix(rider): solved_effort rejects malformed state widths
- `3336515f` test(reference): spindle golden capture + pinned fixture
- `41e47e44` feat(native-tests): reference.native_spindle_controller
- `acf9b960` test(reference): spindle golden tolerance bound + linux goldens
- `13c021f1` ledger(migration): spindle controller family → golden
- `9df2575e` ledger(migration): spindle fix round — comparison classes + record placement
- `e85a9de5` test(reference): spindle stale comment repair

## Fix round (post-review)

Review-verified findings and resolutions:

1. **`close-oracle` → `tolerance-oracle` (23 records + this report).**
   `tolerance-oracle` is the sanctioned class (4 existing precedent
   records). All 23 spindle records re-classed; the ledger section
   above now names the real classes. Commit `9df2575e`.
2. **Six reject-only records mis-dispositioned `golden`** with
   `golden_platforms` but zero captured artifacts — set
   `disposition=port` + `comparison_class=contract`, dropped
   `golden_platforms` (precedent: `test_native_atomicity` /
   `test_alpine_track` reject tests):
   `test_spindle_waveform_rejects_bad_inputs`,
   `test_effort_ceiling_rejects_bad_inputs`,
   `test_spindle_activation_rejects_repeated_timestamp`,
   `test_spindle_strength_rejects_malformed_state`,
   `test_spindle_lookups_reject_unknown_joints`,
   `test_spindle_probe_rejects_bad_input`. Commit `9df2575e`.
3. **Stale comments in `test_native_spindle_controller.cpp`** — the
   file header claimed f64 arrays bitwise / state documents
   byte-exact; the tolerance block claimed `rtol=1e-8/atol=1e-9` f64
   and a state bound "one decade looser". Actual: every f64 record is
   `close` at the node's recorded `rtol=1e-7/atol=1e-8`; state
   documents go through the `json_close` deep comparison at the same
   bound. Also repaired the compute-family sentence `acf9b960` split
   across the inserted block. Comment-only; rebuilt +
   `ctest -R 'spindle'` re-run: PASS 2754 checks. Commit `e85a9de5`.
4. **Ledger bookkeeping** — `static_test_functions` 2390→2391,
   `top_level_test_functions` 2374→2375 (the +1 rear-chain record is
   top-level); rear-chain `end_line` 697→696 (source ends at 696);
   record moved from the array tail into its 22-record same-file
   group in source-line order. Commit `9df2575e`.
5. **`bash tools/run_tests.sh full` completed to its tail** — see the
   verification section for counts and the baseline classification
   below.
6. **Ledger serializer canonical** — edits made through
   `json.dump(indent=2, ensure_ascii=True)` + trailing LF;
   `git diff --stat` shows only the flagged fields (52+/76−, all in
   the 23-record family + header counts + the moved record).

## Baseline-red classification

`bash tools/run_tests.sh full` completed to its tail on darwin-arm64
(log `output/full_gate_f5.log`): **exit 8**, the documented
gate-abort shape — the profile stops at the first red stage, so the
pytest phase was never reached (same as every recorded baseline gate
since the F1 fix round).

Observed reds, all inside `ctest --test-dir native/build` (**60/62**,
961.82 s):

| CTest node | Signature | Classification |
|---|---|---|
| `migration_T09` | `FAIL post-advance integration vector matches the oracle` | documented frozen-oracle baseline pair member — immutable T01-era capture vs progressed native physics; verbatim signature (`migration/cpp-linux-completion-evidence.md` F1 fix-round + F4 gate records: exit 8, reds {T09, T10}) |
| `migration_T10` | `FAIL first transition matches the Python oracle` | same documented pair — T01 `transitions.json` frozen-oracle drift; entered the documented set at the F1 fix round (T29 exited then) |

Zero reds outside that pair. `migration_T52`–`T56` are not registered
in the `native/build` binding graph (the standing 51-vs-53 split —
T56's shellcheck env-drift family lives in the root graph and was not
exercised here). The linux-only `migration_T` baseline set
{T03, T04, T09, T10, T14, T15, T16, T18, T20, T31, T32} is a
linux-aarch64 baseline — not applicable to this darwin gate; none of
its members appeared. The `-L reference` lane reds
{`golden_episode_tool`, `native_numeric_completion`,
`pneumatic_tire_oracle`, `native_pneumatic_tire`} and the pytest-side
baseline residual live past the ctest abort and were not exercised by
this gate — `reference.native_spindle_controller` was verified
directly on both platforms instead (PASS 2754 checks, both graphs).

Nothing attributable to the F5 diff: the only failures are the two
frozen-oracle nodes whose signatures match the documented baseline
verbatim, and no spindle-family test appears in any red set on either
platform. The exit code was verified empirically — a scoped rerun of
the same two nodes through `native_runtime`'s ctest wrapper returns
rc=8, the same abort status the baseline gate records.
