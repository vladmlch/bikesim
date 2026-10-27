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
- Full gate: `bash tools/run_tests.sh full` — launched on this branch;
  build + all static-analysis sweeps enumerated clean through the
  clang-tidy sweep start (frontends: none); CTest/pytest tail was still
  running at handoff — complete it and fill in the classification below
  before closing the task.

## Migration ledger

`migration/test-port-map.json`: **23 records** promoted to `golden`
(the 22 planned source symbols + the new rear-chain record), each with
collected node ids, `native_test_id`
`reference.native_spindle_controller::<symbol>`,
`comparison_class=close-oracle`, evidence line, and
`golden_platforms=[darwin-arm64, linux-aarch64]`. Commit `13c021f1`.
No other records touched.

## Commits

- `7f70dfab` fix(rider): solved_effort rejects malformed state widths
- `3336515f` test(reference): spindle golden capture + pinned fixture
- `41e47e44` feat(native-tests): reference.native_spindle_controller
- `acf9b960` test(reference): spindle golden tolerance bound + linux goldens
- `13c021f1` ledger(migration): spindle controller family → golden

## Baseline-red classification

Full-gate tail pending at handoff. Already classified from the linux
runs: `migration_T51` (Not Run) and `migration_T03` (Failed) in
`/build/native-clang` ctest are pre-existing baseline entries unrelated
to the spindle change — compare the completed gate against
`migration/baseline-full.log` / `baseline-full-enospc.log` /
`baseline.md`; nothing observed so far classifies as spindle physics.
