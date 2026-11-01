# Task FT-7 report — T09/T10 frozen-oracle triage

Lane: `lane/d4` · worktree `.worktrees/pfc-lane-d` · Darwin arm64 only
(docker guard honored) · base `907687f1`.

## Root-cause class per test

| Test | Class | Evidence |
|---|---|---|
| `migration_T09` ("post-advance integration vector matches the oracle") | **(a) stale frozen fixture — verified recapture** | Live-oracle parity green before recapture: native `advance(4)` on a fresh `capture_bootstrap` == live `advance_python` — 0/292 integration elements divergent at 1e-9; t=0 state bitwise-identical. Committed model.mjb `f8e48e9e…` vs fresh `fd757045…` — drift confined to fix-explained physics fields. |
| `migration_T10` ("first transition matches the Python oracle") | **(a) stale frozen fixture — verified recapture** + two oracle-stale test semantics | Live-oracle parity green before recapture: 3 transitions / 4 observations, 0 divergent fields at 1e-9 (step 24, t=0.03). The healthy plant also changed two incidental semantics the test encoded: `model_valid` false→true and zero pending reference rejections (was 1). |
| `reference.native_runtime_accounting` (surfaced by the recapture) | **(b) real native-vs-oracle defect — fixed** | `SpindleController::solved_effort` read `config_.joint_power_limit_w` (250 W capability cap) where the oracle's `joint_effort_limits` reads `config.delivered_joint_budget_w` (resolved: declared 375 W re-band else cap). Masked until now because the old capture declared `delivered_joint_power_budget_w = null` so both reads coincided at 250 W. |

## What was done

1. **Verified recapture** (`tools/migration/_ft7_recapture.py`, committed
   with this change) — re-ran the *same* recipes the T01/RCP-01 and
   `_t10_capture`/RCP-03 captures used, through the canonical
   `FixtureWriter`/`write_canonical` codec:
   - `tests/fixtures/migration/bootstrap/welded/` — model.mjb, config,
     state (+state.arrays), integration.f64, meta.json, steps_control
     pair, reset/{1,2} — reset digests stay byte-identical.
   - `tests/fixtures/migration/research/delayed_bootstrap/` — model.mjb,
     config, state (+arrays), research.json (+arrays), env.json,
     transitions.json.
   - `migration/oracle-manifest.json` — file records rebuilt from the
     writer's actual records; `inputs` re-pointed at this worktree's
     TOMLs (welded TOML `6385864c…`→`b2123f3…`);
     `delayed_bootstrap.expected.valid` corrected `false`→`true` with
     the honest diagnostic reason.
   - Structural payloads unchanged across the boundary
     (state.arrays a004–a014 identical) — only physics-dependent floats
     moved: no schema/construction-path breakage.
2. **Port defect fix** — `native/src/rider/spindle_controller.cpp`
   `solved_effort` now reads `config_.delivered_joint_budget_w()` at the
   joint-power-violation fold, mirroring `physical_config.py:508-512`
   and the accounting path (`accounting.cpp` already used the resolver).
3. **Oracle-stale test semantics retired with evidence** (documented,
   not suppressed):
   - `native/tests/migration/test_t10.cpp` — the pending-warning seam
     now asserts drain-empty + drain-at-most-once; positive drain
     coverage stays in `reference.native_runtime_accounting` (welded
     floor pin `weld_foot_min_normal_n=100`).
   - `native/tests/reference/recording_test_support.hpp` +
     `test_native_recording_retention.cpp` — the strict-flush check
     pins the same floor via `welded_runtime(floor_pin=true)`; the
     healthy fixture no longer trips the monitor inside 800 steps.
   - `tests/fixtures/golden/darwin-arm64/
     tests_reference_test_native_numeric_completion.py/
     test_tire_channel_parity_across_backends.{json,bin}` — golden pair
     re-recorded under `BIKE_GOLDEN_CAPTURE` (recorded docs pin
     `model_digest` to the new committed model image); the node's live
     pytest passed; the whole file passed except the ledger-documented
     `float32_shifting` stale pin (disposition `drop`).
4. **Ledger/evidence** — `migration/test-port-map.json` evidence
   appended for T09, T10, the tire-parity golden node, and the
   strict-flush retention node; FT-7 section added to
   `migration/cpp-linux-completion-evidence.md`.

## Negative controls

- Sign-bit flip in `steps_control_integration.f64` → `migration_T09`
  exit 8 (fails). Byte corruption in `delayed_bootstrap/state.arrays/
  a000.f64` → `migration_T10` exit 8. Fixtures restored byte-exact.
- The whole-file pytest run for the re-recorded golden: 531 passed,
  1 failed = `test_numpy_float32_shifting_metadata_retains_baseline_bits`
  — ledger-documented stale pin (disposition `drop`, `rear_teeth` 14 vs
  pinned 18), pre-existing, unrelated.

## Verification exit codes

| Command | Result |
|---|---|
| `ctest --test-dir native/build -R 'migration_T0[589]$\|migration_T1[02]$'` | **0** (5/5 pass; T09 15.9 s, T10 14.2 s) |
| `ctest --test-dir build/native-release -R '<T05,T08,T09,T10,T12 + numeric_domains + numeric_completion + monitor_strictness + runtime_bootstrap + recording_retention + runtime_accounting + batch>'` | **0** (11/11 pass after fixes) |
| `ctest --test-dir build/native-release -R 'native_spindle_controller + native_rider_allocation + native_rider_dynamics + native_rider_forces'` | **0** (4/4 pass — the solved_effort golden stays valid; the scenario's joint powers stay ≤250 W) |
| `BIKE_GOLDEN_CAPTURE=tests/fixtures/golden pytest …::test_tire_channel_parity_across_backends` | **0** (1 passed, 217 s — pair re-recorded) |
| `pytest tests/reference/test_native_numeric_completion.py` | 1 failed (`float32_shifting` stale pin — documented baseline) / 531 passed |
| `bash tools/run_tests.sh native` (final gate, post-`bc68f309`) | **1** — all 9 sweeps + CTest 62/62 green; pytest `3 failed / 3242 passed`; reds classified + repaired below (follow-up commit) |

## Final native gate (`bash tools/run_tests.sh native`, post-`bc68f309`)

Result: **exit 1**, ~3.2 h wall (heavy 3-worktree gate contention for the
first half). Every static-analysis sweep green or report-only;
**CTest 62/62 passed (800.66 s) including `migration_T09` 14.96 s and
`migration_T10` 13.25 s**; contract checks all `passed`; pytest
`3 failed, 3242 passed, 1 skipped in 5915.72 s`:

| pytest red | Classification | Evidence |
|---|---|---|
| `test_native_atomicity.py::test_valid_shift_commits_atomically[ideal_mid_drive]` | **stale literal pin** vs reviewed physics (pre-FT-7 red, introduced by `a0e35355` ft-3 sweep mirror — `git merge-base` shows it is an ancestor of `bc68f309` and absent from lane-g which passed the node) | native committed `ideal_hub.ratio = 1.3492063492063493` (mid-sweep); **live Python oracle produces the identical value bitwise** (`DrivetrainForceApplier.prepare_pedaling` on the same pair/spin); `34/24 + (34/28−34/24)·0.1/0.3 = 85/63`. The pinned `34./28.` literal predates the S6 cut-sweep semantics; the same stale literal also red-failed the ported twin `reference.native_atomicity` (ctest exit 8, `FAIL committed shift lands the 34/28 ratio atomically`) |
| `test_native_recording_retention.py::test_strict_flush_rejection_preserves_retained_evidence` | **stale rejection expectation** — relies on the plant's incidental violation which the re-scoped welded plant no longer trips inside 800 steps (`DID NOT RAISE InvalidReferenceRun`); same failure on lane-g's gate ⇒ pre-existing red independent of the FT-7 recapture | F1-class repair already established: `violating_physics()` pins `weld_foot_min_normal_n = 100.` for a deterministic `('foot_front.normal','foot_rear.normal')` rejection at t=0.00125 s |
| `test_native_numeric_completion.py::test_numpy_float32_shifting_metadata_retains_baseline_bits` | **documented baseline red — ledger-owned, untouched** | `test-port-map.json` entry: disposition `drop`, `python-only`; evidence row records `b21c63ec`'s multi-cog throw → `rear_teeth 14` vs pinned 18, "documented live-oracle red"; lane-g's gate fails it identically; left red by design (np.float32 internals no port mirrors) |

### Repairs applied (follow-up commit)

- `tests/reference/test_native_atomicity.py` + `native/tests/reference/
  test_native_atomicity.cpp` — the ratio pin now asserts the reviewed
  mid-sweep commit value `34/24 + (34/28−34/24)·0.1/(0.2+0.1)` for the
  ideal hub (geometric still pins the landed `34/28`), documented inline
  with the S6 sweep semantics + oracle-verified literal.
- `tests/reference/test_native_recording_retention.py` — the strict-flush
  test takes `tmp_path` and runs `violating_physics()` (the same floor
  pin lifecycle/platform/accounting tests use);
  `tests/reference/_native_runtime_support.py::make_python_ride` gains an
  optional `physics=` keyword (default `PHYSICS` — callers unchanged).

Targeted re-verification: `reference.native_atomicity` ctest **1/1 PASS
(0.49 s)**; pytest `test_native_atomicity.py` + `test_native_recording_
retention.py` **57 passed**; the two repaired nodes green.

## Commits

- `bc68f309 golden(ft7): recapture welded/research oracles + fix solved_effort joint budget`
- `<follow-up> fix(ft7): retire 2 stale pins exposed by the final gate` —
  mid-sweep ratio literal (both atomicity twins) + strict-flush floor pin
  (Python retention node); `Co-Authored-By: Devin`.

## Darwin gate exit-0 reachability

- Focused gate state: T09 + T10 green on both graphs; all known
  welded/delayed_bootstrap consumers green; the defect fix's blast
  radius (solved_effort fold) verified against the spindle/rider
  golden tests — unchanged scenarios stay bitwise-valid.
- Final gate `bash tools/run_tests.sh native`: **exit 1** with the
  pytest red set classified above. After the follow-up repairs the only
  remaining red is the ledger-documented `drop`/`python-only` node
  `test_numpy_float32_shifting_metadata_retains_baseline_bits` — a
  deliberate live-oracle red owned outside FT-7 (`b21c63ec` drift,
  per `test-port-map.json`). **Exit 0 is not reachable in this lane
  until that documented drop-record pin is retired by its owner lane**
  (or re-marked xfail by an explicit ledger decision — neither is in
  FT-7 scope). All in-scope reds are repaired with oracle-parity
  evidence; no unexplained drift was recaptured.
- Linux-aarch64 legs of the recaptured fixtures + golden pair remain
  deferred with the linux wave — same convention as FT-3 (`3b32934f`);
  the linux-side T09/T10 stays in its documented-red set.
