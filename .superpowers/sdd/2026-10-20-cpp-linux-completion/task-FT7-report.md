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
| `bash tools/run_tests.sh native` | **see below** |

## Commits

- `golden(ft7): recapture welded/research oracles + fix solved_effort joint budget`
  (this change set; `Co-Authored-By: Devin`).

## Darwin gate exit-0 reachability

- Focused gate state: T09 + T10 green on both graphs; all known
  welded/delayed_bootstrap consumers green; the defect fix's blast
  radius (solved_effort fold) verified against the spindle/rider
  golden tests — unchanged scenarios stay bitwise-valid.
- `bash tools/run_tests.sh native` — RESULT: *pending final gate run
  (recorded below at commit time)*.
- Linux-aarch64 legs of the recaptured fixtures + golden pair remain
  deferred with the linux wave — same convention as FT-3 (`3b32934f`);
  the linux-side T09/T10 stays in its documented-red set.
