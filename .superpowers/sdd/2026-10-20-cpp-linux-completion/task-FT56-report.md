# Task FT-5/FT-6/FT-2 report — R7 pneumatic reds + R4 adapter-seam disposition

**Lane**: `.worktrees/pfc-lane-d`, branch `lane/d2`, base `3901bdb4`
(integration tip; worktree had no build tree — configured `native/build`
fresh, AppleClang 21.0.0.21000334, MuJoCo 3.12.0). Linux leg:
`bike-dev-pfc-lane-d` (`bike-dev-linux:24.04`, `/build/native`,
CPython 3.13.16, distro clang).

Investigation scope: honest-red families R7 (`pneumatic_tire` ×2) and the
`pinned_savage_40`/`golden_episode_tool` precedent (FT-5), R7
`probe_and_clock` bisect (FT-6), and the R4 adapter-seam contract
disposition (FT-2). No physics, threshold, or oracle changes; no
recapture. Repro scripts: `/tmp/ft5-repro/repro_event.py`,
`/tmp/ft5-repro/repro_savage.py`.

## Headline result

**Both R7 records are green on both platforms at lane/d2 — neither needs
a new fix.** The two recorded failure signatures were measured on lanes
that predated fixes already merged into the inventory base `d20ea74c`:

| Node | darwin-arm64 | linux-aarch64 | Verdict |
|---|---|---|---|
| `test_pneumatic_golden_episode_event_rich` | PASS 13.98 s (cold equilibrium cache) | PASS 14.80 s (cold cache) | FT-5: converges — recorded red was the F2 brake-hold defect, fixed by `051497c` |
| `test_pneumatic_probe_and_clock` | PASS 62.76 s | PASS 71.11 s | FT-6: fixed by FT-1 `466b4996` (`coast_hold_nm` bootstrap) — revert reproduces the exact recorded failure |

## FT-5 surface 1 — `event_rich` equilibrium non-convergence

**Recorded** (3.12b lane/a evidence): `RuntimeError: physical equilibrium
did not converge after 64000 steps: residual=0.0814217;
front_wheel_spin`.

**Determination: the committed config converges — the recorded red was
the F2 brake-hold settle defect, already fixed.** Evidence:

- The event env derives from `viewer_physics_distributed.toml`, which
  declares `initial_front_brake = 1.0` / `initial_rear_brake = 1.0` — its
  held-relaxation settle exercises `StaticBrakeApplier.hold_qfrc`, i.e.
  the F2 surface.
- The F2 fix `051497c` ("re-anchor a saturated brake-hold bite during
  settle relaxation") is an ancestor of both `d20ea74c` (inventory base)
  and `3901bdb4` (lane/d2 tip). The red record was measured on the 3.12b
  lane/a merge `951f0650`, which **does not** contain `a27a8d6` (the F2
  merge-back); the inventory propagated the stale signature.
- Repro `/tmp/ft5-repro/repro_event.py` at `3901bdb4`, cold cache
  (`BIKE_SIM_EQUILIBRIUM_CACHE_DIR` fresh): **SETTLE OK in 7,152 steps,
  residual_qacc=0.0098857, exactly 1 `hold_qfrc` reseat event**.
- Control (`--no-reseat`, wrapper forces `reseat=False`): **SETTLE
  FAILED: residual=0.0814217 dominated by `front_wheel_spin`
  (0.08142174361884008)** — bitwise-identical to the recorded failure.

The reseat is the operative difference; no config/initial-state change
is needed and no physics constants were touched. This is the F2-style
settle-fix precedent already landed — not a `pinned_savage_40`-class
documented-baseline disposition.

## FT-5 surface 2 — `pinned_savage_40` committed-config claim

**Recorded** (3.14b lane/c evidence, `cpp-linux-completion-evidence.md`
L4091-4099): "a direct `freeze.py` run against the committed
`episode/physics.toml` does not converge (`physical equilibrium did not
converge after 64000 steps`) — the committed fixture config is a
genuine, pre-existing gap; the pinned-config capture is what the native
test's oracle path reproduces."

**Verification at lane/d2: the claim no longer holds.**

- `/tmp/ft5-repro/repro_savage.py` (mirrors `freeze.py`'s env creation:
  committed `physics.toml` + `rough_uphill_savage.toml`, `--seed 0`,
  cold cache): **converges in 4,816 steps, residual=0.0098393**
  (darwin-arm64); **3,008 steps, residual=0.0097587** (linux-aarch64).
  Step counts differ per platform (numba/LLVM codegen), both well under
  budget.
- Capture-config variant (committed toml + `activation_tau_s = 0.025`
  under `[articulated]`): **4,720 steps, residual=0.009105692229970314**
  — bitwise-identical to the F2 fix evidence's savage settle
  ("steps=4720, residual_qacc=0.009105692229970314",
  cpp-linux-completion-evidence.md L2798).
- Corroborating: `reference.golden_episode_regressions` builds envs via
  `episode/pinned_savage_40/request.toml` → committed `physics.toml`
  with `use_equilibrium_cache = false` (a real settle per `fresh()`) —
  green on both platforms since the F2 fix (its recorded PASS was part
  of the F2 verification, L2803-2805).

**Canonical capture-config argument (documented):** the
`pinned_savage_40` artifact bytes are bound to the pytest capture path —
`test_pinned_topology._pinned_config` (welded toml carrying
`activation_tau_s = 0.025` in `[articulated]`) run under
`_pin_oracle_kernels` (njit cores rebound to `*_py` twins). The
committed `episode/pinned_savage_40/physics.toml` predates
`activation_tau_s`; it is a *different config*, not a corruption of it.
At capture time its settle genuinely did not converge (the F2 defect);
post-`051497c` it converges, but recapturing under it would still
produce different bytes (activation lag changes the torque trajectory;
settle 4,816 vs 4,720 steps) — the artifact remains bound to its capture
config, and the stale "pre-existing gap" phrasing should be read with
the measured-update above.

## FT-6 — `probe_and_clock` step-1 bisect

**Recorded**: `AssertionError: interval 1: bitwise mismatch`.

**Root cause: the FT-1 staged-input family — not a `step.cpp` tire
`probe_compute`/`compute` defect.** The `interval 1` assert fires inside
the parallel `advance()` parity loop
(`native.snapshot().integration_state` vs `_integration_state`), before
`probe_step_inputs` is ever reached — so the divergence is in the
native-window step path, not the probe fold.

- `create_native_ride` (`native/runtime.py:218`) bootstraps via
  `capture_bootstrap`; pre-`466b4996` it serialized the rider controller
  through `state_dict()` only, dropping the mutable `_coast_hold_nm`
  weld-press servo hold. The native controller then replayed a
  different held torque → different `qfrc_actuator`/`qacc` →
  bitwise-divergent integration state at interval 1. Same family as the
  FT-1 step-114 `ctrl[3]` divergence, manifesting earlier on this plant.
- **Bisect proof**: reverting `466b4996` in `src/bike_sim/native/setup.py`
  (drop the optional 19th `coast_hold_nm` key → 18-key `state_dict()`)
  reproduces the recorded failure verbatim — `AssertionError: interval
  1: bitwise mismatch` — on an otherwise-green lane/d2 tree. Tree
  restored; no committed change.
- The sibling FT-1 fix `4b99733b` (spindle `write()` per-step
  `directional_limit` clip + speed gate) is in the same staged-input
  family; the bootstrap revert alone suffices to reproduce, so the
  operative cure here is `466b4996`.

No `step.cpp` probe-staging defect exists for this node — the suggested
`probe_compute` vs `compute` bisect at ~:1291-92 was a wrong hypothesis;
the probe fold itself was never reached by the recorded failure.

## FT-2 — R4 adapter-seam disposition

| Record | Disposition | Rationale |
|---|---|---|
| `test_publication_failure_retry_does_not_advance_twice` (idx 895) | **drop** | Committed-step contract covered by the ported twins `reference.native_research_environment::test_zero_budget_retains_single_window_and_no_tail_flush` (pending window holds staged command across polls; exactly-once commit; one transition) + `test_invalid_boundary_calls_do_not_change_state` (rejected calls keep pending, state untouched). The injected `adapter._transition` failure is a Python-only seam deleted at T63 — `NativeResearchRuntime` has no publication hook. Native staged-completion design mirrors the invariant (`PolicySession::advance_pending` retains `s.completed`; `begin_advance` never re-derives the staged command on retry, policy_session.cpp:190-243). Python node green darwin 21.7 s — guards the adapter until removal. |
| `test_custom_rider_callback_runs_once_per_external_boundary` (idx 899) | stays **planned** — reported | Committed-step half covered by the zero-budget twin, but the once-per-external-boundary callback contract has **no native test leg**: `research::PolicySession` calls `rider_behavior->act(observation.time_s, ...)` once per `begin_advance` inside the staged-command block (policy_session.cpp:202), yet no native test registers a `RiderBehavior`. A future counting-`RiderBehavior` leg through `begin_advance` + zero-budget polls + `reset` would complete the coverage needed to drop. |

## Ledger updates (`migration/test-port-map.json`)

Canonical serializer (`json.dumps(indent=2)` + ASCII + LF) preserved;
only the four assigned records touched:

- idx 811 `test_pneumatic_golden_episode_event_rich`: `line`/`end_line`
  re-synced 372/428 → **558/619** (the stale-field note in
  honest-red-inventory-2026-10-31.md); FT-5 evidence appended;
  disposition stays `planned` (T52 capture + parked `[[maybe_unused]]`
  C++ registration remain the port gate).
- idx 816 `test_pneumatic_probe_and_clock`: `line`/`end_line` re-synced
  832/909 → **1052/1137**; FT-6 evidence appended; stays `planned`
  (same T52 gate).
- idx 895 `test_publication_failure_retry_does_not_advance_twice`:
  `planned` → **`drop`** (`native_test_id`/`native_source` → `null`,
  `comparison_class` → `python-only`), FT-2 coverage evidence appended.
- idx 899 `test_custom_rider_callback_runs_once_per_external_boundary`:
  stays `planned`; FT-2 coverage-gap evidence appended.

## Verification record

| Command | Exit | Result |
|---|---|---|
| darwin: `pytest test_native_pneumatic_tire.py::test_pneumatic_probe_and_clock` | 0 | 1 passed, 62.76 s |
| darwin: `pytest test_native_pneumatic_tire.py::test_pneumatic_golden_episode_event_rich` (cold cache) | 0 | 1 passed, 13.98 s |
| darwin: `pytest test_native_research_environment.py::test_publication_failure_retry_does_not_advance_twice ::test_custom_rider_callback_runs_once_per_external_boundary` | 0 | 2 passed, 21.70 s |
| darwin: `pytest test_native_research_environment.py::test_native_research_matches_every_external_transition` (parity-leg control) | 0 | 1 passed, 3.36 s |
| linux: `pytest ...::test_pneumatic_probe_and_clock` | 0 | 1 passed, 71.11 s |
| linux: `pytest ...::test_pneumatic_golden_episode_event_rich` (cold cache) | 0 | 1 passed, 14.80 s |
| repro: event settle, reseat on (cold) | 0 | converged 7,152 steps, residual 0.0098857, 1 reseat |
| repro: event settle, reseat off (cold) | 1 | `did not converge after 64000 steps: residual=0.0814217; front_wheel_spin` — recorded failure bitwise |
| repro: committed pinned_savage_40 physics.toml (cold) | 0 | converged 4,816 steps (darwin) / 3,008 steps (linux) |
| repro: capture config + `activation_tau_s=0.025` | 0 | converged 4,720 steps, residual 0.009105692229970314 = F2 evidence bitwise |
| bisect: revert `466b4996`, rerun probe_and_clock | 1 | `interval 1: bitwise mismatch` — recorded failure reproduced |

Profile selection per AGENTS.md: ledger/documentation-only change — no
code was modified, so no gate profile applies (the cheapest covering
profile `quick` excludes these `slow` nodes anyway; the targeted oracle
nodes were run directly on both platforms as recorded above).

## Concerns / follow-ups

- **Stale-record propagation**: both R7 reds were measured on lanes
  predating fixes already in the inventory base. The inventory's R7
  signatures were accurate for their measurement commits but the "still
  red at `d20ea74c`" implication was unverified. Both records now carry
  the measured-green evidence at `3901bdb4`.
- **T52 gate unchanged**: the two pneumatic records stay `planned` —
  their C++ twins remain parked `[[maybe_unused]]` pending per-platform
  golden capture (the oracle blocker is now provably cleared on both
  platforms, so capture can proceed).
- **`pinned_savage_40` note staleness**: the 3.14b "committed config
  does not converge" remark no longer holds post-F2; the capture-config
  argument is documented above. No artifact change — recapture under
  the committed config would still differ from the pinned capture path.
- **FT-2 half-drop**: only idx 895 earned `drop`; idx 899's
  callback-discipline contract has a native mechanism but zero test
  coverage — reported, stays `planned`.
