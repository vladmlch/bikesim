# Task C325 — test-port 3.25 report (runtime step, commands, ownership, bootstrap)

**Batch**: 3.25 — `tests/reference/test_native_runtime_{step,commands,
ownership,bootstrap}.py` (49 records / 52 collected nodes; the
invalid-target ownership test is parametrized ×4). CTest ids
`reference.native_runtime_{bootstrap,step,commands,ownership}`;
labels `t52 runtime`; port-map rows carry `owner_task: T53`.

**Lane**: `.worktrees/pfc-lane-d`, branch `lane/d`, base `fc40dbce`
(post-F4 merge-back). No merge, no push.

## Status: landed — 38 `port`, 10 `planned` (pre-existing oracle reds), 1 `drop`

38 of 49 records ported to four new CTest executables; every ported
check compares native state against native state or literal contract
text — no golden fixtures were needed or created. The ten live-oracle
parity nodes that remain red in the source suite were deliberately
**not** reproduced (porting them past as green would falsify a live
gate); they stay `planned` in the ledger with per-node evidence. One
record is `drop` (`python-only` adapter seam).

## Triage

| Source file | records | port | planned (red) | drop |
|---|---|---|---|---|
| `test_native_runtime_bootstrap.py` | 19 | 19 | 0 | 0 |
| `test_native_runtime_step.py` | 10 | 4 | 6 | 0 |
| `test_native_runtime_commands.py` | 10 | 6 | 4 | 0 |
| `test_native_runtime_ownership.py` | 10 | 9 | 0 | 1 |
| **total** | **49** | **38** | **10** | **1** |

## What the ports reproduce

- **`reference.native_runtime_bootstrap`** (19 checks): construction
  ownership, captured-boundary restore, snapshot counters, survival of
  caller-document destruction, double `reset()` replay, `close()`
  fast-fail on every public surface, caller-held document isolation,
  and all 13 rejection surfaces (missing/unknown state key, digest,
  `model_dims` nq/nv/na, integration width, non-finite lane, malformed
  drive state — pinned to the T09-documented unexpected-acceptance
  reason, unknown joint in mutable state, unknown runtime key, schema
  version, runtime-config envelope, control-period inconsistency,
  missing `model.mjb` via a staged copy).
- **`reference.native_runtime_step`** (4 checks): chunk-independence
  (single `advance(40)` vs `(1,3,9,17,40)` → bitwise identical
  integration vector), `reset()` replay vs a fresh runtime bitwise,
  snapshot owned-copy isolation, `probe_step_inputs` staged
  diagnostics with `step == 0` non-advance invariant.
- **`reference.native_runtime_commands`** (6 checks): budget yield
  between steps (`reason == 'budget'`), current-step no-op, invalid
  target/command rejection naming `target_step`/demand fields before
  native work, `AdvancementGuard` concurrent-operation rejection,
  `closed` fast-fail on every surface, fatal engine boundary
  preserving the committed prefix + poisoned-until-reset.
- **`reference.native_runtime_ownership`** (9 checks): owned/read-only
  batch rows and columns with fresh `as_wire()` exports; dropped
  unacknowledged batch keeps the pending prefix retryable;
  prefix-append / stale / cross-generation / foreign-owner
  acknowledgement rules; samples, columns and snapshots surviving
  `reset()` + `close()`; unmaterialized-batch liveness after close;
  caller-side capture mutation cannot forge accounting (`history.
  work_j` forgery verified absent) or the reset replay; invalid
  target before native call; fatal-prefix preservation + recovery.

## Individual dispositions — the six known pre-existing red nodes

All six assert bitwise equality of `NativeRideRuntime`'s committed
integration vector against the **live Python oracle**. All fail
pre-existing on both platforms with the same 1-ulp first-difference
family (lane 3: `5.66934496e-03` actual vs `5.66934495e-03` expected
on darwin-arm64; F4's report recorded `5.66934492e-03`/`5.66934491e-03`
— the F4 weld fix moved the pre-divergence bytes, not the divergence).
A/B-verified identical pre- and post-F4 (`task-F4-report.md`). A
native↔oracle runtime divergence requiring its own fix task — the
nodes remain live oracle gates in the Python suite; each stays
`planned` with NOT-PORTED evidence in `migration/test-port-map.json`.

| Node | What it gates | Disposition |
|---|---|---|
| `test_native_first_step_uses_settled_material_state` | first `advance(1)` integration vector bitwise vs oracle | `planned` — red at interval 1 already; divergence is upstream of any command path |
| `test_native_step_sequence_matches_python_state` | all 30 committed intervals bitwise vs oracle | `planned` — same first-difference signature, propagates from step 1 |
| `test_native_force_fold_matches_python_recorded_inputs` | `probe_step_inputs` staged force fold vs oracle `apply_forces(advance=False)` | `planned` — red inside the same staged-force path as the step-1 drift |
| `test_native_advance_tracks_python_under_load` | 30 intervals under brake apply/release, bitwise | `planned` — same propagating 1-ulp family under load |
| `test_sensor_channels_read_sensordata_rows` | probe `sensors` rows bitwise vs `sensordata` (adr-vs-id indexing) | `planned` — red via the staged-forward sensordata lanes |
| `test_native_intent_lanes_match_python_past_reaction_delay` | 160-step horizon + intent signals/inclination vs oracle | `planned` — red at the integration-vector compare before the intent reads |

The four `test_native_runtime_commands.py` reds
(`test_explicit_zero_effort_differs_from_automatic`,
`test_motor_ceiling_change_matches_python`,
`test_brake_apply_release_matches_python`,
`test_gear_shift_and_coast_resume_match_python`) share the identical
disposition and evidence — all assert per-interval bitwise equality
against the oracle under scripted command programs and fail with the
same propagating signature on both platforms.

## Drop

`test_row_materialization_is_lazy_and_retryable` (ownership) → `drop`,
`python-only`. The source injects a `MemoryError` into the Python
`SampleBatch` adapter's lazy row cache and asserts the retry. The
native boundary publishes owned immutable `SamplePtr`s and
rematerializes rows per `as_wire()` call — there is no lazy cache to
port. The underlying liveness guarantee is covered by
`reference.native_runtime_ownership::test_unmaterialized_batch_keeps_
native_rows_alive_after_close`.

## Two-platform qualification matrix

| Surface | darwin-arm64 | linux-aarch64 |
|---|---|---|
| `reference.native_runtime_bootstrap` | PASS (~1.4 s, `build/native-release`) | PASS (1.39 s, container `/build/root-clang`, clang++-18) |
| `reference.native_runtime_step` | PASS (0.45 s) | PASS (0.51 s) |
| `reference.native_runtime_commands` | PASS (6.29 s) | PASS (9.64 s) |
| `reference.native_runtime_ownership` | PASS (0.72 s) | PASS (1.00 s) |
| `ctest -R 'reference\.native_runtime_'` | **4/4** | **4/4** |
| source `step.py`+`commands.py` | 10 failed / 10 passed — exactly the 10 recorded reds | 10 failed / 42 passed across all four files — identical red set + signature |
| source `ownership.py`+`bootstrap.py` | green (32 passed) | green (in the 42) |

Golden capture: none — every ported check is native-vs-native; there
is no oracle-derived comparison to freeze, so no byte-flip proof is
applicable. The welded bootstrap fixture is the pre-existing
`tests/fixtures/migration/bootstrap/welded` capture (unchanged).

## Commits

| SHA | Unit |
|---|---|
| `14078ed4` | test(reference): port runtime step/commands/ownership/bootstrap contract suite (C3.25) — 4 CTest sources + shared `runtime_reference_support.hpp` + `Phase3Ports.cmake` registration (single block, `LABELS t52 runtime`, deduped) |
| `f68d1317` | docs(migration): C3.25 runtime batch ledger — `test-port-map.json` (38 port / 10 planned-red / 1 drop, measured `collected_node_ids`, dated `assertions_capture`, per-record evidence) + `python-removal.json` (bootstrap, ownership → `replacement_landed`) |
| (this commit) | docs(migration): C3.25 evidence section + this report |

## Deviations

1. **10 live-oracle parity nodes not reproduced** — documented above;
   they stay `planned` as live gates, not `drop`, because the gate is
   real and the divergence is unfixed.
2. **`test_row_materialization_is_lazy_and_retryable` dropped** —
   Python-adapter lazy-row seam; no native counterpart exists.
3. **Mutability semantics adapted**: the source proves held outputs
   are read-only via `ValueError`/`TypeError` on write. Native
   `RuntimeSnapshot`/`RuntimeSampleBatch` are caller-owned *mutable*
   copies — the port asserts the equivalent liveness/isolation (runtime
   unaffected by held-copy mutation; bytes intact across close) and
   restores the probe byte before the post-close bitwise check. The
   `view` mutation probe targets `drive_mode` because the native view
   is the nested `{drive_mode, endpoint, preview_row, model_fields}`
   shape, not the flat sample row.
4. **Acknowledgement marker**: re-acking a consumed batch trips the
   size check first in the native implementation, producing "belongs
   to a different generation or prefix"; the port asserts marker
   `prefix`, inside the source's `'prefix|stale'` alternation.
5. **Type-freedom parameters**: the source parametrizes invalid
   targets `[True, 1.5, '3', -1]`; only `-1` is a typed native input
   (`std::int64_t` signature) — noted in-file and in the port map.
6. **`weld_servo_gain` staleness (linux)**: the container's
   `/build/native-clang` artifact predated the synced source tree and
   was rebuilt before the source-suite run — an artifact-identity
   gate doing its job, not a port defect.
7. **`fs::copy` parent directories**: the staged missing-model test
   creates `<tmp>/bootstrap/` explicitly — `fs::copy` only creates the
   leaf.

## Final gate

`bash tools/run_tests.sh native` (`native/build`, AppleClang,
darwin-arm64) — **does not exit 0; the outcome is baseline-blocked,
fully mapped, zero unexpected reds.**

- Run 1 (00:37 UTC): build + frontends/tidy/analyzer/cppcheck/odr/
  scripts all green (analyzer report-only, nonblocking), then
  `check_headers` failed **environmentally**: generated
  `header_probe.cpp` files vanished mid-sweep — first failure record
  index 957, all 123 failures were missing-probe-file, **zero actual
  compiler diagnostics**; the shared `native_check_work/headers/`
  tree was emptied underneath the run.
- `check_headers` standalone rerun (`bike-native-checks --kind
  headers --build native/build`, run `20261027T060639486867Z`):
  **passed, 9074/9074 TUs rc=0** — the failure is not reproducible
  and not source-attributable.
- Remaining gate phases run as a resumed sequence (identical
  commands to `preflight_native`):
  - `diagnostic-controls`: 6 controls, 0 failed, health ok.
  - `context`: 5 SDK controls, all passed, health ok.
  - `ctest --test-dir native/build`: **60/62** — the only reds are
    the documented baseline pair `migration_T09` (`post-advance
    integration vector matches the oracle`) + `migration_T10`
    (`first transition matches the Python oracle`), the same
    frozen-oracle pair and the same 60/62 the previous lane gate
    recorded (`cpp-linux-completion-evidence.md`, post-F4 entry).
    Zero unexpected reds; the profile stops at this stage per
    documented gate-abort behavior.
  - `contracts`: all assertions passed (`pure_core_python_free`,
    `hot_kernel_dependency_free`, `ctest_contract_cases`, …).
  - pytest tail (`test_native_*.py` + `test_golden_episode*.py`,
    `-p native_test_reporter`): **3197 passed / 43 failed / 1
    skipped** in 2h01m. Every one of the 43 maps to a ledger record:
    39 nodes on `planned` records (this batch's 10 plus 29 others —
    pneumatic tire, research capture/environment/recording, ride
    frontend, runtime rollout, spindle controller, viewer
    snapshot), 3 nodes on `drop` records (distributed tire ×2,
    numeric completion shifting-metadata) with native-side
    coverage, and 1 node on a `golden` record
    (`test_tire_channel_parity_across_backends`) whose
    authoritative golden CTest is green — all the same
    native↔oracle bitwise divergence family. Inside the tail,
    `test_standalone_header_sweep_compiles_first_party_headers`
    re-ran the serial legacy sweep: run `20261027T074403229844Z`,
    **passed** (this is the ~90–120 min in-suite sweep the test's
    own comment calibrates).
- The four C3.25 CTests live in the standalone `build/native-release`
  graph (`ctest -R 'reference\.native_runtime_'` = 4/4, recorded in
  the matrix above); the binding-graph `native/build` ctest set does
  not include `reference.*` nodes.
- Verdict: single-command exit-0 is unreachable while the
  documented T09/T10 baseline pair remains red (identical to the
  prior lane gate's exit-8-at-ctest). No C3.25-introduced failure
  exists in any phase; the environmental `check_headers` flake is
  documented and did not reproduce on rerun.

## Concerns / notes for merge-back

1. The 10 `planned` records are the gate: any merge-back consumer must
   not read `planned` as "done". The divergence fix is a separate
   runtime task; once it lands these nodes become `golden`-class ports
   per ADR-0003.
2. `python-removal.json` keeps `test_native_runtime_{step,commands}.
   py` at `planned` — the files still carry live oracle gates; only
   `bootstrap`/`ownership` moved to `replacement_landed`.
3. `runtime_reference_support.hpp` carries a `-Wframe-larger-than`
   waiver on the by-value `ride()`/`compose_runtime()` helpers — the
   same precedent as `test_t09.cpp`; both platforms compile clean.
