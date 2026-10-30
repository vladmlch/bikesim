# Task M5 — lane/b F2 brake-hold/settle re-anchor merge-back report

Worktree: `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/python-free-cpp`
Branch: `impl/python-free-cpp` (was `464d1b7`, now `a5ed745`)
Merged: `lane/b` tip `0fdabe2` (base `5be9ec5`; commits `051497c` physics
fix + `0fdabe2` test-double restore/evidence)

## Commits

- `a27a8d6` — `merge: lane/b F2 brake-hold/settle re-anchor (physics fix)`
  (`git merge --no-ff lane/b`, conflict resolution + auto-merges)
- `a5ed745` — `docs(migration): lane/b F2 merge-back evidence`
  (appended `## 2026-10-24 — F2 (lane/b) merge-back`; in-place
  `(12)`→`(13)` replay-class count correction)

Both carry `Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>`.

## Conflict resolution

Two conflicts:

- `tests/reference/test_rider_allocation.py` — kept the landed lane/c
  stub `hold_qfrc=lambda m,d,*a,**k:np.zeros(m.nv)` over lane/b's
  `lambda *a,**k: np.zeros(model.nv)` — semantically equivalent, reads
  `nv` off the arg (marginally more robust). File 42/42 green.
- `migration/cpp-linux-completion-evidence.md` — chronological
  resolution per precedent: the contiguous 3-section F2 block (led by
  the `2026-10-23` defect-fix section) sits between the last
  10-23-dated section (`Task 3.16` lane dev) and the first
  10-24-dated section (`Task 3.12a (lane/a)`), preserving
  non-decreasing date order with zero inversions. Verified: 0 removed
  lines vs HEAD (206-line lane block + 1 blank seam added), 0 removed
  lines vs lane/b.

Auto-merged + verified: `test-port-map.json` byte-identical to HEAD
(2381 records — planned 1824 / port 336 / golden 155 / drop 66);
`python-removal.json` identical (535 rows); EOF LFs intact;
`git ls-files -u` empty; no markers. The F2 mirror itself
(`static_braking.py` ↔ `static_brake.{hpp,cpp}`, `physical_runtime.py`
↔ `step.cpp`) is exact — same `reseat` predicate, expression order,
and the single `advance=true,active=false` call site.

## 22-vs-21 reconciliation (review finding)

`/tmp/f2_quick_gate.log` (the lane's own run log) lists exactly 22
`FAILED` entries: the replay class was **13** (9 `test_replay_session`
+ 4 `test_replay_recording_v3`), not the itemized 12. The reported
total was correct; only the class count was off by one. The lane
section's `(12)` is corrected to `(13)` in place; the reconciliation
with the verbatim node list is recorded in the merge-back evidence
section. Classification unchanged (poisoned shared equilibrium cache,
infra flake).

## Merge-back L2 (physics escalation)

- `cmake --build native/build -j10`: green.
- `cmake --build build/native-release -j10`: green; touched TUs
  (`static_brake.cpp`, `step.cpp`, `test_adapter.cpp`,
  `test_native_static_brake.cpp`) compile clean; only pre-existing
  `ld` duplicate-library chatter.
- `ctest -R 'reference\.(native_static_brake|native_pneumatic|
  native_rider|project_constraint)'` on `build/native-release`:
  **10/11** — `reference.native_static_brake` `FAIL fixture is not a
  regular file` (documented baseline; the new reseat C++ assertions
  compile but are unrun — fixture uncaptured, binary bails at load).
- `ctest -L reference -j8`: **41/48** — the same 7 documented baseline
  reds verbatim: `native_static_brake`, `pneumatic_tire_oracle`,
  `native_clock_contracts`, `native_drive_policies`,
  `native_drivetrain` (fixture-missing family), `period_buffer`
  (initial-state contract differs), `golden_episode_tool` (3.03 oracle
  npz). `reference.golden_episode_regressions` **passed** (8.70 s).
- `ctest -R 'migration_T' -j8`: **50/53** — `T09` (integration vector
  bitwise), `T29` (weld_servo_gain unknown key), `T56` (documented
  shellcheck-0.11.0 stderr-format env drift). **`T25` PASSED** —
  consistent with the F2 root cause verbatim: the contract hash binds
  to the capturing worktree's path and this `python-free-cpp` checkout
  is where the fixture was captured.

### Settle-dependent nodes — cold cache (fresh `BIKE_SIM_EQUILIBRIUM_CACHE_DIR` per run)

- Savage oracle settle (`/tmp/f2-m5/repro_settle.py`, adapted from the
  lane's `/tmp/f2-repro` harness): **converged 4,720 steps,
  residual_qacc=0.009105692229970314**, `cache_hit=False` —
  bitwise-identical to the lane's post-fix signature.
- alpine_rocky distributed settle: **converged 2,400 steps,
  residual=0.013185039244472185, reseat events: 1** — lane verbatim.
- realtime `[rough_uphill_savage.toml]`: **16,000 steps / 20.0 s
  completed**; fails only `factor 0.6379 >= 1.0` (documented RTF
  baseline; lane 0.6899 — host-load-sensitive).
- `test_savage_is_ridden_to_the_end`: **settle + full 110 s /
  88,000-step ride completed** (449 s); fails only `max x = 58.27 m <
  120 m` (documented baseline; lane 58.32 m — chaotic-ride drift, same
  class).
- `test_native_static_brake.py` parity (`NATIVE_TEST_BUILD_PATH=
  native/build`): **9/9** incl. `test_hold_reseat_parity_sequence`.
- `test_tire_channel_parity_across_backends`: progresses past both
  settles; fails downstream `root.front.penetration_m` ~1.74e-6 vs
  atol 1e-9 — documented port-maturity gap, stays `planned`.
- `test_rider_allocation` + `test_monitor_strictness` +
  `test_equilibrium_state_file`: **70/70**.

### Linux aarch64 container (`bike-dev-python-free-cpp`, `/build/root-clang`, clang++-18)

`container.sh sync` → target-scoped build clean (**0 warnings**);
`ctest -R 'reference\.(native_static_brake|golden_episode_
regressions)'`: **1/2** — `native_static_brake` same fixture-missing
baseline; `golden_episode_regressions` **Passed 11.90 s** (native
settle converges on both platforms).

## Unblocked work

- 17 stopped 3.28 `resolved`-fixture records — savage settle now
  converges deterministically (4,720 steps / 0.009105692229970314).
- Savage nodes complete end-to-end; remaining reds are the documented
  plateau-climb/RTF baseline gaps.
- `migration_T25` recapture unblocked for the path-hash fix (settle
  inputs stable; fix belongs to the T25 task).

## Concerns

- None blocking. `native_static_brake`'s new reseat C++ assertions are
  compiled but unrun until the oracle `Node` golden is captured — the
  Python parity suite (9/9 incl. `test_hold_reseat_parity_sequence`)
  is the live gate, as designed.
- The replay-cache poisoning vector (`fingerprint()` excludes
  `ExperimentConfig`/`demand`/`rider_program`; `reset()` re-settles
  dirty state under the same key) remains a live infrastructure defect
  — it manufactures intermittent replay reds under shared `$TMPDIR`;
  recommend the parent surface it.
- L3 (`tools/run_tests.sh native`) not run — bounded merge-back scope;
  checkpoint profile owns it.
