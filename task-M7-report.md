# Task M7 — lane/a batch 3.28-resume merge-back report

Worktree: `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/python-free-cpp`
Branch: `impl/python-free-cpp` (was `9c12ce6`, now `f8abe87`)
Merged: `lane/a` tip `ead6c430` (base `a5ed745`; single resume commit) —
17 previously-stopped `test_native_input_contracts.py` records ported on
top of the F2 settle fix: 16 `port` + 1 `golden`
(`test_schema_migration_preserves_resolved_fork_force`, 24 sidecars:
12 per platform). `test_native_input_contracts.cpp` grows 17 → **25
dispatched functions**. This merge **completes batch 3.28**: all 53
records across `test_native_atomicity.py` (21) +
`test_native_input_contracts.py` (32) now carry landed dispositions —
40 port / 12 drop / 1 golden on the 3.28 evidence tag, zero planned.

## Commits

- `44aa625c` — `merge: lane/a batch 3.28-resume input-contract resolved
  ports` (`git merge --no-ff lane/a`, one conflict resolved + verified
  auto-merges)
- `f8abe871` — `docs(migration): lane/a 3.28-resume merge-back evidence`
  (appended `## 2026-10-24 — Task 3.28 resume (lane/a) merge-back`)

Both carry `Co-Authored-By: Devin
<158243242+devin-ai-integration[bot]@users.noreply.github.com>`.

## Conflict resolution

One real conflict: `migration/cpp-linux-completion-evidence.md` — both
sides appended at the same EOF seam after the F2 merge-back section.
Chronological resolution per precedent: HEAD's `2026-10-25` 3.14a batch +
`2026-10-24` merge-back sections (events `00:14–00:38 +0300`) precede the
lane's `2026-10-2x — Task 3.28 (lane/a) resume` section (commit `ead6c430`
at `07:42 +0300`). Verified byte-exact programmatically: merged file ==
HEAD blob + the lane's appended 7,367-byte section verbatim; zero HEAD
lines absent.

Auto-merged, then verified semantically (programmatic union check,
`/tmp/m7_verify_union.py`):

- `migration/test-port-map.json`: **2,381 records, 0 union violations**
  (key sets identical across base/HEAD/lane/merged; nothing mangled; no
  landed disposition regressed). HEAD changed 55 records (3.14a batch),
  lane changed 17 — disjoint (`both=0`); the 17 are exactly this batch:
  16 `port` + 1 `golden` with
  `golden_platforms: [darwin-arm64, linux-aarch64]`. Merged totals:
  planned 1,752 / port 370 / golden 193 / drop 66.
- `migration/python-removal.json`: 535 rows, **0 union violations** —
  lane's `test_native_input_contracts.py` row → `replacement_landed`
  (verification `reference.native_input_contracts`); HEAD's two 3.14a
  landed rows kept. Merged: 515 planned / 20 replacement_landed.
- `native/tests/reference/test_native_input_contracts.cpp`: lane-side
  only, auto-merged — 25 `test_*` calls in `main()` dispatch (lines
  739–764).
- 24 golden sidecars staged under
  `tests/fixtures/golden/{darwin-arm64,linux-aarch64}/
  tests_reference_test_native_input_contracts.py/`.
- All merged files keep trailing EOF LF; `git ls-files -u` empty; no
  conflict markers.

## Merge-back L2

- `cmake --build native/build -j10`: green.
- `cmake --build build/native-release -j10`: green;
  `reference.native_input_contracts` relinks clean.
- `ctest --test-dir build/native-release -R
  'reference\.native_input_contracts' --output-on-failure`: **1/1**
  (0.66 s).
- `ctest --test-dir build/native-release -L reference -j8
  --output-on-failure`: **43/50** — the same seven documented baseline
  reds verbatim: `native_static_brake`, `pneumatic_tire_oracle`,
  `native_clock_contracts`, `native_drive_policies`, `native_drivetrain`
  (`FAIL fixture is not a regular file` — goldens owned by unmerged
  sibling lanes / uncaptured oracle `Node`), `period_buffer` (`FAIL
  initial-state physical model contract differs`), `golden_episode_tool`
  (`FAIL task 3.03 oracle acquisition … episode.npz`).
  `native_input_contracts` passes inside the sweep (0.08 s);
  `golden_episode_regressions` passes (7.81 s).
- Linux aarch64 container `bike-dev-python-free-cpp`
  (`/build/root-clang`): `container.sh sync` → target-scoped build clean;
  `ctest root-clang -R 'reference\.native_input_contracts'`: **1/1**
  (0.07 s) — linux-aarch64 sidecars replay green.
- Parity pytest cold-cache (`NATIVE_TEST_BUILD_PATH=$PWD/native/build`,
  fresh `BIKE_SIM_EQUILIBRIUM_CACHE_DIR=/tmp/m7-coldcache`, uv env):
  `tests/reference/test_native_input_contracts.py` → **97 passed /
  0 errors in 19.96 s** — the F2-unblocked claim holds on the merged
  tree (the lane/c run on this file had 59 fixture-setup errors).

## Concerns

- None blocking. The merge-back evidence section is titled `2026-10-24`
  per mandate; it ran `2026-10-25 ~08:06 +0300`, and the lane's batch
  section carries the literal `2026-10-2x` label — documented in-place
  (same convention as M6's note on section dates).
- `native_static_brake` and the four other fixture-missing reds remain
  the documented baseline (goldens owned by unmerged sibling lanes /
  uncaptured oracle `Node`); `period_buffer` + `golden_episode_tool`
  likewise. None authored by this batch.
- The replay-cache poisoning vector flagged in M5/M6 remains a live
  infrastructure defect for the parent to surface.
- L3 (`tools/run_tests.sh native`) not run — bounded merge-back scope;
  checkpoint profile owns it.
