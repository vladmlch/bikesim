# Task M9 — lane/c batch 3.14b merge-back report

Worktree: `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/python-free-cpp`
Branch: `impl/python-free-cpp` (was `411c92f3`, now `f197a058`)
Merged: `lane/c` tip `319e2c29` (base `2e59c282`; 9 commits — 5 ports,
registrations+ledgers, task evidence, indent-2 restore, ledger
fix-round): 79 records — 62 golden / 17 port / 0 drop across
`test_contact_safety.py`, `test_attachment_budget.py`,
`test_attachment_wrench.py`, `test_native_attachment_wrench.py` (incl. 3
`owner_task: T53` rows), `test_native_rider_forces.py`; new CTest nodes
`reference.{contact_safety,attachment_budget,attachment_wrench,
native_attachment_wrench,native_rider_forces}` (labels `t52
rider|terrain golden slow`); ~514 golden fixture files both platforms
incl. per-platform `episode/pinned_savage_40/` artifacts; raw-facet
`concave_span_deg` overload (`profile.hpp` + `contact_safety.cpp`) and
`golden::Node::has_next()` test support.

## Commits

- `95ebd73c` — `merge: lane/c batch 3.14b
  contact-safety/attachment/rider-forces ports` (`git merge --no-ff
  lane/c`, one conflict resolved + verified auto-merges)
- `f197a058` — `docs(migration): lane/c 3.14b merge-back evidence`
  (appended `## 2026-10-25 — Task 3.14b (lane/c) merge-back`)

Both carry `Co-Authored-By: Devin
<158243242+devin-ai-integration[bot]@users.noreply.github.com>`.

## Conflict resolution

One real conflict: `migration/cpp-linux-completion-evidence.md` — both
sides appended at the EOF seam after the base's `Task 3.16 (lane/c)
merge-back`. Chronological resolution per the M5 mid-file-insertion
precedent: the lane's `Task 3.14b` section was authored at `2026-10-25
02:10:58 +0300` (`97264449`), so it is inserted between HEAD's
`Task 3.14a (lane/d) merge-back` (authored `00:38 +0300`) and
`Task 3.28 (lane/a) resume` (authored `07:42 +0300`) — preserving real
commit-time order. Verified byte-exact programmatically: merged file ==
HEAD blob with the lane's 4,960-byte suffix (seam blank + 99-line
section) inserted verbatim at that seam; zero HEAD bytes absent;
trailing LF at EOF.

Auto-merged, then verified semantically (per-record union check keyed on
`(source_path, source_symbol)`):

- `test-port-map.json`: 2,381 record keys identical across
  base/HEAD/lane/merged; **121 records changed only by HEAD, 79 only by
  the lane, 0 contested, 2,181 unchanged** — merged field-equal to the
  changing side on every record (0 mismatches). All 62 lane golden rows
  carry `golden_platforms`; the `8c88ce7f` indent-2 restore is
  preserved (merge diff vs HEAD confined to the 79 record bodies).
  Non-record top-level keys identical across all versions.
- `python-removal.json`: 535 rows; 6 HEAD-changed + 5 lane-changed + 0
  contested; the five batch files flipped `planned →
  replacement_landed` with refreshed sha256/line_count and
  `verification_ids` retargeted to the new `reference.*` nodes.
- `Phase3Ports.cmake`: union — HEAD's 3.12b pneumatic trio intact +
  the lane's five registrations at EOF.
- Manifest/initial-state: HEAD's T25 schema-2 layout kept —
  `welded_flat.json` only under platform-keyed
  `initial_state/{darwin-arm64,linux-aarch64}/`; the lane's only
  `manifest.json` adds are the new `pinned_savage_40` captures.
- Python sources: the four lane-touched test files are lane-only
  changes — merged blobs byte-identical to the lane's; no HEAD-side
  edits existed to lose.
- Trailing LFs verified on all text files. Golden `.json`/`.bin`
  fixtures intentionally keep the repo's no-trailing-LF fixture
  convention (verified against the lane blobs byte-verbatim —
  `pinned_savage_40/manifest.json` ends `}` like every existing
  fixture manifest).

## Merge-back L2

- `cmake --build native/build -j10`: green (auto-reconfigure picked up
  the five new registrations).
- `cmake --build build/native-release -j10`: green; all five new
  executables link; only pre-existing `ld` duplicate-library chatter.
- `ctest --test-dir build/native-release -R
  'reference\.(contact_safety|attachment_budget|attachment_wrench|native_attachment_wrench|native_rider_forces)'`:
  **5/5** (1.07 s / 0.82 s / 0.87 s / 1.08 s / 1.08 s).
- `ctest -L reference -j8`: **49/55** — all five new nodes green inside
  the sweep; six reds = same baseline set. Five keep the verbatim
  `FAIL fixture is not a regular file` signature (`native_static_brake`,
  `pneumatic_tire_oracle`, `native_clock_contracts`,
  `native_drive_policies`, `native_drivetrain`). `golden_episode_tool`
  stays red but the signature **evolved** — see Concerns.
- Linux aarch64 `bike-dev-python-free-cpp` (`/build/root-clang`,
  clang++-18): `container.sh sync` → scoped build of the five targets
  clean → `ctest -R 'reference\.(…)'` **5/5** — linux-aarch64 goldens +
  `pinned_savage_40` episode validated on-platform.
- Parity pytest cold-cache (`NATIVE_TEST_BUILD_PATH=$PWD/native/build`
  + `BIKE_NATIVE_BUILD_PATH` same, fresh
  `BIKE_SIM_EQUILIBRIUM_CACHE_DIR=/tmp/m9-coldcache`,
  `uv run --frozen --group native`, all five sources): **159 passed /
  1 warning** in 39.29 s — lane count verbatim. Warning is the
  documented `invalid reference step at 0.005000000:
  ('energy.constraint_work',)` RuntimeWarning
  (`test_native_rider_forces::test_accumulator_order_golden`); LAPACK
  `DLASCL` stderr noise as ledger-noted.
- `ctest -R 'migration_T' -j8`: **50/53** — **`migration_T25` PASSED
  (27.12 s)** (regression check on the just-merged T25 fix). Reds
  baseline-verbatim: `T09` (integration vector bitwise), `T29`
  (`weld_servo_gain: unknown key`), `T56` (documented shellcheck-0.11.0
  stderr-format env drift: `t56_scripts_depth2_bad: stderr missing
  'linux/bad.sh'`).

## Concerns

- **`golden_episode_tool` signature changed (same baseline set member,
  still red).** The lane's per-platform `episode/pinned_savage_40/`
  capture retired the fixture-missing bail (`task 3.03 oracle
  acquisition required … episode.npz`); the test now runs the real
  recapture comparison and fails `FAIL native recorder rows reproduce
  Python oracle` with step-0 channel divergence. This is the lane's own
  documented caveat surfacing: the committed `episode/physics.toml`
  does not converge (`freeze.py` needs the `_pin_oracle_kernels`
  `activation_tau_s = 0.025` config); the pinned capture exists for the
  `native_rider_forces` state-replay oracle path (green on the same
  artifact), while `golden_episode_tool` recaptures under the committed
  config. Not a merge regression — a pre-existing gap now exercised.
  Flagged in the merge-back evidence; parent may want a baseline-text
  update.
- `migration_T56` reproduces standalone (shellcheck 0.11.0 env drift,
  also seen at M5/M8) — environment-dependent, unrelated to the batch.
- The replay-cache poisoning vector flagged in M5–M8 remains a live
  infrastructure defect for the parent to surface.
- L3 (`tools/run_tests.sh native`) not run — bounded merge-back scope;
  checkpoint profile owns it.
