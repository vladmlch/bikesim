# Task M6 — lane/d batch 3.14a merge-back report

Worktree: `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/python-free-cpp`
Branch: `impl/python-free-cpp` (was `a5ed745`, now `9c12ce6`)
Merged: `lane/d` tip `6790a61` (base `e843f0d`; commits `e5dddc6` contacts,
`6790a61` contact math) — 55 records: 37 golden / 18 port / 0 drop;
~6.1 MiB goldens/platform; new nodes `reference.native_rider_contacts` +
`reference.native_rider_contact_math` (`t52 rider golden`).

## Commits

- `ce2a774` — `merge: lane/d batch 3.14a rider contacts/contact-math ports`
  (`git merge --no-ff lane/d`, one conflict resolved + verified auto-merges)
- `9c12ce6` — `docs(migration): lane/d 3.14a merge-back evidence`
  (appended `## 2026-10-24 — Task 3.14a (lane/d) merge-back`)

Both carry `Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>`.

## Conflict resolution

One real conflict: `migration/cpp-linux-completion-evidence.md` — both sides
appended at the same EOF seam. Chronological resolution per precedent: the
lane's `2026-10-25` batch section (commits `2026-10-25 00:14–00:15 +0300`,
postdating every HEAD merge at `2026-10-24 22:26–23:11`) sits at EOF after
`2026-10-24 — F2 (lane/b) merge-back` — zero date inversions inside the
merged content, 0 removed lines vs HEAD stage (exactly the 115-line lane
block + 1 blank seam added).

Auto-merged, then verified semantically (programmatic union check,
`/tmp/verify_union.py`):

- `migration/test-port-map.json`: 2381 records, **0 union violations**
  (every record equals the side that changed it; no landed disposition
  regressed). Batch: 55 records → 37 `golden` / 18 `port`, all on the two
  rider-contact sources. Merged totals: planned 1769 / port 354 /
  golden 192 / drop 66 (base 1926/275/117/63; HEAD-side 104 changed
  records preserved verbatim incl. F2-untouched rows).
- `migration/python-removal.json`: 535 rows, **0 union violations** — lane's
  two rows → `replacement_landed` with CTest-node `verification_ids`;
  HEAD's seven landed rows (3.16 four + 3.12b three) keep their values.
- `native/tests/reference/Phase3Ports.cmake`: all registrations present —
  3.12a trio, 3.12b trio, 3.16 four, plus the batch pair in the lane's
  position after `ride_plots_html_markers` (`t52 rider golden`, `bike_app`).
- Python oracle files (`test_native_rider_contacts.py`,
  `test_native_rider_contact_math.py`): byte-identical to lane/d — additive
  `_golden.record` edits only; HEAD untouched. F2's landed
  `test_rider_allocation.py` / `test_monitor_strictness.py` stub fixes were
  never in the conflict set — intact.
- All four files keep trailing EOF LF; `git ls-files -u` empty; no markers.

## Merge-back L2

- `cmake --build native/build -j10`: green.
- `cmake --build build/native-release -j10`: green; the two new reference
  TUs compile/link clean (only pre-existing `ld` duplicate-library
  chatter).
- `ctest --test-dir build/native-release -R 'reference\.native_rider'
  --output-on-failure`: **5/5** — `native_rider_contacts` 0.95 s,
  `native_rider_contact_math` 0.90 s (this batch), plus 3.16's
  `native_rider_allocation` 39.40 s, `native_rider_dynamics` 0.05 s,
  `native_rider_information_boundary` 0.04 s. (Plan predicted 4/4; the
  regex actually matches five nodes — `information_boundary` also matches
  `native_rider`. All green either way.)
- `ctest --test-dir build/native-release -L reference -j8
  --output-on-failure`: **43/50** — the same seven documented baseline
  reds verbatim: `native_static_brake`, `pneumatic_tire_oracle`,
  `native_clock_contracts`, `native_drive_policies`, `native_drivetrain`
  (all `FAIL fixture is not a regular file` — goldens owned by unmerged
  sibling lanes / uncaptured oracle `Node`), `period_buffer` (`FAIL
  initial-state physical model contract differs`), `golden_episode_tool`
  (`FAIL task 3.03 oracle acquisition … episode.npz`). Fixture-missing
  family is **zero for landed batches** — both new nodes replay their
  captured fixtures and pass; `golden_episode_regressions` passes
  (17.37 s).
- Linux aarch64 container `bike-dev-python-free-cpp` (`/build/root-clang`,
  clang++-18): `container.sh sync` → target-scoped build of the two TUs
  clean (0 `warning:` lines); `ctest root-clang -R
  'reference\.native_rider_(contacts|contact_math)'` → **2/2**
  (0.15 s / 0.11 s). Validates the linux-aarch64 golden tree incl. the
  per-platform-divergent leaves.
- Parity pytest (`NATIVE_TEST_BUILD_PATH=$PWD/native/build`, uv env):
  **309 passed / 51.53 s** — matches the lane's reported 309 verbatim.
  Cold-cache probe (fresh `BIKE_SIM_EQUILIBRIUM_CACHE_DIR`): **309
  passed / 44.28 s** — identical, no cache mediation.

## Concerns

- None blocking. The merge plan's "4/4" for `-R 'reference\.native_rider'`
  undercounted the pattern (it matches 5); noted in the evidence section.
- The merge-back evidence section is titled `2026-10-24` per mandate while
  the lane's batch section carries `2026-10-25` (true commit time) — a
  single label inversion that preserves correct event-time order;
  documented in-place.
- `native_static_brake` remains fixture-uncaptured (documented baseline;
  oracle `Node` golden not yet acquired) — its C++ assertions compile but
  don't execute; the Python parity suite stays the live gate.
- The replay-cache poisoning vector flagged in M5 remains a live
  infrastructure defect for the parent to surface.
- L3 (`tools/run_tests.sh native`) not run — bounded merge-back scope;
  checkpoint profile owns it.
