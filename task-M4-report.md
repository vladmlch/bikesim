# Task M4 — lane/a batch 3.12b merge-back report

Worktree: `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/python-free-cpp`
Branch: `impl/python-free-cpp` (was `2e59c28`, now `464d1b7`)
Merged: `lane/a` tip `4d8fe47` (base `e843f0d`; commits `2d78972` model,
`d3bc3af` wire, `4d8fe47` tire + ledgers + evidence + TESTING.md)

## Commits

- `951f065` — `merge: lane/a batch 3.12b pneumatic model/tire/wire ports`
  (`git merge --no-ff lane/a`, conflict resolution + auto-merges)
- `464d1b7` — `docs(migration): lane/a 3.12b merge-back evidence`
  (appended `## 2026-10-24 — Task 3.12b (lane/a) merge-back`)

Both carry `Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>`.

## Conflict resolution

One real conflict: `migration/cpp-linux-completion-evidence.md` — both sides
appended dated sections at EOF (HEAD: `2026-10-24` Task 3.16 merge-back;
lane/a: `2026-10-24` batch 3.12b section). Resolved chronologically: the
lane's batch section (events at commits 22:46) sits ahead of the 3.16
merge-back (events 22:55–23:03) — the file's same-dated sections are
event-time ordered and the prior merge-back applied the same rule across
date boundaries. The blank-line separator HEAD's seam had dropped is
restored. Merged file verified byte-identical to HEAD outside the inserted
101-line lane section (which matches the lane verbatim).

Auto-merged, then verified semantically:

- `migration/test-port-map.json`: 2381 records, 0 record-level union
  violations vs base `e843f0d` / HEAD `2e59c28` / lane `4d8fe47`.
  3.12b adds 38 golden / 9 port / 2 planned (model 19g; tire 10g/1p/2-planned
  — the planned pair are the parked baseline-reds; wire 9g/8p). Preserved:
  3.12a 53g/8p, 3.28 24p/12d/17p, 3.16 52p/3d.
- `migration/python-removal.json`: 535 rows, 0 union violations. Three
  pneumatic rows → `replacement_landed`; 3.16 rows kept HEAD's values
  (lane's pre-3.16 stale `planned` copies correctly lost).
- Both ledgers keep trailing EOF LF.
- `native/tests/reference/Phase3Ports.cmake`: all registrations present —
  3.12a trio, 3.12b trio (`t52 tyre golden`, `+slow` on tire), 3.16 four.
- `docs/TESTING.md`: lane's +8 baseline-red block kept (HEAD had no edits).
- Three Python parity sources: byte-identical to lane (additive capture
  edits; HEAD untouched — nothing to union).
- `git ls-files -u` empty; no conflict markers in any file.

## Merge-back L2

- `cmake --build native/build -j10`: green.
- `cmake --build build/native-release -j10`: green; 3 new reference
  executables compile/link clean (only pre-existing `ld` duplicate-library
  chatter).
- `ctest --test-dir build/native-release -R 'reference\.native_pneumatic'
  --output-on-failure`: **6/6** — geometry 0.07s, carcass 0.15s, brush
  0.17s, model 0.69s, wire 0.76s, tire 1.33s. The lane's missing-fixture
  reds cleared: fixtures land with the merge.
- `ctest --test-dir build/native-release -L reference -j8
  --output-on-failure`: **41/48** — all 7 reds match the documented
  baseline verbatim: `native_static_brake`, `pneumatic_tire_oracle`,
  `native_clock_contracts`, `native_drive_policies`, `native_drivetrain`
  (`FAIL fixture is not a regular file`), `period_buffer` (`FAIL
  initial-state physical model contract differs`), `golden_episode_tool`
  (`FAIL task 3.03 oracle acquisition required … episode.npz`). All 3 new
  nodes pass inside the run.
- Linux aarch64 container `bike-dev-python-free-cpp` (`/build/root-clang`,
  clang++-18): after `container.sh sync`, target-scoped build of the three
  TUs clean (zero `warning:` lines); `ctest root-clang -R
  'reference\.native_pneumatic_(model|tire|wire)'` → **3/3** (0.13s /
  0.13s / 0.55s). Validates the linux-aarch64 golden tree incl. the
  per-platform `pneumatic_tire/` runtime bundles.
- Parity pytest (3 sources, `NATIVE_TEST_BUILD_PATH=$PWD/native/build`):
  **102 passed / 2 failed / 104 collected** in 120.54s — matches the lane's
  documented counts (36 + 11+2 + 55). The 2 reds are the batch's documented
  baseline pair now in TESTING.md: `test_pneumatic_golden_episode_event_rich`
  (`residual=0.0814217; front_wheel_spin` after 64000 steps) and
  `test_pneumatic_probe_and_clock` (`interval 1: bitwise mismatch`). C++
  ports parked (`planned`) per lane design.

L3 (`tools/run_tests.sh native`) not run — bounded merge-back scope; no
`native/src` content touched.

## Concerns

- None blocking. The ~49 MiB two-platform golden payload was merged as-is
  per the user ruling already recorded in the lane's batch section
  (referenced from the merge-back section).
- `event_rich`/`probe_and_clock` remain intentionally unregistered
  (`planned`) — documented baseline reds, not a merge defect.
