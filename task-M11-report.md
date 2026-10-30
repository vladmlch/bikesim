# Task M11 — lane/a F3 realtime-gate merge-back report

Worktree: `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/python-free-cpp`
Branch: `impl/python-free-cpp` (was `c4cf470f`; now `3cdfc8a7`)
Merged: `lane/a` tip `a1410061` (base `f197a058`; 4 commits —
`test_realtime_gate.cpp` + Phase3Ports registration `33c801f8`,
TESTING.md `7693407f`, T53 ledger record `planned → port` `d8f90ab9`,
lane evidence incl. the real L3-gate tail `a1410061`). The F3 ruling
re-aims the >=1x RTF contract at the produced Release
`bike-measure-realtime` artifact: one CTest node per track
(`reference.realtime_gate.rough_uphill_savage` /
`.rough_uphill_extreme`, labels `reference;t53;runtime;slow`, TIMEOUT
900) spawning the artifact through `platform::run` with `--benchmark`
(the Python `strict=False` equivalent), asserting the schema-1 report.

## Commits

- `44a3eadb` — `merge: lane/a F3 realtime-gate re-aim at the native
  Release artifact` (`git merge --no-ff lane/a`, one conflict resolved +
  verified auto-merges)
- `3cdfc8a7` — `docs(migration): lane/a F3 merge-back evidence`
  (appended `## 2026-10-25 — F3 (lane/a) merge-back`)

Both carry `Co-Authored-By: Devin
<158243242+devin-ai-integration[bot]@users.noreply.github.com>`.

## Conflict resolution

One real conflict:

- `migration/cpp-linux-completion-evidence.md` — both sides appended at
  the EOF seam. Chronological per precedent: HEAD's `Task 3.13 (lane/d)
  merge-back` block (`c4cf470f`, 2026-10-25 23:17 +0300) precedes the
  lane's `Task F3 (lane/a)` section (`a1410061`, 2026-10-26 01:52
  +0300), so the lane's section lands at EOF. Verified byte-exact
  programmatically: merged file == HEAD blob + the lane's 6,459-byte
  suffix verbatim; single trailing LF at EOF.

Auto-merged, then verified semantically:

- `migration/test-port-map.json` — per-record union check keyed on
  `(source_path, source_symbol)`: 2,381 keys identical across
  base/HEAD/lane/merged; **1 record changed only by the lane** (the T53
  `test_realtime_factor_for_full_twenty_seconds` row: `planned → port`,
  `native_test_id` re-aimed at
  `reference.realtime_gate::test_realtime_factor_for_full_twenty_seconds`,
  both collected Python node ids + `comparison_class: contract` +
  both-platform evidence), 65 changed only by HEAD (the 3.13 batch,
  intact), 0 contested, 0 bad merges, no contested top-level fields.
- `native/tests/reference/Phase3Ports.cmake` — union: 55 registrations
  = the 54 existing + `reference.realtime_gate` (`add_executable` +
  `add_dependencies` on `bike-measure-realtime` + per-track `add_test`,
  labels `reference;t53;runtime;slow`, TIMEOUT 900, WORKING_DIRECTORY
  the target dir); zero duplicate test names.
- `docs/TESTING.md` and `test_realtime_gate.cpp` byte-identical to the
  lane blobs (HEAD never touched TESTING.md since the base — the
  supersession annotations + the re-aimed-gate section land verbatim);
  `migration/python-removal.json` byte-identical to HEAD.
- Trailing LFs preserved on every merged text file.
- Lane-local untracked content deliberately not merged: stray
  `tests/reference/_probe_*.py` and the lane's `.superpowers/` dir —
  the merge carries only tracked content. The lane-local
  `task-F3-report.md` was copied into the integration
  `.superpowers/sdd/2026-10-20-cpp-linux-completion/` dir (untracked,
  matching the task-*.md housekeeping convention).

## Merge-back L2

- `cmake --build native/build -j10` (darwin-arm64): green, 105 targets.
  `cmake --build build/native-release -j10`: green — cmake re-generated
  for the Phase3Ports edit; `bike-measure-realtime` (the realtime
  artifact target) up to date, `reference.realtime_gate` compiled +
  linked under the project warning policy.
- `ctest --test-dir build/native-release -R 'reference\.realtime_gate'
  --output-on-failure`: **2/2** — savage 12.36 s wall (runs RTF
  8.8897/8.9445/8.9594, median **8.9445**), extreme 11.57 s wall (runs
  8.9276/8.9002/8.9105, median **8.9105**). All runs
  `duration_reached`, 16,000/16,000 steps,
  `first_failure=[0.005,["energy.constraint_work"]]` retained under
  `--benchmark` — the documented non-strict-monitor equivalence;
  `build_type=Release` provenance asserted.
- `ctest -L reference -j8`: **59/65** — both realtime-gate nodes green
  inside the sweep (18.78 s / 18.64 s under -j8 load). Six reds,
  baseline-verbatim vs the M10 set: five `FAIL fixture is not a
  regular file` (`native_static_brake`, `pneumatic_tire_oracle`,
  `native_clock_contracts`, `native_drive_policies`,
  `native_drivetrain`) plus `golden_episode_tool` keeping its
  documented `FAIL native recorder rows reproduce Python oracle`
  signature.
- Linux aarch64 `bike-dev-python-free-cpp` (`root-clang`, clang++-18,
  Release): `container.sh sync` → `build root-clang
  reference.realtime_gate` (11 steps; `add_dependencies` pulled
  `bike-measure-realtime`) → `ctest -R '^reference\.realtime_gate'
  --output-on-failure`: **2/2** — savage 13.89 s (median **7.1246**),
  extreme 13.71 s (median **7.1403**).
- `ctest -R 'migration_T' -j8` (`build/native-release`): **50/53** —
  `migration_T25` **green** (12.78 s); reds baseline-verbatim: `T09`
  (`integration vector resolves bitwise`), `T29` (`weld_servo_gain:
  unknown key`), `T56` (shellcheck-0.11.0 stderr-format env drift).
- L3 not re-run per merge-back scope; the lane's real L3 result
  (`bash tools/run_tests.sh native` on `d8f90ab9`, exit 8 at the
  binding-graph CTest on the documented T09/T29 baseline pair, T25
  green, all nine static-analysis sweeps green) is carried in the
  merged lane evidence section.

## RTF numbers

| Platform | savage median | extreme median | Lane's own medians |
|---|---|---|---|
| darwin-arm64 (`build/native-release`) | **8.9445** | **8.9105** | 11.5188 / 11.6194 |
| linux-aarch64 (`root-clang` container) | **7.1246** | **7.1403** | 9.1041 / 9.0918 |

All margins are ~7-9x over the unchanged >=1x contract bar; the
merge-back medians are lower than the lane's recorded runs — ordinary
dev-host load variance on a shared machine (four sibling lane
containers up), not a contract regression.

## Concerns / notes

- Merge-back RTF medians (~8.9x darwin / ~7.1x linux) came in below
  the lane's recorded ~11.5x / ~9.1x — within host-load variance;
  both platforms still clear the >=1x bar by ~7-9x.
- Python-path `test_realtime_gate.py` nodes stay baseline-red until
  T63 (superseded, not fixed) — expected and documented in TESTING.md.
- Task 3.10 family is closed on the native side; remaining work is
  only the T63 oracle/Python-node removal.
- `migration_T56` still red on the documented shellcheck-0.11.0 env
  drift; the M5–M9 replay-cache poisoning vector remains a live
  infrastructure defect for the parent to surface.
