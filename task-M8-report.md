# Task M8 — lane/d T25 merge-back report

Worktree: `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/python-free-cpp`
Branch: `impl/python-free-cpp` (was `f8abe87`, now `411c92f3`)
Merged: `lane/d` tip `5b0fc86b` (base `9c12ce6`; 4 commits) — the T25
`contract_sha256` worktree-portability fix: schema-2 initial-state
documents binding asset **content** (`envelope_sha256`/`strength_sha256`)
instead of resolved worktree-absolute paths, `platform::key()`,
platform-sibling reader preference, recaptured per-platform artifacts,
`period_buffer` probe re-pin (graft fix), manifest re-pins, evidence.

## Commits

- `327820b1` — `merge: lane/d T25 contract_sha256 worktree-portability
  fix` (`git merge --no-ff lane/d`, one conflict resolved + verified
  auto-merges)
- `411c92f3` — `docs(migration): lane/d T25 merge-back evidence`
  (appended `## 2026-10-25 — T25 (lane/d) merge-back`)

Both carry `Co-Authored-By: Devin
<158243242+devin-ai-integration[bot]@users.noreply.github.com>`.

## Conflict resolution

One real conflict: `migration/cpp-linux-completion-evidence.md` — both
sides appended at the same EOF seam. Chronological resolution per
precedent: HEAD's 3.28-resume sections (ending `08:06 +0300`) precede
the lane's T25 section (`5b0fc86b` at `08:12:38 +0300`). Verified
byte-exact programmatically: merged file == HEAD blob + the lane's
6,304-byte appended section verbatim; zero HEAD lines absent. All other
lane-touched files auto-merged to lane blobs (hash-checked, zero diffs).
The shared `initial_state/welded_flat.json` deletion preserved (rename
to `darwin-arm64/` + new `linux-aarch64/` sibling).

Semantic checks: all 4 platform schema-2 docs present with lane-evidence
sha256s verbatim (23545/23384/23571/23364 B; contracts `dc9ce816…`/
`639f49c5…`); zero absolute paths in payloads; shared spellings absent;
keyed `integration.f64` pins + shared pin retained; manifest
platform-keyed entries present; ledgers untouched and clean
(`test-port-map.json` 2,381 records, `python-removal.json` 535 rows);
evidence chronological; trailing LFs everywhere.

## Merge-back L2

- `cmake --build native/build -j10`: green. `cmake --build
  build/native-release -j10`: green.
- `ctest --test-dir build/native-release -R 'migration_T'`: **50/53** —
  **`migration_T25` PASSED (5.79 s)**, resolving the documented baseline
  red. Remaining reds baseline-verbatim: `migration_T09` (`integration
  vector resolves bitwise`), `migration_T29` (`weld_servo_gain: unknown
  key`), plus `migration_T56` on the documented shellcheck-0.11.0
  stderr-format env drift (`t56_scripts_depth2_bad: stderr missing
  'linux/bad.sh'`; flake the lane saw standalone but not under the gate).
- `ctest -R 'reference\.(period_buffer|native_pneumatic_tire)'`: **2/2**
  — period_buffer 2.60 s (graft fix: leaves the baseline set),
  native_pneumatic_tire 1.08 s.
- `ctest -L reference -j8`: **44/50** — six documented baseline reds
  verbatim (fixture-missing five: `native_static_brake`,
  `pneumatic_tire_oracle`, `native_clock_contracts`,
  `native_drive_policies`, `native_drivetrain`; plus
  `golden_episode_tool` 3.03 npz).
- **Path-portability proof spanning worktrees**: temp worktree
  `/private/tmp/m8-pathproof` at merge commit `327820b1`, Release
  configure with foreign `BIKE_FIXTURE_ROOT`
  (`/private/tmp/m8-pathproof/tests/fixtures/migration`) — a third
  absolute root distinct from capturing worktree `pfc-lane-d` and this
  merge worktree. `migration_T24` 1.39 s + `migration_T25` 5.75 s green.
- Linux aarch64 `bike-dev-python-free-cpp` (`/build/root-clang`,
  clang++-18): sync → scoped build clean; `migration_T24` 0.88 s,
  `migration_T25` 6.37 s, `period_buffer` 2.41 s,
  `native_pneumatic_tire` 0.55 s — **4/4**.
- Cold-cache probe (fresh `BIKE_SIM_EQUILIBRIUM_CACHE_DIR=
  /tmp/m8-coldcache`): period_buffer 2.04 s + native_pneumatic_tire
  0.53 s green.

## Baseline update

`migration_T25` removed from the live baseline: migration CTest baseline
is now `T09` + `T29` (T56 = documented env flake); `-L reference`
baseline drops `period_buffer` → six reds. `docs/TESTING.md` carries no
`migration_T25` baseline entry (its lists are pytest node IDs) — no
amendment needed; the ledger's earlier sections stand as the pre-fix
historical record.

## Concerns

- None blocking. `migration_T56` reproduces standalone (shellcheck
  0.11.0 stderr-format env drift — also seen at M5) while the gate's
  `check_scripts` sweep does not; environment-dependent, unrelated to
  T25.
- The replay-cache poisoning vector flagged in M5/M6/M7 remains a live
  infrastructure defect for the parent to surface.
- Temp worktree `/private/tmp/m8-pathproof` left registered
  (`git worktree list`); remove with `git worktree remove` when done —
  it is outside the repo and untracked.
- L3 (`tools/run_tests.sh native`) not run — bounded merge-back scope;
  checkpoint profile owns it.
