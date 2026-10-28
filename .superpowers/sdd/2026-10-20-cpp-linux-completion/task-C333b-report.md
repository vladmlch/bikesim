# Task C333b — test-port 3.33b report (research capture/frontend/lifecycle/sweep)

**Batch**: 3.33b — `tests/reference/test_research_capture.py` (7),
`test_research_frontend.py` (1),
`test_research_recording_lifecycle.py` (7),
`test_research_sweep.py` (3)
(18 records / 18 collected nodes; no parametrized expansions). CTest ids
`reference.native_research_{capture_contracts,frontend_contracts,
recording_lifecycle,sweep}`; labels `t53 research`; port-map rows carry
`owner_task: T53`.

**Lane**: `.worktrees/pfc-lane-e`, branch `lane/e`, base `842c2585`
(lane/e 3.29b merge-back tip carrying the lane/a 3.33a merge). Reuses the
3.33a seam `native/tests/reference/research_reference_support.{hpp,cpp}`;
the header gained the 3.33b recording-launch fixture
(`RecordingLaunch`/`recording_env` — `ResearchRun(env, out_root,
enabled=True, mode='test')` with catalog + run metadata frozen the same
way `cli::run_research` freezes them — plus the `row_count` and
`event_kinds` reader probes the Python sources spell on every stream
table). `recording_env` links `bike_cli` (`model_catalog` is the cli
metadata seam), so only the capture/lifecycle TUs compile the support
`.cpp`.

## Status: landed — 18 `port`, 0 `golden`, 0 `planned`

Every record in the four files is a native CTest. The batch is pure
contract work — capture-mode parity is native-vs-native (off vs full),
lifecycle/failure/signal/sweep behavior is structural — so no golden
fixtures were added and no live-oracle parity legs remain open. All four
source files are fully covered; `python-removal.json` marks them
`replacement_landed` with live hashes verified (sha256/line_count
unchanged since T01 for all four).

## Triage

| Source file | records | collected nodes | port | planned |
|---|---|---|---|---|
| `test_research_capture.py` | 7 | 7 | 7 | 0 |
| `test_research_frontend.py` | 1 | 1 | 1 | 0 |
| `test_research_recording_lifecycle.py` | 7 | 7 | 7 | 0 |
| `test_research_sweep.py` | 3 | 3 | 3 | 0 |
| **total** | **18** | **18** | **18** | **0** |

- **`reference.native_research_capture_contracts`** (7 tests, 73 checks):
  off vs full capture produce bitwise-identical step results,
  `integration_state` and pipeline counters while off retains no
  history; sealed evidence survives `reset()`/`close()`, returns the
  identical object on re-seal, forbids array mutation, and rejects
  `begin_control` on a sealed episode; the verify cursor re-delivers an
  un-acked prefix and only consumes after `acknowledge_applied` (the
  boxing-failure analog: the native read is const, so an un-acked
  suffix cannot be lost by construction), with stale/out-of-domain
  cursors refused; the two-episode launch writes nothing before
  finalize, then `Recording` reads back `episode_ids == (0,1)` with the
  multi-episode `select --episode` guard and `finish()` idempotent;
  an incomplete window seals the accepted physical prefix without
  fabricating a completed step and `load_replay_input` refuses the
  `incomplete-prefix` recording; a policy failure before the first
  request seals zero-step evidence (`physical_rows == 0`, one
  observation, not replay-eligible); interruption drains only the
  already-accepted control (`complete_control_steps == 1`, one policy
  call, `launch_interrupted` journaled).
- **`reference.native_research_frontend_contracts`** (1 test, 8
  checks): a partial `advance_pending(.008, 1)` tick calls the policy
  once and keeps the pending latch, a queued pause holds simulated
  time, and stop-at-boundary starts no further window.
- **`reference.native_research_recording_lifecycle`** (7 tests, 30
  checks): `bike-research` parser defaults (`--record` off,
  `--check-policy` conflict rejected); CLI policy-load failure
  publishes zero-step evidence through the recording path; `--out`
  without `--record` never publishes; the two second-Ctrl-C records run
  as isolated child probes (`--seal-child`/`--finalize-child`, same
  pattern as the T36 signal tests) — a SIGINT deferred inside the
  shielded seal journals `operator_stop_deferred` and still publishes
  before the deferred interrupt re-raises, while one inside finalize
  is noted (`"operator interrupt deferred"` fragment) without a
  journaled event; off-main-thread `InterruptShield` is a flag holder
  (`installed()==false`) whose `note()` still records, and an off-main
  `finish()` publishes; a frontend failure between windows keeps the
  committed prefix, marks the episode not replay-eligible and the
  recording is refused by `load_replay_input`.
- **`reference.native_research_sweep`** (3 tests, 19 checks):
  `--dry-run` lists one command per seed in order with passthrough
  arguments against the built `bike-research-sweep` artifact
  (`add_dependencies` keeps it fresh); `parse_seeds` accepts decimal,
  lists and `a-b` ranges and refuses malformed input; a failing child
  stub lands `sweep.json` with per-seed return codes and the aggregate
  nonzero exit.

## Confirmed traps found during the port

1. **`research_sweep` refuses a non-empty `--out` root.** The first
   sweep target wrote the stub executable inside the output root,
   tripping `sweep output already exists` and a segfault-shaped CTest
   result. The stub now lives in the temp root and `--out` is a fresh
   child directory.
2. **Designated-initializer order is part of the contract.** `Knobs`
   fields must appear in declaration order (`decimation` before
   `capture`; `duration_s` before `capture`) or clang-18 rejects the
   TU — three compile fixes on that surface alone.
3. **Linux reference tests live in the root graph, not the native
   graph.** `add_subdirectory(native/tests/reference)` is only on the
   root `CMakeLists.txt`; `/build/native-clang` (configured from
   `/src/native`) reports `ninja: error: unknown target
   'reference.native_research_capture_contracts'`. The qualified Linux
   reference build is `/build/root-ref` via
   `tools/linux/container.sh configure root-ref root`.
4. **A stale sync looks like a configuration bug.** The first
   `container.sh sync` ran before the `Phase3Ports.cmake` registration
   existed, so `ninja -t targets` showed the 3.33a names but not the
   new ones; re-syncing picked up the uncommitted registration (the
   sync ships `git ls-files --cached --others`, i.e. modified and
   untracked-but-unignored files).
5. **`seal_episode()` does not close an external window.** The
   incomplete-window port must step the pending window through
   `advance_control(target_step=1)` before sealing — sealing directly
   keeps the window open and records nothing the replay-refusal leg
   can observe.

## Verification evidence

- **CTest darwin-arm64** (`build/native-release`, AppleClang):
  `ctest -R 'reference.native_research_(capture_contracts|
  frontend_contracts|recording_lifecycle|sweep)$'` — **4/4 Passed**
  (capture 73, frontend 8, lifecycle 30, sweep 19 parent checks;
  the lifecycle's two child probes add their own in-process counts
  inside the `bike::platform::run` captures).
  Broader research scope `ctest -R reference.native_research` —
  **12/12 Passed** (all 3.33a + 3.33b targets).
- **CTest linux-aarch64** (`bike-dev-pfc-lane-e`, `/build/root-ref`,
  clang++-18 Release): same 4-target scope — **4/4 Passed**
  (8.80/0.47/9.42/0.13 s); full `reference.native_research` scope —
  **12/12 Passed** (26.98 s).
- **Static**: all four TUs compile warning-free under AppleClang and
  clang++-18; `<array>`/`<iostream>` includes and designated-initializer
  ordering fixed in-source, no suppressions.

## Ledger

- `migration/test-port-map.json`: all 18 rows landed in place —
  `disposition: port`, `comparison_class: contract`, measured
  `collected_node_ids` (18 nodes via `pytest --collect-only`),
  `native_test_id` rewritten to `reference.<target>::<fn>`,
  row-level 3.33b evidence strings. Counters: `port` 1141→1159,
  `planned` 772→754, `golden` 356, `drop` 125.
- `migration/python-removal.json`: all four source rows →
  `status: replacement_landed` with `verification_ids` set to the
  CTest ids; `sha256`/`line_count` verified live (unchanged since
  inventory on all four). Deletion stays gated at T63.

## Commits

| SHA | Unit |
|---|---|
| `b239bf5d` | 3.33b T53 native research ports — 4 CTests + support fixture, dual-platform green |
| (this commit) | 3.33b ledger dispositions + task report |
