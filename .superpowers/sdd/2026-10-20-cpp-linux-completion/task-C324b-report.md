# Task C3.24b report — golden episodes + ride backend CLI (T52)

Worktree `.worktrees/pfc-lane-d`, branch `lane/d`, base `c981bd70`.
Batch scope: five pytest files, 22 port-map records / 26 collected
nodes (`tests/reference/test_golden_episode.py` 2,
`test_golden_episode_v2.py` 4, `test_episode_compare.py` 3,
`test_divergence_horizon.py` 2, `test_ride_backend_cli.py` 11).

## SHAs

- `9e7b1156` — feat(3.24b): port `test_ride_backend_cli.py` to
  `reference.ride_backend_cli` (native TU + Phase3Ports registration).
- `<ledger sha>` — ledger(3.24b): port-map dispositions (19 port /
  3 drop) + removal rows `replacement_landed` + this report.

## Dispositions

| File | Records | Disposition | Native coverage |
|---|---|---|---|
| `test_golden_episode.py` | 2 | port ×2 | `reference.golden_episode_regressions::{test_capture_is_deterministic,test_saved_artifact_roundtrips}` (pre-existing native port, qualified this batch) |
| `test_golden_episode_v2.py` | 4 | port ×4 | `reference.golden_episode_regressions::{test_v2_state_and_forces_recorded,test_v2_roundtrip_and_determinism,test_exit_total_is_captured_independently_of_components,test_load_old_artifact_without_exit_total}` (v1/v2 parameter loop) |
| `test_episode_compare.py` | 3 | port ×3 | `reference.golden_episode_regressions::{test_replay_matches_golden_bitwise,test_compare_episode_on_saved_artifact,test_compare_reports_first_divergence}` |
| `test_divergence_horizon.py` | 2 | drop ×2 | none — `tools/divergence_horizon.py` measures a perturbation horizon between two live Python-oracle episode runs; no subject remains without the oracle (plan 3.24b note / ADR-0002) |
| `test_ride_backend_cli.py` | 11 | port ×10, drop ×1 | `reference.ride_backend_cli::<symbol>` ×10; `test_python_command_never_loads_the_extension` → drop (natively `python` is not a spellable backend — the single `native` choice; parse-level rejection asserted inside `test_default_backend_and_profile_timestep`) |

Totals: **19 port / 3 drop / 0 golden**; 26/26 collected nodes mapped;
zero `planned` records remaining in the batch.

## Golden-episode triage (per the 3.24b plan note)

The plan says the three golden-episode files "are covered by
`reference.golden_episode_tool` (3.03): triage each record against it
before writing new C++." Triage result: the actual native coverage for
all nine records is `native/tests/reference/
test_golden_episode_regressions.cpp` (nine `bt::run` functions matching
the pytest symbols one-to-one), registered at
`native/tests/reference/Phase3Ports.cmake:159` with labels
`t52 episode golden slow`. `reference.golden_episode_tool` is the
3.03 capture-tool contract (a different subject — the recorder/tool
itself, currently a documented baseline red pending recapture) and does
not cover these pytest records. No duplicate C++ was written.

The nine records are self-contained invariants/contracts (fresh
environments compared for deterministic equality, codec roundtrips,
independent `qfrc_applied` totals, v1/v2 legacy-artifact loading) — no
live-Python-oracle comparison exists, so no golden fixtures were
captured: disposition `port`, `golden_platforms` absent, no new files
under `tests/fixtures/golden/`.

## Native implementation: `reference.ride_backend_cli`

`native/tests/reference/test_ride_backend_cli.cpp` (new, ~390 lines,
10 check functions for the 10 ported records) drives the production
seams rather than monkeypatched stubs:

- Parse/resolve seam: `cli::parse_arguments` + `cli::resolve_ride` —
  the pair every executable `main()` reaches through `cli::dispatch` —
  for the backend/timestep/time-scale/legacy-mode/research-parser
  records.
- Real-binary seam: the produced `bike-ride`, `bike-research`,
  `bike-replay` executables of the same build are spawned through
  `bike::platform::run` (bounded argv boundary: no shell, concurrent
  stream drain, killpg cleanup, monotonic deadline) for the replay
  validation-ordering, resolve-preflight-ordering, missing-input
  diagnostic, and route-dispatch records. Binary paths arrive through
  the generated `bike_ride_cli_paths.hpp`
  (`$<TARGET_FILE:...>` inside `file(GENERATE)` — the
  `bike_native_bench_path.hpp` pattern, never a quoted compile
  definition). `add_dependencies` on the three executables; the test
  binary is a `bike_add_reference_test` `ENGINE` target with labels
  `t52 episode` at `Phase3Ports.cmake:339`.

Oracle→native deltas discharged at the assertions (documented in the TU
header comment):

- `--backend`: the oracle accepts `('python','native')`, default
  `python`; the native contract declares the single `native` choice, so
  `--backend python` is an invalid choice at parse — the stronger
  structural form of the oracle's backend assertions.
- `SystemExit(2)` ↔ `cli::ArgumentError` (a `std::invalid_argument`)
  at the parse/resolve seam and exit code 2 at the dispatch boundary.
- Missing-artifact surface: the oracle pins the Python-extension build
  directory (`BIKE_NATIVE_BUILD_PATH`, `tools/dev.py` repair commands,
  `python=`/`soabi=` tags); the native executable IS the artifact, so
  the ported contract asserts a missing required input file exits
  nonzero (exit 1 — the acquisition failure, not argparse's 2) naming
  the requested path.
- Route dispatch: viewer options verified through
  `cli::ride_viewer_options` (a real viewer is not spawned in a
  headless test environment); headless/research legs verified through
  real binaries — exit code admitted in `{0,2}` (both qualified
  platforms produce 2: the welded profile's constraint-work violation
  marks the run model-invalid; a clean run would be 0) with the
  emission contract (`"reason":"duration_reached"`,
  `"backend":"native"`, `"drive_mode":"articulated_effort"`,
  `"timestep_s":0.00125` physical / `"outcome":"duration"`,
  `"recording":"off"` research) pinning the route itself.
  `--rider lumped` keeps the standalone `bike-research` leg inside the
  unit-test budget; the dispatch contract is rider-independent.

Check counts: **29 checks**, identical on darwin-arm64 and
linux-aarch64 (`PASS 29 checks`, exit 0, direct binary invocation both
platforms). `reference.golden_episode_regressions`: **1095 checks**
identical on both platforms.

## Verification

- Oracle suite (all five files): `uv run --frozen --group native
  python -m pytest -q` — **26 passed** darwin-arm64 (11.60 s, prior
  session at `c981bd70`) and **26 passed** linux-aarch64 (13.04 s,
  container `bike-dev-pfc-lane-d`). `pytest --collect-only` reproduced
  the 26 node IDs recorded in the ledger, including the parameterized
  `[v1]/[v2]` and `[1]/[2]/[4]/[8]` expansions.
- Darwin (`build/native-release`, AppleClang, Release):
  `ctest -R 'reference\.(golden_episode_regressions|ride_backend_cli)'
  --output-on-failure` → **2/2 PASS** — `golden_episode_regressions`
  9.36 s, `ride_backend_cli` ~1.9 s (standalone 4.90 s).
- Linux aarch64 (`bike-dev-pfc-lane-d`, clang++-18, `/build/root-clang`
  Release): same selector → **2/2 PASS** —
  `golden_episode_regressions` 13.79 s, `ride_backend_cli` 1.78 s.
- Darwin L1 `ctest -L reference -j8`: my two tests pass; the sweep's
  reds are the documented-baseline set — `golden_episode_tool`
  (baseline `native recorder rows reproduce Python oracle`, 3.03
  capture pending), `pneumatic_tire_oracle` + `native_numeric_completion`
  + `native_pneumatic_tire` (documented fixture/settle baselines),
  `native_spindle_controller` (0.02 s fixture-content mismatch —
  stale `input.*` capture in this worktree, unrelated to this diff),
  plus 24 `Not Run` entries for executables this incremental build
  tree has never produced (`recording_*`, `replay_*`,
  `native_viewer_snapshot`, …). Final tallies reported when
  `reference.leg_loop` completes.
- Linux stale-binary hazard observed in the prior session (unknown
  `ArticulatedConfig` fields from a pre-weld CLI build) is retired:
  `/build/root-clang` `bike-ride`/`bike-research`/`bike-replay` were
  rebuilt before probing; both platforms now produce identical
  deterministic CLI behavior (exit 2 validation/preflight, exit 1
  missing input, exit {0,2} headless/research episodes with the
  documented warning `invalid reference step …
  ('energy.constraint_work',)` under `--check-policy diagnostic`,
  which is the model-invalid signature and does not abort the run).
- No `run_tests.sh` per lane constraint; no golden fixtures added;
  no pytest files edited (no capture sites needed).

## Ledger updates

`migration/test-port-map.json` — all 22 records updated in place:

- `collected_node_ids`: measured (26 total; `[v1]`/`[v2]` on
  `test_load_old_artifact_without_exit_total`, `[1]/[2]/[4]/[8]` on
  `test_explicit_scale_is_invalid_headless_even_one`).
- `assertions_capture`: `2026-10-28: measured — pytest collected N
  node(s); oracle green darwin-arm64 + linux-aarch64`.
- `native_test_id`: `reference.golden_episode_regressions::<symbol>`
  ×9, `reference.ride_backend_cli::<symbol>` ×10, `null` ×3 drops.
- `comparison_class`: pre-existing `invariant`/`contract` kept on the
  nine episode records; `contract` ×10 + `python-only` ×3 on the rest.
- `evidence`: dated lane/d strings; the stale "source disposition
  only" / "T01 assigns" placeholders are gone.

`migration/python-removal.json` — five rows → `replacement_landed`
with live `sha256`/`line_count` (files unchanged since triage except
`test_golden_episode.py` 37 lines, `test_episode_compare.py` 51,
`test_golden_episode_v2.py` 112, `test_ride_backend_cli.py` 155,
`test_divergence_horizon.py` 39); `verification_ids` =
`reference.golden_episode_regressions` ×3 and
`reference.ride_backend_cli`; `phase3_source_implementation` blocks
removed (landed schema). `test_divergence_horizon.py` is the first
all-drop file to reach `replacement_landed` — `action
delete_with_python` and `replacement_paths []` stand; the generic
verification IDs remain because no CTest node exists for it (its
deletion prerequisite is the qualified drop, recorded in the
port-map). `snapshot_*` fields untouched everywhere.

## Counters

- Records resolved: 22 (19 port, 3 drop, 0 golden, 0 planned).
- Collected nodes covered: 26/26.
- New native test targets: 1 (`reference.ride_backend_cli`, 29
  checks); requalified: 1 (`reference.golden_episode_regressions`,
  1095 checks).
- Golden fixtures added: none (no `golden` records this batch).
- Ledger rows: 22 port-map rows rewritten; 5 removal rows
  `replacement_landed` (`replacement_landed` total 112 → 117).

## Remaining / caveats

- `reference.golden_episode_tool` baseline red is unchanged and out of
  scope (3.03 capture task, not this batch's records).
- `native_spindle_controller` red observed during L1 is a
  worktree fixture mismatch, not a batch regression (no diff touches
  spindle code or fixtures); flagged for the integration lane.
- The 24 `Not Run` reference targets are unbuilt executables in this
  lane's incremental build tree — not a code failure; the
  integration gate builds the full graph.
- Deletion of the five pytest files stays gated at T63 per the
  removal ledger; the oracle suite remains green on both platforms.
