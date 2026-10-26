# Task C318 — test-port 3.18 report (drivetrain core, `drive`)

**Batch**: 3.18 — 69 records / four source files, owner `T52`, labels
`t52 drive`. CTest ids `reference.native_drivetrain`,
`reference.native_drive_policies`,
`reference.drivetrain_policy_clone`,
`reference.motor_freewheel_topology` — all four already registered in
`native/tests/reference/Phase3Ports.cmake` (once each, verified; no
duplicate `bike_add_reference_test` additions).

**Lane**: `.worktrees/pfc-lane-c`, branch `lane/c`, base `c91ae881`.
No merge, no push, no subagents.

## Status: landed, all 69 records green

51 golden records carry both-platform fixtures (95 + 165 node pairs
per platform, identical node sets); all four CTest ids pass with
platform-identical check counts; the four `python-removal.json` rows
are `replacement_landed` with the landed CTest ids; deletion remains
gated to T63 per ADR-0003.

## Triage (69 records)

| File | port | golden | drop | planned |
|---|---|---|---|---|
| `test_native_drivetrain.py` | 4 | 24 | 0 | 0 |
| `test_native_drive_policies.py` | 4 | 27 | 0 | 0 |
| `test_drivetrain_policy_clone.py` | 1 | 0 | 0 | 0 |
| `test_motor_freewheel_topology.py` | 9 | 0 | 0 | 0 |
| **total** | **18** | **51** | **0** | **0** |

No drops, no blocked records, no expected-fail ports.

## Freewheel reconciliation (pre-work gate)

`reference.motor_freewheel_topology` was already registered
(`Phase3Ports.cmake:139`) and `test_motor_freewheel_topology.cpp`
already existed when the batch started, so the nine records were
reconciled **before** any new work: each source symbol maps 1:1 to an
existing C++ function in that TU (rotor/crank freewheel drive paths,
engaged/disengaged topology, mutual-exclusion validation,
invalid-config probes). Dispositions were already `port` with the
correct `native_source`; no second test file was created. This batch's
contribution to those rows is runtime qualification only — collected
node ids and a dated verification-evidence line were filled in the
ledger; disposition and `native_source` untouched.

## Pre-landed sources, absent qualification

All four native TUs were already present in the tree at batch start;
the 69 ledger rows carried only `2026-10-21: source disposition only;
no build, runtime collection, oracle acquisition or CTest qualification
performed` and empty `collected_node_ids`. `native_drive_policies` and
`native_drivetrain` were also on the documented missing-fixture red
list (`FAIL fixture is not a regular file`) — this batch supplies the
missing goldens and the runtime proof.

## Implementation changes (two real defects found by the oracle runs)

1. **`native/src/drivetrain/shifting.cpp` — multi-cog throw selection
   mirrored from the oracle** (`5bbea70a`). Commit `b21c63ec` changed
   `src/bike_sim/physics/shifting.py` to pick the smallest upshift cog
   (largest downshift cog) whose landing cadence stays inside the
   EMA bounds, falling back to the extreme candidate — the native side
   still picked the one-step neighbour. Detected as 11 deterministic
   golden mismatches on the first macOS pytest run
   (`required_cadence_rpm` 147.45 vs 70.22, `gear_ratio` 1.619 vs
   2.125, `ideal_hub.boundary` 0.5291 vs 0.4674). The mirror preserves
   slip gating, landing validation, cooldown/cut durations and the
   atomic `next`-snapshot commit; cassette sort order verified in both
   validators before relying on iteration direction.
2. **`native/tests/reference/drivetrain_reference_support.hpp` —
   state-tree leaves dispatch on the recorded form** (`45924d62`).
   `encode_drive_snapshot`'s `ideal_hub.range` is a plain JSON list in
   a wire state (`assert_tree` records `#length` + `[i]` children) but
   an ndarray in `python_state` (one `<f8` entry); `coefficients` and
   `prepared.jacobian/qpos` are ndarrays on both sides. The old
   path-name special case collapsed `range` unconditionally and lost
   sync at entry 100 of `test_atomic_state_inputs_and_probes`
   (`label expected '$.ideal_hub.range#length', received
   '$.ideal_hub.range'`). `peek_label() == path + "#length"` now
   selects list vs array consumption per recorded entry — no path-name
   heuristics remain.

No other code changes; the two Python oracle files were not touched
(`assert_bitwise_equal`/`assert_tree` auto-record every needed leaf —
no `_golden.record` additions required).

## Golden fixtures

| File | nodes | darwin-arm64 | linux-aarch64 |
|---|---|---|---|
| `test_native_drive_policies.py` | 95 | 95 `.json`+`.bin` pairs, 27.4 MiB | 95 pairs, ~28 MiB |
| `test_native_drivetrain.py` | 165 | 165 `.json`+`.bin` pairs, 54 MiB | 165 pairs, ~53 MiB |

Node sets are identical across platforms for both files (verified
record-level); linux sidecars pulled via `container.sh pull`.

Capture via `BIKE_GOLDEN_CAPTURE=tests/fixtures/golden` on the full
green files (darwin: `pytest -q` → **102 passed** policies, **175
passed** drivetrain; linux-aarch64 same counts inside
`bike-dev-pfc-lane-c`). The no-capture nodes are exactly the `port`
contract records: 7 in policies (`test_invalid_human_torque` ×4,
`test_native_policy_api_exists`,
`test_projection_normalizes_numeric_leaves`,
`test_projection_resolves_runtime`) and 10 in drivetrain
(`test_invalid_config` ×6,
`test_invalid_elastic_topology_rejects` ×2, `test_no_drive`,
`test_real_solved_request_violation`) — verified against collected
node ids; zero orphans, zero foreign nodes in either fixture dir.

## Test matrix

| Test | darwin-arm64 (`build/native-release`) | linux-aarch64 (`/build/root-clang`, clang++-18) |
|---|---|---|
| `reference.native_drivetrain` | PASS 1079181 checks | PASS 1079181 checks |
| `reference.native_drive_policies` | PASS 599289 checks | PASS 599289 checks |
| `reference.drivetrain_policy_clone` | PASS 17 checks | PASS 17 checks |
| `reference.motor_freewheel_topology` | PASS 297 checks | PASS 297 checks |
| oracle pytest `test_native_drivetrain.py` | 175 passed | 175 passed |
| oracle pytest `test_native_drive_policies.py` | 102 passed | 102 passed (capture run) |
| oracle pytest clone+freewheel | 14 passed, 1 skipped | 14 passed, 1 skipped |

Check counts are identical across platforms for all completed rows —
the bitwise comparators consume the same per-node entry counts on both
sides.

Equilibrium-cache note: the parity claims above are **cold-cache**
results — each CTest binary builds and settles its own models; no
canonical construction-settle key was warmed by a prior episode, and
no post-episode re-settle poisons the canonical cache.

## Byte-flip proof

`tests/fixtures/golden/darwin-arm64/
tests_reference_test_native_drivetrain.py/
test_atomic_state_inputs_and_probes.bin`, byte 1024 flipped to `0x01`
→ `FAIL tests/reference/test_native_drivetrain.py::
test_atomic_state_inputs_and_probes comparison 5 label '$#keys':
bytes differ` (names node, comparison index and label). Restored via
`git checkout` — md5 `9b239af1024485b47014b46e32f79f14` before and
after — rerun `PASS 1079181 checks`.

## Commits on `lane/c` (base `c91ae881`)

- `5bbea70a` — fix(native): mirror multi-cog shift throw selection
  from the oracle
- `2edbd7cb` — test(golden): capture drive_policies oracle fixtures on
  darwin-arm64
- `45924d62` — fix(native-test): dispatch drivetrain state-tree leaves
  on the recorded form
- `52ad66c1` — test(golden): capture drivetrain oracle fixtures on
  darwin-arm64
- `353caca9` — test(golden): capture drivetrain oracle fixtures on
  linux-aarch64 + draft C318 report
- `961ecde7` — test(golden): capture drive_policies oracle fixtures on
  linux-aarch64
- `c6546a5b` — docs(migration): qualify batch 3.18 drivetrain records
  on both platforms
- `205e1361` — docs(migration): 3.18 drivetrain-core evidence section

## Baseline reds (documented, not caused by this batch)

From the `ctest -L reference` sweeps recorded in
`migration/cpp-linux-completion-evidence.md` on both platforms:

- `native_static_brake`, `pneumatic_tire_oracle`,
  `native_clock_contracts` — `FAIL fixture is not a regular file`
  (other batches' missing goldens; untouched here).
- `native_drive_policies`, `native_drivetrain` — were the same
  missing-fixture signature before this batch; **cleared by these
  fixtures on both platforms**.
- `golden_episode_tool` — `FAIL native recorder rows reproduce Python
  oracle` (pre-existing recorder drift).
- `migration_T09` (integration vector bitwise), `migration_T29`
  (`weld_servo_gain: unknown key`), `migration_T56` (shellcheck-0.11.0
  env drift) — plus the ten additional linux-only migration reds
  (`T03`, `T14`, `T15`, `T16`, `T18`, `T20`, `T31`, `T32`) from the
  documented ±1-ulp / darwin-pinned-fixture family.

## Scope limitations

- `bash tools/run_tests.sh native` not run — the task requested
  bounded L2 only; the outer gate owns the sweeps.
- Fixture sizes are above the nominal 20 MiB guideline
  (27.4 MiB + 54 MiB on darwin) — consistent with prior large batches
  (3.22a) and required by the recorded-leaf contract.
