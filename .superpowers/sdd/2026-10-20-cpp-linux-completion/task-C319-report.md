# Task C319 — test-port 3.19 report (drive policies and pedalling, `drive`)

**Batch**: 3.19 — 60 records / six source files, owner `T52`, label
`t52 drive`. CTest ids `reference.motor_profile`,
`reference.cadence_shifter`, `reference.pedal_waveform`,
`reference.pedaling_policy`, `reference.assist_controller`,
`reference.system_momentum_drive` — all six already registered in
`native/tests/reference/Phase3Ports.cmake` (lines 115–124; the first
four via the `foreach(_drive_port ...)` block, the last two as
explicit `bike_add_reference_test` entries).

**Lane**: `.worktrees/pfc-lane-c`, branch `lane/c`, base `8fb88439`.
Qualification only — the C++ TUs were landed by an earlier wave; no
new porting was needed.

## Status: qualified, all 60 records green

All 60 records are `disposition: port`; all 60 `native_test_id`s
resolve to a real `void test_*()` function defined and invoked in the
named TU; per-TU function counts equal record counts exactly
(22/9/8/10/9/2). Zero dangling ids, zero missing tests, zero ports
written.

## Triage (60 records)

| File | port | golden | drop | planned |
|---|---|---|---|---|
| `test_motor_profile.py` | 8 | 0 | 0 | 0 |
| `test_assist_controller.py` | 22 | 0 | 0 | 0 |
| `test_cadence_shifter.py` | 9 | 0 | 0 | 0 |
| `test_pedal_waveform.py` | 10 | 0 | 0 | 0 |
| `test_pedaling_policy.py` | 9 | 0 | 0 | 0 |
| `test_system_momentum_drive.py` | 2 | 0 | 0 | 0 |
| **total** | **60** | **0** | **0** | **0** |

## Assertion correspondence — full-depth review

Sampled beyond the required depth: both smallest files entire
(`system_momentum_drive` 2/2, `motor_profile` 8/8) and 100% of each
larger file (cadence 9/9 — required anyway for the 3.18-fix-round
reconcile —, pedal_waveform 10/10, pedaling_policy 9/9,
assist_controller 22/22). Every C++ function asserts the same
contract as its Python record; signature details verified against
headers where positional mapping mattered
(`drivetrain/motor.hpp:81` `step(human, cadence, speed, braking, dt,
request, shaft_rpm)`; `drivetrain/shifting.hpp:100` `update(cadence,
required, dt, pedaling, braking, rear_in_contact, rear_slip_mps)`).

Documented adaptations (no weakening): Python-only mechanisms are
asserted at the native boundary — `pytest.raises(TypeError)` on
removed kwargs → wire-decoder rejection of injected keys;
`FrozenInstanceError`/item-assignment → `const` registry + owned
value data (`is_const_v` checked at compile time); `pedelec_cap`'s
`math.inf` permission sentinel → delivered-torque observation (34 Nm
uncapped request); `copy.deepcopy` probe → value copy with
independence asserted and the `is`-identity adaptation documented;
`braking=1` non-bool rejection → inline-commented drop (Python
marshalling only). `near()` carries pytest.approx's default
rel/abs (1e-6 / 1e-12) unless the source declared another bound.

## Test matrix

| Test | darwin-arm64 (`build/native-release`) | linux-aarch64 (`/build/root-clang`, clang++-18) |
|---|---|---|
| `reference.motor_profile` | PASS 60 checks | PASS 60 checks |
| `reference.cadence_shifter` | PASS 74 checks | PASS 74 checks |
| `reference.pedal_waveform` | PASS 87 checks | PASS 87 checks |
| `reference.pedaling_policy` | PASS 3177 checks | PASS 3177 checks |
| `reference.assist_controller` | PASS 72 checks | PASS 72 checks |
| `reference.system_momentum_drive` | PASS 32 checks | PASS 32 checks |
| oracle pytest, all six files | 101 passed | 101 passed |

Check counts are platform-identical on every target. The container
run went through `container.sh sync` → `build root-clang` (100
steps; five of the six binaries built fresh — `cadence_shifter`
already existed from the 3.18 fix round) → `ctest root-clang -R …`.

All 60 `line`/`end_line` ranges re-verified drift-free via `ast`
against the live sources — no refresh needed on any record.

## Ledger

- `migration/test-port-map.json`: 51 unqualified rows gained measured
  `collected_node_ids` (per-file `pytest --collect-only`; totals
  32/27/9/22/9/2 = 101, matching the oracle run count), dated
  `2026-10-27` `assertions_capture` with measured node counts, and a
  dated evidence line naming the verified C++ function plus the
  platform-identical check counts. The 9 cadence rows were already
  complete from the 3.18 fix round — left untouched per instructions.
- `migration/python-removal.json`: the five not-yet-landed rows →
  `replacement_landed`, `verification_ids` = the single landed CTest
  id (per the 3.18 convention), `phase3_source_implementation`
  blocks dropped, snapshot hashes retained; `sha256`/`line_count`
  confirmed current (files unchanged since snapshot — the refresh
  was a no-op). `test_cadence_shifter.py` was already landed;
  re-verified current (88 lines), unchanged. Deletion stays gated to
  T63 per ADR-0003.
- `migration/cpp-linux-completion-evidence.md`:
  `## 2026-10-27 — 3.19 qualification (lane/c)` section appended.

## Commits on `lane/c` (base `8fb88439`)

- `5dc604c7` — docs(migration): qualify batch 3.19
  drive-policy/pedalling records on both platforms
- (this report + evidence section — see subsequent commit)

## Scope limitations

- `bash tools/run_tests.sh native` not run — the task bounded
  qualification to the six targets + six oracle files; the outer
  gate owns the sweeps.
- No golden fixtures: all 60 records are analytic/contract ports —
  no sidecar capture applies to this batch.
- Unrelated baseline reds (`migration_T*` family, other batches'
  missing-fixture signatures) were not re-enumerated; nothing in
  scope was red.
