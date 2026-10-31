# task-ft3-recap — Darwin golden recapture after the FT-3/FT-4 physics merge

2026-11-01, lane/c4 at `b9d53842` (`merge: lane/e3 4.02 run_tests.sh native-only gate`),
darwin-arm64, `build/native-release` (AppleClang, Accelerate). Scope: refresh only the
darwin-arm64 golden fixtures whose drift is fully explained by the FT-3/FT-4 pedelec
delivery fix (`352375e3`); no linux-aarch64 fixtures, no documented-exception families.

## Why the goldens were legitimately stale

FT-3 (`96dad67c`, `a0e35355`, `60b167ae`, merged as `352375e3`, reviewed mirror-EXACT)
changed the drivetrain contract:

- the ideal mid-drive hub now publishes the physically **applied** ratio, which sweeps
  linearly from the previous gear toward the committed one while `cut_remaining_s`
  decays — `staged = applied + (gear − applied) * dt / (cut_remaining + dt)`
  (`src/bike_sim/sim/ride/drivetrain_forces.py:239-246`);
- required cadence uses the applied ratio; cadence wish = `min(required, cadence_ema)`;
- rider-plant delivery/unload and servo authority follow the new wish path.

Every trajectory touching a cassette shift therefore diverges after the commit step —
exactly the signature the four reds showed.

## Baseline reds → resolution

| CTest | Red signature | Resolution |
|---|---|---|
| `reference.native_atomicity` | `FAIL committed shift lands the 34/28 ratio atomically` | **Contract update**, not a golden: stale pre-FT-3 expectation in both mirrors. No fixture involved (self-contained contract test). |
| `reference.native_runtime_commands` | `interval 120` element 1 `0x400023fa92d10254` vs `0x4000213df9babd1e` | Recaptured `test_gear_shift_and_coast_resume_match_python` (.json+.bin). |
| `reference.native_runtime_rollout` | `row.00000120.values` element 82 | Recaptured `test_one_second_800_interval_golden_episode_all_channels` (.json+.bin; `.json` index shifts because `.other` JSON payload lengths changed with flag flips). |
| `reference.native_research_environment` | `transition.00000087.values` element 7 | Recaptured the two trajectory nodes; **plus** a port-side provenance-seam extension for `test_reset_seed_generation_snapshot_and_facade_ownership` (see below). |

Prior `366914f4` families confirmed green and left untouched:
`native_numeric_completion`, `native_drive_policies`, `pneumatic_tire_oracle`,
`period_buffer`, `native_drivetrain`.

## Per-test diff explanation (all FT-3/FT-4-scoped)

**runtime_commands** — 3 recorded intervals; `interval 60`/`80` bit-identical,
`interval 120` differs in 99/292 integration-state elements starting at index 1.
The cassette shift commits inside the (80,120] torque-resume segment, so divergence
onset is exactly the FT-3 sweep window.

**runtime_rollout** — first and sole divergence at `row.00000120.values` element 82
= `.drive.crank_target_rate_rad_s` (`0.00017151942299346864` →
`1.297983946887864e-05`; live oracle reproduces the new value bitwise). Single
differing element per row until ~row 700, where the delivery tail broadens:
`.other` engagement flags (`motor_enabled`, `assist_demand_gated`,
`motor_freewheel_engaged`, `freehub_engaged`) and rider `would_slip`/`would_separate`
flip, and `history.work_j`, `history.airtime_s`, the final `golden episode
integration state` inherit the changed trajectory. All downstream of the wish/delivery
change — no unexplained field.

**research_environment** (both `one_e_minus_nine` and `complete_episode` nodes share
the same recipe/signature) — first diff `transition.00000087.values` element 7 =
`.observation.specific_force_body_mps2[0]`. Transitions 87–99, integration states at
steps 704–800, `episode_metrics` (4/24), and `recorder.columns` (2776/35880) all
follow; `.drive.crank_target_rate_rad_s` is the principal direct channel (34 rows).
The recorded `metadata.other` provenance (`model_commit`/`model_source_sha256`) is
inert — the port re-seats recorded values verbatim.

## Fixes beyond fixture bytes

1. **`tests/reference/test_native_atomicity.py:89` + twin
   `native/tests/reference/test_native_atomicity.cpp:147`** — the committed-shift
   ratio assertion was pre-FT-3. Kind-aware now: `geometric_ideal_mid_drive` still
   lands `34./28.` atomically (integer-sprocket exception, unchanged); ideal
   mid-drive publishes the first sweep step
   `34./24. + (34./28. - 34./24.) * .1 / (.2 + .1) = 1.3492063492063493`
   (bitwise-verified against the live oracle; `.2` = `ShiftingConfig.
   shift_cut_duration_s` default). `rear_teeth`, `shift_pending`, policy snapshot,
   `shift_time_s` all unchanged — verified live.

2. **`native/tests/reference/test_native_research_environment.cpp`** — the
   documented `metadata.other` provenance seam (re-seat `model_commit`/
   `model_source_sha256` verbatim, erase native-only sections) is extended to
   `resolved_config.physics.articulated.joint_{envelope,strength}_path`. Those
   leaves record absolute **worktree** paths, so the committed golden was
   latent-red on every checkout whose path differs from the capture lane
   (`pfc-lane-f`) — masked until now by the earlier transition failure. Asset
   identity stays bitwise-gated via the `joint_*_sha256` leaves. No fixture
   change; the golden becomes worktree-agnostic like the rest of the seam.

## Parity proven before capture

- `test_native_runtime_commands.py`: 10/10 under `_pin_oracle_kernels`
- `test_native_runtime_rollout.py`: 10/10
- `test_native_research_environment.py`: 21/21 (8 expected `energy.constraint_work`
  invalid-reference-step warnings)
- `test_native_atomicity.py`: 40/40 after the contract fix

Capture mechanism: `BIKE_GOLDEN_CAPTURE=tests/fixtures/golden` + the targeted pytest
nodes only (each node owns one `.json+.bin` pair; untouched nodes untouched).

## Recaptured fixtures (darwin-arm64 only)

- `tests_reference_test_native_runtime_commands.py/test_gear_shift_and_coast_resume_match_python` {.json,.bin}
- `tests_reference_test_native_runtime_rollout.py/test_one_second_800_interval_golden_episode_all_channels` {.json,.bin}
- `tests_reference_test_native_research_environment.py/test_one_second_pair_with_one_e_minus_nine_gate` {.json,.bin}
- `tests_reference_test_native_research_environment.py/test_one_second_pair_complete_episode_gate` {.json,.bin}

## Verification

- Affected ctests: `native_atomicity`, `native_runtime_commands`,
  `native_runtime_rollout`, `native_research_environment` — **4/4 green**.
- Corruption-negative: last-byte flip of the recaptured commands `.bin` →
  `reference.native_runtime_commands` red; restore → green.
- `ctest -L golden`: **35/37** — the only failures are the two documented
  exceptions (`reference.golden_episode_tool` FT-5 `pinned_savage_40`;
  `reference.native_pneumatic_tire` construction-path parity gap).
- `ctest -L reference`: **159/161** in 2296.8s — the only failures the same two
  documented exceptions (`golden_episode_tool`, `native_pneumatic_tire`); all
  four previously-red tests pass in-sweep (#115 atomicity 0.11s, #170
  runtime_commands 6.55s, #174 runtime_rollout 1.98s, #201
  research_environment 5.34s).

## Ledger

`migration/test-port-map.json` evidence appended (no disposition changes) for:
`test_gear_shift_and_coast_resume_match_python`,
`test_one_second_800_interval_golden_episode_all_channels`,
`test_one_second_pair_with_one_e_minus_nine_gate`,
`test_one_second_pair_complete_episode_gate` (recaptures);
`test_reset_seed_generation_snapshot_and_facade_ownership` (seam extension);
`test_valid_shift_commits_atomically` (contract update).

## Deferred / concerns

- **linux-aarch64**: the same four fixtures are stale on linux too; intentionally
  untouched — deferred to the Linux wave as before.
- The C++ test-source edits (`native_atomicity.cpp`, `test_native_research_
  environment.cpp`) are native-test changes; `bash tools/run_tests.sh native`
  is the covering outer profile if the integration lane wants it.
- No unexplained drift was observed in any inspected fixture; nothing else was
  recaptured.
