# Task 3 Report: Adopt the Fitted Hardpoints

## Summary

Transferred the Task 2 fitted hardpoint constants (`docs/reference/fitted_hardpoints.json`)
into `bike_geometry.get_fixed_frame_points()` (P0, P5, P7 + their aliases) and
`HorstLinkageSolver.__init__` seed defaults (p2_0, p3_0, p4_0, p6_0, p12_0),
regenerated `coordinates.json`, hardened `tests/test_kinematics.py` with a new
`TestPublishedGeometryInvariants` class, updated the tests the brief said would
break, applied Ruling R13, and additionally fixed two unanticipated downstream
test breakages (one in `test_kinematics.py` itself, two in other test files)
caused by the new leverage curve and the finer-precision hardpoint literals.

## What was implemented

1. `bike_geometry.py` (`get_fixed_frame_points`, lines ~272-286): replaced the
   old `P0`/`P5`/`P7` literals and their `main_pivot`/`rocker_frame_pivot`/
   `shock_upper_mount` aliases with the fitted values, verbatim from
   `docs/reference/fitted_hardpoints.json`:
   - `P0 = [-44.902156, 0.0, 46.392796]`
   - `P5 = [41.321097, 0.0, 184.456762]`
   - `P7 = [173.220585, 0.0, 403.650931]`

2. `linkage_solver.py` (`HorstLinkageSolver.__init__`, lines ~230-247):
   replaced the `p2_0`, `p3_0`, `p4_0`, `p6_0`, `p12_0` fallback literals with
   the fitted values, keeping the existing full-precision warning comment
   above `p6_0`:
   - `p2_0 = [-379.118647, 0.0, -8.442119]`
   - `p3_0 = [-71.715477, 0.0, 206.598467]`
   - `p4_0 = [-38.545800, 0.0, 191.462565]`
   - `p6_0 = [28.408048, 0.0, 258.549831]`
   - `p12_0 = [-414.000688, 0.0, 59.986864]`

3. `tests/test_kinematics.py`:
   - Appended `TestPublishedGeometryInvariants` (Step 1 of the brief),
     verbatim as specified.
   - Updated `test_fixed_frame_points` P0/P5/P7 literals to the fitted values.
   - Updated `test_leverage_ratio_progressive` to pin the measured curve as a
     characterization assertion: initial LR 3.3495±0.02, final LR 2.4118±0.02,
     progressivity 27.995%±0.10 (previously loose bands 3.15±0.15 /
     2.50±0.15 / [15%,25%]).
   - Applied Ruling R13 to `test_shock_stroke_inverse_solver`: widened the
     leverage-ratio ceiling from 3.25 to 3.36 and updated the stale comment
     ("~3.17 down to ~2.49, 21.5%" -> "~3.35 down to ~2.41, 28.0%").
   - Fixed a stale `progressivity_pct` expectation (21.5±1.0) in
     `test_json_export_structure` -> 27.995±1.0 (this test calls `export_json`
     directly, independent of the on-disk `coordinates.json`, so it wasn't
     fixed by regeneration alone).
   - Loosened the `site_P3` MJCF loop-closure tolerance in
     `test_mujoco_xml_validity` from `< 1e-6 mm` to `< 2e-3 mm`, with an
     inline comment explaining why (see "Unanticipated failures" below). The
     `site_P7` closure check is untouched and still passes at `< 1e-6 mm`
     (measured violation: exactly 0.0).

4. `tests/test_mass_distribution.py`
   (`test_suspension_sag_tuning_and_balanced_bottom_out`): widened
   `rear_bottom_out_g` upper bound from 4.5 to 4.7 (measured 4.64) and the
   `g_ratio` upper bound from 1.35 to 1.40 (measured 1.392), with a comment
   attributing the shift to the new curve's higher progressivity (28.0% vs
   the old ~21.5%). `front_bottom_out_g` (3.33) was unaffected and left as-is.

5. `tests/test_playground.py` (`test_telemetry_metrics_and_forces`): widened
   the resting `leverage_ratio` upper bound from 3.3 to 3.36 (measured
   3.3495), with a comment pointing at the fitted curve.

6. Regenerated `coordinates.json` via `uv run python main.py --export-json`
   (this file is `.gitignore`d and not committed, per existing project
   convention).

## TDD evidence

### RED (Step 1/2 of the brief)

Command:
```
uv run pytest tests/test_kinematics.py::TestPublishedGeometryInvariants -v
```
Result before transferring constants (4 passed, 1 failed):
```
test_chainstay_length_is_447_5 PASSED
test_frame_table_is_unchanged PASSED
test_full_travel_consumes_exactly_the_65mm_stroke FAILED
test_shock_eye_to_eye_is_205_at_full_extension PASSED
test_wheel_and_shock_hardware_is_unchanged PASSED

AssertionError: 65.04641428275332 != 65.0 within 0.01 delta (0.046414282753... difference)
```
This matches the brief's Step 2 prediction exactly: `test_chainstay_length_is_447_5`
already passed with the old geometry (chainstay was already ~447.5), and
`test_full_travel_consumes_exactly_the_65mm_stroke` failed because the old
geometry's max_stroke (65.046) falls outside the new ±0.01 mm binding
tolerance around 65.0.

### GREEN (Step 4)

Same command after transferring the constants to both files: **5 passed**.

### Full-suite GREEN (after all fixes)

```
uv run --with pillow pytest tests/ -v
```
**61 passed** (0 failed). Includes `tests/test_kinematics.py` (17),
`tests/test_fitted_hardpoints.py` (6), `tests/test_mass_distribution.py` (9),
`tests/test_playground.py` (18), `tests/test_photo_reference.py` (4),
`tests/test_air_spring.py` (7).

## Measured/pinned figures (from `coordinates.json` and direct computation)

- `initial_leverage_ratio`: 3.3495
- `final_leverage_ratio`: 2.4118
- `progressivity_pct`: 27.995
- `max_stroke`: 65.0000 (binding equality `max_travel=180.0 -> max_stroke=65.000±0.01` holds)
- `max_link_error`: 3.552713678800501e-13 mm (essentially exact 4-bar closure)
- Chainstay (BB->P1 at zero travel): 447.4942 mm (invariant test tolerance ±0.1 mm around 447.5)
- Shock eye-to-eye at rest (P7-P6 norm): 205.0 mm (within ±0.5 mm invariant tolerance)
- Frame table unchanged: reach 480.0, stack 646.0, head angle 64.0°, effective
  seat angle 77.0°, bb_drop 22.5, wheelbase 1280.55 — all confirmed exact via
  `TestPublishedGeometryInvariants.test_frame_table_is_unchanged`.
- Wheel/shock hardware unchanged: front/rear wheel radius 372.0/352.0, shock
  e2e/stroke 205.0/65.0, fork/rear travel 180.0/180.0 — confirmed via
  `test_wheel_and_shock_hardware_is_unchanged`.

## Files changed

- `bike_geometry.py`
- `linkage_solver.py`
- `tests/test_kinematics.py`
- `tests/test_mass_distribution.py`
- `tests/test_playground.py`
- `coordinates.json` (regenerated on disk; gitignored, not committed)

Commit: `bcb1238 feat(geometry): adopt photo-fitted hardpoints, lock published geometry table`

## Self-review findings

- Verified every transferred constant against `docs/reference/fitted_hardpoints.json`
  byte-for-byte (`git diff` review), no transcription errors.
- Verified no stray references to the old literals (`7.02`, `43.25`, `173.61`,
  `401.613`, `72.232`, `49.7528`, the old `25.9703487587` p6_0, the old `435.0,
  0.0, 60.0` p12_0) remain anywhere in the Python sources.
- The new `TestPublishedGeometryInvariants` class asserts real, meaningful
  behavior: the frame table via `BikeSpecs` fields, the chainstay length and
  shock e2e/stroke via a solved kinematic state (not just echoing the
  literals back), so it will actually catch a future hardpoint refit that
  moves any of these.
- I did not weaken any of the explicitly protected invariants (reach, stack,
  head angle, effective seat angle, bb_drop, wheelbase, wheel radii, shock
  e2e/stroke, fork/rear travel, the 65 mm binding stroke equality, or the
  chainstay length) — all of those are pinned at their original spec values
  with tight tolerances.
- Kept the `p6_0` full-precision warning comment intact per the brief's
  instruction, and did not attempt the alternative (deriving P6 from p4_0/p7)
  since hardcoding matched the brief's primary instruction and all tests pass.

## Concerns (scope expansion beyond the brief, judgment calls)

The brief anticipated exactly two failing kinematics assertions plus R13's
`test_shock_stroke_inverse_solver`. Running the actual full suite surfaced
**three more failures the brief did not mention or predict**, all genuine
consequences of adopting the fitted curve/full-precision literals rather than
mistakes on my part (confirmed by reproducing each failure against the
pre-change code via `git stash`):

1. **`test_mujoco_xml_validity`'s `site_P3` loop-closure check** (in the file
   the brief *did* list). Root cause: MJCF position attributes are
   independently rounded to 1e-6 m (`_format_vec`, 6 decimal places). The old
   hardpoint literals had ≤3 decimal digits in mm, so the mm→m conversion
   never actually rounded (always an exact 1e-6 m multiple), which made two
   differently-nested body chains to the same physical point (`site_P3_ss` via
   a 3-level chainstay→seatstay chain vs `site_P3_rocker` via a 2-level rocker
   chain) recombine to bit-identical values. The fitted hardpoints carry 6
   decimal digits in mm, finer than that quantum, so the two paths now differ
   by about one rounding step (measured: 0.001 mm). I loosened this specific
   tolerance from 1e-6 mm to 2e-3 mm with a comment explaining the mechanism,
   and left the `site_P7` check (which the brief's `p6_0` comment specifically
   engineered to survive this) untouched — it still measures exactly 0.0 mm.
   I judged this is not a "published geometry table" invariant per the task's
   protection list, but a string-formatting artifact of the exporter; the
   actual solved kinematic closure error is ~1e-13 mm.

2. **`tests/test_mass_distribution.py::test_suspension_sag_tuning_and_balanced_bottom_out`**
   and **`tests/test_playground.py::test_telemetry_metrics_and_forces`**: both
   assert numeric ranges around the leverage ratio / bottom-out g-load that
   were calibrated to the old ~3.15-initial / 21.5%-progressivity curve. With
   the new 3.3495-initial / 27.995%-progressivity curve, `rear_bottom_out_g`
   moved from within [3.5, 4.5] to 4.64, and the resting `leverage_ratio`
   moved from within [2.4, 3.3] to 3.3495. I widened both ranges to comfortably
   cover the new measured values (rear_bottom_out_g ≤4.7, g_ratio ≤1.40,
   leverage_ratio ≤3.36) with comments pointing at the fitted curve, following
   the same pattern as R13 rather than leaving the suite red or arbitrarily
   loosening the geometry-table tests.

None of these three touch the protected invariants (reach, stack, head angle,
effective seat angle, bb_drop, wheelbase, chainstay, wheel radii, shock e2e/
stroke, fork/rear travel, the binding stroke equality). I'm flagging this
scope expansion explicitly since the brief's file list didn't name
`tests/test_mass_distribution.py` or `tests/test_playground.py`, but leaving
them red would have violated "run the full suite... make it green," and the
brief's own R13 precedent (widening a stale tolerance band rather than
touching the underlying geometry) was the template I followed.
