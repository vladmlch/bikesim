# Task 1: MJCF `ride` mode — Report

## Summary

Implemented the fourth MJCF simulation mode, `"ride"`, per
`.superpowers/sdd/2026-08-25-ride-mode/task-1-brief.md` and `docs/RIDE.md`
sections 0-3 and 10. In ride mode the bike is a planar (sagittal-plane)
free-rolling multibody over a fixed heightfield road, instead of being welded
to the world as in `standard`/`stand`/`playground`. Every change is gated on
`mode == "ride"`; the three pre-existing golden baselines are byte-for-byte
untouched.

## What was implemented

- **`src/bike_sim/mujoco/terrain.py`** (new): `build_terrain(root, worldbody,
  ground_z_m, spec=FIELD)` emits the `<hfield name="road">` asset (no
  elevation data — zeroed, filled from Python later) using
  `FIELD.nrow/ncol/size_attr`, and the `terrain` hfield geom at
  `(spec.geom_x_m(), 0, spec.geom_z_m(ground_z_m))` with `condim="3"`,
  `friction="1.2 0.005 0.0001"`, `solref="-130000 -800"`, `material="mat_floor"`.
- **`src/bike_sim/mujoco/environment.py`**: ride mode emits the two lights and
  a single `catch_plane` at `FIELD.catch_plane_z_m(ground_z_m)` (material
  `mat_floor` only — no explicit contact params, i.e. MuJoCo defaults), then
  returns early — no floor plane, no stand fixtures. Other modes' code path is
  byte-identical.
- **`src/bike_sim/mujoco/frame.py`**: for ride, the three planar root joints
  (`root_x` slide +X, `root_z` slide +Z, `root_pitch` hinge +Y, in that order)
  are added to the `frame` body immediately after its creation, before
  `build_rider` and before any geometry. `build_rider(...)` is now also called
  for ride (rider on by default there).
- **`src/bike_sim/mujoco/steering_fork.py`**: `"ride"` added to the
  steer-lock mode tuple. `site_handlebar` added at `stem_top` for ride only.
  Front tyre cylinder gets `contype="0" conaffinity="0"` in ride; a new
  `geom_front_contact` sphere (radius 0.372, mass 0, `condim="3"`,
  `friction="1.2 0.005 0.0001"`, `solref="-130000 -800"`, `contype`/`conaffinity`
  `"1"`, `rgba` alpha 0) added. `fork_travel` joint gets
  `solreflimit="0.01 1"` in ride only.
- **`src/bike_sim/mujoco/drivetrain.py`**: same treatment for
  `build_rear_wheel` — tyre non-colliding in ride, `geom_rear_contact` sphere
  radius 0.352 added.
- **`src/bike_sim/mujoco/rear_linkage.py`**: `mode` threaded through
  `_build_seatstay` (→ `build_rear_wheel`) and `_build_shock_assembly`, which
  adds `solreflimit="0.01 1"` to `shock_stroke` for ride. Confirmed
  `build_equality_constraints`'s `stand_clamp` weld stays gated to
  `mode == "playground"` — no change needed, ride never gets it.
- **`src/bike_sim/mujoco/actuators.py`**: `mode == "ride"` branch emits
  exactly three motors: `rear_drive` (`rear_wheel_spin`, gear 1, ctrlrange
  -150..150), `front_brake` (`front_wheel_spin`, gear 1, -200..200),
  `rear_brake` (`rear_wheel_spin`, gear 1, -200..200).
- **`src/bike_sim/mujoco/sensors.py`**: `build_sensors` gained a `mode`
  parameter; ride only appends `sensor_bar_accel` (`site_handlebar`) and
  `sensor_saddle_accel` (`site_seatpost_top`) accelerometers.
- **`src/bike_sim/mujoco/builder.py`**: `mode="ride"` uses
  `timestep="0.0005"`; `build_terrain(...)` called (ride only) right after
  `build_environment`; `mode=mode` threaded into `build_rear_linkage` and
  `build_sensors`.
- **`src/bike_sim/mujoco/exporter.py`**: `export_playground_models` now also
  writes `bike_ride.xml` (`mode="ride", include_rider=True`) and returns it
  under key `"ride"`.
- **`src/bike_sim/mujoco/_xml_format.py`**: `add_geom` gained optional
  `condim`/`solref` kwargs, `add_joint` gained optional `solreflimit` — all
  default `None`, so every pre-existing call site (other modes) is
  byte-for-byte unaffected.

## Tests written

- **`tests/test_ride_model.py`** (new, 14 tests): compiles `mode="ride"` once
  per module and asserts `nq==nv==12`; `neq==2` with no `stand_clamp`;
  `root_x`/`root_z`/`root_pitch` joint types; both wheel contact spheres
  (radius + `contype/conaffinity==1`) and both tyre cylinders
  (`contype/conaffinity==0`); hfield `nrow`/`ncol` match `FIELD`; timestep is
  `0.0005`; the catch plane sits below `FIELD.geom_z_m(ground_z_m)`; both ride
  accelerometers exist and target the correct sites; and the other three
  modes still compile and run `mj_forward` cleanly.
- **`tests/test_golden_baselines.py`**: added `test_baseline_bike_ride_xml`
  (byte-for-byte against `baseline_bike_ride.xml`, mirroring the three
  existing baseline tests) and added `"baseline_bike_ride.xml"` to
  `test_compiled_mujoco_models_validity`'s compile-check list.

## Verification

- Before generating the golden baseline: ran the full suite once with only
  `test_ride_model.py` in place. Its 14 tests passed against the compiled
  model, satisfying the precondition to generate the baseline.
- Ran `test_golden_baselines.py` before the baseline file existed: the three
  pre-existing baseline tests passed unchanged; only the two
  ride-baseline-dependent assertions failed with "file does not exist" —
  confirming the gating did not disturb any pre-existing output.
- Generated `tests/golden/baseline_bike_ride.xml` from
  `generate_mujoco_xml(specs=BikeSpecs(), solver=HorstLinkageSolver(specs),
  mode="ride", include_rider=True)`.
- Final full run: `uv run python -m pytest -q` → **150 passed** (135
  previously-passing + 14 new ride-model tests + 1 new golden-baseline test),
  no warnings, no skips.
- Also smoke-tested `export_playground_models` end-to-end: writes
  `bike_ride.xml` alongside the other three files and returns a 5-key dict;
  confirmed no test or CLI caller inspects the returned dict's key set or
  assumes exactly N output files (checked `test_mass_distribution.py`,
  `test_playground.py`, `cli/export.py`, `cli/main.py`).

## What was checked in the generated golden baseline

Read `tests/golden/baseline_bike_ride.xml` in full and checked it against
`docs/RIDE.md` §0-3, §9, §10:

- **§1 coordinates**: `root_x`/`root_z`/`root_pitch` present with the
  documented joint types and axes, in that order, first in the `frame` body,
  before the rider's geoms. `steer_joint` locked (`range="-0.01 0.01"
  stiffness="50000" damping="1000"`). `fork_travel` range `0 0.180000`;
  `shock_stroke` range `0 0.065000`. Only two `<connect>` equality
  constraints, no `stand_clamp`.
- **§2 road profile**: `<hfield name="road" nrow="2" ncol="24001"
  size="60.000000 0.500000 3.400000 0.500000"/>` — matches `FIELD` exactly,
  no `data` attribute (no elevation baked into XML). `terrain` geom at
  `pos="60.000000 0 -3.149500"` — verified `geom_x_m()==radius_x_m==60` and
  `geom_z_m(ground_z_m) = -0.3495 - 2.8 = -3.1495`, matching the doc's
  `-349.50 mm` ground level (§9). `catch_plane` at `z=-4.149500`, i.e.
  `geom_z_m - 1`, matching the doc's stated `-4.15 m`.
- **§3 wheel-ground contact**: both `geom_*_contact` spheres have the correct
  radii (0.372 front / 0.352 rear, matching the tyre cylinder sizes exactly),
  `mass="0"`, `condim="3"`, `solref="-130000 -800"`,
  `friction="1.2 0.005 0.0001"` set explicitly (matching the doc's "must be
  set on both sides" note, also applied to the `terrain` geom). Both tyre
  cylinders are `contype="0" conaffinity="0"` in ride only.
- **§10 numerical settings**: `timestep="0.0005"`, `integrator="implicitfast"`
  (shared with other modes as documented). `solreflimit="0.01 1"` present on
  both `fork_travel` and `shock_stroke` (final barrier, not the bottom-out
  path — no stiffness/damping was added, consistent with the doc's note that
  suspension forces are applied externally via `qfrc_applied`, not modeled in
  this MJCF layer). Both ride accelerometers present, targeting
  `site_handlebar` / `site_seatpost_top`.
- Exactly three `<motor>` actuators in ride mode, matching brief-specified
  names/joints/gear/ctrlrange verbatim.
- Confirmed decision #2 (no new material): only `mat_floor` is referenced by
  both `catch_plane` and `terrain`; no new `<material>` entries were added to
  `<asset>`.

## Files changed

- `src/bike_sim/mujoco/_xml_format.py` (modified)
- `src/bike_sim/mujoco/terrain.py` (new)
- `src/bike_sim/mujoco/environment.py` (modified)
- `src/bike_sim/mujoco/frame.py` (modified)
- `src/bike_sim/mujoco/steering_fork.py` (modified)
- `src/bike_sim/mujoco/drivetrain.py` (modified)
- `src/bike_sim/mujoco/rear_linkage.py` (modified)
- `src/bike_sim/mujoco/actuators.py` (modified)
- `src/bike_sim/mujoco/sensors.py` (modified)
- `src/bike_sim/mujoco/builder.py` (modified)
- `src/bike_sim/mujoco/exporter.py` (modified)
- `tests/test_ride_model.py` (new)
- `tests/test_golden_baselines.py` (modified)
- `tests/golden/baseline_bike_ride.xml` (new)

## Self-review

Read the full `git diff` with fresh eyes against the brief line by line
(completeness, naming, YAGNI, test quality) before committing:

- Every sub-builder change is gated by `mode == "ride"` (or threads a `mode`
  parameter that only branches for ride); confirmed no other mode's generated
  XML changed by re-running `test_golden_baselines.py` before the new
  baseline existed — all three pre-existing baseline tests passed unchanged.
- No unrequested refactors: `drivetrain.py` kept its pre-existing raw
  `ET.SubElement` dict style rather than switching to `add_geom`/`add_joint`,
  matching that file's established convention; `_xml_format.py` only gained
  optional kwargs, no signature reshuffling.
- `git diff`'s hunk headers occasionally mislabel which function a change
  belongs to (e.g. showing `_build_chainstay` for a change actually inside
  `_build_seatstay`, or `build_bb_and_motor` for a change inside
  `build_rear_wheel`) — this is git's nearest-preceding-`def`-line heuristic
  getting confused by closely-spaced multi-line signatures, not a real
  misplacement. Verified by reading the full post-edit file content directly.
- No new files exceed 500 lines (`rear_linkage.py`, the largest touched file,
  is 355 lines).
- Test suite output is pristine: 150 passed, 0 warnings, 0 skipped.

### Judgment calls made (flagging, not blocking)

1. **"Default contact parameters" on `catch_plane`** (brief's wording is
   ambiguous): interpreted as "material only, omit
   friction/contype/conaffinity/condim/solref so MuJoCo's built-in defaults
   apply" — the plane never needs anything more specific since only the
   `catch_plane` vs. wheel-sphere contact matters, and that pairing uses
   default friction/solref. This differs from `terrain`, which explicitly
   sets contact params per the brief's explicit instruction there.
2. **`rgba` for the two alpha-0 contact spheres**: chose `"0.08 0.08 0.08 0"`
   (matches `mat_tire`'s RGB with alpha 0) since the brief only mandates
   alpha 0, not a specific color.
3. **`site_handlebar` styling**: `size="0.008" rgba="0.9 0.6 0.1 1.0"`,
   matching the existing `site_BB` accelerometer-site convention in
   `frame.py`, since the brief specifies only the site's name and position.
4. **Sphere `size` literal formatting**: used bare `"0.372"`/`"0.352"`
   literals (not `:.6f`-formatted) for the two new contact geoms, matching
   the brief's exact verbatim value strings and the existing codebase's mixed
   convention of bare literals for fixed dimensional constants (e.g.
   `size="0.320"` on the front rim).

None of these affect physics or test correctness; all are cosmetic/styling
choices within the brief's stated freedom. No functional concerns or open
questions remain.

## Note on repository state

This session's working tree/git history did not match the `mujoco_visual`
branch and prior `phase7` commit history described at the start of the
conversation — the repo here only had an `init` commit before this work.
Given the source files, tests, and prior progress notes in
`.superpowers/sdd/2026-08-25-ride-mode/` all matched expectations, this is
almost certainly a difference in the execution environment (e.g. a fresh
sandbox/worktree) rather than a defect in this task's work. The commit below
was made on the repository's current branch (`main`); flagging this in case
it needs to be rebased/cherry-picked onto `mujoco_visual` in the canonical
repository.

## Fix report: review finding (contact-sphere size hardcoded)

### What changed

Per the review's Important finding, replaced the two hardcoded contact-sphere
`size` literals with the same in-scope radius variables already used for the
coaxial tyre cylinder three lines above:

- `src/bike_sim/mujoco/steering_fork.py:131` (in `_build_front_wheel`):
  `size="0.372"` → `size=f"{front_wheel_radius_m:.6f}"`.
- `src/bike_sim/mujoco/drivetrain.py:134` (in `build_rear_wheel`):
  `size="0.352"` → `size=f"{rear_wheel_radius_m:.6f}"`.

Both variables are computed upstream as `specs.front_wheel_radius / 1000.0`
(`steering_fork.py:159`) and `specs.rear_wheel_radius / 1000.0`
(`rear_linkage.py:266`), matching the review's description exactly. No other
lines were touched in either file.

Added a new test, `test_ride_mode_contact_sphere_tracks_wheel_radius` in
`tests/test_ride_model.py`, which builds the ride model with
`BikeSpecs(front_wheel_radius=400.0, rear_wheel_radius=380.0)` (non-default)
and asserts `geom_front_contact`/`geom_rear_contact` sphere sizes equal
`radius_mm / 1000.0`. This exercises the exact invariant the finding was
about — confirmed independently with a one-off script before adding the test
(`geom_front_contact` → `0.4`, `geom_rear_contact` → `0.38` for those inputs,
i.e. the sphere now tracks `BikeSpecs` instead of staying pinned at
0.372/0.352). It was added to the existing file without restructuring any
other test.

### Covering tests run

Command: `uv run python -m pytest tests/test_ride_model.py
tests/test_golden_baselines.py -q`

Result: **1 failed, 20 passed** — the only failure is
`TestGoldenBaselines::test_baseline_bike_ride_xml`.

Full suite: `uv run python -m pytest -q` → **1 failed, 150 passed** (151
collected: 135 pre-existing + 14 original ride tests + 1 new
wheel-radius-tracking test + 1 golden-baseline test, the last of which now
fails).

### Baseline diff — flagging per the review's explicit instruction, NOT regenerated

The review said: "`:.6f` keeps the emitted values byte-identical to what the
current baseline contains... If it does change, stop and report rather than
regenerating, because that would mean the literals were not equal to the
parameters."

It does change, so per that instruction I stopped and did **not** regenerate
`tests/golden/baseline_bike_ride.xml`. Isolated the exact diff by writing the
freshly generated XML to a scratch file and running `diff` against the
committed baseline — the entire diff is two lines:

```
102c102
<             ...size="0.372" mass="0" rgba=...
---
>             ...size="0.372000" mass="0" rgba=...
134c134
<             ...size="0.352" mass="0" condim=...
---
>             ...size="0.352000" mass="0" condim=...
```

Root cause: the review's premise doesn't hold, but not because the literals
were numerically wrong. `specs.front_wheel_radius / 1000.0` for the default
`BikeSpecs()` (372.0 mm) is exactly the same float as the old literal
`0.372`, and likewise `0.352` for the rear. The two are numerically identical
— confirmed by the fact that the *tyre* cylinder geoms (`geom_front_tire`,
`geom_rear_tire`), which already used `f"{front_wheel_radius_m:.6f}"` /
`f"{rear_wheel_radius_m:.6f}"` before this fix, render as `"0.372000"` /
`"0.352000"` in the very same pre-existing baseline file (lines 100 and 133).
The mismatch is purely that the **old contact-sphere literals were written
with 3 decimal places** (`"0.372"`, `"0.352"`) while `:.6f` always emits 6
(`"0.372000"`, `"0.352000"`) — a trailing-zero formatting difference, not a
value difference. So the fix is behavior-preserving in the sense that matters
(the underlying float radius used for collision geometry is unchanged and
now correctly tracks `BikeSpecs`), but it is not byte-identical in the
generated XML text, so the golden baseline test now correctly fails until the
baseline is updated.

This is a decision I am not making unilaterally, per the review's explicit
instruction. Two ways to resolve, either is mechanical:

1. Regenerate `tests/golden/baseline_bike_ride.xml` (this file is not one of
   the three protected pre-existing baselines named in the original task's
   critical constraint — it was created by this same task) to pick up the
   two `"0.372000"`/`"0.352000"` strings, consistent with how the tyre
   cylinder is already formatted right next to it.
2. Alternatively, format the two contact-sphere sizes to match the old
   3-decimal string exactly (e.g. `f"{front_wheel_radius_m:.3f}"`) to keep
   the baseline untouched — this still fixes the review's actual concern
   (the value now comes from `BikeSpecs`, not a hardcoded constant) but
   avoids touching the baseline file at all.

I have not applied either option pending direction, since the review
instructed stopping here rather than regenerating.

### Resolution: baseline regenerated per coordinator ruling

The coordinator reviewed the escalation, agreed the `:.6f`-is-byte-identical
premise was wrong, and ruled: regenerate `baseline_bike_ride.xml`, keep
`:.6f`. Reasoning accepted: the Global Constraint protects only the three
pre-existing baselines (`baseline_bike_model.xml`,
`baseline_bike_playground.xml`, `baseline_bike_playground_stand.xml`,
`baseline_coordinates.json`), all of which remain untouched and passing;
`baseline_bike_ride.xml` was authored by this same task in the previous
round, so regenerating it here finishes the artifact rather than breaking a
pin. Formatting the sphere to `:.3f` instead would have planted a second,
inconsistent numeric convention next to the tyre cylinder's existing `:.6f`
purely to dodge touching a baseline this task just wrote — rejected as tail
wagging the dog.

**Regeneration.** Copied the committed baseline aside, regenerated
`tests/golden/baseline_bike_ride.xml` from current code
(`generate_mujoco_xml(specs=BikeSpecs(), solver=HorstLinkageSolver(specs),
mode="ride", include_rider=True)`), then diffed old vs. new:

```
102c102
<             ...geom_front_contact... size="0.372" mass="0" ...
---
>             ...geom_front_contact... size="0.372000" mass="0" ...
134c134
<             ...geom_rear_contact... size="0.352" mass="0" ...
---
>             ...geom_rear_contact... size="0.352000" mass="0" ...
```

Confirmed (step 2 of the ruling): the diff is exactly these two lines — the
`size` attribute on `geom_front_contact` and on `geom_rear_contact` — nothing
else in the 266-line file moved. This is the expected, purely-cosmetic
change: `0.372` and `0.372000` are the same float; `0.352` and `0.352000`
are the same float.

**Full suite.** Command: `uv run python -m pytest -q`

Output:
```
........................................................................ [ 47%]
........................................................................ [ 95%]
.......                                                                  [100%]
151 passed in 3.03s
```

All 151 tests pass (150 from before this round + the previously-failing
`test_baseline_bike_ride_xml`, now green with the regenerated baseline).

**Commit.** All four files from this round committed together:

```
commit c5185c4
fix(mujoco): derive ride-mode contact sphere radius from BikeSpecs

 src/bike_sim/mujoco/drivetrain.py    |  2 +-
 src/bike_sim/mujoco/steering_fork.py |  2 +-
 tests/golden/baseline_bike_ride.xml  |  4 ++--
 tests/test_ride_model.py             | 15 +++++++++++++++
 4 files changed, 19 insertions(+), 4 deletions(-)
```

`git status --porcelain` afterward shows no tracked changes outstanding
(only pre-existing untracked tooling directories, unrelated to this task).
The Important finding is now fully resolved: the contact sphere radius comes
from `BikeSpecs` at every call site, and a test guards the invariant against
regression.
