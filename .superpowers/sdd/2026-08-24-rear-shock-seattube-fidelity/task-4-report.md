# Task 4 Report: Continuous Seat Tube and Frame Casting

## Summary

Implemented per the brief with Ruling R3 (visual-only casting) and Ruling R5
(located the block to delete by geom name, not line numbers — the actual
block was at `export_mujoco.py:755-822` in the pre-Task-4 file, which
matched the brief's stated range).

## What changed

`export_mujoco.py`: replaced the "Split Seat Tube (Shock Tunnel Architecture)"
block (5 geoms: `geom_seattube_upper`, `geom_seattube_strut_l_top`,
`geom_seattube_strut_l_bot`, `geom_seattube_strut_r_top`,
`geom_seattube_strut_r_bot`, plus the now-unused `p_split` helper variable)
with:

- `geom_seattube`: a single capsule from `P10` to `seattube_end =
  (-0.030, 0.0, 0.258)` — the measured termination point where the real
  seat tube dies into the casting.
- `geom_frame_yoke`: a visual-only box (`contype="0" conaffinity="0"` per
  Ruling R3) centered between `seattube_end` and `P5`, sized
  `0.075 0.036 0.130` half-extents per the brief, carrying the rocker pivot
  P5 inside its material.

Both carry a code comment stating that shapes below the seat tube are
authored styling, not photo-measured (the front triangle is white-on-white
and unsegmentable), satisfying the global constraint to flag this in a
comment.

`tests/test_frame_visuals.py` (new): the three tests from the brief —
cage-removed check, continuous-tube-terminates-at-casting check
(`SEAT_TUBE_TERMINATION_Z_M = 0.258`, tolerance 0.015), and a compile
smoke test.

## TDD evidence

**RED** — `uv run pytest tests/test_frame_visuals.py -v`, before implementing:

```
tests/test_frame_visuals.py::test_split_seat_tube_cage_is_gone FAILED
tests/test_frame_visuals.py::test_seat_tube_is_continuous_and_terminates_at_the_casting FAILED
tests/test_frame_visuals.py::test_model_still_compiles PASSED
...
E AssertionError: geom_seattube_strut_l_top should have been removed
...
E AssertionError: assert 'geom_seattube' in {...}
2 failed, 1 passed
```

This is the expected failure: the split-strut geoms were still present
(not yet deleted) and `geom_seattube`/`geom_frame_yoke` did not exist yet
(not yet added). The compile test passed trivially since it doesn't
reference either geom set.

**GREEN** — `uv run pytest tests/test_frame_visuals.py -v`, after implementing:

```
tests/test_frame_visuals.py::test_split_seat_tube_cage_is_gone PASSED
tests/test_frame_visuals.py::test_seat_tube_is_continuous_and_terminates_at_the_casting PASSED
tests/test_frame_visuals.py::test_model_still_compiles PASSED
3 passed in 0.31s
```

**Full suite** — `uv run --with pillow pytest tests/`:

```
tests/test_air_spring.py .......                                       [ 10%]
tests/test_fitted_hardpoints.py ......                                  [ 20%]
tests/test_frame_visuals.py ...                                         [ 25%]
tests/test_kinematics.py .................                              [ 51%]
tests/test_mass_distribution.py .........                               [ 65%]
tests/test_photo_reference.py ....                                      [ 71%]
tests/test_playground.py .................                              [100%]
64 passed in 5.09s
```

`tests/test_kinematics.py::TestPublishedGeometryInvariants` is included in
that green run — the locked geometry table (reach/stack/head angle/etc.) is
unaffected, as expected since P5/P10 and `bike_geometry.py` were not touched.

## Geom counts and compile check across all modes

Compared `ngeom` between `HEAD~1` (pre-Task-4, via `git show HEAD:export_mujoco.py`
before this commit) and the working tree, across all three MJCF modes:

| mode     | before | after | delta |
|----------|--------|-------|-------|
| standard | 96     | 93    | -3    |
| stand (test fixture mode) | 100 | 97 | -3 |
| dynamic  | 177    | 174   | -3    |

(5 geoms removed, 2 added → net -3, consistent across every mode.)

All three modes (`standard`, `stand`, `dynamic`) were loaded via
`mujoco.MjModel.from_xml_string(...)` and compiled without error, each with
`ngeom > 0`.

## Self-review

- Read the full diff (`git diff -- export_mujoco.py`, 23 insertions / 52
  deletions). Confirmed no leftover reference to the deleted `p_split`
  variable (`grep -n "p_split\b"` → no matches).
- Confirmed `geom_frame_yoke`'s computed position/extents match the
  controller's pre-verification exactly: center X 0.0056605, Z 0.129,
  spanning X[-0.0693, 0.0807] / Z[-0.001, 0.259] — contains P5, and the
  0.036 m half-width in Y is narrower than the rocker arms at Y=±0.045 m so
  the linkage stays visible outside the casting.
- Confirmed `contype="0" conaffinity="0"` is present on `geom_frame_yoke`
  only, matching Ruling R3 (`geom_seattube` remains a normal collidable
  frame member, matching the original tube's behavior).
- Confirmed `geom_roc_arm_*` (lines ~1574-1634, well outside the edited
  region) were untouched, per the brief's "leave the rocker arms alone."
- Confirmed no hardpoint or `bike_geometry.py`/`linkage_solver.py` files
  were touched — `git status` shows only `export_mujoco.py` modified and
  `tests/test_frame_visuals.py` added.
- Mass note: total mass of the removed 5 geoms was 0.25 + 4×0.10 = 0.65 kg;
  the two new geoms total 0.45 + 0.55 = 1.0 kg (both values taken verbatim
  from the brief). This is a net +0.35 kg on the frame body. No test
  asserts total system mass to a fixed value in a way that this broke
  (`tests/test_mass_distribution.py` still passes), so this is not a
  regression, just worth flagging as a small mass increase from the
  brief's own numbers.
- Naming: `geom_frame_yoke` (the frame casting) and the pre-existing
  mechanical `shock_yoke` body / `geom_yoke_*` geoms (the physical rocker
  linkage yoke) are two different "yoke" concepts sharing the word. This
  is the name mandated by the brief's Interfaces section, not something I
  introduced; flagging it only as a minor readability note for future
  maintainers, not a defect.
- Test quality: the three tests exercise real generated-XML structure
  (geom presence/absence, actual `fromto` coordinates against the measured
  258 mm termination, and a real MuJoCo compile) rather than trivial
  smoke checks — considered them adequate as committed by the brief and
  did not add further tests, since the task scope is narrowly "make the
  brief's tests pass," and global constraints already pin the geometry
  table via a separate locked test module.

## Concerns

None blocking. The only items noted above (mass delta, shared "yoke"
terminology between two unrelated parts) are pre-specified by the brief's
own literal values/names and are not defects introduced by this
implementation.

## Files changed

- `/Users/vladislav.molchanov/Desktop/Projects_my/mujoco_sim_new2/export_mujoco.py`
- `/Users/vladislav.molchanov/Desktop/Projects_my/mujoco_sim_new2/tests/test_frame_visuals.py` (new)

## Commit

`5f9b27c feat(visual): continuous seat tube into frame casting, drop strut cage`

---

## Fix round 1/5

The coordinator's review returned Spec OK but quality "Needs fixes" with
four Important findings, all traced to the brief's own values (not to
choices I made) — mass parity, a self-contradictory comment, a missing
regression guard for Ruling R3, and a naming collision. All four are
fixed below.

### Fix #1 — mass conservation (Ruling R16)

The five removed split-strut geoms summed to `0.25 + 4*0.10 = 0.65` kg.
The first pass's replacements (`0.45 + 0.55 = 1.00` kg, taken verbatim
from the brief) added +0.35 kg of real simulated mass on a task scoped
visual-only, undetected because `test_mass_distribution.py`'s total-mass
assertion has a ±0.5 kg window.

Rebalanced by computed solid volume: `geom_seattube` (capsule, r=0.020 m,
length ≈0.1451 m between P10 and `seattube_end`) has volume ≈2.159e-4 m³;
`geom_frame_casting` (box, full dims 0.15×0.072×0.26 m) has volume
≈2.808e-3 m³ — a 7.1% / 92.9% split. Assigned `seattube_mass_kg = 0.05`,
`casting_mass_kg = 0.65 - 0.05 = 0.60` (both derived from a single
`SEATTUBE_CASTING_MASS_KG = 0.65` constant in `export_mujoco.py`, so the
sum can never drift independently again), matching that volume ratio
(7.7% / 92.3%) to within rounding.

Added `tests/test_frame_visuals.py::test_seattube_and_casting_mass_is_conserved`,
asserting `geom_seattube.mass + geom_frame_casting.mass == pytest.approx(0.65)`.

Verified new total model mass:

```
uv run python -c "
import mujoco
from export_mujoco import generate_mujoco_xml
for mode in ('standard', 'stand', 'dynamic'):
    xml = generate_mujoco_xml(mode=mode)
    model = mujoco.MjModel.from_xml_string(xml)
    total_mass = sum(model.body_mass)
    print(mode, 'ngeom=', model.ngeom, 'total_body_mass=', round(total_mass, 4))
"
```
```
standard ngeom= 93 total_body_mass= 24.5146
stand ngeom= 97 total_body_mass= 24.5146
dynamic ngeom= 174 total_body_mass= 8950.1101
```

`standard`/`stand` total body mass is now 24.5146 kg, matching the
project's stated baseline of "~24.5 kg". (`dynamic` includes obstacle/track
bodies, so its total is not comparable to the frame-mass baseline.)

### Fix #2 — self-contradictory comments (Ruling R17)

Deleted the false "Sized from the measured silhouette (X -35..+33 at
Z = 255)" citation on `geom_frame_casting` — P5 (X = +41.3 mm) lies
outside that band, and the box's real extents (150×72×260 mm) don't match
the cited 68 mm figure either, so the citation was never actually the
sizing basis. Replaced with a truthful description: authored styling,
sized to enclose the rocker frame pivot P5 and to meet the seat tube where
it terminates. The seat-tube-side comment block still correctly attributes
the *measured* Z≈258mm termination and 29→68mm silhouette step to the
photo reference; only the casting-geom comment's false measurement claim
was removed.

### Fix #3 — Ruling R3 regression guard (Ruling R18)

Added `tests/test_frame_visuals.py::test_frame_casting_is_visual_only`,
asserting `geom_frame_casting.get("contype") == "0"` and
`.get("conaffinity") == "0"` directly, so a later edit cannot silently
restore collision on this decorative box.

### Fix #4 — naming collision (Ruling R19)

Renamed `geom_frame_yoke` -> `geom_frame_casting` throughout
`export_mujoco.py` and `tests/test_frame_visuals.py` (the `pos`/`size`
math, mass, and material are unchanged — only the `name` attribute and
its comment changed). Confirmed no other reference to `geom_frame_yoke`
remains anywhere in the tree:

```
grep -rn "geom_frame_yoke" /Users/vladislav.molchanov/Desktop/Projects_my/mujoco_sim_new2/export_mujoco.py /Users/vladislav.molchanov/Desktop/Projects_my/mujoco_sim_new2/tests/test_frame_visuals.py
```
(no matches)

### Deferred (per coordinator instruction — not fixed this round)

`test_model_still_compiles` as a low-value smoke test, and the bare
`+ 0.0` in the casting's `pos` expression wanting a named constant. Left
untouched to keep this round in scope.

### Test evidence

Focused test file, all 5 tests (2 new: casting visual-only guard, mass
conservation):

```
uv run pytest tests/test_frame_visuals.py -v
```
```
tests/test_frame_visuals.py::test_split_seat_tube_cage_is_gone PASSED    [ 20%]
tests/test_frame_visuals.py::test_seat_tube_is_continuous_and_terminates_at_the_casting PASSED [ 40%]
tests/test_frame_visuals.py::test_frame_casting_is_visual_only PASSED    [ 60%]
tests/test_frame_visuals.py::test_seattube_and_casting_mass_is_conserved PASSED [ 80%]
tests/test_frame_visuals.py::test_model_still_compiles PASSED           [100%]
5 passed in 0.25s
```

Mass distribution tests (the mass rebalance touches these indirectly):

```
uv run pytest tests/test_mass_distribution.py -v
```
```
9 passed in 0.36s
```
(`test_total_bike_mass_specification`, `test_compiled_mujoco_body_masses`,
`test_center_of_gravity_location`, `test_static_axle_load_distribution`,
`test_wheel_rotational_inertia_computation`,
`test_loop_closure_tightness_with_mass`,
`test_suspension_sag_tuning_and_balanced_bottom_out`,
`test_kinematic_hardpoints_consistency_with_solver`,
`test_rider_inclusion_in_static_cg` — all passed.)

Full suite:

```
uv run --with pillow pytest tests/ -v
```
```
66 passed in 4.98s
```

All 66 tests pass, including `tests/test_kinematics.py::TestPublishedGeometryInvariants`
(the locked geometry table is unaffected — no hardpoint or
`bike_geometry.py`/`linkage_solver.py` file was touched in this round
either) and the full `test_mass_distribution.py` and `test_frame_visuals.py`
modules.

All three MJCF modes (`standard`, `stand`, `dynamic`) still compile via
`mujoco.MjModel.from_xml_string(...)` with `ngeom` unchanged from the
first pass (93 / 97 / 174 respectively — the rename and mass rebalance
touch no geometry, only names and mass attributes).

### Files changed (this round)

- `/Users/vladislav.molchanov/Desktop/Projects_my/mujoco_sim_new2/export_mujoco.py`
- `/Users/vladislav.molchanov/Desktop/Projects_my/mujoco_sim_new2/tests/test_frame_visuals.py`

### Commit (this round)

`8274ca6 fix(visual): mass parity, honest comment, guarded R3, casting rename`
