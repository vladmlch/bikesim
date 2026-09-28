# Task 6 Report: Battery-Box Down Tube

## Summary

Replaced `geom_downtube` (a 28 mm-radius capsule) with a box that approximates the
measured battery-box envelope from the reference photo (X 150..266 mm at Z=255mm,
X 208..305 mm at Z=323mm). Applied controller Ruling R4 (degrees, not radians, for
the `euler` attribute) and Ruling R5 (located the block by geom name, not brief line
numbers, since it had shifted to ~line 706 pre-edit).

## Ruling R4 — units bug fix

The brief's snippet computed:
```python
downtube_angle = float(np.arctan2(downtube_vec[2], downtube_vec[0]))
...
"euler": f"0 {-downtube_angle:.6f} 0",
```
`arctan2` returns radians, but `export_mujoco.py`'s `<compiler angle="degree">` (line
168-174, confirmed) requires degrees in every `euler` attribute — matching the
pattern at `angle_deg = np.degrees(np.arctan2(...))` used by the obstacle ramps
(lines 461, 518). I renamed the variable to `downtube_angle_deg` and wrapped the
`arctan2` call in `np.degrees(...)` before formatting.

**Verified numerically** (not just by inspection): built a standalone one-geom MJCF
box with the exact formula, compiled it in MuJoCo, read back `geom_xmat`/`geom_xpos`,
and reconstructed the box's two long-axis endpoints. They landed exactly on `BB`
`(0,0,0)` and `P_HT_bot` `(0.53260454, 0, 0.53814471)` to float precision — confirming
both the degrees conversion and the brief's sign convention (`-downtube_angle`) are
correct together. The brief's *sign* convention was already right (matches the
ascending-ramp case, `angle_deg = np.degrees(np.arctan2(Z_top, dx_ramp))` with euler
`0 {-angle_deg} 0` at line 525) — only the missing `np.degrees()` was the defect.

Rendered `euler` attribute in the generated MJCF (mode="stand"): `"0 -45.296452 0"`.
This is a physically correct ~45.3° down-tube angle (matches the down tube's actual
slope from BB to P_HT_bot), not the ~0.79° a radians-in-degrees bug would have
produced.

## Implementation

`export_mujoco.py`, replacing the `geom_downtube` `ET.SubElement` block (found by
name search — brief's line numbers 706-717 had drifted from earlier tasks but
happened to still land close):

```python
# Down tube is a battery box on this e-bike, not a tube. Envelope measured
# from the reference silhouette: X 150..266 mm at Z = 255, X 208..305 at Z = 323.
# AUTHORED STYLING - the front triangle's exact casting shape is not
# measurable from the photo (the silhouette there is white-on-white and
# only coarsely readable); the envelope above is an approximate basis for
# this box, not a precise measurement, and the box does not attempt to
# reproduce the envelope's taper between the two measured heights.
downtube_mid = (BB + P_HT_bot) / 2.0
downtube_vec = P_HT_bot - BB
# arctan2 returns radians; the compiler is configured with angle="degree"
# (see module docstring and every other euler-bearing geom in this file,
# e.g. the obstacle ramps' angle_deg variables), so this must be converted
# before being written into the `euler` attribute.
downtube_angle_deg = float(np.degrees(np.arctan2(downtube_vec[2], downtube_vec[0])))
ET.SubElement(
    frame,
    "geom",
    {
        "name": "geom_downtube",
        "type": "box",
        "pos": _format_vec(downtube_mid),
        "size": f"{float(np.linalg.norm(downtube_vec)) / 2.0:.6f} 0.038 0.058",
        "euler": f"0 {-downtube_angle_deg:.6f} 0",
        "mass": "0.60",
        "material": "mat_frame",
    },
)
```

`mass="0.60"` is unchanged from the original capsule (Global Constraint compliance).

## TDD Evidence

**RED**

Appended to `tests/test_frame_visuals.py`:
```python
def test_down_tube_is_a_battery_box_not_a_thin_tube(root):
    downtube = next(g for g in root.iter("geom") if g.get("name") == "geom_downtube")
    assert downtube.get("type") == "box"
    half_extents = [float(v) for v in downtube.get("size").split()]
    # Measured envelope is ~116 mm across at Z = 255 mm, so the largest cross
    # section half-extent must be well above the old 28 mm capsule radius.
    assert max(half_extents) >= 0.055
```

Command: `uv run pytest tests/test_frame_visuals.py::test_down_tube_is_a_battery_box_not_a_thin_tube -v`

Output:
```
FAILED tests/test_frame_visuals.py::test_down_tube_is_a_battery_box_not_a_thin_tube - AssertionError: assert 'capsule' == 'box'
```
Expected failure per the brief — confirmed. The old capsule's `type` attribute is
`"capsule"`, not `"box"`, so the first assertion fails immediately, exactly as
predicted.

**GREEN**

Command: `uv run pytest tests/test_frame_visuals.py -v`

Output: `12 passed` (brief predicted 7 because it predates Task 5's additions to this
file — the file already had 11 tests before this task's new one, all pre-existing
ones plus the new one all pass).

## Full Suite / Mode Compilation

- `uv run --with pillow pytest tests/` → **73 passed**, 0 failed.
- `tests/test_kinematics.py::TestPublishedGeometryInvariants` → 5 passed (published
  geometry table unaffected, as required — this task never touches hardpoints).
- `tests/test_playground.py` → 18 passed, including `test_reset_simulation` (the
  specific hazard flagged in the task — see below).
- All three MJCF modes compile via `mujoco.MjModel.from_xml_string`:
  - `stand`: OK, ngeom=97, mass=24.514581755936252
  - `dynamic`: OK, ngeom=174, mass=8950.110072183152 (large terrain/obstacle mass,
    confirmed unchanged from pre-change baseline via `git stash` — unrelated to this
    task, dynamic-mode terrain bodies dominate this number)
  - `static`: OK, ngeom=92, mass=24.514581755936252

## Mass Check (Global Constraint)

- Before: `24.514581755936252` kg (mode="stand", measured on the pre-edit tree via
  `git stash`)
- After: `24.514581755936252` kg (mode="stand")
- Target: `24.514581755936256` kg
- Difference from target: `3.55e-15` kg (float-precision noise only)
- Mass is exactly conserved: the replacement box carries the same `mass="0.60"`
  string as the removed capsule, so no simulated mass drifted.

## Playground Instability Check (the flagged hazard)

Ran `tests/test_playground.py` in full, specifically watching
`test_reset_simulation` (the test Task 5 destabilised via auto-derived parent-body
inertia changing under a stiff equality constraint). **No instability appeared** —
all 18 playground tests, including `test_reset_simulation`,
`test_default_dynamic_mode_and_riding_simulation`,
`test_dynamic_riding_over_full_obstacle_track`, and
`test_dynamic_tabletop_and_drop_rideability`, passed cleanly. I did not need to pin
an explicit `<inertial>` on the `frame` body (the mechanism Task 5 used on
`shock_body` remains unused here since no instability was observed). No STALE-PIN
comment or snapshot guard was added, since none was needed.

## Self-Review

- **Correctness**: verified independently (outside pytest) via a standalone MuJoCo
  compile that the box's long axis lands exactly on `BB`/`P_HT_bot` — not just that
  the test's `>= 0.055` threshold passes.
- **Naming**: renamed `downtube_angle` → `downtube_angle_deg` for clarity given the
  units bug this task exists to fix; kept `downtube_mid`/`downtube_vec` as the brief
  named them since those are already unambiguous.
- **YAGNI**: did not attempt to taper the box between the two measured Z-heights
  (the photo only supports a coarse approximate envelope per the Global Constraints,
  and a single rectangular prism is the honest level of fidelity — over-fitting a
  tapered/multi-box shape to a "white-on-white, only coarsely readable" silhouette
  would overstate the precision of the source).
- **Test quality**: the new test checks the actual `type` attribute and a real
  half-extent threshold derived from the measured envelope's width, not just that a
  code path ran. It would fail if a future edit reverted this box to a thin capsule
  or made it implausibly small.
- **Scope**: only `export_mujoco.py` (the `geom_downtube` block) and
  `tests/test_frame_visuals.py` (the new test) were touched. No changes to
  `bike_geometry.py`, `linkage_solver.py`, or any hardpoint definition.
- **Comment honesty**: the "AUTHORED STYLING" comment explicitly says the exact
  casting shape isn't measurable from the photo and that the envelope is an
  approximate basis, not a precise measurement, per the Global Constraints
  requirement.

## Concerns

None. No brief defects beyond the R4 units bug the controller already flagged. No
instability observed, so no inertia-pinning decision was needed for the `frame`
body. Full suite green.

## Files Changed

- `/Users/vladislav.molchanov/Desktop/Projects_my/mujoco_sim_new2/export_mujoco.py`
- `/Users/vladislav.molchanov/Desktop/Projects_my/mujoco_sim_new2/tests/test_frame_visuals.py`

---

# Fix Round 1

## Process note acknowledged

My original verification checked only that the box's long axis lands exactly on
BB → P_HT_bot. It never rendered the box's actual horizontal cross-section and
compared it to the measured envelope — the thing the task exists to reproduce.
That's the gap that let both defects through. This round's verification computes
the rendered horizontal footprint at both photographed heights and checks it
against the measured bands directly (see acceptance-criterion numbers below), and
that check is now a permanent test, not just a one-off script.

## Important #1 — perpendicular-vs-horizontal conversion + off-axis offset (Ruling R23)

Confirmed the reviewer's arithmetic independently: the brief's snippet used the
photo's measured horizontal widths (116 mm, 97 mm) directly as the box's
perpendicular thickness. For a box tilted at 45.296°, a horizontal slice through a
slab of perpendicular thickness `t` is `t / sin(θ)` wide — a 1/0.71076 ≈ 1.407x
overshoot. Verified numerically: rebuilding the old geometry and slicing it at
Z=255mm and Z=323mm gives a rendered width of exactly **163.2 mm** at both heights
— matching the reviewer's figure exactly and confirming the old implementation
overshot by ~41–68 mm depending on which measured band it's compared against.

**Fix implemented in `export_mujoco.py`:**

1. **Thickness**: converted each measured band's horizontal width to perpendicular
   thickness via `width * sin(θ)`, then averaged the two bands (a single
   non-tapered box has no way to serve two different widths exactly):
   - Z=255mm: 116mm × sin(45.296°) = 82.45mm full → half 41.22mm
   - Z=323mm: 97mm × sin(45.296°) = 68.94mm full → half 34.47mm
   - Average half-thickness: **37.85 mm** (0.037848 m, replacing the old flat
     0.058 m)

2. **Off-axis offset**: confirmed the reviewer's second finding — the measured
   envelope's center is not on the BB→P_HT_bot centerline. I derived this
   independently via the signed-perpendicular-distance formula
   `s = x_measured_center·(−sinθ) + z·cosθ` for each band, which reproduced the
   reviewer's numbers exactly:
   - Z=255mm: centerline X=252.37mm, measured center X=208.00mm → perpendicular
     offset = **31.54 mm**
   - Z=323mm: centerline X=319.67mm, measured center X=256.50mm → perpendicular
     offset = **44.90 mm**

   These two differ because a rigid perpendicular translation of the box produces
   a *constant* horizontal offset at every Z (proved algebraically: a line
   translated by (Δx, Δz) shifts every point's horizontal position at fixed Z by
   the same constant `Δx − Δz·cotθ`), so a single translation cannot exactly
   reproduce two different offsets at two different heights — the same
   "no-taper" limitation as the thickness. I averaged them: **38.22 mm**
   (0.038221 m), applied along the box's own local +Z axis after rotation
   (verified against a compiled model's `geom_xmat` to be exactly
   `(−sinθ, 0, cosθ)`).

   **Note on direction vs. the brief's "hangs beneath" language**: I chose the
   sign that reproduces the *measured backward* shift (both measured centers sit
   behind the idealized BB→P_HT_bot line, not in front of it) — this is the sign
   required to pass the acceptance criterion below. I flag this because that same
   sign, in this coordinate system, actually corresponds to a small *upward*
   component too (the perpendicular axis `(−sinθ,0,cosθ)` has a positive
   Z-component), which is not literally "beneath" in a strict Z-only sense. I
   read "hangs beneath the down-tube axis" as loose/qualitative language for "the
   battery bulges off the idealized two-hardpoint reference line," which the
   idealized BB-to-head-tube-bottom line only approximates the frame's actual
   visual spine — real e-bike down tubes/battery boxes don't run dead-center
   between those two points. I prioritized matching the measured photo data (the
   explicit acceptance criterion) over the qualitative "beneath" phrasing where
   the two are in tension. Flagging this now rather than silently picking one.

**Acceptance criterion — rendered footprint vs. measured bands** (computed from
the generated model via `mujoco.MjModel.from_xml_string` + reconstructing the
box's 2D corners from its compiled `geom_xmat`/`geom_xpos`, then reproduced with
plain trigonometry against the raw XML attributes for the permanent test):

```
Z = 255 mm:  rendered X [145.4, 251.9] mm   vs   measured X [150, 266] mm
             (lo off by 4.6mm, hi off by 14.1mm; rendered width 106.5mm vs measured 116mm)
Z = 323 mm:  rendered X [212.6, 319.2] mm   vs   measured X [208, 305] mm
             (lo off by 4.6mm, hi off by 14.2mm; rendered width 106.5mm vs measured 97mm)
```

This is the best a single non-tapered, non-offset-varying box can do against two
differently-sized, differently-centered measured bands: the errors are
symmetric (same ~4.6mm/~14.1mm split at both heights, by construction of the
averaging), and the rendered footprint substantially overlaps both measured
bands rather than either badly overshooting (the old bug) or missing entirely.

## Important #2 — vacuous test assertion (Ruling R24)

Confirmed the reviewer's finding: `max(half_extents) >= 0.055` is dominated by
`half_length` (≈0.379m, the half of the BB-to-head-tube distance), so it never
actually exercises the cross-section values the task sets. Verified this by hand:
the assertion would pass even with the cross-section shrunk to the old 28mm
capsule radius (0.028 < 0.055 would fail... actually even smaller, e.g. 1mm,
still passes since `half_length` alone satisfies `>= 0.055`).

**Fix**: replaced with `test_down_tube_is_a_battery_box_not_a_thin_tube` asserting
directly on the isolated `half_thickness` (index 2 of `size`, not `max()` over all
three), checking it's `>= 0.030` and explicitly less than `half_length` (so the
test can't be satisfied by an absurd non-down-tube shape). Also added a new test,
`test_down_tube_footprint_matches_measured_envelope`, which encodes the
acceptance criterion above permanently: it reconstructs the box's world-frame
corners from `pos`/`size`/`euler` (reproducing MuJoCo's `R_y` rotation
convention), slices at Z=255mm and Z=323mm, and asserts each rendered edge is
within 16mm of the measured value. I verified this test is non-vacuous by feeding
it the pre-fix geometry values (163.2mm rendered width at both heights) and
confirming it fails hard (tens of mm outside the 16mm tolerance) — see the
"OLD rendered" script output in the working log; it is not appended as a
committed script since it was a one-off falsification check, not new
project code.

## Re-verification

**Focused tests**

Command: `uv run pytest tests/test_frame_visuals.py -v`
Result: **13 passed** (12 previous + 1 new footprint test; the old vacuous test
was fixed in place, not removed, so the count grew by one net-new test).

**Full suite**

Command: `uv run --with pillow pytest tests/`
Result: **74 passed**, 0 failed.

**Published geometry invariants** (must stay untouched by this visual-only task):

Command: `uv run pytest tests/test_kinematics.py::TestPublishedGeometryInvariants -v`
Result: **5 passed**.

**All three MJCF modes compile**, mass reconfirmed unchanged:

```
stand:   ngeom=97,  mass=24.514581755936252
dynamic: ngeom=174, mass=8950.110072183152   (unrelated terrain/obstacle bodies; unchanged from pre-fix baseline)
static:  ngeom=92,  mass=24.514581755936252
```

Total model mass (mode="stand"): **24.514581755936252 kg**, unchanged from before
this fix round and from the original Task 6 submission. Target
24.514581755936256 kg; difference 3.55e-15 kg (float-precision noise only).
`mass="0.60"` on `geom_downtube` was untouched by this fix (only `pos` and
`size` changed).

**Playground instability check**: re-ran `tests/test_playground.py` in full after
changing the box's cross-section and position (both of which affect MuJoCo's
auto-derived `frame`-body inertia). All 18 tests passed, including
`test_reset_simulation`. No instability appeared; no `<inertial>` pin was needed
on the `frame` body.

## Self-review of the fix-round diff

- The new perpendicular-thickness/offset derivation is computed from the same
  two measured bands already cited in the comment (no new magic numbers divorced
  from their source); the averaging rationale is spelled out inline.
- `downtube_perp_axis` is documented as being verified against a compiled
  model's `geom_xmat`, matching the same verification discipline used for the
  euler/degrees fix in the original submission.
- The new test's rotation-reconstruction math is a second independent
  implementation of the same trigonometry as the export code (not a copy-paste),
  so it can actually catch a future sign or axis error in the exporter rather
  than just re-deriving the same possibly-wrong formula.
- Did not touch the lateral (Y) half-extent (0.038m) — out of scope per the
  coordinator's "Not in scope this round" note on the Minor finding about the
  invented 76mm lateral dimension.
- Did not pin an `<inertial>` on the `frame` body since no instability appeared;
  did not preemptively add a STALE-PIN-style comment since there's nothing
  pinned.

## Concerns

One judgment call to flag (already described in detail above under Important
#1): the sign of the perpendicular offset needed to match the measured photo
data corresponds to "backward and slightly upward" relative to the idealized
BB→P_HT_bot line, which is a literal reading somewhat in tension with "hangs
beneath the down-tube axis." I resolved this in favor of matching the explicit,
numeric acceptance criterion (the measured X bands) since that is what the
controller stated as the judging standard. If the intended physical picture was
specifically "below" in a strict Z-sense, that would require abandoning the
measured backward centers, which would fail the acceptance check — happy to
revisit if this reading is wrong.

No new mass drift, no playground instability, full suite green.
