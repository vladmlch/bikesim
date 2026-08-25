"""Structural assertions on the generated MJCF frame visuals."""

import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from bike_sim.mujoco import generate_mujoco_xml

SEAT_TUBE_TERMINATION_Z_M = 0.258

# Total mass the split-strut cage carried (0.25 + 4*0.10 kg) before Task 4.
# geom_seattube + geom_frame_casting must continue to sum to this value so
# Task 4 stays visual-only and does not add or remove simulated mass.
REMOVED_CAGE_MASS_KG = 0.65


@pytest.fixture(scope="module")
def stand_xml():
    """Stand-mode MJCF built once: every consumer below wants the same string."""
    return generate_mujoco_xml(mode="stand")


@pytest.fixture(scope="module")
def stand_xml_markers():
    return generate_mujoco_xml(mode="stand", debug_markers=True)


@pytest.fixture(scope="module")
def stand_model(stand_xml):
    return mujoco.MjModel.from_xml_string(stand_xml)


@pytest.fixture(scope="module")
def root(stand_xml):
    return ET.fromstring(stand_xml)


def _geom_names(root):
    return {g.get("name") for g in root.iter("geom")}


def test_split_seat_tube_cage_is_gone(root):
    names = _geom_names(root)
    for removed in (
        "geom_seattube_strut_l_top", "geom_seattube_strut_l_bot",
        "geom_seattube_strut_r_top", "geom_seattube_strut_r_bot",
        "geom_seattube_upper",
    ):
        assert removed not in names, f"{removed} should have been removed"


def test_seat_tube_is_continuous_and_terminates_at_the_casting(root):
    names = _geom_names(root)
    assert "geom_seattube" in names
    assert "geom_frame_casting" in names

    seattube = next(g for g in root.iter("geom") if g.get("name") == "geom_seattube")
    coords = [float(v) for v in seattube.get("fromto").split()]
    lower_z = min(coords[2], coords[5])
    assert lower_z == pytest.approx(SEAT_TUBE_TERMINATION_Z_M, abs=0.015)


def test_frame_casting_is_visual_only(root):
    # Ruling R3: the casting must not participate in physics, or it would create
    # spurious contacts with the crank and, in dynamic mode, the ground.
    casting = next(g for g in root.iter("geom") if g.get("name") == "geom_frame_casting")
    assert casting.get("contype") == "0"
    assert casting.get("conaffinity") == "0"


def test_seattube_and_casting_mass_is_conserved(root):
    # Ruling R16: Task 4 is scoped visual-only. The two replacement geoms must
    # sum to the mass of the five removed split-strut geoms (0.25 + 4*0.10 kg),
    # or the frame silently gains/loses real simulated mass.
    seattube = next(g for g in root.iter("geom") if g.get("name") == "geom_seattube")
    casting = next(g for g in root.iter("geom") if g.get("name") == "geom_frame_casting")
    total_mass = float(seattube.get("mass")) + float(casting.get("mass"))
    assert total_mass == pytest.approx(REMOVED_CAGE_MASS_KG, abs=1e-6)


def test_model_still_compiles(stand_model):
    assert stand_model.ngeom > 0


BOTTOM_OUT_CLEARANCE_MM = 2.0

# Ruling R20: geom_shock_tab (the floating bracket) carried 0.10 kg. Its
# replacement, geom_frame_junction, must carry the same mass or the frame
# silently gains/loses real simulated mass (Task 5 is visual-only).
REMOVED_SHOCK_TAB_MASS_KG = 0.10


def test_floating_shock_bracket_is_replaced_by_frame_material(root):
    names = _geom_names(root)
    assert "geom_shock_tab" not in names, "the thin floating bracket must be gone"
    assert "geom_frame_junction" in names


def test_frame_junction_mass_matches_removed_shock_tab(root):
    junction = next(g for g in root.iter("geom") if g.get("name") == "geom_frame_junction")
    assert float(junction.get("mass")) == pytest.approx(REMOVED_SHOCK_TAB_MASS_KG, abs=1e-6)


def test_air_can_clears_the_lower_eyelet_at_bottom_out(root):
    """
    At full bottom-out the eye-to-eye collapses to e2e - stroke. The can is
    measured back from P7, so it must stay shorter than that by the clearance.
    """
    from bike_sim.geometry.specs import BikeSpecs

    specs = BikeSpecs()
    collapsed = specs.shock_eye_to_eye - specs.shock_stroke
    can = next(g for g in root.iter("geom") if g.get("name") == "geom_shock_body")
    coords = [float(v) for v in can.get("fromto").split()]
    can_len_mm = 1000.0 * (
        (coords[3] - coords[0]) ** 2 + (coords[4] - coords[1]) ** 2 + (coords[5] - coords[2]) ** 2
    ) ** 0.5
    # Coordinates round-trip through MJCF's 6-decimal-place (micron-resolution)
    # fromto strings, so reconstructing length via sqrt(sum of squares) carries
    # up to ~1 micron of formatting noise even when the derived length is exact.
    # 2e-3 mm absorbs that noise while staying 1000x tighter than the 2 mm
    # clearance this assertion protects.
    FORMATTING_NOISE_MM = 2e-3
    assert can_len_mm <= collapsed - BOTTOM_OUT_CLEARANCE_MM + FORMATTING_NOISE_MM
    assert can_len_mm == pytest.approx(138.0, abs=0.5)


def test_piggyback_sits_in_the_frame_plane(root):
    piggy = next(g for g in root.iter("geom") if g.get("name") == "geom_shock_piggyback")
    coords = [float(v) for v in piggy.get("fromto").split()]
    lateral = max(abs(coords[1]), abs(coords[4]))
    assert lateral <= 0.012, f"piggyback is {lateral*1000:.0f} mm off-plane; expected <= 12 mm"


def test_shock_body_inertial_pin_matches_child_geom_mass(root):
    """
    shock_body carries an explicit <inertial> pinned to its pre-Task-5 mass
    properties (see export_mujoco.py, "STALE-PIN WARNING") so that visual-only
    edits to the air can, trunnion overhang, or piggyback offsets don't silently
    perturb simulated dynamics. If a future edit changes a child geom's mass
    without updating the pin, this catches the mass half of that drift.
    """
    shock_body = next(b for b in root.iter("body") if b.get("name") == "shock_body")
    inertial = shock_body.find("inertial")
    pinned_mass = float(inertial.get("mass"))
    child_mass = sum(float(g.get("mass")) for g in shock_body.findall("geom"))
    assert pinned_mass == pytest.approx(child_mass, abs=1e-9)


# Task-5-as-shipped snapshot of shock_body's geometry-derived centre of mass
# (six child cylinders, midpoint-of-fromto weighted by mass). Frozen the same
# way export_mujoco.py's <inertial> pos/quat/diaginertia are frozen - see the
# STALE-PIN WARNING there. This is deliberately a fixed snapshot, not a live
# recomputation against the pinned pos: an earlier attempt to make the pin's
# `pos` track the current geometry live was tried and empirically reproduced
# the exact NaN-QACC divergence in test_playground.py::test_reset_simulation
# the pin exists to prevent, so pos/quat/diaginertia must stay a manually
# maintained snapshot rather than something a test can derive automatically.
SHOCK_BODY_GEOMETRY_COM_M = np.array([-0.02746485, 0.0024, -0.04195878])


def test_shock_body_geometry_com_has_not_drifted_unnoticed(root):
    """
    Complements test_..._mass: catches a length-only edit (body_can_len,
    trunnion_overhang, or a piggyback offset) that leaves every child geom's
    mass untouched and so wouldn't move the mass-parity test at all.

    shock_body's six children are all cylinders (geom_shock_trunnion_boss_l/r,
    geom_shock_trunnion_overhang, geom_shock_body, geom_shock_piggyback,
    geom_shock_piggy_bridge), and a uniform-density cylinder's center of mass is
    exactly the midpoint of its `fromto` endpoints - verified this matches
    MuJoCo's own body_ipos to full float precision before relying on it here.

    If this test fails, someone changed shock_body's length geometry. That is
    allowed, but it means export_mujoco.py's frozen <inertial> pin (pos/quat/
    diaginertia - see STALE-PIN WARNING there) is no longer guaranteed to match
    reality and needs a conscious decision, not a silent pass. Once you've made
    that call, refresh SHOCK_BODY_GEOMETRY_COM_M above to the new value.
    """
    shock_body = next(b for b in root.iter("body") if b.get("name") == "shock_body")

    total_mass = 0.0
    com = np.zeros(3)
    for g in shock_body.findall("geom"):
        mass = float(g.get("mass"))
        coords = [float(v) for v in g.get("fromto").split()]
        midpoint = (np.array(coords[:3]) + np.array(coords[3:])) / 2.0
        com += mass * midpoint
        total_mass += mass
    com /= total_mass

    assert np.allclose(com, SHOCK_BODY_GEOMETRY_COM_M, atol=1e-6), (
        f"shock_body's geometry-derived CoM {com} has moved from the Task-5 "
        f"snapshot {SHOCK_BODY_GEOMETRY_COM_M}; a length constant changed. "
        "Decide whether export_mujoco.py's frozen shock_body <inertial> pin "
        "needs recomputing (see STALE-PIN WARNING there), then refresh "
        "SHOCK_BODY_GEOMETRY_COM_M above."
    )


def test_down_tube_is_a_battery_box_not_a_thin_tube(root):
    downtube = next(g for g in root.iter("geom") if g.get("name") == "geom_downtube")
    assert downtube.get("type") == "box"
    half_length, half_lateral, half_thickness = [float(v) for v in downtube.get("size").split()]
    # Ruling R24: a bare max() over all three half-extents is dominated by
    # half_length (~0.38 m, the BB-to-head-tube distance) and would pass even
    # if the cross-section shrank back to the old 28 mm capsule radius, or to
    # near zero. Isolate the cross-section extent (half_thickness, the box's
    # perpendicular thickness) from the length extent and check it directly,
    # and sanity-check that length still dominates thickness so this box
    # reads as a down tube rather than some other shape.
    assert half_thickness >= 0.030, "cross-section must be well above the old 28 mm capsule radius"
    assert half_thickness < half_length


# Reference-photo measured envelope of the battery box: (Z height above BB in
# mm, measured X_lo mm, measured X_hi mm). See export_mujoco.py's geom_downtube
# comment for the derivation (perpendicular thickness/offset averaged from
# these two bands, since a single non-tapered box can't match both exactly).
DOWNTUBE_ENVELOPE_BANDS_MM = (
    (255.0, 150.0, 266.0),
    (323.0, 208.0, 305.0),
)

# Ruling R23 fix-round: the deliberate single-box compromise (averaging two
# differently-sized measured bands into one constant thickness and offset)
# leaves each rendered edge off by ~4.6 mm to ~14.2 mm from the measured
# bands (computed and reported in the Task 6 fix-round investigation). This
# tolerance is set just above that so the test stays meaningful: it would
# fail hard against the pre-fix bug (which rendered ~163 mm wide, blowing
# past either band by tens of mm), but tolerates the shape of compromise a
# single non-tapered box necessarily makes here.
DOWNTUBE_FOOTPRINT_TOLERANCE_MM = 16.0


def test_down_tube_footprint_matches_measured_envelope(root):
    """
    Ruling R23: the previous implementation applied the photo's measured
    HORIZONTAL widths directly as the box's perpendicular thickness. For a
    box tilted ~45.3 degrees, a horizontal slice through a slab of
    perpendicular thickness t is t / sin(theta) wide -- about 1.41x -- so
    that implementation overshot the true footprint substantially. This
    test renders the box's actual horizontal (X) footprint at the two
    photographed heights (via its pos/size/euler, reproducing MuJoCo's own
    R_y(euler_y) rotation convention -- verified against a compiled model's
    geom_xmat during the fix-round investigation) and checks it lands close
    to the measured bands, rather than only checking that some cross-section
    dimension is "big enough" (see test_down_tube_is_a_battery_box_not_a_thin_tube).
    """
    downtube = next(g for g in root.iter("geom") if g.get("name") == "geom_downtube")
    pos = np.array([float(v) for v in downtube.get("pos").split()])
    half_len, _, half_thickness = [float(v) for v in downtube.get("size").split()]
    euler_y_deg = float(downtube.get("euler").split()[1])
    phi = np.radians(euler_y_deg)

    # Box-local x/z axes (length / thickness directions) in the (X, Z)
    # plane, per MuJoCo's R_y(phi) convention: local_x = (cos phi, -sin phi),
    # local_z = (sin phi, cos phi).
    u_x = np.array([np.cos(phi), -np.sin(phi)])
    u_z = np.array([np.sin(phi), np.cos(phi)])
    center = np.array([pos[0], pos[2]])

    corner_signs = [(-1, -1), (-1, 1), (1, 1), (1, -1)]
    corners = [center + sx * half_len * u_x + sz * half_thickness * u_z for sx, sz in corner_signs]

    for z_mm, x_lo, x_hi in DOWNTUBE_ENVELOPE_BANDS_MM:
        z_m = z_mm / 1000.0
        crossings = []
        for i in range(len(corners)):
            p1, p2 = corners[i], corners[(i + 1) % len(corners)]
            z1, z2 = p1[1], p2[1]
            if (z1 - z_m) * (z2 - z_m) <= 0 and z1 != z2:
                t = (z_m - z1) / (z2 - z1)
                crossings.append(p1[0] + t * (p2[0] - p1[0]))
        assert len(crossings) == 2, (
            f"horizontal line at Z={z_mm}mm should cross the tilted box exactly twice, "
            f"got {len(crossings)}"
        )
        rendered_lo, rendered_hi = sorted(x * 1000.0 for x in crossings)
        assert rendered_lo == pytest.approx(x_lo, abs=DOWNTUBE_FOOTPRINT_TOLERANCE_MM), (
            f"Z={z_mm}mm: rendered X_lo={rendered_lo:.1f}mm vs measured {x_lo}mm"
        )
        assert rendered_hi == pytest.approx(x_hi, abs=DOWNTUBE_FOOTPRINT_TOLERANCE_MM), (
            f"Z={z_mm}mm: rendered X_hi={rendered_hi:.1f}mm vs measured {x_hi}mm"
        )


# Ruling R27: the plan's Task 7 write-up names markers on only three bodies.
# There are actually markers on six: frame, fork_lower, chainstay, seatstay,
# rocker and shock_yoke. Every one of them must be gated by debug_markers.
ALL_MARKER_GEOM_NAMES = {
    "marker_bb", "marker_p0", "marker_p5_l", "marker_p5_r", "marker_p7",
    "marker_p8", "marker_p9", "marker_p10", "marker_p11", "marker_cg",
    "marker_pfa",
    "marker_p2",
    "marker_p12", "marker_pra",
    "marker_p3_roc_l", "marker_p3_roc_r", "marker_p4_l", "marker_p4_r",
    "marker_p6_l", "marker_p6_r",
}


def test_markers_are_absent_by_default(root):
    markers = [g.get("name") for g in root.iter("geom")
               if (g.get("name") or "").startswith("marker_")]
    assert markers == [], f"hero render must have no debug markers, found {markers}"


def test_markers_return_when_requested(stand_xml_markers):
    root_el = ET.fromstring(stand_xml_markers)
    markers = {g.get("name") for g in root_el.iter("geom")
               if (g.get("name") or "").startswith("marker_")}
    assert "marker_p5_l" in markers
    assert "marker_p7" in markers
    # Ruling R27: verify all 20 markers across all six bodies come back, not
    # just the two named in the plan.
    assert markers == ALL_MARKER_GEOM_NAMES, (
        f"missing: {ALL_MARKER_GEOM_NAMES - markers}, unexpected: {markers - ALL_MARKER_GEOM_NAMES}"
    )


def test_equality_constraint_sites_survive_marker_removal(stand_model):
    """site_P3_ss / site_P3_rocker / site_P7_shock drive loop closure - never optional."""
    model = stand_model
    site_names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
                  for i in range(model.nsite)}
    for required in ("site_P3_ss", "site_P3_rocker", "site_P7_shock", "site_P6_yoke"):
        assert required in site_names


# ---------------------------------------------------------------------------
# Seatpost extension & saddle mounting
#
# Two defects the photo-registered overlay (tools/render_comparison.py) exposed:
#   1. geom_saddle's underside sat 13 mm above geom_seatpost's top with nothing
#      between them, so the saddle rendered as a detached slab.
#   2. The saddle top landed at Z ~= 533 mm - about 157 mm below where the
#      photograph puts it - reading as a fully slammed dropper post.
# ---------------------------------------------------------------------------

# MEASURED from docs/reference/bulls_sonic_evo_side.jpg, NOT copied from
# export_mujoco.py. The saddle is black against white, so its silhouette
# segments at luminance < 110 (the edge moves under 2 px across thresholds
# 90..150). Mapping through the calibration in
# docs/reference/bulls_reference_points.json - BB at pixel (587.375, 733.325),
# 1.618558 mm/px, image +y -> bike -Z - the top edge over the 154 image columns
# spanning the shell has median Z = 690.0 mm and mean 690.3 mm.
PHOTO_SADDLE_TOP_Z_MM = 690.0

# The measured top edge runs 685.2 .. 694.9 mm across the shell (the high end is
# the tail's kick-up, which a flat box cannot reproduce), and the photograph's
# own control-point noise floor is +-6 mm (design spec section 3). 12 mm covers
# both while still failing hard on the 157 mm error this test was written for.
SADDLE_TOP_TOLERANCE_MM = 12.0

# Photographed saddle shell: X = -288.7 .. -39.5 mm at its widest row, so the
# centroid is -164.1 mm and the shell is 249 mm long.
PHOTO_SADDLE_CENTROID_X_MM = -164.1

# The post and the saddle must MEET: no visible gap, and no interpenetration
# either. Both geoms are emitted from the same derived point, so anything above
# formatting noise (MJCF rounds to 6 dp = 1 micron) means someone reintroduced a
# hand-tuned offset.
POST_SADDLE_JOINT_TOLERANCE_MM = 0.01

# Published geometry table (design spec section 2). This is the VIRTUAL line from
# the saddle at riding height back to the bottom bracket, not the physical
# P10 -> P9 tube (which test_kinematics.py pins at 74.476 deg and which this work
# does not touch). It is a sanity check on saddle placement, not a constraint the
# model enforces, so the window is wide.
PUBLISHED_EFFECTIVE_SEAT_ANGLE_DEG = 77.0
EFFECTIVE_SEAT_ANGLE_TOLERANCE_DEG = 2.0


def _seatpost_top_m(root):
    """Returns the upper seatpost geom's top endpoint (the end away from P9)."""
    upper = next(g for g in root.iter("geom") if g.get("name") == "geom_seatpost_upper")
    coords = [float(v) for v in upper.get("fromto").split()]
    a, b = np.array(coords[:3]), np.array(coords[3:])
    return b if b[2] > a[2] else a


def _saddle_box(root):
    """Returns (centre, half_extents) of geom_saddle in metres."""
    saddle = next(g for g in root.iter("geom") if g.get("name") == "geom_saddle")
    centre = np.array([float(v) for v in saddle.get("pos").split()])
    half = np.array([float(v) for v in saddle.get("size").split()])
    return centre, half


def test_seatpost_extends_above_the_collar(root):
    """The exposed post above the seat collar P9 must exist as its own geom."""
    assert "geom_seatpost_upper" in _geom_names(root)
    top = _seatpost_top_m(root)
    collar = next(g for g in root.iter("geom") if g.get("name") == "geom_seatpost")
    collar_coords = [float(v) for v in collar.get("fromto").split()]
    collar_top_z = max(collar_coords[2], collar_coords[5])
    assert top[2] > collar_top_z + 0.05, (
        f"exposed post is only {(top[2] - collar_top_z) * 1000:.1f} mm long; the "
        "saddle cannot reach photographed height without it"
    )


def test_saddle_sits_on_the_seatpost_top(root):
    """
    Defect 1 regression: geom_saddle floated 13 mm above the post's top. Its
    underside must coincide with the post's top face, and the post's top must
    actually land under the saddle's footprint rather than merely at the same
    height somewhere else in the plane.
    """
    top = _seatpost_top_m(root)
    centre, half = _saddle_box(root)
    underside_z = centre[2] - half[2]

    gap_mm = (underside_z - top[2]) * 1000.0
    assert abs(gap_mm) <= POST_SADDLE_JOINT_TOLERANCE_MM, (
        f"saddle underside is {gap_mm:+.3f} mm from the seatpost top "
        f"({'floating' if gap_mm > 0 else 'sunk into the post'}); it must be derived "
        "from the post's top endpoint, not from a hand-tuned offset"
    )
    for axis, label in ((0, "X"), (1, "Y")):
        assert abs(top[axis] - centre[axis]) <= half[axis], (
            f"seatpost top is outside the saddle's {label} footprint "
            f"({top[axis] * 1000:.1f} mm vs {centre[axis] * 1000:.1f} "
            f"+-{half[axis] * 1000:.1f} mm): the two are coplanar but not touching"
        )


def test_saddle_top_matches_the_photographed_height(root):
    """
    Defect 2 regression: the model's saddle top sat at Z ~= 533 mm against a
    photographed 690 mm. Anchored to PHOTO_SADDLE_TOP_Z_MM, which is measured
    from the reference image (see the constant's derivation above), so this
    cannot degenerate into asserting a constant against itself.
    """
    centre, half = _saddle_box(root)
    saddle_top_mm = (centre[2] + half[2]) * 1000.0
    assert saddle_top_mm == pytest.approx(
        PHOTO_SADDLE_TOP_Z_MM, abs=SADDLE_TOP_TOLERANCE_MM
    ), (
        f"saddle top is at Z={saddle_top_mm:.1f} mm; the reference photograph puts "
        f"it at {PHOTO_SADDLE_TOP_Z_MM:.1f} mm"
    )
    centroid_mm = centre[0] * 1000.0
    assert centroid_mm == pytest.approx(PHOTO_SADDLE_CENTROID_X_MM, abs=15.0), (
        f"saddle centroid X={centroid_mm:.1f} mm vs photographed "
        f"{PHOTO_SADDLE_CENTROID_X_MM:.1f} mm"
    )


def test_effective_seat_angle_from_bb_to_saddle_height(root):
    """
    Cross-check on saddle placement. The seatpost axis produced by the fix,
    extended to the photographed saddle height, should subtend roughly the
    published 77 deg effective seat angle at the bottom bracket. This is
    deliberately independent of the physical seat-tube angle assertion in
    test_kinematics.py (74.476 deg, P10 -> P9), which this work leaves alone.
    """
    upper = next(g for g in root.iter("geom") if g.get("name") == "geom_seatpost_upper")
    coords = [float(v) for v in upper.get("fromto").split()]
    lower_pt, upper_pt = np.array(coords[:3]), np.array(coords[3:])
    if lower_pt[2] > upper_pt[2]:
        lower_pt, upper_pt = upper_pt, lower_pt

    axis = upper_pt - lower_pt
    centre, half = _saddle_box(root)
    saddle_top_z = centre[2] + half[2]
    # Point where the post axis crosses the saddle-top height - the standard
    # "virtual seat tube" construction.
    x_at_saddle = lower_pt[0] + axis[0] * (saddle_top_z - lower_pt[2]) / axis[2]

    angle_deg = np.degrees(np.arctan2(saddle_top_z, -x_at_saddle))
    assert angle_deg == pytest.approx(
        PUBLISHED_EFFECTIVE_SEAT_ANGLE_DEG, abs=EFFECTIVE_SEAT_ANGLE_TOLERANCE_DEG
    ), (
        f"BB -> saddle-height line is {angle_deg:.2f} deg, published effective seat "
        f"angle is {PUBLISHED_EFFECTIVE_SEAT_ANGLE_DEG} deg"
    )


# The single geom_seatpost carried 0.35 kg before the post was split into a
# collar section and an exposed section. Lengthening a cylinder must not add
# simulated mass - the compiled total is pinned to 24.35 kg in
# test_mass_distribution.py::test_compiled_mujoco_body_masses - so the two
# sections must continue to sum to exactly this.
SEATPOST_TOTAL_MASS_KG = 0.35


def test_seatpost_split_conserves_mass(root):
    lower = next(g for g in root.iter("geom") if g.get("name") == "geom_seatpost")
    upper = next(g for g in root.iter("geom") if g.get("name") == "geom_seatpost_upper")
    total = float(lower.get("mass")) + float(upper.get("mass"))
    assert total == pytest.approx(SEATPOST_TOTAL_MASS_KG, abs=1e-9)


def test_debug_markers_do_not_change_total_mass(stand_model, stand_xml_markers):
    """Ruling R21: marker geoms had no `mass` attribute, so MuJoCo derived
    ~0.1646 kg of geometry-based mass for them. Gating them behind a flag must
    not make the bike's total mass depend on that flag, so every marker_* geom
    must carry mass="0" in addition to being gated."""
    model_off = stand_model
    model_on = mujoco.MjModel.from_xml_string(stand_xml_markers)
    assert sum(model_off.body_mass) == pytest.approx(sum(model_on.body_mass), abs=1e-9)


def test_battery_pack_is_enclosed_by_the_down_tube_box(root):
    """Verifies that the battery capsule stays inside the down-tube box it sits in.

    When the down tube became a box offset 38 mm along its own perpendicular to match
    the photographed envelope, the battery was left on the idealised BB -> head-tube
    centreline. Its axis ended up 1-3 mm outside the box wall, so the 32 mm capsule
    protruded ~34 mm for its whole length and rendered as a roll bar bolted underneath.
    Nothing caught it: the two geoms never shared a constant, and no test related them.
    """
    geoms = {g.get("name"): g for g in root.iter("geom")}
    box, battery = geoms["geom_downtube"], geoms["geom_battery_pack"]

    centre = np.array([float(v) for v in box.get("pos").split()])
    half_extents = np.array([float(v) for v in box.get("size").split()])
    pitch = np.radians(float(box.get("euler").split()[1]))
    # The box's own axes after its euler rotation about Y.
    axis_long = np.array([np.cos(pitch), 0.0, -np.sin(pitch)])
    axis_thickness = np.array([np.sin(pitch), 0.0, np.cos(pitch)])

    ends = np.array([float(v) for v in battery.get("fromto").split()])
    radius = float(battery.get("size"))

    for label, point in (("start", ends[:3]), ("end", ends[3:])):
        offset = point - centre
        across = abs(float(offset @ axis_thickness)) + radius
        along = abs(float(offset @ axis_long))
        assert across <= half_extents[2], (
            f"battery {label} stands {(across - half_extents[2]) * 1000.0:.1f} mm proud of "
            f"the down-tube box face - it is meant to sit inside the box, not on it"
        )
        assert along <= half_extents[0], (
            f"battery {label} overshoots the box by {(along - half_extents[0]) * 1000.0:.1f} mm "
            f"along the tube"
        )
