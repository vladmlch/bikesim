"""
Unit tests for eMTB physical mass distribution and Center of Gravity (CG) modeling.

Tests include:
- Total bike mass validation (~24.5 kg full-power eMTB specification).
- Component mass breakdown and subsystem inertia properties.
- Analytical and MuJoCo Center of Gravity (CG) location relative to BB origin.
- Static front / rear axle load distribution and moment equilibrium.
- Wheel rotational inertia calculation (rim + tire outer mass concentration).
- Kinematic loop closure integrity in MuJoCo under mass and gravity dynamics.
"""

import xml.etree.ElementTree as ET

import pytest
import numpy as np
import mujoco

from bike_sim.geometry.hardpoints import compute_front_axle, compute_rear_axle
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.mujoco.exporter import export_playground_models
from bike_sim.physics.mass import (
    BikeMassSpecs,
    compute_static_system_cg,
    compute_component_centers_of_mass,
    compute_suspension_tuning_for_sag,
)

# Rear-sag load convention used throughout this module: 35 % of static system
# weight on the front wheel, 65 % on the rear - the seated rear-sag split bike
# manufacturers publish. It is the split the whole tuning chain in this repository
# was derived from: BikeSpecs.fork_initial_psi = 85.2 reproduces
# ForkAirSpring.calibrate_psi_for_sag at this split to 0.011 % (85.1908 PSI), and
# the pre-refit BikeSpecs.shock_stiffness of 108 600 N/m reproduces the pre-refit
# leverage curve's requirement to 0.019 % (108 620.9 N/m).
#
# KNOWN INCONSISTENCY, recorded here because it outlives this test: it is NOT the
# split the bike model itself produces. compute_static_system_cg puts 46.4 % of
# static weight on the front for the 80 kg rider in RiderSpecs' standing attack
# pose, and the dynamic MuJoCo model measures ~48.8 %. Springs calibrated for
# 35/65 but loaded 48.8/51.2 therefore do not sag 30/30 in the simulator: sampled
# over 24 s of dynamic settling the fork rides 39-45 % of travel (30 % x 48.8/35 =
# 41.8 % predicted) and the rear 21-25 % (30 % x 51.2/65 = 23.6 % predicted). The
# discrepancy is one root cause at both ends, it predates the photo refit, and
# resolving it means re-deriving fork_initial_psi AND shock_stiffness together
# against the model's own CoM - a ride-feel decision, not a bug fix.
REAR_SAG_FRONT_WEIGHT_FRACTION = 0.35


@pytest.fixture(scope="module")
def mass_setup(tmp_path_factory):
    """Sets up specs, mass specs, and compiled MuJoCo models."""
    temp_dir = tmp_path_factory.mktemp("mass_models")
    specs = BikeSpecs()
    mass_specs = BikeMassSpecs()
    solver = HorstLinkageSolver(specs)
    models = export_playground_models(output_dir=temp_dir, specs=specs, solver=solver)

    mj_models = {}
    for mode in ["standard", "stand", "playground"]:
        m = mujoco.MjModel.from_xml_string(models[mode])
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        mj_models[mode] = (m, d)

    return {
        "specs": specs,
        "mass_specs": mass_specs,
        "solver": solver,
        "mj_models": mj_models,
        "cg_info": compute_static_system_cg(specs, mass_specs),
    }


def test_total_bike_mass_specification(mass_setup):
    """Verifies that the target full-power eMTB mass is 24.5 kg (+- 0.5 kg)."""
    mass_specs: BikeMassSpecs = mass_setup["mass_specs"]
    total_specs_mass = mass_specs.total_bike_mass

    assert 24.0 <= total_specs_mass <= 25.0, f"Expected total mass ~24.5 kg, got {total_specs_mass:.2f} kg"
    assert mass_specs.motor_mass == 2.90
    assert mass_specs.battery_mass == 4.30


def test_compiled_mujoco_body_masses(mass_setup):
    """Verifies that the compiled MuJoCo model matches the physical mass breakdown."""
    mass_specs: BikeMassSpecs = mass_setup["mass_specs"]
    model, data = mass_setup["mj_models"]["standard"]
    total_mj_mass = float(sum(model.body_mass))

    # Known, deliberate discrepancy - pinned rather than papered over with a 1 kg
    # window. The compiled model weighs 24.35 kg; BikeMassSpecs budgets 24.40 kg,
    # and it is the 24.40 figure the sag and spring-rate maths in mass_profile runs
    # off. Every body except `frame` is generated directly from BikeMassSpecs and
    # matches it exactly. `frame` does not: its twelve hand-authored front-triangle
    # structural geoms (bb shell, down tube, head tube, both top tubes, seat tube,
    # casting, gusset, junction, three mount bosses) sum to 3.15 kg against
    # frame_structure_mass = 3.20 kg. Nothing derives those twelve numbers from the
    # design table, so the 0.05 kg has been adrift since mass_profile.py was
    # introduced; it was masked by ~0.083 kg of phantom debug-marker mass until
    # a83b8e8 gave the markers mass="0".
    #
    # Effect on tuning is 0.05/104.4 = 0.048 % on the spring rates and exactly zero
    # on the bottom-out g-loads (which cancel mass entirely). Closing it means
    # re-deriving those twelve geom masses from frame_structure_mass, which moves the
    # frame body's CoM and inertia and belongs with the frame-visual work, not here.
    # This assertion is the tripwire that stops it drifting further in the meantime.
    FRAME_STRUCTURE_SHORTFALL_KG = 0.05
    assert total_mj_mass == pytest.approx(
        mass_specs.total_bike_mass - FRAME_STRUCTURE_SHORTFALL_KG, abs=1e-9
    ), (
        f"Compiled MuJoCo mass {total_mj_mass:.4f} kg vs design table "
        f"{mass_specs.total_bike_mass:.4f} kg: the gap is no longer the known "
        f"{FRAME_STRUCTURE_SHORTFALL_KG:.2f} kg front-triangle shortfall"
    )

    # Verify key bodies exist with positive realistic masses
    body_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)]
    assert "frame" in body_names
    assert "front_wheel" in body_names
    assert "rear_wheel" in body_names
    assert "chainstay" in body_names
    assert "seatstay" in body_names
    assert "rocker" in body_names
    assert "shock_yoke" in body_names

    # Check frame body mass (contains motor + battery + frame structure)
    frame_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    assert 11.5 <= model.body_mass[frame_id] <= 13.0

    # Check front wheel mass
    fw_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "front_wheel")
    assert 2.3 <= model.body_mass[fw_id] <= 2.5

    # Check rear wheel mass
    rw_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rear_wheel")
    assert 2.7 <= model.body_mass[rw_id] <= 2.9


def test_center_of_gravity_location(mass_setup):
    """Verifies the calculated Center of Gravity (CG) is in front of BB and low."""
    cg_info = mass_setup["cg_info"]
    cg_pos = cg_info["cg_pos_m"]

    # X_cg should be positive (in front of BB) and within realistic eMTB range
    assert 0.040 <= cg_pos[0] <= 0.180, f"Expected X_cg between +40 and +180 mm, got {cg_pos[0]*1000:.1f} mm"

    # Z_cg should be positive (above BB) and within low-center-of-gravity range
    assert 0.140 <= cg_pos[2] <= 0.220, f"Expected Z_cg between +140 and +220 mm, got {cg_pos[2]*1000:.1f} mm"

    # Y_cg should be zero (bilaterally symmetric)
    assert abs(cg_pos[1]) < 1e-6, f"Expected symmetric CG (Y=0), got Y={cg_pos[1]}"


def test_static_axle_load_distribution(mass_setup):
    """Verifies static front and rear wheel normal loads on flat ground."""
    cg_info = mass_setup["cg_info"]
    f_front_pct = cg_info["front_load_pct"]
    f_rear_pct = cg_info["rear_load_pct"]

    # Total load percentages must sum to 100%
    assert abs((f_front_pct + f_rear_pct) - 100.0) < 1e-4

    # Front wheel load: 40% to 50%
    assert 40.0 <= f_front_pct <= 50.0, f"Expected front load 40-50%, got {f_front_pct:.1f}%"

    # Rear wheel load: 50% to 60%
    assert 50.0 <= f_rear_pct <= 60.0, f"Expected rear load 50-60%, got {f_rear_pct:.1f}%"


def test_wheel_rotational_inertia_computation(mass_setup):
    """Verifies wheel rotational inertia model accounts for rim/tire outer mass concentration."""
    mass_specs: BikeMassSpecs = mass_setup["mass_specs"]
    specs: BikeSpecs = mass_setup["specs"]

    r_f = specs.front_wheel_radius / 1000.0 # 0.372 m
    r_r = specs.rear_wheel_radius / 1000.0  # 0.352 m

    i_yy_front = mass_specs.compute_wheel_rotational_inertia(mass_specs.front_wheel_mass, r_f)
    i_yy_rear = mass_specs.compute_wheel_rotational_inertia(mass_specs.rear_wheel_mass, r_r)

    # A solid uniform disk would have I = 0.5 * m * R^2
    i_solid_front = 0.5 * mass_specs.front_wheel_mass * (r_f ** 2)
    i_solid_rear = 0.5 * mass_specs.rear_wheel_mass * (r_r ** 2)

    # Concentrated rim/tire gives ~1.5x higher rotational inertia than a solid disk
    assert i_yy_front > i_solid_front * 1.4, "Front wheel I_yy should reflect rim/tire mass concentration"
    assert i_yy_rear > i_solid_rear * 1.4, "Rear wheel I_yy should reflect rim/tire mass concentration"

    # Typical 29" wheel rotational inertia is ~0.20 - 0.28 kg*m^2
    assert 0.20 <= i_yy_front <= 0.30
    assert 0.22 <= i_yy_rear <= 0.32


def test_loop_closure_tightness_with_mass(mass_setup):
    """Verifies that loop closure constraints stay tightly closed with physical masses."""
    for mode in ["standard", "stand", "playground"]:
        model, data = mass_setup["mj_models"][mode]
        # Step forward for 100 steps
        for _ in range(100):
            mujoco.mj_step(model, data)

        site_ss = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "site_P3_ss")
        site_roc = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "site_P3_rocker")
        assert site_ss >= 0 and site_roc >= 0

        p3_ss = data.site_xpos[site_ss]
        p3_roc = data.site_xpos[site_roc]
        p3_res_mm = np.linalg.norm(p3_ss - p3_roc) * 1000.0

        limit_mm = 0.5
        assert p3_res_mm < limit_mm, f"Mode {mode}: P3 loop closure error {p3_res_mm:.4f} mm exceeds {limit_mm} mm limit"


def test_suspension_sag_tuning_and_balanced_bottom_out(mass_setup):
    """Verifies suspension stiffness calculation for 30% sag and balanced bottom-out g-loads."""
    specs: BikeSpecs = mass_setup["specs"]
    mass_specs: BikeMassSpecs = mass_setup["mass_specs"]

    tuning = compute_suspension_tuning_for_sag(
        specs=specs,
        mass_specs=mass_specs,
        rider_mass_kg=80.0,
        target_front_sag_pct=30.0,
        target_rear_sag_pct=30.0,
        front_weight_fraction=REAR_SAG_FRONT_WEIGHT_FRACTION,
    )

    # 30% sag of 180mm is 54mm
    assert abs(tuning["target_front_sag_mm"] - 54.0) < 1e-4
    assert abs(tuning["target_rear_sag_mm"] - 54.0) < 1e-4

    # --- Quantities that ARE tunable -------------------------------------------
    #
    # k_fork = F_front*sin(head_angle) / sag  and  k_shock = F_rear*LR(sag)/stroke(sag),
    # so both scale linearly with system mass and with the load split. They are pinned
    # to +-2 %, damping (which goes as sqrt(k*m)) to +-3 %.
    #
    # Why 2 %: 2 % of the 104.4 kg system is 2.09 kg. That absorbs any ordinary
    # component re-spec - including the 0.05 kg by which the compiled model trails
    # the design table (0.048 %, see test_compiled_mujoco_body_masses) - while still
    # catching a change to the leverage curve: the photo refit moved k_shock 5.5 %
    # (108 620.9 -> 114 589.6 N/m), which these bounds would have failed on.
    assert tuning["fork_stiffness_n_m"] == pytest.approx(5966.3, rel=0.02)
    assert tuning["shock_stiffness_n_m"] == pytest.approx(114589.6, rel=0.02)
    assert tuning["fork_damping_n_s_m"] == pytest.approx(441.3, rel=0.03)
    assert tuning["shock_damping_n_s_m"] == pytest.approx(1178.1, rel=0.03)

    # --- Quantities that are NOT tunable ---------------------------------------
    #
    # Both bottom-out g-loads are outputs of the locked geometry and the 30 % sag
    # target alone. No spring rate, rider mass, bike mass, damping setting or load
    # split can move them, so there is nothing here to "balance" and no honest
    # reason to leave slack in the bound. Substituting the spring rates back:
    #
    #   front:  k_fork = F_front*sin(t)/(s*L);  F(L)/F_front = k_fork*L/(sin(t)*F_front)
    #           = 1/s exactly. Head angle, fork travel and load all cancel.
    #
    #   rear:   k = F_rear*LR(sag)/stroke(sag);  F(180)/F_rear
    #           = LR(sag)*stroke(180) / (stroke(sag)*LR(180))
    #           = 2.959292 * 65.00004 / (17.191964 * 2.411763) = 4.639181.
    #           F_rear cancels; only the fitted leverage curve and the sag target remain.
    #
    # rel=1e-6 is ~5 orders above the ~1e-11 relative noise of the solver's Brent
    # root-find (xtol=1e-12 rad over ~400 mm/rad), and corresponds to a sub-micron
    # move of any hardpoint - i.e. it trips on a real geometry edit and nothing else.
    # The hardpoints themselves are locked by
    # tests/test_kinematics.py::TestPublishedGeometryInvariants.
    assert tuning["front_bottom_out_g"] == pytest.approx(1.0 / 0.30, rel=1e-12), (
        f"Front bottom-out g is identically 1/sag_fraction; got {tuning['front_bottom_out_g']:.6f}"
    )
    assert tuning["rear_bottom_out_g"] == pytest.approx(4.639181, rel=1e-6), (
        f"Rear bottom-out g {tuning['rear_bottom_out_g']:.6f} no longer matches the "
        f"photo-fitted leverage curve (docs/reference/fitted_hardpoints.json)"
    )

    # The ratio of the two is therefore the identity 0.30 * rear_bottom_out_g and
    # carries no information the two assertions above do not already carry. Kept
    # only to state that explicitly, so it is not mistaken for a design check.
    g_ratio = tuning["rear_bottom_out_g"] / tuning["front_bottom_out_g"]
    assert g_ratio == pytest.approx(0.30 * tuning["rear_bottom_out_g"], rel=1e-12)
    assert g_ratio == pytest.approx(1.391754, rel=1e-6)

    # The physically meaningful front/rear bottom-out comparison. front_bottom_out_g
    # above is a linear-fork idealisation that the simulator never uses: the fork it
    # runs is the progressive DebonAir+ air spring, which ramps to 6.11 g. So the
    # modelled bike bottoms out harder at the front than the rear, not softer, and
    # both ends are above the 3 g a linear reading suggests.
    assert tuning["fork_air_bottom_out_g"] == pytest.approx(6.1069, rel=0.02)
    air_g_ratio = tuning["rear_bottom_out_g"] / tuning["fork_air_bottom_out_g"]
    assert 0.70 <= air_g_ratio <= 0.85, (
        f"Rear/front bottom-out balance on the modelled springs is {air_g_ratio:.3f}"
    )


def _solve_equilibrium_rear_travel(solver, shock_k_n_m, f_rear_n, lo_mm=0.05, hi_mm=179.95):
    """
    Bisects the static rear force balance for the wheel travel where the shock
    spring force, reduced to the wheel by the leverage ratio, carries `f_rear_n`.

    Residual r(x) = k * stroke(x) / LR(x) - F_rear. stroke(x)/LR(x) is strictly
    increasing over the whole travel range (verified below), so the root is unique.
    """
    def residual(x_mm):
        st = solver.solve_state_from_wheel_travel(float(x_mm))
        return shock_k_n_m * (st["shock_stroke"] / 1000.0) / st["leverage_ratio"] - f_rear_n

    r_lo, r_hi = residual(lo_mm), residual(hi_mm)
    assert r_lo < 0.0 < r_hi, (
        f"Static rear load {f_rear_n:.1f} N is not bracketed by the spring over "
        f"0..180 mm of travel (residuals {r_lo:.1f} .. {r_hi:.1f} N)"
    )
    for _ in range(200):
        mid = 0.5 * (lo_mm + hi_mm)
        if residual(mid) <= 0.0:
            lo_mm = mid
        else:
            hi_mm = mid
        if hi_mm - lo_mm < 1e-9:
            break
    return 0.5 * (lo_mm + hi_mm)


def test_rear_sag_is_30_percent_under_static_load(mass_setup):
    """
    Sag is a force balance, not a stroke ratio.

    The simulator drives the rear shock with a spring that is linear in shock
    stroke at rate BikeSpecs.shock_stiffness (run_playground.py, both the applied
    qfrc and the telemetry). Static sag is therefore the travel x that solves

        shock_stiffness * stroke(x) / LR(x) == F_rear

    where F_rear is the rear share of the static system weight. This test finds
    that root over the full 0..180 mm range and asserts it lands on 30 % of
    travel, so it exercises the spring rate, the published stroke/LR curve and
    the mass model together. Checking only that 30 % of travel consumes ~30 % of
    stroke would be a statement about the kinematics alone and would pass with
    any spring rate whatsoever, including none.

    Load convention: 65 % of the total system weight on the rear wheel, the
    seated rear-sag convention this repository already uses (it is the
    front_weight_fraction=0.35 passed to compute_suspension_tuning_for_sag in
    test_suspension_sag_tuning_and_balanced_bottom_out, and it reproduces the
    pre-refit shock_stiffness of 108 600 N/m to 0.02 % on the pre-refit curve).
    """
    specs: BikeSpecs = mass_setup["specs"]
    mass_specs: BikeMassSpecs = mass_setup["mass_specs"]
    solver: HorstLinkageSolver = mass_setup["solver"]

    total_mass_kg = mass_specs.total_bike_mass + 80.0
    f_rear_n = total_mass_kg * 9.81 * (1.0 - REAR_SAG_FRONT_WEIGHT_FRACTION)

    # stroke(x)/LR(x) must rise monotonically, otherwise the equilibrium above is
    # not unique and "sag" is not a well-defined quantity on this linkage.
    ratios = [
        (lambda st: st["shock_stroke"] / st["leverage_ratio"])(
            solver.solve_state_from_wheel_travel(float(x))
        )
        for x in np.linspace(0.0, specs.rear_wheel_travel, 61)
    ]
    assert np.all(np.diff(ratios) > 0.0), "stroke/LR is not monotonic; sag is not unique"

    sag_mm = _solve_equilibrium_rear_travel(solver, specs.shock_stiffness, f_rear_n)
    sag_pct = 100.0 * sag_mm / specs.rear_wheel_travel

    # +-0.5 mm about 54 mm is +-0.28 percentage points of sag. A rider setting sag
    # by hand cannot read finer than that on a 180 mm bike, and it is 200x the
    # ~2e-3 mm the bisection above resolves, so only a real change in the spring
    # rate, the leverage curve or the mass model can move it.
    assert abs(sag_mm - 54.0) < 0.5, (
        f"Rear settles at {sag_mm:.3f} mm ({sag_pct:.2f} % of travel) under "
        f"{f_rear_n:.1f} N with k = {specs.shock_stiffness:.1f} N/m; 30 % sag is 54.0 mm"
    )


def test_kinematic_hardpoints_consistency_with_solver(mass_setup):
    """Verifies that mass_profile component CoMs derive directly from HorstLinkageSolver."""
    specs: BikeSpecs = mass_setup["specs"]
    mass_specs: BikeMassSpecs = mass_setup["mass_specs"]
    solver: HorstLinkageSolver = mass_setup["solver"]

    st0 = solver.solve_state_from_wheel_travel(0.0)
    components = compute_component_centers_of_mass(specs, mass_specs, solver=solver)

    p0 = np.array(mass_setup["cg_info"]["rear_axle_x"])  # reference
    p4 = np.array(st0["P4"]) / 1000.0
    p6 = np.array(st0["P6"]) / 1000.0
    expected_pos_yoke = 0.5 * (p4 + p6)

    actual_pos_yoke = components["shock_yoke"][0]
    assert np.allclose(actual_pos_yoke, expected_pos_yoke, atol=1e-6), (
        f"Shock yoke CoM {actual_pos_yoke} does not match solver midpoint {expected_pos_yoke}"
    )


def test_rider_inclusion_in_static_cg(mass_setup):
    """Verifies that RiderSpecs properly updates system mass, CG, and front load percentage."""
    from bike_sim.physics.mass import RiderSpecs

    specs: BikeSpecs = mass_setup["specs"]
    mass_specs: BikeMassSpecs = mass_setup["mass_specs"]
    rider = RiderSpecs()

    cg_bike_only = compute_static_system_cg(specs, mass_specs, rider_specs=None)
    cg_with_rider = compute_static_system_cg(specs, mass_specs, rider_specs=rider)

    assert abs(cg_bike_only["total_mass_kg"] - mass_specs.total_bike_mass) < 1e-4
    assert abs(cg_with_rider["total_mass_kg"] - (mass_specs.total_bike_mass + 80.0)) < 1e-4

    # Rider raises the center of mass significantly
    assert cg_with_rider["cg_pos_m"][2] > cg_bike_only["cg_pos_m"][2] + 0.20
    # Combined front load percentage stays realistic ~ 45-48%
    assert 44.0 <= cg_with_rider["front_load_pct"] <= 49.0



COMPONENT_GEOMS = {
    "motor": ("geom_motor_core",),
    "battery": ("geom_battery_pack",),
    "crank_pedals": ("geom_crank_arms",),
    "saddle_post": ("geom_seatpost", "geom_seatpost_upper", "geom_saddle"),
    "frame_structure": (
        "geom_bb_shell",
        "geom_downtube",
        "geom_headtube",
        "geom_toptube_bridge",
        "geom_toptube_main",
        "geom_seattube",
        "geom_frame_casting",
        "geom_seat_gusset",
        "geom_frame_junction",
        "geom_p0_mount",
        "geom_p5_mount",
        "geom_p7_mount",
    ),
}


def _geom_centre(geom):
    """Returns a geom's centre in metres, from either its fromto or its pos."""
    fromto = geom.get("fromto")
    if fromto is not None:
        ends = np.array([float(v) for v in fromto.split()])
        return 0.5 * (ends[:3] + ends[3:])
    return np.array([float(v) for v in (geom.get("pos") or "0 0 0").split()])


def test_analytic_component_positions_match_the_built_model(mass_setup):
    """Verifies that every mass_profile component CoM tracks the geoms the exporter emits.

    mass_profile describes the bike analytically and export_mujoco builds it; nothing in
    the language couples the two, so a constant in one can quietly stop describing the
    other. Every position in the table below had drifted at some point - the frame
    structure by 150 mm, the battery by 40 mm, the saddle by 101 mm - and none of it was
    visible, because the only assertion downstream was a centre-of-gravity window wide
    enough to swallow all of it at once.

    Comparing against the compiled geoms rather than against a second copy of the same
    constants is what makes this a real check.
    """
    specs: BikeSpecs = mass_setup["specs"]
    mass_specs: BikeMassSpecs = mass_setup["mass_specs"]
    solver: HorstLinkageSolver = mass_setup["solver"]

    root = ET.fromstring(generate_mujoco_xml(specs=specs, solver=solver, mode="stand"))
    built = {g.get("name"): g for g in root.iter("geom") if g.get("name")}
    analytic = compute_component_centers_of_mass(specs, mass_specs, solver)

    for component, geom_names in COMPONENT_GEOMS.items():
        missing = [n for n in geom_names if n not in built]
        assert not missing, f"{component} names geoms absent from the model: {missing}"

        mass = sum(float(built[n].get("mass")) for n in geom_names)
        centre = sum(float(built[n].get("mass")) * _geom_centre(built[n]) for n in geom_names) / mass

        analytic_pos, analytic_mass = analytic[component]
        # frame_structure carries a known, deliberately unclosed 0.05 kg shortfall: its
        # twelve hand-authored geoms sum to 3.15 kg against frame_structure_mass = 3.20.
        # It predates mass_profile.py and was masked by phantom debug-marker mass until
        # the markers were given mass="0". Closing it means re-deriving all twelve, which
        # moves the frame CoM and breaks two frame-visual guards for a 0.048 % effect on
        # spring rates and none at all on the bottom-out g-loads. Pinned exactly here so
        # it cannot drift further while it stays open.
        expected_mass = analytic_mass - (0.05 if component == "frame_structure" else 0.0)
        assert mass == pytest.approx(expected_mass, abs=1e-9), (
            f"{component}: built geoms weigh {mass:.6f} kg against an expected "
            f"{expected_mass:.6f} kg (design table says {analytic_mass:.6f} kg)"
        )
        # 2 mm, not 5: the motor's drift is exactly 5.0 mm on X (5.6 mm as a norm), so a
        # 5 mm per-element bound let that one mutation through when this test was first
        # written. Every group is exact today, and the offsets are published to six
        # decimals - sub-micron - so 2 mm is still three orders clear of rounding.
        assert np.allclose(analytic_pos, centre, atol=0.002), (
            f"{component}: mass_profile puts its CoM at "
            f"{np.array2string(analytic_pos, precision=6)} but the exporter builds it at "
            f"{np.array2string(centre, precision=6)} "
            f"(off by {np.linalg.norm(analytic_pos - centre) * 1000.0:.1f} mm)"
        )
