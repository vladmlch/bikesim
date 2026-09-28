"""
Unit tests for the MJCF `ride` mode: free-rolling planar chassis over a heightfield road.

Tests include:
- Planar root joint DOF count and joint types.
- Loop closure equality constraints, with no test-stand weld present.
- Wheel contact geometry: sphere contact patches vs. non-colliding tyre cylinders.
- Heightfield asset dimensions and terrain/catch-plane placement.
- Ride-only accelerometer sensors.
- Simulation timestep.
- Continued compilation of the other three modes.
"""

import mujoco
import numpy as np
import pytest

from bike_sim.geometry.hardpoints import compute_ground_z
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.terrain.heightfield import FIELD


@pytest.fixture(scope="module")
def ride_model():
    """Compiles the ride-mode MJCF model once for the whole test module."""
    xml_str = generate_mujoco_xml(mode="ride", include_rider=True)
    return mujoco.MjModel.from_xml_string(xml_str)


def _ground_z_m() -> float:
    return compute_ground_z(BikeSpecs()) / 1000.0


@pytest.fixture(scope="module")
def seated_ride_model():
    """Compiles the ride-mode MJCF model with the seated rider once for the module."""
    return mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode="ride", rider="seated"))


def test_ride_mode_dof_count(ride_model):
    """With the lumped rider, ride mode has exactly 12 generalized coordinates and 12 DOFs."""
    assert ride_model.nq == 12
    assert ride_model.nv == 12


def test_seated_ride_mode_adds_one_slide_per_rider_mass(seated_ride_model):
    """
    The seated rider adds five vertical slides -- pelvis, torso, arms, two legs -- for 17.

    docs/RIDE.md section 1 table: 12 coordinates for the bike, plus one per lumped rider mass.
    """
    assert seated_ride_model.nq == 17
    assert seated_ride_model.nv == 17
    for joint_name in ("rider_pelvis_z", "rider_torso_z", "rider_arms_z", "rider_leg_front_z", "rider_leg_rear_z"):
        jid = mujoco.mj_name2id(seated_ride_model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        assert jid >= 0, joint_name
        assert seated_ride_model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_SLIDE
        assert not seated_ride_model.jnt_limited[jid]
        assert np.allclose(seated_ride_model.jnt_axis[jid], [0.0, 0.0, 1.0])
    # The torso rides on the pelvis; every other rider body hangs off the frame.
    body = lambda n: mujoco.mj_name2id(seated_ride_model, mujoco.mjtObj.mjOBJ_BODY, n)
    assert seated_ride_model.body_parentid[body("rider_torso")] == body("rider_pelvis")
    for name in ("rider_pelvis", "rider_arms", "rider_leg_front", "rider_leg_rear"):
        assert seated_ride_model.body_parentid[body(name)] == body("frame")


def test_seated_rider_geoms_do_not_collide(seated_ride_model):
    """Every rider geom is visual and mass only: rider-ground contact is a crash, not a collision."""
    for gid in range(seated_ride_model.ngeom):
        name = mujoco.mj_id2name(seated_ride_model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        if name.startswith("geom_rider_"):
            assert seated_ride_model.geom_contype[gid] == 0 and seated_ride_model.geom_conaffinity[gid] == 0, name


def test_seated_rider_accelerometers(seated_ride_model, ride_model):
    """The seated rider carries torso and pelvis accelerometers; the lumped rider does not."""
    for sensor in ("sensor_rider_torso_accel", "sensor_rider_pelvis_accel"):
        assert mujoco.mj_name2id(seated_ride_model, mujoco.mjtObj.mjOBJ_SENSOR, sensor) >= 0
        assert mujoco.mj_name2id(ride_model, mujoco.mjtObj.mjOBJ_SENSOR, sensor) < 0


def test_cranks_and_pedals_are_present_and_visual_only(ride_model):
    """165 mm horizontal cranks with a pedal on each end, welded to the frame and non-colliding."""
    frame_id = mujoco.mj_name2id(ride_model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    for name in ("geom_crank_spindle", "geom_crank_arm_front", "geom_crank_arm_rear", "geom_pedal_front", "geom_pedal_rear"):
        gid = mujoco.mj_name2id(ride_model, mujoco.mjtObj.mjOBJ_GEOM, name)
        assert gid >= 0, name
        assert ride_model.geom_bodyid[gid] == frame_id
        assert ride_model.geom_contype[gid] == 0 and ride_model.geom_conaffinity[gid] == 0
    front = mujoco.mj_name2id(ride_model, mujoco.mjtObj.mjOBJ_GEOM, "geom_pedal_front")
    rear = mujoco.mj_name2id(ride_model, mujoco.mjtObj.mjOBJ_GEOM, "geom_pedal_rear")
    assert ride_model.geom_pos[front][0] == pytest.approx(0.165)
    assert ride_model.geom_pos[rear][0] == pytest.approx(-0.165)
    assert ride_model.geom_pos[front][2] == pytest.approx(0.0) and ride_model.geom_pos[rear][2] == pytest.approx(0.0)
    assert mujoco.mj_name2id(ride_model, mujoco.mjtObj.mjOBJ_GEOM, "geom_crank_arms") < 0


def test_ride_mode_root_joints(ride_model):
    """The three planar chassis root joints exist with the expected joint types."""
    expected_types = {
        "root_x": mujoco.mjtJoint.mjJNT_SLIDE,
        "root_z": mujoco.mjtJoint.mjJNT_SLIDE,
        "root_pitch": mujoco.mjtJoint.mjJNT_HINGE,
    }
    for joint_name, expected_type in expected_types.items():
        jid = mujoco.mj_name2id(ride_model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        assert jid >= 0, f"joint {joint_name} not found in ride model"
        assert ride_model.jnt_type[jid] == expected_type


def test_ride_mode_equality_constraints(ride_model):
    """Only the two loop-closure connects are present; no stand_clamp weld in ride mode."""
    assert ride_model.neq == 2
    eq_names = {
        mujoco.mj_id2name(ride_model, mujoco.mjtObj.mjOBJ_EQUALITY, i)
        for i in range(ride_model.neq)
    }
    assert "stand_clamp" not in eq_names
    assert eq_names == {"seatstay_rocker_joint", "shock_frame_joint"}


@pytest.mark.parametrize(
    "contact_geom,radius_m",
    [("geom_front_contact", 0.372), ("geom_rear_contact", 0.352)],
)
def test_ride_mode_wheel_contact_spheres(ride_model, contact_geom, radius_m):
    """Each wheel has a colliding sphere contact patch of the specified radius."""
    gid = mujoco.mj_name2id(ride_model, mujoco.mjtObj.mjOBJ_GEOM, contact_geom)
    assert gid >= 0, f"{contact_geom} not found in ride model"
    assert ride_model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_SPHERE
    assert ride_model.geom_size[gid][0] == pytest.approx(radius_m)
    assert ride_model.geom_contype[gid] == 1
    assert ride_model.geom_conaffinity[gid] == 1


@pytest.mark.parametrize("tire_geom", ["geom_front_tire", "geom_rear_tire"])
def test_ride_mode_tyre_cylinders_noncolliding(ride_model, tire_geom):
    """The tyre cylinders are non-colliding in ride mode: only the contact spheres collide."""
    gid = mujoco.mj_name2id(ride_model, mujoco.mjtObj.mjOBJ_GEOM, tire_geom)
    assert gid >= 0, f"{tire_geom} not found in ride model"
    assert ride_model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_CYLINDER
    assert ride_model.geom_contype[gid] == 0
    assert ride_model.geom_conaffinity[gid] == 0


def test_ride_mode_contact_sphere_tracks_wheel_radius():
    """Contact sphere radii follow BikeSpecs wheel radii, not a hardcoded constant."""
    specs = BikeSpecs(front_wheel_radius=400.0, rear_wheel_radius=380.0)
    xml_str = generate_mujoco_xml(specs=specs, mode="ride", include_rider=True)
    model = mujoco.MjModel.from_xml_string(xml_str)

    for contact_geom, radius_mm in [
        ("geom_front_contact", specs.front_wheel_radius),
        ("geom_rear_contact", specs.rear_wheel_radius),
    ]:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, contact_geom)
        assert gid >= 0, f"{contact_geom} not found in ride model"
        assert model.geom_size[gid][0] == pytest.approx(radius_mm / 1000.0)


def test_ride_mode_hfield_dimensions(ride_model):
    """The road hfield asset matches FIELD's fixed nrow/ncol grid dimensions."""
    hid = mujoco.mj_name2id(ride_model, mujoco.mjtObj.mjOBJ_HFIELD, "road")
    assert hid >= 0, "road hfield asset not found in ride model"
    assert ride_model.hfield_nrow[hid] == FIELD.nrow
    assert ride_model.hfield_ncol[hid] == FIELD.ncol


def test_ride_mode_timestep(ride_model):
    """Ride mode uses the finer 0.0005 s timestep required for hard contact solving."""
    assert ride_model.opt.timestep == pytest.approx(0.0005)


def test_ride_mode_catch_plane_below_field_floor(ride_model):
    """The runaway catch plane sits strictly below the heightfield's own floor."""
    gid = mujoco.mj_name2id(ride_model, mujoco.mjtObj.mjOBJ_GEOM, "catch_plane")
    assert gid >= 0, "catch_plane geom not found in ride model"
    ground_z_m = _ground_z_m()
    catch_plane_z = float(ride_model.geom_pos[gid][2])
    assert catch_plane_z < FIELD.geom_z_m(ground_z_m)


def test_ride_mode_accelerometers_resolve(ride_model):
    """Both ride-only accelerometers exist and target the correct sites."""
    for sensor_name, site_name in [
        ("sensor_bar_accel", "site_handlebar"),
        ("sensor_saddle_accel", "site_seatpost_top"),
    ]:
        sid = mujoco.mj_name2id(ride_model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
        assert sid >= 0, f"{sensor_name} not found in ride model"
        assert ride_model.sensor_type[sid] == mujoco.mjtSensor.mjSENS_ACCELEROMETER

        site_id = mujoco.mj_name2id(ride_model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        assert site_id >= 0, f"{site_name} site not found in ride model"
        assert ride_model.sensor_objid[sid] == site_id


@pytest.mark.parametrize("mode", ["standard", "stand", "playground"])
def test_other_modes_still_compile(mode):
    """The other three modes remain unaffected and continue to compile cleanly."""
    xml_str = generate_mujoco_xml(mode=mode)
    model = mujoco.MjModel.from_xml_string(xml_str)
    assert model is not None
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)


@pytest.fixture(scope="module")
def articulated_ride_model():
    from bike_sim.physics.rider import RiderSpecs
    rider = RiderSpecs(variant="seated", legs="articulated")
    xml = generate_mujoco_xml(mode="ride", rider=rider, crank_joint=True)
    return mujoco.MjModel.from_xml_string(xml)


def test_articulated_leg_topology(articulated_ride_model):
    m = articulated_ride_model
    # 12 bike DOF + 3 rider slides + 6 leg hinges + 2 pedal hinges + crank_spin = 24
    assert m.nq == 24 and m.nv == 24
    for jname in ("rider_hip_front", "rider_knee_front", "rider_ankle_front",
                  "rider_hip_rear", "rider_knee_rear", "rider_ankle_rear",
                  "pedal_spin_front", "pedal_spin_rear", "crank_spin"):
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname)
        assert jid >= 0, jname
        assert m.jnt_type[jid] == mujoco.mjtJoint.mjJNT_HINGE
    body = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    assert m.body_parentid[body("rider_thigh_front")] == body("rider_pelvis")
    assert m.body_parentid[body("rider_shank_front")] == body("rider_thigh_front")
    assert m.body_parentid[body("rider_foot_front")] == body("rider_shank_front")
    assert m.body_parentid[body("pedal_front")] == body("crank")
    # leg slide bodies are gone
    for jname in ("rider_leg_front_z", "rider_leg_rear_z"):
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname) < 0


def test_foot_pedal_welds_present_and_satisfied(articulated_ride_model):
    m = articulated_ride_model
    welds = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_EQUALITY, i)
             for i in range(m.neq) if m.eq_type[i] == mujoco.mjtEq.mjEQ_WELD}
    assert {"weld_foot_front", "weld_foot_rear"} <= welds
    # built pose is the weld datum: forward at qpos0 must put each foot's weld
    # site on its pedal site (~zero separation)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    for side in ("front", "rear"):
        sf = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"site_foot_{side}")
        sp = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"site_pedal_{side}")
        sep = float(np.linalg.norm(d.site_xpos[sf] - d.site_xpos[sp]))
        assert sep < 0.001, f"{side}: {sep * 1000:.2f} mm"
