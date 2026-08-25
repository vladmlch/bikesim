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


def test_ride_mode_dof_count(ride_model):
    """Ride mode has exactly 12 generalized coordinates and 12 velocity DOFs."""
    assert ride_model.nq == 12
    assert ride_model.nv == 12


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
