"""Physical wheel contact forces and absolute contact-point kinematics."""

from dataclasses import FrozenInstanceError

import mujoco
import numpy as np
import pytest

from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.sim.ride.contact_state import ContactPatch, WheelContactSnapshot
from bike_sim.sim.ride.contacts import TerrainContactQuery, _tracked_wrench
from bike_sim.sim.ride.wheel_kinematics import point_velocity, wheel_point_velocity
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


def test_no_slip_uses_world_wheel_velocity():
    v = point_velocity(np.array([3.5, 0., 0.]), np.array([0., 10., 0.]), np.array([0., 0., -0.35]))
    np.testing.assert_allclose(v, np.zeros(3), atol=1e-12)


@pytest.mark.parametrize("bad", [np.zeros(2), np.array([1., np.nan, 0.])])
def test_point_velocity_rejects_invalid_vectors(bad):
    with pytest.raises(ValueError, match="finite 3D vectors"):
        point_velocity(np.zeros(3), bad, np.zeros(3))


def test_slope_keeps_normal_and_vertical_force_distinct():
    n = np.array([-0.5, 0., np.sqrt(3) / 2])
    patch = ContactPatch(np.zeros(3), n, 100.0, 20.0, 0.0)
    assert patch.normal_load_n == 100.0
    assert patch.world_force_n[2] == pytest.approx(100 * np.sqrt(3) / 2 + 10)
    snapshot = WheelContactSnapshot(0., (patch,), True)
    assert snapshot.normal_load_n == 100.
    assert snapshot.normal_vertical_n == pytest.approx(100 * np.sqrt(3) / 2)
    assert snapshot.vertical_force_n == pytest.approx(100 * np.sqrt(3) / 2 + 10)


def test_native_world_resultant_is_exact_even_with_out_of_plane_contact_normal():
    raw_normal = np.array([0., 0.6, 0.8])
    frame = np.array([
        raw_normal,
        [1., 0., 0.],
        [0., 0.8, -0.6],
    ])
    wrench = np.array([100., 20., 5., 0., 0., 0.])
    transformed = _tracked_wrench(1, 2, frozenset({1}), frozenset({2}), frame, wrench)
    assert transformed is not None
    _, _, exact_force, _ = transformed
    patch = ContactPatch(
        np.zeros(3), np.array([0., 0., 1.]), 100., 20., 0.,
        native_world_force_n=exact_force,
    )
    snapshot = WheelContactSnapshot(0., (patch,), True)
    exact_force[:] = 999.
    np.testing.assert_array_equal(patch.world_force_n, [20., 64., 77.])
    np.testing.assert_array_equal(snapshot.world_force_n, [20., 64., 77.])
    assert snapshot.vertical_force_n == 77.
    assert snapshot.normal_vertical_n == 100.
    with pytest.raises(ValueError):
        patch.world_force_n.setflags(write=True)


def test_catch_plane_load_does_not_count_as_working_road_contact():
    catch_patch = ContactPatch(
        np.zeros(3), np.array([0., 0., 1.]), 200., 0., 0.,
        source_geom="catch_plane",
    )
    catch_only = WheelContactSnapshot(0., (catch_patch,), True)
    assert catch_only.loaded_contact
    assert not catch_only.road_loaded_contact
    assert catch_only.normal_load_n == 200.
    unloaded_road = ContactPatch(np.zeros(3), np.array([0., 0., 1.]), 0., 0., 0.)
    assert not WheelContactSnapshot(0., (catch_patch, unloaded_road), True).road_loaded_contact
    road_patch = ContactPatch(np.zeros(3), np.array([0., 0., 1.]), 50., 0., 0.)
    mixed = WheelContactSnapshot(0., (catch_patch, road_patch), True)
    assert mixed.road_loaded_contact
    assert road_patch.source_geom == "terrain"


def test_contact_patch_and_snapshot_detach_all_input_buffers():
    point = np.array([0., 0., -0.3])
    normal = np.array([0., 0., 1.])
    couple = np.array([0., 2., 0.])
    patch = ContactPatch(point, normal, 100., 20., -0.5, couple_world_nm=couple)
    patches = [patch]
    axis = np.zeros(3)
    snapshot = WheelContactSnapshot(
        time_s=0.25, patches=patches, geometric_contact=True,
        interval_id=500, backend="native_reference", wheel_axis_m=axis,
    )
    point[:] = 9.
    normal[:] = 9.
    couple[:] = 9.
    axis[:] = 9.
    patches.clear()
    assert snapshot.patches == (patch,)
    assert snapshot.loaded_contact
    assert snapshot.normal_load_n == pytest.approx(100.)
    assert snapshot.normal_vertical_n == pytest.approx(100.)
    assert snapshot.tangent_force_n == pytest.approx(20.)
    assert snapshot.vertical_force_n == pytest.approx(100.)
    assert snapshot.wheel_axis_m.tolist() == [0., 0., 0.]
    assert snapshot.effective_radius_m == pytest.approx(0.3)
    assert snapshot.wheel_axis_moment_nm == pytest.approx(-4.)
    assert snapshot.slip_mps == pytest.approx(-0.5)
    for vector in (patch.point_m, patch.normal, patch.couple_world_nm, snapshot.wheel_axis_m, snapshot.world_force_n):
        assert not vector.flags.writeable
        with pytest.raises(ValueError):
            vector.setflags(write=True)
    with pytest.raises(FrozenInstanceError):
        snapshot.interval_id = 501


def test_empty_geometric_snapshot_has_no_loaded_contact_or_force():
    snapshot = WheelContactSnapshot(0., (), True)
    assert snapshot.geometric_contact
    assert not snapshot.loaded_contact
    assert snapshot.normal_load_n == 0.
    assert snapshot.vertical_force_n == 0.
    assert snapshot.normal_vertical_n == 0.
    assert snapshot.tangent_force_n == 0.
    assert snapshot.effective_radius_m == 0.
    np.testing.assert_array_equal(snapshot.world_force_n, np.zeros(3))


@pytest.mark.parametrize(
    "kwargs",
    [
        {"normal_load_n": -1.},
        {"slip_mps": np.inf},
        {"normal": [0., 0.1, 0.9949874371]},
        {"normal": [0., 0., 2.]},
    ],
)
def test_patch_rejects_invalid_physical_values(kwargs):
    values = dict(point_m=np.zeros(3), normal=np.array([0., 0., 1.]), normal_load_n=1., tangent_force_n=0., slip_mps=0.)
    values.update(kwargs)
    with pytest.raises(ValueError):
        ContactPatch(**values)


@pytest.mark.parametrize("time_s", [-0.1, np.nan])
def test_snapshot_rejects_invalid_time(time_s):
    with pytest.raises(ValueError):
        WheelContactSnapshot(time_s, (), False)


def test_native_wrench_keeps_slope_tangent_vertical_and_contact_couple():
    # Contact frame rows: normal, first tangent, second tangent, all in world axes.
    n = np.array([-0.5, 0., np.sqrt(3) / 2])
    tangent = np.array([n[2], 0., -n[0]])
    frame = np.stack((n, tangent, np.array([0., 1., 0.])))
    wrench = np.array([100., 20., 0., 0., 0., 3.])
    result = _tracked_wrench(1, 2, frozenset({1}), frozenset({2}), frame, wrench)
    assert result is not None
    tracked, normal, force, couple = result
    assert tracked == 2
    np.testing.assert_allclose(normal, n)
    np.testing.assert_allclose(force, 100 * n + 20 * tangent)
    np.testing.assert_allclose(couple, [0., 3., 0.])


def test_swapped_geom_order_reverses_contact_frame_force_to_the_same_wheel_force():
    forward_frame = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
    reversed_frame = np.array([[0., 0., -1.], [1., 0., 0.], [0., -1., 0.]])
    forward = _tracked_wrench(1, 2, frozenset({1}), frozenset({2}), forward_frame, np.array([100., 20., 0., 0., 0., 3.]))
    swapped = _tracked_wrench(2, 1, frozenset({1}), frozenset({2}), reversed_frame, np.array([100., -20., 0., 0., 0., 3.]))
    assert forward is not None and swapped is not None
    assert forward[0] == swapped[0] == 2
    np.testing.assert_allclose(swapped[1], forward[1])
    np.testing.assert_allclose(swapped[2], forward[2])
    np.testing.assert_allclose(swapped[3], forward[3])


def test_zero_relative_hinge_speed_still_has_absolute_contact_point_velocity():
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode="ride", rider="none"))
    data = mujoco.MjData(model)
    pitch = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_pitch")
    spin = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "front_wheel_spin")
    data.qvel[model.jnt_dofadr[pitch]] = 1.5
    assert data.qvel[model.jnt_dofadr[spin]] == 0.
    mujoco.mj_forward(model, data)
    wheel = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "front_wheel")
    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "geom_front_contact")
    point = data.geom_xpos[geom] + np.array([0., 0., -0.35])
    actual = wheel_point_velocity(model, data, wheel, point)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jacp, jacr, point, wheel)
    np.testing.assert_allclose(actual, jacp @ data.qvel, atol=1e-10)
    assert np.linalg.norm(actual) > 0.1


def test_native_query_exposes_physical_snapshots_without_changing_legacy_channels():
    sim = RideSimulation(track=get_preset("single_edge"), target_speed_kmh=25.)
    model, data = sim.model, sim.data
    query = TerrainContactQuery(model)
    contacts = query.query(model, data)
    repeated = query.query(model, data)
    assert contacts.front_snapshot is not None
    assert contacts.rear_snapshot is not None
    assert contacts.front_snapshot.backend == "native_reference"
    assert contacts.front_snapshot.interval_id == repeated.front_snapshot.interval_id
    assert contacts.front_snapshot.normal_load_n >= 0.
    assert contacts.front_snapshot.loaded_contact or contacts.rear_snapshot.loaded_contact
    assert contacts.front_support_n == pytest.approx(max(0., contacts.front_snapshot.normal_vertical_n))
    front_loaded = contacts.front_snapshot.loaded_contact
    loaded = contacts.front_snapshot if front_loaded else contacts.rear_snapshot
    wheel_geom = query.front_id if front_loaded else query.rear_id
    assert loaded.geometric_contact
    expected_force = np.zeros(3)
    force6 = np.zeros(6)
    for index in range(data.ncon):
        contact = data.contact[index]
        pair = {int(contact.geom1), int(contact.geom2)}
        if wheel_geom not in pair or not (pair & query.terrain_ids):
            continue
        mujoco.mj_contactForce(model, data, index, force6)
        sign = 1. if int(contact.geom2) == wheel_geom else -1.
        expected_force += sign * (contact.frame.reshape(3, 3).T @ force6[:3])
    np.testing.assert_allclose(loaded.world_force_n, expected_force, rtol=0., atol=1e-12)
    old_point = loaded.patches[0].point_m.copy()
    pitch = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_pitch")
    spin_name = "front_wheel_spin" if front_loaded else "rear_wheel_spin"
    spin = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, spin_name)
    data.qvel[model.jnt_dofadr[pitch]] = 1.5
    data.qvel[model.jnt_dofadr[spin]] = 0.
    mujoco.mj_forward(model, data)
    moving = query.query(model, data)
    moving_snapshot = moving.front_snapshot if front_loaded else moving.rear_snapshot
    assert moving_snapshot.patches
    assert any(abs(patch.slip_mps) > 0.1 for patch in moving_snapshot.patches)
    for patch in moving_snapshot.patches:
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        body_id = int(model.geom_bodyid[wheel_geom])
        mujoco.mj_jac(model, data, jacp, jacr, patch.point_m, body_id)
        assert patch.slip_mps == pytest.approx(float((jacp @ data.qvel) @ patch.tangent))
    np.testing.assert_array_equal(loaded.patches[0].point_m, old_point)


def test_native_query_marks_catch_plane_rows_without_changing_legacy_loads():
    sim = RideSimulation(track=get_preset("single_edge"), target_speed_kmh=25.)
    model, data = sim.model, sim.data
    query = TerrainContactQuery(model)
    before = query.query(model, data)
    assert before.front_snapshot.road_loaded_contact or before.rear_snapshot.road_loaded_contact
    catch_plane = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "catch_plane")
    for index in range(data.ncon):
        contact = data.contact[index]
        if int(contact.geom1) in query.terrain_ids:
            contact.geom1 = catch_plane
        elif int(contact.geom2) in query.terrain_ids:
            contact.geom2 = catch_plane
    after = query.query(model, data)
    for old, new in (
        (before.front_snapshot, after.front_snapshot),
        (before.rear_snapshot, after.rear_snapshot),
    ):
        assert old.normal_load_n == pytest.approx(new.normal_load_n)
        assert new.loaded_contact == old.loaded_contact
        assert not new.road_loaded_contact
        assert all(patch.source_geom == "catch_plane" for patch in new.patches)
    assert after.front_load_n == pytest.approx(before.front_load_n)
    assert after.rear_load_n == pytest.approx(before.rear_load_n)
    assert after.front_support_n == pytest.approx(before.front_support_n)
    assert after.rear_support_n == pytest.approx(before.rear_support_n)
