"""Native connect rows define foot reactions and the crank torque sensor."""
import mujoco
import numpy as np
import pytest

from test_pinned_topology import _compiled, _model
from bike_sim.sim.ride.physical_runtime import _connect_equality_rows
from bike_sim.sim.ride.rider_contacts import RiderContactApplier
from bike_sim.sim.ride.weld_pedals import equality_rows


@pytest.mark.parametrize('gap_m', [0., .001])
def test_spindle_force_sign_and_torque_match_native_body_jacobians(tmp_path, gap_m):
    from bike_sim.sim.ride.weld_pedals import SpindlePins
    model, controller = _compiled(tmp_path)
    data = mujoco.MjData(model)
    data.qpos[model.joint('rider_root_x').qposadr[0]] += gap_m
    mujoco.mj_forward(model, data)
    reader = SpindlePins(model)
    rows = equality_rows(data)
    data.efc_force[:] = 0.
    expected = np.zeros(model.nv)
    for side in ('front', 'rear'):
        eq = reader.eq_ids[side]
        body = int(model.eq_obj1id[eq])
        bike = int(model.eq_obj2id[eq])
        point = data.xpos[body] + data.xmat[body].reshape(3, 3) @ model.eq_data[eq, :3]
        bike_point = data.xpos[bike] + data.xmat[bike].reshape(3, 3) @ model.eq_data[eq, 3:6]
        # At qpos0 the ankle lies above the spindle: compression is +local z.
        force = np.array([20., 0., 100. if side == 'front' else 70.])
        data.efc_force[rows[eq]] = force
        mujoco.mj_applyFT(model, data, force, np.zeros(3), point, body, expected)
        mujoco.mj_applyFT(model, data, -force, np.zeros(3), bike_point, bike, expected)
        assert np.allclose(reader.force_on_rider_n(model, data, side), force)
    multipliers = np.zeros(data.nefc)
    for eq in reader.eq_ids.values():
        multipliers[rows[eq]] = data.efc_force[rows[eq]]
    native = np.zeros(model.nv)
    mujoco.mj_mulJacTVec(model, data, native, multipliers)
    assert np.allclose(native, expected, atol=1e-10)
    assert abs(expected[reader.crank_dof]) > 1.
    assert reader.delivered_crank_torque_nm(model, data) == pytest.approx(expected[reader.crank_dof])
    contacts = RiderContactApplier(model, controller.pose, controller.config)
    contacts.reset(model, data)
    samples, errors = contacts.attachment_samples(model, data)
    assert not errors
    for side in ('front', 'rear'):
        sample = samples[f'foot_{side}']
        assert sample.normal_n == pytest.approx(100. if side == 'front' else 70.)
        assert sample.tangent_n == pytest.approx(20.)
        assert sample.moment_nm == 0.
        assert sample.half_patch_m == 0.
        assert sample.gap_m == pytest.approx(gap_m)
    prepared = contacts.prepare_attachment_raw(model, data)
    contacts.settle_welds(model, data, raw=True, prepared=prepared)
    assert set(contacts.last_attachment_samples) == {'foot_front', 'foot_rear', 'saddle', 'grip_left', 'grip_right'}
    assert not contacts.last_attachment_errors


def test_linkage_closure_rows_exclude_rider_connects(tmp_path):
    model, _ = _compiled(tmp_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    mask = _connect_equality_rows(model, data)
    names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, int(eq)) for eq in data.efc_id[:data.nefc][mask]}
    assert names
    assert not any(name.startswith(('connect_saddle', 'connect_foot_', 'connect_grip_')) for name in names)


@pytest.mark.slow
def test_pinned_samples_are_compressive_at_rest(tmp_path):
    from bike_sim.sim.ride.control import RideControl
    _, env = _model(tmp_path)
    runtime = env.sim.physical
    for _ in range(80):
        runtime.step(front=1., rear=1.,
                     control=RideControl(motor_torque_nm=0., human_torque_nm=0.))
    samples = runtime.sample.channels['attachment_samples']
    for side in ('front', 'rear'):
        assert samples[f'foot_{side}']['normal_n'] >= 20.
        assert samples[f'foot_{side}']['moment_nm'] == 0.
    assert samples['saddle']['normal_n'] > 0.
    assert np.isfinite(runtime.rider_contacts.delivered_crank_torque_nm)
    assert runtime.sample.channels['suspension']['linkage_closure_max_m'] < .002
    assert not any('unobservable' in v for v in runtime.sample.channels['attachment_violations'])
