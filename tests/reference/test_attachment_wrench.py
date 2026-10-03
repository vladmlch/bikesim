import mujoco
import numpy as np
import pytest
from bike_sim.sim.ride.attachment_wrench import (
    equality_qfrc, recover_wrench, relative_planar_jacobian)


def test_wrench_preserves_virtual_work():
    jac = np.array([[1., 0., 0., -1.], [0., 1., 0., 0.],
                    [0., 0., 1., 0.]])
    expected = np.array([20., 100., -2.])
    qforce = jac.T @ expected
    actual = recover_wrench(jac, qforce)
    velocity = np.array([.2, -.1, .3, .4])
    np.testing.assert_allclose(actual, expected, atol=1e-10)
    assert np.isclose(actual @ (jac @ velocity), qforce @ velocity)


def test_rank_deficiency_is_not_silently_reported_as_zero_force():
    with pytest.raises(ValueError, match='rank'):
        recover_wrench(np.zeros((2, 4)), np.zeros(4))


def test_unexplainable_generalized_force_is_not_a_wrench():
    jac = np.array([[1., 0., 0., -1.], [0., 1., 0., 0.]])
    with pytest.raises(ValueError, match='explain'):
        recover_wrench(jac, np.array([1., 0., 0., 1.]))


def _pair_model(equality_xml):
    """Small body 'rider' equalized to a much heavier 'bike' body.

    Both free bodies share one world anchor at (0, 0, 1); the rider's COM
    sits at the anchor so the recovered constraint wrench is directly the
    reaction balancing an applied Cartesian wrench.
    """
    return mujoco.MjModel.from_xml_string(f'''
<mujoco><option gravity="0 0 0" timestep="0.0005"/>
<worldbody>
  <body name="bike" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1e3" diaginertia="1e3 1e3 1e3"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="rider" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
</worldbody>
<equality>{equality_xml}</equality></mujoco>''')


@pytest.mark.parametrize('pad_spacing',[.01,.04])
def test_foot_cop_budget_comes_from_compiled_platform_not_sampling_pads(pad_spacing):
    from types import SimpleNamespace
    from bike_sim.physics.model_config import ArticulatedConfig
    from bike_sim.sim.ride.rider_contacts import RiderContactApplier
    model=mujoco.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0"/>
      <worldbody><body name="bike"><freejoint/><inertial mass="1000" pos="0 0 0" diaginertia="1000 1000 1000"/>
      <geom name="platform" type="box" size=".05 .025 .01" contype="0" conaffinity="0"/></body>
      <body name="rider"><freejoint/><inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/>
      <site name="sole"/></body></worldbody><equality><weld body1="rider" body2="bike"/></equality></mujoco>''')
    data=mujoco.MjData(model)
    data.xfrc_applied[model.body('rider').id,2]=-100.
    mujoco.mj_forward(model,data)
    c=RiderContactApplier.__new__(RiderContactApplier)
    c.config=ArticulatedConfig(pedal_patch_half_length_m=pad_spacing)
    c.supports={'front_pedal':(model.body('rider').id,model.site('sole').id,
        model.body('bike').id,model.geom('platform').id)}
    c.linked_saddle=False;c.welded_pedals=True;c.welded_grip=False
    c._welds=SimpleNamespace(eq_ids={'front':0})
    c._pads=lambda *args:[(None,None,np.array([0.,0.,1.]),None,None,None)]
    samples,errors=c._attachment_samples(model,data)
    assert not errors
    assert samples['foot_front'].half_patch_m == .05


def _recover(model, data, eq_name, rotational):
    eq_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, eq_name)
    assert eq_id >= 0
    anchor = np.array([0., 0., 1.])
    jac = relative_planar_jacobian(
        model, data, model.body('rider').id, model.body('bike').id,
        anchor, rotational=rotational)
    return recover_wrench(jac, equality_qfrc(model, data, eq_id)), jac, eq_id


def _lambda(model, data, eq_id):
    n = data.nefc
    sel = ((data.efc_type[:n] == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY))
           & (data.efc_id[:n] == eq_id))
    return data.efc_force[:n][sel]


@pytest.mark.parametrize('force', ([30., -80.], [-45., 120.]))
def test_weld_recovers_solved_wrench_and_work(force):
    model = _pair_model('<weld name="w" body1="rider" body2="bike"/>')
    data = mujoco.MjData(model)
    rider = model.body('rider').id
    torque = 12.5
    mujoco.mj_forward(model, data)
    data.xfrc_applied[rider] = [force[0], 0., force[1], 0., torque, 0.]
    for _ in range(100):
        mujoco.mj_step(model, data)
    wrench, jac, eq_id = _recover(model, data, 'w', rotational=True)
    lam = _lambda(model, data, eq_id)
    # Translational rows are unscaled: recovered force equals the multipliers.
    np.testing.assert_allclose(wrench[:2], lam[[0, 2]], rtol=1e-8, atol=1e-8)
    # The weld's sin(t/2) residual halves the physical moment in lambda_m
    # (exact at zero misalignment; the solref spring lets the pose drift).
    assert np.isclose(wrench[2], .5*lam[4], rtol=1e-3, atol=1e-3)
    # Physical oracle: after settling the weld balances the applied wrench.
    np.testing.assert_allclose(wrench, [-force[0], -force[1], -torque],
                               rtol=.05, atol=2.)
    qfrc_eq = equality_qfrc(model, data, eq_id)
    np.testing.assert_allclose(jac.T @ wrench, qfrc_eq, rtol=1e-8, atol=1e-8)
    assert np.isclose(wrench @ (jac @ data.qvel), qfrc_eq @ data.qvel)


@pytest.mark.parametrize('force', ([60., 40.], [-25., -95.]))
def test_connect_recovers_force_only(force):
    model = _pair_model('<connect name="c" body1="rider" body2="bike" anchor="0 0 0"/>')
    data = mujoco.MjData(model)
    rider = model.body('rider').id
    mujoco.mj_forward(model, data)
    data.xfrc_applied[rider] = [force[0], 0., force[1], 0., 30., 0.]
    for _ in range(100):
        mujoco.mj_step(model, data)
    wrench, jac, eq_id = _recover(model, data, 'c', rotational=False)
    lam = _lambda(model, data, eq_id)
    np.testing.assert_allclose(wrench, lam[[0, 2]], rtol=1e-8, atol=1e-8)
    # A pin answers force only; the applied couple spins the rider freely.
    np.testing.assert_allclose(wrench, [-force[0], -force[1]],
                               rtol=.05, atol=2.)
    assert wrench.shape == (2,)
    assert abs(data.qvel[10]) > 0.  # rider pitch dof of the free joint spun up


def test_contact_rows_do_not_corrupt_equality_selection():
    """A second support (native contact) shifts the efc arena layout; the
    recovered equality wrench must be unaffected."""
    xml = '''
<mujoco><option gravity="0 0 0" timestep="0.0005"/>
<worldbody>
  <geom name="floor" type="plane" size="5 5 1" pos="0 0 0"/>
  <body name="bike" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1e3" diaginertia="1e3 1e3 1e3"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="rider" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="ball" pos="0.4 0 0.09"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.1"/></body>
</worldbody>
<equality><weld name="w" body1="rider" body2="bike"/></equality></mujoco>'''
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    rider = model.body('rider').id
    mujoco.mj_forward(model, data)
    data.xfrc_applied[rider] = [55., 0., 70., 0., 0., 0.]
    for _ in range(100):
        mujoco.mj_step(model, data)
    n = data.nefc
    assert np.any(data.efc_type[:n] != int(mujoco.mjtConstraint.mjCNSTR_EQUALITY))
    wrench, jac, eq_id = _recover(model, data, 'w', rotational=True)
    lam = _lambda(model, data, eq_id)
    np.testing.assert_allclose(wrench[:2], lam[[0, 2]], rtol=1e-8, atol=1e-8)
    np.testing.assert_allclose(wrench[:2], [-55., -70.], rtol=.05, atol=2.)
