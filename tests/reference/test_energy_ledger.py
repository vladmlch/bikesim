import mujoco
import numpy as np
import pytest
from bike_sim.physics.energy_ledger import step_work, constraint_work_ok
from bike_sim.sim.research.quality import energy_quality


def test_positive_body_work_is_not_net_joint_work():
    work = step_work(np.array([400., -350.]), 100., np.array([20., -20.]), .1)
    assert work.muscle_positive_j == 40.
    assert work.muscle_signed_j == 5.
    assert work.constraint_absolute_j == 4.
    assert work.constraint_signed_j == 0.
    assert not constraint_work_ok(4., 50.)


def test_large_constraint_work_cannot_improve_residual_quality():
    base = dict(energy_scale_j=1., active_work_j=100.,
                source_positive_work_j=100., external_work_j=0., residual_j=10.)
    a = energy_quality(dict(base, solver_constraint_work_j=0.), maximum_ratio=.05)
    b = energy_quality(dict(base, solver_constraint_work_j=100000.), maximum_ratio=.05)
    assert a.residual_ratio == b.residual_ratio
    assert not a.acceptable and not b.acceptable


def test_motor_work_reports_signed_and_positive_separately():
    work = step_work(np.array([50.]), -80., np.zeros(0), .01)
    assert work.motor_signed_j == -.8
    assert work.motor_positive_j == 0.
    work = step_work(np.array([-10.]), 250., np.zeros(0), .01)
    assert work.motor_signed_j == 2.5
    assert work.motor_positive_j == 2.5
    assert work.muscle_signed_j == -.1


def test_invalid_work_inputs_are_rejected():
    for muscle, motor, constraint, dt in (
            (np.array([np.nan]), 0., np.zeros(0), .01),
            (np.zeros(1), np.inf, np.zeros(0), .01),
            (np.zeros(1), 0., np.zeros(0), 0.),
            (np.zeros(1), 0., np.zeros(0), -.1)):
        with pytest.raises(ValueError):
            step_work(muscle, motor, constraint, dt)


def test_constraint_budget_uses_roundoff_floor_when_no_source():
    assert constraint_work_ok(1e-9, 0.)
    assert not constraint_work_ok(1e-7, 0.)
    assert constraint_work_ok(.009, .9)
    assert not constraint_work_ok(.02, .9)
    for absolute, source in ((-.1, 1.), (1., -.1), (float('nan'), 1.)):
        with pytest.raises(ValueError):
            constraint_work_ok(absolute, source)


@pytest.mark.parametrize('source,absolute,allowed', [
    (1., .0099, True), (1., .0101, False),
    (0., .99e-8, True), (0., 1.01e-8, False)])
def test_runtime_constraint_budget_checks_closing_tail_with_fixed_roundoff(source, absolute, allowed):
    from test_monitor_strictness import runtime_for_raws, neutral_raw
    from bike_sim.sim.ride.reference_monitor import InvalidReferenceRun
    runtime = runtime_for_raws()
    # Two opposite-signed closure intervals have zero signed work but the
    # required absolute work. A partial period must pass the same integral gate.
    for i, sign in enumerate((1., -1.)):
        runtime._buffer.push(neutral_raw(i, qvel=[1.], components={
            'act_rider_probe':np.array([source/.001]),
            'closure':np.array([sign*absolute/.001])}))
    if allowed:
        runtime.flush()
        assert runtime.reference_monitor.first_failure is None
    else:
        with pytest.raises(InvalidReferenceRun, match='energy.constraint_work'):
            runtime.flush()
        assert runtime.reference_monitor.first_failure == (.001, ('energy.constraint_work',))
        assert runtime.completed_samples[0].channels['attachment_violations'] == ()
    assert runtime.energy['constraint_work_ok'] is allowed
    assert runtime.energy['constraint_absolute_j'] == pytest.approx(absolute)
    assert runtime.energy['constraint_signed_j'] == pytest.approx(0.)
    assert runtime.energy['source_positive_work_j'] == pytest.approx(source)
    assert runtime.history.duration_s == pytest.approx(.001)


def test_runtime_zero_source_tolerance_does_not_grow_with_episode_steps():
    from test_monitor_strictness import runtime_for_raws, neutral_raw
    from bike_sim.sim.ride.reference_monitor import InvalidReferenceRun
    runtime = runtime_for_raws()
    runtime.sim.steps = 100000
    runtime._buffer.push(neutral_raw(0, qvel=[1.], components={'closure':np.array([1.e-4])}))
    with pytest.raises(InvalidReferenceRun, match='energy.constraint_work'):
        runtime.flush()
    assert runtime.energy['constraint_absolute_j'] == pytest.approx(5.e-8)


@pytest.mark.parametrize('kind',['tendon','joint','contact','equality'])
def test_actual_solver_rows_partition_without_missing_or_duplicate_forces(kind):
    from bike_sim.sim.ride.physical_observations import constraint_components
    if kind == 'contact':
        xml='<worldbody><geom type="plane" size="1 1 .1"/><body pos="0 0 .09"><joint type="slide" axis="0 0 1"/><geom type="sphere" size=".1" mass="1"/></body></worldbody>'
    else:
        joint='limited="true" range="-.1 .1"' if kind=='joint' else ''
        xml=f'<worldbody><body><joint name="a" type="slide" axis="1 0 0" {joint}/><inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/></body><body><joint name="b" type="slide" axis="1 0 0"/><inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/></body></worldbody>'
        if kind=='tendon':
            xml+='<tendon><fixed limited="true" range="-100 0"><joint joint="a" coef="1"/><joint joint="b" coef="-1"/></fixed></tendon>'
        elif kind=='equality':
            xml+='<equality><joint joint1="a" joint2="b"/></equality>'
    model=mujoco.MjModel.from_xml_string('<mujoco><option gravity="0 0 0"/>'+xml+'</mujoco>')
    data=mujoco.MjData(model)
    if kind!='contact':data.qpos[0]=.2
    mujoco.mj_forward(model,data)
    parts=constraint_components(model,data)
    target={'tendon':'joint_limits','joint':'joint_limits','contact':'native_contact','equality':'closure'}[kind]
    assert np.linalg.norm(data.qfrc_constraint)>1.
    np.testing.assert_allclose(parts[target],data.qfrc_constraint,rtol=1e-12,atol=1e-12)
    np.testing.assert_allclose(sum(parts.values()),data.qfrc_constraint,rtol=1e-12,atol=1e-12)
    assert all(not np.any(force) for name,force in parts.items() if name!=target)


def test_cached_equilibrium_restores_the_actual_one_way_constraint_preload():
    from types import SimpleNamespace
    from bike_sim.sim.ride.ideal_freehub import IdealFreehubConstraint
    from bike_sim.sim.ride.equilibrium_cache import _state_arrays,restore_state
    model=mujoco.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0"/>
      <worldbody><body><joint name="a" type="slide" axis="1 0 0"/>
      <inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/></body>
      <body><joint name="b" type="slide" axis="1 0 0"/>
      <inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/></body></worldbody>
      <tendon><fixed name="coupling" limited="true" range="-10 0"><joint joint="a" coef="1"/>
      <joint joint="b" coef="-1"/></fixed></tendon></mujoco>''')
    data=mujoco.MjData(model)
    hub=IdealFreehubConstraint(model,1.,tendon_name='coupling',driver='a',driven='b')
    hub.boundary=-.001;model.tendon_range[hub.tendon_id,1]=hub.boundary
    mujoco.mj_forward(model,data)
    before=data.qacc.copy()
    runtime=SimpleNamespace(sim=SimpleNamespace(model=model),rider_contacts=None,tire=None,
                            drive=SimpleNamespace(ideal_hub=hub,clutch=None))
    state=_state_arrays(runtime)
    hub.reset(model,data)
    mujoco.mj_forward(model,data)
    assert not np.allclose(data.qacc,before)
    assert restore_state(runtime,state)
    mujoco.mj_forward(model,data)
    np.testing.assert_allclose(data.qacc,before,atol=1e-12)
    assert hub.boundary == model.tendon_range[hub.tendon_id,1] == -.001


@pytest.mark.parametrize('kind',['equality','tendon'])
def test_opposite_actual_closure_work_cannot_cancel_inside_one_interval(kind):
    from dataclasses import replace
    from test_monitor_strictness import runtime_for_raws,neutral_raw
    from bike_sim.sim.ride.physical_observations import constraint_components,numerical_constraint_powers
    from bike_sim.sim.ride.reference_monitor import InvalidReferenceRun
    model=mujoco.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0" timestep=".0005"/>
      <worldbody><body><joint name="a" type="slide" axis="1 0 0"/><geom size=".1" mass="1" contype="0" conaffinity="0"/></body>
      <body><joint name="b" type="slide" axis="1 0 0"/><geom size=".1" mass="1" contype="0" conaffinity="0"/></body></worldbody>
      <equality><joint name="one" joint1="a" solref="-100 0"/><joint name="two" joint1="b" solref="-100 0"/></equality></mujoco>''')
    if kind=='tendon':
        bodies=''.join(f'<body><joint name="{name}" type="slide" axis="1 0 0"/>'
            '<geom size=".1" mass="1" contype="0" conaffinity="0"/></body>' for name in 'abcd')
        tendons=''.join(f'<fixed limited="true" range="-10 0" solreflimit="-100 0"><joint joint="{a}" coef="1"/>'
            f'<joint joint="{b}" coef="-1"/></fixed>' for a,b in (('a','b'),('c','d')))
        model=mujoco.MjModel.from_xml_string('<mujoco><option gravity="0 0 0" timestep=".0005"/>'
            '<worldbody>'+bodies+'</worldbody><tendon>'+tendons+'</tendon></mujoco>')
    data=mujoco.MjData(model)
    data.qpos[:]=[.1,-.1] if kind=='equality' else [.1,0.,.1,0.]
    data.qvel[:]=[1.,1.] if kind=='equality' else [1.,0.,-1.,0.]
    mujoco.mj_forward(model,data)
    parts=constraint_components(model,data)
    assert parts['closure' if kind=='equality' else 'joint_limits']@data.qvel == pytest.approx(0.,abs=1e-12)
    powers=numerical_constraint_powers(model,data,data.qvel)
    assert len(powers)==2 and min(powers.values())<0.<max(powers.values())
    runtime=runtime_for_raws()
    raw=neutral_raw(0)
    runtime._buffer.push(replace(raw,qpos=data.qpos.copy(),qvel=data.qvel.copy(),components=parts,
        metadata=dict(raw.metadata,numerical_constraint_power_w=powers)))
    with pytest.raises(InvalidReferenceRun,match='energy.constraint_work'):
        runtime.flush()
    assert runtime.energy['constraint_signed_j'] == pytest.approx(0.,abs=1e-12)
    assert runtime.energy['constraint_absolute_j'] == pytest.approx(sum(map(abs,powers.values()))*.0005)
    assert runtime.energy['constraint_absolute_j'] > 1e-8


def _free_body(extra=''):
    return mujoco.MjModel.from_xml_string(
        '<mujoco><option gravity="0 0 -9.81" timestep="0.0005"/>' +
        extra + '</mujoco>')


def _energy(model, data):
    mv = np.zeros(model.nv)
    mujoco.mj_mulM(model, data, mv, data.qvel)
    kinetic = .5*float(data.qvel @ mv)
    mass = np.asarray(model.body_mass)
    potential = -float(np.sum(mass[:, None]*data.xipos*model.opt.gravity))
    return kinetic + potential


def _row_work(model, data, v_mid, keep):
    """Work of the selected efc rows over the interval: qfrc . v_mid . dt."""
    n = data.nefc
    multipliers = np.zeros(n)
    multipliers[keep(data.efc_type[:n])] = data.efc_force[:n][keep(data.efc_type[:n])]
    qfrc = np.zeros(model.nv)
    mujoco.mj_mulJacTVec(model, data, qfrc, multipliers)
    return float(qfrc @ v_mid*model.opt.timestep)


def _equality_work(model, data, v_mid):
    equality = int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)
    return _row_work(model, data, v_mid, lambda types: types == equality)


def _contact_work(model, data, v_mid):
    contact = int(mujoco.mjtConstraint.mjCNSTR_CONTACT_FRICTIONLESS)
    frictional = int(mujoco.mjtConstraint.mjCNSTR_CONTACT_ELLIPTIC)
    pyramidal = int(mujoco.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL)
    return _row_work(model, data, v_mid,
                     lambda types: np.isin(types, (contact, frictional, pyramidal)))


def test_free_fall_keeps_energy_without_constraint_work():
    model = _free_body('<worldbody><body pos="0 0 2"><freejoint/>'
                       '<inertial pos="0 0 0" mass="5" diaginertia="1 1 1"/>'
                       '<geom size=".1" contype="0" conaffinity="0"/></body></worldbody>')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    e0 = _energy(model, data)
    for _ in range(200):
        mujoco.mj_step(model, data)
        assert data.nefc == 0
    np.testing.assert_allclose(_energy(model, data), e0, rtol=1e-3)


def test_closed_ideal_constraint_does_no_work():
    """An already-merged weld pair keeps its energy: the ideal constraint
    is not silently generating or absorbing work."""
    model = _free_body('''<worldbody>
<body name="a" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="5" diaginertia="1 1 1"/>
  <geom size=".05" contype="0" conaffinity="0"/></body>
<body name="b" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="5" diaginertia="1 1 1"/>
  <geom size=".05" contype="0" conaffinity="0"/></body></worldbody>
<equality><weld body1="a" body2="b" solref="0.005 1"/></equality>''')
    model.opt.gravity[:] = 0.
    data = mujoco.MjData(model)
    data.qvel[:6] = [.2, 0., .1, 0., .3, 0.]
    data.qvel[6:12] = [.2, 0., .1, 0., .3, 0.]
    mujoco.mj_forward(model, data)
    e0 = _energy(model, data)
    absolute = 0.
    for _ in range(200):
        v_mid = data.qvel.copy()
        mujoco.mj_step(model, data)
        v_mid += data.qvel
        absolute += abs(_equality_work(model, data, .5*v_mid))
    np.testing.assert_allclose(_energy(model, data), e0, rtol=1e-6)
    assert absolute < 1e-6*e0


def test_inelastic_impact_loss_is_physical_not_constraint_credit():
    """Two equal inertias merging through the weld dissipate 1/4 m (dv)^2.

    The equality rows carry the whole engagement loss as negative work --
    that is the physical clutch dissipation, not a numerical defect, so the
    ledger may book it as loss without inventing a compensating source term.
    """
    model = _free_body('''<worldbody>
<body name="a" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="5" diaginertia="1 1 1"/>
  <geom size=".05" contype="0" conaffinity="0"/></body>
<body name="b" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="5" diaginertia="1 1 1"/>
  <geom size=".05" contype="0" conaffinity="0"/></body></worldbody>
<equality><weld body1="a" body2="b" solref="0.005 1"/></equality>''')
    model.opt.gravity[:] = 0.
    data = mujoco.MjData(model)
    dv = .8
    data.qvel[0], data.qvel[6] = .5, .5-dv
    mujoco.mj_forward(model, data)
    mass = 5.
    analytic_loss = .25*mass*dv*dv
    equality_work = 0.
    for _ in range(2000):
        v_mid = data.qvel.copy()
        mujoco.mj_step(model, data)
        v_mid += data.qvel
        equality_work += _equality_work(model, data, .5*v_mid)
    e1 = _energy(model, data)
    np.testing.assert_allclose(data.qvel[0], data.qvel[6], atol=1e-3)
    np.testing.assert_allclose(-equality_work, analytic_loss, rtol=.05)
    # Total energy falls by exactly the engagement loss; nothing else moves it.
    np.testing.assert_allclose(e1, .5*mass*.5*.5+.5*mass*(.5-dv)**2-analytic_loss,
                               rtol=.02)


def test_friction_loss_is_nonnegative_and_consistent():
    model = _free_body('''<worldbody>
<geom name="floor" type="plane" size="5 5 1" pos="0 0 0" friction="0.5 0.5 0.5"/>
<body name="block" pos="0 0 0.1"><joint type="slide" axis="1 0 0"/>
  <joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="2" diaginertia="1 1 1"/>
  <geom type="box" size=".1 .1 .1" friction="0.5 0.5 0.5"/></body></worldbody>''')
    data = mujoco.MjData(model)
    data.qvel[:] = [2., 0.]
    mujoco.mj_forward(model, data)
    mass, v0 = 2., 2.
    kinetic0 = .5*mass*v0*v0
    contact_work = 0.
    for _ in range(400):
        v_mid = data.qvel.copy()
        mujoco.mj_step(model, data)
        v_mid += data.qvel
        contact_work += _contact_work(model, data, .5*v_mid)
    kinetic1 = .5*mass*float(data.qvel[0]**2)
    loss = kinetic0-kinetic1
    assert loss > 0.
    np.testing.assert_allclose(-contact_work, loss, rtol=.1)
    # Kinetic energy only ever decreased: the model cannot create energy.
    assert kinetic1 < kinetic0
